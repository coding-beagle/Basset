//! Replays the timeline into a [`ModelState`].
//!
//! Regeneration is incremental: the state after every active feature is snapshotted, and
//! an edit to feature *i* discards snapshots from *i* onward. Because `ModelState` shares
//! its heavy members through `Arc`, a snapshot costs a few map clones, not geometry.
//!
//! Failures are per-feature. A feature whose inputs are missing (e.g. its sketch was
//! deleted or its profile disappeared after a dimension change) is recorded as
//! `Failed` and replay continues, so the user sees exactly which steps broke and the
//! rest of the model stays live.

use std::borrow::Cow;
use std::sync::Arc;

use basset_kernel::{self as kernel, Axis, BoolOp, OpId, Profile, Solid};
use basset_math::{Frame, Vec2, Vec3};
use basset_sketch::{Entity, Font, Tessellation};

use crate::feature::{BodyOp, CombineOp, Extent, Feature, FeatureKind};
use crate::ids::{ComponentId, FeatureId};
use crate::model::{Body, Component, FeatureStatus, ModelState, SolvedSketch};
use crate::parameters::Parameters;
use crate::refs::{AxisRef, BodyRef, EdgeRef, PathRef, PlaneRef, ProfileRef, RegionRef};
use crate::timeline::Timeline;

#[derive(Debug, thiserror::Error)]
pub enum RegenError {
    #[error("plane reference {0} does not exist")]
    MissingPlane(FeatureId),
    #[error("body {0} does not exist")]
    MissingBody(FeatureId),
    #[error("sketch {0} does not exist")]
    MissingSketch(FeatureId),
    #[error("component {0} does not exist")]
    MissingComponent(ComponentId),
    #[error("face {0:?} is not planar or no longer exists")]
    NotAPlanarFace(String),
    #[error("no target bodies: pick at least one body to apply the operation to")]
    NoTargets,
    #[error("no closed profile around ({0}, {1}) in sketch {2}")]
    NoProfileAt(f64, f64, FeatureId),
    #[error("face {0:?} cannot be used as a profile: {1}")]
    BadFaceRegion(String, kernel::KernelError),
    #[error("axis line is not a line entity")]
    BadAxis,
    #[error("edge {0:?} no longer exists on body")]
    MissingEdge(String),
    #[error("target face {0:?} no longer exists on its body")]
    MissingTargetFace(String),
    #[error("sketch did not solve: {0}")]
    Sketch(String),
    #[error("{0}")]
    Kernel(#[from] kernel::KernelError),
    #[error("{0}")]
    SketchOp(#[from] basset_sketch::SketchError),
}

#[derive(Clone, Debug, Default)]
pub struct Regenerator {
    /// `snapshots[i]` is the state after replaying features `0..=i`.
    snapshots: Vec<ModelState>,
    font: Option<Arc<Font>>,
    /// The document's parameter table, behind every sketch and in front of every driven
    /// feature value. Held here rather than read from the timeline because it is not part
    /// of the timeline: changing it re-drives everything, from feature zero.
    parameters: Parameters,
    tessellation: Tessellation,
    kernel_tessellation: kernel::Tessellation,
}

impl Regenerator {
    pub fn set_font(&mut self, font: Option<Arc<Font>>) {
        self.font = font;
    }

    /// Replaces the parameter table and discards every cached state: a parameter can drive
    /// anything at any point in the history, so there is no earlier feature to keep.
    pub fn set_parameters(&mut self, parameters: Parameters) {
        self.parameters = parameters;
        self.invalidate_from(0);
    }

    pub fn parameters(&self) -> &Parameters {
        &self.parameters
    }

    pub fn invalidate_from(&mut self, index: usize) {
        self.snapshots.truncate(index);
    }

    pub fn evaluate(&mut self, timeline: &Timeline) -> &ModelState {
        let count = timeline.active().len();
        self.evaluate_prefix(timeline, count)
    }

    /// The state after the first `count` active features, replaying only what is not
    /// cached. Since every full evaluation snapshots each step, asking for a prefix of
    /// the current cursor never replays anything: it is how a tool sees the model as it
    /// was before the feature it is previewing.
    pub fn evaluate_prefix(&mut self, timeline: &Timeline, count: usize) -> &ModelState {
        let active = timeline.active();
        let count = count.min(active.len());
        self.snapshots.truncate(active.len());
        while self.snapshots.len() < count {
            let index = self.snapshots.len();
            let base = match index {
                0 => ModelState::with_root(),
                _ => self.snapshots[index - 1].clone(),
            };
            let next = self.apply(base, &active[index]);
            self.snapshots.push(next);
        }
        // Nothing before the first feature: hand out the shared empty state.
        match count {
            0 => &EMPTY_STATE,
            n => {
                flag_sketch_faults(&mut self.snapshots[n - 1], &active[..n], &self.parameters);
                &self.snapshots[n - 1]
            }
        }
    }

    fn apply(&self, mut state: ModelState, feature: &Feature) -> ModelState {
        if feature.suppressed {
            state.statuses.insert(feature.id, FeatureStatus::Suppressed);
            return state;
        }
        let (driven, stale) = self.drive(feature);
        let status = match self.apply_kind(&mut state, &driven) {
            Ok(()) if stale.is_empty() => FeatureStatus::Ok,
            Ok(()) => FeatureStatus::Warned(stale.join("; ")),
            Err(e) => {
                log::warn!("feature {} ({}) failed: {e}", feature.id, feature.name);
                FeatureStatus::Failed(e.to_string())
            }
        };
        state.statuses.insert(feature.id, status);
        state
    }

    /// Resolves a feature's expression-driven values into a copy of it, and reports the
    /// ones that could not be resolved.
    ///
    /// A copy, because replay must not write back into the timeline: the stored expression
    /// is the input and the number is derived from it, so a replay that edited the feature
    /// would make regeneration a mutation and undo a lie. The common case — a feature
    /// nobody drives — borrows and clones nothing.
    ///
    /// An expression that no longer evaluates does *not* fail the feature. It keeps the
    /// number it last had and says so, following the same rule the sketch layer applies to
    /// a dimension: deleting a parameter should tell the user what stopped being driven,
    /// not collapse half the model to zero while they work out what happened.
    fn drive<'f>(&self, feature: &'f Feature) -> (Cow<'f, Feature>, Vec<String>) {
        if feature.exprs.is_empty() {
            return (Cow::Borrowed(feature), Vec::new());
        }
        let mut driven = feature.clone();
        let mut stale = Vec::new();
        for (field, text) in &feature.exprs {
            match self.parameters.evaluate(text) {
                Ok(value) if driven.kind.set_numeric_field(*field, value) => {}
                Ok(_) => stale.push(format!(
                    "{field} is driven by {text:?}, but this feature no longer has that value"
                )),
                Err(e) => stale.push(format!("{field} keeps its value: {e}")),
            }
        }
        (Cow::Owned(driven), stale)
    }

    fn apply_kind(&self, state: &mut ModelState, feature: &Feature) -> Result<(), RegenError> {
        let id = feature.id;
        match &feature.kind {
            FeatureKind::NewComponent { name, parent } => {
                if !state.components.contains_key(parent) {
                    return Err(RegenError::MissingComponent(*parent));
                }
                let cid = ComponentId::from_feature(id);
                state.components.insert(
                    cid,
                    Component {
                        id: cid,
                        name: name.clone(),
                        parent: Some(*parent),
                    },
                );
            }
            FeatureKind::Sketch {
                plane,
                component,
                sketch,
            } => {
                let frame = resolve_plane(state, plane)?;
                if !state.components.contains_key(component) {
                    return Err(RegenError::MissingComponent(*component));
                }
                let mut solved = sketch.clone();
                solved.set_font(self.font.clone());
                // The document's table sits behind the sketch's own, so a dimension bound
                // to a document parameter is re-driven here, on every replay.
                let outer = self.parameters.lookup();
                let report = solved
                    .solve_with(&outer)
                    .map_err(|e| RegenError::Sketch(e.to_string()))?;
                let profiles = solved
                    .profiles(&self.tessellation)
                    .into_iter()
                    .map(|p| convert_profile(&frame, &p))
                    .collect();
                state.sketches.insert(
                    id,
                    Arc::new(SolvedSketch {
                        frame,
                        component: *component,
                        sketch: solved,
                        profiles,
                        report,
                    }),
                );
            }
            FeatureKind::OffsetPlane { base, distance } => {
                let frame = resolve_plane(state, base)?.offset(*distance);
                state.planes.insert(id, frame);
            }
            FeatureKind::AngledPlane { base, axis, angle } => {
                let frame = resolve_plane(state, base)?;
                let axis = resolve_axis(state, axis)?;
                state
                    .planes
                    .insert(id, frame.rotated_about(axis.origin, axis.direction, *angle));
            }
            FeatureKind::Extrude {
                regions,
                extent,
                operation,
                component,
            } => {
                let solid = union_all(regions.iter().enumerate().map(|(k, p)| {
                    let profile = resolve_region(state, p)?;
                    match extent {
                        // The target is resolved fresh on every replay, so the
                        // extrusion follows the face through edits of the body it
                        // belongs to, and a missing body or face is this feature's
                        // failure rather than a stale distance.
                        Extent::ToFace(target) => {
                            let body = state
                                .bodies
                                .get(&target.body)
                                .ok_or(RegenError::MissingBody(target.body.0))?;
                            let face = body.solid.face(target.key).ok_or_else(|| {
                                RegenError::MissingTargetFace(format!("{:?}", target.key))
                            })?;
                            Ok(kernel::extrude_to_face(op_id(id, k), &profile, face)?)
                        }
                        _ => Ok(kernel::extrude(
                            op_id(id, k),
                            &profile,
                            convert_extent(*extent),
                        )?),
                    }
                }))?;
                self.finish_body(state, id, *component, solid, operation)?;
            }
            FeatureKind::Revolve {
                regions,
                axis,
                angle,
                operation,
                component,
            } => {
                let axis = resolve_axis(state, axis)?;
                let solid = union_all(regions.iter().enumerate().map(|(k, p)| {
                    let profile = resolve_region(state, p)?;
                    Ok(kernel::revolve(
                        op_id(id, k),
                        &profile,
                        &axis,
                        *angle,
                        &self.kernel_tessellation,
                    )?)
                }))?;
                self.finish_body(state, id, *component, solid, operation)?;
            }
            FeatureKind::Sweep {
                regions,
                path,
                operation,
                component,
            } => {
                let path = resolve_path(state, path, &self.tessellation)?;
                let solid = union_all(regions.iter().enumerate().map(|(k, p)| {
                    let profile = resolve_region(state, p)?;
                    Ok(kernel::sweep(op_id(id, k), &profile, &path)?)
                }))?;
                self.finish_body(state, id, *component, solid, operation)?;
            }
            FeatureKind::Loft {
                regions,
                operation,
                component,
            } => {
                let sections = regions
                    .iter()
                    .map(|p| resolve_region(state, p))
                    .collect::<Result<Vec<_>, _>>()?;
                let solid = kernel::loft(op_id(id, 0), &sections)?;
                self.finish_body(state, id, *component, solid, operation)?;
            }
            FeatureKind::Fillet { edges, radius } => {
                for (body_ref, keys) in group_edges(edges) {
                    let body = state
                        .bodies
                        .get_mut(&body_ref)
                        .ok_or(RegenError::MissingBody(body_ref.0))?;
                    check_edges_exist(&body.solid, &keys)?;
                    let solid = kernel::fillet(
                        op_id(id, 0),
                        &body.solid,
                        &keys,
                        *radius,
                        &self.kernel_tessellation,
                    )?;
                    body.solid = Arc::new(solid);
                }
            }
            FeatureKind::Chamfer { edges, distance } => {
                for (body_ref, keys) in group_edges(edges) {
                    let body = state
                        .bodies
                        .get_mut(&body_ref)
                        .ok_or(RegenError::MissingBody(body_ref.0))?;
                    check_edges_exist(&body.solid, &keys)?;
                    let solid = kernel::chamfer(op_id(id, 0), &body.solid, &keys, *distance)?;
                    body.solid = Arc::new(solid);
                }
            }
            FeatureKind::Combine {
                target,
                tools,
                operation,
                keep_tools,
            } => {
                let op = match operation {
                    CombineOp::Join => BoolOp::Union,
                    CombineOp::Cut => BoolOp::Subtract,
                    CombineOp::Intersect => BoolOp::Intersect,
                };
                let mut result = (*state
                    .bodies
                    .get(target)
                    .ok_or(RegenError::MissingBody(target.0))?
                    .solid)
                    .clone();
                for tool in tools {
                    let tool_body = state
                        .bodies
                        .get(tool)
                        .ok_or(RegenError::MissingBody(tool.0))?;
                    result = kernel::boolean(&result, &tool_body.solid, op)?;
                }
                if !keep_tools {
                    for tool in tools {
                        state.bodies.remove(tool);
                    }
                }
                if let Some(body) = state.bodies.get_mut(target) {
                    body.solid = Arc::new(result);
                }
            }
            FeatureKind::Move { body, transform } => {
                let b = state
                    .bodies
                    .get_mut(body)
                    .ok_or(RegenError::MissingBody(body.0))?;
                b.solid = Arc::new(b.solid.transformed(transform));
            }
        }
        Ok(())
    }

    fn finish_body(
        &self,
        state: &mut ModelState,
        id: FeatureId,
        component: ComponentId,
        solid: Solid,
        operation: &BodyOp,
    ) -> Result<(), RegenError> {
        if !state.components.contains_key(&component) {
            return Err(RegenError::MissingComponent(component));
        }
        match operation {
            BodyOp::NewBody => {
                let body_ref = BodyRef(id);
                state.bodies.insert(
                    body_ref,
                    Body {
                        id: body_ref,
                        name: format!("Body{}", id.0),
                        component,
                        solid: Arc::new(solid),
                    },
                );
            }
            BodyOp::Join(targets) | BodyOp::Cut(targets) | BodyOp::Intersect(targets) => {
                let op = match operation {
                    BodyOp::Join(_) => BoolOp::Union,
                    BodyOp::Cut(_) => BoolOp::Subtract,
                    _ => BoolOp::Intersect,
                };
                if targets.is_empty() {
                    return Err(RegenError::NoTargets);
                }
                // The same tool solid is applied to every listed body, as Fusion does:
                // a body the tool never reaches is a no-op for a cut, a second disjoint
                // shell for a join — the kernel keeps both shells in one solid — and an
                // empty result for an intersect, which the kernel refuses and this
                // feature reports. Each boolean reads and writes only its own body, so
                // the shared operation id cannot collide: face and edge keys are looked
                // up per body, never across them.
                for t in targets {
                    let body = state
                        .bodies
                        .get_mut(t)
                        .ok_or(RegenError::MissingBody(t.0))?;
                    body.solid = Arc::new(kernel::boolean(&body.solid, &solid, op)?);
                }
            }
        }
        Ok(())
    }
}

static EMPTY_STATE: std::sync::LazyLock<ModelState> =
    std::sync::LazyLock::new(ModelState::with_root);

/// Flags the sketches whose state the user needs to know about: loose geometry a later
/// feature builds from, and dimensions whose expression stopped evaluating.
///
/// An under-constrained sketch on its own is an ordinary state of a drawing — Fusion
/// colours it and says nothing more, and a warning on every such sketch would be
/// permanent furniture that nobody reads. It becomes a fault the moment a *later* feature
/// builds from it: an edit anywhere in the timeline is then free to slide the loose
/// geometry off the edges it was drawn against, which changes what the profiles enclose
/// and so what that feature builds, silently.
///
/// A dimension bound to an expression that no longer evaluates — usually because the
/// parameter behind it was deleted or renamed by hand — keeps the value it last had, so
/// the sketch still solves and nothing here can go wrong on its own. It is worth saying
/// anyway: the drawing is no longer the thing the user expressed, and nothing else in the
/// model would ever mention it.
///
/// Both are computed over the finished state rather than while replaying, because whether
/// a sketch is consumed depends on features that come after it, and both are recomputed
/// from scratch each time so that deleting the consumer, or restoring the parameter, takes
/// the warning away again. A sketch with both faults gets one message naming both, since
/// a status holds one string and neither fault is the more urgent.
fn flag_sketch_faults(state: &mut ModelState, active: &[Feature], parameters: &Parameters) {
    let consumed: Vec<FeatureId> = active
        .iter()
        // A suppressed feature builds nothing, so it puts nothing at risk either.
        .filter(|f| !f.suppressed)
        .flat_map(|f| f.kind.dependencies())
        .filter(|id| state.sketches.contains_key(id))
        .collect();
    let outer = parameters.lookup();
    for feature in active {
        let Some(solved) = state.sketches.get(&feature.id) else {
            continue;
        };
        // A failed or suppressed feature has more urgent news, so only `Ok` is
        // overwritten; `Warned` is recomputed so the flag can be taken away as well.
        if !matches!(
            state.statuses.get(&feature.id),
            Some(FeatureStatus::Ok | FeatureStatus::Warned(_))
        ) {
            continue;
        }
        let mut faults = Vec::new();
        let dof = solved.report.degrees_of_freedom;
        if dof > 0 && consumed.contains(&feature.id) {
            let plural = if dof == 1 { "" } else { "s" };
            faults.push(format!(
                "under-constrained: {dof} degree{plural} of freedom, and a feature builds \
                 from it \u{2014} an edit elsewhere can move this geometry"
            ));
        }
        let stale = solved.sketch.failed_bindings_with(&outer).len();
        if stale > 0 {
            let plural = if stale == 1 { "" } else { "s" };
            faults.push(format!(
                "{stale} dimension{plural} stopped being driven: the expression no longer \
                 evaluates, so the value it last had is being used"
            ));
        }
        let status = match faults.is_empty() {
            true => FeatureStatus::Ok,
            false => FeatureStatus::Warned(faults.join("; ")),
        };
        state.statuses.insert(feature.id, status);
    }
}

/// Kernel operation id for the `k`-th solid a feature produces. Separate sub-ids keep the
/// caps of two profiles extruded by one feature from sharing a face key.
fn op_id(id: FeatureId, k: usize) -> OpId {
    OpId::new(id.0).with_sub(k as u32)
}

fn union_all(solids: impl Iterator<Item = Result<Solid, RegenError>>) -> Result<Solid, RegenError> {
    let mut acc: Option<Solid> = None;
    for s in solids {
        let s = s?;
        acc = Some(match acc {
            None => s,
            Some(prev) => kernel::boolean(&prev, &s, BoolOp::Union)?,
        });
    }
    acc.ok_or(RegenError::Kernel(kernel::KernelError::EmptyProfile))
}

fn group_edges(edges: &[EdgeRef]) -> Vec<(BodyRef, Vec<kernel::EdgeKey>)> {
    let mut groups: Vec<(BodyRef, Vec<kernel::EdgeKey>)> = Vec::new();
    for e in edges {
        match groups.iter_mut().find(|(b, _)| *b == e.body) {
            Some((_, keys)) => keys.push(e.key),
            None => groups.push((e.body, vec![e.key])),
        }
    }
    groups
}

fn check_edges_exist(solid: &Solid, keys: &[kernel::EdgeKey]) -> Result<(), RegenError> {
    let existing = solid.edges();
    for k in keys {
        if !existing.iter().any(|e| e.key == *k) {
            return Err(RegenError::MissingEdge(format!("{k:?}")));
        }
    }
    Ok(())
}

pub(crate) fn resolve_plane(state: &ModelState, plane: &PlaneRef) -> Result<Frame, RegenError> {
    match plane {
        PlaneRef::Origin(p) => Ok(match p {
            crate::refs::OriginPlane::XY => Frame::XY,
            crate::refs::OriginPlane::YZ => Frame::YZ,
            crate::refs::OriginPlane::XZ => Frame::XZ,
        }),
        PlaneRef::Feature(id) => state
            .planes
            .get(id)
            .copied()
            .ok_or(RegenError::MissingPlane(*id)),
        PlaneRef::Face(face) => {
            let body = state
                .bodies
                .get(&face.body)
                .ok_or(RegenError::MissingBody(face.body.0))?;
            let f = body
                .solid
                .face(face.key)
                .ok_or_else(|| RegenError::NotAPlanarFace(format!("{:?}", face.key)))?;
            face_frame(f).ok_or_else(|| RegenError::NotAPlanarFace(format!("{:?}", face.key)))
        }
    }
}

/// The sketch frame a planar face provides: anchored at the face centroid so sketch
/// coordinates are intuitive and independent of polygon fragmentation after booleans.
/// `None` for curved faces. The application uses this too, so what the user sees when
/// picking a face is exactly what a sketch on it will use.
pub fn face_frame(face: &kernel::Face) -> Option<Frame> {
    face.frame()
}

fn resolve_axis(state: &ModelState, axis: &AxisRef) -> Result<Axis, RegenError> {
    match axis {
        AxisRef::Origin(a) => Ok(Axis {
            origin: Vec3::ZERO,
            direction: match a {
                crate::refs::OriginAxis::X => Vec3::X,
                crate::refs::OriginAxis::Y => Vec3::Y,
                crate::refs::OriginAxis::Z => Vec3::Z,
            },
        }),
        AxisRef::SketchLine { sketch, line } => {
            let solved = state
                .sketches
                .get(sketch)
                .ok_or(RegenError::MissingSketch(*sketch))?;
            let (a, b) = match solved.sketch.entity(*line).map(|e| &e.entity) {
                Some(Entity::Line { start, end }) => (
                    solved.sketch.point_pos(*start).ok_or(RegenError::BadAxis)?,
                    solved.sketch.point_pos(*end).ok_or(RegenError::BadAxis)?,
                ),
                _ => return Err(RegenError::BadAxis),
            };
            let origin = solved.frame.to_world(a);
            let direction = solved.frame.to_world(b) - origin;
            if direction.length_squared() < basset_math::LINEAR_TOL * basset_math::LINEAR_TOL {
                return Err(RegenError::BadAxis);
            }
            Ok(Axis {
                origin,
                direction: direction.normalize(),
            })
        }
    }
}

/// A region as the kernel wants it, whichever kind of thing the user picked.
fn resolve_region(state: &ModelState, region: &RegionRef) -> Result<Profile, RegenError> {
    match region {
        RegionRef::Profile(p) => resolve_profile(state, p),
        RegionRef::Face(f) => {
            let body = state
                .bodies
                .get(&f.body)
                .ok_or(RegenError::MissingBody(f.body.0))?;
            body.solid
                .face_profile(f.key)
                .map_err(|e| RegenError::BadFaceRegion(format!("{:?}", f.key), e))
        }
    }
}

fn resolve_profile(state: &ModelState, p: &ProfileRef) -> Result<Profile, RegenError> {
    let solved = state
        .sketches
        .get(&p.sketch)
        .ok_or(RegenError::MissingSketch(p.sketch))?;
    solved
        .profiles
        .iter()
        .filter(|profile| profile_contains(profile, p.sample))
        // Nested regions: the smallest region containing the point is the one the user
        // clicked in.
        .min_by(|a, b| profile_area(a).total_cmp(&profile_area(b)))
        .cloned()
        .ok_or(RegenError::NoProfileAt(p.sample.x, p.sample.y, p.sketch))
}

fn resolve_path(
    state: &ModelState,
    path: &PathRef,
    tess: &Tessellation,
) -> Result<kernel::Path3, RegenError> {
    let solved = state
        .sketches
        .get(&path.sketch)
        .ok_or(RegenError::MissingSketch(path.sketch))?;
    let contour = solved.sketch.path(&path.curves, tess)?;
    Ok(kernel::Path3 {
        points: contour
            .points
            .iter()
            .map(|p| solved.frame.to_world(*p))
            .collect(),
    })
}

/// The kernel form of an extent whose distances are stored on the feature. `ToFace` has
/// no such form — its reach exists only once the target is resolved, which the extrude
/// arm of [`Regenerator::apply_kind`] does before ever calling this.
fn convert_extent(e: Extent) -> kernel::Extent {
    match e {
        Extent::OneSide(d) => kernel::Extent::OneSide(d),
        Extent::Symmetric(d) => kernel::Extent::Symmetric(d),
        Extent::TwoSides { positive, negative } => kernel::Extent::TwoSides { positive, negative },
        Extent::ToFace(_) => unreachable!("a to-face extent is resolved during replay"),
    }
}

/// Sketch contours carry `EntityId` curve tags; the kernel wants a small stable integer.
/// The slot index of the entity is stable for the entity's whole life, which is exactly
/// the property face keys need.
fn curve_tag(id: basset_sketch::EntityId) -> u32 {
    use slotmap::Key;
    id.data().as_ffi() as u32
}

fn convert_contour(c: &basset_sketch::Contour) -> kernel::Contour {
    kernel::Contour {
        points: c.points.clone(),
        segments: c
            .segments
            .iter()
            .map(|s| kernel::Segment {
                curve: curve_tag(s.curve),
                kind: match s.kind {
                    basset_sketch::SegmentKind::Line => kernel::SegmentKind::Line,
                    basset_sketch::SegmentKind::Arc {
                        center,
                        radius,
                        ccw,
                    } => kernel::SegmentKind::Arc {
                        center,
                        radius,
                        ccw,
                    },
                },
            })
            .collect(),
        closed: c.closed,
    }
}

pub fn convert_profile(frame: &Frame, p: &basset_sketch::Profile) -> Profile {
    Profile {
        frame: *frame,
        outer: convert_contour(&p.outer),
        holes: p.holes.iter().map(convert_contour).collect(),
    }
}

fn profile_contains(p: &Profile, point: Vec2) -> bool {
    contour_contains(&p.outer, point) && !p.holes.iter().any(|h| contour_contains(h, point))
}

fn contour_contains(c: &kernel::Contour, p: Vec2) -> bool {
    // Even-odd ray cast; robust enough for sample points chosen in region interiors.
    let n = c.points.len();
    let mut inside = false;
    for i in 0..n {
        let a = c.points[i];
        let b = c.points[(i + 1) % n];
        if (a.y > p.y) != (b.y > p.y) {
            let x = a.x + (p.y - a.y) / (b.y - a.y) * (b.x - a.x);
            if p.x < x {
                inside = !inside;
            }
        }
    }
    inside
}

fn profile_area(p: &Profile) -> f64 {
    let area = |c: &kernel::Contour| {
        let n = c.points.len();
        (0..n)
            .map(|i| {
                let a = c.points[i];
                let b = c.points[(i + 1) % n];
                a.x * b.y - b.x * a.y
            })
            .sum::<f64>()
            .abs()
            * 0.5
    };
    area(&p.outer) - p.holes.iter().map(area).sum::<f64>()
}
