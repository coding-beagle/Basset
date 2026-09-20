//! Corner fillets: the tangent arc that rounds off where two curves meet.
//!
//! The arc is found by offsetting both curves by the radius and intersecting the
//! offsets: every point that distance from both curves is a possible centre, and the
//! tangent points are its feet on each curve. That one construction covers line–line,
//! line–arc and arc–arc without a case each, which is why it is preferred here over the
//! usual half-angle formula that only knows about straight corners.
//!
//! Four offsets make up to four candidates — one per corner of the crossing — so the
//! caller says which side of each curve it picked and the candidate whose tangent points
//! lie on those sides is the one the user pointed at. A pick is a point on the curve
//! *away* from the corner, which is exactly what a click gives.
//!
//! The curves are then trimmed back to their tangent points by moving the endpoint that
//! was at the corner, so constraints and dimensions written against them survive, and
//! the arc shares those endpoints as entities, which is how the sketch joins geometry.
//! Two [`Constraint::Tangent`] constraints hold the result together afterwards: without
//! them the first drag or re-solve would leave a rounded corner that no longer met its
//! own edges.

use std::f64::consts::{PI, TAU};

use basset_math::Vec2;

use crate::intersect::{CurveGeom, circle_circle, line_circle, line_line};
use crate::sketch::JOIN_TOL;
use crate::tessellation::ccw_sweep;
use crate::{Constraint, Entity, EntityId, Sketch, SketchError};

/// Where a fillet would go, in sketch coordinates. Computed before anything is edited,
/// so a radius that does not fit is refused with the sketch untouched — and so the
/// editor can draw a handle on a fillet it has already made.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Plan {
    /// Where the two curves meet, or would meet if extended.
    pub corner: Vec2,
    pub center: Vec2,
    pub tangent_a: Vec2,
    pub tangent_b: Vec2,
    pub radius: f64,
}

impl Plan {
    /// Unit direction from the corner towards the arc's centre: the bisector of the
    /// rounded corner, and the line a radius handle belongs on.
    pub fn bisector(&self) -> Vec2 {
        (self.center - self.corner)
            .try_normalize()
            // Only reachable if the centre landed on the corner, which needs a zero
            // radius; `plan` has already refused that.
            .unwrap_or(Vec2::Y)
    }
}

/// What [`fillet`] made.
#[derive(Clone, Copy, Debug)]
pub struct Fillet {
    pub arc: EntityId,
    /// The points the fillet added: the arc's centre and its two ends, which are also
    /// the new endpoints of the two curves.
    pub center: EntityId,
    pub start: EntityId,
    pub end: EntityId,
    pub plan: Plan,
}

/// The infinite carrier of a curve — the line a segment lies on, the circle an arc lies
/// on. Offsetting and intersecting happen here, because a fillet may reach past the
/// drawn ends of what it rounds.
#[derive(Clone, Copy, Debug)]
enum Support {
    Line { origin: Vec2, dir: Vec2 },
    Circle { center: Vec2, radius: f64 },
}

impl Support {
    fn of(geom: &CurveGeom) -> Result<Self, SketchError> {
        match *geom {
            CurveGeom::Line { a, b } => {
                let dir = (b - a).try_normalize().ok_or_else(|| {
                    SketchError::DegenerateGeometry("a zero-length line has no direction".into())
                })?;
                Ok(Support::Line { origin: a, dir })
            }
            CurveGeom::Arc { center, radius, .. } => Ok(Support::Circle { center, radius }),
        }
    }

    /// The same carrier moved `signed` millimetres to one side. `None` when a circle
    /// would be turned inside out, which is the arc-radius-smaller-than-the-fillet case.
    fn offset(self, signed: f64) -> Option<Self> {
        match self {
            Support::Line { origin, dir } => Some(Support::Line {
                origin: origin + dir.perp() * signed,
                dir,
            }),
            Support::Circle { center, radius } => {
                let radius = radius + signed;
                (radius > JOIN_TOL).then_some(Support::Circle { center, radius })
            }
        }
    }

    fn intersect(self, other: Self) -> Vec<Vec2> {
        match (self, other) {
            (
                Support::Line {
                    origin: o0,
                    dir: d0,
                },
                Support::Line {
                    origin: o1,
                    dir: d1,
                },
            ) => line_line(o0, o0 + d0, o1, o1 + d1).into_iter().collect(),
            (Support::Line { origin, dir }, Support::Circle { center, radius })
            | (Support::Circle { center, radius }, Support::Line { origin, dir }) => {
                line_circle(origin, origin + dir, center, radius)
            }
            (
                Support::Circle {
                    center: c0,
                    radius: r0,
                },
                Support::Circle {
                    center: c1,
                    radius: r1,
                },
            ) => circle_circle(c0, r0, c1, r1),
        }
    }

    /// The point of the carrier nearest `p`: the foot of the perpendicular on a line,
    /// the radial projection on a circle.
    fn closest(self, p: Vec2) -> Vec2 {
        match self {
            Support::Line { origin, dir } => origin + dir * dir.dot(p - origin),
            Support::Circle { center, radius } => match (p - center).try_normalize() {
                Some(radial) => center + radial * radius,
                None => center + Vec2::X * radius,
            },
        }
    }

    /// Signed distance from `corner` to `p` *along* the carrier, in millimetres. The
    /// sign is what says which side of the corner a point is on, and comparing the sign
    /// of a tangent point with the sign of the user's pick is what chooses between the
    /// four candidate arcs.
    fn along(self, corner: Vec2, p: Vec2) -> f64 {
        match self {
            Support::Line { dir, .. } => dir.dot(p - corner),
            Support::Circle { center, radius } => {
                let turn = (p - center).to_angle() - (corner - center).to_angle();
                // Wrapped into ±half a turn: the near way round the circle is the way
                // a corner is measured, whichever direction the arc happens to run.
                radius * ((turn + PI).rem_euclid(TAU) - PI)
            }
        }
    }
}

/// Where a fillet of `radius` between `a` and `b` would go.
///
/// `hint_a` and `hint_b` are points on the side of each curve the user picked — a click
/// position, or a point partway along the curve when a corner was picked instead. They
/// decide which of the four possible arcs is meant.
pub fn plan(
    sketch: &Sketch,
    a: EntityId,
    hint_a: Vec2,
    b: EntityId,
    hint_b: Vec2,
    radius: f64,
) -> Result<Plan, SketchError> {
    if a == b {
        return Err(SketchError::InvalidArgument(
            "a fillet needs two different curves".into(),
        ));
    }
    if !radius.is_finite() || radius <= JOIN_TOL {
        return Err(SketchError::InvalidArgument(format!(
            "fillet radius must be positive, got {radius}"
        )));
    }
    let (geom_a, geom_b) = (open_curve(sketch, a)?, open_curve(sketch, b)?);
    let (sup_a, sup_b) = (Support::of(&geom_a)?, Support::of(&geom_b)?);
    let corner = corner_of(sketch, a, b, sup_a, sup_b, hint_a, hint_b).ok_or_else(|| {
        SketchError::InvalidArgument(
            "the curves do not meet, so there is no corner to round".into(),
        )
    })?;
    let reach_a = reach(sketch, a, sup_a, corner, hint_a);
    let reach_b = reach(sketch, b, sup_b, corner, hint_b);
    let mut best: Option<(f64, Plan)> = None;
    for side_a in [radius, -radius] {
        for side_b in [radius, -radius] {
            let (Some(off_a), Some(off_b)) = (sup_a.offset(side_a), sup_b.offset(side_b)) else {
                continue;
            };
            for center in off_a.intersect(off_b) {
                let (ta, tb) = (sup_a.closest(center), sup_b.closest(center));
                // The candidate came from the offsets, so it is the radius from both
                // carriers by construction; this rejects the rounding-error cases the
                // near-tangent intersections throw up.
                if (center.distance(ta) - radius).abs() > JOIN_TOL
                    || (center.distance(tb) - radius).abs() > JOIN_TOL
                {
                    continue;
                }
                let (along_a, along_b) = (sup_a.along(corner, ta), sup_b.along(corner, tb));
                if !on_picked_side(along_a, sup_a.along(corner, hint_a))
                    || !on_picked_side(along_b, sup_b.along(corner, hint_b))
                {
                    continue;
                }
                // A fillet that would run off the far end of a curve has eaten the whole
                // of it; refusing is better than silently stretching the drawing.
                if along_a.abs() > reach_a + JOIN_TOL || along_b.abs() > reach_b + JOIN_TOL {
                    continue;
                }
                // Among the survivors the tightest into the corner is the fillet; the
                // others sit further out along the same pair of curves.
                let score = along_a.abs() + along_b.abs();
                if best.as_ref().is_none_or(|(s, _)| score < *s) {
                    best = Some((
                        score,
                        Plan {
                            corner,
                            center,
                            tangent_a: ta,
                            tangent_b: tb,
                            radius,
                        },
                    ));
                }
            }
        }
    }
    best.map(|(_, plan)| plan).ok_or_else(|| {
        SketchError::InvalidArgument(format!(
            "a radius of {radius:.3} mm does not fit this corner"
        ))
    })
}

/// Rounds the corner between `a` and `b`, trimming both back to the tangent arc.
///
/// The sketch is left untouched when the fillet is refused: everything that can fail is
/// decided by [`plan`] before the first edit.
pub fn fillet(
    sketch: &mut Sketch,
    a: EntityId,
    hint_a: Vec2,
    b: EntityId,
    hint_b: Vec2,
    radius: f64,
) -> Result<Fillet, SketchError> {
    let plan = plan(sketch, a, hint_a, b, hint_b, radius)?;
    let near_a = near_end(sketch, a, plan.corner)?;
    let near_b = near_end(sketch, b, plan.corner)?;
    // The fillet is reference geometry only when both the curves it joins are: a
    // rounded corner between a real edge and a construction line is part of the profile.
    let construction = sketch.entity(a).is_some_and(|d| d.construction)
        && sketch.entity(b).is_some_and(|d| d.construction);

    let pa = sketch.add_point(plan.tangent_a);
    let pb = sketch.add_point(plan.tangent_b);
    let center = sketch.add_point(plan.center);
    replace_end(sketch, a, near_a, pa);
    replace_end(sketch, b, near_b, pb);
    // The corner point is what the two curves used to share; with both moved onto their
    // tangent points nothing is left holding it, unless a dimension was written to it —
    // in which case it stays, and the dimension keeps meaning what it meant.
    crate::edit::prune_orphans(sketch, &[near_a, near_b]);

    // An arc runs counter-clockwise, so which tangent point is the start is decided by
    // which order sweeps the short way: a fillet is the small arc between the curves,
    // never the rest of the circle.
    let angle_of = |p: Vec2| (p - plan.center).to_angle();
    let (start, end) = if ccw_sweep(angle_of(plan.tangent_a), angle_of(plan.tangent_b)) <= PI {
        (pa, pb)
    } else {
        (pb, pa)
    };
    let arc = sketch.add_arc(center, start, end)?;
    sketch.set_construction(arc, construction)?;
    for c in [Constraint::Tangent(arc, a), Constraint::Tangent(arc, b)] {
        // A tangency the sketch will not take (a kind that cannot carry one) is not
        // worth losing the fillet over: the geometry is right either way, it simply
        // stops being held that way under a later edit.
        if let Err(e) = sketch.add_constraint(c) {
            log::debug!("fillet dropped a tangent constraint: {e}");
        }
    }
    Ok(Fillet {
        arc,
        center,
        start,
        end,
        plan,
    })
}

/// The two curves meeting at `point`, for a fillet picked by its corner rather than by
/// its two edges. `None` unless exactly two open curves end there, since three curves at
/// a point name no single corner.
pub fn curves_at(sketch: &Sketch, point: EntityId) -> Option<(EntityId, EntityId)> {
    // A corner is made by the *ends* of curves: an arc whose centre happens to be this
    // point does not turn a corner here, and `Entity::references` would include it.
    let mut curves = Vec::new();
    for (id, data) in sketch.entities() {
        let ends = match data.entity {
            Entity::Line { start, end } | Entity::Arc { start, end, .. } => [start, end],
            _ => continue,
        };
        if ends.contains(&point) {
            curves.push(id);
        }
    }
    match curves[..] {
        [a, b] => Some((a, b)),
        _ => None,
    }
}

/// A point partway along `curve` from `at`, for use as a pick hint when the user named a
/// corner instead of clicking the curves themselves.
pub fn hint_along(sketch: &Sketch, curve: EntityId, at: Vec2) -> Option<Vec2> {
    let geom = CurveGeom::of(sketch, curve)?;
    let t = geom.param_of(at);
    // Away from the end the corner is at, far enough that a short curve still gives a
    // usable direction.
    Some(geom.point_at(if t < 0.5 { 0.75 } else { 0.25 }))
}

/// The geometry of a curve a fillet can round. Circles are refused: rounding into one
/// means cutting it open somewhere, which is the trim tool's job and a different
/// decision for the user to make.
fn open_curve(sketch: &Sketch, id: EntityId) -> Result<CurveGeom, SketchError> {
    let data = sketch.entity(id).ok_or(SketchError::UnknownEntity(id))?;
    if !data.entity.is_open_curve() {
        return Err(SketchError::WrongEntityKind {
            id,
            expected: "line or arc",
            actual: data.entity.kind_name(),
        });
    }
    CurveGeom::of(sketch, id).ok_or(SketchError::UnknownEntity(id))
}

/// Whether a tangent point sits on the side of the corner the user picked. A pick right
/// at the corner says nothing, so it accepts either side rather than refusing.
fn on_picked_side(tangent: f64, hint: f64) -> bool {
    hint.abs() <= JOIN_TOL || tangent.abs() <= JOIN_TOL || tangent.signum() == hint.signum()
}

/// How far the curve runs from the corner on the picked side, in millimetres. Zero when
/// the curve stops at the corner on that side, which refuses every candidate there.
fn reach(sketch: &Sketch, id: EntityId, sup: Support, corner: Vec2, hint: Vec2) -> f64 {
    let hint_side = sup.along(corner, hint);
    let Some((start, end)) = sketch.curve_endpoints(id) else {
        return 0.0;
    };
    [start, end]
        .into_iter()
        .map(|p| sup.along(corner, p))
        .filter(|along| hint_side.abs() <= JOIN_TOL || along.signum() == hint_side.signum())
        .fold(0.0f64, |acc, along| acc.max(along.abs()))
}

/// Where the two curves meet: the point they share if they share one, otherwise the
/// crossing of their carriers nearest what the user picked.
fn corner_of(
    sketch: &Sketch,
    a: EntityId,
    b: EntityId,
    sup_a: Support,
    sup_b: Support,
    hint_a: Vec2,
    hint_b: Vec2,
) -> Option<Vec2> {
    if let Some(shared) = shared_point(sketch, a, b) {
        return sketch.point_pos(shared);
    }
    let between = (hint_a + hint_b) * 0.5;
    sup_a
        .intersect(sup_b)
        .into_iter()
        .min_by(|p, q| p.distance(between).total_cmp(&q.distance(between)))
}

/// The endpoint entity the two curves have in common, if any.
fn shared_point(sketch: &Sketch, a: EntityId, b: EntityId) -> Option<EntityId> {
    let ends = |id: EntityId| match sketch.entity(id).map(|d| &d.entity) {
        Some(Entity::Line { start, end }) | Some(Entity::Arc { start, end, .. }) => {
            vec![*start, *end]
        }
        _ => Vec::new(),
    };
    ends(a).into_iter().find(|p| ends(b).contains(p))
}

/// Which end of the curve the corner is at: the one the fillet moves onto its tangent
/// point.
fn near_end(sketch: &Sketch, id: EntityId, corner: Vec2) -> Result<EntityId, SketchError> {
    let (start, end) = match sketch.entity(id).map(|d| &d.entity) {
        Some(Entity::Line { start, end }) | Some(Entity::Arc { start, end, .. }) => (*start, *end),
        Some(other) => {
            return Err(SketchError::WrongEntityKind {
                id,
                expected: "line or arc",
                actual: other.kind_name(),
            });
        }
        None => return Err(SketchError::UnknownEntity(id)),
    };
    let distance = |p: EntityId| {
        sketch
            .point_pos(p)
            .map_or(f64::INFINITY, |pos| pos.distance(corner))
    };
    Ok(if distance(start) <= distance(end) {
        start
    } else {
        end
    })
}

/// Points `curve`'s `from` end at `to`. The curve keeps its own entity, and with it
/// every constraint and dimension written against it.
fn replace_end(sketch: &mut Sketch, curve: EntityId, from: EntityId, to: EntityId) {
    if let Some(data) = sketch.entities.get_mut(curve) {
        match &mut data.entity {
            Entity::Line { start, end } | Entity::Arc { start, end, .. } => {
                if *start == from {
                    *start = to;
                }
                if *end == from {
                    *end = to;
                }
            }
            _ => {}
        }
    }
}
