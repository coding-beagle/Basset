//! The [`Sketch`] container: entity and constraint storage, editing, hit testing and
//! the entry points into the solver and profile extraction.

use std::sync::Arc;

use basset_math::{LINEAR_TOL, Vec2};
use serde::{Deserialize, Serialize};
use slotmap::{SecondaryMap, SlotMap};

use crate::contour::{Contour, Segment, SegmentKind};
use crate::geometry::{
    arc_bounds, point_arc_distance, point_circle_distance, point_in_rect, point_segment_distance,
    polyline_intersects_rect,
};
use crate::tessellation::{Tessellation, arc_polyline, circle_polyline};
use crate::text::Font;
use crate::{
    Constraint, ConstraintId, Entity, EntityData, EntityId, Profile, SketchError, SolveError,
    SolveReport,
};

/// Endpoints closer than this are treated as joined when building profiles and paths.
/// It is 1000× the kernel's [`LINEAR_TOL`] because solver output is only accurate to
/// its convergence tolerance, not to machine precision, and 0.1 µm is still far below
/// anything a user can draw deliberately.
pub const JOIN_TOL: f64 = LINEAR_TOL * 1e3;

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct Sketch {
    pub(crate) entities: SlotMap<EntityId, EntityData>,
    pub(crate) constraints: SlotMap<ConstraintId, Constraint>,
    /// Fonts are large binary blobs owned by the application, not by the document, so
    /// they are never serialised; the app re-attaches one after loading.
    #[serde(skip)]
    pub(crate) font: Option<Arc<Font>>,
    /// Label positions of dimensions the user has placed. Kept with the sketch, as
    /// Fusion does, so a drawing keeps its layout when the sketch is reopened.
    #[serde(default)]
    pub(crate) labels: SecondaryMap<ConstraintId, Vec2>,
    /// Named constants, in the order the user made them.
    #[serde(default)]
    pub(crate) parameters: Vec<crate::Parameter>,
    /// Dimensions driven by an expression instead of a typed number.
    #[serde(default)]
    pub(crate) dimension_exprs: SecondaryMap<ConstraintId, String>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Hit {
    pub entity: EntityId,
    pub distance: f64,
}

impl Sketch {
    pub fn new() -> Self {
        Self::default()
    }

    // ----- entities -------------------------------------------------------------------

    pub fn add_point(&mut self, pos: Vec2) -> EntityId {
        self.entities.insert(EntityData {
            entity: Entity::Point { pos },
            construction: false,
        })
    }

    pub fn add_line(&mut self, start: EntityId, end: EntityId) -> Result<EntityId, SketchError> {
        self.expect_point(start)?;
        self.expect_point(end)?;
        Ok(self.insert(Entity::Line { start, end }))
    }

    pub fn add_circle(&mut self, center: EntityId, radius: f64) -> Result<EntityId, SketchError> {
        self.expect_point(center)?;
        if !radius.is_finite() || radius <= 0.0 {
            return Err(SketchError::InvalidArgument(format!(
                "circle radius must be positive, got {radius}"
            )));
        }
        Ok(self.insert(Entity::Circle { center, radius }))
    }

    pub fn add_arc(
        &mut self,
        center: EntityId,
        start: EntityId,
        end: EntityId,
    ) -> Result<EntityId, SketchError> {
        self.expect_point(center)?;
        self.expect_point(start)?;
        self.expect_point(end)?;
        if start == center || end == center {
            return Err(SketchError::DegenerateGeometry(
                "arc endpoint cannot be its own centre".into(),
            ));
        }
        Ok(self.insert(Entity::Arc { center, start, end }))
    }

    pub fn add_text(
        &mut self,
        anchor: EntityId,
        text: impl Into<String>,
        height: f64,
        angle: f64,
    ) -> Result<EntityId, SketchError> {
        self.expect_point(anchor)?;
        if !height.is_finite() || height <= 0.0 {
            return Err(SketchError::InvalidArgument(format!(
                "text height must be positive, got {height}"
            )));
        }
        Ok(self.insert(Entity::Text {
            anchor,
            text: text.into(),
            height,
            angle,
        }))
    }

    fn insert(&mut self, entity: Entity) -> EntityId {
        self.entities.insert(EntityData {
            entity,
            construction: false,
        })
    }

    fn expect_point(&self, id: EntityId) -> Result<Vec2, SketchError> {
        match self.entities.get(id) {
            Some(EntityData {
                entity: Entity::Point { pos },
                ..
            }) => Ok(*pos),
            Some(d) => Err(SketchError::WrongEntityKind {
                id,
                expected: "point",
                actual: d.entity.kind_name(),
            }),
            None => Err(SketchError::UnknownEntity(id)),
        }
    }

    pub fn set_construction(
        &mut self,
        id: EntityId,
        construction: bool,
    ) -> Result<(), SketchError> {
        let d = self
            .entities
            .get_mut(id)
            .ok_or(SketchError::UnknownEntity(id))?;
        d.construction = construction;
        Ok(())
    }

    pub fn entity(&self, id: EntityId) -> Option<&EntityData> {
        self.entities.get(id)
    }

    pub fn entities(&self) -> impl Iterator<Item = (EntityId, &EntityData)> {
        self.entities.iter()
    }

    pub fn point_pos(&self, id: EntityId) -> Option<Vec2> {
        self.expect_point(id).ok()
    }

    pub fn set_point_pos(&mut self, id: EntityId, pos: Vec2) -> Result<(), SketchError> {
        match self.entities.get_mut(id) {
            Some(EntityData {
                entity: Entity::Point { pos: p },
                ..
            }) => {
                *p = pos;
                Ok(())
            }
            Some(d) => Err(SketchError::WrongEntityKind {
                id,
                expected: "point",
                actual: d.entity.kind_name(),
            }),
            None => Err(SketchError::UnknownEntity(id)),
        }
    }

    /// Removes an entity together with everything that depends on it: curves built on a
    /// removed point, and constraints mentioning any removed entity.
    pub fn remove_entity(&mut self, id: EntityId) {
        if !self.entities.contains_key(id) {
            return;
        }
        let mut doomed = vec![id];
        let mut i = 0;
        while i < doomed.len() {
            let victim = doomed[i];
            for (other, data) in &self.entities {
                if data.entity.references().contains(&victim) && !doomed.contains(&other) {
                    doomed.push(other);
                }
            }
            i += 1;
        }
        let dead_constraints: Vec<ConstraintId> = self
            .constraints
            .iter()
            .filter(|(_, c)| c.references().iter().any(|r| doomed.contains(r)))
            .map(|(cid, _)| cid)
            .collect();
        for cid in dead_constraints {
            self.constraints.remove(cid);
            self.labels.remove(cid);
        }
        for e in doomed {
            self.entities.remove(e);
        }
    }

    // ----- constraints ----------------------------------------------------------------

    pub fn add_constraint(&mut self, c: Constraint) -> Result<ConstraintId, SketchError> {
        c.validate(self)?;
        Ok(self.constraints.insert(c))
    }

    pub fn remove_constraint(&mut self, id: ConstraintId) {
        self.constraints.remove(id);
        self.labels.remove(id);
    }

    pub fn constraint(&self, id: ConstraintId) -> Option<&Constraint> {
        self.constraints.get(id)
    }

    pub fn constraints(&self) -> impl Iterator<Item = (ConstraintId, &Constraint)> {
        self.constraints.iter()
    }

    pub fn set_dimension_value(&mut self, id: ConstraintId, value: f64) -> Result<(), SketchError> {
        let c = self
            .constraints
            .get_mut(id)
            .ok_or(SketchError::UnknownConstraint(id))?;
        if c.set_dimension_value(value) {
            // Typing a number over a driven dimension releases it, the way editing a
            // spreadsheet cell replaces its formula.
            self.dimension_exprs.remove(id);
            Ok(())
        } else {
            Err(SketchError::NotADimension(id))
        }
    }

    // ----- solving --------------------------------------------------------------------

    pub fn solve(&mut self) -> Result<SolveReport, SolveError> {
        // Parameters first: a dimension bound to an expression must reach the solver
        // holding the value that expression says, not the one it was last solved with.
        self.apply_parameters();
        crate::solver::solve(self)
    }

    /// Interactive drag: moves `point` toward `target` as a weak goal while all
    /// constraints hold, then solves.
    pub fn drag(&mut self, point: EntityId, target: Vec2) -> Result<SolveReport, SolveError> {
        crate::solver::drag(self, &[(point, target)])
    }

    /// Drags several points at once, each toward its own target. Moving a curve or a
    /// selection is this with every point offset by the same amount.
    pub fn drag_points(&mut self, goals: &[(EntityId, Vec2)]) -> Result<SolveReport, SolveError> {
        crate::solver::drag(self, goals)
    }

    /// The points an entity is made of: itself for a point, its endpoints and centre
    /// for a curve, its anchor for text. Moving these moves the entity.
    pub fn entity_points(&self, id: EntityId) -> Vec<EntityId> {
        match self.entities.get(id).map(|d| &d.entity) {
            Some(Entity::Point { .. }) => vec![id],
            Some(e) => e.references(),
            None => Vec::new(),
        }
    }

    // ----- dimension placement --------------------------------------------------------

    /// Where a dimension's label sits, in sketch coordinates, once the user has dragged
    /// it. Unplaced dimensions get a default position from the geometry.
    pub fn dimension_label(&self, id: ConstraintId) -> Option<Vec2> {
        self.labels.get(id).copied()
    }

    pub fn set_dimension_label(&mut self, id: ConstraintId, pos: Vec2) -> Result<(), SketchError> {
        if !self.constraints.contains_key(id) {
            return Err(SketchError::UnknownConstraint(id));
        }
        self.labels.insert(id, pos);
        Ok(())
    }

    // ----- fonts ----------------------------------------------------------------------

    pub fn set_font(&mut self, font: Option<Arc<Font>>) {
        self.font = font;
    }

    pub fn font(&self) -> Option<&Arc<Font>> {
        self.font.as_ref()
    }

    // ----- geometry queries -----------------------------------------------------------

    /// Positions of a curve's endpoints `(start, end)`; `None` for points, circles, text.
    pub fn curve_endpoints(&self, id: EntityId) -> Option<(Vec2, Vec2)> {
        match self.entities.get(id)?.entity {
            Entity::Line { start, end } | Entity::Arc { start, end, .. } => {
                Some((self.point_pos(start)?, self.point_pos(end)?))
            }
            _ => None,
        }
    }

    /// Flattened polyline of a curve. Circles return a closed loop with the first point
    /// repeated at the end so a line strip draws them closed. Points and text yield `None`.
    pub fn curve_polyline(&self, id: EntityId, tess: &Tessellation) -> Option<Vec<Vec2>> {
        match self.entities.get(id)?.entity {
            Entity::Line { start, end } => Some(vec![self.point_pos(start)?, self.point_pos(end)?]),
            Entity::Circle { center, radius } => {
                let mut pts = circle_polyline(self.point_pos(center)?, radius, tess);
                pts.push(pts[0]);
                Some(pts)
            }
            Entity::Arc { center, start, end } => Some(arc_polyline(
                self.point_pos(center)?,
                self.point_pos(start)?,
                self.point_pos(end)?,
                tess,
            )),
            Entity::Point { .. } | Entity::Text { .. } => None,
        }
    }

    /// Oriented corners of a text entity's box (baseline origin at the anchor), in
    /// baseline order: bottom-left, bottom-right, top-right, top-left.
    pub fn text_box(&self, id: EntityId) -> Option<[Vec2; 4]> {
        let Entity::Text {
            anchor,
            ref text,
            height,
            angle,
        } = self.entities.get(id)?.entity
        else {
            return None;
        };
        let origin = self.point_pos(anchor)?;
        let (width, y0, y1) = match &self.font {
            Some(font) => {
                let (asc, desc) = font.vertical_extent(height);
                (font.text_advance(text, height), desc, asc)
            }
            // Without metrics, assume an average glyph is 0.6 em wide.
            None => (0.6 * height * text.chars().count() as f64, 0.0, height),
        };
        let rot = Vec2::from_angle(angle);
        let local = [
            Vec2::new(0.0, y0),
            Vec2::new(width, y0),
            Vec2::new(width, y1),
            Vec2::new(0.0, y1),
        ];
        Some(local.map(|p| origin + rot.rotate(p)))
    }

    pub fn entity_bounds(&self, id: EntityId) -> Option<(Vec2, Vec2)> {
        match self.entities.get(id)?.entity {
            Entity::Point { pos } => Some((pos, pos)),
            Entity::Line { start, end } => {
                let (a, b) = (self.point_pos(start)?, self.point_pos(end)?);
                Some((a.min(b), a.max(b)))
            }
            Entity::Circle { center, radius } => {
                let c = self.point_pos(center)?;
                Some((c - Vec2::splat(radius), c + Vec2::splat(radius)))
            }
            Entity::Arc { center, start, end } => Some(arc_bounds(
                self.point_pos(center)?,
                self.point_pos(start)?,
                self.point_pos(end)?,
            )),
            Entity::Text { .. } => {
                let corners = self.text_box(id)?;
                let min = corners
                    .iter()
                    .fold(Vec2::splat(f64::INFINITY), |m, p| m.min(*p));
                let max = corners
                    .iter()
                    .fold(Vec2::splat(f64::NEG_INFINITY), |m, p| m.max(*p));
                Some((min, max))
            }
        }
    }

    /// Distance from `pos` to an entity, exact for curves; text uses its box.
    pub fn entity_distance(&self, id: EntityId, pos: Vec2) -> Option<f64> {
        match self.entities.get(id)?.entity {
            Entity::Point { pos: p } => Some(p.distance(pos)),
            Entity::Line { start, end } => Some(point_segment_distance(
                pos,
                self.point_pos(start)?,
                self.point_pos(end)?,
            )),
            Entity::Circle { center, radius } => {
                Some(point_circle_distance(pos, self.point_pos(center)?, radius))
            }
            Entity::Arc { center, start, end } => Some(point_arc_distance(
                pos,
                self.point_pos(center)?,
                self.point_pos(start)?,
                self.point_pos(end)?,
            )),
            Entity::Text { .. } => {
                let box_ = self.text_box(id)?;
                if crate::contour::polygon_contains(&box_, pos) {
                    return Some(0.0);
                }
                Some(
                    (0..4)
                        .map(|i| point_segment_distance(pos, box_[i], box_[(i + 1) % 4]))
                        .fold(f64::INFINITY, f64::min),
                )
            }
        }
    }

    /// Entities within `tolerance` of `pos`, nearest first. Points win ties against
    /// curves so a shared endpoint is picked over the curves that meet there.
    pub fn hit_test(&self, pos: Vec2, tolerance: f64) -> Vec<Hit> {
        let mut hits: Vec<(Hit, u8)> = self
            .entities
            .iter()
            .filter_map(|(id, d)| {
                let distance = self.entity_distance(id, pos)?;
                (distance <= tolerance).then_some((
                    Hit {
                        entity: id,
                        distance,
                    },
                    if d.entity.is_point() { 0 } else { 1 },
                ))
            })
            .collect();
        // Quantising the distance to the linear tolerance turns "equal within tolerance"
        // into a total order so the tie-break rule is well defined for sorting.
        hits.sort_by_key(|(h, rank)| (((h.distance / LINEAR_TOL).round()) as i64, *rank));
        hits.into_iter().map(|(h, _)| h).collect()
    }

    /// Rectangle selection. Window mode (`crossing == false`) selects entities whose
    /// bounds lie fully inside; crossing mode also selects anything touching the rectangle.
    pub fn hit_test_rect(&self, min: Vec2, max: Vec2, crossing: bool) -> Vec<EntityId> {
        let (min, max) = (min.min(max), min.max(max));
        let coarse = Tessellation {
            chord_tolerance: 0.05,
            max_segment_angle: 15f64.to_radians(),
        };
        self.entities
            .iter()
            .filter(|(id, d)| {
                let Some((bmin, bmax)) = self.entity_bounds(*id) else {
                    return false;
                };
                let inside = point_in_rect(bmin, min, max) && point_in_rect(bmax, min, max);
                if inside || !crossing {
                    return inside;
                }
                match d.entity {
                    Entity::Point { pos } => point_in_rect(pos, min, max),
                    Entity::Text { .. } => self
                        .text_box(*id)
                        .is_some_and(|b| polyline_intersects_rect(&b, true, min, max)),
                    _ => self
                        .curve_polyline(*id, &coarse)
                        .is_some_and(|p| polyline_intersects_rect(&p, false, min, max)),
                }
            })
            .map(|(id, _)| id)
            .collect()
    }

    // ----- profiles and paths ---------------------------------------------------------

    pub fn profiles(&self, tess: &Tessellation) -> Vec<Profile> {
        crate::profiles::profiles(self, tess)
    }

    /// Ordered open chain of lines/arcs (for sweep paths). Each curve is oriented so it
    /// starts where the previous one ended; an error names the first curve that does not.
    pub fn path(&self, curves: &[EntityId], tess: &Tessellation) -> Result<Contour, SketchError> {
        let mut points: Vec<Vec2> = Vec::new();
        let mut segments: Vec<Segment> = Vec::new();
        let mut prev_end: Option<Vec2> = None;
        for (index, &id) in curves.iter().enumerate() {
            let data = self
                .entities
                .get(id)
                .ok_or(SketchError::UnknownEntity(id))?;
            if !data.entity.is_open_curve() {
                return Err(SketchError::WrongEntityKind {
                    id,
                    expected: "line or arc",
                    actual: data.entity.kind_name(),
                });
            }
            let (start, end) = self
                .curve_endpoints(id)
                .ok_or(SketchError::UnknownEntity(id))?;
            let forward = match prev_end {
                Some(p) => {
                    if p.distance(start) <= JOIN_TOL {
                        true
                    } else if p.distance(end) <= JOIN_TOL {
                        false
                    } else {
                        return Err(SketchError::PathNotConnected { index });
                    }
                }
                // The first curve faces whichever way connects to the second.
                None => match curves.get(1).and_then(|n| self.curve_endpoints(*n)) {
                    Some((ns, ne)) => {
                        !(start.distance(ns) <= JOIN_TOL || start.distance(ne) <= JOIN_TOL)
                            || end.distance(ns) <= JOIN_TOL
                            || end.distance(ne) <= JOIN_TOL
                    }
                    None => true,
                },
            };
            let (mut pts, kind) = self
                .tessellate_open_curve(id, tess)
                .ok_or(SketchError::UnknownEntity(id))?;
            if !forward {
                pts.reverse();
            }
            let kind = match kind {
                SegmentKind::Arc {
                    center,
                    radius,
                    ccw,
                } => SegmentKind::Arc {
                    center,
                    radius,
                    ccw: ccw == forward,
                },
                SegmentKind::Line => SegmentKind::Line,
            };
            let skip = usize::from(prev_end.is_some());
            for _ in 1..pts.len() {
                segments.push(Segment { curve: id, kind });
            }
            points.extend(pts.iter().skip(skip));
            prev_end = pts.last().copied();
        }
        let closed = curves.len() > 1
            && points.len() > 2
            && points
                .first()
                .zip(points.last())
                .is_some_and(|(a, b)| a.distance(*b) <= JOIN_TOL);
        if closed {
            points.pop();
        }
        Ok(Contour {
            points,
            segments,
            closed,
        })
    }

    /// Polyline of a line or arc in its natural direction, with the segment kind that
    /// tags every edge of it.
    pub(crate) fn tessellate_open_curve(
        &self,
        id: EntityId,
        tess: &Tessellation,
    ) -> Option<(Vec<Vec2>, SegmentKind)> {
        match self.entities.get(id)?.entity {
            Entity::Line { .. } => Some((self.curve_polyline(id, tess)?, SegmentKind::Line)),
            Entity::Arc { center, start, .. } => {
                let c = self.point_pos(center)?;
                let radius = self.point_pos(start)?.distance(c);
                Some((
                    self.curve_polyline(id, tess)?,
                    SegmentKind::Arc {
                        center: c,
                        radius,
                        ccw: true,
                    },
                ))
            }
            _ => None,
        }
    }
}
