//! Constraint definitions and their kind validation. The numerical meaning of each
//! constraint lives in `solver::equations`; this module only knows what may reference what.

use serde::{Deserialize, Serialize};

use crate::{Entity, EntityId, Sketch, SketchError};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum Constraint {
    /// Point on point, point on (infinite) line, or point on the circle of a circle/arc.
    Coincident {
        point: EntityId,
        target: EntityId,
    },
    Horizontal(EntityId),
    Vertical(EntityId),
    Parallel(EntityId, EntityId),
    Perpendicular(EntityId, EntityId),
    /// Equal line lengths, or equal radii of circles/arcs.
    Equal(EntityId, EntityId),
    /// Line–circle/arc, or circle/arc–circle/arc.
    Tangent(EntityId, EntityId),
    /// Locks a point in place. Fixed points are removed from the solver's free variables.
    Fix(EntityId),
    Midpoint {
        point: EntityId,
        line: EntityId,
    },
    /// Two points mirrored across a line.
    Symmetric {
        a: EntityId,
        b: EntityId,
        axis: EntityId,
    },
    Concentric(EntityId, EntityId),
    // Driving dimensions.
    /// Point–point or point–line distance.
    Distance {
        a: EntityId,
        b: EntityId,
        value: f64,
    },
    HorizontalDistance {
        a: EntityId,
        b: EntityId,
        value: f64,
    },
    VerticalDistance {
        a: EntityId,
        b: EntityId,
        value: f64,
    },
    Radius {
        curve: EntityId,
        value: f64,
    },
    Diameter {
        curve: EntityId,
        value: f64,
    },
    /// Angle between two lines, in radians.
    Angle {
        a: EntityId,
        b: EntityId,
        value: f64,
    },
}

impl Constraint {
    pub fn is_dimension(&self) -> bool {
        self.dimension_value().is_some()
    }

    pub fn dimension_value(&self) -> Option<f64> {
        match *self {
            Constraint::Distance { value, .. }
            | Constraint::HorizontalDistance { value, .. }
            | Constraint::VerticalDistance { value, .. }
            | Constraint::Radius { value, .. }
            | Constraint::Diameter { value, .. }
            | Constraint::Angle { value, .. } => Some(value),
            _ => None,
        }
    }

    /// Replaces the driving value; `false` if this is not a dimension.
    pub fn set_dimension_value(&mut self, new_value: f64) -> bool {
        match self {
            Constraint::Distance { value, .. }
            | Constraint::HorizontalDistance { value, .. }
            | Constraint::VerticalDistance { value, .. }
            | Constraint::Radius { value, .. }
            | Constraint::Diameter { value, .. }
            | Constraint::Angle { value, .. } => {
                *value = new_value;
                true
            }
            _ => false,
        }
    }

    /// Every entity the constraint mentions, for dependency removal.
    pub fn references(&self) -> Vec<EntityId> {
        match *self {
            Constraint::Coincident { point, target } => vec![point, target],
            Constraint::Horizontal(a) | Constraint::Vertical(a) | Constraint::Fix(a) => vec![a],
            Constraint::Parallel(a, b)
            | Constraint::Perpendicular(a, b)
            | Constraint::Equal(a, b)
            | Constraint::Tangent(a, b)
            | Constraint::Concentric(a, b) => vec![a, b],
            Constraint::Midpoint { point, line } => vec![point, line],
            Constraint::Symmetric { a, b, axis } => vec![a, b, axis],
            Constraint::Distance { a, b, .. }
            | Constraint::HorizontalDistance { a, b, .. }
            | Constraint::VerticalDistance { a, b, .. }
            | Constraint::Angle { a, b, .. } => vec![a, b],
            Constraint::Radius { curve, .. } | Constraint::Diameter { curve, .. } => vec![curve],
        }
    }

    /// Rewrites every reference to `from` as `to`, for edits that replace one entity
    /// with another of the same role (a trimmed circle becoming an arc). The result is
    /// validated by the caller: the new entity may not accept the constraint.
    pub(crate) fn retarget(&mut self, from: EntityId, to: EntityId) {
        let swap = |id: &mut EntityId| {
            if *id == from {
                *id = to;
            }
        };
        match self {
            Constraint::Coincident { point, target } => {
                swap(point);
                swap(target);
            }
            Constraint::Horizontal(a) | Constraint::Vertical(a) | Constraint::Fix(a) => swap(a),
            Constraint::Parallel(a, b)
            | Constraint::Perpendicular(a, b)
            | Constraint::Equal(a, b)
            | Constraint::Tangent(a, b)
            | Constraint::Concentric(a, b) => {
                swap(a);
                swap(b);
            }
            Constraint::Midpoint { point, line } => {
                swap(point);
                swap(line);
            }
            Constraint::Symmetric { a, b, axis } => {
                swap(a);
                swap(b);
                swap(axis);
            }
            Constraint::Distance { a, b, .. }
            | Constraint::HorizontalDistance { a, b, .. }
            | Constraint::VerticalDistance { a, b, .. }
            | Constraint::Angle { a, b, .. } => {
                swap(a);
                swap(b);
            }
            Constraint::Radius { curve, .. } | Constraint::Diameter { curve, .. } => swap(curve),
        }
    }

    /// Checks that every referenced entity exists and has a kind the constraint accepts.
    pub(crate) fn validate(&self, sketch: &Sketch) -> Result<(), SketchError> {
        let kind = |id: EntityId| -> Result<&Entity, SketchError> {
            sketch
                .entity(id)
                .map(|d| &d.entity)
                .ok_or(SketchError::UnknownEntity(id))
        };
        let expect = |id: EntityId,
                      ok: fn(&Entity) -> bool,
                      expected: &'static str|
         -> Result<(), SketchError> {
            let e = kind(id)?;
            if ok(e) {
                Ok(())
            } else {
                Err(SketchError::WrongEntityKind {
                    id,
                    expected,
                    actual: e.kind_name(),
                })
            }
        };
        let point = |id| expect(id, Entity::is_point, "point");
        let line = |id| expect(id, Entity::is_line, "line");
        let circular = |id| expect(id, Entity::is_circular, "circle or arc");
        match *self {
            Constraint::Coincident { point: p, target } => {
                point(p)?;
                expect(
                    target,
                    |e| e.is_point() || e.is_curve(),
                    "point, line, circle or arc",
                )
            }
            Constraint::Horizontal(l) | Constraint::Vertical(l) => line(l),
            Constraint::Parallel(a, b)
            | Constraint::Perpendicular(a, b)
            | Constraint::Angle { a, b, .. } => {
                line(a)?;
                line(b)
            }
            Constraint::Equal(a, b) => {
                let (ea, eb) = (kind(a)?, kind(b)?);
                match (
                    ea.is_line(),
                    eb.is_line(),
                    ea.is_circular(),
                    eb.is_circular(),
                ) {
                    (true, true, _, _) | (_, _, true, true) => Ok(()),
                    (true, false, _, _) => Err(SketchError::WrongEntityKind {
                        id: b,
                        expected: "line",
                        actual: eb.kind_name(),
                    }),
                    (_, _, true, false) => Err(SketchError::WrongEntityKind {
                        id: b,
                        expected: "circle or arc",
                        actual: eb.kind_name(),
                    }),
                    _ => Err(SketchError::WrongEntityKind {
                        id: a,
                        expected: "line, circle or arc",
                        actual: ea.kind_name(),
                    }),
                }
            }
            Constraint::Tangent(a, b) => {
                let (ea, eb) = (kind(a)?, kind(b)?);
                let ok = (ea.is_line() && eb.is_circular())
                    || (ea.is_circular() && eb.is_line())
                    || (ea.is_circular() && eb.is_circular());
                if ok {
                    Ok(())
                } else {
                    let bad = if ea.is_line() || ea.is_circular() {
                        (b, eb)
                    } else {
                        (a, ea)
                    };
                    Err(SketchError::WrongEntityKind {
                        id: bad.0,
                        expected: "line, circle or arc",
                        actual: bad.1.kind_name(),
                    })
                }
            }
            Constraint::Fix(p) => point(p),
            Constraint::Midpoint { point: p, line: l } => {
                point(p)?;
                line(l)
            }
            Constraint::Symmetric { a, b, axis } => {
                point(a)?;
                point(b)?;
                line(axis)
            }
            Constraint::Concentric(a, b) => {
                circular(a)?;
                circular(b)
            }
            Constraint::Distance { a, b, .. } => {
                point(a)?;
                expect(b, |e| e.is_point() || e.is_line(), "point or line")
            }
            Constraint::HorizontalDistance { a, b, .. }
            | Constraint::VerticalDistance { a, b, .. } => {
                point(a)?;
                point(b)
            }
            Constraint::Radius { curve, .. } | Constraint::Diameter { curve, .. } => {
                circular(curve)
            }
        }
    }
}
