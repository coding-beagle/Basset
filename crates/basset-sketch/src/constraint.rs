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
    /// How far an offset lies from what it was offset from: each pair's result is
    /// `value` from its source, on the side it is on now. One dimension for the whole
    /// offset, so changing the distance is editing one number rather than redrawing.
    ///
    /// It names only one pair per run of the result whose pieces meet smoothly, because
    /// the tangencies and shared joints the offset writes down already carry the
    /// distance along such a run; naming every piece would say the same thing twice and
    /// every extra pair would show as redundant. See [`crate::offset`].
    Offset {
        pairs: Vec<OffsetPair>,
        value: f64,
    },
}

/// One curve of an offset and the curve it is the offset of: two lines, or two
/// circles/arcs.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct OffsetPair {
    pub source: EntityId,
    pub result: EntityId,
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
            | Constraint::Angle { value, .. }
            | Constraint::Offset { value, .. } => Some(value),
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
            | Constraint::Angle { value, .. }
            | Constraint::Offset { value, .. } => {
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
            Constraint::Offset { ref pairs, .. } => {
                pairs.iter().flat_map(|p| [p.source, p.result]).collect()
            }
        }
    }

    /// Drops every offset pair that mentions one of `gone`, and says whether anything of
    /// the constraint is left. An offset losing one of its curves still holds the rest
    /// at the distance, so deleting one edge of an offset rectangle should not throw the
    /// dimension of the other three away with it. Every other constraint is all or
    /// nothing.
    pub(crate) fn survives_without(&mut self, gone: &[EntityId]) -> bool {
        match self {
            Constraint::Offset { pairs, .. } => {
                pairs.retain(|p| !gone.contains(&p.source) && !gone.contains(&p.result));
                !pairs.is_empty()
            }
            other => !other.references().iter().any(|r| gone.contains(r)),
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
            Constraint::Offset { pairs, .. } => {
                for p in pairs {
                    swap(&mut p.source);
                    swap(&mut p.result);
                }
            }
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
            Constraint::Offset { ref pairs, .. } => {
                if pairs.is_empty() {
                    return Err(SketchError::InvalidArgument(
                        "an offset dimension needs at least one curve to measure".into(),
                    ));
                }
                for p in pairs {
                    if kind(p.source)?.is_line() {
                        line(p.result)?;
                    } else {
                        circular(p.source)?;
                        circular(p.result)?;
                    }
                }
                Ok(())
            }
        }
    }
}
