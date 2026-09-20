//! Rectangular and circular patterns of sketch geometry.
//!
//! A pattern copies the picked entities, the points they are built from, and every
//! constraint written *between* them. Copying the constraints is what makes a patterned
//! rectangle still a rectangle and a patterned slot still a slot: each copy is as
//! constrained as the original, so dragging one corner of a copy deforms it exactly as
//! it would deform the seed.
//!
//! A turned copy keeps every constraint that still describes it. Horizontal and Vertical
//! are the exception, because they are statements about the sketch's axes rather than
//! about the shape: a quarter turn swaps them, a half turn keeps them, and any other
//! angle leaves them unsayable, so they are dropped and the copy is honestly loose
//! rather than dishonestly crushed. See [`turned`].
//!
//! Copies are not linked back to the seed. Fusion's sketch patterns are; ours would need
//! a pattern entity in the sketch to re-generate from, and until that exists a pattern
//! that silently broke its copies when the seed moved would be worse than one that is
//! honestly plain geometry. Dimensions copy across, so the copies are driven, not loose.

use std::collections::HashMap;
use std::f64::consts::TAU;

use basset_math::{ANGULAR_TOL, Vec2};

use crate::{Entity, EntityId, Sketch, SketchError};

/// A rigid motion of a copy: rotate about `about`, then translate.
#[derive(Clone, Copy, Debug)]
struct Placement {
    about: Vec2,
    rotate: f64,
    translate: Vec2,
}

impl Placement {
    fn apply(&self, p: Vec2) -> Vec2 {
        self.about + Vec2::from_angle(self.rotate).rotate(p - self.about) + self.translate
    }
}

/// `cols` × `rows` copies of `seed`, stepping by `x_step` across and `y_step` up. The
/// seed itself is instance (0, 0) and is left alone; the new entities are returned.
pub fn rectangular(
    sketch: &mut Sketch,
    seed: &[EntityId],
    x_step: Vec2,
    cols: usize,
    y_step: Vec2,
    rows: usize,
) -> Result<Vec<EntityId>, SketchError> {
    if cols == 0 || rows == 0 {
        return Err(SketchError::InvalidArgument(
            "a pattern needs at least one instance in each direction".into(),
        ));
    }
    let mut placements = Vec::new();
    for row in 0..rows {
        for col in 0..cols {
            if row == 0 && col == 0 {
                continue;
            }
            placements.push(Placement {
                about: Vec2::ZERO,
                rotate: 0.0,
                translate: x_step * col as f64 + y_step * row as f64,
            });
        }
    }
    copy_all(sketch, seed, &placements)
}

/// `count` copies of `seed` (the seed included in the count) spread around `center`
/// through `total_angle`. A full turn distributes the copies evenly around the circle;
/// anything less puts the last copy exactly on `total_angle`, as Fusion's angular
/// spacing does.
pub fn circular(
    sketch: &mut Sketch,
    seed: &[EntityId],
    center: Vec2,
    count: usize,
    total_angle: f64,
) -> Result<Vec<EntityId>, SketchError> {
    if count < 2 {
        return Err(SketchError::InvalidArgument(
            "a circular pattern needs at least two instances".into(),
        ));
    }
    if !total_angle.is_finite() || total_angle.abs() <= ANGULAR_TOL {
        return Err(SketchError::InvalidArgument(format!(
            "circular pattern angle must be non-zero, got {total_angle}"
        )));
    }
    let full = total_angle.abs() >= TAU - ANGULAR_TOL;
    let step = total_angle
        / if full {
            count as f64
        } else {
            (count - 1) as f64
        };
    let placements: Vec<Placement> = (1..count)
        .map(|i| Placement {
            about: center,
            rotate: step * i as f64,
            translate: Vec2::ZERO,
        })
        .collect();
    copy_all(sketch, seed, &placements)
}

fn copy_all(
    sketch: &mut Sketch,
    seed: &[EntityId],
    placements: &[Placement],
) -> Result<Vec<EntityId>, SketchError> {
    let set = closure(sketch, seed)?;
    let mut created = Vec::new();
    for placement in placements {
        created.extend(copy_once(sketch, &set, *placement)?);
    }
    Ok(created)
}

/// The seed plus the points its curves are built from, in a stable order with points
/// first so a copy can be made in one pass.
fn closure(sketch: &Sketch, seed: &[EntityId]) -> Result<Vec<EntityId>, SketchError> {
    let mut points = Vec::new();
    let mut curves = Vec::new();
    for &id in seed {
        let data = sketch.entity(id).ok_or(SketchError::UnknownEntity(id))?;
        for p in data.entity.references() {
            if !points.contains(&p) {
                points.push(p);
            }
        }
        if data.entity.is_point() {
            if !points.contains(&id) {
                points.push(id);
            }
        } else if !curves.contains(&id) {
            curves.push(id);
        }
    }
    points.extend(curves);
    Ok(points)
}

/// A constraint as it applies to a copy turned through `rotate`, or `None` when it
/// cannot be expressed there at all.
///
/// Horizontal and Vertical are statements about the sketch's axes, not about the shape,
/// and a copy turned through a quarter of a turn has them the other way round. Copying
/// them unchanged does not merely mis-describe the copy — it contradicts it, and the
/// solver resolves the contradiction by folding the copy flat, which is what made a
/// circular pattern of anything drawn square destroy its own copies. A turn that is not
/// a multiple of a right angle has no axis-aligned answer at all, so those constraints
/// are dropped: the copy is then free where the seed was pinned, which is visible in the
/// blue and is a great deal better than a copy crushed to a point.
fn turned(c: crate::Constraint, rotate: f64) -> Option<crate::Constraint> {
    use crate::Constraint as C;
    // Quarter turns from the rotation, as a whole number: 0 keeps the axes, 2 keeps them
    // reversed, 1 and 3 swap them, and anything in between keeps neither.
    let quarters = rotate / std::f64::consts::FRAC_PI_2;
    let whole = quarters.round();
    let axes_survive = (quarters - whole).abs() * std::f64::consts::FRAC_PI_2 <= ANGULAR_TOL;
    if !axes_survive {
        return match c {
            C::Horizontal(_)
            | C::Vertical(_)
            | C::HorizontalDistance { .. }
            | C::VerticalDistance { .. } => None,
            other => Some(other),
        };
    }
    let quarter = (whole as i64).rem_euclid(4);
    // The turn maps (dx, dy) to (-dy, dx) each quarter, so a half turn negates both and
    // the odd quarters swap the axes with one sign flipped.
    Some(match (quarter, c) {
        (0, other) => other,
        (2, C::HorizontalDistance { a, b, value }) => C::HorizontalDistance {
            a,
            b,
            value: -value,
        },
        (2, C::VerticalDistance { a, b, value }) => C::VerticalDistance {
            a,
            b,
            value: -value,
        },
        (2, other) => other,
        (_, C::Horizontal(l)) => C::Vertical(l),
        (_, C::Vertical(l)) => C::Horizontal(l),
        (1, C::HorizontalDistance { a, b, value }) => C::VerticalDistance { a, b, value },
        (1, C::VerticalDistance { a, b, value }) => C::HorizontalDistance {
            a,
            b,
            value: -value,
        },
        (_, C::HorizontalDistance { a, b, value }) => C::VerticalDistance {
            a,
            b,
            value: -value,
        },
        (_, C::VerticalDistance { a, b, value }) => C::HorizontalDistance { a, b, value },
        (_, other) => other,
    })
}

fn copy_once(
    sketch: &mut Sketch,
    set: &[EntityId],
    placement: Placement,
) -> Result<Vec<EntityId>, SketchError> {
    let mut map: HashMap<EntityId, EntityId> = HashMap::new();
    let mut created = Vec::new();
    for &id in set {
        let data = sketch.entity(id).ok_or(SketchError::UnknownEntity(id))?;
        let construction = data.construction;
        let mapped = |of: EntityId| -> Result<EntityId, SketchError> {
            map.get(&of).copied().ok_or(SketchError::UnknownEntity(of))
        };
        let new_id = match data.entity.clone() {
            Entity::Point { pos } => sketch.add_point(placement.apply(pos)),
            Entity::Line { start, end } => sketch.add_line(mapped(start)?, mapped(end)?)?,
            Entity::Circle { center, radius } => sketch.add_circle(mapped(center)?, radius)?,
            Entity::Arc { center, start, end } => {
                sketch.add_arc(mapped(center)?, mapped(start)?, mapped(end)?)?
            }
            Entity::Text {
                anchor,
                text,
                height,
                angle,
            } => sketch.add_text(mapped(anchor)?, text, height, angle + placement.rotate)?,
        };
        sketch.set_construction(new_id, construction)?;
        map.insert(id, new_id);
        created.push(new_id);
    }
    // Only constraints wholly inside the copied set can be copied: one tying the seed to
    // geometry outside it says where the seed is, not what shape it has, and repeating
    // it would drag every copy back onto the seed.
    let inherited: Vec<crate::Constraint> = sketch
        .constraints()
        .filter(|(_, c)| c.references().iter().all(|r| map.contains_key(r)))
        .map(|(_, c)| c.clone())
        .collect();
    for c in inherited {
        let Some(mut c) = turned(c, placement.rotate) else {
            continue;
        };
        for (from, to) in &map {
            c.retarget(*from, *to);
        }
        if let Err(e) = sketch.add_constraint(c) {
            log::debug!("pattern dropped a constraint: {e}");
        }
    }
    Ok(created)
}
