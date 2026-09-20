//! Rectangular and circular patterns of sketch geometry.
//!
//! A pattern copies the picked entities, the points they are built from, and every
//! constraint written *between* them. Copying the constraints is what makes a patterned
//! rectangle still a rectangle and a patterned slot still a slot: each copy is as
//! constrained as the original, so dragging one corner of a copy deforms it exactly as
//! it would deform the seed.
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
    for mut c in inherited {
        for (from, to) in &map {
            c.retarget(*from, *to);
        }
        if let Err(e) = sketch.add_constraint(c) {
            log::debug!("pattern dropped a constraint: {e}");
        }
    }
    Ok(created)
}
