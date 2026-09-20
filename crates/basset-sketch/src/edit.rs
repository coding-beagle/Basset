//! Trimming and breaking existing geometry.
//!
//! Both tools work on the same idea: a curve is cut at every point where another curve
//! crosses it, which divides it into pieces named by parameter ranges. Break keeps all
//! of them; trim keeps all but the one the user picked. Fusion's Trim behaves this way
//! too, and it is why trimming a line that crosses nothing removes the whole line.
//!
//! Pieces of one curve share the point entities at the cuts, so the drawing stays joined
//! and a later drag moves both sides of a cut together. The picked curve's own entity is
//! reused for its first surviving piece, so constraints and dimensions written against
//! it survive the edit; geometry that has to change kind (a trimmed circle becomes an
//! arc) carries over the constraints that still make sense and drops the rest, the way
//! Fusion drops constraints it cannot honour rather than refusing the edit.

use basset_math::Vec2;

use crate::intersect::{CurveGeom, crossing_params, param_tol};
use crate::{Constraint, Entity, EntityId, Sketch, SketchError, Tessellation};

/// Removes the piece of `curve` under `pick`, cutting at the curves that cross it.
/// Returns the surviving pieces: empty when nothing crossed the curve and the whole of
/// it went.
pub fn trim(
    sketch: &mut Sketch,
    curve: EntityId,
    pick: Vec2,
) -> Result<Vec<EntityId>, SketchError> {
    apply(sketch, curve, Some(pick))
}

/// Cuts `curve` at every crossing without removing anything. `pick` is unused: breaking
/// is about the curve, not about where on it the user clicked.
pub fn break_curve(sketch: &mut Sketch, curve: EntityId) -> Result<Vec<EntityId>, SketchError> {
    apply(sketch, curve, None)
}

/// The shared machinery. `drop_at` is the pick of a trim; `None` breaks instead.
fn apply(
    sketch: &mut Sketch,
    curve: EntityId,
    drop_at: Option<Vec2>,
) -> Result<Vec<EntityId>, SketchError> {
    let data = sketch
        .entity(curve)
        .ok_or(SketchError::UnknownEntity(curve))?;
    if !data.entity.is_curve() {
        return Err(SketchError::WrongEntityKind {
            id: curve,
            expected: "line, arc or circle",
            actual: data.entity.kind_name(),
        });
    }
    let construction = data.construction;
    let geom = CurveGeom::of(sketch, curve).ok_or(SketchError::UnknownEntity(curve))?;
    let cuts = crossing_params(sketch, curve);
    let tol = param_tol(&geom);
    if geom.is_closed() {
        closed(sketch, curve, &geom, &cuts, tol, drop_at, construction)
    } else {
        open(sketch, curve, &geom, &cuts, tol, drop_at, construction)
    }
}

/// A line or an arc: the ends are cuts the geometry already has.
fn open(
    sketch: &mut Sketch,
    curve: EntityId,
    geom: &CurveGeom,
    cuts: &[f64],
    tol: f64,
    drop_at: Option<Vec2>,
    construction: bool,
) -> Result<Vec<EntityId>, SketchError> {
    let (start_point, end_point, center) = ends(sketch, curve)?;
    let bounds = piece_bounds(geom, cuts, tol);
    let keep = kept_pieces(&bounds, geom, tol, drop_at);
    if keep.is_empty() {
        remove_with_orphans(sketch, curve, &[start_point, end_point]);
        return Ok(Vec::new());
    }
    // Only the cuts that actually bound a surviving piece become points; a cut between
    // two removed pieces would otherwise leave a point floating in mid-air.
    let mut points: Vec<Option<EntityId>> = vec![None; bounds.len()];
    points[0] = Some(start_point);
    points[bounds.len() - 1] = Some(end_point);
    for &i in &keep {
        for b in [i, i + 1] {
            if points[b].is_none() {
                points[b] = Some(sketch.add_point(geom.point_at(bounds[b])));
            }
        }
    }
    let mut out = Vec::new();
    for (n, &i) in keep.iter().enumerate() {
        let (a, b) = (
            points[i].expect("bound of a kept piece"),
            points[i + 1].expect("bound of a kept piece"),
        );
        if n == 0 {
            // Reusing the original entity is what keeps its constraints alive.
            set_ends(sketch, curve, a, b);
            out.push(curve);
        } else {
            let piece = match center {
                Some(c) => sketch.add_arc(c, a, b)?,
                None => sketch.add_line(a, b)?,
            };
            sketch.set_construction(piece, construction)?;
            out.push(piece);
        }
    }
    prune_orphans(sketch, &[start_point, end_point]);
    Ok(out)
}

/// A circle: every piece is an arc, and the parameter wraps, so the last piece runs from
/// the last cut back round to the first.
fn closed(
    sketch: &mut Sketch,
    curve: EntityId,
    geom: &CurveGeom,
    cuts: &[f64],
    tol: f64,
    drop_at: Option<Vec2>,
    construction: bool,
) -> Result<Vec<EntityId>, SketchError> {
    let Entity::Circle { center, .. } = sketch
        .entity(curve)
        .ok_or(SketchError::UnknownEntity(curve))?
        .entity
    else {
        return Err(SketchError::UnknownEntity(curve));
    };
    // One crossing does not divide a closed curve: the single piece is the circle itself.
    if cuts.len() < 2 {
        if drop_at.is_some() {
            sketch.remove_entity(curve);
            return Ok(Vec::new());
        }
        return Ok(vec![curve]);
    }
    let bounds = piece_bounds(geom, cuts, tol);
    let keep = kept_pieces(&bounds, geom, tol, drop_at);
    let cut_points: Vec<EntityId> = bounds[..bounds.len() - 1]
        .iter()
        .map(|t| sketch.add_point(geom.point_at(*t)))
        .collect();
    let saved: Vec<Constraint> = sketch
        .constraints()
        .filter(|(_, c)| c.references().contains(&curve))
        .map(|(_, c)| c.clone())
        .collect();
    let mut out = Vec::new();
    for &i in &keep {
        let a = cut_points[i % cut_points.len()];
        let b = cut_points[(i + 1) % cut_points.len()];
        let piece = sketch.add_arc(center, a, b)?;
        sketch.set_construction(piece, construction)?;
        out.push(piece);
    }
    sketch.remove_entity(curve);
    // Unused cut points appear when a trim leaves only one arc; they belong to no curve.
    prune_orphans(sketch, &cut_points);
    if let Some(&heir) = out.first() {
        for mut c in saved {
            c.retarget(curve, heir);
            // A constraint that the arc cannot carry (nothing yet, but the kinds may
            // diverge) is dropped rather than failing the trim.
            if let Err(e) = sketch.add_constraint(c) {
                log::debug!("trim dropped a constraint: {e}");
            }
        }
    }
    Ok(out)
}

/// Parameters dividing the curve into pieces, in order. An open curve is bounded by its
/// own ends; a closed one wraps, so its last piece is written as a range running past 1.
fn piece_bounds(geom: &CurveGeom, cuts: &[f64], tol: f64) -> Vec<f64> {
    if geom.is_closed() {
        let mut bounds: Vec<f64> = cuts.to_vec();
        if let Some(first) = bounds.first().copied() {
            bounds.push(first + 1.0);
        }
        bounds
    } else {
        let mut bounds = vec![0.0];
        bounds.extend(cuts.iter().copied().filter(|t| *t > tol && *t < 1.0 - tol));
        bounds.push(1.0);
        bounds
    }
}

/// Which piece `pick` falls in.
fn picked_piece(bounds: &[f64], geom: &CurveGeom, pick: Vec2) -> usize {
    let t = geom.param_of(pick);
    // A pick before the first cut of a wrapped (circle) parameterisation belongs to the
    // piece that runs past the end.
    let t = if t < bounds[0] { t + 1.0 } else { t };
    bounds
        .windows(2)
        .position(|w| t >= w[0] && t <= w[1])
        .unwrap_or(bounds.len() - 2)
}

/// Indices of the pieces to keep: all of them for a break, all but the picked one for a
/// trim. Pieces shorter than the join tolerance are dropped either way, since they are
/// the slivers two crossings in the same place leave behind.
fn kept_pieces(bounds: &[f64], geom: &CurveGeom, tol: f64, drop_at: Option<Vec2>) -> Vec<usize> {
    let picked = drop_at.map(|p| picked_piece(bounds, geom, p));
    (0..bounds.len() - 1)
        .filter(|i| Some(*i) != picked && bounds[i + 1] - bounds[*i] > tol)
        .collect()
}

/// The piece [`trim`] would remove, as a polyline, so the tool can show the user what
/// their click is about to take. It is derived from the same bounds the trim itself
/// uses, so what is shown is what goes.
pub fn trim_preview(
    sketch: &Sketch,
    curve: EntityId,
    pick: Vec2,
    tess: &Tessellation,
) -> Option<Vec<Vec2>> {
    let geom = CurveGeom::of(sketch, curve)?;
    let bounds = piece_bounds(&geom, &crossing_params(sketch, curve), param_tol(&geom));
    if bounds.len() < 2 {
        return None;
    }
    let i = picked_piece(&bounds, &geom, pick);
    let (from, to) = (bounds[i], bounds[i + 1]);
    let steps = match geom {
        CurveGeom::Line { .. } => 1,
        CurveGeom::Arc { radius, sweep, .. } => tess.segment_count(radius, sweep * (to - from)),
    };
    // A wrapped piece runs past 1; the arc parameterisation is periodic, so a parameter
    // of 1.3 on a circle is simply the point a third of the way round again.
    Some(
        (0..=steps)
            .map(|n| geom.point_at(from + (to - from) * n as f64 / steps as f64))
            .collect(),
    )
}

/// A curve's `(start, end, centre)` points; the centre is `None` for a line.
fn ends(
    sketch: &Sketch,
    curve: EntityId,
) -> Result<(EntityId, EntityId, Option<EntityId>), SketchError> {
    match sketch
        .entity(curve)
        .ok_or(SketchError::UnknownEntity(curve))?
        .entity
    {
        Entity::Line { start, end } => Ok((start, end, None)),
        Entity::Arc { center, start, end } => Ok((start, end, Some(center))),
        ref other => Err(SketchError::WrongEntityKind {
            id: curve,
            expected: "line or arc",
            actual: other.kind_name(),
        }),
    }
}

fn set_ends(sketch: &mut Sketch, curve: EntityId, a: EntityId, b: EntityId) {
    if let Some(data) = sketch.entities.get_mut(curve) {
        match &mut data.entity {
            Entity::Line { start, end } | Entity::Arc { start, end, .. } => {
                *start = a;
                *end = b;
            }
            _ => {}
        }
    }
}

fn remove_with_orphans(sketch: &mut Sketch, curve: EntityId, candidates: &[EntityId]) {
    sketch.remove_entity(curve);
    prune_orphans(sketch, candidates);
}

/// Deletes points that the edit left attached to nothing. A point the user drew
/// deliberately is never a candidate here: only the endpoints of the curve being cut are
/// offered, and one still holding a constraint (a dimension, a coincidence) stays.
fn prune_orphans(sketch: &mut Sketch, candidates: &[EntityId]) {
    for &p in candidates {
        let used_by_curve = sketch
            .entities()
            .any(|(_, d)| d.entity.references().contains(&p));
        let used_by_constraint = sketch
            .constraints()
            .any(|(_, c)| c.references().contains(&p));
        if !used_by_curve && !used_by_constraint {
            sketch.remove_entity(p);
        }
    }
}
