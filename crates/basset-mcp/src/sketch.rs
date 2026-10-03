//! The drawing operations one `sketch_ops` call applies, in order, to one sketch.
//!
//! Each op is a JSON object with an `"op"` and its arguments. A point argument is either
//! an entity id (reusing a point that exists, which is how corners are shared) or
//! `[x, y]` (a new point). The ops are applied to a *copy* of the sketch and the copy is
//! written back only when every one of them succeeded, so a batch is all or nothing and
//! a refused op leaves the document as it was.
//!
//! Angles cross this boundary in degrees, as they do in the editor's dimension boxes.

use basset_math::Vec2;
use basset_sketch::{Constraint, EntityId, Outer, Sketch, edit, fillet, offset, pattern, shapes};
use serde_json::{Value, json};

use crate::ToolError;
use crate::args::{
    f64_of, field, opt, opt_bool, opt_str, req_array, req_f64, req_str, req_u64, req_vec2, vec2_of,
};
use crate::ids::{constraint_from_json, constraint_to_json, entity_from_json, entity_to_json};
use crate::summary::{report, round, v2};

pub fn apply_ops(
    sketch: &mut Sketch,
    outer: Outer,
    ops: &[Value],
) -> Result<Vec<Value>, ToolError> {
    let mut results = Vec::with_capacity(ops.len());
    for (i, op) in ops.iter().enumerate() {
        let name = op
            .get("op")
            .and_then(Value::as_str)
            .ok_or_else(|| ToolError::bad(format!("op {i} has no \"op\" name")))?;
        let result = apply_one(sketch, outer, name, op)
            .map_err(|e| ToolError::bad(format!("op {i} ({name}): {e}")))?;
        results.push(result);
    }
    Ok(results)
}

/// A point by id or by coordinates; coordinates make a new point.
fn point_arg(sketch: &mut Sketch, v: &Value, name: &str) -> Result<EntityId, ToolError> {
    match v {
        Value::Number(_) => {
            let id = entity_from_json(sketch, v)?;
            match sketch.entity(id) {
                Some(d) if d.entity.is_point() => Ok(id),
                Some(d) => Err(ToolError::bad(format!(
                    "\"{name}\" is a {}, not a point",
                    d.entity.kind_name()
                ))),
                None => Err(ToolError::bad(format!("\"{name}\": no entity {v}"))),
            }
        }
        _ => Ok(sketch.add_point(vec2_of(v, name)?)),
    }
}

fn ids(v: &[EntityId]) -> Vec<Value> {
    v.iter().map(|id| entity_to_json(*id)).collect()
}

fn entity_list(sketch: &Sketch, args: &Value, name: &str) -> Result<Vec<EntityId>, ToolError> {
    req_array(args, name)?
        .iter()
        .map(|v| entity_from_json(sketch, v))
        .collect()
}

fn entity_arg(sketch: &Sketch, args: &Value, name: &str) -> Result<EntityId, ToolError> {
    entity_from_json(sketch, field(args, name)?)
}

fn constraint_arg(sketch: &Sketch, args: &Value) -> Result<basset_sketch::ConstraintId, ToolError> {
    constraint_from_json(sketch, field(args, "constraint")?)
}

fn apply_one(
    sketch: &mut Sketch,
    outer: Outer,
    name: &str,
    args: &Value,
) -> Result<Value, ToolError> {
    Ok(match name {
        "point" => {
            let id = sketch.add_point(req_vec2(args, "at")?);
            if opt_bool(args, "construction")?.unwrap_or(false) {
                sketch.set_construction(id, true)?;
            }
            json!({ "point": entity_to_json(id) })
        }
        "line" => {
            let a = point_arg(sketch, field(args, "from")?, "from")?;
            let b = point_arg(sketch, field(args, "to")?, "to")?;
            let line = sketch.add_line(a, b)?;
            if opt_bool(args, "construction")?.unwrap_or(false) {
                sketch.set_construction(line, true)?;
            }
            json!({ "line": entity_to_json(line), "start": entity_to_json(a), "end": entity_to_json(b) })
        }
        "polyline" => {
            let points: Vec<Vec2> = req_array(args, "points")?
                .iter()
                .map(|p| vec2_of(p, "points"))
                .collect::<Result<_, _>>()?;
            if points.len() < 2 {
                return Err(ToolError::bad("a polyline needs at least two points"));
            }
            let closed = opt_bool(args, "closed")?.unwrap_or(false);
            let p = shapes::polyline(sketch, &points, closed);
            json!({ "points": ids(&p.points), "lines": ids(&p.lines) })
        }
        "rectangle" => {
            let r = shapes::rectangle_two_point(sketch, req_vec2(args, "a")?, req_vec2(args, "b")?);
            json!({ "corners": ids(&r.corners), "lines": ids(&r.lines), "constraints": r.constraints.iter().map(|c| constraint_to_json(*c)).collect::<Vec<_>>() })
        }
        "rectangle_center" => {
            let r = shapes::rectangle_center(
                sketch,
                req_vec2(args, "center")?,
                req_vec2(args, "corner")?,
            );
            json!({ "corners": ids(&r.corners), "lines": ids(&r.lines), "center": r.center.map(entity_to_json) })
        }
        "circle" => {
            let c =
                shapes::circle_center(sketch, req_vec2(args, "center")?, req_f64(args, "radius")?);
            json!({ "circle": entity_to_json(c.circle), "center": entity_to_json(c.center) })
        }
        "circle_two_point" => {
            let c = shapes::circle_two_point(sketch, req_vec2(args, "a")?, req_vec2(args, "b")?);
            json!({ "circle": entity_to_json(c.circle), "center": entity_to_json(c.center) })
        }
        "circle_three_point" => {
            let c = shapes::circle_three_point(
                sketch,
                req_vec2(args, "a")?,
                req_vec2(args, "b")?,
                req_vec2(args, "c")?,
            )?;
            json!({ "circle": entity_to_json(c.circle), "center": entity_to_json(c.center) })
        }
        "polygon" => {
            let sides = req_u64(args, "sides")? as usize;
            let center = req_vec2(args, "center")?;
            let vertex = match opt(args, "vertex") {
                Some(v) => vec2_of(v, "vertex")?,
                None => center + Vec2::X * req_f64(args, "radius")?,
            };
            let p = shapes::polygon_center(sketch, center, vertex, sides)?;
            json!({ "center": entity_to_json(p.center), "circle": entity_to_json(p.circle), "vertices": ids(&p.vertices), "edges": ids(&p.edges) })
        }
        "slot" => {
            let s = shapes::slot_center_to_center(
                sketch,
                req_vec2(args, "a")?,
                req_vec2(args, "b")?,
                req_f64(args, "width")?,
            );
            json!({ "centers": ids(&s.centers), "arcs": ids(&s.arcs), "lines": ids(&s.lines), "center_line": entity_to_json(s.center_line) })
        }
        "arc" => {
            let a = shapes::arc_center(
                sketch,
                req_vec2(args, "center")?,
                req_vec2(args, "start")?,
                req_vec2(args, "end")?,
            );
            json!({ "arc": entity_to_json(a.arc), "center": entity_to_json(a.center), "start": entity_to_json(a.start), "end": entity_to_json(a.end) })
        }
        "arc_three_point" => {
            let a = shapes::arc_three_point(
                sketch,
                req_vec2(args, "start")?,
                req_vec2(args, "mid")?,
                req_vec2(args, "end")?,
            )?;
            json!({ "arc": entity_to_json(a.arc), "center": entity_to_json(a.center), "start": entity_to_json(a.start), "end": entity_to_json(a.end) })
        }
        "arc_points" => {
            // An arc on existing points, which is how a corner is shared with the lines
            // beside it: counter-clockwise from start to end about center.
            let c = point_arg(sketch, field(args, "center")?, "center")?;
            let s = point_arg(sketch, field(args, "start")?, "start")?;
            let e = point_arg(sketch, field(args, "end")?, "end")?;
            let arc = sketch.add_arc(c, s, e)?;
            json!({ "arc": entity_to_json(arc) })
        }
        "circle_on" => {
            let c = point_arg(sketch, field(args, "center")?, "center")?;
            let circle = sketch.add_circle(c, req_f64(args, "radius")?)?;
            json!({ "circle": entity_to_json(circle), "center": entity_to_json(c) })
        }
        "construction" => {
            let value = opt_bool(args, "value")?.unwrap_or(true);
            let list = entity_list(sketch, args, "entities")?;
            for id in &list {
                sketch.set_construction(*id, value)?;
            }
            json!({ "changed": list.len() })
        }
        "remove" => {
            let list = entity_list(sketch, args, "entities")?;
            for id in &list {
                if sketch.entity(*id).is_none() {
                    return Err(ToolError::bad(format!("no entity {}", entity_to_json(*id))));
                }
                sketch.remove_entity(*id);
            }
            json!({ "removed": list.len() })
        }
        "remove_constraint" => {
            let id = constraint_arg(sketch, args)?;
            sketch.remove_constraint(id);
            json!({ "removed": 1 })
        }
        "constrain" => {
            let c = constraint_from_args(sketch, args)?;
            let id = sketch.add_constraint(c)?;
            json!({ "constraint": constraint_to_json(id) })
        }
        "set_dimension" => {
            let id = constraint_arg(sketch, args)?;
            let mut value = req_f64(args, "value")?;
            if matches!(sketch.constraint(id), Some(Constraint::Angle { .. })) {
                value = value.to_radians();
            }
            sketch.set_dimension_value(id, value)?;
            json!({ "constraint": constraint_to_json(id), "value": round(req_f64(args, "value")?) })
        }
        "bind_dimension" => {
            let id = constraint_arg(sketch, args)?;
            let value = sketch.bind_dimension_with(id, req_str(args, "expression")?, outer)?;
            json!({ "constraint": constraint_to_json(id), "value": round(value) })
        }
        "unbind_dimension" => {
            let id = constraint_arg(sketch, args)?;
            sketch.unbind_dimension(id);
            json!({ "constraint": constraint_to_json(id) })
        }
        "set_parameter" => {
            let name = req_str(args, "name")?;
            let value = sketch.set_parameter_with(name, req_str(args, "expression")?, outer)?;
            json!({ "name": name, "value": round(value) })
        }
        "remove_parameter" => {
            json!({ "removed": sketch.remove_parameter(req_str(args, "name")?) })
        }
        "move_point" => {
            let id = entity_arg(sketch, args, "point")?;
            sketch.set_point_pos(id, req_vec2(args, "to")?)?;
            json!({ "point": entity_to_json(id) })
        }
        "drag" => {
            let id = entity_arg(sketch, args, "point")?;
            let r = sketch.drag(id, req_vec2(args, "to")?)?;
            json!({ "point": entity_to_json(id), "at": sketch.point_pos(id).map(v2), "solve": report(&r) })
        }
        "trim" => {
            let curve = entity_arg(sketch, args, "curve")?;
            let kept = edit::trim(sketch, curve, req_vec2(args, "at")?)?;
            json!({ "kept": ids(&kept) })
        }
        "break" => {
            let curve = entity_arg(sketch, args, "curve")?;
            let pieces = edit::break_curve(sketch, curve)?;
            json!({ "pieces": ids(&pieces) })
        }
        "fillet" => {
            let a = entity_arg(sketch, args, "a")?;
            let b = entity_arg(sketch, args, "b")?;
            let radius = req_f64(args, "radius")?;
            // Without hints, the corner is the point the two curves share.
            let (hint_a, hint_b) = match (opt(args, "hint_a"), opt(args, "hint_b")) {
                (Some(ha), Some(hb)) => (vec2_of(ha, "hint_a")?, vec2_of(hb, "hint_b")?),
                _ => {
                    let corner = shared_point(sketch, a, b).ok_or_else(|| {
                        ToolError::bad("the curves share no point; pass hint_a and hint_b")
                    })?;
                    let at = sketch.point_pos(corner).unwrap_or(Vec2::ZERO);
                    let ha = fillet::hint_along(sketch, a, at).unwrap_or(at);
                    let hb = fillet::hint_along(sketch, b, at).unwrap_or(at);
                    (ha, hb)
                }
            };
            let f = fillet::fillet(sketch, a, hint_a, b, hint_b, radius)?;
            json!({ "arc": entity_to_json(f.arc), "center": entity_to_json(f.center), "start": entity_to_json(f.start), "end": entity_to_json(f.end) })
        }
        "offset" => {
            let seed = entity_list(sketch, args, "entities")?;
            let corner = match opt_str(args, "corner").unwrap_or("round") {
                "round" => offset::Corner::Round,
                "miter" | "mitre" => offset::Corner::Miter,
                other => {
                    return Err(ToolError::bad(format!(
                        "corner must be round or miter, got {other:?}"
                    )));
                }
            };
            let made = offset::offset(sketch, &seed, req_f64(args, "distance")?, corner)?;
            json!({ "entities": ids(&made) })
        }
        "pattern_rectangular" => {
            let seed = entity_list(sketch, args, "entities")?;
            let made = pattern::rectangular(
                sketch,
                &seed,
                req_vec2(args, "x_step")?,
                req_u64(args, "cols")? as usize,
                opt(args, "y_step")
                    .map(|v| vec2_of(v, "y_step"))
                    .transpose()?
                    .unwrap_or(Vec2::Y),
                opt(args, "rows")
                    .map(|v| {
                        v.as_u64()
                            .ok_or_else(|| ToolError::bad("rows must be an integer"))
                    })
                    .transpose()?
                    .unwrap_or(1) as usize,
            )?;
            json!({ "entities": ids(&made) })
        }
        "pattern_circular" => {
            let seed = entity_list(sketch, args, "entities")?;
            let angle = opt(args, "angle_deg")
                .map(|v| f64_of(v, "angle_deg"))
                .transpose()?
                .unwrap_or(360.0);
            let made = pattern::circular(
                sketch,
                &seed,
                req_vec2(args, "center")?,
                req_u64(args, "count")? as usize,
                angle.to_radians(),
            )?;
            json!({ "entities": ids(&made) })
        }
        "solve" => {
            let r = sketch.solve_with(outer)?;
            report(&r)
        }
        _ => {
            return Err(ToolError::bad(format!(
                "unknown op {name:?}; see the sketch_ops tool description for the list"
            )));
        }
    })
}

fn shared_point(sketch: &Sketch, a: EntityId, b: EntityId) -> Option<EntityId> {
    let pa = sketch.entity_points(a);
    let pb = sketch.entity_points(b);
    pa.into_iter().find(|p| pb.contains(p))
}

fn constraint_from_args(sketch: &Sketch, args: &Value) -> Result<Constraint, ToolError> {
    let kind = req_str(args, "type")?;
    let e = |name: &str| entity_arg(sketch, args, name);
    let value = || req_f64(args, "value");
    Ok(match kind.to_ascii_lowercase().as_str() {
        "coincident" => Constraint::Coincident {
            point: e("point")?,
            target: e("target")?,
        },
        "horizontal" => Constraint::Horizontal(e("entity")?),
        "vertical" => Constraint::Vertical(e("entity")?),
        "parallel" => Constraint::Parallel(e("a")?, e("b")?),
        "perpendicular" => Constraint::Perpendicular(e("a")?, e("b")?),
        "equal" => Constraint::Equal(e("a")?, e("b")?),
        "tangent" => Constraint::Tangent(e("a")?, e("b")?),
        "fix" => Constraint::Fix(e("entity")?),
        "midpoint" => Constraint::Midpoint {
            point: e("point")?,
            line: e("line")?,
        },
        "symmetric" => Constraint::Symmetric {
            a: e("a")?,
            b: e("b")?,
            axis: e("axis")?,
        },
        "concentric" => Constraint::Concentric(e("a")?, e("b")?),
        "distance" => Constraint::Distance {
            a: e("a")?,
            b: e("b")?,
            value: value()?,
        },
        "horizontal_distance" => Constraint::HorizontalDistance {
            a: e("a")?,
            b: e("b")?,
            value: value()?,
        },
        "vertical_distance" => Constraint::VerticalDistance {
            a: e("a")?,
            b: e("b")?,
            value: value()?,
        },
        "radius" => Constraint::Radius {
            curve: e("curve")?,
            value: value()?,
        },
        "diameter" => Constraint::Diameter {
            curve: e("curve")?,
            value: value()?,
        },
        "angle" => Constraint::Angle {
            a: e("a")?,
            b: e("b")?,
            value: value()?.to_radians(),
        },
        other => {
            return Err(ToolError::bad(format!(
                "unknown constraint type {other:?}; one of coincident, horizontal, vertical, parallel, perpendicular, equal, tangent, fix, midpoint, symmetric, concentric, distance, horizontal_distance, vertical_distance, radius, diameter, angle"
            )));
        }
    })
}
