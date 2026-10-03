//! JSON views of the model, rounded to what a reader can use.
//!
//! Numbers are rounded to six decimals: a millimetre model carries nothing meaningful
//! below a nanometre, and `0.30000000000000004` in a reply is noise an agent has to read
//! past. Everything an id names is spelled the way [`crate::ids`] spells it so a reply
//! can be quoted straight back into the next call.

use basset_core::{
    Body, BodyOp, BodyRef, Extent, Feature, FeatureId, FeatureKind, FeatureStatus, ModelState,
    RegionRef, SolvedSketch,
};
use basset_kernel::{Edge, Face, Solid, SurfaceKind};
use basset_math::{Aabb, Frame, Vec2, Vec3};
use basset_sketch::{Constraint, Entity, Sketch, SolveReport, Tessellation};
use serde_json::{Map, Value, json};

use crate::ids::{
    axis_to_json, constraint_to_json, edge_key_to_string, entity_to_json, face_key_to_string,
    face_ref_to_json, plane_to_json,
};

pub fn round(x: f64) -> f64 {
    (x * 1e6).round() / 1e6
}

pub fn v2(p: Vec2) -> Value {
    json!([round(p.x), round(p.y)])
}

pub fn v3(p: Vec3) -> Value {
    json!([round(p.x), round(p.y), round(p.z)])
}

pub fn aabb(b: &Aabb) -> Value {
    if b.is_empty() {
        Value::Null
    } else {
        json!({ "min": v3(b.min), "max": v3(b.max), "size": v3(b.extent()) })
    }
}

pub fn frame(f: &Frame) -> Value {
    json!({ "origin": v3(f.origin), "x": v3(f.x), "y": v3(f.y), "normal": v3(f.z) })
}

pub fn status(s: Option<&FeatureStatus>) -> Value {
    match s {
        None => json!({ "status": "not_evaluated" }),
        Some(FeatureStatus::Ok) => json!({ "status": "ok" }),
        Some(FeatureStatus::Suppressed) => json!({ "status": "suppressed" }),
        Some(FeatureStatus::Warned(m)) => json!({ "status": "warned", "message": m }),
        Some(FeatureStatus::Failed(m)) => json!({ "status": "failed", "message": m }),
    }
}

fn region(r: &RegionRef) -> Value {
    match r {
        RegionRef::Profile(p) => json!({ "sketch": p.sketch.0, "sample": v2(p.sample) }),
        RegionRef::Face(f) => face_ref_to_json(f),
    }
}

fn body_op(op: &BodyOp) -> Value {
    let targets = |b: &[BodyRef]| b.iter().map(|b| json!(b.0.0)).collect::<Vec<_>>();
    match op {
        BodyOp::NewBody => json!({ "operation": "new" }),
        BodyOp::Join(b) => json!({ "operation": "join", "targets": targets(b) }),
        BodyOp::Cut(b) => json!({ "operation": "cut", "targets": targets(b) }),
        BodyOp::Intersect(b) => json!({ "operation": "intersect", "targets": targets(b) }),
    }
}

pub fn extent(e: &Extent) -> Value {
    match e {
        Extent::OneSide(d) => json!({ "type": "one_side", "distance": round(*d) }),
        Extent::Symmetric(d) => json!({ "type": "symmetric", "distance": round(*d) }),
        Extent::TwoSides { positive, negative } => json!({
            "type": "two_sides", "positive": round(*positive), "negative": round(*negative)
        }),
        Extent::ToFace(f) => {
            json!({ "type": "to_face", "body": f.body.0.0, "face": face_key_to_string(f.key) })
        }
    }
}

/// The feature's inputs, without the sketch's geometry (that is `sketch_info`'s job).
pub fn feature(f: &Feature, state: &ModelState) -> Value {
    let mut out = Map::new();
    out.insert("id".into(), json!(f.id.0));
    out.insert("name".into(), json!(f.name));
    out.insert("kind".into(), json!(f.kind.default_name()));
    out.insert("suppressed".into(), json!(f.suppressed));
    let st = status(state.status(f.id));
    for (k, v) in st.as_object().into_iter().flatten() {
        out.insert(k.clone(), v.clone());
    }
    if !f.exprs.is_empty() {
        out.insert(
            "expressions".into(),
            Value::Object(
                f.exprs
                    .iter()
                    .map(|(k, v)| (k.label().to_string(), json!(v)))
                    .collect(),
            ),
        );
    }
    let deps: Vec<u64> = f.kind.dependencies().iter().map(|d| d.0).collect();
    if !deps.is_empty() {
        out.insert("depends_on".into(), json!(deps));
    }
    let inputs = match &f.kind {
        FeatureKind::NewComponent { name, parent } => json!({ "name": name, "parent": parent.0 }),
        FeatureKind::ComponentFromBody { body, name } => json!({ "body": body.0.0, "name": name }),
        FeatureKind::Sketch {
            plane,
            component,
            sketch,
        } => json!({
            "plane": plane_to_json(plane),
            "component": component.0,
            "entities": sketch.entities().count(),
            "constraints": sketch.constraints().count(),
        }),
        FeatureKind::OffsetPlane { base, distance } => {
            json!({ "base": plane_to_json(base), "distance": round(*distance) })
        }
        FeatureKind::AngledPlane { base, axis, angle } => json!({
            "base": plane_to_json(base), "axis": axis_to_json(axis), "angle_deg": round(angle.to_degrees())
        }),
        FeatureKind::Extrude {
            regions,
            extent: e,
            operation,
            component,
        } => json!({
            "regions": regions.iter().map(region).collect::<Vec<_>>(),
            "extent": extent(e),
            "body_op": body_op(operation),
            "component": component.0,
        }),
        FeatureKind::Revolve {
            regions,
            axis,
            angle,
            operation,
            component,
        } => json!({
            "regions": regions.iter().map(region).collect::<Vec<_>>(),
            "axis": axis_to_json(axis),
            "angle_deg": round(angle.to_degrees()),
            "body_op": body_op(operation),
            "component": component.0,
        }),
        FeatureKind::Sweep {
            regions,
            path,
            operation,
            component,
        } => json!({
            "regions": regions.iter().map(region).collect::<Vec<_>>(),
            "path": { "sketch": path.sketch.0, "curves": path.curves.iter().map(|c| entity_to_json(*c)).collect::<Vec<_>>() },
            "body_op": body_op(operation),
            "component": component.0,
        }),
        FeatureKind::Loft {
            regions,
            operation,
            component,
        } => json!({
            "regions": regions.iter().map(region).collect::<Vec<_>>(),
            "body_op": body_op(operation),
            "component": component.0,
        }),
        FeatureKind::Fillet { edges, radius } => json!({
            "edges": edges.iter().map(|e| json!({ "body": e.body.0.0, "edge": edge_key_to_string(e.key) })).collect::<Vec<_>>(),
            "radius": round(*radius),
        }),
        FeatureKind::Chamfer { edges, distance } => json!({
            "edges": edges.iter().map(|e| json!({ "body": e.body.0.0, "edge": edge_key_to_string(e.key) })).collect::<Vec<_>>(),
            "distance": round(*distance),
        }),
        FeatureKind::Combine {
            target,
            tools,
            operation,
            keep_tools,
        } => json!({
            "target": target.0.0,
            "tools": tools.iter().map(|t| json!(t.0.0)).collect::<Vec<_>>(),
            "operation": format!("{operation:?}").to_lowercase(),
            "keep_tools": keep_tools,
        }),
        FeatureKind::Move { body, transform } => json!({
            "body": body.0.0,
            "translation": v3(transform.translation),
        }),
    };
    out.insert("inputs".into(), inputs);
    if f.kind.creates_body() {
        out.insert("body".into(), json!(f.id.0));
        out.insert("body_name".into(), json!(f.body_name()));
    }
    Value::Object(out)
}

pub fn surface(s: &SurfaceKind) -> Value {
    match s {
        SurfaceKind::Planar { normal } => json!({ "type": "planar", "normal": v3(*normal) }),
        SurfaceKind::Cylindrical {
            origin,
            axis,
            radius,
        } => json!({
            "type": "cylindrical", "origin": v3(*origin), "axis": v3(*axis), "radius": round(*radius)
        }),
        SurfaceKind::Conical {
            apex,
            axis,
            half_angle,
        } => json!({
            "type": "conical", "apex": v3(*apex), "axis": v3(*axis), "half_angle_deg": round(half_angle.to_degrees())
        }),
        SurfaceKind::Freeform => json!({ "type": "freeform" }),
    }
}

pub fn face(f: &Face) -> Value {
    json!({
        "key": face_key_to_string(f.key),
        "surface": surface(&f.surface),
        "area": round(f.area()),
        "centroid": v3(f.centroid()),
        "polygons": f.polygons.len(),
    })
}

pub fn edge(e: &Edge) -> Value {
    let first = e.segments.first();
    let last = e.segments.last();
    let mut out = json!({
        "key": edge_key_to_string(e.key),
        "length": round(e.length()),
        "segments": e.segments.len(),
        "smooth": e.smooth,
        "drawn": e.drawn,
    });
    if let (Some(a), Some(b)) = (first, last) {
        out["start"] = v3(a.start);
        out["end"] = v3(b.end);
        let mid = e.segments[e.segments.len() / 2];
        out["midpoint"] = v3((mid.start + mid.end) * 0.5);
    }
    if let Some(dir) = basset_kernel::edge_direction(e) {
        out["direction"] = v3(dir);
        out["shape"] = json!("line");
    } else if let Some(c) = basset_kernel::edge_circle(e) {
        out["shape"] = json!("circle");
        out["center"] = v3(c.center);
        out["axis"] = v3(c.axis);
        out["radius"] = json!(round(c.radius));
    } else {
        out["shape"] = json!("polyline");
    }
    out
}

/// Bulk properties of a solid plus whether its shell is sound: the numbers a kernel test
/// asserts on, because they catch winding, orientation and boolean errors cheaply.
pub fn solid(s: &Solid) -> Value {
    let validity = s.validate();
    let unmatched = s.unmatched_edges();
    json!({
        "volume": round(s.volume()),
        "surface_area": round(s.surface_area()),
        "centroid": v3(s.centroid()),
        "aabb": aabb(&s.aabb()),
        "faces": s.faces.len(),
        "polygons": s.polygon_count(),
        "closed": validity.is_ok(),
        "validation": validity.err().map(|e| e.to_string()),
        "unmatched_edges": unmatched.len(),
    })
}

pub fn body(b: &Body) -> Value {
    let mut out = solid(&b.solid);
    out["id"] = json!(b.id.0.0);
    out["name"] = json!(b.name);
    out["component"] = json!(b.component.0);
    out
}

pub fn report(r: &SolveReport) -> Value {
    json!({
        "converged": r.converged,
        "iterations": r.iterations,
        "residual": r.residual,
        "degrees_of_freedom": r.degrees_of_freedom,
        "under_constrained": r.under_constrained.iter().map(|e| entity_to_json(*e)).collect::<Vec<_>>(),
        "redundant": r.redundant.iter().map(|c| constraint_to_json(*c)).collect::<Vec<_>>(),
    })
}

pub fn entity(sketch: &Sketch, id: basset_sketch::EntityId) -> Option<Value> {
    let data = sketch.entity(id)?;
    let pt = |p| sketch.point_pos(p).map(v2).unwrap_or(Value::Null);
    let mut out = match &data.entity {
        Entity::Point { pos } => json!({ "kind": "point", "at": v2(*pos) }),
        Entity::Line { start, end } => json!({
            "kind": "line", "start": entity_to_json(*start), "end": entity_to_json(*end),
            "from": pt(*start), "to": pt(*end),
        }),
        Entity::Circle { center, radius } => json!({
            "kind": "circle", "center": entity_to_json(*center), "at": pt(*center), "radius": round(*radius),
        }),
        Entity::Arc { center, start, end } => json!({
            "kind": "arc", "center": entity_to_json(*center), "start": entity_to_json(*start), "end": entity_to_json(*end),
            "center_at": pt(*center), "from": pt(*start), "to": pt(*end),
            "radius": sketch.point_pos(*center).zip(sketch.point_pos(*start)).map(|(c, s)| round(c.distance(s))),
        }),
        Entity::Text {
            anchor,
            text,
            height,
            angle,
        } => json!({
            "kind": "text", "anchor": entity_to_json(*anchor), "text": text, "height": round(*height), "angle_deg": round(angle.to_degrees()),
        }),
    };
    out["id"] = entity_to_json(id);
    if data.construction {
        out["construction"] = json!(true);
    }
    Some(out)
}

pub fn constraint(c: &Constraint) -> Value {
    let e = |id: &basset_sketch::EntityId| entity_to_json(*id);
    match c {
        Constraint::Coincident { point, target } => {
            json!({ "type": "coincident", "point": e(point), "target": e(target) })
        }
        Constraint::Horizontal(a) => json!({ "type": "horizontal", "entity": e(a) }),
        Constraint::Vertical(a) => json!({ "type": "vertical", "entity": e(a) }),
        Constraint::Parallel(a, b) => json!({ "type": "parallel", "a": e(a), "b": e(b) }),
        Constraint::Perpendicular(a, b) => json!({ "type": "perpendicular", "a": e(a), "b": e(b) }),
        Constraint::Equal(a, b) => json!({ "type": "equal", "a": e(a), "b": e(b) }),
        Constraint::Tangent(a, b) => json!({ "type": "tangent", "a": e(a), "b": e(b) }),
        Constraint::Fix(a) => json!({ "type": "fix", "entity": e(a) }),
        Constraint::Midpoint { point, line } => {
            json!({ "type": "midpoint", "point": e(point), "line": e(line) })
        }
        Constraint::Symmetric { a, b, axis } => {
            json!({ "type": "symmetric", "a": e(a), "b": e(b), "axis": e(axis) })
        }
        Constraint::Concentric(a, b) => json!({ "type": "concentric", "a": e(a), "b": e(b) }),
        Constraint::Distance { a, b, value } => {
            json!({ "type": "distance", "a": e(a), "b": e(b), "value": round(*value) })
        }
        Constraint::HorizontalDistance { a, b, value } => {
            json!({ "type": "horizontal_distance", "a": e(a), "b": e(b), "value": round(*value) })
        }
        Constraint::VerticalDistance { a, b, value } => {
            json!({ "type": "vertical_distance", "a": e(a), "b": e(b), "value": round(*value) })
        }
        Constraint::Radius { curve, value } => {
            json!({ "type": "radius", "curve": e(curve), "value": round(*value) })
        }
        Constraint::Diameter { curve, value } => {
            json!({ "type": "diameter", "curve": e(curve), "value": round(*value) })
        }
        Constraint::Angle { a, b, value } => {
            json!({ "type": "angle", "a": e(a), "b": e(b), "value_deg": round(value.to_degrees()) })
        }
        Constraint::Offset { pairs, value } => json!({
            "type": "offset", "value": round(*value),
            "pairs": pairs.iter().map(|p| json!({ "source": e(&p.source), "result": e(&p.result) })).collect::<Vec<_>>(),
        }),
    }
}

/// The closed regions of a solved sketch, each with a point inside it that an extrude
/// can name it by. Indexed in the order the regenerator produces them.
pub fn regions(solved: &Sketch) -> Vec<Value> {
    let tess = Tessellation::default();
    solved
        .profiles(&tess)
        .iter()
        .enumerate()
        .map(|(i, p)| {
            let (mut lo, mut hi) = (Vec2::splat(f64::INFINITY), Vec2::splat(f64::NEG_INFINITY));
            for pt in &p.outer.points {
                lo = lo.min(*pt);
                hi = hi.max(*pt);
            }
            let mut curves: Vec<Value> = Vec::new();
            for seg in p
                .outer
                .segments
                .iter()
                .chain(p.holes.iter().flat_map(|h| h.segments.iter()))
            {
                let id = entity_to_json(seg.curve);
                if !curves.contains(&id) {
                    curves.push(id);
                }
            }
            json!({
                "index": i,
                "area": round(p.area()),
                "sample": p.interior_point().map(v2),
                "bbox": { "min": v2(lo), "max": v2(hi) },
                "holes": p.holes.len(),
                "outer_points": p.outer.points.len(),
                "curves": curves,
            })
        })
        .collect()
}

pub fn sketch_info(
    id: FeatureId,
    stored: &Sketch,
    solved: Option<&SolvedSketch>,
    st: Option<&FeatureStatus>,
) -> Value {
    // The solved copy is what the regenerator built from; it holds the positions after
    // constraints were applied, which is what a caller needs to pick points from.
    let geometry: &Sketch = solved.map(|s| &s.sketch).unwrap_or(stored);
    let mut entities: Vec<Value> = geometry
        .entities()
        .filter_map(|(id, _)| entity(geometry, id))
        .collect();
    entities.sort_by_key(|e| e["id"].as_u64());
    let constraints: Vec<Value> = geometry
        .constraints()
        .map(|(cid, c)| {
            let mut v = constraint(c);
            v["id"] = constraint_to_json(cid);
            if let Some(expr) = geometry.dimension_expr(cid) {
                v["expression"] = json!(expr);
            }
            v
        })
        .collect();
    let parameters: Vec<Value> = geometry
        .parameters()
        .iter()
        .map(|p| json!({ "name": p.name, "expression": p.expr }))
        .collect();
    let mut out = json!({
        "sketch": id.0,
        "entities": entities,
        "constraints": constraints,
        "parameters": parameters,
    });
    for (k, v) in status(st).as_object().into_iter().flatten() {
        out[k] = v.clone();
    }
    if let Some(s) = solved {
        out["plane"] = frame(&s.frame);
        out["solve"] = report(&s.report);
        out["regions"] = Value::Array(regions(&s.sketch));
    }
    out
}
