//! The tool catalogue and what each tool does.
//!
//! Every tool is a function of `(&mut Session, &Value) -> Result<Value, ToolError>`,
//! listed once in [`tool_definitions`] with its JSON schema and once in [`call`]. The
//! schemas are deliberately loose about optional fields and strict about the ones a
//! call cannot do without; the error messages do the rest.

use std::path::PathBuf;

use basset_core::{
    BodyOp, BodyRef, CombineOp, ComponentId, Document, EdgeRef, Extent, FeatureId, FeatureKind,
    NumericField, ProfileRef, RegionRef, file,
};
use basset_io::{ExportItem, Unit};
use basset_math::{Affine3, Vec2};
use basset_sketch::Sketch;
use serde_json::{Value, json};

use crate::args::{
    field, opt, opt_bool, opt_f64, opt_str, opt_u64, req_array, req_f64, req_str, req_u64,
    req_vec3, vec2_of,
};
use crate::ids::{
    axis_from_json, edge_key_from_str, edge_key_to_string, face_key_from_str, face_ref_from_json,
    feature_from_json, plane_from_json,
};
use crate::summary::{self, round};
use crate::{Session, ToolError};

pub const SERVER_INSTRUCTIONS: &str = "Basset is a parametric, history-based solid modeller. \
Build a model the way its editor does: create_sketch on a plane, draw into it with sketch_ops, \
read sketch_info to find the closed regions (each has a 'sample' point inside it), then extrude \
those regions by sample point. Inspect results with body_info and check_document; both report \
whether every shell is closed and which features failed or warned. Units are millimetres; \
angles cross this interface in degrees. Entity and constraint ids are the integers the replies \
quote; faces are 'feature.sub:Role' strings and edges are two of those joined by '|'.";

fn tool(name: &str, description: &str, schema: Value) -> Value {
    json!({ "name": name, "description": description, "inputSchema": schema })
}

fn obj(properties: Value, required: &[&str]) -> Value {
    json!({ "type": "object", "properties": properties, "required": required })
}

pub fn tool_definitions() -> Vec<Value> {
    let plane = json!({ "description": "\"XY\" | \"YZ\" | \"XZ\" | plane feature id | {\"body\": id, \"face\": key}" });
    let regions = json!({
        "description": "Which regions to use: \"all\" (every closed region of the sketch), an array of region indexes from sketch_info, an array of [x, y] sample points in sketch coordinates, or an array of {\"body\", \"face\"} planar faces.",
    });
    let body_op = json!({
        "operation": { "type": "string", "enum": ["new", "join", "cut", "intersect"], "default": "new" },
        "targets": { "type": "array", "items": { "type": "integer" }, "description": "Body ids a join/cut/intersect applies to. Defaults to every body in the model." },
        "component": { "type": "integer", "description": "Component id, default root (0)." },
        "name": { "type": "string" },
    });
    vec![
        tool(
            "new_document",
            "Start an empty document, discarding the current one.",
            obj(json!({ "name": { "type": "string" } }), &[]),
        ),
        tool(
            "open_document",
            "Open a .bass file as the current document.",
            obj(json!({ "path": { "type": "string" } }), &["path"]),
        ),
        tool(
            "save_document",
            "Save the current document as .bass. Without a path, saves where it was opened from.",
            obj(json!({ "path": { "type": "string" } }), &[]),
        ),
        tool(
            "document_info",
            "The timeline (every feature with its status), bodies, parameters and components at the cursor.",
            obj(json!({}), &[]),
        ),
        tool(
            "check_document",
            "Regenerate and report everything wrong: failed and warned features, bodies whose shell is not closed, under-constrained sketches. Empty 'problems' means the model is sound.",
            obj(json!({}), &[]),
        ),
        tool(
            "create_sketch",
            "Add an empty sketch feature on a plane. Returns the sketch's feature id to draw into with sketch_ops.",
            obj(
                json!({ "plane": plane, "name": { "type": "string" }, "component": { "type": "integer" } }),
                &["plane"],
            ),
        ),
        tool(
            "sketch_ops",
            concat!(
                "Apply drawing operations to a sketch, all or nothing. Each op is {\"op\": name, ...}. ",
                "Point arguments ('from', 'to', 'center', 'start', 'end' of line/arc_points/circle_on) take an entity id to share an existing point or [x, y] for a new one. Ops: ",
                "point{at,construction?}; line{from,to,construction?}; polyline{points:[[x,y]...],closed?}; rectangle{a,b}; rectangle_center{center,corner}; ",
                "circle{center,radius}; circle_two_point{a,b}; circle_three_point{a,b,c}; circle_on{center(point),radius}; polygon{center,sides,radius|vertex}; slot{a,b,width}; ",
                "arc{center,start,end} (ccw); arc_three_point{start,mid,end}; arc_points{center,start,end} (existing points, ccw); ",
                "construction{entities,value?}; remove{entities}; remove_constraint{constraint}; ",
                "constrain{type,...} with type one of coincident{point,target} horizontal{entity} vertical{entity} parallel{a,b} perpendicular{a,b} equal{a,b} tangent{a,b} fix{entity} midpoint{point,line} symmetric{a,b,axis} concentric{a,b} distance{a,b,value} horizontal_distance{a,b,value} vertical_distance{a,b,value} radius{curve,value} diameter{curve,value} angle{a,b,value(deg)}; ",
                "set_dimension{constraint,value}; bind_dimension{constraint,expression}; unbind_dimension{constraint}; set_parameter{name,expression}; remove_parameter{name}; ",
                "move_point{point,to}; drag{point,to}; trim{curve,at}; break{curve}; fillet{a,b,radius,hint_a?,hint_b?}; offset{entities,distance,corner?:round|miter}; ",
                "pattern_rectangular{entities,x_step,cols,y_step?,rows?}; pattern_circular{entities,center,count,angle_deg?}; solve{}. ",
                "Returns each op's result and the sketch's solve report and regions afterwards."
            ),
            obj(
                json!({ "sketch": { "type": "integer" }, "ops": { "type": "array", "items": { "type": "object" } } }),
                &["sketch", "ops"],
            ),
        ),
        tool(
            "sketch_info",
            "A sketch's solved entities, constraints, solve report (degrees of freedom, loose entities, redundant constraints) and closed regions with a sample point inside each.",
            obj(json!({ "sketch": { "type": "integer" } }), &["sketch"]),
        ),
        tool(
            "extrude",
            "Push sketch regions (or planar faces) along their plane normal into a new body or a boolean on existing ones. Give either 'distance' (one side, negative goes the other way) or 'extent'.",
            obj(
                json!({
                    "sketch": { "type": "integer", "description": "Needed when regions are indexes, sample points or \"all\"." },
                    "regions": regions,
                    "distance": { "type": "number" },
                    "extent": { "description": "{\"type\":\"one_side\",\"distance\"} | {\"type\":\"symmetric\",\"distance\"} | {\"type\":\"two_sides\",\"positive\",\"negative\"} | {\"type\":\"to_face\",\"body\",\"face\"}" },
                    "operation": body_op["operation"], "targets": body_op["targets"], "component": body_op["component"], "name": body_op["name"],
                }),
                &["regions"],
            ),
        ),
        tool(
            "revolve",
            "Spin sketch regions about an axis.",
            obj(
                json!({
                    "sketch": { "type": "integer" }, "regions": regions,
                    "axis": { "description": "\"X\" | \"Y\" | \"Z\" | {\"sketch\": id, \"line\": entity}" },
                    "angle_deg": { "type": "number", "default": 360 },
                    "operation": body_op["operation"], "targets": body_op["targets"], "component": body_op["component"], "name": body_op["name"],
                }),
                &["regions", "axis"],
            ),
        ),
        tool(
            "fillet",
            "Round edges of a body. Edge keys come from body_info.",
            obj(
                json!({ "body": { "type": "integer" }, "edges": { "type": "array", "items": { "type": "string" } }, "radius": { "type": "number" }, "name": { "type": "string" } }),
                &["body", "edges", "radius"],
            ),
        ),
        tool(
            "chamfer",
            "Bevel edges of a body.",
            obj(
                json!({ "body": { "type": "integer" }, "edges": { "type": "array", "items": { "type": "string" } }, "distance": { "type": "number" }, "name": { "type": "string" } }),
                &["body", "edges", "distance"],
            ),
        ),
        tool(
            "combine",
            "Boolean of whole bodies: target (join|cut|intersect) tools.",
            obj(
                json!({ "target": { "type": "integer" }, "tools": { "type": "array", "items": { "type": "integer" } }, "operation": { "type": "string", "enum": ["join", "cut", "intersect"] }, "keep_tools": { "type": "boolean", "default": false } }),
                &["target", "tools", "operation"],
            ),
        ),
        tool(
            "offset_plane",
            "A construction plane parallel to a base plane.",
            obj(
                json!({ "base": plane, "distance": { "type": "number" }, "name": { "type": "string" } }),
                &["base", "distance"],
            ),
        ),
        tool(
            "angled_plane",
            "A construction plane turned about an axis from a base plane.",
            obj(
                json!({ "base": plane, "axis": { "description": "\"X\" | \"Y\" | \"Z\" | {\"sketch\", \"line\"}" }, "angle_deg": { "type": "number" }, "name": { "type": "string" } }),
                &["base", "axis", "angle_deg"],
            ),
        ),
        tool(
            "move_body",
            "Translate a body.",
            obj(
                json!({ "body": { "type": "integer" }, "translation": { "type": "array", "items": { "type": "number" }, "minItems": 3, "maxItems": 3 } }),
                &["body", "translation"],
            ),
        ),
        tool(
            "edit_feature",
            "Change a feature after the fact: its driven numbers (distance, negative, angle_deg, radius), extent, name, suppression; or remove it or move it to another index in the timeline.",
            obj(
                json!({
                    "feature": { "type": "integer" },
                    "distance": { "type": "number" }, "negative": { "type": "number" }, "angle_deg": { "type": "number" }, "radius": { "type": "number" },
                    "extent": { "description": "As for extrude." },
                    "name": { "type": "string" }, "suppressed": { "type": "boolean" },
                    "remove": { "type": "boolean" }, "move_to": { "type": "integer" },
                }),
                &["feature"],
            ),
        ),
        tool(
            "set_cursor",
            "Roll the timeline back or forward: the number of features that are active.",
            obj(json!({ "cursor": { "type": "integer" } }), &["cursor"]),
        ),
        tool("undo", "Undo the last document edit.", obj(json!({}), &[])),
        tool("redo", "Redo the last undone edit.", obj(json!({}), &[])),
        tool(
            "body_info",
            "A body's volume, surface area, bounding box, shell validity, and optionally its faces and edges with keys, surface kinds, areas, centroids and midpoints.",
            obj(
                json!({
                    "body": { "type": "integer" },
                    "faces": { "type": "boolean", "default": false },
                    "edges": { "type": "boolean", "default": false },
                    "touching_face": { "type": "string", "description": "Only edges bordering this face key." },
                }),
                &["body"],
            ),
        ),
        tool(
            "set_parameter",
            "Add or change a document parameter (an expression over other parameters).",
            obj(
                json!({ "name": { "type": "string" }, "expression": { "type": "string" } }),
                &["name", "expression"],
            ),
        ),
        tool(
            "remove_parameter",
            "Delete a document parameter.",
            obj(json!({ "name": { "type": "string" } }), &["name"]),
        ),
        tool(
            "set_feature_expr",
            "Drive a feature's number (distance|negative|angle|radius) by an expression; an empty expression releases it.",
            obj(
                json!({ "feature": { "type": "integer" }, "field": { "type": "string" }, "expression": { "type": "string" } }),
                &["feature", "field", "expression"],
            ),
        ),
        tool(
            "export",
            "Write bodies to an STL or 3MF file.",
            obj(
                json!({ "path": { "type": "string" }, "format": { "type": "string", "enum": ["stl", "3mf"] }, "bodies": { "type": "array", "items": { "type": "integer" }, "description": "Default: every body." } }),
                &["path"],
            ),
        ),
        tool(
            "run_script",
            "Run several tool calls in order: [{\"tool\": name, \"arguments\": {...}}, ...]. An argument string \"$N\" is replaced by the 'feature' of call N's result (\"$N.body\" for another field), and \"${N}\" inside a longer string is substituted as text (\"${2}.0:EndCap\" names a face of the body call 2 made), so later calls can name what earlier ones made. Stops at the first error unless continue_on_error.",
            obj(
                json!({ "calls": { "type": "array", "items": { "type": "object" } }, "continue_on_error": { "type": "boolean" } }),
                &["calls"],
            ),
        ),
    ]
}

pub fn call(session: &mut Session, name: &str, args: &Value) -> Result<Value, ToolError> {
    match name {
        "new_document" => new_document(session, args),
        "open_document" => open_document(session, args),
        "save_document" => save_document(session, args),
        "document_info" => Ok(document_info(session.document_mut())),
        "check_document" => Ok(check_document(session.document_mut())),
        "create_sketch" => create_sketch(session, args),
        "sketch_ops" => sketch_ops(session, args),
        "sketch_info" => {
            let id = feature_from_json(field(args, "sketch")?)?;
            sketch_info(session.document_mut(), id)
        }
        "extrude" => extrude(session, args),
        "revolve" => revolve(session, args),
        "fillet" => blend(session, args, true),
        "chamfer" => blend(session, args, false),
        "combine" => combine(session, args),
        "offset_plane" => offset_plane(session, args),
        "angled_plane" => angled_plane(session, args),
        "move_body" => move_body(session, args),
        "edit_feature" => edit_feature(session, args),
        "set_cursor" => {
            let cursor = req_u64(args, "cursor")? as usize;
            let doc = session.document_mut();
            if cursor > doc.timeline().len() {
                return Err(ToolError::bad(format!(
                    "cursor {cursor} is past the end of the timeline ({} features)",
                    doc.timeline().len()
                )));
            }
            doc.set_cursor(cursor);
            Ok(check_document(doc))
        }
        "undo" => {
            let done = session.document_mut().undo();
            Ok(json!({ "undone": done }))
        }
        "redo" => {
            let done = session.document_mut().redo();
            Ok(json!({ "redone": done }))
        }
        "body_info" => body_info(session, args),
        "set_parameter" => {
            let value = session
                .document_mut()
                .set_parameter(req_str(args, "name")?, req_str(args, "expression")?)?;
            Ok(
                json!({ "name": req_str(args, "name")?, "value": round(value), "check": check_document(session.document_mut()) }),
            )
        }
        "remove_parameter" => {
            let removed = session
                .document_mut()
                .remove_parameter(req_str(args, "name")?);
            Ok(json!({ "removed": removed }))
        }
        "set_feature_expr" => set_feature_expr(session, args),
        "export" => export(session, args),
        "run_script" => run_script(session, args),
        _ => Err(ToolError::bad(format!("unknown tool {name:?}"))),
    }
}

// ----- documents ------------------------------------------------------------------------

fn new_document(session: &mut Session, args: &Value) -> Result<Value, ToolError> {
    let name = opt_str(args, "name").unwrap_or("Untitled");
    session.replace_document(Document::new(name), None);
    Ok(json!({ "name": name }))
}

fn open_document(session: &mut Session, args: &Value) -> Result<Value, ToolError> {
    let path = PathBuf::from(req_str(args, "path")?);
    let doc = file::load(&path)?;
    session.replace_document(doc, Some(path.clone()));
    let mut info = document_info(session.document_mut());
    info["path"] = json!(path.display().to_string());
    Ok(info)
}

fn save_document(session: &mut Session, args: &Value) -> Result<Value, ToolError> {
    let path = match opt_str(args, "path") {
        Some(p) => PathBuf::from(p),
        None => session
            .path()
            .cloned()
            .ok_or_else(|| ToolError::bad("the document has no path yet; give one"))?,
    };
    let path = if path.extension().is_none() {
        path.with_extension(file::EXTENSION)
    } else {
        path
    };
    file::save(&path, session.document())?;
    let doc = session.document().clone();
    session.replace_document(doc, Some(path.clone()));
    Ok(json!({ "saved": path.display().to_string() }))
}

pub fn document_info(doc: &mut Document) -> Value {
    let name = doc.name.clone();
    let cursor = doc.timeline().cursor();
    let features: Vec<_> = doc.timeline().features().to_vec();
    let parameters: Vec<Value> = doc
        .parameters()
        .rows()
        .iter()
        .map(|p| json!({ "name": p.name, "expression": p.expr, "value": doc.parameters().value(&p.name).ok().map(round) }))
        .collect();
    let state = doc.state();
    json!({
        "name": name,
        "cursor": cursor,
        "features": features.iter().map(|f| summary::feature(f, state)).collect::<Vec<_>>(),
        "bodies": state.bodies.values().map(summary::body).collect::<Vec<_>>(),
        "components": state.components.values().map(|c| json!({ "id": c.id.0, "name": c.name, "parent": c.parent.map(|p| p.0) })).collect::<Vec<_>>(),
        "planes": state.planes.iter().map(|(id, f)| json!({ "feature": id.0, "frame": summary::frame(f) })).collect::<Vec<_>>(),
        "parameters": parameters,
    })
}

/// Everything the regenerator and the kernel can say is wrong with the model.
pub fn check_document(doc: &mut Document) -> Value {
    let features: Vec<_> = doc.timeline().active().to_vec();
    let state = doc.state();
    let mut problems = Vec::new();
    for f in &features {
        match state.status(f.id) {
            Some(basset_core::FeatureStatus::Failed(m)) => problems.push(json!({
                "feature": f.id.0, "name": f.name, "kind": f.kind.default_name(), "severity": "error", "message": m
            })),
            Some(basset_core::FeatureStatus::Warned(m)) => problems.push(json!({
                "feature": f.id.0, "name": f.name, "kind": f.kind.default_name(), "severity": "warning", "message": m
            })),
            _ => {}
        }
    }
    for body in state.bodies.values() {
        if let Err(e) = body.solid.validate() {
            let open = body.solid.unmatched_edges();
            problems.push(json!({
                "body": body.id.0.0, "name": body.name, "severity": "error",
                "message": format!("shell is not closed: {e}"),
                "unmatched_edges": open.iter().take(8).map(|[a, b]| json!([summary::v3(*a), summary::v3(*b)])).collect::<Vec<_>>(),
            }));
        }
        if body.solid.volume() <= 0.0 {
            problems.push(json!({
                "body": body.id.0.0, "name": body.name, "severity": "error",
                "message": format!("volume is {:.6}, so the shell is inside out or degenerate", body.solid.volume()),
            }));
        }
    }
    for (id, s) in &state.sketches {
        let r = &s.report;
        if !r.converged {
            problems.push(json!({ "feature": id.0, "severity": "error", "message": format!("sketch did not converge (residual {:.3e})", r.residual) }));
        }
        if !r.redundant.is_empty() {
            problems.push(json!({ "feature": id.0, "severity": "info", "message": format!("{} redundant constraint(s)", r.redundant.len()), "constraints": r.redundant.iter().map(|c| crate::ids::constraint_to_json(*c)).collect::<Vec<_>>() }));
        }
    }
    json!({
        "ok": problems.iter().all(|p| p["severity"] != "error"),
        "problems": problems,
        "bodies": state.bodies.values().map(|b| json!({ "id": b.id.0.0, "name": b.name, "volume": round(b.solid.volume()), "closed": b.solid.is_closed() })).collect::<Vec<_>>(),
    })
}

// ----- sketches -------------------------------------------------------------------------

fn component_arg(args: &Value) -> Result<ComponentId, ToolError> {
    Ok(opt_u64(args, "component")?
        .map(ComponentId)
        .unwrap_or(ComponentId::ROOT))
}

fn create_sketch(session: &mut Session, args: &Value) -> Result<Value, ToolError> {
    let plane = plane_from_json(field(args, "plane")?)?;
    let component = component_arg(args)?;
    let name = opt_str(args, "name").map(String::from);
    let doc = session.document_mut();
    let id = doc.add_named_feature(
        FeatureKind::Sketch {
            plane,
            component,
            sketch: Sketch::new(),
        },
        name,
    );
    let st = doc.state().status(id).cloned();
    let frame = doc
        .state()
        .sketches
        .get(&id)
        .map(|s| summary::frame(&s.frame));
    let mut out = json!({ "feature": id.0, "sketch": id.0, "plane": frame });
    for (k, v) in summary::status(st.as_ref())
        .as_object()
        .into_iter()
        .flatten()
    {
        out[k] = v.clone();
    }
    Ok(out)
}

fn stored_sketch(doc: &Document, id: FeatureId) -> Result<Sketch, ToolError> {
    match doc.timeline().get(id).map(|f| &f.kind) {
        Some(FeatureKind::Sketch { sketch, .. }) => Ok(sketch.clone()),
        Some(k) => Err(ToolError::bad(format!(
            "feature {} is a {}, not a sketch",
            id.0,
            k.default_name()
        ))),
        None => Err(ToolError::bad(format!("no feature {}", id.0))),
    }
}

fn sketch_ops(session: &mut Session, args: &Value) -> Result<Value, ToolError> {
    let id = feature_from_json(field(args, "sketch")?)?;
    let ops = req_array(args, "ops")?;
    let doc = session.document_mut();
    // Work on a copy so a batch that fails halfway leaves the timeline untouched.
    let mut working = stored_sketch(doc, id)?;
    let results = {
        let lookup = doc.parameters().lookup();
        crate::sketch::apply_ops(&mut working, &lookup, ops)?
    };
    doc.edit_sketch(id, |s, _| *s = working)?;
    let mut info = sketch_info(doc, id)?;
    info["results"] = Value::Array(results);
    // The full entity list is what sketch_info is for; after a batch the caller mostly
    // wants the ids it just made and whether the regions it expects exist.
    info.as_object_mut().map(|o| o.remove("entities"));
    info.as_object_mut().map(|o| o.remove("constraints"));
    Ok(info)
}

fn sketch_info(doc: &mut Document, id: FeatureId) -> Result<Value, ToolError> {
    let stored = stored_sketch(doc, id)?;
    let state = doc.state();
    let solved = state.sketches.get(&id).map(|s| s.as_ref());
    Ok(summary::sketch_info(id, &stored, solved, state.status(id)))
}

// ----- generators -----------------------------------------------------------------------

/// Reads the `regions` argument into references, sampling the sketch where needed.
fn regions_arg(doc: &mut Document, args: &Value) -> Result<Vec<RegionRef>, ToolError> {
    let spec = field(args, "regions")?;
    let sketch = opt(args, "sketch").map(feature_from_json).transpose()?;
    let need_sketch = || {
        sketch.ok_or_else(|| {
            ToolError::bad("\"sketch\" is needed to name regions by index, sample point or \"all\"")
        })
    };
    // The regenerator's regions, with a point inside each, in its order.
    // Each region as a point inside it and the signature of the curves around it, in
    // the regenerator's order.
    let samples = |doc: &mut Document, sketch: FeatureId| -> Result<Vec<(Vec2, u64)>, ToolError> {
        let state = doc.state();
        let solved = state
            .sketches
            .get(&sketch)
            .ok_or_else(|| match state.status(sketch) {
                Some(basset_core::FeatureStatus::Failed(m)) => {
                    ToolError::bad(format!("sketch {} failed: {m}", sketch.0))
                }
                _ => ToolError::bad(format!(
                    "feature {} is not a sketch at the timeline cursor",
                    sketch.0
                )),
            })?;
        let tess = basset_sketch::Tessellation::default();
        let profiles = solved.sketch.profiles(&tess);
        profiles
            .iter()
            .enumerate()
            .map(|(i, p)| {
                let sample = p
                    .interior_point()
                    .ok_or_else(|| ToolError::bad(format!("region {i} has no interior point")))?;
                Ok((sample, p.signature()))
            })
            .collect()
    };
    match spec {
        Value::String(s) if s == "all" => {
            let sketch = need_sketch()?;
            let points = samples(doc, sketch)?;
            if points.is_empty() {
                return Err(ToolError::bad(format!(
                    "sketch {} encloses no region; check sketch_info",
                    sketch.0
                )));
            }
            Ok(points
                .into_iter()
                .map(|(sample, curves)| {
                    RegionRef::Profile(ProfileRef::anchored(sketch, sample, curves))
                })
                .collect())
        }
        Value::Array(items) => {
            let mut out = Vec::with_capacity(items.len());
            let mut cached: Option<Vec<(Vec2, u64)>> = None;
            for item in items {
                match item {
                    Value::Number(_) => {
                        let sketch = need_sketch()?;
                        let i = item.as_u64().ok_or_else(|| {
                            ToolError::bad("region indexes are non-negative integers")
                        })? as usize;
                        if cached.is_none() {
                            cached = Some(samples(doc, sketch)?);
                        }
                        let points = cached.as_ref().unwrap();
                        let (sample, curves) = *points.get(i).ok_or_else(|| {
                            ToolError::bad(format!(
                                "sketch {} has {} region(s); there is no region {i}",
                                sketch.0,
                                points.len()
                            ))
                        })?;
                        out.push(RegionRef::Profile(ProfileRef::anchored(
                            sketch, sample, curves,
                        )));
                    }
                    Value::Array(_) => {
                        // A point names the smallest region around it, whose curves are
                        // what the reference should remember.
                        let sketch = need_sketch()?;
                        let sample = vec2_of(item, "regions")?;
                        let curves = doc
                            .state()
                            .sketches
                            .get(&sketch)
                            .map(|s| s.sketch.profiles(&basset_sketch::Tessellation::default()))
                            .and_then(|profiles| {
                                profiles
                                    .into_iter()
                                    .filter(|p| p.contains(sample))
                                    .min_by(|a, b| a.area().total_cmp(&b.area()))
                                    .map(|p| p.signature())
                            })
                            .unwrap_or(0);
                        out.push(RegionRef::Profile(ProfileRef::anchored(
                            sketch, sample, curves,
                        )));
                    }
                    Value::Object(_) => out.push(RegionRef::Face(face_ref_from_json(item)?)),
                    _ => return Err(ToolError::bad(format!("cannot read a region from {item}"))),
                }
            }
            if out.is_empty() {
                return Err(ToolError::bad("\"regions\" is empty"));
            }
            Ok(out)
        }
        _ => Err(ToolError::bad("\"regions\" must be \"all\" or an array")),
    }
}

fn body_op_arg(doc: &mut Document, args: &Value) -> Result<BodyOp, ToolError> {
    let op = opt_str(args, "operation")
        .unwrap_or("new")
        .to_ascii_lowercase();
    if op == "new" || op == "new_body" {
        return Ok(BodyOp::NewBody);
    }
    let targets: Vec<BodyRef> = match opt(args, "targets") {
        Some(Value::Array(items)) => items
            .iter()
            .map(|v| feature_from_json(v).map(BodyRef))
            .collect::<Result<_, _>>()?,
        Some(other) => {
            return Err(ToolError::bad(format!(
                "\"targets\" must be an array of body ids, got {other}"
            )));
        }
        None => doc.state().bodies.keys().copied().collect(),
    };
    if targets.is_empty() {
        return Err(ToolError::bad(format!(
            "a {op} needs a body to apply to, and the model has none"
        )));
    }
    Ok(match op.as_str() {
        "join" | "union" | "add" => BodyOp::Join(targets),
        "cut" | "subtract" => BodyOp::Cut(targets),
        "intersect" => BodyOp::Intersect(targets),
        other => {
            return Err(ToolError::bad(format!(
                "unknown operation {other:?}; one of new, join, cut, intersect"
            )));
        }
    })
}

fn extent_arg(args: &Value) -> Result<Extent, ToolError> {
    if let Some(e) = opt(args, "extent") {
        let kind = e
            .get("type")
            .and_then(Value::as_str)
            .unwrap_or("one_side")
            .to_ascii_lowercase();
        return Ok(match kind.as_str() {
            "one_side" | "distance" => Extent::OneSide(req_f64(e, "distance")?),
            "symmetric" => Extent::Symmetric(req_f64(e, "distance")?),
            "two_sides" => Extent::TwoSides {
                positive: req_f64(e, "positive")?,
                negative: req_f64(e, "negative")?,
            },
            "to_face" => Extent::ToFace(face_ref_from_json(e)?),
            other => {
                return Err(ToolError::bad(format!(
                    "unknown extent type {other:?}; one of one_side, symmetric, two_sides, to_face"
                )));
            }
        });
    }
    let distance = opt_f64(args, "distance")?
        .ok_or_else(|| ToolError::bad("give \"distance\" or \"extent\""))?;
    Ok(Extent::OneSide(distance))
}

/// What every tool that adds a feature replies with: the feature, its status, and the
/// body it made or changed.
fn feature_result(doc: &mut Document, id: FeatureId) -> Value {
    let feature = doc.timeline().get(id).cloned();
    let state = doc.state();
    let mut out = json!({ "feature": id.0 });
    if let Some(f) = &feature {
        out = summary::feature(f, state);
        out["feature"] = json!(id.0);
        let touched: Vec<BodyRef> = match &f.kind {
            FeatureKind::Extrude { operation, .. }
            | FeatureKind::Revolve { operation, .. }
            | FeatureKind::Sweep { operation, .. }
            | FeatureKind::Loft { operation, .. } => match operation {
                BodyOp::NewBody => vec![BodyRef(id)],
                op => op.targets().to_vec(),
            },
            FeatureKind::Fillet { edges, .. } | FeatureKind::Chamfer { edges, .. } => {
                let mut b: Vec<BodyRef> = edges.iter().map(|e| e.body).collect();
                b.dedup();
                b
            }
            FeatureKind::Combine { target, .. } => vec![*target],
            FeatureKind::Move { body, .. } => vec![*body],
            _ => Vec::new(),
        };
        out["bodies"] = Value::Array(
            touched
                .iter()
                .filter_map(|b| state.body(*b))
                .map(summary::body)
                .collect(),
        );
    }
    out
}

fn extrude(session: &mut Session, args: &Value) -> Result<Value, ToolError> {
    let doc = session.document_mut();
    let regions = regions_arg(doc, args)?;
    let extent = extent_arg(args)?;
    let operation = body_op_arg(doc, args)?;
    let component = component_arg(args)?;
    let id = doc.add_named_feature(
        FeatureKind::Extrude {
            regions,
            extent,
            operation,
            component,
        },
        opt_str(args, "name").map(String::from),
    );
    Ok(feature_result(doc, id))
}

fn revolve(session: &mut Session, args: &Value) -> Result<Value, ToolError> {
    let doc = session.document_mut();
    let regions = regions_arg(doc, args)?;
    let axis = axis_from_json(field(args, "axis")?, &|id| stored_sketch(doc, id))?;
    let angle = opt_f64(args, "angle_deg")?.unwrap_or(360.0).to_radians();
    let operation = body_op_arg(doc, args)?;
    let component = component_arg(args)?;
    let id = doc.add_named_feature(
        FeatureKind::Revolve {
            regions,
            axis,
            angle,
            operation,
            component,
        },
        opt_str(args, "name").map(String::from),
    );
    Ok(feature_result(doc, id))
}

fn blend(session: &mut Session, args: &Value, is_fillet: bool) -> Result<Value, ToolError> {
    let body = BodyRef(feature_from_json(field(args, "body")?)?);
    let edges: Vec<EdgeRef> = req_array(args, "edges")?
        .iter()
        .map(|e| {
            let s = e
                .as_str()
                .ok_or_else(|| ToolError::bad("edge keys are strings"))?;
            Ok(EdgeRef {
                body,
                key: edge_key_from_str(s)?,
            })
        })
        .collect::<Result<_, ToolError>>()?;
    if edges.is_empty() {
        return Err(ToolError::bad("\"edges\" is empty"));
    }
    let doc = session.document_mut();
    // Say which edge is missing before the regenerator does, with the keys it knows.
    if let Some(b) = doc.state().body(body) {
        let known: Vec<_> = b.solid.edges().iter().map(|e| e.key).collect();
        for e in &edges {
            if !known.contains(&e.key) {
                return Err(ToolError::bad(format!(
                    "body {} has no edge {}; list them with body_info edges=true",
                    body.0.0,
                    edge_key_to_string(e.key)
                )));
            }
        }
    } else {
        return Err(ToolError::bad(format!(
            "no body {} at the timeline cursor",
            body.0.0
        )));
    }
    let kind = if is_fillet {
        FeatureKind::Fillet {
            edges,
            radius: req_f64(args, "radius")?,
        }
    } else {
        FeatureKind::Chamfer {
            edges,
            distance: req_f64(args, "distance")?,
        }
    };
    let id = doc.add_named_feature(kind, opt_str(args, "name").map(String::from));
    Ok(feature_result(doc, id))
}

fn combine(session: &mut Session, args: &Value) -> Result<Value, ToolError> {
    let target = BodyRef(feature_from_json(field(args, "target")?)?);
    let tools: Vec<BodyRef> = req_array(args, "tools")?
        .iter()
        .map(|v| feature_from_json(v).map(BodyRef))
        .collect::<Result<_, _>>()?;
    let operation = match req_str(args, "operation")?.to_ascii_lowercase().as_str() {
        "join" | "union" => CombineOp::Join,
        "cut" | "subtract" => CombineOp::Cut,
        "intersect" => CombineOp::Intersect,
        other => return Err(ToolError::bad(format!("unknown operation {other:?}"))),
    };
    let keep_tools = opt_bool(args, "keep_tools")?.unwrap_or(false);
    let doc = session.document_mut();
    let id = doc.add_named_feature(
        FeatureKind::Combine {
            target,
            tools,
            operation,
            keep_tools,
        },
        opt_str(args, "name").map(String::from),
    );
    Ok(feature_result(doc, id))
}

fn offset_plane(session: &mut Session, args: &Value) -> Result<Value, ToolError> {
    let base = plane_from_json(field(args, "base")?)?;
    let distance = req_f64(args, "distance")?;
    let doc = session.document_mut();
    let id = doc.add_named_feature(
        FeatureKind::OffsetPlane { base, distance },
        opt_str(args, "name").map(String::from),
    );
    let mut out = feature_result(doc, id);
    out["plane"] = doc
        .state()
        .planes
        .get(&id)
        .map(summary::frame)
        .unwrap_or(Value::Null);
    Ok(out)
}

fn angled_plane(session: &mut Session, args: &Value) -> Result<Value, ToolError> {
    let base = plane_from_json(field(args, "base")?)?;
    let angle = req_f64(args, "angle_deg")?.to_radians();
    let doc = session.document_mut();
    let axis = axis_from_json(field(args, "axis")?, &|id| stored_sketch(doc, id))?;
    let id = doc.add_named_feature(
        FeatureKind::AngledPlane { base, axis, angle },
        opt_str(args, "name").map(String::from),
    );
    let mut out = feature_result(doc, id);
    out["plane"] = doc
        .state()
        .planes
        .get(&id)
        .map(summary::frame)
        .unwrap_or(Value::Null);
    Ok(out)
}

fn move_body(session: &mut Session, args: &Value) -> Result<Value, ToolError> {
    let body = BodyRef(feature_from_json(field(args, "body")?)?);
    let t = req_vec3(args, "translation")?;
    let doc = session.document_mut();
    let id = doc.add_feature(FeatureKind::Move {
        body,
        transform: Affine3::from_translation(t),
    });
    Ok(feature_result(doc, id))
}

// ----- editing --------------------------------------------------------------------------

fn edit_feature(session: &mut Session, args: &Value) -> Result<Value, ToolError> {
    let id = feature_from_json(field(args, "feature")?)?;
    let doc = session.document_mut();
    if doc.timeline().get(id).is_none() {
        return Err(ToolError::bad(format!("no feature {}", id.0)));
    }
    if opt_bool(args, "remove")?.unwrap_or(false) {
        let dependants: Vec<u64> = doc
            .timeline()
            .dependants_of(id)
            .iter()
            .map(|d| d.0)
            .collect();
        doc.remove_feature(id)?;
        return Ok(
            json!({ "removed": id.0, "dependants_now_broken": dependants, "check": check_document(doc) }),
        );
    }
    if let Some(index) = opt_u64(args, "move_to")? {
        doc.reorder_feature(id, index as usize)?;
    }
    if let Some(name) = opt_str(args, "name") {
        doc.rename_feature(id, name)?;
    }
    if let Some(s) = opt_bool(args, "suppressed")? {
        doc.set_suppressed(id, s)?;
    }
    let extent = opt(args, "extent").map(|_| extent_arg(args)).transpose()?;
    let fields = [
        (NumericField::Distance, opt_f64(args, "distance")?),
        (NumericField::Negative, opt_f64(args, "negative")?),
        (NumericField::Angle, opt_f64(args, "angle_deg")?),
        (NumericField::Radius, opt_f64(args, "radius")?),
    ];
    let wanted: Vec<(NumericField, f64)> = fields
        .iter()
        .filter_map(|(f, v)| v.map(|v| (*f, v)))
        .collect();
    if extent.is_some() || !wanted.is_empty() {
        let mut refused = Vec::new();
        doc.edit_feature(id, |f| {
            if let (Some(extent), FeatureKind::Extrude { extent: e, .. }) = (extent, &mut f.kind) {
                *e = extent;
                // A field the new extent no longer offers must not keep driving nothing.
                let offered = f.kind.numeric_fields();
                f.exprs.retain(|k, _| offered.contains(k));
            }
            for (field, value) in &wanted {
                if f.kind.set_numeric_field(*field, *value) {
                    f.exprs.remove(field);
                } else {
                    refused.push(field.label());
                }
            }
        })?;
        if !refused.is_empty() {
            return Err(ToolError::bad(format!(
                "this feature has no {} to set",
                refused.join(", ")
            )));
        }
        if extent.is_some()
            && !matches!(
                doc.timeline().get(id).map(|f| &f.kind),
                Some(FeatureKind::Extrude { .. })
            )
        {
            return Err(ToolError::bad("only an extrude has an extent"));
        }
    }
    Ok(feature_result(doc, id))
}

fn set_feature_expr(session: &mut Session, args: &Value) -> Result<Value, ToolError> {
    let id = feature_from_json(field(args, "feature")?)?;
    let fld = match req_str(args, "field")?.to_ascii_lowercase().as_str() {
        "distance" => NumericField::Distance,
        "negative" | "second_distance" => NumericField::Negative,
        "angle" => NumericField::Angle,
        "radius" => NumericField::Radius,
        other => {
            return Err(ToolError::bad(format!(
                "unknown field {other:?}; one of distance, negative, angle, radius"
            )));
        }
    };
    let expression = req_str(args, "expression")?;
    let doc = session.document_mut();
    if expression.trim().is_empty() {
        let released = doc.clear_feature_expr(id, fld)?;
        return Ok(json!({ "released": released, "feature": feature_result(doc, id) }));
    }
    let value = doc.set_feature_expr(id, fld, expression)?;
    Ok(json!({ "value": round(value), "feature": feature_result(doc, id) }))
}

// ----- inspection -----------------------------------------------------------------------

fn body_info(session: &mut Session, args: &Value) -> Result<Value, ToolError> {
    let id = BodyRef(feature_from_json(field(args, "body")?)?);
    let want_faces = opt_bool(args, "faces")?.unwrap_or(false);
    let want_edges = opt_bool(args, "edges")?.unwrap_or(false);
    let touching = opt_str(args, "touching_face")
        .map(face_key_from_str)
        .transpose()?;
    let doc = session.document_mut();
    let state = doc.state();
    let body = state.body(id).ok_or_else(|| {
        let known: Vec<u64> = state.bodies.keys().map(|b| b.0.0).collect();
        ToolError::bad(format!(
            "no body {} at the timeline cursor; bodies: {known:?}",
            id.0.0
        ))
    })?;
    let mut out = summary::body(body);
    if want_faces {
        out["face_list"] = Value::Array(body.solid.faces.iter().map(summary::face).collect());
    }
    if want_edges || touching.is_some() {
        let edges = body.solid.edges();
        out["edge_list"] = Value::Array(
            edges
                .iter()
                .filter(|e| touching.is_none_or(|f| e.key.touches(f)))
                .map(summary::edge)
                .collect(),
        );
    }
    Ok(out)
}

fn export(session: &mut Session, args: &Value) -> Result<Value, ToolError> {
    let path = PathBuf::from(req_str(args, "path")?);
    let format = match opt_str(args, "format") {
        Some(f) => f.to_ascii_lowercase(),
        None => path
            .extension()
            .and_then(|e| e.to_str())
            .unwrap_or("stl")
            .to_ascii_lowercase(),
    };
    let wanted: Option<Vec<BodyRef>> = match opt(args, "bodies") {
        Some(Value::Array(items)) => Some(
            items
                .iter()
                .map(|v| feature_from_json(v).map(BodyRef))
                .collect::<Result<_, _>>()?,
        ),
        _ => None,
    };
    let doc = session.document_mut();
    let state = doc.state();
    let items: Vec<ExportItem> = state
        .bodies
        .values()
        .filter(|b| wanted.as_ref().is_none_or(|w| w.contains(&b.id)))
        .map(|b| ExportItem::new(b.name.clone(), b.solid.tessellate().mesh))
        .collect();
    if items.is_empty() {
        return Err(ToolError::bad("nothing to export: no matching bodies"));
    }
    match format.as_str() {
        "stl" => basset_io::convenience::export_stl_file(&path, &items)?,
        "3mf" => basset_io::convenience::export_3mf_file(&path, &items, Unit::Millimeter)?,
        other => {
            return Err(ToolError::bad(format!(
                "unknown format {other:?}; stl or 3mf"
            )));
        }
    }
    Ok(
        json!({ "exported": path.display().to_string(), "bodies": items.len(), "triangles": items.iter().map(|i| i.mesh.triangle_count()).sum::<usize>() }),
    )
}

/// Resolves script references in a call's arguments against the results so far.
///
/// A string that is exactly `"$N"` or `"$N.field"` becomes that field of call N's result
/// (`feature` by default), as a value: `"sketch": "$1"` names the sketch call 1 made
/// whatever id the document gave it. Inside a longer string, `${N}` and `${N.field}` are
/// substituted as text, which is how a face key quotes the feature that made the face:
/// `"${2}.0:EndCap"`.
fn resolve_script_refs(value: &Value, results: &[Value]) -> Result<Value, ToolError> {
    let lookup = |reference: &str| -> Result<Value, ToolError> {
        let (index, field) = match reference.split_once('.') {
            Some((i, f)) => (i, f),
            None => (reference, "feature"),
        };
        let index: usize = index.parse().map_err(|_| {
            ToolError::bad(format!(
                "bad script reference ${reference:?}; use \"$N\" or \"$N.field\""
            ))
        })?;
        let result = results.get(index).ok_or_else(|| {
            ToolError::bad(format!(
                "${reference} refers to call {index}, which has not run"
            ))
        })?;
        result
            .get("result")
            .and_then(|r| r.get(field))
            .cloned()
            .ok_or_else(|| {
                ToolError::bad(format!(
                    "${reference}: call {index}'s result has no {field:?}"
                ))
            })
    };
    Ok(match value {
        Value::String(s) if s.starts_with('$') && !s.starts_with("${") => lookup(&s[1..])?,
        Value::String(s) if s.contains("${") => {
            let mut out = String::new();
            let mut rest = s.as_str();
            while let Some(start) = rest.find("${") {
                out.push_str(&rest[..start]);
                let after = &rest[start + 2..];
                let end = after
                    .find('}')
                    .ok_or_else(|| ToolError::bad(format!("unclosed ${{ in {s:?}")))?;
                let value = lookup(&after[..end])?;
                match value {
                    Value::String(v) => out.push_str(&v),
                    other => out.push_str(&other.to_string()),
                }
                rest = &after[end + 1..];
            }
            out.push_str(rest);
            Value::String(out)
        }
        Value::Array(items) => Value::Array(
            items
                .iter()
                .map(|v| resolve_script_refs(v, results))
                .collect::<Result<_, _>>()?,
        ),
        Value::Object(map) => Value::Object(
            map.iter()
                .map(|(k, v)| resolve_script_refs(v, results).map(|v| (k.clone(), v)))
                .collect::<Result<_, _>>()?,
        ),
        other => other.clone(),
    })
}

fn run_script(session: &mut Session, args: &Value) -> Result<Value, ToolError> {
    let calls = req_array(args, "calls")?;
    let keep_going = opt_bool(args, "continue_on_error")?.unwrap_or(false);
    let mut results = Vec::with_capacity(calls.len());
    for (i, c) in calls.iter().enumerate() {
        let name = c
            .get("tool")
            .and_then(Value::as_str)
            .ok_or_else(|| ToolError::bad(format!("call {i} has no \"tool\"")))?;
        let empty = json!({});
        let arguments = c
            .get("arguments")
            .or_else(|| c.get("args"))
            .unwrap_or(&empty);
        let arguments = match resolve_script_refs(arguments, &results) {
            Ok(a) => a,
            Err(e) => {
                results
                    .push(json!({ "call": i, "tool": name, "ok": false, "error": e.to_string() }));
                if keep_going {
                    continue;
                }
                return Ok(json!({ "completed": false, "results": results }));
            }
        };
        match call(session, name, &arguments) {
            Ok(v) => results.push(json!({ "call": i, "tool": name, "ok": true, "result": v })),
            Err(e) => {
                results
                    .push(json!({ "call": i, "tool": name, "ok": false, "error": e.to_string() }));
                if !keep_going {
                    return Ok(json!({ "completed": false, "results": results }));
                }
            }
        }
    }
    Ok(json!({ "completed": true, "results": results }))
}
