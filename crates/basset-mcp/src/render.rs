//! Appearances, the scene and rendered pictures over the protocol: the Render
//! workspace for an agent.
//!
//! An agent cannot look at the viewport, so the picture is the one thing it has to be
//! handed as a file: `render_image` traces the model with the path tracer the editor's
//! in-canvas render uses and writes a PNG, from a named view or a direction, framed to
//! the visible bodies.

use std::path::PathBuf;

use basset_core::{BodyRef, Document};
use basset_math::Vec3;
use basset_render::{
    Appearance, Background, Category, EnvironmentKind, Pattern, RenderCamera, RenderOptions,
    SceneSettings, Srgb,
};
use serde_json::{Value, json};

use crate::args::{field, opt, opt_bool, opt_f64, opt_str, opt_u64, req_str, vec3_of};
use crate::ids::{face_key_from_str, feature_from_json};
use crate::summary::round;
use crate::{Session, ToolError};

fn appearance_json(a: &Appearance) -> Value {
    json!({
        "name": a.display_name(),
        "category": a.category.name(),
        "color": a.color.to_string(),
        "metallic": round(f64::from(a.metallic)),
        "roughness": round(f64::from(a.roughness)),
        "transmission": round(f64::from(a.transmission)),
        "ior": round(f64::from(a.ior)),
        "clearcoat": round(f64::from(a.clearcoat)),
        "emission": round(f64::from(a.emission)),
        "pattern": a.pattern.name(),
        "color2": a.color2.to_string(),
        "pattern_scale": round(f64::from(a.pattern_scale)),
    })
}

/// The library, optionally narrowed to a category or a substring.
pub fn appearance_library(args: &Value) -> Result<Value, ToolError> {
    let category = match opt_str(args, "category") {
        None => None,
        Some(c) => {
            let wanted = c.to_lowercase();
            Some(
                Category::ALL
                    .into_iter()
                    .find(|k| k.name().to_lowercase().starts_with(&wanted))
                    .ok_or_else(|| {
                        let names: Vec<&str> = Category::ALL.iter().map(|k| k.name()).collect();
                        ToolError::bad(format!(
                            "unknown category {c:?}; one of {}",
                            names.join(", ")
                        ))
                    })?,
            )
        }
    };
    let query = opt_str(args, "query").map(str::to_lowercase);
    let entries: Vec<Value> = basset_render::library()
        .iter()
        .filter(|a| category.is_none_or(|c| a.category == c))
        .filter(|a| {
            query
                .as_deref()
                .is_none_or(|q| a.name.to_lowercase().contains(q))
        })
        .map(appearance_json)
        .collect();
    Ok(json!({ "appearances": entries }))
}

/// An appearance by name: the design's own copy first (it may have been edited), then the
/// library.
fn resolve(doc: &Document, name: &str) -> Result<Appearance, ToolError> {
    if let Some(a) = doc.appearances().get(name) {
        return Ok(a.clone());
    }
    basset_render::find(name).cloned().ok_or_else(|| {
        ToolError::bad(format!(
            "no appearance {name:?} in the design or the library; appearance_library lists them"
        ))
    })
}

/// Paints a body, one of its faces, or the document's default.
pub fn set_appearance(session: &mut Session, args: &Value) -> Result<Value, ToolError> {
    let doc = session.document_mut();
    let appearance = match opt(args, "appearance") {
        None | Some(Value::Null) => None,
        Some(v) => Some(resolve(
            doc,
            v.as_str()
                .ok_or_else(|| ToolError::bad("appearance must be a name or null"))?,
        )?),
    };
    let target = if opt_bool(args, "default")?.unwrap_or(false) {
        doc.set_default_appearance(appearance.as_ref());
        json!("default")
    } else {
        let body = BodyRef(feature_from_json(field(args, "body")?)?);
        if !doc.state().bodies.contains_key(&body) {
            return Err(ToolError::bad(format!(
                "no body {} at the timeline cursor",
                body.0.0
            )));
        }
        match opt_str(args, "face") {
            Some(face) => {
                let key = face_key_from_str(face)?;
                doc.set_face_appearance(body, key, appearance.as_ref());
                json!({ "body": body.0.0, "face": face })
            }
            None => {
                doc.set_body_appearance(body, appearance.as_ref());
                json!({ "body": body.0.0 })
            }
        }
    };
    Ok(json!({
        "applied": appearance.as_ref().map(|a| a.display_name().to_owned()),
        "to": target,
        "appearances": appearances_json(doc),
    }))
}

fn appearances_json(doc: &Document) -> Value {
    let a = doc.appearances();
    json!({
        "default": a.default,
        "bodies": a.bodies.iter().map(|(b, n)| json!({ "body": b.0.0, "appearance": n })).collect::<Vec<_>>(),
        "faces": a.faces.iter().map(|f| json!({
            "body": f.body.0.0,
            "face": crate::ids::face_key_to_string(f.face),
            "appearance": f.appearance,
        })).collect::<Vec<_>>(),
        "defined": a.defined.values().map(appearance_json).collect::<Vec<_>>(),
    })
}

/// Creates or changes an appearance of the design, starting from a library entry or an
/// existing one, with any of its numbers overridden.
pub fn define_appearance(session: &mut Session, args: &Value) -> Result<Value, ToolError> {
    let doc = session.document_mut();
    let name = req_str(args, "name")?.trim().to_owned();
    if name.is_empty() {
        return Err(ToolError::bad("an appearance needs a name"));
    }
    let existing = doc.appearances().get(&name).cloned();
    let mut a = match (opt_str(args, "base"), &existing) {
        (Some(base), _) => resolve(doc, base)?,
        (None, Some(e)) => e.clone(),
        (None, None) => Appearance::DEFAULT.clone(),
    };
    a.name = name.clone();
    let color = |key: &str| -> Result<Option<Srgb>, ToolError> {
        opt_str(args, key)
            .map(|c| Srgb::parse(c).ok_or_else(|| ToolError::bad(format!("{key} must be #rrggbb"))))
            .transpose()
    };
    if let Some(c) = color("color")? {
        a.color = c;
    }
    if let Some(c) = color("color2")? {
        a.color2 = c;
    }
    for (key, slot) in [
        ("metallic", &mut a.metallic),
        ("roughness", &mut a.roughness),
        ("transmission", &mut a.transmission),
        ("ior", &mut a.ior),
        ("clearcoat", &mut a.clearcoat),
        ("emission", &mut a.emission),
        ("pattern_scale", &mut a.pattern_scale),
    ] {
        if let Some(v) = opt_f64(args, key)? {
            *slot = v as f32;
        }
    }
    if let Some(p) = opt_str(args, "pattern") {
        let wanted = p.to_lowercase();
        a.pattern = Pattern::ALL
            .into_iter()
            .find(|k| k.name().to_lowercase().starts_with(&wanted))
            .ok_or_else(|| {
                let names: Vec<&str> = Pattern::ALL.iter().map(|k| k.name()).collect();
                ToolError::bad(format!(
                    "unknown pattern {p:?}; one of {}",
                    names.join(", ")
                ))
            })?;
    }
    let a = a.sanitised();
    if existing.is_some() {
        doc.edit_appearance(&name, a.clone())?;
    } else {
        doc.define_appearance(&a);
    }
    Ok(json!({ "appearance": appearance_json(&a) }))
}

/// Reads or changes the scene settings.
pub fn scene_settings(session: &mut Session, args: &Value) -> Result<Value, ToolError> {
    let doc = session.document_mut();
    let mut s = doc.scene().clone();
    if let Some(e) = opt_str(args, "environment") {
        s.environment = EnvironmentKind::from_name(e).ok_or_else(|| {
            let names: Vec<&str> = EnvironmentKind::ALL.iter().map(|k| k.name()).collect();
            ToolError::bad(format!(
                "unknown environment {e:?}; one of {}",
                names.join(", ")
            ))
        })?;
    }
    if let Some(v) = opt_f64(args, "brightness")? {
        s.brightness = v as f32;
    }
    if let Some(v) = opt_f64(args, "rotation")? {
        s.rotation = v as f32;
    }
    if let Some(b) = opt_str(args, "background") {
        s.background = if b.eq_ignore_ascii_case("environment") {
            Background::Environment
        } else {
            Background::Solid(Srgb::parse(b).ok_or_else(|| {
                ToolError::bad("background is \"environment\" or a #rrggbb colour")
            })?)
        };
    }
    if let Some(v) = opt_bool(args, "ground_plane")? {
        s.ground_plane = v;
    }
    if let Some(v) = opt_bool(args, "ground_reflections")? {
        s.ground_reflections = v;
    }
    if let Some(v) = opt_f64(args, "ground_roughness")? {
        s.ground_roughness = v.clamp(0.0, 1.0) as f32;
    }
    if let Some(v) = opt_bool(args, "depth_of_field")? {
        s.depth_of_field = v;
    }
    if let Some(v) = opt_f64(args, "aperture")? {
        s.aperture = v.clamp(0.0, 1.0) as f32;
    }
    doc.set_scene(s.clone());
    Ok(scene_json(&s))
}

fn scene_json(s: &SceneSettings) -> Value {
    json!({
        "environment": s.environment.name(),
        "environments": EnvironmentKind::ALL.iter().map(|k| k.name()).collect::<Vec<_>>(),
        "brightness": round(f64::from(s.brightness)),
        "rotation": round(f64::from(s.rotation)),
        "background": match s.background {
            Background::Environment => "environment".to_owned(),
            Background::Solid(c) => c.to_string(),
        },
        "ground_plane": s.ground_plane,
        "ground_reflections": s.ground_reflections,
        "ground_roughness": round(f64::from(s.ground_roughness)),
        "depth_of_field": s.depth_of_field,
        "aperture": round(f64::from(s.aperture)),
    })
}

/// The direction from the model to the eye for a named view, as the editor's view cube
/// names them.
fn view_direction(name: &str) -> Option<Vec3> {
    let d = match name.to_ascii_lowercase().as_str() {
        "iso" | "isometric" | "home" => Vec3::new(1.0, -1.0, 0.8),
        "front" => Vec3::new(0.0, -1.0, 0.0),
        "back" => Vec3::new(0.0, 1.0, 0.0),
        "right" => Vec3::new(1.0, 0.0, 0.0),
        "left" => Vec3::new(-1.0, 0.0, 0.0),
        "top" => Vec3::new(0.0, -1e-3, 1.0),
        "bottom" => Vec3::new(0.0, -1e-3, -1.0),
        _ => return None,
    };
    Some(d.normalize())
}

/// Traces the model and writes a PNG.
pub fn render_image(session: &mut Session, args: &Value) -> Result<Value, ToolError> {
    let path = PathBuf::from(req_str(args, "path")?);
    let width = opt_u64(args, "width")?.unwrap_or(1200).clamp(16, 8192) as u32;
    let height = opt_u64(args, "height")?.unwrap_or(800).clamp(16, 8192) as u32;
    let samples = opt_u64(args, "samples")?.unwrap_or(128).clamp(1, 16384) as u32;
    let fov = opt_f64(args, "fov")?
        .unwrap_or(30.0)
        .clamp(5.0, 120.0)
        .to_radians();
    let from = match opt(args, "view") {
        None => view_direction("iso").expect("a known view"),
        Some(Value::String(name)) => view_direction(name).ok_or_else(|| {
            ToolError::bad(format!(
                "unknown view {name:?}; iso, front, back, left, right, top, bottom, or a direction [x, y, z] from the model to the eye"
            ))
        })?,
        Some(v) => vec3_of(v, "view")?,
    };
    let doc = session.document_mut();
    let hidden = doc.visibility().hidden_bodies.clone();
    let state = doc.state();
    let tessellations: Vec<(BodyRef, basset_kernel::Tessellated)> = state
        .bodies
        .iter()
        .filter(|(id, _)| !hidden.contains(id))
        .map(|(id, b)| (*id, b.solid.tessellate()))
        .collect();
    if tessellations.is_empty() {
        return Err(ToolError::bad(
            "there is nothing to render: no visible bodies",
        ));
    }
    let scene = basset_core::trace_scene(
        tessellations.iter().map(|(id, t)| (*id, t)),
        doc.appearances(),
        doc.scene(),
    );
    let mut camera = RenderCamera::fit(
        &scene.bounds(),
        from,
        fov,
        f64::from(width) / f64::from(height),
    );
    if doc.scene().depth_of_field {
        camera = camera.with_depth_of_field(f64::from(doc.scene().aperture));
    }
    let options = RenderOptions {
        width,
        height,
        samples,
        max_bounces: 10,
        ..RenderOptions::default()
    };
    let started = std::time::Instant::now();
    let image = basset_render::render(&scene, &camera, &options);
    image
        .save_png(&path)
        .map_err(|e| ToolError::bad(format!("could not write {}: {e}", path.display())))?;
    Ok(json!({
        "path": path.display().to_string(),
        "width": width,
        "height": height,
        "samples": samples,
        "triangles": scene.triangle_count(),
        "seconds": round(started.elapsed().as_secs_f64()),
        "scene": scene_json(doc.scene()),
    }))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn named_views_point_from_the_model_to_the_eye() {
        assert!(view_direction("top").unwrap().z > 0.99);
        assert!(view_direction("Front").unwrap().y < -0.99);
        assert!(view_direction("sideways").is_none());
    }
}
