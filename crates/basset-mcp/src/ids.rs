//! How the wire names things.
//!
//! The protocol is JSON, so every identifier has to round-trip through a number or a
//! string an agent can read back and quote. Features are their integer id. Sketch
//! entities and constraints are slotmap keys, exposed as the packed `u64` the key
//! already is, so a value quoted from one reply is accepted in the next. Faces are
//! `"<feature>.<sub>:<Role>"`, the three parts a `FaceKey` is made of, and an edge is
//! two of those joined with `|`.
//!
//! A slotmap key is an index and a version; only the index is shown. The version exists
//! so a stale key cannot alias a slot that was freed and reused, but a wire id is read
//! against the live sketch, where at most one entity occupies each index, so the index
//! is unambiguous and `4294967297` becomes `1`. A number that names no live entity is
//! refused, which is what the version would have done.

use basset_core::{
    AxisRef, BodyRef, FaceKey, FaceRef, FaceRole, FeatureId, OriginAxis, OriginPlane, PlaneRef,
};
use basset_kernel::{EdgeKey, OpId};
use basset_sketch::{ConstraintId, EntityId, Sketch};
use serde_json::{Value, json};
use slotmap::Key;

use crate::ToolError;

/// The index half of a slotmap key.
fn index_of<K: Key>(id: K) -> u64 {
    id.data().as_ffi() & 0xffff_ffff
}

pub fn entity_to_json(id: EntityId) -> Value {
    json!(index_of(id))
}

pub fn constraint_to_json(id: ConstraintId) -> Value {
    json!(index_of(id))
}

pub fn entity_from_json(sketch: &Sketch, v: &Value) -> Result<EntityId, ToolError> {
    let raw = v
        .as_u64()
        .ok_or_else(|| ToolError::bad(format!("expected an entity id, got {v}")))?;
    sketch
        .entities()
        .map(|(id, _)| id)
        .find(|id| index_of(*id) == raw)
        .ok_or_else(|| ToolError::bad(format!("no entity {raw} in this sketch")))
}

pub fn constraint_from_json(sketch: &Sketch, v: &Value) -> Result<ConstraintId, ToolError> {
    let raw = v
        .as_u64()
        .ok_or_else(|| ToolError::bad(format!("expected a constraint id, got {v}")))?;
    sketch
        .constraints()
        .map(|(id, _)| id)
        .find(|id| index_of(*id) == raw)
        .ok_or_else(|| ToolError::bad(format!("no constraint {raw} in this sketch")))
}

pub fn feature_from_json(v: &Value) -> Result<FeatureId, ToolError> {
    v.as_u64()
        .map(FeatureId)
        .ok_or_else(|| ToolError::bad(format!("expected a feature id, got {v}")))
}

pub fn face_key_to_string(key: FaceKey) -> String {
    format!("{}.{}:{:?}", key.op.feature, key.op.sub, key.role)
}

pub fn edge_key_to_string(key: EdgeKey) -> String {
    format!(
        "{}|{}",
        face_key_to_string(key.a),
        face_key_to_string(key.b)
    )
}

pub fn face_key_from_str(s: &str) -> Result<FaceKey, ToolError> {
    let bad = || {
        ToolError::bad(format!(
            "malformed face key {s:?}; expected feature.sub:Role"
        ))
    };
    let (op, role) = s.split_once(':').ok_or_else(bad)?;
    let (feature, sub) = op.split_once('.').ok_or_else(bad)?;
    let feature: u64 = feature.trim().parse().map_err(|_| bad())?;
    let sub: u32 = sub.trim().parse().map_err(|_| bad())?;
    let role = role.trim();
    let payload = |name: &str| -> Result<u32, ToolError> {
        role.strip_prefix(name)
            .and_then(|r| r.strip_prefix('('))
            .and_then(|r| r.strip_suffix(')'))
            .and_then(|n| n.parse().ok())
            .ok_or_else(bad)
    };
    let role = match role {
        "StartCap" => FaceRole::StartCap,
        "EndCap" => FaceRole::EndCap,
        r if r.starts_with("Side") => FaceRole::Side(payload("Side")?),
        r if r.starts_with("Fillet") => FaceRole::Fillet(payload("Fillet")?),
        r if r.starts_with("Chamfer") => FaceRole::Chamfer(payload("Chamfer")?),
        r if r.starts_with("Generic") => FaceRole::Generic(payload("Generic")?),
        r if r.starts_with("Thread") => FaceRole::Thread(payload("Thread")?),
        _ => return Err(bad()),
    };
    Ok(FaceKey::new(OpId::new(feature).with_sub(sub), role))
}

pub fn edge_key_from_str(s: &str) -> Result<EdgeKey, ToolError> {
    let (a, b) = s
        .split_once('|')
        .ok_or_else(|| ToolError::bad(format!("malformed edge key {s:?}; expected faceA|faceB")))?;
    Ok(EdgeKey::new(face_key_from_str(a)?, face_key_from_str(b)?))
}

/// `{"body": 3, "face": "3.0:EndCap"}`.
pub fn face_ref_from_json(v: &Value) -> Result<FaceRef, ToolError> {
    let body = v
        .get("body")
        .ok_or_else(|| ToolError::bad("a face reference needs a \"body\""))?;
    let face = v
        .get("face")
        .and_then(Value::as_str)
        .ok_or_else(|| ToolError::bad("a face reference needs a \"face\" key string"))?;
    Ok(FaceRef {
        body: BodyRef(feature_from_json(body)?),
        key: face_key_from_str(face)?,
    })
}

pub fn face_ref_to_json(f: &FaceRef) -> Value {
    json!({ "body": f.body.0.0, "face": face_key_to_string(f.key) })
}

/// `"XY"`, `"YZ"`, `"XZ"`, a plane feature's id, or a face reference.
pub fn plane_from_json(v: &Value) -> Result<PlaneRef, ToolError> {
    match v {
        Value::String(s) => match s.to_ascii_uppercase().as_str() {
            "XY" => Ok(PlaneRef::Origin(OriginPlane::XY)),
            "YZ" => Ok(PlaneRef::Origin(OriginPlane::YZ)),
            "XZ" => Ok(PlaneRef::Origin(OriginPlane::XZ)),
            _ => Err(ToolError::bad(format!(
                "unknown plane {s:?}; use XY, YZ, XZ, a plane feature id or {{body, face}}"
            ))),
        },
        Value::Number(_) => Ok(PlaneRef::Feature(feature_from_json(v)?)),
        Value::Object(_) => Ok(PlaneRef::Face(face_ref_from_json(v)?)),
        _ => Err(ToolError::bad(format!("cannot read a plane from {v}"))),
    }
}

pub fn plane_to_json(p: &PlaneRef) -> Value {
    match p {
        PlaneRef::Origin(o) => json!(format!("{o:?}")),
        PlaneRef::Feature(f) => json!(f.0),
        PlaneRef::Face(f) => face_ref_to_json(f),
    }
}

/// `"X"`, `"Y"`, `"Z"` or `{"sketch": id, "line": entity}`. `resolve` looks a sketch up
/// so the line can be checked against it.
pub fn axis_from_json(
    v: &Value,
    resolve: &dyn Fn(FeatureId) -> Result<Sketch, ToolError>,
) -> Result<AxisRef, ToolError> {
    match v {
        Value::String(s) => match s.to_ascii_uppercase().as_str() {
            "X" => Ok(AxisRef::Origin(OriginAxis::X)),
            "Y" => Ok(AxisRef::Origin(OriginAxis::Y)),
            "Z" => Ok(AxisRef::Origin(OriginAxis::Z)),
            _ => Err(ToolError::bad(format!(
                "unknown axis {s:?}; use X, Y, Z or {{sketch, line}}"
            ))),
        },
        Value::Object(o) => {
            let sketch = feature_from_json(
                o.get("sketch")
                    .ok_or_else(|| ToolError::bad("a sketch-line axis needs \"sketch\""))?,
            )?;
            let sketch_data = resolve(sketch)?;
            let line = entity_from_json(
                &sketch_data,
                o.get("line")
                    .ok_or_else(|| ToolError::bad("a sketch-line axis needs \"line\""))?,
            )?;
            if !sketch_data.entity(line).is_some_and(|d| d.entity.is_line()) {
                return Err(ToolError::bad("the axis entity is not a line"));
            }
            Ok(AxisRef::SketchLine { sketch, line })
        }
        _ => Err(ToolError::bad(format!("cannot read an axis from {v}"))),
    }
}

pub fn axis_to_json(a: &AxisRef) -> Value {
    match a {
        AxisRef::Origin(o) => json!(format!("{o:?}")),
        AxisRef::SketchLine { sketch, line } => {
            json!({ "sketch": sketch.0, "line": entity_to_json(*line) })
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn face_and_edge_keys_round_trip() {
        for role in [
            FaceRole::StartCap,
            FaceRole::EndCap,
            FaceRole::Side(7),
            FaceRole::Fillet(0),
            FaceRole::Chamfer(2),
            FaceRole::Generic(9),
            FaceRole::Thread(1),
        ] {
            let key = FaceKey::new(OpId::new(12).with_sub(3), role);
            assert_eq!(face_key_from_str(&face_key_to_string(key)).unwrap(), key);
        }
        let a = FaceKey::new(OpId::new(1), FaceRole::EndCap);
        let b = FaceKey::new(OpId::new(1), FaceRole::Side(2));
        let edge = EdgeKey::new(a, b);
        assert_eq!(edge_key_from_str(&edge_key_to_string(edge)).unwrap(), edge);
        // Order does not matter on the way in.
        assert_eq!(
            edge_key_from_str(&format!(
                "{}|{}",
                face_key_to_string(b),
                face_key_to_string(a)
            ))
            .unwrap(),
            edge
        );
    }

    #[test]
    fn malformed_keys_are_refused() {
        assert!(face_key_from_str("1:EndCap").is_err());
        assert!(face_key_from_str("1.0:Top").is_err());
        assert!(face_key_from_str("1.0:Side(x)").is_err());
        assert!(edge_key_from_str("1.0:EndCap").is_err());
    }
}
