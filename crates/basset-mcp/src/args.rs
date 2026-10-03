//! Reading tool arguments out of a JSON object with messages that name the field.

use basset_math::{Vec2, Vec3};
use serde_json::Value;

use crate::ToolError;

pub fn field<'a>(args: &'a Value, name: &str) -> Result<&'a Value, ToolError> {
    args.get(name)
        .filter(|v| !v.is_null())
        .ok_or_else(|| ToolError::bad(format!("missing argument \"{name}\"")))
}

pub fn opt<'a>(args: &'a Value, name: &str) -> Option<&'a Value> {
    args.get(name).filter(|v| !v.is_null())
}

pub fn f64_of(v: &Value, name: &str) -> Result<f64, ToolError> {
    v.as_f64()
        .filter(|x| x.is_finite())
        .ok_or_else(|| ToolError::bad(format!("\"{name}\" must be a finite number, got {v}")))
}

pub fn req_f64(args: &Value, name: &str) -> Result<f64, ToolError> {
    f64_of(field(args, name)?, name)
}

pub fn opt_f64(args: &Value, name: &str) -> Result<Option<f64>, ToolError> {
    opt(args, name).map(|v| f64_of(v, name)).transpose()
}

pub fn req_u64(args: &Value, name: &str) -> Result<u64, ToolError> {
    field(args, name)?
        .as_u64()
        .ok_or_else(|| ToolError::bad(format!("\"{name}\" must be a non-negative integer")))
}

pub fn opt_u64(args: &Value, name: &str) -> Result<Option<u64>, ToolError> {
    opt(args, name)
        .map(|v| {
            v.as_u64()
                .ok_or_else(|| ToolError::bad(format!("\"{name}\" must be a non-negative integer")))
        })
        .transpose()
}

pub fn req_str<'a>(args: &'a Value, name: &str) -> Result<&'a str, ToolError> {
    field(args, name)?
        .as_str()
        .ok_or_else(|| ToolError::bad(format!("\"{name}\" must be a string")))
}

pub fn opt_str<'a>(args: &'a Value, name: &str) -> Option<&'a str> {
    opt(args, name).and_then(Value::as_str)
}

pub fn opt_bool(args: &Value, name: &str) -> Result<Option<bool>, ToolError> {
    opt(args, name)
        .map(|v| {
            v.as_bool()
                .ok_or_else(|| ToolError::bad(format!("\"{name}\" must be true or false")))
        })
        .transpose()
}

pub fn req_array<'a>(args: &'a Value, name: &str) -> Result<&'a [Value], ToolError> {
    field(args, name)?
        .as_array()
        .map(Vec::as_slice)
        .ok_or_else(|| ToolError::bad(format!("\"{name}\" must be an array")))
}

pub fn vec2_of(v: &Value, name: &str) -> Result<Vec2, ToolError> {
    let bad = || ToolError::bad(format!("\"{name}\" must be a point [x, y], got {v}"));
    match v {
        Value::Array(a) if a.len() == 2 => {
            let x = a[0].as_f64().ok_or_else(bad)?;
            let y = a[1].as_f64().ok_or_else(bad)?;
            if !x.is_finite() || !y.is_finite() {
                return Err(bad());
            }
            Ok(Vec2::new(x, y))
        }
        Value::Object(o) => {
            let x = o.get("x").and_then(Value::as_f64).ok_or_else(bad)?;
            let y = o.get("y").and_then(Value::as_f64).ok_or_else(bad)?;
            Ok(Vec2::new(x, y))
        }
        _ => Err(bad()),
    }
}

pub fn req_vec2(args: &Value, name: &str) -> Result<Vec2, ToolError> {
    vec2_of(field(args, name)?, name)
}

pub fn vec3_of(v: &Value, name: &str) -> Result<Vec3, ToolError> {
    let bad = || ToolError::bad(format!("\"{name}\" must be a point [x, y, z], got {v}"));
    match v {
        Value::Array(a) if a.len() == 3 => {
            let mut out = [0.0; 3];
            for (slot, item) in out.iter_mut().zip(a) {
                *slot = item.as_f64().filter(|x| x.is_finite()).ok_or_else(bad)?;
            }
            Ok(Vec3::from_array(out))
        }
        _ => Err(bad()),
    }
}

pub fn req_vec3(args: &Value, name: &str) -> Result<Vec3, ToolError> {
    vec3_of(field(args, name)?, name)
}
