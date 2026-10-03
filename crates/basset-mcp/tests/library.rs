//! The library of test geometries in `testcases/library/`.
//!
//! Each file is a script of MCP tool calls that builds a model, plus what the model must
//! come out as: which features are ok, how many regions a sketch encloses, and each
//! body's volume, bounding box, face count and whether its shell is closed. Volumes and
//! boxes are what a kernel test asserts on because they catch winding, orientation and
//! boolean errors cheaply; a region count catches the sketch side. The files double as
//! examples of driving the modeller over the protocol, and `make geometries` runs them.
//!
//! File format:
//!
//! ```json
//! {
//!   "description": "what this exercises",
//!   "calls": [ { "tool": "create_sketch", "arguments": { "plane": "XY" } }, ... ],
//!   "expect": {
//!     "ok": true,                       // check_document reports no error
//!     "warnings": 0,                    // optional: exact number of warned features
//!     "statuses": { "$3": "ok" },       // feature statuses, by call index
//!     "regions": { "$1": 5 },           // closed regions a sketch encloses
//!     "bodies": {
//!       "$2": { "volume": 200, "volume_tol": 0.01, "closed": true,
//!               "aabb": [[0, 0, 0], [10, 5, 4]], "faces": 6 }
//!     }
//!   }
//! }
//! ```
//!
//! `$N` in `expect` refers to call N's `feature` (or `body`), as it does in `calls`.

use std::path::{Path, PathBuf};

use basset_mcp::Session;
use serde_json::{Value, json};

fn library_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testcases/library")
}

/// Call N's feature id, or the number itself.
fn feature_of(key: &str, results: &[Value]) -> Result<u64, String> {
    if let Some(reference) = key.strip_prefix('$') {
        let index: usize = reference
            .parse()
            .map_err(|_| format!("bad reference {key:?}"))?;
        let result = results
            .get(index)
            .ok_or_else(|| format!("{key} refers to call {index}, which did not run"))?;
        result["result"]["feature"]
            .as_u64()
            .or_else(|| result["result"]["sketch"].as_u64())
            .ok_or_else(|| format!("call {index} made no feature: {}", result["result"]))
    } else {
        key.parse().map_err(|_| format!("bad feature id {key:?}"))
    }
}

fn close(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol
}

/// Runs one library file and returns every way the model disagreed with the file.
fn run_case(path: &Path) -> Vec<String> {
    let text = std::fs::read_to_string(path).unwrap();
    let case: Value =
        serde_json::from_str(&text).unwrap_or_else(|e| panic!("{}: not JSON: {e}", path.display()));
    let mut session = Session::new();
    let mut failures = Vec::new();
    let script = session
        .call(
            "run_script",
            &json!({ "calls": case["calls"], "continue_on_error": true }),
        )
        .unwrap();
    let results = script["results"].as_array().cloned().unwrap_or_default();
    for r in &results {
        if r["ok"] == false {
            failures.push(format!(
                "call {} ({}) failed: {}",
                r["call"], r["tool"], r["error"]
            ));
        }
    }
    let expect = &case["expect"];
    let check = session.call("check_document", &json!({})).unwrap();
    if let Some(ok) = expect["ok"].as_bool()
        && check["ok"] != ok
    {
        failures.push(format!("expected ok={ok}, check_document says {}", check));
    }
    if let Some(n) = expect["warnings"].as_u64() {
        let warned = check["problems"]
            .as_array()
            .map(|p| p.iter().filter(|p| p["severity"] == "warning").count())
            .unwrap_or(0);
        if warned as u64 != n {
            failures.push(format!(
                "expected {n} warning(s), got {}: {}",
                warned, check["problems"]
            ));
        }
    }
    let info = session.call("document_info", &json!({})).unwrap();
    if let Some(statuses) = expect["statuses"].as_object() {
        for (key, want) in statuses {
            match feature_of(key, &results) {
                Ok(id) => {
                    let got = info["features"]
                        .as_array()
                        .and_then(|f| f.iter().find(|f| f["id"] == id))
                        .map(|f| f["status"].clone())
                        .unwrap_or(Value::Null);
                    if &got != want {
                        failures.push(format!(
                            "feature {key} (id {id}): expected status {want}, got {got}"
                        ));
                    }
                }
                Err(e) => failures.push(e),
            }
        }
    }
    if let Some(regions) = expect["regions"].as_object() {
        for (key, want) in regions {
            match feature_of(key, &results) {
                Ok(id) => {
                    match session.call("sketch_info", &json!({ "sketch": id })) {
                        Ok(sk) => {
                            let got = sk["regions"].as_array().map_or(0, Vec::len) as u64;
                            if Some(got) != want.as_u64() {
                                failures.push(format!("sketch {key} (id {id}): expected {want} region(s), got {got}: {}", sk["regions"]));
                            }
                        }
                        Err(e) => failures.push(format!("sketch {key}: {e}")),
                    }
                }
                Err(e) => failures.push(e),
            }
        }
    }
    if let Some(bodies) = expect["bodies"].as_object() {
        for (key, want) in bodies {
            let id = match feature_of(key, &results) {
                Ok(id) => id,
                Err(e) => {
                    failures.push(e);
                    continue;
                }
            };
            let body = match session.call("body_info", &json!({ "body": id })) {
                Ok(b) => b,
                Err(e) => {
                    failures.push(format!("body {key} (id {id}): {e}"));
                    continue;
                }
            };
            let tag = format!("body {key} (id {id})");
            if let Some(v) = want["volume"].as_f64() {
                let tol = want["volume_tol"].as_f64().unwrap_or(1e-6);
                let got = body["volume"].as_f64().unwrap_or(f64::NAN);
                if !close(got, v, tol) {
                    failures.push(format!("{tag}: expected volume {v} ± {tol}, got {got}"));
                }
            }
            if let Some(closed) = want["closed"].as_bool()
                && body["closed"] != closed
            {
                failures.push(format!(
                    "{tag}: expected closed={closed}, got {} ({})",
                    body["closed"], body["validation"]
                ));
            }
            if let Some(faces) = want["faces"].as_u64()
                && body["faces"] != faces
            {
                failures.push(format!(
                    "{tag}: expected {faces} faces, got {}",
                    body["faces"]
                ));
            }
            if let Some(aabb) = want["aabb"].as_array() {
                let tol = want["aabb_tol"].as_f64().unwrap_or(1e-6);
                let got = &body["aabb"];
                for (which, corner) in ["min", "max"].iter().zip(aabb) {
                    for (axis, w) in corner.as_array().into_iter().flatten().enumerate() {
                        let g = got[which][axis].as_f64().unwrap_or(f64::NAN);
                        if !close(g, w.as_f64().unwrap_or(f64::NAN), tol) {
                            failures
                                .push(format!("{tag}: aabb {which}[{axis}] expected {w}, got {g}"));
                        }
                    }
                }
            }
        }
    }
    failures
}

#[test]
fn every_library_geometry_builds_as_its_file_says() {
    let mut files: Vec<PathBuf> = std::fs::read_dir(library_dir())
        .expect("testcases/library exists")
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.extension().is_some_and(|e| e == "json"))
        .collect();
    files.sort();
    assert!(!files.is_empty(), "the library is empty");
    let mut report = String::new();
    for file in &files {
        let failures = run_case(file);
        if !failures.is_empty() {
            report.push_str(&format!(
                "\n{}:\n",
                file.file_name().unwrap().to_string_lossy()
            ));
            for f in failures {
                report.push_str(&format!("  - {f}\n"));
            }
        }
    }
    assert!(report.is_empty(), "{report}");
}
