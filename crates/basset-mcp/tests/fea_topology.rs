//! The `fea_topology` tool, driven over the same entry point `tools/call` uses.

use basset_mcp::Session;
use serde_json::json;

/// A 100 × 10 × 10 bar along x, sketched on the XY plane and extruded 10 up: its body id
/// and the keys of its −x and +x end faces. The same bar `tests/fea.rs` uses.
fn bar(session: &mut Session) -> (u64, String, String) {
    let sketch = session
        .call("create_sketch", &json!({ "plane": "XY" }))
        .unwrap()["feature"]
        .as_u64()
        .unwrap();
    session
        .call(
            "sketch_ops",
            &json!({ "sketch": sketch, "ops": [{ "op": "rectangle", "a": [0, 0], "b": [100, 10] }] }),
        )
        .unwrap();
    let extrude = session
        .call(
            "extrude",
            &json!({ "sketch": sketch, "regions": "all", "distance": 10 }),
        )
        .unwrap();
    let body = extrude["body"].as_u64().unwrap();
    let faces = session
        .call("body_info", &json!({ "body": body, "faces": true }))
        .unwrap()["face_list"]
        .as_array()
        .unwrap()
        .clone();
    // The side faces are keyed by the sketch's curve slots, so pick the ends by where
    // they are rather than by name.
    let at_x = |x: f64| {
        faces
            .iter()
            .find(|f| (f["centroid"][0].as_f64().unwrap() - x).abs() < 1e-6)
            .unwrap()["key"]
            .as_str()
            .unwrap()
            .to_string()
    };
    (body, at_x(0.0), at_x(100.0))
}

#[test]
fn a_cantilever_is_hollowed_to_the_asked_fraction_and_gets_stiffer() {
    let mut session = Session::new();
    let (body, root, tip) = bar(&mut session);
    let dir = std::env::temp_dir().join(format!("basset-fea-topology-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("bar.vtk");
    let out = session
        .call(
            "fea_topology",
            &json!({
                "body": body,
                "fixed": [root],
                "loads": [{ "face": tip, "force": [0, 0, -100] }],
                "element_size": 2.5,
                "volume_fraction": 0.4,
                "iterations": 10,
                "export": path.to_str().unwrap(),
            }),
        )
        .unwrap();
    assert_eq!(out["body"], body);
    assert_eq!(out["elements"], 640);
    assert_eq!(out["nodes"], 41 * 5 * 5);
    assert_eq!(out["element_size"], json!([2.5, 2.5, 2.5]));
    let iterations = out["iterations"].as_u64().unwrap();
    assert!((1..=10).contains(&iterations), "{out}");
    let fraction = out["volume_fraction"].as_f64().unwrap();
    assert!((fraction - 0.4).abs() < 0.02 * 0.4, "{out}");
    let first = out["compliance"]["first"].as_f64().unwrap();
    let last = out["compliance"]["last"].as_f64().unwrap();
    assert!(last < first, "{out}");
    // The kept volume at the default threshold is a loose bound either side of the
    // fraction: a design ten updates in still has elements of middling density.
    let kept = out["kept_volume"].as_f64().unwrap();
    assert!(kept > 0.2 * 10_000.0 && kept < 0.7 * 10_000.0, "{out}");
    assert!(out["max_von_mises"]["mpa"].as_f64().unwrap() > 0.0, "{out}");
    assert!(
        out["max_displacement"]["mm"].as_f64().unwrap() > 0.0,
        "{out}"
    );
    assert_eq!(out["max_displacement"]["at"].as_array().unwrap().len(), 3);
    assert_eq!(out["exported"], path.to_str().unwrap());
    let text = std::fs::read_to_string(&path).unwrap();
    assert!(text.starts_with("# vtk DataFile"));
    assert!(text.contains("SCALARS density double 1"));
    std::fs::remove_dir_all(&dir).unwrap();
}

#[test]
fn bad_fractions_and_faces_are_named() {
    let mut session = Session::new();
    let (body, root, tip) = bar(&mut session);
    let err = session
        .call(
            "fea_topology",
            &json!({
                "body": body,
                "fixed": [root],
                "loads": [{ "face": tip, "pressure": 1 }],
                "element_size": 5,
                "volume_fraction": 1.2,
            }),
        )
        .unwrap_err();
    assert!(err.0.contains("volume fraction must lie in"), "{err}");

    let err = session
        .call(
            "fea_topology",
            &json!({ "body": body, "fixed": ["99.0:EndCap"], "loads": [{ "face": tip, "pressure": 1 }] }),
        )
        .unwrap_err();
    assert!(
        err.0.contains("no exposed element facet lies on face"),
        "{err}"
    );
}
