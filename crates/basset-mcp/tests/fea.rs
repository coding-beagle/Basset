//! The `fea_static` tool, driven over the same entry point `tools/call` uses.

use basset_mcp::Session;
use serde_json::{Value, json};

/// A 100 × 10 × 10 bar along x, sketched on the XY plane and extruded 10 up: its body id
/// and the keys of its −x and +x end faces.
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
fn a_cantilever_deflects_as_beam_theory_says() {
    let mut session = Session::new();
    let (body, root, tip) = bar(&mut session);
    let out = session
        .call(
            "fea_static",
            &json!({
                "body": body,
                "fixed": [root],
                "loads": [{ "face": tip, "force": [0, 0, -100] }],
                "element_size": 2.0,
            }),
        )
        .unwrap();
    assert_eq!(out["elements"], 1250);
    // PL³/(3EI) = 0.2 mm, with bricks five deep a little stiff.
    let d = out["max_displacement"]["mm"].as_f64().unwrap();
    assert!((d - 0.2).abs() / 0.2 < 0.1, "{out}");
    assert!(
        (out["reaction"][2].as_f64().unwrap() - 100.0).abs() < 1e-3,
        "{out}"
    );
    assert!(
        out["max_von_mises"]["mpa"].as_f64().unwrap() > 40.0,
        "{out}"
    );
}

#[test]
fn a_pressure_exports_vtk_and_a_bad_face_is_named() {
    let mut session = Session::new();
    let (body, root, tip) = bar(&mut session);
    let dir = std::env::temp_dir().join(format!("basset-fea-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("bar.vtk");
    let out = session
        .call(
            "fea_static",
            &json!({
                "body": body,
                "fixed": [root],
                "loads": [{ "face": tip, "pressure": -5 }],
                "material": "aluminium",
                "export": path.to_str().unwrap(),
            }),
        )
        .unwrap();
    // −5 MPa pulls the +x end outward over 100 mm²: 500 N along +x, reacted along −x.
    assert!(
        (out["reaction"][0].as_f64().unwrap() + 500.0).abs() < 1e-3,
        "{out}"
    );
    assert_eq!(out["material"]["youngs_modulus"], 69000.0);
    let text = std::fs::read_to_string(&path).unwrap();
    assert!(text.starts_with("# vtk DataFile"));
    std::fs::remove_dir_all(&dir).unwrap();

    let err = session
        .call(
            "fea_static",
            &json!({ "body": body, "fixed": ["99.0:EndCap"], "loads": [{ "face": tip, "pressure": 1 }] }),
        )
        .unwrap_err();
    assert!(
        err.0.contains("no exposed element facet lies on face"),
        "{err}"
    );

    let err = session
        .call(
            "fea_static",
            &json!({ "body": body, "fixed": [root], "loads": [{ "face": tip }] }),
        )
        .unwrap_err();
    assert!(err.0.contains("exactly one of"), "{err}");
    let _: Value = out;
}

#[test]
fn a_named_material_reports_its_name_mass_and_safety_factor() {
    let mut session = Session::new();
    let (body, root, tip) = bar(&mut session);
    let out = session
        .call(
            "fea_static",
            &json!({
                "body": body,
                "fixed": [root],
                "loads": [{ "face": tip, "force": [0, 0, -100] }],
                "material": "6061-T6",
                "element_size": 2.0,
            }),
        )
        .unwrap();
    assert_eq!(out["material"]["name"], "6061-T6", "{out}");
    assert_eq!(out["material"]["group"], "Aluminium");
    assert_eq!(out["material"]["youngs_modulus"], 68900.0);
    assert_eq!(out["material"]["yield_strength"], 276.0);
    // 10 000 mm³ of aluminium at 2.70 g/cm³ is 27 g; the bar meshes exactly.
    assert!(
        (out["mass_kg"].as_f64().unwrap() - 0.027).abs() < 1e-9,
        "{out}"
    );
    let smax = out["max_von_mises"]["mpa"].as_f64().unwrap();
    let sf = out["safety_factor"].as_f64().unwrap();
    assert!((sf - 276.0 / smax).abs() < 1e-9, "{out}");

    // Spelling is forgiven; an overridden modulus is no longer a library entry, and says
    // so by naming nothing, but keeps the preset's density and yield.
    let out = session
        .call(
            "fea_static",
            &json!({
                "body": body,
                "fixed": [root],
                "loads": [{ "face": tip, "force": [0, 0, -100] }],
                "material": "al 6061 t6",
                "youngs_modulus": 70000,
                "element_size": 5.0,
            }),
        )
        .unwrap();
    assert!(out["material"]["name"].is_null(), "{out}");
    assert_eq!(out["material"]["density"], 2.7);
    assert!(out["safety_factor"].is_number(), "{out}");

    // The default is generic steel, which has no yield strength and so no safety factor.
    let out = session
        .call(
            "fea_static",
            &json!({
                "body": body,
                "fixed": [root],
                "loads": [{ "face": tip, "force": [0, 0, -100] }],
                "element_size": 5.0,
            }),
        )
        .unwrap();
    assert_eq!(out["material"]["name"], "Steel");
    assert!(out["material"]["yield_strength"].is_null());
    assert!(out.get("safety_factor").is_none(), "{out}");
}

#[test]
fn the_material_library_lists_and_filters() {
    let mut session = Session::new();
    let all = session.call("fea_materials", &json!({})).unwrap();
    let all = all["materials"].as_array().unwrap();
    assert!(all.len() >= 20, "{}", all.len());
    for m in all {
        assert!(m["name"].is_string() && m["group"].is_string(), "{m}");
        assert!(m["youngs_modulus"].as_f64().unwrap() > 0.0, "{m}");
        assert!(m["density"].as_f64().unwrap() > 0.0, "{m}");
    }

    let plastics = session
        .call("fea_materials", &json!({ "group": "plastic" }))
        .unwrap();
    let plastics = plastics["materials"].as_array().unwrap();
    assert!(plastics.len() >= 5 && plastics.len() < all.len());
    assert!(
        plastics.iter().all(|m| m["group"] == "Plastic"),
        "{plastics:?}"
    );

    let stainless = session
        .call(
            "fea_materials",
            &json!({ "group": "stainless", "query": "316" }),
        )
        .unwrap();
    assert_eq!(stainless["materials"][0]["name"], "Stainless 316");
    assert_eq!(stainless["materials"].as_array().unwrap().len(), 1);

    let err = session
        .call("fea_materials", &json!({ "group": "wood" }))
        .unwrap_err();
    assert!(err.0.contains("unknown material group"), "{err}");
}

#[test]
fn an_unknown_material_is_refused_with_names_to_try() {
    let mut session = Session::new();
    let (body, root, tip) = bar(&mut session);
    let err = session
        .call(
            "fea_static",
            &json!({
                "body": body,
                "fixed": [root],
                "loads": [{ "face": tip, "pressure": 1 }],
                "material": "unobtainium",
            }),
        )
        .unwrap_err();
    assert!(err.0.contains("unknown material \"unobtainium\""), "{err}");
    assert!(err.0.contains("fea_materials"), "{err}");
    assert!(err.0.contains("6061-T6"), "{err}");
}
