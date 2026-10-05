//! Appearances end to end: painting is undoable without touching the model, a slider
//! drag is one step, the stable face names carry paint through edits, and everything is
//! saved.

use std::sync::Arc;

use basset_core::{
    BodyOp, BodyRef, ComponentId, Document, Extent, FaceRole, FeatureId, FeatureKind, OriginPlane,
    PlaneRef, ProfileRef, RegionRef, Sketch, file, trace_scene,
};
use basset_math::Vec2;
use basset_render::{Appearance, Background, EnvironmentKind, SceneSettings, Srgb, find};
use basset_sketch::shapes;

fn block(height: f64) -> (Document, FeatureId) {
    let mut doc = Document::new("painted");
    let mut s = Sketch::new();
    shapes::rectangle_two_point(&mut s, Vec2::ZERO, Vec2::new(10.0, 5.0));
    let sketch = doc.add_feature(FeatureKind::Sketch {
        plane: PlaneRef::Origin(OriginPlane::XY),
        component: ComponentId::ROOT,
        sketch: s,
    });
    let body = doc.add_feature(FeatureKind::Extrude {
        regions: vec![RegionRef::Profile(ProfileRef::new(
            sketch,
            Vec2::new(1.0, 1.0),
        ))],
        extent: Extent::OneSide(height),
        operation: BodyOp::NewBody,
        component: ComponentId::ROOT,
    });
    (doc, body)
}

fn red() -> &'static Appearance {
    find("Paint - Gloss Red").unwrap()
}

#[test]
fn painting_undoes_and_redoes_without_regenerating_the_model() {
    let (mut doc, body) = block(4.0);
    let solid_before = doc.state().body(BodyRef(body)).unwrap().solid.clone();
    let revision = doc.revision();

    doc.set_body_appearance(BodyRef(body), Some(red()));
    assert_eq!(
        doc.appearances().body(BodyRef(body)).name,
        "Paint - Gloss Red"
    );
    assert_eq!(
        doc.revision(),
        revision,
        "a colour is not a change to the model"
    );

    assert!(doc.undo());
    assert_eq!(doc.appearances().body(BodyRef(body)), &Appearance::DEFAULT);
    assert_eq!(doc.revision(), revision);
    // Nothing was replayed: the very same solid is still in the state.
    let solid_after = doc.state().body(BodyRef(body)).unwrap().solid.clone();
    assert!(Arc::ptr_eq(&solid_before, &solid_after));

    assert!(doc.redo());
    assert_eq!(
        doc.appearances().body(BodyRef(body)).name,
        "Paint - Gloss Red"
    );
}

#[test]
fn undo_takes_back_the_paint_before_the_edit_before_it() {
    let (mut doc, body) = block(4.0);
    doc.set_parameter("w", "10").unwrap();
    doc.set_body_appearance(BodyRef(body), Some(red()));
    assert!(doc.undo());
    assert!(
        doc.parameters().get("w").is_some(),
        "only the paint came off"
    );
    assert!(doc.undo());
    assert!(doc.parameters().get("w").is_none());
}

#[test]
fn a_run_of_edits_to_one_appearance_is_one_undo_step() {
    let (mut doc, body) = block(4.0);
    doc.set_body_appearance(BodyRef(body), Some(red()));
    let name = red().name.clone();
    for i in 1..=20 {
        let mut a = doc.appearances().get(&name).unwrap().clone();
        a.roughness = i as f32 / 20.0;
        doc.edit_appearance(&name, a).unwrap();
    }
    assert_eq!(doc.appearances().get(&name).unwrap().roughness, 1.0);
    assert!(doc.undo());
    assert_eq!(
        doc.appearances().get(&name).unwrap().roughness,
        red().roughness,
        "one undo takes back the whole drag"
    );
    assert!(doc.undo());
    assert_eq!(doc.appearances().body(BodyRef(body)), &Appearance::DEFAULT);
}

#[test]
fn renaming_an_appearance_carries_its_wearers_and_refuses_a_taken_name() {
    let (mut doc, body) = block(4.0);
    doc.set_body_appearance(BodyRef(body), Some(red()));
    doc.set_default_appearance(find("Chrome"));
    let mut a = red().clone();
    a.name = "Brand red".into();
    doc.edit_appearance(&red().name, a.clone()).unwrap();
    assert_eq!(doc.appearances().body(BodyRef(body)).name, "Brand red");
    a.name = "Chrome".into();
    assert!(doc.edit_appearance("Brand red", a).is_err());
}

#[test]
fn a_painted_face_keeps_its_paint_when_the_extrude_grows() {
    let (mut doc, body) = block(4.0);
    let top = {
        let state = doc.state();
        let solid = &state.body(BodyRef(body)).unwrap().solid;
        solid
            .tessellate()
            .face_keys
            .into_iter()
            .find(|k| k.role == FaceRole::EndCap)
            .unwrap()
    };
    doc.set_face_appearance(BodyRef(body), top, find("Gold - Polished"));
    doc.edit_feature_kind(body, |k| {
        if let FeatureKind::Extrude { extent, .. } = k {
            *extent = Extent::OneSide(9.0);
        }
    })
    .unwrap();
    let state = doc.state();
    let tess = state.body(BodyRef(body)).unwrap().solid.tessellate();
    assert!(tess.face_keys.contains(&top), "the top is still the top");
    assert_eq!(
        doc.appearances().face(BodyRef(body), top).name,
        "Gold - Polished"
    );
}

#[test]
fn appearances_and_the_scene_survive_a_file() {
    let (mut doc, body) = block(4.0);
    doc.set_body_appearance(BodyRef(body), Some(red()));
    let scene = SceneSettings {
        environment: EnvironmentKind::Dusk,
        background: Background::Solid(Srgb::hex(0x101820)),
        brightness: 0.5,
        ..SceneSettings::default()
    };
    doc.set_scene(scene.clone());
    let mut bytes = Vec::new();
    file::write(&mut bytes, &doc).unwrap();
    let back = file::read(bytes.as_slice()).unwrap();
    assert_eq!(back.appearances(), doc.appearances());
    assert_eq!(*back.scene(), scene);
}

#[test]
fn scene_settings_are_not_undone() {
    let (mut doc, body) = block(4.0);
    doc.set_body_appearance(BodyRef(body), Some(red()));
    let dusk = SceneSettings {
        environment: EnvironmentKind::Dusk,
        ..SceneSettings::default()
    };
    doc.set_scene(dusk.clone());
    assert!(doc.undo());
    assert_eq!(*doc.scene(), dusk);
}

#[test]
fn a_version_11_file_opens_unpainted_in_the_photo_booth() {
    let file = serde_json::json!({
        "format_version": 11,
        "generator": "basset test",
        "document": {
            "name": "old",
            "units": "Millimeters",
            "timeline": { "features": [], "cursor": 0, "next_id": 1 },
        }
    });
    let doc = file::read(serde_json::to_string(&file).unwrap().as_bytes()).unwrap();
    assert!(doc.appearances().is_empty());
    assert_eq!(*doc.scene(), SceneSettings::default());
}

#[test]
fn the_trace_scene_takes_each_faces_appearance() {
    let (mut doc, body) = block(4.0);
    doc.set_body_appearance(BodyRef(body), Some(red()));
    let state = doc.state();
    let tess = state.body(BodyRef(body)).unwrap().solid.tessellate();
    let scene = trace_scene([(BodyRef(body), &tess)], doc.appearances(), doc.scene());
    assert_eq!(scene.triangle_count(), tess.mesh.triangle_count());
}
