//! End-to-end behaviour of the parametric timeline: edits in the past propagate forward,
//! references survive those edits, and failures are reported per feature.

use std::f64::consts::PI;

use approx::assert_relative_eq;
use basset_core::{
    AxisRef, BodyOp, BodyRef, CombineOp, ComponentId, Document, EdgeRef, Extent, FaceKey, FaceRef,
    FaceRole, FeatureId, FeatureKind, FeatureStatus, OriginAxis, OriginPlane, PathRef, PlaneRef,
    ProfileRef, RegionRef, Sketch,
};
use basset_kernel::{EdgeKey, OpId};
use basset_math::{Affine3, Vec2, Vec3};
use basset_sketch::{Constraint, ConstraintId, shapes};

/// Rectangle from the origin to (w, h) with a driving width dimension so the sketch can
/// be re-dimensioned the way a user would.
fn rect_sketch(w: f64, h: f64) -> (Sketch, ConstraintId) {
    let mut s = Sketch::new();
    let r = shapes::rectangle_two_point(&mut s, Vec2::ZERO, Vec2::new(w, h));
    s.add_constraint(Constraint::Fix(r.corners[0])).unwrap();
    let width = s
        .add_constraint(Constraint::HorizontalDistance {
            a: r.corners[0],
            b: r.corners[1],
            value: w,
        })
        .unwrap();
    s.add_constraint(Constraint::VerticalDistance {
        a: r.corners[0],
        b: r.corners[3],
        value: h,
    })
    .unwrap();
    (s, width)
}

fn sketch_on(doc: &mut Document, plane: PlaneRef, sketch: Sketch) -> FeatureId {
    doc.add_feature(FeatureKind::Sketch {
        plane,
        component: ComponentId::ROOT,
        sketch,
    })
}

fn extrude(
    doc: &mut Document,
    sketch: FeatureId,
    sample: Vec2,
    distance: f64,
    op: BodyOp,
) -> FeatureId {
    doc.add_feature(FeatureKind::Extrude {
        regions: vec![RegionRef::Profile(ProfileRef { sketch, sample })],
        extent: Extent::OneSide(distance),
        operation: op,
        component: ComponentId::ROOT,
    })
}

fn volume(doc: &mut Document, body: FeatureId) -> f64 {
    doc.state()
        .body(BodyRef(body))
        .expect("body exists")
        .solid
        .volume()
}

fn face(body: FeatureId, role: FaceRole) -> FaceKey {
    FaceKey::new(OpId::new(body.0), role)
}

/// A 10×5 rectangle extruded 4 with a fillet on the top front edge.
fn block_with_fillet() -> (Document, FeatureId, FeatureId, FeatureId, ConstraintId) {
    let mut doc = Document::new("test");
    let (sketch, width) = rect_sketch(10.0, 5.0);
    let sk = sketch_on(&mut doc, PlaneRef::Origin(OriginPlane::XY), sketch);
    let ex = extrude(&mut doc, sk, Vec2::new(1.0, 1.0), 4.0, BodyOp::NewBody);
    let side = doc
        .state()
        .body(BodyRef(ex))
        .unwrap()
        .solid
        .edges()
        .iter()
        .map(|e| e.key)
        .find(|k| {
            k.touches(face(ex, FaceRole::EndCap)) && k.a != k.b && {
                // The edge along y = 0: both endpoints have y ≈ 0.
                let other = if k.a.role == FaceRole::EndCap {
                    k.b
                } else {
                    k.a
                };
                let solid = &doc.state().body(BodyRef(ex)).unwrap().solid;
                solid.face(other).unwrap().centroid().y.abs() < 1e-9
            }
        })
        .expect("top front edge");
    let fi = doc.add_feature(FeatureKind::Fillet {
        edges: vec![EdgeRef {
            body: BodyRef(ex),
            key: side,
        }],
        radius: 1.0,
    });
    (doc, sk, ex, fi, width)
}

#[test]
fn extrude_from_sketch_produces_body() {
    let (mut doc, _, ex, fi, _) = block_with_fillet();
    let state = doc.state();
    assert_eq!(state.status(fi), Some(&FeatureStatus::Ok));
    let solid = &state.body(BodyRef(ex)).unwrap().solid;
    let expected = 200.0 - (1.0 - PI / 4.0) * 10.0;
    assert_relative_eq!(solid.volume(), expected, epsilon = 0.05);
    assert!(solid.is_closed());
    assert!(
        solid
            .face(FaceKey::new(OpId::new(fi.0), FaceRole::Fillet(0)))
            .is_some()
    );
}

#[test]
fn editing_extrude_distance_propagates_to_fillet() {
    let (mut doc, _, ex, fi, _) = block_with_fillet();
    doc.edit_feature_kind(ex, |k| {
        if let FeatureKind::Extrude { extent, .. } = k {
            *extent = Extent::OneSide(9.0);
        }
    })
    .unwrap();
    let state = doc.state();
    assert_eq!(
        state.status(fi),
        Some(&FeatureStatus::Ok),
        "fillet edge reference survived"
    );
    let solid = &state.body(BodyRef(ex)).unwrap().solid;
    assert_relative_eq!(
        solid.volume(),
        450.0 - (1.0 - PI / 4.0) * 10.0,
        epsilon = 0.05
    );
    assert_relative_eq!(solid.aabb().max.z, 9.0);
}

#[test]
fn editing_sketch_dimension_propagates_forward() {
    let (mut doc, sk, ex, fi, width) = block_with_fillet();
    doc.edit_feature_kind(sk, |k| {
        if let FeatureKind::Sketch { sketch, .. } = k {
            sketch.set_dimension_value(width, 20.0).unwrap();
        }
    })
    .unwrap();
    let state = doc.state();
    assert_eq!(state.status(fi), Some(&FeatureStatus::Ok));
    let solid = &state.body(BodyRef(ex)).unwrap().solid;
    // Width doubled: the filleted edge along the front is now 20 long. The faceted arc
    // removes slightly more than the true arc, hence the tolerance.
    assert_relative_eq!(
        solid.volume(),
        400.0 - (1.0 - PI / 4.0) * 20.0,
        epsilon = 0.1
    );
}

#[test]
fn rollback_inserts_at_cursor_and_replays_forward() {
    let (mut doc, _, ex, fi, _) = block_with_fillet();
    assert_eq!(doc.timeline().cursor(), 3);
    doc.set_cursor(2);
    assert!(
        doc.state().status(fi).is_none(),
        "rolled-back features are not evaluated"
    );
    // Face frames are centred on the face, so the origin is the middle of the top.
    let cut_sketch = {
        let mut s = Sketch::new();
        shapes::circle_center(&mut s, Vec2::ZERO, 1.0);
        s
    };
    let top = PlaneRef::Face(FaceRef {
        body: BodyRef(ex),
        key: face(ex, FaceRole::EndCap),
    });
    let sk2 = sketch_on(&mut doc, top, cut_sketch);
    let hole = extrude(
        &mut doc,
        sk2,
        Vec2::ZERO,
        -4.0,
        BodyOp::Cut(vec![BodyRef(ex)]),
    );
    assert_eq!(doc.timeline().index_of(hole), Some(3));
    assert_eq!(
        doc.timeline().index_of(fi),
        Some(4),
        "fillet moved after the inserted features"
    );
    doc.set_cursor(doc.timeline().len());
    let state = doc.state();
    assert_eq!(state.status(fi), Some(&FeatureStatus::Ok));
    let solid = &state.body(BodyRef(ex)).unwrap().solid;
    assert_relative_eq!(
        solid.volume(),
        200.0 - PI * 4.0 - (1.0 - PI / 4.0) * 10.0,
        epsilon = 0.2
    );
    assert!(solid.is_closed());
}

#[test]
fn suppressing_a_feature_skips_it_and_fails_dependants() {
    let (mut doc, _, ex, fi, _) = block_with_fillet();
    doc.set_suppressed(fi, true).unwrap();
    assert_eq!(doc.state().status(fi), Some(&FeatureStatus::Suppressed));
    assert_relative_eq!(volume(&mut doc, ex), 200.0, epsilon = 1e-9);

    doc.set_suppressed(fi, false).unwrap();
    doc.set_suppressed(ex, true).unwrap();
    let state = doc.state();
    assert!(state.body(BodyRef(ex)).is_none());
    assert!(matches!(state.status(fi), Some(FeatureStatus::Failed(_))));
}

#[test]
fn an_under_constrained_sketch_that_something_builds_from_warns_without_failing() {
    // `rect_sketch` pins one corner and dimensions both sides, so nothing about it is
    // loose. Replacing it with a rectangle held only by its width leaves it free to
    // translate and resize, and an extrude builds from it — which must warn, and only
    // warn: the feature still builds, and so does everything after it.
    let (mut doc, sk, ex, _, _) = block_with_fillet();
    assert_eq!(doc.state().status(sk), Some(&FeatureStatus::Ok));

    let mut loose = Sketch::new();
    let r = shapes::rectangle_two_point(&mut loose, Vec2::ZERO, Vec2::new(10.0, 5.0));
    loose
        .add_constraint(Constraint::HorizontalDistance {
            a: r.corners[0],
            b: r.corners[1],
            value: 10.0,
        })
        .unwrap();
    doc.edit_feature_kind(sk, |kind| {
        if let FeatureKind::Sketch { sketch, .. } = kind {
            *sketch = loose;
        }
    })
    .unwrap();

    let state = doc.state();
    let warned: Vec<FeatureId> = state.warned_features().map(|(id, _)| id).collect();
    assert_eq!(warned, vec![sk]);
    let (_, message) = state.warned_features().next().unwrap();
    assert!(message.contains("degrees of freedom"), "{message}");
    assert!(state.failed_features().next().is_none());
    assert!(state.body(BodyRef(ex)).is_some(), "extrude still builds");

    // Take the consumer away and the warning goes with it: a loose sketch that nothing
    // builds from is an ordinary drawing in progress, not something to nag about.
    // Suppressing the extrude is enough, since a suppressed feature builds nothing.
    doc.set_suppressed(ex, true).unwrap();
    assert_eq!(doc.state().warned_features().count(), 0);
    doc.set_suppressed(ex, false).unwrap();
    assert_eq!(doc.state().warned_features().count(), 1);
    doc.remove_feature(ex).unwrap();
    let state = doc.state();
    assert_eq!(state.warned_features().count(), 0);
    assert_eq!(state.status(sk), Some(&FeatureStatus::Ok));
}

#[test]
fn deleting_a_referenced_sketch_marks_extrude_failed_but_keeps_replaying() {
    let (mut doc, sk, ex, fi, _) = block_with_fillet();
    // An independent feature after the failure point must still evaluate.
    let plane = doc.add_feature(FeatureKind::OffsetPlane {
        base: PlaneRef::Origin(OriginPlane::XY),
        distance: 3.0,
    });
    doc.remove_feature(sk).unwrap();
    let state = doc.state();
    let failed: Vec<FeatureId> = state.failed_features().map(|(id, _)| id).collect();
    assert_eq!(failed, vec![ex, fi]);
    assert!(state.planes.contains_key(&plane));
    let (_, message) = state.failed_features().next().unwrap();
    assert!(message.contains("sketch"), "{message}");
}

#[test]
fn undo_and_redo_restore_timeline_and_geometry() {
    let (mut doc, _, ex, fi, _) = block_with_fillet();
    let before = volume(&mut doc, ex);
    doc.remove_feature(fi).unwrap();
    assert_eq!(doc.timeline().len(), 2);
    assert_relative_eq!(volume(&mut doc, ex), 200.0, epsilon = 1e-9);
    assert!(doc.undo());
    assert_eq!(doc.timeline().len(), 3);
    assert_relative_eq!(volume(&mut doc, ex), before, epsilon = 1e-9);
    assert!(doc.redo());
    assert_eq!(doc.timeline().len(), 2);
    assert!(!doc.redo());
}

#[test]
fn document_round_trips_through_bass() {
    let (mut doc, _, ex, _, _) = block_with_fillet();
    let before = volume(&mut doc, ex);
    let mut bytes = Vec::new();
    basset_core::file::write(&mut bytes, &doc).unwrap();
    let text = String::from_utf8(bytes.clone()).unwrap();
    assert!(text.contains(&format!(
        "\"format_version\": {}",
        basset_core::file::FORMAT_VERSION
    )));
    let mut loaded = basset_core::file::read(bytes.as_slice()).unwrap();
    assert_eq!(loaded.timeline().len(), 3);
    assert_eq!(loaded.name, "test");
    assert_relative_eq!(volume(&mut loaded, ex), before, epsilon = 1e-9);

    let dir = std::env::temp_dir().join(format!("basset-core-test-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("doc.bass");
    basset_core::file::save(&path, &doc).unwrap();
    let mut from_disk = basset_core::file::load(&path).unwrap();
    assert_relative_eq!(volume(&mut from_disk, ex), before, epsilon = 1e-9);
    std::fs::remove_dir_all(&dir).unwrap();
}

/// A version 1 file names an extrude's inputs `profiles`; loading one must produce a
/// document whose extrude still builds the same body from the same sketch region.
#[test]
fn version_1_files_load_and_regenerate() {
    let (mut doc, _, ex, _, _) = block_with_fillet();
    let expected = volume(&mut doc, ex);
    let mut bytes = Vec::new();
    basset_core::file::write(&mut bytes, &doc).unwrap();
    let mut file: serde_json::Value = serde_json::from_slice(&bytes).unwrap();
    // Rewrite the saved document the way version 1 wrote it.
    file["format_version"] = serde_json::json!(1);
    fn downgrade(value: &mut serde_json::Value) {
        match value {
            serde_json::Value::Array(items) => items.iter_mut().for_each(downgrade),
            serde_json::Value::Object(map) => {
                if let Some(serde_json::Value::Object(body)) = map.get_mut("Extrude")
                    && let Some(serde_json::Value::Array(regions)) = body.remove("regions")
                {
                    let profiles = regions.into_iter().map(|r| r["Profile"].clone()).collect();
                    body.insert("profiles".into(), serde_json::Value::Array(profiles));
                }
                map.iter_mut().for_each(|(_, v)| downgrade(v));
            }
            _ => {}
        }
    }
    downgrade(&mut file["document"]);
    let old = serde_json::to_vec(&file).unwrap();
    assert!(String::from_utf8_lossy(&old).contains("\"profiles\""));

    let mut loaded = basset_core::file::read(old.as_slice()).unwrap();
    assert_eq!(loaded.timeline().len(), 3);
    assert_relative_eq!(volume(&mut loaded, ex), expected, epsilon = 1e-9);
}

#[test]
fn newer_file_versions_are_rejected() {
    let text = r#"{"format_version": 99, "generator": "future", "document": {}}"#;
    let err = basset_core::file::read(text.as_bytes()).unwrap_err();
    assert!(matches!(
        err,
        basset_core::file::FileError::UnsupportedVersion { found: 99, .. }
    ));
}

#[test]
fn offset_and_angled_planes_position_sketches() {
    let mut doc = Document::new("planes");
    let offset = doc.add_feature(FeatureKind::OffsetPlane {
        base: PlaneRef::Origin(OriginPlane::XY),
        distance: 7.0,
    });
    let (sketch, _) = rect_sketch(2.0, 2.0);
    let sk = sketch_on(&mut doc, PlaneRef::Feature(offset), sketch);
    let ex = extrude(&mut doc, sk, Vec2::new(1.0, 1.0), 3.0, BodyOp::NewBody);
    let aabb = doc.state().body(BodyRef(ex)).unwrap().solid.aabb();
    assert_relative_eq!(aabb.min.z, 7.0);
    assert_relative_eq!(aabb.max.z, 10.0);

    // XY rotated 90° about X becomes a plane with normal −Y; extruding along it moves in −Y.
    let angled = doc.add_feature(FeatureKind::AngledPlane {
        base: PlaneRef::Origin(OriginPlane::XY),
        axis: AxisRef::Origin(OriginAxis::X),
        angle: PI / 2.0,
    });
    let (sketch, _) = rect_sketch(2.0, 2.0);
    let sk2 = sketch_on(&mut doc, PlaneRef::Feature(angled), sketch);
    let ex2 = extrude(&mut doc, sk2, Vec2::new(1.0, 1.0), 3.0, BodyOp::NewBody);
    let aabb = doc.state().body(BodyRef(ex2)).unwrap().solid.aabb();
    assert_relative_eq!(aabb.min.y, -3.0, epsilon = 1e-9);
    assert_relative_eq!(aabb.max.y, 0.0, epsilon = 1e-9);
    assert_relative_eq!(aabb.max.z, 2.0, epsilon = 1e-9);
}

#[test]
fn sketch_on_face_and_join() {
    let mut doc = Document::new("join");
    let (sketch, _) = rect_sketch(10.0, 10.0);
    let sk = sketch_on(&mut doc, PlaneRef::Origin(OriginPlane::XY), sketch);
    let base = extrude(&mut doc, sk, Vec2::new(1.0, 1.0), 2.0, BodyOp::NewBody);
    let top = PlaneRef::Face(FaceRef {
        body: BodyRef(base),
        key: face(base, FaceRole::EndCap),
    });
    let boss = {
        let mut s = Sketch::new();
        shapes::circle_center(&mut s, Vec2::ZERO, 2.0);
        s
    };
    let sk2 = sketch_on(&mut doc, top, boss);
    let _ = extrude(
        &mut doc,
        sk2,
        Vec2::ZERO,
        5.0,
        BodyOp::Join(vec![BodyRef(base)]),
    );
    let state = doc.state();
    assert!(
        state.failed_features().next().is_none(),
        "{:?}",
        state.failed_features().collect::<Vec<_>>()
    );
    let solid = &state.body(BodyRef(base)).unwrap().solid;
    assert_relative_eq!(solid.volume(), 200.0 + PI * 4.0 * 5.0, epsilon = 0.5);
    assert!(solid.is_closed());
    // The face frame is centred on the face, so the boss stands at the block's middle.
    assert_relative_eq!(solid.aabb().max.z, 7.0);
    assert_relative_eq!(solid.centroid().x, 5.0, epsilon = 1e-6);
}

/// A planar face of a body is a region in its own right, so a block's top can be pushed
/// further without drawing a sketch on it first.
#[test]
fn extruding_a_planar_face_joins_onto_the_body() {
    let mut doc = Document::new("face extrude");
    let (sketch, _) = rect_sketch(10.0, 10.0);
    let sk = sketch_on(&mut doc, PlaneRef::Origin(OriginPlane::XY), sketch);
    let base = extrude(&mut doc, sk, Vec2::new(1.0, 1.0), 2.0, BodyOp::NewBody);
    let top = FaceRef {
        body: BodyRef(base),
        key: face(base, FaceRole::EndCap),
    };
    let grown = doc.add_feature(FeatureKind::Extrude {
        regions: vec![RegionRef::Face(top)],
        extent: Extent::OneSide(3.0),
        operation: BodyOp::Join(vec![BodyRef(base)]),
        component: ComponentId::ROOT,
    });
    let state = doc.state();
    assert!(
        state.failed_features().next().is_none(),
        "{:?}",
        state.failed_features().collect::<Vec<_>>()
    );
    let solid = &state.body(BodyRef(base)).unwrap().solid;
    assert_relative_eq!(solid.volume(), 100.0 * 5.0, epsilon = 1e-6);
    assert_relative_eq!(solid.aabb().max.z, 5.0, epsilon = 1e-9);
    assert!(solid.is_closed());

    // Editing the block underneath re-runs the face extrude against the new face.
    doc.edit_feature_kind(base, |k| {
        if let FeatureKind::Extrude { extent, .. } = k {
            *extent = Extent::OneSide(6.0);
        }
    })
    .unwrap();
    let state = doc.state();
    assert_eq!(state.status(grown), Some(&FeatureStatus::Ok));
    let solid = &state.body(BodyRef(base)).unwrap().solid;
    assert_relative_eq!(solid.aabb().max.z, 9.0, epsilon = 1e-9);
}

/// Cutting with a face region runs the extrude the other way, which is the push-pull that
/// makes face selection worth having.
#[test]
fn extruding_a_face_inward_cuts_the_body() {
    let mut doc = Document::new("face cut");
    let (sketch, _) = rect_sketch(10.0, 10.0);
    let sk = sketch_on(&mut doc, PlaneRef::Origin(OriginPlane::XY), sketch);
    let base = extrude(&mut doc, sk, Vec2::new(1.0, 1.0), 4.0, BodyOp::NewBody);
    doc.add_feature(FeatureKind::Extrude {
        regions: vec![RegionRef::Face(FaceRef {
            body: BodyRef(base),
            key: face(base, FaceRole::EndCap),
        })],
        extent: Extent::OneSide(-1.5),
        operation: BodyOp::Cut(vec![BodyRef(base)]),
        component: ComponentId::ROOT,
    });
    let state = doc.state();
    assert!(state.failed_features().next().is_none());
    let solid = &state.body(BodyRef(base)).unwrap().solid;
    assert_relative_eq!(solid.volume(), 100.0 * 2.5, epsilon = 1e-6);
}

#[test]
fn combine_cut_consumes_tools() {
    let mut doc = Document::new("combine");
    let (a, _) = rect_sketch(4.0, 4.0);
    let sk = sketch_on(&mut doc, PlaneRef::Origin(OriginPlane::XY), a);
    let target = extrude(&mut doc, sk, Vec2::new(1.0, 1.0), 4.0, BodyOp::NewBody);
    let tool_sketch = {
        let mut s = Sketch::new();
        shapes::rectangle_two_point(&mut s, Vec2::new(1.0, 1.0), Vec2::new(3.0, 3.0));
        s
    };
    let sk2 = sketch_on(&mut doc, PlaneRef::Origin(OriginPlane::XY), tool_sketch);
    let tool = extrude(&mut doc, sk2, Vec2::new(2.0, 2.0), 10.0, BodyOp::NewBody);
    doc.add_feature(FeatureKind::Combine {
        target: BodyRef(target),
        tools: vec![BodyRef(tool)],
        operation: CombineOp::Cut,
        keep_tools: false,
    });
    let state = doc.state();
    assert!(state.body(BodyRef(tool)).is_none());
    assert_relative_eq!(
        state.body(BodyRef(target)).unwrap().solid.volume(),
        64.0 - 16.0,
        epsilon = 1e-9
    );
}

#[test]
fn move_revolve_sweep_and_loft_through_the_document() {
    let mut doc = Document::new("ops");
    // Revolve a 2×3 rectangle sitting at x ∈ [1, 3] about the Y axis.
    let profile = {
        let mut s = Sketch::new();
        shapes::rectangle_two_point(&mut s, Vec2::new(1.0, 0.0), Vec2::new(3.0, 3.0));
        s
    };
    let sk = sketch_on(&mut doc, PlaneRef::Origin(OriginPlane::XY), profile);
    let rev = doc.add_feature(FeatureKind::Revolve {
        regions: vec![RegionRef::Profile(ProfileRef {
            sketch: sk,
            sample: Vec2::new(2.0, 1.0),
        })],
        axis: AxisRef::Origin(OriginAxis::Y),
        angle: 2.0 * PI,
        operation: BodyOp::NewBody,
        component: ComponentId::ROOT,
    });
    assert_relative_eq!(volume(&mut doc, rev), PI * (9.0 - 1.0) * 3.0, epsilon = 0.5);

    doc.add_feature(FeatureKind::Move {
        body: BodyRef(rev),
        transform: Affine3::from_translation(Vec3::new(0.0, 0.0, 50.0)),
    });
    // Facet vertices rarely land exactly on the extreme, so allow the sagitta.
    assert_relative_eq!(
        doc.state().body(BodyRef(rev)).unwrap().solid.aabb().min.z,
        47.0,
        epsilon = 0.01
    );

    // Sweep a circle along an L-shaped path drawn in the XZ plane.
    let path_sketch = {
        let mut s = Sketch::new();
        let a = s.add_point(Vec2::ZERO);
        let b = s.add_point(Vec2::new(10.0, 0.0));
        let c = s.add_point(Vec2::new(10.0, 10.0));
        let l1 = s.add_line(a, b).unwrap();
        let l2 = s.add_line(b, c).unwrap();
        (s, l1, l2)
    };
    let (path_s, l1, l2) = path_sketch;
    let path_sk = sketch_on(&mut doc, PlaneRef::Origin(OriginPlane::XZ), path_s);
    // XZ frame: x → world Z, y → world X, so the path runs along Z then X; the profile
    // must be perpendicular to the first leg: a circle on the XY plane.
    let circle = {
        let mut s = Sketch::new();
        shapes::circle_center(&mut s, Vec2::ZERO, 1.0);
        s
    };
    let circ_sk = sketch_on(&mut doc, PlaneRef::Origin(OriginPlane::XY), circle);
    let sw = doc.add_feature(FeatureKind::Sweep {
        regions: vec![RegionRef::Profile(ProfileRef {
            sketch: circ_sk,
            sample: Vec2::ZERO,
        })],
        path: PathRef {
            sketch: path_sk,
            curves: vec![l1, l2],
        },
        operation: BodyOp::NewBody,
        component: ComponentId::ROOT,
    });
    let status = doc.state().status(sw).cloned();
    assert_eq!(status, Some(FeatureStatus::Ok));
    // A 36-gon circle has 0.5 % less area than the true circle.
    assert_relative_eq!(volume(&mut doc, sw), PI * 20.0, epsilon = 0.5);

    // Loft between two squares on parallel planes.
    let upper = doc.add_feature(FeatureKind::OffsetPlane {
        base: PlaneRef::Origin(OriginPlane::XY),
        distance: 100.0,
    });
    let (sq1, _) = rect_sketch(4.0, 4.0);
    let (sq2, _) = rect_sketch(2.0, 2.0);
    let s1 = sketch_on(&mut doc, PlaneRef::Feature(upper), sq1);
    let upper2 = doc.add_feature(FeatureKind::OffsetPlane {
        base: PlaneRef::Feature(upper),
        distance: 3.0,
    });
    let s2 = sketch_on(&mut doc, PlaneRef::Feature(upper2), sq2);
    let lo = doc.add_feature(FeatureKind::Loft {
        regions: vec![
            RegionRef::Profile(ProfileRef {
                sketch: s1,
                sample: Vec2::ONE,
            }),
            RegionRef::Profile(ProfileRef {
                sketch: s2,
                sample: Vec2::ONE,
            }),
        ],
        operation: BodyOp::NewBody,
        component: ComponentId::ROOT,
    });
    // Frustum-like volume: between the 2×2 prism (12) and the 4×4 prism (48).
    let v = volume(&mut doc, lo);
    assert!(v > 12.0 && v < 48.0, "{v}");
    assert!(doc.state().body(BodyRef(lo)).unwrap().solid.is_closed());
}

#[test]
fn reorder_respects_dependencies() {
    let (mut doc, sk, ex, fi, _) = block_with_fillet();
    assert!(
        doc.reorder_feature(fi, 0).is_err(),
        "fillet cannot precede its extrude"
    );
    assert!(
        doc.reorder_feature(sk, 2).is_err(),
        "sketch cannot follow its extrude"
    );
    let plane = doc.add_feature(FeatureKind::OffsetPlane {
        base: PlaneRef::Origin(OriginPlane::XY),
        distance: 1.0,
    });
    doc.reorder_feature(plane, 0).unwrap();
    assert_eq!(doc.timeline().index_of(plane), Some(0));
    assert_eq!(doc.timeline().index_of(ex), Some(2));
    assert_eq!(doc.state().status(fi), Some(&FeatureStatus::Ok));
}

#[test]
fn edge_keys_are_stable_across_regeneration() {
    let (mut doc, _, ex, _, _) = block_with_fillet();
    let keys_before: Vec<EdgeKey> = doc
        .state()
        .body(BodyRef(ex))
        .unwrap()
        .solid
        .edges()
        .iter()
        .map(|e| e.key)
        .collect();
    doc.edit_feature_kind(ex, |k| {
        if let FeatureKind::Extrude { extent, .. } = k {
            *extent = Extent::Symmetric(6.0);
        }
    })
    .unwrap();
    let keys_after: Vec<EdgeKey> = doc
        .state()
        .body(BodyRef(ex))
        .unwrap()
        .solid
        .edges()
        .iter()
        .map(|e| e.key)
        .collect();
    assert_eq!(keys_before, keys_after);
}

#[test]
fn transactions_group_edits_into_one_undo_step() {
    let (mut doc, _, ex, _, _) = block_with_fillet();
    doc.begin_transaction();
    for d in [5.0, 6.0, 7.0] {
        doc.edit_feature_kind(ex, |k| {
            if let FeatureKind::Extrude { extent, .. } = k {
                *extent = Extent::OneSide(d);
            }
        })
        .unwrap();
    }
    doc.commit_transaction();
    assert_relative_eq!(
        doc.state().body(BodyRef(ex)).unwrap().solid.aabb().max.z,
        7.0
    );
    assert!(doc.undo());
    assert_relative_eq!(
        doc.state().body(BodyRef(ex)).unwrap().solid.aabb().max.z,
        4.0,
        epsilon = 1e-9
    );

    doc.begin_transaction();
    let extra = doc.add_feature(FeatureKind::OffsetPlane {
        base: PlaneRef::Origin(OriginPlane::XY),
        distance: 1.0,
    });
    assert!(doc.timeline().get(extra).is_some());
    doc.rollback_transaction();
    assert!(doc.timeline().get(extra).is_none());
    assert!(!doc.in_transaction());
}

/// A body plus a second sketch whose region will be pushed up to the body's face.
///
/// The base is a 10×5 rectangle extruded 4; the second sketch is a 2×2 rectangle on the
/// same plane, so an extrude of it to the base's top face should be 2·2·4 = 16.
fn base_and_small_sketch() -> (Document, FeatureId, FeatureId) {
    let mut doc = Document::new("test");
    let (sketch, _) = rect_sketch(10.0, 5.0);
    let sk = sketch_on(&mut doc, PlaneRef::Origin(OriginPlane::XY), sketch);
    let base = extrude(&mut doc, sk, Vec2::new(1.0, 1.0), 4.0, BodyOp::NewBody);
    let (small, _) = rect_sketch(2.0, 2.0);
    let sk2 = sketch_on(&mut doc, PlaneRef::Origin(OriginPlane::XY), small);
    (doc, base, sk2)
}

fn extrude_to(doc: &mut Document, sketch: FeatureId, target: FaceRef) -> FeatureId {
    doc.add_feature(FeatureKind::Extrude {
        regions: vec![RegionRef::Profile(ProfileRef {
            sketch,
            sample: Vec2::new(1.0, 1.0),
        })],
        extent: Extent::ToFace(target),
        operation: BodyOp::NewBody,
        component: ComponentId::ROOT,
    })
}

/// The whole point of a to-face extent: the reach is worked out from the target on
/// every replay, so growing the target body grows the extrusion with it.
#[test]
fn an_extrude_to_a_face_reaches_it_and_follows_edits_of_the_target() {
    let (mut doc, base, sk2) = base_and_small_sketch();
    let up = extrude_to(
        &mut doc,
        sk2,
        FaceRef {
            body: BodyRef(base),
            key: face(base, FaceRole::EndCap),
        },
    );
    assert_eq!(doc.state().status(up), Some(&FeatureStatus::Ok));
    assert_relative_eq!(volume(&mut doc, up), 2.0 * 2.0 * 4.0, epsilon = 1e-9);

    doc.edit_feature_kind(base, |k| {
        if let FeatureKind::Extrude { extent, .. } = k {
            *extent = Extent::OneSide(7.0);
        }
    })
    .unwrap();
    assert_relative_eq!(volume(&mut doc, up), 2.0 * 2.0 * 7.0, epsilon = 1e-9);
}

/// The refusals surface as feature errors, not panics: a target the direction runs
/// along, and a target behind the profile, each name their problem and replay goes on.
#[test]
fn an_impossible_to_face_target_fails_the_feature_with_its_reason() {
    // The base's bottom cap shares the profile's own plane, so the reach is zero.
    let (mut doc, base, sk2) = base_and_small_sketch();
    let up = extrude_to(
        &mut doc,
        sk2,
        FaceRef {
            body: BodyRef(base),
            key: face(base, FaceRole::StartCap),
        },
    );
    let state = doc.state();
    let Some(FeatureStatus::Failed(message)) = state.status(up) else {
        panic!("{:?}", state.status(up));
    };
    assert!(message.contains("behind"), "{message}");

    // A sketch on YZ extrudes along x; the top cap's normal is z, at right angles.
    let (mut doc, base, _) = base_and_small_sketch();
    let (side, _) = rect_sketch(2.0, 2.0);
    let sk3 = sketch_on(&mut doc, PlaneRef::Origin(OriginPlane::YZ), side);
    let along = extrude_to(
        &mut doc,
        sk3,
        FaceRef {
            body: BodyRef(base),
            key: face(base, FaceRole::EndCap),
        },
    );
    let state = doc.state();
    let Some(FeatureStatus::Failed(message)) = state.status(along) else {
        panic!("{:?}", state.status(along));
    };
    assert!(message.contains("parallel"), "{message}");
}

/// Deleting the body a to-face extrude reaches for degrades to a per-feature failure,
/// exactly as deleting a referenced sketch does.
#[test]
fn deleting_the_target_body_marks_the_to_face_extrude_failed() {
    let (mut doc, base, sk2) = base_and_small_sketch();
    let up = extrude_to(
        &mut doc,
        sk2,
        FaceRef {
            body: BodyRef(base),
            key: face(base, FaceRole::EndCap),
        },
    );
    doc.remove_feature(base).unwrap();
    let state = doc.state();
    let Some(FeatureStatus::Failed(message)) = state.status(up) else {
        panic!("{:?}", state.status(up));
    };
    assert!(message.contains("does not exist"), "{message}");
}

/// The extent round-trips through the file format and regenerates to the same body.
#[test]
fn a_to_face_extent_survives_a_round_trip_through_bass() {
    let (mut doc, base, sk2) = base_and_small_sketch();
    let target = FaceRef {
        body: BodyRef(base),
        key: face(base, FaceRole::EndCap),
    };
    let up = extrude_to(&mut doc, sk2, target);
    let before = volume(&mut doc, up);
    let mut bytes = Vec::new();
    basset_core::file::write(&mut bytes, &doc).unwrap();
    let mut loaded = basset_core::file::read(bytes.as_slice()).unwrap();
    let Some(FeatureKind::Extrude { extent, .. }) = loaded.timeline().get(up).map(|f| &f.kind)
    else {
        panic!("the extrude is missing after the round trip");
    };
    assert_eq!(*extent, Extent::ToFace(target));
    assert_relative_eq!(volume(&mut loaded, up), before, epsilon = 1e-9);
}

// --- Multi-body boolean targets --------------------------------------------------------

/// A sketch holding one w×h rectangle whose lower-left corner is at `at`.
fn rect_at(at: Vec2, w: f64, h: f64) -> Sketch {
    let mut s = Sketch::new();
    shapes::rectangle_two_point(&mut s, at, at + Vec2::new(w, h));
    s
}

/// Two 10×10×2 plates stacked at z 0..2 and z 2..4, as separate bodies: the shape a cut
/// through a stack meets.
fn stacked_plates(doc: &mut Document) -> (FeatureId, FeatureId) {
    let sk = sketch_on(
        doc,
        PlaneRef::Origin(OriginPlane::XY),
        rect_at(Vec2::ZERO, 10.0, 10.0),
    );
    let lower = extrude(doc, sk, Vec2::new(5.0, 5.0), 2.0, BodyOp::NewBody);
    let plane = doc.add_feature(FeatureKind::OffsetPlane {
        base: PlaneRef::Origin(OriginPlane::XY),
        distance: 2.0,
    });
    let sk2 = sketch_on(
        doc,
        PlaneRef::Feature(plane),
        rect_at(Vec2::ZERO, 10.0, 10.0),
    );
    let upper = extrude(doc, sk2, Vec2::new(5.0, 5.0), 2.0, BodyOp::NewBody);
    (lower, upper)
}

/// One cut listing two bodies removes the same tool from both of them, and the tool's
/// face keys land on each body independently: keys are scoped per body, so the shared
/// operation id cannot collide across them.
#[test]
fn one_cut_through_two_stacked_plates_cuts_both() {
    let mut doc = Document::new("stack");
    let (lower, upper) = stacked_plates(&mut doc);
    let sk = sketch_on(
        &mut doc,
        PlaneRef::Origin(OriginPlane::XY),
        rect_at(Vec2::new(4.0, 4.0), 2.0, 2.0),
    );
    let cut = extrude(
        &mut doc,
        sk,
        Vec2::new(5.0, 5.0),
        10.0,
        BodyOp::Cut(vec![BodyRef(lower), BodyRef(upper)]),
    );
    let state = doc.state();
    assert_eq!(state.status(cut), Some(&FeatureStatus::Ok));
    for body in [lower, upper] {
        let solid = &state.body(BodyRef(body)).unwrap().solid;
        assert_relative_eq!(solid.volume(), 200.0 - 8.0, epsilon = 1e-9);
        assert!(solid.is_closed());
        // The hole's walls on this body are faces the cut's operation id made.
        assert!(
            solid.faces.iter().any(|f| f.key.op.feature == cut.0),
            "body {body} carries no face of the cut"
        );
    }
}

/// One join listing two bodies unions the same tool into both. The bodies stay separate
/// — Fusion merges targets a join bridges, which this kernel-level rule does not attempt
/// — but each of them gains exactly the material of the tool it did not already have.
#[test]
fn one_join_across_two_stacked_plates_adds_to_both() {
    let mut doc = Document::new("stack join");
    let (lower, upper) = stacked_plates(&mut doc);
    let sk = sketch_on(
        &mut doc,
        PlaneRef::Origin(OriginPlane::XY),
        rect_at(Vec2::new(4.0, 4.0), 2.0, 2.0),
    );
    let join = extrude(
        &mut doc,
        sk,
        Vec2::new(5.0, 5.0),
        4.0,
        BodyOp::Join(vec![BodyRef(lower), BodyRef(upper)]),
    );
    let state = doc.state();
    assert_eq!(state.status(join), Some(&FeatureStatus::Ok));
    // The tool is 2×2×4; each plate already holds half of it.
    for body in [lower, upper] {
        let solid = &state.body(BodyRef(body)).unwrap().solid;
        assert_relative_eq!(solid.volume(), 200.0 + 8.0, epsilon = 1e-9);
        assert!(solid.is_closed());
    }
}

/// A body the tool never reaches still takes the boolean, as Fusion applies it: a missed
/// cut leaves that body unchanged, and a missed join keeps the tool as a second disjoint
/// shell of the target rather than refusing. Neither fails the feature.
#[test]
fn a_listed_body_the_tool_misses_takes_the_boolean_anyway() {
    let mut doc = Document::new("miss");
    let sk = sketch_on(
        &mut doc,
        PlaneRef::Origin(OriginPlane::XY),
        rect_at(Vec2::ZERO, 10.0, 10.0),
    );
    let near = extrude(&mut doc, sk, Vec2::new(5.0, 5.0), 2.0, BodyOp::NewBody);
    let sk_far = sketch_on(
        &mut doc,
        PlaneRef::Origin(OriginPlane::XY),
        rect_at(Vec2::new(20.0, 0.0), 10.0, 10.0),
    );
    let far = extrude(&mut doc, sk_far, Vec2::new(25.0, 5.0), 2.0, BodyOp::NewBody);

    let sk_cut = sketch_on(
        &mut doc,
        PlaneRef::Origin(OriginPlane::XY),
        rect_at(Vec2::new(4.0, 4.0), 2.0, 2.0),
    );
    let cut = extrude(
        &mut doc,
        sk_cut,
        Vec2::new(5.0, 5.0),
        10.0,
        BodyOp::Cut(vec![BodyRef(near), BodyRef(far)]),
    );
    {
        let state = doc.state();
        assert_eq!(state.status(cut), Some(&FeatureStatus::Ok));
        assert_relative_eq!(volume(&mut doc, near), 192.0, epsilon = 1e-9);
        assert_relative_eq!(volume(&mut doc, far), 200.0, epsilon = 1e-9);
    }

    // The same tool joined instead: the far body keeps its own 200 and gains the whole
    // 40 of the tool as a second shell, still one closed solid.
    doc.edit_feature_kind(cut, |k| {
        if let FeatureKind::Extrude { operation, .. } = k {
            *operation = BodyOp::Join(vec![BodyRef(far)]);
        }
    })
    .unwrap();
    let state = doc.state();
    assert_eq!(state.status(cut), Some(&FeatureStatus::Ok));
    let solid = &state.body(BodyRef(far)).unwrap().solid;
    assert_relative_eq!(solid.volume(), 200.0 + 40.0, epsilon = 1e-9);
    assert!(solid.is_closed());
}

/// A boolean with no bodies listed is a per-feature failure with a message, never a
/// panic: the UI cannot build one, but a file can say anything.
#[test]
fn a_boolean_with_no_targets_fails_the_feature_with_a_message() {
    let mut doc = Document::new("empty");
    let sk = sketch_on(
        &mut doc,
        PlaneRef::Origin(OriginPlane::XY),
        rect_at(Vec2::ZERO, 10.0, 10.0),
    );
    let cut = extrude(
        &mut doc,
        sk,
        Vec2::new(5.0, 5.0),
        2.0,
        BodyOp::Cut(Vec::new()),
    );
    match doc.state().status(cut) {
        Some(FeatureStatus::Failed(msg)) => {
            assert!(msg.contains("no target bodies"), "{msg}");
        }
        other => panic!("an empty target list came back {other:?}"),
    }
}
