//! Document-wide parameters end to end: one named number drives sketches and features
//! alike, a rename follows every reference, and losing a parameter warns instead of
//! collapsing the model.

use std::f64::consts::PI;

use approx::assert_relative_eq;
use basset_core::{
    AxisRef, BodyOp, BodyRef, ComponentId, Document, Extent, FeatureId, FeatureKind, FeatureStatus,
    NumericField, OriginAxis, OriginPlane, PlaneRef, ProfileRef, RegionRef, Sketch,
};
use basset_math::Vec2;
use basset_sketch::{Constraint, ConstraintId, SketchError, shapes};

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

fn extrude(doc: &mut Document, sketch: FeatureId, sample: Vec2, distance: f64) -> FeatureId {
    doc.add_feature(FeatureKind::Extrude {
        regions: vec![RegionRef::Profile(ProfileRef { sketch, sample })],
        extent: Extent::OneSide(distance),
        operation: BodyOp::NewBody,
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

/// A 10×5 rectangle extruded 4, with the sketch's width dimension available to bind.
fn block() -> (Document, FeatureId, FeatureId, ConstraintId) {
    let mut doc = Document::new("test");
    let (sketch, width) = rect_sketch(10.0, 5.0);
    let sk = sketch_on(&mut doc, PlaneRef::Origin(OriginPlane::XY), sketch);
    let ex = extrude(&mut doc, sk, Vec2::new(1.0, 1.0), 4.0);
    (doc, sk, ex, width)
}

fn sketch_of(doc: &Document, id: FeatureId) -> &Sketch {
    match &doc.timeline().get(id).expect("feature exists").kind {
        FeatureKind::Sketch { sketch, .. } => sketch,
        other => panic!("{} is not a sketch", other.default_name()),
    }
}

#[test]
fn a_document_parameter_drives_an_extrude_and_editing_it_moves_the_geometry() {
    let (mut doc, _, ex, _) = block();
    doc.set_parameter("thickness", "4").unwrap();
    assert_eq!(
        doc.set_feature_expr(ex, NumericField::Distance, "thickness * 2")
            .unwrap(),
        8.0
    );
    assert_relative_eq!(volume(&mut doc, ex), 10.0 * 5.0 * 8.0, epsilon = 1e-9);

    doc.set_parameter("thickness", "1.5").unwrap();
    assert_relative_eq!(volume(&mut doc, ex), 10.0 * 5.0 * 3.0, epsilon = 1e-9);
    // The expression is what is stored; the number merely follows it.
    assert_eq!(
        doc.feature_expr(ex, NumericField::Distance),
        Some("thickness * 2")
    );
}

#[test]
fn a_document_parameter_drives_a_sketch_dimension() {
    let (mut doc, sk, ex, width) = block();
    doc.set_parameter("plate_width", "10").unwrap();
    doc.edit_sketch(sk, |sketch, outer| {
        sketch.bind_dimension_with(width, "plate_width", outer)
    })
    .unwrap()
    .unwrap();
    assert_relative_eq!(volume(&mut doc, ex), 10.0 * 5.0 * 4.0, epsilon = 1e-9);

    doc.set_parameter("plate_width", "20").unwrap();
    assert_relative_eq!(volume(&mut doc, ex), 20.0 * 5.0 * 4.0, epsilon = 1e-9);
}

#[test]
fn a_sketch_parameter_shadows_the_document_one_of_the_same_name() {
    let (mut doc, sk, ex, width) = block();
    doc.set_parameter("w", "20").unwrap();
    doc.edit_sketch(sk, |sketch, outer| {
        sketch.set_parameter_with("w", "7", outer).unwrap();
        sketch.bind_dimension_with(width, "w", outer).unwrap()
    })
    .unwrap();
    assert_relative_eq!(volume(&mut doc, ex), 7.0 * 5.0 * 4.0, epsilon = 1e-9);

    // Changing the document's `w` moves nothing: the sketch defines its own.
    doc.set_parameter("w", "30").unwrap();
    assert_relative_eq!(volume(&mut doc, ex), 7.0 * 5.0 * 4.0, epsilon = 1e-9);
}

#[test]
fn renaming_a_parameter_follows_every_reference_but_not_into_a_sketch_that_shadows_it() {
    let (mut doc, sk, ex, width) = block();
    doc.set_parameter("thickness", "4").unwrap();
    doc.set_parameter("double", "thickness * 2").unwrap();
    doc.set_feature_expr(ex, NumericField::Distance, "thickness + 1")
        .unwrap();
    doc.edit_sketch(sk, |sketch, outer| {
        sketch.bind_dimension_with(width, "thickness * 2", outer)
    })
    .unwrap()
    .unwrap();

    // A second sketch that defines `thickness` itself means its own, whatever the
    // document calls its parameter.
    let (other, other_width) = rect_sketch(10.0, 5.0);
    let shadowing = sketch_on(&mut doc, PlaneRef::Origin(OriginPlane::XZ), other);
    doc.edit_sketch(shadowing, |sketch, outer| {
        sketch.set_parameter_with("thickness", "9", outer).unwrap();
        sketch
            .bind_dimension_with(other_width, "thickness", outer)
            .unwrap()
    })
    .unwrap();

    doc.rename_parameter("thickness", "plate").unwrap();

    assert_eq!(doc.parameters().get("plate").unwrap().expr, "4");
    assert_eq!(doc.parameters().get("double").unwrap().expr, "plate * 2");
    assert_eq!(
        doc.feature_expr(ex, NumericField::Distance),
        Some("plate + 1")
    );
    assert_eq!(
        sketch_of(&doc, sk).dimension_expr(width),
        Some("plate * 2"),
        "a sketch that reads the document's parameter follows the rename"
    );
    assert_eq!(
        sketch_of(&doc, shadowing).dimension_expr(other_width),
        Some("thickness"),
        "a sketch with its own parameter of that name is left alone"
    );
    // And everything still evaluates, which is the point of rewriting at all.
    assert_relative_eq!(volume(&mut doc, ex), 8.0 * 5.0 * 5.0, epsilon = 1e-9);
}

#[test]
fn deleting_a_parameter_warns_and_leaves_the_feature_holding_its_last_value() {
    let (mut doc, _, ex, _) = block();
    doc.set_parameter("thickness", "6").unwrap();
    doc.set_feature_expr(ex, NumericField::Distance, "thickness")
        .unwrap();
    let before = volume(&mut doc, ex);
    assert_relative_eq!(before, 10.0 * 5.0 * 6.0, epsilon = 1e-9);

    assert!(doc.remove_parameter("thickness"));
    assert_relative_eq!(volume(&mut doc, ex), before, epsilon = 1e-9);
    let status = doc.state().status(ex).cloned();
    match status {
        Some(FeatureStatus::Warned(message)) => {
            assert!(message.contains("Distance"), "{message}");
            assert!(message.contains("thickness"), "{message}");
        }
        other => panic!("expected a warning, got {other:?}"),
    }

    // Putting it back takes the warning away again.
    doc.set_parameter("thickness", "2").unwrap();
    assert_relative_eq!(volume(&mut doc, ex), 10.0 * 5.0 * 2.0, epsilon = 1e-9);
    assert_eq!(doc.state().status(ex), Some(&FeatureStatus::Ok));
}

#[test]
fn a_sketch_whose_binding_stopped_evaluating_is_warned_about() {
    let (mut doc, sk, ex, width) = block();
    doc.set_parameter("w", "12").unwrap();
    doc.edit_sketch(sk, |sketch, outer| {
        sketch.bind_dimension_with(width, "w", outer)
    })
    .unwrap()
    .unwrap();
    assert_relative_eq!(volume(&mut doc, ex), 12.0 * 5.0 * 4.0, epsilon = 1e-9);

    doc.remove_parameter("w");
    // The dimension keeps its value, so the model is unchanged and still solves.
    assert_relative_eq!(volume(&mut doc, ex), 12.0 * 5.0 * 4.0, epsilon = 1e-9);
    match doc.state().status(sk) {
        Some(FeatureStatus::Warned(message)) => {
            assert!(
                message.contains("1 dimension stopped being driven"),
                "{message}"
            )
        }
        other => panic!("expected a warning, got {other:?}"),
    }
}

#[test]
fn a_parameter_that_refers_to_itself_is_refused_and_changes_nothing() {
    let (mut doc, _, _, _) = block();
    doc.set_parameter("a", "3").unwrap();
    doc.set_parameter("b", "a * 2").unwrap();
    let before = doc.parameters().clone();

    let err = doc.set_parameter("a", "b + 1").unwrap_err();
    assert!(
        matches!(
            err,
            basset_core::DocumentError::Parameter(SketchError::CircularParameter(_))
        ),
        "{err}"
    );
    assert_eq!(doc.parameters(), &before);
    assert_eq!(doc.parameters().value("b").unwrap(), 6.0);
}

#[test]
fn an_angle_expression_is_written_in_degrees() {
    let mut expressed = Document::new("expressed");
    let (sketch, _) = rect_sketch(10.0, 5.0);
    let sk = sketch_on(&mut expressed, PlaneRef::Origin(OriginPlane::XZ), sketch);
    let rev = expressed.add_feature(FeatureKind::Revolve {
        regions: vec![RegionRef::Profile(ProfileRef {
            sketch: sk,
            sample: Vec2::new(1.0, 1.0),
        })],
        axis: AxisRef::Origin(OriginAxis::Z),
        angle: 0.1,
        operation: BodyOp::NewBody,
        component: ComponentId::ROOT,
    });
    expressed.set_parameter("turn", "90").unwrap();
    expressed
        .set_feature_expr(rev, NumericField::Angle, "turn")
        .unwrap();

    // The stored number is radians, whatever the user typed.
    let FeatureKind::Revolve { angle, .. } = &expressed.timeline().get(rev).unwrap().kind else {
        unreachable!()
    };
    assert_relative_eq!(*angle, PI / 2.0, epsilon = 1e-12);

    // And the geometry is the geometry of a quarter turn, not of 90 radians reduced.
    let mut literal = Document::new("literal");
    let (sketch, _) = rect_sketch(10.0, 5.0);
    let sk2 = sketch_on(&mut literal, PlaneRef::Origin(OriginPlane::XZ), sketch);
    let rev2 = literal.add_feature(FeatureKind::Revolve {
        regions: vec![RegionRef::Profile(ProfileRef {
            sketch: sk2,
            sample: Vec2::new(1.0, 1.0),
        })],
        axis: AxisRef::Origin(OriginAxis::Z),
        angle: PI / 2.0,
        operation: BodyOp::NewBody,
        component: ComponentId::ROOT,
    });
    assert_relative_eq!(
        volume(&mut expressed, rev),
        volume(&mut literal, rev2),
        epsilon = 1e-9
    );
}

#[test]
fn parameters_survive_a_round_trip_through_bass() {
    let (mut doc, sk, ex, width) = block();
    doc.set_parameter("thickness", "3").unwrap();
    doc.set_parameter("w", "thickness * 4").unwrap();
    doc.set_feature_expr(ex, NumericField::Distance, "thickness")
        .unwrap();
    doc.edit_sketch(sk, |sketch, outer| {
        sketch.bind_dimension_with(width, "w", outer)
    })
    .unwrap()
    .unwrap();
    let expected = volume(&mut doc, ex);

    let mut bytes = Vec::new();
    basset_core::file::write(&mut bytes, &doc).unwrap();
    let mut loaded = basset_core::file::read(bytes.as_slice()).unwrap();
    assert_eq!(loaded.parameters(), doc.parameters());
    assert_eq!(
        loaded.feature_expr(ex, NumericField::Distance),
        Some("thickness")
    );
    assert_relative_eq!(volume(&mut loaded, ex), expected, epsilon = 1e-9);

    // Changing the parameter in the loaded document still drives everything.
    loaded.set_parameter("thickness", "6").unwrap();
    assert_relative_eq!(volume(&mut loaded, ex), 24.0 * 5.0 * 6.0, epsilon = 1e-9);
}

#[test]
fn a_version_2_document_loads_with_no_parameters_and_regenerates() {
    let (mut doc, _, ex, _) = block();
    let expected = volume(&mut doc, ex);
    let mut bytes = Vec::new();
    basset_core::file::write(&mut bytes, &doc).unwrap();
    let mut file: serde_json::Value = serde_json::from_slice(&bytes).unwrap();
    // Rewrite it the way version 2 wrote it: no table, and no driven values.
    file["format_version"] = serde_json::json!(2);
    file["document"]
        .as_object_mut()
        .unwrap()
        .remove("parameters");
    let old = serde_json::to_vec(&file).unwrap();

    let mut loaded = basset_core::file::read(old.as_slice()).unwrap();
    assert!(loaded.parameters().is_empty());
    assert_relative_eq!(volume(&mut loaded, ex), expected, epsilon = 1e-9);
}

#[test]
fn undo_restores_a_deleted_parameter_and_the_geometry_it_drove() {
    let (mut doc, _, ex, _) = block();
    doc.set_parameter("thickness", "4").unwrap();
    doc.set_feature_expr(ex, NumericField::Distance, "thickness")
        .unwrap();
    let driven = volume(&mut doc, ex);

    assert!(doc.remove_parameter("thickness"));
    assert!(doc.parameters().get("thickness").is_none());
    assert!(doc.undo());
    assert_eq!(doc.parameters().get("thickness").unwrap().expr, "4");
    assert_relative_eq!(volume(&mut doc, ex), driven, epsilon = 1e-9);

    // And redo takes it away again.
    assert!(doc.redo());
    assert!(doc.parameters().get("thickness").is_none());
}

#[test]
fn a_parameter_edit_inside_a_transaction_undoes_as_one_step() {
    let (mut doc, _, ex, _) = block();
    doc.set_parameter("thickness", "4").unwrap();
    doc.set_feature_expr(ex, NumericField::Distance, "thickness")
        .unwrap();

    doc.begin_transaction();
    for step in 5..10 {
        doc.set_parameter("thickness", &step.to_string()).unwrap();
    }
    doc.commit_transaction();
    assert_relative_eq!(volume(&mut doc, ex), 10.0 * 5.0 * 9.0, epsilon = 1e-9);

    assert!(doc.undo());
    assert_eq!(doc.parameters().get("thickness").unwrap().expr, "4");
    assert_relative_eq!(volume(&mut doc, ex), 10.0 * 5.0 * 4.0, epsilon = 1e-9);
}

#[test]
fn releasing_a_driven_feature_value_keeps_the_number_the_expression_gave() {
    let (mut doc, _, ex, _) = block();
    doc.set_parameter("thickness", "4").unwrap();
    doc.set_feature_expr(ex, NumericField::Distance, "thickness")
        .unwrap();
    doc.set_parameter("thickness", "7").unwrap();

    assert!(doc.clear_feature_expr(ex, NumericField::Distance).unwrap());
    assert_eq!(doc.feature_expr(ex, NumericField::Distance), None);
    assert_relative_eq!(volume(&mut doc, ex), 10.0 * 5.0 * 7.0, epsilon = 1e-9);
    // Nothing drives it any more, so the parameter no longer reaches it.
    doc.set_parameter("thickness", "1").unwrap();
    assert_relative_eq!(volume(&mut doc, ex), 10.0 * 5.0 * 7.0, epsilon = 1e-9);
}

#[test]
fn a_feature_value_cannot_be_driven_by_an_expression_that_does_not_evaluate() {
    let (mut doc, _, ex, _) = block();
    assert!(
        doc.set_feature_expr(ex, NumericField::Distance, "missing")
            .is_err()
    );
    assert_eq!(doc.feature_expr(ex, NumericField::Distance), None);
    assert_relative_eq!(volume(&mut doc, ex), 10.0 * 5.0 * 4.0, epsilon = 1e-9);

    // Nor by a field this feature does not have.
    doc.set_parameter("a", "1").unwrap();
    assert!(matches!(
        doc.set_feature_expr(ex, NumericField::Radius, "a"),
        Err(basset_core::DocumentError::NoSuchField(..))
    ));
}

#[test]
fn renaming_onto_a_name_a_sketch_defines_itself_is_refused_rather_than_capturing_it() {
    let (mut doc, sk, ex, width) = block();
    doc.set_parameter("a", "10").unwrap();
    doc.edit_sketch(sk, |sketch, outer| {
        sketch.set_parameter_with("b", "2", outer).unwrap();
        sketch.bind_dimension_with(width, "a * 3", outer).unwrap()
    })
    .unwrap();
    let before = volume(&mut doc, ex);
    assert_relative_eq!(before, 30.0 * 5.0 * 4.0, epsilon = 1e-9);

    let err = doc.rename_parameter("a", "b").unwrap_err();
    assert!(
        matches!(err, basset_core::DocumentError::ParameterCaptured(id, ref name)
            if id == sk && name == "b"),
        "{err}"
    );
    // Nothing moved: not the table, not the sketch, not the geometry.
    assert_eq!(doc.parameters().get("a").unwrap().expr, "10");
    assert_eq!(sketch_of(&doc, sk).dimension_expr(width), Some("a * 3"));
    assert_relative_eq!(volume(&mut doc, ex), before, epsilon = 1e-9);
}

#[test]
fn renaming_onto_a_name_a_sketch_row_would_then_refer_to_itself_is_refused() {
    let (mut doc, sk, _, _) = block();
    doc.set_parameter("a", "10").unwrap();
    doc.edit_sketch(sk, |sketch, outer| {
        sketch.set_parameter_with("b", "a + 1", outer)
    })
    .unwrap()
    .unwrap();

    assert!(matches!(
        doc.rename_parameter("a", "b"),
        Err(basset_core::DocumentError::ParameterCaptured(..))
    ));
    assert_eq!(sketch_of(&doc, sk).parameter("b").unwrap().expr, "a + 1");
}

#[test]
fn renaming_reports_the_sketches_it_left_alone_because_they_shadow_the_name() {
    let (mut doc, sk, _, width) = block();
    doc.set_parameter("thickness", "4").unwrap();
    doc.edit_sketch(sk, |sketch, outer| {
        sketch.set_parameter_with("thickness", "9", outer).unwrap();
        sketch
            .bind_dimension_with(width, "thickness", outer)
            .unwrap()
    })
    .unwrap();
    assert_eq!(doc.sketches_shadowing("thickness"), vec![sk]);

    let skipped = doc.rename_parameter("thickness", "plate").unwrap();
    assert_eq!(skipped, vec![sk]);
    assert_eq!(sketch_of(&doc, sk).dimension_expr(width), Some("thickness"));
}

#[test]
fn the_number_stored_for_a_driven_feature_keeps_up_with_the_parameter() {
    let (mut doc, _, ex, _) = block();
    doc.set_parameter("t", "10").unwrap();
    doc.set_feature_expr(ex, NumericField::Distance, "t")
        .unwrap();

    let stored = |doc: &Document| {
        doc.timeline()
            .get(ex)
            .unwrap()
            .kind
            .numeric_field(NumericField::Distance)
            .unwrap()
    };
    doc.set_parameter("t", "25").unwrap();
    assert_relative_eq!(stored(&doc), 25.0, epsilon = 1e-12);
    assert_relative_eq!(volume(&mut doc, ex), 10.0 * 5.0 * 25.0, epsilon = 1e-9);

    // Undo swaps the table wholesale; the stored number has to follow it back.
    assert!(doc.undo());
    assert_relative_eq!(stored(&doc), 10.0, epsilon = 1e-12);
    assert_relative_eq!(volume(&mut doc, ex), 10.0 * 5.0 * 10.0, epsilon = 1e-9);
    assert!(doc.redo());
    assert_relative_eq!(stored(&doc), 25.0, epsilon = 1e-12);

    // A rename keeps them in step too, and so does deleting the parameter — which leaves
    // the number where it was, exactly as the feature's geometry does.
    doc.rename_parameter("t", "thickness").unwrap();
    assert_relative_eq!(stored(&doc), 25.0, epsilon = 1e-12);
    doc.remove_parameter("thickness");
    assert_relative_eq!(stored(&doc), 25.0, epsilon = 1e-12);
    assert_relative_eq!(volume(&mut doc, ex), 10.0 * 5.0 * 25.0, epsilon = 1e-9);
}

#[test]
fn a_refused_edit_does_not_consume_the_redo_stack() {
    let (mut doc, _, ex, _) = block();
    doc.rename_feature(ex, "Body").unwrap();
    assert!(doc.undo());
    assert!(doc.can_redo(), "the rename is there to be redone");

    let missing = FeatureId(9999);
    assert!(doc.rename_feature(missing, "nope").is_err());
    assert!(doc.remove_feature(missing).is_err());
    assert!(doc.edit_feature(missing, |f| f.suppressed = true).is_err());
    assert!(doc.reorder_feature(missing, 0).is_err());
    assert!(doc.set_parameter("2bad", "1").is_err());
    assert!(
        doc.can_redo(),
        "a refused edit changes nothing, so it must not clear the redo stack"
    );

    // And a reorder refused because it would break a dependency is no different.
    let (mut doc, sk, _, _) = block();
    doc.rename_feature(sk, "Base").unwrap();
    assert!(doc.undo());
    assert!(doc.reorder_feature(sk, 1).is_err());
    assert!(doc.can_redo());
    assert!(doc.redo());
    assert_eq!(doc.timeline().get(sk).unwrap().name, "Base");
}

#[test]
fn a_cancelled_transaction_leaves_the_redo_stack_as_it_found_it() {
    let (mut doc, _, ex, _) = block();
    doc.rename_feature(ex, "Body").unwrap();
    assert!(doc.undo());
    assert!(doc.can_redo());

    doc.begin_transaction();
    doc.set_parameter("t", "3").unwrap();
    doc.rollback_transaction();

    assert!(doc.parameters().is_empty(), "the transaction was discarded");
    assert!(
        doc.can_redo(),
        "cancelling changed nothing, so redo survives"
    );
    assert!(doc.redo());
    assert_eq!(doc.timeline().get(ex).unwrap().name, "Body");
}
