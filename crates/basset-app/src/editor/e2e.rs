//! End-to-end tests driven through [`Harness`]: window events in, panels and scene out.
//!
//! These cover the seams that the unit tests in `tests.rs` step over — the winit event
//! path, the egui panels, the scene the renderer is handed and the file round trip —
//! because those are exactly the parts that used to need a human with a mouse.

use basset_core::{OriginPlane, PlaneRef};
use basset_math::{Vec2, Vec3};
use winit::event::{ElementState, MouseButton};
use winit::keyboard::NamedKey;

use super::harness::{Harness, TempDir};
use super::selection::SelectMode;
use super::tools::ToolKind;

#[test]
fn a_frame_draws_the_panels_and_the_scene() {
    let mut h = Harness::new();
    let frame = h.frame();
    assert!(frame.has_text("Extrude"), "{:?}", frame.text());
    assert!(frame.has_text("Ready"), "the status bar shows the status");
    assert!(
        frame.has_text("Origin"),
        "the browser lists the origin planes"
    );
    // An empty document with the origin hidden has nothing in the 3D scene; turning the
    // origin on gives the renderer the planes and axes to draw.
    assert_eq!(frame.line_batches, 0);
    h.editor.show_origin = true;
    assert!(h.frame().line_batches > 0);
}

#[test]
fn a_toolbar_button_starts_its_tool() {
    let mut h = Harness::new();
    assert!(h.click_ui("Extrude"), "the toolbar has an Extrude button");
    assert_eq!(
        h.editor.tool.as_ref().map(|t| t.kind),
        Some(ToolKind::Extrude)
    );
    // Escape is the way out of a tool, and it reaches the editor as a keystroke does.
    h.key(NamedKey::Escape);
    assert!(h.editor.tool.is_none());
}

#[test]
fn a_sketch_and_an_extrude_survive_a_round_trip_through_a_file() {
    let dir = TempDir::new("round-trip");
    let path = dir.join("block.bass");
    let mut h = Harness::new();
    h.start_sketch(PlaneRef::Origin(OriginPlane::XY));
    h.rectangle(Vec2::ZERO, Vec2::new(20.0, 10.0));
    h.finish_sketch(true);
    let body = h.extrude(Vec2::new(10.0, 5.0), 3.0);
    assert!((h.volume(body) - 600.0).abs() < 1e-6);

    let mut reopened = h.round_trip(&path);
    assert_eq!(reopened.editor.doc.timeline().len(), 2);
    let bodies = reopened.bodies();
    assert_eq!(bodies.len(), 1);
    assert!(
        (reopened.volume(bodies[0]) - 600.0).abs() < 1e-6,
        "the model regenerates from the file to the same solid"
    );
    // The reopened document is clean: nothing to undo and the title follows the file.
    assert!(!reopened.editor.doc.can_undo());
    assert!(
        reopened
            .editor
            .take_title_change()
            .is_some_and(|t| t.contains("block.bass"))
    );
}

#[test]
fn clicking_a_body_in_the_viewport_selects_the_face_under_the_pointer() {
    let mut h = Harness::new();
    let body = h.block();
    h.editor.set_select_mode(SelectMode::Faces);
    // The top of the block, picked by where it lands on screen rather than by name.
    h.click_world(Vec3::new(5.0, 5.0, 2.0));
    let faces = &h.editor.selection.faces;
    assert_eq!(faces.len(), 1, "{faces:?}");
    assert_eq!(faces[0], super::harness::top_face(body));

    // Clicking off the model clears it again, and the scene stops highlighting.
    h.click_world(Vec3::new(-80.0, -80.0, 0.0));
    assert!(h.editor.selection.faces.is_empty());
}

#[test]
fn the_pointer_navigates_the_camera() {
    let mut h = Harness::new();
    let before = h.editor.camera.eye();
    h.move_px([400.0, 300.0]);
    h.button(MouseButton::Right, ElementState::Pressed);
    h.move_px([500.0, 300.0]);
    h.button(MouseButton::Right, ElementState::Released);
    let orbited = h.editor.camera.eye();
    assert!(
        (orbited - before).length() > 1e-6,
        "right-dragging orbits the camera"
    );
    let target = h.editor.camera.target;
    let before_zoom = (orbited - target).length();
    h.scroll(1.0);
    assert!(
        (h.editor.camera.eye() - target).length() < before_zoom,
        "the wheel zooms in"
    );
}

#[test]
fn keyboard_shortcuts_reach_the_editor() {
    let mut h = Harness::new();
    h.type_key("2");
    assert_eq!(h.editor.select_mode, SelectMode::Faces);

    h.start_sketch(PlaneRef::Origin(OriginPlane::XY));
    h.rectangle(Vec2::ZERO, Vec2::new(10.0, 10.0));
    h.finish_sketch(true);
    assert_eq!(h.editor.doc.timeline().len(), 1);
    h.ctrl_key("z");
    assert!(
        h.editor.doc.timeline().is_empty(),
        "Ctrl+Z undoes the whole sketch session"
    );
    h.ctrl_key("y");
    assert_eq!(h.editor.doc.timeline().len(), 1, "Ctrl+Y puts it back");
}

#[test]
fn the_timeline_and_browser_show_what_the_document_holds() {
    let mut h = Harness::new();
    h.block();
    let frame = h.frame();
    assert!(frame.has_text("Sketch"), "{:?}", frame.text());
    assert!(frame.has_text("Body"), "{:?}", frame.text());
}

#[test]
fn a_sketch_drawn_through_the_pointer_lands_where_it_was_clicked() {
    let mut h = Harness::new();
    h.start_sketch(PlaneRef::Origin(OriginPlane::XY));
    h.sketch().snap_to_grid = false;
    h.sketch()
        .set_tool(super::sketch_mode::SketchTool::Rectangle);
    // Pixels, not rays: this is the whole camera-and-picking path a real click takes.
    h.click_world(Vec3::ZERO);
    h.click_world(Vec3::new(10.0, 10.0, 0.0));
    h.finish_sketch(true);
    let sketch = h.last_feature();
    let state = h.editor.doc.state();
    let solved = state.sketches.get(&sketch).expect("the sketch evaluated");
    assert_eq!(solved.profiles.len(), 1);
    assert!(
        (solved.profiles[0].area() - 100.0).abs() < 0.5,
        "area {} came from the clicked pixels",
        solved.profiles[0].area()
    );

    // Sketch selection draws the points it offers to pick, and clicking inside the
    // region selects it, which fills it.
    h.editor.set_select_mode(SelectMode::Sketch);
    assert!(h.frame().point_batches > 0, "the sketch's points are drawn");
    h.click_world(Vec3::new(5.0, 5.0, 0.0));
    assert!(h.frame().tri_batches > 0, "the region is filled");
}

#[test]
fn hovering_highlights_and_leaving_the_window_clears_it() {
    let mut h = Harness::new();
    h.block();
    h.editor.set_select_mode(SelectMode::Faces);
    h.move_world(Vec3::new(5.0, 5.0, 2.0));
    assert!(
        h.editor.hover.is_some(),
        "the face under the pointer hovers"
    );
    h.pointer_left_window();
    assert!(h.editor.hover.is_none());
}

#[test]
fn a_box_drag_selects_the_curves_it_encloses() {
    let mut h = Harness::new();
    h.start_sketch(PlaneRef::Origin(OriginPlane::XY));
    h.sketch().snap_to_grid = false;
    h.line(Vec2::ZERO, Vec2::new(10.0, 0.0));
    h.sketch().select_tool();
    // Rightwards encloses: a box around the line takes it.
    let from = h.screen_of(Vec3::new(-5.0, -5.0, 0.0));
    let to = h.screen_of(Vec3::new(15.0, 5.0, 0.0));
    h.drag_px(from, to);
    assert!(
        !h.sketch().selected.is_empty(),
        "the rubber band took the line"
    );
}

#[test]
fn a_cancelled_tool_leaves_the_document_as_it_was() {
    let mut h = Harness::new();
    h.start_sketch(PlaneRef::Origin(OriginPlane::XY));
    h.rectangle(Vec2::ZERO, Vec2::new(10.0, 10.0));
    h.finish_sketch(true);
    let sketch = h.last_feature();
    h.start_tool(ToolKind::Extrude);
    h.select_region(sketch, Vec2::new(5.0, 5.0));
    h.sync_tool();
    assert_eq!(h.bodies().len(), 1, "the preview is a real feature");
    h.cancel_tool();
    assert!(h.bodies().is_empty());
    assert_eq!(h.editor.doc.timeline().len(), 1, "only the sketch is left");
}

#[test]
fn a_dimension_typed_into_a_reopened_sketch_re_drives_the_body() {
    let mut h = Harness::new();
    h.start_sketch(PlaneRef::Origin(OriginPlane::XY));
    h.rectangle(Vec2::ZERO, Vec2::new(10.0, 10.0));
    h.finish_sketch(true);
    let sketch = h.last_feature();
    let body = h.extrude(Vec2::new(5.0, 5.0), 2.0);
    assert!((h.volume(body) - 200.0).abs() < 1e-6);

    h.edit_sketch(sketch);
    h.sketch().snap_to_grid = false;
    // The two horizontal edges, dimensioned as the distance between them.
    let (id, _) = h.dimension(Vec2::new(5.0, 0.0), Vec2::new(5.0, 10.0));
    h.sketch().set_dimension(id, 20.0);
    h.finish_sketch(true);
    assert!(
        (h.volume(body) - 400.0).abs() < 1e-6,
        "the extrude replayed over the re-driven sketch: {}",
        h.volume(body)
    );
}
