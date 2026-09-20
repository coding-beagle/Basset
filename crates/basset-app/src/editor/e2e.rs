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
use super::sketch_mode::{SketchEditor, SketchTool};
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

/// Sets up a sketch with two loose endpoints selected, which is what Coincident applies
/// to, and leaves the Select tool active — the tool you must be in to pick geometry, and
/// the one whose hint makes the palette longest.
fn palette_with_a_selection(height: u32) -> Harness {
    let mut h = Harness::new();
    h.editor.set_window_size([800, height]);
    h.start_sketch(PlaneRef::Origin(OriginPlane::XY));
    h.line(Vec2::new(0.0, 0.0), Vec2::new(10.0, 0.0));
    h.line(Vec2::new(0.0, 5.0), Vec2::new(10.0, 6.0));
    h.sketch().set_tool(SketchTool::Select);
    let points: Vec<_> = {
        let s = h.sketch();
        let mut ids: Vec<_> = s
            .sketch
            .entities()
            .filter(|(_, d)| d.entity.is_point())
            .map(|(id, _)| id)
            .collect();
        ids.truncate(2);
        ids
    };
    assert_eq!(points.len(), 2);
    h.sketch().selected = points;
    // egui settles a wrapped layout over two frames, and `click_ui` resolves against the
    // last one, so the caller needs a frame that already knows where everything sits.
    h.frame();
    h.frame();
    h
}

/// Scrolls the sketch palette, which sits against the right edge.
fn scroll_palette(h: &mut Harness, by: f32) {
    let height = h.editor.window_px[1] as f32;
    h.frame_with(vec![egui::Event::PointerMoved(egui::pos2(
        800.0 - 60.0,
        height / 2.0,
    ))]);
    h.frame_with(vec![egui::Event::MouseWheel {
        unit: egui::MouseWheelUnit::Point,
        delta: egui::vec2(0.0, by),
        modifiers: egui::Modifiers::default(),
        phase: egui::TouchPhase::Move,
    }]);
}

/// The constraint buttons are in the sketch toolbar, where Fusion puts them.
///
/// They used to live only at the bottom of the side palette, below the tool hints, the
/// Move box and the Pattern and Parameters headers, in a panel that could not scroll —
/// so on a short window there was no way to constrain a sketch at all. The toolbar is
/// always in view, whatever the window is doing.
#[test]
fn the_toolbar_constrains_the_selection() {
    // Short enough that the palette's lower reaches are off screen.
    let mut h = palette_with_a_selection(360);
    let before = h.sketch().sketch.constraints().count();
    assert!(
        h.click_ui("Coincident"),
        "the toolbar offers Coincident for two selected points: {:?}",
        h.frame().text()
    );
    assert_eq!(
        h.sketch().sketch.constraints().count(),
        before + 1,
        "clicking it applies the constraint"
    );
}

/// The whole set is on show, so what exists is discoverable before anything is selected;
/// only what the selection supports can be clicked.
#[test]
fn the_toolbar_shows_every_constraint_and_enables_the_ones_that_apply() {
    let mut h = palette_with_a_selection(600);
    h.sketch().selected.clear();
    h.frame();
    let frame = h.frame();
    for name in SketchEditor::CONSTRAINT_NAMES {
        assert!(
            frame.has_text(name),
            "{name} is listed even with nothing selected"
        );
    }
    // Nothing applies to an empty selection, so nothing can be applied.
    let before = h.sketch().sketch.constraints().count();
    h.click_ui("Perpendicular");
    assert_eq!(h.sketch().sketch.constraints().count(), before);
}

/// Whatever does not fit in the palette can still be scrolled to.
///
/// Without a scroll area the overflow was simply unreachable, however long you looked.
#[test]
fn the_sketch_palette_scrolls_to_what_does_not_fit() {
    let mut h = palette_with_a_selection(360);
    assert!(
        !h.frame().has_text("degrees of freedom"),
        "this window is too short to show the readout outright"
    );
    scroll_palette(&mut h, -600.0);
    assert!(
        h.frame().has_text("degrees of freedom"),
        "scrolling brings it into reach: {:?}",
        h.frame().text()
    );
}

/// Geometry drawn onto the edges it meets survives the edit that broke the old file.
///
/// This is the whole cascade end to end: a divider attached by clicking the edge keeps a
/// `Coincident` onto it, so re-dimensioning moves it with the edge instead of leaving it
/// a few 1e-7 short, and the regions either side stay separate.
#[test]
fn geometry_drawn_onto_its_edges_survives_a_dimension_change() {
    let mut h = Harness::new();
    h.start_sketch(PlaneRef::Origin(OriginPlane::XY));
    // Off the grid, so the sizes below are the ones asserted rather than the nearest
    // grid multiple; attaching to an edge is the curve snap's job, not the grid's.
    h.sketch().snap_to_grid = false;
    h.rectangle(Vec2::new(0.0, 0.0), Vec2::new(40.0, 8.0));
    // Two dividers, each drawn from the bottom edge to the top edge.
    h.line(Vec2::new(10.0, 0.0), Vec2::new(10.0, 8.0));
    h.line(Vec2::new(30.0, 0.0), Vec2::new(30.0, 8.0));
    let areas = |h: &mut Harness| -> Vec<f64> {
        let mut a: Vec<f64> = h
            .sketch()
            .sketch
            .profiles(&Default::default())
            .iter()
            .map(|p| p.area())
            .collect();
        a.sort_by(f64::total_cmp);
        a
    };
    let before = areas(&mut h);
    assert_eq!(before.len(), 3, "three strips: {before:?}");

    // Dimension the left edge and double it — the edit that merged the regions before.
    let (cid, _) = h.dimension(Vec2::new(0.0, 4.0), Vec2::new(-6.0, 4.0));
    h.sketch().set_dimension(cid, 16.0);

    let after = areas(&mut h);
    assert_eq!(
        after.len(),
        3,
        "still three strips after the change: {after:?}"
    );
    for (was, now) in before.iter().zip(&after) {
        // The solver lands within its convergence tolerance, not on the exact number;
        // that residue is precisely what used to open the regions and no longer does.
        assert!(
            (now - was * 2.0).abs() < 1e-3,
            "each strip doubled with the height: {was} -> {now}"
        );
    }
}

/// Under-constrained geometry announces itself, both while drawing and afterwards.
///
/// A sketch with 26 free parameters used to look exactly like a finished one: the only
/// hint was a line of grey text at the foot of a palette that is open only while
/// sketching. So the moment it mattered — a dimension change propagating through the
/// timeline and dragging loose geometry off the edges it was drawn against — nothing said
/// a word, and the extrude silently built the wrong body.
#[test]
fn an_under_constrained_sketch_says_so_while_drawing_and_after_it_propagates() {
    let mut h = Harness::new();
    h.start_sketch(PlaneRef::Origin(OriginPlane::XY));
    h.rectangle(Vec2::new(0.0, 0.0), Vec2::new(20.0, 10.0));
    let frame = h.frame();
    assert!(
        frame.has_text("degrees of freedom") && frame.has_text("Blue geometry"),
        "the palette names the freedom and the colour it is drawn in: {:?}",
        frame.text()
    );
    // The solver agrees with what is being drawn: every line of a free rectangle is free.
    assert_eq!(
        h.sketch().under_constrained().len(),
        8,
        "4 corners + 4 lines"
    );

    // Out of the sketch and into the model, where the warning has to survive.
    h.finish_sketch(true);
    let body = h.extrude(Vec2::new(10.0, 5.0), 4.0);
    assert!((h.volume(body) - 800.0).abs() < 1e-6);
    let chip = {
        let frame = h.frame();
        frame.rect_of("\u{26a0}").unwrap_or_else(|| {
            panic!(
                "the status bar warns once a feature builds from the sketch: {:?}",
                frame.text()
            )
        })
    };
    // It names the feature, and hovering it says what is wrong.
    assert!(h.frame().has_text("\u{26a0} Sketch1"));
    h.frame_with(vec![egui::Event::PointerMoved(chip.center())]);
    // Tooltips wait out a hover delay before they appear, which costs a few frames.
    for _ in 0..30 {
        h.frame();
    }
    let frame = h.frame();
    assert!(
        frame.has_text("degrees of freedom"),
        "the hover text gives the reason: {:?}",
        frame.text()
    );
}

/// Every constraint is listed by name, as the fallback for the badges in the viewport:
/// a badge can end up under other geometry, and the sketch whose badges are hardest to
/// read is exactly the one that has gone wrong.
#[test]
fn the_palette_lists_the_constraints_and_lights_up_what_they_hold() {
    let mut h = Harness::new();
    h.start_sketch(PlaneRef::Origin(OriginPlane::XY));
    h.rectangle(Vec2::new(0.0, 0.0), Vec2::new(20.0, 10.0));
    let (_, _) = h.dimension(Vec2::new(10.0, 0.0), Vec2::new(10.0, -6.0));
    let constraints = h.sketch().sketch.constraints().count();
    assert!(constraints > 0);
    assert!(
        h.frame().has_text(&format!("Constraints ({constraints})")),
        "the list is headed by the count: {:?}",
        h.frame().text()
    );
    assert!(
        h.click_ui(&format!("Constraints ({constraints})")),
        "expand"
    );
    // The header opens with an animation, and egui only paints the rows it has room for
    // as it grows, so let it settle before looking for one.
    for _ in 0..30 {
        h.frame();
    }
    // The rows sit below the fold on an ordinary window, so scroll to them as a user would.
    scroll_palette(&mut h, -240.0);
    let row = {
        let frame = h.frame();
        frame
            .rect_of("Distance")
            .unwrap_or_else(|| panic!("a dimension row names its value: {:?}", frame.text()))
    };
    // Hovering a row lights up the geometry that constraint holds, which is how the list
    // points back at the drawing.
    h.frame_with(vec![egui::Event::PointerMoved(row.center())]);
    let highlighted = h.sketch().highlighted.clone();
    assert_eq!(highlighted.len(), 2, "the two points the distance spans");
}
