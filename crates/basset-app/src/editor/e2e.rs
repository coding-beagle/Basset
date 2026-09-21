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
use super::sketch_mode::{ConstraintKind, SketchTool};
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
    for kind in ConstraintKind::ALL {
        assert!(
            frame.has_text(kind.name()),
            "{} is listed even with nothing selected",
            kind.name()
        );
    }
    // Nothing is selected, so the click arms the tool and waits rather than guessing
    // what it was meant to act on.
    let before = h.sketch().sketch.constraints().count();
    h.click_ui("Perpendicular");
    assert_eq!(h.sketch().sketch.constraints().count(), before);
    assert_eq!(
        h.sketch().armed_constraint(),
        Some(ConstraintKind::Perpendicular),
        "the button arms the tool"
    );
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

/// A redundant constraint gets a section of its own in the palette, counted in a line,
/// with each one offered for deletion and lighting up what it holds when hovered — the
/// same treatment a conflicting one gets, one step less severe.
#[test]
fn the_palette_lists_redundant_constraints_and_offers_to_delete_them() {
    let mut h = Harness::new();
    h.start_sketch(PlaneRef::Origin(OriginPlane::XY));
    h.rectangle(Vec2::new(0.0, 0.0), Vec2::new(20.0, 10.0));
    assert!(
        !h.frame().has_text("redundant constraint"),
        "a rectangle's own constraints are all doing something"
    );
    let bottom = {
        let s = h.sketch();
        s.sketch
            .entities()
            .find(|(id, d)| {
                matches!(d.entity, basset_sketch::Entity::Line { .. })
                    && s.sketch
                        .entity_bounds(*id)
                        .is_some_and(|(min, max)| min.y == 0.0 && max.y == 0.0)
            })
            .map(|(id, _)| id)
            .expect("bottom edge")
    };
    h.sketch()
        .add_constraint(basset_sketch::Constraint::Horizontal(bottom))
        .expect("consistent, so it is taken");
    let before = h.sketch().sketch.constraints().count();

    // The palette is taller than an 800x600 window, so the section is scrolled to
    // before it is read: everything below the degrees-of-freedom line is off the
    // bottom otherwise, which is what the scroll area is there for.
    scroll_palette(&mut h, -400.0);
    // The count is a line of its own, and the row sits under it. The toolbar has a
    // Horizontal button too, so the row is the "Horizontal" below the count.
    let (count, row) = {
        let frame = h.frame();
        let count = frame
            .rect_of("1 redundant constraint")
            .unwrap_or_else(|| panic!("the palette counts them: {:?}", frame.text()));
        let row = frame
            .texts
            .iter()
            .find(|(r, t)| t.trim() == "Horizontal" && r.min.y > count.min.y)
            .map(|(r, _)| *r)
            .unwrap_or_else(|| panic!("the row names the constraint: {:?}", frame.text()));
        (count, row)
    };
    assert!(row.min.y > count.min.y, "the row is in the section");
    h.frame_with(vec![egui::Event::PointerMoved(row.center())]);
    assert_eq!(
        h.sketch().highlighted,
        vec![bottom],
        "hovering the row lights up the line it holds"
    );

    // The delete button sits at the start of the row, so click just left of the name.
    let cross = {
        let frame = h.frame();
        frame
            .texts
            .iter()
            .find(|(r, t)| t.trim() == "\u{2715}" && (r.center().y - row.center().y).abs() < 4.0)
            .map(|(r, _)| *r)
            .unwrap_or_else(|| panic!("a delete on the row: {:?}", frame.text()))
    };
    h.click_at_ui(cross.center());
    h.frame();
    assert_eq!(
        h.sketch().sketch.constraints().count(),
        before - 1,
        "the delete removed the constraint"
    );
    assert!(
        h.sketch().redundant().is_empty(),
        "and nothing is redundant any more"
    );
    assert!(
        !h.frame().has_text("redundant constraint"),
        "so the section is gone: {:?}",
        h.frame().text()
    );
}

/// The controls a running operation is driven by have to be in front of the user.
///
/// Move and Pattern used to sit at the bottom of the sketch palette, below the tool
/// hints, the degrees-of-freedom readout and the whole constraint list — past the fold
/// of a scrolling panel on an ordinary window. A pattern whose "Select origin" cannot be
/// reached is a pattern whose origin cannot be set, which is exactly how it failed.
#[test]
fn a_running_sketch_operation_is_reachable_on_a_small_window() {
    let mut h = Harness::new();
    h.editor.set_window_size([800, 600]);
    h.start_sketch(PlaneRef::Origin(OriginPlane::XY));
    h.rectangle(Vec2::new(20.0, 0.0), Vec2::new(25.0, 5.0));
    {
        let s = h.sketch();
        s.set_tool(SketchTool::Select);
        s.selected = s
            .sketch
            .entities()
            .filter(|(_, d)| d.entity.is_curve())
            .map(|(id, _)| id)
            .collect();
        s.pattern.circular = true;
        s.pattern.count = 4;
    }
    h.frame();
    h.frame();
    assert!(h.click_ui("Pattern"), "the toolbar offers Pattern");
    assert!(h.sketch().pattern_in_progress());
    h.frame();
    h.frame();

    // Without this the origin cannot be placed at all: the viewport only takes the click
    // while the mode is on, and the mode is only reachable through this button.
    assert!(
        h.click_ui("Select origin"),
        "and the running pattern's own controls are on screen: {:?}",
        h.frame().text()
    );
    assert!(h.sketch().picking_pattern_center());

    h.click_world(Vec3::ZERO);
    assert_eq!(
        h.sketch().pattern.center,
        Vec2::ZERO,
        "the click placed the origin"
    );
    assert!(!h.sketch().picking_pattern_center(), "and the mode ended");
    assert_eq!(
        h.sketch().sketch.profiles(&Default::default()).len(),
        4,
        "four instances about the sketch origin"
    );
}

/// Construction is a mode you can arm at any time, including with a drawing tool already
/// in hand, and what gets drawn next comes out as reference geometry.
#[test]
fn construction_can_be_armed_with_a_tool_already_chosen() {
    let mut h = Harness::new();
    h.editor.set_window_size([1280, 800]);
    h.start_sketch(PlaneRef::Origin(OriginPlane::XY));
    h.frame();
    h.frame();
    assert!(h.click_ui("Line"), "pick a drawing tool from the toolbar");
    assert_eq!(h.sketch().tool, SketchTool::Line);

    assert!(h.click_ui("Construction"), "then arm construction");
    assert!(h.sketch().construction, "the mode is on");
    assert_eq!(h.sketch().tool, SketchTool::Line, "and the tool is kept");

    h.line(Vec2::new(0.0, 0.0), Vec2::new(10.0, 0.0));
    let s = h.sketch();
    assert!(
        s.sketch
            .entities()
            .filter(|(_, d)| d.entity.is_curve())
            .all(|(_, d)| d.construction),
        "what was drawn next is reference geometry"
    );
}

/// The icon in the variant menu is a button and looks like one, so it has to act like
/// one: it used to react on hover and do nothing on click, with only the name beside it
/// actually choosing the variant.
#[test]
fn the_variant_menu_icon_picks_the_variant() {
    let mut h = Harness::new();
    h.editor.set_window_size([1280, 800]);
    h.start_sketch(PlaneRef::Origin(OriginPlane::XY));
    h.sketch().set_tool(SketchTool::Circle);
    h.frame();
    h.frame();

    // Open the Circle button's variant menu with a right-click, then click the *icon* of
    // the three-point circle rather than its name.
    let button = h
        .icon_rect("toolbar", SketchTool::Circle)
        .expect("the toolbar shows the Circle button");
    h.right_click_ui(button.center());
    let menu = h.frame();
    let name = menu
        .rect_of("Circle (3 pt)")
        .expect("the menu lists the other kinds");
    // The icon sits immediately to the left of the name, in the same row.
    let icon = egui::pos2(name.left() - 16.0, name.center().y);
    h.click_at_ui(icon);
    assert_eq!(
        h.sketch().tool,
        SketchTool::Circle3Point,
        "clicking the icon chose that variant"
    );
}

/// The offset tool, all the way through the real interface: the button is on screen with
/// a selection, the result previews at once, the corner style is a choice you can see the
/// effect of, and OK keeps exactly what is on screen.
///
/// The palette's lower reaches have been off screen before now, which made a tool that
/// existed in the code unusable in the window; the assertions quote what was on screen
/// so a failure says which part went missing.
#[test]
fn the_offset_tool_is_reachable_and_previews_what_it_will_keep() {
    let mut h = Harness::new();
    h.editor.set_window_size([800, 600]);
    h.start_sketch(PlaneRef::Origin(OriginPlane::XY));
    h.rectangle(Vec2::new(0.0, 0.0), Vec2::new(40.0, 20.0));
    {
        let s = h.sketch();
        s.set_tool(SketchTool::Select);
        s.selected = s
            .sketch
            .entities()
            .filter(|(_, d)| d.entity.is_curve())
            .map(|(id, _)| id)
            .collect();
        s.offset.distance = 5.0;
    }
    h.frame();
    h.frame();
    let before = h.sketch().sketch.entities().count();

    assert!(
        h.click_ui("Offset"),
        "the palette offers Offset for a selection: {:?}",
        h.frame().text()
    );
    assert!(h.sketch().offset_in_progress());
    assert_eq!(
        h.sketch().offset_status().map(|(n, _)| n),
        Some(8),
        "rounded corners by default, so four edges and four arcs are already on screen"
    );
    h.frame();
    h.frame();

    // The corner style is a control on the running operation, so it has to be on screen
    // while the operation is running — the whole point is to try one and look at it.
    assert!(
        h.click_ui("Square corners"),
        "and the running offset's own controls are reachable: {:?}",
        h.frame().text()
    );
    assert_eq!(
        h.sketch().offset_status().map(|(n, _)| n),
        Some(4),
        "squaring the corners re-made the offset with no arcs in it"
    );
    h.frame();
    h.frame();

    assert!(h.click_ui("OK"), "{:?}", h.frame().text());
    assert!(!h.sketch().offset_in_progress());
    assert_eq!(
        h.sketch().sketch.entities().count(),
        before + 8,
        "four lines and the four corner points they share"
    );
}

/// The offset's distance is set by dragging a handle on the result in the viewport, not
/// only by a field in a panel — the rule for anything the user enters data into. The
/// handle is a real widget at a real place on the geometry, so the test grabs it where
/// the user would and drags it there.
#[test]
fn the_offset_distance_is_dragged_on_the_geometry() {
    let mut h = Harness::new();
    h.editor.set_window_size([800, 600]);
    h.start_sketch(PlaneRef::Origin(OriginPlane::XY));
    h.rectangle(Vec2::new(0.0, 0.0), Vec2::new(40.0, 20.0));
    {
        let s = h.sketch();
        s.set_tool(SketchTool::Select);
        s.selected = s
            .sketch
            .entities()
            .filter(|(_, d)| d.entity.is_curve())
            .map(|(id, _)| id)
            .collect();
        // A coarse grid, so a snapped drag lands on a multiple of it and a freed one
        // almost certainly does not.
        s.fixed_grid_step = Some(5.0);
        s.offset.distance = 5.0;
    }
    h.frame();
    h.frame();
    assert!(h.click_ui("Offset"));
    h.frame();
    h.frame();

    let slider = super::gizmo::slider(&h.editor).expect("the offset has a handle");
    let ppp = h.points_per_pixel();
    let from = h
        .at_world(slider.grip(), ppp)
        .expect("the handle is on screen");
    // Ten millimetres further out, wherever that lands on screen from here.
    let to = h
        .at_world(slider.anchor + slider.dir * 15.0, ppp)
        .expect("and so is where it is going");
    h.drag_ui(from, to, egui::Modifiers::default());

    let distance = h.sketch().offset.distance;
    assert!(
        (distance - 15.0).abs() < 2.5,
        "the drag set the distance, landing near where it was dropped: {distance}"
    );
    assert!(
        (distance / 5.0).fract().abs() < 1e-9,
        "and it snapped to the grid: {distance}"
    );
    assert!(
        h.sketch()
            .offset_status()
            .is_some_and(|(n, e)| n > 0 && e.is_none()),
        "and the result was re-made at it"
    );

    // Shift lets go of the grid for as long as it is held, so the same gesture lands
    // wherever it was dropped instead of on the nearest line.
    let slider = super::gizmo::slider(&h.editor).expect("still running");
    let from = h.at_world(slider.grip(), ppp).expect("on screen");
    let to = h
        .at_world(slider.anchor + slider.dir * (slider.value + 7.3), ppp)
        .expect("on screen");
    h.drag_ui(
        from,
        to,
        egui::Modifiers {
            shift: true,
            ..Default::default()
        },
    );
    let freed = h.sketch().offset.distance;
    assert!(
        (freed / 5.0).fract().abs() > 1e-6,
        "shift freed the drag from the grid: {freed}"
    );
}

/// Every modelling tool button carries a painted icon, and the icon is part of the
/// button rather than a picture beside it. The sketch variant menus had exactly this
/// bug once — a symbol that lit up on hover and did nothing when clicked — so the
/// assertion is that the click lands on the symbol, not merely somewhere on the row.
#[test]
fn clicking_a_modelling_tool_symbol_starts_the_tool() {
    let mut h = Harness::new();
    h.editor.set_window_size([1280, 800]);
    h.frame();

    let symbol = h
        .symbol_of("toolbar", ToolKind::Extrude)
        .expect("the toolbar shows an Extrude button");
    h.click_at_ui(symbol);
    assert_eq!(
        h.editor.tool.as_ref().map(|t| t.kind),
        Some(ToolKind::Extrude),
        "the symbol is the button"
    );
    h.cancel_tool();

    // And in the menus, which are the other way to every one of these tools.
    h.frame();
    assert!(h.click_ui("Modify"), "the menu bar has a Modify menu");
    let symbol = h
        .symbol_of("modify-menu", ToolKind::Chamfer)
        .expect("the menu lists Chamfer with its icon");
    h.click_at_ui(symbol);
    assert_eq!(
        h.editor.tool.as_ref().map(|t| t.kind),
        Some(ToolKind::Chamfer),
        "clicking the menu row's symbol started it"
    );
}

/// Not one modelling tool is text alone: every button in the 3D toolbar and in the
/// Create and Modify menus has a painted symbol beside its name, filed under the tool's
/// own id. A tool added later without one fails here rather than shipping as a word.
#[test]
fn every_modelling_tool_button_carries_an_icon() {
    let mut h = Harness::new();
    h.editor.set_window_size([1600, 900]);
    h.frame();
    for kind in [
        ToolKind::Sketch,
        ToolKind::Extrude,
        ToolKind::Revolve,
        ToolKind::Sweep,
        ToolKind::Loft,
        ToolKind::Fillet,
        ToolKind::Chamfer,
        ToolKind::Combine,
        ToolKind::Move,
        ToolKind::OffsetPlane,
        ToolKind::AngledPlane,
    ] {
        assert!(
            h.icon_rect("toolbar", kind).is_some(),
            "{} has an icon in the toolbar",
            kind.title()
        );
    }
    // Component is a menu-only tool, and the menus hold the same buttons.
    assert!(h.click_ui("Create"));
    for kind in [
        ToolKind::Sketch,
        ToolKind::Extrude,
        ToolKind::Revolve,
        ToolKind::Sweep,
        ToolKind::Loft,
        ToolKind::OffsetPlane,
        ToolKind::AngledPlane,
        ToolKind::Component,
    ] {
        assert!(
            h.icon_rect("create-menu", kind).is_some(),
            "{} has an icon in the Create menu",
            kind.title()
        );
    }
}

/// The sketch fillet, through the real interface: it is folded under the modify button
/// beside Trim and Break, a click on the corner rounds it, and OK keeps the arc.
#[test]
fn the_sketch_fillet_is_reachable_and_rounds_the_corner_clicked() {
    let mut h = Harness::new();
    h.editor.set_window_size([1280, 800]);
    h.start_sketch(PlaneRef::Origin(OriginPlane::XY));
    h.rectangle(Vec2::new(0.0, 0.0), Vec2::new(40.0, 20.0));
    h.sketch().set_tool(SketchTool::Trim);
    h.sketch().fillet.radius = 4.0;
    h.frame();
    h.frame();

    // The folded button shows Trim; the fillet is one of its variants.
    let button = h
        .icon_rect("toolbar", SketchTool::Trim)
        .expect("the sketch toolbar shows the modify button");
    h.right_click_ui(button.center());
    let menu = h.frame();
    let name = menu
        .rect_of("Fillet")
        .expect("the menu lists the fillet among the modify tools");
    h.click_at_ui(egui::pos2(name.left() - 16.0, name.center().y));
    assert_eq!(
        h.sketch().tool,
        SketchTool::Fillet,
        "clicking its icon armed it"
    );

    h.click_world(Vec3::ZERO);
    assert!(
        h.sketch().fillet_in_progress(),
        "one click on the corner started the fillet"
    );
    h.frame();
    h.frame();
    assert!(h.click_ui("OK"), "{:?}", h.frame().text());
    assert!(!h.sketch().fillet_in_progress());
    let arcs = h
        .sketch()
        .sketch
        .entities()
        .filter(|(_, d)| matches!(d.entity, basset_sketch::Entity::Arc { .. }))
        .count();
    assert_eq!(arcs, 1, "the corner is an arc now");
}

/// Hovering a badge lights up the geometry it holds, and hovering the geometry lights up
/// its badges. The palette's constraint list already pointed from a row to the drawing;
/// this is the same link read from the drawing, which is where the user is looking.
#[test]
fn hovering_a_constraint_badge_lights_up_the_geometry_it_holds() {
    let mut h = Harness::new();
    h.start_sketch(PlaneRef::Origin(OriginPlane::XY));
    h.rectangle(Vec2::new(0.0, 0.0), Vec2::new(20.0, 10.0));

    // Away from every badge first, as the baseline for what is lit.
    h.move_world(Vec3::new(60.0, 60.0, 0.0));
    let quiet = amber_segments(&mut h);
    assert_eq!(
        h.sketch().hovered_constraint,
        None,
        "nothing is hovered out here"
    );

    let (center, id) = {
        let s = h.sketch();
        let g = s
            .constraint_glyphs()
            .into_iter()
            .next()
            .expect("a rectangle is held by badged constraints");
        (g.center, g.id)
    };
    h.move_world(center);
    {
        let s = h.sketch();
        assert_eq!(
            s.hovered_constraint,
            Some(id),
            "the badge under the pointer"
        );
        assert!(
            !s.constraint_hover_entities().is_empty(),
            "and it names the geometry it holds"
        );
    }
    assert!(
        amber_segments(&mut h) > quiet,
        "both the badge and its geometry are drawn lit"
    );
}

/// Segments drawn in the hover amber: the geometry under the pointer and any badge lit
/// with it.
fn amber_segments(h: &mut Harness) -> usize {
    let mut lines = Vec::new();
    h.sketch()
        .draw(&mut lines, &mut Vec::new(), &mut Vec::new());
    lines
        .iter()
        .filter(|b| b.color == [1.0, 0.85, 0.3, 1.0])
        .map(|b| b.segments.len())
        .sum()
}

/// Grid snapping is one rule, so it has to hold for a handle that moves geometry as well
/// as for one that sets a size. This is the sketch's move arrow, grabbed where it is
/// drawn and dragged: the offset it leaves behind lands on the grid, and the same
/// gesture with shift held lands wherever it was dropped.
#[test]
fn the_sketch_move_arrow_snaps_and_shift_lets_go() {
    let mut h = Harness::new();
    h.start_sketch(PlaneRef::Origin(OriginPlane::XY));
    h.rectangle(Vec2::ZERO, Vec2::new(40.0, 20.0));
    {
        let s = h.sketch();
        s.set_tool(SketchTool::Select);
        s.selected = s
            .sketch
            .entities()
            .filter(|(_, d)| d.entity.is_curve())
            .map(|(id, _)| id)
            .collect();
        // A coarse grid, pinned so the zoom cannot move it under the test.
        s.fixed_grid_step = Some(5.0);
        s.grid_step = 5.0;
        assert!(s.begin_move(), "the selection can be moved");
    }
    h.frame();

    let camera = h.editor.camera;
    let window = h.editor.window_px;
    let ppp = h.points_per_pixel();
    let grab = |h: &Harness, reach: f64| {
        let g = super::gizmo::current(&h.editor).expect("the move has a manipulator");
        let arm = g.arm(&camera, window);
        let dir = g.arrows[0].dir;
        let from = h.at_world(g.origin + dir * arm, ppp).expect("on screen");
        let to = h
            .at_world(g.origin + dir * (arm + reach), ppp)
            .expect("on screen");
        (from, to)
    };

    let (from, to) = grab(&h, 12.0);
    let during = h.drag_ui(from, to, egui::Modifiers::default());
    let dx = h.sketch().move_op.as_ref().expect("still moving").dx;
    assert!((dx - 12.0).abs() < 4.0, "the drag moved the geometry: {dx}");
    assert!((dx / 5.0).fract().abs() < 1e-9, "onto the grid: {dx}");
    assert!(
        during.iter().any(|t| t.contains("grid 5")),
        "and said so while it was happening: {during:?}"
    );

    // The same gesture with shift held: no grid, and the label says as much.
    let (from, to) = grab(&h, 7.3);
    let during = h.drag_ui(
        from,
        to,
        egui::Modifiers {
            shift: true,
            ..Default::default()
        },
    );
    let freed = h.sketch().move_op.as_ref().expect("still moving").dx;
    assert!(
        (freed / 5.0).fract().abs() > 1e-6,
        "shift freed the drag from the grid: {freed} (was {dx})"
    );
    assert!(
        during.iter().any(|t| t.contains("free")),
        "and the label says the grid is not holding: {during:?}"
    );
}

/// The same rule on a feature's size arrow, which is a different code path entirely: the
/// extrude distance is dragged on the geometry, lands on the grid, and gets out of its
/// way while shift is held.
#[test]
fn the_extrude_distance_arrow_snaps_and_shift_lets_go() {
    let mut h = Harness::new();
    h.start_sketch(PlaneRef::Origin(OriginPlane::XY));
    h.rectangle(Vec2::ZERO, Vec2::new(40.0, 20.0));
    h.finish_sketch(true);
    let sketch = h.last_feature();
    h.start_tool(ToolKind::Extrude);
    h.select_region(sketch, Vec2::new(20.0, 10.0));
    h.sync_tool();
    h.frame();

    let handle = super::tools::handle(&h.editor).expect("the extrude has an arrow");
    // The increment a modelling handle snaps to follows the zoom, so the test asks for
    // the same one the handle will rather than assuming a number.
    let step = basset_viewport::grid::snap_step_for(
        h.editor
            .camera
            .pixel_size_at(handle.tip, h.editor.window_px),
    );
    let ppp = h.points_per_pixel();
    let grab = |h: &Harness, reach: f64| {
        let handle = super::tools::handle(&h.editor).expect("still running");
        let from = h.at_world(handle.tip, ppp).expect("on screen");
        let to = h
            .at_world(handle.tip + handle.dir * reach, ppp)
            .expect("on screen");
        (from, to)
    };
    let distance = |h: &Harness| h.editor.tool.as_ref().expect("running").params.distance;

    let before = distance(&h);
    let (from, to) = grab(&h, step * 3.0);
    let during = h.drag_ui(from, to, egui::Modifiers::default());
    let snapped = distance(&h);
    assert!(
        snapped > before,
        "the drag grew the extrude: {before} -> {snapped}"
    );
    assert!(
        (snapped / step).fract().abs() < 1e-9,
        "onto the grid ({step}): {snapped}"
    );
    assert!(
        during.iter().any(|t| t.contains("grid")),
        "and said so while it was happening: {during:?}"
    );

    let (from, to) = grab(&h, step * 2.37);
    h.drag_ui(
        from,
        to,
        egui::Modifiers {
            shift: true,
            ..Default::default()
        },
    );
    let freed = distance(&h);
    assert!(
        (freed / step).fract().abs() > 1e-6,
        "shift freed the drag from the grid: {freed} (step {step})"
    );

    // The master switch reaches the modelling handles as well as the sketch: off means
    // off, with nothing held. A snapping drag would have rounded this back onto a
    // multiple of the step.
    h.editor.set_snapping(false);
    let (from, to) = grab(&h, step * 1.41);
    h.drag_ui(from, to, egui::Modifiers::default());
    let off = distance(&h);
    assert!(
        (off / step).fract().abs() > 1e-6,
        "the switch is off, so nothing snapped: {off} (step {step})"
    );
}
