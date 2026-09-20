//! Headless tests of the interaction logic: tool dialogs and sketch mode drive the
//! document exactly as the UI would, without a window or GPU.

use basset_core::{BodyRef, FeatureKind, OriginPlane, PlaneRef, ProfileRef, RegionRef};
use basset_math::{Vec2, Vec3};
use basset_sketch::{Constraint, Entity, EntityId};

use super::harness::{
    block, click_at, click_with, dimension, draw_line, draw_rectangle, point_at, sketch, top_face,
};
use super::sketch_mode::{self, SketchTool};
use super::tools::{self, ToolKind};
use super::{Editor, Mode};

#[test]
fn sketch_mode_draws_and_finishes_a_profile() {
    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    assert!(editor.is_sketching());
    let sketch_id = match &editor.mode {
        Mode::Sketch(s) => s.feature,
        Mode::Model => unreachable!(),
    };
    draw_rectangle(&mut editor, Vec2::new(0.0, 0.0), Vec2::new(20.0, 10.0));
    // Lines continue from the rectangle's corner because the click snaps to it.
    {
        let camera = editor.camera;
        let window = editor.window_px;
        let Mode::Sketch(s) = &mut editor.mode else {
            unreachable!()
        };
        s.set_tool(SketchTool::Line);
        for p in [Vec2::new(20.0, 10.0), Vec2::new(30.0, 20.0)] {
            s.pointer_moved(&click_at(p.x, p.y), &camera, window, false);
            s.pointer_up(&click_at(p.x, p.y), &camera, window, true, false);
        }
        let points = s
            .sketch
            .entities()
            .filter(|(_, e)| e.entity.is_point())
            .count();
        assert_eq!(
            points, 5,
            "the shared corner was reused rather than duplicated"
        );
        assert!(s.has_pending());
        s.finish_current();
        assert!(!s.has_pending());
    }
    sketch_mode::finish(&mut editor, true);
    assert!(!editor.is_sketching());
    let state = editor.doc.state();
    let solved = state
        .sketches
        .get(&sketch_id)
        .expect("sketch feature evaluated");
    assert_eq!(solved.profiles.len(), 1);
    assert!((solved.profiles[0].area() - 200.0).abs() < 1e-6);
    assert!(editor.doc.can_undo());
    assert!(
        editor.doc.undo(),
        "the whole sketch session is one undo step"
    );
    assert!(editor.doc.timeline().is_empty());
}

#[test]
fn cancelled_sketch_leaves_no_feature() {
    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    draw_rectangle(&mut editor, Vec2::new(0.0, 0.0), Vec2::new(5.0, 5.0));
    sketch_mode::finish(&mut editor, false);
    assert!(editor.doc.timeline().is_empty());
    assert!(!editor.doc.in_transaction());
}

#[test]
fn extrude_tool_previews_then_commits_or_rolls_back() {
    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    draw_rectangle(&mut editor, Vec2::new(0.0, 0.0), Vec2::new(10.0, 10.0));
    sketch_mode::finish(&mut editor, true);
    let sketch = editor.doc.timeline().features()[0].id;

    tools::start_tool(&mut editor, ToolKind::Extrude);
    assert!(
        editor.tool.as_ref().unwrap().feature.is_none(),
        "nothing to preview yet"
    );
    editor.selection.profiles.push(ProfileRef {
        sketch,
        sample: Vec2::new(5.0, 5.0),
    });
    tools::sync_tool(&mut editor);
    let feature = editor
        .tool
        .as_ref()
        .unwrap()
        .feature
        .expect("preview feature created");
    editor.tool.as_mut().unwrap().params.distance = 4.0;
    tools::sync_tool(&mut editor);
    let volume = editor
        .doc
        .state()
        .body(BodyRef(feature))
        .unwrap()
        .solid
        .volume();
    assert!((volume - 400.0).abs() < 1e-6, "{volume}");

    tools::cancel_tool(&mut editor);
    assert!(editor.doc.state().body(BodyRef(feature)).is_none());
    assert_eq!(editor.doc.timeline().len(), 1);

    tools::start_tool(&mut editor, ToolKind::Extrude);
    editor.selection.profiles.push(ProfileRef {
        sketch,
        sample: Vec2::new(5.0, 5.0),
    });
    tools::sync_tool(&mut editor);
    let feature = editor.tool.as_ref().unwrap().feature.unwrap();
    tools::confirm_tool(&mut editor);
    assert!(editor.tool.is_none());
    assert!(editor.doc.state().body(BodyRef(feature)).is_some());
    assert!(editor.doc.undo());
    assert_eq!(
        editor.doc.timeline().len(),
        1,
        "the tool interaction was one undo step"
    );
}

#[test]
fn editing_an_existing_extrude_loads_its_parameters() {
    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    draw_rectangle(&mut editor, Vec2::new(0.0, 0.0), Vec2::new(10.0, 10.0));
    sketch_mode::finish(&mut editor, true);
    let sketch = editor.doc.timeline().features()[0].id;
    let extrude = editor.doc.add_feature(FeatureKind::Extrude {
        regions: vec![RegionRef::Profile(ProfileRef {
            sketch,
            sample: Vec2::new(5.0, 5.0),
        })],
        extent: basset_core::Extent::OneSide(7.0),
        operation: basset_core::BodyOp::NewBody,
        component: basset_core::ComponentId::ROOT,
    });
    editor.edit_feature(extrude);
    let tool = editor.tool.as_ref().expect("tool dialog opened");
    assert_eq!(tool.kind, ToolKind::Extrude);
    assert_eq!(tool.params.distance, 7.0);
    assert_eq!(editor.selection.profiles.len(), 1);
    editor.tool.as_mut().unwrap().params.distance = 2.0;
    tools::sync_tool(&mut editor);
    tools::confirm_tool(&mut editor);
    let aabb = editor
        .doc
        .state()
        .body(BodyRef(extrude))
        .unwrap()
        .solid
        .aabb();
    assert!((aabb.max.z - 2.0).abs() < 1e-9);
    assert_eq!(
        editor.doc.timeline().cursor(),
        2,
        "cursor restored after editing"
    );
}

#[test]
fn dimension_tool_adds_a_driving_dimension() {
    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    let camera = editor.camera;
    let window = editor.window_px;
    let Mode::Sketch(s) = &mut editor.mode else {
        unreachable!()
    };
    s.set_tool(SketchTool::Circle);
    for p in [Vec2::ZERO, Vec2::new(5.0, 0.0)] {
        s.pointer_up(&click_at(p.x, p.y), &camera, window, true, false);
    }
    let circle = s
        .sketch
        .entities()
        .find(|(_, e)| matches!(e.entity, Entity::Circle { .. }))
        .map(|(id, _)| id)
        .unwrap();
    s.set_tool(SketchTool::Dimension);
    // Click on the circle's rim, then on empty space to place the dimension.
    s.pointer_up(&click_at(5.0, 0.0), &camera, window, true, false);
    assert!(s.dim_edit.is_none(), "one pick is not yet a dimension");
    s.pointer_up(&click_at(30.0, 30.0), &camera, window, true, false);
    let (cid, _) = s.dim_edit.clone().expect("dimension editor opened");
    assert!(
        matches!(s.sketch.constraint(cid), Some(basset_sketch::Constraint::Diameter { curve, .. }) if *curve == circle)
    );
    s.set_dimension(cid, 20.0);
    let Some(Entity::Circle { radius, .. }) = s.sketch.entity(circle).map(|e| e.entity.clone())
    else {
        unreachable!()
    };
    assert!((radius - 10.0).abs() < 1e-6, "{radius}");
    assert!(s.undo());
    let Some(Entity::Circle { radius, .. }) = s.sketch.entity(circle).map(|e| e.entity.clone())
    else {
        unreachable!()
    };
    assert!((radius - 5.0).abs() < 1e-6);
}

/// Points the user places land on the grid, so a sketch drawn by eye still has round
/// dimensions. The step follows the zoom unless it is pinned.
#[test]
fn new_points_snap_to_the_grid() {
    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    let camera = editor.camera;
    let window = editor.window_px;
    let Mode::Sketch(s) = &mut editor.mode else {
        panic!("not sketching")
    };
    s.fixed_grid_step = Some(1.0);
    s.set_tool(SketchTool::Line);
    for p in [Vec2::new(2.4, 3.7), Vec2::new(9.2, 3.1)] {
        s.pointer_moved(&click_at(p.x, p.y), &camera, window, false);
        s.pointer_up(&click_at(p.x, p.y), &camera, window, true, false);
    }
    let mut points: Vec<Vec2> = s
        .sketch
        .entities()
        .filter_map(|(_, e)| match e.entity {
            Entity::Point { pos } => Some(pos),
            _ => None,
        })
        .collect();
    points.sort_by(|a, b| a.x.total_cmp(&b.x));
    assert_eq!(points, vec![Vec2::new(2.0, 4.0), Vec2::new(9.0, 3.0)]);

    // The automatic step is the one the drawn grid implies at this zoom.
    s.fixed_grid_step = None;
    s.pointer_moved(&click_at(1.0, 1.0), &camera, window, false);
    let expected = basset_viewport::grid::snap_step_for(camera.pixel_size_at(Vec3::ZERO, window));
    assert_eq!(s.grid_step, expected);

    // Turning snapping off puts the point exactly where the pointer was.
    s.snap_to_grid = false;
    s.cancel_current();
    for p in [Vec2::new(30.4, 40.7), Vec2::new(41.3, 40.2)] {
        s.pointer_moved(&click_at(p.x, p.y), &camera, window, false);
        s.pointer_up(&click_at(p.x, p.y), &camera, window, true, false);
    }
    assert!(
        s.sketch.entities().any(
            |(_, e)| matches!(e.entity, Entity::Point { pos } if pos == Vec2::new(30.4, 40.7))
        ),
    );
}

/// Dragging on empty space is a rubber band. Rightwards encloses, leftwards touches, which
/// is the convention every CAD user already knows.
#[test]
fn marquee_selects_by_window_and_by_crossing() {
    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    draw_rectangle(&mut editor, Vec2::new(0.0, 0.0), Vec2::new(20.0, 20.0));
    let camera = editor.camera;
    let window = editor.window_px;
    let Mode::Sketch(s) = &mut editor.mode else {
        panic!("not sketching")
    };
    s.set_tool(SketchTool::Select);

    fn band(
        s: &mut super::SketchEditor,
        camera: &basset_viewport::Camera,
        window: [u32; 2],
        from: Vec2,
        to: Vec2,
        shift: bool,
    ) {
        s.pointer_down(&click_at(from.x, from.y), camera, window, shift);
        s.pointer_moved(&click_at(to.x, to.y), camera, window, true);
        s.pointer_up(&click_at(to.x, to.y), camera, window, false, shift);
    }

    // A window around the whole rectangle takes everything: four corners and four lines.
    band(
        s,
        &camera,
        window,
        Vec2::new(-5.0, -5.0),
        Vec2::new(25.0, 25.0),
        false,
    );
    assert_eq!(s.selected.len(), 8);

    // A window over the left half takes only what lies wholly inside: the two left
    // corners and the left edge. The other three edges stick out of it.
    band(
        s,
        &camera,
        window,
        Vec2::new(-5.0, -5.0),
        Vec2::new(10.0, 25.0),
        false,
    );
    assert_eq!(s.selected.len(), 3);

    // The same rectangle drawn leftwards is a crossing band, so the edges it merely
    // touches come along: the top and bottom, but not the far right edge.
    band(
        s,
        &camera,
        window,
        Vec2::new(10.0, 25.0),
        Vec2::new(-5.0, -5.0),
        false,
    );
    assert_eq!(s.selected.len(), 5);

    // Shift adds to the selection instead of replacing it.
    band(
        s,
        &camera,
        window,
        Vec2::new(-5.0, -5.0),
        Vec2::new(25.0, 25.0),
        true,
    );
    assert_eq!(
        s.selected.len(),
        8,
        "the band's hits are added, not doubled"
    );

    // A press and release without travel is still an ordinary click that clears.
    s.pointer_down(&click_at(50.0, 50.0), &camera, window, false);
    s.pointer_up(&click_at(50.0, 50.0), &camera, window, true, false);
    assert!(s.selected.is_empty());
}

/// A planar face of a body is a region in its own right, so Extrude accepts one with no
/// sketch involved.
#[test]
fn extrude_tool_accepts_a_planar_face_as_a_region() {
    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    draw_rectangle(&mut editor, Vec2::new(0.0, 0.0), Vec2::new(10.0, 10.0));
    sketch_mode::finish(&mut editor, true);
    let sketch = editor.doc.timeline().features()[0].id;
    let base = editor.doc.add_feature(FeatureKind::Extrude {
        regions: vec![RegionRef::Profile(ProfileRef {
            sketch,
            sample: Vec2::new(5.0, 5.0),
        })],
        extent: basset_core::Extent::OneSide(2.0),
        operation: basset_core::BodyOp::NewBody,
        component: basset_core::ComponentId::ROOT,
    });
    let top = basset_core::FaceRef {
        body: BodyRef(base),
        key: basset_core::FaceKey::new(
            basset_kernel::OpId::new(base.0),
            basset_core::FaceRole::EndCap,
        ),
    };

    tools::start_tool(&mut editor, ToolKind::Extrude);
    editor.selection.faces.push(top);
    editor.tool.as_mut().unwrap().params.distance = 3.0;
    // No operation was chosen: growing out of a body's face joins that body, as
    // Fusion's default does.
    tools::sync_tool(&mut editor);
    assert_eq!(
        editor.tool.as_ref().unwrap().params.op,
        super::tools::OpKind::Join
    );
    assert_eq!(
        editor.tool.as_ref().unwrap().params.target,
        Some(BodyRef(base))
    );
    let feature = editor
        .tool
        .as_ref()
        .unwrap()
        .feature
        .expect("a face alone is enough to build the feature");
    match &editor.doc.timeline().get(feature).unwrap().kind {
        FeatureKind::Extrude { regions, .. } => {
            assert_eq!(regions, &vec![RegionRef::Face(top)]);
        }
        other => panic!("{other:?}"),
    }
    tools::confirm_tool(&mut editor);
    let state = editor.doc.state();
    assert!(state.failed_features().next().is_none());
    let solid = &state.body(BodyRef(base)).unwrap().solid;
    assert!((solid.volume() - 500.0).abs() < 1e-6, "{}", solid.volume());

    // Re-opening the feature puts the face back in the selection it was built from.
    editor.edit_feature(feature);
    assert_eq!(editor.selection.faces, vec![top]);
}

/// The crosshair sits on the snapped position, not the raw pointer, and says when the next
/// click will reuse an existing point instead of making one.
#[test]
fn cursor_marker_tracks_the_snapped_position() {
    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    let camera = editor.camera;
    let window = editor.window_px;
    let Mode::Sketch(s) = &mut editor.mode else {
        panic!("not sketching")
    };
    s.fixed_grid_step = Some(1.0);
    s.set_tool(SketchTool::Line);

    // Marker centre in world space, for whichever batch the cursor produced.
    let marker = |s: &sketch_mode::SketchEditor| {
        let mut lines = Vec::new();
        let mut points = Vec::new();
        s.draw(&mut lines, &mut points, &mut Vec::new());
        points.last().and_then(|b| b.points.last().copied())
    };

    s.pointer_moved(&click_at(2.4, 3.7), &camera, window, false);
    assert!(!s.cursor_snapped);
    assert_eq!(marker(s), Some(Vec3::new(2.0, 4.0, 0.0)));

    // Drawing a line leaves a point at (2, 4); coming back near it snaps onto it.
    s.pointer_up(&click_at(2.4, 3.7), &camera, window, true, false);
    s.pointer_moved(&click_at(9.2, 3.1), &camera, window, false);
    s.pointer_up(&click_at(9.2, 3.1), &camera, window, true, false);
    s.finish_current();
    s.pointer_moved(&click_at(2.02, 4.03), &camera, window, false);
    assert!(s.cursor_snapped);
    assert_eq!(marker(s), Some(Vec3::new(2.0, 4.0, 0.0)));

    // Select is a picking tool, not a placing one: no marker.
    s.set_tool(SketchTool::Select);
    s.pointer_moved(&click_at(2.4, 3.7), &camera, window, false);
    let mut lines = Vec::new();
    let mut points = Vec::new();
    s.draw(&mut lines, &mut points, &mut Vec::new());
    assert!(points.iter().all(|b| b.size_px != 5.0 && b.size_px != 9.0));
}

/// A shape drawn inside another gives two regions, so the extrude tool can take either the
/// frame around it or the shape itself.
#[test]
fn a_shape_inside_a_rectangle_is_its_own_region() {
    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    draw_rectangle(&mut editor, Vec2::new(0.0, 0.0), Vec2::new(20.0, 20.0));
    draw_rectangle(&mut editor, Vec2::new(5.0, 5.0), Vec2::new(15.0, 15.0));
    sketch_mode::finish(&mut editor, true);
    let sketch = editor.doc.timeline().features()[0].id;
    assert_eq!(
        editor.doc.state().sketches[&sketch].profiles.len(),
        2,
        "the frame and the inner square"
    );

    // Extruding each sample point in turn: the click decides which region is pushed.
    for (sample, expected) in [
        (Vec2::new(10.0, 10.0), 100.0 * 4.0),
        (Vec2::new(2.0, 2.0), (400.0 - 100.0) * 4.0),
    ] {
        tools::start_tool(&mut editor, ToolKind::Extrude);
        editor
            .selection
            .profiles
            .push(ProfileRef { sketch, sample });
        editor.tool.as_mut().unwrap().params.distance = 4.0;
        tools::sync_tool(&mut editor);
        let feature = editor.tool.as_ref().unwrap().feature.unwrap();
        let volume = editor
            .doc
            .state()
            .body(BodyRef(feature))
            .unwrap()
            .solid
            .volume();
        assert!((volume - expected).abs() < 1e-6, "{sample:?}: {volume}");
        tools::cancel_tool(&mut editor);
        editor.selection.clear();
    }
}

/// The selection mode decides what a click lands on, so a corner, a curve and a region in
/// the same place stay individually reachable.
#[test]
fn select_mode_narrows_what_a_click_picks() {
    use super::SelectMode;
    use super::selection::{self, Pick};

    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    draw_rectangle(&mut editor, Vec2::new(0.0, 0.0), Vec2::new(10.0, 10.0));
    sketch_mode::finish(&mut editor, true);
    editor.refresh_cache();
    let sketch = editor.doc.timeline().features()[0].id;

    let pick = |editor: &Editor, x: f64, y: f64| {
        selection::pick(editor, &click_at(x, y), &editor.pick_filter(), 8.0)
    };

    // Sketch mode: the corner is a point, the middle of an edge is a curve, and the
    // inside is a region -- three different answers over the same rectangle.
    editor.set_select_mode(SelectMode::Sketch);
    assert!(matches!(
        pick(&editor, 0.0, 0.0),
        Some(Pick::Point { sketch: s, .. }) if s == sketch
    ));
    assert!(matches!(
        pick(&editor, 5.0, 0.0),
        Some(Pick::Curve { sketch: s, .. }) if s == sketch
    ));
    assert!(matches!(pick(&editor, 5.0, 5.0), Some(Pick::Profile(..))));

    // Vertex mode still finds the corner but no longer the curve or the region.
    editor.set_select_mode(SelectMode::Vertices);
    assert!(matches!(pick(&editor, 0.0, 0.0), Some(Pick::Point { .. })));
    assert_eq!(pick(&editor, 5.0, 0.0), None);
    assert_eq!(pick(&editor, 5.0, 5.0), None);

    // Face mode takes the enclosed region -- a sketch region is a face -- and nothing
    // outside it, since curves and points are not faces.
    editor.set_select_mode(SelectMode::Faces);
    assert!(matches!(pick(&editor, 5.0, 5.0), Some(Pick::Profile(..))));
    assert_eq!(pick(&editor, 15.0, 15.0), None);

    // Any is the default and leaves sketches alone, as before.
    editor.set_select_mode(SelectMode::Any);
    assert_eq!(pick(&editor, 0.0, 0.0), None);

    // Clicking in Sketch mode records the point.
    editor.set_select_mode(SelectMode::Sketch);
    let p = pick(&editor, 0.0, 0.0);
    editor.apply_pick(p, false);
    assert_eq!(editor.selection.points.len(), 1);
    assert_eq!(editor.selection.summary(), "1 point");
    // Changing mode drops a selection the new mode could not have made.
    editor.set_select_mode(SelectMode::Faces);
    assert!(editor.selection.points.is_empty());
}

/// A mode narrows a running tool, but never to nothing: Fillet takes edges, and asking for
/// vertices while it runs must not leave it unusable.
#[test]
fn a_mode_never_makes_a_tool_unpickable() {
    use super::SelectMode;

    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    tools::start_tool(&mut editor, ToolKind::Fillet);
    editor.select_mode = SelectMode::Edges;
    assert!(editor.pick_filter().edges);
    editor.select_mode = SelectMode::Vertices;
    let filter = editor.pick_filter();
    assert!(filter.edges, "fell back to what Fillet accepts");
    assert!(!filter.vertices);

    // A sketch region is a face: Face mode picks both, and leaves Extrude's own filter
    // (which is exactly those two) alone.
    tools::cancel_tool(&mut editor);
    tools::start_tool(&mut editor, ToolKind::Extrude);
    editor.select_mode = SelectMode::Faces;
    let filter = editor.pick_filter();
    assert!(filter.faces && filter.profiles);
    assert!(!filter.curves && !filter.edges);

    // Any restricts nothing: Sketch needs planes, which are not in the no-tool filter.
    tools::cancel_tool(&mut editor);
    tools::start_tool(&mut editor, ToolKind::Sketch);
    editor.select_mode = SelectMode::Any;
    assert!(editor.pick_filter().planes, "Sketch can still pick a plane");
}

/// Selecting a region fills it. An outline alone reads as "these curves are selected",
/// which is not what picking an enclosed area means.
#[test]
fn a_selected_region_is_drawn_filled() {
    use super::scene;

    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    draw_rectangle(&mut editor, Vec2::new(0.0, 0.0), Vec2::new(10.0, 10.0));
    sketch_mode::finish(&mut editor, true);
    editor.refresh_cache();
    let sketch = editor.doc.timeline().features()[0].id;

    assert!(
        scene::build(&editor).tris.is_empty(),
        "nothing selected yet"
    );

    editor.selection.profiles.push(ProfileRef {
        sketch,
        sample: Vec2::new(5.0, 5.0),
    });
    let built = scene::build(&editor);
    let area: f64 = built
        .tris
        .iter()
        .flat_map(|t| &t.triangles)
        .map(|[a, b, c]| (*b - *a).cross(*c - *a).length() * 0.5)
        .sum();
    assert!(
        (area - 100.0).abs() < 1e-6,
        "the whole region is filled: {area}"
    );
    assert!(
        built.tris.iter().all(|t| t.color[3] < 1.0),
        "the fill is translucent, so the geometry under it stays readable"
    );
}

/// What the dimension tool measures follows from what was picked, as in Fusion: two
/// parallel edges give their distance, a centre and an edge their distance, a circle
/// alone its diameter, and two edges that are not parallel their angle.
#[test]
fn dimension_tool_reads_the_picks_like_fusion() {
    use basset_sketch::Constraint;
    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    draw_rectangle(&mut editor, Vec2::ZERO, Vec2::new(10.0, 10.0));
    let camera = editor.camera;
    let window = editor.window_px;
    let Mode::Sketch(s) = &mut editor.mode else {
        unreachable!()
    };
    // The coordinates below are chosen for the geometry, not the grid.
    s.snap_to_grid = false;
    s.set_tool(SketchTool::Circle);
    for p in [Vec2::new(20.0, 5.0), Vec2::new(21.0, 5.0)] {
        s.pointer_up(&click_at(p.x, p.y), &camera, window, true, false);
    }
    let bottom_left = point_at(s, Vec2::ZERO);
    let centre = point_at(s, Vec2::new(20.0, 5.0));

    // Bottom edge and top edge are parallel: distance between them.
    let (_, c) = dimension(
        s,
        &camera,
        window,
        Vec2::new(5.0, 0.0),
        Vec2::new(5.0, 10.0),
    );
    let Constraint::Distance { a, value, .. } = c else {
        panic!("parallel edges should give a distance, got {c:?}")
    };
    assert_eq!(
        a, bottom_left,
        "measured from an endpoint of the first edge"
    );
    assert!((value - 10.0).abs() < 1e-9, "{value}");

    // Circle centre and the bottom edge: distance from the point to the line.
    let (_, c) = dimension(
        s,
        &camera,
        window,
        Vec2::new(20.0, 5.0),
        Vec2::new(5.0, 0.0),
    );
    let Constraint::Distance { a, value, .. } = c else {
        panic!("centre and edge should give a distance, got {c:?}")
    };
    assert_eq!(a, centre);
    assert!((value - 5.0).abs() < 1e-9, "{value}");

    // The circle's rim and the top edge: the circle stands in for its centre.
    let (_, c) = dimension(
        s,
        &camera,
        window,
        Vec2::new(21.0, 5.0),
        Vec2::new(5.0, 10.0),
    );
    let Constraint::Distance { a, value, .. } = c else {
        panic!("circle and edge should give a distance, got {c:?}")
    };
    assert_eq!(a, centre);
    assert!((value - 5.0).abs() < 1e-9, "{value}");

    // Bottom edge and right edge are perpendicular: an angle.
    let (_, c) = dimension(
        s,
        &camera,
        window,
        Vec2::new(5.0, 0.0),
        Vec2::new(10.0, 5.0),
    );
    let Constraint::Angle { value, .. } = c else {
        panic!("non-parallel edges should give an angle, got {c:?}")
    };
    assert!(
        (value.abs() - std::f64::consts::FRAC_PI_2).abs() < 1e-9,
        "{value}"
    );
    // Every dimension was added at its current value, so nothing moved.
    assert!(s.report.as_ref().unwrap().as_ref().unwrap().converged);
    let corner = s
        .sketch
        .point_pos(point_at(s, Vec2::new(10.0, 10.0)))
        .unwrap();
    assert!(corner.distance(Vec2::new(10.0, 10.0)) < 1e-6, "{corner}");
}

/// An angle dimension keeps the sign of the angle it measures. Storing it unsigned
/// used to swing the second line to its mirror image the moment the dimension was
/// added, and retyping the value must not flip it either.
#[test]
fn angle_dimension_keeps_the_lines_where_they_are() {
    use basset_sketch::Constraint;
    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    let camera = editor.camera;
    let window = editor.window_px;
    let Mode::Sketch(s) = &mut editor.mode else {
        unreachable!()
    };
    // The coordinates below are chosen for the geometry, not the grid.
    s.snap_to_grid = false;
    s.set_tool(SketchTool::Line);
    // A base line along +x, then a separate line (well clear of it, so no click snaps
    // onto the base) falling away at -45°.
    for p in [Vec2::ZERO, Vec2::new(10.0, 0.0)] {
        s.pointer_up(&click_at(p.x, p.y), &camera, window, true, false);
    }
    s.finish_current();
    for p in [Vec2::new(0.0, -10.0), Vec2::new(10.0, -20.0)] {
        s.pointer_up(&click_at(p.x, p.y), &camera, window, true, false);
    }
    s.finish_current();
    let tip = point_at(s, Vec2::new(10.0, -20.0));
    let (_, c) = dimension(
        s,
        &camera,
        window,
        Vec2::new(5.0, 0.0),
        Vec2::new(5.0, -15.0),
    );
    let Constraint::Angle { value, .. } = c else {
        panic!("expected an angle, got {c:?}")
    };
    assert!(
        (value + std::f64::consts::FRAC_PI_4).abs() < 1e-9,
        "signed: {value}"
    );
    let pos = s.sketch.point_pos(tip).unwrap();
    assert!(
        pos.distance(Vec2::new(10.0, -20.0)) < 1e-6,
        "line flipped to {pos}"
    );

    // The box shows 45 and a typed 30 stays on the same side.
    let (cid, _) = s
        .sketch
        .constraints()
        .find(|(_, c)| matches!(c, Constraint::Angle { .. }))
        .unwrap();
    assert_eq!(sketch_mode::edit_text(&c), "45.00");
    let typed = sketch_mode::parse_value(&c, "30").unwrap();
    assert!((typed + 30f64.to_radians()).abs() < 1e-12);
    s.set_dimension(cid, typed);
    let pos = s.sketch.point_pos(tip).unwrap();
    assert!(pos.y < -10.0, "still below the base line: {pos}");
}

/// Typing a size pins it while the pointer chooses the side, Enter places the shape,
/// and what was typed becomes a driving dimension.
#[test]
fn typed_sizes_place_the_shape_and_become_dimensions() {
    use basset_sketch::Constraint;
    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    let camera = editor.camera;
    let window = editor.window_px;
    let Mode::Sketch(s) = &mut editor.mode else {
        unreachable!()
    };
    // The coordinates below are chosen for the geometry, not the grid.
    s.snap_to_grid = false;
    s.set_tool(SketchTool::Rectangle);
    assert!(
        !s.type_into_entry("1"),
        "nothing to size before the first click"
    );
    s.pointer_up(&click_at(0.0, 0.0), &camera, window, true, false);
    // The pointer is somewhere up and to the left; the typed sizes decide how far.
    s.pointer_moved(&click_at(-3.0, 2.0), &camera, window, false);
    assert_eq!(s.entries[0].text, "3.00", "boxes follow the pointer");
    assert_eq!(s.entries[1].text, "2.00");
    assert!(!s.entries[0].locked);
    // The first keystroke replaces the live value and locks the box.
    assert!(s.type_into_entry("1"));
    assert!(s.type_into_entry("0"));
    assert_eq!(s.entry_focus, Some(0));
    assert_eq!(s.entries[0].text, "10");
    assert!(s.entries[0].locked);
    // Only the locked size is pinned; the height still follows the pointer.
    s.pointer_moved(&click_at(-3.0, 7.0), &camera, window, false);
    assert_eq!(s.cursor, Some(Vec2::new(-10.0, 7.0)));
    assert_eq!(s.entries[0].text, "10");
    assert_eq!(s.entries[1].text, "7.00");
    assert!(s.focus_next_entry());
    assert_eq!(
        s.entry_focus,
        Some(1),
        "Tab goes to the box still following"
    );
    s.entries[1].text = "4".into();
    s.lock_entry(1);
    assert_eq!(
        s.cursor,
        Some(Vec2::new(-10.0, 4.0)),
        "preview pinned to the sizes"
    );
    s.unlock_entry(1);
    assert_eq!(
        s.entries[1].text, "7.00",
        "released, it follows the pointer again"
    );
    s.entries[1].text = "4".into();
    s.lock_entry(1);
    s.submit_entry();
    assert!(s.take_dirty());
    for p in [
        Vec2::ZERO,
        Vec2::new(-10.0, 0.0),
        Vec2::new(-10.0, 4.0),
        Vec2::new(0.0, 4.0),
    ] {
        point_at(s, p);
    }
    let mut lengths: Vec<f64> = s
        .sketch
        .constraints()
        .filter_map(|(_, c)| match c {
            Constraint::Distance { value, .. } => Some(*value),
            _ => None,
        })
        .collect();
    lengths.sort_by(f64::total_cmp);
    assert_eq!(lengths, vec![4.0, 10.0]);
    assert!(
        s.entries.iter().all(|e| e.text.is_empty()),
        "the next shape starts fresh"
    );

    // A circle by diameter: the pointer only picks the direction of the second click.
    s.set_tool(SketchTool::Circle);
    s.pointer_up(&click_at(30.0, 0.0), &camera, window, true, false);
    s.pointer_moved(&click_at(31.0, 0.0), &camera, window, false);
    s.entries[0].text = "8".into();
    s.lock_entry(0);
    s.submit_entry();
    let radius = s
        .sketch
        .entities()
        .find_map(|(_, e)| match e.entity {
            Entity::Circle { radius, .. } => Some(radius),
            _ => None,
        })
        .unwrap();
    assert!((radius - 4.0).abs() < 1e-9, "{radius}");
    assert!(s.sketch.constraints().any(
        |(_, c)| matches!(c, Constraint::Diameter { value, .. } if (*value - 8.0).abs() < 1e-9)
    ));
}

/// A slot takes a third click for its width, so the width is drawn or typed like the
/// other sizes instead of living in the palette.
#[test]
fn slot_width_comes_from_a_third_click() {
    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    let camera = editor.camera;
    let window = editor.window_px;
    let Mode::Sketch(s) = &mut editor.mode else {
        unreachable!()
    };
    // The coordinates below are chosen for the geometry, not the grid.
    s.snap_to_grid = false;
    s.set_tool(SketchTool::Slot);
    for p in [Vec2::ZERO, Vec2::new(10.0, 0.0), Vec2::new(5.0, 2.0)] {
        s.pointer_up(&click_at(p.x, p.y), &camera, window, true, false);
    }
    let radii: Vec<f64> = s
        .sketch
        .entities()
        .filter_map(|(_, e)| match e.entity {
            Entity::Arc { center, start, .. } => Some(
                s.sketch
                    .point_pos(start)?
                    .distance(s.sketch.point_pos(center)?),
            ),
            _ => None,
        })
        .collect();
    assert_eq!(radii.len(), 2);
    assert!(radii.iter().all(|r| (r - 2.0).abs() < 1e-9), "{radii:?}");
}

/// Create Sketch, then click a body's face: the sketch opens on that face, with its
/// frame sitting on the face.
#[test]
fn sketch_tool_starts_on_a_picked_face() {
    use super::selection::{self, Pick};

    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    let body = block(&mut editor);
    tools::start_tool(&mut editor, ToolKind::Sketch);
    // Clicking from above lands on the top cap.
    let pick = selection::pick(&editor, &click_at(5.0, 5.0), &editor.pick_filter(), 8.0);
    assert!(
        matches!(pick, Some(Pick::Face(f, _)) if f == top_face(body)),
        "{pick:?}"
    );
    editor.apply_pick(pick, false);
    assert!(editor.is_sketching(), "the face pick started the sketch");
    let Mode::Sketch(s) = &editor.mode else {
        unreachable!()
    };
    assert!(
        (s.frame.origin.z - 2.0).abs() < 1e-9,
        "{:?}",
        s.frame.origin
    );
    assert!((s.frame.z.z.abs() - 1.0).abs() < 1e-9);
    assert!(matches!(
        editor.doc.timeline().get(s.feature).map(|f| &f.kind),
        Some(FeatureKind::Sketch { plane: PlaneRef::Face(f), .. }) if *f == top_face(body)
    ));
}

/// Fillet: picking a face takes every edge around it, picking it again lets them all
/// go, and the edges offered while the preview shows are those of the body before the
/// fillet, so a second pick near the first edge is a real edge, not the preview's.
#[test]
fn fillet_picks_faces_as_edge_rings_and_edges_of_the_unfilleted_body() {
    use super::selection::{self, Pick};

    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    let body = block(&mut editor);
    tools::start_tool(&mut editor, ToolKind::Fillet);
    let top = top_face(body);
    let ring = tools::edges_of_face(&editor, &top);
    assert_eq!(ring.len(), 4);
    editor.apply_pick(Some(Pick::Face(top, 0.0)), false);
    assert_eq!(editor.selection.edges.len(), 4);
    assert!(
        editor.selection.faces.is_empty(),
        "the face itself is not kept"
    );
    let feature = editor
        .tool
        .as_ref()
        .unwrap()
        .feature
        .expect("preview built");
    assert!(
        !matches!(
            editor.doc.state().status(feature),
            Some(basset_core::FeatureStatus::Failed(_))
        ),
        "all four edges of the top exist on the input body"
    );
    editor.apply_pick(Some(Pick::Face(top, 0.0)), false);
    assert!(editor.selection.edges.is_empty(), "the whole ring came out");

    // One edge selected: the preview rounds it off. Aiming at where that edge was
    // must still find it (on the unfilleted body), not a seam of the rounded surface.
    let edge_pick = selection::pick(&editor, &click_at(5.0, 0.0), &editor.pick_filter(), 8.0);
    let Some(Pick::Edge(first, _)) = edge_pick else {
        panic!("{edge_pick:?}")
    };
    editor.apply_pick(edge_pick.clone(), false);
    editor.refresh_cache();
    let again = selection::pick(&editor, &click_at(5.0, 0.0), &editor.pick_filter(), 8.0);
    assert!(
        matches!(again, Some(Pick::Edge(e, _)) if e == first),
        "{again:?}"
    );
    let other = selection::pick(&editor, &click_at(10.0, 5.0), &editor.pick_filter(), 8.0);
    assert!(matches!(other, Some(Pick::Edge(..))), "{other:?}");
    editor.apply_pick(other, false);
    assert_eq!(editor.selection.edges.len(), 2);
    let feature = editor.tool.as_ref().unwrap().feature.unwrap();
    assert!(!matches!(
        editor.doc.state().status(feature),
        Some(basset_core::FeatureStatus::Failed(_))
    ));
    // The radius handle sits a radius off the first edge's middle.
    let h = tools::handle(&editor).expect("fillet handle");
    let radius = editor.tool.as_ref().unwrap().params.radius;
    assert!((h.tip.distance(h.origin) - radius).abs() < 1e-9);
}

/// The extrude handle grows from the region along its normal and reaches the distance;
/// the arrow is what the user drags instead of the dialog's number.
#[test]
fn extrude_handle_follows_the_distance() {
    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    draw_rectangle(&mut editor, Vec2::new(0.0, 0.0), Vec2::new(10.0, 10.0));
    sketch_mode::finish(&mut editor, true);
    let sketch = editor.doc.timeline().features()[0].id;
    tools::start_tool(&mut editor, ToolKind::Extrude);
    assert!(tools::handle(&editor).is_none(), "nothing to size yet");
    editor.selection.profiles.push(ProfileRef {
        sketch,
        sample: Vec2::new(5.0, 5.0),
    });
    editor.tool.as_mut().unwrap().params.distance = 7.0;
    tools::sync_tool(&mut editor);
    let h = tools::handle(&editor).expect("extrude handle");
    assert!(
        h.origin.distance(Vec3::new(5.0, 5.0, 0.0)) < 1e-9,
        "{:?}",
        h.origin
    );
    assert!(
        h.tip.distance(Vec3::new(5.0, 5.0, 7.0)) < 1e-9,
        "{:?}",
        h.tip
    );
    assert_eq!(h.dir, Vec3::Z);
    // A sketch region floating in space touches no body: a new body.
    assert_eq!(
        editor.tool.as_ref().unwrap().params.op,
        super::tools::OpKind::NewBody
    );
}

/// In the select tool, empty space inside a closed region picks the curves around it,
/// and pressing on a curve and moving drags the whole curve under its constraints.
#[test]
fn select_tool_picks_regions_and_drags_curves() {
    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    draw_rectangle(&mut editor, Vec2::new(0.0, 0.0), Vec2::new(20.0, 20.0));
    let camera = editor.camera;
    let window = editor.window_px;
    let Mode::Sketch(s) = &mut editor.mode else {
        unreachable!()
    };
    s.fixed_grid_step = Some(1.0);
    s.set_tool(SketchTool::Select);
    s.pointer_moved(&click_at(10.0, 10.0), &camera, window, false);
    assert!(s.hover.is_none());
    assert!(
        s.hover_region.is_some(),
        "the region lights up under the pointer"
    );
    s.pointer_down(&click_at(10.0, 10.0), &camera, window, false);
    s.pointer_up(&click_at(10.0, 10.0), &camera, window, true, false);
    assert_eq!(s.selected.len(), 4, "the four edges of the rectangle");
    assert!(
        s.selected
            .iter()
            .all(|id| s.sketch.entity(*id).unwrap().entity.is_line())
    );
    assert!(!s.selection_is_construction());
    s.toggle_construction();
    assert!(s.selection_is_construction());
    s.toggle_construction();

    // Click outside to drop the selection, then drag the bottom edge up by 3: its two
    // corners move and the rectangle shortens.
    s.pointer_down(&click_at(40.0, 40.0), &camera, window, false);
    s.pointer_up(&click_at(40.0, 40.0), &camera, window, true, false);
    assert!(s.selected.is_empty());
    s.pointer_down(&click_at(10.0, 0.0), &camera, window, false);
    s.pointer_moved(&click_at(10.0, 3.0), &camera, window, true);
    s.pointer_up(&click_at(10.0, 3.0), &camera, window, false, false);
    assert!(s.take_dirty());
    for p in [
        Vec2::new(0.0, 3.0),
        Vec2::new(20.0, 3.0),
        Vec2::new(20.0, 20.0),
        Vec2::new(0.0, 20.0),
    ] {
        point_at(s, p);
    }
    // The whole selection moves together when the press lands on part of it.
    s.pointer_down(&click_at(10.0, 11.5), &camera, window, false);
    s.pointer_up(&click_at(10.0, 11.5), &camera, window, true, false);
    assert_eq!(s.selected.len(), 4);
    s.pointer_down(&click_at(10.0, 3.0), &camera, window, false);
    s.pointer_moved(&click_at(15.0, 3.0), &camera, window, true);
    s.pointer_up(&click_at(15.0, 3.0), &camera, window, false, false);
    point_at(s, Vec2::new(5.0, 3.0));
    point_at(s, Vec2::new(25.0, 20.0));
    // A press and release on a curve without travel only selects; undo has nothing
    // extra to take back beyond the two moves.
    s.pointer_down(&click_at(15.0, 3.0), &camera, window, false);
    s.pointer_up(&click_at(15.0, 3.0), &camera, window, true, false);
    assert_eq!(s.selected.len(), 1);
    assert!(s.undo());
    point_at(s, Vec2::new(0.0, 3.0));
    assert!(s.undo());
    point_at(s, Vec2::new(0.0, 0.0));
}

/// The slot variants all make the same stadium; they differ in what the clicks mean.
#[test]
fn slot_variants_place_the_same_stadium() {
    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    let camera = editor.camera;
    let window = editor.window_px;
    let Mode::Sketch(s) = &mut editor.mode else {
        unreachable!()
    };
    s.snap_to_grid = false;
    let arc_centers = |s: &super::SketchEditor| -> Vec<Vec2> {
        let mut c: Vec<Vec2> = s
            .sketch
            .entities()
            .filter_map(|(_, e)| match e.entity {
                Entity::Arc { center, .. } => s.sketch.point_pos(center),
                _ => None,
            })
            .collect();
        c.sort_by(|a, b| a.x.total_cmp(&b.x));
        c
    };
    // Overall: ends at 0 and 20, width 4, so the arc centres sit 2 in from each end.
    s.set_tool(SketchTool::SlotOverall);
    for p in [Vec2::ZERO, Vec2::new(20.0, 0.0), Vec2::new(10.0, 2.0)] {
        s.pointer_up(&click_at(p.x, p.y), &camera, window, true, false);
    }
    assert_eq!(
        arc_centers(s),
        vec![Vec2::new(2.0, 0.0), Vec2::new(18.0, 0.0)]
    );
    assert!(s.undo());
    // Centre point: middle at 10, one arc centre at 18, so the other is at 2.
    s.set_tool(SketchTool::SlotCenterPoint);
    for p in [
        Vec2::new(10.0, 0.0),
        Vec2::new(18.0, 0.0),
        Vec2::new(10.0, 2.0),
    ] {
        s.pointer_up(&click_at(p.x, p.y), &camera, window, true, false);
    }
    assert_eq!(
        arc_centers(s),
        vec![Vec2::new(2.0, 0.0), Vec2::new(18.0, 0.0)]
    );
    // The middle is a construction point kept at the midpoint of the centre line.
    assert!(s.sketch.entities().any(|(_, e)| {
        e.construction && matches!(e.entity, Entity::Point { pos } if pos.distance(Vec2::new(10.0, 0.0)) < 1e-9)
    }));
    // The toolbar remembers which slot was used last.
    assert_eq!(
        s.variant_of(sketch_mode::ToolGroup::Slot),
        SketchTool::SlotCenterPoint
    );
    assert_eq!(
        s.variant_of(sketch_mode::ToolGroup::Circle),
        SketchTool::Circle
    );
}

/// Dimensions are drawn as in a drawing, with lines that follow the value text when
/// it is dragged, and the placement is kept in the sketch.
#[test]
fn dimensions_are_drawn_and_placed_by_their_label() {
    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    draw_rectangle(&mut editor, Vec2::ZERO, Vec2::new(10.0, 10.0));
    let camera = editor.camera;
    let window = editor.window_px;
    let Mode::Sketch(s) = &mut editor.mode else {
        unreachable!()
    };
    s.snap_to_grid = false;
    let (_, c) = dimension(
        s,
        &camera,
        window,
        Vec2::new(5.0, 0.0),
        Vec2::new(30.0, -30.0),
    );
    assert!(matches!(c, basset_sketch::Constraint::Distance { .. }));
    let g = s.dimension_graphics();
    assert_eq!(g.len(), 1);
    assert_eq!(g[0].text, "10.000");
    // Two extension lines, the dimension line and two arrowheads of two strokes each.
    assert_eq!(g[0].segments.len(), 7);
    let before = g[0].label;
    s.move_label(g[0].id, &click_at(5.0, -6.0));
    assert!(s.take_dirty());
    assert_eq!(
        s.sketch.dimension_label(g[0].id),
        Some(Vec2::new(5.0, -6.0))
    );
    let g = s.dimension_graphics();
    assert!(g[0].label.distance(before) > 1.0);
    assert!(g[0].label.distance(Vec3::new(5.0, -6.0, 0.0)) < 1e-9);
    // The dimension line moved with the label: it now runs along y = -6.
    assert!(
        g[0].segments.iter().any(|[a, b]| (a.y + 6.0).abs() < 1e-9
            && (b.y + 6.0).abs() < 1e-9
            && a.distance(*b) > 9.0),
        "{:?}",
        g[0].segments
    );
    // The lines of the overlay are part of what the sketch draws.
    let mut lines = Vec::new();
    let mut points = Vec::new();
    s.draw(&mut lines, &mut points, &mut Vec::new());
    assert!(lines.iter().any(|l| l.segments.len() == 7));
}

#[test]
fn the_trim_tool_removes_the_piece_under_the_pointer() {
    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    draw_rectangle(&mut editor, Vec2::new(0.0, 0.0), Vec2::new(20.0, 10.0));
    draw_line(&mut editor, Vec2::new(10.0, -5.0), Vec2::new(10.0, 15.0));
    assert_eq!(
        sketch(&mut editor)
            .sketch
            .profiles(&Default::default())
            .len(),
        2
    );

    // Hovering the overhang above the rectangle shows what the click would take.
    let camera = editor.camera;
    let window = editor.window_px;
    let s = sketch(&mut editor);
    s.set_tool(SketchTool::Trim);
    s.pointer_moved(&click_at(10.0, 13.0), &camera, window, false);
    s.draw(&mut Vec::new(), &mut Vec::new(), &mut Vec::new());

    click_with(&mut editor, SketchTool::Trim, Vec2::new(10.0, 13.0));
    click_with(&mut editor, SketchTool::Trim, Vec2::new(10.0, -3.0));
    let s = sketch(&mut editor);
    let profiles = s.sketch.profiles(&Default::default());
    assert_eq!(profiles.len(), 2, "the divider still splits the rectangle");
    for p in &profiles {
        assert!((p.area() - 100.0).abs() < 1e-6, "{}", p.area());
    }
    // Nothing is left sticking out past the rectangle.
    let highest = s
        .sketch
        .entities()
        .filter_map(|(id, _)| s.sketch.entity_bounds(id))
        .fold(f64::NEG_INFINITY, |m, (_, max)| m.max(max.y));
    assert!(highest <= 10.0 + 1e-9, "{highest}");
}

#[test]
fn the_break_tool_cuts_a_curve_without_removing_it() {
    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    draw_rectangle(&mut editor, Vec2::new(0.0, 0.0), Vec2::new(20.0, 10.0));
    draw_line(&mut editor, Vec2::new(10.0, -5.0), Vec2::new(10.0, 15.0));
    let before = sketch(&mut editor)
        .sketch
        .entities()
        .filter(|(_, d)| d.entity.is_curve())
        .count();
    click_with(&mut editor, SketchTool::Break, Vec2::new(10.0, 13.0));
    let s = sketch(&mut editor);
    let after = s
        .sketch
        .entities()
        .filter(|(_, d)| d.entity.is_curve())
        .count();
    assert_eq!(after, before + 2, "the divider became three pieces");
    assert_eq!(s.sketch.profiles(&Default::default()).len(), 2);
}

#[test]
fn m_moves_the_selection_by_typed_offsets() {
    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    draw_rectangle(&mut editor, Vec2::new(0.0, 0.0), Vec2::new(20.0, 10.0));
    let s = sketch(&mut editor);
    s.selected = s
        .sketch
        .entities()
        .filter(|(_, d)| d.entity.is_curve())
        .map(|(id, _)| id)
        .collect();
    assert!(s.begin_move());
    let op = s.move_op.as_mut().expect("move started");
    op.dx = 5.0;
    op.dy = -2.0;
    s.update_move();
    s.finish_move(true);
    assert!(!s.move_in_progress());
    let (min, _) = s
        .sketch
        .entities()
        .filter_map(|(id, _)| s.sketch.entity_bounds(id))
        .fold(
            (Vec2::splat(f64::INFINITY), Vec2::splat(f64::NEG_INFINITY)),
            |(lo, hi), (a, b)| (lo.min(a), hi.max(b)),
        );
    assert!((min - Vec2::new(5.0, -2.0)).length() < 1e-6, "{min:?}");
    // Cancelling a move puts the geometry back where it was.
    assert!(s.begin_move());
    s.move_op.as_mut().unwrap().dx = 100.0;
    s.update_move();
    s.finish_move(false);
    let after = s
        .sketch
        .entities()
        .filter_map(|(id, _)| s.sketch.entity_bounds(id))
        .fold(Vec2::splat(f64::INFINITY), |lo, (a, _)| lo.min(a));
    assert!((after - Vec2::new(5.0, -2.0)).length() < 1e-6, "{after:?}");
}

#[test]
fn e_extrudes_the_region_under_the_pointer() {
    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    draw_rectangle(&mut editor, Vec2::new(0.0, 0.0), Vec2::new(20.0, 10.0));
    let camera = editor.camera;
    let window = editor.window_px;
    {
        let s = sketch(&mut editor);
        s.set_tool(SketchTool::Select);
        s.pointer_moved(&click_at(10.0, 5.0), &camera, window, false);
        assert!(
            s.has_region_selection(),
            "the region under the pointer counts"
        );
    }
    sketch_mode::extrude_region(&mut editor);
    assert!(!editor.is_sketching(), "the sketch is kept and closed");
    assert_eq!(
        editor.tool.as_ref().expect("extrude opened").kind,
        ToolKind::Extrude
    );
    assert_eq!(editor.selection.profiles.len(), 1);
    let feature = editor
        .tool
        .as_ref()
        .unwrap()
        .feature
        .expect("the region was enough to build a preview");
    editor.tool.as_mut().unwrap().params.distance = 3.0;
    tools::sync_tool(&mut editor);
    tools::confirm_tool(&mut editor);
    let volume = editor
        .doc
        .state()
        .body(BodyRef(feature))
        .expect("body")
        .solid
        .volume();
    assert!((volume - 600.0).abs() < 1e-6, "{volume}");
}

#[test]
fn clicking_inside_a_region_picks_it_for_extrude() {
    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    draw_rectangle(&mut editor, Vec2::new(0.0, 0.0), Vec2::new(20.0, 10.0));
    let camera = editor.camera;
    let window = editor.window_px;
    let s = sketch(&mut editor);
    s.set_tool(SketchTool::Select);
    s.pointer_moved(&click_at(10.0, 5.0), &camera, window, false);
    s.pointer_up(&click_at(10.0, 5.0), &camera, window, true, false);
    assert_eq!(s.selected.len(), 4, "the region's curves are selected too");
    assert_eq!(s.selected_regions.len(), 1);
    // Moving the pointer away does not lose the pick.
    s.pointer_moved(&click_at(40.0, 40.0), &camera, window, false);
    assert_eq!(s.region_samples().len(), 1);
    // Clicking empty space outside every region drops it.
    s.pointer_up(&click_at(40.0, 40.0), &camera, window, true, false);
    assert!(s.region_samples().is_empty());
}

#[test]
fn a_pattern_repeats_the_selected_geometry() {
    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    // The grid snap rounds the corner, so the seed's own area is what the copies are
    // compared against rather than a number written here.
    draw_rectangle(&mut editor, Vec2::new(0.0, 0.0), Vec2::new(5.0, 5.0));
    let s = sketch(&mut editor);
    let seed_area = s.sketch.profiles(&Default::default())[0].area();
    s.selected = s
        .sketch
        .entities()
        .filter(|(_, d)| d.entity.is_curve())
        .map(|(id, _)| id)
        .collect();
    s.pattern.circular = false;
    s.pattern.spacing = sketch_mode::Spacing::Between;
    s.pattern.count_x = 3;
    s.pattern.count_y = 2;
    s.pattern.distance_x = 10.0;
    s.pattern.distance_y = 10.0;
    assert!(s.begin_pattern(), "the selection can be repeated");
    // The copies are there before anything is confirmed: the tool previews in the live
    // sketch, which is how the numbers can be judged while they are being set.
    let previewed = s.sketch.profiles(&Default::default());
    assert_eq!(previewed.len(), 6, "3 x 2 including the seed");
    let created = s.finish_pattern(true).expect("the pattern was kept");
    assert!(created > 0);
    let profiles = s.sketch.profiles(&Default::default());
    assert_eq!(profiles.len(), 6);
    for p in &profiles {
        assert!((p.area() - seed_area).abs() < 1e-6, "{}", p.area());
    }
}

/// Changing a number replaces the copies instead of adding a second pattern on top of
/// the first, and cancelling leaves the sketch exactly as it was.
#[test]
fn a_pattern_previews_live_and_can_be_cancelled() {
    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    draw_rectangle(&mut editor, Vec2::new(0.0, 0.0), Vec2::new(5.0, 5.0));
    let s = sketch(&mut editor);
    let before = s.sketch.entities().count();
    s.selected = s
        .sketch
        .entities()
        .filter(|(_, d)| d.entity.is_curve())
        .map(|(id, _)| id)
        .collect();
    s.pattern.circular = false;
    s.pattern.count_x = 2;
    s.pattern.count_y = 1;
    s.pattern.distance_x = 10.0;
    assert!(s.begin_pattern());
    assert_eq!(s.sketch.profiles(&Default::default()).len(), 2);

    s.pattern.count_x = 4;
    s.update_pattern();
    assert_eq!(
        s.sketch.profiles(&Default::default()).len(),
        4,
        "the copies were re-made, not added to"
    );

    // Stated as a total, the same distance spans the four copies instead of separating
    // each pair, so the last one lands on 10 rather than at 30.
    s.pattern.spacing = sketch_mode::Spacing::Total;
    s.update_pattern();
    let right = s
        .sketch
        .entities()
        .filter_map(|(id, _)| s.sketch.entity_bounds(id))
        .fold(f64::NEG_INFINITY, |acc, (_, max)| acc.max(max.x));
    assert!(
        (right - 15.0).abs() < 1e-6,
        "seed 0..5 plus a 10 mm span: {right}"
    );

    assert_eq!(s.finish_pattern(false), None, "cancelled");
    assert_eq!(
        s.sketch.entities().count(),
        before,
        "cancelling puts the sketch back"
    );
}

#[test]
fn a_named_parameter_drives_a_dimension_through_the_editor() {
    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    draw_rectangle(&mut editor, Vec2::new(0.0, 0.0), Vec2::new(20.0, 10.0));
    let s = sketch(&mut editor);
    // Dimension the bottom edge, then drive it by an expression.
    let bottom = s
        .sketch
        .entities()
        .find(|(id, d)| {
            d.entity.is_line()
                && s.sketch
                    .entity_bounds(*id)
                    .is_some_and(|(min, max)| min.y == 0.0 && max.y == 0.0)
        })
        .map(|(id, _)| id)
        .expect("bottom edge");
    let Entity::Line { start, end } = s.sketch.entity(bottom).unwrap().entity else {
        unreachable!()
    };
    s.add_constraint(basset_sketch::Constraint::Distance {
        a: start,
        b: end,
        value: 20.0,
    })
    .unwrap();
    let dim = s.sketch.constraints().last().map(|(id, _)| id).unwrap();
    s.set_parameter("width", "30").unwrap();
    s.bind_dimension(dim, "width / 2").unwrap();
    let length = s
        .sketch
        .point_pos(start)
        .unwrap()
        .distance(s.sketch.point_pos(end).unwrap());
    assert!((length - 15.0).abs() < 1e-6, "{length}");
    // The panel's drafts follow the sketch, and a bad expression is refused.
    s.sync_param_drafts();
    assert_eq!(s.param_drafts, vec![("width".into(), "30".into())]);
    assert!(s.set_parameter("width", "nope * 2").is_err());
    assert_eq!(s.sketch.parameter_value("width").unwrap(), 30.0);
}

/// Clicking a curve while drawing attaches the point to it, rather than leaving a free
/// point wherever the grid put it.
///
/// A divider drawn to an edge used to only *look* attached: `snap` considered existing
/// points and nothing else, so the endpoint was a grid-snapped free point that happened
/// to sit on the edge. Re-solving after any dimension change moved it off, which opened
/// the regions either side of it and silently changed what the extrudes built.
#[test]
fn drawing_onto_a_curve_constrains_the_point_to_it() {
    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    draw_rectangle(&mut editor, Vec2::new(0.0, 0.0), Vec2::new(20.0, 10.0));
    // Both ends land on an edge, away from any corner.
    draw_line(&mut editor, Vec2::new(10.0, 0.0), Vec2::new(10.0, 10.0));
    let s = sketch(&mut editor);

    let onto_curves = s
        .sketch
        .constraints()
        .filter(|(_, c)| match c {
            Constraint::Coincident { target, .. } => s
                .sketch
                .entity(*target)
                .is_some_and(|e| e.entity.is_curve()),
            _ => false,
        })
        .count();
    assert_eq!(onto_curves, 2, "both ends are held onto the edge they meet");
    assert_eq!(s.sketch.profiles(&Default::default()).len(), 2);

    // Moving the top edge away is what used to break it: the divider must follow.
    let top: Vec<EntityId> = s
        .sketch
        .entities()
        .filter(|(id, d)| d.entity.is_point() && s.sketch.point_pos(*id).is_some_and(|p| p.y > 5.0))
        .map(|(id, _)| id)
        .collect();
    let goals: Vec<(EntityId, Vec2)> = top
        .iter()
        .map(|id| (*id, s.sketch.point_pos(*id).unwrap() + Vec2::new(0.0, 10.0)))
        .collect();
    s.sketch.drag_points(&goals).expect("drag");

    let profiles = s.sketch.profiles(&Default::default());
    assert_eq!(profiles.len(), 2, "the divider still splits the rectangle");
    for p in &profiles {
        assert!(
            p.area() > 50.0,
            "each half grew with the rectangle: {}",
            p.area()
        );
    }
}

/// Geometric constraints are drawn on the geometry, not left invisible.
///
/// Only the six dimension kinds used to produce a graphic; Horizontal, Vertical,
/// Coincident, Parallel and the rest returned nothing, so a sketch showed no sign of
/// what was holding it together and a constraint could never be picked to remove.
#[test]
fn geometric_constraints_are_drawn_on_the_geometry() {
    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    draw_rectangle(&mut editor, Vec2::new(0.0, 0.0), Vec2::new(20.0, 10.0));
    let s = sketch(&mut editor);

    // A rectangle is built with horizontal and vertical constraints on its four sides.
    let glyphs = s.constraint_glyphs();
    assert_eq!(glyphs.len(), 4, "one badge per constraint");
    for g in &glyphs {
        assert!(!g.segments.is_empty(), "a badge has strokes to draw");
        assert!(
            s.sketch.constraint(g.id).is_some(),
            "each badge names a live constraint"
        );
    }

    // Every badge sits off the geometry rather than on top of it, and within reach of it.
    let bounds = 40.0;
    for g in &glyphs {
        let p = s.frame.to_local(g.center);
        assert!(
            p.x.abs() < bounds && p.y.abs() < bounds,
            "badge is beside its entity, not flung off: {p:?}"
        );
    }
}

/// A sketch that cannot solve says which constraints disagree, and marks them on the
/// drawing. "solver did not converge (residual 2.7e+00)" is not something a user can act
/// on; "these two dimensions of the same edge disagree" is.
#[test]
fn a_conflicting_sketch_names_and_marks_the_constraints_at_fault() {
    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    draw_rectangle(&mut editor, Vec2::new(0.0, 0.0), Vec2::new(20.0, 10.0));
    let s = sketch(&mut editor);
    let bottom = s
        .sketch
        .entities()
        .find(|(id, d)| {
            matches!(d.entity, Entity::Line { .. })
                && s.sketch
                    .entity_bounds(*id)
                    .is_some_and(|(min, max)| min.y == 0.0 && max.y == 0.0)
        })
        .map(|(id, _)| id)
        .expect("bottom edge");
    let Entity::Line { start, end } = s.sketch.entity(bottom).unwrap().entity else {
        unreachable!()
    };
    for value in [20.0, 30.0] {
        s.add_constraint(basset_sketch::Constraint::Distance {
            a: start,
            b: end,
            value,
        })
        .unwrap();
    }

    let blamed = s.conflicting().to_vec();
    assert_eq!(
        blamed.len(),
        2,
        "both dimensions of that edge, and nothing else"
    );
    for id in &blamed {
        let c = s.sketch.constraint(*id).expect("a live constraint");
        assert!(
            matches!(c, basset_sketch::Constraint::Distance { .. }),
            "the rectangle's own horizontal and vertical constraints are satisfiable"
        );
    }

    // The badges of the offenders are drawn in their own batch, so they read as red.
    let mut lines = Vec::new();
    let mut points = Vec::new();
    s.draw(&mut lines, &mut points, &mut Vec::new());
    let red = lines
        .iter()
        .find(|b| b.color == sketch_mode::CONFLICT_COLOR)
        .expect("a batch for what is in conflict");
    assert!(!red.segments.is_empty(), "the marks are actually drawn");
}

/// Dimensions keep drawing their own value and leader; they must not gain a second badge.
#[test]
fn dimensions_are_not_given_constraint_badges() {
    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    draw_line(&mut editor, Vec2::new(0.0, 0.0), Vec2::new(10.0, 0.0));
    let before = sketch(&mut editor).constraint_glyphs().len();
    let camera = editor.camera;
    let window = editor.window_px;
    dimension(
        sketch(&mut editor),
        &camera,
        window,
        Vec2::new(5.0, 0.0),
        Vec2::new(5.0, -6.0),
    );
    let s = sketch(&mut editor);
    assert!(
        s.sketch
            .constraints()
            .any(|(_, c)| matches!(c, Constraint::Distance { .. })),
        "the dimension was made"
    );
    assert_eq!(
        s.constraint_glyphs().len(),
        before,
        "the dimension draws itself and gets no badge"
    );
}

/// The viewport colours what the solver leaves free, and a curve counts as free when a
/// point defining it does — otherwise a rectangle with one loose corner would look
/// finished except for a single dot.
#[test]
fn under_constrained_geometry_includes_the_curves_its_points_hold() {
    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    draw_rectangle(&mut editor, Vec2::new(0.0, 0.0), Vec2::new(20.0, 10.0));
    let Mode::Sketch(s) = &mut editor.mode else {
        unreachable!()
    };
    let free = s.under_constrained();
    let curves: Vec<EntityId> = s
        .sketch
        .entities()
        .filter(|(_, e)| !e.entity.is_point())
        .map(|(id, _)| id)
        .collect();
    assert_eq!(curves.len(), 4);
    for id in &curves {
        assert!(
            free.contains(id),
            "every line of a loose rectangle is loose"
        );
    }

    // Pin it down completely: nothing is drawn blue any more.
    let corners: Vec<EntityId> = s
        .sketch
        .entities()
        .filter(|(_, e)| e.entity.is_point())
        .map(|(id, _)| id)
        .collect();
    for id in corners {
        s.add_constraint(Constraint::Fix(id)).unwrap();
    }
    assert!(s.under_constrained().is_empty());
}

// ----- sketch mode: regions, construction and constraints --------------------------------

/// Pointing at a closed region fills it. An outline alone says "these curves"; the thing
/// the user is about to extrude is the area, so the area is what lights up.
#[test]
fn the_enclosed_area_under_the_pointer_is_filled() {
    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    draw_rectangle(&mut editor, Vec2::new(0.0, 0.0), Vec2::new(20.0, 20.0));
    let camera = editor.camera;
    let window = editor.window_px;
    let s = sketch(&mut editor);
    s.set_tool(SketchTool::Select);

    let filled = |s: &sketch_mode::SketchEditor| {
        let mut tris = Vec::new();
        s.draw(&mut Vec::new(), &mut Vec::new(), &mut tris);
        tris.iter().map(|t| t.triangles.len()).sum::<usize>()
    };

    // Outside the rectangle nothing is filled.
    s.pointer_moved(&click_at(40.0, 40.0), &camera, window, false);
    assert_eq!(filled(s), 0);

    s.pointer_moved(&click_at(10.0, 10.0), &camera, window, false);
    assert!(filled(s) > 0, "the region under the pointer is filled");

    // Clicking it keeps the fill, because the region stays selected once picked.
    s.pointer_up(&click_at(10.0, 10.0), &camera, window, true, false);
    s.pointer_moved(&click_at(40.0, 40.0), &camera, window, false);
    assert!(filled(s) > 0, "a picked region stays filled");
}

/// Construction is decided before the shape is drawn, not converted afterwards: the mode
/// is what a centre line or a bolt circle actually needs.
#[test]
fn construction_mode_draws_reference_geometry() {
    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    {
        let s = sketch(&mut editor);
        assert!(!s.construction);
        // With nothing selected the command arms the mode rather than converting.
        s.toggle_construction();
        assert!(s.construction);
    }
    draw_rectangle(&mut editor, Vec2::new(0.0, 0.0), Vec2::new(10.0, 10.0));
    let s = sketch(&mut editor);
    let curves: Vec<_> = s
        .sketch
        .entities()
        .filter(|(_, d)| d.entity.is_curve())
        .collect();
    assert!(!curves.is_empty());
    assert!(
        curves.iter().all(|(_, d)| d.construction),
        "every curve of the rectangle is reference geometry"
    );
    assert!(
        s.sketch.profiles(&Default::default()).is_empty(),
        "construction geometry encloses nothing"
    );

    // Turning it off again leaves the next shape as real geometry.
    s.toggle_construction();
    assert!(!s.construction);
    draw_rectangle(&mut editor, Vec2::new(20.0, 0.0), Vec2::new(30.0, 10.0));
    assert_eq!(
        sketch(&mut editor)
            .sketch
            .profiles(&Default::default())
            .len(),
        1
    );
}

/// A constraint is a tool: pick it first and the geometry after, the way Fusion works.
#[test]
fn a_constraint_tool_applies_to_the_picks_that_follow_it() {
    use sketch_mode::ConstraintKind;

    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    draw_line(&mut editor, Vec2::new(0.0, 0.0), Vec2::new(20.0, 3.0));
    let camera = editor.camera;
    let window = editor.window_px;
    let s = sketch(&mut editor);
    s.select_tool();
    s.select_only(Vec::new());

    // Armed with nothing picked, it waits rather than guessing.
    let before = s.sketch.constraints().count();
    s.begin_constraint(ConstraintKind::Horizontal).unwrap();
    assert_eq!(s.armed_constraint(), Some(ConstraintKind::Horizontal));
    assert_eq!(s.sketch.constraints().count(), before);

    // One pick on the line is all Horizontal needs, so it goes on at once.
    s.pointer_up(&click_at(10.0, 1.5), &camera, window, true, false);
    assert_eq!(s.sketch.constraints().count(), before + 1);
    assert!(
        s.selected.is_empty(),
        "the picks are cleared, ready for the next line"
    );
    assert_eq!(
        s.armed_constraint(),
        Some(ConstraintKind::Horizontal),
        "the tool stays armed"
    );
    let (_, line) = s
        .sketch
        .entities()
        .find(|(_, d)| d.entity.is_line())
        .unwrap();
    let Entity::Line { start, end } = line.entity else {
        unreachable!()
    };
    let (a, b) = (
        s.sketch.point_pos(start).unwrap(),
        s.sketch.point_pos(end).unwrap(),
    );
    assert!((a.y - b.y).abs() < 1e-6, "the line was levelled: {a} {b}");
}

/// Order never matters, and what a constraint means for a set of picks is the same
/// question the toolbar asks to decide whether it can be applied at all.
#[test]
fn a_constraint_reads_its_picks_in_either_order() {
    use sketch_mode::ConstraintKind;

    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    draw_rectangle(&mut editor, Vec2::new(0.0, 0.0), Vec2::new(20.0, 10.0));
    let s = sketch(&mut editor);
    let line = s
        .sketch
        .entities()
        .find(|(_, d)| d.entity.is_line())
        .map(|(id, _)| id)
        .unwrap();
    let loose = s.sketch.add_point(Vec2::new(5.0, 7.0));

    for picks in [vec![loose, line], vec![line, loose]] {
        let made = s.constraints_for(ConstraintKind::Midpoint, &picks);
        assert!(
            matches!(made[..], [Constraint::Midpoint { point, line: l }] if point == loose && l == line),
            "midpoint reads the point and the line whichever was picked first: {made:?}"
        );
    }
    // A line has no midpoint constraint with another line, so the toolbar says so.
    assert!(
        s.constraints_for(ConstraintKind::Midpoint, &[line, line])
            .is_empty()
    );
}

/// Equal across a run of curves is one command, which is what "these five holes are the
/// same size" should cost.
#[test]
fn a_transitive_constraint_chains_through_every_pick() {
    use sketch_mode::ConstraintKind;

    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    draw_rectangle(&mut editor, Vec2::new(0.0, 0.0), Vec2::new(20.0, 10.0));
    let s = sketch(&mut editor);
    let lines: Vec<EntityId> = s
        .sketch
        .entities()
        .filter(|(_, d)| d.entity.is_line())
        .map(|(id, _)| id)
        .collect();
    assert_eq!(lines.len(), 4);
    let made = s.constraints_for(ConstraintKind::Equal, &lines);
    assert_eq!(made.len(), 3, "four lines tie together with three equals");
    assert!(made.iter().all(|c| matches!(c, Constraint::Equal(..))));
}

/// The manipulator drives the same numbers the palette shows, so a drag and a typed
/// offset are two ways of saying one thing.
#[test]
fn the_move_manipulator_drives_the_move_numbers() {
    use super::gizmo;

    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    draw_rectangle(&mut editor, Vec2::new(0.0, 0.0), Vec2::new(10.0, 10.0));
    let s = sketch(&mut editor);
    s.selected = s
        .sketch
        .entities()
        .filter(|(_, d)| d.entity.is_curve())
        .map(|(id, _)| id)
        .collect();
    assert!(gizmo::current(&editor).is_none(), "nothing is being moved");

    sketch(&mut editor).begin_move();
    let g = gizmo::current(&editor).expect("the move has a manipulator");
    assert_eq!(g.arrows.len(), 2, "a sketch moves on its plane, not off it");
    assert_eq!(g.rings.len(), 1, "and turns about its normal alone");
    assert_eq!(g.origin, Vec3::new(5.0, 5.0, 0.0), "on what is being moved");

    let s = sketch(&mut editor);
    // Off the grid, so this measures the manipulator rather than the snap.
    s.snap_to_grid = false;
    // A drag on an arrow and a drag on the ring land in the boxes the palette shows.
    assert!(s.nudge_move(true, 4.0));
    assert!(s.turn_move(15.0));
    {
        let op = s.move_op.as_ref().expect("still moving");
        assert_eq!((op.dx, op.dy, op.angle_deg), (4.0, 0.0, 15.0));
    }
    // The rectangle's own horizontal and vertical constraints refuse the turn, which is
    // the point of offering the move as a goal rather than applying it outright. The
    // slide along the arrow is free, so that is what survives.
    s.turn_move(-15.0);
    s.update_move();
    s.finish_move(true);
    let (min, max) = s
        .sketch
        .entities()
        .filter_map(|(id, _)| s.sketch.entity_bounds(id))
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), (a, b)| {
            (lo.min(a.x), hi.max(b.x))
        });
    assert!(
        (min - 4.0).abs() < 1e-6 && (max - 14.0).abs() < 1e-6,
        "the square slid 4 mm across: {min}..{max}"
    );
}

/// The manipulator has to sit on the thing it moves. Getting this wrong is invisible in
/// a unit that only checks the numbers and glaring the moment anyone drags an arrow, so
/// it is asserted against the body's real position rather than reasoned about.
#[test]
fn the_body_manipulator_sits_on_the_body_it_moves() {
    use super::gizmo;

    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    // A 10x10x2 block from the origin, so its centre is a number we can write down.
    block(&mut editor);
    let body = *editor.doc.state().bodies.keys().next().unwrap();
    tools::start_tool(&mut editor, ToolKind::Move);
    editor.selection.bodies.push(body);
    tools::sync_tool(&mut editor);
    editor.refresh_cache();

    let centre = |editor: &mut Editor| {
        let aabb = editor.doc.state().body(body).unwrap().solid.aabb();
        (aabb.min + aabb.max) * 0.5
    };
    let before = centre(&mut editor);
    let g = gizmo::current(&editor).expect("the Move tool has a manipulator");
    assert_eq!(g.origin, before, "it starts on the body");
    assert_eq!(g.arrows.len(), 3);
    assert_eq!(g.rings.len(), 3);

    editor.tool.as_mut().unwrap().params.translate = Vec3::new(7.0, 0.0, 0.0);
    tools::sync_tool(&mut editor);
    editor.refresh_cache();
    let after = centre(&mut editor);
    assert_eq!(after, before + Vec3::new(7.0, 0.0, 0.0), "the body moved");
    let g = gizmo::current(&editor).expect("still moving");
    assert_eq!(
        g.origin, after,
        "and the manipulator moved with it, exactly once"
    );
}

/// Right-drag is the orbit gesture. Orbiting to look at a pattern must not be the
/// gesture that throws it away, which is what sharing `cancel_current` with the
/// end-a-line-chain path used to mean.
#[test]
fn orbiting_does_not_destroy_a_pattern_in_progress() {
    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    draw_rectangle(&mut editor, Vec2::new(0.0, 0.0), Vec2::new(5.0, 5.0));
    let s = sketch(&mut editor);
    s.selected = s
        .sketch
        .entities()
        .filter(|(_, d)| d.entity.is_curve())
        .map(|(id, _)| id)
        .collect();
    s.pattern.circular = false;
    s.pattern.count_x = 3;
    s.pattern.count_y = 1;
    assert!(s.begin_pattern());
    let copies = s.sketch.profiles(&Default::default()).len();
    assert_eq!(copies, 3);

    // The right-button release that ends an orbit, and the one that ends a line chain.
    s.finish_current();
    assert!(s.pattern_in_progress(), "the pattern survived the gesture");
    assert_eq!(s.sketch.profiles(&Default::default()).len(), copies);
}

/// Undo, while a move is being set up, means the move: putting it back is exactly what
/// the user is asking to undo, and neither modal operation has a checkpoint of its own
/// to undo past safely.
#[test]
fn undo_during_a_move_puts_the_move_back() {
    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    draw_rectangle(&mut editor, Vec2::new(0.0, 0.0), Vec2::new(5.0, 5.0));
    draw_rectangle(&mut editor, Vec2::new(10.0, 0.0), Vec2::new(15.0, 5.0));
    let s = sketch(&mut editor);
    let entities = s.sketch.entities().count();
    s.selected = s
        .sketch
        .entities()
        .filter(|(_, d)| d.entity.is_curve())
        .map(|(id, _)| id)
        .collect();
    assert!(s.begin_move());
    s.snap_to_grid = false;
    s.nudge_move(true, 4.0);
    s.update_move();

    editor.undo();
    assert_eq!(editor.status, "Move cancelled");
    let s = sketch(&mut editor);
    assert!(!s.move_in_progress());
    assert_eq!(
        s.sketch.entities().count(),
        entities,
        "and it undid the move, not the shape drawn before it"
    );
    let left = s
        .sketch
        .entities()
        .filter_map(|(id, _)| s.sketch.entity_bounds(id))
        .fold(f64::INFINITY, |acc, (min, _)| acc.min(min.x));
    assert!(
        left.abs() < 1e-9,
        "the geometry went back to where it was: {left}"
    );
}

/// A move that was applied is one step of undo, which is what was missing: the move took
/// its checkpoint on the way in and popped it again on the way out, so afterwards there
/// was nothing on the stack to go back to.
#[test]
fn an_applied_sketch_move_can_be_undone() {
    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    draw_rectangle(&mut editor, Vec2::new(0.0, 0.0), Vec2::new(5.0, 5.0));
    let s = sketch(&mut editor);
    s.selected = s
        .sketch
        .entities()
        .filter(|(_, d)| d.entity.is_curve())
        .map(|(id, _)| id)
        .collect();
    assert!(s.begin_move());
    s.snap_to_grid = false;
    s.nudge_move(true, 7.0);
    s.update_move();
    s.finish_move(true);

    let left = |editor: &mut Editor| {
        let s = sketch(editor);
        s.sketch
            .entities()
            .filter_map(|(id, _)| s.sketch.entity_bounds(id))
            .fold(f64::INFINITY, |acc, (min, _)| acc.min(min.x))
    };
    assert!((left(&mut editor) - 7.0).abs() < 1e-6, "the move applied");
    editor.undo();
    assert!(left(&mut editor).abs() < 1e-9, "and one undo puts it back");
}

/// Escape cancels the pattern itself and the revert reaches the document, so nothing
/// downstream keeps showing copies the user took back.
#[test]
fn escape_cancels_a_pattern_and_the_document_follows() {
    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    draw_rectangle(&mut editor, Vec2::new(0.0, 0.0), Vec2::new(5.0, 5.0));
    let feature = sketch(&mut editor).feature;
    let s = sketch(&mut editor);
    s.selected = s
        .sketch
        .entities()
        .filter(|(_, d)| d.entity.is_curve())
        .map(|(id, _)| id)
        .collect();
    s.pattern.circular = false;
    s.pattern.count_x = 3;
    s.pattern.count_y = 1;
    assert!(s.begin_pattern());
    editor.commit_sketch();
    editor.refresh_cache();
    assert_eq!(
        editor.doc.state().sketches[&feature].profiles.len(),
        3,
        "the preview is in the document, which is why cancelling has to reach it"
    );

    editor.cancel();
    editor.refresh_cache();
    assert!(!sketch(&mut editor).pattern_in_progress());
    assert_eq!(
        editor.doc.state().sketches[&feature].profiles.len(),
        1,
        "the document went back to the seed alone"
    );
}

/// A pick that could never become the armed constraint is refused and named, rather than
/// joining a pile the tool has already given up on.
#[test]
fn a_constraint_tool_refuses_a_pick_it_could_never_use() {
    use sketch_mode::ConstraintKind;

    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    draw_rectangle(&mut editor, Vec2::new(0.0, 0.0), Vec2::new(20.0, 10.0));
    let camera = editor.camera;
    let window = editor.window_px;
    let s = sketch(&mut editor);
    s.select_only(Vec::new());
    s.begin_constraint(ConstraintKind::Perpendicular).unwrap();

    // The corner is a point, and no set of picks containing one is ever perpendicular.
    s.pointer_up(&click_at(0.0, 0.0), &camera, window, true, false);
    assert!(s.constraint_picks().is_empty(), "the pick was refused");
    assert!(
        s.take_constraint_error()
            .is_some_and(|e| e.contains("Perpendicular")),
        "and the user is told what it wants"
    );

    // A line is a pick it can use, and it waits for the second.
    s.pointer_up(&click_at(10.0, 0.0), &camera, window, true, false);
    assert_eq!(s.constraint_picks().len(), 1);
}

/// A transitive constraint chains on past the pair: the pick it just used stays as the
/// start of the next one, so five equal holes are five clicks.
#[test]
fn a_transitive_constraint_tool_chains_click_by_click() {
    use sketch_mode::ConstraintKind;

    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    draw_rectangle(&mut editor, Vec2::new(0.0, 0.0), Vec2::new(20.0, 10.0));
    let camera = editor.camera;
    let window = editor.window_px;
    let s = sketch(&mut editor);
    s.select_only(Vec::new());
    let before = s.sketch.constraints().count();
    s.begin_constraint(ConstraintKind::Equal).unwrap();

    // Bottom, right, top: each click after the first ties onto the one before it.
    s.pointer_up(&click_at(10.0, 0.0), &camera, window, true, false);
    assert_eq!(s.sketch.constraints().count(), before);
    s.pointer_up(&click_at(20.0, 5.0), &camera, window, true, false);
    assert_eq!(s.sketch.constraints().count(), before + 1);
    assert_eq!(s.constraint_picks().len(), 1, "the last pick carries over");
    s.pointer_up(&click_at(10.0, 10.0), &camera, window, true, false);
    assert_eq!(
        s.sketch.constraints().count(),
        before + 2,
        "the third click chains rather than starting a fresh pair"
    );
}

/// One construction command that reads its context: with geometry selected it converts
/// that geometry and leaves the mode alone, and with nothing selected it arms the mode.
#[test]
fn the_construction_command_reads_what_is_selected() {
    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    draw_rectangle(&mut editor, Vec2::new(0.0, 0.0), Vec2::new(10.0, 10.0));
    let s = sketch(&mut editor);
    s.construction = true;
    s.selected = vec![
        s.sketch
            .entities()
            .find(|(_, d)| d.entity.is_curve())
            .map(|(id, _)| id)
            .unwrap(),
    ];
    // With a selection it converts, and leaves the mode where it was.
    s.toggle_construction();
    assert!(s.selection_is_construction());
    assert!(s.construction, "the mode is untouched by a conversion");
    s.toggle_construction();
    assert!(!s.selection_is_construction(), "and it converts back");

    // With nothing selected the same command is the mode.
    s.selected.clear();
    s.toggle_construction();
    assert!(!s.construction);
    s.toggle_construction();
    assert!(s.construction);
}

/// Rotating geometry the constraints hold to the axes used to fold it flat: the solver
/// does not fail, it finds some *other* arrangement satisfying horizontal and vertical,
/// and the cheapest one is the rectangle collapsed to a line. That is a converged solve
/// and a destroyed drawing, so the move is judged by the shape rather than the residual.
#[test]
fn a_rotation_the_constraints_refuse_leaves_the_geometry_alone() {
    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    // The rectangle builder constrains its edges horizontal and vertical, so no rotation
    // of it is possible at all.
    draw_rectangle(&mut editor, Vec2::new(0.0, 0.0), Vec2::new(10.0, 6.0));
    let s = sketch(&mut editor);
    let area = |s: &sketch_mode::SketchEditor| {
        s.sketch
            .profiles(&Default::default())
            .first()
            .map(|p| p.area())
            .unwrap_or(0.0)
    };
    let before = area(s);
    assert!(before > 0.0);
    s.selected = s
        .sketch
        .entities()
        .filter(|(_, d)| d.entity.is_curve())
        .map(|(id, _)| id)
        .collect();
    assert!(s.begin_move());

    s.turn_move(30.0);
    s.update_move();
    assert!(
        s.move_refused().is_some(),
        "the move says it could not be done"
    );
    assert!(
        (area(s) - before).abs() < 1e-9,
        "and the rectangle is untouched: {} was {before}",
        area(s)
    );

    // Taking the rotation back off leaves an ordinary slide, which the constraints allow.
    s.turn_move(-30.0);
    s.nudge_move(true, 5.0);
    s.update_move();
    assert!(s.move_refused().is_none(), "a slide is fine");
    assert!((area(s) - before).abs() < 1e-9, "and keeps its shape");
}

/// A sketch with nothing holding it to the axes turns as asked, so the refusal above is
/// about the constraints and not about rotation being unsupported.
#[test]
fn a_free_shape_rotates_in_the_sketch_plane() {
    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    let s = sketch(&mut editor);
    s.snap_to_grid = false;
    // A bare triangle of points: no constraints at all, so it is free to turn.
    let a = s.sketch.add_point(Vec2::new(0.0, 0.0));
    let b = s.sketch.add_point(Vec2::new(10.0, 0.0));
    let c = s.sketch.add_point(Vec2::new(0.0, 4.0));
    s.selected = vec![a, b, c];
    assert!(s.begin_move());
    s.turn_move(90.0);
    s.update_move();
    assert!(s.move_refused().is_none(), "nothing objects");

    // A quarter turn about the centroid takes (10, 0) to where the rotation says, and
    // every distance is kept, which is what makes it a rotation rather than a reshape.
    let pos = |id| s.sketch.point_pos(id).unwrap();
    assert!(
        (pos(a).distance(pos(b)) - 10.0).abs() < 1e-6
            && (pos(a).distance(pos(c)) - 4.0).abs() < 1e-6,
        "distances kept"
    );
    // The turn is about the centroid of what is being moved, so (10, 0) lands where a
    // quarter turn about that point puts it.
    let pivot = Vec2::new(10.0 / 3.0, 4.0 / 3.0);
    let was = Vec2::new(10.0, 0.0);
    let expected = pivot + Vec2::from_angle(std::f64::consts::FRAC_PI_2).rotate(was - pivot);
    assert!(pos(b).distance(expected) < 1e-6, "{} vs {expected}", pos(b));
}

/// Placing a circular pattern's centre is a mode you ask for, not something every click
/// does: while a pattern is up the pointer is otherwise idle, and a stray click that
/// silently moved the centre of a pattern already placed would be worse than no picking.
#[test]
fn a_circular_pattern_centre_is_picked_in_a_mode() {
    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    draw_rectangle(&mut editor, Vec2::new(20.0, 0.0), Vec2::new(25.0, 5.0));
    let camera = editor.camera;
    let window = editor.window_px;
    let s = sketch(&mut editor);
    s.selected = s
        .sketch
        .entities()
        .filter(|(_, d)| d.entity.is_curve())
        .map(|(id, _)| id)
        .collect();
    s.pattern.circular = true;
    s.pattern.count = 4;
    s.pattern.angle_deg = 360.0;
    assert!(s.begin_pattern());
    let centred = s.pattern.center;

    // A click while the mode is off leaves the centre exactly where it was.
    s.pointer_up(&click_at(0.0, 0.0), &camera, window, true, false);
    assert_eq!(s.pattern.center, centred, "an idle click changes nothing");

    s.pick_pattern_center(true);
    assert!(s.picking_pattern_center());
    s.pointer_up(&click_at(0.0, 0.0), &camera, window, true, false);
    assert_eq!(s.pattern.center, Vec2::ZERO, "the click placed the centre");
    assert!(
        !s.picking_pattern_center(),
        "and the mode ends with the pick, so the next click is idle again"
    );

    // The copies are laid out about the new centre, each a quarter turn on, and each
    // still a square: a copy that inherited the seed's horizontal would have collapsed.
    assert_eq!(s.sketch.profiles(&Default::default()).len(), 4);
    let left = s
        .sketch
        .entities()
        .filter_map(|(id, _)| s.sketch.entity_bounds(id))
        .fold(f64::INFINITY, |acc, (min, _)| acc.min(min.x));
    assert!(left < -20.0, "the pattern went round the origin: {left}");
}

/// A box drag takes what the filter allows and nothing else, so "select every curve in
/// this corner" and "select every point in it" are two different gestures rather than
/// one gesture and some tidying up afterwards.
#[test]
fn the_sketch_filter_decides_what_a_box_drag_takes() {
    use sketch_mode::SketchPick;

    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    draw_rectangle(&mut editor, Vec2::new(0.0, 0.0), Vec2::new(10.0, 5.0));
    let camera = editor.camera;
    let window = editor.window_px;
    let s = sketch(&mut editor);
    s.set_tool(SketchTool::Select);

    // A box enclosing the whole rectangle, dragged rightwards so it encloses rather than
    // crosses.
    let sweep = |s: &mut sketch_mode::SketchEditor| {
        s.pointer_down(&click_at(-5.0, -5.0), &camera, window, false);
        s.pointer_moved(&click_at(15.0, 10.0), &camera, window, true);
        s.pointer_up(&click_at(15.0, 10.0), &camera, window, false, false);
    };
    let kinds = |s: &sketch_mode::SketchEditor| {
        let points = s
            .selected
            .iter()
            .filter(|id| s.sketch.entity(**id).is_some_and(|d| d.entity.is_point()))
            .count();
        (points, s.selected.len() - points)
    };

    s.set_pick(SketchPick::All);
    sweep(s);
    assert_eq!(kinds(s), (4, 4), "everything: four corners and four edges");

    s.set_pick(SketchPick::Curves);
    sweep(s);
    assert_eq!(kinds(s), (0, 4), "curves alone");

    s.set_pick(SketchPick::Points);
    sweep(s);
    assert_eq!(kinds(s), (4, 0), "points alone");

    // Regions takes the enclosed area, which is what an extrude is built from.
    s.set_pick(SketchPick::Regions);
    sweep(s);
    assert_eq!(kinds(s), (0, 0), "no curves or points");
    assert_eq!(s.selected_regions.len(), 1, "the area inside the rectangle");
}

/// Narrowing the filter lets go of what it no longer covers: being left holding things
/// the mode gives no way to see or deselect is worse than having no filter at all.
#[test]
fn narrowing_the_sketch_filter_drops_what_it_no_longer_covers() {
    use sketch_mode::SketchPick;

    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    draw_rectangle(&mut editor, Vec2::new(0.0, 0.0), Vec2::new(10.0, 5.0));
    let s = sketch(&mut editor);
    s.selected = s.sketch.entities().map(|(id, _)| id).collect();
    assert!(s.selected.len() > 4);

    s.set_pick(SketchPick::Points);
    assert!(
        s.selected
            .iter()
            .all(|id| s.sketch.entity(*id).is_some_and(|d| d.entity.is_point())),
        "only the points are still held"
    );
    assert!(!s.selected.is_empty());
}

/// The filter is about choosing between things already drawn, so it must not get in the
/// way of drawing: a line still snaps to a point while the filter says Curves.
#[test]
fn the_sketch_filter_does_not_change_what_drawing_snaps_to() {
    use sketch_mode::SketchPick;

    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    draw_rectangle(&mut editor, Vec2::new(0.0, 0.0), Vec2::new(10.0, 5.0));
    let camera = editor.camera;
    let window = editor.window_px;
    let s = sketch(&mut editor);
    s.set_pick(SketchPick::Curves);
    s.set_tool(SketchTool::Line);
    let before = s
        .sketch
        .entities()
        .filter(|(_, d)| d.entity.is_point())
        .count();

    // Starting a line on the rectangle's corner reuses that point rather than making a
    // second one on top of it.
    s.pointer_moved(&click_at(10.0, 5.0), &camera, window, false);
    assert!(s.cursor_snapped, "the corner is still a snap target");
    s.pointer_up(&click_at(10.0, 5.0), &camera, window, true, false);
    s.pointer_up(&click_at(20.0, 5.0), &camera, window, true, false);
    let after = s
        .sketch
        .entities()
        .filter(|(_, d)| d.entity.is_point())
        .count();
    assert_eq!(after, before + 1, "only the far end is a new point");
}

/// A drawing built on a grid stays on it. A dragged arrow that left geometry at 49.87 mm
/// would quietly undo the point of drawing on a grid at all, so the manipulator snaps
/// like every other position in the sketch, and the rotation ring snaps to whole steps.
#[test]
fn a_dragged_move_snaps_to_the_grid() {
    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    draw_rectangle(&mut editor, Vec2::new(0.0, 0.0), Vec2::new(10.0, 10.0));
    let s = sketch(&mut editor);
    s.selected = s
        .sketch
        .entities()
        .filter(|(_, d)| d.entity.is_curve())
        .map(|(id, _)| id)
        .collect();
    s.snap_to_grid = true;
    s.fixed_grid_step = Some(5.0);
    s.grid_step = 5.0;
    assert!(s.begin_move());

    // A drag that lands between grid lines is taken to the nearest one.
    s.nudge_move(true, 11.3);
    assert_eq!(s.move_op.as_ref().unwrap().dx, 10.0);
    // And a second drag carries on from there rather than re-rounding a running total.
    s.nudge_move(true, 2.6);
    assert_eq!(s.move_op.as_ref().unwrap().dx, 15.0);

    s.turn_move(37.4);
    assert_eq!(
        s.move_op.as_ref().unwrap().angle_deg,
        35.0,
        "and the ring lands on a whole step"
    );

    // Off the grid the drag is taken exactly as given.
    s.snap_to_grid = false;
    s.nudge_move(false, 1.23);
    assert_eq!(s.move_op.as_ref().unwrap().dy, 1.23);
}

/// A move is driven by its boxes and its manipulator, so the sketch underneath it is not
/// up for grabs: a drag made there is thrown away by the next change of a number, and
/// leaves a checkpoint on the undo stack pointing at a state that follows from nothing.
#[test]
fn geometry_cannot_be_dragged_out_from_under_a_move() {
    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    draw_rectangle(&mut editor, Vec2::new(0.0, 0.0), Vec2::new(10.0, 10.0));
    let camera = editor.camera;
    let window = editor.window_px;
    let s = sketch(&mut editor);
    s.set_tool(SketchTool::Select);
    s.selected = s
        .sketch
        .entities()
        .filter(|(_, d)| d.entity.is_curve())
        .map(|(id, _)| id)
        .collect();
    assert!(s.begin_move());
    let corner = point_at(s, Vec2::new(10.0, 10.0));

    // Press on a corner and drag it well away.
    s.pointer_down(&click_at(10.0, 10.0), &camera, window, false);
    s.pointer_moved(&click_at(30.0, 30.0), &camera, window, true);
    s.pointer_up(&click_at(30.0, 30.0), &camera, window, false, false);
    assert_eq!(
        s.sketch.point_pos(corner),
        Some(Vec2::new(10.0, 10.0)),
        "the corner stayed where it was"
    );
    assert!(s.move_in_progress(), "and the move is still running");
}

/// The move is typed in the same boxes a shape's sizes are: start it, type the number,
/// press Enter. Clicking into a field first is a step the drawing does not need.
#[test]
fn a_move_can_be_typed_straight_into_its_boxes() {
    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    draw_rectangle(&mut editor, Vec2::new(0.0, 0.0), Vec2::new(10.0, 10.0));
    let s = sketch(&mut editor);
    s.selected = s
        .sketch
        .entities()
        .filter(|(_, d)| d.entity.is_curve())
        .map(|(id, _)| id)
        .collect();
    assert!(s.begin_move());
    assert_eq!(
        s.entries.iter().map(|e| e.dim).collect::<Vec<_>>(),
        vec![
            sketch_mode::Dim::Dx,
            sketch_mode::Dim::Dy,
            sketch_mode::Dim::Angle
        ],
        "the boxes are there as soon as the move starts"
    );
    assert_eq!(s.entry_focus, Some(0), "and the first one has the keyboard");

    // "50" with the pointer in the viewport goes straight into dX.
    assert!(s.type_into_entry("5"));
    assert!(s.type_into_entry("0"));
    assert_eq!(s.move_op.as_ref().unwrap().dx, 50.0);

    // Enter applies it, as it places a shape from its sizes.
    s.submit_entry();
    assert!(!s.move_in_progress());
    let left = s
        .sketch
        .entities()
        .filter_map(|(id, _)| s.sketch.entity_bounds(id))
        .fold(f64::INFINITY, |acc, (min, _)| acc.min(min.x));
    assert!((left - 50.0).abs() < 1e-6, "moved exactly 50 mm: {left}");
}

/// Overall bounds of everything drawn, for judging which side an offset went and how far.
fn drawn_bounds(s: &sketch_mode::SketchEditor) -> (Vec2, Vec2) {
    s.sketch
        .entities()
        .filter_map(|(id, _)| s.sketch.entity_bounds(id))
        .fold(
            (Vec2::splat(f64::INFINITY), Vec2::splat(f64::NEG_INFINITY)),
            |(lo, hi), (min, max)| (lo.min(min), hi.max(max)),
        )
}

fn select_all_curves(s: &mut sketch_mode::SketchEditor) {
    s.selected = s
        .sketch
        .entities()
        .filter(|(_, d)| d.entity.is_curve())
        .map(|(id, _)| id)
        .collect();
}

/// The offset previews in the live sketch like a pattern does, re-makes itself when the
/// distance changes rather than offsetting its own offset, and leaves nothing behind
/// when it is cancelled.
#[test]
fn an_offset_previews_live_and_can_be_cancelled() {
    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    draw_rectangle(&mut editor, Vec2::new(0.0, 0.0), Vec2::new(40.0, 20.0));
    let s = sketch(&mut editor);
    let before = s.sketch.entities().count();
    select_all_curves(s);
    s.offset.distance = 5.0;
    s.offset.corner = sketch_mode::Corner::Round;
    assert!(s.begin_offset(), "a closed loop can be offset");

    let (min, max) = drawn_bounds(s);
    assert!(
        (min.x + 5.0).abs() < 1e-6 && (max.x - 45.0).abs() < 1e-6,
        "{min:?} {max:?}"
    );
    assert!(
        (min.y + 5.0).abs() < 1e-6 && (max.y - 25.0).abs() < 1e-6,
        "{min:?} {max:?}"
    );

    // Changing the distance replaces the result instead of offsetting it again.
    s.offset.distance = 10.0;
    s.update_offset();
    let (min, max) = drawn_bounds(s);
    assert!(
        (min.x + 10.0).abs() < 1e-6 && (max.x - 50.0).abs() < 1e-6,
        "{min:?} {max:?}"
    );

    assert_eq!(s.finish_offset(false), None, "cancelled");
    assert_eq!(
        s.sketch.entities().count(),
        before,
        "and the sketch is exactly as it was"
    );
}

/// The two corner styles are the two different drawings the user is choosing between:
/// rounded keeps every *point* the distance away, squared keeps every *edge* that far.
#[test]
fn the_corner_style_changes_what_comes_out() {
    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    draw_rectangle(&mut editor, Vec2::new(0.0, 0.0), Vec2::new(40.0, 20.0));
    let s = sketch(&mut editor);
    // Everything, corner points and all: a box drag selects the points too, and they are
    // not something to make the user deselect before the tool will speak to them.
    s.selected = s.sketch.entities().map(|(id, _)| id).collect();
    s.offset.distance = 5.0;
    s.offset.corner = sketch_mode::Corner::Round;
    assert!(s.begin_offset());
    let (made, error) = s.offset_status().expect("an offset is running");
    assert_eq!(error, None);
    assert_eq!(made, 8, "four edges and four corner arcs");

    s.offset.corner = sketch_mode::Corner::Miter;
    s.update_offset();
    let (made, _) = s.offset_status().expect("still running");
    assert_eq!(
        made, 4,
        "squared corners add nothing, the edges run out to meet"
    );
    // Both reach the same corner of the drawing; only rounded gets there on an arc.
    let (_, max) = drawn_bounds(s);
    assert!(
        (max.x - 45.0).abs() < 1e-6 && (max.y - 25.0).abs() < 1e-6,
        "{max:?}"
    );
}

/// An offset larger than the shape has nowhere to go. It says so and leaves the drawing
/// alone, rather than making a knot of crossed lines that looks like geometry.
#[test]
fn an_offset_that_will_not_fit_says_so_and_changes_nothing() {
    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    draw_rectangle(&mut editor, Vec2::new(0.0, 0.0), Vec2::new(40.0, 20.0));
    let s = sketch(&mut editor);
    let before = s.sketch.entities().count();
    select_all_curves(s);
    // The rectangle is 20 tall; 15 in from both sides leaves nothing between them.
    s.offset.distance = -15.0;
    assert!(s.begin_offset());
    let (made, error) = s.offset_status().expect("an offset is running");
    assert_eq!(made, 0);
    assert!(
        error.is_some_and(|e| e.contains("larger than the geometry can carry")),
        "{error:?}"
    );
    assert_eq!(s.sketch.entities().count(), before);
    // Keeping a failed offset keeps nothing, and leaves no checkpoint behind either.
    assert_eq!(s.finish_offset(true), None);
    assert_eq!(s.sketch.entities().count(), before);
    assert!(s.undo());
    assert!(
        s.sketch.entities().count() < before,
        "undo went back past the rectangle, so the failed offset left no step of its own"
    );
}

/// The offset is dragged on the result itself, so the distance is set by looking at the
/// drawing rather than at a field in a panel. The handle sits at `anchor + dir *
/// distance`, which is on the result, and a drag of it *is* the distance.
#[test]
fn the_offset_is_dragged_by_a_handle_on_the_result() {
    use super::gizmo;

    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    draw_rectangle(&mut editor, Vec2::new(0.0, 0.0), Vec2::new(40.0, 20.0));
    let s = sketch(&mut editor);
    select_all_curves(s);
    s.offset.distance = 5.0;
    assert!(gizmo::slider(&editor).is_none(), "nothing has a handle yet");

    assert!(sketch(&mut editor).begin_offset());
    let slider = gizmo::slider(&editor).expect("the offset has a handle");
    // The handle sits on the bottom edge of the result, 5 mm below the drawing.
    assert_eq!(slider.grip(), Vec3::new(20.0, -5.0, 0.0));

    let s = sketch(&mut editor);
    // Off the grid, so this measures the drag rather than the snap.
    s.snap_to_grid = false;
    assert!(s.nudge_offset(3.0));
    s.update_offset();
    assert_eq!(s.offset.distance, 8.0);
    assert_eq!(
        gizmo::slider(&editor).expect("still there").grip(),
        Vec3::new(20.0, -8.0, 0.0)
    );
}

/// Dragging the handle back across the drawing and out the far side takes the distance
/// through zero and negative, which is how the side is chosen. There is no flip to press:
/// the offset is on the side the pointer is.
#[test]
fn dragging_the_offset_across_the_drawing_puts_it_on_the_other_side() {
    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    draw_rectangle(&mut editor, Vec2::new(0.0, 0.0), Vec2::new(40.0, 20.0));
    let s = sketch(&mut editor);
    select_all_curves(s);
    s.snap_to_grid = false;
    s.offset.distance = 5.0;
    assert!(s.begin_offset());
    let (min, _) = drawn_bounds(s);
    assert!((min.y + 5.0).abs() < 1e-6, "outside, 5 mm clear: {min:?}");

    // All the way back through the rectangle and 5 mm out the other side.
    assert!(s.nudge_offset(-10.0));
    s.update_offset();
    assert_eq!(s.offset.distance, -5.0);
    let (min, max) = drawn_bounds(s);
    assert!(
        (min.y).abs() < 1e-6 && (max.y - 20.0).abs() < 1e-6,
        "the offset is inside the rectangle now: {min:?} {max:?}"
    );
}

/// Keeping an offset is one step of undo, not one per curve it drew.
#[test]
fn an_offset_is_one_step_of_undo() {
    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    draw_rectangle(&mut editor, Vec2::new(0.0, 0.0), Vec2::new(40.0, 20.0));
    let s = sketch(&mut editor);
    let before = s.sketch.entities().count();
    select_all_curves(s);
    s.offset.distance = 5.0;
    assert!(s.begin_offset());
    assert_eq!(s.finish_offset(true), Some(8));
    assert!(s.sketch.entities().count() > before);
    assert!(s.undo());
    assert_eq!(
        s.sketch.entities().count(),
        before,
        "one undo took all of it"
    );
}

/// Only one modal operation runs at a time. A second begun on top of the first would
/// take the first one's preview as the sketch to re-derive from, so cancelling it would
/// keep what the first one had provisionally made.
#[test]
fn a_modal_operation_will_not_start_on_top_of_another() {
    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    draw_rectangle(&mut editor, Vec2::new(0.0, 0.0), Vec2::new(40.0, 20.0));
    let s = sketch(&mut editor);
    let before = s.sketch.entities().count();
    select_all_curves(s);
    s.offset.distance = 5.0;
    assert!(s.begin_offset());
    assert!(
        !s.begin_pattern(),
        "a pattern will not start over an offset"
    );
    assert!(!s.begin_move(), "nor will a move");
    assert_eq!(s.modal_name(), Some("Offset"));
    s.finish_modal(false);
    assert_eq!(s.sketch.entities().count(), before);
}

/// A tool that refuses because something else is running says so. Telling someone to
/// select geometry first, when they have selected it and are halfway through a move,
/// says nothing about what to do next.
#[test]
fn a_tool_refused_by_a_running_operation_says_which_one() {
    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    draw_rectangle(&mut editor, Vec2::new(0.0, 0.0), Vec2::new(40.0, 20.0));
    let s = sketch(&mut editor);
    select_all_curves(s);
    assert!(s.begin_move());
    editor.on_key(&winit::keyboard::Key::Character("o".into()));
    assert!(
        editor.status.contains("Move is still up"),
        "{}",
        editor.status
    );
}

/// Keeping an offset that could make nothing is a cancel, and the status line says so
/// rather than leaving the button looking unresponsive.
#[test]
fn a_kept_offset_that_made_nothing_says_it_was_cancelled() {
    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    draw_rectangle(&mut editor, Vec2::new(0.0, 0.0), Vec2::new(40.0, 20.0));
    let s = sketch(&mut editor);
    select_all_curves(s);
    s.offset.distance = -15.0;
    assert!(s.begin_offset());
    editor.confirm();
    assert!(
        editor.status.contains("nothing it could make"),
        "{}",
        editor.status
    );
}

/// Shift lets go of the grid for as long as it is held. The toggle in the palette says
/// whether the drawing is built on a grid at all; shift says "not this one placement",
/// which is the far commoner thing to want and should not need a trip to the palette.
#[test]
fn shift_lets_go_of_the_grid_while_it_is_held() {
    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    let s = sketch(&mut editor);
    s.snap_to_grid = true;
    assert!(s.snapping(), "the grid holds by default");
    s.set_free_snap(true);
    assert!(!s.snapping(), "and lets go while shift is down");
    s.set_free_snap(false);
    assert!(s.snapping(), "and takes hold again when it comes up");

    // The toggle still wins: shift releases a grid that is on, it does not turn one on.
    s.snap_to_grid = false;
    s.set_free_snap(false);
    assert!(!s.snapping());
}

/// Every handle that snaps has to answer to shift, or the escape is only half an escape.
#[test]
fn shift_frees_the_manipulators_as_well_as_the_drawing() {
    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    draw_rectangle(&mut editor, Vec2::new(0.0, 0.0), Vec2::new(40.0, 20.0));
    let s = sketch(&mut editor);
    select_all_curves(s);
    s.snap_to_grid = true;
    s.fixed_grid_step = Some(5.0);
    s.grid_step = 5.0;

    // The move's arrows and ring.
    assert!(s.begin_move());
    s.nudge_move(true, 4.0);
    s.turn_move(2.0);
    {
        let op = s.move_op.as_ref().expect("moving");
        assert_eq!(
            (op.dx, op.angle_deg),
            (5.0, 0.0),
            "snapped to the grid and to 5°"
        );
    }
    s.set_free_snap(true);
    s.nudge_move(true, 1.3);
    s.turn_move(2.0);
    {
        let op = s.move_op.as_ref().expect("moving");
        assert!(
            (op.dx - 6.3).abs() < 1e-9 && (op.angle_deg - 2.0).abs() < 1e-9,
            "{op:?}"
        );
    }
    s.finish_move(false);

    // And the offset's handle.
    s.set_free_snap(false);
    s.offset.distance = 0.0;
    assert!(s.begin_offset());
    s.nudge_offset(4.0);
    assert_eq!(s.offset.distance, 5.0, "snapped");
    s.set_free_snap(true);
    s.nudge_offset(1.3);
    assert!(
        (s.offset.distance - 6.3).abs() < 1e-9,
        "{}",
        s.offset.distance
    );
    s.finish_offset(false);
}

/// A rectangle whose four sides are all tied to one of them, which puts four badges on
/// that one side and one on each of the others. Dense enough that a layout which only
/// stacked outwards from the midpoint would overlap, and small enough to reason about.
fn crowded_rectangle(editor: &mut Editor) -> Vec<EntityId> {
    draw_rectangle(editor, Vec2::new(0.0, 0.0), Vec2::new(20.0, 10.0));
    let s = sketch(editor);
    let sides: Vec<EntityId> = s
        .sketch
        .entities()
        .filter(|(_, d)| matches!(d.entity, Entity::Line { .. }))
        .map(|(id, _)| id)
        .collect();
    assert_eq!(sides.len(), 4, "a rectangle has four sides");
    // Equal all round is redundant but consistent, so the solver takes it and the sketch
    // stays a shape rather than folding flat.
    for other in &sides[1..] {
        s.add_constraint(Constraint::Equal(sides[0], *other))
            .expect("equal sides");
    }
    sides
}

/// Distance from a point to a segment, for asserting a badge is clear of a curve.
fn distance_to_segment(p: Vec2, a: Vec2, b: Vec2) -> f64 {
    let d = b - a;
    let len2 = d.length_squared();
    if len2 <= 0.0 {
        return p.distance(a);
    }
    let t = ((p - a).dot(d) / len2).clamp(0.0, 1.0);
    p.distance(a + d * t)
}

/// Badges declutter: they dodge the geometry they annotate and each other, and the ones
/// decluttering pushed away say where they came from with a leader.
///
/// The old layout stacked every badge of one entity straight out from its midpoint. That
/// is tidy for a rectangle drawn on its own and a pile of overlapping marks the moment
/// several constraints land on one line, which is exactly when the user needs to read
/// them.
#[test]
fn constraint_badges_dodge_the_geometry_and_each_other() {
    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    let sides = crowded_rectangle(&mut editor);
    let camera = editor.camera;
    let window = editor.window_px;
    let s = sketch(&mut editor);
    let px = camera.pixel_size_at(s.frame.to_world(Vec2::ZERO), window);
    let glyphs = s.constraint_glyphs();
    assert_eq!(
        glyphs.len(),
        4 + 3 * 2,
        "four axis marks and three equals on two sides"
    );

    let clear = sketch_mode::glyph_clearance_px();
    for (i, a) in glyphs.iter().enumerate() {
        let pa = s.frame.to_local(a.center) / px;
        for b in &glyphs[i + 1..] {
            let pb = s.frame.to_local(b.center) / px;
            assert!(
                (pa.x - pb.x).abs() >= clear - 1e-6 || (pa.y - pb.y).abs() >= clear - 1e-6,
                "two badges sit on top of each other at {pa:?} and {pb:?}"
            );
        }
        for side in &sides {
            let (q0, q1) = s.sketch.curve_endpoints(*side).expect("a line");
            let d = distance_to_segment(pa, q0 / px, q1 / px);
            assert!(
                d >= clear * 0.5 - 1e-6,
                "a badge is sitting on the drawing, {d} px from a side"
            );
        }
    }
    assert!(
        glyphs.iter().any(|g| g.leader.is_some()),
        "the badge that had to leave its entity says which one it belongs to"
    );
    // And the ones that did not move away are not cluttered with leaders.
    assert!(
        glyphs.iter().any(|g| g.leader.is_none()),
        "a badge in its natural slot needs no line"
    );
}

/// The badge layout is cached. The overlay asks for it twice a frame — once to draw the
/// marks, once to put a hit area over each — and a few hundred constraints make laying
/// them out that often a cost the user feels while panning.
#[test]
fn the_badge_layout_is_cached_until_the_sketch_or_the_view_changes() {
    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    crowded_rectangle(&mut editor);
    let camera = editor.camera;
    let window = editor.window_px;
    let s = sketch(&mut editor);
    s.pointer_moved(&click_at(50.0, 50.0), &camera, window, false);
    let _ = s.constraint_glyphs();
    let built = s.glyph_layouts();

    // Redrawing, and moving the pointer over the sketch, costs nothing.
    for i in 0..50 {
        s.pointer_moved(&click_at(30.0 + f64::from(i), 40.0), &camera, window, false);
        let _ = s.constraint_glyphs();
        let _ = s.constraint_glyphs();
    }
    assert_eq!(s.glyph_layouts(), built, "nothing changed, nothing rebuilt");

    // Moving the geometry does rebuild it: a badge whose entity moved is in the wrong
    // place, and no hash of the sketch can miss that.
    let corner = s
        .sketch
        .entities()
        .find(|(_, d)| matches!(d.entity, Entity::Point { .. }))
        .map(|(id, _)| id)
        .expect("a corner");
    let goal = s.sketch.point_pos(corner).expect("a corner") + Vec2::new(3.0, 3.0);
    s.sketch.drag_points(&[(corner, goal)]).expect("drag");
    let _ = s.constraint_glyphs();
    assert_eq!(s.glyph_layouts(), built + 1, "the sketch changed");

    // So does a real change of zoom, because what collides with what is read in pixels.
    let mut zoomed = camera;
    zoomed.zoom(0.5);
    s.pointer_moved(&click_at(30.0, 40.0), &zoomed, window, false);
    let _ = s.constraint_glyphs();
    assert_eq!(s.glyph_layouts(), built + 2, "the view changed");
}

/// Placement is stable: a layout recomputed after the view moved offers every badge the
/// slot it already had, so nothing shuffles under the user while they zoom. Optimal
/// packing matters far less than a mark staying where the eye last found it.
#[test]
fn badge_placement_survives_a_change_of_zoom() {
    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    crowded_rectangle(&mut editor);
    let camera = editor.camera;
    let window = editor.window_px;
    let s = sketch(&mut editor);

    // Every badge's offset from its entity's midpoint, in pixels: the layout's own units,
    // and what the user sees.
    let slots = |s: &super::SketchEditor, camera: &basset_viewport::Camera| {
        let px = camera.pixel_size_at(s.frame.to_world(Vec2::ZERO), window);
        s.constraint_glyphs()
            .iter()
            .map(|g| {
                let id = s.glyph_entity(g).expect("a badge names its entity");
                let (a, b) = s.sketch.curve_endpoints(id).expect("a line");
                let d = (s.frame.to_local(g.center) - (a + b) * 0.5) / px;
                (
                    g.target,
                    (d.x * 1e6).round() as i64,
                    (d.y * 1e6).round() as i64,
                )
            })
            .collect::<Vec<_>>()
    };

    s.pointer_moved(&click_at(50.0, 50.0), &camera, window, false);
    let before = slots(s, &camera);
    let built = s.glyph_layouts();

    let mut zoomed = camera;
    zoomed.zoom(0.5);
    s.pointer_moved(&click_at(50.0, 50.0), &zoomed, window, false);
    let after = slots(s, &zoomed);
    assert!(
        s.glyph_layouts() > built,
        "the zoom really did make the layout reconsider itself"
    );
    assert_eq!(before, after, "and it put every badge back where it was");
}

/// A few hundred constraints stay responsive: the layout is worked out once and every
/// frame after that reads it back.
///
/// Not a precise benchmark — it runs on whatever the machine is doing at the time — but
/// it fails loudly if the per-frame cost goes back to laying out from scratch, which is
/// the regression that matters.
#[test]
fn several_hundred_constraints_are_laid_out_once_and_redrawn_from_the_cache() {
    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    let camera = editor.camera;
    let window = editor.window_px;
    let s = sketch(&mut editor);
    // The view scale first, so the timed call below is the one that lays the badges out.
    s.pointer_moved(&click_at(50.0, 50.0), &camera, window, false);
    let built = s.glyph_layouts();

    // A field of short lines, each held horizontal and tied to the one before it. The
    // sketch is not solved: the badges are placed from the geometry as it stands, which
    // is what the editor draws between solves anyway.
    let mut lines: Vec<EntityId> = Vec::new();
    for i in 0..160 {
        let x = f64::from(i % 16) * 12.0;
        let y = f64::from(i / 16) * 12.0;
        let a = s.sketch.add_point(Vec2::new(x, y));
        let b = s.sketch.add_point(Vec2::new(x + 8.0, y));
        let line = s.sketch.add_line(a, b).expect("a line");
        s.sketch
            .add_constraint(Constraint::Horizontal(line))
            .expect("horizontal");
        if let Some(prev) = lines.last() {
            s.sketch
                .add_constraint(Constraint::Equal(*prev, line))
                .expect("equal");
        }
        lines.push(line);
    }
    assert_eq!(s.sketch.constraints().count(), 160 + 159);

    let start = std::time::Instant::now();
    let glyphs = s.constraint_glyphs();
    let layout = start.elapsed();
    assert_eq!(s.glyph_layouts(), built + 1, "that call did the layout");
    assert_eq!(
        glyphs.len(),
        160 + 159 * 2,
        "one badge per constraint per entity it holds"
    );

    let start = std::time::Instant::now();
    for _ in 0..200 {
        let _ = s.constraint_glyphs();
    }
    let redraw = start.elapsed();
    assert_eq!(
        s.glyph_layouts(),
        built + 1,
        "a hundred frames of overlay, and no second layout"
    );
    // Generous bounds: the point is that neither is seconds, which is where rebuilding
    // the layout twice a frame was heading.
    assert!(
        layout < std::time::Duration::from_secs(2),
        "one layout of {} constraints took {layout:?}",
        160 + 159
    );
    assert!(
        redraw < std::time::Duration::from_secs(4),
        "200 cached redraws took {redraw:?}"
    );
}

/// A dimension's value is placed in the drawing like everything else the user puts
/// there, so dragging it lands on the grid and shift lets go. Dragged freehand, two
/// dimensions of the same feature never line up with each other.
#[test]
fn dragging_a_dimension_value_snaps_it_to_the_grid() {
    let mut editor = Editor::new(None);
    editor.window_px = [800, 600];
    sketch_mode::enter_new(&mut editor, PlaneRef::Origin(OriginPlane::XY));
    draw_line(&mut editor, Vec2::new(0.0, 0.0), Vec2::new(20.0, 0.0));
    let camera = editor.camera;
    let window = editor.window_px;
    let s = sketch(&mut editor);
    let (cid, _) = dimension(
        s,
        &camera,
        window,
        Vec2::new(10.0, 0.0),
        Vec2::new(10.0, -6.0),
    );

    s.pointer_moved(&click_at(7.0, -9.0), &camera, window, false);
    let step = s.grid_step;
    assert!(step > 0.0, "the grid has a step");

    s.move_label(cid, &click_at(7.3, -9.4));
    let placed = s.sketch.dimension_label(cid).expect("placed");
    assert!(
        (placed.x / step).fract().abs() < 1e-9 && (placed.y / step).fract().abs() < 1e-9,
        "the value landed on the grid: {placed:?} with step {step}"
    );

    // Shift is for the one label that has to sit between the lines.
    s.set_free_snap(true);
    s.move_label(cid, &click_at(7.3, -9.4));
    let free = s.sketch.dimension_label(cid).expect("placed");
    assert!(
        (free.x - 7.3).abs() < 1e-9 && (free.y + 9.4).abs() < 1e-9,
        "shift let go of the grid: {free:?}"
    );
}
