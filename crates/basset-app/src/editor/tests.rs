//! Headless tests of the interaction logic: tool dialogs and sketch mode drive the
//! document exactly as the UI would, without a window or GPU.

use basset_core::{BodyRef, FeatureKind, OriginPlane, PlaneRef, ProfileRef, RegionRef};
use basset_math::{Vec2, Vec3};
use basset_sketch::Entity;

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
        s.draw(&mut lines, &mut points);
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
    s.draw(&mut lines, &mut points);
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
    s.draw(&mut lines, &mut points);
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
    s.draw(&mut Vec::new(), &mut Vec::new());

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
    s.pattern.cols = 3;
    s.pattern.rows = 2;
    s.pattern.dx = 10.0;
    s.pattern.dy = 10.0;
    let created = s.apply_pattern().expect("pattern applied");
    assert!(created > 0);
    let profiles = s.sketch.profiles(&Default::default());
    assert_eq!(profiles.len(), 6);
    for p in &profiles {
        assert!((p.area() - seed_area).abs() < 1e-6, "{}", p.area());
    }
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
