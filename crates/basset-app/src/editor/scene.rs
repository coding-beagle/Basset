//! Builds the per-frame [`Scene`] from editor state.
//!
//! Nothing here is cached: the scene is plain data referencing GPU meshes by handle, so
//! rebuilding it every frame is cheap and guarantees the picture always matches the
//! document and selection.

use basset_math::{Vec2, Vec3};
use basset_sketch::{Entity, Tessellation};
use basset_viewport::{LineBatch, MeshInstance, PointBatch, Scene, TriBatch};

use super::selection::Pick;
use super::tools;
use super::{DisplayMode, Editor, Mode};

const SELECT: [f32; 4] = [0.25, 0.6, 1.0, 1.0];
const HOVER: [f32; 4] = [1.0, 0.85, 0.3, 1.0];
const SKETCH: [f32; 4] = [0.85, 0.85, 0.9, 1.0];
const CONSTRUCTION: [f32; 4] = [0.7, 0.7, 0.5, 1.0];
const PLANE: [f32; 4] = [0.6, 0.7, 0.9, 0.6];
/// A selected or hovered region is filled, not just outlined: an enclosed area is a
/// surface to the user, and an outline alone reads as "these curves are selected".
const SELECT_FILL: [f32; 4] = [0.25, 0.6, 1.0, 0.28];
const HOVER_FILL: [f32; 4] = [1.0, 0.85, 0.3, 0.22];
/// Feature edges in the modes that draw them without a lit face behind them. The renderer's
/// default edge colour is near-black, which is right on a shaded face and invisible against
/// the background a wireframe body stands on.
const BARE_EDGE: [f32; 4] = [0.80, 0.84, 0.90, 1.0];
/// The pair of grid lines a drag has snapped onto. Faint: it is a confirmation, not a
/// thing to look at.
const SNAP_MARK: [f32; 4] = [0.75, 0.88, 1.0, 0.55];
/// Half-length of each of those lines, in pixels, so the mark is the same size to read
/// at any zoom.
const SNAP_MARK_PX: f64 = 26.0;

pub fn build(editor: &Editor) -> Scene<'_> {
    let mut scene = Scene::new(&editor.camera);
    scene.show_grid = editor.show_grid;

    // Bodies with face highlights.
    for (id, mesh) in editor.meshes_iter() {
        if editor.hidden_bodies.contains(&id) {
            continue;
        }
        let mut instance = MeshInstance::new(mesh.handle);
        instance.style = editor.display.mesh_style();
        let bare = matches!(editor.display, DisplayMode::Wireframe | DisplayMode::XRay);
        if bare {
            instance.edge_color = BARE_EDGE;
        }
        let selected_body = editor.selection.bodies.contains(&id)
            && editor.selection.faces.iter().all(|f| f.body != id);
        if selected_body {
            instance.color = [0.45, 0.6, 0.85, 1.0];
            // Wireframe draws no faces, so the tint that normally says "selected" has
            // nothing to land on and the edges have to carry it instead.
            if bare {
                instance.edge_color = SELECT;
            }
        }
        let face_index = |key| {
            mesh.tess
                .face_keys
                .iter()
                .position(|k| *k == key)
                .map(|i| i as u32)
        };
        for f in editor.selection.faces.iter().filter(|f| f.body == id) {
            instance.highlight_faces.extend(face_index(f.key));
        }
        if let Some(Pick::Face(f, _)) = &editor.hover
            && f.body == id
        {
            instance.highlight_faces.extend(face_index(f.key));
        }
        scene.meshes.push(instance);
    }

    // Edge highlights.
    let mut selected_edges = LineBatch::new(SELECT);
    selected_edges.width_px = 3.5;
    selected_edges.depth_test = false;
    let mut hovered_edges = LineBatch::new(HOVER);
    hovered_edges.width_px = 3.0;
    hovered_edges.depth_test = false;
    for e in &editor.selection.edges {
        push_edge(editor, &mut selected_edges, e);
    }
    if let Some(Pick::Edge(e, _)) = &editor.hover {
        // A blend tool takes the whole tangent chain from one click, so the hover has to
        // show the chain too: highlighting the single edge under the cursor and then
        // selecting four is the surprise the chain was added to remove. Same condition
        // as `tools::expand_face_pick`, Ctrl and the dialog checkbox included, so what
        // lights up is always what a click would take.
        let chaining = editor
            .tool
            .as_ref()
            .is_some_and(|t| t.prefers_edges() && t.params.tangent_chain && !editor.pointer.ctrl);
        if chaining {
            for edge in tools::tangent_chain(editor, e) {
                push_edge(editor, &mut hovered_edges, &edge);
            }
        } else {
            push_edge(editor, &mut hovered_edges, e);
        }
    }

    // Planes and axes.
    let mut planes = LineBatch::new(PLANE);
    planes.depth_test = false;
    let mut selected_planes = LineBatch::new(SELECT);
    selected_planes.width_px = 2.5;
    selected_planes.depth_test = false;
    for (plane, frame, half) in editor.visible_planes() {
        let corners = [
            Vec2::new(-half, -half),
            Vec2::new(half, -half),
            Vec2::new(half, half),
            Vec2::new(-half, half),
        ];
        let selected = editor.selection.planes.contains(&plane)
            || matches!(&editor.hover, Some(Pick::Plane(p, _)) if *p == plane);
        let batch = if selected {
            &mut selected_planes
        } else {
            &mut planes
        };
        for i in 0..4 {
            batch.segments.push([
                frame.to_world(corners[i]),
                frame.to_world(corners[(i + 1) % 4]),
            ]);
        }
    }
    if editor.show_origin
        || editor.tool.as_ref().is_some_and(|t| {
            matches!(
                t.kind,
                super::tools::ToolKind::Revolve | super::tools::ToolKind::AngledPlane
            )
        })
    {
        let len = editor.plane_half_size();
        for (dir, color) in [
            (Vec3::X, [0.9, 0.3, 0.3, 1.0]),
            (Vec3::Y, [0.3, 0.9, 0.3, 1.0]),
            (Vec3::Z, [0.3, 0.5, 0.95, 1.0]),
        ] {
            let mut axis = LineBatch::new(color);
            axis.width_px = 2.0;
            axis.segments.push([-dir * len, dir * len]);
            scene.lines.push(axis);
        }
    }

    // Sketches in model mode, plus profile / curve highlights.
    let tess = Tessellation::default();
    let mut sketch_lines = LineBatch::new(SKETCH);
    let mut construction = LineBatch::new(CONSTRUCTION);
    construction.dashed = true;
    let mut profile_lines = LineBatch::new(SELECT);
    profile_lines.width_px = 3.0;
    profile_lines.depth_test = false;
    let mut hover_lines = LineBatch::new(HOVER);
    hover_lines.width_px = 3.0;
    hover_lines.depth_test = false;
    let mut selected_fill = TriBatch::new(SELECT_FILL);
    let mut hover_fill = TriBatch::new(HOVER_FILL);
    for (id, solved) in editor.visible_sketches() {
        let to3 = |p: Vec2| solved.frame.to_world(p);
        for (eid, data) in solved.sketch.entities() {
            if let Entity::Point { .. } = data.entity {
                continue;
            }
            let Some(poly) = solved.sketch.curve_polyline(eid, &tess) else {
                continue;
            };
            let selected = editor.selection.curves.contains(&(id, eid));
            let hovered = matches!(&editor.hover, Some(Pick::Curve { sketch, entity, .. }) if *sketch == id && *entity == eid);
            let batch = if selected {
                &mut profile_lines
            } else if hovered {
                &mut hover_lines
            } else if data.construction {
                &mut construction
            } else {
                &mut sketch_lines
            };
            for w in poly.windows(2) {
                batch.segments.push([to3(w[0]), to3(w[1])]);
            }
        }
        // Profiles: the enclosed area filled, with its loops outlined on top. Selected in
        // blue, hovered in yellow.
        let highlight = |sample: Vec2, lines: &mut LineBatch, fill: &mut TriBatch| {
            let Some(region) = solved
                .profiles
                .iter()
                .filter(|p| p.contains(sample))
                .min_by(|a, b| a.area().total_cmp(&b.area()))
            else {
                return;
            };
            for c in region.loops() {
                let n = c.points.len();
                for i in 0..n {
                    lines
                        .segments
                        .push([to3(c.points[i]), to3(c.points[(i + 1) % n])]);
                }
            }
            fill.triangles.extend(region.triangles());
        };
        for p in editor.selection.profiles.iter().filter(|p| p.sketch == id) {
            highlight(p.sample, &mut profile_lines, &mut selected_fill);
        }
        if let Some(Pick::Profile(p, _)) = &editor.hover
            && p.sketch == id
        {
            highlight(p.sample, &mut hover_lines, &mut hover_fill);
        }
    }

    // Corners and sketch points. Everything a click could land on is drawn, because a
    // selection mode the user cannot see the targets of is worse than no mode at all.
    let mut points = Vec::new();
    let filter = editor.pick_filter();
    let mut candidate_pts = PointBatch::new([0.85, 0.85, 0.9, 0.9]);
    candidate_pts.size_px = 5.0;
    let mut selected_pts = PointBatch::new(SELECT);
    selected_pts.size_px = 9.0;
    let mut hovered_pts = PointBatch::new(HOVER);
    hovered_pts.size_px = 9.0;
    if filter.vertices {
        for id in editor.doc_state_bodies() {
            let Some(body) = editor.pick_body(id) else {
                continue;
            };
            if editor.hidden_bodies.contains(&id) {
                continue;
            }
            for point in basset_kernel::corners(&body.edges) {
                let hit = super::selection::VertexHit { body: id, point };
                let batch = if editor.selection.vertices.contains(&hit) {
                    &mut selected_pts
                } else if matches!(&editor.hover, Some(Pick::Vertex(v, _)) if *v == hit) {
                    &mut hovered_pts
                } else {
                    &mut candidate_pts
                };
                batch.points.push(point);
            }
        }
    }
    if filter.points {
        for (id, solved) in editor.visible_sketches() {
            for (eid, data) in solved.sketch.entities() {
                let Entity::Point { pos } = data.entity else {
                    continue;
                };
                let batch = if editor.selection.points.contains(&(id, eid)) {
                    &mut selected_pts
                } else if matches!(&editor.hover, Some(Pick::Point { sketch, entity, .. }) if *sketch == id && *entity == eid)
                {
                    &mut hovered_pts
                } else {
                    &mut candidate_pts
                };
                batch.points.push(solved.frame.to_world(pos));
            }
        }
    }
    points.extend([candidate_pts, selected_pts, hovered_pts]);

    if let Mode::Sketch(s) = &editor.mode {
        // Put the grid on the sketch plane: the user snaps to it, so they must see it.
        scene.grid_frame = s.frame;
        s.draw(&mut scene.lines, &mut points, &mut scene.tris);
        // A faint square shows the sketch plane's extent.
        let half = editor.plane_half_size();
        let corners = [
            Vec2::new(-half, -half),
            Vec2::new(half, -half),
            Vec2::new(half, half),
            Vec2::new(-half, half),
        ];
        let mut plane = LineBatch::new([0.5, 0.6, 0.8, 0.4]);
        plane.dashed = true;
        for i in 0..4 {
            plane.segments.push([
                s.frame.to_world(corners[i]),
                s.frame.to_world(corners[(i + 1) % 4]),
            ]);
        }
        scene.lines.push(plane);
    }

    // The transform manipulator: an arrow per direction the move can travel and a ring
    // per axis it can turn about. The grips on them are egui widgets drawn over this.
    if let Some(g) = super::gizmo::current(editor) {
        let arm = g.arm(&editor.camera, editor.window_px);
        let radius = g.radius(&editor.camera, editor.window_px);
        let px = editor.camera.pixel_size_at(g.origin, editor.window_px);
        for a in &g.arrows {
            let mut arrow = LineBatch::new(a.color);
            arrow.width_px = 2.5;
            arrow.depth_test = false;
            let tip = g.origin + a.dir * arm;
            arrow.segments.push([g.origin, tip]);
            let side = a.dir.cross(editor.camera.forward()).normalize_or_zero();
            let head = a.dir * (12.0 * px);
            let wing = side * (5.0 * px);
            arrow.segments.push([tip, tip - head + wing]);
            arrow.segments.push([tip, tip - head - wing]);
            scene.lines.push(arrow);
        }
        for r in &g.rings {
            let mut ring = LineBatch::new(r.color);
            ring.width_px = 2.0;
            ring.depth_test = false;
            let points = g.ring_points(r, radius);
            for w in points.windows(2) {
                ring.segments.push([w[0], w[1]]);
            }
            scene.lines.push(ring);
        }
    }

    // The offset's handle: a leader from the geometry out to the grip, so the gap the
    // user is dragging reads as the distance it is.
    if let Some(slider) = super::gizmo::slider(editor) {
        let mut leader = LineBatch::new(slider.color);
        leader.width_px = 1.5;
        leader.dashed = true;
        leader.depth_test = false;
        leader.segments.push([slider.anchor, slider.grip()]);
        scene.lines.push(leader);
    }

    // The span a measurement was taken across, so the number floating beside it is
    // visibly attached to the two things it came from.
    if let Some([a, b]) = super::measure::readout(editor).and_then(|r| r.leader) {
        let mut span = LineBatch::new(HOVER);
        span.width_px = 2.0;
        span.dashed = true;
        span.depth_test = false;
        span.segments.push([a, b]);
        scene.lines.push(span);
    }

    // The running tool's size, as an arrow from where it grows to where it reaches. The
    // grip at the tip is an egui widget drawn over this.
    if let Some(h) = super::tools::handle(editor) {
        let mut arrow = LineBatch::new([0.4, 0.75, 1.0, 1.0]);
        arrow.width_px = 2.5;
        arrow.depth_test = false;
        arrow.segments.push([h.origin, h.tip]);
        let px = editor.camera.pixel_size_at(h.tip, editor.window_px);
        let side = h.dir.cross(editor.camera.forward()).normalize_or_zero();
        let head = h.dir * (12.0 * px);
        let wing = side * (5.0 * px);
        arrow.segments.push([h.tip, h.tip - head + wing]);
        arrow.segments.push([h.tip, h.tip - head - wing]);
        scene.lines.push(arrow);
    }

    // What a drag in progress has landed on. The handle sits on the intersection of two
    // grid lines, and a short length of each drawn through it is what says so — the
    // number beside the grip says how far, this says where. Nothing is drawn while the
    // grid is not holding, so shift reads as the marker disappearing.
    if let Some(hint) = &editor.snap_hint
        && hint.on_grid
    {
        let px = editor.camera.pixel_size_at(hint.at, editor.window_px);
        let arm = px * SNAP_MARK_PX;
        let mut mark = LineBatch::new(SNAP_MARK);
        mark.width_px = 1.0;
        mark.depth_test = false;
        for dir in [scene.grid_frame.x, scene.grid_frame.y] {
            mark.segments
                .push([hint.at - dir * arm, hint.at + dir * arm]);
        }
        scene.lines.push(mark);
    }

    scene.lines.extend([
        sketch_lines,
        construction,
        planes,
        selected_planes,
        profile_lines,
        hover_lines,
        selected_edges,
        hovered_edges,
    ]);
    scene.points = points;
    scene.tris.extend([selected_fill, hover_fill]);
    scene.tris.retain(|t| !t.triangles.is_empty());
    scene.lines.retain(|l| !l.segments.is_empty());
    scene.points.retain(|p: &PointBatch| !p.points.is_empty());
    scene
}

fn push_edge(editor: &Editor, batch: &mut LineBatch, e: &basset_core::EdgeRef) {
    let Some(body) = editor.pick_body(e.body) else {
        return;
    };
    if let Some(edge) = body.edges.iter().find(|x| x.key == e.key) {
        for s in &edge.segments {
            batch.segments.push([s.start, s.end]);
        }
    }
}

#[cfg(test)]
mod tests {
    use basset_viewport::MeshStyle;
    use winit::keyboard::Key;

    use super::*;
    use crate::editor::harness::{Harness, click_at};
    use crate::editor::sketch_mode::SketchTool;
    use crate::editor::tools::ToolKind;
    use basset_core::{EdgeRef, OriginPlane, PlaneRef};

    /// The segments the hover highlight would draw: the one batch in the hover colour
    /// that edges are pushed into.
    fn hovered_segments(editor: &Editor) -> usize {
        build(editor)
            .lines
            .iter()
            .filter(|l| l.color == HOVER && l.width_px == 3.0)
            .map(|l| l.segments.len())
            .sum()
    }

    /// Hovering one edge of a slot's rim lights the whole chain, because that is what the
    /// click takes; Ctrl narrows the highlight to the single edge in the same breath as
    /// it narrows the pick. The two must not disagree.
    #[test]
    fn the_hover_lights_the_chain_a_click_would_take() {
        let mut h = Harness::new();
        h.start_sketch(PlaneRef::Origin(OriginPlane::XY));
        let camera = h.editor.camera;
        let window = h.editor.window_px;
        let s = h.sketch();
        s.snap_to_grid = false;
        s.set_tool(SketchTool::SlotOverall);
        for p in [Vec2::ZERO, Vec2::new(20.0, 0.0), Vec2::new(10.0, 2.0)] {
            s.pointer_up(&click_at(p.x, p.y), &camera, window, true, false);
        }
        h.editor.commit_sketch();
        h.finish_sketch(true);
        let body = h.extrude(Vec2::new(10.0, 0.0), 3.0);

        h.start_tool(ToolKind::Fillet);
        // An edge that something runs on from: the rim of the slot, not one of its sides.
        let edges = h.editor.pick_body(body).unwrap().edges.clone();
        let chained = edges
            .iter()
            .map(|e| EdgeRef { body, key: e.key })
            .find(|e| tools::tangent_chain(&h.editor, e).len() > 1)
            .expect("the slot's rim is a tangent chain");

        h.editor.hover = Some(Pick::Edge(chained, 0.0));
        let whole_chain = hovered_segments(&h.editor);

        h.set_modifiers(false, true);
        let single = hovered_segments(&h.editor);
        assert!(
            whole_chain > single,
            "the chain hover ({whole_chain}) must outdraw the single edge ({single})"
        );

        // The dialog's checkbox is the same choice made once and for all.
        h.set_modifiers(false, false);
        h.editor.tool.as_mut().unwrap().params.tangent_chain = false;
        assert_eq!(hovered_segments(&h.editor), single);
    }

    #[test]
    fn every_display_mode_names_itself_and_maps_to_a_style() {
        let styles: Vec<MeshStyle> = DisplayMode::ALL.iter().map(|m| m.mesh_style()).collect();
        for (i, mode) in DisplayMode::ALL.iter().enumerate() {
            assert!(!mode.title().is_empty(), "{mode:?} has no menu label");
            assert!(
                !styles[..i].contains(&styles[i]),
                "{mode:?} draws the same as an earlier mode"
            );
        }
        assert_eq!(DisplayMode::default(), DisplayMode::Shaded);
        // The shaded modes differ only in their edges; the other two are see-through.
        assert!(DisplayMode::Shaded.mesh_style().draws_edges());
        assert!(!DisplayMode::NoEdges.mesh_style().draws_edges());
        assert!(!DisplayMode::Wireframe.mesh_style().draws_faces());
        assert!(DisplayMode::XRay.mesh_style().is_translucent());
    }

    #[test]
    fn cycling_visits_every_mode_and_returns() {
        let mut mode = DisplayMode::default();
        let mut seen = vec![mode];
        for _ in 0..DisplayMode::ALL.len() - 1 {
            mode = mode.next();
            assert!(!seen.contains(&mode), "{mode:?} came round twice");
            seen.push(mode);
        }
        assert_eq!(mode.next(), DisplayMode::default(), "the cycle wraps");
    }

    #[test]
    fn the_d_key_cycles_the_display_mode() {
        let mut editor = Editor::new(None);
        for expected in [
            DisplayMode::NoEdges,
            DisplayMode::Wireframe,
            DisplayMode::XRay,
            DisplayMode::Shaded,
        ] {
            editor.on_key(&Key::Character("d".into()));
            assert_eq!(editor.display, expected);
        }
        assert!(
            editor.status.contains("Shaded"),
            "status: {}",
            editor.status
        );
    }

    /// A drag that snapped has to show what it snapped to, or the number jumping is
    /// indistinguishable from the handle sticking. The mark is the two grid lines the
    /// handle landed on, and it is drawn only while the grid is holding: shift reads as
    /// the mark going away.
    #[test]
    fn a_snapped_drag_marks_the_grid_lines_it_landed_on() {
        let mut editor = Editor::new(None);
        editor.window_px = [800, 600];
        editor.refresh_cache();
        let marks = |editor: &Editor| {
            build(editor)
                .lines
                .iter()
                .filter(|l| l.color == SNAP_MARK)
                .map(|l| l.segments.len())
                .sum::<usize>()
        };
        assert_eq!(marks(&editor), 0, "nothing is being dragged");

        editor.snap_hint = Some(super::super::snap::Hint {
            at: Vec3::new(10.0, 5.0, 0.0),
            text: "10.00 mm".into(),
            on_grid: true,
        });
        assert_eq!(marks(&editor), 2, "one line of the grid along each axis");

        if let Some(hint) = editor.snap_hint.as_mut() {
            hint.on_grid = false;
        }
        assert_eq!(marks(&editor), 0, "a freed drag has nothing to point at");
    }

    #[test]
    fn the_scene_follows_the_grid_and_origin_toggles() {
        let mut editor = Editor::new(None);
        editor.window_px = [800, 600];
        editor.refresh_cache();
        assert!(build(&editor).show_grid);

        editor.show_grid = false;
        let plain = build(&editor);
        assert!(!plain.show_grid);
        let without_origin = plain.lines.len();
        drop(plain);

        // One axis batch per direction, plus the batch holding the three plane outlines.
        editor.show_origin = true;
        assert_eq!(build(&editor).lines.len(), without_origin + 4);
    }
}
