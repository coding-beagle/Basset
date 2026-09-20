//! A headless harness that drives the whole program the way a user does.
//!
//! The application is split so that everything a user thinks of as the program lives in
//! [`Editor`], free of windowing and GPU types. This module is what makes that split pay:
//! it feeds the editor the same winit events a window would, runs the egui panels for a
//! frame with no surface to present to, builds the scene the renderer would draw, and
//! reads back what the user would have seen. Tests therefore exercise the real input
//! path — pixels, picking, tool dialogs, panels — rather than a parallel API written for
//! their benefit.
//!
//! Two things a window supplies cannot be forged, because winit keeps their fields
//! private: `KeyEvent` and `Modifiers`. Keys are delivered to [`Editor::on_key`], which
//! is exactly what `handle_window_event` does with them, and modifier state is written
//! into the pointer state the same event would have set.
//!
//! ```ignore
//! let mut h = Harness::new();
//! h.start_sketch(PlaneRef::Origin(OriginPlane::XY));
//! h.rectangle(Vec2::ZERO, Vec2::new(10.0, 10.0));
//! h.finish_sketch(true);
//! let body = h.extrude(Vec2::new(5.0, 5.0), 2.0);
//! assert!((h.volume(body) - 200.0).abs() < 1e-9);
//! ```

use std::path::{Path, PathBuf};

use basset_core::{
    BodyOp, BodyRef, ComponentId, Extent, FaceKey, FaceRef, FaceRole, FeatureKind, PlaneRef,
    ProfileRef, RegionRef,
};
use basset_kernel::OpId;
use basset_math::{Ray, Vec2, Vec3};
use basset_sketch::{Constraint, ConstraintId, Entity, EntityId};
use basset_viewport::Camera;
use winit::event::{DeviceId, ElementState, MouseButton, MouseScrollDelta, WindowEvent};
use winit::keyboard::{Key, NamedKey};

use super::sketch_mode::{self, SketchEditor, SketchTool};
use super::tools::{self, ToolKind};
use super::{Editor, Mode};

/// Window size every harness starts with. Fixed, because picking tolerances are in
/// pixels and a test that draws at (10, 10) mm must land on the same pixel every run.
const WINDOW: [u32; 2] = [800, 600];

/// What one headless frame produced: the text egui laid out, and the counts of the
/// scene the renderer would have drawn. Enough to assert on what the user would see
/// without a GPU in the loop.
pub(crate) struct Frame {
    /// Every text run egui painted, with the rectangle it occupies, in that order.
    pub texts: Vec<(egui::Rect, String)>,
    pub line_batches: usize,
    pub point_batches: usize,
    pub tri_batches: usize,
}

impl Frame {
    pub fn text(&self) -> Vec<&str> {
        self.texts.iter().map(|(_, t)| t.as_str()).collect()
    }

    pub fn has_text(&self, needle: &str) -> bool {
        self.texts.iter().any(|(_, t)| t.contains(needle))
    }

    /// Where a piece of text sits on screen, so a test can click the widget showing it.
    ///
    /// An exact match wins over one merely
    /// containing it, because a hint that mentions the OK button in its prose is not the
    /// OK button, and clicking the paragraph would silently do nothing while the test
    /// reported that it had pressed the button.
    pub fn rect_of(&self, needle: &str) -> Option<egui::Rect> {
        self.texts
            .iter()
            .find(|(_, t)| t.trim() == needle)
            .or_else(|| self.texts.iter().find(|(_, t)| t.contains(needle)))
            .map(|(r, _)| *r)
    }
}

pub(crate) struct Harness {
    pub editor: Editor,
    egui: egui::Context,
    /// The last frame's output, so `click_ui` can find a widget by its label.
    last: Option<Frame>,
}

impl Harness {
    pub fn new() -> Self {
        let mut editor = Editor::new(None);
        editor.set_window_size(WINDOW);
        editor.refresh_cache();
        Self {
            editor,
            egui: egui::Context::default(),
            last: None,
        }
    }

    /// A harness opening an existing document, as launching with a path does.
    pub fn open(path: impl Into<PathBuf>) -> Self {
        let mut h = Self::new();
        h.editor.open_path(path.into());
        h.editor.refresh_cache();
        h
    }

    // --- Frames -----------------------------------------------------------------------

    /// Runs one frame: panels, then the scene. Both are what the window loop does with
    /// no GPU involved, so a panic or a borrow error in either fails the test.
    pub fn frame(&mut self) -> &Frame {
        self.frame_with(Vec::new())
    }

    pub fn frame_with(&mut self, events: Vec<egui::Event>) -> &Frame {
        self.editor.refresh_cache();
        let size = self.editor.window_px;
        let input = egui::RawInput {
            screen_rect: Some(egui::Rect::from_min_size(
                egui::Pos2::ZERO,
                egui::vec2(size[0] as f32, size[1] as f32),
            )),
            events,
            ..Default::default()
        };
        let editor = &mut self.editor;
        let mut output = self.egui.run_ui(input, |ui| editor.ui(ui));
        // epaint insists texture deltas are consumed, not dropped. There is no renderer
        // here to upload them to, so they are dropped deliberately.
        output.textures_delta.clear();
        let mut texts = Vec::new();
        for clipped in &output.shapes {
            collect_text(&clipped.shape, &mut texts);
        }
        let scene = self.editor.scene();
        let frame = Frame {
            texts,
            line_batches: scene.lines.len(),
            point_batches: scene.points.len(),
            tri_batches: scene.tris.len(),
        };
        self.last.insert(frame)
    }

    /// Clicks the widget whose label contains `label`, and returns whether one was
    /// found. egui resolves a click against the widget the pointer was already over, so
    /// the move lands in one frame and the press and release in the next, exactly as a
    /// real pointer produces them.
    pub fn click_ui(&mut self, label: &str) -> bool {
        if self.last.is_none() {
            self.frame();
        }
        let Some(rect) = self.last.as_ref().and_then(|f| f.rect_of(label)) else {
            return false;
        };
        let pos = rect.center();
        self.frame_with(vec![egui::Event::PointerMoved(pos)]);
        let button = |pressed| egui::Event::PointerButton {
            pos,
            button: egui::PointerButton::Primary,
            pressed,
            modifiers: egui::Modifiers::default(),
        };
        self.frame_with(vec![button(true), button(false)]);
        // Panels queue their commands and run them after the UI closure returns, so the
        // effect of the click is visible from the next frame on.
        self.frame();
        true
    }

    /// Where a painted tool icon sits, which is the only handle there is on a button
    /// that shows no text.
    pub fn icon_rect(&mut self, salt: &str, tool: SketchTool) -> Option<egui::Rect> {
        self.frame();
        let id = egui::Id::new(("tool-icon", salt, tool.name()));
        self.egui.read_response(id).map(|r| r.rect)
    }

    /// Clicks at a point in egui's coordinates, for a widget that carries no text of its
    /// own — a painted icon, say — whose position is known from something beside it.
    pub fn click_at_ui(&mut self, pos: egui::Pos2) {
        self.press_ui(pos, egui::PointerButton::Primary);
    }

    /// Right-clicks a widget by its label, which is how the variant menus are opened.
    pub fn right_click_ui(&mut self, pos: egui::Pos2) {
        self.press_ui(pos, egui::PointerButton::Secondary);
    }

    /// Drags in egui's coordinates from `from` to `to`, which is how a viewport handle
    /// is grabbed: the press lands on the widget under the pointer and the move that
    /// follows is the drag it reports.
    ///
    /// Modifiers are a standing state in egui, set by `ModifiersChanged` and read by
    /// whatever asks during the frames that follow — which is what lets a widget know
    /// shift is down *while* it is being dragged. The per-event `modifiers` field does
    /// not update it, so a drag that only tagged its button events would be reported as
    /// unmodified. `egui_winit` forwards winit's own `ModifiersChanged` the same way.
    pub fn drag_ui(&mut self, from: egui::Pos2, to: egui::Pos2, modifiers: egui::Modifiers) {
        let button = |pos, pressed| egui::Event::PointerButton {
            pos,
            button: egui::PointerButton::Primary,
            pressed,
            modifiers,
        };
        self.frame_with(vec![
            egui::Event::ModifiersChanged(modifiers),
            egui::Event::PointerMoved(from),
        ]);
        self.frame_with(vec![button(from, true)]);
        self.frame_with(vec![egui::Event::PointerMoved(to)]);
        self.frame_with(vec![button(to, false)]);
        self.frame_with(vec![egui::Event::ModifiersChanged(
            egui::Modifiers::default(),
        )]);
    }

    /// Where a point of the model sits in egui's coordinates, for grabbing a handle that
    /// is drawn on the geometry rather than in a panel.
    pub fn at_world(&self, at: Vec3, ppp: f32) -> Option<egui::Pos2> {
        let px = self
            .editor
            .camera
            .world_to_screen(at, self.editor.window_px)?;
        Some(egui::pos2(
            (px[0] / f64::from(ppp)) as f32,
            (px[1] / f64::from(ppp)) as f32,
        ))
    }

    pub fn points_per_pixel(&self) -> f32 {
        self.egui.pixels_per_point()
    }

    fn press_ui(&mut self, pos: egui::Pos2, button: egui::PointerButton) {
        self.frame_with(vec![egui::Event::PointerMoved(pos)]);
        let event = |pressed| egui::Event::PointerButton {
            pos,
            button,
            pressed,
            modifiers: egui::Modifiers::default(),
        };
        self.frame_with(vec![event(true), event(false)]);
        // Panels queue their commands and run them after the UI closure returns, so the
        // effect is visible from the next frame on.
        self.frame();
    }

    // --- Pointer ----------------------------------------------------------------------

    fn send(&mut self, event: WindowEvent) {
        self.editor.handle_window_event(&event, false);
    }

    pub fn move_px(&mut self, pos: [f64; 2]) {
        self.send(WindowEvent::CursorMoved {
            device_id: DeviceId::dummy(),
            position: winit::dpi::PhysicalPosition::new(pos[0], pos[1]),
        });
    }

    pub fn button(&mut self, button: MouseButton, state: ElementState) {
        self.send(WindowEvent::MouseInput {
            device_id: DeviceId::dummy(),
            state,
            button,
        });
    }

    pub fn click_px(&mut self, pos: [f64; 2]) {
        self.move_px(pos);
        self.button(MouseButton::Left, ElementState::Pressed);
        self.button(MouseButton::Left, ElementState::Released);
    }

    /// A press, a move and a release: a drag, which the editor tells from a click by the
    /// distance moved while the button is down.
    pub fn drag_px(&mut self, from: [f64; 2], to: [f64; 2]) {
        self.move_px(from);
        self.button(MouseButton::Left, ElementState::Pressed);
        self.move_px(to);
        self.button(MouseButton::Left, ElementState::Released);
    }

    pub fn scroll(&mut self, steps: f32) {
        self.send(WindowEvent::MouseWheel {
            device_id: DeviceId::dummy(),
            delta: MouseScrollDelta::LineDelta(0.0, steps),
            phase: winit::event::TouchPhase::Moved,
        });
    }

    pub fn pointer_left_window(&mut self) {
        self.send(WindowEvent::CursorLeft {
            device_id: DeviceId::dummy(),
        });
    }

    /// Where a model-space point lands on screen. Clicks aimed at geometry go through
    /// here, so they exercise the camera and the pixel tolerances picking really uses.
    pub fn screen_of(&self, p: Vec3) -> [f64; 2] {
        self.editor
            .camera
            .world_to_screen(p, self.editor.window_px)
            .unwrap_or_else(|| panic!("{p} is not on screen"))
    }

    pub fn click_world(&mut self, p: Vec3) {
        let pos = self.screen_of(p);
        self.click_px(pos);
    }

    pub fn move_world(&mut self, p: Vec3) {
        let pos = self.screen_of(p);
        self.move_px(pos);
    }

    // --- Keyboard ---------------------------------------------------------------------

    /// winit's `Modifiers` cannot be built outside winit, so the harness writes the
    /// state its event would have set. Held until changed, as a real modifier is.
    pub fn set_modifiers(&mut self, shift: bool, ctrl: bool) {
        self.editor.pointer.shift = shift;
        self.editor.pointer.ctrl = ctrl;
    }

    pub fn key(&mut self, key: NamedKey) {
        self.editor.on_key(&Key::Named(key));
    }

    pub fn type_key(&mut self, text: &str) {
        self.editor.on_key(&Key::Character(text.into()));
    }

    /// A keystroke with control held, released afterwards: `Ctrl+Z` and friends.
    pub fn ctrl_key(&mut self, text: &str) {
        self.set_modifiers(false, true);
        self.type_key(text);
        self.set_modifiers(false, false);
    }

    // --- Sketching --------------------------------------------------------------------

    pub fn start_sketch(&mut self, plane: PlaneRef) {
        sketch_mode::enter_new(&mut self.editor, plane);
    }

    pub fn edit_sketch(&mut self, id: basset_core::FeatureId) {
        let cursor = self.editor.doc.timeline().cursor();
        sketch_mode::enter_existing(&mut self.editor, id, cursor);
    }

    pub fn sketch(&mut self) -> &mut SketchEditor {
        sketch(&mut self.editor)
    }

    pub fn rectangle(&mut self, a: Vec2, b: Vec2) {
        draw_rectangle(&mut self.editor, a, b);
    }

    pub fn line(&mut self, a: Vec2, b: Vec2) {
        draw_line(&mut self.editor, a, b);
    }

    pub fn dimension(&mut self, first: Vec2, second: Vec2) -> (ConstraintId, Constraint) {
        let camera = self.editor.camera;
        let window = self.editor.window_px;
        dimension(sketch(&mut self.editor), &camera, window, first, second)
    }

    pub fn finish_sketch(&mut self, keep: bool) {
        sketch_mode::finish(&mut self.editor, keep);
        self.editor.refresh_cache();
    }

    /// The id of the feature added last, which is what a tool or a finished sketch just
    /// produced.
    pub fn last_feature(&self) -> basset_core::FeatureId {
        self.editor
            .doc
            .timeline()
            .features()
            .last()
            .expect("the timeline is empty")
            .id
    }

    // --- Modelling --------------------------------------------------------------------

    pub fn start_tool(&mut self, kind: ToolKind) {
        tools::start_tool(&mut self.editor, kind);
    }

    pub fn sync_tool(&mut self) {
        tools::sync_tool(&mut self.editor);
        self.editor.refresh_cache();
    }

    pub fn confirm_tool(&mut self) {
        tools::confirm_tool(&mut self.editor);
        self.editor.refresh_cache();
    }

    pub fn cancel_tool(&mut self) {
        tools::cancel_tool(&mut self.editor);
        self.editor.refresh_cache();
    }

    pub fn select_region(&mut self, sketch: basset_core::FeatureId, sample: Vec2) {
        self.editor
            .selection
            .profiles
            .push(ProfileRef { sketch, sample });
    }

    /// Extrudes the region of the newest sketch containing `sample` through the tool, as
    /// picking it in the viewport and typing a distance does.
    pub fn extrude(&mut self, sample: Vec2, distance: f64) -> BodyRef {
        let sketch = self.last_feature();
        self.start_tool(ToolKind::Extrude);
        self.select_region(sketch, sample);
        self.sync_tool();
        self.editor
            .tool
            .as_mut()
            .expect("the extrude tool is running")
            .params
            .distance = distance;
        self.sync_tool();
        let feature = self
            .editor
            .tool
            .as_ref()
            .and_then(|t| t.feature)
            .expect("the extrude previews a feature");
        self.confirm_tool();
        BodyRef(feature)
    }

    /// The 10×10×2 block most modelling tests start from.
    pub fn block(&mut self) -> BodyRef {
        block(&mut self.editor)
    }

    pub fn volume(&mut self, body: BodyRef) -> f64 {
        self.editor
            .doc
            .state()
            .body(body)
            .unwrap_or_else(|| panic!("{body:?} is not in the model"))
            .solid
            .volume()
    }

    pub fn bodies(&mut self) -> Vec<BodyRef> {
        self.editor.doc.state().bodies.keys().copied().collect()
    }

    // --- Documents --------------------------------------------------------------------

    /// Saves to `path` and opens it again in a fresh harness: the document survives a
    /// round trip through the file format and regenerates to the same model.
    pub fn round_trip(&mut self, path: &Path) -> Self {
        self.save_as(path);
        Self::open(path)
    }

    pub fn save_as(&mut self, path: &Path) {
        self.editor.path = Some(path.to_path_buf());
        self.editor.save(false);
        assert!(self.editor.error.is_none(), "{:?}", self.editor.error);
    }
}

/// A temporary directory that removes itself, for the file tests. The standard library
/// has no such type and a test that leaves files behind in `/tmp` is a test that fails
/// on the second run.
pub(crate) struct TempDir(PathBuf);

impl TempDir {
    pub fn new(tag: &str) -> Self {
        let unique = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_nanos())
            .unwrap_or(0);
        let path = std::env::temp_dir().join(format!("basset-test-{tag}-{unique}"));
        std::fs::create_dir_all(&path).expect("creating the temporary directory");
        Self(path)
    }

    pub fn join(&self, name: &str) -> PathBuf {
        self.0.join(name)
    }
}

impl Drop for TempDir {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

fn collect_text(shape: &egui::epaint::Shape, out: &mut Vec<(egui::Rect, String)>) {
    match shape {
        egui::epaint::Shape::Text(t) => {
            let text = t.galley.text();
            if !text.is_empty() {
                out.push((t.galley.rect.translate(t.pos.to_vec2()), text.to_owned()));
            }
        }
        egui::epaint::Shape::Vec(shapes) => {
            for s in shapes {
                collect_text(s, out);
            }
        }
        _ => {}
    }
}

// --- Free helpers -------------------------------------------------------------------
//
// The sketch editor is driven by rays rather than pixels here: a test that draws a
// rectangle cares where its corners are in the sketch plane, not which pixel that was.
// Tests that do care go through the `Harness` pointer methods above.

/// A ray hitting the XY plane at `(x, y)` from above, as a click there would produce.
pub(crate) fn click_at(x: f64, y: f64) -> Ray {
    Ray::new(Vec3::new(x, y, 100.0), -Vec3::Z)
}

pub(crate) fn draw_rectangle(editor: &mut Editor, a: Vec2, b: Vec2) {
    let camera = editor.camera;
    let window = editor.window_px;
    let Mode::Sketch(s) = &mut editor.mode else {
        panic!("not sketching")
    };
    s.set_tool(SketchTool::Rectangle);
    for p in [a, b] {
        s.pointer_moved(&click_at(p.x, p.y), &camera, window, false);
        s.pointer_up(&click_at(p.x, p.y), &camera, window, true, false);
    }
    assert!(s.take_dirty());
    editor.commit_sketch();
}

/// Draws a line from `a` to `b` with the line tool and ends the chain.
pub(crate) fn draw_line(editor: &mut Editor, a: Vec2, b: Vec2) {
    let camera = editor.camera;
    let window = editor.window_px;
    let Mode::Sketch(s) = &mut editor.mode else {
        panic!("not sketching")
    };
    s.set_tool(SketchTool::Line);
    for p in [a, b] {
        s.pointer_moved(&click_at(p.x, p.y), &camera, window, false);
        s.pointer_up(&click_at(p.x, p.y), &camera, window, true, false);
    }
    s.finish_current();
    editor.commit_sketch();
}

pub(crate) fn click_with(editor: &mut Editor, tool: SketchTool, at: Vec2) {
    let camera = editor.camera;
    let window = editor.window_px;
    let Mode::Sketch(s) = &mut editor.mode else {
        panic!("not sketching")
    };
    s.set_tool(tool);
    s.pointer_moved(&click_at(at.x, at.y), &camera, window, false);
    s.pointer_up(&click_at(at.x, at.y), &camera, window, true, false);
    editor.commit_sketch();
}

pub(crate) fn sketch(editor: &mut Editor) -> &mut SketchEditor {
    match &mut editor.mode {
        Mode::Sketch(s) => s,
        Mode::Model => panic!("not sketching"),
    }
}

/// Drives the dimension tool through two clicks and returns the constraint it made.
pub(crate) fn dimension(
    s: &mut SketchEditor,
    camera: &Camera,
    window: [u32; 2],
    first: Vec2,
    second: Vec2,
) -> (ConstraintId, Constraint) {
    s.set_tool(SketchTool::Dimension);
    s.pointer_up(&click_at(first.x, first.y), camera, window, true, false);
    s.pointer_up(&click_at(second.x, second.y), camera, window, true, false);
    let (cid, _) = s.dim_edit.take().expect("dimension created");
    (cid, s.sketch.constraint(cid).cloned().unwrap())
}

pub(crate) fn point_at(s: &SketchEditor, p: Vec2) -> EntityId {
    s.sketch
        .entities()
        .find(|(_, e)| matches!(e.entity, Entity::Point { pos } if pos.distance(p) < 1e-6))
        .map(|(id, _)| id)
        .unwrap_or_else(|| panic!("no point at {p}"))
}

/// A base body for the tests: a 10×10×2 block from a rectangle on XY.
pub(crate) fn block(editor: &mut Editor) -> BodyRef {
    sketch_mode::enter_new(editor, PlaneRef::Origin(basset_core::OriginPlane::XY));
    draw_rectangle(editor, Vec2::new(0.0, 0.0), Vec2::new(10.0, 10.0));
    sketch_mode::finish(editor, true);
    let sketch = editor.doc.timeline().features()[0].id;
    let base = editor.doc.add_feature(FeatureKind::Extrude {
        regions: vec![RegionRef::Profile(ProfileRef {
            sketch,
            sample: Vec2::new(5.0, 5.0),
        })],
        extent: Extent::OneSide(2.0),
        operation: BodyOp::NewBody,
        component: ComponentId::ROOT,
    });
    editor.refresh_cache();
    BodyRef(base)
}

pub(crate) fn top_face(body: BodyRef) -> FaceRef {
    FaceRef {
        body,
        key: FaceKey::new(OpId::new(body.0.0), FaceRole::EndCap),
    }
}
