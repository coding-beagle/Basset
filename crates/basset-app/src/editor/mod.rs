//! The editor: document, camera, selection, tools and panels.
//!
//! Two modes exist. *Model* mode selects faces, edges, planes, profiles and bodies and
//! drives the modelling tools; *Sketch* mode hands pointer input to a [`SketchEditor`]
//! that draws on one plane. Both build the 3D scene from the document's regenerated
//! state every frame, so there is never a second copy of geometry to keep in sync.

mod files;
mod gizmo;
mod panels;
mod scene;
mod selection;
mod sketch_mode;
mod tools;
mod viewcube;

#[cfg(test)]
mod e2e;
#[cfg(test)]
pub(crate) mod harness;
#[cfg(test)]
mod tests;

use std::collections::{HashMap, HashSet};
use std::path::PathBuf;
use std::sync::Arc;

use basset_core::{
    BodyRef, ComponentId, Document, FeatureId, FeatureKind, FeatureStatus, SolvedSketch,
};
use basset_kernel::{Edge, Solid, Tessellated};
use basset_math::{Aabb, Frame, Vec3};
use basset_sketch::Font;
use basset_viewport::{Camera, MeshHandle, MeshStyle, Projection, ViewPreset};
use winit::event::{ElementState, MouseButton, MouseScrollDelta, WindowEvent};
use winit::keyboard::{Key, NamedKey};

pub use selection::{Pick, SelectMode, Selection};
pub use sketch_mode::SketchEditor;
pub use tools::Tool;

/// GPU mesh of a body, rebuilt only when the body's solid changes (detected by `Arc`
/// identity, which regeneration preserves for untouched bodies).
struct BodyMesh {
    handle: MeshHandle,
    tess: Arc<Tessellated>,
    solid: Arc<Solid>,
}

/// The CPU-side geometry picking and highlighting read for one body. Kept apart from the
/// GPU mesh for two reasons: it exists without a window, so the interaction logic can be
/// tested headless; and while a tool previews a feature it describes the body *before*
/// that feature, which is what the user is still picking from. Fusion does the same: the
/// original edges stay selectable under a fillet preview, and the second edge picked is
/// an edge of the input body, not of the half-finished result.
pub(crate) struct PickBody {
    pub solid: Arc<Solid>,
    pub edges: Arc<Vec<Edge>>,
    pub tess: Arc<Tessellated>,
}

#[derive(Default)]
struct Pointer {
    pos: Option<[f64; 2]>,
    press_pos: Option<[f64; 2]>,
    left: bool,
    middle: bool,
    right: bool,
    dragged: bool,
    shift: bool,
    ctrl: bool,
}

pub enum Mode {
    Model,
    Sketch(Box<SketchEditor>),
}

/// How bodies are drawn in the viewport. It is a property of the view, not of the
/// document, so it is neither saved nor undoable: switching to wireframe to see through a
/// part and back again should not put anything in the timeline.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum DisplayMode {
    /// Shaded faces with their feature edges drawn on top.
    #[default]
    Shaded,
    /// Shaded faces alone. Reads as a render rather than a drawing, and is the mode to
    /// judge a fillet blend in.
    NoEdges,
    /// Edges alone over the background: the skeleton view.
    Wireframe,
    /// Translucent faces with their edges, so a body standing behind another is visible
    /// through it.
    XRay,
}

impl DisplayMode {
    /// In the order the menu lists them and `D` cycles them.
    pub const ALL: [DisplayMode; 4] = [
        DisplayMode::Shaded,
        DisplayMode::NoEdges,
        DisplayMode::Wireframe,
        DisplayMode::XRay,
    ];

    pub fn title(self) -> &'static str {
        match self {
            DisplayMode::Shaded => "Shaded with edges",
            DisplayMode::NoEdges => "Shaded",
            DisplayMode::Wireframe => "Wireframe",
            DisplayMode::XRay => "X-ray",
        }
    }

    pub fn mesh_style(self) -> MeshStyle {
        match self {
            DisplayMode::Shaded => MeshStyle::ShadedWithEdges,
            DisplayMode::NoEdges => MeshStyle::Shaded,
            DisplayMode::Wireframe => MeshStyle::Wireframe,
            DisplayMode::XRay => MeshStyle::XRay,
        }
    }

    /// The next mode in [`DisplayMode::ALL`], wrapping. One key that walks the list beats
    /// four keys nobody remembers.
    pub fn next(self) -> Self {
        let i = Self::ALL.iter().position(|m| *m == self).unwrap_or(0);
        Self::ALL[(i + 1) % Self::ALL.len()]
    }
}

pub struct Editor {
    pub doc: Document,
    pub path: Option<PathBuf>,
    pub camera: Camera,
    pub window_px: [u32; 2],
    pub active_component: ComponentId,
    pub hidden_bodies: HashSet<BodyRef>,
    pub hidden_sketches: HashSet<FeatureId>,
    pub show_origin: bool,
    pub show_grid: bool,
    pub display: DisplayMode,
    pub selection: Selection,
    /// What a click may land on. Narrows a running tool's own filter.
    pub select_mode: SelectMode,
    pub hover: Option<Pick>,
    pub mode: Mode,
    pub tool: Option<Tool>,
    pub selected_feature: Option<FeatureId>,
    pub status: String,
    pub error: Option<String>,
    pub font: Option<Arc<Font>>,
    rename: Option<(FeatureId, String)>,
    meshes: HashMap<BodyRef, BodyMesh>,
    /// Bodies as picking sees them, see [`PickBody`]. Refreshed with the cache.
    pick_bodies: HashMap<BodyRef, PickBody>,
    // Read-only copies of the regenerated state for code paths (picking, panels) that
    // must not hold the document's mutable borrow. Refreshed once per frame.
    cached_planes: Vec<(FeatureId, Frame)>,
    cached_sketches: Vec<(FeatureId, Arc<SolvedSketch>)>,
    cached_bodies: Vec<(BodyRef, String)>,
    cached_body_components: HashMap<BodyRef, ComponentId>,
    cached_components: Vec<(ComponentId, String, Option<ComponentId>)>,
    cached_statuses: HashMap<FeatureId, FeatureStatus>,
    cached_extent: f64,
    pointer: Pointer,
    repaint: bool,
    exit: bool,
    title_dirty: bool,
}

impl Editor {
    pub fn new(path: Option<PathBuf>) -> Self {
        let font = Font::find_system_font().map(Arc::new);
        if font.is_none() {
            log::warn!("no system font found; sketch text will not produce profiles");
        }
        let mut editor = Self {
            doc: Document::new("Untitled"),
            path: None,
            camera: Camera::new_default(),
            window_px: [1, 1],
            active_component: ComponentId::ROOT,
            hidden_bodies: HashSet::new(),
            hidden_sketches: HashSet::new(),
            show_origin: false,
            show_grid: true,
            display: DisplayMode::default(),
            selection: Selection::default(),
            select_mode: SelectMode::default(),
            hover: None,
            mode: Mode::Model,
            tool: None,
            selected_feature: None,
            status: "Ready".into(),
            error: None,
            font,
            rename: None,
            meshes: HashMap::new(),
            pick_bodies: HashMap::new(),
            cached_planes: Vec::new(),
            cached_sketches: Vec::new(),
            cached_bodies: Vec::new(),
            cached_body_components: HashMap::new(),
            cached_components: Vec::new(),
            cached_statuses: HashMap::new(),
            cached_extent: 100.0,
            pointer: Pointer::default(),
            repaint: true,
            exit: false,
            title_dirty: true,
        };
        editor.doc.set_font(editor.font.clone());
        if let Some(path) = path {
            editor.open_path(path);
        }
        editor
    }

    pub fn set_window_size(&mut self, size: [u32; 2]) {
        self.window_px = size;
    }

    pub fn request_repaint(&mut self) {
        self.repaint = true;
    }

    pub fn take_repaint_request(&mut self) -> bool {
        std::mem::take(&mut self.repaint)
    }

    pub fn wants_exit(&self) -> bool {
        self.exit
    }

    pub fn take_title_change(&mut self) -> Option<String> {
        if !std::mem::take(&mut self.title_dirty) {
            return None;
        }
        let name = self
            .path
            .as_ref()
            .and_then(|p| p.file_name())
            .map(|n| n.to_string_lossy().into_owned());
        Some(format!(
            "Basset — {}",
            name.unwrap_or_else(|| self.doc.name.clone())
        ))
    }

    pub fn set_status(&mut self, s: impl Into<String>) {
        self.status = s.into();
    }

    pub fn report_error(&mut self, e: impl std::fmt::Display) {
        let msg = e.to_string();
        log::error!("{msg}");
        self.error = Some(msg);
    }

    pub fn is_sketching(&self) -> bool {
        matches!(self.mode, Mode::Sketch(_))
    }

    pub fn set_display_mode(&mut self, mode: DisplayMode) {
        self.display = mode;
        self.set_status(format!("Display: {}", mode.title()));
        self.repaint = true;
    }

    pub fn cycle_display_mode(&mut self) {
        self.set_display_mode(self.display.next());
    }

    // --- State cache ----------------------------------------------------------------

    /// Copies the parts of the regenerated state that panels and picking read.
    pub fn refresh_cache(&mut self) {
        self.refresh_pick_bodies();
        let state = self.doc.state();
        self.cached_planes = state.planes.iter().map(|(id, f)| (*id, *f)).collect();
        self.cached_sketches = state
            .sketches
            .iter()
            .map(|(id, s)| (*id, s.clone()))
            .collect();
        self.cached_bodies = state
            .bodies
            .values()
            .map(|b| (b.id, b.name.clone()))
            .collect();
        self.cached_body_components = state.bodies.values().map(|b| (b.id, b.component)).collect();
        self.cached_components = state
            .components
            .values()
            .map(|c| (c.id, c.name.clone(), c.parent))
            .collect();
        self.cached_statuses = state
            .statuses
            .iter()
            .map(|(id, s)| (*id, s.clone()))
            .collect();
        let mut aabb = Aabb::empty();
        for b in state.bodies.values() {
            aabb = aabb.union(&b.solid.aabb());
        }
        self.cached_extent = if aabb.is_empty() {
            100.0
        } else {
            aabb.extent().length().max(10.0)
        };
    }

    /// Rebuilds [`Self::pick_bodies`] from the state a click should land on: the model
    /// before the feature a tool is previewing, otherwise the current model. Edges and
    /// tessellation are only recomputed for solids that actually changed.
    fn refresh_pick_bodies(&mut self) {
        let previewing = self.tool.as_ref().and_then(|t| t.feature);
        let before = previewing.is_some_and(|id| self.doc.state_before(id).is_some());
        let state = match (before, previewing) {
            (true, Some(id)) => self.doc.state_before(id).expect("checked above"),
            _ => self.doc.state(),
        };
        let mut fresh: HashMap<BodyRef, PickBody> = HashMap::new();
        for (id, body) in &state.bodies {
            let reuse = self
                .pick_bodies
                .remove(id)
                .filter(|p| Arc::ptr_eq(&p.solid, &body.solid));
            fresh.insert(
                *id,
                reuse.unwrap_or_else(|| PickBody {
                    edges: Arc::new(body.solid.edges()),
                    tess: Arc::new(body.solid.tessellate()),
                    solid: body.solid.clone(),
                }),
            );
        }
        self.pick_bodies = fresh;
    }

    pub(crate) fn pick_body(&self, id: BodyRef) -> Option<&PickBody> {
        self.pick_bodies.get(&id)
    }

    // --- Meshes ------------------------------------------------------------------------

    /// Uploads meshes for bodies whose solids changed and drops meshes of bodies that no
    /// longer exist. Called once per frame before building the scene.
    pub fn sync_meshes(
        &mut self,
        renderer: &mut basset_viewport::Renderer,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
    ) {
        let state = self.doc.state();
        let mut live: HashSet<BodyRef> = HashSet::new();
        let mut uploads = Vec::new();
        for (id, body) in &state.bodies {
            live.insert(*id);
            if self
                .meshes
                .get(id)
                .is_some_and(|m| Arc::ptr_eq(&m.solid, &body.solid))
            {
                continue;
            }
            uploads.push((*id, body.solid.clone()));
        }
        for (id, solid) in uploads {
            // Picking usually tessellated this very solid already.
            let tess = match self.pick_bodies.get(&id) {
                Some(p) if Arc::ptr_eq(&p.solid, &solid) => p.tess.clone(),
                _ => Arc::new(solid.tessellate()),
            };
            // The edges come from the kernel's topology, not from the triangles: a face
            // is one shape to the user however many triangles it took to fill it.
            match renderer.upload_mesh(device, queue, &tess.mesh, &solid.display_edges()) {
                Ok(handle) => {
                    if let Some(old) = self.meshes.insert(
                        id,
                        BodyMesh {
                            handle,
                            tess,
                            solid,
                        },
                    ) {
                        renderer.remove_mesh(old.handle);
                    }
                }
                Err(e) => log::error!("mesh upload failed for {id:?}: {e}"),
            }
        }
        let dead: Vec<BodyRef> = self
            .meshes
            .keys()
            .filter(|k| !live.contains(k))
            .copied()
            .collect();
        for id in dead {
            if let Some(m) = self.meshes.remove(&id) {
                renderer.remove_mesh(m.handle);
            }
        }
    }

    fn meshes_iter(&self) -> impl Iterator<Item = (BodyRef, &BodyMesh)> {
        self.meshes.iter().map(|(id, m)| (*id, m))
    }

    pub fn visible_aabb(&mut self) -> Aabb {
        let hidden = self.hidden_bodies.clone();
        let state = self.doc.state();
        let mut aabb = Aabb::empty();
        for (id, body) in &state.bodies {
            if !hidden.contains(id) {
                aabb = aabb.union(&body.solid.aabb());
            }
        }
        if aabb.is_empty() {
            aabb = Aabb {
                min: Vec3::splat(-50.0),
                max: Vec3::splat(50.0),
            };
        }
        aabb
    }

    pub fn zoom_to_fit(&mut self) {
        let aabb = self.visible_aabb();
        self.camera.zoom_to_fit(&aabb);
        self.repaint = true;
    }

    pub fn look_from(&mut self, preset: ViewPreset) {
        self.camera.look_from(preset);
        self.repaint = true;
    }

    pub fn toggle_projection(&mut self) {
        if matches!(self.camera.projection, Projection::Perspective { .. }) {
            self.camera.set_orthographic();
        } else {
            self.camera.set_perspective();
        }
        self.repaint = true;
    }

    // --- Input ----------------------------------------------------------------------------

    pub fn pointer_left_viewport(&mut self) {
        self.pointer_over_ui();
        if let Mode::Sketch(s) = &mut self.mode {
            s.cursor = None;
        }
    }

    /// The pointer is over a panel or popup. Hover highlights go, but the sketch cursor
    /// stays where it was: the entry boxes sit right next to the geometry, and Enter in
    /// one of them must still know where the shape goes.
    pub fn pointer_over_ui(&mut self) {
        if self.hover.take().is_some() {
            self.repaint = true;
        }
        if let Mode::Sketch(s) = &mut self.mode {
            s.hover = None;
        }
    }

    pub fn handle_window_event(&mut self, event: &WindowEvent, egui_has_keyboard: bool) {
        match event {
            WindowEvent::CursorMoved { position, .. } => {
                let pos = [position.x, position.y];
                let prev = self.pointer.pos.replace(pos);
                if let (Some(prev), true) = (prev, self.pointer.middle || self.pointer.right) {
                    let (dx, dy) = (pos[0] - prev[0], pos[1] - prev[1]);
                    if self.pointer.right || (self.pointer.middle && self.pointer.shift) {
                        self.camera.orbit(-dx * 0.008, dy * 0.008);
                    } else {
                        self.camera.pan(dx, dy, self.window_px);
                    }
                    self.repaint = true;
                }
                if self.pointer.left
                    && let Some(press) = self.pointer.press_pos
                    && (pos[0] - press[0]).abs().max((pos[1] - press[1]).abs()) > 4.0
                {
                    self.pointer.dragged = true;
                }
                self.on_pointer_moved(pos);
            }
            WindowEvent::CursorLeft { .. } => {
                self.pointer.pos = None;
                self.pointer_left_viewport();
            }
            WindowEvent::MouseInput { state, button, .. } => {
                let down = *state == ElementState::Pressed;
                match button {
                    MouseButton::Left => {
                        if down {
                            self.pointer.left = true;
                            self.pointer.press_pos = self.pointer.pos;
                            self.pointer.dragged = false;
                            self.on_left_down();
                        } else {
                            self.pointer.left = false;
                            let clicked = !self.pointer.dragged;
                            self.on_left_up(clicked);
                            self.pointer.press_pos = None;
                        }
                    }
                    MouseButton::Middle => self.pointer.middle = down,
                    MouseButton::Right => {
                        self.pointer.right = down;
                        if !down && let Mode::Sketch(s) = &mut self.mode {
                            s.finish_current();
                            self.repaint = true;
                        }
                    }
                    _ => {}
                }
            }
            WindowEvent::MouseWheel { delta, .. } => {
                let steps = match delta {
                    MouseScrollDelta::LineDelta(_, y) => *y as f64,
                    MouseScrollDelta::PixelDelta(p) => p.y / 40.0,
                };
                self.camera.zoom((0.9f64).powf(steps));
                self.repaint = true;
            }
            WindowEvent::ModifiersChanged(m) => {
                self.pointer.shift = m.state().shift_key();
                self.pointer.ctrl = m.state().control_key();
                // Shift lets go of the grid for as long as it is held, and the drawing
                // path reads the sketch's own copy of it rather than reaching back here.
                if let Mode::Sketch(s) = &mut self.mode {
                    s.set_free_snap(self.pointer.shift);
                }
                self.repaint = true;
            }
            WindowEvent::KeyboardInput { event, .. }
                if event.state == ElementState::Pressed && !egui_has_keyboard =>
            {
                self.on_key(&event.logical_key);
            }
            _ => {}
        }
    }

    fn on_key(&mut self, key: &Key) {
        let ctrl = self.pointer.ctrl;
        let shift = self.pointer.shift;
        match key {
            Key::Named(NamedKey::Escape) => self.cancel(),
            Key::Named(NamedKey::Delete) | Key::Named(NamedKey::Backspace) => {
                self.delete_selected()
            }
            Key::Named(NamedKey::Enter) => self.confirm(),
            Key::Named(NamedKey::Tab) => {
                if let Mode::Sketch(s) = &mut self.mode {
                    s.focus_next_entry();
                }
            }
            // A number typed while drawing goes into the size entry box, so the user
            // never has to click the box first.
            Key::Character(c) if !ctrl && self.type_into_entry(c) => {}
            Key::Character(c) => match (c.to_ascii_lowercase().as_str(), ctrl, shift) {
                ("z", true, false) => self.undo(),
                ("z", true, true) | ("y", true, _) => self.redo(),
                ("s", true, _) => self.save(shift),
                ("o", true, _) => self.open(),
                ("n", true, _) => self.new_document(),
                ("f", false, _) => self.zoom_to_fit(),
                // D walks the display modes, as it does in the View menu. It is free in
                // sketch mode too: a sketch is drawn over whatever the bodies show.
                ("d", false, _) => self.cycle_display_mode(),
                // 1-5 switch the selection filter, as in the toolbar. A sketch has its
                // own filter over its own kinds of thing, on the same keys.
                (d @ ("1" | "2" | "3" | "4"), false, _) if matches!(self.mode, Mode::Sketch(_)) => {
                    let i = d.parse::<usize>().unwrap_or(1) - 1;
                    if let Mode::Sketch(s) = &mut self.mode {
                        s.set_pick(sketch_mode::SketchPick::ALL[i]);
                    }
                }
                (d @ ("1" | "2" | "3" | "4" | "5"), false, _) if !self.is_sketching() => {
                    let i = d.parse::<usize>().unwrap_or(1) - 1;
                    self.set_select_mode(SelectMode::ALL[i]);
                }
                ("x", false, _) => {
                    if let Mode::Sketch(s) = &mut self.mode {
                        s.toggle_construction();
                    }
                }
                // M moves the selection by typed offsets, E pushes the region under the
                // pointer into a solid: the two things a sketch is usually finished with.
                ("m", false, _) => {
                    if let Mode::Sketch(s) = &mut self.mode
                        && !s.begin_move()
                    {
                        let why = busy(s).unwrap_or(
                            "Select sketch geometry first, then press M to move it".into(),
                        );
                        self.set_status(why);
                    }
                }
                // O offsets it, after Fusion. Ctrl-O is Open and is matched above.
                ("o", false, _) => {
                    if let Mode::Sketch(s) = &mut self.mode {
                        if s.begin_offset() {
                            // The preview is real geometry in the feature by now, so it
                            // has to reach the document for anything downstream to see.
                            self.commit_sketch();
                        } else {
                            let why = busy(s).unwrap_or(
                                "Select the path or loop first, then press O to offset it".into(),
                            );
                            self.set_status(why);
                        }
                    }
                }
                ("e", false, _) => match &self.mode {
                    Mode::Sketch(_) => sketch_mode::extrude_region(self),
                    Mode::Model => {}
                },
                _ => {}
            },
            _ => {}
        }
        self.repaint = true;
    }

    fn type_into_entry(&mut self, text: &str) -> bool {
        match &mut self.mode {
            Mode::Sketch(s) => s.type_into_entry(text),
            Mode::Model => false,
        }
    }

    fn on_pointer_moved(&mut self, pos: [f64; 2]) {
        let ray = self.camera.ray_from_screen(pos, self.window_px);
        match &mut self.mode {
            Mode::Sketch(s) => {
                let dragging = self.pointer.left && self.pointer.dragged;
                s.pointer_moved(&ray, &self.camera, self.window_px, dragging);
            }
            Mode::Model => {
                let pick = selection::pick(self, &ray, &self.pick_filter(), 8.0);
                if pick != self.hover {
                    self.hover = pick;
                }
            }
        }
        self.repaint = true;
    }

    fn on_left_down(&mut self) {
        if let (Mode::Sketch(s), Some(pos)) = (&mut self.mode, self.pointer.pos) {
            let ray = self.camera.ray_from_screen(pos, self.window_px);
            s.pointer_down(&ray, &self.camera, self.window_px, self.pointer.shift);
            self.repaint = true;
        }
    }

    fn on_left_up(&mut self, clicked: bool) {
        let Some(pos) = self.pointer.pos else {
            // Released with the pointer off the window: there is nowhere to finish a drag
            // or a rubber band, so drop whatever was in progress rather than leaving it
            // drawn until the next click.
            if let Mode::Sketch(s) = &mut self.mode {
                s.cancel_current();
                self.repaint = true;
            }
            return;
        };
        let ray = self.camera.ray_from_screen(pos, self.window_px);
        let shift = self.pointer.shift;
        match &mut self.mode {
            Mode::Sketch(s) => {
                s.pointer_up(&ray, &self.camera, self.window_px, clicked, shift);
                let refused = s.take_constraint_error();
                if s.take_dirty() {
                    self.commit_sketch();
                }
                // A constraint the sketch would not take is the user's next move, not a
                // log line: the tool stays armed and the reason is on screen.
                if let Some(e) = refused {
                    self.report_error(e);
                }
            }
            Mode::Model if clicked => {
                let pick = selection::pick(self, &ray, &self.pick_filter(), 8.0);
                self.apply_pick(pick, shift);
            }
            Mode::Model => {}
        }
        self.repaint = true;
    }

    /// What a click may land on right now: the selection mode, narrowed by a running
    /// tool's own requirements.
    pub fn pick_filter(&self) -> selection::SelectionFilter {
        match self.tool.as_ref() {
            Some(tool) => self.select_mode.narrow(tool.filter()),
            None => self.select_mode.filter(),
        }
    }

    /// Switching what is selectable clears the selection, so the user is never left
    /// acting on things the new mode gives them no way to see or deselect. A running tool
    /// keeps its selection: the mode only narrows what can be added to it.
    pub fn set_select_mode(&mut self, mode: SelectMode) {
        self.select_mode = mode;
        if self.tool.is_none() {
            self.selection.clear();
            self.hover = None;
        }
        self.repaint = true;
    }

    fn apply_pick(&mut self, pick: Option<Pick>, additive: bool) {
        match pick {
            None => {
                if !additive {
                    self.selection.clear();
                    self.selected_feature = None;
                }
            }
            Some(pick) => {
                if !additive && self.tool.is_none() {
                    self.selection.clear();
                }
                if !tools::expand_face_pick(self, &pick) {
                    self.selection.toggle(&pick);
                }
                self.selected_feature = pick.feature();
            }
        }
        if let Some(tool) = self.tool.as_mut() {
            tool.selection_changed(&self.selection);
            tools::sync_tool(self);
        }
    }

    // --- Commands ---------------------------------------------------------------------

    pub fn cancel(&mut self) {
        match &mut self.mode {
            Mode::Sketch(s) => {
                // A modal operation is what Escape means while one is running, and the
                // revert has to reach the document: the copies it takes back are in the
                // feature by now, so anything downstream would keep showing them.
                if let Some(what) = s.modal_name() {
                    s.finish_modal(false);
                    self.commit_sketch();
                    self.set_status(format!("{what} cancelled"));
                } else if s.has_pending() || s.armed_constraint().is_some() {
                    s.cancel_current();
                    s.select_tool();
                } else {
                    s.select_tool();
                }
            }
            Mode::Model => {
                if self.tool.is_some() {
                    tools::cancel_tool(self);
                } else {
                    self.selection.clear();
                    self.selected_feature = None;
                }
            }
        }
        self.repaint = true;
    }

    pub fn confirm(&mut self) {
        if let Mode::Sketch(s) = &mut self.mode {
            if let Some(copies) = s.pattern_in_progress().then(|| s.finish_pattern(true)) {
                self.commit_sketch();
                let n = copies.unwrap_or(0);
                self.set_status(format!("Pattern added {n} entities"));
                self.repaint = true;
                return;
            }
            if s.move_in_progress() {
                s.finish_move(true);
                self.commit_sketch();
                self.repaint = true;
                return;
            }
            if let Some(made) = s.offset_in_progress().then(|| s.finish_offset(true)) {
                self.commit_sketch();
                self.set_status(match made {
                    Some(n) => format!("Offset added {n} curves"),
                    None => "Offset cancelled: there was nothing it could make".to_string(),
                });
                self.repaint = true;
                return;
            }
            s.submit_entry();
            if s.take_dirty() {
                self.commit_sketch();
            }
            self.repaint = true;
            return;
        }
        if self.tool.is_some() {
            tools::confirm_tool(self);
        }
    }

    pub fn delete_selected(&mut self) {
        match &mut self.mode {
            Mode::Sketch(s) => {
                s.delete_selected();
                self.commit_sketch();
            }
            Mode::Model => {
                if self.tool.is_some() {
                    return;
                }
                if let Some(id) = self.selected_feature.take() {
                    self.delete_feature(id);
                } else if let Some(body) = self.selection.bodies.first().copied() {
                    self.delete_feature(body.0);
                }
            }
        }
    }

    pub fn delete_feature(&mut self, id: FeatureId) {
        match self.doc.remove_feature(id) {
            Ok(f) => self.set_status(format!("Deleted {}", f.name)),
            Err(e) => self.report_error(e),
        }
        self.selection.clear();
        self.repaint = true;
    }

    pub fn undo(&mut self) {
        if let Mode::Sketch(s) = &mut self.mode {
            // Undo, while a move or a pattern is being set up, means the thing being set
            // up: putting it back is exactly what the user is asking to undo, and it is
            // also the only safe answer, since neither operation has a checkpoint of its
            // own and undoing past them would pop an unrelated one.
            if let Some(what) = s.modal_name() {
                s.finish_modal(false);
                self.commit_sketch();
                self.set_status(format!("{what} cancelled"));
                self.repaint = true;
                return;
            }
            if s.undo() {
                self.commit_sketch();
            }
            return;
        }
        if self.tool.is_some() {
            return;
        }
        if self.doc.undo() {
            self.selection.clear();
            self.set_status("Undo");
        }
        self.repaint = true;
    }

    pub fn redo(&mut self) {
        if let Mode::Sketch(s) = &mut self.mode {
            // Redo has nothing to say about an operation still being set up; the move
            // or pattern is what the numbers say, and changing them is how to change it.
            if s.modal() {
                self.set_status("Finish or cancel what you are doing first (Enter or Esc)");
                self.repaint = true;
                return;
            }
            if s.redo() {
                self.commit_sketch();
            }
            return;
        }
        if self.tool.is_some() {
            return;
        }
        if self.doc.redo() {
            self.selection.clear();
            self.set_status("Redo");
        }
        self.repaint = true;
    }

    pub fn set_cursor(&mut self, cursor: usize) {
        if self.tool.is_some() || self.is_sketching() {
            return;
        }
        self.doc.set_cursor(cursor);
        self.selection.clear();
        self.repaint = true;
    }

    /// Opens the right editor for a feature: sketch mode for sketches, the tool dialog
    /// for everything else.
    pub fn edit_feature(&mut self, id: FeatureId) {
        if self.tool.is_some() || self.is_sketching() {
            return;
        }
        let Some(feature) = self.doc.timeline().get(id) else {
            return;
        };
        if let FeatureKind::NewComponent { .. } = &feature.kind {
            self.rename = Some((id, feature.name.clone()));
            return;
        }
        // Editing means rolling the timeline back to just after the feature, so the
        // model shows exactly what the feature saw when it was created. The roll-back
        // is part of the edit's transaction so undo restores the cursor too.
        let index = self.doc.timeline().index_of(id).unwrap_or(0);
        let previous_cursor = self.doc.timeline().cursor();
        let is_sketch = matches!(feature.kind, FeatureKind::Sketch { .. });
        self.doc.begin_transaction();
        self.doc.set_cursor(index + 1);
        if is_sketch {
            sketch_mode::enter_existing(self, id, previous_cursor);
        } else {
            tools::edit_existing(self, id, previous_cursor);
        }
        self.repaint = true;
    }

    /// Writes the sketch editor's working copy into its feature so the model updates as
    /// the user draws.
    fn commit_sketch(&mut self) {
        let Mode::Sketch(s) = &self.mode else { return };
        let (id, sketch) = (s.feature, s.sketch.clone());
        if let Err(e) = self.doc.edit_feature_kind(id, |k| {
            if let FeatureKind::Sketch { sketch: target, .. } = k {
                *target = sketch;
            }
        }) {
            self.report_error(e);
        }
        self.repaint = true;
    }

    pub fn ui(&mut self, ui: &mut egui::Ui) {
        panels::show(self, ui);
    }

    pub fn scene(&self) -> basset_viewport::Scene<'_> {
        scene::build(self)
    }
}

/// Why a tool refused to start, when the reason is that something else is already
/// running. A message about selecting geometry first, said to someone who has selected
/// it and is halfway through a move, tells them nothing about what to do next.
fn busy(s: &sketch_mode::SketchEditor) -> Option<String> {
    let what = s.modal_name()?;
    Some(format!(
        "{what} is still up — finish it or cancel it first (Enter or Esc)"
    ))
}
