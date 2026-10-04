//! The Simulate tool: a linear elastic study of one body, set up and run from a dialog
//! and read as a stress plot in the viewport.
//!
//! A study is not a timeline feature. It changes nothing about the model — the body is
//! the same body after a run as before — and a feature that produced no geometry would
//! sit in the timeline as a step that regenerates to nothing, costing an undo entry every
//! time a load was retyped. Like [`Measure`](super::measure::Measure) it is therefore
//! editor state: the editor holds one [`Simulation`] with the body, the faces picked for
//! it, the material and the loads, and the last [`Results`] together with the document
//! revision they were computed at. The revision is what keeps the plot honest: an edit
//! to the model — a fillet, an undo, a parameter — leaves results that describe a body
//! that no longer exists, and rather than keep showing them the dialog marks them stale
//! and the viewport goes back to the plain body until the study is run again.
//!
//! The study is run synchronously on the UI thread. A mesh of the default size solves in
//! well under a second; the element limit in [`basset_fea`] is what keeps a finer one
//! from freezing the window, and its error says to coarsen.

use basset_core::BodyRef;
use basset_fea::{FeaError, Load, LoadKind, Material, Results, Study};
use basset_kernel::{FaceKey, Solid};
use basset_math::{TriMesh, Vec3};
use basset_viewport::{MeshHandle, stress_ramp};

use super::Editor;
use super::selection::{Pick, SelectionFilter};

/// Which of the dialog's two face lists a click in the viewport goes to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Armed {
    Fixed,
    Load,
}

/// The materials the dialog offers by name. `Custom` is what the combo reads once either
/// number has been typed over, so the user is never shown "Steel" beside numbers that
/// are not steel's.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Preset {
    Steel,
    Aluminium,
    Custom,
}

impl Preset {
    pub const NAMED: [Preset; 2] = [Preset::Steel, Preset::Aluminium];

    pub fn name(self) -> &'static str {
        match self {
            Preset::Steel => "Steel",
            Preset::Aluminium => "Aluminium",
            Preset::Custom => "Custom",
        }
    }

    pub fn material(self) -> Option<Material> {
        match self {
            Preset::Steel => Some(Material::STEEL),
            Preset::Aluminium => Some(Material::ALUMINIUM),
            Preset::Custom => None,
        }
    }

    /// The preset these numbers are, if they are one.
    pub fn of(material: Material) -> Preset {
        Self::NAMED
            .into_iter()
            .find(|p| p.material() == Some(material))
            .unwrap_or(Preset::Custom)
    }
}

/// How the load faces are loaded: one total force shared over them, or a pressure.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LoadForm {
    Force,
    Pressure,
}

/// A run's results and the document they describe.
#[derive(Clone, Debug)]
pub struct Outcome {
    pub results: Results,
    /// `Document::revision` when the study ran. Any edit since moves it, and the results
    /// are then of a body the model no longer has.
    pub revision: u64,
    /// Which run this is, counted per simulation. The GPU copy of the plot is keyed on
    /// it, so a re-run with the same scale still re-uploads.
    pub run: u64,
}

impl Outcome {
    /// The range of the plotted stress: the nodal values the surface mesh is coloured
    /// by, so the legend's ends are the plot's own.
    pub fn stress_range(&self) -> (f64, f64) {
        let mut lo = f64::INFINITY;
        let mut hi = f64::NEG_INFINITY;
        for &v in &self.results.nodal_von_mises {
            lo = lo.min(v);
            hi = hi.max(v);
        }
        if lo > hi { (0.0, 0.0) } else { (lo, hi) }
    }

    /// The deformed surface with one colour per vertex, ready for the renderer.
    pub fn plot(&self, scale: f64) -> (TriMesh, Vec<[f32; 3]>) {
        let (mesh, values) = self.results.deformed_surface(scale);
        let colors = plot_colors(&values, self.stress_range());
        (mesh, colors)
    }
}

/// The ramp applied to each value, with a flat plot — every node at the same stress —
/// reading as the cold end rather than dividing by zero.
pub fn plot_colors(values: &[f64], (lo, hi): (f64, f64)) -> Vec<[f32; 3]> {
    let span = hi - lo;
    values
        .iter()
        .map(|&v| {
            let t = if span > 0.0 { (v - lo) / span } else { 0.0 };
            stress_ramp(t as f32)
        })
        .collect()
}

/// The whole tool: what the user has set up, and what the last run said.
#[derive(Clone, Debug)]
pub struct Simulation {
    /// The body under study. `None` until a face is picked, when several bodies exist
    /// and none was selected.
    pub body: Option<BodyRef>,
    pub fixed: Vec<FaceKey>,
    pub loaded: Vec<FaceKey>,
    pub armed: Armed,
    pub load_form: LoadForm,
    /// Total force in newtons, when the load is a force.
    pub force: Vec3,
    /// Pressure in MPa, when it is a pressure.
    pub pressure: f64,
    pub material: Material,
    /// Target brick size in millimetres.
    pub element_size: f64,
    pub outcome: Option<Outcome>,
    /// How many times the displacement is exaggerated in the plot.
    pub scale: f64,
    /// Whether the plot replaces the body in the viewport.
    pub show: bool,
    /// What the dialog says under the Run button: the last error verbatim, or what the
    /// run came to.
    pub message: String,
    runs: u64,
}

/// What a click may land on while the dialog is open: faces, which are the only thing a
/// study is written on.
pub const FILTER: SelectionFilter = SelectionFilter {
    faces: true,
    edges: false,
    vertices: false,
    points: false,
    planes: false,
    profiles: false,
    curves: false,
};

pub const PROMPT: &str = "Simulate: arm a list in the dialog, then click faces of the body to hold or load. \
     Esc to close.";

/// The default brick: a twentieth of the body's longest side, which gives a few thousand
/// elements on any proportioned part and solves while the user watches.
const BRICKS_ALONG_LONGEST: f64 = 20.0;
/// How far the deformed plot moves at its default scale, as a fraction of the body's
/// longest side: visible without reading as a different shape.
const DEFAULT_PLOT_TRAVEL: f64 = 0.1;

impl Simulation {
    fn new(body: Option<BodyRef>, longest: Option<f64>) -> Self {
        Self {
            body,
            fixed: Vec::new(),
            loaded: Vec::new(),
            armed: Armed::Fixed,
            load_form: LoadForm::Force,
            force: Vec3::new(0.0, 0.0, -100.0),
            pressure: 1.0,
            material: Material::STEEL,
            element_size: longest.map_or(5.0, |l| l / BRICKS_ALONG_LONGEST),
            outcome: None,
            scale: 1.0,
            show: true,
            message: String::new(),
            runs: 0,
        }
    }

    /// The study as the solver takes it.
    pub fn study(&self) -> Study {
        let kind = match self.load_form {
            LoadForm::Force => LoadKind::Force(self.force),
            LoadForm::Pressure => LoadKind::Pressure(self.pressure),
        };
        Study {
            material: self.material,
            fixed: self.fixed.clone(),
            loads: self
                .loaded
                .iter()
                .map(|&face| Load { face, kind })
                .collect(),
            element_size: self.element_size,
        }
    }

    /// Whether the results describe the document as it stands.
    pub fn is_stale(&self, revision: u64) -> bool {
        self.outcome
            .as_ref()
            .is_some_and(|o| o.revision != revision)
    }

    /// The results the viewport should be drawing instead of the body, if any: shown,
    /// and of this very model.
    pub fn plotted(&self, revision: u64) -> Option<(BodyRef, &Outcome)> {
        if !self.show {
            return None;
        }
        let outcome = self.outcome.as_ref().filter(|o| o.revision == revision)?;
        Some((self.body?, outcome))
    }

    pub fn armed_list(&mut self) -> &mut Vec<FaceKey> {
        match self.armed {
            Armed::Fixed => &mut self.fixed,
            Armed::Load => &mut self.loaded,
        }
    }
}

/// The longest side of a body's bounding box, what the defaults are measured against.
fn longest_side(solid: &Solid) -> Option<f64> {
    let aabb = solid.aabb();
    (!aabb.is_empty()).then(|| aabb.extent().max_element())
}

fn solid_of(editor: &mut Editor, body: BodyRef) -> Option<std::sync::Arc<Solid>> {
    editor.doc.state().body(body).map(|b| b.solid.clone())
}

pub fn start(editor: &mut Editor) {
    if editor.is_sketching() {
        return;
    }
    if editor.tool.is_some() {
        super::tools::cancel_tool(editor);
    }
    super::measure::stop(editor);
    // One body selected is the body; so is the only body there is. Otherwise the first
    // face picked decides, since a study with no body cannot take a face.
    let body = match editor.selection.bodies.as_slice() {
        [one] => Some(*one),
        _ => match editor.cached_bodies.as_slice() {
            [(one, _)] => Some(*one),
            _ => None,
        },
    };
    let longest = body
        .and_then(|b| solid_of(editor, b))
        .and_then(|s| longest_side(&s));
    editor.selection.clear();
    editor.hover = None;
    editor.simulation = Some(Simulation::new(body, longest));
    editor.set_status(PROMPT);
    editor.request_repaint();
}

pub fn stop(editor: &mut Editor) {
    if editor.simulation.take().is_some() {
        editor.set_status("Simulate closed");
        editor.request_repaint();
    }
}

/// Records a click while the dialog is open. Returns whether the click was consumed; it
/// always is, so picking for a study never doubles as a selection some later tool acts
/// on.
pub fn clicked(editor: &mut Editor, pick: Option<&Pick>) -> bool {
    if editor.simulation.is_none() {
        return false;
    }
    let Some(Pick::Face(face, _)) = pick else {
        return true;
    };
    let face = *face;
    let body_name = editor.body_name(face.body);
    let longest = solid_of(editor, face.body).and_then(|s| longest_side(&s));
    let Some(sim) = editor.simulation.as_mut() else {
        return false;
    };
    match sim.body {
        None => {
            sim.body = Some(face.body);
            if let Some(l) = longest {
                sim.element_size = l / BRICKS_ALONG_LONGEST;
            }
        }
        Some(b) if b != face.body => {
            let studied = editor.body_name(b);
            editor.set_status(format!(
                "A study is of one body: pick faces of {studied}, not of {body_name}"
            ));
            return true;
        }
        Some(_) => {}
    }
    // A face is either held or loaded, never both, so it leaves the other list as it
    // joins this one; picked again, it leaves this one too.
    let armed = sim.armed;
    let other = match armed {
        Armed::Fixed => &mut sim.loaded,
        Armed::Load => &mut sim.fixed,
    };
    other.retain(|k| *k != face.key);
    let list = sim.armed_list();
    match list.iter().position(|k| *k == face.key) {
        Some(i) => {
            list.remove(i);
        }
        None => list.push(face.key),
    }
    let (fixed, loaded) = (sim.fixed.len(), sim.loaded.len());
    editor.set_status(format!(
        "{fixed} fixed, {loaded} loaded; picking {}",
        match armed {
            Armed::Fixed => "fixed faces",
            Armed::Load => "load faces",
        }
    ));
    editor.request_repaint();
    true
}

/// Runs the study as the dialog has it and keeps the answer, or the reason there is none.
pub fn run(editor: &mut Editor) {
    let revision = editor.doc.revision();
    let Some(body) = editor.simulation.as_ref().and_then(|s| s.body) else {
        if let Some(sim) = editor.simulation.as_mut() {
            sim.message = "Pick a face of the body to study first".into();
        }
        return;
    };
    let solid = solid_of(editor, body);
    let Some(sim) = editor.simulation.as_mut() else {
        return;
    };
    let Some(solid) = solid else {
        sim.message = "The body is no longer in the model".into();
        sim.outcome = None;
        return;
    };
    let longest = longest_side(&solid).unwrap_or(1.0);
    match basset_fea::run(&solid, &sim.study()) {
        Ok(results) => {
            let (max_disp, _) = results.max_displacement();
            let (max_vm, _) = results.max_von_mises();
            // The default exaggeration puts the largest movement at a tenth of the body:
            // a real displacement of microns would otherwise plot as no movement at all.
            sim.scale = if max_disp > 0.0 {
                DEFAULT_PLOT_TRAVEL * longest / max_disp
            } else {
                1.0
            };
            sim.runs += 1;
            sim.message = format!(
                "Solved {} elements in {} iterations: max displacement {:.4} mm, max von Mises {:.2} MPa",
                results.mesh.elements.len(),
                results.iterations,
                max_disp,
                max_vm
            );
            sim.outcome = Some(Outcome {
                results,
                revision,
                run: sim.runs,
            });
            sim.show = true;
        }
        Err(e) => {
            sim.message = error_text(&e);
            sim.outcome = None;
        }
    }
    editor.request_repaint();
}

/// The solver's message as it wrote it. A separate function only so the dialog and the
/// tests agree on what is shown.
pub fn error_text(e: &FeaError) -> String {
    e.to_string()
}

/// A face as the dialog lists it. The key's parts are what the user can match against
/// the timeline: the feature that made the face and which of its faces it is.
fn face_label(key: &FaceKey) -> String {
    format!("{:?} of feature {}", key.role, key.op.feature)
}

// --- The dialog --------------------------------------------------------------------

/// What the dialog asked for this frame, carried out once the window closure has let go
/// of the editor.
#[derive(Default)]
struct Asked {
    run: bool,
    close: bool,
}

pub fn dialog(editor: &mut Editor, ctx: &egui::Context) {
    let Some(sim) = editor.simulation.as_ref() else {
        return;
    };
    let body_name = sim.body.map(|b| editor.body_name(b));
    let revision = editor.doc.revision();
    let mut asked = Asked::default();
    egui::Window::new("Simulate")
        .id(egui::Id::new("simulate-dialog"))
        .collapsible(false)
        .resizable(false)
        .anchor(egui::Align2::RIGHT_TOP, [-12.0, 190.0])
        .show(ctx, |ui| {
            let Some(sim) = editor.simulation.as_mut() else {
                return;
            };
            match &body_name {
                Some(name) => ui.label(format!("Body: {name}")),
                None => ui.label("Click a face of the body to study"),
            };
            ui.separator();
            face_list(ui, sim, Armed::Fixed);
            face_list(ui, sim, Armed::Load);
            ui.separator();
            load_ui(ui, sim);
            ui.separator();
            material_ui(ui, sim);
            ui.horizontal(|ui| {
                ui.label("Element size");
                ui.add(
                    egui::DragValue::new(&mut sim.element_size)
                        .speed(0.1)
                        .range(0.01..=f64::INFINITY)
                        .suffix(" mm"),
                );
            });
            ui.separator();
            ui.horizontal(|ui| {
                if ui.button("Run").clicked() {
                    asked.run = true;
                }
                if ui.button("Close").clicked() {
                    asked.close = true;
                }
            });
            if !sim.message.is_empty() {
                let text = egui::RichText::new(&sim.message);
                if sim.outcome.is_some() {
                    ui.label(text.weak());
                } else {
                    ui.colored_label(egui::Color32::from_rgb(230, 120, 100), &sim.message);
                }
            }
            if sim.is_stale(revision) {
                ui.colored_label(
                    egui::Color32::from_rgb(230, 180, 90),
                    "Results are stale: the model has changed since this run",
                );
            }
            if sim.outcome.is_some() {
                ui.separator();
                results_ui(ui, sim);
            }
        });
    if asked.run {
        run(editor);
    }
    if asked.close {
        stop(editor);
    }
}

fn face_list(ui: &mut egui::Ui, sim: &mut Simulation, which: Armed) {
    let (title, armed_label) = match which {
        Armed::Fixed => ("Fixed faces", "Pick fixed"),
        Armed::Load => ("Load faces", "Pick loads"),
    };
    ui.horizontal(|ui| {
        ui.label(egui::RichText::new(title).strong());
        let armed = sim.armed == which;
        if ui
            .selectable_label(armed, armed_label)
            .on_hover_text(
                "Clicks in the viewport add faces to this list; click a face again to remove it",
            )
            .clicked()
        {
            sim.armed = which;
        }
    });
    let list = match which {
        Armed::Fixed => &mut sim.fixed,
        Armed::Load => &mut sim.loaded,
    };
    if list.is_empty() {
        ui.label(egui::RichText::new("none").weak());
    }
    let mut remove = None;
    for (i, key) in list.iter().enumerate() {
        ui.horizontal(|ui| {
            ui.label(face_label(key));
            if ui.small_button("×").on_hover_text("Remove").clicked() {
                remove = Some(i);
            }
        });
    }
    if let Some(i) = remove {
        list.remove(i);
    }
}

fn load_ui(ui: &mut egui::Ui, sim: &mut Simulation) {
    ui.horizontal(|ui| {
        ui.label("Load");
        ui.selectable_value(&mut sim.load_form, LoadForm::Force, "Force");
        ui.selectable_value(&mut sim.load_form, LoadForm::Pressure, "Pressure");
    });
    match sim.load_form {
        LoadForm::Force => {
            ui.horizontal(|ui| {
                for (label, v) in [
                    ("X", &mut sim.force.x),
                    ("Y", &mut sim.force.y),
                    ("Z", &mut sim.force.z),
                ] {
                    ui.label(label);
                    ui.add(egui::DragValue::new(v).speed(1.0).suffix(" N"));
                }
            });
        }
        LoadForm::Pressure => {
            ui.horizontal(|ui| {
                ui.label("Pressure");
                ui.add(
                    egui::DragValue::new(&mut sim.pressure)
                        .speed(0.1)
                        .suffix(" MPa"),
                )
                .on_hover_text("Positive pushes into the face; negative pulls");
            });
        }
    }
}

fn material_ui(ui: &mut egui::Ui, sim: &mut Simulation) {
    let mut preset = Preset::of(sim.material);
    ui.horizontal(|ui| {
        ui.label("Material");
        egui::ComboBox::from_id_salt("simulate-material")
            .selected_text(preset.name())
            .show_ui(ui, |ui| {
                for p in Preset::NAMED {
                    ui.selectable_value(&mut preset, p, p.name());
                }
            });
    });
    if let Some(m) = preset.material() {
        sim.material = m;
    }
    ui.horizontal(|ui| {
        ui.label("Young's modulus");
        ui.add(
            egui::DragValue::new(&mut sim.material.youngs_modulus)
                .speed(1000.0)
                .range(1.0..=f64::INFINITY)
                .suffix(" MPa"),
        );
    });
    ui.horizontal(|ui| {
        ui.label("Poisson's ratio");
        ui.add(
            egui::DragValue::new(&mut sim.material.poisson_ratio)
                .speed(0.005)
                .range(0.0..=0.499),
        );
    });
}

fn results_ui(ui: &mut egui::Ui, sim: &mut Simulation) {
    let Some(outcome) = sim.outcome.as_ref() else {
        return;
    };
    let results = &outcome.results;
    let (max_disp, _) = results.max_displacement();
    let (max_vm, _) = results.max_von_mises();
    let r = results.reaction;
    ui.label(format!("Max displacement {max_disp:.4} mm"));
    ui.label(format!("Max von Mises {max_vm:.2} MPa"));
    ui.label(format!("Reaction {:.2}, {:.2}, {:.2} N", r.x, r.y, r.z));
    ui.label(format!(
        "{} elements, {} iterations",
        results.mesh.elements.len(),
        results.iterations
    ));
    let range = outcome.stress_range();
    // The slider runs to five times the default, which is about half the body: beyond
    // that the plot is a different shape and no longer says anything about this one.
    let top = (sim.scale * 5.0).max(1.0);
    ui.horizontal(|ui| {
        ui.label("Deformation ×");
        ui.add(egui::Slider::new(&mut sim.scale, 0.0..=top).logarithmic(false));
    });
    ui.checkbox(&mut sim.show, "Show results");
    ui.separator();
    legend(ui, range);
}

/// The colour bar: hot at the top, as every stress plot has it, with the plot's own ends
/// in MPa beside it.
fn legend(ui: &mut egui::Ui, (lo, hi): (f64, f64)) {
    const STEPS: usize = 32;
    const SIZE: egui::Vec2 = egui::vec2(18.0, 120.0);
    ui.horizontal(|ui| {
        let (rect, _) = ui.allocate_exact_size(SIZE, egui::Sense::hover());
        let painter = ui.painter();
        let step = rect.height() / STEPS as f32;
        for i in 0..STEPS {
            // Band `i` from the top is the hot end; its colour is read at its middle.
            let t = 1.0 - (i as f32 + 0.5) / STEPS as f32;
            let [r, g, b] = stress_ramp(t);
            let band = egui::Rect::from_min_size(
                egui::pos2(rect.left(), rect.top() + i as f32 * step),
                egui::vec2(rect.width(), step + 0.5),
            );
            // The ramp is linear RGB, as the renderer takes it; egui paints sRGB, and the
            // conversion is what makes the bar the same colours as the plot.
            painter.rect_filled(band, 0.0, egui::Rgba::from_rgb(r, g, b));
        }
        painter.rect_stroke(
            rect,
            0.0,
            ui.style().visuals.widgets.noninteractive.bg_stroke,
            egui::StrokeKind::Inside,
        );
        ui.vertical(|ui| {
            ui.label(format!("{hi:.2} MPa"));
            ui.add_space(SIZE.y - 2.0 * ui.text_style_height(&egui::TextStyle::Body) - 8.0);
            ui.label(format!("{lo:.2} MPa"));
        });
    });
}

// --- The plot on the GPU -----------------------------------------------------------

/// The results mesh as uploaded: which run and at what scale, so it is re-uploaded only
/// when either changes.
pub(super) struct ResultsMesh {
    pub handle: MeshHandle,
    run: u64,
    scale: f64,
}

impl Editor {
    /// Keeps the GPU copy of the plot in step with the results and scale the dialog
    /// shows, and drops it when there is nothing to show. Called from
    /// [`Editor::sync_meshes`], which is where the device and queue are.
    pub(super) fn sync_results_mesh(
        &mut self,
        renderer: &mut basset_viewport::Renderer,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
    ) {
        let revision = self.doc.revision();
        let wanted = self
            .simulation
            .as_ref()
            .and_then(|s| s.plotted(revision).map(|(_, o)| (o.run, s.scale)));
        let current = self.results_mesh.as_ref().map(|m| (m.run, m.scale));
        if wanted == current {
            return;
        }
        if let Some(old) = self.results_mesh.take() {
            renderer.remove_mesh(old.handle);
        }
        let Some((run, scale)) = wanted else {
            return;
        };
        let Some(sim) = self.simulation.as_ref() else {
            return;
        };
        let Some(outcome) = sim.outcome.as_ref() else {
            return;
        };
        let (mesh, colors) = outcome.plot(scale);
        // The brick surface has no feature edges worth drawing: every facet boundary is
        // a stair step, and drawing them would show the mesh rather than the body.
        match renderer.upload_colored_mesh(device, queue, &mesh, &colors, &[]) {
            Ok(handle) => {
                self.results_mesh = Some(ResultsMesh { handle, run, scale });
            }
            Err(e) => log::error!("results mesh upload failed: {e}"),
        }
    }
}
