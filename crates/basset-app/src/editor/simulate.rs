//! The Simulation workspace: a linear elastic study of one body, set up from a study
//! tree and a settings panel, solved on a background thread and read as a stress plot in
//! the viewport.
//!
//! A study is not a timeline feature. It changes nothing about the model — the body is
//! the same body after a run as before — and a feature that produced no geometry would
//! sit in the timeline as a step that regenerates to nothing, costing an undo entry every
//! time a load was retyped. Like [`Measure`](super::measure::Measure) it is therefore
//! editor state: the editor holds one [`Simulation`] with the body, the faces picked for
//! it, the material and the loads, and the last [`Results`] together with the document
//! revision they were computed at. The revision is what keeps the plot honest: an edit
//! to the model — a fillet, an undo, a parameter — leaves results that describe a body
//! that no longer exists, and rather than keep showing them the panel marks them stale
//! and the viewport goes back to the plain body until the study is run again.
//!
//! The study lives in a *workspace* of its own rather than a dialog over the modelling
//! toolbar, the way Fusion separates Design from Simulation. Setting up a study and
//! modelling a part are different jobs that want different toolbars, different browsers
//! (a study tree in place of the component tree) and different meanings for a click on a
//! face, and a dialog that had to coexist with the modelling tools kept being closed by
//! them. The workspace is a property of the [`Editor`] ([`Workspace`]); the study it
//! shows survives a flip back to Design, so the user can go and edit the part and come
//! back to re-run, and only an opened or new document drops it.
//!
//! A study is of one of two kinds ([`StudyKind`]): a static stress study, which asks how
//! the body moves and where it is stressed, and a topology optimisation, which asks where
//! a given fraction of its material should go. Both are set up the same way — the same
//! held and loaded faces, material and brick size — and both end in a plot over the
//! brick mesh; the difference is what the plot is of (a deformed body coloured by stress,
//! or the kept elements coloured by density) and which numbers the readout leads with.
//!
//! A solve runs on its own thread. The default mesh solves in well under a second, but a
//! fine one or an optimisation can take tens of seconds, and a window that does not
//! repaint for that long is a window the user believes has hung. The thread gets the
//! body's `Arc<Solid>` and a copy of the [`Study`] and reports back over a channel: a
//! [`Progress`] every few dozen solver iterations and the answer at the end. The editor
//! drains the channel once per frame ([`Editor::poll_simulation`]) into the progress
//! text and bar and asks for another frame while the job is live, which is what animates
//! the spinner and the elapsed time. Stop raises a flag the solver's observer reads at
//! every report, so the thread gives up within a few iterations rather than solving to
//! the end for an answer nobody wants. The result is tagged with the document revision
//! captured at launch, so a model edited while the solver was running simply gets stale
//! results, by the same rule as an edit after a run.

use std::path::Path;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, mpsc};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use basset_core::BodyRef;
use basset_fea::{
    FeaError, Load, LoadKind, Material, MaterialGroup, MaterialSpec, Phase, Progress, Results,
    Study, TopologyResults, TopologyStudy,
};
use basset_kernel::{FaceKey, Solid};
use basset_math::{TriMesh, Vec3};
use basset_viewport::{MeshHandle, stress_ramp};

use super::commands::Command;
use super::selection::{Pick, SelectionFilter};
use super::{Editor, Workspace};

/// Which of the study's two face lists a click in the viewport goes to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Armed {
    Fixed,
    Load,
}

/// What a run asks of the solver.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StudyKind {
    /// How far the body moves and how hard it is stressed under the loads.
    Static,
    /// Where a fraction of the body's material should go to carry the loads stiffest.
    Topology,
}

impl StudyKind {
    pub const ALL: [StudyKind; 2] = [StudyKind::Static, StudyKind::Topology];

    pub fn name(self) -> &'static str {
        match self {
            StudyKind::Static => "Static stress",
            StudyKind::Topology => "Topology optimisation",
        }
    }
}

/// What the plot colours the surface by.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Quantity {
    /// Nodal von Mises stress, MPa. The default for a static study.
    VonMises,
    /// Displacement magnitude, mm.
    Displacement,
    /// Yield strength over von Mises, capped at [`SAFETY_FACTOR_CAP`]. Only offered when
    /// the material has a yield strength, and plotted on the inverted ramp so the hot
    /// end is the end about to fail.
    SafetyFactor,
    /// The optimiser's density per element. Only offered for a topology outcome, and
    /// its default.
    Density,
}

/// Where the safety factor plot stops caring: a part ten times stronger than it needs to
/// be is as safe as one a hundred times, and letting the unstressed corners run to
/// infinity would put the whole loaded region in one colour.
pub const SAFETY_FACTOR_CAP: f64 = 10.0;

impl Quantity {
    pub const ALL: [Quantity; 4] = [
        Quantity::VonMises,
        Quantity::Displacement,
        Quantity::SafetyFactor,
        Quantity::Density,
    ];

    pub fn name(self) -> &'static str {
        match self {
            Quantity::VonMises => "Von Mises stress",
            Quantity::Displacement => "Displacement",
            Quantity::SafetyFactor => "Safety factor",
            Quantity::Density => "Density",
        }
    }

    /// The unit the legend is labelled in; empty for a ratio.
    pub fn unit(self) -> &'static str {
        match self {
            Quantity::VonMises => "MPa",
            Quantity::Displacement => "mm",
            Quantity::SafetyFactor | Quantity::Density => "",
        }
    }

    /// Whether the cold end of the ramp is the high value: true for the safety factor,
    /// where small is dangerous.
    pub fn inverted(self) -> bool {
        self == Quantity::SafetyFactor
    }

    /// Whether this quantity can be plotted over an outcome of the given shape.
    pub fn available(self, topology: bool, has_yield: bool) -> bool {
        match self {
            Quantity::VonMises | Quantity::Displacement => true,
            Quantity::SafetyFactor => has_yield,
            Quantity::Density => topology,
        }
    }

    /// A value as the legend prints it.
    pub fn format(self, v: f64) -> String {
        match self {
            Quantity::VonMises => format!("{v:.2} MPa"),
            Quantity::Displacement => format!("{v:.4} mm"),
            Quantity::SafetyFactor if v >= SAFETY_FACTOR_CAP => format!("≥ {SAFETY_FACTOR_CAP:.0}"),
            Quantity::SafetyFactor | Quantity::Density => format!("{v:.2}"),
        }
    }
}

/// How the load faces are loaded: one total force shared over them, or a pressure.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LoadForm {
    Force,
    Pressure,
}

/// Everything the plot of an outcome depends on besides the outcome itself. The GPU copy
/// is keyed on it, so a change to any field — and nothing else — re-uploads.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PlotSpec {
    pub quantity: Quantity,
    /// How many times the displacement is exaggerated. Ignored for a topology outcome,
    /// whose surface is drawn undeformed: the shape *is* the answer there.
    pub scale: f64,
    /// The density at and above which a topology outcome's element is kept.
    pub threshold: f64,
    /// The material's yield strength, which the safety factor is measured against.
    pub yield_strength: Option<f64>,
}

/// A run's results and the document they describe.
#[derive(Clone, Debug)]
pub struct Outcome {
    /// The static results: of the study itself, or of the final design of a topology
    /// optimisation, so every reader of stresses and displacements reads this one field.
    pub results: Results,
    /// The optimiser's densities and history when the run was a topology study.
    pub topology: Option<TopologyResults>,
    /// `Document::revision` when the study ran. Any edit since moves it, and the results
    /// are then of a body the model no longer has.
    pub revision: u64,
    /// Which run this is, counted per simulation. The GPU copy of the plot is keyed on
    /// it, so a re-run with the same scale still re-uploads.
    pub run: u64,
    /// How long the solve took, for the message and the study tree.
    pub took: Duration,
}

impl Outcome {
    /// The range of the nodal von Mises stress: what a von Mises plot is coloured by.
    pub fn stress_range(&self) -> (f64, f64) {
        value_range(self.results.nodal_von_mises.iter().copied())
    }

    /// The range of the plotted quantity, so the legend's ends are the plot's own.
    /// Taken over the nodes (or, for a topology outcome, the kept elements) rather than
    /// the surface, which is what the plot's colours are scaled against too.
    pub fn range(&self, spec: &PlotSpec) -> (f64, f64) {
        let r = &self.results;
        let safety = |vm: f64| safety_value(spec.yield_strength, vm);
        match &self.topology {
            Some(t) => {
                let kept = (0..t.densities.len()).filter(|&e| t.densities[e] >= spec.threshold);
                match spec.quantity {
                    Quantity::Density => value_range(kept.map(|e| t.densities[e])),
                    Quantity::VonMises => value_range(kept.map(|e| r.von_mises[e])),
                    Quantity::SafetyFactor => value_range(kept.map(|e| safety(r.von_mises[e]))),
                    Quantity::Displacement => value_range(kept.map(|e| element_displacement(r, e))),
                }
            }
            None => match spec.quantity {
                Quantity::VonMises | Quantity::Density => self.stress_range(),
                Quantity::Displacement => value_range(r.displacements.iter().map(|d| d.length())),
                Quantity::SafetyFactor => {
                    value_range(r.nodal_von_mises.iter().map(|&vm| safety(vm)))
                }
            },
        }
    }

    /// The surface the plot is drawn on with the plotted quantity at every vertex, in
    /// triangle order: the deformed skin of a static study, or the kept elements of a
    /// topology study, undeformed.
    pub fn surface(&self, spec: &PlotSpec) -> (TriMesh, Vec<f64>) {
        let r = &self.results;
        let safety = |vm: f64| safety_value(spec.yield_strength, vm);
        match &self.topology {
            Some(t) => {
                let (mesh, density) = t.surface(spec.threshold);
                // The kept surface knows which element each triangle belongs to but not
                // which nodes, so everything but the density is read per element and
                // painted flat over the triangle; a brick is one stress sample anyway.
                let per_element = |f: &dyn Fn(usize) -> f64| -> Vec<f64> {
                    mesh.face_ids
                        .iter()
                        .flat_map(|&e| std::iter::repeat_n(f(e as usize), 3))
                        .collect()
                };
                let values = match spec.quantity {
                    Quantity::Density => density,
                    Quantity::VonMises => per_element(&|e| r.von_mises[e]),
                    Quantity::SafetyFactor => per_element(&|e| safety(r.von_mises[e])),
                    Quantity::Displacement => per_element(&|e| element_displacement(r, e)),
                };
                (mesh, values)
            }
            None => {
                let (mesh, vm) = r.deformed_surface(spec.scale);
                let values = match spec.quantity {
                    Quantity::VonMises | Quantity::Density => vm,
                    Quantity::SafetyFactor => vm.into_iter().map(safety).collect(),
                    Quantity::Displacement => {
                        nodal_surface_values(r, |n| r.displacements[n].length())
                    }
                };
                (mesh, values)
            }
        }
    }

    /// The plotted surface with one colour per vertex, ready for the renderer.
    pub fn plot(&self, spec: &PlotSpec) -> (TriMesh, Vec<[f32; 3]>) {
        let (mesh, values) = self.surface(spec);
        let colors = plot_colors(&values, self.range(spec), spec.quantity.inverted());
        (mesh, colors)
    }

    /// Mass of what the outcome describes in kilograms, at a density in g/cm³: the whole
    /// mesh for a static study, the kept elements for a topology one.
    pub fn mass_kg(&self, density: f64, threshold: f64) -> f64 {
        match &self.topology {
            Some(t) => basset_fea::materials::mass_kg(density, t.kept_volume(threshold)),
            None => self.results.mass_kg(density),
        }
    }
}

/// The smallest and largest of some values, or a flat `(0, 0)` for none.
fn value_range(values: impl Iterator<Item = f64>) -> (f64, f64) {
    let (lo, hi) = values.fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), v| {
        (lo.min(v), hi.max(v))
    });
    if lo > hi { (0.0, 0.0) } else { (lo, hi) }
}

/// The safety factor plotted for a stress: capped, and the cap itself when the material
/// has no yield strength, so a plot asked for anyway is uniformly safe rather than NaN.
fn safety_value(yield_strength: Option<f64>, von_mises: f64) -> f64 {
    yield_strength.map_or(SAFETY_FACTOR_CAP, |y| {
        basset_fea::safety_factor(y, von_mises).min(SAFETY_FACTOR_CAP)
    })
}

/// Displacement magnitude at an element's centre: the mean of its corners'.
fn element_displacement(r: &Results, e: usize) -> f64 {
    let nodes = &r.mesh.elements[e];
    nodes
        .iter()
        .map(|&n| r.displacements[n])
        .sum::<Vec3>()
        .length()
        / nodes.len() as f64
}

/// A nodal value at every vertex of [`Results::deformed_surface`], in its order: every
/// facet as two triangles `[0, 1, 2]` and `[0, 2, 3]` of its corners. The walk is the
/// surface's own, repeated here because the surface hands back positions, not node
/// indexes, and the plot wants a value the solver did not pick for it.
fn nodal_surface_values(r: &Results, per_node: impl Fn(usize) -> f64) -> Vec<f64> {
    let mut values = Vec::with_capacity(r.mesh.facets.len() * 6);
    for facet in &r.mesh.facets {
        for tri in [[0, 1, 2], [0, 2, 3]] {
            values.extend(tri.iter().map(|&k| per_node(facet.nodes[k])));
        }
    }
    values
}

/// The ramp applied to each value, with a flat plot — every node at the same stress —
/// reading as the cold end rather than dividing by zero. `inverted` runs the ramp the
/// other way, for a quantity whose low end is the dangerous one.
pub fn plot_colors(values: &[f64], (lo, hi): (f64, f64), inverted: bool) -> Vec<[f32; 3]> {
    let span = hi - lo;
    values
        .iter()
        .map(|&v| {
            let t = if span > 0.0 { (v - lo) / span } else { 0.0 };
            let t = if inverted { 1.0 - t } else { t };
            stress_ramp(t as f32)
        })
        .collect()
}

/// What the solver thread sends back: where it has got to, and finally what it found.
enum Message {
    Progress(Progress),
    /// Boxed because it is a whole mesh beside a report of four words: the channel
    /// carries thousands of the small kind for one of the large.
    Done(Box<Result<Answer, FeaError>>),
}

enum Answer {
    Static(Results),
    Topology(TopologyResults),
}

/// A solve in flight on its own thread.
///
/// The editor never joins the thread: it reads the channel once a frame and, when told
/// to stop, drops the job, which raises the cancel flag and drops the receiver. The
/// solver's observer returns `false` at its next report and `run_with` comes back with
/// [`FeaError::Cancelled`], which the thread tries to send to a receiver that is gone and
/// then exits. Tests that want to know the thread has gone take the handle from
/// [`Simulation::stop_job`] and join it.
#[derive(Debug)]
pub struct Job {
    pub started: Instant,
    /// The document revision the solid was taken at, which the results inherit.
    pub revision: u64,
    receiver: mpsc::Receiver<Message>,
    pub cancelled: Arc<AtomicBool>,
    /// What the solver last said it was doing, shown under the spinner.
    pub progress: Option<String>,
    /// How far through it is, when that is known: an optimisation knows how many
    /// iterations it will take; a conjugate gradient does not know when it will converge.
    pub fraction: Option<f32>,
    thread: Option<JoinHandle<()>>,
}

impl std::fmt::Debug for Message {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Message::Progress(p) => write!(f, "Progress({p:?})"),
            Message::Done(answer) => match answer.as_ref() {
                Ok(_) => write!(f, "Done(Ok)"),
                Err(e) => write!(f, "Done(Err({e:?}))"),
            },
        }
    }
}

impl Job {
    pub fn elapsed(&self) -> Duration {
        self.started.elapsed()
    }

    /// Takes a report on board: the text for the spinner and, when the phase knows its
    /// length, the bar. A solve inside an optimisation leaves the bar where the last
    /// finished iteration put it rather than taking it away.
    fn note(&mut self, p: &Progress) {
        self.progress = Some(progress_text(p));
        if let Some(f) = progress_fraction(p) {
            self.fraction = Some(f);
        }
    }
}

impl Drop for Job {
    fn drop(&mut self) {
        self.cancelled.store(true, Ordering::Relaxed);
    }
}

/// A report as the panel shows it.
pub fn progress_text(p: &Progress) -> String {
    match p.phase {
        Phase::Meshing => "Meshing…".to_owned(),
        Phase::Solving => format!(
            "Solving: {} iterations, residual {:.1e}",
            thousands(p.step),
            p.measure
        ),
        Phase::Optimising => match p.of {
            Some(of) => format!(
                "Optimising: iteration {} of {}, compliance {:.3}",
                p.step, of, p.measure
            ),
            None => format!(
                "Optimising: iteration {}, compliance {:.3}",
                p.step, p.measure
            ),
        },
    }
}

/// How far along a report says the job is, for the phases that know.
fn progress_fraction(p: &Progress) -> Option<f32> {
    match (p.phase, p.of) {
        (Phase::Optimising, Some(of)) if of > 0 => Some((p.step as f32 / of as f32).min(1.0)),
        _ => None,
    }
}

/// `1250` as `1 250`: a thin space between groups, the way the rest of the UI writes
/// large counts.
fn thousands(n: usize) -> String {
    let digits = n.to_string();
    let mut out = String::with_capacity(digits.len() + digits.len() / 3);
    for (i, c) in digits.chars().enumerate() {
        if i > 0 && (digits.len() - i).is_multiple_of(3) {
            out.push(' ');
        }
        out.push(c);
    }
    out
}

/// The whole study: what the user has set up, what is running, and what the last run
/// said.
#[derive(Debug)]
pub struct Simulation {
    pub kind: StudyKind,
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
    /// What the solver reads. Editable number by number; see `material_spec`.
    pub material: Material,
    /// The library entry the material was chosen from, kept when E or ν is typed over so
    /// the density and yield strength still have a source: a user who nudges the modulus
    /// of 6061-T6 is still weighing aluminium.
    pub material_spec: Option<&'static MaterialSpec>,
    /// What the material combo's filter box holds. UI state, but the combo is laid out
    /// afresh every frame and has nowhere else to keep it.
    pub material_filter: String,
    /// Target brick size in millimetres.
    pub element_size: f64,
    /// Topology: the fraction of the mesh the design may use.
    pub volume_fraction: f64,
    /// Topology: optimiser iterations.
    pub iterations: usize,
    /// Topology: the density at and above which an element is shown and counted.
    pub threshold: f64,
    pub outcome: Option<Outcome>,
    /// The solve in flight, if one is.
    pub job: Option<Job>,
    /// How many times the displacement is exaggerated in the plot.
    pub scale: f64,
    /// Whether the plot replaces the body in the viewport.
    pub show: bool,
    /// What the plot is coloured by. See [`Simulation::quantity_shown`] for what is
    /// actually drawn when this cannot be.
    pub quantity: Quantity,
    /// What the panel says under the Run button: the last error verbatim, or what the
    /// run came to.
    pub message: String,
    runs: u64,
}

/// What a click may land on in the Simulation workspace: faces, which are the only
/// thing a study is written on.
pub const FILTER: SelectionFilter = SelectionFilter {
    faces: true,
    edges: false,
    vertices: false,
    points: false,
    planes: false,
    profiles: false,
    curves: false,
};

pub const PROMPT: &str = "Simulation: arm Fixed faces or Loads, then click faces of the body to hold or load. \
     The Design tab goes back to modelling.";

/// What a modelling command is told in the Simulation workspace.
pub const REFUSED: &str = "Modelling is done in the Design workspace: click its tab";

/// What the panel says once a run has been stopped, in place of the solver's error.
pub const STOPPED: &str = "Stopped";

/// The default brick: a twentieth of the body's longest side, which gives a few thousand
/// elements on any proportioned part and solves while the user watches.
const BRICKS_ALONG_LONGEST: f64 = 20.0;
/// How far the deformed plot moves at its default scale, as a fraction of the body's
/// longest side: visible without reading as a different shape.
const DEFAULT_PLOT_TRAVEL: f64 = 0.1;

impl Simulation {
    fn new(body: Option<BodyRef>, longest: Option<f64>) -> Self {
        let defaults = TopologyStudy::new(Study {
            material: Material::STEEL,
            fixed: Vec::new(),
            loads: Vec::new(),
            element_size: 1.0,
        });
        Self {
            kind: StudyKind::Static,
            body,
            fixed: Vec::new(),
            loaded: Vec::new(),
            armed: Armed::Fixed,
            load_form: LoadForm::Force,
            force: Vec3::new(0.0, 0.0, -100.0),
            pressure: 1.0,
            material: Material::STEEL,
            material_spec: MaterialSpec::of(Material::STEEL),
            material_filter: String::new(),
            element_size: longest.map_or(5.0, |l| l / BRICKS_ALONG_LONGEST),
            volume_fraction: defaults.volume_fraction,
            iterations: defaults.iterations,
            threshold: 0.5,
            outcome: None,
            job: None,
            scale: 1.0,
            show: true,
            quantity: Quantity::VonMises,
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

    /// The study as the optimiser takes it: the static study with the two settings the
    /// panel exposes; penalty, filter and floor stay at the library's defaults.
    pub fn topology_study(&self) -> TopologyStudy {
        TopologyStudy {
            volume_fraction: self.volume_fraction,
            iterations: self.iterations,
            ..TopologyStudy::new(self.study())
        }
    }

    /// Takes a library material: the solver's numbers and the data sheet behind them.
    pub fn choose_material(&mut self, spec: &'static MaterialSpec) {
        self.material = spec.material;
        self.material_spec = Some(spec);
    }

    /// The name the material shows under: the chosen entry while its numbers stand, else
    /// whatever entry the numbers happen to be, else "Custom".
    pub fn material_name(&self) -> &'static str {
        match self.material_spec {
            Some(spec) if spec.material == self.material => spec.name,
            _ => MaterialSpec::of(self.material).map_or("Custom", |s| s.name),
        }
    }

    /// Density in g/cm³, from the chosen entry.
    pub fn density(&self) -> Option<f64> {
        self.material_spec.map(|s| s.density)
    }

    /// Yield strength in MPa, from the chosen entry, where it has one.
    pub fn yield_strength(&self) -> Option<f64> {
        self.material_spec.and_then(|s| s.yield_strength)
    }

    /// Whether a solve is running.
    pub fn solving(&self) -> bool {
        self.job.is_some()
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

    /// Whether the last outcome is a topology optimisation's.
    pub fn has_topology(&self) -> bool {
        self.outcome.as_ref().is_some_and(|o| o.topology.is_some())
    }

    /// The quantity the plot is actually coloured by: the chosen one when the outcome
    /// and material allow it, else the kind's default. A material swapped for one with
    /// no yield strength after a safety factor plot falls back to stress rather than to
    /// an error.
    pub fn quantity_shown(&self) -> Quantity {
        let topology = self.has_topology();
        if self
            .quantity
            .available(topology, self.yield_strength().is_some())
        {
            self.quantity
        } else if topology {
            Quantity::Density
        } else {
            Quantity::VonMises
        }
    }

    /// Everything the plot of the current outcome depends on.
    pub fn plot_spec(&self) -> PlotSpec {
        PlotSpec {
            quantity: self.quantity_shown(),
            scale: self.scale,
            threshold: self.threshold,
            yield_strength: self.yield_strength(),
        }
    }

    pub fn armed_list(&mut self) -> &mut Vec<FaceKey> {
        match self.armed {
            Armed::Fixed => &mut self.fixed,
            Armed::Load => &mut self.loaded,
        }
    }

    /// Abandons the solve in flight, if there is one, and hands back its thread for
    /// anyone who wants to wait for it to notice; the UI never does. See [`Job`].
    pub fn stop_job(&mut self) -> Option<JoinHandle<()>> {
        let mut job = self.job.take()?;
        self.message = STOPPED.into();
        job.thread.take()
    }
}

/// The longest side of a body's bounding box, what the defaults are measured against.
fn longest_side(solid: &Solid) -> Option<f64> {
    let aabb = solid.aabb();
    (!aabb.is_empty()).then(|| aabb.extent().max_element())
}

fn solid_of(editor: &mut Editor, body: BodyRef) -> Option<Arc<Solid>> {
    editor.doc.state().body(body).map(|b| b.solid.clone())
}

/// What entering the Simulation workspace does: puts down whatever modelling was in
/// progress and makes sure there is a study, keeping the one there is if the user has
/// been here before. Called by [`Editor::set_workspace`] after the workspace is set.
pub fn enter(editor: &mut Editor) {
    if editor.tool.is_some() {
        super::tools::cancel_tool(editor);
    }
    super::measure::stop(editor);
    if editor.simulation.is_none() {
        // One body selected is the body; so is the only body there is. Otherwise the
        // first face picked decides, since a study with no body cannot take a face.
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
        editor.simulation = Some(Simulation::new(body, longest));
    }
    editor.selection.clear();
    editor.hover = None;
    editor.set_status(PROMPT);
    editor.request_repaint();
}

/// What going back to Design does: nothing to the study, which waits for the next visit
/// with its results; only the hover, which was a face of a study and is now a face of
/// the model.
pub fn leave(editor: &mut Editor) {
    editor.hover = None;
    editor.set_status("Design workspace");
    editor.request_repaint();
}

/// Drops the study altogether, with any solve in flight. A new or opened document is of
/// other bodies; this is what they call.
pub fn stop(editor: &mut Editor) {
    if editor.simulation.take().is_some() {
        editor.request_repaint();
    }
}

/// Records a click in the Simulation workspace. Returns whether the click was consumed;
/// in that workspace it always is, so picking for a study never doubles as a selection
/// some later tool acts on.
pub fn clicked(editor: &mut Editor, pick: Option<&Pick>) -> bool {
    if editor.workspace != Workspace::Simulation || editor.simulation.is_none() {
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

/// Starts solving the study as it stands, on a thread of its own. The answer arrives
/// through [`Editor::poll_simulation`]; what is known straight away — no body, a body
/// that has gone, a thread that would not start — is said at once.
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
    // A second Run while one is solving starts over: the user changed something and
    // wants the answer to that, not to what they had before.
    sim.stop_job();
    let kind = sim.kind;
    let study = sim.study();
    let topology = sim.topology_study();
    let cancelled = Arc::new(AtomicBool::new(false));
    let (sender, receiver) = mpsc::channel();
    let flag = cancelled.clone();
    let spawned = std::thread::Builder::new()
        .name("fea-solve".into())
        .spawn(move || {
            // The observer is the cancellation seam: every report is a chance to stop,
            // and the first comes before the mesh is built, so a job stopped at once
            // does no work at all. A send to a dropped receiver is the job having been
            // stopped, and nothing to act on: the flag does that.
            let mut observer = |p: &Progress| {
                let _ = sender.send(Message::Progress(*p));
                !flag.load(Ordering::Relaxed)
            };
            let answer = match kind {
                StudyKind::Static => {
                    basset_fea::run_with(&solid, &study, &mut observer).map(Answer::Static)
                }
                StudyKind::Topology => basset_fea::optimise_with(&solid, &topology, &mut observer)
                    .map(Answer::Topology),
            };
            let _ = sender.send(Message::Done(Box::new(answer)));
        });
    match spawned {
        Ok(thread) => {
            sim.job = Some(Job {
                started: Instant::now(),
                revision,
                receiver,
                cancelled,
                progress: None,
                fraction: None,
                thread: Some(thread),
            });
            sim.message = match kind {
                StudyKind::Static => "Solving…",
                StudyKind::Topology => "Optimising…",
            }
            .into();
        }
        Err(e) => {
            sim.message = format!("Could not start the solver thread: {e}");
        }
    }
    editor.request_repaint();
}

/// Abandons the solve in flight.
pub fn stop_run(editor: &mut Editor) {
    if let Some(sim) = editor.simulation.as_mut() {
        sim.stop_job();
        editor.set_status("Solve stopped");
        editor.request_repaint();
    }
}

impl Editor {
    /// Reads what the solver has sent — progress reports, and the answer if it has
    /// arrived — and keeps the window awake while it has not. Called once per frame
    /// from [`Editor::ui`]; headless, the harness calls it in a loop.
    pub fn poll_simulation(&mut self) {
        let Some(sim) = self.simulation.as_mut() else {
            return;
        };
        let Some(job) = sim.job.as_mut() else {
            return;
        };
        // Everything queued since the last frame, in order: a solve reports faster than
        // the window paints, and only the last report is worth showing.
        let received = loop {
            match job.receiver.try_recv() {
                Ok(Message::Progress(p)) => job.note(&p),
                Ok(Message::Done(answer)) => break (*answer).map_err(Some),
                Err(mpsc::TryRecvError::Empty) => {
                    // The spinner and the clock move only if the next frame comes.
                    self.request_repaint();
                    return;
                }
                Err(mpsc::TryRecvError::Disconnected) => {
                    // The thread ended without an answer: it panicked.
                    break Err(None);
                }
            }
        };
        let job = sim.job.take().expect("checked above");
        let longest = sim
            .body
            .and_then(|b| self.doc.state().body(b).map(|b| b.solid.clone()))
            .and_then(|s| longest_side(&s))
            .unwrap_or(1.0);
        let Some(sim) = self.simulation.as_mut() else {
            return;
        };
        match received {
            Ok(answer) => {
                let (results, topology) = match answer {
                    Answer::Static(r) => (r, None),
                    Answer::Topology(t) => (t.results.clone(), Some(t)),
                };
                let (max_disp, _) = results.max_displacement();
                let (max_vm, _) = results.max_von_mises();
                // The default exaggeration puts the largest movement at a tenth of the
                // body: a real displacement of microns would otherwise plot as no
                // movement at all.
                sim.scale = if max_disp > 0.0 {
                    DEFAULT_PLOT_TRAVEL * longest / max_disp
                } else {
                    1.0
                };
                sim.runs += 1;
                let took = job.elapsed();
                sim.message = match &topology {
                    Some(t) => format!(
                        "Optimised {} elements in {} iterations ({:.1} s): volume fraction {:.2}, compliance {:.4} → {:.4}",
                        results.mesh.elements.len(),
                        t.iterations,
                        took.as_secs_f64(),
                        t.volume_fraction,
                        t.compliance.first().copied().unwrap_or(0.0),
                        t.compliance.last().copied().unwrap_or(0.0)
                    ),
                    None => format!(
                        "Solved {} elements in {} iterations ({:.1} s): max displacement {:.4} mm, max von Mises {:.2} MPa",
                        results.mesh.elements.len(),
                        results.iterations,
                        took.as_secs_f64(),
                        max_disp,
                        max_vm
                    ),
                };
                // Each kind has a quantity that is the point of it; the other kind
                // cannot show it, so a change of kind changes the plot rather than
                // falling back silently.
                sim.quantity = match &topology {
                    Some(_) => Quantity::Density,
                    None if sim.quantity == Quantity::Density => Quantity::VonMises,
                    None => sim.quantity,
                };
                sim.outcome = Some(Outcome {
                    results,
                    topology,
                    revision: job.revision,
                    run: sim.runs,
                    took,
                });
                sim.show = true;
            }
            Err(e) => {
                // A stop is the user's doing and reads as such, not as the solver
                // failing; it only arrives this way when the flag was raised without
                // the job being dropped, since dropping it drops the receiver too.
                sim.message = match e {
                    Some(FeaError::Cancelled) => STOPPED.to_owned(),
                    Some(e) => error_text(&e),
                    None => "the solver thread stopped without an answer".to_owned(),
                };
                sim.outcome = None;
            }
        }
        self.request_repaint();
    }

    /// The study as the viewport and the pick path see it: only in the Simulation
    /// workspace. In Design the study is kept but says nothing about what is drawn or
    /// what a click means.
    pub(super) fn study_view(&self) -> Option<&Simulation> {
        match self.workspace {
            Workspace::Simulation => self.simulation.as_ref(),
            Workspace::Design | Workspace::Render => None,
        }
    }
}

/// The solver's message as it wrote it. A separate function only so the panel and the
/// tests agree on what is shown.
pub fn error_text(e: &FeaError) -> String {
    e.to_string()
}

/// A face as the study tree lists it. The key's parts are what the user can match
/// against the timeline: the feature that made the face and which of its faces it is.
fn face_label(key: &FaceKey) -> String {
    format!("{:?} of feature {}", key.role, key.op.feature)
}

/// A mass as the readout prints it: grams for small parts, kilograms otherwise.
pub fn mass_text(kg: f64) -> String {
    if kg < 1.0 {
        format!("{:.1} g", kg * 1000.0)
    } else {
        format!("{kg:.3} kg")
    }
}

// --- Export ---------------------------------------------------------------------------

/// Writes the last results as legacy VTK to a file the user picks.
pub fn export_vtk(editor: &mut Editor) {
    if editor
        .simulation
        .as_ref()
        .and_then(|s| s.outcome.as_ref())
        .is_none()
    {
        editor.set_status("Run the study before exporting its results");
        return;
    }
    let Some(path) = rfd::FileDialog::new()
        .add_filter("VTK legacy", &["vtk"])
        .set_file_name(format!("{}-study.vtk", editor.doc.name))
        .save_file()
    else {
        return;
    };
    export_vtk_to(editor, &path);
}

/// The file half of [`export_vtk`], which the tests can reach without a dialog. A
/// topology outcome writes the whole grid with a density per cell, so the threshold can
/// be chosen again in the viewer.
pub fn export_vtk_to(editor: &mut Editor, path: &Path) {
    let written = match editor.simulation.as_ref().and_then(|s| s.outcome.as_ref()) {
        Some(Outcome {
            topology: Some(t), ..
        }) => basset_fea::vtk::write_topology_file(t, path),
        Some(outcome) => basset_fea::vtk::write_file(&outcome.results, path),
        None => {
            editor.set_status("Run the study before exporting its results");
            return;
        }
    };
    match written {
        Ok(()) => editor.set_status(format!("Wrote {}", path.display())),
        Err(e) => editor.report_error(format!("could not write {}: {e}", path.display())),
    }
    editor.request_repaint();
}

// --- The workspace's panels -----------------------------------------------------------
//
// These take `&mut Editor` the way the browser and the sketch palette do: a study's
// settings are editor state, not document state, so writing them from a widget opens no
// transaction and needs no command. What *does* something — a solve, an export, a change
// of workspace — is still queued as a `Command` and run after the frame.

/// The Simulation workspace's toolbar: Fusion's Study / Solve / Results groups.
pub(super) fn toolbar(editor: &mut Editor, ui: &mut egui::Ui, commands: &mut Vec<Command>) {
    let revision = editor.doc.revision();
    ui.horizontal_wrapped(|ui| {
        let Some(sim) = editor.simulation.as_mut() else {
            ui.label(egui::RichText::new("No study").weak());
            return;
        };
        ui.label(egui::RichText::new("Study").weak());
        kind_combo(ui, sim, "toolbar-kind");
        for (which, label, hint) in [
            (
                Armed::Fixed,
                "Fixed faces",
                "Clicks in the viewport pick the faces that are held",
            ),
            (
                Armed::Load,
                "Loads",
                "Clicks in the viewport pick the faces the load acts on",
            ),
        ] {
            if ui
                .selectable_label(sim.armed == which, label)
                .on_hover_text(hint)
                .clicked()
            {
                sim.armed = which;
            }
        }
        material_combo(ui, sim, "toolbar-material");
        ui.label("Mesh size");
        ui.add(
            egui::DragValue::new(&mut sim.element_size)
                .speed(0.1)
                .range(0.01..=f64::INFINITY)
                .suffix(" mm"),
        );
        if sim.kind == StudyKind::Topology {
            ui.label("Volume");
            ui.add(
                egui::DragValue::new(&mut sim.volume_fraction)
                    .speed(0.01)
                    .range(0.05..=0.95)
                    .fixed_decimals(2),
            )
            .on_hover_text("The fraction of the mesh the design may keep");
        }
        ui.separator();

        ui.label(egui::RichText::new("Solve").weak());
        solve_buttons(ui, sim, commands);
        ui.separator();

        ui.label(egui::RichText::new("Results").weak());
        let fresh = sim.outcome.is_some() && !sim.is_stale(revision);
        ui.add_enabled(fresh, egui::Checkbox::new(&mut sim.show, "Show plot"))
            .on_disabled_hover_text("Run the study for a plot of this model");
        ui.add_enabled_ui(fresh, |ui| quantity_combo(ui, sim, "toolbar-quantity"));
        if sim.has_topology() {
            ui.label("Threshold");
            ui.add_enabled(
                fresh,
                egui::DragValue::new(&mut sim.threshold)
                    .speed(0.01)
                    .range(0.0..=1.0)
                    .fixed_decimals(2),
            );
        } else {
            ui.label("Deformation ×");
            ui.add_enabled(
                fresh,
                egui::DragValue::new(&mut sim.scale)
                    .speed(0.1)
                    .range(0.0..=f64::INFINITY),
            );
        }
        if ui
            .add_enabled(sim.outcome.is_some(), egui::Button::new("Export VTK…"))
            .on_hover_text("Write the mesh, displacements and stresses as legacy VTK")
            .clicked()
        {
            commands.push(Command::ExportVtk);
        }
        ui.separator();
        if ui
            .button("Fit")
            .on_hover_text("Fit the model in the view")
            .clicked()
        {
            commands.push(Command::Fit);
        }
    });
}

/// Run and Stop, one enabled while the other is not, with the spinner, the clock and
/// the solver's last word between them while a solve is in flight.
fn solve_buttons(ui: &mut egui::Ui, sim: &Simulation, commands: &mut Vec<Command>) {
    let solving = sim.solving();
    if ui
        .add_enabled(!solving && sim.body.is_some(), egui::Button::new("Run"))
        .on_disabled_hover_text(if solving {
            "Solving"
        } else {
            "Pick a face of the body to study first"
        })
        .clicked()
    {
        commands.push(Command::RunStudy);
    }
    if ui.add_enabled(solving, egui::Button::new("Stop")).clicked() {
        commands.push(Command::StopStudy);
    }
    if let Some(job) = sim.job.as_ref() {
        ui.add(egui::Spinner::new().size(14.0));
        ui.label(format!("{:.1} s", job.elapsed().as_secs_f64()));
        if let Some(f) = job.fraction {
            ui.add(egui::ProgressBar::new(f).desired_width(80.0));
        }
        if let Some(p) = job.progress.as_ref() {
            ui.label(egui::RichText::new(p).weak());
        }
    }
}

/// The study tree the browser shows in place of the component tree: one row per thing
/// the study is made of, the way Fusion's Simulation browser lists them.
pub(super) fn study_tree(editor: &mut Editor, ui: &mut egui::Ui) {
    ui.heading(&editor.doc.name);
    let revision = editor.doc.revision();
    let body_name = editor
        .simulation
        .as_ref()
        .and_then(|s| s.body)
        .map(|b| editor.body_name(b));
    let Some(sim) = editor.simulation.as_mut() else {
        ui.label(egui::RichText::new("No study").weak());
        return;
    };
    egui::ScrollArea::vertical().show(ui, |ui| {
        ui.label(egui::RichText::new(format!("{} study", sim.kind.name())).strong());
        ui.label(match &body_name {
            Some(name) => format!("Body: {name}"),
            None => "Body: click a face to choose one".to_owned(),
        });
        let m = sim.material;
        ui.label(format!(
            "Material: {} (E {:.0} MPa, ν {:.2})",
            sim.material_name(),
            m.youngs_modulus,
            m.poisson_ratio
        ));
        face_rows(ui, &mut sim.fixed, "Fixed faces", "study-fixed");
        let load = match sim.load_form {
            LoadForm::Force => format!(
                "{:.1}, {:.1}, {:.1} N over",
                sim.force.x, sim.force.y, sim.force.z
            ),
            LoadForm::Pressure => format!("{:.2} MPa on", sim.pressure),
        };
        super::theme::section(format!("Loads ({})", sim.loaded.len()))
            .id_salt("study-loads")
            .default_open(true)
            .show(ui, |ui| {
                if sim.loaded.is_empty() {
                    ui.label(egui::RichText::new("none").weak());
                } else {
                    ui.label(egui::RichText::new(load).weak());
                }
                face_list_rows(ui, &mut sim.loaded);
            });
        let elements = sim
            .outcome
            .as_ref()
            .map(|o| format!(", {} elements last run", o.results.mesh.elements.len()))
            .unwrap_or_default();
        ui.label(format!("Mesh: {:.2} mm bricks{elements}", sim.element_size));
        if sim.kind == StudyKind::Topology {
            ui.label(format!(
                "Target: {:.0}% of the volume, {} iterations",
                sim.volume_fraction * 100.0,
                sim.iterations
            ));
        }
        let results = if let Some(job) = sim.job.as_ref() {
            format!("Results: solving… {:.1} s", job.elapsed().as_secs_f64())
        } else if let Some(outcome) = sim.outcome.as_ref() {
            if sim.is_stale(revision) {
                "Results: stale — the model has changed since this run".to_owned()
            } else if let Some(t) = outcome.topology.as_ref() {
                format!(
                    "Results: fresh — {:.0}% of the volume kept, compliance {:.4}",
                    t.volume_fraction * 100.0,
                    t.compliance.last().copied().unwrap_or(0.0)
                )
            } else {
                let (max_disp, _) = outcome.results.max_displacement();
                let (max_vm, _) = outcome.results.max_von_mises();
                format!("Results: fresh — {max_disp:.4} mm, {max_vm:.2} MPa")
            }
        } else {
            "Results: none yet".to_owned()
        };
        ui.label(results);
    });
}

/// A collapsing list of one face list's rows, each removable.
fn face_rows(ui: &mut egui::Ui, list: &mut Vec<FaceKey>, title: &str, salt: &str) {
    super::theme::section(format!("{title} ({})", list.len()))
        .id_salt(salt)
        .default_open(true)
        .show(ui, |ui| {
            if list.is_empty() {
                ui.label(egui::RichText::new("none").weak());
            }
            face_list_rows(ui, list);
        });
}

fn face_list_rows(ui: &mut egui::Ui, list: &mut Vec<FaceKey>) {
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

/// The Study side panel: every setting of the study, the Run button, and the readout.
pub(super) fn study_panel(editor: &mut Editor, ui: &mut egui::Ui, commands: &mut Vec<Command>) {
    ui.heading("Study");
    let revision = editor.doc.revision();
    let body_name = editor
        .simulation
        .as_ref()
        .and_then(|s| s.body)
        .map(|b| editor.body_name(b));
    let Some(sim) = editor.simulation.as_mut() else {
        ui.label(egui::RichText::new("No study").weak());
        return;
    };
    egui::ScrollArea::vertical().show(ui, |ui| {
        ui.horizontal(|ui| {
            kind_combo(ui, sim, "study-kind");
            match &body_name {
                Some(name) => ui.label(format!("of {name}")),
                None => ui.label("of: click a face"),
            };
        });
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
        if sim.kind == StudyKind::Topology {
            ui.horizontal(|ui| {
                ui.label("Volume fraction");
                ui.add(egui::Slider::new(&mut sim.volume_fraction, 0.05..=0.95).fixed_decimals(2))
                    .on_hover_text("The fraction of the meshed volume the design may keep");
            });
            ui.horizontal(|ui| {
                ui.label("Iterations");
                ui.add(egui::DragValue::new(&mut sim.iterations).range(1..=500));
            });
        }
        ui.separator();
        ui.horizontal(|ui| solve_buttons(ui, sim, commands));
        if !sim.message.is_empty() {
            if sim.outcome.is_some() || sim.solving() || sim.message == STOPPED {
                ui.label(egui::RichText::new(&sim.message).weak());
            } else {
                ui.colored_label(super::theme::ERROR, &sim.message);
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
    face_list_rows(ui, list);
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

/// Static or topology, as a combo so the toolbar stays one line.
fn kind_combo(ui: &mut egui::Ui, sim: &mut Simulation, salt: &str) {
    egui::ComboBox::from_id_salt(salt)
        .selected_text(sim.kind.name())
        .show_ui(ui, |ui| {
            for k in StudyKind::ALL {
                ui.selectable_value(&mut sim.kind, k, k.name());
            }
        });
}

/// The library combo alone, for the toolbar; the panel adds the two numbers under it.
/// Thirty-odd entries is too many for a flat list, so they come grouped by family with a
/// filter box at the top; the popup shows the entries whose name or family contains
/// what was typed.
fn material_combo(ui: &mut egui::Ui, sim: &mut Simulation, salt: &str) {
    let mut chosen: Option<&'static MaterialSpec> = None;
    egui::ComboBox::from_id_salt(salt)
        .selected_text(sim.material_name())
        .width(150.0)
        .show_ui(ui, |ui| {
            ui.add(
                egui::TextEdit::singleline(&mut sim.material_filter)
                    .hint_text("Filter")
                    .desired_width(140.0),
            );
            let filter = sim.material_filter.trim().to_lowercase();
            let shown = |m: &MaterialSpec| {
                filter.is_empty()
                    || m.name.to_lowercase().contains(&filter)
                    || m.group.name().to_lowercase().contains(&filter)
            };
            for group in MaterialGroup::ALL {
                let members = basset_fea::library()
                    .iter()
                    .filter(|m| m.group == group && shown(m));
                let mut first = true;
                for m in members {
                    if first {
                        ui.label(egui::RichText::new(group.name()).weak().small());
                        first = false;
                    }
                    let selected = sim.material_spec == Some(m) && sim.material == m.material;
                    let hint = format!(
                        "E {:.0} MPa, ν {:.2}, {:.2} g/cm³{}",
                        m.material.youngs_modulus,
                        m.material.poisson_ratio,
                        m.density,
                        m.yield_strength
                            .map(|y| format!(", yield {y:.0} MPa"))
                            .unwrap_or_default()
                    );
                    if ui
                        .selectable_label(selected, m.name)
                        .on_hover_text(hint)
                        .clicked()
                    {
                        chosen = Some(m);
                    }
                }
            }
        });
    if let Some(m) = chosen {
        sim.choose_material(m);
    }
}

fn material_ui(ui: &mut egui::Ui, sim: &mut Simulation) {
    ui.horizontal(|ui| {
        ui.label("Material");
        material_combo(ui, sim, "study-material");
    });
    // The two numbers share a row, and the rest of the data sheet is a hover: the panel
    // is the height of the window and the readout below has to fit under this.
    ui.horizontal(|ui| {
        ui.label("E").on_hover_text("Young's modulus");
        ui.add(
            egui::DragValue::new(&mut sim.material.youngs_modulus)
                .speed(1000.0)
                .range(1.0..=f64::INFINITY)
                .suffix(" MPa"),
        );
        ui.label("ν").on_hover_text("Poisson's ratio");
        ui.add(
            egui::DragValue::new(&mut sim.material.poisson_ratio)
                .speed(0.005)
                .range(0.0..=0.499),
        );
    })
    .response
    .on_hover_text(match sim.material_spec {
        Some(spec) => format!(
            "{}: density {:.2} g/cm³{}",
            spec.name,
            spec.density,
            spec.yield_strength
                .map(|y| format!(", yield {y:.0} MPa"))
                .unwrap_or_else(|| ", no yield strength".to_owned())
        ),
        None => "No library entry: no density or yield strength".to_owned(),
    });
}

/// The plotted quantity, offering only what this outcome and material can show.
fn quantity_combo(ui: &mut egui::Ui, sim: &mut Simulation, salt: &str) {
    let topology = sim.has_topology();
    let has_yield = sim.yield_strength().is_some();
    let mut quantity = sim.quantity_shown();
    egui::ComboBox::from_id_salt(salt)
        .selected_text(quantity.name())
        .show_ui(ui, |ui| {
            for q in Quantity::ALL {
                if q.available(topology, has_yield) {
                    ui.selectable_value(&mut quantity, q, q.name());
                }
            }
        });
    sim.quantity = quantity;
}

fn results_ui(ui: &mut egui::Ui, sim: &mut Simulation) {
    let Some(outcome) = sim.outcome.as_ref() else {
        return;
    };
    let spec = sim.plot_spec();
    let range = outcome.range(&spec);
    readings(ui, sim, outcome);
    legend(ui, spec.quantity, range);
    if outcome.topology.is_some() {
        ui.horizontal(|ui| {
            ui.label("Threshold");
            ui.add(egui::Slider::new(&mut sim.threshold, 0.0..=1.0).fixed_decimals(2))
                .on_hover_text("Elements at or above this density are kept");
        });
    } else {
        // The slider runs to five times the default, which is about half the body:
        // beyond that the plot is a different shape and no longer says anything about
        // this one.
        let top = (sim.scale * 5.0).max(1.0);
        ui.horizontal(|ui| {
            ui.label("Deformation ×");
            ui.add(egui::Slider::new(&mut sim.scale, 0.0..=top).logarithmic(false));
        });
    }
    ui.horizontal(|ui| {
        ui.label("Colour by");
        quantity_combo(ui, sim, "study-quantity");
        ui.checkbox(&mut sim.show, "Show results");
    });
}

/// The numbers a run came to, one per line.
fn readings(ui: &mut egui::Ui, sim: &Simulation, outcome: &Outcome) {
    let results = &outcome.results;
    let (max_disp, _) = results.max_displacement();
    let (max_vm, _) = results.max_von_mises();
    let r = results.reaction;
    if let Some(t) = outcome.topology.as_ref() {
        let kept = t.kept_volume(sim.threshold);
        let total = t.mesh.volume();
        let share = if total > 0.0 { kept / total } else { 0.0 };
        ui.label(format!(
            "Volume fraction {:.2} of a target {:.2}",
            t.volume_fraction, sim.volume_fraction
        ));
        ui.label(format!(
            "Compliance {:.4} → {:.4}",
            t.compliance.first().copied().unwrap_or(0.0),
            t.compliance.last().copied().unwrap_or(0.0)
        ));
        ui.label(format!(
            "Kept volume {:.0}% ({kept:.0} mm³) at the threshold",
            share * 100.0
        ));
    }
    ui.label(format!("Max displacement {max_disp:.4} mm"));
    ui.label(format!("Max von Mises {max_vm:.2} MPa"));
    ui.label(format!("Reaction {:.2}, {:.2}, {:.2} N", r.x, r.y, r.z));
    if let Some(density) = sim.density() {
        ui.label(format!(
            "Mass {}",
            mass_text(outcome.mass_kg(density, sim.threshold))
        ));
    }
    if let Some(yield_strength) = sim.yield_strength() {
        let factor = basset_fea::safety_factor(yield_strength, max_vm);
        let text = if factor.is_infinite() {
            "Safety factor ∞ (nothing is stressed)".to_owned()
        } else {
            format!("Safety factor {factor:.2} against yield at {yield_strength:.0} MPa")
        };
        // Below one the part yields; below two it is closer to yielding than most codes
        // allow. The colours say so before the number is read.
        if factor < 1.0 {
            ui.colored_label(egui::Color32::from_rgb(230, 80, 70), text);
        } else if factor < 2.0 {
            ui.colored_label(egui::Color32::from_rgb(230, 180, 90), text);
        } else {
            ui.label(text);
        }
    }
    ui.label(format!(
        "{} elements, {} iterations{}, {:.1} s",
        results.mesh.elements.len(),
        results.iterations,
        outcome
            .topology
            .as_ref()
            .map(|t| format!(" ({} optimiser steps)", t.iterations))
            .unwrap_or_default(),
        outcome.took.as_secs_f64()
    ));
}

/// The colour bar: a strip across the panel, hot at the right, with the plot's own ends
/// beneath it in the quantity's unit. It lies down rather than standing because the
/// panel is the height of the window and the readings above it need the rows. An
/// inverted quantity puts its low value at the hot end, so the labels swap with it and
/// the bar still reads left to right as best to worst.
fn legend(ui: &mut egui::Ui, quantity: Quantity, (lo, hi): (f64, f64)) {
    const STEPS: usize = 48;
    const HEIGHT: f32 = 14.0;
    let width = ui.available_width().clamp(60.0, 240.0);
    let (rect, response) = ui.allocate_exact_size(egui::vec2(width, HEIGHT), egui::Sense::hover());
    let painter = ui.painter();
    let step = rect.width() / STEPS as f32;
    for i in 0..STEPS {
        // Band `i` from the left; its colour is read at its middle.
        let t = (i as f32 + 0.5) / STEPS as f32;
        let [r, g, b] = stress_ramp(t);
        let band = egui::Rect::from_min_size(
            egui::pos2(rect.left() + i as f32 * step, rect.top()),
            egui::vec2(step + 0.5, rect.height()),
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
    response.on_hover_text(match quantity.unit() {
        "" => quantity.name().to_owned(),
        unit => format!("{} in {unit}", quantity.name()),
    });
    let (left, right) = if quantity.inverted() {
        (hi, lo)
    } else {
        (lo, hi)
    };
    ui.allocate_ui_with_layout(
        egui::vec2(width, ui.text_style_height(&egui::TextStyle::Body)),
        egui::Layout::left_to_right(egui::Align::Center),
        |ui| {
            ui.label(quantity.format(left));
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                ui.label(quantity.format(right));
            });
        },
    );
}

// --- The plot on the GPU -----------------------------------------------------------

/// The results mesh as uploaded: which run and under what plot settings, so it is
/// re-uploaded only when one of them changes.
pub(super) struct ResultsMesh {
    pub handle: MeshHandle,
    run: u64,
    spec: PlotSpec,
}

impl Editor {
    /// Keeps the GPU copy of the plot in step with the results and settings the panel
    /// shows, and drops it when there is nothing to show — including in the Design
    /// workspace, where the plot is not drawn. Called from [`Editor::sync_meshes`],
    /// which is where the device and queue are.
    pub(super) fn sync_results_mesh(
        &mut self,
        renderer: &mut basset_viewport::Renderer,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
    ) {
        let revision = self.doc.revision();
        let wanted = self
            .study_view()
            .and_then(|s| s.plotted(revision).map(|(_, o)| (o.run, s.plot_spec())));
        let current = self.results_mesh.as_ref().map(|m| (m.run, m.spec));
        if wanted == current {
            return;
        }
        if let Some(old) = self.results_mesh.take() {
            renderer.remove_mesh(old.handle);
        }
        let Some((run, spec)) = wanted else {
            return;
        };
        let Some(sim) = self.simulation.as_ref() else {
            return;
        };
        let Some(outcome) = sim.outcome.as_ref() else {
            return;
        };
        let (mesh, colors) = outcome.plot(&spec);
        // The brick surface has no feature edges worth drawing: every facet boundary is
        // a stair step, and drawing them would show the mesh rather than the body.
        match renderer.upload_colored_mesh(device, queue, &mesh, &colors, &[]) {
            Ok(handle) => {
                self.results_mesh = Some(ResultsMesh { handle, run, spec });
            }
            Err(e) => log::error!("results mesh upload failed: {e}"),
        }
    }
}
