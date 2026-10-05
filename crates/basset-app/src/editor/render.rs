//! The Render workspace: appearances, the scene they are lit by, an in-canvas render and
//! final renders to images, as Fusion's Render workspace has them.
//!
//! # Appearances are painted, not configured
//!
//! Fusion's appearance dialog is a library on one side and the model on the other, and
//! a finish is dragged from one onto the other. The same gesture here is a click: a
//! swatch clicked in the library becomes the *brush*, and every face clicked in the
//! viewport after that is painted with it — the face alone, or its whole body, as the
//! panel's "Apply to" says — until Escape puts the brush down. Without a brush a click
//! selects, and the panel shows what the selection wears with a button to take it off.
//! Every paint is a document edit with an undo step of its own; see
//! [`basset_core::Appearances`] for how assignments are kept and why undoing one does not
//! regenerate the model.
//!
//! Appearances show in the Design workspace too, as Fusion shows them: a part painted
//! red is red while it is being modelled. The Simulation workspace keeps the plain grey,
//! because there the colour of a face is what the study says about it.
//!
//! # The viewport in this workspace
//!
//! The view is the raster preview under the scene's environment: physically based
//! shading of every appearance against the same sky and lights the path tracer uses, the
//! sky drawn behind the model, and nothing of the modelling furniture — no grid, no
//! sketches, no edges — so what is on screen is the picture. The in-canvas render is the
//! path tracer drawing over that preview, progressively, at a fraction of the window's
//! resolution chosen by its quality. It is restarted whenever anything it depends on
//! changes (the camera, the window, the model, an appearance, the scene, what is
//! hidden), and until the restarted render's first pass is in the raster preview shows
//! through, so orbiting stays fluid and the picture sharpens the moment the camera stops.
//!
//! # Final renders
//!
//! Render opens a dialog for the size and the number of samples, and the render runs on
//! its own threads into the rendering gallery along the bottom of the window, where its
//! progress shows on its thumbnail. A finished render is opened from the gallery and
//! saved as a PNG. The gallery belongs to the open document, as Fusion's belongs to the
//! design, and a new or opened document starts an empty one.

use std::sync::Arc;
use std::time::Duration;

use basset_core::{BodyRef, FaceRef};
use basset_render::{
    Appearance, Background, Category, EnvironmentKind, Image, Lens, Pattern, RenderCamera,
    RenderJob, RenderOptions, SceneSettings, Srgb, TraceScene,
};
use basset_viewport::{Camera, DistantLight, EnvironmentLight, Lighting, Material, Projection};

use super::commands::Command;
use super::selection::{Pick, SelectionFilter};
use super::{Editor, Workspace};

/// Faces only: a click in this workspace lands on a face, which is painted or selected
/// whole or as part of its body.
pub const FILTER: SelectionFilter = SelectionFilter {
    faces: true,
    edges: false,
    vertices: false,
    points: false,
    planes: false,
    profiles: false,
    curves: false,
};

pub const PROMPT: &str =
    "Render: pick an appearance in the library, then click bodies or faces to paint them";
pub const REFUSED: &str = "Modelling is done in the Design workspace: click its tab";

/// What a paint lands on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ApplyTo {
    /// The whole body the clicked face belongs to.
    Bodies,
    /// The clicked face alone.
    Faces,
}

/// What a [`Command::Paint`] paints.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PaintTarget {
    Body(BodyRef),
    Face(FaceRef),
    /// Every body that has no appearance of its own.
    Document,
}

/// Which page of the side panel is up.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PanelPage {
    Appearance,
    Scene,
}

/// How much of the window's resolution the in-canvas render traces. Fewer pixels clean
/// up sooner; the picture is scaled up to cover the view either way.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Quality {
    Draft,
    Standard,
    High,
}

impl Quality {
    pub const ALL: [Quality; 3] = [Quality::Draft, Quality::Standard, Quality::High];

    pub fn name(self) -> &'static str {
        match self {
            Quality::Draft => "Draft",
            Quality::Standard => "Standard",
            Quality::High => "High",
        }
    }

    fn scale(self) -> f64 {
        match self {
            Quality::Draft => 0.33,
            Quality::Standard => 0.5,
            Quality::High => 1.0,
        }
    }

    /// Where the in-canvas render stops refining. Past this a picture at that resolution
    /// no longer visibly improves; a final render is for more.
    fn samples(self) -> u32 {
        match self {
            Quality::Draft => 256,
            Quality::Standard => 512,
            Quality::High => 1024,
        }
    }
}

/// Everything the in-canvas render's picture depends on. When any of it changes the
/// picture on screen is of something else, so it is hidden and the render restarted.
#[derive(Clone, Debug, PartialEq)]
struct CanvasKey {
    camera: Camera,
    window: [u32; 2],
    quality: Quality,
    revision: u64,
    appearances: u64,
    hidden: Vec<BodyRef>,
}

/// The in-canvas render while it is switched on.
pub struct InCanvas {
    job: Option<RenderJob>,
    key: Option<CanvasKey>,
    /// The picture on screen and the key it was taken under.
    texture: Option<(egui::TextureHandle, CanvasKey)>,
    shown_samples: u32,
}

impl InCanvas {
    fn new() -> Self {
        Self {
            job: None,
            key: None,
            texture: None,
            shown_samples: 0,
        }
    }

    /// Samples in the picture on screen, and the number the render stops at.
    pub fn progress(&self) -> Option<(u32, u32, Duration)> {
        self.job
            .as_ref()
            .map(|j| (j.samples(), j.target(), j.elapsed()))
    }
}

/// The size and quality the Render dialog is set to, kept between renders.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FinalSettings {
    pub width: u32,
    pub height: u32,
    pub samples: u32,
}

impl Default for FinalSettings {
    fn default() -> Self {
        Self {
            width: 1920,
            height: 1080,
            samples: 256,
        }
    }
}

/// One entry of the rendering gallery.
pub struct FinalRender {
    pub name: String,
    pub settings: FinalSettings,
    job: Option<RenderJob>,
    /// The latest picture, finished or not.
    pub image: Option<Arc<Image>>,
    texture: Option<egui::TextureHandle>,
    texture_samples: u32,
    pub elapsed: Duration,
}

impl FinalRender {
    pub fn is_running(&self) -> bool {
        self.job.as_ref().is_some_and(|j| !j.is_finished())
    }

    /// Samples done, of how many.
    pub fn progress(&self) -> (u32, u32) {
        let done = self
            .job
            .as_ref()
            .map(RenderJob::samples)
            .or(self.image.as_ref().map(|i| i.samples))
            .unwrap_or(0);
        (done, self.settings.samples)
    }
}

/// The workspace's state. It lives on the editor whichever workspace is up, so a brush
/// and the panel's place survive a trip to Design and back.
pub struct Studio {
    pub brush: Option<Appearance>,
    pub apply_to: ApplyTo,
    pub page: PanelPage,
    /// What the library's filter box holds.
    pub filter: String,
    /// The appearance of the design whose settings the panel shows.
    pub editing: Option<String>,
    pub quality: Quality,
    pub in_canvas: Option<InCanvas>,
    /// The Render dialog's settings, and whether it is open.
    pub dialog: Option<FinalSettings>,
    pub last_final: FinalSettings,
    pub gallery: Vec<FinalRender>,
    /// The gallery entry open in the viewer.
    pub viewing: Option<usize>,
    /// The traced scene, kept while the model, appearances, scene settings and hidden
    /// bodies are what it was built from: building it is the one costly step in
    /// restarting a render after the camera moves, and the camera is what moves most.
    trace: Option<(TraceKey, Arc<TraceScene>)>,
}

#[derive(Clone, Debug, PartialEq)]
struct TraceKey {
    revision: u64,
    appearances: u64,
    hidden: Vec<BodyRef>,
}

impl Default for Studio {
    fn default() -> Self {
        Self {
            brush: None,
            apply_to: ApplyTo::Bodies,
            page: PanelPage::Appearance,
            filter: String::new(),
            editing: None,
            quality: Quality::Standard,
            in_canvas: None,
            dialog: None,
            last_final: FinalSettings::default(),
            gallery: Vec::new(),
            viewing: None,
            trace: None,
        }
    }
}

// --- Entering and leaving -----------------------------------------------------------

/// What entering the workspace does: puts down any modelling in progress, as entering
/// Simulation does, and says what to do.
pub fn enter(editor: &mut Editor) {
    if editor.tool.is_some() {
        super::tools::cancel_tool(editor);
    }
    super::measure::stop(editor);
    editor.hover = None;
    editor.set_status(PROMPT);
    editor.request_repaint();
}

/// Leaving stops the in-canvas render — tracing a view nobody is looking at would only
/// take the processor from the modelling — but keeps the gallery and any final render
/// still running, which the user asked for and will come back to.
pub fn leave(editor: &mut Editor) {
    editor.render.in_canvas = None;
    editor.render.brush = None;
    editor.hover = None;
    editor.request_repaint();
}

/// Drops everything that belongs to the document: a new or opened one has other bodies.
pub fn reset(editor: &mut Editor) {
    editor.render = Studio {
        quality: editor.render.quality,
        last_final: editor.render.last_final,
        apply_to: editor.render.apply_to,
        ..Studio::default()
    };
}

// --- Painting -----------------------------------------------------------------------

/// Takes a click in the Render workspace while a brush is up. Returns whether it was
/// consumed: with no brush the click is an ordinary selection.
pub fn clicked(editor: &mut Editor, pick: Option<&Pick>) -> bool {
    if editor.workspace != Workspace::Render {
        return false;
    }
    let Some(brush) = editor.render.brush.clone() else {
        return false;
    };
    let Some(Pick::Face(face, _)) = pick else {
        // A click on nothing with a brush up is a miss, not a request to deselect.
        return true;
    };
    let target = match editor.render.apply_to {
        ApplyTo::Bodies => PaintTarget::Body(face.body),
        ApplyTo::Faces => PaintTarget::Face(*face),
    };
    paint(editor, target, Some(&brush));
    true
}

/// Paints a target with an appearance, or with `None` takes its own appearance off.
pub fn paint(editor: &mut Editor, target: PaintTarget, appearance: Option<&Appearance>) {
    if editor.doc.in_transaction() {
        editor.set_status(
            "Finish or cancel the sketch or tool first: a change made now would be undone \
             along with it",
        );
        return;
    }
    let what = appearance.map_or("its own appearance removed".to_owned(), |a| {
        a.display_name().to_owned()
    });
    match target {
        PaintTarget::Body(body) => {
            editor.doc.set_body_appearance(body, appearance);
            let name = editor.body_name(body);
            editor.set_status(format!("{name}: {what}"));
        }
        PaintTarget::Face(face) => {
            editor
                .doc
                .set_face_appearance(face.body, face.key, appearance);
            let name = editor.body_name(face.body);
            editor.set_status(format!("A face of {name}: {what}"));
        }
        PaintTarget::Document => {
            editor.doc.set_default_appearance(appearance);
            editor.set_status(format!("Every body without its own appearance: {what}"));
        }
    }
    if let Some(a) = appearance {
        editor.render.editing = Some(a.display_name().to_owned());
    }
    editor.request_repaint();
}

/// Arms the brush with an appearance, or puts it down.
pub fn arm(editor: &mut Editor, appearance: Option<Appearance>) {
    match &appearance {
        Some(a) => editor.set_status(format!(
            "Painting with {}: click {} to apply it (Esc to stop)",
            a.display_name(),
            match editor.render.apply_to {
                ApplyTo::Bodies => "bodies",
                ApplyTo::Faces => "faces",
            }
        )),
        None => editor.set_status(PROMPT),
    }
    editor.render.brush = appearance;
    editor.request_repaint();
}

// --- What the viewport draws ----------------------------------------------------------

/// The scene's environment as the raster preview shades with it. The sky and lights are
/// the path tracer's own, so a highlight in the preview is where the render puts it.
pub fn viewport_lighting(scene: &SceneSettings) -> Lighting {
    let lighting = scene.lighting();
    Lighting::Environment(EnvironmentLight {
        zenith: lighting.zenith,
        horizon: lighting.horizon,
        nadir: lighting.nadir,
        lights: lighting
            .lights
            .iter()
            .map(|l| DistantLight {
                direction: l.direction,
                angular_radius: l.angular_radius as f32,
                radiance: l.radiance,
            })
            .collect(),
        // Brightness is already in the radiances.
        exposure: 1.0,
        sky_background: scene.background == Background::Environment,
    })
}

/// The clear colour behind the model when the background is a solid colour: linear,
/// and not tone mapped, so it is the colour picked.
pub fn background_color(scene: &SceneSettings) -> Option<[f32; 4]> {
    match scene.background {
        Background::Solid(c) => {
            let [r, g, b] = c.to_linear();
            Some([r, g, b, 1.0])
        }
        Background::Environment => None,
    }
}

/// How an appearance is drawn by the viewport: its base colour (with the opacity glass
/// is shown at) and the shading numbers. Patterns are the path tracer's alone.
pub fn viewport_look(a: &Appearance) -> ([f32; 4], Material) {
    let [r, g, b] = a.color.to_linear();
    let alpha = if a.is_transmissive() {
        a.viewport_alpha()
    } else {
        1.0
    };
    (
        [r, g, b, alpha],
        Material {
            metallic: a.metallic,
            roughness: a.roughness,
            clearcoat: a.clearcoat,
            emission: [r * a.emission, g * a.emission, b * a.emission],
        },
    )
}

/// Whether bodies are drawn in their appearances in the current workspace.
pub fn shows_appearances(editor: &Editor) -> bool {
    editor.workspace != Workspace::Simulation
}

// --- The traced scene and the in-canvas render ----------------------------------------

fn hidden_sorted(editor: &Editor) -> Vec<BodyRef> {
    let mut hidden: Vec<BodyRef> = editor.hidden_bodies.iter().copied().collect();
    hidden.sort();
    hidden
}

/// The traced scene for the model as it is, built if what it was built from moved.
fn trace_scene(editor: &mut Editor) -> Arc<TraceScene> {
    let key = TraceKey {
        revision: editor.doc.revision(),
        appearances: editor.doc.appearance_revision(),
        hidden: hidden_sorted(editor),
    };
    if let Some((k, scene)) = &editor.render.trace
        && *k == key
    {
        return scene.clone();
    }
    let mut bodies: Vec<(BodyRef, Arc<basset_kernel::Tessellated>)> = editor
        .pick_bodies
        .iter()
        .filter(|(id, _)| !editor.hidden_bodies.contains(id))
        .map(|(id, p)| (*id, p.tess.clone()))
        .collect();
    // In a fixed order, so the same model always builds the same scene and renders the
    // same picture.
    bodies.sort_by_key(|(id, _)| *id);
    let scene = Arc::new(basset_core::trace_scene(
        bodies.iter().map(|(id, t)| (*id, t.as_ref())),
        editor.doc.appearances(),
        editor.doc.scene(),
    ));
    editor.render.trace = Some((key, scene.clone()));
    scene
}

/// The viewport camera as the path tracer's: the same eye, axes and projection, so the
/// picture lands on the preview pixel for pixel.
pub fn render_camera(camera: &Camera, scene: &SceneSettings) -> RenderCamera {
    let lens = match camera.projection {
        Projection::Perspective { fov_y } => Lens::Perspective { fov_y },
        Projection::Orthographic { half_height } => Lens::Orthographic { half_height },
    };
    let c = RenderCamera {
        eye: camera.eye(),
        forward: camera.forward(),
        right: camera.right(),
        up: camera.up(),
        lens,
        aperture: 0.0,
        focus_distance: camera.distance,
    };
    if scene.depth_of_field {
        c.with_depth_of_field(f64::from(scene.aperture))
    } else {
        c
    }
}

/// Switches the in-canvas render on or off.
pub fn toggle_in_canvas(editor: &mut Editor) {
    if editor.render.in_canvas.take().is_some() {
        editor.set_status("In-canvas render off");
    } else {
        editor.render.in_canvas = Some(InCanvas::new());
        editor.set_status("In-canvas render on: it refines while the view is still");
    }
    editor.request_repaint();
}

/// Keeps the in-canvas render and the gallery in step with the editor, once a frame:
/// restarts the in-canvas render when what it shows has moved, brings finished passes
/// to the screen, and asks for another frame while anything is still rendering.
pub fn poll(editor: &mut Editor, ctx: &egui::Context) {
    let mut busy = false;
    if editor.workspace == Workspace::Render && editor.render.in_canvas.is_some() {
        let key = CanvasKey {
            camera: editor.camera,
            window: editor.window_px,
            quality: editor.render.quality,
            revision: editor.doc.revision(),
            appearances: editor.doc.appearance_revision(),
            hidden: hidden_sorted(editor),
        };
        let stale = editor
            .render
            .in_canvas
            .as_ref()
            .is_some_and(|c| c.key.as_ref() != Some(&key));
        if stale {
            let scene = trace_scene(editor);
            let camera = render_camera(&editor.camera, editor.doc.scene());
            let scale = key.quality.scale();
            let width = ((f64::from(key.window[0]) * scale).round() as u32).max(1);
            let height = ((f64::from(key.window[1]) * scale).round() as u32).max(1);
            let options = RenderOptions {
                width,
                height,
                samples: key.quality.samples(),
                max_bounces: 6,
                ..RenderOptions::default()
            };
            if let Some(c) = editor.render.in_canvas.as_mut() {
                // Dropping the old job cancels it.
                c.job = Some(RenderJob::start(scene, camera, options));
                c.key = Some(key);
                c.shown_samples = 0;
            }
        }
        if let Some(c) = editor.render.in_canvas.as_mut()
            && let (Some(job), Some(key)) = (c.job.as_ref(), c.key.as_ref())
        {
            busy |= !job.is_finished();
            if let Some(image) = job.latest()
                && image.samples != c.shown_samples
            {
                let color = color_image(&image);
                match &mut c.texture {
                    Some((texture, shown)) => {
                        texture.set(color, egui::TextureOptions::LINEAR);
                        *shown = key.clone();
                    }
                    None => {
                        let texture = ctx.load_texture(
                            "in-canvas render",
                            color,
                            egui::TextureOptions::LINEAR,
                        );
                        c.texture = Some((texture, key.clone()));
                    }
                }
                c.shown_samples = image.samples;
            }
        }
    } else if editor.workspace != Workspace::Render {
        editor.render.in_canvas = None;
    }

    for entry in &mut editor.render.gallery {
        let Some(job) = entry.job.as_ref() else {
            continue;
        };
        busy |= !job.is_finished();
        entry.elapsed = job.elapsed();
        if let Some(image) = job.latest() {
            entry.image = Some(image);
        }
        if job.is_finished() {
            entry.job = None;
        }
    }
    for entry in &mut editor.render.gallery {
        if let Some(image) = &entry.image
            && image.samples != entry.texture_samples
        {
            let color = color_image(image);
            match &mut entry.texture {
                Some(t) => t.set(color, egui::TextureOptions::LINEAR),
                None => {
                    entry.texture =
                        Some(ctx.load_texture(&entry.name, color, egui::TextureOptions::LINEAR))
                }
            }
            entry.texture_samples = image.samples;
        }
    }
    if busy {
        // Passes land a few times a second; polling faster would only spin the UI.
        ctx.request_repaint_after(Duration::from_millis(80));
    }
}

fn color_image(image: &Image) -> egui::ColorImage {
    egui::ColorImage::from_rgba_unmultiplied(
        [image.width as usize, image.height as usize],
        &image.pixels,
    )
}

/// The in-canvas picture, painted over the whole window under the panels, if there is
/// one of the view as it is now.
pub fn paint_canvas(editor: &Editor, ui: &egui::Ui) {
    if editor.workspace != Workspace::Render {
        return;
    }
    let Some(c) = editor.render.in_canvas.as_ref() else {
        return;
    };
    let (Some((texture, shown)), Some(key)) = (&c.texture, &c.key) else {
        return;
    };
    if shown != key || c.shown_samples == 0 {
        return;
    }
    let rect = ui.max_rect();
    ui.painter().image(
        texture.id(),
        rect,
        egui::Rect::from_min_max(egui::pos2(0.0, 0.0), egui::pos2(1.0, 1.0)),
        egui::Color32::WHITE,
    );
}

// --- Final renders --------------------------------------------------------------------

/// Starts a final render of the current view at the dialog's settings, into the gallery.
pub fn start_final(editor: &mut Editor, settings: FinalSettings) {
    let settings = FinalSettings {
        width: settings.width.clamp(16, 8192),
        height: settings.height.clamp(16, 8192),
        samples: settings.samples.clamp(1, 16384),
    };
    editor.render.last_final = settings;
    editor.render.dialog = None;
    let scene = trace_scene(editor);
    let camera = render_camera(&editor.camera, editor.doc.scene());
    let options = RenderOptions {
        width: settings.width,
        height: settings.height,
        samples: settings.samples,
        max_bounces: 10,
        ..RenderOptions::default()
    };
    let n = editor.render.gallery.len() + 1;
    editor.render.gallery.push(FinalRender {
        name: format!("{} {n}", editor.doc.name),
        settings,
        job: Some(RenderJob::start(scene, camera, options)),
        image: None,
        texture: None,
        texture_samples: 0,
        elapsed: Duration::ZERO,
    });
    editor.set_status(format!(
        "Rendering {}×{} at {} samples into the gallery",
        settings.width, settings.height, settings.samples
    ));
    editor.request_repaint();
}

/// Asks where to save a gallery entry and saves it.
pub fn save_final(editor: &mut Editor, index: usize) {
    let Some(entry) = editor.render.gallery.get(index) else {
        return;
    };
    let mut dialog = rfd::FileDialog::new()
        .add_filter("PNG image", &["png"])
        .set_file_name(format!("{}.png", entry.name));
    if let Some(root) = editor.project_root() {
        dialog = dialog.set_directory(root);
    }
    if let Some(path) = dialog.save_file() {
        save_final_to(editor, index, &path);
    }
}

pub fn save_final_to(editor: &mut Editor, index: usize, path: &std::path::Path) {
    let Some(image) = editor
        .render
        .gallery
        .get(index)
        .and_then(|e| e.image.clone())
    else {
        editor.report_error("that render has no picture yet");
        return;
    };
    let path = super::files::with_extension(path.to_path_buf(), "png");
    match image.save_png(&path) {
        Ok(()) => editor.set_status(format!("Saved {}", path.display())),
        Err(e) => editor.report_error(format!("could not save {}: {e}", path.display())),
    }
}

/// Takes an entry out of the gallery, stopping it if it is still running.
pub fn remove_final(editor: &mut Editor, index: usize) {
    if index < editor.render.gallery.len() {
        editor.render.gallery.remove(index);
        editor.render.viewing = match editor.render.viewing {
            Some(v) if v == index => None,
            Some(v) if v > index => Some(v - 1),
            other => other,
        };
    }
}

// --- Panels ---------------------------------------------------------------------------

/// The ribbon: setup, the in-canvas render and final renders.
pub(super) fn toolbar(editor: &mut Editor, ui: &mut egui::Ui, commands: &mut Vec<Command>) {
    ui.horizontal_wrapped(|ui| {
        ui.label(egui::RichText::new("Setup").weak());
        let studio = &mut editor.render;
        for (page, label, hint) in [
            (
                PanelPage::Appearance,
                "Appearance",
                "The appearance library, and what is applied in this design",
            ),
            (
                PanelPage::Scene,
                "Scene settings",
                "Environment, brightness, background, floor and camera",
            ),
        ] {
            if ui
                .selectable_label(studio.page == page, label)
                .on_hover_text(hint)
                .clicked()
            {
                studio.page = page;
            }
        }
        ui.separator();

        ui.label(egui::RichText::new("In-canvas render").weak());
        let on = studio.in_canvas.is_some();
        if ui
            .selectable_label(on, if on { "● Rendering" } else { "Start" })
            .on_hover_text("Path trace the view in place; it refines while the view is still")
            .clicked()
        {
            commands.push(Command::ToggleInCanvas);
        }
        egui::ComboBox::from_id_salt("render-quality")
            .selected_text(studio.quality.name())
            .width(84.0)
            .show_ui(ui, |ui| {
                for q in Quality::ALL {
                    ui.selectable_value(&mut studio.quality, q, q.name());
                }
            })
            .response
            .on_hover_text("How much of the window's resolution the in-canvas render traces");
        if let Some((done, target, elapsed)) =
            studio.in_canvas.as_ref().and_then(InCanvas::progress)
        {
            if done < target {
                ui.add(egui::Spinner::new().size(14.0));
            }
            ui.add(
                egui::ProgressBar::new(done as f32 / target.max(1) as f32)
                    .desired_width(80.0)
                    .text(format!("{done}/{target}")),
            );
            ui.label(egui::RichText::new(format!("{:.1} s", elapsed.as_secs_f64())).weak());
        }
        ui.separator();

        ui.label(egui::RichText::new("Render").weak());
        if ui
            .button("Render…")
            .on_hover_text("Render the current view to an image, into the gallery")
            .clicked()
        {
            studio.dialog = Some(studio.last_final);
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

/// The browser in this workspace: what every body wears, and the scene.
pub(super) fn tree(editor: &mut Editor, ui: &mut egui::Ui, commands: &mut Vec<Command>) {
    ui.heading(&editor.doc.name);
    let bodies = editor.cached_bodies.clone();
    let mut clicked = None;
    egui::ScrollArea::vertical().show(ui, |ui| {
        let appearances = editor.doc.appearances();
        super::theme::section("Appearances")
            .id_salt("render-bodies")
            .default_open(true)
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    swatch(ui, appearances.document_default(), 16.0);
                    ui.label(format!(
                        "Default: {}",
                        appearances.document_default().display_name()
                    ))
                    .on_hover_text("What every body without an appearance of its own wears");
                });
                if bodies.is_empty() {
                    ui.label(egui::RichText::new("No bodies").weak());
                }
                for (id, name) in &bodies {
                    let a = appearances.body(*id);
                    let faces = appearances.face_overrides(*id).count();
                    ui.horizontal(|ui| {
                        swatch(ui, a, 16.0);
                        let selected = editor.selection.bodies.contains(id);
                        let label = if appearances.body_assignment(*id).is_some() {
                            format!("{name} — {}", a.display_name())
                        } else {
                            name.clone()
                        };
                        if ui.selectable_label(selected, label).clicked() {
                            clicked = Some(*id);
                        }
                        if faces > 0
                            && ui
                                .small_button(format!("{faces} face{}", plural(faces)))
                                .on_hover_text("Remove the appearances given to single faces")
                                .clicked()
                        {
                            commands.push(Command::ClearFaceAppearances(*id));
                        }
                    });
                }
            });
        let scene = editor.doc.scene();
        super::theme::section("Scene")
            .id_salt("render-scene-summary")
            .default_open(true)
            .show(ui, |ui| {
                ui.label(format!("Environment: {}", scene.environment.name()));
                ui.label(format!("Brightness: {:+.1} EV", scene.brightness));
                ui.label(match scene.background {
                    Background::Environment => "Background: environment".to_owned(),
                    Background::Solid(c) => format!("Background: {c}"),
                });
                ui.label(if scene.ground_plane {
                    "Floor: on"
                } else {
                    "Floor: off"
                });
            });
        ui.label(
            egui::RichText::new(format!(
                "Rendering gallery: {}",
                editor.render.gallery.len()
            ))
            .weak(),
        );
    });
    if let Some(body) = clicked {
        editor.selection.clear();
        editor.selection.bodies.push(body);
    }
}

fn plural(n: usize) -> &'static str {
    if n == 1 { "" } else { "s" }
}

/// The side panel: the appearance library and the design's appearances, or the scene.
pub(super) fn panel(editor: &mut Editor, ui: &mut egui::Ui, commands: &mut Vec<Command>) {
    let labels = ["Appearance", "Scene"];
    let chosen = match editor.render.page {
        PanelPage::Appearance => 0,
        PanelPage::Scene => 1,
    };
    let responses = super::theme::segmented(ui, &labels, chosen);
    for (page, r) in [PanelPage::Appearance, PanelPage::Scene]
        .into_iter()
        .zip(responses)
    {
        if r.clicked() {
            editor.render.page = page;
        }
    }
    ui.add_space(4.0);
    egui::ScrollArea::vertical()
        .auto_shrink([false, false])
        .show(ui, |ui| match editor.render.page {
            PanelPage::Appearance => appearance_page(editor, ui, commands),
            PanelPage::Scene => scene_page(editor, ui, commands),
        });
}

fn appearance_page(editor: &mut Editor, ui: &mut egui::Ui, commands: &mut Vec<Command>) {
    ui.horizontal(|ui| {
        ui.label("Apply to");
        let studio = &mut editor.render;
        ui.selectable_value(&mut studio.apply_to, ApplyTo::Bodies, "Bodies")
            .on_hover_text("A click paints the whole body");
        ui.selectable_value(&mut studio.apply_to, ApplyTo::Faces, "Faces")
            .on_hover_text("A click paints the one face");
    });
    if let Some(brush) = editor.render.brush.clone() {
        ui.horizontal(|ui| {
            swatch(ui, &brush, 22.0);
            ui.label(egui::RichText::new(format!("Brush: {}", brush.display_name())).strong());
            if ui.small_button("Put down").clicked() {
                commands.push(Command::Brush(None));
            }
        });
        if ui
            .button("Apply to every body")
            .on_hover_text("Make it the default for bodies without an appearance of their own")
            .clicked()
        {
            commands.push(Command::Paint(PaintTarget::Document, Some(Box::new(brush))));
        }
    }

    selection_section(editor, ui, commands);
    design_section(editor, ui, commands);

    super::theme::section("Library")
        .id_salt("render-library")
        .default_open(true)
        .show(ui, |ui| {
            ui.add(
                egui::TextEdit::singleline(&mut editor.render.filter)
                    .hint_text("Filter")
                    .desired_width(f32::INFINITY),
            );
            let filter = editor.render.filter.trim().to_lowercase();
            for category in Category::ALL {
                let entries: Vec<&Appearance> = basset_render::library()
                    .iter()
                    .filter(|a| a.category == category)
                    .filter(|a| {
                        filter.is_empty()
                            || a.name.to_lowercase().contains(&filter)
                            || category.name().to_lowercase().contains(&filter)
                    })
                    .collect();
                if entries.is_empty() {
                    continue;
                }
                super::theme::section(format!("{} ({})", category.name(), entries.len()))
                    .id_salt(("library", category.name()))
                    .default_open(!filter.is_empty() || category == Category::Metal)
                    .show(ui, |ui| {
                        ui.horizontal_wrapped(|ui| {
                            for a in entries {
                                let armed = editor
                                    .render
                                    .brush
                                    .as_ref()
                                    .is_some_and(|b| b.name == a.name);
                                let r = swatch_button(ui, a, armed);
                                if r.on_hover_text(&a.name).clicked() {
                                    commands.push(Command::Brush(if armed {
                                        None
                                    } else {
                                        Some(Box::new(a.clone()))
                                    }));
                                }
                            }
                        });
                    });
            }
        });
}

/// What the selection wears, with a way to take it off.
fn selection_section(editor: &mut Editor, ui: &mut egui::Ui, commands: &mut Vec<Command>) {
    let faces: Vec<FaceRef> = editor.selection.faces.clone();
    let bodies: Vec<BodyRef> = editor.selection.bodies.clone();
    if faces.is_empty() && bodies.is_empty() {
        return;
    }
    super::theme::section("Selected")
        .id_salt("render-selected")
        .default_open(true)
        .show(ui, |ui| {
            let appearances = editor.doc.appearances();
            for face in &faces {
                let own = appearances.face_assignment(face.body, face.key).is_some();
                let a = appearances.face(face.body, face.key);
                ui.horizontal(|ui| {
                    swatch(ui, a, 16.0);
                    ui.label(format!(
                        "Face of {} — {}{}",
                        editor.body_name(face.body),
                        a.display_name(),
                        if own { "" } else { " (from its body)" }
                    ));
                    if own && ui.small_button("×").on_hover_text("Remove").clicked() {
                        commands.push(Command::Paint(PaintTarget::Face(*face), None));
                    }
                });
            }
            let mut owners: Vec<BodyRef> = bodies.clone();
            for f in &faces {
                if !owners.contains(&f.body) {
                    owners.push(f.body);
                }
            }
            for body in owners {
                let own = appearances.body_assignment(body).is_some();
                let a = appearances.body(body);
                ui.horizontal(|ui| {
                    swatch(ui, a, 16.0);
                    ui.label(format!("{} — {}", editor.body_name(body), a.display_name()));
                    if own && ui.small_button("×").on_hover_text("Remove").clicked() {
                        commands.push(Command::Paint(PaintTarget::Body(body), None));
                    }
                });
            }
            if let Some(brush) = editor.render.brush.clone()
                && ui
                    .button(format!("Apply {}", brush.display_name()))
                    .clicked()
            {
                for f in &faces {
                    let target = match editor.render.apply_to {
                        ApplyTo::Faces => PaintTarget::Face(*f),
                        ApplyTo::Bodies => PaintTarget::Body(f.body),
                    };
                    commands.push(Command::Paint(target, Some(Box::new(brush.clone()))));
                }
                for b in &bodies {
                    commands.push(Command::Paint(
                        PaintTarget::Body(*b),
                        Some(Box::new(brush.clone())),
                    ));
                }
            }
        });
}

/// "In this design": the copies the document holds, each one editable.
fn design_section(editor: &mut Editor, ui: &mut egui::Ui, commands: &mut Vec<Command>) {
    let defined: Vec<Appearance> = editor.doc.appearances().defined.values().cloned().collect();
    super::theme::section(format!("In this design ({})", defined.len()))
        .id_salt("render-design")
        .default_open(true)
        .show(ui, |ui| {
            if defined.is_empty() {
                ui.label(
                    egui::RichText::new("Nothing yet: paint something from the library").weak(),
                );
            }
            for a in &defined {
                let uses = editor.doc.appearances().uses(&a.name);
                let editing = editor.render.editing.as_deref() == Some(a.name.as_str());
                ui.horizontal(|ui| {
                    let armed = editor
                        .render
                        .brush
                        .as_ref()
                        .is_some_and(|b| b.name == a.name);
                    if swatch_button(ui, a, armed)
                        .on_hover_text("Paint with it")
                        .clicked()
                    {
                        commands.push(Command::Brush((!armed).then(|| Box::new(a.clone()))));
                    }
                    if ui
                        .selectable_label(editing, format!("{} ({uses})", a.name))
                        .on_hover_text("Edit it; everything wearing it follows")
                        .clicked()
                    {
                        editor.render.editing = (!editing).then(|| a.name.clone());
                    }
                    if ui
                        .small_button("×")
                        .on_hover_text("Remove from the design, and from everything wearing it")
                        .clicked()
                    {
                        commands.push(Command::RemoveAppearance(a.name.clone()));
                    }
                });
                if editing {
                    appearance_editor(ui, a, commands);
                }
            }
        });
}

/// The settings of one appearance of the design. Every change is an edit of the
/// definition; the document folds a run of them into one undo step.
fn appearance_editor(ui: &mut egui::Ui, current: &Appearance, commands: &mut Vec<Command>) {
    let mut a = current.clone();
    egui::Frame::group(ui.style()).show(ui, |ui| {
        egui::Grid::new(("appearance-editor", &current.name))
            .num_columns(2)
            .spacing([8.0, 4.0])
            .show(ui, |ui| {
                ui.label("Name");
                let mut name = a.name.clone();
                let r = ui.add(egui::TextEdit::singleline(&mut name).desired_width(140.0));
                // A rename is applied when the box is left, not on every keystroke: a
                // half-typed name would collide with others on the way to the one meant.
                if r.lost_focus() && name.trim() != a.name {
                    a.name = name;
                }
                ui.end_row();
                ui.label("Colour");
                ui.color_edit_button_srgb(&mut a.color.0);
                ui.end_row();
                let slider = |ui: &mut egui::Ui, label: &str, v: &mut f32, range| {
                    ui.label(label);
                    ui.add(egui::Slider::new(v, range).fixed_decimals(2));
                    ui.end_row();
                };
                slider(ui, "Metallic", &mut a.metallic, 0.0..=1.0);
                slider(ui, "Roughness", &mut a.roughness, 0.0..=1.0);
                slider(ui, "Transmission", &mut a.transmission, 0.0..=1.0);
                if a.transmission > 0.0 {
                    slider(ui, "Refraction", &mut a.ior, 1.0..=2.5);
                }
                slider(ui, "Clear coat", &mut a.clearcoat, 0.0..=1.0);
                slider(ui, "Emission", &mut a.emission, 0.0..=10.0);
                ui.label("Pattern");
                egui::ComboBox::from_id_salt(("pattern", &current.name))
                    .selected_text(a.pattern.name())
                    .show_ui(ui, |ui| {
                        for p in Pattern::ALL {
                            ui.selectable_value(&mut a.pattern, p, p.name());
                        }
                    });
                ui.end_row();
                if a.pattern != Pattern::None {
                    if a.pattern.uses_second_color() {
                        ui.label("Second colour");
                        ui.color_edit_button_srgb(&mut a.color2.0);
                        ui.end_row();
                    }
                    ui.label("Scale");
                    ui.add(
                        egui::DragValue::new(&mut a.pattern_scale)
                            .speed(0.05)
                            .range(0.05..=500.0)
                            .suffix(" mm"),
                    );
                    ui.end_row();
                }
            });
    });
    if a != *current {
        commands.push(Command::EditAppearance(current.name.clone(), Box::new(a)));
    }
}

fn scene_page(editor: &mut Editor, ui: &mut egui::Ui, commands: &mut Vec<Command>) {
    let current = editor.doc.scene().clone();
    let mut s = current.clone();
    super::theme::section("Environment")
        .id_salt("scene-environment")
        .default_open(true)
        .show(ui, |ui| {
            for kind in EnvironmentKind::ALL {
                ui.horizontal(|ui| {
                    environment_swatch(ui, kind);
                    ui.selectable_value(&mut s.environment, kind, kind.name());
                });
            }
        });
    super::theme::section("Lighting")
        .id_salt("scene-lighting")
        .default_open(true)
        .show(ui, |ui| {
            egui::Grid::new("scene-lighting-grid")
                .num_columns(2)
                .show(ui, |ui| {
                    ui.label("Brightness");
                    ui.add(
                        egui::Slider::new(&mut s.brightness, -4.0..=4.0)
                            .fixed_decimals(1)
                            .suffix(" EV"),
                    );
                    ui.end_row();
                    ui.label("Rotation");
                    ui.add(egui::Slider::new(&mut s.rotation, 0.0..=360.0).suffix("°"));
                    ui.end_row();
                });
        });
    super::theme::section("Background")
        .id_salt("scene-background")
        .default_open(true)
        .show(ui, |ui| {
            let solid = matches!(s.background, Background::Solid(_));
            ui.horizontal(|ui| {
                if ui.selectable_label(!solid, "Environment").clicked() {
                    s.background = Background::Environment;
                }
                if ui.selectable_label(solid, "Solid colour").clicked() && !solid {
                    s.background = Background::Solid(Srgb::hex(0xffffff));
                }
                if let Background::Solid(c) = &mut s.background {
                    ui.color_edit_button_srgb(&mut c.0);
                }
            });
        });
    super::theme::section("Floor")
        .id_salt("scene-floor")
        .default_open(true)
        .show(ui, |ui| {
            ui.checkbox(&mut s.ground_plane, "Ground plane")
                .on_hover_text("A floor under the model that catches its shadow");
            ui.add_enabled(
                s.ground_plane,
                egui::Checkbox::new(&mut s.ground_reflections, "Reflections"),
            );
            if s.ground_plane && s.ground_reflections {
                ui.horizontal(|ui| {
                    ui.label("Roughness");
                    ui.add(egui::Slider::new(&mut s.ground_roughness, 0.0..=1.0).fixed_decimals(2));
                });
            }
        });
    super::theme::section("Camera")
        .id_salt("scene-camera")
        .default_open(true)
        .show(ui, |ui| {
            let ortho = editor.camera.is_orthographic();
            ui.horizontal(|ui| {
                if ui.selectable_label(!ortho, "Perspective").clicked() && ortho {
                    commands.push(Command::ToggleProjection);
                }
                if ui.selectable_label(ortho, "Orthographic").clicked() && !ortho {
                    commands.push(Command::ToggleProjection);
                }
            });
            ui.checkbox(&mut s.depth_of_field, "Depth of field")
                .on_hover_text(
                    "Sharp at the point the view orbits about, blurred before and behind it",
                );
            if s.depth_of_field {
                ui.horizontal(|ui| {
                    ui.label("Aperture");
                    ui.add(
                        egui::Slider::new(&mut s.aperture, 0.001..=0.1)
                            .logarithmic(true)
                            .fixed_decimals(3),
                    );
                });
            }
        });
    if s != current {
        commands.push(Command::SetScene(Box::new(s)));
    }
}

/// The Render dialog, the gallery and the viewer: the windows of a final render.
pub(super) fn windows(editor: &mut Editor, ctx: &egui::Context, commands: &mut Vec<Command>) {
    if let Some(mut settings) = editor.render.dialog {
        let mut open = true;
        let mut start = false;
        egui::Window::new("Render")
            .collapsible(false)
            .resizable(false)
            .open(&mut open)
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    for (label, w, h) in [
                        ("720p", 1280, 720),
                        ("1080p", 1920, 1080),
                        ("4K", 3840, 2160),
                        ("Square", 1080, 1080),
                    ] {
                        if ui
                            .selectable_label(settings.width == w && settings.height == h, label)
                            .clicked()
                        {
                            settings.width = w;
                            settings.height = h;
                        }
                    }
                    if ui
                        .button("Viewport")
                        .on_hover_text("The window's own proportions, at its size")
                        .clicked()
                    {
                        settings.width = editor.window_px[0].max(16);
                        settings.height = editor.window_px[1].max(16);
                    }
                });
                egui::Grid::new("render-size").num_columns(2).show(ui, |ui| {
                    ui.label("Width");
                    ui.add(egui::DragValue::new(&mut settings.width).range(16..=8192).suffix(" px"));
                    ui.end_row();
                    ui.label("Height");
                    ui.add(egui::DragValue::new(&mut settings.height).range(16..=8192).suffix(" px"));
                    ui.end_row();
                    ui.label("Samples");
                    ui.add(egui::DragValue::new(&mut settings.samples).range(1..=16384))
                        .on_hover_text("Per pixel. More is cleaner and slower: 128 is a draft, 512 a final");
                    ui.end_row();
                });
                ui.label(
                    egui::RichText::new(
                        "The view is kept upright and framed by its height; a wider image shows more at the sides.",
                    )
                    .weak()
                    .small(),
                );
                if ui.button("Render").clicked() {
                    start = true;
                }
            });
        editor.render.dialog = if open && !start { Some(settings) } else { None };
        if start {
            commands.push(Command::StartRender(settings));
        }
    }

    if let Some(index) = editor.render.viewing {
        let mut open = true;
        if let Some(entry) = editor.render.gallery.get(index) {
            let title = entry.name.clone();
            egui::Window::new(title)
                .id(egui::Id::new("render-viewer"))
                .open(&mut open)
                .default_size([720.0, 480.0])
                .show(ctx, |ui| {
                    let (done, target) = entry.progress();
                    ui.horizontal(|ui| {
                        ui.label(format!(
                            "{}×{} · {done}/{target} samples · {:.1} s",
                            entry.settings.width,
                            entry.settings.height,
                            entry.elapsed.as_secs_f64()
                        ));
                        if ui
                            .add_enabled(entry.image.is_some(), egui::Button::new("Save PNG…"))
                            .clicked()
                        {
                            commands.push(Command::SaveRender(index));
                        }
                    });
                    if let Some(t) = &entry.texture {
                        let avail = ui.available_size();
                        let aspect = entry.settings.width as f32 / entry.settings.height as f32;
                        let w = avail.x.min(avail.y * aspect).max(32.0);
                        ui.image((t.id(), egui::vec2(w, w / aspect)));
                    } else {
                        ui.spinner();
                    }
                });
        }
        if !open {
            editor.render.viewing = None;
        }
    }
}

/// The rendering gallery along the bottom: a thumbnail per render, with its progress.
pub(super) fn gallery(editor: &mut Editor, ui: &mut egui::Ui, commands: &mut Vec<Command>) {
    ui.horizontal(|ui| {
        ui.label(egui::RichText::new("Rendering gallery").weak());
        egui::ScrollArea::horizontal().show(ui, |ui| {
            for (i, entry) in editor.render.gallery.iter().enumerate() {
                ui.vertical(|ui| {
                    let size = egui::vec2(
                        96.0 * entry.settings.width as f32 / entry.settings.height.max(1) as f32,
                        96.0,
                    )
                    .min(egui::vec2(170.0, 96.0));
                    let r = match &entry.texture {
                        Some(t) => {
                            ui.add(egui::Image::new((t.id(), size)).sense(egui::Sense::click()))
                        }
                        None => ui.add_sized(size, egui::Spinner::new()),
                    };
                    if r.on_hover_text("Open").clicked() {
                        editor.render.viewing = Some(i);
                    }
                    ui.horizontal(|ui| {
                        let (done, target) = entry.progress();
                        if entry.is_running() {
                            ui.add(
                                egui::ProgressBar::new(done as f32 / target.max(1) as f32)
                                    .desired_width(size.x - 24.0),
                            );
                        } else {
                            ui.label(egui::RichText::new(&entry.name).small());
                        }
                        if ui
                            .small_button("×")
                            .on_hover_text(if entry.is_running() {
                                "Stop and remove"
                            } else {
                                "Remove"
                            })
                            .clicked()
                        {
                            commands.push(Command::RemoveRender(i));
                        }
                    });
                });
            }
        });
    });
}

// --- Swatches ---------------------------------------------------------------------------

/// A library swatch the size of a thumbnail, framed when it is the brush.
fn swatch_button(ui: &mut egui::Ui, a: &Appearance, armed: bool) -> egui::Response {
    let size = 34.0;
    let (rect, response) = ui.allocate_exact_size(egui::vec2(size, size), egui::Sense::click());
    paint_sphere(ui.painter(), rect.center(), size * 0.44, a);
    if armed || response.hovered() {
        let color = if armed {
            ui.visuals().selection.stroke.color
        } else {
            ui.visuals().widgets.hovered.fg_stroke.color
        };
        ui.painter().rect_stroke(
            rect,
            4.0,
            egui::Stroke::new(if armed { 2.0 } else { 1.0 }, color),
            egui::StrokeKind::Inside,
        );
    }
    response
}

/// A small shaded ball in an appearance, for rows and labels.
pub fn swatch(ui: &mut egui::Ui, a: &Appearance, size: f32) -> egui::Response {
    let (rect, response) = ui.allocate_exact_size(egui::vec2(size, size), egui::Sense::hover());
    paint_sphere(ui.painter(), rect.center(), size * 0.45, a);
    response
}

/// Paints a sphere lit by a soft box from the upper left, in a few dozen coloured
/// vertices: enough to tell polished from satin and metal from paint at a glance, which
/// is what a swatch is for. Not the path tracer: a library of eighty swatches has to
/// paint in a frame.
fn paint_sphere(painter: &egui::Painter, center: egui::Pos2, radius: f32, a: &Appearance) {
    const RINGS: usize = 7;
    const SEGMENTS: usize = 28;
    let mut mesh = egui::Mesh::default();
    let shade = |x: f32, y: f32| -> egui::Color32 {
        let z = (1.0 - x * x - y * y).max(0.0).sqrt();
        let c = sphere_color(a, [x, -y, z]);
        let [r, g, b] = basset_render::color::tone_map(c);
        let alpha = if a.is_transmissive() {
            (255.0 * a.viewport_alpha().max(0.35)) as u8
        } else {
            255
        };
        egui::Color32::from_rgba_unmultiplied(r, g, b, alpha)
    };
    mesh.colored_vertex(center, shade(0.0, 0.0));
    for ring in 1..=RINGS {
        let t = ring as f32 / RINGS as f32;
        for s in 0..SEGMENTS {
            let phi = std::f32::consts::TAU * s as f32 / SEGMENTS as f32;
            let (x, y) = (t * phi.cos(), t * phi.sin());
            mesh.colored_vertex(
                center + egui::vec2(x, y) * radius,
                shade(x * 0.999, y * 0.999),
            );
        }
    }
    let at = |ring: usize, s: usize| -> u32 {
        if ring == 0 {
            0
        } else {
            (1 + (ring - 1) * SEGMENTS + s % SEGMENTS) as u32
        }
    };
    for ring in 0..RINGS {
        for s in 0..SEGMENTS {
            if ring == 0 {
                mesh.add_triangle(0, at(1, s), at(1, s + 1));
            } else {
                mesh.add_triangle(at(ring, s), at(ring + 1, s), at(ring + 1, s + 1));
                mesh.add_triangle(at(ring, s), at(ring + 1, s + 1), at(ring, s + 1));
            }
        }
    }
    painter.add(egui::Shape::mesh(mesh));
}

/// Linear radiance of a sphere's surface with view-space normal `n` (+z towards the
/// viewer, +y up) under a studio of one soft key, a fill and a grey sky.
fn sphere_color(a: &Appearance, n: [f32; 3]) -> [f32; 3] {
    let base = a.color.to_linear();
    let dot = |p: [f32; 3], q: [f32; 3]| p[0] * q[0] + p[1] * q[1] + p[2] * q[2];
    let norm = |p: [f32; 3]| {
        let l = dot(p, p).sqrt();
        [p[0] / l, p[1] / l, p[2] / l]
    };
    let key = norm([-0.55, 0.6, 0.6]);
    let fill = norm([0.7, 0.1, 0.5]);
    let v = [0.0, 0.0, 1.0];
    let ndv = n[2].max(0.0);
    // Reflection of the view about the normal, and the "sky" it sees: bright above,
    // dark below, the key box a bright patch.
    let r = [2.0 * ndv * n[0], 2.0 * ndv * n[1], 2.0 * ndv * n[2] - 1.0];
    let sky = 0.25 + 0.55 * (0.5 + 0.5 * r[1]);
    let box_hit = |l: [f32; 3], width: f32| {
        let d = dot(norm(r), l);
        let edge = (1.0 - width * (0.3 + a.roughness * 2.5)).min(0.999);
        ((d - edge) / (1.0 - edge)).clamp(0.0, 1.0)
    };
    let rough = a.roughness;
    let env = sky * (1.0 - rough)
        + 0.45 * rough
        + 6.0 * box_hit(key, 0.06) * (1.0 - rough * 0.8)
        + 2.0 * box_hit(fill, 0.04) * (1.0 - rough * 0.8);
    let fresnel = (1.0 - ndv).powi(5);
    let m = a.metallic;
    let diffuse_light = 0.3 + 1.1 * dot(n, key).max(0.0) + 0.35 * dot(n, fill).max(0.0);
    let h = norm([key[0] + v[0], key[1] + v[1], key[2] + v[2]]);
    let shininess = 2.0 / (rough.powi(4) + 0.002);
    let highlight = dot(n, h).max(0.0).powf(shininess) * (shininess + 8.0) / 60.0;
    let coat = a.clearcoat * (0.04 + 0.96 * fresnel) * (sky + 6.0 * box_hit(key, 0.03));
    let mut out = [0.0; 3];
    for i in 0..3 {
        let f0 = 0.04 * (1.0 - m) + base[i] * m;
        let spec = (f0 + (1.0 - f0) * fresnel) * (env + highlight.min(8.0) * 0.3);
        let diffuse = base[i] * (1.0 - m) * diffuse_light * (1.0 - a.transmission * 0.7);
        out[i] = diffuse + spec + coat + base[i] * a.emission * 0.4;
    }
    out
}

/// A strip running from the environment's floor colour through its horizon to its
/// zenith, with its lights' colour as a dot: how a list of skies is told apart.
fn environment_swatch(ui: &mut egui::Ui, kind: EnvironmentKind) {
    let (rect, _) = ui.allocate_exact_size(egui::vec2(28.0, 16.0), egui::Sense::hover());
    let lighting = basset_render::SceneSettings {
        environment: kind,
        ..SceneSettings::default()
    }
    .lighting();
    let color = |c: [f32; 3]| {
        let [r, g, b] = basset_render::color::tone_map(c);
        egui::Color32::from_rgb(r, g, b)
    };
    let mut mesh = egui::Mesh::default();
    let (top, mid, bottom) = (
        color(lighting.zenith),
        color(lighting.horizon),
        color(lighting.nadir),
    );
    let ys = [rect.top(), rect.center().y, rect.bottom()];
    for (y, c) in ys.iter().zip([top, mid, bottom]) {
        mesh.colored_vertex(egui::pos2(rect.left(), *y), c);
        mesh.colored_vertex(egui::pos2(rect.right(), *y), c);
    }
    for i in 0..2u32 {
        let k = i * 2;
        mesh.add_triangle(k, k + 1, k + 3);
        mesh.add_triangle(k, k + 3, k + 2);
    }
    ui.painter().add(egui::Shape::mesh(mesh));
    if let Some(l) = lighting.lights.iter().max_by(|a, b| {
        basset_render::color::luminance(a.irradiance())
            .total_cmp(&basset_render::color::luminance(b.irradiance()))
    }) {
        let peak = l.radiance[0]
            .max(l.radiance[1])
            .max(l.radiance[2])
            .max(1e-6);
        let c = l.radiance.map(|v| v / peak);
        ui.painter().circle_filled(
            rect.left_top() + egui::vec2(20.0, 5.0),
            2.5,
            color(c.map(|v| v * 4.0)),
        );
    }
}

/// The body a face belongs to, for the hover highlight in Bodies mode: every one of its
/// faces lights, since the click paints them all.
pub fn hovered_body(editor: &Editor) -> Option<BodyRef> {
    if editor.workspace != Workspace::Render
        || editor.render.brush.is_none()
        || editor.render.apply_to != ApplyTo::Bodies
    {
        return None;
    }
    match &editor.hover {
        Some(Pick::Face(f, _)) => Some(f.body),
        _ => None,
    }
}
