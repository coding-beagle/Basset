//! What a study draws on the body: where its loads push, where it is held, and after a
//! run where the answer peaks — plus the manipulator a force is dragged by.
//!
//! A highlighted face says *which* face is loaded and nothing about how. A force of
//! `(0, 0, -100)` typed into three boxes is unambiguous and still wrong half the time,
//! because the user was thinking of "down" in the view and the face normal in the model,
//! and nothing on screen said which way the number pointed. Every CAD study view answers
//! that with arrows on the loaded faces and ground marks on the held ones, and this
//! module is that: all of it derived from the [`Simulation`] every frame, so the arrows
//! can never disagree with the boxes.
//!
//! The force manipulator borrows the move gizmo's grips and projection
//! (`gizmo::grip_drag`, `gizmo::along_axis`) rather than copying them, because it is the
//! same gesture — a grip dragged along an axis as that axis appears on screen — driving a
//! different number. A pixel of drag has to mean *something* in newtons, and the rule
//! chosen is that one arm's length is the force's present magnitude: dragging the
//! resultant's grip out to twice its length doubles the load, and an axis grip moves its
//! component at the same rate, so the handle feels the same on a 10 N study and a
//! 10 kN one. The results markers are positioned on the *deformed* plot, which is where
//! the user is looking, so the stress maximum is moved by the mean displacement of its
//! brick scaled as the plot is.
//!
//! Only the grips are egui widgets and only the labels are egui text; the arrows and
//! glyphs are scene geometry so they sit in 3D with the body. Nothing here is drawn in
//! the Design workspace: [`Editor::study_view`] is `None` there, and that is the one
//! question every entry point asks first.

use basset_core::BodyRef;
use basset_kernel::{Face, FaceKey};
use basset_math::Vec3;
use basset_viewport::{Camera, LineBatch};

use super::Editor;
use super::gizmo::{self, ARM_PX, Axis};
use super::simulate::{LoadForm, Simulation};
use super::snap::Hint;

/// The loads' orange, opaque: the face fill uses the same hue through a depth-tested
/// wash, and a line in it has to read against that fill.
pub(super) const LOAD_ARROW: [f32; 4] = [1.0, 0.55, 0.15, 1.0];
/// The held faces' blue, which the ground marks and the reaction arrow share: the
/// reaction is what the held faces push back with.
pub(super) const FIXED_MARK: [f32; 4] = [0.30, 0.55, 1.0, 1.0];
/// Where a result peaks. Neither the load nor the fixed colour, because a maximum is a
/// finding and not a setting.
pub(super) const RESULT_MARK: [f32; 4] = [1.0, 0.95, 0.75, 1.0];

/// The resultant and the reaction arrows are the gizmo's arm long; the three axis arrows
/// of the manipulator are shorter so their grips never land on the resultant's.
const AXIS_PX: f64 = 50.0;
/// The arrows spread over a large face, and the pressure arrows: short, because there
/// are many of them and they are saying "here too", not "this much".
const SPREAD_PX: f64 = 36.0;
/// Half the size of a result marker's cross, and the size of a ground mark.
const MARK_PX: f64 = 8.0;
const GROUND_PX: f64 = 12.0;
/// The force a drag is measured against when the present one is too small to be a
/// scale: a study at 0 N would otherwise have a handle that cannot be moved.
const LEAST_REFERENCE_N: f64 = 10.0;
/// A force is dragged in whole newtons while the grid holds, and freely under shift as
/// every other handle is.
const FORCE_STEP_N: f64 = 1.0;

/// A line of text hung on a point of the model.
pub(super) struct Label {
    pub at: Vec3,
    pub text: String,
}

/// The force manipulator as it stands this frame: an arrow per axis at the loaded faces'
/// combined centroid, and the resultant with a grip at its free end.
pub(super) struct ForceGizmo {
    pub origin: Vec3,
    /// The resultant's unit direction, the way the force points.
    pub dir: Vec3,
    pub magnitude: f64,
    /// Which way along `dir` the resultant's shaft leaves `origin`: `-1` when the force
    /// pushes into the face, so the arrow stands outside the body with its head on the
    /// face, and `+1` when it pulls away.
    pub outward: f64,
    /// World length of an axis arrow and of the resultant, at this zoom.
    pub axis_arm: f64,
    pub arm: f64,
}

impl ForceGizmo {
    /// Where the resultant's grip sits: the end of the shaft away from the face.
    pub fn resultant_grip(&self) -> Vec3 {
        self.origin + self.dir * (self.outward * self.arm)
    }

    pub fn axis_tip(&self, axis: Axis) -> Vec3 {
        self.origin + axis_dir(axis) * self.axis_arm
    }
}

fn axis_dir(axis: Axis) -> Vec3 {
    match axis {
        Axis::X => Vec3::X,
        Axis::Y => Vec3::Y,
        Axis::Z => Vec3::Z,
    }
}

const AXES: [(Axis, [f32; 4]); 3] = [
    (Axis::X, [0.93, 0.35, 0.35, 1.0]),
    (Axis::Y, [0.45, 0.85, 0.42, 1.0]),
    (Axis::Z, [0.40, 0.60, 0.98, 1.0]),
];

/// The study the viewport is showing, with the solid it is written on. `None` in Design,
/// and when the study has no body yet.
fn study(editor: &Editor) -> Option<(&Simulation, BodyRef, &basset_kernel::Solid)> {
    let sim = editor.study_view()?;
    let body = sim.body?;
    if editor.hidden_bodies.contains(&body) {
        return None;
    }
    let pick = editor.pick_body(body)?;
    Some((sim, body, &pick.solid))
}

/// The faces of `keys` that the solid still has. A key the model no longer answers to
/// is simply not drawn; the study tree is where it is reported.
fn faces<'a>(solid: &'a basset_kernel::Solid, keys: &[FaceKey]) -> Vec<&'a Face> {
    keys.iter().filter_map(|k| solid.face(*k)).collect()
}

/// Area-weighted centroid of several faces, which is where one arrow standing for all
/// of them belongs.
fn combined_centroid(faces: &[&Face]) -> Option<Vec3> {
    let mut sum = Vec3::ZERO;
    let mut total = 0.0;
    for f in faces {
        let a = f.area();
        sum += f.centroid() * a;
        total += a;
    }
    (total > 0.0).then(|| sum / total)
}

/// The mean outward normal of several faces, for deciding whether a force pushes into
/// them or pulls away. Zero when they cancel, in which case the force is taken to pull.
fn combined_normal(faces: &[&Face]) -> Vec3 {
    faces
        .iter()
        .flat_map(|f| f.polygons.iter().map(|p| p.plane.normal * p.area()))
        .sum::<Vec3>()
        .normalize_or_zero()
}

/// The force manipulator, when there is a force on at least one face.
pub(super) fn manipulator(editor: &Editor) -> Option<ForceGizmo> {
    let (sim, _, solid) = study(editor)?;
    if sim.load_form != LoadForm::Force {
        return None;
    }
    let loaded = faces(solid, &sim.loaded);
    let origin = combined_centroid(&loaded)?;
    let magnitude = sim.force.length();
    // A zero force has no direction of its own; the arrow then stands against the face
    // the way a push would, so the grip is somewhere to grab.
    let normal = combined_normal(&loaded);
    let dir = if magnitude > 0.0 {
        sim.force / magnitude
    } else if normal != Vec3::ZERO {
        -normal
    } else {
        -Vec3::Z
    };
    let outward = if dir.dot(normal) < 0.0 { -1.0 } else { 1.0 };
    let px = editor.camera.pixel_size_at(origin, editor.window_px);
    Some(ForceGizmo {
        origin,
        dir,
        magnitude,
        outward,
        axis_arm: px * AXIS_PX,
        arm: px * ARM_PX,
    })
}

// --- Scene geometry -------------------------------------------------------------------

/// Every line the study draws this frame. Empty outside the Simulation workspace.
pub(super) fn lines(editor: &Editor) -> Vec<LineBatch> {
    let Some((sim, _, solid)) = study(editor) else {
        return Vec::new();
    };
    let camera = &editor.camera;
    let window = editor.window_px;
    let mut out = Vec::new();

    // Loads: an arrow wherever one acts.
    let loaded = faces(solid, &sim.loaded);
    if !loaded.is_empty() {
        let mut arrows = LineBatch::new(LOAD_ARROW);
        arrows.width_px = 2.0;
        arrows.depth_test = false;
        match sim.load_form {
            LoadForm::Force => {
                if let Some(g) = manipulator(editor) {
                    // The spread arrows: one per polygon, shorter than the resultant,
                    // skipped where they would only redraw it.
                    for face in &loaded {
                        for poly in &face.polygons {
                            let at = poly.centroid();
                            if at.distance(g.origin) < g.arm * 0.05 {
                                continue;
                            }
                            let px = camera.pixel_size_at(at, window);
                            let len = px * SPREAD_PX;
                            let outward = if g.dir.dot(poly.plane.normal) < 0.0 {
                                -1.0
                            } else {
                                1.0
                            };
                            let (tail, head) = if outward < 0.0 {
                                (at - g.dir * len, at)
                            } else {
                                (at, at + g.dir * len)
                            };
                            push_arrow(&mut arrows, camera, window, tail, head);
                        }
                    }
                    // The resultant, which the manipulator's grip sits on the end of.
                    let free = g.resultant_grip();
                    let (tail, head) = if g.outward < 0.0 {
                        (free, g.origin)
                    } else {
                        (g.origin, free)
                    };
                    let mut resultant = LineBatch::new(LOAD_ARROW);
                    resultant.width_px = 3.0;
                    resultant.depth_test = false;
                    push_arrow(&mut resultant, camera, window, tail, head);
                    out.push(resultant);
                    // The axes it can be dragged along.
                    for (axis, color) in AXES {
                        let mut arrow = LineBatch::new(color);
                        arrow.width_px = 2.0;
                        arrow.depth_test = false;
                        push_arrow(&mut arrow, camera, window, g.origin, g.axis_tip(axis));
                        out.push(arrow);
                    }
                }
            }
            LoadForm::Pressure => {
                // One arrow per polygon along its normal: into the face for a positive
                // pressure, out of it for suction.
                let sign = if sim.pressure >= 0.0 { -1.0 } else { 1.0 };
                for face in &loaded {
                    for poly in &face.polygons {
                        let at = poly.centroid();
                        let dir = poly.plane.normal * sign;
                        let len = camera.pixel_size_at(at, window) * SPREAD_PX;
                        let (tail, head) = if sign < 0.0 {
                            (at - dir * len, at)
                        } else {
                            (at, at + dir * len)
                        };
                        push_arrow(&mut arrows, camera, window, tail, head);
                    }
                }
            }
        }
        out.push(arrows);
    }

    // Held faces: a ground mark on each.
    let fixed = faces(solid, &sim.fixed);
    if !fixed.is_empty() {
        let mut marks = LineBatch::new(FIXED_MARK);
        marks.width_px = 1.5;
        marks.depth_test = false;
        for face in &fixed {
            let at = face.centroid();
            let normal = combined_normal(&[face]);
            push_ground(&mut marks, camera, window, at, normal);
        }
        out.push(marks);
    }

    // The answer, on the deformed plot.
    if let Some((_, outcome)) = sim.plotted(editor.doc.revision()) {
        let mut marks = LineBatch::new(RESULT_MARK);
        marks.width_px = 1.5;
        marks.depth_test = false;
        for (at, _) in peaks(&outcome.results, sim.scale) {
            push_cross(&mut marks, camera, window, at);
        }
        out.push(marks);
        if let Some(origin) = combined_centroid(&fixed) {
            let reaction = outcome.results.reaction;
            if reaction.length() > 0.0 {
                let mut arrow = LineBatch::new(FIXED_MARK);
                arrow.width_px = 3.0;
                arrow.depth_test = false;
                let dir = reaction.normalize();
                let len = camera.pixel_size_at(origin, window) * ARM_PX;
                // Drawn leaving the face it acts on, head out, so it reads as the push
                // back that it is.
                push_arrow(&mut arrow, camera, window, origin, origin + dir * len);
                out.push(arrow);
            }
        }
    }

    out.retain(|b| !b.segments.is_empty());
    out
}

/// The maxima a run found, each moved as the plot moves it, with the text that names it.
fn peaks(results: &basset_fea::Results, scale: f64) -> Vec<(Vec3, String)> {
    let mut out = Vec::new();
    let mesh = &results.mesh;
    // The stress is read at a brick's centre, which the plot moves by the mean of the
    // brick's corner displacements.
    let (stress, _) = results.max_von_mises();
    if let Some(e) = (0..results.von_mises.len())
        .max_by(|a, b| results.von_mises[*a].total_cmp(&results.von_mises[*b]))
        && let Some(nodes) = mesh.elements.get(e)
    {
        let shift = nodes
            .iter()
            .filter_map(|n| results.displacements.get(*n))
            .sum::<Vec3>()
            / 8.0;
        out.push((
            mesh.element_centre(e) + shift * scale,
            format!("max {stress:.1} MPa"),
        ));
    }
    if let Some(n) = (0..results.displacements.len()).max_by(|a, b| {
        results.displacements[*a]
            .length()
            .total_cmp(&results.displacements[*b].length())
    }) && let (Some(at), Some(d)) = (mesh.nodes.get(n), results.displacements.get(n))
    {
        out.push((*at + *d * scale, format!("max {:.4} mm", d.length())));
    }
    out
}

/// A shaft with a head, the head sized in pixels and turned to face the camera, as the
/// move gizmo draws its own.
fn push_arrow(batch: &mut LineBatch, camera: &Camera, window: [u32; 2], tail: Vec3, head: Vec3) {
    let dir = (head - tail).normalize_or_zero();
    if dir == Vec3::ZERO {
        return;
    }
    let px = camera.pixel_size_at(head, window);
    let side = dir.cross(camera.forward()).normalize_or_zero();
    let back = dir * (10.0 * px);
    let wing = side * (4.0 * px);
    batch.segments.push([tail, head]);
    batch.segments.push([head, head - back + wing]);
    batch.segments.push([head, head - back - wing]);
}

/// A ground symbol: a triangle standing on the face with its apex at `at`, a base line
/// under it and hatching beneath, the way a fixed support is drawn in every statics
/// text. Built in the plane the face normal and the view share, so it faces the camera
/// as far as the face allows.
fn push_ground(batch: &mut LineBatch, camera: &Camera, window: [u32; 2], at: Vec3, normal: Vec3) {
    let px = camera.pixel_size_at(at, window) * GROUND_PX;
    let n = if normal == Vec3::ZERO {
        -camera.forward()
    } else {
        normal
    };
    let mut side = n.cross(camera.forward()).normalize_or_zero();
    if side == Vec3::ZERO {
        side = n.cross(Vec3::Z).normalize_or_zero();
        if side == Vec3::ZERO {
            side = Vec3::X;
        }
    }
    let base = at + n * px;
    let left = base - side * (px * 0.7);
    let right = base + side * (px * 0.7);
    batch.segments.push([at, left]);
    batch.segments.push([at, right]);
    batch
        .segments
        .push([base - side * (px * 1.2), base + side * (px * 1.2)]);
    // Three slanted hatches under the base line.
    for i in -1..=1 {
        let foot = base + side * (px * 0.6 * f64::from(i)) + n * (px * 0.6);
        batch
            .segments
            .push([foot, foot + side * (px * 0.4) - n * (px * 0.6)]);
    }
}

/// A small cross in the screen plane, pinning a point without hiding it.
fn push_cross(batch: &mut LineBatch, camera: &Camera, window: [u32; 2], at: Vec3) {
    let px = camera.pixel_size_at(at, window) * MARK_PX;
    let right = camera.right() * px;
    let up = camera.up() * px;
    batch.segments.push([at - right, at + right]);
    batch.segments.push([at - up, at + up]);
}

// --- Labels ---------------------------------------------------------------------------

/// The text the study hangs on the model this frame: the load's size by its resultant,
/// the maxima by their markers, the reaction by its arrow.
pub(super) fn labels(editor: &Editor) -> Vec<Label> {
    let Some((sim, _, solid)) = study(editor) else {
        return Vec::new();
    };
    let mut out = Vec::new();
    let loaded = faces(solid, &sim.loaded);
    match sim.load_form {
        LoadForm::Force => {
            if let Some(g) = manipulator(editor) {
                out.push(Label {
                    at: g.resultant_grip(),
                    text: newtons(g.magnitude),
                });
            }
        }
        LoadForm::Pressure => {
            if let Some(at) = combined_centroid(&loaded) {
                out.push(Label {
                    at,
                    text: format!("{} MPa", trimmed(sim.pressure, 2)),
                });
            }
        }
    }
    if let Some((_, outcome)) = sim.plotted(editor.doc.revision()) {
        for (at, text) in peaks(&outcome.results, sim.scale) {
            out.push(Label { at, text });
        }
        if let Some(origin) = combined_centroid(&faces(solid, &sim.fixed)) {
            let reaction = outcome.results.reaction;
            if reaction.length() > 0.0 {
                let len = editor.camera.pixel_size_at(origin, editor.window_px) * ARM_PX;
                out.push(Label {
                    at: origin + reaction.normalize() * len,
                    text: format!("reaction {}", newtons(reaction.length())),
                });
            }
        }
    }
    out
}

/// A force as a person reads it: whole newtons when it is one, else to a tenth.
fn newtons(n: f64) -> String {
    if (n - n.round()).abs() < 0.05 {
        format!("{n:.0} N")
    } else {
        format!("{n:.1} N")
    }
}

/// `value` to `decimals` places with the trailing zeros taken off, so a pressure typed
/// as 2.5 reads back as 2.5 and not 2.50.
fn trimmed(value: f64, decimals: usize) -> String {
    let s = format!("{value:.decimals$}");
    if s.contains('.') {
        s.trim_end_matches('0').trim_end_matches('.').to_owned()
    } else {
        s
    }
}

/// Draws the labels beside the points they belong to. Not interactable, so a label
/// sitting over the body never takes a click meant for a face under it.
pub(super) fn overlay(editor: &Editor, ctx: &egui::Context) {
    let labels = labels(editor);
    let ppp = ctx.pixels_per_point();
    for (i, label) in labels.iter().enumerate() {
        let Some(px) = editor.camera.world_to_screen(label.at, editor.window_px) else {
            continue;
        };
        let pos = egui::pos2(px[0] as f32 / ppp + 12.0, px[1] as f32 / ppp + 10.0);
        egui::Area::new(egui::Id::new(("study-mark", i)))
            .fixed_pos(pos)
            .order(egui::Order::Foreground)
            .interactable(false)
            .show(ctx, |ui| {
                egui::Frame::popup(ui.style()).show(ui, |ui| {
                    ui.label(egui::RichText::new(&label.text).strong());
                });
            });
    }
}

// --- The manipulator's grips ----------------------------------------------------------

/// Draws the force manipulator's grips and applies whatever was dragged. Returns whether
/// the force changed.
///
/// Runs after [`gizmo::interact`], which has already released the running drag totals
/// when no button is down, so this only has to add to them.
pub(super) fn interact(editor: &mut Editor, ctx: &egui::Context) -> bool {
    let Some(g) = manipulator(editor) else {
        return false;
    };
    let camera = editor.camera;
    let window = editor.window_px;
    let snap = editor.snapping.at(FORCE_STEP_N);
    // Newtons per world unit of drag: an arm's length is the force there is.
    let reference = g.magnitude.max(LEAST_REFERENCE_N);
    let mut changed = false;
    for (axis, color) in AXES {
        let tip = g.axis_tip(axis);
        let Some(delta) = gizmo::grip_drag(ctx, &camera, window, ("study-force", axis), tip, color)
        else {
            continue;
        };
        let Some(world) = gizmo::along_axis(&camera, window, tip, axis_dir(axis), delta) else {
            continue;
        };
        let by = world / g.axis_arm * reference;
        let value = nudge_component(editor, axis, by, snap);
        editor.snap_hint = Some(Hint::value(tip, value, " N", snap));
        changed = true;
    }
    let grip = g.resultant_grip();
    if let Some(delta) = gizmo::grip_drag(
        ctx,
        &camera,
        window,
        ("study-force-resultant", Axis::X),
        grip,
        LOAD_ARROW,
    ) && let Some(world) = gizmo::along_axis(&camera, window, grip, g.dir * g.outward, delta)
    {
        let by = world / g.arm * reference;
        let value = nudge_magnitude(editor, g.dir, by, snap);
        editor.snap_hint = Some(Hint::value(grip, value, " N", snap));
        changed = true;
    }
    if changed {
        editor.request_repaint();
    }
    changed
}

/// Adds `by` newtons to one component of the study's force, snapped, and hands back the
/// component as it now reads. The one place the manipulator writes the study.
pub(super) fn nudge_component(
    editor: &mut Editor,
    axis: Axis,
    by: f64,
    snap: super::snap::Snap,
) -> f64 {
    let drag = &mut editor.drags.force[axis as usize];
    let Some(sim) = editor.simulation.as_mut() else {
        return 0.0;
    };
    let component = match axis {
        Axis::X => &mut sim.force.x,
        Axis::Y => &mut sim.force.y,
        Axis::Z => &mut sim.force.z,
    };
    *component = drag.advance(*component, by, snap);
    *component
}

/// Lengthens the force along `dir` by `by` newtons, snapped, keeping its direction, and
/// hands back the magnitude it now reads. Dragged back through zero the force reverses
/// rather than stopping, which is how a push becomes a pull without retyping three
/// signs.
pub(super) fn nudge_magnitude(
    editor: &mut Editor,
    dir: Vec3,
    by: f64,
    snap: super::snap::Snap,
) -> f64 {
    let drag = &mut editor.drags.force[3];
    let Some(sim) = editor.simulation.as_mut() else {
        return 0.0;
    };
    let signed = sim.force.dot(dir);
    let next = drag.advance(signed, by, snap);
    sim.force = dir * next;
    next.abs()
}
