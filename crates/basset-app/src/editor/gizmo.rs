//! The transform manipulator: the arrows and rings a move is dragged by.
//!
//! Typing a move into boxes is exact and is what a drawing calls for, but it is not how
//! anyone finds out *where* something should go. Fusion answers that with a manipulator
//! on the thing being moved: an arrow per direction it can travel and a ring per axis it
//! can turn about, dragged straight in the viewport. This is that, for both kinds of
//! move the editor has — a sketch selection under `M`, and a body under the Move tool —
//! because they are the same gesture and deserve the same handle.
//!
//! The manipulator drives the same numbers the palette shows, so dragging and typing are
//! two ways of saying one thing and the boxes always read what the viewport is showing.
//! Only the grips are egui widgets; the shafts and rings are drawn by the scene so they
//! sit in 3D with the geometry.
//!
//! A [`Slider`] is the other shape of handle: one grip sitting *at* the value rather
//! than on an arm of fixed length, so dragging it is dragging the number itself. An
//! offset's distance is one — the result is already drawn at that distance, so that is
//! where the handle belongs, and dragging it back through the geometry and out the far
//! side is how the side gets chosen.

use basset_math::{Vec2, Vec3};
use basset_viewport::Camera;

use super::snap::{Hint, Snap};
use super::tools::ToolKind;
use super::{Editor, Mode};

/// Screen length of an arrow, and radius of a ring, in pixels. Constant on screen so the
/// manipulator is the same size to grab whatever the zoom is.
const ARM_PX: f64 = 78.0;
const RING_PX: f64 = 58.0;
/// Size of a grip's clickable square, in egui points.
const GRIP: f32 = 20.0;

const X_COLOR: [f32; 4] = [0.93, 0.35, 0.35, 1.0];
/// The offset handle, in the same blue its preview is drawn in.
const SLIDE_COLOR: [f32; 4] = [0.55, 0.85, 1.0, 1.0];
const Y_COLOR: [f32; 4] = [0.45, 0.85, 0.42, 1.0];
const Z_COLOR: [f32; 4] = [0.40, 0.60, 0.98, 1.0];

/// One direction the move can travel along.
#[derive(Clone, Copy, Debug)]
pub struct Arrow {
    pub axis: Axis,
    /// Unit direction in world space; a positive drag moves this way.
    pub dir: Vec3,
    pub color: [f32; 4],
}

/// One axis the move can turn about.
#[derive(Clone, Copy, Debug)]
pub struct Ring {
    pub axis: Axis,
    /// Unit normal of the ring's plane in world space.
    pub normal: Vec3,
    /// Where the ring's grip sits, as a unit vector in the ring's plane.
    pub grip: Vec3,
    pub color: [f32; 4],
}

/// Which of a transform's three components a handle drives. Named rather than indexed so
/// the sketch case, which has no Z arrow and only one ring, cannot quietly drive the
/// wrong one.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Axis {
    X,
    Y,
    Z,
}

/// A single grip that sits at the value it drives. `anchor` is where the value is zero
/// and `dir` the direction it grows along, so the grip is at `anchor + dir * value` and
/// a drag of the grip along `dir` *is* the value.
pub struct Slider {
    pub anchor: Vec3,
    pub dir: Vec3,
    pub value: f64,
    pub color: [f32; 4],
}

impl Slider {
    pub fn grip(&self) -> Vec3 {
        self.anchor + self.dir * self.value
    }
}

/// The slider the editor should be showing, or `None` when nothing has one.
///
/// Two operations have one, and never at the same time: only one modal operation runs
/// in a sketch at once.
pub fn slider(editor: &Editor) -> Option<Slider> {
    let Mode::Sketch(s) = &editor.mode else {
        return None;
    };
    // A fillet's radius is dragged on the corner it rounds: the handle sits the radius
    // out along the bisector, so the distance from the corner to the grip is the number.
    let (anchor, dir, value) = match (s.offset_handle(), s.fillet_handle()) {
        (Some((anchor, dir)), _) => (anchor, dir, s.offset.distance),
        (_, Some((anchor, dir))) => (anchor, dir, s.fillet.radius),
        _ => return None,
    };
    let frame = &s.frame;
    Some(Slider {
        anchor: frame.to_world(anchor),
        // A direction, not a point, so it is built from the frame's axes rather than
        // being mapped through the origin.
        dir: frame.x * dir.x + frame.y * dir.y,
        value,
        color: SLIDE_COLOR,
    })
}

/// The manipulator for whatever is being moved right now.
pub struct Gizmo {
    pub origin: Vec3,
    pub arrows: Vec<Arrow>,
    pub rings: Vec<Ring>,
}

/// The manipulator the editor should be showing, or `None` when nothing is being moved.
///
/// A sketch move happens on one plane, so it gets the plane's two arrows and the one
/// ring that turns within it; there is no third direction to offer and offering it would
/// only invite a move off the sketch.
pub fn current(editor: &Editor) -> Option<Gizmo> {
    if let Mode::Sketch(s) = &editor.mode {
        let pivot = s.move_pivot()?;
        let frame = &s.frame;
        return Some(Gizmo {
            origin: frame.to_world(pivot),
            arrows: vec![
                Arrow {
                    axis: Axis::X,
                    dir: frame.x,
                    color: X_COLOR,
                },
                Arrow {
                    axis: Axis::Y,
                    dir: frame.y,
                    color: Y_COLOR,
                },
            ],
            rings: vec![Ring {
                axis: Axis::Z,
                normal: frame.z,
                grip: frame.x,
                color: Z_COLOR,
            }],
        });
    }
    let tool = editor.tool.as_ref()?;
    if tool.kind != ToolKind::Move {
        return None;
    }
    let body = editor.selection.bodies.first()?;
    let aabb = editor.pick_body(*body)?.solid.aabb();
    if aabb.is_empty() {
        return None;
    }
    // The manipulator sits on the body where it is now, translation included, so it
    // follows the preview rather than staying behind at the original.
    let origin = (aabb.min + aabb.max) * 0.5 + tool.params.translate;
    let axes = [
        (Axis::X, Vec3::X, X_COLOR),
        (Axis::Y, Vec3::Y, Y_COLOR),
        (Axis::Z, Vec3::Z, Z_COLOR),
    ];
    Some(Gizmo {
        origin,
        arrows: axes
            .iter()
            .map(|(axis, dir, color)| Arrow {
                axis: *axis,
                dir: *dir,
                color: *color,
            })
            .collect(),
        // Each ring's grip sits on the next axis round, so the three grips never stack.
        rings: vec![
            Ring {
                axis: Axis::X,
                normal: Vec3::X,
                grip: Vec3::Y,
                color: X_COLOR,
            },
            Ring {
                axis: Axis::Y,
                normal: Vec3::Y,
                grip: Vec3::Z,
                color: Y_COLOR,
            },
            Ring {
                axis: Axis::Z,
                normal: Vec3::Z,
                grip: Vec3::X,
                color: Z_COLOR,
            },
        ],
    })
}

impl Gizmo {
    /// World length of an arm, so the manipulator keeps its size on screen.
    pub fn arm(&self, camera: &Camera, window: [u32; 2]) -> f64 {
        camera.pixel_size_at(self.origin, window) * ARM_PX
    }

    pub fn radius(&self, camera: &Camera, window: [u32; 2]) -> f64 {
        camera.pixel_size_at(self.origin, window) * RING_PX
    }

    /// The points of one ring, for drawing it.
    pub fn ring_points(&self, ring: &Ring, radius: f64) -> Vec<Vec3> {
        const STEPS: usize = 48;
        let u = ring.grip;
        let v = ring.normal.cross(u);
        (0..=STEPS)
            .map(|i| {
                let a = std::f64::consts::TAU * i as f64 / STEPS as f64;
                self.origin + (u * a.cos() + v * a.sin()) * radius
            })
            .collect()
    }
}

/// Draws the grips and applies whatever was dragged. Returns whether anything moved.
pub fn interact(editor: &mut Editor, ctx: &egui::Context) -> bool {
    // egui owns the modifier state during a drag, and the snapping has to read the same
    // shift the user is holding as they drag. The sketch keeps its own copy because the
    // drawing path comes from winit and cannot see egui's.
    let shift = ctx.input(|i| i.modifiers.shift);
    editor.snapping.free = shift;
    if let Mode::Sketch(s) = &mut editor.mode {
        s.set_free_snap(shift);
    }
    // A gesture that has ended must not bank its travel into the next one: a running
    // total is only meaningful while the button is down. Done here rather than after the
    // handles because the slider returns early, and a total left behind there would be
    // added to the next drag of the same grip.
    if ctx.input(|i| !i.pointer.any_down()) {
        editor.drags.release();
        if let Mode::Sketch(s) = &mut editor.mode {
            s.release_drags();
        }
    }
    let camera = editor.camera;
    let window = editor.window_px;
    let mut slid = false;
    if let Some(slider) = slider(editor) {
        let grip = slider.grip();
        if let Some(delta) = grip_drag(
            ctx,
            &camera,
            window,
            ("gizmo-slider", Axis::X),
            grip,
            slider.color,
        ) && let Some(world) = along_axis(&camera, window, grip, slider.dir, delta)
            && let Some(value) = slide(editor, world)
        {
            slid = true;
            // The grip sits *at* the value, so the hint belongs where it has landed, not
            // where it was grabbed.
            let at = slider.anchor + slider.dir * value;
            editor.snap_hint = Some(Hint::value(at, value, " mm", editor.snap_at(at)));
        }
    }
    let Some(gizmo) = current(editor) else {
        if slid {
            apply(editor);
        }
        return slid;
    };
    let arm = gizmo.arm(&camera, window);
    let radius = gizmo.radius(&camera, window);
    let mut moved = slid;
    for arrow in &gizmo.arrows {
        let tip = gizmo.origin + arrow.dir * arm;
        let Some(delta) = grip_drag(
            ctx,
            &camera,
            window,
            ("gizmo-arrow", arrow.axis),
            tip,
            arrow.color,
        ) else {
            continue;
        };
        // The tip tracks the pointer along the arrow, whatever angle it is seen at.
        let Some(world) = along_axis(&camera, window, tip, arrow.dir, delta) else {
            continue;
        };
        let snap = editor.snap_at(gizmo.origin);
        if let Some(value) = translate(editor, arrow.axis, world, snap) {
            moved = true;
            editor.snap_hint = Some(Hint::value(tip, value, " mm", snap));
        }
    }
    for ring in &gizmo.rings {
        let grip = gizmo.origin + ring.grip * radius;
        let tangent = ring.normal.cross(ring.grip);
        let Some(delta) = grip_drag(
            ctx,
            &camera,
            window,
            ("gizmo-ring", ring.axis),
            grip,
            ring.color,
        ) else {
            continue;
        };
        // Arc length over radius is the angle turned, which keeps the grip under the
        // pointer for small drags and stays stable for large ones.
        let Some(arc) = along_axis(&camera, window, grip, tangent, delta) else {
            continue;
        };
        let snap = editor.snap_at(gizmo.origin);
        if let Some(value) = rotate(editor, ring.axis, (arc / radius).to_degrees(), snap) {
            moved = true;
            // An angle has no grid increment to name, so the hint says the angle alone.
            editor.snap_hint = Some(Hint {
                at: grip,
                text: format!("{value:.1}\u{b0}"),
                on_grid: snap.is_on(),
            });
        }
    }
    if moved {
        apply(editor);
    }
    moved
}

/// A draggable dot at `at`, returning the drag in physical pixels when it is being
/// dragged. `None` covers both "not dragged" and "off screen".
fn grip_drag(
    ctx: &egui::Context,
    camera: &Camera,
    window: [u32; 2],
    id: (&'static str, Axis),
    at: Vec3,
    color: [f32; 4],
) -> Option<Vec2> {
    let px = camera.world_to_screen(at, window)?;
    let ppp = f64::from(ctx.pixels_per_point());
    let centre = egui::pos2((px[0] / ppp) as f32, (px[1] / ppp) as f32);
    let response = egui::Area::new(egui::Id::new(id))
        .fixed_pos(centre - egui::vec2(GRIP * 0.5, GRIP * 0.5))
        .order(egui::Order::Foreground)
        .show(ctx, |ui| {
            let (rect, response) =
                ui.allocate_exact_size(egui::vec2(GRIP, GRIP), egui::Sense::drag());
            let hot = response.hovered() || response.dragged();
            let fill = if hot {
                egui::Color32::from_rgb(255, 215, 80)
            } else {
                egui::Color32::from_rgb(
                    (color[0] * 255.0) as u8,
                    (color[1] * 255.0) as u8,
                    (color[2] * 255.0) as u8,
                )
            };
            ui.painter().circle(
                rect.center(),
                GRIP * 0.3,
                fill,
                egui::Stroke::new(1.5, egui::Color32::from_gray(20)),
            );
            if hot {
                ui.ctx().set_cursor_icon(egui::CursorIcon::Grab);
            }
            response
        })
        .inner;
    if !response.dragged() {
        return None;
    }
    let delta = response.drag_delta();
    Some(Vec2::new(
        f64::from(delta.x) * ppp,
        f64::from(delta.y) * ppp,
    ))
}

/// How far along `dir`, in world units, a drag of `delta` pixels at `at` reaches.
///
/// The conversion is the length of `dir` *as drawn*: one world unit along an axis that
/// leans away from the camera covers fewer pixels than one along an axis lying in the
/// screen plane, and dividing by the drawn length is what accounts for that. Scaling by
/// the size of a pixel instead would be right only for an axis square to the view and
/// would under-move every other one — the grip would slide out from under the pointer
/// exactly when the user has orbited to see what they are doing.
///
/// `None` when the axis points at or away from the camera, where it is drawn as a dot
/// and there is no direction on screen to drag it along.
fn along_axis(camera: &Camera, window: [u32; 2], at: Vec3, dir: Vec3, delta: Vec2) -> Option<f64> {
    let here = camera.world_to_screen(at, window)?;
    let ahead = camera.world_to_screen(at + dir, window)?;
    let drawn = Vec2::new(ahead[0] - here[0], ahead[1] - here[1]);
    let length = drawn.length();
    // Below about a pixel the direction is noise and the division explodes.
    (length > 1.0).then(|| delta.dot(drawn / length) / length)
}

/// Adds `world` to the move's offset along `axis`, snapped, and hands back the offset it
/// now reads. `None` when there is nothing to move. The sketch's axes are the sketch
/// plane's, so its X and Y are the frame's, not the model's.
///
/// A sketch applies the rule itself rather than being handed it, because the same rule
/// has to hold for the numbers typed into the palette; [`Editor::snap_at`] reports the
/// sketch's own, so the two cannot disagree.
fn translate(editor: &mut Editor, axis: Axis, world: f64, snap: Snap) -> Option<f64> {
    match &mut editor.mode {
        Mode::Sketch(s) => s.nudge_move(axis == Axis::X, world).then(|| {
            let op = s.move_op.as_ref()?;
            Some(if axis == Axis::X { op.dx } else { op.dy })
        })?,
        Mode::Model => {
            let tool = editor.tool.as_mut()?;
            let offset = match axis {
                Axis::X => &mut tool.params.translate.x,
                Axis::Y => &mut tool.params.translate.y,
                Axis::Z => &mut tool.params.translate.z,
            };
            let drag = &mut editor.drags.translate[axis as usize];
            *offset = drag.advance(*offset, world, snap);
            Some(*offset)
        }
    }
}

/// Drags whatever the slider drives by `world` units, and hands back the distance it now
/// reads.
fn slide(editor: &mut Editor, world: f64) -> Option<f64> {
    match &mut editor.mode {
        // Two modal operations own a slider, and only one of them is ever live, so the
        // first that takes the drag is the one being dragged.
        Mode::Sketch(s) => {
            if s.nudge_offset(world) {
                Some(s.offset.distance)
            } else if s.nudge_fillet(world) {
                Some(s.fillet.radius)
            } else {
                None
            }
        }
        Mode::Model => None,
    }
}

/// Turns the move by `degrees`, snapped, and hands back the angle it now reads.
fn rotate(editor: &mut Editor, axis: Axis, degrees: f64, snap: Snap) -> Option<f64> {
    match &mut editor.mode {
        // A sketch turns in its own plane and nowhere else, so its one ring is the only
        // rotation there is to drive.
        Mode::Sketch(s) => (axis == Axis::Z && s.turn_move(degrees))
            .then(|| s.move_op.as_ref().map(|op| op.angle_deg))?,
        Mode::Model => {
            let tool = editor.tool.as_mut()?;
            let angle = match axis {
                Axis::X => &mut tool.params.rotate_deg.x,
                Axis::Y => &mut tool.params.rotate_deg.y,
                Axis::Z => &mut tool.params.rotate_deg.z,
            };
            let drag = &mut editor.drags.rotate[axis as usize];
            *angle = drag.advance_angle(*angle, degrees, snap);
            Some(*angle)
        }
    }
}

/// Pushes the changed numbers through to the geometry, the way editing the boxes does.
fn apply(editor: &mut Editor) {
    match &mut editor.mode {
        Mode::Sketch(s) => {
            // Whichever operation the handle belongs to; they are never both running.
            s.update_move();
            s.update_offset();
            s.update_fillet();
            if s.take_dirty() {
                editor.commit_sketch();
            }
        }
        Mode::Model => super::tools::sync_tool(editor),
    }
    editor.request_repaint();
}
