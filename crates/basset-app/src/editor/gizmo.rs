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

use basset_math::{Vec2, Vec3};
use basset_viewport::Camera;

use super::tools::ToolKind;
use super::{Editor, Mode};

/// Screen length of an arrow, and radius of a ring, in pixels. Constant on screen so the
/// manipulator is the same size to grab whatever the zoom is.
const ARM_PX: f64 = 78.0;
const RING_PX: f64 = 58.0;
/// Size of a grip's clickable square, in egui points.
const GRIP: f32 = 20.0;

const X_COLOR: [f32; 4] = [0.93, 0.35, 0.35, 1.0];
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
    let Some(gizmo) = current(editor) else {
        return false;
    };
    let camera = editor.camera;
    let window = editor.window_px;
    let arm = gizmo.arm(&camera, window);
    let radius = gizmo.radius(&camera, window);
    let mut moved = false;
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
        moved |= translate(editor, arrow.axis, world);
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
        moved |= rotate(editor, ring.axis, (arc / radius).to_degrees());
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

/// Adds `world` to the move's offset along `axis`. The sketch's axes are the sketch
/// plane's, so its X and Y are the frame's, not the model's.
fn translate(editor: &mut Editor, axis: Axis, world: f64) -> bool {
    match &mut editor.mode {
        Mode::Sketch(s) => s.nudge_move(axis == Axis::X, world),
        Mode::Model => {
            let Some(tool) = editor.tool.as_mut() else {
                return false;
            };
            match axis {
                Axis::X => tool.params.translate.x += world,
                Axis::Y => tool.params.translate.y += world,
                Axis::Z => tool.params.translate.z += world,
            }
            true
        }
    }
}

fn rotate(editor: &mut Editor, axis: Axis, degrees: f64) -> bool {
    match &mut editor.mode {
        // A sketch turns in its own plane and nowhere else, so its one ring is the only
        // rotation there is to drive.
        Mode::Sketch(s) => axis == Axis::Z && s.turn_move(degrees),
        Mode::Model => {
            let Some(tool) = editor.tool.as_mut() else {
                return false;
            };
            match axis {
                Axis::X => tool.params.rotate_deg.x += degrees,
                Axis::Y => tool.params.rotate_deg.y += degrees,
                Axis::Z => tool.params.rotate_deg.z += degrees,
            }
            true
        }
    }
}

/// Pushes the changed numbers through to the geometry, the way editing the boxes does.
fn apply(editor: &mut Editor) {
    match &mut editor.mode {
        Mode::Sketch(s) => {
            s.update_move();
            if s.take_dirty() {
                editor.commit_sketch();
            }
        }
        Mode::Model => super::tools::sync_tool(editor),
    }
    editor.request_repaint();
}
