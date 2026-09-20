//! The navigation cube: a small orientation gizmo that doubles as a view picker.
//!
//! The cube is drawn with the camera's own basis, so it always shows the model's
//! orientation from the same angle the viewport does. Clicking it picks a view: a face
//! gives one of the six square-on views, an edge a 45° view between two of them, and a
//! corner an isometric view. Dragging it orbits, which is what makes it feel attached to
//! the model rather than a row of buttons.
//!
//! Which part was clicked is decided by casting the pointer into the unit cube and looking
//! at where it lands, so the hit regions are exactly the shapes that were drawn, and the
//! same classification works at any orientation without a table of 26 screen positions.

use basset_math::Vec3;
use basset_viewport::Camera;

use super::Editor;

/// Side of the gizmo in logical pixels.
const SIZE: f32 = 104.0;

/// How far from the middle of a face a click has to be before it counts as reaching for
/// the edge or corner beyond it. Two thirds leaves the faces comfortably the biggest
/// targets, which is what users reach for most.
const EDGE_BAND: f64 = 0.66;

const FACE_FILL: egui::Color32 = egui::Color32::from_rgb(70, 78, 92);
const FACE_LIT: egui::Color32 = egui::Color32::from_rgb(92, 102, 120);
const OUTLINE: egui::Color32 = egui::Color32::from_rgb(150, 160, 180);
const HOTSPOT: egui::Color32 = egui::Color32::from_rgb(90, 160, 255);

/// Names of the six faces, indexed by axis and sign. "Front" is the face the camera looks
/// at in the Front view, which is the one facing −Y, matching the camera's presets.
fn face_name(axis: usize, positive: bool) -> &'static str {
    match (axis, positive) {
        (0, true) => "Right",
        (0, false) => "Left",
        (1, true) => "Back",
        (1, false) => "Front",
        (2, true) => "Top",
        _ => "Bottom",
    }
}

/// A readable name for any of the 26 directions, e.g. "Top-Front-Right".
fn hotspot_name(d: Vec3) -> String {
    [2usize, 1, 0]
        .into_iter()
        .filter(|&axis| d[axis] != 0.0)
        .map(|axis| face_name(axis, d[axis] > 0.0))
        .collect::<Vec<_>>()
        .join("-")
}

/// Where the cube face nearest the viewer would put a point of the unit cube, as an offset
/// in gizmo pixels from its centre, plus how near the viewer it is.
fn project(camera: &Camera, p: Vec3, scale: f32) -> (egui::Vec2, f64) {
    let eye = -camera.forward();
    let x = p.dot(camera.right());
    let y = p.dot(camera.up());
    (egui::vec2(x as f32 * scale, -y as f32 * scale), p.dot(eye))
}

/// The face, edge or corner a pointer offset from the cube's centre is aiming at, as a
/// direction with components in `{-1, 0, 1}`. `None` when the pointer misses the cube.
///
/// The pointer is a ray straight into the screen through the point of the cube's plane the
/// offset names; the first of the cube's six slabs it enters is the visible surface, and
/// how close that entry point is to the surface's own borders says whether the user was
/// reaching past the face for the edge or corner beyond.
fn hotspot_at(camera: &Camera, offset: egui::Vec2, scale: f32) -> Option<Vec3> {
    let eye = -camera.forward();
    let u = f64::from(offset.x / scale);
    let v = f64::from(-offset.y / scale);
    let origin = camera.right() * u + camera.up() * v + eye * 4.0;
    let dir = -eye;

    // Slab test against the unit cube, remembering which slab let the ray in.
    let (mut t_near, mut t_far) = (f64::NEG_INFINITY, f64::INFINITY);
    let mut entry_axis = 0usize;
    for axis in 0..3 {
        let (o, d) = (origin[axis], dir[axis]);
        if d.abs() < 1e-12 {
            if o.abs() > 1.0 {
                return None;
            }
            continue;
        }
        let (mut lo, mut hi) = ((-1.0 - o) / d, (1.0 - o) / d);
        if lo > hi {
            std::mem::swap(&mut lo, &mut hi);
        }
        if lo > t_near {
            t_near = lo;
            entry_axis = axis;
        }
        t_far = t_far.min(hi);
    }
    if t_near > t_far || t_far < 0.0 {
        return None;
    }
    let hit = origin + dir * t_near;
    let mut d = Vec3::ZERO;
    d[entry_axis] = hit[entry_axis].signum();
    for axis in 0..3 {
        if axis != entry_axis && hit[axis].abs() >= EDGE_BAND {
            d[axis] = hit[axis].signum();
        }
    }
    Some(d)
}

/// The cube's eight corners, and the four corners of each face with its outward normal.
fn faces() -> [(Vec3, [Vec3; 4]); 6] {
    let c = |x: f64, y: f64, z: f64| Vec3::new(x, y, z);
    [
        (
            Vec3::X,
            [
                c(1., -1., -1.),
                c(1., 1., -1.),
                c(1., 1., 1.),
                c(1., -1., 1.),
            ],
        ),
        (
            -Vec3::X,
            [
                c(-1., 1., -1.),
                c(-1., -1., -1.),
                c(-1., -1., 1.),
                c(-1., 1., 1.),
            ],
        ),
        (
            Vec3::Y,
            [
                c(1., 1., -1.),
                c(-1., 1., -1.),
                c(-1., 1., 1.),
                c(1., 1., 1.),
            ],
        ),
        (
            -Vec3::Y,
            [
                c(-1., -1., -1.),
                c(1., -1., -1.),
                c(1., -1., 1.),
                c(-1., -1., 1.),
            ],
        ),
        (
            Vec3::Z,
            [
                c(-1., -1., 1.),
                c(1., -1., 1.),
                c(1., 1., 1.),
                c(-1., 1., 1.),
            ],
        ),
        (
            -Vec3::Z,
            [
                c(-1., 1., -1.),
                c(1., 1., -1.),
                c(1., -1., -1.),
                c(-1., -1., -1.),
            ],
        ),
    ]
}

/// Draws the cube in the top-right of `free`, which the caller passes as the area the
/// docked panels leave over, so the cube never sits on top of the browser or the sketch
/// palette.
pub fn show(editor: &mut Editor, ctx: &egui::Context, free: egui::Rect) {
    let pos = egui::pos2(free.right() - SIZE - 12.0, free.top() + 12.0);
    let mut hovered: Option<Vec3> = None;
    let mut chosen: Option<Vec3> = None;
    let mut orbit = egui::Vec2::ZERO;
    let mut home = false;

    egui::Area::new(egui::Id::new("view-cube"))
        .fixed_pos(pos)
        .order(egui::Order::Foreground)
        .show(ctx, |ui| {
            let (rect, response) =
                ui.allocate_exact_size(egui::vec2(SIZE, SIZE), egui::Sense::click_and_drag());
            let painter = ui.painter_at(rect);
            let centre = rect.center();
            // Leave room for the corners, which stick out furthest under a diagonal view.
            let scale = SIZE * 0.29;
            let camera = &editor.camera;

            if response.dragged() {
                orbit = response.drag_delta();
            }
            let pointer = (!response.dragged())
                .then(|| response.hover_pos())
                .flatten();
            if let Some(p) = pointer {
                hovered = hotspot_at(camera, p - centre, scale);
            }
            if response.clicked() {
                chosen = hovered;
            }

            // Painter's algorithm: the faces pointing away are drawn first and covered.
            let mut visible: Vec<(Vec3, [Vec3; 4], f64)> = faces()
                .into_iter()
                .map(|(n, quad)| {
                    let depth = n.dot(-camera.forward());
                    (n, quad, depth)
                })
                .filter(|(_, _, depth)| *depth > 1e-6)
                .collect();
            visible.sort_by(|a, b| a.2.total_cmp(&b.2));
            for (normal, quad, depth) in &visible {
                let points: Vec<egui::Pos2> = quad
                    .iter()
                    .map(|p| centre + project(camera, *p, scale).0)
                    .collect();
                let lit = hovered.is_some_and(|h| h == *normal);
                let fill = if lit { FACE_LIT } else { FACE_FILL };
                painter.add(egui::Shape::convex_polygon(
                    points,
                    fill,
                    egui::Stroke::new(1.0, OUTLINE),
                ));
                // Label only the face squarely towards the viewer; the two oblique ones are
                // too foreshortened for text and only add noise.
                if *depth > 0.9 {
                    let axis = (0..3).find(|i| normal[*i] != 0.0).unwrap_or(0);
                    painter.text(
                        centre + project(camera, *normal, scale).0,
                        egui::Align2::CENTER_CENTER,
                        face_name(axis, normal[axis] > 0.0),
                        egui::FontId::proportional(12.0),
                        egui::Color32::from_rgb(225, 230, 240),
                    );
                }
            }
            // A dot on the hovered edge or corner: the faces light up on their own, but an
            // edge or corner has no area of its own to shade.
            if let Some(h) = hovered
                && h.abs().element_sum() > 1.5
            {
                let at = centre + project(camera, h.normalize() * 1.45, scale).0;
                painter.circle_filled(at, 4.0, HOTSPOT);
            }

            let label = hovered.map(hotspot_name).unwrap_or_default();
            ui.horizontal(|ui| {
                home = ui
                    .small_button("⌂")
                    .on_hover_text("Isometric view")
                    .clicked();
                ui.label(egui::RichText::new(label).small().weak());
            });
        });

    if orbit != egui::Vec2::ZERO {
        editor
            .camera
            .orbit(-f64::from(orbit.x) * 0.012, f64::from(orbit.y) * 0.012);
        editor.request_repaint();
    }
    if home {
        editor.look_from(basset_viewport::ViewPreset::Isometric);
    }
    if let Some(d) = chosen {
        editor.camera.look_from_direction(d);
        editor.set_status(format!("{} view", hotspot_name(d)));
        editor.request_repaint();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use basset_viewport::ViewPreset;

    fn camera(preset: ViewPreset) -> Camera {
        let mut c = Camera::new_default();
        c.look_from(preset);
        c
    }

    #[test]
    fn centre_of_the_cube_picks_the_face_towards_the_viewer() {
        for (preset, expected) in [
            (ViewPreset::Front, -Vec3::Y),
            (ViewPreset::Back, Vec3::Y),
            (ViewPreset::Right, Vec3::X),
            (ViewPreset::Left, -Vec3::X),
            (ViewPreset::Top, Vec3::Z),
            (ViewPreset::Bottom, -Vec3::Z),
        ] {
            let c = camera(preset);
            assert_eq!(
                hotspot_at(&c, egui::Vec2::ZERO, 30.0),
                Some(expected),
                "{preset:?}"
            );
        }
    }

    #[test]
    fn a_click_near_the_edge_of_a_face_picks_the_edge_or_corner() {
        // Looking along −Y, screen right is +X and screen up is +Z.
        let c = camera(ViewPreset::Front);
        let scale = 30.0;
        assert_eq!(
            hotspot_at(&c, egui::vec2(0.95 * scale, 0.0), scale),
            Some(Vec3::new(1.0, -1.0, 0.0)),
            "right-hand edge"
        );
        assert_eq!(
            hotspot_at(&c, egui::vec2(0.0, -0.95 * scale), scale),
            Some(Vec3::new(0.0, -1.0, 1.0)),
            "top edge"
        );
        assert_eq!(
            hotspot_at(&c, egui::vec2(0.95 * scale, -0.95 * scale), scale),
            Some(Vec3::new(1.0, -1.0, 1.0)),
            "top right corner"
        );
        // Just inside the band is still the face itself.
        assert_eq!(
            hotspot_at(&c, egui::vec2(0.5 * scale, 0.3 * scale), scale),
            Some(-Vec3::Y)
        );
    }

    #[test]
    fn clicks_off_the_cube_hit_nothing() {
        let c = camera(ViewPreset::Front);
        assert_eq!(hotspot_at(&c, egui::vec2(60.0, 60.0), 30.0), None);
    }

    #[test]
    fn every_hotspot_aims_the_camera_at_itself() {
        // Picking a hotspot and then asking what is under the cube's centre must give the
        // same hotspot back, or the cube would not agree with the view it just set.
        let mut camera = Camera::new_default();
        for x in [-1.0, 0.0, 1.0] {
            for y in [-1.0, 0.0, 1.0] {
                for z in [-1.0, 0.0, 1.0] {
                    let d = Vec3::new(x, y, z);
                    if d == Vec3::ZERO {
                        continue;
                    }
                    camera.look_from_direction(d);
                    assert_eq!(hotspot_at(&camera, egui::Vec2::ZERO, 30.0), Some(d), "{d}");
                }
            }
        }
    }

    #[test]
    fn hotspot_names_read_top_down() {
        assert_eq!(hotspot_name(-Vec3::Y), "Front");
        assert_eq!(hotspot_name(Vec3::new(1.0, -1.0, 1.0)), "Top-Front-Right");
        assert_eq!(hotspot_name(Vec3::new(-1.0, 0.0, -1.0)), "Bottom-Left");
    }
}
