//! Orbit camera with the CAD convention of +Z up.
//!
//! The camera is parameterised by a target point, a distance and two angles rather than by
//! an eye position and orientation. Orbit, pan and zoom then become one-line updates that
//! cannot drift the view off its target, and every named view is just a pair of angles.
//! All maths stays in `f64`; the renderer converts to `f32` at the last moment.
//!
//! A third angle, `roll`, turns the picture about the view axis. It is what lets a square-on
//! view be turned a quarter at a time (the Front view with Z pointing sideways), which no
//! pair of yaw and pitch can express for a side view. Orbiting drops it again: dragging is
//! defined against world +Z, and keeping a roll through it would make a horizontal drag spin
//! the model about an axis the user cannot see.

use std::f64::consts::{FRAC_PI_2, FRAC_PI_4, PI, TAU};

use basset_math::{Aabb, Mat3, Mat4, Ray, Vec3};

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Projection {
    Perspective {
        /// Vertical field of view in radians.
        fov_y: f64,
    },
    Orthographic {
        /// Half the visible height at the target plane, in world units (mm).
        half_height: f64,
    },
}

/// Standard named views. "Front" looks along +Y so the XZ plane faces the viewer with X to
/// the right and Z up, matching Fusion / SolidWorks defaults.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ViewPreset {
    Front,
    Back,
    Top,
    Bottom,
    Left,
    Right,
    Isometric,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Camera {
    /// Point the camera orbits around and looks at.
    pub target: Vec3,
    /// Distance from the eye to the target.
    pub distance: f64,
    /// Rotation of the eye about +Z, measured from +X towards +Y.
    pub yaw: f64,
    /// Elevation of the eye above the XY plane, clamped to `[-π/2, π/2]`.
    pub pitch: f64,
    /// Turn of the picture about the view axis, in `(-π, π]`; positive turns the model
    /// clockwise on screen. Kept at zero when looking straight down or up, where a roll is
    /// the same picture as a change of yaw and is stored as one, so "is the view rolled"
    /// has a single answer.
    pub roll: f64,
    pub projection: Projection,
}

/// Default vertical field of view, a little narrower than games use because CAD users
/// judge proportions by eye and wide perspectives exaggerate them.
pub const DEFAULT_FOV_Y: f64 = 40.0_f64.to_radians();

/// Near and far planes scale with the orbit distance so the depth range always brackets
/// the model the user is looking at; a fixed near plane would either clip close-ups or
/// waste depth precision on large assemblies. The 1e5 ratio is comfortable for a 32-bit
/// float depth buffer with perspective projection.
const NEAR_FACTOR: f64 = 0.002;
const FAR_FACTOR: f64 = 200.0;

/// Fraction of the view left empty around a fitted bounding box.
const FIT_MARGIN: f64 = 1.15;

impl Default for Camera {
    fn default() -> Self {
        Self::new_default()
    }
}

impl Camera {
    /// Isometric-ish view of the origin at a distance suited to a part a few hundred
    /// millimetres across.
    pub fn new_default() -> Self {
        let mut camera = Self {
            target: Vec3::ZERO,
            distance: 500.0,
            yaw: 0.0,
            pitch: 0.0,
            roll: 0.0,
            projection: Projection::Perspective {
                fov_y: DEFAULT_FOV_Y,
            },
        };
        camera.look_from(ViewPreset::Isometric);
        camera
    }

    /// Unit vector from the target towards the eye.
    fn eye_direction(&self) -> Vec3 {
        let (sp, cp) = self.pitch.sin_cos();
        let (sy, cy) = self.yaw.sin_cos();
        Vec3::new(cp * cy, cp * sy, sp)
    }

    pub fn eye(&self) -> Vec3 {
        self.target + self.eye_direction() * self.distance
    }

    /// Unit vector the camera looks along.
    pub fn forward(&self) -> Vec3 {
        -self.eye_direction()
    }

    /// Screen-right before the roll. Derived from yaw alone so it stays well defined when
    /// looking straight down or up, where a `look_at` with a fixed +Z up vector degenerates.
    fn unrolled_right(&self) -> Vec3 {
        let (sy, cy) = self.yaw.sin_cos();
        Vec3::new(-sy, cy, 0.0)
    }

    fn unrolled_up(&self) -> Vec3 {
        self.unrolled_right().cross(self.forward())
    }

    /// Screen-right in world space.
    pub fn right(&self) -> Vec3 {
        let (s, c) = self.roll.sin_cos();
        self.unrolled_right() * c + self.unrolled_up() * s
    }

    /// Screen-up in world space.
    pub fn up(&self) -> Vec3 {
        let (s, c) = self.roll.sin_cos();
        self.unrolled_up() * c - self.unrolled_right() * s
    }

    /// Whether the view has been turned about its axis away from world +Z being up.
    /// Looking straight down or up this is always false: the turn is held as yaw.
    pub fn is_rolled(&self) -> bool {
        self.roll.abs() > ANGLE_EPSILON
    }

    /// Whether the camera looks straight along one of the world axes: the views a quarter
    /// turn about the view axis is offered for, because their turns are views too.
    pub fn is_axis_aligned(&self) -> bool {
        is_axis(self.forward())
    }

    /// Turns the picture a quarter turn about the view axis, keeping the target in place.
    pub fn roll_quarter_turn(&mut self, clockwise: bool) {
        let step = if clockwise { FRAC_PI_2 } else { -FRAC_PI_2 };
        self.roll = tidy_angle(self.roll + step);
        self.fold_roll_into_yaw();
    }

    /// Looking straight down or up, a roll is a turn about world Z, which is what yaw is.
    /// Storing it there keeps `roll` zero for those views, so orbiting away from a turned
    /// Top view continues from the picture on screen rather than jumping back upright.
    fn fold_roll_into_yaw(&mut self) {
        if (self.pitch.abs() - FRAC_PI_2).abs() < ANGLE_EPSILON {
            self.yaw = tidy_angle(self.yaw + self.roll * self.pitch.signum()).rem_euclid(TAU);
            self.roll = 0.0;
        }
    }

    /// Rolls the camera so `up` (projected square to the view axis) points up the screen.
    /// Leaves the roll alone when `up` lies along the view axis and so says nothing.
    fn roll_to_up(&mut self, up: Vec3) {
        let wanted_right = self.forward().cross(up);
        if wanted_right.length() < ANGLE_EPSILON {
            return;
        }
        let (r0, u0) = (self.unrolled_right(), self.unrolled_up());
        self.roll = tidy_angle(wanted_right.dot(u0).atan2(wanted_right.dot(r0)));
        self.fold_roll_into_yaw();
    }

    pub fn is_orthographic(&self) -> bool {
        matches!(self.projection, Projection::Orthographic { .. })
    }

    /// Vertical field of view used for perspective sizing, and to convert between the two
    /// projections so switching keeps the framing at the target plane unchanged.
    fn fov_y(&self) -> f64 {
        match self.projection {
            Projection::Perspective { fov_y } => fov_y,
            Projection::Orthographic { .. } => DEFAULT_FOV_Y,
        }
    }

    /// Switches to orthographic projection, preserving the apparent size at the target.
    pub fn set_orthographic(&mut self) {
        if let Projection::Perspective { fov_y } = self.projection {
            self.projection = Projection::Orthographic {
                half_height: self.distance * (fov_y * 0.5).tan(),
            };
        }
    }

    /// Switches to perspective projection, preserving the apparent size at the target.
    pub fn set_perspective(&mut self) {
        if let Projection::Orthographic { half_height } = self.projection {
            let fov_y = DEFAULT_FOV_Y;
            self.distance = half_height / (fov_y * 0.5).tan();
            self.projection = Projection::Perspective { fov_y };
        }
    }

    pub fn orbit(&mut self, delta_yaw: f64, delta_pitch: f64) {
        // Orbiting is about world +Z, so a turned side view is stood upright first. Looking
        // straight down or up that costs nothing, because the turn is already held as yaw.
        self.fold_roll_into_yaw();
        self.roll = 0.0;
        self.yaw = (self.yaw + delta_yaw).rem_euclid(TAU);
        // Stopping exactly at the poles avoids the view flipping over the top; the basis
        // vectors are still well defined there because `right` depends only on yaw.
        self.pitch = (self.pitch + delta_pitch).clamp(-FRAC_PI_2, FRAC_PI_2);
    }

    /// Moves the target so that whatever sits at the target depth follows the cursor
    /// exactly. Pixel `y` grows downwards.
    pub fn pan(&mut self, dx_pixels: f64, dy_pixels: f64, viewport: [u32; 2]) {
        let world_per_pixel = self.pixel_size_at(self.target, viewport);
        self.target -= self.right() * (dx_pixels * world_per_pixel);
        self.target += self.up() * (dy_pixels * world_per_pixel);
    }

    /// Multiplicative zoom: factors below one move closer. Both projections track the same
    /// distance so toggling between them later does not jump.
    pub fn zoom(&mut self, factor: f64) {
        if !(factor.is_finite() && factor > 0.0) {
            return;
        }
        self.distance = (self.distance * factor).max(f64::MIN_POSITIVE);
        if let Projection::Orthographic { half_height } = &mut self.projection {
            *half_height *= factor;
        }
    }

    /// Frames the box's bounding sphere. Uses the vertical field of view, so on very tall
    /// narrow viewports the sides may be clipped slightly; the margin covers common aspects.
    pub fn zoom_to_fit(&mut self, aabb: &Aabb) {
        if aabb.is_empty() {
            return;
        }
        let radius = (aabb.extent().length() * 0.5).max(1e-3) * FIT_MARGIN;
        self.target = aabb.center();
        match &mut self.projection {
            Projection::Perspective { fov_y } => self.distance = radius / (*fov_y * 0.5).sin(),
            Projection::Orthographic { half_height } => {
                *half_height = radius;
                self.distance = radius / (DEFAULT_FOV_Y * 0.5).sin();
            }
        }
    }

    /// Points the camera along an arbitrary direction, given as the vector from the target
    /// towards the eye, with world +Z up the screen (and the presets' orientation when
    /// looking straight down or up). Sketch mode needs "square on to this plane", which is
    /// not a named preset.
    pub fn look_from_direction(&mut self, eye_direction: Vec3) {
        let d = eye_direction.normalize_or_zero();
        if d == Vec3::ZERO {
            return;
        }
        self.roll = 0.0;
        self.pitch = d.z.clamp(-1.0, 1.0).asin();
        // Looking straight down or up leaves yaw undetermined. Choosing the same value the
        // Top/Bottom presets use keeps the view cube and the View menu in agreement.
        self.yaw = if d.x.abs() + d.y.abs() < 1e-9 {
            -FRAC_PI_2
        } else {
            d.y.atan2(d.x)
        };
    }

    /// Turns to look from `eye_direction` the way the view cube does: as if the cube on
    /// screen had been rolled over to show that side, so the picture's orientation is
    /// carried along from the current one instead of reset. Clicking the face already being
    /// looked at therefore keeps its quarter turn, and going from the Right view to the Top
    /// view puts the Right face at the bottom of the screen, as in Fusion.
    ///
    /// Two exceptions keep the cube from leaving views tipped over that the user never
    /// asked for. Until the user has turned a side view themselves, side views stay upright
    /// (Top to Right gives Z up, not the Top view's Y), and so do the oblique edge and
    /// corner views. And a square-on view always lands with a world axis up the screen, so
    /// a cube click straightens a view orbited slightly off square rather than keeping its
    /// tilt.
    pub fn turn_to_direction(&mut self, eye_direction: Vec3) {
        let d = eye_direction.normalize_or_zero();
        if d == Vec3::ZERO {
            return;
        }
        let carried_up = carry_along(self.up(), self.forward(), -d);
        let turned_by_user = self.is_rolled();

        self.look_from_direction(d);
        let upright_up = self.up();
        let up = if is_axis(d) {
            if !turned_by_user && d.z.abs() < 0.5 {
                upright_up
            } else {
                nearest_axis_square_to(d, carried_up, upright_up)
            }
        } else if turned_by_user {
            carried_up
        } else {
            upright_up
        };
        self.roll_to_up(up);
    }

    pub fn look_from(&mut self, preset: ViewPreset) {
        let (yaw, pitch) = match preset {
            ViewPreset::Front => (-FRAC_PI_2, 0.0),
            ViewPreset::Back => (FRAC_PI_2, 0.0),
            ViewPreset::Right => (0.0, 0.0),
            ViewPreset::Left => (PI, 0.0),
            // With yaw = -π/2 the top view keeps X to the right and Y up, like a drawing.
            ViewPreset::Top => (-FRAC_PI_2, FRAC_PI_2),
            ViewPreset::Bottom => (-FRAC_PI_2, -FRAC_PI_2),
            ViewPreset::Isometric => (-FRAC_PI_4, (1.0 / 2.0_f64.sqrt()).atan()),
        };
        self.yaw = yaw;
        self.pitch = pitch;
        self.roll = 0.0;
    }

    fn near_far(&self) -> (f64, f64) {
        (self.distance * NEAR_FACTOR, self.distance * FAR_FACTOR)
    }

    /// World → view (right-handed, camera looks down -Z in view space).
    pub fn view_matrix(&self) -> Mat4 {
        let rotation = Mat3::from_cols(self.right(), self.up(), -self.forward()).transpose();
        let eye = self.eye();
        Mat4::from_mat3(rotation) * Mat4::from_translation(-eye)
    }

    /// View → clip with wgpu's `[0, 1]` depth range.
    pub fn projection_matrix(&self, aspect: f64) -> Mat4 {
        let aspect = if aspect.is_finite() && aspect > 0.0 {
            aspect
        } else {
            1.0
        };
        let (near, far) = self.near_far();
        match self.projection {
            Projection::Perspective { fov_y } => perspective_rh(fov_y, aspect, near, far),
            Projection::Orthographic { half_height } => {
                // In orthographic mode the eye distance is only notional, so allow geometry
                // "behind" the eye rather than clipping it when the user zooms far in.
                orthographic_rh(half_height * aspect, half_height, -far, far)
            }
        }
    }

    pub fn view_projection(&self, aspect: f64) -> Mat4 {
        self.projection_matrix(aspect) * self.view_matrix()
    }

    fn aspect_of(viewport: [u32; 2]) -> f64 {
        f64::from(viewport[0].max(1)) / f64::from(viewport[1].max(1))
    }

    fn ndc_from_pixel(pixel: [f64; 2], viewport: [u32; 2]) -> (f64, f64) {
        let w = f64::from(viewport[0].max(1));
        let h = f64::from(viewport[1].max(1));
        (2.0 * pixel[0] / w - 1.0, 1.0 - 2.0 * pixel[1] / h)
    }

    /// Ray through a pixel (y down), for picking. In orthographic mode rays are parallel.
    pub fn ray_from_screen(&self, pixel: [f64; 2], viewport: [u32; 2]) -> Ray {
        let (x, y) = Self::ndc_from_pixel(pixel, viewport);
        let inverse = self.view_projection(Self::aspect_of(viewport)).inverse();
        let near = inverse.project_point3(Vec3::new(x, y, 0.0));
        let far = inverse.project_point3(Vec3::new(x, y, 1.0));
        match self.projection {
            // Starting at the eye rather than the near plane means hit distances are
            // comparable across pixels and nothing between eye and near plane is skipped.
            Projection::Perspective { .. } => Ray::new(self.eye(), far - self.eye()),
            Projection::Orthographic { .. } => Ray::new(near, far - near),
        }
    }

    /// Pixel coordinates (y down) of a world point, or `None` if it is behind the camera.
    pub fn world_to_screen(&self, p: Vec3, viewport: [u32; 2]) -> Option<[f64; 2]> {
        let clip = self.view_projection(Self::aspect_of(viewport)) * p.extend(1.0);
        if clip.w <= 0.0 {
            return None;
        }
        let ndc = clip.truncate() / clip.w;
        let w = f64::from(viewport[0].max(1));
        let h = f64::from(viewport[1].max(1));
        Some([(ndc.x + 1.0) * 0.5 * w, (1.0 - ndc.y) * 0.5 * h])
    }

    /// World units covered by one pixel at the depth of `p`. Pick tolerances and pan speed
    /// are expressed in pixels, and this converts them to model space.
    pub fn pixel_size_at(&self, p: Vec3, viewport: [u32; 2]) -> f64 {
        let h = f64::from(viewport[1].max(1));
        match self.projection {
            Projection::Perspective { fov_y } => {
                let depth = (p - self.eye())
                    .dot(self.forward())
                    .max(self.distance * NEAR_FACTOR);
                2.0 * depth * (fov_y * 0.5).tan() / h
            }
            Projection::Orthographic { half_height } => 2.0 * half_height / h,
        }
    }

    /// Half height of the view at the target plane, in world units. Used by the grid to
    /// decide how much of the plane needs lines.
    pub fn visible_half_height_at_target(&self) -> f64 {
        match self.projection {
            Projection::Perspective { fov_y } => self.distance * (fov_y * 0.5).tan(),
            Projection::Orthographic { half_height } => half_height,
        }
    }

    /// Keeps `fov_y` accessible for callers that size things by angle.
    pub fn field_of_view(&self) -> f64 {
        self.fov_y()
    }
}

/// Below this two view angles are the same view: far above the rounding a few quarter
/// turns accumulate, far below anything a user could set by dragging.
const ANGLE_EPSILON: f64 = 1e-9;

/// An angle wrapped into `(-π, π]`, with quarter turns made exact so that turning four
/// times returns to precisely where the view started rather than to within rounding of it.
fn tidy_angle(a: f64) -> f64 {
    let quarters = a / FRAC_PI_2;
    let a = if (quarters - quarters.round()).abs() < ANGLE_EPSILON {
        quarters.round() * FRAC_PI_2
    } else {
        a
    };
    let wrapped = (a + PI).rem_euclid(TAU) - PI;
    if wrapped <= -PI + ANGLE_EPSILON {
        PI
    } else {
        wrapped
    }
}

/// Whether a unit vector lies along a world axis.
fn is_axis(v: Vec3) -> bool {
    v.abs().max_element() > 1.0 - ANGLE_EPSILON
}

/// `up` turned by the smallest rotation that takes `from` onto `to` (all unit vectors): what
/// happens to the top of the screen when a cube is rolled over to show another side.
/// Turning right round to the opposite side has no smallest rotation; it is taken about
/// `up` itself, so up stays up.
fn carry_along(up: Vec3, from: Vec3, to: Vec3) -> Vec3 {
    let axis = from.cross(to);
    let sin = axis.length();
    if sin < ANGLE_EPSILON {
        return up;
    }
    let cos = from.dot(to);
    let k = axis / sin;
    // Rodrigues' rotation formula.
    up * cos + k.cross(up) * sin + k * k.dot(up) * (1.0 - cos)
}

/// Of the world axes square to `view_axis`, the one nearest `up`. A tie (up exactly between
/// two axes, as when coming from the isometric view) goes to `preferred`, the orientation the
/// view would have with nothing carried, so the answer never hangs on rounding.
fn nearest_axis_square_to(view_axis: Vec3, up: Vec3, preferred: Vec3) -> Vec3 {
    let mut best = preferred;
    let mut best_score = up.dot(preferred);
    for axis in [Vec3::X, -Vec3::X, Vec3::Y, -Vec3::Y, Vec3::Z, -Vec3::Z] {
        let score = up.dot(axis);
        if axis.dot(view_axis).abs() < 0.5 && score > best_score + 1e-6 {
            best = axis;
            best_score = score;
        }
    }
    best
}

/// Right-handed perspective projection with wgpu's `[0, 1]` depth range (glam's own helper
/// is deprecated in favour of a module `basset_math` does not re-export).
fn perspective_rh(fov_y: f64, aspect: f64, near: f64, far: f64) -> Mat4 {
    let h = 1.0 / (0.5 * fov_y).tan();
    let w = h / aspect;
    let r = far / (near - far);
    #[rustfmt::skip]
    let cols = [
        w,   0.0, 0.0,      0.0,
        0.0, h,   0.0,      0.0,
        0.0, 0.0, r,       -1.0,
        0.0, 0.0, r * near, 0.0,
    ];
    Mat4::from_cols_array(&cols)
}

/// Right-handed orthographic projection centred on the view axis, `[0, 1]` depth range.
fn orthographic_rh(half_width: f64, half_height: f64, near: f64, far: f64) -> Mat4 {
    let r = 1.0 / (near - far);
    #[rustfmt::skip]
    let cols = [
        1.0 / half_width, 0.0,               0.0,      0.0,
        0.0,              1.0 / half_height, 0.0,      0.0,
        0.0,              0.0,               r,        0.0,
        0.0,              0.0,               r * near, 1.0,
    ];
    Mat4::from_cols_array(&cols)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    const VIEWPORT: [u32; 2] = [800, 600];

    fn assert_vec_eq(a: Vec3, b: Vec3) {
        assert_relative_eq!(a.x, b.x, epsilon = 1e-9);
        assert_relative_eq!(a.y, b.y, epsilon = 1e-9);
        assert_relative_eq!(a.z, b.z, epsilon = 1e-9);
    }

    #[test]
    fn basis_is_orthonormal_and_right_handed() {
        let mut camera = Camera::new_default();
        for (yaw, pitch) in [
            (0.0, 0.0),
            (1.0, 0.3),
            (-2.0, -1.2),
            (0.5, FRAC_PI_2),
            (0.5, -FRAC_PI_2),
        ] {
            camera.yaw = yaw;
            camera.pitch = pitch;
            let (r, u, f) = (camera.right(), camera.up(), camera.forward());
            assert_relative_eq!(r.length(), 1.0, epsilon = 1e-12);
            assert_relative_eq!(u.length(), 1.0, epsilon = 1e-12);
            assert_relative_eq!(r.dot(u), 0.0, epsilon = 1e-12);
            assert_relative_eq!(r.dot(f), 0.0, epsilon = 1e-12);
            assert_relative_eq!(u.dot(f), 0.0, epsilon = 1e-12);
            assert_relative_eq!(r.cross(u).dot(-f), 1.0, epsilon = 1e-12);
        }
    }

    #[test]
    fn centre_ray_passes_through_target() {
        for ortho in [false, true] {
            let mut camera = Camera::new_default();
            camera.target = Vec3::new(3.0, -4.0, 5.0);
            camera.orbit(0.7, -0.2);
            if ortho {
                camera.set_orthographic();
            }
            let ray = camera.ray_from_screen([400.0, 300.0], VIEWPORT);
            let to_target = camera.target - ray.origin;
            let t = to_target.dot(ray.direction);
            assert!(t > 0.0);
            assert_relative_eq!(ray.at(t).distance(camera.target), 0.0, epsilon = 1e-6);
            assert_vec_eq(ray.direction, camera.forward());
        }
    }

    #[test]
    fn world_to_screen_inverts_ray_from_screen() {
        for ortho in [false, true] {
            let mut camera = Camera::new_default();
            camera.orbit(0.3, 0.4);
            if ortho {
                camera.set_orthographic();
            }
            for pixel in [[100.0, 50.0], [400.0, 300.0], [799.0, 599.0], [0.0, 0.0]] {
                let ray = camera.ray_from_screen(pixel, VIEWPORT);
                let point = ray.at(camera.distance * 0.8);
                let back = camera
                    .world_to_screen(point, VIEWPORT)
                    .expect("in front of camera");
                assert_relative_eq!(back[0], pixel[0], epsilon = 1e-6);
                assert_relative_eq!(back[1], pixel[1], epsilon = 1e-6);
            }
        }
    }

    #[test]
    fn points_behind_camera_do_not_project() {
        let camera = Camera::new_default();
        let behind = camera.eye() - camera.forward() * 10.0;
        assert!(camera.world_to_screen(behind, VIEWPORT).is_none());
    }

    #[test]
    fn zoom_to_fit_places_box_inside_frustum() {
        let aabb = Aabb {
            min: Vec3::new(-10.0, -20.0, -5.0),
            max: Vec3::new(30.0, 10.0, 25.0),
        };
        for ortho in [false, true] {
            let mut camera = Camera::new_default();
            if ortho {
                camera.set_orthographic();
            }
            camera.zoom_to_fit(&aabb);
            assert_vec_eq(camera.target, aabb.center());
            for i in 0..8 {
                let corner = Vec3::new(
                    if i & 1 == 0 { aabb.min.x } else { aabb.max.x },
                    if i & 2 == 0 { aabb.min.y } else { aabb.max.y },
                    if i & 4 == 0 { aabb.min.z } else { aabb.max.z },
                );
                let clip = camera.view_projection(1.0) * corner.extend(1.0);
                assert!(clip.w > 0.0);
                let ndc = clip.truncate() / clip.w;
                assert!(
                    ndc.x.abs() <= 1.0 && ndc.y.abs() <= 1.0,
                    "corner {corner} at ndc {ndc}"
                );
                assert!(
                    (0.0..=1.0).contains(&ndc.z),
                    "corner {corner} depth {}",
                    ndc.z
                );
            }
        }
    }

    #[test]
    fn zoom_to_fit_ignores_empty_box() {
        let mut camera = Camera::new_default();
        let before = camera;
        camera.zoom_to_fit(&Aabb::empty());
        assert_eq!(camera, before);
    }

    #[test]
    fn orbit_clamps_pitch_and_wraps_yaw() {
        let mut camera = Camera::new_default();
        camera.orbit(0.0, 10.0);
        assert_relative_eq!(camera.pitch, FRAC_PI_2);
        camera.orbit(0.0, -20.0);
        assert_relative_eq!(camera.pitch, -FRAC_PI_2);
        camera.orbit(7.0 * PI, 0.0);
        assert!((0.0..2.0 * PI).contains(&camera.yaw));
    }

    #[test]
    fn presets_face_expected_directions() {
        let cases = [
            (ViewPreset::Front, Vec3::Y, Vec3::Z),
            (ViewPreset::Back, -Vec3::Y, Vec3::Z),
            (ViewPreset::Right, -Vec3::X, Vec3::Z),
            (ViewPreset::Left, Vec3::X, Vec3::Z),
            (ViewPreset::Top, -Vec3::Z, Vec3::Y),
            (ViewPreset::Bottom, Vec3::Z, -Vec3::Y),
        ];
        let mut camera = Camera::new_default();
        for (preset, forward, up) in cases {
            camera.look_from(preset);
            assert_vec_eq(camera.forward(), forward);
            assert_vec_eq(camera.up(), up);
        }
        camera.look_from(ViewPreset::Isometric);
        let f = camera.forward();
        assert!(f.x < 0.0 && f.y > 0.0 && f.z < 0.0);
        assert_relative_eq!(f.x.abs(), f.y.abs(), epsilon = 1e-12);
        assert_relative_eq!(f.x.abs(), f.z.abs(), epsilon = 1e-12);
    }

    #[test]
    fn look_from_direction_matches_the_named_presets() {
        let mut from_preset = Camera::new_default();
        let mut from_direction = Camera::new_default();
        for preset in [
            ViewPreset::Front,
            ViewPreset::Back,
            ViewPreset::Left,
            ViewPreset::Right,
            ViewPreset::Top,
            ViewPreset::Bottom,
            ViewPreset::Isometric,
        ] {
            from_preset.look_from(preset);
            let eye_direction = from_preset.eye() - from_preset.target;
            from_direction.look_from_direction(eye_direction);
            assert_vec_eq(from_direction.forward(), from_preset.forward());
            assert_vec_eq(from_direction.up(), from_preset.up());
        }
    }

    #[test]
    fn look_from_direction_ignores_a_degenerate_direction() {
        let mut camera = Camera::new_default();
        let before = camera;
        camera.look_from_direction(Vec3::ZERO);
        assert_eq!(camera, before);
    }

    #[test]
    fn pan_moves_target_with_cursor() {
        let mut camera = Camera::new_default();
        let before = camera.world_to_screen(camera.target, VIEWPORT).unwrap();
        let old_target = camera.target;
        camera.pan(40.0, -25.0, VIEWPORT);
        let after = camera.world_to_screen(old_target, VIEWPORT).unwrap();
        assert_relative_eq!(after[0] - before[0], 40.0, epsilon = 1e-6);
        assert_relative_eq!(after[1] - before[1], -25.0, epsilon = 1e-6);
    }

    #[test]
    fn zoom_scales_distance_and_ortho_height() {
        let mut camera = Camera::new_default();
        let d = camera.distance;
        camera.zoom(0.5);
        assert_relative_eq!(camera.distance, d * 0.5);
        camera.set_orthographic();
        let Projection::Orthographic { half_height } = camera.projection else {
            panic!()
        };
        camera.zoom(2.0);
        let Projection::Orthographic { half_height: after } = camera.projection else {
            panic!()
        };
        assert_relative_eq!(after, half_height * 2.0);
        camera.zoom(-1.0);
        assert_relative_eq!(camera.distance, d);
    }

    #[test]
    fn pixel_size_matches_projection() {
        let camera = Camera::new_default();
        let size = camera.pixel_size_at(camera.target, VIEWPORT);
        // Two points one pixel apart at the target depth project one pixel apart.
        let a = camera.world_to_screen(camera.target, VIEWPORT).unwrap();
        let b = camera
            .world_to_screen(camera.target + camera.right() * size, VIEWPORT)
            .unwrap();
        assert_relative_eq!(b[0] - a[0], 1.0, epsilon = 1e-6);
    }

    #[test]
    fn projection_toggle_round_trips() {
        let mut camera = Camera::new_default();
        let size = camera.pixel_size_at(camera.target, VIEWPORT);
        camera.set_orthographic();
        assert_relative_eq!(
            camera.pixel_size_at(camera.target, VIEWPORT),
            size,
            epsilon = 1e-9
        );
        camera.set_perspective();
        assert_relative_eq!(
            camera.pixel_size_at(camera.target, VIEWPORT),
            size,
            epsilon = 1e-9
        );
    }

    fn rolled(preset: ViewPreset, quarter_turns_clockwise: usize) -> Camera {
        let mut camera = Camera::new_default();
        camera.look_from(preset);
        for _ in 0..quarter_turns_clockwise {
            camera.roll_quarter_turn(true);
        }
        camera
    }

    #[test]
    fn a_clockwise_quarter_turn_moves_the_top_of_the_screen_to_the_right() {
        // Front view: X right, Z up. Turning the picture clockwise carries what was at the
        // top of the screen (Z) round to the right-hand side.
        let camera = rolled(ViewPreset::Front, 1);
        assert_vec_eq(camera.forward(), Vec3::Y);
        assert_vec_eq(camera.right(), Vec3::Z);
        assert_vec_eq(camera.up(), -Vec3::X);
        let mut back = camera;
        back.roll_quarter_turn(false);
        assert_vec_eq(back.right(), Vec3::X);
        assert_vec_eq(back.up(), Vec3::Z);
        assert!(!back.is_rolled());
    }

    #[test]
    fn four_quarter_turns_return_exactly_to_the_start() {
        for preset in [ViewPreset::Front, ViewPreset::Top, ViewPreset::Bottom] {
            let start = rolled(preset, 0);
            let mut camera = rolled(preset, 4);
            // Exact, not approximate: drift here would make "is this view rolled" flicker.
            assert_eq!(camera.roll, 0.0, "{preset:?}");
            camera.yaw = camera.yaw.rem_euclid(TAU);
            assert_relative_eq!(camera.yaw, start.yaw.rem_euclid(TAU), epsilon = 1e-12);
        }
    }

    #[test]
    fn rolled_basis_stays_orthonormal_and_projects_consistently() {
        let mut camera = Camera::new_default();
        camera.orbit(0.4, -0.3);
        camera.roll = 0.7;
        let (r, u, f) = (camera.right(), camera.up(), camera.forward());
        assert_relative_eq!(r.length(), 1.0, epsilon = 1e-12);
        assert_relative_eq!(u.length(), 1.0, epsilon = 1e-12);
        assert_relative_eq!(r.dot(u), 0.0, epsilon = 1e-12);
        assert_relative_eq!(r.dot(f), 0.0, epsilon = 1e-12);
        assert_relative_eq!(r.cross(u).dot(-f), 1.0, epsilon = 1e-12);
        // Screen-right in world space really is to the right on screen.
        let centre = camera.world_to_screen(camera.target, VIEWPORT).unwrap();
        let right = camera.world_to_screen(camera.target + r, VIEWPORT).unwrap();
        assert!(right[0] > centre[0]);
        assert_relative_eq!(right[1], centre[1], epsilon = 1e-6);
    }

    #[test]
    fn looking_straight_down_a_roll_is_held_as_yaw() {
        for preset in [ViewPreset::Top, ViewPreset::Bottom] {
            let camera = rolled(preset, 1);
            assert!(!camera.is_rolled(), "{preset:?}");
            let upright = rolled(preset, 0);
            // The picture still turned clockwise: old up is now right.
            assert_vec_eq(camera.right(), upright.up());
        }
    }

    #[test]
    fn pan_follows_the_cursor_in_a_rolled_view() {
        let mut camera = rolled(ViewPreset::Front, 1);
        let target = camera.target;
        let before = camera.world_to_screen(target, VIEWPORT).unwrap();
        camera.pan(30.0, 10.0, VIEWPORT);
        let after = camera.world_to_screen(target, VIEWPORT).unwrap();
        assert_relative_eq!(after[0] - before[0], 30.0, epsilon = 1e-6);
        assert_relative_eq!(after[1] - before[1], 10.0, epsilon = 1e-6);
    }

    #[test]
    fn orbit_stands_a_rolled_side_view_upright() {
        let mut camera = rolled(ViewPreset::Front, 1);
        camera.orbit(0.0, 0.0);
        assert!(!camera.is_rolled());
        assert_vec_eq(camera.forward(), Vec3::Y);
        assert_vec_eq(camera.up(), Vec3::Z);
    }

    #[test]
    fn orbit_from_a_turned_top_view_starts_from_the_picture_on_screen() {
        let mut camera = rolled(ViewPreset::Top, 1);
        let right = camera.right();
        camera.orbit(0.0, 0.0);
        assert_vec_eq(camera.right(), right);
    }

    #[test]
    fn zoom_and_projection_toggle_keep_the_roll() {
        let mut camera = rolled(ViewPreset::Right, 3);
        let up = camera.up();
        camera.zoom(0.5);
        camera.set_orthographic();
        camera.zoom_to_fit(&Aabb {
            min: Vec3::splat(-1.0),
            max: Vec3::splat(1.0),
        });
        camera.set_perspective();
        assert_vec_eq(camera.up(), up);
    }

    #[test]
    fn named_views_and_square_on_views_reset_the_roll() {
        let mut camera = rolled(ViewPreset::Front, 1);
        camera.look_from(ViewPreset::Front);
        assert_vec_eq(camera.up(), Vec3::Z);
        let mut camera = rolled(ViewPreset::Front, 1);
        camera.look_from_direction(-Vec3::Y);
        assert_vec_eq(camera.up(), Vec3::Z);
    }

    #[test]
    fn turning_to_the_face_already_shown_keeps_its_quarter_turn() {
        for preset in [ViewPreset::Front, ViewPreset::Top, ViewPreset::Left] {
            let mut camera = rolled(preset, 1);
            let (right, up) = (camera.right(), camera.up());
            camera.turn_to_direction(-camera.forward());
            assert_vec_eq(camera.right(), right);
            assert_vec_eq(camera.up(), up);
        }
    }

    #[test]
    fn turning_to_a_face_from_upright_matches_the_named_views_for_side_faces() {
        // Nobody has turned anything: Top to Right must stay the plain Right view.
        let mut camera = rolled(ViewPreset::Top, 0);
        camera.turn_to_direction(Vec3::X);
        assert_vec_eq(camera.forward(), -Vec3::X);
        assert_vec_eq(camera.up(), Vec3::Z);
        // Front to Top rolls the cube over its top edge, which is the Top preset.
        let mut camera = rolled(ViewPreset::Front, 0);
        camera.turn_to_direction(Vec3::Z);
        assert_vec_eq(camera.up(), Vec3::Y);
        // From isometric the carried up sits exactly between two axes; the preset wins.
        let mut camera = Camera::new_default();
        camera.turn_to_direction(Vec3::Z);
        assert_vec_eq(camera.up(), Vec3::Y);
    }

    #[test]
    fn turning_to_the_top_carries_the_side_the_view_came_from() {
        // As in Fusion: from the Right view, rolling the cube to its top leaves the Right
        // face at the bottom of the screen, so screen-up is −X.
        let mut camera = rolled(ViewPreset::Right, 0);
        camera.turn_to_direction(Vec3::Z);
        assert_vec_eq(camera.forward(), -Vec3::Z);
        assert_vec_eq(camera.up(), -Vec3::X);
    }

    #[test]
    fn turning_from_a_rolled_side_view_carries_the_roll_round() {
        // Front turned clockwise: Z points right. Rolling the cube to the Left face (on the
        // screen's top, since up is −X) tips the view over its top edge, so what was the
        // view direction (+Y) ends up at the top of the screen.
        let mut camera = rolled(ViewPreset::Front, 1);
        camera.turn_to_direction(-Vec3::X);
        assert_vec_eq(camera.forward(), Vec3::X);
        assert_vec_eq(camera.up(), Vec3::Y);
        assert!(camera.is_rolled());
        // The right-hand face (+Z) is reached by a turn about screen-up, which keeps it.
        let mut camera = rolled(ViewPreset::Front, 1);
        camera.turn_to_direction(Vec3::Z);
        assert_vec_eq(camera.forward(), -Vec3::Z);
        assert_vec_eq(camera.up(), -Vec3::X);
    }

    #[test]
    fn turning_to_an_oblique_view_stays_upright_unless_the_view_was_rolled() {
        let corner = Vec3::new(1.0, -1.0, 1.0);
        let mut camera = rolled(ViewPreset::Front, 0);
        camera.turn_to_direction(corner);
        let mut upright = Camera::new_default();
        upright.look_from_direction(corner);
        assert_vec_eq(camera.up(), upright.up());

        let mut camera = rolled(ViewPreset::Front, 1);
        camera.turn_to_direction(corner);
        assert_vec_eq(camera.forward(), -corner.normalize());
        assert!(camera.is_rolled());
    }

    #[test]
    fn turning_to_a_face_straightens_a_slightly_orbited_view() {
        let mut camera = rolled(ViewPreset::Top, 0);
        camera.orbit(0.2, 0.0);
        camera.turn_to_direction(Vec3::Z);
        assert!(camera.is_axis_aligned());
        assert_vec_eq(camera.up(), Vec3::Y);
    }

    #[test]
    fn turning_ignores_a_degenerate_direction() {
        let mut camera = rolled(ViewPreset::Front, 1);
        let before = camera;
        camera.turn_to_direction(Vec3::ZERO);
        assert_eq!(camera, before);
    }

    #[test]
    fn axis_alignment_is_about_the_view_direction_only() {
        assert!(rolled(ViewPreset::Front, 1).is_axis_aligned());
        assert!(rolled(ViewPreset::Bottom, 3).is_axis_aligned());
        assert!(!Camera::new_default().is_axis_aligned());
    }
}
