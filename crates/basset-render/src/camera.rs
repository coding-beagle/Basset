//! The camera a render is taken through.
//!
//! It is the viewport camera's projection written out as numbers — eye, the three axes,
//! and either a vertical field of view or the half-height of an orthographic view — so a
//! render of the view on screen lines up with it pixel for pixel, and the crate needs no
//! dependency on the viewport to take one. A thin lens adds depth of field.

use basset_math::{Aabb, Vec3};

use crate::bvh::TraceRay;
use crate::sampling::Rng;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Lens {
    /// Vertical field of view, radians.
    Perspective { fov_y: f64 },
    /// Half the visible height, millimetres.
    Orthographic { half_height: f64 },
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RenderCamera {
    pub eye: Vec3,
    /// Unit vectors: where the camera looks, and screen right and up.
    pub forward: Vec3,
    pub right: Vec3,
    pub up: Vec3,
    pub lens: Lens,
    /// Radius of the thin lens, millimetres; zero is a pinhole and everything is sharp.
    pub aperture: f64,
    /// Distance along `forward` that is in focus.
    pub focus_distance: f64,
}

impl RenderCamera {
    /// A camera at `eye` looking at `target` with +Z as near to up as the view allows.
    pub fn look_at(eye: Vec3, target: Vec3, lens: Lens) -> Self {
        let forward = (target - eye).normalize_or(-Vec3::Z);
        let mut right = forward.cross(Vec3::Z);
        if right.length_squared() < 1e-12 {
            // Straight down or up: any horizontal right will do; pick the one a top view
            // uses, X to the right.
            right = Vec3::X;
        }
        let right = right.normalize();
        let up = right.cross(forward);
        Self {
            eye,
            forward,
            right,
            up,
            lens,
            aperture: 0.0,
            focus_distance: (target - eye).length(),
        }
    }

    /// A perspective camera looking from the direction `from` (model towards eye) that
    /// fits `bounds` into an image of the given aspect, with a margin.
    ///
    /// The box's corners are fitted, not its bounding sphere: a long low part seen from
    /// the side fills a sphere badly, and the sphere fit leaves it a sliver in the middle
    /// of the picture.
    pub fn fit(bounds: &Aabb, from: Vec3, fov_y: f64, aspect: f64) -> Self {
        let center = bounds.center();
        let from = from.normalize_or(Vec3::new(1.0, -1.0, 1.0).normalize());
        let probe = Self::look_at(center + from, center, Lens::Perspective { fov_y });
        let tan_y = (fov_y * 0.5).tan() / 1.15;
        let tan_x = tan_y * aspect;
        let (lo, hi) = (bounds.min, bounds.max);
        let mut distance: f64 = 1e-3;
        for i in 0..8 {
            let corner = Vec3::new(
                if i & 1 == 0 { lo.x } else { hi.x },
                if i & 2 == 0 { lo.y } else { hi.y },
                if i & 4 == 0 { lo.z } else { hi.z },
            );
            let d = corner - center;
            // The corner sits `along` nearer the eye than the centre; it is inside the
            // frustum when its sideways offset is within tan × its depth.
            let along = d.dot(from);
            let x = d.dot(probe.right).abs();
            let y = d.dot(probe.up).abs();
            distance = distance.max(x / tan_x + along).max(y / tan_y + along);
        }
        // Not so close that the eye is inside the box.
        let radius = bounds.extent().length() * 0.5;
        let distance = distance.max(radius * 1.05);
        Self::look_at(
            center + from * distance,
            center,
            Lens::Perspective { fov_y },
        )
    }

    /// Depth of field focused at the camera's current focus distance, with an aperture
    /// that is a fraction of it: the same blur on a washer as on an engine block.
    pub fn with_depth_of_field(mut self, aperture_fraction: f64) -> Self {
        self.aperture = (aperture_fraction * self.focus_distance).max(0.0);
        self
    }

    /// The ray through the image point `(x, y)` in `[0, 1]²`, y down, as a pixel's
    /// coordinates divided by the image size.
    pub fn ray(&self, x: f64, y: f64, aspect: f64, rng: &mut Rng) -> TraceRay {
        let sx = 2.0 * x - 1.0;
        let sy = 1.0 - 2.0 * y;
        let (origin, dir) = match self.lens {
            Lens::Perspective { fov_y } => {
                let h = (fov_y * 0.5).tan();
                let d = (self.forward + self.right * (sx * h * aspect) + self.up * (sy * h))
                    .normalize();
                (self.eye, d)
            }
            Lens::Orthographic { half_height } => (
                self.eye + self.right * (sx * half_height * aspect) + self.up * (sy * half_height),
                self.forward,
            ),
        };
        if self.aperture <= 0.0 {
            return TraceRay::new(origin, dir);
        }
        // Thin lens: every ray through the pixel meets at the focal plane, from a point
        // spread over the aperture.
        let focus = origin + dir * (self.focus_distance / dir.dot(self.forward).max(1e-6));
        let r = rng.next_f64().sqrt() * self.aperture;
        let phi = 2.0 * std::f64::consts::PI * rng.next_f64();
        let lens = origin + self.right * (r * phi.cos()) + self.up * (r * phi.sin());
        TraceRay::new(lens, (focus - lens).normalize())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_centre_ray_runs_along_forward() {
        let c = RenderCamera::look_at(
            Vec3::new(100.0, -100.0, 100.0),
            Vec3::ZERO,
            Lens::Perspective { fov_y: 0.7 },
        );
        let mut rng = Rng::new(0, 0);
        let r = c.ray(0.5, 0.5, 1.5, &mut rng);
        assert!((r.dir - c.forward).length() < 1e-12);
        // Up is up on screen: the top edge looks higher.
        assert!(c.ray(0.5, 0.0, 1.5, &mut rng).dir.z > r.dir.z);
        assert!(c.up.z > 0.0);
    }

    #[test]
    fn a_fitted_camera_sees_every_corner() {
        let b = Aabb {
            min: Vec3::new(-10.0, -5.0, 0.0),
            max: Vec3::new(30.0, 5.0, 8.0),
        };
        let aspect = 16.0 / 9.0;
        let c = RenderCamera::fit(&b, Vec3::new(1.0, -1.0, 0.7), 0.7, aspect);
        let h = (0.35f64).tan();
        for corner in [b.min, b.max, Vec3::new(b.min.x, b.max.y, b.max.z)] {
            let d = corner - c.eye;
            let z = d.dot(c.forward);
            assert!(z > 0.0);
            assert!((d.dot(c.right) / z).abs() < h * aspect);
            assert!((d.dot(c.up) / z).abs() < h);
        }
    }

    #[test]
    fn a_thin_lens_keeps_the_focal_plane_sharp() {
        let mut c = RenderCamera::look_at(
            Vec3::new(0.0, -100.0, 0.0),
            Vec3::ZERO,
            Lens::Perspective { fov_y: 0.7 },
        );
        c.aperture = 5.0;
        let mut rng = Rng::new(1, 1);
        for _ in 0..50 {
            let r = c.ray(0.3, 0.6, 1.0, &mut rng);
            // Every ray through the same pixel crosses the plane y = 0 at one point.
            let t = -r.origin.y / r.dir.y;
            let p = r.origin + r.dir * t;
            let pinhole = {
                let mut p = c;
                p.aperture = 0.0;
                let r = p.ray(0.3, 0.6, 1.0, &mut rng);
                r.origin + r.dir * (-r.origin.y / r.dir.y)
            };
            assert!((p - pinhole).length() < 1e-9);
        }
    }
}
