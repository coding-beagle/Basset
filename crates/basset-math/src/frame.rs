//! Planes, rays, and orthonormal frames.
//!
//! A [`Frame`] is how a 2D sketch lives in 3D: the sketch stores plain 2D coordinates and
//! the frame maps them into world space. Keeping sketches 2D makes the constraint solver
//! oblivious to plane orientation, which is what allows re-hosting a sketch on a different
//! plane without touching its geometry.

use glam::{DAffine3, DMat3, DVec2, DVec3};
use serde::{Deserialize, Serialize};

use crate::tolerance::ANGULAR_TOL;

/// Infinite oriented plane in Hessian normal form.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Plane {
    pub origin: DVec3,
    pub normal: DVec3,
}

impl Plane {
    pub fn new(origin: DVec3, normal: DVec3) -> Self {
        Self {
            origin,
            normal: normal.normalize(),
        }
    }

    /// Signed distance; positive on the side the normal points to.
    pub fn signed_distance(&self, p: DVec3) -> f64 {
        self.normal.dot(p - self.origin)
    }

    pub fn project(&self, p: DVec3) -> DVec3 {
        p - self.normal * self.signed_distance(p)
    }

    /// Parameter `t` along the ray at which it pierces the plane, if it does and if the
    /// intersection lies ahead of the ray origin.
    pub fn intersect_ray(&self, ray: &Ray) -> Option<f64> {
        let denom = self.normal.dot(ray.direction);
        if denom.abs() < ANGULAR_TOL {
            return None;
        }
        let t = self.normal.dot(self.origin - ray.origin) / denom;
        (t >= 0.0).then_some(t)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Ray {
    pub origin: DVec3,
    pub direction: DVec3,
}

impl Ray {
    pub fn new(origin: DVec3, direction: DVec3) -> Self {
        Self {
            origin,
            direction: direction.normalize(),
        }
    }

    pub fn at(&self, t: f64) -> DVec3 {
        self.origin + self.direction * t
    }
}

/// Right-handed orthonormal frame: `x × y = z`.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Frame {
    pub origin: DVec3,
    pub x: DVec3,
    pub y: DVec3,
    pub z: DVec3,
}

impl Frame {
    pub const XY: Frame = Frame {
        origin: DVec3::ZERO,
        x: DVec3::X,
        y: DVec3::Y,
        z: DVec3::Z,
    };
    pub const YZ: Frame = Frame {
        origin: DVec3::ZERO,
        x: DVec3::Y,
        y: DVec3::Z,
        z: DVec3::X,
    };
    pub const XZ: Frame = Frame {
        origin: DVec3::ZERO,
        x: DVec3::Z,
        y: DVec3::X,
        z: DVec3::Y,
    };

    /// Builds a frame from an origin and normal, deriving in-plane axes deterministically.
    ///
    /// Determinism matters: sketches are stored in frame coordinates, so the same plane
    /// must always yield the same axes or reopening a document would rotate every sketch.
    pub fn from_normal(origin: DVec3, normal: DVec3) -> Self {
        let z = normal.normalize();
        // Prefer world X as the in-plane x axis so sketches on the XY plane read naturally;
        // fall back to world Y when the normal is (anti)parallel to X.
        let hint = if z.x.abs() < 0.9 { DVec3::X } else { DVec3::Y };
        let x = (hint - z * hint.dot(z)).normalize();
        let y = z.cross(x);
        Self { origin, x, y, z }
    }

    /// Builds a frame from an origin and explicit x/y directions, re-orthogonalising `y`
    /// against `x` so callers can pass approximate vectors.
    pub fn from_axes(origin: DVec3, x: DVec3, y: DVec3) -> Self {
        let x = x.normalize();
        let y = (y - x * y.dot(x)).normalize();
        let z = x.cross(y);
        Self { origin, x, y, z }
    }

    pub fn plane(&self) -> Plane {
        Plane {
            origin: self.origin,
            normal: self.z,
        }
    }

    pub fn to_world(&self, p: DVec2) -> DVec3 {
        self.origin + self.x * p.x + self.y * p.y
    }

    pub fn to_world_dir(&self, d: DVec2) -> DVec3 {
        self.x * d.x + self.y * d.y
    }

    /// Projects a world point onto the frame's plane and returns its 2D coordinates.
    pub fn to_local(&self, p: DVec3) -> DVec2 {
        let d = p - self.origin;
        DVec2::new(d.dot(self.x), d.dot(self.y))
    }

    pub fn to_affine(&self) -> DAffine3 {
        DAffine3::from_mat3_translation(DMat3::from_cols(self.x, self.y, self.z), self.origin)
    }

    pub fn transformed(&self, t: &DAffine3) -> Self {
        Self {
            origin: t.transform_point3(self.origin),
            x: t.transform_vector3(self.x).normalize(),
            y: t.transform_vector3(self.y).normalize(),
            z: t.transform_vector3(self.z).normalize(),
        }
    }

    pub fn offset(&self, distance: f64) -> Self {
        Self {
            origin: self.origin + self.z * distance,
            ..*self
        }
    }

    /// Rotates the frame about an axis passing through `axis_origin`.
    pub fn rotated_about(&self, axis_origin: DVec3, axis_dir: DVec3, angle: f64) -> Self {
        let rot = DAffine3::from_translation(axis_origin)
            * DAffine3::from_axis_angle(axis_dir.normalize(), angle)
            * DAffine3::from_translation(-axis_origin);
        self.transformed(&rot)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn frame_from_normal_is_right_handed_and_orthonormal() {
        for n in [
            DVec3::X,
            DVec3::Y,
            DVec3::Z,
            DVec3::new(1.0, 2.0, 3.0),
            -DVec3::X,
        ] {
            let f = Frame::from_normal(DVec3::ZERO, n);
            assert_relative_eq!(f.x.dot(f.y), 0.0, epsilon = 1e-12);
            assert_relative_eq!(f.x.cross(f.y).dot(f.z), 1.0, epsilon = 1e-12);
            assert_relative_eq!(f.z.dot(n.normalize()), 1.0, epsilon = 1e-12);
        }
    }

    #[test]
    fn local_world_round_trip() {
        let f = Frame::from_normal(DVec3::new(1.0, 2.0, 3.0), DVec3::new(0.3, -0.2, 0.9));
        let p = DVec2::new(4.5, -7.25);
        let back = f.to_local(f.to_world(p));
        assert_relative_eq!(back.x, p.x, epsilon = 1e-12);
        assert_relative_eq!(back.y, p.y, epsilon = 1e-12);
    }

    #[test]
    fn xy_frame_matches_world_axes() {
        let f = Frame::from_normal(DVec3::ZERO, DVec3::Z);
        assert_eq!(f, Frame::XY);
    }

    #[test]
    fn plane_ray_intersection() {
        let plane = Plane::new(DVec3::new(0.0, 0.0, 5.0), DVec3::Z);
        let ray = Ray::new(DVec3::ZERO, DVec3::new(0.0, 0.0, 1.0));
        assert_relative_eq!(plane.intersect_ray(&ray).unwrap(), 5.0);
        let away = Ray::new(DVec3::ZERO, -DVec3::Z);
        assert!(plane.intersect_ray(&away).is_none());
        let parallel = Ray::new(DVec3::ZERO, DVec3::X);
        assert!(plane.intersect_ray(&parallel).is_none());
    }

    #[test]
    fn rotated_frame_about_x_axis() {
        let f = Frame::XY.rotated_about(DVec3::ZERO, DVec3::X, std::f64::consts::FRAC_PI_2);
        assert_relative_eq!(f.z.dot(-DVec3::Y), 1.0, epsilon = 1e-12);
        assert_relative_eq!(f.x.dot(DVec3::X), 1.0, epsilon = 1e-12);
    }
}
