//! Indexed triangle mesh used as the interchange type between the kernel, the renderer,
//! and file exporters.
//!
//! It deliberately carries no topology: the kernel owns topology, and everything
//! downstream only needs triangles plus a per-triangle `face_id` so that a picked
//! triangle can be traced back to the kernel face it came from.

use glam::{DAffine3, DVec3};
use serde::{Deserialize, Serialize};

use crate::frame::Ray;

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct TriMesh {
    pub positions: Vec<DVec3>,
    /// Per-vertex normals, same length as `positions`.
    pub normals: Vec<DVec3>,
    /// Three indices per triangle, counter-clockwise when viewed from outside.
    pub indices: Vec<u32>,
    /// One entry per triangle identifying the originating kernel face.
    pub face_ids: Vec<u32>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RayHit {
    /// Distance along the ray to the hit point.
    pub t: f64,
    pub point: DVec3,
    pub triangle: usize,
    pub face_id: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Aabb {
    pub min: DVec3,
    pub max: DVec3,
}

impl Aabb {
    pub fn empty() -> Self {
        Self {
            min: DVec3::splat(f64::INFINITY),
            max: DVec3::splat(f64::NEG_INFINITY),
        }
    }

    pub fn is_empty(&self) -> bool {
        self.min.x > self.max.x
    }

    pub fn include(&mut self, p: DVec3) {
        self.min = self.min.min(p);
        self.max = self.max.max(p);
    }

    pub fn union(&self, other: &Aabb) -> Aabb {
        Aabb {
            min: self.min.min(other.min),
            max: self.max.max(other.max),
        }
    }

    pub fn center(&self) -> DVec3 {
        (self.min + self.max) * 0.5
    }

    pub fn extent(&self) -> DVec3 {
        self.max - self.min
    }

    pub fn from_points(points: impl IntoIterator<Item = DVec3>) -> Self {
        let mut b = Self::empty();
        for p in points {
            b.include(p);
        }
        b
    }
}

impl TriMesh {
    pub fn triangle_count(&self) -> usize {
        self.indices.len() / 3
    }

    pub fn is_empty(&self) -> bool {
        self.indices.is_empty()
    }

    pub fn triangle(&self, i: usize) -> [DVec3; 3] {
        let a = self.indices[3 * i] as usize;
        let b = self.indices[3 * i + 1] as usize;
        let c = self.indices[3 * i + 2] as usize;
        [self.positions[a], self.positions[b], self.positions[c]]
    }

    /// Appends a flat-shaded triangle, duplicating vertices so every face keeps a crisp
    /// normal. Kernels that want smooth shading across a face build meshes themselves.
    pub fn push_triangle(&mut self, tri: [DVec3; 3], face_id: u32) {
        let n = (tri[1] - tri[0]).cross(tri[2] - tri[0]).normalize_or_zero();
        let base = self.positions.len() as u32;
        self.positions.extend_from_slice(&tri);
        self.normals.extend_from_slice(&[n, n, n]);
        self.indices.extend_from_slice(&[base, base + 1, base + 2]);
        self.face_ids.push(face_id);
    }

    pub fn append(&mut self, other: &TriMesh) {
        let base = self.positions.len() as u32;
        self.positions.extend_from_slice(&other.positions);
        self.normals.extend_from_slice(&other.normals);
        self.indices.extend(other.indices.iter().map(|i| i + base));
        self.face_ids.extend_from_slice(&other.face_ids);
    }

    pub fn aabb(&self) -> Aabb {
        Aabb::from_points(self.positions.iter().copied())
    }

    pub fn transform(&mut self, t: &DAffine3) {
        for p in &mut self.positions {
            *p = t.transform_point3(*p);
        }
        // Normals transform by the inverse transpose; for the rigid and uniform-scale
        // transforms CAD uses this is the same as the linear part, but we do it properly.
        let normal_mat = t.matrix3.inverse().transpose();
        for n in &mut self.normals {
            *n = (normal_mat * *n).normalize_or_zero();
        }
    }

    /// Closest intersection with the ray using Möller–Trumbore. Back faces count as hits
    /// so picking still works from inside a body during section views.
    pub fn intersect_ray(&self, ray: &Ray) -> Option<RayHit> {
        let mut best: Option<RayHit> = None;
        for i in 0..self.triangle_count() {
            let [a, b, c] = self.triangle(i);
            if let Some(t) = ray_triangle(ray, a, b, c)
                && best.is_none_or(|h| t < h.t)
            {
                best = Some(RayHit {
                    t,
                    point: ray.at(t),
                    triangle: i,
                    face_id: self.face_ids[i],
                });
            }
        }
        best
    }

    /// Signed volume via the divergence theorem. Negative means inside-out winding, which
    /// is a useful diagnostic in kernel tests.
    pub fn signed_volume(&self) -> f64 {
        (0..self.triangle_count())
            .map(|i| {
                let [a, b, c] = self.triangle(i);
                a.dot(b.cross(c)) / 6.0
            })
            .sum()
    }
}

fn ray_triangle(ray: &Ray, a: DVec3, b: DVec3, c: DVec3) -> Option<f64> {
    const EPS: f64 = 1e-12;
    let e1 = b - a;
    let e2 = c - a;
    let p = ray.direction.cross(e2);
    let det = e1.dot(p);
    if det.abs() < EPS {
        return None;
    }
    let inv = 1.0 / det;
    let s = ray.origin - a;
    let u = s.dot(p) * inv;
    if !(0.0..=1.0).contains(&u) {
        return None;
    }
    let q = s.cross(e1);
    let v = ray.direction.dot(q) * inv;
    if v < 0.0 || u + v > 1.0 {
        return None;
    }
    let t = e2.dot(q) * inv;
    (t > EPS).then_some(t)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn unit_cube() -> TriMesh {
        let mut m = TriMesh::default();
        let v = |x: f64, y: f64, z: f64| DVec3::new(x, y, z);
        let quads = [
            // -z, +z, -y, +y, -x, +x (outward CCW)
            [v(0., 0., 0.), v(0., 1., 0.), v(1., 1., 0.), v(1., 0., 0.)],
            [v(0., 0., 1.), v(1., 0., 1.), v(1., 1., 1.), v(0., 1., 1.)],
            [v(0., 0., 0.), v(1., 0., 0.), v(1., 0., 1.), v(0., 0., 1.)],
            [v(0., 1., 0.), v(0., 1., 1.), v(1., 1., 1.), v(1., 1., 0.)],
            [v(0., 0., 0.), v(0., 0., 1.), v(0., 1., 1.), v(0., 1., 0.)],
            [v(1., 0., 0.), v(1., 1., 0.), v(1., 1., 1.), v(1., 0., 1.)],
        ];
        for (id, q) in quads.iter().enumerate() {
            m.push_triangle([q[0], q[1], q[2]], id as u32);
            m.push_triangle([q[0], q[2], q[3]], id as u32);
        }
        m
    }

    #[test]
    fn cube_volume_and_bounds() {
        let m = unit_cube();
        assert_relative_eq!(m.signed_volume(), 1.0, epsilon = 1e-12);
        let b = m.aabb();
        assert_eq!(b.min, DVec3::ZERO);
        assert_eq!(b.max, DVec3::ONE);
    }

    #[test]
    fn ray_picks_nearest_face() {
        let m = unit_cube();
        let ray = Ray::new(DVec3::new(0.5, 0.5, 5.0), -DVec3::Z);
        let hit = m.intersect_ray(&ray).expect("hit");
        assert_relative_eq!(hit.t, 4.0, epsilon = 1e-12);
        assert_eq!(hit.face_id, 1, "+z face is quad index 1");
        let miss = Ray::new(DVec3::new(5.0, 5.0, 5.0), -DVec3::Z);
        assert!(m.intersect_ray(&miss).is_none());
    }

    #[test]
    fn transform_moves_positions_and_keeps_volume() {
        let mut m = unit_cube();
        m.transform(&DAffine3::from_translation(DVec3::new(10.0, 0.0, 0.0)));
        assert_relative_eq!(m.aabb().min.x, 10.0);
        assert_relative_eq!(m.signed_volume(), 1.0, epsilon = 1e-9);
        m.transform(&DAffine3::from_scale(DVec3::splat(2.0)));
        assert_relative_eq!(m.signed_volume(), 8.0, epsilon = 1e-9);
        for n in &m.normals {
            assert_relative_eq!(n.length(), 1.0, epsilon = 1e-12);
        }
    }
}
