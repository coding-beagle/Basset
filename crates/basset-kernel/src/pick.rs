//! Selection helpers: which face, edge or corner is under a ray.
//!
//! Faces are picked through the tessellation because that is exactly what the user sees;
//! edges and corners are picked against the kernel's edge polylines with a screen-space
//! tolerance converted by the caller into world units at the hit depth.

use basset_math::{Ray, RayHit, Vec3};

use crate::ids::{EdgeKey, FaceKey};
use crate::solid::{Edge, MERGE_TOL, Tessellated};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FacePick {
    pub key: FaceKey,
    pub hit: RayHit,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EdgePick {
    pub key: EdgeKey,
    /// Closest point on the edge to the ray.
    pub point: Vec3,
    /// Distance along the ray to that point.
    pub t: f64,
    pub distance: f64,
}

pub fn pick_face(tess: &Tessellated, ray: &Ray) -> Option<FacePick> {
    let hit = tess.mesh.intersect_ray(ray)?;
    Some(FacePick {
        key: *tess.face_keys.get(hit.face_id as usize)?,
        hit,
    })
}

/// Nearest edge within `tolerance` (world units) of the ray. Among edges within
/// tolerance the closest to the ray origin wins, so foreground edges beat hidden ones.
pub fn pick_edge(edges: &[Edge], ray: &Ray, tolerance: f64) -> Option<EdgePick> {
    let mut best: Option<EdgePick> = None;
    for e in edges {
        for s in &e.segments {
            let (t, u) = closest_params(ray, s.start, s.end);
            if t < 0.0 {
                continue;
            }
            let on_edge = s.start.lerp(s.end, u);
            let distance = ray.at(t).distance(on_edge);
            if distance > tolerance {
                continue;
            }
            // Prefer nearer edges; break near-ties by closeness to the ray.
            let better = best.is_none_or(|b| t + distance < b.t + b.distance);
            if better {
                best = Some(EdgePick {
                    key: e.key,
                    point: on_edge,
                    t,
                    distance,
                });
            }
        }
    }
    best
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VertexPick {
    pub point: Vec3,
    /// Distance along the ray to the closest point on the ray to the corner.
    pub t: f64,
    pub distance: f64,
}

/// Every corner of a solid: the ends of its edge chains, deduplicated.
///
/// A closed edge — a cylinder's seam, a circular cap boundary — has no ends and so
/// contributes nothing, which is right: there is no corner there for the user to aim at.
pub fn corners(edges: &[Edge]) -> Vec<Vec3> {
    let mut out: Vec<Vec3> = Vec::new();
    for e in edges {
        for chain in e.chains() {
            let (first, last) = (chain[0].start, chain.last().unwrap().end);
            if first.distance(last) <= MERGE_TOL {
                continue;
            }
            for p in [first, last] {
                if !out.iter().any(|q| q.distance(p) <= MERGE_TOL) {
                    out.push(p);
                }
            }
        }
    }
    out
}

/// Nearest corner within `tolerance` (world units) of the ray.
pub fn pick_vertex(edges: &[Edge], ray: &Ray, tolerance: f64) -> Option<VertexPick> {
    let mut best: Option<VertexPick> = None;
    for point in corners(edges) {
        let t = (point - ray.origin).dot(ray.direction) / ray.direction.dot(ray.direction);
        if t < 0.0 {
            continue;
        }
        let distance = ray.at(t).distance(point);
        if distance > tolerance {
            continue;
        }
        if best.is_none_or(|b| t + distance < b.t + b.distance) {
            best = Some(VertexPick { point, t, distance });
        }
    }
    best
}

/// Parameters `(t, u)` of the closest points between the ray (`origin + t·dir`) and the
/// segment `a + u·(b − a)`, `u` clamped to the segment.
fn closest_params(ray: &Ray, a: Vec3, b: Vec3) -> (f64, f64) {
    let d1 = ray.direction;
    let d2 = b - a;
    let r = ray.origin - a;
    let (a11, a12, a22) = (d1.dot(d1), d1.dot(d2), d2.dot(d2));
    let (b1, b2) = (-d1.dot(r), d2.dot(r));
    let det = a11 * a22 - a12 * a12;
    let mut u = if det.abs() < 1e-18 {
        0.0
    } else {
        (a11 * b2 - a12 * b1) / det
    };
    u = u.clamp(0.0, 1.0);
    let t = (b1 + a12 * u) / a11;
    (t, u)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ids::{FaceRole, OpId};
    use crate::primitives::cuboid;
    use approx::assert_relative_eq;

    #[test]
    fn picks_the_top_face_and_a_top_edge() {
        let c = cuboid(OpId::new(1), Vec3::ZERO, Vec3::splat(10.0));
        let tess = c.tessellate();
        let ray = Ray::new(Vec3::new(5.0, 5.0, 20.0), -Vec3::Z);
        let pick = pick_face(&tess, &ray).unwrap();
        assert_eq!(pick.key.role, FaceRole::EndCap);
        assert_relative_eq!(pick.hit.t, 10.0);

        let edges = c.edges();
        let ray = Ray::new(Vec3::new(5.0, 0.2, 20.0), -Vec3::Z);
        let e = pick_edge(&edges, &ray, 0.5).unwrap();
        assert!(e.key.touches(FaceKey::new(OpId::new(1), FaceRole::EndCap)));
        assert!(e.key.touches(FaceKey::new(OpId::new(1), FaceRole::Side(0))));
        assert_relative_eq!(e.distance, 0.2, epsilon = 1e-9);
        assert!(pick_edge(&edges, &ray, 0.1).is_none());
    }

    #[test]
    fn picks_a_corner_but_not_the_middle_of_an_edge() {
        let c = cuboid(OpId::new(1), Vec3::ZERO, Vec3::splat(10.0));
        let edges = c.edges();
        // Down the Z axis just off the (10, 10) corner of the top face.
        let ray = Ray::new(Vec3::new(10.2, 10.0, 20.0), -Vec3::Z);
        let v = pick_vertex(&edges, &ray, 0.5).unwrap();
        assert!(v.point.distance(Vec3::new(10.0, 10.0, 10.0)) < 1e-9);
        assert_relative_eq!(v.distance, 0.2, epsilon = 1e-9);
        assert!(pick_vertex(&edges, &ray, 0.1).is_none());
        // Halfway along a top edge is on the edge but far from either of its corners.
        let ray = Ray::new(Vec3::new(5.0, 10.0, 20.0), -Vec3::Z);
        assert!(pick_edge(&edges, &ray, 0.5).is_some());
        assert!(pick_vertex(&edges, &ray, 0.5).is_none());
    }
}
