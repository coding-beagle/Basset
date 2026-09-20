//! Selection helpers: which face, edge or corner is under a ray.
//!
//! Faces are picked through the tessellation because that is exactly what the user sees;
//! edges and corners are picked against the kernel's edge polylines with a screen-space
//! tolerance converted by the caller into world units at the hit depth.

use basset_math::{ANGULAR_TOL, Ray, RayHit, Vec3};

use crate::ids::{EdgeKey, FaceKey};
use crate::solid::{Edge, MERGE_TOL, Tessellated, VertexIndex};

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
///
/// Smooth edges are not targets: nothing is drawn along them, so aiming at one would be
/// aiming at nothing, and a fillet across a surface that does not fold has no radius to
/// build.
pub fn pick_edge(edges: &[Edge], ray: &Ray, tolerance: f64) -> Option<EdgePick> {
    let mut best: Option<EdgePick> = None;
    for e in edges.iter().filter(|e| !e.smooth) {
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
///
/// A chain end where the edge simply carries on straight into the next face is not a
/// corner either. Healing plants a vertex wherever one feature's face happens to end
/// against another, and offering those as snap targets scatters points along edges that
/// the user drew as one.
pub fn corners(edges: &[Edge]) -> Vec<Vec3> {
    // One sweep builds the directions leaving every vertex, so deciding a candidate is a
    // lookup rather than another scan of the body. The scene asks for this every frame.
    let mut index = VertexIndex::default();
    let mut leaving: Vec<Vec<Vec3>> = Vec::new();
    let visible = || edges.iter().filter(|e| !e.smooth);
    for e in visible() {
        for s in &e.segments {
            let Some(d) = (s.end - s.start).try_normalize() else {
                continue;
            };
            for (p, d) in [(s.start, d), (s.end, -d)] {
                let id = index.id(p) as usize;
                if leaving.len() <= id {
                    leaving.resize(id + 1, Vec::new());
                }
                leaving[id].push(d);
            }
        }
    }
    let mut out: Vec<Vec3> = Vec::new();
    let mut taken = vec![false; leaving.len()];
    for e in visible() {
        for chain in e.chains() {
            let (first, last) = (chain[0].start, chain.last().unwrap().end);
            if first.distance(last) <= MERGE_TOL {
                continue;
            }
            for p in [first, last] {
                let id = index.id(p) as usize;
                if taken.get(id).copied().unwrap_or(false) || straight_through(&leaving, id) {
                    continue;
                }
                taken[id] = true;
                out.push(p);
            }
        }
    }
    out
}

/// Whether exactly two edge segments meet at this vertex and continue each other in a
/// straight line: the edge passes through, so there is no corner here.
fn straight_through(leaving: &[Vec<Vec3>], vertex: usize) -> bool {
    match leaving.get(vertex).map(Vec::as_slice) {
        Some([a, b]) => a.dot(*b) < -1.0 + ANGULAR_TOL,
        _ => false,
    }
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

    /// Where two faces of one plane meet there is nothing drawn, so there is nothing to
    /// aim at either: the click goes through to whatever is really there, and the seam's
    /// ends are not offered as snap targets.
    #[test]
    fn a_smooth_edge_is_not_a_target() {
        let mut c = cuboid(OpId::new(1), Vec3::ZERO, Vec3::splat(10.0));
        // Split the top face in two along y = 5, giving the halves separate keys.
        let top = FaceKey::new(OpId::new(1), FaceRole::EndCap);
        let i = c.faces.iter().position(|f| f.key == top).unwrap();
        let v = |x: f64, y: f64| Vec3::new(x, y, 10.0);
        let surface = c.faces[i].surface;
        c.faces[i].polygons = vec![
            crate::solid::Polygon::new(vec![v(0., 0.), v(10., 0.), v(10., 5.), v(0., 5.)]).unwrap(),
        ];
        c.faces.push(crate::solid::Face {
            key: FaceKey::new(OpId::new(1), FaceRole::Side(9)),
            surface,
            polygons: vec![
                crate::solid::Polygon::new(vec![v(0., 5.), v(10., 5.), v(10., 10.), v(0., 10.)])
                    .unwrap(),
            ],
        });
        c.heal();
        let edges = c.edges();
        assert!(
            edges.iter().any(|e| e.smooth),
            "the halves of the top meet smoothly"
        );

        // Straight down onto the seam: the top face is there, the seam is not.
        let ray = Ray::new(Vec3::new(5.0, 5.0, 20.0), -Vec3::Z);
        assert!(pick_edge(&edges, &ray, 0.5).is_none());
        // Nor are the seam's ends corners, though the cube's eight still are.
        let corners = corners(&edges);
        assert_eq!(corners.len(), 8, "{corners:?}");
    }
}
