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

/// Nearest edge within `tolerance` (world units) of the ray.
///
/// Two edges within tolerance are separated by aim first and by depth second, with the
/// aim read in bands half a tolerance wide: an edge the cursor is clearly nearer to wins,
/// and edges the cursor cannot be said to favour go to whichever is in front, so a hidden
/// edge never steals a click from the one drawn over it. Adding the two together, as this
/// used to, measures a millimetre of depth against a millimetre sideways, so on a body a
/// few hundred millimetres away the depths swamped the aim entirely.
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
            let band = |d: f64| (d / (tolerance * 0.5).max(f64::MIN_POSITIVE)).floor();
            let better = best.is_none_or(|b| (band(distance), t) < (band(b.distance), b.t));
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

/// How far two edges' tangents may disagree and still read as one smooth run. A
/// tessellated arc's end chord leaves the true tangent by half a facet, and
/// [`Tessellation`](crate::Tessellation) allows a facet 10°, so anything tighter would
/// break a chain at the very joins it exists for. It stays well inside the 45° at which
/// a fold stops being drawn as an edge at all.
const TANGENT_COS: f64 = 0.966; // 15°

/// Every edge that runs tangentially on from `seed`, `seed` first.
///
/// This is Fusion's tangent chain, and it is what makes rounding the rim of a slotted
/// plate one click rather than eight: the straight stretches and the arcs between them
/// are separate faces, so separate edges, but the user drew one outline and means all of
/// it. The walk stops where the run forks — more than one tangent continuation at a
/// vertex has no single answer, and blending an edge the user did not mean is worse than
/// making them pick it.
///
/// Smooth edges are skipped for the same reason [`pick_edge`] will not aim at one.
pub fn tangent_chain(edges: &[Edge], seed: EdgeKey) -> Vec<EdgeKey> {
    let ends = chain_ends(edges);
    let mut chain = vec![seed];
    let mut frontier = vec![seed];
    while let Some(key) = frontier.pop() {
        for end in ends.iter().filter(|e| e.key == key) {
            let mut tangent = ends.iter().filter(|o| {
                o.key != key
                    && o.point.distance(end.point) <= MERGE_TOL
                    // Both directions lead away from the shared vertex, so continuing
                    // each other means facing opposite ways.
                    && o.direction.dot(end.direction) <= -TANGENT_COS
            });
            let (Some(next), None) = (tangent.next(), tangent.next()) else {
                continue;
            };
            if !chain.contains(&next.key) {
                chain.push(next.key);
                frontier.push(next.key);
            }
        }
    }
    chain
}

/// One end of one polyline of one edge: where it stops and which way it leaves that
/// vertex.
struct ChainEnd {
    key: EdgeKey,
    point: Vec3,
    direction: Vec3,
}

/// The ends of every visible edge polyline. A closed polyline — a cylinder's rim — has
/// none, which is right: it is already the whole chain.
fn chain_ends(edges: &[Edge]) -> Vec<ChainEnd> {
    let mut out = Vec::new();
    for e in edges.iter().filter(|e| !e.smooth) {
        for chain in e.chains() {
            let (first, last) = (chain[0], *chain.last().expect("a chain has a segment"));
            if first.start.distance(last.end) <= MERGE_TOL {
                continue;
            }
            for (point, along) in [
                (first.start, first.end - first.start),
                (last.end, last.start - last.end),
            ] {
                if let Some(direction) = along.try_normalize() {
                    out.push(ChainEnd {
                        key: e.key,
                        point,
                        direction,
                    });
                }
            }
        }
    }
    out
}

/// Parameters `(t, u)` of the closest points between the ray (`origin + t·dir`) and the
/// segment `a + u·(b − a)`, `u` clamped to the segment.
///
/// Setting both partial derivatives of the squared distance to zero gives
/// `[[a11, -a12], [-a12, a22]] · [t, u] = [b1, b2]`, so Cramer's rule *adds* the
/// off-diagonal term in `u`. Subtracting it — as this once did — makes no difference
/// when the ray meets the segment at a right angle, which is every axis-aligned case a
/// test tends to look at; from any other angle, and the default view is isometric, the
/// closest point landed elsewhere along the edge and edge picking simply missed.
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
        (a11 * b2 + a12 * b1) / det
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

    /// A slot-shaped plate: straight sides joined by semicircular ends, so the rim of
    /// its top face is four edges that the user drew as one outline. Picking any of them
    /// offers all four; picking an edge of a cube, where every join is a corner, offers
    /// only itself.
    #[test]
    fn a_tangent_run_is_one_chain_and_a_corner_is_not() {
        use crate::geometry::{Contour, Extent, Profile, Segment, SegmentKind};
        use basset_math::{Frame, Vec2};

        let (half, r) = (10.0, 5.0);
        let arc = |centre: Vec2, curve: u32| Segment {
            curve,
            kind: SegmentKind::Arc {
                center: centre,
                radius: r,
                ccw: true,
            },
        };
        // Counter-clockwise from the bottom-left: straight, right cap, straight, left cap.
        let mut points = vec![Vec2::new(-half, -r)];
        let mut segments = vec![Segment::line(0)];
        let facets = 8;
        for (centre, base, curve, closes) in [
            (Vec2::new(half, 0.0), -90.0_f64, 1, Some(Segment::line(2))),
            (Vec2::new(-half, 0.0), 90.0_f64, 3, None),
        ] {
            points.push(centre + Vec2::from_angle(base.to_radians()) * r);
            for i in 1..facets {
                segments.push(arc(centre, curve));
                let a = (base + 180.0 * i as f64 / facets as f64).to_radians();
                points.push(centre + Vec2::from_angle(a) * r);
            }
            segments.push(arc(centre, curve));
            if let Some(next) = closes {
                points.push(centre + Vec2::from_angle((base + 180.0).to_radians()) * r);
                segments.push(next);
            }
        }
        let profile = Profile::new(
            Frame::XY,
            Contour {
                points,
                segments,
                closed: true,
            },
        );
        let solid = crate::generate::extrude(OpId::new(1), &profile, Extent::OneSide(4.0)).unwrap();
        let edges = solid.edges();
        let top = |curve: u32| {
            EdgeKey::new(
                FaceKey::new(OpId::new(1), FaceRole::EndCap),
                FaceKey::new(OpId::new(1), FaceRole::Side(curve)),
            )
        };
        assert!(
            edges.iter().any(|e| e.key == top(0) && !e.smooth),
            "the straight side's top edge is drawn"
        );
        let mut chain = tangent_chain(&edges, top(0));
        chain.sort();
        let mut expected = vec![top(0), top(1), top(2), top(3)];
        expected.sort();
        assert_eq!(chain, expected, "the whole rim runs on tangentially");
        // The vertical seams between the flats and the round ends are tangent, so they
        // are smooth and no chain ever runs down one onto the bottom rim.
        assert!(
            chain
                .iter()
                .all(|k| k.touches(FaceKey::new(OpId::new(1), FaceRole::EndCap))),
            "{chain:?}"
        );

        let c = cuboid(OpId::new(2), Vec3::ZERO, Vec3::splat(10.0));
        let key = EdgeKey::new(
            FaceKey::new(OpId::new(2), FaceRole::EndCap),
            FaceKey::new(OpId::new(2), FaceRole::Side(0)),
        );
        assert_eq!(
            tangent_chain(&c.edges(), key),
            vec![key],
            "a cube's edges meet at corners, not tangents"
        );
    }

    /// An oblique ray aimed straight at the middle of an edge finds it. Every other test
    /// here looks down an axis, where the ray meets the edge at a right angle and the
    /// closest-point solve is insensitive to its own cross term; the app's default view is
    /// isometric, so this is the case that actually matters.
    #[test]
    fn picks_an_edge_the_ray_meets_at_an_angle() {
        let c = cuboid(OpId::new(1), Vec3::ZERO, Vec3::new(10.0, 10.0, 2.0));
        let edges = c.edges();
        let target = Vec3::new(5.0, 0.0, 2.0);
        let direction = Vec3::new(-1.0, 1.5, -1.2).normalize();
        let ray = Ray::new(target - direction * 40.0, direction);
        let hit = pick_edge(&edges, &ray, 0.05).expect("the edge under the ray");
        assert!(hit.point.distance(target) < 1e-6, "{:?}", hit.point);
        assert_relative_eq!(hit.distance, 0.0, epsilon = 1e-9);
    }

    /// Two edges the same distance from the cursor but at different depths: the near one
    /// wins. Two at the same depth: the one the cursor is nearer to.
    #[test]
    fn picks_the_near_edge_and_then_the_closer_one() {
        let near = cuboid(OpId::new(1), Vec3::ZERO, Vec3::splat(10.0));
        let far = cuboid(
            OpId::new(2),
            Vec3::new(0.0, 0.0, -30.0),
            Vec3::new(10.0, 10.0, -20.0),
        );
        let mut edges = near.edges();
        edges.extend(far.edges());
        // Down the shared (x = 0, y = 5) line: the near block's side edge is at z = 10,
        // the far block's at z = −20.
        let ray = Ray::new(Vec3::new(0.1, 5.0, 40.0), -Vec3::Z);
        let hit = pick_edge(&edges, &ray, 0.5).unwrap();
        assert_relative_eq!(hit.t, 30.0, epsilon = 1e-6);

        // Straight down between two top edges of one block, nearer the y = 0 one.
        let c = cuboid(OpId::new(1), Vec3::new(0.0, 0.0, 0.0), Vec3::splat(1.0));
        let edges = c.edges();
        let ray = Ray::new(Vec3::new(0.5, 0.2, 5.0), -Vec3::Z);
        let hit = pick_edge(&edges, &ray, 0.5).unwrap();
        assert_relative_eq!(hit.point.y, 0.0, epsilon = 1e-9);
        let ray = Ray::new(Vec3::new(0.5, 0.8, 5.0), -Vec3::Z);
        let hit = pick_edge(&edges, &ray, 0.5).unwrap();
        assert_relative_eq!(hit.point.y, 1.0, epsilon = 1e-9);
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
