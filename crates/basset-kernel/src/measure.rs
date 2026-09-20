//! Pure geometric queries: lengths, radii, areas, distances and angles.
//!
//! These exist so that the Measure tool in the UI asks the kernel what something *is*
//! rather than reaching into polygons itself. Nothing here mutates anything, and nothing
//! here knows about selection, formatting or units — the answers are in millimetres and
//! radians like the rest of the crate.
//!
//! Where a query cannot be answered honestly it returns `None` rather than an
//! approximation: a measurement the user cannot trust is worse than one the tool declines
//! to make, and "these faces are not parallel" is itself something the tool wants to say.

use std::collections::HashMap;

use basset_math::{Plane, Vec3};

use crate::solid::{Edge, Face, MERGE_TOL, SurfaceKind, VertexIndex};

/// How far two unit directions may disagree and still count as parallel, measured as the
/// length of their cross product (the sine of the angle between them).
///
/// The normals and directions compared here come from analytic surfaces and straight
/// edges, so the only error to allow for is the arithmetic that produced them. A
/// hundredth of a degree is generous for that while still never calling a 0.1° draft
/// parallel — which would silently report a distance where the honest answer is an angle.
const PARALLEL_SIN: f64 = 1.75e-4;

/// The circle a circular edge lies on.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EdgeCircle {
    pub center: Vec3,
    /// Unit normal of the circle's plane.
    pub axis: Vec3,
    pub radius: f64,
}

/// The direction of a straight edge, or `None` if it bends.
///
/// Every query that treats an edge as a line goes through this, so a curved edge is never
/// silently measured as though it were its end-to-end chord.
pub fn edge_direction(edge: &Edge) -> Option<Vec3> {
    let mut direction: Option<Vec3> = None;
    for s in &edge.segments {
        let d = (s.end - s.start).try_normalize()?;
        match direction {
            None => direction = Some(d),
            // Opposite windings describe the same line, so only the axis has to agree.
            Some(first) if first.cross(d).length() <= PARALLEL_SIN => {}
            Some(_) => return None,
        }
    }
    direction
}

/// The circle a circular edge lies on: a hole's rim, a fillet's boundary, a revolve's
/// seam. `None` for a straight edge, or for any curve that is not an arc.
///
/// A tessellated arc puts its vertices *on* the true circle, so the circle through three
/// of them is the exact one the model was built from — no fitting is involved, and the
/// remaining vertices are checked against it rather than averaged into it. That check is
/// what keeps a swept or lofted curve from being reported as a radius it does not have.
pub fn edge_circle(edge: &Edge) -> Option<EdgeCircle> {
    let chain = edge.chains().into_iter().max_by_key(Vec::len)?;
    if chain.len() < 2 {
        return None;
    }
    let mut points: Vec<Vec3> = chain.iter().map(|s| s.start).collect();
    points.push(chain.last()?.end);
    // A closed rim repeats its start; dropping the repeat keeps the three samples spread
    // around the circle rather than bunching two of them together.
    if points.len() > 3 && points[0].distance(*points.last()?) <= MERGE_TOL {
        points.pop();
    }
    let (a, b, c) = (points[0], points[points.len() / 2], *points.last()?);
    let (ab, ac) = (b - a, c - a);
    let n = ab.cross(ac);
    let n2 = n.length_squared();
    if n2 <= MERGE_TOL * MERGE_TOL {
        return None;
    }
    let center =
        a + (ac.length_squared() * n.cross(ab) + ab.length_squared() * ac.cross(n)) / (2.0 * n2);
    let radius = center.distance(a);
    // Relative to the radius: a 200 mm rim's chord endpoints are computed in the same
    // relative terms, so an absolute slack would reject a perfectly good circle.
    let tol = MERGE_TOL + radius * 1e-6;
    if points
        .iter()
        .any(|p| (center.distance(*p) - radius).abs() > tol)
    {
        return None;
    }
    Some(EdgeCircle {
        center,
        axis: n / n2.sqrt(),
        radius,
    })
}

/// The plane of a planar face, anchored at its area-weighted centroid. `None` for a
/// curved face.
pub fn face_plane(face: &Face) -> Option<Plane> {
    match face.surface {
        SurfaceKind::Planar { normal } => Some(Plane {
            origin: face.centroid(),
            normal,
        }),
        _ => None,
    }
}

/// The radius of a cylindrical face, which is worth reporting beside its area: a hole is
/// named by its diameter far more often than by its wall area.
pub fn face_radius(face: &Face) -> Option<f64> {
    match face.surface {
        SurfaceKind::Cylindrical { radius, .. } => Some(radius),
        _ => None,
    }
}

/// The length of a face's boundary.
///
/// A face is a bag of polygons that a boolean or a healed T-junction may have split
/// arbitrarily, so the boundary is not "the first polygon's outline": it is every polygon
/// edge used by exactly one polygon of this face. Interior seams are used twice and drop
/// out. Holes count, as they do on a drawing.
pub fn face_perimeter(face: &Face) -> f64 {
    // Vertices go through the same index the rest of the kernel welds with, so the two
    // polygons either side of a seam agree that it is one edge and it cancels.
    let mut index = VertexIndex::default();
    let mut uses: HashMap<(u32, u32), (usize, f64)> = HashMap::new();
    for poly in &face.polygons {
        let n = poly.vertices.len();
        for i in 0..n {
            let (a, b) = (poly.vertices[i], poly.vertices[(i + 1) % n]);
            let (ia, ib) = (index.id(a), index.id(b));
            if ia == ib {
                continue;
            }
            let entry = uses
                .entry((ia.min(ib), ia.max(ib)))
                .or_insert((0, a.distance(b)));
            entry.0 += 1;
        }
    }
    uses.values().filter(|(n, _)| *n == 1).map(|(_, l)| l).sum()
}

/// The unsigned angle between two directions, in `0..=π/2`.
///
/// Unsigned because neither a face normal's outward sense nor an edge's winding is
/// something the user chose: reporting 135° for one pick order and 45° for the other
/// would make the same measurement read differently depending on which face was clicked
/// first. This is the angle between the two *lines*, which is what a drawing dimensions.
pub fn direction_angle(a: Vec3, b: Vec3) -> Option<f64> {
    let (a, b) = (a.try_normalize()?, b.try_normalize()?);
    Some(a.dot(b).abs().clamp(0.0, 1.0).acos())
}

/// Whether two directions describe the same line, either way round.
pub fn parallel(a: Vec3, b: Vec3) -> bool {
    match (a.try_normalize(), b.try_normalize()) {
        (Some(a), Some(b)) => a.cross(b).length() <= PARALLEL_SIN,
        _ => false,
    }
}

/// The perpendicular distance between two parallel planar faces, or `None` when they are
/// not parallel (or not planar) and the honest answer is an angle instead.
pub fn parallel_face_distance(a: &Face, b: &Face) -> Option<f64> {
    let (pa, pb) = (face_plane(a)?, face_plane(b)?);
    parallel(pa.normal, pb.normal).then(|| pa.signed_distance(pb.origin).abs())
}

/// The angle between two planar faces, as the angle between their planes.
pub fn face_angle(a: &Face, b: &Face) -> Option<f64> {
    direction_angle(face_plane(a)?.normal, face_plane(b)?.normal)
}

/// The perpendicular distance from a straight edge to a planar face it runs parallel to.
/// `None` when the edge is not straight, the face is not planar, or the edge leans.
pub fn edge_face_distance(edge: &Edge, face: &Face) -> Option<f64> {
    let plane = face_plane(face)?;
    let direction = edge_direction(edge)?;
    if direction.dot(plane.normal).abs() > PARALLEL_SIN {
        return None;
    }
    Some(plane.signed_distance(edge.segments.first()?.start).abs())
}

/// The angle between a straight edge and a planar face, in `0..=π/2`: zero when the edge
/// lies along the face, π/2 when it stands on it.
pub fn edge_face_angle(edge: &Edge, face: &Face) -> Option<f64> {
    let plane = face_plane(face)?;
    let direction = edge_direction(edge)?;
    Some(direction.dot(plane.normal).abs().clamp(0.0, 1.0).asin())
}

/// The angle between two straight edges, in `0..=π/2`.
pub fn edge_angle(a: &Edge, b: &Edge) -> Option<f64> {
    direction_angle(edge_direction(a)?, edge_direction(b)?)
}

/// The smallest distance between two edges. Both are polylines, so this is exactly the
/// minimum over their segment pairs — no iteration and no tolerance involved.
pub fn edge_distance(a: &Edge, b: &Edge) -> f64 {
    let mut best = f64::INFINITY;
    for p in &a.segments {
        for q in &b.segments {
            best = best.min(segment_distance(p.start, p.end, q.start, q.end));
        }
    }
    best
}

/// The distance from a point to the nearest point of an edge.
pub fn point_edge_distance(p: Vec3, edge: &Edge) -> f64 {
    edge.segments
        .iter()
        .map(|s| p.distance(closest_on_segment(p, s.start, s.end)))
        .fold(f64::INFINITY, f64::min)
}

/// The perpendicular distance from a point to the plane of a planar face.
///
/// The plane, not the face's extent: this is what a drawing means by "the corner sits
/// 12 mm above the base", and the nearest point of a bounded face would answer a
/// different and much less useful question.
pub fn point_face_distance(p: Vec3, face: &Face) -> Option<f64> {
    Some(face_plane(face)?.signed_distance(p).abs())
}

fn closest_on_segment(p: Vec3, a: Vec3, b: Vec3) -> Vec3 {
    let ab = b - a;
    let len2 = ab.length_squared();
    if len2 <= f64::MIN_POSITIVE {
        return a;
    }
    a + ab * ((p - a).dot(ab) / len2).clamp(0.0, 1.0)
}

/// Shortest distance between two line segments, by the standard closest-points solve with
/// both parameters clamped to their segments.
fn segment_distance(p1: Vec3, q1: Vec3, p2: Vec3, q2: Vec3) -> f64 {
    let (d1, d2, r) = (q1 - p1, q2 - p2, p1 - p2);
    let (a, e) = (d1.length_squared(), d2.length_squared());
    let tiny = MERGE_TOL * MERGE_TOL;
    if a <= tiny {
        return p1.distance(closest_on_segment(p1, p2, q2));
    }
    if e <= tiny {
        return p2.distance(closest_on_segment(p2, p1, q1));
    }
    let (c, f, b) = (d1.dot(r), d2.dot(r), d1.dot(d2));
    let denom = a * e - b * b;
    // Parallel segments leave the pair of parameters underdetermined; pinning one end and
    // clamping the other lands on a genuine closest pair, which is all that is wanted.
    let mut s = if denom > tiny {
        ((b * f - c * e) / denom).clamp(0.0, 1.0)
    } else {
        0.0
    };
    let mut t = (b * s + f) / e;
    if t < 0.0 {
        t = 0.0;
        s = (-c / a).clamp(0.0, 1.0);
    } else if t > 1.0 {
        t = 1.0;
        s = ((b - c) / a).clamp(0.0, 1.0);
    }
    (p1 + d1 * s).distance(p2 + d2 * t)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ids::{FaceKey, FaceRole, OpId};
    use crate::primitives::{cuboid, cylinder};
    use approx::assert_relative_eq;

    fn face_of(solid: &crate::Solid, role: FaceRole, op: u64) -> &Face {
        solid
            .faces
            .iter()
            .find(|f| f.key == FaceKey::new(OpId::new(op), role))
            .expect("the face is there")
    }

    /// The block every measurement test starts from: 10 × 10 × 2 at the origin.
    fn block() -> crate::Solid {
        cuboid(OpId::new(1), Vec3::ZERO, Vec3::new(10.0, 10.0, 2.0))
    }

    #[test]
    fn a_block_reports_its_volume_area_and_box() {
        let c = block();
        assert_relative_eq!(c.volume(), 200.0, epsilon = 1e-9);
        assert_relative_eq!(
            c.surface_area(),
            2.0 * (100.0 + 20.0 + 20.0),
            epsilon = 1e-9
        );
        let bbox = c.aabb();
        assert_relative_eq!(bbox.extent().x, 10.0, epsilon = 1e-9);
        assert_relative_eq!(bbox.extent().z, 2.0, epsilon = 1e-9);
    }

    #[test]
    fn a_face_reports_its_area_and_perimeter() {
        let c = block();
        let top = face_of(&c, FaceRole::EndCap, 1);
        assert_relative_eq!(top.area(), 100.0, epsilon = 1e-9);
        assert_relative_eq!(face_perimeter(top), 40.0, epsilon = 1e-9);
        let side = face_of(&c, FaceRole::Side(0), 1);
        assert_relative_eq!(side.area(), 20.0, epsilon = 1e-9);
        assert_relative_eq!(face_perimeter(side), 24.0, epsilon = 1e-9);
    }

    /// Splitting a face into two polygons must not put the seam into the perimeter: it is
    /// used by both halves, so it cancels. This is the shape every boolean leaves behind.
    #[test]
    fn an_internal_seam_is_not_part_of_the_perimeter() {
        let mut c = block();
        let top = FaceKey::new(OpId::new(1), FaceRole::EndCap);
        let i = c.faces.iter().position(|f| f.key == top).expect("top face");
        let v = |x: f64, y: f64| Vec3::new(x, y, 2.0);
        c.faces[i].polygons = vec![
            crate::solid::Polygon::new(vec![v(0., 0.), v(10., 0.), v(10., 5.), v(0., 5.)])
                .expect("half"),
            crate::solid::Polygon::new(vec![v(0., 5.), v(10., 5.), v(10., 10.), v(0., 10.)])
                .expect("half"),
        ];
        assert_relative_eq!(c.faces[i].area(), 100.0, epsilon = 1e-9);
        assert_relative_eq!(face_perimeter(&c.faces[i]), 40.0, epsilon = 1e-9);
    }

    #[test]
    fn an_edge_is_straight_or_circular_but_never_both() {
        let c = block();
        let edges = c.edges();
        let straight = edges
            .iter()
            .find(|e| {
                e.key.touches(FaceKey::new(OpId::new(1), FaceRole::EndCap))
                    && e.key.touches(FaceKey::new(OpId::new(1), FaceRole::Side(0)))
            })
            .expect("a top edge");
        assert_relative_eq!(straight.length(), 10.0, epsilon = 1e-9);
        assert!(edge_direction(straight).is_some());
        assert!(edge_circle(straight).is_none());

        let cyl = cylinder(
            OpId::new(2),
            Vec3::ZERO,
            Vec3::Z,
            3.0,
            5.0,
            &crate::geometry::Tessellation::default(),
        );
        let rim = cyl
            .edges()
            .into_iter()
            .find(|e| edge_circle(e).is_some())
            .expect("a cylinder has circular rims");
        let circle = edge_circle(&rim).expect("checked above");
        assert_relative_eq!(circle.radius, 3.0, epsilon = 1e-9);
        assert!(parallel(circle.axis, Vec3::Z));
        assert!(edge_direction(&rim).is_none(), "a rim is not a line");
    }

    #[test]
    fn parallel_faces_give_a_distance_and_the_rest_give_an_angle() {
        let c = block();
        let top = face_of(&c, FaceRole::EndCap, 1);
        let bottom = face_of(&c, FaceRole::StartCap, 1);
        let side = face_of(&c, FaceRole::Side(0), 1);
        assert_relative_eq!(
            parallel_face_distance(top, bottom).expect("parallel"),
            2.0,
            epsilon = 1e-9
        );
        assert!(parallel_face_distance(top, side).is_none());
        assert_relative_eq!(
            face_angle(top, side).expect("planar").to_degrees(),
            90.0,
            epsilon = 1e-9
        );
        // The same pair the other way round reads the same, which is the whole point of
        // measuring between the planes rather than between the outward normals.
        assert_relative_eq!(
            face_angle(side, top).expect("planar"),
            face_angle(top, side).expect("planar"),
            epsilon = 1e-12
        );
    }

    #[test]
    fn an_edge_measures_against_a_face_it_lies_along_or_leans_on() {
        let c = block();
        let edges = c.edges();
        let top_edge = edges
            .iter()
            .find(|e| {
                e.key.touches(FaceKey::new(OpId::new(1), FaceRole::EndCap))
                    && e.key.touches(FaceKey::new(OpId::new(1), FaceRole::Side(0)))
            })
            .expect("a top edge");
        let bottom = face_of(&c, FaceRole::StartCap, 1);
        let side = face_of(&c, FaceRole::Side(1), 1);
        assert_relative_eq!(
            edge_face_distance(top_edge, bottom).expect("parallel"),
            2.0,
            epsilon = 1e-9
        );
        assert_relative_eq!(
            edge_face_angle(top_edge, bottom)
                .expect("planar")
                .to_degrees(),
            0.0,
            epsilon = 1e-9
        );
        assert!(edge_face_distance(top_edge, side).is_none());
        assert_relative_eq!(
            edge_face_angle(top_edge, side)
                .expect("planar")
                .to_degrees(),
            90.0,
            epsilon = 1e-9
        );
    }

    #[test]
    fn edges_report_their_angle_and_their_nearest_approach() {
        let c = block();
        let edges = c.edges();
        let top = |side: u32| {
            edges
                .iter()
                .find(|e| {
                    e.key.touches(FaceKey::new(OpId::new(1), FaceRole::EndCap))
                        && e.key
                            .touches(FaceKey::new(OpId::new(1), FaceRole::Side(side)))
                })
                .expect("a top edge")
        };
        // Neighbouring edges of the top face meet at a corner: 90°, and they touch.
        assert_relative_eq!(
            edge_angle(top(0), top(1)).expect("straight").to_degrees(),
            90.0,
            epsilon = 1e-9
        );
        assert_relative_eq!(edge_distance(top(0), top(1)), 0.0, epsilon = 1e-9);
        // Opposite edges are parallel and ten apart.
        assert_relative_eq!(
            edge_angle(top(0), top(2)).expect("straight").to_degrees(),
            0.0,
            epsilon = 1e-9
        );
        assert_relative_eq!(edge_distance(top(0), top(2)), 10.0, epsilon = 1e-9);
    }

    #[test]
    fn a_point_measures_to_an_edge_and_to_a_plane() {
        let c = block();
        let edges = c.edges();
        let top_edge = edges
            .iter()
            .find(|e| {
                e.key.touches(FaceKey::new(OpId::new(1), FaceRole::EndCap))
                    && e.key.touches(FaceKey::new(OpId::new(1), FaceRole::Side(0)))
            })
            .expect("a top edge");
        // The far top corner is ten from the near top edge, straight across the face.
        let corner = Vec3::new(10.0, 10.0, 2.0);
        assert_relative_eq!(point_edge_distance(corner, top_edge), 10.0, epsilon = 1e-9);
        let bottom = face_of(&c, FaceRole::StartCap, 1);
        assert_relative_eq!(
            point_face_distance(corner, bottom).expect("planar"),
            2.0,
            epsilon = 1e-9
        );
        // Off the end of the segment the answer is the distance to its nearer end, not to
        // the infinite line: an edge is a finite thing.
        let beyond = Vec3::new(-3.0, 0.0, 2.0);
        assert_relative_eq!(point_edge_distance(beyond, top_edge), 3.0, epsilon = 1e-9);
    }
}
