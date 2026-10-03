//! Fillets and chamfers on *compound* bodies: the results of several features, where the
//! edges to blend cross boolean seams, run round holes, meet other blends or chain
//! through corners of mixed convexity. Each body is built the way the editor builds it
//! (generators and booleans), the edges are found by where they lie rather than by a key
//! guessed in advance, and every result is held to a closed shell and an analytic volume.

use std::f64::consts::PI;

use approx::assert_relative_eq;
use basset_kernel::blend::{max_fillet_radius, max_inverted_fillet_radius};
use basset_kernel::primitives::{cuboid, cylinder};
use basset_kernel::{
    BoolOp, Contour, Edge, EdgeKey, Extent, FaceKey, FaceRole, KernelError, OpId, Profile, Solid,
    SurfaceKind, Tessellation, boolean, chamfer, extrude, fillet,
};
use basset_math::{Frame, Vec2, Vec3};

// --- Analytic helpers -----------------------------------------------------------------

/// Cross-section a regular fillet of radius `r` takes off (or adds to) a right-angled
/// edge: the r × r square less the quarter disc.
fn round_area(r: f64) -> f64 {
    r * r * (1.0 - PI / 4.0)
}

/// How far that section's centroid sits from the edge along either face. The square's
/// first moment is r³/2, the quarter disc's is πr³/4 − r³/3, so the remainder's is
/// r³(5/6 − π/4) over an area of r²(1 − π/4).
fn round_offset(r: f64) -> f64 {
    r * (5.0 / 6.0 - PI / 4.0) / (1.0 - PI / 4.0)
}

/// Cross-section of an inverted fillet at a right angle: the quarter disc about the edge.
fn cove_area(r: f64) -> f64 {
    PI * r * r / 4.0
}

/// A quarter disc's centroid, from its corner along either leg.
fn cove_offset(r: f64) -> f64 {
    4.0 * r / (3.0 * PI)
}

/// Pappus: a section of area `area` whose centroid is `rho` from the axis, swept once
/// round it.
fn swept(area: f64, rho: f64) -> f64 {
    2.0 * PI * rho * area
}

fn fine() -> Tessellation {
    Tessellation {
        chord_tolerance: 1e-4,
        max_segment_angle: 2f64.to_radians(),
    }
}

fn near(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-6
}

/// Within a facet's sagitta: a vertex healing put on a rim's chord lies that far inside
/// the circle, and the default tessellation's 10° facets on a radius-5 circle sag 0.02.
fn on_circle(p: Vec3, centre: Vec3, radius: f64) -> bool {
    ((p - centre).length() - radius).abs() < 0.03
}

/// The one selectable edge every vertex of which satisfies `on`.
fn edge_where(solid: &Solid, on: impl Fn(Vec3) -> bool) -> EdgeKey {
    let found: Vec<Edge> = solid
        .edges()
        .into_iter()
        .filter(|e| !e.smooth && e.segments.iter().all(|s| on(s.start) && on(s.end)))
        .collect();
    assert_eq!(
        found.len(),
        1,
        "expected one edge, found {:?}",
        found.iter().map(|e| e.key).collect::<Vec<_>>()
    );
    found[0].key
}

/// Every selectable edge bordering `face`: what a face pick stands for in the editor.
fn edges_of_face(solid: &Solid, face: FaceKey) -> Vec<EdgeKey> {
    solid
        .edges()
        .into_iter()
        .filter(|e| !e.smooth && e.key.touches(face))
        .map(|e| e.key)
        .collect()
}

fn assert_closed(s: &Solid) {
    assert!(s.is_closed(), "{:?}", s.validate());
}

// --- Bodies ---------------------------------------------------------------------------

/// A 20 × 20 × 10 block with a radius-3 hole through its middle.
fn holed_block() -> Solid {
    let block = cuboid(OpId::new(1), Vec3::ZERO, Vec3::new(20.0, 20.0, 10.0));
    let drill = cylinder(
        OpId::new(2),
        Vec3::new(10.0, 10.0, -1.0),
        Vec3::Z,
        3.0,
        12.0,
        &Tessellation::default(),
    );
    boolean(&block, &drill, BoolOp::Subtract).unwrap()
}

fn hole_rim(s: &Solid) -> EdgeKey {
    edge_where(s, |p| {
        near(p.z, 10.0) && on_circle(p, Vec3::new(10.0, 10.0, 10.0), 3.0)
    })
}

/// A 20 × 20 × 10 block with a radius-5 cylindrical boss standing 5 on its top: the
/// boss's base is coplanar with the block's top, as an extrude joined from that face is.
fn bossed_block() -> Solid {
    let block = cuboid(OpId::new(1), Vec3::ZERO, Vec3::new(20.0, 20.0, 10.0));
    let boss = cylinder(
        OpId::new(2),
        Vec3::new(10.0, 10.0, 10.0),
        Vec3::Z,
        5.0,
        5.0,
        &Tessellation::default(),
    );
    boolean(&block, &boss, BoolOp::Union).unwrap()
}

fn boss_foot(s: &Solid) -> EdgeKey {
    edge_where(s, |p| {
        near(p.z, 10.0) && on_circle(p, Vec3::new(10.0, 10.0, 10.0), 5.0)
    })
}

fn boss_rim(s: &Solid) -> EdgeKey {
    edge_where(s, |p| {
        near(p.z, 15.0) && on_circle(p, Vec3::new(10.0, 10.0, 15.0), 5.0)
    })
}

/// The same block with a 10 × 10 × 5 square boss on it.
fn square_bossed_block() -> Solid {
    let block = cuboid(OpId::new(1), Vec3::ZERO, Vec3::new(20.0, 20.0, 10.0));
    let boss = cuboid(
        OpId::new(2),
        Vec3::new(5.0, 5.0, 10.0),
        Vec3::new(15.0, 15.0, 15.0),
    );
    boolean(&block, &boss, BoolOp::Union).unwrap()
}

/// A T in plan: a 20 × 10 bar along x with a 10 × 15 stem up +y from its middle, both
/// 10 high, unioned. Two concave vertical edges at (5, 10) and (15, 10).
fn t_block() -> Solid {
    let bar = cuboid(OpId::new(1), Vec3::ZERO, Vec3::new(20.0, 10.0, 10.0));
    let stem = cuboid(
        OpId::new(2),
        Vec3::new(5.0, 5.0, 0.0),
        Vec3::new(15.0, 20.0, 10.0),
    );
    boolean(&bar, &stem, BoolOp::Union).unwrap()
}

fn t_inner_edge(s: &Solid, x: f64) -> EdgeKey {
    edge_where(s, |p| near(p.x, x) && near(p.y, 10.0))
}

/// Two 10-cubes side by side along x, unioned: their tops merge into one face, their
/// fronts into another, and the top-front edge is one key carrying two collinear
/// segments with a vertex at x = 10 that no geometry asked for.
fn split_bar() -> Solid {
    let a = cuboid(OpId::new(1), Vec3::ZERO, Vec3::splat(10.0));
    let b = cuboid(
        OpId::new(2),
        Vec3::new(10.0, 0.0, 0.0),
        Vec3::new(20.0, 10.0, 10.0),
    );
    boolean(&a, &b, BoolOp::Union).unwrap()
}

/// A 20 × 10 × 10 bar extruded from a profile with a redundant collinear vertex half way
/// along its front: its front is two coplanar faces of one operation, so the top-front
/// edge is two collinear keys meeting at a vertex.
fn collinear_profile_bar() -> Solid {
    let mut outer = Contour::polygon(
        vec![
            Vec2::ZERO,
            Vec2::new(10.0, 0.0),
            Vec2::new(20.0, 0.0),
            Vec2::new(20.0, 10.0),
            Vec2::new(0.0, 10.0),
        ],
        0,
    );
    for (i, s) in outer.segments.iter_mut().enumerate() {
        s.curve = i as u32;
    }
    extrude(
        OpId::new(1),
        &Profile::new(Frame::XY, outer),
        Extent::OneSide(10.0),
    )
    .unwrap()
}

/// A 10-cube whose top-front edge has been rounded at 2.
fn rounded_cube() -> Solid {
    let c = cuboid(OpId::new(1), Vec3::ZERO, Vec3::splat(10.0));
    let key = EdgeKey::new(
        FaceKey::new(OpId::new(1), FaceRole::EndCap),
        FaceKey::new(OpId::new(1), FaceRole::Side(0)),
    );
    fillet(OpId::new(2), &c, &[key], 2.0, &fine()).unwrap()
}

/// A radius-5 cylinder whose top has been cut off by a plane tilted 20° about x through
/// (0, 0, 8): the rim is an ellipse whose dihedral angle runs from 70° to 110°.
fn tilted_cylinder() -> (Solid, Vec3) {
    let cyl = cylinder(
        OpId::new(1),
        Vec3::ZERO,
        Vec3::Z,
        5.0,
        12.0,
        &Tessellation::default(),
    );
    let tilt = 20f64.to_radians();
    let normal = Vec3::new(0.0, tilt.sin(), tilt.cos());
    let frame = Frame::from_normal(Vec3::new(0.0, 0.0, 8.0), normal);
    let mut square = Contour::polygon(
        vec![
            Vec2::new(-20.0, -20.0),
            Vec2::new(20.0, -20.0),
            Vec2::new(20.0, 20.0),
            Vec2::new(-20.0, 20.0),
        ],
        0,
    );
    for (i, s) in square.segments.iter_mut().enumerate() {
        s.curve = i as u32;
    }
    let cutter = extrude(
        OpId::new(2),
        &Profile::new(frame, square),
        Extent::OneSide(20.0),
    )
    .unwrap();
    (boolean(&cyl, &cutter, BoolOp::Subtract).unwrap(), normal)
}

// --- Through-hole ---------------------------------------------------------------------

#[test]
fn rounding_the_rim_of_a_through_hole() {
    let body = holed_block();
    let v0 = body.volume();
    let rim = hole_rim(&body);
    let r = fillet(OpId::new(3), &body, &[rim], 1.0, &Tessellation::default()).unwrap();
    assert_closed(&r);
    // The material is outside the hole, so the removed ring's centroid sits a little
    // beyond the hole's radius.
    let expected = v0 - swept(round_area(1.0), 3.0 + round_offset(1.0));
    assert_relative_eq!(r.volume(), expected, epsilon = 0.3);
    assert!(
        r.face(FaceKey::new(OpId::new(3), FaceRole::Fillet(0)))
            .is_some()
    );
    // A fine arc asked for along a curved chain is drawn no finer than the end facets
    // the boolean can cut the faces along; the result is the same round, closed.
    let r = fillet(OpId::new(3), &body, &[rim], 1.0, &fine()).unwrap();
    assert_closed(&r);
    assert_relative_eq!(r.volume(), expected, epsilon = 0.3);
}

#[test]
fn coving_the_rim_of_a_through_hole() {
    let body = holed_block();
    let v0 = body.volume();
    let rim = hole_rim(&body);
    let r = fillet(OpId::new(3), &body, &[rim], -1.0, &Tessellation::default()).unwrap();
    assert_closed(&r);
    let expected = v0 - swept(cove_area(1.0), 3.0 + cove_offset(1.0));
    assert_relative_eq!(r.volume(), expected, epsilon = 0.3);
    assert!(r.aabb().max.z <= 10.0 + 1e-9);
}

#[test]
fn rounding_a_holes_rim_and_the_top_outer_edges_in_one_feature() {
    let body = holed_block();
    let v0 = body.volume();
    let top = FaceKey::new(OpId::new(1), FaceRole::EndCap);
    let keys = edges_of_face(&body, top);
    assert_eq!(keys.len(), 5, "four outer edges and the rim: {keys:?}");
    let r = fillet(OpId::new(3), &body, &keys, 1.0, &Tessellation::default()).unwrap();
    assert_closed(&r);
    let rim = swept(round_area(1.0), 3.0 + round_offset(1.0));
    let outer = 4.0 * 20.0 * round_area(1.0);
    // The four outer tools overlap at the corners, so a little less than their sum goes.
    assert!(r.volume() > v0 - rim - outer - 0.3, "{}", r.volume());
    assert!(
        r.volume() < v0 - rim - outer + 4.0 * round_area(1.0) + 0.3,
        "{}",
        r.volume()
    );
    let bb = r.aabb();
    assert_relative_eq!(bb.max.z, 10.0, epsilon = 1e-6);
    assert_relative_eq!(bb.max.x, 20.0, epsilon = 1e-6);
}

// --- Boss on a face -------------------------------------------------------------------

#[test]
fn rounding_the_concave_foot_of_a_boss() {
    let body = bossed_block();
    let v0 = body.volume();
    let foot = boss_foot(&body);
    assert_relative_eq!(
        max_fillet_radius(&body, &[foot]).unwrap(),
        5.0,
        epsilon = 0.05
    );
    let r = fillet(OpId::new(3), &body, &[foot], 1.0, &Tessellation::default()).unwrap();
    assert_closed(&r);
    let expected = v0 + swept(round_area(1.0), 5.0 + round_offset(1.0));
    assert_relative_eq!(r.volume(), expected, epsilon = 0.3);
    let bb = r.aabb();
    assert_relative_eq!(bb.max.z, 15.0, epsilon = 1e-6);
    assert_relative_eq!(bb.max.x, 20.0, epsilon = 1e-6);
}

#[test]
fn beading_the_concave_foot_of_a_boss() {
    let body = bossed_block();
    let v0 = body.volume();
    let foot = boss_foot(&body);
    assert_relative_eq!(
        max_inverted_fillet_radius(&body, &[foot]).unwrap(),
        5.0,
        epsilon = 0.05
    );
    let r = fillet(OpId::new(3), &body, &[foot], -1.0, &Tessellation::default()).unwrap();
    assert_closed(&r);
    let expected = v0 + swept(cove_area(1.0), 5.0 + cove_offset(1.0));
    assert_relative_eq!(r.volume(), expected, epsilon = 0.3);
}

#[test]
fn rounding_the_rim_of_a_boss() {
    let body = bossed_block();
    let v0 = body.volume();
    let rim = boss_rim(&body);
    let r = fillet(OpId::new(3), &body, &[rim], 1.0, &Tessellation::default()).unwrap();
    assert_closed(&r);
    let expected = v0 - swept(round_area(1.0), 5.0 - round_offset(1.0));
    assert_relative_eq!(r.volume(), expected, epsilon = 0.3);
}

/// One feature, one subtracted tool and one added: the foot is concave, the rim convex.
#[test]
fn rounding_a_boss_foot_and_rim_together() {
    let body = bossed_block();
    let v0 = body.volume();
    let foot = boss_foot(&body);
    let rim = boss_rim(&body);
    let expected = v0 + swept(round_area(1.0), 5.0 + round_offset(1.0))
        - swept(round_area(1.0), 5.0 - round_offset(1.0));
    for keys in [[foot, rim], [rim, foot]] {
        let r = fillet(OpId::new(3), &body, &keys, 1.0, &Tessellation::default()).unwrap();
        assert_closed(&r);
        assert_relative_eq!(r.volume(), expected, epsilon = 0.3);
        assert_relative_eq!(r.aabb().max.z, 15.0, epsilon = 1e-6);
    }
}

/// Four concave edges round the foot of a square boss, meeting at its corners.
#[test]
fn rounding_the_four_concave_edges_round_a_square_boss() {
    let body = square_bossed_block();
    assert_relative_eq!(body.volume(), 4000.0 + 500.0, epsilon = 1e-6);
    let top = FaceKey::new(OpId::new(1), FaceRole::EndCap);
    let feet: Vec<EdgeKey> = edges_of_face(&body, top)
        .into_iter()
        .filter(|k| {
            k.touches(FaceKey::new(OpId::new(2), FaceRole::Side(0)))
                || k.touches(FaceKey::new(OpId::new(2), FaceRole::Side(1)))
                || k.touches(FaceKey::new(OpId::new(2), FaceRole::Side(2)))
                || k.touches(FaceKey::new(OpId::new(2), FaceRole::Side(3)))
        })
        .collect();
    assert_eq!(feet.len(), 4, "{feet:?}");
    let r = fillet(OpId::new(3), &body, &feet, 1.0, &fine()).unwrap();
    assert_closed(&r);
    // Each bead runs the 10 of its edge; at the boss's corners the beads do not overlap
    // (they lie in different quadrants), so the sum is exact.
    assert_relative_eq!(
        r.volume(),
        4500.0 + 4.0 * 10.0 * round_area(1.0),
        epsilon = 0.05
    );
    let bb = r.aabb();
    assert_relative_eq!(bb.max.z, 15.0, epsilon = 1e-6);
    assert_relative_eq!(bb.max.x, 20.0, epsilon = 1e-6);
}

/// The whole boss: its four concave feet and its four convex top edges in one feature.
#[test]
fn rounding_every_edge_of_a_square_boss() {
    let body = square_bossed_block();
    let top = FaceKey::new(OpId::new(1), FaceRole::EndCap);
    let boss_top = FaceKey::new(OpId::new(2), FaceRole::EndCap);
    let mut keys: Vec<EdgeKey> = edges_of_face(&body, top)
        .into_iter()
        .filter(|k| k.a.op == OpId::new(2) || k.b.op == OpId::new(2))
        .collect();
    keys.extend(edges_of_face(&body, boss_top));
    assert_eq!(keys.len(), 8, "{keys:?}");
    let r = fillet(OpId::new(3), &body, &keys, 1.0, &fine()).unwrap();
    assert_closed(&r);
    let added = 4.0 * 10.0 * round_area(1.0);
    let removed = 4.0 * 10.0 * round_area(1.0);
    assert!(
        r.volume() > 4500.0 + added - removed - 0.05,
        "{}",
        r.volume()
    );
    assert!(
        r.volume() < 4500.0 + added - removed + 4.0 * round_area(1.0) + 0.05,
        "{}",
        r.volume()
    );
}

// --- Unions of overlapping blocks -----------------------------------------------------

#[test]
fn rounding_the_concave_inside_edges_of_a_t() {
    let body = t_block();
    assert_relative_eq!(body.volume(), 3000.0, epsilon = 1e-6);
    let keys = [t_inner_edge(&body, 5.0), t_inner_edge(&body, 15.0)];
    let r = fillet(OpId::new(3), &body, &keys, 1.0, &fine()).unwrap();
    assert_closed(&r);
    assert_relative_eq!(
        r.volume(),
        3000.0 + 2.0 * 10.0 * round_area(1.0),
        epsilon = 0.05
    );
    let bb = r.aabb();
    assert_relative_eq!(bb.max.z, 10.0, epsilon = 1e-6);
    assert_relative_eq!(bb.min.z, 0.0, epsilon = 1e-6);
}

#[test]
fn beading_the_concave_inside_edge_of_a_t() {
    let body = t_block();
    let key = t_inner_edge(&body, 15.0);
    let r = fillet(OpId::new(3), &body, &[key], -1.0, &fine()).unwrap();
    assert_closed(&r);
    assert_relative_eq!(r.volume(), 3000.0 + 10.0 * cove_area(1.0), epsilon = 0.05);
    assert_relative_eq!(r.aabb().max.z, 10.0, epsilon = 1e-6);
}

/// The top outline of the T: eight convex edges through six convex and two concave plan
/// corners, one of the bar's top edges having been split in two by the stem.
#[test]
fn rounding_the_whole_top_loop_of_a_t() {
    let body = t_block();
    let top = FaceKey::new(OpId::new(1), FaceRole::EndCap);
    let keys = edges_of_face(&body, top);
    assert_eq!(
        keys.len(),
        7,
        "the bar's far edge is one key in two pieces: {keys:?}"
    );
    let r = fillet(OpId::new(3), &body, &keys, 1.0, &fine()).unwrap();
    assert_closed(&r);
    let perimeter = 80.0;
    let removed = perimeter * round_area(1.0);
    assert!(r.volume() > 3000.0 - removed - 0.05, "{}", r.volume());
    assert!(
        r.volume() < 3000.0 - removed + 6.0 * round_area(1.0) + 0.05,
        "{}",
        r.volume()
    );
    let bb = r.aabb();
    assert_relative_eq!(bb.max.z, 10.0, epsilon = 1e-6);
    assert_relative_eq!(bb.max.y, 20.0, epsilon = 1e-6);
}

/// The top loop and the two concave inside edges together: a round that ends at the
/// vertex where a bead starts would have to roll onto the bead, a corner blend this
/// kernel does not build, so the feature is refused by name rather than returning a
/// shell with a hole in it (the tools stop a clearance short of the vertex, and each
/// one's faces slice the other's end cap into fragments finer than the healer pairs).
#[test]
fn a_round_meeting_a_bead_at_a_vertex_is_refused() {
    let body = t_block();
    let top = FaceKey::new(OpId::new(1), FaceRole::EndCap);
    let mut keys = edges_of_face(&body, top);
    let inner = t_inner_edge(&body, 15.0);
    keys.push(inner);
    let err = fillet(OpId::new(3), &body, &keys, 1.0, &fine()).unwrap_err();
    assert!(
        matches!(err, KernelError::ConvexMeetsConcave { concave, .. } if concave == inner),
        "{err:?}"
    );
    // Picked the other way round, the same refusal.
    keys.reverse();
    assert!(matches!(
        fillet(OpId::new(3), &body, &keys, 1.0, &fine()).unwrap_err(),
        KernelError::ConvexMeetsConcave { .. }
    ));
    // A bead whose ends meet nothing being rounded is fine alongside rounds elsewhere:
    // the bar's far end edges and the stem's inside edge share no vertex.
    let far: Vec<EdgeKey> = edges_of_face(&body, top)
        .into_iter()
        .filter(|k| k.touches(FaceKey::new(OpId::new(1), FaceRole::Side(1))))
        .collect();
    assert_eq!(far.len(), 1, "{far:?}");
    let mut keys = far;
    keys.push(t_inner_edge(&body, 5.0));
    let r = fillet(OpId::new(3), &body, &keys, 1.0, &fine()).unwrap();
    assert_closed(&r);
    assert_relative_eq!(
        r.volume(),
        3000.0 - 10.0 * round_area(1.0) + 10.0 * round_area(1.0),
        epsilon = 0.05
    );
}

/// The L-block's step profile: three convex edges and one concave, all along x, plus the
/// convex vertical edge whose foot stands on the concave one.
#[test]
fn rounding_a_step_profile_of_mixed_convexity() {
    let cube = cuboid(OpId::new(1), Vec3::ZERO, Vec3::splat(10.0));
    let notch = cuboid(
        OpId::new(9),
        Vec3::new(-1.0, -1.0, 5.0),
        Vec3::new(11.0, 5.0, 11.0),
    );
    let body = boolean(&cube, &notch, BoolOp::Subtract).unwrap();
    assert_relative_eq!(body.volume(), 750.0, epsilon = 1e-6);
    let along_x = |y: f64, z: f64| edge_where(&body, |p| near(p.y, y) && near(p.z, z));
    let keys = [
        along_x(10.0, 10.0),
        along_x(5.0, 10.0),
        along_x(5.0, 5.0),
        along_x(0.0, 5.0),
    ];
    let r = fillet(OpId::new(3), &body, &keys, 1.5, &fine()).unwrap();
    assert_closed(&r);
    let a = round_area(1.5);
    assert_relative_eq!(
        r.volume(),
        750.0 - 3.0 * 10.0 * a + 10.0 * a,
        epsilon = 0.05
    );

    // The vertical edge at the wall's end stands with its foot on the concave edge: a
    // round running into a bead, which is the corner the kernel refuses to fake.
    let wall_end = edge_where(&body, |p| {
        near(p.x, 0.0) && near(p.y, 5.0) && p.z >= 5.0 - 1e-9
    });
    let mut keys = keys.to_vec();
    keys.push(wall_end);
    assert!(matches!(
        fillet(OpId::new(3), &body, &keys, 1.5, &fine()).unwrap_err(),
        KernelError::ConvexMeetsConcave { convex, concave }
            if convex == wall_end && concave == along_x(5.0, 5.0)
    ));
}

// --- Fillet meeting a fillet ----------------------------------------------------------

#[test]
fn rounding_the_arc_where_a_fillet_meets_the_end_face() {
    let body = rounded_cube();
    let v0 = body.volume();
    let fillet_face = FaceKey::new(OpId::new(2), FaceRole::Fillet(0));
    let end = FaceKey::new(OpId::new(1), FaceRole::Side(1));
    let arc = EdgeKey::new(fillet_face, end);
    assert!(body.edges().iter().any(|e| e.key == arc && !e.smooth));
    let r = fillet(OpId::new(3), &body, &[arc], 0.5, &fine()).unwrap();
    assert_closed(&r);
    // A quarter turn about the first fillet's axis; the material lies outside that
    // cylinder, so the removed section's centroid is beyond its radius.
    let expected = v0 - (PI / 2.0) * (2.0 + round_offset(0.5)) * round_area(0.5);
    assert_relative_eq!(r.volume(), expected, epsilon = 0.02);
}

#[test]
fn rounding_a_vertical_edge_that_ends_against_a_fillet() {
    let body = rounded_cube();
    let v0 = body.volume();
    let key = EdgeKey::new(
        FaceKey::new(OpId::new(1), FaceRole::Side(0)),
        FaceKey::new(OpId::new(1), FaceRole::Side(1)),
    );
    let edge = body.edges().into_iter().find(|e| e.key == key).unwrap();
    assert_relative_eq!(edge.length(), 8.0, epsilon = 1e-9);
    let r = fillet(OpId::new(3), &body, &[key], 1.0, &fine()).unwrap();
    assert_closed(&r);
    assert_relative_eq!(r.volume(), v0 - 8.0 * round_area(1.0), epsilon = 0.02);
}

/// The tangent run-out of a fillet is a face boundary but not a corner: there is no
/// dihedral there to fill, and the kernel says so rather than building a flat tool.
#[test]
fn a_fillets_tangent_boundary_is_refused_by_name() {
    let body = rounded_cube();
    let key = EdgeKey::new(
        FaceKey::new(OpId::new(2), FaceRole::Fillet(0)),
        FaceKey::new(OpId::new(1), FaceRole::EndCap),
    );
    let edge = body.edges().into_iter().find(|e| e.key == key).unwrap();
    // Selectable (two different surfaces meet there) but not a corner to fill.
    assert!(!edge.smooth && !edge.blendable());
    assert_eq!(
        fillet(OpId::new(3), &body, &[key], 0.5, &fine()).unwrap_err(),
        KernelError::TangentEdge(key)
    );
}

/// Two fillets meeting at a corner leave a crease; the vertical edge below the crease
/// still rounds cleanly up to it.
#[test]
fn rounding_the_vertical_edge_under_a_creased_corner() {
    let c = cuboid(OpId::new(1), Vec3::ZERO, Vec3::splat(10.0));
    let top = |i| {
        EdgeKey::new(
            FaceKey::new(OpId::new(1), FaceRole::EndCap),
            FaceKey::new(OpId::new(1), FaceRole::Side(i)),
        )
    };
    let body = fillet(OpId::new(2), &c, &[top(0), top(1)], 2.0, &fine()).unwrap();
    let v0 = body.volume();
    let key = EdgeKey::new(
        FaceKey::new(OpId::new(1), FaceRole::Side(0)),
        FaceKey::new(OpId::new(1), FaceRole::Side(1)),
    );
    let r = fillet(OpId::new(3), &body, &[key], 1.0, &fine()).unwrap();
    assert_closed(&r);
    assert_relative_eq!(r.volume(), v0 - 8.0 * round_area(1.0), epsilon = 0.05);
}

// --- Straight edges a boolean split -----------------------------------------------

#[test]
fn a_straight_edge_split_by_a_seam_rounds_as_one() {
    let body = split_bar();
    assert_relative_eq!(body.volume(), 2000.0, epsilon = 1e-6);
    let key = edge_where(&body, |p| near(p.y, 0.0) && near(p.z, 10.0));
    let edge = body.edges().into_iter().find(|e| e.key == key).unwrap();
    assert_eq!(edge.chains().len(), 1);
    assert!(
        edge.segments.len() >= 2,
        "the seam's vertex splits the edge"
    );
    let r = fillet(OpId::new(3), &body, &[key], 2.0, &fine()).unwrap();
    assert_closed(&r);
    assert_relative_eq!(r.volume(), 2000.0 - 20.0 * round_area(2.0), epsilon = 0.01);
    let face = r
        .face(FaceKey::new(OpId::new(3), FaceRole::Fillet(0)))
        .unwrap();
    assert_relative_eq!(face.area(), PI * 2.0 / 2.0 * 20.0, epsilon = 0.02);
    assert!(
        matches!(face.surface, SurfaceKind::Cylindrical { radius, .. } if radius == 2.0),
        "one straight edge, however many pieces, is one cylinder: {:?}",
        face.surface
    );
    let c = chamfer(OpId::new(3), &body, &[key], 2.0).unwrap();
    assert_closed(&c);
    assert_relative_eq!(c.volume(), 2000.0 - 0.5 * 4.0 * 20.0, epsilon = 1e-6);
}

#[test]
fn every_top_edge_of_a_seamed_bar_rounds_together() {
    let body = split_bar();
    let top = FaceKey::new(OpId::new(1), FaceRole::EndCap);
    let keys = edges_of_face(&body, top);
    assert_eq!(keys.len(), 4, "{keys:?}");
    let r = fillet(OpId::new(3), &body, &keys, 2.0, &fine()).unwrap();
    assert_closed(&r);
    let removed = 60.0 * round_area(2.0);
    assert!(r.volume() > 2000.0 - removed - 0.05);
    assert!(r.volume() < 2000.0 - removed + 4.0 * 2.0 * round_area(2.0));
}

/// Two collinear keys of one operation: separate tools that have to meet exactly on the
/// plane through their shared vertex.
#[test]
fn two_collinear_edges_of_one_face_round_into_one_cylinder() {
    let body = collinear_profile_bar();
    assert_relative_eq!(body.volume(), 2000.0, epsilon = 1e-6);
    let top = FaceKey::new(OpId::new(1), FaceRole::EndCap);
    let front: Vec<EdgeKey> = edges_of_face(&body, top)
        .into_iter()
        .filter(|k| {
            k.touches(FaceKey::new(OpId::new(1), FaceRole::Side(0)))
                || k.touches(FaceKey::new(OpId::new(1), FaceRole::Side(1)))
        })
        .collect();
    assert_eq!(front.len(), 2, "{front:?}");
    // The pick walks from one onto the other: a straight run is one chain.
    let chain = basset_kernel::pick::tangent_chain(&body.edges(), front[0]);
    assert_eq!(chain.len(), 2, "{chain:?}");
    let r = fillet(OpId::new(3), &body, &front, 2.0, &fine()).unwrap();
    assert_closed(&r);
    assert_relative_eq!(r.volume(), 2000.0 - 20.0 * round_area(2.0), epsilon = 0.01);
}

// --- Varying dihedral -----------------------------------------------------------------

/// An edge whose dihedral angle changes along it — the rim of a cylinder cut off askew —
/// is refused by name: the tool's section rotates against the cylinder's facet seams
/// and the band's edge zigzags through them. The room the edge has is still measured
/// soundly: the rays fanned into the material stay inside the wedge between the faces,
/// where the ray along the cap's inward normal used to lean 20° outside the 70° wall
/// and leave through the next facet a fraction of a millimetre away.
#[test]
fn an_elliptical_rim_is_refused_but_its_room_is_measured_soundly() {
    let (body, normal) = tilted_cylinder();
    let rim = edge_where(&body, |p| {
        on_circle(Vec3::new(p.x, p.y, 0.0), Vec3::ZERO, 5.0)
            && near((p - Vec3::new(0.0, 0.0, 8.0)).dot(normal), 0.0)
    });
    let limit = max_fillet_radius(&body, &[rim]).unwrap();
    assert!(
        limit > 3.0,
        "eight millimetres of wall and a 10 mm cap: {limit}"
    );
    assert_eq!(
        fillet(OpId::new(3), &body, &[rim], 0.8, &Tessellation::default()).unwrap_err(),
        KernelError::VaryingDihedral(rim)
    );
}

// --- Limits on compound bodies --------------------------------------------------------

/// The room on a face a boolean has fragmented is the room of the whole face, not of the
/// fragment the ray happens to start in.
#[test]
fn the_limit_on_a_holed_top_is_the_room_to_the_hole() {
    let body = holed_block();
    let front = edge_where(&body, |p| near(p.y, 0.0) && near(p.z, 10.0));
    // From the front edge across the top the nearest thing is the hole, 7 away; down the
    // front face is 10.
    assert_relative_eq!(
        max_fillet_radius(&body, &[front]).unwrap(),
        7.0,
        epsilon = 0.05
    );
    let rim = hole_rim(&body);
    // From the rim outward the top reaches 7 to the nearest outer edge; down the hole 10.
    assert_relative_eq!(
        max_fillet_radius(&body, &[rim]).unwrap(),
        7.0,
        epsilon = 0.05
    );
    let r = fillet(OpId::new(3), &body, &[front], 6.0, &Tessellation::default()).unwrap();
    assert_closed(&r);
}

// --- What one pick takes -------------------------------------------------------------

/// A hole's rim is one closed key: one pick, the whole circle, and nothing else.
#[test]
fn a_pick_on_a_hole_rim_takes_the_rim_alone() {
    let body = holed_block();
    let rim = hole_rim(&body);
    assert_eq!(
        basset_kernel::pick::tangent_chain(&body.edges(), rim),
        vec![rim]
    );
}

/// A straight edge a seam split stays one key with one chain, so a pick on it is the
/// whole straight run; the corners stop the walk.
#[test]
fn a_pick_on_a_seamed_straight_edge_takes_the_whole_run() {
    let body = split_bar();
    let key = edge_where(&body, |p| near(p.y, 0.0) && near(p.z, 10.0));
    assert_eq!(
        basset_kernel::pick::tangent_chain(&body.edges(), key),
        vec![key]
    );
}

/// The tangent chain does not walk onto a fillet's run-out, which continues the top
/// edge's line exactly but cannot be blended: a pick on the top loop of a rounded cube
/// takes the three remaining sharp top edges and stops at the round.
#[test]
fn a_tangent_chain_stops_at_a_fillets_run_out() {
    let body = rounded_cube();
    let back = EdgeKey::new(
        FaceKey::new(OpId::new(1), FaceRole::EndCap),
        FaceKey::new(OpId::new(1), FaceRole::Side(2)),
    );
    let chain = basset_kernel::pick::tangent_chain(&body.edges(), back);
    assert_eq!(chain, vec![back], "the top's corners are sharp: {chain:?}");
    let run_out = EdgeKey::new(
        FaceKey::new(OpId::new(2), FaceRole::Fillet(0)),
        FaceKey::new(OpId::new(1), FaceRole::EndCap),
    );
    let side = EdgeKey::new(
        FaceKey::new(OpId::new(1), FaceRole::EndCap),
        FaceKey::new(OpId::new(1), FaceRole::Side(1)),
    );
    // The right-hand top edge ends where the run-out begins, in line with nothing else
    // selectable; the walk must not continue onto the run-out.
    let chain = basset_kernel::pick::tangent_chain(&body.edges(), side);
    assert!(!chain.contains(&run_out), "{chain:?}");
}
