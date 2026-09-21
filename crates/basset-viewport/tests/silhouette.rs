//! The silhouette pass, against the meshes it is actually handed.
//!
//! These need no GPU: the silhouette is decided on the CPU from facet adjacency, and what
//! reaches the GPU is the same pixel-width line batch every other overlay goes through.

use basset_kernel::{
    Contour, Extent, OpId, Profile, Solid, Tessellation,
    blend::fillet,
    generate::{extrude, loft},
    ids::{FaceKey, FaceRole},
    primitives::cuboid,
};
use basset_math::{Frame, Mat4, TriMesh, Vec2, Vec3};
use basset_viewport::{Camera, Projection, Silhouette, SilhouetteCache, ViewPoint};

const R: f64 = 5.0;

fn mesh_of(solid: &Solid) -> TriMesh {
    solid.tessellate().mesh
}

/// A cylinder of radius 5 standing on the origin, 10 tall: the body the whole problem is
/// named after, bounded by nothing but its shading until there is a silhouette.
fn cylinder() -> TriMesh {
    let tess = Tessellation::default();
    let profile = Profile::new(Frame::XY, Contour::circle(Vec2::ZERO, R, 0, &tess));
    let solid = extrude(OpId::new(1), &profile, Extent::OneSide(10.0)).expect("a cylinder");
    mesh_of(&solid)
}

/// A ball lofted through circles of latitude, radius 5 about the origin: curved in both
/// directions, so its silhouette is a ring rather than a pair of lines.
fn lofted_ball() -> TriMesh {
    let tess = Tessellation::default();
    let rings = 16;
    let profiles: Vec<Profile> = (0..=rings)
        .map(|i| {
            let theta = std::f64::consts::PI * i as f64 / rings as f64;
            // The poles become small flat caps rather than points: a loft joins sections,
            // and a section of no size is not one.
            let radius = (R * theta.sin()).max(0.2);
            let frame = Frame {
                origin: Vec3::new(0.0, 0.0, -R * theta.cos()),
                ..Frame::XY
            };
            Profile::new(frame, Contour::circle(Vec2::ZERO, radius, 0, &tess))
        })
        .collect();
    mesh_of(&loft(OpId::new(1), &profiles).expect("a lofted ball"))
}

/// A 10 mm cube with the top edge at y = 0 rounded by a 2 mm fillet, so the blend runs
/// out along y = 0, z = 8 and along y = 2, z = 10.
fn filleted_box() -> Solid {
    let op = OpId::new(1);
    let cube = cuboid(op, Vec3::ZERO, Vec3::splat(10.0));
    let key = basset_kernel::EdgeKey::new(
        FaceKey::new(op, FaceRole::EndCap),
        FaceKey::new(op, FaceRole::Side(0)),
    );
    fillet(OpId::new(2), &cube, &[key], 2.0, &Tessellation::default())
        .expect("filleting one top edge of a cube")
}

fn on_a_tangent_line(p: Vec3) -> bool {
    (p.y.abs() < 1e-6 && (p.z - 8.0).abs() < 1e-6)
        || ((p.y - 2.0).abs() < 1e-6 && (p.z - 10.0).abs() < 1e-6)
}

fn segments(mesh: &TriMesh, view: ViewPoint) -> Vec<[Vec3; 2]> {
    let silhouette = Silhouette::build(mesh);
    let mut facing = Vec::new();
    let mut out = Vec::new();
    silhouette.evaluate(view, &mut facing, &mut out);
    out
}

/// Segments running parallel to the cylinder's axis: the two lines down its sides.
fn upright(segments: &[[Vec3; 2]]) -> Vec<[Vec3; 2]> {
    segments
        .iter()
        .copied()
        .filter(|[a, b]| (a.z - b.z).abs() > 1e-9)
        .collect()
}

/// A camera at `distance` looking along −x at the middle of the cylinder.
fn side_on(distance: f64) -> Camera {
    Camera {
        target: Vec3::new(0.0, 0.0, 5.0),
        distance,
        yaw: 0.0,
        pitch: 0.0,
        projection: Projection::Perspective { fov_y: 0.7 },
    }
}

#[test]
fn a_cylinder_gets_two_lines_down_its_sides() {
    let mesh = cylinder();
    let camera = side_on(10.0);
    let view = ViewPoint::for_instance(&camera, &Mat4::IDENTITY);
    let sides = upright(&segments(&mesh, view));
    assert_eq!(sides.len(), 2, "one line down each side of the wall");
    // Tangent from an eye 10 mm out on a radius of 5: cos θ = r / d, so the line stands at
    // x = r·cos θ = 2.5 — round the back of the equator, not on it. One facet either side.
    for [a, b] in &sides {
        assert!(
            (2.0..3.0).contains(&a.x),
            "the line stands where the wall turns away from the eye, not on the equator: {a:?}"
        );
        assert!((a.x - b.x).abs() < 1e-9 && (a.y - b.y).abs() < 1e-9);
    }
    assert!(
        sides[0][0].y * sides[1][0].y < 0.0,
        "one line on each side of the body"
    );
}

/// The orthographic half. A parallel projection has no eye to take a tangent from: every
/// facet is judged against one direction, and the lines stand on the equator. Treating the
/// camera as an eye a long way off would put them somewhere between the two.
#[test]
fn an_orthographic_camera_puts_the_lines_on_the_equator() {
    let mesh = cylinder();
    let mut camera = side_on(10.0);
    camera.set_orthographic();
    let view = ViewPoint::for_instance(&camera, &Mat4::IDENTITY);
    assert!(matches!(view, ViewPoint::Direction(_)));
    let sides = upright(&segments(&mesh, view));
    assert_eq!(sides.len(), 2);
    for [a, _] in &sides {
        assert!(
            a.x.abs() < 0.5,
            "an orthographic silhouette stands on the equator: {a:?}"
        );
    }
}

/// The whole reason it cannot be baked at tessellation time.
#[test]
fn the_silhouette_follows_the_camera() {
    let mesh = cylinder();
    let mut camera = side_on(10.0);
    let front = segments(&mesh, ViewPoint::for_instance(&camera, &Mat4::IDENTITY));
    camera.orbit(std::f64::consts::FRAC_PI_2, 0.0);
    let turned = segments(&mesh, ViewPoint::for_instance(&camera, &Mat4::IDENTITY));
    // Much the same amount of outline — the rim's front half is a facet longer or shorter
    // depending on where the tangent falls between two facets — in an entirely new place.
    assert!(
        front.len().abs_diff(turned.len()) <= 2,
        "{} then {}",
        front.len(),
        turned.len()
    );
    assert_ne!(front, turned, "a quarter turn must move the outline");
}

/// Moving the body is as good as moving the camera: the adjacency stays in the mesh's own
/// coordinates and the camera is brought into them.
#[test]
fn the_instance_transform_is_taken_into_account() {
    let mesh = cylinder();
    let camera = side_on(10.0);
    let still = segments(&mesh, ViewPoint::for_instance(&camera, &Mat4::IDENTITY));
    let turned = segments(
        &mesh,
        ViewPoint::for_instance(&camera, &Mat4::from_rotation_z(std::f64::consts::FRAC_PI_2)),
    );
    assert_ne!(still, turned, "turning the body must move its outline");
}

#[test]
fn a_lofted_ball_is_outlined_by_a_ring() {
    let mesh = lofted_ball();
    let sil = segments(&mesh, ViewPoint::Direction(-Vec3::X));
    assert!(
        sil.len() > 16,
        "a ring right round the body, not a few bits"
    );
    let points: Vec<Vec3> = sil.iter().flatten().copied().collect();
    for p in &points {
        // Seen along −x, the ring lies in the plane x = 0, out to within a facet of it.
        assert!(p.x.abs() < 0.2 * R, "outline point off the ring: {p:?}");
        assert!((p.length() - R).abs() < 0.1 * R, "not on the ball: {p:?}");
    }
    let (lowest, highest) = points.iter().fold((f64::MAX, f64::MIN), |(lo, hi), p| {
        (lo.min(p.z), hi.max(p.z))
    });
    assert!(
        lowest < -0.8 * R && highest > 0.8 * R,
        "the ring must go right round, not stop at the equator"
    );
}

/// A blend must not sprout a line where it runs into its neighbour. Seen from a direction
/// that has both the top face and the side face turned towards it, every facet of the
/// blend between them faces the camera too, so there is no sign change anywhere on it —
/// and the kernel draws nothing along a tangent boundary either.
#[test]
fn a_blend_sprouts_no_line_where_it_meets_its_neighbour() {
    let solid = filleted_box();
    let mesh = mesh_of(&solid);
    let view = ViewPoint::Direction(Vec3::new(0.0, 1.0, -1.0).normalize());
    let sil = segments(&mesh, view);
    assert!(!sil.is_empty(), "the body still has an outline");
    assert!(
        !sil.iter()
            .any(|[a, b]| on_a_tangent_line(*a) && on_a_tangent_line(*b)),
        "the silhouette ran along the blend's tangent boundary"
    );
    assert!(
        !solid
            .display_edges()
            .iter()
            .any(|[a, b]| on_a_tangent_line(*a) && on_a_tangent_line(*b)),
        "a feature edge was drawn along the blend's tangent boundary"
    );
}

/// Seen from straight above, though, the same boundary *is* the outline: the side face
/// below it is edge-on and the blend above it is turned towards the camera. A silhouette
/// is a question about the camera, not about the body.
#[test]
fn seen_from_above_the_blend_is_bounded_where_it_runs_out() {
    let mesh = mesh_of(&filleted_box());
    let sil = segments(&mesh, ViewPoint::Direction(-Vec3::Z));
    assert!(
        sil.iter()
            .any(|[a, b]| (a.y.abs() < 1e-6 && (a.z - 8.0).abs() < 1e-6)
                && (b.y.abs() < 1e-6 && (b.z - 8.0).abs() < 1e-6)),
        "from above, the body ends where the blend runs out into the side face"
    );
}

#[test]
fn a_still_camera_reuses_the_last_answer() {
    let mesh = cylinder();
    let silhouette = Silhouette::build(&mesh);
    let mut cache = SilhouetteCache::default();
    let view = ViewPoint::Direction(-Vec3::X);
    let first = cache.segments(&silhouette, view).to_vec();
    assert_eq!(cache.view(), Some(view));
    assert_eq!(cache.segments(&silhouette, view), first.as_slice());
    let moved = ViewPoint::Direction(-Vec3::Y);
    assert_ne!(cache.segments(&silhouette, moved), first.as_slice());
    assert_eq!(cache.view(), Some(moved));
}

/// What the pass costs per frame, at a facet count a real model reaches. Printed with
/// `cargo test -- --nocapture`; the assertion is only a guard against the sign test
/// quietly turning into something that walks the mesh again.
#[test]
fn the_per_frame_cost_is_one_sign_test_per_facet() {
    // A ball at the density a user reaches by turning the tessellation up on a curved
    // body: tens of thousands of facets, curved in both directions so the outline is a
    // long ring rather than two lines. Written out directly — what is being measured is
    // the size of the mesh, not how it was modelled.
    let (rings, around) = (120, 240);
    let point = |i: usize, j: usize| {
        let theta = std::f64::consts::PI * i as f64 / rings as f64;
        let phi = std::f64::consts::TAU * j as f64 / around as f64;
        Vec3::new(
            R * theta.sin() * phi.cos(),
            R * theta.sin() * phi.sin(),
            -R * theta.cos(),
        )
    };
    let mut mesh = TriMesh::default();
    for i in 0..rings {
        for j in 0..around {
            let (a, b, c, d) = (
                point(i, j),
                point(i, j + 1),
                point(i + 1, j + 1),
                point(i + 1, j),
            );
            mesh.push_triangle([a, b, c], 0);
            mesh.push_triangle([a, c, d], 0);
        }
    }
    let triangles = mesh.triangle_count();

    let built = std::time::Instant::now();
    let silhouette = Silhouette::build(&mesh);
    let build = built.elapsed();

    let mut cache = SilhouetteCache::default();
    let frames = 200;
    let moving = std::time::Instant::now();
    for i in 0..frames {
        // A new direction every frame: the orbiting camera, the worst case.
        let a = i as f64 * 0.01;
        let n = cache
            .segments(
                &silhouette,
                ViewPoint::Direction(Vec3::new(a.cos(), a.sin(), -0.3)),
            )
            .len();
        assert!(n > 0);
    }
    let per_frame = moving.elapsed() / frames;

    let still = std::time::Instant::now();
    for _ in 0..frames {
        cache.segments(&silhouette, ViewPoint::Direction(Vec3::new(1.0, 0.0, -0.3)));
    }
    let cached = still.elapsed() / frames;

    println!(
        "{triangles} triangles, {} adjacency records: build {build:?}, \
         {per_frame:?} per moving frame, {cached:?} per still frame",
        silhouette.edge_count()
    );
    assert!(
        per_frame < std::time::Duration::from_millis(10),
        "{per_frame:?} for {triangles} triangles"
    );
}

/// A mesh with no facets to pair has no silhouette, and asking for one is not an error.
#[test]
fn an_empty_mesh_has_no_silhouette() {
    let silhouette = Silhouette::build(&TriMesh::default());
    assert!(silhouette.is_empty());
    assert_eq!(silhouette.edge_count(), 0);
}
