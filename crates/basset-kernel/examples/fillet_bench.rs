//! Shared fillet benchmark.
//!
//! Every workload is timed on the same machine and validated, so a change to the blend
//! or the boolean can be judged on wall clock without trading away a closed shell or the
//! volume the fillet is supposed to leave behind. Run with
//! `cargo run --release --example fillet_bench`.

use std::time::Instant;

use basset_kernel::geometry::Tessellation;
use basset_kernel::ids::{EdgeKey, FaceKey, FaceRole, OpId};
use basset_kernel::primitives::{cuboid, cylinder};
use basset_kernel::solid::Solid;
use basset_kernel::{BoolOp, boolean, fillet};
use basset_math::Vec3;

fn tess(chord: f64, angle_deg: f64) -> Tessellation {
    Tessellation {
        chord_tolerance: chord,
        max_segment_angle: angle_deg.to_radians(),
    }
}

fn cube() -> Solid {
    cuboid(OpId::new(1), Vec3::ZERO, Vec3::splat(10.0))
}

fn top_edge(i: u32) -> EdgeKey {
    EdgeKey::new(
        FaceKey::new(OpId::new(1), FaceRole::EndCap),
        FaceKey::new(OpId::new(1), FaceRole::Side(i)),
    )
}

fn rim(op: u64, role: FaceRole) -> EdgeKey {
    EdgeKey::new(
        FaceKey::new(OpId::new(op), role),
        FaceKey::new(OpId::new(op), FaceRole::Side(0)),
    )
}

/// An L-block whose inner corner is concave: exercises the material-adding path.
fn l_block() -> Solid {
    let notch = cuboid(
        OpId::new(9),
        Vec3::new(-1.0, -1.0, 5.0),
        Vec3::new(11.0, 5.0, 11.0),
    );
    boolean(&cube(), &notch, BoolOp::Subtract).expect("L-block")
}

struct Case {
    name: &'static str,
    solid: Solid,
    edges: Vec<EdgeKey>,
    radius: f64,
    tess: Tessellation,
}

fn cases() -> Vec<Case> {
    let default = Tessellation::default();
    vec![
        Case {
            name: "cube/1 edge",
            solid: cube(),
            edges: vec![top_edge(0)],
            radius: 2.0,
            tess: tess(0.001, 5.0),
        },
        Case {
            name: "cube/4 top edges",
            solid: cube(),
            edges: (0..4).map(top_edge).collect(),
            radius: 2.0,
            tess: tess(0.001, 5.0),
        },
        Case {
            name: "L-block/concave edge",
            solid: l_block(),
            edges: vec![EdgeKey::new(
                FaceKey::new(OpId::new(9), FaceRole::StartCap),
                FaceKey::new(OpId::new(9), FaceRole::Side(2)),
            )],
            radius: 2.0,
            tess: tess(0.001, 5.0),
        },
        Case {
            name: "cylinder/1 rim (default tess)",
            solid: cylinder(OpId::new(1), Vec3::ZERO, Vec3::Z, 5.0, 10.0, &default),
            edges: vec![rim(1, FaceRole::EndCap)],
            radius: 1.0,
            tess: default,
        },
        Case {
            name: "cylinder/both rims (default tess)",
            solid: cylinder(OpId::new(1), Vec3::ZERO, Vec3::Z, 5.0, 10.0, &default),
            edges: vec![rim(1, FaceRole::EndCap), rim(1, FaceRole::StartCap)],
            radius: 1.0,
            tess: default,
        },
        Case {
            name: "cylinder/both rims (2deg)",
            solid: cylinder(
                OpId::new(1),
                Vec3::ZERO,
                Vec3::Z,
                5.0,
                10.0,
                &tess(0.001, 2.0),
            ),
            edges: vec![rim(1, FaceRole::EndCap), rim(1, FaceRole::StartCap)],
            radius: 1.0,
            tess: tess(0.001, 2.0),
        },
        Case {
            name: "cylinder/both rims (1deg)",
            solid: cylinder(
                OpId::new(1),
                Vec3::ZERO,
                Vec3::Z,
                5.0,
                10.0,
                &tess(0.0005, 1.0),
            ),
            edges: vec![rim(1, FaceRole::EndCap), rim(1, FaceRole::StartCap)],
            radius: 1.0,
            tess: tess(0.0005, 1.0),
        },
    ]
}

fn main() {
    println!(
        "{:<34} {:>7} {:>9} {:>8} {:>7} {:>7}  volume",
        "case", "in", "time ms", "out", "closed", "valid"
    );
    let mut total = 0.0;
    for case in cases() {
        let t = Instant::now();
        let result = fillet(OpId::new(2), &case.solid, &case.edges, case.radius, &case.tess);
        let ms = t.elapsed().as_secs_f64() * 1e3;
        total += ms;
        match result {
            Ok(r) => println!(
                "{:<34} {:>7} {:>9.1} {:>8} {:>7} {:>7}  {:.4}",
                case.name,
                case.solid.polygon_count(),
                ms,
                r.polygon_count(),
                r.is_closed(),
                r.validate().is_ok(),
                r.volume(),
            ),
            Err(e) => println!(
                "{:<34} {:>7} {:>9.1} {:>8} {:>7} {:>7}  {e}",
                case.name,
                case.solid.polygon_count(),
                ms,
                "-",
                "-",
                "-",
            ),
        }
    }
    println!("{:<34} {:>7} {:>9.1}", "TOTAL", "", total);
}
