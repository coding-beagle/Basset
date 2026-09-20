//! Documents saved from real sessions, in `testcases/` at the workspace root.
//!
//! Each one is a bug a user hit, kept as the file that reproduced it, because a sketch
//! with forty entities and the constraints a person actually wrote is not something a
//! hand-built test reproduces faithfully.

use approx::assert_relative_eq;
use basset_core::Document;
use basset_math::Vec2;

fn open(name: &str) -> Document {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../testcases")
        .join(name);
    basset_core::file::load(&path).unwrap_or_else(|e| panic!("opening {}: {e}", path.display()))
}

/// An 78 × 8 bar divided into five compartments by four verticals, with a 1 mm hole in each
/// corner of the two end compartments. Parts of the outline were drawn twice — the bar's
/// long edges are each traced again by the compartment rectangles and the end caps — and
/// the duplicated stretches used to stop every region that touched them from being found:
/// the sketch enclosed nothing but the two middle compartments and the four holes.
#[test]
fn a_sketch_with_geometry_drawn_twice_encloses_every_region() {
    let mut doc = open("sketch_geometry_not_fully_enclosed.bass");
    let state = doc.state();
    let sketch = state
        .sketches
        .values()
        .next()
        .expect("the document is one sketch");

    // Five compartments plus the four holes, which are regions of their own as well as
    // holes in the compartments they sit in.
    assert_eq!(sketch.profiles.len(), 9);
    let holes: Vec<_> = sketch.profiles.iter().filter(|p| p.area() < 5.0).collect();
    assert_eq!(holes.len(), 4, "the four 1 mm circles");
    let bar: f64 = sketch
        .profiles
        .iter()
        .filter(|p| p.area() >= 5.0)
        .map(|p| p.area())
        .sum();
    let circles: f64 = holes.iter().map(|p| p.area()).sum();
    assert_relative_eq!(bar + circles, 78.0 * 8.0, epsilon = 0.05);

    // The compartments, named by a point inside each, which is how the editor refers to a
    // region the user clicked.
    // The end compartments are 4 × 8 less the two holes punched out of them; a hole's area
    // is the polygon the circle tessellates to, a little under its true one.
    let end = 32.0 - 2.0 * circles / 4.0;
    for (sample, area) in [
        (Vec2::new(0.0, 0.0), 528.0),
        (Vec2::new(34.0, 0.0), 16.0),
        (Vec2::new(-34.0, 0.0), 16.0),
        (Vec2::new(37.0, 0.0), end),
        (Vec2::new(-37.0, 0.0), end),
    ] {
        let region = sketch
            .profiles
            .iter()
            .filter(|p| p.contains(sample))
            .min_by(|a, b| a.area().total_cmp(&b.area()))
            .unwrap_or_else(|| panic!("no region encloses {sample:?}"));
        assert_relative_eq!(region.area(), area, epsilon = 1e-9);
        assert!(region.outer.signed_area() > 0.0, "traced counter-clockwise");
    }
}

/// The same bar drawn a second way: the compartment verticals were drawn to the bar's
/// long edges but land a couple of 1e-7 short of them, because the solver leaves 26
/// degrees of freedom and nothing pins those endpoints to the edge. The tracer used to
/// see no crossing there at all, merge the compartments either side into one
/// self-overlapping loop, and hand the extrude the wrong profile without a word. Now the
/// near miss is healed into a shared node.
#[test]
fn a_bar_whose_dividers_nearly_touch_still_traces_every_compartment() {
    use basset_core::{FeatureKind, FeatureStatus};

    let mut doc = open("crashes_when_sketch_changes_propagate.bass");
    let fid = *doc.state().sketches.keys().next().expect("one sketch");

    // Nine regions: the bar, four compartments and the four corner holes, through every
    // dimension change that used to break the trace.
    for length in [78.0, 90.0, 60.0] {
        doc.edit_feature_kind(fid, |kind| {
            if let FeatureKind::Sketch { sketch, .. } = kind {
                let (id, _) = sketch
                    .constraints()
                    .find(|(_, c)| matches!(c.dimension_value(), Some(v) if v > 40.0))
                    .expect("the bar's length dimension");
                sketch.set_dimension_value(id, length).unwrap();
            }
        })
        .unwrap();
        let state = doc.state();
        let solved = state.sketches.get(&fid).expect("sketch evaluated");
        assert_eq!(solved.profiles.len(), 9, "at length {length}");
        for p in &solved.profiles {
            assert!(p.outer.signed_area() > 0.0, "traced counter-clockwise");
        }
        // The bar is symmetric about both axes, so its regions come in equal pairs: the
        // four holes, the two end compartments and the two middle ones. Merging two
        // regions into one self-overlapping loop breaks that pairing, and nothing else in
        // the file would have shown it.
        let mut areas: Vec<f64> = solved.profiles.iter().map(|p| p.area()).collect();
        areas.sort_by(|a, b| a.total_cmp(b));
        for (a, b) in [(0, 1), (1, 2), (2, 3), (4, 5), (6, 7)] {
            assert_relative_eq!(areas[a], areas[b], epsilon = 1e-5);
        }
        assert!(areas[8] > areas[7], "the bar encloses the compartments");
        // The sketch is wildly under-constrained, but nothing builds from it in this
        // file, so it is a drawing in progress rather than a fault: no warning.
        assert_eq!(state.status(fid), Some(&FeatureStatus::Ok));
        assert!(solved.report.degrees_of_freedom > 0, "and it is loose");
    }
}

/// A battery holder: a 1 mm base plate, walls joined onto it, a boss, and a pocket cut
/// into the boss's top face. Exported to 3MF it made the slicer report "Error 330
/// non-manifold edges", and the file was faithful — the body really was full of cracks.
///
/// Three things made them, and this file walks through all three, so it checks the body
/// after every feature rather than only at the end:
///
/// * the BSP splitter counted a vertex as lying on a plane only within 1e-7, so a vertex
///   between that and the merge tolerance made its edge "spanning" and the split point
///   invented on that edge landed a hair from the vertex itself;
/// * healing then split neighbouring edges at that near-duplicate, adding a zero-area
///   spur out to a point the merge tolerance calls the edge's own start, and each later
///   boolean interpolated from the mess and drifted further;
/// * tessellation fanned each polygon from its first vertex and dropped the zero-area
///   triangles that fan produces at healed T-junctions, which put holes in the mesh even
///   where the solid underneath was sound.
#[test]
fn a_joined_and_pocketed_body_exports_as_a_closed_manifold_mesh() {
    let mut doc = open("ExportAs3MFCreatesBadGeometry.bass");
    for cursor in 1..=5 {
        doc.set_cursor(cursor);
        for body in doc.state().bodies.values() {
            body.solid
                .validate()
                .unwrap_or_else(|e| panic!("after feature {cursor}, body '{}': {e}", body.name));

            // What the exporter and the slicer see. Every triangle edge must be walked
            // once each way; anything else is the non-manifold edge the user hit.
            let mesh = body.solid.tessellate().mesh;
            let polygon_triangles: usize = body
                .solid
                .faces
                .iter()
                .flat_map(|f| f.polygons.iter())
                .map(|p| p.vertices.len() - 2)
                .sum();
            assert_eq!(
                mesh.triangle_count(),
                polygon_triangles,
                "after feature {cursor}: tessellation dropped a triangle"
            );
            let (unpaired, non_manifold) = bad_mesh_edges(&mesh);
            assert_eq!(
                unpaired, 0,
                "after feature {cursor}, body '{}': the shell leaks",
                body.name
            );
            // An edge shared by four triangles is where two parts of the shell pinch
            // together along a line. It is watertight and still unprintable, and it is
            // the half of "Error 330" that a volume or a closedness check will not see.
            assert_eq!(
                non_manifold, 0,
                "after feature {cursor}, body '{}': edges shared by other than two triangles",
                body.name
            );
            assert!(mesh.signed_volume() > 0.0, "and it is wound outward");
        }
    }
}

/// Counts (edges not paired with an opposite-direction twin, edges not used by exactly
/// two triangles), by welded vertex — which is how a 3MF exporter and a slicer decide
/// that two triangles share an edge.
fn bad_mesh_edges(mesh: &basset_math::TriMesh) -> (usize, usize) {
    use std::collections::HashMap;
    const WELD: f64 = basset_kernel::solid::MERGE_TOL;

    let mut points: Vec<basset_math::Vec3> = Vec::new();
    let mut cells: HashMap<(i64, i64, i64), Vec<u32>> = HashMap::new();
    let mut id = |p: basset_math::Vec3, points: &mut Vec<basset_math::Vec3>| -> u32 {
        let cell = |x: f64| (x / (2.0 * WELD)).floor() as i64;
        let c = (cell(p.x), cell(p.y), cell(p.z));
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    for &i in cells
                        .get(&(c.0 + dx, c.1 + dy, c.2 + dz))
                        .into_iter()
                        .flatten()
                    {
                        if points[i as usize].distance(p) <= WELD {
                            return i;
                        }
                    }
                }
            }
        }
        points.push(p);
        let i = (points.len() - 1) as u32;
        cells.entry(c).or_default().push(i);
        i
    };

    let mut directed: HashMap<(u32, u32), i32> = HashMap::new();
    let mut users: HashMap<(u32, u32), usize> = HashMap::new();
    for t in 0..mesh.triangle_count() {
        let ids: Vec<u32> = mesh
            .triangle(t)
            .iter()
            .map(|v| id(*v, &mut points))
            .collect();
        for i in 0..3 {
            let (a, b) = (ids[i], ids[(i + 1) % 3]);
            if a == b {
                continue;
            }
            let (k, s) = if a < b { ((a, b), 1) } else { ((b, a), -1) };
            *directed.entry(k).or_default() += s;
            *users.entry(k).or_default() += 1;
        }
    }
    (
        directed.values().filter(|c| **c != 0).count(),
        users.values().filter(|n| **n != 2).count(),
    )
}
