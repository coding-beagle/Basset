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
