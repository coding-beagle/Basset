//! A face's centroid is the centroid of its area, whatever healing and booleans have done
//! to the polygons that make it up.

use approx::assert_relative_eq;
use basset_kernel::Polygon;
use basset_math::Vec3;

/// A unit square with a healed vertex half way along one edge: the vertex average is
/// pulled towards that edge, the area centroid is not.
#[test]
fn a_polygon_centroid_ignores_vertices_healing_inserted_along_an_edge() {
    let square = Polygon::new(vec![
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(0.5, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(1.0, 1.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
    ])
    .unwrap();
    assert_relative_eq!(square.plane.origin.y, 0.4, epsilon = 1e-12);
    let c = square.centroid();
    assert_relative_eq!(c.x, 0.5, epsilon = 1e-12);
    assert_relative_eq!(c.y, 0.5, epsilon = 1e-12);
    assert_relative_eq!(c.z, 0.0, epsilon = 1e-12);
}

/// A reflex fragment, as a boolean leaves behind: an L shape.
#[test]
fn a_reflex_polygon_centroid_is_its_area_centroid() {
    let l = Polygon::new(vec![
        Vec3::new(0.0, 0.0, 2.0),
        Vec3::new(2.0, 0.0, 2.0),
        Vec3::new(2.0, 1.0, 2.0),
        Vec3::new(1.0, 1.0, 2.0),
        Vec3::new(1.0, 2.0, 2.0),
        Vec3::new(0.0, 2.0, 2.0),
    ])
    .unwrap();
    // Two unit-ish rectangles: 2×1 at y∈[0,1] (centroid (1, 0.5)) and 1×1 at y∈[1,2]
    // (centroid (0.5, 1.5)); weighted by area 2 and 1.
    let c = l.centroid();
    assert_relative_eq!(c.x, (2.0 * 1.0 + 1.0 * 0.5) / 3.0, epsilon = 1e-12);
    assert_relative_eq!(c.y, (2.0 * 0.5 + 1.0 * 1.5) / 3.0, epsilon = 1e-12);
    assert_relative_eq!(c.z, 2.0, epsilon = 1e-12);
}
