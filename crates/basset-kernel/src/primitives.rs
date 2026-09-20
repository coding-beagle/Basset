//! Primitive solids, built through the same generators users reach through sketches so
//! that tests and tools exercise exactly the production code path.

use basset_math::{Frame, Vec2, Vec3};

use crate::error::KernelError;
use crate::geometry::{Contour, Extent, Profile, Tessellation};
use crate::ids::OpId;
use crate::solid::Solid;

/// Axis-aligned box between `min` and `max`. Side faces are `Side(0..4)` counter-clockwise
/// from the −y face; `StartCap` is the bottom.
pub fn cuboid(op: OpId, min: Vec3, max: Vec3) -> Solid {
    let size = max - min;
    let mut outer = Contour::polygon(
        vec![
            Vec2::ZERO,
            Vec2::new(size.x, 0.0),
            Vec2::new(size.x, size.y),
            Vec2::new(0.0, size.y),
        ],
        0,
    );
    for (i, s) in outer.segments.iter_mut().enumerate() {
        s.curve = i as u32;
    }
    let frame = Frame {
        origin: min,
        ..Frame::XY
    };
    crate::generate::extrude(op, &Profile::new(frame, outer), Extent::OneSide(size.z))
        .expect("cuboid inputs are non-degenerate by construction")
}

/// Cylinder standing on `base`, along `axis`. The curved face is `Side(0)`.
pub fn cylinder(
    op: OpId,
    base: Vec3,
    axis: Vec3,
    radius: f64,
    height: f64,
    tess: &Tessellation,
) -> Solid {
    try_cylinder(op, base, axis, radius, height, tess).expect("cylinder inputs are non-degenerate")
}

pub fn try_cylinder(
    op: OpId,
    base: Vec3,
    axis: Vec3,
    radius: f64,
    height: f64,
    tess: &Tessellation,
) -> Result<Solid, KernelError> {
    let frame = Frame::from_normal(base, axis);
    let circle = Contour::circle(Vec2::ZERO, radius, 0, tess);
    crate::generate::extrude(op, &Profile::new(frame, circle), Extent::OneSide(height))
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use std::f64::consts::PI;

    #[test]
    fn cuboid_dimensions() {
        let c = cuboid(
            OpId::new(1),
            Vec3::new(1.0, 2.0, 3.0),
            Vec3::new(3.0, 5.0, 7.0),
        );
        assert_relative_eq!(c.volume(), 2.0 * 3.0 * 4.0);
        assert_eq!(c.aabb().min, Vec3::new(1.0, 2.0, 3.0));
        assert_eq!(c.faces.len(), 6);
    }

    #[test]
    fn cylinder_volume() {
        let t = Tessellation {
            chord_tolerance: 1e-5,
            ..Default::default()
        };
        let c = cylinder(OpId::new(1), Vec3::ZERO, Vec3::X, 2.0, 5.0, &t);
        assert_relative_eq!(c.volume(), PI * 4.0 * 5.0, epsilon = 0.01);
        assert_relative_eq!(c.aabb().max.x, 5.0);
        assert!(c.is_closed());
    }
}
