//! Construction grid, generated on the CPU as ordinary line batches.
//!
//! Generating it as data rather than in a dedicated shader keeps the renderer's pipeline
//! count small and means the grid automatically gets the same pixel-width, depth-tested
//! line rendering as everything else. A few thousand segments per frame is negligible.
//!
//! The grid is drawn on a [`Frame`] rather than always on world XY, so sketch mode can put
//! it on the sketch plane: a grid the user cannot see is a grid they cannot snap to.

use basset_math::{Frame, Vec2};

use crate::camera::Camera;
use crate::scene::LineBatch;

/// Minor lines closer together than this on screen become visual noise, so the spacing
/// steps up by a decade before that happens.
const MIN_MINOR_PIXELS: f64 = 12.0;

/// Snap increments may be finer than the drawn lines: an increment costs nothing visually,
/// and a decade-only snap is uselessly coarse in the middle of a decade (at 10 mm lines,
/// snapping to 10 mm forbids most useful points). Halves and fifths of a decade give the
/// familiar 1 / 2 / 5 / 10 progression.
const MIN_SNAP_PIXELS: f64 = 6.0;

/// How many major cells to draw on each side of the grid centre. Bounded so the segment
/// count stays constant regardless of zoom.
const HALF_EXTENT_MAJOR_CELLS: i64 = 40;

const MINOR_COLOR: [f32; 4] = [0.5, 0.5, 0.5, 0.18];
const MAJOR_COLOR: [f32; 4] = [0.6, 0.6, 0.6, 0.35];
const X_AXIS_COLOR: [f32; 4] = [0.90, 0.25, 0.25, 0.9];
const Y_AXIS_COLOR: [f32; 4] = [0.30, 0.80, 0.30, 0.9];

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GridSpacing {
    /// Distance between minor lines in mm; always a power of ten.
    pub minor: f64,
    /// Distance between major lines: ten minor cells.
    pub major: f64,
}

/// Chooses the coarsest decade whose minor lines are at least [`MIN_MINOR_PIXELS`] apart.
pub fn spacing_for(world_units_per_pixel: f64) -> GridSpacing {
    let min_world = (world_units_per_pixel * MIN_MINOR_PIXELS).max(f64::MIN_POSITIVE);
    let minor = 10f64.powi(min_world.log10().ceil() as i32);
    GridSpacing {
        minor,
        major: minor * 10.0,
    }
}

/// The finest 1 / 2 / 5 × decade increment that is still at least [`MIN_SNAP_PIXELS`] wide
/// on screen. Always divides the drawn minor spacing, so every snapped point lands on a
/// grid line or halfway between two of them, never somewhere the user cannot predict.
pub fn snap_step_for(world_units_per_pixel: f64) -> f64 {
    let min_world = (world_units_per_pixel * MIN_SNAP_PIXELS).max(f64::MIN_POSITIVE);
    let decade = 10f64.powi(min_world.log10().floor() as i32);
    // The candidates are ordered, so the first one wide enough is the finest one.
    [1.0, 2.0, 5.0, 10.0]
        .into_iter()
        .map(|m| m * decade)
        .find(|step| *step >= min_world)
        .unwrap_or(decade * 10.0)
}

/// Snaps a point in frame coordinates to the nearest multiple of `step` on both axes.
pub fn snap_to(p: Vec2, step: f64) -> Vec2 {
    if !(step.is_finite() && step > 0.0) {
        return p;
    }
    Vec2::new((p.x / step).round() * step, (p.y / step).round() * step)
}

/// Builds the grid on `frame` for the current view: minor lines, major lines, then the two
/// in-plane axes so the most important lines draw last.
pub fn build(camera: &Camera, viewport: [u32; 2], frame: &Frame) -> Vec<LineBatch> {
    let spacing = spacing_for(camera.pixel_size_at(camera.target, viewport));
    let GridSpacing { minor, major } = spacing;

    // Snap the centre to a major cell so the grid does not slide under the model as the
    // user pans; the lines only ever appear or disappear at the far edges. The centre is
    // the camera target projected onto the grid plane, so the grid follows the view even
    // when the target is off the plane.
    let centre = frame.to_local(camera.target);
    let centre_x = (centre.x / major).round() * major;
    let centre_y = (centre.y / major).round() * major;
    let half_extent = major * HALF_EXTENT_MAJOR_CELLS as f64;
    let (x0, x1) = (centre_x - half_extent, centre_x + half_extent);
    let (y0, y1) = (centre_y - half_extent, centre_y + half_extent);
    let at = |x: f64, y: f64| frame.to_world(Vec2::new(x, y));

    let mut minor_batch = LineBatch {
        width_px: 1.0,
        ..LineBatch::new(MINOR_COLOR)
    };
    let mut major_batch = LineBatch {
        width_px: 1.0,
        ..LineBatch::new(MAJOR_COLOR)
    };

    let steps = HALF_EXTENT_MAJOR_CELLS * 10;
    for i in -steps..=steps {
        let is_major = i % 10 == 0;
        let x = centre_x + i as f64 * minor;
        let y = centre_y + i as f64 * minor;
        // The axes are drawn separately in colour, so leave a gap for them.
        let on_x_axis = y.abs() < minor * 0.5;
        let on_y_axis = x.abs() < minor * 0.5;
        let batch = if is_major {
            &mut major_batch
        } else {
            &mut minor_batch
        };
        if !on_y_axis {
            batch.segments.push([at(x, y0), at(x, y1)]);
        }
        if !on_x_axis {
            batch.segments.push([at(x0, y), at(x1, y)]);
        }
    }

    let mut batches = vec![minor_batch, major_batch];
    if (y0..=y1).contains(&0.0) {
        let mut x_axis = LineBatch {
            width_px: 1.5,
            ..LineBatch::new(X_AXIS_COLOR)
        };
        x_axis.segments.push([at(x0, 0.0), at(x1, 0.0)]);
        batches.push(x_axis);
    }
    if (x0..=x1).contains(&0.0) {
        let mut y_axis = LineBatch {
            width_px: 1.5,
            ..LineBatch::new(Y_AXIS_COLOR)
        };
        y_axis.segments.push([at(0.0, y0), at(0.0, y1)]);
        batches.push(y_axis);
    }
    batches
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use basset_math::Vec3;

    #[test]
    fn spacing_steps_by_decades() {
        // 0.5 mm per pixel needs 6 mm between minor lines: next decade up is 10 mm.
        let s = spacing_for(0.5);
        assert_relative_eq!(s.minor, 10.0);
        assert_relative_eq!(s.major, 100.0);
        // 0.05 mm per pixel needs 0.6 mm: 1 mm.
        assert_relative_eq!(spacing_for(0.05).minor, 1.0);
        // Exactly on a decade boundary stays there.
        assert_relative_eq!(spacing_for(1.0 / MIN_MINOR_PIXELS).minor, 1.0);
        assert_relative_eq!(spacing_for(0.0004).minor, 0.01);
    }

    #[test]
    fn spacing_never_gets_denser_than_minimum() {
        for exp in -6..6 {
            let wpp = 10f64.powi(exp) * 3.7;
            let s = spacing_for(wpp);
            assert!(s.minor / wpp >= MIN_MINOR_PIXELS - 1e-9);
            assert!(s.minor / wpp < MIN_MINOR_PIXELS * 10.0);
        }
    }

    #[test]
    fn snap_step_follows_the_one_two_five_progression() {
        // 0.1 mm per pixel needs 0.6 mm: the 1 mm step is the finest that fits.
        assert_relative_eq!(snap_step_for(0.1), 1.0);
        // 0.2 mm per pixel needs 1.2 mm: 2 mm.
        assert_relative_eq!(snap_step_for(0.2), 2.0);
        // 0.5 mm per pixel needs 3 mm: 5 mm.
        assert_relative_eq!(snap_step_for(0.5), 5.0);
        // 1 mm per pixel needs 6 mm: 10 mm.
        assert_relative_eq!(snap_step_for(1.0), 10.0);
        assert_relative_eq!(snap_step_for(0.001), 0.01);
    }

    #[test]
    fn snap_step_is_wide_enough_and_divides_the_drawn_spacing() {
        for exp in -6..6 {
            let wpp = 10f64.powi(exp) * 3.7;
            let step = snap_step_for(wpp);
            assert!(step / wpp >= MIN_SNAP_PIXELS - 1e-9, "step {step} at {wpp}");
            let cells = spacing_for(wpp).minor / step;
            assert_relative_eq!(cells, cells.round(), epsilon = 1e-9);
        }
    }

    #[test]
    fn snap_to_rounds_to_the_nearest_multiple() {
        assert_eq!(snap_to(Vec2::new(4.9, -1.2), 1.0), Vec2::new(5.0, -1.0));
        assert_eq!(snap_to(Vec2::new(12.0, 8.0), 5.0), Vec2::new(10.0, 10.0));
        // A meaningless step leaves the point alone rather than producing NaN.
        let p = Vec2::new(1.5, 2.5);
        assert_eq!(snap_to(p, 0.0), p);
        assert_eq!(snap_to(p, f64::NAN), p);
    }

    #[test]
    fn build_contains_axes_and_lies_on_xy_plane() {
        let camera = Camera::new_default();
        let batches = build(&camera, [800, 600], &Frame::XY);
        assert_eq!(batches.len(), 4, "minor, major, x axis, y axis");
        for batch in &batches {
            assert!(!batch.segments.is_empty());
            assert!(batch.depth_test);
            for [a, b] in &batch.segments {
                assert_eq!(a.z, 0.0);
                assert_eq!(b.z, 0.0);
            }
        }
        let x_axis = &batches[2];
        assert_eq!(x_axis.segments.len(), 1);
        assert_eq!(x_axis.segments[0][0].y, 0.0);
        assert_eq!(x_axis.color, X_AXIS_COLOR);
        let y_axis = &batches[3];
        assert_eq!(y_axis.segments[0][0].x, 0.0);
    }

    #[test]
    fn build_omits_axes_when_far_away() {
        let mut camera = Camera::new_default();
        camera.target = Vec3::new(1e6, 1e6, 0.0);
        let batches = build(&camera, [800, 600], &Frame::XY);
        assert_eq!(batches.len(), 2);
    }

    #[test]
    fn build_lies_in_the_given_frame() {
        let camera = Camera::new_default();
        let frame = Frame::from_normal(Vec3::new(0.0, 0.0, 7.0), Vec3::Z);
        for [a, b] in build(&camera, [800, 600], &frame)
            .iter()
            .flat_map(|batch| &batch.segments)
        {
            assert_relative_eq!(a.z, 7.0, epsilon = 1e-9);
            assert_relative_eq!(b.z, 7.0, epsilon = 1e-9);
        }
        // An off-plane camera target still centres the grid under the view.
        let mut camera = Camera::new_default();
        camera.target = Vec3::new(500.0, 0.0, 300.0);
        let batches = build(&camera, [800, 600], &Frame::XY);
        let max_x = batches
            .iter()
            .flat_map(|b| &b.segments)
            .flat_map(|s| [s[0].x, s[1].x])
            .fold(f64::NEG_INFINITY, f64::max);
        assert!(max_x > 500.0);
    }
}
