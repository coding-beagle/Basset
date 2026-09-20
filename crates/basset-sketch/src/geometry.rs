//! Exact 2D distance and intersection helpers used by hit testing and selection.

use basset_math::Vec2;

use crate::tessellation::{angle_within, ccw_sweep};

/// Distance from `p` to the segment `ab`.
pub fn point_segment_distance(p: Vec2, a: Vec2, b: Vec2) -> f64 {
    let d = b - a;
    let len2 = d.length_squared();
    if len2 == 0.0 {
        return p.distance(a);
    }
    let t = ((p - a).dot(d) / len2).clamp(0.0, 1.0);
    p.distance(a + d * t)
}

/// Distance from `p` to the full circle.
pub fn point_circle_distance(p: Vec2, center: Vec2, radius: f64) -> f64 {
    (p.distance(center) - radius).abs()
}

/// Distance from `p` to the CCW arc from `start` to `end` about `center`. Inside the
/// arc's angular span the nearest point is radial; outside it is one of the endpoints.
pub fn point_arc_distance(p: Vec2, center: Vec2, start: Vec2, end: Vec2) -> f64 {
    let radius = start.distance(center);
    let a0 = (start - center).to_angle();
    let sweep = ccw_sweep(a0, (end - center).to_angle());
    let theta = (p - center).to_angle();
    if angle_within(theta, a0, sweep) {
        (p.distance(center) - radius).abs()
    } else {
        p.distance(start).min(p.distance(end))
    }
}

/// Axis-aligned bounds of a CCW arc: its endpoints plus any axis-extreme point it crosses.
pub fn arc_bounds(center: Vec2, start: Vec2, end: Vec2) -> (Vec2, Vec2) {
    let radius = start.distance(center);
    let a0 = (start - center).to_angle();
    let sweep = ccw_sweep(a0, (end - center).to_angle());
    let mut min = start.min(end);
    let mut max = start.max(end);
    for k in 0..4 {
        let angle = k as f64 * std::f64::consts::FRAC_PI_2;
        if angle_within(angle, a0, sweep) {
            let p = center + Vec2::from_angle(angle) * radius;
            min = min.min(p);
            max = max.max(p);
        }
    }
    (min, max)
}

pub fn point_in_rect(p: Vec2, min: Vec2, max: Vec2) -> bool {
    p.x >= min.x && p.x <= max.x && p.y >= min.y && p.y <= max.y
}

/// Whether segment `ab` touches the rectangle (Liang–Barsky clipping).
pub fn segment_intersects_rect(a: Vec2, b: Vec2, min: Vec2, max: Vec2) -> bool {
    let d = b - a;
    let mut t0 = 0.0f64;
    let mut t1 = 1.0f64;
    let checks = [
        (-d.x, a.x - min.x),
        (d.x, max.x - a.x),
        (-d.y, a.y - min.y),
        (d.y, max.y - a.y),
    ];
    for (p, q) in checks {
        if p == 0.0 {
            if q < 0.0 {
                return false;
            }
        } else {
            let t = q / p;
            if p < 0.0 {
                t0 = t0.max(t);
            } else {
                t1 = t1.min(t);
            }
            if t0 > t1 {
                return false;
            }
        }
    }
    true
}

/// Whether any edge of the polyline touches the rectangle.
pub fn polyline_intersects_rect(points: &[Vec2], closed: bool, min: Vec2, max: Vec2) -> bool {
    let n = points.len();
    if n == 1 {
        return point_in_rect(points[0], min, max);
    }
    let edges = if closed { n } else { n.saturating_sub(1) };
    (0..edges).any(|i| segment_intersects_rect(points[i], points[(i + 1) % n], min, max))
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn segment_distance() {
        let (a, b) = (Vec2::ZERO, Vec2::new(10.0, 0.0));
        assert_relative_eq!(point_segment_distance(Vec2::new(5.0, 3.0), a, b), 3.0);
        assert_relative_eq!(point_segment_distance(Vec2::new(-4.0, 3.0), a, b), 5.0);
        assert_relative_eq!(
            point_segment_distance(Vec2::new(1.0, 1.0), a, a),
            2f64.sqrt()
        );
    }

    #[test]
    fn arc_distance_inside_and_outside_span() {
        // Quarter arc from (1,0) to (0,1) about origin.
        let (c, s, e) = (Vec2::ZERO, Vec2::X, Vec2::Y);
        assert_relative_eq!(
            point_arc_distance(Vec2::new(2.0, 2.0), c, s, e),
            8f64.sqrt() - 1.0
        );
        // Below the x axis: nearest is the start point.
        assert_relative_eq!(point_arc_distance(Vec2::new(1.0, -1.0), c, s, e), 1.0);
    }

    #[test]
    fn arc_bounds_include_axis_extremes() {
        // Three-quarter arc from (0,-1) CCW to (-1,0) passes through (1,0) and (0,1).
        let (min, max) = arc_bounds(Vec2::ZERO, -Vec2::Y, -Vec2::X);
        assert_relative_eq!(min.x, -1.0);
        assert_relative_eq!(min.y, -1.0);
        assert_relative_eq!(max.x, 1.0);
        assert_relative_eq!(max.y, 1.0);
    }

    #[test]
    fn rect_clipping() {
        let (min, max) = (Vec2::ZERO, Vec2::splat(1.0));
        assert!(segment_intersects_rect(
            Vec2::new(-1.0, 0.5),
            Vec2::new(2.0, 0.5),
            min,
            max
        ));
        assert!(!segment_intersects_rect(
            Vec2::new(-1.0, 2.0),
            Vec2::new(2.0, 1.5),
            min,
            max
        ));
        // Diagonal that misses the box although its bounding box overlaps it.
        assert!(!segment_intersects_rect(
            Vec2::new(0.9, 2.0),
            Vec2::new(2.0, 0.9),
            min,
            max
        ));
    }
}
