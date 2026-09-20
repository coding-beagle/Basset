//! Flattening of circular geometry into polylines.
//!
//! The segment count is driven by two limits: a chord (sagitta) tolerance so that curved
//! edges stay within a distance of the true arc, and a maximum angle per segment so that
//! tiny arcs still get a rounded appearance instead of collapsing to one chord.

use std::f64::consts::TAU;

use basset_math::{ANGULAR_TOL, Vec2};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Tessellation {
    /// Maximum distance between the polyline and the true arc, in mm.
    pub chord_tolerance: f64,
    /// Maximum angle swept by one segment, in radians.
    pub max_segment_angle: f64,
}

impl Default for Tessellation {
    fn default() -> Self {
        Self {
            chord_tolerance: 0.01,
            max_segment_angle: 10f64.to_radians(),
        }
    }
}

impl Tessellation {
    /// Number of segments for an arc of the given radius sweeping `sweep` radians.
    pub fn segment_count(&self, radius: f64, sweep: f64) -> usize {
        let sweep = sweep.abs();
        let by_angle = (sweep / self.max_segment_angle.max(1e-6)).ceil();
        // Sagitta of a chord spanning angle α is r(1 − cos(α/2)); invert for the tolerance.
        let by_chord = if radius > self.chord_tolerance {
            let alpha = 2.0 * (1.0 - self.chord_tolerance / radius).acos();
            (sweep / alpha.max(1e-6)).ceil()
        } else {
            1.0
        };
        by_angle.max(by_chord).max(1.0) as usize
    }
}

/// Counter-clockwise sweep from `start_angle` to `end_angle` in `(0, 2π]`. Coincident
/// endpoints are treated as a full circle: an arc that sweeps nothing is meaningless, and
/// Fusion also treats a closed arc as a circle.
pub fn ccw_sweep(start_angle: f64, end_angle: f64) -> f64 {
    let mut sweep = (end_angle - start_angle).rem_euclid(TAU);
    if sweep < ANGULAR_TOL {
        sweep = TAU;
    }
    sweep
}

/// Whether angle `theta` lies within the CCW arc beginning at `start_angle` sweeping `sweep`.
pub fn angle_within(theta: f64, start_angle: f64, sweep: f64) -> bool {
    (theta - start_angle).rem_euclid(TAU) <= sweep + ANGULAR_TOL
}

/// Polyline for a CCW arc. The first and last points are exactly `start` and `end` so
/// that consecutive curves in a profile meet without gaps.
pub fn arc_polyline(center: Vec2, start: Vec2, end: Vec2, tess: &Tessellation) -> Vec<Vec2> {
    let radius = (start - center).length();
    let a0 = (start - center).to_angle();
    let a1 = (end - center).to_angle();
    let sweep = ccw_sweep(a0, a1);
    let n = tess.segment_count(radius, sweep);
    let mut pts = Vec::with_capacity(n + 1);
    pts.push(start);
    for i in 1..n {
        let a = a0 + sweep * i as f64 / n as f64;
        pts.push(center + Vec2::from_angle(a) * radius);
    }
    pts.push(end);
    pts
}

/// Closed polyline for a full circle: `n` distinct points, no repetition of the first.
/// Starts at angle 0 so tessellations are deterministic across solves.
pub fn circle_polyline(center: Vec2, radius: f64, tess: &Tessellation) -> Vec<Vec2> {
    let n = tess.segment_count(radius, TAU).max(3);
    (0..n)
        .map(|i| center + Vec2::from_angle(TAU * i as f64 / n as f64) * radius)
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use std::f64::consts::PI;

    #[test]
    fn arc_polyline_hits_endpoints_exactly() {
        let c = Vec2::new(1.0, 1.0);
        let s = Vec2::new(2.0, 1.0);
        let e = Vec2::new(1.0, 2.0);
        let pts = arc_polyline(c, s, e, &Tessellation::default());
        assert_eq!(pts[0], s);
        assert_eq!(*pts.last().unwrap(), e);
        assert!(
            pts.len() >= 10,
            "quarter arc at 10° per segment needs ≥9 segments"
        );
        for p in &pts {
            assert_relative_eq!((*p - c).length(), 1.0, epsilon = 1e-12);
        }
    }

    #[test]
    fn segment_count_respects_chord_tolerance() {
        let t = Tessellation {
            chord_tolerance: 0.001,
            max_segment_angle: PI,
        };
        // Large radius: chord tolerance dominates.
        assert!(t.segment_count(100.0, TAU) > 50);
        // Tiny radius: angle limit dominates.
        assert_eq!(t.segment_count(0.0001, TAU), 2);
    }

    #[test]
    fn sweep_conventions() {
        assert_relative_eq!(ccw_sweep(0.0, PI / 2.0), PI / 2.0);
        assert_relative_eq!(ccw_sweep(PI / 2.0, 0.0), 1.5 * PI);
        assert_relative_eq!(ccw_sweep(1.0, 1.0), TAU);
        assert!(angle_within(0.5, 0.0, 1.0));
        assert!(!angle_within(-0.5, 0.0, 1.0));
    }
}
