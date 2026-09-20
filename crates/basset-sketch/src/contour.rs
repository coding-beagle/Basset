//! Tessellated closed regions handed to the kernel.
//!
//! A [`Contour`] is a polyline whose every edge remembers which sketch curve produced it,
//! so the kernel can group the many small edges of an arc into one cylindrical face with
//! a stable identity. A [`Profile`] is one outer contour plus its holes.

use basset_math::Vec2;

use crate::EntityId;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SegmentKind {
    Line,
    /// Part of a circle; `ccw` is the direction this edge travels around `center`.
    Arc {
        center: Vec2,
        radius: f64,
        ccw: bool,
    },
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Segment {
    pub curve: EntityId,
    pub kind: SegmentKind,
}

/// Polyline. `segments[i]` describes the edge `points[i] → points[(i + 1) % n]` for a
/// closed contour, or `points[i] → points[i + 1]` for an open one.
#[derive(Debug, Clone, PartialEq)]
pub struct Contour {
    pub points: Vec<Vec2>,
    pub segments: Vec<Segment>,
    pub closed: bool,
}

impl Contour {
    /// Shoelace area: positive for counter-clockwise. Open contours are treated as if
    /// closed by the chord from the last to the first point.
    pub fn signed_area(&self) -> f64 {
        signed_area(&self.points)
    }

    /// Even-odd point-in-polygon test on the polyline.
    pub fn contains(&self, p: Vec2) -> bool {
        polygon_contains(&self.points, p)
    }

    /// Flips the traversal direction, keeping segment tags attached to the same edges.
    pub fn reversed(&self) -> Contour {
        let n = self.points.len();
        let mut points = self.points.clone();
        points.reverse();
        let mut segments = Vec::with_capacity(self.segments.len());
        // Edge i (points[i] → points[i+1]) becomes the edge starting at reversed index n-2-i
        // for open contours; for closed ones edge i connects points[i] → points[i+1 mod n]
        // and after reversal starts at index n-1-(i+1 mod n).
        if self.closed {
            let mut ordered = vec![None; self.segments.len()];
            for (i, seg) in self.segments.iter().enumerate() {
                let j = (n - 1 - ((i + 1) % n)) % n;
                ordered[j] = Some(flip(*seg));
            }
            segments.extend(ordered.into_iter().flatten());
        } else {
            for seg in self.segments.iter().rev() {
                segments.push(flip(*seg));
            }
        }
        Contour {
            points,
            segments,
            closed: self.closed,
        }
    }

    /// Length of the polyline.
    pub fn length(&self) -> f64 {
        let n = self.points.len();
        let edges = if self.closed { n } else { n.saturating_sub(1) };
        (0..edges)
            .map(|i| self.points[i].distance(self.points[(i + 1) % n]))
            .sum()
    }
}

fn flip(seg: Segment) -> Segment {
    match seg.kind {
        SegmentKind::Line => seg,
        SegmentKind::Arc {
            center,
            radius,
            ccw,
        } => Segment {
            curve: seg.curve,
            kind: SegmentKind::Arc {
                center,
                radius,
                ccw: !ccw,
            },
        },
    }
}

/// Closed region: `outer` is counter-clockwise, every hole is clockwise.
#[derive(Debug, Clone, PartialEq)]
pub struct Profile {
    pub outer: Contour,
    pub holes: Vec<Contour>,
}

impl Profile {
    /// Enclosed area (outer minus holes), always non-negative for a well-formed profile.
    pub fn area(&self) -> f64 {
        self.outer.signed_area().abs()
            - self
                .holes
                .iter()
                .map(|h| h.signed_area().abs())
                .sum::<f64>()
    }

    pub fn contains(&self, p: Vec2) -> bool {
        self.outer.contains(p) && !self.holes.iter().any(|h| h.contains(p))
    }

    /// A point strictly inside the region, for naming it in a reference that has to
    /// survive re-dimensioning. The centroid is used when it lies inside; for a crescent
    /// or a region whose centroid falls in a hole, the widest gap of a horizontal cut
    /// through the profile is taken instead, which is inside by construction.
    pub fn interior_point(&self) -> Option<Vec2> {
        let points = &self.outer.points;
        if points.len() < 3 {
            return None;
        }
        let centroid = points.iter().fold(Vec2::ZERO, |a, p| a + *p) / points.len() as f64;
        if self.contains(centroid) {
            return Some(centroid);
        }
        let (mut low, mut high) = (f64::INFINITY, f64::NEG_INFINITY);
        for p in points {
            low = low.min(p.y);
            high = high.max(p.y);
        }
        // Sample a few heights rather than one: a single cut can miss a thin region
        // whose only wide part is near one end.
        (1..8)
            .map(|i| low + (high - low) * i as f64 / 8.0)
            .filter_map(|y| self.widest_gap_at(y))
            .max_by(|a, b| a.1.total_cmp(&b.1))
            .map(|(p, _)| p)
    }

    /// Midpoint and width of the widest stretch of the line `y` that lies inside the
    /// profile, if any.
    fn widest_gap_at(&self, y: f64) -> Option<(Vec2, f64)> {
        let mut xs: Vec<f64> = Vec::new();
        for contour in std::iter::once(&self.outer).chain(self.holes.iter()) {
            let pts = &contour.points;
            let n = pts.len();
            for i in 0..n {
                let (a, b) = (pts[i], pts[(i + 1) % n]);
                if (a.y > y) != (b.y > y) {
                    xs.push(a.x + (y - a.y) / (b.y - a.y) * (b.x - a.x));
                }
            }
        }
        xs.sort_by(f64::total_cmp);
        // Crossings pair up into inside stretches: the first is entered at xs[0], left
        // at xs[1], and so on.
        xs.as_chunks::<2>()
            .0
            .iter()
            .map(|w| (Vec2::new((w[0] + w[1]) * 0.5, y), w[1] - w[0]))
            .filter(|(_, width)| *width > 0.0)
            .max_by(|a, b| a.1.total_cmp(&b.1))
    }
}

pub fn signed_area(points: &[Vec2]) -> f64 {
    let n = points.len();
    if n < 3 {
        return 0.0;
    }
    let mut a = 0.0;
    for i in 0..n {
        let p = points[i];
        let q = points[(i + 1) % n];
        a += p.x * q.y - q.x * p.y;
    }
    a * 0.5
}

/// Standard even-odd crossing test. Points exactly on the boundary are unspecified.
pub fn polygon_contains(points: &[Vec2], p: Vec2) -> bool {
    let n = points.len();
    if n < 3 {
        return false;
    }
    let mut inside = false;
    let mut j = n - 1;
    for i in 0..n {
        let (a, b) = (points[i], points[j]);
        if (a.y > p.y) != (b.y > p.y) {
            let x = a.x + (p.y - a.y) / (b.y - a.y) * (b.x - a.x);
            if p.x < x {
                inside = !inside;
            }
        }
        j = i;
    }
    inside
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use slotmap::Key;

    fn square() -> Contour {
        let pts = vec![
            Vec2::ZERO,
            Vec2::new(2.0, 0.0),
            Vec2::new(2.0, 2.0),
            Vec2::new(0.0, 2.0),
        ];
        let segs = (0..4)
            .map(|_| Segment {
                curve: EntityId::null(),
                kind: SegmentKind::Line,
            })
            .collect();
        Contour {
            points: pts,
            segments: segs,
            closed: true,
        }
    }

    #[test]
    fn area_and_containment() {
        let c = square();
        assert_relative_eq!(c.signed_area(), 4.0);
        assert!(c.contains(Vec2::new(1.0, 1.0)));
        assert!(!c.contains(Vec2::new(3.0, 1.0)));
        let r = c.reversed();
        assert_relative_eq!(r.signed_area(), -4.0);
        assert_eq!(r.segments.len(), 4);
    }

    #[test]
    fn reversed_keeps_arc_tags_on_their_edges() {
        let id = EntityId::null();
        let arc = |ccw| Segment {
            curve: id,
            kind: SegmentKind::Arc {
                center: Vec2::ZERO,
                radius: 1.0,
                ccw,
            },
        };
        let line = Segment {
            curve: id,
            kind: SegmentKind::Line,
        };
        // Closed triangle: edge0 arc, edge1 line, edge2 line.
        let c = Contour {
            points: vec![Vec2::ZERO, Vec2::X, Vec2::Y],
            segments: vec![arc(true), line, line],
            closed: true,
        };
        let r = c.reversed();
        // Reversed points: Y, X, ZERO. Edge X→ZERO is index 1 and must be the arc, now CW.
        assert_eq!(r.points, vec![Vec2::Y, Vec2::X, Vec2::ZERO]);
        assert_eq!(r.segments[1], arc(false));
        // Open case: single arc edge reverses to a single CW arc edge.
        let o = Contour {
            points: vec![Vec2::ZERO, Vec2::X],
            segments: vec![arc(true)],
            closed: false,
        };
        assert_eq!(o.reversed().segments, vec![arc(false)]);
    }

    #[test]
    fn profile_area_subtracts_holes() {
        let outer = square();
        let mut hole = square();
        for p in &mut hole.points {
            *p = *p * 0.5 + Vec2::splat(0.5);
        }
        let hole = hole.reversed();
        let prof = Profile {
            outer,
            holes: vec![hole],
        };
        assert_relative_eq!(prof.area(), 3.0);
        assert!(prof.contains(Vec2::new(0.25, 0.25)));
        assert!(!prof.contains(Vec2::new(1.0, 1.0)));
    }
}
