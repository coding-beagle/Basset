//! Inputs to the modelling operations: planar profiles, paths, extents and axes.
//!
//! Profiles arrive already flattened into polylines. Each polyline edge carries a
//! [`Segment`] naming the sketch curve it came from, which is what lets the kernel give
//! every lateral face a key that survives re-tessellation, and lets it record that the
//! face is cylindrical rather than a bundle of unrelated quads.

use std::f64::consts::TAU;

use basset_math::{Frame, Vec2, Vec3};

use crate::error::KernelError;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SegmentKind {
    Line,
    /// Part of a circle in profile coordinates; `ccw` is the direction of travel.
    Arc {
        center: Vec2,
        radius: f64,
        ccw: bool,
    },
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Segment {
    /// Stable tag of the sketch curve that produced this edge.
    pub curve: u32,
    pub kind: SegmentKind,
}

impl Segment {
    pub const fn line(curve: u32) -> Self {
        Self {
            curve,
            kind: SegmentKind::Line,
        }
    }
}

/// Polyline in profile coordinates. `segments[i]` describes `points[i] → points[i + 1]`
/// (wrapping for closed contours).
#[derive(Debug, Clone, PartialEq)]
pub struct Contour {
    pub points: Vec<Vec2>,
    pub segments: Vec<Segment>,
    pub closed: bool,
}

impl Contour {
    /// Closed contour with every edge tagged as a line of the same curve.
    pub fn polygon(points: Vec<Vec2>, curve: u32) -> Self {
        let segments = points.iter().map(|_| Segment::line(curve)).collect();
        Self {
            points,
            segments,
            closed: true,
        }
    }

    /// Closed circle approximated to `tess`, every edge tagged as an arc of `curve`.
    pub fn circle(center: Vec2, radius: f64, curve: u32, tess: &Tessellation) -> Self {
        let n = tess.segment_count(radius, TAU).max(3);
        let points = (0..n)
            .map(|i| center + Vec2::from_angle(TAU * i as f64 / n as f64) * radius)
            .collect::<Vec<_>>();
        let seg = Segment {
            curve,
            kind: SegmentKind::Arc {
                center,
                radius,
                ccw: true,
            },
        };
        Self {
            segments: vec![seg; n],
            points,
            closed: true,
        }
    }

    /// Shoelace area; positive for counter-clockwise.
    pub fn signed_area(&self) -> f64 {
        signed_area(&self.points)
    }

    pub fn contains(&self, p: Vec2) -> bool {
        polygon_contains(&self.points, p)
    }

    /// Reverses direction while keeping every segment attached to the same edge.
    pub fn reversed(&self) -> Contour {
        let n = self.points.len();
        let mut points = self.points.clone();
        points.reverse();
        let mut segments = vec![None; self.segments.len()];
        for (i, seg) in self.segments.iter().enumerate() {
            // Edge i joins points[i] and points[i+1]; after reversal that pair sits at
            // reversed index n-1-(i+1), which for closed contours wraps around.
            let j = if self.closed {
                (n - 1 - ((i + 1) % n)) % n
            } else {
                n - 2 - i
            };
            segments[j] = Some(flip(*seg));
        }
        Contour {
            points,
            segments: segments.into_iter().flatten().collect(),
            closed: self.closed,
        }
    }

    pub fn edge_count(&self) -> usize {
        if self.closed {
            self.points.len()
        } else {
            self.points.len().saturating_sub(1)
        }
    }

    fn validate(&self) -> Result<(), KernelError> {
        if self.points.len() < 3 || self.segments.len() != self.edge_count() {
            return Err(KernelError::DegenerateContour);
        }
        if self.signed_area().abs() < 1e-12 {
            return Err(KernelError::EmptyProfile);
        }
        Ok(())
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

/// A closed planar region placed in space by `frame`.
#[derive(Debug, Clone, PartialEq)]
pub struct Profile {
    pub frame: Frame,
    pub outer: Contour,
    pub holes: Vec<Contour>,
}

impl Profile {
    pub fn new(frame: Frame, outer: Contour) -> Self {
        Self {
            frame,
            outer,
            holes: Vec::new(),
        }
    }

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

    /// Outer counter-clockwise, holes clockwise: the orientation every generator assumes
    /// so that lateral faces come out facing outward. Also rejects degenerate input.
    pub fn normalised(&self) -> Result<Profile, KernelError> {
        self.outer.validate()?;
        let outer = if self.outer.signed_area() < 0.0 {
            self.outer.reversed()
        } else {
            self.outer.clone()
        };
        let mut holes = Vec::with_capacity(self.holes.len());
        for h in &self.holes {
            h.validate()?;
            holes.push(if h.signed_area() > 0.0 {
                h.reversed()
            } else {
                h.clone()
            });
        }
        Ok(Profile {
            frame: self.frame,
            outer,
            holes,
        })
    }

    /// All loops, outer first.
    pub fn loops(&self) -> impl Iterator<Item = &Contour> {
        std::iter::once(&self.outer).chain(self.holes.iter())
    }

    /// The region as world-space triangles, for drawing it filled. A profile the
    /// triangulator cannot handle yields nothing rather than an error: this exists for
    /// highlighting, where the worst outcome is a region that does not light up.
    pub fn triangles(&self) -> Vec<[Vec3; 3]> {
        let points: Vec<Vec3> = self
            .loops()
            .flat_map(|c| c.points.iter())
            .map(|p| self.frame.to_world(*p))
            .collect();
        triangulate(self)
            .unwrap_or_default()
            .into_iter()
            .map(|[a, b, c]| [points[a], points[b], points[c]])
            .collect()
    }
}

/// A 3D polyline a profile is swept along.
#[derive(Debug, Clone, PartialEq)]
pub struct Path3 {
    pub points: Vec<Vec3>,
}

impl Path3 {
    /// Removes consecutive duplicates so every segment has a direction.
    pub(crate) fn cleaned(&self) -> Result<Vec<Vec3>, KernelError> {
        let mut out: Vec<Vec3> = Vec::with_capacity(self.points.len());
        for &p in &self.points {
            if out.last().is_none_or(|q| q.distance_squared(p) > 1e-14) {
                out.push(p);
            }
        }
        if out.len() < 2 {
            return Err(KernelError::DegeneratePath);
        }
        Ok(out)
    }
}

/// How far an extrude pushes the profile along its plane normal, in mm.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Extent {
    OneSide(f64),
    /// Total length, split equally either side of the profile plane.
    Symmetric(f64),
    TwoSides {
        positive: f64,
        negative: f64,
    },
}

impl Extent {
    /// Ordered `(low, high)` offsets along the normal.
    pub(crate) fn range(self) -> Result<(f64, f64), KernelError> {
        let (lo, hi) = match self {
            Extent::OneSide(d) => (d.min(0.0), d.max(0.0)),
            Extent::Symmetric(d) => (-d.abs() / 2.0, d.abs() / 2.0),
            Extent::TwoSides { positive, negative } => (-negative.abs(), positive.abs()),
        };
        if hi - lo < 1e-9 || !(hi - lo).is_finite() {
            return Err(KernelError::ZeroExtent);
        }
        Ok((lo, hi))
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Axis {
    pub origin: Vec3,
    pub direction: Vec3,
}

impl Axis {
    pub fn new(origin: Vec3, direction: Vec3) -> Self {
        Self {
            origin,
            direction: direction.normalize(),
        }
    }
}

/// Resolution for curved surfaces created by the kernel (revolve sweeps, fillets).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Tessellation {
    /// Maximum sagitta between a facet and the true surface, in mm.
    pub chord_tolerance: f64,
    /// Maximum angle one facet may span, in radians.
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
    pub fn segment_count(&self, radius: f64, sweep: f64) -> usize {
        // The small slack keeps an exact multiple (a 90° sweep at 10° per facet) from
        // rounding up to an extra facet when the sweep carries a few ulps of noise.
        const SLACK: f64 = 1e-9;
        let sweep = sweep.abs();
        let by_angle = (sweep / self.max_segment_angle.max(1e-6) - SLACK).ceil();
        let by_chord = if radius > self.chord_tolerance {
            let alpha = 2.0 * (1.0 - self.chord_tolerance / radius).acos();
            (sweep / alpha.max(1e-6) - SLACK).ceil()
        } else {
            1.0
        };
        by_angle.max(by_chord).max(1.0) as usize
    }
}

pub fn signed_area(points: &[Vec2]) -> f64 {
    let n = points.len();
    if n < 3 {
        return 0.0;
    }
    (0..n)
        .map(|i| points[i].perp_dot(points[(i + 1) % n]))
        .sum::<f64>()
        * 0.5
}

/// Even-odd crossing test.
pub fn polygon_contains(points: &[Vec2], p: Vec2) -> bool {
    let n = points.len();
    let mut inside = false;
    let mut j = n.wrapping_sub(1);
    for i in 0..n {
        let (a, b) = (points[i], points[j]);
        if (a.y > p.y) != (b.y > p.y) && p.x < a.x + (p.y - a.y) / (b.y - a.y) * (b.x - a.x) {
            inside = !inside;
        }
        j = i;
    }
    inside
}

/// Triangulates a normalised profile. Returns index triples into the concatenation of the
/// outer loop's points followed by each hole's points, wound counter-clockwise.
pub(crate) fn triangulate(profile: &Profile) -> Result<Vec<[usize; 3]>, KernelError> {
    let mut flat = Vec::new();
    let mut hole_starts = Vec::new();
    for (i, c) in profile.loops().enumerate() {
        if i > 0 {
            hole_starts.push(flat.len() / 2);
        }
        for p in &c.points {
            flat.extend_from_slice(&[p.x, p.y]);
        }
    }
    let all: Vec<Vec2> = profile
        .loops()
        .flat_map(|c| c.points.iter().copied())
        .collect();
    let indices = earcutr::earcut(&flat, &hole_starts, 2)
        .map_err(|_| KernelError::Triangulation(profile.frame.to_world(all[0])))?;
    if indices.is_empty() {
        return Err(KernelError::Triangulation(profile.frame.to_world(all[0])));
    }
    let mut out = Vec::with_capacity(indices.len() / 3);
    for &[a, b, c] in indices.as_chunks::<3>().0 {
        let area = (all[b] - all[a]).perp_dot(all[c] - all[a]);
        if area.abs() < 1e-14 {
            continue;
        }
        out.push(if area > 0.0 { [a, b, c] } else { [a, c, b] });
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn square(size: f64) -> Contour {
        let s = size / 2.0;
        Contour::polygon(
            vec![
                Vec2::new(-s, -s),
                Vec2::new(s, -s),
                Vec2::new(s, s),
                Vec2::new(-s, s),
            ],
            7,
        )
    }

    #[test]
    fn normalisation_orients_loops() {
        let outer = square(4.0).reversed();
        let hole = square(1.0);
        let p = Profile {
            frame: Frame::XY,
            outer,
            holes: vec![hole],
        }
        .normalised()
        .unwrap();
        assert!(p.outer.signed_area() > 0.0);
        assert!(p.holes[0].signed_area() < 0.0);
        assert_relative_eq!(p.area(), 15.0);
        assert!(p.contains(Vec2::new(1.5, 1.5)));
        assert!(!p.contains(Vec2::ZERO));
    }

    #[test]
    fn reversed_keeps_segments_on_edges() {
        let mut c = square(2.0);
        c.segments[1].curve = 99;
        let r = c.reversed();
        // Edge 1 joined points 1 and 2, which are now at reversed indices 2 and 1.
        assert_eq!(r.segments[1].curve, 99);
        assert_eq!(r.reversed(), c);
    }

    #[test]
    fn triangles_of_a_profile_are_world_space_and_cover_its_area() {
        let p = Profile {
            frame: Frame::XY,
            outer: square(4.0),
            holes: vec![square(2.0).reversed()],
        }
        .normalised()
        .unwrap();
        let tris = p.triangles();
        let area: f64 = tris
            .iter()
            .map(|[a, b, c]| (*b - *a).cross(*c - *a).length() * 0.5)
            .sum();
        assert_relative_eq!(area, 12.0, epsilon = 1e-9);
        assert!(tris.iter().flatten().all(|v| v.z == 0.0));
    }

    #[test]
    fn triangulation_covers_area_with_hole() {
        let p = Profile {
            frame: Frame::XY,
            outer: square(4.0),
            holes: vec![square(2.0).reversed()],
        }
        .normalised()
        .unwrap();
        let all: Vec<Vec2> = p.loops().flat_map(|c| c.points.iter().copied()).collect();
        let tris = triangulate(&p).unwrap();
        let area: f64 = tris
            .iter()
            .map(|[a, b, c]| (all[*b] - all[*a]).perp_dot(all[*c] - all[*a]) * 0.5)
            .sum();
        assert_relative_eq!(area, 12.0, epsilon = 1e-9);
    }

    #[test]
    fn extent_ranges() {
        assert_eq!(Extent::OneSide(-3.0).range().unwrap(), (-3.0, 0.0));
        assert_eq!(Extent::Symmetric(4.0).range().unwrap(), (-2.0, 2.0));
        assert_eq!(
            Extent::TwoSides {
                positive: 1.0,
                negative: 2.0
            }
            .range()
            .unwrap(),
            (-2.0, 1.0)
        );
        assert!(Extent::OneSide(0.0).range().is_err());
    }

    #[test]
    fn circle_contour_area() {
        let c = Contour::circle(
            Vec2::ZERO,
            10.0,
            1,
            &Tessellation {
                chord_tolerance: 1e-4,
                ..Default::default()
            },
        );
        assert_relative_eq!(c.signed_area(), std::f64::consts::PI * 100.0, epsilon = 0.2);
    }
}
