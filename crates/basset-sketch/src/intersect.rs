//! Curve–curve intersection in the sketch plane.
//!
//! Trim and break ([`crate::edit`]) need the points where a curve crosses its
//! neighbours, and they need them on the *analytic* curve rather than on the polyline
//! the profile tracer uses: a trim that cut at a tessellation vertex would leave the
//! survivor a fraction of a chord short of the geometry it was trimmed against, and the
//! region the user trimmed open would not close again when they drew the next line.
//!
//! Every curve is reduced to one of two primitives (segment, circular arc) and
//! parameterised over `0..=1` so a piece of it can be named by a parameter range.

use std::f64::consts::TAU;

use basset_math::{ANGULAR_TOL, Vec2};

use crate::geometry::point_segment_distance;
use crate::sketch::JOIN_TOL;
use crate::tessellation::{angle_within, ccw_sweep};
use crate::{Entity, EntityId, Sketch};

/// The analytic shape of a sketch curve, detached from the sketch's points so trimming
/// can reason about positions while it edits the entities those positions came from.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum CurveGeom {
    Line {
        a: Vec2,
        b: Vec2,
    },
    /// Counter-clockwise from `start_angle` through `sweep`. A circle is the closed
    /// case: it sweeps a full turn and its parameter wraps.
    Arc {
        center: Vec2,
        radius: f64,
        start_angle: f64,
        sweep: f64,
        closed: bool,
    },
}

impl CurveGeom {
    /// The shape of a line, arc or circle. Points and text have no shape to cross.
    pub fn of(sketch: &Sketch, id: EntityId) -> Option<Self> {
        match sketch.entity(id)?.entity {
            Entity::Line { start, end } => Some(CurveGeom::Line {
                a: sketch.point_pos(start)?,
                b: sketch.point_pos(end)?,
            }),
            Entity::Circle { center, radius } => Some(CurveGeom::Arc {
                center: sketch.point_pos(center)?,
                radius,
                start_angle: 0.0,
                sweep: TAU,
                closed: true,
            }),
            Entity::Arc { center, start, end } => {
                let c = sketch.point_pos(center)?;
                let (s, e) = (sketch.point_pos(start)?, sketch.point_pos(end)?);
                let start_angle = (s - c).to_angle();
                Some(CurveGeom::Arc {
                    center: c,
                    radius: (s - c).length(),
                    start_angle,
                    sweep: ccw_sweep(start_angle, (e - c).to_angle()),
                    closed: false,
                })
            }
            Entity::Point { .. } | Entity::Text { .. } => None,
        }
    }

    pub fn is_closed(&self) -> bool {
        matches!(self, CurveGeom::Arc { closed: true, .. })
    }

    /// Position at parameter `t`, `0` at the start and `1` at the end.
    pub fn point_at(&self, t: f64) -> Vec2 {
        match *self {
            CurveGeom::Line { a, b } => a.lerp(b, t),
            CurveGeom::Arc {
                center,
                radius,
                start_angle,
                sweep,
                ..
            } => center + Vec2::from_angle(start_angle + sweep * t) * radius,
        }
    }

    /// Parameter of the point on the curve nearest `p`, in `0..=1`. Positions off the
    /// curve are projected onto it, which is what a pick from the screen needs.
    pub fn param_of(&self, p: Vec2) -> f64 {
        match *self {
            CurveGeom::Line { a, b } => {
                let d = b - a;
                let len2 = d.length_squared();
                if len2 <= f64::EPSILON {
                    0.0
                } else {
                    ((p - a).dot(d) / len2).clamp(0.0, 1.0)
                }
            }
            CurveGeom::Arc {
                center,
                start_angle,
                sweep,
                closed,
                ..
            } => {
                let t = (p - center).to_angle();
                let along = (t - start_angle).rem_euclid(TAU) / sweep;
                if closed { along % 1.0 } else { along.min(1.0) }
            }
        }
    }

    /// Curve length, used to reject pieces too short to be worth keeping.
    pub fn length(&self) -> f64 {
        match *self {
            CurveGeom::Line { a, b } => a.distance(b),
            CurveGeom::Arc { radius, sweep, .. } => radius * sweep,
        }
    }

    /// Whether `p` lies on the curve *between its ends*, which is what makes a crossing
    /// of the underlying line or circle a crossing of the drawn curve.
    pub fn covers(&self, p: Vec2) -> bool {
        match *self {
            CurveGeom::Line { a, b } => point_segment_distance(p, a, b) <= JOIN_TOL,
            CurveGeom::Arc {
                center,
                radius,
                start_angle,
                sweep,
                closed,
            } => {
                if ((p - center).length() - radius).abs() > JOIN_TOL {
                    return false;
                }
                closed || angle_within((p - center).to_angle(), start_angle, sweep)
            }
        }
    }
}

/// Where two curves cross, ignoring overlap: collinear segments and identical circles
/// meet everywhere rather than at points, and trimming against them is undefined, so
/// they are reported as not crossing at all.
pub fn intersections(a: &CurveGeom, b: &CurveGeom) -> Vec<Vec2> {
    let candidates = match (a, b) {
        (CurveGeom::Line { a: p0, b: p1 }, CurveGeom::Line { a: q0, b: q1 }) => {
            line_line(*p0, *p1, *q0, *q1).into_iter().collect()
        }
        (CurveGeom::Line { a: p0, b: p1 }, CurveGeom::Arc { center, radius, .. })
        | (CurveGeom::Arc { center, radius, .. }, CurveGeom::Line { a: p0, b: p1 }) => {
            line_circle(*p0, *p1, *center, *radius)
        }
        (
            CurveGeom::Arc {
                center: c0,
                radius: r0,
                ..
            },
            CurveGeom::Arc {
                center: c1,
                radius: r1,
                ..
            },
        ) => circle_circle(*c0, *r0, *c1, *r1),
    };
    let mut out: Vec<Vec2> = Vec::new();
    for p in candidates {
        if a.covers(p) && b.covers(p) && !out.iter().any(|q| q.distance(p) <= JOIN_TOL) {
            out.push(p);
        }
    }
    out
}

/// Intersection of two infinite lines. Parallel lines, including collinear ones, yield
/// nothing.
fn line_line(p0: Vec2, p1: Vec2, q0: Vec2, q1: Vec2) -> Option<Vec2> {
    let (r, s) = (p1 - p0, q1 - q0);
    let denom = r.perp_dot(s);
    // Scale the parallelism test by the lengths involved so it means an angle, not a
    // cross product whose size depends on how big the drawing is.
    if denom.abs() <= ANGULAR_TOL * r.length() * s.length() {
        return None;
    }
    Some(p0 + r * ((q0 - p0).perp_dot(s) / denom))
}

/// Intersections of an infinite line with a full circle.
fn line_circle(p0: Vec2, p1: Vec2, center: Vec2, radius: f64) -> Vec<Vec2> {
    let d = p1 - p0;
    let len = d.length();
    if len <= f64::EPSILON || radius <= 0.0 {
        return Vec::new();
    }
    let dir = d / len;
    let foot = p0 + dir * dir.dot(center - p0);
    let gap = foot.distance(center);
    if gap > radius + JOIN_TOL {
        return Vec::new();
    }
    // A tangent line touches once; clamping keeps the square root real when the foot is
    // a hair outside the circle through rounding.
    let half = (radius * radius - gap.min(radius).powi(2)).max(0.0).sqrt();
    if half <= JOIN_TOL {
        vec![foot]
    } else {
        vec![foot - dir * half, foot + dir * half]
    }
}

/// Intersections of two full circles. Concentric or identical circles yield nothing.
fn circle_circle(c0: Vec2, r0: f64, c1: Vec2, r1: f64) -> Vec<Vec2> {
    let d = c1 - c0;
    let dist = d.length();
    if dist <= JOIN_TOL || dist > r0 + r1 + JOIN_TOL || dist < (r0 - r1).abs() - JOIN_TOL {
        return Vec::new();
    }
    let along = (dist * dist + r0 * r0 - r1 * r1) / (2.0 * dist);
    let half = (r0 * r0 - along * along).max(0.0).sqrt();
    let base = c0 + d / dist * along;
    if half <= JOIN_TOL {
        vec![base]
    } else {
        let off = (d / dist).perp() * half;
        vec![base - off, base + off]
    }
}

/// Every point where `curve` is crossed by another curve of the sketch, as parameters
/// along `curve` sorted from its start. Construction geometry cuts too: it is drawn to
/// be drawn against.
pub fn crossing_params(sketch: &Sketch, curve: EntityId) -> Vec<f64> {
    let Some(geom) = CurveGeom::of(sketch, curve) else {
        return Vec::new();
    };
    let mut params: Vec<f64> = Vec::new();
    for (other, data) in sketch.entities() {
        if other == curve || !data.entity.is_curve() {
            continue;
        }
        let Some(other_geom) = CurveGeom::of(sketch, other) else {
            continue;
        };
        for p in intersections(&geom, &other_geom) {
            params.push(geom.param_of(p));
        }
    }
    params.sort_by(f64::total_cmp);
    // Two curves meeting at one point (a corner) are found twice when a third curve
    // passes through the same place; one cut there is enough.
    params.dedup_by(|a, b| (*a - *b).abs() <= param_tol(&geom));
    params
}

/// Parameter distance that counts as "the same place on this curve", from [`JOIN_TOL`]
/// in millimetres.
pub fn param_tol(geom: &CurveGeom) -> f64 {
    let len = geom.length();
    if len <= JOIN_TOL { 1.0 } else { JOIN_TOL / len }
}
