//! Shape builders. Each creates the entities a Fusion tool would and adds the
//! constraints that make the shape keep its identity under later edits (a rectangle
//! stays rectangular, a polygon stays regular, a slot's arcs stay tangent).
//!
//! Shared corners are expressed by sharing point entities rather than by coincident
//! constraints: it is the same topology with fewer equations.

use std::f64::consts::TAU;

use basset_math::Vec2;

use crate::{Constraint, ConstraintId, EntityId, Sketch, SketchError};

#[derive(Debug, Clone)]
pub struct Rectangle {
    /// Corners in counter-clockwise order starting at the first given corner.
    pub corners: [EntityId; 4],
    /// `lines[i]` joins `corners[i]` to `corners[(i + 1) % 4]`.
    pub lines: [EntityId; 4],
    /// Construction centre point (centre rectangles only).
    pub center: Option<EntityId>,
    pub constraints: Vec<ConstraintId>,
}

#[derive(Debug, Clone, Copy)]
pub struct Circle {
    pub center: EntityId,
    pub circle: EntityId,
}

#[derive(Debug, Clone)]
pub struct Polygon {
    pub center: EntityId,
    /// Construction circumscribed circle.
    pub circle: EntityId,
    pub vertices: Vec<EntityId>,
    pub edges: Vec<EntityId>,
    pub constraints: Vec<ConstraintId>,
}

#[derive(Debug, Clone)]
pub struct Slot {
    pub centers: [EntityId; 2],
    pub arcs: [EntityId; 2],
    pub lines: [EntityId; 2],
    /// Construction line between the arc centres.
    pub center_line: EntityId,
    pub constraints: Vec<ConstraintId>,
}

#[derive(Debug, Clone, Copy)]
pub struct Arc {
    pub center: EntityId,
    pub start: EntityId,
    pub end: EntityId,
    pub arc: EntityId,
}

#[derive(Debug, Clone)]
pub struct Polyline {
    pub points: Vec<EntityId>,
    pub lines: Vec<EntityId>,
}

/// Builders only reference entities they just created, so these cannot fail; a failure
/// would be a bug in this module, not user error.
fn must<T>(r: Result<T, SketchError>) -> T {
    r.unwrap_or_else(|e| panic!("shape builder produced an invalid sketch: {e}"))
}

fn rectangle_from_corners(s: &mut Sketch, a: Vec2, b: Vec2) -> Rectangle {
    let (min, max) = (a.min(b), a.max(b));
    let pts = [min, Vec2::new(max.x, min.y), max, Vec2::new(min.x, max.y)];
    let corners = pts.map(|p| s.add_point(p));
    let lines = [0, 1, 2, 3].map(|i| must(s.add_line(corners[i], corners[(i + 1) % 4])));
    let constraints = vec![
        must(s.add_constraint(Constraint::Horizontal(lines[0]))),
        must(s.add_constraint(Constraint::Vertical(lines[1]))),
        must(s.add_constraint(Constraint::Horizontal(lines[2]))),
        must(s.add_constraint(Constraint::Vertical(lines[3]))),
    ];
    Rectangle {
        corners,
        lines,
        center: None,
        constraints,
    }
}

pub fn rectangle_two_point(s: &mut Sketch, a: Vec2, b: Vec2) -> Rectangle {
    rectangle_from_corners(s, a, b)
}

/// Rectangle defined by its centre and one corner. Two construction diagonals with the
/// centre at their midpoints keep the centre point meaningful when a corner is dragged.
pub fn rectangle_center(s: &mut Sketch, center: Vec2, corner: Vec2) -> Rectangle {
    let opposite = center * 2.0 - corner;
    let mut rect = rectangle_from_corners(s, corner, opposite);
    let c = s.add_point(center);
    must(s.set_construction(c, true));
    for (i, j) in [(0, 2), (1, 3)] {
        let diagonal = must(s.add_line(rect.corners[i], rect.corners[j]));
        must(s.set_construction(diagonal, true));
        rect.constraints
            .push(must(s.add_constraint(Constraint::Midpoint {
                point: c,
                line: diagonal,
            })));
    }
    rect.center = Some(c);
    rect
}

pub fn circle_center(s: &mut Sketch, center: Vec2, radius: f64) -> Circle {
    let c = s.add_point(center);
    let circle = must(s.add_circle(c, radius.abs().max(f64::MIN_POSITIVE)));
    Circle { center: c, circle }
}

/// Circle through two diametrically opposite points.
pub fn circle_two_point(s: &mut Sketch, a: Vec2, b: Vec2) -> Circle {
    circle_center(s, (a + b) * 0.5, a.distance(b) * 0.5)
}

/// Centre and radius of the circle through three points, or `None` when collinear.
pub fn circumcircle(a: Vec2, b: Vec2, c: Vec2) -> Option<(Vec2, f64)> {
    let d = 2.0 * (a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y));
    if d.abs() < 1e-12 * (a.distance(b) + b.distance(c) + c.distance(a)).max(1.0) {
        return None;
    }
    let (a2, b2, c2) = (a.length_squared(), b.length_squared(), c.length_squared());
    let ux = (a2 * (b.y - c.y) + b2 * (c.y - a.y) + c2 * (a.y - b.y)) / d;
    let uy = (a2 * (c.x - b.x) + b2 * (a.x - c.x) + c2 * (b.x - a.x)) / d;
    let center = Vec2::new(ux, uy);
    Some((center, center.distance(a)))
}

pub fn circle_three_point(
    s: &mut Sketch,
    a: Vec2,
    b: Vec2,
    c: Vec2,
) -> Result<Circle, SketchError> {
    let (center, radius) = circumcircle(a, b, c).ok_or_else(|| {
        SketchError::DegenerateGeometry("three-point circle: points are collinear".into())
    })?;
    Ok(circle_center(s, center, radius))
}

/// Regular polygon inscribed in a construction circle; vertices are constrained onto
/// the circle and all edges equal so the polygon stays regular when resized.
pub fn polygon_center(
    s: &mut Sketch,
    center: Vec2,
    first_vertex: Vec2,
    sides: usize,
) -> Result<Polygon, SketchError> {
    if sides < 3 {
        return Err(SketchError::InvalidArgument(format!(
            "a polygon needs at least 3 sides, got {sides}"
        )));
    }
    let radius = center.distance(first_vertex);
    if radius <= 0.0 {
        return Err(SketchError::DegenerateGeometry(
            "polygon vertex coincides with its centre".into(),
        ));
    }
    let c = s.add_point(center);
    let circle = s.add_circle(c, radius)?;
    s.set_construction(circle, true)?;
    let a0 = (first_vertex - center).to_angle();
    let vertices: Vec<EntityId> = (0..sides)
        .map(|i| {
            s.add_point(center + Vec2::from_angle(a0 + TAU * i as f64 / sides as f64) * radius)
        })
        .collect();
    let edges: Vec<EntityId> = (0..sides)
        .map(|i| must(s.add_line(vertices[i], vertices[(i + 1) % sides])))
        .collect();
    let mut constraints = Vec::new();
    for &v in &vertices {
        constraints.push(must(s.add_constraint(Constraint::Coincident {
            point: v,
            target: circle,
        })));
    }
    for &e in &edges[1..] {
        constraints.push(must(s.add_constraint(Constraint::Equal(edges[0], e))));
    }
    Ok(Polygon {
        center: c,
        circle,
        vertices,
        edges,
        constraints,
    })
}

/// Stadium slot: two parallel lines capped by semicircles about `a` and `b`.
pub fn slot_center_to_center(s: &mut Sketch, a: Vec2, b: Vec2, width: f64) -> Slot {
    let r = width.abs() * 0.5;
    let dir = (b - a).normalize_or(Vec2::X);
    let n = dir.perp();
    let (ca, cb) = (s.add_point(a), s.add_point(b));
    // Counter-clockwise around the slot: bottom line, arc at b, top line, arc at a.
    let a2 = s.add_point(a - n * r);
    let b2 = s.add_point(b - n * r);
    let b1 = s.add_point(b + n * r);
    let a1 = s.add_point(a + n * r);
    let bottom = must(s.add_line(a2, b2));
    let arc_b = must(s.add_arc(cb, b2, b1));
    let top = must(s.add_line(b1, a1));
    let arc_a = must(s.add_arc(ca, a1, a2));
    let center_line = must(s.add_line(ca, cb));
    must(s.set_construction(center_line, true));
    let constraints = vec![
        must(s.add_constraint(Constraint::Tangent(bottom, arc_b))),
        must(s.add_constraint(Constraint::Tangent(top, arc_b))),
        must(s.add_constraint(Constraint::Tangent(bottom, arc_a))),
        must(s.add_constraint(Constraint::Tangent(top, arc_a))),
        must(s.add_constraint(Constraint::Equal(arc_a, arc_b))),
    ];
    Slot {
        centers: [ca, cb],
        arcs: [arc_a, arc_b],
        lines: [bottom, top],
        center_line,
        constraints,
    }
}

/// Arc through three points. The entity is always CCW from its `start` to its `end`,
/// so when the requested arc runs clockwise the endpoints are stored swapped.
pub fn arc_three_point(
    s: &mut Sketch,
    start: Vec2,
    mid: Vec2,
    end: Vec2,
) -> Result<Arc, SketchError> {
    let (center, _) = circumcircle(start, mid, end).ok_or_else(|| {
        SketchError::DegenerateGeometry("three-point arc: points are collinear".into())
    })?;
    let a_start = (start - center).to_angle();
    let sweep_to_end = crate::tessellation::ccw_sweep(a_start, (end - center).to_angle());
    let ccw = crate::tessellation::angle_within((mid - center).to_angle(), a_start, sweep_to_end);
    let (p, q) = if ccw { (start, end) } else { (end, start) };
    Ok(arc_center(s, center, p, q))
}

/// CCW arc about `center` from `start` toward `end`; `end` is projected onto the arc's
/// circle so the implicit radius constraint holds from the outset.
pub fn arc_center(s: &mut Sketch, center: Vec2, start: Vec2, end: Vec2) -> Arc {
    let radius = center.distance(start);
    let end_dir = (end - center).normalize_or(Vec2::X);
    let c = s.add_point(center);
    let sp = s.add_point(start);
    let ep = s.add_point(center + end_dir * radius);
    let arc = must(s.add_arc(c, sp, ep));
    Arc {
        center: c,
        start: sp,
        end: ep,
        arc,
    }
}

pub fn polyline(s: &mut Sketch, points: &[Vec2], closed: bool) -> Polyline {
    let ids: Vec<EntityId> = points.iter().map(|p| s.add_point(*p)).collect();
    let n = ids.len();
    let edges = if closed && n > 2 {
        n
    } else {
        n.saturating_sub(1)
    };
    let lines = (0..edges)
        .map(|i| must(s.add_line(ids[i], ids[(i + 1) % n])))
        .collect();
    Polyline { points: ids, lines }
}
