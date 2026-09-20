//! Crate-level tests: solver behaviour, shape builders, profile extraction, hit
//! testing, serialisation and text.

use std::f64::consts::{FRAC_PI_2, FRAC_PI_4, PI};

use approx::assert_relative_eq;
use basset_math::Vec2;

use crate::shapes;
use crate::solver::compile_for_test;
use crate::{
    Constraint, Entity, EntityId, Font, Profile, SegmentKind, Sketch, SketchError, SolveError,
    Tessellation,
};

fn v(x: f64, y: f64) -> Vec2 {
    Vec2::new(x, y)
}

fn pos(s: &Sketch, id: EntityId) -> Vec2 {
    s.point_pos(id).expect("point")
}

fn line(s: &mut Sketch, a: Vec2, b: Vec2) -> (EntityId, EntityId, EntityId) {
    let pa = s.add_point(a);
    let pb = s.add_point(b);
    let l = s.add_line(pa, pb).unwrap();
    (l, pa, pb)
}

/// Vec2 has no `approx` impl, so positions are compared through their distance.
fn near(a: Vec2, b: Vec2) {
    assert!(a.distance(b) < 1e-6, "{a:?} is not {b:?}");
}

fn radius_of(s: &Sketch, id: EntityId) -> f64 {
    match s.entity(id).unwrap().entity {
        Entity::Circle { radius, .. } => radius,
        Entity::Arc { center, start, .. } => pos(s, start).distance(pos(s, center)),
        _ => panic!("not circular"),
    }
}

/// Compares the dual-number Jacobian against central finite differences.
fn check_jacobian(sketch: &Sketch) {
    let mut sys = compile_for_test(sketch);
    let (r0, j) = sys.jacobian().unwrap();
    assert!(!r0.is_empty(), "test sketch has no equations");
    let n = sys.free_count();
    let h = 1e-6;
    for col in 0..n {
        let orig = sys.params_mut()[col];
        sys.params_mut()[col] = orig + h;
        let (rp, _) = sys.jacobian().unwrap();
        sys.params_mut()[col] = orig - h;
        let (rm, _) = sys.jacobian().unwrap();
        sys.params_mut()[col] = orig;
        for row in 0..r0.len() {
            let fd = (rp[row] - rm[row]) / (2.0 * h);
            assert_relative_eq!(j.at(row, col), fd, epsilon = 1e-6, max_relative = 1e-5);
        }
    }
}

// ----- solver: each constraint ----------------------------------------------------------

#[test]
fn unconstrained_sketch_solves_trivially() {
    let mut s = Sketch::new();
    line(&mut s, v(0.0, 0.0), v(3.0, 1.0));
    let r = s.solve().unwrap();
    assert_eq!(r.iterations, 0);
    assert!(r.converged);
    assert_eq!(r.degrees_of_freedom, 4);
}

#[test]
fn coincident_point_point() {
    let mut s = Sketch::new();
    let a = s.add_point(v(0.0, 0.0));
    let b = s.add_point(v(2.0, 3.0));
    s.add_constraint(Constraint::Coincident {
        point: a,
        target: b,
    })
    .unwrap();
    check_jacobian(&s);
    let r = s.solve().unwrap();
    assert!(pos(&s, a).distance(pos(&s, b)) < 1e-9);
    assert_eq!(r.degrees_of_freedom, 2);
}

#[test]
fn coincident_point_on_line_uses_infinite_line() {
    let mut s = Sketch::new();
    let (l, pa, pb) = line(&mut s, v(0.0, 0.0), v(1.0, 0.0));
    s.add_constraint(Constraint::Fix(pa)).unwrap();
    s.add_constraint(Constraint::Fix(pb)).unwrap();
    let p = s.add_point(v(5.0, 0.7));
    s.add_constraint(Constraint::Coincident {
        point: p,
        target: l,
    })
    .unwrap();
    check_jacobian(&s);
    let r = s.solve().unwrap();
    assert_relative_eq!(pos(&s, p).y, 0.0, epsilon = 1e-9);
    assert_relative_eq!(pos(&s, p).x, 5.0, epsilon = 1e-6);
    assert_eq!(r.degrees_of_freedom, 1);
}

#[test]
fn coincident_point_on_circle_and_arc() {
    let mut s = Sketch::new();
    let c = shapes::circle_center(&mut s, v(0.0, 0.0), 2.0);
    s.add_constraint(Constraint::Fix(c.center)).unwrap();
    s.add_constraint(Constraint::Radius {
        curve: c.circle,
        value: 2.0,
    })
    .unwrap();
    let p = s.add_point(v(3.0, 0.5));
    s.add_constraint(Constraint::Coincident {
        point: p,
        target: c.circle,
    })
    .unwrap();
    let arc = shapes::arc_center(&mut s, v(10.0, 0.0), v(11.0, 0.0), v(10.0, 1.0));
    s.add_constraint(Constraint::Fix(arc.center)).unwrap();
    s.add_constraint(Constraint::Fix(arc.start)).unwrap();
    let q = s.add_point(v(9.0, 2.0));
    s.add_constraint(Constraint::Coincident {
        point: q,
        target: arc.arc,
    })
    .unwrap();
    check_jacobian(&s);
    s.solve().unwrap();
    assert_relative_eq!(pos(&s, p).length(), 2.0, epsilon = 1e-9);
    assert_relative_eq!(pos(&s, q).distance(v(10.0, 0.0)), 1.0, epsilon = 1e-9);
}

#[test]
fn horizontal_and_vertical() {
    let mut s = Sketch::new();
    let (h, _, hb) = line(&mut s, v(0.0, 0.0), v(4.0, 1.0));
    let (vl, _, vb) = line(&mut s, v(0.0, 0.0), v(1.0, 4.0));
    s.add_constraint(Constraint::Horizontal(h)).unwrap();
    s.add_constraint(Constraint::Vertical(vl)).unwrap();
    check_jacobian(&s);
    s.solve().unwrap();
    assert_relative_eq!(pos(&s, hb).y, 0.5, epsilon = 1e-9);
    assert_relative_eq!(pos(&s, vb).x, 0.5, epsilon = 1e-9);
}

#[test]
fn parallel_and_perpendicular() {
    let mut s = Sketch::new();
    let (l1, a1, b1) = line(&mut s, v(0.0, 0.0), v(4.0, 0.0));
    s.add_constraint(Constraint::Fix(a1)).unwrap();
    s.add_constraint(Constraint::Fix(b1)).unwrap();
    let (l2, a2, b2) = line(&mut s, v(0.0, 2.0), v(4.0, 3.0));
    let (l3, a3, b3) = line(&mut s, v(6.0, 0.0), v(7.0, 5.0));
    s.add_constraint(Constraint::Parallel(l1, l2)).unwrap();
    s.add_constraint(Constraint::Perpendicular(l1, l3)).unwrap();
    check_jacobian(&s);
    s.solve().unwrap();
    let d2 = pos(&s, b2) - pos(&s, a2);
    let d3 = pos(&s, b3) - pos(&s, a3);
    assert_relative_eq!(d2.y, 0.0, epsilon = 1e-9);
    assert_relative_eq!(d3.x, 0.0, epsilon = 1e-9);
}

#[test]
fn equal_lengths_and_radii() {
    let mut s = Sketch::new();
    let (l1, a1, b1) = line(&mut s, v(0.0, 0.0), v(4.0, 0.0));
    s.add_constraint(Constraint::Fix(a1)).unwrap();
    s.add_constraint(Constraint::Fix(b1)).unwrap();
    let (l2, a2, b2) = line(&mut s, v(0.0, 2.0), v(1.0, 3.0));
    s.add_constraint(Constraint::Equal(l1, l2)).unwrap();
    let c1 = shapes::circle_center(&mut s, v(10.0, 0.0), 3.0);
    let c2 = shapes::circle_center(&mut s, v(20.0, 0.0), 1.0);
    let arc = shapes::arc_center(&mut s, v(30.0, 0.0), v(32.0, 0.0), v(30.0, 2.0));
    s.add_constraint(Constraint::Radius {
        curve: c1.circle,
        value: 3.0,
    })
    .unwrap();
    s.add_constraint(Constraint::Equal(c1.circle, c2.circle))
        .unwrap();
    s.add_constraint(Constraint::Equal(arc.arc, c1.circle))
        .unwrap();
    check_jacobian(&s);
    s.solve().unwrap();
    assert_relative_eq!(pos(&s, a2).distance(pos(&s, b2)), 4.0, epsilon = 1e-9);
    assert_relative_eq!(radius_of(&s, c2.circle), 3.0, epsilon = 1e-9);
    assert_relative_eq!(radius_of(&s, arc.arc), 3.0, epsilon = 1e-9);
}

#[test]
fn tangent_line_circle_keeps_side() {
    let mut s = Sketch::new();
    let (l, a, b) = line(&mut s, v(0.0, 0.0), v(10.0, 0.0));
    s.add_constraint(Constraint::Fix(a)).unwrap();
    s.add_constraint(Constraint::Fix(b)).unwrap();
    let c = shapes::circle_center(&mut s, v(5.0, 3.0), 2.0);
    s.add_constraint(Constraint::Radius {
        curve: c.circle,
        value: 2.0,
    })
    .unwrap();
    s.add_constraint(Constraint::Tangent(l, c.circle)).unwrap();
    check_jacobian(&s);
    s.solve().unwrap();
    assert_relative_eq!(pos(&s, c.center).y, 2.0, epsilon = 1e-9);
    // Circle below the line stays below.
    let mut s2 = Sketch::new();
    let (l, a, b) = line(&mut s2, v(0.0, 0.0), v(10.0, 0.0));
    s2.add_constraint(Constraint::Fix(a)).unwrap();
    s2.add_constraint(Constraint::Fix(b)).unwrap();
    let c = shapes::circle_center(&mut s2, v(5.0, -3.0), 2.0);
    s2.add_constraint(Constraint::Radius {
        curve: c.circle,
        value: 2.0,
    })
    .unwrap();
    s2.add_constraint(Constraint::Tangent(c.circle, l)).unwrap();
    s2.solve().unwrap();
    assert_relative_eq!(pos(&s2, c.center).y, -2.0, epsilon = 1e-9);
}

#[test]
fn tangent_line_arc() {
    let mut s = Sketch::new();
    let (l, a, b) = line(&mut s, v(0.0, 0.0), v(10.0, 0.0));
    s.add_constraint(Constraint::Fix(a)).unwrap();
    s.add_constraint(Constraint::Fix(b)).unwrap();
    let arc = shapes::arc_center(&mut s, v(5.0, 2.5), v(7.0, 2.5), v(3.0, 2.5));
    s.add_constraint(Constraint::Fix(arc.center)).unwrap();
    s.add_constraint(Constraint::Tangent(l, arc.arc)).unwrap();
    check_jacobian(&s);
    s.solve().unwrap();
    assert_relative_eq!(radius_of(&s, arc.arc), 2.5, epsilon = 1e-9);
    assert_relative_eq!(pos(&s, arc.end).distance(v(5.0, 2.5)), 2.5, epsilon = 1e-9);
}

#[test]
fn tangent_at_shared_endpoint() {
    // Line from the origin to (10, 0), arc starting at (10, 0): tangency means the arc's
    // centre must lie on the perpendicular through the shared point.
    let mut s = Sketch::new();
    let (l, a, b) = line(&mut s, v(0.0, 0.0), v(10.0, 0.0));
    s.add_constraint(Constraint::Fix(a)).unwrap();
    s.add_constraint(Constraint::Fix(b)).unwrap();
    let c = s.add_point(v(11.0, 3.0));
    let e = s.add_point(v(10.0, 6.0));
    let arc = s.add_arc(c, b, e).unwrap();
    s.add_constraint(Constraint::Tangent(l, arc)).unwrap();
    check_jacobian(&s);
    let rep = s.solve().unwrap();
    assert_relative_eq!(pos(&s, c).x, 10.0, epsilon = 1e-8);
    assert_eq!(
        rep.degrees_of_freedom, 2,
        "centre height and arc sweep remain"
    );
    // Two arcs sharing an endpoint: centres collinear with it.
    let mut s = Sketch::new();
    let p = s.add_point(v(0.0, 0.0));
    let c1 = s.add_point(v(-2.0, 0.0));
    let c2 = s.add_point(v(3.0, 0.5));
    let e1 = s.add_point(v(-2.0, 2.0));
    let e2 = s.add_point(v(3.0, -2.5));
    let a1 = s.add_arc(c1, e1, p).unwrap();
    let a2 = s.add_arc(c2, p, e2).unwrap();
    s.add_constraint(Constraint::Fix(p)).unwrap();
    s.add_constraint(Constraint::Fix(c1)).unwrap();
    s.add_constraint(Constraint::Tangent(a1, a2)).unwrap();
    check_jacobian(&s);
    s.solve().unwrap();
    assert_relative_eq!(pos(&s, c2).y, 0.0, epsilon = 1e-8);
}

#[test]
fn tangent_circle_circle_external_and_internal() {
    let mut s = Sketch::new();
    let c1 = shapes::circle_center(&mut s, v(0.0, 0.0), 3.0);
    let c2 = shapes::circle_center(&mut s, v(5.5, 0.0), 2.0);
    s.add_constraint(Constraint::Fix(c1.center)).unwrap();
    s.add_constraint(Constraint::Radius {
        curve: c1.circle,
        value: 3.0,
    })
    .unwrap();
    s.add_constraint(Constraint::Radius {
        curve: c2.circle,
        value: 2.0,
    })
    .unwrap();
    s.add_constraint(Constraint::Tangent(c1.circle, c2.circle))
        .unwrap();
    check_jacobian(&s);
    s.solve().unwrap();
    assert_relative_eq!(pos(&s, c2.center).length(), 5.0, epsilon = 1e-9);

    let mut s = Sketch::new();
    let c1 = shapes::circle_center(&mut s, v(0.0, 0.0), 3.0);
    let c2 = shapes::circle_center(&mut s, v(0.8, 0.0), 2.0);
    s.add_constraint(Constraint::Fix(c1.center)).unwrap();
    s.add_constraint(Constraint::Radius {
        curve: c1.circle,
        value: 3.0,
    })
    .unwrap();
    s.add_constraint(Constraint::Radius {
        curve: c2.circle,
        value: 2.0,
    })
    .unwrap();
    s.add_constraint(Constraint::Tangent(c1.circle, c2.circle))
        .unwrap();
    s.solve().unwrap();
    assert_relative_eq!(pos(&s, c2.center).length(), 1.0, epsilon = 1e-9);
}

#[test]
fn fix_holds_point_and_removes_dof() {
    let mut s = Sketch::new();
    let a = s.add_point(v(1.0, 2.0));
    let b = s.add_point(v(5.0, 5.0));
    s.add_constraint(Constraint::Fix(a)).unwrap();
    s.add_constraint(Constraint::Distance { a, b, value: 1.0 })
        .unwrap();
    let r = s.solve().unwrap();
    assert_eq!(pos(&s, a), v(1.0, 2.0));
    assert_relative_eq!(pos(&s, b).distance(v(1.0, 2.0)), 1.0, epsilon = 1e-9);
    assert_eq!(r.degrees_of_freedom, 1);
}

#[test]
fn midpoint_symmetric_concentric() {
    let mut s = Sketch::new();
    let (l, a, b) = line(&mut s, v(0.0, 0.0), v(4.0, 2.0));
    s.add_constraint(Constraint::Fix(a)).unwrap();
    s.add_constraint(Constraint::Fix(b)).unwrap();
    let m = s.add_point(v(3.0, 3.0));
    s.add_constraint(Constraint::Midpoint { point: m, line: l })
        .unwrap();
    let (axis, xa, xb) = line(&mut s, v(0.0, 10.0), v(10.0, 10.0));
    s.add_constraint(Constraint::Fix(xa)).unwrap();
    s.add_constraint(Constraint::Fix(xb)).unwrap();
    let p = s.add_point(v(3.0, 12.0));
    let q = s.add_point(v(4.0, 7.0));
    s.add_constraint(Constraint::Fix(p)).unwrap();
    s.add_constraint(Constraint::Symmetric { a: p, b: q, axis })
        .unwrap();
    let c1 = shapes::circle_center(&mut s, v(20.0, 0.0), 1.0);
    let c2 = shapes::circle_center(&mut s, v(21.0, 1.0), 2.0);
    s.add_constraint(Constraint::Fix(c1.center)).unwrap();
    s.add_constraint(Constraint::Concentric(c1.circle, c2.circle))
        .unwrap();
    check_jacobian(&s);
    s.solve().unwrap();
    assert!(pos(&s, m).distance(v(2.0, 1.0)) < 1e-9);
    assert!(pos(&s, q).distance(v(3.0, 8.0)) < 1e-9);
    assert!(pos(&s, c2.center).distance(v(20.0, 0.0)) < 1e-9);
}

#[test]
fn distance_dimensions() {
    let mut s = Sketch::new();
    let a = s.add_point(v(0.0, 0.0));
    let b = s.add_point(v(3.0, 1.0));
    s.add_constraint(Constraint::Fix(a)).unwrap();
    s.add_constraint(Constraint::Distance { a, b, value: 5.0 })
        .unwrap();
    let (l, la, lb) = line(&mut s, v(0.0, 10.0), v(10.0, 10.0));
    s.add_constraint(Constraint::Fix(la)).unwrap();
    s.add_constraint(Constraint::Fix(lb)).unwrap();
    let p = s.add_point(v(4.0, 7.0));
    s.add_constraint(Constraint::Distance {
        a: p,
        b: l,
        value: 2.0,
    })
    .unwrap();
    let h1 = s.add_point(v(20.0, 0.0));
    let h2 = s.add_point(v(21.0, 0.5));
    s.add_constraint(Constraint::Fix(h1)).unwrap();
    s.add_constraint(Constraint::HorizontalDistance {
        a: h1,
        b: h2,
        value: 4.0,
    })
    .unwrap();
    s.add_constraint(Constraint::VerticalDistance {
        a: h2,
        b: h1,
        value: 3.0,
    })
    .unwrap();
    check_jacobian(&s);
    s.solve().unwrap();
    assert_relative_eq!(pos(&s, b).length(), 5.0, epsilon = 1e-9);
    // Point below the line stays below it.
    assert_relative_eq!(pos(&s, p).y, 8.0, epsilon = 1e-9);
    assert_relative_eq!(pos(&s, h2).x, 24.0, epsilon = 1e-9);
    assert_relative_eq!(pos(&s, h2).y, 3.0, epsilon = 1e-9);
}

#[test]
fn radius_diameter_angle() {
    let mut s = Sketch::new();
    let c = shapes::circle_center(&mut s, v(0.0, 0.0), 1.0);
    s.add_constraint(Constraint::Diameter {
        curve: c.circle,
        value: 9.0,
    })
    .unwrap();
    let arc = shapes::arc_center(&mut s, v(10.0, 0.0), v(11.0, 0.0), v(10.0, 1.0));
    s.add_constraint(Constraint::Fix(arc.center)).unwrap();
    s.add_constraint(Constraint::Radius {
        curve: arc.arc,
        value: 2.0,
    })
    .unwrap();
    let (l1, a1, b1) = line(&mut s, v(0.0, 20.0), v(5.0, 20.0));
    s.add_constraint(Constraint::Fix(a1)).unwrap();
    s.add_constraint(Constraint::Fix(b1)).unwrap();
    let (l2, a2, b2) = line(&mut s, v(0.0, 20.0), v(5.0, 21.0));
    s.add_constraint(Constraint::Fix(a2)).unwrap();
    s.add_constraint(Constraint::Angle {
        a: l1,
        b: l2,
        value: FRAC_PI_4,
    })
    .unwrap();
    check_jacobian(&s);
    s.solve().unwrap();
    assert_relative_eq!(radius_of(&s, c.circle), 4.5, epsilon = 1e-9);
    assert_relative_eq!(radius_of(&s, arc.arc), 2.0, epsilon = 1e-9);
    assert_relative_eq!(pos(&s, arc.end).distance(v(10.0, 0.0)), 2.0, epsilon = 1e-9);
    let d = pos(&s, b2) - pos(&s, a2);
    assert_relative_eq!(d.y.atan2(d.x), FRAC_PI_4, epsilon = 1e-8);
}

#[test]
fn set_dimension_value_drives_resolve() {
    let mut s = Sketch::new();
    let a = s.add_point(v(0.0, 0.0));
    let b = s.add_point(v(3.0, 0.0));
    s.add_constraint(Constraint::Fix(a)).unwrap();
    let d = s
        .add_constraint(Constraint::Distance { a, b, value: 3.0 })
        .unwrap();
    s.solve().unwrap();
    s.set_dimension_value(d, 7.0).unwrap();
    assert_eq!(s.constraint(d).unwrap().dimension_value(), Some(7.0));
    s.solve().unwrap();
    assert_relative_eq!(pos(&s, b).length(), 7.0, epsilon = 1e-9);
    let ab = s.add_line(a, b).unwrap();
    let h = s.add_constraint(Constraint::Horizontal(ab)).unwrap();
    assert!(matches!(
        s.set_dimension_value(h, 1.0),
        Err(SketchError::NotADimension(_))
    ));
    assert!(!s.constraint(h).unwrap().is_dimension());
}

// ----- solver: DOF, drag, conflicts -----------------------------------------------------

fn dimensioned_rectangle(fix: bool) -> (Sketch, shapes::Rectangle) {
    let mut s = Sketch::new();
    let r = shapes::rectangle_two_point(&mut s, v(0.0, 0.0), v(10.0, 5.0));
    s.add_constraint(Constraint::Distance {
        a: r.corners[0],
        b: r.corners[1],
        value: 10.0,
    })
    .unwrap();
    s.add_constraint(Constraint::Distance {
        a: r.corners[1],
        b: r.corners[2],
        value: 5.0,
    })
    .unwrap();
    if fix {
        s.add_constraint(Constraint::Fix(r.corners[0])).unwrap();
    }
    (s, r)
}

#[test]
fn fully_dimensioned_rectangle_has_zero_dof() {
    let (mut s, _) = dimensioned_rectangle(true);
    let r = s.solve().unwrap();
    assert_eq!(r.degrees_of_freedom, 0);
}

#[test]
fn under_constrained_rectangle_reports_remaining_dof() {
    let (mut s, _) = dimensioned_rectangle(false);
    assert_eq!(
        s.solve().unwrap().degrees_of_freedom,
        2,
        "free to translate"
    );
    let mut s = Sketch::new();
    shapes::rectangle_two_point(&mut s, v(0.0, 0.0), v(10.0, 5.0));
    assert_eq!(
        s.solve().unwrap().degrees_of_freedom,
        4,
        "translate + resize"
    );
}

#[test]
fn free_entities_are_named_not_just_counted() {
    // A rectangle pinned by one corner and two dimensions has nothing loose left, even
    // though every other corner still owns parameters: they are determined through the
    // constraint chain, which is exactly what the null space has to see.
    let (mut s, _) = dimensioned_rectangle(true);
    assert!(s.solve().unwrap().under_constrained.is_empty());

    // Without the fix, the whole rectangle can translate: every corner is loose.
    let (mut s, loose) = dimensioned_rectangle(false);
    let free = s.solve().unwrap().under_constrained;
    for corner in loose.corners {
        assert!(free.contains(&corner), "corner should be free to translate");
    }

    // A stray point added to the pinned rectangle is the only thing free.
    let (mut s2, _) = dimensioned_rectangle(true);
    let stray = s2.add_point(v(3.0, 3.0));
    assert_eq!(s2.solve().unwrap().under_constrained, vec![stray]);
}

#[test]
fn free_entities_include_an_undimensioned_radius_only() {
    // The circle's centre is fixed and its radius is not, so the report must name the
    // circle (which owns the radius) and not its centre point.
    let mut s = Sketch::new();
    let c = s.add_point(v(0.0, 0.0));
    let circle = s.add_circle(c, 4.0).unwrap();
    s.add_constraint(Constraint::Fix(c)).unwrap();
    assert_eq!(s.solve().unwrap().under_constrained, vec![circle]);
    s.add_constraint(Constraint::Radius {
        curve: circle,
        value: 4.0,
    })
    .unwrap();
    let report = s.solve().unwrap();
    assert_eq!(report.degrees_of_freedom, 0);
    assert!(report.under_constrained.is_empty());
}

#[test]
fn a_point_on_a_line_is_free_along_it() {
    // One degree of freedom, and it belongs to the point that slides: the line's own ends
    // are fixed, so naming them would send the user after the wrong geometry.
    let mut s = Sketch::new();
    let (l, a, b) = line(&mut s, v(0.0, 0.0), v(10.0, 0.0));
    s.add_constraint(Constraint::Fix(a)).unwrap();
    s.add_constraint(Constraint::Fix(b)).unwrap();
    let p = s.add_point(v(5.0, 0.0));
    s.add_constraint(Constraint::Coincident {
        point: p,
        target: l,
    })
    .unwrap();
    let report = s.solve().unwrap();
    assert_eq!(report.degrees_of_freedom, 1);
    assert_eq!(report.under_constrained, vec![p]);
}

#[test]
fn redundant_but_consistent_constraints_still_solve() {
    let (mut s, r) = dimensioned_rectangle(true);
    s.add_constraint(Constraint::Horizontal(r.lines[0]))
        .unwrap();
    s.add_constraint(Constraint::Parallel(r.lines[0], r.lines[2]))
        .unwrap();
    let rep = s.solve().unwrap();
    assert_eq!(rep.degrees_of_freedom, 0);
}

#[test]
fn conflicting_constraints_do_not_converge() {
    let mut s = Sketch::new();
    let a = s.add_point(v(0.0, 0.0));
    let b = s.add_point(v(3.0, 0.0));
    s.add_constraint(Constraint::Distance { a, b, value: 3.0 })
        .unwrap();
    s.add_constraint(Constraint::Distance { a, b, value: 5.0 })
        .unwrap();
    match s.solve() {
        Err(SolveError::DidNotConverge { residual, .. }) => assert!(residual > 0.1),
        other => panic!("expected non-convergence, got {other:?}"),
    }
    // Geometry is left untouched after a failed solve.
    assert_eq!(pos(&s, b), v(3.0, 0.0));
}

/// Which constraints disagree is the only actionable thing about a conflict, and the
/// solver has it in the residual vector already.
#[test]
fn a_conflict_names_the_constraints_that_disagree() {
    let mut s = Sketch::new();
    let a = s.add_point(v(0.0, 0.0));
    let b = s.add_point(v(3.0, 0.0));
    let c = s.add_point(v(3.0, 4.0));
    let d = s.add_point(v(3.0, 9.0));
    let side = s.add_line(c, d).unwrap();
    // Innocent bystander: satisfied, and satisfiable whatever the others ask for.
    let vertical = s.add_constraint(Constraint::Vertical(side)).unwrap();
    let three = s
        .add_constraint(Constraint::Distance { a, b, value: 3.0 })
        .unwrap();
    let five = s
        .add_constraint(Constraint::Distance { a, b, value: 5.0 })
        .unwrap();
    match s.solve() {
        Err(SolveError::DidNotConverge { conflicting, .. }) => {
            assert!(conflicting.contains(&three), "{conflicting:?}");
            assert!(conflicting.contains(&five), "{conflicting:?}");
            assert!(
                !conflicting.contains(&vertical),
                "a satisfied constraint is not to blame: {conflicting:?}"
            );
        }
        other => panic!("expected a conflict, got {other:?}"),
    }
}

#[test]
fn all_fixed_conflict_reports_immediately() {
    let mut s = Sketch::new();
    let a = s.add_point(v(0.0, 0.0));
    let b = s.add_point(v(3.0, 0.0));
    s.add_constraint(Constraint::Fix(a)).unwrap();
    s.add_constraint(Constraint::Fix(b)).unwrap();
    s.add_constraint(Constraint::Distance { a, b, value: 5.0 })
        .unwrap();
    assert!(matches!(
        s.solve(),
        Err(SolveError::DidNotConverge { iterations: 0, .. })
    ));
}

#[test]
fn drag_free_point_reaches_target() {
    let mut s = Sketch::new();
    let p = s.add_point(v(0.0, 0.0));
    s.drag(p, v(3.0, 4.0)).unwrap();
    assert!(pos(&s, p).distance(v(3.0, 4.0)) < 1e-6);
}

#[test]
fn drag_respects_constraints() {
    let mut s = Sketch::new();
    let (l, a, b) = line(&mut s, v(0.0, 0.0), v(10.0, 0.0));
    s.add_constraint(Constraint::Fix(a)).unwrap();
    s.add_constraint(Constraint::Fix(b)).unwrap();
    let p = s.add_point(v(5.0, 0.0));
    s.add_constraint(Constraint::Coincident {
        point: p,
        target: l,
    })
    .unwrap();
    let rep = s.drag(p, v(7.0, 3.0)).unwrap();
    assert!(rep.converged);
    assert_relative_eq!(pos(&s, p).y, 0.0, epsilon = 1e-9);
    assert_relative_eq!(pos(&s, p).x, 7.0, epsilon = 1e-6);
    // Dragging a corner of a rectangle keeps it a rectangle.
    let mut s = Sketch::new();
    let r = shapes::rectangle_two_point(&mut s, v(0.0, 0.0), v(10.0, 5.0));
    s.add_constraint(Constraint::Fix(r.corners[0])).unwrap();
    s.drag(r.corners[2], v(12.0, 8.0)).unwrap();
    assert!(pos(&s, r.corners[2]).distance(v(12.0, 8.0)) < 1e-6);
    assert!(pos(&s, r.corners[1]).distance(v(12.0, 0.0)) < 1e-6);
    assert!(pos(&s, r.corners[3]).distance(v(0.0, 8.0)) < 1e-6);
    assert!(matches!(
        s.drag(r.lines[0], v(0.0, 0.0)),
        Err(SolveError::NotAPoint(_))
    ));
}

// ----- validation and editing -----------------------------------------------------------

#[test]
fn constraint_validation_rejects_wrong_kinds() {
    let mut s = Sketch::new();
    let (l, a, _) = line(&mut s, v(0.0, 0.0), v(1.0, 0.0));
    let c = shapes::circle_center(&mut s, v(5.0, 5.0), 1.0);
    assert!(matches!(
        s.add_constraint(Constraint::Horizontal(a)),
        Err(SketchError::WrongEntityKind { .. })
    ));
    assert!(matches!(
        s.add_constraint(Constraint::Equal(l, c.circle)),
        Err(SketchError::WrongEntityKind { .. })
    ));
    assert!(matches!(
        s.add_constraint(Constraint::Tangent(l, a)),
        Err(SketchError::WrongEntityKind { .. })
    ));
    assert!(matches!(
        s.add_constraint(Constraint::Radius {
            curve: l,
            value: 1.0
        }),
        Err(SketchError::WrongEntityKind { .. })
    ));
    assert!(s.add_constraint(Constraint::Tangent(c.circle, l)).is_ok());
    assert!(matches!(
        s.add_line(a, c.circle),
        Err(SketchError::WrongEntityKind { .. })
    ));
    s.remove_entity(c.circle);
    assert!(matches!(
        s.add_constraint(Constraint::Fix(c.circle)),
        Err(SketchError::UnknownEntity(_))
    ));
    assert!(matches!(
        s.add_line(a, c.circle),
        Err(SketchError::UnknownEntity(_))
    ));
    assert!(matches!(
        s.add_circle(a, -1.0),
        Err(SketchError::InvalidArgument(_))
    ));
}

#[test]
fn remove_entity_cascades_to_dependents_and_constraints() {
    let mut s = Sketch::new();
    let (l, a, b) = line(&mut s, v(0.0, 0.0), v(1.0, 0.0));
    let arc = shapes::arc_center(&mut s, v(0.0, 0.0), v(2.0, 0.0), v(0.0, 2.0));
    let c = s.add_constraint(Constraint::Horizontal(l)).unwrap();
    let keep = s.add_constraint(Constraint::Fix(arc.center)).unwrap();
    s.remove_entity(a);
    assert!(s.entity(a).is_none());
    assert!(s.entity(l).is_none(), "line depended on the removed point");
    assert!(s.entity(b).is_some());
    assert!(s.constraint(c).is_none());
    assert!(s.constraint(keep).is_some());
    s.remove_entity(arc.start);
    assert!(s.entity(arc.arc).is_none());
    assert_eq!(s.entities().count(), 3);
}

#[test]
fn serde_round_trip() {
    let (mut s, r) = dimensioned_rectangle(true);
    shapes::slot_center_to_center(&mut s, v(20.0, 0.0), v(30.0, 0.0), 4.0);
    let anchor = s.add_point(v(0.0, -5.0));
    s.add_text(anchor, "hi", 3.0, 0.1).unwrap();
    s.set_construction(r.lines[0], true).unwrap();
    let json = serde_json::to_string(&s).unwrap();
    let mut back: Sketch = serde_json::from_str(&json).unwrap();
    assert_eq!(s.entities().count(), back.entities().count());
    assert_eq!(s.constraints().count(), back.constraints().count());
    assert!(back.entity(r.lines[0]).unwrap().construction);
    assert_eq!(back.point_pos(r.corners[2]), Some(v(10.0, 5.0)));
    assert_eq!(
        back.solve().unwrap().degrees_of_freedom,
        s.solve().unwrap().degrees_of_freedom
    );
}

// ----- shapes ---------------------------------------------------------------------------

#[test]
fn rectangle_builders() {
    let mut s = Sketch::new();
    let r = shapes::rectangle_two_point(&mut s, v(10.0, 5.0), v(0.0, 0.0));
    assert_eq!(s.entities().count(), 8);
    assert_eq!(s.constraints().count(), 4);
    assert_eq!(pos(&s, r.corners[0]), v(0.0, 0.0));
    assert_eq!(pos(&s, r.corners[2]), v(10.0, 5.0));
    let mut s = Sketch::new();
    let r = shapes::rectangle_center(&mut s, v(5.0, 5.0), v(8.0, 7.0));
    assert_eq!(s.entities().count(), 11, "8 + centre + 2 diagonals");
    assert_eq!(s.constraints().count(), 6);
    assert_eq!(pos(&s, r.corners[0]), v(2.0, 3.0));
    assert_eq!(pos(&s, r.corners[2]), v(8.0, 7.0));
    let c = r.center.unwrap();
    assert!(s.entity(c).unwrap().construction);
    s.add_constraint(Constraint::Fix(c)).unwrap();
    s.drag(r.corners[2], v(9.0, 9.0)).unwrap();
    assert!(
        pos(&s, r.corners[0]).distance(v(1.0, 1.0)) < 1e-6,
        "centre stays the centre"
    );
    assert_eq!(
        s.profiles(&Tessellation::default()).len(),
        1,
        "construction diagonals ignored"
    );
}

#[test]
fn circle_builders() {
    let mut s = Sketch::new();
    let c = shapes::circle_two_point(&mut s, v(0.0, 0.0), v(4.0, 0.0));
    assert_eq!(pos(&s, c.center), v(2.0, 0.0));
    assert_relative_eq!(radius_of(&s, c.circle), 2.0);
    let c = shapes::circle_three_point(&mut s, v(1.0, 0.0), v(0.0, 1.0), v(-1.0, 0.0)).unwrap();
    assert!(pos(&s, c.center).length() < 1e-12);
    assert_relative_eq!(radius_of(&s, c.circle), 1.0);
    assert!(matches!(
        shapes::circle_three_point(&mut s, v(0.0, 0.0), v(1.0, 1.0), v(2.0, 2.0)),
        Err(SketchError::DegenerateGeometry(_))
    ));
    assert_eq!(s.entities().count(), 4);
}

#[test]
fn polygon_builder() {
    let mut s = Sketch::new();
    let p = shapes::polygon_center(&mut s, v(0.0, 0.0), v(2.0, 0.0), 6).unwrap();
    assert_eq!(p.vertices.len(), 6);
    assert_eq!(p.edges.len(), 6);
    assert_eq!(s.entities().count(), 14);
    assert_eq!(s.constraints().count(), 11, "6 on-circle + 5 equal");
    assert!(s.entity(p.circle).unwrap().construction);
    for i in 0..6 {
        let e = pos(&s, p.vertices[i]).distance(pos(&s, p.vertices[(i + 1) % 6]));
        assert_relative_eq!(e, 2.0, epsilon = 1e-12);
    }
    assert!(shapes::polygon_center(&mut s, v(0.0, 0.0), v(1.0, 0.0), 2).is_err());
    let rep = s.solve().unwrap();
    assert_eq!(rep.degrees_of_freedom, 4, "position, radius, rotation");
    assert_eq!(s.profiles(&Tessellation::default()).len(), 1);
}

#[test]
fn slot_builder() {
    let mut s = Sketch::new();
    let slot = shapes::slot_center_to_center(&mut s, v(0.0, 0.0), v(10.0, 0.0), 4.0);
    assert_eq!(
        s.entities().count(),
        11,
        "6 points, 2 lines, 2 arcs, centre line"
    );
    assert_eq!(s.constraints().count(), 5);
    assert!(s.entity(slot.center_line).unwrap().construction);
    assert_relative_eq!(radius_of(&s, slot.arcs[0]), 2.0);
    let rep = s.solve().unwrap();
    assert_eq!(
        rep.iterations, 0,
        "built shape already satisfies its constraints"
    );
    assert_eq!(rep.degrees_of_freedom, 5);
}

#[test]
fn arc_builders() {
    let mut s = Sketch::new();
    let a = shapes::arc_three_point(&mut s, v(1.0, 0.0), v(0.0, 1.0), v(-1.0, 0.0)).unwrap();
    assert!(pos(&s, a.start).distance(v(1.0, 0.0)) < 1e-12);
    assert!(pos(&s, a.end).distance(v(-1.0, 0.0)) < 1e-12);
    // Clockwise request: endpoints are swapped so the entity stays CCW.
    let b = shapes::arc_three_point(&mut s, v(1.0, 0.0), v(0.0, -1.0), v(-1.0, 0.0)).unwrap();
    assert!(pos(&s, b.start).distance(v(-1.0, 0.0)) < 1e-12);
    assert!(pos(&s, b.end).distance(v(1.0, 0.0)) < 1e-12);
    let c = shapes::arc_center(&mut s, v(0.0, 0.0), v(2.0, 0.0), v(0.0, 5.0));
    assert!(
        pos(&s, c.end).distance(v(0.0, 2.0)) < 1e-12,
        "end projected onto the circle"
    );
    assert!(shapes::arc_three_point(&mut s, v(0.0, 0.0), v(1.0, 0.0), v(2.0, 0.0)).is_err());
}

#[test]
fn polyline_builder() {
    let mut s = Sketch::new();
    let p = shapes::polyline(&mut s, &[v(0.0, 0.0), v(1.0, 0.0), v(1.0, 1.0)], false);
    assert_eq!(p.lines.len(), 2);
    let q = shapes::polyline(&mut s, &[v(5.0, 0.0), v(6.0, 0.0), v(6.0, 1.0)], true);
    assert_eq!(q.lines.len(), 3);
    assert_eq!(s.entities().count(), 11);
}

// ----- profiles -------------------------------------------------------------------------

fn tess() -> Tessellation {
    Tessellation::default()
}

fn total_area(p: &[Profile]) -> f64 {
    p.iter().map(Profile::area).sum()
}

#[test]
fn profile_rectangle() {
    let mut s = Sketch::new();
    let r = shapes::rectangle_two_point(&mut s, v(0.0, 0.0), v(10.0, 5.0));
    let p = s.profiles(&tess());
    assert_eq!(p.len(), 1);
    assert_relative_eq!(p[0].outer.signed_area(), 50.0);
    assert!(p[0].holes.is_empty());
    assert_eq!(p[0].outer.points.len(), 4);
    assert_eq!(p[0].outer.segments.len(), 4);
    let mut tagged: Vec<EntityId> = p[0].outer.segments.iter().map(|s| s.curve).collect();
    tagged.sort();
    let mut lines = r.lines.to_vec();
    lines.sort();
    assert_eq!(tagged, lines);
}

#[test]
fn profile_rectangle_with_hole_and_disjoint_neighbour() {
    let mut s = Sketch::new();
    shapes::rectangle_two_point(&mut s, v(0.0, 0.0), v(10.0, 10.0));
    let c = shapes::circle_center(&mut s, v(5.0, 5.0), 1.0);
    shapes::rectangle_two_point(&mut s, v(20.0, 0.0), v(25.0, 5.0));
    let p = s.profiles(&tess());
    // The rectangle-with-a-hole, the disc inside it, and the neighbour.
    assert_eq!(p.len(), 3);
    let big = p
        .iter()
        .find(|p| p.holes.len() == 1)
        .expect("one profile has the hole");
    assert_relative_eq!(big.outer.signed_area(), 100.0);
    assert!(big.holes[0].signed_area() < 0.0, "holes are clockwise");
    assert_relative_eq!(big.holes[0].signed_area(), -PI, epsilon = 0.02);
    assert!(
        big.holes[0]
            .segments
            .iter()
            .all(|seg| seg.curve == c.circle)
    );
    assert!(!big.contains(v(5.0, 5.0)));
    assert!(big.contains(v(1.0, 1.0)));
    // The disc is a region of its own, so it can be extruded without the rectangle.
    let disc = p
        .iter()
        .find(|p| p.contains(v(5.0, 5.0)))
        .expect("the circle bounds a region too");
    assert_relative_eq!(disc.area(), PI, epsilon = 0.02);
    let small = p
        .iter()
        .find(|p| p.contains(v(22.0, 2.0)))
        .expect("the disjoint neighbour");
    assert_relative_eq!(small.area(), 25.0);
}

#[test]
fn profile_slot_mixes_lines_and_arcs() {
    let mut s = Sketch::new();
    let slot = shapes::slot_center_to_center(&mut s, v(0.0, 0.0), v(10.0, 0.0), 4.0);
    let p = s.profiles(&tess());
    assert_eq!(p.len(), 1);
    let expected = 10.0 * 4.0 + PI * 4.0;
    assert_relative_eq!(p[0].area(), expected, epsilon = 0.1);
    let arc_edges = p[0]
        .outer
        .segments
        .iter()
        .filter(|s| matches!(s.kind, crate::SegmentKind::Arc { ccw: true, .. }))
        .count();
    let line_edges = p[0]
        .outer
        .segments
        .iter()
        .filter(|s| s.kind == crate::SegmentKind::Line)
        .count();
    assert_eq!(line_edges, 2);
    assert!(arc_edges >= 36, "two semicircles at ≤10° per segment");
    assert!(
        p[0].outer
            .segments
            .iter()
            .any(|seg| seg.curve == slot.arcs[0])
    );
    assert!(
        p[0].outer
            .segments
            .iter()
            .any(|seg| seg.curve == slot.lines[1])
    );
}

#[test]
fn profile_ring_and_island() {
    let mut s = Sketch::new();
    shapes::circle_center(&mut s, v(0.0, 0.0), 5.0);
    shapes::circle_center(&mut s, v(0.0, 0.0), 3.0);
    let p = s.profiles(&tess());
    assert_eq!(p.len(), 2, "the ring and the inner disc are both regions");
    let ring = p.iter().find(|p| p.holes.len() == 1).unwrap();
    assert_relative_eq!(ring.area(), PI * (25.0 - 9.0), epsilon = 0.2);
    let disc = p.iter().find(|p| p.holes.is_empty()).unwrap();
    assert_relative_eq!(disc.area(), PI * 9.0, epsilon = 0.2);
    // Three nested rectangles: each is its own region, each punched by the next one in.
    let mut s = Sketch::new();
    shapes::rectangle_two_point(&mut s, v(0.0, 0.0), v(30.0, 30.0));
    shapes::rectangle_two_point(&mut s, v(5.0, 5.0), v(25.0, 25.0));
    shapes::rectangle_two_point(&mut s, v(10.0, 10.0), v(20.0, 20.0));
    let p = s.profiles(&tess());
    assert_eq!(p.len(), 3);
    assert_relative_eq!(
        total_area(&p),
        (900.0 - 400.0) + (400.0 - 100.0) + 100.0,
        epsilon = 1e-6
    );
}

#[test]
fn profile_shared_edge_faces() {
    // Two squares sharing an edge form one component with two faces.
    let mut s = Sketch::new();
    let pl = shapes::polyline(
        &mut s,
        &[v(0.0, 0.0), v(10.0, 0.0), v(10.0, 10.0), v(0.0, 10.0)],
        true,
    );
    let mid_a = s.add_point(v(5.0, 0.0));
    let mid_b = s.add_point(v(5.0, 10.0));
    s.add_line(mid_a, mid_b).unwrap();
    let _ = pl;
    let p = s.profiles(&tess());
    // The divider's endpoints lie mid-edge, so the edges it meets are split there and the
    // square becomes two faces without the user drawing the split themselves.
    assert_eq!(p.len(), 2);
    assert_relative_eq!(total_area(&p), 100.0);
    let mut s = Sketch::new();
    shapes::polyline(
        &mut s,
        &[
            v(0.0, 0.0),
            v(5.0, 0.0),
            v(10.0, 0.0),
            v(10.0, 10.0),
            v(5.0, 10.0),
            v(0.0, 10.0),
        ],
        true,
    );
    let a = s.add_point(v(5.0, 0.0));
    let b = s.add_point(v(5.0, 10.0));
    s.add_line(a, b).unwrap();
    let p = s.profiles(&tess());
    assert_eq!(p.len(), 2);
    assert_relative_eq!(total_area(&p), 100.0);
    for prof in &p {
        assert!(prof.outer.signed_area() > 0.0);
        assert_relative_eq!(prof.area(), 50.0);
    }
}

#[test]
fn crossing_curves_split_into_every_enclosed_region() {
    // Two overlapping squares. Neither shares a single endpoint with the other, yet the
    // overlap, and each square's remainder, are all regions the user can extrude.
    let mut s = Sketch::new();
    shapes::rectangle_two_point(&mut s, v(0.0, 0.0), v(10.0, 10.0));
    shapes::rectangle_two_point(&mut s, v(6.0, 6.0), v(16.0, 16.0));
    let p = s.profiles(&tess());
    assert_eq!(p.len(), 3, "left square, overlap, right square");
    let mut areas: Vec<f64> = p.iter().map(|x| x.area()).collect();
    areas.sort_by(f64::total_cmp);
    assert_relative_eq!(areas[0], 16.0, epsilon = 1e-9);
    assert_relative_eq!(areas[1], 84.0, epsilon = 1e-9);
    assert_relative_eq!(areas[2], 84.0, epsilon = 1e-9);
    // The overlap is selectable by a point inside it, which is how the editor refers to it.
    assert!(p.iter().any(|x| x.contains(v(8.0, 8.0)) && x.area() < 20.0));
}

#[test]
fn geometry_drawn_on_top_of_itself_still_encloses_its_region() {
    // A square whose outline is traced a second time. Every edge exists twice, which used
    // to leave the face walk stepping between the copies and finding no region at all.
    let mut s = Sketch::new();
    shapes::rectangle_two_point(&mut s, v(0.0, 0.0), v(10.0, 10.0));
    shapes::rectangle_two_point(&mut s, v(0.0, 0.0), v(10.0, 10.0));
    let p = s.profiles(&tess());
    assert_eq!(
        p.len(),
        1,
        "the duplicate outline is one region, not two or none"
    );
    assert_relative_eq!(p[0].area(), 100.0, epsilon = 1e-9);
    assert_eq!(p[0].outer.points.len(), 4, "and not a doubled boundary");
}

#[test]
fn a_curve_overlapping_part_of_another_splits_both() {
    // The long line runs the width of the square along its bottom edge and past it on both
    // sides. Only the stretch they share is a duplicate; the overhangs are free ends that
    // bound nothing, so the square is still exactly one region.
    let mut s = Sketch::new();
    shapes::rectangle_two_point(&mut s, v(0.0, 0.0), v(10.0, 10.0));
    let a = s.add_point(v(-5.0, 0.0));
    let b = s.add_point(v(15.0, 0.0));
    s.add_line(a, b).unwrap();
    let p = s.profiles(&tess());
    assert_eq!(p.len(), 1);
    assert_relative_eq!(p[0].area(), 100.0, epsilon = 1e-9);
}

/// A divider that stops fractionally short of the edge it was drawn to meet still
/// splits the region in two.
///
/// `segment_crossing` is exact, so without the T-junction pass the top edge is never cut
/// and the two halves trace as one self-overlapping loop whose area means nothing. The
/// solver only converges to its tolerance, so a near miss like this is the ordinary
/// state of a re-dimensioned sketch, not a pathological case.
#[test]
fn a_divider_stopping_short_of_an_edge_still_splits_the_region() {
    for gap in [5e-7, -5e-7] {
        let mut s = Sketch::new();
        shapes::rectangle_two_point(&mut s, v(0.0, 0.0), v(10.0, 4.0));
        // Short of the top edge (or through it) by far less than JOIN_TOL, and far more
        // than the exact crossing test tolerates — which is nothing.
        line(&mut s, v(5.0, 0.0), v(5.0, 4.0 - gap));
        let p = s.profiles(&tess());
        assert_eq!(p.len(), 2, "both halves are regions (gap {gap:e})");
        assert_relative_eq!(total_area(&p), 40.0, epsilon = 1e-4);
        for half in &p {
            assert_relative_eq!(half.area(), 20.0, epsilon = 1e-4);
        }
    }
}

/// The snap is a tolerance, not a repair: a divider that genuinely falls short leaves the
/// region open, because closing a visible gap would invent geometry the user did not draw.
#[test]
fn a_divider_with_a_real_gap_does_not_split_the_region() {
    let mut s = Sketch::new();
    shapes::rectangle_two_point(&mut s, v(0.0, 0.0), v(10.0, 4.0));
    line(&mut s, v(5.0, 0.0), v(5.0, 3.99));
    let p = s.profiles(&tess());
    assert_eq!(p.len(), 1, "the rectangle is still one region");
    assert_relative_eq!(p[0].area(), 40.0, epsilon = 1e-9);
}

#[test]
fn a_chord_splits_a_circle_into_two_regions() {
    let mut s = Sketch::new();
    shapes::circle_center(&mut s, v(0.0, 0.0), 10.0);
    // A secant that overshoots the circle on both sides: the overhangs are trimmed away
    // and the two halves come out as regions.
    let a = s.add_point(v(-15.0, 0.0));
    let b = s.add_point(v(15.0, 0.0));
    s.add_line(a, b).unwrap();
    let p = s.profiles(&tess());
    assert_eq!(p.len(), 2);
    for half in &p {
        assert_relative_eq!(half.area(), PI * 50.0, epsilon = 0.5);
        assert!(half.outer.signed_area() > 0.0);
    }
    assert!(p.iter().any(|x| x.contains(v(0.0, 5.0))));
    assert!(p.iter().any(|x| x.contains(v(0.0, -5.0))));
    // Every boundary edge still names the curve it came from, so the kernel can give the
    // arc stretch a cylindrical face and the chord a planar one.
    let kinds: Vec<_> = p[0].outer.segments.iter().map(|x| x.kind).collect();
    assert!(kinds.iter().any(|k| matches!(k, SegmentKind::Line)));
    assert!(kinds.iter().any(|k| matches!(k, SegmentKind::Arc { .. })));
}

#[test]
fn two_crossing_circles_make_a_lens() {
    let mut s = Sketch::new();
    shapes::circle_center(&mut s, v(0.0, 0.0), 10.0);
    shapes::circle_center(&mut s, v(10.0, 0.0), 10.0);
    let p = s.profiles(&tess());
    assert_eq!(p.len(), 3, "two crescents and the lens between them");
    let lens = p
        .iter()
        .find(|x| x.contains(v(5.0, 0.0)))
        .expect("the overlap is a region");
    // Area of a symmetric lens of radius r with centres d apart.
    let (r, d) = (10.0f64, 10.0f64);
    let expected = 2.0 * r * r * (d / (2.0 * r)).acos() - d * (r * r - d * d / 4.0).sqrt();
    assert_relative_eq!(lens.area(), expected, epsilon = 0.5);
    assert_relative_eq!(total_area(&p), 2.0 * PI * 100.0 - expected, epsilon = 1.0);
}

#[test]
fn uncrossed_geometry_is_unaffected_by_splitting() {
    // A plain rectangle must still be exactly one region with four segments, not a
    // subdivision of itself: splitting only ever cuts where two curves actually meet.
    let mut s = Sketch::new();
    shapes::rectangle_two_point(&mut s, v(0.0, 0.0), v(4.0, 2.0));
    let p = s.profiles(&tess());
    assert_eq!(p.len(), 1);
    assert_eq!(p[0].outer.points.len(), 4);
    assert_relative_eq!(p[0].area(), 8.0);
    // Touching at a shared endpoint is not a crossing either.
    let mut s = Sketch::new();
    shapes::polyline(
        &mut s,
        &[v(0.0, 0.0), v(4.0, 0.0), v(4.0, 4.0), v(0.0, 4.0)],
        true,
    );
    assert_eq!(s.profiles(&tess())[0].outer.points.len(), 4);
}

#[test]
fn open_polyline_and_construction_geometry_yield_no_profile() {
    let mut s = Sketch::new();
    shapes::polyline(
        &mut s,
        &[v(0.0, 0.0), v(10.0, 0.0), v(10.0, 10.0), v(0.0, 10.0)],
        false,
    );
    assert!(s.profiles(&tess()).is_empty());
    let mut s = Sketch::new();
    let r = shapes::rectangle_two_point(&mut s, v(0.0, 0.0), v(10.0, 5.0));
    s.set_construction(r.lines[0], true).unwrap();
    assert!(s.profiles(&tess()).is_empty());
    let c = shapes::circle_center(&mut s, v(30.0, 0.0), 1.0);
    s.set_construction(c.circle, true).unwrap();
    assert!(s.profiles(&tess()).is_empty());
}

#[test]
fn profile_endpoints_joined_within_tolerance() {
    let mut s = Sketch::new();
    let a = s.add_point(v(0.0, 0.0));
    let b = s.add_point(v(10.0, 0.0));
    let c = s.add_point(v(10.0, 10.0));
    let c2 = s.add_point(v(10.0, 10.0 + 5e-5));
    let d = s.add_point(v(0.0, 10.0));
    // Offset sideways, so this really is a gap: an endpoint that overshot onto the
    // neighbouring curve would be a crossing and would close the loop.
    let d2 = s.add_point(v(-5e-3, 10.0));
    s.add_line(a, b).unwrap();
    s.add_line(b, c).unwrap();
    s.add_line(c2, d).unwrap();
    s.add_line(d2, a).unwrap();
    assert!(
        s.profiles(&tess()).is_empty(),
        "gap of 5 µm at d is too large"
    );
    s.set_point_pos(d2, v(0.0, 10.0)).unwrap();
    assert_eq!(
        s.profiles(&tess()).len(),
        1,
        "gap of 0.05 µm at c is joined"
    );
}

#[test]
fn path_orders_and_orients_curves() {
    let mut s = Sketch::new();
    let a = s.add_point(v(0.0, 0.0));
    let b = s.add_point(v(10.0, 0.0));
    let c = s.add_point(v(10.0, 10.0));
    let center = s.add_point(v(10.0, 5.0));
    let l1 = s.add_line(b, a).unwrap();
    let arc = s.add_arc(center, c, b).unwrap(); // CCW from c to b: runs clockwise b -> c on the path
    let d = s.add_point(v(0.0, 10.0));
    let l2 = s.add_line(c, d).unwrap();
    let path = s.path(&[l1, arc, l2], &tess()).unwrap();
    assert!(!path.closed);
    assert_eq!(path.points[0], v(0.0, 0.0));
    assert_eq!(*path.points.last().unwrap(), v(0.0, 10.0));
    assert_eq!(path.points.len(), path.segments.len() + 1);
    assert!(
        path.segments
            .iter()
            .any(|s| matches!(s.kind, crate::SegmentKind::Arc { ccw: false, .. }))
    );
    assert_relative_eq!(path.length(), 20.0 + PI * 5.0, epsilon = 0.05);
    let (lonely, _, _) = line(&mut s, v(50.0, 50.0), v(60.0, 50.0));
    assert!(matches!(
        s.path(&[l1, lonely], &tess()),
        Err(SketchError::PathNotConnected { index: 1 })
    ));
    let r = shapes::rectangle_two_point(&mut s, v(100.0, 0.0), v(110.0, 10.0));
    let closed = s.path(&r.lines, &tess()).unwrap();
    assert!(closed.closed);
    assert_eq!(closed.points.len(), 4);
    assert_relative_eq!(closed.signed_area(), 100.0);
}

#[test]
fn curve_polyline_and_bounds() {
    let mut s = Sketch::new();
    let c = shapes::circle_center(&mut s, v(1.0, 1.0), 2.0);
    let poly = s.curve_polyline(c.circle, &tess()).unwrap();
    assert_eq!(poly.first(), poly.last());
    assert!(poly.len() > 36);
    assert_eq!(
        s.entity_bounds(c.circle),
        Some((v(-1.0, -1.0), v(3.0, 3.0)))
    );
    let a = shapes::arc_center(&mut s, v(0.0, 0.0), v(0.0, -1.0), v(-1.0, 0.0));
    let (min, max) = s.entity_bounds(a.arc).unwrap();
    assert_relative_eq!(max.x, 1.0);
    assert_relative_eq!(max.y, 1.0);
    assert_relative_eq!(min.x, -1.0);
    assert_relative_eq!(min.y, -1.0);
    assert!(s.curve_polyline(c.center, &tess()).is_none());
}

// ----- hit testing ----------------------------------------------------------------------

#[test]
fn hit_test_orders_nearest_first_and_prefers_points() {
    let mut s = Sketch::new();
    let r = shapes::rectangle_two_point(&mut s, v(0.0, 0.0), v(10.0, 5.0));
    let hits = s.hit_test(v(0.0, 0.0), 0.5);
    assert_eq!(hits.len(), 3, "corner point and its two lines");
    assert_eq!(hits[0].entity, r.corners[0]);
    assert!(hits.iter().all(|h| h.distance < 1e-12));
    let hits = s.hit_test(v(5.0, 0.3), 0.5);
    assert_eq!(hits.len(), 1);
    assert_eq!(hits[0].entity, r.lines[0]);
    assert_relative_eq!(hits[0].distance, 0.3);
    assert!(s.hit_test(v(5.0, 2.5), 0.5).is_empty());
    let c = shapes::circle_center(&mut s, v(30.0, 0.0), 2.0);
    let hits = s.hit_test(v(32.1, 0.0), 0.5);
    assert_eq!(hits[0].entity, c.circle);
    assert_relative_eq!(hits[0].distance, 0.1, epsilon = 1e-12);
    let a = shapes::arc_center(&mut s, v(50.0, 0.0), v(51.0, 0.0), v(50.0, 1.0));
    assert_eq!(
        s.hit_test(v(51.0, -0.4), 0.5)[0].entity,
        a.start,
        "beyond the arc span the endpoint is nearest"
    );
}

/// A point sitting on a curve is the target the user is aiming at, and it is the one a
/// plain nearest-first test never gives them: the cursor is exactly on the line and only
/// nearly on the point. The bias is what makes an endpoint clickable at all.
#[test]
fn a_point_beats_the_curve_running_through_it() {
    let mut s = Sketch::new();
    let r = shapes::rectangle_two_point(&mut s, v(0.0, 0.0), v(10.0, 5.0));
    // Exactly on the bottom edge and a little short of the corner: the edge is zero away
    // and the corner is not, yet the corner is what was meant.
    let hits = s.hit_test(v(0.4, 0.0), 1.0);
    assert_eq!(
        hits[0].entity, r.corners[0],
        "a corner just off the pointer wins over the line under it"
    );
    assert_relative_eq!(hits[0].distance, 0.4);
    // Far enough along the line and the honest distance takes over again, so clicking
    // the middle of an edge still means the edge.
    assert_eq!(
        s.hit_test(v(5.0, 0.0), 1.0)[0].entity,
        r.lines[0],
        "out along the line the line wins"
    );
}

#[test]
fn hit_test_rect_window_vs_crossing() {
    let mut s = Sketch::new();
    let r = shapes::rectangle_two_point(&mut s, v(0.0, 0.0), v(10.0, 5.0));
    let window = s.hit_test_rect(v(-1.0, -1.0), v(11.0, 6.0), false);
    assert_eq!(window.len(), 8);
    let partial = s.hit_test_rect(v(-1.0, -1.0), v(5.0, 6.0), false);
    assert_eq!(partial.len(), 3, "left line and its two corners");
    assert!(partial.contains(&r.lines[3]));
    let crossing = s.hit_test_rect(v(-1.0, -1.0), v(5.0, 6.0), true);
    assert_eq!(crossing.len(), 5, "plus the top and bottom lines");
    assert!(crossing.contains(&r.lines[0]) && crossing.contains(&r.lines[2]));
    // A box wholly inside the rectangle touches nothing in crossing mode.
    assert!(s.hit_test_rect(v(2.0, 1.0), v(8.0, 4.0), true).is_empty());
    let c = shapes::circle_center(&mut s, v(30.0, 0.0), 2.0);
    assert!(
        s.hit_test_rect(v(31.0, -1.0), v(35.0, 1.0), true)
            .contains(&c.circle)
    );
    assert!(
        !s.hit_test_rect(v(31.0, -1.0), v(35.0, 1.0), false)
            .contains(&c.circle)
    );
}

// ----- text -----------------------------------------------------------------------------

fn system_font() -> Option<Font> {
    let f = Font::find_system_font();
    if f.is_none() {
        println!("note: no system TrueType font found; text tests skipped");
    }
    f
}

#[test]
fn text_without_font_only_hit_tests() {
    let mut s = Sketch::new();
    let anchor = s.add_point(v(0.0, 0.0));
    let t = s.add_text(anchor, "AB", 10.0, 0.0).unwrap();
    assert!(s.profiles(&tess()).is_empty());
    let (min, max) = s.entity_bounds(t).unwrap();
    assert_eq!(min, v(0.0, 0.0));
    assert_relative_eq!(max.y, 10.0);
    assert!(max.x > 5.0);
    assert_eq!(s.hit_test(v(3.0, 3.0), 0.1)[0].entity, t);
    assert!(s.add_text(anchor, "x", 0.0, 0.0).is_err());
}

#[test]
fn text_profiles_from_font() {
    let Some(font) = system_font() else { return };
    println!("using font family {:?}", font.family());
    let mut s = Sketch::new();
    let anchor = s.add_point(v(0.0, 0.0));
    let t = s.add_text(anchor, "O", 10.0, 0.0).unwrap();
    s.set_font(Some(std::sync::Arc::new(font)));
    let p = s.profiles(&tess());
    assert_eq!(p.len(), 1);
    assert_eq!(p[0].holes.len(), 1, "an O has one hole");
    assert!(p[0].outer.signed_area() > 0.0);
    assert!(p[0].holes[0].signed_area() < 0.0);
    assert!(p[0].outer.segments.iter().all(|seg| seg.curve == t));
    let (min, max) = s.entity_bounds(t).unwrap();
    assert!(
        p[0].outer
            .points
            .iter()
            .all(|q| q.x >= min.x - 1e-6 && q.x <= max.x + 1e-6)
    );
    assert!(max.y > 5.0 && max.y <= 10.0 + 1e-9);

    // Two glyphs: two profiles, second shifted by the advance.
    let mut s2 = s.clone();
    s2.remove_entity(t);
    let t2 = s2.add_text(anchor, "OO", 10.0, 0.0).unwrap();
    let p2 = s2.profiles(&tess());
    assert_eq!(p2.len(), 2);
    assert!(
        p2.iter()
            .all(|p| p.holes.len() == 1 && p.outer.segments[0].curve == t2)
    );

    // Text inside a rectangle punches through it and the counter comes back as an island.
    let mut s3 = s.clone();
    shapes::rectangle_two_point(&mut s3, v(-10.0, -10.0), v(20.0, 20.0));
    let p3 = s3.profiles(&tess());
    assert_eq!(p3.len(), 2);
    let rect = p3.iter().find(|p| p.outer.signed_area() > 800.0).unwrap();
    assert_eq!(rect.holes.len(), 1);

    // Rotation moves the glyph off the x axis.
    let mut s4 = s.clone();
    s4.remove_entity(t);
    s4.add_text(anchor, "I", 10.0, FRAC_PI_2).unwrap();
    let p4 = s4.profiles(&tess());
    assert_eq!(p4.len(), 1);
    assert!(
        p4[0].outer.points.iter().all(|q| q.x <= 1e-6),
        "rotated 90° the glyph lies at x ≤ 0"
    );
}

#[test]
fn font_rejects_garbage() {
    assert!(matches!(
        Font::from_bytes(vec![0, 1, 2, 3]),
        Err(SketchError::InvalidFont(_))
    ));
    assert!(matches!(
        Font::from_file("/definitely/not/here.ttf"),
        Err(SketchError::Io(_))
    ));
}

/// Moving a whole line means dragging both its points by the same offset; the
/// constraints then decide what survives. The bottom edge of a rectangle dragged up
/// shortens the rectangle rather than shearing it.
#[test]
fn drag_points_moves_a_curve_under_its_constraints() {
    let mut s = Sketch::new();
    let r = shapes::rectangle_two_point(&mut s, v(0.0, 0.0), v(10.0, 5.0));
    s.add_constraint(Constraint::Fix(r.corners[2])).unwrap();
    let goals: Vec<(EntityId, Vec2)> = s
        .entity_points(r.lines[0])
        .into_iter()
        .map(|p| (p, pos(&s, p) + v(0.0, 2.0)))
        .collect();
    assert_eq!(goals.len(), 2);
    s.drag_points(&goals).unwrap();
    assert!(pos(&s, r.corners[0]).distance(v(0.0, 2.0)) < 1e-6);
    assert!(pos(&s, r.corners[1]).distance(v(10.0, 2.0)) < 1e-6);
    assert!(pos(&s, r.corners[2]).distance(v(10.0, 5.0)) < 1e-6);
    assert!(pos(&s, r.corners[3]).distance(v(0.0, 5.0)) < 1e-6);
    assert_eq!(s.entity_points(r.corners[0]), vec![r.corners[0]]);
}

/// Where the user put a dimension's text is part of the sketch: it survives a save and
/// goes away with the dimension.
#[test]
fn dimension_labels_persist_with_the_sketch() {
    let mut s = Sketch::new();
    let (_, a, b) = line(&mut s, v(0.0, 0.0), v(10.0, 0.0));
    let d = s
        .add_constraint(Constraint::Distance { a, b, value: 10.0 })
        .unwrap();
    assert_eq!(s.dimension_label(d), None);
    s.set_dimension_label(d, v(5.0, 3.0)).unwrap();
    let json = serde_json::to_string(&s).unwrap();
    let back: Sketch = serde_json::from_str(&json).unwrap();
    assert_eq!(back.dimension_label(d), Some(v(5.0, 3.0)));
    s.remove_constraint(d);
    assert_eq!(s.dimension_label(d), None);
    assert!(s.set_dimension_label(d, v(0.0, 0.0)).is_err());
}

// ----- trimming and breaking ----------------------------------------------------------

/// The classic trim: a line crossed twice, the middle picked, two stubs left.
#[test]
fn trim_removes_the_picked_piece_between_two_crossings() {
    let mut s = Sketch::new();
    let (l, _, _) = line(&mut s, v(0.0, 0.0), v(10.0, 0.0));
    line(&mut s, v(3.0, -1.0), v(3.0, 1.0));
    line(&mut s, v(7.0, -1.0), v(7.0, 1.0));
    let pieces = crate::edit::trim(&mut s, l, v(5.0, 0.0)).unwrap();
    assert_eq!(pieces.len(), 2);
    let ends: Vec<(Vec2, Vec2)> = pieces
        .iter()
        .map(|p| s.curve_endpoints(*p).expect("line"))
        .collect();
    near(ends[0].0, v(0.0, 0.0));
    near(ends[0].1, v(3.0, 0.0));
    near(ends[1].0, v(7.0, 0.0));
    near(ends[1].1, v(10.0, 0.0));
    // The first piece keeps the original entity, so dimensions written on it survive.
    assert_eq!(pieces[0], l);
}

#[test]
fn trim_of_an_uncrossed_line_removes_all_of_it() {
    let mut s = Sketch::new();
    let (l, a, b) = line(&mut s, v(0.0, 0.0), v(4.0, 0.0));
    assert!(
        crate::edit::trim(&mut s, l, v(2.0, 0.0))
            .unwrap()
            .is_empty()
    );
    assert!(s.entity(l).is_none());
    // Its endpoints went with it: they were the line, not points the user drew.
    assert!(s.entity(a).is_none() && s.entity(b).is_none());
}

/// Trimming past the last crossing leaves one piece, the end shared with nothing.
#[test]
fn trim_of_an_end_piece_leaves_the_rest() {
    let mut s = Sketch::new();
    let (l, _, _) = line(&mut s, v(0.0, 0.0), v(10.0, 0.0));
    line(&mut s, v(4.0, -1.0), v(4.0, 1.0));
    let pieces = crate::edit::trim(&mut s, l, v(8.0, 0.0)).unwrap();
    assert_eq!(pieces, vec![l]);
    let (a, b) = s.curve_endpoints(l).unwrap();
    near(a, v(0.0, 0.0));
    near(b, v(4.0, 0.0));
}

#[test]
fn trimming_a_circle_leaves_an_arc_through_the_far_side() {
    let mut s = Sketch::new();
    let c = s.add_point(v(0.0, 0.0));
    let circle = s.add_circle(c, 5.0).unwrap();
    // A chord across the top cuts the circle at ±(x, 3).
    line(&mut s, v(-6.0, 3.0), v(6.0, 3.0));
    let pieces = crate::edit::trim(&mut s, circle, v(0.0, 5.0)).unwrap();
    assert_eq!(pieces.len(), 1);
    let arc = pieces[0];
    assert!(matches!(s.entity(arc).unwrap().entity, Entity::Arc { .. }));
    assert_relative_eq!(radius_of(&s, arc), 5.0, epsilon = 1e-9);
    // The kept arc is the long way round: it passes through the bottom of the circle.
    let poly = s.curve_polyline(arc, &Tessellation::default()).unwrap();
    assert!(poly.iter().any(|p| p.y < -4.9));
    assert!(!poly.iter().any(|p| p.y > 3.01));
}

#[test]
fn trimming_a_circle_keeps_its_diameter_dimension() {
    let mut s = Sketch::new();
    let c = s.add_point(v(0.0, 0.0));
    let circle = s.add_circle(c, 5.0).unwrap();
    s.add_constraint(Constraint::Diameter {
        curve: circle,
        value: 10.0,
    })
    .unwrap();
    line(&mut s, v(-6.0, 3.0), v(6.0, 3.0));
    let arc = crate::edit::trim(&mut s, circle, v(0.0, 5.0)).unwrap()[0];
    let kept: Vec<&Constraint> = s.constraints().map(|(_, c)| c).collect();
    assert!(
        kept.iter()
            .any(|c| matches!(**c, Constraint::Diameter { curve, .. } if curve == arc)),
        "the diameter moved onto the arc, not into the bin"
    );
}

#[test]
fn breaking_a_line_keeps_every_piece_joined() {
    let mut s = Sketch::new();
    let (l, _, _) = line(&mut s, v(0.0, 0.0), v(10.0, 0.0));
    line(&mut s, v(4.0, -1.0), v(4.0, 1.0));
    let pieces = crate::edit::break_curve(&mut s, l).unwrap();
    assert_eq!(pieces.len(), 2);
    let (_, first_end) = s.curve_endpoints(pieces[0]).unwrap();
    let (second_start, _) = s.curve_endpoints(pieces[1]).unwrap();
    near(first_end, v(4.0, 0.0));
    near(second_start, v(4.0, 0.0));
    // Joined by sharing the point entity, so dragging the cut moves both sides.
    let ends = |id| match s.entity(id).unwrap().entity {
        Entity::Line { start, end } => (start, end),
        _ => panic!("line"),
    };
    assert_eq!(ends(pieces[0]).1, ends(pieces[1]).0);
}

/// A rectangle with a line across it: trimming the crossing piece of the rectangle's
/// top edge opens the region, and trimming the overhangs of the crossing line closes it
/// again as two regions. This is the whole point of trim, so it is tested end to end.
#[test]
fn trim_reshapes_the_regions_a_sketch_encloses() {
    let mut s = Sketch::new();
    let r = shapes::rectangle_two_point(&mut s, v(0.0, 0.0), v(10.0, 6.0));
    let _ = r;
    let (divider, _, _) = line(&mut s, v(5.0, -2.0), v(5.0, 8.0));
    let tess = Tessellation::default();
    assert_eq!(
        s.profiles(&tess).len(),
        2,
        "the divider splits the rectangle"
    );
    crate::edit::trim(&mut s, divider, v(5.0, 7.0)).unwrap();
    crate::edit::trim(&mut s, divider, v(5.0, -1.0)).unwrap();
    let profiles = s.profiles(&tess);
    assert_eq!(profiles.len(), 2);
    for p in &profiles {
        assert_relative_eq!(p.area(), 30.0, epsilon = 1e-6);
        assert!(p.contains(p.interior_point().unwrap()));
    }
}

// ----- patterns -----------------------------------------------------------------------

#[test]
fn rectangular_pattern_repeats_geometry_and_its_constraints() {
    let mut s = Sketch::new();
    let r = shapes::rectangle_two_point(&mut s, v(0.0, 0.0), v(2.0, 1.0));
    let seed: Vec<EntityId> = r.lines.to_vec();
    let copies =
        crate::pattern::rectangular(&mut s, &seed, v(4.0, 0.0), 3, v(0.0, 3.0), 2).unwrap();
    let tess = Tessellation::default();
    assert_eq!(s.profiles(&tess).len(), 6, "one region per instance");
    // Each copy carries the horizontal/vertical constraints the rectangle was built
    // with, so it is a rectangle in its own right rather than four loose lines.
    let copied_lines = copies
        .iter()
        .filter(|id| s.entity(**id).unwrap().entity.is_line())
        .count();
    assert_eq!(copied_lines, 20);
    let constrained = s
        .constraints()
        .filter(|(_, c)| matches!(c, Constraint::Horizontal(_) | Constraint::Vertical(_)))
        .count();
    assert_eq!(constrained, 6 * 4);
    s.solve().unwrap();
    for p in s.profiles(&tess) {
        assert_relative_eq!(p.area(), 2.0, epsilon = 1e-6);
    }
}

#[test]
fn circular_pattern_spreads_copies_around_the_centre() {
    let mut s = Sketch::new();
    let c = s.add_point(v(10.0, 0.0));
    let hole = s.add_circle(c, 1.0).unwrap();
    let copies =
        crate::pattern::circular(&mut s, &[hole], Vec2::ZERO, 4, std::f64::consts::TAU).unwrap();
    let centres: Vec<Vec2> = copies
        .iter()
        .filter_map(|id| match s.entity(*id).unwrap().entity {
            Entity::Circle { center, .. } => s.point_pos(center),
            _ => None,
        })
        .collect();
    assert_eq!(centres.len(), 3);
    near(centres[0], v(0.0, 10.0));
    near(centres[1], v(-10.0, 0.0));
    near(centres[2], v(0.0, -10.0));
}

#[test]
fn a_pattern_needs_more_than_one_instance() {
    let mut s = Sketch::new();
    let c = s.add_point(v(1.0, 0.0));
    let circle = s.add_circle(c, 1.0).unwrap();
    assert!(matches!(
        crate::pattern::circular(&mut s, &[circle], Vec2::ZERO, 1, PI),
        Err(SketchError::InvalidArgument(_))
    ));
}

// ----- named parameters ---------------------------------------------------------------

#[test]
fn a_parameter_drives_every_dimension_written_over_it() {
    let mut s = Sketch::new();
    let (l, a, b) = line(&mut s, v(0.0, 0.0), v(10.0, 0.0));
    s.add_constraint(Constraint::Fix(a)).unwrap();
    s.add_constraint(Constraint::Horizontal(l)).unwrap();
    let dim = s
        .add_constraint(Constraint::Distance { a, b, value: 10.0 })
        .unwrap();
    s.set_parameter("width", "40").unwrap();
    assert_relative_eq!(s.bind_dimension(dim, "width / 2").unwrap(), 20.0);
    s.solve().unwrap();
    assert_relative_eq!(pos(&s, b).x, 20.0, epsilon = 1e-6);
    // Changing the parameter re-drives the dimension through the next solve.
    s.set_parameter("width", "30").unwrap();
    s.solve().unwrap();
    assert_relative_eq!(pos(&s, b).x, 15.0, epsilon = 1e-6);
}

#[test]
fn parameters_may_be_written_over_each_other_but_not_over_themselves() {
    let mut s = Sketch::new();
    s.set_parameter("wall", "2.5").unwrap();
    s.set_parameter("bore", "wall * 4").unwrap();
    assert_relative_eq!(s.parameter_value("bore").unwrap(), 10.0);
    assert!(matches!(
        s.set_parameter("wall", "bore / 2"),
        Err(SketchError::CircularParameter(_))
    ));
    // The rejected edit left the old expression in place.
    assert_relative_eq!(s.parameter_value("wall").unwrap(), 2.5);
    assert!(matches!(
        s.set_parameter("gap", "nonesuch + 1"),
        Err(SketchError::UnknownParameter(_))
    ));
    assert!(matches!(
        s.set_parameter("2gap", "1"),
        Err(SketchError::InvalidArgument(_))
    ));
}

#[test]
fn typing_a_number_over_a_driven_dimension_releases_it() {
    let mut s = Sketch::new();
    let (_, a, b) = line(&mut s, v(0.0, 0.0), v(10.0, 0.0));
    let dim = s
        .add_constraint(Constraint::Distance { a, b, value: 10.0 })
        .unwrap();
    s.set_parameter("len", "12").unwrap();
    s.bind_dimension(dim, "len").unwrap();
    assert_eq!(s.dimension_expr(dim), Some("len"));
    s.set_dimension_value(dim, 3.0).unwrap();
    assert_eq!(s.dimension_expr(dim), None);
    s.apply_parameters();
    assert_relative_eq!(s.constraint(dim).unwrap().dimension_value().unwrap(), 3.0);
}

#[test]
fn an_angle_parameter_is_written_in_degrees() {
    let mut s = Sketch::new();
    let (l1, _, _) = line(&mut s, v(0.0, 0.0), v(10.0, 0.0));
    let (l2, _, _) = line(&mut s, v(0.0, 0.0), v(10.0, 10.0));
    let dim = s
        .add_constraint(Constraint::Angle {
            a: l1,
            b: l2,
            value: FRAC_PI_4,
        })
        .unwrap();
    s.bind_dimension(dim, "60").unwrap();
    assert_relative_eq!(
        s.constraint(dim).unwrap().dimension_value().unwrap(),
        60f64.to_radians()
    );
}

#[test]
fn parameters_survive_a_round_trip_through_json() {
    let mut s = Sketch::new();
    let (_, a, b) = line(&mut s, v(0.0, 0.0), v(10.0, 0.0));
    let dim = s
        .add_constraint(Constraint::Distance { a, b, value: 10.0 })
        .unwrap();
    s.set_parameter("len", "7").unwrap();
    s.bind_dimension(dim, "len + 1").unwrap();
    let json = serde_json::to_string(&s).unwrap();
    let back: Sketch = serde_json::from_str(&json).unwrap();
    assert_eq!(back.parameters().len(), 1);
    assert_eq!(back.dimension_expr(dim), Some("len + 1"));
    assert_relative_eq!(back.parameter_value("len").unwrap(), 7.0);
}

#[test]
fn the_trim_preview_is_the_piece_the_trim_removes() {
    let mut s = Sketch::new();
    let (l, _, _) = line(&mut s, v(0.0, 0.0), v(10.0, 0.0));
    line(&mut s, v(3.0, -1.0), v(3.0, 1.0));
    line(&mut s, v(7.0, -1.0), v(7.0, 1.0));
    let preview = crate::edit::trim_preview(&s, l, v(5.0, 0.0), &Tessellation::default()).unwrap();
    near(preview[0], v(3.0, 0.0));
    near(*preview.last().unwrap(), v(7.0, 0.0));
}

#[test]
fn the_trim_preview_of_a_circle_wraps_around_its_start() {
    let mut s = Sketch::new();
    let c = s.add_point(v(0.0, 0.0));
    let circle = s.add_circle(c, 5.0).unwrap();
    line(&mut s, v(-6.0, 3.0), v(6.0, 3.0));
    // The piece above the chord straddles the circle's own start at angle 0? No: it is
    // the short piece over the top. The piece to its right wraps past the start instead.
    let over_the_top =
        crate::edit::trim_preview(&s, circle, v(0.0, 5.0), &Tessellation::default()).unwrap();
    assert!(over_the_top.iter().all(|p| p.y >= 2.99));
    let wrapping =
        crate::edit::trim_preview(&s, circle, v(0.0, -5.0), &Tessellation::default()).unwrap();
    assert!(wrapping.iter().any(|p| p.x > 4.9), "passes the start angle");
    assert!(wrapping.iter().any(|p| p.y < -4.9));
}

/// A turned copy must not inherit the seed's idea of which way is up.
///
/// Horizontal and Vertical describe the sketch's axes, not the shape, so a copy turned a
/// quarter of a turn has them the other way round. Copying them unchanged contradicts
/// the copy outright, and the solver settles the contradiction by folding it flat — the
/// reason a circular pattern of anything drawn square used to destroy its own copies.
#[test]
fn a_rotated_copy_does_not_inherit_the_seed_s_axes() {
    let mut s = Sketch::new();
    let r = shapes::rectangle_two_point(&mut s, v(20.0, 0.0), v(25.0, 5.0));
    let seed: Vec<EntityId> = r.lines.to_vec();
    let area = |s: &Sketch| {
        s.profiles(&tess())
            .iter()
            .map(|p| p.area())
            .collect::<Vec<_>>()
    };
    let seed_area = area(&s)[0];
    assert!(seed_area > 0.0);

    crate::pattern::circular(&mut s, &seed, Vec2::ZERO, 4, std::f64::consts::TAU).unwrap();
    s.solve().expect("a pattern of a square is solvable");
    let areas = area(&s);
    assert_eq!(areas.len(), 4, "four squares, none folded away");
    for a in &areas {
        assert_relative_eq!(*a, seed_area, epsilon = 1e-9);
    }
}

/// A quarter turn swaps the axes rather than dropping them, so a copy turned that far is
/// as constrained as the seed — just about the other axis.
#[test]
fn a_quarter_turn_swaps_horizontal_for_vertical() {
    let mut s = Sketch::new();
    let a = s.add_point(v(10.0, 0.0));
    let b = s.add_point(v(20.0, 0.0));
    let line = s.add_line(a, b).unwrap();
    s.add_constraint(Constraint::Horizontal(line)).unwrap();

    crate::pattern::circular(&mut s, &[line], Vec2::ZERO, 4, std::f64::consts::TAU).unwrap();
    s.solve().unwrap();
    let verticals = s
        .constraints()
        .filter(|(_, c)| matches!(c, Constraint::Vertical(_)))
        .count();
    let horizontals = s
        .constraints()
        .filter(|(_, c)| matches!(c, Constraint::Horizontal(_)))
        .count();
    assert_eq!(verticals, 2, "the quarter and three-quarter turns");
    assert_eq!(horizontals, 2, "the seed and the half turn");
}

/// A turn that is not a multiple of a right angle has no axis-aligned answer, so the
/// axis constraints are dropped and the copy is honestly loose rather than crushed.
#[test]
fn an_odd_turn_drops_the_axis_constraints() {
    let mut s = Sketch::new();
    let a = s.add_point(v(10.0, 0.0));
    let b = s.add_point(v(20.0, 0.0));
    let line = s.add_line(a, b).unwrap();
    s.add_constraint(Constraint::Horizontal(line)).unwrap();
    s.add_constraint(Constraint::Distance { a, b, value: 10.0 })
        .unwrap();

    crate::pattern::circular(&mut s, &[line], Vec2::ZERO, 3, std::f64::consts::TAU).unwrap();
    s.solve().unwrap();
    assert_eq!(
        s.constraints()
            .filter(|(_, c)| matches!(c, Constraint::Horizontal(_)))
            .count(),
        1,
        "only the seed keeps it"
    );
    // The length is about the shape, not about the axes, so every copy keeps it and is
    // still the right size.
    assert_eq!(
        s.constraints()
            .filter(|(_, c)| matches!(c, Constraint::Distance { .. }))
            .count(),
        3
    );
    for (id, _) in s.entities().filter(|(_, d)| d.entity.is_line()) {
        let (p, q) = s.curve_endpoints(id).unwrap();
        assert_relative_eq!(p.distance(q), 10.0, epsilon = 1e-9);
    }
}
