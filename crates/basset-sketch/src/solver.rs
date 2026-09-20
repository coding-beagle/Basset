//! Constraint solver.
//!
//! # Parameterisation
//! Every point contributes `(x, y)` and every circle its radius to one flat parameter
//! vector. Points under a [`Constraint::Fix`] are *removed* from the free set and read as
//! constants: that keeps the Jacobian smaller and makes "0 remaining degrees of freedom"
//! mean exactly what a user expects. An arc's radius is not a parameter; it is implied
//! by `|start − center|`, and an implicit equation keeps `|end − center|` equal to it.
//!
//! # Equations
//! Each constraint compiles into one or more [`Equation`]s. An equation references at
//! most eight scalar parameters ("local variables"), evaluates its residuals with dual
//! numbers seeded on those variables, and scatters the resulting exact partials into the
//! global Jacobian. Some constraints have a sign or mode chosen at compile time from the
//! current geometry (e.g. which side of a line a point is, or whether two circles touch
//! internally or externally) so that the solver preserves the user's intent instead of
//! flipping the sketch to the other solution branch.
//!
//! # Solve
//! Levenberg–Marquardt on `‖r‖²`. Convergence is `‖r_hard‖ ≤ 1e-10 · max(1, extent)`
//! where `extent` is the sketch's bounding size, so tolerance scales with the model.
//! Soft equations (the drag goal) are included in the least-squares objective but not in
//! the convergence test or the rank estimate. Remaining degrees of freedom are
//! `free parameters − rank(J_hard)`, and the null space of `J_hard` says which parameters
//! they belong to (see [`SolveReport::under_constrained`]). The other side of the same
//! rank question is which constraints the rest of the sketch already implies, which is
//! [`SolveReport::redundant`]; both analyses equilibrate the Jacobian's columns first so
//! that their tolerances mean the same thing at every feature size.

use std::collections::HashMap;
use std::f64::consts::PI;

use basset_math::Vec2;
use serde::{Deserialize, Serialize};

use crate::dual::{DVec, Dual, MAX_LOCAL_VARS};
use crate::linalg::{Mat, solve_spd};
use crate::{Constraint, ConstraintId, Entity, EntityId, Sketch, SolveError};

/// Not `Copy`: it carries the list of loose entities, which is the whole point of
/// reporting more than a count.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SolveReport {
    pub iterations: usize,
    /// Norm of the hard-constraint residual vector at the end of the solve.
    pub residual: f64,
    pub converged: bool,
    /// Remaining degrees of freedom, estimated from the Jacobian rank.
    ///
    /// The estimate is *instantaneous*: it counts directions the constraints do not
    /// resist to first order. A configuration that is pinned only at second order — a
    /// point tied to another by a zero-length distance, or one held by two distance
    /// dimensions from either side exactly on the line between them — therefore reports a
    /// freedom it cannot actually use. Both are degenerate states of a re-dimensioned
    /// sketch, and erring towards "this can move" is the safer way round for a warning.
    pub degrees_of_freedom: usize,
    /// The points and circles those degrees of freedom belong to, so the editor can show
    /// *which* geometry is still loose rather than only how much of it is. Never empty
    /// while [`Self::degrees_of_freedom`] is non-zero, and always empty when it is zero.
    pub under_constrained: Vec<EntityId>,
    /// The constraints the rest of the sketch already implies: deleting one would change
    /// neither the geometry nor [`Self::degrees_of_freedom`]. See [`System::redundant`]
    /// for the exact test and for how this differs from the conflicting constraints of a
    /// solve that failed.
    ///
    /// Defaulted on deserialisation so a document saved before the check existed still
    /// loads; it is re-derived on the next solve anyway.
    #[serde(default)]
    pub redundant: Vec<ConstraintId>,
}

/// Relative tolerance for the rank estimate and for the dependence test behind
/// [`SolveReport::redundant`]. Both run on a column-equilibrated Jacobian, so every entry
/// is at most one whatever the sketch is drawn at, and 1e-9 separates genuine dependence
/// from rounding noise comfortably.
const RANK_TOL: f64 = 1e-9;
const MAX_ITERATIONS: usize = 200;
/// Weight of the drag goal relative to hard constraints. It only biases the solution
/// within the constraint null space; a final hard-only pass removes its influence on
/// constraint satisfaction.
const DRAG_WEIGHT: f64 = 1e-2;

/// A scalar parameter as seen from an equation: either a free variable or a constant.
#[derive(Debug, Clone, Copy)]
enum Var {
    Free(usize),
    Const(f64),
}

/// Where an equation gets a radius from: a circle's own parameter, or an arc's distance
/// between its centre and start point (both already among the local variables).
#[derive(Debug, Clone, Copy)]
enum RadiusSource {
    Slot(usize),
    FromPoints { center: usize, point: usize },
}

/// Residual formulas. Slot indices refer to the equation's local variable list; a point
/// occupies two consecutive slots `(x, y)`.
#[derive(Debug, Clone, Copy)]
enum Formula {
    /// p == q (two residuals).
    PointsCoincide {
        p: usize,
        q: usize,
    },
    /// Signed distance of p to the infinite line ab, minus `sign * value`.
    PointLine {
        p: usize,
        a: usize,
        b: usize,
        sign: f64,
        value: f64,
    },
    /// |p − c| − r − value.
    PointCenterRadius {
        p: usize,
        c: usize,
        r: RadiusSource,
        value: f64,
    },
    /// (b − a).y (Horizontal) or (b − a).x (Vertical).
    Delta {
        a: usize,
        b: usize,
        component: usize,
        value: f64,
    },
    /// Normalised cross (Parallel) or dot (Perpendicular) of the two line directions.
    Directions {
        a: usize,
        b: usize,
        c: usize,
        d: usize,
        cross: bool,
    },
    EqualLength {
        a: usize,
        b: usize,
        c: usize,
        d: usize,
    },
    EqualRadius {
        r1: RadiusSource,
        r2: RadiusSource,
    },
    /// Signed distance of centre c to line ab minus `sign · r`.
    LineCircleTangent {
        a: usize,
        b: usize,
        c: usize,
        r: RadiusSource,
        sign: f64,
    },
    /// |c1 − c2| − (s1·r1 + s2·r2): external tangency for (1, 1), internal for (1, −1).
    CircleCircleTangent {
        c1: usize,
        r1: RadiusSource,
        c2: usize,
        r2: RadiusSource,
        s1: f64,
        s2: f64,
    },
    Midpoint {
        p: usize,
        a: usize,
        b: usize,
    },
    Symmetric {
        a: usize,
        b: usize,
        s: usize,
        e: usize,
    },
    Distance {
        a: usize,
        b: usize,
        value: f64,
    },
    Radius {
        r: RadiusSource,
        value: f64,
    },
    Angle {
        a: usize,
        b: usize,
        c: usize,
        d: usize,
        value: f64,
    },
    /// Weighted pull of p toward a target (two residuals).
    Goal {
        p: usize,
        target: Vec2,
        weight: f64,
    },
}

struct Equation {
    vars: [Var; MAX_LOCAL_VARS],
    formula: Formula,
    /// Soft equations shape the least-squares objective but do not count as constraints.
    hard: bool,
    /// The constraint this equation came from. Equations the sketch implies rather than
    /// the user wrote — an arc's two radii, the drag goal — have none.
    source: Option<ConstraintId>,
}

/// Evaluated residuals of one equation, at most two.
struct Residuals {
    values: [Dual; 2],
    count: usize,
}

impl Residuals {
    fn one(r: Dual) -> Self {
        Self {
            values: [r, Dual::ZERO],
            count: 1,
        }
    }
    fn two(a: Dual, b: Dual) -> Self {
        Self {
            values: [a, b],
            count: 2,
        }
    }
}

fn pt(x: &[Dual; MAX_LOCAL_VARS], slot: usize) -> DVec {
    DVec {
        x: x[slot],
        y: x[slot + 1],
    }
}

fn radius(x: &[Dual; MAX_LOCAL_VARS], src: RadiusSource) -> Dual {
    match src {
        RadiusSource::Slot(s) => x[s],
        RadiusSource::FromPoints { center, point } => (pt(x, point) - pt(x, center)).length(),
    }
}

/// Signed distance from `p` to the infinite line through `a`,`b`; positive on the left.
fn signed_line_distance(p: DVec, a: DVec, b: DVec) -> Dual {
    let d = b - a;
    d.cross(p - a) / safe_length(d)
}

/// Length that never reaches exactly zero, so ratios stay finite for degenerate input.
fn safe_length(v: DVec) -> Dual {
    (v.dot(v) + 1e-24).sqrt()
}

fn wrap_angle(a: f64) -> f64 {
    (a + PI).rem_euclid(2.0 * PI) - PI
}

impl Formula {
    fn eval(&self, x: &[Dual; MAX_LOCAL_VARS]) -> Residuals {
        match *self {
            Formula::PointsCoincide { p, q } => {
                let d = pt(x, p) - pt(x, q);
                Residuals::two(d.x, d.y)
            }
            Formula::PointLine {
                p,
                a,
                b,
                sign,
                value,
            } => Residuals::one(signed_line_distance(pt(x, p), pt(x, a), pt(x, b)) - sign * value),
            Formula::PointCenterRadius { p, c, r, value } => {
                Residuals::one((pt(x, p) - pt(x, c)).length() - radius(x, r) - value)
            }
            Formula::Delta {
                a,
                b,
                component,
                value,
            } => {
                let d = pt(x, b) - pt(x, a);
                Residuals::one(if component == 0 { d.x } else { d.y } - value)
            }
            Formula::Directions { a, b, c, d, cross } => {
                let u = pt(x, b) - pt(x, a);
                let v = pt(x, d) - pt(x, c);
                let num = if cross { u.cross(v) } else { u.dot(v) };
                Residuals::one(num / (safe_length(u) * safe_length(v)))
            }
            Formula::EqualLength { a, b, c, d } => {
                Residuals::one((pt(x, b) - pt(x, a)).length() - (pt(x, d) - pt(x, c)).length())
            }
            Formula::EqualRadius { r1, r2 } => Residuals::one(radius(x, r1) - radius(x, r2)),
            Formula::LineCircleTangent { a, b, c, r, sign } => Residuals::one(
                signed_line_distance(pt(x, c), pt(x, a), pt(x, b)) - radius(x, r) * sign,
            ),
            Formula::CircleCircleTangent {
                c1,
                r1,
                c2,
                r2,
                s1,
                s2,
            } => {
                let dist = (pt(x, c1) - pt(x, c2)).length();
                Residuals::one(dist - (radius(x, r1) * s1 + radius(x, r2) * s2))
            }
            Formula::Midpoint { p, a, b } => {
                let m = (pt(x, a) + pt(x, b)) * 0.5;
                let d = pt(x, p) - m;
                Residuals::two(d.x, d.y)
            }
            Formula::Symmetric { a, b, s, e } => {
                // Midpoint on the axis, and the a→b direction perpendicular to the axis.
                let axis = pt(x, e) - pt(x, s);
                let m = (pt(x, a) + pt(x, b)) * 0.5;
                let len = safe_length(axis);
                Residuals::two(
                    axis.cross(m - pt(x, s)) / len,
                    axis.dot(pt(x, a) - pt(x, b)) / len,
                )
            }
            Formula::Distance { a, b, value } => {
                Residuals::one((pt(x, b) - pt(x, a)).length() - value)
            }
            Formula::Radius { r, value } => Residuals::one(radius(x, r) - value),
            Formula::Angle { a, b, c, d, value } => {
                let u = pt(x, b) - pt(x, a);
                let v = pt(x, d) - pt(x, c);
                let mut ang = Dual::atan2(u.cross(v), u.dot(v)) - value;
                // Wrapping only touches the value: the derivative of a shift is unchanged.
                ang.v = wrap_angle(ang.v);
                Residuals::one(ang)
            }
            Formula::Goal { p, target, weight } => {
                let d = pt(x, p);
                Residuals::two((d.x - target.x) * weight, (d.y - target.y) * weight)
            }
        }
    }
}

/// Builds the local variable list of one equation and records slot positions.
struct Locals {
    vars: [Var; MAX_LOCAL_VARS],
    used: usize,
}

impl Locals {
    fn new() -> Self {
        Self {
            vars: [Var::Const(0.0); MAX_LOCAL_VARS],
            used: 0,
        }
    }

    fn push(&mut self, v: Var) -> usize {
        let slot = self.used;
        self.vars[slot] = v;
        self.used += 1;
        slot
    }
}

/// Flattened parameter vector plus the map from entities to parameter indices.
pub(crate) struct System {
    params: Vec<f64>,
    /// Which entity each parameter belongs to, parallel to `params`. Reporting *which*
    /// geometry a remaining degree of freedom belongs to means going back from a column
    /// of the Jacobian to the entity that put it there.
    owners: Vec<EntityId>,
    /// Base index of a point's `x` (y follows) or a circle's radius; absent for fixed points.
    index: HashMap<EntityId, usize>,
    /// Current value of every point, including fixed ones, for constant lookup.
    point_values: HashMap<EntityId, Vec2>,
    equations: Vec<Equation>,
    /// The constraint being compiled, so every equation remembers who asked for it and a
    /// failed solve can name the constraints that disagree.
    current: Option<ConstraintId>,
    /// Characteristic size of the sketch, for scale-relative tolerances.
    scale: f64,
}

impl System {
    pub(crate) fn compile(sketch: &Sketch, drag: &[(EntityId, Vec2)]) -> Result<Self, SolveError> {
        let fixed: std::collections::HashSet<EntityId> = sketch
            .constraints()
            .filter_map(|(_, c)| {
                if let Constraint::Fix(p) = c {
                    Some(*p)
                } else {
                    None
                }
            })
            .collect();
        let mut sys = System {
            params: Vec::new(),
            owners: Vec::new(),
            index: HashMap::new(),
            point_values: HashMap::new(),
            equations: Vec::new(),
            current: None,
            scale: 1.0,
        };
        let mut min = Vec2::splat(f64::INFINITY);
        let mut max = Vec2::splat(f64::NEG_INFINITY);
        for (id, data) in sketch.entities() {
            match data.entity {
                Entity::Point { pos } => {
                    sys.point_values.insert(id, pos);
                    min = min.min(pos);
                    max = max.max(pos);
                    if !fixed.contains(&id) {
                        sys.index.insert(id, sys.params.len());
                        sys.params.extend([pos.x, pos.y]);
                        sys.owners.extend([id, id]);
                    }
                }
                Entity::Circle { radius, .. } => {
                    sys.index.insert(id, sys.params.len());
                    sys.params.push(radius);
                    sys.owners.push(id);
                }
                _ => {}
            }
        }
        if min.x.is_finite() {
            sys.scale = (max - min).max_element().max(1.0);
        }
        for (id, c) in sketch.constraints() {
            sys.current = Some(id);
            sys.compile_constraint(sketch, c);
        }
        sys.current = None;
        // Implicit arc consistency: |end − c| == |start − c|.
        for (_, data) in sketch.entities() {
            if let Entity::Arc { center, start, end } = data.entity {
                let mut l = Locals::new();
                let c = sys.point(&mut l, center);
                let s = sys.point(&mut l, start);
                let e = sys.point(&mut l, end);
                sys.push(
                    l,
                    Formula::PointCenterRadius {
                        p: e,
                        c,
                        r: RadiusSource::FromPoints {
                            center: c,
                            point: s,
                        },
                        value: 0.0,
                    },
                    true,
                );
            }
        }
        for &(point, target) in drag {
            let data = sketch
                .entity(point)
                .ok_or(SolveError::UnknownEntity(point))?;
            if !data.entity.is_point() {
                return Err(SolveError::NotAPoint(point));
            }
            let mut l = Locals::new();
            let p = sys.point(&mut l, point);
            sys.push(
                l,
                Formula::Goal {
                    p,
                    target,
                    weight: DRAG_WEIGHT,
                },
                false,
            );
        }
        Ok(sys)
    }

    fn point(&self, l: &mut Locals, id: EntityId) -> usize {
        let pos = self.point_values.get(&id).copied().unwrap_or(Vec2::ZERO);
        match self.index.get(&id) {
            Some(&base) => {
                let s = l.push(Var::Free(base));
                l.push(Var::Free(base + 1));
                s
            }
            None => {
                let s = l.push(Var::Const(pos.x));
                l.push(Var::Const(pos.y));
                s
            }
        }
    }

    /// Pushes the parameters describing a circle's or arc's centre and radius.
    fn circular(
        &self,
        l: &mut Locals,
        sketch: &Sketch,
        id: EntityId,
    ) -> Option<(usize, RadiusSource)> {
        match sketch.entity(id)?.entity {
            Entity::Circle { center, radius } => {
                let c = self.point(l, center);
                let r = match self.index.get(&id) {
                    Some(&i) => l.push(Var::Free(i)),
                    None => l.push(Var::Const(radius)),
                };
                Some((c, RadiusSource::Slot(r)))
            }
            Entity::Arc { center, start, .. } => {
                let c = self.point(l, center);
                let s = self.point(l, start);
                Some((
                    c,
                    RadiusSource::FromPoints {
                        center: c,
                        point: s,
                    },
                ))
            }
            _ => None,
        }
    }

    fn line_points(sketch: &Sketch, id: EntityId) -> Option<(EntityId, EntityId)> {
        match sketch.entity(id)?.entity {
            Entity::Line { start, end } => Some((start, end)),
            _ => None,
        }
    }

    fn pos(&self, id: EntityId) -> Vec2 {
        self.point_values.get(&id).copied().unwrap_or(Vec2::ZERO)
    }

    fn current_radius(&self, sketch: &Sketch, id: EntityId) -> f64 {
        match sketch.entity(id).map(|d| &d.entity) {
            Some(Entity::Circle { radius, .. }) => *radius,
            Some(Entity::Arc { center, start, .. }) => {
                (self.pos(*start) - self.pos(*center)).length()
            }
            _ => 0.0,
        }
    }

    fn push(&mut self, l: Locals, formula: Formula, hard: bool) {
        self.equations.push(Equation {
            vars: l.vars,
            formula,
            hard,
            source: self.current,
        });
    }

    /// Sign (+1/−1) of the current side of `p` relative to line `ab`, defaulting to +1.
    fn side_sign(&self, p: EntityId, a: EntityId, b: EntityId) -> f64 {
        let d = self.pos(b) - self.pos(a);
        if d.perp_dot(self.pos(p) - self.pos(a)) < 0.0 {
            -1.0
        } else {
            1.0
        }
    }

    /// Tangency has two formulations. Where the curves share an endpoint (a line meeting
    /// an arc in a slot, two arcs in a fillet chain) the condition is local to that
    /// point: the line is perpendicular to the radius there, or the centres are collinear
    /// with it. Expressing it through the circle's distance instead would be satisfied at
    /// the same geometry but with a rank-deficient Jacobian, since sliding the shared
    /// point along the curve changes nothing to first order; that would misreport the
    /// degrees of freedom and slow the solver. Curves that do not share a point use the
    /// centre-to-line distance or centre-to-centre distance, keeping the current side.
    fn compile_tangent(&mut self, sketch: &Sketch, ea: EntityId, eb: EntityId, mut l: Locals) {
        let endpoints = |id: EntityId| -> Vec<EntityId> {
            match sketch.entity(id).map(|d| &d.entity) {
                Some(Entity::Line { start, end }) | Some(Entity::Arc { start, end, .. }) => {
                    vec![*start, *end]
                }
                _ => Vec::new(),
            }
        };
        let center_of = |id: EntityId| -> Option<EntityId> {
            match sketch.entity(id).map(|d| &d.entity) {
                Some(Entity::Circle { center, .. }) | Some(Entity::Arc { center, .. }) => {
                    Some(*center)
                }
                _ => None,
            }
        };
        let shared = endpoints(ea)
            .into_iter()
            .find(|p| endpoints(eb).contains(p));
        let (line, circ) = if Self::line_points(sketch, ea).is_some() {
            (ea, eb)
        } else {
            (eb, ea)
        };
        if let Some((s, e)) = Self::line_points(sketch, line) {
            let Some(center) = center_of(circ) else {
                return;
            };
            if let Some(p) = shared {
                let a = self.point(&mut l, s);
                let b = self.point(&mut l, e);
                let c = self.point(&mut l, center);
                let d = self.point(&mut l, p);
                self.push(
                    l,
                    Formula::Directions {
                        a,
                        b,
                        c,
                        d,
                        cross: false,
                    },
                    true,
                );
            } else {
                let sign = self.side_sign(center, s, e);
                let a = self.point(&mut l, s);
                let b = self.point(&mut l, e);
                if let Some((c, r)) = self.circular(&mut l, sketch, circ) {
                    self.push(l, Formula::LineCircleTangent { a, b, c, r, sign }, true);
                }
            }
            return;
        }
        let (Some(ca), Some(cb)) = (center_of(ea), center_of(eb)) else {
            return;
        };
        if let Some(p) = shared {
            let a = self.point(&mut l, p);
            let b = self.point(&mut l, ca);
            let d = self.point(&mut l, cb);
            self.push(
                l,
                Formula::Directions {
                    a,
                    b,
                    c: a,
                    d,
                    cross: true,
                },
                true,
            );
            return;
        }
        // Choose the tangency branch (external vs internal) closest to the current
        // geometry so the solver does not jump to the other one.
        let (ra, rb) = (
            self.current_radius(sketch, ea),
            self.current_radius(sketch, eb),
        );
        let dist = (self.pos(ca) - self.pos(cb)).length();
        let external = (dist - (ra + rb)).abs();
        let internal = (dist - (ra - rb).abs()).abs();
        let (s1, s2) = if external <= internal {
            (1.0, 1.0)
        } else if ra >= rb {
            (1.0, -1.0)
        } else {
            (-1.0, 1.0)
        };
        if let (Some((c1, r1)), Some((c2, r2))) = (
            self.circular(&mut l, sketch, ea),
            self.circular(&mut l, sketch, eb),
        ) {
            self.push(
                l,
                Formula::CircleCircleTangent {
                    c1,
                    r1,
                    c2,
                    r2,
                    s1,
                    s2,
                },
                true,
            );
        }
    }

    /// Constraints are validated when added, so unexpected kinds here mean a stale
    /// reference; such constraints are skipped rather than crashing the solve.
    fn compile_constraint(&mut self, sketch: &Sketch, c: &Constraint) {
        let mut l = Locals::new();
        match *c {
            Constraint::Coincident { point, target } => {
                match sketch.entity(target).map(|d| &d.entity) {
                    Some(Entity::Point { .. }) => {
                        let p = self.point(&mut l, point);
                        let q = self.point(&mut l, target);
                        self.push(l, Formula::PointsCoincide { p, q }, true);
                    }
                    Some(Entity::Line { start, end }) => {
                        let (start, end) = (*start, *end);
                        let p = self.point(&mut l, point);
                        let a = self.point(&mut l, start);
                        let b = self.point(&mut l, end);
                        self.push(
                            l,
                            Formula::PointLine {
                                p,
                                a,
                                b,
                                sign: 1.0,
                                value: 0.0,
                            },
                            true,
                        );
                    }
                    Some(Entity::Circle { .. }) | Some(Entity::Arc { .. }) => {
                        let p = self.point(&mut l, point);
                        if let Some((c, r)) = self.circular(&mut l, sketch, target) {
                            self.push(
                                l,
                                Formula::PointCenterRadius {
                                    p,
                                    c,
                                    r,
                                    value: 0.0,
                                },
                                true,
                            );
                        }
                    }
                    _ => log::warn!("coincident constraint references a missing or invalid target"),
                }
            }
            Constraint::Horizontal(line) | Constraint::Vertical(line) => {
                if let Some((s, e)) = Self::line_points(sketch, line) {
                    let a = self.point(&mut l, s);
                    let b = self.point(&mut l, e);
                    let component = if matches!(c, Constraint::Horizontal(_)) {
                        1
                    } else {
                        0
                    };
                    self.push(
                        l,
                        Formula::Delta {
                            a,
                            b,
                            component,
                            value: 0.0,
                        },
                        true,
                    );
                }
            }
            Constraint::Parallel(la, lb) | Constraint::Perpendicular(la, lb) => {
                if let (Some((s1, e1)), Some((s2, e2))) =
                    (Self::line_points(sketch, la), Self::line_points(sketch, lb))
                {
                    let a = self.point(&mut l, s1);
                    let b = self.point(&mut l, e1);
                    let cc = self.point(&mut l, s2);
                    let d = self.point(&mut l, e2);
                    let cross = matches!(c, Constraint::Parallel(..));
                    self.push(
                        l,
                        Formula::Directions {
                            a,
                            b,
                            c: cc,
                            d,
                            cross,
                        },
                        true,
                    );
                }
            }
            Constraint::Equal(ea, eb) => {
                if let (Some((s1, e1)), Some((s2, e2))) =
                    (Self::line_points(sketch, ea), Self::line_points(sketch, eb))
                {
                    let a = self.point(&mut l, s1);
                    let b = self.point(&mut l, e1);
                    let cc = self.point(&mut l, s2);
                    let d = self.point(&mut l, e2);
                    self.push(l, Formula::EqualLength { a, b, c: cc, d }, true);
                } else if let (Some((_, r1)), Some((_, r2))) = (
                    self.circular(&mut l, sketch, ea),
                    self.circular(&mut l, sketch, eb),
                ) {
                    self.push(l, Formula::EqualRadius { r1, r2 }, true);
                }
            }
            Constraint::Tangent(ea, eb) => self.compile_tangent(sketch, ea, eb, l),
            // Fixed points are constants; nothing to solve.
            Constraint::Fix(_) => {}
            Constraint::Midpoint { point, line } => {
                if let Some((s, e)) = Self::line_points(sketch, line) {
                    let p = self.point(&mut l, point);
                    let a = self.point(&mut l, s);
                    let b = self.point(&mut l, e);
                    self.push(l, Formula::Midpoint { p, a, b }, true);
                }
            }
            Constraint::Symmetric { a: pa, b: pb, axis } => {
                if let Some((s, e)) = Self::line_points(sketch, axis) {
                    let a = self.point(&mut l, pa);
                    let b = self.point(&mut l, pb);
                    let s = self.point(&mut l, s);
                    let e = self.point(&mut l, e);
                    self.push(l, Formula::Symmetric { a, b, s, e }, true);
                }
            }
            Constraint::Concentric(ea, eb) => {
                let ca = sketch
                    .entity(ea)
                    .and_then(|d| d.entity.references().first().copied());
                let cb = sketch
                    .entity(eb)
                    .and_then(|d| d.entity.references().first().copied());
                if let (Some(ca), Some(cb)) = (ca, cb) {
                    let p = self.point(&mut l, ca);
                    let q = self.point(&mut l, cb);
                    self.push(l, Formula::PointsCoincide { p, q }, true);
                }
            }
            Constraint::Distance {
                a: pa,
                b: target,
                value,
            } => match sketch.entity(target).map(|d| &d.entity) {
                Some(Entity::Point { .. }) => {
                    let a = self.point(&mut l, pa);
                    let b = self.point(&mut l, target);
                    self.push(l, Formula::Distance { a, b, value }, true);
                }
                Some(Entity::Line { start, end }) => {
                    let (start, end) = (*start, *end);
                    let sign = self.side_sign(pa, start, end);
                    let p = self.point(&mut l, pa);
                    let a = self.point(&mut l, start);
                    let b = self.point(&mut l, end);
                    self.push(
                        l,
                        Formula::PointLine {
                            p,
                            a,
                            b,
                            sign,
                            value,
                        },
                        true,
                    );
                }
                _ => log::warn!("distance constraint references a missing or invalid target"),
            },
            Constraint::HorizontalDistance {
                a: pa,
                b: pb,
                value,
            }
            | Constraint::VerticalDistance {
                a: pa,
                b: pb,
                value,
            } => {
                let component = if matches!(c, Constraint::HorizontalDistance { .. }) {
                    0
                } else {
                    1
                };
                // Unsigned dimension: keep b on the side of a it is on now.
                let delta = self.pos(pb) - self.pos(pa);
                let sign = if delta[component] < 0.0 { -1.0 } else { 1.0 };
                let a = self.point(&mut l, pa);
                let b = self.point(&mut l, pb);
                self.push(
                    l,
                    Formula::Delta {
                        a,
                        b,
                        component,
                        value: sign * value,
                    },
                    true,
                );
            }
            Constraint::Radius { curve, value } | Constraint::Diameter { curve, value } => {
                let value = if matches!(c, Constraint::Diameter { .. }) {
                    value * 0.5
                } else {
                    value
                };
                if let Some((_, r)) = self.circular(&mut l, sketch, curve) {
                    self.push(l, Formula::Radius { r, value }, true);
                }
            }
            Constraint::Angle {
                a: la,
                b: lb,
                value,
            } => {
                if let (Some((s1, e1)), Some((s2, e2))) =
                    (Self::line_points(sketch, la), Self::line_points(sketch, lb))
                {
                    let a = self.point(&mut l, s1);
                    let b = self.point(&mut l, e1);
                    let cc = self.point(&mut l, s2);
                    let d = self.point(&mut l, e2);
                    self.push(
                        l,
                        Formula::Angle {
                            a,
                            b,
                            c: cc,
                            d,
                            value,
                        },
                        true,
                    );
                }
            }
        }
    }

    #[cfg(test)]
    pub(crate) fn free_count(&self) -> usize {
        self.params.len()
    }

    fn hard_row_count(&self) -> usize {
        self.equations
            .iter()
            .filter(|e| e.hard)
            .map(|e| e.residual_count())
            .sum()
    }

    /// Evaluates all residuals and the Jacobian at `params`. Hard rows come first.
    fn evaluate(&self, params: &[f64]) -> Result<(Vec<f64>, Mat), SolveError> {
        let n = params.len();
        let rows: usize = self.equations.iter().map(|e| e.residual_count()).sum();
        let mut r = vec![0.0; rows];
        let mut j = Mat::zeros(rows, n);
        let mut row = 0;
        let ordered = self
            .equations
            .iter()
            .filter(|e| e.hard)
            .chain(self.equations.iter().filter(|e| !e.hard));
        for eq in ordered {
            let mut x = [Dual::ZERO; MAX_LOCAL_VARS];
            for (slot, var) in eq.vars.iter().enumerate() {
                x[slot] = match *var {
                    Var::Free(i) => Dual::variable(params[i], slot),
                    Var::Const(v) => Dual::constant(v),
                };
            }
            let res = eq.formula.eval(&x);
            for k in 0..res.count {
                let val = res.values[k];
                if !val.is_finite() {
                    return Err(SolveError::Numerical(format!(
                        "non-finite residual in {:?}",
                        eq.formula
                    )));
                }
                r[row] = val.v;
                for (slot, var) in eq.vars.iter().enumerate() {
                    if let Var::Free(i) = *var {
                        *j.at_mut(row, i) += val.d[slot];
                    }
                }
                row += 1;
            }
        }
        Ok((r, j))
    }

    /// The rows of the hard Jacobian each equation owns, paired with the constraint that
    /// compiled it (`None` for equations the sketch implies rather than the user wrote:
    /// an arc's second radius). Rows follow [`Self::evaluate`]'s order, hard first.
    ///
    /// Attributing a row back to a constraint is the one piece both the conflicting and
    /// the redundant report need, so it is counted in one place: a second walk that
    /// disagreed about row numbering would name the wrong constraint, and only for the
    /// sketches with an implied equation in them.
    fn hard_row_spans(
        &self,
    ) -> impl Iterator<Item = (Option<ConstraintId>, std::ops::Range<usize>)> {
        let mut row = 0;
        self.equations.iter().filter(|e| e.hard).map(move |eq| {
            let start = row;
            row += eq.residual_count();
            (eq.source, start..row)
        })
    }

    /// The constraints still unsatisfied at the end of a solve, worst first.
    ///
    /// A sketch that will not solve is usually two or three constraints asking for
    /// incompatible things; the rest are satisfied and innocent. Which ones those are is
    /// in the residual vector, one entry per equation, so it costs nothing to say. Rows
    /// follow [`Self::evaluate`]'s order: hard equations first.
    fn conflicts(&self, r: &[f64]) -> Vec<ConstraintId> {
        let tol = self.convergence_tolerance();
        let mut worst: Vec<(ConstraintId, f64)> = Vec::new();
        for (source, rows) in self.hard_row_spans() {
            let norm = r[rows].iter().map(|v| v * v).sum::<f64>().sqrt();
            let Some(id) = source else { continue };
            if norm <= tol {
                continue;
            }
            match worst.iter_mut().find(|(c, _)| *c == id) {
                Some(entry) => entry.1 = entry.1.max(norm),
                None => worst.push((id, norm)),
            }
        }
        worst.sort_by(|a, b| b.1.total_cmp(&a.1));
        worst.into_iter().map(|(id, _)| id).collect()
    }

    fn convergence_tolerance(&self) -> f64 {
        1e-10 * self.scale
    }

    fn hard_norm(&self, r: &[f64]) -> f64 {
        let h = self.hard_row_count();
        r[..h].iter().map(|v| v * v).sum::<f64>().sqrt()
    }

    /// Runs Levenberg–Marquardt from the current parameters. Never errors on
    /// non-convergence; the caller decides what a stall means.
    fn minimise(&mut self, max_iterations: usize) -> Result<LmOutcome, SolveError> {
        let n = self.params.len();
        let tol = self.convergence_tolerance();
        let (mut r, mut j) = self.evaluate(&self.params)?;
        let mut cost: f64 = r.iter().map(|v| v * v).sum();
        let mut lambda = 1e-3;
        let mut iterations = 0;
        let mut jac_for_rank = j.clone();
        let stall_step = 1e-13 * self.scale;
        while iterations < max_iterations {
            if self.hard_norm(&r) <= tol
                && (self.equations.iter().all(|e| e.hard) || cost.sqrt() <= tol)
            {
                break;
            }
            if n == 0 {
                // Nothing can move; whatever the residual is, it stays.
                break;
            }
            iterations += 1;
            let jtj = j.gram();
            let jtj_trace: f64 = (0..n).map(|i| jtj.at(i, i)).sum();
            let jtr = j.transpose_mul_vec(&r);
            let mut accepted = false;
            for _ in 0..40 {
                let mut a = jtj.clone();
                // Identity damping (scaled to the problem) rather than Marquardt's
                // diagonal scaling: sketches are usually under-determined, and identity
                // damping keeps the step in the row space of J, i.e. minimum-norm. Diagonal
                // scaling would let steps drift along the null space, where nonlinearity
                // then raises the cost and stalls the solve.
                let mu = lambda * (jtj_trace / n as f64).max(1e-12);
                for i in 0..n {
                    *a.at_mut(i, i) += mu;
                }
                let neg_jtr: Vec<f64> = jtr.iter().map(|v| -v).collect();
                let Some(step) = solve_spd(&a, &neg_jtr) else {
                    lambda *= 10.0;
                    continue;
                };
                let step_norm = step.iter().map(|v| v * v).sum::<f64>().sqrt();
                let trial: Vec<f64> = self.params.iter().zip(&step).map(|(p, s)| p + s).collect();
                let (r_trial, j_trial) = self.evaluate(&trial)?;
                let cost_trial: f64 = r_trial.iter().map(|v| v * v).sum();
                if cost_trial < cost || step_norm <= stall_step {
                    self.params = trial;
                    r = r_trial;
                    j = j_trial;
                    cost = cost_trial;
                    jac_for_rank = j.clone();
                    lambda = (lambda / 3.0).max(1e-12);
                    accepted = true;
                    if step_norm <= stall_step {
                        return Ok(self.outcome(iterations, &r, &jac_for_rank, false));
                    }
                    break;
                }
                lambda *= 4.0;
                if lambda > 1e16 {
                    break;
                }
            }
            if !accepted {
                // Every damping level failed to reduce the cost: we are at a (possibly
                // infeasible) minimum.
                return Ok(self.outcome(iterations, &r, &jac_for_rank, false));
            }
        }
        let converged = self.hard_norm(&r) <= tol;
        Ok(self.outcome(iterations, &r, &jac_for_rank, converged))
    }

    fn outcome(&self, iterations: usize, r: &[f64], j: &Mat, converged: bool) -> LmOutcome {
        let residual = self.hard_norm(r);
        let converged = converged || residual <= self.convergence_tolerance();
        LmOutcome {
            iterations,
            residual,
            converged,
            rank_matrix: j.clone(),
            conflicting: if converged {
                Vec::new()
            } else {
                self.conflicts(r)
            },
        }
    }

    /// The degrees of freedom left, and the entities they belong to: every point or
    /// circle with a parameter the constraints do not pin down.
    ///
    /// A count alone tells the user that something can still move but not what, and an
    /// under-constrained sketch looks exactly like a finished one — which is how geometry
    /// ends up drifting off the edge it was drawn against when an unrelated dimension
    /// changes. Naming the loose entities is what lets the editor draw them differently.
    fn freedom(&self, j: &Mat) -> (usize, Vec<EntityId>) {
        let (rank, free) = self.hard_rows(j).freedom(RANK_TOL);
        let mut out = Vec::new();
        for (slot, owner) in self.owners.iter().enumerate() {
            if free[slot] && !out.contains(owner) {
                out.push(*owner);
            }
        }
        (self.params.len().saturating_sub(rank), out)
    }

    /// The constraints that hold nothing the rest of the sketch does not already hold,
    /// in the order they were written.
    ///
    /// *Redundant* here means: every equation the constraint compiled is linearly
    /// dependent, at the solution, on the equations of the constraints ahead of it, while
    /// the sketch as a whole is satisfied. Deleting it therefore changes neither the
    /// geometry nor the degrees of freedom — it is over-constraint of the harmless kind,
    /// the sort a user creates by dimensioning a rectangle's fourth side or writing
    /// `Parallel` across a pair of sides already held by two `Horizontal`s.
    ///
    /// That is a different fault from `SolveError::DidNotConverge::conflicting`, and the
    /// two are mutually exclusive by construction. Conflicting constraints are
    /// *inconsistent*: the residual cannot be driven to zero, so there is no solution and
    /// the offenders are the rows still unsatisfied when the solver gave up. Redundant
    /// constraints are *consistent*: the residual is zero and the rows agree exactly —
    /// which is why the dependence has to be measured on the Jacobian rather than the
    /// residual, since a satisfied duplicate is invisible in the residual. A solve
    /// reports one or the other and never both: `conflicting` only ever reaches the user
    /// through the error of a solve that failed, `redundant` only through the report of
    /// one that succeeded. Note that the *same* duplicated dimension is conflicting when
    /// its two values differ and redundant when they agree.
    ///
    /// Which member of a dependent family gets named is decided by order: the equations
    /// the sketch implies rather than the user wrote (an arc's second radius) go in
    /// first and so are always driving, then the constraints in the order they were
    /// added, so it is the later of two interchangeable constraints that is reported.
    fn redundant(&self, j: &Mat) -> Vec<ConstraintId> {
        let hard = self.hard_rows(j);
        let mut spans: Vec<(Option<ConstraintId>, std::ops::Range<usize>)> = Vec::new();
        for (source, rows) in self.hard_row_spans() {
            // One group per constraint, not per equation: a constraint's own equations
            // must not be measured against each other.
            match spans.last_mut() {
                Some((s, prev)) if *s == source && prev.end == rows.start => prev.end = rows.end,
                _ => spans.push((source, rows)),
            }
        }
        let groups: Vec<std::ops::Range<usize>> = spans
            .iter()
            .filter(|(s, _)| s.is_none())
            .chain(spans.iter().filter(|(s, _)| s.is_some()))
            .map(|(_, rows)| rows.clone())
            .collect();
        let dependent = hard.dependent_rows(&groups, RANK_TOL);
        spans
            .into_iter()
            .filter_map(|(source, mut rows)| source.filter(|_| rows.all(|r| dependent[r])))
            .collect()
    }

    /// The hard rows of `j`. Soft rows are the drag goal, which is a preference rather
    /// than a constraint and so must not count toward what the sketch holds fixed.
    fn hard_rows(&self, j: &Mat) -> Mat {
        let hard = self.hard_row_count();
        Mat {
            rows: hard,
            cols: j.cols,
            data: j.data[..hard * j.cols].to_vec(),
        }
    }

    /// Writes solved parameters back into the sketch.
    fn write_back(&self, sketch: &mut Sketch) {
        for (id, data) in sketch.entities.iter_mut() {
            match &mut data.entity {
                Entity::Point { pos } => {
                    if let Some(&base) = self.index.get(&id) {
                        *pos = Vec2::new(self.params[base], self.params[base + 1]);
                    }
                }
                Entity::Circle { radius, .. } => {
                    if let Some(&base) = self.index.get(&id) {
                        // A negative radius is the same circle mirrored; normalise it.
                        *radius = self.params[base].abs();
                    }
                }
                _ => {}
            }
        }
    }

    /// The Jacobian of the hard equations at the current parameters, for testing.
    #[cfg(test)]
    pub(crate) fn jacobian(&self) -> Result<(Vec<f64>, Mat), SolveError> {
        self.evaluate(&self.params)
    }

    #[cfg(test)]
    pub(crate) fn params_mut(&mut self) -> &mut Vec<f64> {
        &mut self.params
    }
}

impl Equation {
    fn residual_count(&self) -> usize {
        match self.formula {
            Formula::PointsCoincide { .. }
            | Formula::Midpoint { .. }
            | Formula::Symmetric { .. }
            | Formula::Goal { .. } => 2,
            _ => 1,
        }
    }
}

struct LmOutcome {
    iterations: usize,
    residual: f64,
    converged: bool,
    rank_matrix: Mat,
    /// Constraints still unsatisfied, worst first; empty when the solve converged.
    conflicting: Vec<ConstraintId>,
}

/// Solves the sketch's constraints in place.
pub(crate) fn solve(sketch: &mut Sketch) -> Result<SolveReport, SolveError> {
    let mut sys = System::compile(sketch, &[])?;
    let out = sys.minimise(MAX_ITERATIONS)?;
    finish(sketch, &sys, out)
}

/// Drag: first pull every listed point toward its target with a soft goal, then remove
/// the goals and re-solve so the result satisfies the constraints exactly. Several
/// points at once is how a whole curve or selection is moved: each point gets the same
/// offset as a goal and the constraints decide how much of it survives.
pub(crate) fn drag(
    sketch: &mut Sketch,
    goals: &[(EntityId, Vec2)],
) -> Result<SolveReport, SolveError> {
    let mut soft = System::compile(sketch, goals)?;
    let first = soft.minimise(MAX_ITERATIONS)?;
    soft.write_back(sketch);
    let mut sys = System::compile(sketch, &[])?;
    let mut out = sys.minimise(MAX_ITERATIONS)?;
    out.iterations += first.iterations;
    finish(sketch, &sys, out)
}

fn finish(sketch: &mut Sketch, sys: &System, out: LmOutcome) -> Result<SolveReport, SolveError> {
    if !out.converged {
        return Err(SolveError::DidNotConverge {
            residual: out.residual,
            iterations: out.iterations,
            conflicting: out.conflicting,
        });
    }
    sys.write_back(sketch);
    let (degrees_of_freedom, under_constrained) = sys.freedom(&out.rank_matrix);
    Ok(SolveReport {
        iterations: out.iterations,
        residual: out.residual,
        converged: true,
        degrees_of_freedom,
        under_constrained,
        redundant: sys.redundant(&out.rank_matrix),
    })
}

/// Solver-side entry for tests: compile and expose Jacobian for finite-difference checks.
#[cfg(test)]
pub(crate) fn compile_for_test(sketch: &Sketch) -> System {
    System::compile(sketch, &[]).expect("compile")
}
