//! Offsetting sketch geometry: a second chain of curves running alongside an existing
//! one at a fixed distance.
//!
//! There are two things people mean by "offset", and on a rectangle they look quite
//! different:
//!
//! * [`Corner::Round`] puts every point of the result at exactly the distance from the
//!   source. An outward offset of a rectangle therefore has rounded corners: the set of
//!   points 5 mm from a corner *is* an arc, and there is nowhere else for them to be.
//!   This is Fusion's offset, and it is the one to use when the distance is a clearance.
//! * [`Corner::Miter`] puts every *edge* at exactly the distance from its own source
//!   edge and runs the edges out until they meet, so an offset rectangle is still a
//!   rectangle. Its corners are further than the distance from the source corners —
//!   that is the price of the square corner — and a sharp enough corner cannot be
//!   mitred at all, because the meeting point runs away to infinity ([`MITER_LIMIT`]).
//!
//! The seed must form a single connected chain of lines and arcs, or one circle on its
//! own. A branch — three curves meeting at a point — has no one answer, so it is
//! refused rather than guessed at.
//!
//! **What the result is tied to.** The offset is new geometry, not a linked copy: we
//! have no offset constraint to re-generate it from, and a copy that silently stopped
//! following its source would be worse than one that plainly never did. What it does
//! carry are the statements that are true of it by construction — each offset line is
//! parallel to its source, each offset arc concentric with its source, each rounded
//! corner centred on the corner it rounds, and joints that were smooth stay tangent —
//! so the result is driven rather than a loose pile of blue.

use std::f64::consts::{PI, TAU};

use basset_math::Vec2;

use crate::intersect::{line_circle, line_line};
use crate::sketch::JOIN_TOL;
use crate::{Constraint, Entity, EntityId, Sketch, SketchError};

/// How the offset closes the gap at a corner that opens up.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Corner {
    /// An arc of the offset distance, centred on the source corner. Every point of the
    /// result is the distance from the source.
    Round,
    /// The two edges run out to where they meet. Every edge is the distance from its
    /// source edge, and the shape keeps its corners.
    Miter,
}

impl Corner {
    pub fn name(self) -> &'static str {
        match self {
            Corner::Round => "Rounded corners",
            Corner::Miter => "Square corners",
        }
    }

    pub fn hint(self) -> &'static str {
        match self {
            Corner::Round => {
                "Every point of the result is the offset distance from the drawing; \
                 corners come out rounded"
            }
            Corner::Miter => {
                "Every edge is the offset distance from its own edge; the edges run out \
                 to meet, so corners stay square"
            }
        }
    }
}

/// How far a mitred corner may run out from the corner it replaces, as a multiple of the
/// offset distance. Past this the corner is so sharp that the mitre is nowhere near the
/// geometry it came from — the limit is about 11°, after which the result stops being
/// recognisable as an offset and rounded corners are what the user wants.
pub const MITER_LIMIT: f64 = 10.0;

/// How far a joint's tangents may differ and still count as smooth, in radians. Curves
/// drawn tangent are only tangent to the solver's convergence tolerance, so an exact
/// test would find a kink at every fillet and fit a microscopic arc into it.
const TANGENT_TOL: f64 = 1e-6;

/// Offsets `seed` by `distance`, returning the entities added.
///
/// `distance` is signed. Positive is outward for a closed chain or a circle — the side
/// that makes the shape bigger — and for an open chain it is the left of the chain's own
/// direction of travel, which is arbitrary from the user's side of the screen. The tool
/// that drives this needs no flip because of that sign: dragging its handle back across
/// the drawing, or typing a minus into its distance box, is the other side.
pub fn offset(
    sketch: &mut Sketch,
    seed: &[EntityId],
    distance: f64,
    corner: Corner,
) -> Result<Vec<EntityId>, SketchError> {
    if !distance.is_finite() || distance.abs() <= JOIN_TOL {
        return Err(SketchError::InvalidArgument(format!(
            "an offset needs a distance to offset by, got {distance}"
        )));
    }
    if let Some(circle) = lone_circle(sketch, seed)? {
        return offset_circle(sketch, circle, distance);
    }
    let chain = Chain::build(sketch, seed)?;
    // A closed chain is normalised counter-clockwise, so its outside is to the *right*
    // of travel and outward is the negative left-offset.
    let left = if chain.closed { -distance } else { distance };
    let offsets = chain.offsets(left);
    let joined = chain.join(&offsets, left, corner)?;
    crossing_free(&joined.pieces, chain.closed)?;
    emit(sketch, &chain, &joined)
}

/// The one circle of a seed that is one circle and nothing else. A circle alongside
/// another *curve* is an error, because a circle joins nothing and so has no chain it
/// could be part of; points and text alongside it are not, because a box drag picks them
/// up and they are not what the user was pointing at.
fn lone_circle(sketch: &Sketch, seed: &[EntityId]) -> Result<Option<EntityId>, SketchError> {
    let mut circle = None;
    let mut others = 0;
    for &id in seed {
        let data = sketch.entity(id).ok_or(SketchError::UnknownEntity(id))?;
        match data.entity {
            Entity::Circle { .. } => circle = Some(id),
            ref e if e.is_curve() => others += 1,
            _ => {}
        }
    }
    match circle {
        Some(id) if others == 0 => Ok(Some(id)),
        Some(_) => Err(SketchError::InvalidArgument(
            "a circle offsets on its own: it joins nothing, so it cannot be part of a \
             chain with other curves"
                .into(),
        )),
        None => Ok(None),
    }
}

/// Where a handle for dragging the offset belongs, and which way its distance grows from
/// there: `anchor + direction * distance` is the point of the result that answers to it.
///
/// The middle of the chain's first curve. The anchor is a point of the *source*, so it
/// stays put as the distance changes and the handle slides along one fixed line rather
/// than wandering out from under the pointer as it is dragged — and it is still there
/// when the distance is one the offset refuses, which is how the user drags back out of
/// a distance that does not fit.
pub fn handle(sketch: &Sketch, seed: &[EntityId]) -> Option<(Vec2, Vec2)> {
    if let Ok(Some(circle)) = lone_circle(sketch, seed) {
        let Entity::Circle { center, radius } = sketch.entity(circle)?.entity else {
            return None;
        };
        // Any radius of it would do; +x is where the rest of the crate starts an arc.
        return Some((sketch.point_pos(center)? + Vec2::X * radius, Vec2::X));
    }
    let chain = Chain::build(sketch, seed).ok()?;
    let (at, tangent) = chain.pieces.first()?.middle()?;
    // Positive is outward for a closed chain, whose outside is to the right of travel,
    // and the left of travel for an open one. Both are what `offset` itself means by it.
    Some((
        at,
        if chain.closed {
            -tangent.perp()
        } else {
            tangent.perp()
        },
    ))
}

/// Refuses a result whose own pieces run through each other.
///
/// Every corner can be right and the whole still be wrong: two parts of a shape that are
/// nowhere near each other in the chain can be nearer than twice the offset, and then
/// their offsets cross — a slit narrower than twice the distance closes over, a U tighter
/// than that has its arms pass through one another. Nothing local sees it, so it is
/// looked for at the end, on the finished curves.
fn crossing_free(pieces: &[Piece], closed: bool) -> Result<(), SketchError> {
    let n = pieces.len();
    let geoms: Vec<crate::CurveGeom> = pieces.iter().map(Piece::geom).collect();
    for a in 0..n {
        for b in (a + 1)..n {
            // Neighbours are *meant* to touch: that shared endpoint is the joint.
            let neighbours = b == a + 1 || (closed && a == 0 && b == n - 1);
            if neighbours {
                continue;
            }
            if !crate::intersect::intersections(&geoms[a], &geoms[b]).is_empty() {
                return Err(SketchError::InvalidArgument(
                    "that offset runs through itself: somewhere the drawing is narrower \
                     than the distance has room for, so two parts of the result cross. \
                     Offset less, or offset the other way"
                        .into(),
                ));
            }
        }
    }
    Ok(())
}

fn offset_circle(
    sketch: &mut Sketch,
    circle: EntityId,
    distance: f64,
) -> Result<Vec<EntityId>, SketchError> {
    let data = sketch
        .entity(circle)
        .ok_or(SketchError::UnknownEntity(circle))?;
    let construction = data.construction;
    let Entity::Circle { center, radius } = data.entity else {
        return Err(SketchError::UnknownEntity(circle));
    };
    let pos = sketch
        .point_pos(center)
        .ok_or(SketchError::UnknownEntity(center))?;
    let radius = radius + distance;
    if radius <= JOIN_TOL {
        return Err(too_far(Some(circle)));
    }
    let new_center = sketch.add_point(pos);
    let new = sketch.add_circle(new_center, radius)?;
    sketch.set_construction(new, construction)?;
    keep(sketch, Constraint::Concentric(new, circle));
    Ok(vec![new])
}

fn too_far(source: Option<EntityId>) -> SketchError {
    let what = match source {
        Some(id) => format!("curve {}", SketchError::entity_id_string(id)),
        None => "part of the chain".to_string(),
    };
    SketchError::InvalidArgument(format!(
        "that offset is larger than the geometry can carry: {what} is consumed by it. \
         Offset less, or offset the other way"
    ))
}

/// Adds a constraint that is true of the result by construction, and shrugs if the
/// sketch will not have it. It describes the offset rather than creating it, so a sketch
/// that is already saying the same thing another way loses nothing by refusing.
fn keep(sketch: &mut Sketch, c: Constraint) {
    if let Err(e) = sketch.add_constraint(c) {
        log::debug!("offset dropped a constraint: {e}");
    }
}

// ----- pieces -----------------------------------------------------------------------

/// One curve of a chain, in the direction the chain runs through it. Detached from the
/// sketch's points so the offset can be worked out whole before any of it is written
/// down.
#[derive(Clone, Copy, Debug)]
enum Piece {
    Line {
        a: Vec2,
        b: Vec2,
    },
    /// From `from` through `sweep` radians about `center`. A negative sweep runs
    /// clockwise, which is how a chain that happens to pass through an arc backwards is
    /// written down.
    Arc {
        center: Vec2,
        radius: f64,
        from: f64,
        sweep: f64,
    },
}

impl Piece {
    fn start(&self) -> Vec2 {
        match *self {
            Piece::Line { a, .. } => a,
            Piece::Arc {
                center,
                radius,
                from,
                ..
            } => center + Vec2::from_angle(from) * radius,
        }
    }

    fn end(&self) -> Vec2 {
        match *self {
            Piece::Line { b, .. } => b,
            Piece::Arc {
                center,
                radius,
                from,
                sweep,
            } => center + Vec2::from_angle(from + sweep) * radius,
        }
    }

    fn start_tangent(&self) -> Option<Vec2> {
        self.tangent_at(0.0)
    }

    fn end_tangent(&self) -> Option<Vec2> {
        match *self {
            Piece::Line { .. } => self.tangent_at(0.0),
            Piece::Arc { sweep, .. } => self.tangent_at(sweep),
        }
    }

    /// Unit direction of travel `delta` radians into the arc, or along the line.
    fn tangent_at(&self, delta: f64) -> Option<Vec2> {
        match *self {
            Piece::Line { a, b } => (b - a).try_normalize(),
            Piece::Arc { from, sweep, .. } => {
                Some(Vec2::from_angle(from + delta).perp() * sweep.signum())
            }
        }
    }

    /// The same curve `d` to the left of the direction of travel.
    fn offset(self, d: f64) -> Option<Piece> {
        match self {
            Piece::Line { a, b } => {
                let n = (b - a).try_normalize()?.perp();
                Some(Piece::Line {
                    a: a + n * d,
                    b: b + n * d,
                })
            }
            Piece::Arc {
                center,
                radius,
                from,
                sweep,
            } => {
                // Travelling counter-clockwise the centre is on the left, so offsetting
                // left draws the radius in; clockwise it pushes the radius out.
                let radius = radius - d * sweep.signum();
                (radius > JOIN_TOL).then_some(Piece::Arc {
                    center,
                    radius,
                    from,
                    sweep,
                })
            }
        }
    }

    /// Moves the start to `p`, keeping the end where it is. `p` is expected on the
    /// underlying line or circle; for an arc only the angle of it is used.
    fn set_start(&mut self, p: Vec2) {
        match self {
            Piece::Line { a, .. } => *a = p,
            Piece::Arc {
                center,
                from,
                sweep,
                ..
            } => {
                let moved = nearest_turn(*from, (p - *center).to_angle());
                *sweep += *from - moved;
                *from = moved;
            }
        }
    }

    fn set_end(&mut self, p: Vec2) {
        match self {
            Piece::Line { b, .. } => *b = p,
            Piece::Arc {
                center,
                from,
                sweep,
                ..
            } => *sweep = nearest_turn(*from + *sweep, (p - *center).to_angle()) - *from,
        }
    }

    fn length(&self) -> f64 {
        match *self {
            Piece::Line { a, b } => a.distance(b),
            Piece::Arc { radius, sweep, .. } => radius * sweep.abs(),
        }
    }

    fn is_arc(&self) -> bool {
        matches!(self, Piece::Arc { .. })
    }

    /// The point halfway along, and the direction of travel there.
    fn middle(&self) -> Option<(Vec2, Vec2)> {
        match *self {
            Piece::Line { a, b } => Some(((a + b) * 0.5, (b - a).try_normalize()?)),
            Piece::Arc {
                center,
                radius,
                from,
                sweep,
            } => {
                let at = from + sweep * 0.5;
                let radial = Vec2::from_angle(at);
                Some((center + radial * radius, radial.perp() * sweep.signum()))
            }
        }
    }

    /// The same shape as the rest of the crate writes it: always counter-clockwise, so a
    /// piece the chain runs through backwards is turned about first.
    fn geom(&self) -> crate::CurveGeom {
        match *self {
            Piece::Line { a, b } => crate::CurveGeom::Line { a, b },
            Piece::Arc {
                center,
                radius,
                from,
                sweep,
            } => crate::CurveGeom::Arc {
                center,
                radius,
                start_angle: if sweep >= 0.0 { from } else { from + sweep },
                sweep: sweep.abs(),
                closed: false,
            },
        }
    }

    /// Whether this piece still runs the way `plain` does. `plain` is the same curve
    /// before its corners were resolved, so disagreement means a trim went past the far
    /// end and turned the piece around.
    fn agrees_with(&self, plain: &Piece) -> bool {
        match (self, plain) {
            (Piece::Line { a, b }, Piece::Line { a: c, b: d }) => (*b - *a).dot(*d - *c) > 0.0,
            (Piece::Arc { sweep, .. }, Piece::Arc { sweep: was, .. }) => sweep * was > 0.0,
            _ => false,
        }
    }
}

/// The representative of `raw` nearest `old`, so nudging an arc's end by a fraction of a
/// degree does not accidentally wrap it the long way round the circle.
fn nearest_turn(old: f64, raw: f64) -> f64 {
    let mut delta = (raw - old).rem_euclid(TAU);
    if delta > PI {
        delta -= TAU;
    }
    old + delta
}

/// Where two offset curves meet, as the crossing of the line or circle each lies on,
/// taking the crossing nearest `near`. Extending past the drawn ends is the point: a
/// mitre is made of exactly that extension, and the nearest crossing to the corner being
/// replaced is the one that belongs to it.
fn meet(a: &Piece, b: &Piece, near: Vec2) -> Option<Vec2> {
    let candidates: Vec<Vec2> = match (a, b) {
        (Piece::Line { a: p0, b: p1 }, Piece::Line { a: q0, b: q1 }) => {
            line_line(*p0, *p1, *q0, *q1).into_iter().collect()
        }
        (Piece::Line { a: p0, b: p1 }, Piece::Arc { center, radius, .. })
        | (Piece::Arc { center, radius, .. }, Piece::Line { a: p0, b: p1 }) => {
            line_circle(*p0, *p1, *center, *radius)
        }
        (
            Piece::Arc {
                center: c0,
                radius: r0,
                ..
            },
            Piece::Arc {
                center: c1,
                radius: r1,
                ..
            },
        ) => crate::intersect::circle_circle(*c0, *r0, *c1, *r1),
    };
    candidates
        .into_iter()
        .min_by(|p, q| p.distance(near).total_cmp(&q.distance(near)))
}

// ----- the chain --------------------------------------------------------------------

/// The seed, ordered end to end and pointed one way. A closed chain is normalised
/// counter-clockwise so that "outward" means something.
struct Chain {
    pieces: Vec<Piece>,
    /// The curve each piece came from.
    sources: Vec<EntityId>,
    /// The source point shared at joint `k`, between piece `k` and the next, when the
    /// two curves really are built on one point rather than merely meeting there.
    joints: Vec<Option<EntityId>>,
    closed: bool,
}

impl Chain {
    fn build(sketch: &Sketch, seed: &[EntityId]) -> Result<Chain, SketchError> {
        let mut curves: Vec<EntityId> = Vec::new();
        for &id in seed {
            let data = sketch.entity(id).ok_or(SketchError::UnknownEntity(id))?;
            if !data.entity.is_open_curve() {
                // Points and text come along in a selection made by dragging a box; they
                // are silently ignored rather than made into a complaint about geometry
                // the user was not thinking about.
                if data.entity.is_point() || matches!(data.entity, Entity::Text { .. }) {
                    continue;
                }
                return Err(SketchError::WrongEntityKind {
                    id,
                    expected: "line or arc",
                    actual: data.entity.kind_name(),
                });
            }
            if !curves.contains(&id) {
                curves.push(id);
            }
        }
        if curves.is_empty() {
            return Err(SketchError::InvalidArgument(
                "an offset needs lines, arcs or a circle to offset".into(),
            ));
        }
        let ends: Vec<(Vec2, Vec2)> = curves
            .iter()
            .map(|&id| {
                sketch
                    .curve_endpoints(id)
                    .ok_or(SketchError::UnknownEntity(id))
            })
            .collect::<Result<_, _>>()?;
        for (i, (a, b)) in ends.iter().enumerate() {
            if a.distance(*b) <= JOIN_TOL && !sketch.entity(curves[i]).is_some_and(is_arc_data) {
                return Err(SketchError::DegenerateGeometry(
                    "a curve of zero length cannot be offset".into(),
                ));
            }
        }
        let order = walk(&ends)?;
        let closed = order.len() > 1 && touches(&ends, order[order.len() - 1], order[0]);
        let mut chain = Chain::assemble(sketch, &curves, &ends, &order, closed)?;
        // Counter-clockwise is what makes "outward" a direction rather than a coin toss.
        if closed && chain.area() < 0.0 {
            let flipped: Vec<(usize, bool)> = order.iter().rev().map(|(i, f)| (*i, !f)).collect();
            chain = Chain::assemble(sketch, &curves, &ends, &flipped, closed)?;
        }
        Ok(chain)
    }

    fn assemble(
        sketch: &Sketch,
        curves: &[EntityId],
        ends: &[(Vec2, Vec2)],
        order: &[(usize, bool)],
        closed: bool,
    ) -> Result<Chain, SketchError> {
        let mut pieces = Vec::with_capacity(order.len());
        let mut sources = Vec::with_capacity(order.len());
        for &(i, forward) in order {
            pieces.push(piece_of(sketch, curves[i], ends[i], forward)?);
            sources.push(curves[i]);
        }
        let joint_count = if closed {
            order.len()
        } else {
            order.len().saturating_sub(1)
        };
        let mut joints = Vec::with_capacity(joint_count);
        for k in 0..joint_count {
            let (i, forward) = order[k];
            let (j, next_forward) = order[(k + 1) % order.len()];
            let leaving = end_point(sketch, curves[i], forward)?;
            let arriving = end_point(sketch, curves[j], !next_forward)?;
            joints.push((leaving == arriving).then_some(leaving));
        }
        Ok(Chain {
            pieces,
            sources,
            joints,
            closed,
        })
    }

    /// Signed area of the closed chain: chords by the shoelace formula, plus the
    /// circular segment each arc adds to or takes from its chord.
    fn area(&self) -> f64 {
        self.pieces
            .iter()
            .map(|p| {
                let (s, e) = (p.start(), p.end());
                let chord = (s.x * e.y - e.x * s.y) * 0.5;
                match *p {
                    Piece::Arc { radius, sweep, .. } => {
                        chord + radius * radius * 0.5 * (sweep - sweep.sin())
                    }
                    Piece::Line { .. } => chord,
                }
            })
            .sum()
    }

    /// Every piece moved `left` to the left of travel, before the corners are sorted out.
    ///
    /// A convex curve tighter than the offset has no offset at all: it shrinks past a
    /// point and out of existence. That is not the shape running out of room — a
    /// filleted rectangle offset in by more than its fillet radius is an ordinary thing
    /// to ask for, and the answer is the same rectangle with square corners — so its
    /// place is `None` and its neighbours are joined to each other instead.
    fn offsets(&self, left: f64) -> Vec<Option<Piece>> {
        self.pieces.iter().map(|p| p.offset(left)).collect()
    }

    /// Resolves every corner, returning the finished run of pieces, what each came from,
    /// and the indices of the joints smooth enough to be worth saying so.
    fn join(
        &self,
        offsets: &[Option<Piece>],
        left: f64,
        corner: Corner,
    ) -> Result<Joined, SketchError> {
        // Curves the offset swallowed whole drop out, and the chain closes over them.
        let live: Vec<usize> = (0..offsets.len())
            .filter(|i| offsets[*i].is_some())
            .collect();
        if live.len() < if self.closed { 2 } else { 1 } {
            return Err(too_far(self.sources.first().copied()));
        }
        let plain: Vec<Piece> = live.iter().map(|i| offsets[*i].expect("live")).collect();
        let mut fixed = plain.clone();
        let n = fixed.len();
        let joint_count = if self.closed { n } else { n - 1 };
        let mut joins: Vec<Join> = Vec::with_capacity(joint_count);
        for k in 0..joint_count {
            let j = (k + 1) % n;
            let (before, after) = (live[k], live[j]);
            let (tail, head) = (self.pieces[before].end(), self.pieces[after].start());
            // The same point unless something between them was swallowed, in which case
            // the middle of what it left behind is the corner the neighbours meet at.
            let corner_at = (tail + head) * 0.5;
            let (Some(t1), Some(t2)) = (
                self.pieces[before].end_tangent(),
                self.pieces[after].start_tangent(),
            ) else {
                return Err(SketchError::DegenerateGeometry(
                    "a curve with no direction cannot be offset".into(),
                ));
            };
            let cross = t1.perp_dot(t2);
            let dot = t1.dot(t2);
            // Smooth: the two offsets already end in the same place, so there is nothing
            // to close and nothing to trim.
            if cross.abs() <= TANGENT_TOL && dot > 0.0 {
                let welded = fixed[k].end();
                fixed[j].set_start(welded);
                joins.push(Join::Smooth);
                continue;
            }
            // A corner that opens away from the side being offset to leaves a gap; the
            // other way round the two offsets run over each other and want trimming.
            //
            // At a cusp, where the chain doubles back, `cross` is zero and its sign is
            // whatever the rounding happened to leave, so it cannot be asked which way
            // the corner turns. A cusp is a gap on both sides — the two offsets sit on
            // opposite sides of the source and can never overlap — so it is named as one
            // rather than left to the noise.
            let cusp = cross.abs() <= TANGENT_TOL && dot < 0.0;
            let gap = cusp || cross * left < 0.0;
            if gap && corner == Corner::Round {
                let turn = if cusp {
                    // A half turn; the side being offset to says which way round it goes.
                    -PI * left.signum()
                } else {
                    cross.atan2(dot)
                };
                joins.push(Join::Round(Piece::Arc {
                    center: corner_at,
                    radius: left.abs(),
                    from: (fixed[k].end() - corner_at).to_angle(),
                    sweep: turn,
                }));
                continue;
            }
            let hit = meet(&fixed[k], &fixed[j], corner_at).ok_or_else(|| {
                SketchError::InvalidArgument(
                    "two edges of that offset never meet, so the corner between them has \
                     nowhere to go; try rounded corners or a smaller distance"
                        .into(),
                )
            })?;
            if gap && hit.distance(corner_at) > MITER_LIMIT * left.abs() {
                return Err(SketchError::InvalidArgument(format!(
                    "that corner is too sharp to square off at {:.3} mm: the edges would \
                     meet {:.1} mm out from it. Use rounded corners instead",
                    left.abs(),
                    hit.distance(corner_at)
                )));
            }
            fixed[k].set_end(hit);
            fixed[j].set_start(hit);
            joins.push(Join::Cut);
        }
        // A piece trimmed past its own far end has been turned round: the corners either
        // side of it have crossed, and the "offset" is a bow tie rather than a smaller
        // copy of the drawing. It is the shape running out of room, and it is what an
        // inward offset bigger than the shape is thick does.
        for (k, (fixed, plain)) in fixed.iter().zip(&plain).enumerate() {
            if fixed.length() <= JOIN_TOL || !fixed.agrees_with(plain) {
                return Err(too_far(Some(self.sources[live[k]])));
            }
        }
        // Weave the rounded corners in between the pieces they join.
        let mut pieces = Vec::with_capacity(n + joins.len());
        let mut sources = Vec::with_capacity(n + joins.len());
        let mut smooth = Vec::new();
        for (k, piece) in fixed.into_iter().enumerate() {
            pieces.push(piece);
            sources.push(Source::Curve(self.sources[live[k]]));
            match joins.get(k) {
                // A rounded corner meets both its neighbours tangentially, so both the
                // joint before it and the joint after it are smooth.
                Some(Join::Round(arc)) => {
                    smooth.push(pieces.len() - 1);
                    pieces.push(*arc);
                    sources.push(Source::Corner(
                        self.corner_point(live[k], live[(k + 1) % n]),
                    ));
                    smooth.push(pieces.len() - 1);
                }
                Some(Join::Smooth) => smooth.push(pieces.len() - 1),
                Some(Join::Cut) | None => {}
            }
        }
        Ok(Joined {
            pieces,
            sources,
            smooth,
        })
    }

    /// The source point two pieces of the chain meet at, when they really are next to
    /// each other. Once a swallowed curve has been dropped from between them there is no
    /// one source point any more, and the corner is centred on nothing in particular.
    fn corner_point(&self, before: usize, after: usize) -> Option<EntityId> {
        (after == (before + 1) % self.pieces.len())
            .then(|| self.joints.get(before).copied().flatten())
            .flatten()
    }
}

/// The finished run of curves, worked out whole before any of it is written down.
struct Joined {
    pieces: Vec<Piece>,
    /// What each piece is the offset of.
    sources: Vec<Source>,
    /// Indices `k` where piece `k` meets the one after it smoothly.
    smooth: Vec<usize>,
}

/// How one corner of the offset came out.
enum Join {
    /// The source was smooth there, so the offsets already met.
    Smooth,
    /// A gap closed by this arc about the source corner.
    Round(Piece),
    /// Both sides run to where they cross: a mitre outside the corner, or a trim inside.
    Cut,
}

/// What an emitted piece is the offset of, so it can be told what it is.
#[derive(Clone, Copy, Debug)]
enum Source {
    Curve(EntityId),
    /// A rounded corner, centred on this source point when the chain has one there.
    Corner(Option<EntityId>),
}

impl Source {
    /// The curve this piece is the offset of; a rounded corner is the offset of no one
    /// curve but of the point where two of them meet.
    fn curve(self) -> Option<EntityId> {
        match self {
            Source::Curve(id) => Some(id),
            Source::Corner(_) => None,
        }
    }
}

fn is_arc_data(data: &crate::EntityData) -> bool {
    matches!(data.entity, Entity::Arc { .. })
}

fn touches(ends: &[(Vec2, Vec2)], from: (usize, bool), to: (usize, bool)) -> bool {
    let leaving = if from.1 {
        ends[from.0].1
    } else {
        ends[from.0].0
    };
    let arriving = if to.1 { ends[to.0].0 } else { ends[to.0].1 };
    leaving.distance(arriving) <= JOIN_TOL
}

/// Orders the curves end to end. The result names each curve and whether the chain runs
/// through it the way it was drawn.
fn walk(ends: &[(Vec2, Vec2)]) -> Result<Vec<(usize, bool)>, SketchError> {
    let n = ends.len();
    let degree = |p: Vec2| {
        ends.iter()
            .flat_map(|(a, b)| [*a, *b])
            .filter(|q| q.distance(p) <= JOIN_TOL)
            .count()
    };
    for (a, b) in ends {
        if degree(*a) > 2 || degree(*b) > 2 {
            return Err(SketchError::InvalidArgument(
                "three curves meet at one of those points, so there is no one chain to \
                 offset; pick the curves of a single path or loop"
                    .into(),
            ));
        }
    }
    // A free end starts an open chain; a loop has none, so it starts anywhere.
    let start = ends
        .iter()
        .enumerate()
        .find_map(|(i, (a, b))| match (degree(*a), degree(*b)) {
            (1, _) => Some((i, true)),
            (_, 1) => Some((i, false)),
            _ => None,
        })
        .unwrap_or((0, true));
    let mut order = vec![start];
    let mut used = vec![false; n];
    used[start.0] = true;
    let mut at = if start.1 {
        ends[start.0].1
    } else {
        ends[start.0].0
    };
    while order.len() < n {
        let next = (0..n).find(|&j| {
            !used[j] && (at.distance(ends[j].0) <= JOIN_TOL || at.distance(ends[j].1) <= JOIN_TOL)
        });
        let Some(j) = next else {
            return Err(SketchError::InvalidArgument(
                "those curves do not all join up into one chain; offset one path or loop \
                 at a time"
                    .into(),
            ));
        };
        let forward = at.distance(ends[j].0) <= JOIN_TOL;
        order.push((j, forward));
        used[j] = true;
        at = if forward { ends[j].1 } else { ends[j].0 };
    }
    Ok(order)
}

fn piece_of(
    sketch: &Sketch,
    id: EntityId,
    ends: (Vec2, Vec2),
    forward: bool,
) -> Result<Piece, SketchError> {
    let entity = &sketch
        .entity(id)
        .ok_or(SketchError::UnknownEntity(id))?
        .entity;
    let (a, b) = if forward {
        (ends.0, ends.1)
    } else {
        (ends.1, ends.0)
    };
    match *entity {
        Entity::Line { .. } => Ok(Piece::Line { a, b }),
        Entity::Arc { center, .. } => {
            let center = sketch
                .point_pos(center)
                .ok_or(SketchError::UnknownEntity(center))?;
            let radius = (a - center).length();
            if radius <= JOIN_TOL {
                return Err(SketchError::DegenerateGeometry(
                    "an arc of zero radius cannot be offset".into(),
                ));
            }
            let from = (a - center).to_angle();
            // The entity is counter-clockwise from its own start; running the chain
            // through it backwards makes the sweep negative and everything downstream
            // reads the direction of travel off that sign.
            let ccw = crate::tessellation::ccw_sweep(from, (b - center).to_angle());
            Ok(Piece::Arc {
                center,
                radius,
                from,
                sweep: if forward { ccw } else { ccw - TAU },
            })
        }
        ref other => Err(SketchError::WrongEntityKind {
            id,
            expected: "line or arc",
            actual: other.kind_name(),
        }),
    }
}

/// The point entity a curve ends at, in the chain's direction of travel.
fn end_point(sketch: &Sketch, id: EntityId, forward: bool) -> Result<EntityId, SketchError> {
    match sketch
        .entity(id)
        .ok_or(SketchError::UnknownEntity(id))?
        .entity
    {
        Entity::Line { start, end } | Entity::Arc { start, end, .. } => {
            Ok(if forward { end } else { start })
        }
        ref other => Err(SketchError::WrongEntityKind {
            id,
            expected: "line or arc",
            actual: other.kind_name(),
        }),
    }
}

// ----- writing it down --------------------------------------------------------------

fn emit(sketch: &mut Sketch, chain: &Chain, joined: &Joined) -> Result<Vec<EntityId>, SketchError> {
    let Joined {
        pieces,
        sources,
        smooth,
    } = joined;
    let n = pieces.len();
    // One point per joint, shared by the pieces either side of it, so the result is a
    // chain in the same sense the source is and a later drag moves both sides together.
    let mut joint_ids: Vec<EntityId> = Vec::with_capacity(n + 1);
    if !chain.closed {
        joint_ids.push(sketch.add_point(pieces[0].start()));
    }
    for piece in pieces {
        joint_ids.push(sketch.add_point(piece.end()));
    }
    let start_of = |k: usize| -> EntityId {
        if chain.closed {
            joint_ids[(k + n - 1) % n]
        } else {
            joint_ids[k]
        }
    };
    let end_of = |k: usize| -> EntityId {
        if chain.closed {
            joint_ids[k]
        } else {
            joint_ids[k + 1]
        }
    };
    let mut created = Vec::with_capacity(n);
    for (k, piece) in pieces.iter().enumerate() {
        let id = match *piece {
            Piece::Line { .. } => sketch.add_line(start_of(k), end_of(k))?,
            Piece::Arc { center, sweep, .. } => {
                let c = sketch.add_point(center);
                // Entity::Arc is always counter-clockwise from its start, so a piece the
                // chain runs through backwards is written down the other way about.
                if sweep >= 0.0 {
                    sketch.add_arc(c, start_of(k), end_of(k))?
                } else {
                    sketch.add_arc(c, end_of(k), start_of(k))?
                }
            }
        };
        created.push(id);
    }
    // Construction geometry offsets to construction geometry; a rounded corner takes
    // after the edge it follows.
    for (k, id) in created.iter().enumerate() {
        // A rounded corner takes after the edge it follows, which is the piece before it.
        let from = sources[k]
            .curve()
            .or_else(|| k.checked_sub(1).and_then(|prev| sources[prev].curve()));
        let construction = from
            .and_then(|c| sketch.entity(c))
            .is_some_and(|d| d.construction);
        sketch.set_construction(*id, construction)?;
    }
    for (k, id) in created.iter().enumerate() {
        match sources[k] {
            Source::Curve(source) if pieces[k].is_arc() => {
                keep(sketch, Constraint::Concentric(*id, source));
            }
            Source::Curve(source) => keep(sketch, Constraint::Parallel(*id, source)),
            Source::Corner(Some(point)) => {
                // The arc rounds that corner, so its centre is that corner.
                let Entity::Arc { center, .. } = sketch
                    .entity(*id)
                    .ok_or(SketchError::UnknownEntity(*id))?
                    .entity
                else {
                    continue;
                };
                keep(
                    sketch,
                    Constraint::Coincident {
                        point: center,
                        target: point,
                    },
                );
            }
            Source::Corner(None) => {}
        }
    }
    for &k in smooth {
        let j = (k + 1) % n;
        // Tangency is only sayable when a curve is involved; two lines meeting smoothly
        // are one line, and the sketch has no way to write that down.
        if pieces[k].is_arc() || pieces[j].is_arc() {
            keep(sketch, Constraint::Tangent(created[k], created[j]));
        }
    }
    Ok(created)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shapes;
    use approx::assert_relative_eq;
    use std::f64::consts::PI;

    /// A 40 × 20 rectangle at the origin, counter-clockwise from its bottom-left corner.
    fn rect(s: &mut Sketch) -> Vec<EntityId> {
        shapes::rectangle_two_point(s, Vec2::ZERO, Vec2::new(40.0, 20.0))
            .lines
            .to_vec()
    }

    /// Area enclosed by the result, read back through the same chain builder the offset
    /// used, so the test measures the sketch rather than the arithmetic that made it.
    fn enclosed(sketch: &Sketch, ids: &[EntityId]) -> f64 {
        Chain::build(sketch, ids)
            .expect("the result is a chain")
            .area()
    }

    fn radii(sketch: &Sketch, ids: &[EntityId]) -> Vec<f64> {
        ids.iter()
            .filter_map(|id| match sketch.entity(*id)?.entity {
                Entity::Arc { center, start, .. } => {
                    Some(sketch.point_pos(start)?.distance(sketch.point_pos(center)?))
                }
                _ => None,
            })
            .collect()
    }

    #[test]
    fn a_rounded_offset_of_a_rectangle_has_rounded_corners() {
        let mut s = Sketch::new();
        let seed = rect(&mut s);
        let made = offset(&mut s, &seed, 5.0, Corner::Round).expect("offsets");
        // Four edges pushed out and four quarter-circles filling the corners.
        assert_eq!(made.len(), 8);
        let arcs = radii(&s, &made);
        assert_eq!(arcs.len(), 4);
        for r in arcs {
            assert_relative_eq!(r, 5.0, epsilon = 1e-9);
        }
        // A stadium-cornered rectangle: the original, a band all the way round, and the
        // four quarter-circles that make up one whole circle of the offset radius.
        assert_relative_eq!(
            enclosed(&s, &made),
            800.0 + 2.0 * 60.0 * 5.0 + PI * 25.0,
            epsilon = 1e-6
        );
    }

    #[test]
    fn every_point_of_a_rounded_offset_is_the_offset_distance_away() {
        let mut s = Sketch::new();
        let seed = rect(&mut s);
        let made = offset(&mut s, &seed, 5.0, Corner::Round).expect("offsets");
        let tess = crate::Tessellation::default();
        for id in &made {
            for p in s.curve_polyline(*id, &tess).expect("a polyline") {
                let nearest = seed
                    .iter()
                    .filter_map(|e| s.entity_distance(*e, p))
                    .fold(f64::INFINITY, f64::min);
                assert_relative_eq!(nearest, 5.0, epsilon = 1e-6);
            }
        }
    }

    #[test]
    fn a_squared_offset_of_a_rectangle_is_a_bigger_rectangle() {
        let mut s = Sketch::new();
        let seed = rect(&mut s);
        let made = offset(&mut s, &seed, 5.0, Corner::Miter).expect("offsets");
        assert_eq!(
            made.len(),
            4,
            "no corners are added, the edges run out to meet"
        );
        assert!(radii(&s, &made).is_empty());
        // 50 × 30: every edge exactly 5 from its own edge, which is what squaring costs.
        assert_relative_eq!(enclosed(&s, &made), 1500.0, epsilon = 1e-9);
    }

    #[test]
    fn a_negative_offset_goes_inward() {
        let mut s = Sketch::new();
        let seed = rect(&mut s);
        let made = offset(&mut s, &seed, -5.0, Corner::Miter).expect("offsets");
        assert_relative_eq!(enclosed(&s, &made), 30.0 * 10.0, epsilon = 1e-9);
        // Inward, the corners close rather than open, so rounding has nothing to round.
        let mut s = Sketch::new();
        let seed = rect(&mut s);
        let made = offset(&mut s, &seed, -5.0, Corner::Round).expect("offsets");
        assert_eq!(made.len(), 4);
    }

    #[test]
    fn an_offset_bigger_than_the_shape_is_refused_rather_than_turned_inside_out() {
        let mut s = Sketch::new();
        let seed = rect(&mut s);
        // The rectangle is 20 tall, so 11 in from both sides crosses over itself.
        let e = offset(&mut s, &seed, -11.0, Corner::Miter).expect_err("cannot fit");
        assert!(
            e.to_string().contains("larger than the geometry can carry"),
            "{e}"
        );
        assert_eq!(
            s.entities().count(),
            8,
            "a refused offset leaves the sketch alone"
        );
    }

    #[test]
    fn a_circle_offsets_to_a_circle() {
        let mut s = Sketch::new();
        let c = shapes::circle_center(&mut s, Vec2::new(3.0, 4.0), 10.0);
        let out = offset(&mut s, &[c.circle], 5.0, Corner::Round).expect("offsets");
        assert_eq!(out.len(), 1);
        let Entity::Circle { center, radius } = s.entity(out[0]).expect("made").entity else {
            panic!("a circle offsets to a circle");
        };
        assert_relative_eq!(radius, 15.0);
        assert_relative_eq!(s.point_pos(center).expect("centred").x, 3.0);
        // Inward past its own centre there is no circle left to draw.
        assert!(offset(&mut s, &[c.circle], -10.0, Corner::Round).is_err());
    }

    #[test]
    fn an_open_chain_offsets_to_an_open_chain() {
        let mut s = Sketch::new();
        // An L: right along x, then up.
        let p = shapes::polyline(
            &mut s,
            &[Vec2::ZERO, Vec2::new(20.0, 0.0), Vec2::new(20.0, 20.0)],
            false,
        );
        // Left of travel is the inside of this L, so a negative distance is its outside.
        let round = offset(&mut s, &p.lines, -4.0, Corner::Round).expect("offsets");
        assert_eq!(round.len(), 3, "the outside of the L gains a corner arc");
        let mut s = Sketch::new();
        let p = shapes::polyline(
            &mut s,
            &[Vec2::ZERO, Vec2::new(20.0, 0.0), Vec2::new(20.0, 20.0)],
            false,
        );
        let flipped = offset(&mut s, &p.lines, 4.0, Corner::Round).expect("offsets");
        assert_eq!(flipped.len(), 2, "the inside of the L closes up instead");
    }

    #[test]
    fn an_arc_offsets_concentrically_and_stays_tangent() {
        let mut s = Sketch::new();
        let slot = shapes::slot_center_to_center(&mut s, Vec2::ZERO, Vec2::new(30.0, 0.0), 10.0);
        let seed = vec![slot.lines[0], slot.arcs[1], slot.lines[1], slot.arcs[0]];
        let made = offset(&mut s, &seed, 3.0, Corner::Round).expect("offsets");
        // A slot offsets to a slot: the caps grow, the flats move out, nothing is added.
        assert_eq!(made.len(), 4);
        for r in radii(&s, &made) {
            assert_relative_eq!(r, 8.0, epsilon = 1e-9);
        }
        assert_relative_eq!(enclosed(&s, &made), 30.0 * 16.0 + PI * 64.0, epsilon = 1e-6);
        assert!(
            s.solve().is_ok_and(|r| r.converged),
            "the result solves as drawn"
        );
    }

    #[test]
    fn the_offset_says_what_it_is() {
        let mut s = Sketch::new();
        let seed = rect(&mut s);
        let made = offset(&mut s, &seed, 5.0, Corner::Round).expect("offsets");
        let parallel = s
            .constraints()
            .filter(|(_, c)| matches!(c, Constraint::Parallel(a, _) if made.contains(a)))
            .count();
        assert_eq!(parallel, 4, "each offset edge is parallel to its own edge");
        let corners = s
            .constraints()
            .filter(|(_, c)| matches!(c, Constraint::Coincident { .. }))
            .count();
        assert_eq!(
            corners, 4,
            "each rounded corner is centred on the corner it rounds"
        );
        let tangent = s
            .constraints()
            .filter(|(_, c)| matches!(c, Constraint::Tangent(..)))
            .count();
        assert_eq!(
            tangent, 8,
            "every corner arc meets both its neighbours smoothly"
        );
        assert!(
            s.solve().is_ok_and(|r| r.converged),
            "and all of it is consistent"
        );
    }

    #[test]
    fn a_branch_has_no_one_answer_and_is_refused() {
        let mut s = Sketch::new();
        let t = shapes::polyline(
            &mut s,
            &[Vec2::new(-10.0, 0.0), Vec2::ZERO, Vec2::new(10.0, 0.0)],
            false,
        );
        let stem = s.add_point(Vec2::ZERO);
        let up = s.add_point(Vec2::new(0.0, 10.0));
        let branch = s.add_line(stem, up).expect("a line");
        let mut seed = t.lines.clone();
        seed.push(branch);
        let e = offset(&mut s, &seed, 2.0, Corner::Round).expect_err("ambiguous");
        assert!(e.to_string().contains("three curves meet"), "{e}");
    }

    #[test]
    fn curves_that_do_not_join_up_are_refused() {
        let mut s = Sketch::new();
        let a = shapes::polyline(&mut s, &[Vec2::ZERO, Vec2::new(10.0, 0.0)], false);
        let b = shapes::polyline(&mut s, &[Vec2::new(50.0, 0.0), Vec2::new(60.0, 0.0)], false);
        let seed = [a.lines[0], b.lines[0]];
        let e = offset(&mut s, &seed, 2.0, Corner::Round).expect_err("two chains");
        assert!(e.to_string().contains("do not all join up"), "{e}");
    }

    #[test]
    fn a_corner_too_sharp_to_square_off_says_so() {
        let mut s = Sketch::new();
        // A two-degree wedge: mitring it would put the corner metres away.
        let tip = Vec2::ZERO;
        let far = Vec2::new(100.0, 0.0);
        let back = Vec2::from_angle(2f64.to_radians()) * 100.0;
        let p = shapes::polyline(&mut s, &[far, tip, back], false);
        let e = offset(&mut s, &p.lines, 5.0, Corner::Miter).expect_err("too sharp");
        assert!(e.to_string().contains("too sharp to square off"), "{e}");
        // Rounded corners have no such trouble: the gap is filled rather than crossed.
        assert!(offset(&mut s, &p.lines, 5.0, Corner::Round).is_ok());
    }

    /// A fillet tighter than the offset simply disappears, and the edges either side of
    /// it meet each other. Refusing this would refuse an offset of almost any real part:
    /// a filleted rectangle taken in by more than its fillet radius is a square-cornered
    /// rectangle, not an impossibility.
    #[test]
    fn a_curve_the_offset_swallows_drops_out_and_its_neighbours_meet() {
        let mut s = Sketch::new();
        // A 40 x 20 rectangle with 3 mm corners, counter-clockwise.
        let r = 3.0;
        let (w, h) = (40.0, 20.0);
        let corners = [
            (Vec2::new(r, r), Vec2::new(w - r, r)),
            (Vec2::new(w - r, r), Vec2::new(w - r, h - r)),
            (Vec2::new(w - r, h - r), Vec2::new(r, h - r)),
            (Vec2::new(r, h - r), Vec2::new(r, r)),
        ];
        let mut seed = Vec::new();
        for (i, (from, to)) in corners.iter().enumerate() {
            let out = [Vec2::NEG_Y, Vec2::X, Vec2::Y, Vec2::NEG_X][i];
            let next = [Vec2::X, Vec2::Y, Vec2::NEG_X, Vec2::NEG_Y][i];
            let a = s.add_point(*from + out * r);
            let b = s.add_point(*to + out * r);
            seed.push(s.add_line(a, b).expect("a flat"));
            let centre = s.add_point(*to);
            let end = s.add_point(*to + next * r);
            seed.push(s.add_arc(centre, b, end).expect("a corner"));
        }
        // 8 in from a 20 mm side leaves 4 mm across, and the 3 mm corners are long gone.
        let made = offset(&mut s, &seed, -8.0, Corner::Round).expect("offsets");
        assert_eq!(made.len(), 4, "the corners went; the flats met each other");
        assert!(radii(&s, &made).is_empty());
        assert_relative_eq!(enclosed(&s, &made), 24.0 * 4.0, epsilon = 1e-9);
    }

    /// At a cusp the turn is a half circle and `cross` is zero, so which way the corner
    /// goes cannot be read off its sign — it is whatever the rounding left. Both sides of
    /// a cusp are a gap, and rounded corners fill both.
    #[test]
    fn a_cusp_is_rounded_on_either_side_of_it() {
        for distance in [3.0, -3.0] {
            let mut s = Sketch::new();
            // Out to (10, 0) and most of the way back: a hairpin, doubling back on itself.
            let out = shapes::polyline(&mut s, &[Vec2::ZERO, Vec2::new(10.0, 0.0)], false);
            let back =
                shapes::polyline(&mut s, &[Vec2::new(10.0, 0.0), Vec2::new(2.0, 0.0)], false);
            let seed = [out.lines[0], back.lines[0]];
            let made = offset(&mut s, &seed, distance, Corner::Round)
                .unwrap_or_else(|e| panic!("offsetting a cusp by {distance}: {e}"));
            assert_eq!(made.len(), 3, "both flats and the half turn joining them");
            let arcs = radii(&s, &made);
            assert_eq!(arcs.len(), 1);
            assert_relative_eq!(arcs[0], 3.0, epsilon = 1e-9);
        }
        // A cusp cannot be squared off: the two edges are parallel and never meet.
        let mut s = Sketch::new();
        let out = shapes::polyline(&mut s, &[Vec2::ZERO, Vec2::new(10.0, 0.0)], false);
        let back = shapes::polyline(&mut s, &[Vec2::new(10.0, 0.0), Vec2::new(2.0, 0.0)], false);
        let e = offset(&mut s, &[out.lines[0], back.lines[0]], 3.0, Corner::Miter)
            .expect_err("a cusp has no square corner");
        assert!(e.to_string().contains("never meet"), "{e}");
    }

    /// Every corner can be right and the whole still be wrong. A slit narrower than twice
    /// the offset closes over: the two walls cross, neither of them reverses, and nothing
    /// local notices.
    #[test]
    fn an_offset_that_runs_through_itself_is_refused() {
        let mut s = Sketch::new();
        // A square with a 4 mm slit cut into it from the right-hand edge.
        let p = shapes::polyline(
            &mut s,
            &[
                Vec2::new(60.0, 0.0),
                Vec2::new(60.0, 28.0),
                Vec2::new(40.0, 28.0),
                Vec2::new(40.0, 20.0),
                Vec2::new(20.0, 20.0),
                Vec2::new(20.0, 40.0),
                Vec2::new(40.0, 40.0),
                Vec2::new(40.0, 32.0),
                Vec2::new(60.0, 32.0),
                Vec2::new(60.0, 60.0),
                Vec2::new(0.0, 60.0),
                Vec2::ZERO,
            ],
            true,
        );
        for corner in [Corner::Miter, Corner::Round] {
            // 3 mm from each wall of a 4 mm slit: the walls pass through each other.
            let e = offset(&mut s, &p.lines, 3.0, corner).expect_err("crosses itself");
            assert!(e.to_string().contains("runs through itself"), "{e}");
        }
        // 1 mm still fits down the slit, so it is not refused out of caution.
        assert!(offset(&mut s, &p.lines, 1.0, Corner::Miter).is_ok());
    }

    #[test]
    fn a_circle_still_offsets_when_its_centre_point_came_along() {
        let mut s = Sketch::new();
        let c = shapes::circle_center(&mut s, Vec2::ZERO, 10.0);
        // A box drag round a circle picks up its centre point too, and that is not a
        // reason to tell the user their circle is part of a chain.
        let made = offset(&mut s, &[c.circle, c.center], 2.0, Corner::Round).expect("offsets");
        assert_eq!(made.len(), 1);
    }

    /// The handle is anchored on the source and points the way the distance grows, so
    /// `anchor + direction * distance` lands on the result — which is what lets a drag
    /// of the handle *be* the distance instead of standing in for it.
    #[test]
    fn the_handle_sits_where_the_result_will_be() {
        let mut s = Sketch::new();
        let seed = rect(&mut s);
        let (anchor, dir) = handle(&s, &seed).expect("a handle");
        for distance in [5.0, -5.0, 12.5] {
            let mut s = s.clone();
            let made = offset(&mut s, &seed, distance, Corner::Miter).expect("offsets");
            let want = anchor + dir * distance;
            let nearest = made
                .iter()
                .filter_map(|id| s.entity_distance(*id, want))
                .fold(f64::INFINITY, f64::min);
            assert!(nearest < 1e-9, "at {distance}: {nearest} from {want:?}");
        }
        // Outward for a closed loop drawn counter-clockwise: away from the middle.
        assert!(
            (anchor + dir * 5.0).distance(Vec2::new(20.0, 10.0))
                > anchor.distance(Vec2::new(20.0, 10.0)),
            "a positive distance grows the shape"
        );
    }

    #[test]
    fn a_circle_has_a_handle_on_its_rim() {
        let mut s = Sketch::new();
        let c = shapes::circle_center(&mut s, Vec2::new(3.0, 4.0), 10.0);
        let (anchor, dir) = handle(&s, &[c.circle]).expect("a handle");
        assert_relative_eq!(anchor.distance(Vec2::new(3.0, 4.0)), 10.0);
        assert_relative_eq!((anchor + dir * 5.0).distance(Vec2::new(3.0, 4.0)), 15.0);
    }

    #[test]
    fn an_offset_of_nothing_is_refused_before_anything_is_made() {
        let mut s = Sketch::new();
        let seed = rect(&mut s);
        assert!(offset(&mut s, &seed, 0.0, Corner::Round).is_err());
        assert!(offset(&mut s, &[], 5.0, Corner::Round).is_err());
        assert_eq!(s.entities().count(), 8);
    }
}
