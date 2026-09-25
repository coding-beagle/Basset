//! The one rule for how a dragged number meets the grid.
//!
//! Every CAD package the user has already learned behaves the same way: a drag lands on
//! the grid, and holding shift lets go of it for as long as it is held. That is one
//! rule, but it has to hold for every handle there is — the sketch's move arrows, the
//! offset's slider, the extrude and blend arrows, the body move gizmo. Written out at
//! each of those it would drift: one handle would round its total, another its
//! increment, a third would forget shift entirely, and the user would learn that
//! snapping works "sometimes". So the rule lives here and the handles ask it.
//!
//! [`Snapping`] is the state the rule is read from — the persistent toggle plus the
//! modifier. [`Snap`] is that state resolved against the grid increment in force at the
//! point being dragged, and is what the arithmetic goes through.
//!
//! # Inference
//!
//! The grid is the floor, not the ceiling. What makes sketching feel read rather than
//! typed is that the pointer also finds the places the *drawing* already names: the
//! middle of a line, the centre of a hole, where two curves cross, the point level with
//! the corner three clicks ago, the line carried on past its end. [`Inference`] is where
//! those are found, ranked and held, and it is asked exactly like the grid is — by the
//! drawing and dragging paths, never reimplemented beside them.
//!
//! **Ranking.** Several candidates are nearly always in range at once, so the order is
//! fixed and written down in [`SnapKind::tier`]: an existing point beats a place the
//! geometry implies (midpoint, crossing), which beats the crossing of two guides, which
//! beats a point *on* a curve, which beats a guide line, which beats the grid. Within a
//! tier the nearer one wins. One ranking, read once, so "why did it take that?" always
//! has the same answer.
//!
//! **Hysteresis.** A snap the pointer jitters in and out of is worse than no snap: the
//! preview flickers between two answers and the user stops trusting either. So a snap,
//! once acquired within the pick radius, is *held* out to [`HOLD_FACTOR`] times it, and
//! a rival in the same tier must be nearer by [`BREAK_MARGIN`] of the radius before it
//! takes over. Only a stronger tier displaces a held snap on distance alone, and even
//! then it must be properly acquired. The hold is the single biggest contributor to how
//! the tool feels, which is why it lives in one pure function rather than in the
//! pointer handler.
//!
//! **Shift.** Shift means what it has always meant and a little more: it lets go of
//! every *computed* place — the grid, the guides, tangent and perpendicular, midpoints,
//! crossings, the origin — because those are exactly the things standing between the
//! pointer and a position of its own. What it does not let go of is a *join*: an
//! existing point (a curve's endpoint, a circle's centre) to be shared outright, or a
//! curve to be held onto with a constraint. That is the README's rule, extended to the
//! one other thing that attaches geometry rather than merely placing it, and it is the
//! rule because a point that only *looks* attached is a bug the user cannot see.
//!
//! The palette's snapping checkbox says the same thing for good: off, only the joins
//! are left. It is described to the user as the master switch, and inference is exactly
//! the kind of help it is switching off — someone who has turned the grid off to draw a
//! shape by eye did not ask for the drawing's own axes to catch them instead.
//!
//! **And the grid still counts.** A guide is only worth taking if it is nearer the
//! pointer than the grid line the click would otherwise land on. Without that a guide
//! from a point a moment ago beats every grid intersection near it, and a drawing made
//! by eye stops coming out with round numbers — which is the thing the grid was for.
//! Geometry that is really there (a point, a midpoint, a crossing, a curve) is exempt:
//! it is a place in the model, not a suggestion.

use std::collections::HashMap;

use basset_math::{Vec2, Vec3};
use basset_sketch::{CurveGeom, Entity, EntityId, JOIN_TOL, Sketch, intersect};
use basset_viewport::Camera;

/// Rotations snap to whole steps of this, for the same reason positions snap to the
/// grid: a ring dragged to 37.4° is almost never what was meant.
pub const ANGLE_STEP_DEG: f64 = 5.0;

/// The persistent toggle plus the modifier: whether the drawing is built on a grid at
/// all, and whether shift is letting go of it right now.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Snapping {
    /// The master switch, shown as the sketch palette's checkbox. Off means no handle
    /// anywhere snaps.
    pub to_grid: bool,
    /// Shift is down. Frees the live drag whatever the toggle says, and is never
    /// remembered: it lasts exactly as long as the key.
    pub free: bool,
}

impl Default for Snapping {
    /// On, because snapping by default is what the user arrived expecting.
    fn default() -> Self {
        Self {
            to_grid: true,
            free: false,
        }
    }
}

impl Snapping {
    /// Whether the grid is holding at this instant.
    pub fn on(self) -> bool {
        self.to_grid && !self.free
    }

    /// The rule for one drag, at the grid increment in force where it is happening.
    pub fn at(self, step: f64) -> Snap {
        Snap {
            step: self
                .on()
                .then_some(step)
                .filter(|s| s.is_finite() && *s > 0.0),
        }
    }
}

/// The running total of one drag, kept unrounded.
///
/// A drag does not arrive as a gesture; it arrives as one small delta per frame. Adding
/// each delta to the *snapped* value and rounding again throws the remainder away sixty
/// times a second: with a 1 mm grid and a pointer moving a tenth of a millimetre per
/// frame, every frame rounds back to where it started and the handle never moves at all,
/// while the mouse walks off without it. Only a fast enough flick — one frame carrying
/// more than half a step — ever registers, which is why such a handle feels like it
/// snaps "sometimes".
///
/// So the raw total is kept here and the grid is applied to *that*. The model still only
/// ever holds snapped values; what this remembers is what the pointer asked for, which
/// is the thing the next frame has to add to. Releasing forgets it, so a later drag
/// starts from where the value actually is.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Drag {
    raw: Option<f64>,
}

impl Drag {
    /// Advances the drag by `by` and hands back the value to show. `value` is what the
    /// model holds, used to start the gesture off when this is the first frame of it.
    pub fn advance(&mut self, value: f64, by: f64, snap: Snap) -> f64 {
        let raw = self.raw.unwrap_or(value) + by;
        self.raw = Some(raw);
        snap.value(raw)
    }

    /// Advances, but not below `floor` — a radius is a size, and the raw total is pinned
    /// too so that dragging further back does not bank travel the grip never made.
    pub fn advance_above(&mut self, value: f64, by: f64, snap: Snap, floor: f64) -> f64 {
        let at = self.advance(value, by, snap);
        if at < floor {
            self.raw = Some(floor);
            return floor;
        }
        at
    }

    /// Advances within `floor..=ceiling` — a blend's radius has both a smallest size
    /// and the most the material allows, and the handle stops at each. The raw total is
    /// pinned at whichever end stopped it, for the same reason [`Self::advance_above`]
    /// pins at its floor: travel the grip never made must not be banked, or pulling
    /// back off the stop would first have to retrace it.
    pub fn advance_within(
        &mut self,
        value: f64,
        by: f64,
        snap: Snap,
        floor: f64,
        ceiling: f64,
    ) -> f64 {
        let at = self.advance(value, by, snap);
        let held = at.clamp(floor, ceiling);
        if held != at {
            self.raw = Some(held);
        }
        held
    }

    /// The same for an angle, which rounds to whole steps rather than to the grid.
    pub fn advance_angle(&mut self, value: f64, by: f64, snap: Snap) -> f64 {
        let raw = self.raw.unwrap_or(value) + by;
        self.raw = Some(raw);
        snap.angle_deg(raw)
    }

    /// Ends the gesture. The next one starts from wherever the value ended up, so a
    /// drag, a typed number and another drag compose the way the user expects.
    pub fn release(&mut self) {
        self.raw = None;
    }
}

/// [`Snapping`] resolved against a grid increment: either an increment to round to, or
/// nothing at all. Built by [`Snapping::at`] and passed to whatever is being dragged, so
/// a handle cannot accidentally consult a different toggle than the one the user sees.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Snap {
    step: Option<f64>,
}

impl Snap {
    pub fn is_on(self) -> bool {
        self.step.is_some()
    }

    /// The increment in force, for the feedback to name.
    pub fn step(self) -> Option<f64> {
        self.step
    }

    /// A dragged scalar — a distance, a radius, an offset — rounded to the grid.
    ///
    /// The *total* is rounded rather than each increment of it, so a slow drag walks
    /// between grid lines instead of accumulating a rounding error per mouse event and
    /// ending up between them.
    pub fn value(self, v: f64) -> f64 {
        match self.step {
            Some(step) => (v / step).round() * step,
            None => v,
        }
    }

    /// A dragged position in the plane, rounded to the grid on both axes.
    pub fn point(self, p: Vec2) -> Vec2 {
        Vec2::new(self.value(p.x), self.value(p.y))
    }

    /// A dragged angle, in degrees, rounded to [`ANGLE_STEP_DEG`]. The grid's increment
    /// is a length and means nothing to a rotation, so only the on/off part carries.
    pub fn angle_deg(self, deg: f64) -> f64 {
        if self.is_on() {
            (deg / ANGLE_STEP_DEG).round() * ANGLE_STEP_DEG
        } else {
            deg
        }
    }
}

/// What the viewport says about a drag while it is happening: the value the handle has
/// landed on, and where. Lives for one frame, set by whichever handle is being dragged.
///
/// Without it a snapped drag is indistinguishable from a sticky one — the number jumps
/// and nothing says why — and a freed drag is indistinguishable from a broken snap.
#[derive(Clone, Debug)]
pub struct Hint {
    /// Where the handle is, in world space; the label sits beside it and the grid marker
    /// on it.
    pub at: Vec3,
    /// The value as it now reads, already snapped.
    pub text: String,
    /// Whether the grid held this drag. False while shift is down or the toggle is off,
    /// and the label says so rather than going quiet.
    pub on_grid: bool,
}

impl Hint {
    /// A hint for a scalar handle: the value, its unit, and the increment it landed on.
    pub fn value(at: Vec3, value: f64, unit: &str, snap: Snap) -> Self {
        let text = match snap.step() {
            Some(step) => format!("{value:.2}{unit}  ·  grid {step}"),
            None => format!("{value:.2}{unit}  ·  free"),
        };
        Self {
            at,
            text,
            on_grid: snap.is_on(),
        }
    }
}

/// Screen offset of the label from the handle it belongs to, in egui points. Below and
/// right, so it never sits under the pointer that is dragging the handle.
const LABEL_OFFSET: egui::Vec2 = egui::vec2(16.0, 12.0);

/// Draws the hint's label beside its handle. Deliberately plain: a drag is a moment of
/// concentration and a decorated tooltip in the middle of it is noise.
pub fn paint(ctx: &egui::Context, camera: &Camera, window: [u32; 2], hint: &Hint) {
    let Some(px) = camera.world_to_screen(hint.at, window) else {
        return;
    };
    let ppp = f64::from(ctx.pixels_per_point());
    let at = egui::pos2((px[0] / ppp) as f32, (px[1] / ppp) as f32) + LABEL_OFFSET;
    // A freed drag reads in the same grey as everything else; a snapped one is tinted,
    // so "the grid took this" is legible without reading the words.
    let color = if hint.on_grid {
        egui::Color32::from_rgb(210, 230, 255)
    } else {
        egui::Color32::from_gray(190)
    };
    let painter = ctx.layer_painter(egui::LayerId::new(
        egui::Order::Foreground,
        egui::Id::new("snap-hint"),
    ));
    let galley = painter.layout_no_wrap(hint.text.clone(), egui::FontId::proportional(12.0), color);
    let rect = egui::Rect::from_min_size(at, galley.size()).expand(4.0);
    painter.rect_filled(
        rect,
        3.0,
        egui::Color32::from_rgba_unmultiplied(15, 20, 30, 190),
    );
    painter.galley(at, galley, color);
}

// --- Inference ----------------------------------------------------------------------

/// How far past the pick radius an acquired snap is still held, as a multiple of it.
/// Large enough that a hand resting on the mouse cannot shake a snap off, small enough
/// that walking away from one lets go before the pointer is somewhere else entirely.
pub const HOLD_FACTOR: f64 = 1.8;
/// How much nearer a rival in the *same* tier must be before it takes a held snap over,
/// as a fraction of the pick radius. Without it two points a pixel apart trade the snap
/// back and forth with every tremor.
pub const BREAK_MARGIN: f64 = 0.35;
/// How much of the pick radius a *computed* place gets, as against one the sketch
/// holds. An endpoint is a thing the user pointed at and 8 px of slack is right for it;
/// a midpoint, a crossing or an alignment is something the software thought of, and at
/// the same radius the drawing's own axes would catch every click near them and the
/// grid would become unreachable. Half is about four pixels: easy to land on when
/// wanted, easy to stay off when not.
const INFERRED_FRACTION: f64 = 0.5;
/// Guide directions closer to parallel than this (as `|sin|` of the angle between them)
/// have no usable crossing: the intersection runs off to infinity and moves a metre for
/// a pixel of pointer travel.
const GUIDE_CROSS_MIN: f64 = 0.2;
/// How many curves near the pointer are paired up for crossings. Crossings are O(n²) in
/// this and the pointer is only ever near a handful; the cap is what stops a dense
/// drawing paying for the whole sketch on every frame.
const NEAR_CURVES: usize = 8;
/// How many recently touched points grow alignment guides. Three is what a drawing hand
/// can keep track of; more and the screen fills with dashes that mean nothing.
const RECENT_POINTS: usize = 3;

/// What the pointer landed on, and therefore what the marker drawn on it says.
///
/// The order of the variants is not the ranking — [`SnapKind::tier`] is, and it is
/// written out there so that adding a kind forces a decision about where it sits.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum SnapKind {
    /// A point entity that is neither an endpoint nor a centre: a loose point.
    Point,
    /// An endpoint of a line or arc.
    Endpoint,
    /// The centre of a circle or arc.
    Center,
    /// The sketch's origin.
    Origin,
    /// Halfway along a line or arc. Not an entity: a place the geometry implies.
    Midpoint,
    /// Where two curves cross, whether or not anything is drawn there.
    Intersection,
    /// Where two guide lines cross — the one place two inferences agree.
    GuideCross,
    /// The nearest place *on* a curve. The point gets held there by a constraint.
    OnCurve,
    /// Level with a point the user recently touched.
    Horizontal,
    /// Directly above or below one.
    Vertical,
    /// Out along an existing line's own direction, past its end.
    Extension,
    /// Carrying on tangentially from the curve the drawing continues from.
    Tangent,
    /// Square to it.
    Perpendicular,
}

impl SnapKind {
    /// The ranking, lowest first. A point of the drawing beats a place the drawing
    /// implies, which beats two guides agreeing, which beats a curve, which beats a
    /// single guide; the grid is not here at all because it is the fallback rather than
    /// a candidate — it is what happens when nothing was inferred.
    pub fn tier(self) -> u8 {
        match self {
            SnapKind::Point | SnapKind::Endpoint | SnapKind::Center | SnapKind::Origin => 0,
            SnapKind::Midpoint | SnapKind::Intersection => 1,
            SnapKind::GuideCross => 2,
            SnapKind::OnCurve => 3,
            SnapKind::Horizontal
            | SnapKind::Vertical
            | SnapKind::Extension
            | SnapKind::Tangent
            | SnapKind::Perpendicular => 4,
        }
    }

    /// How near the pointer has to be for this to be acquired, from the pick radius the
    /// hit test uses. See [`INFERRED_FRACTION`] for why the two differ.
    pub fn pick_radius(self, tol: f64) -> f64 {
        match self {
            SnapKind::Point | SnapKind::Endpoint | SnapKind::Center | SnapKind::OnCurve => tol,
            _ => tol * INFERRED_FRACTION,
        }
    }

    /// Whether this is a *join* — something that attaches the new point to geometry
    /// that is already there, by sharing its entity or by earning a constraint onto it.
    /// Joins are what shift and the palette's switch never give up; see the module
    /// header for why.
    pub fn joins(self) -> bool {
        matches!(
            self,
            SnapKind::Point | SnapKind::Endpoint | SnapKind::Center | SnapKind::OnCurve
        )
    }

    /// Whether the grid gets to overrule this when the grid line is the nearer of the
    /// two. True for the guides, which are suggestions; false for anything the drawing
    /// actually holds.
    pub fn yields_to_grid(self) -> bool {
        matches!(
            self,
            SnapKind::GuideCross
                | SnapKind::Horizontal
                | SnapKind::Vertical
                | SnapKind::Extension
                | SnapKind::Tangent
                | SnapKind::Perpendicular
        )
    }
}

/// An inferred line the pointer can be caught on, drawn dashed from `anchor` so the
/// user can see *which* point or curve is doing the catching.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Guide {
    pub kind: SnapKind,
    /// The geometry the guide comes from: a touched point, a line's end, the origin.
    pub anchor: Vec2,
    /// Unit direction along the guide.
    pub dir: Vec2,
}

impl Guide {
    /// The nearest point on the guide to `p`, and how far off it `p` is.
    fn foot(self, p: Vec2) -> (Vec2, f64) {
        let along = (p - self.anchor).dot(self.dir);
        let foot = self.anchor + self.dir * along;
        (foot, foot.distance(p))
    }

    fn key(self) -> (SnapKind, i64, i64, i64, i64) {
        (
            self.kind,
            quantise(self.anchor.x),
            quantise(self.anchor.y),
            quantise(self.dir.x),
            quantise(self.dir.y),
        )
    }
}

/// Positions are compared for identity at a tenth of a micron: far below anything the
/// solver or the user can mean, far above the float noise of re-deriving a guide from
/// the same geometry next frame.
fn quantise(v: f64) -> i64 {
    (v * 1e4).round() as i64
}

/// One place the pointer could land, with everything the caller needs to both use it
/// and draw it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Candidate {
    pub kind: SnapKind,
    pub at: Vec2,
    /// An existing point entity to share outright, which is what joins a loop.
    pub point: Option<EntityId>,
    /// A curve the position lies on, which earns the new point a `Coincident` rather
    /// than leaving it where the pointer happened to be.
    pub curve: Option<EntityId>,
    /// The guides that caught this, to be drawn. Two only for [`SnapKind::GuideCross`].
    pub guides: [Option<Guide>; 2],
}

/// What makes two candidates from consecutive frames *the same* snap. Positions move by
/// float noise as the geometry is re-derived, so identity is the thing snapped to, not
/// where it currently is.
pub type SnapKey = (
    SnapKind,
    Option<EntityId>,
    Option<EntityId>,
    [Option<(SnapKind, i64, i64, i64, i64)>; 2],
    Option<(i64, i64)>,
);

impl Candidate {
    fn new(kind: SnapKind, at: Vec2) -> Self {
        Self {
            kind,
            at,
            point: None,
            curve: None,
            guides: [None, None],
        }
    }

    pub fn key(&self) -> SnapKey {
        // A crossing of two curves is named by nothing but where it is — it belongs to
        // no entity and no guide — so its position joins the key. Everything else is
        // identified by what it is *of*, which is what lets a held snap survive the
        // geometry under it shifting by a hair as the sketch re-solves.
        let anonymous = self.point.is_none() && self.curve.is_none() && self.guides[0].is_none();
        (
            self.kind,
            self.point,
            self.curve,
            [
                self.guides[0].map(Guide::key),
                self.guides[1].map(Guide::key),
            ],
            anonymous.then(|| (quantise(self.at.x), quantise(self.at.y))),
        )
    }
}

/// Where the point being placed is continuing from: the previous click, or the end of
/// the chain being drawn, together with the curve it came off. Tangent and
/// perpendicular mean nothing without it, which is why it is asked for rather than
/// guessed.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Continuation {
    pub from: Option<Vec2>,
    pub curve: Option<EntityId>,
    /// A point being dragged, and so not a place to snap to: it is under the pointer by
    /// definition, and the curves hanging off it follow it about, so neither says
    /// anything about where it should land.
    pub moving: Option<EntityId>,
}

/// The ranking and the hold, and the short memory of points the pointer has touched
/// that alignment guides grow from.
///
/// Everything here is ordinary arithmetic over a [`Sketch`], so the behaviour that
/// decides how snapping feels is testable without a window.
#[derive(Clone, Debug, Default)]
pub struct Inference {
    held: Option<SnapKey>,
    current: Option<Candidate>,
    recent: Vec<Vec2>,
}

impl Inference {
    /// Resolves the pointer against the sketch. `tol` is the pick radius in millimetres
    /// (the same screen-space tolerance the hit test uses); `joins_only` is shift or the
    /// palette's switch being off; `grid` is where the click would land without any of
    /// this, which the guides have to beat to be worth taking.
    ///
    /// `None` means nothing was inferred and the caller should fall back to the grid,
    /// which is the one rule this module already owned.
    pub fn resolve(
        &mut self,
        sketch: &Sketch,
        cont: Continuation,
        pointer: Vec2,
        tol: f64,
        joins_only: bool,
        grid: Option<Vec2>,
    ) -> Option<Candidate> {
        let candidates = gather(sketch, &self.recent, cont, pointer, tol, joins_only);
        let chosen = choose(&candidates, self.held, pointer, tol, grid).map(|i| candidates[i]);
        // Hovering a point is touching it: the guides the *next* point lines up with
        // grow from wherever the pointer has just been resting, which is what makes
        // "level with that corner" available without asking for it.
        if let Some(c) = chosen.filter(|c| c.kind.tier() == 0) {
            self.touch(c.at);
        }
        self.held = chosen.map(|c| c.key());
        self.current = chosen;
        chosen
    }

    /// What the last resolve landed on, for the overlay to draw.
    pub fn current(&self) -> Option<&Candidate> {
        self.current.as_ref()
    }

    /// Remembers a point the user touched — hovered a snap on, or clicked — so that
    /// alignment with it is inferred from here on. Newest first, deduplicated, and
    /// bounded: guides the user cannot account for are noise.
    pub fn touch(&mut self, p: Vec2) {
        self.recent.retain(|q| q.distance(p) > JOIN_TOL);
        self.recent.insert(0, p);
        self.recent.truncate(RECENT_POINTS);
    }

    /// Lets go of the held snap without forgetting the touched points: called when the
    /// gesture ends or the tool changes, so the next one acquires from scratch rather
    /// than inheriting a hold the user cannot see the reason for.
    pub fn release(&mut self) {
        self.held = None;
        self.current = None;
    }
}

/// The ranking and the hold, as one pure function over candidates already gathered.
///
/// Returns the index of the winner, or `None` when nothing is within reach. A candidate
/// is *acquired* within `tol` and *held* out to `tol * HOLD_FACTOR`, so `held` is what
/// makes the difference between a snap the pointer is leaving and one it never had.
pub fn choose(
    candidates: &[Candidate],
    held: Option<SnapKey>,
    pointer: Vec2,
    tol: f64,
    grid: Option<Vec2>,
) -> Option<usize> {
    // A guide further from the pointer than the grid line underneath it is not help.
    let worth_it = |kind: SnapKind, d: f64| match grid {
        Some(g) if kind.yields_to_grid() => d < pointer.distance(g),
        _ => true,
    };
    let held_at = held.and_then(|key| {
        candidates
            .iter()
            .enumerate()
            .map(|(i, c)| (i, c, pointer.distance(c.at)))
            .filter(|(_, c, d)| {
                c.key() == key
                    && *d <= c.kind.pick_radius(tol) * HOLD_FACTOR
                    && worth_it(c.kind, *d)
            })
            .min_by(|a, b| a.2.total_cmp(&b.2))
            .map(|(i, c, d)| (i, c.kind, d))
    });
    // The challenger is the best thing properly acquired this frame: within the pick
    // radius, ranked by tier and then by distance.
    let best = candidates
        .iter()
        .enumerate()
        .map(|(i, c)| (i, c.kind, pointer.distance(c.at)))
        .filter(|(_, kind, d)| *d <= kind.pick_radius(tol) && worth_it(*kind, *d))
        .min_by(|a, b| {
            a.1.tier()
                .cmp(&b.1.tier())
                .then_with(|| a.2.total_cmp(&b.2))
        });
    match (held_at, best) {
        (Some((hi, hk, hd)), Some((bi, bk, bd))) => {
            // A rival in the same tier has to be nearer by a margin of the held snap's
            // own radius; a stronger tier only has to be properly acquired, which the
            // filter above has already decided.
            let margin = hk.pick_radius(tol) * BREAK_MARGIN;
            let takes_over = bk.tier() < hk.tier() || (bk.tier() == hk.tier() && bd + margin < hd);
            Some(if takes_over { bi } else { hi })
        }
        (Some((hi, _, _)), None) => Some(hi),
        (None, best) => best.map(|(i, _, _)| i),
    }
}

/// Everything within `reach` of the pointer that the drawing names.
///
/// Free of any state: what it returns depends only on the sketch, the points the user
/// has touched and where the pointer is, which is what makes the ranking above testable
/// on its own.
pub fn gather(
    sketch: &Sketch,
    recent: &[Vec2],
    cont: Continuation,
    pointer: Vec2,
    tol: f64,
    joins_only: bool,
) -> Vec<Candidate> {
    // Gathered out to the hold radius rather than the pick radius: a candidate has to
    // still be in the list for the hold to have anything to hold on to.
    let reach = |kind: SnapKind| kind.pick_radius(tol) * HOLD_FACTOR;
    let mut out: Vec<Candidate> = Vec::new();
    let roles = point_roles(sketch);
    let follows_the_pointer = |id: EntityId| match cont.moving {
        Some(moving) => {
            id == moving
                || sketch
                    .entity(id)
                    .is_some_and(|e| e.entity.references().contains(&moving))
        }
        None => false,
    };
    for (id, data) in sketch.entities() {
        if follows_the_pointer(id) {
            continue;
        }
        if let Entity::Point { pos } = data.entity {
            let kind = roles.get(&id).copied().unwrap_or(SnapKind::Point);
            if pos.distance(pointer) > reach(kind) {
                continue;
            }
            let mut c = Candidate::new(kind, pos);
            c.point = Some(id);
            out.push(c);
        }
    }
    // The origin is not an entity of the sketch, but it is the one place every drawing
    // shares and the thing a first dimension is nearly always taken from.
    if Vec2::ZERO.distance(pointer) <= reach(SnapKind::Origin)
        && !out.iter().any(|c| c.at.length() <= JOIN_TOL)
    {
        out.push(Candidate::new(SnapKind::Origin, Vec2::ZERO));
    }

    let mut near = near_curves(sketch, pointer, reach(SnapKind::OnCurve));
    near.retain(|(id, _)| !follows_the_pointer(*id));
    for &(id, geom) in &near {
        if let Some(on) = sketch.closest_point_on(id, pointer)
            && on.distance(pointer) <= reach(SnapKind::OnCurve)
        {
            let mut c = Candidate::new(SnapKind::OnCurve, on);
            c.curve = Some(id);
            out.push(c);
        }
        // A full circle has no midpoint to speak of: every point on it is halfway
        // round from somewhere, so offering one would be arbitrary.
        if !geom.is_closed() {
            let mid = geom.point_at(0.5);
            if mid.distance(pointer) <= reach(SnapKind::Midpoint) {
                let mut c = Candidate::new(SnapKind::Midpoint, mid);
                c.curve = Some(id);
                out.push(c);
            }
        }
    }
    if joins_only {
        // Shift, or the switch off: only what attaches the point to geometry that is
        // already there survives, and everything below here is inference.
        out.retain(|c| c.kind.joins());
        return out;
    }
    for (i, &(_, a)) in near.iter().enumerate() {
        for &(_, b) in &near[i + 1..] {
            for p in intersect::intersections(&a, &b) {
                if p.distance(pointer) <= reach(SnapKind::Intersection) {
                    out.push(Candidate::new(SnapKind::Intersection, p));
                }
            }
        }
    }

    let guides = guides(sketch, recent, cont);
    let mut caught: Vec<(Guide, Vec2)> = Vec::new();
    for g in guides {
        let (foot, off) = g.foot(pointer);
        // A foot sitting on the anchor is the anchor, which is already a candidate of a
        // stronger tier; offering it again as a guide would only confuse the marker.
        if off <= reach(g.kind) && foot.distance(g.anchor) > JOIN_TOL {
            caught.push((g, foot));
        }
    }
    for (i, &(g, foot)) in caught.iter().enumerate() {
        let mut c = Candidate::new(g.kind, foot);
        c.guides[0] = Some(g);
        out.push(c);
        // Two guides agreeing is the strongest thing an inference can say short of a
        // real point: it names a position in both axes rather than one.
        for &(h, _) in &caught[i + 1..] {
            if g.dir.perp_dot(h.dir).abs() < GUIDE_CROSS_MIN {
                continue;
            }
            if let Some(p) = cross_of(g, h)
                && p.distance(pointer) <= reach(SnapKind::GuideCross)
            {
                let mut c = Candidate::new(SnapKind::GuideCross, p);
                c.guides = [Some(g), Some(h)];
                out.push(c);
            }
        }
    }
    out
}

/// What each point entity *is* to the curves built on it, which is what its marker says
/// and — because a centre is a more deliberate thing to point at than an end that
/// happens to sit there — which of two names it takes when it is both.
fn point_roles(sketch: &Sketch) -> HashMap<EntityId, SnapKind> {
    let mut roles: HashMap<EntityId, SnapKind> = HashMap::new();
    let mut mark = |id: EntityId, kind: SnapKind| {
        let slot = roles.entry(id).or_insert(kind);
        if kind == SnapKind::Center {
            *slot = kind;
        }
    };
    for (_, data) in sketch.entities() {
        match data.entity {
            Entity::Line { start, end } => {
                mark(start, SnapKind::Endpoint);
                mark(end, SnapKind::Endpoint);
            }
            Entity::Arc { center, start, end } => {
                mark(start, SnapKind::Endpoint);
                mark(end, SnapKind::Endpoint);
                mark(center, SnapKind::Center);
            }
            Entity::Circle { center, .. } => mark(center, SnapKind::Center),
            Entity::Point { .. } | Entity::Text { .. } => {}
        }
    }
    roles
}

/// The curves worth considering, nearest first and capped at [`NEAR_CURVES`].
fn near_curves(sketch: &Sketch, pointer: Vec2, reach: f64) -> Vec<(EntityId, CurveGeom)> {
    let mut near: Vec<(EntityId, CurveGeom, f64)> = sketch
        .entities()
        .filter_map(|(id, _)| {
            let geom = CurveGeom::of(sketch, id)?;
            let d = sketch.entity_distance(id, pointer)?;
            (d <= reach).then_some((id, geom, d))
        })
        .collect();
    near.sort_by(|a, b| a.2.total_cmp(&b.2));
    near.truncate(NEAR_CURVES);
    near.into_iter().map(|(id, g, _)| (id, g)).collect()
}

/// Every guide line on offer: the two axes through each point the user has touched and
/// through the origin, the continuation of each line those points belong to, and the
/// tangent and normal of the curve the drawing is continuing from.
fn guides(sketch: &Sketch, recent: &[Vec2], cont: Continuation) -> Vec<Guide> {
    let mut out: Vec<Guide> = Vec::new();
    let x = Vec2::new(1.0, 0.0);
    let y = Vec2::new(0.0, 1.0);
    // The point being drawn from counts as touched whether or not it was hovered: it is
    // the one the user is most obviously working relative to.
    let anchors: Vec<Vec2> = cont
        .from
        .into_iter()
        .chain(recent.iter().copied())
        .chain(std::iter::once(Vec2::ZERO))
        .collect();
    for a in &anchors {
        for dir in [x, y] {
            let kind = if dir == x {
                SnapKind::Horizontal
            } else {
                SnapKind::Vertical
            };
            let g = Guide {
                kind,
                anchor: *a,
                dir,
            };
            if !out.iter().any(|h| h.key() == g.key()) {
                out.push(g);
            }
        }
    }
    // Extensions come only from lines the user has actually touched an end of. Every
    // line in the drawing carried out to infinity would catch the pointer everywhere.
    for (_, data) in sketch.entities() {
        let Entity::Line { start, end } = data.entity else {
            continue;
        };
        let (Some(a), Some(b)) = (sketch.point_pos(start), sketch.point_pos(end)) else {
            continue;
        };
        let dir = b - a;
        if dir.length() <= JOIN_TOL {
            continue;
        }
        let dir = dir.normalize();
        for at in [a, b] {
            if !anchors.iter().any(|p| p.distance(at) <= JOIN_TOL) {
                continue;
            }
            let g = Guide {
                kind: SnapKind::Extension,
                anchor: at,
                dir,
            };
            if !out.iter().any(|h| h.key() == g.key()) {
                out.push(g);
            }
        }
    }
    if let (Some(from), Some(curve)) = (cont.from, cont.curve)
        && let Some(geom) = CurveGeom::of(sketch, curve)
        && let Some(dir) = tangent_at(&geom, from)
    {
        out.push(Guide {
            kind: SnapKind::Tangent,
            anchor: from,
            dir,
        });
        out.push(Guide {
            kind: SnapKind::Perpendicular,
            anchor: from,
            dir: Vec2::new(-dir.y, dir.x),
        });
    }
    out
}

/// Where two guides cross. Near-parallel ones are rejected by the caller, so this is
/// the ordinary two-line solve with a guard against dividing by nothing.
fn cross_of(g: Guide, h: Guide) -> Option<Vec2> {
    let denom = g.dir.perp_dot(h.dir);
    if denom.abs() <= f64::EPSILON {
        return None;
    }
    Some(g.anchor + g.dir * ((h.anchor - g.anchor).perp_dot(h.dir) / denom))
}

/// Unit tangent of `geom` at the point of it nearest `at`. `None` for a degenerate
/// curve, where there is no direction to continue in.
fn tangent_at(geom: &CurveGeom, at: Vec2) -> Option<Vec2> {
    match *geom {
        CurveGeom::Line { a, b } => {
            let d = b - a;
            (d.length() > JOIN_TOL).then(|| d.normalize())
        }
        CurveGeom::Arc { center, sweep, .. } => {
            let r = at - center;
            (r.length() > JOIN_TOL).then(|| {
                let t = Vec2::new(-r.y, r.x).normalize();
                if sweep >= 0.0 { t } else { -t }
            })
        }
    }
}

// --- Feedback -----------------------------------------------------------------------

/// Half-width of a snap marker, in pixels. Bigger than the plain cursor dot, because it
/// is the thing the user is reading when they are deciding whether to click.
const MARKER_PX: f64 = 6.0;
/// Length of one dash of a guide line, and of the gap after it, in pixels.
const DASH_PX: f64 = 6.0;
/// How far past the snapped point a guide is drawn, in pixels, so it reads as a line
/// that carries on rather than one that stops where the pointer is.
const GUIDE_OVERSHOOT_PX: f64 = 24.0;

/// The marker glyph for a snap, as segments in sketch-plane coordinates.
///
/// One shape per kind, and different enough to tell apart at a glance: the user has to
/// be able to see *what* caught the pointer without moving their eyes off the drawing.
/// These are the conventions every CAD package already taught them — a square for an
/// end, a triangle for a middle, a circle for a centre, a cross for a crossing.
pub fn marker(kind: SnapKind, at: Vec2, px: f64) -> Vec<[Vec2; 2]> {
    let r = px * MARKER_PX;
    let poly = |n: usize, phase: f64| -> Vec<[Vec2; 2]> {
        (0..n)
            .map(|i| {
                let a = phase + std::f64::consts::TAU * i as f64 / n as f64;
                let b = phase + std::f64::consts::TAU * (i + 1) as f64 / n as f64;
                [at + Vec2::from_angle(a) * r, at + Vec2::from_angle(b) * r]
            })
            .collect()
    };
    let cross = |phase: f64| -> Vec<[Vec2; 2]> {
        (0..2)
            .map(|i| {
                let a = phase + std::f64::consts::PI * i as f64 / 2.0;
                let d = Vec2::from_angle(a) * r;
                [at - d, at + d]
            })
            .collect()
    };
    let bar = |dir: Vec2| vec![[at - dir * r, at + dir * r]];
    match kind {
        // A loose point and an endpoint are both squares; an endpoint is the commoner
        // of the two by far and the one the eye should find fastest, so the loose point
        // is the turned one rather than the other way round.
        SnapKind::Endpoint => poly(4, 0.0),
        SnapKind::Point => poly(4, std::f64::consts::FRAC_PI_4),
        SnapKind::Center => poly(8, 0.0),
        SnapKind::Origin => {
            let mut v = poly(8, 0.0);
            v.extend(cross(0.0));
            v
        }
        SnapKind::Midpoint => poly(3, std::f64::consts::FRAC_PI_2),
        SnapKind::Intersection | SnapKind::GuideCross => cross(std::f64::consts::FRAC_PI_4),
        SnapKind::OnCurve => poly(4, std::f64::consts::FRAC_PI_4),
        SnapKind::Horizontal => bar(Vec2::new(1.0, 0.0)),
        SnapKind::Vertical => bar(Vec2::new(0.0, 1.0)),
        SnapKind::Extension => {
            let mut v = bar(Vec2::new(1.0, 0.0));
            v.extend(bar(Vec2::new(0.0, 1.0)));
            v
        }
        // A tangent is a circle touching a line; a perpendicular is the draughtsman's
        // ⊥, both drawn small.
        SnapKind::Tangent => {
            let mut v = poly(8, 0.0);
            v.push([at + Vec2::new(-r, r), at + Vec2::new(r, r)]);
            v
        }
        SnapKind::Perpendicular => vec![
            [at + Vec2::new(-r, -r), at + Vec2::new(r, -r)],
            [at + Vec2::new(0.0, -r), at + Vec2::new(0.0, r)],
        ],
    }
}

/// A guide drawn dashed from its anchor, through the snapped point and a little past
/// it. Dashed rather than solid because it is not geometry, and it has to be legible as
/// something that will not be there after the click.
pub fn guide_dashes(guide: Guide, at: Vec2, px: f64) -> Vec<[Vec2; 2]> {
    let along = (at - guide.anchor).dot(guide.dir);
    let end = guide.anchor + guide.dir * (along + px * GUIDE_OVERSHOOT_PX * along.signum());
    let span = end - guide.anchor;
    let len = span.length();
    let dash = px * DASH_PX;
    if len <= f64::EPSILON || dash <= 0.0 {
        return Vec::new();
    }
    let dir = span / len;
    let mut out = Vec::new();
    let mut t = 0.0;
    while t < len {
        let to = (t + dash).min(len);
        out.push([guide.anchor + dir * t, guide.anchor + dir * to]);
        t += dash * 2.0;
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn shift_frees_a_drag_and_the_toggle_switches_it_off_entirely() {
        let on = Snapping::default();
        assert!(on.on(), "snapping is on before anyone touches it");
        assert!(!Snapping { free: true, ..on }.on(), "shift lets go");
        // Shift releases a grid that is on; it does not turn one on.
        let off = Snapping {
            to_grid: false,
            free: false,
        };
        assert!(!off.on());
        assert!(!Snapping { free: true, ..off }.on());
    }

    #[test]
    fn a_resolved_snap_rounds_values_points_and_angles() {
        let snap = Snapping::default().at(5.0);
        assert_eq!(snap.value(12.0), 10.0);
        assert_eq!(snap.point(Vec2::new(12.0, -8.0)), Vec2::new(10.0, -10.0));
        assert_eq!(snap.angle_deg(7.4), ANGLE_STEP_DEG);

        let free = Snapping {
            to_grid: true,
            free: true,
        }
        .at(5.0);
        assert_eq!(free.value(12.0), 12.0);
        assert_eq!(free.angle_deg(7.4), 7.4);
        assert!(!free.is_on());
    }

    /// A drag stopped at either end of its range must not bank the travel past it: the
    /// raw total stops with the handle, so pulling back moves the value at once.
    #[test]
    fn a_clamped_drag_does_not_bank_travel_past_either_end() {
        let snap = Snapping::default().at(1.0);
        let mut drag = Drag::default();
        assert_eq!(drag.advance_within(5.0, 4.0, snap, 0.5, 6.0), 6.0);
        assert_eq!(
            drag.advance_within(6.0, -1.0, snap, 0.5, 6.0),
            5.0,
            "one step back off the ceiling is one step, not the banked overshoot first"
        );
        drag.release();
        assert_eq!(drag.advance_within(5.0, -7.0, snap, 0.5, 6.0), 0.5);
        assert_eq!(
            drag.advance_within(0.5, 1.5, snap, 0.5, 6.0),
            2.0,
            "and the same coming back up off the floor"
        );
    }

    /// A nonsensical increment must not silently quantise everything to zero or NaN; it
    /// means "no grid here", which is what a drag with no grid already does.
    #[test]
    fn an_unusable_step_is_no_grid_at_all() {
        for step in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            let snap = Snapping::default().at(step);
            assert!(!snap.is_on(), "{step}");
            assert_eq!(snap.value(3.7), 3.7);
        }
    }

    #[test]
    fn the_hint_says_which_of_the_two_happened() {
        let snapped = Hint::value(Vec3::ZERO, 10.0, " mm", Snapping::default().at(5.0));
        assert!(
            snapped.on_grid && snapped.text.contains("grid 5"),
            "{snapped:?}"
        );
        let freed = Hint::value(
            Vec3::ZERO,
            10.3,
            " mm",
            Snapping {
                to_grid: true,
                free: true,
            }
            .at(5.0),
        );
        assert!(!freed.on_grid && freed.text.contains("free"), "{freed:?}");
    }

    // --- Inference ------------------------------------------------------------------

    /// A sketch with a 20 mm line along the bottom, a 10 mm vertical line meeting it at
    /// the origin and a circle of radius 5 about (30, 0).
    fn drawing() -> Sketch {
        let mut sketch = Sketch::new();
        let a = sketch.add_point(Vec2::new(0.0, 0.0));
        let b = sketch.add_point(Vec2::new(20.0, 0.0));
        let c = sketch.add_point(Vec2::new(0.0, 10.0));
        let centre = sketch.add_point(Vec2::new(30.0, 0.0));
        sketch.add_line(a, b).expect("bottom");
        sketch.add_line(a, c).expect("side");
        sketch.add_circle(centre, 5.0).expect("hole");
        sketch
    }

    fn kinds(candidates: &[Candidate]) -> Vec<SnapKind> {
        let mut k: Vec<SnapKind> = candidates.iter().map(|c| c.kind).collect();
        k.dedup();
        k
    }

    fn at(kind: SnapKind, at: Vec2) -> Candidate {
        Candidate::new(kind, at)
    }

    /// The whole ranking in one go: with the pointer in range of all of them, the
    /// endpoint wins, then the midpoint, then the curve, then the guide - and with none
    /// of them in range nothing is chosen, which is the caller's cue to use the grid.
    #[test]
    fn a_point_beats_a_curve_beats_a_guide() {
        let p = Vec2::new(10.0, 10.0);
        let guide = Guide {
            kind: SnapKind::Horizontal,
            anchor: Vec2::new(0.0, 10.2),
            dir: Vec2::new(1.0, 0.0),
        };
        let mut aligned = at(SnapKind::Horizontal, Vec2::new(10.0, 10.2));
        aligned.guides[0] = Some(guide);
        let all = vec![
            aligned,
            at(SnapKind::OnCurve, Vec2::new(10.0, 10.3)),
            at(SnapKind::Midpoint, Vec2::new(10.0, 10.4)),
            at(SnapKind::Endpoint, Vec2::new(10.0, 10.5)),
        ];
        let pick = |list: &[Candidate]| choose(list, None, p, 1.0, None).map(|i| list[i].kind);
        assert_eq!(pick(&all), Some(SnapKind::Endpoint), "a point first");
        assert_eq!(
            pick(&all[..3]),
            Some(SnapKind::Midpoint),
            "then what it implies"
        );
        assert_eq!(pick(&all[..2]), Some(SnapKind::OnCurve), "then the curve");
        assert_eq!(
            pick(&all[..1]),
            Some(SnapKind::Horizontal),
            "then the guide"
        );
        assert_eq!(
            choose(&all, None, Vec2::new(10.0, 40.0), 1.0, None),
            None,
            "nothing in range means the grid, which is not a candidate"
        );
    }

    /// Within one tier the nearer one wins, and nothing outside the pick radius is
    /// acquired however strong its kind.
    #[test]
    fn within_a_tier_the_nearer_one_wins_and_the_radius_is_a_radius() {
        let p = Vec2::ZERO;
        let near = at(SnapKind::Endpoint, Vec2::new(0.3, 0.0));
        let far = at(SnapKind::Endpoint, Vec2::new(0.6, 0.0));
        let list = vec![far, near];
        assert_eq!(choose(&list, None, p, 1.0, None), Some(1));
        let list = vec![at(SnapKind::Endpoint, Vec2::new(2.0, 0.0))];
        assert_eq!(choose(&list, None, p, 1.0, None), None, "out of reach");
    }

    /// The hold: an acquired snap survives the pointer wandering out past the pick
    /// radius, and lets go once it is properly gone. Without it the preview argues with
    /// itself every time the hand shakes.
    #[test]
    fn an_acquired_snap_is_held_past_the_pick_radius_and_then_let_go() {
        let end = at(SnapKind::Endpoint, Vec2::ZERO);
        let list = vec![end];
        let held = Some(end.key());
        let tol = 1.0;
        assert_eq!(choose(&list, None, Vec2::new(0.9, 0.0), tol, None), Some(0));
        assert_eq!(
            choose(&list, held, Vec2::new(1.5, 0.0), tol, None),
            Some(0),
            "held out past the radius it was acquired in"
        );
        assert_eq!(
            choose(&list, held, Vec2::new(1.9, 0.0), tol, None),
            None,
            "and let go once the pointer has really left it"
        );
        assert_eq!(
            choose(&list, None, Vec2::new(1.5, 0.0), tol, None),
            None,
            "a snap never acquired is not held"
        );
    }

    /// Two candidates of one tier a hair apart must not trade the snap back and forth:
    /// the challenger has to be clearly nearer, not merely nearer.
    #[test]
    fn a_rival_in_the_same_tier_has_to_beat_the_held_snap_by_a_margin() {
        let held_at = at(SnapKind::Endpoint, Vec2::ZERO);
        let rival = at(SnapKind::Endpoint, Vec2::new(1.0, 0.0));
        let list = vec![held_at, rival];
        let held = Some(held_at.key());
        // Just past halfway, so the rival is nearer - but not by the margin.
        let pointer = Vec2::new(0.55, 0.0);
        assert_eq!(choose(&list, held, pointer, 1.0, None), Some(0), "held");
        assert_eq!(
            choose(&list, None, pointer, 1.0, None),
            Some(1),
            "with nothing held the nearer one simply wins"
        );
        // Well over to the rival's side, and it takes over.
        assert_eq!(choose(&list, held, Vec2::new(0.9, 0.0), 1.0, None), Some(1));
    }

    /// A stronger tier does not have to beat the margin - a point appearing under the
    /// pointer while a guide is held is exactly the moment to change the answer.
    #[test]
    fn a_stronger_tier_takes_a_held_snap_over_on_acquisition_alone() {
        let guide = at(SnapKind::Horizontal, Vec2::ZERO);
        let point = at(SnapKind::Endpoint, Vec2::new(0.4, 0.0));
        let list = vec![guide, point];
        let held = Some(guide.key());
        assert_eq!(choose(&list, held, Vec2::new(0.2, 0.0), 1.0, None), Some(1));
    }

    /// A guide is worth taking only when it is nearer than the grid line it displaces.
    /// Without this the axes of the drawing catch every click near them and a sketch
    /// made by eye stops coming out in round numbers.
    #[test]
    fn a_guide_further_off_than_the_grid_is_not_help() {
        let pointer = Vec2::new(0.2, 0.9);
        let guide = at(SnapKind::Horizontal, Vec2::new(0.2, 0.0));
        let list = vec![guide];
        assert_eq!(choose(&list, None, pointer, 5.0, None), Some(0), "no grid");
        assert_eq!(
            choose(&list, None, pointer, 5.0, Some(Vec2::new(0.0, 1.0))),
            None,
            "the grid line is nearer, so the grid is the better answer"
        );
        // Geometry is not a suggestion: the grid does not overrule a real place.
        let mid = vec![at(SnapKind::Midpoint, Vec2::new(0.2, 0.0))];
        assert_eq!(
            choose(&mid, None, pointer, 5.0, Some(Vec2::new(0.0, 1.0))),
            Some(0)
        );
    }

    /// The places the drawing names, found where they are.
    #[test]
    fn the_drawing_names_endpoints_midpoints_centres_and_curves() {
        let sketch = drawing();
        let found = |p: Vec2| {
            kinds(&gather(
                &sketch,
                &[],
                Continuation::default(),
                p,
                0.5,
                false,
            ))
        };
        assert!(found(Vec2::new(20.0, 0.1)).contains(&SnapKind::Endpoint));
        assert!(found(Vec2::new(10.0, 0.1)).contains(&SnapKind::Midpoint));
        assert!(found(Vec2::new(30.0, 0.1)).contains(&SnapKind::Center));
        assert!(found(Vec2::new(13.0, 0.05)).contains(&SnapKind::OnCurve));
        assert!(found(Vec2::new(0.05, 0.05)).contains(&SnapKind::Endpoint));
    }

    /// A crossing is a place whether or not anything is drawn there: two curves that
    /// pass through each other are offered at the point they pass.
    #[test]
    fn two_curves_crossing_are_offered_where_they_cross() {
        let mut sketch = Sketch::new();
        let a = sketch.add_point(Vec2::new(-10.0, 5.0));
        let b = sketch.add_point(Vec2::new(10.0, 5.0));
        let c = sketch.add_point(Vec2::new(0.0, -10.0));
        let d = sketch.add_point(Vec2::new(0.0, 10.0));
        sketch.add_line(a, b).expect("across");
        sketch.add_line(c, d).expect("up");
        let found = gather(
            &sketch,
            &[],
            Continuation::default(),
            Vec2::new(0.1, 5.1),
            0.5,
            false,
        );
        let crossing = found
            .iter()
            .find(|c| c.kind == SnapKind::Intersection)
            .expect("the crossing");
        assert!(crossing.at.distance(Vec2::new(0.0, 5.0)) < 1e-9);
    }

    /// Alignment with a point the user has touched, and the continuation of the line it
    /// belongs to. Both carry the geometry they came from, so the dashed line can be
    /// drawn back to it.
    #[test]
    fn touched_points_and_drawn_lines_grow_guides() {
        let sketch = drawing();
        let recent = [Vec2::new(20.0, 0.0)];
        let found = |p: Vec2| gather(&sketch, &recent, Continuation::default(), p, 1.0, false);
        let vertical = found(Vec2::new(20.05, 8.0));
        let caught = vertical
            .iter()
            .find(|c| c.kind == SnapKind::Vertical)
            .expect("above the point it was touched at");
        assert!((caught.at.x - 20.0).abs() < 1e-9 && (caught.at.y - 8.0).abs() < 1e-9);
        assert_eq!(
            caught.guides[0].map(|g| g.anchor),
            Some(Vec2::new(20.0, 0.0)),
            "the guide names the point it comes from"
        );
        // Out along the bottom line, past its end: the extension.
        let out = found(Vec2::new(28.0, 0.05));
        assert!(
            kinds(&out).contains(&SnapKind::Extension),
            "{:?}",
            kinds(&out)
        );
    }

    /// Where two guides agree, the place they agree on is offered as a point - the one
    /// inference that names a position in both axes.
    #[test]
    fn two_guides_that_agree_are_offered_as_a_point() {
        let sketch = Sketch::new();
        let recent = [Vec2::new(10.0, 0.0), Vec2::new(0.0, 7.0)];
        let found = gather(
            &sketch,
            &recent,
            Continuation::default(),
            Vec2::new(10.05, 7.05),
            1.0,
            false,
        );
        let cross = found
            .iter()
            .find(|c| c.kind == SnapKind::GuideCross)
            .expect("level with one and above the other");
        assert!(cross.at.distance(Vec2::new(10.0, 7.0)) < 1e-9);
        assert!(cross.guides[1].is_some(), "both guides are drawn");
    }

    /// Continuing from a curve offers its tangent and its normal, which is how a chain
    /// carries on smoothly without reaching for a constraint afterwards.
    #[test]
    fn continuing_from_a_curve_offers_its_tangent_and_its_normal() {
        let mut sketch = Sketch::new();
        let a = sketch.add_point(Vec2::new(0.0, 0.0));
        let b = sketch.add_point(Vec2::new(10.0, 0.0));
        let line = sketch.add_line(a, b).expect("line");
        let cont = Continuation {
            from: Some(Vec2::new(10.0, 0.0)),
            curve: Some(line),
            moving: None,
        };
        let along = gather(&sketch, &[], cont, Vec2::new(16.0, 0.05), 1.0, false);
        assert!(
            kinds(&along).contains(&SnapKind::Tangent),
            "{:?}",
            kinds(&along)
        );
        let across = gather(&sketch, &[], cont, Vec2::new(10.05, 6.0), 1.0, false);
        assert!(
            kinds(&across).contains(&SnapKind::Perpendicular),
            "{:?}",
            kinds(&across)
        );
    }

    /// Shift, and the palette's switch, leave the joins and nothing else: the point to
    /// share and the curve to be held onto survive, every computed place goes.
    #[test]
    fn shift_lets_go_of_everything_but_the_joins() {
        let sketch = drawing();
        let recent = [Vec2::new(20.0, 0.0)];
        let cont = Continuation::default();
        let free = |p: Vec2| kinds(&gather(&sketch, &recent, cont, p, 1.0, true));
        assert!(
            free(Vec2::new(20.0, 0.2)).contains(&SnapKind::Endpoint),
            "an existing point is never given up: it is how geometry is joined"
        );
        assert_eq!(
            free(Vec2::new(13.0, 0.1)),
            vec![SnapKind::OnCurve],
            "nor is a curve, which the new point is held onto by a constraint"
        );
        let middle = free(Vec2::new(10.0, 0.1));
        assert!(
            middle.contains(&SnapKind::OnCurve) && !middle.contains(&SnapKind::Midpoint),
            "but the midpoint of that same line is computed, and goes"
        );
        assert!(free(Vec2::new(0.1, 0.1)).iter().all(|k| k.joins()));
    }

    /// The memory the guides grow from: newest first, no duplicates, and bounded, so
    /// the screen never fills with dashes the user cannot account for.
    #[test]
    fn touched_points_are_remembered_newest_first_and_bounded() {
        let mut inference = Inference::default();
        for i in 0..6 {
            inference.touch(Vec2::new(f64::from(i), 0.0));
        }
        assert_eq!(
            inference.recent,
            vec![
                Vec2::new(5.0, 0.0),
                Vec2::new(4.0, 0.0),
                Vec2::new(3.0, 0.0)
            ]
        );
        inference.touch(Vec2::new(4.0, 0.0));
        assert_eq!(inference.recent[0], Vec2::new(4.0, 0.0));
        assert_eq!(
            inference.recent.len(),
            3,
            "touching one again does not add it"
        );
    }

    /// End to end through the state: the pointer acquires the endpoint, keeps it while
    /// it wanders, and hovering it is what makes that corner an anchor for the guides
    /// the next point will line up with.
    #[test]
    fn resolving_holds_the_snap_and_remembers_what_was_hovered() {
        let sketch = drawing();
        let mut inference = Inference::default();
        let cont = Continuation::default();
        let found = inference
            .resolve(&sketch, cont, Vec2::new(19.8, 0.1), 1.0, false, None)
            .expect("the endpoint");
        assert_eq!(found.kind, SnapKind::Endpoint);
        assert!(
            found.point.is_some(),
            "and it hands back the entity to share"
        );
        assert_eq!(inference.recent.first(), Some(&Vec2::new(20.0, 0.0)));
        let held = inference
            .resolve(&sketch, cont, Vec2::new(18.6, 0.1), 1.0, false, None)
            .expect("still held");
        assert_eq!(held.kind, SnapKind::Endpoint);
        assert!(inference.current().is_some(), "and the overlay can draw it");
        inference.release();
        assert!(inference.current().is_none());
    }

    /// The feedback is geometry the overlay can draw: a glyph on the point, different
    /// per kind, and a dashed line back to whatever caught it.
    #[test]
    fn every_kind_has_a_marker_and_a_guide_is_drawn_dashed() {
        for kind in [
            SnapKind::Point,
            SnapKind::Endpoint,
            SnapKind::Center,
            SnapKind::Origin,
            SnapKind::Midpoint,
            SnapKind::Intersection,
            SnapKind::GuideCross,
            SnapKind::OnCurve,
            SnapKind::Horizontal,
            SnapKind::Vertical,
            SnapKind::Extension,
            SnapKind::Tangent,
            SnapKind::Perpendicular,
        ] {
            let glyph = marker(kind, Vec2::new(3.0, 4.0), 0.1);
            assert!(!glyph.is_empty(), "{kind:?} has nothing to draw");
            for seg in &glyph {
                assert!(
                    seg[0].distance(Vec2::new(3.0, 4.0)) < 2.0,
                    "{kind:?} strays"
                );
            }
        }
        let guide = Guide {
            kind: SnapKind::Horizontal,
            anchor: Vec2::ZERO,
            dir: Vec2::new(1.0, 0.0),
        };
        let dashes = guide_dashes(guide, Vec2::new(10.0, 0.0), 0.1);
        assert!(dashes.len() > 3, "a dashed line, not a solid one");
        assert!(
            dashes
                .iter()
                .all(|d| d[0].y.abs() < 1e-9 && d[1].y.abs() < 1e-9),
            "drawn along the guide"
        );
        assert!(
            dashes.last().map(|d| d[1].x) > Some(10.0),
            "and carried past the point, because it is a line that goes on"
        );
    }

    /// A point being dragged must not snap to itself, nor to the curves that follow it
    /// about: both are under the pointer by construction, and a drag that sticks to
    /// them cannot be moved at all.
    #[test]
    fn a_dragged_point_does_not_snap_to_itself_or_to_what_hangs_off_it() {
        let mut sketch = Sketch::new();
        let a = sketch.add_point(Vec2::new(0.0, 0.0));
        let b = sketch.add_point(Vec2::new(20.0, 0.0));
        sketch.add_line(a, b).expect("line");
        let cont = Continuation {
            moving: Some(b),
            ..Default::default()
        };
        let found = gather(&sketch, &[], cont, Vec2::new(20.05, 0.05), 1.0, false);
        assert!(
            found
                .iter()
                .all(|c| c.point != Some(b) && c.curve.is_none()),
            "{:?}",
            kinds(&found)
        );
        // The other end of that line is still a place to land, which is how a drag
        // closes a loop.
        let onto = gather(&sketch, &[], cont, Vec2::new(0.05, 0.05), 1.0, false);
        assert!(onto.iter().any(|c| c.point == Some(a)));
    }
}
