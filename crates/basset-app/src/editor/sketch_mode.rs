//! Sketch mode: drawing and constraining 2D geometry on one plane.
//!
//! The editor works on a copy of the sketch and writes it back into the feature after
//! every change, so the rest of the model (extrudes, fillets…) updates live while the
//! user draws. Points are snapped to existing points within a screen-space tolerance
//! and joined by sharing the point entity, which is how a closed loop of lines becomes
//! a profile without the user thinking about coincidence constraints.
//!
//! Points that snap to nothing land on the grid instead. The increment follows the zoom
//! like the drawn grid does, so the user is always snapping to something they can see,
//! and can be pinned to a fixed value when a drawing calls for one.
//!
//! While a shape is being drawn its sizes can also be typed. A typed value pins that
//! size while the pointer keeps choosing direction and side, and once the shape exists
//! the value becomes a driving dimension, the way Fusion's entry boxes work: what the
//! user stated stays true under later edits, what they merely pointed at stays free.

use basset_core::{FeatureId, FeatureKind, PlaneRef, ProfileRef};
use basset_math::{Frame, Ray, Vec2, Vec3};
pub use basset_sketch::offset::Corner;
use basset_sketch::{
    Constraint, ConstraintId, Entity, EntityId, Profile, Sketch, SketchError, SolveError,
    SolveReport, Tessellation, edit, fillet, offset, pattern, shapes,
};
use basset_viewport::{Camera, LineBatch, PointBatch, TriBatch, grid};

use super::Editor;
use super::snap::{self, Snap, Snapping};

/// Half-length of a crosshair arm, in pixels.
const CURSOR_ARM_PX: f64 = 14.0;
/// Pixels left clear around the centre, so the marker itself stays readable.
const CURSOR_GAP_PX: f64 = 4.0;
/// Lines whose directions differ by less than this are dimensioned by distance rather
/// than by angle. It is far below anything drawn by hand, so only lines that are
/// parallel by construction (grid, constraint) qualify, as in Fusion.
const PARALLEL_TOL: f64 = 1e-6;
/// Arrowhead length of a dimension line, in pixels.
const ARROW_PX: f64 = 9.0;
/// How far an unplaced dimension sits from what it measures, in pixels.
const LABEL_GAP_PX: f64 = 28.0;
/// Geometry the constraints do not pin down, drawn in Fusion's convention of blue for
/// under-constrained and white for solved.
const LOOSE_COLOR: [f32; 4] = [0.55, 0.72, 1.0, 1.0];
/// Constraints and dimensions the solver could not satisfy, drawn red wherever they are
/// drawn at all: the palette names them, but the answer to "which one is fighting?"
/// belongs on the drawing.
pub const CONFLICT_COLOR: [f32; 4] = [0.95, 0.35, 0.3, 1.0];
/// A closed region the pointer is inside, filled so the user can see the area itself
/// rather than infer it from the curves around it. Matches the model-mode profile
/// highlight, because it is the same thing being pointed at.
const REGION_HOVER_FILL: [f32; 4] = [1.0, 0.85, 0.3, 0.16];
/// A region the user has picked, in the selection blue.
const REGION_SELECT_FILL: [f32; 4] = [0.25, 0.6, 1.0, 0.22];
/// The smallest fillet a drag may land on, in mm. A radius is a size, not an offset:
/// dragging the handle back through the corner has nowhere further to go, and a zero
/// radius is a corner that was never rounded rather than a fillet turned inside out.
const MIN_FILLET_RADIUS: f64 = 0.01;
/// Half-extent of a constraint badge.
const GLYPH_PX: f64 = 5.0;
/// The snap marker, in the same amber the crosshair turns when a click will join
/// geometry: one colour means "the drawing caught this" wherever it appears.
pub(crate) const SNAP_MARKER_COLOR: [f32; 4] = [1.0, 0.85, 0.3, 1.0];
/// The guide lines, dimmer than the marker because they are scaffolding rather than a
/// place — the eye should land on the point, not on the line that found it.
pub(crate) const SNAP_GUIDE_COLOR: [f32; 4] = [1.0, 0.85, 0.3, 0.45];
/// Clearance between the geometry and the first badge on it. Measured from the anchor,
/// so it is wide enough that the first slot's box clears the curve it is written on.
const GLYPH_GAP_PX: f64 = 15.0;
/// Half-extent of the box a badge claims on screen: the symbol plus enough air that two
/// of them read as two marks rather than as one smudge.
const GLYPH_BOX_PX: f64 = 9.0;
/// How much further out each successive ring of candidate slots sits. Wider than a
/// badge's box, so a badge pushed to the next ring is clear of the one that displaced it.
const GLYPH_STEP_PX: f64 = 20.0;
/// How many rings a badge may be pushed out through before the layout gives up and puts
/// it in its first slot anyway. A badge that vanished would be worse than one that
/// overlaps: it could never be hovered, named or deleted.
const GLYPH_RINGS: usize = 5;
/// Directions tried in each ring, as eighths of a turn about the entity's own normal.
const GLYPH_DIRS: usize = 8;
/// What turning away from the entity's natural side costs, in pixels per radian. Half a
/// turn costs about a ring, so a badge crosses to the other side of its line before it
/// walks a long way out along the near side.
const GLYPH_TURN_PX: f64 = 6.0;
/// Beyond this the badge has visibly left its anchor and gets a leader line back to it.
const GLYPH_LEADER_PX: f64 = 22.0;
/// How near the pointer must come to a badge's centre to be on it, in pixels.
const GLYPH_HIT_PX: f64 = 11.0;
/// Cell size of the grid the drawing's segments are bucketed into for the collision
/// tests, in pixels. A little larger than a badge's box, so a badge reads a handful of
/// cells.
const GLYPH_CELL_PX: f64 = 24.0;
/// Cells one segment may be written into. A line is thousands of cells long when the
/// view is zoomed right into one corner of it, and the layout's cost should follow the
/// drawing rather than the zoom; the price is a coarser bucket for a curve whose whole
/// length nobody can see anyway.
const GLYPH_MAX_CELLS: usize = 256;
/// Zoom steps, as a divisor of one e-fold, at which the packing is reconsidered. The
/// layout is decided in pixels and the badges are drawn at the true scale, so between
/// two steps the whole overlay simply scales with the view: no badge moves relative to
/// its geometry while the user zooms, which is the property that matters more than an
/// optimal packing.
const GLYPH_SCALE_STEPS: f64 = 4.0;

/// What a click or a box drag may land on in a sketch.
///
/// Model mode has the same filter for the same reason: a corner, the curves meeting
/// there and the region beyond them are all within a few pixels of each other, so which
/// of them a drag takes is a question only the user can answer. A box across a drawing
/// otherwise takes the points as well as the curves, and moving what you meant to
/// delete is a poor way to find that out.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum SketchPick {
    #[default]
    All,
    /// Lines, arcs, circles and text — what a drawing is made of.
    Curves,
    /// Endpoints, centres and loose points.
    Points,
    /// Closed areas, which are what a feature is built from.
    Regions,
}

impl SketchPick {
    pub const ALL: [SketchPick; 4] = [
        SketchPick::All,
        SketchPick::Curves,
        SketchPick::Points,
        SketchPick::Regions,
    ];

    pub fn name(self) -> &'static str {
        match self {
            SketchPick::All => "All",
            SketchPick::Curves => "Curves",
            SketchPick::Points => "Points",
            SketchPick::Regions => "Regions",
        }
    }

    pub fn hint(self) -> &'static str {
        match self {
            SketchPick::All => "Clicks and box drags take curves, points and regions",
            SketchPick::Curves => "Only lines, arcs, circles and text",
            SketchPick::Points => "Only endpoints, centres and loose points",
            SketchPick::Regions => "Only the closed areas a feature is built from",
        }
    }

    fn takes_curves(self) -> bool {
        matches!(self, SketchPick::All | SketchPick::Curves)
    }

    fn takes_points(self) -> bool {
        matches!(self, SketchPick::All | SketchPick::Points)
    }

    fn takes_regions(self) -> bool {
        matches!(self, SketchPick::All | SketchPick::Regions)
    }
}

/// A geometric constraint the toolbar offers. Dimensions are a tool of their own; these
/// are the ones that carry no number.
///
/// Each of these is a *tool*, not a command on the current selection: the user picks the
/// constraint first and the geometry after, which is how Fusion works and is what makes
/// a constraint discoverable — the prompt tells you what to pick, instead of the button
/// staying grey until you have guessed. Picking geometry first still works: clicking the
/// button then applies it at once and leaves the tool armed for the next pair.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ConstraintKind {
    Coincident,
    Horizontal,
    Vertical,
    Parallel,
    Perpendicular,
    Tangent,
    Equal,
    Concentric,
    Midpoint,
    Symmetric,
    Fix,
}

impl ConstraintKind {
    pub const ALL: [ConstraintKind; 11] = [
        ConstraintKind::Coincident,
        ConstraintKind::Horizontal,
        ConstraintKind::Vertical,
        ConstraintKind::Parallel,
        ConstraintKind::Perpendicular,
        ConstraintKind::Tangent,
        ConstraintKind::Equal,
        ConstraintKind::Concentric,
        ConstraintKind::Midpoint,
        ConstraintKind::Symmetric,
        ConstraintKind::Fix,
    ];

    pub fn name(self) -> &'static str {
        match self {
            ConstraintKind::Coincident => "Coincident",
            ConstraintKind::Horizontal => "Horizontal",
            ConstraintKind::Vertical => "Vertical",
            ConstraintKind::Parallel => "Parallel",
            ConstraintKind::Perpendicular => "Perpendicular",
            ConstraintKind::Tangent => "Tangent",
            ConstraintKind::Equal => "Equal",
            ConstraintKind::Concentric => "Concentric",
            ConstraintKind::Midpoint => "Midpoint",
            ConstraintKind::Symmetric => "Symmetric",
            ConstraintKind::Fix => "Fix",
        }
    }

    /// What the tool is waiting for, shown while it is armed. Order is never required —
    /// the tool works out which pick is the point and which the line — so the wording
    /// says what to pick rather than what to pick first.
    pub fn hint(self) -> &'static str {
        match self {
            ConstraintKind::Coincident => {
                "Pick a point, then the point, line, circle or arc to put it on"
            }
            ConstraintKind::Horizontal => "Pick a line, or two points to level with each other",
            ConstraintKind::Vertical => "Pick a line, or two points to stack above each other",
            ConstraintKind::Parallel => "Pick two or more lines",
            ConstraintKind::Perpendicular => "Pick two lines",
            ConstraintKind::Tangent => {
                "Pick a circle or arc, and the line, circle or arc it touches"
            }
            ConstraintKind::Equal => "Pick two or more lines, or two or more circles and arcs",
            ConstraintKind::Concentric => "Pick two or more circles and arcs",
            ConstraintKind::Midpoint => "Pick a point and the line to centre it on",
            ConstraintKind::Symmetric => "Pick two points and the line to mirror them about",
            ConstraintKind::Fix => "Pick a point or a line to pin in place",
        }
    }

    /// Whether the constraint is transitive, so picking on past a pair keeps tying each
    /// new entity to the one before. Equal across five holes is then five clicks rather
    /// than five commands, which is how Fusion's equal behaves.
    pub fn chains(self) -> bool {
        matches!(
            self,
            ConstraintKind::Parallel | ConstraintKind::Equal | ConstraintKind::Concentric
        )
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SketchTool {
    Select,
    Line,
    Rectangle,
    CenterRectangle,
    Circle,
    Circle2Point,
    Circle3Point,
    Arc3Point,
    ArcCenter,
    Polygon,
    /// Centre to centre, then the width.
    Slot,
    /// End to end, then the width: the overall length is what was drawn.
    SlotOverall,
    /// Centre of the slot, one arc centre, then the width.
    SlotCenterPoint,
    Text,
    Dimension,
    /// Removes the piece of a curve between the crossings either side of the click.
    Trim,
    /// Cuts a curve at its crossings without removing anything.
    Break,
    /// Rounds the corner between two curves with a tangent arc of a chosen radius,
    /// trimming both of them back to it.
    Fillet,
    /// Applies one geometric constraint to the entities picked next.
    Constrain(ConstraintKind),
}

/// A toolbar button. Variants of one shape (the three ways to draw a circle…) share a
/// button, as they do in Fusion: the button shows the variant last used, and holding it
/// or right-clicking lists the others.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ToolGroup {
    Select,
    Line,
    Rectangle,
    Circle,
    Arc,
    Polygon,
    Slot,
    Text,
    Dimension,
    Trim,
    /// The constraint tools. Deliberately outside [`ToolGroup::ALL`]: they have their own
    /// row in the toolbar, one button per constraint, rather than one folded button.
    Constrain,
}

impl ToolGroup {
    pub const ALL: [ToolGroup; 10] = [
        ToolGroup::Select,
        ToolGroup::Line,
        ToolGroup::Rectangle,
        ToolGroup::Circle,
        ToolGroup::Arc,
        ToolGroup::Polygon,
        ToolGroup::Slot,
        ToolGroup::Text,
        ToolGroup::Dimension,
        ToolGroup::Trim,
    ];

    pub fn name(self) -> &'static str {
        match self {
            ToolGroup::Select => "Select",
            ToolGroup::Line => "Line",
            ToolGroup::Rectangle => "Rectangle",
            ToolGroup::Circle => "Circle",
            ToolGroup::Arc => "Arc",
            ToolGroup::Polygon => "Polygon",
            ToolGroup::Slot => "Slot",
            ToolGroup::Text => "Text",
            ToolGroup::Dimension => "Dimension",
            ToolGroup::Trim => "Trim",
            ToolGroup::Constrain => "Constrain",
        }
    }

    /// The tools folded under this button, the default first.
    pub fn variants(self) -> &'static [SketchTool] {
        match self {
            ToolGroup::Select => &[SketchTool::Select],
            ToolGroup::Line => &[SketchTool::Line],
            ToolGroup::Rectangle => &[SketchTool::Rectangle, SketchTool::CenterRectangle],
            ToolGroup::Circle => &[
                SketchTool::Circle,
                SketchTool::Circle2Point,
                SketchTool::Circle3Point,
            ],
            ToolGroup::Arc => &[SketchTool::Arc3Point, SketchTool::ArcCenter],
            ToolGroup::Polygon => &[SketchTool::Polygon],
            ToolGroup::Slot => &[
                SketchTool::Slot,
                SketchTool::SlotOverall,
                SketchTool::SlotCenterPoint,
            ],
            ToolGroup::Text => &[SketchTool::Text],
            ToolGroup::Dimension => &[SketchTool::Dimension],
            ToolGroup::Trim => &[SketchTool::Trim, SketchTool::Break, SketchTool::Fillet],
            ToolGroup::Constrain => &[],
        }
    }
}

impl SketchTool {
    pub fn name(self) -> &'static str {
        match self {
            SketchTool::Select => "Select",
            SketchTool::Line => "Line",
            SketchTool::Rectangle => "Rectangle (2 pt)",
            SketchTool::CenterRectangle => "Rectangle (centre)",
            SketchTool::Circle => "Circle (centre)",
            SketchTool::Circle2Point => "Circle (2 pt)",
            SketchTool::Circle3Point => "Circle (3 pt)",
            SketchTool::Arc3Point => "Arc (3 pt)",
            SketchTool::ArcCenter => "Arc (centre)",
            SketchTool::Polygon => "Polygon",
            SketchTool::Slot => "Slot (centre to centre)",
            SketchTool::SlotOverall => "Slot (overall)",
            SketchTool::SlotCenterPoint => "Slot (centre point)",
            SketchTool::Text => "Text",
            SketchTool::Dimension => "Dimension",
            SketchTool::Trim => "Trim",
            SketchTool::Break => "Break",
            SketchTool::Fillet => "Fillet",
            SketchTool::Constrain(kind) => kind.name(),
        }
    }

    pub fn group(self) -> ToolGroup {
        match self {
            SketchTool::Select => ToolGroup::Select,
            SketchTool::Line => ToolGroup::Line,
            SketchTool::Rectangle | SketchTool::CenterRectangle => ToolGroup::Rectangle,
            SketchTool::Circle | SketchTool::Circle2Point | SketchTool::Circle3Point => {
                ToolGroup::Circle
            }
            SketchTool::Arc3Point | SketchTool::ArcCenter => ToolGroup::Arc,
            SketchTool::Polygon => ToolGroup::Polygon,
            SketchTool::Slot | SketchTool::SlotOverall | SketchTool::SlotCenterPoint => {
                ToolGroup::Slot
            }
            SketchTool::Text => ToolGroup::Text,
            SketchTool::Dimension => ToolGroup::Dimension,
            SketchTool::Trim | SketchTool::Break | SketchTool::Fillet => ToolGroup::Trim,
            SketchTool::Constrain(_) => ToolGroup::Constrain,
        }
    }

    /// Clicks needed before the shape exists.
    fn clicks(self) -> usize {
        match self {
            SketchTool::Select
            | SketchTool::Line
            | SketchTool::Dimension
            | SketchTool::Trim
            | SketchTool::Break
            | SketchTool::Fillet
            | SketchTool::Constrain(_) => 0,
            SketchTool::Text => 1,
            SketchTool::Circle3Point
            | SketchTool::Arc3Point
            | SketchTool::ArcCenter
            | SketchTool::Slot
            | SketchTool::SlotOverall
            | SketchTool::SlotCenterPoint => 3,
            _ => 2,
        }
    }

    /// Sizes that can be typed while the shape is being drawn, in Tab order.
    pub fn dims(self) -> &'static [Dim] {
        match self {
            SketchTool::Line => &[Dim::Length, Dim::Angle],
            SketchTool::Rectangle | SketchTool::CenterRectangle => &[Dim::Width, Dim::Height],
            SketchTool::Circle | SketchTool::Circle2Point => &[Dim::Diameter],
            SketchTool::ArcCenter | SketchTool::Polygon => &[Dim::Radius],
            SketchTool::Slot | SketchTool::SlotOverall | SketchTool::SlotCenterPoint => {
                &[Dim::Length, Dim::Width]
            }
            _ => &[],
        }
    }
}

/// A size of the shape being drawn that the user may type instead of pointing at.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Dim {
    Length,
    Angle,
    /// A move's offset across and up. Unlike a size these are signed and may be zero:
    /// "50 mm that way and nothing up" is an ordinary thing to ask for.
    Dx,
    Dy,
    /// An offset's distance. Signed for the same reason a move's offset is, and for one
    /// more: the sign is which side of the drawing the result went, so typing a minus is
    /// the flip, exactly as dragging the handle out the far side is.
    Distance,
    Width,
    Height,
    Diameter,
    Radius,
}

impl Dim {
    pub fn label(self) -> &'static str {
        match self {
            Dim::Length => "Length",
            Dim::Angle => "Angle",
            Dim::Dx => "dX",
            Dim::Dy => "dY",
            Dim::Distance => "Distance",
            Dim::Width => "Width",
            Dim::Height => "Height",
            Dim::Diameter => "Diameter",
            Dim::Radius => "Radius",
        }
    }

    pub fn unit(self) -> &'static str {
        match self {
            Dim::Angle => "°",
            _ => "mm",
        }
    }
}

/// One entry box for one [`Dim`] of the current tool. Until the user types into it the
/// box mirrors the size the pointer is currently drawing, as Fusion's do; typing locks
/// it, and a locked value is what the shape gets regardless of the pointer.
#[derive(Clone, Debug)]
pub struct Entry {
    pub dim: Dim,
    pub text: String,
    pub locked: bool,
}

impl Entry {
    /// The locked value in sketch units (mm, or radians for angles). Sizes are taken
    /// unsigned because the pointer decides which way the shape grows; `None` while the
    /// box is live, empty, unparsable or zero.
    pub fn value(&self) -> Option<f64> {
        if !self.locked {
            return None;
        }
        let v: f64 = self.text.trim().parse().ok()?;
        if !v.is_finite() {
            return None;
        }
        match self.dim {
            Dim::Angle => Some(v.to_radians()),
            // A move's offset keeps its sign, and zero along one axis is a real answer.
            // An offset's distance keeps its sign because the sign is the side; zero is
            // no offset at all, and the tool says so rather than the box swallowing it.
            Dim::Dx | Dim::Dy | Dim::Distance => Some(v),
            _ => (v != 0.0).then_some(v.abs()),
        }
    }
}

fn typed_value(entries: &[Entry], dim: Dim) -> Option<f64> {
    entries.iter().find(|e| e.dim == dim).and_then(Entry::value)
}

/// Everything a shape needs besides its clicks: palette settings and typed sizes.
#[derive(Clone, Debug)]
struct ShapeParams {
    sides: usize,
    text: String,
    text_height: f64,
    typed: Vec<(Dim, f64)>,
}

impl ShapeParams {
    fn typed(&self, dim: Dim) -> Option<f64> {
        self.typed.iter().find(|(d, _)| *d == dim).map(|(_, v)| *v)
    }
}

/// Where a click landed: the snapped position, and what it landed on.
#[derive(Clone, Copy, Default)]
struct Click {
    pos: Vec2,
    /// An existing point the click landed on. New geometry shares the entity outright,
    /// which is what joins a loop without the user asking for a constraint.
    snapped: Option<EntityId>,
    /// A curve the click landed on, when it landed on no point. Sharing an entity is not
    /// possible here, so the new point gets a `Coincident` onto the curve instead.
    on_curve: Option<EntityId>,
}

/// Geometry being dragged with the select tool: every point that moves, with where it
/// started, and where the pointer went down. Each point is offered the same offset as a
/// goal and the constraints decide how much of it survives.
#[derive(Clone)]
struct Drag {
    points: Vec<(EntityId, Vec2)>,
    press: Vec2,
}

/// A dimension drawn the way a mechanical drawing shows it: extension lines, a
/// dimension line with arrowheads (or a leader), and the value at `label`.
#[derive(Clone, Debug)]
pub struct DimGraphic {
    pub id: ConstraintId,
    /// World position of the value text.
    pub label: Vec3,
    pub text: String,
    pub segments: Vec<[Vec3; 2]>,
}

/// A geometric constraint drawn on the geometry it acts on: a small badge of strokes
/// beside the entity, at a constant size on screen the way the dimension arrows are.
///
/// Dimensions draw their own value and leader; these are the ones that have no number,
/// and without them a sketch gives the user no way to see what is holding it together.
pub struct ConstraintGlyph {
    pub id: ConstraintId,
    /// Which of the constraint's entities this badge sits on. A constraint between two
    /// curves draws one badge on each, so the id alone does not identify a badge, and the
    /// interactive area over it needs something stable that the geometry moving does not
    /// change.
    pub target: usize,
    /// World centre of the badge, for placing an interactive area over it.
    pub center: Vec3,
    pub segments: Vec<[Vec3; 2]>,
    /// A line back to the geometry, drawn when decluttering pushed the badge far enough
    /// off its entity that which entity it belongs to stopped being obvious.
    pub leader: Option<[Vec3; 2]>,
}

/// A badge's placement in the units the layout is decided in: an anchor on the geometry,
/// the direction it was pushed in, and how far in *pixels*.
///
/// Keeping the distance in pixels is what makes the overlay stable. The world position
/// is derived at the current zoom every time the badges are asked for, so a badge holds
/// its size on screen exactly while the packing — which slot each badge took — is
/// reconsidered only when the view has really changed.
#[derive(Clone)]
struct GlyphPlacement {
    id: ConstraintId,
    target: usize,
    /// Index into the candidate slots, kept so the next layout can offer a badge the
    /// place it already had.
    slot: usize,
    base: Vec2,
    dir: Vec2,
    dist_px: f64,
    /// Local axes the symbol is drawn on: the entity's own, so a badge that dodges to
    /// the far side of its line does not also turn over.
    u: Vec2,
    v: Vec2,
}

/// The badge layout, kept until something it depends on changes.
///
/// The overlay asks for the badges twice a frame — once to draw them, once to put a hit
/// area over each — and a few hundred constraints make laying them out that often a real
/// cost. `key` hashes everything the layout reads; `builds` counts the rebuilds so a
/// test can assert that panning and redrawing cause none.
#[derive(Default)]
struct GlyphCache {
    key: Option<u64>,
    placements: Vec<GlyphPlacement>,
    builds: u64,
}

/// A keyboard-driven move of the selection. The numbers are typed rather than dragged,
/// which is the whole point: a part goes exactly where the drawing says, and the
/// constraints still decide how much of the offer they will take.
#[derive(Clone, Debug)]
pub struct MoveOp {
    pub dx: f64,
    pub dy: f64,
    pub angle_deg: f64,
    /// Every point that moves, at the position it held when the move began, so the
    /// numbers always mean an offset from the start rather than from the last keystroke.
    start: Vec<(EntityId, Vec2)>,
    /// What the rotation turns about: the centre of what is being moved.
    pivot: Vec2,
    /// The sketch as it was before the move. Every change re-derives from this rather
    /// than nudging what the last change left, so the result depends on the numbers
    /// alone and not on the order they were typed in — and so cancelling, and undoing
    /// afterwards, are both just putting this back.
    base: Sketch,
    /// Set when the constraints would not take the move, naming what refused. The
    /// geometry is left where it was; see [`SketchEditor::update_move`].
    pub refused: Option<String>,
}

impl MoveOp {
    /// Where a point should end up under the current numbers.
    fn target(&self, from: Vec2) -> Vec2 {
        let turned =
            self.pivot + Vec2::from_angle(self.angle_deg.to_radians()).rotate(from - self.pivot);
        turned + Vec2::new(self.dx, self.dy)
    }

    /// Where the rotation centre ends up, which is where the manipulator belongs.
    fn moved_pivot(&self) -> Vec2 {
        self.target(self.pivot)
    }
}

/// How far a moved shape may stray from rigid before the move is judged refused, as a
/// fraction of its own size. The solver converges to ~1e-9, so a genuinely rigid result
/// is far below this and anything above it is the constraints having found some other
/// answer entirely.
const RIGID_TOL: f64 = 1e-4;

/// What a rectangular pattern's distance means. Fusion offers both and the difference
/// is the single thing users get wrong about patterns: "20 mm apart" and "20 mm from
/// end to end" are the same number and a different drawing.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Spacing {
    /// The gap between one copy and the next.
    Between,
    /// The total span from the seed to the last copy.
    Total,
}

/// Settings of the pattern tool, kept between uses so repeating a pattern is one click
/// rather than four numbers again.
#[derive(Clone, Debug)]
pub struct PatternParams {
    pub circular: bool,
    pub spacing: Spacing,
    /// Copies across, the seed included, and how far they reach.
    pub count_x: usize,
    pub distance_x: f64,
    /// Copies up, the seed included, and how far they reach.
    pub count_y: usize,
    pub distance_y: f64,
    /// Copies around, the seed included, and through how much of a turn.
    pub count: usize,
    pub angle_deg: f64,
    pub center: Vec2,
}

impl Default for PatternParams {
    fn default() -> Self {
        Self {
            circular: false,
            spacing: Spacing::Between,
            count_x: 3,
            distance_x: 20.0,
            count_y: 1,
            distance_y: 20.0,
            count: 6,
            angle_deg: 360.0,
            center: Vec2::ZERO,
        }
    }
}

impl PatternParams {
    /// How many copies the settings make, not counting the original. This is the number
    /// the user is thinking in; the entity count the pattern module returns includes
    /// every point of every copy and means nothing to anyone.
    pub fn copies(&self) -> usize {
        let instances = if self.circular {
            self.count
        } else {
            self.count_x * self.count_y
        };
        instances.saturating_sub(1)
    }

    /// The step from one copy to the next along a direction, which is what the pattern
    /// module wants however the user chose to state it.
    fn step(&self, distance: f64, count: usize) -> f64 {
        match self.spacing {
            Spacing::Between => distance,
            // One copy is no span at all; dividing by the gaps rather than the copies is
            // what puts the last copy exactly on the stated total.
            Spacing::Total => distance / (count.max(2) - 1) as f64,
        }
    }
}

/// A pattern being set up. The copies are made in the live sketch so the user is looking
/// at the real thing rather than at a sketch of it, and every change re-makes them from
/// `base`, which is the sketch as it was before the tool started. That is what lets a
/// number be changed twice without the copies piling up.
pub struct PatternOp {
    seed: Vec<EntityId>,
    base: Sketch,
    /// Why the last attempt made nothing, for the palette to show.
    pub error: Option<String>,
    pub created: usize,
    /// The next click in the viewport places the circular pattern's centre.
    ///
    /// A mode rather than a standing behaviour: while a pattern is up the pointer is
    /// otherwise doing nothing, and a stray click that silently moved the centre of a
    /// pattern the user had already placed would be worse than no picking at all.
    picking_center: bool,
}

/// Settings of the offset tool, kept between uses.
#[derive(Clone, Debug)]
pub struct OffsetParams {
    /// Signed. Positive grows a closed shape; for an open chain the sides have no names
    /// the drawing can tell apart, so the sign is the side and there is no flip to press:
    /// drag the handle out the far side, or type the minus.
    pub distance: f64,
    pub corner: Corner,
}

impl Default for OffsetParams {
    fn default() -> Self {
        Self {
            distance: 5.0,
            corner: Corner::Round,
        }
    }
}

/// An offset being set up. Like a pattern, the result is made in the live sketch so the
/// user is looking at the real thing, and every change re-makes it from `base` — the
/// sketch as it was before the tool started — so a distance can be changed twice without
/// the offsets piling up.
pub struct OffsetOp {
    seed: Vec<EntityId>,
    base: Sketch,
    /// Why the last attempt made nothing, for the palette to show. An offset refuses far
    /// more often than a pattern does — a distance bigger than the shape, a corner too
    /// sharp to square off — so saying why is most of the tool.
    pub error: Option<String>,
    pub created: usize,
}

/// Settings of the sketch fillet tool, kept between uses the way the offset's are: the
/// radius of the last corner rounded is nearly always the radius of the next one.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FilletParams {
    pub radius: f64,
}

impl Default for FilletParams {
    fn default() -> Self {
        Self { radius: 5.0 }
    }
}

/// A corner fillet being set up. Like an offset, the arc is made in the live sketch so
/// the user is looking at the real thing, and every change of the radius re-makes it
/// from `base` — the sketch as it was before the tool started — so the radius can be
/// changed twice without the corner being eaten twice.
/// Every value in the sketch that a handle can drag, each remembering its own gesture.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct Drags {
    offset: snap::Drag,
    fillet: snap::Drag,
    /// The move's two axes, X then Y.
    move_xy: [snap::Drag; 2],
    turn: snap::Drag,
}

pub struct FilletOp {
    a: EntityId,
    hint_a: Vec2,
    b: EntityId,
    hint_b: Vec2,
    base: Sketch,
    /// Where the arc went, for the handle that drags its radius. `None` when the last
    /// attempt made nothing.
    plan: Option<fillet::Plan>,
    /// Why the last attempt made nothing, for the palette to say. A fillet refuses often
    /// — a radius bigger than the edges, a corner the curves do not actually make — and
    /// the reason is the difference between a tool that is broken and one that is busy.
    pub error: Option<String>,
}

/// Rubber-band selection in progress.
#[derive(Clone, Copy)]
struct Marquee {
    start: Vec2,
    current: Vec2,
}

impl Marquee {
    /// Dragging leftwards means *crossing*: anything the rectangle touches is selected.
    /// Dragging rightwards is a window: only what lies wholly inside. This is the CAD
    /// convention every user of Fusion or AutoCAD already has in their fingers.
    fn crossing(&self) -> bool {
        self.current.x < self.start.x
    }

    fn corners(&self) -> [Vec2; 4] {
        let (a, b) = (self.start, self.current);
        [a, Vec2::new(b.x, a.y), b, Vec2::new(a.x, b.y)]
    }
}

pub struct SketchEditor {
    pub feature: FeatureId,
    pub frame: Frame,
    pub sketch: Sketch,
    pub tool: SketchTool,
    /// Which variant each folded toolbar button shows, so going back to Circle gives
    /// the kind of circle used last.
    pub last_variant: Vec<SketchTool>,
    pub hover: Option<EntityId>,
    /// Geometry the palette wants lit up — the entities of the constraint row under the
    /// pointer. It is set from the panel each frame and drawn like a hover, which is how
    /// a row in the list points at the geometry it belongs to.
    pub highlighted: Vec<EntityId>,
    /// Closed region under the pointer when nothing else is, as an index into
    /// [`Self::profiles`]. Clicking it selects the curves around it.
    pub hover_region: Option<usize>,
    /// Where the next click will land, with typed sizes applied.
    pub cursor: Option<Vec2>,
    /// True when [`Self::cursor`] came from an existing point rather than the grid, so the
    /// marker can tell the user their next click will join geometry instead of making a
    /// new point.
    pub cursor_snapped: bool,
    /// World size of one pixel at the cursor, so the marker keeps its size on screen.
    cursor_px: f64,
    /// World size of one pixel at the sketch plane's origin. The badges use this rather
    /// than [`Self::cursor_px`] because it depends on the view alone: a scale measured
    /// where the pointer happens to be changes as the pointer moves, which under a
    /// perspective camera sized every badge by how far the pointer was from it and made
    /// the whole overlay breathe.
    view_px: f64,
    /// The constraint badge under the pointer, so hovering a badge can light up the
    /// geometry it holds — the palette's constraint list read the other way round.
    pub hovered_constraint: Option<ConstraintId>,
    /// Where each badge sits, rebuilt only when the sketch, the solve report or the view
    /// scale changes. Interior mutability because the overlay and the renderer both ask
    /// for the badges through `&self`.
    glyph_cache: std::cell::RefCell<GlyphCache>,
    /// The snapped pointer position before typed sizes are applied, kept so the cursor
    /// can be recomputed when an entry box changes without the pointer moving.
    raw_cursor: Option<Vec2>,
    pub selected: Vec<EntityId>,
    /// What the Select tool may pick. Drawing, snapping and the constraint tools ignore
    /// it: it is about choosing between things that are already there, not about what
    /// the pointer may touch.
    pub pick: SketchPick,
    /// Whether the next shape drawn is construction geometry. Deciding before drawing is
    /// how Fusion works and is what a centre line or a bolt circle actually needs: the
    /// alternative is drawing a real curve, watching it open or close a profile, and
    /// converting it afterwards.
    pub construction: bool,
    pub polygon_sides: usize,
    pub text: String,
    pub text_height: f64,
    /// Entry boxes for the current tool's sizes; empty for tools without any.
    pub entries: Vec<Entry>,
    /// Entry box that should take keyboard focus on the next frame, set when the user
    /// starts typing a number with the pointer in the viewport.
    pub entry_focus: Option<usize>,
    pub dim_edit: Option<(ConstraintId, String)>,
    /// What the armed constraint tool has been pointed at so far.
    ///
    /// Kept apart from [`Self::selected`] deliberately: a pick made as an argument to a
    /// constraint is not a selection, and if it were, Delete and the construction toggle
    /// would act on geometry the user only pointed at. The picks are drawn lit up all
    /// the same, which is the part the user actually wanted from a selection.
    constraint_picks: Vec<EntityId>,
    /// Why the constraint tool refused the last pick, for the editor to report once.
    constraint_error: Option<String>,
    /// The last solve. The error is kept whole rather than as a message because a failed
    /// solve names the constraints that disagree, and pointing at them is the only useful
    /// thing to say about it.
    pub report: Option<Result<SolveReport, SolveError>>,
    pub snap_to_grid: bool,
    /// The running totals of the handles that can be dragged, so each one snaps the
    /// travel the pointer has actually made rather than one frame's slice of it. See
    /// [`snap::Drag`].
    drags: Drags,
    /// Shift is down, so the grid lets go for as long as it is. The toggle says whether
    /// the drawing is built on a grid at all; this says "not this one placement", which
    /// is the far commoner thing to want and is not worth a trip to the palette and back.
    free_snap: bool,
    /// Where the pointer lands when the drawing itself names a place: the ranking, the
    /// hold that stops it flickering, and the short memory of touched points that the
    /// alignment guides grow from. See [`snap::Inference`].
    inference: snap::Inference,
    /// A step the user pinned, or `None` to follow the zoom.
    pub fixed_grid_step: Option<f64>,
    /// The step actually in use, for the palette to display. Updated as the pointer moves
    /// because it depends on the zoom.
    pub grid_step: f64,
    clicks: Vec<Click>,
    /// Last point of a line chain, so consecutive lines share their joint.
    chain_end: Option<EntityId>,
    drag: Option<Drag>,
    marquee: Option<Marquee>,
    /// Closed regions of the current geometry, recomputed after every change. The
    /// select tool offers them as pickable things, the way a face is in model mode.
    profiles: Vec<Profile>,
    /// First pick of the dimension tool, waiting for the second pick or a placing click.
    pub dim_first: Option<EntityId>,
    /// Closed regions the user has picked, named by a point inside each of them so they
    /// survive the re-solve that every edit causes. This is what `E` extrudes.
    pub selected_regions: Vec<Vec2>,
    /// The piece the trim tool would remove if clicked now, drawn as a warning.
    trim_preview: Option<Vec<Vec2>>,
    /// A keyboard-driven move of the selection, started with `M`.
    pub move_op: Option<MoveOp>,
    /// Settings of the pattern tool, kept between uses.
    pub pattern: PatternParams,
    /// The pattern being set up, if any.
    pattern_op: Option<PatternOp>,
    /// Settings of the offset tool, kept between uses.
    pub offset: OffsetParams,
    /// The offset being set up, if any.
    offset_op: Option<OffsetOp>,
    /// Settings of the fillet tool, kept between uses.
    pub fillet: FilletParams,
    /// The corner fillet being set up, if any.
    fillet_op: Option<FilletOp>,
    /// The first curve of a fillet and where it was clicked, while the second is awaited.
    /// The click position is kept because it is what says which side of the corner the
    /// user means — the same pick that chooses between the four arcs that would fit.
    fillet_pick: Option<(EntityId, Vec2)>,
    /// Editable text of the parameter panel: what the user is typing, which is not the
    /// same as what the sketch has accepted.
    pub param_drafts: Vec<(String, String)>,
    pub new_param: (String, String),
    pub param_error: Option<String>,
    undo: Vec<Sketch>,
    redo: Vec<Sketch>,
    dirty: bool,
    saved_camera: Camera,
    /// Cursor to put back when an edit of an existing sketch ends.
    restore_cursor: Option<usize>,
    tess: Tessellation,
}

impl SketchEditor {
    fn new(feature: FeatureId, frame: Frame, sketch: Sketch, saved_camera: Camera) -> Self {
        let mut s = Self {
            feature,
            frame,
            sketch,
            tool: SketchTool::Line,
            last_variant: ToolGroup::ALL.iter().map(|g| g.variants()[0]).collect(),
            hover: None,
            highlighted: Vec::new(),
            hover_region: None,
            cursor: None,
            cursor_snapped: false,
            cursor_px: 1.0,
            view_px: 1.0,
            hovered_constraint: None,
            glyph_cache: std::cell::RefCell::new(GlyphCache::default()),
            raw_cursor: None,
            selected: Vec::new(),
            pick: SketchPick::default(),
            construction: false,
            polygon_sides: 6,
            text: "Text".into(),
            text_height: 10.0,
            entries: Vec::new(),
            entry_focus: None,
            dim_edit: None,
            constraint_picks: Vec::new(),
            constraint_error: None,
            report: None,
            snap_to_grid: true,
            free_snap: false,
            inference: snap::Inference::default(),
            drags: Drags::default(),
            fixed_grid_step: None,
            grid_step: 1.0,
            clicks: Vec::new(),
            chain_end: None,
            drag: None,
            marquee: None,
            profiles: Vec::new(),
            dim_first: None,
            selected_regions: Vec::new(),
            trim_preview: None,
            move_op: None,
            pattern: PatternParams::default(),
            pattern_op: None,
            offset: OffsetParams::default(),
            offset_op: None,
            fillet: FilletParams::default(),
            fillet_op: None,
            fillet_pick: None,
            param_drafts: Vec::new(),
            new_param: (String::new(), String::new()),
            param_error: None,
            undo: Vec::new(),
            redo: Vec::new(),
            dirty: false,
            saved_camera,
            restore_cursor: None,
            tess: Tessellation::default(),
        };
        s.reset_entries();
        s.solve();
        s
    }

    pub fn take_dirty(&mut self) -> bool {
        std::mem::take(&mut self.dirty)
    }

    pub fn has_pending(&self) -> bool {
        !self.clicks.is_empty()
            || self.chain_end.is_some()
            || self.dim_first.is_some()
            || self.move_op.is_some()
            || self.offset_op.is_some()
            || self.fillet_op.is_some()
            || self.fillet_pick.is_some()
    }

    pub fn move_in_progress(&self) -> bool {
        self.move_op.is_some()
    }

    /// True while a modal operation owns the sketch. Each of them re-derives the result
    /// from a copy taken when it started, so anything else that edits the sketch
    /// meanwhile is silently thrown away the next time a number changes — and undo,
    /// which pops checkpoints none of them took, corrupts the stack outright. The editor
    /// therefore refuses those commands rather than losing the user's work, and only one
    /// modal operation runs at a time: a second begun on top of the first would take the
    /// first one's preview for its base and keep it on cancelling.
    pub fn modal(&self) -> bool {
        self.move_op.is_some()
            || self.pattern_op.is_some()
            || self.offset_op.is_some()
            || self.fillet_op.is_some()
    }

    /// What the modal operation in progress is called, for the messages that have to
    /// name it. `None` when nothing modal is running.
    pub fn modal_name(&self) -> Option<&'static str> {
        match (
            self.move_op.is_some(),
            self.pattern_op.is_some(),
            self.offset_op.is_some(),
            self.fillet_op.is_some(),
        ) {
            (true, ..) => Some("Move"),
            (_, true, ..) => Some("Pattern"),
            (_, _, true, _) => Some("Offset"),
            (.., true) => Some("Fillet"),
            _ => None,
        }
    }

    /// Ends whichever modal operation is running, keeping or discarding it.
    pub fn finish_modal(&mut self, keep: bool) {
        self.finish_move(keep);
        self.finish_pattern(keep);
        self.finish_offset(keep);
        self.finish_fillet(keep);
    }

    pub fn select_tool(&mut self) {
        self.set_tool(SketchTool::Select);
    }

    pub fn set_tool(&mut self, tool: SketchTool) {
        self.cancel_current();
        self.tool = tool;
        // Only the folded shape buttons remember a variant; a constraint tool has no
        // button of its own to remember it on.
        if let Some(slot) = ToolGroup::ALL.iter().position(|g| *g == tool.group()) {
            self.last_variant[slot] = tool;
        }
        self.reset_entries();
    }

    /// The variant a toolbar button currently stands for.
    pub fn variant_of(&self, group: ToolGroup) -> SketchTool {
        ToolGroup::ALL
            .iter()
            .position(|g| *g == group)
            .map(|i| self.last_variant[i])
            // `Constrain` is outside `ALL` and folds no variants, so indexing its empty
            // list eagerly would panic the moment anyone asked about it.
            .unwrap_or_else(|| {
                group
                    .variants()
                    .first()
                    .copied()
                    .unwrap_or(SketchTool::Select)
            })
    }

    pub fn cancel_current(&mut self) {
        self.finish_modal(false);
        self.constraint_picks.clear();
        self.finish_current();
    }

    /// Ends a line chain (right click / Enter) without leaving the tool.
    ///
    /// Deliberately narrower than [`Self::cancel_current`]: right-drag is also the orbit
    /// gesture, and orbiting to look at a pattern must not be the gesture that throws it
    /// away. Only what is half-drawn goes.
    pub fn finish_current(&mut self) {
        // The held snap goes with whatever was half-drawn: a hold the user can no longer
        // see the reason for is exactly the stickiness this is meant to avoid.
        self.inference.release();
        self.trim_preview = None;
        self.fillet_pick = None;
        self.clicks.clear();
        self.chain_end = None;
        self.dim_first = None;
        self.drag = None;
        self.marquee = None;
        self.reset_entries();
    }

    /// Empties the entry boxes. Typed sizes belong to one shape; the next one starts
    /// from what the pointer says, as it does in Fusion.
    fn reset_entries(&mut self) {
        // A move is typed in the same boxes a shape's sizes are, and for the same
        // reason: the number is usually known, and clicking into a field to say it is a
        // step the drawing does not need.
        if self.move_op.is_some() {
            self.entries = [Dim::Dx, Dim::Dy, Dim::Angle]
                .into_iter()
                .map(|dim| Entry {
                    dim,
                    text: String::new(),
                    locked: false,
                })
                .collect();
            // The first box takes the keyboard straight away, so a move begun with M can
            // be finished by typing a number and pressing Enter.
            self.entry_focus = Some(0);
            self.mirror_move_entries();
            return;
        }
        // An offset is one number, and it is typed in the same box for the same reason:
        // a clearance is usually a stated figure, and dragging to it is the fine
        // adjustment rather than the way it is said.
        if self.offset_op.is_some() {
            self.entries = vec![Entry {
                dim: Dim::Distance,
                text: String::new(),
                locked: false,
            }];
            self.entry_focus = Some(0);
            self.mirror_offset_entry();
            return;
        }
        // A fillet is one number too, and the box is where it is stated exactly; the
        // handle on the drawing is the way it is found.
        if self.fillet_op.is_some() {
            self.entries = vec![Entry {
                dim: Dim::Radius,
                text: String::new(),
                locked: false,
            }];
            self.entry_focus = Some(0);
            self.mirror_fillet_entry();
            return;
        }
        self.entries = self
            .tool
            .dims()
            .iter()
            .map(|dim| Entry {
                dim: *dim,
                text: String::new(),
                locked: false,
            })
            .collect();
        self.entry_focus = None;
        self.refresh_cursor();
    }

    fn checkpoint(&mut self) {
        let before = self.sketch.clone();
        self.push_undo(before);
    }

    /// Puts `before` on the undo stack as one step. The stack is bounded, because a
    /// session's worth of edits held in full copies of the sketch is otherwise unbounded
    /// memory.
    fn push_undo(&mut self, before: Sketch) {
        self.undo.push(before);
        if self.undo.len() > 100 {
            self.undo.remove(0);
        }
        self.redo.clear();
    }

    pub fn undo(&mut self) -> bool {
        let Some(prev) = self.undo.pop() else {
            return false;
        };
        self.redo.push(std::mem::replace(&mut self.sketch, prev));
        self.after_change();
        true
    }

    pub fn redo(&mut self) -> bool {
        let Some(next) = self.redo.pop() else {
            return false;
        };
        self.undo.push(std::mem::replace(&mut self.sketch, next));
        self.after_change();
        true
    }

    fn after_change(&mut self) {
        self.solve();
        self.selected.retain(|id| self.sketch.entity(*id).is_some());
        // A region that the edit opened up is no longer selectable; one that merely
        // changed shape still contains its sample point and stays picked.
        let profiles = std::mem::take(&mut self.profiles);
        self.selected_regions
            .retain(|p| profiles.iter().any(|f| f.contains(*p)));
        self.profiles = profiles;
        self.dirty = true;
    }

    fn solve(&mut self) {
        self.report = Some(self.sketch.solve());
        self.profiles = self.sketch.profiles(&self.tess);
        self.hover_region = None;
    }

    /// Narrows or widens what the Select tool takes, dropping anything already selected
    /// that the new filter no longer covers — being left holding things the mode gives
    /// no way to see or deselect is worse than having no filter.
    pub fn set_pick(&mut self, mode: SketchPick) {
        self.pick = mode;
        let kept: Vec<EntityId> = self
            .selected
            .iter()
            .copied()
            .filter(|id| self.pickable(*id))
            .collect();
        self.selected = kept;
        if !self.pick.takes_regions() {
            self.selected_regions.clear();
            self.hover_region = None;
        }
    }

    /// Whether the Select tool may take this entity under the current filter.
    fn pickable(&self, id: EntityId) -> bool {
        match self.sketch.entity(id) {
            Some(data) if data.entity.is_point() => self.pick.takes_points(),
            Some(_) => self.pick.takes_curves(),
            None => false,
        }
    }

    /// The nearest thing under `pos` the filter allows.
    fn pick_at(&self, pos: Vec2, tol: f64) -> Option<EntityId> {
        self.sketch
            .hit_test(pos, tol)
            .into_iter()
            .map(|h| h.entity)
            .find(|id| self.pickable(*id))
    }

    /// The smallest closed region around `pos`, if any.
    fn region_at(&self, pos: Vec2) -> Option<usize> {
        if !self.pick.takes_regions() {
            return None;
        }
        self.profiles
            .iter()
            .enumerate()
            .filter(|(_, p)| p.contains(pos))
            .min_by(|(_, a), (_, b)| a.area().total_cmp(&b.area()))
            .map(|(i, _)| i)
    }

    /// The curves bounding a region, outer loop and holes alike.
    fn region_curves(&self, index: usize) -> Vec<EntityId> {
        let mut out = Vec::new();
        if let Some(p) = self.profiles.get(index) {
            for c in std::iter::once(&p.outer).chain(p.holes.iter()) {
                for seg in &c.segments {
                    if !out.contains(&seg.curve) {
                        out.push(seg.curve);
                    }
                }
            }
        }
        out
    }

    /// True when everything selected is construction geometry, so the toolbar can show
    /// the construction toggle as "on".
    pub fn selection_is_construction(&self) -> bool {
        !self.selected.is_empty()
            && self
                .selected
                .iter()
                .all(|id| self.sketch.entity(*id).is_some_and(|e| e.construction))
    }

    // --- Pointer -------------------------------------------------------------------

    fn to_plane(&self, ray: &Ray) -> Option<Vec2> {
        let t = self.frame.plane().intersect_ray(ray)?;
        Some(self.frame.to_local(ray.at(t)))
    }

    fn tolerance(&self, pos: Vec2, camera: &Camera, window: [u32; 2]) -> f64 {
        camera.pixel_size_at(self.frame.to_world(pos), window) * 8.0
    }

    /// The grid increment in force at this moment: the pinned one, or the finest that is
    /// still comfortably far apart on screen at the current zoom.
    fn step_for(&self, pos: Vec2, camera: &Camera, window: [u32; 2]) -> f64 {
        match self.fixed_grid_step {
            Some(step) if step.is_finite() && step > 0.0 => step,
            _ => grid::snap_step_for(camera.pixel_size_at(self.frame.to_world(pos), window)),
        }
    }

    /// Where the pointer lands: whatever the drawing names there, and the grid when it
    /// names nothing.
    ///
    /// The whole ranking — an existing point over an implied place over a curve over a
    /// guide line — and the hold that keeps the answer still while the hand shakes live
    /// in [`snap::Inference`], asked here rather than written out again. Landing on a
    /// curve records the curve, so the point is held there by a constraint rather than
    /// by where the grid happened to put it: without that a divider drawn to an edge
    /// only *looks* attached, and the next re-solve is free to move it off and silently
    /// open the regions either side of it.
    fn snap(&mut self, pos: Vec2, tol: f64) -> Click {
        let cont = self.continuation();
        // Shift, or the palette's switch turned off, leaves only the joins: see the
        // snap module's header for why those two mean the same thing here.
        let joins_only = self.free_snap || !self.snap_to_grid;
        let grid = self
            .snap_rule()
            .is_on()
            .then(|| self.snap_rule().point(pos));
        match self
            .inference
            .resolve(&self.sketch, cont, pos, tol, joins_only, grid)
        {
            Some(found) => Click {
                pos: found.at,
                snapped: found.point,
                on_curve: found.curve,
            },
            None => Click {
                pos: self.to_grid(pos),
                snapped: None,
                on_curve: None,
            },
        }
    }

    /// Where a single dragged point is being asked to go. The same inference the
    /// drawing tools use, minus the joining: a drag moves a point that already exists
    /// and sharing it with another one is a different operation than this gesture.
    fn drag_goal(&mut self, dragged: EntityId, pos: Vec2, tol: f64) -> Vec2 {
        // The point being dragged is its own nearest snap, and the curves hanging off
        // it follow it about, so neither is any guide to where it should land: they are
        // named here so the inference passes over them.
        let cont = snap::Continuation {
            moving: Some(dragged),
            ..Default::default()
        };
        let joins_only = self.free_snap || !self.snap_to_grid;
        let grid = self
            .snap_rule()
            .is_on()
            .then(|| self.snap_rule().point(pos));
        match self
            .inference
            .resolve(&self.sketch, cont, pos, tol, joins_only, grid)
        {
            Some(found) => found.at,
            None => self.to_grid(pos),
        }
    }

    /// What the point being placed is continuing from, which is what makes tangent and
    /// perpendicular mean anything. A line chain continues from its open end; every
    /// other tool continues from its last click.
    fn continuation(&self) -> snap::Continuation {
        let (from, curve) = match self.tool {
            SketchTool::Line => (
                self.chain_end.and_then(|id| self.sketch.point_pos(id)),
                self.chain_curve(),
            ),
            _ => (self.clicks.last().map(|c| c.pos), None),
        };
        snap::Continuation {
            from,
            curve,
            moving: None,
        }
    }

    /// The curve the open end of the chain belongs to — the one just drawn, when there
    /// is more than one, because that is the direction the hand is carrying.
    fn chain_curve(&self) -> Option<EntityId> {
        let end = self.chain_end?;
        self.sketch
            .entities()
            .filter(|(_, data)| {
                data.entity.is_open_curve() && data.entity.references().contains(&end)
            })
            .map(|(id, _)| id)
            .last()
    }

    fn to_grid(&self, pos: Vec2) -> Vec2 {
        self.snap_rule().point(pos)
    }

    /// How a drag in this sketch meets the grid right now. Everything the sketch snaps
    /// goes through this, so the toggle, shift and the increment are read once.
    ///
    /// Snapping to an existing *point* is a different thing and is never given up: it is
    /// how geometry gets joined, and shift is for escaping the grid, not for drawing
    /// something that only looks attached.
    pub fn snap_rule(&self) -> Snap {
        Snapping {
            to_grid: self.snap_to_grid,
            free: self.free_snap,
        }
        .at(self.grid_step)
    }

    /// Forgets every handle's running total, at the end of a drag. Without this the
    /// next gesture would carry on from the last one's raw position rather than from
    /// where the value now is, and a value changed in between — typed into a box, or
    /// undone — would be overwritten by travel the pointer made before it.
    pub fn release_drags(&mut self) {
        self.drags = Drags::default();
    }

    /// Tells the sketch whether shift is down. Called from wherever the modifier is
    /// known rather than read at the point of use, because the drawing path comes from
    /// winit and the manipulators come from egui and neither can see the other's.
    pub fn set_free_snap(&mut self, on: bool) {
        self.free_snap = on;
    }

    /// Snaps, then applies typed sizes. A typed value beats the snap: the user has said
    /// exactly what they want, and a snapped point almost never lies at that size.
    fn aim(&mut self, pos: Vec2, tol: f64) -> Click {
        let click = self.snap(pos, tol);
        let pos = self.constrained_cursor(click.pos);
        if pos == click.pos {
            click
        } else {
            Click {
                pos,
                snapped: None,
                on_curve: None,
            }
        }
    }

    fn typed(&self, dim: Dim) -> Option<f64> {
        typed_value(&self.entries, dim)
    }

    /// Where the next click lands once typed sizes are applied. A typed size pins the
    /// distance from the previous click while the pointer still chooses direction and
    /// side, so the shape follows the mouse at exactly the size that was entered.
    fn constrained_cursor(&self, raw: Vec2) -> Vec2 {
        if self.entries.iter().all(|e| e.value().is_none()) {
            return raw;
        }
        let first = self.clicks.first().map(|c| c.pos);
        match self.tool {
            SketchTool::Line => {
                let Some(start) = self.chain_end.and_then(|id| self.sketch.point_pos(id)) else {
                    return raw;
                };
                let delta = raw - start;
                let angle = self.typed(Dim::Angle);
                let dir = match angle {
                    Some(a) => Vec2::from_angle(a),
                    None => delta.normalize_or(Vec2::X),
                };
                // With only the angle typed, the line reaches as far along that
                // direction as the pointer does.
                let length = self.typed(Dim::Length).unwrap_or_else(|| {
                    if angle.is_some() {
                        delta.dot(dir).max(0.0)
                    } else {
                        delta.length()
                    }
                });
                start + dir * length
            }
            SketchTool::Rectangle | SketchTool::CenterRectangle => {
                let Some(first) = first else { return raw };
                // A centre rectangle's second click is a corner, half the size away.
                let scale = if self.tool == SketchTool::CenterRectangle {
                    0.5
                } else {
                    1.0
                };
                let axis = |cur: f64, origin: f64, size: Option<f64>| match size {
                    Some(s) => origin + s * scale * if cur < origin { -1.0 } else { 1.0 },
                    None => cur,
                };
                Vec2::new(
                    axis(raw.x, first.x, self.typed(Dim::Width)),
                    axis(raw.y, first.y, self.typed(Dim::Height)),
                )
            }
            SketchTool::Circle => {
                at_distance(first, raw, self.typed(Dim::Diameter).map(|d| d * 0.5))
            }
            SketchTool::Circle2Point => at_distance(first, raw, self.typed(Dim::Diameter)),
            SketchTool::Polygon => at_distance(first, raw, self.typed(Dim::Radius)),
            // The arc's third click only picks the end direction; the radius is fixed by
            // the second.
            SketchTool::ArcCenter if self.clicks.len() == 1 => {
                at_distance(first, raw, self.typed(Dim::Radius))
            }
            SketchTool::Slot | SketchTool::SlotOverall | SketchTool::SlotCenterPoint => {
                match self.clicks[..] {
                    // The second click's distance from the first is what the typed
                    // length means for that variant: centre to centre, end to end, or
                    // (from the middle) half of centre to centre.
                    [_] => {
                        let length = self.typed(Dim::Length).map(|l| {
                            if self.tool == SketchTool::SlotCenterPoint {
                                l * 0.5
                            } else {
                                l
                            }
                        });
                        at_distance(first, raw, length)
                    }
                    [a, b] => match self.typed(Dim::Width) {
                        Some(width) => {
                            let axis = (b.pos - a.pos).normalize_or(Vec2::X);
                            let side = if axis.perp_dot(raw - a.pos) < 0.0 {
                                -1.0
                            } else {
                                1.0
                            };
                            let foot = a.pos + axis * axis.dot(raw - a.pos);
                            foot + axis.perp() * (side * width * 0.5)
                        }
                        None => raw,
                    },
                    _ => raw,
                }
            }
            _ => raw,
        }
    }

    /// Re-derives the cursor from the last pointer position, for when a typed size
    /// changes while the pointer stands still.
    pub fn refresh_cursor(&mut self) {
        // While a move is running the boxes drive the geometry rather than the pointer,
        // so there is no cursor to re-derive anything from.
        if self.move_op.is_some() {
            self.drive_move_from_entries();
            return;
        }
        if self.offset_op.is_some() {
            self.drive_offset_from_entry();
            return;
        }
        if self.fillet_op.is_some() {
            self.drive_fillet_from_entry();
            return;
        }
        if let Some(raw) = self.raw_cursor {
            let constrained = self.constrained_cursor(raw);
            if constrained != raw {
                self.cursor_snapped = false;
            }
            self.cursor = Some(constrained);
            let live = self.live_dims(constrained);
            for entry in self.entries.iter_mut().filter(|e| !e.locked) {
                if let Some((_, v)) = live.iter().find(|(d, _)| *d == entry.dim) {
                    entry.text = match entry.dim {
                        Dim::Angle => format!("{:.2}", v.to_degrees()),
                        _ => format!("{v:.2}"),
                    };
                }
            }
        }
    }

    /// The sizes the pointer is drawing right now, for the live entry boxes.
    fn live_dims(&self, cursor: Vec2) -> Vec<(Dim, f64)> {
        let first = self.clicks.first().map(|c| c.pos);
        match self.tool {
            SketchTool::Line => {
                let Some(start) = self.chain_end.and_then(|id| self.sketch.point_pos(id)) else {
                    return Vec::new();
                };
                let d = cursor - start;
                vec![(Dim::Length, d.length()), (Dim::Angle, d.to_angle())]
            }
            SketchTool::Rectangle | SketchTool::CenterRectangle => {
                let Some(first) = first else {
                    return Vec::new();
                };
                let scale = if self.tool == SketchTool::CenterRectangle {
                    2.0
                } else {
                    1.0
                };
                let d = (cursor - first).abs() * scale;
                vec![(Dim::Width, d.x), (Dim::Height, d.y)]
            }
            SketchTool::Circle => first
                .map(|f| vec![(Dim::Diameter, 2.0 * f.distance(cursor))])
                .unwrap_or_default(),
            SketchTool::Circle2Point => first
                .map(|f| vec![(Dim::Diameter, f.distance(cursor))])
                .unwrap_or_default(),
            SketchTool::Polygon | SketchTool::ArcCenter => first
                .filter(|_| self.clicks.len() == 1)
                .map(|f| vec![(Dim::Radius, f.distance(cursor))])
                .unwrap_or_default(),
            SketchTool::Slot | SketchTool::SlotOverall | SketchTool::SlotCenterPoint => {
                let scale = if self.tool == SketchTool::SlotCenterPoint {
                    2.0
                } else {
                    1.0
                };
                match self.clicks[..] {
                    [a] => vec![(Dim::Length, a.pos.distance(cursor) * scale)],
                    [a, b] => vec![
                        (Dim::Length, a.pos.distance(b.pos) * scale),
                        (Dim::Width, 2.0 * distance_to_line(cursor, a.pos, b.pos)),
                    ],
                    _ => Vec::new(),
                }
            }
            _ => Vec::new(),
        }
    }

    /// The user edited an entry box: from now on it drives the shape.
    pub fn lock_entry(&mut self, index: usize) {
        if let Some(e) = self.entries.get_mut(index) {
            e.locked = true;
        }
        self.refresh_cursor();
    }

    /// Releases a locked box back to following the pointer.
    pub fn unlock_entry(&mut self, index: usize) {
        if let Some(e) = self.entries.get_mut(index) {
            e.locked = false;
        }
        self.refresh_cursor();
    }

    /// Tab with the pointer in the viewport: focus the first box that is still following
    /// the pointer, so the next keystrokes fill it. Inside the box egui's own Tab moves on.
    pub fn focus_next_entry(&mut self) -> bool {
        if self.entries.is_empty() || !self.has_pending() {
            return false;
        }
        let index = self.entries.iter().position(|e| !e.locked).unwrap_or(0);
        self.entry_focus = Some(index);
        true
    }

    fn place_cursor(&mut self, click: Click) {
        self.raw_cursor = Some(click.pos);
        self.cursor_snapped = click.snapped.is_some();
        self.cursor = Some(click.pos);
        self.refresh_cursor();
    }

    pub fn pointer_moved(&mut self, ray: &Ray, camera: &Camera, window: [u32; 2], dragging: bool) {
        let Some(pos) = self.to_plane(ray) else {
            self.cursor = None;
            self.raw_cursor = None;
            self.hovered_constraint = None;
            return;
        };
        self.grid_step = self.step_for(pos, camera, window);
        self.cursor_px = camera.pixel_size_at(self.frame.to_world(pos), window);
        self.view_px = camera.pixel_size_at(self.frame.to_world(Vec2::ZERO), window);
        // Badges are hit-tested here rather than by the egui area over them, because the
        // highlight belongs to the drawing: the area's job is the tooltip and the
        // context menu, and it is rebuilt from this layout anyway.
        self.hovered_constraint = self.constraint_at(pos);
        let tol = self.tolerance(pos, camera, window);
        if self.pattern_op.is_some() {
            // The crosshair follows the pointer only while a centre is being picked;
            // nothing else in the sketch may be touched.
            self.cursor = self
                .picking_pattern_center()
                .then(|| self.snap(pos, tol).pos);
            self.cursor_snapped = self.cursor.is_some_and(|p| {
                self.sketch.hit_test(p, tol).iter().any(|h| {
                    self.sketch
                        .entity(h.entity)
                        .is_some_and(|e| e.entity.is_point())
                })
            });
            return;
        }
        // Every other modal operation owns the sketch outright, so the crosshair goes
        // away rather than promising a click that will not land.
        if self.modal() {
            self.cursor = None;
            return;
        }
        if self.tool == SketchTool::Select {
            self.cursor = Some(pos);
            self.raw_cursor = Some(pos);
            self.cursor_snapped = false;
        } else {
            let click = self.snap(pos, tol);
            self.place_cursor(click);
        }
        if dragging {
            if let Some(m) = &mut self.marquee {
                m.current = pos;
                return;
            }
            if let Some(drag) = self.drag.clone() {
                // The pointer's travel is snapped to the grid, so nudging a corner does
                // not silently take the sketch off it and a moved shape stays on it.
                //
                // One point on its own is aimed rather than nudged: it goes where the
                // pointer says, and so it is offered the drawing's own places too — the
                // middle of that line, level with that corner. Several points are a
                // shape being moved, and a shape has no one position to infer for, so
                // those keep the grid.
                let delta = match drag.points[..] {
                    [(id, _)] => {
                        let goal = self.drag_goal(id, pos, tol);
                        goal - self.sketch.point_pos(id).unwrap_or(goal)
                    }
                    _ => self.to_grid(pos) - self.to_grid(drag.press),
                };
                let goals: Vec<(EntityId, Vec2)> = drag
                    .points
                    .iter()
                    .map(|(id, start)| (*id, *start + delta))
                    .collect();
                if self.sketch.drag_points(&goals).is_ok() {
                    self.dirty = true;
                }
                return;
            }
        }
        // The filter applies to the Select tool alone: the drawing tools need to see
        // every point to snap to, whatever the user is choosing to select.
        self.hover = match self.tool {
            SketchTool::Select => self.pick_at(pos, tol),
            _ => self.sketch.hit_test(pos, tol).first().map(|h| h.entity),
        };
        self.hover_region = match (self.tool, self.hover) {
            (SketchTool::Select, None) => self.region_at(pos),
            _ => None,
        };
        self.update_trim_preview(pos, tol);
    }

    /// The points that move when `id` is dragged: the whole selection if it is part of
    /// it, otherwise just the entity under the pointer.
    fn drag_set(&self, id: EntityId) -> Vec<(EntityId, Vec2)> {
        let entities: Vec<EntityId> = if self.selected.contains(&id) {
            self.selected.clone()
        } else {
            vec![id]
        };
        let mut points: Vec<(EntityId, Vec2)> = Vec::new();
        for e in entities {
            for p in self.sketch.entity_points(e) {
                if !points.iter().any(|(q, _)| *q == p)
                    && let Some(pos) = self.sketch.point_pos(p)
                {
                    points.push((p, pos));
                }
            }
        }
        points
    }

    pub fn pointer_down(&mut self, ray: &Ray, camera: &Camera, window: [u32; 2], _shift: bool) {
        // A modal operation re-makes the whole sketch from the copy it started with, so
        // a drag made underneath it would be thrown away by the next change of a number
        // — and would leave its checkpoint on the undo stack pointing at a state that no
        // longer follows from anything. The manipulator and the boxes are how the
        // geometry moves while one is running.
        if self.tool != SketchTool::Select || self.modal() {
            return;
        }
        let Some(pos) = self.to_plane(ray) else {
            return;
        };
        self.grid_step = self.step_for(pos, camera, window);
        let tol = self.tolerance(pos, camera, window);
        match self.pick_at(pos, tol) {
            // Anything can be dragged: a point alone, a curve by all its points, or the
            // whole selection when the press lands on part of it.
            Some(id) => {
                self.checkpoint();
                self.drag = Some(Drag {
                    points: self.drag_set(id),
                    press: pos,
                });
            }
            // Pressing on empty space starts a rubber band. It only becomes a selection if
            // the pointer actually travels; a press and release in place is still a click.
            None => {
                self.marquee = Some(Marquee {
                    start: pos,
                    current: pos,
                })
            }
        }
    }

    pub fn pointer_up(
        &mut self,
        ray: &Ray,
        camera: &Camera,
        window: [u32; 2],
        clicked: bool,
        shift: bool,
    ) {
        if let Some(drag) = self.drag.take() {
            self.marquee = None;
            if !clicked {
                self.solve();
                self.dirty = true;
                return;
            }
            // A press and release without travel is a click, not a move: the
            // checkpoint taken for the drag is not a change.
            self.undo.pop();
            let _ = drag;
        }
        if let Some(m) = self.marquee.take()
            && !clicked
        {
            self.select_in(m, shift);
            return;
        }
        if !clicked {
            return;
        }
        let Some(pos) = self.to_plane(ray) else {
            return;
        };
        self.grid_step = self.step_for(pos, camera, window);
        let tol = self.tolerance(pos, camera, window);
        // While a modal operation is up the viewport belongs to it: a click on a
        // circular pattern puts its centre where the user pointed, which is how one is
        // actually placed, and nothing else may edit the sketch underneath it.
        if self.pattern_op.is_some() {
            if self.picking_pattern_center() {
                // Snapping means the centre can be put on an existing point — the middle
                // of a bolt circle is nearly always a point that is already drawn.
                self.pattern.center = self.snap(pos, tol).pos;
                self.pick_pattern_center(false);
                self.update_pattern();
            }
            return;
        }
        if self.modal() {
            return;
        }
        match self.tool {
            SketchTool::Select => {
                let hit = self.pick_at(pos, tol);
                match hit {
                    Some(id) if shift => match self.selected.iter().position(|s| *s == id) {
                        Some(i) => {
                            self.selected.remove(i);
                        }
                        None => self.selected.push(id),
                    },
                    Some(id) => {
                        self.clear_selection();
                        self.selected.push(id);
                    }
                    // Empty space inside a closed region picks the region: the curves
                    // around it, which is what moving, deleting or constraining it means.
                    None => match self.region_at(pos) {
                        Some(region) => {
                            let curves = self.region_curves(region);
                            let sample = self.region_sample(region);
                            if !shift {
                                self.clear_selection();
                            }
                            // Shift on a region already wholly selected takes it out.
                            if shift && curves.iter().all(|c| self.selected.contains(c)) {
                                self.selected.retain(|c| !curves.contains(c));
                                self.selected_regions
                                    .retain(|p| !self.profiles[region].contains(*p));
                            } else {
                                for c in curves {
                                    if !self.selected.contains(&c) {
                                        self.selected.push(c);
                                    }
                                }
                                // The region itself is remembered as well as its curves:
                                // the curves are what a constraint acts on, the region is
                                // what an extrude does.
                                if let Some(sample) = sample
                                    && !self.selected_regions.contains(&sample)
                                {
                                    self.selected_regions.push(sample);
                                }
                            }
                        }
                        None => self.clear_selection(),
                    },
                }
            }
            SketchTool::Dimension => self.dimension_click(pos, tol),
            SketchTool::Constrain(kind) => self.constraint_click(kind, pos, tol),
            SketchTool::Trim | SketchTool::Break => self.trim_click(pos, tol),
            SketchTool::Fillet => self.fillet_click(pos, tol),
            SketchTool::Line => {
                let click = self.aim(pos, tol);
                // A placed point is a touched point: the next one may want to line up
                // with it, which is the whole of what the alignment guides are for.
                self.inference.touch(click.pos);
                self.line_click(click);
            }
            _ => {
                let click = self.aim(pos, tol);
                self.inference.touch(click.pos);
                self.shape_click(click);
            }
        }
    }

    /// Applies a finished rubber band. Shift adds to the selection the way shift-clicking
    /// does; without it the band replaces what was selected.
    fn select_in(&mut self, m: Marquee, shift: bool) {
        let hits = self.sketch.hit_test_rect(m.start, m.current, m.crossing());
        if !shift {
            self.clear_selection();
        }
        let allowed: Vec<EntityId> = hits.into_iter().filter(|id| self.pickable(*id)).collect();
        for id in allowed {
            if !self.selected.contains(&id) {
                self.selected.push(id);
            }
        }
        // A box in Regions mode takes the areas it encloses, which is how a handful of
        // profiles are handed to an extrude in one gesture.
        if self.pick.takes_regions() {
            let (min, max) = (m.start.min(m.current), m.start.max(m.current));
            let enclosed: Vec<Vec2> = (0..self.profiles.len())
                .filter_map(|i| self.region_sample(i))
                .filter(|p| point_in_rect(*p, min, max))
                .collect();
            for sample in enclosed {
                if !self.selected_regions.contains(&sample) {
                    self.selected_regions.push(sample);
                }
            }
        }
    }

    // --- Regions ---------------------------------------------------------------------

    /// A point inside a region, which is how a region is named in a feature reference.
    fn region_sample(&self, index: usize) -> Option<Vec2> {
        self.profiles.get(index)?.interior_point()
    }

    /// The regions a modelling tool would act on: the ones the user picked, or the one
    /// under the pointer when they picked none. Pressing E with the pointer over a
    /// region extrudes it without a click first, as Fusion's press-pull does.
    pub fn region_samples(&self) -> Vec<Vec2> {
        if !self.selected_regions.is_empty() {
            return self.selected_regions.clone();
        }
        self.hover_region
            .and_then(|r| self.region_sample(r))
            .into_iter()
            .collect()
    }

    pub fn has_region_selection(&self) -> bool {
        !self.region_samples().is_empty()
    }

    fn clear_selection(&mut self) {
        self.selected.clear();
        self.selected_regions.clear();
    }

    /// Replaces the selection with exactly these entities, as picking them one by one
    /// would. Used by the constraint list, where clicking a row means "show me what this
    /// holds"; the regions go because a set of curves and a filled region are different
    /// kinds of selection and keeping both would leave the palette offering nonsense.
    pub fn select_only(&mut self, entities: Vec<EntityId>) {
        self.clear_selection();
        self.selected = entities;
    }

    // --- Trim and break --------------------------------------------------------------

    fn curve_at(&self, pos: Vec2, tol: f64) -> Option<EntityId> {
        self.sketch
            .hit_test(pos, tol)
            .into_iter()
            .find(|h| {
                self.sketch
                    .entity(h.entity)
                    .is_some_and(|d| d.entity.is_curve())
            })
            .map(|h| h.entity)
    }

    fn trim_click(&mut self, pos: Vec2, tol: f64) {
        let Some(curve) = self.curve_at(pos, tol) else {
            return;
        };
        self.checkpoint();
        let result = match self.tool {
            SketchTool::Break => edit::break_curve(&mut self.sketch, curve),
            _ => edit::trim(&mut self.sketch, curve, pos),
        };
        match result {
            Ok(_) => self.after_change(),
            Err(e) => {
                // Nothing partial is left behind: the sketch goes back to the copy the
                // checkpoint took a moment ago.
                log::warn!("{}: {e}", self.tool.name());
                if let Some(prev) = self.undo.pop() {
                    self.sketch = prev;
                }
            }
        }
        self.trim_preview = None;
    }

    /// Keeps the "this is what will go" highlight in step with the pointer.
    fn update_trim_preview(&mut self, pos: Vec2, tol: f64) {
        self.trim_preview = match self.tool {
            SketchTool::Trim => self
                .curve_at(pos, tol)
                .and_then(|c| edit::trim_preview(&self.sketch, c, pos, &self.tess)),
            _ => None,
        };
    }

    // --- Move ------------------------------------------------------------------------

    /// Starts a typed move of the selection. `false` when nothing is selected, so the
    /// key press can fall through to whatever else `M` might mean.
    pub fn begin_move(&mut self) -> bool {
        if self.selected.is_empty() || self.modal() {
            return false;
        }
        let selected = self.selected.clone();
        let mut start: Vec<(EntityId, Vec2)> = Vec::new();
        for entity in selected {
            for p in self.sketch.entity_points(entity) {
                if !start.iter().any(|(q, _)| *q == p)
                    && let Some(pos) = self.sketch.point_pos(p)
                {
                    start.push((p, pos));
                }
            }
        }
        if start.is_empty() {
            return false;
        }
        let pivot =
            start.iter().map(|(_, p)| *p).fold(Vec2::ZERO, |a, p| a + p) / start.len() as f64;
        self.move_op = Some(MoveOp {
            dx: 0.0,
            dy: 0.0,
            angle_deg: 0.0,
            start,
            pivot,
            base: self.sketch.clone(),
            refused: None,
        });
        self.reset_entries();
        true
    }

    /// Where the manipulator sits: the rotation centre where the current numbers put it,
    /// so the arrows travel with the geometry instead of staying behind at the start.
    pub fn move_pivot(&self) -> Option<Vec2> {
        self.move_op.as_ref().map(MoveOp::moved_pivot)
    }

    /// Why the constraints would not take the move, for the palette to say.
    pub fn move_refused(&self) -> Option<&str> {
        self.move_op.as_ref()?.refused.as_deref()
    }

    /// Adds to the move's offset along one of the sketch plane's axes, for the viewport
    /// manipulator. The numbers it changes are the ones the boxes show, so dragging and
    /// typing are two ways of saying the same thing.
    ///
    /// The result is snapped like every other position in the sketch: a drawing built on
    /// a grid stays on it, and a dragged arrow that left geometry at 49.87 mm would
    /// quietly undo the point of drawing on a grid at all.
    pub fn nudge_move(&mut self, along_x: bool, distance: f64) -> bool {
        let snap = self.snap_rule();
        let Some(op) = self.move_op.as_mut() else {
            return false;
        };
        let drag = &mut self.drags.move_xy[usize::from(!along_x)];
        let offset = if along_x { &mut op.dx } else { &mut op.dy };
        *offset = drag.advance(*offset, distance, snap);
        true
    }

    /// Adds to the move's rotation, for the viewport manipulator's ring. Snapped to
    /// whole steps for the same reason the offsets are; a ring dragged to 37.4° is
    /// almost never what was meant.
    pub fn turn_move(&mut self, degrees: f64) -> bool {
        let snap = self.snap_rule();
        let Some(op) = self.move_op.as_mut() else {
            return false;
        };
        op.angle_deg = self.drags.turn.advance_angle(op.angle_deg, degrees, snap);
        true
    }

    /// Writes the move's current numbers into the boxes that are not locked, so a drag
    /// on the manipulator reads back as a number the moment it happens.
    fn mirror_move_entries(&mut self) {
        let Some(op) = self.move_op.as_ref() else {
            return;
        };
        let live = [
            (Dim::Dx, op.dx),
            (Dim::Dy, op.dy),
            (Dim::Angle, op.angle_deg),
        ];
        for entry in self.entries.iter_mut().filter(|e| !e.locked) {
            if let Some((_, v)) = live.iter().find(|(d, _)| *d == entry.dim) {
                entry.text = format!("{v:.2}");
            }
        }
    }

    /// Takes whatever has been typed into the boxes and moves the geometry by it.
    fn drive_move_from_entries(&mut self) {
        let typed = [
            (Dim::Dx, typed_value(&self.entries, Dim::Dx)),
            (Dim::Dy, typed_value(&self.entries, Dim::Dy)),
            (Dim::Angle, typed_value(&self.entries, Dim::Angle)),
        ];
        let Some(op) = self.move_op.as_mut() else {
            return;
        };
        for (dim, value) in typed {
            let Some(v) = value else { continue };
            match dim {
                Dim::Dx => op.dx = v,
                Dim::Dy => op.dy = v,
                // The box is in degrees but `Entry::value` hands back radians, as the
                // line tool's angle wants.
                _ => op.angle_deg = v.to_degrees(),
            }
        }
        self.update_move();
    }

    /// Re-applies the move to the sketch as it was when the move began.
    ///
    /// A move is a rigid transform, and the result has to be one. When the constraints
    /// cannot take it — asking an axis-constrained rectangle to turn thirty degrees, say
    /// — the solver does not fail; it finds some *other* arrangement that does satisfy
    /// them, and the cheapest such arrangement is usually the shape folded flat onto
    /// itself. That is a converged solve and a destroyed drawing, so the residual cannot
    /// tell them apart. Measuring the shape can: if the moved points no longer hold
    /// their distances to each other, the move was refused, and the geometry stays where
    /// it was with the palette saying why.
    pub fn update_move(&mut self) {
        let Some(op) = self.move_op.as_ref() else {
            return;
        };
        let (base, start) = (op.base.clone(), op.start.clone());
        let goals: Vec<(EntityId, Vec2)> = start
            .iter()
            .map(|(id, from)| (*id, op.target(*from)))
            .collect();
        self.sketch = base.clone();
        let outcome = self.sketch.drag_points(&goals);
        let refused = match outcome {
            Err(e) => Some(e.to_string()),
            Ok(_) => rigid_error(&self.sketch, &start).map(|_| {
                "the constraints will not allow this move: something is holding this \
                 geometry to the axes or to a dimension"
                    .to_string()
            }),
        };
        if refused.is_some() {
            self.sketch = base;
        }
        if let Some(op) = self.move_op.as_mut() {
            op.refused = refused;
        }
        self.solve();
        self.mirror_move_entries();
        self.dirty = true;
    }

    /// Ends the move. Keeping it makes the whole thing one step of undo; cancelling puts
    /// back the sketch the move started from.
    pub fn finish_move(&mut self, keep: bool) {
        let Some(op) = self.move_op.take() else {
            return;
        };
        if keep {
            // The base goes on the undo stack now rather than when the move started, so
            // a move that was cancelled leaves no step behind and one that was kept is
            // exactly one.
            self.push_undo(op.base);
        } else {
            self.sketch = op.base;
        }
        self.after_change();
    }

    // --- Pattern ---------------------------------------------------------------------

    /// Starts a pattern of the selection. `false` when nothing is selected, so the
    /// command can say why instead of doing nothing.
    ///
    /// The copies appear at once from the settings last used, because a pattern with no
    /// preview is a pattern the user has to undo to understand.
    pub fn begin_pattern(&mut self) -> bool {
        if self.selected.is_empty() || self.modal() {
            return false;
        }
        self.pattern_center_from_selection();
        self.pattern_op = Some(PatternOp {
            seed: self.selected.clone(),
            base: self.sketch.clone(),
            error: None,
            created: 0,
            picking_center: false,
        });
        self.update_pattern();
        true
    }

    pub fn pattern_in_progress(&self) -> bool {
        self.pattern_op.is_some()
    }

    /// Whether the next click places the circular pattern's centre.
    pub fn picking_pattern_center(&self) -> bool {
        self.pattern_op.as_ref().is_some_and(|op| op.picking_center)
    }

    /// Arms or disarms picking the circular pattern's centre in the viewport.
    pub fn pick_pattern_center(&mut self, on: bool) {
        if let Some(op) = self.pattern_op.as_mut() {
            op.picking_center = on;
        }
    }

    /// Re-makes every copy from the sketch as it was before the tool started, so editing
    /// a number replaces the pattern rather than adding a second one on top of it.
    pub fn update_pattern(&mut self) {
        let Some(op) = self.pattern_op.as_ref() else {
            return;
        };
        let (seed, base) = (op.seed.clone(), op.base.clone());
        let p = self.pattern.clone();
        self.sketch = base;
        let result = if p.circular {
            pattern::circular(
                &mut self.sketch,
                &seed,
                p.center,
                p.count,
                p.angle_deg.to_radians(),
            )
        } else {
            pattern::rectangular(
                &mut self.sketch,
                &seed,
                Vec2::new(p.step(p.distance_x, p.count_x), 0.0),
                p.count_x,
                Vec2::new(0.0, p.step(p.distance_y, p.count_y)),
                p.count_y,
            )
        };
        let Some(op) = self.pattern_op.as_mut() else {
            return;
        };
        match result {
            Ok(created) => {
                op.created = created.len();
                op.error = None;
            }
            Err(e) => {
                op.created = 0;
                op.error = Some(e.to_string());
            }
        }
        // A failed pattern leaves the base sketch, which is what the user had before —
        // never a half-made one.
        if self.pattern_op.as_ref().is_some_and(|o| o.error.is_some()) {
            self.sketch = self.pattern_op.as_ref().expect("just checked").base.clone();
        }
        self.solve();
        self.dirty = true;
    }

    /// Ends the pattern. Keeping it makes the whole thing one step of undo; cancelling
    /// puts back the sketch the tool started from.
    pub fn finish_pattern(&mut self, keep: bool) -> Option<usize> {
        let op = self.pattern_op.take()?;
        if keep {
            self.push_undo(op.base);
            self.after_change();
            Some(op.created)
        } else {
            self.sketch = op.base;
            self.after_change();
            None
        }
    }

    /// What the palette needs to say about the pattern being set up: how many copies are
    /// on screen right now, and why there are none if there are none.
    pub fn pattern_status(&self) -> Option<(usize, Option<&str>)> {
        let op = self.pattern_op.as_ref()?;
        let copies = if op.error.is_some() {
            0
        } else {
            self.pattern.copies()
        };
        Some((copies, op.error.as_deref()))
    }

    /// The entities the pattern preview added, so they can be drawn as the provisional
    /// things they are rather than as geometry the user has already committed to.
    fn previewed(&self) -> std::collections::HashSet<EntityId> {
        let base = match (
            self.pattern_op.as_ref(),
            self.offset_op.as_ref(),
            self.fillet_op.as_ref(),
        ) {
            (Some(op), ..) => &op.base,
            (_, Some(op), _) => &op.base,
            (.., Some(op)) => &op.base,
            _ => return std::collections::HashSet::new(),
        };
        self.sketch
            .entities()
            .map(|(id, _)| id)
            .filter(|id| base.entity(*id).is_none())
            .collect()
    }

    /// Centres a circular pattern on what is selected, which is almost always what the
    /// user means by "around here".
    pub fn pattern_center_from_selection(&mut self) {
        let points: Vec<Vec2> = self
            .selected
            .iter()
            .flat_map(|id| self.sketch.entity_points(*id))
            .filter_map(|p| self.sketch.point_pos(p))
            .collect();
        if !points.is_empty() {
            self.pattern.center =
                points.iter().fold(Vec2::ZERO, |a, p| a + *p) / points.len() as f64;
        }
    }

    // --- Offset ----------------------------------------------------------------------

    /// Starts an offset of the selection. `false` when nothing is selected, so the
    /// command can say why instead of doing nothing.
    ///
    /// The result appears at once from the settings last used, for the same reason a
    /// pattern's copies do: which side an offset went, and what its corners did, are
    /// things to look at rather than to imagine.
    pub fn begin_offset(&mut self) -> bool {
        if self.selected.is_empty() || self.modal() {
            return false;
        }
        self.offset_op = Some(OffsetOp {
            seed: self.selected.clone(),
            base: self.sketch.clone(),
            error: None,
            created: 0,
        });
        self.update_offset();
        // The box appears with the offset and takes the keyboard straight away, so an
        // offset begun with O can be finished by typing the clearance and pressing Enter.
        self.reset_entries();
        true
    }

    pub fn offset_in_progress(&self) -> bool {
        self.offset_op.is_some()
    }

    /// Re-makes the offset from the sketch as it was before the tool started, so editing
    /// the distance replaces it rather than offsetting the offset.
    pub fn update_offset(&mut self) {
        let Some(op) = self.offset_op.as_ref() else {
            return;
        };
        let (seed, base) = (op.seed.clone(), op.base.clone());
        let (distance, corner) = (self.offset.distance, self.offset.corner);
        self.sketch = base;
        let result = offset::offset(&mut self.sketch, &seed, distance, corner);
        let Some(op) = self.offset_op.as_mut() else {
            return;
        };
        match result {
            Ok(created) => {
                op.created = created.len();
                op.error = None;
            }
            Err(e) => {
                op.created = 0;
                op.error = Some(e.to_string());
                // A refused offset leaves the sketch the user had, never a half-made one.
                self.sketch = op.base.clone();
            }
        }
        self.solve();
        self.mirror_offset_entry();
        self.dirty = true;
    }

    /// Writes the distance into the box, so whatever moved the offset — the handle, the
    /// palette's own spinner — reads back as a number at once.
    ///
    /// A box the user has typed into is left alone while it still says what the offset
    /// is made at, half-typed minus and all. One that says some *other* number has been
    /// overtaken by something else driving the same distance, and a box disagreeing with
    /// the geometry beside it is worse than no box at all, so it goes back to mirroring.
    fn mirror_offset_entry(&mut self) {
        let distance = self.offset.distance;
        for entry in self.entries.iter_mut().filter(|e| e.dim == Dim::Distance) {
            if entry.locked
                && entry
                    .text
                    .trim()
                    .parse::<f64>()
                    .ok()
                    .is_none_or(|typed| typed == distance)
            {
                continue;
            }
            entry.locked = false;
            entry.text = format!("{distance:.2}");
        }
    }

    /// Re-makes the offset at whatever has been typed into the box.
    ///
    /// A typed distance is taken exactly: the grid is there to steady a pointer, and a
    /// user who has typed 3.2 has already said what they want. Nothing happens when the
    /// number has not changed, so the idle refreshes that come with every keystroke
    /// elsewhere do not rebuild the geometry.
    fn drive_offset_from_entry(&mut self) {
        let Some(v) = typed_value(&self.entries, Dim::Distance) else {
            return;
        };
        if self.offset_op.is_none() || v == self.offset.distance {
            return;
        }
        self.offset.distance = v;
        self.update_offset();
    }

    /// Where the offset's drag handle belongs in the sketch plane, and which way its
    /// distance grows from there. `None` when no offset is running.
    pub fn offset_handle(&self) -> Option<(Vec2, Vec2)> {
        let op = self.offset_op.as_ref()?;
        offset::handle(&op.base, &op.seed)
    }

    /// Drags the offset's distance by `by` along the handle's direction.
    ///
    /// Dragging back across the geometry and out the other side takes the distance
    /// through zero and negative, which is how the side is chosen: the offset is on the
    /// side the pointer is, and there is no flip to go and press.
    pub fn nudge_offset(&mut self, by: f64) -> bool {
        if self.offset_op.is_none() {
            return false;
        }
        let snap = self.snap_rule();
        self.offset.distance = self.drags.offset.advance(self.offset.distance, by, snap);
        // A drag is the user saying the distance again, so it takes the box back off
        // whatever was typed into it rather than leaving a number that disagrees with
        // the geometry under the pointer.
        for entry in self.entries.iter_mut().filter(|e| e.dim == Dim::Distance) {
            entry.locked = false;
        }
        self.mirror_offset_entry();
        true
    }

    /// Ends the offset. Keeping it makes the whole thing one step of undo; cancelling
    /// puts back the sketch the tool started from.
    pub fn finish_offset(&mut self, keep: bool) -> Option<usize> {
        let op = self.offset_op.take()?;
        // The distance box belongs to the offset, not to the sketch it leaves behind.
        self.reset_entries();
        if keep && op.error.is_none() && op.created > 0 {
            self.push_undo(op.base);
            self.after_change();
            Some(op.created)
        } else {
            self.sketch = op.base;
            self.after_change();
            None
        }
    }

    /// What the palette needs to say: how many curves are on screen, and why there are
    /// none if there are none.
    pub fn offset_status(&self) -> Option<(usize, Option<&str>)> {
        let op = self.offset_op.as_ref()?;
        Some((op.created, op.error.as_deref()))
    }

    // --- Fillet ----------------------------------------------------------------------

    /// A click with the fillet tool armed: pick a corner, or the two curves either side
    /// of one.
    ///
    /// Pointing at the corner itself is the shorter way to say the same thing and is how
    /// most fillets are asked for, so a click on a point where exactly two curves end
    /// takes both of them at once. Anything else is the two-pick route, which is what a
    /// corner the curves do not actually reach needs.
    fn fillet_click(&mut self, pos: Vec2, tol: f64) {
        if let Some(point) = self.point_at(pos, tol)
            && let Some((a, b)) = fillet::curves_at(&self.sketch, point)
            && let Some(corner) = self.sketch.point_pos(point)
            && let (Some(hint_a), Some(hint_b)) = (
                fillet::hint_along(&self.sketch, a, corner),
                fillet::hint_along(&self.sketch, b, corner),
            )
        {
            self.begin_fillet(a, hint_a, b, hint_b);
            return;
        }
        let Some(curve) = self.curve_at(pos, tol) else {
            // A click on nothing takes back a half-made pick rather than leaving a curve
            // lit up that the next click would silently pair with something far away.
            self.fillet_pick = None;
            return;
        };
        match self.fillet_pick.take() {
            Some((first, hint)) if first != curve => {
                self.begin_fillet(first, hint, curve, pos);
            }
            _ => self.fillet_pick = Some((curve, pos)),
        }
    }

    /// The point entity under the pointer, if the pointer is on one.
    fn point_at(&self, pos: Vec2, tol: f64) -> Option<EntityId> {
        self.sketch
            .hit_test(pos, tol)
            .into_iter()
            .find(|h| {
                self.sketch
                    .entity(h.entity)
                    .is_some_and(|d| d.entity.is_point())
            })
            .map(|h| h.entity)
    }

    /// Starts a fillet between two curves, at the radius last used. `false` when
    /// something modal is already running, so the caller can say why.
    pub fn begin_fillet(&mut self, a: EntityId, hint_a: Vec2, b: EntityId, hint_b: Vec2) -> bool {
        if self.modal() {
            return false;
        }
        self.fillet_pick = None;
        self.fillet_op = Some(FilletOp {
            a,
            hint_a,
            b,
            hint_b,
            base: self.sketch.clone(),
            plan: None,
            error: None,
        });
        self.update_fillet();
        // The box appears with the arc and takes the keyboard straight away, so a fillet
        // can be finished by typing the radius and pressing Enter.
        self.reset_entries();
        true
    }

    pub fn fillet_in_progress(&self) -> bool {
        self.fillet_op.is_some()
    }

    /// Re-makes the fillet from the sketch as it was before the tool started, so changing
    /// the radius replaces the arc rather than rounding the rounded corner.
    pub fn update_fillet(&mut self) {
        let Some(op) = self.fillet_op.as_ref() else {
            return;
        };
        let (a, hint_a, b, hint_b) = (op.a, op.hint_a, op.b, op.hint_b);
        let base = op.base.clone();
        let radius = self.fillet.radius;
        self.sketch = base;
        let result = fillet::fillet(&mut self.sketch, a, hint_a, b, hint_b, radius);
        let Some(op) = self.fillet_op.as_mut() else {
            return;
        };
        match result {
            Ok(made) => {
                op.plan = Some(made.plan);
                op.error = None;
            }
            Err(e) => {
                op.plan = None;
                op.error = Some(e.to_string());
                // A refused fillet leaves the sketch the user had, never a half-cut
                // corner.
                self.sketch = op.base.clone();
            }
        }
        self.solve();
        self.mirror_fillet_entry();
        self.dirty = true;
    }

    /// Writes the radius into the box, so the handle and the box always agree. Same rule
    /// as the offset's: a box still saying what the arc is made at is left alone, one
    /// saying something else has been overtaken and goes back to mirroring.
    fn mirror_fillet_entry(&mut self) {
        let radius = self.fillet.radius;
        for entry in self.entries.iter_mut().filter(|e| e.dim == Dim::Radius) {
            if entry.locked
                && entry
                    .text
                    .trim()
                    .parse::<f64>()
                    .ok()
                    .is_none_or(|typed| typed == radius)
            {
                continue;
            }
            entry.locked = false;
            entry.text = format!("{radius:.2}");
        }
    }

    /// Re-makes the fillet at whatever has been typed into the box. A typed radius is
    /// taken exactly: the grid steadies a pointer, and a user who typed 3.2 has said
    /// what they want.
    fn drive_fillet_from_entry(&mut self) {
        let Some(v) = typed_value(&self.entries, Dim::Radius) else {
            return;
        };
        if self.fillet_op.is_none() || v == self.fillet.radius {
            return;
        }
        self.fillet.radius = v;
        self.update_fillet();
    }

    /// Where the fillet's drag handle belongs in the sketch plane, and which way its
    /// radius grows from there: on the bisector of the corner, pointing into the round.
    ///
    /// The handle sits exactly the radius from the corner, so what the user is dragging
    /// *is* the number — pull away from the corner and the fillet grows, which is also
    /// the way the arc itself travels. `None` before the first radius that fits, because
    /// there is no corner drawn to put a handle on.
    pub fn fillet_handle(&self) -> Option<(Vec2, Vec2)> {
        let plan = self.fillet_op.as_ref()?.plan?;
        Some((plan.corner, plan.bisector()))
    }

    /// Drags the fillet's radius by `by` along the handle's direction.
    ///
    /// Unlike an offset's distance a radius has no far side to cross into: dragging back
    /// through the corner stops at the smallest fillet there is rather than turning it
    /// inside out.
    pub fn nudge_fillet(&mut self, by: f64) -> bool {
        if self.fillet_op.is_none() {
            return false;
        }
        // The shared rule, so the radius snaps and shift frees it exactly as every other
        // dragged value does.
        let snap = self.snap_rule();
        let floor = snap
            .step()
            .unwrap_or(MIN_FILLET_RADIUS)
            .max(MIN_FILLET_RADIUS);
        self.fillet.radius = self
            .drags
            .fillet
            .advance_above(self.fillet.radius, by, snap, floor);
        // A drag is the user saying the radius again, so it takes the box back off
        // whatever was typed into it.
        for entry in self.entries.iter_mut().filter(|e| e.dim == Dim::Radius) {
            entry.locked = false;
        }
        self.mirror_fillet_entry();
        true
    }

    /// Ends the fillet. Keeping it makes the whole thing one step of undo; cancelling
    /// puts back the sketch the tool started from. `true` when a rounded corner was kept.
    pub fn finish_fillet(&mut self, keep: bool) -> bool {
        let Some(op) = self.fillet_op.take() else {
            return false;
        };
        // The radius box belongs to the fillet, not to the sketch it leaves behind.
        self.reset_entries();
        if keep && op.error.is_none() && op.plan.is_some() {
            self.push_undo(op.base);
            self.after_change();
            true
        } else {
            self.sketch = op.base;
            self.after_change();
            false
        }
    }

    /// What the palette needs to say: whether there is an arc on screen, and why there is
    /// none if there is none.
    pub fn fillet_status(&self) -> Option<(bool, Option<&str>)> {
        let op = self.fillet_op.as_ref()?;
        Some((op.plan.is_some(), op.error.as_deref()))
    }

    /// What the fillet tool is waiting for, for the status line. `None` when it is not
    /// the armed tool or is already running.
    pub fn fillet_prompt(&self) -> Option<&'static str> {
        if self.tool != SketchTool::Fillet || self.fillet_op.is_some() {
            return None;
        }
        Some(match self.fillet_pick {
            Some(_) => "Fillet: pick the second curve",
            None => "Fillet: pick a corner, or the first of two curves",
        })
    }

    // --- Parameters ------------------------------------------------------------------

    /// Refreshes the panel's draft text from the sketch when the set of parameters has
    /// changed underneath it (undo, a load, a rejected edit).
    pub fn sync_param_drafts(&mut self) {
        let live: Vec<(String, String)> = self
            .sketch
            .parameters()
            .iter()
            .map(|p| (p.name.clone(), p.expr.clone()))
            .collect();
        let names: Vec<&String> = live.iter().map(|(n, _)| n).collect();
        if self.param_drafts.len() != live.len()
            || !self.param_drafts.iter().all(|(n, _)| names.contains(&n))
        {
            self.param_drafts = live;
        }
    }

    /// Adds or re-expresses a named constant, re-driving the dimensions that use it.
    pub fn set_parameter(&mut self, name: &str, expression: &str) -> Result<(), String> {
        self.checkpoint();
        match self.sketch.set_parameter(name, expression) {
            Ok(_) => {
                self.after_change();
                Ok(())
            }
            Err(e) => {
                self.undo.pop();
                Err(e.to_string())
            }
        }
    }

    pub fn remove_parameter(&mut self, name: &str) {
        self.checkpoint();
        if self.sketch.remove_parameter(name) {
            self.after_change();
        } else {
            self.undo.pop();
        }
    }

    /// Drives a dimension by an expression instead of a number.
    pub fn bind_dimension(&mut self, id: ConstraintId, expression: &str) -> Result<(), String> {
        self.checkpoint();
        match self.sketch.bind_dimension(id, expression) {
            Ok(_) => {
                self.after_change();
                Ok(())
            }
            Err(e) => {
                self.undo.pop();
                Err(e.to_string())
            }
        }
    }

    // --- Entry boxes -----------------------------------------------------------------

    /// Routes a character typed with the pointer in the viewport into an entry box, so
    /// the user can start typing a size without clicking the box first. Returns `false`
    /// for anything that is not part of a number, leaving it to the key bindings.
    pub fn type_into_entry(&mut self, text: &str) -> bool {
        if self.entries.is_empty() || !self.has_pending() {
            return false;
        }
        if text.is_empty()
            || !text
                .chars()
                .all(|c| c.is_ascii_digit() || c == '.' || c == '-')
        {
            return false;
        }
        // A typed number replaces the value outright, so whatever travel a drag had
        // banked no longer refers to anything. Without this the next drag would carry on
        // from the old raw total and the typed figure would vanish under it.
        self.release_drags();
        // Typing continues in the box that is taking focus, otherwise starts in the
        // first one still following the pointer, so "10 Tab 5" fills width then height.
        // The first keystroke replaces the live value rather than appending to it.
        let index = self
            .entry_focus
            .unwrap_or_else(|| self.entries.iter().position(|e| !e.locked).unwrap_or(0));
        let entry = &mut self.entries[index];
        if !entry.locked {
            entry.text.clear();
            entry.locked = true;
        }
        entry.text.push_str(text);
        self.entry_focus = Some(index);
        self.refresh_cursor();
        true
    }

    /// Enter while drawing: finishes the shape from the locked sizes, taking whatever
    /// the pointer still decides (direction, side, unlocked sizes) from where it is now.
    pub fn submit_entry(&mut self) {
        self.refresh_cursor();
        // Enter in a move's box applies the move, as it places a shape from its sizes.
        if self.move_op.is_some() {
            self.finish_move(true);
            return;
        }
        // And in an offset's box it keeps the offset, which is what OK in the palette
        // does; an offset that made nothing is a cancel, and `finish_offset` says so.
        if self.offset_op.is_some() {
            self.finish_offset(true);
            return;
        }
        if self.fillet_op.is_some() {
            self.finish_fillet(true);
            return;
        }
        let Some(cursor) = self.cursor else { return };
        let click = Click {
            pos: cursor,
            snapped: None,
            on_curve: None,
        };
        match self.tool {
            SketchTool::Line if self.chain_end.is_some() => self.line_click(click),
            tool if tool.clicks() > 0 && self.clicks.len() + 1 == tool.clicks() => {
                self.shape_click(click)
            }
            _ => {}
        }
    }

    fn params(&self) -> ShapeParams {
        ShapeParams {
            sides: self.polygon_sides.max(3),
            text: self.text.clone(),
            text_height: self.text_height,
            typed: self
                .entries
                .iter()
                .filter_map(|e| e.value().map(|v| (e.dim, v)))
                .collect(),
        }
    }

    // --- Drawing tools ---------------------------------------------------------------

    fn line_click(&mut self, click: Click) {
        let end = match click.snapped {
            Some(id) => id,
            None => {
                self.checkpoint();
                let point = self.sketch.add_point(click.pos);
                if let Some(target) = click.on_curve
                    && let Err(e) = self
                        .sketch
                        .add_constraint(Constraint::Coincident { point, target })
                {
                    log::warn!("point on curve: {e}");
                }
                point
            }
        };
        if let Some(start) = self.chain_end {
            if start != end {
                // A new point took its checkpoint above; a snapped one has not yet.
                if click.snapped.is_some() {
                    self.checkpoint();
                }
                match self.sketch.add_line(start, end) {
                    Ok(line) => {
                        if self.construction {
                            let _ = self.sketch.set_construction(line, true);
                        }
                        for c in line_dims(line, start, end, &self.params()) {
                            if let Err(e) = self.sketch.add_constraint(c) {
                                log::warn!("line dimension: {e}");
                            }
                        }
                    }
                    Err(e) => log::warn!("line: {e}"),
                }
                self.after_change();
            }
            // Snapping back onto an existing point closes the chain.
            self.chain_end = if click.snapped.is_some() {
                None
            } else {
                Some(end)
            };
        } else {
            self.chain_end = Some(end);
        }
        self.reset_entries();
    }

    fn shape_click(&mut self, click: Click) {
        self.clicks.push(click);
        if self.clicks.len() < self.tool.clicks() {
            return;
        }
        let clicks = std::mem::take(&mut self.clicks);
        let params = self.params();
        self.checkpoint();
        let before = self.entity_ids();
        match build_shape(&mut self.sketch, self.tool, &clicks, &params) {
            Ok(()) => {
                self.mark_new_as_construction(&before);
                self.after_change()
            }
            Err(e) => {
                // A degenerate shape (collinear circle points…) leaves nothing behind,
                // not even the partial entities the builder had already added.
                log::warn!("{}: {e}", self.tool.name());
                if let Some(prev) = self.undo.pop() {
                    self.sketch = prev;
                }
            }
        }
        self.reset_entries();
    }

    // --- Dimension tool --------------------------------------------------------------

    /// Fusion's dimension tool: the first click picks an entity, the second either picks
    /// a partner or, on empty space, places a dimension of the first alone. What the
    /// dimension measures follows from the pair, see [`Self::dimension_between`].
    fn dimension_click(&mut self, pos: Vec2, tol: f64) {
        let hit = self
            .sketch
            .hit_test(pos, tol)
            .into_iter()
            .next()
            .map(|h| h.entity);
        let constraint = match (self.dim_first.take(), hit) {
            (None, Some(id)) => {
                let dimensionable = self
                    .sketch
                    .entity(id)
                    .is_some_and(|e| e.entity.is_point() || e.entity.is_curve());
                if dimensionable {
                    self.dim_first = Some(id);
                }
                None
            }
            (None, None) => None,
            (Some(first), second) => self.dimension_between(first, second),
        };
        if let Some(c) = constraint {
            self.checkpoint();
            match self.sketch.add_constraint(c.clone()) {
                Ok(id) => {
                    self.dim_edit = Some((id, edit_text(&c)));
                    self.after_change();
                }
                Err(e) => log::warn!("dimension: {e}"),
            }
        }
    }

    /// The dimension a pair of picks means, with its current value so adding it moves
    /// nothing:
    ///
    /// - a line alone: its length; a circle alone: its diameter; an arc alone: its radius
    /// - two parallel lines: the distance between them; two other lines: their angle
    /// - a line and a point: their distance; circles and arcs stand in for their centres
    /// - two points (or centres): their distance
    fn dimension_between(&self, first: EntityId, second: Option<EntityId>) -> Option<Constraint> {
        let kind = |id: EntityId| self.sketch.entity(id).map(|e| e.entity.clone());
        let a = kind(first)?;
        let Some(second) = second else {
            return match a {
                Entity::Line { start, end } => Some(Constraint::Distance {
                    a: start,
                    b: end,
                    value: self.distance(start, end),
                }),
                Entity::Circle { radius, .. } => Some(Constraint::Diameter {
                    curve: first,
                    value: radius * 2.0,
                }),
                Entity::Arc { center, start, .. } => Some(Constraint::Radius {
                    curve: first,
                    value: self.distance(center, start),
                }),
                _ => None,
            };
        };
        if second == first {
            return None;
        }
        let b = kind(second)?;
        // In a distance, a circle or arc means its centre, as it does in Fusion.
        let as_point = |id: EntityId, e: &Entity| match *e {
            Entity::Point { .. } => Some(id),
            Entity::Circle { center, .. } | Entity::Arc { center, .. } => Some(center),
            _ => None,
        };
        let point_line = |p: EntityId, line: EntityId| Constraint::Distance {
            a: p,
            b: line,
            value: self.point_line(p, line),
        };
        match (&a, &b) {
            (Entity::Line { start, .. }, Entity::Line { .. }) => {
                if self.parallel(first, second) {
                    Some(point_line(*start, second))
                } else {
                    Some(Constraint::Angle {
                        a: first,
                        b: second,
                        value: self.angle_between(first, second),
                    })
                }
            }
            (Entity::Line { .. }, other) => as_point(second, other).map(|p| point_line(p, first)),
            (other, Entity::Line { .. }) => as_point(first, other).map(|p| point_line(p, second)),
            (pa, pb) => match (as_point(first, pa), as_point(second, pb)) {
                (Some(p), Some(q)) if p != q => Some(Constraint::Distance {
                    a: p,
                    b: q,
                    value: self.distance(p, q),
                }),
                _ => None,
            },
        }
    }

    fn distance(&self, a: EntityId, b: EntityId) -> f64 {
        match (self.sketch.point_pos(a), self.sketch.point_pos(b)) {
            (Some(a), Some(b)) => a.distance(b),
            _ => 0.0,
        }
    }

    fn point_line(&self, p: EntityId, line: EntityId) -> f64 {
        match (self.sketch.point_pos(p), self.sketch.curve_endpoints(line)) {
            (Some(p), Some((a, b))) => distance_to_line(p, a, b),
            _ => 0.0,
        }
    }

    fn parallel(&self, a: EntityId, b: EntityId) -> bool {
        let (Some((a0, a1)), Some((b0, b1))) = (
            self.sketch.curve_endpoints(a),
            self.sketch.curve_endpoints(b),
        ) else {
            return false;
        };
        let (u, v) = (
            (a1 - a0).normalize_or(Vec2::X),
            (b1 - b0).normalize_or(Vec2::X),
        );
        u.perp_dot(v).abs() <= PARALLEL_TOL
    }

    /// Signed angle from the first line's direction to the second's, matching the
    /// solver's convention. Keeping the sign is what stops a fresh angle dimension from
    /// swinging the second line to the mirror-image position.
    fn angle_between(&self, a: EntityId, b: EntityId) -> f64 {
        let (Some((a0, a1)), Some((b0, b1))) = (
            self.sketch.curve_endpoints(a),
            self.sketch.curve_endpoints(b),
        ) else {
            return 0.0;
        };
        (a1 - a0).angle_to(b1 - b0)
    }

    // --- Commands --------------------------------------------------------------------

    pub fn delete_selected(&mut self) {
        if self.selected.is_empty() {
            return;
        }
        self.checkpoint();
        for id in std::mem::take(&mut self.selected) {
            self.sketch.remove_entity(id);
        }
        self.after_change();
    }

    /// What `X` means: convert the selection, or, with nothing selected, arm the mode so
    /// the next shape is drawn as construction. The toolbar splits these into two
    /// buttons; a keystroke can read its context, a lit button cannot.
    pub fn toggle_construction(&mut self) {
        if self.selected.is_empty() {
            self.construction = !self.construction;
            return;
        }
        self.convert_construction();
    }

    /// Makes the selection construction geometry, or ordinary geometry if it already is.
    pub fn convert_construction(&mut self) {
        if self.selected.is_empty() {
            return;
        }
        self.checkpoint();
        // Mixed selections go all-construction first, which is the answer that leaves
        // the user looking at what they asked for rather than at an inverted half.
        let target = !self.selection_is_construction();
        for id in self.selected.clone() {
            let _ = self.sketch.set_construction(id, target);
        }
        self.after_change();
    }

    /// Marks everything added since `before` as construction, so a shape drawn with the
    /// mode on is construction from the moment it exists. Points are left alone: they
    /// are shared with whatever else joins them and carry no profile meaning.
    fn mark_new_as_construction(&mut self, before: &std::collections::HashSet<EntityId>) {
        if !self.construction {
            return;
        }
        let fresh: Vec<EntityId> = self
            .sketch
            .entities()
            .filter(|(id, data)| !before.contains(id) && !data.entity.is_point())
            .map(|(id, _)| id)
            .collect();
        for id in fresh {
            let _ = self.sketch.set_construction(id, true);
        }
    }

    fn entity_ids(&self) -> std::collections::HashSet<EntityId> {
        self.sketch.entities().map(|(id, _)| id).collect()
    }

    pub fn set_dimension(&mut self, id: ConstraintId, value: f64) {
        self.checkpoint();
        if self.sketch.set_dimension_value(id, value).is_ok() {
            self.after_change();
        }
    }

    pub fn remove_constraint(&mut self, id: ConstraintId) {
        self.checkpoint();
        self.sketch.remove_constraint(id);
        self.after_change();
    }

    /// The constraint tool is armed: what it is waiting for, for the palette to say.
    pub fn armed_constraint(&self) -> Option<ConstraintKind> {
        match self.tool {
            SketchTool::Constrain(kind) => Some(kind),
            _ => None,
        }
    }

    /// What the armed constraint tool has been pointed at, for the palette and the
    /// viewport to light up.
    pub fn constraint_picks(&self) -> &[EntityId] {
        &self.constraint_picks
    }

    /// Arms a constraint tool, or disarms it when it is already the armed one and has
    /// nothing half-picked — a button that only ever turns on is a trap.
    ///
    /// Geometry already selected is not thrown away: if it already supports the
    /// constraint it is applied at once, and otherwise it becomes the tool's opening
    /// picks, so selecting first and selecting after both work.
    pub fn begin_constraint(&mut self, kind: ConstraintKind) -> Result<(), String> {
        if self.armed_constraint() == Some(kind) && self.constraint_picks.is_empty() {
            self.select_tool();
            return Ok(());
        }
        let selected = std::mem::take(&mut self.selected);
        self.set_tool(SketchTool::Constrain(kind));
        self.constraint_picks = selected;
        let result = self.apply_constraint_picks(kind);
        // Picks that cannot ever become this constraint are not kept: the user would be
        // left adding to a set the tool has already given up on.
        if !self.viable_picks(kind, &self.constraint_picks) {
            self.constraint_picks.clear();
        }
        result
    }

    /// Applies `kind` if the picks support it. A transitive constraint keeps its last
    /// pick, so a third click ties onto the second rather than starting again; every
    /// other kind starts clean.
    fn apply_constraint_picks(&mut self, kind: ConstraintKind) -> Result<(), String> {
        let constraints = self.constraints_for(kind, &self.constraint_picks);
        if constraints.is_empty() {
            return Ok(());
        }
        let result = self.add_constraints(constraints);
        if result.is_ok() {
            self.constraint_picks = match (kind.chains(), self.constraint_picks.last()) {
                (true, Some(last)) => vec![*last],
                _ => Vec::new(),
            };
        }
        result
    }

    /// Whether these picks could still become `kind` once there are more of them.
    ///
    /// A constraint tool that quietly swallows a pick it can never use leaves the user
    /// clicking at a sketch that says nothing back, which is the way a tool-shaped
    /// constraint goes wrong. Refusing the pick and naming what is wanted is the whole
    /// difference between a tool and a guessing game.
    fn viable_picks(&self, kind: ConstraintKind, sel: &[EntityId]) -> bool {
        let entity = |id: &EntityId| self.sketch.entity(*id).map(|e| &e.entity);
        let count =
            |f: fn(&Entity) -> bool| sel.iter().filter(|id| entity(id).is_some_and(f)).count();
        let points = count(|e| matches!(e, Entity::Point { .. }));
        let lines = count(|e| matches!(e, Entity::Line { .. }));
        let rounds = count(|e| matches!(e, Entity::Circle { .. } | Entity::Arc { .. }));
        let n = sel.len();
        // Anything the tools cannot name at all (text) is never a viable pick.
        if points + lines + rounds != n {
            return false;
        }
        let curves = lines + rounds;
        match kind {
            ConstraintKind::Coincident => n <= 2 && points >= 1 && points + curves == n,
            ConstraintKind::Horizontal | ConstraintKind::Vertical => {
                lines == n || (points == n && n <= 2)
            }
            ConstraintKind::Parallel => lines == n,
            ConstraintKind::Perpendicular => lines == n && n <= 2,
            // Two lines cross or run parallel; tangency needs something curved.
            ConstraintKind::Tangent => curves == n && n <= 2 && (n < 2 || rounds >= 1),
            ConstraintKind::Equal => lines == n || rounds == n,
            ConstraintKind::Concentric => rounds == n,
            ConstraintKind::Midpoint => points <= 1 && lines <= 1 && points + lines == n,
            ConstraintKind::Symmetric => points <= 2 && lines <= 1 && points + lines == n,
            ConstraintKind::Fix => n <= 1 && points + lines == n,
        }
    }

    // --- Constraint tool ---------------------------------------------------------------

    /// A pick while a constraint tool is armed. Picks gather until they mean something,
    /// then the constraint goes on and the tool stays armed for the next one. Clicking a
    /// pick again drops it; clicking empty space starts over.
    fn constraint_click(&mut self, kind: ConstraintKind, pos: Vec2, tol: f64) {
        let Some(hit) = self.sketch.hit_test(pos, tol).into_iter().next() else {
            self.constraint_picks.clear();
            return;
        };
        if let Some(i) = self
            .constraint_picks
            .iter()
            .position(|id| *id == hit.entity)
        {
            self.constraint_picks.remove(i);
            return;
        }
        let mut picks = self.constraint_picks.clone();
        picks.push(hit.entity);
        if !self.viable_picks(kind, &picks) {
            // The earlier picks stand: one wrong click should cost one click, not all of
            // them.
            self.constraint_error = Some(format!("{} needs: {}", kind.name(), kind.hint()));
            return;
        }
        self.constraint_picks = picks;
        if let Err(e) = self.apply_constraint_picks(kind) {
            self.constraint_error = Some(e);
            self.constraint_picks.clear();
        }
    }

    /// The reason the constraint tool refused the last pick, for the editor to report.
    pub fn take_constraint_error(&mut self) -> Option<String> {
        self.constraint_error.take()
    }

    #[cfg(test)]
    pub fn add_constraint(&mut self, c: Constraint) -> Result<(), String> {
        self.add_constraints(vec![c])
    }

    /// Adds several constraints as one undoable change, so a command that means two
    /// constraints (equal across three lines, a line pinned by both its ends) is one
    /// step of undo rather than two.
    pub fn add_constraints(&mut self, constraints: Vec<Constraint>) -> Result<(), String> {
        self.checkpoint();
        for c in constraints {
            if let Err(e) = self.sketch.add_constraint(c) {
                // Nothing partial is left behind: the sketch goes back to the checkpoint.
                if let Some(prev) = self.undo.pop() {
                    self.sketch = prev;
                }
                return Err(e.to_string());
            }
        }
        self.after_change();
        Ok(())
    }

    /// What `kind` means for `sel`, or nothing when the picks do not support it yet.
    ///
    /// Order never matters: a point and a line are the same command whichever was picked
    /// first, because the constraint itself knows which is which. Where a constraint is
    /// transitive — equal, parallel, concentric — more than two picks chain, so
    /// "these five holes are all the same size" is one command.
    pub fn constraints_for(&self, kind: ConstraintKind, sel: &[EntityId]) -> Vec<Constraint> {
        let entity = |id: &EntityId| self.sketch.entity(*id).map(|e| &e.entity);
        let is = |f: fn(&Entity) -> bool| move |id: &EntityId| entity(id).is_some_and(&f);
        let point = is(|e| matches!(e, Entity::Point { .. }));
        let line = is(|e| matches!(e, Entity::Line { .. }));
        let round = is(|e| matches!(e, Entity::Circle { .. } | Entity::Arc { .. }));
        let curve = is(|e| e.is_curve());
        let points: Vec<EntityId> = sel.iter().copied().filter(point).collect();
        let lines: Vec<EntityId> = sel.iter().copied().filter(line).collect();
        let rounds: Vec<EntityId> = sel.iter().copied().filter(round).collect();
        // A transitive constraint over a run of entities: each one tied to the one before.
        let chain = |ids: &[EntityId], make: fn(EntityId, EntityId) -> Constraint| {
            ids.windows(2).map(|w| make(w[0], w[1])).collect::<Vec<_>>()
        };
        match kind {
            ConstraintKind::Coincident => match (points.len(), sel.len()) {
                // A point and anything it can sit on. Two points merge; a point and a
                // curve means "on the curve", which is what the solver reads it as.
                (1 | 2, 2) => {
                    let p = points[0];
                    let target = *sel.iter().find(|id| **id != p).expect("two picks");
                    if point(&target) || curve(&target) {
                        vec![Constraint::Coincident { point: p, target }]
                    } else {
                        Vec::new()
                    }
                }
                _ => Vec::new(),
            },
            ConstraintKind::Horizontal | ConstraintKind::Vertical => {
                let horizontal = kind == ConstraintKind::Horizontal;
                if !lines.is_empty() && lines.len() == sel.len() {
                    return lines
                        .iter()
                        .map(|l| {
                            if horizontal {
                                Constraint::Horizontal(*l)
                            } else {
                                Constraint::Vertical(*l)
                            }
                        })
                        .collect();
                }
                // Two loose points are levelled or stacked by a zero offset along the
                // axis, which is the only way to say it without a line between them.
                match points[..] {
                    [a, b] if sel.len() == 2 && horizontal => {
                        vec![Constraint::HorizontalDistance { a, b, value: 0.0 }]
                    }
                    [a, b] if sel.len() == 2 => {
                        vec![Constraint::VerticalDistance { a, b, value: 0.0 }]
                    }
                    _ => Vec::new(),
                }
            }
            ConstraintKind::Parallel if lines.len() >= 2 && lines.len() == sel.len() => {
                chain(&lines, Constraint::Parallel)
            }
            ConstraintKind::Perpendicular if lines.len() == 2 && sel.len() == 2 => {
                vec![Constraint::Perpendicular(lines[0], lines[1])]
            }
            // Tangency needs something curved to be tangent to; two lines are parallel
            // or crossing, never tangent.
            ConstraintKind::Tangent if sel.len() == 2 && !rounds.is_empty() => {
                if sel.iter().all(curve) {
                    vec![Constraint::Tangent(sel[0], sel[1])]
                } else {
                    Vec::new()
                }
            }
            ConstraintKind::Equal if lines.len() >= 2 && lines.len() == sel.len() => {
                chain(&lines, Constraint::Equal)
            }
            ConstraintKind::Equal if rounds.len() >= 2 && rounds.len() == sel.len() => {
                chain(&rounds, Constraint::Equal)
            }
            ConstraintKind::Concentric if rounds.len() >= 2 && rounds.len() == sel.len() => {
                chain(&rounds, Constraint::Concentric)
            }
            ConstraintKind::Midpoint if points.len() == 1 && lines.len() == 1 && sel.len() == 2 => {
                vec![Constraint::Midpoint {
                    point: points[0],
                    line: lines[0],
                }]
            }
            ConstraintKind::Symmetric
                if points.len() == 2 && lines.len() == 1 && sel.len() == 3 =>
            {
                vec![Constraint::Symmetric {
                    a: points[0],
                    b: points[1],
                    axis: lines[0],
                }]
            }
            // A line is pinned by pinning both its ends: the solver only knows how to
            // fix a point, and a line with both ends fixed is a fixed line.
            ConstraintKind::Fix if sel.len() == 1 => match entity(&sel[0]) {
                Some(Entity::Point { .. }) => vec![Constraint::Fix(sel[0])],
                Some(Entity::Line { start, end }) => {
                    vec![Constraint::Fix(*start), Constraint::Fix(*end)]
                }
                _ => Vec::new(),
            },
            _ => Vec::new(),
        }
    }

    // --- Drawing ----------------------------------------------------------------------

    /// Plane position the entry boxes hang off: the last click, or the open end of a
    /// line chain. `None` when nothing is being drawn.
    pub fn entry_anchor(&self) -> Option<Vec3> {
        if let Some(pivot) = self.move_pivot() {
            return Some(self.frame.to_world(pivot));
        }
        // Beside the drag handle, because the box and the handle are the same number
        // said two ways and reading one while dragging the other is the whole point.
        if let Some((anchor, dir)) = self.offset_handle() {
            return Some(self.frame.to_world(anchor + dir * self.offset.distance));
        }
        if let Some((anchor, dir)) = self.fillet_handle() {
            return Some(self.frame.to_world(anchor + dir * self.fillet.radius));
        }
        let pos = match self.tool {
            SketchTool::Line => self.chain_end.and_then(|id| self.sketch.point_pos(id)),
            _ => self.clicks.last().map(|c| c.pos),
        }?;
        Some(self.frame.to_world(pos))
    }

    /// The constraints the last solve could not satisfy, worst first. Empty unless the
    /// sketch is conflicting.
    pub fn conflicting(&self) -> &[ConstraintId] {
        match &self.report {
            Some(Err(SolveError::DidNotConverge { conflicting, .. })) => conflicting,
            _ => &[],
        }
    }

    /// Everything the constraints leave free to move, curves included.
    ///
    /// The solver names the points and circles that own a loose parameter; a curve is
    /// loose when a point that defines it is, and the curve is what the user sees and
    /// clicks. Drawing these differently is the whole early warning: a sketch that is
    /// still free to move looks identical to a finished one until a dimension change
    /// drags it somewhere unintended.
    pub fn under_constrained(&self) -> std::collections::HashSet<EntityId> {
        let Some(Ok(report)) = &self.report else {
            return std::collections::HashSet::new();
        };
        let mut out: std::collections::HashSet<EntityId> =
            report.under_constrained.iter().copied().collect();
        let curves: Vec<EntityId> = self
            .sketch
            .entities()
            .filter(|(id, _)| {
                !out.contains(id)
                    && self
                        .sketch
                        .entity_points(*id)
                        .iter()
                        .any(|p| out.contains(p))
            })
            .map(|(id, _)| id)
            .collect();
        out.extend(curves);
        out
    }

    /// Triangles of one closed region, for filling it. Reuses the kernel's
    /// triangulation so a sketch region lights up exactly as the same region does in
    /// model mode; a region the triangulator cannot handle simply yields nothing.
    fn region_fill(&self, index: usize) -> Vec<[Vec3; 3]> {
        match self.profiles.get(index) {
            Some(profile) => basset_core::convert_profile(&self.frame, profile).triangles(),
            None => Vec::new(),
        }
    }

    /// Line, point and triangle batches for the sketch overlay.
    pub fn draw(
        &self,
        lines: &mut Vec<LineBatch>,
        points: &mut Vec<PointBatch>,
        tris: &mut Vec<TriBatch>,
    ) {
        let to3 = |p: Vec2| self.frame.to_world(p);
        // Filled regions go down first so the geometry and the badges stay readable on
        // top of them.
        let mut selected_fill = TriBatch::new(REGION_SELECT_FILL);
        for (index, _) in self.profiles.iter().enumerate().filter(|(_, p)| {
            self.selected_regions
                .iter()
                .any(|sample| p.contains(*sample))
        }) {
            selected_fill.triangles.extend(self.region_fill(index));
        }
        let mut hover_fill = TriBatch::new(REGION_HOVER_FILL);
        if let Some(index) = self.hover_region {
            hover_fill.triangles.extend(self.region_fill(index));
        }
        tris.extend([selected_fill, hover_fill]);
        let mut normal = LineBatch::new([0.92, 0.92, 0.95, 1.0]);
        normal.depth_test = false;
        let mut construction = LineBatch::new([0.75, 0.75, 0.55, 1.0]);
        construction.dashed = true;
        construction.depth_test = false;
        // Periwinkle for anything still free to move, after Fusion's blue: not the
        // saturated blue of the selection, which is also drawn three pixels wide.
        let mut loose = LineBatch::new(LOOSE_COLOR);
        loose.depth_test = false;
        let mut loose_construction = LineBatch::new(LOOSE_COLOR);
        loose_construction.dashed = true;
        loose_construction.depth_test = false;
        let mut selected = LineBatch::new([0.25, 0.6, 1.0, 1.0]);
        selected.width_px = 3.0;
        selected.depth_test = false;
        let mut hovered = LineBatch::new([1.0, 0.85, 0.3, 1.0]);
        hovered.width_px = 2.5;
        hovered.depth_test = false;
        let mut pts = PointBatch::new([0.9, 0.9, 0.9, 1.0]);
        let mut loose_pts = PointBatch::new(LOOSE_COLOR);
        let mut sel_pts = PointBatch::new([0.25, 0.6, 1.0, 1.0]);
        sel_pts.size_px = 8.0;
        let free = self.under_constrained();
        // What a pattern or an offset has made is provisional until OK, so it is drawn
        // as a preview rather than as geometry the user has already committed to.
        let copies = self.previewed();
        let mut preview = LineBatch::new([0.55, 0.85, 1.0, 0.85]);
        preview.depth_test = false;

        let region_curves = self
            .hover_region
            .map(|r| self.region_curves(r))
            .unwrap_or_default();
        // Hovering a badge lights its geometry, exactly as hovering a row of the
        // palette's constraint list does.
        let held_by_badge = self.constraint_hover_entities();
        for (id, data) in self.sketch.entities() {
            let is_selected = self.selected.contains(&id);
            let is_hovered = self.hover == Some(id)
                || self.dim_first == Some(id)
                || region_curves.contains(&id)
                || self.constraint_picks.contains(&id)
                || self.fillet_pick.is_some_and(|(pick, _)| pick == id)
                || self.highlighted.contains(&id)
                || held_by_badge.contains(&id);
            if let Entity::Point { pos } = data.entity {
                if is_selected || is_hovered {
                    sel_pts.points.push(to3(pos));
                } else if free.contains(&id) {
                    loose_pts.points.push(to3(pos));
                } else {
                    pts.points.push(to3(pos));
                }
                continue;
            }
            let Some(polyline) = outline(&self.sketch, id, &self.tess) else {
                continue;
            };
            let batch = match (
                is_selected,
                is_hovered,
                data.construction,
                free.contains(&id),
            ) {
                _ if copies.contains(&id) => &mut preview,
                (true, ..) => &mut selected,
                (_, true, ..) => &mut hovered,
                (.., true, true) => &mut loose_construction,
                (.., true, false) => &mut construction,
                (.., false, true) => &mut loose,
                _ => &mut normal,
            };
            for w in polyline.windows(2) {
                batch.segments.push([to3(w[0]), to3(w[1])]);
            }
            // Text outlines when a font is available.
            if let Entity::Text {
                anchor,
                text,
                height,
                angle,
            } = &data.entity
                && let Some(font) = self.sketch.font()
                && let Some(origin) = self.sketch.point_pos(*anchor)
            {
                for outline in font.text_outlines(text, *height, *angle, origin, &self.tess) {
                    for i in 0..outline.len() {
                        batch
                            .segments
                            .push([to3(outline[i]), to3(outline[(i + 1) % outline.len()])]);
                    }
                }
            }
        }
        if let Some(piece) = &self.trim_preview {
            // Red, wide and on top: the user is about to lose this, and seeing which
            // piece before clicking is most of what makes trim usable.
            let mut doomed = LineBatch::new([1.0, 0.35, 0.3, 1.0]);
            doomed.width_px = 4.0;
            doomed.depth_test = false;
            for w in piece.windows(2) {
                doomed.segments.push([to3(w[0]), to3(w[1])]);
            }
            lines.push(doomed);
        }
        // A circular pattern turns about a point the user cannot otherwise see, and two
        // numbers in a palette are not a position. It is drawn as the pivot mark a
        // drawing uses.
        if self.pattern_op.is_some() && self.pattern.circular {
            let c = self.pattern.center;
            let r = self.cursor_px * 7.0;
            let mut pivot = LineBatch::new([1.0, 0.75, 0.35, 0.95]);
            pivot.width_px = 2.0;
            pivot.depth_test = false;
            for axis in [Vec2::new(1.0, 0.0), Vec2::new(0.0, 1.0)] {
                pivot
                    .segments
                    .push([to3(c - axis * r * 1.8), to3(c + axis * r * 1.8)]);
            }
            const STEPS: usize = 16;
            let ring = |i: usize| {
                c + Vec2::from_angle(std::f64::consts::TAU * i as f64 / STEPS as f64) * r
            };
            for i in 0..STEPS {
                pivot.segments.push([to3(ring(i)), to3(ring(i + 1))]);
            }
            lines.push(pivot);
        }
        if let Some(m) = self.marquee {
            let mut band = LineBatch::new([0.55, 0.8, 1.0, 0.9]);
            band.depth_test = false;
            // Crossing bands are dashed, the way the drawing conventions of every CAD tool
            // distinguish "touches" from "encloses".
            band.dashed = m.crossing();
            let corners = m.corners();
            for i in 0..4 {
                band.segments
                    .push([to3(corners[i]), to3(corners[(i + 1) % 4])]);
            }
            lines.push(band);
        }
        if let Some(cursor) = self.cursor {
            let mut preview = LineBatch::new([0.6, 0.9, 1.0, 0.9]);
            preview.depth_test = false;
            // The preview is dashed when the mode is on, so the decision is visible
            // while the shape is still being aimed rather than only after it lands.
            preview.dashed = self.construction;
            if let Some(start) = self.chain_end.and_then(|id| self.sketch.point_pos(id)) {
                preview.segments.push([to3(start), to3(cursor)]);
            }
            self.shape_preview(cursor, &mut preview);
            if !preview.segments.is_empty() {
                lines.push(preview);
            }
        }
        let mut dims = LineBatch::new([0.55, 0.75, 0.95, 0.9]);
        dims.depth_test = false;
        // Red for the ones the solver could not satisfy: the palette names them, but the
        // answer to "which constraint is fighting?" belongs on the drawing.
        let mut conflicting = LineBatch::new(CONFLICT_COLOR);
        conflicting.width_px = 2.0;
        conflicting.depth_test = false;
        let in_conflict = self.conflicting().to_vec();
        for g in self.dimension_graphics() {
            let batch = if in_conflict.contains(&g.id) {
                &mut conflicting
            } else {
                &mut dims
            };
            batch.segments.extend(g.segments);
        }
        // Amber, so a constraint mark is never mistaken for a dimension or for geometry.
        let mut glyphs = LineBatch::new([0.95, 0.72, 0.30, 0.95]);
        glyphs.depth_test = false;
        // A badge the pointer is on, or one belonging to geometry that is lit up, is
        // drawn in the hover colour and a little wider. That is the other half of the
        // link the palette's constraint list gives: from the drawing to the mark, as
        // well as from the list to the drawing.
        let mut lit_glyphs = LineBatch::new([1.0, 0.85, 0.3, 1.0]);
        lit_glyphs.width_px = 2.5;
        lit_glyphs.depth_test = false;
        for g in self.constraint_glyphs() {
            let lit = self.hovered_constraint == Some(g.id)
                || self.glyph_entity(&g).is_some_and(|id| {
                    self.hover == Some(id)
                        || self.selected.contains(&id)
                        || self.highlighted.contains(&id)
                });
            let batch = match (in_conflict.contains(&g.id), lit) {
                (true, _) => &mut conflicting,
                (_, true) => &mut lit_glyphs,
                _ => &mut glyphs,
            };
            batch.segments.extend(g.segments);
            batch.segments.extend(g.leader);
        }
        lines.extend([
            normal,
            construction,
            loose,
            loose_construction,
            preview,
            hovered,
            selected,
            dims,
            glyphs,
            lit_glyphs,
            conflicting,
        ]);
        points.extend([pts, loose_pts, sel_pts]);
        // Crosshair on the snapped position, so the user aims at where the point will
        // actually land rather than at the pointer, which the grid snap can pull away
        // from by half a step. Amber when it will reuse an existing point.
        if let Some(cursor) = self.cursor
            && (self.tool != SketchTool::Select || self.picking_pattern_center())
        {
            let color = if self.cursor_snapped {
                [1.0, 0.85, 0.3, 1.0]
            } else {
                [0.6, 0.9, 1.0, 0.9]
            };
            let mut cross = LineBatch::new(color);
            cross.depth_test = false;
            let arm = self.cursor_px * CURSOR_ARM_PX;
            let gap = self.cursor_px * CURSOR_GAP_PX;
            for axis in [Vec2::new(1.0, 0.0), Vec2::new(0.0, 1.0)] {
                for dir in [1.0, -1.0] {
                    let d = axis * dir;
                    cross
                        .segments
                        .push([to3(cursor + d * gap), to3(cursor + d * arm)]);
                }
            }
            lines.push(cross);
            let mut marker = PointBatch::new(color);
            marker.size_px = if self.cursor_snapped { 9.0 } else { 5.0 };
            marker.points.push(to3(cursor));
            points.push(marker);
        }
        self.snap_feedback(lines);
    }

    /// What the pointer snapped to, and why: a glyph naming the kind, drawn on the
    /// snapped point, and the guide lines that caught it drawn dashed back to the
    /// geometry they come from.
    ///
    /// Both are needed. The glyph alone says "something held this" without saying what,
    /// and an alignment with a corner off the other side of the screen is unreadable
    /// without the line joining the two.
    fn snap_feedback(&self, lines: &mut Vec<LineBatch>) {
        let Some(found) = self.inference.current() else {
            return;
        };
        // Only while the drawing tools are aiming: with Select the pointer is picking
        // what is already there, and a marker on it would be a promise of a click that
        // places nothing.
        if self.cursor.is_none()
            || (self.tool == SketchTool::Select && !self.picking_pattern_center())
        {
            return;
        }
        let to3 = |p: Vec2| self.frame.to_world(p);
        let mut glyph = LineBatch::new(SNAP_MARKER_COLOR);
        glyph.width_px = 1.8;
        glyph.depth_test = false;
        for seg in snap::marker(found.kind, found.at, self.cursor_px) {
            glyph.segments.push([to3(seg[0]), to3(seg[1])]);
        }
        let mut guides = LineBatch::new(SNAP_GUIDE_COLOR);
        guides.width_px = 1.0;
        guides.depth_test = false;
        for guide in found.guides.iter().flatten() {
            for seg in snap::guide_dashes(*guide, found.at, self.cursor_px) {
                guides.segments.push([to3(seg[0]), to3(seg[1])]);
            }
        }
        lines.extend([glyph, guides]);
    }

    /// Rubber band of the shape the next click would make. It is built for real in a
    /// scratch copy of the sketch, so every tool previews exactly what it will create,
    /// typed sizes included. Tools still short of their last click show a line from the
    /// first click instead.
    fn shape_preview(&self, cursor: Vec2, preview: &mut LineBatch) {
        let to3 = |p: Vec2| self.frame.to_world(p);
        let needed = self.tool.clicks();
        if needed == 0 {
            return;
        }
        if self.clicks.len() + 1 < needed {
            if let Some(first) = self.clicks.first().map(|c| c.pos) {
                preview.segments.push([to3(first), to3(cursor)]);
            }
            return;
        }
        let mut clicks = self.clicks.clone();
        clicks.push(Click {
            pos: cursor,
            snapped: None,
            on_curve: None,
        });
        let mut scratch = self.sketch.clone();
        if build_shape(&mut scratch, self.tool, &clicks, &self.params()).is_err() {
            return;
        }
        for (id, _) in scratch.entities() {
            if self.sketch.entity(id).is_some() {
                continue;
            }
            if let Some(polyline) = outline(&scratch, id, &self.tess) {
                for w in polyline.windows(2) {
                    preview.segments.push([to3(w[0]), to3(w[1])]);
                }
            }
        }
    }

    /// Moves a dimension's value text to where the pointer is on the sketch plane. The
    /// lines are re-derived from the text position, so dragging the text is how the
    /// user places the whole dimension.
    pub fn move_label(&mut self, id: ConstraintId, ray: &Ray) {
        if let Some(pos) = self.to_plane(ray) {
            // A dimension's value is placed in the drawing like anything else the user
            // puts there, so it lands on the grid and shift lets go of it. Dragged
            // freehand, two dimensions of the same feature never line up with each other.
            let snapped = self.to_grid(pos);
            let _ = self.sketch.set_dimension_label(id, snapped);
            self.dirty = true;
        }
    }

    /// Every dimension as drawn: value text position, text, and the lines around it.
    pub fn dimension_graphics(&self) -> Vec<DimGraphic> {
        let px = self.cursor_px;
        let mut out = Vec::new();
        for (cid, c) in self.sketch.constraints() {
            if let Some(g) = self.dimension_graphic(cid, c, px) {
                out.push(g);
            }
        }
        out
    }

    fn dimension_graphic(&self, cid: ConstraintId, c: &Constraint, px: f64) -> Option<DimGraphic> {
        let pos = |id: EntityId| self.sketch.point_pos(id);
        let placed = self.sketch.dimension_label(cid);
        let gap = LABEL_GAP_PX * px;
        let arrow = ARROW_PX * px;
        let to3 = |p: Vec2| self.frame.to_world(p);
        let mut segments: Vec<[Vec2; 2]> = Vec::new();
        let label = match c {
            Constraint::Distance { a, b, .. } => {
                match (pos(*a), pos(*b)) {
                    // Point to point: extension lines out to the dimension line, which
                    // runs parallel to the pair at the label's offset.
                    (Some(pa), Some(pb)) => {
                        let dir = (pb - pa).normalize_or(Vec2::X);
                        let n = dir.perp();
                        let label = placed.unwrap_or((pa + pb) * 0.5 + n * gap);
                        let off = (label - pa).dot(n);
                        distance_lines(&mut segments, pa, pb, n, off, arrow);
                        label
                    }
                    // Point to line: the dimension line is perpendicular to the line,
                    // from the point to its foot, slid along the line to the label.
                    (Some(p), None) | (None, Some(p)) => {
                        let line = if pos(*a).is_some() { *b } else { *a };
                        let (la, lb) = self.sketch.curve_endpoints(line)?;
                        let dir = (lb - la).normalize_or(Vec2::X);
                        let foot = la + dir * dir.dot(p - la);
                        let label = placed.unwrap_or((p + foot) * 0.5 + dir * gap);
                        let slide = (label - p).dot(dir);
                        distance_lines(&mut segments, p, foot, dir, slide, arrow);
                        label
                    }
                    _ => return None,
                }
            }
            Constraint::HorizontalDistance { a, b, .. }
            | Constraint::VerticalDistance { a, b, .. } => {
                let (pa, pb) = (pos(*a)?, pos(*b)?);
                let horizontal = matches!(c, Constraint::HorizontalDistance { .. });
                // The dimension line runs along the measured axis; the extension lines
                // along the other one.
                let n = if horizontal { Vec2::Y } else { Vec2::X };
                let label = placed.unwrap_or((pa + pb) * 0.5 + n * gap);
                let off = (label - pa).dot(n);
                let (qa, qb) = (pa + n * off, pb + n * off);
                segments.push([pa, qa]);
                segments.push([pb, qb]);
                segments.push([qa, qb]);
                arrowheads(&mut segments, qa, qb, arrow);
                label
            }
            Constraint::Radius { curve, .. } | Constraint::Diameter { curve, .. } => {
                let (center, radius) = match self.sketch.entity(*curve).map(|e| &e.entity)? {
                    Entity::Circle { center, radius } => (pos(*center)?, *radius),
                    Entity::Arc { center, start, .. } => {
                        let c = pos(*center)?;
                        (c, c.distance(pos(*start)?))
                    }
                    _ => return None,
                };
                let default_dir = Vec2::from_angle(std::f64::consts::FRAC_PI_4);
                let label = placed.unwrap_or(center + default_dir * (radius + gap));
                let dir = (label - center).normalize_or(default_dir);
                let rim = center + dir * radius;
                // A leader from the rim to the text, and for a diameter the line right
                // across the circle with an arrowhead at each end.
                if matches!(c, Constraint::Diameter { .. }) {
                    let far = center - dir * radius;
                    segments.push([far, rim]);
                    arrowheads(&mut segments, far, rim, arrow);
                } else {
                    segments.push([center, rim]);
                    arrowhead(&mut segments, rim, dir, arrow);
                }
                segments.push([rim, label]);
                label
            }
            Constraint::Angle { a, b, .. } => {
                let (a0, a1) = self.sketch.curve_endpoints(*a)?;
                let (b0, b1) = self.sketch.curve_endpoints(*b)?;
                let apex = line_intersection(a0, a1, b0, b1)?;
                // Each line is measured along whichever of its directions leaves the
                // apex toward the line itself, so the arc spans the angle the user sees
                // between the drawn stretches.
                let da = ((a0 + a1) * 0.5 - apex).normalize_or(a1 - a0);
                let db = ((b0 + b1) * 0.5 - apex).normalize_or(b1 - b0);
                let bisector = (da + db).normalize_or(da.perp());
                let label = placed.unwrap_or(apex + bisector * gap * 1.5);
                let r = label.distance(apex).max(px);
                let sweep = da.angle_to(db);
                let steps = ((sweep.abs() / 0.15).ceil() as usize).max(2);
                let start = da.to_angle();
                let mut prev = apex + da * r;
                for i in 1..=steps {
                    let next = apex + Vec2::from_angle(start + sweep * i as f64 / steps as f64) * r;
                    segments.push([prev, next]);
                    prev = next;
                }
                // Extension lines reach out to the arc when it lies beyond the lines.
                for (p0, p1, d) in [(a0, a1, da), (b0, b1, db)] {
                    let reach = (p0 - apex).dot(d).max((p1 - apex).dot(d));
                    if reach < r {
                        segments.push([apex + d * reach, apex + d * r]);
                    }
                }
                label
            }
            _ => return None,
        };
        Some(DimGraphic {
            id: cid,
            label: to3(label),
            // A driven dimension is marked the way a spreadsheet marks a formula cell:
            // the number is still what matters, but the user must be able to see that
            // retyping it will replace an expression.
            text: match self.sketch.dimension_expr(cid) {
                Some(_) => format!("ƒ {}", format_value(c)),
                None => format_value(c),
            },
            segments: segments.iter().map(|[a, b]| [to3(*a), to3(*b)]).collect(),
        })
    }

    /// A badge for every geometric constraint, placed beside the geometry it holds.
    ///
    /// The placement is a cached, decluttered layout: see [`Self::lay_out_glyphs`]. This
    /// only turns it into world coordinates at the current zoom, which is why a badge
    /// keeps its size on screen exactly while its slot changes only when the view does.
    pub fn constraint_glyphs(&self) -> Vec<ConstraintGlyph> {
        let key = self.glyph_key();
        {
            let mut cache = self.glyph_cache.borrow_mut();
            if cache.key != Some(key) {
                // The previous placements go back in so a badge that is still free to
                // stay where it was does. Re-packing optimally on every change reads as
                // jitter, and a badge the eye has already found moving is worse than a
                // badge in a slightly worse slot.
                let placements = self.lay_out_glyphs(&cache.placements);
                cache.placements = placements;
                cache.key = Some(key);
                cache.builds += 1;
            }
        }
        let cache = self.glyph_cache.borrow();
        let px = self.glyph_scale();
        let size = px * GLYPH_PX;
        let to3 = |p: Vec2| self.frame.to_world(p);
        cache
            .placements
            .iter()
            .filter_map(|p| {
                let strokes = glyph_strokes(self.sketch.constraint(p.id)?);
                let center = p.base + p.dir * (p.dist_px * px);
                let place = |q: Vec2| center + p.u * (q.x * size) + p.v * (q.y * size);
                Some(ConstraintGlyph {
                    id: p.id,
                    target: p.target,
                    center: to3(center),
                    segments: strokes
                        .iter()
                        .map(|[a, b]| [to3(place(*a)), to3(place(*b))])
                        .collect(),
                    // A badge that had to move away says which entity it came from; one
                    // sitting in its natural slot needs no line, and drawing one anyway
                    // would double the ink on a tidy sketch.
                    leader: (p.dist_px > GLYPH_LEADER_PX).then(|| {
                        let start = p.base + p.dir * (GLYPH_GAP_PX * 0.3 * px);
                        let end = center - p.dir * (GLYPH_BOX_PX * px);
                        [to3(start), to3(end)]
                    }),
                })
            })
            .collect()
    }

    /// How many times the badge layout has been rebuilt. The overlay asks for the badges
    /// twice a frame, so this is what a test asserts the caching on: the number a user
    /// would feel is rebuilds per second, not calls. Test-only: nothing the application
    /// does should depend on how often a cache happened to miss.
    #[cfg(test)]
    pub fn glyph_layouts(&self) -> u64 {
        self.glyph_cache.borrow().builds
    }

    /// World size of one pixel for the badges. Falls back to millimetres when no pointer
    /// has moved yet, so a sketch built by a test still lays its badges out sensibly.
    fn glyph_scale(&self) -> f64 {
        if self.view_px.is_finite() && self.view_px > 0.0 {
            self.view_px
        } else {
            1.0
        }
    }

    /// Everything the badge layout reads, hashed.
    ///
    /// A revision counter would be cheaper still but would go stale: `sketch` is a public
    /// field and the panels, the tools and the tests all write to it directly. Hashing a
    /// few hundred point positions is a rounding error next to the layout it saves, and
    /// it cannot be wrong.
    fn glyph_key(&self) -> u64 {
        use std::hash::{Hash, Hasher};
        let mut h = std::collections::hash_map::DefaultHasher::new();
        // The scale is quantised, so panning and zooming rescale the whole overlay
        // rather than re-deciding it. See `GLYPH_SCALE_STEPS`.
        ((self.glyph_scale().ln() * GLYPH_SCALE_STEPS).round() as i64).hash(&mut h);
        for (id, data) in self.sketch.entities() {
            id.hash(&mut h);
            match data.entity {
                Entity::Point { pos } => (pos.x.to_bits(), pos.y.to_bits()).hash(&mut h),
                Entity::Circle { radius, .. } => radius.to_bits().hash(&mut h),
                _ => {}
            }
        }
        for (cid, c) in self.sketch.constraints() {
            cid.hash(&mut h);
            c.references().hash(&mut h);
            glyph_kind(c)
                .map(|k| std::mem::discriminant(&k))
                .hash(&mut h);
            // A dimension's value box is an obstacle, so dragging one re-lays the badges.
            self.sketch
                .dimension_label(cid)
                .map(|p| (p.x.to_bits(), p.y.to_bits()))
                .hash(&mut h);
        }
        // The solve report only changes the colours, but a conflicting badge is drawn
        // wider and the palette may have just deleted what it blamed.
        self.conflicting().hash(&mut h);
        h.finish()
    }

    /// Decides where every badge goes.
    ///
    /// Each badge has a natural anchor on its entity — the midpoint of a line, the rim of
    /// a circle — and a ranked list of slots around it: successive rings outwards, each
    /// tried in eight directions, ordered by distance plus a penalty for turning away
    /// from the entity's own normal. The first slot that collides with neither the
    /// drawing, nor a dimension's value box, nor a badge already placed wins, and the
    /// slot a badge held in `previous` is offered to it first so a layout recomputed
    /// after an edit leaves everything it can where the user last saw it.
    ///
    /// Greedy and in the sketch's own constraint order rather than optimal: the packing
    /// only has to be legible, and an optimal packing that re-shuffles when a ninth
    /// constraint is added is worse to use than a greedy one that appends to it.
    fn lay_out_glyphs(&self, previous: &[GlyphPlacement]) -> Vec<GlyphPlacement> {
        let px = self.glyph_scale();
        let mut space = GlyphSpace::new(self, px);
        let mut out = Vec::new();
        for (cid, c) in self.sketch.constraints() {
            if glyph_strokes(c).is_empty() {
                continue;
            }
            // Horizontal and Vertical are statements about the axes, so their marks keep
            // the axes' orientation; every other badge lines up with its entity.
            let axis_aligned = matches!(c, Constraint::Horizontal(_) | Constraint::Vertical(_));
            for (target, id) in glyph_targets(c).into_iter().enumerate() {
                let Some((base, outward, along)) = self.glyph_anchor(id) else {
                    continue;
                };
                let (u, v) = if axis_aligned {
                    (Vec2::new(1.0, 0.0), Vec2::new(0.0, 1.0))
                } else {
                    (along, outward)
                };
                let candidates = glyph_candidates(outward);
                let anchor = base / px;
                let sticky = previous
                    .iter()
                    .find(|p| p.id == cid && p.target == target)
                    .map(|p| p.slot);
                let chosen = sticky
                    .into_iter()
                    .chain(0..candidates.len())
                    .find(|index| match candidates.get(*index) {
                        Some((dir, dist)) => space.free(anchor + *dir * *dist),
                        None => false,
                    })
                    // Nothing was clear anywhere: the sketch is denser than the screen
                    // can show. The badge still goes down, in its natural slot, because
                    // one that is not drawn can never be hovered, named or deleted.
                    .unwrap_or(0);
                let (dir, dist) = candidates[chosen];
                space.claim(anchor + dir * dist);
                out.push(GlyphPlacement {
                    id: cid,
                    target,
                    slot: chosen,
                    base,
                    dir,
                    dist_px: dist,
                    u,
                    v,
                });
            }
        }
        out
    }

    /// The badge under a point on the sketch plane, if any. Badges are a constant size on
    /// screen, so the reach is in pixels too: at a small zoom the geometry crowds but the
    /// badges do not, and a reach in millimetres would swallow the whole drawing.
    fn constraint_at(&self, pos: Vec2) -> Option<ConstraintId> {
        let reach = self.glyph_scale() * GLYPH_HIT_PX;
        self.constraint_glyphs()
            .into_iter()
            .filter_map(|g| {
                let d = self.frame.to_local(g.center).distance(pos);
                (d <= reach).then_some((d, g.id))
            })
            .min_by(|a, b| a.0.total_cmp(&b.0))
            .map(|(_, id)| id)
    }

    /// The geometry the badge under the pointer holds, lit up while it is. The palette's
    /// constraint list already highlights from a row; this is the same link read from the
    /// drawing instead, and it uses the same `references()` the palette does so the two
    /// light up exactly the same entities.
    pub fn constraint_hover_entities(&self) -> Vec<EntityId> {
        self.hovered_constraint
            .and_then(|id| self.sketch.constraint(id))
            .map(|c| c.references())
            .unwrap_or_default()
    }

    /// The entity a badge sits on, for lighting the badge when its geometry is — and for
    /// a test to ask a badge which curve it was laid out against.
    pub fn glyph_entity(&self, g: &ConstraintGlyph) -> Option<EntityId> {
        let c = self.sketch.constraint(g.id)?;
        glyph_targets(c).get(g.target).copied()
    }

    /// Where a badge for a constraint on `id` sits: a point on the entity, the direction
    /// away from it, and the direction along it.
    fn glyph_anchor(&self, id: EntityId) -> Option<(Vec2, Vec2, Vec2)> {
        let x = Vec2::new(1.0, 0.0);
        match self.sketch.entity(id)?.entity {
            Entity::Point { pos } => {
                // Up and to the right, clear of the crosshair and of the point itself.
                let diagonal = std::f64::consts::FRAC_1_SQRT_2;
                Some((pos, Vec2::new(diagonal, diagonal), x))
            }
            Entity::Line { start, end } => {
                let (a, b) = (self.sketch.point_pos(start)?, self.sketch.point_pos(end)?);
                let along = (b - a).try_normalize()?;
                Some(((a + b) * 0.5, along.perp(), along))
            }
            Entity::Circle { center, radius } => {
                let c = self.sketch.point_pos(center)?;
                let radial = Vec2::new(0.0, 1.0);
                Some((c + radial * radius, radial, x))
            }
            Entity::Arc { center, start, end } => {
                let c = self.sketch.point_pos(center)?;
                let (a, b) = (self.sketch.point_pos(start)?, self.sketch.point_pos(end)?);
                let a0 = (a - c).to_angle();
                let sweep = ((b - c).to_angle() - a0).rem_euclid(std::f64::consts::TAU);
                let radial = Vec2::from_angle(a0 + sweep * 0.5);
                Some((c + radial * a.distance(c), radial, radial.perp()))
            }
            _ => None,
        }
    }

    pub fn saved_camera(&self) -> &Camera {
        &self.saved_camera
    }
}

// --- Shape building -----------------------------------------------------------------------

/// How far apart two badge centres must be to read as two marks, in pixels. Exposed to
/// the tests so they assert on the property the layout exists to keep rather than on a
/// number copied out of it.
#[cfg(test)]
pub fn glyph_clearance_px() -> f64 {
    GLYPH_BOX_PX * 2.0
}

/// The slots a badge may take, best first: rings outwards from the anchor, each tried in
/// eight directions, ordered by how far out they are plus what turning away from the
/// entity's own normal costs. Half a turn is worth about one ring, so a badge crosses to
/// the other side of its line before it walks a long way out along the near side.
fn glyph_candidates(outward: Vec2) -> Vec<(Vec2, f64)> {
    let mut out: Vec<(Vec2, f64, f64)> = Vec::with_capacity(GLYPH_RINGS * GLYPH_DIRS);
    for ring in 0..GLYPH_RINGS {
        let dist = GLYPH_GAP_PX + ring as f64 * GLYPH_STEP_PX;
        for step in 0..GLYPH_DIRS {
            // 0, -45, +45, -90, +90 … so the two sides of the entity are tried in step,
            // and the order is the same every time the layout runs.
            let eighths = step.div_ceil(2) as f64;
            let sign = if step.is_multiple_of(2) { 1.0 } else { -1.0 };
            let turn = sign * eighths * std::f64::consts::FRAC_PI_4;
            let dir = Vec2::from_angle(turn).rotate(outward);
            out.push((dir, dist, dist + turn.abs() * GLYPH_TURN_PX));
        }
    }
    // A stable sort, so slots of equal cost keep the ring-then-direction order above and
    // two runs of the layout on the same drawing agree exactly.
    out.sort_by(|a, b| a.2.total_cmp(&b.2));
    out.into_iter().map(|(dir, dist, _)| (dir, dist)).collect()
}

/// Which cell of the collision grid a point falls in. `as i32` saturates rather than
/// wrapping, so a coordinate the user has sent to infinity buckets absurdly instead of
/// aliasing onto somebody else's cell.
fn glyph_cell(p: Vec2) -> (i32, i32) {
    (
        (p.x / GLYPH_CELL_PX).floor() as i32,
        (p.y / GLYPH_CELL_PX).floor() as i32,
    )
}

/// Whether a segment touches an axis-aligned square, by Liang–Barsky clipping. A
/// degenerate segment is a point, and falls out of the same test.
fn segment_hits_box(a: Vec2, b: Vec2, center: Vec2, half: f64) -> bool {
    let d = b - a;
    let (mut t0, mut t1) = (0.0f64, 1.0f64);
    for (p, q) in [
        (-d.x, a.x - (center.x - half)),
        (d.x, (center.x + half) - a.x),
        (-d.y, a.y - (center.y - half)),
        (d.y, (center.y + half) - a.y),
    ] {
        if p == 0.0 {
            // Parallel to this edge: either wholly inside its slab or wholly outside.
            if q < 0.0 {
                return false;
            }
            continue;
        }
        let r = q / p;
        if p < 0.0 {
            if r > t1 {
                return false;
            }
            t0 = t0.max(r);
        } else {
            if r < t0 {
                return false;
            }
            t1 = t1.min(r);
        }
    }
    true
}

/// Where a badge may not go: the drawing it would cover, the dimension values already on
/// it, and the badges placed before it.
///
/// Everything is in pixels on the sketch plane, because that is the space the user reads
/// a collision in — two marks a millimetre apart are on top of each other zoomed out and
/// comfortably apart zoomed in, and only the second of those is a collision.
struct GlyphSpace {
    segments: Vec<[Vec2; 2]>,
    /// Segments of the drawing bucketed by cell. A few hundred badges against a few
    /// hundred curves is 1e5 pairs tested without this, every time the layout runs.
    cells: std::collections::HashMap<(i32, i32), Vec<usize>>,
    /// Centres of the boxes already claimed, bucketed the same way.
    taken: std::collections::HashMap<(i32, i32), Vec<Vec2>>,
}

impl GlyphSpace {
    fn new(s: &SketchEditor, px: f64) -> Self {
        let mut space = Self {
            segments: Vec::new(),
            cells: std::collections::HashMap::new(),
            taken: std::collections::HashMap::new(),
        };
        for (id, data) in s.sketch.entities() {
            // A point is drawn as a dot the badge should not sit on either, so it goes in
            // as a segment of no length.
            if let Entity::Point { pos } = data.entity {
                space.add_segment([pos / px, pos / px]);
                continue;
            }
            let Some(polyline) = outline(&s.sketch, id, &s.tess) else {
                continue;
            };
            for w in polyline.windows(2) {
                space.add_segment([w[0] / px, w[1] / px]);
            }
        }
        // A dimension's value box is drawn over the sketch too, and a badge landing on
        // one hides a number the user is trying to read.
        for g in s.dimension_graphics() {
            space.claim(s.frame.to_local(g.label) / px);
        }
        space
    }

    fn add_segment(&mut self, seg: [Vec2; 2]) {
        let index = self.segments.len();
        self.segments.push(seg);
        let len = seg[0].distance(seg[1]);
        if !len.is_finite() {
            return;
        }
        let steps = ((len / (GLYPH_CELL_PX * 0.5)).ceil() as usize).clamp(1, GLYPH_MAX_CELLS);
        for i in 0..=steps {
            let p = seg[0] + (seg[1] - seg[0]) * (i as f64 / steps as f64);
            let bucket = self.cells.entry(glyph_cell(p)).or_default();
            if bucket.last() != Some(&index) {
                bucket.push(index);
            }
        }
    }

    /// The cells a badge centred at `center` has to look in. One cell of margin, because
    /// a segment is sampled into cells rather than rasterised exactly and may cross the
    /// box while its nearest sample sits just outside.
    fn range(center: Vec2) -> ((i32, i32), (i32, i32)) {
        let margin = Vec2::splat(GLYPH_BOX_PX + GLYPH_CELL_PX);
        (glyph_cell(center - margin), glyph_cell(center + margin))
    }

    fn free(&self, center: Vec2) -> bool {
        let (lo, hi) = Self::range(center);
        for cx in lo.0..=hi.0 {
            for cy in lo.1..=hi.1 {
                if let Some(others) = self.taken.get(&(cx, cy))
                    && others.iter().any(|o| {
                        (o.x - center.x).abs() < GLYPH_BOX_PX * 2.0
                            && (o.y - center.y).abs() < GLYPH_BOX_PX * 2.0
                    })
                {
                    return false;
                }
                let Some(ids) = self.cells.get(&(cx, cy)) else {
                    continue;
                };
                if ids.iter().any(|i| {
                    let [a, b] = self.segments[*i];
                    segment_hits_box(a, b, center, GLYPH_BOX_PX)
                }) {
                    return false;
                }
            }
        }
        true
    }

    fn claim(&mut self, center: Vec2) {
        self.taken
            .entry(glyph_cell(center))
            .or_default()
            .push(center);
    }
}

/// The entities a constraint puts a badge on. A constraint between two curves marks
/// both, as a drawing does, so it is clear which pair it ties together.
fn glyph_targets(c: &Constraint) -> Vec<EntityId> {
    match c {
        Constraint::Horizontal(a) | Constraint::Vertical(a) | Constraint::Fix(a) => vec![*a],
        Constraint::Parallel(a, b)
        | Constraint::Perpendicular(a, b)
        | Constraint::Equal(a, b)
        | Constraint::Tangent(a, b)
        | Constraint::Concentric(a, b) => vec![*a, *b],
        Constraint::Coincident { point, .. } | Constraint::Midpoint { point, .. } => vec![*point],
        Constraint::Symmetric { a, b, .. } => vec![*a, *b],
        // Dimensions draw their own value, extension lines and arrowheads.
        _ => Vec::new(),
    }
}

/// A closed ring of `r`, for the badges drawn as circles.
fn glyph_ring(r: f64) -> Vec<[Vec2; 2]> {
    const STEPS: usize = 10;
    (0..STEPS)
        .map(|i| {
            let angle = |k: usize| std::f64::consts::TAU * k as f64 / STEPS as f64;
            [
                Vec2::from_angle(angle(i)) * r,
                Vec2::from_angle(angle(i + 1)) * r,
            ]
        })
        .collect()
}

/// Which constraint tool made `c`, or `None` for a dimension, which is the dimension
/// tool's work and draws its own value rather than a badge.
fn glyph_kind(c: &Constraint) -> Option<ConstraintKind> {
    Some(match c {
        Constraint::Horizontal(_) => ConstraintKind::Horizontal,
        Constraint::Vertical(_) => ConstraintKind::Vertical,
        Constraint::Parallel(..) => ConstraintKind::Parallel,
        Constraint::Perpendicular(..) => ConstraintKind::Perpendicular,
        Constraint::Equal(..) => ConstraintKind::Equal,
        Constraint::Tangent(..) => ConstraintKind::Tangent,
        Constraint::Concentric(..) => ConstraintKind::Concentric,
        Constraint::Coincident { .. } => ConstraintKind::Coincident,
        Constraint::Midpoint { .. } => ConstraintKind::Midpoint,
        Constraint::Symmetric { .. } => ConstraintKind::Symmetric,
        Constraint::Fix(_) => ConstraintKind::Fix,
        _ => return None,
    })
}

/// The badge for a constraint, in a local frame spanning roughly -1..1 on each axis.
/// Returns nothing for the dimensions, which draw themselves.
fn glyph_strokes(c: &Constraint) -> Vec<[Vec2; 2]> {
    glyph_kind(c).map(kind_strokes).unwrap_or_default()
}

/// The symbol for a constraint, drawn both as the badge beside the geometry and as the
/// toolbar button that applies it. One definition, so the button teaches the badge.
pub fn kind_strokes(kind: ConstraintKind) -> Vec<[Vec2; 2]> {
    let v = Vec2::new;
    match kind {
        ConstraintKind::Horizontal => vec![[v(-1.0, 0.0), v(1.0, 0.0)]],
        ConstraintKind::Vertical => vec![[v(0.0, -1.0), v(0.0, 1.0)]],
        // Two slanted strokes, the drawing convention for parallel.
        ConstraintKind::Parallel => {
            vec![[v(-0.8, -1.0), v(-0.2, 1.0)], [v(0.2, -1.0), v(0.8, 1.0)]]
        }
        ConstraintKind::Perpendicular => {
            vec![[v(-1.0, 1.0), v(-1.0, -1.0)], [v(-1.0, -1.0), v(1.0, -1.0)]]
        }
        ConstraintKind::Equal => vec![
            [v(-1.0, 0.45), v(1.0, 0.45)],
            [v(-1.0, -0.45), v(1.0, -0.45)],
        ],
        // A curve resting on its tangent line.
        ConstraintKind::Tangent => {
            let mut out = vec![[v(-1.0, -0.75), v(1.0, -0.75)]];
            out.extend(
                glyph_ring(0.75)
                    .into_iter()
                    .filter(|[a, b]| a.y >= -0.01 && b.y >= -0.01),
            );
            out
        }
        ConstraintKind::Concentric => {
            let mut out = glyph_ring(1.0);
            out.extend(glyph_ring(0.45));
            out
        }
        ConstraintKind::Coincident => glyph_ring(0.8),
        ConstraintKind::Midpoint => vec![
            [v(0.0, 1.0), v(-0.9, -0.7)],
            [v(-0.9, -0.7), v(0.9, -0.7)],
            [v(0.9, -0.7), v(0.0, 1.0)],
        ],
        ConstraintKind::Symmetric => vec![
            [v(0.0, -1.0), v(0.0, 1.0)],
            [v(-1.0, 0.7), v(-0.35, 0.0)],
            [v(-1.0, -0.7), v(-0.35, 0.0)],
            [v(1.0, 0.7), v(0.35, 0.0)],
            [v(1.0, -0.7), v(0.35, 0.0)],
        ],
        // A pinned point: a box around it.
        ConstraintKind::Fix => vec![
            [v(-0.8, -0.8), v(0.8, -0.8)],
            [v(0.8, -0.8), v(0.8, 0.8)],
            [v(0.8, 0.8), v(-0.8, 0.8)],
            [v(-0.8, 0.8), v(-0.8, -0.8)],
        ],
    }
}

/// Creates the tool's shape from its clicks, ties it to any points the clicks snapped
/// to, and turns typed sizes into driving dimensions. Errors are degenerate input
/// (collinear circle points…) and may leave partial entities behind, so the caller
/// restores its checkpoint.
fn build_shape(
    s: &mut Sketch,
    tool: SketchTool,
    c: &[Click],
    params: &ShapeParams,
) -> Result<(), SketchError> {
    let p = |i: usize| c[i].pos;
    // Points a builder creates on top of a snapped click get tied to the existing
    // point, so shapes join the rest of the sketch just like lines do.
    let mut tie: Vec<(EntityId, usize)> = Vec::new();
    // A typed value is a decision the user made, so it becomes a dimension the way it
    // does in Fusion; a size picked with the pointer stays free to be dragged.
    let mut dims: Vec<Constraint> = Vec::new();
    match tool {
        SketchTool::Rectangle | SketchTool::CenterRectangle => {
            let r = if tool == SketchTool::Rectangle {
                let r = shapes::rectangle_two_point(s, p(0), p(1));
                tie.push((r.corners[0], 0));
                tie.push((r.corners[2], 1));
                r
            } else {
                let r = shapes::rectangle_center(s, p(0), p(1));
                if let Some(center) = r.center {
                    tie.push((center, 0));
                }
                r
            };
            // Corners run counter-clockwise from the minimum corner, so the first edge
            // is the bottom (width) and the second the right side (height).
            if let Some(width) = params.typed(Dim::Width) {
                dims.push(Constraint::Distance {
                    a: r.corners[0],
                    b: r.corners[1],
                    value: width,
                });
            }
            if let Some(height) = params.typed(Dim::Height) {
                dims.push(Constraint::Distance {
                    a: r.corners[1],
                    b: r.corners[2],
                    value: height,
                });
            }
        }
        SketchTool::Circle | SketchTool::Circle2Point => {
            let circle = if tool == SketchTool::Circle {
                let r = (p(1) - p(0)).length().max(1e-3);
                let circle = shapes::circle_center(s, p(0), r);
                tie.push((circle.center, 0));
                circle
            } else {
                shapes::circle_two_point(s, p(0), p(1))
            };
            if let Some(diameter) = params.typed(Dim::Diameter) {
                dims.push(Constraint::Diameter {
                    curve: circle.circle,
                    value: diameter,
                });
            }
        }
        SketchTool::Circle3Point => {
            shapes::circle_three_point(s, p(0), p(1), p(2))?;
        }
        SketchTool::Arc3Point => {
            let a = shapes::arc_three_point(s, p(0), p(1), p(2))?;
            tie.push((a.start, 0));
            tie.push((a.end, 2));
        }
        SketchTool::ArcCenter => {
            let a = shapes::arc_center(s, p(0), p(1), p(2));
            tie.push((a.center, 0));
            tie.push((a.start, 1));
            if let Some(radius) = params.typed(Dim::Radius) {
                dims.push(Constraint::Radius {
                    curve: a.arc,
                    value: radius,
                });
            }
        }
        SketchTool::Polygon => {
            let poly = shapes::polygon_center(s, p(0), p(1), params.sides)?;
            tie.push((poly.center, 0));
            // The construction circle is the polygon's circumcircle, so its radius is
            // exactly the size the user typed.
            if let Some(radius) = params.typed(Dim::Radius) {
                dims.push(Constraint::Radius {
                    curve: poly.circle,
                    value: radius,
                });
            }
        }
        SketchTool::Slot | SketchTool::SlotOverall | SketchTool::SlotCenterPoint => {
            // Every slot is the same stadium; the variants differ in which two points
            // the first clicks give and a third click always sets the width, as in
            // Fusion.
            let width = (2.0 * distance_to_line(p(2), p(0), p(1))).max(1e-3);
            let axis = (p(1) - p(0)).normalize_or(Vec2::X);
            let (a, b) = match tool {
                SketchTool::SlotOverall => {
                    // The clicks are the slot's ends; the arc centres sit half a width
                    // inside them. A slot shorter than its width has no straight part.
                    let inset = (width * 0.5).min(p(0).distance(p(1)) * 0.5 - 1e-3);
                    (p(0) + axis * inset, p(1) - axis * inset)
                }
                SketchTool::SlotCenterPoint => (p(0) * 2.0 - p(1), p(1)),
                _ => (p(0), p(1)),
            };
            let slot = shapes::slot_center_to_center(s, a, b, width);
            match tool {
                SketchTool::Slot => {
                    tie.push((slot.centers[0], 0));
                    tie.push((slot.centers[1], 1));
                }
                SketchTool::SlotCenterPoint => {
                    tie.push((slot.centers[1], 1));
                    // The middle is a construction point on the centre line, so the
                    // slot stays centred on what was clicked.
                    let middle = s.add_point(p(0));
                    s.set_construction(middle, true)?;
                    s.add_constraint(Constraint::Midpoint {
                        point: middle,
                        line: slot.center_line,
                    })?;
                    tie.push((middle, 0));
                }
                _ => {}
            }
            if let Some(length) = params.typed(Dim::Length) {
                // Typed lengths are between the arc centres for every variant but
                // overall, where the ends are a width apart from them.
                let value = if tool == SketchTool::SlotOverall {
                    (length - width).max(1e-3)
                } else {
                    length
                };
                dims.push(Constraint::Distance {
                    a: slot.centers[0],
                    b: slot.centers[1],
                    value,
                });
            }
            // Both arcs are equal, so one diameter fixes the whole width.
            if let Some(width) = params.typed(Dim::Width) {
                dims.push(Constraint::Diameter {
                    curve: slot.arcs[0],
                    value: width,
                });
            }
        }
        SketchTool::Text => {
            let anchor = s.add_point(p(0));
            tie.push((anchor, 0));
            s.add_text(anchor, params.text.clone(), params.text_height, 0.0)?;
        }
        // Tools that draw nothing: the chain tools place their own geometry, and the
        // editing tools never reach the builder at all.
        SketchTool::Select
        | SketchTool::Line
        | SketchTool::Dimension
        | SketchTool::Trim
        | SketchTool::Break
        | SketchTool::Fillet
        | SketchTool::Constrain(_) => {}
    }
    for (created, click) in tie {
        // A click on a curve ties the same way a click on a point does; the solver takes
        // `Coincident` against a line or circle as "on it", not "at its centre".
        if let Some(target) = c[click].snapped.or(c[click].on_curve)
            && target != created
        {
            s.add_constraint(Constraint::Coincident {
                point: created,
                target,
            })?;
        }
    }
    for d in dims {
        s.add_constraint(d)?;
    }
    Ok(())
}

/// Dimensions for a line the user typed sizes for. A typed angle only becomes a
/// constraint when it is horizontal or vertical: there is no axis entity to measure a
/// general angle against, so other angles position the line and leave it free.
fn line_dims(
    line: EntityId,
    start: EntityId,
    end: EntityId,
    params: &ShapeParams,
) -> Vec<Constraint> {
    let mut dims = Vec::new();
    if let Some(length) = params.typed(Dim::Length) {
        dims.push(Constraint::Distance {
            a: start,
            b: end,
            value: length,
        });
    }
    if let Some(angle) = params.typed(Dim::Angle) {
        let quarter_turns = angle / std::f64::consts::FRAC_PI_2;
        if (quarter_turns - quarter_turns.round()).abs() < 1e-9 {
            let flat = quarter_turns.round().rem_euclid(2.0) == 0.0;
            dims.push(if flat {
                Constraint::Horizontal(line)
            } else {
                Constraint::Vertical(line)
            });
        }
    }
    dims
}

/// Closed polyline of a curve or text box, for drawing.
fn outline(sketch: &Sketch, id: EntityId, tess: &Tessellation) -> Option<Vec<Vec2>> {
    match sketch.entity(id).map(|d| &d.entity)? {
        Entity::Text { .. } => sketch.text_box(id).map(|b| {
            let mut v = b.to_vec();
            v.push(b[0]);
            v
        }),
        _ => sketch.curve_polyline(id, tess),
    }
}

/// How far the moved points have strayed from rigid, relative to their own size, or
/// `None` when they are still rigid.
///
/// Distances to the first point are enough: a transform that preserves every distance
/// from one point, for points that are not all collinear, is a rotation and a
/// translation. Collinear sets can still be reflected, which a move cannot produce, so
/// nothing is lost by not testing for it.
fn rigid_error(sketch: &Sketch, start: &[(EntityId, Vec2)]) -> Option<f64> {
    let (anchor, from) = *start.first()?;
    let now_anchor = sketch.point_pos(anchor)?;
    let scale = start
        .iter()
        .map(|(_, p)| p.distance(from))
        .fold(0.0, f64::max);
    if scale <= 0.0 {
        return None;
    }
    let worst = start
        .iter()
        .filter_map(|(id, was)| {
            let now = sketch.point_pos(*id)?;
            Some((now.distance(now_anchor) - was.distance(from)).abs())
        })
        .fold(0.0, f64::max);
    (worst > scale * RIGID_TOL).then_some(worst / scale)
}

/// `toward` moved to exactly `distance` from `from` along the same direction, or left
/// alone when there is no typed distance or no origin yet.
fn at_distance(from: Option<Vec2>, toward: Vec2, distance: Option<f64>) -> Vec2 {
    match (from, distance) {
        (Some(from), Some(d)) => from + (toward - from).normalize_or(Vec2::X) * d,
        _ => toward,
    }
}

/// Extension lines from `a` and `b` along `n` by `off`, the dimension line joining
/// their ends, and arrowheads at both ends. Extension lines stop a little short of the
/// geometry, as drawing convention has it.
fn distance_lines(segments: &mut Vec<[Vec2; 2]>, a: Vec2, b: Vec2, n: Vec2, off: f64, arrow: f64) {
    let (qa, qb) = (a + n * off, b + n * off);
    let clear = n * (arrow * 0.3).copysign(off);
    segments.push([a + clear, qa]);
    segments.push([b + clear, qb]);
    segments.push([qa, qb]);
    arrowheads(segments, qa, qb, arrow);
}

/// Arrowheads pointing outward at both ends of the line `a`–`b`, or inward when the
/// line is too short to hold them.
fn arrowheads(segments: &mut Vec<[Vec2; 2]>, a: Vec2, b: Vec2, size: f64) {
    let dir = (b - a).normalize_or(Vec2::X);
    let flip = if a.distance(b) < size * 3.0 {
        -1.0
    } else {
        1.0
    };
    arrowhead(segments, a, -dir * flip, size);
    arrowhead(segments, b, dir * flip, size);
}

/// An arrowhead at `tip` pointing along `dir`.
fn arrowhead(segments: &mut Vec<[Vec2; 2]>, tip: Vec2, dir: Vec2, size: f64) {
    let dir = dir.normalize_or(Vec2::X);
    let back = tip - dir * size;
    let side = dir.perp() * (size * 0.35);
    segments.push([tip, back + side]);
    segments.push([tip, back - side]);
}

/// Where two infinite lines cross, or `None` when parallel.
fn line_intersection(a0: Vec2, a1: Vec2, b0: Vec2, b1: Vec2) -> Option<Vec2> {
    let (da, db) = (a1 - a0, b1 - b0);
    let denom = da.perp_dot(db);
    if denom.abs() < 1e-12 {
        return None;
    }
    let t = (b0 - a0).perp_dot(db) / denom;
    Some(a0 + da * t)
}

fn point_in_rect(p: Vec2, min: Vec2, max: Vec2) -> bool {
    p.x >= min.x && p.x <= max.x && p.y >= min.y && p.y <= max.y
}

fn distance_to_line(p: Vec2, a: Vec2, b: Vec2) -> f64 {
    let d = b - a;
    if d.length_squared() < 1e-18 {
        return p.distance(a);
    }
    (d.perp_dot(p - a) / d.length()).abs()
}

// --- Dimension text -----------------------------------------------------------------------

/// Angles are stored signed for the solver but shown unsigned, as Fusion shows them.
fn format_value(c: &Constraint) -> String {
    match c {
        Constraint::Angle { value, .. } => format!("{:.2}°", value.to_degrees().abs()),
        Constraint::Diameter { value, .. } => format!("⌀{value:.3}"),
        Constraint::Radius { value, .. } => format!("R{value:.3}"),
        other => other
            .dimension_value()
            .map(|v| format!("{v:.3}"))
            .unwrap_or_default(),
    }
}

/// The bare number to prefill a dimension's edit box with.
pub fn edit_text(c: &Constraint) -> String {
    match c {
        Constraint::Angle { value, .. } => format!("{:.2}", value.to_degrees().abs()),
        other => other
            .dimension_value()
            .map(|v| format!("{v:.3}"))
            .unwrap_or_default(),
    }
}

/// Parses what the user typed in a dimension box: degrees for angles, mm otherwise. An
/// angle keeps the sign of the dimension it replaces, so retyping a value never flips
/// the geometry to the other side.
pub fn parse_value(c: &Constraint, text: &str) -> Option<f64> {
    let cleaned: String = text
        .chars()
        .filter(|ch| ch.is_ascii_digit() || matches!(ch, '.' | '-' | 'e' | 'E'))
        .collect();
    let v: f64 = cleaned.parse().ok()?;
    Some(match c {
        Constraint::Angle { value, .. } => v.abs().to_radians().copysign(*value),
        _ => v,
    })
}

// --- Entering and leaving ---------------------------------------------------------------

/// Points the camera squarely at the sketch plane, keeping the current distance.
fn look_at_plane(camera: &mut Camera, frame: &Frame) {
    camera.look_from_direction(frame.z);
    camera.target = frame.origin;
}

pub fn enter_new(editor: &mut Editor, plane: PlaneRef) {
    let Some(frame) = editor.plane_frame(&plane) else {
        editor.report_error("that plane cannot be sketched on (it is not planar)");
        return;
    };
    editor.doc.begin_transaction();
    let component = editor.active_component;
    let id = editor.doc.add_feature(FeatureKind::Sketch {
        plane,
        component,
        sketch: Sketch::new(),
    });
    start(editor, id, frame, Sketch::new(), None);
}

pub fn enter_existing(editor: &mut Editor, id: FeatureId, previous_cursor: usize) {
    let Some(FeatureKind::Sketch { plane, sketch, .. }) =
        editor.doc.timeline().get(id).map(|f| f.kind.clone())
    else {
        return;
    };
    editor.refresh_cache();
    let Some(frame) = editor.plane_frame(&plane) else {
        editor.report_error("the sketch plane no longer exists");
        return;
    };
    editor.doc.begin_transaction();
    start(editor, id, frame, sketch, Some(previous_cursor));
}

fn start(
    editor: &mut Editor,
    id: FeatureId,
    frame: Frame,
    mut sketch: Sketch,
    restore_cursor: Option<usize>,
) {
    sketch.set_font(editor.font.clone());
    let saved = editor.camera;
    look_at_plane(&mut editor.camera, &frame);
    editor.selection.clear();
    editor.tool = None;
    let mut sketch_editor = SketchEditor::new(id, frame, sketch, saved);
    sketch_editor.restore_cursor = restore_cursor;
    // A sketch opens with snapping as the user left it, not as a fresh one would have
    // it: a switch that quietly turns itself back on is a switch nobody trusts.
    sketch_editor.snap_to_grid = editor.snapping.to_grid;
    sketch_editor.set_free_snap(editor.pointer.shift);
    editor.mode = super::Mode::Sketch(Box::new(sketch_editor));
    editor.set_status(
        "Sketching: type a size and Enter to place it, M moves the selection, E extrudes \
         the region under the pointer, right-click or Esc to end a chain",
    );
    editor.request_repaint();
}

/// Fusion's most-used gesture: point at a closed region inside a sketch, press E, and
/// the extrude tool opens with that region already chosen. The sketch is finished and
/// kept first — the extrude is a separate timeline feature and has to be able to see the
/// sketch it consumes.
pub fn extrude_region(editor: &mut Editor) {
    let super::Mode::Sketch(s) = &editor.mode else {
        return;
    };
    let samples = s.region_samples();
    let sketch = s.feature;
    if samples.is_empty() {
        editor.set_status("Point at a closed region, or click inside one, then press E");
        editor.request_repaint();
        return;
    }
    finish(editor, true);
    editor.selection.clear();
    editor.selection.profiles = samples
        .into_iter()
        .map(|sample| ProfileRef { sketch, sample })
        .collect();
    super::tools::start_tool(editor, super::tools::ToolKind::Extrude);
}

pub fn finish(editor: &mut Editor, keep: bool) {
    let super::Mode::Sketch(s) = std::mem::replace(&mut editor.mode, super::Mode::Model) else {
        return;
    };
    editor.camera = *s.saved_camera();
    // Cursor first: it must be inside the transaction so undo restores it as well.
    if let Some(c) = s.restore_cursor {
        editor.doc.set_cursor(c);
    }
    if keep {
        let sketch = s.sketch.clone();
        let _ = editor.doc.edit_feature_kind(s.feature, |k| {
            if let FeatureKind::Sketch { sketch: target, .. } = k {
                *target = sketch;
            }
        });
        editor.doc.commit_transaction();
        editor.set_status("Sketch finished");
    } else {
        editor.doc.rollback_transaction();
        editor.set_status("Sketch cancelled");
    }
    editor.request_repaint();
}

/// The offset's distance box: the number and the handle are the same value said two
/// ways, and these are the tests that they stay that way.
///
/// They live here rather than in `editor::tests` because they need nothing of the
/// editor: the box, the drag and the geometry are all the sketch editor's own.
#[cfg(test)]
mod offset_entry_tests {
    use super::*;

    /// A sketch editor holding a 40 × 20 rectangle with every curve of it selected —
    /// what an offset starts from.
    fn with_rectangle() -> SketchEditor {
        let mut sketch = Sketch::new();
        shapes::rectangle_two_point(&mut sketch, Vec2::ZERO, Vec2::new(40.0, 20.0));
        let mut s = SketchEditor::new(FeatureId(0), Frame::XY, sketch, Camera::default());
        s.selected = s
            .sketch
            .entities()
            .filter(|(_, d)| d.entity.is_curve())
            .map(|(id, _)| id)
            .collect();
        s
    }

    /// Overall bounds of everything drawn, for judging which side the offset went.
    fn drawn_bounds(s: &SketchEditor) -> (Vec2, Vec2) {
        s.sketch
            .entities()
            .filter_map(|(id, _)| s.sketch.entity_bounds(id))
            .fold(
                (Vec2::splat(f64::INFINITY), Vec2::splat(f64::NEG_INFINITY)),
                |(lo, hi), (min, max)| (lo.min(min), hi.max(max)),
            )
    }

    fn box_text(s: &SketchEditor) -> Option<&str> {
        s.entries
            .iter()
            .find(|e| e.dim == Dim::Distance)
            .map(|e| e.text.as_str())
    }

    /// The box is there the moment the offset is, it holds the keyboard, and what is
    /// typed into it re-makes the geometry — exactly, without the grid rounding it off.
    #[test]
    fn the_distance_can_be_typed_and_the_offset_is_re_made_at_it() {
        let mut s = with_rectangle();
        s.offset.distance = 5.0;
        assert!(s.begin_offset());
        assert_eq!(
            s.entries.iter().map(|e| e.dim).collect::<Vec<_>>(),
            vec![Dim::Distance],
            "the box is there as soon as the offset is"
        );
        assert_eq!(s.entry_focus, Some(0), "and it has the keyboard");
        assert_eq!(box_text(&s), Some("5.00"), "showing the distance in use");

        assert!(
            s.snap_rule().is_on(),
            "the grid is on, so this measures the typing"
        );
        for c in ["3", ".", "2"] {
            assert!(s.type_into_entry(c));
        }
        assert_eq!(s.offset.distance, 3.2, "a typed distance is taken exactly");
        let (min, max) = drawn_bounds(&s);
        assert!(
            (min.y + 3.2).abs() < 1e-9 && (max.y - 23.2).abs() < 1e-9,
            "and the drawing is at it: {min:?} {max:?}"
        );
    }

    /// The other direction: a drag of the handle writes the number back into the box as
    /// it happens, and takes the box off whatever was typed into it — a number that
    /// disagreed with the geometry under the pointer would be worse than no number.
    #[test]
    fn dragging_the_handle_writes_the_number_into_the_box() {
        let mut s = with_rectangle();
        s.snap_to_grid = false;
        s.offset.distance = 5.0;
        assert!(s.begin_offset());
        // The box hangs off the handle, which sits on the result: 5 mm below the bottom
        // edge, halfway along it.
        assert_eq!(s.entry_anchor(), Some(Vec3::new(20.0, -5.0, 0.0)));

        assert!(s.nudge_offset(3.0));
        s.update_offset();
        assert_eq!(box_text(&s), Some("8.00"));
        assert_eq!(s.entry_anchor(), Some(Vec3::new(20.0, -8.0, 0.0)));

        assert!(s.type_into_entry("2"));
        assert!(s.entries[0].locked, "typing locks the box");
        assert!(s.nudge_offset(1.0));
        assert!(!s.entries[0].locked, "and a drag releases it again");
        assert_eq!(box_text(&s), Some("3.00"));
        assert_eq!(s.offset.distance, 3.0);
    }

    /// A dragged distance lands on the grid; shift lets go of it for as long as it is
    /// held, exactly as it does for a drawn point or a dragged move.
    #[test]
    fn a_dragged_distance_snaps_and_shift_lets_it_go() {
        let mut s = with_rectangle();
        s.grid_step = 1.0;
        s.offset.distance = 0.0;
        assert!(s.begin_offset());
        assert!(s.nudge_offset(4.4));
        assert_eq!(s.offset.distance, 4.0, "snapped to the grid");
        s.set_free_snap(true);
        assert!(s.nudge_offset(0.3));
        // 4.7 mm of pointer travel in one gesture, so that is where the handle goes the
        // moment the grid stops holding it. Reading 4.3 — the snapped 4.0 plus the last
        // slice — would mean the grip had quietly fallen behind the mouse.
        assert!(
            (s.offset.distance - 4.7).abs() < 1e-9,
            "shift lets go of it: {}",
            s.offset.distance
        );
    }

    /// A drag does not arrive as one gesture, it arrives as one small delta per frame,
    /// and the value has to reach the pointer either way.
    ///
    /// The regression is the one users describe as snapping that "does not follow the
    /// mouse": rounding each frame's slice instead of the running total returns the same
    /// number every frame — a tenth of a millimetre never survives rounding to a
    /// millimetre — so the handle sits still while the pointer walks away from it, and
    /// only a flick fast enough to cross half a step in a single frame moves anything.
    #[test]
    fn a_drag_delivered_frame_by_frame_still_reaches_the_pointer() {
        let mut s = with_rectangle();
        s.grid_step = 1.0;
        s.offset.distance = 0.0;
        assert!(s.begin_offset());
        // 4.4 mm of pointer travel, as sixty frames of a slow drag.
        for _ in 0..60 {
            s.nudge_offset(4.4 / 60.0);
        }
        assert_eq!(
            s.offset.distance, 4.0,
            "the pointer moved 4.4 mm, so the handle belongs on the 4 mm line"
        );
        // And the gesture, once it ends, does not lend its travel to the next one.
        s.release_drags();
        s.nudge_offset(0.4);
        assert_eq!(s.offset.distance, 4.0, "0.4 mm from 4.0 stays on 4.0");
    }

    /// The same, for every other handle a sketch offers: one slow drag, one grid line.
    #[test]
    fn every_sketch_handle_reaches_the_pointer_frame_by_frame() {
        let mut s = with_rectangle();
        s.grid_step = 1.0;

        assert!(s.begin_move());
        for _ in 0..60 {
            s.nudge_move(true, 4.4 / 60.0);
            s.nudge_move(false, -2.6 / 60.0);
            // 12° of turn, which the 5° step rounds to 10.
            s.turn_move(12.0 / 60.0);
        }
        let op = s.move_op.as_ref().expect("moving");
        assert_eq!(
            (op.dx, op.dy, op.angle_deg),
            (4.0, -3.0, 10.0),
            "each axis and the ring land on their own step"
        );
    }

    /// The sign is the side, so a typed minus is the flip. Half a minus is not a
    /// distance at all, and the preview stays where it was rather than flickering off.
    #[test]
    fn a_typed_minus_puts_the_offset_on_the_other_side() {
        let mut s = with_rectangle();
        s.offset.distance = 5.0;
        assert!(s.begin_offset());
        assert!(s.type_into_entry("-"));
        assert_eq!(s.offset.distance, 5.0, "a lone minus is not a number yet");
        assert!(s.type_into_entry("5"));
        assert_eq!(s.offset.distance, -5.0);
        let (min, max) = drawn_bounds(&s);
        assert!(
            min.abs_diff_eq(Vec2::ZERO, 1e-9) && max.abs_diff_eq(Vec2::new(40.0, 20.0), 1e-9),
            "the offset went inside the rectangle: {min:?} {max:?}"
        );
        assert_eq!(s.offset_status().map(|(n, _)| n), Some(4));
    }

    /// The palette's spinner drives the same distance, and the box follows it rather
    /// than sitting there holding a number the drawing is no longer at.
    #[test]
    fn a_distance_changed_elsewhere_takes_the_box_back_from_what_was_typed() {
        let mut s = with_rectangle();
        s.offset.distance = 5.0;
        assert!(s.begin_offset());
        assert!(s.type_into_entry("7"));
        assert_eq!(box_text(&s), Some("7"), "the user's own typing stands");

        // What the palette's spinner does: the number, then the same update the box asks
        // for.
        s.offset.distance = 9.0;
        s.update_offset();
        assert_eq!(box_text(&s), Some("9.00"));
        assert!(!s.entries[0].locked);
    }

    /// The box belongs to the offset, so it goes when the offset does — by Enter, which
    /// keeps it, as much as by the palette.
    #[test]
    fn enter_in_the_box_keeps_the_offset_and_the_box_goes_with_it() {
        let mut s = with_rectangle();
        s.offset.distance = 5.0;
        assert!(s.begin_offset());
        assert!(s.type_into_entry("7"));
        s.submit_entry();
        assert!(!s.offset_in_progress(), "Enter keeps the offset");
        assert!(
            s.entries.iter().all(|e| e.dim != Dim::Distance),
            "and takes its box away"
        );
        let (min, max) = drawn_bounds(&s);
        assert!(
            (min.y + 7.0).abs() < 1e-9 && (max.y - 27.0).abs() < 1e-9,
            "what was typed is what was kept: {min:?} {max:?}"
        );
    }
}
