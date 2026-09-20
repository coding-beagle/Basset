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

use basset_math::{Vec2, Vec3};
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
}
