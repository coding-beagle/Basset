//! 2D parametric sketching for Basset.
//!
//! # Design
//!
//! **Entities.** Points are first-class; lines, arcs, circles and text reference points
//! by [`EntityId`]. This makes "move a point" the only geometric edit and keeps the
//! solver's unknowns down to point coordinates and circle radii. Arcs carry no radius:
//! it is `|start − center|`, and the solver enforces `|end − center|` equals it.
//!
//! **Constraints** ([`Constraint`]) are validated for entity kinds when added. They are
//! compiled into residual equations written once over forward-mode dual numbers
//! ([`dual::Dual`]), so the Jacobian is exact and every constraint is a few lines of
//! ordinary arithmetic. The solver ([`solver`]) runs dense Levenberg–Marquardt and
//! reports remaining degrees of freedom from the Jacobian rank. Fixed points are
//! excluded from the unknowns. Dimensions with an inherent sign ambiguity (point–line
//! distance, horizontal/vertical distance, tangency side) resolve it from the geometry
//! at the start of each solve so the sketch never flips to the mirror solution.
//!
//! **Profiles** ([`profiles`]) trace faces of the planar graph formed by non-construction
//! curves whose endpoints coincide within [`sketch::JOIN_TOL`], then nest loops by
//! containment into outer contours with holes. Every polyline edge carries a
//! [`Segment`] naming its source curve so the kernel can build one face per curve.
//! Curves are not split at crossings.
//!
//! **Text** ([`text`]) uses `ttf-parser` glyph outlines flattened to the requested chord
//! tolerance. Fonts are attached at runtime and never serialised.
//!
//! **Editing** ([`edit`]) trims and breaks existing curves at the points where other
//! curves cross them, found analytically by [`intersect`]. [`fillet`] rounds the corner
//! where two of them meet, trimming both back to a tangent arc and writing the tangency
//! down as constraints so the corner stays rounded under later edits. [`pattern`] repeats geometry
//! in a grid or around a centre, copying the constraints written between the copied
//! entities so each copy holds its shape. [`offset`] draws a chain of curves alongside
//! an existing one, either rounding the corners so every point is the same distance
//! from the source or mitring them so every edge is.
//!
//! **Parameters** ([`parameters`]) are named constants, written as expressions over each
//! other, that can drive dimensions; solving re-evaluates them first.
//!
//! **Shapes** ([`shapes`]) mirror Fusion's sketch tools and add the constraints those
//! tools add, so a rectangle stays a rectangle under later edits.

pub mod constraint;
pub mod contour;
pub mod dual;
pub mod edit;
pub mod entity;
pub mod error;
pub mod expr;
pub mod fillet;
pub mod geometry;
pub mod intersect;
pub mod linalg;
pub mod offset;
pub mod parameters;
pub mod pattern;
pub mod profiles;
pub mod shapes;
pub mod sketch;
pub mod solver;
pub mod tessellation;
pub mod text;

slotmap::new_key_type! {
    pub struct EntityId;
    pub struct ConstraintId;
}

pub use constraint::Constraint;
pub use contour::{Contour, Profile, Segment, SegmentKind};
pub use entity::{Entity, EntityData};
pub use error::{SketchError, SolveError};
pub use intersect::CurveGeom;
pub use offset::Corner;
pub use parameters::Parameter;
pub use sketch::{Hit, JOIN_TOL, Sketch};
pub use solver::SolveReport;
pub use tessellation::Tessellation;
pub use text::Font;

#[cfg(test)]
mod tests;
