//! Typed errors. Everything a user action can trigger is reported through these; the
//! crate never panics on user-derived data.

use slotmap::Key;

use crate::{ConstraintId, EntityId};

#[derive(Debug, thiserror::Error)]
pub enum SketchError {
    #[error("entity {0:?} does not exist")]
    UnknownEntity(EntityId),
    #[error("constraint {0:?} does not exist")]
    UnknownConstraint(ConstraintId),
    #[error("entity {id:?} is a {actual}, but a {expected} was required")]
    WrongEntityKind {
        id: EntityId,
        expected: &'static str,
        actual: &'static str,
    },
    #[error("constraint {0:?} is not a dimension")]
    NotADimension(ConstraintId),
    #[error("degenerate geometry: {0}")]
    DegenerateGeometry(String),
    #[error("invalid argument: {0}")]
    InvalidArgument(String),
    #[error("curve {index} in the path is not connected to its predecessor")]
    PathNotConnected { index: usize },
    #[error("expression could not be read: {0}")]
    BadExpression(String),
    #[error("no parameter named {0:?}")]
    UnknownParameter(String),
    #[error("parameter refers to itself: {0}")]
    CircularParameter(String),
    #[error("font could not be parsed: {0}")]
    InvalidFont(String),
    #[error("i/o error: {0}")]
    Io(#[from] std::io::Error),
}

impl SketchError {
    /// Convenience so callers can format ids without depending on slotmap directly.
    pub fn entity_id_string(id: EntityId) -> String {
        format!("{:?}", id.data())
    }
}

#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum SolveError {
    /// The constraint system has no solution reachable from the current state: it is
    /// either conflicting (over-constrained) or the solver got stuck in a local minimum.
    ///
    /// `conflicting` names the constraints still unsatisfied when the solver gave up,
    /// worst first, so the editor can point at them instead of quoting a residual at the
    /// user. It is empty when nothing can be attributed — a solve that failed with no
    /// constraint left over is a local minimum, not a disagreement.
    #[error("constraints conflict (residual {residual:.3e} after {iterations} iterations)")]
    DidNotConverge {
        residual: f64,
        iterations: usize,
        conflicting: Vec<ConstraintId>,
    },
    /// A residual or derivative became NaN/inf, usually from degenerate geometry such as a
    /// zero-length line in a parallel constraint.
    #[error("numerical failure: {0}")]
    Numerical(String),
    #[error("entity {0:?} does not exist")]
    UnknownEntity(EntityId),
    #[error("entity {0:?} is not a point")]
    NotAPoint(EntityId),
}
