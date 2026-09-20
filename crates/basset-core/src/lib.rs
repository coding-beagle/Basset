//! Document model and regeneration engine.
//!
//! A [`Document`] is a [`Timeline`] of parametric [`Feature`]s plus metadata. Geometry is
//! never stored: it is always the result of replaying the timeline, which is what makes
//! every past edit propagate forward. The replay produces a [`ModelState`] holding the
//! planes, solved sketches, bodies and components that exist at the timeline cursor.
//!
//! The crate is split along those lines:
//! * `ids`/`refs`  — how features name each other (the topological naming strategy).
//! * `feature`     — the parametric inputs of every supported operation.
//! * `timeline`    — ordering, rollback cursor, insertion at the cursor.
//! * `regen`       — evaluating features into geometry, with per-feature caching.
//! * `document`    — the user-facing aggregate with undo/redo.
//! * `file`        — the versioned `.bass` on-disk format.

pub mod document;
pub mod feature;
pub mod file;
pub mod ids;
pub mod model;
pub mod refs;
pub mod regen;
pub mod timeline;

pub use document::{Document, DocumentError, Units};
pub use feature::{BodyOp, CombineOp, Extent, Feature, FeatureKind};
pub use ids::{ComponentId, FeatureId};
pub use model::{Body, Component, FeatureStatus, ModelState, SolvedSketch};
pub use refs::{
    AxisRef, BodyRef, EdgeRef, FaceRef, OriginAxis, OriginPlane, PathRef, PlaneRef, ProfileRef,
    RegionRef,
};
pub use regen::convert_profile;
pub use timeline::Timeline;

// Re-exported so application code only needs one dependency for the object model.
pub use basset_kernel::{EdgeKey, FaceKey, FaceRole, Solid};
pub use basset_sketch::{ConstraintId, EntityId, Sketch};
