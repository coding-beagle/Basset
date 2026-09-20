//! How features refer to geometry produced by earlier features.
//!
//! This is Basset's answer to the topological naming problem. Every reference goes
//! through the `FeatureId` that produced the thing, plus a selector that is stable under
//! edits of that feature's *parameters*:
//!
//! * faces and edges use the kernel's `FaceKey`/`EdgeKey`, which are derived from the
//!   originating operation and the profile curve, not from array positions;
//! * sketch profiles are selected by a *sample point* inside the region, so re-dimensioning
//!   a sketch keeps the extrude attached to "the region around that point" exactly the way
//!   a user would expect after clicking there.

use basset_kernel::{EdgeKey, FaceKey};
use basset_math::Vec2;
use basset_sketch::EntityId;
use serde::{Deserialize, Serialize};

use crate::ids::FeatureId;

#[derive(Copy, Clone, Eq, PartialEq, Hash, Debug, Serialize, Deserialize)]
pub enum OriginPlane {
    XY,
    YZ,
    XZ,
}

#[derive(Copy, Clone, Eq, PartialEq, Hash, Debug, Serialize, Deserialize)]
pub enum OriginAxis {
    X,
    Y,
    Z,
}

/// A body is named by the feature that created it; modifying features (fillet, combine,
/// move) replace the solid in place and keep the name, mirroring Fusion's browser.
#[derive(Copy, Clone, Eq, PartialEq, Hash, Ord, PartialOrd, Debug, Serialize, Deserialize)]
pub struct BodyRef(pub FeatureId);

#[derive(Copy, Clone, PartialEq, Debug, Serialize, Deserialize)]
pub enum PlaneRef {
    Origin(OriginPlane),
    /// A construction plane produced by an offset/angled-plane feature.
    Feature(FeatureId),
    /// A planar face of a body, sketching directly on a solid.
    Face(FaceRef),
}

#[derive(Copy, Clone, PartialEq, Debug, Serialize, Deserialize)]
pub enum AxisRef {
    Origin(OriginAxis),
    /// A line drawn in a sketch, in that sketch's plane.
    SketchLine {
        sketch: FeatureId,
        line: EntityId,
    },
}

#[derive(Copy, Clone, PartialEq, Debug, Serialize, Deserialize)]
pub struct ProfileRef {
    pub sketch: FeatureId,
    /// A point (in sketch coordinates) inside the wanted region.
    pub sample: Vec2,
}

/// Anything a generator can push, spin or skin: a region of a sketch, or a planar face of
/// a body used as a ready-made region. Both resolve to the same kernel profile, so every
/// generator accepts either without caring which the user picked.
#[derive(Copy, Clone, PartialEq, Debug, Serialize, Deserialize)]
pub enum RegionRef {
    Profile(ProfileRef),
    Face(FaceRef),
}

impl RegionRef {
    /// The feature that produces this region, for dependency tracking.
    pub fn source(&self) -> FeatureId {
        match self {
            RegionRef::Profile(p) => p.sketch,
            RegionRef::Face(f) => f.body.0,
        }
    }
}

#[derive(Clone, PartialEq, Debug, Serialize, Deserialize)]
pub struct PathRef {
    pub sketch: FeatureId,
    /// Ordered chain of curve entities.
    pub curves: Vec<EntityId>,
}

#[derive(Copy, Clone, Eq, PartialEq, Hash, Debug, Serialize, Deserialize)]
pub struct FaceRef {
    pub body: BodyRef,
    pub key: FaceKey,
}

#[derive(Copy, Clone, Eq, PartialEq, Hash, Debug, Serialize, Deserialize)]
pub struct EdgeRef {
    pub body: BodyRef,
    pub key: EdgeKey,
}
