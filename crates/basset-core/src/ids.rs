//! Stable identifiers.
//!
//! Identifiers are plain integers allocated by a per-document counter rather than UUIDs:
//! they must be deterministic so that tests can assert on them and so that two loads of
//! the same file resolve references identically. They are never reused after deletion,
//! which is what keeps a dangling reference detectable instead of silently rebinding.

use serde::{Deserialize, Serialize};
use std::fmt;

#[derive(Copy, Clone, Eq, PartialEq, Hash, Ord, PartialOrd, Debug, Serialize, Deserialize)]
pub struct FeatureId(pub u64);

impl fmt::Display for FeatureId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "f{}", self.0)
    }
}

/// Components are created by features, so a component shares the id of the feature that
/// created it. The root component exists before any feature and gets the reserved id 0.
#[derive(Copy, Clone, Eq, PartialEq, Hash, Ord, PartialOrd, Debug, Serialize, Deserialize)]
pub struct ComponentId(pub u64);

impl ComponentId {
    pub const ROOT: ComponentId = ComponentId(0);

    pub fn from_feature(id: FeatureId) -> Self {
        ComponentId(id.0)
    }
}

impl fmt::Display for ComponentId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if *self == Self::ROOT {
            write!(f, "root")
        } else {
            write!(f, "c{}", self.0)
        }
    }
}
