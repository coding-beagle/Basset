//! What the user has chosen to hide, saved with the document.
//!
//! None of this changes the model — a hidden body still exists and still takes part in
//! regeneration — so it lives beside the timeline rather than in it, and an undo step never
//! restores it. It is saved because a file that has been tidied for presentation should
//! open the way it was left, not with every sketch and the origin back on screen.

use std::collections::BTreeSet;

use serde::{Deserialize, Serialize};

use crate::ids::FeatureId;
use crate::refs::BodyRef;

/// Every field defaults to what a fresh editor shows, so a file written before visibility
/// was saved opens exactly as it always did.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
pub struct Visibility {
    /// Ordered sets so that saving the same state twice writes the same bytes, which keeps
    /// a file under version control from churning.
    pub hidden_bodies: BTreeSet<BodyRef>,
    pub hidden_sketches: BTreeSet<FeatureId>,
    /// The three origin planes and axes.
    pub show_origin: bool,
    pub show_grid: bool,
}

impl Default for Visibility {
    fn default() -> Self {
        Self {
            hidden_bodies: BTreeSet::new(),
            hidden_sketches: BTreeSet::new(),
            show_origin: false,
            show_grid: true,
        }
    }
}
