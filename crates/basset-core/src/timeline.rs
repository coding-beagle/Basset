//! Ordered feature list with a rollback cursor.
//!
//! The cursor splits the timeline into an active prefix and an inactive suffix. New
//! features are inserted *at* the cursor, which is how "roll back, add a feature, roll
//! forward" inserts history in the middle exactly like Fusion.

use serde::{Deserialize, Serialize};

use crate::feature::{Feature, FeatureKind};
use crate::ids::FeatureId;

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct Timeline {
    features: Vec<Feature>,
    /// Number of active features. `cursor == features.len()` means fully rolled forward.
    cursor: usize,
    next_id: u64,
}

impl Timeline {
    pub fn new() -> Self {
        // Id 0 is reserved for the root component so a `ComponentId::from_feature` never
        // collides with it.
        Self {
            features: Vec::new(),
            cursor: 0,
            next_id: 1,
        }
    }

    pub fn features(&self) -> &[Feature] {
        &self.features
    }

    pub fn len(&self) -> usize {
        self.features.len()
    }

    pub fn is_empty(&self) -> bool {
        self.features.is_empty()
    }

    pub fn cursor(&self) -> usize {
        self.cursor
    }

    pub fn active(&self) -> &[Feature] {
        &self.features[..self.cursor]
    }

    /// Every feature, mutably, for edits that touch the whole timeline at once — renaming
    /// a document parameter, which every feature and sketch may mention. Ordinary edits go
    /// through [`Timeline::edit`], which reports where regeneration has to restart.
    pub fn features_mut(&mut self) -> &mut [Feature] {
        &mut self.features
    }

    pub fn get(&self, id: FeatureId) -> Option<&Feature> {
        self.features.iter().find(|f| f.id == id)
    }

    pub fn index_of(&self, id: FeatureId) -> Option<usize> {
        self.features.iter().position(|f| f.id == id)
    }

    /// Inserts at the cursor and advances the cursor past the new feature.
    pub fn insert(&mut self, kind: FeatureKind, name: Option<String>) -> FeatureId {
        let id = FeatureId(self.next_id);
        self.next_id += 1;
        let name = name.unwrap_or_else(|| format!("{}{}", kind.default_name(), id.0));
        self.features.insert(
            self.cursor,
            Feature {
                id,
                name,
                suppressed: false,
                kind,
                exprs: Default::default(),
            },
        );
        self.cursor += 1;
        id
    }

    /// Returns the index of the edited feature so callers know where regeneration must
    /// restart.
    pub fn edit(&mut self, id: FeatureId, f: impl FnOnce(&mut Feature)) -> Option<usize> {
        let index = self.index_of(id)?;
        f(&mut self.features[index]);
        Some(index)
    }

    pub fn remove(&mut self, id: FeatureId) -> Option<Feature> {
        let index = self.index_of(id)?;
        if index < self.cursor {
            self.cursor -= 1;
        }
        Some(self.features.remove(index))
    }

    /// Moves a feature to `new_index`. Returns the smaller of the old and new indices,
    /// which is where regeneration has to restart. Rejects moves that would place a
    /// feature before something it depends on.
    pub fn reorder(&mut self, id: FeatureId, new_index: usize) -> Result<usize, ReorderError> {
        let old = self.index_of(id).ok_or(ReorderError::UnknownFeature(id))?;
        let new_index = new_index.min(self.features.len() - 1);
        let deps = self.features[old].kind.dependencies();
        for (i, other) in self.features.iter().enumerate() {
            let dependent_on_us = other.kind.dependencies().contains(&id);
            if deps.contains(&other.id) && i >= new_index && i != old {
                return Err(ReorderError::WouldPrecedeDependency(other.id));
            }
            if dependent_on_us && i <= new_index && i != old {
                return Err(ReorderError::WouldFollowDependant(other.id));
            }
        }
        let feature = self.features.remove(old);
        self.features.insert(new_index, feature);
        Ok(old.min(new_index))
    }

    pub fn set_cursor(&mut self, cursor: usize) {
        self.cursor = cursor.min(self.features.len());
    }

    pub fn dependants_of(&self, id: FeatureId) -> Vec<FeatureId> {
        self.features
            .iter()
            .filter(|f| f.kind.dependencies().contains(&id))
            .map(|f| f.id)
            .collect()
    }
}

#[derive(Debug, thiserror::Error, PartialEq)]
pub enum ReorderError {
    #[error("unknown feature {0}")]
    UnknownFeature(FeatureId),
    #[error("feature would be placed before {0}, which it depends on")]
    WouldPrecedeDependency(FeatureId),
    #[error("feature would be placed after {0}, which depends on it")]
    WouldFollowDependant(FeatureId),
}
