//! The user-facing document: a timeline plus metadata, with cached regeneration and a
//! linear undo/redo history.
//!
//! Undo is implemented by snapshotting the timeline (not the geometry) before each
//! mutation. Timelines are small — sketches are the largest part — so this is cheaper and
//! far less error-prone than inverse operations, and it means undo can never disagree
//! with what regeneration produces.

use std::sync::Arc;

use basset_sketch::Font;
use serde::{Deserialize, Serialize};

use crate::feature::{Feature, FeatureKind};
use crate::ids::FeatureId;
use crate::model::ModelState;
use crate::regen::Regenerator;
use crate::timeline::{ReorderError, Timeline};

#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Units {
    Millimeters,
    Centimeters,
    Meters,
    Inches,
}

impl Units {
    /// Scale from the unit to the internal millimetre representation.
    pub fn to_mm(self) -> f64 {
        match self {
            Units::Millimeters => 1.0,
            Units::Centimeters => 10.0,
            Units::Meters => 1000.0,
            Units::Inches => 25.4,
        }
    }
}

#[derive(Debug, thiserror::Error)]
pub enum DocumentError {
    #[error("unknown feature {0}")]
    UnknownFeature(FeatureId),
    #[error(transparent)]
    Reorder(#[from] ReorderError),
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Document {
    pub name: String,
    pub units: Units,
    timeline: Timeline,
    #[serde(skip)]
    regen: Regenerator,
    #[serde(skip)]
    undo: Vec<Timeline>,
    #[serde(skip)]
    redo: Vec<Timeline>,
    /// While set, mutations do not record undo entries: the whole transaction undoes as
    /// one step. This is what lets a tool dialog live-update its feature on every slider
    /// change without turning undo into a slider replay.
    #[serde(skip)]
    in_transaction: bool,
}

impl Default for Document {
    fn default() -> Self {
        Self::new("Untitled")
    }
}

impl Document {
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            units: Units::Millimeters,
            timeline: Timeline::new(),
            regen: Regenerator::default(),
            undo: Vec::new(),
            redo: Vec::new(),
            in_transaction: false,
        }
    }

    /// Starts grouping mutations into one undo step. Nested calls are ignored.
    pub fn begin_transaction(&mut self) {
        if !self.in_transaction {
            self.record_undo();
            self.in_transaction = true;
        }
    }

    /// Keeps everything done since [`Self::begin_transaction`] as a single undo step.
    pub fn commit_transaction(&mut self) {
        self.in_transaction = false;
    }

    /// Discards everything done since [`Self::begin_transaction`].
    pub fn rollback_transaction(&mut self) {
        if !self.in_transaction {
            return;
        }
        self.in_transaction = false;
        if let Some(previous) = self.undo.pop() {
            self.timeline = previous;
            self.regen.invalidate_from(0);
        }
    }

    pub fn in_transaction(&self) -> bool {
        self.in_transaction
    }

    pub fn timeline(&self) -> &Timeline {
        &self.timeline
    }

    /// Fonts are a runtime resource, not document content, so they are attached after
    /// loading; text in sketches only produces profiles once a font is set.
    pub fn set_font(&mut self, font: Option<Arc<Font>>) {
        self.regen.set_font(font);
        self.regen.invalidate_from(0);
    }

    /// Evaluates the timeline up to the cursor, reusing cached results where inputs are
    /// unchanged.
    pub fn state(&mut self) -> &ModelState {
        self.regen.evaluate(&self.timeline)
    }

    /// The model as it stood just before `id` was applied: what that feature's inputs
    /// were picked from. `None` if the feature is unknown or beyond the cursor.
    pub fn state_before(&mut self, id: FeatureId) -> Option<&ModelState> {
        let index = self.timeline.index_of(id)?;
        if index >= self.timeline.cursor() {
            return None;
        }
        Some(self.regen.evaluate_prefix(&self.timeline, index))
    }

    pub fn add_feature(&mut self, kind: FeatureKind) -> FeatureId {
        self.add_named_feature(kind, None)
    }

    pub fn add_named_feature(&mut self, kind: FeatureKind, name: Option<String>) -> FeatureId {
        self.record_undo();
        let at = self.timeline.cursor();
        let id = self.timeline.insert(kind, name);
        self.regen.invalidate_from(at);
        id
    }

    pub fn edit_feature(
        &mut self,
        id: FeatureId,
        f: impl FnOnce(&mut Feature),
    ) -> Result<(), DocumentError> {
        self.record_undo();
        let index = self
            .timeline
            .edit(id, f)
            .ok_or(DocumentError::UnknownFeature(id))?;
        self.regen.invalidate_from(index);
        Ok(())
    }

    pub fn edit_feature_kind(
        &mut self,
        id: FeatureId,
        f: impl FnOnce(&mut FeatureKind),
    ) -> Result<(), DocumentError> {
        self.edit_feature(id, |feature| f(&mut feature.kind))
    }

    pub fn set_suppressed(&mut self, id: FeatureId, suppressed: bool) -> Result<(), DocumentError> {
        self.edit_feature(id, |f| f.suppressed = suppressed)
    }

    pub fn rename_feature(
        &mut self,
        id: FeatureId,
        name: impl Into<String>,
    ) -> Result<(), DocumentError> {
        // Renaming changes nothing geometric, so it must not trigger regeneration.
        self.record_undo();
        let name = name.into();
        self.timeline
            .edit(id, |f| f.name = name)
            .ok_or(DocumentError::UnknownFeature(id))?;
        Ok(())
    }

    pub fn remove_feature(&mut self, id: FeatureId) -> Result<Feature, DocumentError> {
        self.record_undo();
        let index = self
            .timeline
            .index_of(id)
            .ok_or(DocumentError::UnknownFeature(id))?;
        let feature = self
            .timeline
            .remove(id)
            .ok_or(DocumentError::UnknownFeature(id))?;
        self.regen.invalidate_from(index);
        Ok(feature)
    }

    pub fn reorder_feature(
        &mut self,
        id: FeatureId,
        new_index: usize,
    ) -> Result<(), DocumentError> {
        self.record_undo();
        let from = self.timeline.reorder(id, new_index)?;
        self.regen.invalidate_from(from);
        Ok(())
    }

    /// Rolls the timeline to `cursor` active features. Cached states make rolling back
    /// and forward free.
    pub fn set_cursor(&mut self, cursor: usize) {
        self.record_undo();
        self.timeline.set_cursor(cursor);
    }

    pub fn can_undo(&self) -> bool {
        !self.undo.is_empty()
    }

    pub fn can_redo(&self) -> bool {
        !self.redo.is_empty()
    }

    pub fn undo(&mut self) -> bool {
        let Some(previous) = self.undo.pop() else {
            return false;
        };
        self.redo
            .push(std::mem::replace(&mut self.timeline, previous));
        self.regen.invalidate_from(0);
        true
    }

    pub fn redo(&mut self) -> bool {
        let Some(next) = self.redo.pop() else {
            return false;
        };
        self.undo.push(std::mem::replace(&mut self.timeline, next));
        self.regen.invalidate_from(0);
        true
    }

    fn record_undo(&mut self) {
        const MAX_UNDO: usize = 200;
        if self.in_transaction {
            return;
        }
        self.undo.push(self.timeline.clone());
        if self.undo.len() > MAX_UNDO {
            self.undo.remove(0);
        }
        self.redo.clear();
    }
}
