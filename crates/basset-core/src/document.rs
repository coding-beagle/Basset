//! The user-facing document: a timeline plus metadata, with cached regeneration and a
//! linear undo/redo history.
//!
//! Undo is implemented by snapshotting the timeline and the parameter table (never the
//! geometry) before each mutation. Both are small — sketches are the largest part — so
//! this is cheaper and far less error-prone than inverse operations, and it means undo can
//! never disagree with what regeneration produces.

use std::sync::Arc;

use basset_sketch::{Font, SketchError, expr};
use serde::{Deserialize, Serialize};

use crate::feature::{Feature, FeatureKind, NumericField};
use crate::ids::FeatureId;
use crate::model::ModelState;
use crate::parameters::Parameters;
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
    #[error("feature {0} has no {1} to drive")]
    NoSuchField(FeatureId, NumericField),
    /// The message says what would go wrong rather than what is wrong, because the
    /// document and the sketch are each perfectly legal on their own.
    #[error("sketch {0} defines {1:?} itself, so renaming would capture its references")]
    ParameterCaptured(FeatureId, String),
    #[error(transparent)]
    Reorder(#[from] ReorderError),
    #[error(transparent)]
    Parameter(#[from] SketchError),
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Document {
    pub name: String,
    pub units: Units,
    timeline: Timeline,
    /// Names every sketch and every feature in the file can read. Kept beside the timeline
    /// rather than inside it because it is not a step of the history: changing one value
    /// re-drives the model from the beginning.
    #[serde(default)]
    parameters: Parameters,
    #[serde(skip)]
    regen: Regenerator,
    #[serde(skip)]
    undo: Vec<Snapshot>,
    #[serde(skip)]
    redo: Vec<Snapshot>,
    /// What `redo` held when the current transaction opened. Opening one records an undo
    /// entry, which clears redo; a transaction that is rolled back has changed nothing, so
    /// what the user could redo before they opened the dialog they must still be able to
    /// redo after they cancel it.
    #[serde(skip)]
    redo_before_transaction: Vec<Snapshot>,
    /// While set, mutations do not record undo entries: the whole transaction undoes as
    /// one step. This is what lets a tool dialog live-update its feature on every slider
    /// change without turning undo into a slider replay.
    #[serde(skip)]
    in_transaction: bool,
}

/// What one undo step restores.
///
/// The parameters travel with the timeline because they drive it: restoring a timeline
/// into a document whose table had moved on would undo the edit and leave the model
/// meaning something neither version ever meant.
#[derive(Clone, Debug)]
struct Snapshot {
    timeline: Timeline,
    parameters: Parameters,
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
            parameters: Parameters::new(),
            regen: Regenerator::default(),
            undo: Vec::new(),
            redo: Vec::new(),
            redo_before_transaction: Vec::new(),
            in_transaction: false,
        }
    }

    /// Starts grouping mutations into one undo step. Nested calls are ignored.
    pub fn begin_transaction(&mut self) {
        if !self.in_transaction {
            self.redo_before_transaction = self.redo.clone();
            self.record_undo();
            self.in_transaction = true;
        }
    }

    /// Keeps everything done since [`Self::begin_transaction`] as a single undo step.
    pub fn commit_transaction(&mut self) {
        self.in_transaction = false;
        // The transaction happened, so the redo stack it cleared is genuinely gone.
        self.redo_before_transaction.clear();
    }

    /// Discards everything done since [`Self::begin_transaction`], leaving the document
    /// exactly as it was — including what could be redone.
    pub fn rollback_transaction(&mut self) {
        if !self.in_transaction {
            return;
        }
        self.in_transaction = false;
        if let Some(previous) = self.undo.pop() {
            self.restore(previous);
        }
        self.redo = std::mem::take(&mut self.redo_before_transaction);
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
        self.sync_parameters();
        self.regen.evaluate(&self.timeline)
    }

    /// Pushes the parameter table into the regenerator, but only when it has actually
    /// changed: handing it over throws away every cached state, and this runs on every
    /// frame.
    fn sync_parameters(&mut self) {
        if self.regen.parameters() != &self.parameters {
            self.regen.set_parameters(self.parameters.clone());
        }
    }

    /// The model as it stood just before `id` was applied: what that feature's inputs
    /// were picked from. `None` if the feature is unknown or beyond the cursor.
    pub fn state_before(&mut self, id: FeatureId) -> Option<&ModelState> {
        let index = self.timeline.index_of(id)?;
        if index >= self.timeline.cursor() {
            return None;
        }
        self.sync_parameters();
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
        // The feature has to exist before anything is recorded: see [`Document::push_undo`].
        if self.timeline.index_of(id).is_none() {
            return Err(DocumentError::UnknownFeature(id));
        }
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
        if self.timeline.index_of(id).is_none() {
            return Err(DocumentError::UnknownFeature(id));
        }
        // Renaming changes nothing geometric, so it must not trigger regeneration.
        self.record_undo();
        let name = name.into();
        self.timeline
            .edit(id, |f| f.name = name)
            .ok_or(DocumentError::UnknownFeature(id))?;
        Ok(())
    }

    pub fn remove_feature(&mut self, id: FeatureId) -> Result<Feature, DocumentError> {
        let index = self
            .timeline
            .index_of(id)
            .ok_or(DocumentError::UnknownFeature(id))?;
        self.record_undo();
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
        // A reorder is validated by performing it, and it leaves the timeline untouched
        // when it refuses, so the snapshot is taken first and kept only if it succeeded.
        let snapshot = self.snapshot();
        let from = self.timeline.reorder(id, new_index)?;
        self.push_undo(snapshot);
        self.regen.invalidate_from(from);
        Ok(())
    }

    /// Rolls the timeline to `cursor` active features. Cached states make rolling back
    /// and forward free.
    pub fn set_cursor(&mut self, cursor: usize) {
        self.record_undo();
        self.timeline.set_cursor(cursor);
    }

    // ----- parameters -------------------------------------------------------------------

    pub fn parameters(&self) -> &Parameters {
        &self.parameters
    }

    /// Adds or re-expresses a document parameter. The expression is evaluated first, so a
    /// typo is refused instead of un-driving everything that reads the name.
    pub fn set_parameter(&mut self, name: &str, expression: &str) -> Result<f64, DocumentError> {
        // On a table of its own, so a refused edit records no undo entry and leaves the
        // document untouched.
        let mut next = self.parameters.clone();
        let value = next.set(name, expression)?;
        self.record_undo();
        self.parameters = next;
        self.refresh_driven_values();
        self.regen.invalidate_from(0);
        Ok(value)
    }

    /// Deletes a parameter. Anything driven by it keeps the value it last had and is
    /// reported as warned on the next regeneration; nothing is deleted with it.
    pub fn remove_parameter(&mut self, name: &str) -> bool {
        if self.parameters.get(name).is_none() {
            return false;
        }
        self.record_undo();
        let removed = self.parameters.remove(name);
        self.refresh_driven_values();
        self.regen.invalidate_from(0);
        removed
    }

    /// Renames a parameter and follows the rename everywhere the old name is read: this
    /// document's own table, every feature value driven by an expression, and every sketch
    /// that does not define the name itself. Returns the sketches left alone because they
    /// define `from` themselves, so the UI can say how far the rename reached.
    ///
    /// Rewriting is the whole reason this exists. References are by name inside expression
    /// text, so remove-and-re-add would leave every dependant reading a name that no longer
    /// resolves. A sketch that *shadows* `from` is deliberately left alone — there the name
    /// means its own parameter, which this rename has nothing to do with — and
    /// `Sketch::rewrite_outer_parameter` is what decides that.
    ///
    /// # Capture
    ///
    /// Refused if any sketch both reads `from` and defines `to` for itself. Rewriting that
    /// sketch's expressions would leave them resolving to the sketch's own `to` instead of
    /// the document's parameter — the drawing would quietly change shape, or, if the
    /// sketch's own row is what mentions `from`, become a cycle. Nothing in either table
    /// alone can see that, because capture is a collision between two scopes; this is the
    /// only place that holds both. A refusal naming the sketch is something the user can
    /// act on, which silently changing their model is not.
    pub fn rename_parameter(
        &mut self,
        from: &str,
        to: &str,
    ) -> Result<Vec<FeatureId>, DocumentError> {
        let mut next = self.parameters.clone();
        next.rename(from, to)?;
        let mut shadowing = Vec::new();
        for feature in self.timeline.features() {
            let FeatureKind::Sketch { sketch, .. } = &feature.kind else {
                continue;
            };
            if sketch.parameter(from).is_some() {
                // Skipped by the rewrite, so nothing here can be captured either.
                shadowing.push(feature.id);
            } else if sketch.parameter(to).is_some() && sketch.mentions_parameter(from) {
                return Err(DocumentError::ParameterCaptured(feature.id, to.to_string()));
            }
        }
        self.record_undo();
        self.parameters = next;
        for feature in self.timeline.features_mut() {
            for text in feature.exprs.values_mut() {
                *text = expr::rename(text, from, to);
            }
            if let FeatureKind::Sketch { sketch, .. } = &mut feature.kind {
                sketch.rewrite_outer_parameter(from, to);
            }
        }
        self.refresh_driven_values();
        self.regen.invalidate_from(0);
        Ok(shadowing)
    }

    /// The sketches that define `name` themselves, and so mean their own parameter rather
    /// than the document's. A rename skips them, and an edit to the document's value does
    /// not reach them; both are worth showing the user.
    pub fn sketches_shadowing(&self, name: &str) -> Vec<FeatureId> {
        self.timeline
            .features()
            .iter()
            .filter(|f| match &f.kind {
                FeatureKind::Sketch { sketch, .. } => sketch.parameter(name).is_some(),
                _ => false,
            })
            .map(|f| f.id)
            .collect()
    }

    /// Drives one of a feature's numbers by an expression over the document's parameters.
    ///
    /// The expression is evaluated before anything is stored, so a bad one is refused
    /// rather than left to warn on every regeneration. The evaluated value is written into
    /// the feature as well as the text, so every reader of the feature — the UI panels
    /// included — sees a plain number whether or not an expression put it there. Replay
    /// re-evaluates the text each time, which is what makes changing the parameter move the
    /// geometry.
    pub fn set_feature_expr(
        &mut self,
        id: FeatureId,
        field: NumericField,
        expression: &str,
    ) -> Result<f64, DocumentError> {
        let feature = self
            .timeline
            .get(id)
            .ok_or(DocumentError::UnknownFeature(id))?;
        if !feature.kind.numeric_fields().contains(&field) {
            return Err(DocumentError::NoSuchField(id, field));
        }
        let value = self.parameters.evaluate(expression)?;
        let expression = expression.to_string();
        self.record_undo();
        let index = self
            .timeline
            .edit(id, |f| {
                f.kind.set_numeric_field(field, value);
                f.exprs.insert(field, expression);
            })
            .ok_or(DocumentError::UnknownFeature(id))?;
        self.regen.invalidate_from(index);
        Ok(value)
    }

    /// Releases a driven feature value back to a plain number, keeping the value the
    /// expression currently gives — the same rule as retyping a number over a driven sketch
    /// dimension. Returns whether it was driven at all.
    pub fn clear_feature_expr(
        &mut self,
        id: FeatureId,
        field: NumericField,
    ) -> Result<bool, DocumentError> {
        let Some(feature) = self.timeline.get(id) else {
            return Err(DocumentError::UnknownFeature(id));
        };
        let Some(expression) = feature.exprs.get(&field) else {
            return Ok(false);
        };
        // The stored number is whatever was last written in; the expression may have moved
        // since, and releasing must keep what the user currently sees, not an older value.
        let value = self.parameters.evaluate(expression).ok();
        self.record_undo();
        let index = self
            .timeline
            .edit(id, |f| {
                if let Some(value) = value {
                    f.kind.set_numeric_field(field, value);
                }
                f.exprs.remove(&field);
            })
            .ok_or(DocumentError::UnknownFeature(id))?;
        self.regen.invalidate_from(index);
        Ok(true)
    }

    /// Edits a sketch feature with the document's parameter table in hand.
    ///
    /// Binding a dimension to an expression has to resolve the names in it *now*, to refuse
    /// a bad one and to apply a good one, and the names may be the document's. The closure
    /// therefore receives the outer table alongside the sketch, which is what the
    /// `_with` half of the sketch API takes. Errors from inside the closure come back
    /// through `R`; the document only reports that the feature exists and is a sketch.
    pub fn edit_sketch<R>(
        &mut self,
        id: FeatureId,
        f: impl FnOnce(&mut basset_sketch::Sketch, basset_sketch::Outer) -> R,
    ) -> Result<R, DocumentError> {
        if !matches!(
            self.timeline.get(id).map(|f| &f.kind),
            Some(FeatureKind::Sketch { .. })
        ) {
            return Err(DocumentError::UnknownFeature(id));
        }
        self.record_undo();
        let lookup = self.parameters.lookup();
        let mut out = None;
        let index = self
            .timeline
            .edit(id, |feature| {
                if let FeatureKind::Sketch { sketch, .. } = &mut feature.kind {
                    out = Some(f(sketch, &lookup));
                }
            })
            .ok_or(DocumentError::UnknownFeature(id))?;
        self.regen.invalidate_from(index);
        out.ok_or(DocumentError::UnknownFeature(id))
    }

    /// The expression driving one of a feature's numbers, if any.
    pub fn feature_expr(&self, id: FeatureId, field: NumericField) -> Option<&str> {
        self.timeline.get(id)?.exprs.get(&field).map(String::as_str)
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
        let current = self.restore(previous);
        self.redo.push(current);
        true
    }

    pub fn redo(&mut self) -> bool {
        let Some(next) = self.redo.pop() else {
            return false;
        };
        let current = self.restore(next);
        self.undo.push(current);
        true
    }

    /// Swaps a snapshot in and hands back the one it replaced.
    fn restore(&mut self, snapshot: Snapshot) -> Snapshot {
        let timeline = std::mem::replace(&mut self.timeline, snapshot.timeline);
        let parameters = std::mem::replace(&mut self.parameters, snapshot.parameters);
        self.refresh_driven_values();
        self.regen.invalidate_from(0);
        Snapshot {
            timeline,
            parameters,
        }
    }

    /// Writes each expression-driven feature value back into the number the timeline
    /// stores for it.
    ///
    /// Replay drives a *copy* of a feature, deliberately: regeneration must not mutate the
    /// history. So after the table moves, the stored numbers are still the ones the old
    /// table gave. The geometry is right either way, but the stored number is the only one
    /// a panel can read, and releasing the expression would snap the value from under the
    /// user. This reconciles the two, and it runs after every change to the table —
    /// including the wholesale swaps undo, redo and a rolled-back transaction perform.
    ///
    /// It records no undo entry and moves no cursor: it derives nothing, it only catches
    /// the timeline up with a change that was recorded already. An expression that no
    /// longer evaluates leaves its number alone, the same rule replay follows.
    fn refresh_driven_values(&mut self) {
        let parameters = &self.parameters;
        for feature in self.timeline.features_mut() {
            if feature.exprs.is_empty() {
                continue;
            }
            let values: Vec<(NumericField, f64)> = feature
                .exprs
                .iter()
                .filter_map(|(field, text)| Some((*field, parameters.evaluate(text).ok()?)))
                .collect();
            for (field, value) in values {
                feature.kind.set_numeric_field(field, value);
            }
        }
    }

    fn record_undo(&mut self) {
        let snapshot = self.snapshot();
        self.push_undo(snapshot);
    }

    /// Keeps a snapshot taken earlier. Separate from [`Document::record_undo`] so an
    /// operation that can be refused snapshots first and only commits the entry once it
    /// knows it changed something: an entry for an edit that did not happen would clear
    /// the redo stack and make the user's next redo do nothing.
    fn push_undo(&mut self, snapshot: Snapshot) {
        const MAX_UNDO: usize = 200;
        if self.in_transaction {
            return;
        }
        self.undo.push(snapshot);
        if self.undo.len() > MAX_UNDO {
            self.undo.remove(0);
        }
        self.redo.clear();
    }

    fn snapshot(&self) -> Snapshot {
        Snapshot {
            timeline: self.timeline.clone(),
            parameters: self.parameters.clone(),
        }
    }
}
