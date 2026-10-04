//! What changed between two versions of a document.
//!
//! A `.bass` file in version control diffs as JSON, and a JSON diff of a timeline says
//! `"extent": {"OneSide": 12.0}` became `{"OneSide": 15.0}` — true, and no help to
//! someone looking at the part. This module says it in the terms the modeller works in:
//! which features were added, removed or edited, which parameters moved, and for every
//! body, which faces are new and which are gone. The editor draws the answer over the
//! model — new in green, gone in red — the way a text editor colours a diff in the gutter.
//!
//! Features and bodies are matched by id, which is what makes a diff possible at all:
//! feature ids are allotted once and never reused, so the same id in two versions of a
//! file is the same feature, and a body is named by the feature that created it. Faces
//! are matched by their [`FaceKey`], the operation and role that made them, for the same
//! reason. A face under the same key in both versions is compared by its geometry, so a
//! taller extrude shows its side faces as changed and its base as untouched, which is
//! what the user did.
//!
//! Both documents are replayed to their full length, cursor or no cursor: the file holds
//! the whole timeline, and the diff is of the file.

use std::collections::{BTreeMap, BTreeSet};
use std::sync::Arc;

use basset_kernel::{FaceKey, Solid};

use crate::document::Document;
use crate::ids::FeatureId;
use crate::model::ModelState;
use crate::refs::BodyRef;

/// How a feature or a parameter differs between the base and the working version.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Change {
    /// In the working version only.
    Added,
    /// In the base version only.
    Removed,
    /// In both, and not the same.
    Modified,
}

#[derive(Clone, Debug, PartialEq)]
pub struct FeatureChange {
    pub id: FeatureId,
    /// The working name, or the base name for a removed feature.
    pub name: String,
    /// What kind of feature it is, as [`FeatureKind::default_name`] puts it, so a
    /// removed feature can be drawn without its kind being looked up in the base.
    ///
    /// [`FeatureKind::default_name`]: crate::feature::FeatureKind::default_name
    pub kind: &'static str,
    pub change: Change,
    /// Where a chip for this feature goes in the working timeline: its own index, or for
    /// a removed feature the index of whatever now follows the place it stood in.
    pub slot: usize,
    /// Where the feature sits in the base timeline, if it is there. For a removed
    /// feature this is the only place it has.
    pub base_index: Option<usize>,
    /// Where the feature sits in the working timeline, if it is there.
    pub index: Option<usize>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ParameterChange {
    pub name: String,
    pub change: Change,
    pub base: Option<String>,
    pub working: Option<String>,
}

/// How a body differs. There is no `Unchanged`: a body that is the same is not listed.
#[derive(Clone, Debug)]
pub enum BodyChange {
    /// The working version has it and the base does not.
    Added,
    /// The base version has it and the working does not. The base solid is kept so it
    /// can be drawn where it was.
    Removed { base: Arc<Solid> },
    /// Both have it, and some face differs.
    Modified {
        base: Arc<Solid>,
        /// Faces of the working solid the base solid does not have in this shape: new
        /// faces, and faces under a shared key whose geometry moved.
        added_faces: Vec<FaceKey>,
        /// Faces of the base solid the working solid does not have in this shape: faces
        /// that are gone, and the base shape of every face that moved.
        removed_faces: Vec<FaceKey>,
    },
}

impl BodyChange {
    pub fn change(&self) -> Change {
        match self {
            BodyChange::Added => Change::Added,
            BodyChange::Removed { .. } => Change::Removed,
            BodyChange::Modified { .. } => Change::Modified,
        }
    }

    /// The base solid, for the two changes that have one to show.
    pub fn base(&self) -> Option<&Arc<Solid>> {
        match self {
            BodyChange::Added => None,
            BodyChange::Removed { base } | BodyChange::Modified { base, .. } => Some(base),
        }
    }
}

/// How many of a kind of thing were added, removed and modified.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Counts {
    pub added: usize,
    pub removed: usize,
    pub modified: usize,
}

impl Counts {
    fn tally(&mut self, change: Change) {
        match change {
            Change::Added => self.added += 1,
            Change::Removed => self.removed += 1,
            Change::Modified => self.modified += 1,
        }
    }

    pub fn total(&self) -> usize {
        self.added + self.removed + self.modified
    }

    /// `+2 ~1 −1`, with the zero terms left out; empty when nothing changed.
    pub fn label(&self) -> String {
        let mut parts = Vec::new();
        if self.added > 0 {
            parts.push(format!("+{}", self.added));
        }
        if self.modified > 0 {
            parts.push(format!("~{}", self.modified));
        }
        if self.removed > 0 {
            parts.push(format!("\u{2212}{}", self.removed));
        }
        parts.join(" ")
    }
}

#[derive(Clone, Debug, Default)]
pub struct DocumentDiff {
    /// Every feature that differs, in working-timeline order, with the removed ones
    /// placed by their base index.
    pub features: Vec<FeatureChange>,
    pub parameters: Vec<ParameterChange>,
    pub bodies: BTreeMap<BodyRef, BodyChange>,
}

impl DocumentDiff {
    pub fn is_empty(&self) -> bool {
        self.features.is_empty() && self.parameters.is_empty() && self.bodies.is_empty()
    }

    pub fn feature(&self, id: FeatureId) -> Option<&FeatureChange> {
        self.features.iter().find(|f| f.id == id)
    }

    pub fn body(&self, id: BodyRef) -> Option<&BodyChange> {
        self.bodies.get(&id)
    }

    /// The features the base has and the working version does not, in base order.
    pub fn removed_features(&self) -> impl Iterator<Item = &FeatureChange> {
        self.features.iter().filter(|f| f.change == Change::Removed)
    }

    pub fn feature_counts(&self) -> Counts {
        let mut counts = Counts::default();
        for f in &self.features {
            counts.tally(f.change);
        }
        counts
    }

    pub fn body_counts(&self) -> Counts {
        let mut counts = Counts::default();
        for b in self.bodies.values() {
            counts.tally(b.change());
        }
        counts
    }

    /// One line for a status bar: `features +1 ~2 · bodies ~1 · parameters ~1`, or
    /// `no changes`.
    pub fn summary(&self) -> String {
        if self.is_empty() {
            return "no changes".into();
        }
        let mut parts = Vec::new();
        let features = self.feature_counts();
        if features.total() > 0 {
            parts.push(format!("features {}", features.label()));
        }
        let bodies = self.body_counts();
        if bodies.total() > 0 {
            parts.push(format!("bodies {}", bodies.label()));
        }
        let mut parameters = Counts::default();
        for p in &self.parameters {
            parameters.tally(p.change);
        }
        if parameters.total() > 0 {
            parts.push(format!("parameters {}", parameters.label()));
        }
        parts.join(" \u{b7} ")
    }
}

/// Compares `working` against `base`: what someone who had `base` would have to do to
/// arrive at `working`.
///
/// Both documents are evaluated, so both are `&mut`; neither is otherwise changed. A
/// feature is compared as the file stores it, so a rename or a suppression is a change,
/// and the cursor is not, because the cursor is a place in the history rather than a
/// part of it.
pub fn diff(base: &mut Document, working: &mut Document) -> DocumentDiff {
    let mut out = DocumentDiff {
        features: diff_features(base, working),
        parameters: diff_parameters(base, working),
        bodies: BTreeMap::new(),
    };
    let base_state = base.full_state();
    let working_state = working.full_state();
    out.bodies = diff_bodies(&base_state, &working_state);
    out
}

fn diff_features(base: &Document, working: &Document) -> Vec<FeatureChange> {
    let base_features = base.timeline().features();
    let working_features = working.timeline().features();
    let base_index: BTreeMap<FeatureId, usize> = base_features
        .iter()
        .enumerate()
        .map(|(i, f)| (f.id, i))
        .collect();
    let working_index: BTreeMap<FeatureId, usize> = working_features
        .iter()
        .enumerate()
        .map(|(i, f)| (f.id, i))
        .collect();

    let mut changes = Vec::new();
    for (i, f) in working_features.iter().enumerate() {
        match base_index.get(&f.id) {
            None => changes.push(FeatureChange {
                id: f.id,
                name: f.name.clone(),
                kind: f.kind.default_name(),
                change: Change::Added,
                slot: i,
                base_index: None,
                index: Some(i),
            }),
            Some(&bi) => {
                let was = &base_features[bi];
                // The file's own view of the feature, so whatever the file would show as
                // different is different here, and nothing else is. Two features that
                // serialise alike are alike.
                let same = serde_json::to_value(was).ok() == serde_json::to_value(f).ok();
                if !same {
                    changes.push(FeatureChange {
                        id: f.id,
                        name: f.name.clone(),
                        kind: f.kind.default_name(),
                        change: Change::Modified,
                        slot: i,
                        base_index: Some(bi),
                        index: Some(i),
                    });
                }
            }
        }
    }
    for (bi, f) in base_features.iter().enumerate() {
        if !working_index.contains_key(&f.id) {
            // A removed feature takes the place of the base neighbour it followed, so
            // the list reads as one history with the gaps marked rather than as two
            // lists.
            let slot = base_features[..bi]
                .iter()
                .rev()
                .find_map(|f| working_index.get(&f.id).map(|i| i + 1))
                .unwrap_or(0);
            changes.push(FeatureChange {
                id: f.id,
                name: f.name.clone(),
                kind: f.kind.default_name(),
                change: Change::Removed,
                slot,
                base_index: Some(bi),
                index: None,
            });
        }
    }
    // A removed feature comes before whatever now stands where it stood, which is the
    // order a text diff puts the old line and the new one in.
    changes.sort_by_key(|c| (c.slot, c.index.is_some()));
    changes
}

fn diff_parameters(base: &Document, working: &Document) -> Vec<ParameterChange> {
    let base_rows: BTreeMap<&str, &str> = base
        .parameters()
        .rows()
        .iter()
        .map(|p| (p.name.as_str(), p.expr.as_str()))
        .collect();
    let working_rows: BTreeMap<&str, &str> = working
        .parameters()
        .rows()
        .iter()
        .map(|p| (p.name.as_str(), p.expr.as_str()))
        .collect();
    let mut changes = Vec::new();
    for p in working.parameters().rows() {
        match base_rows.get(p.name.as_str()) {
            None => changes.push(ParameterChange {
                name: p.name.clone(),
                change: Change::Added,
                base: None,
                working: Some(p.expr.clone()),
            }),
            Some(&was) if was != p.expr => changes.push(ParameterChange {
                name: p.name.clone(),
                change: Change::Modified,
                base: Some(was.to_string()),
                working: Some(p.expr.clone()),
            }),
            Some(_) => {}
        }
    }
    for p in base.parameters().rows() {
        if !working_rows.contains_key(p.name.as_str()) {
            changes.push(ParameterChange {
                name: p.name.clone(),
                change: Change::Removed,
                base: Some(p.expr.clone()),
                working: None,
            });
        }
    }
    changes
}

fn diff_bodies(base: &ModelState, working: &ModelState) -> BTreeMap<BodyRef, BodyChange> {
    let mut out = BTreeMap::new();
    for (id, body) in &working.bodies {
        match base.bodies.get(id) {
            None => {
                out.insert(*id, BodyChange::Added);
            }
            Some(was) => {
                if let Some(change) = diff_solid(&was.solid, &body.solid) {
                    out.insert(*id, change);
                }
            }
        }
    }
    for (id, body) in &base.bodies {
        if !working.bodies.contains_key(id) {
            out.insert(
                *id,
                BodyChange::Removed {
                    base: body.solid.clone(),
                },
            );
        }
    }
    out
}

/// The faces that differ between two solids, or `None` when they are the same shape.
///
/// Two solids built by the same operations hold the same face keys, and the same key
/// with the same vertices is the same face; a boolean can leave two faces under one key,
/// so a key's shape is the set of vertices of every polygon under it. The shared
/// pointer is checked first because an unchanged body is usually the very same `Arc`
/// the regenerator cached, and a body in a document of twenty is nineteen times more
/// often unchanged than not.
fn diff_solid(base: &Arc<Solid>, working: &Arc<Solid>) -> Option<BodyChange> {
    if Arc::ptr_eq(base, working) {
        return None;
    }
    let base_faces = face_shapes(base);
    let working_faces = face_shapes(working);
    let mut added_faces = Vec::new();
    let mut removed_faces = Vec::new();
    for (key, shape) in &working_faces {
        match base_faces.get(key) {
            Some(was) if was == shape => {}
            Some(_) => {
                added_faces.push(*key);
                removed_faces.push(*key);
            }
            None => added_faces.push(*key),
        }
    }
    for key in base_faces.keys() {
        if !working_faces.contains_key(key) {
            removed_faces.push(*key);
        }
    }
    if added_faces.is_empty() && removed_faces.is_empty() {
        return None;
    }
    Some(BodyChange::Modified {
        base: base.clone(),
        added_faces,
        removed_faces,
    })
}

/// Vertices quantised to a hundredth of a micron. Two faces that differ by less than
/// that are the same face replayed through a different route, not an edit.
const QUANTUM: f64 = 1e-5;

fn face_shapes(solid: &Solid) -> BTreeMap<FaceKey, BTreeSet<[i64; 3]>> {
    let mut shapes: BTreeMap<FaceKey, BTreeSet<[i64; 3]>> = BTreeMap::new();
    for face in &solid.faces {
        let shape = shapes.entry(face.key).or_default();
        for v in face.polygons.iter().flat_map(|p| p.vertices.iter()) {
            shape.insert([
                (v.x / QUANTUM).round() as i64,
                (v.y / QUANTUM).round() as i64,
                (v.z / QUANTUM).round() as i64,
            ]);
        }
    }
    shapes
}

#[cfg(test)]
mod tests {
    use basset_math::Vec2;
    use basset_sketch::{Sketch, shapes};

    use super::*;
    use crate::feature::{BodyOp, Extent, FeatureKind, NumericField};
    use crate::ids::ComponentId;
    use crate::refs::{OriginPlane, PlaneRef, ProfileRef, RegionRef};

    /// A block, with the ids of its sketch and its extrude.
    fn block(size: f64, height: f64) -> (Document, FeatureId, FeatureId) {
        let mut doc = Document::new("block");
        let mut sketch = Sketch::new();
        shapes::rectangle_two_point(&mut sketch, Vec2::ZERO, Vec2::new(size, size));
        let sk = doc.add_feature(FeatureKind::Sketch {
            plane: PlaneRef::Origin(OriginPlane::XY),
            component: ComponentId::ROOT,
            sketch,
        });
        let ext = doc.add_feature(FeatureKind::Extrude {
            regions: vec![RegionRef::Profile(ProfileRef::new(sk, Vec2::new(1.0, 1.0)))],
            extent: Extent::OneSide(height),
            operation: BodyOp::NewBody,
            component: ComponentId::ROOT,
        });
        (doc, sk, ext)
    }

    #[test]
    fn a_document_is_no_different_from_itself() {
        let (mut a, _, _) = block(10.0, 5.0);
        let mut b = a.clone();
        let d = diff(&mut a, &mut b);
        assert!(d.is_empty(), "{d:?}");
        assert_eq!(d.summary(), "no changes");
    }

    #[test]
    fn a_taller_extrude_moves_its_sides_and_top_and_keeps_its_base() {
        let (mut base, _, ext) = block(10.0, 5.0);
        let mut working = base.clone();
        working
            .edit_feature(ext, |f| {
                f.kind.set_numeric_field(NumericField::Distance, 8.0);
            })
            .unwrap();
        let d = diff(&mut base, &mut working);

        let feature = d.feature(ext).expect("the extrude changed");
        assert_eq!(feature.change, Change::Modified);
        assert_eq!(
            d.features.len(),
            1,
            "the sketch is as it was: {:?}",
            d.features
        );

        let Some(BodyChange::Modified {
            added_faces,
            removed_faces,
            ..
        }) = d.body(BodyRef(ext))
        else {
            panic!("the body changed: {:?}", d.bodies);
        };
        // The base stays where it was; the four sides and the top move. A moved face is
        // both its new shape and its old one.
        assert_eq!(added_faces.len(), 5, "{added_faces:?}");
        assert_eq!(removed_faces.len(), 5, "{removed_faces:?}");
        assert!(
            added_faces
                .iter()
                .all(|k| k.role != basset_kernel::FaceRole::StartCap),
            "the start cap did not move: {added_faces:?}"
        );
        assert_eq!(d.summary(), "features ~1 \u{b7} bodies ~1");
    }

    #[test]
    fn a_new_body_is_added_and_a_deleted_one_is_removed_with_its_solid() {
        let (mut base, sk, ext) = block(10.0, 5.0);
        let mut working = base.clone();
        // Delete the block's extrude and add a different one from the same sketch.
        working.remove_feature(ext).unwrap();
        let other = working.add_feature(FeatureKind::Extrude {
            regions: vec![RegionRef::Profile(ProfileRef::new(sk, Vec2::new(1.0, 1.0)))],
            extent: Extent::OneSide(2.0),
            operation: BodyOp::NewBody,
            component: ComponentId::ROOT,
        });
        let d = diff(&mut base, &mut working);

        assert_eq!(d.feature(ext).map(|f| f.change), Some(Change::Removed));
        assert_eq!(d.feature(other).map(|f| f.change), Some(Change::Added));
        assert!(matches!(d.body(BodyRef(other)), Some(BodyChange::Added)));
        let Some(BodyChange::Removed { base }) = d.body(BodyRef(ext)) else {
            panic!("the old body is gone: {:?}", d.bodies);
        };
        assert!(
            (base.volume() - 500.0).abs() < 1e-9,
            "the base solid is kept"
        );
        assert_eq!(d.feature_counts().label(), "+1 \u{2212}1");
        // The removed feature sits where it was: after the sketch, before the new one.
        let order: Vec<Change> = d.features.iter().map(|f| f.change).collect();
        assert_eq!(
            order,
            vec![Change::Removed, Change::Added],
            "{:?}",
            d.features
        );
        assert!(
            d.features
                .iter()
                .all(|f| f.slot == 1 && f.kind == "Extrude")
        );
    }

    #[test]
    fn the_whole_timeline_is_compared_whatever_the_cursor_says() {
        let (mut base, _, ext) = block(10.0, 5.0);
        let mut working = base.clone();
        working
            .edit_feature(ext, |f| {
                f.kind.set_numeric_field(NumericField::Distance, 8.0);
            })
            .unwrap();
        working.set_cursor(1);
        let revision = working.revision();
        let d = diff(&mut base, &mut working);
        assert!(
            matches!(d.body(BodyRef(ext)), Some(BodyChange::Modified { .. })),
            "the body past the cursor is still in the file: {:?}",
            d.bodies
        );
        assert_eq!(working.timeline().cursor(), 1, "the cursor is where it was");
        assert_eq!(working.revision(), revision, "looking is not editing");
        assert!(
            working.state().bodies.is_empty(),
            "the view is still rolled back"
        );
    }

    #[test]
    fn a_rename_and_a_suppression_are_changes_to_the_file() {
        let (mut base, sk, ext) = block(10.0, 5.0);
        let mut working = base.clone();
        working.rename_feature(sk, "Outline").unwrap();
        working.set_suppressed(ext, true).unwrap();
        let d = diff(&mut base, &mut working);
        assert_eq!(d.feature(sk).map(|f| f.change), Some(Change::Modified));
        assert_eq!(d.feature(ext).map(|f| f.change), Some(Change::Modified));
        // Suppressing the extrude takes its body out of the model.
        assert!(matches!(
            d.body(BodyRef(ext)),
            Some(BodyChange::Removed { .. })
        ));
    }

    #[test]
    fn parameters_are_compared_by_their_text() {
        let (mut base, _, _) = block(10.0, 5.0);
        base.set_parameter("width", "10").unwrap();
        base.set_parameter("gone", "1").unwrap();
        let mut working = base.clone();
        working.set_parameter("width", "12").unwrap();
        working.set_parameter("height", "3").unwrap();
        working.remove_parameter("gone");
        let d = diff(&mut base, &mut working);
        let by_name = |n: &str| d.parameters.iter().find(|p| p.name == n).map(|p| p.change);
        assert_eq!(by_name("width"), Some(Change::Modified));
        assert_eq!(by_name("height"), Some(Change::Added));
        assert_eq!(by_name("gone"), Some(Change::Removed));
        assert!(
            d.summary().contains("parameters +1 ~1 \u{2212}1"),
            "{}",
            d.summary()
        );
    }

    #[test]
    fn the_revision_counts_edits_and_undo_but_not_reads() {
        let (mut doc, _, ext) = block(10.0, 5.0);
        let before = doc.revision();
        let _ = doc.state();
        let _ = doc.full_state();
        assert_eq!(doc.revision(), before, "reading moves nothing");
        doc.edit_feature(ext, |f| {
            f.kind.set_numeric_field(NumericField::Distance, 8.0);
        })
        .unwrap();
        let edited = doc.revision();
        assert_ne!(edited, before);
        assert!(doc.undo());
        assert_ne!(doc.revision(), edited, "undo is a change too");
        // Inside a transaction every live update still counts, or a dialog's slider
        // would move the model without anything watching the revision noticing.
        doc.begin_transaction();
        let opened = doc.revision();
        doc.edit_feature(ext, |f| {
            f.kind.set_numeric_field(NumericField::Distance, 9.0);
        })
        .unwrap();
        assert_ne!(doc.revision(), opened);
        doc.commit_transaction();
    }
}
