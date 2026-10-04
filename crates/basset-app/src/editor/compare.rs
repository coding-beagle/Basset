//! Comparing the open document with a version of it in git, drawn as geometry.
//!
//! `git diff` on a `.bass` file says which numbers changed. The user wants to know what
//! the part looks like now compared with how it looked then, so this holds an older
//! version of the document — read out of the repository, regenerated — and the
//! [`DocumentDiff`] between it and the one being edited. The scene draws that diff over
//! the model: faces and bodies the old version did not have in green, faces and bodies
//! it had and this one does not as red ghosts; the timeline marks the features that
//! differ and shows the ones that are gone. The diff is recomputed only when the
//! document's revision moves, so a live tool dialog sees its change go green as it drags
//! and an idle frame costs nothing.
//!
//! The git side of this — which repository, which commits — is in [`super::git`] and
//! the project it belongs to in [`super::project`]; what differs between two documents
//! is [`basset_core::diff`]. This module is the editor's use of them.

use std::sync::Arc;

use basset_core::{BodyRef, Document, DocumentDiff, Solid, file};

use super::Editor;
use super::git::Commit;

/// How many commits the Git menu offers to compare with.
pub(crate) const LOG_LENGTH: usize = 12;

/// The open document against one version of it in the repository.
pub(crate) struct Compare {
    /// What the user asked for: `HEAD`, or a hash from the log.
    pub spec: String,
    /// What that resolved to when the comparison started.
    pub commit: Commit,
    base: Document,
    pub diff: DocumentDiff,
    /// The document revision the diff was computed against.
    at_revision: Option<u64>,
}

impl Compare {
    /// `HEAD (abc1234)` or `abc1234 Shorten the arm`: how the comparison is named in the
    /// banner and the status bar.
    pub fn label(&self) -> String {
        if self.spec == "HEAD" {
            format!("HEAD ({})", self.commit.short)
        } else {
            format!("{} {}", self.commit.short, self.commit.subject)
        }
    }

    /// The base solids there is something to draw of: every removed body, and every
    /// changed body in the shape it had.
    pub fn base_solids(&self) -> impl Iterator<Item = (BodyRef, &Arc<Solid>)> {
        self.diff
            .bodies
            .iter()
            .filter_map(|(id, change)| change.base().map(|solid| (*id, solid)))
    }
}

impl Editor {
    /// Starts comparing the document with the version of it at `spec`, replacing any
    /// comparison already running.
    pub(crate) fn compare_with(&mut self, spec: &str) {
        let Some((repo, rel)) = self
            .project
            .as_ref()
            .and_then(|p| Some((p.repo.clone(), p.rel.clone()?)))
        else {
            self.report_error(if self.path.is_none() {
                "save the document inside a git repository first".to_string()
            } else {
                "the document is not in a git repository".to_string()
            });
            return;
        };
        let commit = match repo.resolve(spec) {
            Ok(c) => c,
            Err(e) => {
                self.report_error(format!("cannot compare with {spec}: {e}"));
                return;
            }
        };
        let base = match repo
            .show(&commit.hash, &rel)
            .and_then(|bytes| file::read(bytes.as_slice()).map_err(|e| e.to_string()))
        {
            Ok(mut doc) => {
                doc.set_font(self.font.clone());
                doc
            }
            Err(e) => {
                self.report_error(format!("cannot compare with {spec}: {e}"));
                return;
            }
        };
        self.compare = Some(Compare {
            spec: spec.to_string(),
            commit,
            base,
            diff: DocumentDiff::default(),
            at_revision: None,
        });
        self.refresh_compare();
        let status = self
            .compare
            .as_ref()
            .map(|c| format!("Comparing with {}: {}", c.label(), c.diff.summary()))
            .unwrap_or_default();
        self.set_status(status);
        self.request_repaint();
    }

    pub(crate) fn stop_compare(&mut self) {
        if self.compare.take().is_some() {
            self.set_status("Comparison off");
            self.request_repaint();
        }
    }

    /// Starts a comparison with `HEAD`, or stops the one running: what one key means.
    pub(crate) fn toggle_compare(&mut self) {
        if self.compare.is_some() {
            self.stop_compare();
        } else {
            self.compare_with("HEAD");
        }
    }

    /// Brings the diff up to date with the document, if the document moved. Called from
    /// [`Editor::refresh_cache`] every frame; a frame in which nothing changed does one
    /// integer comparison.
    pub(crate) fn refresh_compare(&mut self) {
        let revision = self.doc.revision();
        let Some(compare) = &mut self.compare else {
            return;
        };
        if compare.at_revision == Some(revision) {
            return;
        }
        compare.diff = basset_core::diff::diff(&mut compare.base, &mut self.doc);
        compare.at_revision = Some(revision);
    }
}
