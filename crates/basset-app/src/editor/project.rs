//! The open project: a folder of parts under git.
//!
//! A design is rarely one document. A project is the git repository a document lives in,
//! seen as a folder of `.bass` parts: the panel lists them with their state, opens one
//! with a click, commits any set of them with one message, and shows the history of the
//! whole project — every commit, who made it, what it touched — with any part openable as
//! it was at any commit. Saving and committing stay two different acts: Save writes the
//! file, Commit records the files the user ticks, so a half-finished part can be saved
//! for the evening without entering the history.
//!
//! The repository side — files, states, log, show, commit — is [`super::git`]; the
//! geometric comparison with an older version is [`super::compare`]. This module is the
//! editor's state for the project and the commands the panel runs.

use std::collections::BTreeSet;
use std::path::{Path, PathBuf};

use basset_core::file;

use super::Editor;
use super::git::{Commit, FileState, ProjectFile, Repo};

/// How many commits the History section lists.
pub(crate) const HISTORY_LENGTH: usize = 50;

/// The project and where the open document stands in it. Read when a file is opened,
/// saved or committed and when the Git menu opens; nothing here is consulted on a frame.
#[derive(Clone, Debug)]
pub(crate) struct Project {
    pub repo: Repo,
    /// The branch, or `HEAD` when detached, or `None` in a repository with no commits.
    pub branch: Option<String>,
    /// The open document's path inside the project, when it is saved there.
    pub rel: Option<String>,
    /// The open document's state in git, when it is saved in the project.
    pub file: Option<FileState>,
    /// Every part of the project, sorted by path.
    pub files: Vec<ProjectFile>,
    /// The project's commits, newest first, at most [`HISTORY_LENGTH`].
    pub log: Vec<Commit>,
    /// The commit the History section has unfolded.
    pub picked: Option<String>,
}

impl Project {
    /// Reads a project from scratch: `doc` is the open document's path, if it has one.
    pub fn read(repo: Repo, doc: Option<&Path>) -> Self {
        let rel = doc.and_then(|p| repo.relative(p));
        let file = rel.as_deref().map(|r| repo.state(r));
        Self {
            branch: repo.branch(),
            files: repo.files(),
            log: repo.log(HISTORY_LENGTH, None),
            picked: None,
            repo,
            rel,
            file,
        }
    }

    pub fn name(&self) -> String {
        self.repo.name()
    }

    /// The parts a commit would record something for.
    pub fn changed(&self) -> impl Iterator<Item = &ProjectFile> {
        self.files.iter().filter(|f| f.state.is_changed())
    }

    /// The commits that touched the open document, newest first.
    pub fn history_of_document(&self) -> impl Iterator<Item = &Commit> {
        let rel = self.rel.as_deref();
        self.log
            .iter()
            .filter(move |c| rel.is_some_and(|r| c.touches(r)))
    }

    pub fn commit_by_hash(&self, hash: &str) -> Option<&Commit> {
        self.log.iter().find(|c| c.hash == hash)
    }

    /// `⎇ main`, with `●` while the open document differs from the last commit and
    /// `· 2 changed` while other parts do.
    pub fn chip(&self) -> String {
        let branch = self.branch.as_deref().unwrap_or("no commits");
        let mut text = format!("\u{2387} {branch}");
        if self.file == Some(FileState::Modified) {
            text.push_str(" \u{25cf}");
        }
        let others = self
            .changed()
            .filter(|f| Some(&f.path) != self.rel.as_ref())
            .count();
        if others > 0 {
            text.push_str(&format!(" \u{b7} {others} changed"));
        }
        text
    }
}

/// The commit box: a message, and which parts go in.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub(crate) struct CommitBox {
    pub message: String,
    pub include: BTreeSet<String>,
}

impl Editor {
    /// Reads the project from scratch. The project is the repository the open document
    /// is in; a document outside any repository leaves a project that was opened on
    /// purpose where it is, so a new part can be started from the panel and saved into
    /// it.
    pub(crate) fn refresh_project(&mut self) {
        let discovered = self.path.as_deref().and_then(Repo::discover);
        let repo = match (discovered, &self.project) {
            (Some(repo), _) => repo,
            (None, Some(project)) => project.repo.clone(),
            (None, None) => return,
        };
        let picked = self.project.as_ref().and_then(|p| p.picked.clone());
        let mut project = Project::read(repo, self.path.as_deref());
        project.picked = picked.filter(|h| project.commit_by_hash(h).is_some());
        self.project = Some(project);
    }

    /// Opens the project `dir` is in, if it is in one.
    pub(crate) fn open_project_path(&mut self, dir: &Path) {
        match Repo::containing(dir) {
            Some(repo) => self.adopt_project(repo, "Opened project"),
            None => self.report_error(format!(
                "{} is not in a git repository; New project makes it one",
                dir.display()
            )),
        }
    }

    /// Makes `dir` a project — a git repository, created if it is not one — and opens it.
    pub(crate) fn new_project_path(&mut self, dir: &Path) {
        match Repo::init(dir) {
            Ok(repo) => self.adopt_project(repo, "New project"),
            Err(e) => self.report_error(format!("cannot start a project: {e}")),
        }
    }

    fn adopt_project(&mut self, repo: Repo, verb: &str) {
        // A document from elsewhere stays open but is not part of the new project.
        let in_project = self
            .path
            .as_deref()
            .is_some_and(|p| repo.relative(p).is_some());
        let doc = if in_project {
            self.path.as_deref()
        } else {
            None
        };
        let project = Project::read(repo, doc);
        self.set_status(format!("{verb} {}", project.name()));
        self.project = Some(project);
        self.show_project = true;
        self.commit_box = None;
        self.request_repaint();
    }

    /// Picks a folder and opens the project it is in.
    pub(crate) fn open_project(&mut self) {
        let Some(dir) = rfd::FileDialog::new().pick_folder() else {
            return;
        };
        self.open_project_path(&dir);
    }

    /// Picks a folder and makes it a project.
    pub(crate) fn new_project(&mut self) {
        let Some(dir) = rfd::FileDialog::new().pick_folder() else {
            return;
        };
        self.new_project_path(&dir);
    }

    /// Forgets the project. The document stays open; the status bar and the panel stop
    /// talking about git until a file in a repository is opened again.
    pub(crate) fn close_project(&mut self) {
        if self.project.take().is_some() {
            self.compare = None;
            self.commit_box = None;
            self.set_status("Project closed");
            self.request_repaint();
        }
    }

    /// The folder a new document should be saved into: the project's.
    pub(crate) fn project_root(&self) -> Option<PathBuf> {
        self.project.as_ref().map(|p| p.repo.root().to_path_buf())
    }

    /// Opens the part at `rel` in the project.
    pub(crate) fn open_part(&mut self, rel: &str) {
        let Some(path) = self.project.as_ref().map(|p| p.repo.file(rel)) else {
            self.report_error("no project is open");
            return;
        };
        if self.path.as_deref() == Some(path.as_path()) {
            return;
        }
        self.open_path(path);
    }

    /// Opens the part `rel` as it was at `rev`, as a document of its own with no file:
    /// looking at it costs nothing, keeping it is Save As.
    pub(crate) fn open_version(&mut self, rev: &str, rel: &str) {
        let Some(project) = &self.project else {
            self.report_error("no project is open");
            return;
        };
        let commit = match project.repo.resolve(rev) {
            Ok(c) => c,
            Err(e) => {
                self.report_error(format!("cannot open {rel} at {rev}: {e}"));
                return;
            }
        };
        let doc = project
            .repo
            .show(&commit.hash, rel)
            .and_then(|bytes| file::read(bytes.as_slice()).map_err(|e| e.to_string()));
        let mut doc = match doc {
            Ok(doc) => doc,
            Err(e) => {
                self.report_error(format!(
                    "cannot open {rel} at {commit_short}: {e}",
                    commit_short = commit.short
                ));
                return;
            }
        };
        if self.tool.is_some() || self.is_sketching() {
            self.cancel();
        }
        let stem = Path::new(rel)
            .file_stem()
            .map(|s| s.to_string_lossy().into_owned())
            .unwrap_or_else(|| rel.to_string());
        doc.name = format!("{stem} @ {}", commit.short);
        doc.set_font(self.font.clone());
        self.doc = doc;
        self.path = None;
        self.compare = None;
        self.commit_box = None;
        self.selection.clear();
        self.active_component = basset_core::ComponentId::ROOT;
        self.title_dirty = true;
        if let Some(project) = &mut self.project {
            project.rel = None;
            project.file = None;
        }
        self.set_status(format!(
            "Opened {rel} as it was at {} \u{2014} Save As to keep it",
            commit.short
        ));
        self.zoom_to_fit();
        self.request_repaint();
    }

    /// Opens the commit box with every changed part ticked, the open document among them.
    pub(crate) fn begin_commit(&mut self) {
        let Some(project) = &self.project else {
            self.report_error("save the document inside a git repository first");
            return;
        };
        let mut include: BTreeSet<String> = project.changed().map(|f| f.path.clone()).collect();
        if let Some(rel) = &project.rel {
            include.insert(rel.clone());
        }
        self.commit_box = Some(CommitBox {
            message: String::new(),
            include,
        });
        self.request_repaint();
    }

    /// Saves the open document if it is among `paths`, then commits `paths` with
    /// `message`.
    pub(crate) fn commit_files(&mut self, paths: &[String], message: &str) {
        let Some(project) = &self.project else {
            self.report_error("save the document inside a git repository first");
            return;
        };
        if paths.is_empty() {
            self.report_error("nothing to commit: tick at least one part");
            return;
        }
        if project.rel.as_ref().is_some_and(|rel| paths.contains(rel)) {
            self.save(false);
            if self.error.is_some() {
                return;
            }
        }
        let Some(project) = &self.project else {
            return;
        };
        let refs: Vec<&str> = paths.iter().map(String::as_str).collect();
        match project.repo.commit(&refs, message) {
            Ok(commit) => {
                let what = match paths.len() {
                    1 => paths[0].clone(),
                    n => format!("{n} parts"),
                };
                self.set_status(format!(
                    "Committed {} {} ({what})",
                    commit.short, commit.subject
                ));
                self.refresh_project();
                // A comparison with HEAD is now a comparison with what was just
                // committed, which is the document itself; restarting it says so, and
                // leaves the user looking at a clean model rather than a stale diff.
                if self.compare.as_ref().is_some_and(|c| c.spec == "HEAD") {
                    self.compare_with("HEAD");
                }
            }
            Err(e) => self.report_error(format!("commit failed: {e}")),
        }
        self.request_repaint();
    }

    /// Saves the open document and commits it, alone.
    #[cfg(test)]
    pub(crate) fn commit(&mut self, message: &str) {
        if self.path.is_none() {
            self.report_error("save the document inside a git repository first");
            return;
        }
        if self.project.as_ref().and_then(|p| p.rel.as_ref()).is_none() {
            self.save(false);
            if self.error.is_some() {
                return;
            }
        }
        let Some(rel) = self.project.as_ref().and_then(|p| p.rel.clone()) else {
            self.report_error("the document is not in a git repository");
            return;
        };
        self.commit_files(&[rel], message);
    }
}
