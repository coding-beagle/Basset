//! A project's git repository, through the `git` on the path.
//!
//! A project is a folder of `.bass` files under git: the parts of one design, each a
//! document, committed together or apart. JSON lives happily in git; what git cannot do
//! is show the difference between two versions as geometry, which is the editor's job
//! (see [`super::compare`]). This module is the little the editor needs from the
//! repository: which one a folder is, which `.bass` files it holds and in what state,
//! what commits were made and what each touched, the bytes of an older version of a
//! file, and a way to commit some of them.
//!
//! It shells out rather than linking a git library because the user already has a git
//! configured the way they want — credentials, hooks, signing, an `includeIf` per
//! directory — and a library would have to be taught all of it. Every call is one short
//! `git` process; nothing here runs on a frame, so the cost is a few milliseconds when a
//! file is opened, saved, committed or compared.

use std::path::{Path, PathBuf};
use std::process::Command;

use basset_core::file;

/// One commit.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct Commit {
    pub hash: String,
    /// The abbreviated hash, as `git log --abbrev-commit` would print it.
    pub short: String,
    pub subject: String,
    pub author: String,
    /// `3 days ago`, in git's own words.
    pub when: String,
    /// The paths the commit touched, relative to the root, with forward slashes.
    pub files: Vec<String>,
}

impl Commit {
    pub fn touches(&self, rel: &str) -> bool {
        self.files.iter().any(|f| f == rel)
    }
}

/// How the working copy of a file stands against the index and `HEAD`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum FileState {
    /// Git does not know the file.
    Untracked,
    /// The file is as the last commit left it.
    Clean,
    /// The file differs from the last commit, whether or not the change is staged.
    Modified,
}

impl FileState {
    pub fn label(self) -> &'static str {
        match self {
            FileState::Untracked => "untracked",
            FileState::Clean => "clean",
            FileState::Modified => "modified",
        }
    }

    /// Whether a commit of the file would record anything.
    pub fn is_changed(self) -> bool {
        !matches!(self, FileState::Clean)
    }
}

/// One `.bass` file of the project.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct ProjectFile {
    /// Relative to the root, with forward slashes.
    pub path: String,
    pub state: FileState,
}

/// A repository: its root on disk. Paths inside it are strings relative to the root with
/// forward slashes, which is how `git show rev:path` wants them.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct Repo {
    root: PathBuf,
}

/// The log format every listing uses: a record separator, then the fields, each
/// separated by `\x1f`; `--name-only` appends the touched paths one per line.
const LOG_FORMAT: &str = "--format=%x1e%H%x1f%h%x1f%s%x1f%an%x1f%cr";

impl Repo {
    /// The repository `dir` is in, if there is one and `git` is on the path. `dir` may
    /// be the root itself or any directory under it.
    pub fn containing(dir: &Path) -> Option<Repo> {
        let dir = dir.canonicalize().ok()?;
        let out = Command::new("git")
            .arg("-C")
            .arg(&dir)
            .args(["rev-parse", "--show-toplevel"])
            .env("GIT_TERMINAL_PROMPT", "0")
            .output()
            .ok()?;
        if !out.status.success() {
            return None;
        }
        let root = PathBuf::from(String::from_utf8_lossy(&out.stdout).trim());
        Some(Repo { root })
    }

    /// The repository `file` lives in, if there is one.
    pub fn discover(file: &Path) -> Option<Repo> {
        Self::containing(file.parent()?)
    }

    /// Makes `dir` a repository — creating the folder if it does not exist — or returns
    /// the one it already is. This is how a project starts.
    pub fn init(dir: &Path) -> Result<Repo, String> {
        std::fs::create_dir_all(dir)
            .map_err(|e| format!("cannot create {}: {e}", dir.display()))?;
        if let Some(existing) = Self::containing(dir) {
            return Ok(existing);
        }
        let out = Command::new("git")
            .arg("-C")
            .arg(dir)
            .args(["init", "-q"])
            .env("GIT_TERMINAL_PROMPT", "0")
            .output()
            .map_err(|e| format!("could not run git: {e}"))?;
        if !out.status.success() {
            let err = String::from_utf8_lossy(&out.stderr).trim().to_string();
            return Err(if err.is_empty() {
                "git init failed".into()
            } else {
                err
            });
        }
        Self::containing(dir).ok_or_else(|| "git init made no repository".to_string())
    }

    pub fn root(&self) -> &Path {
        &self.root
    }

    /// The project's name: its folder's.
    pub fn name(&self) -> String {
        self.root
            .file_name()
            .map(|n| n.to_string_lossy().into_owned())
            .unwrap_or_else(|| self.root.display().to_string())
    }

    /// `file`'s path inside the repository, or `None` if it is not under the root.
    pub fn relative(&self, file: &Path) -> Option<String> {
        // `--show-toplevel` prints the resolved path, as `canonicalize` does for the
        // file, so the one is a prefix of the other.
        let file = file.canonicalize().ok()?;
        let rel = file.strip_prefix(&self.root).ok()?;
        Some(
            rel.components()
                .map(|c| c.as_os_str().to_string_lossy().into_owned())
                .collect::<Vec<_>>()
                .join("/"),
        )
    }

    /// Where a path inside the repository is on disk.
    pub fn file(&self, rel: &str) -> PathBuf {
        self.root.join(rel)
    }

    /// Runs `git` in the repository's root. `Ok` is stdout; `Err` is what git said on
    /// stderr, trimmed, or the reason it could not run at all.
    fn git(&self, args: &[&str]) -> Result<Vec<u8>, String> {
        let out = Command::new("git")
            .arg("-C")
            .arg(&self.root)
            .args(args)
            .env("GIT_TERMINAL_PROMPT", "0")
            // A status while the user's IDE holds the index lock should wait for
            // nothing: this is a read, and a stale answer beats a stuck one.
            .env("GIT_OPTIONAL_LOCKS", "0")
            .output()
            .map_err(|e| format!("could not run git: {e}"))?;
        if out.status.success() {
            Ok(out.stdout)
        } else {
            let err = String::from_utf8_lossy(&out.stderr).trim().to_string();
            Err(if err.is_empty() {
                format!("git {} failed", args.first().copied().unwrap_or(""))
            } else {
                err
            })
        }
    }

    fn git_text(&self, args: &[&str]) -> Result<String, String> {
        self.git(args)
            .map(|bytes| String::from_utf8_lossy(&bytes).trim_end().to_string())
    }

    /// The current branch, or `HEAD` when detached. `None` in a repository with no
    /// commits yet, where there is no `HEAD` to name.
    pub fn branch(&self) -> Option<String> {
        self.git_text(&["rev-parse", "--abbrev-ref", "HEAD"]).ok()
    }

    /// How one file stands. A path git has never seen and a path that does not exist
    /// are both untracked; the caller knows which.
    pub fn state(&self, rel: &str) -> FileState {
        if self
            .git(&["ls-files", "--error-unmatch", "--", rel])
            .is_err()
        {
            return FileState::Untracked;
        }
        match self.git_text(&["status", "--porcelain", "--", rel]) {
            Ok(s) if s.trim().is_empty() => FileState::Clean,
            Ok(_) => FileState::Modified,
            Err(_) => FileState::Untracked,
        }
    }

    /// Every `.bass` file in the project — tracked or not, as long as it is on disk —
    /// with its state, sorted by path.
    pub fn files(&self) -> Vec<ProjectFile> {
        let is_part = |p: &str| {
            Path::new(p)
                .extension()
                .is_some_and(|e| e.eq_ignore_ascii_case(file::EXTENSION))
        };
        let mut files: Vec<ProjectFile> = self
            .git(&["ls-files", "-z"])
            .map(|bytes| {
                String::from_utf8_lossy(&bytes)
                    .split('\0')
                    .filter(|p| !p.is_empty() && is_part(p))
                    .map(|p| ProjectFile {
                        path: p.to_string(),
                        state: FileState::Clean,
                    })
                    .collect()
            })
            .unwrap_or_default();
        if let Ok(bytes) = self.git(&["status", "--porcelain=v1", "-z", "-uall"]) {
            let text = String::from_utf8_lossy(&bytes);
            let mut fields = text.split('\0');
            while let Some(entry) = fields.next() {
                if entry.len() < 4 {
                    continue;
                }
                let (x, y) = (entry.as_bytes()[0], entry.as_bytes()[1]);
                let path = &entry[3..];
                // A rename or copy carries the original path as the next field.
                if x == b'R' || x == b'C' || y == b'R' || y == b'C' {
                    fields.next();
                }
                if !is_part(path) {
                    continue;
                }
                let deleted = x == b'D' || y == b'D';
                let state = if x == b'?' {
                    FileState::Untracked
                } else {
                    FileState::Modified
                };
                match files.iter_mut().find(|f| f.path == path) {
                    Some(f) if deleted => f.state = FileState::Untracked,
                    Some(f) => f.state = state,
                    None if deleted => {}
                    None => files.push(ProjectFile {
                        path: path.to_string(),
                        state,
                    }),
                }
            }
            // A deleted file is listed by `ls-files` and gone from disk; a project lists
            // what it has.
            files.retain(|f| f.state != FileState::Untracked || self.file(&f.path).exists());
        }
        files.sort_by(|a, b| a.path.cmp(&b.path));
        files
    }

    /// The most recent `limit` commits, newest first: of the whole project, or only those
    /// that touched `rel`.
    pub fn log(&self, limit: usize, rel: Option<&str>) -> Vec<Commit> {
        let n = limit.to_string();
        let mut args = vec!["log", "-n", &n, LOG_FORMAT, "--name-only"];
        if let Some(rel) = rel {
            args.extend(["--", rel]);
        }
        self.git_text(&args)
            .map(|text| parse_log(&text))
            .unwrap_or_default()
    }

    /// The commit a revision names: `HEAD`, a hash, a branch, a tag — anything
    /// `git rev-parse` takes.
    pub fn resolve(&self, rev: &str) -> Result<Commit, String> {
        let spec = format!("{rev}^{{commit}}");
        let text = self.git_text(&["log", "-n", "1", LOG_FORMAT, "--name-only", &spec])?;
        parse_log(&text)
            .into_iter()
            .next()
            .ok_or_else(|| format!("{rev} names no commit"))
    }

    /// The file `rel` as it was at `rev`.
    pub fn show(&self, rev: &str, rel: &str) -> Result<Vec<u8>, String> {
        let spec = format!("{rev}:{rel}");
        self.git(&["show", &spec]).map_err(|e| {
            if e.contains("does not exist") || e.contains("exists on disk, but not in") {
                format!("{rel} is not in {rev}")
            } else {
                e
            }
        })
    }

    /// Stages `paths` and commits them, and only them, with `message`.
    pub fn commit(&self, paths: &[&str], message: &str) -> Result<Commit, String> {
        let message = message.trim();
        if message.is_empty() {
            return Err("a commit needs a message".into());
        }
        if paths.is_empty() {
            return Err("nothing to commit".into());
        }
        let mut add = vec!["add", "--"];
        add.extend_from_slice(paths);
        self.git(&add)?;
        let mut commit = vec!["commit", "-m", message, "--"];
        commit.extend_from_slice(paths);
        self.git(&commit)?;
        self.resolve("HEAD")
    }
}

/// Reads what `git log` prints under [`LOG_FORMAT`] with `--name-only`.
fn parse_log(text: &str) -> Vec<Commit> {
    text.split('\x1e')
        .filter_map(|record| {
            let record = record.trim_matches('\n');
            let (header, body) = record.split_once('\n').unwrap_or((record, ""));
            let mut parts = header.split('\x1f');
            let hash = parts.next().filter(|h| !h.is_empty())?.to_string();
            Some(Commit {
                hash,
                short: parts.next().unwrap_or("").to_string(),
                subject: parts.next().unwrap_or("").to_string(),
                author: parts.next().unwrap_or("").to_string(),
                when: parts.next().unwrap_or("").to_string(),
                files: body
                    .lines()
                    .map(str::trim)
                    .filter(|l| !l.is_empty())
                    .map(str::to_string)
                    .collect(),
            })
        })
        .collect()
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use crate::editor::harness::TempDir;

    /// A fresh repository in a temporary directory, with an identity so commits work
    /// on a machine that has none configured.
    pub(crate) fn init(dir: &TempDir) -> bool {
        let root = dir.join("");
        Command::new("git")
            .arg("-C")
            .arg(&root)
            .args(["init", "-q"])
            .output()
            .is_ok_and(|o| o.status.success())
            && identify(&root)
    }

    /// Gives the repository at `root` an identity, so commits work on a machine that
    /// has none configured, and no signing, so they need no key.
    pub(crate) fn identify(root: &Path) -> bool {
        let run = |args: &[&str]| {
            Command::new("git")
                .arg("-C")
                .arg(root)
                .args(args)
                .output()
                .is_ok_and(|o| o.status.success())
        };
        run(&["config", "user.email", "basset@example.com"])
            && run(&["config", "user.name", "Basset Test"])
            && run(&["config", "commit.gpgsign", "false"])
    }

    pub(crate) fn git_available() -> bool {
        Command::new("git")
            .arg("--version")
            .output()
            .is_ok_and(|o| o.status.success())
    }

    #[test]
    fn a_file_outside_any_repository_has_none() {
        if !git_available() {
            return;
        }
        let dir = TempDir::new("git-none");
        let path = dir.join("loose.bass");
        std::fs::write(&path, "{}").unwrap();
        // A temp dir can itself sit inside a repository on some machines, in which case
        // discovery rightly finds it; what the test can insist on is that a path that
        // does not exist is never in one.
        assert!(Repo::discover(&dir.join("missing/file.bass")).is_none());
    }

    #[test]
    fn a_tracked_file_reports_its_state_and_history() {
        if !git_available() {
            return;
        }
        let dir = TempDir::new("git-track");
        assert!(init(&dir));
        let path = dir.join("part.bass");
        std::fs::write(&path, "one").unwrap();
        let repo = Repo::discover(&path).expect("the file is in the new repository");
        let rel = repo.relative(&path).unwrap();
        assert_eq!(rel, "part.bass");
        assert_eq!(repo.state(&rel), FileState::Untracked);
        assert!(repo.log(5, None).is_empty());
        assert_eq!(
            repo.files(),
            vec![ProjectFile {
                path: rel.clone(),
                state: FileState::Untracked
            }]
        );

        let first = repo.commit(&[&rel], "first").expect("commit");
        assert_eq!(first.subject, "first");
        assert_eq!(first.author, "Basset Test");
        assert_eq!(first.files, vec!["part.bass"]);
        assert_eq!(repo.state(&rel), FileState::Clean);
        assert_eq!(repo.branch().as_deref().map(|b| b.is_empty()), Some(false));

        std::fs::write(&path, "two").unwrap();
        assert_eq!(repo.state(&rel), FileState::Modified);
        assert_eq!(repo.files()[0].state, FileState::Modified);
        assert_eq!(repo.show("HEAD", &rel).unwrap(), b"one");
        let second = repo.commit(&[&rel], "second").unwrap();
        let log = repo.log(5, Some(&rel));
        assert_eq!(log.len(), 2);
        // Hashes, not whole commits: `when` is relative and a second may have passed.
        assert_eq!(log[0].hash, repo.resolve("HEAD").unwrap().hash);
        assert_eq!(log[0].hash, second.hash);
        assert_eq!(log[1].hash, first.hash);
        assert_eq!(repo.show(&first.hash, &rel).unwrap(), b"one");
        assert_eq!(repo.show("HEAD", &rel).unwrap(), b"two");

        assert!(
            repo.commit(&[&rel], "   ").is_err(),
            "an empty message is refused"
        );
        assert!(repo.commit(&[], "x").is_err(), "so is a commit of nothing");
        assert!(repo.resolve("no-such-branch").is_err());
    }

    #[test]
    fn a_project_lists_its_parts_and_what_each_commit_touched() {
        if !git_available() {
            return;
        }
        let dir = TempDir::new("git-project");
        assert!(init(&dir));
        std::fs::create_dir_all(dir.join("parts/arm")).unwrap();
        std::fs::write(dir.join("parts/arm/link.bass"), "x").unwrap();
        std::fs::write(dir.join("base.bass"), "y").unwrap();
        std::fs::write(dir.join("notes.txt"), "not a part").unwrap();
        let repo = Repo::containing(&dir.join("parts")).unwrap();
        assert_eq!(
            repo.name(),
            dir.join("").file_name().unwrap().to_string_lossy()
        );
        assert_eq!(
            repo.relative(&dir.join("parts/arm/link.bass")).as_deref(),
            Some("parts/arm/link.bass")
        );
        let files = repo.files();
        let paths: Vec<&str> = files.iter().map(|f| f.path.as_str()).collect();
        assert_eq!(paths, ["base.bass", "parts/arm/link.bass"]);

        let both = repo
            .commit(&["base.bass", "parts/arm/link.bass"], "add both")
            .unwrap();
        assert_eq!(both.files, ["base.bass", "parts/arm/link.bass"]);
        assert!(both.touches("base.bass"));
        assert!(!both.touches("notes.txt"));
        std::fs::write(dir.join("base.bass"), "y2").unwrap();
        let one = repo.commit(&["base.bass"], "base only").unwrap();
        assert_eq!(one.files, ["base.bass"]);
        assert_eq!(repo.log(10, None).len(), 2);
        assert_eq!(repo.log(10, Some("parts/arm/link.bass")).len(), 1);
        assert!(repo.files().iter().all(|f| f.state == FileState::Clean));

        // A part deleted from disk is no longer a part of the project.
        std::fs::remove_file(dir.join("base.bass")).unwrap();
        let files = repo.files();
        let paths: Vec<&str> = files.iter().map(|f| f.path.as_str()).collect();
        assert_eq!(paths, ["parts/arm/link.bass"]);
        assert!(
            repo.show("HEAD~2", "base.bass").is_err(),
            "the repository has two commits, so there is nothing before them"
        );
    }

    #[test]
    fn init_makes_a_repository_and_finds_one_that_exists() {
        if !git_available() {
            return;
        }
        let dir = TempDir::new("git-init");
        let proj = dir.join("new/project");
        let repo = Repo::init(&proj).expect("init");
        assert_eq!(repo.name(), "project");
        assert!(repo.branch().is_none(), "no commits yet");
        assert_eq!(
            Repo::init(&proj).unwrap(),
            repo,
            "init again is the same repository"
        );
        assert_eq!(Repo::containing(&proj).unwrap(), repo);
    }

    #[test]
    fn the_log_is_read_record_by_record() {
        let text = "\x1eaaa\x1fa\x1fsubject one\x1fAnn\x1f2 days ago\n\nfirst.bass\nsecond.bass\n\x1ebbb\x1fb\x1fsubject two\x1fBob\x1f3 days ago\n";
        let log = parse_log(text);
        assert_eq!(log.len(), 2);
        assert_eq!(log[0].files, ["first.bass", "second.bass"]);
        assert_eq!(log[0].author, "Ann");
        assert_eq!(log[1].subject, "subject two");
        assert!(log[1].files.is_empty());
        assert!(parse_log("").is_empty());
    }
}
