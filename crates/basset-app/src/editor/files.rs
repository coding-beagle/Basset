//! Document files and mesh export through native dialogs.

use std::path::{Path, PathBuf};

use basset_core::file;
use basset_io::{ExportItem, Unit};

use super::Editor;

impl Editor {
    pub fn new_document(&mut self) {
        if self.tool.is_some() || self.is_sketching() {
            self.cancel();
        }
        self.doc = basset_core::Document::new("Untitled");
        self.doc.set_font(self.font.clone());
        self.path = None;
        self.selection.clear();
        self.hidden_bodies.clear();
        self.hidden_sketches.clear();
        self.active_component = basset_core::ComponentId::ROOT;
        self.title_dirty = true;
        self.set_status("New document");
        self.request_repaint();
    }

    pub fn open(&mut self) {
        let Some(path) = rfd::FileDialog::new()
            .add_filter("Basset document", &[file::EXTENSION])
            .pick_file()
        else {
            return;
        };
        self.open_path(path);
    }

    pub fn open_path(&mut self, path: PathBuf) {
        match file::load(&path) {
            Ok(mut doc) => {
                if self.tool.is_some() || self.is_sketching() {
                    self.cancel();
                }
                doc.set_font(self.font.clone());
                self.doc = doc;
                self.path = Some(path.clone());
                self.selection.clear();
                self.hidden_bodies.clear();
                self.hidden_sketches.clear();
                self.active_component = basset_core::ComponentId::ROOT;
                self.title_dirty = true;
                self.set_status(format!("Opened {}", path.display()));
                self.zoom_to_fit();
            }
            Err(e) => self.report_error(format!("could not open {}: {e}", path.display())),
        }
        self.request_repaint();
    }

    pub fn save(&mut self, as_new: bool) {
        let path = match (&self.path, as_new) {
            (Some(p), false) => p.clone(),
            _ => {
                let Some(p) = rfd::FileDialog::new()
                    .add_filter("Basset document", &[file::EXTENSION])
                    .set_file_name(format!("{}.{}", self.doc.name, file::EXTENSION))
                    .save_file()
                else {
                    return;
                };
                with_extension(p, file::EXTENSION)
            }
        };
        match file::save(&path, &self.doc) {
            Ok(()) => {
                self.doc.name = path
                    .file_stem()
                    .map(|s| s.to_string_lossy().into_owned())
                    .unwrap_or_else(|| self.doc.name.clone());
                self.path = Some(path.clone());
                self.title_dirty = true;
                self.set_status(format!("Saved {}", path.display()));
            }
            Err(e) => self.report_error(format!("could not save: {e}")),
        }
    }

    pub fn export_stl(&mut self) {
        self.export("stl");
    }

    pub fn export_3mf(&mut self) {
        self.export("3mf");
    }

    /// Exports the selected bodies, or every visible body when nothing is selected, so
    /// "export the whole component" is the zero-click default.
    fn export(&mut self, ext: &str) {
        let items = self.export_items();
        if items.is_empty() {
            self.report_error("nothing to export: create or show a body first");
            return;
        }
        let Some(path) = rfd::FileDialog::new()
            .add_filter(ext.to_uppercase(), &[ext])
            .set_file_name(format!("{}.{ext}", self.doc.name))
            .save_file()
        else {
            return;
        };
        let path = with_extension(path, ext);
        let result = match ext {
            "stl" => basset_io::convenience::export_stl_file(&path, &items),
            _ => basset_io::convenience::export_3mf_file(&path, &items, Unit::Millimeter),
        };
        match result {
            Ok(()) => self.set_status(format!(
                "Exported {} bod{} to {}",
                items.len(),
                if items.len() == 1 { "y" } else { "ies" },
                path.display()
            )),
            Err(e) => self.report_error(format!("export failed: {e}")),
        }
    }

    fn export_items(&mut self) -> Vec<ExportItem> {
        let selected = self.selection.bodies.clone();
        let hidden = self.hidden_bodies.clone();
        let state = self.doc.state();
        state
            .bodies
            .values()
            .filter(|b| {
                if selected.is_empty() {
                    !hidden.contains(&b.id)
                } else {
                    selected.contains(&b.id)
                }
            })
            .map(|b| ExportItem::new(b.name.clone(), b.solid.tessellate().mesh))
            .collect()
    }

    pub fn quit(&mut self) {
        self.exit = true;
    }
}

fn with_extension(path: PathBuf, ext: &str) -> PathBuf {
    if path
        .extension()
        .is_some_and(|e| e.eq_ignore_ascii_case(ext))
    {
        path
    } else {
        Path::new(&path).with_extension(ext)
    }
}
