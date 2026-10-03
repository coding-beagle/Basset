//! Document files and mesh export through native dialogs.

use std::path::{Path, PathBuf};

use basset_core::{BodyRef, ComponentId, Visibility, file};
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
        self.apply_visibility(&Visibility::default());
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
                let visibility = self.doc.visibility().clone();
                self.apply_visibility(&visibility);
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
        self.doc.set_visibility(self.visibility());
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

    /// What is hidden, in the form the document saves. The editor keeps its own working
    /// copy because every frame reads it and the panels toggle it in place; the document's
    /// copy is brought up to date on save and read back on open, which are the only two
    /// moments the file sees it.
    pub fn visibility(&self) -> Visibility {
        Visibility {
            hidden_bodies: self.hidden_bodies.iter().copied().collect(),
            hidden_sketches: self.hidden_sketches.iter().copied().collect(),
            show_origin: self.show_origin,
            show_grid: self.show_grid,
            show_axes: self.show_axes,
        }
    }

    fn apply_visibility(&mut self, visibility: &Visibility) {
        self.hidden_bodies = visibility.hidden_bodies.iter().copied().collect();
        self.hidden_sketches = visibility.hidden_sketches.iter().copied().collect();
        self.show_origin = visibility.show_origin;
        self.show_grid = visibility.show_grid;
        self.show_axes = visibility.show_axes;
    }

    pub fn export_stl(&mut self) {
        self.export(MeshFormat::Stl);
    }

    pub fn export_3mf(&mut self) {
        self.export(MeshFormat::ThreeMf);
    }

    /// Exports the selected bodies, or every visible body when nothing is selected, so
    /// "export the whole component" is the zero-click default.
    fn export(&mut self, format: MeshFormat) {
        let bodies = if self.selection.bodies.is_empty() {
            self.cached_bodies
                .iter()
                .map(|(b, _)| *b)
                .filter(|b| !self.hidden_bodies.contains(b))
                .collect()
        } else {
            self.selection.bodies.clone()
        };
        let name = self.doc.name.clone();
        self.export_bodies(&bodies, format, &name);
    }

    /// Exports the visible bodies of a component and of the components inside it, into
    /// a file named after it: what the component's row in the browser offers.
    pub(crate) fn export_component(&mut self, id: ComponentId, format: MeshFormat) {
        let bodies = self.component_export_bodies(id);
        let name = self
            .cached_components
            .iter()
            .find(|(c, _, _)| *c == id)
            .map_or_else(|| self.doc.name.clone(), |(_, n, _)| n.clone());
        self.export_bodies(&bodies, format, &name);
    }

    /// The visible bodies of a component and of every component under it.
    pub(crate) fn component_export_bodies(&self, id: ComponentId) -> Vec<BodyRef> {
        let mut tree = vec![id];
        let mut i = 0;
        while let Some(&c) = tree.get(i) {
            tree.extend(
                self.cached_components
                    .iter()
                    .filter(|(_, _, p)| *p == Some(c))
                    .map(|(child, _, _)| *child),
            );
            i += 1;
        }
        self.cached_bodies
            .iter()
            .map(|(b, _)| *b)
            .filter(|b| {
                !self.hidden_bodies.contains(b)
                    && self
                        .cached_body_components
                        .get(b)
                        .is_some_and(|c| tree.contains(c))
            })
            .collect()
    }

    /// Asks where to write `bodies` and writes them there, suggesting `name` for the file.
    pub(crate) fn export_bodies(&mut self, bodies: &[BodyRef], format: MeshFormat, name: &str) {
        if bodies.is_empty() {
            self.report_error("nothing to export: create or show a body first");
            return;
        }
        let ext = format.extension();
        let Some(path) = rfd::FileDialog::new()
            .add_filter(ext.to_uppercase(), &[ext])
            .set_file_name(format!("{name}.{ext}"))
            .save_file()
        else {
            return;
        };
        self.write_export(&with_extension(path, ext), bodies, format);
    }

    /// The half of an export after the dialog, apart so a test can give it a path.
    pub(crate) fn write_export(&mut self, path: &Path, bodies: &[BodyRef], format: MeshFormat) {
        let items = self.export_items(bodies);
        let result = match format {
            MeshFormat::Stl => basset_io::convenience::export_stl_file(path, &items),
            MeshFormat::ThreeMf => {
                basset_io::convenience::export_3mf_file(path, &items, Unit::Millimeter)
            }
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

    fn export_items(&mut self, bodies: &[BodyRef]) -> Vec<ExportItem> {
        let state = self.doc.state();
        state
            .bodies
            .values()
            .filter(|b| bodies.contains(&b.id))
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

/// The mesh formats a body can be exported to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum MeshFormat {
    Stl,
    ThreeMf,
}

impl MeshFormat {
    fn extension(self) -> &'static str {
        match self {
            Self::Stl => "stl",
            Self::ThreeMf => "3mf",
        }
    }

    pub(crate) fn label(self) -> &'static str {
        match self {
            Self::Stl => "Export STL…",
            Self::ThreeMf => "Export 3MF…",
        }
    }
}
