//! A Model Context Protocol server for Basset.
//!
//! The server holds one [`Session`]: a document and the path it was opened from. Tools
//! build the document the way the application's dialogs do — a sketch is a timeline
//! feature holding a `Sketch`, an extrude names regions of it by sample points — so a
//! model made over the protocol is the same `.bass` file the editor saves, and anything
//! the regenerator reports (a failed feature, an open shell, a loose sketch) comes back
//! to the caller as data rather than as a yellow badge.
//!
//! [`protocol`] is the JSON-RPC framing, [`tools`] the catalogue and dispatch, [`sketch`]
//! the batch of drawing operations one `sketch_ops` call applies, [`summary`] the JSON
//! view of features, bodies, faces and edges, and [`ids`] how all of those are named on
//! the wire.

pub mod args;
pub mod ids;
pub mod protocol;
pub mod sketch;
pub mod summary;
pub mod tools;

use std::path::PathBuf;

use basset_core::Document;
use serde_json::Value;

/// Anything a tool call can refuse. One string, because the caller is an agent reading
/// text: what matters is that the message says what was wrong and what to do instead.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
#[error("{0}")]
pub struct ToolError(pub String);

impl ToolError {
    pub fn bad(message: impl Into<String>) -> Self {
        Self(message.into())
    }
}

macro_rules! from_error {
    ($($t:ty),* $(,)?) => {
        $(impl From<$t> for ToolError {
            fn from(e: $t) -> Self {
                ToolError(e.to_string())
            }
        })*
    };
}

from_error!(
    basset_core::DocumentError,
    basset_core::file::FileError,
    basset_sketch::SketchError,
    basset_sketch::SolveError,
    basset_kernel::KernelError,
    basset_io::IoError,
    basset_fea::FeaError,
    std::io::Error,
    serde_json::Error,
);

/// One open document and where it came from.
pub struct Session {
    doc: Document,
    path: Option<PathBuf>,
}

impl Default for Session {
    fn default() -> Self {
        Self::new()
    }
}

impl Session {
    pub fn new() -> Self {
        Self {
            doc: Document::new("Untitled"),
            path: None,
        }
    }

    pub fn document(&self) -> &Document {
        &self.doc
    }

    pub fn document_mut(&mut self) -> &mut Document {
        &mut self.doc
    }

    pub fn path(&self) -> Option<&PathBuf> {
        self.path.as_ref()
    }

    pub fn replace_document(&mut self, doc: Document, path: Option<PathBuf>) {
        self.doc = doc;
        self.path = path;
    }

    /// Runs one tool by name. The same entry point `tools/call` uses, so a test can drive
    /// the server without the JSON-RPC framing.
    pub fn call(&mut self, name: &str, args: &Value) -> Result<Value, ToolError> {
        tools::call(self, name, args)
    }
}
