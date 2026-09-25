//! The `.bass` on-disk format.
//!
//! JSON with an explicit `format_version` so old files can be migrated forward rather
//! than rejected. Only the timeline and metadata are stored; geometry is regenerated on
//! load, which keeps files small and means a kernel improvement automatically applies to
//! existing documents.

use std::io::{Read, Write};
use std::path::Path;

use serde::{Deserialize, Serialize};

use crate::document::Document;

pub const FORMAT_VERSION: u32 = 3;
pub const EXTENSION: &str = "bass";

#[derive(Serialize, Deserialize)]
struct DocumentFile {
    format_version: u32,
    /// Free-form producer string for diagnostics when a file fails to load elsewhere.
    generator: String,
    document: serde_json::Value,
}

#[derive(Debug, thiserror::Error)]
pub enum FileError {
    #[error("io error: {0}")]
    Io(#[from] std::io::Error),
    #[error("malformed document: {0}")]
    Json(#[from] serde_json::Error),
    #[error("file format version {found} is newer than supported version {supported}")]
    UnsupportedVersion { found: u32, supported: u32 },
}

pub fn write<W: Write>(w: W, doc: &Document) -> Result<(), FileError> {
    let file = DocumentFile {
        format_version: FORMAT_VERSION,
        generator: format!("basset {}", env!("CARGO_PKG_VERSION")),
        document: serde_json::to_value(doc)?,
    };
    serde_json::to_writer_pretty(w, &file)?;
    Ok(())
}

pub fn read<R: Read>(r: R) -> Result<Document, FileError> {
    let file: DocumentFile = serde_json::from_reader(r)?;
    if file.format_version > FORMAT_VERSION {
        return Err(FileError::UnsupportedVersion {
            found: file.format_version,
            supported: FORMAT_VERSION,
        });
    }
    let value = migrate(file.format_version, file.document);
    Ok(serde_json::from_value(value)?)
}

pub fn save(path: impl AsRef<Path>, doc: &Document) -> Result<(), FileError> {
    let path = path.as_ref();
    // Write to a sibling temp file and rename so a crash mid-write never destroys the
    // previous good copy.
    let tmp = path.with_extension(format!("{EXTENSION}.tmp"));
    write(std::io::BufWriter::new(std::fs::File::create(&tmp)?), doc)?;
    std::fs::rename(&tmp, path)?;
    Ok(())
}

pub fn load(path: impl AsRef<Path>) -> Result<Document, FileError> {
    read(std::io::BufReader::new(std::fs::File::open(path)?))
}

/// Applies in-order migrations from `from` to `FORMAT_VERSION`. The JSON value form lets
/// a migration touch fields that no longer exist in the Rust types.
fn migrate(from: u32, value: serde_json::Value) -> serde_json::Value {
    (from..FORMAT_VERSION).fold(value, migrate_step)
}

/// Upgrades a document from `version` to `version + 1`. Each future format version adds
/// one arm here, e.g. `3 => migrate_v3_to_v4(value)`.
fn migrate_step(mut value: serde_json::Value, version: u32) -> serde_json::Value {
    match version {
        1 => {
            migrate_v1_profiles_to_regions(&mut value);
            value
        }
        2 => migrate_v2_parameters(value),
        _ => value,
    }
}

/// Version 3 added document parameters and the expressions that drive feature values.
///
/// There is nothing to do: both are new fields, both are `serde(default)`, so a version 2
/// document loads with an empty table and no driven values — which is exactly what it
/// had. The version was bumped all the same, because migration is only half of what it is
/// for. A file *written* now can carry parameters, and an older build reading it would
/// drop them silently, saving a document whose extrude distances no longer say where they
/// came from. Refusing to open it is much better than that, and the existing
/// [`FileError::UnsupportedVersion`] path already refuses anything newer than it knows.
fn migrate_v2_parameters(value: serde_json::Value) -> serde_json::Value {
    value
}

/// Version 2 let the generators take a planar body face as well as a sketch region, so
/// their `profiles` list became a `regions` list of a two-variant enum. Every version 1
/// entry was a sketch region, which is the `Profile` variant.
fn migrate_v1_profiles_to_regions(value: &mut serde_json::Value) {
    const GENERATORS: [&str; 4] = ["Extrude", "Revolve", "Sweep", "Loft"];
    match value {
        serde_json::Value::Array(items) => {
            for item in items {
                migrate_v1_profiles_to_regions(item);
            }
        }
        serde_json::Value::Object(map) => {
            for name in GENERATORS {
                let Some(serde_json::Value::Object(body)) = map.get_mut(name) else {
                    continue;
                };
                let Some(serde_json::Value::Array(profiles)) = body.remove("profiles") else {
                    continue;
                };
                let regions = profiles
                    .into_iter()
                    .map(|p| serde_json::json!({ "Profile": p }))
                    .collect();
                body.insert("regions".into(), serde_json::Value::Array(regions));
            }
            for (_, v) in map.iter_mut() {
                migrate_v1_profiles_to_regions(v);
            }
        }
        _ => {}
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_version_2_document_loads_with_an_empty_parameter_table() {
        let file = serde_json::json!({
            "format_version": 2,
            "generator": "basset test",
            "document": {
                "name": "old",
                "units": "Millimeters",
                "timeline": { "features": [], "cursor": 0, "next_id": 1 },
            }
        });
        let text = serde_json::to_string(&file).unwrap();
        let doc = read(text.as_bytes()).expect("a version 2 file still loads");
        assert_eq!(doc.name, "old");
        assert!(doc.parameters().is_empty());
    }

    #[test]
    fn a_file_newer_than_this_build_is_refused_rather_than_read_in_part() {
        let file = serde_json::json!({
            "format_version": FORMAT_VERSION + 1,
            "generator": "basset from the future",
            "document": {},
        });
        let text = serde_json::to_string(&file).unwrap();
        assert!(matches!(
            read(text.as_bytes()),
            Err(FileError::UnsupportedVersion { .. })
        ));
    }

    #[test]
    fn v1_profiles_become_sketch_regions() {
        let mut doc = serde_json::json!({
            "timeline": { "features": [
                { "id": 1, "kind": { "Sketch": { "plane": { "Origin": "XY" } } } },
                { "id": 2, "kind": { "Extrude": {
                    "profiles": [{ "sketch": 1, "sample": [1.0, 2.0] }],
                    "extent": { "OneSide": 5.0 }
                } } },
            ] }
        });
        migrate_v1_profiles_to_regions(&mut doc);
        let extrude = &doc["timeline"]["features"][1]["kind"]["Extrude"];
        assert!(extrude.get("profiles").is_none());
        assert_eq!(
            extrude["regions"],
            serde_json::json!([{ "Profile": { "sketch": 1, "sample": [1.0, 2.0] } }])
        );
        // Untouched fields survive, and a second pass is a no-op.
        assert_eq!(extrude["extent"], serde_json::json!({ "OneSide": 5.0 }));
        let once = doc.clone();
        migrate_v1_profiles_to_regions(&mut doc);
        assert_eq!(doc, once);
    }
}
