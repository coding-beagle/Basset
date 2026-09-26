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

pub const FORMAT_VERSION: u32 = 5;
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
        3 => migrate_v3_to_face(value),
        4 => {
            migrate_v4_single_targets(&mut value);
            value
        }
        _ => value,
    }
}

/// Version 5 let a boolean feature name several target bodies, so the `Join`, `Cut` and
/// `Intersect` variants of a generator's operation carry a list where they carried one
/// body. Every version 4 operation meant exactly that one body, which is the one-element
/// list. `NewBody` serialises as a bare string and needs nothing.
fn migrate_v4_single_targets(value: &mut serde_json::Value) {
    const GENERATORS: [&str; 4] = ["Extrude", "Revolve", "Sweep", "Loft"];
    const BOOLEANS: [&str; 3] = ["Join", "Cut", "Intersect"];
    match value {
        serde_json::Value::Array(items) => {
            for item in items {
                migrate_v4_single_targets(item);
            }
        }
        serde_json::Value::Object(map) => {
            for name in GENERATORS {
                let Some(serde_json::Value::Object(body)) = map.get_mut(name) else {
                    continue;
                };
                let Some(serde_json::Value::Object(op)) = body.get_mut("operation") else {
                    continue;
                };
                for boolean in BOOLEANS {
                    if let Some(target) = op.get_mut(boolean)
                        && !target.is_array()
                    {
                        *target = serde_json::Value::Array(vec![target.take()]);
                    }
                }
            }
            for (_, v) in map.iter_mut() {
                migrate_v4_single_targets(v);
            }
        }
        _ => {}
    }
}

/// Version 4 added the to-face extrude extent.
///
/// Another additive change, so again there is nothing to rewrite: a version 3 document
/// has no such extent and loads as it is. The bump exists for the same reason version 3's
/// did — this time an older build reading a newer file would not even drop the extent
/// silently, it would refuse the unknown variant with a "malformed document" error, and
/// "this file is newer than your build" is the honest version of that message.
fn migrate_v3_to_face(value: serde_json::Value) -> serde_json::Value {
    value
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
    fn v4_single_targets_become_one_element_lists() {
        let mut doc = serde_json::json!({
            "timeline": { "features": [
                { "id": 2, "kind": { "Extrude": {
                    "regions": [{ "Profile": { "sketch": 1, "sample": [1.0, 1.0] } }],
                    "extent": { "OneSide": 5.0 },
                    "operation": "NewBody"
                } } },
                { "id": 3, "kind": { "Extrude": {
                    "regions": [{ "Profile": { "sketch": 1, "sample": [2.0, 2.0] } }],
                    "extent": { "OneSide": 5.0 },
                    "operation": { "Cut": 2 }
                } } },
                { "id": 4, "kind": { "Revolve": {
                    "regions": [], "axis": { "Origin": "Y" }, "angle": 1.0,
                    "operation": { "Join": 2 }
                } } },
            ] }
        });
        migrate_v4_single_targets(&mut doc);
        let features = &doc["timeline"]["features"];
        assert_eq!(
            features[0]["kind"]["Extrude"]["operation"],
            serde_json::json!("NewBody"),
            "a new body carries no target to wrap"
        );
        assert_eq!(
            features[1]["kind"]["Extrude"]["operation"],
            serde_json::json!({ "Cut": [2] })
        );
        assert_eq!(
            features[2]["kind"]["Revolve"]["operation"],
            serde_json::json!({ "Join": [2] })
        );
        // A second pass is a no-op: an already-listed target is not wrapped again.
        let once = doc.clone();
        migrate_v4_single_targets(&mut doc);
        assert_eq!(doc, once);
    }

    /// The whole journey of an old file: a document is built with today's types, its
    /// JSON is rewritten to the version 4 single-target shape, and loading it back must
    /// regenerate the very same solids the original document held.
    #[test]
    fn a_version_4_file_with_a_single_target_cut_loads_and_regenerates_identically() {
        use basset_math::Vec2;
        use basset_sketch::{Sketch, shapes};

        use crate::feature::{BodyOp, Extent, FeatureKind};
        use crate::ids::ComponentId;
        use crate::refs::{BodyRef, OriginPlane, PlaneRef, ProfileRef, RegionRef};

        let mut doc = Document::new("old cut");
        let mut sketch = Sketch::new();
        shapes::rectangle_two_point(&mut sketch, Vec2::ZERO, Vec2::new(10.0, 10.0));
        shapes::rectangle_two_point(&mut sketch, Vec2::new(4.0, 4.0), Vec2::new(6.0, 6.0));
        let sk = doc.add_feature(FeatureKind::Sketch {
            plane: PlaneRef::Origin(OriginPlane::XY),
            component: ComponentId::ROOT,
            sketch,
        });
        let base = doc.add_feature(FeatureKind::Extrude {
            regions: vec![RegionRef::Profile(ProfileRef {
                sketch: sk,
                sample: Vec2::new(1.0, 1.0),
            })],
            extent: Extent::OneSide(2.0),
            operation: BodyOp::NewBody,
            component: ComponentId::ROOT,
        });
        doc.add_feature(FeatureKind::Extrude {
            regions: vec![RegionRef::Profile(ProfileRef {
                sketch: sk,
                sample: Vec2::new(5.0, 5.0),
            })],
            extent: Extent::OneSide(2.0),
            operation: BodyOp::Cut(vec![BodyRef(base)]),
            component: ComponentId::ROOT,
        });
        let expected = doc.state().body(BodyRef(base)).unwrap().solid.volume();
        assert!(
            (expected - (200.0 - 8.0)).abs() < 1e-9,
            "the document itself cuts the hole: {expected}"
        );

        // Rewrite what `write` produces into the shape a version 4 build saved: the
        // target list back to the bare body, and the version stamp to match.
        let mut bytes = Vec::new();
        write(&mut bytes, &doc).unwrap();
        let mut file: serde_json::Value = serde_json::from_slice(&bytes).unwrap();
        file["format_version"] = serde_json::json!(4);
        let op = &mut file["document"]["timeline"]["features"][2]["kind"]["Extrude"]["operation"];
        assert_eq!(*op, serde_json::json!({ "Cut": [base.0] }));
        *op = serde_json::json!({ "Cut": base.0 });

        let text = serde_json::to_string(&file).unwrap();
        let mut loaded = read(text.as_bytes()).expect("a version 4 file still loads");
        let volume = loaded.state().body(BodyRef(base)).unwrap().solid.volume();
        assert!(
            (volume - expected).abs() < 1e-12,
            "loaded {volume}, built {expected}"
        );

        // And a fresh save of the loaded document round-trips as itself.
        let mut again = Vec::new();
        write(&mut again, &loaded).unwrap();
        let mut reread = read(again.as_slice()).unwrap();
        let volume = reread.state().body(BodyRef(base)).unwrap().solid.volume();
        assert!((volume - expected).abs() < 1e-12);
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
