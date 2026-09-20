//! 3MF export and a minimal import.
//!
//! A 3MF file is an OPC (zip) package with three required parts:
//!
//! * `[Content_Types].xml` — maps extensions to media types,
//! * `_rels/.rels` — points the package root at the model part,
//! * `3D/3dmodel.model` — the XML model: resources (objects with meshes) and a build
//!   (placed instances of those objects).
//!
//! Unlike STL, 3MF keeps every body as a named object in its own frame and records the
//! placement on the build item, so nothing is baked into the vertices.

use std::collections::HashMap;
use std::io::{Read, Seek, Write};

use basset_math::{Affine3, TriMesh, Vec3};
use zip::write::SimpleFileOptions;
use zip::{CompressionMethod, ZipArchive, ZipWriter};

use crate::xml::{TagKind, TagScanner, escape};
use crate::{ExportItem, IoError, Unit, validate_items};

const MODEL_PART: &str = "3D/3dmodel.model";
const RELS_PART: &str = "_rels/.rels";
const CONTENT_TYPES_PART: &str = "[Content_Types].xml";

const CORE_NAMESPACE: &str = "http://schemas.microsoft.com/3dmanufacturing/core/2015/02";
const MODEL_RELATIONSHIP: &str = "http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel";

/// Writes a complete 3MF package. `unit` is recorded verbatim; coordinates are not
/// rescaled, so pass the unit the meshes are actually expressed in.
pub fn write<W: Write + Seek>(w: W, items: &[ExportItem], unit: Unit) -> Result<(), IoError> {
    validate_items(items)?;

    let mut zip = ZipWriter::new(w);
    let options = SimpleFileOptions::default().compression_method(CompressionMethod::Deflated);

    zip.start_file(CONTENT_TYPES_PART, options)?;
    zip.write_all(content_types_xml().as_bytes())?;

    zip.start_file(RELS_PART, options)?;
    zip.write_all(rels_xml().as_bytes())?;

    zip.start_file(MODEL_PART, options)?;
    zip.write_all(model_xml(items, unit).as_bytes())?;

    zip.finish()?.flush()?;
    Ok(())
}

fn content_types_xml() -> String {
    concat!(
        "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n",
        "<Types xmlns=\"http://schemas.openxmlformats.org/package/2006/content-types\">\n",
        "  <Default Extension=\"rels\" ContentType=\"application/vnd.openxmlformats-package.relationships+xml\"/>\n",
        "  <Default Extension=\"model\" ContentType=\"application/vnd.ms-package.3dmanufacturing-3dmodel+xml\"/>\n",
        "</Types>\n",
    )
    .to_string()
}

fn rels_xml() -> String {
    format!(
        concat!(
            "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n",
            "<Relationships xmlns=\"http://schemas.openxmlformats.org/package/2006/relationships\">\n",
            "  <Relationship Target=\"/{model}\" Id=\"rel0\" Type=\"{rel}\"/>\n",
            "</Relationships>\n",
        ),
        model = MODEL_PART,
        rel = MODEL_RELATIONSHIP,
    )
}

fn model_xml(items: &[ExportItem], unit: Unit) -> String {
    let mut xml = String::new();
    xml.push_str("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    xml.push_str(&format!(
        "<model unit=\"{}\" xml:lang=\"en-US\" xmlns=\"{CORE_NAMESPACE}\">\n",
        unit.as_3mf_str()
    ));
    xml.push_str("  <metadata name=\"Application\">basset670</metadata>\n");

    // Object ids must be positive; one id per item keeps build items trivially mapped.
    let object_id = |index: usize| index + 1;

    xml.push_str("  <resources>\n");
    for (index, item) in items.iter().enumerate() {
        let indexed = IndexedMesh::dedup(&item.mesh, &item.name);
        xml.push_str(&format!(
            "    <object id=\"{}\" type=\"model\" name=\"{}\">\n",
            object_id(index),
            escape(&item.name)
        ));
        xml.push_str("      <mesh>\n        <vertices>\n");
        for p in &indexed.positions {
            xml.push_str(&format!(
                "          <vertex x=\"{}\" y=\"{}\" z=\"{}\"/>\n",
                p.x, p.y, p.z
            ));
        }
        xml.push_str("        </vertices>\n        <triangles>\n");
        for t in indexed.triangles.as_chunks::<3>().0 {
            xml.push_str(&format!(
                "          <triangle v1=\"{}\" v2=\"{}\" v3=\"{}\"/>\n",
                t[0], t[1], t[2]
            ));
        }
        xml.push_str("        </triangles>\n      </mesh>\n    </object>\n");
    }
    xml.push_str("  </resources>\n");

    xml.push_str("  <build>\n");
    for (index, item) in items.iter().enumerate() {
        xml.push_str(&format!(
            "    <item objectid=\"{}\" transform=\"{}\"/>\n",
            object_id(index),
            format_transform(&item.transform)
        ));
    }
    xml.push_str("  </build>\n</model>\n");
    xml
}

/// Serialises an affine transform in 3MF's `transform` attribute layout.
///
/// The core spec defines the 12 numbers as the first three columns of a 4x4 matrix
/// written row by row: `m00 m01 m02 m10 m11 m12 m20 m21 m22 m30 m31 m32`, with the
/// implicit fourth column `0 0 0 1`. The matrix acts on *row* vectors
/// (`[x y z 1] * M`), so `m30 m31 m32` is the translation and each of the first three
/// rows is where a basis vector lands. glam's `DAffine3` acts on column vectors and
/// stores the same information as columns: `x_axis` is where X lands. Hence 3MF row `i`
/// is exactly glam column `i`, and no transposition arithmetic is needed beyond
/// choosing the right accessor.
fn format_transform(t: &Affine3) -> String {
    let rows = [
        t.matrix3.x_axis,
        t.matrix3.y_axis,
        t.matrix3.z_axis,
        t.translation,
    ];
    rows.iter()
        .flat_map(|r| [r.x, r.y, r.z])
        .map(|v| v.to_string())
        .collect::<Vec<_>>()
        .join(" ")
}

fn parse_transform(s: &str) -> Result<Affine3, IoError> {
    let nums: Vec<f64> = s
        .split_ascii_whitespace()
        .map(|tok| {
            tok.parse::<f64>()
                .map_err(|_| malformed(format!("bad transform number '{tok}'")))
        })
        .collect::<Result<_, _>>()?;
    let [m00, m01, m02, m10, m11, m12, m20, m21, m22, m30, m31, m32] = nums[..] else {
        return Err(malformed(format!(
            "transform needs 12 numbers, found {}",
            nums.len()
        )));
    };
    Ok(Affine3::from_cols(
        Vec3::new(m00, m01, m02),
        Vec3::new(m10, m11, m12),
        Vec3::new(m20, m21, m22),
        Vec3::new(m30, m31, m32),
    ))
}

/// A mesh with coincident vertices merged. The kernel emits flat-shaded meshes with
/// three private vertices per triangle; slicers want shared vertices so they can walk
/// edges and detect holes, and the file shrinks by roughly 4x.
struct IndexedMesh {
    positions: Vec<Vec3>,
    triangles: Vec<u32>,
}

impl IndexedMesh {
    fn dedup(mesh: &TriMesh, name: &str) -> Self {
        let mut positions = Vec::new();
        let mut triangles = Vec::with_capacity(mesh.indices.len());
        let mut lookup: HashMap<[u64; 3], u32> = HashMap::new();
        let mut dropped = 0usize;

        for i in 0..mesh.triangle_count() {
            let tri = mesh.triangle(i);
            let mut ids = [0u32; 3];
            for (id, p) in ids.iter_mut().zip(tri) {
                *id = *lookup.entry(position_key(p)).or_insert_with(|| {
                    positions.push(p);
                    (positions.len() - 1) as u32
                });
            }
            // A triangle with a repeated index has zero area; the spec forbids it and it
            // carries no information, so it is dropped rather than failing the export.
            if ids[0] == ids[1] || ids[1] == ids[2] || ids[0] == ids[2] {
                dropped += 1;
                continue;
            }
            triangles.extend_from_slice(&ids);
        }
        if dropped > 0 {
            log::warn!("3MF export of '{name}': dropped {dropped} degenerate triangle(s)");
        }
        Self {
            positions,
            triangles,
        }
    }
}

/// Exact-match key. `-0.0` is folded into `0.0` so a vertex on a mirror plane does not
/// split in two depending on which operation produced it.
fn position_key(p: Vec3) -> [u64; 3] {
    let norm = |v: f64| if v == 0.0 { 0.0f64 } else { v };
    [
        norm(p.x).to_bits(),
        norm(p.y).to_bits(),
        norm(p.z).to_bits(),
    ]
}

/// Reads the objects and build items of a 3MF package back into export items.
///
/// Covers the core mesh subset we write: objects with inline meshes and build items
/// referencing them. Components, materials, and extensions are ignored, which is enough
/// for round-trip testing and for importing files produced by common CAD tools.
pub fn read<R: Read + Seek>(r: R) -> Result<Vec<ExportItem>, IoError> {
    let mut archive = ZipArchive::new(r)?;
    let model_path = model_part_path(&mut archive)?;
    let model_xml = read_part(&mut archive, &model_path)?;
    parse_model(&model_xml)
}

/// The model part's location comes from the root relationships file; falling back to
/// the conventional path keeps slightly non-conforming packages readable.
fn model_part_path<R: Read + Seek>(archive: &mut ZipArchive<R>) -> Result<String, IoError> {
    let Ok(rels) = read_part(archive, RELS_PART) else {
        return Ok(MODEL_PART.to_string());
    };
    let mut scanner = TagScanner::new(&rels);
    while let Some(tag) = scanner.next_tag()? {
        if tag.name == "Relationship" && tag.attr("Type") == Some(MODEL_RELATIONSHIP) {
            let target = tag.required("Target")?;
            return Ok(target.trim_start_matches('/').to_string());
        }
    }
    Ok(MODEL_PART.to_string())
}

fn read_part<R: Read + Seek>(archive: &mut ZipArchive<R>, name: &str) -> Result<String, IoError> {
    let mut text = String::new();
    archive.by_name(name)?.read_to_string(&mut text)?;
    Ok(text)
}

/// An object under construction while its tags stream past.
struct PendingObject {
    id: u32,
    name: String,
    mesh: TriMesh,
}

fn parse_model(xml: &str) -> Result<Vec<ExportItem>, IoError> {
    let mut objects: HashMap<u32, (String, TriMesh)> = HashMap::new();
    let mut items = Vec::new();
    let mut current: Option<PendingObject> = None;
    let mut scanner = TagScanner::new(xml);

    while let Some(tag) = scanner.next_tag()? {
        match (tag.name, tag.kind) {
            ("object", TagKind::Open) => {
                let id = tag.parse_attr("id")?;
                let name = tag.attr("name").unwrap_or("").to_string();
                current = Some(PendingObject {
                    id,
                    name,
                    mesh: TriMesh::default(),
                });
            }
            ("object", TagKind::Close) => {
                let obj = current
                    .take()
                    .ok_or_else(|| malformed("</object> without <object>"))?;
                if obj.mesh.is_empty() {
                    log::warn!(
                        "3MF import: object {} ('{}') has no mesh and is skipped",
                        obj.id,
                        obj.name
                    );
                    continue;
                }
                finish_mesh(&mut objects, obj)?;
            }
            ("vertex", _) => {
                let obj = current
                    .as_mut()
                    .ok_or_else(|| malformed("<vertex> outside <object>"))?;
                let p = Vec3::new(
                    tag.parse_attr("x")?,
                    tag.parse_attr("y")?,
                    tag.parse_attr("z")?,
                );
                obj.mesh.positions.push(p);
            }
            ("triangle", _) => {
                let obj = current
                    .as_mut()
                    .ok_or_else(|| malformed("<triangle> outside <object>"))?;
                let tri = [
                    tag.parse_attr("v1")?,
                    tag.parse_attr("v2")?,
                    tag.parse_attr("v3")?,
                ];
                obj.mesh.indices.extend_from_slice(&tri);
                obj.mesh.face_ids.push(0);
            }
            ("item", _) => {
                let object_id: u32 = tag.parse_attr("objectid")?;
                let (name, mesh) = objects.get(&object_id).ok_or_else(|| {
                    malformed(format!("build item references unknown object {object_id}"))
                })?;
                let transform = match tag.attr("transform") {
                    Some(t) => parse_transform(t)?,
                    None => Affine3::IDENTITY,
                };
                items.push(ExportItem {
                    name: name.clone(),
                    mesh: mesh.clone(),
                    transform,
                });
            }
            _ => {}
        }
    }
    Ok(items)
}

/// Validates indices and derives per-vertex normals, which the file does not store.
/// Shared vertices get area-weighted averages, so a read-back mesh shades smoothly
/// rather than flat; for the round trips this reader exists for, only positions matter.
fn finish_mesh(
    objects: &mut HashMap<u32, (String, TriMesh)>,
    obj: PendingObject,
) -> Result<(), IoError> {
    let PendingObject { id, name, mut mesh } = obj;
    let count = mesh.positions.len();
    if let Some(&index) = mesh.indices.iter().find(|&&i| i as usize >= count) {
        return Err(IoError::IndexOutOfRange { name, index, count });
    }
    mesh.normals = vec![Vec3::ZERO; count];
    for i in 0..mesh.triangle_count() {
        let [a, b, c] = mesh.triangle(i);
        let weighted = (b - a).cross(c - a);
        for k in 0..3 {
            mesh.normals[mesh.indices[3 * i + k] as usize] += weighted;
        }
    }
    for n in &mut mesh.normals {
        *n = n.normalize_or_zero();
    }
    if objects.insert(id, (name, mesh)).is_some() {
        return Err(malformed(format!("duplicate object id {id}")));
    }
    Ok(())
}

fn malformed(msg: impl Into<String>) -> IoError {
    IoError::Malformed3mf(msg.into())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_util::unit_cube;
    use approx::assert_relative_eq;
    use std::io::Cursor;

    fn write_to_vec(items: &[ExportItem], unit: Unit) -> Vec<u8> {
        let mut cursor = Cursor::new(Vec::new());
        write(&mut cursor, items, unit).unwrap();
        cursor.into_inner()
    }

    fn part(bytes: &[u8], name: &str) -> String {
        let mut archive = ZipArchive::new(Cursor::new(bytes)).unwrap();
        read_part(&mut archive, name).unwrap()
    }

    #[test]
    fn package_contains_required_parts() {
        let bytes = write_to_vec(&[ExportItem::new("cube", unit_cube())], Unit::Millimeter);
        let archive = ZipArchive::new(Cursor::new(&bytes)).unwrap();
        let names: Vec<&str> = archive.file_names().collect();
        for required in [CONTENT_TYPES_PART, RELS_PART, MODEL_PART] {
            assert!(names.contains(&required), "missing {required} in {names:?}");
        }
        let rels = part(&bytes, RELS_PART);
        assert!(rels.contains(MODEL_RELATIONSHIP));
        assert!(rels.contains("Target=\"/3D/3dmodel.model\""));
        let types = part(&bytes, CONTENT_TYPES_PART);
        assert!(types.contains("Extension=\"model\""));
        assert!(types.contains("Extension=\"rels\""));
    }

    #[test]
    fn model_has_namespace_unit_and_language() {
        let bytes = write_to_vec(&[ExportItem::new("cube", unit_cube())], Unit::Inch);
        let model = part(&bytes, MODEL_PART);
        assert!(model.contains(&format!("xmlns=\"{CORE_NAMESPACE}\"")));
        assert!(model.contains("unit=\"inch\""));
        assert!(model.contains("xml:lang=\"en-US\""));
        assert!(model.contains("<object id=\"1\" type=\"model\" name=\"cube\">"));
        assert!(model.contains("<item objectid=\"1\" transform=\"1 0 0 0 1 0 0 0 1 0 0 0\"/>"));
    }

    #[test]
    fn vertices_are_deduplicated() {
        let bytes = write_to_vec(&[ExportItem::new("cube", unit_cube())], Unit::Millimeter);
        let model = part(&bytes, MODEL_PART);
        assert_eq!(model.matches("<vertex ").count(), 8);
        assert_eq!(model.matches("<triangle ").count(), 12);
    }

    #[test]
    fn negative_zero_merges_with_zero() {
        let mut mesh = unit_cube();
        mesh.positions[0] = Vec3::new(-0.0, 0.0, -0.0);
        let indexed = IndexedMesh::dedup(&mesh, "cube");
        assert_eq!(indexed.positions.len(), 8);
    }

    #[test]
    fn degenerate_triangles_are_dropped() {
        let mut mesh = unit_cube();
        mesh.push_triangle([Vec3::ZERO, Vec3::ZERO, Vec3::X], 0);
        let indexed = IndexedMesh::dedup(&mesh, "cube");
        assert_eq!(indexed.triangles.len() / 3, 12);
    }

    #[test]
    fn round_trip_preserves_names_volume_and_transform() {
        let shift = Vec3::new(3.0, 4.0, 5.0);
        let items = [
            ExportItem::new("first", unit_cube()),
            ExportItem::new("second", unit_cube()).with_transform(Affine3::from_translation(shift)),
        ];
        let bytes = write_to_vec(&items, Unit::Millimeter);
        let back = read(Cursor::new(bytes)).unwrap();
        assert_eq!(back.len(), 2);
        assert_eq!(back[0].name, "first");
        assert_eq!(back[1].name, "second");
        for item in &back {
            assert_eq!(item.mesh.positions.len(), 8);
            assert_eq!(item.mesh.triangle_count(), 12);
            assert_relative_eq!(item.mesh.signed_volume(), 1.0, epsilon = 1e-12);
            assert_eq!(item.mesh.normals.len(), 8);
        }
        assert_relative_eq!(back[1].transform.translation.x, shift.x);
        let world = back[1].world_mesh();
        assert_relative_eq!(world.aabb().min.y, shift.y);
        assert_relative_eq!(world.signed_volume(), 1.0, epsilon = 1e-12);
    }

    #[test]
    fn rotation_transform_round_trips_exactly() {
        let t = Affine3::from_rotation_translation(
            basset_math::Quat::from_axis_angle(Vec3::new(1.0, 2.0, 3.0).normalize(), 0.7),
            Vec3::new(-1.5, 2.25, 8.0),
        );
        let parsed = parse_transform(&format_transform(&t)).unwrap();
        assert!(parsed.abs_diff_eq(t, 1e-15));
        assert!(parse_transform("1 2 3").is_err());
        assert!(parse_transform("1 0 0 0 1 0 0 0 1 0 0 x").is_err());
    }

    #[test]
    fn names_are_escaped_and_restored() {
        let name = "bracket <v2> & \"final\"";
        let bytes = write_to_vec(&[ExportItem::new(name, unit_cube())], Unit::Millimeter);
        let model = part(&bytes, MODEL_PART);
        assert!(model.contains("name=\"bracket &lt;v2&gt; &amp; &quot;final&quot;\""));
        let back = read(Cursor::new(bytes)).unwrap();
        assert_eq!(back[0].name, name);
    }

    #[test]
    fn empty_item_list_is_a_typed_error() {
        let mut cursor = Cursor::new(Vec::new());
        assert!(matches!(
            write(&mut cursor, &[], Unit::Millimeter),
            Err(IoError::NoItems)
        ));
        assert!(cursor.into_inner().is_empty());
    }

    #[test]
    fn reader_falls_back_to_default_model_path_without_rels() {
        let mut cursor = Cursor::new(Vec::new());
        {
            let mut zip = ZipWriter::new(&mut cursor);
            zip.start_file(MODEL_PART, SimpleFileOptions::default())
                .unwrap();
            zip.write_all(model_xml(&[ExportItem::new("c", unit_cube())], Unit::Meter).as_bytes())
                .unwrap();
            zip.finish().unwrap();
        }
        let back = read(Cursor::new(cursor.into_inner())).unwrap();
        assert_eq!(back.len(), 1);
    }

    #[test]
    fn reader_rejects_bad_references() {
        let bad_item = format!(
            "<model xmlns=\"{CORE_NAMESPACE}\"><resources></resources><build><item objectid=\"7\"/></build></model>"
        );
        assert!(matches!(
            parse_model(&bad_item),
            Err(IoError::Malformed3mf(_))
        ));

        let bad_index = "<model><resources><object id=\"1\"><mesh><vertices><vertex x=\"0\" y=\"0\" z=\"0\"/>\
            </vertices><triangles><triangle v1=\"0\" v2=\"1\" v3=\"2\"/></triangles></mesh></object></resources></model>";
        assert!(matches!(
            parse_model(bad_index),
            Err(IoError::IndexOutOfRange {
                index: 1,
                count: 1,
                ..
            })
        ));

        assert!(matches!(
            read(Cursor::new(b"not a zip")),
            Err(IoError::Zip(_))
        ));
    }
}
