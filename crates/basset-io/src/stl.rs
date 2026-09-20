//! STL export and import.
//!
//! STL is a bag of independent triangles with no objects, transforms, or units. Export
//! therefore bakes each item's transform into its vertices, and binary export merges
//! every item into one solid because the binary layout has no room for names. ASCII
//! export keeps one `solid` block per item; most readers merge them anyway but the names
//! survive for anyone reading the file.
//!
//! Coordinates are written in model units (millimetres), the de facto assumption of
//! every slicer.

use std::io::{BufWriter, Read, Write};

use basset_math::{TriMesh, Vec3};

use crate::{ExportItem, IoError, validate_items};

/// Fixed by the format: 80-byte header, u32 triangle count, then 50 bytes per triangle.
const HEADER_LEN: usize = 80;
const TRIANGLE_LEN: usize = 50;
const HEADER_TEXT: &[u8] = b"basset670 binary STL";

/// Writes every item as one merged binary STL.
///
/// Facet normals are recomputed from the winding rather than copied from the mesh so
/// the file is self-consistent even if a caller hands over stale normals.
pub fn write_binary<W: Write>(w: W, items: &[ExportItem]) -> Result<(), IoError> {
    validate_items(items)?;
    let mut w = BufWriter::new(w);

    let mut header = [0u8; HEADER_LEN];
    header[..HEADER_TEXT.len()].copy_from_slice(HEADER_TEXT);
    w.write_all(&header)?;

    let total: usize = items.iter().map(|i| i.mesh.triangle_count()).sum();
    let count = u32::try_from(total).map_err(|_| {
        IoError::MalformedStl(format!("{total} triangles exceed the binary STL limit"))
    })?;
    w.write_all(&count.to_le_bytes())?;

    for item in items {
        let mesh = item.world_mesh();
        for i in 0..mesh.triangle_count() {
            let tri = mesh.triangle(i);
            write_vec3_f32(&mut w, facet_normal(&tri))?;
            for p in tri {
                write_vec3_f32(&mut w, p)?;
            }
            // Attribute byte count; some tools abuse it for colour, we leave it zero.
            w.write_all(&0u16.to_le_bytes())?;
        }
    }
    w.flush()?;
    Ok(())
}

/// Writes one `solid <name>` block per item.
pub fn write_ascii<W: Write>(w: W, items: &[ExportItem]) -> Result<(), IoError> {
    validate_items(items)?;
    let mut w = BufWriter::new(w);
    for item in items {
        let name = ascii_solid_name(&item.name);
        let mesh = item.world_mesh();
        writeln!(w, "solid {name}")?;
        for i in 0..mesh.triangle_count() {
            let tri = mesh.triangle(i);
            let n = facet_normal(&tri);
            writeln!(w, "  facet normal {} {} {}", n.x, n.y, n.z)?;
            writeln!(w, "    outer loop")?;
            for p in tri {
                writeln!(w, "      vertex {} {} {}", p.x, p.y, p.z)?;
            }
            writeln!(w, "    endloop")?;
            writeln!(w, "  endfacet")?;
        }
        writeln!(w, "endsolid {name}")?;
    }
    w.flush()?;
    Ok(())
}

/// Reads a binary or ASCII STL into a flat-shaded mesh. All `face_ids` are 0 because
/// STL carries no face provenance; the caller can re-derive faces later if it wants.
pub fn read<R: Read>(mut r: R) -> Result<TriMesh, IoError> {
    let mut bytes = Vec::new();
    r.read_to_end(&mut bytes)?;

    // Binary files may legitimately start with "solid" (some exporters put it in the
    // header), so the size check comes first: it is exact for well-formed binaries.
    if let Some(count) = binary_triangle_count(&bytes) {
        return read_binary(&bytes, count);
    }
    if bytes.trim_ascii_start().starts_with(b"solid") {
        let text = str::from_utf8(&bytes)
            .map_err(|e| IoError::MalformedStl(format!("ASCII STL is not UTF-8: {e}")))?;
        return read_ascii(text);
    }
    Err(IoError::MalformedStl(
        "neither a valid binary STL nor an ASCII one starting with 'solid'".into(),
    ))
}

fn binary_triangle_count(bytes: &[u8]) -> Option<usize> {
    let count_bytes: [u8; 4] = bytes.get(HEADER_LEN..HEADER_LEN + 4)?.try_into().ok()?;
    let count = u32::from_le_bytes(count_bytes) as usize;
    (bytes.len() == HEADER_LEN + 4 + count * TRIANGLE_LEN).then_some(count)
}

fn read_binary(bytes: &[u8], count: usize) -> Result<TriMesh, IoError> {
    let mut mesh = TriMesh::default();
    let mut cursor = &bytes[HEADER_LEN + 4..];
    for _ in 0..count {
        let (record, rest) = cursor.split_at(TRIANGLE_LEN);
        cursor = rest;
        // Skip the stored normal (bytes 0..12): we always recompute from winding.
        let mut tri = [Vec3::ZERO; 3];
        for (k, p) in tri.iter_mut().enumerate() {
            *p = read_vec3_f32(&record[12 + k * 12..]);
        }
        mesh.push_triangle(tri, 0);
    }
    Ok(mesh)
}

/// Token-based parse: every `vertex x y z` triple becomes a triangle corner and every
/// third corner closes a triangle. This tolerates the many small deviations found in
/// the wild (missing normals, odd whitespace, solids without names).
fn read_ascii(text: &str) -> Result<TriMesh, IoError> {
    let mut mesh = TriMesh::default();
    let mut tokens = text.split_ascii_whitespace();
    let mut corners: Vec<Vec3> = Vec::with_capacity(3);
    while let Some(tok) = tokens.next() {
        if tok != "vertex" {
            continue;
        }
        let mut p = Vec3::ZERO;
        for coord in [&mut p.x, &mut p.y, &mut p.z] {
            let raw = tokens.next().ok_or_else(|| {
                IoError::MalformedStl("vertex with fewer than 3 coordinates".into())
            })?;
            *coord = raw
                .parse()
                .map_err(|_| IoError::MalformedStl(format!("bad coordinate '{raw}'")))?;
        }
        corners.push(p);
        if corners.len() == 3 {
            mesh.push_triangle([corners[0], corners[1], corners[2]], 0);
            corners.clear();
        }
    }
    if !corners.is_empty() {
        return Err(IoError::MalformedStl(format!(
            "facet with only {} vertices",
            corners.len()
        )));
    }
    Ok(mesh)
}

fn facet_normal(tri: &[Vec3; 3]) -> Vec3 {
    (tri[1] - tri[0]).cross(tri[2] - tri[0]).normalize_or_zero()
}

fn write_vec3_f32<W: Write>(w: &mut W, v: Vec3) -> std::io::Result<()> {
    for c in [v.x, v.y, v.z] {
        w.write_all(&(c as f32).to_le_bytes())?;
    }
    Ok(())
}

fn read_vec3_f32(b: &[u8]) -> Vec3 {
    let f = |i: usize| f32::from_le_bytes([b[i], b[i + 1], b[i + 2], b[i + 3]]) as f64;
    Vec3::new(f(0), f(4), f(8))
}

/// The solid name must stay on one line, and a name that itself starts with
/// "endsolid" or contains line breaks would confuse line-based readers.
fn ascii_solid_name(name: &str) -> String {
    let cleaned: String = name
        .chars()
        .map(|c| if c.is_control() { ' ' } else { c })
        .collect();
    let cleaned = cleaned.trim();
    if cleaned.is_empty() {
        "body".to_string()
    } else {
        cleaned.to_string()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_util::unit_cube;
    use approx::assert_relative_eq;
    use basset_math::Affine3;
    use std::io::Cursor;

    fn cube_item(name: &str) -> ExportItem {
        ExportItem::new(name, unit_cube())
    }

    #[test]
    fn binary_round_trip_preserves_count_and_volume() {
        let mut buf = Vec::new();
        write_binary(&mut buf, &[cube_item("cube")]).unwrap();
        assert_eq!(buf.len(), HEADER_LEN + 4 + 12 * TRIANGLE_LEN);
        assert!(buf.starts_with(HEADER_TEXT));

        let mesh = read(Cursor::new(buf)).unwrap();
        assert_eq!(mesh.triangle_count(), 12);
        assert_relative_eq!(mesh.signed_volume(), 1.0, epsilon = 1e-6);
        assert!(mesh.face_ids.iter().all(|&id| id == 0));
    }

    #[test]
    fn binary_merges_items_into_one_solid() {
        let items = [
            cube_item("a"),
            cube_item("b").with_transform(Affine3::from_translation(Vec3::new(5.0, 0.0, 0.0))),
        ];
        let mut buf = Vec::new();
        write_binary(&mut buf, &items).unwrap();
        let mesh = read(Cursor::new(buf)).unwrap();
        assert_eq!(mesh.triangle_count(), 24);
        assert_relative_eq!(mesh.signed_volume(), 2.0, epsilon = 1e-6);
    }

    #[test]
    fn binary_normals_are_recomputed_from_winding() {
        let mut mesh = unit_cube();
        // Corrupt the stored normals; the file must not care.
        for n in &mut mesh.normals {
            *n = Vec3::new(9.0, 9.0, 9.0);
        }
        let mut buf = Vec::new();
        write_binary(&mut buf, &[ExportItem::new("c", mesh)]).unwrap();
        // First triangle is on the -z face; its normal record follows the header+count.
        let n = read_vec3_f32(&buf[HEADER_LEN + 4..]);
        assert_relative_eq!(n.z, -1.0, epsilon = 1e-6);
        assert_relative_eq!(n.x, 0.0, epsilon = 1e-6);
    }

    #[test]
    fn ascii_round_trip_preserves_volume() {
        let mut buf = Vec::new();
        write_ascii(&mut buf, &[cube_item("my cube")]).unwrap();
        let text = String::from_utf8(buf.clone()).unwrap();
        assert!(text.starts_with("solid my cube\n"));
        assert!(text.trim_end().ends_with("endsolid my cube"));
        assert_eq!(text.matches("facet normal").count(), 12);

        let mesh = read(Cursor::new(buf)).unwrap();
        assert_eq!(mesh.triangle_count(), 12);
        assert_relative_eq!(mesh.signed_volume(), 1.0, epsilon = 1e-12);
    }

    #[test]
    fn ascii_writes_one_solid_per_item() {
        let mut buf = Vec::new();
        write_ascii(&mut buf, &[cube_item("a"), cube_item("b")]).unwrap();
        let text = String::from_utf8(buf.clone()).unwrap();
        assert_eq!(
            text.matches("\nsolid ").count() + usize::from(text.starts_with("solid ")),
            2
        );
        assert_eq!(read(Cursor::new(buf)).unwrap().triangle_count(), 24);
    }

    #[test]
    fn transform_is_applied_on_export() {
        let shift = Vec3::new(10.0, -2.0, 3.5);
        let item = cube_item("moved").with_transform(Affine3::from_translation(shift));
        type Writer = fn(&mut Vec<u8>, &[ExportItem]) -> Result<(), IoError>;
        let writers: [Writer; 2] = [|w, i| write_binary(w, i), |w, i| write_ascii(w, i)];
        for writer in writers {
            let mut buf = Vec::new();
            writer(&mut buf, std::slice::from_ref(&item)).unwrap();
            let aabb = read(Cursor::new(buf)).unwrap().aabb();
            assert_relative_eq!(aabb.min.x, shift.x, epsilon = 1e-6);
            assert_relative_eq!(aabb.min.y, shift.y, epsilon = 1e-6);
            assert_relative_eq!(aabb.max.z, shift.z + 1.0, epsilon = 1e-6);
        }
    }

    #[test]
    fn empty_item_list_is_a_typed_error() {
        let mut buf = Vec::new();
        assert!(matches!(write_binary(&mut buf, &[]), Err(IoError::NoItems)));
        assert!(matches!(write_ascii(&mut buf, &[]), Err(IoError::NoItems)));
        assert!(buf.is_empty(), "nothing should be written on failure");
    }

    #[test]
    fn solid_name_is_sanitised() {
        assert_eq!(ascii_solid_name("two\nlines"), "two lines");
        assert_eq!(ascii_solid_name("   "), "body");
    }

    #[test]
    fn read_rejects_garbage_and_truncated_files() {
        assert!(matches!(
            read(Cursor::new(b"hello")),
            Err(IoError::MalformedStl(_))
        ));
        let mut buf = Vec::new();
        write_binary(&mut buf, &[cube_item("c")]).unwrap();
        buf.truncate(buf.len() - 7);
        assert!(matches!(
            read(Cursor::new(buf)),
            Err(IoError::MalformedStl(_))
        ));
        let partial = "solid x\nvertex 0 0 0\nvertex 1 0 0\nendsolid x\n";
        assert!(matches!(
            read(Cursor::new(partial)),
            Err(IoError::MalformedStl(_))
        ));
    }

    #[test]
    fn read_accepts_binary_whose_header_starts_with_solid() {
        let mut buf = Vec::new();
        write_binary(&mut buf, &[cube_item("c")]).unwrap();
        buf[..5].copy_from_slice(b"solid");
        assert_eq!(read(Cursor::new(buf)).unwrap().triangle_count(), 12);
    }
}
