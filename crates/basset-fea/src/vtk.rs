//! Legacy ASCII VTK output of a study's results, for ParaView and friends.
//!
//! The legacy format is a few hundred bytes of header around plain numbers, which is why
//! it is the one written here: no XML, no compression, no dependency, and every viewer
//! reads it. Displacement is a point vector, von Mises a point scalar (nodal average,
//! smooth to look at) and a cell scalar (what the element actually computed).

use std::fmt::Write as _;
use std::io::{self, Write};

use crate::Results;

pub fn write(results: &Results, out: &mut dyn Write) -> io::Result<()> {
    let mesh = &results.mesh;
    let mut s = String::new();
    s.push_str(
        "# vtk DataFile Version 3.0\nbasset-fea static study\nASCII\nDATASET UNSTRUCTURED_GRID\n",
    );
    let _ = writeln!(s, "POINTS {} double", mesh.nodes.len());
    for p in &mesh.nodes {
        let _ = writeln!(s, "{} {} {}", p.x, p.y, p.z);
    }
    let _ = writeln!(
        s,
        "CELLS {} {}",
        mesh.elements.len(),
        mesh.elements.len() * 9
    );
    for e in &mesh.elements {
        s.push('8');
        for n in e {
            let _ = write!(s, " {n}");
        }
        s.push('\n');
    }
    let _ = writeln!(s, "CELL_TYPES {}", mesh.elements.len());
    for _ in &mesh.elements {
        s.push_str("12\n");
    }
    let _ = writeln!(s, "POINT_DATA {}", mesh.nodes.len());
    s.push_str("VECTORS displacement double\n");
    for d in &results.displacements {
        let _ = writeln!(s, "{} {} {}", d.x, d.y, d.z);
    }
    s.push_str("SCALARS von_mises double 1\nLOOKUP_TABLE default\n");
    for v in &results.nodal_von_mises {
        let _ = writeln!(s, "{v}");
    }
    let _ = writeln!(s, "CELL_DATA {}", mesh.elements.len());
    s.push_str("SCALARS von_mises double 1\nLOOKUP_TABLE default\n");
    for v in &results.von_mises {
        let _ = writeln!(s, "{v}");
    }
    out.write_all(s.as_bytes())
}

pub fn write_file(results: &Results, path: &std::path::Path) -> io::Result<()> {
    let mut f = io::BufWriter::new(std::fs::File::create(path)?);
    write(results, &mut f)?;
    f.flush()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Load, LoadKind, Material, Study};
    use basset_kernel::{FaceKey, FaceRole, OpId, primitives};
    use basset_math::Vec3;

    #[test]
    fn writes_a_grid_with_one_cell_per_element() {
        let solid = primitives::cuboid(OpId::new(1), Vec3::ZERO, Vec3::new(2.0, 1.0, 1.0));
        let study = Study {
            material: Material::STEEL,
            fixed: vec![FaceKey::new(OpId::new(1), FaceRole::Side(3))],
            loads: vec![Load {
                face: FaceKey::new(OpId::new(1), FaceRole::Side(1)),
                kind: LoadKind::Force(Vec3::X),
            }],
            element_size: 1.0,
        };
        let r = crate::run(&solid, &study).unwrap();
        let mut buf = Vec::new();
        write(&r, &mut buf).unwrap();
        let text = String::from_utf8(buf).unwrap();
        assert!(text.contains("POINTS 12 double"));
        assert!(text.contains("CELLS 2 18"));
        assert!(text.contains("CELL_TYPES 2\n12\n12\n"));
        assert!(text.contains("VECTORS displacement double"));
    }
}
