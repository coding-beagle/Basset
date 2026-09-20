//! File-path wrappers over the streaming writers, for callers that just want a file on
//! disk.

use std::fs::File;
use std::io::BufWriter;
use std::path::Path;

use crate::{ExportItem, IoError, Unit, stl, threemf};

/// Writes a binary STL; the binary form is smaller and universally supported.
pub fn export_stl_file(path: impl AsRef<Path>, items: &[ExportItem]) -> Result<(), IoError> {
    let file = BufWriter::new(File::create(path)?);
    stl::write_binary(file, items)
}

pub fn export_3mf_file(
    path: impl AsRef<Path>,
    items: &[ExportItem],
    unit: Unit,
) -> Result<(), IoError> {
    let file = BufWriter::new(File::create(path)?);
    threemf::write(file, items, unit)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_util::unit_cube;
    use approx::assert_relative_eq;
    use std::fs::File;

    #[test]
    fn files_are_written_and_readable() {
        let dir = std::env::temp_dir().join(format!("basset-io-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let items = [ExportItem::new("cube", unit_cube())];

        let stl_path = dir.join("cube.stl");
        export_stl_file(&stl_path, &items).unwrap();
        let mesh = stl::read(File::open(&stl_path).unwrap()).unwrap();
        assert_relative_eq!(mesh.signed_volume(), 1.0, epsilon = 1e-6);

        let mf_path = dir.join("cube.3mf");
        export_3mf_file(&mf_path, &items, Unit::Millimeter).unwrap();
        let back = threemf::read(File::open(&mf_path).unwrap()).unwrap();
        assert_eq!(back[0].name, "cube");

        std::fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn unwritable_path_is_an_io_error() {
        let items = [ExportItem::new("cube", unit_cube())];
        let bad = Path::new("/nonexistent-dir-for-basset-io/x.stl");
        assert!(matches!(export_stl_file(bad, &items), Err(IoError::Io(_))));
    }
}
