//! Mesh interchange for Basset: STL (binary and ASCII) and 3MF.
//!
//! Bodies live in their component's frame, so every [`ExportItem`] carries the world
//! transform that places it. STL has no notion of objects or transforms, so the transform
//! is baked into the vertices at export time; 3MF keeps the mesh in its own frame and
//! records the transform on the build item, which is what slicers expect.
//!
//! Units: the modeller works in millimetres. STL is unitless and every consumer we know
//! of assumes millimetres, so STL export simply writes model coordinates. 3MF requires an
//! explicit unit and we write whichever [`Unit`] the caller asks for without rescaling;
//! it is the caller's job to pass the unit the coordinates are actually in.

pub mod convenience;
pub mod stl;
pub mod threemf;

mod xml;

use basset_math::{Affine3, TriMesh};

/// One body to export, positioned in world space by `transform`.
#[derive(Debug, Clone, PartialEq)]
pub struct ExportItem {
    pub name: String,
    pub mesh: TriMesh,
    /// Component-frame to world-frame placement, applied at export time.
    pub transform: Affine3,
}

impl ExportItem {
    /// An item that is already in world space.
    pub fn new(name: impl Into<String>, mesh: TriMesh) -> Self {
        Self {
            name: name.into(),
            mesh,
            transform: Affine3::IDENTITY,
        }
    }

    pub fn with_transform(mut self, transform: Affine3) -> Self {
        self.transform = transform;
        self
    }

    /// The mesh with `transform` baked in; what an object-less format like STL receives.
    pub fn world_mesh(&self) -> TriMesh {
        let mut mesh = self.mesh.clone();
        mesh.transform(&self.transform);
        mesh
    }
}

/// Length unit recorded in a 3MF package. The 3MF core spec enumerates exactly these
/// names (plus `micron`, which no CAD workflow of ours needs).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum Unit {
    #[default]
    Millimeter,
    Centimeter,
    Meter,
    Inch,
    Foot,
}

impl Unit {
    /// The spelling the 3MF `unit` attribute requires.
    pub fn as_3mf_str(self) -> &'static str {
        match self {
            Unit::Millimeter => "millimeter",
            Unit::Centimeter => "centimeter",
            Unit::Meter => "meter",
            Unit::Inch => "inch",
            Unit::Foot => "foot",
        }
    }

    pub fn from_3mf_str(s: &str) -> Option<Self> {
        Some(match s {
            "millimeter" => Unit::Millimeter,
            "centimeter" => Unit::Centimeter,
            "meter" => Unit::Meter,
            "inch" => Unit::Inch,
            "foot" => Unit::Foot,
            _ => return None,
        })
    }
}

/// Every failure this crate can report. Malformed input and empty exports are user-data
/// problems, so they get a variant rather than a panic.
#[derive(Debug, thiserror::Error)]
pub enum IoError {
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),

    #[error("zip package error: {0}")]
    Zip(#[from] zip::result::ZipError),

    /// Exporting nothing is almost always a UI slip (no body selected), so it is reported
    /// rather than silently producing an empty file.
    #[error("nothing to export: the item list is empty")]
    NoItems,

    #[error("mesh '{name}' has no triangles")]
    EmptyMesh { name: String },

    #[error("mesh '{name}' has a non-finite coordinate")]
    NonFiniteCoordinate { name: String },

    #[error("mesh '{name}' references vertex {index} but only has {count} vertices")]
    IndexOutOfRange {
        name: String,
        index: u32,
        count: usize,
    },

    #[error("malformed STL: {0}")]
    MalformedStl(String),

    #[error("malformed 3MF: {0}")]
    Malformed3mf(String),
}

/// Rejects meshes that no format can represent, before any bytes are written so a
/// failed export never leaves a half-written file behind.
fn validate_items(items: &[ExportItem]) -> Result<(), IoError> {
    if items.is_empty() {
        return Err(IoError::NoItems);
    }
    for item in items {
        let mesh = &item.mesh;
        if mesh.is_empty() {
            return Err(IoError::EmptyMesh {
                name: item.name.clone(),
            });
        }
        if mesh.positions.iter().any(|p| !p.is_finite()) {
            return Err(IoError::NonFiniteCoordinate {
                name: item.name.clone(),
            });
        }
        let count = mesh.positions.len();
        if let Some(&index) = mesh.indices.iter().find(|&&i| i as usize >= count) {
            return Err(IoError::IndexOutOfRange {
                name: item.name.clone(),
                index,
                count,
            });
        }
    }
    Ok(())
}

#[cfg(test)]
pub(crate) mod test_util {
    use basset_math::{TriMesh, Vec3};

    /// Flat-shaded unit cube at the origin: 36 vertices, 12 triangles, volume 1.
    pub fn unit_cube() -> TriMesh {
        let mut m = TriMesh::default();
        let v = |x: f64, y: f64, z: f64| Vec3::new(x, y, z);
        let quads = [
            [v(0., 0., 0.), v(0., 1., 0.), v(1., 1., 0.), v(1., 0., 0.)],
            [v(0., 0., 1.), v(1., 0., 1.), v(1., 1., 1.), v(0., 1., 1.)],
            [v(0., 0., 0.), v(1., 0., 0.), v(1., 0., 1.), v(0., 0., 1.)],
            [v(0., 1., 0.), v(0., 1., 1.), v(1., 1., 1.), v(1., 1., 0.)],
            [v(0., 0., 0.), v(0., 0., 1.), v(0., 1., 1.), v(0., 1., 0.)],
            [v(1., 0., 0.), v(1., 1., 0.), v(1., 1., 1.), v(1., 0., 1.)],
        ];
        for (id, q) in quads.iter().enumerate() {
            m.push_triangle([q[0], q[1], q[2]], id as u32);
            m.push_triangle([q[0], q[2], q[3]], id as u32);
        }
        m
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use basset_math::Vec3;

    #[test]
    fn empty_item_list_is_rejected() {
        assert!(matches!(validate_items(&[]), Err(IoError::NoItems)));
    }

    #[test]
    fn empty_mesh_is_rejected_by_name() {
        let items = [ExportItem::new("hollow", TriMesh::default())];
        match validate_items(&items) {
            Err(IoError::EmptyMesh { name }) => assert_eq!(name, "hollow"),
            other => panic!("unexpected {other:?}"),
        }
    }

    #[test]
    fn out_of_range_index_is_rejected() {
        let mut mesh = test_util::unit_cube();
        mesh.indices[5] = 999;
        let items = [ExportItem::new("cube", mesh)];
        assert!(matches!(
            validate_items(&items),
            Err(IoError::IndexOutOfRange {
                index: 999,
                count: 36,
                ..
            })
        ));
    }

    #[test]
    fn non_finite_coordinate_is_rejected() {
        let mut mesh = test_util::unit_cube();
        mesh.positions[0] = Vec3::new(f64::NAN, 0.0, 0.0);
        let items = [ExportItem::new("cube", mesh)];
        assert!(matches!(
            validate_items(&items),
            Err(IoError::NonFiniteCoordinate { .. })
        ));
    }

    #[test]
    fn unit_names_round_trip() {
        for u in [
            Unit::Millimeter,
            Unit::Centimeter,
            Unit::Meter,
            Unit::Inch,
            Unit::Foot,
        ] {
            assert_eq!(Unit::from_3mf_str(u.as_3mf_str()), Some(u));
        }
        assert_eq!(Unit::from_3mf_str("furlong"), None);
    }
}
