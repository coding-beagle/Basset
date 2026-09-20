//! Conversion of a [`TriMesh`] into GPU buffers, plus feature-edge extraction.

use std::collections::HashMap;

use basset_math::{TriMesh, Vec3};
use bytemuck::{Pod, Zeroable};
use wgpu::util::DeviceExt;

use crate::error::ViewportError;

#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
pub(crate) struct MeshVertex {
    pub position: [f32; 3],
    pub normal: [f32; 3],
    pub face_id: u32,
}

impl MeshVertex {
    pub const LAYOUT: wgpu::VertexBufferLayout<'static> = wgpu::VertexBufferLayout {
        array_stride: size_of::<MeshVertex>() as wgpu::BufferAddress,
        step_mode: wgpu::VertexStepMode::Vertex,
        attributes: &wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3, 2 => Uint32],
    };
}

/// A line segment as stored in an instance buffer: both endpoints, `f32`.
#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
pub(crate) struct SegmentInstance {
    pub a: [f32; 3],
    pub b: [f32; 3],
}

impl SegmentInstance {
    pub const LAYOUT: wgpu::VertexBufferLayout<'static> = wgpu::VertexBufferLayout {
        array_stride: size_of::<SegmentInstance>() as wgpu::BufferAddress,
        step_mode: wgpu::VertexStepMode::Instance,
        attributes: &wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3],
    };

    pub fn new(a: Vec3, b: Vec3) -> Self {
        Self {
            a: a.as_vec3().to_array(),
            b: b.as_vec3().to_array(),
        }
    }
}

pub(crate) struct GpuMesh {
    pub vertices: wgpu::Buffer,
    pub indices: wgpu::Buffer,
    pub index_count: u32,
    /// Feature edges for [`crate::MeshStyle::ShadedWithEdges`]; `None` when the mesh has none.
    pub edges: Option<wgpu::Buffer>,
    pub edge_count: u32,
    /// Number of `u32` words needed to hold one highlight bit per face id.
    pub highlight_words: u32,
}

/// Adjacent triangles whose normals differ by more than this are separated by a crease
/// edge even when they belong to the same kernel face (e.g. coarse cylinder facets do not
/// qualify, a folded sheet does).
const CREASE_ANGLE_COS: f64 = 0.5; // 60°

/// Positions closer than this are merged when finding shared edges. Flat-shaded meshes
/// duplicate vertices per face, so edges cannot be matched by index alone.
const WELD_QUANTUM: f64 = 1e-6;

impl GpuMesh {
    pub fn upload(device: &wgpu::Device, mesh: &TriMesh) -> Result<Self, ViewportError> {
        validate(mesh)?;
        let (vertices, indices) = split_vertices_by_face(mesh);
        let edges = feature_edges(mesh);
        let max_face_id = mesh.face_ids.iter().copied().max().unwrap_or(0);

        let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("mesh vertices"),
            contents: bytemuck::cast_slice(&vertices),
            usage: wgpu::BufferUsages::VERTEX,
        });
        let index_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("mesh indices"),
            contents: bytemuck::cast_slice(&indices),
            usage: wgpu::BufferUsages::INDEX,
        });
        let edge_buffer = (!edges.is_empty()).then(|| {
            device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("mesh feature edges"),
                contents: bytemuck::cast_slice(&edges),
                usage: wgpu::BufferUsages::VERTEX,
            })
        });
        Ok(Self {
            vertices: vertex_buffer,
            indices: index_buffer,
            index_count: indices.len() as u32,
            edges: edge_buffer,
            edge_count: edges.len() as u32,
            highlight_words: max_face_id / 32 + 1,
        })
    }
}

fn validate(mesh: &TriMesh) -> Result<(), ViewportError> {
    if !mesh.indices.len().is_multiple_of(3) {
        return Err(ViewportError::IndicesNotTriangles(mesh.indices.len()));
    }
    if mesh.normals.len() != mesh.positions.len() {
        return Err(ViewportError::NormalCountMismatch {
            positions: mesh.positions.len(),
            normals: mesh.normals.len(),
        });
    }
    if mesh.face_ids.len() != mesh.triangle_count() {
        return Err(ViewportError::FaceIdCountMismatch {
            triangles: mesh.triangle_count(),
            face_ids: mesh.face_ids.len(),
        });
    }
    if let Some(&index) = mesh
        .indices
        .iter()
        .find(|&&i| i as usize >= mesh.positions.len())
    {
        return Err(ViewportError::IndexOutOfRange {
            index,
            vertex_count: mesh.positions.len(),
        });
    }
    Ok(())
}

/// Face ids live per triangle in a `TriMesh` but per vertex on the GPU. A vertex shared by
/// triangles of different faces (smooth-shaded kernels do this along tangent edges) is
/// duplicated once per face; everything else keeps its index reuse.
fn split_vertices_by_face(mesh: &TriMesh) -> (Vec<MeshVertex>, Vec<u32>) {
    let mut vertices: Vec<MeshVertex> = mesh
        .positions
        .iter()
        .zip(&mesh.normals)
        .map(|(p, n)| MeshVertex {
            position: p.as_vec3().to_array(),
            normal: n.as_vec3().to_array(),
            face_id: u32::MAX,
        })
        .collect();
    let mut duplicates: HashMap<(u32, u32), u32> = HashMap::new();
    let mut indices = Vec::with_capacity(mesh.indices.len());

    for (tri, &face_id) in mesh.indices.as_chunks::<3>().0.iter().zip(&mesh.face_ids) {
        for &index in tri {
            let vertex = &mut vertices[index as usize];
            let resolved = if vertex.face_id == u32::MAX {
                vertex.face_id = face_id;
                index
            } else if vertex.face_id == face_id {
                index
            } else {
                *duplicates.entry((index, face_id)).or_insert_with(|| {
                    let copy = MeshVertex {
                        face_id,
                        ..vertices[index as usize]
                    };
                    vertices.push(copy);
                    (vertices.len() - 1) as u32
                })
            };
            indices.push(resolved);
        }
    }
    (vertices, indices)
}

struct EdgeRecord {
    face_id: u32,
    normal: Vec3,
    /// Number of triangles seen so far sharing this edge.
    uses: u32,
    feature: bool,
}

/// Edges between different kernel faces, creases sharper than [`CREASE_ANGLE_COS`], and
/// open borders. Silhouettes are view-dependent and intentionally not included.
fn feature_edges(mesh: &TriMesh) -> Vec<SegmentInstance> {
    let mut welded: HashMap<[i64; 3], u32> = HashMap::new();
    let weld = |p: Vec3, welded: &mut HashMap<[i64; 3], u32>| {
        let key = [
            (p.x / WELD_QUANTUM).round() as i64,
            (p.y / WELD_QUANTUM).round() as i64,
            (p.z / WELD_QUANTUM).round() as i64,
        ];
        let next = welded.len() as u32;
        *welded.entry(key).or_insert(next)
    };

    let mut edges: HashMap<(u32, u32), EdgeRecord> = HashMap::new();
    let mut endpoints: HashMap<(u32, u32), [Vec3; 2]> = HashMap::new();
    for (i, &face_id) in mesh.face_ids.iter().enumerate() {
        let tri = mesh.triangle(i);
        let normal = (tri[1] - tri[0]).cross(tri[2] - tri[0]).normalize_or_zero();
        let keys = [
            weld(tri[0], &mut welded),
            weld(tri[1], &mut welded),
            weld(tri[2], &mut welded),
        ];
        for k in 0..3 {
            let (ka, kb) = (keys[k], keys[(k + 1) % 3]);
            if ka == kb {
                continue; // degenerate triangle edge
            }
            let key = (ka.min(kb), ka.max(kb));
            match edges.get_mut(&key) {
                None => {
                    edges.insert(
                        key,
                        EdgeRecord {
                            face_id,
                            normal,
                            uses: 1,
                            feature: false,
                        },
                    );
                    endpoints.insert(key, [tri[k], tri[(k + 1) % 3]]);
                }
                Some(record) => {
                    record.uses += 1;
                    if record.face_id != face_id || record.normal.dot(normal) < CREASE_ANGLE_COS {
                        record.feature = true;
                    }
                }
            }
        }
    }

    let mut segments: Vec<(&(u32, u32), SegmentInstance)> = edges
        .iter()
        .filter(|(_, r)| r.feature || r.uses == 1)
        .map(|(key, _)| {
            let [a, b] = endpoints[key];
            (key, SegmentInstance::new(a, b))
        })
        .collect();
    // HashMap order is arbitrary; sort so uploads are deterministic frame to frame.
    segments.sort_by_key(|(key, _)| **key);
    segments.into_iter().map(|(_, s)| s).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Unit cube, two triangles per face, six face ids.
    pub(crate) fn cube() -> TriMesh {
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

    #[test]
    fn cube_has_twelve_feature_edges() {
        let edges = feature_edges(&cube());
        assert_eq!(
            edges.len(),
            12,
            "diagonals within a face are not feature edges"
        );
    }

    #[test]
    fn open_border_is_an_edge() {
        let mut m = TriMesh::default();
        m.push_triangle([Vec3::ZERO, Vec3::X, Vec3::Y], 0);
        assert_eq!(feature_edges(&m).len(), 3);
    }

    #[test]
    fn shared_vertex_across_faces_is_duplicated() {
        // Two triangles sharing vertices 0 and 1 but tagged with different faces.
        let m = TriMesh {
            positions: vec![Vec3::ZERO, Vec3::X, Vec3::Y, Vec3::new(1.0, 1.0, 0.0)],
            normals: vec![Vec3::Z; 4],
            indices: vec![0, 1, 2, 1, 3, 2],
            face_ids: vec![0, 1],
        };
        let (vertices, indices) = split_vertices_by_face(&m);
        assert_eq!(
            vertices.len(),
            6,
            "vertices 1 and 2 are duplicated for face 1"
        );
        assert_eq!(indices.len(), 6);
        for (tri, &face) in indices.as_chunks::<3>().0.iter().zip(&m.face_ids) {
            for &i in tri {
                assert_eq!(vertices[i as usize].face_id, face);
            }
        }
    }

    #[test]
    fn validation_rejects_malformed_meshes() {
        let mut m = cube();
        m.indices.push(0);
        assert!(matches!(
            validate(&m),
            Err(ViewportError::IndicesNotTriangles(_))
        ));
        let mut m = cube();
        m.indices[0] = 999;
        assert!(matches!(
            validate(&m),
            Err(ViewportError::IndexOutOfRange { index: 999, .. })
        ));
        let mut m = cube();
        m.face_ids.pop();
        assert!(matches!(
            validate(&m),
            Err(ViewportError::FaceIdCountMismatch { .. })
        ));
        let mut m = cube();
        m.normals.pop();
        assert!(matches!(
            validate(&m),
            Err(ViewportError::NormalCountMismatch { .. })
        ));
        assert!(validate(&cube()).is_ok());
    }
}
