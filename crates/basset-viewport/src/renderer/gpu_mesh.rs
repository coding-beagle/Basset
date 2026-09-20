//! Conversion of a [`TriMesh`] into GPU buffers.

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

/// A line segment as stored in an instance buffer: both endpoints, `f32`, and how far
/// along its polyline the segment starts.
///
/// `start` is what makes a dashed curve look dashed. The dash pattern is measured in
/// pixels along the segment, and a tessellated curve is made of segments a few pixels
/// long — shorter than one dash — so a pattern that restarted at every segment put every
/// one of them inside a dash and drew the whole curve solid. Carrying the distance
/// already travelled lets the pattern run on across the joins, so a dashed circle reads
/// as dashed at the size it is actually drawn rather than only when zoomed into.
#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
pub(crate) struct SegmentInstance {
    pub a: [f32; 3],
    pub b: [f32; 3],
    /// Distance from the start of this polyline to `a`, in world units.
    pub start: f32,
    _pad: f32,
}

impl SegmentInstance {
    pub const LAYOUT: wgpu::VertexBufferLayout<'static> = wgpu::VertexBufferLayout {
        array_stride: size_of::<SegmentInstance>() as wgpu::BufferAddress,
        step_mode: wgpu::VertexStepMode::Instance,
        attributes: &wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3, 2 => Float32],
    };

    pub fn new(a: Vec3, b: Vec3) -> Self {
        Self::at(a, b, 0.0)
    }

    pub fn at(a: Vec3, b: Vec3, start: f64) -> Self {
        Self {
            a: a.as_vec3().to_array(),
            b: b.as_vec3().to_array(),
            start: start as f32,
            _pad: 0.0,
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

impl GpuMesh {
    pub fn upload(
        device: &wgpu::Device,
        mesh: &TriMesh,
        edges: &[[Vec3; 2]],
    ) -> Result<Self, ViewportError> {
        validate(mesh)?;
        let (vertices, indices) = split_vertices_by_face(mesh);
        let edges: Vec<SegmentInstance> = edges
            .iter()
            .map(|[a, b]| SegmentInstance::new(*a, *b))
            .collect();
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
