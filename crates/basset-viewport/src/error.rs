//! Errors raised by the viewport. Mesh data ultimately comes from user documents (via the
//! kernel), so malformed meshes must be reported, never allowed to panic inside upload code
//! or, worse, produce out-of-bounds GPU reads.

use thiserror::Error;

#[derive(Debug, Clone, PartialEq, Eq, Error)]
pub enum ViewportError {
    #[error("mesh index buffer length {0} is not a multiple of three")]
    IndicesNotTriangles(usize),
    #[error("mesh index {index} is out of range for {vertex_count} vertices")]
    IndexOutOfRange { index: u32, vertex_count: usize },
    #[error("mesh has {normals} normals for {positions} positions")]
    NormalCountMismatch { positions: usize, normals: usize },
    #[error("mesh has {face_ids} face ids for {triangles} triangles")]
    FaceIdCountMismatch { triangles: usize, face_ids: usize },
}
