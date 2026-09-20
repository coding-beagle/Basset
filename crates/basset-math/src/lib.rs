//! Numeric foundations shared by every Basset crate.
//!
//! All modelling maths is done in `f64`. Renderers convert to `f32` at upload time; the
//! kernel never does, because accumulated single-precision error is the classic source of
//! boolean failures in CAD kernels.
//!
//! Units are millimetres and radians throughout the codebase. Conversions happen at the
//! UI boundary only.

pub mod frame;
pub mod mesh;
pub mod tolerance;

pub use frame::{Frame, Plane, Ray};
pub use glam::{
    DAffine3 as Affine3, DMat3 as Mat3, DMat4 as Mat4, DQuat as Quat, DVec2 as Vec2, DVec3 as Vec3,
    EulerRot,
};
pub use mesh::{Aabb, RayHit, TriMesh};
pub use tolerance::{
    ANGULAR_TOL, LINEAR_TOL, approx_eq, approx_eq_vec2, approx_eq_vec3, approx_zero,
};
