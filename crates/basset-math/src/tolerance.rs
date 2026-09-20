//! Tolerances used for geometric coincidence decisions.
//!
//! Two distinct tolerances exist because lengths and angles scale differently: a model
//! 1 m across still needs sub-micron point coincidence, while angular comparisons must
//! not depend on model size at all.

/// Distance below which two lengths (in mm) are treated as equal.
pub const LINEAR_TOL: f64 = 1e-7;

/// Angle (in radians) below which two directions are treated as parallel.
pub const ANGULAR_TOL: f64 = 1e-9;

#[inline]
pub fn approx_eq(a: f64, b: f64) -> bool {
    (a - b).abs() <= LINEAR_TOL
}

#[inline]
pub fn approx_zero(a: f64) -> bool {
    a.abs() <= LINEAR_TOL
}

#[inline]
pub fn approx_eq_vec2(a: glam::DVec2, b: glam::DVec2) -> bool {
    a.distance_squared(b) <= LINEAR_TOL * LINEAR_TOL
}

#[inline]
pub fn approx_eq_vec3(a: glam::DVec3, b: glam::DVec3) -> bool {
    a.distance_squared(b) <= LINEAR_TOL * LINEAR_TOL
}
