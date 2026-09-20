// Shared by every pipeline: bound at group 0.
struct Globals {
    view_proj: mat4x4<f32>,
    // xyz: eye position in world space.
    camera_pos: vec4<f32>,
    // xyz: unit vector towards the key light, in world space.
    key_light: vec4<f32>,
    // xy: viewport size in pixels, zw: reciprocal.
    viewport: vec4<f32>,
};
@group(0) @binding(0) var<uniform> globals: Globals;

// Clip-space w below which a vertex is considered behind the eye. Screen-space expansion
// divides by w, so segments straddling the eye must be clipped before that division.
const NEAR_W: f32 = 1e-4;

// Sends a vertex outside the clip volume so the whole primitive is dropped.
fn discard_vertex() -> vec4<f32> {
    return vec4<f32>(0.0, 0.0, 2.0, 1.0);
}
