// A distant light of `Lighting::Environment`.
struct DistantLight {
    // xyz: unit vector towards the light, w: its angular radius in radians.
    direction: vec4<f32>,
    // rgb: irradiance on a surface facing it.
    irradiance: vec4<f32>,
};

// Shared by every pipeline: bound at group 0.
struct Globals {
    view_proj: mat4x4<f32>,
    // xyz: eye position in world space.
    camera_pos: vec4<f32>,
    // xyz: unit vector towards the key light, in world space.
    key_light: vec4<f32>,
    // xy: viewport size in pixels, zw: reciprocal.
    viewport: vec4<f32>,
    inv_view_proj: mat4x4<f32>,
    // x: 1.0 under the environment lighting, y: exposure, z: number of lights.
    environment: vec4<f32>,
    sky_zenith: vec4<f32>,
    sky_horizon: vec4<f32>,
    sky_nadir: vec4<f32>,
    // The sky's irradiance over pi in Legendre polynomials P0, P1, P2 and P4 of the
    // normal's height.
    sky_irradiance: array<vec4<f32>, 4>,
    lights: array<DistantLight, 4>,
};
@group(0) @binding(0) var<uniform> globals: Globals;

// Clip-space w below which a vertex is considered behind the eye. Screen-space expansion
// divides by w, so segments straddling the eye must be clipped before that division.
const NEAR_W: f32 = 1e-4;

// Sends a vertex outside the clip volume so the whole primitive is dropped.
fn discard_vertex() -> vec4<f32> {
    return vec4<f32>(0.0, 0.0, 2.0, 1.0);
}
