// The sky gradient behind the model: one triangle covering the screen, each pixel showing
// the sky along its view ray.

struct VsOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) ndc: vec2<f32>,
};

@vertex
fn vs_main(@builtin(vertex_index) vertex_index: u32) -> VsOut {
    // (-1,-1), (3,-1), (-1,3): a triangle whose inside covers the whole of clip space.
    let ndc = vec2<f32>(f32((vertex_index << 1u) & 2u), f32(vertex_index & 2u)) * 2.0 - 1.0;
    var out: VsOut;
    out.clip = vec4<f32>(ndc, 0.0, 1.0);
    out.ndc = ndc;
    return out;
}

@fragment
fn fs_main(in: VsOut) -> @location(0) vec4<f32> {
    // The ray runs from the pixel's point on the near plane to its point on the far
    // plane. Unprojecting both rather than building the ray from the camera's position
    // makes an orthographic view come out right with no special case: its two points
    // differ only along the view direction, so every pixel looks the same way.
    let near = globals.inv_view_proj * vec4<f32>(in.ndc, 0.0, 1.0);
    let far = globals.inv_view_proj * vec4<f32>(in.ndc, 1.0, 1.0);
    let direction = normalize(far.xyz / far.w - near.xyz / near.w);
    return vec4<f32>(tone_map(sky_radiance(direction)), 1.0);
}
