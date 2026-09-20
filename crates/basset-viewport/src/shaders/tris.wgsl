struct TriDraw {
    color: vec4<f32>,
    params: vec4<f32>,
};
@group(1) @binding(0) var<uniform> draw: TriDraw;

// Plain world-space triangles: a flat translucent fill, used to light up a region.
struct VsIn {
    @location(0) position: vec3<f32>,
};

@vertex
fn vs_main(in: VsIn) -> @builtin(position) vec4<f32> {
    let clip = globals.view_proj * vec4<f32>(in.position, 1.0);
    if clip.w < NEAR_W {
        return discard_vertex();
    }
    return clip;
}

@fragment
fn fs_main() -> @location(0) vec4<f32> {
    return draw.color;
}
