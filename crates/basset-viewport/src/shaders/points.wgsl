struct PointDraw {
    color: vec4<f32>,
    // x: marker size in pixels.
    params: vec4<f32>,
};
@group(1) @binding(0) var<uniform> draw: PointDraw;

// One instance per point; six vertices make a screen-aligned square.
struct VsIn {
    @builtin(vertex_index) vertex_index: u32,
    @location(0) position: vec3<f32>,
};

@vertex
fn vs_main(in: VsIn) -> @builtin(position) vec4<f32> {
    let clip = globals.view_proj * vec4<f32>(in.position, 1.0);
    if clip.w < NEAR_W {
        return discard_vertex();
    }
    let half_size = max(draw.params.x, 1.0) * 0.5;
    var corner = vec2<f32>(-1.0, -1.0);
    switch in.vertex_index {
        case 1u: { corner = vec2<f32>(1.0, -1.0); }
        case 2u, 4u: { corner = vec2<f32>(1.0, 1.0); }
        case 5u: { corner = vec2<f32>(-1.0, 1.0); }
        default: {}
    }
    let offset_ndc = corner * half_size * globals.viewport.zw * 2.0;
    return vec4<f32>(clip.xy + offset_ndc * clip.w, clip.z, clip.w);
}

@fragment
fn fs_main() -> @location(0) vec4<f32> {
    return draw.color;
}
