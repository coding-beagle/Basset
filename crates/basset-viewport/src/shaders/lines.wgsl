struct LineDraw {
    model: mat4x4<f32>,
    color: vec4<f32>,
    // x: width in pixels, y: 1.0 when dashed, z: dash length px, w: gap length px.
    params: vec4<f32>,
};
@group(1) @binding(0) var<uniform> draw: LineDraw;

// One instance per segment; six vertices expand it to a screen-aligned quad.
struct VsIn {
    @builtin(vertex_index) vertex_index: u32,
    @location(0) a: vec3<f32>,
    @location(1) b: vec3<f32>,
};

struct VsOut {
    @builtin(position) clip: vec4<f32>,
    // Distance along the segment in pixels; linear in screen space so dashes stay even.
    @location(0) @interpolate(linear) along_px: f32,
};

@vertex
fn vs_main(in: VsIn) -> VsOut {
    var out: VsOut;
    var ca = globals.view_proj * draw.model * vec4<f32>(in.a, 1.0);
    var cb = globals.view_proj * draw.model * vec4<f32>(in.b, 1.0);
    if ca.w < NEAR_W && cb.w < NEAR_W {
        out.clip = discard_vertex();
        out.along_px = 0.0;
        return out;
    }
    if ca.w < NEAR_W {
        ca = mix(ca, cb, (NEAR_W - ca.w) / (cb.w - ca.w));
    } else if cb.w < NEAR_W {
        cb = mix(cb, ca, (NEAR_W - cb.w) / (ca.w - cb.w));
    }

    let half_viewport = globals.viewport.xy * 0.5;
    let pa = ca.xy / ca.w * half_viewport;
    let pb = cb.xy / cb.w * half_viewport;
    let delta = pb - pa;
    let len = length(delta);
    var dir = vec2<f32>(1.0, 0.0);
    if len > 1e-6 {
        dir = delta / len;
    }
    let normal = vec2<f32>(-dir.y, dir.x);
    let half_width = max(draw.params.x, 0.5) * 0.5;

    // Quad corners: (end, side) for the two triangles a-b-b', a-b'-a'.
    let use_b = in.vertex_index == 1u || in.vertex_index == 2u || in.vertex_index == 4u;
    let side = select(-1.0, 1.0, in.vertex_index == 2u || in.vertex_index == 4u || in.vertex_index == 5u);
    // Extending the ends by half the width gives square caps so joined segments
    // don't show notches at corners.
    var pixel = pa - dir * half_width;
    var clip = ca;
    out.along_px = -half_width;
    if use_b {
        pixel = pb + dir * half_width;
        clip = cb;
        out.along_px = len + half_width;
    }
    pixel += normal * side * half_width;

    out.clip = vec4<f32>(pixel / half_viewport * clip.w, clip.z, clip.w);
    return out;
}

@fragment
fn fs_main(in: VsOut) -> @location(0) vec4<f32> {
    if draw.params.y > 0.5 {
        let period = draw.params.z + draw.params.w;
        let phase = in.along_px - floor(in.along_px / period) * period;
        if phase > draw.params.z {
            discard;
        }
    }
    return draw.color;
}
