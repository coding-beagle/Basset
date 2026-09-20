struct MeshDraw {
    model: mat4x4<f32>,
    // Inverse transpose of `model`, padded to a mat4 to keep the layout trivial.
    normal_matrix: mat4x4<f32>,
    color: vec4<f32>,
    highlight_color: vec4<f32>,
};
@group(1) @binding(0) var<uniform> draw: MeshDraw;

// One bit per kernel face id; see `HighlightBits` on the Rust side.
@group(2) @binding(0) var<storage, read> highlight_bits: array<u32>;

struct VsIn {
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) face_id: u32,
};

struct VsOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) world_pos: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) @interpolate(flat) highlighted: u32,
};

fn is_highlighted(face_id: u32) -> u32 {
    let word = face_id / 32u;
    if word >= arrayLength(&highlight_bits) {
        return 0u;
    }
    return (highlight_bits[word] >> (face_id % 32u)) & 1u;
}

@vertex
fn vs_main(in: VsIn) -> VsOut {
    var out: VsOut;
    let world = draw.model * vec4<f32>(in.position, 1.0);
    out.clip = globals.view_proj * world;
    out.world_pos = world.xyz;
    out.normal = (draw.normal_matrix * vec4<f32>(in.normal, 0.0)).xyz;
    out.highlighted = is_highlighted(in.face_id);
    return out;
}

@fragment
fn fs_main(in: VsOut) -> @location(0) vec4<f32> {
    let base = select(draw.color, draw.highlight_color, in.highlighted == 1u);
    let view_dir = normalize(globals.camera_pos.xyz - in.world_pos);
    var n = normalize(in.normal);
    // Two-sided lighting: back faces are visible inside open or sectioned bodies, and a
    // black interior reads as a rendering bug rather than as geometry.
    if dot(n, view_dir) < 0.0 {
        n = -n;
    }
    // Headlight keeps everything facing the user readable; the key light from upper-left
    // adds the shape cues a headlight alone cannot give.
    let headlight = max(dot(n, view_dir), 0.0);
    let key_dir = globals.key_light.xyz;
    let key = max(dot(n, key_dir), 0.0);
    let half_vec = normalize(view_dir + key_dir);
    let specular = pow(max(dot(n, half_vec), 0.0), 40.0) * 0.18;
    let lit = base.rgb * (0.28 + 0.42 * headlight + 0.38 * key) + vec3<f32>(specular);
    return vec4<f32>(min(lit, vec3<f32>(1.0)), base.a);
}
