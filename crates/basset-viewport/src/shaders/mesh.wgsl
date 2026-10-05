struct MeshDraw {
    model: mat4x4<f32>,
    // Inverse transpose of `model`, padded to a mat4 to keep the layout trivial.
    normal_matrix: mat4x4<f32>,
    color: vec4<f32>,
    highlight_color: vec4<f32>,
    // x: 1.0 when the vertices' own colours replace `color`, y: 1.0 when only the faces
    // in `mask_bits` are drawn.
    params: vec4<f32>,
    // x: metallic, y: roughness, z: clearcoat.
    material: vec4<f32>,
    // rgb: emitted radiance.
    emission: vec4<f32>,
};
@group(1) @binding(0) var<uniform> draw: MeshDraw;

// One bit per kernel face id; see `FaceBits` on the Rust side.
@group(2) @binding(0) var<storage, read> highlight_bits: array<u32>;
@group(2) @binding(1) var<storage, read> mask_bits: array<u32>;

struct VsIn {
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) face_id: u32,
    @location(3) color: vec3<f32>,
};

struct VsOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) world_pos: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) @interpolate(flat) highlighted: u32,
    @location(3) color: vec3<f32>,
};

fn is_highlighted(face_id: u32) -> u32 {
    let word = face_id / 32u;
    if word >= arrayLength(&highlight_bits) {
        return 0u;
    }
    return (highlight_bits[word] >> (face_id % 32u)) & 1u;
}

fn is_masked_in(face_id: u32) -> bool {
    let word = face_id / 32u;
    if word >= arrayLength(&mask_bits) {
        return false;
    }
    return ((mask_bits[word] >> (face_id % 32u)) & 1u) == 1u;
}

@vertex
fn vs_main(in: VsIn) -> VsOut {
    var out: VsOut;
    let world = draw.model * vec4<f32>(in.position, 1.0);
    out.clip = globals.view_proj * world;
    // Every vertex belongs to exactly one face (the upload splits those shared between
    // faces), so dropping each vertex of a masked-out face drops its triangles whole.
    if draw.params.y > 0.5 && !is_masked_in(in.face_id) {
        out.clip = discard_vertex();
    }
    out.world_pos = world.xyz;
    out.normal = (draw.normal_matrix * vec4<f32>(in.normal, 0.0)).xyz;
    out.highlighted = is_highlighted(in.face_id);
    out.color = in.color;
    return out;
}

// The modelling view's lighting. Headlight keeps everything facing the user readable; the
// key light from upper-left adds the shape cues a headlight alone cannot give.
//
// The material shapes the highlight without turning this into a renderer. Roughness sets
// its sharpness and strength, scaled so that 0.5 — the default material — gives exactly
// the exponent 40 and strength 0.18 every body has always had; a metal tints it with the
// base colour and gives up most of its diffuse colour to it; a clear coat adds a sharp
// white glint on top. With the default material every extra term is an exact zero or
// one, so a plain body is lit to the bit as it was.
fn shade_studio(base: vec3<f32>, n: vec3<f32>, view_dir: vec3<f32>) -> vec3<f32> {
    let metallic = draw.material.x;
    let roughness = clamp(draw.material.y, 0.03, 1.0);
    let headlight = max(dot(n, view_dir), 0.0);
    let key_dir = globals.key_light.xyz;
    let key = max(dot(n, key_dir), 0.0);
    let half_vec = normalize(view_dir + key_dir);
    let n_h = max(dot(n, half_vec), 0.0);
    let specular = pow(n_h, 10.0 / (roughness * roughness)) * (0.18 * (0.5 / roughness));
    let highlight = mix(vec3<f32>(1.0), base, metallic) * specular;
    let diffuse = base * (0.28 + 0.42 * headlight + 0.38 * key) * (1.0 - 0.7 * metallic);
    let coat = pow(n_h, 4000.0) * 0.9 * draw.material.z;
    return diffuse + highlight + vec3<f32>(coat) + draw.emission.rgb;
}

// Perceptual roughness of the clear coat: a lacquer is close to a mirror.
const CLEARCOAT_ROUGHNESS: f32 = 0.05;

// GGX normal distribution for roughness `alpha` (perceptual roughness squared).
fn ggx_distribution(n_h: f32, alpha: f32) -> f32 {
    let a2 = alpha * alpha;
    let d = n_h * n_h * (a2 - 1.0) + 1.0;
    return a2 / (PI * d * d);
}

// Height-correlated Smith visibility, the geometry term with the BRDF's 1/(4 n.l n.v)
// folded in.
fn smith_visibility(n_v: f32, n_l: f32, alpha: f32) -> f32 {
    let a2 = alpha * alpha;
    let view = n_l * sqrt(n_v * n_v * (1.0 - a2) + a2);
    let light = n_v * sqrt(n_l * n_l * (1.0 - a2) + a2);
    return 0.5 / max(view + light, 1e-6);
}

fn fresnel_schlick(f0: vec3<f32>, cos_theta: f32) -> vec3<f32> {
    return f0 + (vec3<f32>(1.0) - f0) * pow(1.0 - cos_theta, 5.0);
}

// Karis' analytic fit of the split-sum environment BRDF (EnvBRDFApprox): the fraction of
// a blurred environment a surface reflects, by its F0, roughness and viewing angle.
fn environment_brdf(f0: vec3<f32>, roughness: f32, n_v: f32) -> vec3<f32> {
    let c0 = vec4<f32>(-1.0, -0.0275, -0.572, 0.022);
    let c1 = vec4<f32>(1.0, 0.0425, 1.04, -0.04);
    let r = roughness * c0 + c1;
    let a004 = min(r.x * r.x, exp2(-9.28 * n_v)) * r.x + r.y;
    let ab = vec2<f32>(-1.04, 1.04) * a004 + r.zw;
    return f0 * ab.x + ab.y;
}

// A distant light's specular lobe, D times visibility. The light is a disc, not a point,
// so the GGX lobe is widened by half its angular radius (half because the half-vector
// turns half as far as the light does) — Karis' sphere-light widening. The widened lobe is
// still normalised, so this stands in for convolving the lobe with the disc and keeps its
// energy: a mirror shows a soft box as a highlight of the box's own size and brightness
// instead of a point it would show of a point light. Karis' further (alpha/alpha')^2
// factor belongs with his representative-point lookup, which this does not do, and would
// all but erase the highlight on a smooth surface without it.
fn light_lobe(n_h: f32, n_v: f32, n_l: f32, alpha: f32, angular_radius: f32) -> f32 {
    let widened = min(alpha + 0.5 * angular_radius, 1.0);
    return ggx_distribution(n_h, widened) * smith_visibility(n_v, n_l, alpha);
}

// Metallic-roughness shading under the sky and the distant lights, before exposure.
fn shade_environment(base: vec3<f32>, n: vec3<f32>, v: vec3<f32>) -> vec3<f32> {
    let metallic = clamp(draw.material.x, 0.0, 1.0);
    let roughness = clamp(draw.material.y, 0.03, 1.0);
    let clearcoat = clamp(draw.material.z, 0.0, 1.0);
    let alpha = roughness * roughness;
    let coat_alpha = CLEARCOAT_ROUGHNESS * CLEARCOAT_ROUGHNESS;
    let f0 = mix(vec3<f32>(0.04), base, metallic);
    let diffuse_color = base * (1.0 - metallic);
    let n_v = max(dot(n, v), 1e-4);
    // What the coat reflects is not there to light the layer under it. Taken at the view
    // angle for every light, as is usual, rather than per half-vector.
    let coat_fresnel = fresnel_schlick(vec3<f32>(0.04), n_v).x * clearcoat;

    var direct = vec3<f32>(0.0);
    let light_count = min(u32(globals.environment.z), 4u);
    for (var i = 0u; i < light_count; i += 1u) {
        let light = globals.lights[i];
        let l = light.direction.xyz;
        let n_l = dot(n, l);
        if n_l <= 0.0 {
            continue;
        }
        let h = normalize(v + l);
        let n_h = max(dot(n, h), 0.0);
        let v_h = max(dot(v, h), 0.0);
        let radius = light.direction.w;
        let specular = light_lobe(n_h, n_v, n_l, alpha, radius) * fresnel_schlick(f0, v_h);
        let coat = light_lobe(n_h, n_v, n_l, coat_alpha, radius)
            * fresnel_schlick(vec3<f32>(0.04), v_h).x * clearcoat;
        let layer = (diffuse_color / PI + specular) * (1.0 - coat_fresnel) + vec3<f32>(coat);
        direct += layer * light.irradiance.rgb * n_l;
    }

    // The sky reflected about the normal, blurred towards the cosine-weighted average
    // around the reflection as the surface roughens: the prefiltered environment, without
    // the prefiltering, which a three-colour gradient has little need of.
    let r = reflect(-v, n);
    let reflected = mix(sky_radiance(r), sky_irradiance(r), roughness);
    let ambient = diffuse_color * sky_irradiance(n)
        + reflected * environment_brdf(f0, roughness, n_v);
    let coat_ambient = sky_radiance(r)
        * environment_brdf(vec3<f32>(0.04), CLEARCOAT_ROUGHNESS, n_v).x * clearcoat;
    return direct + ambient * (1.0 - coat_fresnel) + coat_ambient + draw.emission.rgb;
}

@fragment
fn fs_main(in: VsOut) -> @location(0) vec4<f32> {
    var base = draw.color;
    if draw.params.x > 0.5 {
        base = vec4<f32>(in.color, draw.color.a);
    }
    base = select(base, draw.highlight_color, in.highlighted == 1u);
    let view_dir = normalize(globals.camera_pos.xyz - in.world_pos);
    var n = normalize(in.normal);
    // Two-sided lighting: back faces are visible inside open or sectioned bodies, and a
    // black interior reads as a rendering bug rather than as geometry.
    if dot(n, view_dir) < 0.0 {
        n = -n;
    }
    if globals.environment.x > 0.5 {
        return vec4<f32>(tone_map(shade_environment(base.rgb, n, view_dir)), base.a);
    }
    return vec4<f32>(min(shade_studio(base.rgb, n, view_dir), vec3<f32>(1.0)), base.a);
}
