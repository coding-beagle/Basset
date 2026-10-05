// The sky and tone mapping of `Lighting::Environment`, shared by the mesh and sky shaders.
// `EnvironmentLight::sky` and `sky_irradiance_coefficients` on the Rust side are the
// definitions these must agree with.

const PI: f32 = 3.14159265;

// Radiance of the sky along the unit direction `d`: horizon to pole by the square root of
// the height.
fn sky_radiance(d: vec3<f32>) -> vec3<f32> {
    let z = clamp(d.z, -1.0, 1.0);
    let t = sqrt(abs(z));
    var pole = globals.sky_nadir.rgb;
    if z >= 0.0 {
        pole = globals.sky_zenith.rgb;
    }
    return globals.sky_horizon.rgb * (1.0 - t) + pole * t;
}

// Cosine-weighted irradiance of the sky about the unit normal `n`, over pi: what a white
// Lambertian surface facing `n` reflects.
fn sky_irradiance(n: vec3<f32>) -> vec3<f32> {
    let z = clamp(n.z, -1.0, 1.0);
    let z2 = z * z;
    let p2 = 1.5 * z2 - 0.5;
    let p4 = (35.0 * z2 * z2 - 30.0 * z2 + 3.0) / 8.0;
    let c = globals.sky_irradiance;
    return c[0].rgb + c[1].rgb * z + c[2].rgb * p2 + c[3].rgb * p4;
}

// Exposure, then Narkowicz's fit of the ACES filmic curve, which rolls highlights off
// rather than clipping them so a bright reflection keeps its shape. The result is linear;
// the sRGB target encodes it.
fn tone_map(radiance: vec3<f32>) -> vec3<f32> {
    let x = max(radiance * globals.environment.y, vec3<f32>(0.0));
    let mapped = (x * (2.51 * x + 0.03)) / (x * (2.43 * x + 0.59) + 0.14);
    return clamp(mapped, vec3<f32>(0.0), vec3<f32>(1.0));
}
