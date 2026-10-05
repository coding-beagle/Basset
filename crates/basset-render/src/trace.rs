//! The path tracer: one sample of one pixel.
//!
//! A unidirectional path tracer with next-event estimation towards the environment's
//! distant lights, combined with the scattering function's own samples by multiple
//! importance sampling (the power heuristic). The lights are small and bright — a sun, a
//! soft box — and are where nearly all the noise of plain path tracing would come from;
//! sampling them directly is what lets an in-canvas render be readable after a handful of
//! passes. The sky is broad and dim and is only ever reached by escaping into it, so it
//! needs no light sampling of its own.
//!
//! Surfaces are the metallic–roughness model of [`crate::appearance`]: a Lambertian base
//! weighted by one minus metallic, a GGX specular lobe whose reflectance runs from the
//! four per cent of a dielectric to the base colour of a metal, an optional smooth clear
//! coat over both, and for transmissive appearances a rough dielectric interface chosen
//! with probability `transmission`. The opaque lobes are sampled as one mixture and
//! evaluated as one (so their combined density is what MIS sees); the dielectric branch
//! is not light-sampled, so it is treated as a delta for MIS and its paths count the
//! lights they run into in full.
//!
//! # The floor
//!
//! Fusion's floor is invisible except for what the model does to it: its shadow and,
//! optionally, its reflection, over a background that runs on under it. That is a shadow
//! catcher, done the way production renderers do it: when a camera ray lands on the floor
//! the sample is shaded twice in one go — once as a real floor with the model in the
//! world, once as the same floor with nothing in the world but the environment — and the
//! pixel keeps the background scaled by the ratio of the two. Where the model shades the
//! floor the ratio drops below one, where it reflects in it the ratio moves, and far from
//! the model the two agree and the floor disappears into the background with no seam to
//! fade. The ratio is taken of the sums over all a pixel's samples, not per sample, so it
//! converges rather than averaging noisy quotients. Secondary rays meet the floor as a
//! real surface: a chrome part reflects a floor, and a floor bounces light up into the
//! underside of the part.

use basset_math::Vec3;

use crate::bvh::TraceRay;
use crate::camera::RenderCamera;
use crate::color::luminance;
use crate::sampling::{
    Basis, Rng, cosine_hemisphere, cosine_pdf, fresnel_dielectric, ggx_d, power_heuristic, reflect,
    refract, sample_visible_normal, schlick, smith_g1, smith_g2, uniform_cone,
    visible_reflection_pdf,
};
use crate::scene::{Contact, Surface, TraceScene};

type C3 = [f64; 3];

const BLACK: C3 = [0.0; 3];

fn add(a: C3, b: C3) -> C3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn mul(a: C3, b: C3) -> C3 {
    [a[0] * b[0], a[1] * b[1], a[2] * b[2]]
}

fn scale(a: C3, s: f64) -> C3 {
    a.map(|v| v * s)
}

fn lum(c: C3) -> f64 {
    f64::from(luminance(c.map(|v| v as f32)))
}

fn widen(c: crate::color::Rgb) -> C3 {
    c.map(f64::from)
}

/// What one sample of a pixel contributes. A sample either sees the model or the
/// background (`direct`), or lands on the floor, in which case the background behind
/// the floor and the floor's two shadings are kept for the pixel to take the ratio of.
#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct Sample {
    pub direct: C3,
    pub floor: Option<FloorSample>,
}

#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct FloorSample {
    pub background: C3,
    pub with_model: C3,
    pub without_model: C3,
}

/// How a path left its last surface, which is what the MIS weight of a light it then
/// runs into depends on.
#[derive(Clone, Copy, Debug)]
enum Arrival {
    Camera,
    /// Scattered by a lobe that next-event estimation also covers, with this density.
    Sampled(f64),
    /// A delta-like scatter (glass) that next-event estimation does not cover.
    Specular,
}

pub(crate) struct Tracer<'a> {
    pub scene: &'a TraceScene,
    pub max_bounces: u32,
    /// The largest radiance a single sample may carry, against fireflies.
    pub clamp: f64,
}

impl Tracer<'_> {
    pub fn sample(
        &self,
        camera: &RenderCamera,
        x: f64,
        y: f64,
        aspect: f64,
        rng: &mut Rng,
    ) -> Sample {
        let ray = camera.ray(x, y, aspect, rng);
        let scene = self.scene;
        match scene.intersect(&ray, f64::INFINITY, true) {
            None => Sample {
                direct: self.background(ray.dir),
                floor: None,
            },
            Some(Contact::Ground { t }) => Sample {
                direct: BLACK,
                floor: Some(self.floor(&ray, t, rng)),
            },
            Some(contact) => Sample {
                direct: self.clamped(self.radiance_at(&ray, contact, 0, BLACK.map(|_| 1.0), rng)),
                floor: None,
            },
        }
    }

    fn clamped(&self, c: C3) -> C3 {
        let m = c[0].max(c[1]).max(c[2]);
        if m > self.clamp && m.is_finite() {
            scale(c, self.clamp / m)
        } else if !m.is_finite() {
            BLACK
        } else {
            c
        }
    }

    /// What a camera ray that meets nothing shows.
    fn background(&self, dir: Vec3) -> C3 {
        match self.scene.solid_background {
            Some(c) => c,
            None => widen(self.scene.lighting.sky(dir)),
        }
    }

    /// Radiance of the lights alone in a direction, and the density next-event
    /// estimation would have picked that direction with.
    fn lights_toward(&self, dir: Vec3) -> (C3, f64) {
        let lights = &self.scene.lighting.lights;
        let total: f64 = self.scene.light_weights.iter().sum();
        let mut radiance = BLACK;
        let mut pdf = 0.0;
        for (l, w) in lights.iter().zip(&self.scene.light_weights) {
            if l.covers(dir) {
                radiance = add(radiance, widen(l.radiance));
                pdf += (w / total) / l.solid_angle;
            }
        }
        (radiance, pdf)
    }

    /// What a ray escaping in `dir` after arriving the way it did sees: the sky, and any
    /// light weighted against the chance next-event estimation already counted it.
    fn escape(&self, dir: Vec3, arrival: Arrival) -> C3 {
        let sky = widen(self.scene.lighting.sky(dir));
        let (lights, light_pdf) = self.lights_toward(dir);
        let weight = match arrival {
            Arrival::Sampled(pdf) => power_heuristic(pdf, light_pdf),
            Arrival::Camera | Arrival::Specular => 1.0,
        };
        add(sky, scale(lights, weight))
    }

    /// How much light gets from `origin` along `dir` to infinity: zero past anything
    /// opaque, tinted through anything transmissive. Refraction does not bend a shadow
    /// ray — glass casts a tinted shadow rather than a caustic — which is the usual
    /// compromise and what makes glass parts look right without a caustics solver.
    fn transmittance(&self, origin: Vec3, dir: Vec3) -> C3 {
        let scene = self.scene;
        let mut through = [1.0; 3];
        let mut ray = TraceRay::new(origin, dir);
        for _ in 0..16 {
            let Some(contact) = scene.intersect(&ray, f64::INFINITY, true) else {
                return through;
            };
            let Contact::Triangle { t, index, .. } = contact else {
                return BLACK;
            };
            let corner = &scene.corners[index as usize];
            let m = &scene.materials[corner.material as usize];
            if m.transmission <= 0.0 {
                return BLACK;
            }
            let p = ray.origin + ray.dir * t;
            let s = m.at(p, corner.normals[0]);
            let tint = s.base.map(|c| c.sqrt());
            through = mul(through, scale(tint, s.transmission * 0.96));
            if lum(through) < 1e-4 {
                return BLACK;
            }
            ray = TraceRay::new(p + ray.dir * scene.epsilon * 4.0, ray.dir);
        }
        BLACK
    }

    /// Next-event estimation: one light, one direction on it, weighted against the
    /// scattering function's own density for that direction.
    fn sample_light(
        &self,
        surface: &Surface,
        frame: &Basis,
        p: Vec3,
        wo: Vec3,
        ng: Vec3,
        rng: &mut Rng,
    ) -> C3 {
        let lights = &self.scene.lighting.lights;
        if lights.is_empty() {
            return BLACK;
        }
        let total: f64 = self.scene.light_weights.iter().sum();
        let mut pick = rng.next_f64() * total;
        let mut chosen = lights.len() - 1;
        for (i, w) in self.scene.light_weights.iter().enumerate() {
            if pick < *w {
                chosen = i;
                break;
            }
            pick -= w;
        }
        let light = &lights[chosen];
        let cone = Basis::new(light.direction);
        let wi = cone.to_world(uniform_cone(
            rng.next_f64(),
            rng.next_f64(),
            light.cos_radius,
        ));
        if wi.dot(ng) <= 0.0 {
            return BLACK;
        }
        let (radiance, light_pdf) = self.lights_toward(wi);
        if light_pdf <= 0.0 {
            return BLACK;
        }
        let (f, bsdf_pdf) = eval_opaque(surface, frame.to_local(wo), frame.to_local(wi));
        if lum(f) <= 0.0 {
            return BLACK;
        }
        let visible = self.transmittance(p + ng * self.scene.epsilon, wi);
        let weight = power_heuristic(light_pdf, bsdf_pdf) / light_pdf;
        scale(mul(mul(f, radiance), visible), weight)
    }

    /// The light arriving back along `ray`, which has just met `contact` at the given
    /// depth carrying `throughput`; the result is already multiplied by the throughput.
    fn radiance_at(
        &self,
        ray: &TraceRay,
        contact: Contact,
        depth: u32,
        throughput: C3,
        rng: &mut Rng,
    ) -> C3 {
        let mut ray = *ray;
        let mut contact = Some(contact);
        let mut throughput = throughput;
        let mut arrival = if depth == 0 {
            Arrival::Camera
        } else {
            Arrival::Specular
        };
        let mut total = BLACK;
        let scene = self.scene;
        for bounce in depth..self.max_bounces {
            let Some(hit) = contact else {
                total = add(total, mul(throughput, self.escape(ray.dir, arrival)));
                break;
            };
            let (p, surface, ns, ng, transmissive_geometry) = match hit {
                Contact::Triangle { t, index, u, v } => {
                    let tri = &scene.triangles[index as usize];
                    let corner = &scene.corners[index as usize];
                    let p = ray.origin + ray.dir * t;
                    let [n0, n1, n2] = corner.normals;
                    let mut ns = (n0 * (1.0 - u - v) + n1 * u + n2 * v).normalize_or(n0);
                    let mut ng = tri.e1.cross(tri.e2).normalize();
                    if ng.dot(ns) < 0.0 {
                        ng = -ng;
                    }
                    let material = &scene.materials[corner.material as usize];
                    let surface = material.at(p, ns);
                    if surface.transmission <= 0.0 && ng.dot(ray.dir) > 0.0 {
                        // Seen from behind: an opaque surface is two-sided.
                        ng = -ng;
                        ns = -ns;
                    }
                    (p, surface, ns, ng, surface.transmission > 0.0)
                }
                Contact::Ground { t } => {
                    let p = ray.origin + ray.dir * t;
                    let g = scene
                        .ground
                        .as_ref()
                        .expect("contact with a floor that exists");
                    (p, g.material.at(p, Vec3::Z), Vec3::Z, Vec3::Z, false)
                }
            };
            let wo = -ray.dir;
            total = add(total, mul(throughput, surface.emission));

            // Choose between the opaque lobes and the dielectric interface.
            let glass = transmissive_geometry && rng.next_f64() < surface.transmission;
            let next = if glass {
                sample_glass(&surface, ns, ng, wo, rng).map(|(wi, w)| (wi, w, Arrival::Specular))
            } else {
                // A shading normal tilted past the view would put the viewer under the
                // surface; bend it back towards the geometric one.
                let ns = if ns.dot(wo) <= 1e-4 { ng } else { ns };
                let frame = Basis::new(ns);
                // The branch was chosen with probability 1 − transmission, so the opaque
                // lobes are estimated at full weight here.
                let direct = self.sample_light(&surface, &frame, p, wo, ng, rng);
                total = add(total, mul(throughput, direct));
                sample_opaque(&surface, &frame, wo, ng, rng)
                    .map(|(wi, w, pdf)| (wi, w, Arrival::Sampled(pdf)))
            };
            let Some((wi, weight, how)) = next else {
                break;
            };
            throughput = mul(throughput, weight);
            arrival = how;
            // Russian roulette once the path has had a few bounces to matter.
            if bounce >= 3 {
                let q = throughput[0]
                    .max(throughput[1])
                    .max(throughput[2])
                    .clamp(0.05, 0.95);
                if rng.next_f64() >= q {
                    break;
                }
                throughput = scale(throughput, 1.0 / q);
            }
            let side = if wi.dot(ng) >= 0.0 { ng } else { -ng };
            ray = TraceRay::new(p + side * scene.epsilon, wi);
            contact = scene.intersect(&ray, f64::INFINITY, true);
            if bounce + 1 == self.max_bounces && contact.is_none() {
                total = add(total, mul(throughput, self.escape(ray.dir, arrival)));
            }
        }
        total
    }

    /// A camera ray that landed on the floor, shaded with and without the model.
    fn floor(&self, ray: &TraceRay, t: f64, rng: &mut Rng) -> FloorSample {
        let scene = self.scene;
        let background = self.background(ray.dir);
        let g = scene.ground.as_ref().expect("a floor was hit");
        let p = ray.origin + ray.dir * t;
        let surface = g.material.at(p, Vec3::Z);
        let frame = Basis::new(Vec3::Z);
        let wo = -ray.dir;
        let lights = &scene.lighting.lights;

        // Direct light: the same light sample, once tested against the model and once not.
        let mut direct_with = BLACK;
        let mut direct_without = BLACK;
        if !lights.is_empty() {
            let total: f64 = scene.light_weights.iter().sum();
            let mut pick = rng.next_f64() * total;
            let mut chosen = lights.len() - 1;
            for (i, w) in scene.light_weights.iter().enumerate() {
                if pick < *w {
                    chosen = i;
                    break;
                }
                pick -= w;
            }
            let light = &lights[chosen];
            let wi = Basis::new(light.direction).to_world(uniform_cone(
                rng.next_f64(),
                rng.next_f64(),
                light.cos_radius,
            ));
            if wi.z > 0.0 {
                let (radiance, light_pdf) = self.lights_toward(wi);
                let (f, bsdf_pdf) = eval_opaque(&surface, frame.to_local(wo), frame.to_local(wi));
                if light_pdf > 0.0 {
                    let unoccluded = scale(
                        mul(f, radiance),
                        power_heuristic(light_pdf, bsdf_pdf) / light_pdf,
                    );
                    direct_without = unoccluded;
                    direct_with = mul(
                        unoccluded,
                        self.transmittance(p + Vec3::Z * scene.epsilon, wi),
                    );
                }
            }
        }

        // One bounce: the same direction continued into the world, or straight out to an
        // empty environment.
        let mut indirect_with = BLACK;
        let mut indirect_without = BLACK;
        if let Some((wi, weight, pdf)) = sample_opaque(&surface, &frame, wo, Vec3::Z, rng) {
            let arrival = Arrival::Sampled(pdf);
            indirect_without = mul(weight, self.escape(wi, arrival));
            let next = TraceRay::new(p + Vec3::Z * scene.epsilon, wi);
            indirect_with = match scene.intersect(&next, f64::INFINITY, true) {
                None => indirect_without,
                // The bounce lands on the model. Lights are only met by escaping, and an
                // escape after that surface weighs them by the surface's own density, so
                // the floor's density is not needed past here.
                Some(contact) if self.max_bounces > 1 => {
                    self.radiance_at(&next, contact, 1, weight, rng)
                }
                Some(_) => BLACK,
            };
        }
        FloorSample {
            background,
            with_model: self.clamped(add(direct_with, indirect_with)),
            without_model: self.clamped(add(direct_without, indirect_without)),
        }
    }
}

/// Reflectance at normal incidence of the specular lobe.
fn f0(s: &Surface) -> C3 {
    s.base.map(|c| 0.04 + (c - 0.04) * s.metallic)
}

fn alpha(roughness: f64) -> f64 {
    (roughness * roughness).max(1e-4)
}

/// Probabilities of picking the diffuse, specular and coat lobes for a view direction.
fn lobe_weights(s: &Surface, wo: Vec3) -> [f64; 3] {
    let coat = s.clearcoat * schlick([0.04; 3], wo.z)[0];
    let under = 1.0 - coat;
    let diffuse = lum(s.base) * (1.0 - s.metallic) * under;
    let spec = lum(schlick(f0(s), wo.z)) * under;
    let total = diffuse + spec + coat;
    if total <= 0.0 {
        return [1.0, 0.0, 0.0];
    }
    // Keep every lobe that exists samplable, or a dim lobe's light arrives only through
    // next-event estimation and its reflections of the sky never appear.
    let floor = |w: f64, exists: bool| if exists { w.max(0.05 * total) } else { 0.0 };
    let d = floor(diffuse, s.metallic < 1.0);
    let sp = floor(spec, true);
    let c = floor(coat, s.clearcoat > 0.0);
    let t = d + sp + c;
    [d / t, sp / t, c / t]
}

/// The opaque lobes' BSDF times the cosine, and their combined sampling density, for
/// local directions (shading normal +Z).
fn eval_opaque(s: &Surface, wo: Vec3, wi: Vec3) -> (C3, f64) {
    if wo.z <= 0.0 || wi.z <= 0.0 {
        return (BLACK, 0.0);
    }
    let h = (wo + wi).normalize();
    let a = alpha(s.roughness);
    let ac = alpha(s.coat_roughness);
    let coat_f = s.clearcoat * schlick([0.04; 3], wo.dot(h))[0];
    let coat_view = s.clearcoat * schlick([0.04; 3], wo.z)[0];
    let under = 1.0 - coat_view;

    let diffuse = scale(
        s.base,
        (1.0 - s.metallic) * std::f64::consts::FRAC_1_PI * wi.z * under,
    );
    let d = ggx_d(h, a);
    let g = smith_g2(wo, wi, a);
    let spec = scale(schlick(f0(s), wo.dot(h)), d * g / (4.0 * wo.z) * under);
    let coat = if s.clearcoat > 0.0 {
        coat_f * ggx_d(h, ac) * smith_g2(wo, wi, ac) / (4.0 * wo.z)
    } else {
        0.0
    };
    let f = add(add(diffuse, spec), [coat; 3]);

    let [pd, ps, pc] = lobe_weights(s, wo);
    let pdf = pd * cosine_pdf(wi)
        + ps * visible_reflection_pdf(wo, h, a)
        + pc * visible_reflection_pdf(wo, h, ac);
    (f, pdf)
}

/// One scattered direction from the opaque lobes: world direction, throughput weight
/// (BSDF × cosine ÷ density) and the density.
fn sample_opaque(
    s: &Surface,
    frame: &Basis,
    wo_world: Vec3,
    ng: Vec3,
    rng: &mut Rng,
) -> Option<(Vec3, C3, f64)> {
    let wo = frame.to_local(wo_world);
    if wo.z <= 0.0 {
        return None;
    }
    let [pd, ps, _] = lobe_weights(s, wo);
    let u = rng.next_f64();
    let (u1, u2) = (rng.next_f64(), rng.next_f64());
    let wi = if u < pd {
        cosine_hemisphere(u1, u2)
    } else {
        let a = if u < pd + ps {
            alpha(s.roughness)
        } else {
            alpha(s.coat_roughness)
        };
        reflect(wo, sample_visible_normal(wo, a, u1, u2))
    };
    if wi.z <= 0.0 {
        return None;
    }
    let world = frame.to_world(wi);
    // Below the true surface even if above the shading normal: the light would have to
    // pass through the body.
    if world.dot(ng) <= 0.0 {
        return None;
    }
    let (f, pdf) = eval_opaque(s, wo, wi);
    if pdf <= 0.0 {
        return None;
    }
    Some((world, scale(f, 1.0 / pdf), pdf))
}

/// One scattered direction through or off a dielectric interface.
fn sample_glass(s: &Surface, ns: Vec3, ng: Vec3, wo: Vec3, rng: &mut Rng) -> Option<(Vec3, C3)> {
    // Which side the path is on decides the indices: the normals point out of the body.
    let entering = wo.dot(ng) > 0.0;
    let (n, eta_out_over_in) = if entering {
        (ns, s.ior)
    } else {
        (-ns, 1.0 / s.ior)
    };
    let frame = Basis::new(n);
    let wo_l = frame.to_local(wo);
    if wo_l.z <= 0.0 {
        return None;
    }
    let a = alpha(s.roughness);
    let h = if s.roughness < 0.02 {
        Vec3::Z
    } else {
        sample_visible_normal(wo_l, a, rng.next_f64(), rng.next_f64())
    };
    let cos = wo_l.dot(h);
    let fresnel = fresnel_dielectric(cos, eta_out_over_in);
    let shadowing = |wi: Vec3| {
        if s.roughness < 0.02 {
            1.0
        } else {
            smith_g2(wo_l, wi, a) / smith_g1(wo_l, a).max(1e-9)
        }
    };
    if rng.next_f64() < fresnel {
        let wi = reflect(wo_l, h);
        if wi.z <= 0.0 {
            return None;
        }
        Some((frame.to_world(wi), [shadowing(wi); 3]))
    } else {
        let wi = refract(wo_l, h, 1.0 / eta_out_over_in)?;
        if wi.z >= 0.0 {
            return None;
        }
        // Tinted on each crossing by the square root of the colour, so a ray through a
        // pane comes out the colour the appearance names.
        let tint = s.base.map(|c| c.sqrt());
        Some((frame.to_world(wi), scale(tint, shadowing(wi))))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn surface(base: f64, metallic: f64, roughness: f64) -> Surface {
        Surface {
            base: [base; 3],
            metallic,
            roughness,
            transmission: 0.0,
            ior: 1.5,
            clearcoat: 0.0,
            coat_roughness: 0.05,
            emission: BLACK,
        }
    }

    /// The white furnace: a surface that absorbs nothing, lit by a uniform environment
    /// of radiance one, reflects at most one. Estimated by sampling the lobes, which also
    /// checks that weights and densities agree.
    fn albedo(s: &Surface, wo: Vec3, n: usize) -> f64 {
        let frame = Basis::new(Vec3::Z);
        let mut rng = Rng::new(11, 5);
        let mut sum = 0.0;
        for _ in 0..n {
            if let Some((_, w, _)) = sample_opaque(s, &frame, wo, Vec3::Z, &mut rng) {
                sum += w[0];
            }
        }
        sum / n as f64
    }

    #[test]
    fn a_white_diffuse_surface_reflects_everything_and_nothing_more() {
        let wo = Vec3::new(0.3, 0.0, 0.95).normalize();
        let a = albedo(&surface(1.0, 0.0, 1.0), wo, 40_000);
        // A white dielectric is diffuse plus a few per cent of specular on top, which
        // is the familiar small energy gain of the simple model.
        assert!(a > 0.97 && a < 1.1, "{a}");
    }

    #[test]
    fn a_rough_white_metal_loses_only_what_single_scattering_cannot_return() {
        for roughness in [0.1, 0.5, 0.9] {
            let wo = Vec3::new(0.4, 0.0, 0.9).normalize();
            let a = albedo(&surface(1.0, 1.0, roughness), wo, 40_000);
            // Single-scattering GGX loses the light that would have bounced between
            // facets: little when smooth, more than half at the roughest. Checked against
            // quadrature of the same lobe above; this pins the trend.
            let floor = 1.0 - 0.65 * roughness;
            assert!(a > floor && a <= 1.01, "roughness {roughness}: {a}");
        }
    }

    /// ∫ f cos dω by quadrature, independent of any sampling.
    fn integrated_albedo(s: &Surface, wo: Vec3) -> f64 {
        let n = 400;
        let mut sum = 0.0;
        for i in 0..n {
            for j in 0..n {
                let theta = (i as f64 + 0.5) / n as f64 * std::f64::consts::FRAC_PI_2;
                let phi = (j as f64 + 0.5) / n as f64 * std::f64::consts::TAU;
                let wi = Vec3::new(
                    theta.sin() * phi.cos(),
                    theta.sin() * phi.sin(),
                    theta.cos(),
                );
                let (f, _) = eval_opaque(s, wo, wi);
                sum += f[0]
                    * theta.sin()
                    * (std::f64::consts::FRAC_PI_2 / n as f64)
                    * (std::f64::consts::TAU / n as f64);
            }
        }
        sum
    }

    #[test]
    fn sampling_and_quadrature_agree_on_the_albedo() {
        let wo = Vec3::new(0.4, 0.0, 0.9).normalize();
        for (metallic, roughness) in [(1.0, 0.3), (1.0, 0.9), (0.0, 0.5)] {
            let s = surface(0.8, metallic, roughness);
            let (a, b) = (albedo(&s, wo, 60_000), integrated_albedo(&s, wo));
            assert!((a - b).abs() < 0.02, "sampled {a}, integrated {b}");
        }
    }

    #[test]
    fn evaluation_and_sampling_agree_on_the_density() {
        // ∫ pdf dω over the hemisphere ≈ 1 minus what falls below it.
        let s = surface(0.6, 0.3, 0.4);
        let wo = Vec3::new(0.5, 0.2, 0.8).normalize();
        let n = 200;
        let mut integral = 0.0;
        for i in 0..n {
            for j in 0..n {
                let theta = (i as f64 + 0.5) / n as f64 * std::f64::consts::FRAC_PI_2;
                let phi = (j as f64 + 0.5) / n as f64 * std::f64::consts::TAU;
                let wi = Vec3::new(
                    theta.sin() * phi.cos(),
                    theta.sin() * phi.sin(),
                    theta.cos(),
                );
                let (_, pdf) = eval_opaque(&s, wo, wi);
                integral += pdf
                    * theta.sin()
                    * (std::f64::consts::FRAC_PI_2 / n as f64)
                    * (std::f64::consts::TAU / n as f64);
            }
        }
        assert!(integral > 0.9 && integral < 1.01, "{integral}");
    }

    #[test]
    fn clear_glass_mostly_transmits_head_on() {
        let s = Surface {
            transmission: 1.0,
            ..surface(1.0, 0.0, 0.0)
        };
        let mut rng = Rng::new(3, 3);
        let mut through = 0;
        for _ in 0..10_000 {
            let (wi, _) = sample_glass(&s, Vec3::Z, Vec3::Z, Vec3::Z, &mut rng).unwrap();
            if wi.z < 0.0 {
                through += 1;
            }
        }
        let f = through as f64 / 10_000.0;
        assert!((f - 0.96).abs() < 0.01, "{f}");
    }
}
