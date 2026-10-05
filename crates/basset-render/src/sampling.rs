//! Random numbers and the scattering functions the integrator samples: Lambert, the
//! GGX microfacet lobe (sampled through its visible normals), and Fresnel.
//!
//! Everything works in a local frame where the shading normal is +Z, which is what turns
//! the cosines into plain `z` components.

use std::f64::consts::{FRAC_1_PI, PI};

use basset_math::Vec3;

/// PCG32: small, fast and good enough that no pattern shows in an image. Seeded per pixel
/// and per pass from a hash, so a render is the same image every time it is made.
#[derive(Clone, Debug)]
pub struct Rng {
    state: u64,
    inc: u64,
}

impl Rng {
    pub fn new(seed: u64, stream: u64) -> Self {
        let mut rng = Self {
            state: 0,
            inc: (stream << 1) | 1,
        };
        rng.next_u32();
        rng.state = rng.state.wrapping_add(seed);
        rng.next_u32();
        rng
    }

    pub fn next_u32(&mut self) -> u32 {
        let old = self.state;
        self.state = old
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(self.inc);
        let xorshifted = (((old >> 18) ^ old) >> 27) as u32;
        let rot = (old >> 59) as u32;
        xorshifted.rotate_right(rot)
    }

    /// Uniform in `[0, 1)`.
    pub fn next_f64(&mut self) -> f64 {
        f64::from(self.next_u32()) * (1.0 / 4_294_967_296.0)
    }
}

/// A 64-bit mix (splitmix64's finaliser), for turning a pixel index and a pass number
/// into unrelated seeds.
pub fn hash(mut x: u64) -> u64 {
    x = (x ^ (x >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
    x = (x ^ (x >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
    x ^ (x >> 31)
}

/// An orthonormal frame about a normal (Duff et al.'s branchless construction).
#[derive(Clone, Copy, Debug)]
pub struct Basis {
    pub t: Vec3,
    pub b: Vec3,
    pub n: Vec3,
}

impl Basis {
    pub fn new(n: Vec3) -> Self {
        let sign = 1f64.copysign(n.z);
        let a = -1.0 / (sign + n.z);
        let b = n.x * n.y * a;
        Self {
            t: Vec3::new(1.0 + sign * n.x * n.x * a, sign * b, -sign * n.x),
            b: Vec3::new(b, sign + n.y * n.y * a, -n.y),
            n,
        }
    }

    pub fn to_local(self, v: Vec3) -> Vec3 {
        Vec3::new(v.dot(self.t), v.dot(self.b), v.dot(self.n))
    }

    pub fn to_world(self, v: Vec3) -> Vec3 {
        self.t * v.x + self.b * v.y + self.n * v.z
    }
}

pub fn cosine_hemisphere(u1: f64, u2: f64) -> Vec3 {
    let r = u1.sqrt();
    let phi = 2.0 * PI * u2;
    Vec3::new(r * phi.cos(), r * phi.sin(), (1.0 - u1).max(0.0).sqrt())
}

pub fn cosine_pdf(wi: Vec3) -> f64 {
    wi.z.max(0.0) * FRAC_1_PI
}

/// A direction inside a cone about +Z of the given `cos` half-angle, uniformly over its
/// solid angle.
pub fn uniform_cone(u1: f64, u2: f64, cos_max: f64) -> Vec3 {
    let cos = 1.0 - u1 * (1.0 - cos_max);
    let sin = (1.0 - cos * cos).max(0.0).sqrt();
    let phi = 2.0 * PI * u2;
    Vec3::new(sin * phi.cos(), sin * phi.sin(), cos)
}

/// The GGX (Trowbridge–Reitz) normal distribution for `alpha` = roughness².
pub fn ggx_d(h: Vec3, alpha: f64) -> f64 {
    let a2 = alpha * alpha;
    let c = h.z;
    if c <= 0.0 {
        return 0.0;
    }
    let d = c * c * (a2 - 1.0) + 1.0;
    a2 / (PI * d * d)
}

/// Smith's shadowing term for one direction.
pub fn smith_g1(w: Vec3, alpha: f64) -> f64 {
    let c = w.z.abs();
    if c <= 0.0 {
        return 0.0;
    }
    let a2 = alpha * alpha;
    let tan2 = (1.0 - c * c).max(0.0) / (c * c);
    2.0 / (1.0 + (1.0 + a2 * tan2).sqrt())
}

/// Shadowing–masking for both directions, height-correlated as Heitz recommends.
pub fn smith_g2(wo: Vec3, wi: Vec3, alpha: f64) -> f64 {
    let lambda = |w: Vec3| {
        let c = w.z.abs();
        if c <= 0.0 {
            return f64::INFINITY;
        }
        let tan2 = (1.0 - c * c).max(0.0) / (c * c);
        ((1.0 + alpha * alpha * tan2).sqrt() - 1.0) * 0.5
    };
    1.0 / (1.0 + lambda(wo) + lambda(wi))
}

/// A microfacet normal sampled from the normals visible from `wo` (Heitz 2018). Sampling
/// what the viewer can see, rather than every normal, wastes no samples on facets turned
/// away, which is most of the noise a rough metal would otherwise have at grazing angles.
pub fn sample_visible_normal(wo: Vec3, alpha: f64, u1: f64, u2: f64) -> Vec3 {
    let vh = Vec3::new(alpha * wo.x, alpha * wo.y, wo.z).normalize();
    let lensq = vh.x * vh.x + vh.y * vh.y;
    let t1 = if lensq > 0.0 {
        Vec3::new(-vh.y, vh.x, 0.0) / lensq.sqrt()
    } else {
        Vec3::X
    };
    let t2 = vh.cross(t1);
    let r = u1.sqrt();
    let phi = 2.0 * PI * u2;
    let p1 = r * phi.cos();
    let mut p2 = r * phi.sin();
    let s = 0.5 * (1.0 + vh.z);
    p2 = (1.0 - s) * (1.0 - p1 * p1).max(0.0).sqrt() + s * p2;
    let nh = t1 * p1 + t2 * p2 + vh * (1.0 - p1 * p1 - p2 * p2).max(0.0).sqrt();
    Vec3::new(alpha * nh.x, alpha * nh.y, nh.z.max(1e-9)).normalize()
}

/// The density of a reflected direction under visible-normal sampling, per solid angle.
pub fn visible_reflection_pdf(wo: Vec3, h: Vec3, alpha: f64) -> f64 {
    if wo.z <= 0.0 {
        return 0.0;
    }
    smith_g1(wo, alpha) * ggx_d(h, alpha) / (4.0 * wo.z)
}

/// Schlick's approximation for a coloured reflectance at normal incidence.
pub fn schlick(f0: [f64; 3], cos: f64) -> [f64; 3] {
    let m = (1.0 - cos.clamp(0.0, 1.0)).powi(5);
    f0.map(|f| f + (1.0 - f) * m)
}

/// Exact Fresnel reflectance of an uncoated dielectric interface for unpolarised light
/// arriving at `cos_i`, where `eta` is the index on the far side over the index on the
/// side the light arrives from (1.5 entering glass from air, 1/1.5 leaving it). Returns 1
/// under total internal reflection.
pub fn fresnel_dielectric(cos_i: f64, eta: f64) -> f64 {
    let cos_i = cos_i.clamp(0.0, 1.0);
    let sin_t2 = (1.0 - cos_i * cos_i) / (eta * eta);
    if sin_t2 >= 1.0 {
        return 1.0;
    }
    let cos_t = (1.0 - sin_t2).sqrt();
    let rs = (eta * cos_i - cos_t) / (eta * cos_i + cos_t);
    let rp = (cos_i - eta * cos_t) / (cos_i + eta * cos_t);
    0.5 * (rs * rs + rp * rp)
}

pub fn reflect(w: Vec3, n: Vec3) -> Vec3 {
    n * (2.0 * w.dot(n)) - w
}

/// `w` (pointing away from the surface) refracted through the interface with normal `n`
/// on its side, for an index ratio `eta` = incident over transmitted. `None` under total
/// internal reflection.
pub fn refract(w: Vec3, n: Vec3, eta: f64) -> Option<Vec3> {
    let cos_i = w.dot(n);
    let sin_t2 = eta * eta * (1.0 - cos_i * cos_i).max(0.0);
    if sin_t2 >= 1.0 {
        return None;
    }
    let cos_t = (1.0 - sin_t2).sqrt();
    Some((-w * eta + n * (eta * cos_i - cos_t)).normalize())
}

/// The power heuristic (β = 2) for combining two sampling strategies.
pub fn power_heuristic(pdf_a: f64, pdf_b: f64) -> f64 {
    let (a, b) = (pdf_a * pdf_a, pdf_b * pdf_b);
    if a + b <= 0.0 { 0.0 } else { a / (a + b) }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_basis_is_orthonormal_for_any_normal() {
        let mut rng = Rng::new(1, 2);
        for _ in 0..1000 {
            let n = Vec3::new(
                rng.next_f64() * 2.0 - 1.0,
                rng.next_f64() * 2.0 - 1.0,
                rng.next_f64() * 2.0 - 1.0,
            )
            .normalize();
            let b = Basis::new(n);
            assert!(b.t.dot(b.b).abs() < 1e-9 && b.t.dot(n).abs() < 1e-9);
            assert!((b.t.length() - 1.0).abs() < 1e-9 && (b.b.length() - 1.0).abs() < 1e-9);
            let v = Vec3::new(0.3, -0.5, 0.8);
            assert!((b.to_world(b.to_local(v)) - v).length() < 1e-9);
        }
    }

    #[test]
    fn the_ggx_distribution_projects_to_one() {
        // ∫ D(h) cos θh dω = 1, the defining normalisation of a microfacet distribution.
        for alpha in [0.05, 0.2, 0.6, 1.0] {
            let n = 400;
            let mut sum = 0.0;
            for i in 0..n {
                let theta = (i as f64 + 0.5) / n as f64 * PI / 2.0;
                let h = Vec3::new(theta.sin(), 0.0, theta.cos());
                sum += ggx_d(h, alpha) * theta.cos() * theta.sin() * (PI / 2.0 / n as f64);
            }
            sum *= 2.0 * PI;
            assert!((sum - 1.0).abs() < 0.02, "alpha {alpha}: {sum}");
        }
    }

    #[test]
    fn visible_normal_sampling_matches_its_pdf() {
        // Every sampled normal faces up, and most reflections stay above the surface;
        // how many do not grows with roughness (a third at alpha 0.8, seen this steeply).
        let mut rng = Rng::new(7, 3);
        let wo = Vec3::new(0.5, 0.0, 0.8).normalize();
        for alpha in [0.1, 0.4, 0.8] {
            let n = 20_000;
            let mut above = 0;
            for _ in 0..n {
                let h = sample_visible_normal(wo, alpha, rng.next_f64(), rng.next_f64());
                assert!(h.z > 0.0);
                if reflect(wo, h).z > 0.0 {
                    above += 1;
                }
            }
            let fraction = above as f64 / n as f64;
            assert!(
                fraction > 0.5 && fraction <= 1.0,
                "alpha {alpha}: {fraction}"
            );
        }
    }

    #[test]
    fn fresnel_is_four_percent_for_glass_head_on_and_total_past_the_critical_angle() {
        assert!((fresnel_dielectric(1.0, 1.5) - 0.04).abs() < 1e-3);
        assert!((fresnel_dielectric(1.0, 1.0 / 1.5) - 0.04).abs() < 1e-3);
        // From inside glass the critical angle is asin(1/1.5) ≈ 41.8°.
        assert!(fresnel_dielectric(30f64.to_radians().cos(), 1.0 / 1.5) < 1.0);
        assert_eq!(fresnel_dielectric(60f64.to_radians().cos(), 1.0 / 1.5), 1.0);
        let w = Vec3::new(60f64.to_radians().sin(), 0.0, 60f64.to_radians().cos());
        assert!(refract(w, Vec3::Z, 1.5).is_none());
    }

    #[test]
    fn refraction_obeys_snell() {
        let theta = 30f64.to_radians();
        let w = Vec3::new(theta.sin(), 0.0, theta.cos());
        let eta = 1.0 / 1.5;
        let t = refract(w, Vec3::Z, eta).unwrap();
        assert!(t.z < 0.0);
        let sin_t = (t.x * t.x + t.y * t.y).sqrt();
        assert!((sin_t - eta * theta.sin()).abs() < 1e-9);
    }
}
