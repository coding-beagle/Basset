//! How the meshes of a [`crate::Scene`] are lit.
//!
//! The modelling view uses [`Lighting::Studio`]: a headlight and a key light that follow the
//! camera, tuned so every face reads clearly whatever the angle. [`Lighting::Environment`]
//! is for looking at what a part will look like: a sky that surrounds the model and a few
//! distant lights, shaded physically and tone mapped. It is a preview of what an offline
//! renderer would make of the same environment, so the sky and the lights are defined
//! here precisely enough for one to reproduce them — [`EnvironmentLight::sky`] and
//! [`DistantLight::irradiance`] are the definitions, not approximations of them.
//!
//! World up is +Z, as everywhere in Basset.

use std::f64::consts::PI;

use basset_math::Vec3;

#[derive(Debug, Clone, PartialEq, Default)]
pub enum Lighting {
    /// The modelling view's lighting: legible rather than realistic.
    #[default]
    Studio,
    /// Physically based shading under a sky and distant lights.
    Environment(EnvironmentLight),
}

#[derive(Debug, Clone, PartialEq)]
pub struct EnvironmentLight {
    /// Linear radiance of the sky straight up, at the horizon, and straight down.
    pub zenith: [f32; 3],
    pub horizon: [f32; 3],
    pub nadir: [f32; 3],
    /// Up to [`EnvironmentLight::MAX_LIGHTS`] distant lights: soft boxes and suns. Any
    /// beyond that are ignored.
    pub lights: Vec<DistantLight>,
    /// Multiplies everything before tone mapping (2^EV).
    pub exposure: f32,
    /// Draw the sky gradient behind the model instead of clearing to
    /// [`crate::Scene::background`].
    pub sky_background: bool,
}

/// A light far enough away that only its direction and apparent size matter: a disc of
/// uniform `radiance` subtending `angular_radius` about `direction`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DistantLight {
    /// Unit vector in world space, from the surface towards the light.
    pub direction: Vec3,
    /// Half the angle the disc subtends, in radians. Zero is a point-like sun; a soft box
    /// is a few tenths.
    pub angular_radius: f32,
    /// Linear radiance of the disc.
    pub radiance: [f32; 3],
}

impl DistantLight {
    /// Irradiance the light delivers to a surface facing it: `radiance · π · sin²(angular_radius)`.
    ///
    /// That is the exact projected solid angle of a disc centred on the normal, so a
    /// surface facing the light receives exactly this; one tilted by `θ` is given
    /// `irradiance · cos θ`, treating the disc as a point for the purpose of the tilt.
    /// The definition rather than the plain solid angle `2π(1 − cos r)` is used because
    /// it is what a path tracer integrating the same disc over a facing surface measures.
    pub fn irradiance(&self) -> [f32; 3] {
        let r = f64::from(self.angular_radius).clamp(0.0, PI / 2.0);
        let factor = (PI * r.sin().powi(2)) as f32;
        self.radiance.map(|c| c * factor)
    }
}

impl EnvironmentLight {
    /// How many [`DistantLight`]s the raster preview shades with.
    pub const MAX_LIGHTS: usize = 4;

    /// Radiance of the sky seen along the unit `direction`.
    ///
    /// Blends from the horizon to the zenith (or nadir) by the square root of the height,
    /// so the horizon band is narrow and most of the dome is near the pole colour, as a
    /// real sky's is. The shader evaluates exactly this formula.
    pub fn sky(&self, direction: Vec3) -> [f32; 3] {
        let z = direction.z.clamp(-1.0, 1.0);
        let t = z.abs().sqrt() as f32;
        let pole = if z >= 0.0 { self.zenith } else { self.nadir };
        std::array::from_fn(|i| self.horizon[i] * (1.0 - t) + pole[i] * t)
    }

    /// Cosine-weighted irradiance of the sky about `normal`, divided by π: the radiance a
    /// white Lambertian surface facing `normal` reflects. An approximation; see
    /// [`sky_irradiance_coefficients`].
    #[cfg(test)]
    pub(crate) fn sky_irradiance(&self, normal: Vec3) -> [f32; 3] {
        let z = normal.z.clamp(-1.0, 1.0);
        let basis = legendre_basis(z);
        let coefficients = sky_irradiance_coefficients(self);
        std::array::from_fn(|i| {
            (0..4)
                .map(|l| f64::from(coefficients[l][i]) * basis[l])
                .sum::<f64>() as f32
        })
    }
}

/// Legendre polynomials P0, P1, P2 and P4 of `z`, the basis the sky irradiance is
/// expanded in. The shader evaluates the same four.
#[cfg(test)]
fn legendre_basis(z: f64) -> [f64; 4] {
    let z2 = z * z;
    [
        1.0,
        z,
        1.5 * z2 - 0.5,
        (35.0 * z2 * z2 - 30.0 * z2 + 3.0) / 8.0,
    ]
}

/// Coefficients of the sky's irradiance about a normal, over π, in the Legendre
/// polynomials P0, P1, P2 and P4 of the normal's height.
///
/// The sky depends on height alone, so it is a sum of zonal harmonics `c_l P_l(z)`, and by
/// the Funk–Hecke theorem convolving it with the clamped cosine scales each by
/// `λ_l = 2π ∫₀¹ t P_l(t) dt` — π, 2π/3, π/4, 0 and −π/24 for l = 0…4, and small and
/// shrinking beyond. With `A = zenith − horizon`, `B = nadir − horizon` and
/// `I_l = ∫₀¹ √u P_l(u) du` (2/3, 2/5, 2/21, −2/77 for l = 0, 1, 2, 4),
/// `c_l = (2l+1)/2 · (2·horizon·[l = 0] + I_l (A + (−1)^l B))`, which gives the constants
/// below once multiplied by `λ_l / π`. The terms dropped from l = 6 on are worth well under
/// a percent of the sky's brightness; the tests check that against the integral itself.
pub(crate) fn sky_irradiance_coefficients(env: &EnvironmentLight) -> [[f32; 3]; 4] {
    let channels: [[f64; 4]; 3] = std::array::from_fn(|i| {
        let horizon = f64::from(env.horizon[i]);
        let up = f64::from(env.zenith[i]) - horizon;
        let down = f64::from(env.nadir[i]) - horizon;
        [
            horizon + (up + down) / 3.0,
            0.4 * (up - down),
            5.0 / 84.0 * (up + down),
            3.0 / 616.0 * (up + down),
        ]
    });
    std::array::from_fn(|l| channels.map(|c| c[l] as f32))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sky(zenith: [f32; 3], horizon: [f32; 3], nadir: [f32; 3]) -> EnvironmentLight {
        EnvironmentLight {
            zenith,
            horizon,
            nadir,
            lights: Vec::new(),
            exposure: 1.0,
            sky_background: false,
        }
    }

    /// The irradiance about `normal`, over π, by brute-force quadrature of the sky over
    /// the hemisphere the normal faces.
    fn integrated(env: &EnvironmentLight, normal: Vec3) -> [f64; 3] {
        let (steps_theta, steps_phi) = (400, 400);
        let tangent = normal.any_orthonormal_vector();
        let bitangent = normal.cross(tangent);
        let mut sum = [0.0; 3];
        for i in 0..steps_theta {
            let theta = (f64::from(i) + 0.5) / f64::from(steps_theta) * PI / 2.0;
            for j in 0..steps_phi {
                let phi = (f64::from(j) + 0.5) / f64::from(steps_phi) * 2.0 * PI;
                let d = normal * theta.cos()
                    + (tangent * phi.cos() + bitangent * phi.sin()) * theta.sin();
                let weight = theta.cos() * theta.sin();
                let radiance = env.sky(d);
                for c in 0..3 {
                    sum[c] += f64::from(radiance[c]) * weight;
                }
            }
        }
        let cell = (PI / 2.0 / f64::from(steps_theta)) * (2.0 * PI / f64::from(steps_phi));
        sum.map(|s| s * cell / PI)
    }

    #[test]
    fn the_sky_runs_from_horizon_to_the_poles() {
        let env = sky([0.2, 0.4, 1.0], [1.0, 1.0, 1.0], [0.1, 0.1, 0.1]);
        assert_eq!(env.sky(Vec3::Z), [0.2, 0.4, 1.0]);
        assert_eq!(env.sky(Vec3::X), [1.0, 1.0, 1.0]);
        assert_eq!(env.sky(-Vec3::Z), [0.1, 0.1, 0.1]);
        // A quarter of the way up by height is half way by colour: the square root.
        let quarter = env.sky(Vec3::new(0.0, (1.0f64 - 0.0625).sqrt(), 0.25));
        approx::assert_relative_eq!(quarter[0], 0.6, epsilon = 1e-6);
    }

    #[test]
    fn a_uniform_sky_gives_exactly_its_own_radiance() {
        let env = sky([0.7; 3], [0.7; 3], [0.7; 3]);
        for normal in [Vec3::Z, Vec3::X, Vec3::new(0.3, -0.4, 0.5).normalize()] {
            for c in env.sky_irradiance(normal) {
                approx::assert_relative_eq!(c, 0.7, epsilon = 1e-6);
            }
        }
    }

    #[test]
    fn sky_irradiance_matches_the_integral() {
        let envs = [
            sky([0.2, 0.4, 1.0], [1.0, 0.9, 0.8], [0.05, 0.04, 0.03]),
            sky([1.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 1.0, 0.0]),
            sky([0.0; 3], [1.0; 3], [0.0; 3]),
        ];
        for env in &envs {
            for normal in [
                Vec3::Z,
                -Vec3::Z,
                Vec3::X,
                Vec3::new(0.0, 0.6, 0.8),
                Vec3::new(0.5, 0.5, -0.7).normalize(),
                Vec3::new(0.1, 0.0, 0.995).normalize(),
            ] {
                let approximated = env.sky_irradiance(normal);
                let exact = integrated(env, normal);
                for c in 0..3 {
                    assert!(
                        (f64::from(approximated[c]) - exact[c]).abs() < 0.01,
                        "{env:?} about {normal:?}: channel {c} is {} but integrates to {}",
                        approximated[c],
                        exact[c]
                    );
                }
            }
        }
    }

    #[test]
    fn a_facing_disc_delivers_its_projected_solid_angle() {
        let light = DistantLight {
            direction: Vec3::Z,
            angular_radius: 0.25,
            radiance: [2.0, 1.0, 0.0],
        };
        let factor = (PI * 0.25f64.sin().powi(2)) as f32;
        assert_eq!(light.irradiance(), [2.0 * factor, factor, 0.0]);
        // A disc covering the whole hemisphere is the uniform sky: π times its radiance.
        let dome = DistantLight {
            angular_radius: std::f32::consts::FRAC_PI_2,
            ..light
        };
        approx::assert_relative_eq!(dome.irradiance()[1], std::f32::consts::PI, epsilon = 1e-5);
    }
}
