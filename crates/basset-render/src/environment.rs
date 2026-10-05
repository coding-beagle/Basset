//! The light a scene is lit by: a sky and a handful of distant lights, as Fusion's
//! environment library has them (Photo Booth, Sharp Highlights, a sunny sky, …).
//!
//! # Why procedural
//!
//! Fusion lights a render with photographed HDR panoramas. Those are tens of megabytes
//! each and would be the only binary assets in the repository, so the environments here
//! are described instead: a sky that is a gradient from the horizon up to the zenith and
//! down to the nadir, and a few distant lights — discs of uniform radiance a few degrees
//! to a few tens of degrees across — standing in for the soft boxes of a studio or the
//! sun. That turns out to be most of what makes a product shot: the broad soft
//! reflections of the boxes running along a polished edge, the sky's gradient in a
//! curved face, and a floor for the part to stand on.
//!
//! The description is also exactly what both renderers can use. The path tracer samples
//! the lights directly (they are what would otherwise make it noisy) and meets the sky
//! by escaping into it; the viewport's environment shading evaluates the same gradient
//! and lights in closed form, so the raster preview and the trace agree on where every
//! highlight is.
//!
//! # The sky
//!
//! [`sky`] is the gradient, and the formula is shared with the viewport's shader, which
//! must not drift from it: `t = sqrt(|z|)` of the direction's vertical component, then
//! horizon → zenith above the horizon and horizon → nadir below it. The square root
//! keeps the horizon band narrow, as a real sky's is.

use std::f64::consts::PI;

use basset_math::Vec3;
use serde::{Deserialize, Serialize};

use crate::color::{Rgb, lerp, scale};

/// The environments a scene can be lit by, as Fusion lists them.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum EnvironmentKind {
    /// A bright white studio: two large soft boxes in front and one overhead over a white
    /// floor. Fusion's default, and the kindest to every finish.
    #[default]
    PhotoBooth,
    /// A dark studio with narrow strip lights, for crisp lines along polished edges.
    SharpHighlights,
    /// An evenly lit grey room with one large overhead box; flat, for judging form.
    GreyRoom,
    /// A warm key and a soft warm fill.
    WarmLight,
    /// A cool, bluish studio.
    CoolLight,
    /// Blue sky over a ground, with the sun high and to the side.
    ClearSky,
    /// A low orange sun under a dusk sky.
    Dusk,
    /// Nearly black, with two coloured rim lights: for glow and silhouette.
    DarkRoom,
}

impl EnvironmentKind {
    pub const ALL: [EnvironmentKind; 8] = [
        EnvironmentKind::PhotoBooth,
        EnvironmentKind::SharpHighlights,
        EnvironmentKind::GreyRoom,
        EnvironmentKind::WarmLight,
        EnvironmentKind::CoolLight,
        EnvironmentKind::ClearSky,
        EnvironmentKind::Dusk,
        EnvironmentKind::DarkRoom,
    ];

    pub fn name(self) -> &'static str {
        match self {
            EnvironmentKind::PhotoBooth => "Photo Booth",
            EnvironmentKind::SharpHighlights => "Sharp Highlights",
            EnvironmentKind::GreyRoom => "Grey Room",
            EnvironmentKind::WarmLight => "Warm Light",
            EnvironmentKind::CoolLight => "Cool Light",
            EnvironmentKind::ClearSky => "Clear Sky",
            EnvironmentKind::Dusk => "Dusk",
            EnvironmentKind::DarkRoom => "Dark Room",
        }
    }

    /// The kind a typed name means, forgiving case and spacing.
    pub fn from_name(name: &str) -> Option<EnvironmentKind> {
        let fold = |s: &str| -> String {
            s.chars()
                .filter(|c| c.is_ascii_alphanumeric())
                .map(|c| c.to_ascii_lowercase())
                .collect()
        };
        let wanted = fold(name);
        Self::ALL.into_iter().find(|k| fold(k.name()) == wanted)
    }

    /// The environment as the renderers read it, before brightness and rotation.
    pub fn describe(self) -> Environment {
        // Azimuths are measured from +X towards +Y. The default isometric view looks from
        // azimuth -45°, so a key light around -100° sits in front of the model and to the
        // viewer's left, where a photographer puts one.
        let light = |azimuth: f64, elevation: f64, radius: f64, radiance: Rgb| Light {
            azimuth_deg: azimuth,
            elevation_deg: elevation,
            radius_deg: radius,
            radiance,
        };
        match self {
            EnvironmentKind::PhotoBooth => Environment {
                zenith: [0.72, 0.72, 0.72],
                horizon: [0.62, 0.62, 0.63],
                nadir: [0.34, 0.34, 0.35],
                lights: vec![
                    light(-105.0, 32.0, 20.0, [6.0, 6.0, 5.9]),
                    light(5.0, 28.0, 16.0, [3.4, 3.4, 3.45]),
                    light(-50.0, 78.0, 18.0, [5.0, 5.0, 5.0]),
                ],
                ground: [0.78, 0.78, 0.78],
            },
            EnvironmentKind::SharpHighlights => Environment {
                zenith: [0.05, 0.05, 0.055],
                horizon: [0.10, 0.10, 0.11],
                nadir: [0.03, 0.03, 0.03],
                lights: vec![
                    light(-110.0, 25.0, 5.0, [60.0, 60.0, 60.0]),
                    light(20.0, 22.0, 4.0, [45.0, 46.0, 48.0]),
                    light(-45.0, 70.0, 6.0, [35.0, 35.0, 35.0]),
                    light(150.0, 18.0, 4.0, [40.0, 40.0, 42.0]),
                ],
                ground: [0.35, 0.35, 0.36],
            },
            EnvironmentKind::GreyRoom => Environment {
                zenith: [0.62, 0.62, 0.62],
                horizon: [0.55, 0.55, 0.55],
                nadir: [0.35, 0.35, 0.35],
                lights: vec![light(-60.0, 85.0, 30.0, [1.6, 1.6, 1.6])],
                ground: [0.5, 0.5, 0.5],
            },
            EnvironmentKind::WarmLight => Environment {
                zenith: [0.42, 0.34, 0.26],
                horizon: [0.55, 0.43, 0.32],
                nadir: [0.22, 0.17, 0.12],
                lights: vec![
                    light(-95.0, 35.0, 10.0, [14.0, 9.5, 5.5]),
                    light(15.0, 20.0, 25.0, [1.2, 0.95, 0.7]),
                ],
                ground: [0.62, 0.52, 0.42],
            },
            EnvironmentKind::CoolLight => Environment {
                zenith: [0.36, 0.45, 0.58],
                horizon: [0.48, 0.55, 0.64],
                nadir: [0.18, 0.21, 0.26],
                lights: vec![
                    light(-100.0, 38.0, 12.0, [8.0, 9.5, 12.0]),
                    light(10.0, 25.0, 22.0, [1.1, 1.3, 1.6]),
                ],
                ground: [0.5, 0.55, 0.62],
            },
            EnvironmentKind::ClearSky => Environment {
                zenith: [0.16, 0.33, 0.80],
                horizon: [0.72, 0.82, 0.95],
                nadir: [0.20, 0.18, 0.15],
                // The real sun is half a degree across; a degree and a half keeps its
                // shadows from being razor-edged at the size a part is.
                lights: vec![light(-120.0, 55.0, 1.5, [2400.0, 2250.0, 2000.0])],
                ground: [0.45, 0.42, 0.38],
            },
            EnvironmentKind::Dusk => Environment {
                zenith: [0.10, 0.12, 0.30],
                horizon: [0.95, 0.52, 0.28],
                nadir: [0.10, 0.07, 0.06],
                lights: vec![light(-80.0, 8.0, 2.5, [420.0, 190.0, 70.0])],
                ground: [0.4, 0.33, 0.28],
            },
            EnvironmentKind::DarkRoom => Environment {
                zenith: [0.012, 0.012, 0.015],
                horizon: [0.02, 0.02, 0.025],
                nadir: [0.005, 0.005, 0.005],
                lights: vec![
                    light(150.0, 30.0, 8.0, [4.0, 12.0, 30.0]),
                    light(-20.0, 25.0, 8.0, [30.0, 10.0, 4.0]),
                    light(-100.0, 45.0, 15.0, [1.2, 1.2, 1.2]),
                ],
                ground: [0.25, 0.25, 0.25],
            },
        }
    }
}

/// A distant light as an environment describes it, in degrees.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Light {
    pub azimuth_deg: f64,
    pub elevation_deg: f64,
    /// Angular radius of the disc.
    pub radius_deg: f64,
    /// Linear radiance across the disc.
    pub radiance: Rgb,
}

/// An environment as described: the sky's three colours (linear radiance), its lights,
/// and the albedo of the floor it puts under the model.
#[derive(Clone, Debug, PartialEq)]
pub struct Environment {
    pub zenith: Rgb,
    pub horizon: Rgb,
    pub nadir: Rgb,
    pub lights: Vec<Light>,
    pub ground: Rgb,
}

/// A distant light in world space, ready to shade with.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DistantLight {
    /// Unit vector towards the light.
    pub direction: Vec3,
    /// Angular radius, radians.
    pub angular_radius: f64,
    pub radiance: Rgb,
    /// `cos(angular_radius)`, which is what a direction is tested against.
    pub cos_radius: f64,
    /// The solid angle of the disc, `2π(1 − cos θ)`: what sampling it uniformly divides by.
    pub solid_angle: f64,
}

impl DistantLight {
    pub fn new(direction: Vec3, angular_radius: f64, radiance: Rgb) -> Self {
        let cos_radius = angular_radius.cos();
        Self {
            direction: direction.normalize(),
            angular_radius,
            radiance,
            cos_radius,
            solid_angle: 2.0 * PI * (1.0 - cos_radius),
        }
    }

    /// Whether a direction falls on the disc.
    pub fn covers(&self, dir: Vec3) -> bool {
        dir.dot(self.direction) >= self.cos_radius
    }

    /// Irradiance the disc delivers to a surface facing it, `radiance · π · sin²θ`: the
    /// cosine-weighted integral over the cap, exactly. The viewport's `DistantLight`
    /// uses the same definition, so the two renderers light a face equally.
    pub fn irradiance(&self) -> Rgb {
        let s = self.angular_radius.sin();
        scale(self.radiance, (PI * s * s) as f32)
    }
}

/// An environment with brightness and rotation applied: what the tracer and the viewport
/// both shade with.
#[derive(Clone, Debug, PartialEq)]
pub struct Lighting {
    pub zenith: Rgb,
    pub horizon: Rgb,
    pub nadir: Rgb,
    pub lights: Vec<DistantLight>,
    pub ground: Rgb,
}

impl Lighting {
    /// `exposure` multiplies every radiance; `rotation` turns the lights about +Z
    /// (radians, counter-clockwise from above). The sky is symmetric about Z and does
    /// not care.
    pub fn new(environment: &Environment, exposure: f32, rotation: f64) -> Self {
        let lights = environment
            .lights
            .iter()
            .map(|l| {
                let azimuth = l.azimuth_deg.to_radians() + rotation;
                let elevation = l.elevation_deg.to_radians();
                let direction = Vec3::new(
                    elevation.cos() * azimuth.cos(),
                    elevation.cos() * azimuth.sin(),
                    elevation.sin(),
                );
                DistantLight::new(
                    direction,
                    l.radius_deg.to_radians(),
                    scale(l.radiance, exposure),
                )
            })
            .collect();
        Self {
            zenith: scale(environment.zenith, exposure),
            horizon: scale(environment.horizon, exposure),
            nadir: scale(environment.nadir, exposure),
            lights,
            ground: environment.ground,
        }
    }

    pub fn sky(&self, dir: Vec3) -> Rgb {
        sky(self.zenith, self.horizon, self.nadir, dir)
    }

    /// Everything an escaping ray sees: the sky, and any light it runs into.
    pub fn radiance(&self, dir: Vec3) -> Rgb {
        let mut c = self.sky(dir);
        for l in &self.lights {
            if l.covers(dir) {
                c = crate::color::add(c, l.radiance);
            }
        }
        c
    }
}

/// The sky gradient; see the module documentation. Shared, formula for formula, with
/// the viewport's environment shader.
pub fn sky(zenith: Rgb, horizon: Rgb, nadir: Rgb, dir: Vec3) -> Rgb {
    let z = dir.z.clamp(-1.0, 1.0);
    let t = z.abs().sqrt() as f32;
    if z >= 0.0 {
        lerp(horizon, zenith, t)
    } else {
        lerp(horizon, nadir, t)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn names_round_trip() {
        for k in EnvironmentKind::ALL {
            assert_eq!(EnvironmentKind::from_name(k.name()), Some(k));
        }
        assert_eq!(
            EnvironmentKind::from_name("photo booth"),
            Some(EnvironmentKind::PhotoBooth)
        );
        assert_eq!(EnvironmentKind::from_name("nowhere"), None);
    }

    #[test]
    fn the_sky_runs_from_nadir_through_horizon_to_zenith() {
        let (z, h, n) = ([1.0; 3], [0.5; 3], [0.0; 3]);
        assert_eq!(sky(z, h, n, Vec3::Z), z);
        assert_eq!(sky(z, h, n, Vec3::X), h);
        assert_eq!(sky(z, h, n, -Vec3::Z), n);
        let up = sky(z, h, n, Vec3::new(1.0, 0.0, 0.25).normalize())[0];
        assert!(up > 0.5 && up < 1.0);
    }

    #[test]
    fn rotation_turns_the_lights_about_z() {
        let env = EnvironmentKind::ClearSky.describe();
        let a = Lighting::new(&env, 1.0, 0.0);
        let b = Lighting::new(&env, 1.0, std::f64::consts::FRAC_PI_2);
        let (da, db) = (a.lights[0].direction, b.lights[0].direction);
        assert!((da.z - db.z).abs() < 1e-12);
        let turned = Vec3::new(-da.y, da.x, da.z);
        assert!((turned - db).length() < 1e-12);
    }

    #[test]
    fn a_light_covers_its_own_disc_and_nothing_else() {
        let l = DistantLight::new(Vec3::Z, 10f64.to_radians(), [1.0; 3]);
        assert!(l.covers(Vec3::Z));
        assert!(l.covers(Vec3::new(
            0.0,
            9f64.to_radians().sin(),
            9f64.to_radians().cos()
        )));
        assert!(!l.covers(Vec3::new(
            0.0,
            11f64.to_radians().sin(),
            11f64.to_radians().cos()
        )));
    }
}
