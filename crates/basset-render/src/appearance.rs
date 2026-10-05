//! What a surface looks like: an [`Appearance`].
//!
//! The model is the metallic–roughness one every real-time engine and most production
//! renderers share, with the two extensions a product shot needs — light passing through
//! (glass, clear plastic) and a clear lacquer over the base (car paint, varnished wood) —
//! and a short list of procedural [`Pattern`]s for the finishes a flat colour cannot
//! give (a brushed plate, wood grain, a carbon weave, cast or powder-coated texture).
//! Every number is a fraction a user can reason about without a manual: how metallic, how
//! rough, how much light gets through, how much lacquer.
//!
//! An appearance is not a physical material. Fusion keeps the two apart for a reason this
//! codebase shares: the density and stiffness a study needs (`basset-fea`'s material library) are
//! not what makes a part look like aluminium, and a part painted red is still steel.

use serde::{Deserialize, Serialize};

use crate::color::Srgb;

/// The family an appearance belongs to; what the library browser groups by.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum Category {
    Metal,
    Paint,
    Plastic,
    Rubber,
    Glass,
    Wood,
    Stone,
    Emissive,
    #[default]
    Other,
}

impl Category {
    /// In the order the library lists them: what a mechanical part is most often made of
    /// first.
    pub const ALL: [Category; 9] = [
        Category::Metal,
        Category::Paint,
        Category::Plastic,
        Category::Rubber,
        Category::Glass,
        Category::Wood,
        Category::Stone,
        Category::Emissive,
        Category::Other,
    ];

    pub fn name(self) -> &'static str {
        match self {
            Category::Metal => "Metal",
            Category::Paint => "Paint",
            Category::Plastic => "Plastic",
            Category::Rubber => "Rubber",
            Category::Glass => "Glass",
            Category::Wood => "Wood",
            Category::Stone => "Stone & ceramic",
            Category::Emissive => "Emissive",
            Category::Other => "Other",
        }
    }
}

/// A procedural variation over the surface, evaluated from the world position of the
/// point being shaded. Procedural rather than image textures because there is nothing to
/// wrap an image with: the kernel's faces have no texture coordinates, and a pattern
/// defined in space needs none.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum Pattern {
    #[default]
    None,
    /// Fine streaks along world X, in roughness more than in colour, as a plate run
    /// through a finishing belt has.
    Brushed,
    /// Growth rings about the world X axis with the grain running along it, between the
    /// colour (early wood) and the second colour (late wood): a board lying along X
    /// shows long grain on its faces and rings on its ends.
    Wood,
    /// A two-by-two twill of tows, alternating the colour and the second colour.
    CarbonFibre,
    /// Small flecks of the second colour: cast iron, powder coat, granite, concrete.
    Speckle,
}

impl Pattern {
    pub const ALL: [Pattern; 5] = [
        Pattern::None,
        Pattern::Brushed,
        Pattern::Wood,
        Pattern::CarbonFibre,
        Pattern::Speckle,
    ];

    pub fn name(self) -> &'static str {
        match self {
            Pattern::None => "None",
            Pattern::Brushed => "Brushed",
            Pattern::Wood => "Wood grain",
            Pattern::CarbonFibre => "Carbon weave",
            Pattern::Speckle => "Speckle",
        }
    }

    /// Whether the pattern reads the second colour; what decides whether an editor
    /// offers it.
    pub fn uses_second_color(self) -> bool {
        !matches!(self, Pattern::None | Pattern::Brushed)
    }
}

/// How a surface looks. Every field defaults, so an appearance saved by a build that knew
/// fewer of them loads as the plain version of itself.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Appearance {
    /// Unique within a document and within the library: it is what a body's assignment
    /// refers to.
    pub name: String,
    pub category: Category,
    /// The base colour: the diffuse colour of a dielectric, the reflectance of a metal,
    /// the tint of glass.
    pub color: Srgb,
    /// 0 is a dielectric (paint, plastic, wood), 1 a bare metal. In between is a
    /// metallic paint's flake.
    pub metallic: f32,
    /// 0 is a mirror, 1 completely matte. Perceptual: it is squared before it reaches
    /// the microfacet distribution, so equal steps look like equal steps.
    pub roughness: f32,
    /// How much of the light that is not reflected passes through rather than being
    /// scattered back: 1 for glass, 0 for anything opaque.
    pub transmission: f32,
    /// Index of refraction of a transmissive surface: 1.5 for glass, 1.49 acrylic.
    pub ior: f32,
    /// Weight of a smooth clear lacquer over everything else.
    pub clearcoat: f32,
    /// Light given off, as a multiple of the base colour. Zero for anything that is not
    /// a lamp or an LED.
    pub emission: f32,
    pub pattern: Pattern,
    /// The pattern's second colour; see [`Pattern`].
    pub color2: Srgb,
    /// Size of one period of the pattern, in millimetres: the spacing of the rings, the
    /// width of a tow, the size of a fleck.
    pub pattern_scale: f32,
}

impl Default for Appearance {
    fn default() -> Self {
        Self::DEFAULT.clone()
    }
}

impl Appearance {
    /// What a body nobody has given an appearance looks like: the light satin grey the
    /// modelling viewport draws every body in, so a part looks the same in the render as
    /// it did while it was being drawn until someone decides otherwise.
    pub const DEFAULT: Appearance = Appearance {
        name: String::new(),
        category: Category::Other,
        // The viewport's `MeshInstance::DEFAULT_COLOR`, [0.62, 0.66, 0.70] linear.
        color: Srgb::hex(0xced4da),
        metallic: 0.0,
        roughness: 0.45,
        transmission: 0.0,
        ior: 1.5,
        clearcoat: 0.0,
        emission: 0.0,
        pattern: Pattern::None,
        color2: Srgb::BLACK,
        pattern_scale: 4.0,
    };

    /// The name an unnamed appearance goes by.
    pub const DEFAULT_NAME: &str = "Default";

    pub fn display_name(&self) -> &str {
        if self.name.is_empty() {
            Self::DEFAULT_NAME
        } else {
            &self.name
        }
    }

    /// Every number pulled into the range it means something in, so an appearance typed
    /// by hand into a file or a tool call cannot hand the tracer a negative roughness or
    /// an index of refraction below one.
    pub fn sanitised(mut self) -> Self {
        let unit = |v: f32| {
            if v.is_finite() {
                v.clamp(0.0, 1.0)
            } else {
                0.0
            }
        };
        self.metallic = unit(self.metallic);
        self.roughness = unit(self.roughness);
        self.transmission = unit(self.transmission);
        self.clearcoat = unit(self.clearcoat);
        self.ior = if self.ior.is_finite() {
            self.ior.clamp(1.0, 3.0)
        } else {
            1.5
        };
        self.emission = if self.emission.is_finite() {
            self.emission.clamp(0.0, 100.0)
        } else {
            0.0
        };
        self.pattern_scale = if self.pattern_scale.is_finite() {
            self.pattern_scale.clamp(0.01, 10_000.0)
        } else {
            4.0
        };
        self
    }

    /// Whether light gets through it, which the viewport draws translucent.
    pub fn is_transmissive(&self) -> bool {
        self.transmission > 0.0
    }

    /// The opacity the viewport draws a transmissive appearance with. Glass is not
    /// invisible in a shaded view: it keeps a little of itself, more as it gets rougher
    /// and more as less light gets through.
    pub fn viewport_alpha(&self) -> f32 {
        let clear = self.transmission * (1.0 - 0.6 * self.roughness);
        (1.0 - 0.8 * clear).clamp(0.15, 1.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_partial_appearance_loads_with_defaults_for_the_rest() {
        let a: Appearance =
            serde_json::from_str(r##"{ "name": "Red", "color": "#ff0000" }"##).unwrap();
        assert_eq!(a.name, "Red");
        assert_eq!(a.color, Srgb([255, 0, 0]));
        assert_eq!(a.roughness, Appearance::DEFAULT.roughness);
        assert_eq!(a.pattern, Pattern::None);
    }

    #[test]
    fn sanitising_pulls_numbers_into_range() {
        let a = Appearance {
            roughness: -1.0,
            metallic: 2.0,
            ior: 0.5,
            emission: f32::NAN,
            ..Appearance::default()
        }
        .sanitised();
        assert_eq!(a.roughness, 0.0);
        assert_eq!(a.metallic, 1.0);
        assert_eq!(a.ior, 1.0);
        assert_eq!(a.emission, 0.0);
    }

    #[test]
    fn the_default_appearance_matches_the_viewport_grey() {
        let linear = Appearance::DEFAULT.color.to_linear();
        for (got, want) in linear.iter().zip([0.62, 0.66, 0.70]) {
            assert!((got - want).abs() < 0.005, "{linear:?}");
        }
    }
}
