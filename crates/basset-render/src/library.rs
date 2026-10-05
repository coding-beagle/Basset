//! The appearance library: about eighty finishes a mechanical part is commonly given,
//! grouped as Fusion's appearance browser groups them.
//!
//! The metals' colours are their measured reflectance at normal incidence (the F0 of the
//! physically based shading literature), converted to sRGB, so polished aluminium is
//! nearly white and gold is the warm yellow it actually reflects; it is the roughness that
//! makes satin and brushed finishes read as different from polished ones. Dielectrics are
//! their diffuse colour with the four per cent reflectance every paint and plastic has,
//! and a gloss paint is a matte one under a clear coat. Names are "Material - Finish", as
//! Fusion writes them, so the list sorts by material and a filter for either word finds
//! the entry.

use std::sync::OnceLock;

use crate::appearance::{Appearance, Category, Pattern};
use crate::color::Srgb;

/// Every library entry, in the order the browser lists them.
pub fn library() -> &'static [Appearance] {
    static LIBRARY: OnceLock<Vec<Appearance>> = OnceLock::new();
    LIBRARY.get_or_init(build)
}

/// The entry a name means, forgiving case, spaces, hyphens and the American spelling of
/// aluminium, as the material library's `find` does. Exact after that folding first;
/// failing that, the one entry whose name contains the query, so `anodized red` and
/// `walnut` resolve. Two candidates is an ambiguous name and finds nothing.
pub fn find(name: &str) -> Option<&'static Appearance> {
    let wanted = normalise(name);
    if wanted.is_empty() {
        return None;
    }
    let lib = library();
    if let Some(exact) = lib.iter().find(|a| normalise(&a.name) == wanted) {
        return Some(exact);
    }
    let mut candidates = lib.iter().filter(|a| normalise(&a.name).contains(&wanted));
    match (candidates.next(), candidates.next()) {
        (Some(one), None) => Some(one),
        _ => None,
    }
}

fn normalise(name: &str) -> String {
    let folded: String = name
        .chars()
        .filter(|c| c.is_ascii_alphanumeric())
        .map(|c| c.to_ascii_lowercase())
        .collect();
    folded
        .replace("aluminum", "aluminium")
        .replace("anodised", "anodized")
}

/// The starting point every entry is written against: an opaque dielectric of middling
/// roughness with nothing else going on.
fn base(name: &str, category: Category, color: u32) -> Appearance {
    Appearance {
        name: name.to_owned(),
        category,
        color: Srgb::hex(color),
        ..Appearance::DEFAULT
    }
}

fn metal(name: &str, color: u32, roughness: f32) -> Appearance {
    Appearance {
        metallic: 1.0,
        roughness,
        ..base(name, Category::Metal, color)
    }
}

fn brushed(name: &str, color: u32, roughness: f32) -> Appearance {
    Appearance {
        pattern: Pattern::Brushed,
        pattern_scale: 0.6,
        ..metal(name, color, roughness)
    }
}

/// Anodising is a dyed oxide over the metal: the colour is the dye and the sheen the
/// metal underneath, which a partly metallic satin gives well.
fn anodized(colour: &str, color: u32) -> Appearance {
    Appearance {
        metallic: 0.75,
        roughness: 0.32,
        ..base(
            &format!("Aluminium - Anodized {colour}"),
            Category::Metal,
            color,
        )
    }
}

fn gloss_paint(colour: &str, color: u32) -> Appearance {
    Appearance {
        roughness: 0.35,
        clearcoat: 1.0,
        ..base(&format!("Paint - Gloss {colour}"), Category::Paint, color)
    }
}

fn matte_paint(colour: &str, color: u32) -> Appearance {
    Appearance {
        roughness: 0.8,
        ..base(&format!("Paint - Matte {colour}"), Category::Paint, color)
    }
}

/// A metallic paint is flakes of metal in a tinted binder under a clear coat.
fn metallic_paint(colour: &str, color: u32) -> Appearance {
    Appearance {
        metallic: 0.6,
        roughness: 0.4,
        clearcoat: 1.0,
        ..base(
            &format!("Paint - Metallic {colour}"),
            Category::Paint,
            color,
        )
    }
}

fn powder_coat(colour: &str, color: u32, fleck: u32) -> Appearance {
    Appearance {
        roughness: 0.62,
        pattern: Pattern::Speckle,
        color2: Srgb::hex(fleck),
        pattern_scale: 0.35,
        ..base(&format!("Powder Coat - {colour}"), Category::Paint, color)
    }
}

fn plastic(name: &str, color: u32, roughness: f32) -> Appearance {
    Appearance {
        roughness,
        ..base(name, Category::Plastic, color)
    }
}

fn clear(name: &str, category: Category, color: u32, roughness: f32, ior: f32) -> Appearance {
    Appearance {
        transmission: 1.0,
        roughness,
        ior,
        ..base(name, category, color)
    }
}

fn rubber(name: &str, color: u32) -> Appearance {
    Appearance {
        roughness: 0.85,
        ..base(name, Category::Rubber, color)
    }
}

fn wood(name: &str, early: u32, late: u32, rings: f32, varnished: bool) -> Appearance {
    Appearance {
        roughness: if varnished { 0.45 } else { 0.7 },
        clearcoat: if varnished { 0.8 } else { 0.0 },
        pattern: Pattern::Wood,
        color2: Srgb::hex(late),
        pattern_scale: rings,
        ..base(name, Category::Wood, early)
    }
}

fn stone(name: &str, color: u32, fleck: u32, roughness: f32, scale: f32) -> Appearance {
    Appearance {
        roughness,
        pattern: Pattern::Speckle,
        color2: Srgb::hex(fleck),
        pattern_scale: scale,
        ..base(name, Category::Stone, color)
    }
}

fn emissive(name: &str, color: u32, strength: f32) -> Appearance {
    Appearance {
        emission: strength,
        roughness: 0.3,
        ..base(name, Category::Emissive, color)
    }
}

fn build() -> Vec<Appearance> {
    let mut lib = vec![
        // Metals. Reflectances from the usual measured tables: aluminium 0.91, silver
        // 0.97, iron 0.56, chromium 0.55, gold (1.0, 0.77, 0.34), copper (0.95, 0.64,
        // 0.54), titanium (0.54, 0.50, 0.45), linear.
        metal("Aluminium - Polished", 0xf5f6f6, 0.06),
        metal("Aluminium - Satin", 0xf0f1f2, 0.3),
        brushed("Aluminium - Brushed", 0xeef0f1, 0.24),
        metal("Aluminium - Bead Blasted", 0xe6e8ea, 0.55),
        Appearance {
            pattern: Pattern::Speckle,
            color2: Srgb::hex(0xb8bcbf),
            pattern_scale: 0.5,
            ..metal("Aluminium - Cast", 0xd9dcdf, 0.6)
        },
        anodized("Black", 0x2a2b2e),
        anodized("Silver", 0xc8ccd0),
        anodized("Red", 0xb3202a),
        anodized("Blue", 0x1f4fa8),
        anodized("Gold", 0xc9a24a),
        anodized("Green", 0x2e7d4a),
        anodized("Purple", 0x6a3a9a),
        anodized("Orange", 0xd9661e),
        metal("Steel - Polished", 0xc4c6c7, 0.06),
        metal("Steel - Satin", 0xc0c2c4, 0.32),
        brushed("Steel - Brushed", 0xc2c4c6, 0.26),
        metal("Steel - Black Oxide", 0x3a3b3d, 0.38),
        Appearance {
            pattern: Pattern::Speckle,
            color2: Srgb::hex(0x9ea2a6),
            pattern_scale: 0.4,
            ..metal("Steel - Galvanized", 0xc9ced3, 0.38)
        },
        metal("Stainless Steel - Polished", 0xd6d1cb, 0.05),
        metal("Stainless Steel - Satin", 0xd2cdc8, 0.28),
        brushed("Stainless Steel - Brushed", 0xd4cfca, 0.22),
        metal("Chrome", 0xc4c5c5, 0.02),
        metal("Nickel - Plated", 0xd8d1c3, 0.12),
        Appearance {
            pattern: Pattern::Speckle,
            color2: Srgb::hex(0x55575a),
            pattern_scale: 0.6,
            ..metal("Iron - Cast", 0x8f9194, 0.62)
        },
        metal("Titanium - Satin", 0xc2bbb2, 0.3),
        metal("Gold - Polished", 0xffe39d, 0.06),
        metal("Gold - Satin", 0xffe0a0, 0.3),
        metal("Silver - Polished", 0xfaf8f4, 0.05),
        metal("Copper - Polished", 0xfad1c2, 0.08),
        metal("Copper - Satin", 0xf5c8b5, 0.32),
        metal("Brass - Polished", 0xf2e5af, 0.08),
        metal("Brass - Satin", 0xeedda5, 0.32),
        metal("Bronze - Satin", 0xe7c3a0, 0.35),
        metal("Mirror", 0xfafafa, 0.0),
        // Paint.
        gloss_paint("White", 0xf2f2ef),
        gloss_paint("Black", 0x111214),
        gloss_paint("Red", 0xb5121b),
        gloss_paint("Blue", 0x1846a3),
        gloss_paint("Yellow", 0xf2c40f),
        gloss_paint("Green", 0x1f7a3a),
        gloss_paint("Orange", 0xe8611a),
        gloss_paint("Grey", 0x7a7e83),
        matte_paint("White", 0xe9e9e6),
        matte_paint("Black", 0x1a1a1c),
        matte_paint("Grey", 0x6f7378),
        matte_paint("Olive", 0x5b6236),
        metallic_paint("Silver", 0xbfc3c8),
        metallic_paint("Red", 0x9a1018),
        metallic_paint("Blue", 0x1b3f8f),
        metallic_paint("Graphite", 0x45484d),
        powder_coat("Black", 0x1c1d1f, 0x34363a),
        powder_coat("White", 0xe6e6e2, 0xcfcfca),
        powder_coat("Safety Yellow", 0xf0b80c, 0xc99607),
        // Plastic.
        plastic("Plastic - Glossy White", 0xf0f0ec, 0.15),
        plastic("Plastic - Glossy Black", 0x141517, 0.15),
        plastic("Plastic - Glossy Red", 0xc0161e, 0.15),
        plastic("Plastic - Glossy Blue", 0x1b55b8, 0.15),
        plastic("Plastic - Glossy Yellow", 0xf3c614, 0.15),
        plastic("Plastic - Glossy Green", 0x20914a, 0.15),
        plastic("Plastic - Matte White", 0xe8e8e4, 0.55),
        plastic("Plastic - Matte Black", 0x1d1e20, 0.55),
        plastic("Plastic - Matte Grey", 0x8a8d91, 0.55),
        Appearance {
            pattern: Pattern::Speckle,
            color2: Srgb::hex(0x2c2d30),
            pattern_scale: 0.25,
            ..plastic("ABS - Textured Black", 0x1b1c1e, 0.72)
        },
        plastic("ABS - Light Grey", 0xc9cbc8, 0.4),
        plastic("Nylon - White", 0xece9e0, 0.5),
        plastic("Nylon - Black", 0x222326, 0.5),
        plastic("Delrin - White", 0xf2f0ea, 0.3),
        plastic("PLA - Orange", 0xf07a20, 0.35),
        plastic("PETG - Teal", 0x1e8f8a, 0.25),
        clear("Acrylic - Clear", Category::Plastic, 0xffffff, 0.0, 1.49),
        clear("Acrylic - Smoked", Category::Plastic, 0x6b6e73, 0.0, 1.49),
        clear(
            "Polycarbonate - Clear",
            Category::Plastic,
            0xfdfeff,
            0.02,
            1.58,
        ),
        clear(
            "Plastic - Translucent Blue",
            Category::Plastic,
            0x5a8fe0,
            0.18,
            1.5,
        ),
        clear(
            "Plastic - Translucent Red",
            Category::Plastic,
            0xe0505a,
            0.18,
            1.5,
        ),
        clear(
            "Plastic - Frosted White",
            Category::Plastic,
            0xf4f4f4,
            0.45,
            1.5,
        ),
        // Rubber.
        rubber("Rubber - Black", 0x1a1a1b),
        rubber("Rubber - Grey", 0x5e6064),
        rubber("Silicone - Red", 0xb0362c),
        rubber("Silicone - Blue", 0x2f62b0),
        // Glass.
        clear("Glass - Clear", Category::Glass, 0xffffff, 0.0, 1.5),
        clear("Glass - Frosted", Category::Glass, 0xfbfbfb, 0.35, 1.5),
        clear("Glass - Tinted Blue", Category::Glass, 0x9fc3ee, 0.0, 1.5),
        clear("Glass - Tinted Green", Category::Glass, 0xa8dcb4, 0.0, 1.5),
        clear("Glass - Tinted Bronze", Category::Glass, 0xc9a27e, 0.0, 1.5),
        // Wood. Ring spacing in millimetres.
        wood("Wood - Oak", 0xc89a62, 0x9c6e3c, 3.0, false),
        wood("Wood - Oak Varnished", 0xc4904f, 0x8f5f2e, 3.0, true),
        wood("Wood - Walnut", 0x6e4a32, 0x3f2718, 2.5, false),
        wood("Wood - Walnut Varnished", 0x6a432a, 0x3a2213, 2.5, true),
        wood("Wood - Maple", 0xe3c79a, 0xcaa676, 2.0, false),
        wood("Wood - Cherry", 0xa95f3d, 0x7c3d22, 2.2, true),
        wood("Wood - Pine", 0xe2bf86, 0xb98a4d, 4.0, false),
        // Stone and ceramic.
        Appearance {
            roughness: 0.12,
            clearcoat: 0.6,
            ..base("Ceramic - White Glazed", Category::Stone, 0xf3f2ee)
        },
        Appearance {
            roughness: 0.5,
            ..base("Ceramic - Unglazed", Category::Stone, 0xd8cfc2)
        },
        stone("Concrete", 0x9a9893, 0x77746e, 0.9, 0.8),
        stone("Granite - Black", 0x1e1f21, 0x6c6d70, 0.22, 1.2),
        stone("Granite - Grey", 0x8e8e8c, 0x3a3a3a, 0.3, 1.2),
        stone("Marble - White", 0xeeece7, 0xc7c4bd, 0.18, 6.0),
        // Emissive.
        emissive("LED - White", 0xffffff, 4.0),
        emissive("LED - Warm White", 0xffd9a0, 4.0),
        emissive("LED - Red", 0xff2a1a, 4.0),
        emissive("LED - Green", 0x2aff4a, 4.0),
        emissive("LED - Blue", 0x2a6aff, 4.0),
        // Other.
        Appearance {
            roughness: 0.38,
            clearcoat: 1.0,
            pattern: Pattern::CarbonFibre,
            color2: Srgb::hex(0x34373c),
            pattern_scale: 2.0,
            ..base("Carbon Fibre - Twill", Category::Other, 0x15161a)
        },
        Appearance {
            roughness: 0.75,
            pattern: Pattern::CarbonFibre,
            color2: Srgb::hex(0x2c2f34),
            pattern_scale: 2.0,
            ..base("Carbon Fibre - Matte", Category::Other, 0x17181c)
        },
        Appearance {
            name: Appearance::DEFAULT_NAME.to_owned(),
            ..Appearance::DEFAULT
        },
    ];
    for a in &mut lib {
        *a = a.clone().sanitised();
    }
    lib
}

#[cfg(test)]
mod tests {
    use std::collections::HashSet;

    use super::*;

    #[test]
    fn names_are_unique_after_folding() {
        let mut seen = HashSet::new();
        for a in library() {
            assert!(seen.insert(normalise(&a.name)), "{} twice", a.name);
        }
    }

    #[test]
    fn every_entry_is_already_sane() {
        for a in library() {
            assert_eq!(*a, a.clone().sanitised(), "{}", a.name);
        }
    }

    #[test]
    fn every_category_has_entries() {
        for c in Category::ALL {
            assert!(
                library().iter().any(|a| a.category == c),
                "{} is empty",
                c.name()
            );
        }
    }

    #[test]
    fn names_resolve_forgivingly() {
        assert_eq!(
            find("aluminum - anodized red").map(|a| a.name.as_str()),
            Some("Aluminium - Anodized Red")
        );
        assert_eq!(
            find("anodised blue").map(|a| a.name.as_str()),
            Some("Aluminium - Anodized Blue")
        );
        assert_eq!(
            find("walnut varnished").map(|a| a.name.as_str()),
            Some("Wood - Walnut Varnished")
        );
        assert_eq!(find("chrome").map(|a| a.name.as_str()), Some("Chrome"));
        // Several glosses: ambiguous.
        assert!(find("gloss").is_none());
        assert!(find("").is_none());
        assert!(find("unobtainium").is_none());
    }

    #[test]
    fn the_default_is_in_the_library_under_its_name() {
        let d = find(Appearance::DEFAULT_NAME).expect("default entry");
        assert_eq!(d.color, Appearance::DEFAULT.color);
    }
}
