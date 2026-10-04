//! A library of common engineering materials.
//!
//! # Why a library
//!
//! A [`Material`] is the two numbers the linear elastic solve needs, and nothing else. But
//! a study is rarely run for the displacement alone: the question behind it is "will it
//! hold", which needs a yield strength to compare the von Mises stress against, and "what
//! does it weigh", which needs a density. Neither belongs on `Material`, because the solver
//! never reads them, so they live here on a [`MaterialSpec`] that wraps the solver's
//! numbers with the rest of a data sheet: a name, a family, a density and a yield point.
//!
//! The values are textbook room-temperature figures for the common condition of each
//! alloy (the temper or heat treatment is in the name where it changes the numbers, and in
//! a comment where it does not). They are what a designer reaches for before a supplier's
//! certificate arrives; they are not that certificate. Polymers in particular vary by
//! grade, filler and moisture, and their "yield" is the tensile strength at yield of a dry
//! specimen.
//!
//! # Naming
//!
//! Names are short and standard — `6061-T6`, `S355`, `Ti-6Al-4V` — because an agent or a
//! user types them. [`find`] forgives case, spaces, hyphens and the American spelling of
//! aluminium, and falls back to a substring match when that singles out one entry, so
//! `al 6061-t6`, `304` and `pmma` all resolve. The generic `Steel` and `Aluminium` entries
//! carry exactly [`Material::STEEL`] and [`Material::ALUMINIUM`], so documents and tool
//! calls written before the library existed resolve to the same numbers they always did.

use crate::{Material, Results};

/// The family a material belongs to; what a picker groups by and a filter selects on.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum MaterialGroup {
    Steel,
    StainlessSteel,
    Aluminium,
    Titanium,
    CopperAlloy,
    CastIron,
    Magnesium,
    Plastic,
    Other,
}

impl MaterialGroup {
    /// Every group, in the order a picker should list them: the metals a machine shop
    /// sees most first, plastics last.
    pub const ALL: [MaterialGroup; 9] = [
        MaterialGroup::Steel,
        MaterialGroup::StainlessSteel,
        MaterialGroup::Aluminium,
        MaterialGroup::Titanium,
        MaterialGroup::CopperAlloy,
        MaterialGroup::CastIron,
        MaterialGroup::Magnesium,
        MaterialGroup::Plastic,
        MaterialGroup::Other,
    ];

    pub fn name(self) -> &'static str {
        match self {
            MaterialGroup::Steel => "Steel",
            MaterialGroup::StainlessSteel => "Stainless steel",
            MaterialGroup::Aluminium => "Aluminium",
            MaterialGroup::Titanium => "Titanium",
            MaterialGroup::CopperAlloy => "Copper alloy",
            MaterialGroup::CastIron => "Cast iron",
            MaterialGroup::Magnesium => "Magnesium",
            MaterialGroup::Plastic => "Plastic",
            MaterialGroup::Other => "Other",
        }
    }

    /// The group whose name matches, with the same tolerance as [`find`]: `stainless`,
    /// `Stainless steel`, `copper-alloy` and `plastics` all resolve.
    pub fn from_name(name: &str) -> Option<MaterialGroup> {
        let wanted = normalise(name);
        if wanted.is_empty() {
            return None;
        }
        Self::ALL
            .into_iter()
            .find(|g| normalise(g.name()) == wanted)
            .or_else(|| {
                let mut hits = Self::ALL
                    .into_iter()
                    .filter(|g| normalise(g.name()).contains(&wanted));
                match (hits.next(), hits.next()) {
                    (Some(g), None) => Some(g),
                    _ => None,
                }
            })
    }
}

/// One entry of the library: the solver's numbers with the rest of the data sheet.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MaterialSpec {
    pub name: &'static str,
    pub group: MaterialGroup,
    /// What the solver reads.
    pub material: Material,
    /// Density in g/cm³, the unit data sheets quote; [`MaterialSpec::mass_kg`] does the
    /// conversion from the kernel's cubic millimetres.
    pub density: f64,
    /// Yield strength in MPa. `None` where a material has no yield point to speak of:
    /// the generic entries, which name no grade, and grey cast iron, which breaks before
    /// it yields.
    pub yield_strength: Option<f64>,
}

impl MaterialSpec {
    /// Mass in kilograms of a body of this material with the given volume in mm³.
    pub fn mass_kg(&self, volume_mm3: f64) -> f64 {
        mass_kg(self.density, volume_mm3)
    }

    /// The entry whose solver numbers are exactly these, if any; what a UI uses to show
    /// the name behind a pair of numbers, or "Custom" when there is none. Where two
    /// entries share their numbers the first in [`library`] wins, which is why the generic
    /// `Steel` and `Aluminium` come first: the constants they stand for name themselves.
    pub fn of(material: Material) -> Option<&'static MaterialSpec> {
        library().iter().find(|m| m.material == material)
    }
}

/// Mass in kilograms from a density in g/cm³ and a volume in mm³. A cubic millimetre is
/// a thousandth of a cubic centimetre, and a gram a thousandth of a kilogram.
pub fn mass_kg(density_g_cm3: f64, volume_mm3: f64) -> f64 {
    density_g_cm3 * volume_mm3 * 1e-6
}

/// How many times the yield strength the peak von Mises stress is below: the number a
/// designer reads first. Infinite when nothing is stressed, so an unloaded part is not
/// reported as failing by a division by zero.
pub fn safety_factor(yield_strength: f64, max_von_mises: f64) -> f64 {
    if max_von_mises <= 0.0 {
        f64::INFINITY
    } else {
        yield_strength / max_von_mises
    }
}

impl Results {
    /// Mass in kilograms of the meshed body at the given density in g/cm³. The volume is
    /// the mesh's, not the solid's, so it carries the voxel approximation; a body meshed
    /// finer weighs closer to the truth.
    pub fn mass_kg(&self, density_g_cm3: f64) -> f64 {
        mass_kg(density_g_cm3, self.mesh.volume())
    }
}

const fn spec(
    name: &'static str,
    group: MaterialGroup,
    youngs_modulus: f64,
    poisson_ratio: f64,
    density: f64,
    yield_strength: Option<f64>,
) -> MaterialSpec {
    MaterialSpec {
        name,
        group,
        material: Material {
            youngs_modulus,
            poisson_ratio,
        },
        density,
        yield_strength,
    }
}

use MaterialGroup::*;

/// The library. Young's modulus in MPa, Poisson's ratio, density in g/cm³, yield in MPa.
/// The two generic entries come first and carry the solver's constants exactly; see
/// [`MaterialSpec::of`] for why the order matters.
static LIBRARY: [MaterialSpec; 29] = [
    spec(
        "Steel",
        Steel,
        Material::STEEL.youngs_modulus,
        Material::STEEL.poisson_ratio,
        7.85,
        None,
    ),
    spec(
        "Aluminium",
        Aluminium,
        Material::ALUMINIUM.youngs_modulus,
        Material::ALUMINIUM.poisson_ratio,
        2.70,
        None,
    ),
    // Structural steels (EN 10025), named by their yield strength.
    spec("S235", Steel, 210_000.0, 0.30, 7.85, Some(235.0)),
    spec("S355", Steel, 210_000.0, 0.30, 7.85, Some(355.0)),
    // Medium-carbon steel, cold drawn.
    spec("AISI 1045", Steel, 205_000.0, 0.29, 7.85, Some(450.0)),
    // Chromium-molybdenum steel, quenched and tempered to about 1000 MPa tensile.
    spec("AISI 4140", Steel, 205_000.0, 0.29, 7.85, Some(655.0)),
    // Austenitic stainless steels, annealed.
    spec(
        "Stainless 304",
        StainlessSteel,
        193_000.0,
        0.29,
        8.00,
        Some(215.0),
    ),
    spec(
        "Stainless 316",
        StainlessSteel,
        193_000.0,
        0.28,
        8.00,
        Some(205.0),
    ),
    // Precipitation-hardening stainless, condition H900.
    spec(
        "17-4PH",
        StainlessSteel,
        197_000.0,
        0.27,
        7.80,
        Some(1170.0),
    ),
    // Wrought aluminium alloys by temper.
    spec("6061-T6", Aluminium, 68_900.0, 0.33, 2.70, Some(276.0)),
    spec("7075-T6", Aluminium, 71_700.0, 0.33, 2.81, Some(503.0)),
    spec("5052-H32", Aluminium, 70_300.0, 0.33, 2.68, Some(193.0)),
    spec("2024-T3", Aluminium, 73_100.0, 0.33, 2.78, Some(345.0)),
    // Casting alloy, T6.
    spec("A356", Aluminium, 72_400.0, 0.33, 2.67, Some(165.0)),
    // Grade 5 titanium, annealed.
    spec("Ti-6Al-4V", Titanium, 113_800.0, 0.342, 4.43, Some(880.0)),
    // Copper alloys, half hard (H02): the annealed figures are a third of these.
    spec(
        "Copper C110",
        CopperAlloy,
        117_000.0,
        0.34,
        8.94,
        Some(250.0),
    ),
    spec(
        "Brass C260",
        CopperAlloy,
        110_000.0,
        0.375,
        8.53,
        Some(360.0),
    ),
    // Bearing bronze, as cast.
    spec(
        "Bronze C932",
        CopperAlloy,
        100_000.0,
        0.34,
        8.93,
        Some(125.0),
    ),
    // Class 30 grey iron: no yield point, tensile about 210 MPa.
    spec("Grey cast iron", CastIron, 100_000.0, 0.26, 7.20, None),
    // Ductile iron 65-45-12.
    spec(
        "Ductile iron",
        CastIron,
        169_000.0,
        0.275,
        7.10,
        Some(310.0),
    ),
    // Wrought magnesium sheet, H24.
    spec("AZ31B", Magnesium, 45_000.0, 0.35, 1.77, Some(200.0)),
    // Thermoplastics, dry as moulded; grades vary widely.
    spec("ABS", Plastic, 2_300.0, 0.35, 1.04, Some(40.0)),
    spec("PLA", Plastic, 3_500.0, 0.36, 1.24, Some(60.0)),
    spec("Nylon 6", Plastic, 2_800.0, 0.39, 1.14, Some(70.0)),
    spec("Polycarbonate", Plastic, 2_400.0, 0.37, 1.20, Some(62.0)),
    spec("PEEK", Plastic, 3_600.0, 0.38, 1.32, Some(100.0)),
    spec("Acrylic (PMMA)", Plastic, 3_100.0, 0.37, 1.18, Some(70.0)),
    spec("Delrin (POM)", Plastic, 3_000.0, 0.35, 1.41, Some(70.0)),
    // Concrete has no entry: it is not linear elastic in tension. Wood is orthotropic.
    // Glass, for a window or a lens mount: soda-lime, no yield.
    spec("Glass", Other, 70_000.0, 0.22, 2.50, None),
];

/// Every material, generic entries first, then by group.
pub fn library() -> &'static [MaterialSpec] {
    &LIBRARY
}

/// A name reduced to what matters for matching: ASCII lowercase, letters and digits only,
/// and `aluminum` spelt as the library spells it.
fn normalise(name: &str) -> String {
    let folded: String = name
        .chars()
        .filter(|c| c.is_ascii_alphanumeric())
        .map(|c| c.to_ascii_lowercase())
        .collect();
    folded.replace("aluminum", "aluminium")
}

/// The entry a name means. Exact after [`normalise`] first; failing that, the one entry
/// whose name contains the query or is contained by it (`304` finds `Stainless 304`, and
/// `al 6061-t6` finds `6061-T6`). Two candidates means the name was ambiguous and nothing
/// is returned, so `steel` alone finds only the generic entry and `stainless` finds none.
pub fn find(name: &str) -> Option<&'static MaterialSpec> {
    let wanted = normalise(name);
    if wanted.is_empty() {
        return None;
    }
    library()
        .iter()
        .find(|m| normalise(m.name) == wanted)
        .or_else(|| {
            let mut hits = library().iter().filter(|m| {
                let have = normalise(m.name);
                have.contains(&wanted) || wanted.contains(&have)
            });
            match (hits.next(), hits.next()) {
                (Some(m), None) => Some(m),
                _ => None,
            }
        })
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn names_are_unique_and_values_sane() {
        let lib = library();
        assert!(lib.len() >= 20);
        for (i, m) in lib.iter().enumerate() {
            assert!(
                lib[..i]
                    .iter()
                    .all(|o| normalise(o.name) != normalise(m.name)),
                "duplicate name {}",
                m.name
            );
            assert!(m.material.youngs_modulus > 0.0, "{}", m.name);
            assert!((0.0..0.5).contains(&m.material.poisson_ratio), "{}", m.name);
            assert!(m.density > 0.0, "{}", m.name);
            if let Some(y) = m.yield_strength {
                assert!(y > 0.0, "{}", m.name);
            }
        }
    }

    #[test]
    fn find_round_trips_every_name_and_forgives_case() {
        for m in library() {
            assert_eq!(find(m.name).map(|f| f.name), Some(m.name));
            assert_eq!(
                find(&m.name.to_uppercase()).map(|f| f.name),
                Some(m.name),
                "{}",
                m.name
            );
            assert_eq!(
                find(&m.name.to_lowercase().replace('-', " ")).map(|f| f.name),
                Some(m.name)
            );
        }
        assert_eq!(find("al 6061-t6").unwrap().name, "6061-T6");
        assert_eq!(find("304").unwrap().name, "Stainless 304");
        assert_eq!(find("pmma").unwrap().name, "Acrylic (PMMA)");
        assert_eq!(find("aluminum").unwrap().name, "Aluminium");
        assert!(find("stainless").is_none(), "ambiguous");
        assert!(find("").is_none());
        assert!(find("unobtainium").is_none());
    }

    #[test]
    fn generic_entries_are_the_solver_constants() {
        assert_eq!(find("steel").unwrap().material, Material::STEEL);
        assert_eq!(find("aluminium").unwrap().material, Material::ALUMINIUM);
        assert_eq!(MaterialSpec::of(Material::STEEL).unwrap().name, "Steel");
        assert_eq!(
            MaterialSpec::of(Material::ALUMINIUM).unwrap().name,
            "Aluminium"
        );
        assert!(
            MaterialSpec::of(Material {
                youngs_modulus: 1.0,
                poisson_ratio: 0.1,
            })
            .is_none()
        );
    }

    #[test]
    fn groups_resolve_by_name() {
        for g in MaterialGroup::ALL {
            assert_eq!(MaterialGroup::from_name(g.name()), Some(g));
        }
        assert_eq!(
            MaterialGroup::from_name("stainless"),
            Some(MaterialGroup::StainlessSteel)
        );
        assert_eq!(
            MaterialGroup::from_name("copper-alloy"),
            Some(MaterialGroup::CopperAlloy)
        );
        assert_eq!(
            MaterialGroup::from_name("steel"),
            Some(MaterialGroup::Steel)
        );
        assert_eq!(MaterialGroup::from_name("wood"), None);
    }

    #[test]
    fn mass_and_safety_factor() {
        // A 10 mm steel cube weighs 7.85 g.
        assert_relative_eq!(find("steel").unwrap().mass_kg(1000.0), 0.00785);
        assert_relative_eq!(safety_factor(276.0, 138.0), 2.0);
        assert!(safety_factor(276.0, 0.0).is_infinite());
    }
}
