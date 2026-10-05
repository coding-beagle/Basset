//! Colour: the sRGB colours appearances are written in, the linear light the tracer
//! works in, and the tone curve between the two.
//!
//! An appearance's colour is kept as eight-bit sRGB because every way a colour reaches
//! one is eight-bit sRGB already — a swatch in the library is written as a hex code, and
//! a colour picker hands back three bytes — and because it then saves as `"#c0c4c8"`,
//! which a person reading a diff of the file can picture. Storing linear floats instead
//! would round-trip through the picker with a different value every time it was opened.

use std::fmt;

use serde::{Deserialize, Deserializer, Serialize, Serializer};

/// Linear RGB, the space light adds up in.
pub type Rgb = [f32; 3];

/// An eight-bit sRGB colour. Serialised as `#rrggbb`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Srgb(pub [u8; 3]);

impl Srgb {
    pub const WHITE: Srgb = Srgb([255, 255, 255]);
    pub const BLACK: Srgb = Srgb([0, 0, 0]);

    /// From a `0xRRGGBB` literal, which is how the library writes its swatches.
    pub const fn hex(rgb: u32) -> Srgb {
        Srgb([(rgb >> 16) as u8, (rgb >> 8) as u8, rgb as u8])
    }

    pub fn to_linear(self) -> Rgb {
        self.0.map(|c| srgb_to_linear(f32::from(c) / 255.0))
    }

    /// The nearest eight-bit colour to a linear one. Components outside `[0, 1]` clamp.
    pub fn from_linear(rgb: Rgb) -> Srgb {
        Srgb(rgb.map(|c| (linear_to_srgb(c.clamp(0.0, 1.0)) * 255.0).round() as u8))
    }

    /// `#rrggbb`, or `None` for anything else.
    pub fn parse(text: &str) -> Option<Srgb> {
        let hex = text.trim().strip_prefix('#')?;
        if hex.len() != 6 || !hex.chars().all(|c| c.is_ascii_hexdigit()) {
            return None;
        }
        u32::from_str_radix(hex, 16).ok().map(Srgb::hex)
    }
}

impl fmt::Display for Srgb {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let [r, g, b] = self.0;
        write!(f, "#{r:02x}{g:02x}{b:02x}")
    }
}

impl Serialize for Srgb {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        serializer.collect_str(self)
    }
}

impl<'de> Deserialize<'de> for Srgb {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        let text = String::deserialize(deserializer)?;
        Srgb::parse(&text)
            .ok_or_else(|| serde::de::Error::custom(format!("{text:?} is not a #rrggbb colour")))
    }
}

pub fn srgb_to_linear(c: f32) -> f32 {
    if c <= 0.04045 {
        c / 12.92
    } else {
        ((c + 0.055) / 1.055).powf(2.4)
    }
}

pub fn linear_to_srgb(c: f32) -> f32 {
    if c <= 0.003_130_8 {
        c * 12.92
    } else {
        1.055 * c.powf(1.0 / 2.4) - 0.055
    }
}

/// Relative luminance (Rec. 709 weights) of a linear colour.
pub fn luminance(c: Rgb) -> f32 {
    0.2126 * c[0] + 0.7152 * c[1] + 0.0722 * c[2]
}

/// The filmic curve both the path tracer and the viewport's environment shading end in:
/// Narkowicz's fit of the ACES reference transform. Highlights roll off instead of
/// clipping, which is most of what makes a polished metal look polished rather than
/// painted white where the soft boxes reflect in it.
pub fn aces(x: f32) -> f32 {
    let x = x.max(0.0);
    ((x * (2.51 * x + 0.03)) / (x * (2.43 * x + 0.59) + 0.14)).clamp(0.0, 1.0)
}

/// The radiance [`aces`] maps to `y`. A solid background is chosen as a colour on screen
/// but lives in the scene as a radiance, and this is how the one becomes the other so the
/// background comes out of the tone curve as exactly the colour picked. The curve
/// approaches 1.03 and is clamped at 1, so pure white is taken a hair below.
pub fn aces_inverse(y: f32) -> f32 {
    let y = f64::from(y.clamp(0.0, 0.999));
    // y (2.43 x² + 0.59 x + 0.14) = 2.51 x² + 0.03 x, a quadratic in x.
    let a = 2.43 * y - 2.51;
    let b = 0.59 * y - 0.03;
    let c = 0.14 * y;
    if a.abs() < 1e-12 {
        return (-c / b) as f32;
    }
    let disc = (b * b - 4.0 * a * c).max(0.0).sqrt();
    // `a` is negative over the whole range, so the positive root is this one.
    ((-b - disc) / (2.0 * a)).max(0.0) as f32
}

/// Linear scene radiance to a displayable eight-bit sRGB value, through the tone curve.
pub fn tone_map(c: Rgb) -> [u8; 3] {
    c.map(|v| (linear_to_srgb(aces(v)) * 255.0 + 0.5).clamp(0.0, 255.0) as u8)
}

pub(crate) fn scale(c: Rgb, s: f32) -> Rgb {
    c.map(|v| v * s)
}

pub(crate) fn add(a: Rgb, b: Rgb) -> Rgb {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

pub(crate) fn lerp(a: Rgb, b: Rgb, t: f32) -> Rgb {
    [
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
        a[2] + (b[2] - a[2]) * t,
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn eight_bit_srgb_survives_the_round_trip_through_linear() {
        for v in 0..=255u8 {
            let c = Srgb([v, v / 2, 255 - v]);
            assert_eq!(Srgb::from_linear(c.to_linear()), c);
        }
    }

    #[test]
    fn colours_save_as_hex_and_read_back() {
        let c = Srgb::hex(0xc0c4c8);
        let json = serde_json::to_string(&c).unwrap();
        assert_eq!(json, "\"#c0c4c8\"");
        assert_eq!(serde_json::from_str::<Srgb>(&json).unwrap(), c);
        assert!(serde_json::from_str::<Srgb>("\"c0c4c8\"").is_err());
        assert!(serde_json::from_str::<Srgb>("\"#c0c4cz\"").is_err());
    }

    #[test]
    fn the_tone_curve_is_monotonic_and_saturates() {
        let mut last = 0.0;
        for i in 0..200 {
            let y = aces(i as f32 * 0.1);
            assert!(y >= last);
            last = y;
        }
        assert_eq!(aces(0.0), 0.0);
        assert!(aces(100.0) > 0.99);
        assert_eq!(tone_map([0.0; 3]), [0, 0, 0]);
    }

    #[test]
    fn the_inverse_curve_undoes_the_curve() {
        for i in 0..100 {
            let y = i as f32 / 100.0;
            assert!((aces(aces_inverse(y)) - y).abs() < 1e-4, "{y}");
        }
    }
}
