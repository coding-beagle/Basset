//! Text support: a font wrapper around `ttf-parser` and glyph outline flattening.
//!
//! [`Font`] owns the raw file bytes and re-parses the face on every use. Parsing only
//! locates tables (no allocation, microseconds), and this avoids a self-referential
//! struct without an extra dependency.

use std::path::{Path, PathBuf};

use basset_math::Vec2;
use ttf_parser::{Face, GlyphId, OutlineBuilder};

use crate::SketchError;
use crate::tessellation::Tessellation;

pub struct Font {
    data: Vec<u8>,
    units_per_em: f64,
    ascender: f64,
    descender: f64,
    family: String,
}

impl std::fmt::Debug for Font {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Font")
            .field("family", &self.family)
            .field("bytes", &self.data.len())
            .finish()
    }
}

impl Font {
    pub fn from_bytes(data: Vec<u8>) -> Result<Font, SketchError> {
        let face = Face::parse(&data, 0).map_err(|e| SketchError::InvalidFont(e.to_string()))?;
        let units_per_em = f64::from(face.units_per_em());
        if units_per_em <= 0.0 {
            return Err(SketchError::InvalidFont("units_per_em is zero".into()));
        }
        let ascender = f64::from(face.ascender());
        let descender = f64::from(face.descender());
        let family = face
            .names()
            .into_iter()
            .find(|n| n.name_id == ttf_parser::name_id::FAMILY && n.is_unicode())
            .and_then(|n| n.to_string())
            .unwrap_or_default();
        Ok(Font {
            data,
            units_per_em,
            ascender,
            descender,
            family,
        })
    }

    pub fn from_file(path: impl AsRef<Path>) -> Result<Font, SketchError> {
        Font::from_bytes(std::fs::read(path)?)
    }

    /// Looks for a usable TrueType font in the common Linux font directories,
    /// preferring the ubiquitous DejaVu Sans and Liberation Sans so results are
    /// predictable across machines.
    pub fn find_system_font() -> Option<Font> {
        let home = std::env::var_os("HOME").map(PathBuf::from);
        let mut dirs = vec![
            PathBuf::from("/usr/share/fonts"),
            PathBuf::from("/usr/local/share/fonts"),
        ];
        if let Some(h) = home {
            dirs.push(h.join(".fonts"));
            dirs.push(h.join(".local/share/fonts"));
        }
        let mut candidates = Vec::new();
        for d in dirs {
            collect_ttf(&d, 0, &mut candidates);
        }
        let preferred = [
            "DejaVuSans.ttf",
            "LiberationSans-Regular.ttf",
            "Ubuntu-R.ttf",
            "NotoSans-Regular.ttf",
        ];
        let pick = preferred
            .iter()
            .find_map(|name| {
                candidates
                    .iter()
                    .find(|p| p.file_name().is_some_and(|f| f == *name))
            })
            .or_else(|| candidates.first());
        pick.and_then(|p| Font::from_file(p).ok())
    }

    pub fn family(&self) -> &str {
        &self.family
    }

    fn face(&self) -> Option<Face<'_>> {
        // Already validated in `from_bytes`; a failure here is impossible in practice.
        Face::parse(&self.data, 0).ok()
    }

    /// Scale factor from font units to mm for a given em height.
    fn scale(&self, height: f64) -> f64 {
        height / self.units_per_em
    }

    /// `(ascender, descender)` in mm; descender is negative.
    pub fn vertical_extent(&self, height: f64) -> (f64, f64) {
        (
            self.ascender * self.scale(height),
            self.descender * self.scale(height),
        )
    }

    /// Total horizontal advance of the string in mm (no kerning).
    pub fn text_advance(&self, text: &str, height: f64) -> f64 {
        let Some(face) = self.face() else { return 0.0 };
        let s = self.scale(height);
        text.chars()
            .map(|c| f64::from(glyph_advance(&face, c)))
            .sum::<f64>()
            * s
    }

    /// Closed glyph outlines for the string, flattened and placed in sketch space.
    /// Orientation is whatever the font uses; callers normalise it.
    pub fn text_outlines(
        &self,
        text: &str,
        height: f64,
        angle: f64,
        origin: Vec2,
        tess: &Tessellation,
    ) -> Vec<Vec<Vec2>> {
        let Some(face) = self.face() else {
            return Vec::new();
        };
        let s = self.scale(height);
        let rot = Vec2::from_angle(angle);
        let mut pen = 0.0;
        let mut out = Vec::new();
        for c in text.chars() {
            if let Some(gid) = face.glyph_index(c) {
                let mut builder = Flattener::new(s, tess.chord_tolerance);
                face.outline_glyph(gid, &mut builder);
                builder.finish();
                for contour in builder.contours {
                    out.push(
                        contour
                            .into_iter()
                            .map(|p| origin + rot.rotate(p + Vec2::new(pen, 0.0)))
                            .collect(),
                    );
                }
            }
            pen += f64::from(glyph_advance(&face, c)) * s;
        }
        out
    }
}

fn glyph_advance(face: &Face<'_>, c: char) -> u16 {
    let gid = face.glyph_index(c).unwrap_or(GlyphId(0));
    face.glyph_hor_advance(gid).unwrap_or(0)
}

fn collect_ttf(dir: &Path, depth: usize, out: &mut Vec<PathBuf>) {
    if depth > 4 {
        return;
    }
    let Ok(entries) = std::fs::read_dir(dir) else {
        return;
    };
    for entry in entries.flatten() {
        let path = entry.path();
        if path.is_dir() {
            collect_ttf(&path, depth + 1, out);
        } else if path
            .extension()
            .is_some_and(|e| e.eq_ignore_ascii_case("ttf"))
        {
            out.push(path);
        }
    }
}

/// Collects glyph contours as polylines in mm, subdividing Béziers by curvature so the
/// polyline stays within the chord tolerance.
struct Flattener {
    scale: f64,
    tolerance: f64,
    contours: Vec<Vec<Vec2>>,
    current: Vec<Vec2>,
}

impl Flattener {
    fn new(scale: f64, tolerance: f64) -> Self {
        Self {
            scale,
            tolerance,
            contours: Vec::new(),
            current: Vec::new(),
        }
    }

    fn p(&self, x: f32, y: f32) -> Vec2 {
        Vec2::new(f64::from(x), f64::from(y)) * self.scale
    }

    fn last(&self) -> Vec2 {
        self.current.last().copied().unwrap_or(Vec2::ZERO)
    }

    /// Segment count from the bound `|B''|max / (8 n²) ≤ tol` on chord deviation.
    fn segments_for(&self, second_derivative_max: f64) -> usize {
        ((second_derivative_max / (8.0 * self.tolerance))
            .sqrt()
            .ceil() as usize)
            .clamp(1, 64)
    }

    fn finish(&mut self) {
        if self.current.len() >= 3 {
            // Fonts repeat the start point at the end of a contour; drop it.
            if let (Some(&first), Some(&last)) = (self.current.first(), self.current.last())
                && first.distance(last) < 1e-9
            {
                self.current.pop();
            }
            let c = std::mem::take(&mut self.current);
            self.contours.push(c);
        } else {
            self.current.clear();
        }
    }
}

impl OutlineBuilder for Flattener {
    fn move_to(&mut self, x: f32, y: f32) {
        self.finish();
        let p = self.p(x, y);
        self.current.push(p);
    }

    fn line_to(&mut self, x: f32, y: f32) {
        let p = self.p(x, y);
        self.current.push(p);
    }

    fn quad_to(&mut self, x1: f32, y1: f32, x: f32, y: f32) {
        let p0 = self.last();
        let p1 = self.p(x1, y1);
        let p2 = self.p(x, y);
        let n = self.segments_for(2.0 * (p0 - p1 * 2.0 + p2).length());
        for i in 1..=n {
            let t = i as f64 / n as f64;
            let u = 1.0 - t;
            self.current
                .push(p0 * (u * u) + p1 * (2.0 * u * t) + p2 * (t * t));
        }
    }

    fn curve_to(&mut self, x1: f32, y1: f32, x2: f32, y2: f32, x: f32, y: f32) {
        let p0 = self.last();
        let p1 = self.p(x1, y1);
        let p2 = self.p(x2, y2);
        let p3 = self.p(x, y);
        let m = (p0 - p1 * 2.0 + p2)
            .length()
            .max((p1 - p2 * 2.0 + p3).length());
        let n = self.segments_for(6.0 * m);
        for i in 1..=n {
            let t = i as f64 / n as f64;
            let u = 1.0 - t;
            self.current.push(
                p0 * (u * u * u)
                    + p1 * (3.0 * u * u * t)
                    + p2 * (3.0 * u * t * t)
                    + p3 * (t * t * t),
            );
        }
    }

    fn close(&mut self) {
        self.finish();
    }
}
