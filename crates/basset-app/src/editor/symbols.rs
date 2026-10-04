//! The tool symbols, and the painter that draws them.
//!
//! The set is designed as SVG in `assets/symbols/symbols.json`, where it can be previewed
//! and compared, and `assets/symbols/gen.py` turns it into the polylines in
//! [`symbol_data`](super::symbol_data). Painting them here, rather than rasterising SVG,
//! keeps them sharp at every size and lets each button tint them for its own state.

use super::theme;

/// Every symbol in the set.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Symbol {
    CreateSketch,
    Extrude,
    Revolve,
    Sweep,
    Loft,
    Fillet,
    Chamfer,
    Thread,
    Combine,
    Move,
    OffsetPlane,
    AngledPlane,
    Component,
    Simulate,
    Measure,
    Fit,
    Select,
    Line,
    Rectangle,
    CenterRectangle,
    Circle,
    Circle2Point,
    Circle3Point,
    Arc3Point,
    ArcCenter,
    Polygon,
    Slot,
    SlotOverall,
    SlotCenterPoint,
    Text,
    Dimension,
    Trim,
    Break,
    SketchFillet,
    Construction,
    Delete,
    SketchMove,
    Pattern,
    Offset,
    FinishSketch,
    Coincident,
    Horizontal,
    Vertical,
    Parallel,
    Perpendicular,
    Tangent,
    Equal,
    Concentric,
    Midpoint,
    Symmetric,
    Fix,
}

/// Which of the symbol's colours a part is drawn in.
#[derive(Clone, Copy)]
pub(super) enum Ink {
    /// The button's text colour, which carries its hover, selected and disabled states.
    Main,
    /// What the tool does: the arrow of an extrude, the face a fillet makes.
    Accent,
    /// The faces of a solid: the main colour, mostly transparent.
    Faint,
}

pub(super) struct Paint {
    pub ink: Ink,
    pub alpha: f32,
}

pub(super) struct Pen {
    pub ink: Ink,
    pub alpha: f32,
    pub width: f32,
    pub dash: Option<[f32; 2]>,
}

/// One outline in the 24-unit frame the set is drawn in. `tris` triangulates a concave
/// fill; a convex one is filled as it stands, and so has none.
pub(super) struct Path {
    pub pts: &'static [[f32; 2]],
    pub closed: bool,
    pub tris: &'static [[u16; 3]],
}

/// One element of a symbol: outlines with an optional fill and an optional stroke.
pub(super) struct Prim {
    pub fill: Option<Paint>,
    pub stroke: Option<Pen>,
    pub paths: &'static [Path],
}

/// The side of the square the set is designed in.
const FRAME: f32 = 24.0;

/// Paint `symbol` to fill the square `rect`, in `color`.
///
/// The accent follows the button: a disabled button is all one grey, and anything else
/// keeps the blue that marks what the tool does.
pub(crate) fn paint(
    painter: &egui::Painter,
    rect: egui::Rect,
    symbol: Symbol,
    color: egui::Color32,
) {
    let accent = if color == theme::TEXT_DISABLED {
        color
    } else {
        theme::ACCENT_TEXT
    };
    paint_inked(painter, rect, symbol, color, accent);
}

/// [`paint`], with the accent given rather than chosen: for a symbol on a filled button,
/// where the usual blue would vanish into the fill.
pub(crate) fn paint_inked(
    painter: &egui::Painter,
    rect: egui::Rect,
    symbol: Symbol,
    color: egui::Color32,
    accent: egui::Color32,
) {
    let ink = |ink: Ink, alpha: f32| {
        let c = match ink {
            Ink::Main => color,
            Ink::Accent => accent,
            Ink::Faint => color.gamma_multiply(0.18),
        };
        if alpha < 1.0 {
            c.gamma_multiply(alpha)
        } else {
            c
        }
    };
    let scale = rect.width().min(rect.height()) / FRAME;
    let origin = rect.center() - egui::vec2(FRAME, FRAME) * scale * 0.5;
    let at = |p: &[f32; 2]| origin + egui::vec2(p[0], p[1]) * scale;
    for prim in super::symbol_data::prims(symbol) {
        for path in prim.paths {
            let pts: Vec<egui::Pos2> = path.pts.iter().map(at).collect();
            if let Some(fill) = &prim.fill {
                let fill = ink(fill.ink, fill.alpha);
                if path.tris.is_empty() {
                    painter.add(egui::Shape::convex_polygon(
                        pts.clone(),
                        fill,
                        egui::Stroke::NONE,
                    ));
                } else {
                    let mut mesh = egui::Mesh::default();
                    for p in &pts {
                        mesh.colored_vertex(*p, fill);
                    }
                    for [a, b, c] in path.tris {
                        mesh.add_triangle(u32::from(*a), u32::from(*b), u32::from(*c));
                    }
                    painter.add(mesh);
                }
            }
            if let Some(pen) = &prim.stroke {
                // Thin strokes stop reading as lines below a pixel, so the small sizes
                // (the constraint buttons, the timeline) keep a pixel at least.
                let stroke =
                    egui::Stroke::new((pen.width * scale).max(1.0), ink(pen.ink, pen.alpha));
                match pen.dash {
                    Some([on, off]) => {
                        let mut line = pts.clone();
                        if path.closed {
                            line.push(pts[0]);
                        }
                        painter.extend(egui::Shape::dashed_line(
                            &line,
                            stroke,
                            on * scale,
                            off * scale,
                        ));
                    }
                    None if path.closed => {
                        painter.add(egui::Shape::closed_line(pts, stroke));
                    }
                    None => {
                        painter.add(egui::Shape::line(pts, stroke));
                    }
                }
            }
        }
    }
}
