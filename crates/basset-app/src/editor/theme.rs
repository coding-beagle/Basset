//! The look of the panels: typeface, palette, spacing, and the few widgets the editor
//! draws itself because egui's own would make it look like every other egui program.
//!
//! One place sets the egui style, so every panel, popup and overlay draws from the same
//! few colours rather than each picking its own grey. Both the window and the test
//! harness install it before their first frame, so tests lay out what the user sees.

use std::sync::Arc;

use egui::{
    Color32, CornerRadius, FontData, FontDefinitions, FontFamily, FontId, Margin, Shadow, Stroke,
    TextStyle,
};

/// Text that says something failed: an error under a field, a feature that did not build.
pub const ERROR: Color32 = Color32::from_rgb(240, 120, 104);
/// Text that says something built but deserves a look.
pub const WARNING: Color32 = Color32::from_rgb(236, 190, 92);
/// The one colour that means "this is the active thing": the timeline cursor, the
/// focused field, the selected tool.
pub const ACCENT: Color32 = Color32::from_rgb(77, 141, 255);
/// Accent for text and strokes drawn on [`ACCENT_TINT`]; the plain accent is too dark to
/// read there.
pub const ACCENT_TEXT: Color32 = Color32::from_rgb(150, 192, 255);
/// The fill of something selected: the accent at a quarter strength over the panel,
/// rather than egui's solid block of blue.
pub const ACCENT_TINT: Color32 = Color32::from_rgb(38, 56, 90);
/// The fill of the one primary button on a row: the accent, darkened to carry white text.
pub const ACCENT_BUTTON: Color32 = Color32::from_rgb(46, 99, 196);

pub const TEXT: Color32 = Color32::from_rgb(200, 205, 214);
pub const TEXT_STRONG: Color32 = Color32::from_rgb(240, 242, 246);
pub const TEXT_WEAK: Color32 = Color32::from_rgb(128, 135, 148);
pub const TEXT_DISABLED: Color32 = Color32::from_rgb(84, 90, 102);

/// The menu strip and status bar: the window's frame, darkest of all.
const DEEP: Color32 = Color32::from_rgb(17, 18, 22);
/// The ribbon and side panels.
const PANEL: Color32 = Color32::from_rgb(27, 29, 34);
/// Windows and popups, lifted off the panels they float over.
const RAISED: Color32 = Color32::from_rgb(36, 39, 46);
/// Text boxes and the track of a segmented control: sunk below the panel.
pub const FIELD: Color32 = Color32::from_rgb(15, 16, 19);
pub const LINE: Color32 = Color32::from_rgb(44, 48, 56);
const BUTTON: Color32 = Color32::from_rgb(40, 44, 52);
pub const HOVER: Color32 = Color32::from_rgb(46, 50, 59);
pub const PRESS: Color32 = Color32::from_rgb(56, 61, 72);
const EDGE: Color32 = Color32::from_rgb(72, 79, 92);

/// The family headings are set in: the semibold cut of the body face.
pub fn heading_family() -> FontFamily {
    FontFamily::Name("heading".into())
}

/// Installs the fonts and style on a context once; later calls are free.
pub(crate) fn install(ctx: &egui::Context) {
    let id = egui::Id::new("basset-theme");
    if ctx.data(|d| d.get_temp::<bool>(id)).unwrap_or(false) {
        return;
    }
    ctx.data_mut(|d| d.insert_temp(id, true));
    ctx.set_fonts(fonts());
    // The viewport colours are chosen for a dark surround; a light panel beside them
    // would follow the desktop theme into a window that does not match itself.
    ctx.set_theme(egui::Theme::Dark);
    ctx.style_mut_of(egui::Theme::Dark, apply);
}

/// IBM Plex for text and numbers, ahead of egui's own fonts, which stay behind it as
/// fallbacks for the symbols Plex does not carry (the transport arrows, ✔, ⚠).
fn fonts() -> FontDefinitions {
    let mut fonts = FontDefinitions::default();
    let faces: [(&str, &'static [u8]); 3] = [
        (
            "plex-sans",
            include_bytes!("../../assets/fonts/IBMPlexSans-Regular.ttf"),
        ),
        (
            "plex-sans-semibold",
            include_bytes!("../../assets/fonts/IBMPlexSans-SemiBold.ttf"),
        ),
        (
            "plex-mono",
            include_bytes!("../../assets/fonts/IBMPlexMono-Regular.ttf"),
        ),
    ];
    for (name, bytes) in faces {
        fonts
            .font_data
            .insert(name.into(), Arc::new(FontData::from_static(bytes)));
    }
    let fallbacks = |family: &FontFamily| fonts.families.get(family).cloned().unwrap_or_default();
    let proportional = fallbacks(&FontFamily::Proportional);
    let monospace = fallbacks(&FontFamily::Monospace);
    let with = |first: &str, rest: &[String]| {
        std::iter::once(first.to_owned())
            .chain(rest.iter().cloned())
            .collect::<Vec<_>>()
    };
    let families = [
        (FontFamily::Proportional, with("plex-sans", &proportional)),
        (heading_family(), with("plex-sans-semibold", &proportional)),
        (FontFamily::Monospace, with("plex-mono", &monospace)),
    ];
    fonts.families.extend(families);
    fonts
}

fn apply(style: &mut egui::Style) {
    style.text_styles = [
        (
            TextStyle::Small,
            FontId::new(10.5, FontFamily::Proportional),
        ),
        (TextStyle::Body, FontId::new(12.5, FontFamily::Proportional)),
        (
            TextStyle::Button,
            FontId::new(12.5, FontFamily::Proportional),
        ),
        (TextStyle::Heading, FontId::new(15.0, heading_family())),
        (
            TextStyle::Monospace,
            FontId::new(12.5, FontFamily::Monospace),
        ),
    ]
    .into();

    let s = &mut style.spacing;
    s.item_spacing = egui::vec2(6.0, 3.0);
    s.button_padding = egui::vec2(8.0, 1.0);
    s.window_margin = Margin::same(12);
    s.menu_margin = Margin::same(5);
    s.indent = 14.0;
    s.icon_width = 14.0;
    s.icon_width_inner = 8.0;
    s.scroll.bar_width = 6.0;
    s.scroll.floating = true;

    let radius = CornerRadius::same(5);
    let v = &mut style.visuals;
    v.dark_mode = true;
    v.panel_fill = PANEL;
    v.window_fill = RAISED;
    v.window_stroke = Stroke::new(1.0, LINE);
    v.window_corner_radius = CornerRadius::same(9);
    v.menu_corner_radius = CornerRadius::same(7);
    v.window_shadow = Shadow {
        offset: [0, 8],
        blur: 28,
        spread: 0,
        color: Color32::from_black_alpha(140),
    };
    v.popup_shadow = Shadow {
        offset: [0, 4],
        blur: 16,
        spread: 0,
        color: Color32::from_black_alpha(120),
    };
    v.extreme_bg_color = FIELD;
    v.text_edit_bg_color = Some(FIELD);
    v.faint_bg_color = Color32::from_rgb(31, 33, 39);
    v.code_bg_color = FIELD;
    v.hyperlink_color = ACCENT_TEXT;
    v.warn_fg_color = WARNING;
    v.error_fg_color = ERROR;
    v.weak_text_color = Some(TEXT_WEAK);
    v.selection.bg_fill = ACCENT_TINT;
    v.selection.stroke = Stroke::new(1.0, ACCENT_TEXT);
    v.text_cursor.stroke = Stroke::new(2.0, ACCENT_TEXT);
    v.collapsing_header_frame = false;
    v.indent_has_left_vline = false;
    v.striped = false;

    let w = &mut v.widgets;
    w.noninteractive.bg_fill = PANEL;
    w.noninteractive.weak_bg_fill = PANEL;
    w.noninteractive.bg_stroke = Stroke::new(1.0, LINE);
    w.noninteractive.fg_stroke = Stroke::new(1.0, TEXT);
    w.noninteractive.corner_radius = radius;

    w.inactive.bg_fill = BUTTON;
    w.inactive.weak_bg_fill = BUTTON;
    w.inactive.bg_stroke = Stroke::NONE;
    w.inactive.fg_stroke = Stroke::new(1.0, TEXT);
    w.inactive.corner_radius = radius;

    w.hovered.bg_fill = HOVER;
    w.hovered.weak_bg_fill = HOVER;
    w.hovered.bg_stroke = Stroke::new(1.0, EDGE);
    w.hovered.fg_stroke = Stroke::new(1.5, TEXT_STRONG);
    w.hovered.corner_radius = radius;
    w.hovered.expansion = 0.0;

    w.active.bg_fill = PRESS;
    w.active.weak_bg_fill = PRESS;
    w.active.bg_stroke = Stroke::new(1.0, ACCENT);
    w.active.fg_stroke = Stroke::new(2.0, TEXT_STRONG);
    w.active.corner_radius = radius;
    w.active.expansion = 0.0;

    w.open.bg_fill = HOVER;
    w.open.weak_bg_fill = HOVER;
    w.open.bg_stroke = Stroke::new(1.0, EDGE);
    w.open.fg_stroke = Stroke::new(1.0, TEXT_STRONG);
    w.open.corner_radius = radius;
}

// --- Frames ---------------------------------------------------------------------------

/// The menu strip along the top of the window.
pub fn menu_frame() -> egui::Frame {
    egui::Frame::new()
        .inner_margin(Margin::symmetric(8, 2))
        .fill(DEEP)
}

/// The ribbon of tools under the menu strip.
pub fn ribbon_frame() -> egui::Frame {
    egui::Frame::new()
        .inner_margin(Margin::symmetric(8, 3))
        .fill(PANEL)
}

/// The side panels.
pub fn side_frame() -> egui::Frame {
    egui::Frame::new()
        .inner_margin(Margin::symmetric(8, 2))
        .fill(PANEL)
}

/// The timeline, a band between the viewport and the status bar.
pub fn timeline_frame() -> egui::Frame {
    egui::Frame::new()
        .inner_margin(Margin::symmetric(8, 3))
        .fill(PANEL)
}

/// The status bar: the window's footer, matching the menu strip it mirrors.
pub fn status_frame() -> egui::Frame {
    egui::Frame::new()
        .inner_margin(Margin::symmetric(10, 2))
        .fill(DEEP)
}

// --- Widgets --------------------------------------------------------------------------

/// The open/closed mark of a section header: a chevron rather than egui's filled
/// triangle, turning from pointing right to pointing down as the section opens.
pub fn chevron(ui: &mut egui::Ui, openness: f32, response: &egui::Response) {
    let rect = response.rect;
    let c = rect.center();
    let r = rect.width().min(rect.height()) * 0.28;
    let color = if response.hovered() {
        TEXT_STRONG
    } else {
        TEXT_WEAK
    };
    let rot = egui::emath::Rot2::from_angle(openness * std::f32::consts::FRAC_PI_2);
    let pts = [
        egui::vec2(-0.5 * r, -r),
        egui::vec2(0.5 * r, 0.0),
        egui::vec2(-0.5 * r, r),
    ]
    .map(|p| c + rot * p);
    ui.painter()
        .add(egui::Shape::line(pts.to_vec(), Stroke::new(1.5, color)));
}

/// A section of a side panel: a header with a chevron, open or shut. Every section in
/// the panels goes through this, so they all fold the same way.
pub fn section(text: impl Into<egui::WidgetText>) -> egui::CollapsingHeader {
    egui::CollapsingHeader::new(text).icon(chevron)
}

/// A quiet caption over a group of rows ("BODIES"), set small and spaced.
pub fn caption(ui: &mut egui::Ui, text: &str) {
    ui.add_space(2.0);
    ui.label(
        egui::RichText::new(text.to_uppercase())
            .size(10.0)
            .extra_letter_spacing(0.8)
            .color(TEXT_WEAK),
    );
}

/// An eye that says whether a body or sketch is shown, and toggles it on a click.
pub fn eye(ui: &mut egui::Ui, visible: bool) -> egui::Response {
    let (rect, response) = ui.allocate_exact_size(egui::vec2(18.0, 18.0), egui::Sense::click());
    let color = match (visible, response.hovered()) {
        (_, true) => TEXT_STRONG,
        (true, false) => TEXT,
        (false, false) => Color32::from_rgb(80, 86, 98),
    };
    let stroke = Stroke::new(1.3, color);
    let c = rect.center();
    let (w, h) = (6.5, 4.0);
    // Two arcs make the lids; the pupil is a dot when the thing is shown, and a slash
    // through the eye says it is not.
    let lid = |sign: f32| {
        (0..=10)
            .map(|i| {
                let t = i as f32 / 10.0 * 2.0 - 1.0;
                c + egui::vec2(t * w, sign * h * (1.0 - t * t))
            })
            .collect::<Vec<_>>()
    };
    let painter = ui.painter();
    painter.add(egui::Shape::line(lid(-1.0), stroke));
    painter.add(egui::Shape::line(lid(1.0), stroke));
    if visible {
        painter.circle_filled(c, 1.9, color);
    } else {
        painter.line_segment([c + egui::vec2(-w, h), c + egui::vec2(w, -h)], stroke);
    }
    response.on_hover_text(if visible { "Hide" } else { "Show" })
}

/// Height of a segmented control.
pub const SEGMENT_HEIGHT: f32 = 20.0;

fn segment_galley(ui: &egui::Ui, label: &str) -> Arc<egui::Galley> {
    let font = TextStyle::Button.resolve(ui.style());
    ui.painter().layout_no_wrap(label.to_owned(), font, TEXT)
}

/// The width a segmented control of these labels will take, so a wrapping row can make
/// room for it as one piece rather than splitting it across lines.
pub fn segmented_width(ui: &egui::Ui, labels: &[&str]) -> f32 {
    labels
        .iter()
        .map(|l| segment_galley(ui, l).size().x + 18.0)
        .sum::<f32>()
        + 4.0
}

/// A row of mutually exclusive choices on one sunk track, the chosen one raised: the
/// workspace switch and the selection filters. Returns each segment's response, in the
/// order given, for hover text and clicks.
pub fn segmented(ui: &mut egui::Ui, labels: &[&str], chosen: usize) -> Vec<egui::Response> {
    let width = segmented_width(ui, labels);
    let (track, _) =
        ui.allocate_exact_size(egui::vec2(width, SEGMENT_HEIGHT), egui::Sense::hover());
    ui.painter().rect_filled(track, 6.0, FIELD);
    let mut x = track.left() + 2.0;
    let mut out = Vec::with_capacity(labels.len());
    for (i, label) in labels.iter().enumerate() {
        let galley = segment_galley(ui, label);
        let w = galley.size().x + 18.0;
        let rect = egui::Rect::from_min_size(
            egui::pos2(x, track.top() + 2.0),
            egui::vec2(w, SEGMENT_HEIGHT - 4.0),
        );
        x += w;
        let response = ui.interact(
            rect,
            ui.id().with(("segment", *label)),
            egui::Sense::click(),
        );
        let on = i == chosen;
        if on {
            ui.painter().rect_filled(rect, 4.0, PRESS);
        } else if response.hovered() {
            ui.painter().rect_filled(rect, 4.0, PANEL);
        }
        let color = if on || response.hovered() {
            TEXT_STRONG
        } else {
            TEXT_WEAK
        };
        ui.painter()
            .galley(rect.center() - galley.size() * 0.5, galley, color);
        out.push(response);
    }
    out
}
