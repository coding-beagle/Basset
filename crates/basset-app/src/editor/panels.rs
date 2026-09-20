//! Menus, toolbar, browser, timeline, status bar and popups.
//!
//! Panels only read editor state and queue commands; the editor mutates itself after
//! the panel closures return. That keeps borrow scopes short and lets any panel trigger
//! any command without threading `&mut Editor` through egui closures.

use basset_core::{BodyRef, ComponentId, FeatureId, FeatureKind, FeatureStatus, PlaneRef};
use basset_viewport::ViewPreset;

use super::sketch_mode::{self, SketchTool, ToolGroup, edit_text, parse_value};
use super::tools::{self, ToolKind};
use super::{Editor, Mode, SelectMode};

/// The blue the viewport draws under-constrained geometry in, so the words that explain it
/// match what the user is looking at.
const LOOSE_LABEL: egui::Color32 = egui::Color32::from_rgb(140, 184, 255);
/// Amber for a feature that built but warns about its result, matching the timeline chip.
const WARNING_LABEL: egui::Color32 = egui::Color32::from_rgb(235, 190, 90);

/// Deferred commands, so panel code never needs `&mut Editor` while it borrows state.
enum Command {
    Tool(ToolKind),
    New,
    Open,
    Save(bool),
    ExportStl,
    Export3mf,
    Quit,
    Undo,
    Redo,
    Fit,
    View(ViewPreset),
    ToggleProjection,
    SetCursor(usize),
    Edit(FeatureId),
    Suppress(FeatureId, bool),
    Delete(FeatureId),
    Rename(FeatureId),
    SelectFeature(FeatureId),
    ToggleBody(BodyRef),
    ToggleSketch(FeatureId),
    Activate(ComponentId),
    FinishSketch(bool),
    SketchTool(SketchTool),
    SketchConstraint(basset_sketch::Constraint),
    SketchSelect(Vec<basset_sketch::EntityId>),
    SketchDimension(basset_sketch::ConstraintId, f64),
    SketchRemoveConstraint(basset_sketch::ConstraintId),
    SketchDelete,
    SketchConstruction,
    /// A number in the move palette changed: re-apply the move from where it started.
    SketchMoveUpdate,
    /// Keep (`true`) or undo (`false`) the move in progress.
    SketchMoveFinish(bool),
    SketchPattern,
    SketchSetParameter(String, String),
    SketchRemoveParameter(String),
    SketchBindDimension(basset_sketch::ConstraintId, String),
    /// Enter in an entry box: place the shape from the typed sizes.
    SketchSubmitEntry,
    /// The sketch editor changed its sketch outside a pointer event (a dragged label):
    /// write it into the feature.
    SketchCommit,
    SelectMode(SelectMode),
}

pub fn show(editor: &mut Editor, ui: &mut egui::Ui) {
    let mut commands: Vec<Command> = Vec::new();
    editor.refresh_cache();

    egui::Panel::top("menu").show(ui, |ui| {
        menu_bar(editor, ui, &mut commands);
        toolbar(editor, ui, &mut commands);
    });
    egui::Panel::bottom("status").show(ui, |ui| {
        ui.horizontal(|ui| {
            ui.label(&editor.status);
            warning_summary(editor, ui, &mut commands);
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                ui.label(egui::RichText::new(editor.selection.summary()).weak());
            });
        });
    });
    egui::Panel::bottom("timeline")
        .default_size(64.0)
        .show(ui, |ui| timeline(editor, ui, &mut commands));
    egui::Panel::left("browser")
        .default_size(230.0)
        .show(ui, |ui| browser(editor, ui, &mut commands));
    if editor.is_sketching() {
        egui::Panel::right("sketch-palette")
            .default_size(210.0)
            .show(ui, |ui| sketch_palette(editor, ui, &mut commands));
    }

    let free = ui.available_rect_before_wrap();
    let ctx = ui.ctx().clone();
    super::viewcube::show(editor, &ctx, free);
    tools::dialog(editor, &ctx);
    entry_overlay(editor, &ctx, &mut commands);
    dimension_overlay(editor, &ctx, &mut commands);
    constraint_overlay(editor, &ctx, &mut commands);
    error_popup(editor, &ctx);
    rename_popup(editor, &ctx);

    for c in commands {
        run(editor, c);
    }
}

fn run(editor: &mut Editor, c: Command) {
    match c {
        Command::Tool(kind) => tools::start_tool(editor, kind),
        Command::New => editor.new_document(),
        Command::Open => editor.open(),
        Command::Save(as_new) => editor.save(as_new),
        Command::ExportStl => editor.export_stl(),
        Command::Export3mf => editor.export_3mf(),
        Command::Quit => editor.quit(),
        Command::Undo => editor.undo(),
        Command::Redo => editor.redo(),
        Command::Fit => editor.zoom_to_fit(),
        Command::View(p) => editor.look_from(p),
        Command::ToggleProjection => editor.toggle_projection(),
        Command::SetCursor(c) => editor.set_cursor(c),
        Command::Edit(id) => editor.edit_feature(id),
        Command::Suppress(id, on) => {
            if let Err(e) = editor.doc.set_suppressed(id, on) {
                editor.report_error(e);
            }
        }
        Command::Delete(id) => editor.delete_feature(id),
        Command::Rename(id) => {
            let name = editor
                .doc
                .timeline()
                .get(id)
                .map(|f| f.name.clone())
                .unwrap_or_default();
            editor.rename = Some((id, name));
        }
        Command::SelectFeature(id) => editor.selected_feature = Some(id),
        Command::SelectMode(m) => editor.set_select_mode(m),
        Command::ToggleBody(b) => {
            if !editor.hidden_bodies.remove(&b) {
                editor.hidden_bodies.insert(b);
            }
        }
        Command::ToggleSketch(s) => {
            if !editor.hidden_sketches.remove(&s) {
                editor.hidden_sketches.insert(s);
            }
        }
        Command::Activate(c) => editor.active_component = c,
        Command::FinishSketch(keep) => sketch_mode::finish(editor, keep),
        Command::SketchTool(t) => {
            if let Mode::Sketch(s) = &mut editor.mode {
                s.set_tool(t);
            }
        }
        Command::SketchConstraint(c) => {
            if let Mode::Sketch(s) = &mut editor.mode {
                match s.add_constraint(c) {
                    Ok(()) => editor.commit_sketch(),
                    Err(e) => editor.report_error(e),
                }
            }
        }
        Command::SketchSelect(entities) => {
            if let Mode::Sketch(s) = &mut editor.mode {
                s.select_only(entities);
            }
        }
        Command::SketchDimension(id, v) => {
            if let Mode::Sketch(s) = &mut editor.mode {
                s.set_dimension(id, v);
                editor.commit_sketch();
            }
        }
        Command::SketchRemoveConstraint(id) => {
            if let Mode::Sketch(s) = &mut editor.mode {
                s.remove_constraint(id);
                editor.commit_sketch();
            }
        }
        Command::SketchDelete => editor.delete_selected(),
        Command::SketchSubmitEntry => {
            if let Mode::Sketch(s) = &mut editor.mode {
                s.submit_entry();
                if s.take_dirty() {
                    editor.commit_sketch();
                }
            }
        }
        Command::SketchMoveUpdate => {
            if let Mode::Sketch(s) = &mut editor.mode {
                s.update_move();
            }
        }
        Command::SketchMoveFinish(keep) => {
            if let Mode::Sketch(s) = &mut editor.mode {
                s.finish_move(keep);
                editor.commit_sketch();
            }
        }
        Command::SketchPattern => {
            if let Mode::Sketch(s) = &mut editor.mode {
                match s.apply_pattern() {
                    Ok(n) => {
                        editor.commit_sketch();
                        editor.set_status(format!("Pattern created {n} entities"));
                    }
                    Err(e) => editor.report_error(e),
                }
            }
        }
        Command::SketchSetParameter(name, expression) => {
            if let Mode::Sketch(s) = &mut editor.mode {
                match s.set_parameter(&name, &expression) {
                    Ok(()) => {
                        s.param_error = None;
                        editor.commit_sketch();
                    }
                    Err(e) => s.param_error = Some(e),
                }
            }
        }
        Command::SketchRemoveParameter(name) => {
            if let Mode::Sketch(s) = &mut editor.mode {
                s.remove_parameter(&name);
                s.param_error = None;
                editor.commit_sketch();
            }
        }
        Command::SketchBindDimension(id, expression) => {
            if let Mode::Sketch(s) = &mut editor.mode {
                match s.bind_dimension(id, &expression) {
                    Ok(()) => editor.commit_sketch(),
                    Err(e) => editor.report_error(e),
                }
            }
        }
        Command::SketchConstruction => {
            if let Mode::Sketch(s) = &mut editor.mode {
                s.toggle_construction();
                editor.commit_sketch();
            }
        }
        Command::SketchCommit => {
            if let Mode::Sketch(s) = &mut editor.mode
                && s.take_dirty()
            {
                editor.commit_sketch();
            }
        }
    }
    editor.request_repaint();
}

fn menu_bar(editor: &Editor, ui: &mut egui::Ui, commands: &mut Vec<Command>) {
    egui::MenuBar::new().ui(ui, |ui| {
        ui.menu_button("File", |ui| {
            if ui.button("New").clicked() {
                commands.push(Command::New);
            }
            if ui.button("Open…").clicked() {
                commands.push(Command::Open);
            }
            if ui.button("Save").clicked() {
                commands.push(Command::Save(false));
            }
            if ui.button("Save As…").clicked() {
                commands.push(Command::Save(true));
            }
            ui.separator();
            if ui.button("Export STL…").clicked() {
                commands.push(Command::ExportStl);
            }
            if ui.button("Export 3MF…").clicked() {
                commands.push(Command::Export3mf);
            }
            ui.separator();
            if ui.button("Quit").clicked() {
                commands.push(Command::Quit);
            }
        });
        ui.menu_button("Edit", |ui| {
            if ui
                .add_enabled(editor.doc.can_undo(), egui::Button::new("Undo"))
                .clicked()
            {
                commands.push(Command::Undo);
            }
            if ui
                .add_enabled(editor.doc.can_redo(), egui::Button::new("Redo"))
                .clicked()
            {
                commands.push(Command::Redo);
            }
        });
        ui.menu_button("View", |ui| {
            if ui.button("Fit (F)").clicked() {
                commands.push(Command::Fit);
            }
            for (name, p) in [
                ("Isometric", ViewPreset::Isometric),
                ("Top", ViewPreset::Top),
                ("Front", ViewPreset::Front),
                ("Right", ViewPreset::Right),
                ("Bottom", ViewPreset::Bottom),
                ("Back", ViewPreset::Back),
                ("Left", ViewPreset::Left),
            ] {
                if ui.button(name).clicked() {
                    commands.push(Command::View(p));
                }
            }
            ui.separator();
            if ui.button("Toggle orthographic").clicked() {
                commands.push(Command::ToggleProjection);
            }
        });
        ui.menu_button("Create", |ui| {
            for kind in [
                ToolKind::Sketch,
                ToolKind::Extrude,
                ToolKind::Revolve,
                ToolKind::Sweep,
                ToolKind::Loft,
                ToolKind::OffsetPlane,
                ToolKind::AngledPlane,
                ToolKind::Component,
            ] {
                if ui.button(kind.title()).clicked() {
                    commands.push(Command::Tool(kind));
                }
            }
        });
        ui.menu_button("Modify", |ui| {
            for kind in [
                ToolKind::Fillet,
                ToolKind::Chamfer,
                ToolKind::Combine,
                ToolKind::Move,
            ] {
                if ui.button(kind.title()).clicked() {
                    commands.push(Command::Tool(kind));
                }
            }
        });
    });
}

fn toolbar(editor: &Editor, ui: &mut egui::Ui, commands: &mut Vec<Command>) {
    if let Mode::Sketch(s) = &editor.mode {
        sketch_toolbar(s, ui, commands);
        return;
    }
    ui.horizontal_wrapped(|ui| {
        let busy = editor.tool.is_some();
        let groups: [&[(ToolKind, &str)]; 3] = [
            &[(ToolKind::Sketch, "Sketch")],
            &[
                (ToolKind::Extrude, "Extrude"),
                (ToolKind::Revolve, "Revolve"),
                (ToolKind::Sweep, "Sweep"),
                (ToolKind::Loft, "Loft"),
            ],
            &[
                (ToolKind::Fillet, "Fillet"),
                (ToolKind::Chamfer, "Chamfer"),
                (ToolKind::Combine, "Combine"),
                (ToolKind::Move, "Move"),
                (ToolKind::OffsetPlane, "Offset Plane"),
                (ToolKind::AngledPlane, "Angled Plane"),
            ],
        ];
        for group in groups {
            for (kind, label) in group {
                if ui.add_enabled(!busy, egui::Button::new(*label)).clicked() {
                    commands.push(Command::Tool(*kind));
                }
            }
            ui.separator();
        }
        if ui.button("Fit").clicked() {
            commands.push(Command::Fit);
        }
        ui.separator();
        ui.label("Select:");
        for (i, mode) in SelectMode::ALL.iter().enumerate() {
            let button = egui::Button::new(mode.name()).selected(editor.select_mode == *mode);
            if ui
                .add(button)
                .on_hover_text(format!("{} (press {})", mode.name(), i + 1))
                .clicked()
            {
                commands.push(Command::SelectMode(*mode));
            }
        }
        ui.separator();
        ui.label(egui::RichText::new("Right-drag orbit · middle-drag pan · wheel zoom").weak());
    });
}

/// Holding a folded tool button this long opens its variants, as in Fusion.
const LONG_PRESS_SECS: f64 = 0.35;

/// The sketch tools as a row of icons at the top, with the constraints of the moment
/// (construction, delete) and the way out. Variants of a shape share a button.
fn sketch_toolbar(s: &super::SketchEditor, ui: &mut egui::Ui, commands: &mut Vec<Command>) {
    ui.horizontal_wrapped(|ui| {
        for group in ToolGroup::ALL {
            let variant = s.variant_of(group);
            let selected = s.tool.group() == group;
            let response = tool_icon(ui, variant, selected).on_hover_ui(|ui| {
                ui.label(variant.name());
                if group.variants().len() > 1 {
                    ui.label(egui::RichText::new("Hold or right-click for other kinds").weak());
                }
            });
            let popup_id = ui.make_persistent_id(("tool-variants", group.name()));
            let mut open_menu = false;
            if group.variants().len() > 1 {
                let pressed = response.is_pointer_button_down_on();
                if pressed {
                    // The window only redraws when something asks it to, and a pointer
                    // held still sends no events: without this the frame that would
                    // notice the press has aged past the threshold is never drawn, and
                    // the menu never opens however long the button is held.
                    ui.ctx().request_repaint();
                }
                let held = pressed
                    && ui.input(|i| {
                        i.pointer
                            .press_start_time()
                            .is_some_and(|t| i.time - t > LONG_PRESS_SECS)
                    });
                open_menu = held || response.secondary_clicked();
            }
            // A plain click picks the variant shown; a long press is not a click, the
            // menu it opened takes over.
            if response.clicked() && !egui::Popup::is_id_open(ui.ctx(), popup_id) {
                commands.push(Command::SketchTool(variant));
            }
            if group.variants().len() > 1 {
                // The menu ignores egui's own click handling: the release that ends a
                // long press would otherwise close it the moment it opened. It closes
                // on a choice, or on a click that lands on neither it nor its button.
                let shown = egui::Popup::from_response(&response)
                    .id(popup_id)
                    .open_memory(open_menu.then_some(egui::SetOpenCommand::Bool(true)))
                    .close_behavior(egui::PopupCloseBehavior::IgnoreClicks)
                    .show(|ui| {
                        let mut chosen = false;
                        for tool in group.variants() {
                            ui.horizontal(|ui| {
                                tool_icon(ui, *tool, s.tool == *tool);
                                if ui.selectable_label(s.tool == *tool, tool.name()).clicked() {
                                    commands.push(Command::SketchTool(*tool));
                                    chosen = true;
                                }
                            });
                        }
                        chosen
                    });
                if let Some(shown) = shown {
                    let clicked_away = ui.input(|i| {
                        i.pointer.any_click()
                            && i.pointer.interact_pos().is_some_and(|p| {
                                !shown.response.rect.contains(p) && !response.rect.contains(p)
                            })
                    });
                    if shown.inner || clicked_away {
                        egui::Popup::close_id(ui.ctx(), popup_id);
                    }
                }
            }
        }
        ui.separator();
        // Constraints live in the toolbar, as they do in Fusion: this is where the user
        // looks after selecting something, and it is always in view, which the palette's
        // lower reaches are not. The whole set is shown so the user can see what exists;
        // only what the selection supports is enabled.
        let applicable = s.applicable_constraints();
        for name in super::SketchEditor::CONSTRAINT_NAMES {
            let offered = applicable.iter().find(|(n, _)| *n == name);
            let response = ui
                .add_enabled(offered.is_some(), egui::Button::new(name))
                .on_disabled_hover_text("Select the geometry this applies to");
            if response.clicked()
                && let Some((_, c)) = offered
            {
                commands.push(Command::SketchConstraint(c.clone()));
            }
        }
        ui.separator();
        let construction =
            egui::Button::new("Construction (X)").selected(s.selection_is_construction());
        if ui
            .add_enabled(!s.selected.is_empty(), construction)
            .on_hover_text("Toggle construction geometry on the selection")
            .clicked()
        {
            commands.push(Command::SketchConstruction);
        }
        if ui
            .add_enabled(!s.selected.is_empty(), egui::Button::new("Delete"))
            .clicked()
        {
            commands.push(Command::SketchDelete);
        }
        ui.separator();
        if ui.button("✔ Finish Sketch").clicked() {
            commands.push(Command::FinishSketch(true));
        }
        if ui.button("✖ Cancel Sketch").clicked() {
            commands.push(Command::FinishSketch(false));
        }
    });
}

/// An amber note in the status bar for features that built but warn about something.
///
/// The degrees-of-freedom readout used to live only at the bottom of the sketch palette,
/// which is open only while sketching — so the moment it matters most, when an edit
/// propagates through the timeline and the loose geometry actually moves, nothing said a
/// word. This is always in view, and clicking it selects the feature that warned.
fn warning_summary(editor: &Editor, ui: &mut egui::Ui, commands: &mut Vec<Command>) {
    let name_of = |id: FeatureId| {
        editor
            .doc
            .timeline()
            .features()
            .iter()
            .find(|f| f.id == id)
            .map_or("feature", |f| f.name.as_str())
    };
    let warned: Vec<(FeatureId, String)> = editor
        .cached_statuses
        .iter()
        .filter_map(|(id, s)| match s {
            FeatureStatus::Warned(msg) => Some((*id, msg.clone())),
            _ => None,
        })
        .collect();
    let Some((first, _)) = warned.first() else {
        return;
    };
    // One warning is named outright, because the name is what the user needs to act on
    // and there is room for it; several are counted, and the click takes them to the
    // first, which the hover text says so that it is not a surprise.
    let text = match warned.len() {
        1 => format!("\u{26a0} {}", name_of(*first)),
        n => format!("\u{26a0} {n} features need attention"),
    };
    let response = ui
        .add(egui::Button::new(egui::RichText::new(text).color(WARNING_LABEL)).frame(false))
        .on_hover_ui(|ui| {
            for (id, msg) in &warned {
                ui.colored_label(WARNING_LABEL, format!("{}: {msg}", name_of(*id)));
            }
            let goes_to = name_of(*first);
            ui.label(egui::RichText::new(format!("Click selects {goes_to}")).weak());
        });
    if response.clicked() {
        commands.push(Command::SelectFeature(*first));
    }
}

/// A tool button with its icon painted rather than typed: the default fonts have no
/// reliable glyphs for these shapes, and a drawn icon reads the same on every machine.
fn tool_icon(ui: &mut egui::Ui, tool: SketchTool, selected: bool) -> egui::Response {
    const SIZE: f32 = 28.0;
    let (rect, response) = ui.allocate_exact_size(egui::vec2(SIZE, SIZE), egui::Sense::click());
    let visuals = ui.style().interact_selectable(&response, selected);
    let painter = ui.painter();
    painter.rect(
        rect,
        visuals.corner_radius,
        visuals.weak_bg_fill,
        visuals.bg_stroke,
        egui::StrokeKind::Inside,
    );
    let stroke = egui::Stroke::new(1.6, visuals.fg_stroke.color);
    let dot = |p: egui::Pos2| painter.circle_filled(p, 1.8, stroke.color);
    let inner = rect.shrink(6.0);
    let (l, r, t, b) = (inner.left(), inner.right(), inner.top(), inner.bottom());
    let c = inner.center();
    let line = |a: egui::Pos2, b: egui::Pos2| painter.line_segment([a, b], stroke);
    let arc = |center: egui::Pos2, radius: f32, from: f32, to: f32| {
        let n = 12;
        let pts: Vec<egui::Pos2> = (0..=n)
            .map(|i| {
                let a = from + (to - from) * i as f32 / n as f32;
                center + egui::vec2(a.cos(), -a.sin()) * radius
            })
            .collect();
        painter.add(egui::Shape::line(pts, stroke));
    };
    let pi = std::f32::consts::PI;
    match tool {
        SketchTool::Select => {
            // A pointer arrow.
            let tip = egui::pos2(l + 2.0, t + 1.0);
            painter.add(egui::Shape::convex_polygon(
                vec![
                    tip,
                    egui::pos2(tip.x, b - 2.0),
                    egui::pos2(tip.x + 4.0, b - 6.0),
                    egui::pos2(tip.x + 10.0, b - 3.0),
                    egui::pos2(tip.x + 12.0, b - 6.0),
                    egui::pos2(tip.x + 6.0, b - 9.0),
                    egui::pos2(r - 2.0, b - 9.0),
                ],
                stroke.color,
                egui::Stroke::NONE,
            ));
        }
        SketchTool::Line => {
            line(egui::pos2(l, b), egui::pos2(r, t));
            dot(egui::pos2(l, b));
            dot(egui::pos2(r, t));
        }
        SketchTool::Rectangle => {
            painter.rect_stroke(inner, 0.0, stroke, egui::StrokeKind::Middle);
            dot(egui::pos2(l, b));
            dot(egui::pos2(r, t));
        }
        SketchTool::CenterRectangle => {
            painter.rect_stroke(inner, 0.0, stroke, egui::StrokeKind::Middle);
            dot(c);
            dot(egui::pos2(r, t));
        }
        SketchTool::Circle => {
            painter.circle_stroke(c, inner.width() * 0.5, stroke);
            dot(c);
        }
        SketchTool::Circle2Point => {
            painter.circle_stroke(c, inner.width() * 0.5, stroke);
            dot(egui::pos2(l, c.y));
            dot(egui::pos2(r, c.y));
        }
        SketchTool::Circle3Point => {
            let radius = inner.width() * 0.5;
            painter.circle_stroke(c, radius, stroke);
            for a in [pi * 0.5, pi * 7.0 / 6.0, pi * 11.0 / 6.0] {
                dot(c + egui::vec2(a.cos(), -a.sin()) * radius);
            }
        }
        SketchTool::Arc3Point => {
            let center = egui::pos2(c.x, b);
            arc(center, inner.width() * 0.5, 0.0, pi);
            dot(egui::pos2(l, b));
            dot(egui::pos2(c.x, t));
            dot(egui::pos2(r, b));
        }
        SketchTool::ArcCenter => {
            let center = egui::pos2(c.x, b);
            arc(center, inner.width() * 0.5, 0.0, pi);
            dot(center);
            dot(egui::pos2(r, b));
        }
        SketchTool::Polygon => {
            let radius = inner.width() * 0.5;
            let pts: Vec<egui::Pos2> = (0..6)
                .map(|i| {
                    let a = pi / 3.0 * i as f32;
                    c + egui::vec2(a.cos(), -a.sin()) * radius
                })
                .collect();
            painter.add(egui::Shape::closed_line(pts, stroke));
        }
        SketchTool::Slot | SketchTool::SlotOverall | SketchTool::SlotCenterPoint => {
            let radius = inner.height() * 0.3;
            let (ca, cb) = (egui::pos2(l + radius, c.y), egui::pos2(r - radius, c.y));
            line(
                egui::pos2(ca.x, c.y - radius),
                egui::pos2(cb.x, c.y - radius),
            );
            line(
                egui::pos2(ca.x, c.y + radius),
                egui::pos2(cb.x, c.y + radius),
            );
            arc(ca, radius, pi * 0.5, pi * 1.5);
            arc(cb, radius, -pi * 0.5, pi * 0.5);
            match tool {
                SketchTool::Slot => {
                    dot(ca);
                    dot(cb);
                }
                SketchTool::SlotOverall => {
                    dot(egui::pos2(l, c.y));
                    dot(egui::pos2(r, c.y));
                }
                _ => {
                    dot(c);
                    dot(cb);
                }
            }
        }
        SketchTool::Text => {
            painter.text(
                c,
                egui::Align2::CENTER_CENTER,
                "T",
                egui::FontId::proportional(18.0),
                stroke.color,
            );
        }
        SketchTool::Trim | SketchTool::Break => {
            // A line crossed by a second one; for Trim the crossed-off piece is gone,
            // for Break the two halves are drawn apart.
            let y = c.y;
            line(egui::pos2(l, t + 2.0), egui::pos2(l + 7.0, b - 2.0));
            if tool == SketchTool::Trim {
                line(egui::pos2(l + 6.0, y), egui::pos2(r, y));
                dot(egui::pos2(l + 3.0, y));
            } else {
                line(egui::pos2(l, y - 2.0), egui::pos2(l + 4.0, y - 2.0));
                line(egui::pos2(l + 8.0, y + 2.0), egui::pos2(r, y + 2.0));
            }
        }
        SketchTool::Dimension => {
            // A dimension line with arrowheads and extension lines.
            line(egui::pos2(l, t), egui::pos2(l, b));
            line(egui::pos2(r, t), egui::pos2(r, b));
            let y = c.y;
            line(egui::pos2(l, y), egui::pos2(r, y));
            for (x, d) in [(l, 1.0), (r, -1.0)] {
                line(egui::pos2(x, y), egui::pos2(x + 4.0 * d, y - 3.0));
                line(egui::pos2(x, y), egui::pos2(x + 4.0 * d, y + 3.0));
            }
        }
    }
    response
}

fn browser(editor: &mut Editor, ui: &mut egui::Ui, commands: &mut Vec<Command>) {
    ui.heading(&editor.doc.name);
    egui::ScrollArea::vertical().show(ui, |ui| {
        ui.collapsing("Origin", |ui| {
            ui.checkbox(&mut editor.show_origin, "Show origin planes and axes");
            ui.checkbox(&mut editor.show_grid, "Show grid");
        });
        let planes = editor.cached_planes.clone();
        if !planes.is_empty() {
            ui.collapsing("Construction planes", |ui| {
                for (id, _) in planes {
                    let name = editor.feature_name(id);
                    let selected = editor.selected_feature == Some(id);
                    if ui.selectable_label(selected, name).clicked() {
                        commands.push(Command::SelectFeature(id));
                        editor.selection.clear();
                        editor.selection.planes.push(PlaneRef::Feature(id));
                    }
                }
            });
        }
        let components = editor.cached_components.clone();
        component_tree(editor, ui, ComponentId::ROOT, &components, commands);
    });
}

fn component_tree(
    editor: &mut Editor,
    ui: &mut egui::Ui,
    id: ComponentId,
    components: &[(ComponentId, String, Option<ComponentId>)],
    commands: &mut Vec<Command>,
) {
    let name = components
        .iter()
        .find(|(c, _, _)| *c == id)
        .map(|(_, n, _)| n.clone())
        .unwrap_or_default();
    let active = editor.active_component == id;
    let header = if active {
        format!("● {name}")
    } else {
        name.clone()
    };
    egui::CollapsingHeader::new(header)
        .id_salt(("component", id.0))
        .default_open(true)
        .show(ui, |ui| {
            if !active && ui.small_button("Activate").clicked() {
                commands.push(Command::Activate(id));
            }
            let bodies: Vec<(BodyRef, String)> = editor
                .cached_bodies
                .iter()
                .filter(|(b, _)| editor.cached_body_components.get(b) == Some(&id))
                .cloned()
                .collect();
            if !bodies.is_empty() {
                ui.label(egui::RichText::new("Bodies").weak());
                for (b, bname) in bodies {
                    ui.horizontal(|ui| {
                        let mut visible = !editor.hidden_bodies.contains(&b);
                        if ui.checkbox(&mut visible, "").changed() {
                            commands.push(Command::ToggleBody(b));
                        }
                        let selected = editor.selection.bodies.contains(&b);
                        if ui.selectable_label(selected, bname).clicked() {
                            editor.selection.clear();
                            editor.selection.bodies.push(b);
                            commands.push(Command::SelectFeature(b.0));
                        }
                    });
                }
            }
            let sketches: Vec<FeatureId> = editor
                .cached_sketches
                .iter()
                .filter(|(_, s)| s.component == id)
                .map(|(sid, _)| *sid)
                .collect();
            if !sketches.is_empty() {
                ui.label(egui::RichText::new("Sketches").weak());
                for s in sketches {
                    ui.horizontal(|ui| {
                        let mut visible = !editor.hidden_sketches.contains(&s);
                        if ui.checkbox(&mut visible, "").changed() {
                            commands.push(Command::ToggleSketch(s));
                        }
                        let label = ui.selectable_label(
                            editor.selected_feature == Some(s),
                            editor.feature_name(s),
                        );
                        if label.clicked() {
                            commands.push(Command::SelectFeature(s));
                        }
                        if label.double_clicked() {
                            commands.push(Command::Edit(s));
                        }
                    });
                }
            }
            let children: Vec<ComponentId> = components
                .iter()
                .filter(|(_, _, p)| *p == Some(id))
                .map(|(c, _, _)| *c)
                .collect();
            for child in children {
                component_tree(editor, ui, child, components, commands);
            }
        });
}

fn timeline(editor: &Editor, ui: &mut egui::Ui, commands: &mut Vec<Command>) {
    let timeline = editor.doc.timeline();
    let cursor = timeline.cursor();
    let len = timeline.len();
    let locked = editor.tool.is_some() || editor.is_sketching();
    ui.horizontal(|ui| {
        ui.add_enabled_ui(!locked, |ui| {
            if ui.button("⏮").clicked() {
                commands.push(Command::SetCursor(0));
            }
            if ui.button("◀").clicked() {
                commands.push(Command::SetCursor(cursor.saturating_sub(1)));
            }
            if ui.button("▶").clicked() {
                commands.push(Command::SetCursor((cursor + 1).min(len)));
            }
            if ui.button("⏭").clicked() {
                commands.push(Command::SetCursor(len));
            }
        });
        ui.separator();
        egui::ScrollArea::horizontal().show(ui, |ui| {
            ui.horizontal(|ui| {
                for (i, f) in timeline.features().iter().enumerate() {
                    if i == cursor {
                        cursor_marker(ui, true);
                    }
                    let status = editor.cached_statuses.get(&f.id);
                    let abbrev = abbreviation(&f.kind);
                    let mut text = egui::RichText::new(abbrev);
                    if i >= cursor {
                        text = text.weak();
                    }
                    if f.suppressed {
                        text = text.strikethrough();
                    }
                    let fill = match status {
                        Some(FeatureStatus::Failed(_)) => {
                            Some(egui::Color32::from_rgb(120, 50, 40))
                        }
                        Some(FeatureStatus::Warned(_)) => {
                            Some(egui::Color32::from_rgb(110, 85, 30))
                        }
                        _ => None,
                    };
                    let mut button =
                        egui::Button::new(text).selected(editor.selected_feature == Some(f.id));
                    if let Some(fill) = fill {
                        button = button.fill(fill);
                    }
                    let response = ui.add(button).on_hover_ui(|ui| {
                        ui.label(&f.name);
                        match status {
                            Some(FeatureStatus::Failed(msg)) => {
                                ui.colored_label(egui::Color32::from_rgb(230, 120, 100), msg);
                            }
                            Some(FeatureStatus::Warned(msg)) => {
                                ui.colored_label(WARNING_LABEL, msg);
                            }
                            Some(FeatureStatus::Suppressed) => {
                                ui.label("suppressed");
                            }
                            _ => {}
                        }
                    });
                    if response.clicked() {
                        commands.push(Command::SelectFeature(f.id));
                    }
                    if response.double_clicked() && !locked {
                        commands.push(Command::Edit(f.id));
                    }
                    response.context_menu(|ui| {
                        ui.label(&f.name);
                        ui.separator();
                        if ui.add_enabled(!locked, egui::Button::new("Edit")).clicked() {
                            commands.push(Command::Edit(f.id));
                            ui.close();
                        }
                        if ui.button("Rename").clicked() {
                            commands.push(Command::Rename(f.id));
                            ui.close();
                        }
                        let label = if f.suppressed {
                            "Unsuppress"
                        } else {
                            "Suppress"
                        };
                        if ui.add_enabled(!locked, egui::Button::new(label)).clicked() {
                            commands.push(Command::Suppress(f.id, !f.suppressed));
                            ui.close();
                        }
                        if ui
                            .add_enabled(!locked, egui::Button::new("Roll to here"))
                            .clicked()
                        {
                            commands.push(Command::SetCursor(i + 1));
                            ui.close();
                        }
                        if ui
                            .add_enabled(!locked, egui::Button::new("Delete"))
                            .clicked()
                        {
                            commands.push(Command::Delete(f.id));
                            ui.close();
                        }
                    });
                }
                if cursor == len {
                    cursor_marker(ui, true);
                }
            });
        });
    });
}

fn cursor_marker(ui: &mut egui::Ui, _active: bool) {
    let (rect, _) = ui.allocate_exact_size(egui::vec2(6.0, 22.0), egui::Sense::hover());
    ui.painter()
        .rect_filled(rect, 1.0, egui::Color32::from_rgb(90, 160, 255));
}

fn abbreviation(kind: &FeatureKind) -> &'static str {
    match kind {
        FeatureKind::NewComponent { .. } => "Cmp",
        FeatureKind::Sketch { .. } => "Sk",
        FeatureKind::OffsetPlane { .. } => "Pl+",
        FeatureKind::AngledPlane { .. } => "Pl∠",
        FeatureKind::Extrude { .. } => "Ext",
        FeatureKind::Revolve { .. } => "Rev",
        FeatureKind::Sweep { .. } => "Swp",
        FeatureKind::Loft { .. } => "Lft",
        FeatureKind::Fillet { .. } => "Fil",
        FeatureKind::Chamfer { .. } => "Chm",
        FeatureKind::Combine { .. } => "Cmb",
        FeatureKind::Move { .. } => "Mov",
    }
}

fn sketch_palette(editor: &mut Editor, ui: &mut egui::Ui, commands: &mut Vec<Command>) {
    let Mode::Sketch(s) = &mut editor.mode else {
        return;
    };
    let mut centre_on_selection = false;
    // The palette is taller than the panel on an ordinary window, and without a scroll
    // area whatever overflowed was simply unreachable — which is how a sketch could end
    // up with no way to apply a constraint at all.
    egui::ScrollArea::vertical().show(ui, |ui| {
        ui.heading("Sketch");
        ui.label(egui::RichText::new("Click points; snap to existing points to join").weak());
        ui.separator();
        ui.horizontal(|ui| {
            ui.checkbox(&mut s.snap_to_grid, "Snap to grid");
            if s.snap_to_grid {
                let label = match s.fixed_grid_step {
                    Some(step) => format!("{step} mm"),
                    None => format!("{} mm (auto)", s.grid_step),
                };
                ui.label(egui::RichText::new(label).weak());
            }
        });
        if s.snap_to_grid {
            // Pinning the step is what a drawing with a stated increment needs; leaving it
            // automatic keeps the snap matched to what the grid is actually showing.
            let mut pinned = s.fixed_grid_step.is_some();
            ui.horizontal(|ui| {
                if ui.checkbox(&mut pinned, "Fixed step").changed() {
                    s.fixed_grid_step = pinned.then_some(s.grid_step);
                }
                if let Some(step) = s.fixed_grid_step.as_mut() {
                    ui.add(
                        egui::DragValue::new(step)
                            .speed(0.1)
                            .range(1e-3..=1e4)
                            .suffix(" mm"),
                    );
                }
            });
        }
        ui.separator();
        ui.label(s.tool.name());
        if s.tool == SketchTool::Select {
            ui.label(
                egui::RichText::new(
                    "Click inside a closed region to select its curves. Drag geometry to move \
                     it. Drag on empty space to box-select: right encloses, left touches",
                )
                .weak(),
            );
            if s.has_region_selection() {
                ui.colored_label(
                    egui::Color32::from_rgb(140, 200, 255),
                    "Press E to extrude this region",
                );
            }
        } else if matches!(s.tool, SketchTool::Trim | SketchTool::Break) {
            let hint = if s.tool == SketchTool::Trim {
                "Click the piece to remove: it is cut at the curves that cross it, and the \
                 piece under the pointer is drawn in red. A curve nothing crosses goes whole"
            } else {
                "Click a curve to cut it at every crossing, keeping both sides joined"
            };
            ui.label(egui::RichText::new(hint).weak());
        } else if !s.tool.dims().is_empty() {
            ui.label(
                egui::RichText::new(
                    "After the first click the boxes follow the pointer; typing locks a size, Tab \
                     moves to the next, Enter places the shape",
                )
                .weak(),
            );
        }
        ui.separator();
        match s.tool {
            SketchTool::Polygon => {
                ui.add(egui::Slider::new(&mut s.polygon_sides, 3..=24).text("sides"));
            }
            SketchTool::Dimension => {
                let hint = if s.dim_first.is_some() {
                    "Pick a second entity, or click empty space to dimension the first alone"
                } else {
                    "Pick a line, circle, arc or point. Two parallel lines: distance; two \
                     others: angle; a circle with a line or point: distance from its centre"
                };
                ui.label(egui::RichText::new(hint).weak());
            }
            SketchTool::Text => {
                ui.text_edit_singleline(&mut s.text);
                ui.add(
                    egui::DragValue::new(&mut s.text_height)
                        .speed(0.5)
                        .prefix("height ")
                        .suffix(" mm"),
                );
                if s.sketch.font().is_none() {
                    ui.colored_label(
                        egui::Color32::YELLOW,
                        "No font found: text will not produce profiles",
                    );
                }
            }
            _ => {}
        }
        ui.separator();
        ui.label(
            egui::RichText::new("Constraints are in the toolbar; select 1–3 entities first").weak(),
        );
        ui.separator();
        match &s.report {
            Some(Ok(r)) => {
                let dof = r.degrees_of_freedom;
                if dof == 0 {
                    ui.label("Fully constrained");
                } else {
                    // Blue is the readout the user actually reads: it is on the geometry
                    // they are looking at, not in a corner of a palette.
                    ui.colored_label(LOOSE_LABEL, format!("{dof} degrees of freedom"));
                    ui.label(
                        egui::RichText::new(
                            "Blue geometry is still free to move: a later dimension change \
                             can drag it off what it was drawn against",
                        )
                        .weak(),
                    );
                }
                if !r.converged {
                    ui.colored_label(egui::Color32::YELLOW, "Constraints conflict");
                }
            }
            Some(Err(e)) => {
                ui.colored_label(egui::Color32::from_rgb(230, 120, 100), e);
            }
            None => {}
        }
        // Directly under the degrees-of-freedom readout, because that is the line that
        // prompts the question the list answers: what *is* holding this sketch?
        constraint_list(s, ui, commands);
        if let Some(op) = s.move_op.as_mut() {
            ui.separator();
            ui.label("Move (M)");
            let mut changed = false;
            for (value, label, unit) in [
                (&mut op.dx, "dX ", " mm"),
                (&mut op.dy, "dY ", " mm"),
                (&mut op.angle_deg, "rotate ", "°"),
            ] {
                changed |= ui
                    .add(
                        egui::DragValue::new(value)
                            .speed(0.5)
                            .prefix(label)
                            .suffix(unit),
                    )
                    .changed();
            }
            ui.label(
                egui::RichText::new("Enter applies, Esc puts it back. Constraints still hold")
                    .weak(),
            );
            ui.horizontal(|ui| {
                if ui.button("Apply").clicked() {
                    commands.push(Command::SketchMoveFinish(true));
                }
                if ui.button("Cancel").clicked() {
                    commands.push(Command::SketchMoveFinish(false));
                }
            });
            if changed {
                commands.push(Command::SketchMoveUpdate);
            }
        }
        ui.separator();
        egui::CollapsingHeader::new("Pattern")
            .default_open(false)
            .show(ui, |ui| {
                let p = &mut s.pattern;
                ui.horizontal(|ui| {
                    ui.selectable_value(&mut p.circular, false, "Rectangular");
                    ui.selectable_value(&mut p.circular, true, "Circular");
                });
                if p.circular {
                    ui.add(egui::Slider::new(&mut p.count, 2..=64).text("instances"));
                    ui.add(
                        egui::DragValue::new(&mut p.angle_deg)
                            .speed(1.0)
                            .range(-360.0..=360.0)
                            .suffix("° total"),
                    );
                    ui.horizontal(|ui| {
                        ui.add(
                            egui::DragValue::new(&mut p.center.x)
                                .speed(0.5)
                                .prefix("x "),
                        );
                        ui.add(
                            egui::DragValue::new(&mut p.center.y)
                                .speed(0.5)
                                .prefix("y "),
                        );
                    });
                    if ui.button("Centre on selection").clicked() {
                        centre_on_selection = true;
                    }
                } else {
                    ui.horizontal(|ui| {
                        ui.add(
                            egui::DragValue::new(&mut p.cols)
                                .range(1..=64)
                                .prefix("cols "),
                        );
                        ui.add(egui::DragValue::new(&mut p.dx).speed(0.5).prefix("dX "));
                    });
                    ui.horizontal(|ui| {
                        ui.add(
                            egui::DragValue::new(&mut p.rows)
                                .range(1..=64)
                                .prefix("rows "),
                        );
                        ui.add(egui::DragValue::new(&mut p.dy).speed(0.5).prefix("dY "));
                    });
                }
                let enabled = !s.selected.is_empty();
                if ui
                    .add_enabled(enabled, egui::Button::new("Repeat selection"))
                    .on_disabled_hover_text("Select the geometry to repeat first")
                    .clicked()
                {
                    commands.push(Command::SketchPattern);
                }
            });
        ui.separator();
        egui::CollapsingHeader::new("Parameters")
            .default_open(false)
            .show(ui, |ui| parameters_panel(s, ui, commands));
    });
    if centre_on_selection {
        s.pattern_center_from_selection();
    }
}

/// Every constraint in the sketch, as a list.
///
/// The badges in the viewport are the primary way to see and delete a constraint, but a
/// badge can sit under other geometry, and a sketch that has gone wrong is exactly the
/// one whose badges are hard to read. The list is the fallback that always works:
/// hovering a row lights up the geometry the constraint holds, and every row can be
/// deleted from here.
fn constraint_list(s: &mut super::SketchEditor, ui: &mut egui::Ui, commands: &mut Vec<Command>) {
    let rows: Vec<(
        basset_sketch::ConstraintId,
        String,
        Vec<basset_sketch::EntityId>,
    )> = s
        .sketch
        .constraints()
        .map(|(id, c)| {
            let label = match c.dimension_value() {
                // Angles are stored in radians and shown in degrees, as everywhere else.
                Some(v) if matches!(c, basset_sketch::Constraint::Angle { .. }) => {
                    format!("{} {:.2}\u{b0}", constraint_name(c), v.to_degrees())
                }
                Some(v) => format!("{} {v:.3} mm", constraint_name(c)),
                None => constraint_name(c).to_string(),
            };
            (id, label, c.references())
        })
        .collect();
    let mut highlight = Vec::new();
    egui::CollapsingHeader::new(format!("Constraints ({})", rows.len()))
        .default_open(false)
        .show(ui, |ui| {
            if rows.is_empty() {
                ui.label(
                    egui::RichText::new("Nothing holds this sketch yet: every point is free")
                        .weak(),
                );
            }
            for (id, label, entities) in &rows {
                ui.horizontal(|ui| {
                    if ui
                        .add(egui::Button::new("\u{2715}").frame(false))
                        .on_hover_text("Delete")
                        .clicked()
                    {
                        commands.push(Command::SketchRemoveConstraint(*id));
                    }
                    // Clicking selects what the constraint holds, so the toolbar and the
                    // keyboard act on that geometry without hunting for it in the viewport.
                    let response = ui.add(egui::Label::new(label).sense(egui::Sense::click()));
                    if response.contains_pointer() {
                        highlight = entities.clone();
                    }
                    if response.clicked() {
                        commands.push(Command::SketchSelect(entities.clone()));
                    }
                });
            }
        });
    s.highlighted = highlight;
}

/// The named constants of the sketch: name, expression, and what it currently works out
/// to. Editing is deliberately explicit — an expression is only taken when the user
/// leaves the box or presses Enter — because a half-typed name is not an error worth
/// shouting about.
fn parameters_panel(s: &mut super::SketchEditor, ui: &mut egui::Ui, commands: &mut Vec<Command>) {
    s.sync_param_drafts();
    let drafts = s.param_drafts.clone();
    for (index, (name, expression)) in drafts.iter().enumerate() {
        ui.horizontal(|ui| {
            ui.label(egui::RichText::new(name).strong());
            let draft = &mut s.param_drafts[index].1;
            let response = ui.add(
                egui::TextEdit::singleline(draft)
                    .desired_width(90.0)
                    .hint_text("expression"),
            );
            let submit = response.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
            if submit || (response.lost_focus() && *draft != *expression) {
                commands.push(Command::SketchSetParameter(name.clone(), draft.clone()));
            }
            match s.sketch.parameter_value(name) {
                Ok(v) => ui.label(egui::RichText::new(format!("= {v:.3}")).weak()),
                Err(_) => ui.colored_label(egui::Color32::YELLOW, "?"),
            };
            if ui.small_button("✕").clicked() {
                commands.push(Command::SketchRemoveParameter(name.clone()));
            }
        });
    }
    ui.horizontal(|ui| {
        ui.add(
            egui::TextEdit::singleline(&mut s.new_param.0)
                .desired_width(70.0)
                .hint_text("name"),
        );
        ui.add(
            egui::TextEdit::singleline(&mut s.new_param.1)
                .desired_width(90.0)
                .hint_text("value or expression"),
        );
        let ready = !s.new_param.0.trim().is_empty() && !s.new_param.1.trim().is_empty();
        if ui.add_enabled(ready, egui::Button::new("Add")).clicked() {
            commands.push(Command::SketchSetParameter(
                s.new_param.0.trim().to_string(),
                s.new_param.1.trim().to_string(),
            ));
            s.new_param = (String::new(), String::new());
        }
    });
    if let Some(e) = &s.param_error {
        ui.colored_label(egui::Color32::from_rgb(230, 120, 100), e);
    }
    ui.label(
        egui::RichText::new(
            "Click a dimension and type a name or a sum to drive it, e.g. wall * 2",
        )
        .weak(),
    );
}

/// Entry boxes for the sizes of the shape being drawn, hung off its last click so they
/// stay put while the pointer roams. Typing in the viewport lands here too (the editor
/// asks for focus through `entry_focus`), Tab moves on, Enter places the shape.
fn entry_overlay(editor: &mut Editor, ctx: &egui::Context, commands: &mut Vec<Command>) {
    let Mode::Sketch(s) = &mut editor.mode else {
        return;
    };
    if s.entries.is_empty() || !s.has_pending() {
        return;
    }
    let Some(anchor) = s.entry_anchor() else {
        return;
    };
    let Some(px) = editor.camera.world_to_screen(anchor, editor.window_px) else {
        return;
    };
    let ppp = ctx.pixels_per_point();
    let pos = egui::pos2(px[0] as f32 / ppp + 18.0, px[1] as f32 / ppp + 18.0);
    let focus = s.entry_focus.take();
    let count = s.entries.len();
    let mut next_focus: Option<usize> = None;
    let mut edited: Option<usize> = None;
    let mut toggled: Option<usize> = None;
    let mut submit = false;
    egui::Area::new(egui::Id::new("sketch-entry"))
        .fixed_pos(pos)
        .order(egui::Order::Foreground)
        .show(ctx, |ui| {
            egui::Frame::popup(ui.style()).show(ui, |ui| {
                egui::Grid::new("sketch-entry-grid").show(ui, |ui| {
                    for (i, entry) in s.entries.iter_mut().enumerate() {
                        ui.label(entry.dim.label());
                        let id = ui.make_persistent_id(("sketch-entry", i));
                        let text = egui::TextEdit::singleline(&mut entry.text)
                            .id(id)
                            .desired_width(64.0)
                            .text_color_opt(entry.locked.then_some(egui::Color32::WHITE));
                        let response = ui.add(text);
                        if focus == Some(i) {
                            response.request_focus();
                        }
                        // A box taken by Tab or click still shows the live value; the
                        // first keystroke replaces it, so select it all on arrival.
                        if response.gained_focus() {
                            let mut state =
                                egui::TextEdit::load_state(ui.ctx(), id).unwrap_or_default();
                            let end = egui::text::CCursor::new(entry.text.chars().count());
                            state
                                .cursor
                                .set_char_range(Some(egui::text::CCursorRange::two(
                                    egui::text::CCursor::new(0),
                                    end,
                                )));
                            state.store(ui.ctx(), id);
                        }
                        if response.changed() {
                            edited = Some(i);
                        }
                        if response.lost_focus() {
                            let (enter, tab) = ui.input(|i| {
                                (
                                    i.key_pressed(egui::Key::Enter),
                                    i.key_pressed(egui::Key::Tab),
                                )
                            });
                            submit |= enter;
                            // Tab cycles through the boxes and wraps, rather than
                            // wandering off into the rest of the UI.
                            if tab {
                                next_focus = Some((i + 1) % count);
                            }
                        }
                        ui.label(egui::RichText::new(entry.dim.unit()).weak());
                        // Not focusable, so Tab goes straight from one box to the next.
                        let lock = egui::Button::new(if entry.locked { "🔒" } else { "🔓" })
                            .small()
                            .sense(egui::Sense::CLICK);
                        if ui
                            .add(lock)
                            .on_hover_text("Locked sizes ignore the pointer; click to release")
                            .clicked()
                        {
                            toggled = Some(i);
                        }
                        ui.end_row();
                    }
                });
            });
        });
    if next_focus.is_some() {
        s.entry_focus = next_focus;
    }
    if let Some(i) = edited {
        s.lock_entry(i);
    }
    if let Some(i) = toggled {
        if s.entries[i].locked {
            s.unlock_entry(i);
        } else {
            s.lock_entry(i);
        }
    }
    if submit {
        commands.push(Command::SketchSubmitEntry);
    }
}

/// A hit area over every constraint badge, so a constraint can be named and removed.
///
/// The badge itself is drawn with the sketch geometry; this puts an invisible button on
/// top of it. Without one a geometric constraint could be applied but never inspected or
/// undone, because it has no value text to click the way a dimension does.
fn constraint_overlay(editor: &mut Editor, ctx: &egui::Context, commands: &mut Vec<Command>) {
    let camera = editor.camera;
    let window = editor.window_px;
    let Mode::Sketch(s) = &mut editor.mode else {
        return;
    };
    let ppp = ctx.pixels_per_point();
    for g in &s.constraint_glyphs() {
        let Some(px) = camera.world_to_screen(g.center, window) else {
            continue;
        };
        let Some(name) = s.sketch.constraint(g.id).map(constraint_name) else {
            continue;
        };
        let pos = egui::pos2(px[0] as f32 / ppp, px[1] as f32 / ppp);
        let size = egui::vec2(16.0, 16.0);
        egui::Area::new(egui::Id::new(("constraint", g.id, g.target)))
            .fixed_pos(pos - size * 0.5)
            .order(egui::Order::Foreground)
            .show(ctx, |ui| {
                let response = ui.allocate_response(size, egui::Sense::click());
                response
                    .clone()
                    .on_hover_text(format!("{name} \u{2014} right-click to delete"));
                response.context_menu(|ui| {
                    if ui
                        .button(format!("Delete {}", name.to_lowercase()))
                        .clicked()
                    {
                        commands.push(Command::SketchRemoveConstraint(g.id));
                        ui.close();
                    }
                });
            });
    }
}

/// What a constraint is called, for the badge tooltip and its delete entry.
fn constraint_name(c: &basset_sketch::Constraint) -> &'static str {
    use basset_sketch::Constraint as C;
    match c {
        C::Coincident { .. } => "Coincident",
        C::Horizontal(_) => "Horizontal",
        C::Vertical(_) => "Vertical",
        C::Parallel(..) => "Parallel",
        C::Perpendicular(..) => "Perpendicular",
        C::Equal(..) => "Equal",
        C::Tangent(..) => "Tangent",
        C::Fix(_) => "Fix",
        C::Midpoint { .. } => "Midpoint",
        C::Symmetric { .. } => "Symmetric",
        C::Concentric(..) => "Concentric",
        C::Distance { .. } => "Distance",
        C::HorizontalDistance { .. } => "Horizontal distance",
        C::VerticalDistance { .. } => "Vertical distance",
        C::Radius { .. } => "Radius",
        C::Diameter { .. } => "Diameter",
        C::Angle { .. } => "Angle",
    }
}

/// Dimension values drawn over the sketch. Click one to edit it, drag it to place the
/// dimension; the lines follow the value text, as in a drawing.
fn dimension_overlay(editor: &mut Editor, ctx: &egui::Context, commands: &mut Vec<Command>) {
    let camera = editor.camera;
    let window = editor.window_px;
    let Mode::Sketch(s) = &mut editor.mode else {
        return;
    };
    let ppp = ctx.pixels_per_point();
    let to_screen = |world: basset_math::Vec3| {
        camera
            .world_to_screen(world, window)
            .map(|px| egui::pos2(px[0] as f32 / ppp, px[1] as f32 / ppp))
    };
    let graphics = s.dimension_graphics();
    let mut open_edit: Option<basset_sketch::ConstraintId> = None;
    let mut dragged: Option<basset_sketch::ConstraintId> = None;
    for g in &graphics {
        let Some(pos) = to_screen(g.label) else {
            continue;
        };
        let size = egui::vec2(8.0 * g.text.len() as f32 + 12.0, 20.0);
        egui::Area::new(egui::Id::new(("dim", g.id)))
            .fixed_pos(pos - size * 0.5)
            .order(egui::Order::Foreground)
            .show(ctx, |ui| {
                let button = egui::Button::new(egui::RichText::new(&g.text).small())
                    .fill(egui::Color32::from_rgba_unmultiplied(20, 40, 70, 200))
                    .sense(egui::Sense::click_and_drag());
                let response = ui.add(button);
                if response.clicked() {
                    open_edit = Some(g.id);
                }
                if response.dragged() {
                    dragged = Some(g.id);
                    ui.ctx().set_cursor_icon(egui::CursorIcon::Grabbing);
                }
                response.context_menu(|ui| {
                    if ui.button("Delete dimension").clicked() {
                        commands.push(Command::SketchRemoveConstraint(g.id));
                        ui.close();
                    }
                });
            });
    }
    if let Some(cid) = dragged
        && let Some(pointer) = ctx.input(|i| i.pointer.interact_pos())
    {
        let px = [f64::from(pointer.x * ppp), f64::from(pointer.y * ppp)];
        let ray = camera.ray_from_screen(px, window);
        s.move_label(cid, &ray);
        commands.push(Command::SketchCommit);
    }
    if let Some(cid) = open_edit
        && let Some(c) = s.sketch.constraint(cid)
    {
        // A driven dimension opens showing its expression, not the number it worked out
        // to, so editing one never silently replaces the intent with its result.
        let text = match s.sketch.dimension_expr(cid) {
            Some(e) => e.to_string(),
            None => edit_text(c),
        };
        s.dim_edit = Some((cid, text));
    }
    if let Some((cid, mut text)) = s.dim_edit.clone() {
        let mut keep = true;
        // The box opens beside the dimension it edits, where the eye already is; a
        // dimension with nowhere to draw itself gets the top of the viewport instead.
        let beside = graphics
            .iter()
            .find(|g| g.id == cid)
            .and_then(|g| to_screen(g.label))
            .map(|p| p + egui::vec2(24.0, 24.0));
        let window = egui::Window::new("Dimension")
            .id(egui::Id::new("dim-edit"))
            .collapsible(false)
            .resizable(false);
        let window = match beside {
            Some(p) => window.fixed_pos(p),
            None => window.anchor(egui::Align2::CENTER_TOP, [0.0, 80.0]),
        };
        window.show(ctx, |ui| {
            let response = ui.text_edit_singleline(&mut text);
            response.request_focus();
            let submit = response.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
            ui.label(
                egui::RichText::new("A number, or an expression over the sketch's parameters")
                    .weak(),
            );
            ui.horizontal(|ui| {
                if ui.button("OK").clicked() || submit {
                    // Anything that is not a plain number is taken as an expression, so
                    // typing `bore / 2` here binds the dimension rather than failing.
                    let number = text.trim().parse::<f64>().is_ok();
                    match (number, s.sketch.constraint(cid)) {
                        (true, Some(c)) => {
                            if let Some(v) = parse_value(c, &text) {
                                commands.push(Command::SketchDimension(cid, v));
                            }
                        }
                        (false, Some(_)) => {
                            commands.push(Command::SketchBindDimension(cid, text.clone()));
                        }
                        (_, None) => {}
                    }
                    keep = false;
                }
                if ui.button("Cancel").clicked() || ui.input(|i| i.key_pressed(egui::Key::Escape)) {
                    keep = false;
                }
            });
        });
        s.dim_edit = if keep { Some((cid, text)) } else { None };
    }
}

fn error_popup(editor: &mut Editor, ctx: &egui::Context) {
    let Some(message) = editor.error.clone() else {
        return;
    };
    let mut open = true;
    egui::Window::new("Error")
        .collapsible(false)
        .resizable(false)
        .anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0])
        .show(ctx, |ui| {
            ui.label(message);
            if ui.button("OK").clicked() {
                open = false;
            }
        });
    if !open {
        editor.error = None;
    }
}

fn rename_popup(editor: &mut Editor, ctx: &egui::Context) {
    let Some((id, mut name)) = editor.rename.clone() else {
        return;
    };
    let mut done: Option<bool> = None;
    egui::Window::new("Rename")
        .collapsible(false)
        .resizable(false)
        .anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0])
        .show(ctx, |ui| {
            let response = ui.text_edit_singleline(&mut name);
            response.request_focus();
            if response.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter)) {
                done = Some(true);
            }
            ui.horizontal(|ui| {
                if ui.button("OK").clicked() {
                    done = Some(true);
                }
                if ui.button("Cancel").clicked() {
                    done = Some(false);
                }
            });
        });
    match done {
        Some(true) => {
            if let Err(e) = editor.doc.rename_feature(id, name.trim()) {
                editor.report_error(e);
            }
            editor.rename = None;
        }
        Some(false) => editor.rename = None,
        None => editor.rename = Some((id, name)),
    }
}

impl Editor {
    pub fn feature_name(&self, id: FeatureId) -> String {
        self.doc
            .timeline()
            .get(id)
            .map(|f| f.name.clone())
            .unwrap_or_else(|| format!("{id}"))
    }
}
