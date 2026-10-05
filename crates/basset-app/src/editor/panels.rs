//! Menus, toolbar, browser, timeline, status bar and popups.
//!
//! Panels only read editor state and queue commands; the editor mutates itself after
//! the panel closures return. That keeps borrow scopes short and lets any panel trigger
//! any command without threading `&mut Editor` through egui closures.

use basset_core::{BodyRef, Change, ComponentId, FeatureId, FeatureKind, FeatureStatus, PlaneRef};
use basset_viewport::ViewPreset;

use super::commands::{self, Command};
use super::files::MeshFormat;
use super::sketch_mode::{self, SketchTool, ToolGroup, edit_text, parse_value};
use super::symbols::Symbol;
use super::tools::{self, ToolKind};
use super::{DisplayMode, Editor, Mode, SelectMode, Workspace};

/// The blue the viewport draws under-constrained geometry in, so the words that explain it
/// match what the user is looking at.
const LOOSE_LABEL: egui::Color32 = egui::Color32::from_rgb(140, 184, 255);
/// Amber for a feature that built but warns about its result, matching the timeline chip.
const WARNING_LABEL: egui::Color32 = super::theme::WARNING;
/// The orange the viewport draws a redundant constraint's badge in, so the row that
/// names it matches the mark on the drawing.
const REDUNDANT_LABEL: egui::Color32 = egui::Color32::from_rgb(255, 160, 80);
/// The green and red of a comparison, matching what the viewport paints new and gone
/// geometry in (see `scene::DIFF_ADDED` and `scene::DIFF_REMOVED`), so the mark under a
/// timeline chip and the colour on the body say the same thing. Modified is the mix: a
/// feature that is in both versions and not the same.
const DIFF_ADDED_LABEL: egui::Color32 = egui::Color32::from_rgb(110, 210, 120);
const DIFF_REMOVED_LABEL: egui::Color32 = egui::Color32::from_rgb(235, 100, 90);
const DIFF_MODIFIED_LABEL: egui::Color32 = egui::Color32::from_rgb(230, 190, 80);

fn change_color(change: Change) -> egui::Color32 {
    match change {
        Change::Added => DIFF_ADDED_LABEL,
        Change::Removed => DIFF_REMOVED_LABEL,
        Change::Modified => DIFF_MODIFIED_LABEL,
    }
}

pub fn show(editor: &mut Editor, ui: &mut egui::Ui) {
    let mut commands: Vec<Command> = Vec::new();
    editor.refresh_cache();
    // The hint belongs to the drag happening now; a stale one would leave a number
    // hanging over geometry nobody is touching.
    editor.snap_hint = None;
    // The in-canvas render catches up with the view, and its picture goes down first,
    // under every panel, over the whole window the camera's pixels are measured in.
    super::render::poll(editor, ui.ctx());
    super::render::paint_canvas(editor, ui);

    let simulation = editor.workspace == Workspace::Simulation;
    let rendering = editor.workspace == Workspace::Render;
    egui::Panel::top("menu")
        .frame(super::theme::menu_frame())
        .show_separator_line(false)
        .show(ui, |ui| {
            ui.horizontal(|ui| {
                workspace_tabs(editor, ui, &mut commands);
                ui.add_space(6.0);
                menu_bar(editor, ui, &mut commands);
            });
        });
    egui::Panel::top("ribbon")
        .frame(super::theme::ribbon_frame())
        .show(ui, |ui| {
            if simulation {
                super::simulate::toolbar(editor, ui, &mut commands);
            } else if rendering {
                super::render::toolbar(editor, ui, &mut commands);
            } else {
                toolbar(editor, ui, &mut commands);
            }
        });
    egui::Panel::bottom("status")
        .frame(super::theme::status_frame())
        .show_separator_line(false)
        .show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.label(&editor.status);
                warning_summary(editor, ui, &mut commands);
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    ui.label(egui::RichText::new(editor.selection.summary()).weak());
                    git_chip(editor, ui, &mut commands);
                    if !editor.is_sketching() && editor.workspace.models() {
                        ui.label(
                            egui::RichText::new("Right-drag orbit · middle-drag pan · wheel zoom")
                                .small()
                                .weak(),
                        );
                    }
                });
            });
        });
    // The timeline is the Design workspace's: a study has no history to walk, and a
    // cursor dragged while results were up would only make them stale. The Render
    // workspace has the rendering gallery in its place, once there is something in it.
    if editor.workspace.models() {
        egui::Panel::bottom("timeline")
            .frame(super::theme::timeline_frame())
            .default_size(64.0)
            .show(ui, |ui| timeline(editor, ui, &mut commands));
    } else if rendering && !editor.render.gallery.is_empty() {
        egui::Panel::bottom("gallery")
            .frame(super::theme::timeline_frame())
            .default_size(130.0)
            .show(ui, |ui| super::render::gallery(editor, ui, &mut commands));
    }
    egui::Panel::left("browser")
        .frame(super::theme::side_frame())
        .default_size(230.0)
        .show(ui, |ui| {
            if simulation {
                super::simulate::study_tree(editor, ui);
            } else if rendering {
                super::render::tree(editor, ui, &mut commands);
            } else {
                browser(editor, ui, &mut commands);
            }
        });
    if rendering {
        egui::Panel::right("render")
            .frame(super::theme::side_frame())
            .default_size(290.0)
            .show(ui, |ui| super::render::panel(editor, ui, &mut commands));
    } else if simulation {
        egui::Panel::right("study")
            .frame(super::theme::side_frame())
            .default_size(250.0)
            .show(ui, |ui| {
                super::simulate::study_panel(editor, ui, &mut commands)
            });
    } else if editor.is_sketching() {
        egui::Panel::right("sketch-palette")
            .frame(super::theme::side_frame())
            .default_size(210.0)
            .show(ui, |ui| sketch_palette(editor, ui, &mut commands));
    } else if editor.show_project && editor.project.is_some() {
        egui::Panel::right("project")
            .frame(super::theme::side_frame())
            .default_size(250.0)
            .show(ui, |ui| project_panel(editor, ui, &mut commands));
    }

    let free = ui.available_rect_before_wrap();
    let ctx = ui.ctx().clone();
    super::viewcube::show(editor, &ctx, free);
    compare_banner(editor, &ctx, free, &mut commands);
    sketch_operation_dialog(editor, &ctx, free, &mut commands);
    tools::dialog(editor, &ctx);
    super::gizmo::interact(editor, &ctx);
    super::study_marks::interact(editor, &ctx);
    if let Some(hint) = &editor.snap_hint {
        super::snap::paint(&ctx, &editor.camera, editor.window_px, hint);
    }
    entry_overlay(editor, &ctx, &mut commands);
    dimension_overlay(editor, &ctx, &mut commands);
    measure_overlay(editor, &ctx);
    super::study_marks::overlay(editor, &ctx);
    constraint_overlay(editor, &ctx, &mut commands);
    shortcut_overlay(editor, &ctx);
    super::render::windows(editor, &ctx, &mut commands);
    command_palette(editor, &ctx, &mut commands);
    error_popup(editor, &ctx);
    rename_popup(editor, &ctx);
    commit_popup(editor, &ctx, &mut commands);

    for c in commands {
        run(editor, c);
    }
}

pub(super) fn run(editor: &mut Editor, c: Command) {
    // One gate for every way a modelling command can arrive — key, menu, palette — so
    // the Simulation and Render workspaces cannot be modelled in by a path the toolbar
    // forgot.
    if !editor.workspace.models() && c.is_modelling() {
        editor.set_status(editor.workspace.refusal());
        editor.request_repaint();
        return;
    }
    match c {
        Command::Tool(kind) => tools::start_tool(editor, kind),
        Command::Measure(on) => {
            if on {
                super::measure::start(editor)
            } else {
                super::measure::stop(editor)
            }
        }
        Command::ToggleMeasure => {
            if editor.measure.is_some() {
                super::measure::stop(editor)
            } else {
                super::measure::start(editor)
            }
        }
        Command::Workspace(workspace) => editor.set_workspace(workspace),
        Command::ToggleWorkspace => editor.toggle_workspace(),
        Command::RunStudy => super::simulate::run(editor),
        Command::StopStudy => super::simulate::stop_run(editor),
        Command::ExportVtk => super::simulate::export_vtk(editor),
        Command::Brush(a) => super::render::arm(editor, a.map(|a| *a)),
        Command::Paint(target, a) => super::render::paint(editor, target, a.as_deref()),
        Command::ClearFaceAppearances(body) => {
            if editor.doc.in_transaction() {
                editor.set_status("Finish or cancel the sketch or tool first");
            } else {
                editor.doc.clear_face_appearances(body);
            }
        }
        Command::EditAppearance(name, a) => {
            let renamed = a.name.trim() != name;
            let new_name = a.name.trim().to_owned();
            match editor.doc.edit_appearance(&name, *a) {
                Ok(()) if renamed => editor.render.editing = Some(new_name),
                Ok(()) => {}
                Err(e) => editor.report_error(e),
            }
        }
        Command::RemoveAppearance(name) => {
            if editor.doc.remove_appearance(&name) {
                editor.set_status(format!("Removed {name} from the design"));
            }
        }
        Command::SetScene(s) => editor.doc.set_scene(*s),
        Command::ToggleInCanvas => super::render::toggle_in_canvas(editor),
        Command::StartRender(settings) => super::render::start_final(editor, settings),
        Command::SaveRender(i) => super::render::save_final(editor, i),
        Command::RemoveRender(i) => super::render::remove_final(editor, i),
        Command::New => editor.new_document(),
        Command::Open => editor.open(),
        Command::Save(as_new) => editor.save(as_new),
        Command::ExportStl => editor.export_stl(),
        Command::Export3mf => editor.export_3mf(),
        Command::ExportBodies(bodies, format) => {
            let name = match bodies.as_slice() {
                [one] => editor.body_name(*one),
                _ => editor.doc.name.clone(),
            };
            editor.export_bodies(&bodies, format, &name);
        }
        Command::ExportComponent(id, format) => editor.export_component(id, format),
        Command::Quit => editor.quit(),
        Command::Compare(Some(spec)) => editor.compare_with(&spec),
        Command::Compare(None) => editor.stop_compare(),
        Command::ToggleCompare => editor.toggle_compare(),
        Command::RefreshProject => editor.refresh_project(),
        Command::Commit => editor.begin_commit(),
        Command::CommitWith(message, paths) => {
            editor.commit_box = None;
            editor.commit_files(&paths, &message);
        }
        Command::OpenProject => editor.open_project(),
        Command::NewProject => editor.new_project(),
        Command::CloseProject => editor.close_project(),
        Command::ToggleProjectPanel => {
            if editor.project.is_none() {
                editor.report_error("open a project first: File \u{203a} Open project…");
            } else {
                editor.show_project = !editor.show_project;
            }
        }
        Command::OpenPart(rel) => editor.open_part(&rel),
        Command::OpenVersion(rev, rel) => editor.open_version(&rev, &rel),
        Command::PickCommit(hash) => {
            if let Some(project) = &mut editor.project {
                project.picked = hash;
            }
        }
        Command::Undo => editor.undo(),
        Command::Redo => editor.redo(),
        Command::Fit => editor.zoom_to_fit(),
        Command::View(p) => editor.look_from(p),
        Command::RollView(clockwise) => editor.roll_view(clockwise),
        Command::ToggleProjection => editor.toggle_projection(),
        Command::Display(m) => editor.set_display_mode(m),
        Command::CycleDisplay => editor.cycle_display_mode(),
        // Escape dismisses the topmost thing first. The overlay is drawn over
        // everything, and putting it away should not also put down the tool under it.
        Command::Cancel => {
            if editor.show_shortcuts {
                editor.show_shortcuts = false;
            } else {
                editor.cancel();
            }
        }
        Command::Confirm => editor.confirm(),
        Command::DeleteSelected => editor.delete_selected(),
        Command::FocusNextEntry => {
            if let Mode::Sketch(s) = &mut editor.mode {
                s.focus_next_entry();
            }
        }
        Command::ToggleShortcuts => editor.show_shortcuts = !editor.show_shortcuts,
        Command::OpenPalette => {
            editor.palette = Some(commands::Palette {
                just_opened: true,
                ..Default::default()
            })
        }
        Command::ToggleGrid => editor.show_grid = !editor.show_grid,
        Command::ToggleSnap => editor.set_snapping(!editor.snapping.to_grid),
        Command::ToggleOrigin => editor.show_origin = !editor.show_origin,
        Command::ToggleAxes => editor.show_axes = !editor.show_axes,
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
        Command::BeginBodyRename(b) => editor.begin_body_rename(b),
        Command::RenameBody(b, name) => editor.rename_body(b, &name),
        Command::ComponentsFromBodies(bodies) => editor.components_from_bodies(&bodies),
        Command::ComponentsFromSelectedBodies => {
            let bodies = editor.selection.bodies.clone();
            editor.components_from_bodies(&bodies);
        }
        Command::FinishSketch(keep) => sketch_mode::finish(editor, keep),
        // Arming a constraint tool applies it straight away to a selection that already
        // suits it, so selecting first and selecting after are the same command.
        Command::SketchTool(SketchTool::Constrain(kind)) => {
            if let Mode::Sketch(s) = &mut editor.mode {
                match s.begin_constraint(kind) {
                    Ok(()) => editor.commit_sketch(),
                    Err(e) => editor.report_error(e),
                }
            }
        }
        Command::SketchPick(mode) => {
            if let Mode::Sketch(s) = &mut editor.mode {
                // Narrowing the filter drops what it no longer covers, so the user is
                // never left acting on things the new mode gives them no way to see.
                s.set_pick(mode);
            }
        }
        // A group key picks the variant its toolbar button is showing, which is what
        // makes the key and the button the same button.
        Command::SketchGroup(group) => {
            if let Mode::Sketch(s) = &mut editor.mode {
                let variant = s.variant_of(group);
                s.set_tool(variant);
                if s.take_dirty() {
                    editor.commit_sketch();
                }
            }
        }
        Command::SketchExtrudeRegion => sketch_mode::extrude_region(editor),
        Command::SketchTool(t) => {
            if let Mode::Sketch(s) = &mut editor.mode {
                // Picking another tool cancels a move or a pattern, and that revert has
                // to reach the document or the feature keeps the copies it just took back.
                s.set_tool(t);
                if s.take_dirty() {
                    editor.commit_sketch();
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
                if s.begin_pattern() {
                    editor.commit_sketch();
                } else {
                    editor.set_status("Select the sketch geometry to repeat, then Pattern");
                }
            }
        }
        Command::SketchPatternUpdate => {
            if let Mode::Sketch(s) = &mut editor.mode {
                s.update_pattern();
                editor.commit_sketch();
            }
        }
        Command::SketchPatternFinish(keep) => {
            if let Mode::Sketch(s) = &mut editor.mode {
                let created = s.finish_pattern(keep);
                editor.commit_sketch();
                match created {
                    Some(n) => editor.set_status(format!("Pattern added {n} entities")),
                    None => editor.set_status("Pattern cancelled"),
                }
            }
        }
        Command::SketchOffset => {
            if let Mode::Sketch(s) = &mut editor.mode {
                if s.begin_offset() {
                    // The preview is real geometry in the feature by now, so it has to
                    // reach the document for anything downstream to see it.
                    editor.commit_sketch();
                } else {
                    // Somebody halfway through a move, told to select something first,
                    // learns nothing about what to do next: say which it is.
                    let why =
                        super::busy(s).unwrap_or("Select the path or loop to offset first".into());
                    editor.set_status(why);
                }
            }
        }
        Command::SketchOffsetUpdate => {
            if let Mode::Sketch(s) = &mut editor.mode {
                s.update_offset();
                editor.commit_sketch();
            }
        }
        Command::SketchOffsetFinish(keep) => {
            if let Mode::Sketch(s) = &mut editor.mode {
                let made = s.finish_offset(keep);
                editor.commit_sketch();
                // Keeping an offset that made nothing is a cancel, and saying so is what
                // tells the user the button was pressed at all.
                editor.set_status(match (keep, made) {
                    (_, Some(n)) => format!("Offset added {n} curves"),
                    (true, None) => "Offset cancelled: there was nothing it could make".into(),
                    (false, None) => "Offset cancelled".to_string(),
                });
            }
        }
        Command::SketchFilletUpdate => {
            if let Mode::Sketch(s) = &mut editor.mode {
                s.update_fillet();
                editor.commit_sketch();
            }
        }
        Command::SketchFilletFinish(keep) => {
            if let Mode::Sketch(s) = &mut editor.mode {
                let kept = s.finish_fillet(keep);
                editor.commit_sketch();
                editor.set_status(match (keep, kept) {
                    (_, true) => "Corner rounded".to_string(),
                    (true, false) => "Fillet cancelled: that radius does not fit".into(),
                    (false, false) => "Fillet cancelled".into(),
                });
            }
        }
        Command::SetParameter(name, expression) => {
            editor.set_document_parameter(&name, &expression);
        }
        Command::AddParameter(name, expression) => {
            editor.add_document_parameter(&name, &expression);
        }
        Command::RenameParameter(from, to) => editor.rename_document_parameter(&from, &to),
        Command::RemoveParameter(name) => editor.remove_document_parameter(&name),
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
        Command::SketchMoveBegin => {
            if let Mode::Sketch(s) = &mut editor.mode
                && !s.begin_move()
            {
                let why =
                    super::busy(s).unwrap_or("Select the sketch geometry to move first".into());
                editor.set_status(why);
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

/// The workspace tabs, Fusion-style, at the left of the menu bar: the active one is
/// drawn as the selected tab and the other is a click away. They share the menu bar's
/// row rather than taking one of their own because every row the top panel grows is a
/// row off the viewport, and they sit before the menus because they are not commands
/// among commands: they decide which commands there are.
fn workspace_tabs(editor: &Editor, ui: &mut egui::Ui, commands: &mut Vec<Command>) {
    let titles: Vec<&str> = Workspace::ALL.iter().map(|w| w.title()).collect();
    let chosen = Workspace::ALL
        .iter()
        .position(|w| *w == editor.workspace)
        .unwrap_or(0);
    let responses = super::theme::segmented(ui, &titles, chosen);
    for (workspace, response) in Workspace::ALL.into_iter().zip(responses) {
        if response
            .on_hover_text(match workspace {
                Workspace::Design => "Model the part: sketches, features, the timeline",
                Workspace::Simulation => "Study one body under load; the model is not changed",
                Workspace::Render => {
                    "Give bodies appearances, light the scene, and render pictures of it"
                }
            })
            .clicked()
            && editor.workspace != workspace
        {
            commands.push(Command::Workspace(workspace));
        }
    }
}

fn menu_bar(editor: &Editor, ui: &mut egui::Ui, commands: &mut Vec<Command>) {
    egui::MenuBar::new().ui(ui, |ui| {
        ui.menu_button("File", |ui| {
            if ui
                .button(format!("New{}", commands::hint("file.new")))
                .clicked()
            {
                commands.push(Command::New);
            }
            if ui
                .button(format!("Open…{}", commands::hint("file.open")))
                .clicked()
            {
                commands.push(Command::Open);
            }
            ui.separator();
            if ui
                .button(format!("New project…{}", commands::hint("project.new")))
                .on_hover_text("Pick a folder and make it a git repository of parts")
                .clicked()
            {
                commands.push(Command::NewProject);
            }
            if ui
                .button(format!("Open project…{}", commands::hint("project.open")))
                .on_hover_text("Pick a folder inside a git repository")
                .clicked()
            {
                commands.push(Command::OpenProject);
            }
            ui.separator();
            if ui
                .button(format!("Save{}", commands::hint("file.save")))
                .clicked()
            {
                commands.push(Command::Save(false));
            }
            if ui
                .button(format!("Save As…{}", commands::hint("file.save_as")))
                .clicked()
            {
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
                .add_enabled(
                    editor.doc.can_undo(),
                    egui::Button::new(format!("Undo{}", commands::hint("edit.undo"))),
                )
                .clicked()
            {
                commands.push(Command::Undo);
            }
            if ui
                .add_enabled(
                    editor.doc.can_redo(),
                    egui::Button::new(format!("Redo{}", commands::hint("edit.redo"))),
                )
                .clicked()
            {
                commands.push(Command::Redo);
            }
        });
        git_menu(editor, ui, commands);
        ui.menu_button("View", |ui| {
            if ui
                .button(format!("Fit{}", commands::hint("view.fit")))
                .clicked()
            {
                commands.push(Command::Fit);
            }
            for (name, id, p) in [
                ("Isometric", "view.isometric", ViewPreset::Isometric),
                ("Top", "view.top", ViewPreset::Top),
                ("Front", "view.front", ViewPreset::Front),
                ("Right", "view.right", ViewPreset::Right),
                ("Bottom", "view.bottom", ViewPreset::Bottom),
                ("Back", "view.back", ViewPreset::Back),
                ("Left", "view.left", ViewPreset::Left),
            ] {
                if ui.button(format!("{name}{}", commands::hint(id))).clicked() {
                    commands.push(Command::View(p));
                }
            }
            ui.separator();
            if ui
                .button(format!(
                    "Toggle orthographic{}",
                    commands::hint("view.projection")
                ))
                .clicked()
            {
                commands.push(Command::ToggleProjection);
            }
            ui.separator();
            ui.label(
                egui::RichText::new(format!("Display{}", commands::hint("view.display"))).weak(),
            );
            for mode in DisplayMode::ALL {
                if ui
                    .selectable_label(editor.display == mode, mode.title())
                    .clicked()
                {
                    commands.push(Command::Display(mode));
                }
            }
            ui.separator();
            // Selectable labels rather than checkboxes: the menu only has `&Editor`, and
            // a checkbox would need somewhere to write the new value before the command
            // that actually applies it runs.
            if ui
                .selectable_label(
                    editor.show_grid,
                    format!("Show grid{}", commands::hint("view.grid")),
                )
                .clicked()
            {
                commands.push(Command::ToggleGrid);
            }
            // The master switch, reachable without opening a sketch: the modelling
            // handles snap too, and a user who wants them free needs somewhere to say so
            // that is not the sketch palette. Shift remains the way to free one drag.
            if ui
                .selectable_label(
                    editor.snapping.to_grid,
                    format!(
                        "Snap to grid (shift to override){}",
                        commands::hint("view.snap")
                    ),
                )
                .clicked()
            {
                commands.push(Command::ToggleSnap);
            }
            if ui
                .selectable_label(
                    editor.show_origin,
                    format!(
                        "Show origin planes and axes{}",
                        commands::hint("view.origin")
                    ),
                )
                .clicked()
            {
                commands.push(Command::ToggleOrigin);
            }
            if ui
                .selectable_label(
                    editor.show_axes,
                    format!("Show grid axes{}", commands::hint("view.axes")),
                )
                .clicked()
            {
                commands.push(Command::ToggleAxes);
            }
        });
        ui.menu_button("Create", |ui| {
            tool_menu(
                ui,
                "create-menu",
                &[
                    ToolKind::Sketch,
                    ToolKind::Extrude,
                    ToolKind::Revolve,
                    ToolKind::Sweep,
                    ToolKind::Loft,
                    ToolKind::OffsetPlane,
                    ToolKind::AngledPlane,
                    ToolKind::Component,
                ],
                commands,
            );
            let can = editor.tool.is_none() && !editor.selection.bodies.is_empty();
            if ui
                .add_enabled(can, egui::Button::new("Create Components from Bodies"))
                .on_disabled_hover_text("Select one or more bodies first")
                .clicked()
            {
                commands.push(Command::ComponentsFromSelectedBodies);
                ui.close();
            }
        });
        ui.menu_button("Modify", |ui| {
            tool_menu(
                ui,
                "modify-menu",
                &[
                    ToolKind::Fillet,
                    ToolKind::Chamfer,
                    ToolKind::Thread,
                    ToolKind::Combine,
                    ToolKind::Move,
                ],
                commands,
            );
            let label = format!("Simulation workspace{}", commands::hint("modify.simulate"));
            if tool_button(
                ui,
                "modify-menu",
                AnyTool::Simulate,
                Some(&label),
                false,
                true,
            )
            .clicked()
            {
                commands.push(Command::Workspace(Workspace::Simulation));
                ui.close();
            }
            if ui
                .button(format!(
                    "Render workspace…{}",
                    commands::hint("render.workspace")
                ))
                .on_hover_text("Appearances, the scene, and rendered pictures")
                .clicked()
            {
                commands.push(Command::Workspace(Workspace::Render));
                ui.close();
            }
        });
        // The two ways to find a command without already knowing where it is. They are
        // in a menu as well as on keys, because a shortcut overlay only reachable by a
        // shortcut helps whoever needed it least.
        ui.menu_button("Help", |ui| {
            if ui
                .button(format!(
                    "Keyboard shortcuts{}",
                    commands::hint("help.shortcuts")
                ))
                .clicked()
            {
                commands.push(Command::ToggleShortcuts);
            }
            if ui
                .button(format!("Command palette{}", commands::hint("help.palette")))
                .clicked()
            {
                commands.push(Command::OpenPalette);
            }
        });
    });
}

fn toolbar(editor: &Editor, ui: &mut egui::Ui, commands: &mut Vec<Command>) {
    if let Mode::Sketch(s) = &editor.mode {
        sketch_toolbar(s, ui, commands);
        return;
    }
    let busy = editor.tool.is_some();
    ui.horizontal_wrapped(|ui| {
        let groups: [(&str, &[(ToolKind, &str)]); 3] = [
            (
                "Create",
                &[
                    (ToolKind::Sketch, "Sketch"),
                    (ToolKind::Extrude, "Extrude"),
                    (ToolKind::Revolve, "Revolve"),
                    (ToolKind::Sweep, "Sweep"),
                    (ToolKind::Loft, "Loft"),
                ],
            ),
            (
                "Modify",
                &[
                    (ToolKind::Fillet, "Fillet"),
                    (ToolKind::Chamfer, "Chamfer"),
                    (ToolKind::Thread, "Thread"),
                    (ToolKind::Combine, "Combine"),
                    (ToolKind::Move, "Move"),
                ],
            ),
            (
                "Construct",
                &[
                    (ToolKind::OffsetPlane, "Offset Plane"),
                    (ToolKind::AngledPlane, "Angled Plane"),
                ],
            ),
        ];
        for (caption, tools) in groups {
            let labels: Vec<Option<&str>> = tools.iter().map(|(_, l)| Some(*l)).collect();
            ribbon_group(ui, caption, &labels, |ui| {
                for (kind, label) in tools {
                    // Icon and label are one button, so the symbol is as clickable as
                    // the word under it.
                    if shaped_tool_button(
                        ui,
                        "toolbar",
                        AnyTool::Model(*kind),
                        Some(label),
                        false,
                        !busy,
                        ButtonShape::Stacked,
                    )
                    .on_hover_text(format!("{}{}", kind.title(), commands::tool_hint(*kind)))
                    .clicked()
                    {
                        commands.push(Command::Tool(*kind));
                    }
                }
            });
            ui.separator();
        }
        let measuring = editor.measure.is_some();
        ribbon_group(
            ui,
            "Inspect",
            &[Some("Measure"), Some("Simulate"), Some("Fit")],
            |ui| {
                if shaped_tool_button(
                    ui,
                    "toolbar",
                    AnyTool::Measure,
                    Some("Measure"),
                    measuring,
                    !busy,
                    ButtonShape::Stacked,
                )
                .on_hover_text("Measure (no change to the model)")
                .clicked()
                {
                    commands.push(Command::Measure(!measuring));
                }
                // The way into the other workspace, beside the tool it most resembles:
                // it studies the model and changes nothing.
                if shaped_tool_button(
                    ui,
                    "toolbar",
                    AnyTool::Simulate,
                    Some("Simulate"),
                    false,
                    !busy,
                    ButtonShape::Stacked,
                )
                .on_hover_text(format!(
                    "Simulation workspace: a linear elastic study of one body (no change to the model){}",
                    commands::hint("modify.simulate")
                ))
                .clicked()
                {
                    commands.push(Command::Workspace(Workspace::Simulation));
                }
                if shaped_tool_button(
                    ui,
                    "toolbar",
                    AnyTool::Fit,
                    Some("Fit"),
                    false,
                    true,
                    ButtonShape::Stacked,
                )
                .on_hover_text(format!(
                    "Fit the model in the view{}",
                    commands::hint("view.fit")
                ))
                .clicked()
                {
                    commands.push(Command::Fit);
                }
            },
        );
        ui.separator();
        let names: Vec<&str> = SelectMode::ALL.iter().map(|m| m.name()).collect();
        let chosen = SelectMode::ALL
            .iter()
            .position(|m| *m == editor.select_mode)
            .unwrap_or(0);
        if let Some(i) = ribbon_segmented(ui, "Select", &names, chosen, |i| {
            format!("{} (press {})", names[i], i + 1)
        }) {
            commands.push(Command::SelectMode(SelectMode::ALL[i]));
        }
    });
}

/// Height of a ribbon button: symbol over name.
const RIBBON_BUTTON: f32 = ICON + 15.0;

/// A captioned group of the ribbon, measured before it is laid out so a narrow window
/// wraps the ribbon between groups rather than through one. `labels` are the names of
/// the stacked buttons `add` will place, which is all the measuring needs.
fn ribbon_group(
    ui: &mut egui::Ui,
    caption: &str,
    labels: &[Option<&str>],
    add: impl FnOnce(&mut egui::Ui),
) {
    const GAP: f32 = 2.0;
    let buttons: f32 = labels
        .iter()
        .map(|l| tool_button_size(ui, *l, ButtonShape::Stacked).x + GAP)
        .sum::<f32>()
        - GAP;
    ribbon_column(ui, caption, buttons, |ui| {
        ui.horizontal(|ui| {
            ui.spacing_mut().item_spacing.x = GAP;
            add(ui);
        });
    });
}

/// A segmented choice in the ribbon, captioned like a group of tools and centred on the
/// buttons' height. Returns the segment clicked, if one was.
fn ribbon_segmented(
    ui: &mut egui::Ui,
    caption: &str,
    labels: &[&str],
    chosen: usize,
    hover: impl Fn(usize) -> String,
) -> Option<usize> {
    let width = super::theme::segmented_width(ui, labels);
    let mut clicked = None;
    ribbon_column(ui, caption, width, |ui| {
        ui.add_space((RIBBON_BUTTON - super::theme::SEGMENT_HEIGHT) * 0.5);
        let responses = super::theme::segmented(ui, labels, chosen);
        ui.add_space((RIBBON_BUTTON - super::theme::SEGMENT_HEIGHT) * 0.5);
        for (i, r) in responses.into_iter().enumerate() {
            if r.on_hover_text(hover(i)).clicked() && i != chosen {
                clicked = Some(i);
            }
        }
    });
    clicked
}

/// One column of the ribbon: content of a known width, the caption centred under it.
fn ribbon_column(ui: &mut egui::Ui, caption: &str, width: f32, add: impl FnOnce(&mut egui::Ui)) {
    let font = egui::FontId::proportional(10.0);
    let caption_text = caption.to_uppercase();
    let caption_width = ui
        .painter()
        .layout_no_wrap(caption_text.clone(), font.clone(), egui::Color32::WHITE)
        .size()
        .x
        + caption.len() as f32 * 0.8;
    let width = width.max(caption_width);
    let height = RIBBON_BUTTON + 1.0 + 13.0;
    ui.allocate_ui_with_layout(
        egui::vec2(width, height),
        egui::Layout::top_down(egui::Align::Center),
        |ui| {
            ui.spacing_mut().item_spacing.y = 1.0;
            add(ui);
            ui.label(
                egui::RichText::new(caption_text)
                    .font(font)
                    .extra_letter_spacing(0.8)
                    .color(super::theme::TEXT_WEAK),
            );
        },
    );
}

/// One menu's worth of modelling tools, each an icon and its name in one button.
///
/// The icon is part of the button rather than a picture beside it: a symbol that reacts
/// to the pointer and then ignores the click is the menu saying one thing and meaning
/// another, which is exactly the bug the sketch variant menus had.
fn tool_menu(ui: &mut egui::Ui, salt: &str, kinds: &[ToolKind], commands: &mut Vec<Command>) {
    for kind in kinds {
        let label = format!("{}{}", kind.title(), commands::tool_hint(*kind));
        if tool_button(ui, salt, *kind, Some(&label), false, true).clicked() {
            commands.push(Command::Tool(*kind));
            ui.close();
        }
    }
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
            let response = tool_icon(ui, "toolbar", variant, selected).on_hover_ui(|ui| {
                ui.label(format!(
                    "{}{}",
                    variant.name(),
                    commands::sketch_tool_hint(group, variant)
                ));
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
                                // The icon takes the click as readily as the name does.
                                // It looks like a button and reacts like one on hover, so
                                // a click on it that did nothing was the row saying one
                                // thing and meaning another.
                                let icon = tool_icon(ui, "menu", *tool, s.tool == *tool);
                                let label = ui.selectable_label(
                                    s.tool == *tool,
                                    format!(
                                        "{}{}",
                                        tool.name(),
                                        commands::sketch_tool_hint(group, *tool)
                                    ),
                                );
                                if icon.clicked() || label.clicked() {
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
        // What a click or a box drag may take. A sketch puts a corner, the curves that
        // meet there and the region beyond them within a few pixels of each other, and
        // the filter is how the user says which of them they mean — the same answer, and
        // the same keys, as the model-mode filter.
        let names: Vec<&str> = sketch_mode::SketchPick::ALL
            .iter()
            .map(|m| m.name())
            .collect();
        let chosen = sketch_mode::SketchPick::ALL
            .iter()
            .position(|m| *m == s.pick)
            .unwrap_or(0);
        ui.add_space(2.0);
        ui.allocate_ui_with_layout(
            egui::vec2(super::theme::segmented_width(ui, &names), ICON),
            egui::Layout::top_down(egui::Align::Min),
            |ui| {
                ui.add_space((ICON - super::theme::SEGMENT_HEIGHT) * 0.5);
                let responses = super::theme::segmented(ui, &names, chosen);
                for (i, (mode, response)) in sketch_mode::SketchPick::ALL
                    .iter()
                    .zip(responses)
                    .enumerate()
                {
                    if response
                        .on_hover_text(format!("{} ({})", mode.hint(), i + 1))
                        .clicked()
                    {
                        commands.push(Command::SketchPick(*mode));
                    }
                }
            },
        );
        ui.separator();
        // Constraints are tools, like the shapes to their left: clicking one arms it and
        // the picks that follow are what it acts on. They are always available, because
        // a button that greys out until you have guessed what it wants teaches nobody
        // what it does. Picking the geometry first still works — an armed tool applies
        // at once to a selection that already suits it.
        let armed = s.armed_constraint();
        for kind in sketch_mode::ConstraintKind::ALL {
            let ready = !s.constraints_for(kind, &s.selected).is_empty();
            let response =
                constraint_button(ui, kind, armed == Some(kind), ready).on_hover_ui(|ui| {
                    ui.strong(format!(
                        "{}{}",
                        kind.name(),
                        commands::constraint_hint(kind)
                    ));
                    ui.label(kind.hint());
                    if ready {
                        ui.colored_label(LOOSE_LABEL, "The selection is ready for this");
                    }
                });
            if response.clicked() {
                commands.push(Command::SketchTool(SketchTool::Constrain(kind)));
            }
        }
        ui.separator();
        // One button that reads its context, the way `X` does: with geometry selected it
        // converts that geometry, and with nothing selected it arms the mode so the next
        // shape is drawn as construction. Its lit state says which of those is true of
        // what is in front of the user right now, and the palette carries the standing
        // "the next shape is construction" note so an armed mode is never only implied.
        let selection = !s.selected.is_empty();
        let lit = if selection {
            s.selection_is_construction()
        } else {
            s.construction
        };
        // The keys live in the tooltips rather than in the names, here and along the
        // rest of this row. The row already wraps on an ordinary window, and every line
        // it gains is a line taken off the palette beside it — which is where the
        // degrees of freedom and the redundant constraints are reported, and they are
        // worth more than a key printed twice: the overlay and the palette have it.
        if ui
            .add(action_button(Symbol::Construction, "Construction", lit))
            .on_hover_text(format!(
                "{}{}",
                if selection {
                    "Make the selection construction geometry, or ordinary geometry again"
                } else {
                    "Draw the next shape as construction (reference) geometry"
                },
                commands::hint("sketch.construction")
            ))
            .clicked()
        {
            commands.push(Command::SketchConstruction);
        }
        if ui
            .add_enabled(
                !s.selected.is_empty(),
                action_button(Symbol::Delete, "Delete", false),
            )
            .on_hover_text(format!(
                "Delete the selection{}",
                commands::hint("edit.delete")
            ))
            .clicked()
        {
            commands.push(Command::SketchDelete);
        }
        // Move is the manipulator's only entry point: without a button the arrows in the
        // viewport exist only for whoever already knows to press M.
        if ui
            .add_enabled(
                !s.selected.is_empty() && !s.modal(),
                action_button(Symbol::SketchMove, "Move", false),
            )
            .on_hover_text(format!(
                "Move the selection: drag the arrows and ring, or type offsets{}",
                commands::hint("sketch.move")
            ))
            .on_disabled_hover_text("Select the geometry to move first")
            .clicked()
        {
            commands.push(Command::SketchMoveBegin);
        }
        // A pattern is a tool with a preview, not a button that silently drops copies
        // into the sketch: the copies appear at once and the palette holds the numbers
        // and the OK, so what the numbers mean is visible while they are being changed.
        if ui
            .add_enabled(
                !s.selected.is_empty() && !s.modal(),
                action_button(Symbol::Pattern, "Pattern", false),
            )
            .on_hover_text("Repeat the selection; the copies preview as you set them up")
            .on_disabled_hover_text("Select the geometry to repeat first")
            .clicked()
        {
            commands.push(Command::SketchPattern);
        }
        // Offset, like pattern, is a tool with a preview: which side it went and what it
        // did to the corners are things to look at, not to guess from a number.
        if ui
            .add_enabled(
                !s.selected.is_empty() && !s.modal(),
                action_button(Symbol::Offset, "Offset", false),
            )
            .on_hover_text(format!(
                "Draw a chain of curves alongside the selection at a fixed distance{}",
                commands::hint("sketch.offset")
            ))
            .on_disabled_hover_text("Select the path or loop to offset first")
            .clicked()
        {
            commands.push(Command::SketchOffset);
        }
        ui.separator();
        // The way out that keeps the work is the one primary button on the row.
        if ui.add(finish_button()).clicked() {
            commands.push(Command::FinishSketch(true));
        }
        if ui
            .add(egui::Button::new("✖ Cancel Sketch").frame_when_inactive(false))
            .clicked()
        {
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

/// A constraint button: the constraint's symbol, and its name.
///
/// The name is there because a row of bare symbols is only discoverable to someone who
/// already knows them.
/// A selection that would satisfy the constraint outlines the button, so the user can
/// see which one is about to do something without hovering all eleven.
fn constraint_button(
    ui: &mut egui::Ui,
    kind: sketch_mode::ConstraintKind,
    armed: bool,
    ready: bool,
) -> egui::Response {
    const GLYPH: f32 = 13.0;
    let font = egui::FontId::proportional(12.0);
    let galley = ui.painter().layout_no_wrap(
        kind.name().to_owned(),
        font,
        ui.style().visuals.text_color(),
    );
    let size = egui::vec2(GLYPH + 4.0 + galley.size().x + 12.0, 24.0);
    let (rect, response) = ui.allocate_exact_size(size, egui::Sense::click());
    let (fill, color) = if armed {
        (Some(super::theme::ACCENT_TINT), super::theme::ACCENT_TEXT)
    } else if response.hovered() {
        (Some(super::theme::HOVER), super::theme::TEXT_STRONG)
    } else {
        (None, super::theme::TEXT)
    };
    let painter = ui.painter();
    if let Some(fill) = fill {
        painter.rect_filled(rect, 5.0, fill);
    }
    // A selection that suits the constraint outlines it, in the blue of loose geometry.
    if ready {
        painter.rect_stroke(
            rect,
            5.0,
            egui::Stroke::new(1.0, LOOSE_LABEL),
            egui::StrokeKind::Inside,
        );
    }
    let glyph = egui::Rect::from_center_size(
        egui::pos2(rect.left() + 6.0 + GLYPH * 0.5, rect.center().y),
        egui::vec2(GLYPH, GLYPH),
    );
    super::symbols::paint(painter, glyph, constraint_symbol(kind), color);
    painter.galley(
        egui::pos2(
            rect.left() + 6.0 + GLYPH + 4.0,
            rect.center().y - galley.size().y * 0.5,
        ),
        galley,
        color,
    );
    response
}

/// A sketch action — construction, delete, move and the rest: a small symbol and the
/// action's name, flat until touched like the tools beside it.
///
/// Smaller than a tool's symbol because these act on what is already drawn rather than
/// drawing, and because the row they sit in has to stay within three lines.
fn action_button(symbol: Symbol, label: &str, selected: bool) -> impl egui::Widget + '_ {
    move |ui: &mut egui::Ui| {
        const SMALL: f32 = 18.0;
        let font = egui::FontId::proportional(12.5);
        let galley = ui
            .painter()
            .layout_no_wrap(label.to_owned(), font, super::theme::TEXT);
        let size = egui::vec2(3.0 + SMALL + 3.0 + galley.size().x + 8.0, 24.0);
        let (rect, response) = ui.allocate_exact_size(size, egui::Sense::click());
        let (fill, color) = if !ui.is_enabled() {
            (None, super::theme::TEXT_DISABLED)
        } else if selected {
            (Some(super::theme::ACCENT_TINT), super::theme::ACCENT_TEXT)
        } else if response.is_pointer_button_down_on() {
            (Some(super::theme::PRESS), super::theme::TEXT_STRONG)
        } else if response.hovered() {
            (Some(super::theme::HOVER), super::theme::TEXT_STRONG)
        } else {
            (None, super::theme::TEXT)
        };
        let painter = ui.painter();
        if let Some(fill) = fill {
            painter.rect_filled(rect, 5.0, fill);
        }
        let icon = egui::Rect::from_min_size(
            egui::pos2(rect.left() + 3.0, rect.center().y - SMALL * 0.5),
            egui::vec2(SMALL, SMALL),
        );
        super::symbols::paint(painter, icon, symbol, color);
        painter.galley(
            egui::pos2(icon.right() + 3.0, rect.center().y - galley.size().y * 0.5),
            galley,
            color,
        );
        response
    }
}

/// The one primary button of the sketch toolbar: the check and "Finish Sketch", in white
/// on the accent.
fn finish_button() -> impl egui::Widget {
    |ui: &mut egui::Ui| {
        const SMALL: f32 = 18.0;
        let font = egui::FontId::proportional(12.5);
        let galley = ui.painter().layout_no_wrap(
            "Finish Sketch".to_owned(),
            font,
            super::theme::TEXT_STRONG,
        );
        let size = egui::vec2(4.0 + SMALL + 3.0 + galley.size().x + 10.0, 24.0);
        let (rect, response) = ui.allocate_exact_size(size, egui::Sense::click());
        let fill = if response.hovered() {
            super::theme::ACCENT
        } else {
            super::theme::ACCENT_BUTTON
        };
        let painter = ui.painter();
        painter.rect_filled(rect, 5.0, fill);
        let icon = egui::Rect::from_min_size(
            egui::pos2(rect.left() + 4.0, rect.center().y - SMALL * 0.5),
            egui::vec2(SMALL, SMALL),
        );
        let white = super::theme::TEXT_STRONG;
        super::symbols::paint_inked(painter, icon, Symbol::FinishSketch, white, white);
        painter.galley(
            egui::pos2(icon.right() + 3.0, rect.center().y - galley.size().y * 0.5),
            galley,
            white,
        );
        response
    }
}

/// Either kind of tool, so one icon set serves the sketch toolbar and the modelling one.
///
/// The two enums live in different modules and mean different things, but a button is a
/// button: without this every modelling tool would have needed a second, parallel way of
/// drawing a symbol, and the two would have drifted.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum AnyTool {
    Sketch(SketchTool),
    Model(ToolKind),
    /// The way into the Simulation workspace, which is neither: it is a button with a
    /// symbol all the same.
    Simulate,
    /// The caliper: inspects the model, changes nothing.
    Measure,
    /// Frames the model in the view.
    Fit,
}

impl AnyTool {
    /// The name the icon is filed under; the sketch titles and the modelling ones do not
    /// collide.
    pub(crate) fn name(self) -> &'static str {
        match self {
            AnyTool::Sketch(t) => t.name(),
            AnyTool::Model(k) => k.title(),
            AnyTool::Simulate => "Simulate",
            AnyTool::Measure => "Measure",
            AnyTool::Fit => "Fit",
        }
    }
}

impl From<SketchTool> for AnyTool {
    fn from(t: SketchTool) -> Self {
        AnyTool::Sketch(t)
    }
}

impl From<ToolKind> for AnyTool {
    fn from(k: ToolKind) -> Self {
        AnyTool::Model(k)
    }
}

/// Side of the square a tool symbol is painted in.
const ICON: f32 = 28.0;

/// The id of a painted tool icon, so it can be found without text to search for.
///
/// Absolute rather than derived from the surrounding `Ui`: `salt` already separates the
/// toolbar's copy of an icon from the same icon in a variant menu, and an id that does
/// not depend on where the button happens to sit is one anything can ask for.
pub(crate) fn tool_icon_id(salt: &str, tool: AnyTool) -> egui::Id {
    egui::Id::new(("tool-icon", salt, tool.name()))
}

/// A tool button showing its icon alone.
fn tool_icon(
    ui: &mut egui::Ui,
    salt: &str,
    tool: impl Into<AnyTool>,
    selected: bool,
) -> egui::Response {
    tool_button(ui, salt, tool, None, selected, true)
}

/// A tool button with its icon painted rather than typed: the default fonts have no
/// reliable glyphs for these shapes, and a drawn icon reads the same on every machine.
///
/// `label` puts the name beside the symbol, which is what the modelling toolbar and the
/// menus want: a row of bare symbols is discoverable only to someone who already knows
/// them. Icon and label are one widget, so a click anywhere on it starts the tool —
/// the symbol used to be a decorative thing beside a button, and clicking it did nothing.
fn tool_button(
    ui: &mut egui::Ui,
    salt: &str,
    tool: impl Into<AnyTool>,
    label: Option<&str>,
    selected: bool,
    enabled: bool,
) -> egui::Response {
    shaped_tool_button(
        ui,
        salt,
        tool.into(),
        label,
        selected,
        enabled,
        ButtonShape::Inline,
    )
}

/// How a tool button arranges its symbol and its name.
#[derive(Clone, Copy, PartialEq, Eq)]
enum ButtonShape {
    /// Name beside the symbol: menus, and the icon-only sketch tools.
    Inline,
    /// Name under the symbol, small: the ribbon.
    Stacked,
}

/// Font of a stacked button's name.
const STACKED_LABEL: f32 = 11.5;

/// The size a tool button will take, so a ribbon group can be measured before it is laid
/// out and wrapped as one piece.
fn tool_button_size(ui: &egui::Ui, label: Option<&str>, shape: ButtonShape) -> egui::Vec2 {
    let font = match shape {
        ButtonShape::Inline => egui::FontId::proportional(13.0),
        ButtonShape::Stacked => egui::FontId::proportional(STACKED_LABEL),
    };
    let text = label.map_or(0.0, |text| {
        ui.painter()
            .layout_no_wrap(text.to_owned(), font, egui::Color32::WHITE)
            .size()
            .x
    });
    match shape {
        ButtonShape::Inline => {
            egui::vec2(ICON + if label.is_some() { text + 8.0 } else { 0.0 }, ICON)
        }
        ButtonShape::Stacked => egui::vec2((text + 14.0).max(ICON + 12.0), ICON + 15.0),
    }
}

fn shaped_tool_button(
    ui: &mut egui::Ui,
    salt: &str,
    tool: AnyTool,
    label: Option<&str>,
    selected: bool,
    enabled: bool,
    shape: ButtonShape,
) -> egui::Response {
    let size = tool_button_size(ui, label, shape);
    let (rect, _) = ui.allocate_exact_size(size, egui::Sense::hover());
    // A stable id rather than an automatic one: these buttons may paint no text, so an
    // id is the only handle anything — a test, or egui's own focus — has on them. `salt`
    // keeps the toolbar's copy of an icon apart from the same icon in a variant menu.
    let sense = if enabled {
        egui::Sense::click()
    } else {
        egui::Sense::hover()
    };
    let response = ui.interact(rect, tool_icon_id(salt, tool), sense);
    // Flat until touched, the way a ribbon is: a strip of filled boxes is what makes a
    // toolbar look like a form.
    let (fill, color) = if !enabled {
        (None, super::theme::TEXT_DISABLED)
    } else if selected {
        (Some(super::theme::ACCENT_TINT), super::theme::ACCENT_TEXT)
    } else if response.is_pointer_button_down_on() {
        (Some(super::theme::PRESS), super::theme::TEXT_STRONG)
    } else if response.hovered() {
        (Some(super::theme::HOVER), super::theme::TEXT_STRONG)
    } else {
        (None, super::theme::TEXT)
    };
    let painter = ui.painter();
    if let Some(fill) = fill {
        painter.rect_filled(rect, 5.0, fill);
    }
    let icon = match shape {
        ButtonShape::Inline => egui::Rect::from_min_size(rect.min, egui::vec2(ICON, ICON)),
        ButtonShape::Stacked => egui::Rect::from_center_size(
            egui::pos2(rect.center().x, rect.top() + 1.0 + ICON * 0.5),
            egui::vec2(ICON, ICON),
        ),
    };
    if let Some(text) = label {
        let font = match shape {
            ButtonShape::Inline => egui::FontId::proportional(13.0),
            ButtonShape::Stacked => egui::FontId::proportional(STACKED_LABEL),
        };
        let galley = painter.layout_no_wrap(text.to_owned(), font, color);
        let at = match shape {
            ButtonShape::Inline => egui::pos2(
                rect.left() + ICON + 4.0,
                rect.center().y - galley.size().y * 0.5,
            ),
            ButtonShape::Stacked => egui::pos2(
                rect.center().x - galley.size().x * 0.5,
                rect.bottom() - 2.0 - galley.size().y,
            ),
        };
        painter.galley(at, galley, color);
    }
    paint_symbol(painter, icon, tool, color);
    response
}

/// A tool's symbol, painted into the square `rect` in `color`.
fn paint_symbol(painter: &egui::Painter, rect: egui::Rect, tool: AnyTool, color: egui::Color32) {
    super::symbols::paint(painter, rect, tool_symbol(tool), color);
}

/// Which symbol of the set a tool wears.
fn tool_symbol(tool: AnyTool) -> Symbol {
    match tool {
        AnyTool::Model(kind) => match kind {
            ToolKind::Sketch => Symbol::CreateSketch,
            ToolKind::Extrude => Symbol::Extrude,
            ToolKind::Revolve => Symbol::Revolve,
            ToolKind::Sweep => Symbol::Sweep,
            ToolKind::Loft => Symbol::Loft,
            ToolKind::Fillet => Symbol::Fillet,
            ToolKind::Chamfer => Symbol::Chamfer,
            ToolKind::Thread => Symbol::Thread,
            ToolKind::Combine => Symbol::Combine,
            ToolKind::Move => Symbol::Move,
            ToolKind::OffsetPlane => Symbol::OffsetPlane,
            ToolKind::AngledPlane => Symbol::AngledPlane,
            ToolKind::Component => Symbol::Component,
        },
        AnyTool::Simulate => Symbol::Simulate,
        AnyTool::Measure => Symbol::Measure,
        AnyTool::Fit => Symbol::Fit,
        AnyTool::Sketch(t) => match t {
            SketchTool::Select => Symbol::Select,
            SketchTool::Line => Symbol::Line,
            SketchTool::Rectangle => Symbol::Rectangle,
            SketchTool::CenterRectangle => Symbol::CenterRectangle,
            SketchTool::Circle => Symbol::Circle,
            SketchTool::Circle2Point => Symbol::Circle2Point,
            SketchTool::Circle3Point => Symbol::Circle3Point,
            SketchTool::Arc3Point => Symbol::Arc3Point,
            SketchTool::ArcCenter => Symbol::ArcCenter,
            SketchTool::Polygon => Symbol::Polygon,
            SketchTool::Slot => Symbol::Slot,
            SketchTool::SlotOverall => Symbol::SlotOverall,
            SketchTool::SlotCenterPoint => Symbol::SlotCenterPoint,
            SketchTool::Text => Symbol::Text,
            SketchTool::Dimension => Symbol::Dimension,
            SketchTool::Trim => Symbol::Trim,
            SketchTool::Break => Symbol::Break,
            SketchTool::Fillet => Symbol::SketchFillet,
            SketchTool::Constrain(kind) => constraint_symbol(kind),
        },
    }
}

/// The symbol of a constraint, on its button and on the toolbar alike.
fn constraint_symbol(kind: sketch_mode::ConstraintKind) -> Symbol {
    use sketch_mode::ConstraintKind as K;
    match kind {
        K::Coincident => Symbol::Coincident,
        K::Horizontal => Symbol::Horizontal,
        K::Vertical => Symbol::Vertical,
        K::Parallel => Symbol::Parallel,
        K::Perpendicular => Symbol::Perpendicular,
        K::Tangent => Symbol::Tangent,
        K::Equal => Symbol::Equal,
        K::Concentric => Symbol::Concentric,
        K::Midpoint => Symbol::Midpoint,
        K::Symmetric => Symbol::Symmetric,
        K::Fix => Symbol::Fix,
    }
}

/// The measurement, drawn on the geometry it describes.
///
/// In the viewport rather than a side panel because that is where the user is looking,
/// and selectable so a number can be copied straight into a dimension box. It is derived
/// fresh every frame from the picks, so it cannot outlive or contradict the model.
fn measure_overlay(editor: &Editor, ctx: &egui::Context) {
    let Some(readout) = super::measure::readout(editor) else {
        return;
    };
    let Some(px) = editor
        .camera
        .world_to_screen(readout.anchor, editor.window_px)
    else {
        return;
    };
    let ppp = ctx.pixels_per_point();
    let pos = egui::pos2(px[0] as f32 / ppp + 16.0, px[1] as f32 / ppp + 16.0);
    egui::Area::new(egui::Id::new("measure-readout"))
        .fixed_pos(pos)
        .order(egui::Order::Foreground)
        .show(ctx, |ui| {
            egui::Frame::popup(ui.style()).show(ui, |ui| {
                ui.vertical(|ui| {
                    for (i, text) in readout.lines.iter().enumerate() {
                        // The first line names what was measured; the rest are the values.
                        let rich = match i {
                            0 => egui::RichText::new(text).strong(),
                            _ => egui::RichText::new(text),
                        };
                        ui.add(egui::Label::new(rich).selectable(true));
                    }
                });
            });
        });
}

/// The head of a side panel: what the panel is, small and spaced, over the name of the
/// thing it shows.
fn panel_title(ui: &mut egui::Ui, kind: &str, name: &str) {
    ui.add_space(4.0);
    super::theme::caption(ui, kind);
    ui.heading(name);
    ui.add_space(2.0);
}

/// A selectable row of a list that takes the rest of the line, so the whole row lights
/// up on hover and selection rather than a box hugging the text.
fn row_label(
    ui: &mut egui::Ui,
    selected: bool,
    text: impl Into<egui::WidgetText>,
) -> egui::Response {
    ui.with_layout(egui::Layout::top_down_justified(egui::Align::LEFT), |ui| {
        ui.selectable_label(selected, text)
    })
    .inner
}

fn browser(editor: &mut Editor, ui: &mut egui::Ui, commands: &mut Vec<Command>) {
    panel_title(ui, "Browser", &editor.doc.name);
    egui::ScrollArea::vertical().show(ui, |ui| {
        super::theme::section("Origin").show(ui, |ui| {
            ui.checkbox(&mut editor.show_origin, "Show origin planes and axes");
            ui.checkbox(&mut editor.show_grid, "Show grid");
            ui.checkbox(&mut editor.show_axes, "Show grid axes");
        });
        // Salted, because the count is part of the header text and a section keyed on its
        // own label would shut itself the moment a parameter was added to it.
        super::theme::section(format!("Parameters ({})", editor.doc.parameters().len()))
            .id_salt("document-parameters")
            .default_open(false)
            .show(ui, |ui| document_parameters(editor, ui, commands));
        let planes = editor.cached_planes.clone();
        if !planes.is_empty() {
            super::theme::section("Construction planes").show(ui, |ui| {
                for (id, _) in planes {
                    let name = editor.feature_name(id);
                    let selected = editor.selected_feature == Some(id);
                    if row_label(ui, selected, name).clicked() {
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

/// Editable text of the document's parameter panel.
///
/// Drafts rather than live values, for the same reason the sketch's panel keeps them: an
/// expression is only taken when the user leaves the box or presses Enter, so a name half
/// typed is not an error shouted about on every keystroke.
#[derive(Default)]
pub(crate) struct ParametersUi {
    /// One row per parameter. The committed name is kept beside the draft one because
    /// editing that box is a *rename* — an operation that has to say what it renames
    /// from, and that follows the old name into every feature and sketch that reads it.
    pub(crate) rows: Vec<ParamRow>,
    pub(crate) new_name: String,
    pub(crate) new_expr: String,
    /// Why the last edit was refused, shown under the table rather than in the error
    /// popup: the user is typing, and a modal over a typo is worse than the typo.
    pub(crate) error: Option<String>,
    /// The name of a delete the user is being warned about.
    pub(crate) confirm_delete: Option<String>,
    /// The table moved under the drafts (an edit went through, or an undo), so they are
    /// rebuilt from it on the next pass.
    pub(crate) stale: bool,
}

#[derive(Clone, Default)]
pub(crate) struct ParamRow {
    pub(crate) name: String,
    pub(crate) draft_name: String,
    pub(crate) draft_expr: String,
}

impl ParametersUi {
    /// Puts a row's name box back to the name the parameter still has, after a rename
    /// was refused. [`Self::sync`] cannot do it: the live names have not changed, which
    /// is precisely the case a refusal leaves behind.
    pub(crate) fn revert_name(&mut self, name: &str) {
        if let Some(row) = self.rows.iter_mut().find(|r| r.name == name) {
            row.draft_name = name.to_string();
        }
    }

    /// Rebuilds the drafts from the table when the set of names it holds has changed
    /// underneath them, or when an edit of ours went through. Text the user is in the
    /// middle of typing survives everything else.
    fn sync(&mut self, live: &[(String, String)]) {
        let same = self.rows.len() == live.len()
            && self
                .rows
                .iter()
                .zip(live)
                .all(|(row, (name, _))| row.name == *name);
        if same && !self.stale {
            return;
        }
        self.stale = false;
        self.rows = live
            .iter()
            .map(|(name, expr)| ParamRow {
                name: name.clone(),
                draft_name: name.clone(),
                draft_expr: expr.clone(),
            })
            .collect();
    }
}

/// The document's named constants: what every sketch and every feature in the file can
/// read.
///
/// It lives in the browser, beside the origin and the components, because that is the
/// panel that describes the *document* and it is the one panel that is up whether or not
/// a sketch is open — and a table that only drives sketches would not need to have been
/// lifted out of them. A window off a menu would have hidden the very names the feature
/// exists to keep in front of the user.
fn document_parameters(editor: &mut Editor, ui: &mut egui::Ui, commands: &mut Vec<Command>) {
    let live: Vec<(String, String)> = editor
        .doc
        .parameters()
        .rows()
        .iter()
        .map(|p| (p.name.clone(), p.expr.clone()))
        .collect();
    let values: Vec<Option<f64>> = live
        .iter()
        .map(|(name, _)| editor.doc.parameters().value(name).ok())
        .collect();
    // A sketch with a row of the same name means its own, and neither an edit here nor a
    // rename reaches it. Said here as well as in the sketch's own panel, because from
    // this side it is the reason a parameter change appears not to have worked.
    let shadowed: Vec<Vec<String>> = live
        .iter()
        .map(|(name, _)| {
            editor
                .doc
                .sketches_shadowing(name)
                .into_iter()
                .map(|id| editor.feature_name(id))
                .collect()
        })
        .collect();
    // Greyed out while a tool or a sketch holds the document's transaction open, because
    // an edit made inside one is rolled back by a Cancel that has nothing to do with it.
    // This is only what the panel *looks* like: greying a box does not stop it committing
    // — egui takes focus off a disabled widget, which fires the very commit being
    // prevented — so the refusal itself lives in `Editor::parameters_held`, at the moment
    // the command is applied.
    let locked = editor.doc.in_transaction();
    editor.params_panel.sync(&live);
    if locked {
        // A warning the user can no longer act on does not stay on screen; the edit
        // itself is refused where it is applied, see `Editor::parameters_held`.
        editor.params_panel.confirm_delete = None;
    }
    let ui_state = &mut editor.params_panel;
    ui.add_enabled_ui(!locked, |ui| {
        for (index, (((name, expression), value), shadowed)) in
            live.iter().zip(&values).zip(&shadowed).enumerate()
        {
            let mut blanked = false;
            ui.horizontal(|ui| {
                // The drafts were synced from this very list a moment ago, so the row is
                // there; a missing one is not worth taking the panel down over.
                let Some(row) = ui_state.rows.get_mut(index) else {
                    return;
                };
                let name_box = ui.add(
                    egui::TextEdit::singleline(&mut row.draft_name)
                        .desired_width(64.0)
                        .hint_text("name"),
                );
                let typed = row.draft_name.trim().to_string();
                if name_box.lost_focus() && typed != *name {
                    if typed.is_empty() {
                        // Silently swallowing this would leave an empty box that never
                        // becomes anything, over a parameter that still has its name.
                        row.draft_name = name.clone();
                        blanked = true;
                    } else {
                        commands.push(Command::RenameParameter(name.clone(), typed));
                    }
                }
                let expr_box = ui.add(
                    egui::TextEdit::singleline(&mut row.draft_expr)
                        .desired_width(86.0)
                        .hint_text("expression"),
                );
                if row.draft_expr != *expression && expr_box.lost_focus() {
                    commands.push(Command::SetParameter(name.clone(), row.draft_expr.clone()));
                }
                match value {
                    Some(v) => ui.label(egui::RichText::new(format!("= {v:.3}")).weak()),
                    None => ui
                        .colored_label(egui::Color32::YELLOW, "?")
                        .on_hover_text("This expression does not work out to a number"),
                };
                if !shadowed.is_empty() {
                    ui.colored_label(WARNING_LABEL, "shadowed")
                        .on_hover_text(format!(
                            "{} define a parameter named {name} of their own and mean \
                             that one, so this value does not reach them",
                            shadowed.join(", ")
                        ));
                }
                if ui.small_button("Delete").clicked() {
                    ui_state.confirm_delete = Some(name.clone());
                }
            });
            if blanked {
                ui_state.error = Some(format!("{name} still needs a name"));
            }
        }
        ui.horizontal(|ui| {
            ui.add(
                egui::TextEdit::singleline(&mut ui_state.new_name)
                    .desired_width(64.0)
                    .hint_text("name"),
            );
            ui.add(
                egui::TextEdit::singleline(&mut ui_state.new_expr)
                    .desired_width(86.0)
                    .hint_text("value or expression"),
            );
            let ready =
                !ui_state.new_name.trim().is_empty() && !ui_state.new_expr.trim().is_empty();
            if ui.add_enabled(ready, egui::Button::new("Add")).clicked() {
                // The boxes are emptied by the command, once the parameter is actually
                // in the table: clearing them here would throw away a name and an
                // expression the user has to see to fix a typo in either.
                commands.push(Command::AddParameter(
                    ui_state.new_name.trim().to_string(),
                    ui_state.new_expr.trim().to_string(),
                ));
            }
        });
        if let Some(e) = &ui_state.error {
            ui.colored_label(super::theme::ERROR, e);
        }
        ui.label(
            egui::RichText::new(
                "Every sketch and every feature in this document can read these names. A \
                 sketch parameter of the same name hides the one here. Expressions take \
                 + - * / ^, pi, and functions such as sqrt, min and rad",
            )
            .weak(),
        );
        if locked {
            ui.label(
                egui::RichText::new(
                    "Finish or cancel the sketch or tool first: a parameter changed now \
                     would be undone along with it",
                )
                .weak(),
            );
        }
    });
    // Inside the guard as well, so the buttons of a warning that latched before a tool
    // opened cannot be pressed through it.
    ui.add_enabled_ui(!locked, |ui| delete_confirmation(editor, ui, commands));
}

/// Warns before deleting a parameter something still reads, because nothing is deleted
/// with it: the dependants keep the value they last had and start warning instead.
fn delete_confirmation(editor: &mut Editor, ui: &mut egui::Ui, commands: &mut Vec<Command>) {
    let Some(name) = editor.params_panel.confirm_delete.clone() else {
        return;
    };
    let readers = readers_of(editor, &name);
    if readers.is_empty() {
        editor.params_panel.confirm_delete = None;
        commands.push(Command::RemoveParameter(name));
        return;
    }
    ui.colored_label(
        WARNING_LABEL,
        format!("{name} is still read by {}", readers.join(", ")),
    );
    ui.label(
        egui::RichText::new(
            "Deleting it changes nothing straight away: each of those keeps the value it \
             has now and is flagged until it is given another one",
        )
        .weak(),
    );
    ui.horizontal(|ui| {
        if ui.button("Delete anyway").clicked() {
            editor.params_panel.confirm_delete = None;
            commands.push(Command::RemoveParameter(name.clone()));
        }
        if ui.button("Keep it").clicked() {
            editor.params_panel.confirm_delete = None;
        }
    });
}

/// What a named document parameter is read by, in the user's own words.
///
/// The document's own table, the driven values of features, and the sketches that do not
/// define the name themselves — a sketch that shadows it reads its own parameter, which
/// this name has nothing to do with. What cannot be checked is a sketch's *unbound*
/// text: nothing outside an expression can mention a parameter, so there is nothing else
/// to look in.
fn readers_of(editor: &Editor, name: &str) -> Vec<String> {
    let mut out = Vec::new();
    if editor.doc.parameters().mentions(name) {
        out.push("another parameter".to_string());
    }
    for feature in editor.doc.timeline().features() {
        let driven = feature.exprs.values().any(|text| {
            basset_sketch::expr::referenced_names(text)
                .iter()
                .any(|n| n == name)
        });
        let in_sketch = match &feature.kind {
            FeatureKind::Sketch { sketch, .. } => {
                sketch.parameter(name).is_none() && sketch.mentions_parameter(name)
            }
            _ => false,
        };
        if driven || in_sketch {
            out.push(feature.name.clone());
        }
    }
    out
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
        egui::RichText::new(name.clone()).strong()
    } else {
        egui::RichText::new(name.clone())
    };
    let response = super::theme::section(header)
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
                super::theme::caption(ui, "Bodies");
                for (b, bname) in bodies {
                    ui.horizontal(|ui| {
                        let visible = !editor.hidden_bodies.contains(&b);
                        if super::theme::eye(ui, visible).clicked() {
                            commands.push(Command::ToggleBody(b));
                        }
                        body_row(editor, ui, b, bname, commands);
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
                super::theme::caption(ui, "Sketches");
                for s in sketches {
                    ui.horizontal(|ui| {
                        let visible = !editor.hidden_sketches.contains(&s);
                        if super::theme::eye(ui, visible).clicked() {
                            commands.push(Command::ToggleSketch(s));
                        }
                        let label = row_label(
                            ui,
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
    response.header_response.context_menu(|ui| {
        ui.label(&name);
        ui.separator();
        for format in [MeshFormat::Stl, MeshFormat::ThreeMf] {
            if ui.button(format.label()).clicked() {
                commands.push(Command::ExportComponent(id, format));
                ui.close();
            }
        }
    });
}

/// A body's name box in the browser, while the user is renaming it.
pub(crate) struct BodyRename {
    pub body: BodyRef,
    pub draft: String,
    /// Set when the box opens and cleared once it has taken the keyboard. Asked for only
    /// once, because a box that grabbed focus every frame could never be clicked away
    /// from — and clicking away is one of the ways a rename is kept.
    pub focus: bool,
}

/// A body's name in the browser: a label to select it by, or the box renaming it.
///
/// Renaming follows Fusion: a double-click or the context menu's Rename opens the box,
/// Enter or clicking elsewhere keeps what was typed, Escape puts the old name back. The
/// context menu also offers Create Components from Bodies and the mesh exports, which
/// take every selected body when the row is one of them — a right-click on a selection
/// acts on the selection — and only this row's body otherwise.
fn body_row(
    editor: &mut Editor,
    ui: &mut egui::Ui,
    b: BodyRef,
    name: String,
    commands: &mut Vec<Command>,
) {
    if let Some(rename) = editor.body_rename.as_mut().filter(|r| r.body == b) {
        let response = ui.add(
            egui::TextEdit::singleline(&mut rename.draft)
                .id_salt(("body-rename", b.0.0))
                .desired_width(140.0),
        );
        if std::mem::take(&mut rename.focus) {
            response.request_focus();
        }
        // Escape also takes the focus away, so it has to be told apart from the ways
        // of leaving the box that keep the name.
        if response.lost_focus() {
            if ui.input(|i| i.key_pressed(egui::Key::Escape)) {
                editor.body_rename = None;
            } else {
                commands.push(Command::RenameBody(b, rename.draft.clone()));
            }
        }
        return;
    }
    let locked = editor.tool.is_some() || editor.is_sketching();
    let selected = editor.selection.bodies.contains(&b);
    let response = row_label(ui, selected, &name);
    if response.clicked() {
        editor.selection.clear();
        editor.selection.bodies.push(b);
        commands.push(Command::SelectFeature(b.0));
    }
    if response.double_clicked() && !locked {
        commands.push(Command::BeginBodyRename(b));
    }
    response.context_menu(|ui| {
        ui.label(&name);
        ui.separator();
        if ui
            .add_enabled(!locked, egui::Button::new("Rename"))
            .clicked()
        {
            commands.push(Command::BeginBodyRename(b));
            ui.close();
        }
        let bodies = if selected {
            editor.selection.bodies.clone()
        } else {
            vec![b]
        };
        if ui
            .add_enabled(!locked, egui::Button::new("Create Components from Bodies"))
            .clicked()
        {
            commands.push(Command::ComponentsFromBodies(bodies.clone()));
            ui.close();
        }
        ui.separator();
        for format in [MeshFormat::Stl, MeshFormat::ThreeMf] {
            if ui.button(format.label()).clicked() {
                commands.push(Command::ExportBodies(bodies.clone(), format));
                ui.close();
            }
        }
    });
}

fn timeline(editor: &Editor, ui: &mut egui::Ui, commands: &mut Vec<Command>) {
    let timeline = editor.doc.timeline();
    let cursor = timeline.cursor();
    let len = timeline.len();
    let locked = editor.tool.is_some() || editor.is_sketching();
    let diff = editor.compare.as_ref().map(|c| &c.diff);
    ui.horizontal(|ui| {
        ui.add_enabled_ui(!locked, |ui| {
            ui.spacing_mut().item_spacing.x = 0.0;
            let transport = |ui: &mut egui::Ui, glyph: &str| {
                ui.add(
                    egui::Button::new(egui::RichText::new(glyph).color(super::theme::TEXT_WEAK))
                        .frame_when_inactive(false)
                        .min_size(egui::vec2(22.0, 22.0)),
                )
                .clicked()
            };
            if transport(ui, "⏮") {
                commands.push(Command::SetCursor(0));
            }
            if transport(ui, "◀") {
                commands.push(Command::SetCursor(cursor.saturating_sub(1)));
            }
            if transport(ui, "▶") {
                commands.push(Command::SetCursor((cursor + 1).min(len)));
            }
            if transport(ui, "⏭") {
                commands.push(Command::SetCursor(len));
            }
        });
        ui.separator();
        egui::ScrollArea::horizontal().show(ui, |ui| {
            ui.horizontal(|ui| {
                for (i, f) in timeline.features().iter().enumerate() {
                    if let Some(diff) = diff {
                        removed_chips(ui, diff, i);
                    }
                    if i == cursor {
                        cursor_marker(ui, true);
                    }
                    let status = editor.cached_statuses.get(&f.id);
                    let chip = Chip {
                        kind: f.kind.default_name(),
                        rolled_back: i >= cursor,
                        suppressed: f.suppressed,
                        selected: editor.selected_feature == Some(f.id),
                        status: match status {
                            Some(FeatureStatus::Failed(_)) => Some(super::theme::ERROR),
                            Some(FeatureStatus::Warned(_)) => Some(WARNING_LABEL),
                            _ => None,
                        },
                    };
                    let change = diff.and_then(|d| d.feature(f.id)).map(|c| c.change);
                    let response = chip.show(ui).on_hover_ui(|ui| {
                        ui.label(&f.name);
                        if let Some(change) = change {
                            ui.colored_label(change_color(change), change_label(change));
                        }
                        match status {
                            Some(FeatureStatus::Failed(msg)) => {
                                ui.colored_label(super::theme::ERROR, msg);
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
                    if let Some(change) = change {
                        change_mark(ui, response.rect, change);
                    }
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
                if let Some(diff) = diff {
                    removed_chips(ui, diff, len);
                }
                if cursor == len {
                    cursor_marker(ui, true);
                }
            });
        });
    });
}

/// One feature on the timeline: its tool's symbol and its abbreviation on a rounded
/// chip, tinted when it failed or warned, dimmed once the cursor has rolled back past it.
struct Chip {
    /// The kind as [`FeatureKind::default_name`] names it.
    kind: &'static str,
    rolled_back: bool,
    suppressed: bool,
    selected: bool,
    /// The colour of a failure or warning, which tints the chip and draws its edge.
    status: Option<egui::Color32>,
}

impl Chip {
    fn show(self, ui: &mut egui::Ui) -> egui::Response {
        const H: f32 = 22.0;
        let abbrev = abbreviation_of(self.kind);
        let galley = ui.painter().layout_no_wrap(
            abbrev.to_owned(),
            egui::FontId::proportional(11.5),
            egui::Color32::WHITE,
        );
        let tool = feature_tool(self.kind);
        let icon_w = if tool.is_some() { H - 2.0 } else { 4.0 };
        let size = egui::vec2(icon_w + galley.size().x + 8.0, H);
        let (rect, response) = ui.allocate_exact_size(size, egui::Sense::click());
        let base = egui::Color32::from_rgb(34, 37, 44);
        let fill = if self.selected {
            super::theme::ACCENT_TINT
        } else if response.hovered() {
            super::theme::HOVER
        } else {
            base
        };
        let fill = match self.status {
            Some(c) => fill.lerp_to_gamma(c, 0.22),
            None => fill,
        };
        let stroke = match (self.status, self.selected) {
            (Some(c), _) => egui::Stroke::new(1.0, c.gamma_multiply(0.7)),
            (None, true) => egui::Stroke::new(1.0, super::theme::ACCENT),
            (None, false) => egui::Stroke::NONE,
        };
        let painter = ui.painter();
        painter.rect(rect, 6.0, fill, stroke, egui::StrokeKind::Inside);
        let color = if self.rolled_back {
            super::theme::TEXT_DISABLED
        } else if self.selected {
            super::theme::ACCENT_TEXT
        } else {
            super::theme::TEXT
        };
        if let Some(tool) = tool {
            let icon = egui::Rect::from_min_size(rect.min + egui::vec2(1.0, 0.0), egui::vec2(H, H));
            paint_symbol(painter, icon.shrink(1.0), AnyTool::Model(tool), color);
        }
        let at = egui::pos2(
            rect.left() + icon_w + 2.0,
            rect.center().y - galley.size().y * 0.5,
        );
        let width = galley.size().x;
        painter.galley_with_override_text_color(at, galley, color);
        if self.suppressed {
            let y = rect.center().y;
            painter.line_segment(
                [egui::pos2(at.x - 1.0, y), egui::pos2(at.x + width + 1.0, y)],
                egui::Stroke::new(1.0, color),
            );
        }
        response
    }
}

/// The modelling tool whose symbol stands for a feature of this kind on the timeline.
fn feature_tool(kind: &str) -> Option<ToolKind> {
    Some(match kind {
        "Component" | "Component from Body" => ToolKind::Component,
        "Sketch" => ToolKind::Sketch,
        "Offset Plane" => ToolKind::OffsetPlane,
        "Angled Plane" => ToolKind::AngledPlane,
        "Extrude" => ToolKind::Extrude,
        "Revolve" => ToolKind::Revolve,
        "Sweep" => ToolKind::Sweep,
        "Loft" => ToolKind::Loft,
        "Fillet" => ToolKind::Fillet,
        "Chamfer" => ToolKind::Chamfer,
        "Thread" => ToolKind::Thread,
        "Combine" => ToolKind::Combine,
        "Move" => ToolKind::Move,
        _ => return None,
    })
}

/// The chips for the features the compared version had at `slot` and this one has not:
/// struck through and red, named on hover, and clickable to nothing, since there is no
/// feature to select. Drawn before the chip that now stands at `slot`, the way a text
/// diff shows the old line above the new.
fn removed_chips(ui: &mut egui::Ui, diff: &basset_core::DocumentDiff, slot: usize) {
    for gone in diff.removed_features().filter(|f| f.slot == slot) {
        let text = egui::RichText::new(abbreviation_of(gone.kind))
            .strikethrough()
            .color(DIFF_REMOVED_LABEL);
        let response = ui.add(egui::Button::new(text).frame(false));
        change_mark(ui, response.rect, Change::Removed);
        response.on_hover_ui(|ui| {
            ui.label(&gone.name);
            ui.colored_label(DIFF_REMOVED_LABEL, change_label(Change::Removed));
        });
    }
}

/// A bar along the foot of a timeline chip in the colour of its change. A bar rather
/// than a fill, because the fill already says whether the feature failed or warned and a
/// feature can be both new and broken.
fn change_mark(ui: &mut egui::Ui, rect: egui::Rect, change: Change) {
    let bar = egui::Rect::from_min_max(
        egui::pos2(rect.left() + 2.0, rect.bottom() - 3.0),
        egui::pos2(rect.right() - 2.0, rect.bottom() - 1.0),
    );
    ui.painter().rect_filled(bar, 1.0, change_color(change));
}

fn change_label(change: Change) -> &'static str {
    match change {
        Change::Added => "added since the compared version",
        Change::Removed => "removed since the compared version",
        Change::Modified => "changed since the compared version",
    }
}

fn cursor_marker(ui: &mut egui::Ui, _active: bool) {
    let (rect, _) = ui.allocate_exact_size(egui::vec2(4.0, 22.0), egui::Sense::hover());
    ui.painter().rect_filled(rect, 2.0, super::theme::ACCENT);
}

/// The chip text for a kind named as [`FeatureKind::default_name`] names it, which is
/// all a diff keeps of a feature that is gone.
fn abbreviation_of(kind: &str) -> &'static str {
    match kind {
        "Component" | "Component from Body" => "Cmp",
        "Sketch" => "Sk",
        "Offset Plane" => "Pl+",
        "Angled Plane" => "PlA",
        "Extrude" => "Ext",
        "Revolve" => "Rev",
        "Sweep" => "Swp",
        "Loft" => "Lft",
        "Fillet" => "Fil",
        "Chamfer" => "Chm",
        "Thread" => "Thr",
        "Combine" => "Cmb",
        "Move" => "Mov",
        _ => "?",
    }
}

/// The Git menu: where the document stands, a commit, and the versions to compare with.
///
/// The menu is the one place the log is listed, so it is the one place that asks for a
/// fresh reading of it: the status is re-read the frame the menu opens, which is the
/// latest moment that still has it current when the user looks.
fn git_menu(editor: &Editor, ui: &mut egui::Ui, commands: &mut Vec<Command>) {
    let response = ui.menu_button("Git", |ui| {
        let Some(project) = &editor.project else {
            ui.label(
                egui::RichText::new(if editor.path.is_none() {
                    "Save the document inside a git repository, or open a project, to use git here"
                } else {
                    "The document is not in a git repository"
                })
                .weak(),
            );
            ui.separator();
            if ui.button("New project…").clicked() {
                commands.push(Command::NewProject);
                ui.close();
            }
            if ui.button("Open project…").clicked() {
                commands.push(Command::OpenProject);
                ui.close();
            }
            return;
        };
        let branch = project.branch.as_deref().unwrap_or("no commits yet");
        let where_ = match (&project.rel, project.file) {
            (Some(rel), Some(state)) => {
                format!(
                    "{} \u{b7} {branch} \u{b7} {rel} {}",
                    project.name(),
                    state.label()
                )
            }
            _ => format!("{} \u{b7} {branch}", project.name()),
        };
        ui.label(egui::RichText::new(where_).weak());
        ui.separator();
        if ui
            .add(
                egui::Button::new(format!("Project panel{}", commands::hint("project.panel")))
                    .selected(editor.show_project),
            )
            .clicked()
        {
            commands.push(Command::ToggleProjectPanel);
            ui.close();
        }
        if ui
            .button(format!("Commit…{}", commands::hint("file.commit")))
            .clicked()
        {
            commands.push(Command::Commit);
            ui.close();
        }
        ui.separator();
        ui.label(egui::RichText::new("Compare with").weak());
        let comparing = editor.compare.as_ref().map(|c| c.spec.as_str());
        if ui
            .selectable_label(comparing.is_none(), "Nothing")
            .clicked()
        {
            commands.push(Command::Compare(None));
            ui.close();
        }
        let in_history = project.rel.is_some() && project.branch.is_some();
        if ui
            .add_enabled(
                in_history,
                egui::Button::new(format!(
                    "HEAD, the last commit{}",
                    commands::hint("file.compare")
                ))
                .selected(comparing == Some("HEAD")),
            )
            .clicked()
        {
            commands.push(Command::Compare(Some("HEAD".into())));
            ui.close();
        }
        for commit in project
            .history_of_document()
            .take(super::compare::LOG_LENGTH)
        {
            let text = format!("{} {}", commit.short, commit.subject);
            if ui
                .selectable_label(comparing == Some(commit.hash.as_str()), text)
                .on_hover_text(format!("{} \u{b7} {}", commit.author, commit.when))
                .clicked()
            {
                commands.push(Command::Compare(Some(commit.hash.clone())));
                ui.close();
            }
        }
        ui.separator();
        if ui.button("Close project").clicked() {
            commands.push(Command::CloseProject);
            ui.close();
        }
    });
    // Opening the menu is the ask for a fresh status: `clicked` is the press that opened
    // it (or closed it, which re-reads once more, harmlessly).
    if response.response.clicked() && editor.project.is_some() {
        commands.push(Command::RefreshProject);
    }
}

/// The status bar's word on git: the branch, whether the document has changed since the
/// last commit and how many other parts have, or, while comparing, what with and what
/// differs. Clicking it starts or stops the comparison with HEAD.
fn git_chip(editor: &Editor, ui: &mut egui::Ui, commands: &mut Vec<Command>) {
    let Some(project) = &editor.project else {
        return;
    };
    let (text, color, hover) = match &editor.compare {
        Some(compare) => (
            format!("vs {}: {}", compare.label(), compare.diff.summary()),
            DIFF_MODIFIED_LABEL,
            "Comparing with a version in git. Click to stop.".to_string(),
        ),
        None => (
            project.chip(),
            ui.visuals().weak_text_color(),
            match (&project.rel, project.file) {
                (Some(rel), Some(state)) => format!(
                    "{rel} is {} in project {}. Click to compare with HEAD.",
                    state.label(),
                    project.name()
                ),
                _ => format!(
                    "Project {}. Save the document into it to compare and commit it.",
                    project.name()
                ),
            },
        ),
    };
    let response = ui
        .add(egui::Button::new(egui::RichText::new(text).color(color)).frame(false))
        .on_hover_text(hover);
    if response.clicked() {
        commands.push(Command::ToggleCompare);
    }
}

/// While comparing, a strip at the top of the viewport that says what the colours mean
/// and offers the way out. It is the only legend the overlay has, since the colours
/// alone could as well be a failed feature or a selection.
fn compare_banner(
    editor: &Editor,
    ctx: &egui::Context,
    free: egui::Rect,
    commands: &mut Vec<Command>,
) {
    let Some(compare) = &editor.compare else {
        return;
    };
    egui::Area::new(egui::Id::new("compare-banner"))
        .order(egui::Order::Foreground)
        .fixed_pos(egui::pos2(free.center().x, free.top() + 8.0))
        .pivot(egui::Align2::CENTER_TOP)
        .show(ctx, |ui| {
            egui::Frame::popup(ui.style()).show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.label(format!("Comparing with {}", compare.label()));
                    ui.separator();
                    ui.colored_label(DIFF_ADDED_LABEL, "\u{25a0} added");
                    ui.colored_label(DIFF_REMOVED_LABEL, "\u{25a0} removed");
                    ui.separator();
                    ui.label(egui::RichText::new(compare.diff.summary()).weak());
                    if ui.button("Done").clicked() {
                        commands.push(Command::Compare(None));
                    }
                });
            });
        });
}

/// The commit box: a message, the parts to record, and Commit saves the open document if
/// it is among them and commits what is ticked.
fn commit_popup(editor: &mut Editor, ctx: &egui::Context, commands: &mut Vec<Command>) {
    let Some(mut commit_box) = editor.commit_box.clone() else {
        return;
    };
    let Some(project) = &editor.project else {
        editor.commit_box = None;
        return;
    };
    // What can be ticked: every changed part, and the open document whether or not it
    // has been saved since it changed — committing it saves it.
    let mut choices: Vec<(String, &'static str)> = project
        .changed()
        .map(|f| (f.path.clone(), f.state.label()))
        .collect();
    if let Some(rel) = &project.rel
        && !choices.iter().any(|(p, _)| p == rel)
    {
        choices.insert(0, (rel.clone(), "open"));
    }
    let mut done: Option<bool> = None;
    egui::Window::new("Commit to git")
        .collapsible(false)
        .resizable(false)
        .anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0])
        .show(ctx, |ui| {
            ui.label(
                egui::RichText::new(format!(
                    "{} \u{b7} {}",
                    project.name(),
                    project.branch.as_deref().unwrap_or("first commit")
                ))
                .weak(),
            );
            if choices.is_empty() {
                ui.label("Nothing has changed since the last commit.");
            }
            for (path, state) in &choices {
                let mut on = commit_box.include.contains(path);
                let label = if *state == "open" {
                    path.clone()
                } else {
                    format!("{path}  \u{b7} {state}")
                };
                if ui.checkbox(&mut on, label).changed() {
                    if on {
                        commit_box.include.insert(path.clone());
                    } else {
                        commit_box.include.remove(path);
                    }
                }
            }
            let response = ui.add(
                egui::TextEdit::multiline(&mut commit_box.message)
                    .hint_text("What changed, and why")
                    .desired_rows(3)
                    .desired_width(360.0),
            );
            if commit_box.message.is_empty() {
                response.request_focus();
            }
            ui.horizontal(|ui| {
                let n = commit_box.include.len();
                let label = match n {
                    0 | 1 => "Commit".to_string(),
                    n => format!("Commit {n} parts"),
                };
                if ui
                    .add_enabled(
                        !commit_box.message.trim().is_empty() && n > 0,
                        egui::Button::new(label),
                    )
                    .clicked()
                {
                    done = Some(true);
                }
                if ui.button("Cancel").clicked() {
                    done = Some(false);
                }
            });
        });
    match done {
        Some(true) => commands.push(Command::CommitWith(
            commit_box.message.clone(),
            commit_box.include.iter().cloned().collect(),
        )),
        Some(false) => editor.commit_box = None,
        None => editor.commit_box = Some(commit_box),
    }
}

/// The Project panel: the parts of the project with their state, and its history.
///
/// A part is opened with a click; a commit unfolds to what it touched, and any part in it
/// opens as it was then. The panel is the one place the whole project is in view, so the
/// commit button lives here too, beside the parts it will record.
fn project_panel(editor: &Editor, ui: &mut egui::Ui, commands: &mut Vec<Command>) {
    let Some(project) = &editor.project else {
        return;
    };
    // Truncated, not wrapped and not extended: a panel grows to fit an unbroken line,
    // and a long folder name would take the viewport with it.
    ui.add(egui::Label::new(egui::RichText::new(project.name()).heading()).truncate())
        .on_hover_text(project.repo.root().display().to_string());
    let changed = project.changed().count();
    let parts = project.files.len();
    ui.label(
        egui::RichText::new(format!(
            "\u{2387} {} \u{b7} {parts} part{} \u{b7} {changed} changed",
            project.branch.as_deref().unwrap_or("no commits yet"),
            if parts == 1 { "" } else { "s" },
        ))
        .weak(),
    );
    ui.horizontal(|ui| {
        if ui
            .button("Commit\u{2026}")
            .on_hover_text("Record the changed parts in git")
            .clicked()
        {
            commands.push(Command::Commit);
        }
        if ui
            .button("New part")
            .on_hover_text("A new document; Save puts it in the project's folder")
            .clicked()
        {
            commands.push(Command::New);
        }
        if ui
            .small_button("\u{21bb}")
            .on_hover_text("Re-read the project from git")
            .clicked()
        {
            commands.push(Command::RefreshProject);
        }
        if ui
            .small_button("\u{2715}")
            .on_hover_text("Hide the panel (Ctrl+Shift+H brings it back)")
            .clicked()
        {
            commands.push(Command::ToggleProjectPanel);
        }
    });
    ui.separator();
    egui::ScrollArea::vertical().show(ui, |ui| {
        super::theme::section(format!("Parts ({parts})"))
            .id_salt("project-parts")
            .default_open(true)
            .show(ui, |ui| {
                if project.files.is_empty() {
                    ui.label(
                        egui::RichText::new("No parts yet. Save a document into the project.")
                            .weak(),
                    );
                }
                for file in &project.files {
                    let current = project.rel.as_deref() == Some(file.path.as_str());
                    let (mark, color) = match file.state {
                        super::git::FileState::Modified => ("\u{25cf} ", DIFF_MODIFIED_LABEL),
                        super::git::FileState::Untracked => ("+ ", DIFF_ADDED_LABEL),
                        super::git::FileState::Clean => ("", ui.visuals().text_color()),
                    };
                    let text = egui::RichText::new(format!("{mark}{}", file.path)).color(color);
                    if ui
                        .selectable_label(current, text)
                        .on_hover_text(format!("{} in git", file.state.label()))
                        .clicked()
                        && !current
                    {
                        commands.push(Command::OpenPart(file.path.clone()));
                    }
                }
            });
        super::theme::section(format!("History ({})", project.log.len()))
            .id_salt("project-history")
            .default_open(true)
            .show(ui, |ui| {
                if project.log.is_empty() {
                    ui.label(
                        egui::RichText::new("No commits yet. Commit… records the parts.").weak(),
                    );
                }
                for commit in &project.log {
                    history_row(editor, project, commit, ui, commands);
                }
            });
    });
}

/// One commit in the History section: a line for the commit, and, unfolded, what it
/// touched and what can be done with it.
fn history_row(
    editor: &Editor,
    project: &super::project::Project,
    commit: &super::git::Commit,
    ui: &mut egui::Ui,
    commands: &mut Vec<Command>,
) {
    let picked = project.picked.as_deref() == Some(commit.hash.as_str());
    let comparing = editor
        .compare
        .as_ref()
        .is_some_and(|c| c.commit.hash == commit.hash);
    let title = egui::RichText::new(format!("{} {}", commit.short, commit.subject));
    let title = if comparing {
        title.color(DIFF_MODIFIED_LABEL)
    } else {
        title
    };
    if ui.selectable_label(picked, title).clicked() {
        commands.push(Command::PickCommit(if picked {
            None
        } else {
            Some(commit.hash.clone())
        }));
    }
    ui.label(
        egui::RichText::new(format!("    {} \u{b7} {}", commit.author, commit.when))
            .weak()
            .small(),
    );
    if !picked {
        return;
    }
    ui.indent(("commit", &commit.hash), |ui| {
        let open = project.rel.as_deref();
        for path in &commit.files {
            let is_part = std::path::Path::new(path)
                .extension()
                .is_some_and(|e| e.eq_ignore_ascii_case(basset_core::file::EXTENSION));
            if !is_part {
                ui.label(egui::RichText::new(path).weak());
                continue;
            }
            ui.horizontal(|ui| {
                ui.label(path);
                if ui
                    .small_button("Open as it was")
                    .on_hover_text("Open this version as a document of its own")
                    .clicked()
                {
                    commands.push(Command::OpenVersion(commit.hash.clone(), path.clone()));
                }
                if open == Some(path.as_str())
                    && ui
                        .add(egui::Button::new("Compare").small().selected(comparing))
                        .on_hover_text("Draw what changed since this commit over the model")
                        .clicked()
                {
                    commands.push(Command::Compare(if comparing {
                        None
                    } else {
                        Some(commit.hash.clone())
                    }));
                }
            });
        }
    });
}

fn sketch_palette(editor: &mut Editor, ui: &mut egui::Ui, commands: &mut Vec<Command>) {
    sketch_palette_body(editor, ui, commands);
    // The palette's checkbox and the View menu's are the same switch, so whatever this
    // one was left at is written back through the editor: the modelling handles have no
    // palette of their own and read it from there.
    if let Mode::Sketch(s) = &editor.mode {
        let to_grid = s.snap_to_grid;
        if to_grid != editor.snapping.to_grid {
            editor.set_snapping(to_grid);
        }
    }
}

fn sketch_palette_body(editor: &mut Editor, ui: &mut egui::Ui, commands: &mut Vec<Command>) {
    let Mode::Sketch(s) = &mut editor.mode else {
        return;
    };
    // The palette is taller than the panel on an ordinary window, and without a scroll
    // area whatever overflowed was simply unreachable — which is how a sketch could end
    // up with no way to apply a constraint at all.
    egui::ScrollArea::vertical().show(ui, |ui| {
        ui.heading("Sketch");
        ui.label(egui::RichText::new("Click points; snap to existing points to join").weak());
        ui.separator();
        ui.checkbox(&mut editor.show_axes, "Show grid axes");
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
        if let Some(kind) = s.armed_constraint() {
            ui.label(egui::RichText::new(kind.hint()).weak());
            let picked = s.constraint_picks().len();
            if picked > 0 {
                ui.colored_label(
                    LOOSE_LABEL,
                    format!("{picked} picked — it applies as soon as the picks are enough"),
                );
            }
            ui.label(
                egui::RichText::new(
                    "Click a pick again to drop it, empty space to start over, Esc (or the \
                     lit button) to put the tool down",
                )
                .weak(),
            );
        } else if s.tool == SketchTool::Select {
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
        } else if let Some(prompt) = s.fillet_prompt() {
            ui.label(egui::RichText::new(prompt).strong());
            ui.label(
                egui::RichText::new(
                    "Click the corner where two curves meet, or the two curves either side \
                     of it. The arc appears at once and its radius is dragged on the \
                     drawing",
                )
                .weak(),
            );
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
        if s.construction {
            ui.colored_label(
                egui::Color32::from_rgb(200, 200, 150),
                "Construction: the next shape is drawn as reference geometry",
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
        let mut highlight = Vec::new();
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
                highlight = redundant_report(s, &r.redundant, ui, commands);
            }
            Some(Err(e)) => conflict_report(s, e, ui, commands),
            None => {}
        }
        // Directly under the degrees-of-freedom readout, because that is the line that
        // prompts the question the list answers: what *is* holding this sketch?
        constraint_list(s, ui, commands, highlight);
        ui.separator();
        super::theme::section("Parameters")
            .default_open(false)
            .show(ui, |ui| parameters_panel(s, ui, commands));
    });
}

/// The sketch's modal operations — Move and Pattern — in a window of their own.
///
/// They used to sit at the bottom of the sketch palette, below the tool hints, the
/// degrees-of-freedom readout and the whole constraint list. On an ordinary window that
/// put them past the fold of a scrolling panel, so the controls that a running operation
/// is *driven by* could not be seen without knowing to scroll for them — and a pattern
/// whose "Select origin" cannot be reached is a pattern whose origin cannot be set. An
/// operation that owns the sketch belongs in front of the user for as long as it does,
/// which is what the modelling tools already do with their dialog.
fn sketch_operation_dialog(
    editor: &mut Editor,
    ctx: &egui::Context,
    free: egui::Rect,
    commands: &mut Vec<Command>,
) {
    let Mode::Sketch(s) = &mut editor.mode else {
        return;
    };
    let Some(title) = s.modal_name() else {
        return;
    };
    let mut centre_on_selection = false;
    let mut pick_center: Option<bool> = None;
    egui::Window::new(title)
        .id(egui::Id::new("sketch-operation"))
        .collapsible(false)
        .resizable(false)
        .default_pos(free.right_top() + egui::vec2(-260.0, 16.0))
        .show(ctx, |ui| {
            ui.set_max_width(240.0);
            if let Some(op) = s.move_op.as_mut() {
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
                if let Some(why) = s.move_refused() {
                    ui.colored_label(super::theme::WARNING, why);
                    ui.label(
                        egui::RichText::new(
                            "The geometry is left where it was. Delete or relax what is \
                             holding it, or move it a way the constraints allow",
                        )
                        .weak(),
                    );
                }
                ui.label(
                    egui::RichText::new(
                        "Drag the arrows and the ring in the viewport, or type here. Enter \
                         applies, Esc puts it back. Constraints still hold",
                    )
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
            if s.pattern_in_progress() {
                let mut changed = false;
                let p = &mut s.pattern;
                ui.horizontal(|ui| {
                    changed |= ui
                        .selectable_value(&mut p.circular, false, "Rectangular")
                        .changed();
                    changed |= ui
                        .selectable_value(&mut p.circular, true, "Circular")
                        .changed();
                });
                if p.circular {
                    changed |= ui
                        .add(
                            egui::DragValue::new(&mut p.count)
                                .range(2..=usize::MAX)
                                .prefix("instances "),
                        )
                        .changed();
                    changed |= ui
                        .add(
                            egui::DragValue::new(&mut p.angle_deg)
                                .speed(1.0)
                                .range(-360.0..=360.0)
                                .prefix("through ")
                                .suffix("\u{b0}"),
                        )
                        .changed();
                    ui.label(egui::RichText::new("Centre").weak());
                    ui.horizontal(|ui| {
                        changed |= ui
                            .add(
                                egui::DragValue::new(&mut p.center.x)
                                    .speed(0.5)
                                    .prefix("x ")
                                    .suffix(" mm"),
                            )
                            .changed();
                        changed |= ui
                            .add(
                                egui::DragValue::new(&mut p.center.y)
                                    .speed(0.5)
                                    .prefix("y ")
                                    .suffix(" mm"),
                            )
                            .changed();
                    });
                    ui.horizontal(|ui| {
                        let picking = s.picking_pattern_center();
                        if ui
                            .add(egui::Button::new("Select origin").selected(picking))
                            .on_hover_text(
                                "Then click in the viewport: the centre snaps to an existing \
                                 point if there is one under the pointer",
                            )
                            .clicked()
                        {
                            pick_center = Some(!picking);
                        }
                        if ui
                            .button("Centre on selection")
                            .on_hover_text("Put the centre at the middle of what is being repeated")
                            .clicked()
                        {
                            centre_on_selection = true;
                        }
                    });
                    if s.picking_pattern_center() {
                        ui.colored_label(LOOSE_LABEL, "Click the origin in the viewport");
                    }
                    ui.label(
                        egui::RichText::new(
                            "Counts include the original. A full turn spreads the instances \
                             evenly all the way round; less than one puts the last one \
                             exactly on the angle",
                        )
                        .weak(),
                    );
                    ui.label(
                        egui::RichText::new(
                            "The copies on screen are the pattern: OK keeps exactly what you \
                             are looking at, Esc puts the sketch back",
                        )
                        .weak(),
                    );
                } else {
                    // The count includes the seed, as Fusion's does, so "3" is what the user
                    // ends up looking at rather than what was added to what they drew.
                    ui.horizontal(|ui| {
                        changed |= ui
                            .add(
                                egui::DragValue::new(&mut p.count_x)
                                    .range(1..=usize::MAX)
                                    .prefix("across "),
                            )
                            .changed();
                        changed |= ui
                            .add(
                                egui::DragValue::new(&mut p.distance_x)
                                    .speed(0.5)
                                    .prefix("dX ")
                                    .suffix(" mm"),
                            )
                            .changed();
                    });
                    ui.horizontal(|ui| {
                        changed |= ui
                            .add(
                                egui::DragValue::new(&mut p.count_y)
                                    .range(1..=usize::MAX)
                                    .prefix("up "),
                            )
                            .changed();
                        changed |= ui
                            .add(
                                egui::DragValue::new(&mut p.distance_y)
                                    .speed(0.5)
                                    .prefix("dY ")
                                    .suffix(" mm"),
                            )
                            .changed();
                    });
                    ui.horizontal(|ui| {
                        ui.label("Distance is");
                        changed |= ui
                            .selectable_value(
                                &mut p.spacing,
                                sketch_mode::Spacing::Between,
                                "between copies",
                            )
                            .changed();
                        changed |= ui
                            .selectable_value(
                                &mut p.spacing,
                                sketch_mode::Spacing::Total,
                                "in total",
                            )
                            .changed();
                    });
                    ui.label(
                        egui::RichText::new(
                            "Counts include the original; 1 means no copies that way",
                        )
                        .weak(),
                    );
                    ui.label(
                        egui::RichText::new(
                            "The copies on screen are the pattern: OK keeps exactly what you \
                             are looking at, Esc puts the sketch back",
                        )
                        .weak(),
                    );
                }
                match s.pattern_status() {
                    Some((_, Some(error))) => ui.colored_label(super::theme::ERROR, error),
                    Some((copies, None)) => ui.colored_label(
                        LOOSE_LABEL,
                        match copies {
                            1 => "1 copy".to_string(),
                            n => format!("{n} copies"),
                        },
                    ),
                    None => ui.label(""),
                };
                ui.horizontal(|ui| {
                    if ui.button("OK").clicked() {
                        commands.push(Command::SketchPatternFinish(true));
                    }
                    if ui.button("Cancel").clicked() {
                        commands.push(Command::SketchPatternFinish(false));
                    }
                });
                if changed {
                    commands.push(Command::SketchPatternUpdate);
                }
            }
            if s.offset_in_progress() {
                let mut changed = false;
                changed |= ui
                    .add(
                        egui::DragValue::new(&mut s.offset.distance)
                            .speed(0.5)
                            .prefix("distance ")
                            .suffix(" mm"),
                    )
                    .changed();
                for corner in [sketch_mode::Corner::Round, sketch_mode::Corner::Miter] {
                    changed |= ui
                        .selectable_value(&mut s.offset.corner, corner, corner.name())
                        .on_hover_text(corner.hint())
                        .changed();
                }
                match s.offset_status() {
                    Some((_, Some(error))) => ui.colored_label(super::theme::ERROR, error),
                    Some((made, None)) => ui.colored_label(
                        LOOSE_LABEL,
                        match made {
                            1 => "1 curve".to_string(),
                            n => format!("{n} curves"),
                        },
                    ),
                    None => ui.label(""),
                };
                ui.label(
                    egui::RichText::new(
                        "Rounded corners keep every point of the result the distance from \
                         the drawing; square corners keep every edge that far from its own \
                         edge, and run the edges out to meet",
                    )
                    .weak(),
                );
                ui.label(
                    egui::RichText::new(
                        "Drag the handle on the result in the viewport — across the \
                         geometry and out the far side puts it on that side. It snaps to \
                         the grid; hold shift for anywhere in between",
                    )
                    .weak(),
                );
                ui.label(
                    egui::RichText::new(
                        "What is on screen is the offset: OK keeps exactly that, Esc puts \
                         the sketch back",
                    )
                    .weak(),
                );
                ui.horizontal(|ui| {
                    if ui.button("OK").clicked() {
                        commands.push(Command::SketchOffsetFinish(true));
                    }
                    if ui.button("Cancel").clicked() {
                        commands.push(Command::SketchOffsetFinish(false));
                    }
                });
                if changed {
                    commands.push(Command::SketchOffsetUpdate);
                }
            }
            if s.fillet_in_progress() {
                let changed = ui
                    .add(
                        egui::DragValue::new(&mut s.fillet.radius)
                            .speed(0.25)
                            .range(0.01..=f64::MAX)
                            .prefix("radius ")
                            .suffix(" mm"),
                    )
                    .changed();
                match s.fillet_status() {
                    Some((_, Some(error))) => ui.colored_label(super::theme::ERROR, error),
                    Some((true, None)) => ui.colored_label(LOOSE_LABEL, "Corner rounded"),
                    _ => ui.label(""),
                };
                ui.label(
                    egui::RichText::new(
                        "Drag the handle on the corner in the viewport: it sits the radius \
                         out along the bisector, so where you drop it is the radius. It \
                         snaps to the grid; hold shift for anywhere in between",
                    )
                    .weak(),
                );
                ui.label(
                    egui::RichText::new(
                        "The arc on screen is the fillet, tangent to both curves and holding \
                         them that way: OK keeps it, Esc puts the corner back",
                    )
                    .weak(),
                );
                ui.horizontal(|ui| {
                    if ui.button("OK").clicked() {
                        commands.push(Command::SketchFilletFinish(true));
                    }
                    if ui.button("Cancel").clicked() {
                        commands.push(Command::SketchFilletFinish(false));
                    }
                });
                if changed {
                    commands.push(Command::SketchFilletUpdate);
                }
            }
        });
    if let Some(on) = pick_center {
        s.pick_pattern_center(on);
    }
    if centre_on_selection {
        s.pattern_center_from_selection();
        s.pick_pattern_center(false);
        commands.push(Command::SketchPatternUpdate);
    }
}

/// Every constraint in the sketch, as a list.
///
/// The badges in the viewport are the primary way to see and delete a constraint, but a
/// badge can sit under other geometry, and a sketch that has gone wrong is exactly the
/// one whose badges are hard to read. The list is the fallback that always works:
/// hovering a row lights up the geometry the constraint holds, and every row can be
/// deleted from here.
/// What a failed solve has to say. A residual and an iteration count mean nothing to
/// someone drawing a bracket; the constraints that could not be satisfied are the thing
/// to act on, and they are also drawn red on the sketch.
fn conflict_report(
    s: &super::SketchEditor,
    error: &basset_sketch::SolveError,
    ui: &mut egui::Ui,
    commands: &mut Vec<Command>,
) {
    let basset_sketch::SolveError::DidNotConverge { conflicting, .. } = error else {
        ui.colored_label(super::theme::ERROR, error.to_string());
        return;
    };
    ui.colored_label(egui::Color32::YELLOW, "Constraints conflict");
    if conflicting.is_empty() {
        ui.label(
            egui::RichText::new(
                "No single constraint is left over: the solver could not find its way \
                 there from the current shape. Drag the geometry nearer to what you \
                 want and it will usually take.",
            )
            .weak(),
        );
        return;
    }
    ui.label(egui::RichText::new("These ask for incompatible things; delete or relax one:").weak());
    for id in conflicting {
        let Some(c) = s.sketch.constraint(*id) else {
            continue;
        };
        ui.horizontal(|ui| {
            if ui.add(egui::Button::new("Delete").frame(false)).clicked() {
                commands.push(Command::SketchRemoveConstraint(*id));
            }
            let response = ui.add(
                egui::Label::new(
                    egui::RichText::new(constraint_label(c))
                        .color(egui::Color32::from_rgb(240, 150, 140)),
                )
                .sense(egui::Sense::click()),
            );
            if response.clicked() {
                commands.push(Command::SketchSelect(c.references()));
            }
        });
    }
}

/// What a successful solve has to say about constraints that earn nothing. A redundant
/// constraint is not wrong today, but it is the one that turns into a conflict the day a
/// dimension changes, and a sketch with two of something is a sketch whose author has
/// lost track of it. Hovering a row lights up what the constraint holds, as the main
/// list does, and the highlight is handed back so the two lists share one.
fn redundant_report(
    s: &super::SketchEditor,
    redundant: &[basset_sketch::ConstraintId],
    ui: &mut egui::Ui,
    commands: &mut Vec<Command>,
) -> Vec<basset_sketch::EntityId> {
    let mut highlight = Vec::new();
    if redundant.is_empty() {
        return highlight;
    }
    let count = match redundant.len() {
        1 => "1 redundant constraint".to_string(),
        n => format!("{n} redundant constraints"),
    };
    ui.colored_label(REDUNDANT_LABEL, count);
    ui.label(
        egui::RichText::new(
            "Orange constraints repeat what the rest of the sketch already says; deleting \
             one changes nothing:",
        )
        .weak(),
    );
    for id in redundant {
        let Some(c) = s.sketch.constraint(*id) else {
            continue;
        };
        ui.horizontal(|ui| {
            if ui.add(egui::Button::new("Delete").frame(false)).clicked() {
                commands.push(Command::SketchRemoveConstraint(*id));
            }
            let response = ui.add(
                egui::Label::new(egui::RichText::new(constraint_label(c)).color(REDUNDANT_LABEL))
                    .sense(egui::Sense::click()),
            );
            if response.contains_pointer() {
                highlight = c.references();
            }
            if response.clicked() {
                commands.push(Command::SketchSelect(c.references()));
            }
        });
    }
    highlight
}

/// A constraint as the user reads it: its name, and its value when it has one.
fn constraint_label(c: &basset_sketch::Constraint) -> String {
    match c.dimension_value() {
        // Angles are stored in radians and shown in degrees, as everywhere else.
        Some(v) if matches!(c, basset_sketch::Constraint::Angle { .. }) => {
            format!("{} {:.2}\u{b0}", constraint_name(c), v.to_degrees())
        }
        Some(v) => format!("{} {v:.3} mm", constraint_name(c)),
        None => constraint_name(c).to_string(),
    }
}

/// Every constraint the sketch holds, collapsed by default.
///
/// `highlight` is what the redundant list above already lit up this frame, so the
/// geometry stays lit while the pointer is on that row rather than on one of these.
fn constraint_list(
    s: &mut super::SketchEditor,
    ui: &mut egui::Ui,
    commands: &mut Vec<Command>,
    mut highlight: Vec<basset_sketch::EntityId>,
) {
    let rows: Vec<(
        basset_sketch::ConstraintId,
        String,
        Vec<basset_sketch::EntityId>,
    )> = s
        .sketch
        .constraints()
        .map(|(id, c)| (id, constraint_label(c), c.references()))
        .collect();
    super::theme::section(format!("Constraints ({})", rows.len()))
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
                    if ui.add(egui::Button::new("Delete").frame(false)).clicked() {
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
            let outer = s.outer.lookup();
            match s.sketch.parameter_value_with(name, &outer) {
                Ok(v) => ui.label(egui::RichText::new(format!("= {v:.3}")).weak()),
                Err(_) => ui.colored_label(egui::Color32::YELLOW, "?"),
            };
            if s.shadows(name) {
                // Shadowing is the one thing about two scopes that can genuinely mislead:
                // the document's `width` is still there, this sketch simply means its own.
                ui.colored_label(WARNING_LABEL, "hides")
                    .on_hover_text(format!(
                        "The document also has a parameter named {name}. Inside this \
                         sketch the name means this row, and expressions here cannot \
                         reach the document's one"
                    ));
            }
            if ui.small_button("Delete").clicked() {
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
        ui.colored_label(super::theme::ERROR, e);
    }
    // The document's own table, read-only and visibly apart: these names are in scope
    // here, and a user who cannot see them has no way to know what they may write.
    if !s.outer.is_empty() {
        ui.separator();
        ui.label(egui::RichText::new("From the document").weak());
        for row in s.outer.rows() {
            let hidden = s.sketch.parameter(&row.name).is_some();
            ui.horizontal(|ui| {
                let name = egui::RichText::new(&row.name).weak();
                ui.label(if hidden { name.strikethrough() } else { name });
                match s.outer.value(&row.name) {
                    Ok(v) => ui.label(egui::RichText::new(format!("= {v:.3}")).weak()),
                    Err(_) => ui.colored_label(egui::Color32::YELLOW, "?"),
                };
                if hidden {
                    ui.colored_label(WARNING_LABEL, "hidden here")
                        .on_hover_text("This sketch has a parameter of the same name");
                }
            });
        }
        ui.label(egui::RichText::new("Edit these in the browser, under Parameters").weak());
    }
    ui.label(
        egui::RichText::new(
            "Click a dimension and type a name or a sum to drive it, e.g. wall * 2. \
             Names are looked for in this sketch first and then in the document, and \
             expressions take + - * / ^, pi, and functions such as sqrt, min and rad",
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
///
/// The button yields to geometry under the pointer. A badge sits a few pixels off the
/// entity it marks, and egui's hit test reaches `interact_radius` beyond a widget, so
/// without this a press on the entity's own pick radius — on a tied corner, right where
/// the user is aiming — left egui holding a click on the badge: the press reached the
/// editor, the release was claimed by egui, and the shape never started. The badge is
/// the secondary thing here; a tooltip and a delete entry are reachable from the side
/// of it that faces away from the drawing.
fn constraint_overlay(editor: &mut Editor, ctx: &egui::Context, commands: &mut Vec<Command>) {
    let camera = editor.camera;
    let window = editor.window_px;
    let Mode::Sketch(s) = &mut editor.mode else {
        return;
    };
    let ppp = ctx.pixels_per_point();
    let geometry_first = s.hover.is_some();
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
            // Not interactable: egui then leaves the pointer to whatever is under it,
            // rather than counting the press as over one of its areas.
            .interactable(!geometry_first)
            .show(ctx, |ui| {
                let sense = if geometry_first {
                    egui::Sense::hover()
                } else {
                    egui::Sense::click()
                };
                let response = ui.allocate_response(size, sense);
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
        C::Offset { .. } => "Offset",
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
    let conflicting = s.conflicting().to_vec();
    let redundant = s.redundant().to_vec();
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
                // A dimension the solver could not satisfy reads red here as well as on
                // its leader lines, so the number the user must change is the one lit up;
                // one it did not need reads orange for the same reason.
                let text = egui::RichText::new(&g.text).small();
                let (text, fill) = if conflicting.contains(&g.id) {
                    (
                        text.color(egui::Color32::from_rgb(255, 180, 170)),
                        egui::Color32::from_rgba_unmultiplied(90, 25, 25, 220),
                    )
                } else if redundant.contains(&g.id) {
                    (
                        text.color(egui::Color32::from_rgb(255, 205, 150)),
                        egui::Color32::from_rgba_unmultiplied(90, 50, 15, 220),
                    )
                } else {
                    (text, egui::Color32::from_rgba_unmultiplied(20, 40, 70, 200))
                };
                let button = egui::Button::new(text)
                    .fill(fill)
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
                egui::RichText::new(
                    "A number, or an expression over this sketch's parameters and the \
                     document's — sqrt, min, rad and the rest are available too",
                )
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

/// How many matches the palette shows at once. More than this and the list is taller
/// than the sketch it is covering.
const PALETTE_ROWS: usize = 12;

/// Every command live in this mode, grouped, with its key. Read straight out of the
/// table, so a binding that exists is listed and a listing that exists is bound.
fn shortcut_overlay(editor: &mut Editor, ctx: &egui::Context) {
    if !editor.show_shortcuts {
        return;
    }
    let sketching = editor.is_sketching();
    let mut open = true;
    egui::Window::new(if sketching {
        "Keyboard shortcuts — sketch mode"
    } else {
        "Keyboard shortcuts"
    })
    .collapsible(false)
    .resizable(false)
    .anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0])
    .show(ctx, |ui| {
        ui.label(
            egui::RichText::new(
                "Only what is live in this mode is listed; the rest appears in the other one.",
            )
            .weak(),
        );
        ui.separator();
        egui::ScrollArea::vertical()
            .max_height(520.0)
            .show(ui, |ui| {
                // Two columns, because the list is long and a single column of thirty
                // rows is a scroll rather than a thing you read.
                ui.columns(2, |columns| {
                    for (i, group) in commands::Group::ALL.iter().enumerate() {
                        let rows: Vec<_> = commands::BINDINGS
                            .iter()
                            .filter(|b| b.group == *group && commands::available(b, editor))
                            .collect();
                        if rows.is_empty() {
                            continue;
                        }
                        let ui = &mut columns[i % 2];
                        ui.strong(group.name());
                        for binding in rows {
                            let keys = binding
                                .chords
                                .iter()
                                .map(|c| c.label())
                                .collect::<Vec<_>>()
                                .join(" / ");
                            ui.horizontal(|ui| {
                                let key = egui::RichText::new(if keys.is_empty() {
                                    "—".to_string()
                                } else {
                                    keys
                                })
                                .monospace();
                                ui.add_sized(
                                    [92.0, 16.0],
                                    egui::Label::new(key).halign(egui::Align::RIGHT),
                                );
                                let label = egui::RichText::new(binding.label);
                                // A command that cannot do anything right now is dimmed
                                // rather than hidden: the list is also how you learn the
                                // key exists at all.
                                ui.label(if (binding.enabled)(editor) {
                                    label
                                } else {
                                    label.weak()
                                });
                            });
                        }
                        ui.add_space(6.0);
                    }
                });
            });
        ui.separator();
        if ui.button("Close").clicked() {
            open = false;
        }
    });
    editor.show_shortcuts = open;
}

/// A fuzzy search over the same table, so every command is reachable without hunting
/// the toolbar for it.
fn command_palette(editor: &mut Editor, ctx: &egui::Context, queue: &mut Vec<Command>) {
    let Some(mut palette) = editor.palette.take() else {
        return;
    };
    let matches = commands::search(editor, &palette.query);
    palette.selected = palette.selected.min(matches.len().saturating_sub(1));
    let mut chosen: Option<&'static commands::Binding> = None;
    let mut close = false;
    egui::Window::new("Command")
        .collapsible(false)
        .resizable(false)
        .anchor(egui::Align2::CENTER_TOP, [0.0, 90.0])
        .show(ctx, |ui| {
            let entry = ui.add(
                egui::TextEdit::singleline(&mut palette.query)
                    .hint_text("Type a command…")
                    .desired_width(380.0),
            );
            // Asked for once, on the frame it opens: requesting it every frame would
            // take the focus back from anything the palette itself puts up.
            if palette.just_opened {
                entry.request_focus();
                palette.just_opened = false;
            }
            let (down, up, enter, escape) = ui.input(|i| {
                (
                    i.key_pressed(egui::Key::ArrowDown),
                    i.key_pressed(egui::Key::ArrowUp),
                    i.key_pressed(egui::Key::Enter),
                    i.key_pressed(egui::Key::Escape),
                )
            });
            if down && !matches.is_empty() {
                palette.selected = (palette.selected + 1).min(matches.len() - 1);
            }
            if up {
                palette.selected = palette.selected.saturating_sub(1);
            }
            if escape {
                close = true;
            }
            ui.separator();
            if matches.is_empty() {
                ui.label(egui::RichText::new("Nothing matches that").weak());
            }
            for (i, binding) in matches.iter().take(PALETTE_ROWS).enumerate() {
                let runnable = (binding.enabled)(editor);
                let label = egui::RichText::new(binding.label);
                let row = ui
                    .horizontal(|ui| {
                        let hit = ui.selectable_label(
                            i == palette.selected,
                            if runnable { label } else { label.weak() },
                        );
                        if let Some(keys) = binding.shortcut_label() {
                            ui.with_layout(
                                egui::Layout::right_to_left(egui::Align::Center),
                                |ui| {
                                    ui.label(egui::RichText::new(keys).monospace().weak());
                                },
                            );
                        }
                        hit
                    })
                    .inner;
                if row.clicked() {
                    chosen = Some(binding);
                }
                if row.hovered() {
                    palette.selected = i;
                }
            }
            if enter {
                chosen = matches.get(palette.selected).copied();
            }
        });
    if let Some(binding) = chosen {
        // A command that cannot run is left alone rather than run and refused: the row
        // is already dimmed, and the palette closing on a command that did nothing reads
        // as the palette having eaten the keystroke.
        if (binding.enabled)(editor) {
            queue.push((binding.make)());
            close = true;
        }
    }
    if !close {
        editor.palette = Some(palette);
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
