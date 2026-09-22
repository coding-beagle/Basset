//! Every command the editor can run, in one table.
//!
//! A command is a thing the user can ask for — a tool, a view, a file operation — with
//! a name, the keys that run it, the mode it is live in and a test for whether it can do
//! anything right now. [`Editor::on_key`](super::Editor::on_key) is a lookup into this
//! table, the help overlay is a listing of it and the command palette is a search over
//! it, so a binding cannot exist in the handler and be missing from the help, or be
//! listed and do nothing.
//!
//! The commands themselves are the same [`Command`] values the toolbar and the menus
//! queue, run by the same [`panels::run`](super::panels::run): a key and its button are
//! one code path, not two that have to be kept saying the same thing.

use winit::keyboard::{Key, NamedKey};

use super::sketch_mode::{ConstraintKind, SketchPick, SketchTool, ToolGroup};
use super::tools::ToolKind;
use super::{DisplayMode, Editor, Mode, SelectMode};
use basset_viewport::ViewPreset;

/// Deferred commands, so panel code never needs `&mut Editor` while it borrows state.
#[derive(Clone, Debug)]
pub(crate) enum Command {
    Tool(ToolKind),
    /// Start or leave the Measure tool, which owns no feature; see [`super::measure`].
    Measure(bool),
    /// Leave it if it is running, start it if it is not: what one key has to mean.
    ToggleMeasure,
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
    Display(DisplayMode),
    /// Walk the display modes, as `D` and the View menu's list do.
    CycleDisplay,
    ToggleGrid,
    ToggleSnap,
    ToggleOrigin,
    /// Escape: put down whatever is running, or clear the selection.
    Cancel,
    /// Enter: keep whatever is running.
    Confirm,
    /// Delete the selection, whatever kind of thing it is.
    DeleteSelected,
    /// Tab: move to the next size entry box while drawing.
    FocusNextEntry,
    SetCursor(usize),
    Edit(basset_core::FeatureId),
    Suppress(basset_core::FeatureId, bool),
    Delete(basset_core::FeatureId),
    Rename(basset_core::FeatureId),
    SelectFeature(basset_core::FeatureId),
    ToggleBody(basset_core::BodyRef),
    ToggleSketch(basset_core::FeatureId),
    Activate(basset_core::ComponentId),
    FinishSketch(bool),
    SketchTool(SketchTool),
    /// The tool the toolbar's folded button for this group is showing — the variant used
    /// last. A key into the same button rather than beside it: pressing `R` draws the
    /// rectangle the Rectangle button would draw.
    SketchGroup(ToolGroup),
    SketchPick(SketchPick),
    SketchSelect(Vec<basset_sketch::EntityId>),
    SketchDimension(basset_sketch::ConstraintId, f64),
    SketchRemoveConstraint(basset_sketch::ConstraintId),
    SketchDelete,
    SketchConstruction,
    /// Start a move of the selection, as `M` does.
    SketchMoveBegin,
    /// A number in the move palette changed: re-apply the move from where it started.
    SketchMoveUpdate,
    /// Keep (`true`) or undo (`false`) the move in progress.
    SketchMoveFinish(bool),
    /// Start a pattern of the selection.
    SketchPattern,
    /// A number in the pattern palette changed: re-make the copies.
    SketchPatternUpdate,
    /// Keep (`true`) or undo (`false`) the pattern in progress.
    SketchPatternFinish(bool),
    SketchOffset,
    /// A setting in the offset palette changed: re-make the result.
    SketchOffsetUpdate,
    /// Keep (`true`) or undo (`false`) the offset in progress.
    SketchOffsetFinish(bool),
    /// The radius in the fillet palette changed: re-make the arc.
    SketchFilletUpdate,
    /// Keep (`true`) or undo (`false`) the corner fillet in progress.
    SketchFilletFinish(bool),
    /// Add the document parameter the panel's bottom row names. Distinct from
    /// [`Command::SetParameter`] only in that the row empties itself when it is taken.
    AddParameter(String, String),
    /// Add a document parameter, or give an existing one a new expression.
    SetParameter(String, String),
    /// Rename a document parameter, following the old name into every feature and every
    /// sketch that does not shadow it.
    RenameParameter(String, String),
    RemoveParameter(String),
    SketchSetParameter(String, String),
    SketchRemoveParameter(String),
    SketchBindDimension(basset_sketch::ConstraintId, String),
    /// Finish the sketch and open Extrude on the regions under the pointer.
    SketchExtrudeRegion,
    /// Enter in an entry box: place the shape from the typed sizes.
    SketchSubmitEntry,
    /// The sketch editor changed its sketch outside a pointer event (a dragged label):
    /// write it into the feature.
    SketchCommit,
    SelectMode(SelectMode),
    /// Show or hide the keyboard shortcut overlay.
    ToggleShortcuts,
    /// Open the command palette, empty and focused.
    OpenPalette,
}

/// Whether shift has to be down, has to be up, or is not part of the chord at all.
///
/// The third case is not tidiness: `Ctrl+O` has always opened a file whether or not
/// shift happened to be down, and a user still holding shift from the last chord should
/// not find the key dead.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Shift {
    Up,
    Down,
    Either,
}

/// A key, as the table names it. Characters are matched case-insensitively, because the
/// logical key winit reports for shift+f is `F` and for f is `f`, and which of the two a
/// binding wants is said by [`Shift`] rather than by the letter's case.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Stroke {
    Char(char),
    Named(NamedKey),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct Chord {
    pub stroke: Stroke,
    pub ctrl: bool,
    pub shift: Shift,
}

impl Chord {
    const fn plain(c: char) -> Self {
        Chord {
            stroke: Stroke::Char(c),
            ctrl: false,
            shift: Shift::Up,
        }
    }

    const fn shifted(c: char) -> Self {
        Chord {
            stroke: Stroke::Char(c),
            ctrl: false,
            shift: Shift::Down,
        }
    }

    const fn ctrl(c: char) -> Self {
        Chord {
            stroke: Stroke::Char(c),
            ctrl: true,
            shift: Shift::Up,
        }
    }

    const fn ctrl_shift(c: char) -> Self {
        Chord {
            stroke: Stroke::Char(c),
            ctrl: true,
            shift: Shift::Down,
        }
    }

    /// `Ctrl+<c>` whether or not shift is down. See [`Shift::Either`].
    const fn ctrl_any(c: char) -> Self {
        Chord {
            stroke: Stroke::Char(c),
            ctrl: true,
            shift: Shift::Either,
        }
    }

    const fn named(key: NamedKey) -> Self {
        Chord {
            stroke: Stroke::Named(key),
            ctrl: false,
            shift: Shift::Either,
        }
    }

    /// Whether the key winit reported, with these modifiers, is this chord.
    pub fn matches(&self, key: &Key, ctrl: bool, shift: bool) -> bool {
        if self.ctrl != ctrl {
            return false;
        }
        match self.shift {
            Shift::Up if shift => return false,
            Shift::Down if !shift => return false,
            _ => {}
        }
        match (self.stroke, key) {
            (Stroke::Named(want), Key::Named(got)) => want == *got,
            (Stroke::Char(want), Key::Character(got)) => {
                let mut chars = got.chars();
                match (chars.next(), chars.next()) {
                    (Some(c), None) => c.to_ascii_lowercase() == want,
                    _ => false,
                }
            }
            _ => false,
        }
    }

    /// How the chord is written in a menu, a tooltip or the help overlay.
    pub fn label(&self) -> String {
        let mut s = String::new();
        if self.ctrl {
            s.push_str("Ctrl+");
        }
        if self.shift == Shift::Down {
            s.push_str("Shift+");
        }
        match self.stroke {
            Stroke::Char(c) => s.push(c.to_ascii_uppercase()),
            Stroke::Named(k) => s.push_str(named_label(k)),
        }
        s
    }
}

fn named_label(key: NamedKey) -> &'static str {
    match key {
        NamedKey::Escape => "Esc",
        NamedKey::Enter => "Enter",
        NamedKey::Tab => "Tab",
        NamedKey::Delete => "Del",
        NamedKey::Backspace => "Backspace",
        NamedKey::F1 => "F1",
        // Nothing else is bound to a named key; a new one shows as its debug name rather
        // than as a lie.
        _ => "(key)",
    }
}

/// Which mode a command is live in. A key means different things in the two modes — `R`
/// draws a rectangle in a sketch and revolves a profile outside one — so the mode is
/// part of the binding rather than a test buried in the handler.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum LiveIn {
    Model,
    Sketch,
    Both,
}

impl LiveIn {
    pub fn covers(self, sketching: bool) -> bool {
        match self {
            LiveIn::Both => true,
            LiveIn::Sketch => sketching,
            LiveIn::Model => !sketching,
        }
    }
}

/// How the help overlay and the palette group what they list.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Group {
    File,
    Edit,
    View,
    Select,
    Create,
    Modify,
    Shape,
    Constrain,
    Sketch,
    Help,
}

impl Group {
    /// In the order the overlay lists them.
    pub const ALL: [Group; 10] = [
        Group::File,
        Group::Edit,
        Group::View,
        Group::Select,
        Group::Create,
        Group::Modify,
        Group::Shape,
        Group::Constrain,
        Group::Sketch,
        Group::Help,
    ];

    pub fn name(self) -> &'static str {
        match self {
            Group::File => "File",
            Group::Edit => "Edit",
            Group::View => "View",
            Group::Select => "Select",
            Group::Create => "Create",
            Group::Modify => "Modify",
            Group::Shape => "Sketch tools",
            Group::Constrain => "Sketch constraints",
            Group::Sketch => "Sketch",
            Group::Help => "Help",
        }
    }
}

/// One command: what it is called, what runs it, and when.
pub(crate) struct Binding {
    /// Stable name, for tests and for searching the palette by something other than the
    /// label's wording.
    pub id: &'static str,
    pub label: &'static str,
    pub group: Group,
    /// Every chord that runs it. The first is the one shown; the rest are the aliases
    /// fingers already have (`Backspace` for `Del`, `Ctrl+Y` for redo).
    pub chords: &'static [Chord],
    pub live: LiveIn,
    /// Built rather than stored, because a command may carry a `String` or a `Vec` and
    /// so cannot sit in a `const` table.
    pub make: fn() -> Command,
    /// Whether running it now would do anything. Shown by dimming in the overlay and the
    /// palette; the command itself still explains its own refusals, which is where the
    /// message that says what to select first comes from.
    pub enabled: fn(&Editor) -> bool,
}

impl Binding {
    /// The chord shown beside the command's name.
    pub fn chord(&self) -> Option<Chord> {
        self.chords.first().copied()
    }

    pub fn shortcut_label(&self) -> Option<String> {
        self.chord().map(|c| c.label())
    }
}

const ALWAYS: fn(&Editor) -> bool = |_| true;
/// A modelling tool cannot start while another one's dialog is up, exactly as the
/// toolbar greys out while one is running.
const IDLE: fn(&Editor) -> bool = |e| e.tool.is_none();
/// Sketch operations that act on a selection, and cannot start on top of one another.
const HAS_SKETCH_SELECTION: fn(&Editor) -> bool = |e| match &e.mode {
    Mode::Sketch(s) => !s.selected.is_empty() && !s.modal(),
    Mode::Model => false,
};

/// Everything the editor can be asked to do.
///
/// Keys follow Fusion where Fusion has one and it is free here: `L` line, `R` rectangle,
/// `C` circle, `A` arc, `T` trim, `X` construction, `M` move, `O` offset, `E` extrude,
/// `I` measure. `F` has fitted the view and `D` walked the display modes since before
/// any of the tools had keys, so Fillet takes `Shift+F` and Dimension — Fusion's `D` —
/// takes `Shift+D`. Constraints take `Shift` and a letter of their own name.
pub(crate) const BINDINGS: &[Binding] = &[
    // --- File ---
    Binding {
        id: "file.new",
        label: "New document",
        group: Group::File,
        chords: &[Chord::ctrl_any('n')],
        live: LiveIn::Both,
        make: || Command::New,
        enabled: ALWAYS,
    },
    Binding {
        id: "file.open",
        label: "Open…",
        group: Group::File,
        chords: &[Chord::ctrl_any('o')],
        live: LiveIn::Both,
        make: || Command::Open,
        enabled: ALWAYS,
    },
    Binding {
        id: "file.save",
        label: "Save",
        group: Group::File,
        chords: &[Chord::ctrl('s')],
        live: LiveIn::Both,
        make: || Command::Save(false),
        enabled: ALWAYS,
    },
    Binding {
        id: "file.save_as",
        label: "Save As…",
        group: Group::File,
        chords: &[Chord::ctrl_shift('s')],
        live: LiveIn::Both,
        make: || Command::Save(true),
        enabled: ALWAYS,
    },
    Binding {
        id: "file.export_stl",
        label: "Export STL…",
        group: Group::File,
        chords: &[],
        live: LiveIn::Both,
        make: || Command::ExportStl,
        enabled: ALWAYS,
    },
    Binding {
        id: "file.export_3mf",
        label: "Export 3MF…",
        group: Group::File,
        chords: &[],
        live: LiveIn::Both,
        make: || Command::Export3mf,
        enabled: ALWAYS,
    },
    Binding {
        id: "file.quit",
        label: "Quit",
        group: Group::File,
        chords: &[],
        live: LiveIn::Both,
        make: || Command::Quit,
        enabled: ALWAYS,
    },
    // --- Edit ---
    Binding {
        id: "edit.undo",
        label: "Undo",
        group: Group::Edit,
        chords: &[Chord::ctrl('z')],
        live: LiveIn::Both,
        make: || Command::Undo,
        // A sketch keeps its own history, which the document's stack knows nothing of.
        enabled: |e| e.is_sketching() || e.doc.can_undo(),
    },
    Binding {
        id: "edit.redo",
        label: "Redo",
        group: Group::Edit,
        chords: &[Chord::ctrl_shift('z'), Chord::ctrl_any('y')],
        live: LiveIn::Both,
        make: || Command::Redo,
        enabled: |e| e.is_sketching() || e.doc.can_redo(),
    },
    Binding {
        id: "edit.cancel",
        label: "Cancel, or put the tool down",
        group: Group::Edit,
        chords: &[Chord::named(NamedKey::Escape)],
        live: LiveIn::Both,
        make: || Command::Cancel,
        enabled: ALWAYS,
    },
    Binding {
        id: "edit.confirm",
        label: "Confirm",
        group: Group::Edit,
        chords: &[Chord::named(NamedKey::Enter)],
        live: LiveIn::Both,
        make: || Command::Confirm,
        enabled: ALWAYS,
    },
    Binding {
        id: "edit.delete",
        label: "Delete selection",
        group: Group::Edit,
        chords: &[
            Chord::named(NamedKey::Delete),
            Chord::named(NamedKey::Backspace),
        ],
        live: LiveIn::Both,
        make: || Command::DeleteSelected,
        enabled: ALWAYS,
    },
    Binding {
        id: "edit.next_entry",
        label: "Next size entry box",
        group: Group::Edit,
        chords: &[Chord::named(NamedKey::Tab)],
        live: LiveIn::Sketch,
        make: || Command::FocusNextEntry,
        enabled: ALWAYS,
    },
    // --- View ---
    Binding {
        id: "view.fit",
        label: "Fit the model in the view",
        group: Group::View,
        chords: &[Chord::plain('f')],
        live: LiveIn::Both,
        make: || Command::Fit,
        enabled: ALWAYS,
    },
    Binding {
        id: "view.display",
        label: "Next display mode",
        group: Group::View,
        chords: &[Chord::plain('d')],
        live: LiveIn::Both,
        make: || Command::CycleDisplay,
        enabled: ALWAYS,
    },
    Binding {
        id: "view.isometric",
        label: "Isometric view",
        group: Group::View,
        chords: &[],
        live: LiveIn::Both,
        make: || Command::View(ViewPreset::Isometric),
        enabled: ALWAYS,
    },
    Binding {
        id: "view.top",
        label: "Top view",
        group: Group::View,
        chords: &[],
        live: LiveIn::Both,
        make: || Command::View(ViewPreset::Top),
        enabled: ALWAYS,
    },
    Binding {
        id: "view.front",
        label: "Front view",
        group: Group::View,
        chords: &[],
        live: LiveIn::Both,
        make: || Command::View(ViewPreset::Front),
        enabled: ALWAYS,
    },
    Binding {
        id: "view.right",
        label: "Right view",
        group: Group::View,
        chords: &[],
        live: LiveIn::Both,
        make: || Command::View(ViewPreset::Right),
        enabled: ALWAYS,
    },
    Binding {
        id: "view.projection",
        label: "Toggle orthographic",
        group: Group::View,
        chords: &[],
        live: LiveIn::Both,
        make: || Command::ToggleProjection,
        enabled: ALWAYS,
    },
    Binding {
        id: "view.grid",
        label: "Show grid",
        group: Group::View,
        chords: &[],
        live: LiveIn::Both,
        make: || Command::ToggleGrid,
        enabled: ALWAYS,
    },
    Binding {
        id: "view.snap",
        label: "Snap to grid",
        group: Group::View,
        chords: &[],
        live: LiveIn::Both,
        make: || Command::ToggleSnap,
        enabled: ALWAYS,
    },
    Binding {
        id: "view.origin",
        label: "Show origin planes and axes",
        group: Group::View,
        chords: &[],
        live: LiveIn::Both,
        make: || Command::ToggleOrigin,
        enabled: ALWAYS,
    },
    // --- Selection filters. Two sets on the same digits, because a sketch and a model
    // have different kinds of thing to narrow a click down to.
    Binding {
        id: "select.model.all",
        label: "Select: anything",
        group: Group::Select,
        chords: &[Chord::plain('1')],
        live: LiveIn::Model,
        make: || Command::SelectMode(SelectMode::ALL[0]),
        enabled: ALWAYS,
    },
    Binding {
        id: "select.model.faces",
        label: "Select: faces",
        group: Group::Select,
        chords: &[Chord::plain('2')],
        live: LiveIn::Model,
        make: || Command::SelectMode(SelectMode::ALL[1]),
        enabled: ALWAYS,
    },
    Binding {
        id: "select.model.edges",
        label: "Select: edges",
        group: Group::Select,
        chords: &[Chord::plain('3')],
        live: LiveIn::Model,
        make: || Command::SelectMode(SelectMode::ALL[2]),
        enabled: ALWAYS,
    },
    Binding {
        id: "select.model.vertices",
        label: "Select: vertices",
        group: Group::Select,
        chords: &[Chord::plain('4')],
        live: LiveIn::Model,
        make: || Command::SelectMode(SelectMode::ALL[3]),
        enabled: ALWAYS,
    },
    Binding {
        id: "select.model.sketch",
        label: "Select: sketch geometry",
        group: Group::Select,
        chords: &[Chord::plain('5')],
        live: LiveIn::Model,
        make: || Command::SelectMode(SelectMode::ALL[4]),
        enabled: ALWAYS,
    },
    Binding {
        id: "select.sketch.all",
        label: "Select: anything",
        group: Group::Select,
        chords: &[Chord::plain('1')],
        live: LiveIn::Sketch,
        make: || Command::SketchPick(SketchPick::ALL[0]),
        enabled: ALWAYS,
    },
    Binding {
        id: "select.sketch.curves",
        label: "Select: curves",
        group: Group::Select,
        chords: &[Chord::plain('2')],
        live: LiveIn::Sketch,
        make: || Command::SketchPick(SketchPick::ALL[1]),
        enabled: ALWAYS,
    },
    Binding {
        id: "select.sketch.points",
        label: "Select: points",
        group: Group::Select,
        chords: &[Chord::plain('3')],
        live: LiveIn::Sketch,
        make: || Command::SketchPick(SketchPick::ALL[2]),
        enabled: ALWAYS,
    },
    Binding {
        id: "select.sketch.regions",
        label: "Select: regions",
        group: Group::Select,
        chords: &[Chord::plain('4')],
        live: LiveIn::Sketch,
        make: || Command::SketchPick(SketchPick::ALL[3]),
        enabled: ALWAYS,
    },
    // --- Modelling tools ---
    Binding {
        id: "create.sketch",
        label: "Create Sketch",
        group: Group::Create,
        chords: &[Chord::plain('s')],
        live: LiveIn::Model,
        make: || Command::Tool(ToolKind::Sketch),
        enabled: IDLE,
    },
    Binding {
        id: "create.extrude",
        label: "Extrude",
        group: Group::Create,
        chords: &[Chord::plain('e')],
        live: LiveIn::Model,
        make: || Command::Tool(ToolKind::Extrude),
        enabled: IDLE,
    },
    Binding {
        id: "create.revolve",
        label: "Revolve",
        group: Group::Create,
        chords: &[Chord::plain('r')],
        live: LiveIn::Model,
        make: || Command::Tool(ToolKind::Revolve),
        enabled: IDLE,
    },
    Binding {
        id: "create.sweep",
        label: "Sweep",
        group: Group::Create,
        chords: &[Chord::plain('w')],
        live: LiveIn::Model,
        make: || Command::Tool(ToolKind::Sweep),
        enabled: IDLE,
    },
    Binding {
        id: "create.loft",
        label: "Loft",
        group: Group::Create,
        chords: &[Chord::plain('l')],
        live: LiveIn::Model,
        make: || Command::Tool(ToolKind::Loft),
        enabled: IDLE,
    },
    Binding {
        id: "create.offset_plane",
        label: "Offset Plane",
        group: Group::Create,
        chords: &[Chord::plain('p')],
        live: LiveIn::Model,
        make: || Command::Tool(ToolKind::OffsetPlane),
        enabled: IDLE,
    },
    Binding {
        id: "create.angled_plane",
        label: "Plane at Angle",
        group: Group::Create,
        chords: &[Chord::shifted('p')],
        live: LiveIn::Model,
        make: || Command::Tool(ToolKind::AngledPlane),
        enabled: IDLE,
    },
    Binding {
        id: "create.component",
        label: "New Component",
        group: Group::Create,
        chords: &[Chord::shifted('n')],
        live: LiveIn::Model,
        make: || Command::Tool(ToolKind::Component),
        enabled: IDLE,
    },
    Binding {
        id: "modify.fillet",
        label: "Fillet",
        group: Group::Modify,
        chords: &[Chord::shifted('f')],
        live: LiveIn::Model,
        make: || Command::Tool(ToolKind::Fillet),
        enabled: IDLE,
    },
    Binding {
        id: "modify.chamfer",
        label: "Chamfer",
        group: Group::Modify,
        chords: &[Chord::shifted('c')],
        live: LiveIn::Model,
        make: || Command::Tool(ToolKind::Chamfer),
        enabled: IDLE,
    },
    Binding {
        id: "modify.combine",
        label: "Combine",
        group: Group::Modify,
        chords: &[Chord::plain('b')],
        live: LiveIn::Model,
        make: || Command::Tool(ToolKind::Combine),
        enabled: IDLE,
    },
    Binding {
        id: "modify.move",
        label: "Move",
        group: Group::Modify,
        chords: &[Chord::plain('m')],
        live: LiveIn::Model,
        make: || Command::Tool(ToolKind::Move),
        enabled: IDLE,
    },
    Binding {
        id: "modify.measure",
        label: "Measure",
        group: Group::Modify,
        chords: &[Chord::plain('i')],
        live: LiveIn::Model,
        make: || Command::ToggleMeasure,
        enabled: IDLE,
    },
    // --- Sketch tools. Each key opens the tool the toolbar's button for that shape is
    // showing, so the key and the button draw the same thing.
    Binding {
        id: "sketch.line",
        label: "Line",
        group: Group::Shape,
        chords: &[Chord::plain('l')],
        live: LiveIn::Sketch,
        make: || Command::SketchGroup(ToolGroup::Line),
        enabled: ALWAYS,
    },
    Binding {
        id: "sketch.rectangle",
        label: "Rectangle",
        group: Group::Shape,
        chords: &[Chord::plain('r')],
        live: LiveIn::Sketch,
        make: || Command::SketchGroup(ToolGroup::Rectangle),
        enabled: ALWAYS,
    },
    Binding {
        id: "sketch.circle",
        label: "Circle",
        group: Group::Shape,
        chords: &[Chord::plain('c')],
        live: LiveIn::Sketch,
        make: || Command::SketchGroup(ToolGroup::Circle),
        enabled: ALWAYS,
    },
    Binding {
        id: "sketch.arc",
        label: "Arc",
        group: Group::Shape,
        chords: &[Chord::plain('a')],
        live: LiveIn::Sketch,
        make: || Command::SketchGroup(ToolGroup::Arc),
        enabled: ALWAYS,
    },
    Binding {
        id: "sketch.polygon",
        label: "Polygon",
        group: Group::Shape,
        chords: &[Chord::plain('p')],
        live: LiveIn::Sketch,
        make: || Command::SketchGroup(ToolGroup::Polygon),
        enabled: ALWAYS,
    },
    Binding {
        id: "sketch.slot",
        label: "Slot",
        group: Group::Shape,
        chords: &[Chord::plain('s')],
        live: LiveIn::Sketch,
        make: || Command::SketchGroup(ToolGroup::Slot),
        enabled: ALWAYS,
    },
    Binding {
        id: "sketch.text",
        label: "Text",
        group: Group::Shape,
        chords: &[],
        live: LiveIn::Sketch,
        make: || Command::SketchGroup(ToolGroup::Text),
        enabled: ALWAYS,
    },
    Binding {
        id: "sketch.dimension",
        label: "Dimension",
        group: Group::Shape,
        chords: &[Chord::shifted('d')],
        live: LiveIn::Sketch,
        make: || Command::SketchGroup(ToolGroup::Dimension),
        enabled: ALWAYS,
    },
    // Trim's button folds Break and Fillet under it, but each of the three has its own
    // key: a `T` that sometimes broke instead of trimming is worse than no key at all.
    Binding {
        id: "sketch.trim",
        label: "Trim",
        group: Group::Shape,
        chords: &[Chord::plain('t')],
        live: LiveIn::Sketch,
        make: || Command::SketchTool(SketchTool::Trim),
        enabled: ALWAYS,
    },
    Binding {
        id: "sketch.break",
        label: "Break",
        group: Group::Shape,
        chords: &[Chord::plain('b')],
        live: LiveIn::Sketch,
        make: || Command::SketchTool(SketchTool::Break),
        enabled: ALWAYS,
    },
    Binding {
        id: "sketch.fillet",
        label: "Fillet corner",
        group: Group::Shape,
        chords: &[Chord::shifted('g')],
        live: LiveIn::Sketch,
        make: || Command::SketchTool(SketchTool::Fillet),
        enabled: ALWAYS,
    },
    Binding {
        id: "sketch.select",
        label: "Select tool",
        group: Group::Shape,
        chords: &[],
        live: LiveIn::Sketch,
        make: || Command::SketchGroup(ToolGroup::Select),
        enabled: ALWAYS,
    },
    // --- Sketch constraints ---
    Binding {
        id: "constrain.horizontal",
        label: "Horizontal",
        group: Group::Constrain,
        chords: &[Chord::plain('h')],
        live: LiveIn::Sketch,
        make: || Command::SketchTool(SketchTool::Constrain(ConstraintKind::Horizontal)),
        enabled: ALWAYS,
    },
    Binding {
        id: "constrain.vertical",
        label: "Vertical",
        group: Group::Constrain,
        chords: &[Chord::plain('v')],
        live: LiveIn::Sketch,
        make: || Command::SketchTool(SketchTool::Constrain(ConstraintKind::Vertical)),
        enabled: ALWAYS,
    },
    Binding {
        id: "constrain.coincident",
        label: "Coincident",
        group: Group::Constrain,
        chords: &[Chord::shifted('c')],
        live: LiveIn::Sketch,
        make: || Command::SketchTool(SketchTool::Constrain(ConstraintKind::Coincident)),
        enabled: ALWAYS,
    },
    Binding {
        id: "constrain.parallel",
        label: "Parallel",
        group: Group::Constrain,
        chords: &[Chord::shifted('p')],
        live: LiveIn::Sketch,
        make: || Command::SketchTool(SketchTool::Constrain(ConstraintKind::Parallel)),
        enabled: ALWAYS,
    },
    Binding {
        id: "constrain.perpendicular",
        label: "Perpendicular",
        group: Group::Constrain,
        chords: &[Chord::shifted('r')],
        live: LiveIn::Sketch,
        make: || Command::SketchTool(SketchTool::Constrain(ConstraintKind::Perpendicular)),
        enabled: ALWAYS,
    },
    Binding {
        id: "constrain.tangent",
        label: "Tangent",
        group: Group::Constrain,
        chords: &[Chord::shifted('t')],
        live: LiveIn::Sketch,
        make: || Command::SketchTool(SketchTool::Constrain(ConstraintKind::Tangent)),
        enabled: ALWAYS,
    },
    Binding {
        id: "constrain.equal",
        label: "Equal",
        group: Group::Constrain,
        chords: &[Chord::shifted('e')],
        live: LiveIn::Sketch,
        make: || Command::SketchTool(SketchTool::Constrain(ConstraintKind::Equal)),
        enabled: ALWAYS,
    },
    Binding {
        id: "constrain.concentric",
        label: "Concentric",
        group: Group::Constrain,
        chords: &[Chord::shifted('n')],
        live: LiveIn::Sketch,
        make: || Command::SketchTool(SketchTool::Constrain(ConstraintKind::Concentric)),
        enabled: ALWAYS,
    },
    Binding {
        id: "constrain.midpoint",
        label: "Midpoint",
        group: Group::Constrain,
        chords: &[Chord::shifted('m')],
        live: LiveIn::Sketch,
        make: || Command::SketchTool(SketchTool::Constrain(ConstraintKind::Midpoint)),
        enabled: ALWAYS,
    },
    Binding {
        id: "constrain.symmetric",
        label: "Symmetric",
        group: Group::Constrain,
        chords: &[Chord::shifted('s')],
        live: LiveIn::Sketch,
        make: || Command::SketchTool(SketchTool::Constrain(ConstraintKind::Symmetric)),
        enabled: ALWAYS,
    },
    Binding {
        id: "constrain.fix",
        label: "Fix",
        group: Group::Constrain,
        chords: &[Chord::shifted('f')],
        live: LiveIn::Sketch,
        make: || Command::SketchTool(SketchTool::Constrain(ConstraintKind::Fix)),
        enabled: ALWAYS,
    },
    // --- Sketch operations ---
    Binding {
        id: "sketch.construction",
        label: "Construction geometry",
        group: Group::Sketch,
        chords: &[Chord::plain('x')],
        live: LiveIn::Sketch,
        make: || Command::SketchConstruction,
        enabled: ALWAYS,
    },
    Binding {
        id: "sketch.move",
        label: "Move the selection",
        group: Group::Sketch,
        chords: &[Chord::plain('m')],
        live: LiveIn::Sketch,
        make: || Command::SketchMoveBegin,
        enabled: HAS_SKETCH_SELECTION,
    },
    Binding {
        id: "sketch.offset",
        label: "Offset the selection",
        group: Group::Sketch,
        chords: &[Chord::plain('o')],
        live: LiveIn::Sketch,
        make: || Command::SketchOffset,
        enabled: HAS_SKETCH_SELECTION,
    },
    Binding {
        id: "sketch.pattern",
        label: "Pattern the selection",
        group: Group::Sketch,
        chords: &[],
        live: LiveIn::Sketch,
        make: || Command::SketchPattern,
        enabled: HAS_SKETCH_SELECTION,
    },
    Binding {
        id: "sketch.extrude",
        label: "Extrude the region under the pointer",
        group: Group::Sketch,
        chords: &[Chord::plain('e')],
        live: LiveIn::Sketch,
        make: || Command::SketchExtrudeRegion,
        enabled: ALWAYS,
    },
    Binding {
        id: "sketch.finish",
        label: "Finish Sketch",
        group: Group::Sketch,
        chords: &[],
        live: LiveIn::Sketch,
        make: || Command::FinishSketch(true),
        enabled: ALWAYS,
    },
    Binding {
        id: "sketch.abandon",
        label: "Cancel Sketch",
        group: Group::Sketch,
        chords: &[],
        live: LiveIn::Sketch,
        make: || Command::FinishSketch(false),
        enabled: ALWAYS,
    },
    // --- Help ---
    Binding {
        id: "help.shortcuts",
        label: "Keyboard shortcuts",
        group: Group::Help,
        chords: &[
            Chord {
                stroke: Stroke::Char('?'),
                ctrl: false,
                shift: Shift::Either,
            },
            Chord::named(NamedKey::F1),
        ],
        live: LiveIn::Both,
        make: || Command::ToggleShortcuts,
        enabled: ALWAYS,
    },
    Binding {
        id: "help.palette",
        label: "Command palette",
        group: Group::Help,
        chords: &[Chord::ctrl('p'), Chord::ctrl_shift('p')],
        live: LiveIn::Both,
        make: || Command::OpenPalette,
        enabled: ALWAYS,
    },
];

/// The command palette while it is open: what has been typed and which of the matches
/// is highlighted. The matches themselves are not kept, because they are a function of
/// the query and of the editor, both of which can change under it.
#[derive(Clone, Debug, Default)]
pub(crate) struct Palette {
    pub query: String,
    pub selected: usize,
    /// Set for the one frame after it opens, so the text box can take the focus without
    /// stealing it back every frame afterwards.
    pub just_opened: bool,
}

/// The binding a keystroke runs, or `None` when the key means nothing in this mode.
pub(crate) fn lookup(
    key: &Key,
    ctrl: bool,
    shift: bool,
    sketching: bool,
) -> Option<&'static Binding> {
    BINDINGS
        .iter()
        .find(|b| b.live.covers(sketching) && b.chords.iter().any(|c| c.matches(key, ctrl, shift)))
}

/// The binding for a command id, for menus and tooltips that want to print its key.
pub(crate) fn binding(id: &str) -> Option<&'static Binding> {
    BINDINGS.iter().find(|b| b.id == id)
}

/// The key printed after a menu item or a tooltip, as ` (Ctrl+S)`, or nothing when the
/// command has no key. Callers append it to their own label.
pub(crate) fn hint(id: &str) -> String {
    suffix(binding(id))
}

fn suffix(binding: Option<&'static Binding>) -> String {
    match binding.and_then(|b| b.shortcut_label()) {
        Some(k) => format!(" ({k})"),
        None => String::new(),
    }
}

/// The binding whose command is this one, found by asking each one what it makes. The
/// toolbar knows the tool it is drawing a button for, not the command id, and looking
/// the key up by what the button does is what keeps the two from drifting apart.
fn made_by(predicate: impl Fn(&Command) -> bool) -> Option<&'static Binding> {
    BINDINGS.iter().find(|b| predicate(&(b.make)()))
}

/// The key beside a modelling tool's button or menu item.
pub(crate) fn tool_hint(kind: ToolKind) -> String {
    suffix(made_by(|c| matches!(c, Command::Tool(k) if *k == kind)))
}

/// The key beside a sketch toolbar button: its group's key, or the tool's own where the
/// tool is bound directly (Trim, Break, the corner fillet).
pub(crate) fn sketch_tool_hint(group: ToolGroup, tool: SketchTool) -> String {
    let found = made_by(|c| matches!(c, Command::SketchTool(t) if *t == tool))
        .or_else(|| made_by(|c| matches!(c, Command::SketchGroup(g) if *g == group)));
    suffix(found)
}

/// The key beside a constraint button.
pub(crate) fn constraint_hint(kind: ConstraintKind) -> String {
    suffix(made_by(
        |c| matches!(c, Command::SketchTool(SketchTool::Constrain(k)) if *k == kind),
    ))
}

/// Score a command against what has been typed in the palette: the letters have to
/// appear in order, and a run of them, or one starting a word, scores above the same
/// letters scattered. Higher is better; `None` means no match.
///
/// The label and the id are both searched, so "extrude" finds Extrude by its name and
/// "crex" finds it through `create.extrude`.
pub(crate) fn score(binding: &Binding, query: &str) -> Option<i32> {
    let query = query.trim().to_ascii_lowercase();
    if query.is_empty() {
        return Some(0);
    }
    let label = subsequence_score(&binding.label.to_ascii_lowercase(), &query);
    // The id is a fallback rather than a second name, so a hit on it ranks below a hit
    // on the words the user can actually see.
    let id = subsequence_score(&binding.id.to_ascii_lowercase(), &query).map(|s| s - 5);
    match (label, id) {
        (Some(a), Some(b)) => Some(a.max(b)),
        (a, b) => a.or(b),
    }
}

fn subsequence_score(haystack: &str, needle: &str) -> Option<i32> {
    let hay: Vec<char> = haystack.chars().collect();
    let mut score = 0;
    let mut at = 0usize;
    let mut previous: Option<usize> = None;
    for want in needle.chars() {
        if want == ' ' {
            continue;
        }
        let found = hay.get(at..)?.iter().position(|c| *c == want)? + at;
        let starts_word = found == 0 || matches!(hay[found - 1], ' ' | '.' | '-' | '/');
        if previous.is_some_and(|p| p + 1 == found) {
            score += 4;
        }
        if starts_word {
            score += 3;
        }
        score += 1;
        previous = Some(found);
        at = found + 1;
    }
    // Of two names matched by the same letters the shorter is the better answer.
    Some(score - (hay.len() as i32) / 8)
}

/// What the palette lists: everything live in this mode that matches, best first, ties
/// broken by the table's own order so the list does not reshuffle under the pointer.
pub(crate) fn search(editor: &Editor, query: &str) -> Vec<&'static Binding> {
    let sketching = editor.is_sketching();
    let mut hits: Vec<(i32, usize, &'static Binding)> = BINDINGS
        .iter()
        .enumerate()
        .filter(|(_, b)| b.live.covers(sketching))
        .filter_map(|(i, b)| score(b, query).map(|s| (s, i, b)))
        .collect();
    hits.sort_by(|a, b| b.0.cmp(&a.0).then(a.1.cmp(&b.1)));
    hits.into_iter().map(|(_, _, b)| b).collect()
}
