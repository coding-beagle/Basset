//! Modelling tool dialogs.
//!
//! A tool owns one timeline feature. The feature is created the moment the dialog has
//! enough input to build it and is re-edited on every parameter change, so the viewport
//! is always a live preview of exactly what OK will keep. The document transaction makes
//! the whole interaction a single undo step, and Cancel is a rollback.

use std::collections::BTreeMap;

use basset_core::{
    AxisRef, BodyOp, BodyRef, CombineOp, EdgeRef, Extent, FeatureId, FeatureKind, FeatureStatus,
    NumericField, OriginAxis, Parameters, PathRef, PlaneRef, RegionRef,
};
use basset_math::{Aabb, Affine3, Quat, Vec3};

use super::selection::{Pick, Selection, SelectionFilter};
use super::snap::Hint;
use super::{Editor, sketch_mode};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ToolKind {
    Sketch,
    Extrude,
    Revolve,
    Sweep,
    Loft,
    Fillet,
    Chamfer,
    Combine,
    Move,
    OffsetPlane,
    AngledPlane,
    Component,
}

impl ToolKind {
    pub fn title(self) -> &'static str {
        match self {
            ToolKind::Sketch => "Create Sketch",
            ToolKind::Extrude => "Extrude",
            ToolKind::Revolve => "Revolve",
            ToolKind::Sweep => "Sweep",
            ToolKind::Loft => "Loft",
            ToolKind::Fillet => "Fillet",
            ToolKind::Chamfer => "Chamfer",
            ToolKind::Combine => "Combine",
            ToolKind::Move => "Move",
            ToolKind::OffsetPlane => "Offset Plane",
            ToolKind::AngledPlane => "Plane at Angle",
            ToolKind::Component => "New Component",
        }
    }

    fn prompt(self) -> &'static str {
        match self {
            ToolKind::Sketch => "Select a plane or a planar face to sketch on",
            ToolKind::Extrude => "Select sketch regions or planar faces to extrude",
            ToolKind::Revolve => "Select regions, then an axis (origin axis or sketch line)",
            ToolKind::Sweep => "Select a region, then the sketch curves of the path in order",
            ToolKind::Loft => "Select two or more regions in order",
            ToolKind::Fillet => "Select edges to round",
            ToolKind::Chamfer => "Select edges to chamfer",
            ToolKind::Combine => "Select the target body first, then tool bodies",
            ToolKind::Move => "Select a body to move",
            ToolKind::OffsetPlane => "Select the base plane or face",
            ToolKind::AngledPlane => "Select the base plane, then an axis",
            ToolKind::Component => "Name the component",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ExtentKind {
    OneSide,
    Symmetric,
    TwoSides,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OpKind {
    NewBody,
    Join,
    Cut,
    Intersect,
}

/// Every parameter any tool has. One flat struct keeps the dialog code trivial; unused
/// fields are simply ignored by tools that do not read them.
#[derive(Clone, Debug)]
pub struct Params {
    pub distance: f64,
    pub extent: ExtentKind,
    pub negative: f64,
    pub angle_deg: f64,
    pub radius: f64,
    pub op: OpKind,
    pub target: Option<BodyRef>,
    pub combine: CombineOp,
    pub keep_tools: bool,
    pub translate: Vec3,
    pub rotate_deg: Vec3,
    pub axis: Option<AxisRef>,
    pub name: String,
    /// Fillet and Chamfer: one pick takes the whole tangentially continuous run of
    /// edges, as Fusion's tangent chain does. Ctrl-clicking overrides it for one pick.
    pub tangent_chain: bool,
}

impl Default for Params {
    fn default() -> Self {
        Self {
            distance: 10.0,
            extent: ExtentKind::OneSide,
            negative: 0.0,
            angle_deg: 360.0,
            radius: 1.0,
            op: OpKind::NewBody,
            target: None,
            combine: CombineOp::Join,
            keep_tools: false,
            translate: Vec3::ZERO,
            rotate_deg: Vec3::ZERO,
            axis: None,
            name: "Component".into(),
            tangent_chain: true,
        }
    }
}

/// Every field a dialog can offer, so a tool that stops offering one can take back the
/// expression that was driving it.
const FIELDS: [NumericField; 4] = [
    NumericField::Distance,
    NumericField::Negative,
    NumericField::Angle,
    NumericField::Radius,
];

/// The expressions driving a running tool's numbers.
///
/// Held by the tool rather than in [`Params`] because an expression belongs to the
/// *feature*: `Params` is the flat bag of numbers every dialog shares, and the number is
/// what the expression works out to, not what the user stated. These are written onto the
/// feature on every sync, so they are part of the tool's transaction and a Cancel takes
/// them back with everything else.
#[derive(Clone, Debug, Default)]
pub struct FieldExprs {
    /// Text being edited, one entry per field whose toggle is on. Separate from the live
    /// expressions so a half-typed name does not un-drive the field on every keystroke.
    drafts: BTreeMap<NumericField, String>,
    /// The expressions that last evaluated: what actually drives the feature.
    live: BTreeMap<NumericField, String>,
    /// Which field was refused and why, for the dialog to show beside it. A refusal has
    /// to be visible where the user typed, not swallowed or sent to a popup over the
    /// dialog they are still working in.
    error: Option<(NumericField, String)>,
}

impl FieldExprs {
    /// The expression driving a field, if one does.
    #[cfg(test)]
    pub fn get(&self, field: NumericField) -> Option<&str> {
        self.live.get(&field).map(String::as_str)
    }

    /// Takes what a field's box says: if it works out, the field is driven by it and the
    /// number becomes its result.
    ///
    /// Returns whether the value moved. A refusal is kept for the dialog to show and the
    /// field keeps whatever drove it before, so a typo never un-drives a feature that was
    /// working.
    fn commit(
        &mut self,
        field: NumericField,
        parameters: &Parameters,
        text: &str,
        value: &mut f64,
    ) -> bool {
        match parameters.evaluate(text) {
            Ok(v) => {
                *value = v;
                self.live.insert(field, text.to_string());
                self.clear_error(field);
                true
            }
            Err(e) => {
                self.error = Some((field, e.to_string()));
                false
            }
        }
    }

    fn is_open(&self, field: NumericField) -> bool {
        self.drafts.contains_key(&field)
    }

    /// Opens the expression box for a field, starting from whatever already drives it.
    fn open(&mut self, field: NumericField) {
        let text = self.live.get(&field).cloned().unwrap_or_default();
        self.drafts.insert(field, text);
    }

    /// Releases a field back to a plain number, keeping the value the expression gave it.
    fn close(&mut self, field: NumericField) {
        self.drafts.remove(&field);
        self.live.remove(&field);
        self.clear_error(field);
    }

    fn clear_error(&mut self, field: NumericField) {
        if self.error.as_ref().is_some_and(|(f, _)| *f == field) {
            self.error = None;
        }
    }

    /// Loads what already drives a feature, so re-editing one shows `wall * 2` rather
    /// than the 10 it works out to.
    fn load(&mut self, exprs: &BTreeMap<NumericField, String>) {
        for (field, text) in exprs {
            self.live.insert(*field, text.clone());
            self.drafts.insert(*field, text.clone());
        }
    }
}

pub struct Tool {
    pub kind: ToolKind,
    pub feature: Option<FeatureId>,
    pub params: Params,
    /// What drives this tool's numbers, for as long as the dialog is open.
    pub exprs: FieldExprs,
    /// Cursor position to restore when editing an existing feature ends.
    restore_cursor: Option<usize>,
    /// The user picked the operation themselves. Until they do, an extrude follows
    /// Fusion's rule: it joins whatever body it lands on and is a new body otherwise.
    pub op_chosen: bool,
    /// Fillet and Chamfer: the largest size the selected edges can take, and the
    /// selection it was worked out for. Asking the kernel costs a pass over the body's
    /// edges, which is nothing beside a boolean but too much to spend on every frame of
    /// a drag, so it is kept until the selection itself moves on.
    limit: (Vec<EdgeRef>, Option<f64>),
}

impl Tool {
    pub fn filter(&self) -> SelectionFilter {
        match self.kind {
            ToolKind::Sketch | ToolKind::OffsetPlane => SelectionFilter::PLANES,
            ToolKind::AngledPlane => SelectionFilter {
                curves: true,
                ..SelectionFilter::PLANES
            },
            ToolKind::Extrude | ToolKind::Loft => SelectionFilter::REGIONS,
            ToolKind::Revolve | ToolKind::Sweep => SelectionFilter {
                curves: true,
                ..SelectionFilter::REGIONS
            },
            // A face stands for every edge around it, as in Fusion; the pick handler
            // expands it, so the feature itself only ever sees edges.
            ToolKind::Fillet | ToolKind::Chamfer => SelectionFilter {
                faces: true,
                ..SelectionFilter::EDGES
            },
            ToolKind::Combine | ToolKind::Move => SelectionFilter::BODIES,
            ToolKind::Component => SelectionFilter::NONE,
        }
    }

    /// Whether this tool is really asking for edges, so a click near one means the edge
    /// and not the face it lies on. Only the blend tools are: they take faces too, but
    /// only as shorthand for every edge around one.
    pub fn prefers_edges(&self) -> bool {
        matches!(self.kind, ToolKind::Fillet | ToolKind::Chamfer)
    }

    /// The largest radius or distance this blend may be given, as last worked out for
    /// the selection it is running on. `None` for a tool that is not a blend, and while
    /// nothing is selected to measure.
    pub fn blend_limit(&self) -> Option<f64> {
        self.limit.1
    }

    pub fn selection_changed(&mut self, selection: &Selection) {
        // A sketch line picked while an axis is wanted becomes the axis.
        if matches!(self.kind, ToolKind::Revolve | ToolKind::AngledPlane)
            && let Some((sketch, line)) = selection.curves.last().copied()
        {
            self.params.axis = Some(AxisRef::SketchLine { sketch, line });
        }
    }

    /// The feature this tool would create from its parameters and the selection, or
    /// `None` while input is incomplete.
    fn build_kind(&self, editor: &Editor) -> Option<FeatureKind> {
        let sel = &editor.selection;
        let component = editor.active_component;
        let operation = |p: &Params| -> Option<BodyOp> {
            match p.op {
                OpKind::NewBody => Some(BodyOp::NewBody),
                OpKind::Join => p.target.map(BodyOp::Join),
                OpKind::Cut => p.target.map(BodyOp::Cut),
                OpKind::Intersect => p.target.map(BodyOp::Intersect),
            }
        };
        let p = &self.params;
        let regions = sel.regions();
        Some(match self.kind {
            ToolKind::Sketch | ToolKind::Component => return None,
            ToolKind::Extrude => {
                if regions.is_empty() {
                    return None;
                }
                let extent = match p.extent {
                    ExtentKind::OneSide => Extent::OneSide(p.distance),
                    ExtentKind::Symmetric => Extent::Symmetric(p.distance),
                    ExtentKind::TwoSides => Extent::TwoSides {
                        positive: p.distance,
                        negative: p.negative,
                    },
                };
                FeatureKind::Extrude {
                    regions,
                    extent,
                    operation: operation(p)?,
                    component,
                }
            }
            ToolKind::Revolve => FeatureKind::Revolve {
                regions: (!regions.is_empty()).then_some(regions)?,
                axis: p.axis?,
                angle: p.angle_deg.to_radians(),
                operation: operation(p)?,
                component,
            },
            ToolKind::Sweep => FeatureKind::Sweep {
                regions: (!regions.is_empty()).then_some(regions)?,
                path: {
                    let (sketch, _) = *sel.curves.first()?;
                    PathRef {
                        sketch,
                        curves: sel
                            .curves
                            .iter()
                            .filter(|(s, _)| *s == sketch)
                            .map(|(_, e)| *e)
                            .collect(),
                    }
                },
                operation: operation(p)?,
                component,
            },
            ToolKind::Loft => FeatureKind::Loft {
                regions: (regions.len() >= 2).then_some(regions)?,
                operation: operation(p)?,
                component,
            },
            ToolKind::Fillet => FeatureKind::Fillet {
                edges: (!sel.edges.is_empty()).then(|| sel.edges.clone())?,
                radius: p.radius,
            },
            ToolKind::Chamfer => FeatureKind::Chamfer {
                edges: (!sel.edges.is_empty()).then(|| sel.edges.clone())?,
                distance: p.radius,
            },
            ToolKind::Combine => {
                let target = *sel.bodies.first()?;
                let tools: Vec<BodyRef> = sel.bodies[1..].to_vec();
                if tools.is_empty() {
                    return None;
                }
                FeatureKind::Combine {
                    target,
                    tools,
                    operation: p.combine,
                    keep_tools: p.keep_tools,
                }
            }
            ToolKind::Move => {
                let body = *sel.bodies.first()?;
                let rot = Quat::from_euler(
                    glam_euler(),
                    p.rotate_deg.x.to_radians(),
                    p.rotate_deg.y.to_radians(),
                    p.rotate_deg.z.to_radians(),
                );
                // The rotation turns the body about itself, not about the world origin.
                // A body 200 mm out would otherwise swing across the screen for a few
                // degrees typed, and the manipulator's rings — drawn on the body —
                // would be promising something quite different from what they do. The
                // pivot comes from the body as it was before this feature, so it stays
                // put while the parameters are being changed.
                let aabb = editor.pick_body(body)?.solid.aabb();
                let pivot = (aabb.min + aabb.max) * 0.5;
                FeatureKind::Move {
                    body,
                    transform: Affine3::from_translation(pivot + p.translate)
                        * Affine3::from_quat(rot)
                        * Affine3::from_translation(-pivot),
                }
            }
            ToolKind::OffsetPlane => FeatureKind::OffsetPlane {
                base: *sel.planes.first()?,
                distance: p.distance,
            },
            ToolKind::AngledPlane => FeatureKind::AngledPlane {
                base: *sel.planes.first()?,
                axis: p.axis?,
                angle: p.angle_deg.to_radians(),
            },
        })
    }
}

fn glam_euler() -> basset_math::EulerRot {
    basset_math::EulerRot::XYZ
}

pub fn start_tool(editor: &mut Editor, kind: ToolKind) {
    if editor.is_sketching() {
        return;
    }
    if editor.tool.is_some() {
        cancel_tool(editor);
    }
    // Measuring and modelling are different jobs: a tool takes over the picking, so the
    // readout would stop agreeing with what a click does.
    super::measure::stop(editor);
    let mut params = Params::default();
    match kind {
        ToolKind::Fillet => params.radius = 2.0,
        ToolKind::Chamfer => params.radius = 1.0,
        ToolKind::AngledPlane => params.angle_deg = 45.0,
        ToolKind::Revolve => params.axis = Some(AxisRef::Origin(OriginAxis::Y)),
        _ => {}
    }
    // Keep a compatible selection made before the tool was picked, the way Fusion lets
    // you select edges and then press Fillet.
    let filter = Tool {
        kind,
        feature: None,
        params: params.clone(),
        exprs: FieldExprs::default(),
        restore_cursor: None,
        op_chosen: false,
        limit: (Vec::new(), None),
    }
    .filter();
    let mut sel = Selection::default();
    if filter.edges {
        sel.edges = editor.selection.edges.clone();
        if matches!(kind, ToolKind::Fillet | ToolKind::Chamfer) {
            for f in &editor.selection.faces {
                for e in edges_of_face(editor, f) {
                    if !sel.edges.contains(&e) {
                        sel.edges.push(e);
                    }
                }
            }
        }
    }
    if filter.faces && matches!(kind, ToolKind::Combine | ToolKind::Move) {
        sel.bodies = editor.selection.bodies.clone();
    }
    if filter.profiles {
        sel.profiles = editor.selection.profiles.clone();
        sel.faces = editor.selection.faces.clone();
        sel.bodies = editor.selection.bodies.clone();
    }
    if filter.planes {
        sel.planes = editor.selection.planes.clone();
        for f in &editor.selection.faces {
            sel.planes.push(PlaneRef::Face(*f));
        }
    }
    editor.selection = sel;
    editor.tool = Some(Tool {
        kind,
        feature: None,
        params,
        exprs: FieldExprs::default(),
        restore_cursor: None,
        op_chosen: false,
        limit: (Vec::new(), None),
    });
    if kind == ToolKind::Component {
        editor.doc.begin_transaction();
    }
    editor.set_status(kind.prompt());
    sync_tool(editor);
    editor.request_repaint();
}

/// Opens the dialog for an existing feature with its parameters and references loaded.
pub fn edit_existing(editor: &mut Editor, id: FeatureId, previous_cursor: usize) {
    let Some(feature) = editor.doc.timeline().get(id).cloned() else {
        return;
    };
    let mut params = Params::default();
    let mut sel = Selection::default();
    let kind = match &feature.kind {
        FeatureKind::Extrude {
            regions,
            extent,
            operation,
            ..
        } => {
            load_regions(&mut sel, regions);
            match *extent {
                Extent::OneSide(d) => {
                    params.extent = ExtentKind::OneSide;
                    params.distance = d;
                }
                Extent::Symmetric(d) => {
                    params.extent = ExtentKind::Symmetric;
                    params.distance = d;
                }
                Extent::TwoSides { positive, negative } => {
                    params.extent = ExtentKind::TwoSides;
                    params.distance = positive;
                    params.negative = negative;
                }
            }
            load_op(&mut params, *operation);
            ToolKind::Extrude
        }
        FeatureKind::Revolve {
            regions,
            axis,
            angle,
            operation,
            ..
        } => {
            load_regions(&mut sel, regions);
            params.axis = Some(*axis);
            params.angle_deg = angle.to_degrees();
            load_op(&mut params, *operation);
            ToolKind::Revolve
        }
        FeatureKind::Sweep {
            regions,
            path,
            operation,
            ..
        } => {
            load_regions(&mut sel, regions);
            sel.curves = path.curves.iter().map(|c| (path.sketch, *c)).collect();
            load_op(&mut params, *operation);
            ToolKind::Sweep
        }
        FeatureKind::Loft {
            regions, operation, ..
        } => {
            load_regions(&mut sel, regions);
            load_op(&mut params, *operation);
            ToolKind::Loft
        }
        FeatureKind::Fillet { edges, radius } => {
            sel.edges = edges.clone();
            params.radius = *radius;
            ToolKind::Fillet
        }
        FeatureKind::Chamfer { edges, distance } => {
            sel.edges = edges.clone();
            params.radius = *distance;
            ToolKind::Chamfer
        }
        FeatureKind::Combine {
            target,
            tools,
            operation,
            keep_tools,
        } => {
            sel.bodies = std::iter::once(*target)
                .chain(tools.iter().copied())
                .collect();
            params.combine = *operation;
            params.keep_tools = *keep_tools;
            ToolKind::Combine
        }
        FeatureKind::Move { body, transform } => {
            sel.bodies = vec![*body];
            let (_, rot, trans) = transform.to_scale_rotation_translation();
            let (x, y, z) = rot.to_euler(glam_euler());
            params.translate = trans;
            params.rotate_deg = Vec3::new(x.to_degrees(), y.to_degrees(), z.to_degrees());
            ToolKind::Move
        }
        FeatureKind::OffsetPlane { base, distance } => {
            sel.planes = vec![*base];
            params.distance = *distance;
            ToolKind::OffsetPlane
        }
        FeatureKind::AngledPlane { base, axis, angle } => {
            sel.planes = vec![*base];
            params.axis = Some(*axis);
            params.angle_deg = angle.to_degrees();
            ToolKind::AngledPlane
        }
        FeatureKind::Sketch { .. } | FeatureKind::NewComponent { .. } => return,
    };
    let restore_cursor = Some(previous_cursor);
    // A driven feature re-opens showing what drives it, not the number it works out to:
    // anything else would quietly replace the intent with its result on the next OK.
    let mut exprs = FieldExprs::default();
    exprs.load(&feature.exprs);
    editor.doc.begin_transaction();
    editor.selection = sel;
    editor.tool = Some(Tool {
        kind,
        feature: Some(id),
        params,
        exprs,
        restore_cursor,
        // An existing feature's operation was decided when it was made.
        op_chosen: true,
        limit: (Vec::new(), None),
    });
    editor.set_status(format!("Editing {}", feature.name));
}

/// Splits a feature's regions back into the selection buckets the viewport highlights
/// from, so re-opening a generator shows exactly what it is built on.
fn load_regions(sel: &mut Selection, regions: &[RegionRef]) {
    for region in regions {
        match region {
            RegionRef::Profile(p) => sel.profiles.push(*p),
            RegionRef::Face(f) => {
                sel.faces.push(*f);
                if !sel.bodies.contains(&f.body) {
                    sel.bodies.push(f.body);
                }
            }
        }
    }
}

fn load_op(params: &mut Params, op: BodyOp) {
    match op {
        BodyOp::NewBody => params.op = OpKind::NewBody,
        BodyOp::Join(t) => (params.op, params.target) = (OpKind::Join, Some(t)),
        BodyOp::Cut(t) => (params.op, params.target) = (OpKind::Cut, Some(t)),
        BodyOp::Intersect(t) => (params.op, params.target) = (OpKind::Intersect, Some(t)),
    }
}

/// Pushes the tool's current inputs into its feature, creating it on first validity.
pub fn sync_tool(editor: &mut Editor) {
    // The join heuristic and face regions read the pick cache, which may predate the
    // selection change that brought us here.
    editor.refresh_cache();
    let Some(tool) = editor.tool.as_ref() else {
        return;
    };
    if tool.kind == ToolKind::Sketch {
        // A planar face picked after the tool started lands in `faces`; it is as good a
        // sketch plane as a construction plane.
        let plane = editor
            .selection
            .planes
            .first()
            .copied()
            .or_else(|| editor.selection.faces.first().map(|f| PlaneRef::Face(*f)));
        if let Some(plane) = plane {
            editor.tool = None;
            editor.selection.clear();
            sketch_mode::enter_new(editor, plane);
        }
        return;
    }
    if tool.kind == ToolKind::Extrude && !tool.op_chosen {
        let target = extrude_lands_on(editor);
        if let Some(t) = editor.tool.as_mut() {
            match target {
                Some(body) => (t.params.op, t.params.target) = (OpKind::Join, Some(body)),
                None => (t.params.op, t.params.target) = (OpKind::NewBody, None),
            }
        }
    }
    let tool = editor.tool.as_ref().unwrap();
    // Join/Cut/Intersect need a target; default to the first other body.
    if matches!(
        tool.params.op,
        OpKind::Join | OpKind::Cut | OpKind::Intersect
    ) && tool.params.target.is_none()
    {
        let own = tool.feature.map(BodyRef);
        let target = editor
            .cached_bodies
            .iter()
            .map(|(id, _)| *id)
            .find(|id| Some(*id) != own);
        if let Some(t) = editor.tool.as_mut() {
            t.params.target = target;
        }
    }
    refresh_blend_limit(editor);
    let tool = editor.tool.as_ref().unwrap();
    let Some(kind) = tool.build_kind(editor) else {
        return;
    };
    match tool.feature {
        None => {
            editor.doc.begin_transaction();
            let id = editor.doc.add_feature(kind);
            if let Some(t) = editor.tool.as_mut() {
                t.feature = Some(id);
            }
        }
        Some(id) => {
            if let Err(e) = editor.doc.edit_feature_kind(id, |k| *k = kind) {
                editor.report_error(e);
            }
        }
    }
    apply_field_exprs(editor);
    editor.request_repaint();
}

/// Writes what the dialog says drives each number onto the feature it is previewing.
///
/// It runs on every sync rather than once at OK because the feature is the preview: an
/// expression has to reach it to be seen, and it is inside the tool's transaction either
/// way, so Cancel takes it back with the rest. The number itself is already in the kind
/// the sync just wrote — [`basset_core::Document::set_feature_expr`] writes the same
/// value again — so the text and the number cannot disagree.
fn apply_field_exprs(editor: &mut Editor) {
    let Some(tool) = editor.tool.as_ref() else {
        return;
    };
    let Some(id) = tool.feature else {
        return;
    };
    let live = tool.exprs.live.clone();
    let Some(offered) = editor
        .doc
        .timeline()
        .get(id)
        .map(|f| f.kind.numeric_fields().to_vec())
    else {
        return;
    };
    for field in FIELDS {
        // A field the kind no longer offers — the second distance of an extrude that is
        // no longer two-sided — is released rather than left driving nothing, which would
        // warn on every regeneration from then on.
        let result = match live.get(&field).filter(|_| offered.contains(&field)) {
            Some(text) => editor.doc.set_feature_expr(id, field, text).map(|_| ()),
            None => editor.doc.clear_feature_expr(id, field).map(|_| ()),
        };
        if let Err(e) = result {
            editor.report_error(e);
        }
    }
}

/// Works out how large the running blend may be, if the selection has moved since the
/// last time it was asked.
///
/// The bodies picking runs against are the model *before* the previewed feature, which
/// is the same body the fillet is applied to, so the number is the one the kernel will
/// hold the radius against. Several bodies blended at once take the smallest of theirs:
/// one feature carries one radius, and it has to fit everywhere it lands.
fn refresh_blend_limit(editor: &mut Editor) {
    let Some(tool) = editor.tool.as_ref() else {
        return;
    };
    if !matches!(tool.kind, ToolKind::Fillet | ToolKind::Chamfer)
        || tool.limit.0 == editor.selection.edges
    {
        return;
    }
    let kind = tool.kind;
    let edges = editor.selection.edges.clone();
    let mut by_body: Vec<(BodyRef, Vec<basset_kernel::EdgeKey>)> = Vec::new();
    for e in &edges {
        match by_body.iter_mut().find(|(b, _)| *b == e.body) {
            Some((_, keys)) => keys.push(e.key),
            None => by_body.push((e.body, vec![e.key])),
        }
    }
    let mut limit: Option<f64> = None;
    for (body, keys) in by_body {
        let Some(solid) = editor.pick_body(body).map(|p| p.solid.clone()) else {
            continue;
        };
        let found = match kind {
            ToolKind::Chamfer => basset_kernel::blend::max_chamfer_distance(&solid, &keys),
            _ => basset_kernel::blend::max_fillet_radius(&solid, &keys),
        };
        if let Some(found) = found {
            limit = Some(limit.map_or(found, |l: f64| l.min(found)));
        }
    }
    if let Some(tool) = editor.tool.as_mut() {
        tool.limit = (edges, limit);
    }
}

pub fn confirm_tool(editor: &mut Editor) {
    let Some(tool) = editor.tool.take() else {
        return;
    };
    if tool.kind == ToolKind::Component {
        let name = tool.params.name.trim().to_string();
        if !name.is_empty() {
            editor.doc.add_feature(FeatureKind::NewComponent {
                name,
                parent: editor.active_component,
            });
        }
        editor.doc.commit_transaction();
    } else if tool.feature.is_some() {
        if let Some(c) = tool.restore_cursor {
            editor.doc.set_cursor(c);
        }
        editor.doc.commit_transaction();
        editor.set_status(format!("{} applied", tool.kind.title()));
    } else {
        editor.doc.rollback_transaction();
        editor.set_status("Nothing to apply");
    }
    editor.selection.clear();
    editor.request_repaint();
}

pub fn cancel_tool(editor: &mut Editor) {
    let Some(tool) = editor.tool.take() else {
        return;
    };
    editor.doc.rollback_transaction();
    if let Some(c) = tool.restore_cursor {
        editor.doc.set_cursor(c);
    }
    editor.selection.clear();
    editor.set_status("Cancelled");
    editor.request_repaint();
}

/// The tool dialog. Returns after possibly mutating the editor.
pub fn dialog(editor: &mut Editor, ctx: &egui::Context) {
    let Some(tool) = editor.tool.as_ref() else {
        return;
    };
    let kind = tool.kind;
    let mut changed = false;
    let mut action: Option<bool> = None;
    let status = tool
        .feature
        .and_then(|id| editor.cached_statuses.get(&id).cloned());
    let blend_limit = tool.blend_limit();
    let bodies: Vec<(BodyRef, String)> = editor.cached_bodies.clone();
    let selection_text = editor.selection.summary();
    let edge_count = editor.selection.edges.len();
    // Cloned out before the window closure takes the editor mutably. It is a handful of
    // rows, and an expression typed into a dialog has to be judged against the table as
    // it stands this frame.
    let parameters = editor.doc.parameters().clone();

    let op_before = tool.params.op;
    egui::Window::new(kind.title())
        .id(egui::Id::new("tool-dialog"))
        .collapsible(false)
        .resizable(false)
        .anchor(egui::Align2::RIGHT_TOP, [-12.0, 190.0])
        .show(ctx, |ui| {
            let tool = editor.tool.as_mut().expect("checked above");
            let (p, ex) = (&mut tool.params, &mut tool.exprs);
            ui.label(kind.prompt());
            ui.label(egui::RichText::new(&selection_text).weak());
            ui.separator();
            match kind {
                ToolKind::Extrude => {
                    ui.horizontal(|ui| {
                        ui.label("Extent");
                        for (k, name) in [
                            (ExtentKind::OneSide, "One side"),
                            (ExtentKind::Symmetric, "Symmetric"),
                            (ExtentKind::TwoSides, "Two sides"),
                        ] {
                            changed |= ui.selectable_value(&mut p.extent, k, name).changed();
                        }
                    });
                    changed |= drag(
                        ui,
                        ex,
                        &parameters,
                        NumericField::Distance,
                        "Distance",
                        &mut p.distance,
                        0.5,
                        "mm",
                    );
                    if p.extent == ExtentKind::TwoSides {
                        changed |= drag(
                            ui,
                            ex,
                            &parameters,
                            NumericField::Negative,
                            "Negative",
                            &mut p.negative,
                            0.5,
                            "mm",
                        );
                    }
                    changed |= operation_ui(ui, p, &bodies);
                }
                ToolKind::Revolve => {
                    changed |= axis_ui(ui, p);
                    changed |= drag(
                        ui,
                        ex,
                        &parameters,
                        NumericField::Angle,
                        "Angle",
                        &mut p.angle_deg,
                        1.0,
                        "°",
                    );
                    changed |= operation_ui(ui, p, &bodies);
                }
                ToolKind::Sweep | ToolKind::Loft => {
                    changed |= operation_ui(ui, p, &bodies);
                }
                ToolKind::Fillet | ToolKind::Chamfer => {
                    // The count is the one thing a blend's dialog cannot show as
                    // geometry: the highlighted edges may be behind the body or off the
                    // side of it, and "6 edges" is how the user knows the rim closed.
                    ui.label(format!(
                        "{edge_count} edge{} selected",
                        if edge_count == 1 { "" } else { "s" }
                    ));
                    changed |= ui
                        .checkbox(&mut p.tangent_chain, "Tangent chain")
                        .on_hover_text(
                            "Take the whole smooth run of edges with each pick \
                             (hold Ctrl while clicking for a single edge)",
                        )
                        .changed();
                    // The one parameter is a radius on a fillet and a setback on a
                    // chamfer, which is also which field an expression on it drives.
                    let (label, field) = match kind {
                        ToolKind::Fillet => ("Radius", NumericField::Radius),
                        _ => ("Distance", NumericField::Distance),
                    };
                    changed |= drag(ui, ex, &parameters, field, label, &mut p.radius, 0.1, "mm");
                    // What the material allows. The handle in the viewport stops there of
                    // its own accord, so this is for the box, which does not: a number
                    // typed past it is still sent to the kernel and still refused, and
                    // the user should be able to see why before that happens.
                    if let Some(max) = blend_limit {
                        let over = p.radius > max;
                        let text = egui::RichText::new(format!(
                            "{} {max:.2} mm fits between these edges and the faces they \
                             sit on",
                            if over { "only" } else { "up to" }
                        ));
                        ui.label(if over {
                            text.color(egui::Color32::from_rgb(235, 190, 90))
                        } else {
                            text.weak()
                        });
                    }
                }
                ToolKind::Combine => {
                    ui.horizontal(|ui| {
                        ui.label("Operation");
                        for (op, name) in [
                            (CombineOp::Join, "Join"),
                            (CombineOp::Cut, "Cut"),
                            (CombineOp::Intersect, "Intersect"),
                        ] {
                            changed |= ui.selectable_value(&mut p.combine, op, name).changed();
                        }
                    });
                    changed |= ui.checkbox(&mut p.keep_tools, "Keep tool bodies").changed();
                }
                ToolKind::Move => {
                    ui.label("Translate (mm)");
                    changed |= vec3_ui(ui, &mut p.translate, 0.5);
                    ui.label("Rotate (°)");
                    changed |= vec3_ui(ui, &mut p.rotate_deg, 1.0);
                }
                ToolKind::OffsetPlane => {
                    changed |= drag(
                        ui,
                        ex,
                        &parameters,
                        NumericField::Distance,
                        "Distance",
                        &mut p.distance,
                        0.5,
                        "mm",
                    );
                }
                ToolKind::AngledPlane => {
                    changed |= axis_ui(ui, p);
                    changed |= drag(
                        ui,
                        ex,
                        &parameters,
                        NumericField::Angle,
                        "Angle",
                        &mut p.angle_deg,
                        1.0,
                        "°",
                    );
                }
                ToolKind::Component => {
                    ui.horizontal(|ui| {
                        ui.label("Name");
                        ui.text_edit_singleline(&mut p.name);
                    });
                }
                ToolKind::Sketch => {}
            }
            match &status {
                Some(FeatureStatus::Failed(msg)) => {
                    ui.separator();
                    ui.colored_label(egui::Color32::from_rgb(230, 120, 100), msg);
                }
                Some(FeatureStatus::Warned(msg)) => {
                    ui.separator();
                    ui.colored_label(egui::Color32::from_rgb(235, 190, 90), msg);
                }
                _ => {}
            }
            ui.separator();
            ui.horizontal(|ui| {
                let ready = editor
                    .tool
                    .as_ref()
                    .is_some_and(|t| t.feature.is_some() || t.kind == ToolKind::Component);
                if ui.add_enabled(ready, egui::Button::new("OK")).clicked() {
                    action = Some(true);
                }
                if ui.button("Cancel").clicked() {
                    action = Some(false);
                }
            });
        });

    if let Some(t) = editor.tool.as_mut()
        && t.params.op != op_before
    {
        t.op_chosen = true;
    }
    changed |= handle_drag(editor, ctx);
    if changed {
        sync_tool(editor);
    }
    match action {
        Some(true) => confirm_tool(editor),
        Some(false) => cancel_tool(editor),
        None => {}
    }
}

// --- Viewport handles ----------------------------------------------------------------------

/// An arrow in the viewport that drags a tool's size: the extrude distance, the fillet
/// radius, the plane offset. `origin` is where it grows from, `tip` where the size
/// currently reaches, `dir` the unit direction a positive drag moves along.
#[derive(Clone, Copy, Debug)]
pub struct Handle {
    pub origin: Vec3,
    pub tip: Vec3,
    pub dir: Vec3,
}

/// The number the viewport handle drags, for the kinds that have one. The handle is the
/// same value as the dialog's box under a different skin, so it has to obey the same rule
/// about being driven.
fn handle_field(kind: ToolKind) -> Option<NumericField> {
    match kind {
        ToolKind::Extrude | ToolKind::OffsetPlane | ToolKind::Chamfer => {
            Some(NumericField::Distance)
        }
        ToolKind::Fillet => Some(NumericField::Radius),
        _ => None,
    }
}

/// The handle for the running tool, or `None` when it has nothing to drag yet.
pub fn handle(editor: &Editor) -> Option<Handle> {
    let tool = editor.tool.as_ref()?;
    let p = &tool.params;
    match tool.kind {
        ToolKind::Extrude => {
            let region = editor.selection.regions().into_iter().next()?;
            let (origin, dir, _) = editor.region_geometry(&region)?;
            // The handle shows the side the extrude reaches on; a two-sided extrude has
            // one for its positive side only, the negative side stays in the dialog.
            let reach = match p.extent {
                ExtentKind::OneSide => p.distance,
                ExtentKind::Symmetric => p.distance * 0.5,
                ExtentKind::TwoSides => p.distance,
            };
            Some(Handle {
                origin,
                tip: origin + dir * reach,
                dir,
            })
        }
        ToolKind::Fillet | ToolKind::Chamfer => {
            let edge = editor.selection.edges.first()?;
            let body = editor.pick_body(edge.body)?;
            let edge = body.edges.iter().find(|e| e.key == edge.key)?;
            // The middle segment of the edge, and the direction the blend grows in.
            let seg = edge.segments.get(edge.segments.len() / 2)?;
            let origin = (seg.start + seg.end) * 0.5;
            // Into the corner, not out of it: a fillet's radius is measured towards the
            // centre of the blend, which is inside the material on a convex edge and out
            // in the notch on a concave one. The arrow used to point the opposite way and
            // so pointed at the material the tool leaves alone.
            let dir = basset_kernel::blend::blend_direction(seg)?;
            Some(Handle {
                origin,
                tip: origin + dir * p.radius,
                dir,
            })
        }
        ToolKind::OffsetPlane => {
            let base = editor.selection.planes.first()?;
            let frame = editor.plane_frame(base)?;
            Some(Handle {
                origin: frame.origin,
                tip: frame.origin + frame.z * p.distance,
                dir: frame.z,
            })
        }
        _ => None,
    }
}

/// The number a field's box edits, for a caller that names a field rather than holding
/// the box.
///
/// The dialog hands its `&mut f64` straight to [`drag`], so this is the one other place
/// that has to agree with it about which of the flat [`Params`] each field is. A blend's
/// single size is `radius` whether the kind calls it a radius or a setback.
#[cfg(test)]
fn params_field(kind: ToolKind, params: &mut Params, field: NumericField) -> Option<&mut f64> {
    match (kind, field) {
        (ToolKind::Fillet | ToolKind::Chamfer, NumericField::Radius | NumericField::Distance) => {
            Some(&mut params.radius)
        }
        (_, NumericField::Distance) => Some(&mut params.distance),
        (_, NumericField::Negative) => Some(&mut params.negative),
        (_, NumericField::Angle) => Some(&mut params.angle_deg),
        (_, NumericField::Radius) => None,
    }
}

/// Drives one of the running tool's numbers by an expression, as typing it into the
/// dialog's box and leaving the box does.
///
/// A test cannot type into an egui box, so this is how one states an expression: it fills
/// the same draft, takes it through the same [`FieldExprs::commit`] the box does, and
/// syncs the tool. Returns whether it was taken.
#[cfg(test)]
pub(crate) fn type_expression(editor: &mut Editor, field: NumericField, text: &str) -> bool {
    let parameters = editor.doc.parameters().clone();
    let Some(tool) = editor.tool.as_mut() else {
        return false;
    };
    let kind = tool.kind;
    tool.exprs.drafts.insert(field, text.to_string());
    let mut spare = 0.0;
    let value = match params_field(kind, &mut tool.params, field) {
        Some(slot) => slot,
        None => &mut spare,
    };
    if !tool.exprs.commit(field, &parameters, text, value) {
        return false;
    }
    sync_tool(editor);
    true
}

/// The increment a dragged size rounds to, in mm. See the comment at the drag itself
/// for why it is fixed rather than the zoom-following one the sketch grid uses.
pub(crate) const HANDLE_STEP: f64 = 1.0;

/// Draws the handle's grip at its tip and applies a drag of it to the tool's size.
/// Returns whether the size changed. The arrow shaft itself is drawn by the scene, so
/// it sits in 3D with the geometry; only the grip is an egui widget.
fn handle_drag(editor: &mut Editor, ctx: &egui::Context) -> bool {
    let Some(h) = handle(editor) else {
        return false;
    };
    let window = editor.window_px;
    let camera = editor.camera;
    let Some(tip_px) = camera.world_to_screen(h.tip, window) else {
        return false;
    };
    let ppp = f64::from(ctx.pixels_per_point());
    const GRIP: f32 = 22.0;
    let centre = egui::pos2((tip_px[0] / ppp) as f32, (tip_px[1] / ppp) as f32);
    let response = egui::Area::new(egui::Id::new("tool-handle"))
        .fixed_pos(centre - egui::vec2(GRIP * 0.5, GRIP * 0.5))
        .order(egui::Order::Foreground)
        .show(ctx, |ui| {
            let (rect, response) =
                ui.allocate_exact_size(egui::vec2(GRIP, GRIP), egui::Sense::drag());
            let hot = response.hovered() || response.dragged();
            let fill = if hot {
                egui::Color32::from_rgb(255, 215, 80)
            } else {
                egui::Color32::from_rgb(90, 160, 255)
            };
            ui.painter().circle(
                rect.center(),
                GRIP * 0.32,
                fill,
                egui::Stroke::new(1.5, egui::Color32::from_gray(20)),
            );
            if hot {
                ui.ctx().set_cursor_icon(egui::CursorIcon::Grab);
            }
            response
        })
        .inner;
    if !response.dragged() {
        // The gesture is over, so its running total goes with it: the next grab of the
        // grip starts from wherever the size actually is. See [`snap::Drag::release`].
        editor.drags.slide.release();
        return false;
    }
    // A driven size is read-only wherever it is shown, and the arrow is one of the places
    // it is shown. Letting the drag through would write a number the next sync overwrites
    // from the expression, leaving the arrow, the dialog's readout and the solid all
    // saying different things; releasing the expression on a drag would throw away what
    // the user stated because they brushed a grip. So the drag is ignored and the reason
    // is said, which is the only part of it the user cannot already see.
    if let Some(tool) = editor.tool.as_ref()
        && let Some(field) = handle_field(tool.kind)
        && tool.exprs.is_open(field)
    {
        editor.set_status(format!(
            "{} is driven by an expression: turn the expression off to drag it",
            field.label()
        ));
        return false;
    }
    // Project the drag onto the arrow's direction on screen, then scale by the world
    // size of a pixel at the tip so the tip follows the pointer along the arrow.
    let delta = response.drag_delta();
    let Some(ahead) = camera.world_to_screen(h.tip + h.dir, window) else {
        return false;
    };
    let screen_dir = basset_math::Vec2::new(ahead[0] - tip_px[0], ahead[1] - tip_px[1]);
    let screen_dir = screen_dir.normalize_or_zero();
    if screen_dir == basset_math::Vec2::ZERO {
        return false;
    }
    let along =
        (f64::from(delta.x) * ppp) * screen_dir.x + (f64::from(delta.y) * ppp) * screen_dir.y;
    let world = along * camera.pixel_size_at(h.tip, window);
    // A size lands on round numbers, and shift lets go of that for as long as it is
    // held. A dragged extrude that stopped at 12.37 mm would make the number a chore to
    // read back. Unlike a sketch drag, the increment here is fixed rather than following
    // the zoom: a size is a number first and a picture second, so quantising it to 5 or
    // 10 mm because the camera stood back — or changing the increment mid-gesture as the
    // arrow's tip moved through the view — makes the handle feel arbitrary. The
    // gesture's running total goes through [`snap::Drag`], as the gizmo's does: rounding
    // each frame's slice instead would throw a slow drag's sub-step deltas away one by
    // one and the handle would only ever move on a flick.
    let snap = editor.snapping.at(HANDLE_STEP);
    let Some(tool) = editor.tool.as_mut() else {
        return false;
    };
    let drag = &mut editor.drags.slide;
    let limit = tool.blend_limit();
    let p = &mut tool.params;
    let value = match tool.kind {
        ToolKind::Extrude => {
            let scale = if p.extent == ExtentKind::Symmetric {
                2.0
            } else {
                1.0
            };
            p.distance = if p.extent == ExtentKind::OneSide {
                let at = drag.advance(p.distance, world * scale, snap);
                // A one-sided extrude may cross to the other side of its sketch, but it
                // cannot pause at no length on the way: the kernel refuses a zero
                // extent, and a drag resting there would leave a dead preview and a
                // warning per frame. The size holds one increment short, on whichever
                // side it was already on, until the drag carries it across.
                if at == 0.0 {
                    let short = snap.step().unwrap_or(0.01);
                    if p.distance < 0.0 { -short } else { short }
                } else {
                    at
                }
            } else {
                drag.advance_above(p.distance, world * scale, snap, 0.01)
            };
            p.distance
        }
        ToolKind::Fillet | ToolKind::Chamfer => {
            // Stopped at what the material allows rather than let past it and refused:
            // a blend dragged out of range would take the shell off the screen and
            // leave the user pulling back through a preview that is not there. The
            // handle simply stops, which is the same thing the geometry does.
            let ceiling = limit.unwrap_or(f64::INFINITY).max(0.01);
            p.radius = drag.advance_within(p.radius, world, snap, 0.01, ceiling);
            p.radius
        }
        ToolKind::OffsetPlane => {
            p.distance = drag.advance(p.distance, world, snap);
            p.distance
        }
        _ => return false,
    };
    // The arrow has already been laid out for this frame, so the hint goes on the tip
    // the size now reaches rather than the one it was grabbed at.
    let at = h.origin + h.dir * value;
    editor.snap_hint = Some(Hint::value(at, value, " mm", snap));
    true
}

/// The body an extrude of the current selection would touch, so it can join it the way
/// Fusion's default does. Bounding boxes stand in for the real geometry: a false
/// positive only means a union with something the extrusion does not reach, which the
/// kernel handles, and the dialog still lets the user choose otherwise.
fn extrude_lands_on(editor: &Editor) -> Option<BodyRef> {
    let tool = editor.tool.as_ref()?;
    let own = tool.feature.map(BodyRef);
    let (lo, hi) = match tool.params.extent {
        ExtentKind::OneSide => (tool.params.distance.min(0.0), tool.params.distance.max(0.0)),
        ExtentKind::Symmetric => (
            -tool.params.distance.abs() * 0.5,
            tool.params.distance.abs() * 0.5,
        ),
        ExtentKind::TwoSides => (-tool.params.negative.abs(), tool.params.distance.abs()),
    };
    let mut swept = Aabb::empty();
    let mut face_bodies: Vec<BodyRef> = Vec::new();
    for region in editor.selection.regions() {
        let Some((_, normal, aabb)) = editor.region_geometry(&region) else {
            continue;
        };
        for d in [lo, hi] {
            let shifted = Aabb {
                min: aabb.min + normal * d,
                max: aabb.max + normal * d,
            };
            swept = swept.union(&shifted);
        }
        if let RegionRef::Face(f) = region {
            face_bodies.push(f.body);
        }
    }
    if swept.is_empty() {
        return None;
    }
    // Two boxes that merely touch count: extruding up from a body's top face must join
    // that body, and the extrusion's box only touches it there.
    let tol = 1e-6;
    let touches = |a: &Aabb, b: &Aabb| {
        (0..3).all(|i| a.min[i] <= b.max[i] + tol && b.min[i] <= a.max[i] + tol)
    };
    let mut candidates: Vec<BodyRef> = editor
        .doc_state_bodies()
        .into_iter()
        .filter(|id| Some(*id) != own && !editor.hidden_bodies.contains(id))
        .filter(|id| {
            editor
                .pick_body(*id)
                .is_some_and(|b| touches(&b.solid.aabb(), &swept))
        })
        .collect();
    // The body a picked face belongs to is the one the user means.
    candidates.sort_by_key(|id| !face_bodies.contains(id));
    candidates.first().copied()
}

/// Every edge bordering a face, so a face pick can stand for all of them.
///
/// Smooth edges are left out: where the face simply carries on into its neighbour there
/// is nothing drawn to round, and a blend tool built along one has no dihedral to fill.
pub fn edges_of_face(editor: &Editor, face: &basset_core::FaceRef) -> Vec<basset_core::EdgeRef> {
    let Some(body) = editor.pick_body(face.body) else {
        return Vec::new();
    };
    body.edges
        .iter()
        .filter(|e| !e.smooth && e.key.touches(face.key))
        .map(|e| basset_core::EdgeRef {
            body: face.body,
            key: e.key,
        })
        .collect()
}

/// The tangentially continuous run of edges a picked edge belongs to: what Fusion calls
/// the tangent chain, and what the user means by "round that rim". An edge that nothing
/// runs on from stands for itself alone.
pub fn tangent_chain(editor: &Editor, edge: &basset_core::EdgeRef) -> Vec<basset_core::EdgeRef> {
    let Some(body) = editor.pick_body(edge.body) else {
        return vec![*edge];
    };
    basset_kernel::pick::tangent_chain(&body.edges, edge.key)
        .into_iter()
        .map(|key| basset_core::EdgeRef {
            body: edge.body,
            key,
        })
        .collect()
}

/// Fillet and Chamfer read a pick as shorthand for a group of edges: a face stands for
/// the ring around it, an edge for its tangent chain. Picking the same thing again takes
/// the group back out, so a misaimed click is undone by repeating it rather than by
/// restarting the tool. Returns whether the pick was consumed this way.
///
/// Ctrl is the escape hatch from the chain, for the one edge of a rim that wants a
/// different radius; the dialog's checkbox is the same choice made for every pick.
pub fn expand_face_pick(editor: &mut Editor, pick: &Pick) -> bool {
    let Some(tool) = editor.tool.as_ref() else {
        return false;
    };
    if !tool.prefers_edges() {
        return false;
    }
    let chaining = tool.params.tangent_chain && !editor.pointer.ctrl;
    let group = match pick {
        Pick::Face(face, _) => edges_of_face(editor, face),
        Pick::Edge(edge, _) if chaining => tangent_chain(editor, edge),
        // Anything else is an ordinary pick for the selection to toggle.
        _ => return false,
    };
    if group.is_empty() {
        return true;
    }
    // All of it already in means the user is pointing at their own selection: let it go.
    let all_in = group.iter().all(|e| editor.selection.edges.contains(e));
    if all_in {
        editor.selection.edges.retain(|e| !group.contains(e));
    } else {
        for e in group {
            if !editor.selection.edges.contains(&e) {
                editor.selection.edges.push(e);
            }
        }
    }
    true
}

impl Editor {
    /// A region's anchor, outward normal and world bounding box: what the extrude
    /// handle and the join heuristic need. Profiles come from the solved sketch, faces
    /// from the body as picking sees it.
    pub fn region_geometry(&self, region: &RegionRef) -> Option<(Vec3, Vec3, Aabb)> {
        match region {
            RegionRef::Profile(p) => {
                let solved = self
                    .cached_sketches
                    .iter()
                    .find(|(id, _)| *id == p.sketch)
                    .map(|(_, s)| s.clone())?;
                let profile = solved
                    .profiles
                    .iter()
                    .filter(|r| r.contains(p.sample))
                    .min_by(|a, b| a.area().total_cmp(&b.area()))?;
                let frame = &profile.frame;
                let mut aabb = Aabb::empty();
                let mut sum = basset_math::Vec2::ZERO;
                for pt in &profile.outer.points {
                    aabb.include(frame.to_world(*pt));
                    sum += *pt;
                }
                let n = profile.outer.points.len().max(1) as f64;
                Some((frame.to_world(sum / n), frame.z, aabb))
            }
            RegionRef::Face(f) => {
                let body = self.pick_body(f.body)?;
                let face = body.solid.face(f.key)?;
                let frame = face.frame()?;
                let mut aabb = Aabb::empty();
                for poly in &face.polygons {
                    for v in &poly.vertices {
                        aabb.include(*v);
                    }
                }
                Some((frame.origin, frame.z, aabb))
            }
        }
    }
}

/// One number of a dialog: a drag box, or the expression driving it.
///
/// Every number in every tool goes through here, which is what makes "any of them can be
/// driven by a parameter" one change rather than one per tool. The toggle swaps the box
/// for an expression; while an expression is set the number is read-only and shows what
/// the expression works out to, because there the expression is the input and the number
/// is only its result.
///
/// Returns whether the value moved, which is the same signal the plain drag box gave: the
/// caller syncs the feature on it.
fn drag(
    ui: &mut egui::Ui,
    exprs: &mut FieldExprs,
    parameters: &Parameters,
    field: NumericField,
    label: &str,
    value: &mut f64,
    speed: f64,
    suffix: &str,
) -> bool {
    let mut changed = false;
    ui.horizontal(|ui| {
        ui.label(label);
        match exprs.drafts.get_mut(&field) {
            Some(draft) => {
                let response = ui.add(
                    egui::TextEdit::singleline(draft)
                        .desired_width(96.0)
                        .hint_text("expression"),
                );
                let text = draft.trim().to_string();
                // Taken when the box is left or Enter is pressed, as everywhere else an
                // expression is typed: a name half written is not yet an error.
                if response.lost_focus() && !text.is_empty() {
                    changed |= exprs.commit(field, parameters, &text, value);
                }
                ui.label(egui::RichText::new(format!("= {value:.3} {suffix}")).weak());
            }
            None => {
                changed |= ui
                    .add(
                        egui::DragValue::new(value)
                            .speed(speed)
                            .suffix(format!(" {suffix}")),
                    )
                    .changed();
            }
        }
        let open = exprs.is_open(field);
        if ui
            .selectable_label(open, "\u{192}")
            .on_hover_text("Drive this by an expression over the document's parameters")
            .clicked()
        {
            if open {
                // Releasing keeps the value the expression gave it, the same rule
                // retyping a number over a driven sketch dimension follows.
                exprs.close(field);
                changed = true;
            } else {
                exprs.open(field);
            }
        }
    });
    if let Some((f, message)) = &exprs.error
        && *f == field
    {
        ui.colored_label(egui::Color32::from_rgb(230, 120, 100), message);
    }
    changed
}

fn vec3_ui(ui: &mut egui::Ui, v: &mut Vec3, speed: f64) -> bool {
    ui.horizontal(|ui| {
        ui.add(egui::DragValue::new(&mut v.x).speed(speed).prefix("x "))
            .changed()
            | ui.add(egui::DragValue::new(&mut v.y).speed(speed).prefix("y "))
                .changed()
            | ui.add(egui::DragValue::new(&mut v.z).speed(speed).prefix("z "))
                .changed()
    })
    .inner
}

fn operation_ui(ui: &mut egui::Ui, p: &mut Params, bodies: &[(BodyRef, String)]) -> bool {
    let mut changed = false;
    ui.horizontal(|ui| {
        ui.label("Operation");
        for (op, name) in [
            (OpKind::NewBody, "New body"),
            (OpKind::Join, "Join"),
            (OpKind::Cut, "Cut"),
            (OpKind::Intersect, "Intersect"),
        ] {
            changed |= ui.selectable_value(&mut p.op, op, name).changed();
        }
    });
    if p.op != OpKind::NewBody {
        let current = p
            .target
            .and_then(|t| bodies.iter().find(|(id, _)| *id == t))
            .map(|(_, n)| n.clone());
        egui::ComboBox::from_label("Target body")
            .selected_text(current.unwrap_or_else(|| "(none)".into()))
            .show_ui(ui, |ui| {
                for (id, name) in bodies {
                    changed |= ui
                        .selectable_value(&mut p.target, Some(*id), name)
                        .changed();
                }
            });
    }
    changed
}

fn axis_ui(ui: &mut egui::Ui, p: &mut Params) -> bool {
    let mut changed = false;
    ui.horizontal(|ui| {
        ui.label("Axis");
        for (axis, name) in [
            (OriginAxis::X, "X"),
            (OriginAxis::Y, "Y"),
            (OriginAxis::Z, "Z"),
        ] {
            changed |= ui
                .selectable_value(&mut p.axis, Some(AxisRef::Origin(axis)), name)
                .changed();
        }
        if let Some(AxisRef::SketchLine { .. }) = p.axis {
            ui.label("sketch line");
        } else {
            ui.label(egui::RichText::new("or click a sketch line").weak());
        }
    });
    changed
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::editor::Mode;
    use crate::editor::harness::{Harness, block, click_at, top_face};
    use crate::editor::selection::Pick;
    use crate::editor::sketch_mode::SketchTool;
    use basset_core::OriginPlane;
    use basset_math::Vec2;

    /// A 20×4 slot 3 mm thick: the rim of its top face is two straight edges and two
    /// half-round ones, which is the geometry a tangent chain exists for.
    fn slot(h: &mut Harness) -> BodyRef {
        h.start_sketch(PlaneRef::Origin(OriginPlane::XY));
        let camera = h.editor.camera;
        let window = h.editor.window_px;
        let s = h.sketch();
        s.snap_to_grid = false;
        s.set_tool(SketchTool::SlotOverall);
        for p in [Vec2::ZERO, Vec2::new(20.0, 0.0), Vec2::new(10.0, 2.0)] {
            s.pointer_up(&click_at(p.x, p.y), &camera, window, true, false);
        }
        h.editor.commit_sketch();
        h.finish_sketch(true);
        h.extrude(Vec2::new(10.0, 0.0), 3.0)
    }

    /// One pick of a slot's top rim takes the whole rim, because the user drew one
    /// outline; picking any of it again lets all of it go. Ctrl is how they get at the
    /// single edge instead.
    #[test]
    fn a_fillet_pick_takes_the_tangent_chain_and_gives_it_back() {
        let mut h = Harness::new();
        let body = slot(&mut h);
        h.start_tool(ToolKind::Fillet);
        let rim = edges_of_face(&h.editor, &top_face(body));
        assert_eq!(rim.len(), 4, "two flats and two round ends");

        h.editor.apply_pick(Some(Pick::Edge(rim[0], 0.0)), false);
        assert_eq!(
            h.editor.selection.edges.len(),
            4,
            "the rim went in together"
        );
        // A window egui has not laid out before spends its first frame sizing itself,
        // so the frame that shows its contents is the second one.
        h.frame();
        assert!(
            h.frame().has_text("4 edges selected"),
            "{:?}",
            h.frame().text()
        );

        // Any edge of the chain lets the whole chain go, not only the one picked first.
        h.editor.apply_pick(Some(Pick::Edge(rim[2], 0.0)), false);
        assert!(h.editor.selection.edges.is_empty());

        h.set_modifiers(false, true);
        h.editor.apply_pick(Some(Pick::Edge(rim[0], 0.0)), false);
        assert_eq!(
            h.editor.selection.edges,
            vec![rim[0]],
            "Ctrl takes one edge"
        );
        h.editor.apply_pick(Some(Pick::Edge(rim[0], 0.0)), false);
        assert!(h.editor.selection.edges.is_empty(), "and gives it back");

        // The same switch lives in the dialog for the user who wants it every time.
        h.set_modifiers(false, false);
        h.editor.tool.as_mut().unwrap().params.tangent_chain = false;
        h.editor.apply_pick(Some(Pick::Edge(rim[0], 0.0)), false);
        assert_eq!(h.editor.selection.edges, vec![rim[0]]);
        assert!(
            h.frame().has_text("1 edge selected"),
            "{:?}",
            h.frame().text()
        );
    }

    /// The radius arrow points at the material the fillet works on: into the body at a
    /// convex edge. It used to point straight out of it.
    #[test]
    fn the_fillet_arrow_points_into_the_corner() {
        let mut editor = Editor::new(None);
        editor.window_px = [800, 600];
        let body = block(&mut editor);
        start_tool(&mut editor, ToolKind::Fillet);
        let ring = edges_of_face(&editor, &top_face(body));
        editor.selection.edges.push(ring[0]);
        let h = handle(&editor).expect("a fillet handle");
        assert!(
            (h.tip.distance(h.origin) - editor.tool.as_ref().unwrap().params.radius).abs() < 1e-9
        );
        // The block spans z 0..2 and x, y 0..10; a step along the arrow from an edge of
        // the top face lands inside it.
        assert!(h.tip.z < h.origin.z, "{:?}", h.tip);
        let inside = |v: f64| (0.0..=10.0).contains(&v);
        assert!(inside(h.tip.x) && inside(h.tip.y), "{:?}", h.tip);
        assert!(matches!(editor.mode, Mode::Model));
    }

    /// The block is 10 mm across and 2 mm thick, so a fillet round the rim of its top
    /// face is bounded by the thickness: past 2 mm the round has eaten the whole side
    /// and is taking material from under it. The dialog says so before the kernel has
    /// to, and a radius typed past it anyway comes back as a refusal rather than as a
    /// body that quietly stops changing.
    #[test]
    fn a_fillet_is_bounded_by_the_material_and_the_dialog_says_by_how_much() {
        let mut h = Harness::new();
        let body = h.block();
        h.start_tool(ToolKind::Fillet);
        h.editor
            .apply_pick(Some(Pick::Face(top_face(body), 0.0)), false);
        let limit = h
            .editor
            .tool
            .as_ref()
            .unwrap()
            .blend_limit()
            .expect("a limit for the top rim");
        let thickness = h.editor.pick_body(body).unwrap().solid.aabb().extent().z;
        assert!(
            (limit - thickness).abs() < 1e-6,
            "the rim of a {thickness} mm wall took {limit}"
        );
        h.frame();
        assert!(
            h.frame().has_text(&format!("up to {limit:.2} mm fits")),
            "{:?}",
            h.frame().text()
        );

        h.editor.tool.as_mut().unwrap().params.radius = limit * 2.0;
        sync_tool(&mut h.editor);
        let feature = h.editor.tool.as_ref().unwrap().feature.unwrap();
        assert!(
            matches!(
                h.editor.doc.state().status(feature),
                Some(FeatureStatus::Failed(_))
            ),
            "twice the limit went through"
        );
        h.frame();
        assert!(
            h.frame()
                .has_text("runs past the material it has to work with"),
            "{:?}",
            h.frame().text()
        );
        assert!(
            h.frame().has_text(&format!("only {limit:.2} mm fits")),
            "{:?}",
            h.frame().text()
        );
    }
}
