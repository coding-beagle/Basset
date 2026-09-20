//! The Measure tool: what the user picked, and what it measures out to.
//!
//! Measuring is the one modelling action that must leave no trace. It therefore does not
//! go through [`Tool`](super::tools::Tool), which owns a timeline feature and a document
//! transaction by design: a measure tool built on that could only ever be a feature that
//! is careful not to commit. Instead the editor holds a [`Measure`] of its own, the picks
//! are mirrored into the ordinary [`Selection`](super::Selection) so the geometry
//! highlights exactly as it does everywhere else, and the document is never touched.
//!
//! The readout is derived on demand rather than cached, so it can never disagree with the
//! model it describes — an undo behind the tool's back re-reads rather than goes stale.
//! Everything geometric is a call into [`basset_kernel::measure`]; this module only picks
//! which question to ask and writes the answer as a person reads it.

use basset_core::{BodyRef, EdgeRef, FaceRef, FeatureId};
use basset_kernel::{Edge, Face, Solid, measure as query};
use basset_math::Vec3;
use basset_sketch::{Entity, EntityId};

use super::Editor;
use super::selection::{Pick, SelectionFilter};

/// One thing the user picked to measure.
///
/// A body corner and a sketch point are both just a position: nothing downstream asks
/// which it was, and collapsing them here is what lets a sketch point be measured against
/// a corner of the solid it drove.
#[derive(Clone, Debug, PartialEq)]
pub enum Subject {
    Point(Vec3),
    Edge(EdgeRef),
    Face(FaceRef),
    Body(BodyRef),
}

/// What the viewport draws: lines of text anchored on the geometry they describe, and,
/// for a measurement between two things, the span it was taken across.
#[derive(Clone, Debug, PartialEq)]
pub struct Readout {
    pub anchor: Vec3,
    pub lines: Vec<String>,
    pub leader: Option<[Vec3; 2]>,
}

/// The tool's whole state: at most two picks. A third starts again, which is how every
/// CAD measure tool behaves and saves the user a trip to Escape between measurements.
#[derive(Clone, Debug, Default)]
pub struct Measure {
    pub subjects: Vec<Subject>,
}

/// What a click may land on while measuring: everything with a size, and nothing that has
/// none. Planes and sketch regions are left out because neither has a measurement the
/// tool can give — an origin plane is infinite, and a region's area belongs to the face
/// it becomes.
pub const FILTER: SelectionFilter = SelectionFilter {
    faces: true,
    edges: true,
    vertices: true,
    points: true,
    planes: false,
    profiles: false,
    curves: false,
};

pub const PROMPT: &str = "Measure: click an edge, face or corner; a second for distance or angle. \
     Ctrl-click a face for its whole body. Esc to finish.";

pub fn start(editor: &mut Editor) {
    if editor.is_sketching() {
        return;
    }
    if editor.tool.is_some() {
        super::tools::cancel_tool(editor);
    }
    editor.selection.clear();
    editor.hover = None;
    editor.measure = Some(Measure::default());
    editor.set_status(PROMPT);
    editor.request_repaint();
}

pub fn stop(editor: &mut Editor) {
    if editor.measure.take().is_some() {
        editor.selection.clear();
        editor.set_status("Measure finished");
        editor.request_repaint();
    }
}

/// Records a click while measuring. `whole_body` is the Ctrl modifier: the same click on
/// the same face, asking about the body it belongs to instead.
///
/// Returns whether the click was consumed, so the ordinary selection path can be left
/// alone entirely — a measurement must never be a selection that a later tool acts on.
pub fn clicked(editor: &mut Editor, pick: Option<&Pick>, whole_body: bool) -> bool {
    if editor.measure.is_none() {
        return false;
    }
    let subject = pick.and_then(|p| subject_of(editor, p, whole_body));
    let Some(measure) = editor.measure.as_mut() else {
        return false;
    };
    match subject {
        // A click on nothing clears, so there is always a way to put the readout away
        // without leaving the tool.
        None => measure.subjects.clear(),
        Some(s) => {
            // A third pick starts again, as does picking the same thing twice. So does a
            // body, either side of the pair: its volume is not relative to anything, so
            // it is always measured on its own.
            let solitary = |x: &Subject| matches!(x, Subject::Body(_));
            if measure.subjects.len() >= 2
                || measure.subjects.contains(&s)
                || solitary(&s)
                || measure.subjects.iter().any(solitary)
            {
                measure.subjects.clear();
            }
            measure.subjects.push(s);
        }
    }
    mirror_selection(editor);
    editor.set_status(match readout(editor) {
        Some(r) => r.lines.join("  ·  "),
        None => PROMPT.to_string(),
    });
    editor.request_repaint();
    true
}

fn subject_of(editor: &Editor, pick: &Pick, whole_body: bool) -> Option<Subject> {
    match pick {
        Pick::Vertex(v, _) => Some(Subject::Point(v.point)),
        Pick::Point { sketch, entity, .. } => {
            sketch_point(editor, *sketch, *entity).map(Subject::Point)
        }
        Pick::Edge(e, _) => Some(Subject::Edge(*e)),
        Pick::Face(f, _) if whole_body => Some(Subject::Body(f.body)),
        Pick::Face(f, _) => Some(Subject::Face(*f)),
        Pick::Plane(..) | Pick::Profile(..) | Pick::Curve { .. } => None,
    }
}

fn sketch_point(editor: &Editor, sketch: FeatureId, entity: EntityId) -> Option<Vec3> {
    let (_, solved) = editor
        .visible_sketches()
        .into_iter()
        .find(|(id, _)| *id == sketch)?;
    match solved.sketch.entity(entity)?.entity {
        Entity::Point { pos } => Some(solved.frame.to_world(pos)),
        _ => None,
    }
}

/// Puts the picks into the ordinary selection so the viewport highlights them the way it
/// highlights everything else. The selection is a view of the measurement here, never the
/// other way round: it is rebuilt from scratch on every pick.
fn mirror_selection(editor: &mut Editor) {
    let subjects = match &editor.measure {
        Some(m) => m.subjects.clone(),
        None => return,
    };
    editor.selection.clear();
    for s in &subjects {
        match s {
            Subject::Edge(e) => editor.selection.edges.push(*e),
            Subject::Face(f) => {
                editor.selection.faces.push(*f);
                if !editor.selection.bodies.contains(&f.body) {
                    editor.selection.bodies.push(f.body);
                }
            }
            // A body with no face of it selected is what the scene draws as a whole
            // selected body, which is exactly what a body measurement should look like.
            Subject::Body(b) => editor.selection.bodies.push(*b),
            Subject::Point(_) => {}
        }
    }
    // A picked point kept only its position, so which corner or sketch point it was is
    // found again here. Position is the identity a corner has anyway — the kernel gives
    // it no key — so nothing is lost by re-deriving it, and a `Subject` that carried its
    // provenance would make the same point picked two ways compare unequal.
    for s in &subjects {
        let Subject::Point(p) = s else { continue };
        let corner = editor.doc_state_bodies().into_iter().find(|body| {
            editor.pick_body(*body).is_some_and(|pick| {
                basset_kernel::corners(&pick.edges)
                    .iter()
                    .any(|c| c.distance(*p) < 1e-9)
            })
        });
        if let Some(body) = corner {
            editor
                .selection
                .vertices
                .push(super::selection::VertexHit { body, point: *p });
            continue;
        }
        for (id, solved) in editor.visible_sketches() {
            let found = solved.sketch.entities().find(|(_, data)| {
                matches!(data.entity, Entity::Point { pos } if solved.frame.to_world(pos).distance(*p) < 1e-9)
            });
            if let Some((entity, _)) = found {
                editor.selection.points.push((id, entity));
                break;
            }
        }
    }
}

/// The measurement the current picks make, ready to draw.
pub fn readout(editor: &Editor) -> Option<Readout> {
    let subjects = &editor.measure.as_ref()?.subjects;
    let lines = match subjects.as_slice() {
        [] => return None,
        [one] => describe_one(editor, one)?,
        [a, b] => describe_pair(editor, a, b),
        _ => return None,
    };
    let anchors: Vec<Vec3> = subjects
        .iter()
        .filter_map(|s| anchor_of(editor, s))
        .collect();
    let anchor = match anchors.as_slice() {
        [] => return None,
        [a] => *a,
        [a, b, ..] => (*a + *b) * 0.5,
    };
    Some(Readout {
        anchor,
        lines,
        leader: match anchors.as_slice() {
            [a, b, ..] => Some([*a, *b]),
            _ => None,
        },
    })
}

fn describe_one(editor: &Editor, subject: &Subject) -> Option<Vec<String>> {
    Some(match subject {
        Subject::Point(p) => vec![
            "Vertex".into(),
            format!("X {}", mm(p.x)),
            format!("Y {}", mm(p.y)),
            format!("Z {}", mm(p.z)),
        ],
        Subject::Edge(e) => {
            let edge = edge_of(editor, e)?;
            let mut lines = vec!["Edge".into(), format!("Length {}", mm(edge.length()))];
            if let Some(circle) = query::edge_circle(edge) {
                lines.push(format!("Radius {}", mm(circle.radius)));
                lines.push(format!("Diameter {}", mm(circle.radius * 2.0)));
            }
            lines
        }
        Subject::Face(f) => {
            let face = face_of(editor, f)?;
            let mut lines = vec![
                "Face".into(),
                format!("Area {}", area(face.area())),
                format!("Perimeter {}", mm(query::face_perimeter(face))),
            ];
            if let Some(radius) = query::face_radius(face) {
                lines.push(format!("Radius {}", mm(radius)));
                lines.push(format!("Diameter {}", mm(radius * 2.0)));
            }
            lines
        }
        Subject::Body(b) => {
            let solid = solid_of(editor, *b)?;
            let box_ = solid.aabb();
            let size = box_.extent();
            vec![
                format!("Body {}", editor.body_name(*b)),
                format!("Volume {}", volume(solid.volume().abs())),
                format!("Surface area {}", area(solid.surface_area())),
                format!(
                    "Bounding box {:.3} × {:.3} × {:.3} mm",
                    size.x, size.y, size.z
                ),
            ]
        }
    })
}

/// Two picks. Every arm answers with the one measurement that pair has, and says plainly
/// when it has none rather than making something up: "angle between a body and an edge"
/// is not a question with an answer, and a blank readout would read as a broken tool.
fn describe_pair(editor: &Editor, a: &Subject, b: &Subject) -> Vec<String> {
    let nothing = || vec!["No measurement for this pair".to_string()];
    let lines = match (a, b) {
        (Subject::Point(p), Subject::Point(q)) => {
            let d = *q - *p;
            Some(vec![
                format!("Distance {}", mm(p.distance(*q))),
                format!("ΔX {}", mm(d.x)),
                format!("ΔY {}", mm(d.y)),
                format!("ΔZ {}", mm(d.z)),
            ])
        }
        (Subject::Point(p), Subject::Edge(e)) | (Subject::Edge(e), Subject::Point(p)) => {
            edge_of(editor, e).map(|edge| {
                vec![format!(
                    "Distance to edge {}",
                    mm(query::point_edge_distance(*p, edge))
                )]
            })
        }
        (Subject::Point(p), Subject::Face(f)) | (Subject::Face(f), Subject::Point(p)) => {
            face_of(editor, f)
                .and_then(|face| query::point_face_distance(*p, face))
                .map(|d| vec![format!("Distance to face {}", mm(d))])
        }
        (Subject::Edge(x), Subject::Edge(y)) => {
            match (edge_of(editor, x), edge_of(editor, y)) {
                (Some(x), Some(y)) => {
                    let mut lines = Vec::new();
                    match query::edge_angle(x, y) {
                        // Parallel lines have no angle worth printing; the gap between
                        // them is the measurement the user is after.
                        Some(angle) if angle.to_degrees() < 1e-6 => {
                            lines.push("Parallel".into());
                        }
                        Some(angle) => lines.push(format!("Angle {}", deg(angle))),
                        None => {}
                    }
                    lines.push(format!(
                        "Minimum distance {}",
                        mm(query::edge_distance(x, y))
                    ));
                    Some(lines)
                }
                _ => None,
            }
        }
        (Subject::Edge(e), Subject::Face(f)) | (Subject::Face(f), Subject::Edge(e)) => {
            match (edge_of(editor, e), face_of(editor, f)) {
                (Some(edge), Some(face)) => match query::edge_face_distance(edge, face) {
                    Some(d) => Some(vec!["Parallel".into(), format!("Distance {}", mm(d))]),
                    None => query::edge_face_angle(edge, face)
                        .map(|angle| vec![format!("Angle {}", deg(angle))]),
                },
                _ => None,
            }
        }
        (Subject::Face(x), Subject::Face(y)) => match (face_of(editor, x), face_of(editor, y)) {
            (Some(x), Some(y)) => match query::parallel_face_distance(x, y) {
                Some(d) => Some(vec!["Parallel".into(), format!("Distance {}", mm(d))]),
                None => query::face_angle(x, y).map(|angle| vec![format!("Angle {}", deg(angle))]),
            },
            _ => None,
        },
        // Picking never pairs a body with anything (see `clicked`); this arm is what
        // keeps the match total.
        (Subject::Body(_), _) | (_, Subject::Body(_)) => None,
    };
    lines.filter(|l| !l.is_empty()).unwrap_or_else(nothing)
}

/// Where the readout for one pick hangs: on the thing itself, so the number is beside
/// what it describes and not in the middle of the screen.
fn anchor_of(editor: &Editor, subject: &Subject) -> Option<Vec3> {
    match subject {
        Subject::Point(p) => Some(*p),
        Subject::Edge(e) => edge_of(editor, e).and_then(edge_midpoint),
        Subject::Face(f) => face_of(editor, f).map(Face::centroid),
        Subject::Body(b) => solid_of(editor, *b).map(|s| s.aabb().center()),
    }
}

/// Halfway along the edge by arc length, so a rim's label sits on the rim rather than at
/// the centre of the circle it encloses.
fn edge_midpoint(edge: &Edge) -> Option<Vec3> {
    let chain = edge.chains().into_iter().max_by_key(Vec::len)?;
    let total: f64 = chain.iter().map(|s| s.start.distance(s.end)).sum();
    let mut walked = 0.0;
    for s in &chain {
        let len = s.start.distance(s.end);
        if walked + len >= total * 0.5 && len > 0.0 {
            return Some(s.start.lerp(s.end, (total * 0.5 - walked) / len));
        }
        walked += len;
    }
    chain.first().map(|s| s.start)
}

fn face_of<'a>(editor: &'a Editor, f: &FaceRef) -> Option<&'a Face> {
    editor.pick_body(f.body)?.solid.face(f.key)
}

fn edge_of<'a>(editor: &'a Editor, e: &EdgeRef) -> Option<&'a Edge> {
    editor
        .pick_body(e.body)?
        .edges
        .iter()
        .find(|edge| edge.key == e.key)
}

fn solid_of(editor: &Editor, b: BodyRef) -> Option<&Solid> {
    editor.pick_body(b).map(|p| p.solid.as_ref())
}

// Dimensions read the way they do everywhere else in the app: three decimals of a
// millimetre, two of a degree.
fn mm(v: f64) -> String {
    format!("{v:.3} mm")
}

fn deg(radians: f64) -> String {
    format!("{:.2}\u{b0}", radians.to_degrees())
}

fn area(v: f64) -> String {
    format!("{v:.3} mm\u{b2}")
}

fn volume(v: f64) -> String {
    format!("{v:.3} mm\u{b3}")
}
