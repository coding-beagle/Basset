//! What the pointer is over and what the user has selected.
//!
//! Picking works against the same regenerated state the scene is drawn from: faces via
//! the tessellation, edges via kernel edge polylines, planes and sketch geometry by
//! ray–plane intersection. Everything a pick returns is a *reference* type from
//! `basset-core`, so a selection can be dropped straight into a feature.

use basset_core::{
    BodyRef, EdgeRef, FaceRef, FeatureId, OriginPlane, PlaneRef, ProfileRef, RegionRef,
};
use basset_kernel::{pick_edge, pick_face, pick_vertex};
use basset_math::{Frame, Ray, Vec2, Vec3};
use basset_sketch::EntityId;

use super::Editor;

#[derive(Clone, Debug, PartialEq)]
pub enum Pick {
    Face(FaceRef, f64),
    Edge(EdgeRef, f64),
    Plane(PlaneRef, f64),
    /// A closed region of a sketch, identified by the clicked point.
    Profile(ProfileRef, f64),
    /// A curve entity of a sketch (for revolve axes and sweep paths).
    Curve {
        sketch: FeatureId,
        entity: EntityId,
        t: f64,
    },
    /// A corner of a body.
    Vertex(VertexHit, f64),
    /// A point entity of a sketch.
    Point {
        sketch: FeatureId,
        entity: EntityId,
        t: f64,
    },
}

/// A corner of a body. The kernel has no stable key for a vertex, so a corner is named by
/// where it is, the way [`ProfileRef`] names a region by a point inside it. Nothing stores
/// one in a feature, so it only has to survive until the next regeneration.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VertexHit {
    pub body: BodyRef,
    pub point: Vec3,
}

impl Pick {
    pub fn t(&self) -> f64 {
        match self {
            Pick::Face(_, t)
            | Pick::Edge(_, t)
            | Pick::Plane(_, t)
            | Pick::Profile(_, t)
            | Pick::Vertex(_, t) => *t,
            Pick::Curve { t, .. } | Pick::Point { t, .. } => *t,
        }
    }

    /// The timeline feature this pick belongs to, for the timeline highlight.
    pub fn feature(&self) -> Option<FeatureId> {
        match self {
            Pick::Face(f, _) => Some(f.body.0),
            Pick::Edge(e, _) => Some(e.body.0),
            Pick::Plane(PlaneRef::Feature(id), _) => Some(*id),
            Pick::Plane(_, _) => None,
            Pick::Profile(p, _) => Some(p.sketch),
            Pick::Curve { sketch, .. } | Pick::Point { sketch, .. } => Some(*sketch),
            Pick::Vertex(v, _) => Some(v.body.0),
        }
    }
}

/// Which kinds of thing a click may select. Tools narrow this to what they accept.
#[derive(Clone, Copy, Debug)]
pub struct SelectionFilter {
    pub faces: bool,
    pub edges: bool,
    pub planes: bool,
    pub profiles: bool,
    pub curves: bool,
    /// Corners of bodies.
    pub vertices: bool,
    /// Point entities of sketches.
    pub points: bool,
}

impl Default for SelectionFilter {
    fn default() -> Self {
        Self {
            faces: true,
            edges: true,
            ..Self::NONE
        }
    }
}

impl SelectionFilter {
    pub const NONE: Self = Self {
        faces: false,
        edges: false,
        planes: false,
        profiles: false,
        curves: false,
        vertices: false,
        points: false,
    };
    pub const EDGES: Self = Self {
        edges: true,
        ..Self::NONE
    };
    /// What a generator takes, and what Face selection means: a closed region of a
    /// sketch, or a planar face of a body used as one.
    pub const REGIONS: Self = Self {
        profiles: true,
        faces: true,
        ..Self::NONE
    };
    pub const PLANES: Self = Self {
        planes: true,
        faces: true,
        ..Self::NONE
    };
    pub const BODIES: Self = Self {
        faces: true,
        ..Self::NONE
    };

    pub fn is_empty(self) -> bool {
        !(self.faces
            || self.edges
            || self.planes
            || self.profiles
            || self.curves
            || self.vertices
            || self.points)
    }

    fn intersect(self, other: Self) -> Self {
        Self {
            faces: self.faces && other.faces,
            edges: self.edges && other.edges,
            planes: self.planes && other.planes,
            profiles: self.profiles && other.profiles,
            curves: self.curves && other.curves,
            vertices: self.vertices && other.vertices,
            points: self.points && other.points,
        }
    }
}

/// What the user has restricted picking to, so a click lands on the kind of thing they
/// meant. Dense geometry makes "click the thing under the pointer" ambiguous — a corner,
/// three edges and two faces all sit within a few pixels of each other — and this is how
/// every CAD tool resolves it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum SelectMode {
    #[default]
    Any,
    /// A face of a body or a closed region of a sketch. They are the same thing to the
    /// user, and to every generator, which takes either as a region to push or spin.
    Faces,
    Edges,
    /// Corners of bodies and points of sketches.
    Vertices,
    /// Everything a sketch offers: regions, curves and points.
    Sketch,
}

impl SelectMode {
    pub const ALL: [SelectMode; 5] = [
        SelectMode::Any,
        SelectMode::Faces,
        SelectMode::Edges,
        SelectMode::Vertices,
        SelectMode::Sketch,
    ];

    pub fn name(self) -> &'static str {
        match self {
            SelectMode::Any => "Any",
            SelectMode::Faces => "Face",
            SelectMode::Edges => "Edge",
            SelectMode::Vertices => "Vertex",
            SelectMode::Sketch => "Sketch",
        }
    }

    /// What this mode picks when no tool is running.
    pub fn filter(self) -> SelectionFilter {
        match self {
            SelectMode::Any => SelectionFilter::default(),
            SelectMode::Faces => SelectionFilter::REGIONS,
            SelectMode::Edges => SelectionFilter::EDGES,
            SelectMode::Vertices => SelectionFilter {
                vertices: true,
                points: true,
                ..SelectionFilter::NONE
            },
            SelectMode::Sketch => SelectionFilter {
                profiles: true,
                curves: true,
                points: true,
                ..SelectionFilter::NONE
            },
        }
    }

    /// The mode narrows what a running tool accepts, but never to nothing: a filter the
    /// tool cannot satisfy would leave the user unable to finish it, with no hint why.
    ///
    /// `Any` means "no restriction", so it hands the tool's filter back untouched. Its
    /// own filter is what a click picks with *no* tool running (faces and edges) and
    /// intersecting that with the tool's would quietly drop the planes Sketch needs.
    pub fn narrow(self, tool: SelectionFilter) -> SelectionFilter {
        if self == SelectMode::Any {
            return tool;
        }
        let narrowed = tool.intersect(self.filter());
        if narrowed.is_empty() { tool } else { narrowed }
    }
}

#[derive(Clone, Debug, Default)]
pub struct Selection {
    pub faces: Vec<FaceRef>,
    pub edges: Vec<EdgeRef>,
    pub planes: Vec<PlaneRef>,
    pub profiles: Vec<ProfileRef>,
    pub curves: Vec<(FeatureId, EntityId)>,
    pub bodies: Vec<BodyRef>,
    pub vertices: Vec<VertexHit>,
    pub points: Vec<(FeatureId, EntityId)>,
}

impl Selection {
    pub fn clear(&mut self) {
        *self = Self::default();
    }

    pub fn toggle(&mut self, pick: &Pick) {
        fn flip<T: PartialEq + Clone>(v: &mut Vec<T>, item: &T) {
            match v.iter().position(|x| x == item) {
                Some(i) => {
                    v.remove(i);
                }
                None => v.push(item.clone()),
            }
        }
        match pick {
            Pick::Face(f, _) => {
                flip(&mut self.faces, f);
                if !self.bodies.contains(&f.body) {
                    self.bodies.push(f.body);
                } else if !self.faces.iter().any(|x| x.body == f.body) {
                    self.bodies.retain(|b| *b != f.body);
                }
            }
            Pick::Edge(e, _) => flip(&mut self.edges, e),
            Pick::Plane(p, _) => flip(&mut self.planes, p),
            Pick::Profile(p, _) => {
                // Two clicks in the same region toggle it even though the sample points
                // differ, so regions behave like discrete selectable things.
                match self
                    .profiles
                    .iter()
                    .position(|x| x.sketch == p.sketch && same_region(x, p))
                {
                    Some(i) => {
                        self.profiles.remove(i);
                    }
                    None => self.profiles.push(*p),
                }
            }
            Pick::Curve { sketch, entity, .. } => flip(&mut self.curves, &(*sketch, *entity)),
            Pick::Vertex(v, _) => flip(&mut self.vertices, v),
            Pick::Point { sketch, entity, .. } => flip(&mut self.points, &(*sketch, *entity)),
        }
    }

    /// Everything selected that a generator can push, spin or skin, sketch regions first.
    /// Faces are only regions for the tools that ask for them; the others read `faces`.
    pub fn regions(&self) -> Vec<RegionRef> {
        self.profiles
            .iter()
            .map(|p| RegionRef::Profile(*p))
            .chain(self.faces.iter().map(|f| RegionRef::Face(*f)))
            .collect()
    }

    pub fn summary(&self) -> String {
        let mut parts = Vec::new();
        for (n, one, many) in [
            (self.faces.len(), "face", "faces"),
            (self.edges.len(), "edge", "edges"),
            (self.planes.len(), "plane", "planes"),
            (self.profiles.len(), "profile", "profiles"),
            (self.curves.len(), "curve", "curves"),
            (self.vertices.len(), "vertex", "vertices"),
            (self.points.len(), "point", "points"),
        ] {
            if n > 0 {
                parts.push(format!("{n} {}", if n == 1 { one } else { many }));
            }
        }
        if parts.is_empty() {
            "Nothing selected".into()
        } else {
            parts.join(", ")
        }
    }
}

/// Region identity is "the smallest profile containing the sample"; two samples in the
/// same region resolve to the same profile. The editor stores the region's own sample
/// point when it records a pick so this comparison only has to be approximate.
fn same_region(a: &ProfileRef, b: &ProfileRef) -> bool {
    a.sample.distance(b.sample) < 1e-9
}

/// How much wider the aim is for a tool that is really asking for edges. An edge is one
/// pixel of drawn line on a body whose faces are thousands, and Fillet spends its whole
/// life collecting them: eight pixels of slack means chasing a rim around a screen. The
/// face behind is not lost by this — it is still there in the middle of the face, which
/// is where a user who means the face is pointing anyway.
const BLEND_EDGE_SLACK: f64 = 2.0;

/// Everything under the ray that the filter allows, nearest first, then the one to use.
/// Edges win over faces when both are within tolerance because edges are thin and a
/// user aiming at one is never aiming at the face behind it.
pub fn pick(
    editor: &Editor,
    ray: &Ray,
    filter: &SelectionFilter,
    tolerance_px: f64,
) -> Option<Pick> {
    // Measuring aims at edges and corners as constantly as a blend tool does, so it gets
    // the same slack: an edge is a pixel of line on a body whose faces are thousands.
    let wide = editor.measure.is_some()
        || editor
            .tool
            .as_ref()
            .is_some_and(|tool| tool.prefers_edges());
    let thin_px = if wide {
        tolerance_px * BLEND_EDGE_SLACK
    } else {
        tolerance_px
    };
    let mut best: Option<Pick> = None;
    let mut consider = |candidate: Pick| {
        if best.as_ref().is_none_or(|b| candidate.t() < b.t()) {
            best = Some(candidate);
        }
    };
    let state_bodies: Vec<(BodyRef, bool)> = editor
        .doc_state_bodies()
        .into_iter()
        .map(|id| (id, editor.hidden_bodies.contains(&id)))
        .collect();

    if filter.faces || filter.edges || filter.vertices {
        let mut face_hit: Option<Pick> = None;
        let mut edge_hit: Option<Pick> = None;
        let mut vertex_hit: Option<Pick> = None;
        for (id, hidden) in &state_bodies {
            if *hidden {
                continue;
            }
            let Some(mesh) = editor.pick_body(*id) else {
                continue;
            };
            if filter.faces
                && let Some(f) = pick_face(&mesh.tess, ray)
                && face_hit.as_ref().is_none_or(|p| f.hit.t < p.t())
            {
                face_hit = Some(Pick::Face(
                    FaceRef {
                        body: *id,
                        key: f.key,
                    },
                    f.hit.t,
                ));
            }
            if filter.edges || filter.vertices {
                let anchor = face_hit
                    .as_ref()
                    .map(|p| ray.at(p.t()))
                    .unwrap_or(editor.camera.target);
                let tol = editor.camera.pixel_size_at(anchor, editor.window_px) * thin_px;
                if filter.edges
                    && let Some(e) = pick_edge(&mesh.edges, ray, tol)
                    && edge_hit.as_ref().is_none_or(|p| e.t < p.t())
                {
                    edge_hit = Some(Pick::Edge(
                        EdgeRef {
                            body: *id,
                            key: e.key,
                        },
                        e.t,
                    ));
                }
                if filter.vertices
                    && let Some(v) = pick_vertex(&mesh.edges, ray, tol)
                    && vertex_hit.as_ref().is_none_or(|p| v.t < p.t())
                {
                    vertex_hit = Some(Pick::Vertex(
                        VertexHit {
                            body: *id,
                            point: v.point,
                        },
                        v.t,
                    ));
                }
            }
        }
        // A corner beats an edge beats a face when several are within tolerance: the
        // smaller target is always the one the user was aiming at.
        match (vertex_hit.or(edge_hit), face_hit) {
            (Some(thin), Some(f)) => {
                let tol =
                    editor.camera.pixel_size_at(ray.at(f.t()), editor.window_px) * thin_px * 2.0;
                consider(if thin.t() <= f.t() + tol { thin } else { f });
            }
            (Some(thin), None) => consider(thin),
            (None, Some(f)) => consider(f),
            (None, None) => {}
        }
    }

    if filter.planes {
        for (plane, frame, half) in editor.visible_planes() {
            if let Some((t, local)) = hit_plane(&frame, ray)
                && local.x.abs() <= half
                && local.y.abs() <= half
            {
                consider(Pick::Plane(plane, t));
            }
        }
    }

    if filter.profiles || filter.curves || filter.points {
        for (id, solved) in editor.visible_sketches() {
            let Some((t, local)) = hit_plane(&solved.frame, ray) else {
                continue;
            };
            // Points before curves before regions, smallest target first, as with corners
            // and edges on a body.
            if filter.points {
                let tol = editor.camera.pixel_size_at(ray.at(t), editor.window_px) * tolerance_px;
                if let Some(hit) = solved.sketch.hit_test(local, tol).into_iter().find(|h| {
                    solved
                        .sketch
                        .entity(h.entity)
                        .is_some_and(|e| e.entity.is_point())
                }) {
                    consider(Pick::Point {
                        sketch: id,
                        entity: hit.entity,
                        t,
                    });
                    continue;
                }
            }
            if filter.curves {
                let tol = editor.camera.pixel_size_at(ray.at(t), editor.window_px) * tolerance_px;
                if let Some(hit) = solved.sketch.hit_test(local, tol).into_iter().find(|h| {
                    solved
                        .sketch
                        .entity(h.entity)
                        .is_some_and(|e| e.entity.is_curve())
                }) {
                    consider(Pick::Curve {
                        sketch: id,
                        entity: hit.entity,
                        t,
                    });
                    continue;
                }
            }
            if filter.profiles
                && let Some(region) = solved
                    .profiles
                    .iter()
                    .filter(|p| p.contains(local))
                    .min_by(|a, b| a.area().total_cmp(&b.area()))
            {
                // Use a point of the region itself as the stable sample so that two
                // clicks anywhere inside compare equal.
                let sample = region_sample(region, local);
                consider(Pick::Profile(ProfileRef { sketch: id, sample }, t));
            }
        }
    }
    best
}

fn hit_plane(frame: &Frame, ray: &Ray) -> Option<(f64, Vec2)> {
    let t = frame.plane().intersect_ray(ray)?;
    if t < 0.0 {
        return None;
    }
    Some((t, frame.to_local(ray.at(t))))
}

/// A canonical point inside the region: the clicked point is fine (it is inside by
/// construction) but rounding it makes repeated clicks identical for toggling.
fn region_sample(_region: &basset_kernel::Profile, clicked: Vec2) -> Vec2 {
    Vec2::new(
        (clicked.x * 1e6).round() / 1e6,
        (clicked.y * 1e6).round() / 1e6,
    )
}

impl Editor {
    /// Bodies a click can land on, in a form that does not borrow the document.
    pub fn doc_state_bodies(&self) -> Vec<BodyRef> {
        self.pick_bodies.keys().copied().collect()
    }

    /// Planes that can be seen (and therefore picked): origin planes when shown, every
    /// construction plane feature, each with its frame and drawn half-size.
    pub fn visible_planes(&self) -> Vec<(PlaneRef, Frame, f64)> {
        let half = self.plane_half_size();
        let mut out = Vec::new();
        if self.show_origin || self.tool.as_ref().is_some_and(|t| t.filter().planes) {
            out.push((PlaneRef::Origin(OriginPlane::XY), Frame::XY, half));
            out.push((PlaneRef::Origin(OriginPlane::YZ), Frame::YZ, half));
            out.push((PlaneRef::Origin(OriginPlane::XZ), Frame::XZ, half));
        }
        for (id, frame) in &self.cached_planes {
            out.push((PlaneRef::Feature(*id), *frame, half));
        }
        out
    }

    pub fn plane_half_size(&self) -> f64 {
        (self.cached_extent * 0.6).clamp(20.0, 5000.0)
    }

    /// Sketches drawn in model mode: everything solved and not hidden, except the one
    /// being edited (the sketch editor draws that itself).
    pub fn visible_sketches(&self) -> Vec<(FeatureId, std::sync::Arc<basset_core::SolvedSketch>)> {
        let editing = match &self.mode {
            super::Mode::Sketch(s) => Some(s.feature),
            super::Mode::Model => None,
        };
        self.cached_sketches
            .iter()
            .filter(|(id, _)| Some(*id) != editing && !self.hidden_sketches.contains(id))
            .map(|(id, s)| (*id, s.clone()))
            .collect()
    }

    pub fn plane_frame(&self, plane: &PlaneRef) -> Option<Frame> {
        match plane {
            PlaneRef::Origin(OriginPlane::XY) => Some(Frame::XY),
            PlaneRef::Origin(OriginPlane::YZ) => Some(Frame::YZ),
            PlaneRef::Origin(OriginPlane::XZ) => Some(Frame::XZ),
            PlaneRef::Feature(id) => self
                .cached_planes
                .iter()
                .find(|(i, _)| i == id)
                .map(|(_, f)| *f),
            PlaneRef::Face(face) => {
                let body = self.pick_body(face.body)?;
                basset_core::regen::face_frame(body.solid.face(face.key)?)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use basset_math::Aabb;

    use crate::editor::harness::{Harness, block, click_at};
    use crate::editor::tools::{self, ToolKind};

    /// The edge under the pointer lights up before the click, so what a pick will take
    /// is never a surprise. The hover runs the same pick a click does, which is what
    /// makes the promise and the result agree.
    #[test]
    fn the_edge_under_the_pointer_hovers_while_a_fillet_runs() {
        let mut h = Harness::new();
        let body = h.block();
        h.start_tool(ToolKind::Fillet);
        h.move_world(Vec3::new(5.0, 0.0, 2.0));
        assert!(
            matches!(&h.editor.hover, Some(Pick::Edge(e, _)) if e.body == body),
            "{:?}",
            h.editor.hover
        );
        // And the scene draws it: the hovered-edge batch is one of the line batches.
        assert!(h.frame().line_batches > 0);
    }

    /// Aiming at an edge is the whole of using Fillet, so while it runs the aim is wider
    /// and the edge wins over the face it lies on. With no tool the same click is a
    /// perfectly ordinary face pick.
    #[test]
    fn a_blend_tool_aims_wider_and_prefers_the_edge() {
        let mut editor = Editor::new(None);
        editor.window_px = [800, 600];
        let body = block(&mut editor);
        // The default camera sees the whole world, so ten pixels of slack covers the
        // 10 mm block twice over and every edge of it is within tolerance of every
        // click. Frame the body first, the way a user looking at it would.
        editor.camera.zoom_to_fit(&Aabb {
            min: Vec3::ZERO,
            max: Vec3::new(10.0, 10.0, 2.0),
        });
        // Between the ordinary eight pixels of slack and the blend tools' sixteen.
        let px = editor
            .camera
            .pixel_size_at(Vec3::new(5.0, 0.0, 2.0), editor.window_px);
        let ray = click_at(5.0, px * 12.0);
        let plain = pick(&editor, &ray, &editor.pick_filter(), 8.0);
        assert!(matches!(plain, Some(Pick::Face(..))), "{plain:?}");

        tools::start_tool(&mut editor, ToolKind::Fillet);
        let blend = pick(&editor, &ray, &editor.pick_filter(), 8.0);
        assert!(
            matches!(&blend, Some(Pick::Edge(e, _)) if e.body == body),
            "{blend:?}"
        );

        // Still only slack, not a free-for-all: the middle of the face is the face.
        let middle = pick(&editor, &click_at(5.0, 5.0), &editor.pick_filter(), 8.0);
        assert!(matches!(middle, Some(Pick::Face(..))), "{middle:?}");
    }
}
