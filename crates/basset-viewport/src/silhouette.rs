//! View-dependent silhouette edges.
//!
//! The kernel's feature edges are everything the *body* knows about itself: folds, and
//! boundaries between surfaces that are not tangent. They are the same from every angle,
//! which is exactly why they cannot bound a curved body. A cylinder standing against the
//! background has no fold down its side — the wall is one smooth face — so the drawn edges
//! alone leave it bounded by its shading, and against a background of similar value it
//! dissolves into it. What bounds it is its *silhouette*: the line where the surface turns
//! away from the eye.
//!
//! An edge of the mesh is on the silhouette when one of the two facets sharing it faces the
//! camera and the other faces away. That depends on where the camera is, so it is
//! recomputed per camera change rather than baked at tessellation time, and it is the
//! viewport's business rather than the kernel's.
//!
//! The work is split so the per-frame half is as small as it can be: [`Silhouette`] welds
//! the mesh and builds its facet adjacency once per upload, and per frame only one dot
//! product per facet and one sign comparison per edge are evaluated. [`SilhouetteCache`]
//! holds the last answer against the view that produced it, so a still camera re-evaluates
//! nothing at all.

use std::collections::HashMap;

use basset_math::{Mat4, TriMesh, Vec3};

use crate::camera::{Camera, Projection};

/// Distance below which two mesh vertices are one vertex.
///
/// The mesh arrives with a separate copy of every triangle corner, and two copies of one
/// corner agree only to the kernel's merge tolerance, because BSP splitting moves a vertex
/// a little each time a boolean passes through it. Welding tighter than the kernel does
/// would leave facet pairs unpaired, and an unpaired edge can never be tested for a sign
/// change: the silhouette would come back with gaps in it. This is the kernel's
/// `MERGE_TOL`, restated because the viewport does not depend on the kernel.
const WELD_TOL: f64 = 1e-5;

/// Height below which a triangle is a sliver whose own normal says nothing.
///
/// The orientation of a facet comes from the cross product of its edges, and that is
/// reliable only when the triangle stands well clear of the noise in its corners: the
/// corners agree with their neighbours to [`WELD_TOL`], so a triangle a few tolerances
/// high has a normal that is mostly round-off. Boolean caps are full of such triangles.
/// A BSP fragment's boundary runs along the cutting plane with the real edge as a
/// sub-stretch of it, so after healing the polygon has three or more vertices in a row
/// on one line, and covering every vertex — which the exporter needs for a closed file —
/// forces the triangulation to emit a triangle across them with no area to speak of. Its
/// cross product normalises to a direction that can point anywhere, including straight
/// back into the body, and a facet that faces the wrong way disagrees with every
/// neighbour: all three of its edges get drawn, as a line radiating across the flat face
/// to the fragment's far corner and two dashes along the hole's rim. A hundred weld
/// tolerances is a micron on the mm-scale bodies this draws; a real facet is never that
/// thin.
const SLIVER_HEIGHT: f64 = 100.0 * WELD_TOL;

/// The direction a facet faces: its geometric normal, or for a sliver the normal the mesh
/// carries at its corners. `None` when neither says anything.
///
/// The carried normal is the fallback and not the rule because it is the *shaded* normal:
/// on a curved face it describes the ideal surface rather than the drawn triangle, and
/// testing that would put the outline up to half a facet off the shading it bounds. On a
/// flat face it is the plane's exact normal, which is precisely what a sliver lying in that
/// plane should be judged by, and on a curved face it is within the smoothing cone of the
/// facet's own — close enough that a sliver on a rim behaves like its neighbours instead
/// of sprouting lines.
fn facet_normal(p: [Vec3; 3], carried: [Vec3; 3]) -> Option<Vec3> {
    let cross = (p[1] - p[0]).cross(p[2] - p[0]);
    let longest = (p[1] - p[0])
        .length()
        .max((p[2] - p[1]).length())
        .max((p[0] - p[2]).length());
    // Twice the area over the base is the height; `longest` is zero only when all three
    // corners coincide, and then the comparison is false and the fallback decides.
    if cross.length() > SLIVER_HEIGHT * longest {
        return cross.try_normalize();
    }
    (carried[0] + carried[1] + carried[2]).try_normalize()
}

/// Where the camera is, expressed in the mesh's own coordinates.
///
/// The two projections need different arithmetic and get separate variants rather than one
/// with a sentinel: under perspective a facet is judged against the direction from the eye
/// *to that facet*, which differs across the body, while under an orthographic projection
/// every facet is judged against the single view direction. Standing in for an
/// orthographic camera with an eye a long way off is the usual fudge, and it puts the
/// silhouette of a large body visibly in the wrong place.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ViewPoint {
    Eye(Vec3),
    Direction(Vec3),
}

impl ViewPoint {
    /// The camera as a mesh drawn with `model` sees it. Working in the mesh's own
    /// coordinates keeps the adjacency's positions untouched, so moving a body only
    /// re-evaluates the sign test rather than transforming every vertex.
    ///
    /// Falls back to the world-space camera when `model` cannot be inverted (a zero-scale
    /// preview): a wrong silhouette on a degenerate body, rather than NaNs in the buffer.
    pub fn for_instance(camera: &Camera, model: &Mat4) -> Self {
        let world = match camera.projection {
            Projection::Perspective { .. } => ViewPoint::Eye(camera.eye()),
            Projection::Orthographic { .. } => ViewPoint::Direction(camera.forward()),
        };
        if model.determinant().abs() < 1e-18 {
            return world;
        }
        let inverse = model.inverse();
        match world {
            ViewPoint::Eye(eye) => ViewPoint::Eye(inverse.transform_point3(eye)),
            ViewPoint::Direction(d) => ViewPoint::Direction(inverse.transform_vector3(d)),
        }
    }
}

/// One facet's plane, as the sign test needs it: the triangle's geometric normal and a
/// point on it.
///
/// Geometric, not the shaded normal the mesh carries. A smooth-shaded cylinder's vertex
/// normals describe the ideal surface the facets stand for, so testing those would put the
/// line where the ideal cylinder turns away rather than where the drawn triangles do, and
/// the silhouette would sit up to half a facet off the edge of the shading it is meant to
/// bound.
struct Facet {
    normal: Vec3,
    point: Vec3,
}

/// A mesh edge with the two facets that share it, ready for the per-frame sign test.
struct AdjacentEdge {
    a: u32,
    b: u32,
    facets: [u32; 2],
}

/// Facet adjacency for one mesh: built once per upload, evaluated per camera change.
pub struct Silhouette {
    points: Vec<Vec3>,
    facets: Vec<Facet>,
    edges: Vec<AdjacentEdge>,
}

impl Silhouette {
    pub fn build(mesh: &TriMesh) -> Self {
        let mut weld = VertexIndex::default();
        let mut facets: Vec<Facet> = Vec::with_capacity(mesh.triangle_count());
        let mut users: HashMap<(u32, u32), Vec<u32>> = HashMap::new();
        for tri in mesh.indices.as_chunks::<3>().0 {
            let p = tri.map(|i| mesh.positions[i as usize]);
            let Some(normal) = facet_normal(p, tri.map(|i| mesh.normals[i as usize])) else {
                continue;
            };
            let facet = facets.len() as u32;
            facets.push(Facet {
                normal,
                point: (p[0] + p[1] + p[2]) / 3.0,
            });
            let ids = p.map(|v| weld.id(v));
            for i in 0..3 {
                let (a, b) = (ids[i], ids[(i + 1) % 3]);
                if a == b {
                    continue;
                }
                let key = if a < b { (a, b) } else { (b, a) };
                users.entry(key).or_default().push(facet);
            }
        }

        let mut edges: Vec<AdjacentEdge> = Vec::new();
        for ((a, b), users) in users {
            // Consecutive pairs rather than every pair: the edge is on the silhouette when
            // the facets on it do not all agree, and if every consecutive pair agrees then
            // all of them do. Two users is the closed-shell case and gives one pair; a
            // non-manifold edge, where two bodies were joined along a line, gives more.
            // An edge with one user is a border the shell should not have, and a border
            // has no far side to disagree with.
            for pair in users.windows(2) {
                edges.push(AdjacentEdge {
                    a,
                    b,
                    facets: [pair[0], pair[1]],
                });
            }
        }
        // Hash-map order is arbitrary; sort so one mesh always yields its segments in one
        // order, whatever run built it.
        edges.sort_unstable_by_key(|e| (e.a, e.b, e.facets));
        Self {
            points: weld.points,
            facets,
            edges,
        }
    }

    /// Number of adjacency records: the size of the per-frame sign test.
    pub fn edge_count(&self) -> usize {
        self.edges.len()
    }

    /// Whether the mesh has any adjacency at all to test.
    pub fn is_empty(&self) -> bool {
        self.edges.is_empty()
    }

    /// Writes the silhouette segments for `view` into `out`, in the mesh's own coordinates.
    ///
    /// `facing` is scratch, one bool per facet; the caller owns it so a per-frame call
    /// allocates nothing.
    pub fn evaluate(&self, view: ViewPoint, facing: &mut Vec<bool>, out: &mut Vec<[Vec3; 2]>) {
        facing.clear();
        facing.reserve(self.facets.len());
        match view {
            // A facet faces the camera when its outward normal leans back towards the eye.
            // Both arms ask that same question; only what it is measured against differs.
            ViewPoint::Eye(eye) => {
                facing.extend(
                    self.facets
                        .iter()
                        .map(|f| f.normal.dot(f.point - eye) < 0.0),
                );
            }
            ViewPoint::Direction(d) => {
                facing.extend(self.facets.iter().map(|f| f.normal.dot(d) < 0.0));
            }
        }
        out.clear();
        for e in &self.edges {
            if facing[e.facets[0] as usize] != facing[e.facets[1] as usize] {
                out.push([self.points[e.a as usize], self.points[e.b as usize]]);
            }
        }
    }
}

/// The last silhouette computed for one drawn instance, held against the view that
/// produced it. An unchanged view is the common case — the camera is still between
/// interactions — and costs one comparison.
#[derive(Default)]
pub struct SilhouetteCache {
    view: Option<ViewPoint>,
    facing: Vec<bool>,
    segments: Vec<[Vec3; 2]>,
    /// Instances come and go with the document; anything not asked for this frame is
    /// dropped, the same way highlight buffers are.
    pub(crate) used_this_frame: bool,
}

impl SilhouetteCache {
    pub fn segments(&mut self, silhouette: &Silhouette, view: ViewPoint) -> &[[Vec3; 2]] {
        if self.view != Some(view) {
            silhouette.evaluate(view, &mut self.facing, &mut self.segments);
            self.view = Some(view);
        }
        &self.segments
    }

    /// Whether the last call had to recompute. For tests and measurement.
    pub fn view(&self) -> Option<ViewPoint> {
        self.view
    }
}

/// Snaps points within [`WELD_TOL`] of each other onto one id, through a grid of cell size
/// `2·WELD_TOL` probed across the neighbouring cells so a pair straddling a cell boundary
/// still merges. The kernel welds the same way, and for the same reason.
#[derive(Default)]
struct VertexIndex {
    cells: HashMap<(i64, i64, i64), Vec<u32>>,
    points: Vec<Vec3>,
}

impl VertexIndex {
    fn id(&mut self, p: Vec3) -> u32 {
        let cell = |x: f64| (x / (2.0 * WELD_TOL)).floor() as i64;
        let c = (cell(p.x), cell(p.y), cell(p.z));
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    let Some(ids) = self.cells.get(&(c.0 + dx, c.1 + dy, c.2 + dz)) else {
                        continue;
                    };
                    for &id in ids {
                        if self.points[id as usize].distance_squared(p) <= WELD_TOL * WELD_TOL {
                            return id;
                        }
                    }
                }
            }
        }
        let id = self.points.len() as u32;
        self.points.push(p);
        self.cells.entry(c).or_default().push(id);
        id
    }
}
