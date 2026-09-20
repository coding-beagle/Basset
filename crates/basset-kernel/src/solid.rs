//! The boundary representation.
//!
//! A [`Solid`] is a closed shell of [`Face`]s. Each face owns a set of planar convex
//! polygons plus a [`SurfaceKind`] recording the analytic surface those polygons
//! approximate. Booleans only ever split polygons, so a face's key and surface survive
//! every downstream operation: that is what keeps references from later features valid.
//!
//! Polygons start convex, which is what the BSP splitter in [`crate::csg`] wants, but
//! they do not stay that way: a boolean leaves reflex fragments, and healing T-junctions
//! inserts vertices in the middle of an edge. Nothing downstream may assume a polygon
//! fans from its first vertex.
//!
//! Curved surfaces are faceted at creation time; the surface kind is what lets the
//! tessellator shade them smoothly and lets selection report "cylinder, radius 5".

use std::collections::HashMap;

use basset_math::{Aabb, Affine3, Frame, Plane, TriMesh, Vec2, Vec3};

use crate::error::KernelError;
use crate::geometry::{Contour, Profile, Segment};
use crate::ids::{EdgeKey, FaceKey};

/// Distance below which two vertices are the same vertex. Looser than the maths crate's
/// `LINEAR_TOL` because BSP splitting accumulates error per split: each split interpolates
/// a crossing point on an edge whose ends a previous split already moved, so a vertex that
/// four booleans have passed through is several microns from where the first one put it.
/// Measured on `testcases/ExportAs3MFCreatesBadGeometry.bass`, which drifts 7.4e-6 across
/// its four features; at 1e-6 the two copies of a corner stayed separate vertices and the
/// exported shell pinched together along the line between them.
///
/// Ten nanometres is four orders of magnitude below anything a printer or a user can
/// resolve, so nothing real is welded away by being generous here.
pub const MERGE_TOL: f64 = 1e-5;

/// Surfaces meeting at a sharper angle than this are separated by a visible crease.
/// [`smooth_normals`] shades to the same cut-off, so an edge that reads as sharp is also
/// drawn as one.
const DISPLAY_CREASE_COS: f64 = 0.7; // ≈ 45.6°

/// One polygon's use of an edge: (face index, directed a→b, polygon normal).
type EdgeUser = (usize, bool, Vec3);

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SurfaceKind {
    Planar {
        normal: Vec3,
    },
    Cylindrical {
        origin: Vec3,
        axis: Vec3,
        radius: f64,
    },
    Conical {
        apex: Vec3,
        axis: Vec3,
        half_angle: f64,
    },
    /// Faceted surface with no simple analytic description (sweeps, lofts, blends of
    /// curved edges).
    Freeform,
}

impl SurfaceKind {
    pub fn is_planar(&self) -> bool {
        matches!(self, SurfaceKind::Planar { .. })
    }

    fn transformed(&self, t: &Affine3) -> SurfaceKind {
        let dir = |d: Vec3| t.transform_vector3(d).normalize_or_zero();
        let scale = t.matrix3.determinant().abs().cbrt();
        match *self {
            SurfaceKind::Planar { normal } => SurfaceKind::Planar {
                normal: (t.matrix3.inverse().transpose() * normal).normalize_or_zero(),
            },
            SurfaceKind::Cylindrical {
                origin,
                axis,
                radius,
            } => SurfaceKind::Cylindrical {
                origin: t.transform_point3(origin),
                axis: dir(axis),
                radius: radius * scale,
            },
            SurfaceKind::Conical {
                apex,
                axis,
                half_angle,
            } => SurfaceKind::Conical {
                apex: t.transform_point3(apex),
                axis: dir(axis),
                half_angle,
            },
            SurfaceKind::Freeform => SurfaceKind::Freeform,
        }
    }

    /// Whether two faces meeting along an edge lie on one and the same surface, so the
    /// boundary between them is a bookkeeping seam and not something the user can see.
    ///
    /// Two planar faces that share an edge and agree on their normal are necessarily the
    /// same plane, so the normal settles it. `Freeform` never continues: with no analytic
    /// surface to compare there is nothing to be sure about, and an edge drawn where the
    /// shape is smooth is a smaller fault than a missing one where it is not.
    fn continues(&self, other: &SurfaceKind) -> bool {
        let same_dir = |a: Vec3, b: Vec3| a.dot(b) > 1.0 - 1e-9;
        match (*self, *other) {
            (SurfaceKind::Planar { normal: a }, SurfaceKind::Planar { normal: b }) => {
                same_dir(a, b)
            }
            (
                SurfaceKind::Cylindrical {
                    origin: oa,
                    axis: aa,
                    radius: ra,
                },
                SurfaceKind::Cylindrical {
                    origin: ob,
                    axis: ab,
                    radius: rb,
                },
            ) => {
                same_dir(aa, ab)
                    && (ra - rb).abs() < MERGE_TOL
                    && (ob - oa).reject_from_normalized(aa).length() < MERGE_TOL
            }
            (
                SurfaceKind::Conical {
                    apex: pa,
                    axis: aa,
                    half_angle: ha,
                },
                SurfaceKind::Conical {
                    apex: pb,
                    axis: ab,
                    half_angle: hb,
                },
            ) => same_dir(aa, ab) && (ha - hb).abs() < 1e-9 && pa.distance(pb) < MERGE_TOL,
            _ => false,
        }
    }

    fn flipped(&self) -> SurfaceKind {
        match *self {
            SurfaceKind::Planar { normal } => SurfaceKind::Planar { normal: -normal },
            other => other,
        }
    }
}

/// Planar convex polygon, counter-clockwise about its outward `plane.normal`.
#[derive(Debug, Clone, PartialEq)]
pub struct Polygon {
    pub vertices: Vec<Vec3>,
    pub plane: Plane,
}

impl Polygon {
    /// Builds a polygon, computing the plane by Newell's method so slightly non-planar
    /// input still gets a sensible normal. Returns `None` for degenerate input.
    pub fn new(mut vertices: Vec<Vec3>) -> Option<Self> {
        // Revolving a point that sits on the axis yields repeated vertices; drop them so
        // every polygon edge has a direction.
        vertices.dedup_by(|a, b| a.distance_squared(*b) < MERGE_TOL * MERGE_TOL);
        if vertices.len() > 1
            && vertices[0].distance_squared(*vertices.last().unwrap()) < MERGE_TOL * MERGE_TOL
        {
            vertices.pop();
        }
        if vertices.len() < 3 {
            return None;
        }
        let normal = newell_normal(&vertices);
        if normal.length_squared() < 1e-24 {
            return None;
        }
        let normal = normal.normalize();
        let centroid = vertices.iter().sum::<Vec3>() / vertices.len() as f64;
        Some(Self {
            vertices,
            plane: Plane {
                origin: centroid,
                normal,
            },
        })
    }

    pub fn flipped(&self) -> Polygon {
        let mut vertices = self.vertices.clone();
        vertices.reverse();
        Polygon {
            vertices,
            plane: Plane {
                origin: self.plane.origin,
                normal: -self.plane.normal,
            },
        }
    }

    pub fn area(&self) -> f64 {
        newell_normal(&self.vertices).length() * 0.5
    }

    fn transformed(&self, t: &Affine3, flip: bool) -> Option<Polygon> {
        let mut vertices: Vec<Vec3> = self
            .vertices
            .iter()
            .map(|v| t.transform_point3(*v))
            .collect();
        if flip {
            vertices.reverse();
        }
        Polygon::new(vertices)
    }
}

/// Unnormalised normal whose length is twice the polygon area.
pub(crate) fn newell_normal(vertices: &[Vec3]) -> Vec3 {
    let mut n = Vec3::ZERO;
    for (i, a) in vertices.iter().enumerate() {
        let b = vertices[(i + 1) % vertices.len()];
        n += Vec3::new(
            (a.y - b.y) * (a.z + b.z),
            (a.z - b.z) * (a.x + b.x),
            (a.x - b.x) * (a.y + b.y),
        );
    }
    n
}

#[derive(Debug, Clone, PartialEq)]
pub struct Face {
    pub key: FaceKey,
    pub surface: SurfaceKind,
    pub polygons: Vec<Polygon>,
}

impl Face {
    pub fn area(&self) -> f64 {
        self.polygons.iter().map(Polygon::area).sum()
    }

    pub fn centroid(&self) -> Vec3 {
        let mut sum = Vec3::ZERO;
        let mut total = 0.0;
        for p in &self.polygons {
            let a = p.area();
            sum += p.plane.origin * a;
            total += a;
        }
        if total > 0.0 { sum / total } else { Vec3::ZERO }
    }

    /// Frame of a planar face, anchored at the average of its vertices so the origin is
    /// independent of how booleans happened to fragment the face. `None` for curved faces.
    ///
    /// This is both the frame of a sketch drawn on the face and the frame of the profile
    /// the face yields, so what the user sees when they pick it is what they get.
    pub fn frame(&self) -> Option<Frame> {
        let SurfaceKind::Planar { normal } = self.surface else {
            return None;
        };
        let mut sum = Vec3::ZERO;
        let mut n = 0usize;
        for poly in &self.polygons {
            for v in &poly.vertices {
                sum += *v;
                n += 1;
            }
        }
        (n > 0).then(|| Frame::from_normal(sum / n as f64, normal))
    }
}

/// Stable curve tag for the boundary a face shares with `neighbour`.
///
/// A face-derived profile has no sketch curves to name its edges, so the neighbouring
/// face plays that role: the lateral face an extrude grows from that stretch is keyed by
/// what the stretch borders, which is stable across edits of the body underneath. FNV-1a
/// rather than `DefaultHasher` because the tag ends up inside a persisted `FaceKey` and
/// must mean the same thing in a future build.
fn neighbour_tag(neighbour: FaceKey) -> u32 {
    let mut hash: u32 = 0x811c_9dc5;
    let mut eat = |byte: u8| {
        hash ^= u32::from(byte);
        hash = hash.wrapping_mul(0x0100_0193);
    };
    for byte in neighbour.op.feature.to_le_bytes() {
        eat(byte);
    }
    for byte in neighbour.op.sub.to_le_bytes() {
        eat(byte);
    }
    let (discriminant, payload) = match neighbour.role {
        crate::ids::FaceRole::StartCap => (0u8, 0u32),
        crate::ids::FaceRole::EndCap => (1, 0),
        crate::ids::FaceRole::Side(c) => (2, c),
        crate::ids::FaceRole::Fillet(c) => (3, c),
        crate::ids::FaceRole::Chamfer(c) => (4, c),
        crate::ids::FaceRole::Generic(c) => (5, c),
    };
    eat(discriminant);
    for byte in payload.to_le_bytes() {
        eat(byte);
    }
    hash
}

/// One straight piece of an edge. `start → end` follows the winding of face `key.a`, so
/// `normal_a × direction` points into face `a` and the pair of normals gives the dihedral.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EdgeSegment {
    pub start: Vec3,
    pub end: Vec3,
    pub normal_a: Vec3,
    pub normal_b: Vec3,
}

/// All straight pieces of the boundary between two faces. Curved edges have many.
#[derive(Debug, Clone, PartialEq)]
pub struct Edge {
    pub key: EdgeKey,
    pub segments: Vec<EdgeSegment>,
    /// The two faces continue into each other here: one surface, split only because two
    /// features happened to name the halves. A sketch line cut in two makes a wall like
    /// this. Nothing is drawn along such an edge and nothing can be picked on it, because
    /// to the user there is no edge there.
    pub smooth: bool,
}

impl Edge {
    pub fn length(&self) -> f64 {
        self.segments.iter().map(|s| s.start.distance(s.end)).sum()
    }

    /// Orders the segments into connected polylines. A key normally yields one chain, but
    /// a boolean can leave two faces touching along separate stretches.
    pub fn chains(&self) -> Vec<Vec<EdgeSegment>> {
        let mut remaining: Vec<EdgeSegment> = self.segments.clone();
        let mut chains = Vec::new();
        while let Some(first) = remaining.pop() {
            let mut chain = vec![first];
            loop {
                let tail = chain.last().unwrap().end;
                match remaining
                    .iter()
                    .position(|s| s.start.distance_squared(tail) < MERGE_TOL * MERGE_TOL)
                {
                    Some(i) => chain.push(remaining.swap_remove(i)),
                    None => break,
                }
            }
            loop {
                let head = chain[0].start;
                match remaining
                    .iter()
                    .position(|s| s.end.distance_squared(head) < MERGE_TOL * MERGE_TOL)
                {
                    Some(i) => chain.insert(0, remaining.swap_remove(i)),
                    None => break,
                }
            }
            chains.push(chain);
        }
        chains
    }
}

/// Triangles for rendering plus the map from `face_id` back to kernel faces.
#[derive(Debug, Clone, Default)]
pub struct Tessellated {
    pub mesh: TriMesh,
    /// `mesh.face_ids[i]` indexes this table.
    pub face_keys: Vec<FaceKey>,
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct Solid {
    pub faces: Vec<Face>,
}

impl Solid {
    pub fn is_empty(&self) -> bool {
        self.faces.iter().all(|f| f.polygons.is_empty())
    }

    pub fn face(&self, key: FaceKey) -> Option<&Face> {
        self.faces.iter().find(|f| f.key == key)
    }

    pub fn face_keys(&self) -> impl Iterator<Item = FaceKey> + '_ {
        self.faces.iter().map(|f| f.key)
    }

    /// A planar face expressed as a profile, so a face of an existing body can be pushed,
    /// revolved or lofted exactly like a sketch region.
    ///
    /// The boundary is taken from the face's edges rather than from its polygons, which
    /// means the interior lines left by boolean splitting are already gone and every
    /// stretch knows which face it borders.
    pub fn face_profile(&self, key: FaceKey) -> Result<Profile, KernelError> {
        let face = self.face(key).ok_or(KernelError::MissingFace(key))?;
        let frame = face.frame().ok_or(KernelError::NotPlanarFace(key))?;
        let mut pieces: Vec<(Vec3, Vec3, u32)> = Vec::new();
        for edge in self.edges().into_iter().filter(|e| e.key.touches(key)) {
            let neighbour = if edge.key.a == key {
                edge.key.b
            } else {
                edge.key.a
            };
            let tag = neighbour_tag(neighbour);
            for s in &edge.segments {
                // Segments run along face `key.a`'s winding, so flip the ones where the
                // neighbour is `a`; then every piece follows *this* face's winding and the
                // loops come out oriented against its normal.
                let (start, end) = if edge.key.a == key {
                    (s.start, s.end)
                } else {
                    (s.end, s.start)
                };
                pieces.push((start, end, tag));
            }
        }
        let mut contours: Vec<Contour> = Vec::new();
        for chain in chain_loops(pieces) {
            let points: Vec<Vec2> = chain.iter().map(|(a, _, _)| frame.to_local(*a)).collect();
            let segments = chain
                .iter()
                .map(|(_, _, tag)| Segment::line(*tag))
                .collect();
            contours.push(Contour {
                points,
                segments,
                closed: true,
            });
        }
        // The outer loop is the one with the most area; anything else it encloses is a hole.
        let outer = contours
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.signed_area().abs().total_cmp(&b.1.signed_area().abs()))
            .map(|(i, _)| i)
            .ok_or(KernelError::EmptyProfile)?;
        let holes = contours
            .iter()
            .enumerate()
            .filter(|(i, _)| *i != outer)
            .map(|(_, c)| c.clone())
            .collect();
        Profile {
            frame,
            outer: contours[outer].clone(),
            holes,
        }
        .normalised()
    }

    pub fn polygon_count(&self) -> usize {
        self.faces.iter().map(|f| f.polygons.len()).sum()
    }

    pub fn aabb(&self) -> Aabb {
        Aabb::from_points(
            self.faces
                .iter()
                .flat_map(|f| f.polygons.iter())
                .flat_map(|p| p.vertices.iter().copied()),
        )
    }

    pub fn surface_area(&self) -> f64 {
        self.faces.iter().map(Face::area).sum()
    }

    /// Signed volume by the divergence theorem; negative means inside-out.
    pub fn volume(&self) -> f64 {
        let mut v = 0.0;
        for p in self.faces.iter().flat_map(|f| f.polygons.iter()) {
            let a = p.vertices[0];
            for w in p.vertices[1..].windows(2) {
                v += a.dot(w[0].cross(w[1]));
            }
        }
        v / 6.0
    }

    /// Volume-weighted centroid, using the tetrahedra of the volume computation.
    pub fn centroid(&self) -> Vec3 {
        let mut c = Vec3::ZERO;
        let mut v = 0.0;
        for p in self.faces.iter().flat_map(|f| f.polygons.iter()) {
            let a = p.vertices[0];
            for w in p.vertices[1..].windows(2) {
                let tet = a.dot(w[0].cross(w[1])) / 6.0;
                c += (a + w[0] + w[1]) / 4.0 * tet;
                v += tet;
            }
        }
        if v.abs() > 1e-18 {
            c / v
        } else {
            self.aabb().center()
        }
    }

    pub fn transformed(&self, t: &Affine3) -> Solid {
        // A reflection turns outward faces inward; flipping the winding restores it.
        let flip = t.matrix3.determinant() < 0.0;
        Solid {
            faces: self
                .faces
                .iter()
                .map(|f| Face {
                    key: f.key,
                    surface: if flip {
                        f.surface.transformed(t).flipped()
                    } else {
                        f.surface.transformed(t)
                    },
                    polygons: f
                        .polygons
                        .iter()
                        .filter_map(|p| p.transformed(t, flip))
                        .collect(),
                })
                .collect(),
        }
    }

    /// Flips every face so the shell faces outward. Generators build sides by a fixed rule
    /// and use this rather than reasoning about every orientation case.
    pub(crate) fn ensure_outward(mut self) -> Solid {
        if self.volume() < 0.0 {
            for f in &mut self.faces {
                f.surface = f.surface.flipped();
                for p in &mut f.polygons {
                    *p = p.flipped();
                }
            }
        }
        self
    }

    /// Drops faces left without polygons by a boolean.
    pub(crate) fn prune(&mut self) {
        self.faces.retain(|f| !f.polygons.is_empty());
    }

    /// Whether every polygon edge is shared by exactly one other polygon with opposite
    /// direction. Anything else means the shell leaks and later booleans will misbehave.
    pub fn is_closed(&self) -> bool {
        self.validate().is_ok()
    }

    pub fn validate(&self) -> Result<(), KernelError> {
        if self.is_empty() {
            return Err(KernelError::NotClosed("no faces".into()));
        }
        let open = self.unmatched_edges();
        if !open.is_empty() {
            return Err(KernelError::NotClosed(format!(
                "{} unmatched edge(s), first at {:?}",
                open.len(),
                open[0][0]
            )));
        }
        Ok(())
    }

    /// Polygon edges not paired with an opposite-direction twin. Empty for a closed shell.
    pub fn unmatched_edges(&self) -> Vec<[Vec3; 2]> {
        let mut index = VertexIndex::default();
        let mut directed: HashMap<(u32, u32), i32> = HashMap::new();
        for p in self.faces.iter().flat_map(|f| f.polygons.iter()) {
            let ids: Vec<u32> = p.vertices.iter().map(|v| index.id(*v)).collect();
            for (i, &a) in ids.iter().enumerate() {
                let b = ids[(i + 1) % ids.len()];
                if a == b {
                    continue;
                }
                let (k, s) = if a < b { ((a, b), 1) } else { ((b, a), -1) };
                *directed.entry(k).or_default() += s;
            }
        }
        let mut open: Vec<[Vec3; 2]> = directed
            .into_iter()
            .filter(|(_, c)| *c != 0)
            .map(|((a, b), _)| [index.points[a as usize], index.points[b as usize]])
            .collect();
        open.sort_by(|x, y| {
            x[0].to_array()
                .partial_cmp(&y[0].to_array())
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        open
    }

    /// Snaps near-coincident vertices together and splits polygon edges at vertices of
    /// neighbouring polygons that lie on them.
    ///
    /// BSP splitting cuts a polygon by a plane without touching its neighbours, so the
    /// result is riddled with T-junctions. Edge extraction and closedness checks need every
    /// shared edge to be shared vertex-for-vertex, so booleans call this before returning.
    ///
    /// The snap has to come first, and it has to be written back into the polygons rather
    /// than only used for the analysis. Splitting an edge computes the crossing point by
    /// interpolation, so the two polygons either side of a shared edge come out of a
    /// boolean agreeing on its endpoints to within [`MERGE_TOL`] but not exactly. Leaving
    /// that difference in place lets the *next* boolean interpolate from two slightly
    /// different edges and drift further, until a pair is far enough apart that nothing
    /// pairs them up and the shell has a crack in it.
    pub(crate) fn heal(&mut self) {
        let mut index = VertexIndex::default();
        for p in self.faces.iter_mut().flat_map(|f| f.polygons.iter_mut()) {
            for v in &mut p.vertices {
                let id = index.id(*v) as usize;
                *v = index.points[id];
            }
        }
        let mut by_x: Vec<(f64, Vec3)> = index.points.iter().map(|p| (p.x, *p)).collect();
        by_x.sort_by(|a, b| a.0.total_cmp(&b.0));
        let xs: Vec<f64> = by_x.iter().map(|p| p.0).collect();

        for p in self.faces.iter_mut().flat_map(|f| f.polygons.iter_mut()) {
            let n = p.vertices.len();
            let mut out = Vec::with_capacity(n);
            for i in 0..n {
                let a = p.vertices[i];
                let b = p.vertices[(i + 1) % n];
                out.push(a);
                let (lo, hi) = (a.x.min(b.x) - MERGE_TOL, a.x.max(b.x) + MERGE_TOL);
                let start = xs.partition_point(|x| *x < lo);
                let ab = b - a;
                let len2 = ab.length_squared();
                if len2 < MERGE_TOL * MERGE_TOL {
                    continue;
                }
                // Only points strictly *inside* the edge split it. The guard is a
                // distance, not a parameter: a vertex a few microns off `a` sits at
                // t ~ 1e-7 on a 14 mm edge, and splitting there would add a spur out to a
                // point the merge tolerance already treats as `a` and straight back.
                let end_t = MERGE_TOL / len2.sqrt();
                let mut inserts: Vec<(f64, Vec3)> = Vec::new();
                for &(_, q) in by_x[start..].iter().take_while(|(x, _)| *x <= hi) {
                    let t = (q - a).dot(ab) / len2;
                    if t <= end_t || t >= 1.0 - end_t {
                        continue;
                    }
                    let foot = a + ab * t;
                    if foot.distance_squared(q) < MERGE_TOL * MERGE_TOL {
                        inserts.push((t, q));
                    }
                }
                inserts.sort_by(|x, y| x.0.total_cmp(&y.0));
                inserts.dedup_by(|x, y| x.1.distance_squared(y.1) < MERGE_TOL * MERGE_TOL);
                out.extend(inserts.into_iter().map(|(_, q)| q));
            }
            p.vertices = out;
        }
    }

    /// Every polygon edge with the polygons that use it, keyed by the undirected pair of
    /// welded vertex ids. Welding is by [`VertexIndex`], which probes neighbouring cells,
    /// so vertices that a boolean left agreeing only to `MERGE_TOL` still pair up.
    fn shared_polygon_edges(&self) -> (VertexIndex, HashMap<(u32, u32), Vec<EdgeUser>>) {
        let mut index = VertexIndex::default();
        let mut shared: HashMap<(u32, u32), Vec<EdgeUser>> = HashMap::new();
        for (fi, f) in self.faces.iter().enumerate() {
            for p in &f.polygons {
                let ids: Vec<u32> = p.vertices.iter().map(|v| index.id(*v)).collect();
                for (i, &a) in ids.iter().enumerate() {
                    let b = ids[(i + 1) % ids.len()];
                    if a == b {
                        continue;
                    }
                    let (k, forward) = if a < b {
                        ((a, b), true)
                    } else {
                        ((b, a), false)
                    };
                    shared
                        .entry(k)
                        .or_default()
                        .push((fi, forward, p.plane.normal));
                }
            }
        }
        (index, shared)
    }

    /// Whether two polygons meeting along an edge join without anything to see: they
    /// belong to one face, or to two faces of the same surface, and they do not fold.
    fn continuous(&self, a: (usize, Vec3), b: (usize, Vec3)) -> bool {
        if a.1.dot(b.1) < DISPLAY_CREASE_COS {
            return false;
        }
        a.0 == b.0 || self.faces[a.0].surface.continues(&self.faces[b.0].surface)
    }

    /// The straight pieces a viewer should see as the body's outline: every fold or change
    /// of surface, whether or not the topology calls it a face boundary.
    ///
    /// These are segments, not polylines: one visual line comes back cut wherever a
    /// neighbouring face happens to end against it. That suits a line batch, and nothing
    /// downstream counts them.
    ///
    /// The viewport cannot work these out from the triangles alone. A boolean leaves the
    /// vertices of two neighbouring polygons agreeing only to within `MERGE_TOL`, so a
    /// weld done on rounded coordinates misses the pair and every triangle edge it misses
    /// reads as an open border: the user sees the mesh. The topology is known exactly
    /// here, so it is answered here.
    ///
    /// Edges used by a single polygon are left out. In a closed solid there are none, and
    /// where a construction has leaked, drawing the leak paints the triangle soup the
    /// user is least able to act on; [`Solid::validate`] is how brokenness is reported.
    pub fn display_edges(&self) -> Vec<[Vec3; 2]> {
        let (index, shared) = self.shared_polygon_edges();
        let mut out: Vec<((u32, u32), [Vec3; 2])> = shared
            .into_iter()
            .filter(|(_, users)| {
                // Every pair, not only each against the first: a non-manifold edge, where
                // two solids were joined along a line, has more than two polygons on it
                // and any one pair of them can be the one that folds.
                users.iter().enumerate().any(|(i, (fa, _, na))| {
                    users[i + 1..]
                        .iter()
                        .any(|(fb, _, nb)| !self.continuous((*fa, *na), (*fb, *nb)))
                })
            })
            .map(|((a, b), _)| ((a, b), [index.points[a as usize], index.points[b as usize]]))
            .collect();
        // Hash-map order is arbitrary; sort so the upload is the same for the same solid.
        out.sort_by_key(|(key, _)| *key);
        out.into_iter().map(|(_, seg)| seg).collect()
    }

    /// Boundaries between distinct faces. Requires a healed solid (every operation that
    /// returns a `Solid` guarantees this).
    pub fn edges(&self) -> Vec<Edge> {
        let (index, shared) = self.shared_polygon_edges();
        // Segments carry whether they are smooth, decided from the face *indices* here.
        // Looking the faces up again by key would not do: a union can leave two faces
        // under one key, and then the key names the wrong one.
        let mut edges: HashMap<EdgeKey, Vec<(EdgeSegment, bool)>> = HashMap::new();
        for ((ia, ib), users) in shared {
            // Interior to one face: both sides belong to the same face. Not an edge.
            let Some(&(f0, fwd0, n0)) = users.first() else {
                continue;
            };
            let Some(&(f1, _, n1)) = users.iter().find(|(f, _, _)| *f != f0) else {
                continue;
            };
            let (key0, key1) = (self.faces[f0].key, self.faces[f1].key);
            let key = EdgeKey::new(key0, key1);
            // Orient the segment along face `key.a`'s winding.
            let (start, end, na, nb) = {
                let (pa, pb) = (index.points[ia as usize], index.points[ib as usize]);
                let (s, e) = if fwd0 { (pa, pb) } else { (pb, pa) };
                if key.a == key0 {
                    (s, e, n0, n1)
                } else {
                    (e, s, n1, n0)
                }
            };
            let (fa, fb) = if key.a == key0 { (f0, f1) } else { (f1, f0) };
            edges.entry(key).or_default().push((
                EdgeSegment {
                    start,
                    end,
                    normal_a: na,
                    normal_b: nb,
                },
                self.continuous((fa, na), (fb, nb)),
            ));
        }
        // Hash-map order would make the chain start point (and so blend tool geometry)
        // vary between runs; sort so identical inputs always give identical solids.
        let mut out: Vec<Edge> = edges
            .into_iter()
            .map(|(key, mut pieces)| {
                pieces.sort_by(|x, y| {
                    x.0.start
                        .to_array()
                        .partial_cmp(&y.0.start.to_array())
                        .unwrap_or(std::cmp::Ordering::Equal)
                });
                Edge {
                    key,
                    smooth: pieces.iter().all(|(_, smooth)| *smooth),
                    segments: pieces.into_iter().map(|(s, _)| s).collect(),
                }
            })
            .collect();
        out.sort_by_key(|e| e.key);
        out
    }

    /// Flat-shaded triangles for planar faces; smooth normals across facets of curved
    /// faces so a 36-facet cylinder does not look like a 36-facet cylinder.
    pub fn tessellate(&self) -> Tessellated {
        let mut mesh = TriMesh::default();
        let mut face_keys = Vec::with_capacity(self.faces.len());
        for (fi, f) in self.faces.iter().enumerate() {
            face_keys.push(f.key);
            let smooth = if f.surface.is_planar() {
                None
            } else {
                Some(smooth_normals(f))
            };
            for (pi, p) in f.polygons.iter().enumerate() {
                let n = p.plane.normal;
                for [a, b, c] in triangulate_polygon(p) {
                    let tri = [p.vertices[a], p.vertices[b], p.vertices[c]];
                    let base = mesh.positions.len() as u32;
                    mesh.positions.extend_from_slice(&tri);
                    match &smooth {
                        Some(s) => mesh.normals.extend([s[pi][a], s[pi][b], s[pi][c]]),
                        None => mesh.normals.extend([n, n, n]),
                    }
                    mesh.indices.extend([base, base + 1, base + 2]);
                    mesh.face_ids.push(fi as u32);
                }
            }
        }
        Tessellated { mesh, face_keys }
    }
}

/// Index triples covering `polygon`, wound about its own normal.
///
/// A fan from vertex 0 is not good enough. `heal` leaves T-junction vertices sitting mid-edge,
/// so a fan makes zero-area triangles whenever the apex is collinear with a pair, and booleans
/// leave genuinely non-convex fragments, which a fan covers with triangles that spill outside
/// the polygon. Ear clipping handles both, and — this is the part the exporter depends on — it
/// uses *every* vertex and emits triangles whose union is exactly the polygon, so each polygon
/// edge is covered once and the shell stays closed. A dropped sliver would be a hole, and a
/// hole is a non-manifold edge in the file.
fn triangulate_polygon(polygon: &Polygon) -> Vec<[usize; 3]> {
    let n = polygon.vertices.len();
    if n < 3 {
        return Vec::new();
    }
    let frame = Frame::from_normal(polygon.plane.origin, polygon.plane.normal);
    let local: Vec<Vec2> = polygon
        .vertices
        .iter()
        .map(|v| frame.to_local(*v))
        .collect();
    let flat: Vec<f64> = local.iter().flat_map(|p| [p.x, p.y]).collect();
    let Ok(indices) = earcutr::earcut(&flat, &[], 2) else {
        return fan(n);
    };
    if indices.len() < 3 * (n - 2) {
        // Earcut gave up part-way through (it does that on self-touching input). A partial
        // cover would leave the shell open, so fall back to something that at least uses
        // every vertex.
        return fan(n);
    }
    indices
        .as_chunks::<3>()
        .0
        .iter()
        .map(|&[a, b, c]| {
            // The frame is right-handed about the polygon normal, so a counter-clockwise
            // triangle in local coordinates already faces outward.
            if (local[b] - local[a]).perp_dot(local[c] - local[a]) < 0.0 {
                [a, c, b]
            } else {
                [a, b, c]
            }
        })
        .collect()
}

fn fan(n: usize) -> Vec<[usize; 3]> {
    (1..n - 1).map(|i| [0, i, i + 1]).collect()
}

/// Per-polygon, per-vertex normals averaged over the face's polygons that meet at the
/// vertex with a similar orientation. The angle cut-off keeps genuine creases inside a
/// face (text glyph corners, for example) sharp while smoothing facet seams.
fn smooth_normals(face: &Face) -> Vec<Vec<Vec3>> {
    let mut index = VertexIndex::default();
    let mut at_vertex: HashMap<u32, Vec<Vec3>> = HashMap::new();
    let ids: Vec<Vec<u32>> = face
        .polygons
        .iter()
        .map(|p| {
            p.vertices
                .iter()
                .map(|v| {
                    let id = index.id(*v);
                    at_vertex.entry(id).or_default().push(p.plane.normal);
                    id
                })
                .collect()
        })
        .collect();
    face.polygons
        .iter()
        .zip(&ids)
        .map(|(p, ids)| {
            ids.iter()
                .map(|id| {
                    let n = at_vertex[id]
                        .iter()
                        .filter(|m| m.dot(p.plane.normal) > DISPLAY_CREASE_COS)
                        .sum::<Vec3>();
                    n.try_normalize().unwrap_or(p.plane.normal)
                })
                .collect()
        })
        .collect()
}

/// Snaps points within [`MERGE_TOL`] of each other to one id. Uses a grid of cell size
/// `2·MERGE_TOL` and probes the neighbouring cells so points straddling a cell boundary
/// still merge.
#[derive(Default)]
pub(crate) struct VertexIndex {
    cells: HashMap<(i64, i64, i64), Vec<u32>>,
    pub(crate) points: Vec<Vec3>,
}

impl VertexIndex {
    pub(crate) fn id(&mut self, p: Vec3) -> u32 {
        let cell = |x: f64| (x / (2.0 * MERGE_TOL)).floor() as i64;
        let c = (cell(p.x), cell(p.y), cell(p.z));
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    if let Some(ids) = self.cells.get(&(c.0 + dx, c.1 + dy, c.2 + dz)) {
                        for &id in ids {
                            if self.points[id as usize].distance_squared(p) <= MERGE_TOL * MERGE_TOL
                            {
                                return id;
                            }
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

/// Accumulates polygons under face keys while an operation runs.
#[derive(Default)]
pub(crate) struct SolidBuilder {
    faces: Vec<Face>,
    lookup: HashMap<FaceKey, usize>,
}

impl SolidBuilder {
    pub(crate) fn face(&mut self, key: FaceKey, surface: SurfaceKind) -> &mut Face {
        let i = *self.lookup.entry(key).or_insert_with(|| {
            self.faces.push(Face {
                key,
                surface,
                polygons: Vec::new(),
            });
            self.faces.len() - 1
        });
        &mut self.faces[i]
    }

    pub(crate) fn push(&mut self, key: FaceKey, surface: SurfaceKind, vertices: Vec<Vec3>) {
        if let Some(p) = Polygon::new(vertices) {
            self.face(key, surface).polygons.push(p);
        }
    }

    /// Adds a quad, splitting it into two triangles when it is not planar (sweeps and
    /// lofts produce those) so the convexity invariant holds.
    pub(crate) fn push_quad(&mut self, key: FaceKey, surface: SurfaceKind, q: [Vec3; 4]) {
        let n = newell_normal(&q);
        let planar = n.length_squared() > 1e-24 && {
            let n = n.normalize();
            let d = q[0].dot(n);
            q.iter().all(|v| (v.dot(n) - d).abs() < MERGE_TOL)
        };
        if planar {
            self.push(key, surface, q.to_vec());
        } else {
            self.push(key, surface, vec![q[0], q[1], q[2]]);
            self.push(key, surface, vec![q[0], q[2], q[3]]);
        }
    }

    pub(crate) fn finish(mut self) -> Solid {
        self.faces.retain(|f| !f.polygons.is_empty());
        for f in &mut self.faces {
            // Generators cannot always know a face's normal up front (sweeps) and may
            // lump several planes under one curve tag (text outlines); settle it here.
            let first = f.polygons[0].plane.normal;
            let coplanar = f
                .polygons
                .iter()
                .all(|p| p.plane.normal.dot(first) > 1.0 - 1e-9);
            f.surface = match (f.surface, coplanar) {
                (SurfaceKind::Planar { .. } | SurfaceKind::Freeform, true) => {
                    SurfaceKind::Planar { normal: first }
                }
                (SurfaceKind::Planar { .. }, false) => SurfaceKind::Freeform,
                (other, _) => other,
            };
        }
        Solid { faces: self.faces }.ensure_outward()
    }
}

/// Orders boundary pieces into closed loops. Pieces that do not close into a loop are
/// dropped: a face whose boundary has a gap has no well-defined region either way, and the
/// caller reports the empty result rather than building a solid from half an outline.
fn chain_loops(mut pieces: Vec<(Vec3, Vec3, u32)>) -> Vec<Vec<(Vec3, Vec3, u32)>> {
    let joined = |a: Vec3, b: Vec3| a.distance_squared(b) < MERGE_TOL * MERGE_TOL;
    let mut loops = Vec::new();
    while let Some(first) = pieces.pop() {
        let mut chain = vec![first];
        loop {
            let tail = chain.last().unwrap().1;
            if joined(tail, chain[0].0) {
                break;
            }
            match pieces.iter().position(|p| joined(p.0, tail)) {
                Some(i) => chain.push(pieces.swap_remove(i)),
                None => break,
            }
        }
        if chain.len() >= 3 && joined(chain.last().unwrap().1, chain[0].0) {
            loops.push(chain);
        }
    }
    loops
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ids::{FaceRole, OpId};
    use approx::assert_relative_eq;

    fn corners_of(s: &Solid) -> Vec<Vec3> {
        crate::pick::corners(&s.edges())
    }

    /// Unit cube built by hand, faces outward.
    pub(crate) fn unit_cube() -> Solid {
        let op = OpId::new(1);
        let v = |x: f64, y: f64, z: f64| Vec3::new(x, y, z);
        let quads: [(FaceRole, [Vec3; 4]); 6] = [
            (
                FaceRole::StartCap,
                [v(0., 0., 0.), v(0., 1., 0.), v(1., 1., 0.), v(1., 0., 0.)],
            ),
            (
                FaceRole::EndCap,
                [v(0., 0., 1.), v(1., 0., 1.), v(1., 1., 1.), v(0., 1., 1.)],
            ),
            (
                FaceRole::Side(0),
                [v(0., 0., 0.), v(1., 0., 0.), v(1., 0., 1.), v(0., 0., 1.)],
            ),
            (
                FaceRole::Side(1),
                [v(1., 0., 0.), v(1., 1., 0.), v(1., 1., 1.), v(1., 0., 1.)],
            ),
            (
                FaceRole::Side(2),
                [v(1., 1., 0.), v(0., 1., 0.), v(0., 1., 1.), v(1., 1., 1.)],
            ),
            (
                FaceRole::Side(3),
                [v(0., 1., 0.), v(0., 0., 0.), v(0., 0., 1.), v(0., 1., 1.)],
            ),
        ];
        let mut b = SolidBuilder::default();
        for (role, q) in quads {
            let n = newell_normal(&q).normalize();
            b.push(
                FaceKey::new(op, role),
                SurfaceKind::Planar { normal: n },
                q.to_vec(),
            );
        }
        b.finish()
    }

    #[test]
    fn cube_mass_properties() {
        let c = unit_cube();
        assert_relative_eq!(c.volume(), 1.0);
        assert_relative_eq!(c.surface_area(), 6.0);
        assert_relative_eq!(c.centroid().x, 0.5);
        assert!(c.is_closed());
        assert_eq!(c.edges().len(), 12);
        let t = c.tessellate();
        assert_eq!(t.mesh.triangle_count(), 12);
        assert_relative_eq!(t.mesh.signed_volume(), 1.0);
    }

    #[test]
    fn edge_segments_follow_face_a_winding() {
        let c = unit_cube();
        for e in c.edges() {
            let s = &e.segments[0];
            let dir = s.end - s.start;
            // Interior of face a is to the left of the edge when walking along it.
            let left = s.normal_a.cross(dir);
            let fa = c.face(e.key.a).unwrap();
            let towards_face = fa.centroid() - (s.start + s.end) * 0.5;
            assert!(left.dot(towards_face) > 0.0, "{:?}", e.key);
        }
    }

    #[test]
    fn reflection_keeps_faces_outward() {
        let mirrored = unit_cube().transformed(&Affine3::from_scale(Vec3::new(-1.0, 1.0, 1.0)));
        assert_relative_eq!(mirrored.volume(), 1.0);
        assert!(mirrored.is_closed());
    }

    /// The split face's own diagonal is no more a display edge than a triangulation
    /// diagonal is: one face is one shape however many polygons fill it.
    #[test]
    fn display_edges_ignore_polygons_inside_a_face() {
        let mut c = unit_cube();
        let i = c
            .faces
            .iter()
            .position(|f| f.key.role == FaceRole::Side(1))
            .unwrap();
        let v = |x: f64, y: f64, z: f64| Vec3::new(x, y, z);
        c.faces[i].polygons = vec![
            Polygon::new(vec![v(1., 0., 0.), v(1., 1., 0.), v(1., 0., 1.)]).unwrap(),
            Polygon::new(vec![v(1., 1., 0.), v(1., 1., 1.), v(1., 0., 1.)]).unwrap(),
        ];
        assert_eq!(c.display_edges().len(), 12);
    }

    /// A fold inside one face is drawn although no face boundary runs along it: the user
    /// sees a crease there, so the outline must show one.
    #[test]
    fn display_edges_include_a_crease_inside_a_face() {
        let mut c = unit_cube();
        let i = c
            .faces
            .iter()
            .position(|f| f.key.role == FaceRole::Side(1))
            .unwrap();
        let v = |x: f64, y: f64, z: f64| Vec3::new(x, y, z);
        // Tent the +x face outwards along y = 0.5.
        c.faces[i].polygons = vec![
            Polygon::new(vec![
                v(1., 0., 0.),
                v(1.5, 0.5, 0.),
                v(1.5, 0.5, 1.),
                v(1., 0., 1.),
            ])
            .unwrap(),
            Polygon::new(vec![
                v(1.5, 0.5, 0.),
                v(1., 1., 0.),
                v(1., 1., 1.),
                v(1.5, 0.5, 1.),
            ])
            .unwrap(),
        ];
        let ridge = c.display_edges().into_iter().find(|[a, b]| {
            a.distance(Vec3::new(1.5, 0.5, 0.)) < 1e-9 && b.distance(Vec3::new(1.5, 0.5, 1.)) < 1e-9
                || b.distance(Vec3::new(1.5, 0.5, 0.)) < 1e-9
                    && a.distance(Vec3::new(1.5, 0.5, 1.)) < 1e-9
        });
        assert!(ridge.is_some(), "the ridge of the tent is a visible edge");
    }

    /// The curved arms of the continuity test, which decide whether a hole drilled in two
    /// steps reads as one bore or as two stacked ones.
    #[test]
    fn a_surface_continues_only_into_the_same_surface() {
        let cyl = |origin: Vec3, radius: f64| SurfaceKind::Cylindrical {
            origin,
            axis: Vec3::Z,
            radius,
        };
        // Same axis line, different point on it: still the same cylinder.
        assert!(cyl(Vec3::ZERO, 4.0).continues(&cyl(Vec3::new(0.0, 0.0, 12.0), 4.0)));
        assert!(!cyl(Vec3::ZERO, 4.0).continues(&cyl(Vec3::ZERO, 4.001)));
        assert!(!cyl(Vec3::ZERO, 4.0).continues(&cyl(Vec3::new(0.1, 0.0, 0.0), 4.0)));
        assert!(
            !cyl(Vec3::ZERO, 4.0).continues(&SurfaceKind::Planar { normal: Vec3::Z }),
            "a bore does not continue into the face it breaks through"
        );

        let cone = |apex: Vec3, half_angle: f64| SurfaceKind::Conical {
            apex,
            axis: Vec3::Z,
            half_angle,
        };
        assert!(cone(Vec3::ZERO, 0.5).continues(&cone(Vec3::ZERO, 0.5)));
        assert!(!cone(Vec3::ZERO, 0.5).continues(&cone(Vec3::ZERO, 0.6)));
        assert!(!cone(Vec3::ZERO, 0.5).continues(&cone(Vec3::new(0.0, 0.0, 1.0), 0.5)));
        // Nothing is known about a freeform surface, so nothing is assumed.
        assert!(!SurfaceKind::Freeform.continues(&SurfaceKind::Freeform));
    }

    /// A sketch line cut in two extrudes into two faces of one flat wall. The user drew a
    /// wall, not two, and no line belongs down the middle of it.
    #[test]
    fn display_edges_ignore_a_seam_between_coplanar_faces() {
        let mut c = unit_cube();
        let i = c
            .faces
            .iter()
            .position(|f| f.key.role == FaceRole::Side(1))
            .unwrap();
        let v = |x: f64, y: f64, z: f64| Vec3::new(x, y, z);
        let lower = Polygon::new(vec![
            v(1., 0., 0.),
            v(1., 1., 0.),
            v(1., 1., 0.5),
            v(1., 0., 0.5),
        ])
        .unwrap();
        c.faces[i].polygons = vec![lower];
        c.faces.push(Face {
            key: FaceKey::new(OpId::new(1), FaceRole::Side(4)),
            surface: c.faces[i].surface,
            polygons: vec![
                Polygon::new(vec![
                    v(1., 0., 0.5),
                    v(1., 1., 0.5),
                    v(1., 1., 1.),
                    v(1., 0., 1.),
                ])
                .unwrap(),
            ],
        });
        c.heal();
        assert!(c.is_closed());
        let seam = c
            .edges()
            .into_iter()
            .find(|e| e.key.touches(FaceKey::new(OpId::new(1), FaceRole::Side(4))) && e.smooth);
        assert!(seam.is_some(), "the two halves of the wall meet smoothly");
        assert_eq!(
            c.display_edges().len(),
            // The wall's four edges are each cut in two by the seam's endpoints.
            14,
            "nothing is drawn along the seam itself"
        );
        assert!(
            !corners_of(&c)
                .iter()
                .any(|p| p.distance(Vec3::new(1., 0., 0.5)) < MERGE_TOL),
            "the seam's ends are not corners the user can snap to"
        );
    }

    /// Healing is entitled to leave two polygons' copies of a shared vertex disagreeing
    /// by up to `MERGE_TOL`, and a boolean routinely does. The outline must not notice:
    /// the viewport used to weld on rounded coordinates, miss such a pair, and draw the
    /// unmatched triangle edge as if the body had a hole there.
    #[test]
    fn display_edges_survive_vertices_that_agree_only_to_merge_tol() {
        let mut c = split_and_healed_cube();
        let before = c.display_edges().len();
        let mut n = 0.0_f64;
        for p in c.faces.iter_mut().flat_map(|f| f.polygons.iter_mut()) {
            for v in &mut p.vertices {
                // Deterministic, always inside the tolerance, never the same twice.
                n += 1.0;
                let dir = Vec3::new(n.sin(), n.cos(), (n * 0.7).sin()).normalize();
                // Half the tolerance each way, so any two copies of a vertex stay the
                // same vertex by the kernel's own definition.
                *v += dir * (0.45 * MERGE_TOL);
            }
        }
        assert_eq!(
            c.display_edges().len(),
            before,
            "outline changed under a nudge inside MERGE_TOL"
        );
    }

    /// The cube with its +x face split in two and the T-junctions that leaves healed:
    /// the shape of a face a boolean has cut through.
    fn split_and_healed_cube() -> Solid {
        let mut c = split_cube();
        c.heal();
        c
    }

    /// The same cube before healing: the +x face is split without telling its neighbours.
    fn split_cube() -> Solid {
        let mut c = unit_cube();
        let i = c
            .faces
            .iter()
            .position(|f| f.key.role == FaceRole::Side(1))
            .unwrap();
        let v = |x: f64, y: f64, z: f64| Vec3::new(x, y, z);
        c.faces[i].polygons = vec![
            Polygon::new(vec![
                v(1., 0., 0.),
                v(1., 1., 0.),
                v(1., 1., 0.5),
                v(1., 0., 0.5),
            ])
            .unwrap(),
            Polygon::new(vec![
                v(1., 0., 0.5),
                v(1., 1., 0.5),
                v(1., 1., 1.),
                v(1., 0., 1.),
            ])
            .unwrap(),
        ];
        c
    }

    #[test]
    fn heal_removes_t_junctions() {
        let mut c = split_cube();
        assert!(!c.is_closed());
        c.heal();
        assert!(c.is_closed());
        assert_eq!(c.edges().len(), 12);
    }

    #[test]
    fn face_profile_recovers_a_flat_face() {
        let cube = unit_cube();
        let key = FaceKey::new(OpId::new(1), FaceRole::EndCap);
        let profile = cube.face_profile(key).expect("planar face");
        assert_eq!(profile.outer.points.len(), 4);
        assert!(profile.holes.is_empty());
        assert_relative_eq!(profile.area(), 1.0, epsilon = 1e-9);
        // The frame sits on the face and faces the way the face does.
        assert_relative_eq!(profile.frame.origin.z, 1.0, epsilon = 1e-9);
        assert_relative_eq!(profile.frame.z.z, 1.0, epsilon = 1e-9);
        // Normalised profiles wind counter-clockwise, which is what the generators need.
        assert!(profile.outer.signed_area() > 0.0);
        // Each stretch is tagged by the face it borders, so the four sides differ.
        let mut tags: Vec<u32> = profile.outer.segments.iter().map(|s| s.curve).collect();
        tags.sort_unstable();
        tags.dedup();
        assert_eq!(tags.len(), 4);
    }

    #[test]
    fn face_profile_survives_a_split_face() {
        // Booleans leave a face as several polygons; the profile must be its outline, not
        // the polygons' shared edges as well.
        let mut cube = unit_cube();
        let i = cube
            .faces
            .iter()
            .position(|f| f.key.role == FaceRole::EndCap)
            .unwrap();
        let v = |x: f64, y: f64| Vec3::new(x, y, 1.0);
        cube.faces[i].polygons = vec![
            Polygon::new(vec![v(0., 0.), v(0.5, 0.), v(0.5, 1.), v(0., 1.)]).unwrap(),
            Polygon::new(vec![v(0.5, 0.), v(1., 0.), v(1., 1.), v(0.5, 1.)]).unwrap(),
        ];
        cube.heal();
        let profile = cube
            .face_profile(FaceKey::new(OpId::new(1), FaceRole::EndCap))
            .expect("planar face");
        assert_relative_eq!(profile.area(), 1.0, epsilon = 1e-9);
        assert!(profile.holes.is_empty());
    }

    #[test]
    fn face_profile_rejects_missing_and_curved_faces() {
        let mut cube = unit_cube();
        let missing = FaceKey::new(OpId::new(9), FaceRole::StartCap);
        assert!(matches!(
            cube.face_profile(missing),
            Err(KernelError::MissingFace(_))
        ));
        let i = cube
            .faces
            .iter()
            .position(|f| f.key.role == FaceRole::Side(0))
            .unwrap();
        let key = cube.faces[i].key;
        cube.faces[i].surface = SurfaceKind::Cylindrical {
            origin: Vec3::ZERO,
            axis: Vec3::Z,
            radius: 1.0,
        };
        assert!(matches!(
            cube.face_profile(key),
            Err(KernelError::NotPlanarFace(_))
        ));
    }

    #[test]
    fn neighbour_tags_are_distinct_and_stable() {
        let op = OpId::new(3);
        let a = neighbour_tag(FaceKey::new(op, FaceRole::StartCap));
        assert_eq!(a, neighbour_tag(FaceKey::new(op, FaceRole::StartCap)));
        assert_ne!(a, neighbour_tag(FaceKey::new(op, FaceRole::EndCap)));
        assert_ne!(
            a,
            neighbour_tag(FaceKey::new(OpId::new(4), FaceRole::StartCap))
        );
        assert_ne!(
            neighbour_tag(FaceKey::new(op, FaceRole::Side(1))),
            neighbour_tag(FaceKey::new(op, FaceRole::Side(2)))
        );
        // The payload and the variant are both hashed, so a side and a fillet of the same
        // index do not collide.
        assert_ne!(
            neighbour_tag(FaceKey::new(op, FaceRole::Side(1))),
            neighbour_tag(FaceKey::new(op, FaceRole::Fillet(1)))
        );
    }

    /// The fan triangulation this replaced covered a non-convex polygon with triangles
    /// that spilled outside it, so the mesh a slicer saw did not match the solid.
    #[test]
    fn triangulation_covers_a_non_convex_polygon_exactly() {
        // An L in the z = 0 plane, wound counter-clockwise about +z.
        let v = |x: f64, y: f64| Vec3::new(x, y, 0.0);
        let p = Polygon::new(vec![
            v(0., 0.),
            v(2., 0.),
            v(2., 1.),
            v(1., 1.),
            v(1., 2.),
            v(0., 2.),
        ])
        .unwrap();
        let tris = triangulate_polygon(&p);
        assert_eq!(tris.len(), 4, "six vertices, so four triangles");
        let area: f64 = tris
            .iter()
            .map(|&[a, b, c]| {
                let (a, b, c) = (p.vertices[a], p.vertices[b], p.vertices[c]);
                (b - a).cross(c - a).dot(p.plane.normal) / 2.0
            })
            .sum();
        // Signed, so a triangle laid down the wrong way or outside the L shows up.
        assert_relative_eq!(area, 3.0, epsilon = 1e-12);
    }

    /// Healing leaves T-junction vertices sitting mid-edge. A fan from vertex 0 made a
    /// zero-area triangle out of each one and `tessellate` used to drop those, which took
    /// a bite out of the shell: a hole in the STL and a non-manifold edge in the 3MF.
    #[test]
    fn tessellation_of_a_healed_solid_keeps_every_triangle_and_stays_closed() {
        let c = split_and_healed_cube();
        let mesh = c.tessellate().mesh;
        let expected: usize = c
            .faces
            .iter()
            .flat_map(|f| f.polygons.iter())
            .map(|p| p.vertices.len() - 2)
            .sum();
        assert_eq!(mesh.triangle_count(), expected, "a triangle went missing");
        assert_eq!(unmatched_mesh_edges(&mesh), 0);
    }

    /// Every triangle edge should be walked once in each direction. Counts by welded
    /// vertex id, the way an exporter and a slicer do.
    fn unmatched_mesh_edges(mesh: &TriMesh) -> usize {
        let mut index = VertexIndex::default();
        let mut directed: HashMap<(u32, u32), i32> = HashMap::new();
        for i in 0..mesh.triangle_count() {
            let ids: Vec<u32> = mesh.triangle(i).iter().map(|v| index.id(*v)).collect();
            for (j, &a) in ids.iter().enumerate() {
                let b = ids[(j + 1) % 3];
                if a == b {
                    continue;
                }
                let (k, s) = if a < b { ((a, b), 1) } else { ((b, a), -1) };
                *directed.entry(k).or_default() += s;
            }
        }
        directed.values().filter(|c| **c != 0).count()
    }

    /// Healing used to split an edge at a vertex a few microns off its own start: the
    /// parametric guard let `t ≈ 1e-7` through on a 14 mm edge. The polygon came back
    /// with a zero-area spur out to a point the merge tolerance calls its own start, and
    /// the spur's edges matched nothing.
    #[test]
    fn healing_does_not_split_an_edge_at_its_own_endpoint() {
        let mut c = unit_cube();
        // A stray vertex a tenth of the tolerance off one corner, on another face, so
        // heal sees it as a candidate split point for every edge meeting that corner.
        let i = c
            .faces
            .iter()
            .position(|f| f.key.role == FaceRole::StartCap)
            .unwrap();
        c.faces[i].polygons[0].vertices[0] += Vec3::new(0.0, 0.1 * MERGE_TOL, 0.0);
        c.heal();
        for p in c.faces.iter().flat_map(|f| f.polygons.iter()) {
            let n = p.vertices.len();
            for (j, a) in p.vertices.iter().enumerate() {
                for b in &p.vertices[j + 1..] {
                    assert!(
                        a.distance(*b) > MERGE_TOL,
                        "polygon visits the same point twice: {:?}",
                        p.vertices
                    );
                }
            }
            assert_eq!(n, p.vertices.len());
        }
        assert!(c.is_closed());
    }

    /// The snap is what stops error compounding across booleans: after healing, two
    /// polygons' copies of a shared vertex are the same `f64`s, not merely close ones.
    #[test]
    fn healing_snaps_near_coincident_vertices_onto_one_point() {
        let mut c = unit_cube();
        let mut n = 0.0_f64;
        for p in c.faces.iter_mut().flat_map(|f| f.polygons.iter_mut()) {
            for v in &mut p.vertices {
                n += 1.0;
                let dir = Vec3::new(n.sin(), n.cos(), (n * 0.7).sin()).normalize();
                *v += dir * (0.45 * MERGE_TOL);
            }
        }
        c.heal();
        let mut seen: Vec<Vec3> = Vec::new();
        for v in c
            .faces
            .iter()
            .flat_map(|f| f.polygons.iter())
            .flat_map(|p| &p.vertices)
        {
            match seen.iter().find(|s| s.distance(*v) <= MERGE_TOL) {
                Some(s) => assert_eq!(s, v, "two copies of one vertex still differ"),
                None => seen.push(*v),
            }
        }
        assert_eq!(seen.len(), 8, "a unit cube has eight corners");
    }

    #[test]
    fn vertex_index_merges_across_cells() {
        let mut idx = VertexIndex::default();
        let a = idx.id(Vec3::new(2.0 * MERGE_TOL - 1e-9, 0.0, 0.0));
        let b = idx.id(Vec3::new(2.0 * MERGE_TOL + 1e-9, 0.0, 0.0));
        assert_eq!(a, b);
        assert_ne!(a, idx.id(Vec3::new(1.0, 0.0, 0.0)));
    }
}
