//! The boundary representation.
//!
//! A [`Solid`] is a closed shell of [`Face`]s. Each face owns a set of planar convex
//! polygons plus a [`SurfaceKind`] recording the analytic surface those polygons
//! approximate. Booleans only ever split polygons, so a face's key and surface survive
//! every downstream operation: that is what keeps references from later features valid.
//!
//! Polygons are kept convex because the BSP splitter in [`crate::csg`] relies on it.
//! Curved surfaces are faceted at creation time; the surface kind is what lets the
//! tessellator shade them smoothly and lets selection report "cylinder, radius 5".

use std::collections::HashMap;

use basset_math::{Aabb, Affine3, Frame, Plane, TriMesh, Vec2, Vec3};

use crate::error::KernelError;
use crate::geometry::{Contour, Profile, Segment};
use crate::ids::{EdgeKey, FaceKey};

/// Distance below which two vertices are the same vertex. Looser than the maths crate's
/// `LINEAR_TOL` because BSP splitting accumulates a little error per split.
pub const MERGE_TOL: f64 = 1e-6;

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

    /// Splits polygon edges at vertices of neighbouring polygons that lie on them.
    ///
    /// BSP splitting cuts a polygon by a plane without touching its neighbours, so the
    /// result is riddled with T-junctions. Edge extraction and closedness checks need every
    /// shared edge to be shared vertex-for-vertex, so booleans call this before returning.
    pub(crate) fn heal(&mut self) {
        let mut index = VertexIndex::default();
        for p in self.faces.iter().flat_map(|f| f.polygons.iter()) {
            for v in &p.vertices {
                index.id(*v);
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
                let mut inserts: Vec<(f64, Vec3)> = Vec::new();
                for &(_, q) in by_x[start..].iter().take_while(|(x, _)| *x <= hi) {
                    let t = (q - a).dot(ab) / len2;
                    if t <= 1e-9 || t >= 1.0 - 1e-9 {
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

    /// Boundaries between distinct faces. Requires a healed solid (every operation that
    /// returns a `Solid` guarantees this).
    pub fn edges(&self) -> Vec<Edge> {
        let mut index = VertexIndex::default();
        // (face index, directed a→b, polygon normal), keyed by undirected vertex pair.
        type User = (usize, bool, Vec3);
        let mut shared: HashMap<(u32, u32), Vec<User>> = HashMap::new();
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
        let mut edges: HashMap<EdgeKey, Vec<EdgeSegment>> = HashMap::new();
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
            edges.entry(key).or_default().push(EdgeSegment {
                start,
                end,
                normal_a: na,
                normal_b: nb,
            });
        }
        // Hash-map order would make the chain start point (and so blend tool geometry)
        // vary between runs; sort so identical inputs always give identical solids.
        let mut out: Vec<Edge> = edges
            .into_iter()
            .map(|(key, mut segments)| {
                segments.sort_by(|x, y| {
                    x.start
                        .to_array()
                        .partial_cmp(&y.start.to_array())
                        .unwrap_or(std::cmp::Ordering::Equal)
                });
                Edge { key, segments }
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
                for i in 1..p.vertices.len() - 1 {
                    let tri = [p.vertices[0], p.vertices[i], p.vertices[i + 1]];
                    if (tri[1] - tri[0]).cross(tri[2] - tri[0]).length_squared() < 1e-24 {
                        continue;
                    }
                    let base = mesh.positions.len() as u32;
                    mesh.positions.extend_from_slice(&tri);
                    match &smooth {
                        Some(s) => mesh.normals.extend([s[pi][0], s[pi][i], s[pi][i + 1]]),
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

/// Per-polygon, per-vertex normals averaged over the face's polygons that meet at the
/// vertex with a similar orientation. The angle cut-off keeps genuine creases inside a
/// face (text glyph corners, for example) sharp while smoothing facet seams.
fn smooth_normals(face: &Face) -> Vec<Vec<Vec3>> {
    const CREASE_COS: f64 = 0.7; // ≈ 45°
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
                        .filter(|m| m.dot(p.plane.normal) > CREASE_COS)
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

    #[test]
    fn heal_removes_t_junctions() {
        let mut c = unit_cube();
        // Split the +x face into two halves without telling its neighbours.
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

    #[test]
    fn vertex_index_merges_across_cells() {
        let mut idx = VertexIndex::default();
        let a = idx.id(Vec3::new(2.0 * MERGE_TOL - 1e-9, 0.0, 0.0));
        let b = idx.id(Vec3::new(2.0 * MERGE_TOL + 1e-9, 0.0, 0.0));
        assert_eq!(a, b);
        assert_ne!(a, idx.id(Vec3::new(1.0, 0.0, 0.0)));
    }
}
