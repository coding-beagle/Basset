//! Boolean operations by BSP partitioning.
//!
//! This is the classic csg.js scheme: each solid becomes a binary space partition tree,
//! each tree clips the other's polygons, and the surviving fragments are merged. It is
//! exact for the polygonal representation (no re-fitting of curves), which is why the
//! MVP kernel can afford it, and every surviving fragment keeps the face it came from,
//! which is why face keys survive booleans.
//!
//! Known limits, inherited from the algorithm: overlapping *coplanar* faces with the same
//! orientation are both kept, and the tree is only as shallow as the shape allows. A
//! splitting plane is taken from one of the polygons, so on a convex body — a cylinder,
//! or a fillet tool swept round a rim — every candidate leaves the whole remainder in
//! front of it, the tree degenerates into a list of depth `n`, and both the build and the
//! clip cost `O(n²)`. Measured on one rim of a cylinder: 450 tool facets take 10 ms,
//! 1700 take 175 ms, 8.8k take 5.3 s, 14k take 7.9 s, and 30k overflows the stack.
//! [`blend`](crate::blend) budgets its tools against those numbers; a general fix means
//! splitting on planes that are not face planes, which this scheme cannot express.
//!
//! Keeping every fragment under its own key is right for naming and wrong for what the
//! user sees, because two bodies joined flush leave the one flat face they now share as
//! two. [`Solid::merge_continuous_faces`] is the other half of the assembly: it puts the
//! faces different operations have grown into one patch of surface back together.

use basset_math::Vec3;

use crate::error::KernelError;
use crate::solid::{Face, MERGE_TOL, Polygon, Solid, SurfaceKind};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BoolOp {
    Union,
    Subtract,
    Intersect,
}

/// Distance from a plane below which a vertex counts as lying on it.
///
/// This is the merge tolerance, and it has to be: a vertex further from the plane than
/// this but closer than [`MERGE_TOL`] would make its edge "spanning", and the split point
/// manufactured on that edge would land within the merge tolerance of the vertex itself.
/// That near-duplicate is what a later split drags further out of place, and it is what
/// leaves the shell with slivers and holes no amount of healing can pair up.
const EPSILON: f64 = MERGE_TOL;

pub fn boolean(a: &Solid, b: &Solid, op: BoolOp) -> Result<Solid, KernelError> {
    let sources = [a, b];
    let mut na = Node::new(polygons_of(a, 0));
    let mut nb = Node::new(polygons_of(b, 1));
    let result = match op {
        BoolOp::Union => {
            na.clip_to(&nb);
            nb.clip_to(&na);
            nb.invert();
            nb.clip_to(&na);
            nb.invert();
            na.build(nb.all_polygons());
            na.all_polygons()
        }
        BoolOp::Subtract => {
            na.invert();
            na.clip_to(&nb);
            nb.clip_to(&na);
            nb.invert();
            nb.clip_to(&na);
            nb.invert();
            na.build(nb.all_polygons());
            na.invert();
            na.all_polygons()
        }
        BoolOp::Intersect => {
            na.invert();
            nb.clip_to(&na);
            nb.invert();
            na.clip_to(&nb);
            nb.clip_to(&na);
            na.build(nb.all_polygons());
            na.invert();
            na.all_polygons()
        }
    };
    assemble(result, &sources)
}

/// Polygon tagged with where it came from so the result can be regrouped into faces.
#[derive(Clone, Debug)]
struct CsgPolygon {
    vertices: Vec<Vec3>,
    normal: Vec3,
    w: f64,
    /// (solid index, face index)
    source: (usize, usize),
    /// Odd number of inversions: the polygon now faces the other way.
    flipped: bool,
}

impl CsgPolygon {
    fn flip(&mut self) {
        self.vertices.reverse();
        self.normal = -self.normal;
        self.w = -self.w;
        self.flipped = !self.flipped;
    }
}

fn polygons_of(s: &Solid, index: usize) -> Vec<CsgPolygon> {
    let mut out = Vec::with_capacity(s.polygon_count());
    for (fi, f) in s.faces.iter().enumerate() {
        for p in &f.polygons {
            out.push(CsgPolygon {
                vertices: p.vertices.clone(),
                normal: p.plane.normal,
                w: p.plane.normal.dot(p.plane.origin),
                source: (index, fi),
                flipped: false,
            });
        }
    }
    out
}

fn assemble(polys: Vec<CsgPolygon>, sources: &[&Solid; 2]) -> Result<Solid, KernelError> {
    let mut faces: Vec<Face> = Vec::new();
    for p in polys {
        let Some(polygon) = Polygon::new(p.vertices) else {
            continue;
        };
        let src = &sources[p.source.0].faces[p.source.1];
        let surface = match (src.surface, p.flipped) {
            (SurfaceKind::Planar { normal }, true) => SurfaceKind::Planar { normal: -normal },
            (s, _) => s,
        };
        match faces.iter_mut().find(|f| f.key == src.key) {
            Some(f) => f.polygons.push(polygon),
            None => faces.push(Face {
                key: src.key,
                surface,
                polygons: vec![polygon],
            }),
        }
    }
    let mut solid = Solid { faces };
    solid.prune();
    if solid.is_empty() || solid.volume() < 1e-12 {
        return Err(KernelError::EmptyResult);
    }
    solid.heal();
    solid.merge_continuous_faces();
    Ok(solid)
}

#[derive(Clone, Copy)]
struct SplitPlane {
    normal: Vec3,
    w: f64,
}

const COPLANAR: u8 = 0;
const FRONT: u8 = 1;
const BACK: u8 = 2;
const SPANNING: u8 = 3;

impl SplitPlane {
    fn flip(&mut self) {
        self.normal = -self.normal;
        self.w = -self.w;
    }

    fn classify(&self, v: Vec3) -> u8 {
        let t = self.normal.dot(v) - self.w;
        if t < -EPSILON {
            BACK
        } else if t > EPSILON {
            FRONT
        } else {
            COPLANAR
        }
    }

    /// Sorts `polygon` into the four bins, splitting it when it straddles the plane.
    fn split(
        &self,
        polygon: &CsgPolygon,
        coplanar_front: &mut Vec<CsgPolygon>,
        coplanar_back: &mut Vec<CsgPolygon>,
        front: &mut Vec<CsgPolygon>,
        back: &mut Vec<CsgPolygon>,
    ) {
        let mut polygon_type = 0u8;
        let types: Vec<u8> = polygon
            .vertices
            .iter()
            .map(|v| {
                let t = self.classify(*v);
                polygon_type |= t;
                t
            })
            .collect();
        match polygon_type {
            COPLANAR => {
                if self.normal.dot(polygon.normal) > 0.0 {
                    coplanar_front.push(polygon.clone());
                } else {
                    coplanar_back.push(polygon.clone());
                }
            }
            FRONT => front.push(polygon.clone()),
            BACK => back.push(polygon.clone()),
            _ => {
                let n = polygon.vertices.len();
                let mut f = Vec::with_capacity(n + 1);
                let mut b = Vec::with_capacity(n + 1);
                for i in 0..n {
                    let j = (i + 1) % n;
                    let (ti, tj) = (types[i], types[j]);
                    let (vi, vj) = (polygon.vertices[i], polygon.vertices[j]);
                    if ti != BACK {
                        f.push(vi);
                    }
                    if ti != FRONT {
                        b.push(vi);
                    }
                    if (ti | tj) == SPANNING {
                        let t = (self.w - self.normal.dot(vi)) / self.normal.dot(vj - vi);
                        let v = vi.lerp(vj, t);
                        f.push(v);
                        b.push(v);
                    }
                }
                if let Some(p) = fragment(polygon, f) {
                    front.push(p);
                }
                if let Some(p) = fragment(polygon, b) {
                    back.push(p);
                }
            }
        }
    }
}

/// A piece of `parent` with new vertices; dropped if it has collapsed to nothing, since
/// zero-area slivers only add noise to later splits.
fn fragment(parent: &CsgPolygon, vertices: Vec<Vec3>) -> Option<CsgPolygon> {
    if vertices.len() < 3 {
        return None;
    }
    if crate::solid::newell_normal(&vertices).length_squared() < 1e-28 {
        return None;
    }
    Some(CsgPolygon {
        vertices,
        ..parent.clone()
    })
}

struct Node {
    plane: Option<SplitPlane>,
    front: Option<Box<Node>>,
    back: Option<Box<Node>>,
    polygons: Vec<CsgPolygon>,
}

impl Node {
    fn empty() -> Box<Node> {
        Box::new(Node {
            plane: None,
            front: None,
            back: None,
            polygons: Vec::new(),
        })
    }

    fn new(polygons: Vec<CsgPolygon>) -> Self {
        let mut n = *Node::empty();
        n.build(polygons);
        n
    }

    fn invert(&mut self) {
        for p in &mut self.polygons {
            p.flip();
        }
        if let Some(p) = &mut self.plane {
            p.flip();
        }
        if let Some(f) = &mut self.front {
            f.invert();
        }
        if let Some(b) = &mut self.back {
            b.invert();
        }
        std::mem::swap(&mut self.front, &mut self.back);
    }

    /// Removes the parts of `polygons` inside this tree.
    fn clip_polygons(&self, polygons: Vec<CsgPolygon>) -> Vec<CsgPolygon> {
        let Some(plane) = self.plane else {
            return polygons;
        };
        let mut front = Vec::new();
        let mut back = Vec::new();
        let (mut coplanar_front, mut coplanar_back) = (Vec::new(), Vec::new());
        for p in &polygons {
            plane.split(
                p,
                &mut coplanar_front,
                &mut coplanar_back,
                &mut front,
                &mut back,
            );
        }
        // Coplanar polygons go with the side their normal agrees with.
        front.extend(coplanar_front);
        back.extend(coplanar_back);
        let mut front = match &self.front {
            Some(f) => f.clip_polygons(front),
            None => front,
        };
        let back = match &self.back {
            Some(b) => b.clip_polygons(back),
            None => Vec::new(),
        };
        front.extend(back);
        front
    }

    fn clip_to(&mut self, other: &Node) {
        self.polygons = other.clip_polygons(std::mem::take(&mut self.polygons));
        if let Some(f) = &mut self.front {
            f.clip_to(other);
        }
        if let Some(b) = &mut self.back {
            b.clip_to(other);
        }
    }

    fn all_polygons(&self) -> Vec<CsgPolygon> {
        let mut out = self.polygons.clone();
        if let Some(f) = &self.front {
            out.extend(f.all_polygons());
        }
        if let Some(b) = &self.back {
            out.extend(b.all_polygons());
        }
        out
    }

    fn build(&mut self, polygons: Vec<CsgPolygon>) {
        if polygons.is_empty() {
            return;
        }
        // Whether the plane comes from this very set of polygons, which is what the
        // progress check below relies on; a node that already had a plane got it from a
        // different set and a one-sided split there is ordinary.
        let mut chosen_here = false;
        let plane = *self.plane.get_or_insert_with(|| {
            chosen_here = true;
            choose_plane(&polygons)
        });
        let mut front = Vec::new();
        let mut back = Vec::new();
        let (mut coplanar_front, mut coplanar_back) = (Vec::new(), Vec::new());
        for p in &polygons {
            plane.split(
                p,
                &mut coplanar_front,
                &mut coplanar_back,
                &mut front,
                &mut back,
            );
        }
        // The recursion shrinks only because a plane taken from these polygons consumes
        // at least the polygon it came from. A polygon whose vertices stray further from
        // its own plane than EPSILON — healing inserts a T-junction vertex within
        // MERGE_TOL of an edge without projecting it onto the face, and `Polygon::new`
        // then centres the plane on a centroid that the stray vertex has pulled off it —
        // fails that test against its own plane and lands whole on one side. The child
        // would get an identical set, deterministically choose the same plane, and
        // recurse until the stack ran out. Keep the set here instead: it is within a
        // hair of the plane, so coplanar is also the right answer geometrically.
        let made_progress = !coplanar_front.is_empty()
            || !coplanar_back.is_empty()
            || (!front.is_empty() && !back.is_empty());
        if chosen_here && !made_progress {
            self.polygons.extend(polygons);
            return;
        }
        // `front` and `back` already hold a copy of everything that is still wanted, so
        // the input set is dead here — but it is a local, and a local lives until the end
        // of the function, i.e. across both recursive calls. A tree that degenerates into
        // a list (which is what a tool swept round a curved edge produces: every facet
        // plane leaves the rest of the tool in front of it) then keeps one nearly-full
        // set alive per level, and the peak is quadratic in the polygon count rather than
        // linear. Measured on one rim of a cylinder: 1700 tool facets peaked at 252 MB
        // before this line and 19 MB after it.
        drop(polygons);
        self.polygons.extend(coplanar_front);
        self.polygons.extend(coplanar_back);
        if !front.is_empty() {
            self.front.get_or_insert_with(Node::empty).build(front);
        }
        if !back.is_empty() {
            self.back.get_or_insert_with(Node::empty).build(back);
        }
    }
}

/// Picks the splitting plane among a few evenly spaced candidates by the usual heuristic:
/// penalise splits heavily and imbalance lightly. Cheap, and it keeps trees for boxes and
/// cylinders shallow where "take the first polygon" would produce a linked list. It can
/// only rank the planes it is given, though, and on a convex body they are all equally
/// bad: see the module header.
fn choose_plane(polygons: &[CsgPolygon]) -> SplitPlane {
    const CANDIDATES: usize = 6;
    /// Polygons each candidate is scored against; see the sampling note below.
    const SCORE_SAMPLE: usize = 64;
    let step = (polygons.len() / CANDIDATES).max(1);
    // Scored against a bounded sample rather than the whole set. Scoring every polygon
    // against every candidate costs `CANDIDATES · n` at a node whose subtree will visit
    // `O(n)` nodes, which made the ranking itself the dominant term: on a 14k-facet
    // fillet tool it was 1.0e9 vertex classifications against 2.0e8 for all the actual
    // splitting, and dropping it to a sample took the boolean from 14.7 s to 7.9 s for
    // the same volume. A ranking is a guess either way, and a sample of this size still
    // separates a plane that halves the set from one that shaves it.
    let sample_step = (polygons.len() / SCORE_SAMPLE).max(1);
    let mut best: Option<(f64, SplitPlane)> = None;
    for candidate in polygons.iter().step_by(step).take(CANDIDATES) {
        let plane = SplitPlane {
            normal: candidate.normal,
            w: candidate.w,
        };
        let (mut f, mut b, mut s) = (0i64, 0i64, 0i64);
        for p in polygons.iter().step_by(sample_step) {
            let mut t = 0u8;
            for v in &p.vertices {
                t |= plane.classify(*v);
            }
            match t {
                FRONT => f += 1,
                BACK => b += 1,
                SPANNING => s += 1,
                _ => {}
            }
        }
        let score = 4.0 * s as f64 + (f - b).abs() as f64;
        if best.is_none_or(|(bs, _)| score < bs) {
            best = Some((score, plane));
        }
    }
    best.map(|(_, p)| p).unwrap_or(SplitPlane {
        normal: polygons[0].normal,
        w: polygons[0].w,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ids::{FaceRole, OpId};
    use crate::primitives::cuboid;
    use approx::assert_relative_eq;
    use basset_math::Vec3;

    /// The splitter's "on the plane" tolerance is the merge tolerance, and has to be.
    /// A vertex further off than this but closer than `MERGE_TOL` used to make its edge
    /// spanning, and the crossing point invented on that edge landed within the merge
    /// tolerance of the vertex: a near-duplicate that every later split drags further out
    /// of place, until the shell has a crack the healer cannot close.
    #[test]
    fn a_vertex_inside_the_merge_tolerance_of_a_plane_does_not_split_its_edge() {
        let v = |x: f64, y: f64| Vec3::new(x, y, 0.0);
        // A triangle with one corner half a merge tolerance past the plane x = 0.
        let poly = CsgPolygon {
            vertices: vec![v(-0.5 * MERGE_TOL, 0.0), v(1.0, 0.0), v(1.0, 1.0)],
            normal: Vec3::Z,
            w: 0.0,
            source: (0, 0),
            flipped: false,
        };
        let plane = SplitPlane {
            normal: Vec3::X,
            w: 0.0,
        };
        let (mut cf, mut cb, mut front, mut back) = (vec![], vec![], vec![], vec![]);
        plane.split(&poly, &mut cf, &mut cb, &mut front, &mut back);
        assert_eq!(back.len(), 0, "no sliver behind the plane");
        assert_eq!(front.len(), 1);
        assert_eq!(
            front[0].vertices, poly.vertices,
            "the triangle came through untouched"
        );
    }

    fn cube(op: u64, min: Vec3, size: f64) -> Solid {
        cuboid(OpId::new(op), min, min + Vec3::splat(size))
    }

    #[test]
    fn union_of_overlapping_cubes() {
        let a = cube(1, Vec3::ZERO, 2.0);
        let b = cube(2, Vec3::splat(1.0), 2.0);
        let u = boolean(&a, &b, BoolOp::Union).unwrap();
        assert_relative_eq!(u.volume(), 8.0 + 8.0 - 1.0, epsilon = 1e-9);
        assert!(u.is_closed(), "{:?}", u.validate());
        assert_eq!(
            u.faces.len(),
            12,
            "every original face survives with a fragment"
        );
    }

    /// Two blocks joined side by side are one block: the tops that finish at the same
    /// height are one face the user can pick, sketch on and export, not two.
    #[test]
    fn union_joins_faces_that_run_on_into_each_other() {
        let a = cube(1, Vec3::ZERO, 2.0);
        let b = cube(2, Vec3::new(2.0, 0.0, 0.0), 2.0);
        let u = boolean(&a, &b, BoolOp::Union).unwrap();
        assert_relative_eq!(u.volume(), 16.0, epsilon = 1e-9);
        assert!(u.is_closed(), "{:?}", u.validate());
        // Six faces, as for the 4 × 2 × 2 block this is: the tops, the bottoms and the
        // two long sides each merged, the shared wall between them is gone, and the two
        // ends are what is left of the cubes' own outer sides.
        assert_eq!(u.faces.len(), 6, "{:?}", u.face_keys().collect::<Vec<_>>());
        let top = u
            .face(crate::ids::FaceKey::new(OpId::new(1), FaceRole::EndCap))
            .expect("named by the earlier of the two operations");
        assert_relative_eq!(top.area(), 8.0, epsilon = 1e-9);
        // And it traces as one region, which is what a sketch or a push on it would use.
        let profile = u
            .face_profile(crate::ids::FaceKey::new(OpId::new(1), FaceRole::EndCap))
            .unwrap();
        assert_relative_eq!(profile.area(), 8.0, epsilon = 1e-9);
    }

    #[test]
    fn subtract_keeps_face_keys_of_the_target_and_flips_tool_surfaces() {
        let a = cube(1, Vec3::ZERO, 2.0);
        let b = cube(2, Vec3::new(0.5, 0.5, 1.0), 1.0);
        let d = boolean(&a, &b, BoolOp::Subtract).unwrap();
        assert_relative_eq!(d.volume(), 8.0 - 1.0, epsilon = 1e-9);
        assert!(d.is_closed(), "{:?}", d.validate());
        // The pocket floor comes from the tool's start cap and now faces up.
        let floor = d
            .face(crate::ids::FaceKey::new(OpId::new(2), FaceRole::StartCap))
            .unwrap();
        assert_eq!(floor.surface, SurfaceKind::Planar { normal: Vec3::Z });
        assert!(
            d.face(crate::ids::FaceKey::new(OpId::new(1), FaceRole::EndCap))
                .is_some()
        );
    }

    /// A polygon further from its own plane than `EPSILON` used to hang the builder.
    ///
    /// `heal` inserts a T-junction vertex that lies within `MERGE_TOL` (1e-6) of an edge
    /// without projecting it onto the face, and `Polygon::new` centres the plane on the
    /// vertex centroid, so a healed face polygon can stray from its own plane by more
    /// than the 1e-7 the classifier allows. It then lands whole on one side of a plane
    /// taken from itself, the child node receives an identical set, `choose_plane` is
    /// deterministic and picks the same plane again, and the recursion never ends.
    #[test]
    fn build_terminates_on_a_polygon_that_misses_its_own_plane() {
        // A plane at y = 8 facing -y, and a polygon with two vertices nudged behind it by
        // more than the classifier's tolerance.
        let normal = Vec3::new(0.0, -1.0, 0.0);
        let w = -8.0;
        let off = 2.0 * EPSILON;
        let poly = CsgPolygon {
            vertices: vec![
                Vec3::new(0.0, 8.0, 0.0),
                Vec3::new(1.0, 8.0 + off, 0.0),
                Vec3::new(1.0, 8.0 + off, 1.0),
                Vec3::new(0.0, 8.0, 1.0),
            ],
            normal,
            w,
            source: (0, 0),
            flipped: false,
        };
        // The premise: against its own plane the polygon is behind, never coplanar, so
        // the split consumes nothing.
        let plane = SplitPlane { normal, w };
        let types: Vec<u8> = poly.vertices.iter().map(|v| plane.classify(*v)).collect();
        assert!(types.contains(&BACK), "{types:?}");
        assert!(
            !types.contains(&FRONT),
            "must not span, or it would be split"
        );

        let node = Node::new(vec![poly]);
        // Kept at the node rather than pushed into a child that could not shrink it.
        assert_eq!(node.all_polygons().len(), 1);
        assert!(node.front.is_none() && node.back.is_none());
    }

    /// The same degeneracy reaching `boolean`: the union must still terminate and report
    /// the right volume.
    #[test]
    fn union_survives_a_face_polygon_that_misses_its_own_plane() {
        let mut a = cube(1, Vec3::ZERO, 2.0);
        let b = cube(2, Vec3::splat(1.0), 2.0);
        // Nudge one vertex of every polygon off its face, far enough that the splitter
        // does not write it off as coplanar: the case where a polygon is not flat and the
        // tree has to cope anyway.
        let nudge = 2.0 * EPSILON;
        for f in &mut a.faces {
            for p in &mut f.polygons {
                p.vertices[0] += p.plane.normal * nudge;
            }
        }
        let u = boolean(&a, &b, BoolOp::Union).unwrap();
        // Moving corners outward adds a little real volume; the tolerance tracks the
        // nudge rather than being a fixed number that quietly stops meaning anything.
        assert_relative_eq!(u.volume(), 8.0 + 8.0 - 1.0, epsilon = 8.0 * nudge);
    }

    #[test]
    fn intersect_of_offset_cubes() {
        let a = cube(1, Vec3::ZERO, 2.0);
        let b = cube(2, Vec3::splat(1.0), 2.0);
        let i = boolean(&a, &b, BoolOp::Intersect).unwrap();
        assert_relative_eq!(i.volume(), 1.0, epsilon = 1e-9);
        assert!(i.is_closed());
        assert_eq!(i.edges().len(), 12);
    }

    #[test]
    fn disjoint_subtract_leaves_target_unchanged() {
        let a = cube(1, Vec3::ZERO, 1.0);
        let b = cube(2, Vec3::splat(5.0), 1.0);
        let d = boolean(&a, &b, BoolOp::Subtract).unwrap();
        assert_relative_eq!(d.volume(), 1.0, epsilon = 1e-12);
        assert!(d.is_closed());
    }

    #[test]
    fn subtracting_everything_is_an_error() {
        let a = cube(1, Vec3::ZERO, 1.0);
        let b = cube(2, Vec3::splat(-1.0), 3.0);
        assert_eq!(
            boolean(&a, &b, BoolOp::Subtract),
            Err(KernelError::EmptyResult)
        );
    }

    #[test]
    fn coplanar_join_on_top_face() {
        // The most common modelling case: extrude a smaller block up from the top face.
        let a = cube(1, Vec3::ZERO, 2.0);
        let b = cube(2, Vec3::new(0.5, 0.5, 2.0), 1.0);
        let u = boolean(&a, &b, BoolOp::Union).unwrap();
        assert_relative_eq!(u.volume(), 9.0, epsilon = 1e-9);
        assert!(u.is_closed(), "{:?}", u.validate());
        // The tool's bottom cap was swallowed and the target's top now has a hole in it.
        assert!(
            u.face(crate::ids::FaceKey::new(OpId::new(2), FaceRole::StartCap))
                .is_none()
        );
    }

    #[test]
    fn coplanar_cut_from_top_face() {
        let a = cube(1, Vec3::ZERO, 2.0);
        let b = cube(2, Vec3::new(0.5, 0.5, 1.0), 1.0);
        // Tool's top cap is coplanar with the target's top face.
        let d = boolean(&a, &b, BoolOp::Subtract).unwrap();
        assert_relative_eq!(d.volume(), 7.0, epsilon = 1e-9);
        assert!(d.is_closed(), "{:?}", d.validate());
    }
}
