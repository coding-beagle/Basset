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
//! or a fillet tool swept round a rim — every candidate has the whole remainder behind
//! it, the tree degenerates into a list of depth `n`, and both the build and the clip
//! cost `O(n²)`. That is not a poor choice of plane but the shape: a convex solid *is*
//! the intersection of its face half-spaces, so a tree of face planes over it is a list
//! whichever order they are taken in. [`blend`](crate::blend) budgets its tools against
//! what that costs. The walks are iterative, so depth costs heap rather than stack, and a
//! polygon clear of the other solid's bounding box never enters its tree at all.
//!
//! Quadratic in the polygon count means the constant in front of it is what the user
//! feels, and it is a constant per *node visit*, not per polygon: on the list-shaped tree
//! a fillet produces, one boolean visits nodes tens of millions of times and splits a
//! polygon a few tens of thousands of times. So the visit is what the code is shaped
//! around. A polygon carries its bounding box, which settles which side of a plane it
//! falls almost every time without its vertices being read; a clip walks each polygon
//! down the tree on its own rather than carrying a list per node; a set that has run
//! empty is not routed into a child, which would visit every node below it; and the
//! polygon moved between lists is two corners and a pointer, with everything the visit
//! does not need behind that pointer. Together those took the seven cases of
//! `examples/fillet_bench` from 2.82 s to 0.46 s without changing a single output.
//!
//! The general fix is a plane that is not a face plane: an axis-aligned cut through the
//! middle of a convex sweep halves it, and the tree is `O(log n)` deep. The scheme can
//! carry such a node — it stores no polygons and only routes — but tried plainly it
//! costs more than it saves, because every polygon of *either* solid that crosses a cut
//! is fragmented along it whether or not the boolean touches it there: the rim fillet
//! above came out with three and a half times the polygons and a handful of slivers the
//! healer could not close. Making it pay wants the fragments of an untouched polygon
//! put back together on the way out, which is where that work stopped.
//!
//! Keeping every fragment under its own key is right for naming and wrong for what the
//! user sees, because two bodies joined flush leave the one flat face they now share as
//! two. [`Solid::merge_continuous_faces`] is the other half of the assembly: it puts the
//! faces different operations have grown into one patch of surface back together.

use basset_math::{Aabb, Vec3};

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
    let pool = &mut Pool::default();
    let mut na = Node::new(polygons_of(a, 0), pool);
    let mut nb = Node::new(polygons_of(b, 1), pool);
    let result = match op {
        BoolOp::Union => {
            na.clip_to(&nb, pool);
            nb.clip_to(&na, pool);
            nb.invert();
            nb.clip_to(&na, pool);
            nb.invert();
            na.build(nb.all_polygons(), pool);
            na.all_polygons()
        }
        BoolOp::Subtract => {
            na.invert();
            na.clip_to(&nb, pool);
            nb.clip_to(&na, pool);
            nb.invert();
            nb.clip_to(&na, pool);
            nb.invert();
            na.build(nb.all_polygons(), pool);
            na.invert();
            na.all_polygons()
        }
        BoolOp::Intersect => {
            na.invert();
            nb.clip_to(&na, pool);
            nb.invert();
            na.clip_to(&nb, pool);
            nb.clip_to(&na, pool);
            na.build(nb.all_polygons(), pool);
            na.invert();
            na.all_polygons()
        }
    };
    assemble(result, &sources)
}

/// Polygon tagged with where it came from so the result can be regrouped into faces.
///
/// Only the box is here; everything else is behind one pointer. A polygon is sorted
/// against a plane once per level of tree it passes through, and on a list-shaped tree
/// (see the module header) that is every level, so a boolean on a cylinder rim moves
/// polygons between lists tens of millions of times and reads the geometry of one a few
/// thousand times. What the sort needs is the box; splitting it off from what it does not
/// need halves the bytes those moves copy, and costs a pointer hop on the rare path.
#[derive(Clone, Debug)]
struct CsgPolygon {
    /// Box round the vertices, carried with the polygon rather than recomputed. It
    /// settles the whole-polygon verdict for almost every polygon and plane without the
    /// vertices being read at all, and it answers [`outside`], which used to walk them.
    lo: Vec3,
    hi: Vec3,
    inner: Box<PolygonData>,
}

#[derive(Clone, Debug)]
struct PolygonData {
    vertices: Vec<Vec3>,
    normal: Vec3,
    w: f64,
    /// (solid index, face index)
    source: (u32, u32),
    /// Odd number of inversions: the polygon now faces the other way.
    flipped: bool,
}

impl std::ops::Deref for CsgPolygon {
    type Target = PolygonData;

    fn deref(&self) -> &PolygonData {
        &self.inner
    }
}

impl CsgPolygon {
    fn new(vertices: Vec<Vec3>, normal: Vec3, w: f64, source: (u32, u32), flipped: bool) -> Self {
        let (mut lo, mut hi) = (Vec3::splat(f64::INFINITY), Vec3::splat(f64::NEG_INFINITY));
        for v in &vertices {
            lo = lo.min(*v);
            hi = hi.max(*v);
        }
        CsgPolygon {
            lo,
            hi,
            inner: Box::new(PolygonData {
                vertices,
                normal,
                w,
                source,
                flipped,
            }),
        }
    }

    fn flip(&mut self) {
        let inner = &mut *self.inner;
        inner.vertices.reverse();
        inner.normal = -inner.normal;
        inner.w = -inner.w;
        inner.flipped = !inner.flipped;
    }
}

fn polygons_of(s: &Solid, index: u32) -> Vec<CsgPolygon> {
    let mut out = Vec::with_capacity(s.polygon_count());
    for (fi, f) in s.faces.iter().enumerate() {
        for p in &f.polygons {
            out.push(CsgPolygon::new(
                p.vertices.clone(),
                p.plane.normal,
                p.plane.normal.dot(p.plane.origin),
                (index, fi as u32),
                false,
            ));
        }
    }
    out
}

fn assemble(polys: Vec<CsgPolygon>, sources: &[&Solid; 2]) -> Result<Solid, KernelError> {
    let mut faces: Vec<Face> = Vec::new();
    for p in polys {
        let p = *p.inner;
        let Some(polygon) = Polygon::new(p.vertices) else {
            continue;
        };
        let src = &sources[p.source.0 as usize].faces[p.source.1 as usize];
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

    /// Which side of the plane a box lies wholly on, or `None` when the plane cuts it.
    ///
    /// The extent of a box along a direction is the sum of its half-widths weighted by
    /// the direction's components, which is exact for an axis-aligned box and so never
    /// claims a side the vertices would not agree with. `None` is the answer whenever
    /// there is any doubt, including the coplanar case the caller has to resolve
    /// per-vertex anyway.
    fn classify_box(&self, lo: Vec3, hi: Vec3) -> Option<u8> {
        let centre = (lo + hi) * 0.5;
        let half = (hi - lo) * 0.5;
        let reach = self.normal.abs().dot(half) + EPSILON;
        let d = self.normal.dot(centre) - self.w;
        if d > reach {
            Some(FRONT)
        } else if d < -reach {
            Some(BACK)
        } else {
            None
        }
    }

    /// Where `polygon` belongs, whole: `None` where the plane cuts it.
    ///
    /// Takes the polygon by reference and returns a verdict rather than pushing, because
    /// what a caller does with each outcome differs and the three whole-polygon ones are
    /// a move of the polygon it already holds.
    fn side_of(&self, polygon: &CsgPolygon) -> Option<u8> {
        // The box first: a polygon whose box clears the plane is on one side of it, and
        // answering that costs one dot product against numbers sitting in the cache line
        // the work list just moved. Only a polygon the box cannot settle is read.
        if let Some(side) = self.classify_box(polygon.lo, polygon.hi) {
            return Some(side);
        }
        // The whole-polygon verdict next, and the per-vertex verdicts only for the one
        // outcome that needs them: nearly every polygon at nearly every node lands whole
        // on one side, and keeping a vector of verdicts for those was one heap
        // allocation per polygon per node.
        let mut polygon_type = 0u8;
        for v in &polygon.vertices {
            polygon_type |= self.classify(*v);
        }
        match polygon_type {
            SPANNING => None,
            t => Some(t),
        }
    }

    /// Cuts a straddling polygon in two, as `(front piece, back piece)`. Either is `None`
    /// where it collapsed to nothing, and the back piece is not built at all when
    /// `keep_back` is false: at a node with nothing behind it, behind is inside the solid
    /// and thrown away.
    fn split_spanning(
        &self,
        polygon: &CsgPolygon,
        keep_back: bool,
    ) -> (Option<CsgPolygon>, Option<CsgPolygon>) {
        let n = polygon.vertices.len();
        let mut f = Vec::with_capacity(n + 1);
        let mut b = Vec::with_capacity(if keep_back { n + 1 } else { 0 });
        // The verdicts come one edge at a time, each carried over into the next: the loop
        // wants the pair, and collecting them all up front cost a heap allocation per
        // polygon that straddles anything.
        let mut ti = self.classify(polygon.vertices[0]);
        for i in 0..n {
            let j = if i + 1 == n { 0 } else { i + 1 };
            let (vi, vj) = (polygon.vertices[i], polygon.vertices[j]);
            let tj = self.classify(vj);
            if ti != BACK {
                f.push(vi);
            }
            if keep_back && ti != FRONT {
                b.push(vi);
            }
            if (ti | tj) == SPANNING {
                let t = (self.w - self.normal.dot(vi)) / self.normal.dot(vj - vi);
                let v = vi.lerp(vj, t);
                f.push(v);
                if keep_back {
                    b.push(v);
                }
            }
            ti = tj;
        }
        (
            fragment(polygon, f),
            keep_back.then(|| fragment(polygon, b)).flatten(),
        )
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
    Some(CsgPolygon::new(
        vertices,
        parent.normal,
        parent.w,
        parent.source,
        parent.flipped,
    ))
}

struct Node {
    plane: Option<SplitPlane>,
    front: Option<Box<Node>>,
    back: Option<Box<Node>>,
    polygons: Vec<CsgPolygon>,
    /// Box round every polygon this tree was built from, kept at the root only.
    ///
    /// The tree partitions space by its planes, and the planes are infinite, so a
    /// polygon nowhere near the solid still walks the whole tree to learn that it is
    /// outside — on a list-shaped tree, every node. The box answers that without the
    /// walk: outside the box is outside the solid, or, once the tree has been inverted,
    /// inside it. Only the root is ever clipped against, so only the root carries it.
    bounds: Aabb,
    /// Odd number of inversions: the tree now describes the complement of the solid,
    /// and the box bounds what is *not* in it.
    inverted: bool,
}

/// Slack round the bounds, so a polygon the box says is clear of the solid is also
/// clear of the splitter's own coplanarity tolerance at every face.
const BOUNDS_PAD: f64 = 2.0 * EPSILON;

fn outside(bounds: &Aabb, polygon: &CsgPolygon) -> bool {
    (0..3).any(|i| {
        polygon.hi[i] < bounds.min[i] - BOUNDS_PAD || polygon.lo[i] > bounds.max[i] + BOUNDS_PAD
    })
}

/// Polygon lists on loan to the build.
///
/// A build sorts a node's set into a list per side, and a set near the root of a
/// list-shaped tree is nearly as long as the whole solid, so each of those lists is large
/// and short-lived and the next node wants two more the same size. Asked of the allocator
/// fresh, a block that size is mapped rather than taken from a free list, and then every
/// page of it is faulted in the first time the sort writes to it. Passing the finished
/// ones back here and handing them out again keeps the pages: worth 8% of the whole blend
/// benchmark.
#[derive(Default)]
struct Pool {
    free: Vec<Vec<CsgPolygon>>,
}

impl Pool {
    fn take(&mut self, capacity: usize) -> Vec<CsgPolygon> {
        let mut v = self.free.pop().unwrap_or_default();
        v.reserve(capacity);
        v
    }

    /// Hands a list back, dropping whatever is still in it: a list of polygons the walk
    /// has decided against is exactly what a node returns here.
    fn give(&mut self, mut v: Vec<CsgPolygon>) {
        v.clear();
        self.free.push(v);
    }
}

/// Every polygon of `polygons` sorted against `plane`, for a build: the two sides come
/// back and anything coplanar with the plane is added straight to `coplanar`, which is
/// the node's own set and where both coplanar orientations were going anyway. Leaves
/// `polygons` empty, with its allocation intact for the pool.
fn build_partition(
    plane: &SplitPlane,
    polygons: &mut Vec<CsgPolygon>,
    coplanar: &mut Vec<CsgPolygon>,
    pool: &mut Pool,
) -> (Vec<CsgPolygon>, Vec<CsgPolygon>) {
    let n = polygons.len();
    // Sized up front: a node's two sides together hold everything that arrived, and
    // growing them from nothing copied the polygons again at every doubling.
    let mut front = pool.take(n);
    let mut back = pool.take(n);
    for p in polygons.drain(..) {
        match plane.side_of(&p) {
            Some(FRONT) => front.push(p),
            Some(BACK) => back.push(p),
            Some(_) => coplanar.push(p),
            None => {
                let (f, b) = plane.split_spanning(&p, true);
                front.extend(f);
                back.extend(b);
            }
        }
    }
    (front, back)
}

// None of the tree walks recurse. A tree built from a convex sweep is a list one node
// deep per facet (see the module header), and a recursive walk of it is one stack frame
// per facet: on a 2 MiB thread stack that was an abort somewhere between 4.5k and 4.8k
// polygons, and it was the hard ceiling the blend budgets were set under. Each walk
// carries its own work list on the heap instead, so depth costs memory it would have
// cost anyway and nothing else.
impl Node {
    fn empty() -> Box<Node> {
        Box::new(Node {
            plane: None,
            front: None,
            back: None,
            polygons: Vec::new(),
            bounds: Aabb::empty(),
            inverted: false,
        })
    }

    fn new(polygons: Vec<CsgPolygon>, pool: &mut Pool) -> Self {
        let mut n = *Node::empty();
        n.build(polygons, pool);
        n
    }

    fn invert(&mut self) {
        self.inverted = !self.inverted;
        let mut stack: Vec<&mut Node> = vec![self];
        while let Some(node) = stack.pop() {
            for p in &mut node.polygons {
                p.flip();
            }
            if let Some(p) = &mut node.plane {
                p.flip();
            }
            std::mem::swap(&mut node.front, &mut node.back);
            stack.extend(node.front.as_deref_mut());
            stack.extend(node.back.as_deref_mut());
        }
    }

    /// Removes the parts of `polygons` inside this tree.
    fn clip_polygons(&self, polygons: Vec<CsgPolygon>, pool: &mut Pool) -> Vec<CsgPolygon> {
        if polygons.is_empty() {
            return polygons;
        }
        let mut out = pool.take(polygons.len());
        // Settle everything the bounds can settle before the walk; see `bounds`. Kept
        // whole rather than walked, which is also fewer fragments in the result.
        let mut work: Vec<(&Node, CsgPolygon)> = Vec::new();
        let mut polygons = polygons;
        for p in polygons.drain(..) {
            match (outside(&self.bounds, &p), self.inverted) {
                (true, false) => out.push(p),
                (true, true) => {}
                (false, _) => work.push((self, p)),
            }
        }
        pool.give(polygons);
        // One polygon at a time, walked down as far as it goes.
        //
        // The natural shape here is a set per node, sorted into a set per child, and it
        // is what the build below has to do because choosing a plane needs the whole set.
        // A clip does not: nothing at a node depends on what else arrived there. Carrying
        // sets meant two lists and their copies at every node the walk touched — and it
        // touches one per polygon per level, which on a list-shaped tree (see the module
        // header) is millions, averaging under three polygons each. Descending with the
        // polygon instead is a pointer hop per level and allocates nothing; only a
        // polygon the plane actually cuts leaves a piece behind on the work list.
        while let Some((mut node, mut p)) = work.pop() {
            loop {
                let Some(plane) = node.plane else {
                    out.push(p);
                    break;
                };
                let side = match plane.side_of(&p) {
                    Some(COPLANAR) => {
                        // Coplanar goes with the side its own normal agrees with.
                        if plane.normal.dot(p.normal) > 0.0 {
                            FRONT
                        } else {
                            BACK
                        }
                    }
                    Some(side) => side,
                    None => {
                        let (f, b) = plane.split_spanning(&p, node.back.is_some());
                        if let Some(b) = b
                            && let Some(back) = &node.back
                        {
                            work.push((back, b));
                        }
                        match f {
                            Some(f) => {
                                p = f;
                                FRONT
                            }
                            None => break,
                        }
                    }
                };
                if side == FRONT {
                    match &node.front {
                        Some(f) => node = f,
                        None => {
                            out.push(p);
                            break;
                        }
                    }
                } else {
                    // Behind a face with nothing behind it is inside the solid: dropped.
                    match &node.back {
                        Some(b) => node = b,
                        None => break,
                    }
                }
            }
        }
        out
    }

    fn clip_to(&mut self, other: &Node, pool: &mut Pool) {
        let mut stack: Vec<&mut Node> = vec![self];
        while let Some(node) = stack.pop() {
            node.polygons = other.clip_polygons(std::mem::take(&mut node.polygons), pool);
            stack.extend(node.front.as_deref_mut());
            stack.extend(node.back.as_deref_mut());
        }
    }

    fn all_polygons(&self) -> Vec<CsgPolygon> {
        let mut out = Vec::new();
        let mut stack: Vec<&Node> = vec![self];
        while let Some(node) = stack.pop() {
            out.extend(node.polygons.iter().cloned());
            stack.extend(node.front.as_deref());
            stack.extend(node.back.as_deref());
        }
        out
    }

    fn build(&mut self, polygons: Vec<CsgPolygon>, pool: &mut Pool) {
        // Whichever solid the polygons describe, the tree now bounds it too — and if the
        // tree is inverted, the complement of it, which the box means then.
        for p in &polygons {
            self.bounds.include(p.lo);
            self.bounds.include(p.hi);
        }
        let mut stack: Vec<(&mut Node, Vec<CsgPolygon>)> = vec![(self, polygons)];
        while let Some((node, mut polygons)) = stack.pop() {
            if polygons.is_empty() {
                pool.give(polygons);
                continue;
            }
            // Whether the plane comes from this very set of polygons, which is what the
            // progress check below relies on; a node that already had a plane got it from
            // a different set and a one-sided split there is ordinary.
            let mut chosen_here = false;
            let plane = *node.plane.get_or_insert_with(|| {
                chosen_here = true;
                choose_plane(&polygons)
            });
            let kept = node.polygons.len();
            // `polygons` is consumed here, so nothing but `front` and `back` is alive
            // while the children wait on the work list. Measured on one rim of a
            // cylinder, 1700 tool facets peaked at 252 MB when the recursive walk kept
            // one nearly-full set alive per level, and 19 MB once it did not.
            let (mut front, mut back) =
                build_partition(&plane, &mut polygons, &mut node.polygons, pool);
            pool.give(polygons);
            // The recursion shrinks only because a plane taken from these polygons
            // consumes at least the polygon it came from. A polygon whose vertices stray
            // further from its own plane than EPSILON — healing inserts a T-junction
            // vertex within MERGE_TOL of an edge without projecting it onto the face, and
            // `Polygon::new` then centres the plane on a centroid that the stray vertex
            // has pulled off it — fails that test against its own plane and lands whole
            // on one side. The child would get an identical set, deterministically choose
            // the same plane, and recurse until the stack ran out. Keep the set here
            // instead: it is within a hair of the plane, so coplanar is also the right
            // answer geometrically.
            let made_progress =
                node.polygons.len() > kept || (!front.is_empty() && !back.is_empty());
            if chosen_here && !made_progress {
                node.polygons.append(&mut front);
                node.polygons.append(&mut back);
                pool.give(front);
                pool.give(back);
                continue;
            }
            if front.is_empty() {
                pool.give(front);
            } else {
                stack.push((node.front.get_or_insert_with(Node::empty), front));
            }
            if back.is_empty() {
                pool.give(back);
            } else {
                stack.push((node.back.get_or_insert_with(Node::empty), back));
            }
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
            // `side_of`, not a vertex loop: the verdicts it gives are the same ones, and
            // it settles nearly every polygon from the box alone.
            match plane.side_of(p) {
                Some(FRONT) => f += 1,
                Some(BACK) => b += 1,
                None => s += 1,
                Some(_) => {}
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
        let poly = CsgPolygon::new(
            vec![v(-0.5 * MERGE_TOL, 0.0), v(1.0, 0.0), v(1.0, 1.0)],
            Vec3::Z,
            0.0,
            (0, 0),
            false,
        );
        let plane = SplitPlane {
            normal: Vec3::X,
            w: 0.0,
        };
        assert_eq!(
            plane.side_of(&poly),
            Some(FRONT),
            "whole and in front: no sliver behind the plane"
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
        let poly = CsgPolygon::new(
            vec![
                Vec3::new(0.0, 8.0, 0.0),
                Vec3::new(1.0, 8.0 + off, 0.0),
                Vec3::new(1.0, 8.0 + off, 1.0),
                Vec3::new(0.0, 8.0, 1.0),
            ],
            normal,
            w,
            (0, 0),
            false,
        );
        // The premise: against its own plane the polygon is behind, never coplanar, so
        // the split consumes nothing.
        let plane = SplitPlane { normal, w };
        let types: Vec<u8> = poly.vertices.iter().map(|v| plane.classify(*v)).collect();
        assert!(types.contains(&BACK), "{types:?}");
        assert!(
            !types.contains(&FRONT),
            "must not span, or it would be split"
        );

        let node = Node::new(vec![poly], &mut Pool::default());
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
