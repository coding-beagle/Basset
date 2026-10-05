//! A bounding volume hierarchy over triangles: the tracer's only acceleration structure.
//!
//! Built top down with the surface area heuristic over sixteen bins per axis, which is
//! the standard compromise — within a few per cent of a full sweep's traversal cost at a
//! fraction of the build time, and the build happens every time the in-canvas render
//! restarts on a changed model. Nodes are stored flat, the two children of a node side
//! by side, so a node holds one index and a traversal fetches siblings together.

use basset_math::Vec3;

/// A ray as the traversal wants it: the reciprocal of the direction precomputed, since
/// every node test divides by it.
#[derive(Clone, Copy, Debug)]
pub struct TraceRay {
    pub origin: Vec3,
    pub dir: Vec3,
    pub inv_dir: Vec3,
}

impl TraceRay {
    pub fn new(origin: Vec3, dir: Vec3) -> Self {
        Self {
            origin,
            dir,
            inv_dir: Vec3::ONE / dir,
        }
    }
}

/// One triangle, stored as a vertex and two edges for the Möller–Trumbore test.
#[derive(Clone, Copy, Debug)]
pub struct Triangle {
    pub v0: Vec3,
    pub e1: Vec3,
    pub e2: Vec3,
}

impl Triangle {
    pub fn new(a: Vec3, b: Vec3, c: Vec3) -> Self {
        Self {
            v0: a,
            e1: b - a,
            e2: c - a,
        }
    }

    fn bounds(&self) -> (Vec3, Vec3) {
        let (a, b, c) = (self.v0, self.v0 + self.e1, self.v0 + self.e2);
        (a.min(b).min(c), a.max(b).max(c))
    }

    /// Distance along the ray and barycentrics `(u, v)` of a hit in `(t_min, t_max)`.
    #[inline]
    pub fn intersect(&self, ray: &TraceRay, t_min: f64, t_max: f64) -> Option<(f64, f64, f64)> {
        let p = ray.dir.cross(self.e2);
        let det = self.e1.dot(p);
        if det.abs() < 1e-18 {
            return None;
        }
        let inv = 1.0 / det;
        let s = ray.origin - self.v0;
        let u = s.dot(p) * inv;
        if !(0.0..=1.0).contains(&u) {
            return None;
        }
        let q = s.cross(self.e1);
        let v = ray.dir.dot(q) * inv;
        if v < 0.0 || u + v > 1.0 {
            return None;
        }
        let t = self.e2.dot(q) * inv;
        (t > t_min && t < t_max).then_some((t, u, v))
    }
}

#[derive(Clone, Copy, Debug)]
struct Node {
    min: Vec3,
    max: Vec3,
    /// For a leaf, the first entry of `order`; for an interior node, the index of the
    /// left child (the right one follows it).
    offset: u32,
    /// Triangles in a leaf; zero marks an interior node.
    count: u32,
}

/// The hierarchy. It holds triangle *indices*; the triangles stay with the caller.
#[derive(Clone, Debug, Default)]
pub struct Bvh {
    nodes: Vec<Node>,
    order: Vec<u32>,
}

/// A hit: which triangle, how far, and where on it.
#[derive(Clone, Copy, Debug)]
pub struct Hit {
    pub t: f64,
    pub triangle: u32,
    pub u: f64,
    pub v: f64,
}

const BINS: usize = 16;
const LEAF_SIZE: usize = 4;

impl Bvh {
    pub fn build(triangles: &[Triangle]) -> Self {
        let mut bvh = Bvh {
            nodes: Vec::with_capacity(triangles.len() * 2),
            order: (0..triangles.len() as u32).collect(),
        };
        if triangles.is_empty() {
            return bvh;
        }
        let boxes: Vec<(Vec3, Vec3)> = triangles.iter().map(Triangle::bounds).collect();
        let centroids: Vec<Vec3> = boxes.iter().map(|(a, b)| (*a + *b) * 0.5).collect();
        // An explicit stack rather than recursion: a pathological model (thousands of
        // coincident slivers) can make the tree deep, and depth must not cost the stack.
        bvh.nodes.push(Node {
            min: Vec3::ZERO,
            max: Vec3::ZERO,
            offset: 0,
            count: 0,
        });
        let mut work = vec![(0usize, 0usize, triangles.len())];
        while let Some((node, start, end)) = work.pop() {
            let (min, max) = bvh.order[start..end]
                .iter()
                .map(|&i| boxes[i as usize])
                .fold((Vec3::INFINITY, Vec3::NEG_INFINITY), |(a, b), (c, d)| {
                    (a.min(c), b.max(d))
                });
            bvh.nodes[node].min = min;
            bvh.nodes[node].max = max;
            let count = end - start;
            let split = (count > LEAF_SIZE)
                .then(|| split(&mut bvh.order[start..end], &boxes, &centroids))
                .flatten();
            match split {
                Some(mid) => {
                    // Both children at once and side by side, so a node needs only the
                    // index of the first.
                    let left = bvh.nodes.len();
                    let empty = Node {
                        min: Vec3::ZERO,
                        max: Vec3::ZERO,
                        offset: 0,
                        count: 0,
                    };
                    bvh.nodes.extend([empty, empty]);
                    bvh.nodes[node].offset = left as u32;
                    work.push((left + 1, start + mid, end));
                    work.push((left, start, start + mid));
                }
                None => {
                    bvh.nodes[node].offset = start as u32;
                    bvh.nodes[node].count = count as u32;
                }
            }
        }
        bvh
    }

    pub fn is_empty(&self) -> bool {
        self.order.is_empty()
    }

    /// The nearest triangle the ray meets in `(t_min, t_max)`.
    pub fn closest(
        &self,
        triangles: &[Triangle],
        ray: &TraceRay,
        t_min: f64,
        mut t_max: f64,
    ) -> Option<Hit> {
        if self.is_empty() {
            return None;
        }
        let mut best = None;
        let mut stack = [0u32; 64];
        let mut top = 1;
        while top > 0 {
            top -= 1;
            let node = &self.nodes[stack[top] as usize];
            if !slab(node, ray, t_min, t_max) {
                continue;
            }
            if node.count > 0 {
                for &i in &self.order[node.offset as usize..(node.offset + node.count) as usize] {
                    if let Some((t, u, v)) = triangles[i as usize].intersect(ray, t_min, t_max) {
                        t_max = t;
                        best = Some(Hit {
                            t,
                            triangle: i,
                            u,
                            v,
                        });
                    }
                }
                continue;
            }
            let left = node.offset;
            let right = node.offset + 1;
            // Visit the nearer child first so the far one is usually culled by the hit.
            let (near, far) = if ray.dir.to_array()[axis_of(node)] < 0.0 {
                (right, left)
            } else {
                (left, right)
            };
            if top + 2 > stack.len() {
                // Deeper than any balanced tree over 2⁶³ triangles; only a degenerate
                // build gets here, and dropping the subtree is better than a panic.
                continue;
            }
            stack[top] = far;
            stack[top + 1] = near;
            top += 2;
        }
        best
    }
}

/// The axis the node is widest along: the one its children were most likely split on,
/// and so the one whose sign orders them front to back.
fn axis_of(node: &Node) -> usize {
    let e = node.max - node.min;
    if e.x >= e.y && e.x >= e.z {
        0
    } else if e.y >= e.z {
        1
    } else {
        2
    }
}

#[inline]
fn slab(node: &Node, ray: &TraceRay, t_min: f64, t_max: f64) -> bool {
    let t1 = (node.min - ray.origin) * ray.inv_dir;
    let t2 = (node.max - ray.origin) * ray.inv_dir;
    let lo = t1.min(t2).max_element().max(t_min);
    let hi = t1.max(t2).min_element().min(t_max);
    lo <= hi
}

fn area(min: Vec3, max: Vec3) -> f64 {
    let e = (max - min).max(Vec3::ZERO);
    2.0 * (e.x * e.y + e.y * e.z + e.z * e.x)
}

/// Partitions `order` by the cheapest binned SAH split and returns the size of the left
/// part, or `None` when no split beats a leaf.
fn split(order: &mut [u32], boxes: &[(Vec3, Vec3)], centroids: &[Vec3]) -> Option<usize> {
    let (cmin, cmax) = order
        .iter()
        .map(|&i| centroids[i as usize])
        .fold((Vec3::INFINITY, Vec3::NEG_INFINITY), |(a, b), c| {
            (a.min(c), b.max(c))
        });
    let extent = cmax - cmin;
    let mut best: Option<(f64, usize, usize)> = None;
    for axis in 0..3 {
        let span = extent.to_array()[axis];
        if span <= 0.0 {
            continue;
        }
        let bin_of = |i: u32| {
            let c = centroids[i as usize].to_array()[axis];
            (((c - cmin.to_array()[axis]) / span * BINS as f64) as usize).min(BINS - 1)
        };
        let mut counts = [0usize; BINS];
        let mut bounds = [(Vec3::INFINITY, Vec3::NEG_INFINITY); BINS];
        for &i in order.iter() {
            let b = bin_of(i);
            counts[b] += 1;
            let (lo, hi) = boxes[i as usize];
            bounds[b] = (bounds[b].0.min(lo), bounds[b].1.max(hi));
        }
        // Sweep from the right to know each split's right-hand area, then from the left.
        let mut right_area = [0.0; BINS];
        let mut acc = (Vec3::INFINITY, Vec3::NEG_INFINITY);
        let mut right_count = [0usize; BINS];
        let mut n = 0;
        for b in (1..BINS).rev() {
            acc = (acc.0.min(bounds[b].0), acc.1.max(bounds[b].1));
            n += counts[b];
            right_area[b] = if n > 0 { area(acc.0, acc.1) } else { 0.0 };
            right_count[b] = n;
        }
        let mut acc = (Vec3::INFINITY, Vec3::NEG_INFINITY);
        let mut n = 0;
        for b in 0..BINS - 1 {
            acc = (acc.0.min(bounds[b].0), acc.1.max(bounds[b].1));
            n += counts[b];
            let (nl, nr) = (n, right_count[b + 1]);
            if nl == 0 || nr == 0 {
                continue;
            }
            let cost = area(acc.0, acc.1) * nl as f64 + right_area[b + 1] * nr as f64;
            if best.is_none_or(|(c, _, _)| cost < c) {
                best = Some((cost, axis, b));
            }
        }
    }
    let (cost, axis, bin) = best?;
    let (lo, hi) = order
        .iter()
        .map(|&i| boxes[i as usize])
        .fold((Vec3::INFINITY, Vec3::NEG_INFINITY), |(a, b), (c, d)| {
            (a.min(c), b.max(d))
        });
    // A leaf costs one intersection per triangle over the node's own area; traversing a
    // split costs roughly one more box test. Stay a leaf when that is no saving.
    if cost >= area(lo, hi) * order.len() as f64 && order.len() <= 16 {
        return None;
    }
    let span = extent.to_array()[axis];
    let lo_c = cmin.to_array()[axis];
    let goes_left = |i: u32| {
        let c = centroids[i as usize].to_array()[axis];
        (((c - lo_c) / span * BINS as f64) as usize).min(BINS - 1) <= bin
    };
    let mut left = 0;
    for k in 0..order.len() {
        if goes_left(order[k]) {
            order.swap(k, left);
            left += 1;
        }
    }
    (left > 0 && left < order.len()).then_some(left)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sampling::Rng;

    fn random_triangles(n: usize, rng: &mut Rng) -> Vec<Triangle> {
        let mut r = || rng.next_f64() * 100.0 - 50.0;
        (0..n)
            .map(|_| {
                let a = Vec3::new(r(), r(), r());
                let jitter = |rng: &mut dyn FnMut() -> f64| Vec3::new(rng(), rng(), rng()) * 0.1;
                let b = a + jitter(&mut r);
                let c = a + jitter(&mut r);
                Triangle::new(a, b, c)
            })
            .collect()
    }

    fn brute(triangles: &[Triangle], ray: &TraceRay) -> Option<(u32, f64)> {
        let mut best: Option<(u32, f64)> = None;
        for (i, t) in triangles.iter().enumerate() {
            if let Some((d, _, _)) = t.intersect(ray, 1e-9, f64::INFINITY)
                && best.is_none_or(|(_, b)| d < b)
            {
                best = Some((i as u32, d));
            }
        }
        best
    }

    #[test]
    fn the_tree_finds_what_brute_force_finds() {
        let mut rng = Rng::new(42, 1);
        let triangles = random_triangles(3000, &mut rng);
        let bvh = Bvh::build(&triangles);
        let mut hits = 0;
        for _ in 0..2000 {
            let origin = Vec3::new(
                rng.next_f64() * 200.0 - 100.0,
                rng.next_f64() * 200.0 - 100.0,
                rng.next_f64() * 200.0 - 100.0,
            );
            let target = Vec3::new(
                rng.next_f64() * 60.0 - 30.0,
                rng.next_f64() * 60.0 - 30.0,
                rng.next_f64() * 60.0 - 30.0,
            );
            let ray = TraceRay::new(origin, (target - origin).normalize());
            let expected = brute(&triangles, &ray);
            let got = bvh.closest(&triangles, &ray, 1e-9, f64::INFINITY);
            match (expected, got) {
                (None, None) => {}
                (Some((_, d)), Some(h)) => {
                    hits += 1;
                    assert!((h.t - d).abs() < 1e-9);
                }
                other => panic!("tree and brute force disagree: {other:?}"),
            }
        }
        assert!(hits > 10, "the test should hit something");
    }

    #[test]
    fn every_triangle_is_in_exactly_one_leaf() {
        let mut rng = Rng::new(3, 9);
        let triangles = random_triangles(777, &mut rng);
        let bvh = Bvh::build(&triangles);
        let mut seen = vec![0; triangles.len()];
        for node in &bvh.nodes {
            if node.count > 0 {
                for &i in &bvh.order[node.offset as usize..(node.offset + node.count) as usize] {
                    seen[i as usize] += 1;
                }
            }
        }
        assert!(seen.iter().all(|&n| n == 1));
    }

    #[test]
    fn an_empty_tree_hits_nothing() {
        let bvh = Bvh::build(&[]);
        let ray = TraceRay::new(Vec3::ZERO, Vec3::X);
        assert!(bvh.closest(&[], &ray, 0.0, f64::INFINITY).is_none());
    }
}
