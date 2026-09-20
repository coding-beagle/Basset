//! Profile extraction: closed regions bounded by non-construction curves.
//!
//! The algorithm is the classic planar face walk:
//! 1. Flatten every line, arc and circle to a polyline.
//! 2. Split those polylines wherever two of them cross, and, where two run along each
//!    other, at the ends of the stretch they share, so the curves form a proper planar
//!    subdivision and *every* enclosed region is a face, not just the ones the user
//!    happened to draw with matching endpoints.
//! 3. Merge fragment endpoints within [`JOIN_TOL`] into graph nodes.
//! 4. Drop edges that repeat geometry another edge already carries. Geometry drawn twice
//!    is common in a real drawing, and two edges of identical shape between the same two
//!    nodes leave those nodes at the same angle, which the face walk in step 6 cannot
//!    order: it steps between the copies and traces a sliver instead of the region.
//! 5. Every fragment becomes two opposite half-edges, each tagged with the tangent
//!    direction it leaves its node in (arcs use their true tangent, which is what makes
//!    ordering correct where a line meets an arc).
//! 6. Faces are traced by always taking the next outgoing edge in the smallest
//!    clockwise turn from the reversed incoming edge. This walks every bounded face
//!    counter-clockwise; the unbounded face comes out clockwise and is dropped.
//! 7. Uncrossed circles and text glyph outlines are loops of their own.
//! 8. Loops are nested by containment: every loop is a region in its own right, and any
//!    loop directly inside it (from another connected component, with nothing between)
//!    is punched out of it as a hole. A circle drawn inside a rectangle therefore yields
//!    two regions — the rectangle-with-a-hole and the disc — which is what lets the user
//!    extrude either one, or both together for the filled rectangle.
//!
//! Splitting happens on the *tessellated* curves, so a crossing point sits on the chord
//! rather than the true arc. Both curves are cut at the identical point, so the graph is
//! still watertight; the region boundary is simply as accurate as the tessellation, which
//! is what the kernel consumes anyway.
//!
//! Every fragment keeps its source curve's id, so two fragments of one curve bounding the
//! same region produce one kernel face in two pieces rather than two faces. That is rare
//! (it needs a curve to leave and re-enter the same region's boundary) and costs only the
//! ability to fillet the two stretches separately.

use basset_math::Vec2;

use crate::contour::{Contour, Profile, Segment, SegmentKind, signed_area};
use crate::sketch::JOIN_TOL;
use crate::tessellation::{Tessellation, circle_polyline};
use crate::{Entity, EntityId, Sketch};

struct Loop {
    contour: Contour,
    /// Loops from the same connected component never nest, so nesting tests skip them.
    component: usize,
    area: f64,
    /// The text entity this outline came from. Glyphs nest by even-odd within one string:
    /// the counter of an "O" is a hole, never a region of its own.
    text: Option<EntityId>,
}

/// One curve flattened for crossing detection and tracing. A position along it is
/// `segment index + fraction`, which is all the splitting code needs to know about shape.
struct Flat {
    curve: EntityId,
    kind: SegmentKind,
    points: Vec<Vec2>,
    /// Circles carry their first point again at the end, so the wrap-around is an
    /// ordinary interior stretch once the ring has been cut somewhere.
    closed: bool,
    min: Vec2,
    max: Vec2,
}

/// A crossing recorded on one curve: where along it, and the exact point. Storing the
/// point (rather than recomputing it from the parameter on each curve) guarantees both
/// curves are cut at the same coordinates and therefore share a graph node.
#[derive(Clone, Copy)]
struct Split {
    at: f64,
    point: Vec2,
}

struct Edge {
    curve: EntityId,
    nodes: [usize; 2],
    /// Tangent direction leaving each node along this edge.
    out_angle: [f64; 2],
    polyline: Vec<Vec2>,
    kind: SegmentKind,
}

/// A half-edge is `(edge index, direction)`, direction 0 = nodes[0] → nodes[1].
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
struct HalfEdge {
    edge: usize,
    dir: usize,
}

pub(crate) fn profiles(sketch: &Sketch, tess: &Tessellation) -> Vec<Profile> {
    let mut loops: Vec<Loop> = Vec::new();
    let mut next_component = 0;

    // --- Graph of curve fragments -----------------------------------------------------
    let flats = flatten(sketch, tess);
    let splits = crossings(&flats);
    let mut nodes: Vec<Vec2> = Vec::new();
    let mut edges: Vec<Edge> = Vec::new();
    for (flat, splits) in flats.iter().zip(&splits) {
        let fragments = fragments(flat, splits);
        if fragments.is_empty() {
            // An uncrossed circle bounds a region all by itself and never enters the graph.
            if flat.closed {
                loops.push(closed_curve_loop(flat, &mut next_component));
            }
            continue;
        }
        for polyline in fragments {
            let a = node_of(&mut nodes, polyline[0]);
            let b = node_of(&mut nodes, *polyline.last().unwrap());
            if a == b {
                continue;
            }
            edges.push(Edge {
                curve: flat.curve,
                nodes: [a, b],
                out_angle: out_angles(&polyline, flat.kind),
                polyline,
                kind: flat.kind,
            });
        }
    }

    let edges = dedup(edges);

    // Outgoing half-edges per node, sorted counter-clockwise by departure angle.
    let mut outgoing: Vec<Vec<(f64, HalfEdge)>> = vec![Vec::new(); nodes.len()];
    for (i, e) in edges.iter().enumerate() {
        outgoing[e.nodes[0]].push((e.out_angle[0], HalfEdge { edge: i, dir: 0 }));
        outgoing[e.nodes[1]].push((e.out_angle[1], HalfEdge { edge: i, dir: 1 }));
    }
    for list in &mut outgoing {
        list.sort_by(|a, b| a.0.total_cmp(&b.0));
    }
    let component_of = components(nodes.len(), &edges);

    // Circle loops were numbered 0..next_component; graph components continue after them.
    let graph_base = next_component;
    next_component += nodes.len();

    let mut visited = vec![[false; 2]; edges.len()];
    for start_edge in 0..edges.len() {
        for start_dir in 0..2 {
            if visited[start_edge][start_dir] {
                continue;
            }
            let start = HalfEdge {
                edge: start_edge,
                dir: start_dir,
            };
            let mut face = Vec::new();
            let mut h = start;
            loop {
                visited[h.edge][h.dir] = true;
                face.push(h);
                let e = &edges[h.edge];
                let arrive = e.nodes[1 - h.dir];
                let twin = HalfEdge {
                    edge: h.edge,
                    dir: 1 - h.dir,
                };
                let list = &outgoing[arrive];
                let twin_index = list.iter().position(|(_, he)| *he == twin).unwrap_or(0);
                // Smallest clockwise turn from the reversed incoming edge = the previous
                // entry in the CCW-sorted list.
                h = list[(twin_index + list.len() - 1) % list.len()].1;
                if h == start || face.len() > 2 * edges.len() {
                    break;
                }
            }
            let (points, segments) = face_polyline(&edges, &nodes, &face);
            let area = signed_area(&points);
            if area > 1e-12 {
                let component = graph_base + component_of[edges[start_edge].nodes[0]];
                loops.push(Loop {
                    contour: Contour {
                        points,
                        segments,
                        closed: true,
                    },
                    component,
                    area,
                    text: None,
                });
            }
        }
    }

    // --- Text ---------------------------------------------------------------------------
    if let Some(font) = sketch.font() {
        for (id, data) in sketch.entities() {
            if data.construction {
                continue;
            }
            if let Entity::Text {
                anchor,
                ref text,
                height,
                angle,
            } = data.entity
                && let Some(origin) = sketch.point_pos(anchor)
            {
                for points in font.text_outlines(text, height, angle, origin, tess) {
                    let mut area = signed_area(&points);
                    if area.abs() < 1e-12 {
                        continue;
                    }
                    let mut points = points;
                    if area < 0.0 {
                        points.reverse();
                        area = -area;
                    }
                    let segments = points
                        .iter()
                        .map(|_| Segment {
                            curve: id,
                            kind: SegmentKind::Line,
                        })
                        .collect();
                    loops.push(Loop {
                        contour: Contour {
                            points,
                            segments,
                            closed: true,
                        },
                        component: next_component,
                        area,
                        text: Some(id),
                    });
                    next_component += 1;
                }
            }
        }
    }

    nest(loops)
}

/// Flattens every non-construction line, arc and circle. Curves that cannot be evaluated
/// (a deleted endpoint, a degenerate arc) are simply absent from the graph.
fn flatten(sketch: &Sketch, tess: &Tessellation) -> Vec<Flat> {
    let mut out = Vec::new();
    for (id, data) in sketch.entities() {
        if data.construction {
            continue;
        }
        let (points, kind, closed) = match data.entity {
            Entity::Line { .. } | Entity::Arc { .. } => {
                let Some((points, kind)) = sketch.tessellate_open_curve(id, tess) else {
                    continue;
                };
                (points, kind, false)
            }
            Entity::Circle { center, radius } => {
                let Some(c) = sketch.point_pos(center) else {
                    continue;
                };
                let mut points = circle_polyline(c, radius, tess);
                points.push(points[0]);
                (
                    points,
                    SegmentKind::Arc {
                        center: c,
                        radius,
                        ccw: true,
                    },
                    true,
                )
            }
            _ => continue,
        };
        if points.len() < 2 {
            continue;
        }
        let min = points.iter().copied().reduce(Vec2::min).unwrap_or_default();
        let max = points.iter().copied().reduce(Vec2::max).unwrap_or_default();
        out.push(Flat {
            curve: id,
            kind,
            points,
            closed,
            min,
            max,
        });
    }
    out
}

/// Every point where two curves cross, recorded against both of them. Crossings at a
/// curve's own endpoint are dropped for that curve: merging nodes already joins it there,
/// and a zero-length fragment would only confuse the walk.
fn crossings(flats: &[Flat]) -> Vec<Vec<Split>> {
    let mut splits: Vec<Vec<Split>> = vec![Vec::new(); flats.len()];
    for i in 0..flats.len() {
        for j in (i + 1)..flats.len() {
            let (a, b) = (&flats[i], &flats[j]);
            // Curves whose bounding boxes miss each other cannot cross; this is what keeps
            // the pairwise scan affordable on sketches with many curves.
            if a.min.x > b.max.x || b.min.x > a.max.x || a.min.y > b.max.y || b.min.y > a.max.y {
                continue;
            }
            for (si, wa) in a.points.windows(2).enumerate() {
                for (sj, wb) in b.points.windows(2).enumerate() {
                    // Two curves lying on top of each other are cut at the ends of the
                    // stretch they share, so the shared stretch is one fragment on each of
                    // them rather than two fragments of unrelated extent; `dedup` then
                    // keeps a single edge for it.
                    if let Some(ends) = segment_overlap(wa[0], wa[1], wb[0], wb[1]) {
                        for (ta, tb, point) in ends {
                            splits[i].push(Split {
                                at: si as f64 + ta,
                                point,
                            });
                            splits[j].push(Split {
                                at: sj as f64 + tb,
                                point,
                            });
                        }
                        continue;
                    }
                    let Some((ta, tb)) = segment_crossing(wa[0], wa[1], wb[0], wb[1]) else {
                        continue;
                    };
                    let point = wa[0].lerp(wa[1], ta);
                    splits[i].push(Split {
                        at: si as f64 + ta,
                        point,
                    });
                    splits[j].push(Split {
                        at: sj as f64 + tb,
                        point,
                    });
                }
            }
        }
    }
    for (flat, list) in flats.iter().zip(&mut splits) {
        list.sort_by(|a, b| a.at.total_cmp(&b.at));
        let ends = [flat.points[0], *flat.points.last().unwrap()];
        let mut kept: Vec<Split> = Vec::new();
        for s in list.iter() {
            let coincides = |p: &Vec2| p.distance(s.point) <= JOIN_TOL;
            // A closed curve's seam is an ordinary interior point, so only open curves
            // discard crossings at their ends.
            if !flat.closed && ends.iter().any(coincides) {
                continue;
            }
            if kept.last().is_some_and(|k| coincides(&k.point)) {
                continue;
            }
            kept.push(*s);
        }
        *list = kept;
    }
    splits
}

/// Parameters `(ta, tb)` in `[0, 1]` where the two segments meet, or `None` if they are
/// parallel or miss. Parallel segments are [`segment_overlap`]'s business: there is no
/// single crossing point to report, only a shared stretch.
fn segment_crossing(a0: Vec2, a1: Vec2, b0: Vec2, b1: Vec2) -> Option<(f64, f64)> {
    let r = a1 - a0;
    let s = b1 - b0;
    let denom = r.perp_dot(s);
    // Scale the parallel test by the segment lengths so it means "the angle between them
    // is tiny", not "the segments are short".
    if denom.abs() <= 1e-12 * r.length() * s.length() {
        return None;
    }
    let d = b0 - a0;
    let ta = d.perp_dot(s) / denom;
    let tb = d.perp_dot(r) / denom;
    ((0.0..=1.0).contains(&ta) && (0.0..=1.0).contains(&tb)).then_some((ta, tb))
}

/// The two ends of the stretch two collinear segments share, as `(parameter on a,
/// parameter on b, the point)`, or `None` when they are not collinear or share at most a
/// point.
///
/// Overlapping geometry is ordinary in a real drawing: an edge traced a second time, a
/// rectangle drawn over the outline of another. The graph has to cut both curves at the
/// ends of the shared stretch, because only then is the overlap a fragment with the same
/// endpoints on both curves — a duplicate that [`dedup`] can drop. Left uncut, the two
/// curves contribute edges of different extent and the region they bound is never traced.
///
/// Each end is an endpoint of one of the two segments, and that exact point is reported
/// rather than one reconstructed from a parameter, so both curves are cut at coordinates
/// that merge into one node.
fn segment_overlap(a0: Vec2, a1: Vec2, b0: Vec2, b1: Vec2) -> Option<[(f64, f64, Vec2); 2]> {
    let r = a1 - a0;
    let s = b1 - b0;
    let (la, ls) = (r.length(), s.length());
    if la <= JOIN_TOL || ls <= JOIN_TOL {
        return None;
    }
    if r.perp_dot(s).abs() > 1e-12 * la * ls {
        return None;
    }
    // Parallel is not enough: the segments must lie on the same line, within the tolerance
    // the graph merges nodes at.
    if (r / la).perp_dot(b0 - a0).abs() > JOIN_TOL {
        return None;
    }
    // Both segments as intervals in a's parameter. The middle two of the four sorted ends
    // bound the intersection of the intervals, and each of them is a real endpoint.
    let param_a = |p: Vec2| (p - a0).dot(r) / (la * la);
    let mut ends = [(0.0, a0), (1.0, a1), (param_a(b0), b0), (param_a(b1), b1)];
    ends.sort_by(|x, y| x.0.total_cmp(&y.0));
    let (lo, hi) = (ends[1], ends[2]);
    if (hi.0 - lo.0) * la <= JOIN_TOL {
        return None;
    }
    let param_b = |p: Vec2| ((p - b0).dot(s) / (ls * ls)).clamp(0.0, 1.0);
    Some([
        (lo.0.clamp(0.0, 1.0), param_b(lo.1), lo.1),
        (hi.0.clamp(0.0, 1.0), param_b(hi.1), hi.1),
    ])
}

/// Cuts a flattened curve at its crossings. Returns nothing when the curve is not cut into
/// usable pieces, which for a circle means it still bounds a region on its own.
fn fragments(flat: &Flat, splits: &[Split]) -> Vec<Vec<Vec2>> {
    let end = (flat.points.len() - 1) as f64;
    let mut out = Vec::new();
    if flat.closed {
        // One cut leaves a closed loop, not an arc between two nodes, so leave the ring
        // whole and let it stand as its own loop.
        if splits.len() < 2 {
            return out;
        }
        for w in splits.windows(2) {
            out.push(cut(flat, w[0].at, w[1].at, w[0].point, w[1].point));
        }
        // The last piece runs through the seam, so it is two stretches joined.
        let (last, first) = (splits[splits.len() - 1], splits[0]);
        let mut wrap = cut(flat, last.at, end, last.point, flat.points[end as usize]);
        let tail = cut(flat, 0.0, first.at, flat.points[0], first.point);
        wrap.pop();
        wrap.extend(tail);
        out.push(wrap);
    } else {
        if splits.is_empty() {
            out.push(flat.points.clone());
            return out;
        }
        let mut from = (0.0, flat.points[0]);
        for s in splits {
            out.push(cut(flat, from.0, s.at, from.1, s.point));
            from = (s.at, s.point);
        }
        out.push(cut(flat, from.0, end, from.1, *flat.points.last().unwrap()));
    }
    out.retain(|f| f.len() >= 2 && f[0].distance(*f.last().unwrap()) > JOIN_TOL);
    out
}

/// The stretch of `flat` between two positions, beginning and ending at the exact crossing
/// points so that the curves cut there share a vertex to the last bit.
fn cut(flat: &Flat, from: f64, to: f64, start: Vec2, end: Vec2) -> Vec<Vec2> {
    let mut points = vec![start];
    let first_interior = from.floor() as usize + 1;
    let last_interior = to.ceil() as usize;
    for p in flat
        .points
        .iter()
        .take(last_interior.min(flat.points.len()))
        .skip(first_interior)
    {
        if p.distance(start) > JOIN_TOL && p.distance(end) > JOIN_TOL {
            points.push(*p);
        }
    }
    points.push(end);
    points
}

/// Tangent direction leaving each end of a fragment, which is what orders the half-edges
/// around a node. Arcs use their analytic tangent so a line meeting an arc tangentially
/// still sorts correctly.
fn out_angles(polyline: &[Vec2], kind: SegmentKind) -> [f64; 2] {
    let start = polyline[0];
    let end = *polyline.last().unwrap();
    match kind {
        SegmentKind::Line => [(end - start).to_angle(), (start - end).to_angle()],
        // Fragments inherit their arc's counter-clockwise direction: CCW tangent at the
        // start, CW tangent at the end.
        SegmentKind::Arc { center, .. } => [
            (start - center).perp().to_angle(),
            (-(end - center).perp()).to_angle(),
        ],
    }
}

fn closed_curve_loop(flat: &Flat, next_component: &mut usize) -> Loop {
    // The repeated seam point would be a zero-length edge in a closed contour.
    let points = flat.points[..flat.points.len() - 1].to_vec();
    let segments = points
        .iter()
        .map(|_| Segment {
            curve: flat.curve,
            kind: flat.kind,
        })
        .collect();
    let contour = Contour {
        points,
        segments,
        closed: true,
    };
    let area = contour.signed_area();
    let component = *next_component;
    *next_component += 1;
    Loop {
        contour,
        component,
        area,
        text: None,
    }
}

/// Drops edges that repeat geometry another edge already carries.
///
/// Two edges between the same nodes with the same shape leave their nodes at the same
/// angle, so the walk cannot tell which of them continues a face and steps between the
/// copies instead, tracing a zero-area sliver and marking the region's real half-edges
/// visited on the way. Keeping one edge per distinct stretch of geometry is what makes a
/// drawing whose outline was traced twice still enclose its regions. Which copy survives
/// decides the curve the kernel names the resulting face after, and between identical
/// curves that choice is arbitrary.
fn dedup(edges: Vec<Edge>) -> Vec<Edge> {
    let mut kept: Vec<(Vec2, Edge)> = Vec::with_capacity(edges.len());
    for edge in edges {
        let mid = polyline_midpoint(&edge.polyline);
        let same = |(m, k): &(Vec2, Edge)| {
            (k.nodes == edge.nodes || k.nodes == [edge.nodes[1], edge.nodes[0]])
                && m.distance(mid) <= JOIN_TOL
        };
        if !kept.iter().any(same) {
            kept.push((mid, edge));
        }
    }
    kept.into_iter().map(|(_, e)| e).collect()
}

/// The point half way along a polyline by arc length. Together with the endpoints this
/// identifies the stretch: it is what tells the two arcs that share a chord apart, which
/// centre and radius alone do not.
fn polyline_midpoint(points: &[Vec2]) -> Vec2 {
    let half = points.windows(2).map(|w| w[0].distance(w[1])).sum::<f64>() / 2.0;
    let mut walked = 0.0;
    for w in points.windows(2) {
        let d = w[0].distance(w[1]);
        if walked + d >= half {
            let t = if d > 0.0 { (half - walked) / d } else { 0.0 };
            return w[0].lerp(w[1], t);
        }
        walked += d;
    }
    points[0]
}

fn node_of(nodes: &mut Vec<Vec2>, p: Vec2) -> usize {
    if let Some(i) = nodes.iter().position(|n| n.distance(p) <= JOIN_TOL) {
        return i;
    }
    nodes.push(p);
    nodes.len() - 1
}

/// Union-find over graph nodes so loops know which component they belong to.
fn components(node_count: usize, edges: &[Edge]) -> Vec<usize> {
    let mut parent: Vec<usize> = (0..node_count).collect();
    fn find(parent: &mut [usize], i: usize) -> usize {
        let mut i = i;
        while parent[i] != i {
            parent[i] = parent[parent[i]];
            i = parent[i];
        }
        i
    }
    for e in edges {
        let a = find(&mut parent, e.nodes[0]);
        let b = find(&mut parent, e.nodes[1]);
        parent[a] = b;
    }
    (0..node_count).map(|i| find(&mut parent, i)).collect()
}

/// Concatenates the polylines of a face's half-edges. Junctions use the merged node
/// position so consecutive edges share a vertex exactly even if their endpoints differed
/// by up to [`JOIN_TOL`].
fn face_polyline(edges: &[Edge], nodes: &[Vec2], face: &[HalfEdge]) -> (Vec<Vec2>, Vec<Segment>) {
    let mut points = Vec::new();
    let mut segments = Vec::new();
    for h in face {
        let e = &edges[h.edge];
        let mut pts = e.polyline.clone();
        if h.dir == 1 {
            pts.reverse();
        }
        pts[0] = nodes[e.nodes[h.dir]];
        let kind = match e.kind {
            SegmentKind::Arc {
                center,
                radius,
                ccw,
            } => SegmentKind::Arc {
                center,
                radius,
                ccw: ccw == (h.dir == 0),
            },
            SegmentKind::Line => SegmentKind::Line,
        };
        for p in pts.iter().take(pts.len() - 1) {
            points.push(*p);
            segments.push(Segment {
                curve: e.curve,
                kind,
            });
        }
    }
    (points, segments)
}

/// Nests CCW loops into profiles with CW holes.
///
/// Every loop becomes a region, including one that sits inside another: the inner loop is
/// both a hole of its container and a region of its own, so a shape drawn inside a
/// rectangle can be extruded by itself, and picking the smallest region containing the
/// click resolves the overlap. Glyph outlines are the exception — a counter belongs to
/// its letter, so it is only ever a hole.
fn nest(loops: Vec<Loop>) -> Vec<Profile> {
    let n = loops.len();
    // The innermost loop enclosing each loop, if any: its container with the least area.
    let parent: Vec<Option<usize>> = (0..n)
        .map(|i| {
            let probe = loops[i].contour.points[0];
            (0..n)
                .filter(|&j| {
                    j != i
                        && loops[j].component != loops[i].component
                        && loops[j].area > loops[i].area
                        && loops[j].contour.contains(probe)
                })
                .min_by(|a, b| loops[*a].area.total_cmp(&loops[*b].area))
        })
        .collect();
    let is_counter = |i: usize| {
        loops[i].text.is_some() && parent[i].is_some_and(|p| loops[p].text == loops[i].text)
    };
    let mut profiles: Vec<(usize, Profile)> = (0..n)
        .filter(|&i| !is_counter(i))
        .map(|i| {
            (
                i,
                Profile {
                    outer: loops[i].contour.clone(),
                    holes: Vec::new(),
                },
            )
        })
        .collect();
    for (i, parent) in parent.iter().enumerate() {
        if let Some(parent) = parent
            && let Some((_, p)) = profiles.iter_mut().find(|(j, _)| j == parent)
        {
            p.holes.push(loops[i].contour.reversed());
        }
    }
    profiles.into_iter().map(|(_, p)| p).collect()
}
