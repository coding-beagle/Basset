//! Fillet and chamfer.
//!
//! Both are built the same way: for every selected edge, construct a prismatic *tool*
//! whose cross-section is the material to remove (convex edge) or add (concave edge),
//! and apply it with a boolean. The cross-section is exact in the plane perpendicular to
//! the edge; along curved edges the prism is mitred at every polyline joint so
//! consecutive pieces meet on the bisecting plane. Corners where several filleted edges
//! meet are simply the intersection of their tools, which is a crease rather than the
//! spherical patch a full-featured kernel would make — an accepted MVP limitation.
//!
//! What a blend costs is decided here rather than in the boolean. The tools of one
//! feature are folded into as few solids as they can be before any of them meets the
//! body ([`merge_tools`]), because a boolean is superlinear in the body it is given and
//! every tool applied on its own hands the next one a body it has fragmented: filleting
//! both rims of a 536-facet cylinder measured 165 ms for the first tool and 1 318 ms for
//! the second, identical one. The other half is the budget, which decides how many
//! facets the tools may carry at all; see [`MAX_FEATURE_POLYGONS`].

use basset_math::{Aabb, Vec3};

use crate::csg::{BoolOp, boolean};
use crate::error::KernelError;
use crate::geometry::Tessellation;
use crate::ids::{EdgeKey, FaceKey, FaceRole, OpId};
use crate::solid::{Edge, EdgeSegment, MERGE_TOL, Solid, SolidBuilder, SurfaceKind, newell_normal};

/// How far the tool reaches beyond the faces it trims. Keeps the tool's planes off the
/// solid's planes so the boolean never has to resolve coincident faces, and closes the
/// hairline gap where a chain ends flush with another face.
const CLEARANCE: f64 = 1e-4;

/// Facets one blend tool may carry.
///
/// The tool is a prism swept along the edge, so its facet count is the edge's polyline
/// length times the arc's: a radius typed into a box multiplies a tessellation density
/// chosen elsewhere, and filleting both rims of a finely tessellated cylinder used to
/// allocate tens of gigabytes before the OOM killer ended the session.
///
/// The BSP boolean degenerates on exactly this shape — a convex sweep gives every
/// candidate splitting plane the whole remainder behind it, so the tree is a list (see
/// [`csg`](crate::csg)) and time is quadratic in the facet count. Nothing else bounds it
/// any more: the walks no longer recurse, so depth costs no stack, and the peak memory
/// measured on two rims at 27k facets was 106 MB. The budgets are therefore a choice of
/// how long a preview may take. Measured on one core, both rims of a cylinder cost
/// 0.18 s at 2.6k facets in, 0.5 s at 4.5k and 3.7 s at 9k, and the per-feature
/// budget below sits at about a second. It is spent by coarsening the arc first, because
/// a faceted fillet is a better answer than a refusal.
const MAX_TOOL_POLYGONS: usize = 4_000;

/// Facets one blend feature may put through its booleans, tools and body together.
///
/// Each tool is applied against the accumulating result, so the last boolean of an
/// n-edge blend runs over everything the earlier ones fragmented, and it is that pass
/// whose time has to stay reasonable. Bounding the tools alone would not bound it. The
/// body's share is fixed, so what is left is shared out between the tools, and each
/// coarsens its arc to fit its share the way it does for [`MAX_TOOL_POLYGONS`].
const MAX_FEATURE_POLYGONS: usize = 7_000;

/// Coarsest arc a fillet is willing to draw, and the `.max(2)` the rings need: a ring has
/// to carry both tangent points and at least one point between them.
const MIN_ARC_SEGMENTS: usize = 2;

/// Facets of the swept tool: one ring of section vertices per chain segment. A section
/// carries the arc's points, its two tangent points and three scaffolding corners. An
/// open chain's end caps are fans over one ring each, so they are counted as two more.
fn tool_polygons(rings: usize, arc_segments: usize) -> usize {
    rings * (arc_segments + 4)
}

/// Whether a chain closes on itself: its end caps are then unnecessary and not built.
fn is_closed(chain: &[EdgeSegment]) -> bool {
    chain.len() > 1
        && chain[0].start.distance_squared(chain.last().unwrap().end) < MERGE_TOL * MERGE_TOL
}

/// Rings a chain's tool is built from, caps included.
fn ring_count(chain: &[EdgeSegment]) -> usize {
    if is_closed(chain) {
        chain.len()
    } else {
        chain.len() + 2
    }
}

#[derive(Clone, Copy)]
enum Blend {
    Fillet { radius: f64 },
    Chamfer { distance: f64 },
}

impl Blend {
    fn kind(self) -> BlendKind {
        match self {
            Blend::Fillet { .. } => BlendKind::Fillet,
            Blend::Chamfer { .. } => BlendKind::Chamfer,
        }
    }
}

pub fn fillet(
    op: OpId,
    solid: &Solid,
    edges: &[EdgeKey],
    radius: f64,
    tess: &Tessellation,
) -> Result<Solid, KernelError> {
    apply(op, solid, edges, Blend::Fillet { radius }, tess)
}

pub fn chamfer(
    op: OpId,
    solid: &Solid,
    edges: &[EdgeKey],
    distance: f64,
) -> Result<Solid, KernelError> {
    apply(
        op,
        solid,
        edges,
        Blend::Chamfer { distance },
        &Tessellation::default(),
    )
}

fn apply(
    op: OpId,
    solid: &Solid,
    keys: &[EdgeKey],
    blend: Blend,
    tess: &Tessellation,
) -> Result<Solid, KernelError> {
    let tools = tools_for(op, solid, keys, blend, tess)?;
    let mut result = solid.clone();
    for (tool, bool_op) in merge_tools(tools, solid.polygon_count()) {
        result = boolean(&result, &tool, bool_op)?;
    }
    Ok(result)
}

/// One tool per chain of every selected edge, with the operation that applies it.
fn tools_for(
    op: OpId,
    solid: &Solid,
    keys: &[EdgeKey],
    blend: Blend,
    tess: &Tessellation,
) -> Result<Vec<(Solid, BoolOp)>, KernelError> {
    let size = match blend {
        Blend::Fillet { radius } => radius,
        Blend::Chamfer { distance } => distance,
    };
    if !size.is_finite() || size <= 0.0 {
        return Err(KernelError::NonPositiveBlend);
    }
    // Tools are built from the original edges, before any of them is blended away, so
    // the result does not depend on the order edges were selected in.
    let all_edges = solid.edges();
    let mut chains = Vec::new();
    for (i, key) in keys.iter().enumerate() {
        let edge = all_edges
            .iter()
            .find(|e| e.key == *key)
            .ok_or(KernelError::MissingEdge(*key))?;
        chains.extend(edge.chains().into_iter().map(|c| (i as u32, *key, c)));
    }
    // Whether there is material for it at all, checked after the edges are known to exist
    // so a stale reference is still reported as the stale reference it is. The comparison
    // carries a few ulps of slack because the limit arrives through a normalise, a dot
    // product and a division, and a blend that fits its material exactly — the 2 mm round
    // on the rim of a 2 mm plate — must not be turned away over the last bits of that.
    if let Some(limit) = size_limit(solid, &all_edges, keys, blend.kind())
        && size > limit * (1.0 + 1e-9)
    {
        return Err(KernelError::BlendTooLarge { size, limit });
    }
    // The feature's budget less the body, shared equally between the tools: decided
    // before any tool is built, so an over-ambitious blend costs the user a coarser arc
    // or a message rather than the minutes it would take to fail part-way through. A
    // body that alone leaves the tools less than their coarsest form is refused here,
    // with the whole feature's count, because no arc can be coarsened out of that.
    let body = solid.polygon_count();
    let coarsest: usize = chains
        .iter()
        .map(|(_, _, c)| tool_polygons(ring_count(c), MIN_ARC_SEGMENTS))
        .sum();
    if body + coarsest > MAX_FEATURE_POLYGONS {
        return Err(KernelError::BlendTooDense {
            needed: body + coarsest,
            budget: MAX_FEATURE_POLYGONS,
        });
    }
    // Each chain gets its coarsest form and an equal share of the slack over that, so a
    // long chain beside a short one is not starved by an even split of the whole.
    let slack = (MAX_FEATURE_POLYGONS - body - coarsest) / chains.len().max(1);
    let mut tools = Vec::with_capacity(chains.len());
    for (i, key, chain) in &chains {
        let budget =
            (tool_polygons(ring_count(chain), MIN_ARC_SEGMENTS) + slack).min(MAX_TOOL_POLYGONS);
        tools.push(tool_for_chain(op, *i, *key, chain, blend, tess, budget)?);
    }
    Ok(tools)
}

// --- How large a blend may be ---------------------------------------------------------
//
// Nothing below this point notices when a blend has run out of material. The tool is
// built from the edge and the size alone and the boolean applies it either way, so past
// the point where the tool is wider than what it has to work in the answer is a shell
// that is closed and wrong: a face swallowed whole, a neighbouring feature cut away, or —
// round a closed rim — a swept prism turned inside out. The bound is derived here, before
// a tool exists, from the one quantity every case turns on: the *setback*, how far from
// the edge the blend lands on each of the two faces it joins.

/// A blend's kind without its size. How much room an edge has does not depend on the size
/// being asked for, only on how a size of a given kind becomes a setback, so the limit is
/// derived once and the size held up against it.
#[derive(Clone, Copy, PartialEq, Eq)]
enum BlendKind {
    Fillet,
    Chamfer,
}

impl BlendKind {
    /// Setback per unit of size, across a face whose dihedral angle at the edge is `phi`.
    ///
    /// A fillet of radius r rolls in the corner with its centre on the bisector and
    /// touches each face where the perpendicular from that centre meets it, which is
    /// r/tan(phi/2) from the edge: the far leg of the right triangle whose near leg is r
    /// and whose angle at the edge is phi/2. It is the same `d` [`section`] lays its
    /// cross-section out with. A chamfer's setback is its distance, by definition.
    fn setback_per_size(self, phi: f64) -> f64 {
        match self {
            BlendKind::Fillet => 1.0 / (phi / 2.0).tan(),
            BlendKind::Chamfer => 1.0,
        }
    }
}

/// Distance along a ray below which a crossing is the boundary the ray started from
/// rather than one ahead of it: the segment being measured is part of the boundary the
/// ray is cast into, and the ray leaves from a point on it.
const OWN_BOUNDARY: f64 = 8.0 * MERGE_TOL;

/// How close a ray and a boundary segment come before they count as crossing. Inside a
/// planar face the two are coplanar and the crossing is exact, so this only absorbs
/// rounding; a ray that finds nothing falls back to the face's bounding box, which is
/// never smaller than the truth.
const CROSSES: f64 = 1e-6;

/// The largest blend of `kind` the edges `keys` can carry together, or `None` when none
/// of them is an edge of this solid.
///
/// Every bound is the same statement seen from a different side: *the strip of face a
/// blend consumes has to be there to consume*. The blend lands a setback from the edge on
/// each adjacent face, and everything between the edge and that landing is either gone
/// (convex, the tool is subtracted) or buried (concave, the tool is added). So each
/// segment's setback is held against how far its face actually reaches from that segment
/// in the direction the blend travels — [`reach_in_face`] — and the size at which the two
/// are equal is the limit.
///
/// * A cube's edge is stopped by the far side of each face it borders. A 10 mm face takes
///   a 10 mm setback and no more; past that the tool's tangent plane lies beyond the far
///   edge and the subtraction takes the material behind it as well.
/// * A cylinder's rim is stopped by itself. The ray across the top disc from one rim
///   segment leaves through the rim 2R away, and that far side is being blended by the
///   same feature and is coming this way, so the two setbacks share the 2R between them
///   and r < R. That is also exactly where the tool's centre circle, of radius R − r,
///   collapses to a point and the sweep turns inside out.
/// * A concave edge is the same bound with the material's sign reversed: the fillet's
///   landing on the floor of the pocket has to be on floor that exists, or the union
///   raises a wall where the far side of the pocket used to open out.
/// * Two edges of one face blended together are the rim case without the closure. Each
///   one's ray lands on the other, both are in `keys`, and the room between them is
///   divided between their two setbacks.
///
/// Where the blocking boundary is itself being blended its setback is taken off the room
/// rather than half of it being assumed, because two edges of different dihedral angles
/// do not eat into the gap at the same rate. Both setbacks are proportional to the one
/// size the feature applies, so the largest size that fits is `reach / (here + there)`.
fn size_limit(solid: &Solid, all_edges: &[Edge], keys: &[EdgeKey], kind: BlendKind) -> Option<f64> {
    let selected: Vec<&Edge> = keys
        .iter()
        .filter_map(|k| all_edges.iter().find(|e| e.key == *k))
        .collect();
    if selected.is_empty() {
        return None;
    }
    let mut limit = f64::INFINITY;
    for edge in &selected {
        for face_key in [edge.key.a, edge.key.b] {
            let Some(face) = solid.face(face_key) else {
                continue;
            };
            // The face's own boundary, each segment flagged with whether this feature is
            // blending it too, which is what decides whether the room ahead of a segment
            // is this blend's alone or shared with the one coming the other way.
            let boundary: Vec<(&EdgeSegment, bool)> = all_edges
                .iter()
                .filter(|e| e.key.touches(face_key))
                .flat_map(|e| e.segments.iter().map(|s| (s, keys.contains(&e.key))))
                .collect();
            let extent = Aabb::from_points(face.polygons.iter().flat_map(|p| p.vertices.clone()));
            for seg in &edge.segments {
                let (_, da, db, phi) = dihedral(seg);
                let here = kind.setback_per_size(phi);
                // Smooth or degenerate: there is no corner here and so no setback, and a
                // zero would let every size through. `tool_for_chain` refuses such an
                // edge by name, which is the message the user should get.
                if !here.is_finite() || here <= 0.0 {
                    continue;
                }
                let into = if face_key == edge.key.a { da } else { db };
                let from = (seg.start + seg.end) * 0.5;
                let (reach, blocker) = reach_in_face(from, into, &boundary, &extent);
                let there = blocker
                    .map(|p| kind.setback_per_size(p))
                    .filter(|s| s.is_finite() && *s > 0.0)
                    .unwrap_or(0.0);
                limit = limit.min(reach / (here + there));
            }
        }
    }
    limit.is_finite().then_some(limit.max(0.0))
}

/// How far a point `from` on the boundary of a face travels across it along `into` before
/// leaving it, together with the dihedral angle of the boundary it leaves through when
/// that boundary is itself being blended.
///
/// The ray is cast against the face's own boundary segments. That is exact for a planar
/// face, where ray and boundary are coplanar and the crossing is a real intersection, and
/// for the ruled curved faces a blend actually meets, where `into` runs along the ruling
/// and the ray stays on the surface — the side of a cylinder measured down from one rim
/// to the other. On a curved face where it does neither, the ray leaves the surface and
/// finds nothing; the answer is then the face's bounding box measured along `into`, which
/// the face cannot reach past and which is an honest ceiling, if a loose one.
fn reach_in_face(
    from: Vec3,
    into: Vec3,
    boundary: &[(&EdgeSegment, bool)],
    extent: &Aabb,
) -> (f64, Option<f64>) {
    let cap: f64 = (0..3)
        .map(|i| (into[i] * (extent.min[i] - from[i])).max(into[i] * (extent.max[i] - from[i])))
        .sum();
    let mut best = f64::INFINITY;
    let mut blocker = None;
    for (seg, blended) in boundary {
        let along = seg.end - seg.start;
        // Nowhere near the ray's line: no point of the segment reaches within half its
        // own length of the line its midpoint sits that far off. Most of a rim is ruled
        // out here, before the crossing is solved for, which is what keeps the check
        // per-edge rather than something the user waits on.
        let mid = (seg.start + seg.end) * 0.5 - from;
        let reach = along.length() * 0.5 + CROSSES;
        if (mid - into * mid.dot(into)).length_squared() > reach * reach {
            continue;
        }
        let (near, far) = {
            let (a, b) = ((seg.start - from).dot(into), (seg.end - from).dot(into));
            (a.min(b), a.max(b))
        };
        // Behind the ray, or no nearer than what has been found already: the crossing
        // lies on the segment, so it is at least as far along as the nearer endpoint.
        if far <= OWN_BOUNDARY || near > best {
            continue;
        }
        let (a, b, c) = (into.dot(into), into.dot(along), along.dot(along));
        let denom = a * c - b * b;
        // Parallel to the ray: a boundary running beside it never stops it.
        if denom.abs() < 1e-12 {
            continue;
        }
        let w = from - seg.start;
        // The closest approach of the two lines, pinned to the segment so that a ray
        // leaving exactly through a vertex — an odd-sided rim crossed through its centre
        // — is the hit it is rather than a miss either neighbour disowns.
        let u = ((a * along.dot(w) - b * into.dot(w)) / denom).clamp(0.0, 1.0);
        let q = seg.start + along * u;
        let t = (q - from).dot(into);
        if t <= OWN_BOUNDARY || t >= best || (q - from - into * t).length() > CROSSES {
            continue;
        }
        best = t;
        blocker = blended.then(|| dihedral(seg).3);
    }
    if best <= cap {
        (best, blocker)
    } else {
        (cap, None)
    }
}

/// The largest fillet radius the edges `keys` of `solid` can take together, or `None`
/// when none of them is an edge of it.
///
/// The editor clamps a dragged radius to this so a blend cannot be driven past the
/// material in the first place; [`fillet`] refuses anything over it with
/// [`KernelError::BlendTooLarge`], which is the same number arrived at the same way.
pub fn max_fillet_radius(solid: &Solid, keys: &[EdgeKey]) -> Option<f64> {
    size_limit(solid, &solid.edges(), keys, BlendKind::Fillet)
}

/// The largest chamfer distance the edges `keys` of `solid` can take together. See
/// [`max_fillet_radius`]; a chamfer's setback is its distance, so the limit is the room
/// itself rather than the room turned back through the dihedral angle.
pub fn max_chamfer_distance(solid: &Solid, keys: &[EdgeKey]) -> Option<f64> {
    size_limit(solid, &solid.edges(), keys, BlendKind::Chamfer)
}

/// Folds the tools into as few as possible, so the body goes through one boolean rather
/// than one per edge.
///
/// This is where a multi-edge blend is won or lost. Every boolean runs against the
/// *accumulating* result, so the second one meets a body the first has fragmented, and
/// the boolean is superlinear in that count: filleting both rims of a 536-facet cylinder
/// measured 165 ms for the first tool against the bare body and 1 318 ms for the second,
/// identical tool against the 2 702 facets the first left behind. Subtracting a set of
/// tools is subtracting their union whichever order it is done in, and the same for
/// adding, so the tools can be folded together first — and a tool is small where the
/// body is large, which is why folding them is nearly free.
///
/// Tools that do not reach each other are folded by concatenating their faces: two
/// closed shells that share no point are one closed shell of two components, which the
/// BSP handles without ever being asked about the gap between them. Only tools that do
/// meet — the corner where two filleted edges of a box run together — cost a real
/// boolean, and that one is tool against tool rather than tool against body.
///
/// Folding may only reorder tools that share an operation: a subtraction and a union
/// that overlap are not interchangeable, so the tools are folded within runs of one
/// operation and the runs keep their order.
fn merge_tools(tools: Vec<(Solid, BoolOp)>, body: usize) -> Vec<(Solid, BoolOp)> {
    let mut out: Vec<(Solid, BoolOp)> = Vec::new();
    // A group is one tool being built up, with the boxes of the tools already in it kept
    // apart rather than as one box round the lot: a third tool clear of both of two
    // grouped tools is still clear of them where their common box, which spans the gap
    // between them, says otherwise.
    let mut groups: Vec<(Solid, Vec<Aabb>)> = Vec::new();
    let mut run: Option<BoolOp> = None;
    for (tool, op) in tools {
        if let Some(previous) = run.replace(op).filter(|p| *p != op) {
            out.extend(groups.drain(..).map(|(t, _)| (t, previous)));
        }
        let box_of = tool.aabb();
        // Into the first group it stays clear of, which is free; failing that a boolean,
        // which is worth its cost only while tool against tool is the smaller problem
        // than tool against body — the usual one, since a tool is a band across a body.
        // Four fillets meeting at the corners of a box are four 87-facet tools, and
        // folding those against each other cost 11 ms where applying each to the
        // eight-facet box cost 2 ms.
        match groups
            .iter_mut()
            .find(|(_, boxes)| boxes.iter().all(|b| clear_of(b, &box_of)))
        {
            Some((acc, boxes)) => {
                absorb(acc, tool);
                boxes.push(box_of);
            }
            None => {
                let fold = groups
                    .first()
                    .is_some_and(|(acc, _)| acc.polygon_count() + tool.polygon_count() < body);
                match fold.then(|| boolean(&groups[0].0, &tool, BoolOp::Union)) {
                    Some(Ok(merged)) => {
                        groups[0].0 = merged;
                        groups[0].1.push(box_of);
                    }
                    Some(Err(e)) => {
                        // The tools stay separate and cost a boolean each against the
                        // body, which is what they cost before any of this.
                        log::warn!("blend: tools left unfolded, their union failed: {e:?}");
                        groups.push((tool, vec![box_of]));
                    }
                    None => groups.push((tool, vec![box_of])),
                }
            }
        }
    }
    if let Some(op) = run {
        out.extend(groups.into_iter().map(|(t, _)| (t, op)));
    }
    out
}

/// Concatenates one tool's faces into another's, joining them by key rather than
/// appending blindly: the two chains of one selected edge carry the same blend key, and
/// a solid with that key twice would name one surface two different faces.
fn absorb(acc: &mut Solid, tool: Solid) {
    for face in tool.faces {
        match acc.faces.iter_mut().find(|f| f.key == face.key) {
            Some(existing) => existing.polygons.extend(face.polygons),
            None => acc.faces.push(face),
        }
    }
}

/// Whether two boxes are far enough apart that no point of one is a point of the other,
/// with the boolean's own coplanarity tolerance to spare. Anything closer is folded by a
/// boolean instead, because concatenating shells that touch would hand the BSP a
/// non-manifold edge.
fn clear_of(a: &Aabb, b: &Aabb) -> bool {
    let gap = 8.0 * MERGE_TOL;
    (0..3).any(|i| a.max[i] + gap < b.min[i] || b.max[i] + gap < a.min[i])
}

/// Cross-section of the tool in the plane perpendicular to the edge at a point `c`.
/// Coordinates are 3D offsets from `c`; `blend_start..blend_end` are the vertices lying
/// on the new surface, the rest is scaffolding that ends up outside the result.
struct Section {
    offsets: Vec<Vec3>,
    blend_range: std::ops::Range<usize>,
}

/// In-face directions leading away from the edge into faces `a` and `b`, and the angle
/// between them (the material's dihedral angle for a convex edge).
fn dihedral(seg: &EdgeSegment) -> (Vec3, Vec3, Vec3, f64) {
    let t = (seg.end - seg.start).normalize();
    let da = seg.normal_a.cross(t).normalize();
    let db = -(seg.normal_b.cross(t)).normalize();
    let phi = da.dot(db).clamp(-1.0, 1.0).acos();
    (t, da, db, phi)
}

/// Which way the blend leaves the edge: into the material for a convex edge, out into
/// the open for a concave one. It is the same in-face bisector [`section`] puts the
/// fillet's centre along, which is why an arrow drawn along it reads as the radius —
/// pointing it the other way, out of a corner the fillet is cutting into, says the
/// opposite of what the tool does.
///
/// `None` where the two faces do not fold: their in-face directions are then opposite
/// and there is no corner to blend, the condition [`Edge::smooth`](crate::Edge) reports.
pub fn blend_direction(seg: &EdgeSegment) -> Option<Vec3> {
    // Not `dihedral`: this one is handed whatever edge the pointer landed on, including
    // a degenerate segment left by a boolean, and must answer rather than produce NaN.
    let t = (seg.end - seg.start).try_normalize()?;
    let da = seg.normal_a.cross(t).try_normalize()?;
    let db = (-seg.normal_b.cross(t)).try_normalize()?;
    (da + db).try_normalize()
}

/// `arc_segments` is fixed per chain: every ring must have the same vertex count for the
/// quads between them to make sense, and rounding noise in the dihedral angle would
/// otherwise let neighbouring segments round the count differently.
fn section(seg: &EdgeSegment, blend: Blend, arc_segments: usize, convex: bool) -> Section {
    let (t, da, db, phi) = dihedral(seg);
    let (na, nb) = (seg.normal_a, seg.normal_b);
    let (ta, tb, arc) = match blend {
        Blend::Chamfer { distance } => (da * distance, db * distance, Vec::new()),
        Blend::Fillet { radius } => {
            let d = radius / (phi / 2.0).tan();
            let centre = (da + db).normalize() * (radius / (phi / 2.0).sin());
            let ta = da * d;
            let tb = db * d;
            // The arc from ta to tb about `centre`, on the side nearest the edge.
            let sweep = std::f64::consts::PI - phi;
            let n = arc_segments;
            let u = (ta - centre).normalize();
            let v = t.cross(u)
                * (if t.cross(u).dot(tb - centre) >= 0.0 {
                    1.0
                } else {
                    -1.0
                });
            let arc = (1..n)
                .map(|i| {
                    let a = sweep * i as f64 / n as f64;
                    centre + (u * a.cos() + v * a.sin()) * radius
                })
                .collect();
            (ta, tb, arc)
        }
    };
    // Scaffolding corner beyond (convex) or beneath (concave) the edge, clear of both faces.
    let corner_scale = CLEARANCE / (1.0 + na.dot(nb));
    let sign = if convex { 1.0 } else { -1.0 };
    let mut offsets = vec![ta];
    offsets.extend(arc);
    offsets.push(tb);
    let blend_range = 0..offsets.len();
    offsets.push(tb + nb * (CLEARANCE * sign));
    offsets.push((na + nb) * corner_scale * sign);
    offsets.push(ta + na * (CLEARANCE * sign));
    Section {
        offsets,
        blend_range,
    }
}

fn tool_for_chain(
    op: OpId,
    index: u32,
    key: EdgeKey,
    chain: &[EdgeSegment],
    blend: Blend,
    tess: &Tessellation,
    budget: usize,
) -> Result<(Solid, BoolOp), KernelError> {
    let first = &chain[0];
    let (t0, _, db, _) = dihedral(first);
    let turn = first.normal_a.dot(db);
    if turn.abs() < 1e-6 {
        // The faces do not fold here, so there is no dihedral to fill. This is the same
        // condition `Edge::smooth` reports, which is why the editor never offers such an
        // edge; a saved feature whose edge has since flattened arrives here instead.
        return Err(KernelError::TangentEdge(key));
    }
    let convex = turn < 0.0;
    let bool_op = if convex {
        BoolOp::Subtract
    } else {
        BoolOp::Union
    };
    let closed = is_closed(chain);
    let rings = ring_count(chain);
    let affordable = (budget / rings).saturating_sub(4);
    let arc_segments = match blend {
        Blend::Fillet { radius } => {
            if affordable < MIN_ARC_SEGMENTS {
                return Err(KernelError::BlendTooDense {
                    needed: tool_polygons(rings, MIN_ARC_SEGMENTS),
                    budget,
                });
            }
            let wanted = chain
                .iter()
                .map(|s| tess.segment_count(radius, std::f64::consts::PI - dihedral(s).3))
                .max()
                .unwrap_or(1)
                .max(MIN_ARC_SEGMENTS);
            if wanted > affordable {
                log::warn!(
                    "blend on {key:?}: arc coarsened from {wanted} to {affordable} facets \
                     to keep the tool for its {} edge segments inside {budget} polygons",
                    chain.len()
                );
            }
            wanted.min(affordable)
        }
        Blend::Chamfer { .. } => {
            if tool_polygons(rings, 0) > budget {
                return Err(KernelError::BlendTooDense {
                    needed: tool_polygons(rings, 0),
                    budget,
                });
            }
            0
        }
    };

    // One ring of section points per joint. Interior joints take the section of the
    // incoming segment projected onto the bisecting plane; chain ends extend slightly.
    let mut rings: Vec<Vec<Vec3>> = Vec::with_capacity(chain.len() + 1);
    let sections: Vec<Section> = chain
        .iter()
        .map(|s| section(s, blend, arc_segments, convex))
        .collect();
    let joint_count = if closed { chain.len() } else { chain.len() + 1 };
    for j in 0..joint_count {
        let (incoming, outgoing) = if closed {
            (
                Some(&chain[(j + chain.len() - 1) % chain.len()]),
                Some(&chain[j]),
            )
        } else {
            (if j > 0 { Some(&chain[j - 1]) } else { None }, chain.get(j))
        };
        let ring = match (incoming, outgoing) {
            (Some(inc), Some(out)) => {
                let ti = (inc.end - inc.start).normalize();
                let to = (out.end - out.start).normalize();
                let bisector = (ti + to).try_normalize().unwrap_or(ti);
                let denom = ti.dot(bisector);
                let sec = &sections[if closed {
                    (j + chain.len() - 1) % chain.len()
                } else {
                    j - 1
                }];
                sec.offsets
                    .iter()
                    .map(|o| {
                        let p = inc.end + *o;
                        p + ti * ((inc.end - p).dot(bisector) / denom)
                    })
                    .collect()
            }
            (None, Some(out)) => {
                let t = (out.end - out.start).normalize();
                sections[j]
                    .offsets
                    .iter()
                    .map(|o| out.start + *o - t * CLEARANCE)
                    .collect()
            }
            (Some(inc), None) => {
                let t = (inc.end - inc.start).normalize();
                sections[j - 1]
                    .offsets
                    .iter()
                    .map(|o| inc.end + *o + t * CLEARANCE)
                    .collect()
            }
            (None, None) => unreachable!("a chain has at least one segment"),
        };
        rings.push(ring);
    }

    let role = match blend {
        Blend::Fillet { .. } => FaceRole::Fillet(index),
        Blend::Chamfer { .. } => FaceRole::Chamfer(index),
    };
    let blend_key = FaceKey::new(op, role);
    let surface = match (blend, chain.len()) {
        (Blend::Fillet { radius }, 1) => {
            let sec = &sections[0];
            let da = sec.offsets[0];
            let db = sec.offsets[sec.blend_range.end - 1];
            let phi = da.normalize().dot(db.normalize()).clamp(-1.0, 1.0).acos();
            let centre = first.start + (da + db).normalize() * (radius / (phi / 2.0).sin());
            SurfaceKind::Cylindrical {
                origin: centre,
                axis: t0,
                radius,
            }
        }
        (Blend::Chamfer { .. }, 1) => SurfaceKind::Planar { normal: Vec3::ZERO },
        _ => SurfaceKind::Freeform,
    };
    let scaffold = |i: usize| FaceKey::new(op, FaceRole::Generic(index * 8 + i as u32));

    let mut b = SolidBuilder::default();
    let n = sections[0].offsets.len();
    // Rings advancing along +t with counter-clockwise winding (seen from +t) make the
    // extrude quad order face outward; a clockwise ring needs the mirror order.
    let ccw = newell_normal(&rings[0]).dot(t0) > 0.0;
    let pair_count = if closed { rings.len() } else { rings.len() - 1 };
    for r in 0..pair_count {
        let (cur, next) = (&rings[r], &rings[(r + 1) % rings.len()]);
        for i in 0..n {
            let k = (i + 1) % n;
            let on_blend =
                sections[0].blend_range.contains(&i) && sections[0].blend_range.contains(&k);
            let (key, surf) = if on_blend {
                (blend_key, surface)
            } else {
                (scaffold(i), SurfaceKind::Planar { normal: Vec3::ZERO })
            };
            let quad = if ccw {
                [cur[i], cur[k], next[k], next[i]]
            } else {
                [cur[k], cur[i], next[i], next[k]]
            };
            b.push_quad(key, surf, quad);
        }
    }
    if !closed {
        let last = chain.last().unwrap();
        let t_end = (last.end - last.start).normalize();
        let ends = [
            (&rings[0], first.start - t0 * CLEARANCE, -t0, 6u32),
            (
                rings.last().unwrap(),
                last.end + t_end * CLEARANCE,
                t_end,
                7,
            ),
        ];
        for (ring, centre, desired, i) in ends {
            let cap_key = FaceKey::new(op, FaceRole::Generic(index * 8 + i));
            for [p, q, r] in cap_triangles(ring, centre, &sections[0].blend_range) {
                let poly = if (q - p).cross(r - p).dot(desired) >= 0.0 {
                    vec![p, q, r]
                } else {
                    vec![p, r, q]
                };
                b.push(cap_key, SurfaceKind::Planar { normal: Vec3::ZERO }, poly);
            }
        }
    }
    let tool = b.finish();
    if tool.is_empty() {
        return Err(KernelError::TangentEdge(key));
    }
    Ok((tool, bool_op))
}

/// Triangulates a section ring as a fan from the edge point.
///
/// The ring is star-shaped about the edge point by construction, and fanning from there
/// keeps every triangle vertex that lies near the solid's faces exactly *on* the edge
/// point. An ear-clipping of the same ring produces slivers converging on the tiny
/// scaffold corner, whose intersections with the solid's planes cluster within the vertex
/// merge tolerance and break the shell.
fn cap_triangles(ring: &[Vec3], centre: Vec3, blend: &std::ops::Range<usize>) -> Vec<[Vec3; 3]> {
    let n = ring.len();
    (0..n)
        .filter(|i| !(blend.contains(i) && blend.contains(&(i + 1))) || *i + 1 < blend.end)
        .map(|i| [centre, ring[i], ring[(i + 1) % n]])
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::primitives::cuboid;
    use approx::assert_relative_eq;
    use basset_math::Vec3;
    use std::f64::consts::PI;

    fn cube() -> Solid {
        cuboid(OpId::new(1), Vec3::ZERO, Vec3::splat(10.0))
    }

    fn edge_between(a: FaceRole, b: FaceRole) -> EdgeKey {
        EdgeKey::new(FaceKey::new(OpId::new(1), a), FaceKey::new(OpId::new(1), b))
    }

    fn fine() -> Tessellation {
        Tessellation {
            chord_tolerance: 1e-4,
            max_segment_angle: 2f64.to_radians(),
        }
    }

    #[test]
    fn chamfer_one_edge_of_cube() {
        let c = cube();
        let key = edge_between(FaceRole::EndCap, FaceRole::Side(0));
        let r = chamfer(OpId::new(2), &c, &[key], 2.0).unwrap();
        assert_relative_eq!(r.volume(), 1000.0 - 0.5 * 2.0 * 2.0 * 10.0, epsilon = 1e-3);
        assert!(r.is_closed(), "{:?}", r.validate());
        let face = r
            .face(FaceKey::new(OpId::new(2), FaceRole::Chamfer(0)))
            .unwrap();
        assert!(matches!(face.surface, SurfaceKind::Planar { .. }));
        assert_relative_eq!(face.area(), 2.0 * 2f64.sqrt() * 10.0, epsilon = 1e-2);
        // The original faces still exist under their keys.
        assert!(
            r.face(FaceKey::new(OpId::new(1), FaceRole::EndCap))
                .is_some()
        );
        assert_eq!(r.faces.len(), 7);
    }

    #[test]
    fn fillet_one_edge_of_cube() {
        let c = cube();
        let key = edge_between(FaceRole::EndCap, FaceRole::Side(0));
        let r = fillet(OpId::new(2), &c, &[key], 3.0, &fine()).unwrap();
        let removed = (9.0 - PI * 9.0 / 4.0) * 10.0;
        assert_relative_eq!(r.volume(), 1000.0 - removed, epsilon = 0.05);
        assert!(r.is_closed(), "{:?}", r.validate());
        let face = r
            .face(FaceKey::new(OpId::new(2), FaceRole::Fillet(0)))
            .unwrap();
        assert!(matches!(face.surface, SurfaceKind::Cylindrical { radius, .. } if radius == 3.0));
        assert_relative_eq!(face.area(), PI * 3.0 / 2.0 * 10.0, epsilon = 0.05);
        // The fillet trims the top face by 3 mm along one side.
        let top = r
            .face(FaceKey::new(OpId::new(1), FaceRole::EndCap))
            .unwrap();
        assert_relative_eq!(top.area(), 70.0, epsilon = 1e-2);
    }

    #[test]
    fn fillet_all_top_edges() {
        let c = cube();
        let keys: Vec<EdgeKey> = (0..4)
            .map(|i| edge_between(FaceRole::EndCap, FaceRole::Side(i)))
            .collect();
        let r = fillet(OpId::new(2), &c, &keys, 2.0, &fine()).unwrap();
        assert!(r.is_closed(), "{:?}", r.validate());
        assert!(r.volume() < 1000.0 - 4.0 * (4.0 - PI) * 8.0);
        assert!(r.volume() > 1000.0 - 4.0 * (4.0 - PI) * 10.0);
        for i in 0..4 {
            assert!(
                r.face(FaceKey::new(OpId::new(2), FaceRole::Fillet(i)))
                    .is_some()
            );
        }
    }

    #[test]
    fn fillet_concave_edge_adds_material() {
        // An L-block: cube with the top-front quarter removed.
        let c = cube();
        let notch = cuboid(
            OpId::new(9),
            Vec3::new(-1.0, -1.0, 5.0),
            Vec3::new(11.0, 5.0, 11.0),
        );
        let l = boolean(&c, &notch, BoolOp::Subtract).unwrap();
        assert_relative_eq!(l.volume(), 1000.0 - 250.0, epsilon = 1e-6);
        // The concave edge lies between the notch's top (now floor) and its far wall.
        let key = EdgeKey::new(
            FaceKey::new(OpId::new(9), FaceRole::StartCap),
            FaceKey::new(OpId::new(9), FaceRole::Side(2)),
        );
        assert!(
            l.edges().iter().any(|e| e.key == key),
            "{:?}",
            l.edges().iter().map(|e| e.key).collect::<Vec<_>>()
        );
        let r = fillet(OpId::new(3), &l, &[key], 2.0, &fine()).unwrap();
        let added = (4.0 - PI) * 10.0;
        assert_relative_eq!(r.volume(), 750.0 + added, epsilon = 0.05);
        assert!(r.is_closed(), "{:?}", r.validate());
    }

    #[test]
    fn fillet_curved_edge_of_cylinder() {
        let cyl = crate::primitives::cylinder(
            OpId::new(1),
            Vec3::ZERO,
            Vec3::Z,
            5.0,
            10.0,
            &Tessellation::default(),
        );
        let key = EdgeKey::new(
            FaceKey::new(OpId::new(1), FaceRole::EndCap),
            FaceKey::new(OpId::new(1), FaceRole::Side(0)),
        );
        let r = fillet(OpId::new(2), &cyl, &[key], 1.0, &Tessellation::default()).unwrap();
        assert!(r.is_closed(), "{:?}", r.validate());
        // Pappus: removed volume of the corner ring ≈ (1 − π/4) · 2π · r_centroid.
        // Pappus: the removed ring is the corner region (area 1 − π/4, centroid 0.777 from
        // the corner) swept around the rim; compare against the faceted cylinder itself.
        let expected = cyl.volume() - (1.0 - PI / 4.0) * 2.0 * PI * (5.0 - 0.777);
        assert_relative_eq!(r.volume(), expected, epsilon = 1.5);
        assert!(matches!(
            r.face(FaceKey::new(OpId::new(2), FaceRole::Fillet(0)))
                .unwrap()
                .surface,
            SurfaceKind::Freeform
        ));
    }

    /// The direction the radius is measured along points into the body at a convex edge
    /// and out of it at a concave one: towards the centre of the blend either way.
    #[test]
    fn blend_direction_follows_the_material() {
        let c = cube();
        let key = edge_between(FaceRole::EndCap, FaceRole::Side(0));
        let edge = c.edges().into_iter().find(|e| e.key == key).unwrap();
        let seg = edge.segments[0];
        let dir = blend_direction(&seg).unwrap();
        // Half a step along it from the edge is inside a solid cube.
        let midpoint = (seg.start + seg.end) * 0.5;
        let inside = midpoint + dir;
        assert!(
            inside.x > 0.0 && inside.x < 10.0 && inside.y > 0.0 && inside.y < 10.0,
            "{inside}"
        );
        assert!(inside.z < 10.0, "{inside}");
        assert!(
            dir.dot(seg.normal_a + seg.normal_b) < 0.0,
            "and so away from the outward bisector it used to be drawn along"
        );
    }

    fn rimmed_cylinder(segment_angle_deg: f64) -> Solid {
        crate::primitives::cylinder(
            OpId::new(1),
            Vec3::ZERO,
            Vec3::Z,
            5.0,
            10.0,
            &Tessellation {
                chord_tolerance: 0.01,
                max_segment_angle: segment_angle_deg.to_radians(),
            },
        )
    }

    fn rim(role: FaceRole) -> EdgeKey {
        EdgeKey::new(
            FaceKey::new(OpId::new(1), role),
            FaceKey::new(OpId::new(1), FaceRole::Side(0)),
        )
    }

    /// The tool's facet count is the rim's polyline length times the arc's, and the BSP
    /// boolean is quadratic in it, so a fine body plus a fine arc is how a radius box
    /// takes the machine down. The arc gives way first: the blend is still there, still
    /// closed and still the right size, only faceted.
    #[test]
    fn a_blend_too_fine_for_the_budget_is_coarsened_rather_than_run() {
        let cyl = rimmed_cylinder(3.0);
        let rim_segments = cyl
            .edges()
            .iter()
            .find(|e| e.key == rim(FaceRole::EndCap))
            .unwrap()
            .segments
            .len();
        let r = fillet(OpId::new(2), &cyl, &[rim(FaceRole::EndCap)], 1.0, &fine()).unwrap();
        assert!(r.is_closed(), "{:?}", r.validate());

        let blend = r
            .face(FaceKey::new(OpId::new(2), FaceRole::Fillet(0)))
            .unwrap();
        // `fine()` asks for 56 facets across the arc, which on this rim would be one quad
        // per rim segment per facet: well past the budget before the body is counted.
        assert!(
            rim_segments * 56 > MAX_TOOL_POLYGONS,
            "the fixture stopped exercising the budget"
        );
        assert!(
            blend.polygons.len() < MAX_TOOL_POLYGONS,
            "the arc was not coarsened: {} quads",
            blend.polygons.len()
        );
        assert!(
            blend.polygons.len() > rim_segments * MIN_ARC_SEGMENTS,
            "and the arc must not have collapsed to the coarsest one going"
        );

        // Pappus, as in `fillet_curved_edge_of_cylinder`: the removed ring is the corner
        // region (area 1 − π/4, centroid 0.777 in from the corner) swept round the rim.
        let expected = cyl.volume() - (1.0 - PI / 4.0) * 2.0 * PI * (5.0 - 0.777);
        assert_relative_eq!(r.volume(), expected, epsilon = 1.0);
        // A fillet only ever takes material off this rim, so the body's extent is intact.
        assert_relative_eq!(r.aabb().min.z, 0.0, epsilon = 1e-9);
        assert_relative_eq!(r.aabb().max.z, 10.0, epsilon = 1e-9);
        assert_relative_eq!(r.aabb().max.x, cyl.aabb().max.x, epsilon = 1e-9);
    }

    /// Past the point where even the coarsest arc fits, there is nothing left to give up
    /// and the feature has to say so rather than spend the session's time finding out.
    #[test]
    fn a_blend_that_cannot_be_coarsened_far_enough_is_refused() {
        let cyl = rimmed_cylinder(0.2);
        let err = fillet(OpId::new(2), &cyl, &[rim(FaceRole::EndCap)], 1.0, &fine()).unwrap_err();
        // The body alone is most of the feature's budget here, so it is the feature that
        // refuses, and the count it quotes is the whole feature at its coarsest.
        assert!(
            matches!(
                err,
                KernelError::BlendTooDense { needed, budget }
                    if budget == MAX_FEATURE_POLYGONS && needed > cyl.polygon_count()
            ),
            "{err:?}"
        );
    }

    /// The budget is decided from an estimate of the tool's facet count, made before the
    /// tool is built; a tool bigger than its estimate would put the feature over what it
    /// was told it could afford. Checked on an open chain, whose end caps are the part
    /// the estimate has to allow for, and on a closed one, which has none.
    #[test]
    fn a_tool_fits_the_budget_it_was_given() {
        let c = cube();
        let cube_edges = c.edges();
        let open = cube_edges
            .iter()
            .find(|e| e.key == edge_between(FaceRole::EndCap, FaceRole::Side(0)))
            .unwrap()
            .chains()
            .remove(0);
        let cyl = rimmed_cylinder(3.0);
        let cyl_edges = cyl.edges();
        let closed = cyl_edges
            .iter()
            .find(|e| e.key == rim(FaceRole::EndCap))
            .unwrap()
            .chains()
            .remove(0);
        assert!(!is_closed(&open) && is_closed(&closed), "the fixtures");
        for (chain, key) in [
            (&open, edge_between(FaceRole::EndCap, FaceRole::Side(0))),
            (&closed, rim(FaceRole::EndCap)),
        ] {
            for budget in [
                tool_polygons(ring_count(chain), MIN_ARC_SEGMENTS),
                tool_polygons(ring_count(chain), 7),
                MAX_TOOL_POLYGONS,
            ] {
                let (tool, _) = tool_for_chain(
                    OpId::new(2),
                    0,
                    key,
                    chain,
                    Blend::Fillet { radius: 1.0 },
                    &fine(),
                    budget,
                )
                .unwrap();
                assert!(
                    tool.polygon_count() <= budget,
                    "{} facets for a budget of {budget} on a chain of {} ({})",
                    tool.polygon_count(),
                    chain.len(),
                    if is_closed(chain) { "closed" } else { "open" }
                );
            }
        }
    }

    /// Two rims on a body that leaves room for both only at a coarser arc than either
    /// tool's own budget would allow: the feature shares what is left rather than
    /// building each tool to its own limit and then refusing the pair.
    #[test]
    fn the_feature_budget_is_shared_between_the_tools() {
        let cyl = rimmed_cylinder(1.0);
        let rim_segments = cyl
            .edges()
            .iter()
            .find(|e| e.key == rim(FaceRole::EndCap))
            .unwrap()
            .segments
            .len();
        // The premise: each tool at the limit of its own budget would fit, but two of
        // them beside the body would not.
        let own_limit = MAX_TOOL_POLYGONS / rim_segments - 4;
        assert!(
            own_limit > MIN_ARC_SEGMENTS,
            "the fixture has nothing to share"
        );
        assert!(
            cyl.polygon_count() + 2 * tool_polygons(rim_segments, own_limit) > MAX_FEATURE_POLYGONS,
            "the fixture stopped exercising the feature budget"
        );
        let r = fillet(
            OpId::new(2),
            &cyl,
            &[rim(FaceRole::EndCap), rim(FaceRole::StartCap)],
            1.0,
            &fine(),
        )
        .unwrap();
        assert!(r.is_closed(), "{:?}", r.validate());
        for n in 0..2 {
            let blend = r
                .face(FaceKey::new(OpId::new(2), FaceRole::Fillet(n)))
                .unwrap();
            // The boolean fragments the facets, so count facets by their normals: one
            // per arc step per rim segment, and a fragment shares its parent's.
            let facets = blend
                .polygons
                .iter()
                .map(|p| {
                    let n = p.plane.normal * 1e6;
                    (n.x.round() as i64, n.y.round() as i64, n.z.round() as i64)
                })
                .collect::<std::collections::HashSet<_>>()
                .len();
            assert!(
                facets < rim_segments * own_limit,
                "rim {n} was built to its own limit, not its share: {facets} facets"
            );
            assert!(facets >= rim_segments * MIN_ARC_SEGMENTS, "{facets} facets");
        }
        // Both rims are gone: two corner rings swept round, as in the single-rim test.
        // A coarse arc is chords inside the true one, so the tool takes a little more
        // than the exact ring, and the result sits below the figure, not around it.
        let ring = (1.0 - PI / 4.0) * 2.0 * PI * (5.0 - 0.777);
        let exact = cyl.volume() - 2.0 * ring;
        assert!(
            r.volume() < exact && r.volume() > exact - 4.0,
            "{} against {exact}",
            r.volume()
        );
    }

    #[test]
    fn missing_edge_is_reported() {
        let c = cube();
        let bogus = EdgeKey::new(
            FaceKey::new(OpId::new(7), FaceRole::EndCap),
            FaceKey::new(OpId::new(7), FaceRole::Side(0)),
        );
        assert_eq!(
            fillet(OpId::new(2), &c, &[bogus], 1.0, &fine()),
            Err(KernelError::MissingEdge(bogus))
        );
        assert_eq!(
            chamfer(OpId::new(2), &c, &[], 0.0),
            Err(KernelError::NonPositiveBlend)
        );
    }

    /// Every case below turns on the same claim: folding the tools together leaves
    /// exactly what applying them one at a time left. Volume is the sharp end of it —
    /// a fold that lost a cut or made one twice moves it — and closure is the other,
    /// because the shell a broken fold leaves is one the healer cannot pair up.
    ///
    /// Here, a chain that wraps a closed rim, and two of them: the fold by
    /// concatenation, two closed shells with a gap between them handed to the BSP as one.
    #[test]
    fn folding_two_rims_leaves_what_two_features_would_have_left() {
        let cyl = rimmed_cylinder(10.0);
        let tess = Tessellation::default();
        let folded = fillet(
            OpId::new(2),
            &cyl,
            &[rim(FaceRole::EndCap), rim(FaceRole::StartCap)],
            1.0,
            &tess,
        )
        .unwrap();
        let one = fillet(OpId::new(2), &cyl, &[rim(FaceRole::EndCap)], 1.0, &tess).unwrap();
        let two = fillet(OpId::new(3), &one, &[rim(FaceRole::StartCap)], 1.0, &tess).unwrap();
        assert!(folded.is_closed(), "{:?}", folded.validate());
        assert!(folded.validate().is_ok(), "{:?}", folded.validate());
        assert_relative_eq!(folded.volume(), two.volume(), epsilon = 1e-6);
        assert!(
            folded
                .face(FaceKey::new(OpId::new(2), FaceRole::Fillet(0)))
                .is_some()
                && folded
                    .face(FaceKey::new(OpId::new(2), FaceRole::Fillet(1)))
                    .is_some(),
            "both blends are named, and separately"
        );
    }

    /// Tools that do meet — four fillets running into each other at the corners of a
    /// box, each tool's box straddling the top face, a side and both ends — and whose
    /// fold is therefore a boolean, or nothing at all.
    #[test]
    fn folding_tools_that_meet_at_a_corner_leaves_what_applying_them_in_turn_left() {
        let c = cube();
        let keys: Vec<EdgeKey> = (0..4)
            .map(|i| edge_between(FaceRole::EndCap, FaceRole::Side(i)))
            .collect();
        let folded = fillet(OpId::new(2), &c, &keys, 2.0, &fine()).unwrap();
        // The same tools, applied one at a time as they were before the fold. Not four
        // separate fillet features: those would each be built against the edges the last
        // one left, which is a different shape and not what the fold has to reproduce.
        let mut one_at_a_time = c.clone();
        for (tool, op) in tools_for(
            OpId::new(2),
            &c,
            &keys,
            Blend::Fillet { radius: 2.0 },
            &fine(),
        )
        .unwrap()
        {
            one_at_a_time = boolean(&one_at_a_time, &tool, op).unwrap();
        }
        assert!(folded.is_closed(), "{:?}", folded.validate());
        assert!(folded.validate().is_ok(), "{:?}", folded.validate());
        assert_relative_eq!(folded.volume(), one_at_a_time.volume(), epsilon = 1e-6);
    }

    /// The material-adding path, folded: two concave edges on opposite sides of a boss,
    /// far enough apart to be concatenated, whose tools are unions rather than
    /// subtractions.
    #[test]
    fn folding_two_concave_edges_adds_what_two_features_would_have_added() {
        let base = cuboid(OpId::new(1), Vec3::ZERO, Vec3::splat(10.0));
        let boss = cuboid(
            OpId::new(2),
            Vec3::new(3.0, 3.0, 10.0),
            Vec3::new(7.0, 7.0, 14.0),
        );
        let body = boolean(&base, &boss, BoolOp::Union).unwrap();
        let top = FaceKey::new(OpId::new(1), FaceRole::EndCap);
        let keys: Vec<EdgeKey> = [0u32, 2]
            .iter()
            .map(|i| {
                let side = FaceKey::new(OpId::new(2), FaceRole::Side(*i));
                body.edges()
                    .into_iter()
                    .map(|e| e.key)
                    .find(|k| k.touches(top) && k.touches(side))
                    .expect("the boss meets the top face along each of its sides")
            })
            .collect();
        let folded = fillet(OpId::new(3), &body, &keys, 1.0, &fine()).unwrap();
        let one = fillet(OpId::new(3), &body, &keys[..1], 1.0, &fine()).unwrap();
        let two = fillet(OpId::new(4), &one, &keys[1..], 1.0, &fine()).unwrap();
        assert!(folded.is_closed(), "{:?}", folded.validate());
        assert!(folded.validate().is_ok(), "{:?}", folded.validate());
        assert_relative_eq!(folded.volume(), two.volume(), epsilon = 1e-6);
        // Each fillet fills a quarter-circle gusset four long: (1 − π/4) · 4 apiece.
        let added = 2.0 * (1.0 - PI / 4.0) * 4.0;
        assert_relative_eq!(folded.volume(), body.volume() + added, epsilon = 0.05);
    }

    fn brick(op: u64, min: Vec3) -> Solid {
        cuboid(OpId::new(op), min, min + Vec3::splat(2.0))
    }

    #[test]
    fn tools_that_cannot_reach_each_other_are_folded_without_a_boolean() {
        let tools = vec![
            (brick(1, Vec3::ZERO), BoolOp::Subtract),
            (brick(2, Vec3::new(5.0, 0.0, 0.0)), BoolOp::Subtract),
        ];
        // A body of one polygon, so the size guard rules out any boolean fold: what
        // comes back can only have been concatenated.
        let merged = merge_tools(tools, 1);
        assert_eq!(merged.len(), 1);
        assert_eq!(merged[0].1, BoolOp::Subtract);
        assert_eq!(
            merged[0].0.faces.len(),
            12,
            "two shells, neither one healed away"
        );
        assert_relative_eq!(merged[0].0.volume(), 16.0, epsilon = 1e-9);
        assert!(merged[0].0.is_closed());
    }

    #[test]
    fn tools_that_overlap_are_folded_into_their_union_when_the_body_is_the_bigger_problem() {
        let tools = vec![
            (brick(1, Vec3::ZERO), BoolOp::Subtract),
            (brick(2, Vec3::splat(1.0)), BoolOp::Subtract),
        ];
        let merged = merge_tools(tools.clone(), 10_000);
        assert_eq!(merged.len(), 1);
        assert_relative_eq!(merged[0].0.volume(), 8.0 + 8.0 - 1.0, epsilon = 1e-9);
        assert!(merged[0].0.is_closed());
        // The same pair against a body smaller than they are stays apart, because the
        // boolean saved would cost more than the boolean spent.
        assert_eq!(merge_tools(tools, 8).len(), 2);
    }

    /// Subtracting and adding do not commute, so the fold may reorder tools only within
    /// a run of one operation.
    #[test]
    fn a_subtraction_and_a_union_keep_their_order_through_the_fold() {
        let tools = vec![
            (brick(1, Vec3::ZERO), BoolOp::Subtract),
            (brick(2, Vec3::new(5.0, 0.0, 0.0)), BoolOp::Union),
            (brick(3, Vec3::new(10.0, 0.0, 0.0)), BoolOp::Subtract),
            (brick(4, Vec3::new(15.0, 0.0, 0.0)), BoolOp::Subtract),
        ];
        let merged = merge_tools(tools, 1);
        let ops: Vec<BoolOp> = merged.iter().map(|(_, o)| *o).collect();
        assert_eq!(
            ops,
            vec![BoolOp::Subtract, BoolOp::Union, BoolOp::Subtract],
            "the two subtractions either side of the union are not brought together"
        );
        assert_eq!(
            merged[2].0.faces.len(),
            12,
            "the trailing run is folded, though"
        );
    }

    // --- Range ------------------------------------------------------------------------

    /// The L-block of [`fillet_concave_edge_adds_material`], and the concave edge between
    /// the pocket's floor and its far wall. The floor reaches 5 mm from that edge to the
    /// front of the block and the wall reaches 5 mm up it to the top.
    fn l_block() -> (Solid, EdgeKey) {
        let notch = cuboid(
            OpId::new(9),
            Vec3::new(-1.0, -1.0, 5.0),
            Vec3::new(11.0, 5.0, 11.0),
        );
        let key = EdgeKey::new(
            FaceKey::new(OpId::new(9), FaceRole::StartCap),
            FaceKey::new(OpId::new(9), FaceRole::Side(2)),
        );
        (boolean(&cube(), &notch, BoolOp::Subtract).unwrap(), key)
    }

    /// A fillet whose setback runs past the far side of a face has eaten the face, and
    /// what the boolean then takes away is whatever lay behind it. On a 10 mm cube both
    /// faces of a top edge are 10 mm across and the dihedral is a right angle, so the
    /// setback is the radius and the limit is 10 mm exactly.
    #[test]
    fn a_fillet_wider_than_the_faces_it_sits_between_is_refused() {
        let c = cube();
        let key = edge_between(FaceRole::EndCap, FaceRole::Side(0));
        let limit = max_fillet_radius(&c, &[key]).unwrap();
        assert_relative_eq!(limit, 10.0, epsilon = 1e-9);
        let err = fillet(OpId::new(2), &c, &[key], 12.0, &fine()).unwrap_err();
        assert!(
            matches!(err, KernelError::BlendTooLarge { size, limit }
                     if size == 12.0 && (limit - 10.0).abs() < 1e-9),
            "{err:?}"
        );
    }

    /// Round a closed rim the blend meets itself: the ray across the cap from one rim
    /// segment leaves through the rim on the far side, and that far side is coming this
    /// way at the same rate, so the radius is bounded by the cylinder's own radius —
    /// which is also where the tool's centre circle, of radius R − r, collapses.
    #[test]
    fn a_fillet_that_would_close_a_rim_on_itself_is_refused() {
        let cyl = rimmed_cylinder(10.0);
        let key = rim(FaceRole::EndCap);
        let limit = max_fillet_radius(&cyl, &[key]).unwrap();
        assert!(limit < 5.0, "a rim of radius 5 cannot take {limit}");
        assert!(
            limit > 4.9,
            "and the faceted rim is barely inside it, not {limit}"
        );
        assert!(matches!(
            fillet(OpId::new(2), &cyl, &[key], 6.0, &Tessellation::default()).unwrap_err(),
            KernelError::BlendTooLarge { .. }
        ));
    }

    /// The concave case, where the tool is added rather than taken away: past the limit
    /// the fillet's landing on the pocket floor is beyond the front of the block, and
    /// the union raises a wall standing in mid-air where the pocket used to open out.
    #[test]
    fn a_fillet_that_would_overflow_a_pocket_is_refused() {
        let (l, key) = l_block();
        let limit = max_fillet_radius(&l, &[key]).unwrap();
        assert_relative_eq!(limit, 5.0, epsilon = 1e-9);
        assert!(matches!(
            fillet(OpId::new(3), &l, &[key], 6.0, &fine()).unwrap_err(),
            KernelError::BlendTooLarge { .. }
        ));
    }

    /// Two edges of one face blended together divide the room between them rather than
    /// each claiming all of it, so four edges round the top of a cube stop at half what
    /// one of them alone could take.
    #[test]
    fn edges_blended_together_share_the_face_between_them() {
        let c = cube();
        let keys: Vec<EdgeKey> = (0..4)
            .map(|i| edge_between(FaceRole::EndCap, FaceRole::Side(i)))
            .collect();
        assert_relative_eq!(max_fillet_radius(&c, &keys).unwrap(), 5.0, epsilon = 1e-9);
        assert!(matches!(
            fillet(OpId::new(2), &c, &keys, 6.0, &fine()).unwrap_err(),
            KernelError::BlendTooLarge { .. }
        ));
    }

    /// A chamfer's setback is its distance, so it is bounded by the room itself rather
    /// than by the room turned back through the dihedral angle.
    #[test]
    fn a_chamfer_past_the_face_it_cuts_is_refused() {
        let c = cube();
        let key = edge_between(FaceRole::EndCap, FaceRole::Side(0));
        assert_relative_eq!(
            max_chamfer_distance(&c, &[key]).unwrap(),
            10.0,
            epsilon = 1e-9
        );
        assert!(matches!(
            chamfer(OpId::new(2), &c, &[key], 11.0).unwrap_err(),
            KernelError::BlendTooLarge { .. }
        ));
        assert!(chamfer(OpId::new(2), &c, &[key], 4.0).is_ok());
    }

    /// The limit is the largest blend that works, not the largest that is comfortable:
    /// every one of the shapes above takes the radius it reports and still comes out a
    /// closed, valid solid.
    #[test]
    fn the_largest_radius_the_limit_allows_still_makes_a_solid() {
        let (l, pocket) = l_block();
        let cases: Vec<(&str, Solid, Vec<EdgeKey>)> = vec![
            (
                "cube edge",
                cube(),
                vec![edge_between(FaceRole::EndCap, FaceRole::Side(0))],
            ),
            (
                "cube top",
                cube(),
                (0..4)
                    .map(|i| edge_between(FaceRole::EndCap, FaceRole::Side(i)))
                    .collect(),
            ),
            (
                "cylinder rim",
                rimmed_cylinder(10.0),
                vec![rim(FaceRole::EndCap)],
            ),
            ("pocket", l, vec![pocket]),
        ];
        for (name, solid, keys) in cases {
            let limit = max_fillet_radius(&solid, &keys).unwrap();
            let r = fillet(OpId::new(4), &solid, &keys, limit, &Tessellation::default())
                .unwrap_or_else(|e| panic!("{name} at its own limit of {limit}: {e}"));
            assert!(r.is_closed(), "{name}: {:?}", r.validate());
            assert!(r.validate().is_ok(), "{name}: {:?}", r.validate());
            assert!(r.volume() > 0.0, "{name} has no volume left");
        }
    }

    /// The limit that rejects a fillet a machinist would ask for is a worse bug than the
    /// one it fixes, so the ordinary sizes stay ordinary: a fifth of the face on a cube,
    /// a fifth of the radius on a rim, and the same again with every edge of the face
    /// taken at once.
    #[test]
    fn a_fillet_well_inside_the_material_is_not_refused() {
        let c = cube();
        let key = edge_between(FaceRole::EndCap, FaceRole::Side(0));
        assert!(max_fillet_radius(&c, &[key]).unwrap() >= 2.0);
        assert!(fillet(OpId::new(2), &c, &[key], 2.0, &fine()).is_ok());
        let keys: Vec<EdgeKey> = (0..4)
            .map(|i| edge_between(FaceRole::EndCap, FaceRole::Side(i)))
            .collect();
        assert!(fillet(OpId::new(2), &c, &keys, 2.0, &fine()).is_ok());

        let cyl = rimmed_cylinder(10.0);
        assert!(max_fillet_radius(&cyl, &[rim(FaceRole::EndCap)]).unwrap() >= 1.0);
        assert!(
            fillet(
                OpId::new(2),
                &cyl,
                &[rim(FaceRole::EndCap), rim(FaceRole::StartCap)],
                1.0,
                &Tessellation::default(),
            )
            .is_ok()
        );

        let (l, pocket) = l_block();
        assert!(fillet(OpId::new(3), &l, &[pocket], 2.0, &fine()).is_ok());
    }

    /// A thin wall is bounded by its thickness, because the setback down the wall's own
    /// face is what has to fit; a 2 mm plate takes a 2 mm fillet on its rim and no more,
    /// whatever the 50 mm faces either side of it would otherwise allow.
    #[test]
    fn a_thin_wall_bounds_the_fillet_that_rounds_its_rim() {
        let plate = cuboid(OpId::new(1), Vec3::ZERO, Vec3::new(50.0, 50.0, 2.0));
        let key = edge_between(FaceRole::EndCap, FaceRole::Side(0));
        assert_relative_eq!(
            max_fillet_radius(&plate, &[key]).unwrap(),
            2.0,
            epsilon = 1e-9
        );
    }
}
