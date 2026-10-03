//! Fillet and chamfer.
//!
//! Both are built the same way: for every selected edge, construct a prismatic *tool*
//! whose cross-section is the material to remove (convex edge) or add (concave edge),
//! and apply it with a boolean. A fillet's radius is signed: positive is the round
//! everyone expects, negative is the *inverted* round — the complementary quarter-disc,
//! centred on the edge rather than tangent to the faces — which cuts a cove into a convex
//! edge and lays a bead into a concave one. The same handle dragged the other way, as
//! Fusion's Press/Pull does it; see [`section`] for the geometry. The cross-section is
//! exact in the plane perpendicular to the edge; along curved edges the prism is mitred
//! at every polyline joint so consecutive pieces meet on the bisecting plane. Corners
//! where several filleted edges meet are simply the intersection of their tools, which is
//! a crease rather than the spherical patch a full-featured kernel would make — an
//! accepted MVP limitation.
//!
//! Compound bodies — a boss joined onto a face, a hole cut through, blocks unioned into a
//! T — are where the edges stop being simple, and the chain is shaped for them. A run of
//! selected edges that continue one another tangentially (a straight edge a seam cut into
//! two keys, a slot's straights and arcs) is one chain and one tool, mitred through the
//! joins, each edge keeping its own blend face ([`join_tangent_runs`]). Where a chain
//! ends decides how its tool ends: it runs a clearance past an end that opens into air
//! (a cube's corner) and stops a clearance short of one that does not — the foot of a
//! wall at the inside corner of the T, the top face a concave edge climbs to — so no
//! tool notches a face it was not asked to touch ([`ChainEnd`]). The tools of one
//! feature are applied beads first, then rounds, whatever order the edges were picked
//! in. Three shapes are refused by name rather than built badly: an edge whose faces
//! fold by less than the kernel's tangent threshold (a fillet's own run-out, which the
//! editor can select but nothing can round), an edge whose dihedral angle changes along
//! it (a cylinder cut off askew), and a round whose end meets a bead in the same feature,
//! which wants the corner blend this kernel does not have. The arc along a chain that
//! turns is drawn no finer than [`MIN_CURVED_FACET_ANGLE`], for the boolean's sake.
//!
//! What a blend costs is decided here rather than in the boolean. The tools of one
//! feature are folded into as few solids as they can be before any of them meets the
//! body ([`merge_tools`]), because a boolean is superlinear in the body it is given and
//! every tool applied on its own hands the next one a body it has fragmented: filleting
//! both rims of a 536-facet cylinder measured 165 ms for the first tool and 1 318 ms for
//! the second, identical one. The other half is the budget, which decides how many
//! facets the tools may carry at all; see [`MAX_FEATURE_POLYGONS`].
//!
//! How *large* a blend may be is decided here as well, and for the same reason: nothing
//! below this module notices that a tool has outgrown the material it was meant to work
//! in. See [`size_limit`].

use basset_math::{Aabb, Vec3};

use crate::csg::{BoolOp, boolean};
use crate::error::KernelError;
use crate::geometry::Tessellation;
use crate::ids::{EdgeKey, FaceKey, FaceRole, OpId};
use crate::solid::{
    Edge, EdgeSegment, MERGE_TOL, Solid, SolidBuilder, SurfaceKind, TANGENT_EDGE_COS, newell_normal,
};

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

/// Finest arc a fillet draws along a chain that *turns*: no facet narrower than this.
///
/// A fillet's end facets are tangent to the faces they blend into, so each lies within
/// half a facet's angle of the face plane, and the boolean has to cut the face along it.
/// Along a straight edge every ring puts that facet in one and the same plane and the
/// cut is one clean line. Along a curved edge each pair of rings has its own, and planes
/// a degree or two off the face classify a strip of it some ten-thousandths wide as
/// coplanar (the BSP's tolerance over the sine of the tilt); neighbouring strips do not
/// agree about where the face ends, and the shell comes back with slivers it cannot pair
/// up. Measured on the arc where a round meets an end face: 22 facets to the quarter
/// circle (a 4° end facet) close, 30 (3°) leak; a hole's rim is touchier (see below).
/// The floor keeps the end facet 5° off the face, and a nine-facet quarter round shaded
/// smoothly still reads as a round. Ten degrees, not nine: nine arc facets to the quarter
/// close on a hole's rim at every drill tessellation tried (36 to 180 facets), ten leak
/// on two of them, and the default tessellation draws 10° facets anyway, so nothing a
/// user sees by default changes.
const MIN_CURVED_FACET_ANGLE: f64 = 10.0 * std::f64::consts::PI / 180.0;

/// Facets of the swept tool: one ring of section vertices per chain segment. A section
/// carries the arc's points, its two tangent points and three scaffolding corners. An
/// open chain's end caps are fans over one ring each, so they are counted as two more.
fn tool_polygons(rings: usize, arc_segments: usize) -> usize {
    rings * (arc_segments + 4)
}

/// One straight piece of a chain, with the selected edge it belongs to: `index` is that
/// edge's position in the feature's list and names the blend face the piece ends up on.
///
/// A chain is a connected run of these. It usually comes from one edge, but a run that
/// continues tangentially from one selected edge onto the next — a straight edge a seam
/// has cut into two keys, the straights and arcs of a slot's outline — is one chain too,
/// so the tool is mitred through the join rather than two tools overhanging each other
/// there ([`join_tangent_runs`]).
#[derive(Clone, Copy)]
struct Link {
    seg: EdgeSegment,
    index: u32,
    key: EdgeKey,
}

/// Whether a chain closes on itself: its end caps are then unnecessary and not built.
fn is_closed(chain: &[Link]) -> bool {
    chain.len() > 1
        && chain[0]
            .seg
            .start
            .distance_squared(chain.last().unwrap().seg.end)
            < MERGE_TOL * MERGE_TOL
}

/// Rings a chain's tool is built from, caps included.
fn ring_count(chain: &[Link]) -> usize {
    if is_closed(chain) {
        chain.len()
    } else {
        chain.len() + 2
    }
}

/// How the tool finishes at an end of an open chain.
///
/// The tool's own end cap must not lie in a face of the body: a coplanar face is what the
/// boolean is worst at, and the cap of a tool stopped exactly where its edge stops lies
/// in whatever face the edge stops against. So the tool either runs past the end or
/// stops short of it, by [`CLEARANCE`], and which is right depends on what is there.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ChainEnd {
    /// The run-on changes nothing: a subtracted tool reaching into the air beyond the
    /// face the edge leaves through, an added one reaching into the wall it ends against.
    /// This also closes the hairline gap where a chain ends flush with another face.
    Overhang,
    /// The run-on would cut a notch into material or stand a sliver of material in the
    /// air: a convex edge ending at the foot of a wall (the inside corner of a T, a boss
    /// standing on a face), a concave edge ending at the face it climbs to, or an edge
    /// that runs tangentially on into one the user did not pick. The tool stops
    /// [`SHORT`] before the end instead, leaving that much of the edge as it was.
    Short,
}

/// How far a [`ChainEnd::Short`] tool stops before its edge's end. Twice [`CLEARANCE`],
/// and not once, because of the tool that may be waiting there: at the inside corner of a
/// T both top edges stop short of the same vertex, and each tool's scaffolding runs a
/// clearance *past* its own faces — into the very space the other tool's end cap would
/// occupy if it stopped a clearance short. The two were then coplanar there, cap against
/// scaffold, and the boolean folding the tools together doubled the polygons. At two
/// clearances the caps clear the scaffolds with a clearance to spare, so two short-ended
/// tools meeting at a corner share no point at all and are folded by concatenation.
const SHORT: f64 = 2.0 * CLEARANCE;

#[derive(Clone, Copy)]
enum Blend {
    /// Signed: negative is the inverted round, see the module docs.
    Fillet { radius: f64 },
    /// Unsigned. The triangle a chamfer takes off an edge has its apex *on* the edge, so
    /// the chamfer already is its own "inverted" form — mirroring the section about the
    /// edge gives the same triangle back — and there is no complementary shape for a sign
    /// to select. A negative distance is refused like a zero one.
    Chamfer { distance: f64 },
}

impl Blend {
    fn kind(self) -> BlendKind {
        match self {
            Blend::Fillet { radius } if radius < 0.0 => BlendKind::InvertedFillet,
            Blend::Fillet { .. } => BlendKind::Fillet,
            Blend::Chamfer { .. } => BlendKind::Chamfer,
        }
    }

    /// The blend's size without its sign: what is held against the material and what the
    /// arc is drawn with. A chamfer's distance comes back as it is, sign and all, so a
    /// negative one still fails the positivity check.
    fn size(self) -> f64 {
        match self {
            Blend::Fillet { radius } => radius.abs(),
            Blend::Chamfer { distance } => distance,
        }
    }
}

/// Rounds `edges` of `solid` with `radius`. Negative inverts the round: the section is
/// the quarter-disc about the edge rather than the one tangent to the faces, so a convex
/// edge gets a cove and a concave one a bead. Zero is refused.
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
    let mut tools = tools_for(op, solid, keys, blend, tess)?;
    // Added material first, then the cuts, whatever order the edges were picked in. Where
    // a bead and a round touch — a concave edge climbing to a face whose edge is rounded
    // in the same feature — the two tools overlap by their clearances, and the order
    // decides what is left there: a round cut through the bead's clearance sliver takes
    // it away, a bead laid after the round leaves that sliver standing as a fin on the
    // new surface. Same op either way, so one order serves and the result does not
    // depend on how the user clicked.
    tools.sort_by_key(|(_, bool_op)| *bool_op != BoolOp::Union);
    let mut result = solid.clone();
    for (tool, bool_op) in merge_tools(tools, solid.polygon_count()) {
        result = boolean(&result, &tool, bool_op)?;
    }
    Ok(result)
}

/// One tool per chain of the selected edges, with the operation that applies it.
fn tools_for(
    op: OpId,
    solid: &Solid,
    keys: &[EdgeKey],
    blend: Blend,
    tess: &Tessellation,
) -> Result<Vec<(Solid, BoolOp)>, KernelError> {
    let size = blend.size();
    if !size.is_finite() || size <= 0.0 {
        return Err(KernelError::NonPositiveBlend);
    }
    // Tools are built from the original edges, before any of them is blended away, so
    // the result does not depend on the order edges were selected in.
    let all_edges = solid.edges();
    let mut chains: Vec<Vec<Link>> = Vec::new();
    for (i, key) in keys.iter().enumerate() {
        let edge = all_edges
            .iter()
            .find(|e| e.key == *key)
            .ok_or(KernelError::MissingEdge(*key))?;
        for chain in edge.chains() {
            // A piece that does not fold has no corner to fill, and a section built on
            // it is a sliver whose planes lie within rounding of the body's own. The run-
            // out of an earlier fillet is the usual way one arrives here: it is a face
            // boundary the user can click, half a facet from flat.
            if chain
                .iter()
                .any(|s| s.normal_a.dot(s.normal_b) >= TANGENT_EDGE_COS)
            {
                return Err(KernelError::TangentEdge(*key));
            }
            // The section is drawn per segment from that segment's two facet normals,
            // and a joint ring is the incoming section carried onto the bisecting plane.
            // While the dihedral is constant the tangent points of consecutive rings on a
            // faceted face lie on that face's seams, and the band runs cleanly from
            // facet to facet. When it is not — a cylinder cut off askew, whose rim meets
            // the cut at 70° on one side and 110° on the other — the in-face direction
            // leans across the seams, each ring's tangent point lands in its own facet's
            // plane a step off the seam, and the band's edge zigzags through the body's
            // facets. Every density tried leaked. A joint section built from the seam
            // itself would fix it; until then the edge is refused by name.
            let (lo, hi) = chain.iter().fold((f64::INFINITY, 0.0f64), |(lo, hi), s| {
                let phi = dihedral(s).3;
                (lo.min(phi), hi.max(phi))
            });
            if hi - lo > DIHEDRAL_DRIFT {
                return Err(KernelError::VaryingDihedral(*key));
            }
            // One tool is one operation, so a chain is split wherever its convexity
            // changes. Between two smooth faces that cannot happen without passing
            // through tangency, which was refused above; it is here for the faceted
            // surface whose neighbouring facets fold the other way.
            let mut run: Vec<Link> = Vec::new();
            for seg in chain {
                if run
                    .last()
                    .is_some_and(|prev| is_convex(&prev.seg) != is_convex(&seg))
                {
                    chains.push(std::mem::take(&mut run));
                }
                run.push(Link {
                    seg,
                    index: i as u32,
                    key: *key,
                });
            }
            chains.push(run);
        }
    }
    let chains = join_tangent_runs(chains);
    // A round and a bead that meet want a corner blend between them — the round rolling
    // onto the bead's cylinder — and this kernel builds none: it would stop each tool a
    // clearance short of the vertex, and the other tool's faces, a clearance away, then
    // slice that end cap into fragments finer than the healer can pair. Refused by name
    // instead of returning a shell with a hole in it.
    for (i, a) in chains.iter().enumerate() {
        for b in &chains[i + 1..] {
            let (ca, cb) = (is_convex(&a[0].seg), is_convex(&b[0].seg));
            if ca == cb || !chains_touch(a, b) {
                continue;
            }
            let (convex, concave) = if ca == Some(true) { (a, b) } else { (b, a) };
            return Err(KernelError::ConvexMeetsConcave {
                convex: convex[0].key,
                concave: concave[0].key,
            });
        }
    }
    // Whether there is material for it at all, checked after the edges are known to exist
    // so a stale reference is still reported as the stale reference it is.
    if let Some(limit) = size_limit(solid, &all_edges, keys, blend.kind())
        && size > limit
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
        .map(|c| tool_polygons(ring_count(c), MIN_ARC_SEGMENTS))
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
    for chain in &chains {
        let budget =
            (tool_polygons(ring_count(chain), MIN_ARC_SEGMENTS) + slack).min(MAX_TOOL_POLYGONS);
        let ends = chain_ends(chain, &all_edges);
        tools.push(tool_for_chain(op, chain, ends, blend, tess, budget)?);
    }
    Ok(tools)
}

/// How much the dihedral angle may vary along one edge before it is refused: two
/// degrees, well inside what a tessellated rim perpendicular to its axis shows (none) and
/// well outside rounding.
const DIHEDRAL_DRIFT: f64 = 2.0 * std::f64::consts::PI / 180.0;

/// Whether two chains share a point: a vertex of either lies on a segment of the other.
fn chains_touch(a: &[Link], b: &[Link]) -> bool {
    let on = |chain: &[Link], p: Vec3| {
        chain
            .iter()
            .any(|l| distance_to_segment(p, l.seg.start, l.seg.end) <= MERGE_TOL)
    };
    a.iter().any(|l| on(b, l.seg.start) || on(b, l.seg.end))
        || b.iter().any(|l| on(a, l.seg.start) || on(a, l.seg.end))
}

/// Whether the material at a segment is convex there: the tool for it is subtracted.
/// `None` where the faces do not fold at all.
fn is_convex(seg: &EdgeSegment) -> Option<bool> {
    let (_, _, db, _) = dihedral(seg);
    let turn = seg.normal_a.dot(db);
    (turn.abs() >= 1e-6).then_some(turn < 0.0)
}

/// How far two chain directions may disagree at a shared vertex and still be one run:
/// the same 15° the editor's tangent chain walks by ([`crate::pick::tangent_chain`]), so
/// what one click selects is what one tool is built for. A tessellated arc's end chord
/// leaves the true tangent by half a facet, at most 5°.
const JOIN_COS: f64 = 0.966;

/// Joins chains that continue one another tangentially into single chains.
///
/// Two selected edges that meet without a corner — one straight edge a boolean seam has
/// cut into two keys, or the straight and the arc of a slot's outline — are one run of
/// edge to the user, and are one tool here, mitred through the join like any other
/// polyline joint. Built separately they overhang each other: each tool's end cap stands a
/// clearance inside the other's start, a facet's angle off its first ring, and the
/// boolean between the two leaves slivers it cannot heal. Only runs of one convexity are
/// joined (a tool is one operation), and a vertex where more than one selected chain
/// could continue is left alone, as the editor's chain pick leaves a fork alone.
///
/// Sharp turns are *not* joined, though a mitre would be exact for equal radii: two
/// tools built separately keep their own faces and meet in the crease the module
/// documents, and a mitre at a corner whose two edges have different dihedral angles
/// would twist the band between them.
fn join_tangent_runs(mut chains: Vec<Vec<Link>>) -> Vec<Vec<Link>> {
    let start_of = |c: &[Link]| c[0].seg.start;
    let end_of = |c: &[Link]| c.last().unwrap().seg.end;
    let dir_in = |c: &[Link]| (c[0].seg.end - c[0].seg.start).normalize();
    let dir_out = |c: &[Link]| {
        let l = c.last().unwrap().seg;
        (l.end - l.start).normalize()
    };
    let touching = |a: Vec3, b: Vec3| a.distance_squared(b) < MERGE_TOL * MERGE_TOL;
    loop {
        // The first pair of open chains whose ends meet tangentially, with no third
        // chain's end at that vertex. Chains are few, so this is cheap enough to restart
        // after every join.
        let mut found: Option<(usize, usize, bool)> = None;
        'outer: for i in 0..chains.len() {
            if is_closed(&chains[i]) {
                continue;
            }
            let (v, t, convex) = (
                end_of(&chains[i]),
                dir_out(&chains[i]),
                is_convex(&chains[i][0].seg),
            );
            // Other chains with an end at `v`, and whether it is their start (joined as
            // they are) or their end (joined reversed).
            let mut candidates = Vec::new();
            for (j, other) in chains.iter().enumerate() {
                if j == i || is_closed(other) || is_convex(&other[0].seg) != convex {
                    continue;
                }
                if touching(start_of(other), v) {
                    candidates.push((j, false, dir_in(other)));
                }
                if touching(end_of(other), v) {
                    candidates.push((j, true, -dir_out(other)));
                }
            }
            if let [(j, reversed, dir)] = candidates[..]
                && t.dot(dir) >= JOIN_COS
            {
                found = Some((i, j, reversed));
                break 'outer;
            }
        }
        let Some((i, j, reversed)) = found else {
            return chains;
        };
        let mut other = chains.remove(j);
        if reversed {
            other = reverse_chain(other);
        }
        // `i` may have moved down by one when `j` was removed ahead of it.
        let i = if j < i { i - 1 } else { i };
        chains[i].extend(other);
    }
}

/// The same chain walked the other way. A segment runs along face `a`'s winding, so
/// turning it round swaps the roles of the two faces as well as the two ends; the
/// dihedral it describes is unchanged.
fn reverse_chain(chain: Vec<Link>) -> Vec<Link> {
    chain
        .into_iter()
        .rev()
        .map(|l| Link {
            seg: EdgeSegment {
                start: l.seg.end,
                end: l.seg.start,
                normal_a: l.seg.normal_b,
                normal_b: l.seg.normal_a,
            },
            ..l
        })
        .collect()
}

/// A face at a chain's end counts as what the edge stops *against* only if it folds
/// against the edge's direction by more than the kernel's tangent threshold, sin 20°; a
/// face the edge runs along tangentially is one it continues past, not one it ends at.
const END_FACE_SIN: f64 = 0.34;

/// How the tool finishes at each end of `chain`: `[at its start, at its end]`. Both
/// `Overhang` for a closed chain, which has no ends.
fn chain_ends(chain: &[Link], all_edges: &[Edge]) -> [ChainEnd; 2] {
    if is_closed(chain) {
        return [ChainEnd::Overhang; 2];
    }
    let convex = is_convex(&chain[0].seg).unwrap_or(true);
    let first = chain[0];
    let last = *chain.last().unwrap();
    let t_in = (first.seg.end - first.seg.start).normalize();
    let t_out = (last.seg.end - last.seg.start).normalize();
    [
        end_style(first.seg.start, -t_in, convex, first.key, all_edges),
        end_style(last.seg.end, t_out, convex, last.key, all_edges),
    ]
}

/// What lies beyond the vertex `v` where a chain ends, leaving it along `t`.
///
/// The faces meeting at `v` other than the edge's own two are what the edge stops
/// against. Take the one squarest to the edge: if the edge *leaves* through it (`t`
/// along its outward normal) the space beyond is air, and if it *runs into* it (`t`
/// against the normal) the space beyond is material. A subtracted tool may run on into
/// air and an added one into material; either run on the other way would mark the body,
/// so the tool stops short. No face squarer than the tangent threshold means the edge
/// continues tangentially onto something the user did not pick, and the tool stops short
/// of that too. Nothing at all beyond the edge's own faces — which a closed shell does
/// not produce — keeps the overhang, as before.
fn end_style(v: Vec3, t: Vec3, convex: bool, own: EdgeKey, all_edges: &[Edge]) -> ChainEnd {
    let mut squarest: Option<f64> = None;
    for edge in all_edges {
        for seg in &edge.segments {
            if distance_to_segment(v, seg.start, seg.end) > MERGE_TOL {
                continue;
            }
            for (face, normal) in [(edge.key.a, seg.normal_a), (edge.key.b, seg.normal_b)] {
                if face == own.a || face == own.b {
                    continue;
                }
                let along = t.dot(normal);
                if squarest.is_none_or(|s: f64| along.abs() > s.abs()) {
                    squarest = Some(along);
                }
            }
        }
    }
    match squarest {
        None => ChainEnd::Overhang,
        Some(along) if along.abs() < END_FACE_SIN => ChainEnd::Short,
        Some(along) if (along > 0.0) == convex => ChainEnd::Overhang,
        Some(_) => ChainEnd::Short,
    }
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
    InvertedFillet,
    Chamfer,
}

impl BlendKind {
    /// Setback per unit of size, across a face whose dihedral angle at the edge is `phi`.
    ///
    /// A fillet of radius r rolls in the corner with its centre on the bisector and
    /// touches each face where the perpendicular from that centre meets it, which is
    /// r/tan(phi/2) from the edge: the far leg of the right triangle whose near leg is r
    /// and whose angle at the edge is phi/2. It is the same `d` [`section`] lays its
    /// cross-section out with. A chamfer's setback is its distance, by definition, and
    /// so is an inverted fillet's: its arc is centred on the edge and meets each face a
    /// radius out along it, whatever the dihedral angle. At 90° all three agree.
    fn setback_per_size(self, phi: f64) -> f64 {
        match self {
            BlendKind::Fillet => 1.0 / (phi / 2.0).tan(),
            BlendKind::InvertedFillet | BlendKind::Chamfer => 1.0,
        }
    }
}

/// Distance along a ray below which a crossing is the boundary the ray started from
/// rather than one ahead of it: the segment being measured is part of the boundary the
/// ray is cast into, and the ray leaves from a point on it.
///
/// Twice what a tool stopped [`SHORT`] of its edge's end leaves of that edge. A bead laid
/// into the foot of a wall ends that much before the wall's end face, and the sliver of
/// wall between is a face boundary like any other; measured as the room the next edge
/// has, it bounded the round of that edge at two ten-thousandths. It is the corner the
/// ray started from, not the far side of anything.
const OWN_BOUNDARY: f64 = 2.0 * SHORT;

/// How close a ray and a boundary segment come before they count as crossing. Inside a
/// planar face the two are coplanar and the crossing is exact, so this only absorbs
/// rounding; a ray that finds nothing has left a curved surface, and falls back to the
/// nearest boundary under the face's bounding box (see [`reach_in_face`]).
const CROSSES: f64 = 1e-6;

/// Relative slack on the limit. It arrives through a normalise, a dot product and a
/// division, so a blend that fits its material exactly — the 2 mm round on the rim of a
/// 2 mm plate — lands a few ulps either side of it, and neither the check nor the editor
/// showing the number may turn such a blend away over the last bits of that. Carried by
/// the limit itself rather than by each comparison, so every reader agrees on where it is.
const LIMIT_SLACK: f64 = 1e-9;

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
            // is this blend's alone or shared with the one coming the other way, and with
            // the edge it belongs to, so a segment can be told apart from the edge being
            // measured.
            let boundary: Vec<(&EdgeSegment, bool, EdgeKey)> = all_edges
                .iter()
                .filter(|e| e.key.touches(face_key))
                .flat_map(|e| {
                    let blended = keys.contains(&e.key);
                    e.segments.iter().map(move |s| (s, blended, e.key))
                })
                .collect();
            let extent = Aabb::from_points(
                face.polygons
                    .iter()
                    .flat_map(|p| p.vertices.iter().copied()),
            );
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
                // The face is measured from several points along the segment, not its
                // midpoint alone. A straight edge on a planar face is one segment however
                // long it is, so on a tapered face — a trapezoid's long side, a drafted
                // wall — the room at the narrow end is nothing like the room in the
                // middle, and a midpoint ray sails past the very corner the blend will
                // overrun. The ends are sampled just inside the segment because its exact
                // endpoints sit on the neighbouring boundary, where a ray is all
                // `OWN_BOUNDARY` cases.
                for from in sample_points(seg) {
                    let (reach, blocker) = reach_in_face(from, into, edge.key, &boundary, &extent);
                    let there = blocker
                        .map(|p| kind.setback_per_size(p))
                        .filter(|s| s.is_finite() && *s > 0.0)
                        .unwrap_or(0.0);
                    limit = limit.min(reach / (here + there));
                }
            }
        }
        // The two adjacent faces say nothing about what is *behind* them: a wall with a
        // cavity at its back is as roomy as a solid block until the tool breaks through.
        // For a convex edge — where the tool is subtracted — every point of the removed
        // cross-section lies within one setback of the edge, so rays fanned across the
        // wedge between the two inward normals must each find that much material ahead
        // of them, measured against the whole solid rather than the two faces. Slightly
        // stricter than the true cross-section, which only reaches a full setback along
        // the faces themselves, but never looser; the symmetric shapes the exact limits
        // are pinned on all have more room diagonally than along a face, so they are
        // untouched. Concave edges add material and are not bounded here.
        for seg in &edge.segments {
            let (_, _, db, phi) = dihedral(seg);
            let here = kind.setback_per_size(phi);
            if !here.is_finite() || here <= 0.0 || seg.normal_a.dot(db) >= 0.0 {
                continue;
            }
            for from in sample_points(seg) {
                for dir in interior_fan(seg) {
                    if let Some(depth) = ray_into_solid(solid, from, dir) {
                        limit = limit.min(depth / here);
                    }
                }
            }
        }
    }
    limit
        .is_finite()
        .then_some(limit.max(0.0) * (1.0 + LIMIT_SLACK))
}

/// Where along a segment the face is measured: near both ends, the middle, and the
/// quarters. The ends are what catch a tapered face; the quarters catch a boundary —
/// a hole, a notch — that bulges toward the middle third of a long segment without
/// reaching either end or the midpoint. Not the endpoints themselves: those lie on the
/// neighbouring boundary, within [`OWN_BOUNDARY`] of everything a ray from there would
/// have to measure against.
fn sample_points(seg: &EdgeSegment) -> impl Iterator<Item = Vec3> + '_ {
    [0.05, 0.25, 0.5, 0.75, 0.95]
        .into_iter()
        .map(|u| seg.start.lerp(seg.end, u))
}

/// Directions across the wedge of material at a convex edge, between the two in-face
/// directions and strictly inside them. Five is a compromise: a cavity corner can still
/// slip between neighbouring rays, which the fan answers by bounding the whole
/// cross-section by its widest reach (see the caller) rather than by each ray's own
/// share of it.
///
/// The wedge is spanned by the faces themselves, not by their inward normals: those
/// coincide only at a right angle. On an acute edge — the high side of a cylinder cut
/// off askew, where the cap meets the wall at 70° — the ray along the cap's inward normal
/// leans 20° *outside* the wall and leaves the body through the next facet round, a
/// fraction of a millimetre away, and that fraction became the limit on a rim with eight
/// millimetres of wall under it. The rays divide the wedge in six and skip the two
/// faces: a ray running in a face's own plane is skipped by [`ray_into_solid`] as
/// coplanar with it, but on a faceted surface it drifts across the facet and meets the
/// neighbouring facet's plane at the seam, and a ray only a degree or two inside a convex
/// faceted face is overtaken by that plane a little further on. Eleven degrees off the
/// face clears a 20° facet.
fn interior_fan(seg: &EdgeSegment) -> impl Iterator<Item = Vec3> {
    let (_, da, db, _) = dihedral(seg);
    (1..6).filter_map(move |k| {
        let w = k as f64 / 6.0;
        (da * (1.0 - w) + db * w).try_normalize()
    })
}

/// Distance along `dir` from `from` to the first face of the solid ahead of it, or
/// `None` when the ray leaves without meeting one. The origin sits on the solid's own
/// surface — an edge of it — so anything within [`OWN_BOUNDARY`] is the surface the ray
/// started from, and a hit on a polygon's border counts as a hit: for a bound the
/// grazing ray is the one that must not be waved through.
fn ray_into_solid(solid: &Solid, from: Vec3, dir: Vec3) -> Option<f64> {
    let mut best = f64::INFINITY;
    for face in &solid.faces {
        for p in &face.polygons {
            let denom = p.plane.normal.dot(dir);
            // Only crossings that *leave* material count — the polygon's outward normal
            // has to agree with the ray. The ray starts inside the material at the edge,
            // so where it ends is an exit; an entry seen first means the ray grazed out
            // between two facets of a tessellated curve a fraction of a facet ago, and
            // stopping at that re-entry would bound every blend on a fine rim by the
            // sagitta of one facet. Near-parallel crossings are skipped too — the
            // fan's endmost rays run *inside* the adjacent faces, and both sides of
            // this quotient are then rounding noise, which measured a ten-millimetre
            // cylinder at a sixth of a millimetre deep. Normal and direction are unit
            // vectors, so the cut is a cosine and scale-free.
            if denom < 1e-6 {
                continue;
            }
            let t = p.plane.normal.dot(p.plane.origin - from) / denom;
            if t <= OWN_BOUNDARY || t >= best {
                continue;
            }
            let q = from + dir * t;
            let inside = p.vertices.iter().enumerate().all(|(i, a)| {
                let b = p.vertices[(i + 1) % p.vertices.len()];
                let along = b - *a;
                along.cross(q - *a).dot(p.plane.normal) >= -MERGE_TOL * along.length()
            });
            if inside {
                best = t;
            }
        }
    }
    best.is_finite().then_some(best)
}

/// Distance from `p` to the nearest point of the segment `a`–`b`.
fn distance_to_segment(p: Vec3, a: Vec3, b: Vec3) -> f64 {
    let along = b - a;
    let len2 = along.length_squared();
    if len2 < 1e-24 {
        return p.distance(a);
    }
    let u = ((p - a).dot(along) / len2).clamp(0.0, 1.0);
    p.distance(a + along * u)
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
/// finds nothing; the answer is then the *nearest* piece of the face's boundary other
/// than the edge being measured, still capped by the face's bounding box along `into`.
/// The straight distance to a boundary point can only under-state the distance across
/// the surface to it, so this never claims room the face does not have, where the plain
/// bounding-box ceiling it replaced did: from an edge cut across a blend that wraps a
/// rim, the box spans the whole rim while the band itself is a fillet-width wide. The
/// price is paid in the other direction — a boundary lying beside the ray rather than
/// ahead of it, as a band's own tangent boundaries do, bounds a reach it would never
/// stop — and that conservatism is accepted; TODO.md carries the honest account.
fn reach_in_face(
    from: Vec3,
    into: Vec3,
    own: EdgeKey,
    boundary: &[(&EdgeSegment, bool, EdgeKey)],
    extent: &Aabb,
) -> (f64, Option<f64>) {
    let cap: f64 = (0..3)
        .map(|i| (into[i] * (extent.min[i] - from[i])).max(into[i] * (extent.max[i] - from[i])))
        .sum();
    let mut best = f64::INFINITY;
    let mut blocker = None;
    for (seg, blended, _) in boundary {
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
        // No crossing: the ray left a doubly-curved surface. The edge's own segments are
        // left out — the ray starts on them, and on a closed rim the edge's far side is
        // beside the surface path rather than across it — and so are boundaries that do
        // not fold, by the same 20° the rest of the kernel calls tangent
        // ([`TANGENT_EDGE_COS`]): a fillet band's tangent circles run *beside* a path
        // along the band, and material continues across them onto the face the band
        // blends into, so they are where the face's name changes and not where its room
        // ends. What remains — real folds, cut ends — bounds the reach by plain
        // distance, and anything within `OWN_BOUNDARY` is the corner the sample itself
        // sits near.
        let nearest = boundary
            .iter()
            .filter(|(seg, _, key)| {
                *key != own && seg.normal_a.dot(seg.normal_b) < TANGENT_EDGE_COS
            })
            .map(|(seg, _, _)| distance_to_segment(from, seg.start, seg.end))
            .filter(|d| *d > OWN_BOUNDARY)
            .fold(f64::INFINITY, f64::min);
        (cap.min(nearest), None)
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

/// The largest *magnitude* an inverted (negative) fillet radius may have on the edges
/// `keys` of `solid`. The editor clamps the handle's negative travel to this. It differs
/// from [`max_fillet_radius`] away from 90°: an inverted round sits a radius out along
/// each face where a regular one sits r/tan(phi/2) out, so on an obtuse edge the inverted
/// form runs out of face first and on an acute one the regular form does.
pub fn max_inverted_fillet_radius(solid: &Solid, keys: &[EdgeKey]) -> Option<f64> {
    size_limit(solid, &solid.edges(), keys, BlendKind::InvertedFillet)
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
///
/// A regular fillet's section is the region between the two faces and an arc tangent to
/// both, centred r/sin(phi/2) along the in-face bisector. An inverted fillet's is the
/// sector of the disc of radius |r| *about the edge line*, cut off by the two face
/// planes: it lands |r| out along each face and its arc bulges away from the edge,
/// sweeping the dihedral angle phi itself rather than its supplement. That is the rule
/// at every dihedral, not only 90°: the inverted section is always the set of points
/// within |r| of the edge that lie between the two face planes, so an acute edge takes a
/// narrow sector and an obtuse one a wide one, and its setback along each face is |r|
/// exactly (which is why [`BlendKind::InvertedFillet`] bounds it like a chamfer). At 90°
/// the two are complementary in the r × r square at the edge: the inverted section is
/// that square less a copy of the regular section turned through 180°, so the cove is
/// precisely the quarter-cylinder of material a regular round of the same radius leaves
/// standing — the "press through to the other shape" reading the handle gives it. Either
/// way the scaffolding past the faces is the same, so one tool builder serves both.
fn section(seg: &EdgeSegment, blend: Blend, arc_segments: usize, convex: bool) -> Section {
    let (t, da, db, phi) = dihedral(seg);
    let (na, nb) = (seg.normal_a, seg.normal_b);
    let (ta, tb, arc) = match blend {
        Blend::Chamfer { distance } => (da * distance, db * distance, Vec::new()),
        Blend::Fillet { radius } => {
            let inverted = radius < 0.0;
            let radius = radius.abs();
            let (centre, ta, tb, sweep) = if inverted {
                (Vec3::ZERO, da * radius, db * radius, phi)
            } else {
                let d = radius / (phi / 2.0).tan();
                let centre = (da + db).normalize() * (radius / (phi / 2.0).sin());
                (centre, da * d, db * d, std::f64::consts::PI - phi)
            };
            // The arc from ta to tb about `centre`: for a regular fillet on the side
            // nearest the edge, for an inverted one through the bisector away from it.
            // Both are the shorter way round from ta to tb, so one sweep rule serves.
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
    chain: &[Link],
    ends: [ChainEnd; 2],
    blend: Blend,
    tess: &Tessellation,
    budget: usize,
) -> Result<(Solid, BoolOp), KernelError> {
    let first = chain[0];
    let key = first.key;
    let (t0, _, _, _) = dihedral(&first.seg);
    // The faces do not fold here, so there is no dihedral to fill. This is the same
    // condition `Edge::smooth` reports, which is why the editor never offers such an
    // edge; a saved feature whose edge has since flattened arrives here instead.
    let convex = is_convex(&first.seg).ok_or(KernelError::TangentEdge(key))?;
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
            // The inverted arc sweeps the dihedral angle, the regular one its supplement.
            let sweeps = chain.iter().map(|l| {
                let phi = dihedral(&l.seg).3;
                if radius < 0.0 {
                    phi
                } else {
                    std::f64::consts::PI - phi
                }
            });
            let mut wanted = sweeps
                .clone()
                .map(|sweep| tess.segment_count(radius.abs(), sweep))
                .max()
                .unwrap_or(1)
                .max(MIN_ARC_SEGMENTS);
            let curved = chain
                .iter()
                .any(|l| (l.seg.end - l.seg.start).normalize().dot(t0) < 1.0 - 1e-9);
            if curved {
                let widest = sweeps.fold(0.0, f64::max);
                let coarsest = (widest / MIN_CURVED_FACET_ANGLE).floor() as usize;
                wanted = wanted.min(coarsest.max(MIN_ARC_SEGMENTS));
            }
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

    // Along the chain the tool runs a clearance past an open end or stops a clearance
    // short of it, by what lies beyond ([`ChainEnd`]).
    let run_on = |end: ChainEnd| match end {
        ChainEnd::Overhang => CLEARANCE,
        ChainEnd::Short => -SHORT,
    };
    let last = *chain.last().unwrap();
    let t_end = (last.seg.end - last.seg.start).normalize();
    let start_point = first.seg.start - t0 * run_on(ends[0]);
    let end_point = last.seg.end + t_end * run_on(ends[1]);

    // One ring of section points per joint. Interior joints take the section of the
    // incoming segment projected onto the bisecting plane; chain ends are moved along the
    // edge to where the tool finishes.
    let mut rings: Vec<Vec<Vec3>> = Vec::with_capacity(chain.len() + 1);
    let sections: Vec<Section> = chain
        .iter()
        .map(|l| section(&l.seg, blend, arc_segments, convex))
        .collect();
    let joint_count = if closed { chain.len() } else { chain.len() + 1 };
    for j in 0..joint_count {
        let (incoming, outgoing) = if closed {
            (
                Some(&chain[(j + chain.len() - 1) % chain.len()].seg),
                Some(&chain[j].seg),
            )
        } else {
            (
                if j > 0 { Some(&chain[j - 1].seg) } else { None },
                chain.get(j).map(|l| &l.seg),
            )
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
            (None, Some(_)) => sections[j]
                .offsets
                .iter()
                .map(|o| start_point + *o)
                .collect(),
            (Some(_), None) => sections[j - 1]
                .offsets
                .iter()
                .map(|o| end_point + *o)
                .collect(),
            (None, None) => unreachable!("a chain has at least one segment"),
        };
        rings.push(ring);
    }

    // The blend face of each selected edge in the chain, with the surface it lies on. A
    // run of collinear pieces — which is what every straight edge is once a boolean's
    // healing has put vertices along it — is one cylinder, so it reports as one; a chain
    // that turns is left freeform.
    let role = |index: u32| match blend {
        Blend::Fillet { .. } => FaceRole::Fillet(index),
        Blend::Chamfer { .. } => FaceRole::Chamfer(index),
    };
    let mut blend_faces: Vec<(u32, FaceKey, SurfaceKind)> = Vec::new();
    for link in chain {
        if blend_faces.iter().any(|(i, _, _)| *i == link.index) {
            continue;
        }
        let own: Vec<&Link> = chain.iter().filter(|l| l.index == link.index).collect();
        let straight = own.iter().all(|l| {
            let d = (l.seg.end - l.seg.start).normalize();
            d.dot(t0) > 1.0 - 1e-9 && (l.seg.start - first.seg.start).cross(t0).length() < MERGE_TOL
        });
        let surface = match (blend, straight) {
            (Blend::Fillet { radius }, true) if radius < 0.0 => SurfaceKind::Cylindrical {
                // The inverted arc is drawn about the edge itself.
                origin: first.seg.start,
                axis: t0,
                radius: -radius,
            },
            (Blend::Fillet { radius }, true) => {
                let sec = &sections[0];
                let da = sec.offsets[0];
                let db = sec.offsets[sec.blend_range.end - 1];
                let phi = da.normalize().dot(db.normalize()).clamp(-1.0, 1.0).acos();
                let centre = first.seg.start + (da + db).normalize() * (radius / (phi / 2.0).sin());
                SurfaceKind::Cylindrical {
                    origin: centre,
                    axis: t0,
                    radius,
                }
            }
            (Blend::Chamfer { .. }, true) => SurfaceKind::Planar { normal: Vec3::ZERO },
            _ => SurfaceKind::Freeform,
        };
        blend_faces.push((link.index, FaceKey::new(op, role(link.index)), surface));
    }
    let blend_face = |index: u32| {
        blend_faces
            .iter()
            .find(|(i, _, _)| *i == index)
            .map(|(_, k, s)| (*k, *s))
            .expect("every link's index was entered")
    };
    let scaffold = |i: usize| FaceKey::new(op, FaceRole::Generic(first.index * 8 + i as u32));

    let mut b = SolidBuilder::default();
    let n = sections[0].offsets.len();
    // Rings advancing along +t with counter-clockwise winding (seen from +t) make the
    // extrude quad order face outward; a clockwise ring needs the mirror order.
    let ccw = newell_normal(&rings[0]).dot(t0) > 0.0;
    let pair_count = if closed { rings.len() } else { rings.len() - 1 };
    for r in 0..pair_count {
        let (cur, next) = (&rings[r], &rings[(r + 1) % rings.len()]);
        let (blend_key, surface) = blend_face(chain[r].index);
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
        let caps = [
            (&rings[0], start_point, -t0, 6u32),
            (rings.last().unwrap(), end_point, t_end, 7),
        ];
        for (ring, centre, desired, i) in caps {
            let cap_key = FaceKey::new(op, FaceRole::Generic(first.index * 8 + i));
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

    /// A negative radius is the complementary round: on a convex edge it cuts the
    /// quarter-cylinder about the edge away, a cove where a regular fillet leaves a
    /// bulge. Regular and inverted together take exactly the r × r square at the edge.
    #[test]
    fn inverted_fillet_cuts_a_cove_into_a_convex_edge() {
        let c = cube();
        let key = edge_between(FaceRole::EndCap, FaceRole::Side(0));
        let r = fillet(OpId::new(2), &c, &[key], -2.0, &fine()).unwrap();
        assert_relative_eq!(r.volume(), 1000.0 - PI * 4.0 / 4.0 * 10.0, epsilon = 0.05);
        assert!(r.is_closed(), "{:?}", r.validate());
        let bb = r.aabb();
        assert_relative_eq!(bb.min.length_squared(), 0.0, epsilon = 1e-9);
        assert_relative_eq!(bb.max.length_squared(), 300.0, epsilon = 1e-6);
        // The regular round on the same edge takes the rest of the 2 × 2 square.
        let regular = fillet(OpId::new(2), &c, &[key], 2.0, &fine()).unwrap();
        assert_relative_eq!(regular.volume(), 1000.0 - (4.0 - PI) * 10.0, epsilon = 0.05);
        assert_relative_eq!(
            (1000.0 - r.volume()) + (1000.0 - regular.volume()),
            2.0 * 2.0 * 10.0,
            epsilon = 0.1
        );
        // The new face keeps the fillet's key — flipping the sign must not orphan a
        // later feature that names it — and is a cylinder about the edge of radius |r|.
        let face = r
            .face(FaceKey::new(OpId::new(2), FaceRole::Fillet(0)))
            .unwrap();
        match face.surface {
            SurfaceKind::Cylindrical { origin, radius, .. } => {
                assert_eq!(radius, 2.0);
                assert_relative_eq!(origin.y, 0.0, epsilon = 1e-9);
                assert_relative_eq!(origin.z, 10.0, epsilon = 1e-9);
            }
            other => panic!("{other:?}"),
        }
        assert_relative_eq!(face.area(), PI * 2.0 / 2.0 * 10.0, epsilon = 0.05);
        // Each adjacent face loses a 2 mm strip, as it would to a chamfer.
        let top = r
            .face(FaceKey::new(OpId::new(1), FaceRole::EndCap))
            .unwrap();
        assert_relative_eq!(top.area(), 80.0, epsilon = 1e-2);
    }

    /// On a concave edge the inverted round adds the quarter-cylinder: a bead in the
    /// corner rather than the regular fillet's cove-shaped infill.
    #[test]
    fn inverted_fillet_lays_a_bead_into_a_concave_edge() {
        let (l, key) = l_block();
        let r = fillet(OpId::new(3), &l, &[key], -2.0, &fine()).unwrap();
        assert_relative_eq!(r.volume(), 750.0 + PI * 4.0 / 4.0 * 10.0, epsilon = 0.05);
        assert!(r.is_closed(), "{:?}", r.validate());
        // Nothing grows outside the block's envelope, bar the tool's end clearance,
        // which a regular concave fillet leaves past the chain's ends as well.
        let bb = r.aabb();
        assert!(bb.min.min_element() > -2.0 * CLEARANCE, "{bb:?}");
        assert!(bb.max.max_element() < 10.0 + 2.0 * CLEARANCE, "{bb:?}");
    }

    /// The inverted round is bounded by the faces it sits between, like the regular one:
    /// at 90° by the same number, and zero is refused whichever way it is read.
    #[test]
    fn inverted_fillet_is_bounded_by_the_material_and_refuses_zero() {
        let c = cube();
        let key = edge_between(FaceRole::EndCap, FaceRole::Side(0));
        let limit = max_inverted_fillet_radius(&c, &[key]).unwrap();
        assert_relative_eq!(
            limit,
            max_fillet_radius(&c, &[key]).unwrap(),
            epsilon = 1e-9
        );
        assert!(matches!(
            fillet(OpId::new(2), &c, &[key], -(limit * 1.5), &fine()),
            Err(KernelError::BlendTooLarge { .. })
        ));
        assert!(matches!(
            fillet(OpId::new(2), &c, &[key], 0.0, &fine()),
            Err(KernelError::NonPositiveBlend)
        ));
        // A chamfer has no inverted form, so a negative distance is still a refusal.
        assert!(matches!(
            chamfer(OpId::new(2), &c, &[key], -2.0),
            Err(KernelError::NonPositiveBlend)
        ));
    }

    /// Round a closed rim the inverted tool is a ring of quarter-discs about the rim, and
    /// the sweep closes on itself the way the regular one does.
    #[test]
    fn inverted_fillet_round_a_cylinder_rim() {
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
        let r = fillet(OpId::new(2), &cyl, &[key], -1.0, &Tessellation::default()).unwrap();
        assert!(r.is_closed(), "{:?}", r.validate());
        // Pappus: a quarter-disc of radius 1 (area π/4, centroid 4/(3π) in from the rim)
        // swept round the rim.
        let expected = cyl.volume() - PI / 4.0 * 2.0 * PI * (5.0 - 4.0 / (3.0 * PI));
        assert_relative_eq!(r.volume(), expected, epsilon = 1.5);
        assert!(r.aabb().max.z <= 10.0 + 1e-9);
    }

    /// Off 90° the inverted section is the sector of the dihedral angle, |r| out along
    /// each face: a 45° wedge loses a 45° sector, an eighth of the disc.
    #[test]
    fn inverted_fillet_on_an_acute_edge_takes_the_sector_of_the_dihedral() {
        let prism = wedge();
        assert_relative_eq!(prism.volume(), 500.0, epsilon = 1e-6);
        let edge = vertical_edge_at(&prism, 10.0, 0.0);
        let r = fillet(OpId::new(2), &prism, &[edge], -2.0, &fine()).unwrap();
        assert!(r.is_closed(), "{:?}", r.validate());
        let sector = PI * 4.0 * (45.0 / 360.0);
        assert_relative_eq!(r.volume(), 500.0 - sector * 10.0, epsilon = 0.05);
        // A radius out along each face: the side along y = 0 ends 2 mm short of x = 10.
        let side = r
            .faces
            .iter()
            .find(|f| {
                f.polygons
                    .iter()
                    .all(|p| p.vertices.iter().all(|v| v.y.abs() < 1e-9))
            })
            .expect("the side along y = 0");
        let far_x = side
            .polygons
            .iter()
            .flat_map(|p| p.vertices.iter().map(|v| v.x))
            .fold(f64::MIN, f64::max);
        assert_relative_eq!(far_x, 8.0, epsilon = 1e-6);
        // On an acute edge the regular round lands further out (r/tan(22.5°) ≈ 2.41 r)
        // than the inverted one (r), so it is the regular limit that is the tighter.
        let (inverted, regular) = (
            max_inverted_fillet_radius(&prism, &[edge]).unwrap(),
            max_fillet_radius(&prism, &[edge]).unwrap(),
        );
        assert!(
            inverted > regular,
            "inverted {inverted} vs regular {regular}"
        );
    }

    /// A right triangular prism, legs 10 along x and y, 10 tall: its two vertical edges
    /// at the ends of the hypotenuse have 45° dihedrals.
    fn wedge() -> Solid {
        use basset_math::{Frame, Vec2};
        // One curve tag per side, or the extrude folds the three into one face.
        let profile = crate::Profile::new(
            Frame::XY,
            crate::Contour {
                points: vec![
                    Vec2::new(0.0, 0.0),
                    Vec2::new(10.0, 0.0),
                    Vec2::new(0.0, 10.0),
                ],
                segments: (0..3).map(crate::Segment::line).collect(),
                closed: true,
            },
        );
        crate::extrude(OpId::new(1), &profile, crate::Extent::OneSide(10.0)).unwrap()
    }

    /// The vertical edge of a prism standing at (x, y).
    fn vertical_edge_at(solid: &Solid, x: f64, y: f64) -> EdgeKey {
        solid
            .edges()
            .into_iter()
            .find(|e| {
                e.segments.iter().all(|s| {
                    [s.start, s.end]
                        .iter()
                        .all(|p| (p.x - x).abs() < 1e-9 && (p.y - y).abs() < 1e-9)
                })
            })
            .unwrap_or_else(|| panic!("no vertical edge at ({x}, {y})"))
            .key
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
        let links = |chain: Vec<EdgeSegment>, key: EdgeKey| -> Vec<Link> {
            chain
                .into_iter()
                .map(|seg| Link { seg, index: 0, key })
                .collect()
        };
        let open = links(open, edge_between(FaceRole::EndCap, FaceRole::Side(0)));
        let closed = links(closed, rim(FaceRole::EndCap));
        assert!(!is_closed(&open) && is_closed(&closed), "the fixtures");
        for chain in [&open, &closed] {
            for budget in [
                tool_polygons(ring_count(chain), MIN_ARC_SEGMENTS),
                tool_polygons(ring_count(chain), 7),
                MAX_TOOL_POLYGONS,
            ] {
                let (tool, _) = tool_for_chain(
                    OpId::new(2),
                    chain,
                    [ChainEnd::Overhang; 2],
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
        assert_relative_eq!(merged[0].0.volume(), 16.0, epsilon = 1e-6);
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
        assert_relative_eq!(merged[0].0.volume(), 8.0 + 8.0 - 1.0, epsilon = 1e-6);
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
        assert_relative_eq!(limit, 10.0, epsilon = 1e-6);
        let err = fillet(OpId::new(2), &c, &[key], 12.0, &fine()).unwrap_err();
        assert!(
            matches!(err, KernelError::BlendTooLarge { size, limit }
                     if size == 12.0 && (limit - 10.0).abs() < 1e-6),
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
        assert_relative_eq!(limit, 5.0, epsilon = 1e-6);
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
        assert_relative_eq!(max_fillet_radius(&c, &keys).unwrap(), 5.0, epsilon = 1e-6);
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
            epsilon = 1e-6
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
            epsilon = 1e-6
        );
    }

    /// Extruded trapezoid, 10 mm tall: the top face is 10 mm deep behind the middle of
    /// its long side and 2.5 mm behind either end, where the slanted sides close in.
    fn trapezoid_block() -> Solid {
        let mut outer = crate::Contour::polygon(
            vec![
                basset_math::Vec2::ZERO,
                basset_math::Vec2::new(10.0, 0.0),
                basset_math::Vec2::new(8.0, 10.0),
                basset_math::Vec2::new(2.0, 10.0),
            ],
            0,
        );
        for (i, s) in outer.segments.iter_mut().enumerate() {
            s.curve = i as u32;
        }
        crate::extrude(
            OpId::new(1),
            &crate::Profile::new(basset_math::Frame::XY, outer),
            crate::geometry::Extent::OneSide(10.0),
        )
        .unwrap()
    }

    /// A face is measured where it is narrowest, not only behind the segment's midpoint.
    /// A straight edge on a planar face is one segment however long it is, so on a
    /// tapered face the midpoint ray reports the room in the middle — 10 mm here — while
    /// the ends have 2.5 mm before the slanted sides close in, and a fillet sized to the
    /// middle overruns both corners.
    #[test]
    fn a_tapered_face_is_measured_at_its_narrow_ends_too() {
        let block = trapezoid_block();
        let key = edge_between(FaceRole::EndCap, FaceRole::Side(0));
        let limit = max_fillet_radius(&block, &[key]).unwrap();
        // A ray across the top face from x = 0.5 — the sample near the segment's start —
        // leaves through the slanted side x = 0.2·y at y = 2.5.
        assert_relative_eq!(limit, 2.5, epsilon = 1e-6);
        // Sized to what the midpoint alone would have allowed: refused, not overrun.
        assert!(matches!(
            fillet(OpId::new(2), &block, &[key], 4.0, &fine()).unwrap_err(),
            KernelError::BlendTooLarge { .. }
        ));
        let r = fillet(
            OpId::new(2),
            &block,
            &[key],
            limit,
            &Tessellation::default(),
        )
        .unwrap();
        assert!(r.is_closed(), "{:?}", r.validate());
        assert!(r.validate().is_ok(), "{:?}", r.validate());
        assert!(r.volume() < block.volume() && r.volume() > 0.0);
    }

    /// An edge a boolean left across a rim's fillet band. The band is doubly curved, so
    /// the in-face ray leaves it without meeting a boundary, and the reach used to fall
    /// back to the band's bounding box: the whole rim, about 5 mm, for a band one
    /// fillet-radius wide. The conservative fallback and the interior rays bound it by
    /// what is actually there instead.
    ///
    /// The at-the-limit closure invariant is not asserted here: the healer already
    /// fails on this edge at radii well inside *any* limit (0.1 leaks where 0.05 and
    /// 0.25 close), under the old loose bound as much as this one, so closure is
    /// checked at a small radius the boolean handles. TODO.md carries the account.
    #[test]
    fn an_edge_across_a_curved_band_is_not_measured_by_the_bands_bounding_box() {
        let cyl = crate::primitives::cylinder(
            OpId::new(1),
            Vec3::ZERO,
            Vec3::Z,
            5.0,
            10.0,
            &Tessellation::default(),
        );
        let rimmed = fillet(
            OpId::new(2),
            &cyl,
            &[rim(FaceRole::EndCap)],
            1.0,
            &Tessellation::default(),
        )
        .unwrap();
        // Cut the filleted cylinder in half; the cut plane crosses the band, leaving a
        // quarter-arc edge between the cut face and the doubly-curved band.
        let cutter = cuboid(
            OpId::new(3),
            Vec3::new(0.0, -7.0, -1.0),
            Vec3::new(7.0, 7.0, 12.0),
        );
        let half = boolean(&rimmed, &cutter, BoolOp::Subtract).unwrap();
        assert!(half.is_closed(), "{:?}", half.validate());
        let key = EdgeKey::new(
            FaceKey::new(OpId::new(2), FaceRole::Fillet(0)),
            FaceKey::new(OpId::new(3), FaceRole::Side(3)),
        );
        assert!(
            half.edges().iter().any(|e| e.key == key && !e.smooth),
            "the cut leaves a pickable edge on the band"
        );
        let limit = max_fillet_radius(&half, &[key]).unwrap();
        // The interior rays leave the convex band within a couple of millimetres whatever
        // direction they take, so the bound is a fraction of the 5 mm the band's box
        // offered; what exactly depends on where the rays cross the band's facets.
        assert!(limit < 3.0, "the rim's box is not the room: {limit}");
        assert!(limit > 0.02, "and the bound must not collapse: {limit}");
        assert!(matches!(
            fillet(OpId::new(4), &half, &[key], 5.0, &Tessellation::default()).unwrap_err(),
            KernelError::BlendTooLarge { .. }
        ));
        let r = fillet(OpId::new(4), &half, &[key], 0.05, &Tessellation::default()).unwrap();
        assert!(r.is_closed(), "{:?}", r.validate());
        assert!(r.validate().is_ok(), "{:?}", r.validate());
    }

    /// A cavity behind the faces bounds the blend, where the two adjacent faces alone
    /// see a solid 10 mm block. The interior rays fanned across the material's wedge
    /// meet the cavity's ceiling and wall, and the fillet sized to what they find stays
    /// out of it: the same box without the cavity pins its limit at 10.
    ///
    /// The exact bound is 2.5: the arc of a 2.5 round passes through the cavity's near
    /// corner, 1 in and 0.5 down. No ray from the edge can find less than that corner's
    /// distance, √5/2, and the fan is conservative — it asks for the setback along every
    /// ray rather than the section's actual depth there, so that a corner between two
    /// rays is still kept out — so the limit it reports lies between the two.
    #[test]
    fn a_cavity_behind_the_faces_bounds_the_fillet() {
        let hollow = boolean(
            &cube(),
            &cuboid(
                OpId::new(5),
                Vec3::new(2.0, 1.0, 4.0),
                Vec3::new(8.0, 6.0, 9.5),
            ),
            BoolOp::Subtract,
        )
        .unwrap();
        assert!(hollow.is_closed(), "{:?}", hollow.validate());
        assert_relative_eq!(hollow.volume(), 835.0, epsilon = 1e-6);
        let key = edge_between(FaceRole::EndCap, FaceRole::Side(0));
        let limit = max_fillet_radius(&hollow, &[key]).unwrap();
        assert!(
            limit >= 5f64.sqrt() / 2.0 - 1e-9 && limit <= 2.5 + 1e-9,
            "{limit}"
        );
        assert!(matches!(
            fillet(OpId::new(6), &hollow, &[key], 3.0, &fine()).unwrap_err(),
            KernelError::BlendTooLarge { .. }
        ));
        let r = fillet(
            OpId::new(6),
            &hollow,
            &[key],
            limit,
            &Tessellation::default(),
        )
        .unwrap();
        assert!(r.is_closed(), "{:?}", r.validate());
        assert!(r.validate().is_ok(), "{:?}", r.validate());
        // The fillet took its corner strip off the outside and nothing off the cavity.
        let removed = (1.0 - PI / 4.0) * limit * limit * 10.0;
        assert_relative_eq!(r.volume(), 835.0 - removed, epsilon = 0.5);
    }
}
