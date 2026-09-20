//! Fillet and chamfer.
//!
//! Both are built the same way: for every selected edge, construct a prismatic *tool*
//! whose cross-section is the material to remove (convex edge) or add (concave edge),
//! and apply it with a boolean. The cross-section is exact in the plane perpendicular to
//! the edge; along curved edges the prism is mitred at every polyline joint so
//! consecutive pieces meet on the bisecting plane. Corners where several filleted edges
//! meet are simply the intersection of their tools, which is a crease rather than the
//! spherical patch a full-featured kernel would make — an accepted MVP limitation.

use basset_math::Vec3;

use crate::csg::{BoolOp, boolean};
use crate::error::KernelError;
use crate::geometry::Tessellation;
use crate::ids::{EdgeKey, FaceKey, FaceRole, OpId};
use crate::solid::{EdgeSegment, MERGE_TOL, Solid, SolidBuilder, SurfaceKind, newell_normal};

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
/// candidate splitting plane the whole remainder in front of it, so the tree is a list
/// (see [`csg`](crate::csg)). Time is quadratic in the facet count, and the recursion is
/// one frame deep per facet, which is the hard limit: measured on a 2 MiB thread stack,
/// 4 563 polygons come through and 4 803 abort. The budget leaves room for the booleans
/// that follow and is spent by coarsening the arc first, because a faceted fillet is a
/// better answer than a refusal.
const MAX_TOOL_POLYGONS: usize = 2_000;

/// Facets one blend feature may put through its booleans, tools and body together.
///
/// Each tool is applied against the accumulating result, so the last boolean of an
/// n-edge blend builds a tree over everything the earlier ones fragmented, and it is that
/// tree that has to stay inside the stack. Bounding the tools alone would not bound it.
const MAX_FEATURE_POLYGONS: usize = 4_500;

/// Coarsest arc a fillet is willing to draw, and the `.max(2)` the rings need: a ring has
/// to carry both tangent points and at least one point between them.
const MIN_ARC_SEGMENTS: usize = 2;

/// Facets of the swept tool: one ring of section vertices per chain segment. A section
/// carries the arc's points, its two tangent points and three scaffolding corners, and an
/// open chain's two end caps add a handful more that the budget can absorb.
fn tool_polygons(chain_len: usize, arc_segments: usize) -> usize {
    chain_len * (arc_segments + 4)
}

#[derive(Clone, Copy)]
enum Blend {
    Fillet { radius: f64 },
    Chamfer { distance: f64 },
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
    let mut tools = Vec::new();
    for (i, key) in keys.iter().enumerate() {
        let edge = all_edges
            .iter()
            .find(|e| e.key == *key)
            .ok_or(KernelError::MissingEdge(*key))?;
        for chain in edge.chains() {
            tools.push(tool_for_chain(op, i as u32, *key, &chain, blend, tess)?);
        }
    }
    // Checked before any boolean runs, so an over-ambitious blend costs the user a
    // message rather than the minutes it would take to fail part-way through.
    let needed =
        solid.polygon_count() + tools.iter().map(|(t, _)| t.polygon_count()).sum::<usize>();
    if needed > MAX_FEATURE_POLYGONS {
        return Err(KernelError::BlendTooDense {
            needed,
            budget: MAX_FEATURE_POLYGONS,
        });
    }
    let mut result = solid.clone();
    for (tool, bool_op) in tools {
        result = boolean(&result, &tool, bool_op)?;
    }
    Ok(result)
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
    let affordable = (MAX_TOOL_POLYGONS / chain.len()).saturating_sub(4);
    let arc_segments = match blend {
        Blend::Fillet { radius } => {
            if affordable < MIN_ARC_SEGMENTS {
                return Err(KernelError::BlendTooDense {
                    needed: tool_polygons(chain.len(), MIN_ARC_SEGMENTS),
                    budget: MAX_TOOL_POLYGONS,
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
                     to keep the tool for its {} edge segments inside {MAX_TOOL_POLYGONS} \
                     polygons",
                    chain.len()
                );
            }
            wanted.min(affordable)
        }
        Blend::Chamfer { .. } => {
            if tool_polygons(chain.len(), 0) > MAX_TOOL_POLYGONS {
                return Err(KernelError::BlendTooDense {
                    needed: tool_polygons(chain.len(), 0),
                    budget: MAX_TOOL_POLYGONS,
                });
            }
            0
        }
    };

    let closed = chain.len() > 1
        && chain[0].start.distance_squared(chain.last().unwrap().end) < MERGE_TOL * MERGE_TOL;
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
    /// and the feature has to say so rather than spend the session's memory finding out.
    #[test]
    fn a_blend_that_cannot_be_coarsened_far_enough_is_refused() {
        let cyl = rimmed_cylinder(0.2);
        let err = fillet(OpId::new(2), &cyl, &[rim(FaceRole::EndCap)], 1.0, &fine()).unwrap_err();
        assert!(
            matches!(
                err,
                KernelError::BlendTooDense { budget, .. } if budget == MAX_TOOL_POLYGONS
            ),
            "{err:?}"
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
}
