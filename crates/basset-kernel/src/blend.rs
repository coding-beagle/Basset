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
    let arc_segments = match blend {
        Blend::Fillet { radius } => chain
            .iter()
            .map(|s| tess.segment_count(radius, std::f64::consts::PI - dihedral(s).3))
            .max()
            .unwrap_or(1)
            .max(2),
        Blend::Chamfer { .. } => 0,
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
