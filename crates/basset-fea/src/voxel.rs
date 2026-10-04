//! A voxel hexahedral mesh of a solid.
//!
//! A grid of identical bricks is laid over the solid's bounding box, sized so whole
//! bricks fit it exactly along every axis, and each cell whose centre is inside the solid
//! becomes an element. "Inside" is ray parity against the solid's own tessellation: one
//! ray is cast along every column of cells and the crossings it collects decide every
//! cell in the column at once, so meshing costs one ray per column rather than one per
//! cell. The ray runs a hair off the column's centre line so that it never passes exactly
//! through a triangle edge or vertex, where two triangles would both report a hit and the
//! parity would flip.
//!
//! Each exposed facet of the result is tagged with the kernel face nearest its centre.
//! That is how a study's fixed faces and loads, named by [`FaceKey`], find their nodes.

use std::collections::HashMap;

use basset_kernel::{FaceKey, Solid};
use basset_math::{Ray, Vec3};

use crate::FeaError;

/// Node ordering of a brick, matching the VTK hexahedron: the bottom four counter-clockwise
/// about +z starting at the minimum corner, then the top four above them.
pub const CORNERS: [[usize; 3]; 8] = [
    [0, 0, 0],
    [1, 0, 0],
    [1, 1, 0],
    [0, 1, 0],
    [0, 0, 1],
    [1, 0, 1],
    [1, 1, 1],
    [0, 1, 1],
];

/// One exposed square of an element: its four nodes, counter-clockwise about the outward
/// normal, and the kernel face it stands in for.
#[derive(Debug, Clone, PartialEq)]
pub struct Facet {
    pub element: usize,
    pub nodes: [usize; 4],
    pub normal: Vec3,
    pub area: f64,
    pub face: FaceKey,
}

#[derive(Debug, Clone, PartialEq)]
pub struct HexMesh {
    pub nodes: Vec<Vec3>,
    /// Eight node indexes per element in [`CORNERS`] order.
    pub elements: Vec<[usize; 8]>,
    /// Brick edge lengths along x, y and z.
    pub size: Vec3,
    pub facets: Vec<Facet>,
}

impl HexMesh {
    pub fn element_centre(&self, e: usize) -> Vec3 {
        match self.elements.get(e) {
            Some(nodes) => nodes.iter().map(|&n| self.nodes[n]).sum::<Vec3>() / 8.0,
            None => Vec3::ZERO,
        }
    }

    pub fn volume(&self) -> f64 {
        self.elements.len() as f64 * self.size.x * self.size.y * self.size.z
    }

    /// Every kernel face some facet stands in for, in key order.
    pub fn touched_faces(&self) -> Vec<FaceKey> {
        let mut keys: Vec<FaceKey> = self.facets.iter().map(|f| f.face).collect();
        keys.sort();
        keys.dedup();
        keys
    }
}

/// Meshes `solid` with bricks about `element_size` on a side.
pub fn mesh(solid: &Solid, element_size: f64) -> Result<HexMesh, FeaError> {
    if element_size.is_nan() || element_size <= 0.0 || element_size.is_infinite() {
        return Err(FeaError::BadElementSize(element_size));
    }
    if solid.is_empty() {
        return Err(FeaError::EmptySolid);
    }
    let aabb = solid.aabb();
    let extent = aabb.extent();
    let counts =
        [extent.x, extent.y, extent.z].map(|e| ((e / element_size).round() as usize).max(1));
    let cells = counts[0] * counts[1] * counts[2];
    if cells > crate::MAX_ELEMENTS {
        return Err(FeaError::TooManyElements {
            elements: cells,
            limit: crate::MAX_ELEMENTS,
        });
    }
    let size = Vec3::new(
        extent.x / counts[0] as f64,
        extent.y / counts[1] as f64,
        extent.z / counts[2] as f64,
    );
    let tess = solid.tessellate();
    let tris: Vec<[Vec3; 3]> = (0..tess.mesh.triangle_count())
        .map(|i| tess.mesh.triangle(i))
        .collect();

    // Which cells are inside, by column along x.
    let mut inside = vec![false; cells];
    let at = |i: usize, j: usize, k: usize| (k * counts[1] + j) * counts[0] + i;
    let start_x = aabb.min.x - size.x;
    for k in 0..counts[2] {
        for j in 0..counts[1] {
            // Off-centre by an irrational-looking fraction of a brick, so the line misses
            // every edge of an axis-aligned facet; see the module comment.
            let y = aabb.min.y + (j as f64 + 0.5 + 0.0137) * size.y;
            let z = aabb.min.z + (k as f64 + 0.5 + 0.0071) * size.z;
            let ray = Ray::new(Vec3::new(start_x, y, z), Vec3::X);
            let mut crossings: Vec<f64> = tris
                .iter()
                .filter(|t| {
                    let (ymin, ymax) = minmax(t[0].y, t[1].y, t[2].y);
                    let (zmin, zmax) = minmax(t[0].z, t[1].z, t[2].z);
                    y >= ymin && y <= ymax && z >= zmin && z <= zmax
                })
                .filter_map(|t| ray_triangle(&ray, t))
                .collect();
            crossings.sort_by(f64::total_cmp);
            for i in 0..counts[0] {
                let x = start_x + (i as f64 + 1.5) * size.x;
                let below = crossings.partition_point(|&t| t < x - start_x);
                inside[at(i, j, k)] = below % 2 == 1;
            }
        }
    }

    // Nodes and elements.
    let mut node_of: HashMap<[usize; 3], usize> = HashMap::new();
    let mut nodes = Vec::new();
    let mut elements = Vec::new();
    let mut element_at = vec![usize::MAX; cells];
    for k in 0..counts[2] {
        for j in 0..counts[1] {
            for i in 0..counts[0] {
                if !inside[at(i, j, k)] {
                    continue;
                }
                let corners = CORNERS.map(|[di, dj, dk]| {
                    let key = [i + di, j + dj, k + dk];
                    *node_of.entry(key).or_insert_with(|| {
                        nodes.push(
                            aabb.min
                                + Vec3::new(
                                    key[0] as f64 * size.x,
                                    key[1] as f64 * size.y,
                                    key[2] as f64 * size.z,
                                ),
                        );
                        nodes.len() - 1
                    })
                });
                element_at[at(i, j, k)] = elements.len();
                elements.push(corners);
            }
        }
    }
    if elements.is_empty() {
        return Err(FeaError::NothingMeshed);
    }

    // Exposed facets, tagged with the nearest kernel face.
    let mut facets = Vec::new();
    for k in 0..counts[2] {
        for j in 0..counts[1] {
            for i in 0..counts[0] {
                let e = element_at[at(i, j, k)];
                if e == usize::MAX {
                    continue;
                }
                for (axis, dir) in [(0, -1i64), (0, 1), (1, -1), (1, 1), (2, -1), (2, 1)] {
                    let mut n = [i as i64, j as i64, k as i64];
                    n[axis] += dir;
                    let neighbour_inside = n[0] >= 0
                        && n[1] >= 0
                        && n[2] >= 0
                        && (n[0] as usize) < counts[0]
                        && (n[1] as usize) < counts[1]
                        && (n[2] as usize) < counts[2]
                        && inside[at(n[0] as usize, n[1] as usize, n[2] as usize)];
                    if neighbour_inside {
                        continue;
                    }
                    let local = facet_corners(axis, dir > 0);
                    let nodes_of_facet = local.map(|c| elements[e][c]);
                    let centre = nodes_of_facet.iter().map(|&n| nodes[n]).sum::<Vec3>() / 4.0;
                    let mut normal = Vec3::ZERO;
                    normal[axis] = dir as f64;
                    let area = match axis {
                        0 => size.y * size.z,
                        1 => size.x * size.z,
                        _ => size.x * size.y,
                    };
                    let face = nearest_face(&tess, &tris, centre, normal);
                    facets.push(Facet {
                        element: e,
                        nodes: nodes_of_facet,
                        normal,
                        area,
                        face,
                    });
                }
            }
        }
    }

    Ok(HexMesh {
        nodes,
        elements,
        size,
        facets,
    })
}

/// The four corners of a brick's facet on one side, counter-clockwise about its outward
/// normal.
pub(crate) fn facet_corners(axis: usize, positive: bool) -> [usize; 4] {
    match (axis, positive) {
        (0, false) => [0, 4, 7, 3],
        (0, true) => [1, 2, 6, 5],
        (1, false) => [0, 1, 5, 4],
        (1, true) => [3, 7, 6, 2],
        (2, false) => [0, 3, 2, 1],
        _ => [4, 5, 6, 7],
    }
}

/// The kernel face whose tessellation comes nearest `p`. A triangle facing the other way
/// from the facet is passed over for one that does not, so a facet on a thin wall takes
/// the wall's near side rather than its far one when both are equally close.
fn nearest_face(
    tess: &basset_kernel::Tessellated,
    tris: &[[Vec3; 3]],
    p: Vec3,
    normal: Vec3,
) -> FaceKey {
    let mut best: Option<(f64, bool, usize)> = None;
    for (i, t) in tris.iter().enumerate() {
        let d = point_triangle_distance_squared(p, t);
        let facing = (t[1] - t[0]).cross(t[2] - t[0]).dot(normal) >= 0.0;
        // A facing triangle wins over one that is not unless it is clearly further away.
        let better = match best {
            None => true,
            Some((bd, bf, _)) => {
                if facing == bf {
                    d < bd
                } else if facing {
                    d < bd * 4.0 + 1e-18
                } else {
                    d * 4.0 + 1e-18 < bd
                }
            }
        };
        if better {
            best = Some((d, facing, i));
        }
    }
    let tri = best.map(|b| b.2).unwrap_or(0);
    tess.face_keys[tess.mesh.face_ids[tri] as usize]
}

fn minmax(a: f64, b: f64, c: f64) -> (f64, f64) {
    (a.min(b).min(c), a.max(b).max(c))
}

/// Every crossing of the ray's line ahead of its origin, back faces included.
fn ray_triangle(ray: &Ray, [a, b, c]: &[Vec3; 3]) -> Option<f64> {
    const EPS: f64 = 1e-12;
    let e1 = *b - *a;
    let e2 = *c - *a;
    let p = ray.direction.cross(e2);
    let det = e1.dot(p);
    if det.abs() < EPS {
        return None;
    }
    let inv = 1.0 / det;
    let s = ray.origin - *a;
    let u = s.dot(p) * inv;
    if !(0.0..=1.0).contains(&u) {
        return None;
    }
    let q = s.cross(e1);
    let v = ray.direction.dot(q) * inv;
    if v < 0.0 || u + v > 1.0 {
        return None;
    }
    let t = e2.dot(q) * inv;
    (t > 0.0).then_some(t)
}

/// Ericson, *Real-Time Collision Detection* §5.1.5.
fn point_triangle_distance_squared(p: Vec3, [a, b, c]: &[Vec3; 3]) -> f64 {
    let (a, b, c) = (*a, *b, *c);
    let ab = b - a;
    let ac = c - a;
    let ap = p - a;
    let d1 = ab.dot(ap);
    let d2 = ac.dot(ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return ap.length_squared();
    }
    let bp = p - b;
    let d3 = ab.dot(bp);
    let d4 = ac.dot(bp);
    if d3 >= 0.0 && d4 <= d3 {
        return bp.length_squared();
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return (a + ab * v - p).length_squared();
    }
    let cp = p - c;
    let d5 = ab.dot(cp);
    let d6 = ac.dot(cp);
    if d6 >= 0.0 && d5 <= d6 {
        return cp.length_squared();
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return (a + ac * w - p).length_squared();
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return (b + (c - b) * w - p).length_squared();
    }
    let denom = 1.0 / (va + vb + vc);
    let v = vb * denom;
    let w = vc * denom;
    (a + ab * v + ac * w - p).length_squared()
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use basset_kernel::{FaceRole, OpId, Tessellation, primitives};

    #[test]
    fn a_box_meshes_exactly() {
        let solid = primitives::cuboid(OpId::new(1), Vec3::ZERO, Vec3::new(10.0, 4.0, 2.0));
        let m = mesh(&solid, 1.0).unwrap();
        assert_eq!(m.elements.len(), 80);
        assert_eq!(m.nodes.len(), 11 * 5 * 3);
        assert_relative_eq!(m.volume(), 80.0);
        assert_eq!(m.facets.len(), 2 * (40 + 20 + 8));
        // Every facet names the box face it lies on.
        let key = |role| FaceKey::new(OpId::new(1), role);
        for f in &m.facets {
            let want = match (f.normal.x as i32, f.normal.y as i32, f.normal.z as i32) {
                (0, 0, -1) => key(FaceRole::StartCap),
                (0, 0, 1) => key(FaceRole::EndCap),
                (0, -1, 0) => key(FaceRole::Side(0)),
                (1, 0, 0) => key(FaceRole::Side(1)),
                (0, 1, 0) => key(FaceRole::Side(2)),
                _ => key(FaceRole::Side(3)),
            };
            assert_eq!(f.face, want, "facet with normal {}", f.normal);
        }
    }

    #[test]
    fn a_cylinder_meshes_to_about_its_volume() {
        let solid = primitives::cylinder(
            OpId::new(1),
            Vec3::ZERO,
            Vec3::Z,
            10.0,
            5.0,
            &Tessellation::default(),
        );
        let m = mesh(&solid, 0.5).unwrap();
        let exact = solid.volume();
        assert!(
            (m.volume() - exact).abs() / exact < 0.03,
            "voxel volume {} vs {exact}",
            m.volume()
        );
        let touched = m.touched_faces();
        assert_eq!(touched.len(), 3, "{touched:?}");
    }

    #[test]
    fn element_size_is_checked() {
        let solid = primitives::cuboid(OpId::new(1), Vec3::ZERO, Vec3::ONE);
        assert!(matches!(
            mesh(&solid, 0.0),
            Err(FeaError::BadElementSize(_))
        ));
        assert!(matches!(
            mesh(&solid, 0.001),
            Err(FeaError::TooManyElements { .. })
        ));
    }
}
