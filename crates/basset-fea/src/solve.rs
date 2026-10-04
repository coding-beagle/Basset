//! Boundary conditions and the linear solve.
//!
//! The stiffness matrix is never formed. Every element is the same [`Brick`], so the
//! product `K·u` is a gather, a 24×24 multiply and a scatter per element, and a
//! Jacobi-preconditioned conjugate gradient needs nothing more. Fixed degrees of freedom
//! are handled by masking: they are dropped from the residual and never move, which is
//! the same as deleting their rows and columns.

use basset_math::Vec3;

use crate::element::{Brick, DOF, von_mises};
use crate::voxel::HexMesh;
use crate::{FeaError, LoadKind, Results, Study};

/// Relative residual at which the conjugate gradient stops.
const TOLERANCE: f64 = 1e-9;

pub fn run(mesh: HexMesh, study: &Study) -> Result<Results, FeaError> {
    let touched = mesh.touched_faces();
    for face in study
        .fixed
        .iter()
        .chain(study.loads.iter().map(|l| &l.face))
    {
        if !touched.contains(face) {
            return Err(FeaError::FaceNotOnMesh {
                face: *face,
                touched: touched.clone(),
            });
        }
    }
    let ndof = 3 * mesh.nodes.len();
    let mut fixed = vec![false; ndof];
    for facet in mesh.facets.iter().filter(|f| study.fixed.contains(&f.face)) {
        for &n in &facet.nodes {
            fixed[3 * n..3 * n + 3].fill(true);
        }
    }
    if fixed.iter().all(|&f| f) {
        return Err(FeaError::EverythingFixed);
    }

    // Loads: a force is shared by area over the face's facets; a pressure acts on each.
    let mut force = vec![0.0; ndof];
    for load in &study.loads {
        let area: f64 = mesh
            .facets
            .iter()
            .filter(|f| f.face == load.face)
            .map(|f| f.area)
            .sum();
        for facet in mesh.facets.iter().filter(|f| f.face == load.face) {
            let on_facet = match load.kind {
                LoadKind::Force(f) => f * (facet.area / area),
                LoadKind::Pressure(p) => -facet.normal * (p * facet.area),
            };
            let per_node = on_facet / 4.0;
            for &n in &facet.nodes {
                force[3 * n] += per_node.x;
                force[3 * n + 1] += per_node.y;
                force[3 * n + 2] += per_node.z;
            }
        }
    }

    let brick = Brick::new(mesh.size, &study.material);
    let system = System {
        mesh: &mesh,
        brick: &brick,
        fixed: &fixed,
    };
    let (u, iterations) = conjugate_gradient(&system, &force)?;

    // Reactions are what the fixed nodes push back with: the full product at those rows.
    let mut ku = vec![0.0; ndof];
    system.multiply(&u, &mut ku, false);
    let mut reaction = Vec3::ZERO;
    for n in 0..mesh.nodes.len() {
        if fixed[3 * n] {
            reaction += Vec3::new(ku[3 * n], ku[3 * n + 1], ku[3 * n + 2]);
        }
    }

    let displacements: Vec<Vec3> = (0..mesh.nodes.len())
        .map(|n| Vec3::new(u[3 * n], u[3 * n + 1], u[3 * n + 2]))
        .collect();
    let mut von = Vec::with_capacity(mesh.elements.len());
    let mut nodal = vec![0.0; mesh.nodes.len()];
    let mut count = vec![0u32; mesh.nodes.len()];
    for element in &mesh.elements {
        let ue = gather(element, &u);
        let s = von_mises(&brick.stress(&ue));
        von.push(s);
        for &n in element {
            nodal[n] += s;
            count[n] += 1;
        }
    }
    for (v, c) in nodal.iter_mut().zip(&count) {
        if *c > 0 {
            *v /= *c as f64;
        }
    }

    Ok(Results {
        mesh,
        material: study.material,
        displacements,
        von_mises: von,
        nodal_von_mises: nodal,
        reaction,
        iterations,
    })
}

struct System<'a> {
    mesh: &'a HexMesh,
    brick: &'a Brick,
    fixed: &'a [bool],
}

impl System<'_> {
    /// `out = K·u`. With `masked`, fixed rows and columns are treated as deleted.
    fn multiply(&self, u: &[f64], out: &mut [f64], masked: bool) {
        out.fill(0.0);
        let k = &self.brick.stiffness;
        for element in &self.mesh.elements {
            let mut ue = gather(element, u);
            if masked {
                for (i, &n) in element.iter().enumerate() {
                    for d in 0..3 {
                        if self.fixed[3 * n + d] {
                            ue[3 * i + d] = 0.0;
                        }
                    }
                }
            }
            for (i, &n) in element.iter().enumerate() {
                for d in 0..3 {
                    let row = 3 * i + d;
                    let g = 3 * n + d;
                    if masked && self.fixed[g] {
                        continue;
                    }
                    out[g] += k[row].iter().zip(&ue).map(|(a, b)| a * b).sum::<f64>();
                }
            }
        }
    }

    /// The diagonal of the masked system, for the Jacobi preconditioner.
    fn diagonal(&self) -> Vec<f64> {
        let mut diag = vec![0.0; self.fixed.len()];
        let k = &self.brick.stiffness;
        for element in &self.mesh.elements {
            for (i, &n) in element.iter().enumerate() {
                for d in 0..3 {
                    diag[3 * n + d] += k[3 * i + d][3 * i + d];
                }
            }
        }
        for (v, &f) in diag.iter_mut().zip(self.fixed) {
            if f {
                *v = 1.0;
            }
        }
        diag
    }
}

fn gather(element: &[usize; 8], u: &[f64]) -> [f64; DOF] {
    let mut ue = [0.0; DOF];
    for (i, &n) in element.iter().enumerate() {
        ue[3 * i..3 * i + 3].copy_from_slice(&u[3 * n..3 * n + 3]);
    }
    ue
}

fn dot(a: &[f64], b: &[f64]) -> f64 {
    a.iter().zip(b).map(|(x, y)| x * y).sum()
}

/// Jacobi-preconditioned conjugate gradient on the masked system.
fn conjugate_gradient(system: &System, force: &[f64]) -> Result<(Vec<f64>, usize), FeaError> {
    let n = force.len();
    let fixed = system.fixed;
    let inv_diag: Vec<f64> = system.diagonal().iter().map(|d| 1.0 / d).collect();
    let mut b = force.to_vec();
    for (v, &f) in b.iter_mut().zip(fixed) {
        if f {
            *v = 0.0;
        }
    }
    let norm_b = dot(&b, &b).sqrt();
    let mut u = vec![0.0; n];
    if norm_b == 0.0 {
        return Ok((u, 0));
    }
    let mut r = b;
    let mut z: Vec<f64> = r.iter().zip(&inv_diag).map(|(r, d)| r * d).collect();
    let mut p = z.clone();
    let mut rz = dot(&r, &z);
    let mut q = vec![0.0; n];
    // A floating body never converges; the cap turns that into an error with a hint
    // rather than a hang. Well-posed problems take a small fraction of this.
    let max_iterations = (3 * n).clamp(1_000, 50_000);
    for it in 1..=max_iterations {
        system.multiply(&p, &mut q, true);
        let pq = dot(&p, &q);
        if pq.is_nan() || pq <= 0.0 || pq.is_infinite() {
            return Err(FeaError::DidNotConverge {
                iterations: it,
                residual: dot(&r, &r).sqrt() / norm_b,
            });
        }
        let alpha = rz / pq;
        for i in 0..n {
            u[i] += alpha * p[i];
            r[i] -= alpha * q[i];
        }
        let residual = dot(&r, &r).sqrt() / norm_b;
        if residual < TOLERANCE {
            return Ok((u, it));
        }
        for i in 0..n {
            z[i] = r[i] * inv_diag[i];
        }
        let rz_new = dot(&r, &z);
        let beta = rz_new / rz;
        rz = rz_new;
        for i in 0..n {
            p[i] = z[i] + beta * p[i];
        }
    }
    Err(FeaError::DidNotConverge {
        iterations: max_iterations,
        residual: dot(&r, &r).sqrt() / norm_b,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Load, Material};
    use approx::assert_relative_eq;
    use basset_kernel::{FaceKey, FaceRole, OpId, primitives};

    fn key(role: FaceRole) -> FaceKey {
        FaceKey::new(OpId::new(1), role)
    }

    /// A 10×10×100 bar along x; `Side(3)` is its −x end and `Side(1)` its +x end.
    fn bar() -> basset_kernel::Solid {
        primitives::cuboid(OpId::new(1), Vec3::ZERO, Vec3::new(100.0, 10.0, 10.0))
    }

    #[test]
    fn a_bar_in_tension_matches_hookes_law() {
        let study = Study {
            material: Material::STEEL,
            fixed: vec![key(FaceRole::Side(3))],
            loads: vec![Load {
                face: key(FaceRole::Side(1)),
                kind: LoadKind::Force(Vec3::new(1000.0, 0.0, 0.0)),
            }],
            element_size: 5.0,
        };
        let r = crate::run(&bar(), &study).unwrap();
        // σ = F/A = 10 MPa; δ = FL/(EA) = 0.005 mm. The fixed end is clamped laterally too,
        // so the stress is uniform only away from it; the tip and the mid-length agree.
        let (dmax, at) = r.max_displacement();
        assert_relative_eq!(dmax, 0.005, max_relative = 0.02);
        assert_relative_eq!(at.x, 100.0);
        let mid = (0..r.mesh.elements.len())
            .find(|&e| (r.mesh.element_centre(e).x - 52.5).abs() < 1e-9)
            .unwrap();
        assert_relative_eq!(r.von_mises[mid], 10.0, max_relative = 0.02);
        assert_relative_eq!(r.reaction.x, -1000.0, max_relative = 1e-6);
        assert!(
            r.reaction.y.abs() < 1e-6 && r.reaction.z.abs() < 1e-6,
            "{}",
            r.reaction
        );
    }

    #[test]
    fn a_cantilever_bends_about_as_beam_theory_says() {
        let study = Study {
            material: Material::STEEL,
            fixed: vec![key(FaceRole::Side(3))],
            loads: vec![Load {
                face: key(FaceRole::Side(1)),
                kind: LoadKind::Force(Vec3::new(0.0, 0.0, -100.0)),
            }],
            element_size: 2.0,
        };
        let r = crate::run(&bar(), &study).unwrap();
        // δ = PL³/(3EI) with I = bh³/12 = 833.3: 0.2 mm, plus a little shear deflection.
        // Fully integrated bricks five deep are a few percent stiff.
        let (dmax, at) = r.max_displacement();
        assert!((dmax - 0.2).abs() / 0.2 < 0.1, "tip deflection {dmax}");
        assert_relative_eq!(at.x, 100.0);
        // Bending stress at the root, M c / I = 100·100·5/833.3 = 60 MPa; the brick
        // centres sit 1 mm in from the surface and the clamp disturbs the root, so look
        // one element in and accept the centre-of-brick value, 48 MPa, loosely.
        let (smax, _) = r.max_von_mises();
        assert!(smax > 40.0 && smax < 90.0, "max von Mises {smax}");
        assert_relative_eq!(r.reaction.z, 100.0, max_relative = 1e-6);
    }

    #[test]
    fn pressure_on_the_top_pushes_down() {
        let study = Study {
            material: Material::STEEL,
            fixed: vec![key(FaceRole::StartCap)],
            loads: vec![Load {
                face: key(FaceRole::EndCap),
                kind: LoadKind::Pressure(2.0),
            }],
            element_size: 5.0,
        };
        let r = crate::run(&bar(), &study).unwrap();
        // 2 MPa over 1000 mm² is 2000 N downwards; the reaction is upward.
        assert_relative_eq!(r.reaction.z, 2000.0, max_relative = 1e-6);
        assert!(r.max_displacement().1.z > 9.0);
    }

    #[test]
    fn mistakes_are_named() {
        let bad_face = FaceKey::new(OpId::new(9), FaceRole::EndCap);
        let study = Study {
            material: Material::STEEL,
            fixed: vec![bad_face],
            loads: vec![Load {
                face: key(FaceRole::EndCap),
                kind: LoadKind::Pressure(1.0),
            }],
            element_size: 5.0,
        };
        assert!(matches!(
            crate::run(&bar(), &study),
            Err(FeaError::FaceNotOnMesh { face, .. }) if face == bad_face
        ));
        let study = Study {
            fixed: vec![],
            ..study
        };
        assert_eq!(crate::run(&bar(), &study), Err(FeaError::NothingFixed));
    }
}
