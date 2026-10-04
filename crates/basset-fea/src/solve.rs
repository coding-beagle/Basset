//! Boundary conditions and the linear solve.
//!
//! The stiffness matrix is never formed. Every element is the same [`Brick`], so the
//! product `K·u` is a gather, a 24×24 multiply and a scatter per element, and a
//! Jacobi-preconditioned conjugate gradient needs nothing more. Fixed degrees of freedom
//! are handled by masking: they are dropped from the residual and never move, which is
//! the same as deleting their rows and columns.
//!
//! The pieces are public to the crate rather than one function because topology
//! optimisation ([`crate::topology`]) solves the same system dozens of times with a
//! different stiffness scale per element each time, and wants the boundary conditions
//! built once, the solve warm-started from the last displacement, and the stresses
//! recovered only at the end.

use basset_math::Vec3;

use crate::element::{Brick, DOF, von_mises};
use crate::voxel::HexMesh;
use crate::{FeaError, LoadKind, Phase, Progress, Results, Study};

/// Relative residual at which a static solve's conjugate gradient stops.
pub(crate) const TOLERANCE: f64 = 1e-9;

/// How often the conjugate gradient tells its observer where it is. An iteration is a
/// few milliseconds on a modest mesh; every twenty-five is often enough for a bar and
/// rarely enough to be free.
const REPORT_EVERY: usize = 25;

pub fn run(mesh: HexMesh, study: &Study) -> Result<Results, FeaError> {
    run_with(mesh, study, &mut |_| true)
}

pub fn run_with(
    mesh: HexMesh,
    study: &Study,
    observer: &mut dyn FnMut(&Progress) -> bool,
) -> Result<Results, FeaError> {
    let Boundary { fixed, force } = boundary(&mesh, study)?;
    let brick = Brick::new(mesh.size, &study.material);
    let system = System {
        mesh: &mesh,
        brick: &brick,
        fixed: &fixed,
        scale: None,
    };
    let mut u = vec![0.0; force.len()];
    let iterations = conjugate_gradient(&system, &force, &mut u, TOLERANCE, observer)?;
    Ok(results(mesh, study, &brick, &fixed, None, &u, iterations))
}

/// What a study does to a mesh: which degrees of freedom are held and the force at each.
pub(crate) struct Boundary {
    pub fixed: Vec<bool>,
    pub force: Vec<f64>,
}

/// Attaches a study's fixed faces and loads to the mesh by face key.
pub(crate) fn boundary(mesh: &HexMesh, study: &Study) -> Result<Boundary, FeaError> {
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
    Ok(Boundary { fixed, force })
}

/// Reactions, stresses and nodal averages from a converged displacement `u`. With a
/// `scale`, each element's stress is scaled with its stiffness, so an element that
/// topology optimisation has all but removed carries all but no stress.
pub(crate) fn results(
    mesh: HexMesh,
    study: &Study,
    brick: &Brick,
    fixed: &[bool],
    scale: Option<&[f64]>,
    u: &[f64],
    iterations: usize,
) -> Results {
    let system = System {
        mesh: &mesh,
        brick,
        fixed,
        scale,
    };
    // Reactions are what the fixed nodes push back with: the full product at those rows.
    let mut ku = vec![0.0; u.len()];
    system.multiply(u, &mut ku, false);
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
    for (e, element) in mesh.elements.iter().enumerate() {
        let ue = gather(element, u);
        let s = von_mises(&brick.stress(&ue)) * scale.map_or(1.0, |s| s[e]);
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

    Results {
        mesh,
        material: study.material,
        displacements,
        von_mises: von,
        nodal_von_mises: nodal,
        reaction,
        iterations,
    }
}

/// The linear system `K·u = f` with fixed degrees of freedom masked out. With a `scale`,
/// element `e` contributes `scale[e]` times the shared brick stiffness, which is how
/// topology optimisation softens the elements it is removing without a second stiffness
/// matrix; a static study passes `None` and pays one multiply by 1.0 per row for the
/// generality, nothing beside the 24-wide dot product next to it.
pub(crate) struct System<'a> {
    pub mesh: &'a HexMesh,
    pub brick: &'a Brick,
    pub fixed: &'a [bool],
    pub scale: Option<&'a [f64]>,
}

impl System<'_> {
    /// `out = K·u`. With `masked`, fixed rows and columns are treated as deleted.
    pub(crate) fn multiply(&self, u: &[f64], out: &mut [f64], masked: bool) {
        out.fill(0.0);
        let k = &self.brick.stiffness;
        for (e, element) in self.mesh.elements.iter().enumerate() {
            let scale = self.scale.map_or(1.0, |s| s[e]);
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
                    out[g] += scale * k[row].iter().zip(&ue).map(|(a, b)| a * b).sum::<f64>();
                }
            }
        }
    }

    /// The diagonal of the masked system, for the Jacobi preconditioner.
    fn diagonal(&self) -> Vec<f64> {
        let mut diag = vec![0.0; self.fixed.len()];
        let k = &self.brick.stiffness;
        for (e, element) in self.mesh.elements.iter().enumerate() {
            let scale = self.scale.map_or(1.0, |s| s[e]);
            for (i, &n) in element.iter().enumerate() {
                for d in 0..3 {
                    diag[3 * n + d] += scale * k[3 * i + d][3 * i + d];
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

pub(crate) fn gather(element: &[usize; 8], u: &[f64]) -> [f64; DOF] {
    let mut ue = [0.0; DOF];
    for (i, &n) in element.iter().enumerate() {
        ue[3 * i..3 * i + 3].copy_from_slice(&u[3 * n..3 * n + 3]);
    }
    ue
}

/// Twice the strain energy of one unscaled brick with corner displacements `ue`:
/// `ueᵀ·k₀·ue`, the quantity every compliance sensitivity is made of.
pub(crate) fn energy(brick: &Brick, ue: &[f64; DOF]) -> f64 {
    brick
        .stiffness
        .iter()
        .zip(ue)
        .map(|(row, &ui)| ui * row.iter().zip(ue).map(|(a, b)| a * b).sum::<f64>())
        .sum()
}

fn dot(a: &[f64], b: &[f64]) -> f64 {
    a.iter().zip(b).map(|(x, y)| x * y).sum()
}

/// Jacobi-preconditioned conjugate gradient on the masked system, starting from whatever
/// `u` holds (zero for a cold start, the last answer for a warm one) and stopping when
/// the residual relative to the load falls below `tolerance`. Returns the iterations
/// taken. The observer hears from it every [`REPORT_EVERY`] iterations and may stop it.
pub(crate) fn conjugate_gradient(
    system: &System,
    force: &[f64],
    u: &mut [f64],
    tolerance: f64,
    observer: &mut dyn FnMut(&Progress) -> bool,
) -> Result<usize, FeaError> {
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
    if norm_b == 0.0 {
        u.fill(0.0);
        return Ok(0);
    }
    for (v, &f) in u.iter_mut().zip(fixed) {
        if f {
            *v = 0.0;
        }
    }
    // r = b − K·u₀; for a cold start that is b itself.
    let mut r = b;
    if u.iter().any(|&v| v != 0.0) {
        let mut ku = vec![0.0; n];
        system.multiply(u, &mut ku, true);
        for (r, k) in r.iter_mut().zip(&ku) {
            *r -= k;
        }
    }
    let mut z: Vec<f64> = r.iter().zip(&inv_diag).map(|(r, d)| r * d).collect();
    let mut p = z.clone();
    let mut rz = dot(&r, &z);
    let mut q = vec![0.0; n];
    let progress = |step, residual| Progress {
        phase: Phase::Solving,
        step,
        of: None,
        measure: residual,
    };
    crate::report(observer, progress(0, dot(&r, &r).sqrt() / norm_b))?;
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
        if residual < tolerance {
            return Ok(it);
        }
        if it % REPORT_EVERY == 0 {
            crate::report(observer, progress(it, residual))?;
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
    fn progress_is_reported_and_can_cancel() {
        let study = Study {
            material: Material::STEEL,
            fixed: vec![key(FaceRole::Side(3))],
            loads: vec![Load {
                face: key(FaceRole::Side(1)),
                kind: LoadKind::Force(Vec3::new(0.0, 0.0, -100.0)),
            }],
            element_size: 5.0,
        };
        let mut seen = Vec::new();
        let r = crate::run_with(&bar(), &study, &mut |p| {
            seen.push(*p);
            true
        })
        .unwrap();
        assert_eq!(seen[0].phase, crate::Phase::Meshing);
        let solving: Vec<_> = seen
            .iter()
            .filter(|p| p.phase == crate::Phase::Solving)
            .collect();
        // Step 0 and then every 25 iterations, the residual falling as it goes.
        assert!(solving.len() >= 2, "{seen:?}");
        assert_eq!(solving[0].step, 0);
        assert_eq!(solving[1].step, 25);
        assert!(solving.last().unwrap().measure < solving[0].measure);
        assert!(solving.last().unwrap().step <= r.iterations);

        // Saying stop after the mesh is built gives Cancelled, not a result.
        let mut calls = 0;
        let err = crate::run_with(&bar(), &study, &mut |_| {
            calls += 1;
            calls < 2
        })
        .unwrap_err();
        assert_eq!(err, FeaError::Cancelled);
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
