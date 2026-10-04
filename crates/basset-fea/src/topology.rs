//! Density-based topology optimisation: given this much material, where should it go.
//!
//! # The method
//!
//! This is SIMP (Solid Isotropic Material with Penalisation), the textbook approach and
//! the one every commercial tool started from. Every element gets a density `ρ` between
//! almost nothing and one, its stiffness is scaled by `ρ^p`, and the densities are moved
//! to make the structure as stiff as possible under its loads — to minimise the
//! *compliance* `fᵀu` — while using only a given fraction of the design volume. The
//! penalty `p` (three, classically) makes an element of middling density poor value for
//! its volume, so the optimum is pushed towards every element being either solid or
//! empty: a shape, rather than a fog. Each step solves the static problem on the current
//! densities, differentiates the compliance with respect to each density
//! (`−p ρ^(p−1) uₑᵀ k₀ uₑ`, which costs nothing beyond the solve), smooths those
//! sensitivities over a neighbourhood a few elements wide, and takes an
//! *optimality-criteria* step: each density is scaled by a power of the ratio of its
//! sensitivity to a Lagrange multiplier, which a bisection sets so the volume comes out
//! on target. Move limits and damping stop the design lurching.
//!
//! The filter is the part that makes the answer worth looking at. Without it SIMP forms
//! checkerboards — alternating solid and empty elements that a trilinear brick thinks are
//! stiffer than they are — and the shape it finds depends on the mesh. Sigmund's
//! sensitivity filter, a density-weighted average of the sensitivities over a ball of
//! radius `filter_radius` elements, suppresses both and sets a minimum feature size.
//!
//! # Fitting it to the voxel solver
//!
//! Because the mesh is a regular grid of identical bricks, the solver needs only one
//! scale per element ([`crate::solve::System::scale`]) and the filter's neighbourhoods
//! come from grid indices rather than a search. The stiffness of an "empty" element is
//! not `ρ_min^p` but `ρ_min + (1 − ρ_min) ρ^p`: a floor of one part in a thousand rather
//! than one in a billion, because a matrix-free conjugate gradient crawls when the
//! stiffness contrast is that large and the difference to the answer is nil. Each solve
//! starts from the last one's displacements and stops at a looser tolerance than a static
//! study; the final solve on the finished densities is run tight so the stresses it
//! reports are as good as a static study's.
//!
//! Elements with a facet on a fixed or loaded face are *passive*: held at full density
//! so the optimiser cannot remove what the boundary conditions act on.

use std::collections::HashMap;

use basset_kernel::Solid;
use basset_math::{TriMesh, Vec3};

use crate::element::Brick;
use crate::solve::{self, Boundary, System};
use crate::voxel::{HexMesh, facet_corners};
use crate::{FeaError, Phase, Progress, Results, Study};

/// A static study plus what the optimiser may do with it.
#[derive(Debug, Clone, PartialEq)]
pub struct TopologyStudy {
    pub study: Study,
    /// Fraction of the meshed volume the result may use, in `(0, 1)`.
    pub volume_fraction: f64,
    /// The SIMP exponent. Three is the classic value; one would leave intermediate
    /// densities worth having.
    pub penalty: f64,
    /// Radius of the sensitivity filter, in multiples of the element size. 1.5 reaches an
    /// element's face and edge neighbours; anything below one does nothing.
    pub filter_radius: f64,
    /// Optimality-criteria updates to take, at most.
    pub iterations: usize,
    /// Smallest density an element may have, and the stiffness floor as a fraction of
    /// the solid stiffness.
    pub min_density: f64,
}

impl TopologyStudy {
    pub fn new(study: Study) -> Self {
        TopologyStudy {
            study,
            volume_fraction: 0.4,
            penalty: 3.0,
            filter_radius: 1.5,
            iterations: 40,
            min_density: 1e-3,
        }
    }

    fn check(&self) -> Result<(), FeaError> {
        let bad = |name, range, value| FeaError::BadTopologyParameter { name, range, value };
        if !(self.volume_fraction > 0.0 && self.volume_fraction < 1.0) {
            return Err(bad("volume fraction", "(0, 1)", self.volume_fraction));
        }
        if !(self.penalty >= 1.0 && self.penalty <= 10.0) {
            return Err(bad("penalty", "[1, 10]", self.penalty));
        }
        if !(self.filter_radius >= 0.0 && self.filter_radius <= 10.0) {
            return Err(bad("filter radius", "[0, 10]", self.filter_radius));
        }
        if !(self.min_density > 0.0 && self.min_density < 1.0) {
            return Err(bad("minimum density", "(0, 1)", self.min_density));
        }
        if self.iterations == 0 {
            return Err(bad("iterations", "[1, ∞)", 0.0));
        }
        Ok(())
    }

    /// Stiffness of an element of density `rho` as a fraction of a solid one.
    fn stiffness(&self, rho: f64) -> f64 {
        self.min_density + (1.0 - self.min_density) * rho.powf(self.penalty)
    }

    /// Derivative of [`Self::stiffness`].
    fn stiffness_slope(&self, rho: f64) -> f64 {
        (1.0 - self.min_density) * self.penalty * rho.powf(self.penalty - 1.0)
    }
}

/// Largest change in any density below which the design is taken as settled.
const CONVERGED: f64 = 0.01;
/// Most a density may change in one update.
const MOVE: f64 = 0.2;
/// Exponent on the optimality-criteria ratio; a half is the classic damping.
const DAMPING: f64 = 0.5;
/// Relative residual the inner solves stop at: the densities move by tenths, so the
/// displacements need not be known to nine places until the end.
const LOOSE_TOLERANCE: f64 = 1e-6;

/// What the optimiser found: a density per element and a static solve on that design.
#[derive(Debug, Clone, PartialEq)]
pub struct TopologyResults {
    pub mesh: HexMesh,
    /// Density of every element, from `min_density` to one.
    pub densities: Vec<f64>,
    /// Compliance `fᵀu` of the design at each solve, the first being the uniform start
    /// and the last the finished design.
    pub compliance: Vec<f64>,
    /// Mean density over the mesh: what the design actually uses of it.
    pub volume_fraction: f64,
    /// Optimality-criteria updates taken.
    pub iterations: usize,
    /// The static study solved on the finished densities, stresses scaled with
    /// stiffness, so a result can be plotted exactly as a static study's.
    pub results: Results,
}

impl TopologyResults {
    /// The exposed surface of the elements with density at least `threshold` — the
    /// mesh's own skin where it survives, and the new faces between kept and removed
    /// elements — with each vertex's density (the average over the kept elements that
    /// meet there) alongside in triangle order. `face_ids` carry the element each
    /// triangle belongs to, as [`Results::deformed_surface`] does.
    pub fn surface(&self, threshold: f64) -> (TriMesh, Vec<f64>) {
        let grid = Grid::new(&self.mesh);
        let kept: Vec<bool> = self.densities.iter().map(|&d| d >= threshold).collect();
        let mut nodal = vec![0.0; self.mesh.nodes.len()];
        let mut count = vec![0u32; self.mesh.nodes.len()];
        for (e, element) in self.mesh.elements.iter().enumerate() {
            if kept[e] {
                for &n in element {
                    nodal[n] += self.densities[e];
                    count[n] += 1;
                }
            }
        }
        for (v, c) in nodal.iter_mut().zip(&count) {
            if *c > 0 {
                *v /= *c as f64;
            }
        }
        let mut mesh = TriMesh::default();
        let mut values = Vec::new();
        for (e, element) in self.mesh.elements.iter().enumerate() {
            if !kept[e] {
                continue;
            }
            for (axis, positive) in [
                (0, false),
                (0, true),
                (1, false),
                (1, true),
                (2, false),
                (2, true),
            ] {
                if grid.neighbour(e, axis, positive).is_some_and(|n| kept[n]) {
                    continue;
                }
                let corners = facet_corners(axis, positive).map(|c| element[c]);
                let p = corners.map(|n| self.mesh.nodes[n]);
                for tri in [[0, 1, 2], [0, 2, 3]] {
                    mesh.push_triangle([p[tri[0]], p[tri[1]], p[tri[2]]], e as u32);
                    values.extend(tri.iter().map(|&k| nodal[corners[k]]));
                }
            }
        }
        (mesh, values)
    }

    /// Volume of the elements with density at least `threshold`.
    pub fn kept_volume(&self, threshold: f64) -> f64 {
        let size = self.mesh.size;
        self.densities.iter().filter(|&&d| d >= threshold).count() as f64 * size.x * size.y * size.z
    }
}

/// Optimises the material layout of a solid.
pub fn optimise(solid: &Solid, study: &TopologyStudy) -> Result<TopologyResults, FeaError> {
    optimise_with(solid, study, &mut |_| true)
}

/// Optimises the material layout of a solid, telling `observer` where it has got to: the
/// conjugate gradient of each solve reports as [`Phase::Solving`], and each finished
/// design as [`Phase::Optimising`] with its compliance as the measure. Returning `false`
/// gives [`FeaError::Cancelled`].
pub fn optimise_with(
    solid: &Solid,
    study: &TopologyStudy,
    observer: &mut dyn FnMut(&Progress) -> bool,
) -> Result<TopologyResults, FeaError> {
    study.check()?;
    let mesh = crate::mesh_for(solid, &study.study, observer)?;
    let Boundary { fixed, force } = solve::boundary(&mesh, &study.study)?;
    let brick = Brick::new(mesh.size, &study.study.material);
    let n = mesh.elements.len();

    // Elements under a boundary condition are the optimiser's to keep, not to question.
    let mut passive = vec![false; n];
    for facet in &mesh.facets {
        let bound = study.study.fixed.contains(&facet.face)
            || study.study.loads.iter().any(|l| l.face == facet.face);
        if bound {
            passive[facet.element] = true;
        }
    }
    let filter = Filter::new(&Grid::new(&mesh), study.filter_radius);

    let mut rho: Vec<f64> = passive
        .iter()
        .map(|&p| if p { 1.0 } else { study.volume_fraction })
        .collect();
    let mut u = vec![0.0; force.len()];
    let mut scale = vec![0.0; n];
    let mut sensitivity = vec![0.0; n];
    let mut filtered = vec![0.0; n];
    let mut compliance = Vec::new();
    let mut updates = 0;
    let mut settled = false;
    loop {
        for (s, &r) in scale.iter_mut().zip(&rho) {
            *s = study.stiffness(r);
        }
        let system = System {
            mesh: &mesh,
            brick: &brick,
            fixed: &fixed,
            scale: Some(&scale),
        };
        let last = updates == study.iterations || settled;
        let tolerance = if last {
            solve::TOLERANCE
        } else {
            LOOSE_TOLERANCE
        };
        solve::conjugate_gradient(&system, &force, &mut u, tolerance, observer)?;

        // Compliance and its gradient fall out of the element strain energies.
        let mut c = 0.0;
        for (e, element) in mesh.elements.iter().enumerate() {
            let energy = solve::energy(&brick, &solve::gather(element, &u));
            c += scale[e] * energy;
            sensitivity[e] = -study.stiffness_slope(rho[e]) * energy;
        }
        compliance.push(c);
        crate::report(
            observer,
            Progress {
                phase: Phase::Optimising,
                step: updates,
                of: Some(study.iterations),
                measure: c,
            },
        )?;
        if last {
            break;
        }

        filter.apply(&rho, &sensitivity, &mut filtered);
        let change = optimality_criteria(study, &passive, &filtered, &mut rho);
        updates += 1;
        settled = change < CONVERGED;
    }

    let volume_fraction = rho.iter().sum::<f64>() / n as f64;
    let results = solve::results(
        mesh.clone(),
        &study.study,
        &brick,
        &fixed,
        Some(&scale),
        &u,
        updates,
    );
    Ok(TopologyResults {
        mesh,
        densities: rho,
        compliance,
        volume_fraction,
        iterations: updates,
        results,
    })
}

/// One optimality-criteria update of `rho` in place, returning the largest change. The
/// Lagrange multiplier on the volume constraint is found by bisection: the volume is a
/// decreasing function of it, so halve an interval until the volume lands on target.
/// Passive elements take no part and stay at one.
fn optimality_criteria(
    study: &TopologyStudy,
    passive: &[bool],
    sensitivity: &[f64],
    rho: &mut [f64],
) -> f64 {
    let target = study.volume_fraction * rho.len() as f64;
    // The ratio −dc/λ is near one at the answer, so λ lives near the sensitivities'
    // scale; an upper bound a thousand times the largest is beyond any of them.
    let largest = sensitivity.iter().fold(0.0, |m: f64, &s| m.max(-s));
    let (mut low, mut high) = (0.0, (1e3 * largest).max(1e-300));
    let mut next = rho.to_vec();
    while (high - low) / (high + low) > 1e-4 {
        let lambda = 0.5 * (low + high);
        let mut volume = 0.0;
        for e in 0..rho.len() {
            next[e] = if passive[e] {
                1.0
            } else {
                let ratio = (-sensitivity[e] / lambda).max(0.0).powf(DAMPING);
                (rho[e] * ratio).clamp(
                    (rho[e] - MOVE).max(study.min_density),
                    (rho[e] + MOVE).min(1.0),
                )
            };
            volume += next[e];
        }
        if volume > target {
            low = lambda;
        } else {
            high = lambda;
        }
    }
    let mut change = 0.0f64;
    for (r, &n) in rho.iter_mut().zip(&next) {
        change = change.max((n - *r).abs());
        *r = n;
    }
    change
}

/// The voxel grid behind a mesh: which cell each element is, and which element is in each
/// cell, recovered from the element centres because the mesh keeps only nodes.
struct Grid {
    cells: Vec<[i64; 3]>,
    element_at: HashMap<[i64; 3], usize>,
}

impl Grid {
    fn new(mesh: &HexMesh) -> Grid {
        let origin = mesh
            .nodes
            .iter()
            .fold(Vec3::splat(f64::INFINITY), |m, &p| m.min(p));
        let cells: Vec<[i64; 3]> = (0..mesh.elements.len())
            .map(|e| {
                let c = (mesh.element_centre(e) - origin) / mesh.size;
                [c.x.floor() as i64, c.y.floor() as i64, c.z.floor() as i64]
            })
            .collect();
        let element_at = cells.iter().enumerate().map(|(e, &c)| (c, e)).collect();
        Grid { cells, element_at }
    }

    fn at(&self, cell: [i64; 3]) -> Option<usize> {
        self.element_at.get(&cell).copied()
    }

    /// The element sharing element `e`'s facet on one side, if there is one.
    fn neighbour(&self, e: usize, axis: usize, positive: bool) -> Option<usize> {
        let mut cell = self.cells[e];
        cell[axis] += if positive { 1 } else { -1 };
        self.at(cell)
    }
}

/// Sigmund's sensitivity filter: each element's sensitivity is replaced by the average
/// of its neighbours' within `radius` elements, weighted by their densities and by how
/// near they are. The neighbour lists are built once; the weight is `radius − distance`,
/// in element units, so bricks that are not cubes are treated by index, not length.
struct Filter {
    offsets: Vec<usize>,
    entries: Vec<(usize, f64)>,
}

impl Filter {
    fn new(grid: &Grid, radius: f64) -> Filter {
        let reach = radius.ceil() as i64;
        let mut offsets = Vec::with_capacity(grid.cells.len() + 1);
        let mut entries = Vec::new();
        offsets.push(0);
        for &cell in &grid.cells {
            for di in -reach..=reach {
                for dj in -reach..=reach {
                    for dk in -reach..=reach {
                        let d = ((di * di + dj * dj + dk * dk) as f64).sqrt();
                        if d >= radius && !(di == 0 && dj == 0 && dk == 0) {
                            continue;
                        }
                        if let Some(f) = grid.at([cell[0] + di, cell[1] + dj, cell[2] + dk]) {
                            // An element always sees itself, even with a radius of zero.
                            entries.push((f, (radius - d).max(f64::MIN_POSITIVE)));
                        }
                    }
                }
            }
            offsets.push(entries.len());
        }
        Filter { offsets, entries }
    }

    fn apply(&self, rho: &[f64], sensitivity: &[f64], out: &mut [f64]) {
        for e in 0..rho.len() {
            let (mut sum, mut weight) = (0.0, 0.0);
            for &(f, w) in &self.entries[self.offsets[e]..self.offsets[e + 1]] {
                sum += w * rho[f] * sensitivity[f];
                weight += w;
            }
            out[e] = sum / (weight * rho[e].max(1e-9));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Load, LoadKind, Material};
    use basset_kernel::{FaceKey, FaceRole, OpId, primitives};

    fn key(role: FaceRole) -> FaceKey {
        FaceKey::new(OpId::new(1), role)
    }

    /// A 40 × 12 × 2 slab along x, two bricks thick; `Side(3)` is its −x end and
    /// `Side(1)` its +x end. Two bricks thick keeps the solve a fraction of a second
    /// while still being a three-dimensional problem.
    fn slab() -> Solid {
        primitives::cuboid(OpId::new(1), Vec3::ZERO, Vec3::new(40.0, 12.0, 2.0))
    }

    fn cantilever() -> TopologyStudy {
        let study = Study {
            material: Material::STEEL,
            fixed: vec![key(FaceRole::Side(3))],
            loads: vec![Load {
                face: key(FaceRole::Side(1)),
                kind: LoadKind::Force(Vec3::new(0.0, -100.0, 0.0)),
            }],
            element_size: 1.0,
        };
        TopologyStudy {
            iterations: 15,
            ..TopologyStudy::new(study)
        }
    }

    #[test]
    fn a_cantilever_slab_stiffens_as_material_is_moved() {
        let study = cantilever();
        let r = optimise(&slab(), &study).unwrap();
        assert_eq!(r.mesh.elements.len(), 40 * 12 * 2);
        assert_eq!(r.densities.len(), r.mesh.elements.len());
        assert!((1..=15).contains(&r.iterations), "{}", r.iterations);
        assert_eq!(r.compliance.len(), r.iterations + 1);
        let (first, last) = (r.compliance[0], *r.compliance.last().unwrap());
        assert!(last < first, "compliance {first} -> {last}");
        assert!(
            (r.volume_fraction - 0.4).abs() < 0.02 * 0.4,
            "volume fraction {}",
            r.volume_fraction
        );
        // Everything the clamp and the load touch is still there.
        for facet in &r.mesh.facets {
            if facet.face == key(FaceRole::Side(3)) || facet.face == key(FaceRole::Side(1)) {
                assert_eq!(r.densities[facet.element], 1.0);
            }
        }
        for &d in &r.densities {
            assert!((study.min_density..=1.0).contains(&d), "{d}");
        }
        // The static solve on the result is a real one: the reaction carries the load.
        assert!(
            (r.results.reaction.y - 100.0).abs() < 1e-3,
            "{}",
            r.results.reaction
        );
        assert!(r.results.max_von_mises().0 > 0.0);

        let (surface, values) = r.surface(0.5);
        assert!(surface.triangle_count() > 0);
        assert_eq!(values.len(), 3 * surface.triangle_count());
        assert!(values.iter().all(|v| (0.5..=1.0).contains(v)));
        let kept = r.kept_volume(0.5);
        let total = r.mesh.volume();
        assert!(
            kept > 0.25 * total && kept < 0.6 * total,
            "kept {kept} of {total}"
        );
    }

    #[test]
    fn the_observer_can_stop_it() {
        let study = cantilever();
        let mut optimising = 0;
        let err = optimise_with(&slab(), &study, &mut |p| {
            if p.phase == Phase::Optimising {
                optimising += 1;
            }
            optimising < 3
        })
        .unwrap_err();
        assert_eq!(err, FeaError::Cancelled);
        assert_eq!(optimising, 3);
    }

    #[test]
    fn parameters_are_checked() {
        let study = TopologyStudy {
            volume_fraction: 1.5,
            ..cantilever()
        };
        assert!(matches!(
            optimise(&slab(), &study),
            Err(FeaError::BadTopologyParameter {
                name: "volume fraction",
                ..
            })
        ));
    }
}
