//! Basic finite element analysis over a kernel [`Solid`].
//!
//! # What it is
//!
//! A linear elastic static study: a body of one isotropic material, some of its faces held
//! still, forces or pressures on others, and the question of how far it moves and how hard
//! it is stressed. The answer is a displacement at every node and a von Mises stress in
//! every element, with the maxima and where they are.
//!
//! # How it is built
//!
//! The mesh is a **voxel grid of identical bricks** ([`voxel`]), not a tetrahedral mesh
//! fitted to the surface. The solid is sampled at every cell centre of a grid laid over
//! its bounding box, by ray parity against its own tessellation, and every cell whose
//! centre is inside is an element. Boundary conditions are attached by kernel face: each
//! exposed brick facet is tagged with the [`FaceKey`] of the nearest face of the solid, so
//! a study names "the end cap of extrude 3" exactly as a fillet would, and survives the
//! same edits.
//!
//! The choice is deliberate. Fitting a tetrahedral mesh to a faceted boundary
//! representation that booleans have left agreeing only to a micron is the hard half of
//! a meshing library; a voxel grid cannot fail to mesh anything the kernel can tessellate,
//! and because every element is the same brick the stiffness matrix is computed **once**
//! ([`element::Brick`]) and never assembled: the solver ([`solve`]) multiplies by it
//! element by element inside a Jacobi-preconditioned conjugate gradient. The price is a
//! stair-stepped surface, which blurs stress concentrations at curved and oblique faces
//! and overstates the stiffness of thin sections a brick or two thick. Mesh finer to see
//! more; the element count is what the solve time follows.
//!
//! Units follow the rest of Basset: millimetres and newtons, so moduli and stresses are
//! in megapascals and a pressure is N/mm².
//!
//! A solve of any size takes a while, so every entry point has a `_with` twin that takes
//! an observer ([`run_with`], [`optimise_with`]): a closure handed a [`Progress`] every
//! so often that returns `false` to stop the work. [`topology`] builds on the same solver
//! to ask a different question: given this much material, where should it go.
//!
//! [`materials`] is a library of named engineering materials — the solver's two numbers
//! with a density and a yield strength beside them — so a study can be set up by name and
//! its result read as a mass and a safety factor.

pub mod element;
pub mod materials;
pub mod solve;
pub mod topology;
pub mod voxel;
pub mod vtk;

use basset_kernel::{FaceKey, Solid};
use basset_math::{TriMesh, Vec3};
use serde::{Deserialize, Serialize};

pub use materials::{MaterialGroup, MaterialSpec, find as find_material, library, safety_factor};
pub use topology::{TopologyResults, TopologyStudy, optimise, optimise_with};
pub use voxel::HexMesh;

/// Where a solve has got to. Handed to the observer of [`run_with`] and
/// [`optimise_with`] at every phase change and every few dozen conjugate gradient
/// iterations: often enough for a progress bar, rarely enough to cost nothing.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Progress {
    pub phase: Phase,
    /// Iterations done in this phase.
    pub step: usize,
    /// How many there will be at most, when that is known.
    pub of: Option<usize>,
    /// What the phase is driving down: the relative residual while solving, the
    /// compliance while optimising, zero while meshing.
    pub measure: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Phase {
    Meshing,
    Solving,
    Optimising,
}

/// Hands `progress` to the observer and turns a `false` into [`FeaError::Cancelled`],
/// so a solver loop can stop with a plain `?`.
pub(crate) fn report(
    observer: &mut dyn FnMut(&Progress) -> bool,
    progress: Progress,
) -> Result<(), FeaError> {
    if observer(&progress) {
        Ok(())
    } else {
        Err(FeaError::Cancelled)
    }
}

/// Isotropic linear elastic material. Steel by default. The named materials of
/// [`materials::library`] wrap one of these with a density and a yield strength.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Material {
    /// Young's modulus in MPa (N/mm²).
    pub youngs_modulus: f64,
    /// Poisson's ratio; must lie in `[0, 0.5)`.
    pub poisson_ratio: f64,
}

impl Material {
    /// Generic steel; the library's `Steel` entry carries exactly these numbers.
    pub const STEEL: Material = Material {
        youngs_modulus: 200_000.0,
        poisson_ratio: 0.3,
    };
    /// Generic aluminium; the library's `Aluminium` entry carries exactly these numbers.
    pub const ALUMINIUM: Material = Material {
        youngs_modulus: 69_000.0,
        poisson_ratio: 0.33,
    };
}

impl Default for Material {
    fn default() -> Self {
        Self::STEEL
    }
}

/// What a load on a face is.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum LoadKind {
    /// A total force in newtons, spread over the face in proportion to area.
    Force(Vec3),
    /// A pressure in MPa pushing *into* the face (against its outward normal). Negative
    /// pulls.
    Pressure(f64),
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Load {
    pub face: FaceKey,
    pub kind: LoadKind,
}

/// Everything a static study is, apart from the body it runs on.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Study {
    pub material: Material,
    /// Faces whose nodes are held in every direction.
    pub fixed: Vec<FaceKey>,
    pub loads: Vec<Load>,
    /// Target brick edge length in millimetres. Each axis is divided into whole bricks
    /// that fit the body's bounding box, so the actual size is within a half-brick of this.
    pub element_size: f64,
}

#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum FeaError {
    #[error("the body has no volume to mesh")]
    EmptySolid,
    #[error("element size must be positive, got {0}")]
    BadElementSize(f64),
    #[error("Young's modulus must be positive, got {0}")]
    BadModulus(f64),
    #[error("Poisson's ratio must lie in [0, 0.5), got {0}")]
    BadPoisson(f64),
    #[error(
        "the mesh would have {elements} elements, more than the limit of {limit}; use a larger element size"
    )]
    TooManyElements { elements: usize, limit: usize },
    #[error("no element centre fell inside the body at this element size; use a smaller one")]
    NothingMeshed,
    #[error("a study needs at least one fixed face")]
    NothingFixed,
    #[error("a study needs at least one load")]
    NothingLoaded,
    #[error(
        "no exposed element facet lies on face {face:?}; the faces the mesh touches are {touched:?}"
    )]
    FaceNotOnMesh {
        face: FaceKey,
        touched: Vec<FaceKey>,
    },
    #[error("every node is fixed; nothing is free to move")]
    EverythingFixed,
    #[error(
        "the solve did not converge in {iterations} iterations (relative residual {residual:.3e}); the fixed faces may leave the body free to move as a whole"
    )]
    DidNotConverge { iterations: usize, residual: f64 },
    #[error("the solve was cancelled")]
    Cancelled,
    #[error("{name} must lie in {range}, got {value}")]
    BadTopologyParameter {
        name: &'static str,
        range: &'static str,
        value: f64,
    },
}

/// The largest mesh a study will build. Beyond this the conjugate gradient would run for
/// minutes; the error says to coarsen rather than letting the machine go quiet.
pub const MAX_ELEMENTS: usize = 400_000;

/// The outcome of a study: the mesh it ran on and what moved and how hard.
#[derive(Debug, Clone, PartialEq)]
pub struct Results {
    pub mesh: HexMesh,
    pub material: Material,
    /// Displacement of every node, in millimetres.
    pub displacements: Vec<Vec3>,
    /// Von Mises stress at every element's centre, in MPa.
    pub von_mises: Vec<f64>,
    /// Von Mises stress at every node: the average of its elements, for a smooth plot.
    pub nodal_von_mises: Vec<f64>,
    /// Total reaction at the fixed nodes. Equal and opposite to the applied load when
    /// the solve is sound; a check the caller can print.
    pub reaction: Vec3,
    pub iterations: usize,
}

impl Results {
    pub fn max_displacement(&self) -> (f64, Vec3) {
        let (i, d) = self
            .displacements
            .iter()
            .enumerate()
            .map(|(i, d)| (i, d.length()))
            .fold(
                (0, 0.0),
                |best, cur| if cur.1 > best.1 { cur } else { best },
            );
        (d, self.mesh.nodes.get(i).copied().unwrap_or(Vec3::ZERO))
    }

    pub fn max_von_mises(&self) -> (f64, Vec3) {
        let (i, s) = self
            .von_mises
            .iter()
            .copied()
            .enumerate()
            .fold(
                (0, 0.0),
                |best, cur| if cur.1 > best.1 { cur } else { best },
            );
        (s, self.mesh.element_centre(i))
    }

    /// The exposed surface of the mesh, each node moved by `scale` times its displacement,
    /// with the nodal von Mises stress of every vertex alongside, in triangle order.
    /// `face_ids` carry the element each triangle belongs to.
    pub fn deformed_surface(&self, scale: f64) -> (TriMesh, Vec<f64>) {
        let mut mesh = TriMesh::default();
        let mut values = Vec::new();
        for facet in &self.mesh.facets {
            let p: Vec<Vec3> = facet
                .nodes
                .iter()
                .map(|&n| self.mesh.nodes[n] + self.displacements[n] * scale)
                .collect();
            for tri in [[0, 1, 2], [0, 2, 3]] {
                mesh.push_triangle([p[tri[0]], p[tri[1]], p[tri[2]]], facet.element as u32);
                values.extend(tri.iter().map(|&k| self.nodal_von_mises[facet.nodes[k]]));
            }
        }
        (mesh, values)
    }
}

/// Runs a study on a solid.
pub fn run(solid: &Solid, study: &Study) -> Result<Results, FeaError> {
    run_with(solid, study, &mut |_| true)
}

/// Runs a study on a solid, telling `observer` how it is getting on. The observer
/// returns `false` to stop, and the result is then [`FeaError::Cancelled`]. It is a plain
/// `FnMut` with no `Send` demanded of it: a caller that wants the solve off the UI thread
/// spawns the thread and hands in whatever channel it likes.
pub fn run_with(
    solid: &Solid,
    study: &Study,
    observer: &mut dyn FnMut(&Progress) -> bool,
) -> Result<Results, FeaError> {
    let mesh = mesh_for(solid, study, observer)?;
    solve::run_with(mesh, study, observer)
}

/// Checks a study's material and boundary conditions and meshes the solid: the part of a
/// static study and a topology optimisation that is the same.
pub(crate) fn mesh_for(
    solid: &Solid,
    study: &Study,
    observer: &mut dyn FnMut(&Progress) -> bool,
) -> Result<HexMesh, FeaError> {
    if study.material.youngs_modulus.is_nan() || study.material.youngs_modulus <= 0.0 {
        return Err(FeaError::BadModulus(study.material.youngs_modulus));
    }
    if !(0.0..0.5).contains(&study.material.poisson_ratio) {
        return Err(FeaError::BadPoisson(study.material.poisson_ratio));
    }
    if study.fixed.is_empty() {
        return Err(FeaError::NothingFixed);
    }
    if study.loads.is_empty() {
        return Err(FeaError::NothingLoaded);
    }
    report(
        observer,
        Progress {
            phase: Phase::Meshing,
            step: 0,
            of: None,
            measure: 0.0,
        },
    )?;
    voxel::mesh(solid, study.element_size)
}
