//! The eight-node brick: shape functions, strain-displacement matrix, stiffness.
//!
//! Every element of a voxel mesh is the same brick, so one stiffness matrix serves them
//! all and nothing is ever assembled. The stiffness uses full 2×2×2 Gauss integration;
//! stresses are evaluated at the brick's centre, where the trilinear element is most
//! accurate. A fully integrated brick is known to be too stiff in bending when the mesh
//! is one or two elements thick — expect a cantilever to come out a few percent stiffer
//! than beam theory on a coarse mesh and to converge as the mesh is refined.

use basset_math::Vec3;

use crate::Material;

/// Degrees of freedom of one brick: three per node.
pub const DOF: usize = 24;

/// Signs of each corner's natural coordinates, in [`crate::voxel::CORNERS`] order.
const SIGNS: [[f64; 3]; 8] = [
    [-1.0, -1.0, -1.0],
    [1.0, -1.0, -1.0],
    [1.0, 1.0, -1.0],
    [-1.0, 1.0, -1.0],
    [-1.0, -1.0, 1.0],
    [1.0, -1.0, 1.0],
    [1.0, 1.0, 1.0],
    [-1.0, 1.0, 1.0],
];

/// A brick of fixed size and material: its stiffness and the strain operator at its centre.
#[derive(Debug, Clone, PartialEq)]
pub struct Brick {
    pub stiffness: Box<[[f64; DOF]; DOF]>,
    /// Strain–displacement matrix at the centre, Voigt order xx, yy, zz, xy, yz, zx with
    /// engineering shear strains.
    pub b_centre: [[f64; DOF]; 6],
    /// Isotropic elasticity matrix in the same Voigt order.
    pub d: [[f64; 6]; 6],
}

impl Brick {
    pub fn new(size: Vec3, material: &Material) -> Brick {
        let d = elasticity(material);
        let mut k = Box::new([[0.0; DOF]; DOF]);
        let g = 1.0 / 3f64.sqrt();
        let det_j = size.x * size.y * size.z / 8.0;
        for &xi in &[-g, g] {
            for &eta in &[-g, g] {
                for &zeta in &[-g, g] {
                    let b = strain_matrix(size, [xi, eta, zeta]);
                    // k += Bᵀ D B · detJ (unit Gauss weights)
                    let mut db = [[0.0; DOF]; 6];
                    for (r, db_row) in db.iter_mut().enumerate() {
                        for c in 0..DOF {
                            db_row[c] = (0..6).map(|m| d[r][m] * b[m][c]).sum();
                        }
                    }
                    for (r, k_row) in k.iter_mut().enumerate() {
                        for c in 0..DOF {
                            k_row[c] += det_j * (0..6).map(|m| b[m][r] * db[m][c]).sum::<f64>();
                        }
                    }
                }
            }
        }
        Brick {
            stiffness: k,
            b_centre: strain_matrix(size, [0.0; 3]),
            d,
        }
    }

    /// Stress at the centre of a brick whose corner displacements are `u`, Voigt order.
    pub fn stress(&self, u: &[f64; DOF]) -> [f64; 6] {
        let mut strain = [0.0; 6];
        for (r, s) in strain.iter_mut().enumerate() {
            *s = (0..DOF).map(|c| self.b_centre[r][c] * u[c]).sum();
        }
        let mut stress = [0.0; 6];
        for (r, s) in stress.iter_mut().enumerate() {
            *s = (0..6).map(|m| self.d[r][m] * strain[m]).sum();
        }
        stress
    }
}

/// The isotropic Hooke matrix.
fn elasticity(m: &Material) -> [[f64; 6]; 6] {
    let (e, nu) = (m.youngs_modulus, m.poisson_ratio);
    let lambda = e * nu / ((1.0 + nu) * (1.0 - 2.0 * nu));
    let mu = e / (2.0 * (1.0 + nu));
    let mut d = [[0.0; 6]; 6];
    for i in 0..3 {
        d[i][..3].fill(lambda);
        d[i][i] = lambda + 2.0 * mu;
        d[i + 3][i + 3] = mu;
    }
    d
}

/// B at natural coordinates `n`, for a brick whose Jacobian is the constant diagonal
/// `size / 2`.
fn strain_matrix(size: Vec3, n: [f64; 3]) -> [[f64; DOF]; 6] {
    let mut b = [[0.0; DOF]; 6];
    let scale = [2.0 / size.x, 2.0 / size.y, 2.0 / size.z];
    for (i, s) in SIGNS.iter().enumerate() {
        let dn = [
            0.125 * s[0] * (1.0 + s[1] * n[1]) * (1.0 + s[2] * n[2]) * scale[0],
            0.125 * s[1] * (1.0 + s[0] * n[0]) * (1.0 + s[2] * n[2]) * scale[1],
            0.125 * s[2] * (1.0 + s[0] * n[0]) * (1.0 + s[1] * n[1]) * scale[2],
        ];
        let c = 3 * i;
        b[0][c] = dn[0];
        b[1][c + 1] = dn[1];
        b[2][c + 2] = dn[2];
        b[3][c] = dn[1];
        b[3][c + 1] = dn[0];
        b[4][c + 1] = dn[2];
        b[4][c + 2] = dn[1];
        b[5][c] = dn[2];
        b[5][c + 2] = dn[0];
    }
    b
}

/// Von Mises equivalent stress of a Voigt stress vector.
pub fn von_mises(s: &[f64; 6]) -> f64 {
    let [sx, sy, sz, txy, tyz, tzx] = *s;
    (0.5 * ((sx - sy).powi(2) + (sy - sz).powi(2) + (sz - sx).powi(2))
        + 3.0 * (txy * txy + tyz * tyz + tzx * tzx))
        .max(0.0)
        .sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn stiffness_is_symmetric_and_annihilates_rigid_motion() {
        let brick = Brick::new(Vec3::new(1.0, 2.0, 0.5), &Material::STEEL);
        let k = &brick.stiffness;
        for r in 0..DOF {
            for c in 0..DOF {
                assert_relative_eq!(k[r][c], k[c][r], max_relative = 1e-9, epsilon = 1e-6);
            }
        }
        // A uniform translation along x produces no force.
        let mut u = [0.0; DOF];
        for i in 0..8 {
            u[3 * i] = 1.0;
        }
        for row in k.iter() {
            let f: f64 = (0..DOF).map(|c| row[c] * u[c]).sum();
            assert!(f.abs() < 1e-6, "{f}");
        }
    }

    #[test]
    fn uniform_strain_gives_hookes_law() {
        let brick = Brick::new(Vec3::ONE, &Material::STEEL);
        // Stretch 1% along x with free lateral contraction ν·1%: uniaxial stress E·1%.
        let mut u = [0.0; DOF];
        for (i, s) in SIGNS.iter().enumerate() {
            u[3 * i] = if s[0] > 0.0 { 0.01 } else { 0.0 };
            u[3 * i + 1] = if s[1] > 0.0 { -0.003 } else { 0.0 };
            u[3 * i + 2] = if s[2] > 0.0 { -0.003 } else { 0.0 };
        }
        let s = brick.stress(&u);
        assert_relative_eq!(s[0], 2000.0, max_relative = 1e-9);
        assert!(s[1].abs() < 1e-6 && s[2].abs() < 1e-6, "{s:?}");
        assert_relative_eq!(von_mises(&s), 2000.0, max_relative = 1e-9);
    }
}
