//! The small amount of dense linear algebra the solver needs, written in place so the
//! crate has no numeric dependency. Sketch systems are a few hundred unknowns at most,
//! so dense O(n³) routines are the simplest correct choice.

/// Dense row-major matrix.
#[derive(Debug, Clone, PartialEq)]
pub struct Mat {
    pub rows: usize,
    pub cols: usize,
    pub data: Vec<f64>,
}

impl Mat {
    pub fn zeros(rows: usize, cols: usize) -> Self {
        Self {
            rows,
            cols,
            data: vec![0.0; rows * cols],
        }
    }

    #[inline]
    pub fn at(&self, r: usize, c: usize) -> f64 {
        self.data[r * self.cols + c]
    }

    #[inline]
    pub fn at_mut(&mut self, r: usize, c: usize) -> &mut f64 {
        &mut self.data[r * self.cols + c]
    }

    /// `Aᵀ A`, the normal-equations matrix.
    pub fn gram(&self) -> Mat {
        let n = self.cols;
        let mut g = Mat::zeros(n, n);
        for r in 0..self.rows {
            let row = &self.data[r * n..(r + 1) * n];
            for i in 0..n {
                if row[i] == 0.0 {
                    continue;
                }
                for j in i..n {
                    g.data[i * n + j] += row[i] * row[j];
                }
            }
        }
        for i in 0..n {
            for j in 0..i {
                g.data[i * n + j] = g.data[j * n + i];
            }
        }
        g
    }

    /// `Aᵀ v`.
    pub fn transpose_mul_vec(&self, v: &[f64]) -> Vec<f64> {
        let mut out = vec![0.0; self.cols];
        for (row, vr) in self.data.chunks_exact(self.cols).zip(v) {
            for (o, a) in out.iter_mut().zip(row) {
                *o += a * vr;
            }
        }
        out
    }

    /// Numerical rank by Gaussian elimination with full pivoting. `rel_tol` is relative to
    /// the largest pivot, so the answer does not depend on the units of the rows.
    pub fn rank(&self, rel_tol: f64) -> usize {
        let mut m = self.clone();
        let (rows, cols) = (m.rows, m.cols);
        let mut rank = 0;
        let mut first_pivot = None;
        let mut row_used = vec![false; rows];
        let mut col_used = vec![false; cols];
        loop {
            let mut best = (0.0f64, 0, 0);
            for (r, _) in row_used.iter().enumerate().filter(|(_, used)| !**used) {
                for (c, _) in col_used.iter().enumerate().filter(|(_, used)| !**used) {
                    if m.at(r, c).abs() > best.0 {
                        best = (m.at(r, c).abs(), r, c);
                    }
                }
            }
            let (pivot, pr, pc) = best;
            let threshold = first_pivot.map_or(0.0, |p: f64| p * rel_tol);
            if pivot <= threshold || pivot == 0.0 {
                break;
            }
            first_pivot.get_or_insert(pivot);
            rank += 1;
            row_used[pr] = true;
            col_used[pc] = true;
            let pv = m.at(pr, pc);
            for r in (0..rows).filter(|&r| r != pr && !row_used[r]) {
                let f = m.at(r, pc) / pv;
                if f == 0.0 {
                    continue;
                }
                for c in 0..cols {
                    let sub = f * m.at(pr, c);
                    *m.at_mut(r, c) -= sub;
                }
            }
        }
        rank
    }
}

/// Solves `A x = b` for symmetric positive-definite `A` by Cholesky decomposition.
/// Returns `None` if `A` is not positive-definite (the caller then raises the damping).
pub fn solve_spd(a: &Mat, b: &[f64]) -> Option<Vec<f64>> {
    let n = a.rows;
    debug_assert_eq!(a.cols, n);
    let mut l = vec![0.0; n * n];
    for i in 0..n {
        for j in 0..=i {
            let mut sum = a.at(i, j);
            for k in 0..j {
                sum -= l[i * n + k] * l[j * n + k];
            }
            if i == j {
                if sum <= 0.0 || !sum.is_finite() {
                    return None;
                }
                l[i * n + i] = sum.sqrt();
            } else {
                l[i * n + j] = sum / l[j * n + j];
            }
        }
    }
    // Forward substitution L y = b, then back substitution Lᵀ x = y.
    let mut y = vec![0.0; n];
    for i in 0..n {
        let mut s = b[i];
        for k in 0..i {
            s -= l[i * n + k] * y[k];
        }
        y[i] = s / l[i * n + i];
    }
    let mut x = vec![0.0; n];
    for i in (0..n).rev() {
        let mut s = y[i];
        for k in i + 1..n {
            s -= l[k * n + i] * x[k];
        }
        x[i] = s / l[i * n + i];
    }
    Some(x)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn cholesky_solves_spd_system() {
        let a = Mat {
            rows: 2,
            cols: 2,
            data: vec![4.0, 2.0, 2.0, 3.0],
        };
        let x = solve_spd(&a, &[2.0, 1.0]).expect("spd");
        assert_relative_eq!(4.0 * x[0] + 2.0 * x[1], 2.0, epsilon = 1e-12);
        assert_relative_eq!(2.0 * x[0] + 3.0 * x[1], 1.0, epsilon = 1e-12);
    }

    #[test]
    fn cholesky_rejects_indefinite() {
        let a = Mat {
            rows: 2,
            cols: 2,
            data: vec![1.0, 2.0, 2.0, 1.0],
        };
        assert!(solve_spd(&a, &[1.0, 1.0]).is_none());
    }

    #[test]
    fn rank_detects_dependent_rows() {
        let full = Mat {
            rows: 2,
            cols: 3,
            data: vec![1.0, 0.0, 0.0, 0.0, 1.0, 0.0],
        };
        assert_eq!(full.rank(1e-9), 2);
        let dep = Mat {
            rows: 3,
            cols: 2,
            data: vec![1.0, 2.0, 2.0, 4.0, 0.0, 1.0],
        };
        assert_eq!(dep.rank(1e-9), 2);
        let zero = Mat::zeros(2, 2);
        assert_eq!(zero.rank(1e-9), 0);
    }

    #[test]
    fn gram_matches_manual_product() {
        let a = Mat {
            rows: 2,
            cols: 2,
            data: vec![1.0, 2.0, 3.0, 4.0],
        };
        let g = a.gram();
        assert_eq!(g.data, vec![10.0, 14.0, 14.0, 20.0]);
        assert_eq!(a.transpose_mul_vec(&[1.0, 1.0]), vec![4.0, 6.0]);
    }
}
