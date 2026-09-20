//! The small amount of dense linear algebra the solver needs, written in place so the
//! crate has no numeric dependency. Sketch systems are a few hundred unknowns at most,
//! so dense O(n³) routines are the simplest correct choice.

/// How far a column must move, per unit of motion of the free column that drives it, to
/// count as free in [`Mat::freedom`].
///
/// The null-space vector is built with the driving column set to exactly one, so this
/// thresholds a ratio: "how much this unknown moves per unit of motion of that one". A
/// component below it is elimination noise rather than a direction the geometry can
/// really take.
///
/// The ratio is measured on the *column-equilibrated* matrix (see [`Mat::equilibrate`]),
/// which is what makes the threshold mean the same thing whatever units and feature sizes
/// the sketch mixes. Raw columns do not: a residual normalised by a length, such as
/// `Angle` or `Parallel`, writes entries of order `1/L`, so a chain running from a 1000 mm
/// line down to a 1 mm detail divides a perfectly genuine motion by the size ratio at
/// every link, and a long enough chain pushes it under any absolute tolerance. Dividing
/// each column by its norm first cancels exactly that factor, because the product
/// `|xᵢ| · ‖Aᵢ‖` — motion times how hard the constraints resist it — is invariant when the
/// unknowns are rescaled.
const FREE_TOL: f64 = 1e-6;

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

    /// The rank, and which columns some solution of `A x = 0` moves — the unknowns the
    /// rows leave free.
    ///
    /// Rank is by Gaussian elimination with full pivoting; `rel_tol` is relative to the
    /// largest pivot, so the answer does not depend on the units of the rows.
    ///
    /// The rank alone says *how many* degrees of freedom remain; the columns say *where*
    /// they are, which is what lets the editor point at the geometry that is still loose
    /// instead of only counting it. A column with no pivot is free by construction, and
    /// so is every pivot column that a free one drags along, because moving the free
    /// unknown forces the pivot ones to follow. Both answers come out of one elimination
    /// because this runs on every solve, and a solve runs on every frame of a drag.
    pub fn freedom(&self, rel_tol: f64) -> (usize, Vec<bool>) {
        // Equilibrated, so that both the pivot choice and the null-space threshold see a
        // matrix whose columns are all the same size: the answer is then a property of
        // the geometry rather than of the units and feature sizes it is drawn at. Rank is
        // unaffected (scaling a column by a non-zero factor cannot change it); what
        // changes is which column full pivoting reaches for, which now follows dependence
        // rather than whichever feature happens to be drawn largest.
        let e = self.equilibrate().eliminate(rel_tol);
        let mut pivot_col = vec![false; self.cols];
        for &(_, c) in &e.pivots {
            pivot_col[c] = true;
        }
        let mut free = vec![false; self.cols];
        for f in (0..self.cols).filter(|c| !pivot_col[*c]) {
            // One null-space vector per free column: move that unknown by exactly one and
            // back-substitute what the pivot unknowns must do to keep every row at zero.
            // The pivots were chosen in order, so no pivot row has an entry in an earlier
            // pivot column and the substitution runs straight back up them.
            let mut x = vec![0.0; self.cols];
            x[f] = 1.0;
            for (i, &(r, c)) in e.pivots.iter().enumerate().rev() {
                let mut sum = e.m.at(r, f);
                for &(_, later) in &e.pivots[i + 1..] {
                    sum += e.m.at(r, later) * x[later];
                }
                x[c] = -sum / e.m.at(r, c);
            }
            for (c, v) in x.iter().enumerate() {
                free[c] |= v.abs() > FREE_TOL;
            }
        }
        (e.pivots.len(), free)
    }

    /// The same matrix with every non-zero column scaled to unit norm.
    ///
    /// Column equilibration is the standard cure for a Jacobian whose unknowns are in
    /// wildly different units or at wildly different sizes: it leaves the null space and
    /// the rank alone (up to the same scaling of each component) while putting every
    /// entry on one footing, so a magnitude comparison anywhere downstream — a pivot
    /// search, a tolerance — compares dependence rather than size. An all-zero column has
    /// no scale to normalise by and is left as it is; it is unconstrained, which the rank
    /// analysis already reads correctly.
    pub fn equilibrate(&self) -> Mat {
        let mut m = self.clone();
        for c in 0..self.cols {
            let norm = (0..self.rows)
                .map(|r| self.at(r, c) * self.at(r, c))
                .sum::<f64>()
                .sqrt();
            if norm == 0.0 || !norm.is_finite() {
                continue;
            }
            for r in 0..self.rows {
                *m.at_mut(r, c) /= norm;
            }
        }
        m
    }

    /// Which rows add nothing to the span of the groups of rows *before* them, i.e. are
    /// linearly dependent on them.
    ///
    /// Groups are taken in the caller's order; each row is orthogonalised against the
    /// basis as it stood before its own group started (modified Gram–Schmidt, run twice,
    /// because one pass loses orthogonality exactly when the rows are nearly dependent —
    /// the case being measured) and reported when the remainder has shrunk below
    /// `rel_tol` of its own norm. Rows are grouped rather than taken one by one so that a
    /// group is never measured against itself: two rows of one constraint that happen to
    /// agree make that constraint degenerate, not redundant against its neighbours.
    /// Within a group, only the rows that survive join the basis for later groups.
    ///
    /// Order decides *which* of a dependent set is named, and that is deliberate: the
    /// caller puts first the rows it wants treated as driving, so a family of
    /// interchangeable rows names every copy after the first rather than all of them.
    /// Rows outside every group take no part, in the basis or in the result.
    ///
    /// Works on the equilibrated matrix for the reason [`Self::freedom`] does: dependence
    /// is a property of the geometry, and a row scaled down by a long feature's length
    /// must not look dependent because of it.
    pub fn dependent_rows(&self, groups: &[std::ops::Range<usize>], rel_tol: f64) -> Vec<bool> {
        let m = self.equilibrate();
        let mut basis: Vec<Vec<f64>> = Vec::new();
        let mut dependent = vec![false; self.rows];
        for group in groups {
            let before = basis.len();
            for r in group.clone() {
                let row = &m.data[r * m.cols..(r + 1) * m.cols];
                let norm0 = norm(row);
                // A zero row constrains nothing, so it adds nothing to the span either:
                // dependent on whatever came before, vacuously.
                dependent[r] =
                    norm0 == 0.0 || norm(&reduce(row, &basis[..before])) <= rel_tol * norm0;
            }
            for r in group.clone().filter(|r| !dependent[*r]) {
                let mut row = reduce(&m.data[r * m.cols..(r + 1) * m.cols], &basis);
                let left = norm(&row);
                if left == 0.0 {
                    continue;
                }
                for a in row.iter_mut() {
                    *a /= left;
                }
                basis.push(row);
            }
        }
        dependent
    }

    /// Gaussian elimination with full pivoting: the reduced matrix and the
    /// `(row, column)` of each pivot, in the order they were taken.
    fn eliminate(&self, rel_tol: f64) -> Elimination {
        let mut m = self.clone();
        let (rows, cols) = (m.rows, m.cols);
        let mut pivots = Vec::new();
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
            pivots.push((pr, pc));
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
        Elimination { m, pivots }
    }
}

fn norm(v: &[f64]) -> f64 {
    v.iter().map(|a| a * a).sum::<f64>().sqrt()
}

/// `v` with its component along each (unit) basis vector removed, twice over.
fn reduce(v: &[f64], basis: &[Vec<f64>]) -> Vec<f64> {
    let mut out = v.to_vec();
    for _ in 0..2 {
        for b in basis {
            let d: f64 = out.iter().zip(b).map(|(a, c)| a * c).sum();
            for (a, c) in out.iter_mut().zip(b) {
                *a -= d * c;
            }
        }
    }
    out
}

/// The result of [`Mat::eliminate`], kept together because the pivot positions are
/// meaningless without the reduced matrix they index.
struct Elimination {
    m: Mat,
    pivots: Vec<(usize, usize)>,
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
        assert_eq!(full.freedom(1e-9).0, 2);
        let dep = Mat {
            rows: 3,
            cols: 2,
            data: vec![1.0, 2.0, 2.0, 4.0, 0.0, 1.0],
        };
        assert_eq!(dep.freedom(1e-9).0, 2);
        let zero = Mat::zeros(2, 2);
        assert_eq!(zero.freedom(1e-9).0, 0);
    }

    #[test]
    fn freedom_survives_a_thousandfold_rescaling_of_one_unknown() {
        // x1 is pinned to x0 (row 1) and x2 to x1 (row 2), each through a lever with a
        // 1e3 size ratio, so x2 moves 1e-6 per unit of x0 — under the raw threshold, and
        // not under one that measures each column against its own norm.
        let chain = Mat {
            rows: 2,
            cols: 3,
            data: vec![1e-3, 1.0, 0.0, 0.0, 1e-3, 1.0],
        };
        assert_eq!(chain.freedom(1e-9), (2, vec![true, true, true]));

        // The same system with the middle unknown measured in a different unit: a
        // different matrix, the same geometry, and so the same answer.
        let mut rescaled = chain.clone();
        for r in 0..rescaled.rows {
            *rescaled.at_mut(r, 1) *= 1e4;
        }
        assert_eq!(rescaled.freedom(1e-9), chain.freedom(1e-9));
    }

    #[test]
    fn dependent_rows_names_the_copy_and_not_the_original() {
        // Row 2 is row 0 doubled and row 3 is row 1 negated: both say what an earlier row
        // said, and it is the later of each pair that is named.
        let m = Mat {
            rows: 4,
            cols: 2,
            data: vec![1.0, 0.0, 0.0, 1.0, 2.0, 0.0, 0.0, -1.0],
        };
        let one_by_one: Vec<_> = (0..4).map(|r| r..r + 1).collect();
        assert_eq!(
            m.dependent_rows(&one_by_one, 1e-9),
            vec![false, false, true, true]
        );

        // Put the copies first and they become the drivers instead.
        let reversed: Vec<_> = (0..4).rev().map(|r| r..r + 1).collect();
        assert_eq!(
            m.dependent_rows(&reversed, 1e-9),
            vec![true, true, false, false]
        );

        // Grouped, the pair 2..4 is measured against 0..2 only, never against itself.
        assert_eq!(
            m.dependent_rows(&[0..2, 2..4], 1e-9),
            vec![false, false, true, true]
        );
        // A group whose second row repeats its first is degenerate in itself, not
        // dependent on what came before: nothing in the group is named, and the group
        // contributes the one direction it actually spans.
        let pair = Mat {
            rows: 4,
            cols: 2,
            data: vec![1.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 2.0],
        };
        assert_eq!(
            pair.dependent_rows(&[0..1, 1..3, 3..4], 1e-9),
            vec![false, false, false, true]
        );
    }

    #[test]
    fn dependence_is_not_a_matter_of_how_big_the_columns_are() {
        // Two rows saying the same thing about unknowns a thousand apart in scale: the
        // rows are proportional whatever the columns weigh.
        let m = Mat {
            rows: 2,
            cols: 2,
            data: vec![1e-3, 1e3, 2e-3, 2e3],
        };
        assert_eq!(m.dependent_rows(&[0..1, 1..2], 1e-9), vec![false, true]);
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
    #[test]
    fn free_columns_are_the_ones_the_rows_do_not_pin() {
        // x0 + x1 = 0 holds neither down: either can move if the other follows.
        let coupled = Mat {
            rows: 1,
            cols: 2,
            data: vec![1.0, 1.0],
        };
        assert_eq!(coupled.freedom(1e-9), (1, vec![true, true]));

        // Two rows pinning two of three unknowns: only the third is free, and it drags
        // nothing with it.
        let partly = Mat {
            rows: 2,
            cols: 3,
            data: vec![1.0, 0.0, 0.0, 0.0, 1.0, 0.0],
        };
        assert_eq!(partly.freedom(1e-9), (2, vec![false, false, true]));

        // A pivot column a free one drags along counts as free: x0 is pinned only
        // relative to x2.
        let dragged = Mat {
            rows: 2,
            cols: 3,
            data: vec![1.0, 0.0, -1.0, 0.0, 1.0, 0.0],
        };
        assert_eq!(dragged.freedom(1e-9), (2, vec![true, false, true]));

        // Nothing constrained at all, and nothing left free.
        assert_eq!(Mat::zeros(2, 2).freedom(1e-9), (0, vec![true, true]));
        let identity = Mat {
            rows: 2,
            cols: 2,
            data: vec![1.0, 0.0, 0.0, 1.0],
        };
        assert_eq!(identity.freedom(1e-9), (2, vec![false, false]));
    }
}
