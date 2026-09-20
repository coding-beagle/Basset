//! Forward-mode automatic differentiation.
//!
//! Every constraint residual is written once in terms of [`Dual`] and evaluated with the
//! constraint's local variables seeded as unit directions. The result carries the exact
//! partial derivatives with respect to those variables, which is what makes the
//! Levenberg–Marquardt Jacobian exact without ever hand-deriving a formula.
//!
//! The gradient has a fixed width so evaluation allocates nothing: no constraint in this
//! crate touches more than four points (eight scalars).

use std::ops::{Add, Div, Mul, Neg, Sub};

/// Maximum number of scalar variables a single constraint may reference.
pub const MAX_LOCAL_VARS: usize = 8;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Dual {
    pub v: f64,
    pub d: [f64; MAX_LOCAL_VARS],
}

impl Dual {
    pub const ZERO: Dual = Dual {
        v: 0.0,
        d: [0.0; MAX_LOCAL_VARS],
    };

    /// A constant: no dependence on any variable.
    pub fn constant(v: f64) -> Self {
        Self {
            v,
            d: [0.0; MAX_LOCAL_VARS],
        }
    }

    /// The `slot`-th local variable, seeded with a unit derivative.
    pub fn variable(v: f64, slot: usize) -> Self {
        let mut d = [0.0; MAX_LOCAL_VARS];
        d[slot] = 1.0;
        Self { v, d }
    }

    fn map(self, v: f64, scale: f64) -> Self {
        let mut d = self.d;
        for x in &mut d {
            *x *= scale;
        }
        Self { v, d }
    }

    /// Square root with a zero derivative at zero. The true derivative is infinite there;
    /// clamping it keeps a degenerate configuration (e.g. a zero-length line) from
    /// poisoning the whole Jacobian with NaN, letting the solver step away from it.
    pub fn sqrt(self) -> Self {
        if self.v <= 0.0 {
            return Self::constant(0.0);
        }
        let s = self.v.sqrt();
        self.map(s, 0.5 / s)
    }

    pub fn sin(self) -> Self {
        self.map(self.v.sin(), self.v.cos())
    }

    pub fn cos(self) -> Self {
        self.map(self.v.cos(), -self.v.sin())
    }

    /// `atan2(y, x)` with the standard partials `(x·dy − y·dx) / (x² + y²)`.
    pub fn atan2(y: Self, x: Self) -> Self {
        let denom = x.v * x.v + y.v * y.v;
        let v = y.v.atan2(x.v);
        if denom == 0.0 {
            return Self::constant(v);
        }
        let mut d = [0.0; MAX_LOCAL_VARS];
        for ((d, xd), yd) in d.iter_mut().zip(x.d).zip(y.d) {
            *d = (x.v * yd - y.v * xd) / denom;
        }
        Self { v, d }
    }

    pub fn is_finite(&self) -> bool {
        self.v.is_finite() && self.d.iter().all(|x| x.is_finite())
    }
}

impl Add for Dual {
    type Output = Dual;
    fn add(self, o: Dual) -> Dual {
        let mut d = self.d;
        for (a, b) in d.iter_mut().zip(o.d) {
            *a += b;
        }
        Dual { v: self.v + o.v, d }
    }
}

impl Sub for Dual {
    type Output = Dual;
    fn sub(self, o: Dual) -> Dual {
        let mut d = self.d;
        for (a, b) in d.iter_mut().zip(o.d) {
            *a -= b;
        }
        Dual { v: self.v - o.v, d }
    }
}

impl Mul for Dual {
    type Output = Dual;
    fn mul(self, o: Dual) -> Dual {
        let mut d = [0.0; MAX_LOCAL_VARS];
        for ((d, sd), od) in d.iter_mut().zip(self.d).zip(o.d) {
            *d = sd * o.v + self.v * od;
        }
        Dual { v: self.v * o.v, d }
    }
}

impl Div for Dual {
    type Output = Dual;
    fn div(self, o: Dual) -> Dual {
        let inv = 1.0 / o.v;
        let mut d = [0.0; MAX_LOCAL_VARS];
        for ((d, sd), od) in d.iter_mut().zip(self.d).zip(o.d) {
            *d = (sd * o.v - self.v * od) * inv * inv;
        }
        Dual { v: self.v * inv, d }
    }
}

impl Neg for Dual {
    type Output = Dual;
    fn neg(self) -> Dual {
        self.map(-self.v, -1.0)
    }
}

impl Add<f64> for Dual {
    type Output = Dual;
    fn add(self, o: f64) -> Dual {
        Dual {
            v: self.v + o,
            d: self.d,
        }
    }
}

impl Sub<f64> for Dual {
    type Output = Dual;
    fn sub(self, o: f64) -> Dual {
        Dual {
            v: self.v - o,
            d: self.d,
        }
    }
}

impl Mul<f64> for Dual {
    type Output = Dual;
    fn mul(self, o: f64) -> Dual {
        self.map(self.v * o, o)
    }
}

impl Div<f64> for Dual {
    type Output = Dual;
    fn div(self, o: f64) -> Dual {
        self.map(self.v / o, 1.0 / o)
    }
}

/// A 2D vector of duals; mirrors the handful of `Vec2` operations residuals need.
#[derive(Debug, Clone, Copy)]
pub struct DVec {
    pub x: Dual,
    pub y: Dual,
}

impl DVec {
    pub fn dot(self, o: DVec) -> Dual {
        self.x * o.x + self.y * o.y
    }

    /// Z component of the 3D cross product; positive when `o` is counter-clockwise of `self`.
    pub fn cross(self, o: DVec) -> Dual {
        self.x * o.y - self.y * o.x
    }

    pub fn length(self) -> Dual {
        self.dot(self).sqrt()
    }
}

impl Sub for DVec {
    type Output = DVec;
    fn sub(self, o: DVec) -> DVec {
        DVec {
            x: self.x - o.x,
            y: self.y - o.y,
        }
    }
}

impl Add for DVec {
    type Output = DVec;
    fn add(self, o: DVec) -> DVec {
        DVec {
            x: self.x + o.x,
            y: self.y + o.y,
        }
    }
}

impl Mul<f64> for DVec {
    type Output = DVec;
    fn mul(self, s: f64) -> DVec {
        DVec {
            x: self.x * s,
            y: self.y * s,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn derivatives_of_arithmetic() {
        let x = Dual::variable(3.0, 0);
        let y = Dual::variable(2.0, 1);
        let f = x * y + x / y - x * 2.0;
        assert_relative_eq!(f.v, 6.0 + 1.5 - 6.0);
        assert_relative_eq!(f.d[0], 2.0 + 0.5 - 2.0);
        assert_relative_eq!(f.d[1], 3.0 - 3.0 / 4.0);
    }

    #[test]
    fn derivatives_of_transcendentals() {
        let x = Dual::variable(0.7, 0);
        assert_relative_eq!(x.sin().d[0], 0.7f64.cos());
        assert_relative_eq!(x.cos().d[0], -0.7f64.sin());
        assert_relative_eq!(x.sqrt().d[0], 0.5 / 0.7f64.sqrt());
        let y = Dual::variable(0.3, 1);
        let a = Dual::atan2(y, x);
        let denom = 0.7 * 0.7 + 0.3 * 0.3;
        assert_relative_eq!(a.d[0], -0.3 / denom);
        assert_relative_eq!(a.d[1], 0.7 / denom);
    }

    #[test]
    fn sqrt_at_zero_is_finite() {
        let z = Dual::variable(0.0, 0).sqrt();
        assert!(z.is_finite());
        assert_eq!(z.d[0], 0.0);
    }
}
