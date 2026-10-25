//! Kleine, abhängigkeitsfreie Vektor- und Lineare-Algebra-Hilfen.

use std::ops::{Add, AddAssign, Div, Mul, Neg, Sub, SubAssign};

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct V3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[inline]
pub const fn v3(x: f64, y: f64, z: f64) -> V3 {
    V3 { x, y, z }
}

impl V3 {
    pub const ZERO: V3 = v3(0.0, 0.0, 0.0);
    pub const X: V3 = v3(1.0, 0.0, 0.0);
    pub const Y: V3 = v3(0.0, 1.0, 0.0);
    pub const Z: V3 = v3(0.0, 0.0, 1.0);

    #[inline]
    pub fn dot(self, o: V3) -> f64 {
        self.x * o.x + self.y * o.y + self.z * o.z
    }
    #[inline]
    pub fn cross(self, o: V3) -> V3 {
        v3(
            self.y * o.z - self.z * o.y,
            self.z * o.x - self.x * o.z,
            self.x * o.y - self.y * o.x,
        )
    }
    #[inline]
    pub fn norm2(self) -> f64 {
        self.dot(self)
    }
    #[inline]
    pub fn norm(self) -> f64 {
        self.norm2().sqrt()
    }
    #[inline]
    pub fn normalized(self) -> V3 {
        let n = self.norm();
        if n > 1e-300 {
            self / n
        } else {
            V3::Z
        }
    }
    #[inline]
    pub fn dist(self, o: V3) -> f64 {
        (self - o).norm()
    }
    #[inline]
    pub fn get(self, i: usize) -> f64 {
        match i {
            0 => self.x,
            1 => self.y,
            _ => self.z,
        }
    }
    #[inline]
    pub fn min(self, o: V3) -> V3 {
        v3(self.x.min(o.x), self.y.min(o.y), self.z.min(o.z))
    }
    #[inline]
    pub fn max(self, o: V3) -> V3 {
        v3(self.x.max(o.x), self.y.max(o.y), self.z.max(o.z))
    }
    pub fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }
    /// Ein beliebiger Einheitsvektor senkrecht zu `self` (self normiert).
    pub fn any_perp(self) -> V3 {
        let a = if self.x.abs() < 0.6 {
            V3::X
        } else if self.y.abs() < 0.6 {
            V3::Y
        } else {
            V3::Z
        };
        self.cross(a).normalized()
    }
    /// Orthonormalbasis (e1, e2) senkrecht zu self.
    pub fn basis(self) -> (V3, V3) {
        let e1 = self.any_perp();
        let e2 = self.cross(e1).normalized();
        (e1, e2)
    }
}

impl Add for V3 {
    type Output = V3;
    #[inline]
    fn add(self, o: V3) -> V3 {
        v3(self.x + o.x, self.y + o.y, self.z + o.z)
    }
}
impl AddAssign for V3 {
    #[inline]
    fn add_assign(&mut self, o: V3) {
        *self = *self + o;
    }
}
impl Sub for V3 {
    type Output = V3;
    #[inline]
    fn sub(self, o: V3) -> V3 {
        v3(self.x - o.x, self.y - o.y, self.z - o.z)
    }
}
impl SubAssign for V3 {
    #[inline]
    fn sub_assign(&mut self, o: V3) {
        *self = *self - o;
    }
}
impl Mul<f64> for V3 {
    type Output = V3;
    #[inline]
    fn mul(self, s: f64) -> V3 {
        v3(self.x * s, self.y * s, self.z * s)
    }
}
impl Mul<V3> for f64 {
    type Output = V3;
    #[inline]
    fn mul(self, v: V3) -> V3 {
        v * self
    }
}
impl Div<f64> for V3 {
    type Output = V3;
    #[inline]
    fn div(self, s: f64) -> V3 {
        v3(self.x / s, self.y / s, self.z / s)
    }
}
impl Neg for V3 {
    type Output = V3;
    #[inline]
    fn neg(self) -> V3 {
        v3(-self.x, -self.y, -self.z)
    }
}

/// Symmetrische 3x3-Matrix (zeilenweise).
#[derive(Clone, Copy, Debug, Default)]
pub struct M3 {
    pub m: [[f64; 3]; 3],
}

impl M3 {
    pub fn zero() -> M3 {
        M3::default()
    }
    pub fn add_outer(&mut self, a: V3, w: f64) {
        for i in 0..3 {
            for j in 0..3 {
                self.m[i][j] += w * a.get(i) * a.get(j);
            }
        }
    }
    pub fn mul_v(&self, v: V3) -> V3 {
        v3(
            self.m[0][0] * v.x + self.m[0][1] * v.y + self.m[0][2] * v.z,
            self.m[1][0] * v.x + self.m[1][1] * v.y + self.m[1][2] * v.z,
            self.m[2][0] * v.x + self.m[2][1] * v.y + self.m[2][2] * v.z,
        )
    }

    /// Eigenzerlegung einer symmetrischen Matrix (Jacobi).
    /// Liefert Eigenwerte aufsteigend und zugehörige Eigenvektoren.
    pub fn eigen_sym(&self) -> ([f64; 3], [V3; 3]) {
        let mut a = self.m;
        let mut v = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        for _sweep in 0..50 {
            let off = a[0][1].abs() + a[0][2].abs() + a[1][2].abs();
            let scale = a[0][0].abs() + a[1][1].abs() + a[2][2].abs() + 1e-300;
            if off <= 1e-15 * scale {
                break;
            }
            for (p, q) in [(0usize, 1usize), (0, 2), (1, 2)] {
                if a[p][q].abs() < 1e-300 {
                    continue;
                }
                let theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
                let t = theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt());
                let t = if theta == 0.0 { 1.0 } else { t };
                let c = 1.0 / (t * t + 1.0).sqrt();
                let s = t * c;
                for k in 0..3 {
                    let akp = a[k][p];
                    let akq = a[k][q];
                    a[k][p] = c * akp - s * akq;
                    a[k][q] = s * akp + c * akq;
                }
                for k in 0..3 {
                    let apk = a[p][k];
                    let aqk = a[q][k];
                    a[p][k] = c * apk - s * aqk;
                    a[q][k] = s * apk + c * aqk;
                }
                for row in v.iter_mut() {
                    let vkp = row[p];
                    let vkq = row[q];
                    row[p] = c * vkp - s * vkq;
                    row[q] = s * vkp + c * vkq;
                }
            }
        }
        let mut idx = [0usize, 1, 2];
        idx.sort_by(|&i, &j| a[i][i].partial_cmp(&a[j][j]).unwrap_or(std::cmp::Ordering::Equal));
        let vals = [a[idx[0]][idx[0]], a[idx[1]][idx[1]], a[idx[2]][idx[2]]];
        let vecs = [
            v3(v[0][idx[0]], v[1][idx[0]], v[2][idx[0]]).normalized(),
            v3(v[0][idx[1]], v[1][idx[1]], v[2][idx[1]]).normalized(),
            v3(v[0][idx[2]], v[1][idx[2]], v[2][idx[2]]).normalized(),
        ];
        (vals, vecs)
    }
}

/// Löst ein kleines lineares Gleichungssystem A x = b (Gauß mit Pivotsuche).
/// Gibt None zurück, wenn die Matrix (numerisch) singulär ist.
pub fn solve(mut a: Vec<Vec<f64>>, mut b: Vec<f64>) -> Option<Vec<f64>> {
    let n = b.len();
    let mut maxabs: f64 = 0.0;
    for row in &a {
        for &x in row {
            maxabs = maxabs.max(x.abs());
        }
    }
    if maxabs == 0.0 || !maxabs.is_finite() {
        return None;
    }
    for col in 0..n {
        let mut piv = col;
        for r in col + 1..n {
            if a[r][col].abs() > a[piv][col].abs() {
                piv = r;
            }
        }
        if a[piv][col].abs() < 1e-13 * maxabs {
            return None;
        }
        a.swap(col, piv);
        b.swap(col, piv);
        for r in col + 1..n {
            let f = a[r][col] / a[col][col];
            if f != 0.0 {
                for c in col..n {
                    a[r][c] -= f * a[col][c];
                }
                b[r] -= f * b[col];
            }
        }
    }
    let mut x = vec![0.0; n];
    for r in (0..n).rev() {
        let mut s = b[r];
        for c in r + 1..n {
            s -= a[r][c] * x[c];
        }
        x[r] = s / a[r][r];
    }
    if x.iter().all(|v| v.is_finite()) {
        Some(x)
    } else {
        None
    }
}

/// Winkel zwischen zwei Einheitsvektoren in Radiant.
#[inline]
pub fn angle(a: V3, b: V3) -> f64 {
    a.dot(b).clamp(-1.0, 1.0).acos()
}
