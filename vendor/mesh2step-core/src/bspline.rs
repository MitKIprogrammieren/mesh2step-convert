//! Freiformflächen: bikubische B-Spline-Fläche als Höhenfeld über einer Bezugsebene.
//!
//! S(u,v) = o + u·e1 + v·e2 + h(u,v)·n, wobei h ein Tensorprodukt-B-Spline ist.
//! Da kubische B-Splines mit Greville-Abszissen lineare Funktionen exakt darstellen,
//! ist S selbst eine gewöhnliche B-Spline-Fläche (Kontrollpunkte o + gu·e1 + gv·e2 + c·n).

use crate::math::V3;
use std::sync::{Arc, OnceLock, RwLock};

pub const DEG: usize = 3;

#[derive(Debug)]
pub struct BSpline {
    pub o: V3,
    pub e1: V3,
    pub e2: V3,
    pub n: V3,
    /// Knotenvektoren (geklemmt, inkl. Wiederholungen)
    pub ku: Vec<f64>,
    pub kv: Vec<f64>,
    pub nu: usize,
    pub nv: usize,
    /// Höhen der Kontrollpunkte, Index i*nv + j
    pub c: Vec<f64>,
}

static REG: OnceLock<RwLock<Vec<Arc<BSpline>>>> = OnceLock::new();

fn reg() -> &'static RwLock<Vec<Arc<BSpline>>> {
    REG.get_or_init(|| RwLock::new(Vec::new()))
}

pub fn register(b: BSpline) -> u32 {
    let mut r = reg().write().unwrap();
    r.push(Arc::new(b));
    (r.len() - 1) as u32
}

pub fn get(id: u32) -> Arc<BSpline> {
    reg().read().unwrap()[id as usize].clone()
}

/// Geklemmter, gleichmäßiger Knotenvektor über [a, b] mit `nseg` Segmenten.
pub fn knots(a: f64, b: f64, nseg: usize) -> Vec<f64> {
    let mut k = vec![a; DEG + 1];
    for i in 1..nseg {
        k.push(a + (b - a) * i as f64 / nseg as f64);
    }
    k.extend(std::iter::repeat(b).take(DEG + 1));
    k
}

/// Basisfunktionen (Wert, 1. Ableitung) an t; liefert ersten Kontrollpunkt-Index.
pub fn basis(k: &[f64], t: f64) -> (usize, [f64; 4], [f64; 4]) {
    let n = k.len() - DEG - 1; // Anzahl Kontrollpunkte
    let t = t.clamp(k[DEG], k[n]);
    // Spanne s mit k[s] <= t < k[s+1], s in [DEG, n-1]
    let mut s = DEG;
    while s + 1 < n && t >= k[s + 1] {
        s += 1;
    }
    let div = |a: f64, b: f64| if b.abs() > 1e-300 { a / b } else { 0.0 };
    let mut nb = [0.0f64; 4];
    nb[0] = 1.0;
    let mut n2 = [0.0f64; 4];
    for p in 1..=DEG {
        let mut new = [0.0f64; 4];
        for r in 0..=p {
            let i = s + r - p; // Funktionsindex
            let left = if r >= 1 { nb[r - 1] } else { 0.0 };
            let right = if r < p { nb[r] } else { 0.0 };
            new[r] = div(t - k[i], k[i + p] - k[i]) * left + div(k[i + p + 1] - t, k[i + p + 1] - k[i + 1]) * right;
        }
        if p == DEG - 1 {
            n2 = new;
        }
        nb = new;
    }
    let mut d = [0.0f64; 4];
    for r in 0..=DEG {
        let i = s + r - DEG;
        let left = if r >= 1 { n2[r - 1] } else { 0.0 };
        let right = if r < DEG { n2[r] } else { 0.0 };
        d[r] = DEG as f64 * (div(left, k[i + DEG] - k[i]) - div(right, k[i + DEG + 1] - k[i + 1]));
    }
    (s - DEG, nb, d)
}

/// Greville-Abszisse des Kontrollpunkts i.
pub fn greville(k: &[f64], i: usize) -> f64 {
    (k[i + 1] + k[i + 2] + k[i + 3]) / 3.0
}

impl BSpline {
    pub fn uv(&self, x: V3) -> (f64, f64) {
        let d = x - self.o;
        (d.dot(self.e1), d.dot(self.e2))
    }

    /// Höhe und partielle Ableitungen.
    pub fn eval(&self, u: f64, v: f64) -> (f64, f64, f64) {
        let (iu, bu, du) = basis(&self.ku, u);
        let (iv, bv, dv) = basis(&self.kv, v);
        let (mut h, mut hu, mut hv) = (0.0, 0.0, 0.0);
        for a in 0..4 {
            let row = (iu + a) * self.nv + iv;
            for b in 0..4 {
                let c = self.c[row + b];
                h += bu[a] * bv[b] * c;
                hu += du[a] * bv[b] * c;
                hv += bu[a] * dv[b] * c;
            }
        }
        (h, hu, hv)
    }

    /// Liegt der Punkt (projiziert) im Parameterbereich der Fläche?
    pub fn inside(&self, u: f64, v: f64) -> bool {
        let (u0, u1) = (self.ku[0], self.ku[self.ku.len() - 1]);
        let (v0, v1) = (self.kv[0], self.kv[self.kv.len() - 1]);
        u >= u0 && u <= u1 && v >= v0 && v <= v1
    }

    pub fn dist(&self, x: V3) -> f64 {
        let (u, v) = self.uv(x);
        if !self.inside(u, v) {
            return 1e6; // außerhalb des Flächenbereichs: passt nie
        }
        let (h, hu, hv) = self.eval(u, v);
        ((x - self.o).dot(self.n) - h) / (1.0 + hu * hu + hv * hv).sqrt()
    }

    pub fn normal(&self, x: V3) -> V3 {
        let (u, v) = self.uv(x);
        let (_, hu, hv) = self.eval(u, v);
        (self.n - self.e1 * hu - self.e2 * hv).normalized()
    }

    /// Kontrollpunkte im Raum (Index i*nv + j).
    pub fn control_points(&self) -> Vec<V3> {
        let mut out = Vec::with_capacity(self.nu * self.nv);
        for i in 0..self.nu {
            let gu = greville(&self.ku, i);
            for j in 0..self.nv {
                let gv = greville(&self.kv, j);
                out.push(self.o + self.e1 * gu + self.e2 * gv + self.n * self.c[i * self.nv + j]);
            }
        }
        out
    }
}

/// Löst ein symmetrisch positiv definites Bandsystem (obere Bandspeicherung a[k][d] = A[k][k+d]).
fn band_cholesky_solve(mut a: Vec<Vec<f64>>, mut b: Vec<f64>, bw: usize) -> Option<Vec<f64>> {
    let n = b.len();
    // Zerlegung A = Uᵀ U
    for k in 0..n {
        let mut s = a[k][0];
        let lo = k.saturating_sub(bw);
        for i in lo..k {
            let u = a[i][k - i];
            s -= u * u;
        }
        if s <= 1e-300 {
            return None;
        }
        let d = s.sqrt();
        a[k][0] = d;
        let hi = (k + bw).min(n - 1);
        for j in k + 1..=hi {
            let mut s = a[k][j - k];
            let lo2 = j.saturating_sub(bw).max(lo);
            for i in lo2..k {
                s -= a[i][k - i] * a[i][j - i];
            }
            a[k][j - k] = s / d;
        }
    }
    // Vorwärts: Uᵀ y = b
    for k in 0..n {
        let mut s = b[k];
        let lo = k.saturating_sub(bw);
        for i in lo..k {
            s -= a[i][k - i] * b[i];
        }
        b[k] = s / a[k][0];
    }
    // Rückwärts: U x = y
    for k in (0..n).rev() {
        let mut s = b[k];
        let hi = (k + bw).min(n - 1);
        for j in k + 1..=hi {
            s -= a[k][j - k] * b[j];
        }
        b[k] = s / a[k][0];
    }
    if b.iter().all(|x| x.is_finite()) {
        Some(b)
    } else {
        None
    }
}

/// Gewichtete Kleinste-Quadrate-Anpassung der Höhen auf festen Knoten.
/// `data` = (u, v, h, Gewicht); `lam` = Glättungsgewicht (zweite Differenzen).
#[allow(clippy::too_many_arguments)]
fn solve_on_knots(o: V3, e1: V3, e2: V3, n: V3, ku: &[f64], kv: &[f64], data: &[(f64, f64, f64, f64)], lam: f64) -> Option<BSpline> {
    let nu = ku.len() - DEG - 1;
    let nv = kv.len() - DEG - 1;
    let nunk = nu * nv;
    let bw = DEG * nv + DEG;
    let mut a = vec![vec![0.0f64; bw + 1]; nunk];
    let mut b = vec![0.0f64; nunk];
    for &(u, v, h, wt) in data {
        let (iu, bu, _) = basis(ku, u);
        let (iv, bv, _) = basis(kv, v);
        let mut idx = [0usize; 16];
        let mut w = [0.0f64; 16];
        let mut c = 0;
        for p in 0..4 {
            for q in 0..4 {
                idx[c] = (iu + p) * nv + iv + q;
                w[c] = bu[p] * bv[q];
                c += 1;
            }
        }
        for x in 0..16 {
            b[idx[x]] += wt * w[x] * h;
            for y in 0..16 {
                if idx[y] >= idx[x] {
                    a[idx[x]][idx[y] - idx[x]] += wt * w[x] * w[y];
                }
            }
        }
    }
    let mut add_diff = |ids: [usize; 3]| {
        let w = [1.0, -2.0, 1.0];
        for x in 0..3 {
            for y in 0..3 {
                if ids[y] >= ids[x] {
                    a[ids[x]][ids[y] - ids[x]] += lam * w[x] * w[y];
                }
            }
        }
    };
    for i in 0..nu {
        for j in 0..nv {
            if i + 2 < nu {
                add_diff([i * nv + j, (i + 1) * nv + j, (i + 2) * nv + j]);
            }
            if j + 2 < nv {
                add_diff([i * nv + j, i * nv + j + 1, i * nv + j + 2]);
            }
        }
    }
    for row in a.iter_mut() {
        row[0] += 1e-12;
    }
    let c = band_cholesky_solve(a, b, bw)?;
    Some(BSpline { o, e1, e2, n, ku: ku.to_vec(), kv: kv.to_vec(), nu, nv, c })
}

/// Zieht eine Freiformfläche an ihre Randkanten heran: `hard` (Abtastpunkte der Kanten) gehen
/// mit sehr hohem Gewicht ein, `soft` (Netzpunkte der Region) normal. Reicht die Knotenauflösung
/// nicht, wird verfeinert. Liefert None, wenn `hard_tol` bzw. `soft_tol` nicht erreichbar sind.
/// Rückgabe: (Fläche, größte Randabweichung) – bei Nichterreichen die beste gefundene Näherung.
pub fn snap_to_boundary(s: &BSpline, soft: &[V3], hard: &[V3], soft_tol: f64, hard_tol: f64) -> Option<(BSpline, f64)> {
    let (u0, u1) = (s.ku[0], s.ku[s.ku.len() - 1]);
    let (v0, v1) = (s.kv[0], s.kv[s.kv.len() - 1]);
    let to_uvh = |p: V3| {
        let d = p - s.o;
        (d.dot(s.e1), d.dot(s.e2), d.dot(s.n))
    };
    let hard_uvh: Vec<(f64, f64, f64)> = hard.iter().map(|&p| to_uvh(p)).collect();
    // Randpunkte außerhalb des Parameterbereichs: Fläche kann sie nie erreichen
    if hard_uvh.iter().any(|&(u, v, _)| u < u0 || u > u1 || v < v0 || v > v1) {
        return None;
    }
    let mut data: Vec<(f64, f64, f64, f64)> = soft
        .iter()
        .map(|&p| to_uvh(p))
        .filter(|&(u, v, _)| u >= u0 && u <= u1 && v >= v0 && v <= v1)
        .map(|(u, v, h)| (u, v, h, 1.0))
        .collect();
    let nsoft = data.len();
    // Anker: Gitter der Originalfläche über den ganzen Parameterbereich mit kleinem Gewicht.
    // Hält die Fläche in Bereichen ohne Netzpunkte (außerhalb des Trimmrands, Taschen) ruhig –
    // sonst schwingt sie dort weit aus und CAD-Kernel projizieren die Kanten falsch.
    let ng = 48;
    let mut anchors = Vec::with_capacity((ng + 1) * (ng + 1));
    for i in 0..=ng {
        for j in 0..=ng {
            let u = u0 + (u1 - u0) * i as f64 / ng as f64;
            let v = v0 + (v1 - v0) * j as f64 / ng as f64;
            anchors.push((u, v, s.eval(u, v).0));
        }
    }
    let w_anchor = 0.05 * (nsoft.max(1) as f64 / anchors.len() as f64).max(0.05);
    data.extend(anchors.iter().map(|&(u, v, h)| (u, v, h, w_anchor)));
    let w_hard = 1e4 * (nsoft.max(1) as f64 / hard_uvh.len().max(1) as f64).max(1.0);
    data.extend(hard_uvh.iter().map(|&(u, v, h)| (u, v, h, w_hard)));
    let worst0 = hard.iter().map(|&p| s.dist(p).abs()).fold(0.0, f64::max);
    let fm: f64 = std::env::var("M2S_SNAPMOVE").ok().and_then(|x| x.parse().ok()).unwrap_or(8.0);
    let move_lim = (worst0 * fm).max(soft_tol * 2.0);
    let (mut nsu, mut nsv) = (s.nu - DEG, s.nv - DEG);
    let mut best: Option<(BSpline, f64)> = None;
    for _ in 0..7 {
        let ku = knots(u0, u1, nsu);
        let kv = knots(v0, v1, nsv);
        let nunk = (nsu + DEG) * (nsv + DEG);
        // Glättung: zweite Differenzen des Kontrollnetzes, skaliert mit der Knotenweite
        // (feinere Knoten → kleinere Differenzen für dieselbe Krümmung)
        let fl: f64 = std::env::var("M2S_SNAPLAM").ok().and_then(|x| x.parse().ok()).unwrap_or(0.01);
        let h = ((u1 - u0) / nsu as f64).min((v1 - v0) / nsv as f64).max(1e-9);
        let lam = fl * nsoft.max(1) as f64 / nunk as f64 / (h * h);
        if let Some(t) = solve_on_knots(s.o, s.e1, s.e2, s.n, &ku, &kv, &data, lam) {
            let mh = hard.iter().map(|&p| t.dist(p).abs()).fold(0.0, f64::max);
            let ms = soft.iter().map(|&p| t.dist(p).abs()).filter(|d| *d < 1e5).fold(0.0, f64::max);
            // Gesamte Fläche darf sich nur so weit ändern, wie es der Rand erfordert
            let ma = anchors.iter().map(|&(u, v, h)| (t.eval(u, v).0 - h).abs()).fold(0.0, f64::max);
            let ms = ms.max(if ma > move_lim { f64::MAX } else { 0.0 });
            if std::env::var("M2S_DEBUG").is_ok() {
                eprintln!("  snap {nsu}x{nsv}: Rand {mh:.5} (≤{hard_tol:.5}), Netz {ms:.5} (≤{soft_tol:.5})");
            }
            if ms <= soft_tol {
                if mh <= hard_tol {
                    return Some((t, mh));
                }
                if best.as_ref().map_or(true, |b| mh < b.1) {
                    best = Some((t, mh));
                }
            }
        }
        if nsu >= 80 && nsv >= 80 {
            break;
        }
        nsu = (nsu * 3 / 2 + 1).min(80);
        nsv = (nsv * 3 / 2 + 1).min(80);
    }
    best
}

/// Passt eine Freiformfläche an Punkte an (Bezugsrahmen vorgegeben).
/// `check` sind die Punkte, die innerhalb `tol` liegen müssen (meist = pts).
/// `bpts` sind Randpunkte (Ecken der späteren B-Rep-Kanten): stark gewichtet und auf
/// `bvtol` genau, damit die Kanten ohne Nacharbeit auf der Fläche liegen.
pub fn fit(pts: &[V3], mids: &[V3], bmids: &[V3], o: V3, e1: V3, e2: V3, n: V3, tol: f64, mtol: f64, btol: f64, bpts: &[V3], bvtol: f64) -> Option<BSpline> {
    if pts.len() < 16 {
        return None;
    }
    const BW: f64 = 25.0;
    let uv: Vec<(f64, f64, f64)> = pts
        .iter()
        .map(|&p| {
            let d = p - o;
            (d.dot(e1), d.dot(e2), d.dot(n))
        })
        .collect();
    let (mut u0, mut u1, mut v0, mut v1) = (f64::MAX, f64::MIN, f64::MAX, f64::MIN);
    for &(u, v, _) in &uv {
        u0 = u0.min(u);
        u1 = u1.max(u);
        v0 = v0.min(v);
        v1 = v1.max(v);
    }
    let (mu, mv) = ((u1 - u0) * 0.03 + 1e-6, (v1 - v0) * 0.03 + 1e-6);
    let (u0, u1, v0, v1) = (u0 - mu, u1 + mu, v0 - mv, v1 + mv);
    let (lu, lv) = (u1 - u0, v1 - v0);
    let lmax = lu.max(lv);

    let mut ns = ((pts.len() as f64).sqrt() / 4.0).round().clamp(3.0, 12.0) as usize;
    loop {
        let nsu = ((ns as f64 * lu / lmax).round() as usize).max(2);
        let nsv = ((ns as f64 * lv / lmax).round() as usize).max(2);
        let ku = knots(u0, u1, nsu);
        let kv = knots(v0, v1, nsv);
        let nu = nsu + DEG;
        let nv = nsv + DEG;
        let nunk = nu * nv;
        if nunk > pts.len() * 3 + 64 {
            if std::env::var("M2S_DEBUG").is_ok() {
                eprintln!("  bspline: {} Punkte, zu wenige für ns {ns}", pts.len());
            }
            return None;
        }
        let bw = DEG * nv + DEG;
        let mut lam_mul = 1.0;
        let mut maxd;
        loop {
        let mut a = vec![vec![0.0f64; bw + 1]; nunk];
        let mut b = vec![0.0f64; nunk];
        let buv = bpts.iter().map(|&p| {
            let d = p - o;
            (d.dot(e1), d.dot(e2), d.dot(n), BW)
        });
        for (u, v, h, wt) in uv.iter().map(|&(u, v, h)| (u, v, h, 1.0)).chain(buv) {
            let (iu, bu, _) = basis(&ku, u);
            let (iv, bv, _) = basis(&kv, v);
            let mut idx = [0usize; 16];
            let mut w = [0.0f64; 16];
            let mut c = 0;
            for p in 0..4 {
                for q in 0..4 {
                    idx[c] = (iu + p) * nv + iv + q;
                    w[c] = bu[p] * bv[q];
                    c += 1;
                }
            }
            for x in 0..16 {
                b[idx[x]] += wt * w[x] * h;
                for y in 0..16 {
                    if idx[y] >= idx[x] {
                        a[idx[x]][idx[y] - idx[x]] += wt * w[x] * w[y];
                    }
                }
            }
        }
        // Glättung (zweite Differenzen) – füllt datenfreie Bereiche (Taschen, Bohrungen) ruhig aus
        let lam = lam_mul * 1e-5 * (pts.len() as f64 / nunk as f64).max(0.1);
        let mut add_diff = |ids: [usize; 3]| {
            let w = [1.0, -2.0, 1.0];
            for x in 0..3 {
                for y in 0..3 {
                    if ids[y] >= ids[x] {
                        a[ids[x]][ids[y] - ids[x]] += lam * w[x] * w[y];
                    }
                }
            }
        };
        for i in 0..nu {
            for j in 0..nv {
                if i + 2 < nu {
                    add_diff([i * nv + j, (i + 1) * nv + j, (i + 2) * nv + j]);
                }
                if j + 2 < nv {
                    add_diff([i * nv + j, i * nv + j + 1, i * nv + j + 2]);
                }
            }
        }
        for row in a.iter_mut() {
            row[0] += 1e-12;
        }
        let c = band_cholesky_solve(a, b, bw)?;
        let s = BSpline { o, e1, e2, n, ku: ku.clone(), kv: kv.clone(), nu, nv, c };
        maxd = pts.iter().map(|&p| s.dist(p).abs()).fold(0.0, f64::max);
        let maxbv = bpts.iter().map(|&p| s.dist(p).abs()).fold(0.0, f64::max);
        if maxd > tol || maxbv > bvtol {
            break; // stärker glätten hilft nicht – feiner werden
        }
        // Kantenmitten prüfen: erkennt Überschwinger zwischen den Stützpunkten
        let maxm = mids.iter().map(|&p| s.dist(p).abs()).fold(0.0, f64::max);
        let maxb = bmids.iter().map(|&p| s.dist(p).abs()).fold(0.0, f64::max);
        if maxm <= mtol && maxb <= btol {
            return Some(s);
        }
        if lam_mul >= 1e3 {
            break;
        }
        lam_mul *= 30.0;
        }
        if ns >= 60 {
            if std::env::var("M2S_DEBUG").is_ok() {
                eprintln!("  bspline: {} Punkte, ns {ns}, max. Abweichung {maxd:.4} > {tol:.4}", pts.len());
            }
            return None;
        }
        ns = (ns * 3 / 2 + 1).min(60);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::v3;

    #[test]
    fn basis_partition_of_unity() {
        let k = knots(0.0, 2.0, 5);
        for i in 0..=40 {
            let t = i as f64 * 0.05;
            let (_, b, d) = basis(&k, t);
            assert!((b.iter().sum::<f64>() - 1.0).abs() < 1e-12);
            assert!(d.iter().sum::<f64>().abs() < 1e-9);
        }
    }

    #[test]
    fn fit_sattel() {
        let mut pts = vec![];
        for i in 0..30 {
            for j in 0..20 {
                let (x, y) = (i as f64, j as f64 * 0.8);
                pts.push(v3(x, y, 0.01 * (x - 15.0) * (y - 8.0) + 0.3 * (x / 5.0).sin()));
            }
        }
        let s = fit(&pts, &[], &[], V3::ZERO, V3::X, V3::Y, V3::Z, 0.002, 1.0, 1.0, &[], 1.0).expect("fit");
        let mx = pts.iter().map(|&p| s.dist(p).abs()).fold(0.0, f64::max);
        assert!(mx < 0.002, "{mx}");
        // Kontrollpunkte reproduzieren die Fläche als Standard-B-Spline (lineare Präzision)
        let cps = s.control_points();
        let (u, v) = (7.3, 4.1);
        let (iu, bu, _) = basis(&s.ku, u);
        let (iv, bv, _) = basis(&s.kv, v);
        let mut p = V3::ZERO;
        for a in 0..4 {
            for b in 0..4 {
                p += cps[(iu + a) * s.nv + iv + b] * (bu[a] * bv[b]);
            }
        }
        let (h, _, _) = s.eval(u, v);
        assert!(p.dist(v3(u, v, h)) < 1e-9, "{:?}", p);
    }
}
