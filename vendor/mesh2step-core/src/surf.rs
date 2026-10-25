//! Analytische Flächen und deren Anpassung (Fitting) an Punktwolken.

use crate::math::{angle, solve, v3, M3, V3};

#[derive(Clone, Copy, Debug)]
pub enum Surf {
    /// Ebene durch o mit Normale n
    Plane { o: V3, n: V3 },
    /// Zylinder: Achse durch p, Richtung a, Radius r
    Cyl { p: V3, a: V3, r: f64 },
    /// Kegel: Achse durch p, Richtung a (Öffnungsrichtung), Radius r bei p, halber Öffnungswinkel ang
    Cone { p: V3, a: V3, r: f64, ang: f64 },
    /// Kugel
    Sphere { c: V3, r: f64 },
    /// Torus: Mittelpunkt c, Achse a, großer Radius rmaj, kleiner Radius rmin
    Torus { c: V3, a: V3, rmaj: f64, rmin: f64 },
    /// Freiformfläche (B-Spline, siehe bspline.rs)
    BSpline { id: u32 },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Kind {
    Plane,
    Cyl,
    Cone,
    Sphere,
    Torus,
    BSpline,
}

impl Kind {
    pub fn name_de(self) -> &'static str {
        match self {
            Kind::Plane => "Ebenen",
            Kind::Cyl => "Zylinder",
            Kind::Cone => "Kegel",
            Kind::Sphere => "Kugeln",
            Kind::Torus => "Tori",
            Kind::BSpline => "Freiformflächen",
        }
    }
}

impl Surf {
    pub fn kind(&self) -> Kind {
        match self {
            Surf::Plane { .. } => Kind::Plane,
            Surf::Cyl { .. } => Kind::Cyl,
            Surf::Cone { .. } => Kind::Cone,
            Surf::Sphere { .. } => Kind::Sphere,
            Surf::Torus { .. } => Kind::Torus,
            Surf::BSpline { .. } => Kind::BSpline,
        }
    }

    /// Vorzeichenbehafteter Abstand (positiv in Richtung der natürlichen Normale).
    pub fn dist(&self, x: V3) -> f64 {
        match *self {
            Surf::Plane { o, n } => (x - o).dot(n),
            Surf::Cyl { p, a, r } => {
                let d = x - p;
                (d - a * d.dot(a)).norm() - r
            }
            Surf::Cone { p, a, r, ang } => {
                let d = x - p;
                let h = d.dot(a);
                let rho = (d - a * h).norm();
                (rho - r - h * ang.tan()) * ang.cos()
            }
            Surf::Sphere { c, r } => (x - c).norm() - r,
            Surf::Torus { c, a, rmaj, rmin } => {
                let d = x - c;
                let h = d.dot(a);
                let rho = (d - a * h).norm();
                ((rho - rmaj).powi(2) + h * h).sqrt() - rmin
            }
            Surf::BSpline { id } => crate::bspline::get(id).dist(x),
        }
    }

    /// Natürliche (nach außen zeigende) Flächennormale am Punkt x.
    pub fn normal(&self, x: V3) -> V3 {
        match *self {
            Surf::Plane { n, .. } => n,
            Surf::Cyl { p, a, .. } => {
                let d = x - p;
                let w = d - a * d.dot(a);
                if w.norm() < 1e-300 {
                    a.any_perp()
                } else {
                    w.normalized()
                }
            }
            Surf::Cone { p, a, ang, .. } => {
                let d = x - p;
                let w = d - a * d.dot(a);
                let u = if w.norm() < 1e-300 { a.any_perp() } else { w.normalized() };
                (u * ang.cos() - a * ang.sin()).normalized()
            }
            Surf::Sphere { c, .. } => (x - c).normalized(),
            Surf::Torus { c, a, rmaj, .. } => {
                let d = x - c;
                let w = d - a * d.dot(a);
                let u = if w.norm() < 1e-300 { a.any_perp() } else { w.normalized() };
                (x - (c + u * rmaj)).normalized()
            }
            Surf::BSpline { id } => crate::bspline::get(id).normal(x),
        }
    }

    /// Projektion auf die Fläche (Newton-Schritte, für alle Typen ausreichend genau).
    pub fn project(&self, x: V3) -> V3 {
        let mut p = x;
        for _ in 0..4 {
            let d = self.dist(p);
            p -= self.normal(p) * d;
            if d.abs() < 1e-12 {
                break;
            }
        }
        p
    }

    pub fn axis(&self) -> Option<(V3, V3)> {
        match *self {
            Surf::Plane { .. } => None,
            Surf::Cyl { p, a, .. } | Surf::Cone { p, a, .. } => Some((p, a)),
            Surf::Torus { c, a, .. } => Some((c, a)),
            Surf::Sphere { .. } | Surf::BSpline { .. } => None,
        }
    }
}

// ---------------------------------------------------------------------------
// Ebenen-Fit

pub fn fit_plane(pts: &[V3]) -> Option<Surf> {
    if pts.len() < 3 {
        return None;
    }
    let mut c = V3::ZERO;
    for p in pts {
        c += *p;
    }
    c = c / pts.len() as f64;
    let mut m = M3::zero();
    for p in pts {
        m.add_outer(*p - c, 1.0);
    }
    let (_, vecs) = m.eigen_sym();
    Some(Surf::Plane { o: c, n: vecs[0] })
}

/// Ebene mit vorgegebener Normale (nur Versatz anpassen).
pub fn fit_plane_dir(pts: &[V3], n: V3) -> Surf {
    let mut s = 0.0;
    let mut c = V3::ZERO;
    for p in pts {
        s += p.dot(n);
        c += *p;
    }
    let k = pts.len().max(1) as f64;
    let c = c / k;
    let off = s / k;
    Surf::Plane { o: c - n * (c.dot(n) - off), n }
}

// ---------------------------------------------------------------------------
// Hilfen

/// Kreis-Fit in 2D (Kåsa, danach Gauß-Newton). Liefert (cx, cy, r, rms).
pub fn fit_circle_2d(pts: &[(f64, f64)]) -> Option<(f64, f64, f64, f64)> {
    if pts.len() < 3 {
        return None;
    }
    let mut a = vec![vec![0.0; 3]; 3];
    let mut b = vec![0.0; 3];
    let (mut mx, mut my) = (0.0, 0.0);
    for &(x, y) in pts {
        mx += x;
        my += y;
    }
    mx /= pts.len() as f64;
    my /= pts.len() as f64;
    for &(x, y) in pts {
        let (x, y) = (x - mx, y - my);
        let row = [x, y, 1.0];
        let rhs = -(x * x + y * y);
        for i in 0..3 {
            for j in 0..3 {
                a[i][j] += row[i] * row[j];
            }
            b[i] += row[i] * rhs;
        }
    }
    let s = solve(a, b)?;
    let mut cx = -s[0] / 2.0;
    let mut cy = -s[1] / 2.0;
    let r2 = cx * cx + cy * cy - s[2];
    if r2 <= 0.0 {
        return None;
    }
    let mut r = r2.sqrt();
    // Geometrische Verfeinerung
    for _ in 0..20 {
        let mut jtj = vec![vec![0.0; 3]; 3];
        let mut jtr = vec![0.0; 3];
        for &(x, y) in pts {
            let (x, y) = (x - mx, y - my);
            let dx = x - cx;
            let dy = y - cy;
            let d = (dx * dx + dy * dy).sqrt().max(1e-300);
            let res = d - r;
            let j = [-dx / d, -dy / d, -1.0];
            for i in 0..3 {
                for k in 0..3 {
                    jtj[i][k] += j[i] * j[k];
                }
                jtr[i] += j[i] * res;
            }
        }
        let step = match solve(jtj, jtr.iter().map(|v| -v).collect()) {
            Some(s) => s,
            None => break,
        };
        cx += step[0];
        cy += step[1];
        r += step[2];
        if step.iter().map(|v| v.abs()).sum::<f64>() < 1e-14 * (1.0 + r) {
            break;
        }
    }
    let mut ss = 0.0;
    for &(x, y) in pts {
        let d = ((x - mx - cx).powi(2) + (y - my - cy).powi(2)).sqrt() - r;
        ss += d * d;
    }
    Some((cx + mx, cy + my, r.abs(), (ss / pts.len() as f64).sqrt()))
}

/// Levenberg-Marquardt mit numerischer Jacobi-Matrix.
fn lm(mut prm: Vec<f64>, pts: &[V3], f: &dyn Fn(&[f64], V3) -> f64, iters: usize) -> (Vec<f64>, f64) {
    let n = prm.len();
    let cost = |p: &[f64]| -> f64 { pts.iter().map(|&x| f(p, x).powi(2)).sum() };
    let mut c0 = cost(&prm);
    let mut lambda = 1e-3;
    for _ in 0..iters {
        let mut jtj = vec![vec![0.0; n]; n];
        let mut jtr = vec![0.0; n];
        let mut row = vec![0.0; n];
        for &x in pts {
            let r0 = f(&prm, x);
            for k in 0..n {
                let h = 1e-7 * (1.0 + prm[k].abs());
                let mut p2 = prm.clone();
                p2[k] += h;
                row[k] = (f(&p2, x) - r0) / h;
            }
            for i in 0..n {
                for j in 0..n {
                    jtj[i][j] += row[i] * row[j];
                }
                jtr[i] += row[i] * r0;
            }
        }
        let mut improved = false;
        for _ in 0..8 {
            let mut a = jtj.clone();
            for (i, ai) in a.iter_mut().enumerate() {
                ai[i] += lambda * (jtj[i][i] + 1e-12);
            }
            let step = match solve(a, jtr.iter().map(|v| -v).collect()) {
                Some(s) => s,
                None => {
                    lambda *= 10.0;
                    continue;
                }
            };
            let p2: Vec<f64> = prm.iter().zip(&step).map(|(a, b)| a + b).collect();
            let c2 = cost(&p2);
            if c2.is_finite() && c2 < c0 {
                let rel = (c0 - c2) / (c0 + 1e-300);
                prm = p2;
                c0 = c2;
                lambda = (lambda * 0.3).max(1e-9);
                improved = rel > 1e-10;
                break;
            }
            lambda *= 10.0;
        }
        if !improved {
            break;
        }
    }
    (prm, (c0 / pts.len().max(1) as f64).sqrt())
}

/// Normiert Punkte (Schwerpunkt, Größe) für numerisch stabile Fits.
struct Frame {
    c: V3,
    s: f64,
}
impl Frame {
    fn new(pts: &[V3]) -> Frame {
        let mut c = V3::ZERO;
        for p in pts {
            c += *p;
        }
        c = c / pts.len().max(1) as f64;
        let mut s: f64 = 0.0;
        for p in pts {
            s = s.max((*p - c).norm());
        }
        Frame { c, s: s.max(1e-12) }
    }
    fn to(&self, p: V3) -> V3 {
        (p - self.c) / self.s
    }
    fn from(&self, p: V3) -> V3 {
        p * self.s + self.c
    }
}

fn rms(s: &Surf, pts: &[V3]) -> f64 {
    (pts.iter().map(|&p| s.dist(p).powi(2)).sum::<f64>() / pts.len().max(1) as f64).sqrt()
}

/// Abstand-Funktionen in parametrischer Form (für LM).
fn cyl_from(prm: &[f64], p0: V3, a0: V3, e1: V3, e2: V3) -> (V3, V3, f64) {
    let a = (a0 + e1 * prm[0] + e2 * prm[1]).normalized();
    let p = p0 + e1 * prm[2] + e2 * prm[3];
    (p, a, prm[4])
}

// ---------------------------------------------------------------------------
// Zylinder

/// pts: Punkte, nrm: (Normale, Gewicht) der Facetten.
pub fn fit_cylinder(pts: &[V3], nrm: &[(V3, f64, V3)], init: Option<&Surf>) -> Option<Surf> {
    if pts.len() < 5 {
        return None;
    }
    let fr = Frame::new(pts);
    let q: Vec<V3> = pts.iter().map(|&p| fr.to(p)).collect();
    let mut cands: Vec<V3> = Vec::new();
    if let Some(Surf::Cyl { a, .. }) | Some(Surf::Cone { a, .. }) | Some(Surf::Torus { a, .. }) = init {
        cands.push(*a);
    }
    let mut m = M3::zero();
    for &(n, w, _) in nrm {
        m.add_outer(n, w);
    }
    let (vals, vecs) = m.eigen_sym();
    cands.push(vecs[0]);
    if vals[1] < 0.05 * vals[2].max(1e-300) {
        cands.push(vecs[1]);
        // Punkt-Hauptrichtung als weiterer Kandidat
        let mut pm = M3::zero();
        for p in &q {
            pm.add_outer(*p, 1.0);
        }
        let (_, pv) = pm.eigen_sym();
        cands.push(pv[2]);
    }
    let mut best: Option<(Surf, f64)> = None;
    for a0 in cands {
        let (e1, e2) = a0.basis();
        let p2: Vec<(f64, f64)> = q.iter().map(|p| (p.dot(e1), p.dot(e2))).collect();
        let (cx, cy, r, _) = match fit_circle_2d(&p2) {
            Some(c) => c,
            None => continue,
        };
        if r > 1e4 {
            continue;
        }
        let p0 = e1 * cx + e2 * cy;
        let f = |prm: &[f64], x: V3| {
            let (p, a, r) = cyl_from(prm, p0, a0, e1, e2);
            let d = x - p;
            (d - a * d.dot(a)).norm() - r
        };
        let (prm, res) = lm(vec![0.0, 0.0, 0.0, 0.0, r], &q, &f, 40);
        let (p, a, r) = cyl_from(&prm, p0, a0, e1, e2);
        if !(r > 0.0) || !r.is_finite() {
            continue;
        }
        let cand = Surf::Cyl { p: fr.from(p), a, r: r * fr.s };
        let score = res + normal_error(&cand, nrm);
        if best.as_ref().map_or(true, |b| score < b.1) {
            best = Some((cand, score));
        }
    }
    best.map(|b| canon(b.0))
}

// ---------------------------------------------------------------------------
// Kegel

pub fn fit_cone(pts: &[V3], nrm: &[(V3, f64, V3)], init: Option<&Surf>) -> Option<Surf> {
    if pts.len() < 6 {
        return None;
    }
    let fr = Frame::new(pts);
    let q: Vec<V3> = pts.iter().map(|&p| fr.to(p)).collect();
    let mut starts: Vec<(V3, V3, f64)> = Vec::new(); // (apex, a, ang) im normierten Raum

    if let Some(Surf::Cone { p, a, r, ang }) = init {
        let apex = *p - *a * (r / ang.tan());
        starts.push((fr.to(apex), *a, *ang));
    }
    // Achse aus zentrierter Normalenkovarianz
    let mut wsum = 0.0;
    let mut mean = V3::ZERO;
    for &(n, w, _) in nrm {
        mean += n * w;
        wsum += w;
    }
    if wsum <= 0.0 {
        return None;
    }
    mean = mean / wsum;
    let mut m = M3::zero();
    for &(n, w, _) in nrm {
        m.add_outer(n - mean, w);
    }
    let (_, vecs) = m.eigen_sym();
    let mut a = vecs[0];
    let c = mean.dot(a);
    let mut ang = (-c).clamp(-1.0, 1.0).asin();
    if ang < 0.0 {
        a = -a;
        ang = -ang;
    }
    // Spitze: Schnittpunkt aller Facettenebenen
    let mut mm = vec![vec![0.0; 3]; 3];
    let mut rhs = vec![0.0; 3];
    for &(n, w, cpt) in nrm {
        let cq = fr.to(cpt);
        let d = n.dot(cq);
        for i in 0..3 {
            for j in 0..3 {
                mm[i][j] += w * n.get(i) * n.get(j);
            }
            rhs[i] += w * n.get(i) * d;
        }
    }
    if ang > 0.004 && ang < 1.53 {
        if let Some(x) = solve(mm, rhs) {
            let apex = v3(x[0], x[1], x[2]);
            if apex.norm() < 1e3 {
                starts.push((apex, a, ang));
            }
        }
    }
    let mut best: Option<(Surf, f64)> = None;
    for (apex, a0, ang0) in starts {
        // Basispunkt auf der Achse auf Höhe des Schwerpunkts
        let h = (V3::ZERO - apex).dot(a0);
        let (a0, ang0, h) = if h < 0.0 { (-a0, ang0, -h) } else { (a0, ang0, h) };
        let p0 = apex + a0 * h;
        let r0 = h * ang0.tan();
        let (e1, e2) = a0.basis();
        let f = |prm: &[f64], x: V3| {
            let a = (a0 + e1 * prm[0] + e2 * prm[1]).normalized();
            let p = p0 + e1 * prm[2] + e2 * prm[3];
            let d = x - p;
            let hh = d.dot(a);
            let rho = (d - a * hh).norm();
            (rho - prm[4] - hh * prm[5].tan()) * prm[5].cos()
        };
        let (prm, res) = lm(vec![0.0, 0.0, 0.0, 0.0, r0, ang0], &q, &f, 50);
        let a = (a0 + e1 * prm[0] + e2 * prm[1]).normalized();
        let p = p0 + e1 * prm[2] + e2 * prm[3];
        let (r, ang) = (prm[4], prm[5]);
        if !r.is_finite() || !ang.is_finite() || ang.abs() < 0.002 || ang.abs() > 1.55 {
            continue;
        }
        let s = canon(Surf::Cone { p: fr.from(p), a, r: r * fr.s, ang });
        let score = res + normal_error(&s, nrm);
        if best.as_ref().map_or(true, |b| score < b.1) {
            best = Some((s, score));
        }
    }
    best.map(|b| b.0)
}

// ---------------------------------------------------------------------------
// Kugel

pub fn fit_sphere(pts: &[V3]) -> Option<Surf> {
    if pts.len() < 5 {
        return None;
    }
    let fr = Frame::new(pts);
    let q: Vec<V3> = pts.iter().map(|&p| fr.to(p)).collect();
    let mut a = vec![vec![0.0; 4]; 4];
    let mut b = vec![0.0; 4];
    for p in &q {
        let row = [p.x, p.y, p.z, 1.0];
        let rhs = -p.norm2();
        for i in 0..4 {
            for j in 0..4 {
                a[i][j] += row[i] * row[j];
            }
            b[i] += row[i] * rhs;
        }
    }
    let s = solve(a, b)?;
    let c = v3(-s[0] / 2.0, -s[1] / 2.0, -s[2] / 2.0);
    let r2 = c.norm2() - s[3];
    if r2 <= 0.0 {
        return None;
    }
    let f = |prm: &[f64], x: V3| (x - v3(prm[0], prm[1], prm[2])).norm() - prm[3];
    let (prm, _) = lm(vec![c.x, c.y, c.z, r2.sqrt()], &q, &f, 30);
    if !(prm[3] > 0.0) || prm[3] > 1e4 {
        return None;
    }
    Some(Surf::Sphere { c: fr.from(v3(prm[0], prm[1], prm[2])), r: prm[3] * fr.s })
}

// ---------------------------------------------------------------------------
// Torus

/// Kreis im Raum an Punkte anpassen: (Mittelpunkt, Normale, Radius, rms)
pub fn fit_circle_3d(pts: &[V3]) -> Option<(V3, V3, f64, f64)> {
    let pl = fit_plane(pts)?;
    let (o, n) = match pl {
        Surf::Plane { o, n } => (o, n),
        _ => unreachable!(),
    };
    let (e1, e2) = n.basis();
    let p2: Vec<(f64, f64)> = pts.iter().map(|p| ((*p - o).dot(e1), (*p - o).dot(e2))).collect();
    let (cx, cy, r, _) = fit_circle_2d(&p2)?;
    let c = o + e1 * cx + e2 * cy;
    let mut ss = 0.0;
    for p in pts {
        let d = *p - c;
        let h = d.dot(n);
        let rho = (d - n * h).norm();
        ss += (rho - r).powi(2) + h * h;
    }
    Some((c, n, r, (ss / pts.len() as f64).sqrt()))
}

/// samples: (Punkt, Normale) an Facettenschwerpunkten
pub fn fit_torus(pts: &[V3], samples: &[(V3, V3)], init: Option<&Surf>) -> Option<Surf> {
    if pts.len() < 8 || samples.len() < 6 {
        return None;
    }
    let fr = Frame::new(pts);
    let q: Vec<V3> = pts.iter().map(|&p| fr.to(p)).collect();
    let smp: Vec<(V3, V3)> = samples.iter().map(|&(p, n)| (fr.to(p), n)).collect();
    let mut starts: Vec<(V3, V3, f64, f64)> = Vec::new();
    if let Some(Surf::Torus { c, a, rmaj, rmin }) = init {
        starts.push((fr.to(*c), *a, rmaj / fr.s, rmin / fr.s));
    }
    // Suche über den kleinen Radius: Röhrenmittelpunkte liegen auf einem Kreis
    let eval = |rr: f64| -> Option<(V3, V3, f64, f64)> {
        let cs: Vec<V3> = smp.iter().map(|&(p, n)| p - n * rr).collect();
        fit_circle_3d(&cs)
    };
    let mut best_rr: Option<(f64, f64)> = None;
    for i in 0..60 {
        let t = i as f64 / 59.0;
        let mag = 10f64.powf(-3.0 + 3.3 * t);
        for rr in [mag, -mag] {
            if let Some((_, _, _, e)) = eval(rr) {
                if best_rr.map_or(true, |b| e < b.1) {
                    best_rr = Some((rr, e));
                }
            }
        }
    }
    if let Some((rr0, _)) = best_rr {
        // Goldener Schnitt zur Verfeinerung
        let (mut lo, mut hi) = (rr0 * 0.85, rr0 * 1.15);
        if lo > hi {
            std::mem::swap(&mut lo, &mut hi);
        }
        let gr = 0.618_033_988_75;
        for _ in 0..40 {
            let m1 = hi - gr * (hi - lo);
            let m2 = lo + gr * (hi - lo);
            let e1 = eval(m1).map_or(f64::MAX, |r| r.3);
            let e2 = eval(m2).map_or(f64::MAX, |r| r.3);
            if e1 < e2 {
                hi = m2;
            } else {
                lo = m1;
            }
        }
        let rr = 0.5 * (lo + hi);
        if let Some((c, n, rmaj, _)) = eval(rr) {
            starts.push((c, n, rmaj, rr.abs()));
        }
    }
    let mut best: Option<(Surf, f64)> = None;
    for (c0, a0, rmaj0, rmin0) in starts {
        let (e1, e2) = a0.basis();
        let f = |prm: &[f64], x: V3| {
            let a = (a0 + e1 * prm[0] + e2 * prm[1]).normalized();
            let c = c0 + e1 * prm[2] + e2 * prm[3] + a0 * prm[4];
            let d = x - c;
            let h = d.dot(a);
            let rho = (d - a * h).norm();
            ((rho - prm[5]).powi(2) + h * h).sqrt() - prm[6]
        };
        let (prm, res) = lm(vec![0.0, 0.0, 0.0, 0.0, 0.0, rmaj0, rmin0], &q, &f, 50);
        let a = (a0 + e1 * prm[0] + e2 * prm[1]).normalized();
        let c = c0 + e1 * prm[2] + e2 * prm[3] + a0 * prm[4];
        let (rmaj, rmin) = (prm[5], prm[6]);
        if !(rmin > 0.0) || !(rmaj > rmin * 1.001) || rmaj > 1e4 {
            continue;
        }
        let s = Surf::Torus { c: fr.from(c), a, rmaj: rmaj * fr.s, rmin: rmin * fr.s };
        if best.as_ref().map_or(true, |b| res < b.1) {
            best = Some((s, res));
        }
    }
    best.map(|b| canon(b.0))
}

/// Kanonische Darstellung (Achsenpunkt nahe Ursprung bzw. Lotfußpunkt).
pub fn canon(s: Surf) -> Surf {
    match s {
        Surf::Cyl { p, a, r } => {
            let p = p - a * p.dot(a);
            Surf::Cyl { p, a, r }
        }
        Surf::Cone { p, a, r, ang } => {
            if ang < 0.0 {
                Surf::Cone { p, a: -a, r, ang: -ang }
            } else {
                Surf::Cone { p, a, r, ang }
            }
        }
        other => other,
    }
}

pub fn rms_of(s: &Surf, pts: &[V3]) -> f64 {
    rms(s, pts)
}

/// Prüft, ob zwei Flächen (näherungsweise) identisch sind.
pub fn same_surface(a: &Surf, b: &Surf, tol: f64) -> bool {
    let ang_tol = 2e-3;
    match (*a, *b) {
        (Surf::Plane { o: o1, n: n1 }, Surf::Plane { o: o2, n: n2 }) => {
            angle(n1, n2) < ang_tol && (o2 - o1).dot(n1).abs() < tol
        }
        (Surf::Cyl { p: p1, a: a1, r: r1 }, Surf::Cyl { p: p2, a: a2, r: r2 }) => {
            let par = angle(a1, a2).min(angle(a1, -a2)) < ang_tol;
            let d = p2 - p1;
            par && (r1 - r2).abs() < tol && (d - a1 * d.dot(a1)).norm() < tol
        }
        (Surf::Sphere { c: c1, r: r1 }, Surf::Sphere { c: c2, r: r2 }) => c1.dist(c2) < tol && (r1 - r2).abs() < tol,
        (Surf::Cone { p: p1, a: a1, r: r1, ang: g1 }, Surf::Cone { a: a2, ang: g2, .. }) => {
            if angle(a1, a2) > ang_tol || (g1 - g2).abs() > ang_tol {
                return false;
            }
            // Spitze vergleichen
            let s2 = *b;
            s2.dist(p1 + a1.any_perp() * r1).abs() < tol && s2.dist(p1 + a1 * 1.0 + a1.any_perp() * (r1 + g1.tan())).abs() < tol
        }
        (Surf::Torus { c: c1, a: a1, rmaj: r1, rmin: m1 }, Surf::Torus { c: c2, a: a2, rmaj: r2, rmin: m2 }) => {
            angle(a1, a2).min(angle(a1, -a2)) < ang_tol && c1.dist(c2) < tol && (r1 - r2).abs() < tol && (m1 - m2).abs() < tol
        }
        _ => false,
    }
}

/// Mittlere Abweichung zwischen Facetten- und Flächennormalen (0 = perfekt).
fn normal_error(s: &Surf, nrm: &[(V3, f64, V3)]) -> f64 {
    let mut e = 0.0;
    let mut w = 0.0;
    for &(n, wt, c) in nrm {
        e += wt * (1.0 - n.dot(s.normal(c)).abs());
        w += wt;
    }
    if w > 0.0 {
        e / w
    } else {
        0.0
    }
}
