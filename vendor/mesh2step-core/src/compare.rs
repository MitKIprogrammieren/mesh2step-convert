//! Maßvergleich zwischen einem Dreiecksnetz (STL/3MF) und einem CAD-Modell (STEP).
//!
//! Für jedes Dreieck werden Ecken, Kantenmitten und Schwerpunkt auf die nächste Fläche
//! des Gegenstücks projiziert (vorzeichenbehaftet: + = Punkt liegt außerhalb, also Material
//! steht über). Zylinder werden aus den CAD-Flächen übernommen und am Netz per
//! Kreisausgleich nachgemessen.

use crate::math::{v3, V3};
use crate::stepread::{CadModel, CadSurf};
use std::collections::HashMap;

/// Einfaches indiziertes Dreiecksnetz.
#[derive(Clone, Debug, Default)]
pub struct TriMesh {
    pub verts: Vec<V3>,
    pub tris: Vec<[u32; 3]>,
}

impl TriMesh {
    /// Netze zusammenführen und gleiche Ecken verschweißen.
    pub fn from_raws(raws: &[crate::io::RawMesh]) -> TriMesh {
        let mut map: HashMap<(i64, i64, i64), u32> = HashMap::new();
        let mut m = TriMesh::default();
        let q = 1e5;
        for r in raws {
            let mut remap = Vec::with_capacity(r.verts.len());
            for v in &r.verts {
                let k = ((v.x * q).round() as i64, (v.y * q).round() as i64, (v.z * q).round() as i64);
                let id = *map.entry(k).or_insert_with(|| {
                    m.verts.push(*v);
                    (m.verts.len() - 1) as u32
                });
                remap.push(id);
            }
            for t in &r.tris {
                let (a, b, c) = (remap[t[0] as usize], remap[t[1] as usize], remap[t[2] as usize]);
                if a != b && b != c && a != c {
                    m.tris.push([a, b, c]);
                }
            }
        }
        m
    }

    pub fn from_cad(c: &CadModel) -> TriMesh {
        TriMesh { verts: c.verts.clone(), tris: c.tris.clone() }
    }

    fn tri(&self, t: usize) -> (V3, V3, V3) {
        let i = self.tris[t];
        (self.verts[i[0] as usize], self.verts[i[1] as usize], self.verts[i[2] as usize])
    }

    /// Volumen (bezogen auf die Mitte des Umrisses, damit offene Netze fern vom
    /// Nullpunkt keine großen Rechenfehler erzeugen).
    pub fn signed_volume(&self) -> f64 {
        if self.verts.is_empty() {
            return 0.0;
        }
        let (mut lo, mut hi) = (self.verts[0], self.verts[0]);
        for v in &self.verts {
            lo = lo.min(*v);
            hi = hi.max(*v);
        }
        let c = (lo + hi) * 0.5;
        let mut v = 0.0;
        for t in 0..self.tris.len() {
            let (a, b, d) = self.tri(t);
            v += (a - c).dot((b - c).cross(d - c));
        }
        v / 6.0
    }

    /// Dreiecke so drehen, dass das Volumen positiv ist (Normalen nach außen).
    pub fn orient_outward(&mut self) -> bool {
        if self.signed_volume() < 0.0 {
            for t in self.tris.iter_mut() {
                t.swap(1, 2);
            }
            return true;
        }
        false
    }

    pub fn transformed(&self, m: &Xf) -> TriMesh {
        TriMesh { verts: self.verts.iter().map(|&p| apply(m, p)).collect(), tris: self.tris.clone() }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Stats {
    pub min: V3,
    pub max: V3,
    pub size: V3,
    pub center: V3,
    pub area: f64,
    pub volume: f64,
    pub triangles: usize,
    pub open_edges: usize,
}

pub fn stats(m: &TriMesh) -> Stats {
    let mut mn = v3(f64::MAX, f64::MAX, f64::MAX);
    let mut mx = v3(f64::MIN, f64::MIN, f64::MIN);
    let mut used = vec![false; m.verts.len()];
    for t in &m.tris {
        for &i in t {
            used[i as usize] = true;
        }
    }
    for (i, v) in m.verts.iter().enumerate() {
        if used[i] {
            mn = mn.min(*v);
            mx = mx.max(*v);
        }
    }
    if m.tris.is_empty() {
        mn = V3::ZERO;
        mx = V3::ZERO;
    }
    let mut area = 0.0;
    let mut edges: HashMap<(u32, u32), u32> = HashMap::with_capacity(m.tris.len() * 3);
    for (ti, t) in m.tris.iter().enumerate() {
        let (a, b, c) = m.tri(ti);
        area += 0.5 * (b - a).cross(c - a).norm();
        for k in 0..3 {
            let (p, q) = (t[k], t[(k + 1) % 3]);
            *edges.entry(if p < q { (p, q) } else { (q, p) }).or_insert(0) += 1;
        }
    }
    let open = edges.values().filter(|&&c| c == 1).count();
    Stats { min: mn, max: mx, size: mx - mn, center: (mn + mx) * 0.5, area, volume: m.signed_volume().abs(), triangles: m.tris.len(), open_edges: open }
}

// ---------------------------------------------------------------------------
// Räumliches Gitter für die Nächster-Punkt-Suche

pub struct Grid<'a> {
    mesh: &'a TriMesh,
    min: V3,
    dims: [usize; 3],
    cs: [f64; 3],
    start: Vec<u32>,
    items: Vec<u32>,
    /// Eckennormalen
    nrm: Vec<V3>,
}

#[derive(Clone, Copy, Debug)]
pub struct Hit {
    /// Abstand
    pub d: f64,
    /// vorzeichenbehafteter Abstand (+ = außerhalb des Ziels)
    pub s: f64,
    /// nächster Punkt
    pub q: V3,
    pub t: u32,
}

fn closest_on_tri(p: V3, a: V3, b: V3, c: V3) -> V3 {
    let ab = b - a;
    let ac = c - a;
    let ap = p - a;
    let d1 = ab.dot(ap);
    let d2 = ac.dot(ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return a;
    }
    let bp = p - b;
    let d3 = ab.dot(bp);
    let d4 = ac.dot(bp);
    if d3 >= 0.0 && d4 <= d3 {
        return b;
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return a + ab * v;
    }
    let cp = p - c;
    let d5 = ab.dot(cp);
    let d6 = ac.dot(cp);
    if d6 >= 0.0 && d5 <= d6 {
        return c;
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return a + ac * w;
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return b + (c - b) * w;
    }
    let den = 1.0 / (va + vb + vc);
    a + ab * (vb * den) + ac * (vc * den)
}

impl<'a> Grid<'a> {
    pub fn new(mesh: &'a TriMesh) -> Grid<'a> {
        let st = stats(mesh);
        let nt = mesh.tris.len().max(1);
        let maxext = st.size.x.max(st.size.y).max(st.size.z).max(1e-9);
        let pad = maxext * 1e-6 + 1e-9;
        let min = st.min - v3(pad, pad, pad);
        let ext = [st.size.x + 2.0 * pad, st.size.y + 2.0 * pad, st.size.z + 2.0 * pad];
        let vol = (ext[0] * ext[1] * ext[2]).max(1e-18);
        let cell = (vol / (nt as f64 * 1.5)).cbrt().max(maxext / 400.0);
        let dims = [0, 1, 2].map(|k| ((ext[k] / cell).ceil() as usize).clamp(1, 400));
        let cs = [0, 1, 2].map(|k| ext[k] / dims[k] as f64);
        let ncell = dims[0] * dims[1] * dims[2];
        let cellr = |p: V3| -> [usize; 3] { [0, 1, 2].map(|k| ((((p.get(k) - min.get(k)) / cs[k]).floor()).max(0.0) as usize).min(dims[k] - 1)) };
        let mut ranges = Vec::with_capacity(mesh.tris.len());
        let mut counts = vec![0u32; ncell + 1];
        for t in 0..mesh.tris.len() {
            let (a, b, c) = mesh.tri(t);
            let lo = cellr(a.min(b).min(c));
            let hi = cellr(a.max(b).max(c));
            for z in lo[2]..=hi[2] {
                for y in lo[1]..=hi[1] {
                    for x in lo[0]..=hi[0] {
                        counts[(z * dims[1] + y) * dims[0] + x + 1] += 1;
                    }
                }
            }
            ranges.push((lo, hi));
        }
        for i in 1..=ncell {
            counts[i] += counts[i - 1];
        }
        let mut items = vec![0u32; counts[ncell] as usize];
        let mut fill = counts[..ncell].to_vec();
        for (t, (lo, hi)) in ranges.iter().enumerate() {
            for z in lo[2]..=hi[2] {
                for y in lo[1]..=hi[1] {
                    for x in lo[0]..=hi[0] {
                        let c = (z * dims[1] + y) * dims[0] + x;
                        items[fill[c] as usize] = t as u32;
                        fill[c] += 1;
                    }
                }
            }
        }
        // flächengewichtete Eckennormalen: für das Vorzeichen am nächsten Punkt interpoliert
        // (schmale Dreiecke haben kaum Gewicht, an Kanten/Ecken stimmt die Seite)
        let mut vn = vec![V3::ZERO; mesh.verts.len()];
        for t in 0..mesh.tris.len() {
            let (a, b, c) = mesh.tri(t);
            let n = (b - a).cross(c - a);
            for &i in &mesh.tris[t] {
                vn[i as usize] += n;
            }
        }
        let nrm = vn.into_iter().map(|n| n.normalized()).collect();
        Grid { mesh, min, dims, cs, start: counts, items, nrm }
    }

    pub fn query(&self, p: V3, stamp: &mut Stamp) -> Hit {
        let dims = self.dims;
        stamp.next(self.mesh.tris.len());
        let c = [0, 1, 2].map(|k| ((((p.get(k) - self.min.get(k)) / self.cs[k]).floor()).max(0.0) as usize).min(dims[k] - 1));
        let hi_b = [0, 1, 2].map(|k| self.min.get(k) + dims[k] as f64 * self.cs[k]);
        let o = [0, 1, 2].map(|k| (self.min.get(k) - p.get(k)).max(p.get(k) - hi_b[k]).max(0.0));
        let outside = (o[0] * o[0] + o[1] * o[1] + o[2] * o[2]).sqrt();
        let min_cs = self.cs[0].min(self.cs[1]).min(self.cs[2]);
        let mut best = f64::MAX;
        let mut bt = u32::MAX;
        let mut bq = V3::ZERO;
        let maxr = dims[0].max(dims[1]).max(dims[2]) as i64;
        for r in 0..=maxr {
            let (cx, cy, cz) = (c[0] as i64, c[1] as i64, c[2] as i64);
            for z in (cz - r).max(0)..=(cz + r).min(dims[2] as i64 - 1) {
                for y in (cy - r).max(0)..=(cy + r).min(dims[1] as i64 - 1) {
                    let shell = z == cz - r || z == cz + r || y == cy - r || y == cy + r;
                    let xs: Vec<i64> = if shell { ((cx - r).max(0)..=(cx + r).min(dims[0] as i64 - 1)).collect() } else { [cx - r, cx + r].into_iter().filter(|&x| x >= 0 && x < dims[0] as i64).collect() };
                    for x in xs {
                        let cell = (z as usize * dims[1] + y as usize) * dims[0] + x as usize;
                        for i in self.start[cell]..self.start[cell + 1] {
                            let t = self.items[i as usize];
                            if !stamp.visit(t) {
                                continue;
                            }
                            let (a, b, cc) = self.mesh.tri(t as usize);
                            let q = closest_on_tri(p, a, b, cc);
                            let d = (p - q).norm2();
                            if d < best {
                                best = d;
                                bt = t;
                                bq = q;
                            }
                        }
                    }
                }
            }
            if bt != u32::MAX && best.sqrt() <= (r as f64 * min_cs).max(outside) {
                break;
            }
        }
        if bt == u32::MAX {
            return Hit { d: f64::MAX, s: f64::MAX, q: p, t: 0 };
        }
        let d = best.sqrt();
        // Normale am nächsten Punkt aus den Eckennormalen (baryzentrisch)
        let tr = self.mesh.tris[bt as usize];
        let (a, b, c) = self.mesh.tri(bt as usize);
        let (v0, v1, v2) = (b - a, c - a, bq - a);
        let (d00, d01, d11, d20, d21) = (v0.dot(v0), v0.dot(v1), v1.dot(v1), v2.dot(v0), v2.dot(v1));
        let den = d00 * d11 - d01 * d01;
        let n = if den.abs() > 1e-300 {
            let w1 = ((d11 * d20 - d01 * d21) / den).clamp(0.0, 1.0);
            let w2 = ((d00 * d21 - d01 * d20) / den).clamp(0.0, 1.0);
            let w0 = (1.0 - w1 - w2).max(0.0);
            self.nrm[tr[0] as usize] * w0 + self.nrm[tr[1] as usize] * w1 + self.nrm[tr[2] as usize] * w2
        } else {
            self.nrm[tr[0] as usize] + self.nrm[tr[1] as usize] + self.nrm[tr[2] as usize]
        };
        let n = if n.norm2() > 1e-24 { n } else { v0.cross(v1) };
        let dot = (p - bq).dot(n);
        Hit { d, s: if dot >= 0.0 { d } else { -d }, q: bq, t: bt }
    }
}

/// Markierung bereits geprüfter Dreiecke (je Suchthread eine).
pub struct Stamp {
    mark: Vec<u32>,
    tick: u32,
}

impl Stamp {
    pub fn new() -> Stamp {
        Stamp { mark: Vec::new(), tick: 0 }
    }
    fn next(&mut self, n: usize) {
        if self.mark.len() != n {
            self.mark = vec![0; n];
            self.tick = 0;
        }
        self.tick = self.tick.wrapping_add(1);
        if self.tick == 0 {
            self.mark.iter_mut().for_each(|x| *x = 0);
            self.tick = 1;
        }
    }
    fn visit(&mut self, t: u32) -> bool {
        let m = &mut self.mark[t as usize];
        if *m == self.tick {
            return false;
        }
        *m = self.tick;
        true
    }
}

impl Default for Stamp {
    fn default() -> Self {
        Stamp::new()
    }
}

// ---------------------------------------------------------------------------
// Abweichung je Dreieck

#[derive(Clone, Debug, Default)]
pub struct Deviation {
    /// Wert mit größtem Betrag je Dreieck
    pub face_dev: Vec<f32>,
    /// je Dreieck: Ecken a, b, c und Kantenmitten ab, bc, ca
    pub sample_dev: Vec<[f32; 6]>,
    /// flächengewichteter Mittelwert je Dreieck
    pub face_mean: Vec<f32>,
    pub face_area: Vec<f64>,
    pub mean_abs: f64,
    pub rms: f64,
    pub max_pos: f64,
    pub max_neg: f64,
    pub max_abs: f64,
    pub max_at: V3,
    pub total_area: f64,
}

impl Deviation {
    /// Vorzeichen umdrehen (Gegenrichtung in die Sicht des anderen Modells bringen).
    pub fn negate(&mut self) {
        for v in self.face_dev.iter_mut() {
            *v = -*v;
        }
        for v in self.face_mean.iter_mut() {
            *v = -*v;
        }
        for s in self.sample_dev.iter_mut() {
            for v in s.iter_mut() {
                *v = -*v;
            }
        }
        let (p, n) = (self.max_pos, self.max_neg);
        self.max_pos = -n;
        self.max_neg = -p;
    }

    /// Anteil der Fläche, deren Dreiecke vollständig innerhalb ±tol liegen.
    pub fn within(&self, tol: f64) -> f64 {
        let mut ok = 0.0;
        for (i, &d) in self.face_dev.iter().enumerate() {
            if (d as f64).abs() <= tol {
                ok += self.face_area[i];
            }
        }
        if self.total_area > 0.0 {
            ok / self.total_area
        } else {
            0.0
        }
    }

    /// Flächenanteile je Klasse zwischen lo und hi.
    pub fn histogram(&self, lo: f64, hi: f64, bins: usize) -> Vec<f64> {
        let mut h = vec![0.0; bins];
        for (i, &v) in self.face_mean.iter().enumerate() {
            let b = (((v as f64 - lo) / (hi - lo) * bins as f64).floor().max(0.0) as usize).min(bins - 1);
            h[b] += self.face_area[i];
        }
        let s: f64 = h.iter().sum();
        if s > 0.0 {
            h.iter_mut().for_each(|x| *x /= s);
        }
        h
    }
}

/// Misst `src` gegen das Ziel im Gitter. `progress(0..1)`; liefert None bei Abbruch.
pub fn deviation(src: &TriMesh, grid: &Grid, progress: &dyn Fn(f32) -> bool) -> Option<Deviation> {
    let nt = src.tris.len();
    let nv = src.verts.len();
    let mut st = Stamp::new();
    let mut vd = vec![f64::NAN; nv];
    let mut d = Deviation { face_dev: vec![0.0; nt], sample_dev: vec![[0.0; 6]; nt], face_mean: vec![0.0; nt], face_area: vec![0.0; nt], max_pos: f64::MIN, max_neg: f64::MAX, ..Default::default() };
    let (mut w_abs, mut w_sq, mut w_sum) = (0.0, 0.0, 0.0);
    // Gewichte: Ecken je 1/12, Kantenmitten je 3/16, Schwerpunkt 3/16 (Summe 1)
    const W: [f64; 7] = [1.0 / 12.0, 1.0 / 12.0, 1.0 / 12.0, 3.0 / 16.0, 3.0 / 16.0, 3.0 / 16.0, 3.0 / 16.0];
    for t in 0..nt {
        if t % 4096 == 0 && !progress(t as f32 / nt.max(1) as f32) {
            return None;
        }
        let idx = src.tris[t];
        let (a, b, c) = src.tri(t);
        let area = 0.5 * (b - a).cross(c - a).norm();
        let mut vals = [0.0f64; 7];
        let mut pts = [V3::ZERO; 7];
        for k in 0..3 {
            let vi = idx[k] as usize;
            if vd[vi].is_nan() {
                vd[vi] = grid.query(src.verts[vi], &mut st).s;
            }
            vals[k] = vd[vi];
            pts[k] = src.verts[vi];
        }
        let mids = [(a + b) * 0.5, (b + c) * 0.5, (c + a) * 0.5, (a + b + c) / 3.0];
        for (k, m) in mids.iter().enumerate() {
            vals[3 + k] = grid.query(*m, &mut st).s;
            pts[3 + k] = *m;
        }
        let mut ext = 0.0f64;
        let mut mean = 0.0;
        for k in 0..7 {
            let v = vals[k];
            if v.abs() > ext.abs() {
                ext = v;
            }
            mean += v * W[k];
            let w = area * W[k];
            w_sum += w;
            w_abs += v.abs() * w;
            w_sq += v * v * w;
            d.max_pos = d.max_pos.max(v);
            d.max_neg = d.max_neg.min(v);
            if v.abs() > d.max_abs {
                d.max_abs = v.abs();
                d.max_at = pts[k];
            }
        }
        d.face_dev[t] = ext as f32;
        d.face_mean[t] = mean as f32;
        d.face_area[t] = area;
        for k in 0..6 {
            d.sample_dev[t][k] = vals[k] as f32;
        }
    }
    progress(1.0);
    d.total_area = w_sum;
    if w_sum > 0.0 {
        d.mean_abs = w_abs / w_sum;
        d.rms = (w_sq / w_sum).sqrt();
    }
    if nt == 0 {
        d.max_pos = 0.0;
        d.max_neg = 0.0;
    }
    Some(d)
}

// ---------------------------------------------------------------------------
// Ausrichtung

/// Starre Transformation als 3×4-Zeilenmatrix.
pub type Xf = [f64; 12];
pub const IDENT: Xf = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0];

pub fn apply(m: &Xf, p: V3) -> V3 {
    v3(m[0] * p.x + m[1] * p.y + m[2] * p.z + m[3], m[4] * p.x + m[5] * p.y + m[6] * p.z + m[7], m[8] * p.x + m[9] * p.y + m[10] * p.z + m[11])
}

/// A nach B: erst B, dann A
pub fn compose(a: &Xf, b: &Xf) -> Xf {
    let mut r = [0.0; 12];
    for i in 0..3 {
        for j in 0..3 {
            r[i * 4 + j] = a[i * 4] * b[j] + a[i * 4 + 1] * b[4 + j] + a[i * 4 + 2] * b[8 + j];
        }
        r[i * 4 + 3] = a[i * 4] * b[3] + a[i * 4 + 1] * b[7] + a[i * 4 + 2] * b[11] + a[i * 4 + 3];
    }
    r
}

/// Verschiebung (mm) und Drehwinkel (Grad) einer Transformation.
pub fn xf_amount(m: &Xf) -> (f64, f64) {
    let tr = ((m[0] + m[5] + m[10] - 1.0) / 2.0).clamp(-1.0, 1.0);
    ((m[3] * m[3] + m[7] * m[7] + m[11] * m[11]).sqrt(), tr.acos().to_degrees())
}

fn jacobi4(a0: [[f64; 4]; 4]) -> ([f64; 4], [[f64; 4]; 4]) {
    let mut a = a0;
    let mut v = [[0.0; 4]; 4];
    for (i, row) in v.iter_mut().enumerate() {
        row[i] = 1.0;
    }
    for _ in 0..60 {
        let mut off = 0.0;
        for p in 0..4 {
            for q in p + 1..4 {
                off += a[p][q] * a[p][q];
            }
        }
        if off < 1e-24 {
            break;
        }
        for p in 0..4 {
            for q in p + 1..4 {
                if a[p][q].abs() < 1e-30 {
                    continue;
                }
                let th = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
                let t = if th >= 0.0 { 1.0 } else { -1.0 } / (th.abs() + (th * th + 1.0).sqrt());
                let c = 1.0 / (t * t + 1.0).sqrt();
                let s = t * c;
                for k in 0..4 {
                    let (x, y) = (a[k][p], a[k][q]);
                    a[k][p] = c * x - s * y;
                    a[k][q] = s * x + c * y;
                }
                for k in 0..4 {
                    let (x, y) = (a[p][k], a[q][k]);
                    a[p][k] = c * x - s * y;
                    a[q][k] = s * x + c * y;
                }
                for k in 0..4 {
                    let (x, y) = (v[k][p], v[k][q]);
                    v[k][p] = c * x - s * y;
                    v[k][q] = s * x + c * y;
                }
            }
        }
    }
    ([a[0][0], a[1][1], a[2][2], a[3][3]], v)
}

/// Optimale Drehung + Verschiebung von P auf Q (Horn, Quaternionen).
fn rigid_fit(p: &[V3], q: &[V3]) -> Xf {
    let n = p.len().max(1) as f64;
    let cp = p.iter().fold(V3::ZERO, |a, &b| a + b) / n;
    let cq = q.iter().fold(V3::ZERO, |a, &b| a + b) / n;
    let mut s = [[0.0; 3]; 3];
    for (a, b) in p.iter().zip(q.iter()) {
        let (x, y) = (*a - cp, *b - cq);
        let xa = [x.x, x.y, x.z];
        let ya = [y.x, y.y, y.z];
        for i in 0..3 {
            for j in 0..3 {
                s[i][j] += xa[i] * ya[j];
            }
        }
    }
    let [[sxx, sxy, sxz], [syx, syy, syz], [szx, szy, szz]] = s;
    let nm = [
        [sxx + syy + szz, syz - szy, szx - sxz, sxy - syx],
        [syz - szy, sxx - syy - szz, sxy + syx, szx + sxz],
        [szx - sxz, sxy + syx, -sxx + syy - szz, syz + szy],
        [sxy - syx, szx + sxz, syz + szy, -sxx - syy + szz],
    ];
    let (vals, vecs) = jacobi4(nm);
    let mut bi = 0;
    for i in 1..4 {
        if vals[i] > vals[bi] {
            bi = i;
        }
    }
    let (w, x, y, z) = (vecs[0][bi], vecs[1][bi], vecs[2][bi], vecs[3][bi]);
    let r = [
        w * w + x * x - y * y - z * z,
        2.0 * (x * y - w * z),
        2.0 * (x * z + w * y),
        2.0 * (x * y + w * z),
        w * w - x * x + y * y - z * z,
        2.0 * (y * z - w * x),
        2.0 * (x * z - w * y),
        2.0 * (y * z + w * x),
        w * w - x * x - y * y + z * z,
    ];
    let t = cq - v3(r[0] * cp.x + r[1] * cp.y + r[2] * cp.z, r[3] * cp.x + r[4] * cp.y + r[5] * cp.z, r[6] * cp.x + r[7] * cp.y + r[8] * cp.z);
    [r[0], r[1], r[2], t.x, r[3], r[4], r[5], t.y, r[6], r[7], r[8], t.z]
}

/// Flächengleichverteilte Stichprobe von Punkten.
pub fn sample_points(m: &TriMesh, n: usize) -> Vec<V3> {
    let nt = m.tris.len();
    if nt == 0 {
        return Vec::new();
    }
    let mut cum = Vec::with_capacity(nt);
    let mut s = 0.0;
    for t in 0..nt {
        let (a, b, c) = m.tri(t);
        s += 0.5 * (b - a).cross(c - a).norm();
        cum.push(s);
    }
    let mut seed: u32 = 12345;
    let mut rnd = move || {
        seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
        seed as f64 / 4294967296.0
    };
    (0..n)
        .map(|_| {
            let r = rnd() * s;
            let t = cum.partition_point(|&x| x < r).min(nt - 1);
            let (mut u, mut v) = (rnd(), rnd());
            if u + v > 1.0 {
                u = 1.0 - u;
                v = 1.0 - v;
            }
            let (a, b, c) = m.tri(t);
            a + (b - a) * u + (c - a) * v
        })
        .collect()
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Align {
    /// so wie in den Dateien
    None,
    /// Mittelpunkte der Umrisse übereinander
    Center,
    /// Verschieben und Drehen (Best-Fit, ICP)
    BestFit,
}

/// Transformation, die `mov` auf das Ziel legt.
pub fn align(mov: &TriMesh, target: &TriMesh, grid: &Grid, mode: Align, progress: &dyn Fn(f32) -> bool) -> Option<Xf> {
    if mode == Align::None {
        return Some(IDENT);
    }
    let (sm, stt) = (stats(mov), stats(target));
    let d = stt.center - sm.center;
    let mut m: Xf = [1.0, 0.0, 0.0, d.x, 0.0, 1.0, 0.0, d.y, 0.0, 0.0, 1.0, d.z];
    if mode == Align::Center {
        return Some(m);
    }
    let base = sample_points(mov, 6000);
    let mut st = Stamp::new();
    let mut last = f64::MAX;
    let iters = 50;
    for it in 0..iters {
        if !progress(it as f32 / iters as f32) {
            return None;
        }
        let p: Vec<V3> = base.iter().map(|&x| apply(&m, x)).collect();
        let hits: Vec<Hit> = p.iter().map(|&x| grid.query(x, &mut st)).collect();
        // grobe Ausreißer (z. B. fehlende Bereiche) nach den ersten Runden ausblenden
        let mut ds: Vec<f64> = hits.iter().map(|h| h.d).collect();
        ds.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let cut = if it < 5 { f64::MAX } else { (ds[ds.len() * 9 / 10] * 3.0).max(1e-6) };
        let (mut pp, mut qq) = (Vec::with_capacity(p.len()), Vec::with_capacity(p.len()));
        let mut err = 0.0;
        for (x, h) in p.iter().zip(hits.iter()) {
            if h.d <= cut {
                pp.push(*x);
                qq.push(h.q);
                err += h.d * h.d;
            }
        }
        if pp.len() < 10 {
            break;
        }
        err = (err / pp.len() as f64).sqrt();
        let dm = rigid_fit(&pp, &qq);
        m = compose(&dm, &m);
        if (last - err).abs() < 1e-8 {
            break;
        }
        last = err;
    }
    progress(1.0);
    Some(m)
}

// ---------------------------------------------------------------------------
// Bohrungen und Zylinder

#[derive(Clone, Debug)]
pub struct Cylinder {
    /// Punkt auf der Achse (bei h = 0), Achsrichtung und Bezugsrichtungen
    pub o: V3,
    pub axis: V3,
    pub u: V3,
    pub w: V3,
    pub r: f64,
    /// Lage entlang der Achse
    pub h0: f64,
    pub h1: f64,
    /// Bohrung (Außennormale zeigt zur Achse) oder Zapfen
    pub hole: bool,
    /// Winkelabdeckung in Grad
    pub coverage: f64,
    /// Anzahl zusammengefasster CAD-Flächen
    pub faces: usize,
    /// Dreiecke des CAD-Netzes
    pub cad_tris: Vec<u32>,
}

impl Cylinder {
    pub fn diameter(&self) -> f64 {
        2.0 * self.r
    }
    pub fn length(&self) -> f64 {
        self.h1 - self.h0
    }
    pub fn axis_point(&self, h: f64) -> V3 {
        self.o + self.axis * h
    }
}

fn coverage_deg(angles: &[f64]) -> f64 {
    let mut bins = [false; 72];
    for &a in angles {
        let i = (((a + std::f64::consts::PI) / std::f64::consts::TAU * 72.0).floor() as i64).clamp(0, 71) as usize;
        bins[i] = true;
    }
    bins.iter().filter(|b| **b).count() as f64 * 5.0
}

/// Zylinderflächen des CAD-Modells, gleiche Zylinder (geteilte Flächen) zusammengefasst.
pub fn cylinders(cad: &CadModel) -> Vec<Cylinder> {
    let mut out: Vec<Cylinder> = Vec::new();
    for f in &cad.faces {
        let CadSurf::Cyl(fr, r) = &f.surf else { continue };
        let (o, axis, u) = (fr.o, fr.z, fr.x);
        let w = axis.cross(u);
        let mut h0 = f64::MAX;
        let mut h1 = f64::MIN;
        let mut ang = Vec::new();
        let tris: Vec<u32> = (f.first..=f.last).map(|t| t as u32).collect();
        for &t in &tris {
            for &vi in &cad.tris[t as usize] {
                let p = cad.verts[vi as usize] - o;
                let h = p.dot(axis);
                h0 = h0.min(h);
                h1 = h1.max(h);
                ang.push(p.dot(w).atan2(p.dot(u)));
            }
        }
        if h0 > h1 {
            continue;
        }
        let hole = !f.same_sense;
        // gleiche Achse, gleicher Radius, gleiche Art: zusammenfassen
        let same = out.iter_mut().find(|c| {
            let par = c.axis.dot(axis).abs() > 1.0 - 1e-6;
            let d = o - c.o;
            let off = (d - c.axis * d.dot(c.axis)).norm();
            par && c.hole == hole && (c.r - r).abs() <= (1e-4 * r).max(1e-4) && off <= (1e-4 * r).max(1e-4)
        });
        match same {
            Some(c) => {
                // Lage im Bezugssystem des vorhandenen Zylinders
                for &t in &tris {
                    for &vi in &cad.tris[t as usize] {
                        let h = (cad.verts[vi as usize] - c.o).dot(c.axis);
                        c.h0 = c.h0.min(h);
                        c.h1 = c.h1.max(h);
                    }
                }
                let mut all = Vec::new();
                for &t in c.cad_tris.iter().chain(tris.iter()) {
                    for &vi in &cad.tris[t as usize] {
                        let p = cad.verts[vi as usize] - c.o;
                        all.push(p.dot(c.w).atan2(p.dot(c.u)));
                    }
                }
                c.coverage = coverage_deg(&all);
                c.cad_tris.extend(tris);
                c.faces += 1;
            }
            None => out.push(Cylinder { o, axis, u, w, r: *r, h0, h1, hole, coverage: coverage_deg(&ang), faces: 1, cad_tris: tris }),
        }
    }
    out.retain(|c| c.coverage >= 180.0 && c.length() > 1e-4);
    out.sort_by(|a, b| b.hole.cmp(&a.hole).then(b.r.partial_cmp(&a.r).unwrap_or(std::cmp::Ordering::Equal)));
    out
}

/// Kreisausgleich (Kåsa, dann Gauß-Newton): Mittelpunkt, Radius, Rundheit (max−min), Abdeckung (°).
pub fn fit_circle(xy: &[[f64; 2]]) -> Option<(f64, f64, f64, f64, f64)> {
    let n = xy.len();
    if n < 3 {
        return None;
    }
    let (mx, my) = xy.iter().fold((0.0, 0.0), |a, p| (a.0 + p[0], a.1 + p[1]));
    let (mx, my) = (mx / n as f64, my / n as f64);
    let (mut suu, mut suv, mut svv, mut suuu, mut svvv, mut suvv, mut svuu) = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    for p in xy {
        let (u, v) = (p[0] - mx, p[1] - my);
        suu += u * u;
        suv += u * v;
        svv += v * v;
        suuu += u * u * u;
        svvv += v * v * v;
        suvv += u * v * v;
        svuu += v * u * u;
    }
    let det = suu * svv - suv * suv;
    if det.abs() < 1e-24 {
        return None;
    }
    let (b1, b2) = (0.5 * (suuu + suvv), 0.5 * (svvv + svuu));
    let mut cx = (b1 * svv - b2 * suv) / det + mx;
    let mut cy = (suu * b2 - suv * b1) / det + my;
    for _ in 0..10 {
        let ds: Vec<f64> = xy.iter().map(|p| (p[0] - cx).hypot(p[1] - cy)).collect();
        let r = ds.iter().sum::<f64>() / n as f64;
        let (mut ux, mut uy) = (0.0, 0.0);
        for (p, &d) in xy.iter().zip(ds.iter()) {
            if d > 0.0 {
                ux += (cx - p[0]) / d;
                uy += (cy - p[1]) / d;
            }
        }
        ux /= n as f64;
        uy /= n as f64;
        let (mut j11, mut j12, mut j22, mut g1, mut g2) = (0.0, 0.0, 0.0, 0.0, 0.0);
        for (p, &d) in xy.iter().zip(ds.iter()) {
            if d <= 0.0 {
                continue;
            }
            let (ax, ay) = ((cx - p[0]) / d - ux, (cy - p[1]) / d - uy);
            let res = d - r;
            j11 += ax * ax;
            j12 += ax * ay;
            j22 += ay * ay;
            g1 += ax * res;
            g2 += ay * res;
        }
        let dd = j11 * j22 - j12 * j12;
        if dd.abs() < 1e-30 {
            break;
        }
        let (sx, sy) = ((j22 * g1 - j12 * g2) / dd, (j11 * g2 - j12 * g1) / dd);
        cx -= sx;
        cy -= sy;
        if sx.hypot(sy) < 1e-12 {
            break;
        }
    }
    let ds: Vec<f64> = xy.iter().map(|p| (p[0] - cx).hypot(p[1] - cy)).collect();
    let r = ds.iter().sum::<f64>() / n as f64;
    let rmin = ds.iter().cloned().fold(f64::MAX, f64::min);
    let rmax = ds.iter().cloned().fold(f64::MIN, f64::max);
    let ang: Vec<f64> = xy.iter().map(|p| (p[1] - cy).atan2(p[0] - cx)).collect();
    Some((cx, cy, r, rmax - rmin, coverage_deg(&ang)))
}

#[derive(Clone, Debug)]
pub struct CylMeasure {
    /// Ist-Radius (Kreisausgleich durch die Netzecken auf dem Mantel)
    pub r: f64,
    /// Abstand der Ist-Achse von der Soll-Achse
    pub offset: f64,
    /// Rundheit (größter minus kleinster Radius)
    pub roundness: f64,
    pub points: usize,
    pub coverage: f64,
    /// verwendete Dreiecke des Netzes
    pub tris: Vec<u32>,
}

/// Misst jeden Soll-Zylinder am Netz: Manteldreiecke (Normale radial, Lage passend)
/// sammeln und einen Kreis durch ihre Ecken legen.
pub fn measure_cylinders(m: &TriMesh, cyls: &[Cylinder]) -> Vec<Option<CylMeasure>> {
    let nt = m.tris.len();
    let mut fnrm = Vec::with_capacity(nt);
    let mut fc = Vec::with_capacity(nt);
    for t in 0..nt {
        let (a, b, c) = m.tri(t);
        fnrm.push((b - a).cross(c - a).normalized());
        fc.push((a + b + c) / 3.0);
    }
    cyls.iter()
        .map(|c| {
            let win = (c.r * 0.15).max(0.5);
            let mut verts: Vec<u32> = Vec::new();
            let mut tris = Vec::new();
            for t in 0..nt {
                let d = fc[t] - c.o;
                let h = d.dot(c.axis);
                let slack = (c.length() * 0.05).clamp(0.02, 0.3);
                if h < c.h0 - slack || h > c.h1 + slack {
                    continue;
                }
                let n = fnrm[t];
                if n.dot(c.axis).abs() > 0.3 {
                    continue;
                }
                let rv = d - c.axis * h;
                let rr = rv.norm();
                if rr <= 0.0 || (rr - c.r).abs() > win {
                    continue;
                }
                let radial = n.dot(rv) / rr;
                if (c.hole && radial > -0.7) || (!c.hole && radial < 0.7) {
                    continue;
                }
                tris.push(t as u32);
                verts.extend_from_slice(&m.tris[t]);
            }
            verts.sort_unstable();
            verts.dedup();
            if verts.len() < 12 {
                return None;
            }
            // nur Ecken innerhalb der Zylinderlänge (Übergänge an den Enden zählen nicht)
            let mut xy: Vec<[f64; 2]> = verts
                .iter()
                .filter_map(|&v| {
                    let d = m.verts[v as usize] - c.o;
                    let h = d.dot(c.axis);
                    let eps = (c.length() * 0.1).clamp(0.05, 0.5);
                    if h < c.h0 - eps || h > c.h1 + eps {
                        return None;
                    }
                    Some([d.dot(c.u), d.dot(c.w)])
                })
                .collect();
            if xy.len() < 12 {
                return None;
            }
            let mut fit = fit_circle(&xy)?;
            // Ausreißer (Rundungen, Fasen am Rand) entfernen und neu ausgleichen
            for _ in 0..3 {
                let res: Vec<f64> = xy.iter().map(|p| ((p[0] - fit.0).hypot(p[1] - fit.1) - fit.2).abs()).collect();
                let mut srt = res.clone();
                srt.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
                let med = srt[srt.len() / 2];
                let lim = (med * 4.0).max(0.01);
                if srt[srt.len() - 1] <= lim {
                    break;
                }
                let kept: Vec<[f64; 2]> = xy.iter().zip(res.iter()).filter(|(_, &r)| r <= lim).map(|(p, _)| *p).collect();
                if kept.len() < 12 {
                    break;
                }
                xy = kept;
                fit = fit_circle(&xy)?;
            }
            let (cx, cy, r, round, cov) = fit;
            if cov < c.coverage.min(270.0) * 0.6 {
                return None;
            }
            Some(CylMeasure { r, offset: cx.hypot(cy), roundness: round, points: xy.len(), coverage: cov, tris })
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Gesamturteil

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Verdict {
    Ok,
    Warn,
    Fail,
}

/// `cyl_bad`: gemessene Zylinder außerhalb der Toleranz (nicht messbare zählen nicht).
pub fn verdict(max_abs: f64, in_tol: f64, tol: f64, cyl_bad: usize) -> Verdict {
    if max_abs <= tol && cyl_bad == 0 {
        Verdict::Ok
    } else if in_tol >= 0.99 && cyl_bad == 0 {
        Verdict::Warn
    } else {
        Verdict::Fail
    }
}
