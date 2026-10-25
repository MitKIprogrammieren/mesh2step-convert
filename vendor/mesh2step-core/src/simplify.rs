//! Netz vereinfachen: Kantenkollaps mit Fehlerquadriken (Garland–Heckbert).
//!
//! Flache Bereiche werden zu wenigen großen Dreiecken zusammengefasst, Kanten,
//! Rundungen und offene Ränder bleiben fein. Nie weiter als `tol` (mm) von der
//! ursprünglichen Fläche entfernt; Dreiecke klappen nicht um und die Topologie
//! (geschlossen, mannigfaltig) bleibt erhalten.

use crate::math::V3;
use std::cmp::Ordering;
use std::collections::BinaryHeap;

#[derive(Clone, Debug)]
pub struct SimplifyOpts {
    /// Zielanteil der Dreiecke (0..1)
    pub ratio: f64,
    /// größte erlaubte Abweichung von der Originalfläche in mm
    pub tol: f64,
}

impl Default for SimplifyOpts {
    fn default() -> Self {
        SimplifyOpts { ratio: 0.3, tol: 0.05 }
    }
}

/// Symmetrische 4x4-Quadrik (10 Werte).
#[derive(Clone, Copy, Default)]
struct Q([f64; 10]);

impl Q {
    fn plane(n: V3, d: f64, w: f64) -> Q {
        let (a, b, c) = (n.x, n.y, n.z);
        Q([a * a * w, a * b * w, a * c * w, a * d * w, b * b * w, b * c * w, b * d * w, c * c * w, c * d * w, d * d * w])
    }
    fn add(&mut self, o: &Q) {
        for i in 0..10 {
            self.0[i] += o.0[i];
        }
    }
    fn sum(a: &Q, b: &Q) -> Q {
        let mut q = *a;
        q.add(b);
        q
    }
    fn err(&self, p: V3) -> f64 {
        let q = &self.0;
        let (x, y, z) = (p.x, p.y, p.z);
        (q[0] * x * x + 2.0 * q[1] * x * y + 2.0 * q[2] * x * z + 2.0 * q[3] * x + q[4] * y * y + 2.0 * q[5] * y * z + 2.0 * q[6] * y + q[7] * z * z + 2.0 * q[8] * z + q[9]).max(0.0)
    }
    /// Punkt mit kleinstem Fehler (falls die Matrix gut lösbar ist).
    fn optimum(&self) -> Option<V3> {
        let q = &self.0;
        let a = [[q[0], q[1], q[2]], [q[1], q[4], q[5]], [q[2], q[5], q[7]]];
        let b = [-q[3], -q[6], -q[8]];
        let det = a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
        let scale = (a[0][0] + a[1][1] + a[2][2]).abs().max(1e-300);
        if det.abs() < 1e-9 * scale * scale * scale {
            return None;
        }
        let inv = 1.0 / det;
        let x = (b[0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) - a[0][1] * (b[1] * a[2][2] - a[1][2] * b[2]) + a[0][2] * (b[1] * a[2][1] - a[1][1] * b[2])) * inv;
        let y = (a[0][0] * (b[1] * a[2][2] - a[1][2] * b[2]) - b[0] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) + a[0][2] * (a[1][0] * b[2] - b[1] * a[2][0])) * inv;
        let z = (a[0][0] * (a[1][1] * b[2] - b[1] * a[2][1]) - a[0][1] * (a[1][0] * b[2] - b[1] * a[2][0]) + b[0] * (a[1][0] * a[2][1] - a[1][1] * a[2][0])) * inv;
        let p = V3 { x, y, z };
        if p.is_finite() {
            Some(p)
        } else {
            None
        }
    }
}

#[derive(PartialEq)]
struct Cand {
    cost: f64,
    a: u32,
    b: u32,
    /// Stand der beiden Punkte bei der Berechnung (veraltete Einträge überspringen)
    va: u32,
    vb: u32,
    p: V3,
}

impl Eq for Cand {}
impl PartialOrd for Cand {
    fn partial_cmp(&self, o: &Self) -> Option<Ordering> {
        Some(self.cmp(o))
    }
}
impl Ord for Cand {
    fn cmp(&self, o: &Self) -> Ordering {
        // kleinste Kosten zuerst (BinaryHeap ist ein Max-Heap)
        o.cost.partial_cmp(&self.cost).unwrap_or(Ordering::Equal)
    }
}

fn tri_normal(v: &[V3], t: &[u32; 3]) -> V3 {
    (v[t[1] as usize] - v[t[0] as usize]).cross(v[t[2] as usize] - v[t[0] as usize])
}

/// Vereinfacht ein indiziertes Netz (Punkte sollten verschweißt sein).
/// Rückgabe: neue Punkte und Dreiecke.
pub fn simplify(verts: &[V3], tris: &[[u32; 3]], o: &SimplifyOpts) -> (Vec<V3>, Vec<[u32; 3]>) {
    let nv = verts.len();
    let mut v: Vec<V3> = verts.to_vec();
    let mut t: Vec<[u32; 3]> = tris.to_vec();
    let mut alive = vec![true; t.len()];
    let target = ((t.len() as f64) * o.ratio.clamp(0.01, 1.0)).round() as usize;
    let max_cost = o.tol * o.tol;

    // Dreiecke je Punkt
    let mut vt: Vec<Vec<u32>> = vec![Vec::new(); nv];
    for (i, tr) in t.iter().enumerate() {
        for &k in tr {
            vt[k as usize].push(i as u32);
        }
    }
    // Quadriken aus den Flächenebenen
    let mut q = vec![Q::default(); nv];
    for tr in &t {
        let n = tri_normal(&v, tr);
        let a = n.norm();
        if a <= 1e-300 {
            continue;
        }
        let nn = n / a;
        let d = -nn.dot(v[tr[0] as usize]);
        let pq = Q::plane(nn, d, 1.0);
        for &k in tr {
            q[k as usize].add(&pq);
        }
    }
    // Offene Ränder festhalten: senkrechte Ebene durch jede Randkante, stark gewichtet
    {
        let mut edges: Vec<(u32, u32, u32)> = Vec::with_capacity(t.len() * 3);
        for (i, tr) in t.iter().enumerate() {
            for k in 0..3 {
                let (a, b) = (tr[k], tr[(k + 1) % 3]);
                edges.push((a.min(b), a.max(b), i as u32));
            }
        }
        edges.sort_unstable();
        let mut i = 0;
        while i < edges.len() {
            let mut j = i + 1;
            while j < edges.len() && edges[j].0 == edges[i].0 && edges[j].1 == edges[i].1 {
                j += 1;
            }
            if j - i == 1 {
                let (a, b, ti) = edges[i];
                let (pa, pb) = (v[a as usize], v[b as usize]);
                let e = pb - pa;
                let n = tri_normal(&v, &t[ti as usize]);
                let side = e.cross(n);
                if side.norm() > 1e-300 {
                    let sn = side.normalized();
                    let pq = Q::plane(sn, -sn.dot(pa), 1000.0);
                    q[a as usize].add(&pq);
                    q[b as usize].add(&pq);
                }
            }
            i = j;
        }
    }

    let mut ver = vec![0u32; nv];
    let mut heap = BinaryHeap::new();

    let neighbors = |vt: &Vec<Vec<u32>>, t: &Vec<[u32; 3]>, alive: &Vec<bool>, a: u32| -> Vec<u32> {
        let mut n = Vec::new();
        for &ti in &vt[a as usize] {
            if !alive[ti as usize] {
                continue;
            }
            for &k in &t[ti as usize] {
                if k != a && !n.contains(&k) {
                    n.push(k);
                }
            }
        }
        n
    };

    let best_point = |q: &Vec<Q>, v: &Vec<V3>, a: u32, b: u32| -> (f64, V3) {
        let qs = Q::sum(&q[a as usize], &q[b as usize]);
        let (pa, pb) = (v[a as usize], v[b as usize]);
        let mut cands = vec![pa, pb, (pa + pb) * 0.5];
        if let Some(p) = qs.optimum() {
            // nicht weit weg von der Kante legen (schlecht bestimmte Matrix)
            let l = pa.dist(pb);
            if p.dist((pa + pb) * 0.5) <= l * 1.5 + 1e-9 {
                cands.push(p);
            }
        }
        let mut best = (f64::MAX, pa);
        for p in cands {
            let e = qs.err(p);
            if e < best.0 {
                best = (e, p);
            }
        }
        best
    };

    let push_edges = |heap: &mut BinaryHeap<Cand>, q: &Vec<Q>, v: &Vec<V3>, vt: &Vec<Vec<u32>>, t: &Vec<[u32; 3]>, alive: &Vec<bool>, ver: &Vec<u32>, a: u32| {
        for b in neighbors(vt, t, alive, a) {
            let (cost, p) = best_point(q, v, a, b);
            heap.push(Cand { cost, a, b, va: ver[a as usize], vb: ver[b as usize], p });
        }
    };

    for a in 0..nv as u32 {
        for b in neighbors(&vt, &t, &alive, a) {
            if a < b {
                let (cost, p) = best_point(&q, &v, a, b);
                heap.push(Cand { cost, a, b, va: 0, vb: 0, p });
            }
        }
    }

    let mut count = alive.iter().filter(|&&x| x).count();
    while count > target {
        let c = match heap.pop() {
            Some(c) => c,
            None => break,
        };
        if c.cost > max_cost {
            break;
        }
        let (a, b) = (c.a, c.b);
        if ver[a as usize] != c.va || ver[b as usize] != c.vb {
            continue;
        }
        // Dreiecke an der Kante und Nachbarschaften
        let shared: Vec<u32> = vt[a as usize].iter().copied().filter(|&ti| alive[ti as usize] && t[ti as usize].contains(&b)).collect();
        if shared.is_empty() || shared.len() > 2 {
            continue;
        }
        // Verbindungsbedingung: gemeinsame Nachbarn nur die gegenüberliegenden Ecken
        let na = neighbors(&vt, &t, &alive, a);
        let nb = neighbors(&vt, &t, &alive, b);
        let common = na.iter().filter(|x| nb.contains(x)).count();
        if common != shared.len() {
            continue;
        }
        // keine umklappenden oder entartenden Dreiecke
        let mut ok = true;
        for &(x, other) in &[(a, b), (b, a)] {
            for &ti in &vt[x as usize] {
                if !alive[ti as usize] || t[ti as usize].contains(&other) {
                    continue;
                }
                let tr = t[ti as usize];
                let n0 = tri_normal(&v, &tr);
                let pt = |k: u32| if k == x { c.p } else { v[k as usize] };
                let (p0, p1, p2) = (pt(tr[0]), pt(tr[1]), pt(tr[2]));
                let n1 = (p1 - p0).cross(p2 - p0);
                let l0 = n0.norm();
                let l1 = n1.norm();
                if l1 <= l0 * 1e-6 || n0.dot(n1) <= 0.2 * l0 * l1 {
                    ok = false;
                    break;
                }
            }
            if !ok {
                break;
            }
        }
        if !ok {
            continue;
        }
        // Kollaps: b verschwindet in a
        v[a as usize] = c.p;
        let qb = q[b as usize];
        q[a as usize].add(&qb);
        for &ti in &shared {
            alive[ti as usize] = false;
            count -= 1;
        }
        let tb: Vec<u32> = vt[b as usize].clone();
        for ti in tb {
            if !alive[ti as usize] {
                continue;
            }
            for k in t[ti as usize].iter_mut() {
                if *k == b {
                    *k = a;
                }
            }
            vt[a as usize].push(ti);
        }
        vt[b as usize].clear();
        vt[a as usize].retain(|&ti| alive[ti as usize]);
        ver[a as usize] += 1;
        ver[b as usize] += 1;
        // nur Kanten an a ändern sich (Lage und Quadrik von a); ältere Einträge
        // mit a sind über den Stand von a veraltet
        push_edges(&mut heap, &q, &v, &vt, &t, &alive, &ver, a);
    }

    // aufräumen
    let mut newi = vec![u32::MAX; nv];
    let mut out_v = Vec::new();
    let mut out_t = Vec::with_capacity(count);
    for (i, tr) in t.iter().enumerate() {
        if !alive[i] {
            continue;
        }
        let mut nt = [0u32; 3];
        for k in 0..3 {
            let x = tr[k] as usize;
            if newi[x] == u32::MAX {
                newi[x] = out_v.len() as u32;
                out_v.push(v[x]);
            }
            nt[k] = newi[x];
        }
        out_t.push(nt);
    }
    (out_v, out_t)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::v3;

    fn sphere(r: f64, nu: usize, nv: usize) -> (Vec<V3>, Vec<[u32; 3]>) {
        let mut v = vec![v3(0.0, 0.0, r)];
        for j in 1..nv {
            let th = std::f64::consts::PI * j as f64 / nv as f64;
            for i in 0..nu {
                let ph = std::f64::consts::TAU * i as f64 / nu as f64;
                v.push(v3(r * th.sin() * ph.cos(), r * th.sin() * ph.sin(), r * th.cos()));
            }
        }
        v.push(v3(0.0, 0.0, -r));
        let s = (v.len() - 1) as u32;
        let id = |j: usize, i: usize| (1 + (j - 1) * nu + i % nu) as u32;
        let mut t = Vec::new();
        for i in 0..nu {
            t.push([0, id(1, i), id(1, i + 1)]);
        }
        for j in 1..nv - 1 {
            for i in 0..nu {
                t.push([id(j, i), id(j + 1, i), id(j + 1, i + 1)]);
                t.push([id(j, i), id(j + 1, i + 1), id(j, i + 1)]);
            }
        }
        for i in 0..nu {
            t.push([s, id(nv - 1, i + 1), id(nv - 1, i)]);
        }
        (v, t)
    }

    fn closed(v: &[V3], t: &[[u32; 3]]) -> bool {
        let a = crate::repair::analyze_indexed(v.to_vec(), t.to_vec(), false);
        a.issues.watertight()
    }

    /// Flaches, fein unterteiltes Quadrat als Deckel eines Würfels.
    fn fine_box(n: usize) -> (Vec<V3>, Vec<[u32; 3]>) {
        // Würfel 20 mm, jede Seite n x n Felder
        let mut v = Vec::new();
        let mut t = Vec::new();
        let mut idx = std::collections::HashMap::new();
        let mut id = |p: V3, v: &mut Vec<V3>| -> u32 {
            let k = ((p.x * 1000.0).round() as i64, (p.y * 1000.0).round() as i64, (p.z * 1000.0).round() as i64);
            *idx.entry(k).or_insert_with(|| {
                v.push(p);
                (v.len() - 1) as u32
            })
        };
        let s = 20.0;
        let faces: [(V3, V3, V3); 6] = [
            (v3(0., 0., 0.), v3(0., s, 0.), v3(s, 0., 0.)),
            (v3(0., 0., s), v3(s, 0., 0.), v3(0., s, 0.)),
            (v3(0., 0., 0.), v3(s, 0., 0.), v3(0., 0., s)),
            (v3(0., s, 0.), v3(0., 0., s), v3(s, 0., 0.)),
            (v3(0., 0., 0.), v3(0., 0., s), v3(0., s, 0.)),
            (v3(s, 0., 0.), v3(0., s, 0.), v3(0., 0., s)),
        ];
        for (o, du, dv) in faces {
            for j in 0..n {
                for i in 0..n {
                    let p = |a: usize, b: usize| o + du * (a as f64 / n as f64) + dv * (b as f64 / n as f64);
                    let a = id(p(i, j), &mut v);
                    let b = id(p(i + 1, j), &mut v);
                    let c = id(p(i + 1, j + 1), &mut v);
                    let d = id(p(i, j + 1), &mut v);
                    t.push([a, b, c]);
                    t.push([a, c, d]);
                }
            }
        }
        (v, t)
    }

    #[test]
    fn flat_box_collapses() {
        let (v, t) = fine_box(20);
        assert_eq!(t.len(), 6 * 800);
        assert!(closed(&v, &t));
        let (v2, t2) = simplify(&v, &t, &SimplifyOpts { ratio: 0.01, tol: 0.01 });
        assert!(closed(&v2, &t2), "nicht mehr geschlossen");
        assert!(t2.len() <= 200, "nur auf {} reduziert", t2.len());
        // alle Punkte auf der Würfeloberfläche
        for p in &v2 {
            let d = [p.x, p.y, p.z, 20.0 - p.x, 20.0 - p.y, 20.0 - p.z].iter().fold(f64::MAX, |m, x| m.min(x.abs()));
            assert!(d < 1e-6, "Punkt neben der Fläche: {p:?}");
        }
        let vol = crate::repair::analyze_indexed(v2, t2, false).issues.volume;
        assert!((vol - 8000.0).abs() < 1e-3, "{vol}");
    }

    #[test]
    fn sphere_keeps_shape() {
        let (v, t) = sphere(20.0, 128, 64);
        let n0 = t.len();
        let tol = 0.05;
        let (v2, t2) = simplify(&v, &t, &SimplifyOpts { ratio: 0.3, tol });
        assert!(closed(&v2, &t2));
        assert!(t2.len() < n0 / 2, "{} von {n0}", t2.len());
        for p in &v2 {
            assert!((p.norm() - 20.0).abs() < tol * 1.5, "Abweichung {}", p.norm() - 20.0);
        }
    }

    #[test]
    fn open_rim_stays() {
        // Halbkugel (offen): Rand muss erhalten bleiben
        let (v, t) = sphere(20.0, 64, 32);
        let keep: Vec<[u32; 3]> = t.into_iter().filter(|tr| tr.iter().all(|&k| v[k as usize].z >= -1e-9)).collect();
        let rim0: Vec<V3> = v.iter().copied().filter(|p| p.z.abs() < 1e-9).collect();
        let (v2, _t2) = simplify(&v, &keep, &SimplifyOpts { ratio: 0.2, tol: 0.05 });
        let rim1 = v2.iter().filter(|p| p.z.abs() < 1e-6).count();
        assert!(rim1 >= rim0.len() * 9 / 10, "Rand {} von {}", rim1, rim0.len());
    }
}
