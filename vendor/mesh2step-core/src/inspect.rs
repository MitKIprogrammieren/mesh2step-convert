//! Prüfwerkzeuge für die Vorschau: Wandstärke je Dreieck und erkannte Bohrungen.

use crate::math::V3;
use crate::mesh::Mesh;
use crate::segment::Segmentation;
use crate::surf::Surf;

type T3 = [[f32; 3]; 3];

fn v(p: [f32; 3]) -> V3 {
    V3 { x: p[0] as f64, y: p[1] as f64, z: p[2] as f64 }
}

/// Gleichmäßiges Raster über den Dreiecken für schnelle Strahltests.
pub struct RayGrid<'a> {
    tris: &'a [T3],
    lo: V3,
    cell: f64,
    n: [usize; 3],
    start: Vec<u32>,
    items: Vec<u32>,
}

impl<'a> RayGrid<'a> {
    pub fn new(tris: &'a [T3]) -> RayGrid<'a> {
        let mut lo = V3 { x: f64::MAX, y: f64::MAX, z: f64::MAX };
        let mut hi = V3 { x: f64::MIN, y: f64::MIN, z: f64::MIN };
        for t in tris {
            for p in t {
                let q = v(*p);
                lo = V3 { x: lo.x.min(q.x), y: lo.y.min(q.y), z: lo.z.min(q.z) };
                hi = V3 { x: hi.x.max(q.x), y: hi.y.max(q.y), z: hi.z.max(q.z) };
            }
        }
        if tris.is_empty() {
            lo = V3 { x: 0.0, y: 0.0, z: 0.0 };
            hi = V3 { x: 1.0, y: 1.0, z: 1.0 };
        }
        let ext = hi - lo;
        let maxe = ext.x.max(ext.y).max(ext.z).max(1e-9);
        let res = ((tris.len() as f64).cbrt() * 2.0).clamp(1.0, 200.0);
        let cell = maxe / res;
        let n = [(ext.x / cell).ceil().max(1.0) as usize, (ext.y / cell).ceil().max(1.0) as usize, (ext.z / cell).ceil().max(1.0) as usize];
        let ncell = n[0] * n[1] * n[2];
        let idx = |q: V3| -> [usize; 3] {
            let d = q - lo;
            [((d.x / cell) as usize).min(n[0] - 1), ((d.y / cell) as usize).min(n[1] - 1), ((d.z / cell) as usize).min(n[2] - 1)]
        };
        // zweimal durchlaufen: zählen, dann einsortieren
        let mut count = vec![0u32; ncell + 1];
        let ranges: Vec<([usize; 3], [usize; 3])> = tris
            .iter()
            .map(|t| {
                let (a, b, c) = (v(t[0]), v(t[1]), v(t[2]));
                let mn = V3 { x: a.x.min(b.x).min(c.x), y: a.y.min(b.y).min(c.y), z: a.z.min(b.z).min(c.z) };
                let mx = V3 { x: a.x.max(b.x).max(c.x), y: a.y.max(b.y).max(c.y), z: a.z.max(b.z).max(c.z) };
                (idx(mn), idx(mx))
            })
            .collect();
        for (a, b) in &ranges {
            for x in a[0]..=b[0] {
                for y in a[1]..=b[1] {
                    for z in a[2]..=b[2] {
                        count[(z * n[1] + y) * n[0] + x + 1] += 1;
                    }
                }
            }
        }
        for i in 1..count.len() {
            count[i] += count[i - 1];
        }
        let start = count.clone();
        let mut fill = count;
        let mut items = vec![0u32; *start.last().unwrap() as usize];
        for (t, (a, b)) in ranges.iter().enumerate() {
            for x in a[0]..=b[0] {
                for y in a[1]..=b[1] {
                    for z in a[2]..=b[2] {
                        let c = (z * n[1] + y) * n[0] + x;
                        items[fill[c] as usize] = t as u32;
                        fill[c] += 1;
                    }
                }
            }
        }
        RayGrid { tris, lo, cell, n, start, items }
    }

    /// Nächster Treffer entlang des Strahls (Abstand), Dreieck `skip` ausgenommen.
    pub fn hit(&self, o: V3, d: V3, max: f64, skip: u32) -> Option<f64> {
        let n = self.n;
        let p = o - self.lo;
        let mut c = [0i64; 3];
        let pc = [p.x / self.cell, p.y / self.cell, p.z / self.cell];
        for k in 0..3 {
            c[k] = pc[k].floor() as i64;
            if c[k] < 0 || c[k] >= n[k] as i64 {
                // Start außerhalb des Rasters: nur innen liegende Startpunkte werden verwendet
                c[k] = c[k].clamp(0, n[k] as i64 - 1);
            }
        }
        let dd = [d.x, d.y, d.z];
        let mut step = [0i64; 3];
        let mut tmax = [f64::INFINITY; 3];
        let mut tdelta = [f64::INFINITY; 3];
        for k in 0..3 {
            if dd[k] > 1e-12 {
                step[k] = 1;
                tmax[k] = ((c[k] + 1) as f64 - pc[k]) * self.cell / dd[k];
                tdelta[k] = self.cell / dd[k];
            } else if dd[k] < -1e-12 {
                step[k] = -1;
                tmax[k] = (pc[k] - c[k] as f64) * self.cell / -dd[k];
                tdelta[k] = self.cell / -dd[k];
            }
        }
        let mut best = f64::INFINITY;
        let mut t_cell;
        loop {
            let ci = ((c[2] as usize * n[1]) + c[1] as usize) * n[0] + c[0] as usize;
            for &t in &self.items[self.start[ci] as usize..self.start[ci + 1] as usize] {
                if t == skip {
                    continue;
                }
                let tr = &self.tris[t as usize];
                if let Some(h) = ray_tri(o, d, v(tr[0]), v(tr[1]), v(tr[2])) {
                    if h > 1e-6 && h < best {
                        best = h;
                    }
                }
            }
            let k = if tmax[0] < tmax[1] { if tmax[0] < tmax[2] { 0 } else { 2 } } else if tmax[1] < tmax[2] { 1 } else { 2 };
            t_cell = tmax[k].min(f64::INFINITY);
            if best <= t_cell || t_cell > max || !t_cell.is_finite() {
                break;
            }
            c[k] += step[k];
            if c[k] < 0 || c[k] >= n[k] as i64 {
                break;
            }
            tmax[k] += tdelta[k];
        }
        let _ = t_cell;
        if best <= max {
            Some(best)
        } else {
            None
        }
    }
}

fn ray_tri(o: V3, d: V3, a: V3, b: V3, c: V3) -> Option<f64> {
    let e1 = b - a;
    let e2 = c - a;
    let p = d.cross(e2);
    let det = e1.dot(p);
    if det.abs() < 1e-14 {
        return None;
    }
    let inv = 1.0 / det;
    let s = o - a;
    let u = s.dot(p) * inv;
    if !(-1e-9..=1.0 + 1e-9).contains(&u) {
        return None;
    }
    let q = s.cross(e1);
    let w = d.dot(q) * inv;
    if w < -1e-9 || u + w > 1.0 + 1e-9 {
        return None;
    }
    Some(e2.dot(q) * inv)
}

/// Wandstärke je Dreieck: Abstand vom Dreiecksmittelpunkt entgegen der Normalen bis zur
/// gegenüberliegenden Wand (unendlich, wenn nichts getroffen wird).
pub fn thickness(tris: &[T3]) -> Vec<f32> {
    thickness_checked(tris, 0.0)
}

/// Wie `thickness`; Werte unter `recheck` werden mit fünf leicht geneigten Strahlen
/// nachgemessen (Median). Das trennt echte dünne Wände von Stellen direkt an Kanten,
/// wo ein einzelner Strahl zufällig früh auf die Nachbarfläche trifft.
pub fn thickness_checked(tris: &[T3], recheck: f64) -> Vec<f32> {
    let g = RayGrid::new(tris);
    tris.iter()
        .enumerate()
        .map(|(i, t)| {
            let (a, b, c) = (v(t[0]), v(t[1]), v(t[2]));
            let n = (b - a).cross(c - a);
            let l = n.norm();
            if l < 1e-14 {
                return f32::INFINITY;
            }
            let d = n * (-1.0 / l);
            let o = (a + b + c) * (1.0 / 3.0);
            let h0 = g.hit(o, d, 1e9, i as u32).unwrap_or(f64::INFINITY);
            if h0 >= recheck {
                return h0 as f32;
            }
            let (e1, e2) = d.basis();
            let tilt = 0.4; // ~22°
            let mut hs: Vec<f64> = vec![h0];
            for (x, y) in [(1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0)] {
                let dd = (d + e1 * (x * tilt) + e2 * (y * tilt)).normalized();
                // senkrechter Anteil, damit geneigte Strahlen dieselbe Wanddicke messen
                let h = g.hit(o, dd, 1e9, i as u32).unwrap_or(f64::INFINITY) * dd.dot(d);
                hs.push(h);
            }
            hs.sort_by(|x, y| x.partial_cmp(y).unwrap_or(std::cmp::Ordering::Equal));
            hs[2] as f32
        })
        .collect()
}

/// Ausgedünnte Anzeige-Version eines großen Netzes (Eckpunkte auf ein Raster zusammenziehen).
/// Anders als jedes n-te Dreieck wegzulassen bleibt die Oberfläche geschlossen.
/// Rückgabe: Dreiecke und je Dreieck der Index des Ursprungsdreiecks (für Farben und Auswahl).
pub fn decimate_display(tris: &[T3], target: usize) -> (Vec<T3>, Vec<u32>) {
    if tris.len() <= target || target == 0 {
        return (tris.to_vec(), (0..tris.len() as u32).collect());
    }
    let mut area = 0.0f64;
    let mut lo = [f32::MAX; 3];
    for t in tris {
        let (a, b, c) = (v(t[0]), v(t[1]), v(t[2]));
        area += (b - a).cross(c - a).norm() * 0.5;
        for p in t {
            for k in 0..3 {
                lo[k] = lo[k].min(p[k]);
            }
        }
    }
    let mut h = ((2.0 * area / target as f64).sqrt() as f32).max(1e-6);
    let mut best: (Vec<T3>, Vec<u32>) = (Vec::new(), Vec::new());
    for _ in 0..8 {
        let key = |p: &[f32; 3]| -> (i32, i32, i32) { (((p[0] - lo[0]) / h) as i32, ((p[1] - lo[1]) / h) as i32, ((p[2] - lo[2]) / h) as i32) };
        // Mittelpunkt je Rasterzelle
        let mut cells: std::collections::HashMap<(i32, i32, i32), ([f64; 3], u32)> = std::collections::HashMap::with_capacity(tris.len());
        for t in tris {
            for p in t {
                let e = cells.entry(key(p)).or_insert(([0.0; 3], 0));
                for k in 0..3 {
                    e.0[k] += p[k] as f64;
                }
                e.1 += 1;
            }
        }
        let rep = |p: &[f32; 3]| -> [f32; 3] {
            let (s, n) = cells[&key(p)];
            [(s[0] / n as f64) as f32, (s[1] / n as f64) as f32, (s[2] / n as f64) as f32]
        };
        let mut seen: std::collections::HashSet<[(i32, i32, i32); 3]> = std::collections::HashSet::new();
        let (mut out, mut src) = (Vec::new(), Vec::new());
        for (i, t) in tris.iter().enumerate() {
            let k = [key(&t[0]), key(&t[1]), key(&t[2])];
            if k[0] == k[1] || k[1] == k[2] || k[0] == k[2] {
                continue;
            }
            // gleiche Zellen in gleicher Umlaufrichtung nur einmal
            let m = (0..3).min_by_key(|&j| k[j]).unwrap();
            let canon = [k[m], k[(m + 1) % 3], k[(m + 2) % 3]];
            if !seen.insert(canon) {
                continue;
            }
            out.push([rep(&t[0]), rep(&t[1]), rep(&t[2])]);
            src.push(i as u32);
        }
        let done = out.len() <= target + target / 5;
        best = (out, src);
        if done {
            break;
        }
        h *= 1.3;
    }
    best
}

/// Eine erkannte Bohrung (zylindrische Innenfläche).
#[derive(Clone, Debug, Default)]
pub struct Hole {
    pub diameter: f64,
    pub depth: f64,
    pub through: bool,
    /// Dreiecke in der Vorschau
    pub tris: Vec<u32>,
}

/// Gleich große Bohrungen zusammengefasst.
#[derive(Clone, Debug, Default)]
pub struct HoleGroup {
    pub diameter: f64,
    pub count: usize,
    /// Tiefe (bei Sacklöchern), sonst 0
    pub depth: f64,
    pub through: bool,
    pub hint: String,
    pub tris: Vec<u32>,
}

/// Zylinderflächen, deren Normalen zur Achse zeigen, als Bohrungen erkennen.
/// `base` ist der Index des ersten Dreiecks dieses Netzes in der Vorschau.
pub fn find_holes(m: &Mesh, seg: &Segmentation, base: u32) -> Vec<Hole> {
    // Teilstücke desselben Zylinders (gleiche Achse, gleicher Radius) zusammenfassen
    let mut cyls: Vec<(V3, V3, f64, Vec<u32>)> = Vec::new();
    for r in &seg.regions {
        let Surf::Cyl { p, a, r: rad } = r.surf else { continue };
        if r.tris.len() < 2 || rad > 60.0 {
            continue;
        }
        // nach innen gewölbt? (Normale zeigt zur Achse)
        let mut inward = 0.0;
        for &t in &r.tris {
            let c = m.tri_centroid(t as usize);
            let rel = c - p;
            let radial = rel - a * rel.dot(a);
            inward += -m.tri_n[t as usize].dot(radial) * m.tri_a[t as usize];
        }
        if inward <= 0.0 {
            continue;
        }
        let found = cyls.iter_mut().find(|(p2, a2, r2, _)| {
            (r2 - rad).abs() < 0.02 * rad.max(0.5) && a2.dot(a).abs() > 0.999 && {
                let rel = p - *p2;
                (rel - *a2 * rel.dot(*a2)).norm() < 0.05 * rad.max(0.5)
            }
        });
        match found {
            Some(c) => c.3.extend_from_slice(&r.tris),
            None => cyls.push((p, a, rad, r.tris.clone())),
        }
    }
    // Strahltest auf dem Netz für „durchgehend“
    let tri_f: Vec<T3> = m
        .tris
        .iter()
        .map(|t| t.map(|i| {
            let q = m.verts[i as usize];
            [q.x as f32, q.y as f32, q.z as f32]
        }))
        .collect();
    let grid = if cyls.is_empty() { None } else { Some(RayGrid::new(&tri_f)) };
    let mut out = Vec::new();
    for (p, a, rad, tris) in cyls {
        // Ausdehnung entlang der Achse und Winkelabdeckung (Halbschalen sind keine Bohrung)
        let (mut lo, mut hi) = (f64::MAX, f64::MIN);
        let e1 = {
            let t = if a.x.abs() < 0.9 { V3 { x: 1.0, y: 0.0, z: 0.0 } } else { V3 { x: 0.0, y: 1.0, z: 0.0 } };
            let u = a.cross(t);
            u * (1.0 / u.norm())
        };
        let e2 = a.cross(e1);
        let mut bins = [false; 24];
        for &t in &tris {
            for &vi in &m.tris[t as usize] {
                let rel = m.verts[vi as usize] - p;
                let s = rel.dot(a);
                lo = lo.min(s);
                hi = hi.max(s);
                let ang = rel.dot(e2).atan2(rel.dot(e1));
                let b = (((ang + std::f64::consts::PI) / (2.0 * std::f64::consts::PI)) * 24.0) as usize;
                bins[b.min(23)] = true;
            }
        }
        if bins.iter().filter(|&&x| x).count() < 20 {
            continue;
        }
        let depth = hi - lo;
        // auf der Achse in der Mitte starten und in beide Richtungen schauen: trifft der Strahl
        // kurz hinter dem Ende eine Wand, ist dort ein Boden (Sackloch)
        let mid = p + a * ((lo + hi) * 0.5);
        let open = |dir: V3| -> bool {
            match grid.as_ref().and_then(|g| g.hit(mid, dir, depth * 0.5 + rad.max(0.5) * 0.5 + 0.2, u32::MAX)) {
                Some(_) => false,
                None => true,
            }
        };
        let through = open(a) && open(a * -1.0);
        out.push(Hole { diameter: rad * 2.0, depth, through, tris: tris.iter().map(|&t| t + base).collect() });
    }
    out
}

/// Schrauben-Hinweis zu einem Bohrungsdurchmesser.
pub fn screw_hint(d: f64) -> &'static str {
    match d {
        x if (1.5..1.75).contains(&x) => "M2 Kernloch",
        x if (2.05..2.3).contains(&x) => "M2,5 Kernloch",
        x if (2.3..2.45).contains(&x) => "M2 Durchgang",
        x if (2.45..2.65).contains(&x) => "M3 Kernloch",
        x if (2.65..2.95).contains(&x) => "M2,5 Durchgang",
        x if (2.95..3.65).contains(&x) => "M3 Durchgang · M4 Kernloch",
        x if (4.1..4.65).contains(&x) => "M4 Durchgang · M5 Kernloch",
        x if (5.1..5.7).contains(&x) => "M5 Durchgang",
        x if (6.1..6.7).contains(&x) => "M6 Durchgang",
        x if (8.2..8.7).contains(&x) => "M8 Durchgang",
        _ => "",
    }
}

/// Bohrungen gleichen Durchmessers (auf 0,1 mm) und gleicher Art zusammenfassen.
pub fn group_holes(holes: &[Hole]) -> Vec<HoleGroup> {
    let mut groups: Vec<HoleGroup> = Vec::new();
    for h in holes {
        let d = (h.diameter * 10.0).round() / 10.0;
        let depth = if h.through { 0.0 } else { (h.depth * 10.0).round() / 10.0 };
        match groups.iter_mut().find(|g| (g.diameter - d).abs() < 0.05 && g.through == h.through && (g.depth - depth).abs() < 0.25) {
            Some(g) => {
                g.count += 1;
                g.tris.extend_from_slice(&h.tris);
            }
            None => groups.push(HoleGroup { diameter: d, count: 1, depth, through: h.through, hint: screw_hint(d).to_string(), tris: h.tris.clone() }),
        }
    }
    groups.sort_by(|a, b| a.diameter.partial_cmp(&b.diameter).unwrap());
    groups
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cube(s: f32) -> Vec<T3> {
        let p = |x: f32, y: f32, z: f32| [x * s, y * s, z * s];
        let q = [
            [p(0., 0., 0.), p(1., 0., 0.), p(1., 1., 0.), p(0., 1., 0.)],
            [p(0., 0., 1.), p(0., 1., 1.), p(1., 1., 1.), p(1., 0., 1.)],
            [p(0., 0., 0.), p(0., 0., 1.), p(1., 0., 1.), p(1., 0., 0.)],
            [p(0., 1., 0.), p(1., 1., 0.), p(1., 1., 1.), p(0., 1., 1.)],
            [p(0., 0., 0.), p(0., 1., 0.), p(0., 1., 1.), p(0., 0., 1.)],
            [p(1., 0., 0.), p(1., 0., 1.), p(1., 1., 1.), p(1., 1., 0.)],
        ];
        let mut out = Vec::new();
        for f in q {
            // Umlauf so, dass die Normale nach außen zeigt
            out.push([f[0], f[2], f[1]]);
            out.push([f[0], f[3], f[2]]);
        }
        out
    }

    #[test]
    fn decimate_keeps_small_meshes() {
        let t = cube(1.0);
        let (d, src) = decimate_display(&t, 100);
        assert_eq!(d.len(), 12);
        assert_eq!(src.len(), 12);
    }

    #[test]
    fn cube_thickness_is_edge_length() {
        let t = cube(2.0);
        let th = thickness(&t);
        for x in th {
            assert!((x - 2.0).abs() < 1e-4, "{x}");
        }
    }
}
