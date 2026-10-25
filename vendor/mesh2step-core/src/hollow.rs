//! Aushöhlen: geschlossene Hohlraum-Schale im Abstand `wall` unter der Außenhaut.
//!
//! Abstandsfeld auf einem Raster (innen/außen per Strahl, Abstand zur Oberfläche in
//! einem Band um die Haut), daraus die Grenzfläche des Hohlraums mit „Surface Nets“.
//! Die Außenhaut bleibt unverändert; die Hohlraum-Schale zeigt nach innen (wie ein
//! Hohlraum im Volumenkörper), sodass Slicer Wände um den Hohlraum drucken.

use crate::math::{v3, V3};

#[derive(Clone, Debug)]
pub struct HollowOpts {
    /// Wandstärke in mm
    pub wall: f64,
    /// Rasterweite in mm (0 = automatisch)
    pub voxel: f64,
    /// Decke des Hohlraums als 45°-Dach formen, damit der Drucker innen nichts „in die Luft“
    /// drucken muss (Druckrichtung +Z)
    pub self_supporting: bool,
}

impl Default for HollowOpts {
    fn default() -> Self {
        HollowOpts { wall: 2.0, voxel: 0.0, self_supporting: true }
    }
}

pub struct Hollowed {
    /// Dreiecke der Hohlraum-Schale (nach innen orientiert)
    pub verts: Vec<V3>,
    pub tris: Vec<[u32; 3]>,
    /// Rauminhalt des Hohlraums (mm³)
    pub cavity_volume: f64,
}

fn point_tri_dist2(p: V3, a: V3, b: V3, c: V3) -> f64 {
    // Ericson, Real-Time Collision Detection
    let ab = b - a;
    let ac = c - a;
    let ap = p - a;
    let d1 = ab.dot(ap);
    let d2 = ac.dot(ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return ap.norm2();
    }
    let bp = p - b;
    let d3 = ab.dot(bp);
    let d4 = ac.dot(bp);
    if d3 >= 0.0 && d4 <= d3 {
        return bp.norm2();
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return (p - (a + ab * v)).norm2();
    }
    let cp = p - c;
    let d5 = ab.dot(cp);
    let d6 = ac.dot(cp);
    if d6 >= 0.0 && d5 <= d6 {
        return cp.norm2();
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return (p - (a + ac * w)).norm2();
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return (p - (b + (c - b) * w)).norm2();
    }
    let denom = 1.0 / (va + vb + vc);
    let v = vb * denom;
    let w = vc * denom;
    (p - (a + ab * v + ac * w)).norm2()
}

/// Hohlraum so beschneiden, dass seine Decke nirgends flacher als 45° überhängt.
///
/// Von oben nach unten: der Hohlraum einer Schicht darf höchstens eine Zelle über den
/// Hohlraum der Schicht darüber hinausragen (dann trägt jede Deckenzelle auf ihrer
/// Nachbarzelle). Neue Hohlräume beginnen oben nur als Grat (Mittellinie der Fläche) –
/// darüber spannt die Decke höchstens eine Zelle. So entsteht ein Satteldach ^.
fn self_supporting_roof(field: &mut [f32], nx: usize, ny: usize, nz: usize, h: f32) {
    let layer = nx * ny;
    let mut above = vec![false; layer];
    let mut dist = vec![0u32; layer];
    for k in (0..nz).rev() {
        let base = k * layer;
        let cav: Vec<bool> = (0..layer).map(|i| field[base + i] > 0.0).collect();
        if !cav.iter().any(|&c| c) {
            above.iter_mut().for_each(|a| *a = false);
            continue;
        }
        // Schachbrett-Abstand zum Rand des Hohlraums in dieser Schicht (zwei Durchläufe)
        for i in 0..layer {
            dist[i] = if cav[i] { u32::MAX / 2 } else { 0 };
        }
        for j in 0..ny {
            for i in 0..nx {
                let id = j * nx + i;
                if dist[id] == 0 {
                    continue;
                }
                let mut d = dist[id];
                if i == 0 || j == 0 || i + 1 == nx {
                    d = 1;
                } else {
                    d = d.min(dist[id - 1] + 1).min(dist[id - nx] + 1).min(dist[id - nx - 1] + 1).min(dist[id - nx + 1] + 1);
                }
                dist[id] = d;
            }
        }
        for j in (0..ny).rev() {
            for i in (0..nx).rev() {
                let id = j * nx + i;
                if dist[id] == 0 {
                    continue;
                }
                let mut d = dist[id];
                if i + 1 == nx || j + 1 == ny || i == 0 {
                    d = 1;
                } else {
                    d = d.min(dist[id + 1] + 1).min(dist[id + nx] + 1).min(dist[id + nx + 1] + 1).min(dist[id + nx - 1] + 1);
                }
                dist[id] = d;
            }
        }
        let mut allowed = vec![false; layer];
        for j in 1..ny.saturating_sub(1) {
            for i in 1..nx.saturating_sub(1) {
                let id = j * nx + i;
                if !cav[id] {
                    continue;
                }
                // unter Hohlraum der Schicht darüber (± eine Zelle)?
                let mut ok = false;
                'n: for dj in [-1i64, 0, 1] {
                    for di in [-1i64, 0, 1] {
                        let q = ((j as i64 + dj) as usize) * nx + (i as i64 + di) as usize;
                        if above[q] {
                            ok = true;
                            break 'n;
                        }
                    }
                }
                // Grat: lokales Maximum des Randabstands (Mittellinie)
                if !ok {
                    let d = dist[id];
                    ok = d >= 1 && [id - 1, id + 1, id - nx, id + nx, id - nx - 1, id - nx + 1, id + nx - 1, id + nx + 1].iter().all(|&q| dist[q] <= d);
                }
                allowed[id] = ok;
            }
        }
        for i in 0..layer {
            if cav[i] && !allowed[i] {
                // knapp außerhalb des Hohlraums: Fläche verläuft zwischen den Zellen
                field[base + i] = -0.25 * h;
            }
        }
        above = allowed;
    }
}

/// Hohlraum-Schale berechnen. Das Netz muss geschlossen sein.
pub fn hollow(verts: &[V3], tris: &[[u32; 3]], o: &HollowOpts) -> Result<Hollowed, String> {
    if tris.is_empty() {
        return Err("Leeres Netz".into());
    }
    let wall = o.wall.max(0.1);
    let mut mn = v3(f64::MAX, f64::MAX, f64::MAX);
    let mut mx = -mn;
    for p in verts {
        mn = mn.min(*p);
        mx = mx.max(*p);
    }
    let size = mx - mn;
    let maxdim = size.x.max(size.y).max(size.z);
    if maxdim < wall * 2.5 {
        return Err("Das Modell ist für diese Wandstärke zu klein.".into());
    }
    let mut h = if o.voxel > 0.0 { o.voxel } else { (wall / 3.0).max(maxdim / 220.0) };
    // Speicher begrenzen (~ 12 Mio. Rasterpunkte)
    loop {
        let n = ((size.x / h) as usize + 5) * ((size.y / h) as usize + 5) * ((size.z / h) as usize + 5);
        if n <= 12_000_000 {
            break;
        }
        h *= 1.15;
    }
    let org = mn - v3(2.0 * h, 2.0 * h, 2.0 * h);
    let nx = (size.x / h) as usize + 5;
    let ny = (size.y / h) as usize + 5;
    let nz = (size.z / h) as usize + 5;
    let idx = |i: usize, j: usize, k: usize| (k * ny + j) * nx + i;
    let pos = |i: usize, j: usize, k: usize| v3(org.x + i as f64 * h, org.y + j as f64 * h, org.z + k as f64 * h);

    // 1. innen/außen: Strahl in +z je Spalte (leicht versetzt gegen Kantentreffer)
    let jx = h * 1.234e-4;
    let jy = h * 2.345e-4;
    let mut cols: Vec<Vec<f64>> = vec![Vec::new(); nx * ny];
    for t in tris {
        let (a, b, c) = (verts[t[0] as usize], verts[t[1] as usize], verts[t[2] as usize]);
        let (x0, x1) = (a.x.min(b.x).min(c.x), a.x.max(b.x).max(c.x));
        let (y0, y1) = (a.y.min(b.y).min(c.y), a.y.max(b.y).max(c.y));
        let i0 = (((x0 - org.x - jx) / h).ceil().max(0.0)) as usize;
        let i1 = (((x1 - org.x - jx) / h).floor() as isize).min(nx as isize - 1);
        let j0 = (((y0 - org.y - jy) / h).ceil().max(0.0)) as usize;
        let j1 = (((y1 - org.y - jy) / h).floor() as isize).min(ny as isize - 1);
        if i1 < i0 as isize || j1 < j0 as isize {
            continue;
        }
        let den = (b.y - c.y) * (a.x - c.x) + (c.x - b.x) * (a.y - c.y);
        if den.abs() < 1e-300 {
            continue;
        }
        for j in j0..=j1 as usize {
            for i in i0..=i1 as usize {
                let x = org.x + i as f64 * h + jx;
                let y = org.y + j as f64 * h + jy;
                let l1 = ((b.y - c.y) * (x - c.x) + (c.x - b.x) * (y - c.y)) / den;
                let l2 = ((c.y - a.y) * (x - c.x) + (a.x - c.x) * (y - c.y)) / den;
                let l3 = 1.0 - l1 - l2;
                if l1 >= 0.0 && l2 >= 0.0 && l3 >= 0.0 {
                    cols[j * nx + i].push(l1 * a.z + l2 * b.z + l3 * c.z);
                }
            }
        }
    }
    let mut inside = vec![false; nx * ny * nz];
    for j in 0..ny {
        for i in 0..nx {
            let zs = &mut cols[j * nx + i];
            if zs.len() < 2 {
                continue;
            }
            zs.sort_by(|a, b| a.partial_cmp(b).unwrap());
            let mut it = 0;
            let mut par = false;
            for k in 0..nz {
                let z = org.z + k as f64 * h;
                while it < zs.len() && zs[it] < z {
                    par = !par;
                    it += 1;
                }
                inside[idx(i, j, k)] = par;
            }
        }
    }

    // 2. Abstand zur Oberfläche im Band bis wall + 2h
    let band = wall + 2.0 * h;
    let mut dist = vec![f32::INFINITY; nx * ny * nz];
    for t in tris {
        let (a, b, c) = (verts[t[0] as usize], verts[t[1] as usize], verts[t[2] as usize]);
        let lo = a.min(b).min(c) - v3(band, band, band);
        let hi = a.max(b).max(c) + v3(band, band, band);
        let r = |v: f64, o: f64, n: usize| (((v - o) / h).floor().max(0.0) as usize).min(n - 1);
        let (i0, i1) = (r(lo.x, org.x, nx), r(hi.x, org.x, nx) + 1);
        let (j0, j1) = (r(lo.y, org.y, ny), r(hi.y, org.y, ny) + 1);
        let (k0, k1) = (r(lo.z, org.z, nz), r(hi.z, org.z, nz) + 1);
        for k in k0..k1.min(nz) {
            for j in j0..j1.min(ny) {
                for i in i0..i1.min(nx) {
                    let id = idx(i, j, k);
                    if !inside[id] {
                        continue;
                    }
                    let d2 = point_tri_dist2(pos(i, j, k), a, b, c);
                    if d2 < band * band {
                        let d = d2.sqrt() as f32;
                        if d < dist[id] {
                            dist[id] = d;
                        }
                    }
                }
            }
        }
    }
    // Feld: > 0 im Hohlraum
    let w32 = wall as f32;
    let mut field: Vec<f32> = (0..nx * ny * nz)
        .map(|id| if inside[id] { if dist[id].is_finite() { dist[id] - w32 } else { band as f32 } } else { -1.0 })
        .collect();
    drop(dist);
    drop(inside);
    if o.self_supporting {
        self_supporting_roof(&mut field, nx, ny, nz, h as f32);
    }
    // Dünne Stege des Hohlraums entfernen (Öffnen: erst schrumpfen, dann wachsen),
    // sonst berühren sich Teile nur über Kanten und die Schale wird nicht-mannigfaltig
    {
        let n = nx * ny * nz;
        let cav: Vec<bool> = field.iter().map(|&f| f > 0.0).collect();
        let nb = |id: usize| -> [usize; 6] { [id - 1, id + 1, id - nx, id + nx, id - nx * ny, id + nx * ny] };
        let interior = |id: usize| {
            let i = id % nx;
            let j = (id / nx) % ny;
            let k = id / (nx * ny);
            i > 0 && j > 0 && k > 0 && i + 1 < nx && j + 1 < ny && k + 1 < nz
        };
        let mut er = vec![false; n];
        for id in 0..n {
            if cav[id] && interior(id) && nb(id).iter().all(|&x| cav[x]) {
                er[id] = true;
            }
        }
        for id in 0..n {
            if cav[id] {
                let keep = er[id] || (interior(id) && nb(id).iter().any(|&x| er[x]));
                if !keep {
                    field[id] = -1e-3;
                }
            }
        }
    }
    if !field.iter().any(|&f| f > 0.0) {
        return Err("Zu dünn zum Aushöhlen – kleinere Wandstärke wählen.".into());
    }

    // 3. Surface Nets: ein Punkt je Zelle mit Vorzeichenwechsel
    let cidx = |i: usize, j: usize, k: usize| (k * (ny - 1) + j) * (nx - 1) + i;
    let mut cell_v = vec![u32::MAX; (nx - 1) * (ny - 1) * (nz - 1)];
    let mut out_v: Vec<V3> = Vec::new();
    let corners = [(0, 0, 0), (1, 0, 0), (0, 1, 0), (1, 1, 0), (0, 0, 1), (1, 0, 1), (0, 1, 1), (1, 1, 1)];
    let edges = [(0, 1), (2, 3), (4, 5), (6, 7), (0, 2), (1, 3), (4, 6), (5, 7), (0, 4), (1, 5), (2, 6), (3, 7)];
    for k in 0..nz - 1 {
        for j in 0..ny - 1 {
            for i in 0..nx - 1 {
                let f: Vec<f32> = corners.iter().map(|&(a, b, c)| field[idx(i + a, j + b, k + c)]).collect();
                let pos_n = f.iter().filter(|&&x| x > 0.0).count();
                if pos_n == 0 || pos_n == 8 {
                    continue;
                }
                let mut s = V3::ZERO;
                let mut n = 0.0;
                for &(e0, e1) in &edges {
                    let (f0, f1) = (f[e0], f[e1]);
                    if (f0 > 0.0) != (f1 > 0.0) {
                        let t = (f0 / (f0 - f1)) as f64;
                        let (a, b, c) = corners[e0];
                        let (d, e, g) = corners[e1];
                        let p0 = v3(a as f64, b as f64, c as f64);
                        let p1 = v3(d as f64, e as f64, g as f64);
                        s += p0 + (p1 - p0) * t;
                        n += 1.0;
                    }
                }
                let local = s / n;
                cell_v[cidx(i, j, k)] = out_v.len() as u32;
                out_v.push(org + v3((i as f64 + local.x) * h, (j as f64 + local.y) * h, (k as f64 + local.z) * h));
            }
        }
    }
    // Vierecke je Rasterkante mit Vorzeichenwechsel; Normale zeigt in den Hohlraum
    let mut out_t: Vec<[u32; 3]> = Vec::new();
    let quad = |q: [u32; 4], dir: V3, out_v: &Vec<V3>, out_t: &mut Vec<[u32; 3]>| {
        if q.iter().any(|&x| x == u32::MAX) {
            return;
        }
        let p: Vec<V3> = q.iter().map(|&x| out_v[x as usize]).collect();
        let n = (p[1] - p[0]).cross(p[2] - p[0]) + (p[2] - p[0]).cross(p[3] - p[0]);
        let q = if n.dot(dir) >= 0.0 { q } else { [q[0], q[3], q[2], q[1]] };
        // kürzere Diagonale teilen
        if out_v[q[0] as usize].dist(out_v[q[2] as usize]) <= out_v[q[1] as usize].dist(out_v[q[3] as usize]) {
            out_t.push([q[0], q[1], q[2]]);
            out_t.push([q[0], q[2], q[3]]);
        } else {
            out_t.push([q[0], q[1], q[3]]);
            out_t.push([q[1], q[2], q[3]]);
        }
    };
    for k in 1..nz - 1 {
        for j in 1..ny - 1 {
            for i in 1..nx - 1 {
                let f0 = field[idx(i, j, k)];
                // Kante in x
                if i + 1 < nx {
                    let f1 = field[idx(i + 1, j, k)];
                    if (f0 > 0.0) != (f1 > 0.0) {
                        let dir = if f0 > 0.0 { -V3::X } else { V3::X };
                        quad([cidx(i, j - 1, k - 1), cidx(i, j, k - 1), cidx(i, j, k), cidx(i, j - 1, k)].map(|c| cell_v[c]), dir, &out_v, &mut out_t);
                    }
                }
                if j + 1 < ny {
                    let f1 = field[idx(i, j + 1, k)];
                    if (f0 > 0.0) != (f1 > 0.0) {
                        let dir = if f0 > 0.0 { -V3::Y } else { V3::Y };
                        quad([cidx(i - 1, j, k - 1), cidx(i, j, k - 1), cidx(i, j, k), cidx(i - 1, j, k)].map(|c| cell_v[c]), dir, &out_v, &mut out_t);
                    }
                }
                if k + 1 < nz {
                    let f1 = field[idx(i, j, k + 1)];
                    if (f0 > 0.0) != (f1 > 0.0) {
                        let dir = if f0 > 0.0 { -V3::Z } else { V3::Z };
                        quad([cidx(i - 1, j - 1, k), cidx(i, j - 1, k), cidx(i, j, k), cidx(i - 1, j, k)].map(|c| cell_v[c]), dir, &out_v, &mut out_t);
                    }
                }
            }
        }
    }
    if out_t.is_empty() {
        return Err("Zu dünn zum Aushöhlen – kleinere Wandstärke wählen.".into());
    }
    // verbliebene Berührungsstellen auftrennen (Schale vorübergehend nach außen gedreht)
    let raw = crate::io::RawMesh { name: "Hohlraum".into(), verts: out_v, tris: out_t.iter().map(|t| [t[0], t[2], t[1]]).collect(), ..Default::default() };
    let ro = crate::repair::RepairOpts { remove_small: false, remove_inner: false, ..Default::default() };
    let fixed = crate::repair::repair(&raw, &ro);
    let out_t: Vec<[u32; 3]> = fixed.tris.iter().map(|t| [t[0], t[2], t[1]]).collect();
    // Rasterdreiecke zusammenfassen (Abweichung deutlich unter der Rasterweite)
    let (out_v, out_t) = crate::simplify::simplify(&fixed.verts, &out_t, &crate::simplify::SimplifyOpts { ratio: 0.02, tol: h * 0.2 });
    // Hohlraum-Schale: Normale zeigt in den Hohlraum → negatives Volumen
    let vol: f64 = out_t.iter().map(|t| out_v[t[0] as usize].dot(out_v[t[1] as usize].cross(out_v[t[2] as usize])) / 6.0).sum();
    Ok(Hollowed { verts: out_v, tris: out_t, cavity_volume: -vol })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cube(s: f64) -> (Vec<V3>, Vec<[u32; 3]>) {
        let mut v = Vec::new();
        for k in 0..8 {
            v.push(v3(s * (k & 1) as f64, s * ((k >> 1) & 1) as f64, s * ((k >> 2) & 1) as f64));
        }
        let t = vec![[0, 2, 1], [1, 2, 3], [4, 5, 6], [5, 7, 6], [0, 1, 4], [1, 5, 4], [2, 6, 3], [3, 6, 7], [0, 4, 2], [2, 4, 6], [1, 3, 5], [3, 7, 5]];
        (v, t)
    }

    #[test]
    fn hollow_cube() {
        let (v, t) = cube(40.0);
        let h = hollow(&v, &t, &HollowOpts { wall: 3.0, voxel: 0.5, self_supporting: false }).unwrap();
        // Hohlraum etwa (40-6)^3, Ecken leicht gerundet
        let ideal = 34f64.powi(3);
        assert!((h.cavity_volume - ideal).abs() / ideal < 0.05, "{} vs {}", h.cavity_volume, ideal);
        // geschlossen und mannigfaltig
        let a = crate::repair::analyze_indexed(h.verts.clone(), h.tris.clone(), false);
        let is = &a.issues;
        assert!(is.open_edges == 0 && is.nonmanifold_edges == 0 && is.nonmanifold_verts == 0 && is.flipped == 0, "{is:?}");
        // allein betrachtet zeigt die Schale nach innen (wie ein Hohlraum)
        assert_eq!(is.inverted_shells, 1);
        // alle Punkte mindestens ~wall von der Außenhaut entfernt
        for p in &h.verts {
            let d = [p.x, p.y, p.z, 40.0 - p.x, 40.0 - p.y, 40.0 - p.z].iter().fold(f64::MAX, |m, x| m.min(*x));
            assert!(d > 3.0 - 0.3 && d < 3.0 + 1.0, "Wand {d}");
        }
        // zusammen mit dem Würfel: ein Körper mit Hohlraum
        let mut av = v.clone();
        let mut at = t.clone();
        let base = av.len() as u32;
        av.extend(h.verts.iter().copied());
        at.extend(h.tris.iter().map(|x| [x[0] + base, x[1] + base, x[2] + base]));
        let a2 = crate::repair::analyze_indexed(av, at, false);
        assert!(a2.issues.watertight());
        assert_eq!(a2.issues.voids, 1, "{:?}", a2.issues);
        assert!((a2.issues.volume - (64000.0 - h.cavity_volume)).abs() < 1.0, "{}", a2.issues.volume);
    }

    #[test]
    fn roof_is_self_supporting() {
        let (v, t) = cube(40.0);
        let h = hollow(&v, &t, &HollowOpts { wall: 3.0, voxel: 0.5, self_supporting: true }).unwrap();
        let a = crate::repair::analyze_indexed(h.verts.clone(), h.tris.clone(), false);
        assert!(a.issues.open_edges == 0 && a.issues.nonmanifold_edges == 0, "{:?}", a.issues);
        // Satteldach: Hohlraum kleiner als ohne, aber deutlich mehr als die Hälfte
        let full = 34f64.powi(3);
        assert!(h.cavity_volume < full * 0.95 && h.cavity_volume > full * 0.6, "{}", h.cavity_volume);
        // Deckenflächen (Normale der nach innen zeigenden Schale zeigt nach unten) nicht flacher als ~45°
        let (mut flat, mut total) = (0.0, 0.0);
        for x in &h.tris {
            let (p0, p1, p2) = (h.verts[x[0] as usize], h.verts[x[1] as usize], h.verts[x[2] as usize]);
            let n = (p1 - p0).cross(p2 - p0);
            let ar = n.norm() * 0.5;
            total += ar;
            let nz = n.z / n.norm().max(1e-12);
            // Hohlraum-Schale zeigt in den Hohlraum: Decke hat Normale nach unten
            if nz < -0.85 {
                flat += ar;
            }
        }
        assert!(flat / total < 0.03, "flache Decke: {:.3}", flat / total);
    }

    #[test]
    fn too_thin() {
        let (v, t) = cube(4.0);
        assert!(hollow(&v, &t, &HollowOpts { wall: 3.0, voxel: 0.0, self_supporting: false }).is_err());
    }
}
