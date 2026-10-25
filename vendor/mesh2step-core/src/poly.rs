//! Triangulierung ebener Polygone mit Löchern (Ohrenschneiden mit Brücken zu den Löchern).

pub type P2 = [f64; 2];

fn cross(o: P2, a: P2, b: P2) -> f64 {
    (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
}

/// Vorzeichenbehaftete Fläche (positiv = gegen den Uhrzeigersinn).
pub fn signed_area(r: &[P2]) -> f64 {
    let n = r.len();
    let mut s = 0.0;
    for i in 0..n {
        let (a, b) = (r[i], r[(i + 1) % n]);
        s += a[0] * b[1] - b[0] * a[1];
    }
    s * 0.5
}

/// Punkt-in-Polygon (gerade/ungerade).
pub fn point_in(r: &[P2], p: P2) -> bool {
    let n = r.len();
    let mut inside = false;
    let mut j = n.wrapping_sub(1);
    for i in 0..n {
        let (a, b) = (r[i], r[j]);
        if (a[1] > p[1]) != (b[1] > p[1]) {
            let x = (b[0] - a[0]) * (p[1] - a[1]) / (b[1] - a[1]) + a[0];
            if p[0] < x {
                inside = !inside;
            }
        }
        j = i;
    }
    inside
}

/// Abstand Punkt–Strecke.
pub fn seg_dist(p: P2, a: P2, b: P2) -> f64 {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    let l2 = dx * dx + dy * dy;
    let t = if l2 > 0.0 { (((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l2).clamp(0.0, 1.0) } else { 0.0 };
    let (x, y) = (a[0] + t * dx - p[0], a[1] + t * dy - p[1]);
    (x * x + y * y).sqrt()
}

fn in_tri(a: P2, b: P2, c: P2, p: P2) -> bool {
    cross(a, b, p) >= 0.0 && cross(b, c, p) >= 0.0 && cross(c, a, p) >= 0.0
}

/// Trianguliert einen Außenring (gegen den Uhrzeigersinn) mit Löchern (im Uhrzeigersinn).
/// Die Indizes beziehen sich auf die Verkettung aller Ringe (Außenring, dann Löcher).
/// Die Dreiecke sind gegen den Uhrzeigersinn orientiert.
pub fn triangulate(rings: &[Vec<P2>]) -> Vec<[usize; 3]> {
    if rings.is_empty() || rings[0].len() < 3 {
        return Vec::new();
    }
    let mut pts: Vec<P2> = Vec::new();
    let mut ring_idx: Vec<Vec<usize>> = Vec::new();
    for r in rings {
        let start = pts.len();
        pts.extend_from_slice(r);
        ring_idx.push((start..pts.len()).collect());
    }
    let mut outer = ring_idx[0].clone();
    // Löcher nach größtem x sortiert einbinden
    let mut holes: Vec<Vec<usize>> = ring_idx[1..].iter().filter(|h| h.len() >= 3).cloned().collect();
    holes.sort_by(|a, b| {
        let ma = a.iter().map(|&i| pts[i][0]).fold(f64::MIN, f64::max);
        let mb = b.iter().map(|&i| pts[i][0]).fold(f64::MIN, f64::max);
        mb.partial_cmp(&ma).unwrap_or(std::cmp::Ordering::Equal)
    });
    for h in holes {
        bridge(&pts, &mut outer, &h);
    }
    ear_clip(&pts, outer)
}

/// Verbindet ein Loch über eine Brücke mit dem Außenring (Eberly).
fn bridge(pts: &[P2], outer: &mut Vec<usize>, hole: &[usize]) {
    // rechtester Punkt des Lochs
    let (hi, &m) = hole
        .iter()
        .enumerate()
        .max_by(|a, b| pts[*a.1][0].partial_cmp(&pts[*b.1][0]).unwrap_or(std::cmp::Ordering::Equal))
        .unwrap();
    let mp = pts[m];
    // Strahl nach +x: nächste Kante des Außenrings
    let n = outer.len();
    let mut best_x = f64::MAX;
    let mut best: Option<usize> = None; // Position im Außenring
    for i in 0..n {
        let (a, b) = (pts[outer[i]], pts[outer[(i + 1) % n]]);
        if (a[1] - mp[1]) * (b[1] - mp[1]) > 0.0 || a[1] == b[1] {
            if a[1] == mp[1] && a[0] >= mp[0] && a[0] < best_x {
                best_x = a[0];
                best = Some(i);
            }
            continue;
        }
        let t = (mp[1] - a[1]) / (b[1] - a[1]);
        let x = a[0] + t * (b[0] - a[0]);
        if x >= mp[0] && x < best_x {
            best_x = x;
            // Endpunkt mit größerem x
            best = Some(if a[0] > b[0] { i } else { (i + 1) % n });
        }
    }
    let mut pi = match best {
        Some(p) => p,
        None => {
            // kein Treffer (sollte nicht vorkommen): nächster Punkt
            (0..n)
                .min_by(|&a, &b| {
                    let da = (pts[outer[a]][0] - mp[0]).powi(2) + (pts[outer[a]][1] - mp[1]).powi(2);
                    let db = (pts[outer[b]][0] - mp[0]).powi(2) + (pts[outer[b]][1] - mp[1]).powi(2);
                    da.partial_cmp(&db).unwrap_or(std::cmp::Ordering::Equal)
                })
                .unwrap_or(0)
        }
    };
    // Reflexe Punkte im Dreieck (M, I, P) prüfen: den mit kleinstem Winkel nehmen
    let p = pts[outer[pi]];
    let ip = [best_x, mp[1]];
    let (ta, tb, tc) = if p[1] < mp[1] { (mp, p, ip) } else { (mp, ip, p) };
    let mut best_ang = f64::MAX;
    let mut best_d = f64::MAX;
    for j in 0..n {
        let q = pts[outer[j]];
        if j == pi || q == p {
            continue;
        }
        let prev = pts[outer[(j + n - 1) % n]];
        let next = pts[outer[(j + 1) % n]];
        let reflex = cross(prev, q, next) <= 0.0;
        if reflex && in_tri(ta, tb, tc, q) && q[0] >= mp[0] {
            let ang = ((q[1] - mp[1]).abs()).atan2(q[0] - mp[0]);
            let d = (q[0] - mp[0]).powi(2) + (q[1] - mp[1]).powi(2);
            if ang < best_ang - 1e-12 || (ang <= best_ang + 1e-12 && d < best_d) {
                best_ang = ang;
                best_d = d;
                pi = j;
            }
        }
    }
    let mut out = Vec::with_capacity(outer.len() + hole.len() + 2);
    out.extend_from_slice(&outer[..=pi]);
    for k in 0..=hole.len() {
        out.push(hole[(hi + k) % hole.len()]);
    }
    out.push(outer[pi]);
    out.extend_from_slice(&outer[pi + 1..]);
    *outer = out;
}

fn ear_clip(pts: &[P2], ring: Vec<usize>) -> Vec<[usize; 3]> {
    let mut tris = Vec::with_capacity(ring.len());
    let n = ring.len();
    if n < 3 {
        return tris;
    }
    let mut prev: Vec<usize> = (0..n).map(|i| (i + n - 1) % n).collect();
    let mut next: Vec<usize> = (0..n).map(|i| (i + 1) % n).collect();
    let mut alive = n;
    let mut i = 0;
    let mut stall = 0;
    let mut relax = 0; // 0 = streng, 1 = flache Ohren erlauben, 2 = erzwingen
    let reflex_list = |prev: &Vec<usize>, next: &Vec<usize>, start: usize, alive: usize| -> Vec<usize> {
        let mut v = Vec::new();
        let mut k = start;
        for _ in 0..alive {
            if cross(pts[ring[prev[k]]], pts[ring[k]], pts[ring[next[k]]]) <= 0.0 {
                v.push(k);
            }
            k = next[k];
        }
        v
    };
    let mut reflex = reflex_list(&prev, &next, 0, alive);
    while alive > 3 {
        let (a, b, c) = (prev[i], i, next[i]);
        let (pa, pb, pc) = (pts[ring[a]], pts[ring[b]], pts[ring[c]]);
        let cr = cross(pa, pb, pc);
        let mut ear = if relax >= 2 { true } else if relax == 1 { cr >= 0.0 } else { cr > 0.0 };
        if ear && relax < 2 {
            for &r in &reflex {
                if r == a || r == b || r == c || next[prev[r]] != r {
                    continue;
                }
                let q = pts[ring[r]];
                if q == pa || q == pb || q == pc {
                    continue;
                }
                if in_tri(pa, pb, pc, q) {
                    ear = false;
                    break;
                }
            }
        }
        if ear {
            if cr != 0.0 || relax >= 1 {
                if ring[a] != ring[b] && ring[b] != ring[c] && ring[a] != ring[c] {
                    tris.push([ring[a], ring[b], ring[c]]);
                }
            }
            next[a] = c;
            prev[c] = a;
            alive -= 1;
            i = c;
            stall = 0;
            relax = 0;
            // Reflexliste aktualisieren (Nachbarn können konvex geworden sein)
            reflex.retain(|&r| r != b);
            for &k in &[a, c] {
                let cv = cross(pts[ring[prev[k]]], pts[ring[k]], pts[ring[next[k]]]) <= 0.0;
                let pos = reflex.iter().position(|&r| r == k);
                match (cv, pos) {
                    (true, None) => reflex.push(k),
                    (false, Some(p)) => {
                        reflex.swap_remove(p);
                    }
                    _ => {}
                }
            }
        } else {
            i = next[i];
            stall += 1;
            if stall > alive {
                relax += 1;
                stall = 0;
                reflex = reflex_list(&prev, &next, i, alive);
            }
        }
    }
    let (a, b, c) = (prev[i], i, next[i]);
    if ring[a] != ring[b] && ring[b] != ring[c] && ring[a] != ring[c] {
        tris.push([ring[a], ring[b], ring[c]]);
    }
    tris
}

#[cfg(test)]
mod tests {
    use super::*;

    fn area_sum(pts: &[P2], t: &[[usize; 3]]) -> f64 {
        t.iter().map(|t| cross(pts[t[0]], pts[t[1]], pts[t[2]]) * 0.5).sum()
    }

    #[test]
    fn square_with_two_holes() {
        let outer = vec![[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]];
        let h1: Vec<P2> = (0..16).map(|k| {
            let a = -(k as f64) / 16.0 * std::f64::consts::TAU;
            [3.0 + a.cos(), 3.0 + a.sin()]
        }).collect();
        let h2 = vec![[6.0, 6.0], [6.0, 8.0], [8.0, 8.0], [8.0, 6.0]];
        let rings = vec![outer, h1.clone(), h2];
        let t = triangulate(&rings);
        let all: Vec<P2> = rings.concat();
        let a = area_sum(&all, &t);
        let expect = 100.0 - signed_area(&h1).abs() - 4.0;
        assert!((a - expect).abs() < 1e-9, "{a} vs {expect}");
        assert!(t.iter().all(|t| cross(all[t[0]], all[t[1]], all[t[2]]) >= -1e-12));
    }

    #[test]
    fn concave_comb() {
        let mut r = vec![[0.0, 0.0], [20.0, 0.0], [20.0, 5.0]];
        for k in (0..10).rev() {
            let x = k as f64 * 2.0;
            r.push([x + 1.5, 5.0]);
            r.push([x + 1.0, 10.0]);
            r.push([x + 0.5, 5.0]);
        }
        r.push([0.0, 5.0]);
        let t = triangulate(&[r.clone()]);
        assert!((area_sum(&r, &t) - signed_area(&r)).abs() < 1e-9);
        assert_eq!(t.len(), r.len() - 2);
    }
}
