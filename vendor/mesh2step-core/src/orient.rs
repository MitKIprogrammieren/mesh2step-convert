//! Drucklage vorschlagen: mögliche Auflageflächen finden und nach Überhang,
//! Auflagefläche und Höhe bewerten.

use crate::math::{v3, V3};

/// Eine mögliche Lage auf dem Druckbett.
#[derive(Clone, Debug)]
pub struct Placement {
    /// Richtung (im Modell), die nach unten zeigt (äußere Normale der Auflage)
    pub down: V3,
    /// Fläche, die steiler als der Grenzwinkel nach unten zeigt (mm²), ohne Auflage
    pub overhang: f64,
    /// Anteil der Überhangfläche an der Gesamtoberfläche (0..1)
    pub overhang_frac: f64,
    /// Fläche, die direkt auf dem Bett liegt (mm²)
    pub contact: f64,
    /// Höhe in dieser Lage (mm)
    pub height: f64,
    pub score: f64,
    /// Lage wie geladen
    pub original: bool,
}

/// Drehung, die `d` auf (0,0,-1) bringt (Zeilen einer 3x3-Matrix).
pub fn rotation_to_down(d: V3) -> [V3; 3] {
    let d = d.normalized();
    let target = v3(0.0, 0.0, -1.0);
    let c = d.dot(target);
    if c > 1.0 - 1e-12 {
        return [V3::X, V3::Y, V3::Z];
    }
    if c < -1.0 + 1e-12 {
        // 180° um die X-Achse
        return [V3::X, v3(0.0, -1.0, 0.0), v3(0.0, 0.0, -1.0)];
    }
    let axis = d.cross(target).normalized();
    let s = (1.0 - c * c).sqrt();
    let (x, y, z) = (axis.x, axis.y, axis.z);
    let t = 1.0 - c;
    [
        v3(t * x * x + c, t * x * y - s * z, t * x * z + s * y),
        v3(t * x * y + s * z, t * y * y + c, t * y * z - s * x),
        v3(t * x * z - s * y, t * y * z + s * x, t * z * z + c),
    ]
}

fn rot(m: &[V3; 3], p: V3) -> V3 {
    v3(m[0].dot(p), m[1].dot(p), m[2].dot(p))
}

/// Netz in die Lage drehen und auf das Bett stellen (Mitte über dem Ursprung, unten z = 0).
pub fn apply(verts: &[V3], down: V3) -> Vec<V3> {
    let m = rotation_to_down(down);
    let mut out: Vec<V3> = verts.iter().map(|&p| rot(&m, p)).collect();
    let mut mn = v3(f64::MAX, f64::MAX, f64::MAX);
    let mut mx = -mn;
    for p in &out {
        mn = mn.min(*p);
        mx = mx.max(*p);
    }
    let shift = v3(-(mn.x + mx.x) * 0.5, -(mn.y + mx.y) * 0.5, -mn.z);
    for p in out.iter_mut() {
        *p += shift;
    }
    out
}

/// Lagen bewerten. `max_angle_deg`: Überhang ab diesem Winkel zur Senkrechten (üblich 45°).
/// Rückgabe: nach Bewertung sortiert (beste zuerst); die geladene Lage ist immer dabei.
pub fn suggest(verts: &[V3], tris: &[[u32; 3]], max_angle_deg: f64) -> Vec<Placement> {
    // Flächen nach Normalenrichtung sammeln (Raster auf der Einheitskugel)
    let mut bins: std::collections::HashMap<(i32, i32, i32), (V3, f64)> = std::collections::HashMap::new();
    let mut normals = Vec::with_capacity(tris.len());
    let mut total = 0.0;
    for t in tris {
        let (a, b, c) = (verts[t[0] as usize], verts[t[1] as usize], verts[t[2] as usize]);
        let cr = (b - a).cross(c - a);
        let area = cr.norm() * 0.5;
        let n = if area > 0.0 { cr.normalized() } else { V3::Z };
        normals.push((n, area, (a + b + c) / 3.0));
        total += area;
        if area > 0.0 {
            let k = ((n.x * 12.0).round() as i32, (n.y * 12.0).round() as i32, (n.z * 12.0).round() as i32);
            let e = bins.entry(k).or_insert((V3::ZERO, 0.0));
            e.0 += n * area;
            e.1 += area;
        }
    }
    if total <= 0.0 {
        return Vec::new();
    }
    let mut cands: Vec<(V3, f64)> = bins.values().filter(|(_, a)| *a > 0.0).map(|(s, a)| (s.normalized(), *a)).collect();
    cands.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));
    cands.truncate(40);
    let mut dirs: Vec<V3> = cands.into_iter().map(|c| c.0).collect();
    for d in [V3::X, V3::Y, V3::Z, -V3::X, -V3::Y, -V3::Z] {
        dirs.push(d);
    }
    let orig = v3(0.0, 0.0, -1.0);
    dirs.push(orig);
    // doppelte Richtungen entfernen
    let mut uniq: Vec<V3> = Vec::new();
    for d in dirs {
        if !uniq.iter().any(|u| u.dot(d) > 0.999) {
            uniq.push(d);
        }
    }
    let cos_lim = (90.0 - max_angle_deg).to_radians().sin();
    let diag = {
        let mut mn = v3(f64::MAX, f64::MAX, f64::MAX);
        let mut mx = -mn;
        for p in verts {
            mn = mn.min(*p);
            mx = mx.max(*p);
        }
        (mx - mn).norm().max(1e-9)
    };
    let bed_tol = (diag * 1e-3).max(0.05);
    let mut out = Vec::new();
    for d in uniq {
        // Höhe entlang -d: z' = -d·p
        let mut zmin = f64::MAX;
        let mut zmax = f64::MIN;
        for p in verts {
            let z = -d.dot(*p);
            zmin = zmin.min(z);
            zmax = zmax.max(z);
        }
        let mut overhang = 0.0;
        let mut contact = 0.0;
        for &(n, area, c) in &normals {
            let down = n.dot(d); // 1 = zeigt genau nach unten
            if down <= cos_lim {
                continue;
            }
            let z = -d.dot(c) - zmin;
            if z < bed_tol && down > 0.999 {
                contact += area;
            } else {
                overhang += area * (down - cos_lim) / (1.0 - cos_lim);
            }
        }
        let height = zmax - zmin;
        // weniger Überhang, mehr Auflage, etwas niedriger
        let score = overhang / total * 100.0 - (contact / total).sqrt() * 20.0 + height / diag * 2.0;
        out.push(Placement { down: d, overhang, overhang_frac: overhang / total, contact, height, score, original: d.dot(orig) > 0.999 });
    }
    out.sort_by(|a, b| a.score.partial_cmp(&b.score).unwrap_or(std::cmp::Ordering::Equal));
    out
}

/// Bis zu `n` deutlich verschiedene Vorschläge (die geladene Lage immer mit dabei).
pub fn top(mut all: Vec<Placement>, n: usize) -> Vec<Placement> {
    let mut sel: Vec<Placement> = Vec::new();
    for p in all.iter() {
        if sel.len() >= n {
            break;
        }
        if !sel.iter().any(|s| s.down.dot(p.down) > 0.95) {
            sel.push(p.clone());
        }
    }
    if !sel.iter().any(|s| s.original) {
        if let Some(i) = all.iter().position(|p| p.original) {
            let o = all.swap_remove(i);
            if sel.len() >= n {
                sel.pop();
            }
            sel.push(o);
        }
    }
    sel
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tri_prism() -> (Vec<V3>, Vec<[u32; 3]>) {
        // Keil: liegt gut auf der großen Rechteckseite, schlecht auf der Spitze
        let v = vec![
            v3(0., 0., 0.),
            v3(40., 0., 0.),
            v3(0., 0., 30.),
            v3(0., 10., 0.),
            v3(40., 10., 0.),
            v3(0., 10., 30.),
        ];
        let mut t: Vec<[u32; 3]> = vec![[0, 2, 1], [3, 4, 5], [0, 1, 4], [0, 4, 3], [0, 3, 5], [0, 5, 2], [1, 2, 5], [1, 5, 4]];
        // nach außen orientieren
        let vol: f64 = t.iter().map(|x| v[x[0] as usize].dot(v[x[1] as usize].cross(v[x[2] as usize]))).sum();
        if vol < 0.0 {
            for x in t.iter_mut() {
                x.swap(1, 2);
            }
        }
        (v, t)
    }

    #[test]
    fn rotation_maps_down() {
        for d in [V3::X, -V3::X, V3::Y, V3::Z, -V3::Z, v3(1., 2., 3.).normalized()] {
            let m = rotation_to_down(d);
            let r = rot(&m, d);
            assert!((r - v3(0., 0., -1.)).norm() < 1e-9, "{d:?} -> {r:?}");
        }
    }

    #[test]
    fn wedge_lies_flat() {
        let (v, t) = tri_prism();
        // so drehen, dass der Keil schlecht steht (Spitze unten)
        let tilted: Vec<V3> = v.iter().map(|p| v3(p.x, p.z, -p.y)).collect();
        let all = suggest(&tilted, &t, 45.0);
        let best = &all[0];
        assert!(best.overhang_frac < 0.01, "{best:?}");
        assert!(best.contact > 300.0, "{best:?}");
        let placed = apply(&tilted, best.down);
        let minz = placed.iter().fold(f64::MAX, |m, p| m.min(p.z));
        assert!(minz.abs() < 1e-9);
        let t3 = top(all, 3);
        assert!(t3.iter().any(|p| p.original));
    }
}
