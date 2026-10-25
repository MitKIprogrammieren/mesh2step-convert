//! Netzaufbereitung: Verschweißen, Halbkanten, Orientierung, Komponenten.

use crate::io::RawMesh;
use crate::math::{V3};
use std::collections::HashMap;

pub const NONE: u32 = u32::MAX;

pub struct Mesh {
    pub verts: Vec<V3>,
    pub tris: Vec<[u32; 3]>,
    pub tri_n: Vec<V3>,
    pub tri_a: Vec<f64>,
    /// Zwillings-Halbkante; Halbkante h = 3*t + k läuft von tris[t][k] nach tris[t][(k+1)%3]
    pub twin: Vec<u32>,
    pub comp: Vec<u32>,
    pub comps: Vec<Component>,
    pub bbox_min: V3,
    pub bbox_max: V3,
    pub warnings: Vec<String>,
}

#[derive(Clone, Debug)]
pub struct Component {
    pub tris: Vec<u32>,
    pub closed: bool,
    pub volume: f64,
    /// Index der umschließenden Komponente, falls diese Komponente ein Hohlraum ist
    pub void_of: Option<u32>,
}

impl Mesh {
    #[inline]
    pub fn he_from(&self, h: u32) -> u32 {
        self.tris[(h / 3) as usize][(h % 3) as usize]
    }
    #[inline]
    pub fn he_to(&self, h: u32) -> u32 {
        self.tris[(h / 3) as usize][((h % 3 + 1) % 3) as usize]
    }
    #[inline]
    pub fn he_next(h: u32) -> u32 {
        h - h % 3 + (h % 3 + 1) % 3
    }
    #[inline]
    pub fn he_prev(h: u32) -> u32 {
        h - h % 3 + (h % 3 + 2) % 3
    }
    pub fn diag(&self) -> f64 {
        (self.bbox_max - self.bbox_min).norm()
    }
    pub fn tri_centroid(&self, t: usize) -> V3 {
        let [a, b, c] = self.tris[t];
        (self.verts[a as usize] + self.verts[b as usize] + self.verts[c as usize]) / 3.0
    }
}

pub fn build(raw: &RawMesh) -> Result<Mesh, String> {
    if raw.verts.is_empty() || raw.tris.is_empty() {
        return Err("Leeres Netz".into());
    }
    let mut warnings = Vec::new();
    let mut bmin = V3 { x: f64::MAX, y: f64::MAX, z: f64::MAX };
    let mut bmax = -bmin;
    for p in &raw.verts {
        if !p.is_finite() {
            return Err("Ungültige Koordinaten (NaN/Unendlich) in der Datei".into());
        }
        bmin = bmin.min(*p);
        bmax = bmax.max(*p);
    }
    let diag = (bmax - bmin).norm().max(1e-9);

    // --- Verschweißen (Rasterverfahren) ---
    let eps = diag * 1e-7;
    let cell = eps * 4.0;
    let key = |p: V3| ((p.x / cell).floor() as i64, (p.y / cell).floor() as i64, (p.z / cell).floor() as i64);
    let mut grid: HashMap<(i64, i64, i64), Vec<u32>> = HashMap::with_capacity(raw.verts.len() / 3);
    let mut verts: Vec<V3> = Vec::with_capacity(raw.verts.len() / 4);
    let mut remap = vec![0u32; raw.verts.len()];
    for (i, &p) in raw.verts.iter().enumerate() {
        let (kx, ky, kz) = key(p);
        let mut found = NONE;
        'outer: for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    if let Some(list) = grid.get(&(kx + dx, ky + dy, kz + dz)) {
                        for &j in list {
                            if verts[j as usize].dist(p) <= eps {
                                found = j;
                                break 'outer;
                            }
                        }
                    }
                }
            }
        }
        if found == NONE {
            found = verts.len() as u32;
            verts.push(p);
            grid.entry((kx, ky, kz)).or_default().push(found);
        }
        remap[i] = found;
    }

    // --- Entartete und doppelte Dreiecke entfernen ---
    let mut tris: Vec<[u32; 3]> = Vec::with_capacity(raw.tris.len());
    // Schlüssel: sortierte Ecken -> (Index in tris, Umlaufsinn)
    let mut seen: HashMap<[u32; 3], (usize, bool)> = HashMap::with_capacity(raw.tris.len());
    let mut cancel: Vec<bool> = Vec::new();
    let mut n_degen = 0;
    let mut n_dup = 0;
    let mut n_fold = 0;
    for t in &raw.tris {
        if t.iter().any(|&i| i as usize >= raw.verts.len()) {
            continue;
        }
        let a = remap[t[0] as usize];
        let b = remap[t[1] as usize];
        let c = remap[t[2] as usize];
        if a == b || b == c || a == c {
            n_degen += 1;
            continue;
        }
        let mut s = [a, b, c];
        s.sort();
        // Umlaufsinn: gerade Permutation der sortierten Ecken?
        let par = (a < b) as u8 + (b < c) as u8 + (c < a) as u8 == 2;
        if let Some(&(i, p0)) = seen.get(&s) {
            if p0 != par && !cancel[i] {
                // gleiches Dreieck in Gegenrichtung: eine flache „Falte“ – beide entfernen
                cancel[i] = true;
                n_fold += 1;
            } else {
                n_dup += 1;
            }
            continue;
        }
        seen.insert(s, (tris.len(), par));
        tris.push([a, b, c]);
        cancel.push(false);
    }
    if n_fold > 0 {
        let mut k = 0;
        tris.retain(|_| {
            let keep = !cancel[k];
            k += 1;
            keep
        });
    }
    if n_dup > 0 {
        warnings.push(format!("{n_dup} doppelte Dreiecke entfernt"));
    }
    if n_fold > 0 {
        warnings.push(format!("{n_fold} gefaltete Dreieckspaare entfernt"));
    }
    let _ = n_degen;

    let mut mesh = Mesh {
        verts,
        tris,
        tri_n: vec![],
        tri_a: vec![],
        twin: vec![],
        comp: vec![],
        comps: vec![],
        bbox_min: bmin,
        bbox_max: bmax,
        warnings,
    };
    fix_orientation(&mut mesh);
    flip_caps(&mut mesh, diag * 5e-6);
    compute_twins(&mut mesh);
    compute_normals(&mut mesh);
    compute_components(&mut mesh);
    Ok(mesh)
}

fn edge_map(m: &Mesh) -> HashMap<(u32, u32), Vec<u32>> {
    let mut em: HashMap<(u32, u32), Vec<u32>> = HashMap::with_capacity(m.tris.len() * 2);
    for (t, tri) in m.tris.iter().enumerate() {
        for k in 0..3 {
            let a = tri[k];
            let b = tri[(k + 1) % 3];
            em.entry((a.min(b), a.max(b))).or_default().push((3 * t + k) as u32);
        }
    }
    em
}

/// Dreiecke so drehen, dass benachbarte Dreiecke konsistent orientiert sind.
fn fix_orientation(m: &mut Mesh) {
    let em = edge_map(m);
    let nt = m.tris.len();
    // Nachbarschaft über manifold-Kanten
    let mut adj: Vec<Vec<(u32, bool)>> = vec![Vec::new(); nt];
    for list in em.values() {
        if list.len() == 2 {
            let (h1, h2) = (list[0], list[1]);
            let same_dir = m.he_from(h1) == m.he_from(h2);
            adj[(h1 / 3) as usize].push((h2 / 3, same_dir));
            adj[(h2 / 3) as usize].push((h1 / 3, same_dir));
        }
    }
    let mut flip = vec![false; nt];
    let mut vis = vec![false; nt];
    let mut n_flipped = 0;
    let mut stack = Vec::new();
    for s in 0..nt {
        if vis[s] {
            continue;
        }
        vis[s] = true;
        stack.push(s);
        while let Some(t) = stack.pop() {
            for &(u, same) in &adj[t] {
                let u = u as usize;
                if !vis[u] {
                    vis[u] = true;
                    // gleiche Richtung auf gemeinsamer Kante => einer muss gedreht werden
                    flip[u] = flip[t] ^ same;
                    if flip[u] {
                        n_flipped += 1;
                    }
                    stack.push(u);
                }
            }
        }
    }
    for t in 0..nt {
        if flip[t] {
            m.tris[t].swap(1, 2);
        }
    }
    if n_flipped > 0 {
        m.warnings.push(format!("{n_flipped} falsch orientierte Dreiecke gedreht"));
    }
}

/// Flache „Kappen“-Dreiecke beseitigen: Liegt eine Ecke (fast) auf der gegenüberliegenden
/// längsten Kante, hat das Dreieck keine Fläche. CAD-Exporte erzeugen solche Fächer an
/// T-Stößen (z. B. Zahnkopf ↔ Zahnflanke). Sie lassen Randschleifen ihre Ecke mehrfach
/// besuchen („Achterschleife“), worauf ganze Flächen in Einzelfacetten zerfallen.
/// Abhilfe ohne Geometrieänderung: die längste Kante mit dem Nachbardreieck umklappen.
/// Gibt die Zahl der Umklappungen zurück.
pub fn flip_caps(m: &mut Mesh, h_eps: f64) -> usize {
    // Kappe: (Index der längsten Kante k, so dass tris[k] -> tris[k+1] die lange Kante ist)
    let cap = |v: &[V3], t: &[u32; 3]| -> Option<usize> {
        let mut best = (0usize, -1.0f64);
        for k in 0..3 {
            let l = v[t[k] as usize].dist(v[t[(k + 1) % 3] as usize]);
            if l > best.1 {
                best = (k, l);
            }
        }
        let (k, l) = best;
        if l <= 0.0 {
            return None;
        }
        let (a, b, w) = (v[t[k] as usize], v[t[(k + 1) % 3] as usize], v[t[(k + 2) % 3] as usize]);
        let d = b - a;
        let s = (w - a).dot(d) / (l * l);
        let h = d.cross(w - a).norm() / l;
        if h < h_eps && s > 1e-6 && s < 1.0 - 1e-6 {
            Some(k)
        } else {
            None
        }
    };
    let mut total = 0;
    for _pass in 0..64 {
        let em = edge_map(m);
        let mut touched = vec![false; m.tris.len()];
        let mut changed = 0;
        for t in 0..m.tris.len() {
            if touched[t] {
                continue;
            }
            let tri = m.tris[t];
            let Some(k) = cap(&m.verts, &tri) else { continue };
            let (a, b, w) = (tri[k], tri[(k + 1) % 3], tri[(k + 2) % 3]);
            let Some(list) = em.get(&(a.min(b), a.max(b))) else { continue };
            if list.len() != 2 {
                continue;
            }
            let other = if list[0] / 3 == t as u32 { list[1] } else { list[0] };
            let u = (other / 3) as usize;
            if u == t || touched[u] || m.he_from(other) != b || m.he_to(other) != a {
                continue;
            }
            let q = m.tris[u][((other % 3) as usize + 2) % 3];
            // Nachbar muss selbst Fläche haben, sonst erst später (nach anderen Umklappungen)
            if q == w || cap(&m.verts, &m.tris[u]).is_some() {
                continue;
            }
            // neue Kante w–q darf es noch nicht geben (sonst nicht-mannigfaltig)
            if em.contains_key(&(w.min(q), w.max(q))) {
                continue;
            }
            m.tris[t] = [a, q, w];
            m.tris[u] = [q, b, w];
            touched[t] = true;
            touched[u] = true;
            changed += 1;
        }
        total += changed;
        if changed == 0 {
            break;
        }
    }
    total
}

fn compute_twins(m: &mut Mesh) {
    let em = edge_map(m);
    m.twin = vec![NONE; m.tris.len() * 3];
    let mut n_open = 0usize;
    let mut n_nonman = 0usize;
    for list in em.values() {
        if list.len() == 2 && m.he_from(list[0]) == m.he_to(list[1]) {
            m.twin[list[0] as usize] = list[1];
            m.twin[list[1] as usize] = list[0];
        } else if list.len() == 1 {
            n_open += 1;
        } else if list.len() > 2 {
            n_nonman += 1;
            // Paare mit entgegengesetzter Richtung bilden, soweit möglich
            let mut used = vec![false; list.len()];
            for i in 0..list.len() {
                if used[i] {
                    continue;
                }
                for j in i + 1..list.len() {
                    if !used[j] && m.he_from(list[i]) == m.he_to(list[j]) {
                        used[i] = true;
                        used[j] = true;
                        m.twin[list[i] as usize] = list[j];
                        m.twin[list[j] as usize] = list[i];
                        break;
                    }
                }
            }
        }
    }
    if n_open > 0 {
        m.warnings.push(format!("Netz ist nicht geschlossen ({n_open} offene Kanten)"));
    }
    if n_nonman > 0 {
        m.warnings.push(format!("{n_nonman} nicht-mannigfaltige Kanten"));
    }
}

fn compute_normals(m: &mut Mesh) {
    m.tri_n = Vec::with_capacity(m.tris.len());
    m.tri_a = Vec::with_capacity(m.tris.len());
    for t in &m.tris {
        let a = m.verts[t[0] as usize];
        let b = m.verts[t[1] as usize];
        let c = m.verts[t[2] as usize];
        let cr = (b - a).cross(c - a);
        let area = cr.norm() * 0.5;
        m.tri_a.push(area);
        m.tri_n.push(if area > 0.0 { cr.normalized() } else { V3::ZERO });
    }
    // Normalen entarteter Dreiecke vom Nachbarn übernehmen
    for t in 0..m.tris.len() {
        if m.tri_n[t] == V3::ZERO {
            for k in 0..3 {
                let tw = m.twin[3 * t + k];
                if tw != NONE && m.tri_n[(tw / 3) as usize] != V3::ZERO {
                    m.tri_n[t] = m.tri_n[(tw / 3) as usize];
                    break;
                }
            }
        }
    }
}

fn compute_components(m: &mut Mesh) {
    let nt = m.tris.len();
    m.comp = vec![NONE; nt];
    let mut comps = Vec::new();
    let mut stack = Vec::new();
    for s in 0..nt {
        if m.comp[s] != NONE {
            continue;
        }
        let ci = comps.len() as u32;
        let mut list = Vec::new();
        let mut closed = true;
        m.comp[s] = ci;
        stack.push(s as u32);
        while let Some(t) = stack.pop() {
            list.push(t);
            for k in 0..3 {
                let tw = m.twin[(3 * t + k) as usize];
                if tw == NONE {
                    closed = false;
                    continue;
                }
                let u = (tw / 3) as usize;
                if m.comp[u] == NONE {
                    m.comp[u] = ci;
                    stack.push(u as u32);
                }
            }
        }
        let mut vol = 0.0;
        for &t in &list {
            let [a, b, c] = m.tris[t as usize];
            vol += m.verts[a as usize].dot(m.verts[b as usize].cross(m.verts[c as usize])) / 6.0;
        }
        comps.push(Component { tris: list, closed, volume: vol, void_of: None });
    }
    // Innen/außen klären: negative Volumina sind entweder Hohlräume oder invertiert
    let n = comps.len();
    for i in 0..n {
        if !comps[i].closed || comps[i].volume >= 0.0 {
            continue;
        }
        let probe = m.verts[m.tris[comps[i].tris[0] as usize][0] as usize];
        let mut host = None;
        let mut best = f64::MAX;
        for j in 0..n {
            if j == i || !comps[j].closed || comps[j].volume.abs() <= comps[i].volume.abs() {
                continue;
            }
            if point_inside(m, &comps[j].tris, probe) && comps[j].volume.abs() < best {
                best = comps[j].volume.abs();
                host = Some(j as u32);
            }
        }
        match host {
            Some(h) => comps[i].void_of = Some(h),
            None => {
                for &t in &comps[i].tris {
                    m.tris[t as usize].swap(1, 2);
                    m.tri_n[t as usize] = -m.tri_n[t as usize];
                }
                comps[i].volume = -comps[i].volume;
            }
        }
    }
    // Twins nach dem Drehen neu berechnen (Halbkantenindizes ändern sich)
    let need = comps.iter().any(|c| c.volume < 0.0 && c.void_of.is_none());
    let _ = need;
    m.comps = comps;
    compute_twins_silent(m);
}

fn compute_twins_silent(m: &mut Mesh) {
    let w = m.warnings.clone();
    compute_twins(m);
    m.warnings = w;
}

/// Punkt-in-Netz-Test per Strahl (Paritätsprüfung, mehrere Richtungen).
fn point_inside(m: &Mesh, tris: &[u32], p: V3) -> bool {
    let dirs = [
        V3 { x: 0.5773, y: 0.5774, z: 0.5775 },
        V3 { x: -0.6011, y: 0.3123, z: 0.7357 },
        V3 { x: 0.2113, y: -0.8712, z: 0.4431 },
    ];
    let mut votes = 0;
    for d in dirs {
        let d = d.normalized();
        let mut cnt = 0;
        for &t in tris {
            let [a, b, c] = m.tris[t as usize];
            let (a, b, c) = (m.verts[a as usize], m.verts[b as usize], m.verts[c as usize]);
            let e1 = b - a;
            let e2 = c - a;
            let pv = d.cross(e2);
            let det = e1.dot(pv);
            if det.abs() < 1e-14 {
                continue;
            }
            let inv = 1.0 / det;
            let tv = p - a;
            let u = tv.dot(pv) * inv;
            if !(0.0..=1.0).contains(&u) {
                continue;
            }
            let q = tv.cross(e1);
            let v = d.dot(q) * inv;
            if v < 0.0 || u + v > 1.0 {
                continue;
            }
            if e2.dot(q) * inv > 1e-12 {
                cnt += 1;
            }
        }
        if cnt % 2 == 1 {
            votes += 1;
        }
    }
    votes >= 2
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::io::RawMesh;
    use crate::math::v3;

    /// Fächer flacher Dreiecke an einem T-Stoß (wie an Zahnköpfen aus CAD-Exporten):
    /// Danach darf kein Dreieck ohne Fläche übrig bleiben, Gesamtfläche und Dichtheit bleiben.
    #[test]
    fn kappen_faecher_wird_umgeklappt() {
        // Tetraeder A,B,C,D; die Seitenfläche über A–B ist an P1..P3 (auf A–B) unterteilt,
        // der Boden nicht. Die Lücke schließen drei flache Kappen mit gemeinsamer Ecke A.
        let mut verts = vec![v3(0.0, 0.0, 0.0), v3(4.0, 0.0, 0.0), v3(2.0, 3.0, 0.0), v3(2.0, 1.0, 3.0)];
        for i in 1..4 {
            verts.push(v3(i as f64, 0.0, 0.0));
        }
        let (a, b, c, d, p1, p2, p3) = (0u32, 1, 2, 3, 4, 5, 6);
        let tris = vec![
            [a, c, b],
            [b, c, d],
            [c, a, d],
            [a, p1, d],
            [p1, p2, d],
            [p2, p3, d],
            [p3, b, d],
            [a, b, p3],
            [a, p3, p2],
            [a, p2, p1],
        ];
        let m = build(&RawMesh { verts, tris, ..Default::default() }).unwrap();
        assert!(m.tri_a.iter().all(|&x| x > 1e-6), "flache Kappen übrig: {:?}", m.tris);
        // Oberfläche des Tetraeders bleibt erhalten
        let tet = [[a, c, b], [b, c, d], [c, a, d], [a, b, d]];
        let want: f64 = tet.iter().map(|t| (m.verts[t[1] as usize] - m.verts[t[0] as usize]).cross(m.verts[t[2] as usize] - m.verts[t[0] as usize]).norm() / 2.0).sum();
        let got: f64 = m.tri_a.iter().sum();
        assert!((got - want).abs() < 1e-9, "{want} vs {got}");
        assert!(m.warnings.iter().all(|w| !w.contains("nicht geschlossen") && !w.contains("mannigfaltig")), "{:?}", m.warnings);
    }
}
