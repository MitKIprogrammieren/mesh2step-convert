//! Aufteilen von Modellen: nach Farbbereichen (Bemalung) oder mit einer Schnittebene,
//! jeweils mit geschlossenen Teilen, optional mit Verbindern (Zapfen/Dübel).

use crate::math::{v3, V3};
use crate::paint::ColoredMesh;
use crate::poly::{self, P2};
use std::collections::HashMap;

/// Ein fertiges, geschlossenes Teil.
#[derive(Clone, Debug, Default)]
pub struct Part {
    pub name: String,
    pub verts: Vec<V3>,
    pub tris: Vec<[u32; 3]>,
    /// Farbe je Dreieck (Filament, 1-basiert)
    pub col: Vec<u8>,
    /// Hauptfarbe des Teils
    pub color: u8,
    /// Index des Ausgangsobjekts
    pub group: usize,
    pub kind: PartKind,
    /// offene Kanten (0 = geschlossen)
    pub open_edges: usize,
    pub volume: f64,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum PartKind {
    #[default]
    Body,
    ColorPart,
    CutPart,
    Dowel,
}

impl Part {
    fn from_soup(name: String, verts: &[V3], tris: &[[u32; 3]], col: &[u8], color: u8, group: usize, kind: PartKind) -> Part {
        let mut map: HashMap<u32, u32> = HashMap::new();
        let mut p = Part { name, color, group, kind, ..Default::default() };
        for (t, c) in tris.iter().zip(col) {
            let mut nt = [0u32; 3];
            for k in 0..3 {
                nt[k] = *map.entry(t[k]).or_insert_with(|| {
                    p.verts.push(verts[t[k] as usize]);
                    (p.verts.len() - 1) as u32
                });
            }
            if nt[0] != nt[1] && nt[1] != nt[2] && nt[0] != nt[2] {
                p.tris.push(nt);
                p.col.push(*c);
            }
        }
        p.update_stats();
        p
    }

    pub fn update_stats(&mut self) {
        // je ungerichteter Kante: (vorwärts, rückwärts) – genau (1, 1) ist mannigfaltig
        let mut em: HashMap<(u32, u32), (u32, u32)> = HashMap::with_capacity(self.tris.len() * 2);
        let mut vol = 0.0;
        for t in &self.tris {
            for k in 0..3 {
                let (a, b) = (t[k], t[(k + 1) % 3]);
                let e = em.entry((a.min(b), a.max(b))).or_default();
                if a < b {
                    e.0 += 1;
                } else {
                    e.1 += 1;
                }
            }
            let (a, b, c) = (self.verts[t[0] as usize], self.verts[t[1] as usize], self.verts[t[2] as usize]);
            vol += a.dot(b.cross(c)) / 6.0;
        }
        self.open_edges = em.values().filter(|&&v| v != (1, 1)).count();
        self.volume = vol;
    }

    pub fn bbox(&self) -> (V3, V3) {
        let mut mn = v3(f64::MAX, f64::MAX, f64::MAX);
        let mut mx = v3(f64::MIN, f64::MIN, f64::MIN);
        for p in &self.verts {
            mn = mn.min(*p);
            mx = mx.max(*p);
        }
        (mn, mx)
    }
}

fn tri_normal_area(v: &[V3], t: &[u32; 3]) -> V3 {
    (v[t[1] as usize] - v[t[0] as usize]).cross(v[t[2] as usize] - v[t[0] as usize]) * 0.5
}

/// Dreiecke einheitlich und nach außen orientieren (je zusammenhängendem Teil).
pub fn orient(m: &mut ColoredMesh) -> usize {
    let n = m.tris.len();
    let mut em: HashMap<(u32, u32), Vec<u32>> = HashMap::with_capacity(n * 2);
    for (t, tri) in m.tris.iter().enumerate() {
        for k in 0..3 {
            let (a, b) = (tri[k], tri[(k + 1) % 3]);
            em.entry((a.min(b), a.max(b))).or_default().push(t as u32);
        }
    }
    let mut flip = vec![false; n];
    let mut seen = vec![false; n];
    let mut flipped = 0;
    for s in 0..n {
        if seen[s] {
            continue;
        }
        seen[s] = true;
        let mut comp = vec![s];
        let mut stack = vec![s];
        while let Some(t) = stack.pop() {
            let tri = m.tris[t];
            for k in 0..3 {
                let (a, b) = (tri[k], tri[(k + 1) % 3]);
                let list = &em[&(a.min(b), a.max(b))];
                if list.len() != 2 {
                    continue;
                }
                let u = if list[0] as usize == t { list[1] } else { list[0] } as usize;
                if seen[u] {
                    continue;
                }
                // gleiche Richtung auf der gemeinsamen Kante => u muss andersherum sein als t
                let ut = m.tris[u];
                let same = (0..3).any(|j| ut[j] == a && ut[(j + 1) % 3] == b);
                flip[u] = flip[t] ^ same;
                seen[u] = true;
                comp.push(u);
                stack.push(u);
            }
        }
        // Volumenvorzeichen des Bauteils
        let mut vol = 0.0;
        for &t in &comp {
            let tri = m.tris[t];
            let (a, b, c) = (m.verts[tri[0] as usize], m.verts[tri[1] as usize], m.verts[tri[2] as usize]);
            let v = a.dot(b.cross(c));
            vol += if flip[t] { -v } else { v };
        }
        if vol < 0.0 {
            for &t in &comp {
                flip[t] = !flip[t];
            }
        }
    }
    for t in 0..n {
        if flip[t] {
            m.tris[t].swap(1, 2);
            flipped += 1;
        }
    }
    flipped
}

// ---------------------------------------------------------------------------
// Farbbereiche

#[derive(Clone, Debug)]
pub struct RegionInfo {
    pub color: u8,
    pub tris: Vec<u32>,
    pub area: f64,
    pub center: V3,
}

#[derive(Clone, Debug, Default)]
pub struct Regions {
    pub of: Vec<u32>,
    pub list: Vec<RegionInfo>,
    /// Farbe mit der größten Fläche (bleibt als Grundkörper)
    pub base_color: u8,
}

fn undirected_map(tris: &[[u32; 3]]) -> HashMap<(u32, u32), Vec<u32>> {
    let mut em: HashMap<(u32, u32), Vec<u32>> = HashMap::with_capacity(tris.len() * 2);
    for (t, tri) in tris.iter().enumerate() {
        for k in 0..3 {
            let (a, b) = (tri[k], tri[(k + 1) % 3]);
            em.entry((a.min(b), a.max(b))).or_default().push(t as u32);
        }
    }
    em
}

pub fn regions(m: &ColoredMesh) -> Regions {
    let n = m.tris.len();
    let em = undirected_map(&m.tris);
    let mut of = vec![u32::MAX; n];
    let mut list = Vec::new();
    for s in 0..n {
        if of[s] != u32::MAX {
            continue;
        }
        let rid = list.len() as u32;
        let col = m.col[s];
        let mut tris = vec![s as u32];
        of[s] = rid;
        let mut i = 0;
        while i < tris.len() {
            let t = tris[i] as usize;
            i += 1;
            let tri = m.tris[t];
            for k in 0..3 {
                let (a, b) = (tri[k], tri[(k + 1) % 3]);
                for &u in &em[&(a.min(b), a.max(b))] {
                    let u = u as usize;
                    if of[u] == u32::MAX && m.col[u] == col {
                        of[u] = rid;
                        tris.push(u as u32);
                    }
                }
            }
        }
        let mut area = 0.0;
        let mut c = V3::ZERO;
        for &t in &tris {
            let tri = m.tris[t as usize];
            let a = tri_normal_area(&m.verts, &tri).norm();
            area += a;
            c = c + (m.verts[tri[0] as usize] + m.verts[tri[1] as usize] + m.verts[tri[2] as usize]) * (a / 3.0);
        }
        let center = if area > 0.0 { c / area } else { m.verts[m.tris[s][0] as usize] };
        list.push(RegionInfo { color: col, tris, area, center });
    }
    let mut by_col: HashMap<u8, f64> = HashMap::new();
    for r in &list {
        *by_col.entry(r.color).or_default() += r.area;
    }
    let base_color = by_col.iter().max_by(|a, b| a.1.partial_cmp(b.1).unwrap().then(b.0.cmp(a.0))).map(|(c, _)| *c).unwrap_or(1);
    Regions { of, list, base_color }
}

/// Kleine Farbflecken (Fläche < min_area) der umgebenden Farbe zuschlagen und
/// Farbbereiche beseitigen, die sich nur in einem Punkt berühren.
pub fn clean_specks(m: &mut ColoredMesh, min_area: f64) -> usize {
    let n = merge_specks(m, min_area);
    let p = fix_pinches(m);
    if p > 0 && min_area > 0.0 {
        merge_specks(m, min_area);
    }
    n + p
}

/// Ecken, an denen dieselbe Farbe in mehreren getrennten Fächern vorkommt („Einschnürung“),
/// führen zu nicht-mannigfaltigen Teilen. Der kleinere Fächer bekommt die Nachbarfarbe.
pub fn fix_pinches(m: &mut ColoredMesh) -> usize {
    let mut changed_total = 0;
    for _ in 0..10 {
        let nv = m.verts.len();
        let mut inc: Vec<Vec<u32>> = vec![Vec::new(); nv];
        for (t, tri) in m.tris.iter().enumerate() {
            for &v in tri {
                inc[v as usize].push(t as u32);
            }
        }
        let mut changed = 0;
        for v in 0..nv {
            let list = &inc[v];
            if list.len() < 4 {
                continue;
            }
            // Fächer ordnen: Dreieck (v, a, b) -> Nachfolger beginnt mit (v, b, ..)
            let rot = |t: u32| -> (u32, u32) {
                let tri = m.tris[t as usize];
                let k = tri.iter().position(|&x| x as usize == v).unwrap();
                (tri[(k + 1) % 3], tri[(k + 2) % 3])
            };
            let mut by_a: HashMap<u32, u32> = HashMap::with_capacity(list.len());
            let mut ok = true;
            for &t in list {
                if by_a.insert(rot(t).0, t).is_some() {
                    ok = false;
                }
            }
            if !ok {
                continue;
            }
            let mut order = vec![list[0]];
            let mut cur = list[0];
            for _ in 0..list.len() {
                let b = rot(cur).1;
                match by_a.get(&b) {
                    Some(&nx) if nx != list[0] => {
                        order.push(nx);
                        cur = nx;
                    }
                    Some(_) => break,
                    None => {
                        ok = false;
                        break;
                    }
                }
            }
            if !ok || order.len() != list.len() {
                continue;
            }
            // Läufe gleicher Farbe (zyklisch)
            let k = order.len();
            let start = match (0..k).find(|&i| m.col[order[i] as usize] != m.col[order[(i + k - 1) % k] as usize]) {
                Some(s) => s,
                None => continue, // eine Farbe
            };
            let mut runs: Vec<(u8, Vec<u32>)> = Vec::new();
            for j in 0..k {
                let t = order[(start + j) % k];
                let c = m.col[t as usize];
                match runs.last_mut() {
                    Some((rc, ts)) if *rc == c => ts.push(t),
                    _ => runs.push((c, vec![t])),
                }
            }
            let mut cnt: HashMap<u8, usize> = HashMap::new();
            for (c, _) in &runs {
                *cnt.entry(*c).or_default() += 1;
            }
            // kleinsten Lauf einer mehrfach vorkommenden Farbe umfärben
            let cand = (0..runs.len())
                .filter(|&i| cnt[&runs[i].0] > 1)
                .min_by_key(|&i| runs[i].1.len());
            if let Some(i) = cand {
                let prev = &runs[(i + runs.len() - 1) % runs.len()];
                let c = prev.0;
                for &t in &runs[i].1.clone() {
                    m.col[t as usize] = c;
                }
                changed += 1;
            }
        }
        changed_total += changed;
        if changed == 0 {
            break;
        }
    }
    changed_total
}

/// Nächster Punkt auf einem Dreieck.
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
        return a + ab * (d1 / (d1 - d3));
    }
    let cp = p - c;
    let d5 = ab.dot(cp);
    let d6 = ac.dot(cp);
    if d6 >= 0.0 && d5 <= d6 {
        return c;
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        return a + ac * (d2 / (d2 - d6));
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        return b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));
    }
    let den = 1.0 / (va + vb + vc);
    a + ab * (vb * den) + ac * (vc * den)
}

/// Farbgrenzen glätten: Punkte auf einer Grenze zwischen zwei Farben wandern zur Mitte
/// ihrer Grenznachbarn und werden auf die ursprüngliche Oberfläche zurückprojiziert.
/// `strength` 0..1. Rückgabe: Anzahl bewegter Punkte.
pub fn smooth_boundaries(m: &mut ColoredMesh, strength: f64) -> usize {
    if strength <= 0.0 {
        return 0;
    }
    let nv = m.verts.len();
    let em = undirected_map(&m.tris);
    // Grenzkanten zwischen verschiedenen Farben
    let mut bnb: Vec<Vec<u32>> = vec![Vec::new(); nv];
    for (&(a, b), l) in &em {
        if l.len() == 2 && m.col[l[0] as usize] != m.col[l[1] as usize] {
            bnb[a as usize].push(b);
            bnb[b as usize].push(a);
        } else if l.len() != 2 {
            // offene/nicht-mannigfaltige Kanten: Endpunkte festhalten
            bnb[a as usize].push(u32::MAX);
            bnb[b as usize].push(u32::MAX);
        }
    }
    let mut inc: Vec<Vec<u32>> = vec![Vec::new(); nv];
    for (t, tri) in m.tris.iter().enumerate() {
        for &v in tri {
            inc[v as usize].push(t as u32);
        }
    }
    // bewegliche Punkte: genau zwei Grenznachbarn, höchstens zwei Farben ringsum
    let movable: Vec<usize> = (0..nv)
        .filter(|&v| {
            if bnb[v].len() != 2 || bnb[v].contains(&u32::MAX) {
                return false;
            }
            let mut cols: Vec<u8> = inc[v].iter().map(|&t| m.col[t as usize]).collect();
            cols.sort();
            cols.dedup();
            cols.len() == 2
        })
        .collect();
    if movable.is_empty() {
        return 0;
    }
    let orig = m.verts.clone();
    let orig_n: Vec<V3> = m.tris.iter().map(|t| tri_normal_area(&orig, t)).collect();
    let iters = (strength * 12.0).ceil() as usize;
    let alpha = 0.5;
    for _ in 0..iters {
        let mut newp: Vec<(usize, V3)> = Vec::with_capacity(movable.len());
        for &v in &movable {
            let (a, b) = (bnb[v][0] as usize, bnb[v][1] as usize);
            let p = m.verts[v];
            let target = p + ((m.verts[a] + m.verts[b]) * 0.5 - p) * alpha;
            // auf die ursprüngliche Oberfläche (Dreiecke um v) projizieren
            let mut best = p;
            let mut bd = f64::MAX;
            for &t in &inc[v] {
                let tri = m.tris[t as usize];
                let q = closest_on_tri(target, orig[tri[0] as usize], orig[tri[1] as usize], orig[tri[2] as usize]);
                let d = q.dist(target);
                if d < bd {
                    bd = d;
                    best = q;
                }
            }
            newp.push((v, best));
        }
        for (v, q) in newp {
            let old = m.verts[v];
            m.verts[v] = q;
            // keine umklappenden oder entartenden Dreiecke zulassen
            let ok = inc[v].iter().all(|&t| {
                let n = tri_normal_area(&m.verts, &m.tris[t as usize]);
                let o = orig_n[t as usize];
                n.dot(o) > 0.05 * o.norm2() && n.norm() > 0.02 * o.norm()
            });
            if !ok {
                m.verts[v] = old;
            }
        }
    }
    movable.len()
}

/// „Grenze anpassen“: Farbgrenzen scharf und glatt nachziehen.
///
/// Statt der Treppe entlang der Dreieckskanten wird je Farbe ein geglättetes Anteilsfeld an
/// den Ecken berechnet und jedes Dreieck genau an der 50-%-Linie geteilt. So läuft die
/// Farbkante als glatte Linie quer durch die Dreiecke. `strength` 0..1 steuert die Glättung.
/// Rückgabe: Zahl der geteilten Dreiecke.
pub fn sharpen_boundaries(m: &mut ColoredMesh, strength: f64) -> usize {
    if strength <= 0.0 || m.tris.is_empty() {
        return 0;
    }
    let iters = 1 + (strength * 6.0).round() as usize;
    // Farben nach Fläche, größte (Grundfarbe) zuletzt und nicht eigens bearbeitet
    let mut area: HashMap<u8, f64> = HashMap::new();
    for (t, &c) in m.tris.iter().zip(&m.col) {
        *area.entry(c).or_default() += tri_normal_area(&m.verts, t).norm();
    }
    let mut colors: Vec<(u8, f64)> = area.into_iter().collect();
    if colors.len() < 2 {
        return 0;
    }
    colors.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
    colors.pop();
    let mut split_count = 0usize;
    for (c, _) in colors {
        let nv = m.verts.len();
        let em = undirected_map(&m.tris);
        // Anteil der Farbe je Ecke (nach Fläche gewichtet)
        let mut num = vec![0.0f64; nv];
        let mut den = vec![0.0f64; nv];
        for (t, &tc) in m.tris.iter().zip(&m.col) {
            let a = tri_normal_area(&m.verts, t).norm();
            for &v in t {
                den[v as usize] += a;
                if tc == c {
                    num[v as usize] += a;
                }
            }
        }
        let mut phi: Vec<f64> = (0..nv).map(|v| if den[v] > 0.0 { num[v] / den[v] } else { 0.0 }).collect();
        // Nachbarn je Ecke (nur dort, wo sich die Farbe ändert, wird geglättet)
        let mut nb: Vec<Vec<u32>> = vec![Vec::new(); nv];
        for &(a, b) in em.keys() {
            nb[a as usize].push(b);
            nb[b as usize].push(a);
        }
        let mut active: Vec<bool> = (0..nv).map(|v| phi[v] > 0.0 && phi[v] < 1.0).collect();
        for _ in 0..iters {
            // Band um die Grenze um einen Ring erweitern
            let grow: Vec<u32> = (0..nv as u32).filter(|&v| active[v as usize]).flat_map(|v| nb[v as usize].clone()).collect();
            for v in grow {
                active[v as usize] = true;
            }
            let old = phi.clone();
            for v in 0..nv {
                if !active[v] || nb[v].is_empty() {
                    continue;
                }
                let avg: f64 = nb[v].iter().map(|&u| old[u as usize]).sum::<f64>() / nb[v].len() as f64;
                phi[v] = 0.5 * old[v] + 0.5 * avg;
            }
        }
        // „andere“ Farbe je Dreieck (für Stücke außerhalb der Linie)
        let other = |t: usize, m: &ColoredMesh| -> u8 {
            if m.col[t] != c {
                return m.col[t];
            }
            let tri = m.tris[t];
            let mut cnt: HashMap<u8, usize> = HashMap::new();
            for k in 0..3 {
                let (a, b) = (tri[k], tri[(k + 1) % 3]);
                if let Some(l) = em.get(&(a.min(b), a.max(b))) {
                    for &u in l {
                        let uc = m.col[u as usize];
                        if uc != c {
                            *cnt.entry(uc).or_default() += 1;
                        }
                    }
                }
            }
            cnt.into_iter().max_by_key(|x| (x.1, std::cmp::Reverse(x.0))).map(|x| x.0).unwrap_or(c)
        };
        let mut cache: HashMap<(u32, u32), u32> = HashMap::new();
        let mut new_tris: Vec<[u32; 3]> = Vec::with_capacity(m.tris.len() + 64);
        let mut new_col: Vec<u8> = Vec::with_capacity(m.tris.len() + 64);
        let mut verts = m.verts.clone();
        for t in 0..m.tris.len() {
            let tri = m.tris[t];
            let ins = [phi[tri[0] as usize] >= 0.5, phi[tri[1] as usize] >= 0.5, phi[tri[2] as usize] >= 0.5];
            let n_in = ins.iter().filter(|&&x| x).count();
            let oc = other(t, m);
            if n_in == 3 {
                new_tris.push(tri);
                new_col.push(c);
                continue;
            }
            if n_in == 0 {
                new_tris.push(tri);
                new_col.push(oc);
                continue;
            }
            // allein liegende Ecke
            let i = (0..3).find(|&k| ins[k] != ins[(k + 1) % 3] && ins[k] != ins[(k + 2) % 3]).unwrap();
            let (o, p, q) = (tri[i], tri[(i + 1) % 3], tri[(i + 2) % 3]);
            let mut pt = |u: u32, w: u32, verts: &mut Vec<V3>| -> u32 {
                let (lo, hi) = (u.min(w), u.max(w));
                *cache.entry((lo, hi)).or_insert_with(|| {
                    let (fl, fh) = (phi[lo as usize], phi[hi as usize]);
                    let s = if (fh - fl).abs() > 1e-12 { ((0.5 - fl) / (fh - fl)).clamp(0.06, 0.94) } else { 0.5 };
                    let (a, b) = (m.verts[lo as usize], m.verts[hi as usize]);
                    verts.push(a + (b - a) * s);
                    (verts.len() - 1) as u32
                })
            };
            let x = pt(o, p, &mut verts);
            let y = pt(o, q, &mut verts);
            let (c_odd, c_rest) = if ins[i] { (c, oc) } else { (oc, c) };
            new_tris.push([o, x, y]);
            new_col.push(c_odd);
            new_tris.push([x, p, q]);
            new_col.push(c_rest);
            new_tris.push([x, q, y]);
            new_col.push(c_rest);
            split_count += 1;
        }
        m.verts = verts;
        m.tris = new_tris;
        m.col = new_col;
    }
    split_count
}

fn merge_specks(m: &mut ColoredMesh, min_area: f64) -> usize {
    if min_area <= 0.0 {
        return 0;
    }
    let mut merged = 0;
    for _ in 0..8 {
        let reg = regions(m);
        let em = undirected_map(&m.tris);
        let mut order: Vec<usize> = (0..reg.list.len()).filter(|&r| reg.list[r].area < min_area).collect();
        if order.is_empty() || reg.list.len() < 2 {
            break;
        }
        order.sort_by(|&a, &b| reg.list[a].area.partial_cmp(&reg.list[b].area).unwrap());
        let mut changed = false;
        let mut recolored = vec![false; reg.list.len()];
        for r in order {
            // Nachbarbereich mit der längsten gemeinsamen Grenze
            let mut border: HashMap<u32, f64> = HashMap::new();
            for &t in &reg.list[r].tris {
                let tri = m.tris[t as usize];
                for k in 0..3 {
                    let (a, b) = (tri[k], tri[(k + 1) % 3]);
                    for &u in &em[&(a.min(b), a.max(b))] {
                        let ru = reg.of[u as usize];
                        if ru != r as u32 && !recolored[ru as usize] {
                            *border.entry(ru).or_default() += m.verts[a as usize].dist(m.verts[b as usize]);
                        }
                    }
                }
            }
            if let Some((&nb, _)) = border.iter().max_by(|a, b| a.1.partial_cmp(b.1).unwrap().then(b.0.cmp(a.0))) {
                let c = reg.list[nb as usize].color;
                for &t in &reg.list[r].tris {
                    m.col[t as usize] = c;
                }
                recolored[r] = true;
                merged += 1;
                changed = true;
            }
        }
        if !changed {
            break;
        }
    }
    merged
}

// ---------------------------------------------------------------------------
// Arbeitsnetz mit gerichteter Kantentabelle

struct Work {
    verts: Vec<V3>,
    tris: Vec<[u32; 3]>,
    alive: Vec<bool>,
    lab: Vec<u32>,
    col: Vec<u8>,
    emap: HashMap<(u32, u32), u32>,
}

const BASE: u32 = u32::MAX;

impl Work {
    fn new(m: &ColoredMesh, reg: &Regions) -> Work {
        let mut w = Work {
            verts: m.verts.clone(),
            tris: Vec::with_capacity(m.tris.len()),
            alive: Vec::new(),
            lab: Vec::new(),
            col: Vec::new(),
            emap: HashMap::with_capacity(m.tris.len() * 3),
        };
        for (t, tri) in m.tris.iter().enumerate() {
            w.add(*tri, reg.of[t], m.col[t]);
        }
        w
    }
    fn add(&mut self, t: [u32; 3], lab: u32, col: u8) -> u32 {
        let id = self.tris.len() as u32;
        self.tris.push(t);
        self.alive.push(true);
        self.lab.push(lab);
        self.col.push(col);
        for k in 0..3 {
            self.emap.insert((t[k], t[(k + 1) % 3]), id);
        }
        id
    }
    fn remove(&mut self, id: u32) {
        let t = self.tris[id as usize];
        self.alive[id as usize] = false;
        for k in 0..3 {
            let e = (t[k], t[(k + 1) % 3]);
            if self.emap.get(&e) == Some(&id) {
                self.emap.remove(&e);
            }
        }
    }
}

/// Gerichtete Randkanten zu geschlossenen Schleifen verketten.
fn chain_loops(edges: &[(u32, u32)]) -> (Vec<Vec<u32>>, usize) {
    let mut out: HashMap<u32, Vec<usize>> = HashMap::new();
    for (i, e) in edges.iter().enumerate() {
        out.entry(e.0).or_default().push(i);
    }
    let mut used = vec![false; edges.len()];
    let mut loops = Vec::new();
    let mut broken = 0;
    for s in 0..edges.len() {
        if used[s] {
            continue;
        }
        used[s] = true;
        let start = edges[s].0;
        let mut lp = vec![start];
        let mut cur = edges[s].1;
        let mut ok = false;
        for _ in 0..edges.len() + 1 {
            if cur == start {
                ok = true;
                break;
            }
            lp.push(cur);
            let next = out.get(&cur).and_then(|l| l.iter().copied().find(|&i| !used[i]));
            match next {
                Some(i) => {
                    used[i] = true;
                    cur = edges[i].1;
                }
                None => break,
            }
        }
        if ok && lp.len() >= 3 {
            loops.push(lp);
        } else if !ok {
            broken += 1;
        }
    }
    (loops, broken)
}

/// Ebene durch Punkte (Newell-Normale, Schwerpunkt).
fn newell(pts: &[V3]) -> (V3, V3) {
    let mut n = V3::ZERO;
    let mut c = V3::ZERO;
    for i in 0..pts.len() {
        let (a, b) = (pts[i], pts[(i + 1) % pts.len()]);
        n = n + v3((a.y - b.y) * (a.z + b.z), (a.z - b.z) * (a.x + b.x), (a.x - b.x) * (a.y + b.y));
        c = c + a;
    }
    (c / pts.len().max(1) as f64, n)
}

/// Minimalflächen-ähnliche Kappe über einer geschlossenen Schleife.
/// Rückgabe: zusätzliche Punkte (Index ab loop.len()) und Dreiecke, deren Rand der
/// Schleifenrichtung folgt.
/// Schnitt Strahl p + t·d mit Dreieck (beidseitig); liefert t.
fn ray_tri(p: V3, d: V3, a: V3, b: V3, c: V3) -> Option<f64> {
    let e1 = b - a;
    let e2 = c - a;
    let h = d.cross(e2);
    let det = e1.dot(h);
    if det.abs() < 1e-14 {
        return None;
    }
    let f = 1.0 / det;
    let s = p - a;
    let u = f * s.dot(h);
    if !(-1e-9..=1.0 + 1e-9).contains(&u) {
        return None;
    }
    let q = s.cross(e1);
    let v = f * d.dot(q);
    if v < -1e-9 || u + v > 1.0 + 1e-9 {
        return None;
    }
    Some(f * e2.dot(q))
}

fn membrane(lp: &[V3]) -> (Vec<V3>, Vec<[usize; 3]>) {
    let n = lp.len();
    let (c, nn) = newell(lp);
    let len = nn.norm();
    let normal = if len > 1e-30 { nn / len } else { V3::Z };
    let (e1, e2) = normal.basis();
    // e1 x e2 = normal
    let p2: Vec<P2> = lp.iter().map(|p| [(*p - c).dot(e1), (*p - c).dot(e2)]).collect();
    let mut pts: Vec<V3> = lp.to_vec();
    let mut q2: Vec<P2> = p2.clone();
    let mut tris: Vec<[usize; 3]> = Vec::new();
    let simple = poly::signed_area(&p2) > 0.0 && is_simple(&p2);
    if simple {
        tris = poly::triangulate(&[p2.clone()]);
    }
    let planar_ok = !tris.is_empty();
    if !planar_ok {
        // Fächer vom Schwerpunkt
        pts.push(c);
        q2.push([0.0, 0.0]);
        for i in 0..n {
            tris.push([i, (i + 1) % n, n]);
        }
    }
    let avg = (0..n).map(|i| lp[i].dist(lp[(i + 1) % n])).sum::<f64>() / n as f64;
    let span = p2.iter().fold(0.0f64, |m, p| m.max(p[0].abs()).max(p[1].abs()));
    let target = avg.max(span / 16.0).max(1e-9);
    let orient = |a: P2, b: P2, cc: P2| (b[0] - a[0]) * (cc[1] - a[1]) - (b[1] - a[1]) * (cc[0] - a[0]);
    let in_circle = |a: P2, b: P2, cc: P2, d: P2| -> bool {
        let (adx, ady) = (a[0] - d[0], a[1] - d[1]);
        let (bdx, bdy) = (b[0] - d[0], b[1] - d[1]);
        let (cdx, cdy) = (cc[0] - d[0], cc[1] - d[1]);
        let det = (adx * adx + ady * ady) * (bdx * cdy - cdx * bdy) - (bdx * bdx + bdy * bdy) * (adx * cdy - cdx * ady)
            + (cdx * cdx + cdy * cdy) * (adx * bdy - bdx * ady);
        det > 1e-12 * (span * span).max(1e-12) * (span * span).max(1e-12)
    };
    let edge_map = |tris: &Vec<[usize; 3]>| -> HashMap<(usize, usize), Vec<usize>> {
        let mut em: HashMap<(usize, usize), Vec<usize>> = HashMap::with_capacity(tris.len() * 2);
        for (t, tri) in tris.iter().enumerate() {
            for k in 0..3 {
                let (a, b) = (tri[k], tri[(k + 1) % 3]);
                em.entry((a.min(b), a.max(b))).or_default().push(t);
            }
        }
        em
    };
    // Delaunay-Kantentausch in der Projektion (nur bei ebener Ausgangstriangulierung sinnvoll)
    let flip_pass = |tris: &mut Vec<[usize; 3]>, q2: &Vec<P2>| {
        for _ in 0..20 {
            let mut em = edge_map(tris);
            let mut keys: Vec<(usize, usize)> = em.iter().filter(|(_, l)| l.len() == 2).map(|(k, _)| *k).collect();
            keys.sort();
            let mut flipped = 0;
            for (a, b) in keys {
                let l = match em.get(&(a, b)) {
                    Some(l) if l.len() == 2 => l.clone(),
                    _ => continue,
                };
                let (t1, t2) = (l[0], l[1]);
                // t1 enthält a->b oder b->a; so drehen, dass t1 = (x, y, cx) mit Kante x->y
                let tri1 = tris[t1];
                let k1 = match (0..3).find(|&k| {
                    let (x, y) = (tri1[k], tri1[(k + 1) % 3]);
                    (x == a && y == b) || (x == b && y == a)
                }) {
                    Some(k) => k,
                    None => continue,
                };
                let (x, y, cx) = (tri1[k1], tri1[(k1 + 1) % 3], tri1[(k1 + 2) % 3]);
                let tri2 = tris[t2];
                let k2 = match (0..3).find(|&k| tri2[k] == y && tri2[(k + 1) % 3] == x) {
                    Some(k) => k,
                    None => continue,
                };
                let d = tri2[(k2 + 2) % 3];
                if cx < n && d < n {
                    continue; // würde eine Sehne erzeugen
                }
                if em.contains_key(&(cx.min(d), cx.max(d))) {
                    continue;
                }
                if !in_circle(q2[x], q2[y], q2[cx], q2[d]) {
                    continue;
                }
                // neue Dreiecke (x, d, cx) und (d, y, cx) müssen gültig orientiert sein
                if orient(q2[x], q2[d], q2[cx]) <= 0.0 || orient(q2[d], q2[y], q2[cx]) <= 0.0 {
                    continue;
                }
                tris[t1] = [x, d, cx];
                tris[t2] = [d, y, cx];
                // Kantentabelle aktualisieren
                em.remove(&(a, b));
                em.insert((cx.min(d), cx.max(d)), vec![t1, t2]);
                for (e, from, to) in [((x.min(d), x.max(d)), t2, t1), ((y.min(cx), y.max(cx)), t1, t2)] {
                    if let Some(v) = em.get_mut(&e) {
                        for t in v.iter_mut() {
                            if *t == from {
                                *t = to;
                            }
                        }
                    }
                }
                flipped += 1;
            }
            if flipped == 0 {
                break;
            }
        }
    };
    if planar_ok {
        flip_pass(&mut tris, &q2);
    }
    // Verfeinern: Sehnen zwischen zwei Randpunkten immer teilen (sie könnten mit
    // vorhandenen Netzkanten zusammenfallen), danach lange innere Kanten halbieren
    for _ in 0..60 {
        if tris.len() > 20_000 {
            break;
        }
        let em = edge_map(&tris);
        let longest = |t: usize| -> (usize, usize) {
            let tri = tris[t];
            let mut best = (0.0, 0, 0);
            for k in 0..3 {
                let (a, b) = (tri[k], tri[(k + 1) % 3]);
                let d = pts[a].dist(pts[b]);
                if d > best.0 {
                    best = (d, a.min(b), a.max(b));
                }
            }
            (best.1, best.2)
        };
        let mut cand: Vec<(f64, usize, usize)> = em
            .iter()
            .filter(|(_, l)| l.len() == 2)
            .filter_map(|(&(a, b), l)| {
                if a < n && b < n {
                    return Some((f64::MAX, a, b));
                }
                // nur Kanten teilen, die in beiden Dreiecken die längste sind (keine Splitter)
                if longest(l[0]) != (a, b) || longest(l[1]) != (a, b) {
                    return None;
                }
                Some((pts[a].dist(pts[b]), a, b))
            })
            .filter(|c| c.0 > target * 1.5)
            .collect();
        if cand.is_empty() {
            break;
        }
        cand.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap().then(a.1.cmp(&b.1)).then(a.2.cmp(&b.2)));
        let mut used = vec![false; tris.len()];
        for (_, a, b) in cand {
            let l = &em[&(a, b)];
            let (t1, t2) = (l[0], l[1]);
            if used[t1] || used[t2] {
                continue;
            }
            used[t1] = true;
            used[t2] = true;
            pts.push((pts[a] + pts[b]) * 0.5);
            q2.push([(q2[a][0] + q2[b][0]) * 0.5, (q2[a][1] + q2[b][1]) * 0.5]);
            let m = pts.len() - 1;
            for &t in &[t1, t2] {
                let tri = tris[t];
                let k = (0..3).find(|&k| {
                    let (x, y) = (tri[k], tri[(k + 1) % 3]);
                    (x == a && y == b) || (x == b && y == a)
                });
                if let Some(k) = k {
                    let (x, y, z) = (tri[k], tri[(k + 1) % 3], tri[(k + 2) % 3]);
                    tris[t] = [x, m, z];
                    tris.push([m, y, z]);
                    used.push(true);
                }
            }
        }
        if planar_ok {
            flip_pass(&mut tris, &q2);
        }
    }
    // Dreieck nur aus Randpunkten (Schleife mit 3 Punkten): Mittelpunkt einfügen
    if tris.len() == 1 && n == 3 {
        pts.push(c);
        tris = vec![[0, 1, 3], [1, 2, 3], [2, 0, 3]];
    }
    // Glätten: innere Punkte zum Mittel der Nachbarn (diskrete harmonische Fläche)
    if pts.len() > n {
        let mut nb: Vec<Vec<usize>> = vec![Vec::new(); pts.len()];
        for tri in &tris {
            for k in 0..3 {
                let (a, b) = (tri[k], tri[(k + 1) % 3]);
                if !nb[a].contains(&b) {
                    nb[a].push(b);
                }
                if !nb[b].contains(&a) {
                    nb[b].push(a);
                }
            }
        }
        for _ in 0..80 {
            for i in n..pts.len() {
                if nb[i].is_empty() {
                    continue;
                }
                let mut s = V3::ZERO;
                for &j in &nb[i] {
                    s = s + pts[j];
                }
                pts[i] = s / nb[i].len() as f64;
            }
        }
    }
    (pts[n..].to_vec(), tris)
}

fn seg_cross(a: P2, b: P2, c: P2, d: P2) -> bool {
    let o = |p: P2, q: P2, r: P2| (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0]);
    let (d1, d2, d3, d4) = (o(a, b, c), o(a, b, d), o(c, d, a), o(c, d, b));
    ((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0)) && ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0))
}

/// Polygon ohne Selbstüberschneidung? (Raster-Beschleunigung für große Polygone)
pub(crate) fn is_simple(p: &[P2]) -> bool {
    let n = p.len();
    if n < 4 {
        return true;
    }
    let (mut mn, mut mx) = ([f64::MAX; 2], [f64::MIN; 2]);
    for q in p {
        for k in 0..2 {
            mn[k] = mn[k].min(q[k]);
            mx[k] = mx[k].max(q[k]);
        }
    }
    let g = ((n as f64).sqrt() as usize).clamp(1, 256);
    let cell = |x: f64, k: usize| -> usize {
        let s = (mx[k] - mn[k]).max(1e-30);
        (((x - mn[k]) / s * g as f64) as usize).min(g - 1)
    };
    let mut grid: Vec<Vec<usize>> = vec![Vec::new(); g * g];
    for i in 0..n {
        let (a, b) = (p[i], p[(i + 1) % n]);
        let (x0, x1) = (cell(a[0].min(b[0]), 0), cell(a[0].max(b[0]), 0));
        let (y0, y1) = (cell(a[1].min(b[1]), 1), cell(a[1].max(b[1]), 1));
        for x in x0..=x1 {
            for y in y0..=y1 {
                grid[y * g + x].push(i);
            }
        }
    }
    for c in &grid {
        for (ii, &i) in c.iter().enumerate() {
            for &j in &c[ii + 1..] {
                if (i + 1) % n == j || (j + 1) % n == i {
                    continue;
                }
                if seg_cross(p[i], p[(i + 1) % n], p[j], p[(j + 1) % n]) {
                    return false;
                }
            }
        }
    }
    true
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CapMethod {
    /// Minimalfläche bei gewölbten Bereichen, sonst Tiefe
    Auto,
    /// gespannte Fläche über dem Rand (wie eine Seifenhaut)
    Membrane,
    /// Bereich als Schale mit fester Tiefe nach innen versetzen
    Depth,
}

#[derive(Clone, Debug)]
pub struct ColorSplitOpts {
    pub cap: CapMethod,
    /// Tiefe für die Schalen-Methode (mm)
    pub depth: f64,
    /// gleichfarbige Bereiche zu einem Teil zusammenfassen
    pub merge_same_color: bool,
}

/// Trennt die ausgewählten Farbbereiche als eigene geschlossene Teile ab.
pub fn split_by_color(m: &ColoredMesh, reg: &Regions, extract: &[bool], o: &ColorSplitOpts, group: usize) -> (Vec<Part>, Vec<String>) {
    let mut w = Work::new(m, reg);
    let mut warnings = Vec::new();
    let mut parts: Vec<Part> = Vec::new();
    let mut order: Vec<usize> = (0..reg.list.len()).filter(|&r| extract.get(r).copied().unwrap_or(false)).collect();
    order.sort_by(|&a, &b| reg.list[a].area.partial_cmp(&reg.list[b].area).unwrap());
    let mut open_total = 0usize;
    // aktuelle Dreiecke je Bereich (Kappen kommen später hinzu)
    let mut members: HashMap<u32, Vec<u32>> = HashMap::new();
    for (r, info) in reg.list.iter().enumerate() {
        members.insert(r as u32, info.tris.clone());
    }
    let mut thin = 0usize;
    let mut hollow = 0usize;
    for r in order {
        let rl = r as u32;
        let rcol = reg.list[r].color;
        let rt: Vec<u32> = members.get(&rl).map(|v| v.iter().copied().filter(|&t| w.alive[t as usize]).collect()).unwrap_or_default();
        if rt.is_empty() {
            continue;
        }
        // Randkanten und Nachbarn
        let mut bedges: Vec<(u32, u32)> = Vec::new();
        let mut nb_of_edge: HashMap<(u32, u32), u32> = HashMap::new();
        for &t in &rt {
            let tri = w.tris[t as usize];
            for k in 0..3 {
                let (a, b) = (tri[k], tri[(k + 1) % 3]);
                match w.emap.get(&(b, a)) {
                    Some(&u) if w.lab[u as usize] == rl => {}
                    Some(&u) => {
                        bedges.push((a, b));
                        nb_of_edge.insert((a, b), u);
                    }
                    None => {
                        bedges.push((a, b));
                        open_total += 1;
                    }
                }
            }
        }
        let (loops, _broken) = chain_loops(&bedges);
        if std::env::var("M2S_DEBUG").is_ok() {
            for lp in &loops {
                let mut u = lp.clone();
                u.sort();
                u.dedup();
                let zs: Vec<f64> = lp.iter().map(|&i| w.verts[i as usize].z).collect();
                let (zmin, zmax) = zs.iter().fold((f64::MAX, f64::MIN), |a, &z| (a.0.min(z), a.1.max(z)));
                eprintln!("Bereich {r}: Schleife {} Punkte ({} verschieden) z {zmin:.2}..{zmax:.2}", lp.len(), u.len());
            }
        }
        let mut ptris: Vec<[u32; 3]> = rt.iter().map(|&t| w.tris[t as usize]).collect();
        let mut pcol: Vec<u8> = rt.iter().map(|&t| w.col[t as usize]).collect();
        // neue Flächen für den Rest: (Dreieck, Bereich, Farbe)
        let mut rest_new: Vec<([u32; 3], u32, u8)> = Vec::new();
        // Nachbarbereich je Schleife (häufigster entlang des Randes)
        let loop_label = |lp: &Vec<u32>, w: &Work| -> (u32, u8) {
            let mut cnt: HashMap<u32, (f64, u8)> = HashMap::new();
            for i in 0..lp.len() {
                let e = (lp[i], lp[(i + 1) % lp.len()]);
                if let Some(&u) = nb_of_edge.get(&e) {
                    let l = w.verts[e.0 as usize].dist(w.verts[e.1 as usize]);
                    let ent = cnt.entry(w.lab[u as usize]).or_insert((0.0, w.col[u as usize]));
                    ent.0 += l;
                }
            }
            cnt.into_iter()
                .max_by(|a, b| a.1 .0.partial_cmp(&b.1 .0).unwrap().then(b.0.cmp(&a.0)))
                .map(|(l, (_, c))| (l, c))
                .unwrap_or((BASE, reg.base_color))
        };
        if !loops.is_empty() {
            // Methode bestimmen: wie weit wölbt sich der Bereich über die Randebene?
            let all: Vec<V3> = loops.iter().flatten().map(|&i| w.verts[i as usize]).collect();
            let (c, nn) = newell(&all);
            let nrm = if nn.norm() > 1e-30 { nn.normalized() } else { V3::Z };
            let mut h = 0.0f64;
            for &t in &rt {
                for &v in &w.tris[t as usize] {
                    h = h.max((w.verts[v as usize] - c).dot(nrm).abs());
                }
            }
            // Gespannte Haut vorab berechnen: nur sinnvoll, wenn der Bereich nach außen
            // gewölbt ist (Knopf, Kappe). Bei Vertiefungen (Taschen, Anzeigen, Kerben)
            // läge die Haut außerhalb des Körpers – das Teil wäre „Luft“ mit negativem
            // Volumen und der Grundkörper bekäme eine zugeklebte Tasche.
            let mems: Vec<(Vec<V3>, Vec<[usize; 3]>)> = if o.cap == CapMethod::Depth {
                Vec::new()
            } else {
                loops.iter().map(|lp| membrane(&lp.iter().map(|&i| w.verts[i as usize]).collect::<Vec<_>>())).collect()
            };
            let bulges = if mems.is_empty() {
                false
            } else {
                let det = |a: V3, b: V3, c: V3| a.dot(b.cross(c)) / 6.0;
                let mut vol = 0.0;
                for &t in &rt {
                    let [a, b, c] = w.tris[t as usize];
                    vol += det(w.verts[a as usize], w.verts[b as usize], w.verts[c as usize]);
                }
                let mut cap_area = 0.0;
                for (lp, (extra, ct)) in loops.iter().zip(&mems) {
                    let pt = |k: usize| if k < lp.len() { w.verts[lp[k] as usize] } else { extra[k - lp.len()] };
                    for t in ct {
                        let (a, b, c) = (pt(t[0]), pt(t[1]), pt(t[2]));
                        vol += det(a, c, b);
                        cap_area += (b - a).cross(c - a).norm() * 0.5;
                    }
                }
                if std::env::var("M2S_DEBUG").is_ok() {
                    eprintln!("  Wölbung: Volumen {vol:.2}, Haut {cap_area:.2} mm², h {h:.2} → {:.2}", vol / (cap_area * h).max(1e-12));
                }
                vol > 0.2 * cap_area * h
            };
            let use_depth = match o.cap {
                CapMethod::Depth => true,
                CapMethod::Membrane => false,
                // Erhabene Bereiche (Knöpfe, Rahmen, Kappen) werden gespannt: das Teil ist
                // genau die Erhebung mit flachem Boden auf Höhe des Randes. Nur fast
                // ebene Bemalung (Logos, Schrift, Aufdrucke) und Vertiefungen bekommen
                // eine Schale mit fester Tiefe.
                CapMethod::Auto => !bulges || h < 0.4f64.max(0.25 * o.depth),
            };
            if o.cap == CapMethod::Membrane && !bulges {
                hollow += 1;
            }
            if !use_depth && h < 0.2 * o.depth.max(0.2) {
                thin += 1;
            }
            if use_depth {
                // Schale: Bereich um `depth` entlang der Eckennormalen nach innen versetzen
                let mut vn: HashMap<u32, (V3, f64)> = HashMap::new();
                let mut tn: Vec<V3> = Vec::new();
                for &t in &rt {
                    let tri = w.tris[t as usize];
                    let na = tri_normal_area(&w.verts, &tri);
                    tn.push(if na.norm() > 0.0 { na.normalized() } else { V3::ZERO });
                    for &v in &tri {
                        let e = vn.entry(v).or_insert((V3::ZERO, 0.0));
                        e.0 = e.0 + na;
                    }
                }
                for (k, &t) in rt.iter().enumerate() {
                    for &v in &w.tris[t as usize] {
                        let e = vn.get_mut(&v).unwrap();
                        let d = if e.0.norm() > 0.0 { e.0.normalized().dot(tn[k]) } else { 1.0 };
                        e.1 = if e.1 == 0.0 { d } else { e.1.min(d) };
                    }
                }
                let mut off: HashMap<u32, u32> = HashMap::new();
                let mut vlist: Vec<u32> = vn.keys().copied().collect();
                vlist.sort();
                for v in vlist {
                    let (n, cmin) = vn[&v];
                    let dir = if n.norm() > 0.0 { n.normalized() } else { V3::Z };
                    let f = 1.0 / cmin.max(0.5);
                    w.verts.push(w.verts[v as usize] - dir * (o.depth * f));
                    off.insert(v, (w.verts.len() - 1) as u32);
                }
                // Boden der Schale
                for &t in &rt {
                    let [a, b, c] = w.tris[t as usize];
                    let (a2, b2, c2) = (off[&a], off[&b], off[&c]);
                    ptris.push([a2, c2, b2]);
                    pcol.push(rcol);
                }
                // Seitenwände je Randschleife
                for lp in &loops {
                    let (nl, ncol) = loop_label(lp, &w);
                    for i in 0..lp.len() {
                        let (a, b) = (lp[i], lp[(i + 1) % lp.len()]);
                        let (a2, b2) = (off[&a], off[&b]);
                        ptris.push([b, a, a2]);
                        ptris.push([b, a2, b2]);
                        pcol.push(rcol);
                        pcol.push(rcol);
                        rest_new.push(([b, a2, a], nl, ncol));
                        rest_new.push(([b, b2, a2], nl, ncol));
                    }
                }
                // Boden gehört im Rest zum größten Nachbarn
                let (nl, ncol) = loops
                    .iter()
                    .max_by_key(|l| l.len())
                    .map(|l| loop_label(l, &w))
                    .unwrap_or((BASE, reg.base_color));
                for &t in &rt {
                    let [a, b, c] = w.tris[t as usize];
                    rest_new.push(([off[&a], off[&b], off[&c]], nl, ncol));
                }
            } else {
                // Richtung der Erhebung
                let mut sgn = 0.0;
                for &t in &rt {
                    for &v in &w.tris[t as usize] {
                        sgn += (w.verts[v as usize] - c).dot(nrm);
                    }
                }
                let up = if sgn < 0.0 { nrm * -1.0 } else { nrm };
                // Mindestdicke: liegt die bemalte Fläche fast auf Randhöhe (z. B. eine
                // Anzeige mit vertiefter Scheibe in einem Ring), wird die Haut nach innen
                // abgesenkt, damit das Teil nicht hauchdünn wird oder sich durchdringt.
                let tmin = 0.6f64.min(o.depth.max(0.2));
                let mut clear = f64::MAX;
                for (lp, (extra, _)) in loops.iter().zip(&mems) {
                    let _ = lp;
                    for &q in extra {
                        let mut best: Option<f64> = None;
                        for &t in &rt {
                            let [a, b, cc] = w.tris[t as usize];
                            if let Some(d) = ray_tri(q, up, w.verts[a as usize], w.verts[b as usize], w.verts[cc as usize]) {
                                if best.map_or(true, |x: f64| d.abs() < x.abs()) {
                                    best = Some(d);
                                }
                            }
                        }
                        if let Some(d) = best {
                            clear = clear.min(d);
                        }
                    }
                }
                // Nur bei einer einzelnen Randschleife (bei Bändern um einen Ring o. Ä. sind
                // die Häute Querschnitte) und nur, wenn die Haut die Fläche höchstens streift.
                let sink = if loops.len() == 1 && clear < tmin && clear > -tmin { tmin - clear } else { 0.0 };
                if std::env::var("M2S_DEBUG").is_ok() {
                    eprintln!("  Haut-Abstand {clear:.2} mm → absenken {sink:.2} mm");
                }
                for (lp, (extra, ct)) in loops.iter().zip(mems) {
                    if std::env::var("M2S_DEBUG").is_ok() {
                        eprintln!("  Haut: {} Randpunkte, {} Zusatzpunkte, {} Dreiecke", lp.len(), extra.len(), ct.len());
                    }
                    let (nl, ncol) = loop_label(lp, &w);
                    // Rand der Haut: Schleife selbst oder abgesenkte Kopie mit Wand
                    let rim: Vec<u32> = if sink > 1e-3 {
                        let mut r = Vec::with_capacity(lp.len());
                        for &i in lp {
                            w.verts.push(w.verts[i as usize] - up * sink);
                            r.push((w.verts.len() - 1) as u32);
                        }
                        for i in 0..lp.len() {
                            let j = (i + 1) % lp.len();
                            let (a, b, a2, b2) = (lp[i], lp[j], r[i], r[j]);
                            ptris.push([b, a, a2]);
                            ptris.push([b, a2, b2]);
                            pcol.push(rcol);
                            pcol.push(rcol);
                            rest_new.push(([b, a2, a], nl, ncol));
                            rest_new.push(([b, b2, a2], nl, ncol));
                        }
                        r
                    } else {
                        lp.clone()
                    };
                    let base = w.verts.len() as u32;
                    w.verts.extend(extra.iter().map(|&q| q - up * sink));
                    let idx = |k: usize| if k < rim.len() { rim[k] } else { base + (k - rim.len()) as u32 };
                    for t in ct {
                        let (a, b, c) = (idx(t[0]), idx(t[1]), idx(t[2]));
                        // Teil: Kappe gegen die Schleifenrichtung; Rest: in Schleifenrichtung
                        ptris.push([a, c, b]);
                        pcol.push(rcol);
                        rest_new.push(([a, b, c], nl, ncol));
                    }
                }
            }
        }
        for &t in &rt {
            w.remove(t);
        }
        for (t, l, c) in rest_new {
            let id = w.add(t, l, c);
            members.entry(l).or_default().push(id);
        }
        let name = format!("{} Farbe {}", m.name, rcol);
        parts.push(Part::from_soup(name, &w.verts, &ptris, &pcol, rcol, group, PartKind::ColorPart));
    }
    // Rest = Grundkörper
    let rest: Vec<usize> = (0..w.tris.len()).filter(|&t| w.alive[t]).collect();
    if !rest.is_empty() {
        let tris: Vec<[u32; 3]> = rest.iter().map(|&t| w.tris[t]).collect();
        let col: Vec<u8> = rest.iter().map(|&t| w.col[t]).collect();
        let main = if col.is_empty() { reg.base_color } else { most_common(&col) };
        parts.insert(0, Part::from_soup(format!("{} Grundkörper", m.name), &w.verts, &tris, &col, main, group, PartKind::Body));
    }
    if o.merge_same_color {
        let mut merged: Vec<Part> = Vec::new();
        for p in parts {
            if p.kind == PartKind::ColorPart {
                if let Some(q) = merged.iter_mut().find(|q| q.kind == PartKind::ColorPart && q.color == p.color) {
                    let base = q.verts.len() as u32;
                    q.verts.extend(p.verts);
                    q.tris.extend(p.tris.iter().map(|t| [t[0] + base, t[1] + base, t[2] + base]));
                    q.col.extend(p.col);
                    q.update_stats();
                    continue;
                }
            }
            merged.push(p);
        }
        parts = merged;
    }
    if open_total > 0 {
        warnings.push(format!("{}: Netz ist nicht geschlossen ({open_total} offene Kanten) – Teile können Lücken haben", m.name));
    }
    if hollow > 0 {
        warnings.push(format!("{hollow} Bereiche sind Vertiefungen – „Gespannt“ füllt sie auf; besser „Automatisch“ oder „Schale“"));
    }
    if thin > 0 {
        warnings.push(format!("{thin} Bereiche sind fast eben – für dickere Teile „Tiefe“ als Abschluss wählen"));
    }
    (parts, warnings)
}

// ---------------------------------------------------------------------------
// Schnitt mit einer Ebene

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ConnKind {
    None,
    /// Zapfen auf der unteren Hälfte, Loch in der oberen
    Peg,
    /// Löcher in beiden Hälften und lose Dübel
    Dowel,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ConnShape {
    Cylinder,
    Square,
    Triangle,
}

#[derive(Clone, Debug)]
pub struct CutOpts {
    pub origin: V3,
    pub normal: V3,
    pub conn: ConnKind,
    pub shape: ConnShape,
    /// Durchmesser bzw. Kantenlänge (mm)
    pub size: f64,
    /// Tiefe je Seite (mm)
    pub depth: f64,
    /// Spiel je Seite (mm)
    pub clearance: f64,
    /// Anzahl je Schnittfläche (0 = automatisch)
    pub count: usize,
}

impl Default for CutOpts {
    fn default() -> Self {
        CutOpts {
            origin: V3::ZERO,
            normal: V3::Z,
            conn: ConnKind::None,
            shape: ConnShape::Cylinder,
            size: 5.0,
            depth: 5.0,
            clearance: 0.15,
            count: 0,
        }
    }
}

fn conn_ring(shape: ConnShape, size: f64) -> Vec<P2> {
    let (n, r, rot) = match shape {
        ConnShape::Cylinder => (32usize, size * 0.5, 0.0),
        ConnShape::Square => (4, size * std::f64::consts::FRAC_1_SQRT_2, std::f64::consts::FRAC_PI_4),
        // gleichseitiges Dreieck mit Kantenlänge size
        ConnShape::Triangle => (3, size / 3f64.sqrt(), std::f64::consts::FRAC_PI_2),
    };
    (0..n)
        .map(|k| {
            let a = rot + k as f64 / n as f64 * std::f64::consts::TAU;
            [r * a.cos(), r * a.sin()]
        })
        .collect()
}

fn ring_radius(shape: ConnShape, size: f64) -> f64 {
    match shape {
        ConnShape::Cylinder => size * 0.5,
        ConnShape::Square => size * std::f64::consts::FRAC_1_SQRT_2,
        ConnShape::Triangle => size / 3f64.sqrt(),
    }
}

/// Auf welcher Seite der Zapfen eines Farbteils sitzt.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ConnSide {
    /// Zapfen am Farbteil, Loch im Grundkörper
    Part,
    /// Zapfen am Grundkörper, Loch im Farbteil
    Body,
}

/// Verbinder zwischen Farbteilen und Grundkörper.
#[derive(Clone, Debug)]
pub struct ColorConnOpts {
    pub shape: ConnShape,
    pub side: ConnSide,
    /// Querschnitt des Zapfens in Prozent der Kontaktfläche
    pub area_pct: f64,
    /// Spiel je Seite (mm)
    pub tol: f64,
}

/// Geschlossenes Prisma mit dem Querschnitt `ring` von `a` nach `b` (nach außen orientiert).
fn prism(ring: &[P2], a: V3, b: V3) -> (Vec<V3>, Vec<[u32; 3]>) {
    let n = (b - a).normalized();
    let (e1, e2) = n.basis();
    let k = ring.len() as u32;
    let mut v = Vec::with_capacity(2 * ring.len() + 2);
    for c in [a, b] {
        for p in ring {
            v.push(c + e1 * p[0] + e2 * p[1]);
        }
    }
    v.push(a);
    v.push(b);
    let (ca, cb) = (2 * k, 2 * k + 1);
    let mut t = Vec::new();
    for i in 0..k {
        let j = (i + 1) % k;
        t.push([i, j, k + j]);
        t.push([i, k + j, k + i]);
        t.push([ca, j, i]);
        t.push([cb, k + i, k + j]);
    }
    if volume(&v, &t) < 0.0 {
        for x in t.iter_mut() {
            x.swap(1, 2);
        }
    }
    (v, t)
}

/// Strahl gegen ein Netz: Abstand zum nächsten Treffer (∞ ohne Treffer).
fn ray_dist(verts: &[V3], tris: &[[u32; 3]], o: V3, d: V3) -> f64 {
    let mut best = f64::INFINITY;
    for t in tris {
        if let Some(h) = ray_tri(o, d, verts[t[0] as usize], verts[t[1] as usize], verts[t[2] as usize]) {
            if h > 1e-6 && h < best {
                best = h;
            }
        }
    }
    best
}

/// Vereinigung bzw. Differenz und Farben der neuen Dreiecke übernehmen.
fn apply_bool(p: &mut Part, other_v: &[V3], other_t: &[[u32; 3]], add: bool, new_col: u8) -> bool {
    let r = if add {
        let mut v = p.verts.clone();
        let mut t = p.tris.clone();
        let base = v.len() as u32;
        v.extend_from_slice(other_v);
        t.extend(other_t.iter().map(|x| [x[0] + base, x[1] + base, x[2] + base]));
        crate::union::union(&v, &t)
    } else {
        crate::union::subtract(&p.verts, &p.tris, other_v, other_t)
    };
    if r.merged == 0 || r.tris.is_empty() {
        return false;
    }
    let n0 = p.tris.len() as u32;
    let col: Vec<u8> = r.origin.iter().map(|&o| if o < n0 { p.col[o as usize] } else { new_col }).collect();
    let mut np = Part::from_soup(p.name.clone(), &r.verts, &r.tris, &col, p.color, p.group, p.kind);
    if np.open_edges > 0 {
        // Nähte schließen (gleiche Punkte aus beiden Körpern)
        let (v, t) = crate::repair::weld(&np.verts, &np.tris, 1e-7);
        if t.len() == np.tris.len() {
            np.verts = v;
            np.tris = t;
            np.update_stats();
        }
    }
    *p = np;
    true
}

/// Zapfen zwischen jedem Farbteil und seinem Grundkörper anbringen.
pub fn add_color_connectors(parts: &mut [Part], o: &ColorConnOpts) -> Vec<String> {
    let mut warnings = Vec::new();
    let mut skipped = 0usize;
    let body_idx: Vec<usize> = (0..parts.len()).filter(|&i| parts[i].kind == PartKind::Body).collect();
    for pi in 0..parts.len() {
        if parts[pi].kind != PartKind::ColorPart {
            continue;
        }
        let Some(&bi) = body_idx.iter().find(|&&b| parts[b].group == parts[pi].group) else { continue };
        // Kontaktfläche: Dreiecke des Farbteils, die im Grundkörper gegenläufig vorkommen
        let q = |p: V3| ((p.x * 1e4).round() as i64, (p.y * 1e4).round() as i64, (p.z * 1e4).round() as i64);
        let body_c: std::collections::HashSet<(i64, i64, i64)> = parts[bi]
            .tris
            .iter()
            .map(|t| q((parts[bi].verts[t[0] as usize] + parts[bi].verts[t[1] as usize] + parts[bi].verts[t[2] as usize]) / 3.0))
            .collect();
        let part = &parts[pi];
        let (mut area, mut g, mut nsum) = (0.0, V3::ZERO, V3::ZERO);
        let mut contact: Vec<(V3, f64)> = Vec::new();
        for t in &part.tris {
            let (a, b, c) = (part.verts[t[0] as usize], part.verts[t[1] as usize], part.verts[t[2] as usize]);
            let cen = (a + b + c) / 3.0;
            if !body_c.contains(&q(cen)) {
                continue;
            }
            let n = (b - a).cross(c - a) * 0.5;
            let ar = n.norm();
            area += ar;
            g += cen * ar;
            nsum += n;
            contact.push((cen, ar));
        }
        if area <= 1e-9 || nsum.norm() < 0.3 * area {
            skipped += 1;
            continue;
        }
        let g = g / area;
        let n = nsum.normalized();
        // Mittelpunkt auf der Kontaktfläche (nächster Dreiecksmittelpunkt zum Schwerpunkt)
        let c = contact.iter().min_by(|x, y| x.0.dist(g).partial_cmp(&y.0.dist(g)).unwrap()).map(|x| x.0).unwrap();
        let a_conn = area * (o.area_pct / 100.0).clamp(0.01, 0.6);
        let size = match o.shape {
            ConnShape::Cylinder => 2.0 * (a_conn / std::f64::consts::PI).sqrt(),
            ConnShape::Square => a_conn.sqrt(),
            ConnShape::Triangle => (4.0 * a_conn / 3f64.sqrt()).sqrt(),
        };
        let (pv, pt) = (parts[pi].verts.clone(), parts[pi].tris.clone());
        let (bv, bt) = (parts[bi].verts.clone(), parts[bi].tris.clone());
        // Dicke beider Seiten entlang der Normalen (n zeigt vom Farbteil in den Grundkörper)
        let t_body = ray_dist(&bv, &bt, c + n * 1e-3, n);
        let t_part = ray_dist(&pv, &pt, c - n * 1e-3, -n);
        let (into, back, len_max, base_max) = match o.side {
            ConnSide::Part => (n, -n, t_body, t_part),
            ConnSide::Body => (-n, n, t_part, t_body),
        };
        let len = (size * 1.2).min(len_max * 0.6);
        let base = (size * 0.5).min(base_max * 0.5);
        if size < 1.2 || len < 0.8 || base < 0.3 || !len.is_finite() {
            skipped += 1;
            continue;
        }
        let ring = conn_ring(o.shape, size);
        let ring_hole = conn_ring(o.shape, size + 2.0 * o.tol);
        let (zv, zt) = prism(&ring, c + back * base, c + into * len);
        let (hv, ht) = prism(&ring_hole, c + back * (base + 0.2), c + into * (len + o.tol));
        let (owner, hole) = match o.side {
            ConnSide::Part => (pi, bi),
            ConnSide::Body => (bi, pi),
        };
        let owner_col = parts[owner].color;
        let hole_col = parts[hole].color;
        let mut a = parts[owner].clone();
        let mut b = parts[hole].clone();
        if apply_bool(&mut a, &zv, &zt, true, owner_col) && apply_bool(&mut b, &hv, &ht, false, hole_col) {
            parts[owner] = a;
            parts[hole] = b;
        } else {
            skipped += 1;
        }
    }
    if skipped > 0 {
        warnings.push(format!("{skipped} Farbteil(e) ohne Verbinder – zu klein oder zu dünn"));
    }
    warnings
}

/// Normale einer Schnittebene senkrecht zur Achse (0=x,1=y,2=z), um `tilt_deg` geneigt.
pub fn axis_normal(axis: usize, tilt_deg: f64) -> V3 {
    let a = tilt_deg.to_radians();
    let (s, c) = a.sin_cos();
    // Neigung um die nächste Achse
    let mut n = [0.0; 3];
    n[axis % 3] = c;
    n[(axis + 2) % 3] = s;
    v3(n[0], n[1], n[2])
}

/// Strahl gegen Dreiecke: kleinster positiver Abstand.
fn ray_hit(verts: &[V3], tris: &[[u32; 3]], o: V3, d: V3) -> f64 {
    let mut best = f64::MAX;
    for t in tris {
        let (a, b, c) = (verts[t[0] as usize], verts[t[1] as usize], verts[t[2] as usize]);
        let e1 = b - a;
        let e2 = c - a;
        let p = d.cross(e2);
        let det = e1.dot(p);
        if det.abs() < 1e-14 {
            continue;
        }
        let inv = 1.0 / det;
        let s = o - a;
        let u = s.dot(p) * inv;
        if !(-1e-9..=1.0 + 1e-9).contains(&u) {
            continue;
        }
        let q = s.cross(e1);
        let v = d.dot(q) * inv;
        if v < -1e-9 || u + v > 1.0 + 1e-9 {
            continue;
        }
        let t = e2.dot(q) * inv;
        if t > 1e-6 && t < best {
            best = t;
        }
    }
    best
}

/// Schneidet ein Netz mit einer Ebene. Liefert obere (Normalenseite) und untere Hälfte
/// sowie ggf. Dübel.
pub fn cut(m: &ColoredMesh, o: &CutOpts, group: usize) -> (Vec<Part>, Vec<String>) {
    let mut warnings = Vec::new();
    let n = if o.normal.norm() > 0.0 { o.normal.normalized() } else { V3::Z };
    let scale = m.verts.iter().fold(0.0f64, |s, p| s.max(p.x.abs()).max(p.y.abs()).max(p.z.abs())).max(1.0);
    // Ebene minimal verschieben, falls Punkte (fast) darauf liegen
    let mut origin = o.origin;
    let mut sd: Vec<f64> = Vec::new();
    for attempt in 0..8 {
        sd = m.verts.iter().map(|p| (*p - origin).dot(n)).collect();
        let eps = scale * 1e-9;
        if sd.iter().all(|s| s.abs() > eps) {
            break;
        }
        origin = origin + n * (scale * 3.1e-8 * (attempt as f64 + 1.0));
    }
    let mut verts = m.verts.clone();
    let mut cutp: HashMap<(u32, u32), u32> = HashMap::new();
    let mut up: (Vec<[u32; 3]>, Vec<u8>) = (Vec::new(), Vec::new());
    let mut lo: (Vec<[u32; 3]>, Vec<u8>) = (Vec::new(), Vec::new());
    let mut up_edges: Vec<(u32, u32)> = Vec::new();
    let mut edge_col: HashMap<(u32, u32), u8> = HashMap::new();
    let mut point = |a: u32, b: u32, verts: &mut Vec<V3>| -> u32 {
        let key = (a.min(b), a.max(b));
        if let Some(&i) = cutp.get(&key) {
            return i;
        }
        let (p, q) = (verts[key.0 as usize], verts[key.1 as usize]);
        let (sp, sq) = (sd[key.0 as usize], sd[key.1 as usize]);
        let t = sp / (sp - sq);
        verts.push(p + (q - p) * t);
        let i = (verts.len() - 1) as u32;
        cutp.insert(key, i);
        i
    };
    for (ti, t) in m.tris.iter().enumerate() {
        let c = m.col[ti];
        let pos: Vec<bool> = t.iter().map(|&v| sd[v as usize] > 0.0).collect();
        let np = pos.iter().filter(|&&x| x).count();
        if np == 3 {
            up.0.push(*t);
            up.1.push(c);
            continue;
        }
        if np == 0 {
            lo.0.push(*t);
            lo.1.push(c);
            continue;
        }
        // einzelne Ecke auf einer Seite finden
        let lone_pos = np == 1;
        let k = (0..3).find(|&k| pos[k] == lone_pos).unwrap();
        let (a, b, cc) = (t[k], t[(k + 1) % 3], t[(k + 2) % 3]);
        let p = point(a, b, &mut verts);
        let q = point(cc, a, &mut verts);
        let (lone, other) = if lone_pos { (&mut up, &mut lo) } else { (&mut lo, &mut up) };
        lone.0.push([a, p, q]);
        lone.1.push(c);
        other.0.push([p, b, cc]);
        other.1.push(c);
        other.0.push([p, cc, q]);
        other.1.push(c);
        // Randkante der oberen Hälfte (in deren Umlaufrichtung)
        let e = if lone_pos { (p, q) } else { (q, p) };
        up_edges.push(e);
        edge_col.insert(e, c);
    }
    if up.0.is_empty() || lo.0.is_empty() {
        warnings.push(format!("{}: Die Schnittebene trifft das Objekt nicht", m.name));
        let p = Part::from_soup(m.name.clone(), &m.verts, &m.tris, &m.col, most_common(&m.col), group, PartKind::Body);
        return (vec![p], warnings);
    }
    let (loops, broken) = chain_loops(&up_edges);
    if broken > 0 {
        warnings.push(format!("{}: Schnittkontur nicht geschlossen ({broken} offene Stellen) – Netz hat Lücken", m.name));
    }
    // Ebenenbasis: e1 x e2 = n
    let (e1, e2) = n.basis();
    let to2 = |p: V3| -> P2 { [(p - origin).dot(e1), (p - origin).dot(e2)] };
    let to3 = |q: P2| -> V3 { origin + e1 * q[0] + e2 * q[1] };
    // Schleifen der oberen Hälfte: die untere Kappe enthält dieselbe Richtung (Normale +n)
    let rings: Vec<Vec<P2>> = loops.iter().map(|l| l.iter().map(|&i| to2(verts[i as usize])).collect()).collect();
    let areas: Vec<f64> = rings.iter().map(|r| poly::signed_area(r)).collect();
    // Außenringe gegen den Uhrzeigersinn (Fläche > 0), Löcher < 0
    let outers: Vec<usize> = (0..rings.len()).filter(|&i| areas[i] > 0.0).collect();
    let mut holes_of: HashMap<usize, Vec<usize>> = HashMap::new();
    for h in (0..rings.len()).filter(|&i| areas[i] < 0.0) {
        let p = rings[h][0];
        let best = outers
            .iter()
            .copied()
            .filter(|&o| poly::point_in(&rings[o], p))
            .min_by(|&a, &b| areas[a].partial_cmp(&areas[b]).unwrap());
        match best {
            Some(o) => holes_of.entry(o).or_default().push(h),
            None => warnings.push(format!("{}: Schnittkontur mit falscher Orientierung übersprungen", m.name)),
        }
    }
    let cap_col = most_common(&edge_col.values().copied().collect::<Vec<_>>());
    let mut dowels: Vec<Part> = Vec::new();
    let r_conn = ring_radius(o.shape, o.size);
    let mut placed = 0usize;
    let mut skipped = 0usize;
    for &oi in &outers {
        let holes = holes_of.get(&oi).cloned().unwrap_or_default();
        // Verbinderpositionen
        let mut centers: Vec<P2> = Vec::new();
        if o.conn != ConnKind::None {
            let mut segs: Vec<(P2, P2)> = Vec::new();
            for &ri in std::iter::once(&oi).chain(holes.iter()) {
                let r = &rings[ri];
                for i in 0..r.len() {
                    segs.push((r[i], r[(i + 1) % r.len()]));
                }
            }
            let (mut mn, mut mx) = ([f64::MAX; 2], [f64::MIN; 2]);
            for p in &rings[oi] {
                for k in 0..2 {
                    mn[k] = mn[k].min(p[k]);
                    mx[k] = mx[k].max(p[k]);
                }
            }
            let need = r_conn + o.clearance + 1.2;
            let steps = 48usize;
            let mut cand: Vec<(f64, P2)> = Vec::new();
            for iy in 0..steps {
                for ix in 0..steps {
                    let p = [
                        mn[0] + (mx[0] - mn[0]) * (ix as f64 + 0.5) / steps as f64,
                        mn[1] + (mx[1] - mn[1]) * (iy as f64 + 0.5) / steps as f64,
                    ];
                    if !poly::point_in(&rings[oi], p) || holes.iter().any(|&h| poly::point_in(&rings[h], p)) {
                        continue;
                    }
                    let d = segs.iter().fold(f64::MAX, |m, s| m.min(poly::seg_dist(p, s.0, s.1)));
                    if d >= need {
                        cand.push((d, p));
                    }
                }
            }
            cand.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap());
            let area = areas[oi] - holes.iter().map(|&h| -areas[h]).sum::<f64>();
            let want = if o.count > 0 { o.count } else if area > 25.0 * r_conn * r_conn * 4.0 { 2 } else { 1 };
            let sep = (4.0 * r_conn).max(0.35 * area.sqrt());
            for (_, p) in &cand {
                if centers.len() >= want {
                    break;
                }
                if centers.iter().all(|c| ((c[0] - p[0]).powi(2) + (c[1] - p[1]).powi(2)).sqrt() >= sep) {
                    centers.push(*p);
                }
            }
            if centers.is_empty() {
                skipped += 1;
            }
        }
        // Tiefe je Verbinder prüfen (Material auf beiden Seiten)
        let mut conns: Vec<(P2, f64)> = Vec::new();
        for c in &centers {
            let pc = to3(*c);
            let ring = conn_ring(o.shape, o.size + 2.0 * o.clearance);
            let mut up_t = f64::MAX;
            let mut lo_t = f64::MAX;
            for q in std::iter::once([0.0, 0.0]).chain(ring.iter().copied()) {
                let p = pc + e1 * q[0] + e2 * q[1];
                up_t = up_t.min(ray_hit(&verts, &up.0, p, n));
                lo_t = lo_t.min(ray_hit(&verts, &lo.0, p, -n));
            }
            let avail = match o.conn {
                ConnKind::Peg => up_t,
                _ => up_t.min(lo_t),
            } - 0.8;
            let d = o.depth.min(avail);
            if d >= 1.5 {
                conns.push((*c, d));
            } else {
                skipped += 1;
            }
        }
        placed += conns.len();
        // Kappen triangulieren: untere (Normale +n), obere (Normale -n)
        for side in 0..2 {
            let lower = side == 0;
            // Lochradius: Zapfen-Seite (unten bei Peg) ohne Spiel, sonst mit Spiel
            let plain = lower && o.conn == ConnKind::Peg;
            let size = if plain { o.size } else { o.size + 2.0 * o.clearance };
            let mut rs: Vec<Vec<P2>> = vec![rings[oi].clone()];
            let mut ids: Vec<u32> = loops[oi].clone();
            for &h in &holes {
                rs.push(rings[h].clone());
                ids.extend_from_slice(&loops[h]);
            }
            let mut conn_rings: Vec<(Vec<u32>, f64)> = Vec::new();
            for (c, d) in &conns {
                // Loch im Uhrzeigersinn
                let mut r: Vec<P2> = conn_ring(o.shape, size).iter().map(|q| [c[0] + q[0], c[1] + q[1]]).collect();
                r.reverse();
                let start = verts.len() as u32;
                for q in &r {
                    verts.push(to3(*q));
                }
                let rid: Vec<u32> = (start..verts.len() as u32).collect();
                ids.extend_from_slice(&rid);
                rs.push(r);
                conn_rings.push((rid, *d));
            }
            let tri = poly::triangulate(&rs);
            let target = if lower { &mut lo } else { &mut up };
            for t in tri {
                let (a, b, c) = (ids[t[0]], ids[t[1]], ids[t[2]]);
                // gegen den Uhrzeigersinn in (e1,e2) = Normale +n
                target.0.push(if lower { [a, b, c] } else { [a, c, b] });
                target.1.push(cap_col);
            }
            // Verbinder: Kappe enthält Kanten ring[k]->ring[k+1] (unten) bzw. umgekehrt (oben)
            for (rid, d) in conn_rings {
                let ring: Vec<u32> = if lower { rid } else { rid.into_iter().rev().collect() };
                let (dir, len) = match (o.conn, lower) {
                    (ConnKind::Peg, true) => (n, d - 0.3),       // Zapfen nach oben
                    (ConnKind::Peg, false) => (n, d),            // Loch nach oben
                    (_, true) => (-n, d),                        // Loch nach unten
                    (_, false) => (n, d),
                };
                tube(&mut verts, &ring, dir * len, &mut target.0, &mut target.1, cap_col);
            }
        }
        if o.conn == ConnKind::Dowel {
            for (c, d) in &conns {
                let pc = to3(*c);
                let len = 2.0 * d - 0.6;
                let mut dv: Vec<V3> = Vec::new();
                let ring2 = conn_ring(o.shape, o.size);
                let bot: Vec<V3> = ring2.iter().map(|q| pc - n * (len * 0.5) + e1 * q[0] + e2 * q[1]).collect();
                dv.extend(bot.iter().copied());
                let ring: Vec<u32> = (0..bot.len() as u32).collect();
                let mut dt: Vec<[u32; 3]> = Vec::new();
                let mut dc: Vec<u8> = Vec::new();
                // Boden mit Normale -n: Ring liegt gegen den Uhrzeigersinn um +n -> umgekehrt fächern
                for k in 1..ring.len() - 1 {
                    dt.push([ring[0], ring[k + 1], ring[k]]);
                    dc.push(cap_col);
                }
                // Boden enthält ring[k+1]->ring[k]; die Röhre braucht Kappenkanten ring[k]->ring[k+1]
                let rev: Vec<u32> = ring.iter().rev().copied().collect();
                tube(&mut dv, &rev, n * len, &mut dt, &mut dc, cap_col);
                dowels.push(Part::from_soup(format!("{} Dübel", m.name), &dv, &dt, &dc, cap_col, group, PartKind::Dowel));
            }
        }
    }
    if o.conn != ConnKind::None && skipped > 0 {
        warnings.push(format!("{skipped} Verbinder passten nicht (Schnittfläche zu klein oder zu dünn)"));
    }
    let _ = placed;
    let mut parts = vec![
        Part::from_soup(format!("{} oben", m.name), &verts, &up.0, &up.1, most_common(&up.1), group, PartKind::CutPart),
        Part::from_soup(format!("{} unten", m.name), &verts, &lo.0, &lo.1, most_common(&lo.1), group, PartKind::CutPart),
    ];
    parts.extend(dowels);
    (parts, warnings)
}

/// Röhre von einem Ring (Kanten ring[k]->ring[k+1] gehören zur angrenzenden Fläche)
/// entlang `off` mit Abschlussdeckel.
fn tube(verts: &mut Vec<V3>, ring: &[u32], off: V3, tris: &mut Vec<[u32; 3]>, col: &mut Vec<u8>, c: u8) {
    let n = ring.len();
    let start = verts.len() as u32;
    for &i in ring {
        let p = verts[i as usize] + off;
        verts.push(p);
    }
    let far = |k: usize| start + (k % n) as u32;
    for k in 0..n {
        let (a, b) = (ring[k], ring[(k + 1) % n]);
        let (a2, b2) = (far(k), far(k + 1));
        tris.push([b, a, a2]);
        tris.push([b, a2, b2]);
        col.push(c);
        col.push(c);
    }
    // Deckel: enthält b2->a2 (Gegenkante zur Wand a2->b2)
    for k in 1..n - 1 {
        tris.push([far(0), far(k + 1), far(k)]);
        col.push(c);
    }
}

fn most_common(c: &[u8]) -> u8 {
    let mut cnt: HashMap<u8, usize> = HashMap::new();
    for &x in c {
        *cnt.entry(x).or_default() += 1;
    }
    cnt.into_iter().max_by_key(|x| (x.1, std::cmp::Reverse(x.0))).map(|x| x.0).unwrap_or(1)
}

/// Netzvolumen
pub fn volume(verts: &[V3], tris: &[[u32; 3]]) -> f64 {
    tris.iter().map(|t| verts[t[0] as usize].dot(verts[t[1] as usize].cross(verts[t[2] as usize])) / 6.0).sum()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sharpen_straightens_staircase() {
        // ebenes Raster 12×12, Farbgrenze entlang der Diagonale x + y = 12 als Treppe
        let n = 12usize;
        let mut verts = Vec::new();
        for j in 0..=n {
            for i in 0..=n {
                verts.push(v3(i as f64, j as f64, 0.0));
            }
        }
        let id = |i: usize, j: usize| (j * (n + 1) + i) as u32;
        let (mut tris, mut col) = (Vec::new(), Vec::new());
        for j in 0..n {
            for i in 0..n {
                // Quadrat in zwei Dreiecke, Teilung quer zur Grenze (Treppe entsteht)
                for t in [[id(i, j), id(i + 1, j), id(i + 1, j + 1)], [id(i, j), id(i + 1, j + 1), id(i, j + 1)]] {
                    let cx: f64 = t.iter().map(|&v| verts[v as usize].x + verts[v as usize].y).sum::<f64>() / 3.0;
                    tris.push(t);
                    col.push(if cx < 12.0 { 2u8 } else { 1u8 });
                }
            }
        }
        let dev = |m: &ColoredMesh| -> f64 {
            let em = undirected_map(&m.tris);
            let mut d: f64 = 0.0;
            for (&(a, b), l) in &em {
                if l.len() == 2 && m.col[l[0] as usize] != m.col[l[1] as usize] {
                    for v in [a, b] {
                        let p = m.verts[v as usize];
                        if p.x > 1.5 && p.x < 10.5 && p.y > 1.5 && p.y < 10.5 {
                            d = d.max(((p.x + p.y - 12.0) / 2f64.sqrt()).abs());
                        }
                    }
                }
            }
            d
        };
        let mut m = ColoredMesh { name: "t".into(), verts, tris, col };
        let before = dev(&m);
        let n_split = sharpen_boundaries(&mut m, 0.5);
        let after = dev(&m);
        assert!(n_split > 0);
        eprintln!("Treppe vorher {before:.3}, nachher {after:.3}");
        assert!(after < before * 0.6, "vorher {before:.3}, nachher {after:.3}");
        // keine Kante mit mehr als zwei Dreiecken, keine Risse im Inneren
        let em = undirected_map(&m.tris);
        assert!(em.values().all(|l| l.len() <= 2));
        // gleiche Fläche wie vorher (nichts verloren, nichts doppelt)
        let area: f64 = m.tris.iter().map(|t| tri_normal_area(&m.verts, t).norm()).sum();
        assert!((area - 144.0).abs() < 1e-6, "{area}");
    }

    /// Würfel aus 2x2 Unterteilung je Seite, Oberseite teilweise rot (Farbe 2)
    fn cube(nsub: usize, s: f64) -> ColoredMesh {
        let mut m = ColoredMesh { name: "W".into(), ..Default::default() };
        let mut idx: HashMap<[i64; 3], u32> = HashMap::new();
        let mut vid = |m: &mut ColoredMesh, p: [i64; 3]| -> u32 {
            *idx.entry(p).or_insert_with(|| {
                m.verts.push(v3(p[0] as f64 * s / nsub as f64, p[1] as f64 * s / nsub as f64, p[2] as f64 * s / nsub as f64));
                (m.verts.len() - 1) as u32
            })
        };
        let n = nsub as i64;
        for axis in 0..3 {
            for side in 0..2 {
                for i in 0..n {
                    for j in 0..n {
                        let mk = |a: i64, b: i64| -> [i64; 3] {
                            let mut p = [0i64; 3];
                            p[axis] = side * n;
                            p[(axis + 1) % 3] = a;
                            p[(axis + 2) % 3] = b;
                            p
                        };
                        let q = [mk(i, j), mk(i + 1, j), mk(i + 1, j + 1), mk(i, j + 1)];
                        let ids: Vec<u32> = q.iter().map(|p| vid(&mut m, *p)).collect();
                        let (t1, t2) = if side == 1 { ([ids[0], ids[1], ids[2]], [ids[0], ids[2], ids[3]]) } else { ([ids[0], ids[2], ids[1]], [ids[0], ids[3], ids[2]]) };
                        // Oberseite (z max), mittlere Felder rot
                        let red = axis == 2 && side == 1 && i >= 1 && i < n - 1 && j >= 1 && j < n - 1;
                        let c = if red { 2 } else { 1 };
                        m.tris.push(t1);
                        m.tris.push(t2);
                        m.col.push(c);
                        m.col.push(c);
                    }
                }
            }
        }
        m
    }

    /// Geschlossener Block 40×40 mit Höhenfeld oben; `h(x,y)` relativ zu z=10.
    /// Dreiecke mit einem Punkt abseits der Grundhöhe bekommen Farbe 2.
    fn heightfield(h: impl Fn(f64, f64) -> f64) -> ColoredMesh {
        let n = 20usize;
        let st = 2.0;
        let mut verts = Vec::new();
        for j in 0..=n {
            for i in 0..=n {
                let (x, y) = (i as f64 * st, j as f64 * st);
                verts.push(v3(x, y, 10.0 + h(x, y)));
            }
        }
        let top = |i: usize, j: usize| (j * (n + 1) + i) as u32;
        let mut tris = Vec::new();
        let mut col = Vec::new();
        for j in 0..n {
            for i in 0..n {
                let (a, b, c, d) = (top(i, j), top(i + 1, j), top(i + 1, j + 1), top(i, j + 1));
                for t in [[a, b, c], [a, c, d]] {
                    let raised = t.iter().any(|&k| (verts[k as usize].z - 10.0).abs() > 1e-9);
                    tris.push(t);
                    col.push(if raised { 2 } else { 1 });
                }
            }
        }
        // Boden und Seiten: Randring oben mit Bodenpunkten verbinden
        let mut ring = Vec::new();
        for i in 0..n { ring.push(top(i, 0)); }
        for j in 0..n { ring.push(top(n, j)); }
        for i in (1..=n).rev() { ring.push(top(i, n)); }
        for j in (1..=n).rev() { ring.push(top(0, j)); }
        let b0 = verts.len() as u32;
        for &k in &ring {
            let p = verts[k as usize];
            verts.push(v3(p.x, p.y, 0.0));
        }
        let m = ring.len();
        for i in 0..m {
            let j = (i + 1) % m;
            tris.push([ring[i], b0 + i as u32, b0 + j as u32]);
            tris.push([ring[i], b0 + j as u32, ring[j]]);
            col.push(1);
            col.push(1);
        }
        for i in 1..m - 1 {
            tris.push([b0, b0 + i as u32 + 1, b0 + i as u32]);
            col.push(1);
        }
        let mut cm = ColoredMesh { name: "hf".into(), verts, tris, col };
        orient(&mut cm);
        cm
    }

    fn split_auto(m: &ColoredMesh) -> Vec<Part> {
        let reg = regions(m);
        let extract: Vec<bool> = reg.list.iter().map(|r| r.color != reg.base_color).collect();
        let (parts, _) = split_by_color(m, &reg, &extract, &ColorSplitOpts { cap: CapMethod::Auto, depth: 1.5, merge_same_color: false }, 0);
        let v0 = volume(&m.verts, &m.tris);
        let s: f64 = parts.iter().map(|p| p.volume).sum();
        for p in &parts {
            assert_eq!(p.open_edges, 0, "{}", p.name);
            assert!(p.volume > 0.0, "{} {}", p.name, p.volume);
        }
        assert!((s - v0).abs() < 1e-6 * v0.max(1.0) + 1e-6, "{s} vs {v0}");
        parts
    }

    #[test]
    fn raised_bezel_is_spanned() {
        // Erhebung 1,6 mm (Rahmen einer Anzeige): Teil = genau die Erhebung
        let m = heightfield(|x, y| if (14.0..=26.0).contains(&x) && (14.0..=26.0).contains(&y) { 1.6 } else { 0.0 });
        let parts = split_auto(&m);
        let bump = parts.iter().find(|p| p.kind == PartKind::ColorPart).unwrap();
        // Pyramidenstumpf: oben 12×12, unten 16×16, Höhe 1,6
        let want = 1.6 / 3.0 * (144.0 + 256.0 + (144.0f64 * 256.0).sqrt());
        assert!((bump.volume - want).abs() < 0.02 * want, "{} vs {want}", bump.volume);
    }

    #[test]
    fn flush_face_gets_minimum_thickness() {
        // Ring mit Fläche auf Grundhöhe in der Mitte: Haut wird abgesenkt
        let m = heightfield(|x, y| {
            let r = ((x - 20.0).powi(2) + (y - 20.0).powi(2)).sqrt();
            if (6.0..=8.0).contains(&r) { 1.0 } else { 0.0 }
        });
        let parts = split_auto(&m);
        assert!(parts.iter().filter(|p| p.kind == PartKind::ColorPart).all(|p| p.volume > 5.0));
    }

    #[test]
    fn pocket_gets_shell() {
        // Vertiefung 1,6 mm: keine gespannte Haut (wäre negatives Volumen)
        let m = heightfield(|x, y| if (14.0..=26.0).contains(&x) && (14.0..=26.0).contains(&y) { -1.6 } else { 0.0 });
        split_auto(&m);
    }

    #[test]
    fn cube_is_closed() {
        let mut m = cube(4, 20.0);
        orient(&mut m);
        let v = volume(&m.verts, &m.tris);
        assert!((v - 8000.0).abs() < 1e-6, "{v}");
    }

    #[test]
    fn color_split_depth_and_membrane() {
        for cap in [CapMethod::Depth, CapMethod::Membrane, CapMethod::Auto] {
            let mut m = cube(4, 20.0);
            orient(&mut m);
            let reg = regions(&m);
            assert_eq!(reg.base_color, 1);
            let extract: Vec<bool> = reg.list.iter().map(|r| r.color != reg.base_color).collect();
            let (parts, _w) = split_by_color(&m, &reg, &extract, &ColorSplitOpts { cap, depth: 1.0, merge_same_color: false }, 0);
            assert_eq!(parts.len(), 2, "{cap:?}");
            let total: f64 = parts.iter().map(|p| p.volume).sum();
            for p in &parts {
                assert_eq!(p.open_edges, 0, "{cap:?} {}", p.name);
            }
            assert!((total - 8000.0).abs() < 1e-6, "{cap:?} {total}");
            let red = parts.iter().find(|p| p.color == 2).unwrap();
            if cap != CapMethod::Membrane {
                // 10x10 Fläche, 1 mm tief
                assert!((red.volume - 100.0).abs() < 1e-6, "{cap:?} {}", red.volume);
            }
        }
    }

    #[test]
    fn color_part_connectors() {
        for (side, shape, depth) in [(ConnSide::Part, ConnShape::Cylinder, 1.0), (ConnSide::Body, ConnShape::Square, 3.0), (ConnSide::Part, ConnShape::Triangle, 2.0)] {
            let mut m = cube(4, 20.0);
            orient(&mut m);
            let reg = regions(&m);
            let extract: Vec<bool> = reg.list.iter().map(|r| r.color != reg.base_color).collect();
            let (mut parts, _w) = split_by_color(&m, &reg, &extract, &ColorSplitOpts { cap: CapMethod::Depth, depth, merge_same_color: false }, 0);
            let v0: Vec<f64> = parts.iter().map(|p| p.volume).collect();
            let w = add_color_connectors(&mut parts, &ColorConnOpts { shape, side, area_pct: 10.0, tol: 0.2 });
            assert!(w.is_empty(), "{side:?} {w:?}");
            for (p, v) in parts.iter().zip(&v0) {
                assert_eq!(p.open_edges, 0, "{side:?} {shape:?} {} offen", p.name);
                eprintln!("{side:?} {shape:?} {}: {:.1} -> {:.1}", p.name, v, p.volume);
            }
            let (owner, hole) = match side {
                ConnSide::Part => (parts.iter().find(|p| p.kind == PartKind::ColorPart).unwrap(), parts.iter().find(|p| p.kind == PartKind::Body).unwrap()),
                ConnSide::Body => (parts.iter().find(|p| p.kind == PartKind::Body).unwrap(), parts.iter().find(|p| p.kind == PartKind::ColorPart).unwrap()),
            };
            let o0 = if owner.kind == PartKind::Body { v0[parts.iter().position(|p| p.kind == PartKind::Body).unwrap()] } else { v0[parts.iter().position(|p| p.kind == PartKind::ColorPart).unwrap()] };
            let h0 = if hole.kind == PartKind::Body { v0[parts.iter().position(|p| p.kind == PartKind::Body).unwrap()] } else { v0[parts.iter().position(|p| p.kind == PartKind::ColorPart).unwrap()] };
            assert!(owner.volume > o0 + 1.0, "Zapfen fehlt");
            assert!(hole.volume < h0 - 1.0, "Loch fehlt");
            // Loch etwas größer als der Zapfen (Spiel)
            assert!((h0 - hole.volume) > (owner.volume - o0) * 0.95);
        }
    }

    #[test]
    fn smoothing_keeps_volume() {
        let data = crate::testgen::painted_3mf();
        let p = std::env::temp_dir().join("m2s_smooth.3mf");
        std::fs::write(&p, &data).unwrap();
        let (objs, _, _) = crate::paint::load_colored(&p).unwrap();
        let mut m = objs[0].clone();
        clean_specks(&mut m, 2.0);
        let v0 = volume(&m.verts, &m.tris);
        let n = smooth_boundaries(&mut m, 0.6);
        assert!(n > 0);
        let v1 = volume(&m.verts, &m.tris);
        // Punkte bleiben auf der Oberfläche: Volumen ändert sich kaum
        assert!((v1 - v0).abs() < 0.01 * v0, "{v0} {v1}");
        let reg = regions(&m);
        let extract: Vec<bool> = reg.list.iter().map(|r| r.color != reg.base_color).collect();
        let (parts, _) = split_by_color(&m, &reg, &extract, &ColorSplitOpts { cap: CapMethod::Auto, depth: 1.5, merge_same_color: true }, 0);
        for p in &parts {
            assert_eq!(p.open_edges, 0, "{}", p.name);
        }
    }

    #[test]
    fn plane_cut_with_connectors() {
        for conn in [ConnKind::None, ConnKind::Peg, ConnKind::Dowel] {
            for shape in [ConnShape::Cylinder, ConnShape::Square, ConnShape::Triangle] {
                let mut m = cube(4, 20.0);
                orient(&mut m);
                let o = CutOpts {
                    origin: v3(10.0, 10.0, 10.0),
                    normal: v3(0.1, 0.0, 1.0),
                    conn,
                    shape,
                    size: 4.0,
                    depth: 4.0,
                    clearance: 0.15,
                    count: 0,
                };
                let (parts, w) = cut(&m, &o, 0);
                for p in &parts {
                    assert_eq!(p.open_edges, 0, "{conn:?} {shape:?} {} {w:?}", p.name);
                    assert!(p.volume > 0.0, "{conn:?} {} {}", p.name, p.volume);
                }
                let halves: f64 = parts.iter().filter(|p| p.kind == PartKind::CutPart).map(|p| p.volume).sum();
                if conn == ConnKind::None {
                    assert!((halves - 8000.0).abs() < 1e-6, "{halves}");
                } else {
                    assert!(halves < 8000.0 + 200.0 && halves > 7000.0, "{halves}");
                    let nd = parts.iter().filter(|p| p.kind == PartKind::Dowel).count();
                    if conn == ConnKind::Dowel {
                        assert!(nd >= 1);
                    }
                }
            }
        }
    }

    #[test]
    fn cut_ring_with_hole() {
        // Rohr: Zylinder mit Loch (aus testgen)
        let raw = crate::testgen::box_with_hole(30.0, 20.0, 10.0, 4.0, 32);
        let (mut cm, _) = crate::paint::refine(&raw);
        orient(&mut cm);
        let v0 = volume(&cm.verts, &cm.tris);
        let o = CutOpts { origin: v3(0.0, 0.0, 5.0), normal: V3::Z, ..Default::default() };
        let (parts, _) = cut(&cm, &o, 0);
        let s: f64 = parts.iter().map(|p| p.volume).sum();
        for p in &parts {
            assert_eq!(p.open_edges, 0);
        }
        assert!((s - v0).abs() < 1e-6 * v0.abs().max(1.0), "{s} {v0}");
    }
}


#[cfg(test)]
mod painted_tests {
    use super::*;

    #[test]
    fn painted_sphere_split() {
        let data = crate::testgen::painted_3mf();
        let dir = std::env::temp_dir().join("m2s_painted.3mf");
        std::fs::write(&dir, &data).unwrap();
        let (objs, colors, warns) = crate::paint::load_colored(&dir).unwrap();
        assert_eq!(colors.len(), 4);
        assert!(warns.iter().all(|w| !w.contains("unlesbar")), "{warns:?}");
        let mut m = objs[0].clone();
        let v0 = volume(&m.verts, &m.tris);
        clean_specks(&mut m, 0.5);
        let reg = regions(&m);
        let extract: Vec<bool> = reg.list.iter().map(|r| r.color != reg.base_color).collect();
        for cap in [CapMethod::Auto, CapMethod::Membrane, CapMethod::Depth] {
            let (parts, w) = split_by_color(&m, &reg, &extract, &ColorSplitOpts { cap, depth: 1.5, merge_same_color: true }, 0);
            let s: f64 = parts.iter().map(|p| p.volume).sum();
            for p in &parts {
                assert_eq!(p.open_edges, 0, "{cap:?} {} {w:?}", p.name);
                assert!(p.volume > 0.0, "{cap:?} {} {}", p.name, p.volume);
            }
            assert!((s - v0).abs() < 1e-6 * v0, "{cap:?} {s} {v0}");
            let mut cols: Vec<u8> = parts.iter().map(|p| p.color).collect();
            cols.sort();
            assert_eq!(cols, vec![1, 2, 3], "{cap:?}");
        }
    }
}

