//! Überlappende Körper vereinigen (exakt, ohne Raster).
//!
//! 1. Dreieckspaare verschiedener Körper, die sich schneiden, suchen.
//! 2. Schnittpunkte Kante×Dreieck berechnen und global auf den Kanten einfügen
//!    (beide Nachbardreiecke einer Kante bekommen denselben Punkt – keine Risse).
//! 3. Betroffene Dreiecke entlang der Schnittlinien neu unterteilen.
//! 4. Teilflächen, die im jeweils anderen Körper liegen, entfernen.
//! Hohlräume (nach innen orientierte Schalen) bleiben unberührt.

use crate::math::{v3, V3};
use std::collections::HashMap;

fn tri_pts(v: &[V3], t: &[u32; 3]) -> (V3, V3, V3) {
    (v[t[0] as usize], v[t[1] as usize], v[t[2] as usize])
}

/// Strecke p→q gegen Dreieck (a,b,c): Parameter s auf der Strecke, falls Treffer im Inneren.
fn seg_tri(p: V3, q: V3, a: V3, b: V3, c: V3) -> Option<f64> {
    let d = q - p;
    let e1 = b - a;
    let e2 = c - a;
    let h = d.cross(e2);
    let det = e1.dot(h);
    let scale = e1.norm() * e2.norm() * d.norm();
    if det.abs() <= 1e-12 * scale {
        return None; // parallel / koplanar
    }
    let f = 1.0 / det;
    let s = p - a;
    let u = f * s.dot(h);
    let qv = s.cross(e1);
    let w = f * d.dot(qv);
    let t = f * e2.dot(qv);
    let eps = 1e-12;
    if u < -eps || w < -eps || u + w > 1.0 + eps || t < -eps || t > 1.0 + eps {
        return None;
    }
    Some(t.clamp(0.0, 1.0))
}

/// Komponenten über gemeinsame Kanten.
fn shells(tris: &[[u32; 3]]) -> Vec<u32> {
    let mut parent: Vec<u32> = (0..tris.len() as u32).collect();
    fn find(p: &mut [u32], mut x: u32) -> u32 {
        while p[x as usize] != x {
            p[x as usize] = p[p[x as usize] as usize];
            x = p[x as usize];
        }
        x
    }
    let mut edge: HashMap<(u32, u32), u32> = HashMap::new();
    for (i, t) in tris.iter().enumerate() {
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            let key = (a.min(b), a.max(b));
            match edge.get(&key) {
                Some(&j) => {
                    let (ri, rj) = (find(&mut parent, i as u32), find(&mut parent, j));
                    if ri != rj {
                        parent[ri as usize] = rj;
                    }
                }
                None => {
                    edge.insert(key, i as u32);
                }
            }
        }
    }
    let mut id: HashMap<u32, u32> = HashMap::new();
    (0..tris.len() as u32)
        .map(|i| {
            let r = find(&mut parent, i);
            let n = id.len() as u32;
            *id.entry(r).or_insert(n)
        })
        .collect()
}

fn inside_shell(v: &[V3], tris: &[[u32; 3]], list: &[u32], p: V3) -> bool {
    let dirs = [v3(0.5773, 0.5774, 0.5775), v3(-0.6011, 0.3123, 0.7357), v3(0.2113, -0.8712, 0.4431)];
    let mut votes = 0;
    for d in dirs {
        let d = d.normalized();
        let mut cnt = 0;
        for &t in list {
            let (a, b, c) = tri_pts(v, &tris[t as usize]);
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
            let w = d.dot(q) * inv;
            if w < 0.0 || u + w > 1.0 {
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

// ---------------------------------------------------------------------------
// Ebene Unterteilung eines Dreiecks

fn cross2(a: (f64, f64), b: (f64, f64), c: (f64, f64)) -> f64 {
    (b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0)
}

fn area2(poly: &[usize], p: &[(f64, f64)]) -> f64 {
    let mut s = 0.0;
    for i in 0..poly.len() {
        let (a, b) = (p[poly[i]], p[poly[(i + 1) % poly.len()]]);
        s += a.0 * b.1 - b.0 * a.1;
    }
    s * 0.5
}

fn point_in_poly(pt: (f64, f64), poly: &[usize], p: &[(f64, f64)]) -> bool {
    let mut inside = false;
    let n = poly.len();
    for i in 0..n {
        let (a, b) = (p[poly[i]], p[poly[(i + 1) % n]]);
        if (a.1 > pt.1) != (b.1 > pt.1) {
            let x = a.0 + (pt.1 - a.1) / (b.1 - a.1) * (b.0 - a.0);
            if x > pt.0 {
                inside = !inside;
            }
        }
    }
    inside
}

/// Ohrenschneiden eines einfachen Polygons (gegen den Uhrzeigersinn).
fn ear_clip(poly: &[usize], p: &[(f64, f64)]) -> Vec<[usize; 3]> {
    let mut idx: Vec<usize> = poly.to_vec();
    let mut out = Vec::new();
    let mut guard = 0;
    while idx.len() > 3 && guard < poly.len() * poly.len() + 10 {
        guard += 1;
        let m = idx.len();
        let mut cut = None;
        for i in 0..m {
            let (ia, ib, ic) = (idx[(i + m - 1) % m], idx[i], idx[(i + 1) % m]);
            let (a, b, c) = (p[ia], p[ib], p[ic]);
            if cross2(a, b, c) <= 1e-18 {
                continue;
            }
            let mut ok = true;
            for &j in &idx {
                if j == ia || j == ib || j == ic {
                    continue;
                }
                let q = p[j];
                if q == a || q == b || q == c {
                    continue;
                }
                if cross2(a, b, q) >= 0.0 && cross2(b, c, q) >= 0.0 && cross2(c, a, q) >= 0.0 {
                    ok = false;
                    break;
                }
            }
            if ok {
                cut = Some(i);
                break;
            }
        }
        let i = match cut {
            Some(i) => i,
            None => {
                // Notlösung: konvexste Ecke abschneiden
                let mut best = (0, f64::MIN);
                for i in 0..m {
                    let c = cross2(p[idx[(i + m - 1) % m]], p[idx[i]], p[idx[(i + 1) % m]]);
                    if c > best.1 {
                        best = (i, c);
                    }
                }
                best.0
            }
        };
        let m = idx.len();
        out.push([idx[(i + m - 1) % m], idx[i], idx[(i + 1) % m]]);
        idx.remove(i);
    }
    if idx.len() == 3 {
        out.push([idx[0], idx[1], idx[2]]);
    }
    out
}

/// Dreieck (Ecken 0,1,2) mit Randpunkten und inneren Schnittlinien unterteilen.
/// `nodes`: 2D-Lage aller Knoten; `boundary`: Umlauf (gegen den Uhrzeigersinn) mit
/// eingefügten Kantenpunkten; `segs`: Schnittlinien als Knotenpaare.
fn subdivide(nodes: &[(f64, f64)], boundary: &[usize], segs: &[(usize, usize)]) -> Vec<[usize; 3]> {
    let n = nodes.len();
    let mut adj: Vec<Vec<usize>> = vec![Vec::new(); n];
    let add = |a: usize, b: usize, adj: &mut Vec<Vec<usize>>| {
        if a != b && !adj[a].contains(&b) {
            adj[a].push(b);
            adj[b].push(a);
        }
    };
    for i in 0..boundary.len() {
        add(boundary[i], boundary[(i + 1) % boundary.len()], &mut adj);
    }
    for &(a, b) in segs {
        add(a, b, &mut adj);
    }
    // Nachbarn nach Winkel sortieren
    for (v, list) in adj.iter_mut().enumerate() {
        let o = nodes[v];
        list.sort_by(|&x, &y| {
            let ax = (nodes[x].1 - o.1).atan2(nodes[x].0 - o.0);
            let ay = (nodes[y].1 - o.1).atan2(nodes[y].0 - o.0);
            ax.partial_cmp(&ay).unwrap_or(std::cmp::Ordering::Equal)
        });
    }
    // Flächen umlaufen (Fläche jeweils links)
    let mut used: HashMap<(usize, usize), bool> = HashMap::new();
    let mut faces: Vec<Vec<usize>> = Vec::new();
    for u in 0..n {
        for &v in &adj[u] {
            if used.contains_key(&(u, v)) {
                continue;
            }
            let mut face = Vec::new();
            let (mut a, mut b) = (u, v);
            let mut guard = 0;
            loop {
                used.insert((a, b), true);
                face.push(a);
                let list = &adj[b];
                let k = list.iter().position(|&x| x == a).unwrap_or(0);
                let c = list[(k + list.len() - 1) % list.len()];
                a = b;
                b = c;
                guard += 1;
                if (a == u && b == v) || guard > 4 * n + 8 {
                    break;
                }
            }
            faces.push(face);
        }
    }
    let mut pos: Vec<Vec<usize>> = Vec::new();
    let mut holes: Vec<Vec<usize>> = Vec::new();
    for f in faces {
        let a = area2(&f, nodes);
        if a > 1e-18 {
            pos.push(f);
        } else if a < -1e-18 && !f.contains(&boundary[0]) {
            holes.push(f);
        }
    }
    // Löcher (geschlossene Schnittlinien im Inneren) an die umgebende Fläche anbinden
    for h in holes {
        let probe = nodes[h[0]];
        let mut best: Option<(usize, f64)> = None;
        for (i, f) in pos.iter().enumerate() {
            if f.contains(&h[0]) {
                continue;
            }
            if point_in_poly(probe, f, nodes) {
                let a = area2(f, nodes);
                if best.map_or(true, |(_, ba)| a < ba) {
                    best = Some((i, a));
                }
            }
        }
        if let Some((fi, _)) = best {
            let f = &pos[fi];
            // nächstes Punktpaar
            let mut bp = (0, 0, f64::MAX);
            for (i, &x) in f.iter().enumerate() {
                for (j, &y) in h.iter().enumerate() {
                    let d = (nodes[x].0 - nodes[y].0).powi(2) + (nodes[x].1 - nodes[y].1).powi(2);
                    if d < bp.2 {
                        bp = (i, j, d);
                    }
                }
            }
            let (i, j, _) = bp;
            let mut np = Vec::with_capacity(f.len() + h.len() + 2);
            np.extend_from_slice(&f[..=i]);
            for k in 0..=h.len() {
                np.push(h[(j + k) % h.len()]);
            }
            np.push(f[i]);
            np.extend_from_slice(&f[i + 1..]);
            pos[fi] = np;
        }
    }
    let mut out = Vec::new();
    for f in &pos {
        out.extend(ear_clip(f, nodes));
    }
    out
}

/// Vereinigen und anschließend aufräumen (Splitter an den Schnittlinien, Orientierung).
pub fn union_clean(verts: &[V3], tris: &[[u32; 3]], name: &str) -> (crate::repair::Repaired, usize) {
    let r = union(verts, tris);
    let raw = crate::io::RawMesh { name: name.to_string(), verts: r.verts, tris: r.tris, ..Default::default() };
    let ro = crate::repair::RepairOpts { remove_small: false, ..Default::default() };
    (crate::repair::repair(&raw, &ro), r.merged)
}

/// Ergebnis der Vereinigung.
pub struct UnionResult {
    pub verts: Vec<V3>,
    pub tris: Vec<[u32; 3]>,
    /// Zahl der Körper, die mit anderen verschmolzen wurden
    pub merged: usize,
    /// Ursprungsdreieck je Ausgabedreieck
    pub origin: Vec<u32>,
}

/// Überlappende, nach außen orientierte Körper eines Netzes vereinigen.
pub fn union(verts: &[V3], tris: &[[u32; 3]]) -> UnionResult {
    boolean(verts, tris, None)
}

/// Körper B von Körper A abziehen (A − B). Beide geschlossen und nach außen orientiert.
pub fn subtract(av: &[V3], at: &[[u32; 3]], bv: &[V3], bt: &[[u32; 3]]) -> UnionResult {
    let mut verts = av.to_vec();
    let mut tris = at.to_vec();
    let base = verts.len() as u32;
    verts.extend_from_slice(bv);
    tris.extend(bt.iter().map(|t| [t[0] + base, t[1] + base, t[2] + base]));
    let mut tool = vec![false; at.len()];
    tool.extend(std::iter::repeat(true).take(bt.len()));
    boolean(&verts, &tris, Some(&tool))
}

/// Gemeinsamer Kern: ohne `tool` Vereinigung aller Körper; mit `tool` (je Dreieck) werden die
/// markierten Körper von den übrigen abgezogen.
fn boolean(verts: &[V3], tris: &[[u32; 3]], tool: Option<&[bool]>) -> UnionResult {
    let shell = shells(tris);
    let ns = shell.iter().map(|&s| s as usize + 1).max().unwrap_or(0);
    let mut lists: Vec<Vec<u32>> = vec![Vec::new(); ns];
    for (i, &s) in shell.iter().enumerate() {
        lists[s as usize].push(i as u32);
    }
    // Volumen je Körper: Hohlräume (negativ) bleiben außen vor
    let vol: Vec<f64> = lists
        .iter()
        .map(|l| l.iter().map(|&t| {
            let (a, b, c) = tri_pts(verts, &tris[t as usize]);
            a.dot(b.cross(c)) / 6.0
        }).sum())
        .collect();
    let bbox: Vec<(V3, V3)> = lists
        .iter()
        .map(|l| {
            let mut mn = v3(f64::MAX, f64::MAX, f64::MAX);
            let mut mx = -mn;
            for &t in l {
                for &k in &tris[t as usize] {
                    mn = mn.min(verts[k as usize]);
                    mx = mx.max(verts[k as usize]);
                }
            }
            (mn, mx)
        })
        .collect();
    let overlap = |i: usize, j: usize| {
        let (a, b) = (bbox[i], bbox[j]);
        a.0.x <= b.1.x && b.0.x <= a.1.x && a.0.y <= b.1.y && b.0.y <= a.1.y && a.0.z <= b.1.z && b.0.z <= a.1.z
    };
    let solid: Vec<bool> = vol.iter().map(|&v| v > 0.0).collect();
    let is_tool: Vec<bool> = lists.iter().map(|l| tool.map(|tt| l.iter().any(|&t| tt[t as usize])).unwrap_or(false)).collect();
    let mut pairs_shell: Vec<(usize, usize)> = Vec::new();
    for i in 0..ns {
        for j in i + 1..ns {
            let wanted = if tool.is_some() { is_tool[i] != is_tool[j] } else { true };
            if wanted && solid[i] && solid[j] && overlap(i, j) {
                pairs_shell.push((i, j));
            }
        }
    }
    // Abziehen: Werkzeug-Körper ohne Berührung fallen weg
    let drop_tool = |t: &[[u32; 3]]| -> Vec<[u32; 3]> {
        t.iter().enumerate().filter(|(i, _)| !is_tool[shell[*i] as usize]).map(|(_, x)| *x).collect()
    };
    if pairs_shell.is_empty() {
        let t = if tool.is_some() { drop_tool(tris) } else { tris.to_vec() };
        let origin: Vec<u32> = (0..tris.len() as u32).filter(|&i| tool.is_none() || !is_tool[shell[i as usize] as usize]).collect();
        return UnionResult { verts: verts.to_vec(), tris: t, merged: 0, origin };
    }
    let involved: Vec<bool> = {
        let mut x = vec![false; ns];
        for &(i, j) in &pairs_shell {
            x[i] = true;
            x[j] = true;
        }
        x
    };

    // --- Grobsuche: Raster über alle beteiligten Dreiecke
    let mut mn = v3(f64::MAX, f64::MAX, f64::MAX);
    let mut mx = -mn;
    let mut avg = 0.0f64;
    let mut cnt = 0.0f64;
    for (i, t) in tris.iter().enumerate() {
        if !involved[shell[i] as usize] {
            continue;
        }
        let (a, b, c) = tri_pts(verts, t);
        mn = mn.min(a).min(b).min(c);
        mx = mx.max(a).max(b).max(c);
        avg += a.dist(b) + b.dist(c) + c.dist(a);
        cnt += 3.0;
    }
    let cell = (avg / cnt.max(1.0)).max((mx - mn).norm() / 200.0).max(1e-6);
    let key = |p: V3| (((p.x - mn.x) / cell) as i64, ((p.y - mn.y) / cell) as i64, ((p.z - mn.z) / cell) as i64);
    let mut grid: HashMap<(i64, i64, i64), Vec<u32>> = HashMap::new();
    for (i, t) in tris.iter().enumerate() {
        if !involved[shell[i] as usize] {
            continue;
        }
        let (a, b, c) = tri_pts(verts, t);
        let (lo, hi) = (key(a.min(b).min(c)), key(a.max(b).max(c)));
        for x in lo.0..=hi.0 {
            for y in lo.1..=hi.1 {
                for z in lo.2..=hi.2 {
                    grid.entry((x, y, z)).or_default().push(i as u32);
                }
            }
        }
    }
    let mut cand: Vec<(u32, u32)> = Vec::new();
    for list in grid.values() {
        for (a_i, &a) in list.iter().enumerate() {
            for &b in &list[a_i + 1..] {
                let (sa, sb) = (shell[a as usize], shell[b as usize]);
                if sa != sb && pairs_shell.contains(&((sa.min(sb)) as usize, (sa.max(sb)) as usize)) {
                    cand.push((a.min(b), a.max(b)));
                }
            }
        }
    }
    cand.sort_unstable();
    cand.dedup();

    // --- Schnittpunkte
    let mut out_v: Vec<V3> = verts.to_vec();
    // (Kante, Dreieck) -> Punkt
    let mut ef_point: HashMap<((u32, u32), u32), u32> = HashMap::new();
    // Punkte auf Kanten: Kante -> [(Parameter von min nach max, Punkt)]
    let mut edge_pts: HashMap<(u32, u32), Vec<(f64, u32)>> = HashMap::new();
    // je Dreieck: innere Punkte und Schnittlinien
    let mut face_pts: HashMap<u32, Vec<u32>> = HashMap::new();
    let mut face_segs: HashMap<u32, Vec<(u32, u32)>> = HashMap::new();
    for &(ta, tb) in &cand {
        let mut pts: Vec<u32> = Vec::new();
        for &(e_t, f_t) in &[(ta, tb), (tb, ta)] {
            let te = tris[e_t as usize];
            let (a, b, c) = tri_pts(verts, &tris[f_t as usize]);
            for k in 0..3 {
                let (p0, p1) = (te[k], te[(k + 1) % 3]);
                let e = (p0.min(p1), p0.max(p1));
                let (q0, q1) = (verts[e.0 as usize], verts[e.1 as usize]);
                if let Some(s) = seg_tri(q0, q1, a, b, c) {
                    if s <= 1e-9 || s >= 1.0 - 1e-9 {
                        continue; // Treffer genau in einer Ecke: übergehen
                    }
                    let pid = *ef_point.entry((e, f_t)).or_insert_with(|| {
                        out_v.push(q0 + (q1 - q0) * s);
                        let id = (out_v.len() - 1) as u32;
                        edge_pts.entry(e).or_default().push((s, id));
                        face_pts.entry(f_t).or_default().push(id);
                        id
                    });
                    if !pts.contains(&pid) {
                        pts.push(pid);
                    }
                }
            }
        }
        if pts.len() == 2 {
            for t in [ta, tb] {
                face_segs.entry(t).or_default().push((pts[0], pts[1]));
            }
        }
    }
    if face_segs.is_empty() {
        // keine echten Durchdringungen: nur ganz innenliegende Körper entfernen
        let mut keep = vec![true; tris.len()];
        let mut flip = vec![false; tris.len()];
        let mut merged = 0;
        // Werkzeug-Körper zunächst entfernen; nur innenliegende werden als Hohlraum behalten
        for (i, t) in tris.iter().enumerate() {
            let _ = t;
            if is_tool[shell[i] as usize] {
                keep[i] = false;
            }
        }
        for &(i, j) in &pairs_shell {
            for &(inner, outer) in &[(i, j), (j, i)] {
                let probe = verts[tris[lists[inner][0] as usize][0] as usize];
                if inside_shell(verts, tris, &lists[outer], probe) {
                    for &t in &lists[inner] {
                        if is_tool[inner] {
                            keep[t as usize] = true;
                            flip[t as usize] = true;
                        } else {
                            keep[t as usize] = false;
                        }
                    }
                    merged += 1;
                }
            }
        }
        let t: Vec<[u32; 3]> = tris
            .iter()
            .enumerate()
            .filter(|(i, _)| keep[*i])
            .map(|(i, t)| if flip[i] { [t[0], t[2], t[1]] } else { *t })
            .collect();
        let origin: Vec<u32> = (0..tris.len() as u32).filter(|&i| keep[i as usize]).collect();
        return UnionResult { verts: verts.to_vec(), tris: t, merged, origin };
    }

    // --- Dreiecke neu unterteilen
    let mut new_tris: Vec<([u32; 3], u32, u32)> = Vec::new(); // (Dreieck, Körper, Ursprungsdreieck)
    for (ti, t) in tris.iter().enumerate() {
        let ti32 = ti as u32;
        let edges_have = (0..3).any(|k| {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            edge_pts.contains_key(&(a.min(b), a.max(b)))
        });
        let segs = face_segs.get(&ti32);
        if segs.is_none() && !edges_have {
            new_tris.push((*t, shell[ti], ti32));
            continue;
        }
        let (a, b, c) = tri_pts(verts, t);
        let nrm = (b - a).cross(c - a);
        if nrm.norm() <= 1e-300 {
            new_tris.push((*t, shell[ti], ti32));
            continue;
        }
        let (e1, e2) = nrm.normalized().basis();
        let mut gid: Vec<u32> = Vec::new();
        let mut local: HashMap<u32, usize> = HashMap::new();
        let mut nodes: Vec<(f64, f64)> = Vec::new();
        let mut add = |g: u32, gid: &mut Vec<u32>, nodes: &mut Vec<(f64, f64)>, out_v: &Vec<V3>| -> usize {
            *local.entry(g).or_insert_with(|| {
                let p = out_v[g as usize];
                nodes.push((p.dot(e1), p.dot(e2)));
                gid.push(g);
                gid.len() - 1
            })
        };
        let mut boundary = Vec::new();
        for k in 0..3 {
            let (p0, p1) = (t[k], t[(k + 1) % 3]);
            boundary.push(add(p0, &mut gid, &mut nodes, &out_v));
            let e = (p0.min(p1), p0.max(p1));
            if let Some(list) = edge_pts.get(&e) {
                let mut l = list.clone();
                // Parameter läuft von e.0 nach e.1
                if p0 != e.0 {
                    for x in l.iter_mut() {
                        x.0 = 1.0 - x.0;
                    }
                }
                l.sort_by(|x, y| x.0.partial_cmp(&y.0).unwrap());
                for (_, pid) in l {
                    boundary.push(add(pid, &mut gid, &mut nodes, &out_v));
                }
            }
        }
        let mut segs_local = Vec::new();
        if let Some(sg) = segs {
            for &(p, q) in sg {
                let (lp, lq) = (add(p, &mut gid, &mut nodes, &out_v), add(q, &mut gid, &mut nodes, &out_v));
                segs_local.push((lp, lq));
            }
        }
        let sub = subdivide(&nodes, &boundary, &segs_local);
        for s in sub {
            let tri = [gid[s[0]], gid[s[1]], gid[s[2]]];
            let n2 = (out_v[tri[1] as usize] - out_v[tri[0] as usize]).cross(out_v[tri[2] as usize] - out_v[tri[0] as usize]);
            let tri = if n2.dot(nrm) < 0.0 { [tri[0], tri[2], tri[1]] } else { tri };
            new_tris.push((tri, shell[ti], ti32));
        }
    }

    // --- Teile entfernen, die im anderen Körper liegen
    // Bereiche zwischen Schnittlinien gemeinsam einordnen (Kanten über Schnittlinien trennen)
    let mut cut_edge: std::collections::HashSet<(u32, u32)> = std::collections::HashSet::new();
    for segs in face_segs.values() {
        for &(p, q) in segs {
            cut_edge.insert((p.min(q), p.max(q)));
        }
    }
    let nt = new_tris.len();
    let mut parent: Vec<u32> = (0..nt as u32).collect();
    fn find(p: &mut [u32], mut x: u32) -> u32 {
        while p[x as usize] != x {
            p[x as usize] = p[p[x as usize] as usize];
            x = p[x as usize];
        }
        x
    }
    let mut edge_owner: HashMap<(u32, u32), u32> = HashMap::new();
    for (i, (t, s, _)) in new_tris.iter().enumerate() {
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            let key = (a.min(b), a.max(b));
            if cut_edge.contains(&key) {
                continue;
            }
            match edge_owner.get(&key) {
                Some(&j) if new_tris[j as usize].1 == *s => {
                    let (ri, rj) = (find(&mut parent, i as u32), find(&mut parent, j));
                    if ri != rj {
                        parent[ri as usize] = rj;
                    }
                }
                Some(_) => {}
                None => {
                    edge_owner.insert(key, i as u32);
                }
            }
        }
    }
    let mut region_inside: HashMap<u32, bool> = HashMap::new();
    let mut keep = vec![true; nt];
    let mut flip = vec![false; nt];
    let mut merged_shells = std::collections::HashSet::new();
    for i in 0..nt {
        let (t, s, _) = new_tris[i];
        if !involved[s as usize] {
            // unbeteiligte Werkzeug-Körper fallen beim Abziehen weg
            if is_tool[s as usize] {
                keep[i] = false;
            }
            continue;
        }
        let r = find(&mut parent, i as u32);
        let inside = *region_inside.entry(r).or_insert_with(|| {
            let (a, b, c) = tri_pts(&out_v, &t);
            let probe = (a + b + c) / 3.0;
            pairs_shell.iter().filter_map(|&(x, y)| if x == s as usize { Some(y) } else if y == s as usize { Some(x) } else { None }).any(|o| inside_shell(verts, tris, &lists[o], probe))
        });
        if is_tool[s as usize] {
            // Werkzeug: nur der Teil im anderen Körper bleibt – umgedreht als Wand der Aussparung
            if inside {
                flip[i] = true;
                merged_shells.insert(s);
            } else {
                keep[i] = false;
            }
        } else if inside {
            keep[i] = false;
            merged_shells.insert(s);
        }
    }
    let tris_out: Vec<[u32; 3]> = new_tris
        .iter()
        .enumerate()
        .filter(|(i, _)| keep[*i])
        .map(|(i, t)| if flip[i] { [t.0[0], t.0[2], t.0[1]] } else { t.0 })
        .collect();
    let origin: Vec<u32> = new_tris.iter().enumerate().filter(|(i, _)| keep[*i]).map(|(_, t)| t.2).collect();
    // unbenutzte Punkte entfernen
    let mut newi = vec![u32::MAX; out_v.len()];
    let mut fv = Vec::new();
    let tris_out: Vec<[u32; 3]> = tris_out
        .iter()
        .map(|t| {
            let mut r = [0u32; 3];
            for k in 0..3 {
                let x = t[k] as usize;
                if newi[x] == u32::MAX {
                    newi[x] = fv.len() as u32;
                    fv.push(out_v[x]);
                }
                r[k] = newi[x];
            }
            r
        })
        .collect();
    UnionResult { verts: fv, tris: tris_out, merged: merged_shells.len(), origin }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cube(o: V3, s: f64) -> (Vec<V3>, Vec<[u32; 3]>) {
        let mut v = Vec::new();
        for k in 0..8 {
            v.push(o + v3(s * (k & 1) as f64, s * ((k >> 1) & 1) as f64, s * ((k >> 2) & 1) as f64));
        }
        let t = vec![[0, 2, 1], [1, 2, 3], [4, 5, 6], [5, 7, 6], [0, 1, 4], [1, 5, 4], [2, 6, 3], [3, 6, 7], [0, 4, 2], [2, 4, 6], [1, 3, 5], [3, 7, 5]];
        (v, t)
    }

    fn join(a: (Vec<V3>, Vec<[u32; 3]>), b: (Vec<V3>, Vec<[u32; 3]>)) -> (Vec<V3>, Vec<[u32; 3]>) {
        let (mut v, mut t) = a;
        let base = v.len() as u32;
        v.extend(b.0);
        t.extend(b.1.iter().map(|x| [x[0] + base, x[1] + base, x[2] + base]));
        (v, t)
    }

    fn check(v: Vec<V3>, t: Vec<[u32; 3]>, vol: f64) {
        let a = crate::repair::analyze_indexed(v, t, false);
        let is = &a.issues;
        assert!(is.open_edges == 0 && is.nonmanifold_edges == 0 && is.flipped == 0, "{is:?}");
        assert_eq!(is.shells, 1, "{is:?}");
        assert!((is.volume - vol).abs() / vol < 1e-6, "Volumen {} statt {vol}", is.volume);
    }

    #[test]
    fn subtract_cube_corner() {
        let (av, at) = cube(V3::ZERO, 10.0);
        let (bv, bt) = cube(v3(5.0, 4.3, 3.7), 10.0);
        let r = subtract(&av, &at, &bv, &bt);
        check(r.verts, r.tris, 1000.0 - 5.0 * 5.7 * 6.3);
    }

    #[test]
    fn subtract_inner_cube_makes_cavity() {
        let (av, at) = cube(V3::ZERO, 10.0);
        let (bv, bt) = cube(v3(3.0, 3.0, 3.0), 2.0);
        let r = subtract(&av, &at, &bv, &bt);
        let a = crate::repair::analyze_indexed(r.verts, r.tris, false);
        assert!((a.issues.volume - 992.0).abs() < 1e-6, "{:?}", a.issues);
    }

    #[test]
    fn two_cubes_overlapping() {
        // allgemeine Lage (keine Kante läuft genau über eine Diagonale)
        let (v, t) = join(cube(V3::ZERO, 10.0), cube(v3(5.0, 4.3, 3.7), 10.0));
        let r = union(&v, &t);
        assert_eq!(r.merged, 2);
        check(r.verts, r.tris, 2000.0 - 5.0 * 5.7 * 6.3);
    }

    #[test]
    fn rotated_cube_through_cube() {
        // zweiter Würfel um 30° gedreht und versetzt – allgemeine Lage
        let (v2, t2) = cube(v3(-5.0, -5.0, -5.0), 10.0);
        let (s, c) = (30f64.to_radians().sin(), 30f64.to_radians().cos());
        let v2: Vec<V3> = v2.iter().map(|p| v3(p.x * c - p.y * s + 8.0, p.x * s + p.y * c + 6.0, p.z + 7.3)).collect();
        let (v, t) = join(cube(V3::ZERO, 10.0), (v2, t2));
        let r = union(&v, &t);
        assert_eq!(r.merged, 2);
        let a = crate::repair::analyze_indexed(r.verts.clone(), r.tris.clone(), false);
        let is = &a.issues;
        assert!(is.open_edges == 0 && is.nonmanifold_edges == 0 && is.flipped == 0, "{is:?}");
        assert_eq!(is.shells, 1);
        // Volumen zwischen größtem Einzelkörper und Summe
        assert!(is.volume > 1000.0 && is.volume < 2000.0, "{}", is.volume);
    }

    #[test]
    fn cube_inside_cube() {
        let (v, t) = join(cube(V3::ZERO, 10.0), cube(v3(3.0, 3.0, 3.0), 2.0));
        let r = union(&v, &t);
        check(r.verts, r.tris, 1000.0);
    }

    #[test]
    fn separate_untouched() {
        let (v, t) = join(cube(V3::ZERO, 10.0), cube(v3(20.0, 0.0, 0.0), 10.0));
        let r = union(&v, &t);
        assert_eq!(r.merged, 0);
        assert_eq!(r.tris.len(), 24);
    }

    #[test]
    fn curved_bodies() {
        // Kugel und Zylinder aus den Prüfkörpern, ineinander geschoben
        let shapes = crate::testgen::shapes();
        let get = |n: &str| shapes.iter().find(|m| m.name == n).cloned().unwrap();
        let a = crate::repair::repair(&get("kugel"), &Default::default());
        let b = crate::repair::repair(&get("zylinder"), &Default::default());
        let va = a.after.issues.volume;
        let vb = b.after.issues.volume;
        // Zylinder seitlich in die Kugel schieben (allgemeine Lage)
        let ca = a.verts.iter().fold(V3::ZERO, |s, p| s + *p) / a.verts.len() as f64;
        let cb = b.verts.iter().fold(V3::ZERO, |s, p| s + *p) / b.verts.len() as f64;
        let shift = ca - cb + v3(7.31, 3.17, 1.93);
        let bv: Vec<V3> = b.verts.iter().map(|p| *p + shift).collect();
        let (v, t) = join((a.verts.clone(), a.tris.clone()), (bv.clone(), b.tris.clone()));
        let (r, merged) = union_clean(&v, &t, "x");
        assert_eq!(merged, 2);
        let is = &r.after.issues;
        assert!(is.watertight(), "{is:?}");
        assert_eq!(is.shells, 1, "{is:?}");
        assert_eq!(is.degenerate + is.duplicate, 0, "{is:?}");
        assert!(is.volume > va.max(vb) && is.volume < va + vb, "{} ({va} + {vb})", is.volume);
        eprintln!("Kugel {va:.0} + Zylinder {vb:.0} -> Vereinigung {:.0} mm³, {} Dreiecke", is.volume, r.tris.len());
        // unabhängige Probe: Zufallspunkte innen ⇔ in Kugel oder Zylinder
        let la: Vec<u32> = (0..a.tris.len() as u32).collect();
        let lb: Vec<u32> = (0..b.tris.len() as u32).collect();
        let lr: Vec<u32> = (0..r.tris.len() as u32).collect();
        let mut mn = v3(f64::MAX, f64::MAX, f64::MAX);
        let mut mx = -mn;
        for p in r.verts.iter() {
            mn = mn.min(*p);
            mx = mx.max(*p);
        }
        let mut seed: u64 = 7;
        let mut rnd = || {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            ((seed >> 11) as f64) / ((1u64 << 53) as f64)
        };
        let (mut agree, n) = (0, 3000);
        for _ in 0..n {
            let p = v3(mn.x + (mx.x - mn.x) * rnd(), mn.y + (mx.y - mn.y) * rnd(), mn.z + (mx.z - mn.z) * rnd());
            let want = inside_shell(&a.verts, &a.tris, &la, p) || inside_shell(&bv, &b.tris, &lb, p);
            let got = inside_shell(&r.verts, &r.tris, &lr, p);
            if want == got {
                agree += 1;
            }
        }
        assert!(agree as f64 / n as f64 > 0.995, "nur {agree} von {n} Punkten stimmen");
        eprintln!("Zufallsprobe: {agree} von {n} Punkten stimmen");
    }
}
