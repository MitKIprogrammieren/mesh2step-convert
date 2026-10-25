//! Netzreparatur (ähnlich „Reparieren“ in 3D Builder).
//!
//! Findet und behebt die üblichen Fehler in STL- und 3MF-Netzen:
//! offene Kanten und Löcher, Spalte und T-Stöße, nicht-mannigfaltige Kanten und Ecken,
//! falsch orientierte Dreiecke und umgestülpte Körper, entartete und doppelte Dreiecke,
//! lose Kleinteile, innenliegende Doppelkörper und – auf Wunsch – Selbstüberschneidungen.

use crate::io::RawMesh;
use crate::math::{v3, V3};
use crate::poly::{self, P2};
use std::collections::{HashMap, HashSet};

const NONE: u32 = u32::MAX;

/// Problemkennzeichen je Dreieck (Bitmaske) für die Vorschau.
pub mod flag {
    pub const DEGEN: u8 = 1;
    pub const DUP: u8 = 2;
    pub const NONMAN: u8 = 4;
    pub const FLIP: u8 = 8;
    pub const SMALL: u8 = 16;
    pub const INTERSECT: u8 = 32;
    pub const OPEN: u8 = 64;
    pub const INNER: u8 = 128;
}

#[derive(Clone, Debug, PartialEq)]
pub struct RepairOpts {
    /// Spalte bis zu dieser Breite schließen (mm, 0 = automatisch)
    pub gap: f64,
    /// nicht-mannigfaltige Kanten und Ecken auftrennen
    pub nonmanifold: bool,
    /// Löcher füllen
    pub fill_holes: bool,
    /// größtes zu füllendes Loch in Kanten (0 = alle)
    pub max_hole: usize,
    /// lose Kleinteile entfernen
    pub remove_small: bool,
    /// Kleinteil = weniger als dieser Anteil der Gesamtfläche
    pub small_frac: f64,
    /// Körper entfernen, die vollständig in einem anderen liegen
    pub remove_inner: bool,
    /// sich überschneidende Dreiecke entfernen und neu schließen (experimentell)
    pub intersections: bool,
}

impl Default for RepairOpts {
    fn default() -> Self {
        RepairOpts {
            gap: 0.0,
            nonmanifold: true,
            fill_holes: true,
            max_hole: 0,
            remove_small: true,
            small_frac: 0.002,
            remove_inner: true,
            intersections: false,
        }
    }
}

/// Gefundene Fehler eines Netzes.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Issues {
    pub triangles: usize,
    pub vertices: usize,
    pub degenerate: usize,
    pub duplicate: usize,
    pub open_edges: usize,
    pub holes: usize,
    pub nonmanifold_edges: usize,
    pub nonmanifold_verts: usize,
    /// Dreiecke, die gegen ihre Nachbarn orientiert sind
    pub flipped: usize,
    /// geschlossene Körper, deren Innenseite nach außen zeigt
    pub inverted_shells: usize,
    pub shells: usize,
    pub small_shells: usize,
    /// Körper, die vollständig in einem anderen Körper liegen (keine Hohlräume)
    pub inner_shells: usize,
    /// Hohlräume (korrekt nach innen orientiert, kein Fehler)
    pub voids: usize,
    /// Dreiecke mit Selbstüberschneidung (None = nicht geprüft)
    pub intersecting: Option<usize>,
    pub volume: f64,
    pub area: f64,
}

impl Issues {
    /// Anzahl der Fehlerarten, die beim Drucken oder Umwandeln stören.
    pub fn defects(&self) -> usize {
        [
            self.degenerate,
            self.duplicate,
            self.open_edges,
            self.nonmanifold_edges,
            self.nonmanifold_verts,
            self.flipped,
            self.inverted_shells,
            self.inner_shells,
            self.intersecting.unwrap_or(0),
        ]
        .iter()
        .filter(|&&n| n > 0)
        .count()
    }
    /// Geschlossen und mannigfaltig (druckbar).
    pub fn watertight(&self) -> bool {
        self.open_edges == 0 && self.nonmanifold_edges == 0 && self.nonmanifold_verts == 0 && self.flipped == 0 && self.inverted_shells == 0
    }
    pub fn add(&mut self, o: &Issues) {
        self.triangles += o.triangles;
        self.vertices += o.vertices;
        self.degenerate += o.degenerate;
        self.duplicate += o.duplicate;
        self.open_edges += o.open_edges;
        self.holes += o.holes;
        self.nonmanifold_edges += o.nonmanifold_edges;
        self.nonmanifold_verts += o.nonmanifold_verts;
        self.flipped += o.flipped;
        self.inverted_shells += o.inverted_shells;
        self.shells += o.shells;
        self.small_shells += o.small_shells;
        self.inner_shells += o.inner_shells;
        self.voids += o.voids;
        self.intersecting = match (self.intersecting, o.intersecting) {
            (Some(a), Some(b)) => Some(a + b),
            (a, None) => a,
            (None, b) => b,
        };
        self.volume += o.volume;
        self.area += o.area;
    }
}

/// Ergebnis der Prüfung: Fehler, verschweißtes Netz und Kennzeichen je Dreieck.
#[derive(Clone, Debug, Default)]
pub struct Analysis {
    pub issues: Issues,
    pub verts: Vec<V3>,
    pub tris: Vec<[u32; 3]>,
    pub flags: Vec<u8>,
    /// offene Kanten (Lochränder) mit dem angrenzenden Dreieck
    pub open: Vec<([V3; 2], u32)>,
}

/// Repariertes Netz.
#[derive(Clone, Debug, Default)]
pub struct Repaired {
    pub name: String,
    pub verts: Vec<V3>,
    pub tris: Vec<[u32; 3]>,
    /// je Dreieck: beim Füllen eines Lochs neu entstanden
    pub filled: Vec<bool>,
    /// Prüfung des Ergebnisses
    pub after: Analysis,
    /// was gemacht wurde (für die Anzeige)
    pub log: Vec<String>,
}

// ---------------------------------------------------------------------------
// Hilfen

struct Dsu(Vec<u32>);

impl Dsu {
    fn new(n: usize) -> Dsu {
        Dsu((0..n as u32).collect())
    }
    fn find(&mut self, mut x: u32) -> u32 {
        while self.0[x as usize] != x {
            let p = self.0[self.0[x as usize] as usize];
            self.0[x as usize] = p;
            x = p;
        }
        x
    }
    fn union(&mut self, a: u32, b: u32) {
        let (ra, rb) = (self.find(a), self.find(b));
        if ra != rb {
            self.0[ra.max(rb) as usize] = ra.min(rb);
        }
    }
}

fn bbox(v: &[V3]) -> (V3, V3) {
    let mut mn = v3(f64::MAX, f64::MAX, f64::MAX);
    let mut mx = -mn;
    for p in v {
        mn = mn.min(*p);
        mx = mx.max(*p);
    }
    if v.is_empty() {
        (V3::ZERO, V3::ZERO)
    } else {
        (mn, mx)
    }
}

#[inline]
fn tv(v: &[V3], t: &[u32; 3]) -> (V3, V3, V3) {
    (v[t[0] as usize], v[t[1] as usize], v[t[2] as usize])
}

#[inline]
fn idx_degenerate(t: &[u32; 3]) -> bool {
    t[0] == t[1] || t[1] == t[2] || t[0] == t[2]
}

/// Fläche null (Ecken doppelt oder auf einer Linie).
fn is_degenerate(v: &[V3], t: &[u32; 3]) -> bool {
    if idx_degenerate(t) {
        return true;
    }
    let (a, b, c) = tv(v, t);
    let l = (b - a).norm2().max((c - b).norm2()).max((a - c).norm2());
    if l <= 0.0 {
        return true;
    }
    (b - a).cross(c - a).norm() <= 1e-9 * l
}

fn tri_area(v: &[V3], t: &[u32; 3]) -> f64 {
    let (a, b, c) = tv(v, t);
    (b - a).cross(c - a).norm() * 0.5
}

#[inline]
fn he_from(t: &[[u32; 3]], h: u32) -> u32 {
    t[(h / 3) as usize][(h % 3) as usize]
}
#[inline]
fn he_to(t: &[[u32; 3]], h: u32) -> u32 {
    t[(h / 3) as usize][((h % 3 + 1) % 3) as usize]
}

/// Ungerichtete Kante -> Halbkanten (h = 3*Dreieck + k läuft von t[k] nach t[k+1]).
fn edge_map(t: &[[u32; 3]]) -> HashMap<(u32, u32), Vec<u32>> {
    let mut em: HashMap<(u32, u32), Vec<u32>> = HashMap::with_capacity(t.len() * 2);
    for (i, tri) in t.iter().enumerate() {
        if idx_degenerate(tri) {
            continue;
        }
        for k in 0..3 {
            let (a, b) = (tri[k], tri[(k + 1) % 3]);
            em.entry((a.min(b), a.max(b))).or_default().push((3 * i + k) as u32);
        }
    }
    em
}

/// Ecken zusammenlegen, die näher als `eps` beieinander liegen.
pub fn weld(verts: &[V3], tris: &[[u32; 3]], eps: f64) -> (Vec<V3>, Vec<[u32; 3]>) {
    let cell = (eps * 2.0).max(1e-12);
    let key = |p: V3| ((p.x / cell).floor() as i64, (p.y / cell).floor() as i64, (p.z / cell).floor() as i64);
    let mut grid: HashMap<(i64, i64, i64), Vec<u32>> = HashMap::with_capacity(verts.len() / 3 + 1);
    let mut out: Vec<V3> = Vec::with_capacity(verts.len() / 4 + 1);
    let mut remap = vec![NONE; verts.len()];
    let mut used = vec![false; verts.len()];
    for t in tris {
        for &i in t {
            if (i as usize) < verts.len() {
                used[i as usize] = true;
            }
        }
    }
    for (i, &p) in verts.iter().enumerate() {
        if !used[i] || !p.is_finite() {
            continue;
        }
        let (kx, ky, kz) = key(p);
        let mut found = NONE;
        'outer: for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    if let Some(list) = grid.get(&(kx + dx, ky + dy, kz + dz)) {
                        for &j in list {
                            if out[j as usize].dist(p) <= eps {
                                found = j;
                                break 'outer;
                            }
                        }
                    }
                }
            }
        }
        if found == NONE {
            found = out.len() as u32;
            out.push(p);
            grid.entry((kx, ky, kz)).or_default().push(found);
        }
        remap[i] = found;
    }
    let mut nt = Vec::with_capacity(tris.len());
    for t in tris {
        if t.iter().any(|&i| i as usize >= verts.len() || remap[i as usize] == NONE) {
            continue;
        }
        nt.push([remap[t[0] as usize], remap[t[1] as usize], remap[t[2] as usize]]);
    }
    (out, nt)
}

/// Punkt-in-Körper-Test per Strahl (Paritätsprüfung, Mehrheit aus drei Richtungen).
fn point_inside(v: &[V3], t: &[[u32; 3]], tris: &[u32], p: V3) -> bool {
    let dirs = [v3(0.5773, 0.5774, 0.5775), v3(-0.6011, 0.3123, 0.7357), v3(0.2113, -0.8712, 0.4431)];
    let mut votes = 0;
    for d in dirs {
        let d = d.normalized();
        let mut cnt = 0;
        for &i in tris {
            let (a, b, c) = tv(v, &t[i as usize]);
            let e1 = b - a;
            let e2 = c - a;
            let pv = d.cross(e2);
            let det = e1.dot(pv);
            if det.abs() < 1e-14 {
                continue;
            }
            let inv = 1.0 / det;
            let s = p - a;
            let u = s.dot(pv) * inv;
            if !(0.0..=1.0).contains(&u) {
                continue;
            }
            let q = s.cross(e1);
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

#[derive(Clone)]
struct Shell {
    tris: Vec<u32>,
    closed: bool,
    volume: f64,
    area: f64,
    bmin: V3,
    bmax: V3,
}

impl Shell {
    fn contains_box(&self, p: V3) -> bool {
        p.x >= self.bmin.x && p.y >= self.bmin.y && p.z >= self.bmin.z && p.x <= self.bmax.x && p.y <= self.bmax.y && p.z <= self.bmax.z
    }
}

/// Verschachtelungstiefe je geschlossenem Körper (Anzahl umschließender Körper).
fn nesting(v: &[V3], t: &[[u32; 3]], shells: &[Shell]) -> Vec<usize> {
    let mut depth = vec![0usize; shells.len()];
    let closed: Vec<usize> = (0..shells.len()).filter(|&i| shells[i].closed).collect();
    if closed.len() < 2 || closed.len() > 400 {
        return depth;
    }
    for &i in &closed {
        let Some(&t0) = shells[i].tris.iter().find(|&&x| !is_degenerate(v, &t[x as usize])) else {
            continue;
        };
        let (a, b, c) = tv(v, &t[t0 as usize]);
        let probe = (a + b + c) / 3.0;
        for &j in &closed {
            if j == i || shells[j].volume.abs() <= shells[i].volume.abs() || !shells[j].contains_box(probe) {
                continue;
            }
            if point_inside(v, t, &shells[j].tris, probe) {
                depth[i] += 1;
            }
        }
    }
    depth
}

// ---------------------------------------------------------------------------
// Selbstüberschneidungen

fn seg_tri(p: V3, q: V3, a: V3, b: V3, c: V3) -> bool {
    const E: f64 = 1e-7;
    let d = q - p;
    let e1 = b - a;
    let e2 = c - a;
    let h = d.cross(e2);
    let det = e1.dot(h);
    if det.abs() <= 1e-12 * d.norm() * e1.norm() * e2.norm() {
        return false;
    }
    let f = 1.0 / det;
    let s = p - a;
    let u = f * s.dot(h);
    // auf dem Dreieck inklusive Rand (Durchstoß genau durch eine Kante zählt),
    // auf der Strecke ohne Endpunkte (bloßes Berühren zählt nicht)
    if u < -E || u > 1.0 + E {
        return false;
    }
    let qv = s.cross(e1);
    let w = f * d.dot(qv);
    if w < -E || u + w > 1.0 + E {
        return false;
    }
    let tt = f * e2.dot(qv);
    tt > E && tt < 1.0 - E
}

fn tri_tri(a: (V3, V3, V3), b: (V3, V3, V3)) -> bool {
    let ea = [(a.0, a.1), (a.1, a.2), (a.2, a.0)];
    let eb = [(b.0, b.1), (b.1, b.2), (b.2, b.0)];
    ea.iter().any(|&(p, q)| seg_tri(p, q, b.0, b.1, b.2)) || eb.iter().any(|&(p, q)| seg_tri(p, q, a.0, a.1, a.2))
}

/// Dreiecke, die ein anderes (nicht benachbartes) Dreieck durchstoßen. None = Netz zu groß.
pub fn find_intersections(v: &[V3], t: &[[u32; 3]], skip: &[bool]) -> Option<Vec<bool>> {
    let nt = t.len();
    if nt > 3_000_000 {
        return None;
    }
    let mut hit = vec![false; nt];
    if nt < 2 {
        return Some(hit);
    }
    let boxes: Vec<(V3, V3)> = t
        .iter()
        .map(|x| {
            let (a, b, c) = tv(v, x);
            (a.min(b).min(c), a.max(b).max(c))
        })
        .collect();
    let (gmin, gmax) = bbox(v);
    let diag = (gmax - gmin).norm().max(1e-9);
    // Zellgröße ≈ typische Dreiecksgröße
    let mut ext: Vec<f64> = boxes.iter().map(|(a, b)| (*b - *a).x.max((*b - *a).y).max((*b - *a).z)).collect();
    let mid = ext.len() / 2;
    ext.select_nth_unstable_by(mid, |a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let mut cell = (ext[mid] * 2.0).max(diag * 1e-4);
    let range = |x: f64, c: f64, o: f64| ((x - o) / c).floor() as i64;
    // Zellgröße verdoppeln, bis die Zahl der Einträge vertretbar ist
    loop {
        let mut cnt: u64 = 0;
        for (a, b) in &boxes {
            let n = |k: usize| (range(b.get(k), cell, gmin.get(k)) - range(a.get(k), cell, gmin.get(k)) + 1) as u64;
            cnt += n(0) * n(1) * n(2);
        }
        if cnt <= 40_000_000 {
            break;
        }
        cell *= 2.0;
    }
    let mut grid: HashMap<(i64, i64, i64), Vec<u32>> = HashMap::new();
    for (i, (a, b)) in boxes.iter().enumerate() {
        if skip.get(i).copied().unwrap_or(false) || is_degenerate(v, &t[i]) {
            continue;
        }
        for x in range(a.x, cell, gmin.x)..=range(b.x, cell, gmin.x) {
            for y in range(a.y, cell, gmin.y)..=range(b.y, cell, gmin.y) {
                for z in range(a.z, cell, gmin.z)..=range(b.z, cell, gmin.z) {
                    grid.entry((x, y, z)).or_default().push(i as u32);
                }
            }
        }
    }
    for (&(cx, cy, cz), list) in &grid {
        for ii in 0..list.len() {
            let i = list[ii] as usize;
            for &j in &list[ii + 1..] {
                let j = j as usize;
                if hit[i] && hit[j] {
                    continue;
                }
                let (ai, bi) = boxes[i];
                let (aj, bj) = boxes[j];
                let lo = ai.max(aj);
                let hi = bi.min(bj);
                if lo.x > hi.x || lo.y > hi.y || lo.z > hi.z {
                    continue;
                }
                // jedes Paar nur in der Zelle der unteren Ecke des Überlappungsbereichs prüfen
                if (range(lo.x, cell, gmin.x), range(lo.y, cell, gmin.y), range(lo.z, cell, gmin.z)) != (cx, cy, cz) {
                    continue;
                }
                let (ti, tj) = (&t[i], &t[j]);
                if ti.iter().any(|x| tj.contains(x)) {
                    continue;
                }
                if tri_tri(tv(v, ti), tv(v, tj)) {
                    hit[i] = true;
                    hit[j] = true;
                }
            }
        }
    }
    Some(hit)
}

// ---------------------------------------------------------------------------
// Prüfung

/// Prüft ein Netz aus der Datei (Ecken werden dafür verschweißt).
pub fn analyze(raw: &RawMesh, intersections: bool) -> Analysis {
    let (mn, mx) = bbox(&raw.verts);
    let diag = (mx - mn).norm().max(1e-9);
    let (v, t) = weld(&raw.verts, &raw.tris, diag * 1e-7);
    analyze_indexed(v, t, intersections)
}

/// Prüft ein Netz mit gemeinsamen Ecken (Topologie über die Eckindizes).
pub fn analyze_indexed(v: Vec<V3>, t: Vec<[u32; 3]>, intersections: bool) -> Analysis {
    let nt = t.len();
    let mut fl = vec![0u8; nt];
    let mut is = Issues { triangles: nt, ..Default::default() };
    {
        let mut used = vec![false; v.len()];
        for x in &t {
            for &i in x {
                used[i as usize] = true;
            }
        }
        is.vertices = used.iter().filter(|&&u| u).count();
    }
    // entartet / doppelt
    let mut seen: HashMap<[u32; 3], u32> = HashMap::with_capacity(nt);
    for (i, x) in t.iter().enumerate() {
        if is_degenerate(&v, x) {
            fl[i] |= flag::DEGEN;
            is.degenerate += 1;
        }
        if idx_degenerate(x) {
            continue;
        }
        let mut s = *x;
        s.sort_unstable();
        if let Some(&j) = seen.get(&s) {
            fl[i] |= flag::DUP;
            fl[j as usize] |= flag::DUP;
            is.duplicate += 1;
        } else {
            seen.insert(s, i as u32);
        }
    }
    // Kanten
    let em = edge_map(&t);
    let mut open = Vec::new();
    let mut hole_dsu = Dsu::new(v.len());
    let mut comp = Dsu::new(nt);
    let mut corner = Dsu::new(nt * 3);
    let mut adj: Vec<(u32, u32, bool)> = Vec::new();
    for (&(a, b), list) in &em {
        for w in list.windows(2) {
            comp.union(w[0] / 3, w[1] / 3);
        }
        match list.len() {
            1 => {
                is.open_edges += 1;
                fl[(list[0] / 3) as usize] |= flag::OPEN;
                open.push(([v[a as usize], v[b as usize]], list[0] / 3));
                hole_dsu.union(a, b);
            }
            2 => {
                let (h1, h2) = (list[0], list[1]);
                let same = he_from(&t, h1) == he_from(&t, h2);
                adj.push((h1 / 3, h2 / 3, same));
                // Ecken der beiden Dreiecke an a und b verbinden
                let c = |h: u32, vtx: u32| -> u32 {
                    let tt = h / 3;
                    let k = t[tt as usize].iter().position(|&x| x == vtx).unwrap() as u32;
                    3 * tt + k
                };
                corner.union(c(h1, a), c(h2, a));
                corner.union(c(h1, b), c(h2, b));
            }
            _ => {
                is.nonmanifold_edges += 1;
                for &h in list {
                    fl[(h / 3) as usize] |= flag::NONMAN;
                }
            }
        }
    }
    {
        let mut roots = HashSet::new();
        for (&(a, _), list) in &em {
            if list.len() == 1 {
                roots.insert(hole_dsu.find(a));
            }
        }
        is.holes = roots.len();
    }
    // nicht-mannigfaltige Ecken: mehr als ein Dreiecksfächer an einer Ecke
    {
        let mut vc: Vec<(u32, u32)> = Vec::with_capacity(nt * 3);
        for (i, x) in t.iter().enumerate() {
            if idx_degenerate(x) {
                continue;
            }
            for k in 0..3 {
                vc.push((x[k], (3 * i + k) as u32));
            }
        }
        vc.sort_unstable();
        let mut i = 0;
        while i < vc.len() {
            let mut j = i;
            let mut roots: Vec<u32> = Vec::new();
            while j < vc.len() && vc[j].0 == vc[i].0 {
                let r = corner.find(vc[j].1);
                if !roots.contains(&r) {
                    roots.push(r);
                }
                j += 1;
            }
            if roots.len() > 1 {
                is.nonmanifold_verts += 1;
                for q in &vc[i..j] {
                    fl[(q.1 / 3) as usize] |= flag::NONMAN;
                }
            }
            i = j;
        }
    }
    // Orientierung: Gegenläufer innerhalb zusammenhängender Bereiche (Minderheit)
    let mut flip = vec![false; nt];
    {
        let mut nb: Vec<Vec<(u32, bool)>> = vec![Vec::new(); nt];
        for &(a, b, same) in &adj {
            nb[a as usize].push((b, same));
            nb[b as usize].push((a, same));
        }
        let mut vis = vec![false; nt];
        let mut stack = Vec::new();
        for s in 0..nt {
            if vis[s] {
                continue;
            }
            vis[s] = true;
            stack.push(s);
            let mut region = Vec::new();
            while let Some(x) = stack.pop() {
                region.push(x);
                for &(u, same) in &nb[x] {
                    let u = u as usize;
                    if !vis[u] {
                        vis[u] = true;
                        flip[u] = flip[x] ^ same;
                        stack.push(u);
                    }
                }
            }
            let nf = region.iter().filter(|&&x| flip[x]).count();
            let minority = nf * 2 > region.len();
            for &x in &region {
                if minority {
                    flip[x] = !flip[x];
                }
                if flip[x] {
                    fl[x] |= flag::FLIP;
                    is.flipped += 1;
                }
            }
        }
    }
    // Körper
    let mut groups: HashMap<u32, Vec<u32>> = HashMap::new();
    for i in 0..nt {
        if !idx_degenerate(&t[i]) {
            groups.entry(comp.find(i as u32)).or_default().push(i as u32);
        }
    }
    let mut shells: Vec<Shell> = groups
        .into_values()
        .map(|list| {
            let mut vol = 0.0;
            let mut area = 0.0;
            let mut closed = true;
            let mut pts = Vec::new();
            for &i in &list {
                let x = &t[i as usize];
                let (a, b, c) = tv(&v, x);
                let s = a.dot(b.cross(c)) / 6.0;
                vol += if flip[i as usize] { -s } else { s };
                area += tri_area(&v, x);
                if fl[i as usize] & (flag::OPEN | flag::NONMAN) != 0 {
                    closed = false;
                }
                pts.extend([a, b, c]);
            }
            let (bmin, bmax) = bbox(&pts);
            Shell {
                tris: list,
                closed,
                volume: vol,
                area,
                bmin,
                bmax,
            }
        })
        .collect();
    shells.sort_by(|a, b| b.area.partial_cmp(&a.area).unwrap_or(std::cmp::Ordering::Equal));
    is.shells = shells.len();
    is.area = shells.iter().map(|s| s.area).sum();
    if shells.len() > 1 {
        for s in &shells {
            if s.area < is.area * RepairOpts::default().small_frac {
                is.small_shells += 1;
                for &i in &s.tris {
                    fl[i as usize] |= flag::SMALL;
                }
            }
        }
    }
    let depth = nesting(&v, &t, &shells);
    for (i, s) in shells.iter().enumerate() {
        if !s.closed {
            continue;
        }
        if depth[i] % 2 == 0 {
            if s.volume < 0.0 {
                is.inverted_shells += 1;
            }
            is.volume += s.volume.abs();
        } else if s.volume > 0.0 {
            is.inner_shells += 1;
            for &x in &s.tris {
                fl[x as usize] |= flag::INNER;
            }
        } else {
            is.voids += 1;
            is.volume -= s.volume.abs();
        }
    }
    if intersections {
        if let Some(h) = find_intersections(&v, &t, &[]) {
            let mut n = 0;
            for (i, &x) in h.iter().enumerate() {
                if x {
                    fl[i] |= flag::INTERSECT;
                    n += 1;
                }
            }
            is.intersecting = Some(n);
        }
    }
    Analysis {
        issues: is,
        verts: v,
        tris: t,
        flags: fl,
        open,
    }
}

// ---------------------------------------------------------------------------
// Reparatur

struct W {
    v: Vec<V3>,
    t: Vec<[u32; 3]>,
    filled: Vec<bool>,
    log: Vec<String>,
}

impl W {
    fn retain(&mut self, keep: &[bool]) {
        let mut k = 0;
        self.t.retain(|_| {
            k += 1;
            keep[k - 1]
        });
        let mut k = 0;
        self.filled.retain(|_| {
            k += 1;
            keep[k - 1]
        });
    }
    fn push(&mut self, tri: [u32; 3], filled: bool) {
        self.t.push(tri);
        self.filled.push(filled);
    }
    fn note(&mut self, n: usize, one: &str, many: &str) {
        if n > 0 {
            self.log.push(if n == 1 { one.to_string() } else { many.replace("{n}", &n.to_string()) });
        }
    }
}

/// Entartete, doppelte und gefaltete Dreiecke entfernen; Dreiecke ohne Fläche,
/// deren Ecke auf der Gegenkante liegt (T-Stoß), durch Teilen des Nachbarn auflösen.
fn clean(w: &mut W) {
    // gleiche Ecken
    let keep: Vec<bool> = w.t.iter().map(|x| !idx_degenerate(x)).collect();
    let n_idx = keep.iter().filter(|&&k| !k).count();
    w.retain(&keep);
    // doppelt / gefaltet
    let mut seen: HashMap<[u32; 3], (usize, bool)> = HashMap::with_capacity(w.t.len());
    let mut keep = vec![true; w.t.len()];
    let (mut n_dup, mut n_fold) = (0, 0);
    for (i, x) in w.t.iter().enumerate() {
        let mut s = *x;
        s.sort_unstable();
        let par = (x[0] < x[1]) as u8 + (x[1] < x[2]) as u8 + (x[2] < x[0]) as u8 == 2;
        match seen.get(&s) {
            Some(&(j, p0)) => {
                if p0 != par && keep[j] {
                    keep[j] = false;
                    n_fold += 1;
                } else {
                    n_dup += 1;
                }
                keep[i] = false;
            }
            None => {
                seen.insert(s, (i, par));
            }
        }
    }
    w.retain(&keep);
    // Dreiecke ohne Fläche
    let mut n_sliver = 0;
    for _round in 0..4 {
        let em = edge_map(&w.t);
        let nt = w.t.len();
        let mut keep = vec![true; nt];
        let mut touched = vec![false; nt];
        let mut changed = false;
        for i in 0..nt {
            if !is_degenerate(&w.v, &w.t[i]) || touched[i] {
                continue;
            }
            let x = w.t[i];
            // längste Kante und gegenüberliegende Ecke
            let mut best = (0usize, -1.0);
            for k in 0..3 {
                let l = w.v[x[k] as usize].dist(w.v[x[(k + 1) % 3] as usize]);
                if l > best.1 {
                    best = (k, l);
                }
            }
            let k = best.0;
            let (a, b, c) = (x[k], x[(k + 1) % 3], x[(k + 2) % 3]);
            let (pa, pb, pc) = (w.v[a as usize], w.v[b as usize], w.v[c as usize]);
            let ab = pb - pa;
            let s = if ab.norm2() > 0.0 { (pc - pa).dot(ab) / ab.norm2() } else { 0.0 };
            keep[i] = false;
            touched[i] = true;
            changed = true;
            n_sliver += 1;
            if !(0.001..=0.999).contains(&s) {
                continue; // zwei Ecken fallen zusammen – Spalt schließt später das Zusammenführen
            }
            // Nachbar über a-b teilen: (x, y, d) -> (x, c, d) + (c, y, d)
            if let Some(list) = em.get(&(a.min(b), a.max(b))) {
                let other: Vec<u32> = list.iter().copied().filter(|&h| (h / 3) as usize != i).collect();
                if other.len() == 1 {
                    let u = (other[0] / 3) as usize;
                    if !touched[u] && keep[u] && !is_degenerate(&w.v, &w.t[u]) {
                        let kk = (other[0] % 3) as usize;
                        let tu = w.t[u];
                        let (xx, yy, d) = (tu[kk], tu[(kk + 1) % 3], tu[(kk + 2) % 3]);
                        let f = w.filled[u];
                        w.t[u] = [xx, c, d];
                        touched[u] = true;
                        w.t.push([c, yy, d]);
                        w.filled.push(f);
                        keep.push(true);
                        touched.push(true);
                    }
                }
            }
        }
        w.retain(&keep);
        if !changed {
            break;
        }
    }
    let keep: Vec<bool> = w.t.iter().map(|x| !is_degenerate(&w.v, x)).collect();
    w.retain(&keep);
    w.note(n_idx + n_sliver, "1 entartetes Dreieck entfernt", "{n} entartete Dreiecke entfernt");
    w.note(n_dup, "1 doppeltes Dreieck entfernt", "{n} doppelte Dreiecke entfernt");
    w.note(n_fold, "1 gefaltetes Dreieckspaar entfernt", "{n} gefaltete Dreieckspaare entfernt");
}

fn boundary_halfedges(t: &[[u32; 3]]) -> Vec<u32> {
    let em = edge_map(t);
    let mut b: Vec<u32> = em.values().filter(|l| l.len() == 1).map(|l| l[0]).collect();
    b.sort_unstable();
    b
}

/// Spalte schließen: nahe Randecken zusammenlegen und Randkanten an Randecken teilen,
/// die auf ihnen liegen (T-Stöße). So werden „Risse“ wieder zu gemeinsamen Kanten.
fn zip_gaps(w: &mut W, gap: f64) {
    let (mut n_snap, mut n_split) = (0usize, 0usize);
    for _round in 0..4 {
        let bh = boundary_halfedges(&w.t);
        if bh.is_empty() {
            break;
        }
        let mut changed = false;
        // 1) Randecken mit einer nahen Ecke zusammenlegen (nicht über eine vorhandene Kante).
        //    Ziel kann jede Ecke sein – so verschwinden auch fast deckungsgleiche Doppeldreiecke.
        {
            let em = edge_map(&w.t);
            let mut bv: Vec<u32> = bh.iter().flat_map(|&h| [he_from(&w.t, h), he_to(&w.t, h)]).collect();
            bv.sort_unstable();
            bv.dedup();
            let is_b: HashSet<u32> = bv.iter().copied().collect();
            let cell = gap.max(1e-12);
            let key = |p: V3| ((p.x / cell).floor() as i64, (p.y / cell).floor() as i64, (p.z / cell).floor() as i64);
            let mut grid: HashMap<(i64, i64, i64), Vec<u32>> = HashMap::new();
            let mut used = vec![false; w.v.len()];
            for x in &w.t {
                for &i in x {
                    used[i as usize] = true;
                }
            }
            for (i, &u) in used.iter().enumerate() {
                if u {
                    grid.entry(key(w.v[i])).or_default().push(i as u32);
                }
            }
            let mut rep: HashMap<u32, u32> = HashMap::new();
            let mut target: HashSet<u32> = HashSet::new();
            for &i in &bv {
                if target.contains(&i) {
                    continue;
                }
                let p = w.v[i as usize];
                let (kx, ky, kz) = key(p);
                let mut best: Option<(f64, u32)> = None;
                for dx in -1..=1 {
                    for dy in -1..=1 {
                        for dz in -1..=1 {
                            if let Some(list) = grid.get(&(kx + dx, ky + dy, kz + dz)) {
                                for &j in list {
                                    if j == i || rep.contains_key(&j) || em.contains_key(&(i.min(j), i.max(j))) {
                                        continue;
                                    }
                                    let d = w.v[j as usize].dist(p);
                                    // Randecken bevorzugen (echte Risse), sonst nächste Ecke
                                    let d = if is_b.contains(&j) { d * 0.5 } else { d };
                                    if w.v[j as usize].dist(p) <= gap && best.map_or(true, |b| d < b.0) {
                                        best = Some((d, j));
                                    }
                                }
                            }
                        }
                    }
                }
                if let Some((_, j)) = best {
                    rep.insert(i, j);
                    target.insert(j);
                }
            }
            if !rep.is_empty() {
                n_snap += rep.len();
                for x in w.t.iter_mut() {
                    for i in x.iter_mut() {
                        if let Some(&r) = rep.get(i) {
                            *i = r;
                        }
                    }
                }
                let keep: Vec<bool> = w.t.iter().map(|x| !idx_degenerate(x)).collect();
                w.retain(&keep);
                changed = true;
            }
        }
        // 2) T-Stöße: Randecke liegt auf einer Randkante
        {
            let bh = boundary_halfedges(&w.t);
            if bh.is_empty() {
                break;
            }
            let mut bv: Vec<u32> = bh.iter().flat_map(|&h| [he_from(&w.t, h), he_to(&w.t, h)]).collect();
            bv.sort_unstable();
            bv.dedup();
            let mean = bh.iter().map(|&h| w.v[he_from(&w.t, h) as usize].dist(w.v[he_to(&w.t, h) as usize])).sum::<f64>() / bh.len() as f64;
            let cell = mean.max(gap * 2.0).max(1e-9);
            let key = |p: V3| ((p.x / cell).floor() as i64, (p.y / cell).floor() as i64, (p.z / cell).floor() as i64);
            let mut grid: HashMap<(i64, i64, i64), Vec<u32>> = HashMap::new();
            for &i in &bv {
                grid.entry(key(w.v[i as usize])).or_default().push(i);
            }
            let mut splits: HashMap<u32, Vec<(f64, u32)>> = HashMap::new(); // Halbkante -> (Parameter, Ecke)
            for &h in &bh {
                let (u, vv) = (he_from(&w.t, h), he_to(&w.t, h));
                let (pu, pv) = (w.v[u as usize], w.v[vv as usize]);
                let d = pv - pu;
                let len = d.norm();
                if len <= gap * 2.0 {
                    continue;
                }
                let steps = ((len / cell) * 2.0).ceil().min(20_000.0) as usize + 1;
                let mut cand: HashSet<u32> = HashSet::new();
                let mut last = None;
                for s in 0..=steps {
                    let p = pu + d * (s as f64 / steps as f64);
                    let k = key(p);
                    if last == Some(k) {
                        continue;
                    }
                    last = Some(k);
                    for dx in -1..=1 {
                        for dy in -1..=1 {
                            for dz in -1..=1 {
                                if let Some(list) = grid.get(&(k.0 + dx, k.1 + dy, k.2 + dz)) {
                                    cand.extend(list.iter().copied());
                                }
                            }
                        }
                    }
                }
                for c in cand {
                    if c == u || c == vv {
                        continue;
                    }
                    let pc = w.v[c as usize];
                    let s = (pc - pu).dot(d) / (len * len);
                    if s <= 1e-4 || s >= 1.0 - 1e-4 {
                        continue;
                    }
                    let dist = (pu + d * s).dist(pc);
                    if dist <= gap && pc.dist(pu) > gap && pc.dist(pv) > gap {
                        splits.entry(h).or_default().push((s, c));
                    }
                }
            }
            let mut done = HashSet::new();
            for (h, mut list) in splits {
                let ti = (h / 3) as usize;
                if !done.insert(ti) {
                    continue; // je Durchgang nur eine Kante je Dreieck teilen
                }
                list.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
                list.dedup_by_key(|x| x.1);
                let k = (h % 3) as usize;
                let x = w.t[ti];
                let (u, vv, apex) = (x[k], x[(k + 1) % 3], x[(k + 2) % 3]);
                let f = w.filled[ti];
                let mut chain = vec![u];
                chain.extend(list.iter().map(|q| q.1));
                chain.push(vv);
                w.t[ti] = [chain[0], chain[1], apex];
                for s in 1..chain.len() - 1 {
                    w.push([chain[s], chain[s + 1], apex], f);
                }
                n_split += list.len();
                changed = true;
            }
        }
        clean_quiet(w);
        if !changed {
            break;
        }
    }
    w.note(n_snap, "1 Spalt geschlossen", "{n} Spaltecken zusammengeführt");
    w.note(n_split, "1 T-Stoß aufgelöst", "{n} T-Stöße aufgelöst");
}

fn clean_quiet(w: &mut W) {
    let n = w.log.len();
    clean(w);
    w.log.truncate(n);
}

/// Zwillingshalbkanten bestimmen. An Kanten mit mehr als zwei Dreiecken werden die
/// Dreiecke nach ihrem Winkel um die Kante sortiert und benachbarte Paare verbunden.
fn topology(v: &[V3], t: &[[u32; 3]]) -> Vec<u32> {
    let em = edge_map(t);
    let mut tw = vec![NONE; t.len() * 3];
    for (&(a, b), list) in &em {
        match list.len() {
            1 => {}
            2 => {
                tw[list[0] as usize] = list[1];
                tw[list[1] as usize] = list[0];
            }
            _ => {
                let (pa, pb) = (v[a as usize], v[b as usize]);
                let d = (pb - pa).normalized();
                let (e1, e2) = d.basis();
                let mut ring: Vec<(f64, u32, bool)> = list
                    .iter()
                    .map(|&h| {
                        let x = t[(h / 3) as usize];
                        let apex = x[((h % 3 + 2) % 3) as usize];
                        let q = v[apex as usize] - pa;
                        ((q.dot(e2)).atan2(q.dot(e1)), h, he_from(t, h) == a)
                    })
                    .collect();
                ring.sort_by(|x, y| x.0.partial_cmp(&y.0).unwrap_or(std::cmp::Ordering::Equal));
                // benachbarte Paare mit Gegenrichtung zuerst
                let mut items: Vec<(u32, bool)> = ring.iter().map(|r| (r.1, r.2)).collect();
                loop {
                    let n = items.len();
                    if n < 2 {
                        break;
                    }
                    let pos = (0..n).find(|&i| items[i].1 != items[(i + 1) % n].1);
                    let i = match pos {
                        Some(i) => i,
                        None => 0, // alle gleich gerichtet: trotzdem Nachbarn verbinden (Orientierung folgt)
                    };
                    let j = (i + 1) % n;
                    let (h1, h2) = (items[i].0, items[j].0);
                    tw[h1 as usize] = h2;
                    tw[h2 as usize] = h1;
                    let (hi, lo) = (i.max(j), i.min(j));
                    items.remove(hi);
                    items.remove(lo);
                }
            }
        }
    }
    tw
}

/// Dreiecke über die Zwillingskanten einheitlich ausrichten. Rückgabe: Anzahl gedreht.
fn orient(t: &mut [[u32; 3]], tw: &[u32]) -> usize {
    let nt = t.len();
    let mut flip = vec![false; nt];
    let mut vis = vec![false; nt];
    let mut stack = Vec::new();
    let mut n = 0;
    for s in 0..nt {
        if vis[s] {
            continue;
        }
        vis[s] = true;
        stack.push(s);
        let mut region = Vec::new();
        while let Some(x) = stack.pop() {
            region.push(x);
            for k in 0..3 {
                let h = (3 * x + k) as u32;
                let g = tw[h as usize];
                if g == NONE {
                    continue;
                }
                let u = (g / 3) as usize;
                if !vis[u] {
                    vis[u] = true;
                    let same = he_from(t, h) == he_from(t, g);
                    flip[u] = flip[x] ^ same;
                    stack.push(u);
                }
            }
        }
        let nf = region.iter().filter(|&&x| flip[x]).count();
        let invert = nf * 2 > region.len();
        for &x in &region {
            if flip[x] != invert {
                t[x].swap(1, 2);
                n += 1;
            }
        }
    }
    n
}

/// Zwillinge entfernen, die nach dem Ausrichten noch gleich gerichtet sind (nicht orientierbar).
fn drop_bad_twins(t: &[[u32; 3]], tw: &mut [u32]) {
    for h in 0..tw.len() {
        let g = tw[h];
        if g != NONE && he_from(t, h as u32) == he_from(t, g) {
            tw[h] = NONE;
            tw[g as usize] = NONE;
        }
    }
}

/// Ecken mit mehreren Dreiecksfächern (und damit auch nicht-mannigfaltige Kanten)
/// auftrennen: jeder Fächer bekommt eine eigene Ecke. Rückgabe: Zahl neuer Ecken.
fn split_vertices(w: &mut W, tw: &[u32]) -> usize {
    let nt = w.t.len();
    let mut cd = Dsu::new(nt * 3);
    for h in 0..tw.len() {
        let g = tw[h];
        if g == NONE || (g as usize) < h {
            continue;
        }
        let (t1, k1) = (h / 3, h % 3);
        let (t2, k2) = ((g / 3) as usize, (g % 3) as usize);
        // h: a -> b in t1, g: b -> a in t2
        cd.union((3 * t1 + k1) as u32, (3 * t2 + (k2 + 1) % 3) as u32);
        cd.union((3 * t1 + (k1 + 1) % 3) as u32, (3 * t2 + k2) as u32);
    }
    let mut vc: Vec<(u32, u32)> = Vec::with_capacity(nt * 3);
    for (i, x) in w.t.iter().enumerate() {
        for k in 0..3 {
            vc.push((x[k], (3 * i + k) as u32));
        }
    }
    vc.sort_unstable();
    let mut n_new = 0;
    let mut i = 0;
    while i < vc.len() {
        let mut j = i;
        while j < vc.len() && vc[j].0 == vc[i].0 {
            j += 1;
        }
        let vtx = vc[i].0;
        let mut root_map: Vec<(u32, u32)> = Vec::new(); // Wurzel -> Ecke
        for q in &vc[i..j] {
            let r = cd.find(q.1);
            let target = match root_map.iter().find(|x| x.0 == r) {
                Some(x) => x.1,
                None => {
                    let id = if root_map.is_empty() {
                        vtx
                    } else {
                        n_new += 1;
                        w.v.push(w.v[vtx as usize]);
                        (w.v.len() - 1) as u32
                    };
                    root_map.push((r, id));
                    id
                }
            };
            let (tt, kk) = ((q.1 / 3) as usize, (q.1 % 3) as usize);
            w.t[tt][kk] = target;
        }
        i = j;
    }
    n_new
}

/// Zusammenhängende Teile über die Zwillingskanten.
fn shells(v: &[V3], t: &[[u32; 3]], tw: &[u32]) -> Vec<Shell> {
    let nt = t.len();
    let mut seen = vec![false; nt];
    let mut out = Vec::new();
    let mut stack = Vec::new();
    for s in 0..nt {
        if seen[s] {
            continue;
        }
        seen[s] = true;
        stack.push(s);
        let mut list = Vec::new();
        let mut closed = true;
        let (mut vol, mut area) = (0.0, 0.0);
        let mut mn = v3(f64::MAX, f64::MAX, f64::MAX);
        let mut mx = -mn;
        while let Some(x) = stack.pop() {
            list.push(x as u32);
            let (a, b, c) = tv(v, &t[x]);
            vol += a.dot(b.cross(c)) / 6.0;
            area += (b - a).cross(c - a).norm() * 0.5;
            mn = mn.min(a).min(b).min(c);
            mx = mx.max(a).max(b).max(c);
            for k in 0..3 {
                let g = tw[3 * x + k];
                if g == NONE {
                    closed = false;
                    continue;
                }
                let u = (g / 3) as usize;
                if !seen[u] {
                    seen[u] = true;
                    stack.push(u);
                }
            }
        }
        out.push(Shell {
            tris: list,
            closed,
            volume: vol,
            area,
            bmin: mn,
            bmax: mx,
        });
    }
    out
}

/// Randschleifen (Löcher) als Eckenfolge in Füllrichtung. Der Rand wird über die
/// Dreiecksfächer verfolgt, damit auch Ecken, an denen sich zwei Löcher berühren,
/// richtig zugeordnet werden; solche Schleifen werden an der Berührstelle geteilt.
fn hole_loops(t: &[[u32; 3]], tw: &[u32]) -> Vec<Vec<u32>> {
    let next = |h: u32| h - h % 3 + (h % 3 + 1) % 3;
    let mut used = vec![false; tw.len()];
    let mut loops = Vec::new();
    for h0 in 0..tw.len() as u32 {
        if tw[h0 as usize] != NONE || used[h0 as usize] || idx_degenerate(&t[(h0 / 3) as usize]) {
            continue;
        }
        let mut chain = Vec::new();
        let mut h = h0;
        let mut ok = false;
        for _ in 0..tw.len() {
            used[h as usize] = true;
            chain.push(he_from(t, h));
            // nächste Randkante ab dem Endpunkt im selben Fächer
            let mut g = next(h);
            let mut guard = 0;
            while tw[g as usize] != NONE && guard < 4096 {
                g = next(tw[g as usize]);
                guard += 1;
            }
            if tw[g as usize] != NONE {
                break;
            }
            if g == h0 {
                ok = true;
                break;
            }
            if used[g as usize] {
                break;
            }
            h = g;
        }
        if !ok {
            continue;
        }
        chain.reverse();
        // an mehrfach besuchten Ecken in einfache Schleifen zerlegen
        let mut stack: Vec<u32> = Vec::new();
        let mut pos: HashMap<u32, usize> = HashMap::new();
        for &vtx in chain.iter().chain(std::iter::once(&chain[0])) {
            if let Some(&p) = pos.get(&vtx) {
                let sub: Vec<u32> = stack[p..].to_vec();
                for x in &sub[1..] {
                    pos.remove(x);
                }
                stack.truncate(p + 1);
                if sub.len() >= 3 {
                    loops.push(sub);
                }
            } else {
                pos.insert(vtx, stack.len());
                stack.push(vtx);
            }
        }
    }
    loops
}

/// Loch triangulieren. Kleine Löcher: Triangulierung mit kleinster Fläche (dynamische
/// Programmierung, mit Strafe für umgeklappte Dreiecke); große: Ohrenschneiden in der
/// Ausgleichsebene bzw. Fächer um den Schwerpunkt.
fn fill_loop(w: &mut W, lp: &[u32], edges: &mut HashSet<(u32, u32)>) -> usize {
    let n = lp.len();
    let p: Vec<V3> = lp.iter().map(|&i| w.v[i as usize]).collect();
    // Normale nach Newell
    let mut nn = V3::ZERO;
    for i in 0..n {
        let (a, b) = (p[i], p[(i + 1) % n]);
        nn = nn + v3((a.y - b.y) * (a.z + b.z), (a.z - b.z) * (a.x + b.x), (a.x - b.x) * (a.y + b.y));
    }
    let nl = nn.norm();
    let nu = if nl > 0.0 { nn / nl } else { V3::Z };
    let ekey = |a: u32, b: u32| (a.min(b), a.max(b));
    let add = |w: &mut W, x: [u32; 3], edges: &mut HashSet<(u32, u32)>| {
        for k in 0..3 {
            edges.insert(ekey(x[k], x[(k + 1) % 3]));
        }
        w.push(x, true);
    };
    if n == 3 {
        add(w, [lp[0], lp[1], lp[2]], edges);
        return 1;
    }
    if n <= 400 {
        let cost = |i: usize, j: usize, k: usize| -> f64 {
            let cr = (p[j] - p[i]).cross(p[k] - p[i]);
            let a = cr.norm() * 0.5;
            let l = (p[j] - p[i]).norm2().max((p[k] - p[j]).norm2()).max((p[i] - p[k]).norm2());
            if 2.0 * a <= 1e-9 * l {
                // Dreiecke ohne Fläche (Ecken auf einer Linie) möglichst vermeiden
                return 1e4 * l + 1.0;
            }
            let d = (cr / (2.0 * a)).dot(nu);
            // Diagonalen, die es im Netz schon gibt, würden nicht-mannigfaltige Kanten erzeugen
            let mut dup = 0.0;
            for (x, y) in [(i, j), (j, k), (i, k)] {
                let boundary = y == x + 1 || (x == 0 && y == n - 1);
                if !boundary && lp[x] != lp[y] && edges.contains(&ekey(lp[x], lp[y])) {
                    dup += 1.0;
                }
            }
            a * (1.0 + 2.0 * (1.0 - d)) + if d < 0.0 { a * 50.0 } else { 0.0 } + dup * (1e4 * l + 1.0)
        };
        let mut c = vec![0.0f64; n * n];
        let mut best = vec![0u32; n * n];
        for len in 2..n {
            for i in 0..n - len {
                let j = i + len;
                let mut bv = f64::MAX;
                let mut bk = i + 1;
                for k in i + 1..j {
                    let v = c[i * n + k] + c[k * n + j] + cost(i, k, j);
                    if v < bv {
                        bv = v;
                        bk = k;
                    }
                }
                c[i * n + j] = bv;
                best[i * n + j] = bk as u32;
            }
        }
        let mut stack = vec![(0usize, n - 1)];
        let mut cnt = 0;
        while let Some((i, j)) = stack.pop() {
            if j < i + 2 {
                continue;
            }
            let k = best[i * n + j] as usize;
            add(w, [lp[i], lp[k], lp[j]], edges);
            cnt += 1;
            stack.push((i, k));
            stack.push((k, j));
        }
        return cnt;
    }
    // große Löcher: Projektion auf die Ausgleichsebene
    let (e1, e2) = nu.basis();
    let o = p[0];
    let ring: Vec<P2> = p.iter().map(|q| [(*q - o).dot(e1), (*q - o).dot(e2)]).collect();
    if poly::signed_area(&ring) > 0.0 && crate::split::is_simple(&ring) {
        let tris = poly::triangulate(&[ring]);
        if tris.len() == n - 2 {
            for x in &tris {
                add(w, [lp[x[0]], lp[x[1]], lp[x[2]]], edges);
            }
            return tris.len();
        }
    }
    let mut cen = V3::ZERO;
    for q in &p {
        cen = cen + *q;
    }
    w.v.push(cen / n as f64);
    let ci = (w.v.len() - 1) as u32;
    for i in 0..n {
        add(w, [lp[i], lp[(i + 1) % n], ci], edges);
    }
    n
}

/// Topologie aufbauen, ausrichten, Ecken trennen, Löcher füllen, nach außen ausrichten.
fn close_up(w: &mut W, o: &RepairOpts, first: bool) {
    let mut tw = topology(&w.v, &w.t);
    let n_flip = orient(&mut w.t, &tw);
    tw = topology(&w.v, &w.t);
    drop_bad_twins(&w.t, &mut tw);
    if first {
        w.note(n_flip, "1 falsch orientiertes Dreieck gedreht", "{n} falsch orientierte Dreiecke gedreht");
    }
    // lose Kleinteile
    if o.remove_small && first {
        let sh = shells(&w.v, &w.t, &tw);
        if sh.len() > 1 {
            let total: f64 = sh.iter().map(|s| s.area).sum();
            let largest = sh.iter().map(|s| s.area).fold(0.0, f64::max);
            let mut keep = vec![true; w.t.len()];
            let mut n = 0;
            for s in &sh {
                if (s.area < total * o.small_frac || (!s.closed && s.tris.len() <= 4)) && s.area < largest {
                    n += 1;
                    for &x in &s.tris {
                        keep[x as usize] = false;
                    }
                }
            }
            if n > 0 {
                w.retain(&keep);
                w.note(n, "1 loses Kleinteil entfernt", "{n} lose Kleinteile entfernt");
                tw = topology(&w.v, &w.t);
                drop_bad_twins(&w.t, &mut tw);
            }
        }
    }
    // Löcher
    if o.fill_holes {
        let loops = hole_loops(&w.t, &tw);
        let mut edges: HashSet<(u32, u32)> = w.t.iter().flat_map(|x| (0..3).map(move |k| (x[k].min(x[(k + 1) % 3]), x[k].max(x[(k + 1) % 3])))).collect();
        let (mut nh, mut nt, mut skipped) = (0, 0, 0);
        for lp in &loops {
            if o.max_hole > 0 && lp.len() > o.max_hole {
                skipped += 1;
                continue;
            }
            nt += fill_loop(w, lp, &mut edges);
            nh += 1;
        }
        if nh > 0 {
            w.log.push(if nh == 1 {
                format!("1 Loch gefüllt ({nt} Dreiecke)")
            } else {
                format!("{nh} Löcher gefüllt ({nt} Dreiecke)")
            });
        }
        w.note(skipped, "1 großes Loch offen gelassen", "{n} große Löcher offen gelassen");
        tw = topology(&w.v, &w.t);
        drop_bad_twins(&w.t, &mut tw);
        // neue Dreiecke einheitlich ausrichten (sollte bereits passen)
        if orient(&mut w.t, &tw) > 0 {
            tw = topology(&w.v, &w.t);
            drop_bad_twins(&w.t, &mut tw);
        }
    }
    if o.nonmanifold {
        let n = split_vertices(w, &tw);
        if first {
            w.note(n, "1 nicht-mannigfaltige Stelle getrennt", "{n} nicht-mannigfaltige Stellen getrennt");
        }
        tw = topology(&w.v, &w.t);
        drop_bad_twins(&w.t, &mut tw);
    }
    // Körper nach außen ausrichten, Hohlräume erkennen, innenliegende Körper
    let sh = shells(&w.v, &w.t, &tw);
    let depth = nesting(&w.v, &w.t, &sh);
    let (mut n_inv, mut n_inner) = (0, 0);
    let mut keep = vec![true; w.t.len()];
    let main_area = sh.iter().map(|s| s.area).fold(0.0, f64::max);
    let diag = {
        let (a, b) = bbox(&w.v);
        (b - a).norm().max(1e-9)
    };
    let mut n_flat = 0;
    for (i, s) in sh.iter().enumerate() {
        // flache „Kissen“ ohne Volumen (z. B. zugeklappte Doppeldreiecke) neben einem größeren Körper
        if s.closed && s.area < main_area && s.volume.abs() <= s.area * diag * 1e-6 {
            n_flat += 1;
            for &x in &s.tris {
                keep[x as usize] = false;
            }
            continue;
        }
        if !s.closed {
            // offene Teile: grob nach außen (positives Volumen) drehen
            if s.volume < 0.0 && depth[i] == 0 {
                for &x in &s.tris {
                    w.t[x as usize].swap(1, 2);
                }
            }
            continue;
        }
        if depth[i] % 2 == 0 {
            if s.volume < 0.0 {
                n_inv += 1;
                for &x in &s.tris {
                    w.t[x as usize].swap(1, 2);
                }
            }
        } else if s.volume > 0.0 && o.remove_inner {
            n_inner += 1;
            for &x in &s.tris {
                keep[x as usize] = false;
            }
        }
    }
    if n_inner + n_flat > 0 {
        w.retain(&keep);
    }
    w.note(n_flat, "1 flacher Rest ohne Volumen entfernt", "{n} flache Reste ohne Volumen entfernt");
    if first {
        w.note(n_inv, "1 umgestülpter Körper umgedreht", "{n} umgestülpte Körper umgedreht");
    }
    w.note(n_inner, "1 innenliegender Körper entfernt", "{n} innenliegende Körper entfernt");
}

/// Repariert ein Netz.
/// Übrig gebliebene „Flossen“ entfernen: Dreiecke an Kanten mit mehr als zwei Dreiecken, die
/// keinen Partner gefunden haben – und das entstehende Loch schließen.
fn remove_fins(w: &mut W, o: &RepairOpts) {
    if o.nonmanifold {
        let mut total = 0;
        for _ in 0..3 {
            let tw = topology(&w.v, &w.t);
            let em = edge_map(&w.t);
            let mut keep = vec![true; w.t.len()];
            let mut n = 0;
            for list in em.values() {
                if list.len() > 2 {
                    // Flächen, die sich nur an der Kante berühren (alle Dreiecke gepaart), bleiben
                    for h in list.iter().copied().filter(|&h| tw[h as usize] == NONE) {
                        if keep[(h / 3) as usize] {
                            keep[(h / 3) as usize] = false;
                            n += 1;
                        }
                    }
                }
            }
            if n == 0 {
                break;
            }
            total += n;
            w.retain(&keep);
            let mut oo = o.clone();
            oo.fill_holes = true;
            close_up(w, &oo, false);
        }
        w.note(total, "1 abstehendes Dreieck entfernt", "{n} abstehende Dreiecke entfernt");
    }
}

pub fn repair(raw: &RawMesh, o: &RepairOpts) -> Repaired {
    let (mn, mx) = bbox(&raw.verts);
    let diag = (mx - mn).norm().max(1e-9);
    let gap = if o.gap > 0.0 { o.gap } else { diag * 2e-5 };
    let (v, t) = weld(&raw.verts, &raw.tris, (diag * 1e-6).min(gap));
    let mut w = W {
        filled: vec![false; t.len()],
        v,
        t,
        log: Vec::new(),
    };
    let n0 = w.v.len();
    clean(&mut w);
    zip_gaps(&mut w, gap);
    close_up(&mut w, o, true);
    // übrig gebliebene „Flossen“: Dreiecke an Kanten mit mehr als zwei Dreiecken, die keinen
    // Partner gefunden haben – entfernen und das entstehende Loch schließen
    remove_fins(&mut w, o);
    if o.intersections {
        // Stand vor dem Beheben merken: wird das Netz dadurch schlechter (offen, zerfallen,
        // umgestülpte Splitter), wird dieser Schritt zurückgenommen
        let backup = (w.v.clone(), w.t.clone(), w.filled.clone(), w.log.len());
        let score = |v: &[V3], t: &[[u32; 3]]| {
            let a = analyze_indexed(v.to_vec(), t.to_vec(), false).issues;
            a.open_edges + a.nonmanifold_edges + a.nonmanifold_verts + a.flipped + 10 * a.inverted_shells + 10 * a.shells.saturating_sub(1)
        };
        let score0 = score(&w.v, &w.t);
        let mut total = 0;
        for _ in 0..3 {
            let Some(hit) = find_intersections(&w.v, &w.t, &[]) else { break };
            let n = hit.iter().filter(|&&x| x).count();
            if n == 0 {
                break;
            }
            if n * 4 > w.t.len() {
                w.log.push(format!("Zu viele Überschneidungen ({n} Dreiecke) – nicht automatisch behoben"));
                break;
            }
            // überschneidende Dreiecke samt direkten Nachbarn entfernen und neu schließen
            let tw = topology(&w.v, &w.t);
            let mut keep = vec![true; w.t.len()];
            for (i, &x) in hit.iter().enumerate() {
                if x {
                    keep[i] = false;
                    for k in 0..3 {
                        let g = tw[3 * i + k];
                        if g != NONE {
                            keep[(g / 3) as usize] = false;
                        }
                    }
                }
            }
            total += n;
            w.retain(&keep);
            let mut oo = o.clone();
            oo.fill_holes = true;
            oo.max_hole = 0;
            close_up(&mut w, &oo, false);
        }
        if total > 0 {
            // Splitter aufräumen, die beim Neuschließen entstanden sind
            clean_quiet(&mut w);
            remove_fins(&mut w, o);
            close_up(&mut w, o, true);
            let score1 = score(&w.v, &w.t);
            if score1 > score0 {
                let (v, t, f, nlog) = backup;
                w.v = v;
                w.t = t;
                w.filled = f;
                w.log.truncate(nlog);
                w.log.push(format!("Überschneidungen ({total} Dreiecke) ließen sich nicht sauber beheben – Netz unverändert gelassen"));
                total = 0;
            }
        }
        w.note(total, "1 Überschneidung entfernt", "{n} überschneidende Dreiecke entfernt und neu geschlossen");
    }
    let _ = n0;
    // unbenutzte Ecken entfernen
    let mut map = vec![NONE; w.v.len()];
    let mut nv = Vec::new();
    for x in w.t.iter_mut() {
        for i in x.iter_mut() {
            if map[*i as usize] == NONE {
                map[*i as usize] = nv.len() as u32;
                nv.push(w.v[*i as usize]);
            }
            *i = map[*i as usize];
        }
    }
    let after = analyze_indexed(nv.clone(), w.t.clone(), true);
    if w.log.is_empty() {
        w.log.push("Keine Fehler gefunden".into());
    }
    Repaired {
        name: raw.name.clone(),
        verts: nv,
        tris: w.t,
        filled: w.filled,
        after,
        log: w.log,
    }
}

/// Reparierte Objekte als Datei: `stl` (alle Objekte in einer Datei) oder `3mf`
/// (je Objekt ein eigenes Objekt, für Bambu Studio / OrcaSlicer / PrusaSlicer).
pub fn export(items: &[Repaired], format: &str) -> Vec<u8> {
    let parts: Vec<crate::split::Part> = items
        .iter()
        .enumerate()
        .map(|(i, r)| {
            let mut p = crate::split::Part {
                name: r.name.clone(),
                verts: r.verts.clone(),
                tris: r.tris.clone(),
                col: vec![1; r.tris.len()],
                color: 1,
                group: i,
                ..Default::default()
            };
            p.update_stats();
            p
        })
        .collect();
    if format.eq_ignore_ascii_case("3mf") {
        let names: Vec<String> = items.iter().map(|r| r.name.clone()).collect();
        crate::export3mf::write_3mf(&parts, &names, &crate::export3mf::ExportOpts { assembly: true, ..Default::default() })
    } else {
        let mut all = crate::split::Part {
            name: items.first().map(|r| r.name.clone()).unwrap_or_default(),
            ..Default::default()
        };
        for p in &parts {
            let off = all.verts.len() as u32;
            all.verts.extend(&p.verts);
            all.tris.extend(p.tris.iter().map(|t| [t[0] + off, t[1] + off, t[2] + off]));
        }
        crate::export3mf::write_stl(&all)
    }
}

// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// Würfel als Dreieckssuppe (jedes Dreieck mit eigenen Ecken, wie in STL).
    fn cube(o: V3, s: f64) -> RawMesh {
        let p = |x: f64, y: f64, z: f64| o + v3(x * s, y * s, z * s);
        let c = [p(0., 0., 0.), p(1., 0., 0.), p(1., 1., 0.), p(0., 1., 0.), p(0., 0., 1.), p(1., 0., 1.), p(1., 1., 1.), p(0., 1., 1.)];
        let f: [[usize; 3]; 12] = [
            [0, 2, 1],
            [0, 3, 2],
            [4, 5, 6],
            [4, 6, 7],
            [0, 1, 5],
            [0, 5, 4],
            [1, 2, 6],
            [1, 6, 5],
            [2, 3, 7],
            [2, 7, 6],
            [3, 0, 4],
            [3, 4, 7],
        ];
        let mut m = RawMesh {
            name: "Würfel".into(),
            ..Default::default()
        };
        for t in f {
            let b = m.verts.len() as u32;
            for i in t {
                m.verts.push(c[i]);
            }
            m.tris.push([b, b + 1, b + 2]);
        }
        m
    }

    fn merge(a: &RawMesh, b: &RawMesh) -> RawMesh {
        let mut m = a.clone();
        let off = m.verts.len() as u32;
        m.verts.extend(&b.verts);
        m.tris.extend(b.tris.iter().map(|t| [t[0] + off, t[1] + off, t[2] + off]));
        m
    }

    #[test]
    fn intact_cube_is_ok() {
        let a = analyze(&cube(V3::ZERO, 10.0), true);
        assert!(a.issues.watertight(), "{:?}", a.issues);
        assert_eq!(a.issues.defects(), 0);
        assert!((a.issues.volume - 1000.0).abs() < 1e-6);
        let r = repair(&cube(V3::ZERO, 10.0), &RepairOpts::default());
        assert_eq!(r.tris.len(), 12);
        assert_eq!(r.after.issues.defects(), 0);
    }

    #[test]
    fn hole_and_flip() {
        let mut m = cube(V3::ZERO, 10.0);
        m.tris.remove(2); // Loch im Deckel
        m.tris[5].swap(1, 2); // ein Dreieck falsch herum
        let a = analyze(&m, false);
        assert_eq!(a.issues.holes, 1);
        assert_eq!(a.issues.flipped, 1);
        let r = repair(&m, &RepairOpts::default());
        assert!(r.after.issues.watertight(), "{:?} {:?}", r.after.issues, r.log);
        assert!((r.after.issues.volume - 1000.0).abs() < 1e-6);
        assert_eq!(r.filled.iter().filter(|&&f| f).count(), 1);
    }

    #[test]
    fn inverted_and_big_hole() {
        let mut m = cube(V3::ZERO, 10.0);
        for t in m.tris.iter_mut() {
            t.swap(1, 2);
        }
        let a = analyze(&m, false);
        assert_eq!(a.issues.inverted_shells, 1);
        // ganze Seite entfernen -> viereckiges Loch
        m.tris.drain(8..10);
        let r = repair(&m, &RepairOpts::default());
        assert!(r.after.issues.watertight(), "{:?}", r.after.issues);
        assert!((r.after.issues.volume - 1000.0).abs() < 1e-6, "{}", r.after.issues.volume);
    }

    #[test]
    fn cubes_sharing_edge_are_split() {
        let m = merge(&cube(V3::ZERO, 10.0), &cube(v3(10.0, 10.0, 0.0), 10.0));
        let a = analyze(&m, false);
        assert!(a.issues.nonmanifold_edges > 0);
        let r = repair(&m, &RepairOpts::default());
        assert!(r.after.issues.watertight(), "{:?}", r.after.issues);
        assert_eq!(r.after.issues.shells, 2);
        assert!((r.after.issues.volume - 2000.0).abs() < 1e-6);
    }

    #[test]
    fn crack_and_tjunction() {
        // Würfel, dessen Deckel leicht verschoben ist (Spalt)
        let mut m = cube(V3::ZERO, 10.0);
        // Deckeldreiecke (2, 3) um 0,0005 mm anheben
        for t in [2usize, 3] {
            for k in 0..3 {
                let vi = m.tris[t][k] as usize;
                m.verts[vi].z += 0.0005;
            }
        }
        let a = analyze(&m, false);
        assert!(a.issues.open_edges > 0);
        let r = repair(&m, &RepairOpts { gap: 0.002, ..Default::default() });
        assert!(r.after.issues.watertight(), "{:?} {:?}", r.after.issues, r.log);
        assert_eq!(r.filled.iter().filter(|&&f| f).count(), 0, "{:?}", r.log);
    }

    #[test]
    fn tjunction_split() {
        // Quadrat oben aus 2 Dreiecken, unten aus 3 (Mittelpunkt auf der Kante) -> T-Stoß
        let mut m = RawMesh::default();
        let p = [v3(0., 0., 0.), v3(10., 0., 0.), v3(10., 10., 0.), v3(0., 10., 0.), v3(5., 0., 0.), v3(5., 5., 0.)];
        m.verts = p.to_vec();
        m.tris = vec![[0, 4, 5], [4, 1, 5], [1, 2, 5], [5, 2, 3], [0, 5, 3]];
        // Rückseite als zwei große Dreiecke (ohne Ecken 4 und 5): Ecke 4 liegt auf Kante 0-1
        m.tris.push([0, 3, 2]);
        m.tris.push([0, 2, 1]);
        let a = analyze(&m, false);
        assert!(a.issues.open_edges > 0);
        let r = repair(&m, &RepairOpts::default());
        assert_eq!(r.after.issues.open_edges, 0, "{:?}", r.log);
    }

    #[test]
    fn small_shell_and_inner_body() {
        let big = cube(V3::ZERO, 20.0);
        let inner = cube(v3(5.0, 5.0, 5.0), 5.0);
        let speck = cube(v3(40.0, 0.0, 0.0), 0.1);
        let m = merge(&merge(&big, &inner), &speck);
        let a = analyze(&m, false);
        assert_eq!(a.issues.shells, 3);
        assert_eq!(a.issues.inner_shells, 1);
        assert_eq!(a.issues.small_shells, 1);
        let r = repair(&m, &RepairOpts::default());
        assert_eq!(r.after.issues.shells, 1, "{:?}", r.log);
        // Hohlraum (nach innen orientiert) bleibt erhalten
        let mut void = cube(v3(5.0, 5.0, 5.0), 5.0);
        for t in void.tris.iter_mut() {
            t.swap(1, 2);
        }
        let r = repair(&merge(&big, &void), &RepairOpts::default());
        assert_eq!(r.after.issues.shells, 2);
        assert_eq!(r.after.issues.voids, 1);
        assert!((r.after.issues.volume - (8000.0 - 125.0)).abs() < 1e-6);
    }

    #[test]
    fn duplicates_and_degenerate() {
        let mut m = cube(V3::ZERO, 10.0);
        let t0 = m.tris[0];
        m.tris.push(t0);
        m.tris.push([0, 0, 1]);
        let a = analyze(&m, false);
        assert_eq!(a.issues.duplicate, 1);
        assert!(a.issues.degenerate >= 1);
        let r = repair(&m, &RepairOpts::default());
        assert_eq!(r.tris.len(), 12);
        assert!(r.after.issues.watertight());
    }

    #[test]
    fn intersections_detected() {
        let m = merge(&cube(V3::ZERO, 10.0), &cube(v3(5.0, 5.0, 5.0), 10.0));
        let a = analyze(&m, true);
        assert!(a.issues.intersecting.unwrap() > 0);
        let m = merge(&cube(V3::ZERO, 10.0), &cube(v3(5.3, 4.7, 5.1), 10.0));
        assert!(analyze(&m, true).issues.intersecting.unwrap() > 0);
        // zwei Würfel, die sich nur an einer Kante berühren, überschneiden sich nicht
        let r = repair(&merge(&cube(V3::ZERO, 10.0), &cube(v3(10.0, 10.0, 0.0), 10.0)), &RepairOpts::default());
        assert_eq!(r.after.issues.intersecting, Some(0));
        let b = analyze(&cube(V3::ZERO, 10.0), true);
        assert_eq!(b.issues.intersecting, Some(0));
    }

    #[test]
    fn big_hole_large_loop() {
        // Zylindermantel mit 600 Segmenten, Boden geschlossen, Deckel offen
        let n = 600;
        let mut m = RawMesh::default();
        for i in 0..n {
            let a = i as f64 / n as f64 * std::f64::consts::TAU;
            m.verts.push(v3(a.cos() * 10.0, a.sin() * 10.0, 0.0));
            m.verts.push(v3(a.cos() * 10.0, a.sin() * 10.0, 10.0));
        }
        m.verts.push(v3(0.0, 0.0, 0.0));
        let c = (2 * n) as u32;
        for i in 0..n as u32 {
            let j = (i + 1) % n as u32;
            m.tris.push([2 * i, 2 * j, 2 * j + 1]);
            m.tris.push([2 * i, 2 * j + 1, 2 * i + 1]);
            m.tris.push([c, 2 * j, 2 * i]);
        }
        let r = repair(&m, &RepairOpts::default());
        assert!(r.after.issues.watertight(), "{:?}", r.after.issues);
        let exact = std::f64::consts::PI * 100.0 * 10.0;
        assert!((r.after.issues.volume - exact).abs() / exact < 0.01, "{}", r.after.issues.volume);
    }
}
