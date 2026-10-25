//! Oberflächenstruktur: Muster (Rändel, Waben, Leder …) als echte Geometrie auf
//! ein Netz prägen.
//!
//! Ablauf: Kanten der strukturierten Flächen fein unterteilen (Ränder bleiben
//! geschlossen), jeden Punkt entlang der geglätteten Normale um die Musterhöhe
//! nach außen schieben und das Ergebnis mit Fehlerquadriken wieder ausdünnen.
//! Ausgesparte Flächen und scharfe Kanten bleiben glatt; zur Grenze hin wird
//! die Höhe weich auf null geblendet.

use crate::math::{v3, V3};
use std::collections::HashMap;
use std::f64::consts::{PI, TAU};

/// Eingebaute Muster. Jedes hat in u die Periode 1 (für nahtloses Umwickeln).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Pattern {
    Knurl,
    Hex,
    Ribs,
    Bumps,
    Leather,
    Wood,
    Carbon,
    Waves,
    Stone,
    Fabric,
    Dots,
    Noise,
}

impl Pattern {
    pub const ALL: [Pattern; 12] = [
        Pattern::Knurl,
        Pattern::Hex,
        Pattern::Ribs,
        Pattern::Bumps,
        Pattern::Leather,
        Pattern::Wood,
        Pattern::Carbon,
        Pattern::Waves,
        Pattern::Stone,
        Pattern::Fabric,
        Pattern::Dots,
        Pattern::Noise,
    ];
    pub fn name_de(self) -> &'static str {
        match self {
            Pattern::Knurl => "Rändel",
            Pattern::Hex => "Waben",
            Pattern::Ribs => "Rillen",
            Pattern::Bumps => "Noppen",
            Pattern::Leather => "Leder",
            Pattern::Wood => "Holz",
            Pattern::Carbon => "Karbon",
            Pattern::Waves => "Wellen",
            Pattern::Stone => "Stein",
            Pattern::Fabric => "Gewebe",
            Pattern::Dots => "Punkte",
            Pattern::Noise => "Rauschen",
        }
    }
}

/// Eigene Höhenkarte (z. B. aus einem Bild), Werte 0..1, wird gekachelt.
#[derive(Clone, Debug)]
pub struct HeightMap {
    pub w: usize,
    pub h: usize,
    pub data: Vec<f32>,
}

impl HeightMap {
    fn sample(&self, u: f64, v: f64) -> f64 {
        if self.w == 0 || self.h == 0 {
            return 0.0;
        }
        let x = frac(u) * self.w as f64 - 0.5;
        let y = frac(v) * self.h as f64 - 0.5;
        let (x0, y0) = (x.floor(), y.floor());
        let (fx, fy) = (x - x0, y - y0);
        let at = |i: f64, j: f64| {
            let i = (i as i64).rem_euclid(self.w as i64) as usize;
            let j = (j as i64).rem_euclid(self.h as i64) as usize;
            self.data[j * self.w + i] as f64
        };
        let a = at(x0, y0) * (1.0 - fx) + at(x0 + 1.0, y0) * fx;
        let b = at(x0, y0 + 1.0) * (1.0 - fx) + at(x0 + 1.0, y0 + 1.0) * fx;
        a * (1.0 - fy) + b * fy
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Projection {
    /// dreiseitig überblendet – passt für fast alles
    Auto,
    /// um die Z-Achse gewickelt (Knöpfe, Griffe, Becher)
    Cylinder,
    Sphere,
    /// von oben (XY)
    Planar,
}

#[derive(Clone, Debug)]
pub struct TexOpts {
    pub pattern: Pattern,
    /// ersetzt `pattern`, wenn gesetzt
    pub custom: Option<HeightMap>,
    pub projection: Projection,
    /// Größe einer Musterperiode in mm
    pub scale: f64,
    /// Prägetiefe in mm (nach außen)
    pub depth: f64,
    /// Drehung des Musters in Grad
    pub rotation: f64,
    /// fast waagrechte Flächen (Deckel, Standfläche) glatt lassen
    pub skip_flat: bool,
    /// Grenzwinkel zur Waagrechten in Grad
    pub flat_deg: f64,
    /// je Eingangsdreieck: ausgespart (vom Nutzer gewählt)
    pub excluded: Vec<bool>,
    /// je Eingangsdreieck: ausdrücklich strukturieren (auch wenn flach)
    pub included: Vec<bool>,
    /// Kanten schärfer als dieser Winkel bleiben glatt
    pub crease_deg: f64,
    /// höchstens so viele Dreiecke beim Unterteilen
    pub max_tris: usize,
    /// erlaubte Abweichung beim Ausdünnen (mm); 0 = nicht ausdünnen
    pub simplify_tol: f64,
}

impl Default for TexOpts {
    fn default() -> Self {
        TexOpts {
            pattern: Pattern::Knurl,
            custom: None,
            projection: Projection::Auto,
            scale: 3.0,
            depth: 0.6,
            rotation: 0.0,
            skip_flat: true,
            flat_deg: 30.0,
            excluded: Vec::new(),
            included: Vec::new(),
            crease_deg: 50.0,
            max_tris: 1_500_000,
            simplify_tol: 0.01,
        }
    }
}

#[derive(Clone, Debug, Default)]
pub struct TexResult {
    pub verts: Vec<V3>,
    pub tris: Vec<[u32; 3]>,
    /// Dreiecke vor dem Ausdünnen
    pub subdivided: usize,
    /// verwendete Kantenlänge beim Unterteilen (mm)
    pub edge: f64,
    pub warnings: Vec<String>,
}

// ---------------------------------------------------------------------------
// Muster

#[inline]
fn frac(x: f64) -> f64 {
    x - x.floor()
}

#[inline]
fn tri(x: f64) -> f64 {
    (frac(x) - 0.5).abs() * 2.0
}

#[inline]
fn smooth(a: f64, b: f64, x: f64) -> f64 {
    let t = ((x - a) / (b - a)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

#[inline]
fn hash(i: i64, j: i64, k: i64) -> f64 {
    let mut h = (i as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15) ^ (j as u64).wrapping_mul(0xC2B2_AE3D_27D4_EB4F) ^ (k as u64).wrapping_mul(0x1656_67B1_9E37_79F9);
    h ^= h >> 31;
    h = h.wrapping_mul(0xBF58_476D_1CE4_E5B9);
    h ^= h >> 29;
    (h >> 11) as f64 / (1u64 << 53) as f64
}

/// Wertrauschen, in u mit Periode 1 (Gitter f je Einheit).
fn vnoise(u: f64, v: f64, f: i64, seed: i64) -> f64 {
    let x = u * f as f64;
    let y = v * f as f64;
    let (x0, y0) = (x.floor(), y.floor());
    let (mut fx, mut fy) = (x - x0, y - y0);
    fx = fx * fx * (3.0 - 2.0 * fx);
    fy = fy * fy * (3.0 - 2.0 * fy);
    let g = |i: f64, j: f64| hash((i as i64).rem_euclid(f), j as i64, seed);
    let a = g(x0, y0) * (1.0 - fx) + g(x0 + 1.0, y0) * fx;
    let b = g(x0, y0 + 1.0) * (1.0 - fx) + g(x0 + 1.0, y0 + 1.0) * fx;
    a * (1.0 - fy) + b * fy
}

fn fbm(u: f64, v: f64, oct: u32, seed: i64) -> f64 {
    let (mut s, mut amp, mut tot) = (0.0, 1.0, 0.0);
    for o in 0..oct {
        s += amp * vnoise(u, v, 2i64 << o, seed + o as i64);
        tot += amp;
        amp *= 0.5;
    }
    s / tot
}

/// Zellen: Abstand zum nächsten und zweitnächsten Punkt (Punkte je Zelle zufällig
/// verschoben; `jit` = 0 ergibt ein regelmäßiges Gitter). Periode 1 in u.
fn cells(u: f64, v: f64, n: i64, jit: f64, hex: bool, seed: i64) -> (f64, f64) {
    let x = u * n as f64;
    let y = v * n as f64 * if hex { 2.0 / 3f64.sqrt() } else { 1.0 };
    let (cx, cy) = (x.floor() as i64, y.floor() as i64);
    let (mut d1, mut d2) = (f64::MAX, f64::MAX);
    for dj in -2..=2 {
        for di in -2..=2 {
            let (i, j) = (cx + di, cy + dj);
            let wi = i.rem_euclid(n);
            let (mut px, mut py) = (i as f64 + 0.5, j as f64 + 0.5);
            if hex && j.rem_euclid(2) == 1 {
                px += 0.5;
            }
            px += (hash(wi, j, seed) - 0.5) * jit;
            py += (hash(wi, j, seed + 7) - 0.5) * jit;
            let dx = x - px;
            let mut dy = y - py;
            if hex {
                dy *= 3f64.sqrt() / 2.0;
            }
            let d = (dx * dx + dy * dy).sqrt();
            if d < d1 {
                d2 = d1;
                d1 = d;
            } else if d < d2 {
                d2 = d;
            }
        }
    }
    (d1, d2)
}

/// Höhe des Musters (0..1) an (u, v); Periode 1 in u.
pub fn height(p: Pattern, u: f64, v: f64) -> f64 {
    match p {
        Pattern::Knurl => tri(u + v).min(tri(u - v)),
        Pattern::Hex => {
            let (d1, d2) = cells(u, v, 1, 0.0, true, 0);
            smooth(0.0, 0.32, d2 - d1).sqrt()
        }
        Pattern::Ribs => 0.5 + 0.5 * (TAU * u).cos(),
        Pattern::Bumps => {
            let r = ((frac(u) - 0.5).powi(2) + (frac(v) - 0.5).powi(2)).sqrt() / 0.36;
            (1.0 - r * r).max(0.0).sqrt()
        }
        Pattern::Dots => {
            let r = ((frac(u) - 0.5).powi(2) + (frac(v) - 0.5).powi(2)).sqrt() / 0.22;
            (1.0 - r * r).max(0.0).sqrt()
        }
        Pattern::Leather => {
            let n = fbm(u, v, 4, 7);
            let (d1, d2) = cells(u, v, 3, 0.9, false, 3);
            (smooth(0.0, 0.18, d2 - d1) * 0.75 + n * 0.25).clamp(0.0, 1.0)
        }
        Pattern::Wood => {
            let n = fbm(u, v * 0.25, 4, 21);
            0.5 + 0.5 * (TAU * (u * 2.0 + n * 1.2)).sin()
        }
        Pattern::Carbon => {
            // Köperbindung 2/2: diagonale Treppe aus Quer- und Längsfäden
            let (fu, fv) = (frac(u * 4.0), frac(v * 4.0));
            let a = ((u * 4.0).floor() + (v * 4.0).floor()).rem_euclid(4.0);
            let w = if a < 2.0 { (PI * fv).sin() } else { (PI * fu).sin() };
            0.15 + 0.85 * w.powf(0.7)
        }
        Pattern::Waves => 0.5 + 0.5 * (TAU * (v + 0.25 * (TAU * u).sin())).sin(),
        Pattern::Stone => {
            let (d1, d2) = cells(u, v, 2, 0.85, false, 5);
            let edge = smooth(0.0, 0.12, d2 - d1);
            (edge * (0.85 + 0.15 * fbm(u, v, 3, 9))).clamp(0.0, 1.0)
        }
        Pattern::Fabric => {
            let (fu, fv) = (frac(u * 2.0), frac(v * 2.0));
            let a = ((u * 2.0).floor() + (v * 2.0).floor()).rem_euclid(2.0);
            0.3 + 0.7 * if a == 0.0 { (PI * fv).sin() } else { (PI * fu).sin() }
        }
        Pattern::Noise => fbm(u, v, 5, 11),
    }
}

fn sample(o: &TexOpts, u: f64, v: f64) -> f64 {
    // Drehung um den Mittelpunkt der Musterzelle
    let (s, c) = o.rotation.to_radians().sin_cos();
    let (ru, rv) = (u * c - v * s, u * s + v * c);
    match &o.custom {
        Some(m) => m.sample(ru, rv),
        None => height(o.pattern, ru, rv),
    }
}

// ---------------------------------------------------------------------------
// Netz

fn tri_n(v: &[V3], t: &[u32; 3]) -> V3 {
    (v[t[1] as usize] - v[t[0] as usize]).cross(v[t[2] as usize] - v[t[0] as usize])
}

#[inline]
fn ek(a: u32, b: u32) -> (u32, u32) {
    if a < b {
        (a, b)
    } else {
        (b, a)
    }
}

/// Sinus der Neigung gegen die Waagrechte (0 = waagrecht, 1 = senkrecht).
fn flat_sin(verts: &[V3], t: &[u32; 3]) -> f64 {
    let n = tri_n(verts, t).normalized();
    (n.x * n.x + n.y * n.y).sqrt()
}

/// Liegt das Dreieck flacher als `flat_deg` (Deckel, Standfläche)?
pub fn is_flat(verts: &[V3], t: &[u32; 3], flat_deg: f64) -> bool {
    flat_sin(verts, t) < flat_deg.to_radians().sin()
}

/// Welche Eingangsdreiecke werden strukturiert?
pub fn textured_mask(verts: &[V3], tris: &[[u32; 3]], o: &TexOpts) -> Vec<bool> {
    let lim = o.flat_deg.to_radians().sin();
    tris.iter()
        .enumerate()
        .map(|(i, t)| {
            if o.excluded.get(i).copied().unwrap_or(false) {
                return false;
            }
            if o.included.get(i).copied().unwrap_or(false) {
                return true;
            }
            !(o.skip_flat && flat_sin(verts, t) < lim)
        })
        .collect()
}

/// Glatte Bereiche für die Auswahl per Klick: zusammenhängende Dreiecke ohne
/// scharfe Kante dazwischen. Rückgabe: Bereich je Dreieck.
pub fn smooth_regions(verts: &[V3], tris: &[[u32; 3]], crease_deg: f64) -> Vec<u32> {
    let mut edges: HashMap<(u32, u32), Vec<u32>> = HashMap::new();
    for (i, t) in tris.iter().enumerate() {
        for k in 0..3 {
            edges.entry(ek(t[k], t[(k + 1) % 3])).or_default().push(i as u32);
        }
    }
    let n: Vec<V3> = tris.iter().map(|t| tri_n(verts, t).normalized()).collect();
    let cl = crease_deg.to_radians().cos();
    let mut reg = vec![u32::MAX; tris.len()];
    let mut next = 0;
    for s in 0..tris.len() {
        if reg[s] != u32::MAX {
            continue;
        }
        reg[s] = next;
        let mut st = vec![s as u32];
        while let Some(a) = st.pop() {
            let t = tris[a as usize];
            for k in 0..3 {
                if let Some(nb) = edges.get(&ek(t[k], t[(k + 1) % 3])) {
                    if nb.len() != 2 {
                        continue;
                    }
                    for &b in nb {
                        if reg[b as usize] == u32::MAX && n[a as usize].dot(n[b as usize]) > cl {
                            reg[b as usize] = next;
                            st.push(b);
                        }
                    }
                }
            }
        }
        next += 1;
    }
    reg
}

/// Kanten länger als `l` an strukturierten Dreiecken halbieren, bis keine mehr
/// zu lang ist. Rückgabe: Punkte, Dreiecke, Herkunft (Eingangsdreieck) je Dreieck.
fn subdivide(verts: &[V3], tris: &[[u32; 3]], tex: &[bool], l: f64, max_tris: usize) -> (Vec<V3>, Vec<[u32; 3]>, Vec<u32>) {
    let mut v = verts.to_vec();
    let mut t = tris.to_vec();
    let mut par: Vec<u32> = (0..tris.len() as u32).collect();
    // bis 1,4·l: die Teilung landet dann bei etwa l
    let l2 = (l * 1.3) * (l * 1.3);
    for _pass in 0..24 {
        // zu lange Kanten an strukturierten Dreiecken markieren
        let mut mid: HashMap<(u32, u32), u32> = HashMap::new();
        for (i, tr) in t.iter().enumerate() {
            if !tex[par[i] as usize] {
                continue;
            }
            for k in 0..3 {
                let (a, b) = (tr[k], tr[(k + 1) % 3]);
                if v[a as usize].dist(v[b as usize]).powi(2) > l2 {
                    mid.entry(ek(a, b)).or_insert(u32::MAX);
                }
            }
        }
        if mid.is_empty() {
            break;
        }
        // grobe Schätzung des Zuwachses; bei Überschreitung abbrechen
        if t.len() + mid.len() * 2 > max_tris {
            break;
        }
        for (k, m) in mid.iter_mut() {
            *m = v.len() as u32;
            let p = (v[k.0 as usize] + v[k.1 as usize]) * 0.5;
            v.push(p);
        }
        let mut nt = Vec::with_capacity(t.len() * 2);
        let mut np = Vec::with_capacity(t.len() * 2);
        for (i, tr) in t.iter().enumerate() {
            let m: [Option<u32>; 3] = std::array::from_fn(|k| mid.get(&ek(tr[k], tr[(k + 1) % 3])).copied());
            let [a, b, c] = *tr;
            let p = par[i];
            let mut push = |x: [u32; 3]| {
                nt.push(x);
                np.push(p);
            };
            match (m[0], m[1], m[2]) {
                (None, None, None) => push(*tr),
                (Some(ab), Some(bc), Some(ca)) => {
                    push([a, ab, ca]);
                    push([ab, b, bc]);
                    push([ca, bc, c]);
                    push([ab, bc, ca]);
                }
                (Some(ab), None, None) => {
                    push([a, ab, c]);
                    push([ab, b, c]);
                }
                (None, Some(bc), None) => {
                    push([a, b, bc]);
                    push([a, bc, c]);
                }
                (None, None, Some(ca)) => {
                    push([a, b, ca]);
                    push([ca, b, c]);
                }
                (Some(ab), Some(bc), None) => {
                    push([ab, b, bc]);
                    // Viereck a-ab-bc-c an der kürzeren Diagonale teilen
                    if v[a as usize].dist(v[bc as usize]) < v[ab as usize].dist(v[c as usize]) {
                        push([a, ab, bc]);
                        push([a, bc, c]);
                    } else {
                        push([a, ab, c]);
                        push([ab, bc, c]);
                    }
                }
                (None, Some(bc), Some(ca)) => {
                    push([ca, bc, c]);
                    if v[a as usize].dist(v[bc as usize]) < v[b as usize].dist(v[ca as usize]) {
                        push([a, b, bc]);
                        push([a, bc, ca]);
                    } else {
                        push([a, b, ca]);
                        push([b, bc, ca]);
                    }
                }
                (Some(ab), None, Some(ca)) => {
                    push([a, ab, ca]);
                    if v[ab as usize].dist(v[c as usize]) < v[b as usize].dist(v[ca as usize]) {
                        push([ab, b, c]);
                        push([ab, c, ca]);
                    } else {
                        push([ab, b, ca]);
                        push([b, c, ca]);
                    }
                }
            }
        }
        t = nt;
        par = np;
    }
    (v, t, par)
}

/// Fläche der strukturierten Dreiecke.
fn textured_area(verts: &[V3], tris: &[[u32; 3]], tex: &[bool]) -> f64 {
    tris.iter().zip(tex).filter(|(_, &x)| x).map(|(t, _)| tri_n(verts, t).norm() * 0.5).sum()
}

/// Muster auf das Netz prägen.
pub fn apply(verts: &[V3], tris: &[[u32; 3]], o: &TexOpts) -> TexResult {
    let mut res = TexResult::default();
    if tris.is_empty() || o.scale <= 0.0 {
        res.verts = verts.to_vec();
        res.tris = tris.to_vec();
        return res;
    }
    let tex = textured_mask(verts, tris, o);
    // Kantenlänge: ~7 Punkte je Musterperiode, durch die Dreieckszahl begrenzt
    let area = textured_area(verts, tris, &tex);
    let mut l = (o.scale / 16.0).max(0.05);
    let budget = (o.max_tris as f64 * 0.8).max(1000.0);
    let need = area * 2.0 / (l * l);
    if need > budget {
        l *= (need / budget).sqrt();
        res.warnings.push(format!("Sehr feines Muster – Auflösung auf {:.2} mm begrenzt", l).replace('.', ","));
    }
    res.edge = l;
    let (mut v, t, par) = subdivide(verts, tris, &tex, l, o.max_tris);
    res.subdivided = t.len();
    let nv = v.len();

    // Normalen je Punkt (flächengewichtet), getrennt für Muster und alle
    let mut vn = vec![V3::ZERO; nv];
    for tr in &t {
        let n = tri_n(&v, tr);
        for &k in tr {
            vn[k as usize] += n;
        }
    }
    let vn: Vec<V3> = vn.into_iter().map(|n| n.normalized()).collect();

    // gesperrte Punkte: an ausgesparten Dreiecken oder auf scharfen Kanten
    let mut blocked = vec![false; nv];
    let mut touched = vec![false; nv];
    for (i, tr) in t.iter().enumerate() {
        let on = tex[par[i] as usize];
        for &k in tr {
            if on {
                touched[k as usize] = true;
            } else {
                blocked[k as usize] = true;
            }
        }
    }
    let cl = o.crease_deg.to_radians().cos();
    let mut edges: HashMap<(u32, u32), (u32, u32)> = HashMap::new();
    for (i, tr) in t.iter().enumerate() {
        for k in 0..3 {
            let e = edges.entry(ek(tr[k], tr[(k + 1) % 3])).or_insert((u32::MAX, u32::MAX));
            if e.0 == u32::MAX {
                e.0 = i as u32;
            } else {
                e.1 = i as u32;
            }
        }
    }
    let fnrm: Vec<V3> = t.iter().map(|tr| tri_n(&v, tr).normalized()).collect();
    for (&(a, b), &(x, y)) in &edges {
        let sharp = y == u32::MAX || fnrm[x as usize].dot(fnrm[y as usize]) < cl;
        if sharp {
            blocked[a as usize] = true;
            blocked[b as usize] = true;
        }
    }
    // Abstand zu gesperrten Punkten (entlang der Kanten), für weiches Ausblenden
    let fade = (o.scale * 0.5).max(l * 1.5);
    let mut dist = vec![f64::INFINITY; nv];
    let mut adj: Vec<Vec<u32>> = vec![Vec::new(); nv];
    for &(a, b) in edges.keys() {
        adj[a as usize].push(b);
        adj[b as usize].push(a);
    }
    {
        use std::cmp::Ordering;
        #[derive(PartialEq)]
        struct It(f64, u32);
        impl Eq for It {}
        impl PartialOrd for It {
            fn partial_cmp(&self, o: &Self) -> Option<Ordering> {
                Some(self.cmp(o))
            }
        }
        impl Ord for It {
            fn cmp(&self, o: &Self) -> Ordering {
                o.0.partial_cmp(&self.0).unwrap_or(Ordering::Equal)
            }
        }
        let mut h = std::collections::BinaryHeap::new();
        for i in 0..nv {
            if blocked[i] {
                dist[i] = 0.0;
                h.push(It(0.0, i as u32));
            }
        }
        while let Some(It(d, a)) = h.pop() {
            if d > dist[a as usize] || d > fade {
                continue;
            }
            for &b in &adj[a as usize] {
                let nd = d + v[a as usize].dist(v[b as usize]);
                if nd < dist[b as usize] {
                    dist[b as usize] = nd;
                    h.push(It(nd, b));
                }
            }
        }
    }

    // Projektion vorbereiten
    let (mut lo, mut hi) = (v3(f64::MAX, f64::MAX, f64::MAX), v3(f64::MIN, f64::MIN, f64::MIN));
    for p in verts {
        lo = lo.min(*p);
        hi = hi.max(*p);
    }
    let c = (lo + hi) * 0.5;
    let s = o.scale;
    // mittlerer Radius der strukturierten Punkte (Zylinder / Kugel)
    let (mut rc, mut rs, mut nr) = (0.0, 0.0, 0.0);
    for i in 0..nv {
        if touched[i] {
            let d = v[i] - c;
            rc += (d.x * d.x + d.y * d.y).sqrt();
            rs += d.norm();
            nr += 1.0;
        }
    }
    let rc = if nr > 0.0 { rc / nr } else { 1.0 }.max(1e-3);
    let rs = if nr > 0.0 { rs / nr } else { 1.0 }.max(1e-3);
    // ganze Zahl Perioden um den Umfang: keine Naht
    let per_c = (TAU * rc / s).round().max(1.0);
    let per_s = (TAU * rs / s).round().max(1.0);

    let h_at = |p: V3, n: V3| -> f64 {
        let planar = |a: f64, b: f64| sample(o, a / s, b / s);
        let tri3 = || {
            let w = [n.x.abs().powi(4), n.y.abs().powi(4), n.z.abs().powi(4)];
            let ws = (w[0] + w[1] + w[2]).max(1e-12);
            (w[0] * planar(p.y, p.z) + w[1] * planar(p.x, p.z) + w[2] * planar(p.x, p.y)) / ws
        };
        match o.projection {
            Projection::Auto => tri3(),
            Projection::Planar => planar(p.x, p.y),
            Projection::Cylinder => {
                let d = p - c;
                let ang = d.y.atan2(d.x) / TAU;
                let side = sample(o, ang * per_c, (p.z - lo.z) * per_c / (TAU * rc));
                // Deckel und Boden: von oben projiziert, weich überblendet
                let t = smooth(0.6, 0.85, n.z.abs());
                if t <= 0.0 {
                    side
                } else {
                    side * (1.0 - t) + planar(p.x, p.y) * t
                }
            }
            Projection::Sphere => {
                let d = (p - c).normalized();
                let lon = d.y.atan2(d.x) / TAU;
                let lat = d.z.clamp(-1.0, 1.0).asin() / TAU;
                sample(o, lon * per_s, lat * per_s)
            }
        }
    };

    let disp: Vec<f64> = (0..nv)
        .map(|i| {
            if !touched[i] || blocked[i] {
                return 0.0;
            }
            let w = smooth(0.0, fade, dist[i]);
            if w <= 0.0 {
                return 0.0;
            }
            o.depth * h_at(v[i], vn[i]).clamp(0.0, 1.0) * w
        })
        .collect();
    for i in 0..nv {
        if disp[i] != 0.0 {
            v[i] += vn[i] * disp[i];
        }
    }

    // wieder ausdünnen: glatte Bereiche werden grob, das Muster bleibt
    if o.simplify_tol > 0.0 {
        let so = crate::simplify::SimplifyOpts { ratio: 0.02, tol: o.simplify_tol };
        let (sv, st) = crate::simplify::simplify(&v, &t, &so);
        res.verts = sv;
        res.tris = st;
    } else {
        res.verts = v;
        res.tris = t;
    }
    if o.depth > 0.0 && (hi - lo).x.min((hi - lo).y).min((hi - lo).z) * 0.1 < o.depth {
        res.warnings.push("Tiefe ist groß im Verhältnis zum Modell".into());
    }
    res
}

/// Muster als kleines Graustufenbild (für Vorschaubilder in der App).
pub fn thumbnail(p: Pattern, n: usize) -> Vec<u8> {
    let mut hmap = vec![0f64; n * n];
    for j in 0..n {
        for i in 0..n {
            hmap[j * n + i] = height(p, i as f64 / n as f64 * 2.0, j as f64 / n as f64 * 2.0);
        }
    }
    let mut out = Vec::with_capacity(n * n * 4);
    let l = v3(-0.5, -0.6, 0.62).normalized();
    for j in 0..n {
        for i in 0..n {
            let at = |a: usize, b: usize| hmap[(b.min(n - 1)) * n + a.min(n - 1)];
            let gx = at(i + 1, j) - at(i.saturating_sub(1), j);
            let gy = at(i, j + 1) - at(i, j.saturating_sub(1));
            let nn = v3(-gx * 9.0, -gy * 9.0, 1.0).normalized();
            let s = nn.dot(l).max(0.0) * 0.75 + 0.25 * hmap[j * n + i];
            let g = |b: f64| (s * 205.0 + b).clamp(0.0, 255.0) as u8;
            out.extend_from_slice(&[g(30.0), g(32.0), g(38.0), 255]);
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cylinder(r: f64, h: f64, n: usize) -> (Vec<V3>, Vec<[u32; 3]>) {
        let mut v = vec![v3(0.0, 0.0, 0.0), v3(0.0, 0.0, h)];
        for i in 0..n {
            let a = TAU * i as f64 / n as f64;
            v.push(v3(r * a.cos(), r * a.sin(), 0.0));
            v.push(v3(r * a.cos(), r * a.sin(), h));
        }
        let mut t = Vec::new();
        for i in 0..n {
            let j = (i + 1) % n;
            let (b0, t0, b1, t1) = (2 + 2 * i as u32, 3 + 2 * i as u32, 2 + 2 * j as u32, 3 + 2 * j as u32);
            t.push([0, b1, b0]);
            t.push([1, t0, t1]);
            t.push([b0, b1, t1]);
            t.push([b0, t1, t0]);
        }
        (v, t)
    }

    fn closed(t: &[[u32; 3]]) -> bool {
        let mut e: HashMap<(u32, u32), i32> = HashMap::new();
        for tr in t {
            for k in 0..3 {
                *e.entry((tr[k], tr[(k + 1) % 3])).or_default() += 1;
            }
        }
        e.iter().all(|(&(a, b), &c)| c == 1 && e.get(&(b, a)) == Some(&1))
    }

    #[test]
    fn patterns_in_range() {
        for p in Pattern::ALL {
            for i in 0..50 {
                for j in 0..50 {
                    let h = height(p, i as f64 * 0.037 - 0.4, j as f64 * 0.041 - 0.7);
                    assert!((-1e-9..=1.0 + 1e-9).contains(&h), "{p:?} {h}");
                }
            }
            // Periode 1 in u
            for k in 0..20 {
                let (u, v) = (k as f64 * 0.05, k as f64 * 0.13);
                assert!((height(p, u, v) - height(p, u + 1.0, v)).abs() < 1e-6, "{p:?} nicht periodisch");
            }
        }
    }

    #[test]
    fn knurl_on_cylinder_stays_closed() {
        let (v, t) = cylinder(12.0, 20.0, 48);
        assert!(closed(&t));
        let o = TexOpts { projection: Projection::Cylinder, scale: 3.0, depth: 0.8, ..Default::default() };
        let r = apply(&v, &t, &o);
        assert!(closed(&r.tris), "Ergebnis nicht geschlossen");
        assert!(r.subdivided > t.len() * 20);
        // Mantel nach außen gewachsen, Deckel glatt geblieben
        let rmax = r.verts.iter().map(|p| (p.x * p.x + p.y * p.y).sqrt()).fold(0.0, f64::max);
        assert!(rmax > 12.3 && rmax < 12.0 + 0.8 + 0.01, "rmax {rmax}");
        let zmax = r.verts.iter().map(|p| p.z).fold(f64::MIN, f64::max);
        assert!((zmax - 20.0).abs() < 0.01, "Deckel verschoben: {zmax}");
        // ausgedünnt
        assert!(r.tris.len() < r.subdivided);
    }

    #[test]
    fn excluded_region_untouched() {
        let (v, t) = cylinder(10.0, 10.0, 32);
        let mut ex = vec![false; t.len()];
        // ganze Mantelfläche aussparen → nichts ändert sich
        let reg = smooth_regions(&v, &t, 50.0);
        let side = reg[2];
        for (i, r) in reg.iter().enumerate() {
            ex[i] = *r == side;
        }
        let o = TexOpts { excluded: ex, scale: 2.0, depth: 1.0, simplify_tol: 0.0, ..Default::default() };
        let r = apply(&v, &t, &o);
        let rmax = r.verts.iter().map(|p| (p.x * p.x + p.y * p.y).sqrt()).fold(0.0, f64::max);
        assert!(rmax <= 10.0 + 1e-9);
    }
}
