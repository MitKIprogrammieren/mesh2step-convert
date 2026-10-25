//! Aufbau der B-Rep-Topologie (Ecken, Kanten, Schleifen, Flächen) aus der Segmentierung.

use crate::math::{angle, solve, V3};
use crate::mesh::{Mesh, NONE};
use crate::segment::Segmentation;
use crate::surf::{self, Surf};
use std::collections::HashMap;

#[derive(Clone, Debug)]
pub enum Curve {
    Line { p: V3, d: V3 },
    /// Kreis: Mittelpunkt, Normale (Umlaufsinn), Referenzrichtung, Radius
    Circle { c: V3, n: V3, x: V3, r: f64 },
    /// Ellipse: x = Hauptachsenrichtung
    Ellipse { c: V3, n: V3, x: V3, a: f64, b: f64 },
    /// Polygonzug (B-Spline Grad 1); erster/letzter Punkt = Ecken
    Poly(Vec<V3>),
}

#[derive(Clone, Debug)]
pub struct Edge {
    pub v0: u32,
    pub v1: u32,
    pub curve: Curve,
    /// Netz-Halbkanten dieser Kante (für gezielte Nachbesserung)
    pub hes: Vec<u32>,
}

#[derive(Clone, Debug)]
pub struct Face {
    pub surf: Surf,
    /// true, wenn die Außennormale des Körpers der natürlichen Flächennormale entspricht
    pub same_sense: bool,
    /// Schleifen: Folge von (Kante, vorwärts)
    pub loops: Vec<Vec<(u32, bool)>>,
    /// Index der äußeren Schleife (bei Ebenen bestimmt)
    pub outer: Option<usize>,
    /// Region der Segmentierung, aus der die Fläche stammt
    pub region: u32,
}

#[derive(Clone, Debug)]
pub struct Shell {
    pub faces: Vec<u32>,
    pub closed: bool,
}

#[derive(Clone, Debug)]
pub struct Solid {
    pub name: String,
    pub outer: u32,
    pub voids: Vec<u32>,
}

#[derive(Clone, Debug, Default)]
pub struct Brep {
    pub verts: Vec<V3>,
    pub edges: Vec<Edge>,
    pub faces: Vec<Face>,
    pub shells: Vec<Shell>,
    pub solids: Vec<Solid>,
    pub tol: f64,
    /// enge Toleranz der Grundformen (bei Freiformteilen ist `tol` großzügiger)
    pub fit_tol: f64,
}

struct Chain {
    hes: Vec<u32>,
    r: u32,
    s: u32,
    closed: bool,
}

/// `force_poly`: Rand-Halbkanten, deren Kanten als Polygonzug statt als Gerade/Bogen
/// geschrieben werden (nach einem Selbstschnitt angenäherter Bögen in einer früheren Runde).
pub fn build(m: &Mesh, seg: &Segmentation, tol: f64, fit_tol: f64, name: &str, force_poly: &std::collections::HashSet<u32>) -> Brep {
    let reg = |t: u32| seg.region_of[t as usize];
    let reg_he = |h: u32| reg(h / 3);
    let nbr = |h: u32| -> u32 {
        let tw = m.twin[h as usize];
        if tw == NONE {
            NONE
        } else {
            reg(tw / 3)
        }
    };
    let is_bd = |h: u32| -> bool {
        let tw = m.twin[h as usize];
        tw == NONE || reg(tw / 3) != reg(h / 3)
    };
    let next_bd = |h: u32| -> u32 {
        let r = reg_he(h);
        let mut g = Mesh::he_next(h);
        for _ in 0..10000 {
            let tw = m.twin[g as usize];
            if tw == NONE || reg(tw / 3) != r {
                return g;
            }
            g = Mesh::he_next(tw);
        }
        g
    };
    let prev_bd = |h: u32| -> u32 {
        let r = reg_he(h);
        let mut g = Mesh::he_prev(h);
        for _ in 0..10000 {
            let tw = m.twin[g as usize];
            if tw == NONE || reg(tw / 3) != r {
                return g;
            }
            g = Mesh::he_prev(tw);
        }
        g
    };

    let nh = m.twin.len() as u32;
    // --- Ecken bestimmen ---
    let mut out_cnt: HashMap<u32, u32> = HashMap::new();
    let mut vregs: HashMap<u32, Vec<u32>> = HashMap::new();
    for h in 0..nh {
        if !is_bd(h) {
            continue;
        }
        let a = m.he_from(h);
        let b = m.he_to(h);
        *out_cnt.entry(a).or_insert(0) += 1;
        for v in [a, b] {
            let e = vregs.entry(v).or_default();
            for r in [reg_he(h), nbr(h)] {
                if !e.contains(&r) {
                    e.push(r);
                }
            }
        }
    }
    let mut corner: HashMap<u32, bool> = HashMap::new();
    for (&v, rs) in &vregs {
        let c = rs.len() >= 3 || out_cnt.get(&v).copied().unwrap_or(0) >= 3;
        corner.insert(v, c);
    }
    let is_corner = |v: u32, corner: &HashMap<u32, bool>| corner.get(&v).copied().unwrap_or(false);

    // --- Ketten bilden ---
    let mut chain_of = vec![NONE; nh as usize];
    let mut chains: Vec<Chain> = Vec::new();
    for h in 0..nh {
        if !is_bd(h) || chain_of[h as usize] != NONE {
            continue;
        }
        let tw = m.twin[h as usize];
        if tw != NONE && reg_he(h) > reg(tw / 3) {
            continue; // Gegenseite erzeugt die Kette
        }
        let s = nbr(h);
        // rückwärts zum Anfang
        let mut start = h;
        let mut closed = false;
        loop {
            if is_corner(m.he_from(start), &corner) {
                break;
            }
            let p = prev_bd(start);
            if nbr(p) != s {
                break;
            }
            if p == h {
                closed = true;
                break;
            }
            start = p;
        }
        let mut hes = vec![start];
        let mut cur = start;
        loop {
            if is_corner(m.he_to(cur), &corner) {
                break;
            }
            let n = next_bd(cur);
            if n == start || nbr(n) != s {
                break;
            }
            hes.push(n);
            cur = n;
            if hes.len() > nh as usize {
                break;
            }
        }
        if closed {
            // Start am kleinsten Vertex-Index (deterministisch für beide Seiten)
            let (mi, _) = hes.iter().enumerate().min_by_key(|(_, &x)| m.he_from(x)).unwrap();
            hes.rotate_left(mi);
            corner.insert(m.he_from(hes[0]), true);
        }
        let cid = chains.len() as u32;
        for &x in &hes {
            chain_of[x as usize] = cid;
            let t = m.twin[x as usize];
            if t != NONE {
                chain_of[t as usize] = cid;
            }
        }
        chains.push(Chain { hes, r: reg_he(start), s, closed });
    }

    // --- Ecken-Positionen lösen ---
    let surf_of = |r: u32| -> Option<Surf> {
        if r == NONE {
            None
        } else {
            Some(seg.regions[r as usize].surf)
        }
    };
    let mut bvert_of: HashMap<u32, u32> = HashMap::new();
    let mut verts: Vec<V3> = Vec::new();
    let mut get_vert = |v: u32, verts: &mut Vec<V3>| -> u32 {
        if let Some(&i) = bvert_of.get(&v) {
            return i;
        }
        let surfs: Vec<Surf> = vregs.get(&v).map(|rs| rs.iter().filter_map(|&r| surf_of(r)).collect()).unwrap_or_default();
        let p = solve_point(m.verts[v as usize], &surfs, tol);
        let i = verts.len() as u32;
        verts.push(p);
        bvert_of.insert(v, i);
        i
    };

    // --- Kurven anpassen, Ketten ggf. teilen ---
    // edge_of[h] = (Kante, vorwärts) für jede Rand-Halbkante
    let mut edge_of: Vec<(u32, bool)> = vec![(NONE, true); nh as usize];
    let mut edges: Vec<Edge> = Vec::new();
    for ch in &chains {
        let sa = surf_of(ch.r);
        let sb = surf_of(ch.s);
        let mut pts: Vec<V3> = ch.hes.iter().map(|&h| m.verts[m.he_from(h) as usize]).collect();
        pts.push(m.verts[m.he_to(*ch.hes.last().unwrap()) as usize]);
        // Kurven an Kettenpunkte immer mit der engen Toleranz anpassen (sonst ersetzt z. B. ein
        // Kreis eine unregelmäßige Schnittkurve); die Lage auf den Flächen prüft danach `tol`.
        let segs = fit_chain(&pts, sa.as_ref(), sb.as_ref(), fit_tol.min(tol), ch.closed, 0);
        for (i0, i1, curve) in segs {
            let mv0 = m.he_from(ch.hes[i0]);
            let mv1 = m.he_to(ch.hes[i1 - 1]);
            let v0 = get_vert(mv0, &mut verts);
            let v1 = get_vert(mv1, &mut verts);
            let ss: Vec<Surf> = [sa, sb].iter().flatten().copied().collect();
            let (p0, p1) = (verts[v0 as usize], verts[v1 as usize]);
            // An Freiformflächen müssen Kanten so eng auf der Fläche liegen, wie die Prüfung
            // (ff_edge_tol) verlangt – sonst wird die Fläche später facettiert
            let has_ff = ss.iter().any(|s| matches!(s, Surf::BSpline { .. }));
            let ff_lim = ff_edge_tol_of(fit_tol) * 0.5;
            let ctol = if has_ff { ff_lim.min(tol * 1.5) } else { tol * 1.5 };
            // Ecken müssen eng auf der Kurve liegen (sonst klafft im CAD-Programm an der Ecke ein
            // Spalt); bei Freiformteilen ist `tol` großzügig, daher hier die enge Toleranz
            let vtol = (fit_tol * 2.0).max(1e-3).min(ctol);
            // Kurve gegen Ecken und angrenzende Flächen prüfen; sonst Polygonzug
            let forced = ch.hes[i0..i1].iter().any(|h| force_poly.contains(h));
            let curve = match curve {
                Curve::Poly(_) => None,
                _ if forced => None,
                c => {
                    let dv = curve_dist(&c, p0).max(curve_dist(&c, p1));
                    let ds = sample_curve(&c, p0, p1, v0 == v1, 24)
                        .iter()
                        .map(|&q| ss.iter().map(|s| s.dist(q).abs()).fold(0.0, f64::max))
                        .fold(0.0, f64::max);
                    // Bögen: Kurve muss dem Kettenverlauf folgen (sonst z. B. Ellipsenbogen
                    // „falsch herum“ – liegt auf beiden Flächen, schneidet aber die Bohrung)
                    let follows = match c {
                        Curve::Circle { .. } | Curve::Ellipse { .. } => {
                            let chain = &pts[i0..=i1];
                            let cs = sample_curve(&c, p0, p1, v0 == v1, 48);
                            let (mut lo, mut hi) = (chain[0], chain[0]);
                            for q in chain {
                                lo = V3 { x: lo.x.min(q.x), y: lo.y.min(q.y), z: lo.z.min(q.z) };
                                hi = V3 { x: hi.x.max(q.x), y: hi.y.max(q.y), z: hi.z.max(q.z) };
                            }
                            let lim = (ctol * 4.0).max((hi - lo).norm() * 0.15);
                            let d1 = cs.iter().map(|&q| poly_dist(chain, q)).fold(0.0, f64::max);
                            let d2 = chain.iter().map(|&q| poly_dist(&cs, q)).fold(0.0, f64::max);
                            d1 <= lim && d2 <= lim
                        }
                        _ => true,
                    };
                    if dv <= vtol && ds <= ctol && follows {
                        Some(c)
                    } else {
                        None
                    }
                }
            };
            // Geraden immer exakt durch die Ecken legen (Richtung konsistent zur Kante)
            let curve = match curve {
                Some(Curve::Line { .. }) if (p1 - p0).norm() > 1e-9 => Some(Curve::Line { p: p0, d: (p1 - p0).normalized() }),
                Some(Curve::Line { .. }) => None,
                c => c,
            };
            let curve = match curve {
                Some(c) => c,
                None => {
                    // Innere Punkte auf beide Flächen projizieren, Enden = Ecken, bei Bedarf verdichten
                    let mut raw = Vec::with_capacity(i1 - i0 + 1);
                    raw.push(p0);
                    for k in i0 + 1..i1 {
                        raw.push(solve_limited(pts[k], &ss, tol, tol));
                    }
                    raw.push(p1);
                    let mut ps = vec![raw[0]];
                    let dtol = if has_ff { ff_lim.min(tol) } else { tol };
                    for w in raw.windows(2) {
                        densify(w[0], w[1], &ss, dtol, tol, 0, &mut ps);
                        ps.push(w[1]);
                    }
                    Curve::Poly(ps)
                }
            };
            let eid = edges.len() as u32;
            edges.push(Edge { v0, v1, curve, hes: ch.hes[i0..i1].to_vec() });
            for &h in &ch.hes[i0..i1] {
                edge_of[h as usize] = (eid, true);
                let t = m.twin[h as usize];
                if t != NONE {
                    edge_of[t as usize] = (eid, false);
                }
            }
        }
    }

    // --- Schleifen je Region ---
    let nreg = seg.regions.len();
    let mut bd_by_reg: Vec<Vec<u32>> = vec![Vec::new(); nreg];
    for h in 0..nh {
        if is_bd(h) {
            bd_by_reg[reg_he(h) as usize].push(h);
        }
    }
    let mut faces: Vec<Face> = Vec::with_capacity(nreg);
    let mut face_of_reg = vec![NONE; nreg];
    let mut used = vec![false; nh as usize];
    for (ri, hs) in bd_by_reg.iter().enumerate() {
        if hs.is_empty() {
            continue;
        }
        let mut loops = Vec::new();
        let mut loop_pts: Vec<Vec<V3>> = Vec::new();
        for &h0 in hs {
            if used[h0 as usize] {
                continue;
            }
            let mut seq = Vec::new();
            let mut h = h0;
            loop {
                if used[h as usize] {
                    break;
                }
                used[h as usize] = true;
                seq.push(h);
                h = next_bd(h);
                if seq.len() > nh as usize {
                    break;
                }
            }
            // An Kantengrenze ausrichten
            let k = seq.len();
            let start = (0..k).find(|&i| {
                let prev = seq[(i + k - 1) % k];
                edge_of[prev as usize].0 != edge_of[seq[i] as usize].0
            });
            let mut uses: Vec<(u32, bool)> = Vec::new();
            match start {
                None => {
                    uses.push(edge_of[seq[0] as usize]);
                }
                Some(s0) => {
                    for j in 0..k {
                        let e = edge_of[seq[(s0 + j) % k] as usize];
                        if uses.last().map_or(true, |l: &(u32, bool)| l.0 != e.0) {
                            uses.push(e);
                        }
                    }
                }
            }
            loop_pts.push(seq.iter().map(|&x| m.verts[m.he_from(x) as usize]).collect());
            loops.push(uses);
        }
        let region = &seg.regions[ri];
        // Orientierung: mittlere Netznormale vs. natürliche Flächennormale
        let mut acc = 0.0;
        for &t in &region.tris {
            let t = t as usize;
            acc += m.tri_n[t].dot(region.surf.normal(m.tri_centroid(t))) * m.tri_a[t];
        }
        let same_sense = acc >= 0.0;
        let outer = if let Surf::Plane { n, .. } = region.surf {
            // größte (positive) Fläche bzgl. der Normalen
            let nn = if same_sense { n } else { -n };
            let mut best = None;
            let mut best_a = f64::MIN;
            for (i, lp) in loop_pts.iter().enumerate() {
                let mut a = V3::ZERO;
                for j in 0..lp.len() {
                    a += lp[j].cross(lp[(j + 1) % lp.len()]);
                }
                let ar = a.dot(nn);
                if ar > best_a {
                    best_a = ar;
                    best = Some(i);
                }
            }
            best
        } else {
            None
        };
        face_of_reg[ri] = faces.len() as u32;
        // Ebenen immer mit Außennormale schreiben (gleiche Orientierung wie die Fläche)
        let (surf, same_sense) = match region.surf {
            Surf::Plane { o, n } if !same_sense => (Surf::Plane { o, n: -n }, true),
            s => (s, same_sense),
        };
        faces.push(Face { surf, same_sense, loops, outer, region: ri as u32 });
    }

    // --- Schalen und Körper ---
    let mut shells = Vec::new();
    let mut shell_of_comp = vec![NONE; m.comps.len()];
    for (ci, comp) in m.comps.iter().enumerate() {
        let mut fs: Vec<u32> = comp
            .tris
            .iter()
            .map(|&t| face_of_reg[reg(t) as usize])
            .filter(|&f| f != NONE)
            .collect();
        fs.sort_unstable();
        fs.dedup();
        if fs.is_empty() {
            continue;
        }
        shell_of_comp[ci] = shells.len() as u32;
        shells.push(Shell { faces: fs, closed: comp.closed });
    }
    let mut solids = Vec::new();
    let mut k = 0;
    for (ci, comp) in m.comps.iter().enumerate() {
        if comp.void_of.is_some() || shell_of_comp[ci] == NONE {
            continue;
        }
        k += 1;
        let voids = m
            .comps
            .iter()
            .enumerate()
            .filter(|(_, c)| c.void_of == Some(ci as u32))
            .map(|(j, _)| shell_of_comp[j])
            .filter(|&s| s != NONE)
            .collect();
        let nm = if m.comps.len() > 1 { format!("{name}_{k}") } else { name.to_string() };
        solids.push(Solid { name: nm, outer: shell_of_comp[ci], voids });
    }
    Brep { verts, edges, faces, shells, solids, tol, fit_tol }
}

impl Brep {
    /// Zulässiger Abstand einer Kante von einer angrenzenden Freiformfläche. Muss eng sein:
    /// CAD-Kernel trimmen die Fläche an der Projektion der Kante – liegt die Kante daneben,
    /// entsteht ein Spalt zur Nachbarfläche.
    pub fn ff_edge_tol(&self) -> f64 {
        ff_edge_tol_of(self.fit_tol)
    }
}

/// Abstand eines Punktes zu einem Polygonzug.
fn poly_dist(ps: &[V3], q: V3) -> f64 {
    if ps.len() == 1 {
        return ps[0].dist(q);
    }
    let mut best = f64::MAX;
    for w in ps.windows(2) {
        let d = w[1] - w[0];
        let l2 = d.dot(d);
        let t = if l2 > 0.0 { ((q - w[0]).dot(d) / l2).clamp(0.0, 1.0) } else { 0.0 };
        best = best.min((w[0] + d * t).dist(q));
    }
    best
}

/// Erlaubter Abstand der Kanten von Freiformflächen (sonst Spalte nach dem Trimmen im CAD-Programm).
pub fn ff_edge_tol_of(fit_tol: f64) -> f64 {
    let floor = std::env::var("M2S_FFEDGE").ok().and_then(|s| s.parse::<f64>().ok()).unwrap_or(1e-3);
    (fit_tol * 2.0).max(floor)
}

/// Dichte Abtastpunkte einer Kante (Polygonzüge auch zwischen den Stützpunkten).
fn edge_samples(b: &Brep, e: usize) -> Vec<V3> {
    let ed = &b.edges[e];
    match &ed.curve {
        Curve::Poly(ps) => {
            let mut out = Vec::with_capacity(ps.len() * 4);
            for w in ps.windows(2) {
                for k in 0..4 {
                    out.push(w[0] + (w[1] - w[0]) * (k as f64 / 4.0));
                }
            }
            out.extend(ps.last().copied());
            out
        }
        c => sample_curve(c, b.verts[ed.v0 as usize], b.verts[ed.v1 as usize], ed.v0 == ed.v1, 48),
    }
}

/// Zieht jede Freiformfläche an ihre Randkanten heran, damit Kanten exakt auf der Fläche
/// liegen (sonst trimmt das CAD-Programm die Fläche neben der Kante und es entstehen Löcher).
/// Gelingt das nicht, bleibt die Fläche unverändert; die Prüfung stuft sie dann zurück.
pub fn snap_freeform(b: &mut Brep, m: &Mesh, seg: &mut Segmentation, soft_tol: f64) -> usize {
    let hard_tol = b.ff_edge_tol();
    let mut snapped = 0;
    for fi in 0..b.faces.len() {
        let id = match b.faces[fi].surf {
            Surf::BSpline { id } => id,
            _ => continue,
        };
        let s = crate::bspline::get(id);
        let mut hard = Vec::new();
        let mut seen = std::collections::HashSet::new();
        for &(e, _) in b.faces[fi].loops.iter().flatten() {
            if seen.insert(e) {
                hard.extend(edge_samples(b, e as usize));
            }
        }
        let worst = hard.iter().map(|&p| s.dist(p).abs()).fold(0.0, f64::max);
        if worst <= hard_tol * 0.5 {
            continue;
        }
        let r = b.faces[fi].region as usize;
        let mut vs: Vec<u32> = seg.regions[r].tris.iter().flat_map(|&t| m.tris[t as usize]).collect();
        vs.sort_unstable();
        vs.dedup();
        let soft: Vec<V3> = vs.iter().map(|&v| m.verts[v as usize]).collect();
        if std::env::var("M2S_DEBUG").is_ok() {
            eprintln!("Freiformfläche {fi}: Rand weicht {worst:.4} ab, {} Randpunkte", hard.len());
        }
        // Auch eine nur bessere Näherung übernehmen: beim nächsten Aufbau werden die Kanten
        // auf die angezogene Fläche projiziert, danach passt der Rand meist exakt.
        if let Some((t, dev)) = crate::bspline::snap_to_boundary(&s, &soft, &hard, soft_tol, hard_tol) {
            if dev < worst * 0.5 {
                let surf = Surf::BSpline { id: crate::bspline::register(t) };
                b.faces[fi].surf = surf;
                seg.regions[r].surf = surf;
                snapped += 1;
            }
        }
    }
    snapped
}

/// Schiebt die inneren Punkte von Polygonkanten an Freiformflächen exakt auf den Schnitt
/// der beiden angrenzenden Flächen (nur kleine Korrekturen). Liefert true, wenn sich etwas bewegt hat.
pub fn project_ff_edges(b: &mut Brep) -> bool {
    let lim = b.ff_edge_tol();
    let mut faces_of: Vec<Vec<u32>> = vec![Vec::new(); b.edges.len()];
    for (fi, f) in b.faces.iter().enumerate() {
        for &(e, _) in f.loops.iter().flatten() {
            if !faces_of[e as usize].contains(&(fi as u32)) {
                faces_of[e as usize].push(fi as u32);
            }
        }
    }
    let mut moved = false;
    for ei in 0..b.edges.len() {
        let fs = &faces_of[ei];
        if !fs.iter().any(|&f| matches!(b.faces[f as usize].surf, Surf::BSpline { .. })) {
            continue;
        }
        let ss: Vec<Surf> = fs.iter().map(|&f| b.faces[f as usize].surf).collect();
        let Curve::Poly(ps) = &mut b.edges[ei].curve else { continue };
        let n = ps.len();
        for k in 1..n.saturating_sub(1) {
            let p0 = ps[k];
            let d0 = ss.iter().map(|s| s.dist(p0).abs()).fold(0.0, f64::max);
            if d0 <= lim * 0.25 {
                continue;
            }
            // Bewegung begrenzen: nur Feinkorrektur, keine Sprünge auf einen anderen Schnittast
            let q = solve_point(p0, &ss, f64::MAX);
            let d1 = ss.iter().map(|s| s.dist(q).abs()).fold(0.0, f64::max);
            if q.is_finite() && d1 < d0 && q.dist(p0) <= (d0 * 20.0).max(lim * 4.0) {
                ps[k] = q;
                moved = true;
            }
        }
    }
    moved
}

/// Punkt nahe p0 finden, der auf allen Flächen liegt (regularisiertes Gauß-Newton).
pub fn solve_point(p0: V3, surfs: &[Surf], tol: f64) -> V3 {
    if surfs.is_empty() {
        return p0;
    }
    let mut p = p0;
    let mu = 1e-6;
    for _ in 0..30 {
        let mut a = vec![vec![0.0; 3]; 3];
        let mut b = vec![0.0; 3];
        let mut maxd: f64 = 0.0;
        for s in surfs {
            let d = s.dist(p);
            maxd = maxd.max(d.abs());
            let g = s.normal(p);
            for i in 0..3 {
                for j in 0..3 {
                    a[i][j] += g.get(i) * g.get(j);
                }
                b[i] -= g.get(i) * d;
            }
        }
        let off = p - p0;
        for i in 0..3 {
            a[i][i] += mu;
            b[i] -= mu * off.get(i);
        }
        if maxd < 1e-11 {
            break;
        }
        let x = match solve(a, b) {
            Some(x) => x,
            None => break,
        };
        p = p + V3 { x: x[0], y: x[1], z: x[2] };
    }
    let ok = surfs.iter().all(|s| s.dist(p).abs() <= tol * 0.5) && p.dist(p0) <= tol * 2.0 && p.is_finite();
    if ok {
        p
    } else {
        p0
    }
}

// ---------------------------------------------------------------------------
// Kurven für Ketten

fn max_dev(c: &Curve, pts: &[V3]) -> f64 {
    pts.iter().map(|&p| curve_dist(c, p)).fold(0.0, f64::max)
}

pub fn curve_dist(c: &Curve, p: V3) -> f64 {
    match c {
        Curve::Line { p: o, d } => {
            let v = p - *o;
            (v - *d * v.dot(*d)).norm()
        }
        Curve::Circle { c, n, r, .. } => {
            let v = p - *c;
            let h = v.dot(*n);
            let rho = (v - *n * h).norm();
            ((rho - r).powi(2) + h * h).sqrt()
        }
        Curve::Ellipse { c, n, x, a, b } => {
            let v = p - *c;
            let h = v.dot(*n);
            let y = n.cross(*x);
            let (px, py) = (v.dot(*x), v.dot(y));
            // Näherung: Abstand über Parameter-Suche
            let mut best = f64::MAX;
            let mut t0 = py.atan2(px * b / a);
            for _ in 0..3 {
                let mut tb = t0;
                for i in -20..=20 {
                    let t = t0 + i as f64 * 0.01;
                    let d = (px - a * t.cos()).powi(2) + (py - b * t.sin()).powi(2);
                    if d < best {
                        best = d;
                        tb = t;
                    }
                }
                t0 = tb;
            }
            (best + h * h).sqrt()
        }
        Curve::Poly(ps) => {
            let mut best = f64::MAX;
            for w in ps.windows(2) {
                let d = w[1] - w[0];
                let l2 = d.norm2();
                let t = if l2 > 0.0 { ((p - w[0]).dot(d) / l2).clamp(0.0, 1.0) } else { 0.0 };
                best = best.min(p.dist(w[0] + d * t));
            }
            best
        }
    }
}

/// Orientiert Kreis/Ellipse so, dass die Kette gegen den Uhrzeigersinn (bzgl. n) läuft.
fn orient_conic(c: V3, n: V3, pts: &[V3]) -> V3 {
    let mut s = 0.0;
    for w in pts.windows(2) {
        s += (w[0] - c).cross(w[1] - c).dot(n);
    }
    if s < 0.0 {
        -n
    } else {
        n
    }
}

fn circle_from(c: V3, n: V3, r: f64, pts: &[V3]) -> Curve {
    let n = orient_conic(c, n, pts);
    let x = {
        let v = pts[0] - c;
        let v = v - n * v.dot(n);
        if v.norm() > 1e-12 {
            v.normalized()
        } else {
            n.any_perp()
        }
    };
    Curve::Circle { c, n, x, r }
}

/// Analytische Schnittkurve zweier Flächen (wo einfach möglich).
fn analytic(sa: &Surf, sb: &Surf, pts: &[V3], closed: bool) -> Option<Curve> {
    let par = |a: V3, b: V3| angle(a, b).min(angle(a, -b)) < 1e-6;
    let mid = pts[pts.len() / 2];
    match (*sa, *sb) {
        (Surf::Plane { o: o1, n: n1 }, Surf::Plane { o: o2, n: n2 }) => {
            if closed {
                return None;
            }
            let d = n1.cross(n2);
            if d.norm() < 1e-9 {
                return None;
            }
            let d = d.normalized();
            // Punkt auf beiden Ebenen nahe mid
            let p = solve_point(mid, &[Surf::Plane { o: o1, n: n1 }, Surf::Plane { o: o2, n: n2 }], f64::MAX);
            let d = if (pts[pts.len() - 1] - pts[0]).dot(d) < 0.0 { -d } else { d };
            Some(Curve::Line { p, d })
        }
        (Surf::Plane { o, n }, other) | (other, Surf::Plane { o, n }) => match other {
            Surf::Sphere { c, r } => {
                let d = (c - o).dot(n);
                if d.abs() >= r {
                    return None;
                }
                Some(circle_from(c - n * d, n, (r * r - d * d).sqrt(), pts))
            }
            Surf::Cyl { p, a, r } => {
                let an = a.dot(n);
                // nahezu senkrecht/parallel (Rauschen) wie exakt behandeln; die Abweichung wird danach geprüft
                if par(a, n) || angle(a, n).min(angle(a, -n)) < 0.02 {
                    let t = (o - p).dot(n) / an;
                    Some(circle_from(p + a * t, n, r, pts))
                } else if an.abs() < 0.02 {
                    let a = (a - n * an).normalized();
                    if closed {
                        return None;
                    }
                    let dd = (p - o).dot(n);
                    let e = a.cross(n).normalized();
                    let foot = p - n * dd;
                    let h2 = r * r - dd * dd;
                    let h = if h2 > 0.0 { h2.sqrt() } else { 0.0 };
                    let l1 = foot + e * h;
                    let l2 = foot - e * h;
                    let dist = |q: V3| {
                        let v = mid - q;
                        (v - a * v.dot(a)).norm()
                    };
                    let q = if dist(l1) <= dist(l2) { l1 } else { l2 };
                    let d = if (pts[pts.len() - 1] - pts[0]).dot(a) < 0.0 { -a } else { a };
                    Some(Curve::Line { p: q, d })
                } else {
                    let t = (o - p).dot(n) / an;
                    let c = p + a * t;
                    let mj = (a - n * an).normalized();
                    let nn = orient_conic(c, n, pts);
                    Some(Curve::Ellipse { c, n: nn, x: mj, a: r / an.abs(), b: r })
                }
            }
            Surf::Cone { p, a, r, ang } => {
                if !par(a, n) {
                    return None;
                }
                let t = (o - p).dot(n) / a.dot(n);
                let rr = r + t * ang.tan();
                if rr <= 0.0 {
                    return None;
                }
                Some(circle_from(p + a * t, n, rr, pts))
            }
            Surf::Torus { c, a, rmaj, rmin } => {
                if !par(a, n) {
                    return None;
                }
                let t = (o - c).dot(n) / a.dot(n);
                let center = c + a * t;
                let h2 = rmin * rmin - t * t;
                let h = if h2 > 0.0 { h2.sqrt() } else { 0.0 };
                let v = mid - center;
                let rho = (v - n * v.dot(n)).norm();
                let rr = if (rho - (rmaj + h)).abs() < (rho - (rmaj - h)).abs() { rmaj + h } else { rmaj - h };
                Some(circle_from(center, n, rr, pts))
            }
            Surf::Plane { .. } | Surf::BSpline { .. } => None,
        },
        _ => {
            // Koaxiale Rotationsflächen schneiden sich in Kreisen senkrecht zur Achse
            let ax = |s: &Surf| -> Option<(V3, V3)> {
                match *s {
                    Surf::Sphere { c, .. } => Some((c, V3::ZERO)),
                    _ => s.axis(),
                }
            };
            let (p1, a1) = ax(sa)?;
            let (p2, a2) = ax(sb)?;
            let (p, a) = if a1 == V3::ZERO && a2 == V3::ZERO {
                return None;
            } else if a1 == V3::ZERO {
                (p2, a2)
            } else {
                (p1, a1)
            };
            let on_axis = |q: V3| {
                let v = q - p;
                (v - a * v.dot(a)).norm() < 1e-6 * (1.0 + v.norm())
            };
            if (a1 != V3::ZERO && !par(a1, a)) || (a2 != V3::ZERO && !par(a2, a)) || !on_axis(p1) || !on_axis(p2) {
                return None;
            }
            let mut h = 0.0;
            let mut rr = 0.0;
            for q in pts {
                let v = *q - p;
                let hh = v.dot(a);
                h += hh;
                rr += (v - a * hh).norm();
            }
            h /= pts.len() as f64;
            rr /= pts.len() as f64;
            Some(circle_from(p + a * h, a, rr, pts))
        }
    }
}

/// Liefert Teilstücke (Index-Bereiche der Kettensegmente) mit Kurven.
fn fit_chain(pts: &[V3], sa: Option<&Surf>, sb: Option<&Surf>, tol: f64, closed: bool, depth: usize) -> Vec<(usize, usize, Curve)> {
    let nseg = pts.len() - 1;
    let ctol = tol * 1.5;
    let ends_ok = |c: &Curve| curve_dist(c, pts[0]) <= ctol && curve_dist(c, pts[nseg]) <= ctol;
    // 1) analytisch
    if let (Some(a), Some(b)) = (sa, sb) {
        if let Some(c) = analytic(a, b, pts, closed) {
            if max_dev(&c, pts) <= ctol && ends_ok(&c) && arc_ok(&c, pts, closed) {
                return vec![(0, nseg, c)];
            }
        }
    }
    // 2) Gerade
    if !closed {
        let d = pts[nseg] - pts[0];
        if d.norm() > tol {
            let c = Curve::Line { p: pts[0], d: d.normalized() };
            if max_dev(&c, pts) <= ctol {
                return vec![(0, nseg, c)];
            }
        }
    }
    // 3) Kreis (mind. 4 Punkte)
    if pts.len() >= 5 || (closed && pts.len() >= 4) {
        if let Some((c, n, r, _)) = surf::fit_circle_3d(&pts[..if closed { nseg } else { pts.len() }]) {
            let cv = circle_from(c, n, r, pts);
            if max_dev(&cv, pts) <= ctol && arc_ok(&cv, pts, closed) {
                return vec![(0, nseg, cv)];
            }
        }
    }
    // 4) an scharfem Knick teilen
    if depth < 8 && nseg >= 2 {
        let mut best = (0usize, 0.0f64);
        for i in 1..nseg {
            let a = (pts[i] - pts[i - 1]).normalized();
            let b = (pts[i + 1] - pts[i]).normalized();
            let t = angle(a, b);
            if t > best.1 {
                best = (i, t);
            }
        }
        if best.1 > 50f64.to_radians() && !closed {
            let i = best.0;
            let mut left = fit_chain(&pts[..=i], sa, sb, tol, false, depth + 1);
            let right = fit_chain(&pts[i..], sa, sb, tol, false, depth + 1);
            for (a, b, c) in right {
                left.push((a + i, b + i, c));
            }
            return left;
        }
    }
    if closed && nseg < 3 {
        // Zu kurze geschlossene Kette: aufteilen, damit kein entartetes Polygon entsteht
        if nseg == 2 {
            return vec![
                (0, 1, Curve::Line { p: pts[0], d: (pts[1] - pts[0]).normalized() }),
                (1, 2, Curve::Line { p: pts[1], d: (pts[2] - pts[1]).normalized() }),
            ];
        }
    }
    // Polygonzug: an Knicken > 15° teilen, damit CAD-Kernel glatte Parameterkurven bilden können
    let mut out = Vec::new();
    let mut start = 0;
    for i in 1..nseg {
        let a = (pts[i] - pts[i - 1]).normalized();
        let b = (pts[i + 1] - pts[i]).normalized();
        if angle(a, b) > 15f64.to_radians() && i - start >= 1 && !(closed && start == 0 && i == nseg) {
            out.push((start, i, Curve::Poly(Vec::new())));
            start = i;
        }
    }
    out.push((start, nseg, Curve::Poly(Vec::new())));
    out
}

/// Kreisbögen: Die Kette muss monoton im Umlaufsinn laufen (keine Rückläufe).
fn arc_ok(c: &Curve, pts: &[V3], closed: bool) -> bool {
    let (cc, n) = match c {
        Curve::Circle { c, n, .. } | Curve::Ellipse { c, n, .. } => (*c, *n),
        _ => return true,
    };
    let mut total = 0.0;
    for w in pts.windows(2) {
        let a = w[0] - cc;
        let b = w[1] - cc;
        let s = a.cross(b).dot(n);
        let ang = s.atan2(a.dot(b));
        if ang < -1e-9 {
            return false;
        }
        total += ang;
    }
    if closed {
        (total - std::f64::consts::TAU).abs() < 0.05
    } else {
        total < std::f64::consts::TAU - 1e-6
    }
}

/// Fügt zwischen a und b projizierte Zwischenpunkte ein, solange die Sehne zu weit von den Flächen abweicht.
fn densify(a: V3, b: V3, ss: &[Surf], tol: f64, lim: f64, depth: usize, out: &mut Vec<V3>) {
    if depth >= 6 || ss.is_empty() {
        return;
    }
    let m = (a + b) * 0.5;
    let dev = ss.iter().map(|s| s.dist(m).abs()).fold(0.0, f64::max);
    if dev <= tol * 0.5 {
        return;
    }
    let q = solve_limited(m, ss, lim, lim);
    if q == m {
        return;
    }
    densify(a, q, ss, tol, lim, depth + 1, out);
    out.push(q);
    densify(q, b, ss, tol, lim, depth + 1, out);
}

/// Abtastpunkte einer Kurve zwischen zwei Ecken (Kreise/Ellipsen gegen den Uhrzeigersinn).
pub fn sample_curve(c: &Curve, p0: V3, p1: V3, closed: bool, n: usize) -> Vec<V3> {
    match c {
        Curve::Line { .. } => (0..=n).map(|i| p0 + (p1 - p0) * (i as f64 / n as f64)).collect(),
        Curve::Circle { c, n: nn, x, r } => {
            let y = nn.cross(*x);
            let th = |p: V3| ((p - *c).dot(y)).atan2((p - *c).dot(*x));
            let t0 = th(p0);
            let mut t1 = th(p1);
            if closed {
                t1 = t0 + std::f64::consts::TAU;
            } else if t1 <= t0 {
                t1 += std::f64::consts::TAU;
            }
            (0..=n)
                .map(|i| {
                    let t = t0 + (t1 - t0) * i as f64 / n as f64;
                    *c + (*x * t.cos() + y * t.sin()) * *r
                })
                .collect()
        }
        Curve::Ellipse { c, n: nn, x, a, b } => {
            let y = nn.cross(*x);
            let th = |p: V3| ((p - *c).dot(y) / b).atan2((p - *c).dot(*x) / a);
            let t0 = th(p0);
            let mut t1 = th(p1);
            if closed {
                t1 = t0 + std::f64::consts::TAU;
            } else if t1 <= t0 {
                t1 += std::f64::consts::TAU;
            }
            (0..=n)
                .map(|i| {
                    let t = t0 + (t1 - t0) * i as f64 / n as f64;
                    *c + *x * (a * t.cos()) + y * (b * t.sin())
                })
                .collect()
        }
        Curve::Poly(ps) => ps.clone(),
    }
}

/// Wie solve_point, verschiebt den Punkt aber höchstens um `maxmove` (verhindert Zickzack in Polygonzügen).
fn solve_limited(p0: V3, surfs: &[Surf], tol: f64, maxmove: f64) -> V3 {
    let p = solve_point(p0, surfs, tol);
    if p.dist(p0) <= maxmove {
        p
    } else {
        p0
    }
}
