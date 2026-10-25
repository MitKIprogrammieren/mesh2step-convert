//! Interne Prüfung der erzeugten B-Rep (Topologie und Geometrie).

use crate::brep::{curve_dist, Brep, Curve};
use crate::math::V3;
use std::collections::HashMap;

#[derive(Debug, Default, Clone)]
pub struct CheckResult {
    pub errors: Vec<String>,
    /// größte Abweichung einer Kante von ihren Flächen (mm)
    pub max_edge_dev: f64,
    /// größte Abweichung einer Ecke von ihren Kanten (mm)
    pub max_vertex_dev: f64,
    /// Flächen, deren Kanten zu weit von der Fläche abweichen
    pub bad_faces: Vec<u32>,
    /// Problemkanten je Fläche (Fläche, Kante) – erlaubt gezieltes Nachbessern großer Flächen
    pub bad_edges: Vec<(u32, u32)>,
    /// Achterschleifen: (Fläche, Lage der doppelt besuchten Ecke)
    pub pinches: Vec<(u32, V3)>,
}

pub fn sample_edge(b: &Brep, ei: usize, n: usize) -> Vec<V3> {
    let e = &b.edges[ei];
    match &e.curve {
        Curve::Poly(ps) => {
            let mut out = Vec::new();
            for w in ps.windows(2) {
                out.push(w[0]);
                out.push((w[0] + w[1]) * 0.5);
            }
            out.push(*ps.last().unwrap());
            out
        }
        c => crate::brep::sample_curve(c, b.verts[e.v0 as usize], b.verts[e.v1 as usize], e.v0 == e.v1, n),
    }
}

pub fn check(b: &Brep) -> CheckResult {
    let mut res = CheckResult::default();
    let mut uses: HashMap<u32, Vec<(u32, bool)>> = HashMap::new();
    for (fi, f) in b.faces.iter().enumerate() {
        if f.loops.is_empty() {
            res.errors.push(format!("Fläche {fi} ohne Rand"));
        }
        // Eine Schleife darf jede Ecke nur einmal besuchen (sonst „Achterschleife“)
        let mut seen: Vec<V3> = Vec::new();
        let mut pinch = false;
        for lp in &f.loops {
            for &(e, fwd) in lp {
                let ed = &b.edges[e as usize];
                let st = b.verts[(if fwd { ed.v0 } else { ed.v1 }) as usize];
                if seen.iter().any(|q| q.dist(st) < 1e-4) {
                    pinch = true;
                    res.pinches.push((fi as u32, st));
                }
                seen.push(st);
            }
            if seen.len() > 400 {
                break;
            }
        }
        if pinch {
            if std::env::var("M2S_DEBUG").is_ok() {
                eprintln!("Fläche {fi}: Achterschleife {:?}", res.pinches.iter().filter(|p| p.0 == fi as u32).map(|p| p.1).collect::<Vec<_>>());
            }
            res.bad_faces.push(fi as u32);
        }
        for lp in &f.loops {
            if lp.is_empty() {
                res.errors.push(format!("Fläche {fi}: leere Schleife"));
                continue;
            }
            for (i, &(e, fwd)) in lp.iter().enumerate() {
                uses.entry(e).or_default().push((fi as u32, fwd));
                let ed = &b.edges[e as usize];
                let end = if fwd { ed.v1 } else { ed.v0 };
                let (ne, nf) = lp[(i + 1) % lp.len()];
                let nd = &b.edges[ne as usize];
                let st = if nf { nd.v0 } else { nd.v1 };
                if end != st {
                    res.errors.push(format!("Fläche {fi}: Schleife nicht geschlossen"));
                }
            }
        }
    }
    let closed_faces: std::collections::HashSet<u32> =
        b.shells.iter().filter(|s| s.closed).flat_map(|s| s.faces.iter().copied()).collect();
    for (ei, e) in b.edges.iter().enumerate() {
        let u = uses.get(&(ei as u32)).cloned().unwrap_or_default();
        let in_closed = u.iter().any(|(f, _)| closed_faces.contains(f));
        if in_closed {
            let nf = u.iter().filter(|x| x.1).count();
            let nr = u.iter().filter(|x| !x.1).count();
            if !(nf == 1 && nr == 1) {
                res.errors.push(format!("Kante {ei}: {nf}x vorwärts / {nr}x rückwärts verwendet"));
            }
        }
        let lim = b.tol * 2.0;
        for v in [e.v0, e.v1] {
            let d = curve_dist(&e.curve, b.verts[v as usize]);
            res.max_vertex_dev = res.max_vertex_dev.max(d);
            if d > lim {
                if std::env::var("M2S_DEBUG").is_ok() { eprintln!("Ecke weicht {d:.4} von Kante {ei} ab"); }
                res.bad_faces.extend(u.iter().map(|x| x.0));
            }
        }
        let samples = sample_edge(b, ei, 16);
        for (f, _) in &u {
            let s = b.faces[*f as usize].surf;
            // Freiformflächen eng prüfen (sonst Spalte nach dem Trimmen im CAD-Programm)
            let lim = if matches!(s, crate::surf::Surf::BSpline { .. }) { b.ff_edge_tol() } else { lim };
            for p in &samples {
                let d = s.dist(*p).abs();
                if d > lim {
                    if std::env::var("M2S_DEBUG").is_ok() { let ks: Vec<String> = u.iter().map(|x| format!("{:?}", b.faces[x.0 as usize].surf.kind())).collect(); let k = match &e.curve { Curve::Line{..}=>"L", Curve::Circle{..}=>"C", Curve::Ellipse{..}=>"E", Curve::Poly(_)=>"P" }; eprintln!("Kante {ei} weicht {d:.4} von Fläche {f} ({:?}) ab [{}] {k}", s.kind(), ks.join("/")); }
                    res.bad_faces.push(*f);
                    res.bad_edges.push((*f, ei as u32));
                }
                if d > res.max_edge_dev {
                    res.max_edge_dev = d;
                    if std::env::var("M2S_DEBUG").is_ok() {
                        let k = match &e.curve { Curve::Line{..}=>"Linie", Curve::Circle{..}=>"Kreis", Curve::Ellipse{..}=>"Ellipse", Curve::Poly(_)=>"Polygon" };
                        eprintln!("max dev {d:.4} edge {ei} {k} surf {:?}", s.kind());
                    }
                }
            }
        }
        if let Curve::Poly(ps) = &e.curve {
            if ps.len() < 2 {
                res.errors.push(format!("Kante {ei}: Polygon mit < 2 Punkten"));
            }
        }
    }
    // Ebene Flächen: Randschleifen dürfen sich nicht selbst schneiden
    // (für Freiformflächen im Parameterbereich = Projektion auf die Bezugsebene)
    for (fi, f) in b.faces.iter().enumerate() {
        let frame = match f.surf {
            crate::surf::Surf::Plane { o, n } => Some((o, n)),
            crate::surf::Surf::BSpline { id } => {
                let bs = crate::bspline::get(id);
                Some((bs.o, bs.n))
            }
            _ => None,
        };
        if let Some((o, n)) = frame {
            let nn = if f.same_sense { n } else { -n };
            let mut hits = Vec::new();
            // Freiformflächen: Mindestabstand nicht benachbarter Kanten = 2x STEP-Unschärfe
            let near = if matches!(f.surf, crate::surf::Surf::BSpline { .. }) { 2.0 * crate::step::uncertainty(b) } else { 0.0 };
            let si = planar_loops_intersect(b, f, o, n, near, &mut hits);
            res.bad_edges.extend(hits.iter().map(|&e| (fi as u32, e)));
            let lo = loop_orientation_ok(b, f, o, nn);
            if si || !lo {
                if std::env::var("M2S_DEBUG").is_ok() {
                    eprintln!("Fläche {fi}: Selbstschnitt {si}, Orientierung ok {lo}");
                }
                res.bad_faces.push(fi as u32);
            }
        }
    }
    if std::env::var("M2S_STATS").is_ok() {
        print_stats(b);
    }
    res.errors.dedup();
    res.bad_faces.sort_unstable();
    res.bad_faces.dedup();
    res
}

fn planar_loops_intersect(b: &Brep, f: &crate::brep::Face, o: V3, n: V3, near: f64, hits: &mut Vec<u32>) -> bool {
    let (e1, e2) = n.basis();
    let mut segs: Vec<((f64, f64), (f64, f64))> = Vec::new();
    let mut seg_edge: Vec<u32> = Vec::new();
    let mut seg_verts: Vec<(u32, u32)> = Vec::new();
    let mut corners: Vec<(f64, f64)> = Vec::new();
    for lp in &f.loops {
        for &(e, fwd) in lp {
            let q = b.verts[b.edges[e as usize].v0 as usize] - o;
            corners.push((q.dot(e1), q.dot(e2)));
            let ns = match b.edges[e as usize].curve {
                Curve::Line { .. } => 1,
                Curve::Poly(_) => 1,
                _ => 96,
            };
            let mut pts = sample_edge(b, e as usize, ns);
            if !fwd {
                pts.reverse();
            }
            for w in pts.windows(2) {
                let a = w[0] - o;
                let c = w[1] - o;
                segs.push(((a.dot(e1), a.dot(e2)), (c.dot(e1), c.dot(e2))));
                seg_edge.push(e);
                let ed = &b.edges[e as usize];
                seg_verts.push((ed.v0, ed.v1));
            }
        }
    }
    let n_s = segs.len();
    if n_s > 4000 {
        return false;
    }
    let cross = |a: (f64, f64), b: (f64, f64), c: (f64, f64)| (b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0);
    // nach x sortieren für einfachen Sweep
    let mut idx: Vec<usize> = (0..n_s).collect();
    let minx = |s: &((f64, f64), (f64, f64))| s.0 .0.min(s.1 .0);
    let maxx = |s: &((f64, f64), (f64, f64))| s.0 .0.max(s.1 .0);
    idx.sort_by(|&i, &j| minx(&segs[i]).partial_cmp(&minx(&segs[j])).unwrap());
    let eps = 1e-12;
    let pseg = |p: (f64, f64), a: (f64, f64), c: (f64, f64)| {
        let (dx, dy) = (c.0 - a.0, c.1 - a.1);
        let l2 = dx * dx + dy * dy;
        let t = if l2 > 0.0 { (((p.0 - a.0) * dx + (p.1 - a.1) * dy) / l2).clamp(0.0, 1.0) } else { 0.0 };
        ((p.0 - a.0 - dx * t).powi(2) + (p.1 - a.1 - dy * t).powi(2)).sqrt()
    };
    for ii in 0..n_s {
        let i = idx[ii];
        let si = segs[i];
        let mxi = maxx(&si) + near;
        for &j in &idx[ii + 1..] {
            let sj = segs[j];
            if minx(&sj) > mxi {
                break;
            }
            // Fast-Berührung zweier Kanten ohne gemeinsame Ecke: CAD-Kernel rechnen mit
            // Toleranzen und werten das als Selbstschnitt (z. B. Spitzen schmaler Splitterflächen)
            if near > 0.0 {
                let (a, c) = (seg_verts[i], seg_verts[j]);
                let shares = a.0 == c.0 || a.0 == c.1 || a.1 == c.0 || a.1 == c.1;
                if !shares {
                    let d = pseg(si.0, sj.0, sj.1).min(pseg(si.1, sj.0, sj.1)).min(pseg(sj.0, si.0, si.1)).min(pseg(sj.1, si.0, si.1));
                    if d < near {
                        if std::env::var("M2S_DEBUG").is_ok() {
                            eprintln!("Fast-Berührung: Kante {} x Kante {} Abstand {d:.4}", seg_edge[i], seg_edge[j]);
                        }
                        hits.push(seg_edge[i]);
                        hits.push(seg_edge[j]);
                        continue;
                    }
                }
            }
            // benachbarte Segmente (gemeinsamer Endpunkt) überspringen
            let share = |p: (f64, f64), q: (f64, f64)| (p.0 - q.0).abs() < 1e-9 && (p.1 - q.1).abs() < 1e-9;
            if share(si.0, sj.0) || share(si.0, sj.1) || share(si.1, sj.0) || share(si.1, sj.1) {
                continue;
            }
            let d1 = cross(si.0, si.1, sj.0);
            let d2 = cross(si.0, si.1, sj.1);
            let d3 = cross(sj.0, sj.1, si.0);
            let d4 = cross(sj.0, sj.1, si.1);
            if ((d1 > eps && d2 < -eps) || (d1 < -eps && d2 > eps)) && ((d3 > eps && d4 < -eps) || (d3 < -eps && d4 > eps)) {
                // Schnittpunkt; Berührungen innerhalb der Toleranz an Endpunkten ignorieren
                let t = d1 / (d1 - d2);
                let ip = (sj.0 .0 + (sj.1 .0 - sj.0 .0) * t, sj.0 .1 + (sj.1 .1 - sj.0 .1) * t);
                let near = |p: (f64, f64)| ((p.0 - ip.0).powi(2) + (p.1 - ip.1).powi(2)).sqrt() < 0.5 * b.tol;
                if corners.iter().any(|&c| near(c)) {
                    continue;
                }
                if std::env::var("M2S_DEBUG").is_ok() {
                    let k = |e: u32| match b.edges[e as usize].curve { Curve::Line{..}=>"L", Curve::Circle{..}=>"C", Curve::Ellipse{..}=>"E", Curve::Poly(_)=>"P" };
                    eprintln!("Selbstschnitt: Kante {} ({}) x Kante {} ({}) bei {:?}", seg_edge[i], k(seg_edge[i]), seg_edge[j], k(seg_edge[j]), ip);
                    if std::env::var("M2S_DEBUG2").is_ok() { for e in [seg_edge[i], seg_edge[j]] { let ed = &b.edges[e as usize]; eprintln!("   e{e}: {:?} v0 {:?} v1 {:?}", ed.curve, b.verts[ed.v0 as usize], b.verts[ed.v1 as usize]); } }
                }
                hits.push(seg_edge[i]);
                hits.push(seg_edge[j]);
                if hits.len() > 40 {
                    return true;
                }
            }
        }
    }
    !hits.is_empty()
}

/// Äußere Schleife muss bzgl. der Außennormale gegen den Uhrzeigersinn laufen, innere im Uhrzeigersinn.
fn loop_orientation_ok(b: &Brep, f: &crate::brep::Face, o: V3, n: V3) -> bool {
    let (e1, e2) = n.basis();
    let mut areas = Vec::new();
    let mut polys: Vec<Vec<(f64, f64)>> = Vec::new();
    for lp in &f.loops {
        let mut pts: Vec<(f64, f64)> = Vec::new();
        for &(e, fwd) in lp {
            let ns = match b.edges[e as usize].curve {
                Curve::Line { .. } | Curve::Poly(_) => 1,
                _ => 64,
            };
            let mut ps = sample_edge(b, e as usize, ns);
            if !fwd {
                ps.reverse();
            }
            ps.pop();
            for p in ps {
                let q = p - o;
                pts.push((q.dot(e1), q.dot(e2)));
            }
        }
        let mut a = 0.0;
        for i in 0..pts.len() {
            let (x1, y1) = pts[i];
            let (x2, y2) = pts[(i + 1) % pts.len()];
            a += x1 * y2 - x2 * y1;
        }
        areas.push(a * 0.5);
        polys.push(pts);
    }
    if areas.is_empty() {
        return true;
    }
    let (imax, _) = areas.iter().enumerate().max_by(|a, b| a.1.abs().partial_cmp(&b.1.abs()).unwrap()).unwrap();
    let tiny = (b.tol * b.tol).max(1e-12);
    // Bei Ebenen/Höhenfeldern hat genau die äußere Schleife positive Fläche
    let mut ok = areas[imax] > -tiny && areas.iter().enumerate().all(|(i, &a)| i == imax || a < tiny);
    // Innere Schleifen müssen innerhalb der äußeren liegen
    let inside = |p: (f64, f64), poly: &[(f64, f64)]| {
        let mut c = false;
        for i in 0..poly.len() {
            let (a, b2) = (poly[i], poly[(i + 1) % poly.len()]);
            if (a.1 > p.1) != (b2.1 > p.1) && p.0 < (b2.0 - a.0) * (p.1 - a.1) / (b2.1 - a.1) + a.0 {
                c = !c;
            }
        }
        c
    };
    for (i, poly) in polys.iter().enumerate() {
        if i != imax && !poly.is_empty() {
            // Mittelpunkt einer Polygonkante (vermeidet gemeinsame Ecken)
            let p = ((poly[0].0 + poly[1 % poly.len()].0) * 0.5, (poly[0].1 + poly[1 % poly.len()].1) * 0.5);
            if !inside(p, &polys[imax]) {
                ok = false;
            }
        }
    }
    if !ok && std::env::var("M2S_DEBUG").is_ok() {
        eprintln!("Schleifenorientierung falsch: Flächen {:?} ({:?})", areas, f.surf.kind());
    }
    ok
}

/// Statistik der Abweichungen (Kante–Fläche je Flächenart, Ecke–Kante) für die Fehlersuche.
fn print_stats(b: &Brep) {
    let th = [0.001, 0.002, 0.005, 0.01, 0.02, 0.05];
    let mut by: std::collections::BTreeMap<String, ([usize; 6], f64)> = Default::default();
    for (fi, f) in b.faces.iter().enumerate() {
        let _ = fi;
        let mut seen = std::collections::HashSet::new();
        for &(e, _) in f.loops.iter().flatten() {
            if !seen.insert(e) { continue; }
            let d = sample_edge(b, e as usize, 16).iter().map(|&p| f.surf.dist(p).abs()).fold(0.0, f64::max);
            let ent = by.entry(format!("{:?}", f.surf.kind())).or_default();
            for (i, &t) in th.iter().enumerate() { if d > t { ent.0[i] += 1; } }
            ent.1 = ent.1.max(d);
        }
    }
    eprintln!("STATS Kante-Fläche (> {:?} mm):", th);
    for (k, (c, mx)) in &by { eprintln!("  {k:8} {:?} max {mx:.4}", c); }
    let mut vc = [0usize; 6];
    let mut vmax: f64 = 0.0;
    for e in &b.edges {
        for v in [e.v0, e.v1] {
            let d = curve_dist(&e.curve, b.verts[v as usize]);
            for (i, &t) in th.iter().enumerate() { if d > t { vc[i] += 1; } }
            vmax = vmax.max(d);
            if d > 0.005 {
                let k = match &e.curve { Curve::Line{..}=>"L", Curve::Circle{..}=>"C", Curve::Ellipse{..}=>"E", Curve::Poly(_)=>"P" };
                eprintln!("    Ecke {v} {:?} weicht {d:.4} von Kante ({k}) ab", b.verts[v as usize]);
            }
        }
    }
    eprintln!("  Ecke-Kante {:?} max {vmax:.4}", vc);
}
