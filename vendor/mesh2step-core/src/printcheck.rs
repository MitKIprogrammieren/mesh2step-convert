//! Druckbarkeit prüfen: Überhänge, scharfe Kanten, dünne Wände, kleine Details, Brücken,
//! Standfläche, Größe und Netzfehler – mit den betroffenen Dreiecken zum Markieren.

use crate::math::{v3, V3};
use std::collections::HashMap;

type T3 = [[f32; 3]; 3];

#[derive(Clone, Debug)]
pub struct CheckOpts {
    /// Düsendurchmesser (mm)
    pub nozzle: f64,
    /// Überhänge flacher als dieser Winkel zur Senkrechten brauchen Stützen (Grad)
    pub overhang_deg: f64,
    /// Mindestwandstärke (mm)
    pub min_wall: f64,
    /// Messerkanten: Innenwinkel darunter gilt als scharf (Grad)
    pub sharp_deg: f64,
    /// Druckraum (mm)
    pub bed: [f64; 3],
    /// längste Brücke ohne Stütze (mm)
    pub bridge_max: f64,
}

impl Default for CheckOpts {
    fn default() -> Self {
        CheckOpts { nozzle: 0.4, overhang_deg: 45.0, min_wall: 0.8, sharp_deg: 30.0, bed: [256.0, 256.0, 256.0], bridge_max: 10.0 }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Level {
    Ok,
    Warn,
    Err,
}

#[derive(Clone, Debug)]
pub struct Finding {
    /// Kennung: overhang, sharp, wall, detail, bridge, base, size, mesh
    pub key: &'static str,
    pub title: &'static str,
    pub summary: String,
    pub badge: String,
    pub level: Level,
    /// betroffene Dreiecke (Indizes der Eingabe)
    pub tris: Vec<u32>,
}

#[derive(Clone, Debug)]
pub struct PrintReport {
    pub findings: Vec<Finding>,
    pub level: Level,
    pub verdict: String,
    pub dims: [f64; 3],
}

fn v(p: [f32; 3]) -> V3 {
    v3(p[0] as f64, p[1] as f64, p[2] as f64)
}

fn de(x: f64, d: usize) -> String {
    format!("{x:.d$}").replace('.', ",")
}

/// Zusammenhängende Gruppen der markierten Dreiecke (über gemeinsame Kanten).
fn components(sel: &[u32], adj: &[Vec<u32>]) -> Vec<Vec<u32>> {
    let set: std::collections::HashSet<u32> = sel.iter().copied().collect();
    let mut seen = std::collections::HashSet::new();
    let mut out = Vec::new();
    for &s in sel {
        if !seen.insert(s) {
            continue;
        }
        let mut comp = vec![s];
        let mut stack = vec![s];
        while let Some(t) = stack.pop() {
            for &u in &adj[t as usize] {
                if set.contains(&u) && seen.insert(u) {
                    comp.push(u);
                    stack.push(u);
                }
            }
        }
        out.push(comp);
    }
    out
}

pub fn check(tris: &[T3], o: &CheckOpts) -> PrintReport {
    let n = tris.len();
    let mut lo = [f64::MAX; 3];
    let mut hi = [f64::MIN; 3];
    for t in tris {
        for p in t {
            for k in 0..3 {
                lo[k] = lo[k].min(p[k] as f64);
                hi[k] = hi[k].max(p[k] as f64);
            }
        }
    }
    if n == 0 {
        lo = [0.0; 3];
        hi = [0.0; 3];
    }
    let dims = [hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]];
    let normals: Vec<V3> = tris.iter().map(|t| (v(t[1]) - v(t[0])).cross(v(t[2]) - v(t[0]))).collect();
    let area: Vec<f64> = normals.iter().map(|x| x.norm() * 0.5).collect();
    let unit: Vec<V3> = normals.iter().map(|x| if x.norm() > 1e-30 { x.normalized() } else { V3::ZERO }).collect();
    let total_area: f64 = area.iter().sum::<f64>().max(1e-12);
    let zmin = lo[2];
    let on_bed = |t: usize| tris[t].iter().all(|p| (p[2] as f64) < zmin + 0.25);

    // Kanten über gleiche Punkte (Netz aus der Datei kann doppelte Eckpunkte haben)
    let key = |p: [f32; 3]| ((p[0] as f64 * 1e4).round() as i64, (p[1] as f64 * 1e4).round() as i64, (p[2] as f64 * 1e4).round() as i64);
    let mut ids: HashMap<(i64, i64, i64), u32> = HashMap::with_capacity(n * 2);
    let tv: Vec<[u32; 3]> = tris
        .iter()
        .map(|t| {
            t.map(|p| {
                let l = ids.len() as u32;
                *ids.entry(key(p)).or_insert(l)
            })
        })
        .collect();
    let mut edges: HashMap<(u32, u32), Vec<u32>> = HashMap::with_capacity(n * 2);
    for (i, t) in tv.iter().enumerate() {
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            if a != b {
                edges.entry((a.min(b), a.max(b))).or_default().push(i as u32);
            }
        }
    }
    let mut adj: Vec<Vec<u32>> = vec![Vec::new(); n];
    let (mut open, mut nonman) = (0usize, 0usize);
    let mut bad_mesh: Vec<u32> = Vec::new();
    for l in edges.values() {
        match l.len() {
            1 => {
                open += 1;
                bad_mesh.push(l[0]);
            }
            2 => {
                adj[l[0] as usize].push(l[1]);
                adj[l[1] as usize].push(l[0]);
            }
            _ => {
                nonman += 1;
                bad_mesh.extend_from_slice(l);
            }
        }
    }
    let mut findings = Vec::new();

    // 1 Überhänge
    let lim = o.overhang_deg.to_radians().sin();
    let over: Vec<u32> = (0..n).filter(|&t| unit[t].z < -lim && !on_bed(t)).map(|t| t as u32).collect();
    let over_area: f64 = over.iter().map(|&t| area[t as usize]).sum();
    findings.push(if over_area < 0.5 {
        Finding { key: "overhang", title: "Überhänge", summary: format!("nichts flacher als {}°", de(o.overhang_deg, 0)), badge: "ok".into(), level: Level::Ok, tris: over }
    } else {
        Finding {
            key: "overhang",
            title: "Überhänge",
            summary: format!("{} cm² flacher als {}°", de(over_area / 100.0, 1), de(o.overhang_deg, 0)),
            badge: "Stützen nötig".into(),
            level: Level::Warn,
            tris: over,
        }
    });

    // 2 Scharfe Kanten (nach außen gewölbte Kanten mit kleinem Innenwinkel)
    let sharp_cos = (std::f64::consts::PI - o.sharp_deg.to_radians()).cos();
    let mut sharp_tris: Vec<u32> = Vec::new();
    let mut sharp_edges = 0usize;
    for l in edges.values() {
        if l.len() != 2 {
            continue;
        }
        let (a, b) = (l[0] as usize, l[1] as usize);
        let d = unit[a].dot(unit[b]);
        if d < sharp_cos {
            // gewölbt nach außen? Mittelpunkt von b liegt hinter der Ebene von a
            let ca = (v(tris[a][0]) + v(tris[a][1]) + v(tris[a][2])) / 3.0;
            let cb = (v(tris[b][0]) + v(tris[b][1]) + v(tris[b][2])) / 3.0;
            if (cb - ca).dot(unit[a]) < 0.0 {
                sharp_edges += 1;
                sharp_tris.push(a as u32);
                sharp_tris.push(b as u32);
            }
        }
    }
    sharp_tris.sort_unstable();
    sharp_tris.dedup();
    findings.push(if sharp_edges == 0 {
        Finding { key: "sharp", title: "Scharfe Kanten", summary: format!("keine Kanten unter {}°", de(o.sharp_deg, 0)), badge: "ok".into(), level: Level::Ok, tris: sharp_tris }
    } else {
        let places = components(&sharp_tris, &adj).len();
        Finding {
            key: "sharp",
            title: "Scharfe Kanten",
            summary: format!("{places} Stelle{} spitzer als {}°", if places == 1 { "" } else { "n" }, de(o.sharp_deg, 0)),
            badge: "brechen leicht".into(),
            level: Level::Warn,
            tris: sharp_tris,
        }
    });

    // 3/4 Wandstärke und kleine Details
    let th = crate::inspect::thickness_checked(tris, o.min_wall);
    // Splitterdreiecke an Rundungsanfängen liefern zufällig kurze Strahlen – nicht mitzählen
    let min_tri = (o.nozzle * o.nozzle * 0.08).max(1e-4);
    let thin: Vec<u32> = (0..n).filter(|&t| area[t] > min_tri && th[t].is_finite() && (th[t] as f64) < o.min_wall && (th[t] as f64) >= o.nozzle).map(|t| t as u32).collect();
    let tiny: Vec<u32> = (0..n).filter(|&t| area[t] > min_tri && th[t].is_finite() && (th[t] as f64) < o.nozzle && th[t] > 0.01).map(|t| t as u32).collect();
    // nur zusammenhängende Stellen mit nennenswerter Fläche zählen (und markieren)
    let significant = |sel: &[u32], min_area: f64| -> (usize, Vec<u32>) {
        let comps: Vec<Vec<u32>> = components(sel, &adj).into_iter().filter(|c| c.iter().map(|&t| area[t as usize]).sum::<f64>() > min_area).collect();
        (comps.len(), comps.into_iter().flatten().collect())
    };
    let (thin_places, thin) = significant(&thin, (o.min_wall * 3.0).powi(2));
    let (tiny_places, tiny) = significant(&tiny, (o.nozzle * 4.0).powi(2));
    findings.push(if thin_places == 0 {
        Finding { key: "wall", title: "Dünne Wände", summary: format!("alles mindestens {} mm", de(o.min_wall, 1)), badge: "ok".into(), level: Level::Ok, tris: thin }
    } else {
        Finding {
            key: "wall",
            title: "Dünne Wände",
            summary: format!("{thin_places} Stelle{} unter {} mm", if thin_places == 1 { "" } else { "n" }, de(o.min_wall, 1)),
            badge: "knapp".into(),
            level: Level::Warn,
            tris: thin,
        }
    });
    findings.push(if tiny_places == 0 {
        Finding { key: "detail", title: "Kleine Details", summary: format!("nichts dünner als {} mm", de(o.nozzle, 1)), badge: "ok".into(), level: Level::Ok, tris: tiny }
    } else {
        Finding {
            key: "detail",
            title: "Kleine Details",
            summary: format!("{tiny_places} Stelle{} dünner als die Düse", if tiny_places == 1 { "" } else { "n" }),
            badge: "gehen evtl. verloren".into(),
            level: Level::Warn,
            tris: tiny,
        }
    });

    // 5 Brücken: waagrechte Unterseiten in der Luft, kleinste Ausdehnung = Spannweite
    let flat_down: Vec<u32> = (0..n).filter(|&t| unit[t].z < -0.985 && !on_bed(t)).map(|t| t as u32).collect();
    let mut span_max = 0.0f64;
    let mut bridge_tris: Vec<u32> = Vec::new();
    for comp in components(&flat_down, &adj) {
        let (mut a, mut b) = ([f64::MAX; 2], [f64::MIN; 2]);
        for &t in &comp {
            for p in &tris[t as usize] {
                for k in 0..2 {
                    a[k] = a[k].min(p[k] as f64);
                    b[k] = b[k].max(p[k] as f64);
                }
            }
        }
        let span = (b[0] - a[0]).min(b[1] - a[1]);
        if span > o.bridge_max {
            bridge_tris.extend_from_slice(&comp);
        }
        span_max = span_max.max(span);
    }
    findings.push(if span_max <= o.bridge_max {
        Finding {
            key: "bridge",
            title: "Brücken",
            summary: if span_max > 0.0 { format!("längste freie Spannweite {} mm", de(span_max, 0)) } else { "keine freien Spannweiten".into() },
            badge: "ok".into(),
            level: Level::Ok,
            tris: bridge_tris,
        }
    } else {
        Finding { key: "bridge", title: "Brücken", summary: format!("bis {} mm freie Spannweite", de(span_max, 0)), badge: "hängt durch".into(), level: Level::Warn, tris: bridge_tris }
    });

    // 6 Standfläche
    let base: Vec<u32> = (0..n).filter(|&t| on_bed(t) && unit[t].z < -0.9).map(|t| t as u32).collect();
    let base_area: f64 = base.iter().map(|&t| area[t as usize]).sum();
    let foot = (dims[0] * dims[1]).max(1e-9);
    findings.push(if base_area < 1.0 {
        Finding { key: "base", title: "Standfläche", summary: "liegt kaum auf dem Druckbett auf".into(), badge: "Lage ändern".into(), level: Level::Err, tris: base }
    } else if dims[2] > 4.0 * base_area.sqrt() || base_area < 0.08 * foot {
        Finding {
            key: "base",
            title: "Standfläche",
            summary: format!("Auflage {} cm² · hoch und schmal", de(base_area / 100.0, 1)),
            badge: "Brim empfohlen".into(),
            level: Level::Warn,
            tris: base,
        }
    } else {
        Finding { key: "base", title: "Standfläche", summary: format!("Auflage {} cm² · steht sicher", de(base_area / 100.0, 1)), badge: "ok".into(), level: Level::Ok, tris: base }
    });

    // 7 Größe
    let fits = dims[0] <= o.bed[0] && dims[1] <= o.bed[1] && dims[2] <= o.bed[2];
    let fits_rot = dims[1] <= o.bed[0] && dims[0] <= o.bed[1] && dims[2] <= o.bed[2];
    let dim_txt = format!("{} × {} × {} mm", de(dims[0], 0), de(dims[1], 0), de(dims[2], 0));
    findings.push(if fits || fits_rot {
        Finding { key: "size", title: "Größe", summary: format!("{dim_txt} · passt aufs Bett"), badge: "ok".into(), level: Level::Ok, tris: Vec::new() }
    } else {
        Finding { key: "size", title: "Größe", summary: format!("{dim_txt} · größer als der Druckraum"), badge: "zu groß".into(), level: Level::Err, tris: Vec::new() }
    });

    // 8 Netz
    bad_mesh.sort_unstable();
    bad_mesh.dedup();
    findings.push(if open == 0 && nonman == 0 {
        Finding { key: "mesh", title: "Netz", summary: "geschlossen, keine Fehler".into(), badge: "ok".into(), level: Level::Ok, tris: Vec::new() }
    } else {
        let mut parts = Vec::new();
        if open > 0 {
            parts.push(format!("{open} offene Kanten"));
        }
        if nonman > 0 {
            parts.push(format!("{nonman} nicht-mannigfaltig"));
        }
        Finding { key: "mesh", title: "Netz", summary: parts.join(" · "), badge: "reparieren".into(), level: Level::Warn, tris: bad_mesh }
    });

    let level = findings.iter().map(|f| f.level).max().unwrap_or(Level::Ok);
    let hints = findings.iter().filter(|f| f.level != Level::Ok).count();
    let verdict = match level {
        Level::Ok => "Druckbar".to_string(),
        Level::Err => "Probleme beim Druck".to_string(),
        Level::Warn => {
            if findings.iter().any(|f| f.key == "overhang" && f.level == Level::Warn) {
                "Druckbar mit Stützen".to_string()
            } else {
                format!("Druckbar · {hints} Hinweis{}", if hints == 1 { "" } else { "e" })
            }
        }
    };
    let _ = total_area;
    PrintReport { findings, level, verdict, dims }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn boxm(x0: f32, y0: f32, z0: f32, x1: f32, y1: f32, z1: f32) -> Vec<T3> {
        let p = |i: usize| -> [f32; 3] { [if i & 1 == 1 { x1 } else { x0 }, if i & 2 == 2 { y1 } else { y0 }, if i & 4 == 4 { z1 } else { z0 }] };
        let t = [[0, 2, 1], [1, 2, 3], [4, 5, 6], [5, 7, 6], [0, 1, 4], [1, 5, 4], [2, 6, 3], [3, 6, 7], [0, 4, 2], [2, 4, 6], [1, 3, 5], [3, 7, 5]];
        t.iter().map(|x| [p(x[0]), p(x[1]), p(x[2])]).collect()
    }

    fn get<'a>(r: &'a PrintReport, k: &str) -> &'a Finding {
        r.findings.iter().find(|f| f.key == k).unwrap()
    }

    #[test]
    fn plain_box_is_printable() {
        let r = check(&boxm(0.0, 0.0, 0.0, 20.0, 20.0, 10.0), &CheckOpts::default());
        assert_eq!(r.level, Level::Ok, "{:?}", r.findings.iter().map(|f| (f.key, &f.summary)).collect::<Vec<_>>());
        assert_eq!(r.verdict, "Druckbar");
    }

    #[test]
    fn floating_box_needs_support_and_thin_plate_found() {
        // Platte 0,6 mm dick auf einem Sockel – Unterseite steht über (Brücke/Überhang)
        let mut t = boxm(0.0, 0.0, 0.0, 4.0, 4.0, 10.0);
        t.extend(boxm(-15.0, -15.0, 10.0, 19.0, 19.0, 10.6));
        let r = check(&t, &CheckOpts::default());
        assert_eq!(get(&r, "overhang").level, Level::Warn);
        assert_eq!(get(&r, "wall").level, Level::Warn, "{}", get(&r, "wall").summary);
        assert_eq!(get(&r, "bridge").level, Level::Warn);
        assert!(get(&r, "mesh").level == Level::Ok || get(&r, "mesh").level == Level::Warn);
    }

    #[test]
    fn too_big_for_bed() {
        let r = check(&boxm(0.0, 0.0, 0.0, 300.0, 20.0, 10.0), &CheckOpts::default());
        assert_eq!(get(&r, "size").level, Level::Err);
        assert_eq!(r.level, Level::Err);
    }
}
