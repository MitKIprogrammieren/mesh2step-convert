//! mesh2step – wandelt STL/3MF-Dreiecksnetze in STEP-Volumenkörper mit
//! echten analytischen Flächen (Ebene, Zylinder, Kegel, Kugel, Torus) um.

pub mod brep;
pub mod bspline;
pub mod check;
pub mod compare;
pub mod engines;
pub mod export3mf;
pub mod external;
pub mod hollow;
pub mod inspect;
pub mod io;
pub mod json;
pub mod math;
pub mod mesh;
pub mod orient;
pub mod paint;
pub mod poly;
pub mod printcheck;
pub mod repair;
pub mod segment;
pub mod simplify;
pub mod split;
pub mod step;
pub mod stepread;
pub mod surf;
pub mod testgen;
pub mod texture;
pub mod union;
pub mod vector;
pub mod zip;

pub use segment::{Cancelled, Mode, Params};

use std::collections::BTreeMap;
use std::path::Path;

#[derive(Debug, Clone, Default)]
pub struct Report {
    pub triangles: usize,
    pub bodies: usize,
    pub faces: usize,
    pub faces_by_kind: BTreeMap<String, usize>,
    pub edges: usize,
    pub tolerance: f64,
    pub max_deviation: f64,
    pub warnings: Vec<String>,
    pub check_errors: Vec<String>,
    pub seconds: f64,
    /// Daten für die 3D-Vorschau (Dreiecke eingefärbt nach erkannter Fläche)
    pub preview: Option<std::sync::Arc<Preview>>,
    /// Erkannte Bohrungen (gleiche zusammengefasst), Dreiecke beziehen sich auf die Vorschau
    pub holes: Vec<inspect::HoleGroup>,
    /// erstellt mit (leer = Mesh2STEP), z. B. "stl2step 1.7.0 · TrueForm"
    pub engine: String,
    /// Hinweise (keine Warnungen), z. B. automatisch angepasste Toleranz
    pub notes: Vec<String>,
    /// Volumen-Abweichung STEP gegen Netz in Prozent (falls gemessen)
    pub volume_delta_pct: Option<f64>,
}

/// Flächenart je Dreieck für die Vorschau.
pub mod preview_kind {
    pub const PLANE: u8 = 0;
    pub const CYL: u8 = 1;
    pub const CONE: u8 = 2;
    pub const SPHERE: u8 = 3;
    pub const TORUS: u8 = 4;
    pub const BSPLINE: u8 = 5;
    /// einzelnes Dreieck ohne erkannte Grundform
    pub const FACET: u8 = 6;
    /// noch nicht umgewandelt (Eingabe)
    pub const RAW: u8 = 255;
}

#[derive(Debug, Clone, Default)]
pub struct Preview {
    pub tris: Vec<[[f32; 3]; 3]>,
    pub kind: Vec<u8>,
    /// Grenzkanten zwischen Flächen: Endpunkte und die beiden angrenzenden Dreiecke
    pub edges: Vec<([[f32; 3]; 2], u32, u32)>,
    /// Dreiecke nicht einheitlich orientiert: beide Seiten zeichnen
    pub two_sided: bool,
    /// feste Farbe je Dreieck (ersetzt die Farbe nach Flächenart)
    pub rgb: Vec<[u8; 3]>,
    /// Gruppe je Dreieck (Farbbereich bzw. Teil) für Auswahl und Explosionsansicht
    pub group: Vec<u32>,
}

/// Standardfarben für Filamente, falls die Datei keine enthält.
pub const DEFAULT_PALETTE: [[u8; 3]; 16] = [
    [236, 236, 236],
    [220, 52, 52],
    [40, 90, 210],
    [40, 160, 70],
    [240, 190, 30],
    [50, 50, 55],
    [240, 130, 30],
    [140, 70, 200],
    [30, 170, 190],
    [230, 100, 170],
    [140, 90, 50],
    [150, 200, 60],
    [120, 120, 130],
    [250, 220, 160],
    [20, 80, 80],
    [190, 30, 90],
];

pub fn filament_rgb(colors: &[[u8; 3]], f: u8) -> [u8; 3] {
    let i = (f.max(1) - 1) as usize;
    colors.get(i).copied().unwrap_or(DEFAULT_PALETTE[i % DEFAULT_PALETTE.len()])
}

/// Vorschau aus farbigen Netzen: (Eckpunkte, Dreiecke, Filament je Dreieck, Gruppe je Dreieck).
/// Grenzlinien werden zwischen verschiedenen Gruppen gezogen.
pub fn preview_colored(items: &[(&[math::V3], &[[u32; 3]], &[u8], &[u32])], colors: &[[u8; 3]]) -> Preview {
    let mut p = Preview::default();
    let f = |v: math::V3| [v.x as f32, v.y as f32, v.z as f32];
    for (verts, tris, col, group) in items {
        let base = p.tris.len() as u32;
        let mut em: std::collections::HashMap<(u32, u32), u32> = std::collections::HashMap::with_capacity(tris.len() * 3);
        for (t, tri) in tris.iter().enumerate() {
            p.tris.push([f(verts[tri[0] as usize]), f(verts[tri[1] as usize]), f(verts[tri[2] as usize])]);
            p.kind.push(preview_kind::RAW);
            p.rgb.push(filament_rgb(colors, col[t]));
            p.group.push(group[t]);
            for k in 0..3 {
                em.insert((tri[k], tri[(k + 1) % 3]), t as u32);
            }
        }
        for (&(a, b), &t) in &em {
            match em.get(&(b, a)) {
                Some(&u) if a < b => {
                    if group[t as usize] != group[u as usize] {
                        p.edges.push(([f(verts[a as usize]), f(verts[b as usize])], base + t, base + u));
                    }
                }
                None => p.edges.push(([f(verts[a as usize]), f(verts[b as usize])], base + t, base + t)),
                _ => {}
            }
        }
    }
    p
}

impl Preview {
    fn append_mesh(&mut self, m: &mesh::Mesh, seg: Option<&segment::Segmentation>) {
        let base = self.tris.len() as u32;
        let f = |v: math::V3| [v.x as f32, v.y as f32, v.z as f32];
        for (t, tri) in m.tris.iter().enumerate() {
            self.tris.push([f(m.verts[tri[0] as usize]), f(m.verts[tri[1] as usize]), f(m.verts[tri[2] as usize])]);
            let k = match seg {
                None => preview_kind::RAW,
                Some(s) => {
                    let r = &s.regions[s.region_of[t] as usize];
                    match r.surf {
                        surf::Surf::Plane { .. } if r.tris.len() <= 1 => preview_kind::FACET,
                        other => other.kind() as u8,
                    }
                }
            };
            self.kind.push(k);
        }
        if let Some(s) = seg {
            for h in 0..m.twin.len() {
                let tw = m.twin[h];
                let t1 = h / 3;
                let open = tw == mesh::NONE;
                if open || (tw as usize > h && s.region_of[t1] != s.region_of[(tw / 3) as usize]) {
                    let a = m.verts[m.he_from(h as u32) as usize];
                    let b = m.verts[m.he_to(h as u32) as usize];
                    let t2 = if open { t1 as u32 } else { tw / 3 };
                    self.edges.push(([f(a), f(b)], base + t1 as u32, base + t2));
                }
            }
        }
    }
}

/// Vorschau der Eingabedatei (unverarbeitet), z. B. direkt nach dem Hinzufügen.
pub fn preview_input(path: &Path) -> Result<Preview, String> {
    let raws = io::load_file(path)?;
    let mut p = Preview::default();
    for raw in &raws {
        // Netz wie bei der Umwandlung aufbereiten, damit alle Dreiecke nach außen zeigen
        // (viele STL-Exporte haben gemischte Umlaufrichtungen)
        match mesh::build(raw) {
            Ok(m) => p.append_mesh(&m, None),
            Err(_) => {
                p.two_sided = true;
                let f = |v: math::V3| [v.x as f32, v.y as f32, v.z as f32];
                for t in &raw.tris {
                    p.tris.push([f(raw.verts[t[0] as usize]), f(raw.verts[t[1] as usize]), f(raw.verts[t[2] as usize])]);
                    p.kind.push(preview_kind::RAW);
                }
            }
        }
    }
    Ok(p)
}

impl Report {
    pub fn summary_de(&self) -> String {
        let kinds: Vec<String> = self.faces_by_kind.iter().map(|(k, v)| format!("{v} {k}")).collect();
        format!(
            "{} Flächen ({}), {} Kanten, {} Körper – {:.1} s",
            self.faces,
            kinds.join(", "),
            self.edges,
            self.bodies,
            self.seconds
        )
    }
}

/// Konvertiert bereits geladene Netze in einen STEP-Text.
/// Ein Durchlauf mit fester Toleranz: Flächen erkennen, B-Rep aufbauen, Problemflächen facettieren.
fn build_at(m: &mesh::Mesh, prm: &Params, tol: f64, name: &str) -> (segment::Segmentation, brep::Brep, check::CheckResult, usize) {
    let mut seg = segment::segment(m, prm, tol);
    let mut force_poly: std::collections::HashSet<u32> = std::collections::HashSet::new();
        prm.report(0.75, "Kanten und Ecken berechnen");
        if std::env::var("M2S_DEBUG").is_ok() {
            for (i, r) in seg.regions.iter().enumerate() {
                eprintln!("R{i}: {} Dreiecke {:?}", r.tris.len(), r.surf);
            }
        }
        // Flächen, deren Kanten nicht sauber auf ihnen liegen, auf exakte Facetten zurückstufen
        let mut round = 0;
        let mut demoted = 0usize;
        let mut last_bad = usize::MAX;
        let mut stagnant = 0;
        let (b, ck) = loop {
            let has_ff = seg.regions.iter().any(|r| matches!(r.surf, surf::Surf::BSpline { .. }));
            let btol = if has_ff { segment::freeform_tol(m, tol) } else { tol };
            let mut b = brep::build(m, &seg, btol, tol, name, &force_poly);
            if has_ff {
                // Freiformflächen an ihre Kanten anpassen und Kanten neu projizieren, bis beides
                // zusammenpasst (sonst trimmt das CAD-Programm die Fläche neben der Kante → Spalt)
                for _ in 0..4 {
                    let n = brep::snap_freeform(&mut b, m, &mut seg, btol * 2.0);
                    let moved = brep::project_ff_edges(&mut b);
                    if n == 0 && !moved {
                        break;
                    }
                }
            }
            prm.report(0.8, &format!("Kanten prüfen (Runde {})", round + 1));
            if std::env::var("M2S_DEBUG").is_ok() { eprintln!("=== Runde {} ===", round + 1); }
            let ck = check::check(&b);
            round += 1;
            // Abbrechen, wenn die Nachbesserung nichts mehr bringt (spart viel Zeit)
            let nbad = ck.bad_faces.len();
            if nbad >= last_bad {
                stagnant += 1;
            } else {
                stagnant = 0;
            }
            last_bad = nbad;
            if ck.bad_faces.is_empty() || round > 9 || stagnant >= 2 {
                if std::env::var("M2S_DEBUG").is_ok() {
                    eprintln!("Runde {round}: {} problematische Flächen verbleiben", ck.bad_faces.len());
                    for &f in &ck.bad_faces {
                        eprintln!("  übrig: Fläche {f} {:?} {} Dreiecke", b.faces[f as usize].surf.kind(), seg.regions[b.faces[f as usize].region as usize].tris.len());
                    }
                }
                break (b, ck);
            }
            let _bad_regions: Vec<u32> = ck.bad_faces.iter().map(|&f| b.faces[f as usize].region).collect();
            if std::env::var("M2S_DEBUG").is_ok() {
                for &f in &ck.bad_faces {
                    let r = b.faces[f as usize].region as usize;
                    eprintln!("demote face {f}: {:?} {} Dreiecke, {} Schleifen", seg.regions[r].surf.kind(), seg.regions[r].tris.len(), b.faces[f as usize].loops.len());
                }
            }
            // Große Freiformflächen nur dort nachbessern, wo die Problemkanten liegen
            let mut whole = Vec::new();
            let mut local_tris: Vec<u32> = Vec::new();
            // Zuerst: Problemkanten, die als Gerade/Bogen angenähert sind, als Polygonzug
            // (exakt dem Netz folgend) neu aufbauen – statt die ganze Fläche zu facettieren
            let mut spared: std::collections::HashSet<u32> = std::collections::HashSet::new();
            for &f in &ck.bad_faces {
                let mut any = false;
                for &(ff, e) in &ck.bad_edges {
                    if ff != f {
                        continue;
                    }
                    let ed = &b.edges[e as usize];
                    if !matches!(ed.curve, brep::Curve::Poly(_)) && ed.hes.iter().any(|h| !force_poly.contains(h)) {
                        force_poly.extend(ed.hes.iter().copied());
                        any = true;
                    }
                }
                if any {
                    spared.insert(f);
                }
            }
            // Achterschleifen großer Flächen: nur die Dreiecke an der doppelt besuchten Ecke
            // herauslösen, statt die ganze Fläche zu facettieren
            let spared_before = spared.clone();
            let mut pinch_tris: Vec<u32> = Vec::new();
            for &(f, pos) in &ck.pinches {
                let face = &b.faces[f as usize];
                let r = face.region as usize;
                if spared_before.contains(&f) || seg.regions[r].tris.len() <= 30 || round > 6 {
                    continue;
                }
                // Nächste Ecke der Fläche suchen (die Position kommt aus der Kante und liegt nicht
                // immer exakt auf einem Netzpunkt)
                let mut best = (f64::MAX, u32::MAX);
                for &t in &seg.regions[r].tris {
                    for &v in &m.tris[t as usize] {
                        let d = m.verts[v as usize].dist(pos);
                        if d < best.0 {
                            best = (d, v);
                        }
                    }
                }
                let near: Vec<u32> = if best.0 < (tol * 5.0).max(0.02) {
                    seg.regions[r].tris.iter().copied().filter(|&t| m.tris[t as usize].contains(&best.1)).collect()
                } else {
                    Vec::new()
                };
                // Lange Streifendreiecke (grob vernetzte Zylinder) behalten: sie tragen die Fläche,
                // die Achterschleife entsteht an den kleinen Fächerdreiecken der Ecke
                let area = |t: u32| { let [a, b2, c] = m.tris[t as usize].map(|v| m.verts[v as usize]); (b2 - a).cross(c - a).norm() / 2.0 };
                let mut ar: Vec<f64> = near.iter().map(|&t| area(t)).collect();
                ar.sort_by(|a, b| a.partial_cmp(b).unwrap());
                let med = ar.get(ar.len() / 2).copied().unwrap_or(0.0);
                let near: Vec<u32> = if near.len() > 1 { near.into_iter().filter(|&t| area(t) <= med * 10.0).collect() } else { near };
                if !near.is_empty() {
                    if std::env::var("M2S_DEBUG").is_ok() {
                        for &t in &near {
                            let [a, b2, c] = m.tris[t as usize].map(|v| m.verts[v as usize]);
                            eprintln!("  Achter-Dreieck {t}: Fläche {:.4} {:?} {:?} {:?}", (b2 - a).cross(c - a).norm() / 2.0, a, b2, c);
                        }
                    }
                    pinch_tris.extend(near);
                    spared.insert(f);
                }
            }
            for &f in &ck.bad_faces {
                if spared.contains(&f) {
                    continue;
                }
                let face = &b.faces[f as usize];
                let r = face.region as usize;
                let edges: Vec<u32> = ck.bad_edges.iter().filter(|(ff, _)| *ff == f).map(|(_, e)| *e).collect();
                let big = matches!(face.surf, surf::Surf::BSpline { .. }) && seg.regions[r].tris.len() > 60 && round <= 5;
                if big && !edges.is_empty() {
                    for e in edges {
                        for &h in &b.edges[e as usize].hes {
                            for t in [h / 3, m.twin[h as usize] / 3] {
                                if (t as usize) < m.tris.len() && seg.region_of[t as usize] == r as u32 {
                                    local_tris.push(t);
                                }
                            }
                        }
                    }
                } else {
                    whole.push(face.region);
                }
            }
            demoted += whole.len();
            segment::demote_to_facets(m, &mut seg, &whole);
            if !local_tris.is_empty() {
                segment::demote_tris(m, &mut seg, &local_tris, 1);
            }
            if !pinch_tris.is_empty() {
                let rings = std::env::var("M2S_PINCHRINGS").ok().and_then(|v| v.parse().ok()).unwrap_or(1);
                segment::demote_tris(m, &mut seg, &pinch_tris, rings);
            }
        };
    (seg, b, ck, demoted)
}

pub fn convert_meshes(raws: &[io::RawMesh], name: &str, file_name: &str, prm: &Params) -> Result<(String, Report), String> {
    let t0 = std::time::Instant::now();
    let mut rep = Report::default();
    let mut breps = Vec::new();
    let mut preview = Preview::default();
    let mut holes = Vec::new();
    for (i, raw) in raws.iter().enumerate() {
        prm.report(0.0, &format!("Netz aufbereiten ({}/{})", i + 1, raws.len()));
        let m = mesh::build(raw)?;
        let tol = prm.tolerance(&m);
        rep.triangles += m.tris.len();
        rep.warnings.extend(m.warnings.iter().map(|w| if raws.len() > 1 { format!("{}: {w}", raw.name) } else { w.clone() }));
        // Automatische Toleranz: zerfällt das Ergebnis in sehr viele Einzelfacetten (z. B. CAD-Export
        // mit fein unterteilten, leicht gekrümmten Flächen wie Zahnflanken), gröbere Toleranzen
        // probieren und das Ergebnis mit den wenigsten Flächen nehmen. Die Stufen laufen parallel.
        let auto = prm.tol.is_none() && prm.mode != segment::Mode::Facets;
        // Große, exakte CAD-Netze: die sehr enge Anfangstoleranz dauert lange und verliert fast
        // immer gegen die Stufen – gleich mit den Stufen beginnen
        let skip_tight = auto && m.tris.len() > 30_000 && tol < 0.0015;
        let steps: Vec<f64> = [0.002, 0.005, 0.01, 0.02].into_iter().filter(|&t| t > tol * 1.5).collect();
        let run_steps = |prm: &Params| -> Vec<(f64, (segment::Segmentation, brep::Brep, check::CheckResult, usize))> {
            prm.report(0.0, &format!("{} Toleranzstufen parallel berechnen", steps.len()));
            std::thread::scope(|sc| {
                let hs: Vec<_> = steps
                    .iter()
                    .map(|&t2| {
                        let (m, name) = (&m, &raw.name);
                        // Fortschritt nur vom ersten Durchlauf melden
                        let p2 = if t2 == steps[0] { prm.clone() } else { Params { progress: None, ..prm.clone() } };
                        sc.spawn(move || (t2, build_at(m, &p2, t2, name)))
                    })
                    .collect();
                let r: Vec<_> = hs.into_iter().filter_map(|h| h.join().ok()).collect();
                // abgebrochene Stufen: Abbruch hier weiterreichen
                prm.report(0.95, "Ergebnisse vergleichen");
                r
            })
        };
        let open = |b: &brep::Brep| b.shells.iter().filter(|s| !s.closed).count();
        let (mut seg, mut b, mut ck, mut demoted);
        let mut tol = tol;
        let mut results = Vec::new();
        if skip_tight && !steps.is_empty() {
            results = run_steps(prm);
        }
        if !results.is_empty() {
            let (t0, r0) = results.remove(0);
            (seg, b, ck, demoted) = r0;
            tol = t0;
        } else {
            (seg, b, ck, demoted) = build_at(&m, prm, tol, &raw.name);
            let tiny = seg.regions.iter().filter(|r| r.tris.len() <= 2 && matches!(r.surf, surf::Surf::Plane { .. })).count();
            if auto && tiny * 4 > b.faces.len() && b.faces.len() > 300 && !steps.is_empty() {
                results = run_steps(prm);
            }
        }
        if !results.is_empty() {
            let n0 = b.faces.len();
            let dev0 = ck.max_edge_dev.max(ck.max_vertex_dev);
            let tiny_of = |s: &segment::Segmentation| s.regions.iter().filter(|r| r.tris.len() <= 2 && matches!(r.surf, surf::Surf::Plane { .. })).count();
            let tiny0 = tiny_of(&seg);
            for (t2, (s2, b2, c2, d2)) in results {
                // Abweichung höchstens 0,025 mm bzw. das Doppelte der Toleranz (oder nicht schlechter
                // als das erste Ergebnis) – unter der Druckgenauigkeit, Rundungen bleiben erhalten
                let dev2 = c2.max_edge_dev.max(c2.max_vertex_dev);
                if std::env::var("M2S_DEBUG").is_ok() {
                    eprintln!("Toleranz {t2:.4}: {} Flächen, Abweichung {dev2:.4} (bisher {}, anfangs {n0}), Kleinst-Ebenen {} (anfangs {tiny0})", b2.faces.len(), b.faces.len(), tiny_of(&s2));
                }
                // Deutlich weniger Flächen als am Anfang – oder grob vernetzte Rundungen, die erst
                // mit der gröberen Stufe als Grundform erkannt werden (Facetten halbiert)
                let gain_vs_start = skip_tight || b2.faces.len() * 10 < n0 * 7 || (b2.faces.len() * 10 < n0 * 9 && tiny_of(&s2) * 2 <= tiny0);
                if gain_vs_start && b2.faces.len() * 100 < b.faces.len() * 92 && dev2 <= (2.0 * t2).max(0.025).max(dev0 * 1.1) && c2.errors.len() <= ck.errors.len() && open(&b2) <= open(&b) {
                    seg = s2;
                    b = b2;
                    ck = c2;
                    demoted = d2;
                    tol = t2;
                }
            }
        }
        rep.tolerance = rep.tolerance.max(tol);
        let (b, ck) = (b, ck);
        if let Ok(e) = std::env::var("M2S_EDGE") {
            if let Ok(e) = e.parse::<usize>() {
                if let Some(ed) = b.edges.get(e) {
                    eprintln!("EDGE {e}: {:?}", ed.curve);
                }
            }
        }
        if let Ok(list) = std::env::var("M2S_FACE") {
            let mut ids: Vec<usize> = list.split(',').filter_map(|x| x.parse::<usize>().ok()).collect();
            if let Some(v) = list.strip_prefix("v:") {
                let c: Vec<f64> = v.split(';').filter_map(|x| x.parse().ok()).collect();
                let q = math::v3(c[0], c[1], c[2]);
                for (fi, f) in b.faces.iter().enumerate() {
                    if f.loops.iter().flatten().any(|&(e, _)| { let ed = &b.edges[e as usize]; b.verts[ed.v0 as usize].dist(q) < 2e-3 || b.verts[ed.v1 as usize].dist(q) < 2e-3 }) {
                        ids.push(fi);
                    }
                }
            }
            for fi in ids {
                if fi >= b.faces.len() { continue; }
                let f = &b.faces[fi];
                let r = &seg.regions[f.region as usize];
                eprintln!("FACE {fi}: {:?} same_sense {} {} Dreiecke", f.surf, f.same_sense, r.tris.len());
                for lp in &f.loops {
                    for &(e, fwd) in lp {
                        let ed = &b.edges[e as usize];
                        let k = match &ed.curve { brep::Curve::Line{..}=>"L".to_string(), brep::Curve::Circle{r,..}=>format!("C r{r:.3}"), brep::Curve::Ellipse{..}=>"E".into(), brep::Curve::Poly(p)=>format!("P{}", p.len()) };
                        let (a, c) = if fwd { (ed.v0, ed.v1) } else { (ed.v1, ed.v0) };
                        eprintln!("   e{e} {k} {:?} -> {:?}", b.verts[a as usize], b.verts[c as usize]);
                    }
                    eprintln!("   --");
                }
            }
        }
        if let Ok(path) = std::env::var("M2S_DUMP") {
            let mut out = String::new();
            for (t, tri) in m.tris.iter().enumerate() {
                let r = seg.region_of[t] as usize;
                let k = seg.regions[r].surf.kind() as u8;
                let p: Vec<String> = tri.iter().map(|&v| { let q = m.verts[v as usize]; format!("{} {} {}", q.x, q.y, q.z) }).collect();
                out.push_str(&format!("{} {} {} {}\n", r, k, seg.regions[r].tris.len(), p.join(" ")));
            }
            let _ = std::fs::write(path, out);
        }
        if std::env::var("M2S_STATS").is_ok() {
            ff_volume_stats(&m, &seg, &b);
        }
        if let Ok(path) = std::env::var("M2S_FACES") {
            // Soll-Werte je Fläche aus dem Netz: Art, Flächeninhalt, Volumenbeitrag (∫ x·n dA / 3)
            let mut out = String::from("[");
            for (fi, f) in b.faces.iter().enumerate() {
                let (mut a, mut v) = (0.0, 0.0);
                for &t in &seg.regions[f.region as usize].tris {
                    let t = t as usize;
                    a += m.tri_a[t];
                    v += m.tri_centroid(t).dot(m.tri_n[t]) * m.tri_a[t] / 3.0;
                }
                out.push_str(&format!("{}[\"{:?}\",{a},{v}]", if fi > 0 { "," } else { "" }, f.surf.kind()));
            }
            out.push(']');
            let _ = std::fs::write(path, out);
        }
        // Zählen, was wirklich als Einzelfacette übrig bleibt (Runden-Zähler überzählt)
        let tiny_final = seg.regions.iter().filter(|r| r.tris.len() <= 2 && matches!(r.surf, surf::Surf::Plane { .. })).count();
        let demoted = demoted.min(tiny_final);
        if demoted > 0 {
            rep.warnings.push(format!("{demoted} Fläche(n) ohne passende Grundform als Facetten übernommen"));
        }
        rep.max_deviation = rep.max_deviation.max(ck.max_edge_dev).max(ck.max_vertex_dev);
        rep.check_errors.extend(ck.errors.into_iter().take(20));
        rep.faces += b.faces.len();
        rep.edges += b.edges.len();
        rep.bodies += b.solids.len().max(if b.shells.iter().any(|s| !s.closed) { 1 } else { 0 });
        for f in &b.faces {
            *rep.faces_by_kind.entry(f.surf.kind().name_de().to_string()).or_insert(0) += 1;
        }
        if b.shells.iter().any(|s| !s.closed) {
            rep.warnings.push("Offenes Netz: Ausgabe als Flächenmodell statt Volumenkörper".into());
        }
        holes.extend(inspect::find_holes(&m, &seg, preview.tris.len() as u32));
        preview.append_mesh(&m, Some(&seg));
        breps.push(b);
    }
    prm.report(0.9, "STEP schreiben");
    let text = step::write_step(&breps, name, file_name);
    rep.seconds = t0.elapsed().as_secs_f64();
    rep.preview = Some(std::sync::Arc::new(preview));
    rep.holes = inspect::group_holes(&holes);
    prm.report(1.0, "Fertig");
    Ok((text, rep))
}

/// Fehlersuche: Volumenabweichung durch Freiformflächen (Fläche gegen Netz, über die Dreiecke
/// integriert) im Verhältnis zum Netzvolumen.
fn ff_volume_stats(m: &mesh::Mesh, seg: &segment::Segmentation, b: &brep::Brep) {
    let mut vol = 0.0;
    for t in &m.tris {
        let (a, bb, c) = (m.verts[t[0] as usize], m.verts[t[1] as usize], m.verts[t[2] as usize]);
        vol += a.dot(bb.cross(c)) / 6.0;
    }
    let bary = [(1. / 3., 1. / 3.), (0.6, 0.2), (0.2, 0.6), (0.2, 0.2), (0.5, 0.5), (0.5, 0.0), (0.0, 0.5), (0.5, 0.25), (0.25, 0.5)];
    let (mut tot, mut tot_abs) = (0.0, 0.0);
    let mut worst: Vec<(f64, usize)> = Vec::new();
    for (fi, f) in b.faces.iter().enumerate() {
        let surf::Surf::BSpline { id } = f.surf else { continue };
        let bs = bspline::get(id);
        let mut dv = 0.0;
        for &t in &seg.regions[f.region as usize].tris {
            let tr = m.tris[t as usize];
            let (a, bb, c) = (m.verts[tr[0] as usize], m.verts[tr[1] as usize], m.verts[tr[2] as usize]);
            let sgn = if bs.n.dot(m.tri_n[t as usize]) >= 0.0 { 1.0 } else { -1.0 };
            let mut acc = 0.0;
            for &(u, v) in &bary {
                let p = a + (bb - a) * u + (c - a) * v;
                acc += -bs.dist(p);
            }
            dv += m.tri_a[t as usize] * acc / bary.len() as f64 * sgn;
        }
        tot += dv;
        tot_abs += dv.abs();
        worst.push((dv.abs(), fi));
    }
    worst.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap());
    eprintln!("STATS Volumen Netz {:.1} mm³, Freiform-Abweichung {:+.2} mm³ ({:.3} %), Summe |Δ| {:.2} mm³", vol.abs(), tot, 100.0 * tot / vol.abs(), tot_abs);
    for (d, fi) in worst.iter().take(5) {
        eprintln!("  Fläche {fi}: |ΔV| {d:.2} mm³");
    }
}

/// Liest eine STL/3MF-Datei und schreibt die STEP-Datei.
pub fn convert_file(input: &Path, output: &Path, prm: &Params) -> Result<Report, String> {
    prm.report(0.0, "Datei lesen");
    let raws = io::load_file(input)?;
    let name = input.file_stem().map(|s| s.to_string_lossy().to_string()).unwrap_or_else(|| "Teil".into());
    let fname = output.file_name().map(|s| s.to_string_lossy().to_string()).unwrap_or_default();
    let (text, rep) = convert_meshes(&raws, &name, &fname, prm)?;
    std::fs::write(output, text).map_err(|e| format!("STEP-Datei kann nicht geschrieben werden: {e}"))?;
    Ok(rep)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn testkoerper_ohne_pruffehler() {
        for raw in testgen::shapes() {
            let (txt, rep) = convert_meshes(&[raw.clone()], &raw.name, "t.step", &Params::default()).unwrap();
            assert!(rep.check_errors.is_empty(), "{}: {:?}", raw.name, rep.check_errors);
            assert!(txt.contains("MANIFOLD_SOLID_BREP"), "{}", raw.name);
            assert!(rep.max_deviation <= rep.tolerance * 3.0, "{}: {}", raw.name, rep.max_deviation);
        }
    }

    #[test]
    fn welle_wird_korrekt_erkannt() {
        let raw = testgen::shapes().into_iter().find(|m| m.name == "welle").unwrap();
        let (_, rep) = convert_meshes(&[raw], "welle", "w.step", &Params::default()).unwrap();
        let k = |n: &str| rep.faces_by_kind.get(n).copied().unwrap_or(0);
        assert_eq!((k("Ebenen"), k("Zylinder"), k("Kegel"), k("Tori")), (3, 3, 1, 1));
    }
}

#[cfg(test)]
mod preview_tests {
    /// Würfel mit gemischter Umlaufrichtung: Vorschau muss alle Dreiecke nach außen drehen
    #[test]
    fn input_preview_orients_mixed_winding() {
        let v = [[0., 0., 0.], [10., 0., 0.], [10., 10., 0.], [0., 10., 0.], [0., 0., 10.], [10., 0., 10.], [10., 10., 10.], [0., 10., 10.]];
        let f = [[0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7], [0, 1, 5], [0, 5, 4], [1, 2, 6], [1, 6, 5], [2, 3, 7], [2, 7, 6], [3, 0, 4], [3, 4, 7]];
        let mut s = String::from("solid t\n");
        for (i, t) in f.iter().enumerate() {
            let t = if i % 3 == 0 { [t[0], t[2], t[1]] } else { *t };
            s += "facet normal 0 0 0\nouter loop\n";
            for k in t {
                let p: [f64; 3] = v[k];
                s += &format!("vertex {} {} {}\n", p[0], p[1], p[2]);
            }
            s += "endloop\nendfacet\n";
        }
        s += "endsolid t\n";
        let path = std::env::temp_dir().join("m2s_mixed.stl");
        std::fs::write(&path, s).unwrap();
        let p = super::preview_input(&path).unwrap();
        assert_eq!(p.tris.len(), 12);
        for t in &p.tris {
            let a = [t[1][0] - t[0][0], t[1][1] - t[0][1], t[1][2] - t[0][2]];
            let b = [t[2][0] - t[0][0], t[2][1] - t[0][1], t[2][2] - t[0][2]];
            let n = [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
            let c: Vec<f32> = (0..3).map(|k| (t[0][k] + t[1][k] + t[2][k]) / 3.0 - 5.0).collect();
            assert!(n[0] * c[0] + n[1] * c[1] + n[2] * c[2] > 0.0, "Dreieck zeigt nach innen");
        }
    }
}
