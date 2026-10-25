//! Umwandeln mit externen Engines inklusive Vorbereitung (Reparatur, Ausdünnen) und
//! Nachkontrolle: Die erzeugte STEP-Datei wird mit dem eigenen Leser geöffnet, gegen das
//! Netz vermessen und für Vorschau, Flächenzählung und Bohrungsliste ausgewertet.

use crate::compare::{self, TriMesh};
use crate::engines::{self, EngineId, EngineInfo, ExtOptions};
use crate::io::RawMesh;
use crate::math::V3;
use crate::stepread::{CadModel, CadSurf};
use crate::{inspect, preview_kind, repair, simplify, Params, Preview, Report};
use std::collections::{BTreeMap, HashMap};
use std::path::Path;

/// Binäre STL schreiben.
pub fn stl_bytes(parts: &[(&[V3], &[[u32; 3]])]) -> Vec<u8> {
    let n: usize = parts.iter().map(|p| p.1.len()).sum();
    let mut out = Vec::with_capacity(84 + n * 50);
    let mut head = b"Mesh2STEP".to_vec();
    head.resize(80, 0);
    out.extend_from_slice(&head);
    out.extend_from_slice(&(n as u32).to_le_bytes());
    for (v, t) in parts {
        for tr in t.iter() {
            let (a, b, c) = (v[tr[0] as usize], v[tr[1] as usize], v[tr[2] as usize]);
            let nn = (b - a).cross(c - a);
            let l = nn.norm().max(1e-30);
            for x in [nn.x / l, nn.y / l, nn.z / l, a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z] {
                out.extend_from_slice(&(x as f32).to_le_bytes());
            }
            out.extend_from_slice(&[0, 0]);
        }
    }
    out
}

fn fmt_mm(x: f64) -> String {
    format!("{x:.3}").replace('.', ",")
}

/// Netz für die externe Engine vorbereiten: bei Fehlern reparieren, für FreeCAD sehr dichte
/// Netze ausdünnen. Rückgabe: Netze und Hinweise.
fn prepare(raws: &[RawMesh], id: EngineId, tol: f64, notes: &mut Vec<String>, prm: &Params) -> Vec<(Vec<V3>, Vec<[u32; 3]>)> {
    let mut out = Vec::new();
    let mut repaired = 0usize;
    for raw in raws {
        let a = repair::analyze(raw, false);
        let needs = a.issues.defects() > 0;
        // FreeCAD scheitert an offenen oder nicht-mannigfaltigen Netzen fast immer
        if needs || id == EngineId::FreeCad && !a.issues.watertight() {
            prm.report(0.08, "Netz reparieren");
            let r = repair::repair(raw, &repair::RepairOpts::default());
            repaired += 1;
            out.push((r.verts, r.tris));
        } else {
            out.push((raw.verts.clone(), raw.tris.clone()));
        }
    }
    if repaired > 0 {
        notes.push(if repaired == 1 { "Netz vorher automatisch repariert".to_string() } else { format!("{repaired} Netze vorher automatisch repariert") });
    }
    if id == EngineId::FreeCad {
        let total: usize = out.iter().map(|m| m.1.len()).sum();
        const LIMIT: usize = 60_000;
        if total > LIMIT {
            prm.report(0.12, "Netz ausdünnen");
            let ratio = LIMIT as f64 / total as f64;
            for m in out.iter_mut() {
                let (v, t) = simplify::simplify(&m.0, &m.1, &simplify::SimplifyOpts { ratio, tol: (tol * 0.5).max(0.005) });
                *m = (v, t);
            }
            let after: usize = out.iter().map(|m| m.1.len()).sum();
            notes.push(format!("Netz für FreeCAD ausgedünnt: {} → {} Dreiecke (Abweichung ≤ {} mm)", total, after, fmt_mm((tol * 0.5).max(0.005))));
        }
    }
    out
}

/// Bezeichnung für „Erstellt mit …“
pub fn engine_label(info: &EngineInfo, o: &ExtOptions) -> String {
    let mut s = info.id.name().to_string();
    if let Some(v) = &info.version {
        s.push(' ');
        s.push_str(v);
    }
    match info.id {
        EngineId::Stl2Step => s.push_str(if o.trueform { " · TrueForm" } else { " · Verbatim" }),
        EngineId::Breptile => {
            let m = match o.breptile_mode.as_str() {
                "prismatic" => "Prismatisch",
                "tessellated" => "Facetten",
                _ => "Automatisch",
            };
            s.push_str(" · ");
            s.push_str(m);
        }
        _ => {}
    }
    s
}

/// STL/3MF mit einer externen Engine in STEP umwandeln und das Ergebnis prüfen.
pub fn convert_external(info: &EngineInfo, input: &Path, output: &Path, o: &ExtOptions, prm: &Params) -> Result<Report, String> {
    let t0 = std::time::Instant::now();
    prm.report(0.0, "Datei lesen");
    let raws = crate::io::load_file(input)?;
    let src = TriMesh::from_raws(&raws);
    let st = compare::stats(&src);
    let ext = st.size.x.max(st.size.y).max(st.size.z).max(1e-6);
    let tol = prm.tol.unwrap_or((ext * 2e-4).clamp(0.002, 0.05));
    let mut notes = Vec::new();
    let meshes = prepare(&raws, info.id, tol, &mut notes, prm);

    let tmp = std::env::temp_dir().join(format!("mesh2step-ext-{}-{}", std::process::id(), t0.elapsed().as_nanos() ^ output.to_string_lossy().len() as u128));
    std::fs::create_dir_all(&tmp).map_err(|e| e.to_string())?;
    let stl = tmp.join(format!("{}.stl", input.file_stem().map(|s| s.to_string_lossy().to_string()).unwrap_or_else(|| "teil".into())));
    let parts: Vec<(&[V3], &[[u32; 3]])> = meshes.iter().map(|m| (m.0.as_slice(), m.1.as_slice())).collect();
    std::fs::write(&stl, stl_bytes(&parts)).map_err(|e| format!("Zwischendatei kann nicht geschrieben werden: {e}"))?;

    prm.report(0.15, &format!("{} wandelt um", info.id.name()));
    // Fortschritt während der externen Engine (die selbst keinen meldet) langsam weiterzählen
    let done = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
    let ticker = {
        let done = done.clone();
        let prog = prm.progress.clone();
        let name = info.id.name();
        std::thread::spawn(move || {
            let start = std::time::Instant::now();
            while !done.load(std::sync::atomic::Ordering::Relaxed) {
                std::thread::sleep(std::time::Duration::from_millis(250));
                let s = start.elapsed().as_secs_f32();
                let p = 0.15 + 0.6 * (1.0 - (-s / 20.0).exp());
                if let Some(f) = &prog {
                    f(p, &format!("{name} wandelt um · {} s", s as u32));
                }
            }
        })
    };
    let res = engines::convert_file(info, &stl, output, o, prm.cancel.as_deref());
    done.store(true, std::sync::atomic::Ordering::Relaxed);
    let _ = ticker.join();
    let _ = std::fs::remove_dir_all(&tmp);
    let outcome = res?;
    prm.report(0.8, "Ergebnis prüfen");

    let mut rep = Report { engine: engine_label(info, o), triangles: src.tris.len(), tolerance: tol, ..Default::default() };
    rep.warnings = outcome.warnings;
    notes.extend(outcome.notes);
    rep.notes = notes;
    if outcome.open_shells > 0 {
        rep.check_errors.push(format!("{} Körper nicht geschlossen (offene Fläche statt Volumenkörper)", outcome.open_shells));
    }

    // Ergebnis mit dem eigenen Leser öffnen
    let defl = (ext * 1e-3).clamp(0.005, 0.1);
    match crate::stepread::read_file(output, defl) {
        Ok(cad) => {
            prm.report(0.85, "Abweichung messen");
            fill_from_cad(&mut rep, &cad, &src, st.volume, prm);
            for w in cad.warnings.iter().take(5) {
                rep.notes.push(format!("Beim Nachlesen: {w}"));
            }
        }
        Err(e) => rep.warnings.push(format!("Ergebnis konnte nicht nachgelesen werden: {e}")),
    }
    if outcome.open_shells > 0 {
        rep.volume_delta_pct = None;
    }
    rep.bodies = rep.bodies.max(raws.len().min(1));
    rep.seconds = t0.elapsed().as_secs_f64();
    prm.report(1.0, "Fertig");
    Ok(rep)
}

fn kind_of(s: &CadSurf) -> u8 {
    match s {
        CadSurf::Plane(_) => preview_kind::PLANE,
        CadSurf::Cyl(..) => preview_kind::CYL,
        CadSurf::Cone(..) => preview_kind::CONE,
        CadSurf::Sphere(..) => preview_kind::SPHERE,
        CadSurf::Torus(..) => preview_kind::TORUS,
        CadSurf::Nurbs(_) => preview_kind::BSPLINE,
        CadSurf::Other(_) => preview_kind::FACET,
    }
}

/// Ebene Dreiecksflächen (aus Facetten-STEP) als „Facette“ zählen statt als Ebene.
fn is_facet(cad: &CadModel, fi: usize) -> bool {
    let f = &cad.faces[fi];
    matches!(f.surf, CadSurf::Plane(_)) && f.last >= f.first && f.last - f.first < 1
}

/// Bericht aus der nachgelesenen STEP-Datei ergänzen.
pub fn fill_from_cad(rep: &mut Report, cad: &CadModel, src: &TriMesh, src_volume: f64, prm: &Params) {
    rep.faces = cad.faces.len();
    let mut by: BTreeMap<String, usize> = BTreeMap::new();
    let mut kind = vec![preview_kind::FACET; cad.tris.len()];
    let mut face_of = vec![u32::MAX; cad.tris.len()];
    for (fi, f) in cad.faces.iter().enumerate() {
        let facet = is_facet(cad, fi);
        let name = if facet {
            "Facetten"
        } else {
            match f.surf {
                CadSurf::Plane(_) => "Ebenen",
                CadSurf::Cyl(..) => "Zylinder",
                CadSurf::Cone(..) => "Kegel",
                CadSurf::Sphere(..) => "Kugeln",
                CadSurf::Torus(..) => "Tori",
                CadSurf::Nurbs(_) => "Freiformflächen",
                CadSurf::Other(_) => "Sonstige",
            }
        };
        *by.entry(name.to_string()).or_default() += 1;
        let k = if facet { preview_kind::FACET } else { kind_of(&f.surf) };
        for t in f.first..=f.last.min(cad.tris.len().saturating_sub(1)) {
            if f.last < f.first {
                break;
            }
            kind[t] = k;
            face_of[t] = fi as u32;
        }
    }
    rep.faces_by_kind = by;

    // Vorschau mit Flächengrenzen
    let tris: Vec<[[f32; 3]; 3]> = cad.tris.iter().map(|t| [0, 1, 2].map(|k| { let v = cad.verts[t[k] as usize]; [v.x as f32, v.y as f32, v.z as f32] })).collect();
    let q = |v: V3| ((v.x * 1e4).round() as i64, (v.y * 1e4).round() as i64, (v.z * 1e4).round() as i64);
    let mut emap: HashMap<((i64, i64, i64), (i64, i64, i64)), u32> = HashMap::new();
    let mut edges = Vec::new();
    let mut n_edges = 0usize;
    for (ti, t) in cad.tris.iter().enumerate() {
        for k in 0..3 {
            let (a, b) = (cad.verts[t[k] as usize], cad.verts[t[(k + 1) % 3] as usize]);
            let (ka, kb) = (q(a), q(b));
            let key = if ka <= kb { (ka, kb) } else { (kb, ka) };
            match emap.remove(&key) {
                Some(o) => {
                    if face_of[o as usize] != face_of[ti] {
                        edges.push(([[a.x as f32, a.y as f32, a.z as f32], [b.x as f32, b.y as f32, b.z as f32]], o, ti as u32));
                        n_edges += 1;
                    }
                }
                None => {
                    emap.insert(key, ti as u32);
                }
            }
        }
    }
    rep.edges = n_edges;
    rep.preview = Some(std::sync::Arc::new(Preview { tris, kind, edges, two_sided: true, ..Default::default() }));

    // Abweichung: Punkte der STEP-Flächen gegen das Originalnetz
    let cadm = TriMesh::from_cad(cad);
    let grid = compare::Grid::new(src);
    if let Some(d) = compare::deviation(&cadm, &grid, &|p| {
        prm.report(0.85 + 0.12 * p, "Abweichung messen");
        true
    }) {
        rep.max_deviation = d.max_pos.abs().max(d.max_neg.abs());
    }
    let cst = compare::stats(&cadm);
    if src_volume.abs() > 1e-9 && cst.volume.abs() > 1e-9 {
        rep.volume_delta_pct = Some(100.0 * (cst.volume.abs() - src_volume.abs()) / src_volume.abs());
    }
    rep.bodies = count_bodies(cad).max(1);

    // Bohrungen aus echten Zylinderflächen
    let mut holes = Vec::new();
    for c in compare::cylinders(cad) {
        if !c.hole || c.coverage < 300.0 {
            continue;
        }
        let through = !has_floor(cad, &c);
        holes.push(inspect::Hole { diameter: 2.0 * c.r, depth: c.h1 - c.h0, through, tris: c.cad_tris.clone() });
    }
    rep.holes = inspect::group_holes(&holes);
}

/// Boden an einem Bohrungsende (Sackloch): ein Dreieck schneidet die Achse kurz vor oder
/// hinter einem Ende. Bei durchgehenden Bohrungen ist dort nichts.
fn has_floor(cad: &CadModel, c: &compare::Cylinder) -> bool {
    let eps = (c.r * 0.05).max(0.01);
    let reach = c.r * 1.2 + eps;
    for t in &cad.tris {
        let (a, b, cc) = (cad.verts[t[0] as usize], cad.verts[t[1] as usize], cad.verts[t[2] as usize]);
        // Achse o + axis*h gegen das Dreieck (Möller-Trumbore)
        let e1 = b - a;
        let e2 = cc - a;
        let p = c.axis.cross(e2);
        let det = e1.dot(p);
        if det.abs() < 1e-12 {
            continue;
        }
        let inv = 1.0 / det;
        let s = c.o - a;
        let u = s.dot(p) * inv;
        if !(-1e-9..=1.0 + 1e-9).contains(&u) {
            continue;
        }
        let q = s.cross(e1);
        let v = c.axis.dot(q) * inv;
        if v < -1e-9 || u + v > 1.0 + 1e-9 {
            continue;
        }
        let h = e2.dot(q) * inv;
        if (h > c.h0 - reach && h < c.h0 + eps) || (h > c.h1 - eps && h < c.h1 + reach) {
            return true;
        }
    }
    false
}

/// Zusammenhängende Teile über gemeinsame Eckpunkte (gerundet) zählen.
fn count_bodies(cad: &CadModel) -> usize {
    let n = cad.verts.len();
    let mut parent: Vec<usize> = (0..n).collect();
    fn find(p: &mut [usize], mut x: usize) -> usize {
        while p[x] != x {
            p[x] = p[p[x]];
            x = p[x];
        }
        x
    }
    let q = |v: V3| ((v.x * 1e4).round() as i64, (v.y * 1e4).round() as i64, (v.z * 1e4).round() as i64);
    let mut first: HashMap<(i64, i64, i64), usize> = HashMap::new();
    for (i, v) in cad.verts.iter().enumerate() {
        let k = q(*v);
        match first.get(&k) {
            Some(&j) => {
                let (a, b) = (find(&mut parent, i), find(&mut parent, j));
                parent[a] = b;
            }
            None => {
                first.insert(k, i);
            }
        }
    }
    for t in &cad.tris {
        let a = find(&mut parent, t[0] as usize);
        for k in 1..3 {
            let b = find(&mut parent, t[k] as usize);
            if a != b {
                parent[b] = a;
            }
        }
    }
    let mut roots = std::collections::HashSet::new();
    for t in &cad.tris {
        roots.insert(find(&mut parent, t[0] as usize));
    }
    roots.len()
}

/// Volumen-Abweichung einer fertigen STEP-Datei gegen das Eingangsnetz in Prozent.
pub fn volume_delta(input: &Path, step: &Path) -> Option<f64> {
    let raws = crate::io::load_file(input).ok()?;
    let src = TriMesh::from_raws(&raws);
    let st = compare::stats(&src);
    let ext = st.size.x.max(st.size.y).max(st.size.z).max(1e-6);
    let cad = crate::stepread::read_file(step, (ext * 1e-3).clamp(0.005, 0.1)).ok()?;
    let c = compare::stats(&TriMesh::from_cad(&cad));
    if st.volume.abs() < 1e-9 || c.volume.abs() < 1e-9 {
        return None;
    }
    Some(100.0 * (c.volume.abs() - st.volume.abs()) / st.volume.abs())
}

/// STEP-Datei für den Maßvergleich mit einer externen Engine lesen (über eine STL).
pub fn read_step_external(info: &EngineInfo, step: &Path, defl: f64, cancel: Option<&std::sync::atomic::AtomicBool>) -> Result<RawMesh, String> {
    let tmp = std::env::temp_dir().join(format!("mesh2step-read-{}-{}", std::process::id(), step.to_string_lossy().len()));
    std::fs::create_dir_all(&tmp).map_err(|e| e.to_string())?;
    let out = tmp.join("cad.stl");
    let r = engines::step_to_stl(info, step, &out, defl, cancel).and_then(|_| {
        let data = std::fs::read(&out).map_err(|e| e.to_string())?;
        crate::io::load_stl(&data, "CAD")
    });
    let _ = std::fs::remove_dir_all(&tmp);
    r
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn stl_rundweg() {
        let raw = crate::testgen::shapes().into_iter().next().unwrap();
        let b = stl_bytes(&[(&raw.verts, &raw.tris)]);
        let back = crate::io::load_stl(&b, "x").unwrap();
        assert_eq!(back.tris.len(), raw.tris.len());
    }
    #[test]
    fn nachlesen_eigener_step() {
        // eigenen STEP erzeugen und wie ein externes Ergebnis auswerten
        let raw = crate::testgen::shapes().into_iter().next().unwrap();
        let (txt, _) = crate::convert_meshes(&[raw.clone()], &raw.name, "t.step", &Params::default()).unwrap();
        let cad = crate::stepread::read_bytes(txt.as_bytes(), 0.01).unwrap();
        let src = TriMesh::from_raws(&[raw]);
        let st = compare::stats(&src);
        let mut rep = Report::default();
        fill_from_cad(&mut rep, &cad, &src, st.volume, &Params::default());
        assert!(rep.faces > 0);
        assert!(rep.max_deviation < 0.05, "{}", rep.max_deviation);
        assert!(rep.volume_delta_pct.unwrap().abs() < 1.0);
        assert_eq!(rep.bodies, 1);
    }
}
