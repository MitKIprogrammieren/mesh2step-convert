//! Einlesen von STL (binär/ASCII) und 3MF (inkl. Komponenten/Production-Extension
//! wie in Bambu Studio / OrcaSlicer / PrusaSlicer).

use crate::math::{v3, V3};
use crate::zip::ZipArchive;
use std::collections::HashMap;
use std::path::Path;

/// Ein rohes Dreiecksnetz (ein Objekt der Eingabedatei).
#[derive(Clone, Debug, Default)]
pub struct RawMesh {
    pub name: String,
    pub verts: Vec<V3>,
    pub tris: Vec<[u32; 3]>,
    /// Farbbemalung je Dreieck (Bambu/Orca `paint_color`, Prusa `mmu_segmentation`),
    /// leer wenn die Datei keine Bemalung enthält
    pub paint: Vec<String>,
    /// je Dreieck: Eckpunkte wurden wegen gespiegelter Transformation umgedreht
    pub flipped: Vec<bool>,
    /// Standard-Filament (1-basiert) je Dreieck, leer = unbekannt (1)
    pub extruder: Vec<u8>,
}

/// 3MF mit Zusatzinformationen für das Aufteilen nach Farben.
#[derive(Clone, Debug, Default)]
pub struct Model3mf {
    pub meshes: Vec<RawMesh>,
    /// Filamentfarben (Filament 1 = Index 0)
    pub colors: Vec<[u8; 3]>,
}

pub fn load_file(path: &Path) -> Result<Vec<RawMesh>, String> {
    let data = std::fs::read(path).map_err(|e| format!("Datei kann nicht gelesen werden: {e}"))?;
    let stem = path.file_stem().map(|s| s.to_string_lossy().to_string()).unwrap_or_else(|| "Teil".into());
    let ext = path.extension().map(|e| e.to_string_lossy().to_ascii_lowercase()).unwrap_or_default();
    load_bytes(&data, &ext, &stem)
}

pub fn load_bytes(data: &[u8], ext: &str, name: &str) -> Result<Vec<RawMesh>, String> {
    let is_zip = data.len() > 4 && &data[0..4] == b"PK\x03\x04";
    let meshes = if ext == "3mf" || is_zip {
        load_3mf(data, name)?
    } else {
        vec![load_stl(data, name)?]
    };
    let meshes: Vec<RawMesh> = meshes.into_iter().filter(|m| !m.tris.is_empty()).collect();
    if meshes.is_empty() {
        return Err("Die Datei enthält keine Dreiecke.".into());
    }
    Ok(meshes)
}

// ---------------------------------------------------------------------------
// STL

pub fn load_stl(data: &[u8], name: &str) -> Result<RawMesh, String> {
    if data.len() >= 84 {
        let n = u32::from_le_bytes([data[80], data[81], data[82], data[83]]) as usize;
        if n > 0 && 84 + n.saturating_mul(50) == data.len() {
            return Ok(load_stl_binary(data, n, name));
        }
    }
    let head = String::from_utf8_lossy(&data[..data.len().min(512)]).to_ascii_lowercase();
    if head.trim_start().starts_with("solid") {
        let text = String::from_utf8_lossy(data);
        if text.contains("vertex") {
            return load_stl_ascii(&text, name);
        }
    }
    if data.len() >= 84 {
        // Binär mit falscher Größenangabe: so viele Dreiecke wie vorhanden lesen
        let n = (data.len() - 84) / 50;
        if n > 0 {
            return Ok(load_stl_binary(data, n, name));
        }
    }
    Err("Unbekanntes STL-Format.".into())
}

fn load_stl_binary(data: &[u8], n: usize, name: &str) -> RawMesh {
    let mut verts = Vec::with_capacity(n * 3);
    let mut tris = Vec::with_capacity(n);
    let f = |o: usize| f32::from_le_bytes([data[o], data[o + 1], data[o + 2], data[o + 3]]) as f64;
    for i in 0..n {
        let b = 84 + i * 50 + 12;
        let base = verts.len() as u32;
        for k in 0..3 {
            let o = b + k * 12;
            verts.push(v3(f(o), f(o + 4), f(o + 8)));
        }
        tris.push([base, base + 1, base + 2]);
    }
    RawMesh { name: name.to_string(), verts, tris, ..Default::default() }
}

fn load_stl_ascii(text: &str, name: &str) -> Result<RawMesh, String> {
    let mut verts = Vec::new();
    let mut tris = Vec::new();
    let mut cur: Vec<u32> = Vec::with_capacity(3);
    let mut it = text.split_ascii_whitespace();
    while let Some(tok) = it.next() {
        if tok.eq_ignore_ascii_case("vertex") {
            let mut c = [0.0f64; 3];
            for v in c.iter_mut() {
                *v = it
                    .next()
                    .and_then(|s| s.parse::<f64>().ok())
                    .ok_or("Fehlerhafte Koordinate in ASCII-STL")?;
            }
            cur.push(verts.len() as u32);
            verts.push(v3(c[0], c[1], c[2]));
        } else if tok.eq_ignore_ascii_case("endfacet") || tok.eq_ignore_ascii_case("endloop") {
            if cur.len() >= 3 {
                // Polygone (selten) als Fächer triangulieren
                for k in 1..cur.len() - 1 {
                    tris.push([cur[0], cur[k], cur[k + 1]]);
                }
            }
            cur.clear();
        }
    }
    Ok(RawMesh { name: name.to_string(), verts, tris, ..Default::default() })
}

// ---------------------------------------------------------------------------
// Minimaler XML-Tag-Scanner

pub struct Tag<'a> {
    pub name: &'a str,
    pub attrs: Vec<(&'a str, &'a str)>,
    pub closing: bool,
    pub self_closing: bool,
}

impl<'a> Tag<'a> {
    pub fn attr(&self, key: &str) -> Option<&'a str> {
        self.attrs.iter().find(|(k, _)| *k == key || local(k) == key).map(|(_, v)| *v)
    }
    pub fn local_name(&self) -> &'a str {
        local(self.name)
    }
}

fn local(n: &str) -> &str {
    match n.rfind(':') {
        Some(i) => &n[i + 1..],
        None => n,
    }
}

pub fn scan_tags(xml: &str, mut f: impl FnMut(&Tag)) {
    let b = xml.as_bytes();
    let mut i = 0;
    while i < b.len() {
        if b[i] != b'<' {
            match memchr(b'<', &b[i..]) {
                Some(k) => i += k,
                None => break,
            }
            continue;
        }
        // Kommentare, Deklarationen, CDATA überspringen
        if xml[i..].starts_with("<!--") {
            i = xml[i..].find("-->").map(|k| i + k + 3).unwrap_or(b.len());
            continue;
        }
        if xml[i..].starts_with("<?") || xml[i..].starts_with("<!") {
            i = xml[i..].find('>').map(|k| i + k + 1).unwrap_or(b.len());
            continue;
        }
        let end = match tag_end(b, i) {
            Some(e) => e,
            None => break,
        };
        let inner = &xml[i + 1..end];
        let (closing, inner) = if let Some(s) = inner.strip_prefix('/') { (true, s) } else { (false, inner) };
        let (self_closing, inner) = if let Some(s) = inner.strip_suffix('/') { (true, s) } else { (false, inner) };
        let inner = inner.trim();
        let name_end = inner.find(|c: char| c.is_whitespace()).unwrap_or(inner.len());
        let name = &inner[..name_end];
        let mut attrs = Vec::new();
        let mut rest = &inner[name_end..];
        loop {
            rest = rest.trim_start();
            let eq = match rest.find('=') {
                Some(e) => e,
                None => break,
            };
            let key = rest[..eq].trim();
            let after = rest[eq + 1..].trim_start();
            let q = match after.chars().next() {
                Some(c) if c == '"' || c == '\'' => c,
                _ => break,
            };
            let close = match after[1..].find(q) {
                Some(c) => c + 1,
                None => break,
            };
            attrs.push((key, &after[1..close]));
            rest = &after[close + 1..];
        }
        f(&Tag { name, attrs, closing, self_closing });
        i = end + 1;
    }
}

fn tag_end(b: &[u8], start: usize) -> Option<usize> {
    let mut q: u8 = 0;
    for (k, &c) in b[start..].iter().enumerate() {
        if q != 0 {
            if c == q {
                q = 0;
            }
        } else if c == b'"' || c == b'\'' {
            q = c;
        } else if c == b'>' {
            return Some(start + k);
        }
    }
    None
}

fn memchr(c: u8, s: &[u8]) -> Option<usize> {
    s.iter().position(|&x| x == c)
}

// ---------------------------------------------------------------------------
// 3MF

/// Affine Transformation im 3MF-Zeilenvektor-Format (p' = p * M).
#[derive(Clone, Copy, Debug)]
pub struct Xf {
    pub m: [f64; 12],
}

impl Xf {
    pub const ID: Xf = Xf { m: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0] };

    pub fn parse(s: Option<&str>) -> Xf {
        let s = match s {
            Some(s) => s,
            None => return Xf::ID,
        };
        let v: Vec<f64> = s.split_ascii_whitespace().filter_map(|t| t.parse().ok()).collect();
        if v.len() == 12 {
            let mut m = [0.0; 12];
            m.copy_from_slice(&v);
            Xf { m }
        } else {
            Xf::ID
        }
    }
    pub fn apply(&self, p: V3) -> V3 {
        let m = &self.m;
        v3(
            p.x * m[0] + p.y * m[3] + p.z * m[6] + m[9],
            p.x * m[1] + p.y * m[4] + p.z * m[7] + m[10],
            p.x * m[2] + p.y * m[5] + p.z * m[8] + m[11],
        )
    }
    /// Erst self, dann other.
    pub fn then(&self, o: &Xf) -> Xf {
        let a = |r: usize, c: usize| if r < 3 { self.m[r * 3 + c] } else { self.m[9 + c] };
        let b = |r: usize, c: usize| o.m[r * 3 + c];
        let mut m = [0.0; 12];
        for r in 0..4 {
            for c in 0..3 {
                let mut s = 0.0;
                for k in 0..3 {
                    s += a(r, k) * b(k, c);
                }
                if r == 3 {
                    s += o.m[9 + c];
                }
                m[r * 3 + c] = s;
            }
        }
        Xf { m }
    }
    pub fn det(&self) -> f64 {
        let m = &self.m;
        m[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (m[3] * m[8] - m[5] * m[6]) + m[2] * (m[3] * m[7] - m[4] * m[6])
    }
}

#[derive(Default)]
struct Obj3mf {
    name: String,
    kind: String,
    verts: Vec<V3>,
    tris: Vec<[u32; 3]>,
    paint: Vec<String>,
    comps: Vec<(String, String, Xf)>, // (Pfad, ObjektId, Transformation)
}

fn unit_scale(u: &str) -> f64 {
    match u {
        "micron" => 0.001,
        "centimeter" => 10.0,
        "inch" => 25.4,
        "foot" => 304.8,
        "meter" => 1000.0,
        _ => 1.0,
    }
}

fn parse_model(xml: &str, path: &str, objs: &mut HashMap<(String, String), Obj3mf>, items: &mut Vec<(String, String, Xf)>) -> f64 {
    let mut scale = 1.0;
    let mut cur: Option<(String, Obj3mf)> = None;
    let mut in_build = false;
    scan_tags(xml, |t| {
        match t.local_name() {
            "model" if !t.closing => {
                if let Some(u) = t.attr("unit") {
                    scale = unit_scale(u);
                }
            }
            "object" => {
                if t.closing {
                    if let Some((id, o)) = cur.take() {
                        objs.insert((path.to_string(), id), o);
                    }
                } else {
                    let o = Obj3mf {
                        name: t.attr("name").unwrap_or("").to_string(),
                        kind: t.attr("type").unwrap_or("model").to_string(),
                        ..Default::default()
                    };
                    let id = t.attr("id").unwrap_or("").to_string();
                    if t.self_closing {
                        objs.insert((path.to_string(), id), o);
                    } else {
                        cur = Some((id, o));
                    }
                }
            }
            "vertex" if !t.closing => {
                if let Some((_, o)) = cur.as_mut() {
                    let g = |k: &str| t.attr(k).and_then(|s| s.parse::<f64>().ok()).unwrap_or(0.0);
                    o.verts.push(v3(g("x"), g("y"), g("z")));
                }
            }
            "triangle" if !t.closing => {
                if let Some((_, o)) = cur.as_mut() {
                    let g = |k: &str| t.attr(k).and_then(|s| s.parse::<u32>().ok());
                    if let (Some(a), Some(b), Some(c)) = (g("v1"), g("v2"), g("v3")) {
                        o.tris.push([a, b, c]);
                        let p = t.attr("paint_color").or_else(|| t.attr("mmu_segmentation")).unwrap_or("");
                        if !p.is_empty() {
                            o.paint.resize(o.tris.len() - 1, String::new());
                            o.paint.push(p.to_string());
                        }
                    }
                }
            }
            "component" if !t.closing => {
                if let Some((_, o)) = cur.as_mut() {
                    let p = t.attr("path").map(|s| s.to_string()).unwrap_or_else(|| path.to_string());
                    o.comps.push((p, t.attr("objectid").unwrap_or("").to_string(), Xf::parse(t.attr("transform"))));
                }
            }
            "build" => in_build = !t.closing && !t.self_closing,
            "item" if !t.closing && in_build => {
                let p = t.attr("path").map(|s| s.to_string()).unwrap_or_else(|| path.to_string());
                items.push((p, t.attr("objectid").unwrap_or("").to_string(), Xf::parse(t.attr("transform"))));
            }
            _ => {}
        }
    });
    scale
}

fn norm_path(p: &str) -> String {
    format!("/{}", p.trim_start_matches('/'))
}

pub fn load_3mf(data: &[u8], fallback_name: &str) -> Result<Vec<RawMesh>, String> {
    Ok(load_3mf_full(data, fallback_name)?.meshes)
}

/// Filament je Objekt/Teil aus den Slicer-Projektdaten.
#[derive(Default)]
struct ExtruderInfo {
    /// Bambu/Orca: Objekt-Id -> Filament
    obj: HashMap<String, u8>,
    /// Bambu/Orca: (Objekt-Id, Teil-Id) -> Filament
    part: HashMap<(String, String), u8>,
    /// Prusa: Objekt-Id -> (erstes, letztes Dreieck, Filament)
    ranges: HashMap<String, Vec<(u32, u32, u8)>>,
}

fn parse_extruders(zip: &ZipArchive) -> ExtruderInfo {
    let mut info = ExtruderInfo::default();
    for (k, data) in &zip.files {
        let lk = k.to_ascii_lowercase();
        let bambu = lk.ends_with("metadata/model_settings.config");
        let prusa = lk.ends_with("metadata/slic3r_pe_model.config");
        if !bambu && !prusa {
            continue;
        }
        let xml = String::from_utf8_lossy(data);
        let mut obj: Option<String> = None;
        let mut part: Option<String> = None;
        let mut range: Option<(u32, u32)> = None;
        scan_tags(&xml, |t| match t.local_name() {
            "object" => {
                obj = if t.closing || t.self_closing { None } else { t.attr("id").map(|s| s.to_string()) };
            }
            "part" => {
                part = if t.closing || t.self_closing { None } else { t.attr("id").map(|s| s.to_string()) };
            }
            "volume" => {
                range = if t.closing || t.self_closing {
                    None
                } else {
                    match (t.attr("firstid").and_then(|v| v.parse().ok()), t.attr("lastid").and_then(|v| v.parse().ok())) {
                        (Some(a), Some(b)) => Some((a, b)),
                        _ => None,
                    }
                };
            }
            "metadata" if !t.closing => {
                if t.attr("key") != Some("extruder") {
                    return;
                }
                let e = match t.attr("value").and_then(|v| v.trim().parse::<u32>().ok()) {
                    Some(e) if e > 0 => e.min(255) as u8,
                    _ => return,
                };
                let o = match &obj {
                    Some(o) => o.clone(),
                    None => return,
                };
                if let Some(p) = &part {
                    info.part.insert((o, p.clone()), e);
                } else if let Some((a, b)) = range {
                    info.ranges.entry(o).or_default().push((a, b, e));
                } else {
                    info.obj.insert(o, e);
                }
            }
            _ => {}
        });
    }
    info
}

fn parse_hex_color(s: &str) -> Option<[u8; 3]> {
    let h = s.trim().trim_matches('"').trim_start_matches('#');
    if h.len() < 6 || !h.is_char_boundary(6) {
        return None;
    }
    let p = |i: usize| u8::from_str_radix(&h[i..i + 2], 16).ok();
    Some([p(0)?, p(2)?, p(4)?])
}

/// Filamentfarben aus project_settings.config (Bambu/Orca, JSON) bzw. Slic3r_PE.config (Prusa).
fn parse_colors(zip: &ZipArchive) -> Vec<[u8; 3]> {
    for (k, data) in &zip.files {
        let lk = k.to_ascii_lowercase();
        let text = String::from_utf8_lossy(data);
        if lk.ends_with("metadata/project_settings.config") {
            if let Some(i) = text.find("\"filament_colour\"") {
                let rest = &text[i..];
                if let (Some(a), Some(b)) = (rest.find('['), rest.find(']')) {
                    if a < b {
                        let cols: Vec<[u8; 3]> = rest[a + 1..b].split(',').filter_map(parse_hex_color).collect();
                        if !cols.is_empty() {
                            return cols;
                        }
                    }
                }
            }
        } else if lk.ends_with("metadata/slic3r_pe.config") {
            let mut ext = Vec::new();
            let mut fil = Vec::new();
            for line in text.lines() {
                let l = line.trim_start_matches(';').trim();
                if let Some(v) = l.strip_prefix("extruder_colour =") {
                    ext = v.split(';').map(|c| parse_hex_color(c)).collect::<Vec<_>>();
                } else if let Some(v) = l.strip_prefix("filament_colour =") {
                    fil = v.split(';').map(|c| parse_hex_color(c)).collect::<Vec<_>>();
                }
            }
            let n = ext.len().max(fil.len());
            let cols: Vec<[u8; 3]> = (0..n)
                .filter_map(|i| ext.get(i).copied().flatten().or(fil.get(i).copied().flatten()))
                .collect();
            if !cols.is_empty() {
                return cols;
            }
        }
    }
    Vec::new()
}

pub fn load_3mf_full(data: &[u8], fallback_name: &str) -> Result<Model3mf, String> {
    let zip = ZipArchive::read(data)?;
    // Hauptmodell über _rels/.rels finden
    let mut root = "/3D/3dmodel.model".to_string();
    if let Some(rels) = zip.get("_rels/.rels") {
        let s = String::from_utf8_lossy(rels);
        scan_tags(&s, |t| {
            if t.local_name() == "Relationship" {
                if let (Some(ty), Some(tg)) = (t.attr("Type"), t.attr("Target")) {
                    if ty.ends_with("/3dmodel") {
                        root = norm_path(tg);
                    }
                }
            }
        });
    }
    if zip.get(&root).is_none() {
        // Notfalls irgendeine .model-Datei
        if let Some(k) = zip.files.keys().find(|k| k.to_ascii_lowercase().ends_with(".model")) {
            root = norm_path(k);
        } else {
            return Err("Keine 3D-Modelldaten im 3MF gefunden.".into());
        }
    }
    let mut objs: HashMap<(String, String), Obj3mf> = HashMap::new();
    let mut items = Vec::new();
    let mut scale = 1.0;
    // Alle .model-Dateien einlesen (Production-Extension)
    let mut names: Vec<&String> = zip.files.keys().filter(|k| k.to_ascii_lowercase().ends_with(".model")).collect();
    names.sort();
    for k in names {
        let p = norm_path(k);
        let xml = String::from_utf8_lossy(&zip.files[k]);
        let mut dummy = Vec::new();
        let s = if p.eq_ignore_ascii_case(&root) {
            parse_model(&xml, &p, &mut objs, &mut items)
        } else {
            parse_model(&xml, &p, &mut objs, &mut dummy)
        };
        if p.eq_ignore_ascii_case(&root) {
            scale = s;
        }
    }
    // Pfade case-insensitiv auflösen
    let lookup = |p: &str, id: &str| -> Option<(String, String)> {
        let key = (norm_path(p), id.to_string());
        if objs.contains_key(&key) {
            return Some(key);
        }
        objs.keys().find(|(kp, kid)| kid == id && kp.eq_ignore_ascii_case(&key.0)).cloned()
    };

    let scale_xf = Xf { m: [scale, 0.0, 0.0, 0.0, scale, 0.0, 0.0, 0.0, scale, 0.0, 0.0, 0.0] };
    let einfo = parse_extruders(&zip);
    let colors = parse_colors(&zip);
    let mut out = Vec::new();
    // Falls kein <build> vorhanden: alle Objekte des Hauptmodells
    if items.is_empty() {
        for (p, id) in objs.keys() {
            if p.eq_ignore_ascii_case(&root) {
                items.push((p.clone(), id.clone(), Xf::ID));
            }
        }
        items.sort_by(|a, b| a.1.cmp(&b.1));
    }
    for (n, (p, id, xf)) in items.iter().enumerate() {
        let key = match lookup(p, id) {
            Some(k) => k,
            None => continue,
        };
        let mut mesh = RawMesh::default();
        let xf = xf.then(&scale_xf);
        let ext = einfo.obj.get(id).copied().unwrap_or(1);
        let ctx = FlatCtx { einfo: &einfo, top_id: id.as_str() };
        flatten(&objs, &key, &xf, &mut mesh, 0, &lookup, ext, &ctx);
        if !mesh.paint.iter().any(|p| !p.is_empty()) {
            mesh.paint.clear();
            mesh.flipped.clear();
        }
        let oname = &objs[&key].name;
        mesh.name = if !oname.is_empty() {
            oname.clone()
        } else if items.len() > 1 {
            format!("{fallback_name}_{}", n + 1)
        } else {
            fallback_name.to_string()
        };
        if !mesh.tris.is_empty() {
            out.push(mesh);
        }
    }
    Ok(Model3mf { meshes: out, colors })
}

struct FlatCtx<'a> {
    einfo: &'a ExtruderInfo,
    top_id: &'a str,
}

#[allow(clippy::too_many_arguments)]
fn flatten(
    objs: &HashMap<(String, String), Obj3mf>,
    key: &(String, String),
    xf: &Xf,
    out: &mut RawMesh,
    depth: usize,
    lookup: &dyn Fn(&str, &str) -> Option<(String, String)>,
    ext: u8,
    ctx: &FlatCtx,
) {
    if depth > 16 {
        return;
    }
    let o = match objs.get(key) {
        Some(o) => o,
        None => return,
    };
    if o.kind == "support" || o.kind == "solidsupport" {
        return;
    }
    if !o.tris.is_empty() {
        let base = out.verts.len() as u32;
        let nv = o.verts.len() as u32;
        out.verts.extend(o.verts.iter().map(|&p| xf.apply(p)));
        let flip = xf.det() < 0.0;
        let ranges = if depth == 0 { ctx.einfo.ranges.get(ctx.top_id) } else { None };
        for (i, t) in o.tris.iter().enumerate() {
            if t.iter().all(|&i| i < nv) {
                if flip {
                    out.tris.push([base + t[0], base + t[2], base + t[1]]);
                } else {
                    out.tris.push([base + t[0], base + t[1], base + t[2]]);
                }
                out.paint.push(o.paint.get(i).cloned().unwrap_or_default());
                out.flipped.push(flip);
                let e = ranges
                    .and_then(|r| r.iter().find(|(a, b, _)| (*a as usize) <= i && i <= *b as usize).map(|r| r.2))
                    .unwrap_or(ext);
                out.extruder.push(e);
            }
        }
    }
    for (p, id, cxf) in &o.comps {
        if let Some(k) = lookup(p, id) {
            let x = cxf.then(xf);
            let e = if depth == 0 {
                ctx.einfo.part.get(&(ctx.top_id.to_string(), id.clone())).copied().unwrap_or(ext)
            } else {
                ext
            };
            flatten(objs, &k, &x, out, depth + 1, lookup, e, ctx);
        }
    }
}
