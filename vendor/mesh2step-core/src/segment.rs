//! Zerlegung des Netzes in Regionen, die jeweils einer analytischen Fläche entsprechen.

use crate::math::{angle, V3};
use crate::mesh::{Mesh, NONE};
use crate::surf::{self, Kind, Surf};
use std::collections::{HashMap, VecDeque};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    /// Jedes Dreieck wird eine Fläche
    Facets,
    /// Ebene Bereiche werden zusammengefasst
    Planes,
    /// Zusätzlich Zylinder, Kegel, Kugeln, Tori erkennen
    Full,
}

#[derive(Clone)]
pub struct Params {
    pub mode: Mode,
    /// Abstandstoleranz in mm; None = automatisch aus Bauteilgröße
    pub tol: Option<f64>,
    /// Maximaler Winkel zwischen Nachbarfacetten einer gekrümmten Fläche (Grad)
    pub smooth_angle_deg: f64,
    /// Fortschritts-Callback (0..1)
    pub progress: Option<std::sync::Arc<dyn Fn(f32, &str) + Send + Sync>>,
    /// Abbrechen: wird beim nächsten Fortschrittsschritt geprüft (bricht per Panik ab, die
    /// der Aufrufer mit catch_unwind abfängt)
    pub cancel: Option<std::sync::Arc<std::sync::atomic::AtomicBool>>,
}

/// Nutzlast der Abbruch-Panik
pub struct Cancelled;

impl Default for Params {
    fn default() -> Self {
        Params { mode: Mode::Full, tol: None, smooth_angle_deg: 40.0, progress: None, cancel: None }
    }
}

impl Params {
    /// Toleranz für ein Netz: vorgegeben oder aus der Netzgenauigkeit geschätzt.
    pub fn tolerance(&self, m: &Mesh) -> f64 {
        match self.tol {
            Some(t) if t > 0.0 => t,
            _ => auto_tol(m),
        }
    }
    pub fn report(&self, p: f32, msg: &str) {
        if let Some(c) = &self.cancel {
            if c.load(std::sync::atomic::Ordering::Relaxed) {
                std::panic::panic_any(Cancelled);
            }
        }
        if let Some(cb) = &self.progress {
            cb(p, msg);
        }
    }
}

#[derive(Clone, Debug)]
pub struct Region {
    pub tris: Vec<u32>,
    pub surf: Surf,
}

pub struct Segmentation {
    pub region_of: Vec<u32>,
    pub regions: Vec<Region>,
}

struct Ctx<'a> {
    m: &'a Mesh,
    tol: f64,
    cos_smooth: f64,
    smooth: f64,
    min_area: f64,
}

impl<'a> Ctx<'a> {
    fn p(&self, v: u32) -> V3 {
        self.m.verts[v as usize]
    }

    /// Prüft, ob Dreiecke/Punkte zur Fläche passen. `sign` hält die Orientierung fest.
    fn verify(&self, s: &Surf, verts: &[u32], tris: &[u32], sign: &mut f64) -> bool {
        for &v in verts {
            if s.dist(self.p(v)).abs() > self.tol {
                return false;
            }
        }
        if matches!(s, Surf::Plane { .. }) {
            return true;
        }
        // Sehnenfehler der Facetten: Bei einer echt vernetzten Rundung weichen die Facetten-
        // mitten alle ähnlich stark von der Fläche ab (gleichmäßige Unterteilung). Liegen nur
        // die Ecken auf der Fläche, einzelne große Facetten aber weit daneben (z. B. die
        // geraden Seiten eines Sechskants mit gerundeten Ecken, als Kegel „erkannt“), passt
        // die Grundform nicht.
        if !matches!(s, Surf::BSpline { .. }) {
            let lim = 3.0 * self.tol;
            let mut devs: Vec<f64> = Vec::new();
            let mut worst = 0.0f64;
            for &t in tris {
                let t = t as usize;
                if self.m.tri_a[t] < self.min_area {
                    continue;
                }
                let d = s.dist(self.m.tri_centroid(t)).abs();
                worst = worst.max(d);
                devs.push(d);
            }
            if worst > lim && devs.len() >= 4 {
                let mid = devs.len() / 2;
                let (_, med, _) = devs.select_nth_unstable_by(mid, |a, b| a.partial_cmp(b).unwrap());
                if worst > 6.0 * med.max(self.tol * 0.25) {
                    return false;
                }
            }
        }
        for &t in tris {
            let t = t as usize;
            if self.m.tri_a[t] < self.min_area {
                continue;
            }
            let [a, b, c] = self.m.tris[t];
            let na = s.normal(self.p(a));
            let nb = s.normal(self.p(b));
            let nc = s.normal(self.p(c));
            if na.dot(nb) < self.cos_smooth || nb.dot(nc) < self.cos_smooth || na.dot(nc) < self.cos_smooth {
                return false;
            }
            let nt = self.m.tri_n[t];
            let d = nt.dot(s.normal(self.m.tri_centroid(t)));
            if d.abs() < self.cos_smooth {
                return false;
            }
            if *sign == 0.0 {
                *sign = d.signum();
            } else if d.signum() != *sign {
                return false;
            }
        }
        true
    }
}

/// Wählt die Toleranz automatisch aus dem Rauschen des Netzes.
/// Maß: Abstand der gegenüberliegenden Ecke eines Nachbardreiecks zur Dreiecksebene.
/// In CAD-Exporten sind viele Nachbarpaare exakt eben (ebene Flächen, Streifen von
/// Zylindern) – das 20%-Quantil ist dann ~0. Bei Scans entspricht es dem Rauschen.
pub fn auto_tol(m: &Mesh) -> f64 {
    let diag = m.diag();
    let floor = (diag * 2e-6).max(1e-4);
    let loose = (diag * 1e-3).max(floor);
    let mut ds: Vec<f64> = Vec::with_capacity(m.twin.len() / 2);
    let lim = 30f64.to_radians().cos();
    let min_area = (diag * 1e-5).powi(2);
    for h in 0..m.twin.len() {
        let tw = m.twin[h];
        if tw == NONE || (tw as usize) < h {
            continue;
        }
        let (t1, t2) = (h / 3, (tw / 3) as usize);
        if m.tri_a[t1] < min_area || m.tri_a[t2] < min_area || m.tri_n[t1].dot(m.tri_n[t2]) < lim {
            continue;
        }
        let opp = m.tris[t2][((tw % 3 + 2) % 3) as usize];
        let a = m.verts[m.tris[t1][0] as usize];
        ds.push((m.verts[opp as usize] - a).dot(m.tri_n[t1]).abs());
    }
    if ds.len() < 8 {
        return floor;
    }
    // Exakt ebene Nachbarpaare => CAD-Export: engste Toleranz
    let exact = ds.iter().filter(|&&d| d < diag * 1e-7).count();
    if exact * 100 >= ds.len() {
        return floor;
    }
    ds.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let q = ds[ds.len() / 5];
    (q * 8.0).clamp(floor, loose)
}

pub fn segment(m: &Mesh, prm: &Params, tol: f64) -> Segmentation {
    let diag = m.diag();
    let smooth = prm.smooth_angle_deg.clamp(5.0, 75.0).to_radians();
    let ctx = Ctx { m, tol, cos_smooth: smooth.cos(), smooth, min_area: (diag * 1e-6).powi(2) };
    let nt = m.tris.len();

    if prm.mode == Mode::Facets {
        let regions = (0..nt)
            .map(|t| {
                let n = if m.tri_n[t] == V3::ZERO { V3::Z } else { m.tri_n[t] };
                Region { tris: vec![t as u32], surf: Surf::Plane { o: m.tri_centroid(t), n } }
            })
            .collect();
        return Segmentation { region_of: (0..nt as u32).collect(), regions };
    }

    prm.report(0.05, "Ebene Bereiche suchen");
    let mut seg = planar_pieces(&ctx);
    dbg_dump(m, &seg, "0");
    if prm.mode == Mode::Full {
        prm.report(0.2, "Zylinder, Kegel, Kugeln und Tori erkennen");
        seg = curved_regions(&ctx, seg, prm);
        dbg_dump(m, &seg, "1");
    }
    prm.report(0.6, "Flächen zusammenführen");
    let tm = std::time::Instant::now();
    merge_regions(&ctx, &mut seg);
    if std::env::var("M2S_DEBUG").is_ok() {
        eprintln!("merge: {:.1}s", tm.elapsed().as_secs_f64());
    }
    if prm.mode == Mode::Full {
        dbg_dump(m, &seg, "2");
        absorb_triangles(&ctx, &mut seg);
        merge_regions(&ctx, &mut seg);
        regularize(&ctx, &mut seg);
        dbg_dump(m, &seg, "3");
        prm.report(0.7, "Freiformflächen anpassen");
        if std::env::var("M2S_DEBUG").is_ok() {
            let small = |seg: &Segmentation| seg.regions.iter().filter(|r| matches!(r.surf, Surf::Plane { .. }) && r.tris.len() <= 2).map(|r| r.tris.len()).sum::<usize>();
            eprintln!("STAT vor Freiform: {} Dreiecke in Kleinst-Ebenen", small(&seg));
            freeform_regions(&ctx, &mut seg);
            eprintln!("STAT nach Freiform: {} Dreiecke in Kleinst-Ebenen", small(&seg));
        } else {
            freeform_regions(&ctx, &mut seg);
        }
    }
    split_components(&ctx, &mut seg);
    split_boundaryless(&ctx, &mut seg);
    seg
}

/// Debug: Segmentierung einer Zwischenstufe als Text ausgeben (M2S_DUMPSTAGE=<präfix>)
fn dbg_dump(m: &Mesh, seg: &Segmentation, stage: &str) {
    if let Ok(pre) = std::env::var("M2S_DUMPSTAGE") {
        let mut out = String::new();
        for (t, tri) in m.tris.iter().enumerate() {
            let r = seg.region_of[t] as usize;
            let p: Vec<String> = tri.iter().map(|&v| { let q = m.verts[v as usize]; format!("{} {} {}", q.x, q.y, q.z) }).collect();
            out.push_str(&format!("{} {} {} {}\n", r, seg.regions[r].surf.kind() as u8, seg.regions[r].tris.len(), p.join(" ")));
        }
        let _ = std::fs::write(format!("{pre}{stage}.txt"), out);
    }
}

// ---------------------------------------------------------------------------
// Phase 1: ebene Stücke

fn planar_pieces(ctx: &Ctx) -> Segmentation {
    let m = ctx.m;
    let nt = m.tris.len();
    let mut order: Vec<u32> = (0..nt as u32).collect();
    order.sort_by(|&a, &b| m.tri_a[b as usize].partial_cmp(&m.tri_a[a as usize]).unwrap());
    let mut region_of = vec![NONE; nt];
    let mut regions: Vec<Region> = Vec::new();
    let cos_lim = 20f64.to_radians().cos();
    let mut vstamp = vec![NONE; m.verts.len()];
    let mut queue = VecDeque::new();
    for &seed in &order {
        if region_of[seed as usize] != NONE {
            continue;
        }
        let rid = regions.len() as u32;
        let s = seed as usize;
        let mut n = m.tri_n[s];
        if n == V3::ZERO {
            n = V3::Z;
        }
        let mut plane = Surf::Plane { o: m.tri_centroid(s), n };
        let mut tris = vec![seed];
        let mut verts: Vec<u32> = Vec::new();
        for &v in &m.tris[s] {
            if vstamp[v as usize] != rid {
                vstamp[v as usize] = rid;
                verts.push(v);
            }
        }
        region_of[s] = rid;
        let mut next_fit = 8usize;
        queue.clear();
        queue.push_back(seed);
        while let Some(t) = queue.pop_front() {
            for k in 0..3 {
                let tw = m.twin[(3 * t + k) as usize];
                if tw == NONE {
                    continue;
                }
                let u = (tw / 3) as usize;
                if region_of[u] != NONE {
                    continue;
                }
                let (pn, po) = match plane {
                    Surf::Plane { n, o } => (n, o),
                    _ => unreachable!(),
                };
                // Erlaubte Normalenabweichung hängt von der Dreieckshöhe ab:
                // kleine/schmale Dreiecke dürfen mehr streuen, Facetten von Rundungen nicht.
                let lim = {
                    let [a, b, c] = m.tris[u];
                    let (a, b, c) = (m.verts[a as usize], m.verts[b as usize], m.verts[c as usize]);
                    let le = (b - a).norm().max((c - b).norm()).max((a - c).norm()).max(1e-300);
                    let h = 2.0 * m.tri_a[u] / le;
                    (2.0 * ctx.tol / h.max(1e-300)).atan().max(1f64.to_radians())
                };
                let d = m.tri_n[u].dot(pn);
                if d < cos_lim || (m.tri_a[u] > ctx.min_area && d < lim.cos()) {
                    continue;
                }
                if m.tris[u].iter().any(|&v| (m.verts[v as usize] - po).dot(pn).abs() > ctx.tol) {
                    continue;
                }
                region_of[u] = rid;
                tris.push(u as u32);
                for &v in &m.tris[u] {
                    if vstamp[v as usize] != rid {
                        vstamp[v as usize] = rid;
                        verts.push(v);
                    }
                }
                queue.push_back(u as u32);
                if verts.len() >= next_fit {
                    next_fit = verts.len() * 2;
                    let pts: Vec<V3> = verts.iter().map(|&v| m.verts[v as usize]).collect();
                    if let Some(Surf::Plane { o, n: nn }) = surf::fit_plane(&pts) {
                        let nn = if nn.dot(pn) < 0.0 { -nn } else { nn };
                        let cand = Surf::Plane { o, n: nn };
                        if pts.iter().all(|&p| cand.dist(p).abs() <= ctx.tol) {
                            plane = cand;
                        }
                    }
                }
            }
        }
        regions.push(Region { tris, surf: plane });
    }
    Segmentation { region_of, regions }
}

// ---------------------------------------------------------------------------
// Phase 2: gekrümmte Flächen durch Region-Growing über ebene Stücke

struct Piece {
    tris: Vec<u32>,
    verts: Vec<u32>,
    area: f64,
}

fn region_adjacency(m: &Mesh, region_of: &[u32]) -> HashMap<(u32, u32), f64> {
    let mut adj: HashMap<(u32, u32), f64> = HashMap::new();
    for h in 0..m.twin.len() {
        let tw = m.twin[h];
        if tw == NONE {
            continue;
        }
        let a = region_of[h / 3];
        let b = region_of[(tw / 3) as usize];
        if a == b || a == NONE || b == NONE {
            continue;
        }
        let dih = angle(m.tri_n[h / 3], m.tri_n[(tw / 3) as usize]);
        let e = adj.entry((a, b)).or_insert(0.0);
        if dih > *e {
            *e = dih;
        }
    }
    adj
}

fn unique_verts(m: &Mesh, tris: &[u32]) -> Vec<u32> {
    let mut v: Vec<u32> = tris.iter().flat_map(|&t| m.tris[t as usize]).collect();
    v.sort_unstable();
    v.dedup();
    v
}

struct Grow {
    pieces: Vec<u32>,
    tris: Vec<u32>,
    verts: Vec<u32>,
    surf: Option<Surf>,
    sign: f64,
}

fn sample<T: Copy>(v: &[T], max: usize) -> Vec<T> {
    if v.len() <= max {
        return v.to_vec();
    }
    let step = v.len() as f64 / max as f64;
    (0..max).map(|i| v[(i as f64 * step) as usize]).collect()
}

pub static FIT_STATS: [std::sync::atomic::AtomicU64; 10] = [const { std::sync::atomic::AtomicU64::new(0) }; 10];
/// Versucht alle gekrümmten Flächentypen der Reihe nach.
fn fit_any(ctx: &Ctx, verts: &[u32], tris: &[u32], init: Option<&Surf>) -> Option<(Surf, f64)> {
    let m = ctx.m;
    let pts: Vec<V3> = sample(verts, 400).iter().map(|&v| m.verts[v as usize]).collect();
    let st = sample(tris, 400);
    let nrm3: Vec<(V3, f64, V3)> =
        st.iter().map(|&t| (m.tri_n[t as usize], m.tri_a[t as usize], m.tri_centroid(t as usize))).collect();
    // Torus-Startwerte: Eckpunkte mit gemittelten Normalen (genauer als Facettennormalen)
    let smp: Vec<(V3, V3)> = {
        let mut acc: HashMap<u32, V3> = HashMap::with_capacity(verts.len());
        for &t in tris {
            let w = m.tri_n[t as usize] * m.tri_a[t as usize];
            for &v in &m.tris[t as usize] {
                *acc.entry(v).or_insert(V3::ZERO) += w;
            }
        }
        let mut v: Vec<(V3, V3)> = sample(verts, 400)
            .iter()
            .filter_map(|&vi| acc.get(&vi).map(|n| (m.verts[vi as usize], n.normalized())))
            .collect();
        v.extend(
            st.iter()
                .filter(|&&t| m.tri_a[t as usize] >= ctx.min_area)
                .take(100)
                .map(|&t| (m.tri_centroid(t as usize), m.tri_n[t as usize])),
        );
        v
    };
    let diag = m.diag();
    let mut order = vec![Kind::Cyl, Kind::Cone, Kind::Sphere, Kind::Torus];
    if let Some(s) = init {
        let k = s.kind();
        order.retain(|&x| x != k);
        // bisherigen Typ zuerst nur, wenn er einfacher ist als die Alternativen
        let pos = match k {
            Kind::Cyl => 0,
            Kind::Cone => 1,
            Kind::Sphere => 2,
            _ => 3,
        };
        order.insert(pos.min(order.len()), k);
    }
    let mut best_rms = f64::MAX;
    for k in order {
        // Torus nur versuchen, wenn eine einfachere Fläche schon nahe dran war (teurer Fit)
        // Torus (teuer) nur, wenn eine einfachere Rotationsfläche schon nahe dran war
        if k == Kind::Torus && init.map_or(true, |s| s.kind() != Kind::Torus) {
            let mut mn = V3 { x: f64::MAX, y: f64::MAX, z: f64::MAX };
            let mut mx = -mn;
            for p in &pts {
                mn = mn.min(*p);
                mx = mx.max(*p);
            }
            if best_rms < f64::MAX && best_rms > 0.05 * (mx - mn).norm() {
                continue;
            }
        }
        let tfit = std::time::Instant::now();
        let ki = k as usize;
        let cand = match k {
            Kind::Cyl => surf::fit_cylinder(&pts, &nrm3, init),
            Kind::Sphere => surf::fit_sphere(&pts),
            Kind::Cone => surf::fit_cone(&pts, &nrm3, init),
            Kind::Torus => surf::fit_torus(&pts, &smp, init),
            Kind::Plane | Kind::BSpline => None,
        };
        FIT_STATS[ki].fetch_add(tfit.elapsed().as_micros() as u64, std::sync::atomic::Ordering::Relaxed);
        FIT_STATS[ki + 5].fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        let cand = match cand {
            Some(c) => c,
            None => continue,
        };
        // Entartete Flächen (riesige Radien = eigentlich eben) verwerfen
        let too_big = match cand {
            Surf::Cyl { r, .. } | Surf::Sphere { r, .. } => r > 50.0 * diag,
            Surf::Torus { rmaj, .. } => rmaj > 50.0 * diag,
            Surf::Cone { ang, .. } => ang < 0.002,
            _ => false,
        };
        if too_big {
            continue;
        }
        best_rms = best_rms.min(surf::rms_of(&cand, &pts));
        let mut sign = 0.0;
        if ctx.verify(&cand, verts, tris, &mut sign) {
            return Some((cand, sign));
        }
    }
    None
}

fn curved_regions(ctx: &Ctx, seg: Segmentation, prm: &Params) -> Segmentation {
    let m = ctx.m;
    let np = seg.regions.len();
    let pieces: Vec<Piece> = seg
        .regions
        .iter()
        .map(|r| Piece {
            verts: unique_verts(m, &r.tris),
            area: r.tris.iter().map(|&t| m.tri_a[t as usize]).sum(),
            tris: r.tris.clone(),
        })
        .collect();
    let adj_map = region_adjacency(m, &seg.region_of);
    let mut nbrs: Vec<Vec<u32>> = vec![Vec::new(); np];
    for (&(a, b), &dih) in &adj_map {
        if dih <= ctx.smooth && dih > 1e-9 {
            nbrs[a as usize].push(b);
        }
    }
    for n in nbrs.iter_mut() {
        n.sort_unstable();
        n.dedup();
    }
    let mut order: Vec<u32> = (0..np as u32).filter(|&p| !nbrs[p as usize].is_empty()).collect();
    order.sort_by(|&a, &b| pieces[a as usize].area.partial_cmp(&pieces[b as usize].area).unwrap());

    let mut owner = vec![NONE; np]; // gekrümmte Region, der das Stück gehört
    let mut tried = vec![false; np];
    let mut curved: Vec<Region> = Vec::new();
    let total = order.len().max(1);
    let mut rej_stamp: Vec<u64> = vec![u64::MAX; np];
    let mut vmark = vec![NONE; m.verts.len()];
    let total_area = area_of(m, &(0..m.tris.len() as u32).collect::<Vec<_>>()).max(1e-300);
    let mut n_rejected = 0usize;
    let mut curved_pieces: Vec<Vec<u32>> = Vec::new();
    let mut n_unclean = 0usize;
    let mut n_seeds = 0usize;
    let t0 = std::time::Instant::now();
    let mut work: u64 = 0;

    // Zweiter Durchgang mit den größten Stücken zuerst: grob vernetzte Rundungen (lange
    // Streifendreiecke) werden sonst von den kleinen Eckdreiecken daneben verschluckt
    let mut seq: Vec<(u8, u32)> = order.iter().map(|&p| (0u8, p)).collect();
    seq.extend(order.iter().rev().map(|&p| (1u8, p)));
    let mut pass = 0u8;
    for (oi, &(ps, seed)) in seq.iter().enumerate() {
        if ps != pass {
            pass = ps;
            tried.iter_mut().for_each(|t| *t = false);
            n_seeds = 0;
        }
        if oi % 64 == 0 {
            prm.report(0.2 + 0.4 * ((oi % total) as f32 / total as f32), "Gekrümmte Flächen erkennen");
        }
        if owner[seed as usize] != NONE || tried[seed as usize] {
            continue;
        }
        tried[seed as usize] = true;
        n_seeds += 1;
        // Freiform-Erkennung: viele Startversuche, kaum Treffer
        if n_seeds > 3000 && curved.len() * 40 < n_seeds {
            break;
        }
        // Zeitbudget für sehr große organische Modelle
        // Arbeitsbudget statt Zeitbudget: gleiches Ergebnis auf jedem Rechner (die App auf dem Mac
        // rechnet sonst je nach Kernzahl/Tempo mehr oder weniger Flächen aus als die Kommandozeile).
        // Die Zeitgrenze bleibt nur als Notbremse.
        if work > 350_000 || t0.elapsed().as_secs() > 300 {
            break;
        }

        let mut g = None;
        for attempt in 0..3 {
            let init = initial_set(ctx, &pieces, &nbrs, &owner, seed, attempt);
            if init.len() < 3 {
                break;
            }
            let tris: Vec<u32> = init.iter().flat_map(|&p| pieces[p as usize].tris.iter().copied()).collect();
            let verts = unique_verts(m, &tris);
            if verts.len() < 10 {
                break;
            }
            work += verts.len() as u64;
            let fr = fit_any(ctx, &verts, &tris, None);
            if let Ok(bx) = std::env::var("M2S_SEEDBOX") {
                let b: Vec<f64> = bx.split(',').map(|x| x.parse().unwrap()).collect();
                let c = m.tri_centroid(pieces[seed as usize].tris[0] as usize);
                if c.x >= b[0] && c.x <= b[1] && c.y >= b[2] && c.y <= b[3] && c.z >= b[4] && c.z <= b[5] {
                    eprintln!("seed {seed} pass {pass} att {attempt}: init {:?} verts {} -> {:?}", init, verts.len(), fr.as_ref().map(|x| x.0.kind()));
                }
            }
            if let Some((s, sign)) = fr {
                g = Some(Grow { pieces: init, tris, verts, surf: Some(s), sign });
                break;
            }
        }
        let mut g = match g {
            Some(g) => g,
            None => continue,
        };
        let gid = curved.len() as u32;
        for &p in &g.pieces {
            owner[p as usize] = gid;
        }
        // Wachsen
        let mut queue: VecDeque<u32> = VecDeque::new();
        for &p in &g.pieces {
            for &q in &nbrs[p as usize] {
                queue.push_back(q);
            }
        }
        let mut refits = 0;
        let mut next_refit = g.verts.len() * 3 / 2;
        let mut g_area = area_of(m, &g.tris);
        for &v in &g.verts {
            vmark[v as usize] = gid;
        }
        let mut gen: u64 = 0;
        let mut rejected: Vec<u32> = Vec::new();
        let stamp = |gid: u32, gen: u64| ((gid as u64) << 24) | gen;
        let mut cooldown = 0usize;
        let mut stall_refits = 0;
        let mut stall_fails = 0;
        loop {
            let c = match queue.pop_front() {
                Some(c) => c,
                None => {
                    // Wachstum steht: mit dem am besten passenden abgelehnten Nachbarn neu anpassen
                    rejected.retain(|&q| owner[q as usize] == NONE);
                    if rejected.is_empty() || stall_refits >= 12 {
                        break;
                    }
                    stall_refits += 1;
                    let s = g.surf.unwrap();
                    let dev = |q: u32| pieces[q as usize].verts.iter().map(|&v| s.dist(m.verts[v as usize]).abs()).fold(0.0, f64::max);
                    let mut rs: Vec<(u32, f64)> = rejected.iter().map(|&q| (q, dev(q))).collect();
                    rs.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
                    // Präfixe der am besten passenden Nachbarn probieren (alle, Hälfte, Viertel, einer)
                    let nrs = rs.len();
                    let mut sizes = vec![nrs, nrs / 2, nrs / 4, 1];
                    sizes.retain(|&k| k >= 1);
                    sizes.dedup();
                    let mut found = None;
                    for &k in sizes.iter().skip(stall_fails.min(sizes.len() - 1)) {
                        let mut tris = g.tris.clone();
                        let mut verts = g.verts.clone();
                        let mut seen = std::collections::HashSet::new();
                        for &(q, _) in &rs[..k] {
                            let pc = &pieces[q as usize];
                            tris.extend_from_slice(&pc.tris);
                            for &v in &pc.verts {
                                if vmark[v as usize] != gid && seen.insert(v) {
                                    verts.push(v);
                                }
                            }
                        }
                        work += verts.len() as u64;
                        if let Some(r) = fit_any(ctx, &verts, &tris, Some(&s)) {
                            found = Some(r);
                            break;
                        }
                        stall_fails += 1;
                    }
                    if std::env::var("M2S_DEBUG2").is_ok() && gid < 2 {
                        eprintln!("g{gid} stall refit -> {:?}", found.map(|x| x.0.kind()));
                    }
                    match found {
                        Some((s2, sg)) => {
                            g.surf = Some(s2);
                            g.sign = sg;
                            gen += 1;
                            queue.extend(rejected.drain(..));
                            continue;
                        }
                        None => {
                            stall_fails += 1;
                            if stall_fails >= 4 {
                                break;
                            }
                            continue;
                        }
                    }
                }
            };
            let ci = c as usize;
            if owner[ci] != NONE || rej_stamp[ci] == stamp(gid, gen) {
                continue;
            }
            let s = g.surf.unwrap();
            let pc = &pieces[ci];
            let mut sign = g.sign;
            work += (pc.verts.len() / 8) as u64;
            let mut accepted = ctx.verify(&s, &pc.verts, &pc.tris, &mut sign);
            let mut surf_changed = false;
            let dbg2 = std::env::var("M2S_DEBUG2").is_ok() && gid < 2;
            if dbg2 && !accepted {
                eprintln!("g{gid} cand {ci} rejected; refits {refits} cooldown {cooldown} verts {} area {} vs {}", g.verts.len(), pc.area, g_area);
            }
            if !accepted && refits < 40 && pc.area < 20.0 * g_area && g.verts.len() >= cooldown {
                refits += 1;
                cooldown = g.verts.len() + (g.verts.len() / 5).max(8);
                let mut tris = g.tris.clone();
                tris.extend_from_slice(&pc.tris);
                let mut verts = g.verts.clone();
                verts.extend(pc.verts.iter().copied().filter(|&v| vmark[v as usize] != gid));
                work += verts.len() as u64;
                let r = fit_any(ctx, &verts, &tris, Some(&s));
                if dbg2 {
                    eprintln!("   refit -> {:?}", r.map(|x| x.0.kind()));
                }
                if let Some((s2, sg)) = r {
                    g.surf = Some(s2);
                    g.sign = sg;
                    accepted = true;
                    sign = sg;
                    surf_changed = true;
                }
            }
            if !accepted {
                rej_stamp[ci] = stamp(gid, gen);
                rejected.push(c);
                continue;
            }
            g.sign = sign;
            owner[ci] = gid;
            g.pieces.push(c);
            g.tris.extend_from_slice(&pc.tris);
            g_area += pc.area;
            for &v in &pc.verts {
                if vmark[v as usize] != gid {
                    vmark[v as usize] = gid;
                    g.verts.push(v);
                }
            }
            for &q in &nbrs[ci] {
                if owner[q as usize] == NONE {
                    queue.push_back(q);
                }
            }
            if g.verts.len() >= next_refit {
                next_refit = g.verts.len() * 3 / 2;
                work += g.verts.len() as u64;
                if let Some((s2, sg)) = fit_any(ctx, &g.verts, &g.tris, g.surf.as_ref()) {
                    g.surf = Some(s2);
                    g.sign = sg;
                    surf_changed = true;
                }
            }
            if surf_changed && !rejected.is_empty() {
                // Mit verbesserter Fläche abgelehnte Nachbarn erneut prüfen
                gen += 1;
                queue.extend(rejected.drain(..));
            }
        }
        let sf = g.surf.unwrap();
        if let Ok(bx) = std::env::var("M2S_SEEDBOX") {
            let b: Vec<f64> = bx.split(',').map(|x| x.parse().unwrap()).collect();
            let c = m.tri_centroid(pieces[seed as usize].tris[0] as usize);
            if c.x >= b[0] && c.x <= b[1] && c.y >= b[2] && c.y <= b[3] && c.z >= b[4] && c.z <= b[5] {
                eprintln!("  grown seed {seed}: {:?} {} Dreiecke, signifikant {}", sf, g.tris.len(), significant(m, &sf, &g.verts, g_area / total_area));
            }
        }
        if !significant(m, &sf, &g.verts, g_area / total_area) {
            // Zu kleines Flächenstück (typisch für organische Netze): verwerfen
            for &p in &g.pieces {
                owner[p as usize] = NONE;
                tried[p as usize] = true;
            }
            n_rejected += 1;
            if n_rejected > 150 && n_rejected > 3 * curved.len() {
                break; // Freiform-Modell: weitere Suche lohnt nicht
            }
            continue;
        }
        // Schnellschätzung, ob das Stück sauber begrenzt ist (für Freiform-Abbruch)
        {
            let (mut tot, mut cl) = (0usize, 0usize);
            for &t in &g.tris {
                for k in 0..3 {
                    let tw = m.twin[(3 * t + k) as usize];
                    if tw == NONE {
                        continue;
                    }
                    let u = (tw / 3) as usize;
                    let pu = seg.region_of[u] as usize;
                    if owner[pu] == gid {
                        continue;
                    }
                    tot += 1;
                    if angle(m.tri_n[t as usize], m.tri_n[u]) > 0.5 * ctx.smooth || pieces[pu].tris.len() >= 8 {
                        cl += 1;
                    }
                }
            }
            if tot > 0 && (cl as f64) < 0.3 * tot as f64 {
                n_unclean += 1;
            }
            if n_unclean > 200 && n_unclean * 5 > (curved.len() + 1) * 4 {
                curved.push(Region { tris: g.tris, surf: sf });
                curved_pieces.push(g.pieces);
                break; // Freiform-Modell
            }
        }
        curved.push(Region { tris: g.tris, surf: sf });
        curved_pieces.push(g.pieces);
    }

    if std::env::var("M2S_DEBUG").is_ok() {
        eprintln!("growth: {:.1}s, work {work}, {} regions, {} unclean, {} rejected, seeds {}", t0.elapsed().as_secs_f64(), curved.len(), n_unclean, n_rejected, n_seeds);
        let st: Vec<u64> = FIT_STATS.iter().map(|x| x.load(std::sync::atomic::Ordering::Relaxed)).collect();
        eprintln!("fit µs cyl/cone/sphere/torus(plane=0 idx): {:?}", st);
    }
    if std::env::var("M2S_DEBUG").is_ok() {
    }
    // Unsaubere Stücke (Freiform: glatte Übergänge zu fremden Flächen) wieder auflösen
    if std::env::var("M2S_DEBUG2").is_ok() { for r in &curved { eprintln!("C {} {:?}", r.tris.len(), r.surf); } }
    let nc = curved.len();
    let mut alive = vec![true; nc];
    let piece_big: Vec<bool> = pieces.iter().map(|p| p.tris.len() >= 8 || p.area > 0.005 * total_area).collect();
    for _round in 0..4 {
        let mut label = vec![NONE; m.tris.len()];
        for (i, r) in curved.iter().enumerate() {
            if alive[i] {
                for &t in &r.tris {
                    label[t as usize] = i as u32;
                }
            }
        }
        for (pi, pc) in pieces.iter().enumerate() {
            let o = owner[pi];
            if o == NONE || !alive[o as usize] {
                for &t in &pc.tris {
                    label[t as usize] = (nc + pi) as u32;
                }
            }
        }
        let mut changed = false;
        let mut order_c: Vec<usize> = (0..nc).collect();
        order_c.sort_by(|&a, &b| area_of(m, &curved[a].tris).partial_cmp(&area_of(m, &curved[b].tris)).unwrap());
        // Erst feststellen, welche Stücke unsauber sind; ein Stück, das nur an andere unsaubere
        // Stücke grenzt, bekommt nach deren Auflösung eine zweite Chance (siehe unten)
        for i in order_c {
            if !alive[i] {
                continue;
            }
            let mut fits_cache: HashMap<u32, bool> = HashMap::new();
            let (mut total, mut clean) = (0usize, 0usize);
            let (mut ltot, mut lclean) = (0.0f64, 0.0f64);
            for &t in &curved[i].tris {
                for k in 0..3 {
                    let tw = m.twin[(3 * t + k) as usize];
                    let el = m.verts[m.tris[t as usize][k as usize] as usize].dist(m.verts[m.tris[t as usize][(k as usize + 1) % 3] as usize]);
                    if tw == NONE {
                        total += 1;
                        clean += 1;
                        ltot += el;
                        lclean += el;
                        continue;
                    }
                    let u = (tw / 3) as usize;
                    let nb = label[u];
                    if nb == i as u32 {
                        continue;
                    }
                    total += 1;
                    ltot += el;
                    let dih = angle(m.tri_n[t as usize], m.tri_n[u]);
                    let ok = dih > 0.5 * ctx.smooth
                        || (nb as usize >= nc && piece_big[nb as usize - nc])
                        || ((nb as usize) < nc && related(&curved[i].surf, &curved[nb as usize].surf, ctx.tol))
                        // Nachbar gehört zu einer anderen gekrümmten Region, das Grenzstück passt aber
                        // auch auf diese Fläche (die andere Region hat es sich nur zuerst genommen)
                        || ((nb as usize) < nc && {
                            let pi = seg.region_of[u] as usize;
                            *fits_cache.entry((nc + pi) as u32 | 0x8000_0000).or_insert_with(|| {
                                let pc = &pieces[pi];
                                let mut sg = 0.0;
                                ctx.verify(&curved[i].surf, &pc.verts, &pc.tris, &mut sg)
                            })
                        })
                        || (nb as usize >= nc
                            && *fits_cache.entry(nb).or_insert_with(|| {
                                let pc = &pieces[nb as usize - nc];
                                let mut sg = 0.0;
                                ctx.verify(&curved[i].surf, &pc.verts, &pc.tris, &mut sg)
                            }));
                    if ok {
                        clean += 1;
                        lclean += el;
                    }
                }
            }
            if let Ok(bx) = std::env::var("M2S_SEEDBOX") {
                let b: Vec<f64> = bx.split(',').map(|x| x.parse().unwrap()).collect();
                let c = m.tri_centroid(curved[i].tris[0] as usize);
                if c.x >= b[0] && c.x <= b[1] && c.y >= b[2] && c.y <= b[3] && c.z >= b[4] && c.z <= b[5] {
                    eprintln!("prune {i}: {:?} {} Dreiecke clean {clean}/{total}", curved[i].surf.kind(), curved[i].tris.len());
                }
            }
            // Nach Anzahl der Randkanten oder nach Randlänge sauber genug (lange Streifendreiecke
            // grob vernetzter Rundungen haben wenige, aber lange Kanten)
            if total > 0 && (clean as f64) < 0.6 * total as f64 && lclean < 0.75 * ltot {
                alive[i] = false;
                changed = true;
                for &p in &curved_pieces[i] {
                    owner[p as usize] = NONE;
                    for &t in &pieces[p as usize].tris {
                        label[t as usize] = (nc + p as usize) as u32;
                    }
                }
            }
        }
        if !changed {
            break;
        }
    }
    let curved: Vec<Region> = curved.into_iter().zip(alive.iter()).filter(|(_, &a)| a).map(|(r, _)| r).collect();
    if std::env::var("M2S_DEBUG").is_ok() { eprintln!("prune done: {:.1}s, {} alive", t0.elapsed().as_secs_f64(), curved.len()); }

    // Neue Segmentierung zusammenbauen
    let mut region_of = vec![NONE; m.tris.len()];
    let mut regions: Vec<Region> = Vec::new();
    for r in curved {
        let id = regions.len() as u32;
        for &t in &r.tris {
            region_of[t as usize] = id;
        }
        regions.push(r);
    }
    for (pi, r) in seg.regions.into_iter().enumerate() {
        if owner[pi] == NONE {
            let id = regions.len() as u32;
            for &t in &r.tris {
                region_of[t as usize] = id;
            }
            regions.push(r);
        }
    }
    Segmentation { region_of, regions }
}

fn area_of(m: &Mesh, tris: &[u32]) -> f64 {
    tris.iter().map(|&t| m.tri_a[t as usize]).sum()
}

/// Startmenge für das Wachstum: kleine glatt verbundene Nachbarstücke.
fn initial_set(ctx: &Ctx, pieces: &[Piece], nbrs: &[Vec<u32>], owner: &[u32], seed: u32, attempt: usize) -> Vec<u32> {
    let mut set = vec![seed];
    let mut nverts = pieces[seed as usize].verts.len();
    let mut frontier: Vec<u32> = nbrs[seed as usize].iter().copied().filter(|&q| owner[q as usize] == NONE).collect();
    let seed_area = pieces[seed as usize].area;
    let _ = ctx;
    while (set.len() < 3 || nverts < 14) && set.len() < 24 {
        frontier.retain(|q| !set.contains(q));
        if attempt == 1 {
            // zweiter Versuch: große (wahrscheinlich ebene) Nachbarn meiden
            frontier.retain(|&q| pieces[q as usize].area <= 4.0 * seed_area.max(1e-30));
        }
        if frontier.is_empty() {
            break;
        }
        if attempt == 2 {
            // dritter Versuch: nur ähnlich große Nachbarn, größte zuerst (lange Streifen einer
            // grob vernetzten Rundung, ohne die kleinen Eckdreiecke daneben)
            frontier.retain(|&q| pieces[q as usize].area >= 0.1 * seed_area);
            if frontier.is_empty() {
                break;
            }
        }
        // kleinstes Stück zuerst (dritter Versuch: größtes)
        let cmp = |a: &(usize, &u32), b: &(usize, &u32)| pieces[*a.1 as usize].area.partial_cmp(&pieces[*b.1 as usize].area).unwrap();
        let (bi, _) = if attempt == 2 { frontier.iter().enumerate().max_by(cmp).unwrap() } else { frontier.iter().enumerate().min_by(cmp).unwrap() };
        let q = frontier.swap_remove(bi);
        set.push(q);
        nverts += pieces[q as usize].verts.len().saturating_sub(2);
        for &r in &nbrs[q as usize] {
            if owner[r as usize] == NONE && !set.contains(&r) && !frontier.contains(&r) {
                frontier.push(r);
            }
        }
    }
    set
}

// ---------------------------------------------------------------------------
// Benachbarte Regionen mit gleicher Fläche verschmelzen

fn find(uf: &mut [u32], mut x: u32) -> u32 {
    while uf[x as usize] != x {
        uf[x as usize] = uf[uf[x as usize] as usize];
        x = uf[x as usize];
    }
    x
}

fn merge_regions(ctx: &Ctx, seg: &mut Segmentation) {
    let m = ctx.m;
    for _round in 0..64 {
        let n = seg.regions.len();
        let adj = region_adjacency(m, &seg.region_of);
        let mut uf: Vec<u32> = (0..n as u32).collect();
        let mut pairs: Vec<(u32, u32)> = adj.keys().filter(|(a, b)| a < b).copied().collect();
        pairs.sort();
        let verts: Vec<Vec<u32>> = seg.regions.iter().map(|r| unique_verts(m, &r.tris)).collect();
        // Orientierung jeder Region bzgl. ihrer eigenen Fläche
        let sign: Vec<Option<f64>> = seg
            .regions
            .iter()
            .zip(&verts)
            .map(|(r, v)| {
                let mut sg = 0.0;
                if ctx.verify(&r.surf, v, &r.tris, &mut sg) {
                    Some(sg)
                } else {
                    None
                }
            })
            .collect();
        // Repräsentant (größtes Mitglied), Dreiecksanzahl und Mitgliederzahl je Wurzel
        let mut rep: Vec<u32> = (0..n as u32).collect();
        let mut cnt: Vec<usize> = seg.regions.iter().map(|r| r.tris.len()).collect();
        let mut mem: Vec<usize> = vec![1; n];
        let mut merged_any = false;
        for (a, b) in pairs {
            let ra = find(&mut uf, a);
            let rb = find(&mut uf, b);
            if ra == rb {
                continue;
            }
            let (rbig, rsmall, small) = if cnt[ra as usize] >= cnt[rb as usize] { (ra, rb, b) } else { (rb, ra, a) };
            let big = rep[rbig as usize];
            let sbig = seg.regions[big as usize].surf;
            let ssmall = seg.regions[rep[rsmall as usize] as usize].surf;
            let mut sg = match sign[big as usize] {
                Some(s) => s,
                None => continue,
            };
            let same = sbig.kind() == ssmall.kind() && surf::same_surface(&sbig, &ssmall, ctx.tol);
            let ok = same
                || (mem[rsmall as usize] == 1
                    && ctx.verify(&sbig, &verts[small as usize], &seg.regions[small as usize].tris, &mut sg));
            if !ok {
                continue;
            }
            if let (Surf::Plane { n: n1, .. }, Surf::Plane { n: n2, .. }) = (sbig, ssmall) {
                if n1.dot(n2) < 0.0 {
                    continue;
                }
            }
            let (keep, gone) = (ra.min(rb), ra.max(rb));
            uf[gone as usize] = keep;
            rep[keep as usize] = big;
            cnt[keep as usize] = cnt[ra as usize] + cnt[rb as usize];
            mem[keep as usize] = mem[ra as usize] + mem[rb as usize];
            merged_any = true;
        }
        if !merged_any {
            break;
        }
        // Neu aufbauen
        let mut groups: HashMap<u32, Vec<u32>> = HashMap::new();
        for i in 0..n as u32 {
            let r = find(&mut uf, i);
            groups.entry(r).or_default().push(i);
        }
        let mut keys: Vec<u32> = groups.keys().copied().collect();
        keys.sort();
        let mut new_regions = Vec::new();
        for k in keys {
            let members = &groups[&k];
            let mut tris = Vec::new();
            for &i in members {
                tris.extend_from_slice(&seg.regions[i as usize].tris);
            }
            let main = *members.iter().max_by_key(|&&i| seg.regions[i as usize].tris.len()).unwrap();
            let mut s = seg.regions[main as usize].surf;
            if members.len() > 1 {
                let vs = unique_verts(m, &tris);
                let refit = match s {
                    Surf::Plane { n, .. } => {
                        let pts: Vec<V3> = vs.iter().map(|&v| m.verts[v as usize]).collect();
                        surf::fit_plane(&pts).map(|p| match p {
                            Surf::Plane { o, n: nn } => Surf::Plane { o, n: if nn.dot(n) < 0.0 { -nn } else { nn } },
                            x => x,
                        })
                    }
                    _ => fit_any(ctx, &vs, &tris, Some(&s)).map(|r| r.0),
                };
                if let Some(r) = refit {
                    let mut sg = 0.0;
                    if ctx.verify(&r, &vs, &tris, &mut sg) {
                        s = r;
                    }
                }
            }
            new_regions.push(Region { tris, surf: s });
        }
        let mut region_of = vec![NONE; m.tris.len()];
        for (i, r) in new_regions.iter().enumerate() {
            for &t in &r.tris {
                region_of[t as usize] = i as u32;
            }
        }
        seg.regions = new_regions;
        seg.region_of = region_of;
    }
}

// ---------------------------------------------------------------------------
// Achsen und Normalen ausrichten (parallel zu Koordinatenachsen / zueinander)

fn regularize(ctx: &Ctx, seg: &mut Segmentation) {
    let m = ctx.m;
    let snap_ang = 0.15f64.to_radians();
    // Referenzrichtungen: Koordinatenachsen + Achsen großer Flächen
    let mut refs: Vec<V3> = vec![V3::X, V3::Y, V3::Z];
    let mut dirs: Vec<(usize, V3, f64)> = Vec::new();
    for (i, r) in seg.regions.iter().enumerate() {
        let area = area_of(m, &r.tris);
        match r.surf {
            Surf::Plane { n, .. } => dirs.push((i, n, area)),
            Surf::Cyl { a, .. } | Surf::Cone { a, .. } | Surf::Torus { a, .. } => dirs.push((i, a, area)),
            _ => {}
        }
    }
    dirs.sort_by(|a, b| b.2.partial_cmp(&a.2).unwrap());
    // Nur die größten Flächen dienen als Referenzrichtungen (sonst quadratischer Aufwand)
    for &(_, d, _) in dirs.iter().take(64) {
        if !refs.iter().any(|r| angle(*r, d) < snap_ang || angle(*r, -d) < snap_ang) {
            refs.push(d);
        }
    }
    for (i, d, _) in dirs {
        let mut target = None;
        for r in &refs {
            if angle(*r, d) < snap_ang {
                target = Some(*r);
                break;
            }
            if angle(-*r, d) < snap_ang {
                target = Some(-*r);
                break;
            }
        }
        let target = match target {
            Some(t) if angle(t, d) > 1e-12 => t,
            _ => continue,
        };
        let r = &seg.regions[i];
        let vs = unique_verts(m, &r.tris);
        let pts: Vec<V3> = vs.iter().map(|&v| m.verts[v as usize]).collect();
        let cand = match r.surf {
            Surf::Plane { .. } => Some(surf::fit_plane_dir(&pts, target)),
            Surf::Cyl { r: rad, .. } => {
                let (e1, e2) = target.basis();
                let p2: Vec<(f64, f64)> = pts.iter().map(|p| (p.dot(e1), p.dot(e2))).collect();
                surf::fit_circle_2d(&p2).map(|(cx, cy, rr, _)| {
                    let _ = rad;
                    Surf::Cyl { p: e1 * cx + e2 * cy, a: target, r: rr }
                })
            }
            Surf::Cone { p, r: rad, ang, .. } => Some(Surf::Cone { p, a: target, r: rad, ang }),
            Surf::Torus { c, rmaj, rmin, .. } => Some(Surf::Torus { c, a: target, rmaj, rmin }),
            s => Some(s),
        };
        if let Some(c) = cand {
            let mut sg = 0.0;
            if ctx.verify(&c, &vs, &r.tris, &mut sg) {
                seg.regions[i].surf = c;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Regionen ohne Rand (z. B. komplette Kugel) teilen, damit jede Fläche einen Rand hat

fn split_boundaryless(ctx: &Ctx, seg: &mut Segmentation) {
    let m = ctx.m;
    let n = seg.regions.len();
    for i in 0..n {
        let has_boundary = seg.regions[i].tris.iter().any(|&t| {
            (0..3).any(|k| {
                let tw = m.twin[(3 * t + k) as usize];
                tw == NONE || seg.region_of[(tw / 3) as usize] != i as u32
            })
        });
        if has_boundary {
            continue;
        }
        let s = seg.regions[i].surf;
        let (c, dir) = match s {
            Surf::Sphere { c, .. } => (c, V3::Z),
            Surf::Torus { c, a, .. } => (c, a.any_perp()),
            Surf::Cyl { p, a, .. } | Surf::Cone { p, a, .. } => (p, a.any_perp()),
            Surf::Plane { o, n } => (o, n.any_perp()),
            Surf::BSpline { id } => {
                let b = crate::bspline::get(id);
                (m.tri_centroid(seg.regions[i].tris[0] as usize), b.e1)
            }
        };
        let tris = std::mem::take(&mut seg.regions[i].tris);
        let (mut a, mut b): (Vec<u32>, Vec<u32>) = (Vec::new(), Vec::new());
        for t in tris {
            if (m.tri_centroid(t as usize) - c).dot(dir) >= 0.0 {
                a.push(t)
            } else {
                b.push(t)
            }
        }
        if a.is_empty() || b.is_empty() {
            let all: Vec<u32> = a.into_iter().chain(b).collect();
            let half = all.len() / 2;
            a = all[..half].to_vec();
            b = all[half..].to_vec();
        }
        let nid = seg.regions.len() as u32;
        for &t in &b {
            seg.region_of[t as usize] = nid;
        }
        seg.regions[i].tris = a;
        seg.regions.push(Region { tris: b, surf: s });
    }
}

/// Ist ein gekrümmtes Flächenstück groß genug, um eine echte CAD-Fläche zu sein?
/// (Winkelüberdeckung >= 20° oder nennenswerter Flächenanteil)
fn significant(m: &Mesh, s: &Surf, verts: &[u32], area_frac: f64) -> bool {
    if area_frac > 0.02 {
        return true;
    }
    let min_span = 20f64.to_radians();
    let pts: Vec<V3> = verts.iter().map(|&v| m.verts[v as usize]).collect();
    let coverage = |angles: &mut Vec<f64>| -> f64 {
        if angles.len() < 2 {
            return 0.0;
        }
        angles.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let mut gap = angles[0] + std::f64::consts::TAU - angles[angles.len() - 1];
        for w in angles.windows(2) {
            gap = gap.max(w[1] - w[0]);
        }
        std::f64::consts::TAU - gap
    };
    let around = |c: V3, a: V3| -> f64 {
        let (e1, e2) = a.basis();
        let mut ang: Vec<f64> = pts.iter().map(|p| ((*p - c).dot(e2)).atan2((*p - c).dot(e1))).collect();
        coverage(&mut ang)
    };
    match *s {
        Surf::Cyl { p, a, .. } | Surf::Cone { p, a, .. } => around(p, a) >= min_span,
        Surf::Torus { c, a, rmaj, .. } => {
            if around(c, a) >= min_span {
                return true;
            }
            // Winkel um die Röhre
            let mut ang: Vec<f64> = pts
                .iter()
                .map(|p| {
                    let d = *p - c;
                    let h = d.dot(a);
                    let rho = (d - a * h).norm();
                    h.atan2(rho - rmaj)
                })
                .collect();
            coverage(&mut ang) >= min_span
        }
        Surf::Sphere { c, r } => {
            let mut mn = V3 { x: f64::MAX, y: f64::MAX, z: f64::MAX };
            let mut mx = -mn;
            for p in &pts {
                mn = mn.min(*p);
                mx = mx.max(*p);
            }
            let _ = c;
            (mx - mn).norm() / r >= 2.0 * (min_span / 2.0).sin()
        }
        Surf::Plane { .. } | Surf::BSpline { .. } => true,
    }
}

/// Zwei Flächen gehören geometrisch zusammen (koaxial bzw. Kugelmittelpunkt auf der Achse).
fn related(a: &Surf, b: &Surf, tol: f64) -> bool {
    let lim = tol * 20.0;
    let on_axis = |q: V3, p: V3, d: V3| {
        let v = q - p;
        (v - d * v.dot(d)).norm() < lim
    };
    match (a.axis(), b.axis()) {
        (Some((p1, a1)), Some((p2, a2))) => angle(a1, a2).min(angle(a1, -a2)) < 2e-3 && on_axis(p2, p1, a1),
        (Some((p, d)), None) | (None, Some((p, d))) => match (*a, *b) {
            (Surf::Sphere { c, .. }, _) | (_, Surf::Sphere { c, .. }) => on_axis(c, p, d),
            _ => false,
        },
        (None, None) => match (*a, *b) {
            (Surf::Sphere { c: c1, .. }, Surf::Sphere { c: c2, .. }) => c1.dist(c2) < lim,
            _ => false,
        },
    }
}

/// Debug-Hilfe: prüft Dreiecke gegen eine Fläche.
pub fn debug_verify(m: &Mesh, tol: f64, smooth_deg: f64, s: &Surf, tris: &[u32]) -> bool {
    let smooth = smooth_deg.to_radians();
    let ctx = Ctx { m, tol, cos_smooth: smooth.cos(), smooth, min_area: (m.diag() * 1e-6).powi(2) };
    let vs = unique_verts(m, tris);
    let mut sg = 0.0;
    ctx.verify(s, &vs, tris, &mut sg)
}

/// Einzelne Dreiecke ebener Stücke, die an eine gekrümmte Region grenzen und auf deren
/// Fläche liegen, dieser Region zuschlagen (z. B. Splitter an Kanten).
fn absorb_triangles(ctx: &Ctx, seg: &mut Segmentation) {
    let m = ctx.m;
    let n = seg.regions.len();
    let sign: Vec<f64> = seg
        .regions
        .iter()
        .map(|r| {
            let mut sg = 0.0;
            let vs = unique_verts(m, &r.tris);
            ctx.verify(&r.surf, &vs, &r.tris, &mut sg);
            sg
        })
        .collect();
    let curved: Vec<bool> = seg.regions.iter().map(|r| !matches!(r.surf, Surf::Plane { .. })).collect();
    if !curved.iter().any(|&c| c) {
        return;
    }
    let mut changed = true;
    let mut iter = 0;
    while changed && iter < 50 {
        changed = false;
        iter += 1;
        for t in 0..m.tris.len() {
            let rt = seg.region_of[t] as usize;
            if curved[rt] {
                continue;
            }
            for k in 0..3 {
                let tw = m.twin[3 * t + k];
                if tw == NONE {
                    continue;
                }
                let rn = seg.region_of[(tw / 3) as usize] as usize;
                if !curved[rn] {
                    continue;
                }
                let mut sg = sign[rn];
                let tv = m.tris[t];
                if ctx.verify(&seg.regions[rn].surf, &tv, &[t as u32], &mut sg) {
                    seg.region_of[t] = rn as u32;
                    changed = true;
                    break;
                }
            }
        }
    }
    let mut lists: Vec<Vec<u32>> = vec![Vec::new(); n];
    for (t, &r) in seg.region_of.iter().enumerate() {
        lists[r as usize].push(t as u32);
    }
    for (r, l) in seg.regions.iter_mut().zip(lists) {
        r.tris = l;
    }
}

/// Jede Region in zusammenhängende Teile zerlegen (eine Fläche = ein zusammenhängendes Stück).
fn split_components(ctx: &Ctx, seg: &mut Segmentation) {
    let m = ctx.m;
    let mut new_regions: Vec<Region> = Vec::new();
    let mut region_of = vec![NONE; m.tris.len()];
    let mut stack = Vec::new();
    for r in &seg.regions {
        for &t0 in &r.tris {
            if region_of[t0 as usize] != NONE {
                continue;
            }
            let id = new_regions.len() as u32;
            let mut tris = Vec::new();
            region_of[t0 as usize] = id;
            stack.push(t0);
            let old = seg.region_of[t0 as usize];
            while let Some(t) = stack.pop() {
                tris.push(t);
                for k in 0..3 {
                    let tw = m.twin[(3 * t + k) as usize];
                    if tw == NONE {
                        continue;
                    }
                    let u = tw / 3;
                    if region_of[u as usize] == NONE && seg.region_of[u as usize] == old {
                        region_of[u as usize] = id;
                        stack.push(u);
                    }
                }
            }
            new_regions.push(Region { tris, surf: r.surf });
        }
    }
    seg.regions = new_regions;
    seg.region_of = region_of;
}

/// Ersetzt die angegebenen Regionen durch Einzeldreiecke (ebene Facetten).
pub fn demote_to_facets(m: &Mesh, seg: &mut Segmentation, regions: &[u32]) {
    let mut set: Vec<u32> = regions.to_vec();
    set.sort_unstable();
    set.dedup();
    for &r in &set {
        let tris = std::mem::take(&mut seg.regions[r as usize].tris);
        let mut first = true;
        for t in tris {
            let n = if m.tri_n[t as usize] == V3::ZERO { V3::Z } else { m.tri_n[t as usize] };
            let surf = Surf::Plane { o: m.tri_centroid(t as usize), n };
            if first {
                seg.regions[r as usize] = Region { tris: vec![t], surf };
                first = false;
            } else {
                seg.region_of[t as usize] = seg.regions.len() as u32;
                seg.regions.push(Region { tris: vec![t], surf });
            }
        }
    }
}

/// Toleranz für Freiformflächen (etwas großzügiger als für Grundformen).
pub fn freeform_tol(m: &Mesh, tol: f64) -> f64 {
    let f: f64 = std::env::var("M2S_FFTOL").ok().and_then(|s| s.parse().ok()).unwrap_or(2.5e-4);
    tol.max((m.diag() * f).min(0.05))
}

/// Glatte Facettenbereiche ohne passende Grundform durch B-Spline-Flächen ersetzen.
fn freeform_regions(ctx: &Ctx, seg: &mut Segmentation) {
    let m = ctx.m;
    let ff_tol = freeform_tol(m, ctx.tol);
    let nreg = seg.regions.len();
    let total_area: f64 = m.tri_a.iter().sum::<f64>().max(1e-300);
    // Kandidaten: kleine ebene Stücke (Facetten). Große ebene Flächen aus wenigen Dreiecken sind echte Ebenen.
    let mut cand: Vec<bool> = seg
        .regions
        .iter()
        .map(|r| matches!(r.surf, Surf::Plane { .. }) && r.tris.len() <= 6 && area_of(m, &r.tris) < 0.003 * total_area)
        .collect();
    // Schmale ebene Streifen (CAD-Export gekrümmter Flächen wie Zahnflanken: lange Streifen aus
    // vielen Dreiecken, jeder für sich exakt eben), die längs flach in Nachbarstreifen übergehen
    if std::env::var("M2S_NOSTRIPS").is_err() {
        let lim_hi = 25f64.to_radians();
        let lim_lo = 0.3f64.to_radians();
        for (ri, r) in seg.regions.iter().enumerate() {
            if cand[ri] || r.tris.len() > 400 || !matches!(r.surf, Surf::Plane { .. }) || area_of(m, &r.tris) > 0.01 * total_area {
                continue;
            }
            let vs = unique_verts(m, &r.tris);
            if vs.len() < 3 {
                continue;
            }
            let mut c = V3::ZERO;
            for &v in &vs {
                c += m.verts[v as usize];
            }
            c = c / vs.len() as f64;
            let mut cov = crate::math::M3::zero();
            for &v in &vs {
                cov.add_outer(m.verts[v as usize] - c, 1.0);
            }
            let (ew, ev) = cov.eigen_sym();
            let (long, short) = (ev[2], ev[1]);
            let _ = ew;
            let ext = |d: V3| {
                let (mut lo, mut hi) = (f64::MAX, f64::MIN);
                for &v in &vs {
                    let x = (m.verts[v as usize] - c).dot(d);
                    lo = lo.min(x);
                    hi = hi.max(x);
                }
                hi - lo
            };
            let (l, w) = (ext(long), ext(short));
            if !(l > 0.0 && w < 0.2 * l) {
                continue;
            }
            let (mut tot, mut sm) = (0.0f64, 0.0f64);
            for &t in &r.tris {
                for k in 0..3 {
                    let h = (3 * t + k) as usize;
                    let tw = m.twin[h];
                    if tw == NONE {
                        continue;
                    }
                    let u = (tw / 3) as usize;
                    if seg.region_of[u] as usize == ri {
                        continue;
                    }
                    let l = m.verts[m.he_from(h as u32) as usize].dist(m.verts[m.he_to(h as u32) as usize]);
                    tot += l;
                    let a = angle(m.tri_n[t as usize], m.tri_n[u]);
                    if a > lim_lo && a < lim_hi {
                        sm += l;
                    }
                }
            }
            if tot > 0.0 && sm >= 0.5 * tot {
                cand[ri] = true;
            }
        }
    }
    if !cand.iter().any(|&c| c) {
        return;
    }
    let elen = |h: usize| m.verts[m.he_from(h as u32) as usize].dist(m.verts[m.he_to(h as u32) as usize]);
    // Kleine Regionen, die (nach Kantenlänge) überwiegend glatt in Facettenbereiche übergehen, mitnehmen
    for _ in 0..8 {
        let mut changed = false;
        for ri in 0..nreg {
            // Nur ebene Stücke mitnehmen; erkannte Grundformen bleiben erhalten
            if cand[ri] || seg.regions[ri].tris.len() > 400 || !matches!(seg.regions[ri].surf, Surf::Plane { .. }) {
                continue;
            }
            if area_of(m, &seg.regions[ri].tris) > 0.02 * total_area {
                continue;
            }
            let (mut tot, mut sm) = (0.0f64, 0.0f64);
            for &t in &seg.regions[ri].tris {
                for k in 0..3 {
                    let h = (3 * t + k) as usize;
                    let tw = m.twin[h];
                    if tw == NONE {
                        continue;
                    }
                    let u = (tw / 3) as usize;
                    let ru = seg.region_of[u] as usize;
                    if ru == ri {
                        continue;
                    }
                    let l = elen(h);
                    tot += l;
                    if cand[ru] && angle(m.tri_n[t as usize], m.tri_n[u]) <= ctx.smooth {
                        sm += l;
                    }
                }
            }
            if tot > 0.0 && sm >= 0.5 * tot {
                cand[ri] = true;
                changed = true;
            }
        }
        if !changed {
            break;
        }
    }
    let nt = m.tris.len();
    let mut done = vec![false; nt];
    let fine: f64 = std::env::var("M2S_FFANG").ok().and_then(|s| s.parse::<f64>().ok()).unwrap_or(6.0).to_radians();
    // 1. Durchgang: sehr glatte Bereiche (große Flächen), 2. Durchgang: Rest inkl. Rundungen
    for pass_angle in [fine, ctx.smooth] {
        let mut comp = vec![NONE; nt];
        let mut comps: Vec<Vec<u32>> = Vec::new();
        let mut stack = Vec::new();
        let ok_tri = |t: usize, done: &Vec<bool>| !done[t] && cand[seg.region_of[t] as usize];
        for t0 in 0..nt {
            if comp[t0] != NONE || !ok_tri(t0, &done) {
                continue;
            }
            let id = comps.len() as u32;
            let mut list = Vec::new();
            comp[t0] = id;
            stack.push(t0 as u32);
            while let Some(t) = stack.pop() {
                list.push(t);
                for k in 0..3 {
                    let tw = m.twin[(3 * t + k) as usize];
                    if tw == NONE {
                        continue;
                    }
                    let u = (tw / 3) as usize;
                    if comp[u] == NONE && ok_tri(u, &done) && angle(m.tri_n[t as usize], m.tri_n[u]) <= pass_angle {
                        comp[u] = id;
                        stack.push(u as u32);
                    }
                }
            }
            comps.push(list);
        }
        let mut new_regions: Vec<Region> = Vec::new();
        for c in comps {
            if c.len() >= 12 {
                fit_freeform(ctx, &c, ff_tol, 0, &mut new_regions);
            }
        }
        for r in new_regions {
            let id = seg.regions.len() as u32;
            for &t in &r.tris {
                seg.region_of[t as usize] = id;
                done[t as usize] = true;
            }
            seg.regions.push(r);
        }
        cand.resize(seg.regions.len(), false);
    }
    rebuild_lists(seg);
    // Restliche kleine Facetten an Freiformflächen angrenzend übernehmen, wenn sie darauf liegen
    let fctx = Ctx { m, tol: ff_tol, cos_smooth: ctx.cos_smooth, smooth: ctx.smooth, min_area: ctx.min_area };
    let is_ff: Vec<bool> = seg.regions.iter().map(|r| matches!(r.surf, Surf::BSpline { .. })).collect();
    let small: Vec<bool> = seg.regions.iter().map(|r| matches!(r.surf, Surf::Plane { .. }) && r.tris.len() <= 12).collect();
    let mut changed = true;
    let mut it = 0;
    while changed && it < 30 {
        changed = false;
        it += 1;
        for t in 0..nt {
            let rt = seg.region_of[t] as usize;
            if !small[rt] {
                continue;
            }
            for k in 0..3 {
                let tw = m.twin[3 * t + k];
                if tw == NONE {
                    continue;
                }
                let rn = seg.region_of[(tw / 3) as usize] as usize;
                if !is_ff[rn] || angle(m.tri_n[t], m.tri_n[(tw / 3) as usize]) > ctx.smooth {
                    continue;
                }
                let mut sg = 1.0;
                if fctx.verify(&seg.regions[rn].surf, &m.tris[t], &[t as u32], &mut sg) {
                    seg.region_of[t] = rn as u32;
                    changed = true;
                    break;
                }
            }
        }
    }
    rebuild_lists(seg);
}

fn rebuild_lists(seg: &mut Segmentation) {
    let mut lists: Vec<Vec<u32>> = vec![Vec::new(); seg.regions.len()];
    for (t, &r) in seg.region_of.iter().enumerate() {
        lists[r as usize].push(t as u32);
    }
    for (r, l) in seg.regions.iter_mut().zip(lists) {
        r.tris = l;
    }
}

/// Passt eine Freiformfläche an; teilt den Bereich, wenn er kein Höhenfeld ist oder nicht passt.
fn fit_freeform(ctx: &Ctx, tris: &[u32], ff_tol: f64, depth: usize, out: &mut Vec<Region>) {
    let m = ctx.m;
    if tris.len() < 12 {
        return;
    }
    let verts = unique_verts(m, tris);
    let pts: Vec<V3> = verts.iter().map(|&v| m.verts[v as usize]).collect();
    let mut c = V3::ZERO;
    for p in &pts {
        c += *p;
    }
    c = c / pts.len() as f64;
    let mut cov = crate::math::M3::zero();
    for p in &pts {
        cov.add_outer(*p - c, 1.0);
    }
    let (_, ev) = cov.eigen_sym();
    // Bezugsrichtung: mittlere Facettennormale (robuster als kleinste Hauptachse)
    let mut nsum = V3::ZERO;
    for &t in tris {
        nsum += m.tri_n[t as usize] * m.tri_a[t as usize];
    }
    let mut n = if nsum.norm() > 1e-12 { nsum.normalized() } else { ev[0] };
    if n.dot(ev[0]).abs() > 0.9 {
        n = if ev[0].dot(n) < 0.0 { -ev[0] } else { ev[0] };
    }
    let mut e1 = ev[2] - n * ev[2].dot(n);
    if e1.norm() < 1e-9 {
        e1 = n.any_perp();
    }
    let e1 = e1.normalized();
    let e2 = n.cross(e1).normalized();
    // Höhenfeld-Bedingung: alle Facetten zeigen grob in Richtung n
    // Sehr schmale Streifen (z. B. Fasen entlang gekrümmter Kanten) bleiben facettiert:
    // CAD-Kernel bilden darauf keine zuverlässigen Parameterkurven
    {
        let (mut a0, mut a1, mut b0, mut b1) = (f64::MAX, f64::MIN, f64::MAX, f64::MIN);
        for p in &pts {
            let d = *p - c;
            a0 = a0.min(d.dot(e1));
            a1 = a1.max(d.dot(e1));
            b0 = b0.min(d.dot(e2));
            b1 = b1.max(d.dot(e2));
        }
        if (a1 - a0).min(b1 - b0) < m.diag() * 0.005 {
            return;
        }
    }
    let height_field = tris.iter().all(|&t| m.tri_a[t as usize] < ctx.min_area || m.tri_n[t as usize].dot(n) > 0.5);
    let dbg = std::env::var("M2S_DEBUG").is_ok();
    let mut ok = None;
    if height_field {
        // Kantenmitten als Kontrolle gegen Überschwinger: Randkanten werden zu B-Rep-Kanten
        // (Sehne muss auf der Fläche liegen), innere Kanten nur grob prüfen
        let inset: std::collections::HashSet<u32> = tris.iter().copied().collect();
        let mut mids = Vec::with_capacity(tris.len() * 2);
        let mut bmids = Vec::new();
        for &t in tris {
            for k in 0..3 {
                let h = (3 * t + k) as usize;
                let (x, y) = (m.he_from(h as u32), m.he_to(h as u32));
                let mid = (m.verts[x as usize] + m.verts[y as usize]) * 0.5;
                let tw = m.twin[h];
                if tw == NONE || !inset.contains(&(tw / 3)) {
                    bmids.push(mid);
                } else if x < y {
                    mids.push(mid);
                }
            }
        }
        let mtol = ff_tol * 4.0;
        // Randecken: hier entstehen die B-Rep-Kanten, die eng auf der Fläche liegen müssen
        let mut bv: Vec<u32> = Vec::new();
        for &t in tris {
            for k in 0..3 {
                let h = (3 * t + k) as usize;
                let tw = m.twin[h];
                if tw == NONE || !inset.contains(&(tw / 3)) {
                    bv.push(m.he_from(h as u32));
                    bv.push(m.he_to(h as u32));
                }
            }
        }
        bv.sort_unstable();
        bv.dedup();
        let bpts: Vec<V3> = bv.iter().map(|&v| m.verts[v as usize]).collect();
        let bvtol = crate::brep::ff_edge_tol_of(ctx.tol) * 0.4;
        if let Some(bs) = crate::bspline::fit(&pts, &mids, &bmids, c, e1, e2, n, ff_tol, mtol, ff_tol * 1.8, &bpts, bvtol) {
            let id = crate::bspline::register(bs);
            let s = Surf::BSpline { id };
            // Normalen prüfen (gleiche Kriterien wie bei Grundformen, aber Freiform-Toleranz)
            let fctx = Ctx { m, tol: ff_tol, cos_smooth: ctx.cos_smooth, smooth: ctx.smooth, min_area: ctx.min_area };
            let mut sg = 0.0;
            let v = fctx.verify(&s, &verts, tris, &mut sg);
            if dbg {
                eprintln!("ff depth {depth}: {} tris fit ok, verify {v}", tris.len());
            }
            if v {
                ok = Some(s);
            }
        } else if dbg {
            eprintln!("ff depth {depth}: {} tris fit FAILED", tris.len());
        }
    } else if dbg {
        let bad = tris.iter().filter(|&&t| m.tri_a[t as usize] >= ctx.min_area && m.tri_n[t as usize].dot(n) <= 0.5).count();
        eprintln!("ff depth {depth}: {} tris kein Höhenfeld ({bad} abgewandt)", tris.len());
    }
    if let Some(s) = ok {
        out.push(Region { tris: tris.to_vec(), surf: s });
        return;
    }
    if depth >= 5 {
        return;
    }
    if !height_field {
        // Kein Höhenfeld (z. B. Oberseite + Seitenwand): nach Normalenrichtung trennen
        let parts = split_by_normals(m, tris);
        if parts.len() > 1 {
            for p in parts {
                fit_freeform(ctx, &p, ff_tol, depth + 1, out);
            }
            return;
        }
    }
    // Entlang der längsten Ausdehnung teilen
    let mut proj: Vec<(f64, u32)> = tris.iter().map(|&t| (m.tri_centroid(t as usize).dot(e1), t)).collect();
    proj.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
    let half = proj.len() / 2;
    let a: Vec<u32> = proj[..half].iter().map(|x| x.1).collect();
    let b: Vec<u32> = proj[half..].iter().map(|x| x.1).collect();
    for part in [a, b] {
        for c in connected_parts(m, &part) {
            fit_freeform(ctx, &c, ff_tol, depth + 1, out);
        }
    }
}

/// Zerlegt eine Dreiecksmenge in zusammenhängende Teile.
fn connected_parts(m: &Mesh, tris: &[u32]) -> Vec<Vec<u32>> {
    let set: std::collections::HashSet<u32> = tris.iter().copied().collect();
    let mut seen: std::collections::HashSet<u32> = std::collections::HashSet::new();
    let mut out = Vec::new();
    for &t0 in tris {
        if seen.contains(&t0) {
            continue;
        }
        let mut list = Vec::new();
        let mut stack = vec![t0];
        seen.insert(t0);
        while let Some(t) = stack.pop() {
            list.push(t);
            for k in 0..3 {
                let tw = m.twin[(3 * t + k) as usize];
                if tw != NONE && set.contains(&(tw / 3)) && seen.insert(tw / 3) {
                    stack.push(tw / 3);
                }
            }
        }
        out.push(list);
    }
    out
}

/// 2-Means-Clustering der Facettennormalen, danach zusammenhängende Teile.
fn split_by_normals(m: &Mesh, tris: &[u32]) -> Vec<Vec<u32>> {
    // Startwerte: die zwei am stärksten verschiedenen Normalen (Näherung)
    let n0 = m.tri_n[tris[0] as usize];
    let far = |r: V3| *tris.iter().min_by(|&&a, &&b| m.tri_n[a as usize].dot(r).partial_cmp(&m.tri_n[b as usize].dot(r)).unwrap()).unwrap();
    let mut c1 = m.tri_n[far(n0) as usize];
    let mut c2 = m.tri_n[far(c1) as usize];
    let mut lab = vec![false; tris.len()];
    for _ in 0..15 {
        let (mut s1, mut s2) = (V3::ZERO, V3::ZERO);
        for (i, &t) in tris.iter().enumerate() {
            let n = m.tri_n[t as usize];
            lab[i] = n.dot(c2) > n.dot(c1);
            if lab[i] {
                s2 += n * m.tri_a[t as usize];
            } else {
                s1 += n * m.tri_a[t as usize];
            }
        }
        if s1.norm() < 1e-12 || s2.norm() < 1e-12 {
            return vec![tris.to_vec()];
        }
        c1 = s1.normalized();
        c2 = s2.normalized();
    }
    let a: Vec<u32> = tris.iter().zip(&lab).filter(|(_, &l)| !l).map(|(&t, _)| t).collect();
    let b: Vec<u32> = tris.iter().zip(&lab).filter(|(_, &l)| l).map(|(&t, _)| t).collect();
    let mut out = connected_parts(m, &a);
    out.extend(connected_parts(m, &b));
    out
}

/// Einzelne Dreiecke (plus `rings` Nachbarringe innerhalb derselben Region) als Facetten herauslösen.
pub fn demote_tris(m: &Mesh, seg: &mut Segmentation, tris: &[u32], rings: usize) {
    let mut set: std::collections::HashSet<u32> = tris.iter().copied().collect();
    for _ in 0..rings {
        let cur: Vec<u32> = set.iter().copied().collect();
        for t in cur {
            let r = seg.region_of[t as usize];
            for k in 0..3 {
                let tw = m.twin[(3 * t + k) as usize];
                if tw != NONE && seg.region_of[(tw / 3) as usize] == r {
                    set.insert(tw / 3);
                }
            }
        }
    }
    for t in set {
        let n = if m.tri_n[t as usize] == V3::ZERO { V3::Z } else { m.tri_n[t as usize] };
        seg.region_of[t as usize] = seg.regions.len() as u32;
        seg.regions.push(Region { tris: vec![t], surf: Surf::Plane { o: m.tri_centroid(t as usize), n } });
    }
    rebuild_lists(seg);
    // Region kann durch das Herauslösen zerfallen
    let ctx_dummy = ();
    let _ = ctx_dummy;
    split_components_pub(m, seg);
}

/// Öffentliche Variante von split_components (ohne Kontext).
pub fn split_components_pub(m: &Mesh, seg: &mut Segmentation) {
    let mut new_regions: Vec<Region> = Vec::new();
    let mut region_of = vec![NONE; m.tris.len()];
    let mut stack = Vec::new();
    for r in &seg.regions {
        for &t0 in &r.tris {
            if region_of[t0 as usize] != NONE {
                continue;
            }
            let id = new_regions.len() as u32;
            let mut tris = Vec::new();
            region_of[t0 as usize] = id;
            stack.push(t0);
            let old = seg.region_of[t0 as usize];
            while let Some(t) = stack.pop() {
                tris.push(t);
                for k in 0..3 {
                    let tw = m.twin[(3 * t + k) as usize];
                    if tw == NONE {
                        continue;
                    }
                    let u = tw / 3;
                    if region_of[u as usize] == NONE && seg.region_of[u as usize] == old {
                        region_of[u as usize] = id;
                        stack.push(u);
                    }
                }
            }
            new_regions.push(Region { tris, surf: r.surf });
        }
    }
    seg.regions = new_regions;
    seg.region_of = region_of;
}


#[cfg(test)]
mod tests {
    use super::*;
    use crate::io::RawMesh;
    use crate::math::v3;

    /// Band um eine Achse; die Ecken liegen auf einem Zylinder, aber nur gehäuft an „Ecken“
    /// (gerundetes Vieleck). Die langen geraden Seiten liegen weit innerhalb des Zylinders –
    /// das ist kein Zylinder, auch wenn alle Ecken und Normalen passen.
    fn band(angles: &[f64]) -> Mesh {
        let mut verts = Vec::new();
        for &a in angles {
            let (s, c) = a.to_radians().sin_cos();
            verts.push(v3(5.0 * c, 5.0 * s, 0.0));
            verts.push(v3(5.0 * c, 5.0 * s, 1.0));
        }
        let n = angles.len() as u32;
        let mut tris = Vec::new();
        for i in 0..n {
            let j = (i + 1) % n;
            tris.push([2 * i, 2 * j, 2 * j + 1]);
            tris.push([2 * i, 2 * j + 1, 2 * i + 1]);
        }
        crate::mesh::build(&RawMesh { verts, tris, ..Default::default() }).unwrap()
    }

    fn passt(m: &Mesh) -> bool {
        let smooth = 40f64.to_radians();
        let ctx = Ctx { m, tol: 0.01, cos_smooth: smooth.cos(), smooth, min_area: (m.diag() * 1e-6).powi(2) };
        let s = Surf::Cyl { p: v3(0.0, 0.0, 0.0), a: v3(0.0, 0.0, 1.0), r: 5.0 };
        let verts: Vec<u32> = (0..m.verts.len() as u32).collect();
        let tris: Vec<u32> = (0..m.tris.len() as u32).collect();
        let mut sign = 0.0;
        ctx.verify(&s, &verts, &tris, &mut sign)
    }

    #[test]
    fn gerundetes_vieleck_ist_kein_zylinder() {
        // 9 Ecken alle 40°, je Ecke drei Punkte (±3°)
        let mut ang = Vec::new();
        for k in 0..9 {
            for d in [-3.0, 0.0, 3.0] {
                ang.push(k as f64 * 40.0 + d);
            }
        }
        assert!(!passt(&band(&ang)));
    }

    #[test]
    fn grob_vernetzter_zylinder_bleibt_zylinder() {
        // gleichmäßig grob (24 Segmente, Sehnenfehler ≈ 0,04 mm > 3 × Toleranz)
        let ang: Vec<f64> = (0..24).map(|i| i as f64 * 15.0).collect();
        assert!(passt(&band(&ang)));
    }
}
