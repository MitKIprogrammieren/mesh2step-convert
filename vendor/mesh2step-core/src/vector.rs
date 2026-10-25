//! Vektorix: Bilder (Logos, Grafiken) in Vektorgrafiken umwandeln.
//! Farbquantisierung, Bereinigung, Konturverfolgung, Kurvenanpassung (Bézier,
//! Kreise, Ellipsen) und Export als SVG, PDF, EPS, DXF, PNG und 3MF.

use std::collections::HashMap;

// ---------------------------------------------------------------------------
// Grundtypen

pub type Pt = [f64; 2];

#[derive(Clone, Debug)]
pub enum Cmd {
    M(f64, f64),
    L(f64, f64),
    C(f64, f64, f64, f64, f64, f64),
    Q(f64, f64, f64, f64),
    Z,
}

/// Eine Teilform: Pfad oder Ellipse (cx, cy, rx, ry, Drehung in Grad).
#[derive(Clone, Debug)]
pub enum Shape {
    Path(Vec<Cmd>),
    Ellipse([f64; 5]),
}

#[derive(Clone, Debug)]
pub struct Layer {
    /// Gruppe (Ausgabefarbe)
    pub idx: usize,
    pub subs: Vec<Shape>,
}

#[derive(Clone, Debug)]
pub struct Group {
    /// erkannte Quelltöne
    pub src: Vec<usize>,
    /// vom Nutzer gesetzte Farbe
    pub over: Option<[u8; 3]>,
    pub transp: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Unit {
    None,
    Px,
    Pt,
    In,
    Cm,
    Mm,
}

#[derive(Clone, Debug)]
pub struct Opts {
    /// Analyse-Auflösung (längste Seite)
    pub res: u32,
    pub noise: bool,
    pub halo: bool,
    /// Konturglättung (0 = aus)
    pub smooth: f64,
    /// Pfad-Genauigkeit in Bildpixeln
    pub tol: f64,
    /// Mindestfläche in Bildpixeln²
    pub min_area: f64,
    /// Ecken-Schwelle in Grad
    pub corner: f64,
    /// Formen übereinander stapeln (sonst Aussparungen)
    pub stack: bool,
    pub group_by_color: bool,
    pub quad: bool,
    pub cubic: bool,
    pub circles: bool,
    pub ellipses: bool,
    pub gap: bool,
    pub gap_w: f64,
    pub gap_ns: bool,
    pub stroke: bool,
    pub stroke_w: f64,
    pub stroke_color: [u8; 3],
    pub stroke_only: bool,
    pub scale: f64,
    pub unit: Unit,
    pub dpi: f64,
    pub svg_tiny: bool,
    pub aa: bool,
    /// Flächen beruhigen (kantenerhaltende Glättung vor der Farbsuche): 0 = aus, 1…3 Stärke
    pub calm: f64,
    /// Gerade Kanten begradigen
    pub straight: bool,
    /// Spiegelsymmetrische Formen erkennen und exakt symmetrisch machen
    pub symmetry: bool,
    /// Kanten subpixelgenau aus den Kantenpixeln des Bildes bestimmen
    pub subpx: bool,
}

impl Default for Opts {
    fn default() -> Self {
        Opts {
            res: 1200,
            noise: true,
            halo: true,
            smooth: 1.0,
            tol: 0.3,
            min_area: 3.0,
            corner: 55.0,
            stack: false,
            group_by_color: true,
            quad: true,
            cubic: true,
            circles: true,
            ellipses: true,
            gap: true,
            gap_w: 1.0,
            gap_ns: true,
            stroke: false,
            stroke_w: 1.0,
            stroke_color: [0, 0, 0],
            stroke_only: false,
            scale: 1.0,
            unit: Unit::Px,
            dpi: 96.0,
            svg_tiny: false,
            aa: true,
            calm: 0.0,
            straight: true,
            symmetry: true,
            subpx: true,
        }
    }
}

/// Ergebnis der Farbquantisierung.
#[derive(Clone, Debug, Default)]
pub struct Quant {
    pub labels: Vec<i16>,
    pub pal: Vec<[u8; 3]>,
    pub cnt: Vec<usize>,
    /// presets[k] = Gruppen (Quelltöne) für k Farben, k = 1..=pal.len()
    pub presets: Vec<Vec<Vec<usize>>>,
    pub auto: usize,
}

#[derive(Clone, Copy, Debug)]
pub struct Edit {
    pub x: usize,
    pub y: usize,
    /// Zielgruppe, -1 = transparent
    pub to: i32,
}

/// Arbeitszustand eines Bildes.
#[derive(Clone, Debug, Default)]
pub struct Doc {
    pub src_w: usize,
    pub src_h: usize,
    pub src: Vec<u8>,
    /// Analysebild
    pub w: usize,
    pub h: usize,
    pub sc: f64,
    pub data: Vec<u8>,
    pub q: Quant,
    pub groups: Vec<Group>,
    pub k: usize,
    pub edits: Vec<Edit>,
    pub redo: Vec<Edit>,
    /// Gruppe je Analysepixel (-1 = transparent)
    pub g: Vec<i16>,
    pub layers: Vec<Layer>,
    /// Anzahl symmetrisch gemachter Formen der letzten Vektorisierung
    pub nsym: usize,
}

fn wd(a: [f64; 3], b: [f64; 3]) -> f64 {
    let (x, y, z) = (a[0] - b[0], a[1] - b[1], a[2] - b[2]);
    2.0 * x * x + 4.0 * y * y + 3.0 * z * z
}

fn js_round(v: f64) -> f64 {
    (v + 0.5).floor()
}

pub fn hex(c: [u8; 3]) -> String {
    format!("#{:02x}{:02x}{:02x}", c[0], c[1], c[2])
}

struct Rng(i64);
impl Rng {
    fn next(&mut self) -> f64 {
        self.0 = (self.0 * 16807) % 2147483647;
        self.0 as f64 / 2147483647.0
    }
}

// ---------------------------------------------------------------------------
// Pipeline

impl Doc {
    pub fn new(w: usize, h: usize, rgba: Vec<u8>) -> Doc {
        Doc { src_w: w, src_h: h, src: rgba, ..Default::default() }
    }

    /// Stufe 0: Analysebild erzeugen (bis 4x hochgerechnet).
    pub fn rasterize(&mut self, o: &Opts) {
        let max = o.res as f64;
        let sc = (max / self.src_w.max(self.src_h).max(1) as f64).min(4.0);
        let w = ((self.src_w as f64 * sc).round() as usize).max(1);
        let h = ((self.src_h as f64 * sc).round() as usize).max(1);
        self.data = resample(&self.src, self.src_w, self.src_h, w, h);
        if o.calm > 0.0 {
            // Kantenerhaltende Glättung: schwache Muster (Raster, Waben, Rauschen) verschwinden,
            // kontrastreiche Kanten und feine Linien bleiben
            let r = 2 + (o.calm as usize).min(3);
            let sr = 12.0 + 8.0 * o.calm;
            self.data = bilateral(&self.data, w, h, r, sr);
            if o.calm >= 2.0 {
                self.data = bilateral(&self.data, w, h, r, sr);
            }
        }
        self.w = w;
        self.h = h;
        self.sc = w as f64 / self.src_w.max(1) as f64;
    }

    /// Stufe 1: Farben quantisieren und Palette nach Farbanzahl setzen (0 = automatisch).
    pub fn quantize(&mut self, k: usize) {
        self.q = quantize(&self.data, self.w, self.h);
        let kk = if k == 0 { self.q.auto } else { k };
        self.set_preset(kk);
    }

    pub fn set_preset(&mut self, k: usize) {
        if self.q.pal.is_empty() {
            self.groups.clear();
            self.k = 0;
            return;
        }
        let k = k.clamp(1, self.q.pal.len());
        self.k = k;
        self.groups = self.q.presets[k].iter().map(|m| Group { src: m.clone(), over: None, transp: false }).collect();
        let cnt = &self.q.cnt;
        let gw = |g: &Group| g.src.iter().map(|&i| cnt.get(i).copied().unwrap_or(0)).sum::<usize>();
        self.groups.sort_by(|a, b| gw(b).cmp(&gw(a)));
    }

    pub fn group_weight(&self, g: &Group) -> usize {
        g.src.iter().map(|&i| self.q.cnt.get(i).copied().unwrap_or(0)).sum()
    }

    /// Ausgabefarbe einer Gruppe.
    pub fn gcol(&self, gi: usize) -> [u8; 3] {
        let g = &self.groups[gi];
        if let Some(c) = g.over {
            return c;
        }
        let (mut r, mut gg, mut b, mut w) = (0.0, 0.0, 0.0, 0.0);
        for &i in &g.src {
            let c = self.q.pal[i];
            let k = if self.q.cnt[i] == 0 { 1.0 } else { self.q.cnt[i] as f64 };
            r += c[0] as f64 * k;
            gg += c[1] as f64 * k;
            b += c[2] as f64 * k;
            w += k;
        }
        if w == 0.0 {
            return [0, 0, 0];
        }
        [js_round(r / w) as u8, js_round(gg / w) as u8, js_round(b / w) as u8]
    }

    /// Stufe 2: Gruppen zuordnen, bereinigen, Bearbeitungen anwenden.
    pub fn apply_map(&mut self, o: &Opts) {
        let (w, h) = (self.w, self.h);
        let n = w * h;
        let mut map = vec![-1i16; self.q.pal.len()];
        for (gi, g) in self.groups.iter().enumerate() {
            for &i in &g.src {
                map[i] = if g.transp { -1 } else { gi as i16 };
            }
        }
        let mut gm = vec![0i16; n];
        for p in 0..n {
            let l = self.q.labels[p];
            gm[p] = if l < 0 { -1 } else { map[l as usize] };
        }
        if o.noise {
            let lim = if self.sc >= 1.5 { 2 } else { 1 };
            let reps = if self.sc >= 1.5 { 3 } else { 2 };
            for _ in 0..reps {
                denoise(&mut gm, w, h, lim);
            }
        }
        if o.halo {
            let cols: Vec<[f64; 3]> = (0..self.groups.len())
                .map(|i| {
                    let c = self.gcol(i);
                    [c[0] as f64, c[1] as f64, c[2] as f64]
                })
                .collect();
            dehalo(&mut gm, w, h, &cols, &self.data, self.sc);
        }
        if o.smooth > 0.0 {
            let r = (js_round(o.smooth * self.sc) as usize).max(1);
            majority(&mut gm, w, h, r);
        }
        if o.calm > 0.0 {
            let amin = (o.calm * 12.0 * self.sc * self.sc).round() as usize;
            let cols: Vec<[f64; 3]> = (0..self.groups.len())
                .map(|i| {
                    let c = self.gcol(i);
                    [c[0] as f64, c[1] as f64, c[2] as f64]
                })
                .collect();
            merge_small(&mut gm, w, h, amin, &cols);
        }
        for e in &self.edits {
            if e.x >= w || e.y >= h || e.to >= self.groups.len() as i32 {
                continue;
            }
            let (px, _) = flood(&gm, w, h, e.x, e.y);
            let to = if e.to >= 0 && self.groups[e.to as usize].transp { -1 } else { e.to as i16 };
            for p in px {
                gm[p as usize] = to;
            }
        }
        self.g = gm;
    }

    /// Stufe 3: Konturen verfolgen und Kurven anpassen.
    pub fn vectorize(&mut self, o: &Opts) {
        SYM_COUNT.store(0, std::sync::atomic::Ordering::Relaxed);
        self.layers = vectorize_shared(&self.g, &self.data, self.w, self.h, self.groups.len(), self.sc, o, o.stack);
        self.nsym = SYM_COUNT.load(std::sync::atomic::Ordering::Relaxed);
    }

    /// Hintergrundfarbe: die Gruppe, die den Bildrand am meisten berührt.
    pub fn background_group(&self) -> Option<usize> {
        let (w, h) = (self.w, self.h);
        if self.g.len() != w * h || w == 0 || h == 0 {
            return None;
        }
        let mut cnt = vec![0usize; self.groups.len()];
        let mut add = |p: usize| {
            let v = self.g[p];
            if v >= 0 && (v as usize) < cnt.len() {
                cnt[v as usize] += 1;
            }
        };
        for x in 0..w {
            add(x);
            add((h - 1) * w + x);
        }
        for y in 0..h {
            add(y * w);
            add(y * w + w - 1);
        }
        let (i, &c) = cnt.iter().enumerate().max_by_key(|x| (*x.1, std::cmp::Reverse(x.0)))?;
        if c * 2 >= w + h {
            Some(i)
        } else {
            None
        }
    }

    /// Alles ab einer Stufe neu berechnen (0 = Rasterung, 1 = Farben, 2 = Zuordnung, 3 = Pfade).
    pub fn run(&mut self, stage: u8, o: &Opts, k: usize) {
        if stage == 0 {
            self.rasterize(o);
        }
        if stage <= 1 {
            self.quantize(k);
        }
        if stage <= 2 {
            self.apply_map(o);
        }
        self.vectorize(o);
    }

    /// Fläche unter einem Analysepixel: (Pixel, Gruppe, Nachbargruppen nach Häufigkeit).
    pub fn region_at(&self, x: usize, y: usize) -> Option<(Vec<u32>, i16, Vec<i16>)> {
        if x >= self.w || y >= self.h || self.g.is_empty() {
            return None;
        }
        let (px, mask) = flood(&self.g, self.w, self.h, x, y);
        let v = self.g[y * self.w + x];
        let mut nb: HashMap<i16, usize> = HashMap::new();
        let w = self.w;
        for &p in &px {
            let p = p as usize;
            let (xx, yy) = (p % w, p / w);
            let ns = [
                if xx > 0 { Some(p - 1) } else { None },
                if xx < w - 1 { Some(p + 1) } else { None },
                if yy > 0 { Some(p - w) } else { None },
                if yy < self.h - 1 { Some(p + w) } else { None },
            ];
            for q in ns.into_iter().flatten() {
                if !mask[q] {
                    *nb.entry(self.g[q]).or_default() += 1;
                }
            }
        }
        let mut nbs: Vec<(i16, usize)> = nb.into_iter().collect();
        nbs.sort_by(|a, b| b.1.cmp(&a.1).then(a.0.cmp(&b.0)));
        Some((px, v, nbs.into_iter().map(|x| x.0).collect()))
    }
}

/// Bilateraler Filter (räumlich Radius r, Farbabstand-Glättung sr).
pub fn bilateral(src: &[u8], w: usize, h: usize, r: usize, sr: f64) -> Vec<u8> {
    let ss = r as f64 / 1.5;
    let mut sw = vec![0f64; (2 * r + 1) * (2 * r + 1)];
    for dy in 0..=2 * r {
        for dx in 0..=2 * r {
            let (x, y) = (dx as f64 - r as f64, dy as f64 - r as f64);
            sw[dy * (2 * r + 1) + dx] = (-(x * x + y * y) / (2.0 * ss * ss)).exp();
        }
    }
    let mut lut = vec![0f64; 3 * 255 * 255 + 1];
    for (i, v) in lut.iter_mut().enumerate() {
        *v = (-(i as f64) / (2.0 * sr * sr * 3.0)).exp();
    }
    let mut out = src.to_vec();
    for y in 0..h {
        for x in 0..w {
            let j = (y * w + x) * 4;
            if src[j + 3] < 128 {
                continue;
            }
            let c = [src[j] as i32, src[j + 1] as i32, src[j + 2] as i32];
            let mut acc = [0f64; 3];
            let mut ws = 0.0;
            for dy in 0..=2 * r {
                let yy = y as isize + dy as isize - r as isize;
                if yy < 0 || yy >= h as isize {
                    continue;
                }
                for dx in 0..=2 * r {
                    let xx = x as isize + dx as isize - r as isize;
                    if xx < 0 || xx >= w as isize {
                        continue;
                    }
                    let k = (yy as usize * w + xx as usize) * 4;
                    if src[k + 3] < 128 {
                        continue;
                    }
                    let d = [src[k] as i32 - c[0], src[k + 1] as i32 - c[1], src[k + 2] as i32 - c[2]];
                    let dd = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]) as usize;
                    let wgt = sw[dy * (2 * r + 1) + dx] * lut[dd.min(lut.len() - 1)];
                    acc[0] += wgt * src[k] as f64;
                    acc[1] += wgt * src[k + 1] as f64;
                    acc[2] += wgt * src[k + 2] as f64;
                    ws += wgt;
                }
            }
            for q in 0..3 {
                out[j + q] = (acc[q] / ws).round().clamp(0.0, 255.0) as u8;
            }
        }
    }
    out
}

/// Kleine zusammenhängende Flecken (Raster, Wabenmuster, Rauschen) der Nachbarfläche mit der
/// längsten gemeinsamen Grenze zuschlagen. Schmale, lange Formen wie Schrift haben genug Fläche
/// und bleiben erhalten.
pub fn merge_small(g: &mut [i16], w: usize, h: usize, amin: usize, cols: &[[f64; 3]]) {
    if amin < 2 {
        return;
    }
    for _pass in 0..4 {
        let n = w * h;
        let mut comp = vec![u32::MAX; n];
        let mut comps: Vec<(i16, Vec<u32>)> = Vec::new();
        let mut stack = Vec::new();
        for s0 in 0..n {
            if comp[s0] != u32::MAX {
                continue;
            }
            let id = comps.len() as u32;
            let lab = g[s0];
            comp[s0] = id;
            stack.push(s0 as u32);
            let mut px = Vec::new();
            while let Some(p) = stack.pop() {
                px.push(p);
                let p = p as usize;
                let (x, y) = (p % w, p / w);
                let mut push = |q: usize| {
                    if comp[q] == u32::MAX && g[q] == lab {
                        comp[q] = id;
                        stack.push(q as u32);
                    }
                };
                if x > 0 { push(p - 1); }
                if x + 1 < w { push(p + 1); }
                if y > 0 { push(p - w); }
                if y + 1 < h { push(p + w); }
            }
            comps.push((lab, px));
        }
        let mut changed = 0usize;
        // kleinste zuerst
        // Längliche Stücke (Striche dünner Schrift, feine Linien) nicht wegmischen
        let elongated = |px: &Vec<u32>| {
            let (mut x0, mut x1, mut y0, mut y1) = (usize::MAX, 0, usize::MAX, 0);
            for &p in px {
                let (x, y) = (p as usize % w, p as usize / w);
                x0 = x0.min(x);
                x1 = x1.max(x);
                y0 = y0.min(y);
                y1 = y1.max(y);
            }
            let ext = (x1 - x0 + 1).max(y1 - y0 + 1) as f64;
            ext >= 3.5 * (px.len() as f64).sqrt()
        };
        let mut order: Vec<usize> = (0..comps.len()).filter(|&c| comps[c].1.len() < amin && comps[c].0 >= 0 && !elongated(&comps[c].1)).collect();
        order.sort_by_key(|&c| comps[c].1.len());
        for c in order {
            let mut nb: HashMap<i16, usize> = HashMap::new();
            for &p in &comps[c].1 {
                let p = p as usize;
                let (x, y) = (p % w, p / w);
                let mut look = |q: usize| {
                    let l = g[q];
                    if comp[q] != c as u32 && l >= 0 {
                        *nb.entry(l).or_default() += 1;
                    }
                };
                if x > 0 { look(p - 1); }
                if x + 1 < w { look(p + 1); }
                if y > 0 { look(p - w); }
                if y + 1 < h { look(p + w); }
            }
            // ähnlichste Nachbarfarbe (bei Schrift: der Strich, nicht der Hintergrund),
            // bei Gleichstand die längere gemeinsame Grenze
            let me = comps[c].0;
            let score = |l: i16, len: usize| {
                let d = match (cols.get(me as usize), cols.get(l as usize)) {
                    (Some(a), Some(b)) => wd(*a, *b),
                    _ => 0.0,
                };
                d / (1.0 + len as f64).sqrt()
            };
            if let Some((&to, _)) = nb.iter().min_by(|a, b| score(*a.0, *a.1).partial_cmp(&score(*b.0, *b.1)).unwrap().then(a.0.cmp(b.0))) {
                for &p in &comps[c].1 {
                    g[p as usize] = to;
                }
                changed += 1;
            }
        }
        if changed == 0 {
            break;
        }
    }
}

/// Bild skalieren: verkleinern per Flächenmittel, vergrößern bilinear.
pub fn resample(src: &[u8], sw: usize, sh: usize, w: usize, h: usize) -> Vec<u8> {
    let mut out = vec![0u8; w * h * 4];
    if sw == 0 || sh == 0 {
        return out;
    }
    let fx = sw as f64 / w as f64;
    let fy = sh as f64 / h as f64;
    let px = |x: usize, y: usize, k: usize| src[(y * sw + x) * 4 + k] as f64;
    for y in 0..h {
        for x in 0..w {
            let o = (y * w + x) * 4;
            if fx > 1.0 || fy > 1.0 {
                // Flächenmittel (vormultipliziertes Alpha)
                let (x0, x1) = (x as f64 * fx, (x + 1) as f64 * fx);
                let (y0, y1) = (y as f64 * fy, (y + 1) as f64 * fy);
                let mut acc = [0.0f64; 4];
                let mut wsum = 0.0;
                let mut yy = y0.floor() as usize;
                while (yy as f64) < y1 && yy < sh {
                    let wy = ((yy + 1) as f64).min(y1) - (yy as f64).max(y0);
                    let mut xx = x0.floor() as usize;
                    while (xx as f64) < x1 && xx < sw {
                        let wx = ((xx + 1) as f64).min(x1) - (xx as f64).max(x0);
                        let wgt = wx * wy;
                        let a = px(xx, yy, 3) / 255.0;
                        for k in 0..3 {
                            acc[k] += px(xx, yy, k) * a * wgt;
                        }
                        acc[3] += a * wgt;
                        wsum += wgt;
                        xx += 1;
                    }
                    yy += 1;
                }
                if wsum > 0.0 && acc[3] > 0.0 {
                    for k in 0..3 {
                        out[o + k] = (acc[k] / acc[3]).round().clamp(0.0, 255.0) as u8;
                    }
                    out[o + 3] = (acc[3] / wsum * 255.0).round().clamp(0.0, 255.0) as u8;
                }
            } else {
                let sx = ((x as f64 + 0.5) * fx - 0.5).clamp(0.0, (sw - 1) as f64);
                let sy = ((y as f64 + 0.5) * fy - 0.5).clamp(0.0, (sh - 1) as f64);
                let (ix, iy) = (sx.floor() as usize, sy.floor() as usize);
                let (jx, jy) = ((ix + 1).min(sw - 1), (iy + 1).min(sh - 1));
                let (tx, ty) = (sx - ix as f64, sy - iy as f64);
                let a = |x: usize, y: usize| px(x, y, 3) / 255.0;
                let wts = [((ix, iy), (1.0 - tx) * (1.0 - ty)), ((jx, iy), tx * (1.0 - ty)), ((ix, jy), (1.0 - tx) * ty), ((jx, jy), tx * ty)];
                let mut acc = [0.0f64; 4];
                for ((xx, yy), wgt) in wts {
                    let al = a(xx, yy);
                    for k in 0..3 {
                        acc[k] += px(xx, yy, k) * al * wgt;
                    }
                    acc[3] += al * wgt;
                }
                if acc[3] > 0.0 {
                    for k in 0..3 {
                        out[o + k] = (acc[k] / acc[3]).round().clamp(0.0, 255.0) as u8;
                    }
                    out[o + 3] = (acc[3] * 255.0).round().clamp(0.0, 255.0) as u8;
                }
            }
        }
    }
    out
}

pub fn quantize(d: &[u8], w: usize, h: usize) -> Quant {
    let n = w * h;
    let mut rng = Rng(7);
    let step = (n / 40000).max(1);
    let mut sm: Vec<[f64; 3]> = Vec::new();
    let mut i = 0;
    while i < n {
        let j = i * 4;
        if d[j + 3] >= 128 {
            sm.push([d[j] as f64, d[j + 1] as f64, d[j + 2] as f64]);
        }
        i += step;
    }
    let m = sm.len();
    let mut q = Quant { labels: vec![-1; n], ..Default::default() };
    if m == 0 {
        q.presets = vec![Vec::new()];
        q.auto = 0;
        return q;
    }
    let kmax = 32.min(m);
    let mut c: Vec<[f64; 3]> = vec![sm[(rng.next() * m as f64).floor() as usize]];
    let mut dist = vec![f64::INFINITY; m];
    while c.len() < kmax {
        let last = *c.last().unwrap();
        let mut tot = 0.0;
        for i in 0..m {
            let dd = wd(sm[i], last);
            if dd < dist[i] {
                dist[i] = dd;
            }
            tot += dist[i];
        }
        if tot == 0.0 {
            break;
        }
        let mut t = rng.next() * tot;
        let mut k = 0;
        while k < m - 1 {
            t -= dist[k];
            if t <= 0.0 {
                break;
            }
            k += 1;
        }
        c.push(sm[k]);
    }
    let mut cnt: Vec<usize> = Vec::new();
    let iter = |c: &mut Vec<[f64; 3]>, cnt: &mut Vec<usize>, its: usize| {
        for _ in 0..its {
            let mut sum = vec![[0.0f64; 3]; c.len()];
            *cnt = vec![0; c.len()];
            for p in &sm {
                let mut best = 0;
                let mut bd = f64::INFINITY;
                for (k, ck) in c.iter().enumerate() {
                    let dd = wd(*p, *ck);
                    if dd < bd {
                        bd = dd;
                        best = k;
                    }
                }
                for q in 0..3 {
                    sum[best][q] += p[q];
                }
                cnt[best] += 1;
            }
            for k in 0..c.len() {
                if cnt[k] > 0 {
                    let n = cnt[k] as f64;
                    c[k] = [sum[k][0] / n, sum[k][1] / n, sum[k][2] / n];
                }
            }
        }
    };
    iter(&mut c, &mut cnt, 10);
    // fast gleiche Töne vereinen
    let mut ch = true;
    while ch && c.len() > 1 {
        ch = false;
        let (mut bd, mut a, mut b) = (f64::INFINITY, 0, 0);
        for i in 0..c.len() {
            for j in i + 1..c.len() {
                let dd = wd(c[i], c[j]);
                if dd < bd {
                    bd = dd;
                    a = i;
                    b = j;
                }
            }
        }
        if bd < 900.0 {
            let wa = if cnt[a] == 0 { 1.0 } else { cnt[a] as f64 };
            let wb = if cnt[b] == 0 { 1.0 } else { cnt[b] as f64 };
            c[a] = [(c[a][0] * wa + c[b][0] * wb) / (wa + wb), (c[a][1] * wa + c[b][1] * wb) / (wa + wb), (c[a][2] * wa + c[b][2] * wb) / (wa + wb)];
            cnt[a] = (wa + wb) as usize;
            c.remove(b);
            cnt.remove(b);
            ch = true;
        }
    }
    let keep: Vec<bool> = (0..c.len()).map(|k| cnt[k] as f64 / m as f64 >= 0.0005).collect();
    let mut c: Vec<[f64; 3]> = c.into_iter().zip(keep).filter(|x| x.1).map(|x| x.0).collect();
    iter(&mut c, &mut cnt, 3);
    let mut c: Vec<[f64; 3]> = c.into_iter().enumerate().filter(|(k, _)| cnt[*k] > 0).map(|x| x.1).collect();
    // Seltene, aber deutlich andere Farben (feine rote Linie, Lichtpunkte) nicht verlieren:
    // Pixel weit weg von allen Tönen sammeln und als eigene Töne aufnehmen
    {
        let mut far: Vec<[f64; 3]> = Vec::new();
        let mut p = 0;
        while p < n {
            let j = p * 4;
            if d[j + 3] >= 128 {
                let px = [d[j] as f64, d[j + 1] as f64, d[j + 2] as f64];
                let bd = c.iter().map(|ck| wd(px, *ck)).fold(f64::INFINITY, f64::min);
                if bd > 2500.0 {
                    far.push(px);
                }
            }
            p += 2;
        }
        let need = ((n / 2) as f64 * 0.00004).max(6.0) as usize;
        for _ in 0..16 {
            if far.len() < need || c.len() >= 48 {
                break;
            }
            // dichtester Kern: Mittelwert der Nachbarn des Pixels mit den meisten Nachbarn (Stichprobe)
            let step = (far.len() / 400).max(1);
            let (mut best, mut bn) = (0usize, 0usize);
            let mut i = 0;
            while i < far.len() {
                let k = far.iter().step_by(step).filter(|q| wd(**q, far[i]) < 1500.0).count();
                if k > bn {
                    bn = k;
                    best = i;
                }
                i += step;
            }
            let seed = far[best];
            let members: Vec<[f64; 3]> = far.iter().copied().filter(|q| wd(*q, seed) < 1500.0).collect();
            if members.len() < need {
                break;
            }
            let mut m3 = [0.0; 3];
            for q in &members {
                for k in 0..3 {
                    m3[k] += q[k];
                }
            }
            let mc = [m3[0] / members.len() as f64, m3[1] / members.len() as f64, m3[2] / members.len() as f64];
            c.push(mc);
            far.retain(|q| wd(*q, mc) >= 2500.0);
        }
    }
    let mut full = vec![0usize; c.len()];
    for p in 0..n {
        let j = p * 4;
        if d[j + 3] < 128 {
            continue;
        }
        let px = [d[j] as f64, d[j + 1] as f64, d[j + 2] as f64];
        let mut best = 0;
        let mut bd = f64::INFINITY;
        for (k, ck) in c.iter().enumerate() {
            let dd = wd(px, *ck);
            if dd < bd {
                bd = dd;
                best = k;
            }
        }
        q.labels[p] = best as i16;
        full[best] += 1;
    }
    let pal: Vec<[u8; 3]> = c.iter().map(|x| [js_round(x[0]) as u8, js_round(x[1]) as u8, js_round(x[2]) as u8]).collect();
    // Zusammenführungs-Baum für Voreinstellungen 1…n
    struct Cl {
        c: [f64; 3],
        w: f64,
        m: Vec<usize>,
    }
    let mut cl: Vec<Cl> = pal
        .iter()
        .enumerate()
        .map(|(i, p)| Cl { c: [p[0] as f64, p[1] as f64, p[2] as f64], w: if full[i] == 0 { 1.0 } else { full[i] as f64 }, m: vec![i] })
        .collect();
    let mut presets = vec![Vec::new(); pal.len() + 1];
    presets[cl.len()] = cl.iter().map(|x| x.m.clone()).collect();
    let mut auto = 0;
    let tot_w = (full.iter().sum::<usize>() as f64).max(1.0);
    while cl.len() > 1 {
        let (mut bd, mut a, mut b) = (f64::INFINITY, 0, 0);
        for i in 0..cl.len() {
            for j in i + 1..cl.len() {
                let f = if cl[i].w.min(cl[j].w) / tot_w < 0.012 { 0.15 } else { 1.0 };
                let dd = wd(cl[i].c, cl[j].c) * f;
                if dd < bd {
                    bd = dd;
                    a = i;
                    b = j;
                }
            }
        }
        let (aw, bw) = (cl[a].w, cl[b].w);
        if auto == 0 && wd(cl[a].c, cl[b].c) >= 15000.0 && aw.min(bw) / tot_w >= 0.012 {
            auto = cl.len();
        }
        let w = aw + bw;
        let bc = cl[b].c;
        let mut bm = cl[b].m.clone();
        let ac = cl[a].c;
        cl[a].c = [(ac[0] * aw + bc[0] * bw) / w, (ac[1] * aw + bc[1] * bw) / w, (ac[2] * aw + bc[2] * bw) / w];
        cl[a].w = w;
        cl[a].m.append(&mut bm);
        cl.remove(b);
        presets[cl.len()] = cl.iter().map(|x| x.m.clone()).collect();
    }
    if auto == 0 {
        auto = 1;
    }
    q.pal = pal;
    q.cnt = full;
    q.presets = presets;
    q.auto = auto;
    q
}

fn denoise(l: &mut [i16], w: usize, h: usize, lim: usize) {
    if w < 3 || h < 3 {
        return;
    }
    let src = l.to_vec();
    let mut cnt: Vec<(i16, usize)> = Vec::with_capacity(8);
    for y in 1..h - 1 {
        for x in 1..w - 1 {
            let p = y * w + x;
            let c = src[p];
            let mut same = 0;
            for dy in -1i64..=1 {
                for dx in -1i64..=1 {
                    if (dx != 0 || dy != 0) && src[(p as i64 + dy * w as i64 + dx) as usize] == c {
                        same += 1;
                    }
                }
            }
            if same <= lim {
                cnt.clear();
                let (mut best, mut bc) = (c, 0);
                for dy in -1i64..=1 {
                    for dx in -1i64..=1 {
                        if dx == 0 && dy == 0 {
                            continue;
                        }
                        let v = src[(p as i64 + dy * w as i64 + dx) as usize];
                        let k = match cnt.iter_mut().find(|e| e.0 == v) {
                            Some(e) => {
                                e.1 += 1;
                                e.1
                            }
                            None => {
                                cnt.push((v, 1));
                                1
                            }
                        };
                        if k > bc {
                            bc = k;
                            best = v;
                        }
                    }
                }
                l[p] = best;
            }
        }
    }
}

/// Mehrheitsfilter mit Radius r.
fn majority(g: &mut [i16], w: usize, h: usize, r: usize) {
    let n = w * h;
    let mut labels: Vec<i16> = Vec::new();
    let mut seen = std::collections::HashSet::new();
    for &v in g.iter() {
        if seen.insert(v) {
            labels.push(v);
        }
    }
    let mut integ = vec![0i32; (w + 1) * (h + 1)];
    let mut best = vec![0i32; n];
    let mut bl = vec![0i16; n];
    let mut own = vec![0i32; n];
    for &lab in &labels {
        for v in integ.iter_mut() {
            *v = 0;
        }
        for y in 0..h {
            let mut row = 0;
            for x in 0..w {
                row += (g[y * w + x] == lab) as i32;
                integ[(y + 1) * (w + 1) + x + 1] = integ[y * (w + 1) + x + 1] + row;
            }
        }
        for y in 0..h {
            let y0 = y.saturating_sub(r);
            let y1 = (y + r + 1).min(h);
            for x in 0..w {
                let x0 = x.saturating_sub(r);
                let x1 = (x + r + 1).min(w);
                let c = integ[y1 * (w + 1) + x1] - integ[y0 * (w + 1) + x1] - integ[y1 * (w + 1) + x0] + integ[y0 * (w + 1) + x0];
                let p = y * w + x;
                if g[p] == lab {
                    own[p] = c;
                }
                if c > best[p] {
                    best[p] = c;
                    bl[p] = lab;
                }
            }
        }
    }
    for p in 0..n {
        if best[p] > own[p] {
            g[p] = bl[p];
        }
    }
}

/// Kantensäume (Mischfarben an Rändern) den Nachbarn zuschlagen.
fn dehalo(g: &mut [i16], w: usize, h: usize, cols: &[[f64; 3]], data: &[u8], sc: f64) {
    let n = w * h;
    let mut comp = vec![-1i64; n];
    let thin_w = 1.8 * sc;
    let mut kill = vec![false; n];
    let mut any = false;
    let mut st: Vec<usize> = Vec::new();
    let nbrs = |p: usize| -> [Option<usize>; 4] {
        let (x, y) = (p % w, p / w);
        [
            if x > 0 { Some(p - 1) } else { None },
            if x < w - 1 { Some(p + 1) } else { None },
            if y > 0 { Some(p - w) } else { None },
            if y < h - 1 { Some(p + w) } else { None },
        ]
    };
    for p0 in 0..n {
        if comp[p0] >= 0 || g[p0] < 0 {
            continue;
        }
        let v = g[p0];
        let mut px: Vec<usize> = Vec::new();
        st.push(p0);
        comp[p0] = p0 as i64;
        let mut per = 0usize;
        let mut nb: Vec<(i16, usize)> = Vec::new();
        while let Some(p) = st.pop() {
            px.push(p);
            for q in nbrs(p).into_iter().flatten() {
                let gq = g[q];
                if gq == v {
                    if comp[q] < 0 {
                        comp[q] = p0 as i64;
                        st.push(q);
                    }
                } else {
                    per += 1;
                    match nb.iter_mut().find(|e| e.0 == gq) {
                        Some(e) => e.1 += 1,
                        None => nb.push((gq, 1)),
                    }
                }
            }
        }
        if per == 0 {
            continue;
        }
        let width = 2.0 * px.len() as f64 / per as f64;
        if width >= thin_w {
            continue;
        }
        let mut nbs: Vec<(i16, usize)> = nb.into_iter().filter(|e| e.0 >= 0).collect();
        nbs.sort_by(|a, b| b.1.cmp(&a.1));
        if nbs.is_empty() {
            continue;
        }
        let c = cols[v as usize];
        let a = cols[nbs[0].0 as usize];
        let blend = if nbs.len() > 1 {
            let b = cols[nbs[1].0 as usize];
            let ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
            let mut l2 = ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2];
            if l2 == 0.0 {
                l2 = 1.0;
            }
            let t = ((c[0] - a[0]) * ab[0] + (c[1] - a[1]) * ab[1] + (c[2] - a[2]) * ab[2]) / l2;
            let tc = t.clamp(0.0, 1.0);
            let d = ((c[0] - a[0] - tc * ab[0]).powi(2) + (c[1] - a[1] - tc * ab[1]).powi(2) + (c[2] - a[2] - tc * ab[2]).powi(2)).sqrt();
            t > 0.08 && t < 0.92 && d < 0.22 * l2.sqrt() + 10.0
        } else {
            false
        };
        if !blend && width >= 0.6 * sc {
            continue;
        }
        for p in px {
            kill[p] = true;
        }
        any = true;
    }
    if !any {
        return;
    }
    let mut front: Vec<usize> = (0..n).filter(|&p| kill[p]).collect();
    for _ in 0..64 {
        if front.is_empty() {
            break;
        }
        let mut next = Vec::new();
        let mut set: Vec<(usize, i16)> = Vec::new();
        for &p in &front {
            let j = p * 4;
            let px = [data[j] as f64, data[j + 1] as f64, data[j + 2] as f64];
            let mut best: i32 = -9;
            let mut bd = f64::INFINITY;
            for q in nbrs(p).into_iter().flatten() {
                if kill[q] {
                    continue;
                }
                let gq = g[q];
                if gq < 0 {
                    if bd == f64::INFINITY {
                        best = -1;
                    }
                    continue;
                }
                let dd = wd(px, cols[gq as usize]);
                if dd < bd {
                    bd = dd;
                    best = gq as i32;
                }
            }
            if best == -9 {
                next.push(p);
            } else {
                set.push((p, best as i16));
            }
        }
        for (p, b) in set {
            g[p] = b;
            kill[p] = false;
        }
        if next.len() == front.len() {
            break;
        }
        front = next;
    }
}

/// Zusammenhängende Fläche gleicher Gruppe (4er-Nachbarschaft).
pub fn flood(g: &[i16], w: usize, h: usize, x: usize, y: usize) -> (Vec<u32>, Vec<bool>) {
    let p0 = y * w + x;
    let v = g[p0];
    let mut seen = vec![false; w * h];
    let mut out = Vec::new();
    let mut st = vec![p0];
    seen[p0] = true;
    while let Some(p) = st.pop() {
        out.push(p as u32);
        let (px, py) = (p % w, p / w);
        let mut push = |q: usize| {
            if !seen[q] && g[q] == v {
                seen[q] = true;
                st.push(q);
            }
        };
        if px > 0 {
            push(p - 1);
        }
        if px < w - 1 {
            push(p + 1);
        }
        if py > 0 {
            push(p - w);
        }
        if py < h - 1 {
            push(p + w);
        }
    }
    (out, seen)
}

// ---------------------------------------------------------------------------
// Konturen

const DX: [i64; 4] = [1, 0, -1, 0];
const DY: [i64; 4] = [0, 1, 0, -1];

/// Ränder einer Maske als geschlossene Pixel-Eckpunktfolgen (x0,y0,x1,y1,…).
pub fn trace_mask(m: &[bool], w: usize, h: usize) -> Vec<Vec<i64>> {
    let vw = w + 1;
    let nv = vw * (h + 1);
    let mut o1 = vec![-1i8; nv];
    let mut o2 = vec![-1i8; nv];
    let add = |v: usize, d: i8, o1: &mut Vec<i8>, o2: &mut Vec<i8>| {
        if o1[v] < 0 {
            o1[v] = d;
        } else {
            o2[v] = d;
        }
    };
    for y in 0..h {
        for x in 0..w {
            let p = y * w + x;
            if !m[p] {
                continue;
            }
            if y == 0 || !m[p - w] {
                add(y * vw + x, 0, &mut o1, &mut o2);
            }
            if x == w - 1 || !m[p + 1] {
                add(y * vw + x + 1, 1, &mut o1, &mut o2);
            }
            if y == h - 1 || !m[p + w] {
                add((y + 1) * vw + x + 1, 2, &mut o1, &mut o2);
            }
            if x == 0 || !m[p - 1] {
                add((y + 1) * vw + x, 3, &mut o1, &mut o2);
            }
        }
    }
    let take = |v: usize, prev: i8, o1: &mut Vec<i8>, o2: &mut Vec<i8>| -> i8 {
        let pref = [(prev + 1) & 3, prev, (prev + 3) & 3];
        for d in pref {
            if o1[v] == d {
                o1[v] = -1;
                return d;
            }
            if o2[v] == d {
                o2[v] = -1;
                return d;
            }
        }
        if o1[v] >= 0 {
            let d = o1[v];
            o1[v] = -1;
            return d;
        }
        if o2[v] >= 0 {
            let d = o2[v];
            o2[v] = -1;
            return d;
        }
        -1
    };
    let mut loops = Vec::new();
    for v in 0..nv {
        while o1[v] >= 0 || o2[v] >= 0 {
            let mut d = if o1[v] >= 0 { o1[v] } else { o2[v] };
            if o1[v] == d {
                o1[v] = -1;
            } else {
                o2[v] = -1;
            }
            let sx = (v % vw) as i64;
            let sy = (v / vw) as i64;
            let (mut x, mut y) = (sx, sy);
            let mut pts = vec![x, y];
            x += DX[d as usize];
            y += DY[d as usize];
            let mut guard = 0;
            while !(x == sx && y == sy) && guard < 4_000_000 {
                guard += 1;
                pts.push(x);
                pts.push(y);
                let nd = take((y as usize) * vw + x as usize, d, &mut o1, &mut o2);
                if nd < 0 {
                    break;
                }
                d = nd;
                x += DX[d as usize];
                y += DY[d as usize];
            }
            loops.push(pts);
        }
    }
    loops
}

fn area_of(f: &[i64]) -> f64 {
    let n = f.len() / 2;
    let mut a = 0.0;
    for i in 0..n {
        let j = (i + 1) % n;
        a += (f[i * 2] * f[j * 2 + 1] - f[j * 2] * f[i * 2 + 1]) as f64;
    }
    a / 2.0
}

// ---------------------------------------------------------------------------
// Kurven

fn sub(a: Pt, b: Pt) -> Pt {
    [a[0] - b[0], a[1] - b[1]]
}
fn addp(a: Pt, b: Pt) -> Pt {
    [a[0] + b[0], a[1] + b[1]]
}
fn mulp(a: Pt, k: f64) -> Pt {
    [a[0] * k, a[1] * k]
}
fn dot(a: Pt, b: Pt) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}
fn nrm(a: Pt) -> Pt {
    let l = a[0].hypot(a[1]);
    if l > 0.0 {
        [a[0] / l, a[1] / l]
    } else {
        [1.0, 0.0]
    }
}

fn seg_d(p: Pt, a: Pt, b: Pt) -> f64 {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    let l = dx * dx + dy * dy;
    if l == 0.0 {
        return (p[0] - a[0]).hypot(p[1] - a[1]);
    }
    let t = (((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l).clamp(0.0, 1.0);
    (p[0] - a[0] - t * dx).hypot(p[1] - a[1] - t * dy)
}

fn rdp_open(a: &[Pt], eps: f64) -> Vec<Pt> {
    fn rec(a: &[Pt], s: usize, e: usize, eps: f64, out: &mut Vec<Pt>) {
        let (mut md, mut mi) = (0.0, usize::MAX);
        for i in s + 1..e {
            let d = seg_d(a[i], a[s], a[e]);
            if d > md {
                md = d;
                mi = i;
            }
        }
        if md > eps && mi != usize::MAX {
            rec(a, s, mi, eps, out);
            rec(a, mi, e, eps, out);
        } else {
            out.push(a[s]);
        }
    }
    let mut out = Vec::new();
    if a.len() < 2 {
        return a.to_vec();
    }
    rec(a, 0, a.len() - 1, eps, &mut out);
    out.push(a[a.len() - 1]);
    out
}

struct Ell {
    cx: f64,
    cy: f64,
    a: f64,
    b: f64,
    rot: f64,
    circle: bool,
}

fn fit_ellipse(mp: &[Pt], tol: f64) -> Option<Ell> {
    let n = mp.len() as f64;
    let (mut cx, mut cy) = (0.0, 0.0);
    for p in mp {
        cx += p[0];
        cy += p[1];
    }
    cx /= n;
    cy /= n;
    let (mut xx, mut yy, mut xy) = (0.0, 0.0, 0.0);
    for p in mp {
        let (u, v) = (p[0] - cx, p[1] - cy);
        xx += u * u;
        yy += v * v;
        xy += u * v;
    }
    xx /= n;
    yy /= n;
    xy /= n;
    let tr = xx + yy;
    let det = xx * yy - xy * xy;
    let disc = (tr * tr / 4.0 - det).max(0.0).sqrt();
    let (l1, l2) = (tr / 2.0 + disc, tr / 2.0 - disc);
    if l2 <= 0.0 {
        return None;
    }
    let th = 0.5 * (2.0 * xy).atan2(xx - yy);
    let (mut a, mut b) = ((2.0 * l1).sqrt(), (2.0 * l2).sqrt());
    if b < 4.0 {
        return None;
    }
    let (c, s) = (th.cos(), th.sin());
    let rs: Vec<f64> = mp
        .iter()
        .map(|p| {
            let u = (p[0] - cx) * c + (p[1] - cy) * s;
            let v = -(p[0] - cx) * s + (p[1] - cy) * c;
            (u / a).hypot(v / b)
        })
        .collect();
    let mean = rs.iter().sum::<f64>() / n;
    a *= mean;
    b *= mean;
    let mut maxdev: f64 = 0.0;
    for r in &rs {
        maxdev = maxdev.max((r / mean - 1.0).abs() * b);
    }
    if maxdev > (tol * 1.6 + 0.4).max(0.012 * b) {
        return None;
    }
    Some(Ell { cx, cy, a, b, rot: th.to_degrees(), circle: (a - b).abs() / a < 0.03 })
}

type Bez = [Pt; 4];

fn bz(q: &Bez, t: f64) -> Pt {
    let m = 1.0 - t;
    let (a, b, c, d) = (m * m * m, 3.0 * m * m * t, 3.0 * m * t * t, t * t * t);
    [
        a * q[0][0] + b * q[1][0] + c * q[2][0] + d * q[3][0],
        a * q[0][1] + b * q[1][1] + c * q[2][1] + d * q[3][1],
    ]
}

fn gen_bez(d: &[Pt], f: usize, l: usize, u: &[f64], t1: Pt, t2: Pt) -> Bez {
    let (p0, p3) = (d[f], d[l]);
    let (mut c00, mut c01, mut c11, mut x0, mut x1) = (0.0, 0.0, 0.0, 0.0, 0.0);
    for (i, &t) in u.iter().enumerate() {
        let m = 1.0 - t;
        let (b0, b1, b2, b3) = (m * m * m, 3.0 * m * m * t, 3.0 * m * t * t, t * t * t);
        let a0 = mulp(t1, b1);
        let a1 = mulp(t2, b2);
        c00 += dot(a0, a0);
        c01 += dot(a0, a1);
        c11 += dot(a1, a1);
        let tmp = sub(d[f + i], addp(mulp(p0, b0 + b1), mulp(p3, b2 + b3)));
        x0 += dot(a0, tmp);
        x1 += dot(a1, tmp);
    }
    let det = c00 * c11 - c01 * c01;
    let seg = (p3[0] - p0[0]).hypot(p3[1] - p0[1]);
    let (mut al, mut ar) = if det != 0.0 { ((x0 * c11 - x1 * c01) / det, (c00 * x1 - c01 * x0) / det) } else { (0.0, 0.0) };
    if det == 0.0 || al < 1e-6 * seg || ar < 1e-6 * seg || al > seg * 2.5 || ar > seg * 2.5 {
        al = seg / 3.0;
        ar = seg / 3.0;
    }
    [p0, addp(p0, mulp(t1, al)), addp(p3, mulp(t2, ar)), p3]
}

fn max_err(d: &[Pt], f: usize, l: usize, q: &Bez, u: &[f64]) -> (f64, usize) {
    let mut me = 0.0;
    let mut sp = (f + l) / 2;
    for i in 1..u.len().saturating_sub(1) {
        let p = bz(q, u[i]);
        let e = (p[0] - d[f + i][0]).powi(2) + (p[1] - d[f + i][1]).powi(2);
        if e > me {
            me = e;
            sp = f + i;
        }
    }
    (me, sp)
}

fn reparam(d: &[Pt], f: usize, q: &Bez, u: &[f64]) -> Vec<f64> {
    let q1: Vec<Pt> = (0..3).map(|i| mulp(sub(q[i + 1], q[i]), 3.0)).collect();
    let q2: Vec<Pt> = (0..2).map(|i| mulp(sub(q1[i + 1], q1[i]), 2.0)).collect();
    u.iter()
        .enumerate()
        .map(|(i, &t)| {
            let p = d[f + i];
            let m = 1.0 - t;
            let qu = bz(q, t);
            let q1u = [
                m * m * q1[0][0] + 2.0 * m * t * q1[1][0] + t * t * q1[2][0],
                m * m * q1[0][1] + 2.0 * m * t * q1[1][1] + t * t * q1[2][1],
            ];
            let q2u = [m * q2[0][0] + t * q2[1][0], m * q2[0][1] + t * q2[1][1]];
            let df = sub(qu, p);
            let num = dot(df, q1u);
            let den = dot(q1u, q1u) + dot(df, q2u);
            let nt = if den != 0.0 { t - num / den } else { t };
            nt.clamp(0.0, 1.0)
        })
        .collect()
}

#[allow(clippy::too_many_arguments)]
fn fit_cubic(d: &[Pt], f: usize, l: usize, t1: Pt, t2: Pt, e2: f64, out: &mut Vec<Bez>, depth: usize) {
    let n = l - f + 1;
    let (p0, p3) = (d[f], d[l]);
    if n == 2 || depth > 40 {
        let k = (p3[0] - p0[0]).hypot(p3[1] - p0[1]) / 3.0;
        out.push([p0, addp(p0, mulp(t1, k)), addp(p3, mulp(t2, k)), p3]);
        return;
    }
    let mut u = vec![0.0];
    for i in f + 1..=l {
        let last = *u.last().unwrap();
        u.push(last + (d[i][0] - d[i - 1][0]).hypot(d[i][1] - d[i - 1][1]));
    }
    let len = *u.last().unwrap();
    let len = if len == 0.0 { 1.0 } else { len };
    for v in u.iter_mut() {
        *v /= len;
    }
    let mut q = gen_bez(d, f, l, &u, t1, t2);
    let (mut me, mut sp) = max_err(d, f, l, &q, &u);
    if me < e2 {
        out.push(q);
        return;
    }
    if me < e2 * 9.0 {
        for _ in 0..6 {
            u = reparam(d, f, &q, &u);
            q = gen_bez(d, f, l, &u, t1, t2);
            let r = max_err(d, f, l, &q, &u);
            me = r.0;
            sp = r.1;
            if me < e2 {
                out.push(q);
                return;
            }
        }
    }
    if sp <= f {
        sp = f + 1;
    }
    if sp >= l {
        sp = l - 1;
    }
    // Tangente am Teilungspunkt aus einem breiteren Fenster (robust gegen Punktrauschen)
    let kk = (sp - f).min(l - sp).min(4).max(1);
    let tc = nrm(sub(d[sp - kk], d[sp + kk]));
    fit_cubic(d, f, sp, t1, tc, e2, out, depth + 1);
    fit_cubic(d, sp, l, mulp(tc, -1.0), t2, e2, out, depth + 1);
}

fn end_tan(d: &[Pt], i: usize, dir: i64, k: usize) -> Pt {
    let n = d.len() as i64;
    let mut v = [0.0, 0.0];
    for j in 1..=k as i64 {
        let q = d[(i as i64 + dir * j).clamp(0, n - 1) as usize];
        v = addp(v, mulp(sub(q, d[i]), 1.0 / j as f64));
    }
    nrm(v)
}

/// Eine Pixelkontur in eine Form umwandeln (Koordinaten im Originalbild).
/// Zähler symmetrisch gemachter Formen (je Vektorisierung zurückgesetzt)
pub static SYM_COUNT: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);

/// Prüft eine geschlossene Kontur auf Spiegelsymmetrie zur senkrechten und waagrechten
/// Mittelachse ihres Umrisses. Weicht das Spiegelbild höchstens `tol` ab, werden die Punkte
/// mit ihrem Spiegelpartner gemittelt – die Form wird exakt symmetrisch.
pub fn symmetrize(mp: &mut Vec<Pt>, tol: f64, min_size: f64) -> bool {
    let n = mp.len();
    let (mut x0, mut x1, mut y0, mut y1) = (f64::MAX, f64::MIN, f64::MAX, f64::MIN);
    for p in mp.iter() {
        x0 = x0.min(p[0]);
        x1 = x1.max(p[0]);
        y0 = y0.min(p[1]);
        y1 = y1.max(p[1]);
    }
    if (x1 - x0).max(y1 - y0) < min_size {
        return false;
    }
    let (cx, cy) = ((x0 + x1) / 2.0, (y0 + y1) / 2.0);
    let mut any = false;
    for axis in 0..2 {
        // Rasterindex für die Nächste-Punkt-Suche
        let cell = tol.max(1.0) * 2.0;
        let mut grid: HashMap<(i64, i64), Vec<usize>> = HashMap::new();
        for (i, p) in mp.iter().enumerate() {
            grid.entry(((p[0] / cell).floor() as i64, (p[1] / cell).floor() as i64)).or_default().push(i);
        }
        let mirror = |p: Pt| if axis == 0 { [2.0 * cx - p[0], p[1]] } else { [p[0], 2.0 * cy - p[1]] };
        let nearest = |q: Pt| -> Option<(usize, f64)> {
            let (gx, gy) = ((q[0] / cell).floor() as i64, (q[1] / cell).floor() as i64);
            let mut best: Option<(usize, f64)> = None;
            for dx in -1..=1 {
                for dy in -1..=1 {
                    if let Some(v) = grid.get(&(gx + dx, gy + dy)) {
                        for &i in v {
                            let d = ((mp[i][0] - q[0]).powi(2) + (mp[i][1] - q[1]).powi(2)).sqrt();
                            if best.map_or(true, |b| d < b.1) {
                                best = Some((i, d));
                            }
                        }
                    }
                }
            }
            best
        };
        let mut partner = Vec::with_capacity(n);
        let mut ok = true;
        for p in mp.iter() {
            match nearest(mirror(*p)) {
                Some((i, d)) if d <= tol => partner.push(i),
                _ => {
                    ok = false;
                    break;
                }
            }
        }
        if !ok {
            continue;
        }
        let old = mp.clone();
        for i in 0..n {
            let q = mirror(old[partner[i]]);
            mp[i] = [(old[i][0] + q[0]) / 2.0, (old[i][1] + q[1]) / 2.0];
        }
        any = true;
    }
    any
}

pub fn fit_loop(pts: &[i64], o: &Opts, sc: f64) -> Option<Shape> {
    let nn = pts.len() / 2;
    if nn < 3 {
        return None;
    }
    let eps = (o.tol * sc).max(0.03);
    let p = |i: usize| -> Pt { [pts[i * 2] as f64, pts[i * 2 + 1] as f64] };
    let mp: Vec<Pt> = (0..nn)
        .map(|i| {
            let j = (i + 1) % nn;
            [(p(i)[0] + p(j)[0]) / 2.0, (p(i)[1] + p(j)[1]) / 2.0]
        })
        .collect();
    let k_inv = 1.0 / sc;
    let mut mp = mp;
    if o.symmetry && nn >= 16 && symmetrize(&mut mp, (0.9 * sc).max(1.0), 8.0 * sc) {
        SYM_COUNT.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    }
    let mp = mp;
    if (o.circles || o.ellipses) && nn >= 24 {
        if let Some(e) = fit_ellipse(&mp, eps) {
            if e.circle && o.circles {
                let r = (e.a + e.b) / 2.0;
                return Some(Shape::Ellipse([e.cx * k_inv, e.cy * k_inv, r * k_inv, r * k_inv, 0.0]));
            }
            if o.ellipses && (!e.circle || !o.circles) {
                return Some(Shape::Ellipse([e.cx * k_inv, e.cy * k_inv, e.a * k_inv, e.b * k_inv, e.rot]));
            }
        }
    }
    // Kantenrichtungen und Lauflängen
    let mut dir = vec![0u8; nn];
    for i in 0..nn {
        let j = (i + 1) % nn;
        let (dx, dy) = (pts[j * 2] - pts[i * 2], pts[j * 2 + 1] - pts[i * 2 + 1]);
        dir[i] = if dx > 0 {
            0
        } else if dy > 0 {
            1
        } else if dx < 0 {
            2
        } else {
            3
        };
    }
    let mut run = vec![0usize; nn];
    {
        let mut st = 0;
        while st < nn && dir[st] == dir[(st + nn - 1) % nn] {
            st += 1;
        }
        if st >= nn {
            st = 0;
        }
        let mut i = 0;
        while i < nn {
            let a = (st + i) % nn;
            let mut k = 1;
            while i + k < nn && dir[(st + i + k) % nn] == dir[a] {
                k += 1;
            }
            for q in 0..k {
                run[(st + i + q) % nn] = k;
            }
            i += k;
        }
    }
    let run_min = (js_round(2.5 * sc) as usize).max(3);
    let mut r: Vec<Pt> = Vec::new();
    let mut fixed: Vec<bool> = Vec::new();
    for i in 0..nn {
        r.push(mp[i]);
        fixed.push(false);
        let j = (i + 1) % nn;
        if dir[j] != dir[i] && run[i] >= run_min && run[j] >= run_min {
            r.push(p(j));
            fixed.push(true);
        }
    }
    let n = r.len();
    if n < 3 {
        return None;
    }
    let thr = o.corner.to_radians();
    let n6 = if n / 6 == 0 { 1 } else { n / 6 };
    let k = (js_round(1.5 + 1.2 * sc) as usize).min(n6).max(2);
    let mut ang = vec![0.0f64; n];
    for i in 0..n {
        let a = r[(i + n - k % n) % n];
        let b = r[i];
        let c = r[(i + k) % n];
        let u = sub(b, a);
        let v = sub(c, b);
        ang[i] = (u[0] * v[1] - u[1] * v[0]).atan2(dot(u, v)).abs() as f32 as f64;
    }
    let mut corner = vec![false; n];
    for i in 0..n {
        if fixed[i] {
            corner[i] = true;
            continue;
        }
        if ang[i] < thr {
            continue;
        }
        let mut mx = true;
        for j in 1..=k {
            let (a, b) = ((i + j) % n, (i + n - j % n) % n);
            if ang[a] > ang[i] || ang[b] >= ang[i] || fixed[a] || fixed[b] {
                mx = false;
                break;
            }
        }
        if mx {
            corner[i] = true;
        }
    }
    // glätten (Ecken bleiben)
    let mut pp = r.clone();
    let its = 2 + (js_round(sc) as usize).min(3) + (o.calm * 2.0) as usize;
    for _ in 0..its {
        let mut q = pp.clone();
        for i in 0..n {
            if corner[i] {
                continue;
            }
            let (a, b) = (pp[(i + n - 1) % n], pp[(i + 1) % n]);
            q[i] = [(a[0] + 2.0 * pp[i][0] + b[0]) / 4.0, (a[1] + 2.0 * pp[i][1] + b[1]) / 4.0];
        }
        pp = q;
    }
    let cs: Vec<usize> = (0..n).filter(|&i| corner[i]).collect();
    let mut segs: Vec<(Vec<Pt>, bool)> = Vec::new();
    if cs.is_empty() {
        let mut a = pp.clone();
        a.push(pp[0]);
        segs.push((a, true));
    } else {
        for q in 0..cs.len() {
            let (s0, s1) = (cs[q], cs[(q + 1) % cs.len()]);
            let mut a = vec![pp[s0]];
            let mut i = s0;
            loop {
                i = (i + 1) % n;
                a.push(pp[i]);
                if i == s1 {
                    break;
                }
            }
            segs.push((a, false));
        }
    }
    let e2 = eps * eps;
    let tk = (js_round(1.0 + sc) as usize).max(2);
    let mut c: Vec<Cmd> = vec![Cmd::M(segs[0].0[0][0] * k_inv, segs[0].0[0][1] * k_inv)];
    let s = |v: f64| v * k_inv;
    // Gerade Stücke erkennen: lange Abschnitte, deren Punkte nur wenig (Pixel-Welligkeit)
    // von der Sehne abweichen, werden zu exakten Geraden – Schrift und Kanten werden gerade
    let lt = (0.85 * sc).max(eps);
    let lmin = 7.0 * sc;
    let mut segs2: Vec<(Vec<Pt>, bool, bool)> = Vec::new();
    for (a, closed) in &segs {
        let m = a.len();
        if *closed || m < 4 || !o.straight {
            segs2.push((a.clone(), *closed, false));
            continue;
        }
        // Knoten der groben Vereinfachung als Kandidaten für Geradenenden
        let key = rdp_open(a, lt);
        let mut idx = Vec::new();
        let mut j = 0;
        for q in &key {
            while j < m && (a[j][0] - q[0]).abs() + (a[j][1] - q[1]).abs() > 1e-9 {
                j += 1;
            }
            if j < m {
                idx.push(j);
            }
        }
        if idx.len() < 2 {
            segs2.push((a.clone(), false, false));
            continue;
        }
        let mut start = 0usize;
        for w2 in idx.windows(2) {
            let (i0, i1) = (w2[0], w2[1]);
            let len = ((a[i1][0] - a[i0][0]).powi(2) + (a[i1][1] - a[i0][1]).powi(2)).sqrt();
            let straight = len >= lmin && (i0..=i1).all(|i| seg_d(a[i], a[i0], a[i1]) <= lt);
            if straight {
                if i0 > start {
                    segs2.push((a[start..=i0].to_vec(), false, false));
                }
                segs2.push((vec![a[i0], a[i1]], false, true));
                start = i1;
            }
        }
        if start < m - 1 {
            segs2.push((a[start..].to_vec(), false, false));
        }
    }
    for (a, closed, is_line) in &segs2 {
        let m = a.len();
        if m < 2 {
            continue;
        }
        if *is_line {
            c.push(Cmd::L(s(a[1][0]), s(a[1][1])));
            continue;
        }
        let chord_ok = (1..m - 1).all(|i| seg_d(a[i], a[0], a[m - 1]) <= eps);
        if !closed && chord_ok {
            c.push(Cmd::L(s(a[m - 1][0]), s(a[m - 1][1])));
            continue;
        }
        if !o.quad && !o.cubic {
            let rr = rdp_open(a, eps);
            for q in rr.iter().skip(1) {
                c.push(Cmd::L(s(q[0]), s(q[1])));
            }
            continue;
        }
        let (t1, t2) = if *closed {
            let t = nrm(sub(a[1], a[m - 2]));
            (t, mulp(t, -1.0))
        } else {
            (end_tan(a, 0, 1, tk), end_tan(a, m - 1, -1, tk))
        };
        let mut cur = Vec::new();
        fit_cubic(a, 0, m - 1, t1, t2, e2, &mut cur, 0);
        for q in cur {
            let lin = seg_d(q[1], q[0], q[3]) < eps * 0.25 && seg_d(q[2], q[0], q[3]) < eps * 0.25;
            if lin {
                c.push(Cmd::L(s(q[3][0]), s(q[3][1])));
                continue;
            }
            if o.cubic {
                c.push(Cmd::C(s(q[1][0]), s(q[1][1]), s(q[2][0]), s(q[2][1]), s(q[3][0]), s(q[3][1])));
            } else {
                let m01 = mulp(addp(q[0], q[1]), 0.5);
                let m12 = mulp(addp(q[1], q[2]), 0.5);
                let m23 = mulp(addp(q[2], q[3]), 0.5);
                let aa = mulp(addp(m01, m12), 0.5);
                let bb = mulp(addp(m12, m23), 0.5);
                let mid = mulp(addp(aa, bb), 0.5);
                for hh in [[q[0], m01, aa, mid], [mid, bb, m23, q[3]]] {
                    let qc = mulp(sub(mulp(addp(hh[1], hh[2]), 3.0), addp(hh[0], hh[3])), 0.25);
                    c.push(Cmd::Q(s(qc[0]), s(qc[1]), s(hh[3][0]), s(hh[3][1])));
                }
            }
        }
    }
    c.push(Cmd::Z);
    Some(Shape::Path(c))
}

pub fn vectorize(gm: &[i16], w: usize, h: usize, ng: usize, sc: f64, o: &Opts, stack: bool) -> Vec<Layer> {
    let mut cnt = vec![0usize; ng];
    for &v in gm {
        if v >= 0 && (v as usize) < ng {
            cnt[v as usize] += 1;
        }
    }
    let mut order: Vec<usize> = (0..ng).collect();
    order.sort_by(|&a, &b| cnt[b].cmp(&cnt[a]));
    let mut rank = vec![0usize; ng];
    for (r, &li) in order.iter().enumerate() {
        rank[li] = r;
    }
    let mut m = vec![false; w * h];
    let mut layers = Vec::new();
    let amin = (o.min_area * sc * sc).max(1.0);
    for r in 0..ng {
        let li = order[r];
        if cnt[li] == 0 {
            continue;
        }
        for (p, &v) in gm.iter().enumerate() {
            m[p] = if v < 0 { false } else if stack { rank[v as usize] >= r } else { v as usize == li };
        }
        let mut subs = Vec::new();
        for lp in trace_mask(&m, w, h) {
            if area_of(&lp).abs() < amin {
                continue;
            }
            if let Some(sp) = fit_loop(&lp, o, sc) {
                subs.push(sp);
            }
        }
        if !subs.is_empty() {
            layers.push(Layer { idx: li, subs });
        }
    }
    layers
}

// ---------------------------------------------------------------------------
// Gemeinsame Kanten
//
// Jede Grenze zwischen zwei Farbflächen wird genau einmal verfolgt, subpixelgenau
// aus den Kantenpixeln des Bildes bestimmt und als Kurve angepasst. Beide
// Nachbarflächen benutzen dieselbe Kurve – keine Lücken, keine Überlappungen,
// und an Dreiländerecken laufen durchgehende Kanten glatt weiter.

const DXY: [(i64, i64); 4] = [(1, 0), (0, 1), (-1, 0), (0, -1)];

#[derive(Clone, Copy, Debug)]
enum Seg {
    L(Pt),
    C(Pt, Pt, Pt),
}

fn seg_end(s: &Seg) -> Pt {
    match *s {
        Seg::L(p) => p,
        Seg::C(_, _, p) => p,
    }
}

/// Grenzlinie zwischen zwei Flächen (von Knoten zu Knoten oder geschlossen).
struct Chain {
    /// Pixel-Eckpunkte; geschlossen: letzter == erster
    v: Vec<(i64, i64)>,
    /// Schrittrichtungen (0 = +x, 1 = +y, 2 = -x, 3 = -y)
    d: Vec<u8>,
    /// Fläche links / rechts der Laufrichtung (-1 = nichts)
    l: i16,
    r: i16,
    closed: bool,
    /// doppelte Shoelace-Summe der Eckpunkte
    cross: i64,
}

/// Angepasste Kurve einer Grenzlinie (Analyse-Koordinaten).
struct Fitted {
    start: Pt,
    segs: Vec<Seg>,
    ell: Option<[f64; 5]>,
}

struct Lab<'a> {
    g: &'a [i16],
    w: usize,
    h: usize,
}

impl Lab<'_> {
    fn at(&self, x: i64, y: i64) -> i16 {
        if x < 0 || y < 0 || x >= self.w as i64 || y >= self.h as i64 {
            return -1;
        }
        let v = self.g[y as usize * self.w + x as usize];
        if v < 0 {
            -1
        } else {
            v
        }
    }
    /// (links, rechts) beim Schritt von Eckpunkt (x,y) in Richtung d (y nach unten).
    fn sides(&self, x: i64, y: i64, d: usize) -> (i16, i16) {
        match d {
            0 => (self.at(x, y - 1), self.at(x, y)),
            1 => (self.at(x, y), self.at(x - 1, y)),
            2 => (self.at(x - 1, y), self.at(x - 1, y - 1)),
            _ => (self.at(x - 1, y - 1), self.at(x, y - 1)),
        }
    }
    fn edge(&self, x: i64, y: i64, d: usize) -> bool {
        let (a, b) = self.sides(x, y, d);
        a != b
    }
    /// Index der Pixelkante für den Belegt-Merker.
    fn sid(&self, x: i64, y: i64, d: usize) -> usize {
        let (dx, dy) = DXY[d];
        if dy == 0 {
            let xx = x.min(x + dx);
            y as usize * self.w + xx as usize
        } else {
            let yy = y.min(y + dy);
            (self.h + 1) * self.w + yy as usize * (self.w + 1) + x as usize
        }
    }
}

fn walk_chain(lab: &Lab, deg: &[u8], used: &mut [bool], x0: i64, y0: i64, d0: usize) -> Chain {
    let vw = lab.w + 1;
    let (l, r) = lab.sides(x0, y0, d0);
    let mut v = vec![(x0, y0)];
    let mut ds = Vec::new();
    let (mut x, mut y, mut d) = (x0, y0, d0);
    let mut cross = 0i64;
    let mut closed = false;
    loop {
        used[lab.sid(x, y, d)] = true;
        ds.push(d as u8);
        let (nx, ny) = (x + DXY[d].0, y + DXY[d].1);
        cross += x * ny - nx * y;
        x = nx;
        y = ny;
        v.push((x, y));
        if deg[y as usize * vw + x as usize] >= 3 {
            break;
        }
        if x == x0 && y == y0 {
            closed = true;
            break;
        }
        let mut nd = None;
        for k in [(d + 1) & 3, d, (d + 3) & 3] {
            if lab.edge(x, y, k) && !used[lab.sid(x, y, k)] {
                nd = Some(k);
                break;
            }
        }
        match nd {
            Some(k) => d = k,
            None => break,
        }
    }
    Chain { v, d: ds, l, r, closed, cross }
}

fn build_chains(lab: &Lab) -> (Vec<Chain>, Vec<u8>) {
    let (w, h) = (lab.w as i64, lab.h as i64);
    let vw = lab.w + 1;
    let mut deg = vec![0u8; vw * (lab.h + 1)];
    for y in 0..=h {
        for x in 0..=w {
            deg[y as usize * vw + x as usize] = (0..4).filter(|&d| lab.edge(x, y, d)).count() as u8;
        }
    }
    let mut used = vec![false; (lab.h + 1) * lab.w + lab.h * (lab.w + 1)];
    let mut chains = Vec::new();
    for y in 0..=h {
        for x in 0..=w {
            if deg[y as usize * vw + x as usize] < 3 {
                continue;
            }
            for d in 0..4 {
                if lab.edge(x, y, d) && !used[lab.sid(x, y, d)] {
                    chains.push(walk_chain(lab, &deg, &mut used, x, y, d));
                }
            }
        }
    }
    // Übrig: geschlossene Grenzen ohne Knoten (Inseln)
    for y in 0..=h {
        for x in 0..w {
            if lab.edge(x, y, 0) && !used[lab.sid(x, y, 0)] {
                chains.push(walk_chain(lab, &deg, &mut used, x, y, 0));
            }
        }
    }
    (chains, deg)
}

/// Ausgleichsgerade (Schwerpunkt, Richtung, größte Abweichung).
fn line_fit(p: &[Pt]) -> Option<(Pt, Pt, f64)> {
    let n = p.len();
    if n < 2 {
        return None;
    }
    let c = mulp(p.iter().fold([0.0, 0.0], |a, b| addp(a, *b)), 1.0 / n as f64);
    let (mut xx, mut yy, mut xy) = (0.0, 0.0, 0.0);
    for q in p {
        let (u, v) = (q[0] - c[0], q[1] - c[1]);
        xx += u * u;
        yy += v * v;
        xy += u * v;
    }
    let th = 0.5 * (2.0 * xy).atan2(xx - yy);
    let d = [th.cos(), th.sin()];
    let nv = [-d[1], d[0]];
    let r = p.iter().map(|q| dot(sub(*q, c), nv).abs()).fold(0.0, f64::max);
    Some((c, d, r))
}

/// Subpixel-Lage einer Pixelkante: Abstand der echten Farbkante von der Pixelgrenze
/// entlang der Normalen (zur rechten Fläche hin), aus dem Anteil der Kantenpixel.
#[allow(clippy::too_many_arguments)]
fn refine_edge(lab: &Lab, data: &[u8], m: Pt, n: (i64, i64), l: i16, r: i16, wmax: usize) -> Option<f64> {
    if l < 0 && r < 0 {
        return None;
    }
    let (w, h) = (lab.w as i64, lab.h as i64);
    let pix = |s: f64| -> (i64, i64) { ((m[0] + s * n.0 as f64).floor() as i64, (m[1] + s * n.1 as f64).floor() as i64) };
    let inb = |p: (i64, i64)| p.0 >= 0 && p.1 >= 0 && p.0 < w && p.1 < h;
    let mut rs = Vec::new();
    let mut ls = Vec::new();
    for j in 0..wmax {
        let p = pix(j as f64 + 0.5);
        if !inb(p) || lab.at(p.0, p.1) != r {
            break;
        }
        rs.push(p);
    }
    for j in 0..wmax {
        let p = pix(-(j as f64) - 0.5);
        if !inb(p) || lab.at(p.0, p.1) != l {
            break;
        }
        ls.push(p);
    }
    if rs.len() < 2 || ls.len() < 2 {
        return None;
    }
    let col = |p: (i64, i64)| -> [f64; 4] {
        let o = (p.1 as usize * lab.w + p.0 as usize) * 4;
        let a = data[o + 3] as f64 / 255.0;
        [data[o] as f64 * a, data[o + 1] as f64 * a, data[o + 2] as f64 * a, data[o + 3] as f64]
    };
    let ca = col(*ls.last().unwrap());
    let cb = col(*rs.last().unwrap());
    let df = [cb[0] - ca[0], cb[1] - ca[1], cb[2] - ca[2], cb[3] - ca[3]];
    let dd = df.iter().map(|v| v * v).sum::<f64>();
    if dd < 24.0 * 24.0 {
        return None;
    }
    let mut s = 0.0;
    for p in ls.iter().chain(rs.iter()) {
        let c = col(*p);
        let t = ((c[0] - ca[0]) * df[0] + (c[1] - ca[1]) * df[1] + (c[2] - ca[2]) * df[2] + (c[3] - ca[3]) * df[3]) / dd;
        s += t.clamp(0.0, 1.0);
    }
    Some((rs.len() as f64 - s).clamp(-1.0, 1.0))
}

fn dir_runs(d: &[u8], cyclic: bool) -> Vec<usize> {
    let n = d.len();
    let mut run = vec![0usize; n];
    if n == 0 {
        return run;
    }
    let mut st = 0;
    if cyclic {
        while st < n && d[st] == d[(st + n - 1) % n] {
            st += 1;
        }
        if st >= n {
            st = 0;
        }
    }
    let mut i = 0;
    while i < n {
        let a = (st + i) % n;
        let mut k = 1;
        while i + k < n && d[(st + i + k) % n] == d[a] {
            k += 1;
        }
        for q in 0..k {
            run[(st + i + q) % n] = k;
        }
        i += k;
    }
    run
}

/// Krümmung eines Abschnitts: Pfeilhöhe einer angepassten Parabel über der Sehne.
fn sagitta(a: &[Pt]) -> f64 {
    let m = a.len();
    if m < 5 {
        return 0.0;
    }
    let (p0, p1) = (a[0], a[m - 1]);
    let dv = sub(p1, p0);
    let len = dv[0].hypot(dv[1]);
    if len < 1e-9 {
        return 0.0;
    }
    let t = [dv[0] / len, dv[1] / len];
    let nv = [-t[1], t[0]];
    let mut s = [0.0f64; 5];
    let mut r = [0.0f64; 3];
    for p in a {
        let q = sub(*p, p0);
        let u = dot(q, t) / len * 2.0 - 1.0;
        let v = dot(q, nv);
        let mut uk = 1.0;
        for k in 0..5 {
            s[k] += uk;
            if k < 3 {
                r[k] += uk * v;
            }
            uk *= u;
        }
    }
    let mm = [[s[0], s[1], s[2]], [s[1], s[2], s[3]], [s[2], s[3], s[4]]];
    let det3 = |m: [[f64; 3]; 3]| {
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
    };
    let det = det3(mm);
    if det.abs() < 1e-12 {
        return 0.0;
    }
    let mut m2 = mm;
    for i in 0..3 {
        m2[i][2] = r[i];
    }
    (det3(m2) / det).abs()
}

/// Einen Abschnitt zwischen zwei Ecken als Geraden und Bézierkurven anpassen.
#[allow(clippy::too_many_arguments)]
fn fit_piece(a: &[Pt], closed: bool, t1o: Option<Pt>, t2o: Option<Pt>, o: &Opts, sc: f64, refined: bool, out: &mut Vec<Seg>) {
    let m = a.len();
    if m < 2 {
        return;
    }
    let eps = (o.tol * sc).max(0.03);
    let e2 = eps * eps;
    let tk = (js_round(1.0 + sc) as usize).max(2);
    let (lt, sag) = if refined { ((0.45 * sc).max(0.5), (0.07 * sc).max(0.25)) } else { (0.85 * sc, (0.15 * sc).max(0.4)) };
    let lmin = 7.0 * sc;
    if !o.quad && !o.cubic {
        for q in rdp_open(a, eps).iter().skip(1) {
            out.push(Seg::L(*q));
        }
        return;
    }
    // Teile: (von, bis, gerade)
    let mut parts: Vec<(usize, usize, bool)> = Vec::new();
    if closed || m < 4 || !o.straight {
        parts.push((0, m - 1, false));
    } else {
        let key = rdp_open(a, lt);
        let mut idx = Vec::new();
        let mut j = 0;
        for q in &key {
            while j < m && (a[j][0] - q[0]).abs() + (a[j][1] - q[1]).abs() > 1e-9 {
                j += 1;
            }
            if j < m {
                idx.push(j);
            }
        }
        let mut start = 0usize;
        for w2 in idx.windows(2) {
            let (i0, i1) = (w2[0], w2[1]);
            let len = (a[i1][0] - a[i0][0]).hypot(a[i1][1] - a[i0][1]);
            let straight = len >= lmin && (i0..=i1).all(|i| seg_d(a[i], a[i0], a[i1]) <= lt) && sagitta(&a[i0..=i1]) <= sag;
            if straight {
                if i0 > start {
                    parts.push((start, i0, false));
                }
                parts.push((i0, i1, true));
                start = i1;
            }
        }
        if start < m - 1 {
            parts.push((start, m - 1, false));
        }
    }
    let np = parts.len();
    for (pi, &(f, l, is_line)) in parts.iter().enumerate() {
        if is_line || l - f < 1 {
            out.push(Seg::L(a[l]));
            continue;
        }
        let b = &a[f..=l];
        let mb = b.len();
        let open_chord = !closed && (1..mb - 1).all(|i| seg_d(b[i], b[0], b[mb - 1]) <= eps) && sagitta(b) <= sag;
        if open_chord {
            out.push(Seg::L(b[mb - 1]));
            continue;
        }
        let (t1, t2) = if closed {
            let t = nrm(sub(b[1], b[mb - 2]));
            (t, mulp(t, -1.0))
        } else {
            // an Geraden tangential anschließen, an den Enden ggf. vorgegebene Richtung
            let t1 = if pi > 0 && parts[pi - 1].2 {
                nrm(sub(a[f], a[parts[pi - 1].0]))
            } else if pi == 0 {
                t1o.unwrap_or_else(|| end_tan(b, 0, 1, tk))
            } else {
                end_tan(b, 0, 1, tk)
            };
            let t2 = if pi + 1 < np && parts[pi + 1].2 {
                nrm(sub(a[l], a[parts[pi + 1].1]))
            } else if pi + 1 == np {
                t2o.unwrap_or_else(|| end_tan(b, mb - 1, -1, tk))
            } else {
                end_tan(b, mb - 1, -1, tk)
            };
            (t1, t2)
        };
        let mut cur = Vec::new();
        fit_cubic(b, 0, mb - 1, t1, t2, e2, &mut cur, 0);
        for q in cur {
            let lin = seg_d(q[1], q[0], q[3]) < eps * 0.25 && seg_d(q[2], q[0], q[3]) < eps * 0.25;
            if lin {
                out.push(Seg::L(q[3]));
            } else {
                out.push(Seg::C(q[1], q[2], q[3]));
            }
        }
    }
}

/// Vorbereitete Punktfolge einer Grenzlinie.
struct Prep {
    pts: Vec<Pt>,
    corner: Vec<bool>,
    ell: Option<[f64; 5]>,
}

#[allow(clippy::too_many_arguments)]
pub fn vectorize_shared(gm: &[i16], data: &[u8], w: usize, h: usize, ng: usize, sc: f64, o: &Opts, stack: bool) -> Vec<Layer> {
    if w == 0 || h == 0 || gm.len() != w * h {
        return Vec::new();
    }
    let lab = Lab { g: gm, w, h };
    let (chains, _) = build_chains(&lab);
    let vw = w + 1;
    let use_sub = o.subpx && data.len() == w * h * 4;
    let wmax = ((sc.ceil() as usize) + 2).clamp(2, 8);
    let eps = (o.tol * sc).max(0.03);

    // 1. Mittelpunkte der Pixelkanten, subpixelgenau verschoben
    let mut mids: Vec<Vec<Pt>> = Vec::with_capacity(chains.len());
    let mut jsum: HashMap<usize, ([f64; 2], f64)> = HashMap::new();
    for ch in &chains {
        let n = ch.d.len();
        let mut mm = Vec::with_capacity(n);
        let mut offs = Vec::with_capacity(n);
        for i in 0..n {
            let (x, y) = ch.v[i];
            let (dx, dy) = DXY[ch.d[i] as usize];
            let m = [x as f64 + dx as f64 * 0.5, y as f64 + dy as f64 * 0.5];
            let nv = (-dy, dx);
            let e = if use_sub { refine_edge(&lab, data, m, nv, ch.l, ch.r, wmax).unwrap_or(0.0) } else { 0.0 };
            let off = [nv.0 as f64 * e, nv.1 as f64 * e];
            mm.push(addp(m, off));
            offs.push(off);
        }
        if !ch.closed && n > 0 {
            for (vi, oi) in [(0usize, 0usize), (n, n - 1)] {
                let (x, y) = ch.v[vi];
                let e = jsum.entry(y as usize * vw + x as usize).or_insert(([0.0, 0.0], 0.0));
                e.0 = addp(e.0, offs[oi]);
                e.1 += 1.0;
            }
        }
        mids.push(mm);
    }
    let jpos = |x: i64, y: i64| -> Pt {
        let base = [x as f64, y as f64];
        match jsum.get(&(y as usize * vw + x as usize)) {
            Some((s, c)) if *c > 0.0 => {
                let mut off = mulp(*s, 1.0 / c);
                let l = off[0].hypot(off[1]);
                if l > 0.75 {
                    off = mulp(off, 0.75 / l);
                }
                addp(base, off)
            }
            _ => base,
        }
    };

    // 2. Punktfolgen, Ecken, Glättung, Kreise/Ellipsen
    let run_min = (js_round(2.5 * sc) as usize).max(3);
    let thr = o.corner.to_radians();
    let mut preps: Vec<Prep> = Vec::with_capacity(chains.len());
    for (ci, ch) in chains.iter().enumerate() {
        let n = ch.d.len();
        let mm = &mids[ci];
        let run = dir_runs(&ch.d, ch.closed);
        let mut pts: Vec<Pt> = Vec::with_capacity(n + 4);
        let mut fixed: Vec<bool> = Vec::with_capacity(n + 4);
        if !ch.closed {
            pts.push(jpos(ch.v[0].0, ch.v[0].1));
            fixed.push(true);
        }
        for i in 0..n {
            pts.push(mm[i]);
            fixed.push(false);
            let j = i + 1;
            if j == n && !ch.closed {
                continue;
            }
            let jj = j % n;
            if ch.d[jj] != ch.d[i] && run[i] >= run_min && run[jj] >= run_min {
                let (a, b) = (mm[i], mm[jj]);
                pts.push(if ch.d[i] % 2 == 0 { [b[0], a[1]] } else { [a[0], b[1]] });
                fixed.push(true);
            }
        }
        if !ch.closed {
            let (x, y) = ch.v[n];
            pts.push(jpos(x, y));
            fixed.push(true);
        }
        let mut ell = None;
        if ch.closed {
            if o.symmetry && n >= 16 && symmetrize(&mut pts, (0.9 * sc).max(1.0), 8.0 * sc) {
                SYM_COUNT.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            }
            if (o.circles || o.ellipses) && n >= 24 {
                if let Some(e) = fit_ellipse(&pts, eps) {
                    if e.circle && o.circles {
                        let r = (e.a + e.b) / 2.0;
                        ell = Some([e.cx, e.cy, r, r, 0.0]);
                    } else if o.ellipses {
                        ell = Some([e.cx, e.cy, e.a, e.b, e.rot]);
                    }
                }
            }
        }
        let m = pts.len();
        let mut corner = vec![false; m];
        if ell.is_none() && m >= 2 {
            let get = |i: i64| -> usize {
                if ch.closed {
                    i.rem_euclid(m as i64) as usize
                } else {
                    i.clamp(0, m as i64 - 1) as usize
                }
            };
            let m6 = (m / 6).max(1);
            let k = (js_round(1.5 + 1.2 * sc) as usize).min(m6).max(2) as i64;
            let mut ang = vec![0.0f64; m];
            for i in 0..m {
                let a = pts[get(i as i64 - k)];
                let b = pts[i];
                let c = pts[get(i as i64 + k)];
                let u = sub(b, a);
                let v = sub(c, b);
                ang[i] = (u[0] * v[1] - u[1] * v[0]).atan2(dot(u, v)).abs();
            }
            for i in 0..m {
                if fixed[i] {
                    corner[i] = true;
                    continue;
                }
                if ang[i] < thr {
                    continue;
                }
                let mut mx = true;
                for j in 1..=k {
                    let (a, b) = (get(i as i64 + j), get(i as i64 - j));
                    if (a != i && (ang[a] > ang[i] || fixed[a])) || (b != i && (ang[b] >= ang[i] || fixed[b])) {
                        mx = false;
                        break;
                    }
                }
                corner[i] = mx;
            }
            // Ecken schärfen: Die Pixelauswertung rundet Ecken ab. Die Kanten beiderseits
            // werden als Geraden verlängert und im Schnittpunkt zur scharfen Ecke vereint.
            if use_sub {
                let k0 = js_round(sc) as i64 + 1;
                let k2 = k0 + (js_round(2.5 * sc) as i64).max(3);
                let mut remove = vec![false; m];
                for i in 0..m {
                    if !corner[i] || fixed[i] || (ch.closed && (m as i64) <= 2 * k2 + 2) {
                        continue;
                    }
                    let mut clear = true;
                    'chk: for j in 1..=k2 {
                        for idx in [i as i64 + j, i as i64 - j] {
                            if !ch.closed && (idx < 0 || idx >= m as i64) {
                                clear = false;
                                break 'chk;
                            }
                            let q = get(idx);
                            if q == i || corner[q] || remove[q] {
                                clear = false;
                                break 'chk;
                            }
                        }
                    }
                    if !clear {
                        continue;
                    }
                    let left: Vec<Pt> = (k0..=k2).map(|j| pts[get(i as i64 - j)]).collect();
                    let right: Vec<Pt> = (k0..=k2).map(|j| pts[get(i as i64 + j)]).collect();
                    let (Some(a), Some(b)) = (line_fit(&left), line_fit(&right)) else { continue };
                    let rmax = (0.3 * sc).max(0.35);
                    if a.2 > rmax || b.2 > rmax {
                        continue;
                    }
                    let cr = a.1[0] * b.1[1] - a.1[1] * b.1[0];
                    if cr.abs() < 0.34 {
                        continue;
                    }
                    let dd = sub(b.0, a.0);
                    let t = (dd[0] * b.1[1] - dd[1] * b.1[0]) / cr;
                    let x = addp(a.0, mulp(a.1, t));
                    if (x[0] - pts[i][0]).hypot(x[1] - pts[i][1]) > 2.0 * sc + 1.0 {
                        continue;
                    }
                    pts[i] = x;
                    for j in 1..k0 {
                        remove[get(i as i64 + j)] = true;
                        remove[get(i as i64 - j)] = true;
                    }
                }
                if remove.iter().any(|&r| r) {
                    let keep: Vec<usize> = (0..m).filter(|&i| !remove[i]).collect();
                    pts = keep.iter().map(|&i| pts[i]).collect();
                    corner = keep.iter().map(|&i| corner[i]).collect();
                }
            }
            let m = pts.len();
            let get = |i: i64| -> usize {
                if ch.closed {
                    i.rem_euclid(m as i64) as usize
                } else {
                    i.clamp(0, m as i64 - 1) as usize
                }
            };
            // glätten (Ecken bleiben); subpixelgenaue Punkte brauchen kaum Glättung
            let its = if use_sub { 2 } else { 2 + (js_round(sc) as usize).min(3) } + (o.calm * 2.0) as usize;
            for _ in 0..its {
                let mut q = pts.clone();
                for i in 0..m {
                    if corner[i] {
                        continue;
                    }
                    let (a, b) = (pts[get(i as i64 - 1)], pts[get(i as i64 + 1)]);
                    q[i] = [(a[0] + 2.0 * pts[i][0] + b[0]) / 4.0, (a[1] + 2.0 * pts[i][1] + b[1]) / 4.0];
                }
                pts = q;
            }
        }
        preps.push(Prep { pts, corner, ell });
    }

    // 3. An Knoten durchlaufende Kanten erkennen und tangential verbinden
    let mut t_s: Vec<Option<Pt>> = vec![None; chains.len()];
    let mut t_e: Vec<Option<Pt>> = vec![None; chains.len()];
    {
        let tk = (js_round(1.0 + sc) as usize).max(2);
        let end_dir = |p: &Prep, start: bool| -> Pt {
            let m = p.pts.len();
            let p0 = if start { p.pts[0] } else { p.pts[m - 1] };
            let mut v = [0.0, 0.0];
            for j in 1..=tk.min(m - 1) {
                let i = if start { j } else { m - 1 - j };
                v = addp(v, mulp(sub(p.pts[i], p0), 1.0 / j as f64));
                if p.corner[i] {
                    break;
                }
            }
            nrm(v)
        };
        let mut ends: HashMap<usize, Vec<(usize, bool, Pt)>> = HashMap::new();
        for (ci, ch) in chains.iter().enumerate() {
            if ch.closed || preps[ci].pts.len() < 3 {
                continue;
            }
            let n = ch.d.len();
            let (x0, y0) = ch.v[0];
            let (x1, y1) = ch.v[n];
            ends.entry(y0 as usize * vw + x0 as usize).or_default().push((ci, true, end_dir(&preps[ci], true)));
            ends.entry(y1 as usize * vw + x1 as usize).or_default().push((ci, false, end_dir(&preps[ci], false)));
        }
        let lim = (o.corner.min(40.0) * 0.6).to_radians().cos();
        for list in ends.values() {
            let mut cand: Vec<(f64, usize, usize)> = Vec::new();
            for a in 0..list.len() {
                for b in a + 1..list.len() {
                    let c = -dot(list[a].2, list[b].2);
                    if c > lim {
                        cand.push((c, a, b));
                    }
                }
            }
            cand.sort_by(|x, y| y.0.partial_cmp(&x.0).unwrap_or(std::cmp::Ordering::Equal));
            let mut taken = vec![false; list.len()];
            for (_, a, b) in cand {
                if taken[a] || taken[b] {
                    continue;
                }
                taken[a] = true;
                taken[b] = true;
                let u = nrm(sub(list[a].2, list[b].2));
                for (e, t) in [(list[a], u), (list[b], mulp(u, -1.0))] {
                    if e.1 {
                        t_s[e.0] = Some(t);
                    } else {
                        t_e[e.0] = Some(t);
                    }
                }
            }
        }
    }

    // 4. Jede Grenzlinie einmal anpassen
    let mut fits: Vec<Fitted> = Vec::with_capacity(chains.len());
    for (ci, ch) in chains.iter().enumerate() {
        let p = &preps[ci];
        if let Some(e) = p.ell {
            fits.push(Fitted { start: [e[0], e[1]], segs: Vec::new(), ell: Some(e) });
            continue;
        }
        let m = p.pts.len();
        let cs: Vec<usize> = (0..m).filter(|&i| p.corner[i]).collect();
        let mut segs = Vec::new();
        if m < 2 {
            fits.push(Fitted { start: p.pts.first().copied().unwrap_or([0.0, 0.0]), segs, ell: None });
            continue;
        }
        if ch.closed && cs.is_empty() {
            let mut a = p.pts.clone();
            a.push(p.pts[0]);
            fit_piece(&a, true, None, None, o, sc, use_sub, &mut segs);
            fits.push(Fitted { start: p.pts[0], segs, ell: None });
            continue;
        }
        let nc = cs.len();
        let pieces = if ch.closed { nc } else { nc - 1 };
        for q in 0..pieces {
            let (s0, s1) = (cs[q], cs[(q + 1) % nc]);
            let mut a = vec![p.pts[s0]];
            let mut i = s0;
            loop {
                i = (i + 1) % m;
                a.push(p.pts[i]);
                if i == s1 {
                    break;
                }
            }
            let t1 = if !ch.closed && q == 0 { t_s[ci] } else { None };
            let t2 = if !ch.closed && q + 1 == pieces { t_e[ci] } else { None };
            fit_piece(&a, false, t1, t2, o, sc, use_sub, &mut segs);
        }
        fits.push(Fitted { start: p.pts[cs[0]], segs, ell: None });
    }

    // 5. Flächen aus den gemeinsamen Grenzlinien zusammensetzen
    let k_inv = 1.0 / sc;
    let s = |p: Pt| [p[0] * k_inv, p[1] * k_inv];
    let emit = |c: &mut Vec<Cmd>, start: Pt, segs: &[Seg]| {
        let mut cur = start;
        for sg in segs {
            match *sg {
                Seg::L(p) => {
                    let q = s(p);
                    c.push(Cmd::L(q[0], q[1]));
                    cur = p;
                }
                Seg::C(c1, c2, p) => {
                    if o.cubic {
                        let (a, b, e) = (s(c1), s(c2), s(p));
                        c.push(Cmd::C(a[0], a[1], b[0], b[1], e[0], e[1]));
                    } else {
                        let q = [cur, c1, c2, p];
                        let m01 = mulp(addp(q[0], q[1]), 0.5);
                        let m12 = mulp(addp(q[1], q[2]), 0.5);
                        let m23 = mulp(addp(q[2], q[3]), 0.5);
                        let aa = mulp(addp(m01, m12), 0.5);
                        let bb = mulp(addp(m12, m23), 0.5);
                        let mid = mulp(addp(aa, bb), 0.5);
                        for hh in [[q[0], m01, aa, mid], [mid, bb, m23, q[3]]] {
                            let qc = s(mulp(sub(mulp(addp(hh[1], hh[2]), 3.0), addp(hh[0], hh[3])), 0.25));
                            let e = s(hh[3]);
                            c.push(Cmd::Q(qc[0], qc[1], e[0], e[1]));
                        }
                    }
                    cur = p;
                }
            }
        }
    };
    let oriented = |ci: usize, rev: bool| -> (Pt, Vec<Seg>) {
        let f = &fits[ci];
        if !rev {
            return (f.start, f.segs.clone());
        }
        let mut starts = vec![f.start];
        for sg in &f.segs {
            starts.push(seg_end(sg));
        }
        let mut out = Vec::with_capacity(f.segs.len());
        for i in (0..f.segs.len()).rev() {
            let s0 = starts[i];
            out.push(match f.segs[i] {
                Seg::L(_) => Seg::L(s0),
                Seg::C(c1, c2, _) => Seg::C(c2, c1, s0),
            });
        }
        (*starts.last().unwrap(), out)
    };

    let mut cnt = vec![0usize; ng];
    for &v in gm {
        if v >= 0 && (v as usize) < ng {
            cnt[v as usize] += 1;
        }
    }
    let mut order: Vec<usize> = (0..ng).collect();
    order.sort_by(|&a, &b| cnt[b].cmp(&cnt[a]));
    let mut rank = vec![0usize; ng];
    for (r, &li) in order.iter().enumerate() {
        rank[li] = r;
    }
    let amin = (o.min_area * sc * sc).max(1.0);
    let mut layers = Vec::new();
    for r in 0..ng {
        let li = order[r];
        if cnt[li] == 0 {
            continue;
        }
        let inn = |l: i16| l >= 0 && (l as usize) < ng && if stack { rank[l as usize] >= r } else { l as usize == li };
        let mut subs = Vec::new();
        // gerichtete Grenzlinien: Fläche liegt rechts
        let mut open: Vec<(usize, bool)> = Vec::new();
        let mut by_start: HashMap<(i64, i64), Vec<usize>> = HashMap::new();
        for (ci, ch) in chains.iter().enumerate() {
            let (il, ir) = (inn(ch.l), inn(ch.r));
            if il == ir {
                continue;
            }
            let rev = !ir;
            if ch.closed {
                if (ch.cross as f64 / 2.0).abs() < amin {
                    continue;
                }
                if let Some(e) = fits[ci].ell {
                    subs.push(Shape::Ellipse([e[0] * k_inv, e[1] * k_inv, e[2] * k_inv, e[3] * k_inv, e[4]]));
                } else {
                    let (st, sg) = oriented(ci, rev);
                    let q = s(st);
                    let mut c = vec![Cmd::M(q[0], q[1])];
                    emit(&mut c, st, &sg);
                    c.push(Cmd::Z);
                    subs.push(Shape::Path(c));
                }
                continue;
            }
            let sv = if rev { *ch.v.last().unwrap() } else { ch.v[0] };
            by_start.entry(sv).or_default().push(open.len());
            open.push((ci, rev));
        }
        let first_dir = |e: (usize, bool)| -> usize {
            let ch = &chains[e.0];
            if e.1 {
                (*ch.d.last().unwrap() as usize + 2) & 3
            } else {
                ch.d[0] as usize
            }
        };
        let last_dir = |e: (usize, bool)| -> usize {
            let ch = &chains[e.0];
            if e.1 {
                (ch.d[0] as usize + 2) & 3
            } else {
                *ch.d.last().unwrap() as usize
            }
        };
        let end_v = |e: (usize, bool)| -> (i64, i64) {
            let ch = &chains[e.0];
            if e.1 {
                ch.v[0]
            } else {
                *ch.v.last().unwrap()
            }
        };
        let mut used = vec![false; open.len()];
        for s0 in 0..open.len() {
            if used[s0] {
                continue;
            }
            let mut seq = vec![s0];
            used[s0] = true;
            let mut cur = s0;
            let mut ok = false;
            for _ in 0..open.len() + 1 {
                let e = open[cur];
                let d = last_dir(e);
                let cands = by_start.get(&end_v(e));
                let mut nx = None;
                if let Some(cands) = cands {
                    'pref: for k in [(d + 1) & 3, d, (d + 3) & 3] {
                        for &c in cands {
                            if first_dir(open[c]) == k {
                                nx = Some(c);
                                break 'pref;
                            }
                        }
                    }
                }
                match nx {
                    Some(c) if c == s0 => {
                        ok = true;
                        break;
                    }
                    Some(c) if !used[c] => {
                        used[c] = true;
                        seq.push(c);
                        cur = c;
                    }
                    _ => break,
                }
            }
            if !ok {
                continue;
            }
            let area: f64 = seq.iter().map(|&q| {
                let (ci, rev) = open[q];
                let c = chains[ci].cross as f64;
                if rev {
                    -c
                } else {
                    c
                }
            }).sum::<f64>() / 2.0;
            if area.abs() < amin {
                continue;
            }
            let (st0, _) = oriented(open[seq[0]].0, open[seq[0]].1);
            let q = s(st0);
            let mut c = vec![Cmd::M(q[0], q[1])];
            for &qi in &seq {
                let (st, sg) = oriented(open[qi].0, open[qi].1);
                emit(&mut c, st, &sg);
            }
            c.push(Cmd::Z);
            subs.push(Shape::Path(c));
        }
        if !subs.is_empty() {
            layers.push(Layer { idx: li, subs });
        }
    }
    layers
}

// ---------------------------------------------------------------------------
// Geometrie-Hilfen für Ausgabe

/// Teilpfad als Startpunkt + kubische/lineare Segmente.
pub struct Cubics {
    pub m: Pt,
    /// ('L', [x,y,..]) oder ('C', [x1,y1,x2,y2,x,y])
    pub segs: Vec<(char, [f64; 6])>,
}

pub fn to_cubics(sp: &Shape) -> Vec<Cubics> {
    match sp {
        Shape::Ellipse(e) => {
            let [cx, cy, rx, ry, rot] = *e;
            let t = rot.to_radians();
            let (co, si) = (t.cos(), t.sin());
            let k = 0.5522847;
            let tr = |u: f64, v: f64| -> Pt { [cx + u * co - v * si, cy + u * si + v * co] };
            let pts = [[rx, 0.0], [0.0, ry], [-rx, 0.0], [0.0, -ry]];
            let mut segs = Vec::new();
            for i in 0..4 {
                let (a, b) = (pts[i], pts[(i + 1) % 4]);
                let ta = [-a[1] / ry * rx, a[0] / rx * ry];
                let tb = [-b[1] / ry * rx, b[0] / rx * ry];
                let p1 = tr(a[0] + ta[0] * k, a[1] + ta[1] * k);
                let p2 = tr(b[0] - tb[0] * k, b[1] - tb[1] * k);
                let p3 = tr(b[0], b[1]);
                segs.push(('C', [p1[0], p1[1], p2[0], p2[1], p3[0], p3[1]]));
            }
            vec![Cubics { m: tr(rx, 0.0), segs }]
        }
        Shape::Path(c) => {
            let mut out = Vec::new();
            let mut cur: Pt = [0.0, 0.0];
            let mut curr: Option<Cubics> = None;
            for cmd in c {
                match *cmd {
                    Cmd::M(x, y) => {
                        if let Some(s) = curr.take() {
                            out.push(s);
                        }
                        cur = [x, y];
                        curr = Some(Cubics { m: cur, segs: Vec::new() });
                    }
                    Cmd::L(x, y) => {
                        if let Some(s) = curr.as_mut() {
                            s.segs.push(('L', [x, y, 0.0, 0.0, 0.0, 0.0]));
                        }
                        cur = [x, y];
                    }
                    Cmd::C(a, b, c2, d, x, y) => {
                        if let Some(s) = curr.as_mut() {
                            s.segs.push(('C', [a, b, c2, d, x, y]));
                        }
                        cur = [x, y];
                    }
                    Cmd::Q(qx, qy, x, y) => {
                        if let Some(s) = curr.as_mut() {
                            s.segs.push((
                                'C',
                                [
                                    cur[0] + 2.0 / 3.0 * (qx - cur[0]),
                                    cur[1] + 2.0 / 3.0 * (qy - cur[1]),
                                    x + 2.0 / 3.0 * (qx - x),
                                    y + 2.0 / 3.0 * (qy - y),
                                    x,
                                    y,
                                ],
                            ));
                        }
                        cur = [x, y];
                    }
                    Cmd::Z => {}
                }
            }
            if let Some(s) = curr {
                out.push(s);
            }
            out
        }
    }
}

/// Teilpfade als geschlossene Polygone (Kurven in Stücke zerlegt).
pub fn flatten(sp: &Shape, tol: f64) -> Vec<Vec<Pt>> {
    to_cubics(sp)
        .into_iter()
        .map(|c| {
            let mut pts = vec![c.m];
            let mut cur = c.m;
            for (k, v) in c.segs {
                if k == 'L' {
                    cur = [v[0], v[1]];
                    pts.push(cur);
                } else {
                    let (p1, p2, p3) = ([v[0], v[1]], [v[2], v[3]], [v[4], v[5]]);
                    let len = (p1[0] - cur[0]).hypot(p1[1] - cur[1])
                        + (p2[0] - p1[0]).hypot(p2[1] - p1[1])
                        + (p3[0] - p2[0]).hypot(p3[1] - p2[1]);
                    let steps = ((len / tol.max(1e-3)).sqrt() * 1.5).ceil().clamp(2.0, 64.0) as usize;
                    for i in 1..=steps {
                        let t = i as f64 / steps as f64;
                        let q = bz(&[cur, p1, p2, p3], t);
                        pts.push(q);
                    }
                    cur = p3;
                }
            }
            if pts.len() > 1 && pts[0] == *pts.last().unwrap() {
                pts.pop();
            }
            pts
        })
        .filter(|p| p.len() >= 3)
        .collect()
}

// ---------------------------------------------------------------------------
// Ausgabe

fn r2(v: f64) -> String {
    let x = js_round(v * 1000.0) / 1000.0;
    let x = if x == 0.0 { 0.0 } else { x };
    let s = format!("{x}");
    s
}

fn f3(v: f64) -> String {
    let s = format!("{v:.3}");
    let t = if s.contains('.') { s.trim_end_matches('0').trim_end_matches('.').to_string() } else { s };
    if t == "-0" || t.is_empty() {
        "0".into()
    } else {
        t
    }
}

pub fn sp_d(sp: &Shape) -> String {
    match sp {
        Shape::Ellipse(e) => {
            let [cx, cy, rx, ry, rot] = *e;
            let t = rot.to_radians();
            let (x1, y1) = (cx + rx * t.cos(), cy + rx * t.sin());
            let (x2, y2) = (cx - rx * t.cos(), cy - rx * t.sin());
            let r = format!("{} {} {} 1 0 ", r2(rx), r2(ry), r2(rot));
            format!("M{} {}A{r}{} {}A{r}{} {}Z", r2(x1), r2(y1), r2(x2), r2(y2), r2(x1), r2(y1))
        }
        Shape::Path(c) => {
            let mut s = String::new();
            for cmd in c {
                match *cmd {
                    Cmd::M(x, y) => s += &format!("M{} {}", r2(x), r2(y)),
                    Cmd::L(x, y) => s += &format!("L{} {}", r2(x), r2(y)),
                    Cmd::C(a, b, c2, d, x, y) => s += &format!("C{} {} {} {} {} {}", r2(a), r2(b), r2(c2), r2(d), r2(x), r2(y)),
                    Cmd::Q(a, b, x, y) => s += &format!("Q{} {} {} {}", r2(a), r2(b), r2(x), r2(y)),
                    Cmd::Z => s.push('Z'),
                }
            }
            s
        }
    }
}

pub struct OutSize {
    pub px_w: f64,
    pub px_h: f64,
    pub pt_w: f64,
    pub pt_h: f64,
    pub w: f64,
    pub h: f64,
    pub u: &'static str,
    pub mm_w: f64,
    pub mm_h: f64,
}

pub fn out_size(src_w: usize, src_h: usize, o: &Opts) -> OutSize {
    let (px_w, px_h) = (src_w as f64 * o.scale, src_h as f64 * o.scale);
    let (in_w, in_h) = (px_w / o.dpi, px_h / o.dpi);
    let (w, h, u) = match o.unit {
        Unit::None => (px_w, px_h, ""),
        Unit::Px => (px_w, px_h, "px"),
        Unit::Pt => (in_w * 72.0, in_h * 72.0, "pt"),
        Unit::In => (in_w, in_h, "in"),
        Unit::Cm => (in_w * 2.54, in_h * 2.54, "cm"),
        Unit::Mm => (in_w * 25.4, in_h * 25.4, "mm"),
    };
    OutSize { px_w, px_h, pt_w: in_w * 72.0, pt_h: in_h * 72.0, w, h, u, mm_w: in_w * 25.4, mm_h: in_h * 25.4 }
}

impl Doc {
    pub fn col_of(&self, li: usize) -> String {
        hex(self.gcol(li))
    }

    pub fn build_svg(&self, o: &Opts, png: bool) -> String {
        let z = out_size(self.src_w, self.src_h, o);
        let (w, h) = (self.src_w, self.src_h);
        let mut a = format!(
            "<svg xmlns=\"http://www.w3.org/2000/svg\" version=\"{}\"",
            if o.svg_tiny { "1.2\" baseProfile=\"tiny" } else { "1.1" }
        );
        if o.unit != Unit::None {
            a += &format!(" width=\"{}{}\" height=\"{}{}\"", r2(z.w), z.u, r2(z.h), z.u);
        }
        a += &format!(" viewBox=\"0 0 {w} {h}\"{}>", if png && !o.aa { " shape-rendering=\"crispEdges\"" } else { "" });
        let mut lines = vec![a];
        if o.gap && !o.stroke_only {
            lines.push(format!(
                "<g class=\"gap\" fill=\"none\" stroke-width=\"{}\" stroke-linejoin=\"round\"{}>",
                o.gap_w,
                if o.gap_ns { " vector-effect=\"non-scaling-stroke\"" } else { "" }
            ));
            for l in &self.layers {
                let d: String = l.subs.iter().map(sp_d).collect();
                lines.push(format!("<path stroke=\"{}\" d=\"{d}\"/>", self.col_of(l.idx)));
            }
            lines.push("</g>".into());
        }
        let st = if o.stroke {
            format!(" stroke=\"{}\" stroke-width=\"{}\" stroke-linejoin=\"round\"", hex(o.stroke_color), o.stroke_w)
        } else {
            String::new()
        };
        for (i, l) in self.layers.iter().enumerate() {
            let fill = if o.stroke_only { "none".to_string() } else { self.col_of(l.idx) };
            if o.group_by_color {
                lines.push(format!("<g id=\"farbe-{}\">", i + 1));
                let d: String = l.subs.iter().map(sp_d).collect();
                lines.push(format!("<path fill=\"{fill}\" fill-rule=\"evenodd\"{st} d=\"{d}\"/>"));
                lines.push("</g>".into());
            } else {
                for sp in &l.subs {
                    lines.push(format!("<path fill=\"{fill}\"{st} fill-rule=\"evenodd\" d=\"{}\"/>", sp_d(sp)));
                }
            }
        }
        lines.push("</svg>".into());
        lines.join("\n")
    }

    fn path_ops(sp: &Shape, f: &dyn Fn(f64) -> String, mv: &str, ln: &str, cv: &str, z: &str) -> String {
        let mut s = String::new();
        for c in to_cubics(sp) {
            s += &format!("{} {} {mv}\n", f(c.m[0]), f(c.m[1]));
            for (k, v) in c.segs {
                if k == 'L' {
                    s += &format!("{} {} {ln}\n", f(v[0]), f(v[1]));
                } else {
                    s += &format!("{} {cv}\n", v.iter().map(|x| f(*x)).collect::<Vec<_>>().join(" "));
                }
            }
            s += z;
            s += "\n";
        }
        s
    }

    fn rgbf(c: [u8; 3]) -> String {
        c.iter().map(|v| format!("{:.4}", *v as f64 / 255.0)).collect::<Vec<_>>().join(" ")
    }

    pub fn build_pdf(&self, o: &Opts) -> Vec<u8> {
        let z = out_size(self.src_w, self.src_h, o);
        let (sx, sy) = (z.pt_w / self.src_w as f64, z.pt_h / self.src_h as f64);
        let f = |v: f64| f3(v);
        let mut c = format!("q {} 0 0 {} 0 {} cm 1 j 1 J\n", f(sx), f(-sy), f(z.pt_h));
        if o.gap && !o.stroke_only {
            c += &format!("{} w\n", f(o.gap_w));
            for l in &self.layers {
                c += &format!("{} RG\n", Self::rgbf(self.gcol(l.idx)));
                for sp in &l.subs {
                    c += &Self::path_ops(sp, &f, "m", "l", "c", "h");
                }
                c += "S\n";
            }
        }
        for l in &self.layers {
            let mut p = String::new();
            for sp in &l.subs {
                p += &Self::path_ops(sp, &f, "m", "l", "c", "h");
            }
            if !o.stroke_only {
                c += &format!("{} rg\n", Self::rgbf(self.gcol(l.idx)));
            }
            if o.stroke {
                c += &format!("{} RG {} w\n", Self::rgbf(o.stroke_color), f(o.stroke_w));
            }
            c += &p;
            c += if o.stroke_only {
                if o.stroke {
                    "S"
                } else {
                    "n"
                }
            } else if o.stroke {
                "B*"
            } else {
                "f*"
            };
            c += "\n";
        }
        c += "Q\n";
        let objs = [
            "<< /Type /Catalog /Pages 2 0 R >>".to_string(),
            "<< /Type /Pages /Kids [3 0 R] /Count 1 >>".to_string(),
            format!("<< /Type /Page /Parent 2 0 R /MediaBox [0 0 {} {}] /Contents 4 0 R /Resources << >> >>", f(z.pt_w), f(z.pt_h)),
            format!("<< /Length {} >>\nstream\n{c}endstream", c.len()),
            "<< /Producer (Mesh2STEP Vektorix) >>".to_string(),
        ];
        let mut out = String::from("%PDF-1.4\n");
        let mut offs = Vec::new();
        for (i, ob) in objs.iter().enumerate() {
            offs.push(out.len());
            out += &format!("{} 0 obj\n{ob}\nendobj\n", i + 1);
        }
        let xr = out.len();
        out += &format!("xref\n0 {}\n0000000000 65535 f \n", objs.len() + 1);
        for x in offs {
            out += &format!("{x:010} 00000 n \n");
        }
        out += &format!("trailer\n<< /Size {} /Root 1 0 R /Info 5 0 R >>\nstartxref\n{xr}\n%%EOF\n", objs.len() + 1);
        out.into_bytes()
    }

    pub fn build_eps(&self, o: &Opts) -> String {
        let z = out_size(self.src_w, self.src_h, o);
        let (sx, sy) = (z.pt_w / self.src_w as f64, z.pt_h / self.src_h as f64);
        let f = |v: f64| f3(v);
        let mut s = format!(
            "%!PS-Adobe-3.0 EPSF-3.0\n%%BoundingBox: 0 0 {} {}\n%%Creator: Mesh2STEP Vektorix\n%%LanguageLevel: 2\n%%EndComments\ngsave\n0 {} translate {} {} scale 1 setlinejoin\n",
            z.pt_w.ceil(),
            z.pt_h.ceil(),
            f(z.pt_h),
            f(sx),
            f(-sy)
        );
        if o.gap && !o.stroke_only {
            s += &format!("{} setlinewidth\n", f(o.gap_w));
            for l in &self.layers {
                s += "newpath\n";
                for sp in &l.subs {
                    s += &Self::path_ops(sp, &f, "moveto", "lineto", "curveto", "closepath");
                }
                s += &format!("{} setrgbcolor stroke\n", Self::rgbf(self.gcol(l.idx)));
            }
        }
        for l in &self.layers {
            s += "newpath\n";
            for sp in &l.subs {
                s += &Self::path_ops(sp, &f, "moveto", "lineto", "curveto", "closepath");
            }
            let sc = if o.stroke {
                format!("{} setrgbcolor {} setlinewidth stroke", Self::rgbf(o.stroke_color), f(o.stroke_w))
            } else {
                String::new()
            };
            if o.stroke_only {
                s += &(if sc.is_empty() { "newpath".to_string() } else { sc });
                s += "\n";
            } else {
                s += &format!(
                    "{} setrgbcolor {}\n",
                    Self::rgbf(self.gcol(l.idx)),
                    if !sc.is_empty() { format!("gsave eofill grestore {sc}") } else { "eofill".into() }
                );
            }
        }
        s + "grestore\nshowpage\n%%EOF\n"
    }

    pub fn build_dxf(&self, o: &Opts) -> String {
        let z = out_size(self.src_w, self.src_h, o);
        let k = if matches!(o.unit, Unit::None | Unit::Px) { z.px_w / self.src_w as f64 } else { z.w / self.src_w as f64 };
        let hh = self.src_h as f64;
        let f = |v: f64| format!("{v:.4}");
        let ins = match o.unit {
            Unit::In => 1,
            Unit::Cm => 5,
            Unit::Mm => 4,
            _ => 0,
        };
        let mut s = format!("0\nSECTION\n2\nHEADER\n9\n$ACADVER\n1\nAC1009\n9\n$INSUNITS\n70\n{ins}\n0\nENDSEC\n0\nSECTION\n2\nENTITIES\n");
        for l in &self.layers {
            let ln = format!("FARBE_{}", self.col_of(l.idx)[1..].to_uppercase());
            for sp in &l.subs {
                for c in to_cubics(sp) {
                    let mut pts = vec![c.m];
                    let mut cur = c.m;
                    for (kk, v) in c.segs {
                        if kk == 'L' {
                            cur = [v[0], v[1]];
                            pts.push(cur);
                        } else {
                            for t in 1..=8 {
                                let u = t as f64 / 8.0;
                                pts.push(bz(&[cur, [v[0], v[1]], [v[2], v[3]], [v[4], v[5]]], u));
                            }
                            cur = [v[4], v[5]];
                        }
                    }
                    s += &format!("0\nPOLYLINE\n8\n{ln}\n66\n1\n70\n1\n");
                    for p in pts {
                        s += &format!("0\nVERTEX\n8\n{ln}\n10\n{}\n20\n{}\n", f(p[0] * k), f((hh - p[1]) * k));
                    }
                    s += &format!("0\nSEQEND\n8\n{ln}\n");
                }
            }
        }
        s + "0\nENDSEC\n0\nEOF\n"
    }

    /// Vektorbild rastern (RGBA, Breite w). Für Vorschau und PNG-Export.
    pub fn render_rgba(&self, o: &Opts, w: usize, h: usize, wire: Option<[u8; 3]>) -> Vec<u8> {
        let mut out = vec![0u8; w * h * 4];
        let (sx, sy) = (w as f64 / self.src_w.max(1) as f64, h as f64 / self.src_h.max(1) as f64);
        let ss = if o.aa { 5 } else { 1 };
        for l in &self.layers {
            let col = if let Some(c) = wire { c } else { self.gcol(l.idx) };
            let mut polys: Vec<Vec<Pt>> = Vec::new();
            for sp in &l.subs {
                for p in flatten(sp, 0.35 / sx.max(1e-6)) {
                    polys.push(p.iter().map(|q| [q[0] * sx, q[1] * sy]).collect());
                }
            }
            let mut cov = vec![0f32; w * h];
            if wire.is_some() {
                for p in &polys {
                    stroke_poly(&mut cov, w, h, p, 1.0, ss);
                }
            } else {
                if !o.stroke_only {
                    fill_evenodd(&mut cov, w, h, &polys, ss);
                }
                if o.gap && !o.stroke_only {
                    let gw = if o.gap_ns { o.gap_w } else { o.gap_w * sx };
                    for p in &polys {
                        stroke_poly(&mut cov, w, h, p, gw, ss);
                    }
                }
            }
            composite(&mut out, &cov, col);
            if o.stroke && wire.is_none() {
                let mut sc = vec![0f32; w * h];
                for p in &polys {
                    stroke_poly(&mut sc, w, h, p, o.stroke_w * sx, ss);
                }
                composite(&mut out, &sc, o.stroke_color);
            }
        }
        out
    }

    pub fn build_png(&self, o: &Opts) -> Vec<u8> {
        let z = out_size(self.src_w, self.src_h, o);
        let (mut w, mut h) = (z.px_w.round().max(1.0), z.px_h.round().max(1.0));
        let lim = 8192.0 / w.max(h);
        if lim < 1.0 {
            w = (w * lim).round();
            h = (h * lim).round();
        }
        let rgba = self.render_rgba(o, w as usize, h as usize, None);
        encode_png(w as usize, h as usize, &rgba)
    }

    /// Statistik: (Formen, Knoten)
    pub fn stats(&self) -> (usize, usize) {
        let mut paths = 0;
        let mut nodes = 0;
        for l in &self.layers {
            for sp in &l.subs {
                paths += 1;
                match sp {
                    Shape::Ellipse(_) => nodes += 2,
                    Shape::Path(c) => nodes += c.iter().filter(|x| !matches!(x, Cmd::Z)).count(),
                }
            }
        }
        (paths, nodes)
    }

    /// Farbflächen als 3D-Teile (je Farbe ein Teil) extrudieren.
    /// `width_mm` = Breite des Modells, `height` = Dicke je Farbe, `base` = Sockelplatte (0 = keine).
    pub fn build_3d(&self, o: &Opts, width_mm: f64, height: f64, base: f64) -> Vec<crate::split::Part> {
        let mut oc = o.clone();
        oc.stack = false;
        let layers = if o.stack { vectorize(&self.g, self.w, self.h, self.groups.len(), self.sc, &oc, false) } else { self.layers.clone() };
        let k = width_mm / self.src_w.max(1) as f64;
        let hh = self.src_h as f64;
        let mut parts = Vec::new();
        for (li, l) in layers.iter().enumerate() {
            let mut rings: Vec<Vec<Pt>> = Vec::new();
            for sp in &l.subs {
                for p in flatten(sp, 0.25) {
                    // y nach oben, Millimeter
                    rings.push(p.iter().map(|q| [q[0] * k, (hh - q[1]) * k]).collect());
                }
            }
            let (verts, tris) = extrude_rings(&rings, base, base + height);
            if tris.is_empty() {
                continue;
            }
            let col = vec![(li + 1).min(255) as u8; tris.len()];
            let c = self.gcol(l.idx);
            let mut p = crate::split::Part {
                name: format!("Farbe {} {}", li + 1, hex(c)),
                verts,
                tris,
                col,
                color: (li + 1).min(255) as u8,
                group: 0,
                kind: crate::split::PartKind::ColorPart,
                ..Default::default()
            };
            p.update_stats();
            parts.push(p);
        }
        if base > 0.0 {
            // Sockel: Umriss aller Flächen als Rechteck des Bildes
            let (w, h) = (self.src_w as f64 * k, hh * k);
            let ring = vec![[0.0, 0.0], [w, 0.0], [w, h], [0.0, h]];
            let (verts, tris) = extrude_rings(&[ring], 0.0, base);
            let mut p = crate::split::Part {
                name: "Sockel".into(),
                col: vec![(layers.len() + 1).min(255) as u8; tris.len()],
                verts,
                tris,
                color: (layers.len() + 1).min(255) as u8,
                group: 0,
                kind: crate::split::PartKind::Body,
                ..Default::default()
            };
            p.update_stats();
            parts.push(p);
        }
        parts
    }
}

/// Ringe (gerade/ungerade verschachtelt) zu Körpern zwischen z0 und z1 extrudieren.
pub fn extrude_rings(rings: &[Vec<Pt>], z0: f64, z1: f64) -> (Vec<crate::math::V3>, Vec<[u32; 3]>) {
    use crate::math::v3;
    use crate::poly;
    let rings: Vec<Vec<Pt>> = rings.iter().filter(|r| r.len() >= 3 && poly::signed_area(r).abs() > 1e-9).cloned().collect();
    let n = rings.len();
    // Verschachtelungstiefe je Ring
    let depth: Vec<usize> = (0..n)
        .map(|i| (0..n).filter(|&j| j != i && poly::point_in(&rings[j], rings[i][0]) && poly::signed_area(&rings[j]).abs() > poly::signed_area(&rings[i]).abs()).count())
        .collect();
    let mut verts = Vec::new();
    let mut tris: Vec<[u32; 3]> = Vec::new();
    for i in (0..n).filter(|&i| depth[i] % 2 == 0) {
        // direkte Löcher: Tiefe+1 und in i enthalten
        let holes: Vec<usize> = (0..n)
            .filter(|&j| depth[j] == depth[i] + 1 && poly::point_in(&rings[i], rings[j][0]))
            .collect();
        let mut outer = rings[i].clone();
        if poly::signed_area(&outer) < 0.0 {
            outer.reverse();
        }
        let mut all = vec![outer];
        for &hidx in &holes {
            let mut hr = rings[hidx].clone();
            if poly::signed_area(&hr) > 0.0 {
                hr.reverse();
            }
            all.push(hr);
        }
        let flat: Vec<Pt> = all.concat();
        let t = poly::triangulate(&all);
        if t.is_empty() {
            continue;
        }
        let base = verts.len() as u32;
        let m = flat.len() as u32;
        for p in &flat {
            verts.push(v3(p[0], p[1], z0));
        }
        for p in &flat {
            verts.push(v3(p[0], p[1], z1));
        }
        for tr in &t {
            let (a, b, c) = (tr[0] as u32, tr[1] as u32, tr[2] as u32);
            tris.push([base + m + a, base + m + b, base + m + c]); // oben
            tris.push([base + a, base + c, base + b]); // unten
        }
        // Wände je Ring (Ringe liegen in der Reihenfolge von `all`)
        let mut off = 0u32;
        for r in &all {
            let k = r.len() as u32;
            for j in 0..k {
                let (a, b) = (base + off + j, base + off + (j + 1) % k);
                tris.push([a, b, b + m]);
                tris.push([a, b + m, a + m]);
            }
            off += k;
        }
    }
    (verts, tris)
}

fn composite(out: &mut [u8], cov: &[f32], c: [u8; 3]) {
    for (p, &a) in cov.iter().enumerate() {
        if a <= 0.0 {
            continue;
        }
        let a = a.min(1.0);
        let j = p * 4;
        let da = out[j + 3] as f32 / 255.0;
        let oa = a + da * (1.0 - a);
        for k in 0..3 {
            let dc = out[j + k] as f32 / 255.0 * da;
            let v = (c[k] as f32 / 255.0 * a + dc * (1.0 - a)) / oa.max(1e-6);
            out[j + k] = (v * 255.0).round().clamp(0.0, 255.0) as u8;
        }
        out[j + 3] = (oa * 255.0).round().clamp(0.0, 255.0) as u8;
    }
}

/// Gerade/ungerade Füllung mit Kantenglättung (ss Unterzeilen je Pixel).
pub fn fill_evenodd(cov: &mut [f32], w: usize, h: usize, polys: &[Vec<Pt>], ss: usize) {
    let mut edges: Vec<(f64, f64, f64, f64)> = Vec::new();
    for p in polys {
        for i in 0..p.len() {
            let (a, b) = (p[i], p[(i + 1) % p.len()]);
            if a[1] != b[1] {
                edges.push((a[0], a[1], b[0], b[1]));
            }
        }
    }
    if edges.is_empty() {
        return;
    }
    let ymin = edges.iter().fold(f64::MAX, |m, e| m.min(e.1).min(e.3)).max(0.0).floor() as usize;
    let ymax = (edges.iter().fold(f64::MIN, |m, e| m.max(e.1).max(e.3)).ceil() as usize).min(h);
    // Kanten nach oberem y sortiert für einfache aktive Liste
    let mut xs: Vec<f64> = Vec::new();
    let mut row = vec![0f32; w + 1];
    let wgt = 1.0 / ss as f32;
    for y in ymin..ymax {
        for v in row.iter_mut() {
            *v = 0.0;
        }
        for s in 0..ss {
            let yy = y as f64 + (s as f64 + 0.5) / ss as f64;
            xs.clear();
            for e in &edges {
                let (y0, y1) = (e.1.min(e.3), e.1.max(e.3));
                if yy >= y0 && yy < y1 {
                    let t = (yy - e.1) / (e.3 - e.1);
                    xs.push(e.0 + t * (e.2 - e.0));
                }
            }
            xs.sort_by(|a, b| a.partial_cmp(b).unwrap());
            for pair in xs.chunks(2) {
                if pair.len() < 2 {
                    break;
                }
                let (x0, x1) = (pair[0].max(0.0), pair[1].min(w as f64));
                if x1 <= x0 {
                    continue;
                }
                if ss == 1 {
                    // ohne Glättung: Pixelmitte entscheidet
                    let a = (x0 - 0.5).ceil().max(0.0) as usize;
                    let b = ((x1 - 0.5).ceil().max(0.0) as usize).min(w);
                    for x in a..b {
                        row[x] += 1.0;
                    }
                    continue;
                }
                let (ix0, ix1) = (x0.floor() as usize, x1.floor() as usize);
                if ix0 == ix1 {
                    row[ix0.min(w - 1)] += (x1 - x0) as f32 * wgt;
                } else {
                    row[ix0] += (ix0 as f64 + 1.0 - x0) as f32 * wgt;
                    for x in ix0 + 1..ix1.min(w) {
                        row[x] += wgt;
                    }
                    if ix1 < w {
                        row[ix1] += (x1 - ix1 as f64) as f32 * wgt;
                    }
                }
            }
        }
        for x in 0..w {
            let v = row[x].min(1.0);
            let c = &mut cov[y * w + x];
            if v > *c {
                *c = v;
            }
        }
    }
}

/// Polygonumriss mit Breite bw nachziehen (Segmente als Vierecke, Ecken als Achtecke).
fn stroke_poly(cov: &mut [f32], w: usize, h: usize, p: &[Pt], bw: f64, ss: usize) {
    let r = bw / 2.0;
    if r <= 0.0 {
        return;
    }
    for i in 0..p.len() {
        let (a, b) = (p[i], p[(i + 1) % p.len()]);
        let d = nrm(sub(b, a));
        let nn = [-d[1] * r, d[0] * r];
        let quad = vec![addp(a, nn), addp(b, nn), sub(b, nn), sub(a, nn)];
        let oct: Vec<Pt> = (0..8)
            .map(|k| {
                let t = k as f64 / 8.0 * std::f64::consts::TAU;
                [a[0] + r * t.cos(), a[1] + r * t.sin()]
            })
            .collect();
        fill_evenodd(cov, w, h, &[quad], ss);
        if r > 0.4 {
            fill_evenodd(cov, w, h, &[oct], ss);
        }
    }
}

/// PNG kodieren (RGBA, 8 Bit).
pub fn encode_png(w: usize, h: usize, rgba: &[u8]) -> Vec<u8> {
    let mut raw = Vec::with_capacity((w * 4 + 1) * h);
    for y in 0..h {
        raw.push(0);
        raw.extend_from_slice(&rgba[y * w * 4..(y + 1) * w * 4]);
    }
    let mut z = vec![0x78, 0x01];
    z.extend(crate::zip::deflate(&raw));
    let (mut a, mut b) = (1u32, 0u32);
    for &x in &raw {
        a = (a + x as u32) % 65521;
        b = (b + a) % 65521;
    }
    z.extend_from_slice(&((b << 16) | a).to_be_bytes());
    let mut out = vec![0x89, b'P', b'N', b'G', 0x0D, 0x0A, 0x1A, 0x0A];
    let chunk = |ty: &[u8], data: &[u8], out: &mut Vec<u8>| {
        out.extend_from_slice(&(data.len() as u32).to_be_bytes());
        let mut c = ty.to_vec();
        c.extend_from_slice(data);
        out.extend_from_slice(&c);
        out.extend_from_slice(&crate::zip::crc32(&c).to_be_bytes());
    };
    let mut ihdr = Vec::new();
    ihdr.extend_from_slice(&(w as u32).to_be_bytes());
    ihdr.extend_from_slice(&(h as u32).to_be_bytes());
    ihdr.extend_from_slice(&[8, 6, 0, 0, 0]);
    chunk(b"IHDR", &ihdr, &mut out);
    chunk(b"IDAT", &z, &mut out);
    chunk(b"IEND", &[], &mut out);
    out
}

/// Beispielbild (Berge, Sonne, See) für die Demo.
pub fn sample_image() -> (usize, usize, Vec<u8>) {
    let (w, h) = (640usize, 440usize);
    let mut img = vec![0u8; w * h * 4];
    let fill = |polys: Vec<Vec<Pt>>, c: [u8; 3], img: &mut Vec<u8>| {
        let mut cov = vec![0f32; w * h];
        fill_evenodd(&mut cov, w, h, &polys, 4);
        composite(img, &cov, c);
    };
    fill(vec![vec![[0.0, 0.0], [640.0, 0.0], [640.0, 440.0], [0.0, 440.0]]], [0xf4, 0xe4, 0xc1], &mut img);
    let circ = |cx: f64, cy: f64, rx: f64, ry: f64| -> Vec<Pt> {
        (0..96).map(|k| {
            let t = k as f64 / 96.0 * std::f64::consts::TAU;
            [cx + rx * t.cos(), cy + ry * t.sin()]
        }).collect()
    };
    fill(vec![circ(468.0, 128.0, 70.0, 70.0)], [0xef, 0x8a, 0x25], &mut img);
    fill(
        vec![vec![[0.0, 300.0], [120.0, 170.0], [210.0, 250.0], [330.0, 120.0], [470.0, 270.0], [560.0, 200.0], [640.0, 260.0], [640.0, 440.0], [0.0, 440.0]]],
        [0x8a, 0x9b, 0xc0],
        &mut img,
    );
    fill(vec![vec![[330.0, 120.0], [372.0, 165.0], [350.0, 160.0], [330.0, 178.0], [310.0, 158.0], [292.0, 163.0]]], [255, 255, 255], &mut img);
    let mut hill = vec![[0.0, 360.0]];
    for i in 1..=20 {
        let t = i as f64 / 20.0;
        hill.push(bz(&[[0.0, 360.0], [140.0, 250.0], [250.0, 300.0], [330.0, 330.0]], t));
    }
    for i in 1..=20 {
        let t = i as f64 / 20.0;
        hill.push(bz(&[[330.0, 330.0], [430.0, 370.0], [520.0, 260.0], [640.0, 320.0]], t));
    }
    hill.push([640.0, 440.0]);
    hill.push([0.0, 440.0]);
    fill(vec![hill], [0x3c, 0x4d, 0x77], &mut img);
    fill(vec![circ(250.0, 395.0, 150.0, 30.0)], [0x4a, 0xa3, 0xc6], &mut img);
    for (tx, ty) in [(70.0, 330.0), (100.0, 322.0), (520.0, 325.0), (555.0, 315.0), (588.0, 322.0)] {
        fill(vec![vec![[tx, ty - 58.0], [tx + 20.0, ty], [tx - 20.0, ty]]], [0x2c, 0x6a, 0x49], &mut img);
    }
    (w, h, img)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sample_pipeline() {
        let (w, h, img) = sample_image();
        let mut d = Doc::new(w, h, img);
        let o = Opts::default();
        d.run(0, &o, 0);
        assert!(d.q.pal.len() >= 6, "{} Töne", d.q.pal.len());
        assert!(d.q.auto >= 3 && d.q.auto <= 9, "auto {}", d.q.auto);
        assert!(!d.layers.is_empty());
        let (paths, nodes) = d.stats();
        assert!(paths >= 5 && paths < 200, "{paths} Formen");
        assert!(nodes < 3000, "{nodes} Knoten");
        // Sonne als Kreis erkannt
        assert!(d.layers.iter().any(|l| l.subs.iter().any(|s| matches!(s, Shape::Ellipse(e) if (e[2] - 70.0).abs() < 3.0))));
        let svg = d.build_svg(&o, false);
        assert!(svg.starts_with("<svg") && svg.ends_with("</svg>"));
        let pdf = d.build_pdf(&o);
        assert!(pdf.starts_with(b"%PDF"));
        let eps = d.build_eps(&o);
        assert!(eps.contains("eofill"));
        let dxf = d.build_dxf(&o);
        assert!(dxf.contains("POLYLINE"));
        let png = d.build_png(&o);
        assert_eq!(&png[1..4], b"PNG");
        // gerendert ähnlich wie das Original
        let r = d.render_rgba(&o, w, h, None);
        let mut diff = 0u64;
        for p in 0..w * h {
            for k in 0..3 {
                diff += (r[p * 4 + k] as i64 - d.src[p * 4 + k] as i64).unsigned_abs();
            }
        }
        let mean = diff as f64 / (w * h * 3) as f64;
        assert!(mean < 6.0, "mittlere Abweichung {mean}");
        // 3D
        let parts = d.build_3d(&o, 100.0, 2.0, 0.0);
        assert_eq!(parts.len(), d.layers.len());
        for p in &parts {
            assert!(p.volume > 0.0, "{} {}", p.name, p.volume);
        }
        let area: f64 = parts.iter().map(|p| p.volume / 2.0).sum();
        let expect = 100.0 * 100.0 * 440.0 / 640.0;
        assert!((area - expect).abs() < 0.03 * expect, "{area} {expect}");
        // Bearbeitung: Sonne transparent
        let (px, v, _) = d.region_at(468 * d.w / w, 128 * d.h / h).unwrap();
        assert!(px.len() > 100 && v >= 0);
        let before = d.layers.len();
        d.edits.push(Edit { x: 468 * d.w / w, y: 128 * d.h / h, to: -1 });
        d.run(2, &o, 0);
        assert_eq!(d.layers.len(), before - 1);
    }

    #[test]
    fn presets_monotone() {
        let (w, h, img) = sample_image();
        let mut d = Doc::new(w, h, img);
        let o = Opts { res: 400, ..Default::default() };
        d.rasterize(&o);
        d.quantize(0);
        for k in 1..=d.q.pal.len() {
            assert_eq!(d.q.presets[k].len(), k);
            let total: usize = d.q.presets[k].iter().map(|g| g.len()).sum();
            assert_eq!(total, d.q.pal.len());
        }
    }

    #[test]
    fn png_roundtrip_header() {
        let p = encode_png(2, 2, &[255, 0, 0, 255, 0, 255, 0, 255, 0, 0, 255, 255, 0, 0, 0, 0]);
        assert_eq!(&p[12..16], b"IHDR");
    }
}

/// Einfacher PNG-Leser (8 Bit; Grau, RGB, Palette, mit/ohne Alpha; ohne Interlacing) für die Kommandozeile.
pub fn decode_png(data: &[u8]) -> Result<(usize, usize, Vec<u8>), String> {
    if data.len() < 8 || &data[1..4] != b"PNG" {
        return Err("keine PNG-Datei".into());
    }
    let mut pos = 8;
    let (mut w, mut h, mut depth, mut ct, mut inter) = (0usize, 0usize, 0u8, 0u8, 0u8);
    let mut idat = Vec::new();
    let mut plte: Vec<u8> = Vec::new();
    let mut trns: Vec<u8> = Vec::new();
    while pos + 8 <= data.len() {
        let len = u32::from_be_bytes([data[pos], data[pos + 1], data[pos + 2], data[pos + 3]]) as usize;
        let ty = &data[pos + 4..pos + 8];
        let body = data.get(pos + 8..pos + 8 + len).ok_or("PNG abgeschnitten")?;
        match ty {
            b"IHDR" => {
                w = u32::from_be_bytes([body[0], body[1], body[2], body[3]]) as usize;
                h = u32::from_be_bytes([body[4], body[5], body[6], body[7]]) as usize;
                depth = body[8];
                ct = body[9];
                inter = body[12];
            }
            b"PLTE" => plte = body.to_vec(),
            b"tRNS" => trns = body.to_vec(),
            b"IDAT" => idat.extend_from_slice(body),
            b"IEND" => break,
            _ => {}
        }
        pos += 12 + len;
    }
    if depth != 8 || inter != 0 {
        return Err("nur 8-Bit-PNG ohne Interlacing".into());
    }
    let ch = match ct {
        0 => 1,
        2 => 3,
        3 => 1,
        4 => 2,
        6 => 4,
        _ => return Err("unbekannter PNG-Farbtyp".into()),
    };
    let raw = crate::zip::inflate(idat.get(2..).ok_or("leer")?, (w * ch + 1) * h)?;
    let stride = w * ch;
    let mut img = vec![0u8; stride * h];
    for y in 0..h {
        let f = raw[y * (stride + 1)];
        let line = &raw[y * (stride + 1) + 1..(y + 1) * (stride + 1)];
        for x in 0..stride {
            let a = if x >= ch { img[y * stride + x - ch] as i32 } else { 0 };
            let b = if y > 0 { img[(y - 1) * stride + x] as i32 } else { 0 };
            let c = if x >= ch && y > 0 { img[(y - 1) * stride + x - ch] as i32 } else { 0 };
            let v = line[x] as i32;
            let r = match f {
                0 => v,
                1 => v + a,
                2 => v + b,
                3 => v + (a + b) / 2,
                _ => {
                    let p = a + b - c;
                    let (pa, pb, pc) = ((p - a).abs(), (p - b).abs(), (p - c).abs());
                    v + if pa <= pb && pa <= pc {
                        a
                    } else if pb <= pc {
                        b
                    } else {
                        c
                    }
                }
            };
            img[y * stride + x] = (r & 255) as u8;
        }
    }
    let mut out = vec![0u8; w * h * 4];
    for p in 0..w * h {
        let s = &img[p * ch..p * ch + ch];
        let px = match ct {
            0 => [s[0], s[0], s[0], 255],
            2 => [s[0], s[1], s[2], 255],
            3 => {
                let i = s[0] as usize;
                [plte.get(i * 3).copied().unwrap_or(0), plte.get(i * 3 + 1).copied().unwrap_or(0), plte.get(i * 3 + 2).copied().unwrap_or(0), trns.get(i).copied().unwrap_or(255)]
            }
            4 => [s[0], s[0], s[0], s[1]],
            _ => [s[0], s[1], s[2], s[3]],
        };
        out[p * 4..p * 4 + 4].copy_from_slice(&px);
    }
    Ok((w, h, out))
}
