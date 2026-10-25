//! Einlesen von STEP-Dateien (ISO 10303-21, AP203/AP214/AP242) für den Maßvergleich.
//!
//! Gelesen werden die B-Rep-Flächen (ADVANCED_FACE) mit ihren Randkurven. Jede Fläche
//! wird im Parameterraum trianguliert und so lange verfeinert, bis die Dreiecke höchstens
//! `defl` von der echten Fläche abweichen. Unterstützt: Ebene, Zylinder, Kegel, Kugel,
//! Torus und B-Spline-/NURBS-Flächen; Kanten als Gerade, Kreis, Ellipse, B-Spline, Polylinie.
//! Unbekannte Flächenarten werden aus ihren Randkurven eben angenähert und gemeldet.

use crate::math::{v3, V3};
use crate::poly;
use std::collections::HashMap;
use std::f64::consts::{PI, TAU};

// ---------------------------------------------------------------------------
// Teil 21: Zerlegen der Datei

#[derive(Clone, Debug)]
enum Val {
    Ref(u32),
    Num(f64),
    Str(String),
    Enum(String),
    List(Vec<Val>),
    Typed(#[allow(dead_code)] String, Vec<Val>),
    Null,
}

impl Val {
    fn r(&self) -> Option<u32> {
        match self {
            Val::Ref(i) => Some(*i),
            _ => None,
        }
    }
    fn f(&self) -> Option<f64> {
        match self {
            Val::Num(x) => Some(*x),
            Val::Typed(_, v) => v.first().and_then(|x| x.f()),
            _ => None,
        }
    }
    fn list(&self) -> &[Val] {
        match self {
            Val::List(v) => v,
            _ => &[],
        }
    }
    fn b(&self) -> bool {
        matches!(self, Val::Enum(e) if e == "T")
    }
    fn e(&self) -> &str {
        match self {
            Val::Enum(e) => e,
            _ => "",
        }
    }
}

/// Ein Eintrag; komplexe Einträge haben mehrere Teile (Typ, Argumente).
#[derive(Clone, Debug)]
struct Ent {
    parts: Vec<(String, Vec<Val>)>,
}

impl Ent {
    fn name(&self) -> &str {
        if self.parts.len() == 1 {
            &self.parts[0].0
        } else {
            ""
        }
    }
    fn args(&self) -> &[Val] {
        &self.parts[0].1
    }
    fn part(&self, n: &str) -> Option<&[Val]> {
        self.parts.iter().find(|p| p.0 == n).map(|p| p.1.as_slice())
    }
    fn has(&self, n: &str) -> bool {
        self.parts.iter().any(|p| p.0 == n)
    }
}

struct Lexer<'a> {
    s: &'a [u8],
    i: usize,
}

impl<'a> Lexer<'a> {
    fn ws(&mut self) {
        loop {
            while self.i < self.s.len() && (self.s[self.i] as char).is_ascii_whitespace() {
                self.i += 1;
            }
            if self.i + 1 < self.s.len() && self.s[self.i] == b'/' && self.s[self.i + 1] == b'*' {
                self.i += 2;
                while self.i + 1 < self.s.len() && !(self.s[self.i] == b'*' && self.s[self.i + 1] == b'/') {
                    self.i += 1;
                }
                self.i = (self.i + 2).min(self.s.len());
            } else {
                break;
            }
        }
    }
    fn peek(&mut self) -> u8 {
        self.ws();
        if self.i < self.s.len() {
            self.s[self.i]
        } else {
            0
        }
    }
    fn ident(&mut self) -> String {
        self.ws();
        let st = self.i;
        while self.i < self.s.len() {
            let c = self.s[self.i];
            if c.is_ascii_alphanumeric() || c == b'_' || c == b'-' {
                self.i += 1;
            } else {
                break;
            }
        }
        String::from_utf8_lossy(&self.s[st..self.i]).to_ascii_uppercase()
    }
    fn value(&mut self) -> Result<Val, String> {
        let c = self.peek();
        match c {
            b'#' => {
                self.i += 1;
                let st = self.i;
                while self.i < self.s.len() && self.s[self.i].is_ascii_digit() {
                    self.i += 1;
                }
                let n: u32 = std::str::from_utf8(&self.s[st..self.i]).ok().and_then(|x| x.parse().ok()).ok_or("Ungültiger Verweis")?;
                Ok(Val::Ref(n))
            }
            b'\'' => {
                self.i += 1;
                let mut out = Vec::new();
                while self.i < self.s.len() {
                    if self.s[self.i] == b'\'' {
                        if self.i + 1 < self.s.len() && self.s[self.i + 1] == b'\'' {
                            out.push(b'\'');
                            self.i += 2;
                            continue;
                        }
                        self.i += 1;
                        break;
                    }
                    out.push(self.s[self.i]);
                    self.i += 1;
                }
                Ok(Val::Str(String::from_utf8_lossy(&out).into_owned()))
            }
            b'.' => {
                self.i += 1;
                let st = self.i;
                while self.i < self.s.len() && self.s[self.i] != b'.' {
                    self.i += 1;
                }
                let e = String::from_utf8_lossy(&self.s[st..self.i]).to_ascii_uppercase();
                self.i += 1;
                Ok(Val::Enum(e))
            }
            b'(' => {
                self.i += 1;
                let mut v = Vec::new();
                if self.peek() == b')' {
                    self.i += 1;
                    return Ok(Val::List(v));
                }
                loop {
                    v.push(self.value()?);
                    match self.peek() {
                        b',' => self.i += 1,
                        b')' => {
                            self.i += 1;
                            break;
                        }
                        _ => return Err("Liste nicht geschlossen".into()),
                    }
                }
                Ok(Val::List(v))
            }
            b'$' => {
                self.i += 1;
                Ok(Val::Null)
            }
            b'*' => {
                self.i += 1;
                Ok(Val::Null)
            }
            b'"' => {
                // Binärwert – wird nicht gebraucht
                self.i += 1;
                while self.i < self.s.len() && self.s[self.i] != b'"' {
                    self.i += 1;
                }
                self.i += 1;
                Ok(Val::Null)
            }
            c if c == b'-' || c == b'+' || c.is_ascii_digit() => {
                let st = self.i;
                self.i += 1;
                while self.i < self.s.len() {
                    let d = self.s[self.i];
                    if d.is_ascii_digit() || d == b'.' || d == b'E' || d == b'e' || d == b'-' || d == b'+' {
                        self.i += 1;
                    } else {
                        break;
                    }
                }
                let txt = std::str::from_utf8(&self.s[st..self.i]).unwrap_or("0");
                Ok(Val::Num(txt.parse().unwrap_or(0.0)))
            }
            c if c.is_ascii_alphabetic() => {
                let name = self.ident();
                match self.value()? {
                    Val::List(v) => Ok(Val::Typed(name, v)),
                    other => Ok(Val::Typed(name, vec![other])),
                }
            }
            _ => Err(format!("Unerwartetes Zeichen in der STEP-Datei ({})", c as char)),
        }
    }
}

fn parse(data: &[u8]) -> Result<HashMap<u32, Ent>, String> {
    let text = data;
    // DATA-Abschnitt suchen
    let find = |pat: &[u8], from: usize| -> Option<usize> { text[from..].windows(pat.len()).position(|w| w.eq_ignore_ascii_case(pat)).map(|p| p + from) };
    if find(b"ISO-10303-21", 0).is_none() {
        return Err("Das ist keine STEP-Datei (Kopfzeile ISO-10303-21 fehlt).".into());
    }
    let mut ents = HashMap::new();
    let mut pos = 0;
    while let Some(d) = find(b"DATA", pos) {
        // nur „DATA;“ bzw. „DATA(…);“ als Abschnittsbeginn werten
        let mut lx = Lexer { s: text, i: d + 4 };
        let c = lx.peek();
        if c != b';' && c != b'(' {
            pos = d + 4;
            continue;
        }
        while lx.i < text.len() && text[lx.i] != b';' {
            lx.i += 1;
        }
        lx.i += 1;
        loop {
            let c = lx.peek();
            if c == 0 {
                break;
            }
            if c != b'#' {
                let w = lx.ident();
                if w == "ENDSEC" {
                    break;
                }
                // unbekanntes Wort: bis zum Semikolon überspringen
                while lx.i < text.len() && text[lx.i] != b';' {
                    lx.i += 1;
                }
                lx.i += 1;
                continue;
            }
            lx.i += 1;
            let st = lx.i;
            while lx.i < text.len() && text[lx.i].is_ascii_digit() {
                lx.i += 1;
            }
            let id: u32 = std::str::from_utf8(&text[st..lx.i]).ok().and_then(|x| x.parse().ok()).unwrap_or(0);
            if lx.peek() != b'=' {
                return Err("Fehlerhafte Zeile in der STEP-Datei".into());
            }
            lx.i += 1;
            let mut parts = Vec::new();
            if lx.peek() == b'(' {
                // komplexer Eintrag
                lx.i += 1;
                while lx.peek() != b')' && lx.peek() != 0 {
                    let n = lx.ident();
                    if n.is_empty() {
                        return Err("Fehlerhafter komplexer Eintrag".into());
                    }
                    let a = match lx.value()? {
                        Val::List(v) => v,
                        o => vec![o],
                    };
                    parts.push((n, a));
                }
                lx.i += 1;
            } else {
                let n = lx.ident();
                let a = match lx.value()? {
                    Val::List(v) => v,
                    o => vec![o],
                };
                parts.push((n, a));
            }
            while lx.i < text.len() && text[lx.i] != b';' {
                lx.i += 1;
            }
            lx.i += 1;
            ents.insert(id, Ent { parts });
        }
        pos = lx.i;
    }
    if ents.is_empty() {
        return Err("Die STEP-Datei enthält keine Daten.".into());
    }
    Ok(ents)
}

// ---------------------------------------------------------------------------
// Geometrie

#[derive(Clone, Copy, Debug)]
pub struct Frame {
    pub o: V3,
    pub x: V3,
    pub y: V3,
    pub z: V3,
}

impl Frame {
    fn local(&self, p: V3) -> V3 {
        let d = p - self.o;
        v3(d.dot(self.x), d.dot(self.y), d.dot(self.z))
    }
    fn world(&self, l: V3) -> V3 {
        self.o + self.x * l.x + self.y * l.y + self.z * l.z
    }
}

#[derive(Clone, Debug)]
pub struct Nurbs {
    pub deg: usize,
    pub knots: Vec<f64>,
    /// Kontrollpunkte (homogen: x·w, y·w, z·w, w)
    pub cp: Vec<[f64; 4]>,
}

/// Knotenintervall von t (n = Anzahl Kontrollpunkte − 1, p = Grad)
fn find_span(knots: &[f64], n: usize, p: usize, t: f64) -> usize {
    if t >= knots[n + 1] {
        // letztes nicht leeres Intervall
        let mut i = n;
        while i > p && knots[i] >= knots[n + 1] {
            i -= 1;
        }
        return i;
    }
    if t <= knots[p] {
        let mut i = p;
        while i < n && knots[i + 1] <= knots[p] {
            i += 1;
        }
        return i;
    }
    let (mut lo, mut hi) = (p, n + 1);
    let mut mid = (lo + hi) / 2;
    while t < knots[mid] || t >= knots[mid + 1] {
        if t < knots[mid] {
            hi = mid;
        } else {
            lo = mid;
        }
        mid = (lo + hi) / 2;
    }
    mid
}

/// Nicht verschwindende Basisfunktionen N[i-p..=i] (höchstens Grad 15)
fn basis_funs(knots: &[f64], i: usize, t: f64, p: usize, n: &mut [f64; 16]) {
    let mut left = [0.0; 16];
    let mut right = [0.0; 16];
    n[0] = 1.0;
    for j in 1..=p {
        left[j] = t - knots[i + 1 - j];
        right[j] = knots[i + j] - t;
        let mut saved = 0.0;
        for r in 0..j {
            let den = right[r + 1] + left[j - r];
            let tmp = if den.abs() > 1e-300 { n[r] / den } else { 0.0 };
            n[r] = saved + right[r + 1] * tmp;
            saved = left[j - r] * tmp;
        }
        n[j] = saved;
    }
}

impl Nurbs {
    fn range(&self) -> (f64, f64) {
        (self.knots[self.deg], self.knots[self.cp.len()])
    }
    fn eval(&self, t: f64) -> V3 {
        let (a, b) = self.range();
        let t = t.clamp(a, b);
        let s = find_span(&self.knots, self.cp.len() - 1, self.deg, t);
        let mut n = [0.0; 16];
        basis_funs(&self.knots, s, t, self.deg, &mut n);
        let mut h = [0.0; 4];
        for j in 0..=self.deg {
            let c = self.cp[s - self.deg + j];
            for k in 0..4 {
                h[k] += n[j] * c[k];
            }
        }
        if h[3].abs() < 1e-300 {
            return v3(h[0], h[1], h[2]);
        }
        v3(h[0] / h[3], h[1] / h[3], h[2] / h[3])
    }
}

#[derive(Clone, Debug)]
pub struct NurbsSurf {
    pub du: usize,
    pub dv: usize,
    pub ku: Vec<f64>,
    pub kv: Vec<f64>,
    /// [u][v]
    pub cp: Vec<Vec<[f64; 4]>>,
}

impl NurbsSurf {
    fn rng(&self) -> ((f64, f64), (f64, f64)) {
        ((self.ku[self.du], self.ku[self.cp.len()]), (self.kv[self.dv], self.kv[self.cp[0].len()]))
    }
    fn eval(&self, u: f64, v: f64) -> V3 {
        let ((u0, u1), (v0, v1)) = self.rng();
        let (u, v) = (u.clamp(u0, u1), v.clamp(v0, v1));
        let su = find_span(&self.ku, self.cp.len() - 1, self.du, u);
        let sv = find_span(&self.kv, self.cp[0].len() - 1, self.dv, v);
        let (mut nu, mut nv) = ([0.0; 16], [0.0; 16]);
        basis_funs(&self.ku, su, u, self.du, &mut nu);
        basis_funs(&self.kv, sv, v, self.dv, &mut nv);
        let mut h = [0.0; 4];
        for a in 0..=self.du {
            for b in 0..=self.dv {
                let c = self.cp[su - self.du + a][sv - self.dv + b];
                let w = nu[a] * nv[b];
                for k in 0..4 {
                    h[k] += w * c[k];
                }
            }
        }
        if h[3].abs() < 1e-300 {
            return v3(h[0], h[1], h[2]);
        }
        v3(h[0] / h[3], h[1] / h[3], h[2] / h[3])
    }
}

/// Flächenart einer gelesenen CAD-Fläche.
#[derive(Clone, Debug)]
pub enum CadSurf {
    Plane(Frame),
    Cyl(Frame, f64),
    /// Radius am Ursprung, halber Öffnungswinkel (Bogenmaß)
    Cone(Frame, f64, f64),
    Sphere(Frame, f64),
    Torus(Frame, f64, f64),
    Nurbs(Box<NurbsSurf>),
    /// nicht unterstützt: aus den Randkurven eben angenähert
    Other(String),
}

impl CadSurf {
    pub fn kind_de(&self) -> &'static str {
        match self {
            CadSurf::Plane(_) => "Ebene",
            CadSurf::Cyl(..) => "Zylinder",
            CadSurf::Cone(..) => "Kegel",
            CadSurf::Sphere(..) => "Kugel",
            CadSurf::Torus(..) => "Torus",
            CadSurf::Nurbs(_) => "Freiform",
            CadSurf::Other(_) => "Sonstige",
        }
    }
    fn periodic_u(&self) -> bool {
        matches!(self, CadSurf::Cyl(..) | CadSurf::Cone(..) | CadSurf::Sphere(..) | CadSurf::Torus(..))
    }
    /// Skalierung des Winkels, damit u ungefähr in mm misst (bessere Dreiecke)
    fn uscale(&self) -> f64 {
        match self {
            CadSurf::Cyl(_, r) => r.max(1e-3),
            CadSurf::Cone(_, r, _) => r.abs().max(1.0),
            CadSurf::Sphere(_, r) => r.max(1e-3),
            CadSurf::Torus(_, rmaj, _) => rmaj.max(1e-3),
            _ => 1.0,
        }
    }
    fn vscale(&self) -> f64 {
        match self {
            CadSurf::Sphere(_, r) => r.max(1e-3),
            CadSurf::Torus(_, _, rmin) => rmin.max(1e-3),
            _ => 1.0,
        }
    }
    /// Punkt aus Parametern (u in skalierten Winkeleinheiten)
    fn point(&self, u: f64, v: f64) -> V3 {
        match self {
            CadSurf::Plane(f) => f.world(v3(u, v, 0.0)),
            CadSurf::Cyl(f, r) => {
                let a = u / self.uscale();
                f.world(v3(r * a.cos(), r * a.sin(), v))
            }
            CadSurf::Cone(f, r, ang) => {
                let a = u / self.uscale();
                let rr = r + v * ang.tan();
                f.world(v3(rr * a.cos(), rr * a.sin(), v))
            }
            CadSurf::Sphere(f, r) => {
                let (a, b) = (u / self.uscale(), v / self.vscale());
                f.world(v3(r * b.cos() * a.cos(), r * b.cos() * a.sin(), r * b.sin()))
            }
            CadSurf::Torus(f, rmaj, rmin) => {
                let (a, b) = (u / self.uscale(), v / self.vscale());
                let rr = rmaj + rmin * b.cos();
                f.world(v3(rr * a.cos(), rr * a.sin(), rmin * b.sin()))
            }
            CadSurf::Nurbs(s) => s.eval(u, v),
            CadSurf::Other(_) => V3::ZERO,
        }
    }
    /// natürliche Normale (∂u × ∂v) numerisch
    fn normal(&self, u: f64, v: f64) -> V3 {
        let h = 1e-5 * (1.0 + u.abs().max(v.abs()) * 1e-3);
        let pu = self.point(u + h, v) - self.point(u - h, v);
        let pv = self.point(u, v + h) - self.point(u, v - h);
        pu.cross(pv).normalized()
    }
    /// Größte Kantenlänge (u, v im Parameterraum), damit die Sehne ≤ defl bleibt.
    /// Unendlich = in dieser Richtung keine Grenze (gerade Mantellinien).
    fn target_steps(&self, defl: f64) -> (f64, f64) {
        let s = |rho: f64| (8.0 * rho.max(1e-6) * defl).sqrt();
        match self {
            CadSurf::Sphere(_, r) => (s(*r), s(*r)),
            CadSurf::Torus(_, rmaj, rmin) => {
                let st = s(rmin.abs().min(rmaj.abs()));
                (st * rmaj.abs() / (rmaj.abs() + rmin.abs()).max(1e-9), st)
            }
            CadSurf::Nurbs(n) => {
                // Krümmung aus zweiten Ableitungen (Normalanteil) auf einem Raster schätzen
                let ((u0, u1), (v0, v1)) = n.rng();
                let (hu, hv) = ((u1 - u0) / 16.0, (v1 - v0) / 16.0);
                let (mut cu, mut cv) = (0.0f64, 0.0f64);
                for i in 1..16 {
                    for j in 1..16 {
                        let (u, v) = (u0 + i as f64 * hu, v0 + j as f64 * hv);
                        let p = n.eval(u, v);
                        let (pu0, pu1) = (n.eval(u - hu, v), n.eval(u + hu, v));
                        let (pv0, pv1) = (n.eval(u, v - hv), n.eval(u, v + hv));
                        let nn = (pu1 - pu0).cross(pv1 - pv0).normalized();
                        let suu = (pu1 + pu0 - p * 2.0) / (hu * hu);
                        let svv = (pv1 + pv0 - p * 2.0) / (hv * hv);
                        cu = cu.max(suu.dot(nn).abs());
                        cv = cv.max(svv.dot(nn).abs());
                    }
                }
                let st = |c: f64, span: f64| if c > 1e-12 { (8.0 * defl / c).sqrt().min(span) } else { span };
                let (a, b) = (st(cu, u1 - u0), st(cv, v1 - v0));
                if a >= (u1 - u0) && b >= (v1 - v0) {
                    (f64::INFINITY, f64::INFINITY)
                } else {
                    (a, b)
                }
            }
            _ => (f64::INFINITY, f64::INFINITY),
        }
    }
    /// Parameter (u roh als Winkel·Skala, ohne Abwicklung) eines Punktes nahe der Fläche
    fn param(&self, p: V3, hint: Option<(f64, f64)>, cache: &mut Option<Vec<(f64, f64, V3)>>) -> (f64, f64) {
        match self {
            CadSurf::Plane(f) => {
                let l = f.local(p);
                (l.x, l.y)
            }
            CadSurf::Cyl(f, _) => {
                let l = f.local(p);
                (l.y.atan2(l.x) * self.uscale(), l.z)
            }
            CadSurf::Cone(f, _, _) => {
                let l = f.local(p);
                let a = if l.x.hypot(l.y) < 1e-9 { hint.map(|h| h.0 / self.uscale()).unwrap_or(0.0) } else { l.y.atan2(l.x) };
                (a * self.uscale(), l.z)
            }
            CadSurf::Sphere(f, _) => {
                let l = f.local(p);
                let rxy = l.x.hypot(l.y);
                let a = if rxy < 1e-9 * (1.0 + l.z.abs()) { hint.map(|h| h.0 / self.uscale()).unwrap_or(0.0) } else { l.y.atan2(l.x) };
                (a * self.uscale(), l.z.atan2(rxy) * self.vscale())
            }
            CadSurf::Torus(f, rmaj, _) => {
                let l = f.local(p);
                let a = l.y.atan2(l.x);
                let rxy = l.x.hypot(l.y) - rmaj;
                (a * self.uscale(), l.z.atan2(rxy) * self.vscale())
            }
            CadSurf::Nurbs(s) => nurbs_project(s, p, hint, cache),
            CadSurf::Other(_) => (0.0, 0.0),
        }
    }
    /// Parameter der Pole/Spitze (für Flächen, die von nur einer umlaufenden Kante begrenzt werden)
    fn pole_v(&self, north: bool) -> Option<f64> {
        match self {
            CadSurf::Cone(_, r, ang) => {
                let t = ang.tan();
                if t.abs() < 1e-12 {
                    None
                } else {
                    Some(-r / t)
                }
            }
            CadSurf::Sphere(..) => Some(if north { PI / 2.0 } else { -PI / 2.0 } * self.vscale()),
            _ => None,
        }
    }
}

fn nurbs_project(s: &NurbsSurf, p: V3, hint: Option<(f64, f64)>, cache: &mut Option<Vec<(f64, f64, V3)>>) -> (f64, f64) {
    let ((u0, u1), (v0, v1)) = s.rng();
    if cache.is_none() {
        let n = 24;
        let mut g = Vec::with_capacity((n + 1) * (n + 1));
        for i in 0..=n {
            for j in 0..=n {
                let u = u0 + (u1 - u0) * i as f64 / n as f64;
                let v = v0 + (v1 - v0) * j as f64 / n as f64;
                g.push((u, v, s.eval(u, v)));
            }
        }
        *cache = Some(g);
    }
    let g = cache.as_ref().unwrap();
    let mut best = g.iter().min_by(|a, b| a.2.dist(p).partial_cmp(&b.2.dist(p)).unwrap_or(std::cmp::Ordering::Equal)).map(|x| (x.0, x.1)).unwrap_or((u0, v0));
    if let Some(h) = hint {
        if s.eval(h.0, h.1).dist(p) < s.eval(best.0, best.1).dist(p) {
            best = h;
        }
    }
    let (mut u, mut v) = best;
    let hu = (u1 - u0) * 1e-6;
    let hv = (v1 - v0) * 1e-6;
    for _ in 0..30 {
        let q = s.eval(u, v);
        let su = (s.eval((u + hu).min(u1), v) - s.eval((u - hu).max(u0), v)) / (((u + hu).min(u1) - (u - hu).max(u0)).max(1e-300));
        let sv = (s.eval(u, (v + hv).min(v1)) - s.eval(u, (v - hv).max(v0))) / (((v + hv).min(v1) - (v - hv).max(v0)).max(1e-300));
        let r = p - q;
        let (a, b, c) = (su.dot(su), su.dot(sv), sv.dot(sv));
        let (g1, g2) = (r.dot(su), r.dot(sv));
        let det = a * c - b * b;
        if det.abs() < 1e-300 {
            break;
        }
        let du = (c * g1 - b * g2) / det;
        let dv = (a * g2 - b * g1) / det;
        let nu = (u + du).clamp(u0, u1);
        let nv = (v + dv).clamp(v0, v1);
        let moved = (nu - u).abs() / (u1 - u0).max(1e-300) + (nv - v).abs() / (v1 - v0).max(1e-300);
        u = nu;
        v = nv;
        if moved < 1e-10 {
            break;
        }
    }
    (u, v)
}

/// Knoten für B-Splines ohne ausgeschriebene Knoten (QUASI_UNIFORM, UNIFORM, BEZIER).
fn implicit_knots(kind: &str, n: usize, deg: usize) -> Vec<f64> {
    match kind {
        "UNIFORM" => (0..n + deg + 1).map(|i| i as f64 - deg as f64).collect(),
        "BEZIER" => {
            let segs = ((n.saturating_sub(1)) / deg.max(1)).max(1);
            let mut k = vec![0.0; deg + 1];
            for s in 1..segs {
                for _ in 0..deg {
                    k.push(s as f64);
                }
            }
            k.extend(std::iter::repeat(segs as f64).take(deg + 1));
            k
        }
        _ => {
            // QUASI_UNIFORM: Enden geklemmt, innen gleichmäßig
            let inner = n.saturating_sub(deg + 1);
            let mut k = vec![0.0; deg + 1];
            for i in 1..=inner {
                k.push(i as f64);
            }
            k.extend(std::iter::repeat((inner + 1) as f64).take(deg + 1));
            k
        }
    }
}

/// Welche Art von Knotenangabe ein B-Spline-Eintrag hat (Kurve oder Fläche).
fn knot_kind(e: &Ent, what: &str) -> Option<&'static str> {
    for (k, n) in [("WITH_KNOTS", "B_SPLINE_X_WITH_KNOTS"), ("QUASI_UNIFORM", "QUASI_UNIFORM_X"), ("UNIFORM", "UNIFORM_X"), ("BEZIER", "BEZIER_X")] {
        if e.has(&n.replace('X', what)) {
            return Some(k);
        }
    }
    None
}

// ---------------------------------------------------------------------------
// Ergebnis

/// Eine gelesene CAD-Fläche mit ihren Dreiecken.
#[derive(Clone, Debug)]
pub struct CadFace {
    pub surf: CadSurf,
    /// true, wenn die Außennormale der natürlichen Flächennormale entspricht
    pub same_sense: bool,
    /// Dreiecksbereich [first, last] in `CadModel::tris`
    pub first: usize,
    pub last: usize,
}

#[derive(Clone, Debug, Default)]
pub struct CadModel {
    pub verts: Vec<V3>,
    pub tris: Vec<[u32; 3]>,
    pub faces: Vec<CadFace>,
    /// Längeneinheit der Datei (Anzeige), Umrechnung nach mm ist bereits erfolgt
    pub unit: String,
    pub warnings: Vec<String>,
    /// Name des Produkts/Teils, falls angegeben
    pub name: String,
}

pub fn read_file(path: &std::path::Path, defl: f64) -> Result<CadModel, String> {
    let data = std::fs::read(path).map_err(|e| format!("Datei kann nicht gelesen werden: {e}"))?;
    read_bytes(&data, defl)
}

struct Ctx<'a> {
    e: &'a HashMap<u32, Ent>,
    scale: f64,
    ang: f64,
    defl: f64,
}

impl<'a> Ctx<'a> {
    fn get(&self, id: u32) -> Result<&'a Ent, String> {
        self.e.get(&id).ok_or_else(|| format!("Verweis #{id} fehlt in der STEP-Datei"))
    }
    fn point(&self, id: u32) -> Result<V3, String> {
        let e = self.get(id)?;
        let a = e.part("CARTESIAN_POINT").ok_or_else(|| format!("#{id} ist kein Punkt"))?;
        let c = a.get(1).map(|v| v.list()).unwrap_or(&[]);
        let g = |i: usize| c.get(i).and_then(|x| x.f()).unwrap_or(0.0) * self.scale;
        Ok(v3(g(0), g(1), g(2)))
    }
    fn dir(&self, id: u32) -> Result<V3, String> {
        let e = self.get(id)?;
        let a = e.part("DIRECTION").ok_or_else(|| format!("#{id} ist keine Richtung"))?;
        let c = a.get(1).map(|v| v.list()).unwrap_or(&[]);
        let g = |i: usize| c.get(i).and_then(|x| x.f()).unwrap_or(0.0);
        let d = v3(g(0), g(1), g(2));
        Ok(d.normalized())
    }
    fn vertex(&self, id: u32) -> Result<V3, String> {
        let e = self.get(id)?;
        let a = e.part("VERTEX_POINT").ok_or_else(|| format!("#{id} ist keine Ecke"))?;
        self.point(a.get(1).and_then(|v| v.r()).ok_or("Ecke ohne Punkt")?)
    }
    fn frame(&self, id: u32) -> Result<Frame, String> {
        let e = self.get(id)?;
        let a = e.part("AXIS2_PLACEMENT_3D").ok_or_else(|| format!("#{id} ist kein Achsensystem"))?;
        let o = self.point(a.get(1).and_then(|v| v.r()).ok_or("Achsensystem ohne Ursprung")?)?;
        let z = match a.get(2).and_then(|v| v.r()) {
            Some(r) => self.dir(r)?,
            None => V3::Z,
        };
        let xr = match a.get(3).and_then(|v| v.r()) {
            Some(r) => self.dir(r)?,
            None => z.any_perp(),
        };
        let mut x = xr - z * xr.dot(z);
        if x.norm() < 1e-9 {
            x = z.any_perp();
        }
        let x = x.normalized();
        let y = z.cross(x);
        Ok(Frame { o, x, y, z })
    }
    fn knots(mults: &[Val], ks: &[Val]) -> Vec<f64> {
        let mut out = Vec::new();
        for (m, k) in mults.iter().zip(ks.iter()) {
            let n = m.f().unwrap_or(1.0).max(0.0) as usize;
            let kv = k.f().unwrap_or(0.0);
            for _ in 0..n {
                out.push(kv);
            }
        }
        out
    }
    fn nurbs_curve(&self, e: &Ent) -> Result<Nurbs, String> {
        let kind = knot_kind(e, "CURVE").ok_or("Unbekannte Kurvenart")?;
        let simple = e.parts.len() == 1;
        // Grad und Kontrollpunkte: im einfachen Eintrag nach dem Namen, im komplexen in B_SPLINE_CURVE
        let (base, off): (&[Val], usize) = if simple { (e.args(), 1) } else { (e.part("B_SPLINE_CURVE").ok_or("B-Spline-Kurve unvollständig")?, 0) };
        let deg = base.get(off).and_then(|v| v.f()).unwrap_or(1.0) as usize;
        let cps = base.get(off + 1).map(|v| v.list().to_vec()).unwrap_or_default();
        let pts: Vec<V3> = cps.iter().filter_map(|c| c.r()).map(|r| self.point(r)).collect::<Result<_, _>>()?;
        let knots = if kind == "WITH_KNOTS" {
            let k = e.part("B_SPLINE_CURVE_WITH_KNOTS").unwrap();
            let (m, kv) = if simple { (k.get(6), k.get(7)) } else { (k.first(), k.get(1)) };
            Self::knots(m.map(|v| v.list()).unwrap_or(&[]), kv.map(|v| v.list()).unwrap_or(&[]))
        } else {
            implicit_knots(kind, pts.len(), deg)
        };
        let weights = e.part("RATIONAL_B_SPLINE_CURVE").and_then(|r| r.first()).map(|l| l.list().iter().map(|x| x.f().unwrap_or(1.0)).collect::<Vec<f64>>());
        if pts.len() < 2 || deg == 0 || deg > 15 || knots.len() != pts.len() + deg + 1 {
            return Err("B-Spline-Kurve mit unpassenden Knoten".into());
        }
        let cp = pts
            .iter()
            .enumerate()
            .map(|(i, p)| {
                let wi = weights.as_ref().and_then(|w| w.get(i)).copied().unwrap_or(1.0);
                [p.x * wi, p.y * wi, p.z * wi, wi]
            })
            .collect();
        Ok(Nurbs { deg, knots, cp })
    }
    fn nurbs_surf(&self, e: &Ent) -> Result<NurbsSurf, String> {
        let kind = knot_kind(e, "SURFACE").ok_or("Unbekannte Flächenart")?;
        let simple = e.parts.len() == 1;
        let (base, off): (&[Val], usize) = if simple { (e.args(), 1) } else { (e.part("B_SPLINE_SURFACE").ok_or("B-Spline-Fläche unvollständig")?, 0) };
        let du = base.get(off).and_then(|v| v.f()).unwrap_or(1.0) as usize;
        let dv = base.get(off + 1).and_then(|v| v.f()).unwrap_or(1.0) as usize;
        let grid = base.get(off + 2).map(|v| v.list().to_vec()).unwrap_or_default();
        let w = e.part("RATIONAL_B_SPLINE_SURFACE").and_then(|r| r.first()).map(|l| l.list().iter().map(|row| row.list().iter().map(|x| x.f().unwrap_or(1.0)).collect::<Vec<f64>>()).collect::<Vec<_>>());
        let mut cp = Vec::new();
        for (i, row) in grid.iter().enumerate() {
            let mut r = Vec::new();
            for (j, c) in row.list().iter().enumerate() {
                let p = self.point(c.r().ok_or("Kontrollpunkt fehlt")?)?;
                let wi = w.as_ref().and_then(|w| w.get(i)).and_then(|r| r.get(j)).copied().unwrap_or(1.0);
                r.push([p.x * wi, p.y * wi, p.z * wi, wi]);
            }
            cp.push(r);
        }
        if cp.is_empty() || cp[0].is_empty() || cp.iter().any(|r| r.len() != cp[0].len()) || du == 0 || dv == 0 || du > 15 || dv > 15 {
            return Err("B-Spline-Fläche unvollständig".into());
        }
        let (ku, kv) = if kind == "WITH_KNOTS" {
            let k = e.part("B_SPLINE_SURFACE_WITH_KNOTS").unwrap();
            let (um, vm, uk, vk) = if simple { (k.get(8), k.get(9), k.get(10), k.get(11)) } else { (k.first(), k.get(1), k.get(2), k.get(3)) };
            let l = |v: Option<&Val>| v.map(|x| x.list().to_vec()).unwrap_or_default();
            (Self::knots(&l(um), &l(uk)), Self::knots(&l(vm), &l(vk)))
        } else {
            (implicit_knots(kind, cp.len(), du), implicit_knots(kind, cp[0].len(), dv))
        };
        if ku.len() != cp.len() + du + 1 || kv.len() != cp[0].len() + dv + 1 {
            return Err("B-Spline-Fläche mit unpassenden Knoten".into());
        }
        Ok(NurbsSurf { du, dv, ku, kv, cp })
    }

    fn surface(&self, id: u32) -> Result<CadSurf, String> {
        let e = self.get(id)?;
        let a = e.args();
        let fr = |i: usize| -> Result<Frame, String> { self.frame(a.get(i).and_then(|v| v.r()).ok_or("Fläche ohne Achsensystem")?) };
        let num = |i: usize| a.get(i).and_then(|v| v.f()).unwrap_or(0.0);
        Ok(match e.name() {
            "PLANE" => CadSurf::Plane(fr(1)?),
            "CYLINDRICAL_SURFACE" => CadSurf::Cyl(fr(1)?, num(2) * self.scale),
            "CONICAL_SURFACE" => {
                let raw = num(3);
                let mut ang = raw * self.ang;
                // manche Programme schreiben Grad ohne Einheitenangabe
                if ang.abs() >= PI / 2.0 {
                    ang = raw.to_radians();
                }
                CadSurf::Cone(fr(1)?, num(2) * self.scale, ang)
            }
            "SPHERICAL_SURFACE" => CadSurf::Sphere(fr(1)?, num(2) * self.scale),
            "TOROIDAL_SURFACE" | "DEGENERATE_TOROIDAL_SURFACE" => CadSurf::Torus(fr(1)?, num(2) * self.scale, num(3) * self.scale),
            _ if knot_kind(e, "SURFACE").is_some() => CadSurf::Nurbs(Box::new(self.nurbs_surf(e)?)),
            n => CadSurf::Other(if n.is_empty() { "komplex".into() } else { n.to_string() }),
        })
    }

    /// Randkurve einer Kante als Punktfolge von v0 nach v1 (in Richtung der Kante).
    fn edge_points(&self, id: u32) -> Result<Vec<V3>, String> {
        let e = self.get(id)?;
        let a = e.part("EDGE_CURVE").ok_or_else(|| format!("#{id} ist keine Kante"))?;
        let p0 = self.vertex(a.get(1).and_then(|v| v.r()).ok_or("Kante ohne Startecke")?)?;
        let p1 = self.vertex(a.get(2).and_then(|v| v.r()).ok_or("Kante ohne Endecke")?)?;
        let cid = a.get(3).and_then(|v| v.r()).ok_or("Kante ohne Kurve")?;
        let same = a.get(4).map(|v| v.b()).unwrap_or(true);
        let mut pts = self.curve_points(cid, p0, p1, same, 0)?;
        if pts.len() < 2 {
            pts = vec![p0, p1];
        }
        // Endpunkte exakt auf die Ecken setzen
        let n = pts.len();
        pts[0] = p0;
        pts[n - 1] = p1;
        Ok(pts)
    }

    fn curve_points(&self, cid: u32, p0: V3, p1: V3, same: bool, depth: u32) -> Result<Vec<V3>, String> {
        if depth > 6 {
            return Ok(vec![p0, p1]);
        }
        let ce = self.get(cid)?;
        let closed = p0.dist(p1) < 1e-7 * (1.0 + p0.norm());
        let name = ce.name();
        match name {
            "LINE" => Ok(vec![p0, p1]),
            "SURFACE_CURVE" | "SEAM_CURVE" | "INTERSECTION_CURVE" => {
                let inner = ce.args().get(1).and_then(|v| v.r()).ok_or("Flächenkurve ohne 3D-Kurve")?;
                self.curve_points(inner, p0, p1, same, depth + 1)
            }
            "TRIMMED_CURVE" => {
                let inner = ce.args().get(1).and_then(|v| v.r()).ok_or("beschnittene Kurve ohne Basis")?;
                self.curve_points(inner, p0, p1, same, depth + 1)
            }
            "CIRCLE" | "ELLIPSE" => {
                let f = self.frame(ce.args().get(1).and_then(|v| v.r()).ok_or("Kreis ohne Achsensystem")?)?;
                let ra = ce.args().get(2).and_then(|v| v.f()).unwrap_or(0.0) * self.scale;
                let rb = if name == "ELLIPSE" { ce.args().get(3).and_then(|v| v.f()).unwrap_or(0.0) * self.scale } else { ra };
                let ang = |p: V3| {
                    let l = f.local(p);
                    (l.y / rb.max(1e-300)).atan2(l.x / ra.max(1e-300))
                };
                let t0 = ang(p0);
                let mut t1 = ang(p1);
                if same {
                    while t1 <= t0 + 1e-12 {
                        t1 += TAU;
                    }
                    if !closed && t1 - t0 > TAU - 1e-9 {
                        t1 -= TAU;
                    }
                } else {
                    while t1 >= t0 - 1e-12 {
                        t1 -= TAU;
                    }
                    if !closed && t0 - t1 > TAU - 1e-9 {
                        t1 += TAU;
                    }
                }
                let rmax = ra.max(rb).max(1e-9);
                let step = 2.0 * (1.0 - (self.defl / rmax).min(1.0)).acos();
                let step = if step.is_finite() && step > 1e-4 { step.min(PI / 8.0) } else { PI / 8.0 };
                let n = (((t1 - t0).abs() / step).ceil() as usize).clamp(2, 20000);
                Ok((0..=n).map(|i| {
                    let t = t0 + (t1 - t0) * i as f64 / n as f64;
                    f.world(v3(ra * t.cos(), rb * t.sin(), 0.0))
                }).collect())
            }
            "POLYLINE" => {
                let pts: Vec<V3> = ce.args().get(1).map(|v| v.list().to_vec()).unwrap_or_default().iter().filter_map(|x| x.r()).map(|r| self.point(r)).collect::<Result<_, _>>()?;
                if pts.len() < 2 {
                    return Ok(vec![p0, p1]);
                }
                let mut v = pts;
                if v.last().unwrap().dist(p0) < v[0].dist(p0) {
                    v.reverse();
                }
                Ok(v)
            }
            _ if knot_kind(ce, "CURVE").is_some() => {
                let c = self.nurbs_curve(ce)?;
                let (a, b) = c.range();
                // Parameter der Ecken durch Projektion
                let proj = |p: V3| -> f64 {
                    let n = 200;
                    let mut bt = a;
                    let mut bd = f64::MAX;
                    for i in 0..=n {
                        let t = a + (b - a) * i as f64 / n as f64;
                        let d = c.eval(t).dist(p);
                        if d < bd {
                            bd = d;
                            bt = t;
                        }
                    }
                    // Feinsuche (Goldener Schnitt)
                    let h = (b - a) / n as f64;
                    let (mut lo, mut hi) = ((bt - h).max(a), (bt + h).min(b));
                    for _ in 0..60 {
                        let m1 = lo + (hi - lo) * 0.382;
                        let m2 = lo + (hi - lo) * 0.618;
                        if c.eval(m1).dist(p) < c.eval(m2).dist(p) {
                            hi = m2;
                        } else {
                            lo = m1;
                        }
                    }
                    (lo + hi) / 2.0
                };
                let (ta, tb) = if closed {
                    if same { (a, b) } else { (b, a) }
                } else {
                    let (x, y) = (proj(p0), proj(p1));
                    (x, y)
                };
                // adaptiv abtasten: Abschnitte halbieren, bis die Sehne passt
                let mut out = vec![c.eval(ta)];
                let mut stack: Vec<(f64, V3)> = Vec::new();
                let mut cur = (ta, c.eval(ta));
                let segs0 = (c.cp.len() * 4).clamp(8, 400);
                // Grundunterteilung
                let mut base: Vec<(f64, V3)> = (0..=segs0).map(|i| {
                    let t = ta + (tb - ta) * i as f64 / segs0 as f64;
                    (t, c.eval(t))
                }).collect();
                base.reverse();
                base.pop();
                stack.extend(base);
                let mut guard = 0;
                while let Some(nx) = stack.pop() {
                    guard += 1;
                    let tm = (cur.0 + nx.0) / 2.0;
                    let pm = c.eval(tm);
                    let chord = (cur.1 + nx.1) * 0.5;
                    if pm.dist(chord) > self.defl && (nx.0 - cur.0).abs() > 1e-12 && guard < 200_000 {
                        stack.push(nx);
                        stack.push((tm, pm));
                    } else {
                        out.push(nx.1);
                        cur = nx;
                    }
                }
                Ok(out)
            }
            _ => Ok(vec![p0, p1]),
        }
    }

    /// Schleife als Punktfolge (geschlossen, ohne doppelten Endpunkt). None = Eckenschleife.
    fn loop_points(&self, id: u32, edge_cache: &mut HashMap<u32, Vec<V3>>) -> Result<Option<Vec<V3>>, String> {
        let e = self.get(id)?;
        match e.name() {
            "VERTEX_LOOP" => Ok(None),
            "POLY_LOOP" => {
                let pts: Vec<V3> = e.args().get(1).map(|v| v.list().to_vec()).unwrap_or_default().iter().filter_map(|x| x.r()).map(|r| self.point(r)).collect::<Result<_, _>>()?;
                Ok(Some(pts))
            }
            "EDGE_LOOP" => {
                let mut out: Vec<V3> = Vec::new();
                for oe in e.args().get(1).map(|v| v.list().to_vec()).unwrap_or_default() {
                    let oid = oe.r().ok_or("Schleife ohne Kante")?;
                    let o = self.get(oid)?;
                    let oa = o.part("ORIENTED_EDGE").ok_or("Schleifenglied ist keine orientierte Kante")?;
                    let eid = oa.get(3).and_then(|v| v.r()).ok_or("orientierte Kante ohne Kante")?;
                    let fwd = oa.get(4).map(|v| v.b()).unwrap_or(true);
                    if !edge_cache.contains_key(&eid) {
                        let pts = self.edge_points(eid)?;
                        edge_cache.insert(eid, pts);
                    }
                    let pts = &edge_cache[&eid];
                    let seq: Vec<V3> = if fwd { pts.clone() } else { pts.iter().rev().copied().collect() };
                    for (k, p) in seq.into_iter().enumerate() {
                        if k == 0 {
                            if let Some(l) = out.last() {
                                if l.dist(p) < 1e-9 {
                                    continue;
                                }
                            }
                        }
                        out.push(p);
                    }
                }
                if out.len() > 1 && out[0].dist(*out.last().unwrap()) < 1e-7 {
                    out.pop();
                }
                Ok(Some(out))
            }
            n => Err(format!("Unbekannte Schleifenart {n}")),
        }
    }
}

/// Längeneinheit (Faktor nach mm, Name) und Winkeleinheit (Faktor nach Bogenmaß).
fn units(e: &HashMap<u32, Ent>) -> (f64, String, f64) {
    let si_len = |parts: &[Val]| -> f64 {
        let prefix = parts.first().map(|v| v.e().to_string()).unwrap_or_default();
        let base = match parts.get(1).map(|v| v.e()) {
            Some("METRE") => 1000.0,
            _ => 1000.0,
        };
        base * match prefix.as_str() {
            "MILLI" => 1e-3,
            "CENTI" => 1e-2,
            "DECI" => 1e-1,
            "MICRO" => 1e-6,
            "NANO" => 1e-9,
            "KILO" => 1e3,
            _ => 1.0,
        }
    };
    let mut scale = None;
    let mut name = String::from("mm");
    let mut ang = 1.0;
    // Kontext suchen, der die Einheiten festlegt
    let mut assigned: Vec<u32> = Vec::new();
    for en in e.values() {
        if let Some(a) = en.part("GLOBAL_UNIT_ASSIGNED_CONTEXT") {
            for u in a.first().map(|v| v.list()).unwrap_or(&[]) {
                if let Some(r) = u.r() {
                    assigned.push(r);
                }
            }
        }
    }
    let ids: Vec<u32> = if assigned.is_empty() { e.keys().copied().collect() } else { assigned };
    for id in ids {
        let Some(en) = e.get(&id) else { continue };
        if en.has("LENGTH_UNIT") && scale.is_none() {
            if let Some(si) = en.part("SI_UNIT") {
                let s = si_len(si);
                scale = Some(s);
                name = match si.first().map(|v| v.e()) {
                    Some("MILLI") => "mm".into(),
                    Some("CENTI") => "cm".into(),
                    Some("MICRO") => "µm".into(),
                    _ => "m".into(),
                };
            } else if let Some(cb) = en.part("CONVERSION_BASED_UNIT") {
                let nm = match cb.first() {
                    Some(Val::Str(s)) => s.to_ascii_lowercase(),
                    _ => String::new(),
                };
                let mut f = if nm.contains("inch") { 25.4 } else if nm.contains("foot") || nm.contains("feet") { 304.8 } else { 1.0 };
                if let Some(m) = cb.get(1).and_then(|v| v.r()).and_then(|r| e.get(&r)) {
                    if let Some(mw) = m.part("LENGTH_MEASURE_WITH_UNIT").or_else(|| m.part("MEASURE_WITH_UNIT")) {
                        let v = mw.first().and_then(|x| x.f()).unwrap_or(1.0);
                        let base = mw.get(1).and_then(|x| x.r()).and_then(|r| e.get(&r)).and_then(|u| u.part("SI_UNIT")).map(si_len).unwrap_or(1.0);
                        if v > 0.0 {
                            f = v * base;
                        }
                    }
                }
                scale = Some(f);
                name = if nm.is_empty() { "Zoll".into() } else if nm.contains("inch") { "Zoll".into() } else { nm };
            }
        }
        if en.has("PLANE_ANGLE_UNIT") {
            if let Some(cb) = en.part("CONVERSION_BASED_UNIT") {
                let nm = match cb.first() {
                    Some(Val::Str(s)) => s.to_ascii_lowercase(),
                    _ => String::new(),
                };
                if nm.contains("deg") || nm.contains("grad") {
                    ang = PI / 180.0;
                }
            }
        }
    }
    (scale.unwrap_or(1.0), name, ang)
}

// ---------------------------------------------------------------------------
// Triangulieren einer Fläche

fn unwrap_angles(us: &mut [f64], period: f64) {
    for i in 1..us.len() {
        let mut d = us[i] - us[i - 1];
        while d > period / 2.0 {
            us[i] -= period;
            d -= period;
        }
        while d < -period / 2.0 {
            us[i] += period;
            d += period;
        }
    }
}

struct FaceMesh {
    uv: Vec<[f64; 2]>,
    xyz: Vec<V3>,
    tris: Vec<[usize; 3]>,
}

fn tessellate_face(s: &CadSurf, loops3: &[Vec<V3>], has_vertex_loop: bool, same_sense: bool, defl: f64) -> Option<FaceMesh> {
    let mut cache = None;
    let periodic = s.periodic_u();
    let period = TAU * s.uscale();
    // Parameter je Schleife
    let mut rings: Vec<Vec<[f64; 2]>> = Vec::new();
    for l in loops3 {
        if l.len() < 2 {
            continue;
        }
        let mut uv = Vec::with_capacity(l.len());
        let mut hint: Option<(f64, f64)> = None;
        for &p in l {
            let q = s.param(p, hint, &mut cache);
            uv.push(q);
            hint = Some(q);
        }
        // Polstellen/Spitzen: Winkel vom Nachbarn übernehmen
        let mut us: Vec<f64> = uv.iter().map(|x| x.0).collect();
        if periodic {
            unwrap_angles(&mut us, period);
        }
        // Torus: auch v ist periodisch
        let mut vs: Vec<f64> = uv.iter().map(|x| x.1).collect();
        if let CadSurf::Torus(..) = s {
            unwrap_angles(&mut vs, TAU * s.vscale());
        }
        rings.push(us.iter().zip(vs.iter()).map(|(a, b)| [*a, *b]).collect());
    }
    if rings.is_empty() {
        return None;
    }
    // Schleifen auf die Außennormale ausrichten: Fläche liegt links der Schleife (im uv-Raum)
    if !same_sense {
        for r in rings.iter_mut() {
            r.reverse();
        }
    }
    // umlaufende Schleifen (netto ±2π) gesondert behandeln
    let net = |r: &Vec<[f64; 2]>, k: usize, period: f64| -> f64 {
        let mut d = r[0][k] - r[r.len() - 1][k];
        while d > period / 2.0 {
            d -= period;
        }
        while d < -period / 2.0 {
            d += period;
        }
        ((r[r.len() - 1][k] - r[0][k] + d) / period).round()
    };
    // Torus: läuft eine Schleife nur in v um, u und v tauschen
    let vperiod = TAU * s.vscale();
    let swapped = matches!(s, CadSurf::Torus(..))
        && rings.iter().all(|r| r.len() < 2 || net(r, 0, period) == 0.0)
        && rings.iter().any(|r| r.len() >= 2 && net(r, 1, vperiod) != 0.0);
    if swapped {
        for r in rings.iter_mut() {
            for p in r.iter_mut() {
                p.swap(0, 1);
            }
            r.reverse();
        }
    }
    let period = if swapped { vperiod } else { period };
    let wraps: Vec<f64> = rings.iter().map(|r| {
        if !periodic || r.len() < 2 {
            return 0.0;
        }
        // Netto-Umlauf inklusive Schlusskante
        let mut d = r[0][0] - r[r.len() - 1][0];
        while d > period / 2.0 {
            d -= period;
        }
        while d < -period / 2.0 {
            d += period;
        }
        let total = r[r.len() - 1][0] - r[0][0] + d;
        (total / period).round()
    }).collect();
    let wrapping: Vec<usize> = (0..rings.len()).filter(|&i| wraps[i] != 0.0).collect();
    let mut polys: Vec<Vec<[f64; 2]>> = Vec::new();
    let mut closed: Vec<(usize, Vec<[f64; 2]>)> = (0..rings.len()).filter(|i| !wrapping.contains(i)).map(|i| (i, rings[i].clone())).collect();
    // Herkunft jedes Randpunkts (Index der Schleife, −1 = künstlich ergänzt)
    let key = |p: &[f64; 2]| (p[0].to_bits(), p[1].to_bits());
    let mut tag: HashMap<(u64, u64), i64> = HashMap::new();
    if wrapping.len() == 2 {
        // Band zwischen zwei umlaufenden Kanten (z. B. Zylindermantel ohne Naht)
        let mut a = rings[wrapping[0]].clone();
        let mut b = rings[wrapping[1]].clone();
        if swapped {
            // auch die andere Richtung ist periodisch: die Fläche liegt links von Kante a
            let up = TAU * s.uscale();
            let ya = a.iter().map(|p| p[1]).sum::<f64>() / a.len() as f64;
            let yb = b.iter().map(|p| p[1]).sum::<f64>() / b.len() as f64;
            let target = if wraps[wrapping[0]] > 0.0 { ya + up / 2.0 } else { ya - up / 2.0 };
            let k = ((target - yb) / up).round();
            for p in b.iter_mut() {
                p[1] += k * up;
            }
        }
        if wraps[wrapping[0]] < 0.0 {
            a.reverse();
        }
        if wraps[wrapping[1]] < 0.0 {
            b.reverse();
        }
        // a beginnt bei seinem kleinsten u, auf [u0, u0+period] bringen
        let rot = |r: &mut Vec<[f64; 2]>| {
            let (mi, _) = r.iter().enumerate().fold((0, f64::MAX), |acc, (i, p)| {
                let m = p[0].rem_euclid(period);
                if m < acc.1 { (i, m) } else { acc }
            });
            r.rotate_left(mi);
            let mut us: Vec<f64> = r.iter().map(|p| p[0]).collect();
            us[0] = us[0].rem_euclid(period);
            unwrap_angles(&mut us, period);
            for (p, u) in r.iter_mut().zip(us) {
                p[0] = u;
            }
        };
        rot(&mut a);
        rot(&mut b);
        let start_a = a[0];
        a.push([start_a[0] + period, start_a[1]]);
        let start_b = b[0];
        b.push([start_b[0] + period, start_b[1]]);
        for p in &a {
            tag.insert(key(p), wrapping[0] as i64);
        }
        for p in &b {
            tag.insert(key(p), wrapping[1] as i64);
        }
        b.reverse();
        let mut poly = a;
        poly.extend(b);
        polys.push(poly);
    } else if wrapping.len() == 1 {
        // eine umlaufende Kante: Kegelspitze oder Kugelpol schließt die Fläche
        let i = wrapping[0];
        let mut r = rings[i].clone();
        if swapped {
            return None;
        }
        let going_up = wraps[i] > 0.0;
        // Fläche liegt links: bei steigendem u oberhalb (größeres v)
        let pole = s.pole_v(going_up).or_else(|| {
            // Spitze ohne Lage: Seite anhand der Eckenschleife bzw. Mittelwert
            if has_vertex_loop {
                s.pole_v(true)
            } else {
                None
            }
        });
        let Some(pv) = pole else { return None };
        let first = r[0];
        let last_u = first[0] + wraps[i] * period;
        r.push([last_u, r[r.len() - 1][1]]);
        for p in &r {
            tag.insert(key(p), i as i64);
        }
        r.push([last_u, pv]);
        r.push([first[0], pv]);
        tag.insert(key(&[last_u, pv]), -1);
        tag.insert(key(&[first[0], pv]), -1);
        polys.push(r);
    } else if !wrapping.is_empty() {
        return None;
    }
    if polys.is_empty() {
        // äußere Schleife = größte Fläche
        if closed.is_empty() {
            return None;
        }
        let (oi, _) = closed.iter().enumerate().fold((0, -1.0), |acc, (i, r)| {
            let a = poly::signed_area(&r.1).abs();
            if a > acc.1 { (i, a) } else { acc }
        });
        let (li, outer) = closed.remove(oi);
        for p in &outer {
            tag.insert(key(p), li as i64);
        }
        polys.push(outer);
    }
    // Außenring gegen, Löcher im Uhrzeigersinn; Löcher periodisch in den Bereich schieben
    let mut outer = polys.remove(0);
    if poly::signed_area(&outer) < 0.0 {
        outer.reverse();
    }
    let (umin, umax) = outer.iter().fold((f64::MAX, f64::MIN), |a, p| (a.0.min(p[0]), a.1.max(p[0])));
    let mut holes = Vec::new();
    for (li, mut h) in closed {
        if periodic {
            let c = h.iter().map(|p| p[0]).sum::<f64>() / h.len() as f64;
            let mid = (umin + umax) / 2.0;
            let k = ((mid - c) / period).round();
            for p in h.iter_mut() {
                p[0] += k * period;
            }
        }
        if poly::signed_area(&h) > 0.0 {
            h.reverse();
        }
        for p in &h {
            tag.insert(key(p), li as i64);
        }
        holes.push(h);
    }
    let mut rings_t = vec![outer];
    rings_t.extend(holes);
    // doppelte Nachbarpunkte entfernen (stören das Ohrenschneiden)
    for r in rings_t.iter_mut() {
        let mut out: Vec<[f64; 2]> = Vec::with_capacity(r.len());
        for p in r.iter() {
            if let Some(l) = out.last() {
                if (l[0] - p[0]).abs() < 1e-12 && (l[1] - p[1]).abs() < 1e-12 {
                    continue;
                }
            }
            out.push(*p);
        }
        while out.len() > 1 && (out[0][0] - out[out.len() - 1][0]).abs() < 1e-12 && (out[0][1] - out[out.len() - 1][1]).abs() < 1e-12 {
            out.pop();
        }
        *r = out;
    }
    rings_t.retain(|r| r.len() >= 3);
    if rings_t.is_empty() {
        return None;
    }
    let tri = poly::triangulate(&rings_t);
    let mut uv: Vec<[f64; 2]> = rings_t.iter().flatten().copied().collect();
    if swapped {
        for p in uv.iter_mut() {
            p.swap(0, 1);
        }
    }
    if tri.is_empty() {
        return None;
    }
    // echte Randkanten: aufeinanderfolgende Punkte derselben Schleife (künstliche Verbindungen
    // zwischen Schleifen oder zu einem Pol dürfen geteilt werden)
    let mut border = std::collections::HashSet::new();
    {
        let mut base = 0;
        for r in &rings_t {
            let n = r.len();
            for i in 0..n {
                let (ta, tb) = (tag.get(&key(&r[i])).copied().unwrap_or(-1), tag.get(&key(&r[(i + 1) % n])).copied().unwrap_or(-1));
                if ta >= 0 && ta == tb {
                    let (a, b) = (base + i, base + (i + 1) % n);
                    border.insert(if a < b { (a, b) } else { (b, a) });
                }
            }
            base += n;
        }
    }
    let mut fm = FaceMesh { xyz: uv.iter().map(|p| s.point(p[0], p[1])).collect(), uv, tris: tri };
    if !matches!(s, CadSurf::Plane(_)) {
        // Zielkantenlängen im Parameterraum (doppelt gekrümmte Flächen brauchen Innenpunkte)
        let (su, sv) = s.target_steps(defl);
        let (fu, fv) = (if su.is_finite() { su } else { 1.0 }, if sv.is_finite() { sv } else { 1.0 });
        let scaled: Vec<[f64; 2]> = fm.uv.iter().map(|p| [p[0] / fu, p[1] / fv]).collect();
        delaunay_flip(&scaled, &mut fm.tris);
        if su.is_finite() && sv.is_finite() {
            insert_points(&mut fm, s, su, sv);
        }
    }
    refine(s, &mut fm, defl, &border);
    // Orientierung: Dreiecke sollen zur Außennormale zeigen
    let want = if same_sense { 1.0 } else { -1.0 };
    for t in fm.tris.iter_mut() {
        let (a, b, c) = (fm.xyz[t[0]], fm.xyz[t[1]], fm.xyz[t[2]]);
        let n = (b - a).cross(c - a);
        if n.norm() < 1e-300 {
            continue;
        }
        let m = [(fm.uv[t[0]][0] + fm.uv[t[1]][0] + fm.uv[t[2]][0]) / 3.0, (fm.uv[t[0]][1] + fm.uv[t[1]][1] + fm.uv[t[2]][1]) / 3.0];
        let sn = s.normal(m[0], m[1]) * want;
        if n.dot(sn) < 0.0 {
            t.swap(1, 2);
        }
    }
    Some(fm)
}

fn orient2(a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> f64 {
    (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
}

fn incircle(a: [f64; 2], b: [f64; 2], c: [f64; 2], d: [f64; 2]) -> f64 {
    let (ax, ay) = (a[0] - d[0], a[1] - d[1]);
    let (bx, by) = (b[0] - d[0], b[1] - d[1]);
    let (cx, cy) = (c[0] - d[0], c[1] - d[1]);
    (ax * ax + ay * ay) * (bx * cy - cx * by) - (bx * bx + by * by) * (ax * cy - cx * ay) + (cx * cx + cy * cy) * (ax * by - bx * ay)
}

const NO: usize = usize::MAX;

/// Ebene Dreieckszerlegung mit Nachbarschaft (Kante k = (t[k], t[k+1])), Randkanten ohne Nachbarn.
struct Tri2 {
    q: Vec<[f64; 2]>,
    tris: Vec<[usize; 3]>,
    nbr: Vec<[usize; 3]>,
    eps: f64,
}

impl Tri2 {
    fn new(q: Vec<[f64; 2]>, mut tris: Vec<[usize; 3]>) -> Tri2 {
        for t in tris.iter_mut() {
            if orient2(q[t[0]], q[t[1]], q[t[2]]) < 0.0 {
                t.swap(1, 2);
            }
        }
        let mut em: HashMap<(usize, usize), usize> = HashMap::with_capacity(tris.len() * 3);
        for (ti, t) in tris.iter().enumerate() {
            for k in 0..3 {
                em.insert((t[k], t[(k + 1) % 3]), ti);
            }
        }
        let mut nbr = vec![[NO; 3]; tris.len()];
        for (ti, t) in tris.iter().enumerate() {
            for k in 0..3 {
                if let Some(&tj) = em.get(&(t[(k + 1) % 3], t[k])) {
                    if tj != ti {
                        nbr[ti][k] = tj;
                    }
                }
            }
        }
        let (mut lo, mut hi) = ([f64::MAX; 2], [f64::MIN; 2]);
        for p in &q {
            for k in 0..2 {
                lo[k] = lo[k].min(p[k]);
                hi[k] = hi[k].max(p[k]);
            }
        }
        let sc = (hi[0] - lo[0]).max(hi[1] - lo[1]).max(1e-12);
        Tri2 { q, tris, nbr, eps: sc * sc * 1e-13 }
    }

    /// Kante im Nachbarn tj, die zu (x, y) von ti gehört
    fn edge_in(&self, tj: usize, x: usize, y: usize) -> Option<usize> {
        let u = self.tris[tj];
        (0..3).find(|&j| u[j] == y && u[(j + 1) % 3] == x)
    }

    fn relink(&mut self, o: usize, old: usize, new: usize) {
        if o == NO {
            return;
        }
        for k in 0..3 {
            if self.nbr[o][k] == old {
                let (x, y) = (self.tris[o][k], self.tris[o][(k + 1) % 3]);
                let tn = self.tris[new];
                if (0..3).any(|j| tn[j] == y && tn[(j + 1) % 3] == x) {
                    self.nbr[o][k] = new;
                }
            }
        }
    }

    /// Kante k von ti tauschen, falls sie die Delaunay-Bedingung verletzt. Rückgabe: getauscht.
    fn flip_if_illegal(&mut self, ti: usize, k: usize) -> bool {
        let tj = self.nbr[ti][k];
        if tj == NO {
            return false;
        }
        let tv = self.tris[ti];
        let (x, y, c) = (tv[k], tv[(k + 1) % 3], tv[(k + 2) % 3]);
        let Some(kj) = self.edge_in(tj, x, y) else { return false };
        let d = self.tris[tj][(kj + 2) % 3];
        if d == c {
            return false;
        }
        let q = &self.q;
        if incircle(q[x], q[y], q[c], q[d]) <= self.eps {
            return false;
        }
        if orient2(q[x], q[d], q[c]) <= self.eps || orient2(q[d], q[y], q[c]) <= self.eps {
            return false;
        }
        let n_yc = self.nbr[ti][(k + 1) % 3];
        let n_cx = self.nbr[ti][(k + 2) % 3];
        let n_xd = self.nbr[tj][(kj + 1) % 3];
        let n_dy = self.nbr[tj][(kj + 2) % 3];
        self.tris[ti] = [x, d, c];
        self.tris[tj] = [d, y, c];
        self.nbr[ti] = [n_xd, tj, n_cx];
        self.nbr[tj] = [n_dy, n_yc, ti];
        self.relink(n_xd, tj, ti);
        self.relink(n_yc, ti, tj);
        true
    }

    /// Alle Kanten bis zur Delaunay-Zerlegung tauschen (Lawson, mit Arbeitsliste).
    fn legalize_all(&mut self) {
        let mut stack: Vec<usize> = (0..self.tris.len()).rev().collect();
        let mut inq = vec![true; self.tris.len()];
        let mut guard = 0usize;
        let limit = self.tris.len() * 60 + 1000;
        while let Some(ti) = stack.pop() {
            inq[ti] = false;
            guard += 1;
            if guard > limit {
                break;
            }
            for k in 0..3 {
                let tj = self.nbr[ti][k];
                if self.flip_if_illegal(ti, k) {
                    for t in [ti, tj] {
                        if t != NO && !inq[t] {
                            inq[t] = true;
                            stack.push(t);
                        }
                    }
                    // Nachbarn der neuen Kanten ebenfalls prüfen
                    for t in [ti, tj] {
                        for m in 0..3 {
                            let o = self.nbr[t][m];
                            if o != NO && !inq[o] {
                                inq[o] = true;
                                stack.push(o);
                            }
                        }
                    }
                    break;
                }
            }
        }
    }

    /// Punkt einfügen (Dreieck suchen ab `start`). Rückgabe: Index des Dreiecks, None = außerhalb/zu nah.
    fn insert(&mut self, p: [f64; 2], start: usize, min_v: f64, min_b: f64) -> Option<usize> {
        if self.tris.is_empty() {
            return None;
        }
        let mut t = start.min(self.tris.len() - 1);
        let mut found = None;
        for _ in 0..(self.tris.len() + 16).min(40_000) {
            let tv = self.tris[t];
            let mut next = None;
            let mut out = false;
            for k in 0..3 {
                if orient2(self.q[tv[k]], self.q[tv[(k + 1) % 3]], p) < -self.eps {
                    if self.nbr[t][k] == NO {
                        out = true;
                    } else {
                        next = Some(self.nbr[t][k]);
                    }
                    break;
                }
            }
            if out {
                break;
            }
            match next {
                Some(n) => t = n,
                None => {
                    found = Some(t);
                    break;
                }
            }
        }
        let t = found?;
        let tv = self.tris[t];
        for k in 0..3 {
            let (a, b) = (self.q[tv[k]], self.q[tv[(k + 1) % 3]]);
            if (a[0] - p[0]).hypot(a[1] - p[1]) < min_v {
                return None;
            }
            let lim = if self.nbr[t][k] == NO { min_b } else { min_v * 0.25 };
            if poly::seg_dist(p, a, b) < lim {
                return None;
            }
        }
        let pi = self.q.len();
        self.q.push(p);
        let (a, b, c) = (tv[0], tv[1], tv[2]);
        let (na, nb, nc) = (self.nbr[t][0], self.nbr[t][1], self.nbr[t][2]);
        let t1 = self.tris.len();
        let t2 = t1 + 1;
        self.tris[t] = [a, b, pi];
        self.tris.push([b, c, pi]);
        self.tris.push([c, a, pi]);
        self.nbr[t] = [na, t1, t2];
        self.nbr.push([nb, t2, t]);
        self.nbr.push([nc, t, t1]);
        self.relink(nb, t, t1);
        self.relink(nc, t, t2);
        // Kanten gegenüber dem neuen Punkt legalisieren
        let mut stack = vec![t, t1, t2];
        let mut guard = 0;
        while let Some(ti) = stack.pop() {
            guard += 1;
            if guard > 20_000 {
                break;
            }
            let tv = self.tris[ti];
            let Some(kp) = (0..3).find(|&k| tv[k] == pi) else { continue };
            let k = (kp + 1) % 3;
            let tj = self.nbr[ti][k];
            if self.flip_if_illegal(ti, k) {
                stack.push(ti);
                stack.push(tj);
            }
        }
        Some(t)
    }
}

/// Kantentausch nach Delaunay im Parameterraum: vermeidet lange Fächerdreiecke
/// (z. B. quer über einen ganzen Zylinderumfang). Randkanten bleiben erhalten.
fn delaunay_flip(uv: &[[f64; 2]], tris: &mut Vec<[usize; 3]>) {
    let mut t2 = Tri2::new(uv.to_vec(), std::mem::take(tris));
    t2.legalize_all();
    *tris = t2.tris;
}

/// Innenpunkte auf einem Raster (Abstand su, sv im Parameterraum) einfügen
/// und dabei die Delaunay-Eigenschaft erhalten (Randkanten bleiben unverändert).
fn insert_points(fm: &mut FaceMesh, s: &CadSurf, su: f64, sv: f64) {
    let q: Vec<[f64; 2]> = fm.uv.iter().map(|p| [p[0] / su, p[1] / sv]).collect();
    let mut t2 = Tri2::new(q, std::mem::take(&mut fm.tris));
    let (mut lo, mut hi) = ([f64::MAX; 2], [f64::MIN; 2]);
    for p in &t2.q {
        for k in 0..2 {
            lo[k] = lo[k].min(p[k]);
            hi[k] = hi[k].max(p[k]);
        }
    }
    let rows = ((hi[1] - lo[1]) / 0.85).ceil() as usize;
    let cols = ((hi[0] - lo[0]) / 0.98).ceil() as usize;
    if rows == 0 || cols == 0 || rows * cols > 600_000 {
        fm.tris = t2.tris;
        return;
    }
    let mut cur = 0usize;
    for r in 0..=rows {
        let y = lo[1] + r as f64 * (hi[1] - lo[1]) / rows as f64;
        let off = if r % 2 == 1 { 0.49 } else { 0.0 };
        let xs: Vec<usize> = if r % 2 == 0 { (0..=cols).collect() } else { (0..=cols).rev().collect() };
        for c in xs {
            let x = lo[0] + (c as f64 + off) * (hi[0] - lo[0]) / cols as f64;
            if let Some(t) = t2.insert([x, y], cur, 0.35, 0.45) {
                cur = t;
                let uvp = [x * su, y * sv];
                fm.uv.push(uvp);
                fm.xyz.push(s.point(uvp[0], uvp[1]));
            }
        }
    }
    fm.tris = t2.tris;
}

/// Dreiecke teilen, bis Kantenmitten und Schwerpunkt höchstens `defl` von der Fläche abweichen.
/// Jedes Dreieck wird für sich an seiner am stärksten abweichenden Kante halbiert; kleine
/// T-Stöße zwischen Nachbarn sind für die Abstandsmessung ohne Bedeutung.
fn refine(s: &CadSurf, fm: &mut FaceMesh, defl: f64, border: &std::collections::HashSet<(usize, usize)>) {
    if matches!(s, CadSurf::Plane(_) | CadSurf::Other(_)) {
        return;
    }
    let max_tris = 300_000usize;
    let mut stack: Vec<([usize; 3], u32)> = fm.tris.drain(..).map(|t| (t, 0)).collect();
    let mut out: Vec<[usize; 3]> = Vec::with_capacity(stack.len() * 4);
    let mut mids: HashMap<(usize, usize), usize> = HashMap::new();
    while let Some((t, depth)) = stack.pop() {
        if depth >= 22 || out.len() + stack.len() > max_tris {
            out.push(t);
            continue;
        }
        let mut worst = (0usize, defl);
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            // Randkanten sind schon fein genug abgetastet; teilen würde Lücken zum Nachbarn erzeugen
            if border.contains(&if a < b { (a, b) } else { (b, a) }) {
                continue;
            }
            let mu = [(fm.uv[a][0] + fm.uv[b][0]) / 2.0, (fm.uv[a][1] + fm.uv[b][1]) / 2.0];
            let dv = s.point(mu[0], mu[1]).dist((fm.xyz[a] + fm.xyz[b]) * 0.5);
            if dv > worst.1 {
                worst = (k + 1, dv);
            }
        }
        if worst.0 == 0 {
            // Kanten passen: Wölbung in der Mitte prüfen, sonst fertig
            let c = [(fm.uv[t[0]][0] + fm.uv[t[1]][0] + fm.uv[t[2]][0]) / 3.0, (fm.uv[t[0]][1] + fm.uv[t[1]][1] + fm.uv[t[2]][1]) / 3.0];
            let g = (fm.xyz[t[0]] + fm.xyz[t[1]] + fm.xyz[t[2]]) / 3.0;
            if s.point(c[0], c[1]).dist(g) <= defl {
                out.push(t);
                continue;
            }
            // längste innere Kante (3D) teilen
            let mut lk = 3;
            let mut ll = -1.0;
            for k in 0..3 {
                let (a, b) = (t[k], t[(k + 1) % 3]);
                if border.contains(&if a < b { (a, b) } else { (b, a) }) {
                    continue;
                }
                let l = fm.xyz[a].dist(fm.xyz[b]);
                if l > ll {
                    ll = l;
                    lk = k;
                }
            }
            if lk == 3 {
                out.push(t);
                continue;
            }
            worst = (lk + 1, 0.0);
        }
        let k = worst.0 - 1;
        let (a, b, c) = (t[k], t[(k + 1) % 3], t[(k + 2) % 3]);
        let key = if a < b { (a, b) } else { (b, a) };
        let m = match mids.get(&key) {
            Some(&m) => m,
            None => {
                let mu = [(fm.uv[a][0] + fm.uv[b][0]) / 2.0, (fm.uv[a][1] + fm.uv[b][1]) / 2.0];
                fm.uv.push(mu);
                fm.xyz.push(s.point(mu[0], mu[1]));
                let id = fm.uv.len() - 1;
                mids.insert(key, id);
                id
            }
        };
        stack.push(([a, m, c], depth + 1));
        stack.push(([m, b, c], depth + 1));
    }
    fm.tris = out;
}

/// Fläche, deren Art nicht unterstützt wird: Randpolygon in seiner Ausgleichsebene triangulieren.
fn fallback_face(loops3: &[Vec<V3>]) -> Option<FaceMesh> {
    let all: Vec<V3> = loops3.iter().flatten().copied().collect();
    if all.len() < 3 {
        return None;
    }
    let pl = crate::surf::fit_plane(&all)?;
    let (o, n) = match pl {
        crate::surf::Surf::Plane { o, n } => (o, n),
        _ => return None,
    };
    let (ex, ey) = n.basis();
    let mut rings: Vec<Vec<[f64; 2]>> = loops3.iter().filter(|l| l.len() >= 3).map(|l| l.iter().map(|p| [(*p - o).dot(ex), (*p - o).dot(ey)]).collect()).collect();
    let mut rings3: Vec<Vec<V3>> = loops3.iter().filter(|l| l.len() >= 3).cloned().collect();
    if rings.is_empty() {
        return None;
    }
    let (oi, _) = rings.iter().enumerate().fold((0, -1.0), |acc, (i, r)| {
        let a = poly::signed_area(r).abs();
        if a > acc.1 { (i, a) } else { acc }
    });
    rings.swap(0, oi);
    rings3.swap(0, oi);
    for (i, r) in rings.iter_mut().enumerate() {
        let a = poly::signed_area(r);
        if (i == 0 && a < 0.0) || (i > 0 && a > 0.0) {
            r.reverse();
            rings3[i].reverse();
        }
    }
    let tris = poly::triangulate(&rings);
    Some(FaceMesh { uv: rings.iter().flatten().copied().collect(), xyz: rings3.iter().flatten().copied().collect(), tris })
}

pub fn read_bytes(data: &[u8], defl: f64) -> Result<CadModel, String> {
    let ents = parse(data)?;
    let (scale, unit, ang) = units(&ents);
    let cx = Ctx { e: &ents, scale, ang, defl: defl.max(1e-5) };
    let mut model = CadModel { unit, ..Default::default() };
    for en in ents.values() {
        if let Some(a) = en.part("PRODUCT") {
            if let Some(Val::Str(s)) = a.get(1).or_else(|| a.first()) {
                if !s.is_empty() && model.name.is_empty() {
                    model.name = s.clone();
                }
            }
        }
    }
    // alle Flächen in fester Reihenfolge
    let mut face_ids: Vec<u32> = ents.iter().filter(|(_, e)| e.name() == "ADVANCED_FACE" || e.name() == "FACE_SURFACE").map(|(i, _)| *i).collect();
    face_ids.sort_unstable();
    if face_ids.is_empty() {
        if ents.values().any(|e| e.name() == "MAPPED_ITEM" || e.name().contains("TESSELLATED")) {
            return Err("Die STEP-Datei enthält keine B-Rep-Flächen (nur Netz- oder Verweisdaten).".into());
        }
        if ents.values().any(|e| e.name() == "DOCUMENT_FILE" || e.name() == "NEXT_ASSEMBLY_USAGE_OCCURRENCE") {
            return Err("Die STEP-Datei enthält keine Flächen – vermutlich eine Baugruppe, die ihre Teile aus anderen Dateien lädt. Bitte die Datei des Einzelteils wählen.".into());
        }
        return Err("In der STEP-Datei wurden keine Flächen gefunden.".into());
    }
    if ents.values().any(|e| e.has("ITEM_DEFINED_TRANSFORMATION")) {
        model.warnings.push("Die Datei ist eine Baugruppe: Lageänderungen der Einzelteile werden nicht angewendet.".into());
    }
    let mut edge_cache: HashMap<u32, Vec<V3>> = HashMap::new();
    let mut failed = 0usize;
    let mut other: HashMap<String, usize> = HashMap::new();
    for fid in face_ids {
        let fe = &ents[&fid];
        let a = fe.args();
        let bounds = a.get(1).map(|v| v.list().to_vec()).unwrap_or_default();
        let sid = match a.get(2).and_then(|v| v.r()) {
            Some(s) => s,
            None => {
                failed += 1;
                continue;
            }
        };
        let same = a.get(3).map(|v| v.b()).unwrap_or(true);
        let surf = match cx.surface(sid) {
            Ok(s) => s,
            Err(_) => {
                failed += 1;
                continue;
            }
        };
        let mut loops = Vec::new();
        let mut vloop = false;
        let mut bad = false;
        for b in bounds {
            let Some(bid) = b.r() else { continue };
            let Ok(be) = cx.get(bid) else {
                bad = true;
                break;
            };
            let ba = be.args();
            let lid = ba.get(1).and_then(|v| v.r());
            let ori = ba.get(2).map(|v| v.b()).unwrap_or(true);
            let Some(lid) = lid else { continue };
            match cx.loop_points(lid, &mut edge_cache) {
                Ok(Some(mut pts)) => {
                    if !ori {
                        pts.reverse();
                    }
                    loops.push(pts);
                }
                Ok(None) => vloop = true,
                Err(_) => {
                    bad = true;
                    break;
                }
            }
        }
        if bad || loops.is_empty() {
            failed += 1;
            continue;
        }
        let fm = match &surf {
            CadSurf::Other(n) => {
                *other.entry(n.clone()).or_insert(0) += 1;
                fallback_face(&loops)
            }
            s => tessellate_face(s, &loops, vloop, same, cx.defl).or_else(|| fallback_face(&loops)),
        };
        let Some(fm) = fm else {
            failed += 1;
            continue;
        };
        if fm.tris.is_empty() {
            failed += 1;
            continue;
        }
        let base = model.verts.len() as u32;
        model.verts.extend(fm.xyz.iter().copied());
        let first = model.tris.len();
        for t in &fm.tris {
            model.tris.push([base + t[0] as u32, base + t[1] as u32, base + t[2] as u32]);
        }
        model.faces.push(CadFace { surf, same_sense: same, first, last: model.tris.len() - 1 });
    }
    if model.tris.is_empty() {
        return Err("Die Flächen der STEP-Datei konnten nicht vernetzt werden.".into());
    }
    if failed > 0 {
        model.warnings.push(format!("{failed} Fläche(n) konnten nicht gelesen werden und fehlen im Vergleich."));
    }
    if !other.is_empty() {
        let n: usize = other.values().sum();
        model.warnings.push(format!("{n} Fläche(n) einer nicht unterstützten Art wurden nur eben angenähert."));
    }
    Ok(model)
}
