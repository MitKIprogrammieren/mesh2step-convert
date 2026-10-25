//! Farbbemalung aus Bambu Studio / OrcaSlicer (`paint_color`) und PrusaSlicer
//! (`mmu_segmentation`) dekodieren und das Netz entlang der Farbgrenzen verfeinern.
//!
//! Format (TriangleSelector): Hex-String, Zeichen von hinten gelesen, je Zeichen
//! 4 Bit mit dem niederwertigsten Bit zuerst. Je Knoten ein Nibble `xxyy`:
//! `yy` = Anzahl geteilter Seiten (0 = Blatt). Blatt: `xx` = Farbe (0..2), bei
//! `xx = 3` folgen weitere Nibbles (Farbe - 3, mit 0b1111-Läufen für je +15).
//! Geteilt: `xx` = besondere Seite, danach die Kinder in umgekehrter Reihenfolge.

use crate::io::RawMesh;
use crate::math::{v3, V3};
use std::collections::HashMap;

/// Ein Netz, bei dem jedes Dreieck genau eine Filamentfarbe hat.
#[derive(Clone, Debug, Default)]
pub struct ColoredMesh {
    pub name: String,
    pub verts: Vec<V3>,
    pub tris: Vec<[u32; 3]>,
    /// Filament je Dreieck (1-basiert)
    pub col: Vec<u8>,
}

/// Bitstrom eines Dreiecks.
struct Bits {
    bits: Vec<bool>,
    pos: usize,
}

impl Bits {
    fn from_hex(s: &str) -> Option<Bits> {
        let mut bits = Vec::with_capacity(s.len() * 4);
        for ch in s.trim().bytes().rev() {
            let d = match ch {
                b'0'..=b'9' => ch - b'0',
                b'A'..=b'F' => ch - b'A' + 10,
                b'a'..=b'f' => ch - b'a' + 10,
                _ => return None,
            };
            for i in 0..4 {
                bits.push(d & (1 << i) != 0);
            }
        }
        Some(Bits { bits, pos: 0 })
    }
    fn nibble(&mut self) -> Option<u8> {
        if self.pos + 4 > self.bits.len() {
            return None;
        }
        let mut n = 0u8;
        for i in 0..4 {
            if self.bits[self.pos + i] {
                n |= 1 << i;
            }
        }
        self.pos += 4;
        Some(n)
    }
}

/// Exakte baryzentrische Koordinate (Summe = 2^D).
type Bary = [u64; 3];
const D: u32 = 52;
const S: u64 = 1 << D;
const MAX_DEPTH: usize = 48;

fn mid(a: &Bary, b: &Bary) -> Option<Bary> {
    let mut m = [0u64; 3];
    for k in 0..3 {
        let s = a[k] + b[k];
        if s % 2 != 0 {
            return None;
        }
        m[k] = s / 2;
    }
    Some(m)
}

/// Blätter eines bemalten Dreiecks: (Ecken in Baryzentrik, Farbe 0 = Standard)
fn decode(code: &str, out: &mut Vec<([Bary; 3], u8)>) -> bool {
    let root = [[S, 0, 0], [0, S, 0], [0, 0, S]];
    let mut bits = match Bits::from_hex(code) {
        Some(b) => b,
        None => return false,
    };
    let start = out.len();
    if !decode_node(&mut bits, root, 0, out) {
        out.truncate(start);
        return false;
    }
    true
}

fn decode_node(bits: &mut Bits, v: [Bary; 3], depth: usize, out: &mut Vec<([Bary; 3], u8)>) -> bool {
    let code = match bits.nibble() {
        Some(c) => c,
        None => return false,
    };
    let split = (code & 0b11) as usize;
    if split == 0 {
        let mut state = (code >> 2) as u32;
        if code & 0b1100 == 0b1100 {
            let mut n = 0u32;
            let mut next = match bits.nibble() {
                Some(c) => c,
                None => return false,
            };
            while next == 0b1111 {
                n += 1;
                next = match bits.nibble() {
                    Some(c) => c,
                    None => return false,
                };
            }
            state = next as u32 + 15 * n + 3;
        }
        out.push((v, state.min(255) as u8));
        return true;
    }
    if depth >= MAX_DEPTH {
        return false;
    }
    let special = (code >> 2) as usize % 3;
    // Ecken ab der besonderen Seite drehen
    let r = [v[special], v[(special + 1) % 3], v[(special + 2) % 3]];
    let children: Vec<[Bary; 3]> = match split {
        1 => {
            let m12 = match mid(&r[1], &r[2]) {
                Some(m) => m,
                None => return false,
            };
            vec![[r[0], r[1], m12], [m12, r[2], r[0]]]
        }
        2 => {
            let (m01, m20) = match (mid(&r[0], &r[1]), mid(&r[2], &r[0])) {
                (Some(a), Some(b)) => (a, b),
                _ => return false,
            };
            vec![[r[0], m01, m20], [m01, r[1], m20], [r[1], r[2], m20]]
        }
        _ => {
            // drei Seiten: besondere Seite ist immer 0
            let r = v;
            let (m01, m12, m20) = match (mid(&r[0], &r[1]), mid(&r[1], &r[2]), mid(&r[2], &r[0])) {
                (Some(a), Some(b), Some(c)) => (a, b, c),
                _ => return false,
            };
            vec![[r[0], m01, m20], [m01, r[1], m12], [m12, r[2], m20], [m01, m12, m20]]
        }
    };
    // Kinder stehen in umgekehrter Reihenfolge im Strom
    for c in children.iter().rev() {
        if !decode_node(bits, *c, depth + 1, out) {
            return false;
        }
    }
    true
}

/// Kodierung eines ungeteilten Dreiecks mit Farbe `state` (für den 3MF-Export).
pub fn encode_leaf(state: u8) -> String {
    let hex = |n: u32| char::from_digit(n, 16).unwrap().to_ascii_uppercase();
    if state < 3 {
        return hex((state as u32) << 2).to_string();
    }
    // Nibbles in Stromreihenfolge: 0b1100, (0b1111)*, Rest; Zeichenkette umgekehrt
    let mut nibbles = vec![0b1100u32];
    let mut n = state as u32 - 3;
    while n >= 15 {
        nibbles.push(0b1111);
        n -= 15;
    }
    nibbles.push(n);
    nibbles.iter().rev().map(|&x| hex(x)).collect()
}

/// Eindeutiger Schlüssel eines (Unter-)Eckpunkts.
#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
enum Key {
    V(u32),
    /// Punkt auf Originalkante (kleinerer, größerer Index, Anteil des größeren)
    E(u32, u32, u64),
    /// Punkt im Inneren des Originaldreiecks
    F(u32, u64, u64),
}

fn key_of(b: &Bary, tv: &[u32; 3], t: u32) -> Key {
    let nz: Vec<usize> = (0..3).filter(|&k| b[k] != 0).collect();
    match nz.len() {
        1 => Key::V(tv[nz[0]]),
        2 => {
            let (i, j) = (nz[0], nz[1]);
            let (a, b_) = (tv[i], tv[j]);
            if a == b_ {
                // entartetes Dreieck
                return Key::F(t, b[1], b[2]);
            }
            if a < b_ {
                Key::E(a, b_, b[j])
            } else {
                Key::E(b_, a, b[i])
            }
        }
        _ => Key::F(t, b[1], b[2]),
    }
}

/// Verschweißt die Eckpunkte (bitgenau gleiche Koordinaten).
fn weld(raw: &RawMesh) -> (Vec<V3>, Vec<[u32; 3]>) {
    let mut map: HashMap<[u64; 3], u32> = HashMap::with_capacity(raw.verts.len());
    let mut verts = Vec::new();
    let mut remap = Vec::with_capacity(raw.verts.len());
    for p in &raw.verts {
        let k = [(p.x + 0.0).to_bits(), (p.y + 0.0).to_bits(), (p.z + 0.0).to_bits()];
        let id = *map.entry(k).or_insert_with(|| {
            verts.push(*p);
            (verts.len() - 1) as u32
        });
        remap.push(id);
    }
    let tris = raw.tris.iter().map(|t| [remap[t[0] as usize], remap[t[1] as usize], remap[t[2] as usize]]).collect();
    (verts, tris)
}

/// Dekodiert die Bemalung und liefert ein Netz mit genau einer Farbe je Dreieck.
/// T-Kreuzungen an unterschiedlich fein unterteilten Nachbardreiecken werden geschlossen.
pub fn refine(raw: &RawMesh) -> (ColoredMesh, usize) {
    let (verts, tris) = weld(raw);
    let n = tris.len();
    let default_col = |t: usize| raw.extruder.get(t).copied().filter(|&e| e > 0).unwrap_or(1);
    let mut bad = 0usize;
    // Blätter je Originaldreieck (Originalreihenfolge der Ecken aus der Datei)
    let mut leaves: Vec<([Bary; 3], u8)> = Vec::new();
    let mut leaf_range: Vec<(usize, usize)> = Vec::with_capacity(n);
    let mut tv_orig: Vec<[u32; 3]> = Vec::with_capacity(n);
    for t in 0..n {
        let flipped = raw.flipped.get(t).copied().unwrap_or(false);
        let tri = tris[t];
        // Bemalung bezieht sich auf die Eckenreihenfolge vor der Spiegelung
        let tv = if flipped { [tri[0], tri[2], tri[1]] } else { tri };
        tv_orig.push(tv);
        let s = leaves.len();
        let code = raw.paint.get(t).map(|s| s.as_str()).unwrap_or("");
        let ok = !code.is_empty() && decode(code, &mut leaves);
        if !ok {
            if !code.is_empty() {
                bad += 1;
            }
            leaves.push(([[S, 0, 0], [0, S, 0], [0, 0, S]], 0));
        }
        leaf_range.push((s, leaves.len()));
    }
    // Eckpunkte anlegen
    let mut out = ColoredMesh { name: raw.name.clone(), verts: Vec::new(), tris: Vec::new(), col: Vec::new() };
    let mut map: HashMap<Key, u32> = HashMap::with_capacity(verts.len() + leaves.len());
    let pos = |k: &Key, tv: &[u32; 3]| -> V3 {
        match *k {
            Key::V(i) => verts[i as usize],
            Key::E(a, b, t) => {
                let (pa, pb) = (verts[a as usize], verts[b as usize]);
                let (wa, wb) = ((S - t) as f64 / S as f64, t as f64 / S as f64);
                v3(pa.x * wa + pb.x * wb, pa.y * wa + pb.y * wb, pa.z * wa + pb.z * wb)
            }
            Key::F(_, b1, b2) => {
                let b0 = S - b1 - b2;
                let (p0, p1, p2) = (verts[tv[0] as usize], verts[tv[1] as usize], verts[tv[2] as usize]);
                let (w0, w1, w2) = (b0 as f64 / S as f64, b1 as f64 / S as f64, b2 as f64 / S as f64);
                v3(
                    p0.x * w0 + p1.x * w1 + p2.x * w2,
                    p0.y * w0 + p1.y * w1 + p2.y * w2,
                    p0.z * w0 + p1.z * w1 + p2.z * w2,
                )
            }
        }
    };
    for t in 0..n {
        let tv = tv_orig[t];
        for l in &leaves[leaf_range[t].0..leaf_range[t].1] {
            for b in &l.0 {
                let k = key_of(b, &tv, t as u32);
                if !map.contains_key(&k) {
                    out.verts.push(pos(&k, &tv));
                    map.insert(k, (out.verts.len() - 1) as u32);
                }
            }
        }
    }
    // Dreiecke erzeugen, T-Kreuzungen schließen
    for t in 0..n {
        let tv = tv_orig[t];
        let flipped = raw.flipped.get(t).copied().unwrap_or(false);
        for l in &leaves[leaf_range[t].0..leaf_range[t].1] {
            let col = if l.1 == 0 { default_col(t) } else { l.1 };
            let mut poly: Vec<u32> = Vec::with_capacity(6);
            let mut extra = false;
            for e in 0..3 {
                let (a, b) = (l.0[e], l.0[(e + 1) % 3]);
                poly.push(map[&key_of(&a, &tv, t as u32)]);
                let before = poly.len();
                edge_points(&a, &b, &tv, t as u32, &map, &mut poly, 0);
                extra |= poly.len() > before;
            }
            let push = |out: &mut ColoredMesh, a: u32, b: u32, c: u32| {
                if a == b || b == c || a == c {
                    return;
                }
                out.tris.push(if flipped { [a, c, b] } else { [a, b, c] });
                out.col.push(col);
            };
            if !extra {
                push(&mut out, poly[0], poly[1], poly[2]);
            } else {
                // Fächer vom Schwerpunkt aus (sternförmig, da alle Zusatzpunkte auf den Kanten liegen)
                let mut c = v3(0.0, 0.0, 0.0);
                for k in 0..3 {
                    c = c + pos(&key_of(&l.0[k], &tv, t as u32), &tv);
                }
                c = c * (1.0 / 3.0);
                out.verts.push(c);
                let ci = (out.verts.len() - 1) as u32;
                for k in 0..poly.len() {
                    push(&mut out, poly[k], poly[(k + 1) % poly.len()], ci);
                }
            }
        }
    }
    (out, bad)
}

/// Fügt alle vorhandenen Unterteilungspunkte zwischen a und b (exklusive) in Reihenfolge an.
fn edge_points(a: &Bary, b: &Bary, tv: &[u32; 3], t: u32, map: &HashMap<Key, u32>, out: &mut Vec<u32>, depth: usize) {
    if depth > MAX_DEPTH {
        return;
    }
    let m = match mid(a, b) {
        Some(m) => m,
        None => return,
    };
    if let Some(&i) = map.get(&key_of(&m, tv, t)) {
        edge_points(a, &m, tv, t, map, out, depth + 1);
        out.push(i);
        edge_points(&m, b, tv, t, map, out, depth + 1);
    }
}

/// Lädt eine Datei für das Aufteilen: 3MF mit Bemalung oder STL (eine Farbe).
pub fn load_colored(path: &std::path::Path) -> Result<(Vec<ColoredMesh>, Vec<[u8; 3]>, Vec<String>), String> {
    let data = std::fs::read(path).map_err(|e| format!("Datei kann nicht gelesen werden: {e}"))?;
    let stem = path.file_stem().map(|s| s.to_string_lossy().to_string()).unwrap_or_else(|| "Teil".into());
    let ext = path.extension().map(|e| e.to_string_lossy().to_ascii_lowercase()).unwrap_or_default();
    let is_zip = data.len() > 4 && &data[0..4] == b"PK\x03\x04";
    let (meshes, colors) = if ext == "3mf" || is_zip {
        let m = crate::io::load_3mf_full(&data, &stem)?;
        (m.meshes, m.colors)
    } else {
        (vec![crate::io::load_stl(&data, &stem)?], Vec::new())
    };
    let mut warnings = Vec::new();
    let mut out = Vec::new();
    for m in meshes.iter().filter(|m| !m.tris.is_empty()) {
        let (mut cm, bad) = refine(m);
        if bad > 0 {
            warnings.push(format!("{}: {bad} Dreiecke mit unlesbarer Bemalung", m.name));
        }
        let flipped = crate::split::orient(&mut cm);
        if flipped > 0 {
            warnings.push(format!("{}: {flipped} Dreiecke umorientiert", m.name));
        }
        out.push(cm);
    }
    if out.is_empty() {
        return Err("Die Datei enthält keine Dreiecke.".into());
    }
    Ok((out, colors, warnings))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn leaf_codes() {
        assert_eq!(encode_leaf(1), "4");
        assert_eq!(encode_leaf(2), "8");
        assert_eq!(encode_leaf(3), "0C");
        assert_eq!(encode_leaf(4), "1C");
        assert_eq!(encode_leaf(18), "0FC");
        for s in 1..40u8 {
            let mut out = Vec::new();
            assert!(decode(&encode_leaf(s), &mut out));
            assert_eq!(out.len(), 1);
            assert_eq!(out[0].1, s);
        }
    }

    /// Kodiert einen Baum wie TriangleSelector::serialize
    enum Node {
        Leaf(u8),
        Split(u8, u8, Vec<Node>), // (Seiten, besondere Seite, Kinder in Reihenfolge)
    }
    fn ser(n: &Node, bits: &mut Vec<bool>) {
        match n {
            Node::Leaf(s) => {
                bits.extend([false, false]);
                if *s >= 3 {
                    bits.extend([true, true]);
                    let v = s - 3;
                    for i in 0..4 {
                        bits.push(v & (1 << i) != 0);
                    }
                } else {
                    bits.extend([s & 1 != 0, s & 2 != 0]);
                }
            }
            Node::Split(sides, sp, ch) => {
                bits.extend([sides & 1 != 0, sides & 2 != 0, sp & 1 != 0, sp & 2 != 0]);
                for c in ch.iter().rev() {
                    ser(c, bits);
                }
            }
        }
    }
    fn to_hex(bits: &[bool]) -> String {
        let mut s = String::new();
        for c in bits.chunks(4) {
            let mut v = 0u32;
            for (i, &b) in c.iter().enumerate() {
                if b {
                    v |= 1 << i;
                }
            }
            s.insert(0, char::from_digit(v, 16).unwrap().to_ascii_uppercase());
        }
        s
    }

    #[test]
    fn split_tree_and_tjunctions() {
        // Quadrat aus zwei Dreiecken; das erste wird 3-fach geteilt, ein Kind nochmal einfach
        let tree = Node::Split(
            3,
            0,
            vec![Node::Leaf(1), Node::Leaf(2), Node::Split(1, 1, vec![Node::Leaf(3), Node::Leaf(1)]), Node::Leaf(4)],
        );
        let mut bits = Vec::new();
        ser(&tree, &mut bits);
        let code = to_hex(&bits);
        let mut leaves = Vec::new();
        assert!(decode(&code, &mut leaves));
        assert_eq!(leaves.len(), 5);
        let cols: Vec<u8> = leaves.iter().map(|l| l.1).collect();
        // Kinder werden rückwärts gelesen
        assert_eq!(cols, vec![4, 1, 3, 2, 1]);
        let raw = RawMesh {
            name: "q".into(),
            verts: vec![v3(0., 0., 0.), v3(1., 0., 0.), v3(1., 1., 0.), v3(0., 1., 0.)],
            tris: vec![[0, 1, 2], [0, 2, 3]],
            paint: vec![code, String::new()],
            flipped: vec![false, false],
            extruder: vec![5, 5],
        };
        let (m, bad) = refine(&raw);
        assert_eq!(bad, 0);
        // Fläche erhalten
        let area: f64 = m
            .tris
            .iter()
            .map(|t| {
                let (a, b, c) = (m.verts[t[0] as usize], m.verts[t[1] as usize], m.verts[t[2] as usize]);
                (b - a).cross(c - a).z * 0.5
            })
            .sum();
        assert!((area - 1.0).abs() < 1e-12, "Fläche {area}");
        // keine T-Kreuzungen: jede innere Kante genau zweimal (gegenläufig)
        let mut em: HashMap<(u32, u32), i32> = HashMap::new();
        for t in &m.tris {
            for k in 0..3 {
                let (a, b) = (t[k], t[(k + 1) % 3]);
                *em.entry((a.min(b), a.max(b))).or_default() += if a < b { 1 } else { -1 };
            }
        }
        assert!(em.values().all(|&v| v == 0 || v.abs() == 1));
        let open = em.values().filter(|&&v| v != 0).count();
        // Rand des Quadrats: 4 Originalkanten, unterteilt -> mehr als 4, aber alle innen geschlossen
        let boundary_len: usize = open;
        assert!(boundary_len >= 4);
        // zweites Dreieck: Standardfarbe 5
        assert!(m.col.contains(&5));
        assert!(m.col.contains(&4));
    }
}
