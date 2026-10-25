//! Minimaler ZIP-Leser (Store + Deflate, ZIP64) für 3MF-Dateien.
//! Komplett ohne externe Abhängigkeiten.

use std::collections::HashMap;

pub struct ZipArchive {
    pub files: HashMap<String, Vec<u8>>,
}

fn rd16(b: &[u8], o: usize) -> Option<u64> {
    Some(u16::from_le_bytes(b.get(o..o + 2)?.try_into().ok()?) as u64)
}
fn rd32(b: &[u8], o: usize) -> Option<u64> {
    Some(u32::from_le_bytes(b.get(o..o + 4)?.try_into().ok()?) as u64)
}
fn rd64(b: &[u8], o: usize) -> Option<u64> {
    Some(u64::from_le_bytes(b.get(o..o + 8)?.try_into().ok()?))
}

impl ZipArchive {
    pub fn read(data: &[u8]) -> Result<ZipArchive, String> {
        Self::read_inner(data).ok_or_else(|| "Ungültiges oder beschädigtes ZIP/3MF-Archiv".to_string())?
    }

    fn read_inner(data: &[u8]) -> Option<Result<ZipArchive, String>> {
        // End of central directory suchen (von hinten)
        let min = data.len().saturating_sub(65557);
        let mut eocd = None;
        let mut i = data.len().checked_sub(22)?;
        loop {
            if rd32(data, i)? == 0x0605_4b50 {
                eocd = Some(i);
                break;
            }
            if i == min || i == 0 {
                break;
            }
            i -= 1;
        }
        let eocd = eocd?;
        let mut count = rd16(data, eocd + 10)?;
        let mut cd_off = rd32(data, eocd + 16)?;
        // ZIP64?
        if (count == 0xFFFF || cd_off == 0xFFFF_FFFF) && eocd >= 20 && rd32(data, eocd - 20)? == 0x0706_4b50 {
            let z64 = rd64(data, eocd - 20 + 8)? as usize;
            if rd32(data, z64)? == 0x0606_4b50 {
                count = rd64(data, z64 + 32)?;
                cd_off = rd64(data, z64 + 48)?;
            }
        }
        let mut files = HashMap::new();
        let mut p = cd_off as usize;
        for _ in 0..count {
            if rd32(data, p)? != 0x0201_4b50 {
                break;
            }
            let method = rd16(data, p + 10)?;
            let mut csize = rd32(data, p + 20)?;
            let mut usize_ = rd32(data, p + 24)?;
            let nlen = rd16(data, p + 28)? as usize;
            let elen = rd16(data, p + 30)? as usize;
            let clen = rd16(data, p + 32)? as usize;
            let mut loff = rd32(data, p + 42)?;
            let name = String::from_utf8_lossy(data.get(p + 46..p + 46 + nlen)?).to_string();
            // ZIP64-Extra-Feld
            let mut e = p + 46 + nlen;
            let eend = e + elen;
            while e + 4 <= eend {
                let id = rd16(data, e)?;
                let sz = rd16(data, e + 2)? as usize;
                if id == 0x0001 {
                    let mut q = e + 4;
                    if usize_ == 0xFFFF_FFFF {
                        usize_ = rd64(data, q)?;
                        q += 8;
                    }
                    if csize == 0xFFFF_FFFF {
                        csize = rd64(data, q)?;
                        q += 8;
                    }
                    if loff == 0xFFFF_FFFF {
                        loff = rd64(data, q)?;
                    }
                }
                e += 4 + sz;
            }
            p = eend + clen;
            if name.ends_with('/') {
                continue;
            }
            let lo = loff as usize;
            if rd32(data, lo)? != 0x0403_4b50 {
                return Some(Err(format!("Defekter Eintrag im Archiv: {name}")));
            }
            let lnlen = rd16(data, lo + 26)? as usize;
            let lelen = rd16(data, lo + 28)? as usize;
            let start = lo + 30 + lnlen + lelen;
            let comp = data.get(start..start + csize as usize)?;
            let content = match method {
                0 => comp.to_vec(),
                8 => match inflate(comp, usize_ as usize) {
                    Ok(v) => v,
                    Err(e) => return Some(Err(format!("{name}: {e}"))),
                },
                m => return Some(Err(format!("Nicht unterstützte ZIP-Kompression ({m}) in {name}"))),
            };
            files.insert(name, content);
        }
        Some(Ok(ZipArchive { files }))
    }

    /// Datei unabhängig von Groß-/Kleinschreibung und führendem '/' finden.
    pub fn get(&self, path: &str) -> Option<&Vec<u8>> {
        let p = path.trim_start_matches('/');
        if let Some(v) = self.files.get(p) {
            return Some(v);
        }
        let pl = p.to_ascii_lowercase();
        self.files.iter().find(|(k, _)| k.to_ascii_lowercase() == pl).map(|(_, v)| v)
    }
}

// ---------------------------------------------------------------------------
// DEFLATE (RFC 1951)

struct BitReader<'a> {
    data: &'a [u8],
    pos: usize,
    bit: u32,
    nbits: u32,
}

impl<'a> BitReader<'a> {
    fn new(data: &'a [u8]) -> Self {
        BitReader { data, pos: 0, bit: 0, nbits: 0 }
    }
    #[inline]
    fn need(&mut self, n: u32) -> Result<(), String> {
        while self.nbits < n {
            let b = *self.data.get(self.pos).unwrap_or(&0);
            if self.pos > self.data.len() + 8 {
                return Err("Unerwartetes Ende der komprimierten Daten".into());
            }
            self.pos += 1;
            self.bit |= (b as u32) << self.nbits;
            self.nbits += 8;
        }
        Ok(())
    }
    #[inline]
    fn bits(&mut self, n: u32) -> Result<u32, String> {
        if n == 0 {
            return Ok(0);
        }
        self.need(n)?;
        let v = self.bit & ((1u32 << n) - 1);
        self.bit >>= n;
        self.nbits -= n;
        Ok(v)
    }
    fn align(&mut self) {
        self.bit = 0;
        self.nbits = 0;
    }
}

struct Huffman {
    counts: [u16; 16],
    symbols: Vec<u16>,
}

impl Huffman {
    fn new(lengths: &[u8]) -> Huffman {
        let mut counts = [0u16; 16];
        for &l in lengths {
            counts[l as usize] += 1;
        }
        counts[0] = 0;
        let mut offs = [0u16; 16];
        for i in 1..16 {
            offs[i] = offs[i - 1] + counts[i - 1];
        }
        let mut symbols = vec![0u16; lengths.len()];
        for (s, &l) in lengths.iter().enumerate() {
            if l != 0 {
                symbols[offs[l as usize] as usize] = s as u16;
                offs[l as usize] += 1;
            }
        }
        Huffman { counts, symbols }
    }

    fn decode(&self, br: &mut BitReader) -> Result<u16, String> {
        let mut code: i32 = 0;
        let mut first: i32 = 0;
        let mut index: i32 = 0;
        for len in 1..16 {
            code |= br.bits(1)? as i32;
            let count = self.counts[len] as i32;
            if code - count < first {
                return Ok(self.symbols[(index + (code - first)) as usize]);
            }
            index += count;
            first += count;
            first <<= 1;
            code <<= 1;
        }
        Err("Ungültiger Huffman-Code".into())
    }
}

const LBASE: [u16; 29] = [
    3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 15, 17, 19, 23, 27, 31, 35, 43, 51, 59, 67, 83, 99, 115, 131, 163, 195, 227, 258,
];
const LEXT: [u8; 29] = [0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 0];
const DBASE: [u16; 30] = [
    1, 2, 3, 4, 5, 7, 9, 13, 17, 25, 33, 49, 65, 97, 129, 193, 257, 385, 513, 769, 1025, 1537, 2049, 3073, 4097, 6145,
    8193, 12289, 16385, 24577,
];
const DEXT: [u8; 30] = [0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13];

pub fn inflate(data: &[u8], size_hint: usize) -> Result<Vec<u8>, String> {
    let mut out: Vec<u8> = Vec::with_capacity(size_hint);
    let mut br = BitReader::new(data);
    loop {
        let last = br.bits(1)?;
        let typ = br.bits(2)?;
        match typ {
            0 => {
                br.align();
                let p = br.pos;
                let len = u16::from_le_bytes([*data.get(p).ok_or("EOF")?, *data.get(p + 1).ok_or("EOF")?]) as usize;
                let blk = data.get(p + 4..p + 4 + len).ok_or("Unerwartetes Ende (Stored-Block)")?;
                out.extend_from_slice(blk);
                br.pos = p + 4 + len;
            }
            1 | 2 => {
                let (lit, dist) = if typ == 1 {
                    let mut l = [0u8; 288];
                    for (i, v) in l.iter_mut().enumerate() {
                        *v = if i < 144 {
                            8
                        } else if i < 256 {
                            9
                        } else if i < 280 {
                            7
                        } else {
                            8
                        };
                    }
                    (Huffman::new(&l), Huffman::new(&[5u8; 30]))
                } else {
                    let hlit = br.bits(5)? as usize + 257;
                    let hdist = br.bits(5)? as usize + 1;
                    let hclen = br.bits(4)? as usize + 4;
                    const ORD: [usize; 19] = [16, 17, 18, 0, 8, 7, 9, 6, 10, 5, 11, 4, 12, 3, 13, 2, 14, 1, 15];
                    let mut cl = [0u8; 19];
                    for &o in ORD.iter().take(hclen) {
                        cl[o] = br.bits(3)? as u8;
                    }
                    let clh = Huffman::new(&cl);
                    let mut lens = vec![0u8; hlit + hdist];
                    let mut i = 0;
                    while i < hlit + hdist {
                        let sym = clh.decode(&mut br)?;
                        match sym {
                            0..=15 => {
                                lens[i] = sym as u8;
                                i += 1;
                            }
                            16 => {
                                if i == 0 {
                                    return Err("Ungültige Längencodes".into());
                                }
                                let prev = lens[i - 1];
                                let r = 3 + br.bits(2)? as usize;
                                for _ in 0..r {
                                    if i < lens.len() {
                                        lens[i] = prev;
                                        i += 1;
                                    }
                                }
                            }
                            17 => {
                                let r = 3 + br.bits(3)? as usize;
                                i += r;
                            }
                            _ => {
                                let r = 11 + br.bits(7)? as usize;
                                i += r;
                            }
                        }
                    }
                    lens.truncate(hlit + hdist);
                    (Huffman::new(&lens[..hlit]), Huffman::new(&lens[hlit..]))
                };
                loop {
                    let sym = lit.decode(&mut br)? as usize;
                    if sym < 256 {
                        out.push(sym as u8);
                    } else if sym == 256 {
                        break;
                    } else {
                        let si = sym - 257;
                        if si >= 29 {
                            return Err("Ungültiges Längensymbol".into());
                        }
                        let len = LBASE[si] as usize + br.bits(LEXT[si] as u32)? as usize;
                        let ds = dist.decode(&mut br)? as usize;
                        if ds >= 30 {
                            return Err("Ungültiges Distanzsymbol".into());
                        }
                        let d = DBASE[ds] as usize + br.bits(DEXT[ds] as u32)? as usize;
                        if d > out.len() {
                            return Err("Ungültige Rückwärtsdistanz".into());
                        }
                        let start = out.len() - d;
                        for k in 0..len {
                            let b = out[start + k];
                            out.push(b);
                        }
                    }
                }
            }
            _ => return Err("Ungültiger Deflate-Blocktyp".into()),
        }
        if last == 1 {
            break;
        }
    }
    Ok(out)
}

// ---------------------------------------------------------------------------
// Schreiben: Deflate (feste Huffman-Codes, LZ77) und ZIP-Archiv

struct BitWriter {
    out: Vec<u8>,
    acc: u64,
    n: u32,
}

impl BitWriter {
    fn put(&mut self, v: u32, bits: u32) {
        self.acc |= (v as u64) << self.n;
        self.n += bits;
        while self.n >= 8 {
            self.out.push(self.acc as u8);
            self.acc >>= 8;
            self.n -= 8;
        }
    }
    /// Huffman-Code (MSB zuerst) schreiben
    fn code(&mut self, c: u32, len: u32) {
        let mut r = 0u32;
        for i in 0..len {
            r |= ((c >> i) & 1) << (len - 1 - i);
        }
        self.put(r, len);
    }
    fn finish(mut self) -> Vec<u8> {
        if self.n > 0 {
            self.out.push(self.acc as u8);
        }
        self.out
    }
}

fn put_lit(w: &mut BitWriter, v: usize) {
    match v {
        0..=143 => w.code(0x30 + v as u32, 8),
        144..=255 => w.code(0x190 + (v - 144) as u32, 9),
        256..=279 => w.code((v - 256) as u32, 7),
        _ => w.code(0xC0 + (v - 280) as u32, 8),
    }
}

/// Komprimiert mit festen Huffman-Codes (ausreichend für XML-Text).
pub fn deflate(data: &[u8]) -> Vec<u8> {
    let mut w = BitWriter { out: Vec::with_capacity(data.len() / 3), acc: 0, n: 0 };
    w.put(1, 1); // letzter Block
    w.put(1, 2); // feste Codes
    const WIN: usize = 32768;
    const HBITS: u32 = 15;
    let mut head = vec![usize::MAX; 1 << HBITS];
    let mut prev = vec![usize::MAX; WIN];
    let hash = |i: usize| -> usize {
        let v = (data[i] as u32) | (data[i + 1] as u32) << 8 | (data[i + 2] as u32) << 16;
        (v.wrapping_mul(2654435761) >> (32 - HBITS)) as usize
    };
    let n = data.len();
    let mut i = 0;
    let insert = |i: usize, head: &mut Vec<usize>, prev: &mut Vec<usize>| {
        if i + 2 < n {
            let h = hash(i);
            prev[i % WIN] = head[h];
            head[h] = i;
        }
    };
    while i < n {
        let mut best_len = 0;
        let mut best_d = 0;
        if i + 2 < n {
            let h = hash(i);
            let mut cand = head[h];
            let mut chain = 0;
            let maxl = (n - i).min(258);
            while cand != usize::MAX && i - cand <= WIN - 1 && chain < 48 {
                if data[cand + best_len.min(maxl - 1)] == data[i + best_len.min(maxl - 1)] {
                    let mut l = 0;
                    while l < maxl && data[cand + l] == data[i + l] {
                        l += 1;
                    }
                    if l > best_len {
                        best_len = l;
                        best_d = i - cand;
                        if l == maxl {
                            break;
                        }
                    }
                }
                let p = prev[cand % WIN];
                if p == usize::MAX || p >= cand {
                    break;
                }
                cand = p;
                chain += 1;
            }
        }
        if best_len >= 3 {
            let li = LBASE.iter().rposition(|&b| b as usize <= best_len).unwrap();
            put_lit(&mut w, 257 + li);
            if LEXT[li] > 0 {
                w.put((best_len - LBASE[li] as usize) as u32, LEXT[li] as u32);
            }
            let di = DBASE.iter().rposition(|&b| b as usize <= best_d).unwrap();
            w.code(di as u32, 5);
            if DEXT[di] > 0 {
                w.put((best_d - DBASE[di] as usize) as u32, DEXT[di] as u32);
            }
            for k in 0..best_len {
                insert(i + k, &mut head, &mut prev);
            }
            i += best_len;
        } else {
            put_lit(&mut w, data[i] as usize);
            insert(i, &mut head, &mut prev);
            i += 1;
        }
    }
    put_lit(&mut w, 256);
    w.finish()
}

pub fn crc32(data: &[u8]) -> u32 {
    static TABLE: std::sync::OnceLock<[u32; 256]> = std::sync::OnceLock::new();
    let t = TABLE.get_or_init(|| {
        let mut t = [0u32; 256];
        for (i, e) in t.iter_mut().enumerate() {
            let mut c = i as u32;
            for _ in 0..8 {
                c = if c & 1 != 0 { 0xEDB88320 ^ (c >> 1) } else { c >> 1 };
            }
            *e = c;
        }
        t
    });
    let mut c = 0xFFFF_FFFFu32;
    for &b in data {
        c = t[((c ^ b as u32) & 0xFF) as usize] ^ (c >> 8);
    }
    c ^ 0xFFFF_FFFF
}

/// Einfacher ZIP-Schreiber (Deflate, ohne ZIP64 – bis 4 GB).
#[derive(Default)]
pub struct ZipWriter {
    out: Vec<u8>,
    central: Vec<u8>,
    count: u16,
}

impl ZipWriter {
    pub fn add(&mut self, name: &str, data: &[u8]) {
        let comp = deflate(data);
        let (method, body): (u16, &[u8]) = if comp.len() < data.len() { (8, &comp) } else { (0, data) };
        let crc = crc32(data);
        let off = self.out.len() as u32;
        let nb = name.as_bytes();
        let w16 = |v: &mut Vec<u8>, x: u16| v.extend_from_slice(&x.to_le_bytes());
        let w32 = |v: &mut Vec<u8>, x: u32| v.extend_from_slice(&x.to_le_bytes());
        // lokaler Kopf
        let o = &mut self.out;
        w32(o, 0x04034b50);
        w16(o, 20);
        w16(o, 0x0800); // UTF-8-Namen
        w16(o, method);
        w16(o, 0);
        w16(o, 0x21);
        w32(o, crc);
        w32(o, body.len() as u32);
        w32(o, data.len() as u32);
        w16(o, nb.len() as u16);
        w16(o, 0);
        o.extend_from_slice(nb);
        o.extend_from_slice(body);
        // zentrales Verzeichnis
        let c = &mut self.central;
        w32(c, 0x02014b50);
        w16(c, 20);
        w16(c, 20);
        w16(c, 0x0800);
        w16(c, method);
        w16(c, 0);
        w16(c, 0x21);
        w32(c, crc);
        w32(c, body.len() as u32);
        w32(c, data.len() as u32);
        w16(c, nb.len() as u16);
        w16(c, 0);
        w16(c, 0);
        w16(c, 0);
        w16(c, 0);
        w32(c, 0);
        w32(c, off);
        c.extend_from_slice(nb);
        self.count += 1;
    }

    pub fn finish(mut self) -> Vec<u8> {
        let cd_off = self.out.len() as u32;
        let cd_len = self.central.len() as u32;
        self.out.extend_from_slice(&self.central);
        let o = &mut self.out;
        o.extend_from_slice(&0x06054b50u32.to_le_bytes());
        o.extend_from_slice(&[0, 0, 0, 0]);
        o.extend_from_slice(&self.count.to_le_bytes());
        o.extend_from_slice(&self.count.to_le_bytes());
        o.extend_from_slice(&cd_len.to_le_bytes());
        o.extend_from_slice(&cd_off.to_le_bytes());
        o.extend_from_slice(&[0, 0]);
        self.out
    }
}

#[cfg(test)]
mod write_tests {
    use super::*;

    #[test]
    fn deflate_roundtrip() {
        let mut data = Vec::new();
        for i in 0..20000 {
            data.extend_from_slice(format!("<vertex x=\"{}\" y=\"{}\" z=\"0\"/>\n", i % 97, i * 7 % 13).as_bytes());
        }
        data.extend((0..5000u32).map(|i| (i.wrapping_mul(2654435761) >> 13) as u8));
        let c = deflate(&data);
        assert!(c.len() < data.len() / 3);
        let d = inflate(&c, data.len()).unwrap();
        assert_eq!(d, data);
        let mut z = ZipWriter::default();
        z.add("a/b.txt", &data);
        z.add("leer.txt", b"");
        let bytes = z.finish();
        let r = ZipArchive::read(&bytes).unwrap();
        assert_eq!(r.get("a/b.txt").unwrap(), &data);
        assert_eq!(r.get("leer.txt").unwrap().len(), 0);
        assert_eq!(crc32(b"123456789"), 0xCBF43926);
    }
}
