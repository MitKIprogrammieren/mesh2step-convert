//! Kleiner JSON-Leser (für Ergebnisse externer Engines und Update-Abfragen).

#[derive(Clone, Debug, PartialEq)]
pub enum Json {
    Null,
    Bool(bool),
    Num(f64),
    Str(String),
    Arr(Vec<Json>),
    Obj(Vec<(String, Json)>),
}

impl Json {
    pub fn get(&self, key: &str) -> Option<&Json> {
        match self {
            Json::Obj(v) => v.iter().find(|(k, _)| k == key).map(|(_, x)| x),
            _ => None,
        }
    }
    /// Pfad wie "assets/macos-arm64/sha256"
    pub fn path(&self, p: &str) -> Option<&Json> {
        let mut cur = self;
        for k in p.split('/') {
            cur = cur.get(k)?;
        }
        Some(cur)
    }
    pub fn str(&self) -> Option<&str> {
        match self {
            Json::Str(s) => Some(s),
            _ => None,
        }
    }
    pub fn num(&self) -> Option<f64> {
        match self {
            Json::Num(x) => Some(*x),
            _ => None,
        }
    }
    pub fn bool(&self) -> Option<bool> {
        match self {
            Json::Bool(b) => Some(*b),
            _ => None,
        }
    }
    pub fn arr(&self) -> &[Json] {
        match self {
            Json::Arr(v) => v,
            _ => &[],
        }
    }
}

pub fn parse(s: &str) -> Result<Json, String> {
    let b = s.as_bytes();
    let mut i = 0;
    let v = value(b, &mut i)?;
    Ok(v)
}

fn ws(b: &[u8], i: &mut usize) {
    while *i < b.len() && (b[*i] as char).is_ascii_whitespace() {
        *i += 1;
    }
}

fn value(b: &[u8], i: &mut usize) -> Result<Json, String> {
    ws(b, i);
    if *i >= b.len() {
        return Err("JSON endet unerwartet".into());
    }
    match b[*i] {
        b'{' => {
            *i += 1;
            let mut v = Vec::new();
            ws(b, i);
            if *i < b.len() && b[*i] == b'}' {
                *i += 1;
                return Ok(Json::Obj(v));
            }
            loop {
                ws(b, i);
                let k = match value(b, i)? {
                    Json::Str(s) => s,
                    _ => return Err("JSON-Schlüssel erwartet".into()),
                };
                ws(b, i);
                if *i >= b.len() || b[*i] != b':' {
                    return Err("':' erwartet".into());
                }
                *i += 1;
                let x = value(b, i)?;
                v.push((k, x));
                ws(b, i);
                match b.get(*i) {
                    Some(b',') => *i += 1,
                    Some(b'}') => {
                        *i += 1;
                        return Ok(Json::Obj(v));
                    }
                    _ => return Err("',' oder '}' erwartet".into()),
                }
            }
        }
        b'[' => {
            *i += 1;
            let mut v = Vec::new();
            ws(b, i);
            if *i < b.len() && b[*i] == b']' {
                *i += 1;
                return Ok(Json::Arr(v));
            }
            loop {
                v.push(value(b, i)?);
                ws(b, i);
                match b.get(*i) {
                    Some(b',') => *i += 1,
                    Some(b']') => {
                        *i += 1;
                        return Ok(Json::Arr(v));
                    }
                    _ => return Err("',' oder ']' erwartet".into()),
                }
            }
        }
        b'"' => {
            *i += 1;
            let mut out: Vec<u8> = Vec::new();
            while *i < b.len() {
                let c = b[*i];
                *i += 1;
                match c {
                    b'"' => return Ok(Json::Str(String::from_utf8_lossy(&out).into_owned())),
                    b'\\' => {
                        let e = *b.get(*i).ok_or("Escape am Ende")?;
                        *i += 1;
                        match e {
                            b'n' => out.push(b'\n'),
                            b't' => out.push(b'\t'),
                            b'r' => out.push(b'\r'),
                            b'b' => out.push(8),
                            b'f' => out.push(12),
                            b'u' => {
                                let h = std::str::from_utf8(b.get(*i..*i + 4).ok_or("\\u unvollständig")?).map_err(|_| "\\u ungültig")?;
                                let mut cp = u32::from_str_radix(h, 16).map_err(|_| "\\u ungültig")?;
                                *i += 4;
                                // Ersatzpaar
                                if (0xD800..0xDC00).contains(&cp) && b.get(*i) == Some(&b'\\') && b.get(*i + 1) == Some(&b'u') {
                                    if let Some(h2) = b.get(*i + 2..*i + 6).and_then(|x| std::str::from_utf8(x).ok()).and_then(|x| u32::from_str_radix(x, 16).ok()) {
                                        if (0xDC00..0xE000).contains(&h2) {
                                            cp = 0x10000 + ((cp - 0xD800) << 10) + (h2 - 0xDC00);
                                            *i += 6;
                                        }
                                    }
                                }
                                let ch = char::from_u32(cp).unwrap_or('?');
                                let mut buf = [0u8; 4];
                                out.extend_from_slice(ch.encode_utf8(&mut buf).as_bytes());
                            }
                            o => out.push(o),
                        }
                    }
                    o => out.push(o),
                }
            }
            Err("Zeichenkette nicht geschlossen".into())
        }
        b't' if b[*i..].starts_with(b"true") => {
            *i += 4;
            Ok(Json::Bool(true))
        }
        b'f' if b[*i..].starts_with(b"false") => {
            *i += 5;
            Ok(Json::Bool(false))
        }
        b'n' if b[*i..].starts_with(b"null") => {
            *i += 4;
            Ok(Json::Null)
        }
        _ => {
            let st = *i;
            while *i < b.len() && matches!(b[*i], b'-' | b'+' | b'.' | b'e' | b'E' | b'0'..=b'9') {
                *i += 1;
            }
            let t = std::str::from_utf8(&b[st..*i]).unwrap_or("");
            t.parse::<f64>().map(Json::Num).map_err(|_| format!("Ungültiger JSON-Wert bei Zeichen {st}"))
        }
    }
}

/// Zeichenkette als JSON-Text (für Ausgaben).
pub fn quote(s: &str) -> String {
    let mut o = String::from("\"");
    for c in s.chars() {
        match c {
            '"' => o.push_str("\\\""),
            '\\' => o.push_str("\\\\"),
            '\n' => o.push_str("\\n"),
            c if (c as u32) < 0x20 => o.push_str(&format!("\\u{:04x}", c as u32)),
            c => o.push(c),
        }
    }
    o.push('"');
    o
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn grundlagen() {
        let j = parse(r#"{"ok":true,"n":-1.5e2,"s":"a\"bä","l":[1,{"x":null}],"e":{}}"#).unwrap();
        assert_eq!(j.get("ok").and_then(|x| x.bool()), Some(true));
        assert_eq!(j.get("n").and_then(|x| x.num()), Some(-150.0));
        assert_eq!(j.get("s").and_then(|x| x.str()), Some("a\"bä"));
        assert_eq!(j.path("l").map(|x| x.arr().len()), Some(2));
    }
}
