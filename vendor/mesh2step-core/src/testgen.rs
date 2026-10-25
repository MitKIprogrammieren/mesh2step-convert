//! Erzeugt tessellierte Prüfkörper (wie aus einem CAD-Programm exportiert).

use crate::io::RawMesh;
use crate::math::{v3, V3};
use std::f64::consts::{PI, TAU};

/// Rotationskörper aus geschlossenem (r, z)-Profil um die Z-Achse.
pub fn revolve(name: &str, profile: &[(f64, f64)], n: usize) -> RawMesh {
    let mut verts: Vec<V3> = Vec::new();
    let mut ids: Vec<Vec<u32>> = Vec::new();
    for &(r, z) in profile {
        if r.abs() < 1e-12 {
            let id = verts.len() as u32;
            verts.push(v3(0.0, 0.0, z));
            ids.push(vec![id; n]);
        } else {
            let mut row = Vec::with_capacity(n);
            for j in 0..n {
                let t = TAU * j as f64 / n as f64;
                row.push(verts.len() as u32);
                verts.push(v3(r * t.cos(), r * t.sin(), z));
            }
            ids.push(row);
        }
    }
    let mut tris = Vec::new();
    let m = profile.len();
    for i in 0..m {
        let a = &ids[i];
        let b = &ids[(i + 1) % m];
        for j in 0..n {
            let j2 = (j + 1) % n;
            let (a0, a1, b0, b1) = (a[j], a[j2], b[j], b[j2]);
            if a0 != a1 {
                tris.push([a0, a1, b1]);
            }
            if b0 != b1 {
                tris.push([a0, b1, b0]);
            }
        }
    }
    RawMesh { name: name.into(), verts, tris, ..Default::default() }
}

/// Bogen im Profil von Winkel a0 nach a1 (Grad).
pub fn arc(c: (f64, f64), r: f64, a0: f64, a1: f64, segs: usize) -> Vec<(f64, f64)> {
    (0..=segs)
        .map(|i| {
            let t = (a0 + (a1 - a0) * i as f64 / segs as f64).to_radians();
            (c.0 + r * t.cos(), c.1 + r * t.sin())
        })
        .collect()
}

/// Extrusion eines sternförmigen Außenprofils mit optionaler runder Bohrung um den Ursprung.
/// `outer(theta)` liefert den Außenpunkt in Richtung theta; `extra` sind zusätzliche Winkel (Ecken).
fn extrude_star(name: &str, outer: &dyn Fn(f64) -> (f64, f64), extra: &[f64], hole_r: Option<f64>, n: usize, h: f64) -> RawMesh {
    let mut angles: Vec<f64> = (0..n).map(|i| TAU * i as f64 / n as f64).collect();
    angles.extend(extra.iter().map(|a| a.rem_euclid(TAU)));
    angles.sort_by(|a, b| a.partial_cmp(b).unwrap());
    angles.dedup_by(|a, b| (*a - *b).abs() < 1e-9);
    let ring_out: Vec<(f64, f64)> = angles.iter().map(|&t| outer(t)).collect();
    let hole_angles: Vec<f64> = (0..n).map(|i| TAU * i as f64 / n as f64).collect();
    let mut verts = Vec::new();
    let mut tris: Vec<[u32; 3]> = Vec::new();
    let layer = |z: f64, verts: &mut Vec<V3>| -> (Vec<u32>, Vec<u32>, u32) {
        let o: Vec<u32> = ring_out
            .iter()
            .map(|&(x, y)| {
                verts.push(v3(x, y, z));
                verts.len() as u32 - 1
            })
            .collect();
        let i: Vec<u32> = match hole_r {
            Some(r) => hole_angles
                .iter()
                .map(|&t| {
                    verts.push(v3(r * t.cos(), r * t.sin(), z));
                    verts.len() as u32 - 1
                })
                .collect(),
            None => vec![],
        };
        verts.push(v3(0.0, 0.0, z));
        (o, i, verts.len() as u32 - 1)
    };
    let (o0, i0, c0) = layer(0.0, &mut verts);
    let (o1, i1, c1) = layer(h, &mut verts);
    // Deckel
    for (o, i, c, top) in [(&o0, &i0, c0, false), (&o1, &i1, c1, true)] {
        let mut add = |a: u32, b: u32, cc: u32| {
            if top {
                tris.push([a, b, cc]);
            } else {
                tris.push([a, cc, b]);
            }
        };
        if i.is_empty() {
            for k in 0..o.len() {
                add(o[k], o[(k + 1) % o.len()], c);
            }
        } else {
            // Reißverschluss zwischen Außen- und Innenring nach Winkel
            let (no, ni) = (o.len(), i.len());
            let (mut a, mut b) = (0usize, 0usize);
            while a < no || b < ni {
                let ta = if a < no { angles[a] } else { f64::MAX };
                let tb = if b < ni { hole_angles[b] } else { f64::MAX };
                let take_outer = if a >= no {
                    false
                } else if b >= ni {
                    true
                } else {
                    let ta_next = if a + 1 < no { angles[a + 1] } else { TAU };
                    let tb_next = if b + 1 < ni { hole_angles[b + 1] } else { TAU };
                    let _ = (ta, tb);
                    ta_next <= tb_next
                };
                if take_outer {
                    add(o[a], o[(a + 1) % no], i[b % ni]);
                    a += 1;
                } else {
                    add(o[a % no], i[(b + 1) % ni], i[b]);
                    b += 1;
                }
            }
        }
    }
    // Wände
    let no = o0.len();
    for k in 0..no {
        let k2 = (k + 1) % no;
        tris.push([o0[k], o0[k2], o1[k2]]);
        tris.push([o0[k], o1[k2], o1[k]]);
    }
    let ni = i0.len();
    for k in 0..ni {
        let k2 = (k + 1) % ni;
        tris.push([i0[k], i1[k2], i0[k2]]);
        tris.push([i0[k], i1[k], i1[k2]]);
    }
    // Zentrumspunkte entfernen, falls ungenutzt (bei Bohrung)
    RawMesh { name: name.into(), verts, tris, ..Default::default() }
}

/// Quader w x d x h mit zentrischer Bohrung.
pub fn box_with_hole(w: f64, d: f64, h: f64, r: f64, n: usize) -> RawMesh {
    let (hw, hd) = (w / 2.0, d / 2.0);
    let outer = move |t: f64| {
        let (c, s) = (t.cos(), t.sin());
        let k = (hw / c.abs().max(1e-12)).min(hd / s.abs().max(1e-12));
        (c * k, s * k)
    };
    let corners = [hd.atan2(hw), hd.atan2(-hw), (-hd).atan2(-hw), (-hd).atan2(hw)];
    extrude_star("box_hole", &outer, &corners, Some(r), n, h)
}

/// Platte mit verrundeten senkrechten Kanten (tangentiale Übergänge).
pub fn rounded_plate(w: f64, d: f64, h: f64, rf: f64, seg: usize) -> RawMesh {
    let (hw, hd) = (w / 2.0, d / 2.0);
    let mut pts: Vec<(f64, f64)> = Vec::new();
    let cs = [(hw - rf, hd - rf, 0.0), (-hw + rf, hd - rf, 90.0), (-hw + rf, -hd + rf, 180.0), (hw - rf, -hd + rf, 270.0)];
    for (cx, cy, a0) in cs {
        for i in 0..=seg {
            let t = (a0 + 90.0 * i as f64 / seg as f64).to_radians();
            pts.push((cx + rf * t.cos(), cy + rf * t.sin()));
        }
    }
    let angles: Vec<f64> = pts.iter().map(|&(x, y)| y.atan2(x)).collect();
    let pts2 = pts.clone();
    let angs2 = angles.clone();
    let outer = move |t: f64| {
        let i = angs2.iter().position(|a| (a.rem_euclid(TAU) - t).abs() < 1e-9).unwrap_or(0);
        pts2[i]
    };
    let mut m = extrude_star("rounded_plate", &outer, &angles, None, 0, h);
    m.name = "rounded_plate".into();
    m
}

pub fn shapes() -> Vec<RawMesh> {
    let mut v = Vec::new();
    v.push(box_with_hole(40.0, 30.0, 10.0, 6.0, 48));
    v.push(rounded_plate(50.0, 30.0, 5.0, 6.0, 8));
    // Zylinder
    v.push(revolve("zylinder", &[(0.0, 0.0), (10.0, 0.0), (10.0, 20.0), (0.0, 20.0)], 64));
    // Gestufte Welle mit Fase (Kegel) und Rundung (Torus) und Bohrung
    let mut prof = vec![(3.0, 0.0), (12.0, 0.0), (12.0, 10.0)];
    prof.extend(arc((10.0, 10.0), 2.0, 0.0, 90.0, 6).into_iter().skip(1));
    prof.extend([(8.0, 12.0), (8.0, 25.0), (6.0, 27.0), (3.0, 27.0)]);
    v.push(revolve("welle", &prof, 72));
    // Kugel
    let mut sp = arc((0.0, 0.0), 15.0, -90.0, 90.0, 24);
    sp[0].0 = 0.0;
    sp[24].0 = 0.0;
    v.push(revolve("kugel", &sp, 48));
    // Kuppel auf Zylinder
    let mut dome = vec![(0.0, 0.0), (10.0, 0.0), (10.0, 15.0)];
    let mut a = arc((0.0, 15.0), 10.0, 0.0, 90.0, 12);
    a.remove(0);
    if let Some(l) = a.last_mut() {
        l.0 = 0.0;
    }
    dome.extend(a);
    v.push(revolve("kuppel", &dome, 48));
    // Torus (Ring)
    let mut tor = arc((20.0, 0.0), 5.0, 0.0, 360.0, 32);
    tor.pop();
    v.push(revolve("torus", &tor, 64));
    let _ = PI;
    v
}

/// Absichtlich defektes Netz für die Reparatur (Löcher, falsch orientierte, doppelte und
/// entartete Dreiecke, Spalt, loses Kleinteil, innenliegender Körper).
pub fn broken() -> RawMesh {
    let mut m = revolve("defekt", &{
        let mut prof = vec![(3.0, 0.0), (12.0, 0.0), (12.0, 10.0)];
        prof.extend(arc((10.0, 10.0), 2.0, 0.0, 90.0, 6).into_iter().skip(1));
        prof.extend([(8.0, 12.0), (8.0, 25.0), (6.0, 27.0), (3.0, 27.0)]);
        prof
    }, 72);
    // Dreieckssuppe wie in STL
    let mut v = Vec::new();
    let mut t = Vec::new();
    for x in &m.tris {
        let b = v.len() as u32;
        for &i in x {
            v.push(m.verts[i as usize]);
        }
        t.push([b, b + 1, b + 2]);
    }
    let n = t.len();
    // Löcher
    let mut keep = vec![true; n];
    for s in [n / 7, n / 3, n / 2 + 5] {
        for k in 0..6 {
            keep[(s + k) % n] = false;
        }
    }
    let mut k = 0;
    t.retain(|_| {
        k += 1;
        keep[k - 1]
    });
    // falsch orientiert, doppelt, entartet
    for i in (0..t.len()).step_by(29) {
        t[i].swap(1, 2);
    }
    for i in (5..t.len()).step_by(97) {
        let d = t[i];
        t.push(d);
    }
    let d = t[0];
    t.push([d[0], d[0], d[1]]);
    // Spalt: Ecken einiger Dreiecke minimal verschieben
    for i in (3..t.len()).step_by(41) {
        let vi = t[i][1] as usize;
        v[vi].x += 0.0002;
    }
    // loses Kleinteil und innenliegender Würfel
    let mut add_box = |o: crate::math::V3, s: f64| {
        let c: Vec<crate::math::V3> = (0..8).map(|i| o + v3((i & 1) as f64 * s, ((i >> 1) & 1) as f64 * s, ((i >> 2) & 1) as f64 * s)).collect();
        let f: [[usize; 3]; 12] = [[0, 2, 1], [1, 2, 3], [4, 5, 6], [5, 7, 6], [0, 1, 4], [1, 5, 4], [2, 6, 3], [3, 6, 7], [0, 4, 2], [2, 4, 6], [1, 3, 5], [3, 7, 5]];
        for x in f {
            let b = v.len() as u32;
            for i in x {
                v.push(c[i]);
            }
            t.push([b, b + 1, b + 2]);
        }
    };
    add_box(v3(30.0, 0.0, 0.0), 0.3);
    add_box(v3(-7.0, -1.0, 14.0), 1.5);
    m.verts = v;
    m.tris = t;
    m
}

/// Bemalter Prüfkörper als 3MF (Bambu-Format): Kugel mit farbigem Band und
/// Kappe, einige Dreiecke fein unterteilt bemalt; dazu ein Quader.
pub fn painted_3mf() -> Vec<u8> {
    use std::fmt::Write as _;
    let n = 24;
    let prof: Vec<(f64, f64)> = (0..=n)
        .map(|i| {
            let a = std::f64::consts::PI * i as f64 / n as f64;
            (20.0 * a.sin(), -20.0 * a.cos() + 22.0)
        })
        .collect();
    let s = revolve("Kugel", &prof, 48);
    // Farbe je Dreieck nach Höhe des Schwerpunkts
    let mut model = String::new();
    model.push_str("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<model unit=\"millimeter\" xmlns=\"http://schemas.microsoft.com/3dmanufacturing/core/2015/02\">\n");
    model.push_str(" <metadata name=\"Application\">Mesh2STEP-Test</metadata>\n <resources>\n  <object id=\"1\" type=\"model\">\n   <mesh>\n    <vertices>\n");
    for v in &s.verts {
        let _ = writeln!(model, "     <vertex x=\"{}\" y=\"{}\" z=\"{}\"/>", v.x + 60.0, v.y + 60.0, v.z);
    }
    model.push_str("    </vertices>\n    <triangles>\n");
    for (i, t) in s.tris.iter().enumerate() {
        let z = (s.verts[t[0] as usize].z + s.verts[t[1] as usize].z + s.verts[t[2] as usize].z) / 3.0;
        let paint = if z > 36.0 {
            "8".to_string() // Kappe: Filament 2
        } else if (18.0..24.0).contains(&z) {
            "0C".to_string() // Band: Filament 3
        } else if (24.0..26.0).contains(&z) && i % 2 == 0 {
            // fein unterteilt: 4 Kinder, abwechselnd Filament 3 und Standard
            "0C00C03".to_string()
        } else {
            String::new()
        };
        if paint.is_empty() {
            let _ = writeln!(model, "     <triangle v1=\"{}\" v2=\"{}\" v3=\"{}\"/>", t[0], t[1], t[2]);
        } else {
            let _ = writeln!(model, "     <triangle v1=\"{}\" v2=\"{}\" v3=\"{}\" paint_color=\"{paint}\"/>", t[0], t[1], t[2]);
        }
    }
    model.push_str("    </triangles>\n   </mesh>\n  </object>\n </resources>\n <build>\n  <item objectid=\"1\"/>\n </build>\n</model>\n");
    let mut z = crate::zip::ZipWriter::default();
    z.add("_rels/.rels", b"<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<Relationships xmlns=\"http://schemas.openxmlformats.org/package/2006/relationships\"><Relationship Target=\"/3D/3dmodel.model\" Id=\"rel-1\" Type=\"http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel\"/></Relationships>");
    z.add("3D/3dmodel.model", model.as_bytes());
    z.add(
        "Metadata/project_settings.config",
        br##"{ "filament_colour": ["#F2F2F2", "#E03030", "#2050D0", "#20A040"] }"##,
    );
    z.finish()
}
