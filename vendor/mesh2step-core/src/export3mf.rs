//! Export der aufgeteilten Teile: 3MF im Projektformat von Bambu Studio / OrcaSlicer
//! (Teile mit zugewiesenem Filament, Bemalung bleibt erhalten) sowie binäres STL.

use crate::math::V3;
use crate::paint::encode_leaf;
use crate::split::Part;
use crate::zip::ZipWriter;
use std::fmt::Write as _;

#[derive(Clone, Debug)]
pub struct ExportOpts {
    /// je Ausgangsobjekt ein Objekt mit Teilen (zusammengebaut); sonst jedes Teil einzeln
    pub assembly: bool,
    /// Druckbettgröße für die Anordnung einzelner Teile (mm)
    pub bed: f64,
}

impl Default for ExportOpts {
    fn default() -> Self {
        ExportOpts { assembly: true, bed: 256.0 }
    }
}

fn esc(s: &str) -> String {
    s.replace('&', "&amp;").replace('<', "&lt;").replace('>', "&gt;").replace('"', "&quot;")
}

fn uuid(n: u32, kind: u32) -> String {
    let h = (n as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15) ^ ((kind as u64) << 40);
    format!("{:08x}-{:04x}-4{:03x}-8{:03x}-{:012x}", (h >> 32) as u32, (h >> 16) as u16, n & 0xfff, kind & 0xfff, h & 0xffff_ffff_ffff)
}

fn num(v: f64) -> String {
    let s = format!("{:.6}", v);
    let s = s.trim_end_matches('0').trim_end_matches('.');
    if s == "-0" || s.is_empty() {
        "0".into()
    } else {
        s.to_string()
    }
}

struct Obj<'a> {
    name: String,
    parts: Vec<(&'a Part, V3)>, // Teil und Verschiebung
}

/// Ordnet einzelne Teile nebeneinander auf dem Druckbett an.
fn layout(parts: &[Part], bed: f64) -> Vec<V3> {
    let mut idx: Vec<usize> = (0..parts.len()).collect();
    let bb: Vec<(V3, V3)> = parts.iter().map(|p| p.bbox()).collect();
    idx.sort_by(|&a, &b| {
        let ha = bb[a].1.y - bb[a].0.y;
        let hb = bb[b].1.y - bb[b].0.y;
        hb.partial_cmp(&ha).unwrap_or(std::cmp::Ordering::Equal)
    });
    let gap = 6.0;
    let mut off = vec![V3::ZERO; parts.len()];
    let (mut x, mut y, mut row_h) = (gap, gap, 0.0f64);
    for i in idx {
        let (mn, mx) = bb[i];
        let (w, h) = (mx.x - mn.x, mx.y - mn.y);
        if x + w > bed && x > gap {
            x = gap;
            y += row_h + gap;
            row_h = 0.0;
        }
        off[i] = V3 { x: x - mn.x, y: y - mn.y, z: -mn.z };
        x += w + gap;
        row_h = row_h.max(h);
    }
    off
}

pub fn write_3mf(parts: &[Part], group_names: &[String], o: &ExportOpts) -> Vec<u8> {
    // Objekte zusammenstellen
    let mut objs: Vec<Obj> = Vec::new();
    if o.assembly {
        let ngroups = parts.iter().map(|p| p.group + 1).max().unwrap_or(0);
        for g in 0..ngroups {
            let ps: Vec<&Part> = parts.iter().filter(|p| p.group == g && !p.tris.is_empty()).collect();
            if ps.is_empty() {
                continue;
            }
            // ganzes Objekt auf das Bett stellen
            let minz = ps.iter().flat_map(|p| p.verts.iter()).fold(f64::MAX, |m, v| m.min(v.z));
            let shift = V3 { x: 0.0, y: 0.0, z: -minz };
            objs.push(Obj {
                name: group_names.get(g).cloned().unwrap_or_else(|| format!("Objekt {}", g + 1)),
                parts: ps.into_iter().map(|p| (p, shift)).collect(),
            });
        }
    } else {
        let offs = layout(parts, o.bed);
        for (p, off) in parts.iter().zip(offs) {
            if !p.tris.is_empty() {
                objs.push(Obj { name: p.name.clone(), parts: vec![(p, off)] });
            }
        }
    }
    let mut zip = ZipWriter::default();
    zip.add(
        "[Content_Types].xml",
        br#"<?xml version="1.0" encoding="UTF-8"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
 <Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>
 <Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/>
 <Default Extension="png" ContentType="image/png"/>
 <Default Extension="gcode" ContentType="text/x.gcode"/>
</Types>
"#,
    );
    zip.add(
        "_rels/.rels",
        br#"<?xml version="1.0" encoding="UTF-8"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
 <Relationship Target="/3D/3dmodel.model" Id="rel-1" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>
</Relationships>
"#,
    );
    const HEAD: &str = r#"<?xml version="1.0" encoding="UTF-8"?>
<model unit="millimeter" xml:lang="en-US" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02" xmlns:BambuStudio="http://schemas.bambulab.com/package/2021" xmlns:p="http://schemas.microsoft.com/3dmanufacturing/production/2015/06" requiredextensions="p">
"#;
    let n_parts: u32 = objs.iter().map(|o| o.parts.len() as u32).sum();
    let mut sub_id = 0u32;
    let mut main_model = String::from(HEAD);
    main_model.push_str(" <metadata name=\"Application\">Mesh2STEP-1.4</metadata>\n");
    main_model.push_str(" <metadata name=\"BambuStudio:3mfVersion\">1</metadata>\n");
    main_model.push_str(" <metadata name=\"Title\">Mesh2STEP</metadata>\n");
    main_model.push_str(" <resources>\n");
    let mut build = String::new();
    let mut rels = String::from(
        "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<Relationships xmlns=\"http://schemas.openxmlformats.org/package/2006/relationships\">\n",
    );
    let mut config = String::from("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<config>\n");
    let mut plate = String::from("  <plate>\n    <metadata key=\"plater_id\" value=\"1\"/>\n    <metadata key=\"plater_name\" value=\"\"/>\n    <metadata key=\"locked\" value=\"false\"/>\n");
    for (oi, ob) in objs.iter().enumerate() {
        let main_id = n_parts + 1 + oi as u32;
        let path = format!("3D/Objects/object_{}.model", oi + 1);
        let mut om = String::from(HEAD);
        om.push_str(" <metadata name=\"BambuStudio:3mfVersion\">1</metadata>\n <resources>\n");
        let mut comps = String::new();
        let main_ext = ob.parts.first().map(|(p, _)| p.color).unwrap_or(1).max(1);
        let total_faces: usize = ob.parts.iter().map(|(p, _)| p.tris.len()).sum();
        let _ = writeln!(config, "  <object id=\"{main_id}\">");
        let _ = writeln!(config, "    <metadata key=\"name\" value=\"{}\"/>", esc(&ob.name));
        let _ = writeln!(config, "    <metadata key=\"extruder\" value=\"{main_ext}\"/>");
        let _ = writeln!(config, "    <metadata face_count=\"{total_faces}\"/>");
        for (p, off) in &ob.parts {
            sub_id += 1;
            let ext = p.color.max(1);
            let _ = writeln!(om, "  <object id=\"{sub_id}\" p:UUID=\"{}\" type=\"model\">", uuid(sub_id, 1));
            om.push_str("   <mesh>\n    <vertices>\n");
            for v in &p.verts {
                let _ = writeln!(om, "     <vertex x=\"{}\" y=\"{}\" z=\"{}\"/>", num(v.x + off.x), num(v.y + off.y), num(v.z + off.z));
            }
            om.push_str("    </vertices>\n    <triangles>\n");
            for (t, &c) in p.tris.iter().zip(&p.col) {
                if c != ext && c > 0 {
                    let _ = writeln!(om, "     <triangle v1=\"{}\" v2=\"{}\" v3=\"{}\" paint_color=\"{}\"/>", t[0], t[1], t[2], encode_leaf(c));
                } else {
                    let _ = writeln!(om, "     <triangle v1=\"{}\" v2=\"{}\" v3=\"{}\"/>", t[0], t[1], t[2]);
                }
            }
            om.push_str("    </triangles>\n   </mesh>\n  </object>\n");
            let _ = writeln!(
                comps,
                "    <component p:path=\"/{path}\" objectid=\"{sub_id}\" p:UUID=\"{}\" transform=\"1 0 0 0 1 0 0 0 1 0 0 0\"/>",
                uuid(sub_id, 2)
            );
            let _ = writeln!(config, "    <part id=\"{sub_id}\" subtype=\"normal_part\">");
            let _ = writeln!(config, "      <metadata key=\"name\" value=\"{}\"/>", esc(&p.name));
            let _ = writeln!(config, "      <metadata key=\"matrix\" value=\"1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1\"/>");
            let _ = writeln!(config, "      <metadata key=\"extruder\" value=\"{ext}\"/>");
            let _ = writeln!(
                config,
                "      <mesh_stat face_count=\"{}\" edges_fixed=\"0\" degenerate_facets=\"0\" facets_removed=\"0\" facets_reversed=\"0\" backwards_edges=\"0\"/>",
                p.tris.len()
            );
            config.push_str("    </part>\n");
        }
        om.push_str(" </resources>\n <build/>\n</model>\n");
        zip.add(&path, om.as_bytes());
        let _ = writeln!(rels, " <Relationship Target=\"/{path}\" Id=\"rel-{}\" Type=\"http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel\"/>", oi + 1);
        let _ = writeln!(main_model, "  <object id=\"{main_id}\" name=\"{}\" p:UUID=\"{}\" type=\"model\">\n   <components>", esc(&ob.name), uuid(main_id, 3));
        main_model.push_str(&comps);
        main_model.push_str("   </components>\n  </object>\n");
        let _ = writeln!(
            build,
            "  <item objectid=\"{main_id}\" p:UUID=\"{}\" transform=\"1 0 0 0 1 0 0 0 1 0 0 0\" printable=\"1\"/>",
            uuid(main_id, 4)
        );
        config.push_str("  </object>\n");
        let _ = writeln!(
            plate,
            "    <model_instance>\n      <metadata key=\"object_id\" value=\"{main_id}\"/>\n      <metadata key=\"instance_id\" value=\"0\"/>\n      <metadata key=\"identify_id\" value=\"{}\"/>\n    </model_instance>",
            100 + oi
        );
    }
    main_model.push_str(" </resources>\n");
    let _ = write!(main_model, " <build p:UUID=\"{}\">\n{build} </build>\n</model>\n", uuid(0, 5));
    rels.push_str("</Relationships>\n");
    plate.push_str("  </plate>\n");
    config.push_str(&plate);
    config.push_str("</config>\n");
    zip.add("3D/3dmodel.model", main_model.as_bytes());
    zip.add("3D/_rels/3dmodel.model.rels", rels.as_bytes());
    zip.add("Metadata/model_settings.config", config.as_bytes());
    zip.finish()
}

/// Binäres STL eines Teils.
pub fn write_stl(p: &Part) -> Vec<u8> {
    let mut out = Vec::with_capacity(84 + p.tris.len() * 50);
    let mut head = [0u8; 80];
    let name = format!("Mesh2STEP {}", p.name);
    for (i, b) in name.bytes().take(80).enumerate() {
        head[i] = b;
    }
    out.extend_from_slice(&head);
    out.extend_from_slice(&(p.tris.len() as u32).to_le_bytes());
    for t in &p.tris {
        let (a, b, c) = (p.verts[t[0] as usize], p.verts[t[1] as usize], p.verts[t[2] as usize]);
        let n = (b - a).cross(c - a);
        let n = if n.norm() > 0.0 { n.normalized() } else { n };
        for v in [n, a, b, c] {
            for x in [v.x, v.y, v.z] {
                out.extend_from_slice(&(x as f32).to_le_bytes());
            }
        }
        out.extend_from_slice(&[0, 0]);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::v3;

    fn tetra(name: &str, col: u8, group: usize, dz: f64) -> Part {
        let mut p = Part {
            name: name.into(),
            verts: vec![v3(0., 0., dz), v3(10., 0., dz), v3(0., 10., dz), v3(0., 0., 10. + dz)],
            tris: vec![[0, 2, 1], [0, 1, 3], [1, 2, 3], [0, 3, 2]],
            col: vec![col, col, 3, col],
            color: col,
            group,
            ..Default::default()
        };
        p.update_stats();
        p
    }

    #[test]
    fn roundtrip_3mf() {
        let parts = vec![tetra("A", 1, 0, 0.0), tetra("B", 2, 0, 10.0), tetra("C", 4, 1, 0.0)];
        for assembly in [true, false] {
            let data = write_3mf(&parts, &["Eins".into(), "Zwei".into()], &ExportOpts { assembly, bed: 256.0 });
            let m = crate::io::load_3mf_full(&data, "x").unwrap();
            assert_eq!(m.meshes.len(), if assembly { 2 } else { 3 });
            let all: Vec<&crate::io::RawMesh> = m.meshes.iter().collect();
            // Filamente je Dreieck nach Bemalung
            let mut cols = Vec::new();
            for r in &all {
                let (cm, bad) = crate::paint::refine(r);
                assert_eq!(bad, 0);
                cols.extend(cm.col);
            }
            cols.sort();
            assert_eq!(cols, vec![1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4]);
            if assembly {
                assert_eq!(m.meshes[0].name, "Eins");
            }
        }
    }

    #[test]
    fn stl_roundtrip() {
        let p = tetra("A", 1, 0, 0.0);
        let s = write_stl(&p);
        let r = crate::io::load_stl(&s, "a").unwrap();
        assert_eq!(r.tris.len(), 4);
    }
}
