//! STEP-Ausgabe (ISO 10303-21, AP214 AUTOMOTIVE_DESIGN).

use crate::brep::{Brep, Curve};
use crate::math::V3;
use crate::surf::Surf;
use std::fmt::Write as _;

struct W {
    out: String,
    id: usize,
}

impl W {
    fn add(&mut self, s: impl AsRef<str>) -> usize {
        self.id += 1;
        let _ = writeln!(self.out, "#{}={};", self.id, s.as_ref());
        self.id
    }
    fn pt(&mut self, p: V3) -> usize {
        self.add(format!("CARTESIAN_POINT('',({},{},{}))", r(p.x), r(p.y), r(p.z)))
    }
    fn dir(&mut self, d: V3) -> usize {
        let d = d.normalized();
        self.add(format!("DIRECTION('',({},{},{}))", r(d.x), r(d.y), r(d.z)))
    }
    fn ax2(&mut self, o: V3, z: V3, x: V3) -> usize {
        let z = z.normalized();
        let mut x = x - z * x.dot(z);
        if x.norm() < 1e-9 {
            x = z.any_perp();
        }
        let p = self.pt(o);
        let dz = self.dir(z);
        let dx = self.dir(x.normalized());
        self.add(format!("AXIS2_PLACEMENT_3D('',#{p},#{dz},#{dx})"))
    }
}

/// STEP-konforme Gleitkommazahl (immer mit Dezimalpunkt).
fn r(v: f64) -> String {
    if !v.is_finite() || v.abs() < 1e-13 {
        return "0.".into();
    }
    if v.abs() >= 1e-4 && v.abs() < 1e12 {
        let mut s = format!("{:.12}", v);
        while s.ends_with('0') {
            s.pop();
        }
        if s == "-0." {
            return "0.".into();
        }
        s
    } else {
        let s = format!("{:.12E}", v);
        // Rust: 1.234000000000E-5 -> Mantisse kürzen
        let (m, e) = s.split_once('E').unwrap();
        let mut m = m.to_string();
        while m.ends_with('0') {
            m.pop();
        }
        if !m.contains('.') {
            m.push('.');
        }
        format!("{m}E{e}")
    }
}

fn esc(s: &str) -> String {
    s.chars()
        .map(|c| if c == '\'' { "''".to_string() } else if c.is_ascii() && !c.is_ascii_control() { c.to_string() } else { "_".to_string() })
        .collect()
}

/// Unschärfe (confusion distance), die für diese B-Rep in die STEP-Datei geschrieben wird.
pub fn uncertainty(b: &Brep) -> f64 {
    b.tol.clamp(1e-4, 0.01)
}

pub fn write_step(breps: &[Brep], name: &str, file_name: &str) -> String {
    let mut w = W { out: String::new(), id: 0 };
    // Unschärfe (confusion distance): klein halten, CAD-Kernel passen Kantentoleranzen selbst an
    let tol = breps.iter().map(uncertainty).fold(0.0f64, f64::max);
    let all_closed = breps.iter().all(|b| b.shells.iter().all(|s| s.closed));

    let ctx_app = w.add("APPLICATION_CONTEXT('core data for automotive mechanical design processes')");
    w.add(format!("APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2000,#{ctx_app})"));
    let pctx = w.add(format!("PRODUCT_CONTEXT('',#{ctx_app},'mechanical')"));
    let pdctx = w.add(format!("PRODUCT_DEFINITION_CONTEXT('part definition',#{ctx_app},'design')"));
    let prod = w.add(format!("PRODUCT('{0}','{0}','',(#{pctx}))", esc(name)));
    w.add(format!("PRODUCT_RELATED_PRODUCT_CATEGORY('part',$,(#{prod}))"));
    let pdf = w.add(format!("PRODUCT_DEFINITION_FORMATION('','',#{prod})"));
    let pd = w.add(format!("PRODUCT_DEFINITION('design','',#{pdf},#{pdctx})"));
    let pds = w.add(format!("PRODUCT_DEFINITION_SHAPE('','',#{pd})"));

    let mm = w.add("(LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.))");
    let rad = w.add("(NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.))");
    let sr = w.add("(NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT())");
    let unc = w.add(format!(
        "UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE({}),#{mm},'distance_accuracy_value','confusion accuracy')",
        r(tol)
    ));
    let ctx = w.add(format!(
        "(GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#{unc})) GLOBAL_UNIT_ASSIGNED_CONTEXT((#{mm},#{rad},#{sr})) REPRESENTATION_CONTEXT('Context3D','3D Context'))"
    ));

    let origin = w.ax2(V3::ZERO, V3::Z, V3::X);
    let mut items: Vec<usize> = vec![origin];

    for b in breps {
        // Ecken
        let vids: Vec<usize> = b
            .verts
            .iter()
            .map(|&p| {
                let pt = w.pt(p);
                w.add(format!("VERTEX_POINT('',#{pt})"))
            })
            .collect();
        // Kanten
        let mut eids = Vec::with_capacity(b.edges.len());
        for e in &b.edges {
            let cid = match &e.curve {
                Curve::Line { p, d } => {
                    let pt = w.pt(*p);
                    let dd = w.dir(*d);
                    let vec = w.add(format!("VECTOR('',#{dd},1.)"));
                    w.add(format!("LINE('',#{pt},#{vec})"))
                }
                Curve::Circle { c, n, x, r: rr } => {
                    let ax = w.ax2(*c, *n, *x);
                    w.add(format!("CIRCLE('',#{ax},{})", r(*rr)))
                }
                Curve::Ellipse { c, n, x, a, b: bb } => {
                    let ax = w.ax2(*c, *n, *x);
                    w.add(format!("ELLIPSE('',#{ax},{},{})", r(*a), r(*bb)))
                }
                Curve::Poly(ps) => {
                    let ids: Vec<String> = ps.iter().map(|&p| format!("#{}", w.pt(p))).collect();
                    let n = ps.len();
                    let mut mult = vec!["1".to_string(); n];
                    mult[0] = "2".into();
                    mult[n - 1] = "2".into();
                    let knots: Vec<String> = (0..n).map(|i| r(i as f64)).collect();
                    w.add(format!(
                        "B_SPLINE_CURVE_WITH_KNOTS('',1,({}),.POLYLINE_FORM.,.F.,.F.,({}),({}),.UNSPECIFIED.)",
                        ids.join(","),
                        mult.join(","),
                        knots.join(",")
                    ))
                }
            };
            eids.push(w.add(format!("EDGE_CURVE('',#{},#{},#{cid},.T.)", vids[e.v0 as usize], vids[e.v1 as usize])));
        }
        // Flächen
        let mut fids = Vec::with_capacity(b.faces.len());
        for f in &b.faces {
            let sid = match f.surf {
                Surf::Plane { o, n } => {
                    let ax = w.ax2(o, n, n.any_perp());
                    w.add(format!("PLANE('',#{ax})"))
                }
                Surf::Cyl { p, a, r: rr } => {
                    let ax = w.ax2(p, a, a.any_perp());
                    w.add(format!("CYLINDRICAL_SURFACE('',#{ax},{})", r(rr)))
                }
                Surf::Cone { p, a, r: rr, ang } => {
                    // Radius darf nicht negativ sein: Bezugspunkt ggf. verschieben
                    let (p, rr) = if rr < 0.0 { (p + a * (-rr / ang.tan()), 0.0) } else { (p, rr) };
                    let ax = w.ax2(p, a, a.any_perp());
                    w.add(format!("CONICAL_SURFACE('',#{ax},{},{})", r(rr), r(ang)))
                }
                Surf::Sphere { c, r: rr } => {
                    let ax = w.ax2(c, V3::Z, V3::X);
                    w.add(format!("SPHERICAL_SURFACE('',#{ax},{})", r(rr)))
                }
                Surf::Torus { c, a, rmaj, rmin } => {
                    let ax = w.ax2(c, a, a.any_perp());
                    w.add(format!("TOROIDAL_SURFACE('',#{ax},{},{})", r(rmaj), r(rmin)))
                }
                Surf::BSpline { id } => {
                    let bs = crate::bspline::get(id);
                    let cps = bs.control_points();
                    let mut rows = Vec::with_capacity(bs.nu);
                    for i in 0..bs.nu {
                        let ids: Vec<String> = (0..bs.nv).map(|j| format!("#{}", w.pt(cps[i * bs.nv + j]))).collect();
                        rows.push(format!("({})", ids.join(",")));
                    }
                    let km = |k: &[f64]| -> (String, String) {
                        let mut vals: Vec<f64> = Vec::new();
                        let mut mult: Vec<usize> = Vec::new();
                        for &x in k {
                            if vals.last().map_or(false, |&l| (l - x).abs() < 1e-12) {
                                *mult.last_mut().unwrap() += 1;
                            } else {
                                vals.push(x);
                                mult.push(1);
                            }
                        }
                        (
                            mult.iter().map(|m| m.to_string()).collect::<Vec<_>>().join(","),
                            vals.iter().map(|&v| r(v)).collect::<Vec<_>>().join(","),
                        )
                    };
                    let (mu, ku) = km(&bs.ku);
                    let (mv, kv) = km(&bs.kv);
                    w.add(format!(
                        "B_SPLINE_SURFACE_WITH_KNOTS('',3,3,({}),.UNSPECIFIED.,.F.,.F.,.F.,({mu}),({mv}),({ku}),({kv}),.UNSPECIFIED.)",
                        rows.join(",")
                    ))
                }
            };
            let mut bounds = Vec::new();
            for (li, lp) in f.loops.iter().enumerate() {
                let oes: Vec<String> = lp
                    .iter()
                    .map(|&(e, fwd)| format!("#{}", w.add(format!("ORIENTED_EDGE('',*,*,#{},{})", eids[e as usize], if fwd { ".T." } else { ".F." }))))
                    .collect();
                let el = w.add(format!("EDGE_LOOP('',({}))", oes.join(",")));
                let kind = if f.outer == Some(li) { "FACE_OUTER_BOUND" } else { "FACE_BOUND" };
                bounds.push(format!("#{}", w.add(format!("{kind}('',#{el},.T.)"))));
            }
            fids.push(w.add(format!(
                "ADVANCED_FACE('',({}),#{sid},{})",
                bounds.join(","),
                if f.same_sense { ".T." } else { ".F." }
            )));
        }
        // Schalen
        let shell_ids: Vec<usize> = b
            .shells
            .iter()
            .map(|s| {
                let fl: Vec<String> = s.faces.iter().map(|&f| format!("#{}", fids[f as usize])).collect();
                let kw = if s.closed { "CLOSED_SHELL" } else { "OPEN_SHELL" };
                w.add(format!("{kw}('',({}))", fl.join(",")))
            })
            .collect();
        if all_closed {
            for s in &b.solids {
                let outer = shell_ids[s.outer as usize];
                if s.voids.is_empty() {
                    items.push(w.add(format!("MANIFOLD_SOLID_BREP('{}',#{outer})", esc(&s.name))));
                } else {
                    let vs: Vec<String> = s
                        .voids
                        .iter()
                        .map(|&v| format!("#{}", w.add(format!("ORIENTED_CLOSED_SHELL('',*,#{},.F.)", shell_ids[v as usize]))))
                        .collect();
                    items.push(w.add(format!("BREP_WITH_VOIDS('{}',#{outer},({}))", esc(&s.name), vs.join(","))));
                }
            }
        } else {
            let sl: Vec<String> = shell_ids.iter().map(|s| format!("#{s}")).collect();
            items.push(w.add(format!("SHELL_BASED_SURFACE_MODEL('{}',({}))", esc(name), sl.join(","))));
        }
    }

    let it: Vec<String> = items.iter().map(|i| format!("#{i}")).collect();
    let rep_kind = if all_closed { "ADVANCED_BREP_SHAPE_REPRESENTATION" } else { "MANIFOLD_SURFACE_SHAPE_REPRESENTATION" };
    let rep = w.add(format!("{rep_kind}('{}',({}),#{ctx})", esc(name), it.join(",")));
    w.add(format!("SHAPE_DEFINITION_REPRESENTATION(#{pds},#{rep})"));

    let now = timestamp();
    let mut s = String::with_capacity(w.out.len() + 1024);
    s.push_str("ISO-10303-21;\nHEADER;\n");
    let _ = writeln!(s, "FILE_DESCRIPTION(('mesh2step: aus Dreiecksnetz rekonstruierte Geometrie'),'2;1');");
    let _ = writeln!(s, "FILE_NAME('{}','{}',(''),(''),'mesh2step 1.0','mesh2step','');", esc(file_name), now);
    s.push_str("FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));\nENDSEC;\nDATA;\n");
    s.push_str(&w.out);
    s.push_str("ENDSEC;\nEND-ISO-10303-21;\n");
    s
}

fn timestamp() -> String {
    let secs = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs() as i64)
        .unwrap_or(0);
    let days = secs.div_euclid(86400);
    let rem = secs.rem_euclid(86400);
    // Zivilkalender aus Tagen (Howard Hinnant)
    let z = days + 719468;
    let era = z.div_euclid(146097);
    let doe = z - era * 146097;
    let yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    let y = yoe + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let mo = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = if mo <= 2 { y + 1 } else { y };
    format!("{:04}-{:02}-{:02}T{:02}:{:02}:{:02}", y, mo, d, rem / 3600, (rem / 60) % 60, rem % 60)
}
