//! Parametric reconstruction of imported mechanical mesh bodies.

use serde_json::Value;
use solidcraft_doc::FeatureKind;
use solidcraft_doc::expr::Kind;

use super::{
    CommandSpec,
    features::{add_feature, check_expr},
};
use crate::{
    Result, Sel, Session,
    params::{bad, expr, str_},
};

pub static COMMANDS: &[CommandSpec] = &[
    CommandSpec::new("mesh.convert", "Convert Mesh", convert)
        .icon("import")
        .params("body?: mesh body name (default: the selected mesh body); method?: \"auto\" (prismatic when the mesh is a straight extrusion, else mesh2step; the timeline item is named after the method taken), \"prismatic\" (default: a closed constant-section mechanical mesh as editable planar and cylindrical faces) or \"mesh2step\" (any shape: planes, cylinders, cones, spheres, tori and B-spline patches; desktop only); tolerance?: length expression (default 0.05 mm for prismatic, 0 mm = automatic for mesh2step): reconstruct the mesh body as a B-rep, with a parametric timeline feature in the mesh body's owning component (the active component must be that component or an ancestor; otherwise activate the named component first); the active component is unchanged; name?"),
];

/// The mesh body to convert: `body`, else the one selected body.
fn target(s: &Session, p: &Value, cmd: &str) -> Result<String> {
    if let Some(name) = str_(p, "body").filter(|b| !b.trim().is_empty()) {
        return Ok(name.to_owned());
    }
    let selected: Vec<&str> = s.selection.iter().filter_map(|x| if let Sel::Body { name } = x { Some(name.as_str()) } else { None }).collect();
    match selected.as_slice() {
        [name] => Ok((*name).to_owned()),
        [] => Err(bad(cmd, "select a mesh body")),
        _ => Err(bad(cmd, "select one mesh body")),
    }
}

fn convert(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "mesh.convert";
    let state = s.model.state();
    let body = target(s, p, cmd)?;
    let source = state.body(&body).ok_or_else(|| bad(cmd, format!("no body `{body}`")))?;
    if !source.body.is_mesh() {
        return Err(bad(cmd, "select a mesh body"));
    }
    let owner = s.doc.model_body_component(source);
    if !s.doc.component_within(owner, s.active_component) {
        let name = s.doc.components.iter().find(|c| c.id == owner).map(|c| c.name.as_str()).unwrap_or(s.doc.name.as_str());
        return Err(bad(cmd, format!("activate component '{name}' first")));
    }
    let method = match str_(p, "method").map(str::trim).unwrap_or("") {
        "" | "prismatic" => String::new(),
        "mesh2step" => "mesh2step".to_owned(),
        "auto" => "auto".to_owned(),
        other => return Err(bad(cmd, format!("unknown method `{other}` (auto, prismatic or mesh2step)"))),
    };
    let default_tolerance = if method.is_empty() { "0.05 mm" } else { "0 mm" };
    let tolerance = expr(p, "tolerance").unwrap_or_else(|| default_tolerance.into());
    check_expr(s, &tolerance, Kind::Length, cmd, "tolerance")?;
    // Automatic: the timeline item says which method the shape took (unless named).
    let mut named = p.clone();
    if method == "auto" && str_(p, "name").is_none_or(|n| n.trim().is_empty()) {
        let t = s.doc.eval(&tolerance, Kind::Length).unwrap_or(0.0);
        let t = if t > 0.0 { t.min(1.0) } else { 0.05 };
        let label = if solidcraft_kernel::convert_prismatic_mesh(&source.body, t).is_ok() { "prismatic" } else { "Mesh2STEP" };
        if let Some(map) = named.as_object_mut() {
            map.insert("name".into(), Value::String(format!("Convert Mesh ({label})")));
        }
    }
    let p = &named;
    // Conversion edits the component definition, independent of its occurrence placement.
    let active_component = std::mem::replace(&mut s.active_component, owner);
    let active_occurrence_path = s.active_occurrence_path.take();
    let result = add_feature(s, p, FeatureKind::MeshConvert { body: body.into(), tolerance, method });
    s.active_component = active_component;
    s.active_occurrence_path = active_occurrence_path;
    result
}
