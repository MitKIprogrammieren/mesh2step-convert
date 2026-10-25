//! General reconstruction of imported mesh bodies with the Mesh2STEP converter
//! (`vendor/mesh2step-core`): planes, cylinders, cones, spheres, tori and B-spline patches,
//! any shape. The converter writes STEP text, which the kernel reads back as it reads any
//! STEP file, so the result is an ordinary B-rep body.

use crate::{Body, KernelError, Result};

/// Most triangles the general conversion takes (it segments the whole mesh in memory).
pub const MAX_GENERAL_TRIANGLES: usize = 2_000_000;

fn invalid(message: &str) -> KernelError {
    KernelError::Invalid(format!("mesh conversion (Mesh2STEP): {message}"))
}

/// What a general conversion made, besides the body.
#[derive(Clone, Debug, Default)]
pub struct GeneralConversion {
    /// Faces by kind (`plane`, `cylinder`, `cone`, `sphere`, `torus`, `bspline`, `facet`…).
    pub faces_by_kind: Vec<(String, usize)>,
    /// The tolerance used (mm): the one given, or the one the converter chose.
    pub tolerance: f64,
    /// Largest deviation of the result from the mesh (mm).
    pub max_deviation: f64,
    /// Converter and STEP reader warnings.
    pub warnings: Vec<String>,
}

/// Reconstruct the mesh body `body` with analytic and freeform faces. `tolerance` in mm;
/// `None`, zero or negative lets the converter choose it from the mesh. The source mesh is not
/// modified. A mesh of several separate shells becomes one body of several pieces.
pub fn convert_general_mesh(body: &Body, tolerance: Option<f64>) -> Result<(Body, GeneralConversion)> {
    #[cfg(target_arch = "wasm32")]
    {
        let _ = (body, tolerance);
        Err(invalid("not available in the web app; use the desktop app or the prismatic method"))
    }
    #[cfg(not(target_arch = "wasm32"))]
    {
        crate::guard("convert mesh (Mesh2STEP)", || convert_native(body, tolerance))
    }
}

#[cfg(not(target_arch = "wasm32"))]
fn convert_native(body: &Body, tolerance: Option<f64>) -> Result<(Body, GeneralConversion)> {
    let input = body.mesh.as_ref().ok_or_else(|| invalid("select a mesh body"))?;
    if input.triangles.len() > MAX_GENERAL_TRIANGLES {
        return Err(invalid(&format!(
            "the mesh has {} triangles, more than the {MAX_GENERAL_TRIANGLES} conversion can take; simplify it first",
            input.triangles.len()
        )));
    }
    let tol = match tolerance {
        Some(t) if t.is_finite() && t > 0.0 => {
            if t > 5.0 {
                return Err(invalid("tolerance must be at most 5 mm (0 = automatic)"));
            }
            Some(t)
        }
        Some(t) if !t.is_finite() => return Err(invalid("tolerance is not a number")),
        _ => None,
    };
    let raw = mesh2step_core::io::RawMesh {
        name: "Mesh".into(),
        verts: input.positions.iter().map(|p| mesh2step_core::math::v3(p.x, p.y, p.z)).collect(),
        tris: input.triangles.clone(),
        ..Default::default()
    };
    let prm = mesh2step_core::Params { mode: mesh2step_core::Mode::Full, tol, ..Default::default() };
    // The converter reports trouble with panics in places; turn those into an error.
    let converted = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        mesh2step_core::convert_meshes(std::slice::from_ref(&raw), "Mesh", "mesh.step", &prm)
    }));
    let (step, report) = match converted {
        Ok(Ok(done)) => done,
        Ok(Err(e)) => return Err(invalid(&e)),
        Err(_) => return Err(invalid("the converter stopped on this mesh")),
    };
    let imported = crate::step_import(&step)?;
    let bodies: Vec<Body> = imported.bodies.iter().map(|b| b.body.clone()).collect();
    let solid = match bodies.as_slice() {
        [] => return Err(invalid("the converter made no solid")),
        [one] => one.clone(),
        _ => match crate::join_apart(&bodies)? {
            Some(joined) => joined,
            None => return Err(invalid("the mesh has several shells that could not be joined into one body")),
        },
    };
    let mut warnings = report.warnings.clone();
    warnings.extend(report.check_errors.iter().cloned());
    warnings.extend(imported.warnings.iter().cloned());
    let info = GeneralConversion {
        faces_by_kind: report.faces_by_kind.iter().map(|(k, n)| (k.clone(), *n)).collect(),
        tolerance: report.tolerance,
        max_deviation: report.max_deviation,
        warnings,
    };
    Ok((solid, info))
}
