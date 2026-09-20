//! Parametric inputs of every timeline operation.
//!
//! A feature stores only what the user chose, never derived geometry, so that changing
//! any input and replaying the timeline is the single source of truth. Adding a tool to
//! Basset means adding a variant here and an evaluator in `regen`; nothing else.

use basset_math::Affine3;
use basset_sketch::Sketch;
use serde::{Deserialize, Serialize};

use crate::ids::{ComponentId, FeatureId};
use crate::refs::{AxisRef, BodyRef, EdgeRef, PathRef, PlaneRef, RegionRef};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Feature {
    pub id: FeatureId,
    pub name: String,
    /// Suppressed features are skipped during replay but keep their place and inputs.
    pub suppressed: bool,
    pub kind: FeatureKind,
}

/// How far a profile is pushed. Distances are along the profile plane's normal.
#[derive(Copy, Clone, PartialEq, Debug, Serialize, Deserialize)]
pub enum Extent {
    OneSide(f64),
    Symmetric(f64),
    TwoSides { positive: f64, negative: f64 },
}

/// What a body-creating feature does with the solid it produces.
#[derive(Copy, Clone, PartialEq, Debug, Serialize, Deserialize)]
pub enum BodyOp {
    NewBody,
    Join(BodyRef),
    Cut(BodyRef),
    Intersect(BodyRef),
}

impl BodyOp {
    pub fn target(&self) -> Option<BodyRef> {
        match self {
            BodyOp::NewBody => None,
            BodyOp::Join(b) | BodyOp::Cut(b) | BodyOp::Intersect(b) => Some(*b),
        }
    }
}

#[derive(Copy, Clone, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub enum CombineOp {
    Join,
    Cut,
    Intersect,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum FeatureKind {
    NewComponent {
        name: String,
        parent: ComponentId,
    },
    Sketch {
        plane: PlaneRef,
        component: ComponentId,
        sketch: Sketch,
    },
    OffsetPlane {
        base: PlaneRef,
        distance: f64,
    },
    AngledPlane {
        base: PlaneRef,
        axis: AxisRef,
        angle: f64,
    },
    Extrude {
        regions: Vec<RegionRef>,
        extent: Extent,
        operation: BodyOp,
        component: ComponentId,
    },
    Revolve {
        regions: Vec<RegionRef>,
        axis: AxisRef,
        angle: f64,
        operation: BodyOp,
        component: ComponentId,
    },
    Sweep {
        regions: Vec<RegionRef>,
        path: PathRef,
        operation: BodyOp,
        component: ComponentId,
    },
    Loft {
        regions: Vec<RegionRef>,
        operation: BodyOp,
        component: ComponentId,
    },
    Fillet {
        edges: Vec<EdgeRef>,
        radius: f64,
    },
    Chamfer {
        edges: Vec<EdgeRef>,
        distance: f64,
    },
    Combine {
        target: BodyRef,
        tools: Vec<BodyRef>,
        operation: CombineOp,
        keep_tools: bool,
    },
    Move {
        body: BodyRef,
        transform: Affine3,
    },
}

impl FeatureKind {
    /// Human-readable default name used when the user does not provide one.
    pub fn default_name(&self) -> &'static str {
        match self {
            FeatureKind::NewComponent { .. } => "Component",
            FeatureKind::Sketch { .. } => "Sketch",
            FeatureKind::OffsetPlane { .. } => "Offset Plane",
            FeatureKind::AngledPlane { .. } => "Angled Plane",
            FeatureKind::Extrude { .. } => "Extrude",
            FeatureKind::Revolve { .. } => "Revolve",
            FeatureKind::Sweep { .. } => "Sweep",
            FeatureKind::Loft { .. } => "Loft",
            FeatureKind::Fillet { .. } => "Fillet",
            FeatureKind::Chamfer { .. } => "Chamfer",
            FeatureKind::Combine { .. } => "Combine",
            FeatureKind::Move { .. } => "Move",
        }
    }

    /// Whether this feature creates a body named after itself (as opposed to editing one).
    pub fn creates_body(&self) -> bool {
        match self {
            FeatureKind::Extrude { operation, .. }
            | FeatureKind::Revolve { operation, .. }
            | FeatureKind::Sweep { operation, .. }
            | FeatureKind::Loft { operation, .. } => matches!(operation, BodyOp::NewBody),
            _ => false,
        }
    }

    /// Every feature id this feature depends on. Used to flag dependants when a feature
    /// fails or is deleted, and to keep deletion honest about what else will break.
    pub fn dependencies(&self) -> Vec<FeatureId> {
        let mut out = Vec::new();
        let mut plane = |p: &PlaneRef| match p {
            PlaneRef::Origin(_) => {}
            PlaneRef::Feature(f) => out.push(*f),
            PlaneRef::Face(face) => out.push(face.body.0),
        };
        match self {
            FeatureKind::NewComponent { parent, .. } => {
                if *parent != ComponentId::ROOT {
                    out.push(FeatureId(parent.0));
                }
            }
            FeatureKind::Sketch { plane: p, .. } => plane(p),
            FeatureKind::OffsetPlane { base, .. } => plane(base),
            FeatureKind::AngledPlane { base, axis, .. } => {
                plane(base);
                if let AxisRef::SketchLine { sketch, .. } = axis {
                    out.push(*sketch);
                }
            }
            FeatureKind::Extrude {
                regions, operation, ..
            }
            | FeatureKind::Loft {
                regions, operation, ..
            } => {
                out.extend(regions.iter().map(RegionRef::source));
                out.extend(operation.target().map(|b| b.0));
            }
            FeatureKind::Revolve {
                regions,
                axis,
                operation,
                ..
            } => {
                out.extend(regions.iter().map(RegionRef::source));
                if let AxisRef::SketchLine { sketch, .. } = axis {
                    out.push(*sketch);
                }
                out.extend(operation.target().map(|b| b.0));
            }
            FeatureKind::Sweep {
                regions,
                path,
                operation,
                ..
            } => {
                out.extend(regions.iter().map(RegionRef::source));
                out.push(path.sketch);
                out.extend(operation.target().map(|b| b.0));
            }
            FeatureKind::Fillet { edges, .. } | FeatureKind::Chamfer { edges, .. } => {
                out.extend(edges.iter().map(|e| e.body.0));
            }
            FeatureKind::Combine { target, tools, .. } => {
                out.push(target.0);
                out.extend(tools.iter().map(|b| b.0));
            }
            FeatureKind::Move { body, .. } => out.push(body.0),
        }
        out.sort_unstable();
        out.dedup();
        out
    }
}
