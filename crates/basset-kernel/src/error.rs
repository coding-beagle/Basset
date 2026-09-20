//! Errors for operations driven by user data. Anything here is a legitimate modelling
//! failure that the timeline reports on the feature; kernel bugs panic instead.

use basset_math::Vec3;
use thiserror::Error;

use crate::ids::{EdgeKey, FaceKey};

#[derive(Debug, Clone, PartialEq, Error)]
pub enum KernelError {
    #[error("profile has no area")]
    EmptyProfile,
    #[error("profile contour needs at least three points")]
    DegenerateContour,
    #[error("extent must be non-zero")]
    ZeroExtent,
    #[error("revolve angle must be non-zero")]
    ZeroAngle,
    #[error("revolve axis does not lie in the profile plane")]
    AxisNotInProfilePlane,
    #[error("profile crosses the revolve axis")]
    ProfileCrossesAxis,
    #[error("path needs at least two distinct points")]
    DegeneratePath,
    #[error("loft needs at least two sections")]
    TooFewSections,
    #[error("loft sections must all have the same number of holes")]
    MismatchedHoles,
    #[error("radius or distance must be positive")]
    NonPositiveBlend,
    #[error("edge {0:?} does not exist on this body")]
    MissingEdge(EdgeKey),
    #[error("face {0:?} does not exist on this body")]
    MissingFace(FaceKey),
    #[error("face {0:?} is not planar")]
    NotPlanarFace(FaceKey),
    #[error("edge {0:?} lies between tangent faces and cannot be blended")]
    TangentEdge(EdgeKey),
    #[error(
        "blend would need {needed} facets, over the budget of {budget}: coarsen the body's tessellation, or blend fewer edges at once"
    )]
    BlendTooDense { needed: usize, budget: usize },
    #[error("operation produced no solid")]
    EmptyResult,
    #[error("profile could not be triangulated near {0:?}")]
    Triangulation(Vec3),
    #[error("solid is not closed: {0}")]
    NotClosed(String),
}
