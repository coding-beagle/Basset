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
    #[error(
        "radius or distance must not be zero (a chamfer distance must be positive; a negative fillet radius inverts the round)"
    )]
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
        "the angle between the faces of edge {0:?} changes along it (a curved face cut askew); this kernel cannot blend such an edge yet"
    )]
    VaryingDihedral(EdgeKey),
    #[error(
        "edges {convex:?} (convex) and {concave:?} (concave) meet at a vertex, where the round would have to roll onto the bead; this kernel does not build that corner — blend them in separate features"
    )]
    ConvexMeetsConcave { convex: EdgeKey, concave: EdgeKey },
    #[error(
        "blend would need {needed} facets, over the budget of {budget}: coarsen the body's tessellation, or blend fewer edges at once"
    )]
    BlendTooDense { needed: usize, budget: usize },
    #[error(
        "blend of {size:.3} mm runs past the material it has to work with: this edge has room for {limit:.3} mm"
    )]
    BlendTooLarge { size: f64, limit: f64 },
    #[error("face {0:?} is not cylindrical: a thread goes on a shaft or into a hole")]
    NotCylindricalFace(FaceKey),
    #[error("thread pitch must be positive")]
    NonPositivePitch,
    #[error(
        "a {pitch:.3} mm pitch cuts {depth:.3} mm deep, which leaves nothing of a {radius:.3} mm shaft: use a finer pitch"
    )]
    ThreadTooDeep { pitch: f64, depth: f64, radius: f64 },
    #[error(
        "thread length must be positive and fit on the face: asked for {length:.3} mm, the face is {available:.3} mm long"
    )]
    ThreadLength { length: f64, available: f64 },
    #[error(
        "thread would need {needed} facets, over the budget of {budget}: use a coarser pitch or a shorter length"
    )]
    ThreadTooDense { needed: usize, budget: usize },
    #[error("the target face is parallel to the extrude direction")]
    TargetFaceParallel,
    #[error(
        "the profile straddles the target face's plane, so the extrusion would thin to nothing"
    )]
    TargetFaceBehind,
    #[error("the extrusion never reaches the target face")]
    TargetFaceMissed,
    #[error("operation produced no solid")]
    EmptyResult,
    #[error("profile could not be triangulated near {0:?}")]
    Triangulation(Vec3),
    #[error("solid is not closed: {0}")]
    NotClosed(String),
}
