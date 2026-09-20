//! Per-frame scene description.
//!
//! A [`Scene`] is rebuilt by the application every frame from its document state. It is
//! plain data referencing GPU meshes only through [`MeshHandle`]s, so the application never
//! needs to hold wgpu resources itself and can diff, log or serialise what it asked for.
//!
//! Colours are linear RGBA in `[0, 1]`. The renderer writes them unchanged, so with an sRGB
//! render target the hardware performs the encoding.

use basset_math::{Frame, Mat4, Vec3};

use crate::camera::Camera;

/// Identifies a mesh uploaded with [`crate::Renderer::upload_mesh`]. Handles are never
/// reused, so a stale handle draws nothing rather than someone else's geometry.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct MeshHandle(pub(crate) u64);

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum MeshStyle {
    #[default]
    Shaded,
    /// Shaded faces plus feature edges (face boundaries, creases, open borders).
    ShadedWithEdges,
    /// Translucent, no depth write: used for bodies hidden behind a tool preview or
    /// components being edited in context.
    Ghost,
}

#[derive(Debug, Clone, PartialEq)]
pub struct MeshInstance {
    pub handle: MeshHandle,
    pub transform: Mat4,
    pub color: [f32; 4],
    /// Kernel face ids (as carried in `TriMesh::face_ids`) to tint with `highlight_color`,
    /// used for selection and hover feedback.
    pub highlight_faces: Vec<u32>,
    pub highlight_color: [f32; 4],
    pub style: MeshStyle,
}

impl MeshInstance {
    pub const DEFAULT_COLOR: [f32; 4] = [0.62, 0.66, 0.70, 1.0];
    pub const DEFAULT_HIGHLIGHT: [f32; 4] = [0.20, 0.55, 1.00, 1.0];

    pub fn new(handle: MeshHandle) -> Self {
        Self {
            handle,
            transform: Mat4::IDENTITY,
            color: Self::DEFAULT_COLOR,
            highlight_faces: Vec::new(),
            highlight_color: Self::DEFAULT_HIGHLIGHT,
            style: MeshStyle::Shaded,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct LineBatch {
    pub segments: Vec<[Vec3; 2]>,
    pub color: [f32; 4],
    /// Width on screen, independent of zoom.
    pub width_px: f32,
    /// `false` draws on top of everything, which is how sketch overlays stay visible
    /// through bodies.
    pub depth_test: bool,
    /// Dashed pattern for construction geometry.
    pub dashed: bool,
}

impl LineBatch {
    pub fn new(color: [f32; 4]) -> Self {
        Self {
            segments: Vec::new(),
            color,
            width_px: 1.5,
            depth_test: true,
            dashed: false,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct PointBatch {
    pub points: Vec<Vec3>,
    pub color: [f32; 4],
    /// Side length of the square marker on screen.
    pub size_px: f32,
    pub depth_test: bool,
}

impl PointBatch {
    pub fn new(color: [f32; 4]) -> Self {
        Self {
            points: Vec::new(),
            color,
            size_px: 6.0,
            depth_test: false,
        }
    }
}

/// Flat translucent triangles in world space: how a region is lit up when it is selected
/// or hovered. Plain geometry rather than a mesh handle, because a highlight changes every
/// frame and is not worth an upload.
#[derive(Debug, Clone, PartialEq)]
pub struct TriBatch {
    pub triangles: Vec<[Vec3; 3]>,
    pub color: [f32; 4],
    pub depth_test: bool,
}

impl TriBatch {
    pub fn new(color: [f32; 4]) -> Self {
        Self {
            triangles: Vec::new(),
            color,
            depth_test: false,
        }
    }
}

#[derive(Debug, Clone)]
pub struct Scene<'a> {
    pub camera: &'a Camera,
    pub background: [f32; 4],
    pub meshes: Vec<MeshInstance>,
    pub lines: Vec<LineBatch>,
    pub points: Vec<PointBatch>,
    pub tris: Vec<TriBatch>,
    /// Draw the adaptive construction grid with the frame's x axis red and y axis green.
    pub show_grid: bool,
    /// Plane the grid lies on. Sketch mode puts it on the sketch plane so the lines the
    /// user snaps to are the lines they can see.
    pub grid_frame: Frame,
}

impl<'a> Scene<'a> {
    pub const DEFAULT_BACKGROUND: [f32; 4] = [0.16, 0.17, 0.19, 1.0];

    pub fn new(camera: &'a Camera) -> Self {
        Self {
            camera,
            background: Self::DEFAULT_BACKGROUND,
            meshes: Vec::new(),
            lines: Vec::new(),
            points: Vec::new(),
            tris: Vec::new(),
            show_grid: true,
            grid_frame: Frame::XY,
        }
    }
}
