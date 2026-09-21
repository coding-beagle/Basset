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
    /// Feature edges alone, no faces: the skeleton view. Nothing writes depth, so every
    /// edge of the body is visible, which is the point of the mode.
    Wireframe,
    /// Translucent faces with their edges and no depth write, so a body standing behind
    /// another is still visible through it.
    XRay,
    /// Translucent, no depth write: used for bodies hidden behind a tool preview or
    /// components being edited in context.
    Ghost,
}

impl MeshStyle {
    /// Whether the shaded triangles are drawn at all.
    pub fn draws_faces(self) -> bool {
        !matches!(self, Self::Wireframe)
    }

    /// Whether the mesh's feature edges are drawn. A mesh uploaded without edges draws
    /// none whatever the style says.
    pub fn draws_edges(self) -> bool {
        matches!(self, Self::ShadedWithEdges | Self::Wireframe | Self::XRay)
    }

    /// Whether the body's silhouette — the view-dependent line where its surface turns
    /// away from the eye — is drawn along with its feature edges.
    ///
    /// Only the styles that draw faces *and* edges: the silhouette exists to bound a
    /// shaded face against what is behind it. A wireframe has no faces to bound, and its
    /// far side is already drawn, so the silhouette would be one more line in a picture
    /// made of lines, jumping about as the camera moves.
    pub fn draws_silhouette(self) -> bool {
        self.draws_faces() && self.draws_edges()
    }

    /// Styles that blend rather than replace and leave the depth buffer alone. They are
    /// drawn after the opaque ones so there is something for them to blend over.
    pub fn is_translucent(self) -> bool {
        matches!(self, Self::Ghost | Self::XRay)
    }
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
    /// Colour of the feature edges the style draws. It is per instance because an edge
    /// that reads well over a lit face is invisible over the background, and the
    /// edges-only styles have nothing but the background behind them.
    pub edge_color: [f32; 4],
    pub style: MeshStyle,
}

impl MeshInstance {
    pub const DEFAULT_COLOR: [f32; 4] = [0.62, 0.66, 0.70, 1.0];
    pub const DEFAULT_HIGHLIGHT: [f32; 4] = [0.20, 0.55, 1.00, 1.0];
    /// Near-black, which is what a crease on a shaded face looks like.
    pub const DEFAULT_EDGE: [f32; 4] = [0.08, 0.09, 0.10, 1.0];

    pub fn new(handle: MeshHandle) -> Self {
        Self {
            handle,
            transform: Mat4::IDENTITY,
            color: Self::DEFAULT_COLOR,
            highlight_faces: Vec::new(),
            highlight_color: Self::DEFAULT_HIGHLIGHT,
            edge_color: Self::DEFAULT_EDGE,
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn styles_agree_on_what_they_draw() {
        assert!(MeshStyle::Shaded.draws_faces() && !MeshStyle::Shaded.draws_edges());
        assert!(
            MeshStyle::ShadedWithEdges.draws_faces() && MeshStyle::ShadedWithEdges.draws_edges()
        );
        // The skeleton view is the only style with no faces, and the only reason the
        // renderer may skip a mesh draw call entirely.
        assert!(!MeshStyle::Wireframe.draws_faces() && MeshStyle::Wireframe.draws_edges());
        assert!(MeshStyle::XRay.draws_faces() && MeshStyle::XRay.draws_edges());
        assert!(MeshStyle::Ghost.draws_faces() && !MeshStyle::Ghost.draws_edges());
    }

    #[test]
    fn only_the_shaded_styles_with_edges_outline_themselves() {
        for style in [MeshStyle::ShadedWithEdges, MeshStyle::XRay] {
            assert!(style.draws_silhouette(), "{style:?}");
        }
        // Wireframe draws edges but no faces; Shaded and Ghost draw faces but no edges.
        for style in [MeshStyle::Wireframe, MeshStyle::Shaded, MeshStyle::Ghost] {
            assert!(!style.draws_silhouette(), "{style:?}");
        }
    }

    #[test]
    fn only_the_see_through_styles_are_translucent() {
        for style in [MeshStyle::Ghost, MeshStyle::XRay] {
            assert!(style.is_translucent(), "{style:?}");
        }
        for style in [
            MeshStyle::Shaded,
            MeshStyle::ShadedWithEdges,
            MeshStyle::Wireframe,
        ] {
            assert!(!style.is_translucent(), "{style:?}");
        }
    }

    #[test]
    fn a_new_instance_is_plain_shaded() {
        let instance = MeshInstance::new(MeshHandle(1));
        assert_eq!(instance.style, MeshStyle::Shaded);
        assert_eq!(instance.edge_color, MeshInstance::DEFAULT_EDGE);
        assert_eq!(instance.transform, Mat4::IDENTITY);
    }
}
