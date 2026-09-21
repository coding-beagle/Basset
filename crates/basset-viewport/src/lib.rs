//! wgpu renderer for the Basset viewport.
//!
//! This crate is deliberately windowing-agnostic: the application owns the window, the
//! surface, the `wgpu::Device`/`Queue` and the UI layer, and hands us a command encoder plus
//! a texture view to draw into each frame. That keeps the renderer testable headlessly
//! (see `tests/gpu.rs`) and lets the same code render into an egui-hosted texture, a bare
//! swapchain, or an offscreen export target.
//!
//! The pieces:
//!
//! * [`Camera`] — an orbit camera with the CAD convention of +Z up, all maths in `f64`.
//! * [`Scene`] — a per-frame description of what to draw: mesh instances, line batches,
//!   point batches, background and grid toggle. It borrows nothing from the renderer except
//!   [`MeshHandle`]s, so the application can rebuild it cheaply every frame.
//! * [`Renderer`] — owns pipelines, GPU meshes and the depth / MSAA targets.
//! * [`grid`] — CPU generation of the adaptive ground grid as a plain [`LineBatch`].
//! * [`silhouette`] — the outline of a curved body against what is behind it, which is
//!   view-dependent and so recomputed here per camera change rather than by the kernel.

pub mod camera;
pub mod error;
pub mod grid;
pub mod renderer;
pub mod scene;
pub mod silhouette;

pub use camera::{Camera, Projection, ViewPreset};
pub use error::ViewportError;
pub use renderer::Renderer;
pub use scene::{LineBatch, MeshHandle, MeshInstance, MeshStyle, PointBatch, Scene, TriBatch};
pub use silhouette::{Silhouette, SilhouetteCache, ViewPoint};
