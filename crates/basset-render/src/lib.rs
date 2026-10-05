//! Photorealistic rendering: what surfaces look like, what lights them, and a path
//! tracer that makes a picture of it — Fusion's Render workspace, minus the cloud.
//!
//! * [`appearance`] — the metallic–roughness [`Appearance`] a body or face is given, with
//!   transmission, clear coat, emission and procedural patterns.
//! * [`library`] — the appearance library the browser shows, grouped by [`Category`].
//! * [`environment`] — the procedural studios and skies a scene is lit by.
//! * [`settings`] — a scene's settings as saved in a document.
//! * [`camera`] — the camera a render is taken through, written out as numbers so it can
//!   reproduce the viewport's projection exactly.
//! * [`scene`] — meshes and appearances gathered into something to trace.
//! * [`job`] — progressive rendering on background threads, and a blocking [`render`].
//! * [`image`] — the finished picture and PNG output.
//!
//! The crate sits over `basset-math` alone. It takes triangle meshes with a face id per
//! triangle and is told which appearance each face id takes, the way `basset-fea` takes a
//! solid and face keys, so it knows nothing of documents, bodies or the kernel. Like the
//! FEA crate's solver, the tracer runs on the CPU: the GPU belongs to the viewport, and a
//! CPU tracer is the one that runs headless, in the MCP server and in tests.

pub mod appearance;
mod bvh;
pub mod camera;
pub mod color;
pub mod environment;
pub mod image;
pub mod job;
pub mod library;
mod sampling;
pub mod scene;
pub mod settings;
mod trace;

pub use appearance::{Appearance, Category, Pattern};
pub use camera::{Lens, RenderCamera};
pub use color::{Rgb, Srgb};
pub use environment::{DistantLight, Environment, EnvironmentKind, Lighting};
pub use image::{Image, ImageError};
pub use job::{RenderJob, RenderOptions, render};
pub use library::{find, library};
pub use scene::{SceneBuilder, TraceScene};
pub use settings::{Background, SceneSettings};
