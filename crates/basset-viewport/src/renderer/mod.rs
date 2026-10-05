//! The wgpu renderer: pipelines, GPU meshes, frame targets and per-frame draw recording.
//!
//! Rendering is split into two phases so the borrow checker, not conventions, guarantees
//! nothing is reallocated while a render pass references it: `prepare` (`&mut self`) writes
//! every buffer and bind group the frame needs and returns a plain [`FrameDraws`] list;
//! `record` (`&self`) then only reads.

mod gpu_mesh;
mod pipelines;
mod uniforms;

use std::collections::HashMap;
use std::ops::Range;

use basset_math::{Mat4, TriMesh, Vec3};
use bytemuck::Zeroable;

use crate::camera::Camera;
use crate::error::ViewportError;
use crate::grid;
use crate::lighting::{EnvironmentLight, Lighting, sky_irradiance_coefficients};
use crate::scene::{LineBatch, MeshHandle, MeshInstance, MeshStyle, PointBatch, Scene, TriBatch};
use crate::silhouette::{SilhouetteCache, ViewPoint};

use gpu_mesh::{GpuMesh, SegmentInstance};
use pipelines::{DEPTH_FORMAT, Layouts, Pipelines};
use uniforms::{
    Globals, LightUniform, LineDraw, MeshDraw, PointDraw, StreamBuffer, TriDraw, UniformArena,
};

const EDGE_WIDTH_PX: f32 = 1.2;
const DASH_PX: f32 = 8.0;
/// How far apart two segment ends may be and still count as joined, squared. Curves are
/// tessellated to shared points, so this only has to absorb the `f32` conversion.
const JOIN_EPS: f64 = 1e-12;
const GAP_PX: f32 = 5.0;

pub struct Renderer {
    color_format: wgpu::TextureFormat,
    msaa_samples: u32,
    layouts: Layouts,
    pipelines: Pipelines,
    globals_buffer: wgpu::Buffer,
    globals_bind_group: wgpu::BindGroup,
    targets: Option<FrameTargets>,
    meshes: HashMap<MeshHandle, GpuMesh>,
    next_handle: u64,
    face_bits: HashMap<FaceBitsKey, FaceBits>,
    /// One silhouette per drawn instance, keyed as the face bits are. Per instance
    /// rather than per mesh because the answer depends on where the body is standing as
    /// well as where the camera is.
    silhouettes: HashMap<FaceBitsKey, SilhouetteCache>,
    uniforms: UniformArena,
    line_instances: StreamBuffer,
    point_instances: StreamBuffer,
    tri_vertices: StreamBuffer,
}

/// Depth buffer and, when multisampling, the colour buffer that gets resolved into the
/// caller's target. Sized to the last `render`/`resize` call.
struct FrameTargets {
    size: [u32; 2],
    depth_view: wgpu::TextureView,
    msaa_view: Option<wgpu::TextureView>,
}

/// Per-face state is cached per (mesh, ordinal of that mesh within the frame's instance
/// list). The ordinal lets two instances of one mesh carry different selections and
/// masks while still reusing buffers frame to frame, which is the common case.
type FaceBitsKey = (MeshHandle, u32);

/// An instance's two sets of faces, one bit per face id (see [`face_bits`]): the ones its
/// highlight paints and the ones its mask lets through. Each buffer is rewritten only when
/// its list changes.
struct FaceBits {
    highlight_faces: Vec<u32>,
    highlight_buffer: wgpu::Buffer,
    /// The faces the mask buffer holds. An instance without a mask leaves the buffer as it
    /// was, since the draw's flag tells the shader not to read it.
    mask_faces: Vec<u32>,
    mask_buffer: wgpu::Buffer,
    bind_group: wgpu::BindGroup,
    used_this_frame: bool,
}

/// One bit per face id in `words` little-endian `u32`s: face `f` is bit `f % 32` of word
/// `f / 32`, as the mesh shader reads it. Ids beyond the mesh's largest are dropped, and
/// would not be on any of its triangles anyway.
fn face_bits(faces: &[u32], words: u32) -> Vec<u32> {
    let mut bits = vec![0u32; words as usize];
    for &face in faces {
        if let Some(word) = bits.get_mut((face / 32) as usize) {
            *word |= 1 << (face % 32);
        }
    }
    bits
}

#[derive(Debug, Clone, Copy)]
enum SegmentSource {
    /// The frame's shared line instance stream.
    Frame,
    /// A mesh's own feature-edge buffer.
    MeshEdges(MeshHandle),
}

struct MeshDrawCall {
    handle: MeshHandle,
    uniform_offset: u32,
    face_bits: FaceBitsKey,
    pass: MeshPass,
}

/// Which of the mesh pipelines a call is drawn with, in the order the passes run.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum MeshPass {
    Opaque,
    /// Translucent over the opaque meshes, depth-tested against them: ghost, x-ray and
    /// translucent styles.
    Ghost,
    /// Translucent over everything, tested against nothing.
    Overlay,
}

impl MeshPass {
    const ALL: [MeshPass; 3] = [MeshPass::Opaque, MeshPass::Ghost, MeshPass::Overlay];

    fn of(style: MeshStyle) -> Self {
        if style.shows_through() {
            MeshPass::Overlay
        } else if style.is_translucent() {
            MeshPass::Ghost
        } else {
            MeshPass::Opaque
        }
    }
}

struct LineDrawCall {
    uniform_offset: u32,
    instances: Range<u32>,
    source: SegmentSource,
    depth_test: bool,
}

struct PointDrawCall {
    uniform_offset: u32,
    instances: Range<u32>,
    depth_test: bool,
}

struct TriDrawCall {
    uniform_offset: u32,
    vertices: Range<u32>,
    depth_test: bool,
}

#[derive(Default)]
struct FrameDraws {
    /// Draw the sky gradient behind everything instead of leaving the clear colour.
    sky: bool,
    meshes: Vec<MeshDrawCall>,
    lines: Vec<LineDrawCall>,
    points: Vec<PointDrawCall>,
    tris: Vec<TriDrawCall>,
}

impl Renderer {
    /// `color_format` must match the texture views later passed to [`Renderer::render`].
    /// `msaa_samples` is 1 or 4; other values fall back to 1 because only those two are
    /// guaranteed by WebGPU for every colour format.
    pub fn new(
        device: &wgpu::Device,
        color_format: wgpu::TextureFormat,
        msaa_samples: u32,
    ) -> Self {
        let msaa_samples = match msaa_samples {
            1 | 4 => msaa_samples,
            other => {
                log::warn!("unsupported MSAA sample count {other}; falling back to 1");
                1
            }
        };
        let layouts = pipelines::create_layouts(device);
        let pipelines = pipelines::create_pipelines(device, &layouts, color_format, msaa_samples);
        let globals_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("globals"),
            size: size_of::<Globals>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let globals_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("globals"),
            layout: &layouts.globals,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: globals_buffer.as_entire_binding(),
            }],
        });
        Self {
            color_format,
            msaa_samples,
            layouts,
            pipelines,
            globals_buffer,
            globals_bind_group,
            targets: None,
            meshes: HashMap::new(),
            next_handle: 1,
            face_bits: HashMap::new(),
            silhouettes: HashMap::new(),
            uniforms: UniformArena::new(),
            line_instances: StreamBuffer::new("line instances", wgpu::BufferUsages::VERTEX),
            point_instances: StreamBuffer::new("point instances", wgpu::BufferUsages::VERTEX),
            tri_vertices: StreamBuffer::new("triangle vertices", wgpu::BufferUsages::VERTEX),
        }
    }

    pub fn msaa_samples(&self) -> u32 {
        self.msaa_samples
    }

    pub fn color_format(&self) -> wgpu::TextureFormat {
        self.color_format
    }

    pub fn depth_format() -> wgpu::TextureFormat {
        DEPTH_FORMAT
    }

    /// Recreates the size-dependent targets eagerly. `render` also does this lazily, so
    /// calling `resize` is optional; it merely moves the allocation off the frame path.
    pub fn resize(&mut self, device: &wgpu::Device, size: [u32; 2]) {
        if size[0] == 0 || size[1] == 0 {
            self.targets = None;
            return;
        }
        self.ensure_targets(device, size);
    }

    /// Converts the mesh to `f32` GPU buffers. `edges` are the segments drawn by every
    /// style with [`MeshStyle::draws_edges`]; the caller supplies them because only the modeller
    /// that built the mesh knows which of its triangle edges are real geometry, and a
    /// guess made from the triangles alone shows the user the mesh.
    ///
    /// The `queue` is unused today because upload goes through mapped-at-creation buffers,
    /// but it is part of the signature so a future streaming path does not change callers.
    pub fn upload_mesh(
        &mut self,
        device: &wgpu::Device,
        _queue: &wgpu::Queue,
        mesh: &TriMesh,
        edges: &[[Vec3; 2]],
    ) -> Result<MeshHandle, ViewportError> {
        self.insert_mesh(GpuMesh::upload(device, mesh, None, edges)?)
    }

    /// [`Self::upload_mesh`] with a linear RGB colour per vertex of `mesh.positions`. An
    /// instance of the result ignores its `color` and shows these, lit as usual, with
    /// highlights still painted over them: the colour is the data (a stress plot), and the
    /// instance colour would hide it.
    pub fn upload_colored_mesh(
        &mut self,
        device: &wgpu::Device,
        _queue: &wgpu::Queue,
        mesh: &TriMesh,
        colors: &[[f32; 3]],
        edges: &[[Vec3; 2]],
    ) -> Result<MeshHandle, ViewportError> {
        self.insert_mesh(GpuMesh::upload(device, mesh, Some(colors), edges)?)
    }

    fn insert_mesh(&mut self, gpu: GpuMesh) -> Result<MeshHandle, ViewportError> {
        let handle = MeshHandle(self.next_handle);
        self.next_handle += 1;
        self.meshes.insert(handle, gpu);
        Ok(handle)
    }

    pub fn remove_mesh(&mut self, handle: MeshHandle) {
        self.meshes.remove(&handle);
        self.face_bits.retain(|(h, _), _| *h != handle);
        self.silhouettes.retain(|(h, _), _| *h != handle);
    }

    pub fn has_mesh(&self, handle: MeshHandle) -> bool {
        self.meshes.contains_key(&handle)
    }

    /// Draws the scene into `target`, clearing it with `scene.background` (or, under an
    /// environment with [`EnvironmentLight::sky_background`], painting the sky over it).
    /// `target` must have the renderer's colour format, a single sample, and `size` pixels.
    pub fn render(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        encoder: &mut wgpu::CommandEncoder,
        target: &wgpu::TextureView,
        size: [u32; 2],
        scene: &Scene<'_>,
    ) {
        if size[0] == 0 || size[1] == 0 {
            return;
        }
        self.ensure_targets(device, size);
        let draws = self.prepare(device, queue, size, scene);
        self.record(encoder, target, scene.background, &draws);
    }

    fn ensure_targets(&mut self, device: &wgpu::Device, size: [u32; 2]) {
        if self.targets.as_ref().is_some_and(|t| t.size == size) {
            return;
        }
        let extent = wgpu::Extent3d {
            width: size[0],
            height: size[1],
            depth_or_array_layers: 1,
        };
        let make = |label, format, sample_count| {
            device
                .create_texture(&wgpu::TextureDescriptor {
                    label: Some(label),
                    size: extent,
                    mip_level_count: 1,
                    sample_count,
                    dimension: wgpu::TextureDimension::D2,
                    format,
                    usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
                    view_formats: &[],
                })
                .create_view(&wgpu::TextureViewDescriptor::default())
        };
        let depth_view = make("viewport depth", DEPTH_FORMAT, self.msaa_samples);
        let msaa_view = (self.msaa_samples > 1)
            .then(|| make("viewport msaa colour", self.color_format, self.msaa_samples));
        self.targets = Some(FrameTargets {
            size,
            depth_view,
            msaa_view,
        });
    }

    fn write_globals(
        &self,
        queue: &wgpu::Queue,
        camera: &Camera,
        size: [u32; 2],
        lighting: &Lighting,
    ) {
        let aspect = f64::from(size[0]) / f64::from(size[1]);
        // Key light sits above and to the left of the eye so it follows the orbit: a
        // world-fixed light would leave the model dark from half the view directions.
        let key_light = (-camera.right() * 0.5 + camera.up() * 0.7 - camera.forward()).normalize();
        let view_proj = camera.view_projection(aspect);
        let mut globals = Globals {
            view_proj: view_proj.as_mat4().to_cols_array_2d(),
            camera_pos: camera.eye().as_vec3().extend(1.0).to_array(),
            key_light: key_light.as_vec3().extend(0.0).to_array(),
            viewport: [
                size[0] as f32,
                size[1] as f32,
                1.0 / size[0] as f32,
                1.0 / size[1] as f32,
            ],
            // Inverted in f64 and only then narrowed: the far plane is many times the eye
            // distance away, and an inverse taken in f32 loses the sky's direction.
            inv_view_proj: view_proj.inverse().as_mat4().to_cols_array_2d(),
            environment: [0.0; 4],
            sky_zenith: [0.0; 4],
            sky_horizon: [0.0; 4],
            sky_nadir: [0.0; 4],
            sky_irradiance: [[0.0; 4]; 4],
            lights: [LightUniform::zeroed(); 4],
        };
        if let Lighting::Environment(env) = lighting {
            write_environment(&mut globals, env);
        }
        queue.write_buffer(&self.globals_buffer, 0, bytemuck::bytes_of(&globals));
    }

    fn prepare(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        size: [u32; 2],
        scene: &Scene<'_>,
    ) -> FrameDraws {
        self.write_globals(queue, scene.camera, size, &scene.lighting);
        self.uniforms.begin();
        self.line_instances.clear();
        self.point_instances.clear();
        self.tri_vertices.clear();
        for entry in self.face_bits.values_mut() {
            entry.used_this_frame = false;
        }
        for entry in self.silhouettes.values_mut() {
            entry.used_this_frame = false;
        }

        let mut draws = FrameDraws {
            sky: matches!(&scene.lighting, Lighting::Environment(env) if env.sky_background),
            ..FrameDraws::default()
        };
        let mut ordinals: HashMap<MeshHandle, u32> = HashMap::new();
        for instance in &scene.meshes {
            let Some(gpu) = self.meshes.get(&instance.handle) else {
                log::debug!("scene references removed mesh {:?}", instance.handle);
                continue;
            };
            let ordinal = ordinals.entry(instance.handle).or_default();
            let key = (instance.handle, *ordinal);
            *ordinal += 1;

            // A mask of no faces draws nothing, so it needs no draw call; its bits are still
            // kept up to date below so the cache entry survives the frame.
            let masked_out = instance.face_mask.as_ref().is_some_and(Vec::is_empty);
            if instance.style.draws_faces() && !masked_out {
                let mut color = instance.color;
                if instance.style.fades() {
                    color[3] *= 0.35;
                }
                let material = &instance.material;
                let uniform_offset = self.uniforms.push(&MeshDraw {
                    model: instance.transform.as_mat4().to_cols_array_2d(),
                    normal_matrix: normal_matrix(&instance.transform)
                        .as_mat4()
                        .to_cols_array_2d(),
                    color,
                    highlight_color: instance.highlight_color,
                    params: [
                        if gpu.vertex_colored { 1.0 } else { 0.0 },
                        if instance.face_mask.is_some() {
                            1.0
                        } else {
                            0.0
                        },
                        0.0,
                        0.0,
                    ],
                    material: [
                        material.metallic,
                        material.roughness,
                        material.clearcoat,
                        0.0,
                    ],
                    emission: [
                        material.emission[0],
                        material.emission[1],
                        material.emission[2],
                        0.0,
                    ],
                });
                draws.meshes.push(MeshDrawCall {
                    handle: instance.handle,
                    uniform_offset,
                    face_bits: key,
                    pass: MeshPass::of(instance.style),
                });
            }
            if instance.style.draws_edges() && gpu.edges.is_some() {
                let uniform_offset = self.uniforms.push(&LineDraw {
                    model: instance.transform.as_mat4().to_cols_array_2d(),
                    color: instance.edge_color,
                    params: [EDGE_WIDTH_PX, 0.0, DASH_PX, GAP_PX],
                });
                draws.lines.push(LineDrawCall {
                    uniform_offset,
                    instances: 0..gpu.edge_count,
                    source: SegmentSource::MeshEdges(instance.handle),
                    // Only the shaded style hides the edges it occludes. Wireframe and
                    // x-ray exist to show the far side of the body, and depth-testing
                    // their edges would hide exactly what the user switched modes to see.
                    depth_test: instance.style == MeshStyle::ShadedWithEdges,
                });
            }
            let words = gpu.highlight_words;
            if instance.style.draws_silhouette() {
                self.push_silhouette(instance, key, scene.camera, &mut draws);
            }
            self.sync_face_bits(device, queue, key, words, instance);
        }
        self.face_bits.retain(|_, entry| entry.used_this_frame);
        self.silhouettes.retain(|_, entry| entry.used_this_frame);

        // The grid goes first so model lines drawn later paint over it where they coincide.
        let grid_batches = if scene.show_grid {
            grid::build(scene.camera, size, &scene.grid_frame, scene.show_grid_axes)
        } else {
            Vec::new()
        };
        for batch in grid_batches.iter().chain(&scene.lines) {
            self.push_line_batch(batch, &mut draws);
        }
        for batch in &scene.points {
            self.push_point_batch(batch, &mut draws);
        }
        for batch in &scene.tris {
            self.push_tri_batch(batch, &mut draws);
        }

        self.uniforms.flush(device, queue, &self.layouts.draw);
        self.line_instances.flush(device, queue, 0);
        self.point_instances.flush(device, queue, 0);
        self.tri_vertices.flush(device, queue, 0);
        draws
    }

    /// Adds the instance's silhouette to the frame's line stream — the same pixel-width
    /// batch the kernel's feature edges and every sketch overlay go through, so it is the
    /// same line, drawn the same width, in the same colour as the body's other edges.
    ///
    /// The segments are in the mesh's own coordinates and the draw carries the instance
    /// transform, exactly as the feature-edge buffer does.
    fn push_silhouette(
        &mut self,
        instance: &MeshInstance,
        key: FaceBitsKey,
        camera: &Camera,
        draws: &mut FrameDraws,
    ) {
        // Disjoint field borrows: the cache is read while the stream and the uniform arena
        // are written, and all three hang off `self`.
        let Self {
            meshes,
            silhouettes,
            line_instances,
            uniforms,
            ..
        } = self;
        let Some(gpu) = meshes.get(&instance.handle) else {
            return;
        };
        if gpu.silhouette.is_empty() {
            return;
        }
        let cache = silhouettes.entry(key).or_default();
        cache.used_this_frame = true;
        let view = ViewPoint::for_instance(camera, &instance.transform);
        let segments = cache.segments(&gpu.silhouette, view);
        if segments.is_empty() {
            return;
        }
        let first = (line_instances.len() / size_of::<SegmentInstance>()) as u32;
        for [a, b] in segments {
            // No running distance: a silhouette is never dashed, and its segments arrive
            // in vertex order rather than walked along a polyline, so there is no run for
            // a dash pattern to follow anyway.
            line_instances.push(&SegmentInstance::new(*a, *b));
        }
        let uniform_offset = uniforms.push(&LineDraw {
            model: instance.transform.as_mat4().to_cols_array_2d(),
            color: instance.edge_color,
            params: [EDGE_WIDTH_PX, 0.0, DASH_PX, GAP_PX],
        });
        draws.lines.push(LineDrawCall {
            uniform_offset,
            instances: first..first + segments.len() as u32,
            source: SegmentSource::Frame,
            // As for the feature edges: only the shaded style hides what it occludes.
            depth_test: instance.style == MeshStyle::ShadedWithEdges,
        });
    }

    fn push_line_batch(&mut self, batch: &LineBatch, draws: &mut FrameDraws) {
        if batch.segments.is_empty() {
            return;
        }
        let first = (self.line_instances.len() / size_of::<SegmentInstance>()) as u32;
        // Segments of one polyline arrive in order, each starting where the last ended,
        // which is how a dash pattern knows to run on across the joins. A segment that
        // starts somewhere else begins a new run.
        let mut travelled = 0.0;
        let mut previous_end: Option<Vec3> = None;
        for [a, b] in &batch.segments {
            if previous_end.is_none_or(|end| end.distance_squared(*a) > JOIN_EPS) {
                travelled = 0.0;
            }
            self.line_instances
                .push(&SegmentInstance::at(*a, *b, travelled));
            travelled += a.distance(*b);
            previous_end = Some(*b);
        }
        let uniform_offset = self.uniforms.push(&LineDraw {
            model: Mat4::IDENTITY.as_mat4().to_cols_array_2d(),
            color: batch.color,
            params: [
                batch.width_px,
                if batch.dashed { 1.0 } else { 0.0 },
                DASH_PX,
                GAP_PX,
            ],
        });
        draws.lines.push(LineDrawCall {
            uniform_offset,
            instances: first..first + batch.segments.len() as u32,
            source: SegmentSource::Frame,
            depth_test: batch.depth_test,
        });
    }

    fn push_point_batch(&mut self, batch: &PointBatch, draws: &mut FrameDraws) {
        if batch.points.is_empty() {
            return;
        }
        let first = (self.point_instances.len() / size_of::<[f32; 3]>()) as u32;
        for p in &batch.points {
            self.point_instances.push(&p.as_vec3().to_array());
        }
        let uniform_offset = self.uniforms.push(&PointDraw {
            color: batch.color,
            params: [batch.size_px, 0.0, 0.0, 0.0],
        });
        draws.points.push(PointDrawCall {
            uniform_offset,
            instances: first..first + batch.points.len() as u32,
            depth_test: batch.depth_test,
        });
    }

    fn push_tri_batch(&mut self, batch: &TriBatch, draws: &mut FrameDraws) {
        if batch.triangles.is_empty() {
            return;
        }
        let first = (self.tri_vertices.len() / size_of::<[f32; 3]>()) as u32;
        for tri in &batch.triangles {
            for v in tri {
                self.tri_vertices.push(&v.as_vec3().to_array());
            }
        }
        let uniform_offset = self.uniforms.push(&TriDraw {
            color: batch.color,
            params: [0.0; 4],
        });
        draws.tris.push(TriDrawCall {
            uniform_offset,
            vertices: first..first + (batch.triangles.len() * 3) as u32,
            depth_test: batch.depth_test,
        });
    }

    /// Creates or updates the highlight and mask bit buffers for one instance. The buffers
    /// are sized by the mesh, so a changed selection or mask is a `write_buffer`, never a
    /// reallocation.
    fn sync_face_bits(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        key: FaceBitsKey,
        words: u32,
        instance: &MeshInstance,
    ) {
        let highlight = instance.highlight_faces.as_slice();
        let mask = instance.face_mask.as_deref();
        match self.face_bits.get_mut(&key) {
            Some(entry) => {
                if entry.highlight_faces != highlight {
                    let bits = face_bits(highlight, words);
                    queue.write_buffer(&entry.highlight_buffer, 0, bytemuck::cast_slice(&bits));
                    entry.highlight_faces = highlight.to_vec();
                }
                if let Some(mask) = mask
                    && entry.mask_faces != mask
                {
                    let bits = face_bits(mask, words);
                    queue.write_buffer(&entry.mask_buffer, 0, bytemuck::cast_slice(&bits));
                    entry.mask_faces = mask.to_vec();
                }
                entry.used_this_frame = true;
            }
            None => {
                let make = |label, faces: &[u32]| {
                    let buffer = device.create_buffer(&wgpu::BufferDescriptor {
                        label: Some(label),
                        size: u64::from(words) * 4,
                        usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
                        mapped_at_creation: false,
                    });
                    queue.write_buffer(&buffer, 0, bytemuck::cast_slice(&face_bits(faces, words)));
                    buffer
                };
                let mask_faces = mask.unwrap_or_default();
                let highlight_buffer = make("highlight bits", highlight);
                let mask_buffer = make("face mask bits", mask_faces);
                let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
                    label: Some("face bits"),
                    layout: &self.layouts.face_bits,
                    entries: &[
                        wgpu::BindGroupEntry {
                            binding: 0,
                            resource: highlight_buffer.as_entire_binding(),
                        },
                        wgpu::BindGroupEntry {
                            binding: 1,
                            resource: mask_buffer.as_entire_binding(),
                        },
                    ],
                });
                self.face_bits.insert(
                    key,
                    FaceBits {
                        highlight_faces: highlight.to_vec(),
                        highlight_buffer,
                        mask_faces: mask_faces.to_vec(),
                        mask_buffer,
                        bind_group,
                        used_this_frame: true,
                    },
                );
            }
        }
    }

    fn record(
        &self,
        encoder: &mut wgpu::CommandEncoder,
        target: &wgpu::TextureView,
        background: [f32; 4],
        draws: &FrameDraws,
    ) {
        let targets = self.targets.as_ref().expect("ensure_targets ran");
        let clear = wgpu::Color {
            r: f64::from(background[0]),
            g: f64::from(background[1]),
            b: f64::from(background[2]),
            a: f64::from(background[3]),
        };
        let color_attachment = match &targets.msaa_view {
            Some(msaa) => wgpu::RenderPassColorAttachment {
                view: msaa,
                depth_slice: None,
                resolve_target: Some(target),
                // Only the resolved image is needed; discarding the MSAA surface saves
                // bandwidth on every GPU and matters on tilers.
                ops: wgpu::Operations {
                    load: wgpu::LoadOp::Clear(clear),
                    store: wgpu::StoreOp::Discard,
                },
            },
            None => wgpu::RenderPassColorAttachment {
                view: target,
                depth_slice: None,
                resolve_target: None,
                ops: wgpu::Operations {
                    load: wgpu::LoadOp::Clear(clear),
                    store: wgpu::StoreOp::Store,
                },
            },
        };
        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("viewport"),
            color_attachments: &[Some(color_attachment)],
            depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                view: &targets.depth_view,
                depth_ops: Some(wgpu::Operations {
                    load: wgpu::LoadOp::Clear(1.0),
                    store: wgpu::StoreOp::Discard,
                }),
                stencil_ops: None,
            }),
            timestamp_writes: None,
            occlusion_query_set: None,
            multiview_mask: None,
        });
        pass.set_bind_group(0, &self.globals_bind_group, &[]);
        let draw_uniforms = self.uniforms.bind_group();

        if draws.sky {
            pass.set_pipeline(&self.pipelines.sky);
            pass.draw(0..3, 0..1);
        }

        // Opaque first so the translucent styles blend over them, then the overlay that
        // ignores depth; then depth-tested annotations, then the overlays that must stay
        // visible through geometry.
        for mesh_pass in MeshPass::ALL {
            pass.set_pipeline(match mesh_pass {
                MeshPass::Opaque => &self.pipelines.mesh_opaque,
                MeshPass::Ghost => &self.pipelines.mesh_ghost,
                MeshPass::Overlay => &self.pipelines.mesh_overlay,
            });
            for call in draws.meshes.iter().filter(|c| c.pass == mesh_pass) {
                let (Some(mesh), Some(face_bits)) = (
                    self.meshes.get(&call.handle),
                    self.face_bits.get(&call.face_bits),
                ) else {
                    continue;
                };
                pass.set_bind_group(1, draw_uniforms, &[call.uniform_offset]);
                pass.set_bind_group(2, &face_bits.bind_group, &[]);
                pass.set_vertex_buffer(0, mesh.vertices.slice(..));
                pass.set_index_buffer(mesh.indices.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(0..mesh.index_count, 0, 0..1);
            }
        }
        for depth_test in [true, false] {
            // Region fills first, so the outlines and points drawn next sit on top.
            pass.set_pipeline(if depth_test {
                &self.pipelines.tris_depth
            } else {
                &self.pipelines.tris_overlay
            });
            for call in draws.tris.iter().filter(|c| c.depth_test == depth_test) {
                let Some(buffer) = self.tri_vertices.buffer() else {
                    continue;
                };
                pass.set_bind_group(1, draw_uniforms, &[call.uniform_offset]);
                pass.set_vertex_buffer(0, buffer.slice(..));
                pass.draw(call.vertices.clone(), 0..1);
            }
            pass.set_pipeline(if depth_test {
                &self.pipelines.lines_depth
            } else {
                &self.pipelines.lines_overlay
            });
            for call in draws.lines.iter().filter(|c| c.depth_test == depth_test) {
                let buffer = match call.source {
                    SegmentSource::Frame => self.line_instances.buffer(),
                    SegmentSource::MeshEdges(handle) => {
                        self.meshes.get(&handle).and_then(|m| m.edges.as_ref())
                    }
                };
                let Some(buffer) = buffer else { continue };
                pass.set_bind_group(1, draw_uniforms, &[call.uniform_offset]);
                pass.set_vertex_buffer(0, buffer.slice(..));
                pass.draw(0..6, call.instances.clone());
            }
            pass.set_pipeline(if depth_test {
                &self.pipelines.points_depth
            } else {
                &self.pipelines.points_overlay
            });
            for call in draws.points.iter().filter(|c| c.depth_test == depth_test) {
                let Some(buffer) = self.point_instances.buffer() else {
                    continue;
                };
                pass.set_bind_group(1, draw_uniforms, &[call.uniform_offset]);
                pass.set_vertex_buffer(0, buffer.slice(..));
                pass.draw(0..6, call.instances.clone());
            }
        }
    }
}

/// Fills the environment fields of the frame's globals. Directions are normalised and
/// radii clamped here, once, so the shader can trust them.
fn write_environment(globals: &mut Globals, env: &EnvironmentLight) {
    let rgb = |c: [f32; 3]| [c[0], c[1], c[2], 0.0];
    let lights = &env.lights[..env.lights.len().min(EnvironmentLight::MAX_LIGHTS)];
    globals.environment = [1.0, env.exposure, lights.len() as f32, 0.0];
    globals.sky_zenith = rgb(env.zenith);
    globals.sky_horizon = rgb(env.horizon);
    globals.sky_nadir = rgb(env.nadir);
    globals.sky_irradiance = sky_irradiance_coefficients(env).map(rgb);
    for (slot, light) in globals.lights.iter_mut().zip(lights) {
        let direction = light.direction.normalize_or_zero().as_vec3();
        let radius = light.angular_radius.clamp(0.0, std::f32::consts::FRAC_PI_2);
        *slot = LightUniform {
            direction: direction.extend(radius).to_array(),
            irradiance: rgb(light.irradiance()),
        };
    }
}

/// Inverse transpose for transforming normals; falls back to the model matrix itself when
/// it is singular (a zero-scale preview) rather than propagating NaNs into the shader.
fn normal_matrix(model: &Mat4) -> Mat4 {
    if model.determinant().abs() < 1e-18 {
        return *model;
    }
    let inverse_transpose = model.inverse().transpose();
    // Drop the translation-derived column so it stays a pure direction transform.
    Mat4::from_cols(
        inverse_transpose.x_axis.truncate().extend(0.0),
        inverse_transpose.y_axis.truncate().extend(0.0),
        inverse_transpose.z_axis.truncate().extend(0.0),
        Vec3::ZERO.extend(1.0),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn face_bits_set_one_bit_per_face_in_the_shaders_layout() {
        assert_eq!(face_bits(&[], 2), vec![0, 0]);
        assert_eq!(face_bits(&[0], 1), vec![1]);
        assert_eq!(face_bits(&[31], 1), vec![1 << 31]);
        // Face 32 is the first bit of the second word, as `face_id / 32u` reads it.
        assert_eq!(face_bits(&[3, 32, 33, 3], 2), vec![1 << 3, 0b11]);
    }

    #[test]
    fn face_bits_drop_faces_the_mesh_does_not_have() {
        // A stale selection naming a face beyond the mesh must not write past its buffer.
        assert_eq!(face_bits(&[1, 64, 1000], 2), vec![0b10, 0]);
    }
}
