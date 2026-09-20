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

use crate::camera::Camera;
use crate::error::ViewportError;
use crate::grid;
use crate::scene::{LineBatch, MeshHandle, MeshStyle, PointBatch, Scene, TriBatch};

use gpu_mesh::{GpuMesh, SegmentInstance};
use pipelines::{DEPTH_FORMAT, Layouts, Pipelines};
use uniforms::{Globals, LineDraw, MeshDraw, PointDraw, StreamBuffer, TriDraw, UniformArena};

/// Colour of feature edges drawn for [`MeshStyle::ShadedWithEdges`].
const EDGE_COLOR: [f32; 4] = [0.08, 0.09, 0.10, 1.0];
const EDGE_WIDTH_PX: f32 = 1.2;
const DASH_PX: f32 = 8.0;
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
    highlights: HashMap<HighlightKey, HighlightBits>,
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

/// Highlight state is cached per (mesh, ordinal of that mesh within the frame's instance
/// list). The ordinal lets two instances of one mesh carry different selections while
/// still reusing buffers frame to frame, which is the common case.
type HighlightKey = (MeshHandle, u32);

/// One bit per face id, rebuilt only when the instance's `highlight_faces` changes.
struct HighlightBits {
    faces: Vec<u32>,
    buffer: wgpu::Buffer,
    bind_group: wgpu::BindGroup,
    used_this_frame: bool,
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
    highlight: HighlightKey,
    ghost: bool,
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
            highlights: HashMap::new(),
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

    /// Converts the mesh to `f32` GPU buffers. The `queue` is unused today because upload
    /// goes through mapped-at-creation buffers, but it is part of the signature so a future
    /// streaming path does not change callers.
    pub fn upload_mesh(
        &mut self,
        device: &wgpu::Device,
        _queue: &wgpu::Queue,
        mesh: &TriMesh,
    ) -> Result<MeshHandle, ViewportError> {
        let gpu = GpuMesh::upload(device, mesh)?;
        let handle = MeshHandle(self.next_handle);
        self.next_handle += 1;
        self.meshes.insert(handle, gpu);
        Ok(handle)
    }

    pub fn remove_mesh(&mut self, handle: MeshHandle) {
        self.meshes.remove(&handle);
        self.highlights.retain(|(h, _), _| *h != handle);
    }

    pub fn has_mesh(&self, handle: MeshHandle) -> bool {
        self.meshes.contains_key(&handle)
    }

    /// Draws the scene into `target`, clearing it with `scene.background`. `target` must
    /// have the renderer's colour format, a single sample, and `size` pixels.
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

    fn write_globals(&self, queue: &wgpu::Queue, camera: &Camera, size: [u32; 2]) {
        let aspect = f64::from(size[0]) / f64::from(size[1]);
        // Key light sits above and to the left of the eye so it follows the orbit: a
        // world-fixed light would leave the model dark from half the view directions.
        let key_light = (-camera.right() * 0.5 + camera.up() * 0.7 - camera.forward()).normalize();
        let globals = Globals {
            view_proj: camera.view_projection(aspect).as_mat4().to_cols_array_2d(),
            camera_pos: camera.eye().as_vec3().extend(1.0).to_array(),
            key_light: key_light.as_vec3().extend(0.0).to_array(),
            viewport: [
                size[0] as f32,
                size[1] as f32,
                1.0 / size[0] as f32,
                1.0 / size[1] as f32,
            ],
        };
        queue.write_buffer(&self.globals_buffer, 0, bytemuck::bytes_of(&globals));
    }

    fn prepare(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        size: [u32; 2],
        scene: &Scene<'_>,
    ) -> FrameDraws {
        self.write_globals(queue, scene.camera, size);
        self.uniforms.begin();
        self.line_instances.clear();
        self.point_instances.clear();
        self.tri_vertices.clear();
        for entry in self.highlights.values_mut() {
            entry.used_this_frame = false;
        }

        let mut draws = FrameDraws::default();
        let mut ordinals: HashMap<MeshHandle, u32> = HashMap::new();
        for instance in &scene.meshes {
            let Some(gpu) = self.meshes.get(&instance.handle) else {
                log::debug!("scene references removed mesh {:?}", instance.handle);
                continue;
            };
            let ordinal = ordinals.entry(instance.handle).or_default();
            let key = (instance.handle, *ordinal);
            *ordinal += 1;

            let mut color = instance.color;
            if instance.style == MeshStyle::Ghost {
                color[3] *= 0.35;
            }
            let uniform_offset = self.uniforms.push(&MeshDraw {
                model: instance.transform.as_mat4().to_cols_array_2d(),
                normal_matrix: normal_matrix(&instance.transform)
                    .as_mat4()
                    .to_cols_array_2d(),
                color,
                highlight_color: instance.highlight_color,
            });
            draws.meshes.push(MeshDrawCall {
                handle: instance.handle,
                uniform_offset,
                highlight: key,
                ghost: instance.style == MeshStyle::Ghost,
            });
            if instance.style == MeshStyle::ShadedWithEdges && gpu.edges.is_some() {
                let uniform_offset = self.uniforms.push(&LineDraw {
                    model: instance.transform.as_mat4().to_cols_array_2d(),
                    color: EDGE_COLOR,
                    params: [EDGE_WIDTH_PX, 0.0, DASH_PX, GAP_PX],
                });
                draws.lines.push(LineDrawCall {
                    uniform_offset,
                    instances: 0..gpu.edge_count,
                    source: SegmentSource::MeshEdges(instance.handle),
                    depth_test: true,
                });
            }
            let words = gpu.highlight_words;
            self.sync_highlight(device, queue, key, words, &instance.highlight_faces);
        }
        self.highlights.retain(|_, entry| entry.used_this_frame);

        // The grid goes first so model lines drawn later paint over it where they coincide.
        let grid_batches = if scene.show_grid {
            grid::build(scene.camera, size, &scene.grid_frame)
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

    fn push_line_batch(&mut self, batch: &LineBatch, draws: &mut FrameDraws) {
        if batch.segments.is_empty() {
            return;
        }
        let first = (self.line_instances.len() / size_of::<SegmentInstance>()) as u32;
        for [a, b] in &batch.segments {
            self.line_instances.push(&SegmentInstance::new(*a, *b));
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

    /// Creates or updates the highlight bit buffer for one instance. The buffer is sized by
    /// the mesh, so a changed selection is a `write_buffer`, never a reallocation.
    fn sync_highlight(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        key: HighlightKey,
        words: u32,
        faces: &[u32],
    ) {
        let bits = |faces: &[u32]| {
            let mut bits = vec![0u32; words as usize];
            for &face in faces {
                if let Some(word) = bits.get_mut((face / 32) as usize) {
                    *word |= 1 << (face % 32);
                }
            }
            bits
        };
        match self.highlights.get_mut(&key) {
            Some(entry) => {
                if entry.faces != faces {
                    queue.write_buffer(&entry.buffer, 0, bytemuck::cast_slice(&bits(faces)));
                    entry.faces = faces.to_vec();
                }
                entry.used_this_frame = true;
            }
            None => {
                let buffer = device.create_buffer(&wgpu::BufferDescriptor {
                    label: Some("highlight bits"),
                    size: u64::from(words) * 4,
                    usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
                    mapped_at_creation: false,
                });
                queue.write_buffer(&buffer, 0, bytemuck::cast_slice(&bits(faces)));
                let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
                    label: Some("highlight bits"),
                    layout: &self.layouts.highlight,
                    entries: &[wgpu::BindGroupEntry {
                        binding: 0,
                        resource: buffer.as_entire_binding(),
                    }],
                });
                self.highlights.insert(
                    key,
                    HighlightBits {
                        faces: faces.to_vec(),
                        buffer,
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

        // Opaque first so ghosts blend over them, then depth-tested annotations, then the
        // overlays that must stay visible through geometry.
        for ghost in [false, true] {
            pass.set_pipeline(if ghost {
                &self.pipelines.mesh_ghost
            } else {
                &self.pipelines.mesh_opaque
            });
            for call in draws.meshes.iter().filter(|c| c.ghost == ghost) {
                let (Some(mesh), Some(highlight)) = (
                    self.meshes.get(&call.handle),
                    self.highlights.get(&call.highlight),
                ) else {
                    continue;
                };
                pass.set_bind_group(1, draw_uniforms, &[call.uniform_offset]);
                pass.set_bind_group(2, &highlight.bind_group, &[]);
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
