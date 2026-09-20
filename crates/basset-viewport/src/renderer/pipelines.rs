//! Bind group layouts and render pipelines. Everything here is created once in
//! [`crate::Renderer::new`]; nothing depends on the viewport size.

use super::gpu_mesh::{MeshVertex, SegmentInstance};
use super::uniforms::UniformArena;

pub(crate) struct Layouts {
    /// Group 0: frame-wide uniforms.
    pub globals: wgpu::BindGroupLayout,
    /// Group 1: per-draw uniforms selected by dynamic offset.
    pub draw: wgpu::BindGroupLayout,
    /// Group 2 (mesh only): highlight bit set indexed by face id.
    pub highlight: wgpu::BindGroupLayout,
}

pub(crate) struct Pipelines {
    pub mesh_opaque: wgpu::RenderPipeline,
    pub mesh_ghost: wgpu::RenderPipeline,
    pub lines_depth: wgpu::RenderPipeline,
    pub lines_overlay: wgpu::RenderPipeline,
    pub points_depth: wgpu::RenderPipeline,
    pub points_overlay: wgpu::RenderPipeline,
    pub tris_depth: wgpu::RenderPipeline,
    pub tris_overlay: wgpu::RenderPipeline,
}

pub(crate) const DEPTH_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Depth32Float;

const COMMON_WGSL: &str = include_str!("../shaders/common.wgsl");
const MESH_WGSL: &str = include_str!("../shaders/mesh.wgsl");
const LINES_WGSL: &str = include_str!("../shaders/lines.wgsl");
const POINTS_WGSL: &str = include_str!("../shaders/points.wgsl");
const TRIS_WGSL: &str = include_str!("../shaders/tris.wgsl");

/// Pushes faces back a little so edge and sketch lines drawn exactly on a surface win the
/// depth test instead of z-fighting. Biasing the faces rather than the lines keeps the
/// slope term well defined (it is meaningless for screen-aligned line quads).
const FACE_DEPTH_BIAS: wgpu::DepthBiasState = wgpu::DepthBiasState {
    constant: 2,
    slope_scale: 2.0,
    clamp: 0.0,
};

pub(crate) fn create_layouts(device: &wgpu::Device) -> Layouts {
    let uniform = |label, has_dynamic_offset| {
        device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some(label),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX_FRAGMENT,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset,
                    min_binding_size: None,
                },
                count: None,
            }],
        })
    };
    let highlight = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("highlight bits"),
        entries: &[wgpu::BindGroupLayoutEntry {
            binding: 0,
            visibility: wgpu::ShaderStages::VERTEX,
            ty: wgpu::BindingType::Buffer {
                ty: wgpu::BufferBindingType::Storage { read_only: true },
                has_dynamic_offset: false,
                min_binding_size: None,
            },
            count: None,
        }],
    });
    Layouts {
        globals: uniform("globals", false),
        draw: uniform("per-draw", true),
        highlight,
    }
}

struct PipelineSpec<'a> {
    label: &'a str,
    layout: &'a wgpu::PipelineLayout,
    module: &'a wgpu::ShaderModule,
    vertex_layout: wgpu::VertexBufferLayout<'a>,
    depth_write: bool,
    depth_compare: wgpu::CompareFunction,
    bias: wgpu::DepthBiasState,
    blend: wgpu::BlendState,
}

pub(crate) fn create_pipelines(
    device: &wgpu::Device,
    layouts: &Layouts,
    color_format: wgpu::TextureFormat,
    msaa_samples: u32,
) -> Pipelines {
    let shader = |label, body: &str| {
        device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some(label),
            source: wgpu::ShaderSource::Wgsl(format!("{COMMON_WGSL}\n{body}").into()),
        })
    };
    let mesh_module = shader("mesh", MESH_WGSL);
    let lines_module = shader("lines", LINES_WGSL);
    let points_module = shader("points", POINTS_WGSL);
    let tris_module = shader("tris", TRIS_WGSL);

    let mesh_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some("mesh"),
        bind_group_layouts: &[
            Some(&layouts.globals),
            Some(&layouts.draw),
            Some(&layouts.highlight),
        ],
        immediate_size: 0,
    });
    let overlay_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some("overlay"),
        bind_group_layouts: &[Some(&layouts.globals), Some(&layouts.draw)],
        immediate_size: 0,
    });

    let build = |spec: PipelineSpec<'_>| {
        device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some(spec.label),
            layout: Some(spec.layout),
            vertex: wgpu::VertexState {
                module: spec.module,
                entry_point: Some("vs_main"),
                compilation_options: wgpu::PipelineCompilationOptions::default(),
                buffers: &[Some(spec.vertex_layout)],
            },
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                strip_index_format: None,
                front_face: wgpu::FrontFace::Ccw,
                // Kernel meshes may be viewed from inside (sections) and sketch overlays
                // have no meaningful winding, so nothing is culled.
                cull_mode: None,
                unclipped_depth: false,
                polygon_mode: wgpu::PolygonMode::Fill,
                conservative: false,
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: DEPTH_FORMAT,
                depth_write_enabled: Some(spec.depth_write),
                depth_compare: Some(spec.depth_compare),
                stencil: wgpu::StencilState::default(),
                bias: spec.bias,
            }),
            multisample: wgpu::MultisampleState {
                count: msaa_samples,
                mask: !0,
                alpha_to_coverage_enabled: false,
            },
            fragment: Some(wgpu::FragmentState {
                module: spec.module,
                entry_point: Some("fs_main"),
                compilation_options: wgpu::PipelineCompilationOptions::default(),
                targets: &[Some(wgpu::ColorTargetState {
                    format: color_format,
                    blend: Some(spec.blend),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
            }),
            multiview_mask: None,
            cache: None,
        })
    };

    let mesh = |label, depth_write, blend| {
        build(PipelineSpec {
            label,
            layout: &mesh_layout,
            module: &mesh_module,
            vertex_layout: MeshVertex::LAYOUT,
            depth_write,
            depth_compare: wgpu::CompareFunction::Less,
            bias: FACE_DEPTH_BIAS,
            blend,
        })
    };
    let overlay = |label, module, vertex_layout, depth_test: bool| {
        build(PipelineSpec {
            label,
            layout: &overlay_layout,
            module,
            vertex_layout,
            // Lines and points never write depth: they are annotations, and letting them
            // occlude each other by depth would make dense sketches flicker.
            depth_write: false,
            depth_compare: if depth_test {
                wgpu::CompareFunction::Less
            } else {
                wgpu::CompareFunction::Always
            },
            bias: wgpu::DepthBiasState::default(),
            blend: wgpu::BlendState::ALPHA_BLENDING,
        })
    };

    Pipelines {
        mesh_opaque: mesh("mesh opaque", true, wgpu::BlendState::REPLACE),
        mesh_ghost: mesh("mesh ghost", false, wgpu::BlendState::ALPHA_BLENDING),
        lines_depth: overlay(
            "lines depth-tested",
            &lines_module,
            SegmentInstance::LAYOUT,
            true,
        ),
        lines_overlay: overlay(
            "lines overlay",
            &lines_module,
            SegmentInstance::LAYOUT,
            false,
        ),
        points_depth: overlay("points depth-tested", &points_module, POINT_LAYOUT, true),
        points_overlay: overlay("points overlay", &points_module, POINT_LAYOUT, false),
        tris_depth: overlay("tris depth-tested", &tris_module, TRI_LAYOUT, true),
        tris_overlay: overlay("tris overlay", &tris_module, TRI_LAYOUT, false),
    }
}

/// One `f32x3` position per instance; the shader expands it to a quad.
pub(crate) const POINT_LAYOUT: wgpu::VertexBufferLayout<'static> = wgpu::VertexBufferLayout {
    array_stride: 12,
    step_mode: wgpu::VertexStepMode::Instance,
    attributes: &wgpu::vertex_attr_array![0 => Float32x3],
};

/// One `f32x3` position per vertex; three vertices make a triangle.
pub(crate) const TRI_LAYOUT: wgpu::VertexBufferLayout<'static> = wgpu::VertexBufferLayout {
    array_stride: 12,
    step_mode: wgpu::VertexStepMode::Vertex,
    attributes: &wgpu::vertex_attr_array![0 => Float32x3],
};

/// Sanity check that the arena slot covers every per-draw uniform.
const _: () = {
    use super::uniforms::{LineDraw, MeshDraw, PointDraw, TriDraw};
    assert!(size_of::<MeshDraw>() as u64 <= UniformArena::SLOT);
    assert!(size_of::<LineDraw>() as u64 <= UniformArena::SLOT);
    assert!(size_of::<PointDraw>() as u64 <= UniformArena::SLOT);
    assert!(size_of::<TriDraw>() as u64 <= UniformArena::SLOT);
};
