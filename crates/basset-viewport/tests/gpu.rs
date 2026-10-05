//! Headless GPU tests. They need a real adapter (Vulkan, GL, ...) and quietly skip when
//! none exists so CI without a GPU still passes.

use basset_math::{TriMesh, Vec3};
use basset_viewport::{
    Camera, DistantLight, EnvironmentLight, Lighting, LineBatch, Material, MeshInstance, MeshStyle,
    PointBatch, Renderer, Scene, TriBatch, ViewPreset,
};

const SIZE: [u32; 2] = [128, 96];
const FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Rgba8UnormSrgb;
const BACKGROUND: [f32; 4] = [0.10, 0.20, 0.30, 1.0];

struct Gpu {
    device: wgpu::Device,
    queue: wgpu::Queue,
}

fn gpu() -> Option<Gpu> {
    let instance =
        wgpu::Instance::new(wgpu::InstanceDescriptor::new_without_display_handle_from_env());
    let adapter =
        match pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions::default()))
        {
            Ok(adapter) => adapter,
            Err(err) => {
                eprintln!("no wgpu adapter available, skipping GPU test: {err}");
                return None;
            }
        };
    let (device, queue) = pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor {
        label: Some("basset-viewport test"),
        required_features: wgpu::Features::empty(),
        required_limits: wgpu::Limits::downlevel_defaults(),
        experimental_features: wgpu::ExperimentalFeatures::disabled(),
        memory_hints: wgpu::MemoryHints::default(),
        trace: wgpu::Trace::Off,
    }))
    .expect("device");
    Some(Gpu { device, queue })
}

/// The twelve real edges of [`cube`]: what the modeller would hand the renderer.
fn cube_edges(size: f64) -> Vec<[Vec3; 2]> {
    let h = size / 2.0;
    let mut out = Vec::new();
    for axis in 0..3 {
        for a in [-h, h] {
            for b in [-h, h] {
                let mut lo = [0.0; 3];
                let mut hi = [0.0; 3];
                lo[(axis + 1) % 3] = a;
                hi[(axis + 1) % 3] = a;
                lo[(axis + 2) % 3] = b;
                hi[(axis + 2) % 3] = b;
                lo[axis] = -h;
                hi[axis] = h;
                out.push([Vec3::from_array(lo), Vec3::from_array(hi)]);
            }
        }
    }
    out
}

fn cube(size: f64) -> TriMesh {
    let mut m = TriMesh::default();
    let h = size / 2.0;
    let v = |x: f64, y: f64, z: f64| Vec3::new(x * h, y * h, z * h);
    let quads = [
        [
            v(-1., -1., -1.),
            v(-1., 1., -1.),
            v(1., 1., -1.),
            v(1., -1., -1.),
        ],
        [
            v(-1., -1., 1.),
            v(1., -1., 1.),
            v(1., 1., 1.),
            v(-1., 1., 1.),
        ],
        [
            v(-1., -1., -1.),
            v(1., -1., -1.),
            v(1., -1., 1.),
            v(-1., -1., 1.),
        ],
        [
            v(-1., 1., -1.),
            v(-1., 1., 1.),
            v(1., 1., 1.),
            v(1., 1., -1.),
        ],
        [
            v(-1., -1., -1.),
            v(-1., -1., 1.),
            v(-1., 1., 1.),
            v(-1., 1., -1.),
        ],
        [
            v(1., -1., -1.),
            v(1., 1., -1.),
            v(1., 1., 1.),
            v(1., -1., 1.),
        ],
    ];
    for (id, q) in quads.iter().enumerate() {
        m.push_triangle([q[0], q[1], q[2]], id as u32);
        m.push_triangle([q[0], q[2], q[3]], id as u32);
    }
    m
}

/// Renders the scene with a fresh renderer and returns RGBA8 pixels, row-major, top-down.
fn render_to_pixels(gpu: &Gpu, msaa: u32, scene: &Scene<'_>) -> Vec<[u8; 4]> {
    let mut renderer = Renderer::new(&gpu.device, FORMAT, msaa);
    assert_eq!(renderer.msaa_samples(), msaa);
    render_with(gpu, &mut renderer, scene)
}

fn render_with(gpu: &Gpu, renderer: &mut Renderer, scene: &Scene<'_>) -> Vec<[u8; 4]> {
    let texture = gpu.device.create_texture(&wgpu::TextureDescriptor {
        label: Some("target"),
        size: wgpu::Extent3d {
            width: SIZE[0],
            height: SIZE[1],
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: FORMAT,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
        view_formats: &[],
    });
    let view = texture.create_view(&wgpu::TextureViewDescriptor::default());

    let bytes_per_row = (SIZE[0] * 4).div_ceil(wgpu::COPY_BYTES_PER_ROW_ALIGNMENT)
        * wgpu::COPY_BYTES_PER_ROW_ALIGNMENT;
    let readback = gpu.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("readback"),
        size: u64::from(bytes_per_row * SIZE[1]),
        usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
        mapped_at_creation: false,
    });

    let mut encoder = gpu
        .device
        .create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("test"),
        });
    renderer.render(&gpu.device, &gpu.queue, &mut encoder, &view, SIZE, scene);
    encoder.copy_texture_to_buffer(
        wgpu::TexelCopyTextureInfo {
            texture: &texture,
            mip_level: 0,
            origin: wgpu::Origin3d::ZERO,
            aspect: wgpu::TextureAspect::All,
        },
        wgpu::TexelCopyBufferInfo {
            buffer: &readback,
            layout: wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(bytes_per_row),
                rows_per_image: None,
            },
        },
        wgpu::Extent3d {
            width: SIZE[0],
            height: SIZE[1],
            depth_or_array_layers: 1,
        },
    );
    gpu.queue.submit([encoder.finish()]);

    let slice = readback.slice(..);
    let (tx, rx) = std::sync::mpsc::channel();
    slice.map_async(wgpu::MapMode::Read, move |result| {
        tx.send(result).expect("receiver alive")
    });
    gpu.device
        .poll(wgpu::PollType::wait_indefinitely())
        .expect("poll");
    rx.recv().expect("map callback").expect("map succeeded");

    let mapped = slice.get_mapped_range().expect("mapped range");
    let mut pixels = Vec::with_capacity((SIZE[0] * SIZE[1]) as usize);
    for row in 0..SIZE[1] {
        let start = (row * bytes_per_row) as usize;
        for col in 0..SIZE[0] {
            let i = start + (col * 4) as usize;
            pixels.push([mapped[i], mapped[i + 1], mapped[i + 2], mapped[i + 3]]);
        }
    }
    drop(mapped);
    readback.unmap();
    pixels
}

fn pixel(pixels: &[[u8; 4]], x: u32, y: u32) -> [u8; 4] {
    pixels[(y * SIZE[0] + x) as usize]
}

fn srgb_encode(v: f32) -> u8 {
    let s = if v <= 0.003_130_8 {
        v * 12.92
    } else {
        1.055 * v.powf(1.0 / 2.4) - 0.055
    };
    (s * 255.0).round() as u8
}

fn is_background(p: [u8; 4]) -> bool {
    let expected = [
        srgb_encode(BACKGROUND[0]),
        srgb_encode(BACKGROUND[1]),
        srgb_encode(BACKGROUND[2]),
        255,
    ];
    p.iter().zip(&expected).all(|(a, b)| a.abs_diff(*b) <= 2)
}

fn looking_at_origin() -> Camera {
    let mut camera = Camera::new_default();
    camera.look_from(ViewPreset::Front);
    camera.distance = 100.0;
    camera
}

#[test]
fn cube_covers_centre_but_not_corner() {
    let Some(gpu) = gpu() else { return };
    for msaa in [1, 4] {
        let mut renderer = Renderer::new(&gpu.device, FORMAT, msaa);
        let handle = renderer
            .upload_mesh(&gpu.device, &gpu.queue, &cube(20.0), &cube_edges(20.0))
            .expect("valid cube");
        let camera = looking_at_origin();
        let mut scene = Scene::new(&camera);
        scene.background = BACKGROUND;
        scene.show_grid = false;
        scene.meshes.push(MeshInstance::new(handle));

        let pixels = render_with(&gpu, &mut renderer, &scene);
        let centre = pixel(&pixels, SIZE[0] / 2, SIZE[1] / 2);
        let corner = pixel(&pixels, 1, 1);
        assert!(
            !is_background(centre),
            "msaa {msaa}: centre pixel {centre:?} should show the cube"
        );
        assert!(
            is_background(corner),
            "msaa {msaa}: corner pixel {corner:?} should be background"
        );
        // Front face shading is grey-ish: no channel dominates like the blue background.
        assert!(
            centre[0].abs_diff(centre[2]) < 40,
            "centre pixel {centre:?} should be the neutral body colour"
        );
    }
}

#[test]
fn a_coloured_mesh_shows_its_vertex_colours_and_still_highlights() {
    let Some(gpu) = gpu() else { return };
    let mut renderer = Renderer::new(&gpu.device, FORMAT, 1);
    let mesh = cube(20.0);
    // Every vertex the hot end of the ramp: the front face must come out red whatever
    // the instance colour says.
    let colors = vec![basset_viewport::stress_ramp(1.0); mesh.positions.len()];
    let handle = renderer
        .upload_colored_mesh(&gpu.device, &gpu.queue, &mesh, &colors, &cube_edges(20.0))
        .expect("valid coloured cube");
    let camera = looking_at_origin();
    let mut scene = Scene::new(&camera);
    scene.background = BACKGROUND;
    scene.show_grid = false;
    let mut instance = MeshInstance::new(handle);
    instance.color = [0.0, 0.0, 1.0, 1.0];
    scene.meshes.push(instance.clone());
    let pixels = render_with(&gpu, &mut renderer, &scene);
    let centre = pixel(&pixels, SIZE[0] / 2, SIZE[1] / 2);
    assert!(
        centre[0] > 150 && centre[2] < 60,
        "centre pixel {centre:?} should be the vertex red, not the instance blue"
    );

    // A highlight is painted over the data as it is over a plain body. Face id 2 is the
    // -Y face, which the Front view looks straight at.
    instance.highlight_faces = vec![2];
    instance.highlight_color = [0.0, 1.0, 0.0, 1.0];
    scene.meshes[0] = instance;
    let pixels = render_with(&gpu, &mut renderer, &scene);
    let centre = pixel(&pixels, SIZE[0] / 2, SIZE[1] / 2);
    assert!(
        centre[1] > 150 && centre[0] < 60,
        "centre pixel {centre:?} should be the highlight green"
    );

    // The wrong number of colours is refused rather than read off the end.
    assert!(matches!(
        renderer.upload_colored_mesh(&gpu.device, &gpu.queue, &mesh, &colors[1..], &[]),
        Err(basset_viewport::ViewportError::ColorCountMismatch { .. })
    ));
}

#[test]
fn highlighted_face_changes_colour() {
    let Some(gpu) = gpu() else { return };
    let mut renderer = Renderer::new(&gpu.device, FORMAT, 1);
    let handle = renderer
        .upload_mesh(&gpu.device, &gpu.queue, &cube(20.0), &cube_edges(20.0))
        .expect("valid cube");
    let camera = looking_at_origin();
    let mut scene = Scene::new(&camera);
    scene.background = BACKGROUND;
    scene.show_grid = false;
    scene.meshes.push(MeshInstance::new(handle));
    let plain = pixel(
        &render_with(&gpu, &mut renderer, &scene),
        SIZE[0] / 2,
        SIZE[1] / 2,
    );

    // Face id 2 is the -Y face, which the Front view looks straight at.
    scene.meshes[0].highlight_faces = vec![2];
    scene.meshes[0].highlight_color = [1.0, 0.1, 0.1, 1.0];
    let lit = pixel(
        &render_with(&gpu, &mut renderer, &scene),
        SIZE[0] / 2,
        SIZE[1] / 2,
    );
    assert!(
        lit[0] > lit[2] + 60,
        "highlighted pixel {lit:?} should be red, plain was {plain:?}"
    );

    // Highlighting a different face leaves the front face alone.
    scene.meshes[0].highlight_faces = vec![3];
    let other = pixel(
        &render_with(&gpu, &mut renderer, &scene),
        SIZE[0] / 2,
        SIZE[1] / 2,
    );
    assert_eq!(other, plain);
}

#[test]
fn line_batch_draws_pixels_and_overlays_geometry() {
    let Some(gpu) = gpu() else { return };
    let camera = looking_at_origin();
    let mut scene = Scene::new(&camera);
    scene.background = BACKGROUND;
    scene.show_grid = false;
    // Horizontal line through the target, spanning the whole view (X is screen-right in Front).
    let mut lines = LineBatch::new([1.0, 1.0, 0.0, 1.0]);
    lines
        .segments
        .push([Vec3::new(-100.0, 0.0, 0.0), Vec3::new(100.0, 0.0, 0.0)]);
    lines.width_px = 3.0;
    lines.depth_test = false;
    scene.lines.push(lines);

    let pixels = render_to_pixels(&gpu, 4, &scene);
    let on_line = pixel(&pixels, SIZE[0] / 4, SIZE[1] / 2);
    assert!(
        on_line[0] > 200 && on_line[1] > 200 && on_line[2] < 60,
        "expected yellow on the line, got {on_line:?}"
    );
    let off_line = pixel(&pixels, SIZE[0] / 4, SIZE[1] / 2 - 10);
    assert!(is_background(off_line), "off-line pixel {off_line:?}");

    // Same line with depth test off still shows through an opaque cube in front of it.
    let mut renderer = Renderer::new(&gpu.device, FORMAT, 1);
    let handle = renderer
        .upload_mesh(&gpu.device, &gpu.queue, &cube(20.0), &cube_edges(20.0))
        .expect("valid cube");
    scene.meshes.push(MeshInstance {
        style: MeshStyle::ShadedWithEdges,
        ..MeshInstance::new(handle)
    });
    let pixels = render_with(&gpu, &mut renderer, &scene);
    let through_cube = pixel(&pixels, SIZE[0] / 2, SIZE[1] / 2);
    assert!(
        through_cube[0] > 200 && through_cube[2] < 60,
        "overlay line hidden by cube: {through_cube:?}"
    );
}

#[test]
fn depth_tested_line_is_hidden_by_geometry() {
    let Some(gpu) = gpu() else { return };
    let mut renderer = Renderer::new(&gpu.device, FORMAT, 1);
    let handle = renderer
        .upload_mesh(&gpu.device, &gpu.queue, &cube(20.0), &cube_edges(20.0))
        .expect("valid cube");
    let camera = looking_at_origin();
    let mut scene = Scene::new(&camera);
    scene.background = BACKGROUND;
    scene.show_grid = false;
    scene.meshes.push(MeshInstance::new(handle));
    let mut lines = LineBatch::new([1.0, 1.0, 0.0, 1.0]);
    // Behind the cube (Front view looks along +Y, so +Y is farther away).
    lines
        .segments
        .push([Vec3::new(-100.0, 50.0, 0.0), Vec3::new(100.0, 50.0, 0.0)]);
    lines.depth_test = true;
    scene.lines.push(lines);
    let pixels = render_with(&gpu, &mut renderer, &scene);
    let behind_cube = pixel(&pixels, SIZE[0] / 2, SIZE[1] / 2);
    assert!(
        behind_cube[0].abs_diff(behind_cube[2]) < 40,
        "line should be occluded: {behind_cube:?}"
    );
    let beside_cube = pixel(&pixels, 4, SIZE[1] / 2);
    assert!(
        beside_cube[0] > 200 && beside_cube[2] < 60,
        "line visible beside cube: {beside_cube:?}"
    );
}

#[test]
fn points_and_grid_render_without_errors() {
    let Some(gpu) = gpu() else { return };
    let mut camera = Camera::new_default();
    camera.distance = 100.0;
    let mut scene = Scene::new(&camera);
    scene.background = BACKGROUND;
    scene.show_grid = true;
    let mut points = PointBatch::new([1.0, 0.0, 1.0, 1.0]);
    points.points.push(Vec3::ZERO);
    points.size_px = 9.0;
    scene.points.push(points);
    let mut dashed = LineBatch::new([1.0, 1.0, 1.0, 1.0]);
    dashed.dashed = true;
    dashed
        .segments
        .push([Vec3::new(-50.0, 0.0, 10.0), Vec3::new(50.0, 0.0, 10.0)]);
    scene.lines.push(dashed);

    let pixels = render_to_pixels(&gpu, 4, &scene);
    let centre = pixel(&pixels, SIZE[0] / 2, SIZE[1] / 2);
    assert!(
        centre[0] > 200 && centre[2] > 200 && centre[1] < 60,
        "point marker at target: {centre:?}"
    );
    assert!(pixels.iter().any(|p| !is_background(*p)));
}

#[test]
fn resize_and_ghost_instances_survive_frames() {
    let Some(gpu) = gpu() else { return };
    let mut renderer = Renderer::new(&gpu.device, FORMAT, 4);
    renderer.resize(&gpu.device, [64, 64]);
    let handle = renderer
        .upload_mesh(&gpu.device, &gpu.queue, &cube(20.0), &cube_edges(20.0))
        .expect("valid cube");
    let camera = looking_at_origin();
    let mut scene = Scene::new(&camera);
    scene.background = BACKGROUND;
    scene.show_grid = false;
    scene.meshes.push(MeshInstance {
        style: MeshStyle::Ghost,
        ..MeshInstance::new(handle)
    });
    scene.meshes.push(MeshInstance {
        highlight_faces: vec![0, 1, 2],
        ..MeshInstance::new(handle)
    });
    let first = render_with(&gpu, &mut renderer, &scene);
    renderer.remove_mesh(handle);
    let second = render_with(&gpu, &mut renderer, &scene);
    assert!(!is_background(pixel(&first, SIZE[0] / 2, SIZE[1] / 2)));
    assert!(
        is_background(pixel(&second, SIZE[0] / 2, SIZE[1] / 2)),
        "removed mesh must not draw: {:?}",
        pixel(&second, SIZE[0] / 2, SIZE[1] / 2)
    );
}

/// A small red cube inside a big grey one: as a ghost it is hidden by the grey cube's
/// near face, as an overlay it tints the centre red through it. This is what lets a
/// comparison show the old top of a block inside the taller block that replaced it.
#[test]
fn an_overlay_shows_through_the_body_in_front_of_it_and_a_ghost_does_not() {
    let Some(gpu) = gpu() else { return };
    let mut renderer = Renderer::new(&gpu.device, FORMAT, 1);
    renderer.resize(&gpu.device, SIZE);
    let outer = renderer
        .upload_mesh(&gpu.device, &gpu.queue, &cube(20.0), &[])
        .expect("valid cube");
    let inner = renderer
        .upload_mesh(&gpu.device, &gpu.queue, &cube(8.0), &[])
        .expect("valid cube");
    let camera = looking_at_origin();
    let centre_with = |renderer: &mut Renderer, style: Option<MeshStyle>| {
        let mut scene = Scene::new(&camera);
        scene.background = BACKGROUND;
        scene.show_grid = false;
        scene.meshes.push(MeshInstance::new(outer));
        if let Some(style) = style {
            scene.meshes.push(MeshInstance {
                style,
                color: [1.0, 0.0, 0.0, 1.0],
                ..MeshInstance::new(inner)
            });
        }
        let pixels = render_with(&gpu, renderer, &scene);
        pixel(&pixels, SIZE[0] / 2, SIZE[1] / 2)
    };
    let plain = centre_with(&mut renderer, None);
    let ghost = centre_with(&mut renderer, Some(MeshStyle::Ghost));
    let overlay = centre_with(&mut renderer, Some(MeshStyle::Overlay));
    assert_eq!(
        ghost, plain,
        "a ghost inside the cube is behind its near face and is hidden by it"
    );
    assert!(
        overlay[0] > plain[0] + 20 && overlay[1] < plain[1],
        "the overlay tints the centre red through the cube: {plain:?} -> {overlay:?}"
    );
    assert!(
        overlay[1] > 10,
        "but only tints: the grey face is still there under it: {overlay:?}"
    );
}

#[test]
fn tri_batch_fills_a_region_and_lets_the_background_through() {
    let Some(gpu) = gpu() else { return };
    let camera = looking_at_origin();
    let mut scene = Scene::new(&camera);
    scene.background = BACKGROUND;
    scene.show_grid = false;
    // A square facing the Front view, covering the centre but not the corners.
    let mut fill = TriBatch::new([1.0, 0.0, 0.0, 0.5]);
    let q = [
        Vec3::new(-10.0, 0.0, -10.0),
        Vec3::new(10.0, 0.0, -10.0),
        Vec3::new(10.0, 0.0, 10.0),
        Vec3::new(-10.0, 0.0, 10.0),
    ];
    fill.triangles.push([q[0], q[1], q[2]]);
    fill.triangles.push([q[0], q[2], q[3]]);
    scene.tris.push(fill);

    let pixels = render_to_pixels(&gpu, 1, &scene);
    let centre = pixel(&pixels, SIZE[0] / 2, SIZE[1] / 2);
    assert!(!is_background(centre), "the region is filled: {centre:?}");
    assert!(
        centre[0] > centre[2],
        "the fill is red over a blue background: {centre:?}"
    );
    assert!(
        centre[2] > 20,
        "half alpha keeps the background visible through it: {centre:?}"
    );
    assert!(is_background(pixel(&pixels, 1, 1)), "corner is untouched");
}

/// A dashed curve has to look dashed at the size it is drawn.
///
/// The dash pattern is measured in pixels along a segment, and a tessellated curve is
/// made of segments a few pixels long — shorter than one dash. A pattern that restarted
/// at every segment therefore put every one of them inside a dash and drew the whole
/// curve solid, which is exactly how construction geometry stopped being distinguishable
/// from ordinary geometry on anything small. This draws one such curve as a run of short
/// segments and insists there are gaps in it.
#[test]
fn a_dashed_run_of_short_segments_still_shows_gaps() {
    let Some(gpu) = gpu() else { return };
    let camera = looking_at_origin();
    let row = SIZE[1] / 2;

    // A straight line across the middle of the view, cut into pieces each well under one
    // dash long on screen, the way a small circle's tessellation is.
    let span = 40.0;
    let pieces = 64;
    let point = |i: u32| {
        Vec3::new(
            -span * 0.5 + span * f64::from(i) / f64::from(pieces),
            0.0,
            0.0,
        )
    };
    let mut scene = Scene::new(&camera);
    scene.background = BACKGROUND;
    scene.show_grid = false;
    let mut dashed = LineBatch::new([1.0, 1.0, 1.0, 1.0]);
    dashed.dashed = true;
    dashed.depth_test = false;
    for i in 0..pieces {
        dashed.segments.push([point(i), point(i + 1)]);
    }
    scene.lines.push(dashed);

    let pixels = render_to_pixels(&gpu, 1, &scene);
    let along: Vec<bool> = (0..SIZE[0])
        .map(|x| !is_background(pixel(&pixels, x, row)))
        .collect();
    let drawn = along.iter().filter(|on| **on).count();
    let gaps = along.iter().filter(|on| !**on).count();
    assert!(drawn > 0, "the line is drawn at all");
    assert!(
        gaps > 0 && along.windows(2).filter(|w| w[0] != w[1]).count() >= 4,
        "and it alternates between ink and gap rather than running solid: {along:?}"
    );
}

#[test]
fn wireframe_draws_the_edges_and_leaves_the_faces_out() {
    let Some(gpu) = gpu() else { return };
    let mut renderer = Renderer::new(&gpu.device, FORMAT, 1);
    let handle = renderer
        .upload_mesh(&gpu.device, &gpu.queue, &cube(20.0), &cube_edges(20.0))
        .expect("valid cube");
    let camera = looking_at_origin();
    let mut scene = Scene::new(&camera);
    scene.background = BACKGROUND;
    scene.show_grid = false;
    scene.meshes.push(MeshInstance {
        style: MeshStyle::Wireframe,
        edge_color: [1.0, 1.0, 0.0, 1.0],
        ..MeshInstance::new(handle)
    });

    let pixels = render_with(&gpu, &mut renderer, &scene);
    // The centre of a cube seen face-on is inside a face and crossed by no edge, so in
    // wireframe it must show the background the shaded modes cover up.
    let centre = pixel(&pixels, SIZE[0] / 2, SIZE[1] / 2);
    assert!(
        is_background(centre),
        "wireframe must not fill the face: {centre:?}"
    );
    let edge_pixels = pixels
        .iter()
        .filter(|p| p[0] > 200 && p[1] > 200 && p[2] < 60)
        .count();
    assert!(edge_pixels > 0, "no edges drawn at all");
}

#[test]
fn xray_blends_the_body_with_what_is_behind_it() {
    let Some(gpu) = gpu() else { return };
    let mut renderer = Renderer::new(&gpu.device, FORMAT, 1);
    let handle = renderer
        .upload_mesh(&gpu.device, &gpu.queue, &cube(20.0), &cube_edges(20.0))
        .expect("valid cube");
    let camera = looking_at_origin();
    let mut scene = Scene::new(&camera);
    scene.background = BACKGROUND;
    scene.show_grid = false;
    scene.meshes.push(MeshInstance::new(handle));
    let opaque = pixel(
        &render_with(&gpu, &mut renderer, &scene),
        SIZE[0] / 2,
        SIZE[1] / 2,
    );

    scene.meshes[0].style = MeshStyle::XRay;
    let xray = pixel(
        &render_with(&gpu, &mut renderer, &scene),
        SIZE[0] / 2,
        SIZE[1] / 2,
    );
    assert!(!is_background(xray), "the body still shades: {xray:?}");
    // Letting what is behind through means every channel lands between the shaded body
    // and the background, rather than at either end.
    let background = BACKGROUND.map(srgb_encode);
    for c in 0..3 {
        let (lo, hi) = (background[c].min(opaque[c]), background[c].max(opaque[c]));
        assert!(
            xray[c] > lo && xray[c] < hi,
            "channel {c} of x-ray {xray:?} is not between the background {background:?} \
             and the shaded body {opaque:?}"
        );
    }
}

/// The problem the silhouette exists for: a cylinder standing against the background has
/// no feature edge down its side, so without one it is bounded only by its shading. With
/// it, the pixels where the wall turns away darken to the edge colour.
#[test]
fn a_cylinder_is_bounded_against_the_background() {
    let Some(gpu) = gpu() else { return };
    let solid = basset_kernel::primitives::cylinder(
        basset_kernel::OpId::new(1),
        Vec3::new(0.0, 0.0, -20.0),
        Vec3::Z,
        20.0,
        40.0,
        &basset_kernel::Tessellation::default(),
    );
    let mut renderer = Renderer::new(&gpu.device, FORMAT, 1);
    let handle = renderer
        .upload_mesh(
            &gpu.device,
            &gpu.queue,
            &solid.tessellate().mesh,
            &solid.display_edges(),
        )
        .expect("a valid cylinder");
    let camera = looking_at_origin();
    let mut scene = Scene::new(&camera);
    scene.background = BACKGROUND;
    scene.show_grid = false;
    scene.meshes.push(MeshInstance::new(handle));

    // Half way up the wall, where neither rim is in the way.
    let row = SIZE[1] / 2;
    let darkest_at_the_boundary = |pixels: &[[u8; 4]]| {
        let first = (0..SIZE[0])
            .find(|&x| !is_background(pixel(pixels, x, row)))
            .expect("the cylinder is somewhere in the row");
        (first..first + 3)
            .map(|x| pixel(pixels, x, row)[1])
            .min()
            .expect("three pixels")
    };
    let plain = darkest_at_the_boundary(&render_with(&gpu, &mut renderer, &scene));
    scene.meshes[0].style = MeshStyle::ShadedWithEdges;
    let outlined = darkest_at_the_boundary(&render_with(&gpu, &mut renderer, &scene));
    assert!(
        outlined + 30 < plain,
        "the wall's boundary should darken to the edge colour: {outlined} against {plain}"
    );
}

/// A sky brighter above than below, with one soft box over the camera's shoulder.
fn environment(sky_background: bool) -> Lighting {
    Lighting::Environment(EnvironmentLight {
        zenith: [0.9, 0.9, 1.0],
        horizon: [0.5, 0.5, 0.5],
        nadir: [0.02, 0.02, 0.02],
        lights: vec![DistantLight {
            // Front view looks along +Y, so the camera is towards -Y.
            direction: Vec3::new(-0.3, -1.0, 0.4).normalize(),
            angular_radius: 0.2,
            radiance: [8.0, 8.0, 8.0],
        }],
        exposure: 1.0,
        sky_background,
    })
}

fn brightness(p: [u8; 4]) -> u32 {
    u32::from(p[0]) + u32::from(p[1]) + u32::from(p[2])
}

/// One body in two appearances: two instances of one mesh, each masked to its own faces.
/// The front face shows the colour of the instance that owns it, and an instance whose
/// faces are all out of sight, or which owns none, draws nothing.
#[test]
fn a_face_mask_draws_only_its_faces() {
    let Some(gpu) = gpu() else { return };
    let mut renderer = Renderer::new(&gpu.device, FORMAT, 1);
    let handle = renderer
        .upload_mesh(&gpu.device, &gpu.queue, &cube(20.0), &cube_edges(20.0))
        .expect("valid cube");
    let camera = looking_at_origin();
    let centre_with = |renderer: &mut Renderer, masks: &[(Vec<u32>, [f32; 4])]| {
        let mut scene = Scene::new(&camera);
        scene.background = BACKGROUND;
        scene.show_grid = false;
        scene.lighting = environment(false);
        for (faces, color) in masks {
            scene.meshes.push(MeshInstance {
                face_mask: Some(faces.clone()),
                color: *color,
                ..MeshInstance::new(handle)
            });
        }
        let pixels = render_with(&gpu, renderer, &scene);
        pixel(&pixels, SIZE[0] / 2, SIZE[1] / 2)
    };
    let red = [0.8, 0.05, 0.05, 1.0];
    let blue = [0.05, 0.05, 0.8, 1.0];

    // Face id 2 is the -Y face, which the Front view looks straight at.
    let front_red = centre_with(
        &mut renderer,
        &[(vec![2], red), (vec![0, 1, 3, 4, 5], blue)],
    );
    assert!(
        front_red[0] > front_red[2] + 60,
        "the front face belongs to the red instance: {front_red:?}"
    );
    let front_blue = centre_with(
        &mut renderer,
        &[(vec![0, 1, 3, 4, 5], red), (vec![2], blue)],
    );
    assert!(
        front_blue[2] > front_blue[0] + 60,
        "and to the blue one when the masks swap: {front_blue:?}"
    );
    // The top and bottom are edge-on to the Front view, so with only them let through
    // nothing covers the centre.
    let top_and_bottom = centre_with(&mut renderer, &[(vec![0, 1], red)]);
    assert!(
        is_background(top_and_bottom),
        "faces outside the mask are not drawn: {top_and_bottom:?}"
    );
    let nothing = centre_with(&mut renderer, &[(vec![], red)]);
    assert!(
        is_background(nothing),
        "an empty mask draws no faces: {nothing:?}"
    );
}

#[test]
fn the_sky_background_is_brighter_above_the_horizon_than_below() {
    let Some(gpu) = gpu() else { return };
    let mut camera = looking_at_origin();
    for orthographic in [false, true] {
        if orthographic {
            camera.set_orthographic();
        }
        let mut scene = Scene::new(&camera);
        scene.background = BACKGROUND;
        scene.show_grid = false;
        scene.lighting = environment(true);
        let pixels = render_to_pixels(&gpu, 1, &scene);
        let top = pixel(&pixels, SIZE[0] / 2, 1);
        let middle = pixel(&pixels, SIZE[0] / 2, SIZE[1] / 2);
        let bottom = pixel(&pixels, SIZE[0] / 2, SIZE[1] - 2);
        assert!(!is_background(middle), "the sky replaces the clear colour");
        if orthographic {
            // Every ray of an orthographic view is the view direction, level here, so
            // the whole background is the horizon. The tolerance is for the unprojection
            // in f32, which the square root in the sky amplifies just off the horizon.
            for (a, b) in [(top, middle), (bottom, middle)] {
                assert!(
                    a.iter().zip(&b).all(|(x, y)| x.abs_diff(*y) <= 2),
                    "orthographic rays are parallel: {top:?} {middle:?} {bottom:?}"
                );
            }
        } else {
            assert!(
                brightness(top) > brightness(middle) && brightness(middle) > brightness(bottom),
                "zenith over horizon over nadir: {top:?} {middle:?} {bottom:?}"
            );
            // Neutral sky, neutral pixels: the tone map works per channel alike.
            assert!(middle[0].abs_diff(middle[2]) <= 1, "{middle:?}");
        }
    }

    // Without the flag the environment still lights meshes but the clear colour stays.
    let mut scene = Scene::new(&camera);
    scene.background = BACKGROUND;
    scene.show_grid = false;
    scene.lighting = environment(false);
    let pixels = render_to_pixels(&gpu, 1, &scene);
    assert!(is_background(pixel(&pixels, SIZE[0] / 2, 1)));
}

/// Glass is as see-through as its colour says. A translucent cube lets the background
/// through, as a ghost does, but by its own alpha rather than a ghost's fixed fade, so at
/// alpha 0.8 it hides more of what is behind it than the ghost of the same colour.
#[test]
fn a_translucent_body_blends_by_its_own_alpha() {
    let Some(gpu) = gpu() else { return };
    let mut renderer = Renderer::new(&gpu.device, FORMAT, 1);
    let handle = renderer
        .upload_mesh(&gpu.device, &gpu.queue, &cube(20.0), &cube_edges(20.0))
        .expect("valid cube");
    let camera = looking_at_origin();
    let centre_with = |renderer: &mut Renderer, style: MeshStyle| {
        let mut scene = Scene::new(&camera);
        scene.background = BACKGROUND;
        scene.show_grid = false;
        scene.lighting = environment(false);
        scene.meshes.push(MeshInstance {
            style,
            color: [0.9, 0.9, 0.1, 0.8],
            ..MeshInstance::new(handle)
        });
        let pixels = render_with(&gpu, renderer, &scene);
        pixel(&pixels, SIZE[0] / 2, SIZE[1] / 2)
    };
    let opaque = centre_with(&mut renderer, MeshStyle::Shaded);
    let translucent = centre_with(&mut renderer, MeshStyle::Translucent);
    let ghost = centre_with(&mut renderer, MeshStyle::Ghost);
    assert!(
        !is_background(translucent),
        "the body shades: {translucent:?}"
    );
    // Blue is where the background and the yellow body differ most.
    let background_blue = srgb_encode(BACKGROUND[2]);
    assert!(
        translucent[2] > opaque[2] && translucent[2] < background_blue,
        "the background shows through: {opaque:?} < {translucent:?} < {background_blue}"
    );
    assert!(
        translucent[2] < ghost[2],
        "but less than through a ghost: {translucent:?} against {ghost:?}"
    );
}

/// A UV sphere of radius `r` with smooth normals, all one face.
fn sphere(r: f64) -> TriMesh {
    let (rings, segments) = (48u32, 96u32);
    let point = |i: u32, j: u32| {
        let theta = std::f64::consts::PI * f64::from(i) / f64::from(rings);
        let phi = 2.0 * std::f64::consts::PI * f64::from(j) / f64::from(segments);
        Vec3::new(
            theta.sin() * phi.cos(),
            theta.sin() * phi.sin(),
            theta.cos(),
        )
    };
    let mut mesh = TriMesh::default();
    for i in 0..=rings {
        for j in 0..=segments {
            let n = point(i, j);
            mesh.positions.push(n * r);
            mesh.normals.push(n);
        }
    }
    let index = |i: u32, j: u32| i * (segments + 1) + j;
    for i in 0..rings {
        for j in 0..segments {
            let (a, b, c, d) = (
                index(i, j),
                index(i + 1, j),
                index(i + 1, j + 1),
                index(i, j + 1),
            );
            mesh.indices.extend([a, b, c, a, c, d]);
            mesh.face_ids.extend([0, 0]);
        }
    }
    mesh
}

/// What the soft-box widening is for: a polished metal ball shows the box as a bright,
/// soft-edged patch, not a single hot pixel and not nothing, and a matte one of the same
/// colour shows no such patch.
#[test]
fn a_soft_box_makes_a_soft_highlight_on_polished_metal() {
    let Some(gpu) = gpu() else { return };
    let mut renderer = Renderer::new(&gpu.device, FORMAT, 1);
    let handle = renderer
        .upload_mesh(&gpu.device, &gpu.queue, &sphere(15.0), &[])
        .expect("valid sphere");
    let camera = looking_at_origin();
    let mut scene = Scene::new(&camera);
    scene.background = BACKGROUND;
    scene.show_grid = false;
    scene.lighting = Lighting::Environment(EnvironmentLight {
        zenith: [0.05; 3],
        horizon: [0.05; 3],
        nadir: [0.05; 3],
        lights: vec![DistantLight {
            // Straight behind the camera: the highlight sits in the middle of the ball.
            direction: -Vec3::Y,
            angular_radius: 0.25,
            radiance: [4.0; 3],
        }],
        exposure: 1.0,
        sky_background: false,
    });
    let mut highlight_size = |renderer: &mut Renderer, roughness: f32| {
        scene.meshes = vec![MeshInstance {
            color: [0.9, 0.6, 0.3, 1.0],
            material: Material {
                metallic: 1.0,
                roughness,
                ..Material::DEFAULT
            },
            ..MeshInstance::new(handle)
        }];
        let pixels = render_with(&gpu, renderer, &scene);
        let row = SIZE[1] / 2;
        let lit: Vec<bool> = (0..SIZE[0])
            .map(|x| brightness(pixel(&pixels, x, row)) > 400)
            .collect();
        let centre = pixel(&pixels, SIZE[0] / 2, row);
        let rim = pixel(&pixels, SIZE[0] / 2 + 14, row);
        (lit.iter().filter(|on| **on).count(), centre, rim)
    };
    let (polished, centre, rim) = highlight_size(&mut renderer, 0.05);
    assert!(
        polished >= 3,
        "the soft box shows as a patch several pixels wide, not a point: {polished} \
         ({centre:?})"
    );
    assert!(
        brightness(rim) + 150 < brightness(centre),
        "and the rest of the ball reflects the dark sky: {rim:?} against {centre:?}"
    );
    let (matte, ..) = highlight_size(&mut renderer, 1.0);
    assert!(
        matte < polished,
        "a matte ball spreads the light out: {matte} against {polished}"
    );
}

/// The Studio lighting's highlight is shaped by the material, but the default material is
/// the look every body has always had.
#[test]
fn the_material_shapes_the_studio_highlight() {
    let Some(gpu) = gpu() else { return };
    let mut renderer = Renderer::new(&gpu.device, FORMAT, 1);
    let handle = renderer
        .upload_mesh(&gpu.device, &gpu.queue, &sphere(15.0), &[])
        .expect("valid sphere");
    let camera = looking_at_origin();
    let mut render = |material: Material| {
        let mut scene = Scene::new(&camera);
        scene.background = BACKGROUND;
        scene.show_grid = false;
        scene.meshes.push(MeshInstance {
            material,
            ..MeshInstance::new(handle)
        });
        render_with(&gpu, &mut renderer, &scene)
    };
    let plain = render(Material::DEFAULT);
    let glowing = render(Material {
        emission: [0.0, 0.5, 0.0],
        ..Material::DEFAULT
    });
    let centre = |pixels: &[[u8; 4]]| pixel(pixels, SIZE[0] / 2, SIZE[1] / 2);
    assert!(
        centre(&glowing)[1] > centre(&plain)[1] + 20,
        "emission is added: {:?} against {:?}",
        centre(&glowing),
        centre(&plain)
    );
    let metal = render(Material {
        metallic: 1.0,
        ..Material::DEFAULT
    });
    assert!(
        brightness(centre(&metal)) < brightness(centre(&plain)),
        "a metal gives up its diffuse colour: {:?} against {:?}",
        centre(&metal),
        centre(&plain)
    );
}
