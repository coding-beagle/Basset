//! Headless GPU tests. They need a real adapter (Vulkan, GL, ...) and quietly skip when
//! none exists so CI without a GPU still passes.

use basset_math::{TriMesh, Vec3};
use basset_viewport::{
    Camera, LineBatch, MeshInstance, MeshStyle, PointBatch, Renderer, Scene, TriBatch, ViewPreset,
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
