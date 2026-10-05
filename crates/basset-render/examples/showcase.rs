//! Renders a row of spheres and blocks in library appearances to a PNG, for judging how
//! the environments and finishes look without opening the editor.
//!
//! `cargo run --release -p basset-render --example showcase -- out.png [environment] [samples]`

use std::f64::consts::PI;

use basset_math::{TriMesh, Vec3};
use basset_render::{
    EnvironmentKind, RenderCamera, RenderOptions, SceneBuilder, SceneSettings, find, render,
};

fn sphere(center: Vec3, radius: f64, face: u32, mesh: &mut TriMesh) {
    let (rings, segments) = (48, 96);
    let base = mesh.positions.len() as u32;
    for i in 0..=rings {
        let theta = PI * i as f64 / rings as f64;
        for j in 0..=segments {
            let phi = 2.0 * PI * j as f64 / segments as f64;
            let n = Vec3::new(
                theta.sin() * phi.cos(),
                theta.sin() * phi.sin(),
                theta.cos(),
            );
            mesh.positions.push(center + n * radius);
            mesh.normals.push(n);
        }
    }
    let row = segments + 1;
    for i in 0..rings {
        for j in 0..segments {
            let a = base + (i * row + j) as u32;
            let b = a + row as u32;
            for tri in [[a, b, a + 1], [a + 1, b, b + 1]] {
                mesh.indices.extend(tri);
                mesh.face_ids.push(face);
            }
        }
    }
}

fn block(min: Vec3, max: Vec3, face: u32, mesh: &mut TriMesh) {
    let c = |x: bool, y: bool, z: bool| {
        Vec3::new(
            if x { max.x } else { min.x },
            if y { max.y } else { min.y },
            if z { max.z } else { min.z },
        )
    };
    let quads = [
        [
            c(false, false, false),
            c(false, true, false),
            c(true, true, false),
            c(true, false, false),
        ],
        [
            c(false, false, true),
            c(true, false, true),
            c(true, true, true),
            c(false, true, true),
        ],
        [
            c(false, false, false),
            c(true, false, false),
            c(true, false, true),
            c(false, false, true),
        ],
        [
            c(false, true, false),
            c(false, true, true),
            c(true, true, true),
            c(true, true, false),
        ],
        [
            c(false, false, false),
            c(false, false, true),
            c(false, true, true),
            c(false, true, false),
        ],
        [
            c(true, false, false),
            c(true, true, false),
            c(true, true, true),
            c(true, false, true),
        ],
    ];
    for q in quads {
        let n = (q[1] - q[0]).cross(q[2] - q[0]).normalize();
        for tri in [[q[0], q[1], q[2]], [q[0], q[2], q[3]]] {
            let b = mesh.positions.len() as u32;
            mesh.positions.extend(tri);
            mesh.normals.extend([n; 3]);
            mesh.indices.extend([b, b + 1, b + 2]);
            mesh.face_ids.push(face);
        }
    }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let out = args.get(1).map(String::as_str).unwrap_or("showcase.png");
    let environment = args
        .get(2)
        .and_then(|n| EnvironmentKind::from_name(n))
        .unwrap_or_default();
    let samples = args.get(3).and_then(|s| s.parse().ok()).unwrap_or(128);

    let spheres = [
        "Chrome",
        "Gold - Polished",
        "Aluminium - Brushed",
        "Paint - Gloss Red",
        "Glass - Clear",
        "Plastic - Matte White",
    ];
    let blocks = [
        "Wood - Walnut Varnished",
        "Carbon Fibre - Twill",
        "Aluminium - Anodized Blue",
        "Rubber - Black",
        "Copper - Satin",
        "Granite - Black",
    ];
    let mut builder = SceneBuilder::new();
    let mut ids = Vec::new();
    for name in spheres.iter().chain(&blocks) {
        let a = find(name).unwrap_or_else(|| panic!("{name} is not in the library"));
        ids.push(builder.add_appearance(a));
    }
    let mut mesh = TriMesh::default();
    for (i, _) in spheres.iter().enumerate() {
        sphere(
            Vec3::new(i as f64 * 26.0, 0.0, 10.0),
            10.0,
            i as u32,
            &mut mesh,
        );
    }
    for (i, _) in blocks.iter().enumerate() {
        let x = i as f64 * 26.0;
        block(
            Vec3::new(x - 10.0, 22.0, 0.0),
            Vec3::new(x + 10.0, 42.0, 14.0),
            (spheres.len() + i) as u32,
            &mut mesh,
        );
    }
    builder.add_mesh(&mesh, |face| ids[face as usize]);
    let settings = SceneSettings {
        environment,
        ..SceneSettings::default()
    };
    let scene = builder.build(&settings);
    let options = RenderOptions {
        width: 1200,
        height: 560,
        samples,
        ..RenderOptions::default()
    };
    let camera = RenderCamera::fit(
        &scene.bounds(),
        Vec3::new(0.25, -1.0, 0.55),
        30f64.to_radians(),
        f64::from(options.width) / f64::from(options.height),
    );
    let started = std::time::Instant::now();
    let image = render(&scene, &camera, &options);
    image.save_png(out).expect("writing the image");
    println!(
        "{} ({} samples, {} triangles) in {:.1} s",
        out,
        samples,
        scene.triangle_count(),
        started.elapsed().as_secs_f64()
    );
}
