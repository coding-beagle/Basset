//! What the tracer traces: every triangle of every body in world space with the
//! appearance it was given, the lighting, and the floor.
//!
//! A [`TraceScene`] knows nothing of documents or bodies. The caller hands
//! [`SceneBuilder`] meshes and says which appearance each kernel face takes, exactly as
//! the FEA crate takes a solid and face keys, so the same scene can be built by the
//! editor from the meshes it already has on screen and by the MCP server from a fresh
//! tessellation.

use basset_math::{Aabb, TriMesh, Vec3};

use crate::appearance::{Appearance, Pattern};
use crate::bvh::{Bvh, TraceRay, Triangle};
use crate::color::{Rgb, Srgb, aces_inverse};
use crate::environment::Lighting;
use crate::settings::{Background, SceneSettings};

/// Per-triangle shading data, kept apart from the geometry the tree reads so a
/// traversal never drags normals through the cache.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Corner {
    pub normals: [Vec3; 3],
    pub material: u32,
}

/// An appearance resolved for shading: linear colours, roughness squared.
#[derive(Clone, Debug)]
pub(crate) struct Material {
    pub color: Rgb,
    pub color2: Rgb,
    pub metallic: f64,
    pub roughness: f64,
    pub transmission: f64,
    pub ior: f64,
    pub clearcoat: f64,
    pub coat_roughness: f64,
    pub emission: Rgb,
    pub pattern: Pattern,
    pub pattern_scale: f64,
}

impl Material {
    pub fn new(a: &Appearance) -> Self {
        let a = a.clone().sanitised();
        let color = a.color.to_linear();
        Self {
            color,
            color2: a.color2.to_linear(),
            metallic: f64::from(a.metallic),
            roughness: f64::from(a.roughness),
            transmission: f64::from(a.transmission),
            ior: f64::from(a.ior),
            clearcoat: f64::from(a.clearcoat),
            coat_roughness: 0.05,
            emission: color.map(|c| c * a.emission),
            pattern: a.pattern,
            pattern_scale: f64::from(a.pattern_scale),
        }
    }

    /// The surface at a point, with the pattern evaluated there.
    pub fn at(&self, p: Vec3, n: Vec3) -> Surface {
        let mut s = Surface {
            base: self.color.map(f64::from),
            metallic: self.metallic,
            roughness: self.roughness,
            transmission: self.transmission,
            ior: self.ior,
            clearcoat: self.clearcoat,
            coat_roughness: self.coat_roughness,
            emission: self.emission.map(f64::from),
        };
        let q = p / self.pattern_scale;
        let mix = |a: Rgb, b: Rgb, t: f64| -> [f64; 3] {
            [0, 1, 2].map(|i| f64::from(a[i]) + (f64::from(b[i]) - f64::from(a[i])) * t)
        };
        match self.pattern {
            Pattern::None => {}
            Pattern::Brushed => {
                // Streaks run along X: noise that varies quickly across them and hardly
                // at all along them, in two octaves so the streaks are not all one width.
                let n = 0.6 * noise(Vec3::new(q.x * 0.02, q.y * 6.0, q.z * 6.0))
                    + 0.4 * noise(Vec3::new(q.x * 0.05, q.y * 23.0, q.z * 23.0));
                s.roughness = (s.roughness * (0.55 + 0.9 * n)).clamp(0.02, 1.0);
                s.base = s.base.map(|c| c * (0.93 + 0.12 * n));
            }
            Pattern::Wood => {
                // Rings about the X axis, so a board lying along X shows long grain on
                // its faces and rings on its ends, as sawn timber does. The rings wander
                // with a low-frequency noise and the grain is fine noise stretched along
                // the axis.
                let wobble = fbm(Vec3::new(q.x * 0.08, q.y * 0.35, q.z * 0.35), 3);
                let r = (q.y * q.y + q.z * q.z).sqrt() + 1.6 * wobble;
                let ring = r.fract();
                let late = smoothstep(0.55, 0.85, ring) * (1.0 - smoothstep(0.9, 1.0, ring));
                let grain = noise(Vec3::new(q.x * 0.6, q.y * 18.0, q.z * 18.0));
                let t = (0.8 * late + 0.25 * grain).clamp(0.0, 1.0);
                s.base = mix(self.color, self.color2, t);
                s.roughness = (s.roughness + 0.1 * (grain - 0.5)).clamp(0.02, 1.0);
            }
            Pattern::CarbonFibre => {
                // A 2/2 twill on the plane the face most nearly lies in. Each tow is one
                // pattern period wide; along a tow its sheen rises and falls once, which
                // is what makes the weave catch the light in alternating blocks.
                let (u, v) = planar(q, n);
                let (iu, iv) = (u.floor() as i64, v.floor() as i64);
                let warp = (iu + iv).rem_euclid(4) < 2;
                let across = if warp { v.fract() } else { u.fract() };
                let sheen = (std::f64::consts::PI * across).sin();
                let t = if warp { sheen } else { 1.0 - sheen };
                s.base = mix(self.color, self.color2, 0.25 + 0.75 * t);
                s.roughness = (s.roughness * (0.8 + 0.5 * (1.0 - sheen))).clamp(0.02, 1.0);
            }
            Pattern::Speckle => {
                let n = noise(q * 3.0) * 0.65 + noise(q * 7.3) * 0.35;
                let t = smoothstep(0.58, 0.66, n);
                s.base = mix(self.color, self.color2, t);
            }
        }
        s
    }
}

/// A material evaluated at one point.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Surface {
    pub base: [f64; 3],
    pub metallic: f64,
    pub roughness: f64,
    pub transmission: f64,
    pub ior: f64,
    pub clearcoat: f64,
    pub coat_roughness: f64,
    pub emission: [f64; 3],
}

/// The floor: a horizontal plane under the model, at the bottom of its bounding box.
#[derive(Clone, Debug)]
pub(crate) struct Ground {
    pub z: f64,
    pub material: Material,
}

/// Everything a render needs except the camera.
#[derive(Clone, Debug)]
pub struct TraceScene {
    pub(crate) triangles: Vec<Triangle>,
    pub(crate) corners: Vec<Corner>,
    pub(crate) materials: Vec<Material>,
    pub(crate) bvh: Bvh,
    pub(crate) lighting: Lighting,
    pub(crate) ground: Option<Ground>,
    /// Linear radiance of a solid background, chosen so that it comes out of the tone
    /// curve as the colour the user picked.
    pub(crate) solid_background: Option<[f64; 3]>,
    pub(crate) bounds: Aabb,
    /// How far a secondary ray starts from the surface it leaves: a millionth of the
    /// scene, which clears the `f64` noise of a hit point at any model size.
    pub(crate) epsilon: f64,
    /// Light selection weights for next-event estimation, by irradiance.
    pub(crate) light_weights: Vec<f64>,
}

impl TraceScene {
    pub fn bounds(&self) -> Aabb {
        self.bounds
    }

    pub fn triangle_count(&self) -> usize {
        self.triangles.len()
    }

    pub fn lighting(&self) -> &Lighting {
        &self.lighting
    }
}

/// Collects meshes and appearances into a [`TraceScene`].
#[derive(Default)]
pub struct SceneBuilder {
    triangles: Vec<Triangle>,
    corners: Vec<Corner>,
    materials: Vec<Material>,
    bounds: Option<Aabb>,
}

impl SceneBuilder {
    pub fn new() -> Self {
        Self::default()
    }

    /// Registers an appearance and returns the index meshes refer to it by.
    pub fn add_appearance(&mut self, appearance: &Appearance) -> u32 {
        self.materials.push(Material::new(appearance));
        (self.materials.len() - 1) as u32
    }

    /// Adds every triangle of `mesh`, each with the appearance `material_of` gives its
    /// kernel face id. An index that was never registered falls back to the default
    /// appearance rather than failing, since the mesh is drawn either way.
    pub fn add_mesh(&mut self, mesh: &TriMesh, material_of: impl Fn(u32) -> u32) {
        let n = mesh.triangle_count();
        for t in 0..n {
            let idx = [0, 1, 2].map(|k| mesh.indices[t * 3 + k] as usize);
            let [a, b, c] = idx.map(|i| mesh.positions[i]);
            let tri = Triangle::new(a, b, c);
            let geometric = tri.e1.cross(tri.e2);
            if geometric.length_squared() <= 0.0 || !geometric.is_finite() {
                continue;
            }
            let normals = idx.map(|i| {
                let n = mesh.normals.get(i).copied().unwrap_or(geometric);
                if n.length_squared() > 0.0 && n.is_finite() {
                    n.normalize()
                } else {
                    geometric.normalize()
                }
            });
            let face = mesh.face_ids.get(t).copied().unwrap_or(0);
            self.triangles.push(tri);
            self.corners.push(Corner {
                normals,
                material: material_of(face),
            });
            let mut aabb = self.bounds.unwrap_or_else(Aabb::empty);
            for p in [a, b, c] {
                aabb.include(p);
            }
            self.bounds = Some(aabb);
        }
    }

    pub fn build(mut self, settings: &SceneSettings) -> TraceScene {
        if self.materials.is_empty() {
            self.materials.push(Material::new(&Appearance::DEFAULT));
        }
        let default = 0;
        let count = self.materials.len() as u32;
        for c in &mut self.corners {
            if c.material >= count {
                c.material = default;
            }
        }
        let bounds = self.bounds.unwrap_or(Aabb {
            min: Vec3::splat(-1.0),
            max: Vec3::splat(1.0),
        });
        let size = bounds.extent().length().max(1e-3);
        let lighting = settings.lighting();
        let ground = settings.ground_plane.then(|| {
            let albedo = Srgb::from_linear(lighting.ground);
            let floor = Appearance {
                color: albedo,
                roughness: 0.9,
                clearcoat: if settings.ground_reflections {
                    1.0
                } else {
                    0.0
                },
                ..Appearance::DEFAULT
            };
            let mut material = Material::new(&floor);
            material.coat_roughness = f64::from(settings.ground_roughness.clamp(0.0, 1.0));
            Ground {
                // A hair below the model, so a face lying on the floor does not fight it.
                z: bounds.min.z - size * 1e-5,
                material,
            }
        });
        let solid_background = match settings.background {
            Background::Environment => None,
            Background::Solid(c) => Some(c.to_linear().map(|v| f64::from(aces_inverse(v)))),
        };
        let light_weights = lighting
            .lights
            .iter()
            .map(|l| f64::from(crate::color::luminance(l.irradiance())).max(1e-9))
            .collect();
        let bvh = Bvh::build(&self.triangles);
        TraceScene {
            triangles: self.triangles,
            corners: self.corners,
            materials: self.materials,
            bvh,
            lighting,
            ground,
            solid_background,
            bounds,
            epsilon: size * 1e-6,
            light_weights,
        }
    }
}

/// What a ray meets first.
#[derive(Clone, Copy, Debug)]
pub(crate) enum Contact {
    Triangle { t: f64, index: u32, u: f64, v: f64 },
    Ground { t: f64 },
}

impl TraceScene {
    /// The nearest thing along the ray, the floor included when `with_ground`.
    pub(crate) fn intersect(
        &self,
        ray: &TraceRay,
        t_max: f64,
        with_ground: bool,
    ) -> Option<Contact> {
        let mut best =
            self.bvh
                .closest(&self.triangles, ray, 0.0, t_max)
                .map(|h| Contact::Triangle {
                    t: h.t,
                    index: h.triangle,
                    u: h.u,
                    v: h.v,
                });
        if with_ground
            && let Some(g) = &self.ground
            // The floor is seen from above only: a camera below it (a view from beneath
            // the part) looks straight through, as in Fusion.
            && ray.origin.z > g.z
            && ray.dir.z < 0.0
        {
            let t = (g.z - ray.origin.z) / ray.dir.z;
            let nearer = match best {
                Some(Contact::Triangle { t: tt, .. }) => t < tt,
                _ => true,
            };
            if t > 0.0 && t < t_max && nearer {
                best = Some(Contact::Ground { t });
            }
        }
        best
    }
}

/// The two coordinates of the plane a face most nearly lies in, for patterns that need a
/// direction on the surface.
fn planar(q: Vec3, n: Vec3) -> (f64, f64) {
    let a = n.abs();
    if a.z >= a.x && a.z >= a.y {
        (q.x, q.y)
    } else if a.y >= a.x {
        (q.x, q.z)
    } else {
        (q.y, q.z)
    }
}

fn smoothstep(e0: f64, e1: f64, x: f64) -> f64 {
    let t = ((x - e0) / (e1 - e0)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// Value noise in `[0, 1]`, smoothly interpolated between hashed lattice values.
pub(crate) fn noise(p: Vec3) -> f64 {
    let i = p.floor();
    let f = p - i;
    let w = f * f * (Vec3::splat(3.0) - f * 2.0);
    let (x, y, z) = (i.x as i64, i.y as i64, i.z as i64);
    let h = |dx: i64, dy: i64, dz: i64| -> f64 {
        let k = (x + dx).wrapping_mul(73_856_093)
            ^ (y + dy).wrapping_mul(19_349_663)
            ^ (z + dz).wrapping_mul(83_492_791);
        (crate::sampling::hash(k as u64) >> 11) as f64 * (1.0 / (1u64 << 53) as f64)
    };
    let lerp = |a: f64, b: f64, t: f64| a + (b - a) * t;
    let x0 = lerp(
        lerp(h(0, 0, 0), h(1, 0, 0), w.x),
        lerp(h(0, 1, 0), h(1, 1, 0), w.x),
        w.y,
    );
    let x1 = lerp(
        lerp(h(0, 0, 1), h(1, 0, 1), w.x),
        lerp(h(0, 1, 1), h(1, 1, 1), w.x),
        w.y,
    );
    lerp(x0, x1, w.z)
}

fn fbm(p: Vec3, octaves: u32) -> f64 {
    let (mut sum, mut amp, mut freq, mut norm) = (0.0, 0.5, 1.0, 0.0);
    for _ in 0..octaves {
        sum += amp * noise(p * freq);
        norm += amp;
        amp *= 0.5;
        freq *= 2.0;
    }
    sum / norm
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn noise_stays_in_range_and_is_continuous() {
        let mut last = noise(Vec3::new(0.0, 0.3, 0.7));
        for i in 1..2000 {
            let p = Vec3::new(i as f64 * 0.001, 0.3, 0.7);
            let n = noise(p);
            assert!((0.0..=1.0).contains(&n));
            assert!(
                (n - last).abs() < 0.05,
                "a step of 0.001 jumped {}",
                n - last
            );
            last = n;
        }
    }

    #[test]
    fn a_mesh_with_unknown_materials_falls_back_to_the_default() {
        let mut mesh = TriMesh::default();
        mesh.push_triangle([Vec3::ZERO, Vec3::X, Vec3::Y], 0);
        let mut b = SceneBuilder::new();
        b.add_mesh(&mesh, |_| 7);
        let scene = b.build(&SceneSettings::default());
        assert_eq!(scene.triangle_count(), 1);
        assert_eq!(scene.corners[0].material, 0);
    }

    #[test]
    fn the_floor_sits_under_the_model_and_is_seen_only_from_above() {
        let mut mesh = TriMesh::default();
        mesh.push_triangle(
            [
                Vec3::new(0.0, 0.0, 5.0),
                Vec3::new(1.0, 0.0, 5.0),
                Vec3::new(0.0, 1.0, 6.0),
            ],
            0,
        );
        let mut b = SceneBuilder::new();
        b.add_mesh(&mesh, |_| 0);
        let scene = b.build(&SceneSettings::default());
        let g = scene.ground.as_ref().expect("a floor by default");
        assert!(g.z < 5.0 && g.z > 4.99);
        let down = TraceRay::new(Vec3::new(10.0, 10.0, 20.0), -Vec3::Z);
        assert!(matches!(
            scene.intersect(&down, f64::INFINITY, true),
            Some(Contact::Ground { .. })
        ));
        let up = TraceRay::new(Vec3::new(10.0, 10.0, 0.0), Vec3::Z);
        assert!(scene.intersect(&up, f64::INFINITY, true).is_none());
    }

    #[test]
    fn patterns_vary_and_stay_in_range() {
        for name in [
            "Wood - Oak",
            "Carbon Fibre - Twill",
            "Steel - Brushed",
            "Concrete",
        ] {
            let m = Material::new(crate::library::find(name).unwrap());
            let mut lo = f64::INFINITY;
            let mut hi = f64::NEG_INFINITY;
            for i in 0..400 {
                let p = Vec3::new(i as f64 * 0.37, i as f64 * 0.11, i as f64 * 0.05);
                let s = m.at(p, Vec3::Z);
                assert!(s.base.iter().all(|c| (0.0..=1.0).contains(c)), "{name}");
                assert!((0.0..=1.0).contains(&s.roughness), "{name}");
                lo = lo.min(s.base[0]);
                hi = hi.max(s.base[0]);
            }
            assert!(hi - lo > 0.01, "{name} shows no pattern");
        }
    }
}
