//! Progressive rendering on background threads.
//!
//! A render is a sequence of passes, each one more sample of every pixel, averaged into
//! an accumulation buffer; after every pass the running average is tone mapped into a
//! fresh [`Image`] the caller can pick up. That is what an in-canvas render needs — a
//! rough picture within a fraction of a second that keeps getting cleaner until the
//! camera moves — and a final render is the same thing run to a sample count.
//!
//! The passes run on scoped worker threads that take rows from a shared counter, so a
//! slow row (the one through the glass) does not hold the others up. A [`RenderJob`]
//! owns a coordinating thread that does that and publishes the images; cancelling it is
//! a flag checked between rows, so a camera drag that restarts the render many times a
//! second does not queue up passes nobody will see.

use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use crate::camera::RenderCamera;
use crate::color::tone_map;
use crate::image::Image;
use crate::sampling::{Rng, hash};
use crate::scene::TraceScene;
use crate::trace::{Sample, Tracer};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RenderOptions {
    pub width: u32,
    pub height: u32,
    /// Passes to run before stopping; each is one sample per pixel.
    pub samples: u32,
    pub max_bounces: u32,
    /// Worker threads; zero means one fewer than the machine has, so the window that
    /// asked for the render stays responsive.
    pub threads: usize,
    /// The largest radiance one sample may carry. Lower is less noisy and slightly
    /// darker in the brightest reflections.
    pub clamp: f64,
}

impl Default for RenderOptions {
    fn default() -> Self {
        Self {
            width: 800,
            height: 600,
            samples: 128,
            max_bounces: 8,
            threads: 0,
            clamp: 12.0,
        }
    }
}

impl RenderOptions {
    fn worker_count(&self) -> usize {
        if self.threads > 0 {
            return self.threads;
        }
        std::thread::available_parallelism()
            .map(|n| n.get().saturating_sub(1).max(1))
            .unwrap_or(1)
    }
}

/// Running sums for one pixel. See the floor in [`crate::trace`] for why a pixel keeps
/// the floor's two shadings apart instead of one colour.
#[derive(Clone, Copy, Debug, Default)]
struct Accum {
    direct: [f64; 3],
    background: [f64; 3],
    with_model: [f64; 3],
    without_model: [f64; 3],
}

impl Accum {
    fn add(&mut self, s: &Sample) {
        for i in 0..3 {
            self.direct[i] += s.direct[i];
        }
        if let Some(f) = &s.floor {
            for i in 0..3 {
                self.background[i] += f.background[i];
                self.with_model[i] += f.with_model[i];
                self.without_model[i] += f.without_model[i];
            }
        }
    }

    /// The pixel's average radiance after `n` samples.
    fn resolve(&self, n: u32) -> [f32; 3] {
        let n = f64::from(n.max(1));
        [0, 1, 2].map(|i| {
            // The floor shows the background scaled by how much the model darkened (or
            // brightened, by reflection) it. Capped, so a pixel whose few floor samples
            // happened to see nothing unoccluded does not flare.
            let ratio = if self.without_model[i] > 1e-9 {
                (self.with_model[i] / self.without_model[i]).clamp(0.0, 2.0)
            } else {
                1.0
            };
            ((self.direct[i] + self.background[i] * ratio) / n) as f32
        })
    }
}

/// Renders synchronously, on as many threads as the options say, and returns the image.
pub fn render(scene: &TraceScene, camera: &RenderCamera, options: &RenderOptions) -> Image {
    let cancel = AtomicBool::new(false);
    let mut accum = vec![Accum::default(); (options.width * options.height) as usize];
    for pass in 0..options.samples {
        run_pass(scene, camera, options, &mut accum, pass, &cancel);
    }
    resolve(&accum, options, options.samples)
}

fn resolve(accum: &[Accum], options: &RenderOptions, samples: u32) -> Image {
    let mut pixels = Vec::with_capacity(accum.len() * 4);
    for a in accum {
        let [r, g, b] = tone_map(a.resolve(samples));
        pixels.extend([r, g, b, 255]);
    }
    Image {
        width: options.width,
        height: options.height,
        pixels,
        samples,
    }
}

/// One sample of every pixel. Returns false if cancelled part way, in which case the
/// buffer holds a partial pass and must not be shown as `pass + 1` samples.
fn run_pass(
    scene: &TraceScene,
    camera: &RenderCamera,
    options: &RenderOptions,
    accum: &mut [Accum],
    pass: u32,
    cancel: &AtomicBool,
) -> bool {
    let (w, h) = (options.width as usize, options.height as usize);
    if w == 0 || h == 0 {
        return true;
    }
    let tracer = Tracer {
        scene,
        max_bounces: options.max_bounces.max(1),
        clamp: options.clamp,
    };
    let aspect = w as f64 / h as f64;
    let rows = Mutex::new(accum.chunks_mut(w).enumerate());
    let workers = options.worker_count().min(h);
    std::thread::scope(|s| {
        for _ in 0..workers {
            s.spawn(|| {
                loop {
                    if cancel.load(Ordering::Relaxed) {
                        return;
                    }
                    let next = rows.lock().map(|mut r| r.next()).unwrap_or(None);
                    let Some((y, row)) = next else {
                        return;
                    };
                    for (x, px) in row.iter_mut().enumerate() {
                        let index = (y * w + x) as u64;
                        let mut rng = Rng::new(hash(index ^ (u64::from(pass) << 40)), index);
                        let sx = (x as f64 + rng.next_f64()) / w as f64;
                        let sy = (y as f64 + rng.next_f64()) / h as f64;
                        let sample = tracer.sample(camera, sx, sy, aspect, &mut rng);
                        px.add(&sample);
                    }
                }
            });
        }
    });
    !cancel.load(Ordering::Relaxed)
}

/// Shared between a job's thread and its owner.
struct Shared {
    cancel: AtomicBool,
    finished: AtomicBool,
    samples: AtomicU32,
    latest: Mutex<Option<Arc<Image>>>,
}

/// A render running in the background; see the module documentation.
pub struct RenderJob {
    shared: Arc<Shared>,
    started: Instant,
    target: u32,
    handle: Option<JoinHandle<()>>,
}

impl RenderJob {
    pub fn start(scene: Arc<TraceScene>, camera: RenderCamera, options: RenderOptions) -> Self {
        let shared = Arc::new(Shared {
            cancel: AtomicBool::new(false),
            finished: AtomicBool::new(false),
            samples: AtomicU32::new(0),
            latest: Mutex::new(None),
        });
        let thread_shared = shared.clone();
        let handle = std::thread::Builder::new()
            .name("basset-render".into())
            .spawn(move || {
                let shared = thread_shared;
                let mut accum = vec![Accum::default(); (options.width * options.height) as usize];
                let mut last_publish: Option<Instant> = None;
                for pass in 0..options.samples {
                    if !run_pass(&scene, &camera, &options, &mut accum, pass, &shared.cancel) {
                        break;
                    }
                    let done = pass + 1;
                    shared.samples.store(done, Ordering::Relaxed);
                    // Every early pass is worth seeing; later ones change little, and tone
                    // mapping a large frame is not free, so they are published a few times
                    // a second at most.
                    let due = done <= 8
                        || done == options.samples
                        || last_publish.is_none_or(|t| t.elapsed() > Duration::from_millis(250));
                    if due {
                        let image = Arc::new(resolve(&accum, &options, done));
                        if let Ok(mut latest) = shared.latest.lock() {
                            *latest = Some(image);
                        }
                        last_publish = Some(Instant::now());
                    }
                }
                shared.finished.store(true, Ordering::Release);
            })
            .ok();
        if handle.is_none() {
            log::error!("could not start a render thread");
            shared.finished.store(true, Ordering::Release);
        }
        Self {
            shared,
            started: Instant::now(),
            target: options.samples,
            handle,
        }
    }

    /// The most recent picture, if a pass has finished.
    pub fn latest(&self) -> Option<Arc<Image>> {
        self.shared.latest.lock().ok().and_then(|l| l.clone())
    }

    /// Passes completed so far.
    pub fn samples(&self) -> u32 {
        self.shared.samples.load(Ordering::Relaxed)
    }

    pub fn target(&self) -> u32 {
        self.target
    }

    pub fn elapsed(&self) -> Duration {
        self.started.elapsed()
    }

    pub fn is_finished(&self) -> bool {
        self.shared.finished.load(Ordering::Acquire)
    }

    /// Asks the job to stop after the row it is on.
    pub fn cancel(&self) {
        self.shared.cancel.store(true, Ordering::Relaxed);
    }

    /// Waits for the job to end and returns its last picture.
    pub fn join(mut self) -> Option<Arc<Image>> {
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
        self.latest()
    }
}

impl Drop for RenderJob {
    /// A job nobody holds is a job nobody will look at. The thread is not joined — it
    /// stops within a row — so dropping a job never stalls the frame that dropped it.
    fn drop(&mut self) {
        self.cancel();
    }
}

#[cfg(test)]
mod tests {
    use basset_math::{TriMesh, Vec3};

    use super::*;
    use crate::camera::Lens;
    use crate::scene::SceneBuilder;
    use crate::settings::{Background, SceneSettings};
    use crate::{Appearance, Srgb};

    fn cube(size: f64) -> TriMesh {
        let mut m = TriMesh::default();
        let v = |x: f64, y: f64, z: f64| Vec3::new(x, y, z) * size;
        let quads = [
            [v(0., 0., 0.), v(0., 1., 0.), v(1., 1., 0.), v(1., 0., 0.)],
            [v(0., 0., 1.), v(1., 0., 1.), v(1., 1., 1.), v(0., 1., 1.)],
            [v(0., 0., 0.), v(1., 0., 0.), v(1., 0., 1.), v(0., 0., 1.)],
            [v(0., 1., 0.), v(0., 1., 1.), v(1., 1., 1.), v(1., 1., 0.)],
            [v(0., 0., 0.), v(0., 0., 1.), v(0., 1., 1.), v(0., 1., 0.)],
            [v(1., 0., 0.), v(1., 1., 0.), v(1., 1., 1.), v(1., 0., 1.)],
        ];
        for (id, q) in quads.iter().enumerate() {
            let n = (q[1] - q[0]).cross(q[2] - q[0]).normalize();
            for tri in [[q[0], q[1], q[2]], [q[0], q[2], q[3]]] {
                let base = m.positions.len() as u32;
                m.positions.extend(tri);
                m.normals.extend([n; 3]);
                m.indices.extend([base, base + 1, base + 2]);
                m.face_ids.push(id as u32);
            }
        }
        m
    }

    fn scene_with(appearance: &Appearance, settings: &SceneSettings) -> TraceScene {
        let mut b = SceneBuilder::new();
        let m = b.add_appearance(appearance);
        b.add_mesh(&cube(10.0), |_| m);
        b.build(settings)
    }

    fn small(samples: u32) -> RenderOptions {
        RenderOptions {
            width: 48,
            height: 36,
            samples,
            threads: 2,
            ..RenderOptions::default()
        }
    }

    fn camera(scene: &TraceScene) -> RenderCamera {
        RenderCamera::fit(&scene.bounds(), Vec3::new(1.0, -1.2, 0.8), 0.7, 48.0 / 36.0)
    }

    #[test]
    fn a_render_is_the_same_image_every_time() {
        let scene = scene_with(&Appearance::DEFAULT, &SceneSettings::default());
        let a = render(&scene, &camera(&scene), &small(2));
        let b = render(&scene, &camera(&scene), &small(2));
        assert_eq!(a, b);
    }

    #[test]
    fn the_model_stands_out_of_a_solid_background_in_its_own_colour() {
        let red = Appearance {
            color: Srgb::hex(0xc01010),
            ..Appearance::DEFAULT
        };
        let settings = SceneSettings {
            background: Background::Solid(Srgb::hex(0x203040)),
            ground_plane: false,
            ..SceneSettings::default()
        };
        let scene = scene_with(&red, &settings);
        let image = render(&scene, &camera(&scene), &small(8));
        // A corner is background, exactly the colour chosen (to the curve's rounding).
        let corner = image.pixel(0, 0);
        for (got, want) in corner.iter().zip([0x20, 0x30, 0x40]) {
            assert!((i32::from(*got) - want).abs() <= 2, "{corner:?}");
        }
        // The middle is the cube, red.
        let [r, g, b, _] = image.pixel(24, 18);
        assert!(r > g + 40 && r > b + 40, "{:?}", [r, g, b]);
    }

    #[test]
    fn the_floor_is_invisible_except_for_the_shadow() {
        let scene = scene_with(&Appearance::DEFAULT, &SceneSettings::default());
        let cam = camera(&scene);
        let image = render(&scene, &cam, &small(64));
        // Far from the model along the bottom edge the floor matches the sky it hides,
        // so it is only as bright as a ratio of one makes it: the background.
        let without_floor = {
            let s = scene_with(
                &Appearance::DEFAULT,
                &SceneSettings {
                    ground_plane: false,
                    ..SceneSettings::default()
                },
            );
            render(&s, &cam, &small(64))
        };
        let far = image.pixel(1, 35);
        let sky = without_floor.pixel(1, 35);
        for (a, b) in far.iter().zip(sky) {
            assert!(
                (i32::from(*a) - i32::from(b)).abs() <= 12,
                "{far:?} vs {sky:?}"
            );
        }
        // Somewhere the floor is darker than what it hides: the shadow.
        let darker = (0..image.height)
            .flat_map(|y| (0..image.width).map(move |x| (x, y)))
            .any(|(x, y)| {
                let a = image.pixel(x, y);
                let b = without_floor.pixel(x, y);
                i32::from(b[1]) - i32::from(a[1]) > 15
            });
        let mut worst = 0;
        for y in 0..image.height {
            for x in 0..image.width {
                worst = worst
                    .max(i32::from(without_floor.pixel(x, y)[1]) - i32::from(image.pixel(x, y)[1]));
            }
        }
        assert!(
            darker,
            "no shadow anywhere; the darkest the floor got was {worst} below the sky"
        );
    }

    #[test]
    fn a_job_publishes_passes_and_stops_when_asked() {
        let scene = Arc::new(scene_with(&Appearance::DEFAULT, &SceneSettings::default()));
        let cam = RenderCamera::look_at(
            Vec3::new(40.0, -40.0, 30.0),
            Vec3::splat(5.0),
            Lens::Perspective { fov_y: 0.7 },
        );
        let job = RenderJob::start(scene.clone(), cam, small(3));
        let image = job.join().expect("a finished job has a picture");
        assert_eq!(image.samples, 3);

        let endless = RenderJob::start(scene, cam, small(1_000_000));
        endless.cancel();
        let started = Instant::now();
        endless.join();
        assert!(started.elapsed() < Duration::from_secs(5));
    }
}
