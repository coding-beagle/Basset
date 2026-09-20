//! Window, GPU surface and egui integration.
//!
//! Each frame renders the 3D viewport across the whole window first and then draws egui
//! on top with a load (not clear) pass, so panels simply cover parts of the viewport.
//! Camera and picking therefore work in whole-window pixels, which keeps the editor
//! unaware of panel layout.

use std::path::PathBuf;
use std::sync::Arc;
use std::time::Duration;

use anyhow::Context as _;
use winit::application::ApplicationHandler;
use winit::event::WindowEvent;
use winit::event_loop::{ActiveEventLoop, ControlFlow, EventLoop};
use winit::window::{Window, WindowAttributes, WindowId};

use crate::editor::Editor;

pub fn run(path: Option<PathBuf>) -> anyhow::Result<()> {
    let event_loop = EventLoop::new().context("creating event loop")?;
    event_loop.set_control_flow(ControlFlow::Wait);
    let mut app = App {
        gfx: None,
        editor: Editor::new(path),
    };
    event_loop.run_app(&mut app).context("running event loop")?;
    Ok(())
}

struct App {
    gfx: Option<Gfx>,
    editor: Editor,
}

/// Everything that only exists once a window is open. Created in `resumed` because
/// winit does not allow window creation before that on every platform.
struct Gfx {
    window: Arc<Window>,
    surface: wgpu::Surface<'static>,
    device: wgpu::Device,
    queue: wgpu::Queue,
    config: wgpu::SurfaceConfiguration,
    viewport: basset_viewport::Renderer,
    egui_ctx: egui::Context,
    egui_state: egui_winit::State,
    egui_renderer: egui_wgpu::Renderer,
}

impl Gfx {
    fn new(event_loop: &ActiveEventLoop) -> anyhow::Result<Self> {
        let attributes = WindowAttributes::default()
            .with_title("Basset")
            .with_inner_size(winit::dpi::LogicalSize::new(1400.0, 900.0));
        let window = Arc::new(
            event_loop
                .create_window(attributes)
                .context("creating window")?,
        );

        // Passing the display handle lets wgpu pick a backend that can actually present
        // to this window (Wayland vs X11 on Linux) instead of guessing.
        let instance = wgpu::Instance::new(
            wgpu::InstanceDescriptor::new_with_display_handle_from_env(Box::new(window.clone())),
        );
        let surface = instance
            .create_surface(window.clone())
            .context("creating surface")?;
        let adapter = pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions {
            power_preference: wgpu::PowerPreference::HighPerformance,
            compatible_surface: Some(&surface),
            ..Default::default()
        }))
        .context("no compatible GPU adapter")?;
        log::info!("using adapter: {:?}", adapter.get_info().name);
        let (device, queue) = pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor {
            label: Some("basset"),
            required_features: wgpu::Features::empty(),
            required_limits: wgpu::Limits::default(),
            experimental_features: wgpu::ExperimentalFeatures::disabled(),
            memory_hints: wgpu::MemoryHints::Performance,
            trace: wgpu::Trace::Off,
        }))
        .context("requesting device")?;

        let size = window.inner_size();
        let caps = surface.get_capabilities(&adapter);
        // egui blends in gamma space and wants a plain (non-sRGB) target, while the
        // viewport writes linear colour and wants hardware sRGB encoding. Both get their
        // way by configuring a plain surface with an sRGB *view* format for the 3D pass.
        let plain = caps
            .formats
            .iter()
            .copied()
            .find(|f| !f.is_srgb())
            .unwrap_or(caps.formats[0]);
        let srgb = plain.add_srgb_suffix();
        let mut config = surface
            .get_default_config(&adapter, size.width.max(1), size.height.max(1))
            .context("surface has no default configuration")?;
        config.format = plain;
        config.view_formats = if srgb != plain { vec![srgb] } else { vec![] };
        config.present_mode = wgpu::PresentMode::AutoVsync;
        surface.configure(&device, &config);
        let format = plain;

        let viewport = basset_viewport::Renderer::new(&device, srgb, 4);
        let egui_ctx = egui::Context::default();
        let egui_state = egui_winit::State::new(
            egui_ctx.clone(),
            egui::ViewportId::ROOT,
            &window,
            Some(window.scale_factor() as f32),
            None,
            Some(device.limits().max_texture_dimension_2d as usize),
        );
        let egui_renderer = egui_wgpu::Renderer::new(
            &device,
            format,
            egui_wgpu::RendererOptions {
                msaa_samples: 1,
                ..Default::default()
            },
        );
        Ok(Self {
            window,
            surface,
            device,
            queue,
            config,
            viewport,
            egui_ctx,
            egui_state,
            egui_renderer,
        })
    }

    fn resize(&mut self, width: u32, height: u32) {
        if width == 0 || height == 0 {
            return;
        }
        self.config.width = width;
        self.config.height = height;
        self.surface.configure(&self.device, &self.config);
        self.viewport.resize(&self.device, [width, height]);
    }

    fn render(&mut self, editor: &mut Editor) {
        let size = [self.config.width, self.config.height];
        editor.set_window_size(size);
        editor.sync_meshes(&mut self.viewport, &self.device, &self.queue);

        let raw_input = self.egui_state.take_egui_input(&self.window);
        let full = self.egui_ctx.run_ui(raw_input, |ui| editor.ui(ui));
        self.egui_state
            .handle_platform_output(&self.window, full.platform_output);
        let paint_jobs = self.egui_ctx.tessellate(full.shapes, full.pixels_per_point);
        // epaint insists deltas are consumed, not just read, hence the take/clear dance.
        let mut textures = full.textures_delta;
        for (id, deltas) in std::mem::take(&mut textures.set) {
            for delta in &deltas {
                self.egui_renderer
                    .update_texture(&self.device, &self.queue, id, delta);
            }
        }
        let free: Vec<egui::TextureId> = textures.free.drain().collect();
        textures.clear();

        let frame = match self.surface.get_current_texture() {
            wgpu::CurrentSurfaceTexture::Success(t)
            | wgpu::CurrentSurfaceTexture::Suboptimal(t) => t,
            wgpu::CurrentSurfaceTexture::Timeout | wgpu::CurrentSurfaceTexture::Occluded => return,
            wgpu::CurrentSurfaceTexture::Outdated | wgpu::CurrentSurfaceTexture::Lost => {
                self.surface.configure(&self.device, &self.config);
                return;
            }
            wgpu::CurrentSurfaceTexture::Validation => {
                log::error!("surface validation error");
                return;
            }
        };
        let view = frame
            .texture
            .create_view(&wgpu::TextureViewDescriptor::default());
        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("frame"),
            });

        let scene = editor.scene();
        self.viewport
            .render(&self.device, &self.queue, &mut encoder, &view, size, &scene);

        let screen = egui_wgpu::ScreenDescriptor {
            size_in_pixels: size,
            pixels_per_point: full.pixels_per_point,
        };
        let user_buffers = self.egui_renderer.update_buffers(
            &self.device,
            &self.queue,
            &mut encoder,
            &paint_jobs,
            &screen,
        );
        {
            let pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("egui"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &view,
                    depth_slice: None,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Load,
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: None,
                timestamp_writes: None,
                occlusion_query_set: None,
                multiview_mask: None,
            });
            let mut pass = pass.forget_lifetime();
            self.egui_renderer.render(&mut pass, &paint_jobs, &screen);
        }
        self.queue.submit(
            user_buffers
                .into_iter()
                .chain(std::iter::once(encoder.finish())),
        );
        self.window.pre_present_notify();
        self.queue.present(frame);
        for id in &free {
            self.egui_renderer.free_texture(id);
        }

        let wants_repaint = full
            .viewport_output
            .get(&egui::ViewportId::ROOT)
            .is_some_and(|v| v.repaint_delay == Duration::ZERO);
        if wants_repaint || editor.take_repaint_request() {
            self.window.request_redraw();
        }
        if let Some(title) = editor.take_title_change() {
            self.window.set_title(&title);
        }
    }
}

impl ApplicationHandler for App {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        if self.gfx.is_some() {
            return;
        }
        match Gfx::new(event_loop) {
            Ok(gfx) => {
                gfx.window.request_redraw();
                self.gfx = Some(gfx);
            }
            Err(err) => {
                log::error!("failed to initialise graphics: {err:#}");
                event_loop.exit();
            }
        }
    }

    fn window_event(&mut self, event_loop: &ActiveEventLoop, _id: WindowId, event: WindowEvent) {
        let Some(gfx) = self.gfx.as_mut() else { return };
        match event {
            WindowEvent::CloseRequested => event_loop.exit(),
            WindowEvent::Resized(size) => {
                gfx.resize(size.width, size.height);
                gfx.window.request_redraw();
            }
            WindowEvent::RedrawRequested => gfx.render(&mut self.editor),
            event => {
                let egui_has_keyboard = gfx.egui_ctx.egui_wants_keyboard_input();
                let response = gfx.egui_state.on_window_event(&gfx.window, &event);
                let pointer = matches!(
                    event,
                    WindowEvent::CursorMoved { .. }
                        | WindowEvent::MouseInput { .. }
                        | WindowEvent::MouseWheel { .. }
                        | WindowEvent::Touch(_)
                );
                // egui reports Tab as consumed even when nothing has focus, because it
                // would use it to focus its first widget. The editor wants it first, to
                // put the focus on a size entry box instead.
                let tab = matches!(
                    &event,
                    WindowEvent::KeyboardInput { event, .. }
                        if event.logical_key == winit::keyboard::Key::Named(winit::keyboard::NamedKey::Tab)
                );
                if response.consumed && pointer {
                    self.editor.pointer_over_ui();
                } else if !response.consumed || (tab && !egui_has_keyboard) {
                    self.editor.handle_window_event(&event, egui_has_keyboard);
                }
                if response.repaint || self.editor.take_repaint_request() {
                    gfx.window.request_redraw();
                }
                if self.editor.wants_exit() {
                    event_loop.exit();
                }
            }
        }
    }
}
