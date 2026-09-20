//! GPU-side uniform layouts and the arena that streams per-draw uniforms each frame.
//!
//! Field order and padding must match the WGSL structs in `src/shaders/`. Every struct is
//! a multiple of 16 bytes as WGSL uniform rules require, and every per-draw struct fits in
//! one [`UniformArena::SLOT`] so all pipelines can share a single dynamic-offset bind group.

use std::num::NonZeroU64;

use bytemuck::{Pod, Zeroable};

#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
pub(crate) struct Globals {
    pub view_proj: [[f32; 4]; 4],
    pub camera_pos: [f32; 4],
    pub key_light: [f32; 4],
    pub viewport: [f32; 4],
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
pub(crate) struct MeshDraw {
    pub model: [[f32; 4]; 4],
    pub normal_matrix: [[f32; 4]; 4],
    pub color: [f32; 4],
    pub highlight_color: [f32; 4],
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
pub(crate) struct LineDraw {
    pub model: [[f32; 4]; 4],
    pub color: [f32; 4],
    /// `[width_px, dashed, dash_px, gap_px]`.
    pub params: [f32; 4],
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
pub(crate) struct PointDraw {
    pub color: [f32; 4],
    /// `[size_px, 0, 0, 0]`.
    pub params: [f32; 4],
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
pub(crate) struct TriDraw {
    pub color: [f32; 4],
    /// Unused; the uniform is padded to the 16-byte multiple WGSL requires.
    pub params: [f32; 4],
}

/// Append-only staging area flushed to a GPU buffer once per frame. Used both for uniform
/// slots and for the line/point instance streams; growing doubles the buffer so a busy
/// frame settles quickly to zero reallocations.
pub(crate) struct StreamBuffer {
    label: &'static str,
    usage: wgpu::BufferUsages,
    buffer: Option<wgpu::Buffer>,
    staging: Vec<u8>,
}

impl StreamBuffer {
    pub fn new(label: &'static str, usage: wgpu::BufferUsages) -> Self {
        Self {
            label,
            usage: usage | wgpu::BufferUsages::COPY_DST,
            buffer: None,
            staging: Vec::new(),
        }
    }

    pub fn clear(&mut self) {
        self.staging.clear();
    }

    pub fn len(&self) -> usize {
        self.staging.len()
    }

    pub fn push<T: Pod>(&mut self, value: &T) {
        self.staging.extend_from_slice(bytemuck::bytes_of(value));
    }

    pub fn pad_to(&mut self, alignment: usize) {
        let padded = self.staging.len().div_ceil(alignment) * alignment;
        self.staging.resize(padded, 0);
    }

    pub fn buffer(&self) -> Option<&wgpu::Buffer> {
        self.buffer.as_ref()
    }

    /// Uploads the staged bytes, reallocating if needed. Returns `true` when a new buffer
    /// was created so callers holding bind groups over it can rebuild them.
    pub fn flush(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, min_capacity: u64) -> bool {
        let needed = (self.staging.len() as u64).max(min_capacity);
        let reallocated = match &self.buffer {
            Some(existing) if existing.size() >= needed => false,
            _ => {
                let capacity = self
                    .buffer
                    .as_ref()
                    .map_or(needed, |b| (b.size() * 2).max(needed));
                self.buffer = Some(device.create_buffer(&wgpu::BufferDescriptor {
                    label: Some(self.label),
                    size: capacity,
                    usage: self.usage,
                    mapped_at_creation: false,
                }));
                true
            }
        };
        if let (Some(buffer), false) = (&self.buffer, self.staging.is_empty()) {
            queue.write_buffer(buffer, 0, &self.staging);
        }
        reallocated
    }
}

/// Per-draw uniforms for one frame, addressed by dynamic offset from a single bind group.
pub(crate) struct UniformArena {
    stream: StreamBuffer,
    bind_group: Option<wgpu::BindGroup>,
}

impl UniformArena {
    /// Slot size: the maximum dynamic-offset alignment wgpu guarantees, and larger than
    /// every per-draw struct above.
    pub const SLOT: u64 = 256;

    pub fn new() -> Self {
        Self {
            stream: StreamBuffer::new("per-draw uniforms", wgpu::BufferUsages::UNIFORM),
            bind_group: None,
        }
    }

    pub fn begin(&mut self) {
        self.stream.clear();
    }

    /// Stores `value` in its own slot and returns the dynamic offset that selects it.
    pub fn push<T: Pod>(&mut self, value: &T) -> u32 {
        debug_assert!(size_of::<T>() as u64 <= Self::SLOT);
        let offset = self.stream.len() as u32;
        self.stream.push(value);
        self.stream.pad_to(Self::SLOT as usize);
        offset
    }

    pub fn flush(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        layout: &wgpu::BindGroupLayout,
    ) {
        let reallocated = self.stream.flush(device, queue, Self::SLOT * 64);
        if reallocated || self.bind_group.is_none() {
            let buffer = self.stream.buffer().expect("flush allocates the buffer");
            self.bind_group = Some(device.create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("per-draw uniforms"),
                layout,
                entries: &[wgpu::BindGroupEntry {
                    binding: 0,
                    resource: wgpu::BindingResource::Buffer(wgpu::BufferBinding {
                        buffer,
                        offset: 0,
                        size: NonZeroU64::new(Self::SLOT),
                    }),
                }],
            }));
        }
    }

    pub fn bind_group(&self) -> &wgpu::BindGroup {
        self.bind_group.as_ref().expect("flush before recording")
    }
}
