// Simulator infrastructure. Do not edit; implement your solution in src/lib.rs.
//! Depth-tested 3D armor rendering on RXUI's device, with one shared plate geometry basis.
use super::{Config, Frame, Outcome};
use crate::{Plate, physics, ui::Color};
use anyhow::Result;
use bytemuck::{Pod, Zeroable};
use glam::{Mat4, Vec3, Vec4, camera};
use rxui::{
    Image,
    drawing::GraphicsContext,
    wgpu::{self, util::DeviceExt},
};

pub const WIDTH: f32 = 960.0;
pub const HEIGHT: f32 = 550.0;
const BACKING: [u32; 2] = [1920, 1100];
const EXPORT_SIZE: [u32; 2] = [3840, 2160];
const EXPORT_FPS: usize = 60;

#[derive(Clone, Copy)]
pub struct Camera {
    pub yaw: f32,
    pub elevation: f32,
    pub distance: f32,
    pub focus: bool,
    pub pan_m: [f32; 3],
    pub aspect: f32,
}
impl Default for Camera {
    fn default() -> Self {
        Self {
            yaw: -1.4,
            elevation: 0.4,
            distance: 8.0,
            focus: false,
            pan_m: [0.0; 3],
            aspect: WIDTH / HEIGHT,
        }
    }
}
impl Camera {
    fn matrix(self, frame: &Frame) -> Mat4 {
        let target = Vec3::new(frame.target.x_m as f32, frame.target.y_m as f32, 0.27);
        let look = if self.focus {
            target
        } else {
            Vec3::new(3.2, 0.0, 0.2)
        } + Vec3::from_array(self.pan_m);
        let eye = look
            + self.distance
                * Vec3::new(
                    self.yaw.cos() * self.elevation.cos(),
                    self.yaw.sin() * self.elevation.cos(),
                    self.elevation.sin(),
                );
        camera::rh::proj::directx::perspective(42_f32.to_radians(), self.aspect, 0.01, 200.0)
            * camera::rh::view::look_at_mat4(eye, look, Vec3::Z)
    }

    pub fn pan(&mut self, delta: [f32; 2], viewport_height: f32) {
        let scale = 2.0 * self.distance * 21_f32.to_radians().tan() / viewport_height.max(1.0);
        let right = Vec3::new(-self.yaw.sin(), self.yaw.cos(), 0.0);
        let up = Vec3::new(
            -self.yaw.cos() * self.elevation.sin(),
            -self.yaw.sin() * self.elevation.sin(),
            self.elevation.cos(),
        );
        self.pan_m =
            (Vec3::from_array(self.pan_m) + scale * (-right * delta[0] + up * delta[1])).to_array();
    }

    pub fn project(self, frame: &Frame, point: [f64; 3], size: [f32; 2]) -> Option<[f32; 2]> {
        let clip = self.matrix(frame) * Vec3::from_array(point.map(|v| v as f32)).extend(1.0);
        if clip.w <= 0.0 {
            return None;
        }
        let ndc = clip.truncate() / clip.w;
        (ndc.x.abs() < 1.0 && ndc.y.abs() < 1.0 && (0.0..=1.0).contains(&ndc.z))
            .then_some([(ndc.x + 1.0) * size[0] / 2.0, (1.0 - ndc.y) * size[1] / 2.0])
    }
}

#[derive(Clone, Copy)]
pub struct Layers {
    pub gizmos: bool,
    pub normals: bool,
    pub hitboxes: bool,
    pub trails: bool,
    pub selected_shot: Option<usize>,
}
impl Default for Layers {
    fn default() -> Self {
        Self {
            gizmos: true,
            normals: false,
            hitboxes: false,
            trails: true,
            selected_shot: None,
        }
    }
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct Vertex {
    position: [f32; 4],
    color: [f32; 4],
    uv: [f32; 2],
}

struct Mesh {
    vertices: Vec<Vertex>,
}
impl Mesh {
    fn quad(&mut self, points: [[f64; 3]; 4], color: [f32; 3], normal: [f64; 3], textured: bool) {
        let light = Vec3::new(-0.4, -0.6, 1.0).normalize();
        let shade = if normal == [0.0; 3] {
            1.0
        } else {
            0.45 + 0.55 * Vec3::from_array(normal.map(|v| v as f32)).dot(light).abs()
        };
        for index in [0, 1, 2, 0, 2, 3] {
            self.vertices.push(Vertex {
                position: [
                    points[index][0] as f32,
                    points[index][1] as f32,
                    points[index][2] as f32,
                    1.0,
                ],
                color: [color[0] * shade, color[1] * shade, color[2] * shade, 1.0],
                uv: if textured {
                    [[0., 1.], [1., 1.], [1., 0.], [0., 0.]][index]
                } else {
                    [-1.; 2]
                },
            });
        }
    }

    fn line(&mut self, a: [f64; 3], b: [f64; 3], color: [f32; 3], width: f32) {
        let a = Vec3::from_array(a.map(|v| v as f32));
        let b = Vec3::from_array(b.map(|v| v as f32));
        let direction = (b - a).normalize_or_zero();
        if direction == Vec3::ZERO {
            return;
        }
        let mut side = direction.cross(Vec3::Z).normalize_or_zero() * width;
        if side == Vec3::ZERO {
            side = direction.cross(Vec3::Y).normalize_or_zero() * width;
        }
        for side in [side, direction.cross(side)] {
            self.quad(
                [a - side, b - side, b + side, a + side].map(|v| v.to_array().map(f64::from)),
                color,
                [0.; 3],
                false,
            );
        }
    }

    fn marker(&mut self, point: [f64; 3], color: [f32; 3], radius: f64) {
        for axis in 0..3 {
            let mut a = point;
            let mut b = point;
            a[axis] -= radius;
            b[axis] += radius;
            self.line(a, b, color, 0.0025);
        }
    }

    fn plate_quad(
        &mut self,
        plate: Plate,
        bounds: [f64; 4],
        offset: f64,
        color: [f32; 3],
        textured: bool,
    ) {
        let [left, right, bottom, top] = bounds;
        self.quad(
            [[left, bottom], [right, bottom], [right, top], [left, top]]
                .map(|[x, y]| plate.point(x, y, offset)),
            color,
            plate.outward_normal,
            textured,
        );
    }

    fn disc(&mut self, plate: Plate, x: f64, y: f64, radius: f64, offset: f64, color: [f32; 3]) {
        for index in 0..16 {
            let point = |i| {
                let angle = f64::from(i) * std::f64::consts::TAU / 16.0;
                plate.point(x + radius * angle.cos(), y + radius * angle.sin(), offset)
            };
            let center = plate.point(x, y, offset);
            self.quad(
                [center, point(index), point(index + 1), center],
                color,
                plate.outward_normal,
                false,
            );
        }
    }

    fn armor(&mut self, plate: Plate, config: Config, layers: Layers) {
        // AM02 housing envelope: 141 × 130 mm.
        let ring = [
            [-0.0625, -0.065],
            [0.0625, -0.065],
            [0.0705, -0.057],
            [0.0705, 0.057],
            [0.0625, 0.065],
            [-0.0625, 0.065],
            [-0.0705, 0.057],
            [-0.0705, -0.057],
        ];
        for i in 0..8 {
            let a = ring[i];
            let b = ring[(i + 1) % 8];
            self.quad(
                [
                    plate.point(0., 0., 0.001),
                    plate.point(a[0], a[1], 0.001),
                    plate.point(b[0], b[1], 0.001),
                    plate.point(0., 0., 0.001),
                ],
                [0.025, 0.032, 0.045],
                plate.outward_normal,
                false,
            );
            self.quad(
                [
                    plate.point(a[0], a[1], 0.001),
                    plate.point(a[0], a[1], -0.020),
                    plate.point(b[0], b[1], -0.020),
                    plate.point(b[0], b[1], 0.001),
                ],
                [0.06, 0.075, 0.09],
                plate.right_axis,
                false,
            );
        }
        self.plate_quad(
            plate,
            [-0.056, 0.056, -0.052, 0.052],
            0.0015,
            [0.85; 3],
            true,
        );
        let color = [0.04, 0.5, 1.0];
        for side in [-1., 1.] {
            let x = side * config.size.plate_width_m / 2.0;
            let half = config.size.plate_height_m / 2.0;
            let straight = (half - 0.004).max(0.0);
            self.plate_quad(
                plate,
                [x - 0.004, x + 0.004, -straight, straight],
                0.004,
                color,
                false,
            );
            for end in [-straight, straight] {
                self.disc(plate, x, end, 0.004, 0.004, color);
            }
            self.plate_quad(
                plate,
                [x - 0.0015, x + 0.0015, -straight, straight],
                0.0042,
                [0.8, 0.95, 1.0],
                false,
            );
            for end in [-straight, straight] {
                self.disc(plate, x, end, 0.0015, 0.0042, [0.8, 0.95, 1.0]);
            }
        }
        for x in [-0.05, 0.05] {
            for y in [-0.055, 0.055] {
                self.disc(plate, x, y, 0.0025, 0.002, [0.2; 3]);
                self.disc(plate, x, y, 0.001, 0.0021, [0.01; 3]);
            }
        }
        if layers.normals {
            self.line(
                plate.center_m,
                plate.point(0., 0., 0.15),
                Color::Green.rgb(),
                0.002,
            );
        }
        if layers.hitboxes {
            let w = config.size.plate_width_m / 2.;
            let h = config.size.plate_height_m / 2.;
            let corners = [[-w, -h], [w, -h], [w, h], [-w, h], [-w, -h]];
            for pair in corners.windows(2) {
                self.line(
                    plate.point(pair[0][0], pair[0][1], 0.006),
                    plate.point(pair[1][0], pair[1][1], 0.006),
                    Color::Green.rgb(),
                    0.001,
                );
            }
        }
    }
}

fn geometry(frame: &Frame, config: Config, layers: Layers, grid: [f32; 4]) -> Mesh {
    let mut mesh = Mesh {
        vertices: Vec::new(),
    };
    for i in -12..=18 {
        let v = f64::from(i);
        mesh.line(
            [v, -12., 0.],
            [v, 18., 0.],
            [grid[0], grid[1], grid[2]],
            0.002,
        );
        mesh.line(
            [-12., v, 0.],
            [18., v, 0.],
            [grid[0], grid[1], grid[2]],
            0.002,
        );
    }
    for vertex in &mut mesh.vertices {
        vertex.color[3] = 0.18;
    }
    for plate in frame.target.plates(config.size) {
        mesh.armor(plate, config, layers);
    }
    mesh.marker(physics::MUZZLE_M, Color::White.rgb(), 0.06);
    if let Some(command) = frame.command {
        let p = physics::MUZZLE_M;
        let endpoint = [
            p[0] + 0.3 * command.pitch_rad.cos() * command.yaw_rad.cos(),
            p[1] + 0.3 * command.pitch_rad.cos() * command.yaw_rad.sin(),
            p[2] + 0.3 * command.pitch_rad.sin(),
        ];
        mesh.line(p, endpoint, Color::White.rgb(), 0.012);
    }
    if layers.trails {
        for shot in &frame.shots {
            if layers.selected_shot.is_some_and(|id| id != shot.id) {
                continue;
            }
            if frame.time_s < shot.launch_time_s
                || shot.end_time().is_some_and(|end| frame.time_s - end > 1.0)
            {
                continue;
            }
            let end = shot.end_time().unwrap_or(frame.time_s);
            let color = match shot.outcome {
                Outcome::Hit { .. } => Color::Green.rgb(),
                Outcome::Miss { .. } => Color::Red.rgb(),
                _ => Color::Orange.rgb(),
            };
            for i in 0..48 {
                let t =
                    |step| shot.launch_time_s + (end - shot.launch_time_s) * f64::from(step) / 48.;
                mesh.line(shot.position(t(i)), shot.position(t(i + 1)), color, 0.0015);
            }
            mesh.marker(shot.position(end), color, 0.012);
        }
    }
    if layers.gizmos {
        for line in &frame.gizmos.lines {
            mesh.line(line.start_m, line.end_m, line.color.rgb(), 0.002);
        }
        for point in &frame.gizmos.points {
            mesh.marker(point.position_m, point.color.rgb(), 0.018);
        }
    }
    mesh
}

pub struct Renderer {
    output: wgpu::Texture,
    depth: wgpu::TextureView,
    multisample: wgpu::TextureView,
    pipeline: wgpu::RenderPipeline,
    texture: wgpu::BindGroup,
    vertices: wgpu::Buffer,
    background: [f32; 4],
    grid: [f32; 4],
    count: u32,
}

impl Renderer {
    pub fn new(graphics: &GraphicsContext) -> Result<Self> {
        Self::with_size(graphics, BACKING)
    }

    fn with_size(graphics: &GraphicsContext, size: [u32; 2]) -> Result<Self> {
        let device = graphics.device();
        let output = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("3D viewport"),
            size: wgpu::Extent3d {
                width: size[0],
                height: size[1],
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Rgba8UnormSrgb,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT
                | wgpu::TextureUsages::TEXTURE_BINDING
                | wgpu::TextureUsages::COPY_SRC,
            view_formats: &[],
        });
        let multisample = device
            .create_texture(&wgpu::TextureDescriptor {
                label: Some("3D MSAA"),
                size: output.size(),
                mip_level_count: 1,
                sample_count: 4,
                dimension: wgpu::TextureDimension::D2,
                format: output.format(),
                usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
                view_formats: &[],
            })
            .create_view(&Default::default());
        let depth = device
            .create_texture(&wgpu::TextureDescriptor {
                label: Some("3D depth"),
                size: output.size(),
                mip_level_count: 1,
                sample_count: 4,
                dimension: wgpu::TextureDimension::D2,
                format: wgpu::TextureFormat::Depth32Float,
                usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
                view_formats: &[],
            })
            .create_view(&Default::default());
        let image = image::load_from_memory(include_bytes!("../../assets/3.png"))?.to_rgba8();
        let artwork = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("Armor 3 mask"),
            size: wgpu::Extent3d {
                width: image.width(),
                height: image.height(),
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Rgba8UnormSrgb,
            usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
            view_formats: &[],
        });
        graphics.queue().write_texture(
            artwork.as_image_copy(),
            &image,
            wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(image.width() * 4),
                rows_per_image: Some(image.height()),
            },
            artwork.size(),
        );
        let layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: None,
            entries: &[
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Texture {
                        sample_type: wgpu::TextureSampleType::Float { filterable: true },
                        view_dimension: wgpu::TextureViewDimension::D2,
                        multisampled: false,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 1,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                    count: None,
                },
            ],
        });
        let texture = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: None,
            layout: &layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: wgpu::BindingResource::TextureView(
                        &artwork.create_view(&Default::default()),
                    ),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: wgpu::BindingResource::Sampler(&device.create_sampler(
                        &wgpu::SamplerDescriptor {
                            mag_filter: wgpu::FilterMode::Linear,
                            min_filter: wgpu::FilterMode::Linear,
                            ..Default::default()
                        },
                    )),
                },
            ],
        });
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Armor shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("scene.wgsl").into()),
        });
        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: None,
            bind_group_layouts: &[Some(&layout)],
            immediate_size: 0,
        });
        let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("Depth-tested armor"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs"),
                compilation_options: Default::default(),
                buffers: &[Some(wgpu::VertexBufferLayout {
                    array_stride: std::mem::size_of::<Vertex>() as u64,
                    step_mode: wgpu::VertexStepMode::Vertex,
                    attributes: &wgpu::vertex_attr_array![0=>Float32x4,1=>Float32x4,2=>Float32x2],
                })],
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: Some("fs"),
                compilation_options: Default::default(),
                targets: &[Some(wgpu::ColorTargetState {
                    format: output.format(),
                    blend: Some(wgpu::BlendState::ALPHA_BLENDING),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
            }),
            primitive: wgpu::PrimitiveState {
                cull_mode: None,
                ..Default::default()
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: wgpu::TextureFormat::Depth32Float,
                depth_write_enabled: Some(true),
                depth_compare: Some(wgpu::CompareFunction::LessEqual),
                stencil: Default::default(),
                bias: Default::default(),
            }),
            multisample: wgpu::MultisampleState {
                count: 4,
                ..Default::default()
            },
            multiview_mask: None,
            cache: None,
        });
        let vertices = device.create_buffer(&wgpu::BufferDescriptor {
            label: None,
            size: 4_000_000,
            usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let theme = rxui::Theme::default();
        Ok(Self {
            output,
            depth,
            multisample,
            pipeline,
            texture,
            vertices,
            background: theme.color(rxui::ThemeColor::Background),
            grid: theme.color(rxui::ThemeColor::Border),
            count: 0,
        })
    }

    pub fn image(&self) -> Result<Image, rxui::UiError> {
        Image::from_texture(&self.output)
    }

    pub fn prepare(
        &mut self,
        graphics: &GraphicsContext,
        frame: &Frame,
        config: Config,
        camera: Camera,
        layers: Layers,
    ) {
        let mesh = geometry(frame, config, layers, self.grid);
        self.count = mesh.vertices.len() as u32;
        let matrix = camera.matrix(frame);
        let transformed: Vec<_> = mesh
            .vertices
            .iter()
            .map(|vertex| {
                let mut vertex = *vertex;
                vertex.position = (matrix * Vec4::from_array(vertex.position)).to_array();
                vertex
            })
            .collect();
        let bytes = bytemuck::cast_slice(&transformed);
        if bytes.len() as u64 > self.vertices.size() {
            self.vertices =
                graphics
                    .device()
                    .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                        label: None,
                        contents: bytes,
                        usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
                    });
        } else {
            graphics.queue().write_buffer(&self.vertices, 0, bytes);
        }
    }

    pub fn theme(&mut self, theme: rxui::Theme) {
        self.background = theme.color(rxui::ThemeColor::Background);
        self.grid = theme.color(rxui::ThemeColor::Border);
    }

    pub fn pixels(&self, graphics: &GraphicsContext) -> Result<Vec<u8>> {
        let bytes_per_row = self.output.width() * 4;
        let buffer = graphics.device().create_buffer(&wgpu::BufferDescriptor {
            label: None,
            size: u64::from(bytes_per_row) * u64::from(self.output.height()),
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });
        let mut encoder = graphics
            .device()
            .create_command_encoder(&Default::default());
        self.record(&mut encoder);
        encoder.copy_texture_to_buffer(
            self.output.as_image_copy(),
            wgpu::TexelCopyBufferInfo {
                buffer: &buffer,
                layout: wgpu::TexelCopyBufferLayout {
                    offset: 0,
                    bytes_per_row: Some(bytes_per_row),
                    rows_per_image: Some(self.output.height()),
                },
            },
            self.output.size(),
        );
        graphics.queue().submit([encoder.finish()]);
        let (tx, rx) = std::sync::mpsc::channel();
        buffer
            .slice(..)
            .map_async(wgpu::MapMode::Read, move |result| {
                let _ = tx.send(result);
            });
        graphics.device().poll(wgpu::PollType::Wait {
            submission_index: None,
            timeout: Some(std::time::Duration::from_secs(10)),
        })?;
        rx.recv_timeout(std::time::Duration::from_secs(10))??;
        let pixels = buffer.slice(..).get_mapped_range()?.to_vec();
        buffer.unmap();
        Ok(pixels)
    }

    pub fn record(&self, encoder: &mut wgpu::CommandEncoder) {
        let view = self.output.create_view(&Default::default());
        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("3D viewport pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &self.multisample,
                    resolve_target: Some(&view),
                    depth_slice: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: self.background[0] as f64,
                            g: self.background[1] as f64,
                            b: self.background[2] as f64,
                            a: 1.0,
                        }),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &self.depth,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.0),
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                ..Default::default()
            });
            pass.set_pipeline(&self.pipeline);
            pass.set_bind_group(0, &self.texture, &[]);
            pass.set_vertex_buffer(0, self.vertices.slice(..));
            pass.draw(0..self.count, 0..1);
        }
    }
}

/// Export the native renderer's viewport. ffmpeg is needed only for MP4 output.
pub fn export(options: super::Options) -> Result<()> {
    use std::io::Write;
    let (config, frames) = if let Some(path) = options.replay {
        super::recording::load(&path)?
    } else {
        (options.config, super::generate(options.config)?)
    };
    let graphics = pollster::block_on(GraphicsContext::headless())?;
    let mut renderer = Renderer::with_size(&graphics, EXPORT_SIZE)?;
    let camera = Camera {
        yaw: -2.25,
        elevation: 0.45,
        distance: 2.7,
        focus: true,
        aspect: EXPORT_SIZE[0] as f32 / EXPORT_SIZE[1] as f32,
        ..Camera::default()
    };
    let layers = Layers::default();
    if let Some(path) = options.preview {
        let index = frames
            .partition_point(|frame| frame.time_s < options.at_s)
            .min(frames.len() - 1);
        renderer.prepare(&graphics, &frames[index], config, camera, layers);
        image::save_buffer(
            path,
            &renderer.pixels(&graphics)?,
            EXPORT_SIZE[0],
            EXPORT_SIZE[1],
            image::ColorType::Rgba8,
        )?;
    }
    if let Some(path) = options.video {
        anyhow::ensure!(
            !path.exists(),
            "video already exists; choose another output path"
        );
        let mut encoder = std::process::Command::new("ffmpeg")
            .args([
                "-hide_banner",
                "-loglevel",
                "error",
                "-n",
                "-f",
                "rawvideo",
                "-pix_fmt",
                "rgba",
                "-s",
                "3840x2160",
                "-framerate",
                "60",
                "-i",
                "pipe:0",
                "-an",
                "-c:v",
                "libx264",
                "-preset",
                "medium",
                "-crf",
                "16",
                "-pix_fmt",
                "yuv420p",
                "-movflags",
                "+faststart",
            ])
            .arg(path)
            .stdin(std::process::Stdio::piped())
            .spawn()?;
        let mut pipe = encoder
            .stdin
            .take()
            .ok_or_else(|| anyhow::anyhow!("no video input pipe"))?;
        let end = frames.last().map_or(0., |frame| frame.time_s);
        for frame_index in 0..=(end * EXPORT_FPS as f64) as usize {
            let time = frame_index as f64 / EXPORT_FPS as f64;
            let index = frames
                .partition_point(|frame| frame.time_s < time)
                .min(frames.len() - 1);
            renderer.prepare(&graphics, &frames[index], config, camera, layers);
            pipe.write_all(&renderer.pixels(&graphics)?)?;
        }
        drop(pipe);
        anyhow::ensure!(encoder.wait()?.success(), "ffmpeg export failed");
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn panning_tracks_screen_motion_and_scales_with_zoom() -> Result<()> {
        let mut simulation = crate::sim::Simulation::new(Config::default())?;
        let frame = simulation.step(&mut |_, _| None);
        let point = [3.2, 0.0, 0.2];
        let size = [WIDTH, HEIGHT];
        let mut camera = Camera::default();
        let before = camera
            .project(&frame, point, size)
            .ok_or_else(|| anyhow::anyhow!("camera focus is not visible"))?;
        camera.pan([50.0, -25.0], HEIGHT);
        let after = camera
            .project(&frame, point, size)
            .ok_or_else(|| anyhow::anyhow!("panned focus is not visible"))?;
        assert!((after[0] - before[0] - 50.0).abs() < 0.001);
        assert!((after[1] - before[1] + 25.0).abs() < 0.001);
        let mut farther = Camera {
            distance: camera.distance * 2.0,
            ..Camera::default()
        };
        farther.pan([50.0, -25.0], HEIGHT);
        for (near, far) in camera.pan_m.into_iter().zip(farther.pan_m) {
            assert!((far - 2.0 * near).abs() < 0.00001);
        }
        Ok(())
    }
}
