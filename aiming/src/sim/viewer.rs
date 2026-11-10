// Simulator infrastructure. Do not edit; implement your solution in src/lib.rs.
//! A full-window 3D viewport with native RXUI controls and frame-local overlays.
use super::{
    Config, Frame, Options, Simulation, recording,
    render::{Camera, Layers, Renderer},
};
use crate::{Target, physics};
use anyhow::Result;
use rxui::prelude::*;
use std::{
    cell::RefCell,
    path::PathBuf,
    rc::Rc,
    time::{Duration, Instant},
};

struct Debugger {
    config: Config,
    simulation: Option<Simulation>,
    frames: Vec<Frame>,
    cursor: usize,
    playing: bool,
    elapsed_s: f64,
    last_tick: Instant,
    speed: f32,
    selected_shot: Option<usize>,
    camera: Camera,
    layers: Layers,
    drag: Option<(Camera, bool)>,
    viewport: [f32; 2],
    show_overlay: bool,
    tools: bool,
    settings: bool,
    image: Option<Image>,
    path: String,
    status: String,
}

impl Debugger {
    fn new(options: Options) -> Result<Self> {
        let mut state = Self {
            config: options.config,
            simulation: None,
            frames: Vec::new(),
            cursor: 0,
            playing: false,
            elapsed_s: 0.,
            last_tick: Instant::now(),
            speed: 1.,
            selected_shot: None,
            camera: Camera::default(),
            layers: Layers::default(),
            drag: None,
            viewport: [1320., 820.],
            show_overlay: true,
            tools: false,
            settings: false,
            image: None,
            path: options.record.to_string_lossy().into(),
            status: String::new(),
        };
        if let Some(path) = options.replay {
            state.path = path.to_string_lossy().into();
            state.load()?;
            state.cursor = state
                .frames
                .partition_point(|f| f.time_s < options.at_s)
                .min(state.frames.len() - 1);
        } else {
            state.reset()?;
        }
        Ok(state)
    }

    fn reset(&mut self) -> Result<()> {
        let mut simulation = Simulation::new(self.config)?;
        let frame = simulation.step(&mut crate::aim);
        self.frames = vec![frame];
        self.cursor = 0;
        self.simulation = Some(simulation);
        self.playing = false;
        self.elapsed_s = 0.;
        self.selected_shot = None;
        self.status.clear();
        Ok(())
    }

    fn load(&mut self) -> Result<()> {
        let (config, frames) = recording::load(&PathBuf::from(&self.path))?;
        self.config = config;
        self.frames = frames;
        self.cursor = 0;
        self.simulation = None;
        self.playing = false;
        self.elapsed_s = 0.;
        self.selected_shot = None;
        self.status.clear();
        Ok(())
    }

    fn attempt(&mut self, result: Result<()>) {
        if let Err(error) = result {
            self.status = error.to_string();
            self.playing = false;
        }
    }

    fn schedule(cx: &mut Context<'_, Self>) {
        cx.spawn(
            async { rxui::sleep(Duration::from_millis(16)).await },
            |s, _, cx| {
                let now = Instant::now();
                let elapsed = now.duration_since(s.last_tick).as_secs_f64();
                s.last_tick = now;
                if s.playing {
                    s.elapsed_s += elapsed.min(0.1) * f64::from(s.speed);
                    while s.elapsed_s >= physics::CONTROL_PERIOD_S && s.playing {
                        s.elapsed_s -= physics::CONTROL_PERIOD_S;
                        s.step();
                    }
                }
                Self::schedule(cx);
            },
        )
        .detach();
    }

    fn step(&mut self) {
        if self.cursor + 1 < self.frames.len() {
            self.cursor += 1;
            return;
        }
        let Some(simulation) = &mut self.simulation else {
            self.playing = false;
            return;
        };
        let duration = self.config.duration_s;
        let frame = simulation
            .step(&mut |input, debug| crate::aim(input, debug).filter(|_| input.now_s < duration));
        let finished = frame.time_s >= duration + physics::LAUNCH_DELAY_S + physics::MAX_FLIGHT_S;
        self.frames.push(frame);
        self.cursor += 1;
        if finished {
            self.simulation = None;
            self.playing = false;
        }
    }

    fn inspect_shot(&mut self, id: usize) {
        let Some(shot) = self.frames.last().and_then(|frame| frame.shots.get(id)) else {
            return;
        };
        let accepted = shot.launch_time_s - physics::LAUNCH_DELAY_S;
        self.cursor = self
            .frames
            .partition_point(|frame| frame.time_s < accepted - 1e-8)
            .min(self.frames.len() - 1);
        self.selected_shot = Some(id);
        self.playing = false;
        self.elapsed_s = 0.;
    }

    fn frame(&self) -> &Frame {
        &self.frames[self.cursor]
    }

    fn display_frame(&self) -> Frame {
        let mut frame = self.frame().clone();
        if let Some(id) = self.selected_shot
            && let Some(shot) = self.frames.last().and_then(|f| f.shots.get(id))
        {
            let accepted = shot.launch_time_s - physics::LAUNCH_DELAY_S;
            let index = self
                .frames
                .partition_point(|f| f.time_s < accepted - 1e-8)
                .min(self.frames.len() - 1);
            frame.gizmos = self.frames[index].gizmos.clone();
            frame.command = Some(shot.command);
        }
        frame
    }

    fn scenario(&mut self, spin: f64, velocity: f64) {
        self.config.target = Target {
            omega_rad_s: spin,
            vy_m_s: velocity,
            ..Target::default()
        };
        let result = self.reset();
        self.attempt(result);
    }
}

impl View for Debugger {
    fn view(&self, cx: &mut ViewContext<'_, Self>) -> impl IntoElement {
        let frame = self.frame();
        let display = self.display_frame();
        let shot_count = self.frames.last().map_or(0, |f| f.shots.len());
        let mut scene = stack()
            .fill_width()
            .fill_height()
            .background(ThemeColor::Background);
        if let Some(source) = &self.image {
            scene = scene.child(
                image(source.clone())
                    .width(self.viewport[0])
                    .height(self.viewport[1])
                    .fit(ImageFit::Stretch)
                    .key("viewport")
                    .accessibility_label("3D aiming scene")
                    .on_pointer_down(cx.listener(|s, e: &PointerInput, _| {
                        let pan = e.modifiers.shift
                            || e.button == Some(PointerButton::Secondary)
                            || e.button == Some(PointerButton::Middle);
                        s.drag = Some((s.camera, pan));
                        e.capture_pointer();
                        e.prevent_default();
                    }))
                    .on_pointer_move(cx.listener(|s, e: &PointerInput, _| {
                        if let Some((origin, pan)) = s.drag
                            && let Some(delta) = e.drag_delta()
                        {
                            s.camera = origin;
                            if pan {
                                s.camera.pan(delta, s.viewport[1]);
                            } else {
                                s.camera.yaw = origin.yaw - delta[0] * 0.006;
                                s.camera.elevation =
                                    (origin.elevation + delta[1] * 0.005).clamp(0.05, 1.5);
                            }
                        }
                    }))
                    .on_pointer_up(cx.listener(|s, _, _| s.drag = None))
                    .on_pointer_cancel(cx.listener(|s, _, _| s.drag = None))
                    .on_wheel(cx.listener(|s, e: &WheelInput, _| {
                        s.camera.distance =
                            (s.camera.distance * (e.delta[1] * 0.002).exp()).clamp(0.4, 30.);
                        e.prevent_default();
                    })),
            );
        }
        if self.layers.gizmos {
            let mut occupied = Vec::<[f32; 2]>::new();
            for point in &display.gizmos.points {
                if let Some([x, y]) = self.camera.project(frame, point.position_m, self.viewport) {
                    let x = (x + 8.).clamp(4., (self.viewport[0] - 180.).max(4.));
                    let mut y = (y - 20.).max(10.);
                    while y > 24.
                        && occupied
                            .iter()
                            .any(|p| (p[0] - x).abs() < 170. && (p[1] - y).abs() < 22.)
                    {
                        y -= 23.;
                    }
                    occupied.push([x, y]);
                    scene = scene.child(label(&point.label).absolute().left(x).top(y));
                }
            }
        }
        if self.show_overlay {
            let mut overlay = column().absolute().left(16.).top(16.).gap(5.);
            for (name, value) in &display.gizmos.overlays {
                overlay = overlay.child(label(format!("{name}: {value}")));
            }
            scene = scene.child(overlay);
        }
        scene = scene.child(
            row()
                .absolute()
                .left(12.)
                .right(12.)
                .bottom(12.)
                .padding(8.)
                .gap(8.)
                .background(ThemeColor::Surface)
                .radius(6.)
                .child(
                    button(if self.playing { "Pause" } else { "Play" }).on_click(cx.listener(
                        |s, _, _| {
                            s.playing = !s.playing;
                            s.last_tick = Instant::now();
                        },
                    )),
                )
                .child(button("Step").on_click(cx.listener(|s, _, _| {
                    s.playing = false;
                    s.step();
                })))
                .child(button("Reset").on_click(cx.listener(|s, _, _| {
                    let r = s.reset();
                    s.attempt(r);
                })))
                .child(
                    slider(self.cursor as f32)
                        .flex_grow(1.)
                        .min_width(60.)
                        .range(0., self.frames.len().saturating_sub(1).max(1) as f32)
                        .step(1.)
                        .accessibility_label("Simulation timeline")
                        .on_change(cx.listener(|s, e: &SliderEvent, _| {
                            s.playing = false;
                            s.cursor = (e.value as usize).min(s.frames.len() - 1);
                            s.elapsed_s = 0.;
                        })),
                )
                .child(label(format!("{:.3} s", frame.time_s)))
                .child(
                    button("Overlay")
                        .on_click(cx.listener(|s, _, _| s.show_overlay = !s.show_overlay)),
                )
                .child(button("Tools").on_click(cx.listener(|s, _, _| s.tools = !s.tools))),
        );
        if self.tools {
            let mut tools = column()
                .absolute()
                .right(12.)
                .bottom(68.)
                .width(310.)
                .max_height((self.viewport[1] - 100.).max(100.))
                .scroll_y()
                .padding(12.)
                .gap(10.)
                .background(ThemeColor::Surface)
                .radius(6.)
                .child(
                    row()
                        .gap(6.)
                        .child(
                            button("Stationary")
                                .on_click(cx.listener(|s, _, _| s.scenario(0., 0.))),
                        )
                        .child(
                            button("Spinning").on_click(cx.listener(|s, _, _| s.scenario(4., 0.))),
                        )
                        .child(
                            button("Moving").on_click(cx.listener(|s, _, _| s.scenario(2., 0.15))),
                        ),
                )
                .child(
                    row()
                        .gap(6.)
                        .child(
                            button(if self.layers.gizmos {
                                "Hide gizmos"
                            } else {
                                "Show gizmos"
                            })
                            .on_click(cx.listener(|s, _, _| s.layers.gizmos = !s.layers.gizmos)),
                        )
                        .child(
                            button(if self.layers.trails {
                                "Hide trails"
                            } else {
                                "Show trails"
                            })
                            .on_click(cx.listener(|s, _, _| s.layers.trails = !s.layers.trails)),
                        ),
                )
                .child(
                    row()
                        .gap(6.)
                        .child(
                            button(if self.layers.hitboxes {
                                "Hide hitboxes"
                            } else {
                                "Show hitboxes"
                            })
                            .on_click(
                                cx.listener(|s, _, _| s.layers.hitboxes = !s.layers.hitboxes),
                            ),
                        )
                        .child(
                            button(if self.layers.normals {
                                "Hide normals"
                            } else {
                                "Show normals"
                            })
                            .on_click(cx.listener(|s, _, _| s.layers.normals = !s.layers.normals)),
                        ),
                )
                .child(
                    row()
                        .gap(6.)
                        .child(
                            button(if self.camera.focus {
                                "World camera"
                            } else {
                                "Follow target"
                            })
                            .on_click(cx.listener(|s, _, _| s.camera.focus = !s.camera.focus)),
                        )
                        .child(button("Reset camera").on_click(cx.listener(|s, _, _| {
                            s.camera = Camera {
                                aspect: s.viewport[0] / s.viewport[1],
                                ..Camera::default()
                            };
                        }))),
                )
                .child(
                    row()
                        .gap(8.)
                        .child(label("Speed"))
                        .child(
                            slider(self.speed)
                                .width(150.)
                                .range(0.1, 2.)
                                .step(0.1)
                                .on_change(cx.listener(|s, e: &SliderEvent, _| s.speed = e.value)),
                        )
                        .child(label(format!("{:.1}×", self.speed))),
                )
                .child(
                    row()
                        .gap(6.)
                        .child(
                            number_field(cx, self.selected_shot.unwrap_or(0) as f64)
                                .range(0., shot_count.saturating_sub(1) as f64)
                                .step(1.)
                                .disabled(shot_count == 0)
                                .accessibility_label("Shot number")
                                .on_change(cx.listener(|s, e: &NumberEvent, _| {
                                    s.inspect_shot(e.value as usize)
                                })),
                        )
                        .child(button("Inspect shot").disabled(shot_count == 0).on_click(
                            cx.listener(|s, _, _| s.inspect_shot(s.selected_shot.unwrap_or(0))),
                        )),
                )
                .child(
                    button("Current tick").on_click(cx.listener(|s, _, _| s.selected_shot = None)),
                )
                .child(
                    button(if self.settings {
                        "Hide initial state"
                    } else {
                        "Edit initial state"
                    })
                    .on_click(cx.listener(|s, _, _| s.settings = !s.settings)),
                );
            if self.settings {
                let t = self.config.target;
                let values = [t.x_m, t.y_m, t.theta_rad, t.vx_m_s, t.vy_m_s, t.omega_rad_s];
                let limits = [100., 100., std::f64::consts::TAU, 5., 5., 20.];
                for (index, name) in [
                    "X (m)",
                    "Y (m)",
                    "Heading (rad)",
                    "Vx (m/s)",
                    "Vy (m/s)",
                    "Spin (rad/s)",
                ]
                .into_iter()
                .enumerate()
                {
                    tools = tools.child(
                        row().gap(8.).child(label(name).width(110.)).child(
                            number_field(cx, values[index])
                                .range(-limits[index], limits[index])
                                .step(0.1)
                                .on_change(cx.listener(move |s, e: &NumberEvent, _| {
                                    let t = &mut s.config.target;
                                    match index {
                                        0 => t.x_m = e.value,
                                        1 => t.y_m = e.value,
                                        2 => t.theta_rad = e.value,
                                        3 => t.vx_m_s = e.value,
                                        4 => t.vy_m_s = e.value,
                                        _ => t.omega_rad_s = e.value,
                                    }
                                    let r = s.reset();
                                    s.attempt(r);
                                })),
                        ),
                    );
                }
            }
            tools = tools
                .child(
                    text_input(&self.path)
                        .fill_width()
                        .accessibility_label("MCAP path")
                        .on_change(
                            cx.listener(|s, e: &TextChangeEvent, _| s.path = e.value.clone()),
                        ),
                )
                .child(
                    row()
                        .gap(6.)
                        .child(button("Save MCAP").on_click(cx.listener(|s, _, _| {
                            let r = recording::save(&PathBuf::from(&s.path), s.config, &s.frames);
                            if r.is_ok() {
                                s.status = format!("Saved {} frames", s.frames.len());
                            }
                            s.attempt(r);
                        })))
                        .child(button("Load MCAP").on_click(cx.listener(|s, _, _| {
                            let r = s.load();
                            s.attempt(r);
                        }))),
                );
            scene = scene.child(tools);
        }
        if !self.status.is_empty() {
            scene = scene.child(
                label(&self.status)
                    .absolute()
                    .left(16.)
                    .bottom(72.)
                    .padding(8.)
                    .background(ThemeColor::Surface)
                    .radius(6.),
            );
        }
        scene
    }
}

pub fn run(options: Options) -> Result<()> {
    let state = Debugger::new(options)?;
    let model = Rc::new(RefCell::new(None::<Entity<Debugger>>));
    let renderer = Rc::new(RefCell::new(None::<Renderer>));
    let prepare_model = model.clone();
    let prepare_renderer = renderer.clone();
    let render_renderer = renderer.clone();
    Application::new()
        .prepare_graphics(move |graphics, cx| {
            let Some(root) = prepare_model.borrow().clone() else {
                return Ok(());
            };
            let size = graphics.metrics.logical_size();
            let viewport = [(size.width as f32).max(1.), (size.height as f32).max(1.)];
            if root.read(cx).viewport != viewport
                || root.read(cx).camera.aspect != viewport[0] / viewport[1]
            {
                root.update(cx, |s, _| {
                    s.viewport = viewport;
                    s.camera.aspect = viewport[0] / viewport[1];
                });
            }
            let mut slot = prepare_renderer.borrow_mut();
            if slot.is_none() {
                let gpu = Renderer::new(graphics.graphics).map_err(|e| {
                    ApplicationError::Native(Box::new(std::io::Error::other(e.to_string())))
                })?;
                let image = gpu.image()?;
                root.update(cx, |s, _| s.image = Some(image));
                *slot = Some(gpu);
            }
            if let Some(gpu) = slot.as_mut() {
                gpu.theme(graphics.window.theme());
                let state = root.read(cx);
                let mut layers = state.layers;
                layers.selected_shot = state.selected_shot;
                gpu.prepare(
                    graphics.graphics,
                    &state.display_frame(),
                    state.config,
                    state.camera,
                    layers,
                );
            }
            Ok(())
        })
        .render_graphics(move |_, _, frame| {
            if let Some(gpu) = render_renderer.borrow().as_ref() {
                gpu.record(frame.encoder());
            }
            Ok(())
        })
        .run(move |cx| {
            let root = cx.new(|cx| {
                Debugger::schedule(cx);
                state
            });
            *model.borrow_mut() = Some(root.clone());
            cx.open_window(
                WindowOptions::new()
                    .title("Aiming")
                    .title_hidden(true)
                    .size(1320., 820.),
                root,
            )?;
            Ok(())
        })
        .map_err(|error| anyhow::anyhow!(error.to_string()))?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use anyhow::Context as _;

    #[test]
    fn inspecting_a_shot_freezes_its_original_prediction_while_replay_advances() -> Result<()> {
        let config = Config {
            duration_s: 0.6,
            ..Config::default()
        };
        let mut debugger = Debugger::new(Options {
            config,
            record: "unused.mcap".into(),
            replay: None,
            at_s: 0.,
            preview: None,
            video: None,
            offline: false,
        })?;
        debugger.frames.clear();
        let mut simulation = Simulation::new(config)?;
        for _ in 0..100 {
            debugger.frames.push(simulation.step(&mut |input, debug| {
                debug.overlay("Time", input.now_s);
                (input.now_s == 0.0).then_some(crate::Command {
                    yaw_rad: 0.0,
                    pitch_rad: 0.0,
                    fire: true,
                })
            }));
        }
        debugger.simulation = None;
        debugger.inspect_shot(0);
        let initial = debugger.display_frame();
        let shot = debugger
            .frames
            .last()
            .and_then(|f| f.shots.first())
            .context("no shot")?;
        assert!((initial.time_s + physics::LAUNCH_DELAY_S - shot.launch_time_s).abs() < 1e-8);
        assert!(!initial.gizmos.overlays.is_empty());
        for _ in 0..30 {
            debugger.step();
        }
        let later = debugger.display_frame();
        assert!(later.time_s > initial.time_s);
        assert_eq!(later.command, initial.command);
        assert_eq!(later.gizmos, initial.gizmos);
        debugger.selected_shot = None;
        assert_ne!(debugger.display_frame().gizmos, initial.gizmos);
        Ok(())
    }
}
