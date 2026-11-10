# Aiming assignment

[中文](README.md) · [English](README.en.md)

This is one of two optional paths. **Choose detection or aiming; you do not need to do both.**
See the [main README](../README.en.md) for the gimbal integration scenario.
This path uses a simulator and needs no detector, physical gimbal, camera, or motors.

Aim and fire at armor plates on a robot that translates and spins. You receive a perfect target
model, but it describes the past. Your job is to predict where a plate will be when the projectile
arrives, choose a trajectory, and decide when to fire.

Edit [src/lib.rs](src/lib.rs). It contains `aim` and the types you use. Drawing helpers live in
[src/ui.rs](src/ui.rs). Leave `src/sim/` alone; it contains the evaluator, viewer and MCAP support.
The `aim` template holds fire and includes editable plate-drawing and hit-count examples.

## Reference

https://github.com/user-attachments/assets/2a08b028-4c0a-400b-9102-b1cc2989b7e8

[Watch the reference video](assets/tracking-reference.mp4) (4K, 60 FPS, 22 seconds).
It shows a completed solution tracking and firing at a translating, spinning target.
The green outline is the predicted armor pose at impact; the orange curve is the planned
trajectory. Use it as a visual reference while implementing your own solution.

## Setup and hardware

- Install [Rust / rustup](https://www.rust-lang.org/tools/install), **Rust 1.98.1 or newer**.
- Install a native C/C++ compiler and linker: Xcode Command Line Tools on macOS,
  MSVC C++ Build Tools on Windows, or build tools and window-backend development libraries on Linux.
- The 3D viewer needs a desktop session and a wgpu-compatible graphics driver, such as Metal,
  Vulkan, or DX12. A discrete GPU is unnecessary.
- Allow approximately **1 GB RAM for compilation** and **150 MB RAM at runtime**.
  Actual usage varies by environment.
- `ffmpeg` is needed only for MP4 export.

## Run

From the `aiming` directory:

```sh
cargo run --locked
cargo run --locked -- --config scenarios/stationary.json
cargo run --locked -- --config scenarios/spinning.json
cargo run --locked -- --config scenarios/moving.json
```

Start with the stationary target, then try spinning, then translation and spin together.
The crate builds independently, without OpenCV or camera SDKs. The runner always calls your `aim`
function; no completed aiming implementation is included.

## What you implement

```rust
pub fn aim(input: &Input, debug: &mut Gizmos) -> Option<Command>
```

The simulator calls `aim` every 10 ms. `Input` contains the current simulation time, a delayed
observation, known target geometry, and resolved shot counts. Each observation
contains its capture time and `x_m, y_m, theta_rad, vx_m_s, vy_m_s, omega_rad_s`. Observations are
absent until the initial delivery delay has elapsed. Current target ground truth is not an input.

Return `Some(Command { yaw_rad, pitch_rad, fire })`. `fire: false` aims without shooting; `None`
holds fire without an aim command. Accepted shots latch their angles, wait for the launch delay,
then follow a fixed trajectory. Later commands cannot steer them. Continuous fire requests are
rate-limited. Hit rate should use `hits / (hits + misses)`; shots still in flight are unresolved.

You should handle missing observations and unreachable trajectories, compensate for delays,
predict translation and rotation, and choose a plate that can be hit from the front.

## Fixed physics

All launcher physics and timing are constants in `physics` in [src/lib.rs](src/lib.rs).
`Input` and scenario JSON have no physics configuration. The trajectory helper, debug drawing,
viewer and evaluator all use these same constants automatically.

- `MUZZLE_M`: `[0.0, 0.0, 0.3]` m.
- `MUZZLE_SPEED_M_S`: 25 m/s.
- `GRAVITY_M_S2`: 9.81 m/s².
- `OBSERVATION_DELAY_S`: 0.05 s.
- `LAUNCH_DELAY_S`: 0.02 s.
- `FIRE_PERIOD_S`: 0.25 s.
- `MAX_FLIGHT_S`: 2 s after launch.
- `CONTROL_PERIOD_S`: 0.01 s.

You do not need to pass physics to any helper.

## Step 1: find the latency

Check that `input.observation` exists. Its state is perfect at capture time, not at the current
time. Find its age with `input.now_s - observation.time_s`. Then account for the fixed
`physics::LAUNCH_DELAY_S` between accepting a command and launching the projectile.

The target moves during both delays, and it will keep moving during projectile flight. Keep
observation age, launch delay, and flight time separate so you can add each exactly once.
Missing observations should hold fire. No tracker or state estimation is required.

## Step 2: select an armor plate

Choose a plate before solving its trajectory. Use distance divided by muzzle speed as a rough
flight-time estimate. Predict all four plate poses at observation age + launch delay + that
estimate, then compare their outward normals with the direction from the muzzle. Choose a plate
that faces the incoming shot. The closest plate now may face away by impact time.

Keep the selected index fixed while refining the intercept in step 4. This avoids solving a full
trajectory for every candidate or switching plates midway through refinement. The initial ranking
is approximate, so check the final facing before firing. Hold fire if the chosen plate is
unreachable.

Armor tops tilt inward by 15 degrees. Use the full 3D normal rather than only a horizontal bearing.
A hit must enter the front face inside the 0.135 × 0.056 m scoring rectangle. The visible housing
is larger and does not count as a hit. `Plate::point` uses the plate's right/up/outward axes.

## Step 3: solve the trajectory

World coordinates are +X forward, +Y left, +Z up. Yaw turns from +X toward +Y, and positive pitch
aims upward. Angles use radians, distances metres, and times seconds.

With muzzle position `m`, muzzle speed `v`, yaw `ψ`, pitch `φ`, and gravity `g`, the projectile
position at time `t` **after launch** is:

$$
\begin{aligned}
x(t) &= m_x + v\cos\phi\cos\psi\;t,\\
y(t) &= m_y + v\cos\phi\sin\psi\;t,\\
z(t) &= m_z + v\sin\phi\;t - \tfrac12 gt^2.
\end{aligned}
$$

Horizontal velocity stays constant. Vertical velocity decreases by `g * t`. In the vertical plane
of the shot, at horizontal distance `r`, this becomes:

$$
z(r) = m_z + r\tan\phi - \frac{g r^2}{2v^2\cos^2\phi}.
$$

This is a parabola, or a straight line when gravity is zero. The simulator has no drag or gimbal
slew. More complicated ballistic models are not needed for this assignment.

`Trajectory` groups `yaw_rad`, `pitch_rad`, and `flight_time_s`. The helper
`Trajectory::solve(point_m)` solves the **low arc to a fixed world point** at the
known muzzle speed. It returns `None` if the point is unreachable, the coordinates are nonfinite,
or flight exceeds the fixed lifetime. It does not predict target motion or add any delays.

```rust
let trajectory = Trajectory::solve(aim_point_m)?;
debug.trajectory(trajectory, Color::Orange);
let impact_m = trajectory.position(trajectory.flight_time_s);

Some(Command {
    yaw_rad: trajectory.yaw_rad,
    pitch_rad: trajectory.pitch_rad,
    fire: true, // Decide whether this is a good shot before setting this.
})
```

`position(t)` and `velocity(t)` evaluate the path from launch. Flight time is
neither the observation age nor the absolute impact timestamp.

## Step 4: aim at the future position

Your trajectory aims at a fixed point, but the chosen plate keeps moving. Predict its pose at
impact using observation age, launch delay, and the trajectory's flight time. The future position
depends on flight time, which depends on that position.
A simple way to resolve this is fixed-point iteration:

1. Estimate flight time from distance divided by muzzle speed.
2. Predict the target at observation age + launch delay + estimated flight time.
3. Find the chosen plate's position on that predicted target, keeping the index from step 2.
4. Solve a trajectory to that position and use its flight time as the next estimate.
5. Repeat until the change is small, with a bounded iteration count. Hold fire if it fails.

The prediction interval measured from capture is:

$$
\Delta t = (t_{\mathrm{now}} - t_{\mathrm{capture}})
           + t_{\mathrm{launch\ delay}} + t_{\mathrm{flight}}.
$$

The absolute impact time is `now + launch_delay + flight_time`. Avoid adding observation age
again when computing that timestamp. `Target::at(dt)` predicts constant translation and spin;
`Target::plates(size)` gives the four plate poses at that time. Plate indices remain attached to
the target as it rotates. Predicting the center alone misses the motion caused by spin.

Use a convergence tolerance in seconds and a maximum number of iterations. A converged flight
time should put the projectile endpoint near the predicted armor center. Draw both to check.

Consider the predicted outward normal and the incoming projectile velocity relative to the
plate. `trajectory.velocity(trajectory.flight_time_s)` gives projectile velocity at impact;
`Plate::velocity_m_s` includes both translation and spin. A useful facing score is:

$$
s = -\frac{\mathbf n\cdot(\mathbf v_{\mathrm{projectile}}-
                         \mathbf v_{\mathrm{plate}})}
               {\lVert\mathbf v_{\mathrm{projectile}}-\mathbf v_{\mathrm{plate}}\rVert}.
$$

A larger positive score means a more direct front-face entry. A negative score approaches from
behind. Reject unreachable candidates and consider holding fire near a grazing angle. Changing
the firing threshold trades shot frequency against sensitivity to prediction errors.

## Debugging

Drawing calls belong in your `aim` code. They last for one tick and are stored in MCAP with the
command that produced them:

```rust
use aiming_sim::ui::{Color, Gizmos};

debug.plate(plate, input.size, Color::Cyan);
debug.plate(predicted_plate, input.size, Color::Green);
debug.trajectory(trajectory, Color::Orange);
debug.line(from_m, to_m, Color::Purple);
debug.point("Impact", impact_m, Color::Green);
debug.overlay("Flight", format!("{:.0} ms", trajectory.flight_time_s * 1000.0));
```

`debug.trajectory()` samples the parabola from muzzle to flight time; no drawing loop is needed.
`debug.plate()` draws a scoring rectangle and outward normal without a label. Use `debug.point()`
to add a labeled marker when needed. Helpers ignore nonfinite coordinates and bound drawing counts.

Left-drag orbits; right/middle-drag or Shift-left-drag pans; scroll zooms. The bottom toolbar
provides Play/Pause, Step, Reset, timeline seeking and overlay visibility. Tools contains scenarios,
drawing layers, speed, camera controls, initial-state editing, shot inspection and MCAP save/load.
Inspect shot freezes its original command and drawings while you step through its flight.
Current tick returns to the current diagnostics.

## Record and replay

```sh
cargo run --no-default-features --locked -- --offline --record target/run.mcap
cargo run --locked -- --replay target/run.mcap --at 3.65
```

Recordings contain `/sim/config` and `/sim/frame`: exact inputs, commands, states, shots, counts,
and debug drawings. Replay restores them without rerunning your code. New recordings never
overwrite existing files. Simulation time advances in fixed 10 ms steps, independently of UI speed.

PNG and MP4 exports use the same GPU scene. MP4 requires `ffmpeg`. Exports follow the target
at 3840 × 2160 and 60 FPS; native text labels, overlays and controls are not included.

```sh
cargo run --locked -- --replay target/run.mcap --at 3.65 --preview target/preview.png
cargo run --locked -- --replay target/run.mcap --export-video target/run.mp4
```

The default target has a 0.25 m radius and armor centers at 0.3 m.
Collision crossings are bracketed within 1 ms and refined by bisection; tangencies or multiple
crossings within one bracket can be missed. This is an in-process exercise, not a code sandbox.
Armor artwork uses the existing pattern from the vision simulator.

## Optional further reading

- [平动-自旋耦合目标下的火控拦截时间t解析与数值混合寻优策略](https://bbs.herkules.dev/articles/01M0Z7CK2QE1GT8XY1GNQK53YA)
  discusses analytical and numerical approaches to interception time for translating, spinning
  targets, including Taylor expansion and Newton iteration.
- [弹道解算及云台角度求解算法](https://bbs.herkules.dev/articles/01M2N3NVQSC21BAGE585ZF8DY1)
  discusses ballistic and gimbal-angle solving with quadratic air resistance.

These offer extensions beyond the basic assignment. The simulator uses the gravity-only parabola
above; more complicated ballistic models are unnecessary.
