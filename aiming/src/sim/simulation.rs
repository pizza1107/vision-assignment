// Simulator infrastructure. Do not edit; implement your solution in src/lib.rs.
//! Evaluator-owned timing, projectile physics, and moving armor collision checks.
use crate::{Command, Input, Sample, Size, Stats, Target, physics, ui::Gizmos};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use std::{f64::consts::FRAC_PI_2, io};

/// Collision brackets are at most 1 ms wide; roots are refined by bisection.
const COLLISION_STEP_S: f64 = 0.001;

/// Scenario inputs. Defaults can be overridden individually in a JSON file.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(default, deny_unknown_fields)]
pub struct Config {
    pub target: Target,
    pub size: Size,
    pub duration_s: f64,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            target: Target {
                vy_m_s: 0.15,
                omega_rad_s: 2.0,
                ..Target::default()
            },
            size: Size::default(),
            duration_s: 20.0,
        }
    }
}

impl Config {
    /// Bound scenario values so collision resolution and runtime remain predictable.
    ///
    /// # Errors
    /// Rejects nonfinite values, nonphysical geometry, or values beyond the documented limits.
    pub fn validate(&self) -> io::Result<()> {
        let t = self.target;
        if [t.x_m, t.y_m, t.theta_rad, t.vx_m_s, t.vy_m_s, t.omega_rad_s]
            .iter()
            .any(|value| !value.is_finite())
            || !(0.01..=300.0).contains(&self.duration_s)
            || self.target.x_m.abs() > 100.0
            || self.target.y_m.abs() > 100.0
            || self.target.vx_m_s.abs() > 5.0
            || self.target.vy_m_s.abs() > 5.0
            || self.target.omega_rad_s.abs() > 20.0
            || !(0.0..=0.5).contains(&self.size.mount_pitch_rad)
            || !(0.1..=1.0).contains(&self.size.radius_m)
            || !(0.02..=0.5).contains(&self.size.plate_width_m)
            || !(0.02..=0.5).contains(&self.size.plate_height_m)
            || !(self.size.plate_height_m / 2.0..=2.0).contains(&self.size.plate_center_z_m)
        {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "scenario exceeds the supported geometry or motion limits",
            ));
        }
        Ok(())
    }
}

impl Command {
    fn valid(self) -> bool {
        self.yaw_rad.is_finite()
            && self.pitch_rad.is_finite()
            && (-FRAC_PI_2..=FRAC_PI_2).contains(&self.pitch_rad)
    }
}

/// A shot is pending until launch, then ends at a plate, the ground, or its flight limit.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(tag = "status", rename_all = "snake_case")]
pub enum Outcome {
    Pending,
    Flying,
    Hit { time_s: f64, plate: usize },
    Miss { time_s: f64 },
}

/// One evaluator-owned projectile. Positions use the analytic gravity-only trajectory.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct Shot {
    pub id: usize,
    pub launch_time_s: f64,
    pub command: Command,
    pub outcome: Outcome,
}

impl Shot {
    /// Evaluate the projectile after launch, including positions past an impact for diagnostics.
    pub fn position(self, time_s: f64) -> [f64; 3] {
        let elapsed = (time_s - self.launch_time_s).max(0.0);
        let horizontal = physics::MUZZLE_SPEED_M_S * self.command.pitch_rad.cos();
        [
            physics::MUZZLE_M[0] + horizontal * self.command.yaw_rad.cos() * elapsed,
            physics::MUZZLE_M[1] + horizontal * self.command.yaw_rad.sin() * elapsed,
            physics::MUZZLE_M[2]
                + physics::MUZZLE_SPEED_M_S * self.command.pitch_rad.sin() * elapsed
                - 0.5 * physics::GRAVITY_M_S2 * elapsed * elapsed,
        ]
    }

    /// Time of ground contact or flight timeout, excluding armor collisions.
    pub fn miss_time(self) -> f64 {
        ground_time(self).min(self.launch_time_s + physics::MAX_FLIGHT_S)
    }

    /// Last event time, or None while launch/flight is still pending.
    pub fn end_time(self) -> Option<f64> {
        match self.outcome {
            Outcome::Hit { time_s, .. } | Outcome::Miss { time_s } => Some(time_s),
            _ => None,
        }
    }
}

/// Full evaluator telemetry. Current target ground truth never enters Input.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct Frame {
    pub time_s: f64,
    pub target: Target,
    pub input: Input,
    pub command: Option<Command>,
    pub shots: Vec<Shot>,
    pub stats: Stats,
    #[serde(default)]
    pub gizmos: Gizmos,
}

/// Deterministic simulation. Callback execution time does not change simulation time.
#[derive(Debug)]
pub struct Simulation {
    config: Config,
    tick: u64,
    shots: Vec<Shot>,
    next_fire_s: f64,
    invalid_commands: usize,
}

impl Simulation {
    /// # Errors
    /// Returns an error when the scenario fails Config::validate.
    pub fn new(config: Config) -> io::Result<Self> {
        config.validate()?;
        Ok(Self {
            config,
            tick: 0,
            shots: Vec::new(),
            next_fire_s: 0.0,
            invalid_commands: 0,
        })
    }

    /// Call the controller once and advance the evaluator by one control tick.
    /// Return None from the controller after the scenario to drain outstanding shots.
    pub fn step(
        &mut self,
        controller: &mut impl FnMut(&Input, &mut Gizmos) -> Option<Command>,
    ) -> Frame {
        let now_s = self.tick as f64 * physics::CONTROL_PERIOD_S;
        let previous_s = self.tick.saturating_sub(1) as f64 * physics::CONTROL_PERIOD_S;
        for shot in &mut self.shots {
            advance_shot(shot, self.config, previous_s, now_s);
        }
        let mut stats = Stats {
            shots: self.shots.len(),
            hits: self
                .shots
                .iter()
                .filter(|s| matches!(s.outcome, Outcome::Hit { .. }))
                .count(),
            misses: self
                .shots
                .iter()
                .filter(|s| matches!(s.outcome, Outcome::Miss { .. }))
                .count(),
            invalid_commands: self.invalid_commands,
        };
        let capture_s = now_s - physics::OBSERVATION_DELAY_S;
        let input = Input {
            now_s,
            observation: (capture_s >= 0.0).then(|| Sample {
                time_s: capture_s,
                target: self.config.target.at(capture_s),
            }),
            size: self.config.size,
            stats,
        };
        let mut gizmos = Gizmos::default();
        let command = controller(&input, &mut gizmos);
        if let Some(command) = command {
            if !command.valid() {
                self.invalid_commands += 1;
            } else if command.fire
                && now_s + 1e-12 >= self.next_fire_s
                && now_s < self.config.duration_s
            {
                self.shots.push(Shot {
                    id: self.shots.len(),
                    launch_time_s: now_s + physics::LAUNCH_DELAY_S,
                    command,
                    outcome: Outcome::Pending,
                });
                self.next_fire_s = now_s + physics::FIRE_PERIOD_S;
            }
        }
        stats.shots = self.shots.len();
        stats.invalid_commands = self.invalid_commands;
        self.tick += 1;
        Frame {
            time_s: now_s,
            target: self.config.target.at(now_s),
            input,
            command: command.filter(|c| c.valid()),
            shots: self.shots.clone(),
            stats,
            gizmos,
        }
    }
}

fn advance_shot(shot: &mut Shot, config: Config, previous_s: f64, now_s: f64) {
    if shot.end_time().is_some() || now_s < shot.launch_time_s {
        return;
    }
    shot.outcome = Outcome::Flying;
    let mut begin = previous_s.max(shot.launch_time_s);
    let limit = shot.miss_time();
    let end = now_s.min(limit);
    while begin < end {
        let finish = (begin + COLLISION_STEP_S).min(end);
        let mut hit = None;
        for slot in 0..4 {
            if let Some(time_s) = plate_hit(*shot, config, slot, begin, finish)
                && hit.is_none_or(|(earlier, _)| time_s < earlier)
            {
                hit = Some((time_s, slot));
            }
        }
        if let Some((time_s, plate)) = hit {
            shot.outcome = Outcome::Hit { time_s, plate };
            return;
        }
        begin = finish;
    }
    if now_s >= limit {
        shot.outcome = Outcome::Miss { time_s: limit };
    }
}

fn ground_time(shot: Shot) -> f64 {
    let vz = physics::MUZZLE_SPEED_M_S * shot.command.pitch_rad.sin();
    let z = physics::MUZZLE_M[2];
    let elapsed = (vz + (vz * vz + 2.0 * physics::GRAVITY_M_S2 * z).sqrt()) / physics::GRAVITY_M_S2;
    shot.launch_time_s + elapsed
}

fn plate_hit(shot: Shot, config: Config, slot: usize, begin: f64, end: f64) -> Option<f64> {
    let distance = |time_s| {
        let plate = config.target.at(time_s).plates(config.size)[slot];
        let point = shot.position(time_s);
        (0..3)
            .map(|i| (point[i] - plate.center_m[i]) * plate.outward_normal[i])
            .sum::<f64>()
    };
    // Only entry from the outward face counts. Body and back faces are not scoring surfaces.
    if distance(begin) <= 0.0 || distance(end) > 0.0 {
        return None;
    }
    let (mut low, mut high) = (begin, end);
    for _ in 0..24 {
        let mid = (low + high) / 2.0;
        if distance(mid) > 0.0 {
            low = mid;
        } else {
            high = mid;
        }
    }
    let time_s = (low + high) / 2.0;
    let plate = config.target.at(time_s).plates(config.size)[slot];
    let point = shot.position(time_s);
    let right = (0..3)
        .map(|i| (point[i] - plate.center_m[i]) * plate.right_axis[i])
        .sum::<f64>();
    let up = (0..3)
        .map(|i| (point[i] - plate.center_m[i]) * plate.up_axis[i])
        .sum::<f64>();
    (right.abs() <= config.size.plate_width_m / 2.0 && up.abs() <= config.size.plate_height_m / 2.0)
        .then_some(time_s)
}
