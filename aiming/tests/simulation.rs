use aiming_sim::{
    Command, Input, Target, Trajectory, physics,
    sim::{Config, Frame, Outcome, Simulation},
};
use std::{error::Error, f64::consts::PI};

fn stationary() -> Config {
    Config {
        target: Target {
            omega_rad_s: 0.0,
            ..Target::default()
        },
        duration_s: 1.0,
        ..Config::default()
    }
}

fn shoot_once(config: Config, command: Command) -> Result<Frame, Box<dyn Error>> {
    let mut simulation = Simulation::new(config)?;
    let mut controller =
        |input: &Input, _: &mut aiming_sim::ui::Gizmos| (input.now_s == 0.0).then_some(command);
    let mut frame = simulation.step(&mut controller);
    while frame.time_s < physics::LAUNCH_DELAY_S + physics::MAX_FLIGHT_S + 0.02 {
        frame = simulation.step(&mut controller);
    }
    Ok(frame)
}

#[test]
fn observations_are_delayed_and_unavailable_at_startup() -> Result<(), Box<dyn Error>> {
    let config = Config::default();
    let mut simulation = Simulation::new(config)?;
    for _ in 0..100 {
        simulation.step(&mut |input, _| {
            if input.now_s < physics::OBSERVATION_DELAY_S {
                assert!(
                    input.observation.is_none(),
                    "observation arrived early at {}",
                    input.now_s
                );
            } else {
                let Some(sample) = input.observation else {
                    assert!(
                        input.now_s < physics::OBSERVATION_DELAY_S,
                        "missing observation at {}",
                        input.now_s
                    );
                    return None;
                };
                assert!((sample.time_s + physics::OBSERVATION_DELAY_S - input.now_s).abs() < 1e-12);
                assert_eq!(sample.target, config.target.at(sample.time_s));
                assert_ne!(
                    sample.target,
                    config.target.at(input.now_s),
                    "current GT leaked"
                );
            }
            None
        });
    }
    Ok(())
}

#[test]
fn gravity_and_launch_delay_produce_a_front_face_hit() -> Result<(), Box<dyn Error>> {
    let config = stationary();
    let distance = config.target.x_m - config.size.radius_m;
    let pitch = (physics::GRAVITY_M_S2 * distance / physics::MUZZLE_SPEED_M_S.powi(2)).asin() / 2.0;
    let frame = shoot_once(
        config,
        Command {
            yaw_rad: 0.0,
            pitch_rad: pitch,
            fire: true,
        },
    )?;
    assert_eq!(frame.stats.hits, 1);
    let Outcome::Hit { time_s, plate } = frame.shots[0].outcome else {
        return Err("expected a hit".into());
    };
    assert_eq!(plate, 2);
    let flight_s = distance / (physics::MUZZLE_SPEED_M_S * pitch.cos());
    assert!((time_s - physics::LAUNCH_DELAY_S - flight_s).abs() < 1e-8);
    let impact = frame.shots[0].position(time_s);
    assert!((impact[2] - config.size.plate_center_z_m).abs() < 1e-8);
    Ok(())
}

#[test]
fn moving_rotating_plate_is_checked_at_impact_time() -> Result<(), Box<dyn Error>> {
    let mut config = stationary();
    let trajectory = Trajectory::solve([5.0, 0.0, 0.3]).ok_or("unreachable point")?;
    let impact_s = physics::LAUNCH_DELAY_S + trajectory.flight_time_s;
    // Plate 0 reaches (5, 0, 0.3) at the projectile's known arrival time.
    config.target = Target {
        x_m: 5.25 - 0.5 * impact_s,
        y_m: -0.5 * impact_s,
        theta_rad: PI - 5.0 * impact_s,
        vx_m_s: 0.5,
        vy_m_s: 0.5,
        omega_rad_s: 5.0,
    };
    let frame = shoot_once(
        config,
        Command {
            yaw_rad: trajectory.yaw_rad,
            pitch_rad: trajectory.pitch_rad,
            fire: true,
        },
    )?;
    let Outcome::Hit { time_s, plate } = frame.shots[0].outcome else {
        return Err("moving plate was not hit".into());
    };
    assert_eq!(plate, 0);
    assert!(
        (time_s - impact_s).abs() < 1e-8,
        "impact at {time_s}, expected {impact_s}"
    );
    Ok(())
}

#[test]
fn uncompensated_drop_and_shots_outside_plate_edges_miss() -> Result<(), Box<dyn Error>> {
    let mut config = stationary();
    let dropped = shoot_once(
        config,
        Command {
            yaw_rad: 0.0,
            pitch_rad: 0.0,
            fire: true,
        },
    )?;
    assert_eq!(dropped.stats.misses, 1);
    let trajectory =
        Trajectory::solve([4.75, 0.08, physics::MUZZLE_M[2]]).ok_or("unreachable edge point")?;
    let edge = shoot_once(
        config,
        Command {
            yaw_rad: trajectory.yaw_rad,
            pitch_rad: trajectory.pitch_rad,
            fire: true,
        },
    )?;
    assert_eq!(
        edge.stats.misses, 1,
        "a shot outside the armor width must miss"
    );
    config.target.x_m = physics::MUZZLE_M[0];
    let back = shoot_once(
        config,
        Command {
            yaw_rad: 0.0,
            pitch_rad: 0.0,
            fire: true,
        },
    )?;
    assert_eq!(
        back.stats.misses, 1,
        "exit through a back face must not score"
    );
    Ok(())
}

#[test]
fn fire_rate_is_limited_and_pending_shots_keep_their_original_angles() -> Result<(), Box<dyn Error>>
{
    let config = stationary();
    let mut simulation = Simulation::new(config)?;
    let mut frame = simulation.step(&mut |_, _| {
        Some(Command {
            yaw_rad: 0.0,
            pitch_rad: 0.0,
            fire: true,
        })
    });
    assert_eq!(frame.shots[0].outcome, Outcome::Pending);
    assert_eq!(frame.shots[0].launch_time_s, physics::LAUNCH_DELAY_S);
    for _ in 0..100 {
        frame = simulation.step(&mut |_, _| {
            Some(Command {
                yaw_rad: PI / 2.0,
                pitch_rad: 0.0,
                fire: true,
            })
        });
    }
    assert_eq!(
        frame.stats.shots, 4,
        "four shots per second, no shot at duration_s"
    );
    assert_eq!(
        frame.shots[0].command.yaw_rad, 0.0,
        "pending aim changed after acceptance"
    );
    Ok(())
}

#[test]
fn runs_are_deterministic_and_invalid_commands_cannot_fire() -> Result<(), Box<dyn Error>> {
    let mut a = Simulation::new(stationary())?;
    let mut b = Simulation::new(stationary())?;
    for _ in 0..100 {
        let mut controller = |_: &Input, _: &mut aiming_sim::ui::Gizmos| {
            Some(Command {
                yaw_rad: 0.0,
                pitch_rad: 0.1,
                fire: true,
            })
        };
        assert_eq!(a.step(&mut controller), b.step(&mut controller));
    }
    let frame = a.step(&mut |_, _| {
        Some(Command {
            yaw_rad: f64::NAN,
            pitch_rad: 0.0,
            fire: true,
        })
    });
    assert_eq!(frame.stats.invalid_commands, 1);
    assert_eq!(frame.stats.shots, 4);
    assert_eq!(frame.command, None);
    let mut config = stationary();
    config.target.omega_rad_s = 1000.0;
    assert!(Simulation::new(config).is_err());
    Ok(())
}

#[test]
fn scenario_json_preserves_values_and_rejects_physics_overrides() -> Result<(), Box<dyn Error>> {
    let mut config = stationary();
    config.target.vy_m_s = 0.09999999999999999;
    let copy: Config = serde_json::from_str(&serde_json::to_string(&config)?)?;
    assert_eq!(copy, config);
    config.target.vx_m_s = f64::NAN;
    assert!(config.validate().is_err());
    config.target.vx_m_s = 0.0;
    config.size.plate_center_z_m = 0.0;
    assert!(config.validate().is_err());
    assert!(serde_json::from_str::<Config>(r#"{"physics":{"gravity_m_s2":0}}"#).is_err());
    assert!(serde_json::from_str::<Config>(r#"{"unknown":{}}"#).is_err());
    Ok(())
}

#[test]
fn tilted_basis_matches_the_main_simulator_mount() {
    let size = aiming_sim::Size::default();
    let target = Target::default();
    for plate in target.plates(size) {
        let dot = |a: [f64; 3], b: [f64; 3]| (0..3).map(|i| a[i] * b[i]).sum::<f64>();
        assert!((plate.outward_normal[2] - 15_f64.to_radians().sin()).abs() < 1e-12);
        assert!((plate.up_axis[2] - 15_f64.to_radians().cos()).abs() < 1e-12);
        assert!(dot(plate.up_axis, plate.outward_normal).abs() < 1e-12);
        assert!(dot(plate.right_axis, plate.outward_normal).abs() < 1e-12);
        let upper = plate.point(0., size.plate_height_m / 2., 0.);
        assert!(
            (upper[0] - target.x_m).hypot(upper[1] - target.y_m) < size.radius_m,
            "top edge must lean toward the target center"
        );
    }
}

#[test]
fn tilted_hit_test_uses_plate_up_instead_of_world_height() -> Result<(), Box<dyn Error>> {
    let mut config = stationary();
    config.size.plate_center_z_m = physics::MUZZLE_M[2] - 0.0278;
    let trajectory =
        Trajectory::solve([4.75, 0.0, physics::MUZZLE_M[2]]).ok_or("unreachable point")?;
    let frame = shoot_once(
        config,
        Command {
            yaw_rad: trajectory.yaw_rad,
            pitch_rad: trajectory.pitch_rad,
            fire: true,
        },
    )?;
    assert_eq!(
        frame.stats.misses, 1,
        "a point outside the tilted rectangle scored"
    );
    Ok(())
}

#[test]
fn student_template_draws_observed_plates_without_firing() -> Result<(), Box<dyn Error>> {
    let mut simulation = Simulation::new(Config::default())?;
    for _ in 0..100 {
        let frame = simulation.step(&mut aiming_sim::aim);
        assert!(frame.command.is_none());
        assert_eq!(frame.stats.shots, 0);
        assert!(
            frame
                .gizmos
                .overlays
                .contains(&("Hit rate".to_owned(), "0/0".to_owned()))
        );
        if frame.input.observation.is_some() {
            assert!(!frame.gizmos.lines.is_empty());
        }
        assert!(
            frame.gizmos.points.is_empty(),
            "plate drawings must not add labels"
        );
    }
    Ok(())
}
