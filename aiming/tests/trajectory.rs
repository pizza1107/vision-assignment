use aiming_sim::{
    Command, Trajectory, physics,
    sim::{Outcome, Shot},
    ui::{Color, Gizmos},
};

#[test]
fn solved_low_arc_reaches_fixed_points_in_evaluator_physics()
-> Result<(), Box<dyn std::error::Error>> {
    for point in [[6.0, 1.0, 1.5], [-3.0, -5.0, 0.3], [0.0, 0.0, 2.0]] {
        let trajectory = Trajectory::solve(point).ok_or("point should be reachable")?;
        let shot = Shot {
            id: 0,
            launch_time_s: 0.0,
            command: Command {
                yaw_rad: trajectory.yaw_rad,
                pitch_rad: trajectory.pitch_rad,
                fire: true,
            },
            outcome: Outcome::Flying,
        };
        for ((target, actual), drawn) in point
            .into_iter()
            .zip(shot.position(trajectory.flight_time_s))
            .zip(trajectory.position(trajectory.flight_time_s))
        {
            assert!((target - actual).abs() < 1e-10, "solved path missed target");
            assert!(
                (drawn - actual).abs() < 1e-10,
                "model differs from evaluator"
            );
        }
        let speed = trajectory
            .velocity(0.0)
            .iter()
            .map(|v| v * v)
            .sum::<f64>()
            .sqrt();
        assert!((speed - physics::MUZZLE_SPEED_M_S).abs() < 1e-10);
    }
    Ok(())
}

#[test]
fn solver_rejects_unreachable_points_timeouts_and_invalid_inputs() {
    assert!(Trajectory::solve([1000.0, 0.0, 0.3]).is_none());
    assert!(Trajectory::solve([f64::NAN, 0.0, 0.3]).is_none());
    assert!(Trajectory::solve(physics::MUZZLE_M).is_none());
    // Within ballistic range, but the low arc takes longer than the fixed lifetime.
    assert!(Trajectory::solve([60.0, 0.0, 0.3]).is_none());
}

#[test]
fn gizmo_trajectory_matches_shot_from_muzzle_through_impact()
-> Result<(), Box<dyn std::error::Error>> {
    let trajectory = Trajectory::solve([5.0, 1.0, 0.3]).ok_or("reachable target")?;
    let shot = Shot {
        id: 0,
        launch_time_s: 3.0,
        command: Command {
            yaw_rad: trajectory.yaw_rad,
            pitch_rad: trajectory.pitch_rad,
            fire: true,
        },
        outcome: Outcome::Flying,
    };
    let mut debug = Gizmos::default();
    debug.trajectory(trajectory, Color::Orange);
    assert_eq!(debug.lines.len(), 48);
    for (index, line) in debug.lines.iter().enumerate() {
        assert_eq!(line.color, Color::Orange);
        for (step, point) in [(index, line.start_m), (index + 1, line.end_m)] {
            let time = shot.launch_time_s + trajectory.flight_time_s * step as f64 / 48.0;
            for (drawn, actual) in point.into_iter().zip(shot.position(time)) {
                assert!((drawn - actual).abs() < 1e-10);
            }
        }
    }
    let before = debug.clone();
    for flight_time_s in [f64::INFINITY, -1.0] {
        debug.trajectory(
            Trajectory {
                flight_time_s,
                ..trajectory
            },
            Color::Red,
        );
    }
    assert_eq!(
        debug, before,
        "invalid trajectories must not add partial drawings"
    );
    Ok(())
}
