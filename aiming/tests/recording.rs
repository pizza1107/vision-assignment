use aiming_sim::{physics, sim::Frame};
use std::{error::Error, fs, process::Command};

#[test]
fn cli_records_student_template_drawings() -> Result<(), Box<dyn Error>> {
    let path = std::env::temp_dir().join(format!(
        "aiming-rxui-{}-{}.mcap",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)?
            .as_nanos()
    ));
    let output = Command::new(env!("CARGO_BIN_EXE_aiming-sim"))
        .args(["--offline", "--record"])
        .arg(&path)
        .output()?;
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    let bytes = fs::read(&path)?;
    let mut count = 0;
    let mut previous = 0;
    let mut hits = 0;
    let mut plate_debug_seen = false;
    for message in mcap::MessageStream::new(&bytes)? {
        let message = message?;
        assert!(message.log_time >= previous);
        previous = message.log_time;
        if message.channel.topic == "/sim/frame" {
            let frame: Frame = serde_json::from_slice(&message.data)?;
            let replay: Frame = serde_json::from_slice(&serde_json::to_vec(&frame)?)?;
            assert_eq!(frame, replay, "replay altered a command or drawing");
            plate_debug_seen |= !frame.gizmos.lines.is_empty();
            assert!(
                frame.gizmos.points.is_empty(),
                "plate labels must be opt-in"
            );
            assert!(
                frame
                    .gizmos
                    .overlays
                    .iter()
                    .any(|(name, _)| name == "Hit rate")
            );
            if let Some(observation) = frame.input.observation {
                assert!(
                    (frame.time_s - observation.time_s - physics::OBSERVATION_DELAY_S).abs()
                        < 1e-10
                );
            }
            count += 1;
            hits = frame.stats.hits;
        }
    }
    fs::remove_file(path)?;
    assert!(count > 2000);
    assert!(plate_debug_seen, "plate drawings were not recorded");
    assert_eq!(hits, 0, "unimplemented template must hold fire");
    Ok(())
}
