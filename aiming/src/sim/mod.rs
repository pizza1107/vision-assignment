// Simulator infrastructure. Do not edit; implement your solution in src/lib.rs.
//! Simulator implementation. Students only need to edit `src/lib.rs`.
mod model;
mod recording;
#[cfg(feature = "viewer")]
mod render;
mod simulation;
#[cfg(feature = "viewer")]
mod viewer;

pub use simulation::{Config, Frame, Outcome, Shot, Simulation};

use crate::physics;
use anyhow::{Context, Result, bail};
use std::{env, fs, path::PathBuf};

struct Options {
    config: Config,
    record: PathBuf,
    replay: Option<PathBuf>,
    at_s: f64,
    preview: Option<PathBuf>,
    video: Option<PathBuf>,
    offline: bool,
}

fn options() -> Result<Option<Options>> {
    let mut options = Options {
        config: Config::default(),
        record: "target/aiming.mcap".into(),
        replay: None,
        at_s: 0.0,
        preview: None,
        video: None,
        offline: false,
    };
    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--config" => {
                let path = args.next().context("--config requires a JSON path")?;
                options.config = serde_json::from_reader(
                    fs::File::open(&path).with_context(|| format!("open scenario {path}"))?,
                )?;
            }
            "--record" => options.record = args.next().context("--record requires a path")?.into(),
            "--replay" => {
                options.replay = Some(args.next().context("--replay requires a path")?.into())
            }
            "--at" => options.at_s = args.next().context("--at requires seconds")?.parse()?,
            "--preview" => {
                options.preview = Some(args.next().context("--preview needs PNG path")?.into())
            }
            "--export-video" => {
                options.video = Some(args.next().context("--export-video needs MP4 path")?.into())
            }
            "--offline" => options.offline = true,
            "--help" | "-h" => {
                println!(
                    "aiming-sim [--config FILE.json] [--record NEW.mcap] [--offline]\n\
                    [--replay RUN.mcap] [--at SECONDS]\n\
                    Default: RXUI viewer calling the blank student solution.\n\
                    --offline writes a complete run without opening the debugger."
                );
                return Ok(None);
            }
            _ => bail!("unknown argument {arg}; use --help"),
        }
    }
    options.config.validate()?;
    if !options.at_s.is_finite() || options.at_s < 0.0 {
        bail!("--at must be finite and nonnegative");
    }
    Ok(Some(options))
}

fn generate(config: Config) -> Result<Vec<Frame>> {
    let mut simulation = Simulation::new(config)?;
    let mut frames = Vec::new();
    let end_s = config.duration_s + physics::LAUNCH_DELAY_S + physics::MAX_FLIGHT_S;
    loop {
        let frame = simulation.step(&mut |input, debug| {
            crate::aim(input, debug).filter(|_| input.now_s < config.duration_s)
        });
        let done = frame.time_s >= end_s;
        frames.push(frame);
        if done {
            return Ok(frames);
        }
    }
}

/// Run the simulator CLI, desktop viewer, or headless recording/export.
///
/// # Errors
/// Returns an error if arguments, configuration, recording, or graphics setup fail.
pub fn run() -> Result<()> {
    let Some(options) = options()? else {
        return Ok(());
    };
    if options.preview.is_some() || options.video.is_some() {
        #[cfg(feature = "viewer")]
        {
            return render::export(options);
        }
        #[cfg(not(feature = "viewer"))]
        {
            bail!("enable the viewer feature to export images or video");
        }
    }
    if options.offline {
        let (config, frames) = if let Some(path) = &options.replay {
            recording::load(path)?
        } else {
            (options.config, generate(options.config)?)
        };
        recording::save(&options.record, config, &frames)?;
        if let Some(frame) = frames.last() {
            println!(
                "{} shots, {} hits, {} misses. Saved {}",
                frame.stats.shots,
                frame.stats.hits,
                frame.stats.misses,
                options.record.display()
            );
        }
        return Ok(());
    }
    #[cfg(feature = "viewer")]
    {
        viewer::run(options)
    }
    #[cfg(not(feature = "viewer"))]
    {
        bail!("enable the viewer feature or use --offline")
    }
}
