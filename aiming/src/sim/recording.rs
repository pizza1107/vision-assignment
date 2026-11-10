// Simulator infrastructure. Do not edit; implement your solution in src/lib.rs.
//! Indexed MCAP snapshots, including exact controller inputs, commands, and debug drawings.
use super::{Config, Frame};
use anyhow::{Context, Result, ensure};
use mcap::{Writer, records::MessageHeader};
use schemars::{JsonSchema, generate::SchemaSettings};
use serde::Serialize;
use std::{
    collections::BTreeMap,
    fs,
    io::{BufWriter, Write},
    path::Path,
};

pub fn save(path: &Path, config: Config, frames: &[Frame]) -> Result<()> {
    ensure!(!frames.is_empty(), "there are no frames to save");
    if let Some(parent) = path.parent().filter(|p| !p.as_os_str().is_empty()) {
        fs::create_dir_all(parent)?;
    }
    let mut writer = Writer::new(BufWriter::new(
        fs::File::create_new(path)
            .context("create a new recording; choose a new path to avoid overwriting")?,
    ))?;
    let config_id = channel::<Config>(&mut writer, "/sim/config")?;
    let frame_id = channel::<Frame>(&mut writer, "/sim/frame")?;
    write(&mut writer, config_id, 0, 0.0, &config)?;
    for (sequence, frame) in frames.iter().enumerate() {
        write(&mut writer, frame_id, sequence as u32, frame.time_s, frame)?;
    }
    writer.finish()?;
    writer.into_inner().flush()?;
    Ok(())
}

fn channel<T: JsonSchema>(writer: &mut Writer<BufWriter<fs::File>>, topic: &str) -> Result<u16> {
    let schema = SchemaSettings::draft07()
        .into_generator()
        .into_root_schema_for::<T>();
    let id = writer.add_schema(
        std::any::type_name::<T>(),
        "jsonschema",
        &serde_json::to_vec(&schema)?,
    )?;
    Ok(writer.add_channel(id, topic, "json", &BTreeMap::new())?)
}

fn write(
    writer: &mut Writer<BufWriter<fs::File>>,
    channel_id: u16,
    sequence: u32,
    time_s: f64,
    data: &impl Serialize,
) -> Result<()> {
    let ns = (time_s * 1e9).round() as u64;
    writer.write_to_known_channel(
        &MessageHeader {
            channel_id,
            sequence,
            log_time: ns,
            publish_time: ns,
        },
        &serde_json::to_vec(data)?,
    )?;
    Ok(())
}

pub fn load(path: &Path) -> Result<(Config, Vec<Frame>)> {
    ensure!(
        fs::metadata(path)?.len() <= 256 * 1024 * 1024,
        "recording exceeds 256 MiB"
    );
    let bytes = fs::read(path)?;
    let mut config = None;
    let mut frames = Vec::<Frame>::new();
    for message in mcap::MessageStream::new(&bytes)? {
        let message = message?;
        match message.channel.topic.as_str() {
            "/sim/config" => config = Some(serde_json::from_slice::<Config>(&message.data)?),
            "/sim/frame" => {
                let frame: Frame = serde_json::from_slice(&message.data)?;
                ensure!(
                    frames.len() < 32_000
                        && frame.time_s.is_finite()
                        && frame.time_s >= 0.0
                        && frames
                            .last()
                            .is_none_or(|previous| frame.time_s > previous.time_s),
                    "invalid or excessive frame timeline"
                );
                frames.push(frame);
            }
            _ => {}
        }
    }
    let config = config.context("recording has no simulator configuration")?;
    config.validate()?;
    ensure!(!frames.is_empty(), "recording contains no simulator frames");
    Ok((config, frames))
}
