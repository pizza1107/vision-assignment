//! Optional debug drawing helpers for lines, markers, and text overlays.
use crate::{Plate, Size, Trajectory};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

/// Named diagnostic colors shared by the renderer and recordings.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub enum Color {
    Cyan,
    Orange,
    Green,
    Red,
    Purple,
    White,
}

impl Color {
    /// Linear RGB for scene rendering.
    pub fn rgb(self) -> [f32; 3] {
        match self {
            Self::Cyan => [0.05, 0.8, 1.0],
            Self::Orange => [1.0, 0.45, 0.05],
            Self::Green => [0.15, 1.0, 0.35],
            Self::Red => [1.0, 0.15, 0.2],
            Self::Purple => [0.7, 0.35, 1.0],
            Self::White => [0.9; 3],
        }
    }
}

/// A line in world coordinates, in metres.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct Line {
    pub start_m: [f64; 3],
    pub end_m: [f64; 3],
    pub color: Color,
}

/// A labeled world-space marker, in metres.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct Point {
    pub label: String,
    pub position_m: [f64; 3],
    pub color: Color,
}

/// Drawings last for one controller tick. The viewer restores them when seeking a recording.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct Gizmos {
    pub lines: Vec<Line>,
    pub points: Vec<Point>,
    pub overlays: Vec<(String, String)>,
}

impl Gizmos {
    /// Draw a world-space line. Nonfinite coordinates and excess primitives are ignored.
    pub fn line(&mut self, start_m: [f64; 3], end_m: [f64; 3], color: Color) {
        if self.lines.len() < 2048 && start_m.into_iter().chain(end_m).all(f64::is_finite) {
            self.lines.push(Line {
                start_m,
                end_m,
                color,
            });
        }
    }

    /// Draw connected world-space segments.
    pub fn polyline(&mut self, points_m: &[[f64; 3]], color: Color) {
        for pair in points_m.windows(2) {
            self.line(pair[0], pair[1], color);
        }
    }

    /// Draw the parabola from muzzle to the trajectory's flight time, in 48 segments.
    /// Nonfinite angles and nonpositive or nonfinite durations are ignored.
    pub fn trajectory(&mut self, trajectory: Trajectory, color: Color) {
        if !trajectory.yaw_rad.is_finite()
            || !trajectory.pitch_rad.is_finite()
            || !trajectory.flight_time_s.is_finite()
            || trajectory.flight_time_s <= 0.0
        {
            return;
        }
        let mut previous = trajectory.position(0.0);
        for step in 1..=48 {
            let time_s = trajectory.flight_time_s * f64::from(step) / 48.0;
            let next = trajectory.position(time_s);
            self.line(previous, next, color);
            previous = next;
        }
    }

    /// Draw a labeled marker. Labels are limited to 120 characters.
    pub fn point(&mut self, label: &str, position_m: [f64; 3], color: Color) {
        if self.points.len() < 128 && position_m.into_iter().all(f64::is_finite) {
            self.points.push(Point {
                label: label.chars().take(120).collect(),
                position_m,
                color,
            });
        }
    }

    /// Draw a plate's scoring rectangle and outward normal, without a label.
    pub fn plate(&mut self, plate: Plate, size: Size, color: Color) {
        let w = size.plate_width_m / 2.0;
        let h = size.plate_height_m / 2.0;
        let corners = [[-w, -h], [w, -h], [w, h], [-w, h], [-w, -h]]
            .map(|[right, up]| plate.point(right, up, 0.005));
        self.polyline(&corners, color);
        self.line(plate.center_m, plate.point(0.0, 0.0, 0.12), color);
    }

    /// Add a name/value pair to the scene's text overlay.
    pub fn overlay(&mut self, name: &str, value: impl ToString) {
        if self.overlays.len() < 64 {
            self.overlays.push((
                name.chars().take(120).collect(),
                value.to_string().chars().take(240).collect(),
            ));
        }
    }
}
