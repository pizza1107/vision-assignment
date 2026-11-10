// Simulator infrastructure. Do not edit; implement your solution in src/lib.rs.
//! Default scenario values and target model checks.
use crate::{Size, Target};

impl Default for Target {
    fn default() -> Self {
        Self {
            x_m: 5.0,
            y_m: 0.0,
            theta_rad: 0.0,
            vx_m_s: 0.0,
            vy_m_s: 0.0,
            omega_rad_s: 1.0,
        }
    }
}

impl Default for Size {
    fn default() -> Self {
        Self {
            mount_pitch_rad: 15_f64.to_radians(),
            radius_m: 0.25,
            plate_width_m: 0.135,
            plate_height_m: 0.056,
            plate_center_z_m: 0.3,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn motion_is_independent_of_step_size() {
        let target = Target {
            vx_m_s: 0.7,
            vy_m_s: -0.4,
            omega_rad_s: -2.0,
            ..Target::default()
        };
        let direct = target.at(2.0);
        let stepped = target.at(0.5).at(1.5);
        for (got, want) in [
            (stepped.x_m, direct.x_m),
            (stepped.y_m, direct.y_m),
            (stepped.theta_rad, direct.theta_rad),
        ] {
            assert!((got - want).abs() < 1e-12, "stepped {got} != direct {want}");
        }
    }

    #[test]
    fn plate_velocity_matches_motion_and_radius_stays_fixed() {
        let target = Target {
            vx_m_s: 0.7,
            vy_m_s: -0.4,
            ..Target::default()
        };
        let size = Size::default();
        let later = target.at(1e-6).plates(size);
        for (plate, next) in target.plates(size).iter().zip(later) {
            let radius = (plate.center_m[0] - target.x_m).hypot(plate.center_m[1] - target.y_m);
            assert!((radius - size.radius_m).abs() < 1e-12);
            for axis in 0..2 {
                let velocity = (next.center_m[axis] - plate.center_m[axis]) / 1e-6;
                assert!((velocity - plate.velocity_m_s[axis]).abs() < 1e-6);
            }
        }
    }
}
