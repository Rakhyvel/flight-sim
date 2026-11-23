use std::f64::consts::PI;

use nalgebra_glm::{vec3, Vec3};
use serde::Deserialize;

use crate::{
    config::AirplaneConfig,
    hud::{Mood, Notice},
};

#[derive(Deserialize)]
pub struct Scenario {
    pub title: String,
    pub description: String,
    /// Name of a region, in regions.toml
    pub region: String,
    /// File name in assets/planes, without the .toml
    pub plane: String,
    /// Local solar time [h]
    pub time_of_day: f64,
    pub start: Start,
    /// Name of the runway to land on
    pub land: String,
}

/// Where the plane is when the scenario begins
#[derive(Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum Start {
    /// Stopped on `runway`, ready to take off
    Runway { runway: String },
    /// Lined up with the landing runway
    Final {
        /// How far out from the runway
        distance_nm: f64,
        /// How far above the runway
        height_ft: f64,
        speed_kt: f64,
    },
    // TODO: Be able to start mid-air at some (lat/lon/alt, vel)
}

impl Scenario {
    /// Read a scenario file
    pub fn load(path: &str) -> Self {
        let text = std::fs::read_to_string(path).unwrap_or_else(|e| panic!("{path}: {e}"));
        toml::from_str(&text).unwrap_or_else(|e| panic!("{path}: {e}"))
    }

    /// The plane to fly, fresh from its file
    pub fn plane_config(&self) -> AirplaneConfig {
        let path = format!("assets/planes/{}.toml", self.plane);
        AirplaneConfig::load(&path).unwrap_or_else(|e| panic!("{path}: {e}"))
    }

    // TODO: Also incorporate latitude and day of year
    /// Get the direction toward the sun given the time of day
    pub fn sun_direction(&self) -> Vec3 {
        const NOON_TILT: f32 = 0.5; // [rad] how far south of straight up the noon sun sits
        let arc = ((self.time_of_day - 6.0) / 12.0 * PI) as f32; // 0 at sunrise, pi at sunset
        vec3(
            arc.cos(),
            arc.sin() * NOON_TILT.cos(),
            arc.sin() * NOON_TILT.sin(),
        )
        .normalize()
    }

    /// The title and description, for before the flight
    pub fn briefing(&self) -> Notice {
        Notice {
            title: self.title.clone(),
            lines: self
                .description
                .lines()
                .map(|line| (line.to_string(), Mood::Plain))
                .collect(),
        }
    }
}
