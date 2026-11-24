use std::f64::consts::{PI, TAU};

use nalgebra_glm::{vec2, vec3, DVec2, DVec3, Vec3, Vec4};
use rand::Rng;

use crate::{
    airplane::{air_density, Airplane, SEA_LEVEL_AIR_DENSITY},
    hud::{Mood, Notice},
    judge::{DebriefRow, Grade, Marker, Objective},
    scenario::RescueSetup,
};

// Find somewhere to land
const CLEARING_TRIES: usize = 5000;
const CLEARING_SAMPLE_SPACING: f64 = 10.0; // [m]
const MAX_CLEARING_SLOPE: f64 = 0.052;
const MAX_CLEARING_RISE: f32 = 5.0; // [m]
const MIN_CLEARING_ELEVATION: f32 = 2.0; // [m]
const HIKER_SIDE_OFFSET: f64 = 25.0;

// Picking them up
const PICKUP_RADIUS: f64 = 100.0; // [m]
const STOPPED_SPEED: f64 = 1.0; // [m/s]
const BOARDING_TIME: f64 = 8.0; // [s]
const ABOARD_NOTICE_TIME: f64 = 6.0; // [s]

// Visuals
const HIKER_SIZE: Vec3 = Vec3::new(0.5, 1.8, 0.4); // [m]
const HIKER_COLOR: Vec4 = Vec4::new(0.9, 0.3, 0.1, 1.0);
const SIGNAL_PANEL_SIZE: Vec3 = Vec3::new(4.0, 0.1, 4.0); // [m]
const SIGNAL_PANEL_COLOR: Vec4 = Vec4::new(1.0, 0.5, 0.0, 1.0);

#[derive(Clone, Copy)]
enum Phase {
    Waiting,
    Boarding(f64), // [s]
    Aboard(f64),   // [s]
}

pub struct Rescue {
    /// Where they wait [m, world]
    spot: DVec3,
    num_hikers: u32,
    kg_per_hiker: f64,
    /// Index of the plane's station they sit in
    station: usize,
    /// Runway to fly them to
    destination: String,
    phase: Phase,
    /// Time since the flight began [s]
    clock: f64,
}

impl Rescue {
    pub fn new(
        setup: &RescueSetup,
        area: DVec2,
        plane: &Airplane,
        destination: &str,
        ground: &impl Fn(f64, f64) -> (f32, Vec3),
        rng: &mut impl Rng,
    ) -> Result<Self, String> {
        let station = plane
            .station(&setup.station)
            .ok_or_else(|| format!("the plane has no station {}", setup.station))?;

        let sea_level_length = setup.clearing_length_m.unwrap_or(plane.field_length());
        let clearing = find_clearing(area, setup.search_radius_m, sea_level_length, ground, rng)
            .ok_or_else(|| {
                format!(
                    "nowhere flat within {:.0}m of {}",
                    setup.search_radius_m, setup.area
                )
            })?;

        // figure out where the hikers actually stand
        let end = if rng.gen::<bool>() {
            clearing.dir
        } else {
            -clearing.dir
        };
        let side = vec2(-clearing.dir.y, clearing.dir.x);
        let spot = clearing.middle + end * clearing.length / 2.0 + side * HIKER_SIDE_OFFSET;
        let (height, _) = ground(spot.x, spot.y);

        Ok(Self {
            spot: vec3(spot.x, height as f64, spot.y),
            num_hikers: setup.num_hikers,
            kg_per_hiker: setup.kg_per_hiker,
            station,
            destination: destination.to_string(),
            phase: Phase::Waiting,
            clock: 0.0,
        })
    }

    /// Where the hikers are, on the ground [m, world]
    pub fn spot(&self) -> DVec3 {
        self.spot
    }
}

impl Objective for Rescue {
    fn update(&mut self, plane: &mut Airplane, dt: f64) {
        self.clock += dt;
        let here = vec2(plane.pos.x, plane.pos.z);
        let ground_speed = vec2(plane.vel.x, plane.vel.z).norm();
        let stopped_nearby = plane.contact().wheels
            && ground_speed < STOPPED_SPEED
            && (here - vec2(self.spot.x, self.spot.z)).norm() < PICKUP_RADIUS;

        // phase state machine
        self.phase = match self.phase {
            Phase::Waiting if stopped_nearby => Phase::Boarding(0.0),
            Phase::Boarding(_) if !stopped_nearby => Phase::Waiting,
            Phase::Boarding(t) if t + dt < BOARDING_TIME => Phase::Boarding(t + dt),
            Phase::Boarding(_) => {
                plane.add_station_load(self.station, self.num_hikers as f64 * self.kg_per_hiker);
                Phase::Aboard(self.clock)
            }
            phase => phase,
        };
    }

    fn rows(&self) -> Vec<DebriefRow> {
        let row = match self.phase {
            Phase::Aboard(at) => {
                let minutes = at / 60.0;
                DebriefRow::new(
                    "Hikers",
                    format!("picked up after {minutes:.0} min"),
                    Grade::by(minutes, 10.0, 20.0, f64::INFINITY),
                    "",
                )
            }
            _ => DebriefRow::new("Hikers", "left behind", Grade::Fail, "Left them out there"),
        };
        vec![row]
    }

    fn finished(&self) -> bool {
        matches!(self.phase, Phase::Aboard(_))
    }

    fn notice(&self) -> Option<Notice> {
        match self.phase {
            Phase::Boarding(_) => Some(Notice {
                title: "Hikers boarding".to_string(),
                lines: vec![("Hold it here".to_string(), Mood::Plain)],
            }),
            Phase::Aboard(at) if self.clock - at < ABOARD_NOTICE_TIME => Some(Notice {
                title: "Hikers aboard".to_string(),
                lines: vec![(format!("Fly them to {}", self.destination), Mood::Good)],
            }),
            _ => None,
        }
    }

    fn markers(&self) -> Vec<Marker> {
        if let Phase::Aboard(_) = self.phase {
            return Vec::new(); // they're in the plane
        }

        // A signal panel on the ground
        // TODO: Replace with an SOS billboard
        let panel = Marker {
            center: self.spot + vec3(0.0, SIGNAL_PANEL_SIZE.y as f64 / 2.0, 0.0),
            size: SIGNAL_PANEL_SIZE,
            color: SIGNAL_PANEL_COLOR,
        };

        // the hikers all in a row
        let row_middle = (self.num_hikers as f64 - 1.0) / 2.0;
        let hikers = (0..self.num_hikers).map(|i| Marker {
            center: self.spot + vec3(i as f64 - row_middle, HIKER_SIZE.y as f64 / 2.0, 3.0),
            size: HIKER_SIZE,
            color: HIKER_COLOR,
        });

        std::iter::once(panel).chain(hikers).collect()
    }
}

struct Clearing {
    middle: DVec2,
    dir: DVec2,
    length: f64,
}

/// Find a nice flat place that's possible to land on, along some dir
fn find_clearing(
    center: DVec2,
    radius: f64,
    sea_level_length: f64,
    ground: &impl Fn(f64, f64) -> (f32, Vec3),
    rng: &mut impl Rng,
) -> Option<Clearing> {
    (0..CLEARING_TRIES).find_map(|_| {
        let distance = radius * rng.gen::<f64>().sqrt();
        let bearing = rng.gen_range(0.0..TAU);
        let middle = center + vec2(bearing.cos(), bearing.sin()) * distance;
        let heading = rng.gen_range(0.0..PI);
        let dir = vec2(heading.cos(), heading.sin());

        let (height, _) = ground(middle.x, middle.y);
        let thin_air = SEA_LEVEL_AIR_DENSITY / air_density(height as f64);
        let length = sea_level_length * thin_air * thin_air;

        is_clear(middle, dir, length, ground).then_some(Clearing {
            middle,
            dir,
            length,
        })
    })
}

fn is_clear(
    middle: DVec2,
    dir: DVec2,
    length: f64,
    ground: &impl Fn(f64, f64) -> (f32, Vec3),
) -> bool {
    let samples = (length / CLEARING_SAMPLE_SPACING).ceil() as usize;
    let min_normal_y = MAX_CLEARING_SLOPE.cos() as f32;
    let heights: Option<Vec<f32>> = (0..=samples)
        .map(|i| {
            let p = middle + dir * length * (i as f64 / samples as f64 - 0.5);
            let (height, normal) = ground(p.x, p.y);
            (normal.y >= min_normal_y && height > MIN_CLEARING_ELEVATION).then_some(height)
        })
        .collect();
    heights.is_some_and(|heights| {
        let lowest = heights.iter().copied().fold(f32::INFINITY, f32::min);
        let highest = heights.iter().copied().fold(f32::NEG_INFINITY, f32::max);
        highest - lowest <= MAX_CLEARING_RISE
    })
}

#[cfg(test)]
mod tests {
    use rand::{rngs::StdRng, SeedableRng};

    use super::*;

    /// Flat at 100 m east of x = 1000, a steep slope west of it
    fn ground(x: f64, _z: f64) -> (f32, Vec3) {
        if x > 1000.0 {
            (100.0, vec3(0.0, 1.0, 0.0))
        } else {
            ((100.0 + (1000.0 - x) * 0.5) as f32, vec3(0.45, 0.89, 0.0))
        }
    }

    #[test]
    fn clearings_are_on_the_flat() {
        let mut rng = StdRng::seed_from_u64(1);
        for _ in 0..20 {
            let clearing = find_clearing(vec2(1000.0, 0.0), 2000.0, 300.0, &ground, &mut rng)
                .expect("half the disc is flat");
            let half = clearing.dir * clearing.length / 2.0;
            for end in [clearing.middle - half, clearing.middle + half] {
                assert!(end.x > 1000.0, "an end of the strip is on the slope");
            }
        }
    }

    #[test]
    fn no_clearing_on_a_mountainside() {
        let mut rng = StdRng::seed_from_u64(1);
        assert!(find_clearing(vec2(-5000.0, 0.0), 1000.0, 300.0, &ground, &mut rng).is_none());
    }
}
