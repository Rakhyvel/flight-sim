use std::f64::consts::TAU;

use nalgebra_glm::{vec2, DVec2, DVec3};

/// How close you gotta be to a waypoint for it to count as being counted
const CAPTURE_RADIUS: f64 = 800.0;

#[derive(Clone, Copy)]
pub enum WaypointKind {
    Airport,
    Navaid,
    Fix,
}

#[derive(Clone)]
pub struct Waypoint {
    pub name: String,
    /// World (x, z) [m]
    pub pos: DVec2,
    /// Altitude to cross at [m]
    pub altitude: Option<f64>,
    pub kind: WaypointKind,
}

/// Waypoints flown in order
#[derive(Default)]
pub struct Route {
    pub waypoints: Vec<Waypoint>,
    /// Index to the next waypoint in `waypoints` to fly to
    pub active: usize,
}

impl Route {
    pub fn new(waypoints: Vec<Waypoint>) -> Self {
        Self {
            waypoints,
            active: 0,
        }
    }

    pub fn active(&self) -> Option<&Waypoint> {
        self.waypoints.get(self.active)
    }

    /// The waypoints still to fly, active one first
    pub fn upcoming(&self) -> &[Waypoint] {
        &self.waypoints[self.active.min(self.waypoints.len())..]
    }

    /// Check if the plane has passed the waypoint and advance to the next
    pub fn update(&mut self, plane_pos: DVec3) {
        let Some(wp) = self.active() else { return }; // skip if we're done with this route
        let here = vec2(plane_pos.x, plane_pos.z);
        let close = (wp.pos - here).norm() < CAPTURE_RADIUS;

        // did we fly past it relative to the previous waypoint?
        let passed = self.active > 0 && {
            let from = self.waypoints[self.active - 1].pos;
            let leg = wp.pos - from;
            (here - wp.pos).dot(&leg) > 0.0
        };

        if close || passed {
            println!("completed {}", self.waypoints[self.active].name);
            self.active += 1;
        }
    }
}

/// Get the compass bearing and distance from `from` to `to`
pub fn bearing_distance(from: DVec3, to: DVec2) -> (f64, f64) {
    let d = to - vec2(from.x, from.z);
    (d.x.atan2(-d.y).rem_euclid(TAU), d.norm())
}
