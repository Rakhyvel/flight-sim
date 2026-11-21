use nalgebra_glm::{vec2, Vec2, Vec3};

use crate::{
    airplane::Airplane,
    control::Controls,
    waypoint::{self, Route},
    world::World,
};

/// Snapshot of the current flight data
pub struct FlightData {
    pub pitch: f32,
    pub roll: f32,
    pub heading: f32,
    /// Above sea level [m]
    pub altitude: f32,
    /// Bottom of the wheels above ground [m]
    pub height_above_ground: f32,
    pub airspeed: f32,
    pub ground_speed: f32,
    /// [m/s]
    pub vertical_speed: f32,
    pub throttle: f32,
    pub brakes: f32,
    /// A point the plane is flying toward, relative to the floating origin. None when not moving
    pub flight_path_point: Option<Vec3>,
    pub nav: Option<NavData>,
}

pub struct NavData {
    pub name: String,
    pub next: Option<String>,
    /// Compass bearing to the waypoint [rad]
    pub bearing: f32,
    pub distance: f32,
    /// At this ground speed [s]
    pub time_to_go: Option<f32>,
    /// Altitude to cross at [m]
    pub target_altitude: Option<f32>,
    /// Flight path angle that gets us to the altitude [rad]
    pub target_path: Option<f32>,
    /// The waypoints still to fly, as world (x, z) offsets from the plane [m]
    pub route: Vec<Vec2>,
}

impl FlightData {
    pub fn gather(plane: &Airplane, controls: &Controls, world: &World, route: &Route) -> Self {
        let (ground, _) = world.ground(plane.pos.x, plane.pos.z);
        let speed = plane.vel.norm();
        let ground_speed = vec2(plane.vel.x, plane.vel.z).norm();

        let flight_path_point = (speed > 5.0).then(|| {
            world
                .origin
                .relative(plane.pos + plane.vel / speed * 2000.0) // TODO: name this magic number
        });

        Self {
            pitch: plane.pitch() as f32,
            roll: plane.roll() as f32,
            heading: plane.heading() as f32,
            altitude: plane.pos.y as f32,
            height_above_ground: (plane.pos.y - ground as f64 - plane.gear_height()) as f32,
            airspeed: speed as f32,
            ground_speed: ground_speed as f32,
            vertical_speed: plane.vel.y as f32,
            throttle: controls.throttle,
            brakes: controls.brakes,
            flight_path_point,
            nav: NavData::gather(plane, ground_speed, route),
        }
    }
}

impl NavData {
    fn gather(plane: &Airplane, ground_speed: f64, route: &Route) -> Option<Self> {
        let upcoming = route.upcoming();
        let wp = upcoming.first()?;
        let (bearing, distance) = waypoint::bearing_distance(plane.pos, wp.pos);

        Some(Self {
            name: wp.name.clone(),
            next: upcoming.get(1).map(|w| w.name.clone()),
            bearing: bearing as f32,
            distance: distance as f32,
            time_to_go: (ground_speed > 1.0).then(|| (distance / ground_speed) as f32),
            target_altitude: wp.altitude.map(|a| a as f32),
            target_path: wp
                .altitude
                .map(|a| (a - plane.pos.y).atan2(distance) as f32),
            route: upcoming
                .iter()
                .map(|w| {
                    vec2(
                        (w.pos.x - plane.pos.x) as f32,
                        (w.pos.y - plane.pos.z) as f32,
                    )
                })
                .collect(),
        })
    }
}
