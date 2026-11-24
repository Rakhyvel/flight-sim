//! Camera for flight, modelled as if it were a chase plane behind the plane
use std::f64::consts::{FRAC_PI_2, FRAC_PI_4, PI, TAU};

use apricot::{
    camera::{floating_origin::FloatingOrigin, Camera, ProjectionKind},
    input::Input,
};
use nalgebra_glm::{rotate_vec3, vec3, DQuat, DVec3};
use sdl2::{joystick::HatState, keyboard::Scancode};

use crate::{airplane::Airplane, world::RENDER_DISTANCE};

// Zoom
const WIDE_FOV: f64 = FRAC_PI_4; // [rad]
const ZOOMED_FOV: f64 = 0.21; // [rad]
const ZOOM_RATE: f64 = 8.0; // [1/s]
const ZOOM_BUTTON: usize = 2; // TODO: flick your stick
/// How far ahead of the plane the camera aims when wide [m]
const WIDE_AIM_DISTANCE: f64 = 10.0;
/// How far ahead of the plane the camera aims when zoomed in [m]
const ZOOMED_AIM_DISTANCE: f64 = 10000.0;

// Distance behind the plane
const MIN_DISTANCE: f64 = 0.8; // [m]
const MAX_DISTANCE: f64 = 400.0; // [m]
const WHEEL_STEP: f64 = 0.9;
/// Slope behind the plane to go up
const RISE_PER_DISTANCE: f64 = 0.24;

// Lookin around
const LOOK_RATE: f64 = FRAC_PI_2; // [rad/s]
const MOUSE_LOOK_PER_PX: f64 = 0.003; // [rad]
const MAX_LOOK_PITCH: f64 = 1.4; // [rad]
const LOOK_CENTER_BUTTON: usize = 1; // TODO: keybinding configs

// Chase cam
/// How quickly the chase camera matches the plane's altitude
const CATCH_UP_RATE: f64 = 4.0;

pub struct ChaseCamera {
    distance: f64, // [m]
    fov: f64,      // [rad]
    chase_orientation: DQuat,
    /// Right of straight ahead [rad]
    look_yaw: f64,
    /// Above straight ahead [rad]
    look_pitch: f64,
    time: f64, // [s]
}

impl ChaseCamera {
    pub fn new() -> Self {
        Self {
            distance: 25.0,
            fov: WIDE_FOV,
            chase_orientation: nalgebra_glm::quat_identity(),
            look_yaw: 0.0,
            look_pitch: 0.0,
            time: 0.0,
        }
    }

    pub fn reset(&mut self, plane: &Airplane) {
        self.chase_orientation = plane.orientation;
        self.look_yaw = 0.0;
        self.look_pitch = 0.0
    }

    pub fn update(
        &mut self,
        input: &Input,
        mouse_look: bool,
        plane: &Airplane,
        origin: &FloatingOrigin,
        dt: f64,
    ) -> Camera {
        self.time += dt;
        self.steer_look(input, mouse_look, dt);
        let zoomed = self.zoom(input, dt);

        // Smoothly move the chase cam's orientation to the plane's
        let mut toward = plane.orientation;
        if self.chase_orientation.dot(&toward) <= 0.0 {
            toward = -toward; // take the short way
        }
        self.chase_orientation = self
            .chase_orientation
            .lerp(&toward, (CATCH_UP_RATE * dt).min(1.0))
            .normalize();
        let nose = nalgebra_glm::quat_rotate_vec3(&self.chase_orientation, &vec3(1.0, 0.0, 0.0));
        let up = nalgebra_glm::quat_rotate_vec3(&self.chase_orientation, &vec3(0.0, 1.0, 0.0));

        // Lean over a bit depending on the plane's velocity
        let speed = plane.vel.norm();
        let travel = if speed > 0.1 { plane.vel / speed } else { nose };
        let chase_dir = (nose * 0.8 + travel * 0.2).normalize();

        // Turn to where the pilot is looking
        let (view_dir, view_up) = self.turn_by_look(chase_dir, up);
        let (aim_dir, _) = self.turn_by_look(nose, up);

        // Calculate the camera pos and lookat depending on the zoom
        let plane_pos = plane.pos - origin.pos;
        let cam_pos =
            plane_pos - view_dir * self.distance + view_up * self.distance * RISE_PER_DISTANCE;
        let target = plane_pos
            + view_dir * WIDE_AIM_DISTANCE * (1.0 - zoomed)
            + aim_dir * ZOOMED_AIM_DISTANCE * zoomed;

        Camera::new(
            cam_pos.cast(),
            target.cast(),
            view_up.cast(),
            ProjectionKind::Perspective {
                fov_rad: self.fov as f32,
                near: 3.0,
                far: RENDER_DISTANCE as f32,
            },
        )
    }

    fn steer_look(&mut self, input: &Input, mouse_look: bool, dt: f64) {
        let stick = input.joystick();

        // Adjust yaw and pitch based on joystick and arrow keys
        let (hat_right, hat_up) = hat_direction(stick.map_or(HatState::Centered, |js| js.hat(0)));
        let right = (hat_right + key_axis(input, Scancode::Left, Scancode::Right)).clamp(-1.0, 1.0);
        let up = (hat_up + key_axis(input, Scancode::Down, Scancode::Up)).clamp(-1.0, 1.0);
        self.look_yaw += right * LOOK_RATE * dt;
        self.look_pitch += up * LOOK_RATE * dt;

        // Do the same with the mouse, if enabled
        if mouse_look {
            self.look_yaw += input.mouse_vel.x as f64 * MOUSE_LOOK_PER_PX;
            self.look_pitch -= input.mouse_vel.y as f64 * MOUSE_LOOK_PER_PX;
        }

        // Center camera
        let center_pressed = stick.is_some_and(|js| js.button_pressed(LOOK_CENTER_BUTTON));
        if input.key_pressed(Scancode::C) || center_pressed {
            self.look_yaw = 0.0;
            self.look_pitch = 0.0;
        }

        // Wrap the angles around
        self.look_yaw = (self.look_yaw + PI).rem_euclid(TAU) - PI;
        self.look_pitch = self.look_pitch.clamp(-MAX_LOOK_PITCH, MAX_LOOK_PITCH);
    }

    /// Adjust the plane distance and FOV zoom
    fn zoom(&mut self, input: &Input, dt: f64) -> f64 {
        self.distance = (self.distance * WHEEL_STEP.powf(input.mouse_wheel as f64))
            .clamp(MIN_DISTANCE, MAX_DISTANCE);

        let zoom_held = input.key_down(Scancode::Z)
            || input
                .joystick()
                .is_some_and(|js| js.button_down(ZOOM_BUTTON));
        let target_fov = if zoom_held { ZOOMED_FOV } else { WIDE_FOV };
        self.fov += (target_fov - self.fov) * (ZOOM_RATE * dt).min(1.0);

        (WIDE_FOV - self.fov) / (WIDE_FOV - ZOOMED_FOV)
    }

    /// Rotate (dir, up) by the pitch and yaw
    fn turn_by_look(&self, dir: DVec3, up: DVec3) -> (DVec3, DVec3) {
        let dir = rotate_vec3(&dir, -self.look_yaw, &up);
        let side = dir.cross(&up).normalize();
        (
            rotate_vec3(&dir, self.look_pitch, &side),
            rotate_vec3(&up, self.look_pitch, &side),
        )
    }
}

/// How far (right, up) the hat is pushed
fn hat_direction(hat: HatState) -> (f64, f64) {
    match hat {
        HatState::Centered => (0.0, 0.0),
        HatState::Up => (0.0, 1.0),
        HatState::Down => (0.0, -1.0),
        HatState::Left => (-1.0, 0.0),
        HatState::Right => (1.0, 0.0),
        HatState::LeftUp => (-1.0, 1.0),
        HatState::LeftDown => (-1.0, -1.0),
        HatState::RightUp => (1.0, 1.0),
        HatState::RightDown => (1.0, -1.0),
    }
}

fn key_axis(input: &Input, negative: Scancode, positive: Scancode) -> f64 {
    input.key_down(positive) as i32 as f64 - input.key_down(negative) as i32 as f64
}

fn wander(time: f64, seed: f64) -> f64 {
    const FREQUENCIES: [f64; 3] = [0.23, 0.61, 1.37]; // [Hz] spread so they never line up into a visible loop
    FREQUENCIES
        .iter()
        .map(|f| (TAU * f * time + seed * f * 7.0).sin())
        .sum::<f64>()
        / FREQUENCIES.len() as f64
}
