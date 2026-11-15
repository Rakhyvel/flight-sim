use nalgebra_glm::{vec3, Mat3, Mat4, Quat, Vec3};

use crate::{config::AirplaneConfig, control::Controls};

// TODO:
// - split this off into RigidBodyState {pos, vel, orient, ang vel} and Deriv{vel, accel, ang vel, ang acc}, then define derivative() on Airplane for RK4
pub struct Airplane {
    // State
    /// World position of the plane [m]
    pub pos: Vec3,
    /// World velocity of the plane [m/s]
    pub vel: Vec3,
    /// World orientation of the plane [rad]
    pub orientation: Quat,
    /// Body-space angular velocity of the plan [rad/s]
    body_angular_velocity: Vec3,

    config: AirplaneConfig,
    inertia_inv: Mat3,
}

struct AeroState {
    speed: f32,
    dynamic_pressure: f32,
    angle_of_attack: f32,
    sideslip: f32,
}

struct Deflections {
    elevator: f32,
    aileron: f32,
    rudder: f32,
}

impl Deflections {
    fn from_controls(controls: &Controls, limits: Vec3) -> Self {
        Self {
            elevator: -controls.pitch * limits.x,
            aileron: controls.roll * limits.y,
            rudder: controls.yaw * limits.z,
        }
    }
}

const MAX_PHYSICS_STEP: f32 = 1.0 / 240.0;

impl Airplane {
    pub fn new(config: AirplaneConfig) -> Self {
        let inertia_inv = nalgebra_glm::inverse(&config.inertia);

        Self {
            pos: vec3(0.0, 40.0, 0.0),
            vel: vec3(55.0, 0.0, 0.0),
            orientation: Quat::identity(),
            body_angular_velocity: vec3(0.0, 0.0, 0.0),
            inertia_inv,
            config,
        }
    }

    pub fn update(&mut self, controls: &Controls, dt: f32) {
        let steps = (dt / MAX_PHYSICS_STEP).ceil().max(1.0) as usize;
        let h = dt / steps as f32;
        for _ in 0..steps {
            self.step(controls, h);
        }
    }

    fn step(&mut self, controls: &Controls, dt: f32) {
        let aero = self.calc_aero_state();
        let defl = Deflections::from_controls(controls, self.config.control_surface_limits);

        let forces = self.calc_aero_forces(controls.throttle, &aero, &defl);
        let torque = self.calc_aero_moments(&aero, &defl);

        self.integrate_translation(forces, dt);
        self.integrate_rotation(torque, dt);
    }

    fn calc_aero_state(&self) -> AeroState {
        let speed = self.vel.norm();

        // TODO: Calc with alt and temp
        let air_density = 1.225; // kg/m^3 at sea level
        let dynamic_pressure = 0.5 * air_density * speed * speed;

        // Transform velocity into local body-space
        let inv_orientation = nalgebra_glm::quat_inverse(&self.orientation);
        let local_velocity = nalgebra_glm::quat_rotate_vec3(&inv_orientation, &self.vel);

        let angle_of_attack = (-local_velocity.y).atan2(local_velocity.x);
        let sideslip = if speed > 1.0 {
            (local_velocity.z).atan2(local_velocity.x)
        } else {
            0.0
        };

        AeroState {
            speed,
            dynamic_pressure,
            angle_of_attack,
            sideslip,
        }
    }

    fn calc_aero_forces(&self, throttle: f32, aero: &AeroState, defl: &Deflections) -> Vec3 {
        let forward = nalgebra_glm::quat_rotate_vec3(&self.orientation, &Vec3::new(1.0, 0.0, 0.0));
        let up = nalgebra_glm::quat_rotate_vec3(&self.orientation, &Vec3::new(0.0, 1.0, 0.0));
        let right = nalgebra_glm::quat_rotate_vec3(&self.orientation, &Vec3::new(0.0, 0.0, 1.0));

        let mut force = Vec3::zeros();

        // Thrust
        // TODO: Make this a better model
        force += forward * (throttle * self.config.max_thrust);

        // Lift and drag
        if aero.speed > 1.0 {
            let c_lift = self.lift_coefficient(aero.angle_of_attack)
                + self.config.derivs.c_lift_elevator * defl.elevator;
            let flow_dir = self.vel.normalize();

            // Lift, perpendicular to the flow
            let lift_magnitude = aero.dynamic_pressure * self.config.wing_area * c_lift;
            let lift_dir = up - flow_dir * up.dot(&flow_dir);
            let lift_dir = if lift_dir.norm_squared() > 1e-6 {
                lift_dir.normalize()
            } else {
                up
            };
            force += lift_dir * lift_magnitude;

            // Side force in standard aero frame
            let c_side = self.config.cy_beta * aero.sideslip
                + self.config.derivs.c_side_rudder * defl.rudder;
            force += right * (aero.dynamic_pressure * self.config.wing_area * c_side);

            // Parasite + induced drag
            let aspect_ratio = self.config.wingspan * self.config.wingspan / self.config.wing_area;
            let c_drag_attached = self.config.drag_coefficient_base
                + (c_lift * c_lift)
                    / (std::f32::consts::PI * aspect_ratio * self.config.oswald_efficiency);
            let c_drag_flat_plate = 2.0 * aero.angle_of_attack.sin().powi(2);
            let c_drag = c_drag_attached.max(c_drag_flat_plate);
            let drag_magnitude = aero.dynamic_pressure * self.config.wing_area * c_drag;
            force += -flow_dir * drag_magnitude;
        }

        // Gravity
        force.y -= self.config.mass * 9.81;

        force
    }

    fn lift_coefficient(&self, alpha: f32) -> f32 {
        let linear = self.config.zero_lift_coefficient + self.config.lift_slope * alpha;

        // aoa at which the linear curve hits its limit
        let alpha_stall_pos =
            (self.config.max_cl - self.config.zero_lift_coefficient) / self.config.lift_slope;
        let alpha_stall_neg =
            (self.config.min_cl - self.config.zero_lift_coefficient) / self.config.lift_slope;

        let flat_plate = (2.0 * alpha).sin(); // high aoa lift
        let blend_width = 8.0_f32.to_radians();

        if alpha > alpha_stall_pos {
            let t = ((alpha - alpha_stall_pos) / blend_width).min(1.0);
            let t = t * t * (3.0 - 2.0 * t); // smoothstep
            self.config.max_cl + (flat_plate - self.config.max_cl) * t
        } else if alpha < alpha_stall_neg {
            let t = ((alpha_stall_neg - alpha) / blend_width).min(1.0);
            let t = t * t * (3.0 - 2.0 * t); // smoothstep
            self.config.min_cl + (flat_plate - self.config.min_cl) * t
        } else {
            linear
        }
    }

    fn calc_aero_moments(&self, aero: &AeroState, defl: &Deflections) -> Vec3 {
        if aero.speed < 1.0 {
            return Vec3::zeros();
        }

        // Convert to the standard aero axes (x fwd, y right, z down)
        let alpha = aero.angle_of_attack;
        let beta = aero.sideslip;
        let p = self.body_angular_velocity.x;
        let q = self.body_angular_velocity.z;
        let r = -self.body_angular_velocity.y;

        // Non-dimensional rates
        let p_hat = p * self.config.wingspan / (2.0 * aero.speed);
        let q_hat = q * self.config.mean_chord / (2.0 * aero.speed);
        let r_hat = r * self.config.wingspan / (2.0 * aero.speed);

        // Moment coeffs
        let d = &self.config.derivs;
        let c_roll = d.cl_beta * beta + d.cl_p * p_hat + d.cl_aileron * defl.aileron;
        let c_pitch = d.cm_0 + d.cm_alpha * alpha + d.cm_q * q_hat + d.cm_elevator * defl.elevator;
        let c_yaw = d.cn_beta * beta + d.cn_r * r_hat + d.cn_rudder * defl.rudder;

        // Dimensional moments
        let q_s = aero.dynamic_pressure * self.config.wing_area;
        let roll_moment = q_s * self.config.wingspan * c_roll; // L
        let pitch_moment = q_s * self.config.mean_chord * c_pitch; // M
        let yaw_moment = q_s * self.config.wingspan * c_yaw; // N

        // back to apricot body axes (roll, yaw, pitch)
        vec3(roll_moment, -yaw_moment, pitch_moment)
    }

    fn integrate_translation(&mut self, forces: Vec3, dt: f32) {
        let accel = forces / self.config.mass;
        self.vel += accel * dt;
        self.pos += self.vel * dt;
    }

    fn integrate_rotation(&mut self, torque: Vec3, dt: f32) {
        let h = self.config.inertia * self.body_angular_velocity;
        let angular_accel = self.inertia_inv * (torque - self.body_angular_velocity.cross(&h)); // hey this looks like orbit stuff!
        self.body_angular_velocity += angular_accel * dt;

        let angular_speed = self.body_angular_velocity.norm();
        if angular_speed < 1e-6 {
            return;
        }

        let axis = self.body_angular_velocity / angular_speed;
        let half_angle = angular_speed * dt * 0.5;

        let s = half_angle.sin();

        let delta = Quat::new(half_angle.cos(), axis.x * s, axis.y * s, axis.z * s);

        self.orientation = (self.orientation * delta).normalize();
    }

    pub fn model_matrix(&self) -> Mat4 {
        let translation = nalgebra_glm::translation(&self.pos);
        let rotation = nalgebra_glm::quat_to_mat4(&self.orientation);

        translation * rotation
    }

    /// Nose angle above the horizon [rad], + = nose up
    pub fn pitch(&self) -> f32 {
        let forward = nalgebra_glm::quat_rotate_vec3(&self.orientation, &vec3(1.0, 0.0, 0.0));
        forward.y.clamp(-1.0, 1.0).asin()
    }

    /// Bank angle [rad], + = right wing down. Works inverted too
    pub fn roll(&self) -> f32 {
        let up = nalgebra_glm::quat_rotate_vec3(&self.orientation, &vec3(0.0, 1.0, 0.0));
        let right = nalgebra_glm::quat_rotate_vec3(&self.orientation, &vec3(0.0, 0.0, 1.0));
        (-right.y).atan2(up.y)
    }

    /// Compass heading [rad], 0 = north (-Z), clockwise, 0..2pi
    pub fn heading(&self) -> f32 {
        let forward = nalgebra_glm::quat_rotate_vec3(&self.orientation, &vec3(1.0, 0.0, 0.0));
        forward
            .x
            .atan2(-forward.z)
            .rem_euclid(std::f32::consts::TAU)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn plane_at(local_vel: Vec3) -> Airplane {
        let mut p = Airplane::new(AirplaneConfig::cessna172());
        p.vel = local_vel; // identity orientation, so body == world
        p
    }

    fn moments(p: &Airplane, controls: &Controls) -> Vec3 {
        let aero = p.calc_aero_state();
        let defl = Deflections::from_controls(controls, p.config.control_surface_limits);
        p.calc_aero_moments(&aero, &defl)
    }

    fn forces(p: &Airplane, controls: &Controls) -> Vec3 {
        let aero = p.calc_aero_state();
        let defl = Deflections::from_controls(controls, p.config.control_surface_limits);
        p.calc_aero_forces(controls.throttle, &aero, &defl)
    }

    #[test]
    fn positive_x_rotation_rolls_right() {
        let mut p = plane_at(vec3(55.0, 0.0, 0.0));
        p.body_angular_velocity = vec3(1.0, 0.0, 0.0);
        p.integrate_rotation(Vec3::zeros(), 0.1);
        let right = nalgebra_glm::quat_rotate_vec3(&p.orientation, &vec3(0.0, 0.0, 1.0));
        assert!(right.y < 0.0, "right wing should drop");
    }

    #[test]
    fn positive_z_rotation_pitches_up() {
        let mut p = plane_at(vec3(55.0, 0.0, 0.0));
        p.body_angular_velocity = vec3(0.0, 0.0, 1.0);
        p.integrate_rotation(Vec3::zeros(), 0.1);
        let fwd = nalgebra_glm::quat_rotate_vec3(&p.orientation, &vec3(1.0, 0.0, 0.0));
        assert!(fwd.y > 0.0, "nose should rise");
    }

    #[test]
    fn sideslip_from_right_yaws_nose_right_and_rolls_left() {
        // Moving partly to the right means air comes from the right so beta > 0
        let p = plane_at(vec3(55.0, 0.0, 5.0));
        assert!(p.calc_aero_state().sideslip > 0.0);
        let m = moments(&p, &Controls::new());
        assert!(
            m.y < 0.0,
            "weathercock: nose into wind (right = -Y rotation)"
        );
        assert!(
            m.x < 0.0,
            "dihedral: upwind (right) wing rises => roll left"
        );
    }

    #[test]
    fn rotation_rates_are_damped() {
        let mut p = plane_at(vec3(55.0, 0.0, 0.0));
        p.body_angular_velocity = vec3(1.0, 1.0, 1.0);
        let m = moments(&p, &Controls::new());
        assert!(m.x < 0.0 && m.y < 0.0, "roll/yaw rates should be opposed");
        // pitch includes cm_0, so compare against zero-rate case
        p.body_angular_velocity = Vec3::zeros();
        let m0 = moments(&p, &Controls::new());
        assert!(m.z < m0.z, "pitch rate should be opposed");
    }

    #[test]
    fn higher_aoa_pitches_nose_down() {
        // alpha is about +5 deg, air coming from below
        let p = plane_at(vec3(55.0, -4.8, 0.0));
        assert!(p.calc_aero_state().angle_of_attack > 0.0);
        let m = moments(&p, &Controls::new());
        assert!(m.z < 0.0);
    }

    #[test]
    fn controls_move_the_right_way() {
        let p = plane_at(vec3(55.0, 0.0, 0.0));
        let base = moments(&p, &Controls::new());

        let mut c = Controls::new();
        c.roll = 1.0; // E
        assert!(moments(&p, &c).x > base.x, "E rolls right");

        let mut c = Controls::new();
        c.pitch = 1.0; // S
        assert!(moments(&p, &c).z > base.z, "S pitches up");

        let mut c = Controls::new();
        c.yaw = 1.0; // A
        assert!(moments(&p, &c).y > base.y, "A yaws left (+Y)");
    }

    #[test]
    fn elevator_down_adds_lift() {
        let p = plane_at(vec3(55.0, 0.0, 0.0));
        let base = forces(&p, &Controls::new());

        let mut c = Controls::new();
        c.pitch = -1.0; // stick forward, elevator trailing edge down
        assert!(forces(&p, &c).y > base.y);
    }

    #[test]
    fn left_rudder_pushes_tail_right() {
        let p = plane_at(vec3(55.0, 0.0, 0.0));
        let base = forces(&p, &Controls::new());

        let mut c = Controls::new();
        c.yaw = 1.0; // rudder trailing edge left
        assert!(forces(&p, &c).z > base.z, "side force to the right (+Z)");
    }
}
