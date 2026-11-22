use std::f64::consts::FRAC_PI_2;

use apricot::camera::floating_origin::FloatingOrigin;
use nalgebra_glm::{vec3, DMat3, DQuat, DVec3, Mat4, Vec3};

use crate::{
    config::{AirplaneConfig, ContactKind},
    control::Controls,
};

/// How much stiffer the airframe is than the tires (?)
const AIRFRAME_STIFFNESS: f64 = 4.0;
/// Tire sideways friction coeff
const TIRE_GRIP: f64 = 0.8;
/// Tire rolling friciton coeff
const ROLLING_FRICTION: f64 = 0.02;
/// Airframe scraping coeff
const SCRAPE_FRICTION: f64 = 0.5;
/// Speed to ease off friction [m/s]
const SLIP_SPEED: f64 = 0.3;
/// Where each wing half's lift acts, as a fraction of the wingspan out from the middle
/// (center of lift for an elliptical half wing)
const LIFT_ARM: f64 = 0.21;

// TODO:
// - split this off into RigidBodyState {pos, vel, orient, ang vel} and Deriv{vel, accel, ang vel, ang acc}, then define derivative() on Airplane for RK4
pub struct Airplane {
    // State
    /// World position of the plane [m]
    pub pos: DVec3,
    /// World velocity of the plane [m/s]
    pub vel: DVec3,
    /// World orientation of the plane [rad]
    pub orientation: DQuat,
    /// Body-space angular velocity of the plan [rad/s]
    body_angular_velocity: DVec3,

    config: AirplaneConfig,
    inertia: DMat3,
    inertia_inv: DMat3,
    control_limits: DVec3,
}

struct AeroState {
    /// Air speed [m/s]
    speed: f64,
    air_density: f64,
    dynamic_pressure: f64,
    angle_of_attack: f64,
    sideslip: f64,
}

struct Deflections {
    elevator: f64,
    aileron: f64,
    rudder: f64,
}

impl Deflections {
    fn from_controls(controls: &Controls, limits: DVec3) -> Self {
        Self {
            elevator: -controls.pitch as f64 * limits.x,
            aileron: controls.roll as f64 * limits.y,
            rudder: controls.yaw as f64 * limits.z,
        }
    }
}

const MAX_PHYSICS_STEP: f64 = 1.0 / 240.0;

impl Airplane {
    pub fn new(config: AirplaneConfig) -> Self {
        let inertia = config.inertia.matrix();
        let inertia_inv = nalgebra_glm::inverse(&inertia);
        let control_limits = config.control_limits.radians();

        Self {
            pos: vec3(0.0, 40.0, 0.0),
            vel: vec3(55.0, 0.0, 0.0),
            orientation: DQuat::identity(),
            body_angular_velocity: vec3(0.0, 0.0, 0.0),
            config,
            inertia,
            inertia_inv,
            control_limits,
        }
    }

    /// Put the plane at `pos` facing `heading`
    pub fn place(&mut self, pos: DVec3, heading: f64) {
        self.pos = pos;
        self.vel = DVec3::zeros();
        self.body_angular_velocity = DVec3::zeros();
        self.orientation = nalgebra_glm::quat_angle_axis(FRAC_PI_2 - heading, &DVec3::y())
    }

    pub fn update(
        &mut self,
        controls: &Controls,
        dt: f64,
        ground: &impl Fn(f64, f64) -> (f32, Vec3),
    ) {
        let steps = (dt / MAX_PHYSICS_STEP).ceil().max(1.0) as usize;
        let h = dt / steps as f64;
        for _ in 0..steps {
            self.step(controls, h, ground);
        }
    }

    fn step(&mut self, controls: &Controls, dt: f64, ground: &impl Fn(f64, f64) -> (f32, Vec3)) {
        let aero = self.calc_aero_state();
        let defl = Deflections::from_controls(controls, self.control_limits);

        let forces = self.calc_aero_forces(controls.throttle as f64, &aero, &defl);
        let torque = self.calc_aero_moments(&aero, &defl);
        let (wing_force, wing_torque) = self.calc_wing_forces(&aero, &defl);
        let (ground_force, ground_torque) = self.calc_ground_forces(controls.brakes as f64, ground);

        self.integrate_translation(forces + wing_force + ground_force, dt);
        self.integrate_rotation(torque + wing_torque + ground_torque, dt);
    }

    fn calc_aero_state(&self) -> AeroState {
        let speed = self.vel.norm();

        // TODO: Calc with temp too
        let air_density = 1.225 * (-self.pos.y / 8500.0).exp();
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
            air_density,
            speed,
            dynamic_pressure,
            angle_of_attack,
            sideslip,
        }
    }

    fn calc_aero_forces(&self, throttle: f64, aero: &AeroState, defl: &Deflections) -> DVec3 {
        let mut force = DVec3::zeros();

        // Thrust
        let forward = nalgebra_glm::quat_rotate_vec3(&self.orientation, &DVec3::new(1.0, 0.0, 0.0));
        let power = throttle * self.config.max_power * aero.air_density / 1.225;
        let thrust = (throttle * self.config.max_thrust)
            .min(self.config.prop_efficiency * power / aero.speed.max(1.0));
        force += forward * thrust;

        // Sideways push on the fuselage and fin from sideslip and rudder
        if aero.speed > 1.0 {
            let right =
                nalgebra_glm::quat_rotate_vec3(&self.orientation, &DVec3::new(0.0, 0.0, 1.0));
            let c_side = self.config.cy_beta * aero.sideslip
                + self.config.derivs.c_side_rudder * defl.rudder;
            force += right * (aero.dynamic_pressure * self.config.wing_area * c_side);
        }

        // Gravity
        force.y -= self.config.mass * 9.81;

        force
    }

    fn calc_wing_forces(&self, aero: &AeroState, defl: &Deflections) -> (DVec3, DVec3) {
        if aero.speed < 1.0 {
            return (DVec3::zeros(), DVec3::zeros());
        }

        let c = &self.config;
        let aspect_ratio = c.wingspan * c.wingspan / c.wing_area;
        let inv_orientation = nalgebra_glm::quat_inverse(&self.orientation);
        let body_vel = nalgebra_glm::quat_rotate_vec3(&inv_orientation, &self.vel);

        // control surfaces giveth lift and taketh it away
        let aileron_cl = c.derivs.cl_aileron * defl.aileron / LIFT_ARM;
        let elevator_cl = c.derivs.c_lift_elevator * defl.elevator;

        let mut force = DVec3::zeros();
        let mut torque = DVec3::zeros();
        for side in [-1.0, 1.0] {
            // left, right
            let arm = vec3(0.0, 0.0, side * LIFT_ARM * c.wingspan);

            // the air this half moves through, including the plane rotating
            let v = body_vel + self.body_angular_velocity.cross(&arm);
            let speed = v.norm();
            if speed < 1.0 {
                continue;
            }
            let flow = v / speed;
            let alpha = (-v.y).atan2(v.x);
            let q = 0.5 * aero.air_density * speed * speed;

            let c_lift = self.lift_coefficient(alpha) + elevator_cl - side * aileron_cl;

            let c_drag_attached = c.drag_coefficient_base
                + c_lift * c_lift / (std::f64::consts::PI * aspect_ratio * c.oswald_efficiency);
            let c_drag = c_drag_attached.max(2.0 * alpha.sin().powi(2));

            let up = DVec3::y();
            let lift_dir = up - flow * up.dot(&flow);
            let lift_dir = if lift_dir.norm_squared() > 1e-6 {
                lift_dir.normalize()
            } else {
                up
            };

            let f = (lift_dir * c_lift - flow * c_drag) * (q * 0.5 * c.wing_area);
            force += f;
            torque += arm.cross(&f);
        }

        (
            nalgebra_glm::quat_rotate_vec3(&self.orientation, &force),
            torque,
        )
    }

    fn lift_coefficient(&self, alpha: f64) -> f64 {
        let linear = self.config.zero_lift_coefficient + self.config.lift_slope * alpha;

        // aoa at which the linear curve hits its limit
        let alpha_stall_pos =
            (self.config.max_cl - self.config.zero_lift_coefficient) / self.config.lift_slope;
        let alpha_stall_neg =
            (self.config.min_cl - self.config.zero_lift_coefficient) / self.config.lift_slope;

        let flat_plate = (2.0 * alpha).sin(); // high aoa lift
        let blend_width = 8.0_f64.to_radians();

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

    fn calc_aero_moments(&self, aero: &AeroState, defl: &Deflections) -> DVec3 {
        if aero.speed < 1.0 {
            return DVec3::zeros();
        }

        // Convert to the standard aero axes (x fwd, y right, z down)
        let alpha = aero.angle_of_attack;
        let beta = aero.sideslip;
        let q = self.body_angular_velocity.z;
        let r = -self.body_angular_velocity.y;

        // Non-dimensional rates
        let q_hat = q * self.config.mean_chord / (2.0 * aero.speed);
        let r_hat = r * self.config.wingspan / (2.0 * aero.speed);

        // Moment coeffs
        let d = &self.config.derivs;
        let c_roll = d.cl_beta * beta;
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

    fn calc_ground_forces(
        &self,
        brake_fraction: f64,
        ground: &impl Fn(f64, f64) -> (f32, Vec3),
    ) -> (DVec3, DVec3) {
        let weight = self.config.mass * 9.81;

        // Springs that hold the plane up with wheel sag of squash, with the weight shared between the wheels
        let wheels = self
            .config
            .contacts
            .iter()
            .filter(|c| matches!(c.kind, ContactKind::Wheel { .. }))
            .count() as f64;
        let k_wheel = weight / (wheels * self.config.gear.sag);
        let c_wheel = self.config.gear.damping * 2.0 * (k_wheel * self.config.mass / wheels).sqrt();

        let omega = nalgebra_glm::quat_rotate_vec3(&self.orientation, &self.body_angular_velocity);
        let forward = nalgebra_glm::quat_rotate_vec3(&self.orientation, &vec3(1.0, 0.0, 0.0));

        let mut force = DVec3::zeros();
        let mut torque = DVec3::zeros(); // world space, about the center of grav
        for contact in &self.config.contacts {
            let r = nalgebra_glm::quat_rotate_vec3(&self.orientation, &DVec3::from(contact.pos));
            let center = self.pos + r;
            let (h, n) = ground(center.x, center.z);
            let h = h as f64;
            let n: DVec3 = n.cast();

            // How far the sphere is into the ground
            let depth = contact.radius - (center.y - h) * n.y;
            if depth <= 0.0 {
                continue; // not colliding with the ground
            }

            // Please indicate on the sphere where the ground touched you
            let arm = r - n * contact.radius;
            let v = self.vel + omega.cross(&arm);
            let v_n = v.dot(&n);
            let v_t = v - n * v_n; // sliding along the ground

            let (k, c) = match contact.kind {
                ContactKind::Wheel { .. } => {
                    let c = if v_n < 0.0 {
                        c_wheel
                    } else {
                        // struts extend slowly, stops the plane from bouncing
                        c_wheel * self.config.gear.rebound_damping
                    };
                    (k_wheel, c)
                }
                ContactKind::Airframe => (
                    k_wheel * AIRFRAME_STIFFNESS,
                    c_wheel * AIRFRAME_STIFFNESS.sqrt(),
                ),
            };

            let normal_force = (k * depth - c * v_n).clamp(0.0, 30.0 * weight);

            let roll_dir = forward - n * forward.dot(&n);
            let f_t = match contact.kind {
                ContactKind::Wheel { brakes } if roll_dir.norm_squared() > 1e-4 => {
                    // Wheels can move freely along their direction
                    let roll_dir = roll_dir.normalize();
                    // How much friction the brakes for this wheel adds
                    let brake_friction = if brakes { brake_fraction } else { 0.0 }
                        * self.config.gear.max_brake_friction;
                    // They grip sideways
                    let v_roll = roll_dir * v_t.dot(&roll_dir);

                    let roll = friction(v_roll, ROLLING_FRICTION + brake_friction, normal_force);
                    let side = friction(v_t - v_roll, TIRE_GRIP, normal_force);

                    // Budget the braking
                    let total = roll + side;
                    let max = TIRE_GRIP * normal_force;
                    if total.norm() > max {
                        total * (max / total.norm())
                    } else {
                        total
                    }
                }
                _ => friction(v_t, SCRAPE_FRICTION, normal_force),
            };

            let f = n * normal_force + f_t;
            force += f;
            torque += arm.cross(&f);
        }

        let inv_orientation = nalgebra_glm::quat_inverse(&self.orientation);
        (
            force,
            nalgebra_glm::quat_rotate_vec3(&inv_orientation, &torque),
        )
    }

    fn integrate_translation(&mut self, forces: DVec3, dt: f64) {
        let accel = forces / self.config.mass;
        self.vel += accel * dt;
        self.pos += self.vel * dt;
    }

    fn integrate_rotation(&mut self, torque: DVec3, dt: f64) {
        let h = self.inertia * self.body_angular_velocity;
        let angular_accel = self.inertia_inv * (torque - self.body_angular_velocity.cross(&h)); // hey this looks like orbit stuff!
        self.body_angular_velocity += angular_accel * dt;

        let angular_speed = self.body_angular_velocity.norm();
        if angular_speed < 1e-6 {
            return;
        }

        let axis = self.body_angular_velocity / angular_speed;
        let half_angle = angular_speed * dt * 0.5;

        let s = half_angle.sin();

        let delta = DQuat::new(half_angle.cos(), axis.x * s, axis.y * s, axis.z * s);

        self.orientation = (self.orientation * delta).normalize();
    }

    pub fn model_matrix(&self, origin: &FloatingOrigin) -> Mat4 {
        origin.transform(
            self.pos,
            nalgebra_glm::quat_to_mat4(&self.orientation).cast::<f32>(),
        )
    }

    /// Nose angle above the horizon [rad], + = nose up
    pub fn pitch(&self) -> f64 {
        let forward = nalgebra_glm::quat_rotate_vec3(&self.orientation, &vec3(1.0, 0.0, 0.0));
        forward.y.clamp(-1.0, 1.0).asin()
    }

    /// Bank angle [rad], + = right wing down. Works inverted too
    pub fn roll(&self) -> f64 {
        let up = nalgebra_glm::quat_rotate_vec3(&self.orientation, &vec3(0.0, 1.0, 0.0));
        let right = nalgebra_glm::quat_rotate_vec3(&self.orientation, &vec3(0.0, 0.0, 1.0));
        (-right.y).atan2(up.y)
    }

    /// Compass heading [rad], 0 = north (-Z), clockwise, 0..2pi
    pub fn heading(&self) -> f64 {
        let forward = nalgebra_glm::quat_rotate_vec3(&self.orientation, &vec3(1.0, 0.0, 0.0));
        forward
            .x
            .atan2(-forward.z)
            .rem_euclid(std::f64::consts::TAU)
    }

    pub fn gear_height(&self) -> f64 {
        self.config.gear_height()
    }
}

/// Friction against sliding at `v`, up to `mu * normal_force`
fn friction(v: DVec3, mu: f64, normal_force: f64) -> DVec3 {
    -v * (mu * normal_force / v.norm().max(SLIP_SPEED))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn plane_at(local_vel: DVec3) -> Airplane {
        let mut p = Airplane::new(
            AirplaneConfig::load("assets/planes/dash8.toml").expect("should be there"),
        );
        p.vel = local_vel; // identity orientation, so body == world
        p
    }

    fn moments(p: &Airplane, controls: &Controls) -> DVec3 {
        let aero = p.calc_aero_state();
        let defl = Deflections::from_controls(controls, p.control_limits);
        p.calc_aero_moments(&aero, &defl) + p.calc_wing_forces(&aero, &defl).1
    }

    fn forces(p: &Airplane, controls: &Controls) -> DVec3 {
        let aero = p.calc_aero_state();
        let defl = Deflections::from_controls(controls, p.control_limits);
        p.calc_aero_forces(controls.throttle as f64, &aero, &defl)
            + p.calc_wing_forces(&aero, &defl).0
    }

    #[test]
    fn positive_x_rotation_rolls_right() {
        let mut p = plane_at(vec3(55.0, 0.0, 0.0));
        p.body_angular_velocity = vec3(1.0, 0.0, 0.0);
        p.integrate_rotation(DVec3::zeros(), 0.1);
        let right = nalgebra_glm::quat_rotate_vec3(&p.orientation, &vec3(0.0, 0.0, 1.0));
        assert!(right.y < 0.0, "right wing should drop");
    }

    #[test]
    fn positive_z_rotation_pitches_up() {
        let mut p = plane_at(vec3(55.0, 0.0, 0.0));
        p.body_angular_velocity = vec3(0.0, 0.0, 1.0);
        p.integrate_rotation(DVec3::zeros(), 0.1);
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
        p.body_angular_velocity = DVec3::zeros();
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

    #[test]
    fn ailerons_cause_adverse_yaw() {
        // Rolling right puts more lift, so more drag, on the left half, which drags the nose left
        let p = plane_at(vec3(55.0, 0.0, 0.0));
        let base = moments(&p, &Controls::new());
        let mut c = Controls::new();
        c.roll = 1.0; // E
        assert!(
            moments(&p, &c).y > base.y,
            "rolling right yaws the nose left (+Y)"
        );
    }

    #[test]
    fn rolling_is_damped_before_the_stall() {
        let mut p = plane_at(vec3(55.0, 0.0, 0.0));
        p.body_angular_velocity = vec3(0.5, 0.0, 0.0); // rolling right
        assert!(
            moments(&p, &Controls::new()).x < 0.0,
            "the down-going wing lifts and resists"
        );
    }

    #[test]
    fn rolling_feeds_itself_past_the_stall() {
        // A few degrees past the stall, the down-going wing loses lift and keeps falling
        let alpha = (p_stall_alpha() + 4.0_f64.to_radians()).tan();
        let mut p = plane_at(vec3(55.0, -55.0 * alpha, 0.0));
        p.body_angular_velocity = vec3(0.2, 0.0, 0.0); // rolling right
        let roll = p
            .calc_wing_forces(
                &p.calc_aero_state(),
                &Deflections::from_controls(&Controls::new(), p.control_limits),
            )
            .1
            .x;
        assert!(roll > 0.0, "autorotation: the roll keeps going");
    }

    /// Where the test plane's lift curve tops out
    fn p_stall_alpha() -> f64 {
        let p = plane_at(vec3(55.0, 0.0, 0.0));
        (p.config.max_cl - p.config.zero_lift_coefficient) / p.config.lift_slope
    }
}
