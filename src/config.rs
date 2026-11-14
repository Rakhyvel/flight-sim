use nalgebra_glm::{vec3, Mat3, Vec3};

#[derive(Clone, Copy)]
pub struct AirplaneConfig {
    /// Mass of the airplane [kg]
    pub mass: f32,
    /// Area of the wings [m^2]
    pub wing_area: f32,
    /// Wingspan [m]
    pub wingspan: f32,
    /// Chord thats a bit rude [m]
    pub mean_chord: f32,
    /// Max thrust of the engine [N]
    pub max_thrust: f32,
    /// Rotational inertia tensor [kg*m^2]
    pub inertia: Mat3,

    /// Lift coefficient slope [1/rad]
    pub lift_slope: f32,
    /// Lift coeff at 0 aoa
    pub zero_lift_coefficient: f32,
    /// Max positive lift coeff
    pub max_cl: f32,
    /// Max negative lift coeff
    pub min_cl: f32,
    /// Parasite drag
    pub drag_coefficient_base: f32,
    /// Oswald efficiency factor for induced drag
    pub oswald_efficiency: f32,
    /// Side force per rad of sideslip
    pub cy_beta: f32,

    /// Control surface limits [rad] [elevator, aileron, rudder]
    pub control_surface_limits: Vec3,

    /// Stability derivatives (standard aero sign conventions)
    pub derivs: StabilityDerivatives,
}

#[derive(Clone, Copy)]
pub struct StabilityDerivatives {
    // Lift
    pub c_lift_elevator: f32,
    // Side force
    pub c_side_rudder: f32,
    // Pitch
    pub cm_0: f32,
    pub cm_alpha: f32,
    pub cm_q: f32,
    pub cm_elevator: f32,
    // Roll
    pub cl_beta: f32,
    pub cl_p: f32,
    pub cl_aileron: f32,
    // Yaw
    pub cn_beta: f32,
    pub cn_r: f32,
    pub cn_rudder: f32,
}

impl AirplaneConfig {
    // TODO: Load these from tomls (or karta!)
    pub fn cessna172() -> Self {
        Self {
            mass: 1111.0,
            wing_area: 16.2,
            wingspan: 11.0,
            mean_chord: 1.49,
            max_thrust: 2600.0,
            inertia: Mat3::from_diagonal(&vec3(1285.0, 2667.0, 1825.0)),

            lift_slope: 5.0,
            zero_lift_coefficient: 0.2,
            max_cl: 1.4,
            min_cl: -0.8,
            drag_coefficient_base: 0.028,
            oswald_efficiency: 0.8,
            cy_beta: -0.31,

            control_surface_limits: vec3(
                25.0_f32.to_radians(),
                18.0_f32.to_radians(),
                16.0_f32.to_radians(),
            ),

            derivs: StabilityDerivatives {
                c_lift_elevator: 0.43,
                c_side_rudder: 0.19,
                cm_0: 0.05,
                cm_alpha: -1.8,
                cm_q: -12.4,
                cm_elevator: -1.28,
                cl_beta: -0.09,
                cl_p: -0.47,
                cl_aileron: 0.18,
                cn_beta: 0.065,
                cn_r: -0.1,
                cn_rudder: -0.065,
            },
        }
    }
}
