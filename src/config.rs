use nalgebra_glm::{vec3, DMat3, DVec3};

#[derive(Clone, Copy)]
pub struct AirplaneConfig {
    /// Mass of the airplane [kg]
    pub mass: f64,
    /// Area of the wings [m^2]
    pub wing_area: f64,
    /// Wingspan [m]
    pub wingspan: f64,
    /// Chord thats a bit rude [m]
    pub mean_chord: f64,
    /// Max thrust of the engine [N]
    pub max_thrust: f64,
    /// Rotational inertia tensor [kg*m^2]
    pub inertia: DMat3,

    /// Lift coefficient slope [1/rad]
    pub lift_slope: f64,
    /// Lift coeff at 0 aoa
    pub zero_lift_coefficient: f64,
    /// Max positive lift coeff
    pub max_cl: f64,
    /// Max negative lift coeff
    pub min_cl: f64,
    /// Parasite drag
    pub drag_coefficient_base: f64,
    /// Oswald efficiency factor for induced drag
    pub oswald_efficiency: f64,
    /// Side force per rad of sideslip
    pub cy_beta: f64,

    /// Control surface limits [rad] [elevator, aileron, rudder]
    pub control_surface_limits: DVec3,

    /// Stability derivatives (standard aero sign conventions)
    pub derivs: StabilityDerivatives,
}

#[derive(Clone, Copy)]
pub struct StabilityDerivatives {
    // Lift
    pub c_lift_elevator: f64,
    // Side force
    pub c_side_rudder: f64,
    // Pitch
    pub cm_0: f64,
    pub cm_alpha: f64,
    pub cm_q: f64,
    pub cm_elevator: f64,
    // Roll
    pub cl_beta: f64,
    pub cl_p: f64,
    pub cl_aileron: f64,
    // Yaw
    pub cn_beta: f64,
    pub cn_r: f64,
    pub cn_rudder: f64,
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
            inertia: DMat3::from_diagonal(&vec3(1285.0, 2667.0, 1825.0)),

            lift_slope: 5.0,
            zero_lift_coefficient: 0.2,
            max_cl: 1.4,
            min_cl: -0.8,
            drag_coefficient_base: 0.028,
            oswald_efficiency: 0.8,
            cy_beta: -0.31,

            control_surface_limits: vec3(
                25.0_f64.to_radians(),
                18.0_f64.to_radians(),
                16.0_f64.to_radians(),
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

    pub fn piper_j3_cub() -> Self {
        Self {
            // Geometry and mass
            mass: 550.0,
            wing_area: 16.5,
            wingspan: 10.74,
            mean_chord: 1.54,
            max_thrust: 1700.0,

            // Approximate inertia tensor [kg*m^2]
            inertia: DMat3::from_diagonal(&vec3(
                650.0,  // Ixx: roll
                950.0,  // Iyy: pitch
                1300.0, // Izz: yaw
            )),

            // Aerodynamics
            lift_slope: 4.8,
            zero_lift_coefficient: 0.25,
            max_cl: 1.5,
            min_cl: -0.7,
            drag_coefficient_base: 0.045,
            oswald_efficiency: 0.75,
            cy_beta: -0.35,

            // [elevator, aileron, rudder]
            control_surface_limits: vec3(
                25.0_f64.to_radians(),
                15.0_f64.to_radians(),
                25.0_f64.to_radians(),
            ),

            derivs: StabilityDerivatives {
                // Lift
                c_lift_elevator: 0.35,

                // Side force
                c_side_rudder: 0.17,

                // Pitch
                cm_0: 0.04,
                cm_alpha: -1.5,
                cm_q: -10.0,
                cm_elevator: -1.1,

                // Roll
                cl_beta: -0.08,
                cl_p: -0.42,
                cl_aileron: 0.12,

                // Yaw
                cn_beta: 0.055,
                cn_r: -0.09,
                cn_rudder: -0.055,
            },
        }
    }

    pub fn dash_8() -> Self {
        Self {
            // Geometry and mass (approx. 20,000 kg operating weight)
            mass: 20_500.0,
            wing_area: 54.4,
            wingspan: 28.4,
            mean_chord: 2.15,
            max_thrust: 52_000.0, // Combined twin turboprops (Pratt & Whitney PW150A equivalents)

            // Rotational inertia tensor [kg*m^2] (much larger body, higher resistance to rotation)
            inertia: DMat3::from_diagonal(&vec3(
                45_000.0,  // Ixx: roll
                120_000.0, // Iyy: pitch
                150_000.0, // Izz: yaw
            )),

            // Aerodynamics (cleaner, high-speed regional transport airfoil)
            lift_slope: 5.5,
            zero_lift_coefficient: 0.18,
            max_cl: 1.6,
            min_cl: -0.7,
            drag_coefficient_base: 0.018, // Lower parasite drag
            oswald_efficiency: 0.85,      // High aspect ratio clean wing
            cy_beta: -0.4,

            // Control surface limits [rad] [elevator, aileron, rudder]
            control_surface_limits: vec3(
                20.0_f64.to_radians(), // Smaller deflection angles for high-speed control
                15.0_f64.to_radians(),
                22.0_f64.to_radians(),
            ),

            derivs: StabilityDerivatives {
                // Lift
                c_lift_elevator: 0.5,

                // Side force
                c_side_rudder: 0.22,

                // Pitch (heavier damping due to large tail volume and mass)
                cm_0: 0.03,
                cm_alpha: -2.1,
                cm_q: -18.5,
                cm_elevator: -1.45,

                // Roll
                cl_beta: -0.12,
                cl_p: -0.65,
                cl_aileron: 0.15,

                // Yaw
                cn_beta: 0.09,
                cn_r: -0.22,
                cn_rudder: -0.08,
            },
        }
    }
}
