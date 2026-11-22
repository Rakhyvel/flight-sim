use nalgebra_glm::{vec3, DMat3, DVec3};
use serde::Deserialize;

#[derive(Clone, Deserialize)]
pub struct AirplaneConfig {
    pub name: String,
    /// Mass of the airplane [kg]
    pub mass: f64,
    /// Area of the wings [m^2]
    pub wing_area: f64,
    /// Wingspan [m]
    pub wingspan: f64,
    /// Chord thats a bit rude [m]
    pub mean_chord: f64,
    /// Rotational inertia tensor
    pub inertia: Inertia,

    /// Max thrust of the engine [N]
    pub max_thrust: f64,
    pub max_power: f64,
    pub prop_efficiency: f64,

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

    pub control_limits: ControlLimits,
    /// Stability derivatives (standard aero sign conventions)
    pub derivs: StabilityDerivatives,
    pub gear: GearConfig,
    pub contacts: Vec<ContactPoint>,
    pub flaps: Option<FlapConfig>,
}

/// Rotational inertia about each axis [kg*m^2]
#[derive(Clone, Copy, Deserialize)]
pub struct Inertia {
    pub roll: f64,
    pub yaw: f64,
    pub pitch: f64,
}

impl Inertia {
    /// In body axes, x roll, y yaw, z pitch
    pub fn matrix(&self) -> DMat3 {
        DMat3::from_diagonal(&vec3(self.roll, self.yaw, self.pitch))
    }
}

#[derive(Clone, Copy, Deserialize)]
pub struct ControlLimits {
    pub elevator: f64,
    pub aileron: f64,
    pub rudder: f64,
}

impl ControlLimits {
    pub fn radians(&self) -> DVec3 {
        vec3(self.elevator, self.aileron, self.rudder).map(f64::to_radians)
    }
}

#[derive(Clone, Copy, Deserialize)]
pub struct GearConfig {
    /// How far the gear squishes when sitting still [m]
    pub sag: f64,
    /// Fraction of critical damping when compressing
    pub damping: f64,
    /// How many times more damping there is while extending
    pub rebound_damping: f64,
    /// Friction added to rolling resistance by full brakes
    pub max_brake_friction: f64,
}

#[derive(Clone, Deserialize)]
pub struct FlapConfig {
    /// Flap settings [deg], the last one is full
    pub detents: Vec<f64>,
    /// How much of the full-flap lift each detent gives, 0 to 1, one per detent. Leave it out and
    /// the lift goes up evenly with angle
    #[serde(default)]
    pub lift_fraction: Option<Vec<f64>>,
    /// Extra lift at full flaps
    pub delta_cl: f64,
    /// Extra max lift at full flaps
    pub delta_max_cl: f64,
    /// Extra drag at full flaps
    pub delta_cd: f64,
    /// Pitching moment at full flaps (- = nose down)
    pub delta_cm: f64,
    /// Seconds for the flaps to go from up to fully down
    pub travel_time: f64,
}

#[derive(Clone, Copy, Deserialize)]
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
    pub cl_aileron: f64,
    // Yaw
    pub cn_beta: f64,
    pub cn_r: f64,
    pub cn_rudder: f64,
}

/// Something that can collide with the ground
#[derive(Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(tag = "kind", rename_all = "lowercase")]
pub enum ContactKind {
    /// Smoothly roll across the ground
    Wheel {
        /// Whether this wheel has brakes
        #[serde(default)]
        brakes: bool,
    },
    /// Agressively scrape against the ground
    Airframe,
}

/// A sphere on the airplane that collides with the ground, in body space
#[derive(Clone, Copy, Deserialize)]
pub struct ContactPoint {
    /// Center of the sphere [m]
    pub pos: [f64; 3],
    /// Radius of the sphere
    #[serde(default = "default_contact_radius")]
    pub radius: f64,
    #[serde(flatten)]
    pub kind: ContactKind,
}

fn default_contact_radius() -> f64 {
    0.1
}

impl AirplaneConfig {
    pub fn load(path: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let config: Self = toml::from_str(&std::fs::read_to_string(path)?)?;
        if let Some(f) = &config.flaps {
            if f.lift_fraction
                .as_ref()
                .is_some_and(|l| l.len() != f.detents.len())
            {
                return Err(
                    format!("{path}: flaps.lift_fraction needs one value per detent").into(),
                );
            }
        }
        Ok(toml::from_str(&std::fs::read_to_string(path)?)?)
    }

    /// How far the bottom of the lowest wheel is below the center of gravity [m]
    /// In body space (obviously)
    pub fn gear_height(&self) -> f64 {
        self.contacts
            .iter()
            .map(|c| c.radius - c.pos[1])
            .fold(0.0, f64::max)
    }
}
