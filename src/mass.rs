use nalgebra_glm::{DMat3, DVec3};

use crate::config::AirplaneConfig;

pub struct PlaneMassProperties {
    /// The total mass of the plane, fuel, and cargo
    pub kg: f64,
    pub cg: DVec3,      // [m, body space]
    pub inertia: DMat3, // about `cg` [kg*m^2, body space]
    pub inertia_inv: DMat3,
}

impl PlaneMassProperties {
    /// Calculate the mass properties given a plane config, and the fuel tanks' and stations' masses
    pub fn new(
        config: &AirplaneConfig,
        fuel_tank_masses: &[f64],
        station_load_masses: &[f64],
    ) -> Self {
        // Get where the additional mass is distributed
        let empty = (DVec3::from(config.empty_cg), config.empty_mass);
        let tanks = config
            .tanks
            .iter()
            .zip(fuel_tank_masses)
            .map(|(t, &kg)| (DVec3::from(t.pos), kg));
        let stations = config
            .stations
            .iter()
            .zip(station_load_masses)
            .map(|(s, &kg)| (DVec3::from(s.pos), kg));
        let lumps: Vec<(DVec3, f64)> = std::iter::once(empty)
            .chain(tanks)
            .chain(stations)
            .collect();

        // Find the wet center of mass with the fuel and stations
        let kg: f64 = lumps.iter().map(|(_, m)| m).sum();
        let cg = lumps
            .iter()
            .fold(DVec3::zeros(), |sum, (p, m)| sum + p * *m)
            / kg;

        // Sum up the inertia added by each lump
        let mut inertia = config.empty_inertia.matrix();
        for (p, m) in &lumps {
            let r = p - cg;
            inertia += (DMat3::identity() * r.norm_squared() - r * r.transpose()) * *m;
        }

        Self {
            kg,
            cg,
            inertia,
            inertia_inv: nalgebra_glm::inverse(&inertia),
        }
    }
}
