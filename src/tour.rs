use nalgebra_glm::{vec2, DVec2};

use crate::{
    airplane::Airplane,
    judge::{DebriefRow, Grade, Objective},
    units::{FT_PER_M, M_PER_NM},
};

struct Sighting {
    name: String,
    /// World (x, z) [m]
    pos: DVec2,
    /// Within this far counts as seeing it [m]
    radius: f64,
    /// Closest the plane has come, across the ground [m]
    closest: f64,
}

/// The sights on a tour, and how close the plane has come to each
pub struct Tour {
    sights: Vec<Sighting>,
}

impl Tour {
    /// Watch for `sights`, each a name, world (x, z) [m], and how close counts as seeing it [m]
    pub fn new(sights: impl IntoIterator<Item = (String, DVec2, f64)>) -> Self {
        Self {
            sights: sights
                .into_iter()
                .map(|(name, pos, radius)| Sighting {
                    name,
                    pos,
                    radius,
                    closest: f64::INFINITY,
                })
                .collect(),
        }
    }
}

impl Objective for Tour {
    /// Note how close the plane is to each sight
    fn update(&mut self, plane: &mut Airplane, _dt: f64) {
        let here = vec2(plane.pos.x, plane.pos.z);
        for s in &mut self.sights {
            s.closest = s.closest.min((s.pos - here).norm());
        }
    }

    fn rows(&self) -> Vec<DebriefRow> {
        self.sights
            .iter()
            .map(|s| {
                DebriefRow::new(
                    s.name.clone(),
                    distance(s.closest),
                    Grade::by(s.closest, s.radius, 2.0 * s.radius, 4.0 * s.radius),
                    format!("Missed {}", s.name),
                )
            })
            .collect()
    }
}

/// `m` [m] as feet when close, nautical miles when far
fn distance(m: f64) -> String {
    if m < M_PER_NM {
        format!("{:.0} ft away", m * FT_PER_M)
    } else {
        format!("{:.1} nm away", m / M_PER_NM)
    }
}
