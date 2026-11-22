//! A lookup texture of how much air lies between any point and space, so shaders don't have to
//! march toward the sun

// Keep these in sync with atmosphere.glsl
const PLANET_R: f64 = 6371e3;
const ATMOS_R: f64 = 6471e3;
const H_R: f64 = 8000.0;
const H_M: f64 = 1200.0;
/// The most optical depth the texture can hold [m], anything past it lets no light through anyway
const OD_MAX: f64 = 1.0e6;

/// The cosine of the angle from straight up, -1 to 1
pub const WIDTH: u32 = 256;
/// The altitude from sea level to space, squeezed toward the ground
pub const HEIGHT: u32 = 64;
const STEPS: usize = 500;

/// RGBA pixels, `WIDTH` x `HEIGHT`. Red and green are the Rayleigh depth, blue and alpha the Mie
pub fn pixels() -> Vec<u8> {
    let mut out = Vec::with_capacity((WIDTH * HEIGHT * 4) as usize);
    for y in 0..HEIGHT {
        let v = y as f64 / (HEIGHT - 1) as f64;
        let alt = v * v * (ATMOS_R - PLANET_R); // more rows near the ground, where it changes fastest
        for x in 0..WIDTH {
            let mu = x as f64 / (WIDTH - 1) as f64 * 2.0 - 1.0;
            let (r, m) = optical_depth(alt, mu);
            let ([r_hi, r_lo], [m_hi, m_lo]) = (encode(r), encode(m));
            out.extend_from_slice(&[r_hi, r_lo, m_hi, m_lo]);
        }
    }
    out
}

/// Air between altitude `alt` and space, looking `mu` (cosine from straight up), as (Rayleigh, Mie) [m of sea level air]
fn optical_depth(alt: f64, mu: f64) -> (f64, f64) {
    // Distance to the top of the atmosphere
    let r = PLANET_R + alt;
    let b = r * mu;
    let len = -b + (b * b - (r * r - ATMOS_R * ATMOS_R)).sqrt();

    // March along the ray
    let ds = len / STEPS as f64;
    let (mut od_r, mut od_m) = (0.0, 0.0);
    for i in 0..STEPS {
        let t = (i as f64 + 0.5) * ds;
        let h = (r * r + t * t + 2.0 * r * t * mu).sqrt() - PLANET_R; // law of cosines, so that the ground curves away
        od_r += (-h / H_R).exp() * ds;
        od_m += (-h / H_M).exp() * ds;
    }
    (od_r, od_m)
}

/// `od` as 16 bits, high byte then low
fn encode(od: f64) -> [u8; 2] {
    (((od / OD_MAX).clamp(0.0, 1.0) * 65535.0).round() as u16).to_be_bytes()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn straight_up_from_sea_level_is_one_scale_height_of_air() {
        let (r, m) = optical_depth(0.0, 1.0);
        assert!((r - H_R).abs() < 0.01 * H_R, "{r}");
        assert!((m - H_M).abs() < 0.01 * H_M, "{m}");
    }

    #[test]
    fn the_horizon_has_far_more_air_than_straight_up() {
        assert!(optical_depth(0.0, 0.0).0 > 20.0 * optical_depth(0.0, 1.0).0);
    }
}
