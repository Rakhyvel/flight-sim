//! Synthesized Lycoming O-320 with flat-four exhaust pulses, a 2-blade prop, and wind

use apricot::audio::Synth;
use std::f32::consts::TAU;

/// Firing order 1-3-2-4, as (crank position in revolutions, exhaust loudness).
/// Odd cylinders are on the right bank, even on the left. The two banks sound
/// slightly different, which gives the half-order lope
const CYLINDERS: [(f32, f32); 4] = [
    (0.0, 1.0), // 1, right
    (0.5, 1.0), // 3, right
    (1.0, 0.8), // 2, left
    (1.5, 0.8), // 4, left
];

pub struct EngineSound {
    // Set these from the game
    pub rpm: f32,
    pub throttle: f32, // 0..1
    pub airspeed: f32, // m/s

    // Smoothed copies, so 60 Hz updates don't make the audio step
    rpm_s: f32,
    throttle_s: f32,
    airspeed_s: f32,

    /// Position in the 4-stroke cycle, 0..2 revolutions
    crank: f32,
    /// Each cylinder's exhaust pulse envelope
    pulses: [f32; 4],
    /// Muffler for a resonant low-pass filter (Chamberlin state-variable)
    low: f32,
    band: f32,

    prop_phase: f32,
    wind: f32,
    rng: u32,
}

impl EngineSound {
    pub fn new() -> Self {
        Self {
            rpm: 700.0,
            throttle: 0.0,
            airspeed: 0.0,
            rpm_s: 700.0,
            throttle_s: 0.0,
            airspeed_s: 0.0,
            crank: 0.0,
            pulses: [0.0; 4],
            low: 0.0,
            band: 0.0,
            prop_phase: 0.0,
            wind: 0.0,
            rng: 0x1234_5678,
        }
    }

    /// White noise
    /// TODO: Move this to apricot?
    fn noise(&mut self) -> f32 {
        self.rng ^= self.rng << 13;
        self.rng ^= self.rng >> 17;
        self.rng ^= self.rng << 5;
        self.rng as f32 / u32::MAX as f32 * 2.0 - 1.0
    }
}

impl Synth for EngineSound {
    fn next_sample(&mut self, dt: f32) -> f32 {
        // Smoothly glide to whatever the rpm is
        // TODO: This should probably come from the actual engine models, not something done at the front end
        let k = (dt * 20.0).min(1.0);
        self.rpm_s += (self.rpm - self.rpm_s) * k;
        self.throttle_s += (self.throttle - self.throttle_s) * k;
        self.airspeed_s += (self.airspeed - self.airspeed_s) * k;

        let rev_per_sec = self.rpm_s / 60.0;

        // Exhaust pulses
        let prev = self.crank;
        self.crank = (self.crank + rev_per_sec * dt) % 2.0;
        let wrapped = self.crank < prev;

        for (i, &(fire_at, loudness)) in CYLINDERS.iter().enumerate() {
            let crossed = if wrapped {
                fire_at > prev || fire_at <= self.crank
            } else {
                fire_at > prev && fire_at <= self.crank
            };
            if crossed {
                // Combustion varies more at idle
                let roughness = 0.35 * (1.0 - self.throttle_s) + 0.1;
                let strength = 0.4 + 0.6 * self.throttle_s;
                self.pulses[i] = loudness * strength * (1.0 + roughness * self.noise());
            }
        }

        // Each bang decays over a few ms
        let decay = (-dt / 0.004).exp();
        let mut exhaust_in = 0.0;
        for p in &mut self.pulses {
            exhaust_in += *p;
            *p *= decay;
        }
        exhaust_in *= 0.7 + 0.3 * self.noise(); // part tone, part noise

        // Muffler resonance
        let cutoff = 250.0 + 450.0 * self.throttle_s;
        let f = 2.0 * (std::f32::consts::PI * cutoff * dt).sin();
        let q = 0.5; // 1/Q, lower = more resonant
        self.low += f * self.band;
        let high = exhaust_in - self.low - q * self.band;
        self.band += f * high;
        let exhaust = self.low;

        // Propeller
        // TODO: This should also probably come from some kinda engine/prop configs?
        self.prop_phase = (self.prop_phase + 2.0 * rev_per_sec * dt) % 1.0;
        let p = self.prop_phase * TAU;
        let prop_wave = p.sin() + 0.5 * (2.0 * p).sin() + 0.3 * (3.0 * p).sin();
        let tip = (self.rpm_s / 2700.0).powi(2); // louder as tips near mach
        let prop = prop_wave * tip * 0.25;

        // Wind (just white noise)
        let n = self.noise();
        self.wind += (n - self.wind) * 0.05;
        let wind = self.wind * (self.airspeed_s / 60.0).powi(2) * 0.6;

        // Mix it all together
        let mix = exhaust * 1.5 + prop + wind;
        (mix * 1.2).tanh() * 0.5 // clip peaks so they don't crackle
    }
}
