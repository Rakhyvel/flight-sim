//! Terrain colors: sand at the shore, green to sage grass inland, gray stone where it's high and steep.
//! Shading comes from the 3D shader, so these are flat base colors

use apricot::terrain::{
    chunks::{ChunkCoord, ChunkGrid},
    noise::value_noise,
};
use nalgebra_glm::Vec3;

use crate::heightmap::Heightmap;

// In linear albedo
const SEA_FLOOR: [f32; 3] = [0.18, 0.16, 0.11];
const SAND: [f32; 3] = [0.36, 0.31, 0.22];
const MEADOW: [f32; 3] = [0.07, 0.11, 0.035];
const DRY_GRASS: [f32; 3] = [0.17, 0.14, 0.07];
const FOREST: [f32; 3] = [0.025, 0.04, 0.018];
const ROCK: [f32; 3] = [0.16, 0.15, 0.13];
const SNOW: [f32; 3] = [0.80, 0.82, 0.85];

/// Sand up to this height, then it fades into grass over `SAND_FADE` [m]
const BEACH_TOP: f32 = 8.0;
const SAND_FADE: f32 = 20.0;
/// Trees thin out up to here, and snow settles around `SNOWLINE` [m]
const TREELINE: f32 = 1800.0;
const SNOWLINE: f32 = 2800.0;

/// RGBA pixels for one chunk, one texel per cell, rows along +Z
pub fn chunk_texture(map: &Heightmap, grid: &ChunkGrid, chunk: ChunkCoord) -> Vec<u8> {
    let o = grid.origin(chunk);
    let (x0, z0) = (o.x as i32, o.z as i32);
    let n = grid.chunk_cells as i32;

    let mut pixels = Vec::with_capacity((n * n * 4) as usize);
    for z in 0..n {
        for x in 0..n {
            let c = terrain_albedo(map, x0 + x, z0 + z);
            let srgb = |v: f32| (v.clamp(0.0, 1.0).powf(1.0 / 2.2) * 255.0).round() as u8;
            pixels.extend_from_slice(&[srgb(c.x), srgb(c.y), srgb(c.z), 255]);
        }
    }
    pixels
}

fn terrain_albedo(map: &Heightmap, gx: i32, gz: i32) -> Vec3 {
    let h = map.sample(gx, gz);
    if h <= 0.0 {
        return SEA_FLOOR.into();
    }
    let (wx, wz) = (gx as f32 * map.spacing_x, gz as f32 * map.spacing_z); // meters, for noise

    // Rise per meter
    let dx = (map.sample(gx + 1, gz) - map.sample(gx - 1, gz)) / (2.0 * map.spacing_x);
    let dz = (map.sample(gx, gz + 1) - map.sample(gx, gz - 1)) / (2.0 * map.spacing_z);
    let slope = (dx * dx + dz * dz).sqrt();

    // Get curvature, water collects in valleys so they're greener
    let r = 5;
    let ring = (map.sample(gx + r, gz)
        + map.sample(gx - r, gz)
        + map.sample(gx, gz + r)
        + map.sample(gx, gz - r))
        * 0.25;
    let hollow = ((ring - h) / 15.0).clamp(-1.0, 1.0);

    // Add big patches and fine speckle
    let patches = value_noise(wx, wz, 1.0 / 900.0, 3, 1);
    let speckle = value_noise(wx, wz, 1.0 / 60.0, 2, 2);

    let moisture = (patches + 0.35 * hollow).clamp(0.0, 1.0);

    // Grass is dry on ridges and in dry patches, lush in wet ones
    let mut c = lerp(DRY_GRASS, MEADOW, smoothstep(0.3, 0.6, moisture));

    // Forest where it's wet, not too steep, and below the treeline
    let forest = smoothstep(0.55, 0.7, moisture)
        * (1.0 - smoothstep(0.6, 0.9, slope))
        * (1.0 - smoothstep(TREELINE - 300.0, TREELINE, h));
    c = c.lerp(&FOREST.into(), forest);

    // Beach with a nice ragged edge
    let beach_h = h + (speckle - 0.5) * 6.0;
    c = Vec3::from(SAND).lerp(&c, smoothstep(BEACH_TOP, BEACH_TOP + SAND_FADE, beach_h));

    // Rock on steep slopes
    let rock = smoothstep(0.45, 0.8, slope).max(0.7 * smoothstep(TREELINE, TREELINE + 600.0, h));
    c = c.lerp(&ROCK.into(), rock);

    // Snow high up, except where it's too steep
    let snow = smoothstep(
        SNOWLINE - 200.0,
        SNOWLINE + 200.0,
        h + (patches - 0.5) * 400.0,
    ) * (1.0 - smoothstep(0.7, 1.1, slope));
    c = c.lerp(&SNOW.into(), snow);

    c * (0.85 + 0.3 * speckle)
}

fn lerp(a: [f32; 3], b: [f32; 3], t: f32) -> Vec3 {
    Vec3::from(a).lerp(&Vec3::from(b), t)
}

fn smoothstep(edge0: f32, edge1: f32, x: f32) -> f32 {
    let t = ((x - edge0) / (edge1 - edge0)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}
