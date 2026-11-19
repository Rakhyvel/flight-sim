//! Terrain colors: sand at the shore, green to sage grass inland, gray stone where it's high and steep.
//! Shading comes from the 3D shader, so these are flat base colors

use apricot::terrain::chunks::{ChunkCoord, ChunkGrid};
use nalgebra_glm::Vec3;

use crate::heightmap::Heightmap;

const SAND: [f32; 3] = [222.0, 212.0, 165.0]; // pale yellow from the image
const GRASS_LOW: [f32; 3] = [96.0, 138.0, 58.0]; // lush green
const GRASS_HIGH: [f32; 3] = [150.0, 152.0, 86.0]; // dry, olive-tan upland grass
const STONE: [f32; 3] = [120.0, 115.0, 110.0]; // the old terrain's gray

/// Sand up to this height, then it fades into grass over `SAND_FADE` [m]
const BEACH_TOP: f32 = 8.0;
const SAND_FADE: f32 = 20.0;

/// RGBA pixels for one chunk, one texel per cell, rows along +Z
pub fn chunk_texture(map: &Heightmap, grid: &ChunkGrid, chunk: ChunkCoord) -> Vec<u8> {
    let o = grid.origin(chunk);
    let (x0, z0) = (o.x as i32, o.z as i32);
    let n = grid.chunk_cells as i32;

    let mut pixels = Vec::with_capacity((n * n * 4) as usize);
    for z in 0..n {
        for x in 0..n {
            let (gx, gz) = (x0 + x, z0 + z);
            let h00 = map.sample(gx, gz);
            let h10 = map.sample(gx + 1, gz);
            let h01 = map.sample(gx, gz + 1);
            let h11 = map.sample(gx + 1, gz + 1);
            let h = (h00 + h10 + h01 + h11) * 0.25;

            // Rise per meter across the cell (tan of the slope angle)
            let dx = ((h10 + h11) - (h00 + h01)) * 0.5 / map.spacing_x;
            let dz = ((h01 + h11) - (h00 + h10)) * 0.5 / map.spacing_z;
            let slope = (dx * dx + dz * dz).sqrt();

            let color = terrain_color(h, slope, map.max_height);
            pixels.extend_from_slice(&[color.x as u8, color.y as u8, color.z as u8, 255]);
        }
    }
    pixels
}

fn terrain_color(h: f32, slope: f32, max_height: f32) -> Vec3 {
    let sand = Vec3::from(SAND);
    if h <= 0.0 {
        // Sea floor: sand, which the translucent sea tints blue
        return sand * 0.85;
    }
    let up = (h / max_height.max(1.0)).clamp(0.0, 1.0); // 0 at sea level, 1 at the summit

    // Grass gets drier and paler with height
    let grass = Vec3::from(GRASS_LOW).lerp(&Vec3::from(GRASS_HIGH), smoothstep(0.0, 0.5, up));

    // Beach fades into grass
    let mut color = sand.lerp(&grass, smoothstep(BEACH_TOP, BEACH_TOP + SAND_FADE, h));

    // Stone only where it's both high and steep
    let steep = smoothstep(0.35, 0.8, slope); // about 20° to 40°
    let high = 1.0; //smoothstep(0.15, 0.4, up);
    color = color.lerp(&Vec3::from(STONE), steep * high);

    color
}

fn smoothstep(edge0: f32, edge1: f32, x: f32) -> f32 {
    let t = ((x - edge0) / (edge1 - edge0)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}
