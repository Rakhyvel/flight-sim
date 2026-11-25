use std::{
    borrow::Cow,
    path::{Path, PathBuf},
};

use apricot::render::{assets::TextureId, RenderContext};
use image::{imageops, Rgba, RgbaImage};
use nalgebra_glm::{vec2, vec3, Vec2, Vec3};

use crate::heightmap::{Bounds, Heightmap};

/// Least ground a map shows across [m]
const MIN_VIEW: f64 = 25_000.0;
/// Room left around the route, as a fraction of its size
const VIEW_MARGIN: f64 = 0.2;

const CACHE_DIR: &str = "assets/cache";
const OVERVIEW_WIDTH: f64 = 1040.0; // [px]
const BACKDROP_WIDTH: u32 = 384; // [px]
const BACKDROP_BLUR: f32 = 6.0; // [px, gaussian sigma]
/// How much steeper the hills get drawn so that they show up
const RELIEF_EXAGGERATION: f32 = 4.0;
const M_PER_DEG_LAT: f64 = 111_320.0;

const SEA: Vec3 = Vec3::new(0.16, 0.27, 0.38);
const LOWLAND: Vec3 = Vec3::new(0.45, 0.50, 0.38);
const HIGHLAND: Vec3 = Vec3::new(0.80, 0.74, 0.62);

pub struct Overview {
    pub texture: TextureId,
    pub size: Vec2, // [px]
    /// Blurred, for behind the menu
    pub backdrop: TextureId,
    pub backdrop_size: Vec2, // [px]
    pub corners: Bounds,
}

impl Overview {
    /// The overview made for the region in `tiff`, if one's been made yet
    pub fn load(renderer: &RenderContext, tiff: &str) -> Option<Self> {
        let corners = std::fs::read_to_string(cache_path(tiff, ".toml")).ok()?;
        let corners: Bounds = toml::from_str(&corners).ok()?;
        let (texture, size) = load_texture(renderer, &cache_path(tiff, ".png"))?;
        let (backdrop, backdrop_size) = load_texture(renderer, &cache_path(tiff, "_backdrop.png"))?;
        Some(Self {
            texture,
            size,
            backdrop,
            backdrop_size,
            corners,
        })
    }

    /// Where `lat`, `lon` [deg] is on the overview [px]
    pub fn pixel(&self, lat: f64, lon: f64) -> Vec2 {
        let c = &self.corners;
        vec2(
            ((lon - c.west) / (c.east - c.west)) as f32 * self.size.x,
            ((c.north - lat) / (c.north - c.south)) as f32 * self.size.y,
        )
    }

    /// Overview pixels per meter of ground
    pub fn px_per_m(&self) -> f32 {
        let c = &self.corners;
        self.size.y / ((c.north - c.south) * M_PER_DEG_LAT) as f32
    }
}

/// Shade `map` into an overview, and save it, a blurred copy, and where its corners are
pub fn bake(map: &Heightmap, tiff: &str) -> Result<(), Box<dyn std::error::Error>> {
    let (w, d) = map.size();
    let (w, d) = (w as i32, d as i32);
    let ground_width = w as f64 * map.geo.spacing_x; // [m]
    let ground_depth = d as f64 * map.geo.spacing_z; // [m]
    let width = OVERVIEW_WIDTH.min(w as f64);
    let height = width * ground_depth / ground_width; // square pixels on the ground
    let (step_x, step_z) = (w as f64 / width, d as f64 / height); // [samples per px]
    let m_per_px = (ground_width / width) as f32;

    let at = |x: f64, z: f64| map.sample((x as i32).clamp(0, w - 1), (z as i32).clamp(0, d - 1));
    let sun = vec3(-1.0, 1.0, -1.0).normalize(); // from the northwest, like paper maps
    let image = RgbaImage::from_fn(width as u32, height as u32, |px, pz| {
        let (x, z) = (px as f64 * step_x, pz as f64 * step_z);
        let h = at(x, z);
        let color = if h <= 0.0 {
            SEA
        } else {
            let dx = (at(x + step_x, z) - at(x - step_x, z)) / (2.0 * m_per_px);
            let dz = (at(x, z + step_z) - at(x, z - step_z)) / (2.0 * m_per_px);
            let normal =
                vec3(-dx * RELIEF_EXAGGERATION, 1.0, -dz * RELIEF_EXAGGERATION).normalize();
            let shade = (normal.dot(&sun) / sun.y).clamp(0.3, 1.3); // flat ground comes out at 1
            LOWLAND.lerp(&HIGHLAND, (h / map.max_height).sqrt()) * shade
        };
        Rgba([byte(color.x), byte(color.y), byte(color.z), 255])
    });

    let (north, west) = map.geo.to_lat_lon(0.0, 0.0);
    let (south, east) = map.geo.to_lat_lon(ground_width, ground_depth);
    let corners = Bounds {
        north,
        west,
        south,
        east,
    };

    std::fs::create_dir_all(CACHE_DIR)?;
    image.save(cache_path(tiff, ".png"))?;
    let backdrop_height = (BACKDROP_WIDTH as f64 * height / width) as u32;
    let small = imageops::resize(
        &image,
        BACKDROP_WIDTH,
        backdrop_height,
        imageops::FilterType::Triangle,
    );
    imageops::blur(&small, BACKDROP_BLUR).save(cache_path(tiff, "_backdrop.png"))?;
    std::fs::write(cache_path(tiff, ".toml"), toml::to_string(&corners)?)?;
    Ok(())
}

fn cache_path(tiff: &str, suffix: &str) -> PathBuf {
    let stem = Path::new(tiff)
        .file_stem()
        .map_or(Cow::from(tiff), |s| s.to_string_lossy());
    PathBuf::from(CACHE_DIR).join(format!("{stem}{suffix}"))
}

fn load_texture(renderer: &RenderContext, path: &Path) -> Option<(TextureId, Vec2)> {
    let image = image::open(path).ok()?.to_rgba8();
    let (w, h) = image.dimensions();
    Some((
        renderer.create_texture_rgba(w, h, image.as_raw()),
        vec2(w as f32, h as f32),
    ))
}

fn byte(c: f32) -> u8 {
    (c.clamp(0.0, 1.0) * 255.0).round() as u8
}

pub fn bounds_around(points: impl Iterator<Item = (f64, f64)>, aspect: f64) -> Option<Bounds> {
    let empty = Bounds {
        north: -90.0,
        west: 180.0,
        south: 90.0,
        east: -180.0,
    };
    let b = points.fold(empty, |b, (lat, lon)| Bounds {
        north: b.north.max(lat),
        west: b.west.min(lon),
        south: b.south.min(lat),
        east: b.east.max(lon),
    });
    if b.north < b.south {
        return None;
    }

    let (mid_lat, mid_lon) = ((b.north + b.south) / 2.0, (b.east + b.west) / 2.0);
    let m_per_deg_lon = M_PER_DEG_LAT * mid_lat.to_radians().cos();
    let room = 1.0 + 2.0 * VIEW_MARGIN;
    let mut tall = ((b.north - b.south) * M_PER_DEG_LAT * room).max(MIN_VIEW); // [m]
    let mut wide = ((b.east - b.west) * m_per_deg_lon * room).max(MIN_VIEW); // [m]
    if wide < tall * aspect {
        wide = tall * aspect;
    } else {
        tall = wide / aspect;
    }
    let (half_lat, half_lon) = (tall / M_PER_DEG_LAT / 2.0, wide / m_per_deg_lon / 2.0);
    Some(Bounds {
        north: mid_lat + half_lat,
        west: mid_lon - half_lon,
        south: mid_lat - half_lat,
        east: mid_lon + half_lon,
    })
}
