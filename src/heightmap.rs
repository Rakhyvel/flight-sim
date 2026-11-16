use std::{fs::File, io::BufReader};
use tiff::{
    decoder::{Decoder, DecodingResult, Limits},
    tags::Tag,
};

pub const SEA_FLOOR: f32 = -30.0;

/// A real-world elevation grid loaded from a GeoTIFF
pub struct Heightmap {
    width: usize,
    depth: usize,
    heights: Vec<f32>,
    /// Meters between samples, east-west
    pub spacing_x: f32,
    /// Meters between samples, north-south
    pub spacing_z: f32,
    /// Highest sample in the map [m]
    pub max_height: f32,
}

impl Heightmap {
    /// Load elevation (in meters) from a GeoTIFF DEM
    pub fn load_geotiff(path: &str) -> Result<Self, Box<dyn std::error::Error>> {
        const SCALE: f64 = 1.0;
        let mut decoder =
            Decoder::new(BufReader::new(File::open(path)?))?.with_limits(Limits::unlimited());
        let (width, depth) = decoder.dimensions()?;
        let (width, depth) = (width as usize, depth as usize);

        // The pixel size [sx, sy, sz]
        let pixel_scale = decoder.get_tag_f64_vec(Tag::ModelPixelScaleTag)?;
        // The tiepoint [i, j, k, x, y, z] that pins pixel (0, 0) to a map coordinate
        let tiepoint = decoder.get_tag_f64_vec(Tag::ModelTiepointTag)?;

        let raw: Vec<f32> = match decoder.read_image()? {
            DecodingResult::F32(v) => v,
            DecodingResult::F64(v) => v.into_iter().map(|h| h as f32).collect(),
            DecodingResult::I16(v) => v.into_iter().map(f32::from).collect(),
            DecodingResult::U16(v) => v.into_iter().map(f32::from).collect(),
            DecodingResult::I32(v) => v.into_iter().map(|h| h as f32).collect(),
            _ => return Err("unsupported DEM sample type".into()),
        };

        // The ocean is "no data", usually a huge negative number or NaN. Make it sea level
        let heights: Vec<f32> = raw
            .into_iter()
            .map(|h| {
                if h.is_finite() && h > 0.0 {
                    h * SCALE as f32
                } else {
                    SEA_FLOOR
                }
            })
            .collect();

        let max_height = heights.iter().copied().fold(0.0_f32, f32::max);

        let (spacing_x, spacing_z) = if pixel_scale[0] < 1.0 {
            // Tiny pixel size means degrees (lat/lon). Convert using the map's middle latitude
            const METERS_PER_DEGREE: f64 = 111_320.0 * SCALE;
            let top_lat = tiepoint[4];
            let mid_lat = (top_lat - pixel_scale[1] * depth as f64 / 2.0).to_radians();
            (
                (pixel_scale[0] * METERS_PER_DEGREE * mid_lat.cos()) as f32,
                (pixel_scale[1] * METERS_PER_DEGREE) as f32,
            )
        } else {
            // Already projected, so the pixel size is in meters
            (pixel_scale[0] as f32, pixel_scale[1] as f32)
        };

        println!("Loaded {path}: {width}x{depth}, {spacing_x:.1} x {spacing_z:.1} m per sample");
        Ok(Self {
            width,
            depth,
            heights,
            spacing_x,
            spacing_z,
            max_height,
        })
    }

    /// Height at sample (x, z). Off the edge is sea level
    pub fn sample(&self, x: i32, z: i32) -> f32 {
        if x < 0 || z < 0 || x as usize >= self.width || z as usize >= self.depth {
            return 0.0;
        }
        self.heights[x as usize + z as usize * self.width]
    }

    /// Size in samples
    pub fn size(&self) -> (usize, usize) {
        (self.width, self.depth)
    }
}
