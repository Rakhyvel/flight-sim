use apricot::terrain::{surface_height, surface_normal};
use nalgebra_glm::{smoothstep, vec2, vec3, Vec2, Vec3};
use std::{fs::File, io::BufReader};
use tiff::{
    decoder::{Decoder, DecodingResult, Limits},
    tags::Tag,
};

pub const SEA_FLOOR: f32 = -30.0;

/// Where the map sits on earth. Sample (0, 0) is at world (0, 0), x runs east, z runs south.
#[derive(Clone, Copy, Debug)]
pub struct GeoRef {
    /// Latitude and longitude of sample (0, 0) [deg]
    lat0: f64,
    lon0: f64,
    /// Degrees per sample, east and south
    dlon: f64,
    dlat: f64,
    /// Meters per sample, east and south
    spacing_x: f64,
    spacing_z: f64,
}

impl GeoRef {
    /// World (x, z) [m] of a latitude and longitude [deg]
    pub fn to_world(&self, lat: f64, lon: f64) -> Vec2 {
        vec2(
            ((lon - self.lon0) / self.dlon * self.spacing_x) as f32,
            ((self.lat0 - lat) / self.dlat * self.spacing_z) as f32,
        )
    }

    /// Latitude and longitude [deg] of world (x, z) [m]
    pub fn to_lat_lon(&self, x: f32, z: f32) -> (f64, f64) {
        (
            self.lat0 - z as f64 / self.spacing_z * self.dlat,
            self.lon0 + x as f64 / self.spacing_x * self.dlon,
        )
    }
}

/// Length of one degree at latitude on the WGS84 ellipsoid at `lat`
fn meters_per_deg_lat(lat: f64) -> f64 {
    111_132.92 - 559.82 * (2.0 * lat).cos() + 1.175 * (4.0 * lat).cos()
}

/// Length of one degree at longitude on the WGS84 ellipsoid at `lat`
fn meters_per_deg_lon(lat: f64) -> f64 {
    111_412.84 * lat.cos() - 93.5 * (3.0 * lat).cos() + 0.118 * (5.0 * lat).cos()
}

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
    pub geo: GeoRef,
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

        let geokeys = decoder.get_tag_u16_vec(Tag::GeoKeyDirectoryTag)?;
        let geo_key = |key: u16| {
            geokeys[4..]
                .chunks_exact(4)
                .find(|e| e[0] == key && e[1] == 0) // location 0 is the value
                .map(|e| e[3])
        };
        if geo_key(1024) != Some(2) {
            // Gotta be type 2 babey (geographic lat/long)
            return Err("only lat/long GeoTIFFs are supported".into());
        }

        let half = if geo_key(1025) == Some(2) { 0.0 } else { 0.5 };

        let (dlon, dlat) = (pixel_scale[0], pixel_scale[1]);
        // The tiepoint pins pixel (tiepoint[0], tiepoint[1]) to (lon, lat). Move that to the center of sample (0, 0)
        let lon0 = tiepoint[3] + (half - tiepoint[0]) * dlon;
        let lat0 = tiepoint[4] - (half - tiepoint[1]) * dlat;

        let mid_lat = (lat0 - dlat * depth as f64 / 2.0).to_radians();
        let geo = GeoRef {
            lat0,
            lon0,
            dlon,
            dlat,
            spacing_x: dlon * meters_per_deg_lon(mid_lat),
            spacing_z: dlat * meters_per_deg_lat(mid_lat),
        };

        let (spacing_x, spacing_z) = (geo.spacing_x as f32, geo.spacing_z as f32);

        println!("Loaded {path}: {width}x{depth}, {spacing_x:.1} x {spacing_z:.1} m per sample");
        Ok(Self {
            width,
            depth,
            heights,
            spacing_x,
            spacing_z,
            max_height,
            geo,
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

    /// Smooth ground height at (x, z)
    pub fn ground(&self, x: f32, z: f32) -> (f32, Vec3) {
        let height = |cx: f32, cz: f32| self.sample(cx as i32, cz as i32);
        let (cx, cz) = (x / self.spacing_x, z / self.spacing_z);
        let h = surface_height(height, cx, cz);
        if h <= 0.0 {
            return (0.0, Vec3::y());
        }

        let n = surface_normal(height, cx, cz);
        (
            h,
            // Conver the normal's cell units to meters
            vec3(n.x / self.spacing_x, n.y, n.z / self.spacing_z).normalize(),
        )
    }

    /// BEEP BEEP STEAM ROLLER COMING THROUGH BABEY!
    pub fn flatten_runway(&mut self, a: Vec2, b: Vec2, width: f32) {
        let len = (b - a).norm();
        let dir = (b - a) / len;
        let cell = self.spacing_x.max(self.spacing_z);

        // Figure out the best line through the terrain along the centerline
        let n = (len / 10.0).ceil().max(2.0) as usize;
        let (mut st, mut sh, mut stt, mut sth) = (0.0_f64, 0.0_f64, 0.0_f64, 0.0_f64);
        for k in 0..=n {
            let t = len * k as f32 / n as f32;
            let h = self.height_near(a + dir * t).max(0.0) as f64;
            let t = t as f64;
            st += t;
            sh += h;
            stt += t * t;
            sth += t * h;
        }
        let m = (n + 1) as f64;
        let slope = ((m * sth - st * sh) / (m * stt - st * st)) as f32;
        let h0 = ((sh - slope as f64 * st) / m) as f32;

        // Flat out to one cell past the edge so no terrain triangle pokes through the runway
        let flat = 0.5 * width + cell;
        let blend = 4.0 * cell;
        let overrun = 2.0 * cell;

        let reach = Vec2::repeat(flat + blend + overrun);
        let (lo, hi) = (a.inf(&b) - reach, a.sup(&b) + reach);
        let i0 = ((lo.x / self.spacing_x).floor() as i32).max(0);
        let i1 = ((hi.x / self.spacing_x).ceil() as i32).min(self.width as i32 - 1);
        let j0 = ((lo.y / self.spacing_z).floor() as i32).max(0);
        let j1 = ((hi.y / self.spacing_z).ceil() as i32).min(self.depth as i32 - 1);

        for j in j0..=j1 {
            for i in i0..=i1 {
                let p = vec2(i as f32 * self.spacing_x, j as f32 * self.spacing_z) - a;
                let along = p.dot(&dir);
                let across = p.perp(&dir).abs();
                // How far outside the flat area this sample is
                let past_end = (-along - overrun).max(along - len - overrun).max(0.0);
                let past_side = (across - flat).max(0.0);
                let outside = (past_end * past_end + past_side * past_side).sqrt();

                let w = 1.0 - smoothstep(0.0, blend, outside);
                if w <= 0.0 {
                    continue;
                }
                let target = h0 + slope * along.clamp(0.0, len);
                let h = &mut self.heights[i as usize + j as usize * self.width];
                *h += (target - *h) * w;
            }
        }
    }

    /// The sample nearest world (x, z)
    fn height_near(&self, p: Vec2) -> f32 {
        self.sample(
            (p.x / self.spacing_x).round() as i32,
            (p.y / self.spacing_z).round() as i32,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lat_lon_round_trips_through_the_world() {
        let geo = GeoRef {
            lat0: 21.27,
            lon0: -157.33,
            dlon: 1.0 / 3600.0,
            dlat: 1.0 / 3600.0,
            spacing_x: 28.9,
            spacing_z: 30.7,
        };
        let p = geo.to_world(20.9, -156.43);
        let (lat, lon) = geo.to_lat_lon(p.x, p.y);
        assert!((lat - 20.9).abs() < 1e-6 && (lon + 156.43).abs() < 1e-6);
        // North is -Z and east is +X
        assert!(geo.to_world(21.0, -157.33).y > 0.0 && geo.to_world(21.27, -157.0).x > 0.0);
    }
}
