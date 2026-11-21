use std::error::Error;

use apricot::{
    camera::floating_origin::FloatingOrigin,
    model_set::{Model, ModelSet},
    opengl::DrawOptions,
    render::{assets::MeshId, lighting::ShadowPass, RenderContext},
    terrain::chunks::{ChunkCoord, ChunkGrid},
};
use nalgebra_glm::{vec3, vec4, DVec2, DVec3, Mat4, Vec2, Vec3};

use crate::{
    airport::{self, Airport, Navaid, Runway},
    heightmap::Heightmap,
    region::Region,
    runway_model, terrain_color,
    waypoint::{Waypoint, WaypointKind},
};

const CHUNK_CELLS: u32 = 64;
const LOAD_RADIUS: i32 = 32;
/// How far a runway has to be from the edge of the map to be used [m]
const EDGE_INSET: f64 = 200.0;

/// The origin snaps to multiples of this [m] so it's a whole number and exact even in f32
const ORIGIN_SNAP: f64 = 1024.0;
const RECENTER_DISTANCE: f64 = 2048.0;

pub struct World {
    pub map: Heightmap,
    pub runways: Vec<Runway>,
    /// Where each runway's meshes are anchored, and its asphalt and markings
    runway_meshes: Vec<(DVec3, MeshId, MeshId)>,
    chunk_grid: ChunkGrid,
    pub chunks: ModelSet<ChunkCoord>,
    /// Everything is drawn relative to this
    pub origin: FloatingOrigin,
    /// Every airport and navaid on the map
    pub waypoints: Vec<Waypoint>,
}

impl World {
    pub fn load(renderer: &RenderContext, region: &Region) -> Result<Self, Box<dyn Error>> {
        // Load the map
        let mut map = Heightmap::load_geotiff(&format!("assets/{}", region.tiff))?;

        // Load the airports and navaids
        let on_map = |lat: f64, lon: f64| map.contains_lat_lon(lat, lon, 0.0);
        let mut waypoints = Vec::new();
        for Airport {
            ident,
            latitude_deg,
            longitude_deg,
            ..
        } in airport::load_airports("assets/airports.csv", on_map)?
        {
            waypoints.push(Waypoint {
                name: ident,
                pos: map.geo.to_world(latitude_deg, longitude_deg),
                altitude: None,
                kind: WaypointKind::Airport,
            })
        }
        for Navaid {
            ident,
            latitude_deg,
            longitude_deg,
            ..
        } in airport::load_navaids("assets/navaids.csv", on_map)?
        {
            waypoints.push(Waypoint {
                name: ident,
                pos: map.geo.to_world(latitude_deg, longitude_deg),
                altitude: None,
                kind: WaypointKind::Navaid,
            })
        }

        // Load the runways
        let runways = airport::load_runways("assets/runways.csv", |le, he| {
            map.contains_lat_lon(le.0, le.1, EDGE_INSET)
                && map.contains_lat_lon(he.0, he.1, EDGE_INSET)
        })?;

        let strips: Vec<(DVec2, DVec2, f32)> = runways
            .iter()
            .step_by(2)
            .map(|r| {
                (
                    map.geo.to_world(r.start.0, r.start.1),
                    map.geo.to_world(r.end.0, r.end.1),
                    r.width,
                )
            })
            .collect();
        for &(a, b, width) in &strips {
            map.flatten_runway(a.cast(), b.cast(), width);
        }
        let runway_meshes = strips
            .iter()
            .map(|&(a, b, width)| {
                let (asphalt, marks) =
                    runway_model::runway_meshes(a, b, width, |x, z| map.ground(x, z));
                (
                    DVec3::new(a.x, 0.0, a.y),
                    renderer.add_mesh_data(asphalt, None),
                    renderer.add_mesh_data(marks, None),
                )
            })
            .collect();

        Ok(Self {
            map,
            runways,
            runway_meshes,
            chunk_grid: ChunkGrid::new(CHUNK_CELLS),
            chunks: ModelSet::new(),
            origin: FloatingOrigin::default(),
            waypoints,
        })
    }

    /// Height and normal of the ground at world (x, z)
    pub fn ground(&self, x: f64, z: f64) -> (f32, Vec3) {
        self.map.ground(x, z)
    }

    pub fn runway(&self, name: &str) -> Option<&Runway> {
        self.runways.iter().find(|r| r.name == name)
    }

    /// Where to put a plane on `runway`
    pub fn spawn_point(&self, runway: &Runway) -> (DVec3, f64) {
        let a = self.map.geo.to_world(runway.start.0, runway.start.1);
        let b = self.map.geo.to_world(runway.end.0, runway.end.1);
        let dir = (b - a).normalize();
        let spot = a + dir * 30.0;
        let (ground, _) = self.ground(spot.x, spot.y);
        (vec3(spot.x, ground as f64, spot.y), dir.x.atan2(-dir.y))
    }

    pub fn recenter(&mut self, pos: DVec3) {
        let d = pos - self.origin.pos;
        if d.x.abs() < RECENTER_DISTANCE && d.z.abs() < RECENTER_DISTANCE {
            return;
        }
        let snap = |v: f64| (v / ORIGIN_SNAP).round() * ORIGIN_SNAP;
        self.origin.pos = DVec3::new(snap(pos.x), 0.0, snap(pos.z));

        let moved: Vec<_> = self
            .chunks
            .iter()
            .map(|(k, _)| (k, self.chunk_transform(k)))
            .collect();
        for (k, transform) in moved {
            self.chunks.set_transform(k, transform);
        }
    }

    fn chunk_transform(&self, chunk: ChunkCoord) -> Mat4 {
        let o = self.chunk_grid.origin(chunk); // in cells, whole numbers
        let corner = DVec3::new(
            o.x as f64 * self.map.geo.spacing_x,
            0.0,
            o.z as f64 * self.map.geo.spacing_z,
        );
        let scale = nalgebra_glm::scaling(&vec3(self.map.spacing_x, 1.0, self.map.spacing_z));
        self.origin.transform(corner, scale)
    }

    /// Stream terrain chunks around `pos`, building at most `max_loads` of them
    pub fn update(&mut self, renderer: &RenderContext, pos: DVec3, max_loads: usize) {
        let (sx, sz) = (self.map.geo.spacing_x, self.map.geo.spacing_z);
        let cell_pos: Vec3 = vec3((pos.x / sx) as f32, 0.0, (pos.z / sz) as f32);

        let (w, d) = self.map.size();
        let max_cx = (w as u32).div_ceil(CHUNK_CELLS) as i32;
        let max_cz = (d as u32).div_ceil(CHUNK_CELLS) as i32;

        // Closest first, and only chunks that are actually on the map
        let wanted: Vec<ChunkCoord> = self
            .chunk_grid
            .chunks_near(cell_pos, LOAD_RADIUS)
            .into_iter()
            .filter(|c| c.x >= 0 && c.z >= 0 && c.x < max_cx && c.z < max_cz)
            .collect();

        // Unload anything that's dropped out of range
        let stale: Vec<ChunkCoord> = self
            .chunks
            .iter()
            .map(|(k, _)| k)
            .filter(|k| !wanted.contains(k))
            .collect();
        for k in stale {
            if let Some(model) = self.chunks.remove(k) {
                renderer.unload_mesh(model.mesh);
            }
        }

        let missing: Vec<ChunkCoord> = wanted
            .into_iter()
            .filter(|&c| self.chunks.get(c).is_none())
            .take(max_loads)
            .collect();

        // Build a few missing chunks per frame, so flying fast doesn't cause a hitch
        let map = &self.map; // borrow just the map, so the closure doesn't borrow all of self
        for chunk in missing {
            let mesh = self
                .chunk_grid
                .mesh_smooth(chunk, |x, z| map.sample(x as i32, z as i32));
            let mesh_id = renderer.add_mesh_data(mesh, None);

            let transform = self.chunk_transform(chunk);

            let pixels = terrain_color::chunk_texture(map, &self.chunk_grid, chunk);
            let texture = renderer.create_texture_rgba(CHUNK_CELLS, CHUNK_CELLS, &pixels);
            renderer.get_texture_from_id(texture).set_smooth(true);

            self.chunks
                .insert(renderer, chunk, Model::new(mesh_id, texture, transform));
        }
    }

    pub fn draw_shadows(&self, pass: &ShadowPass) {
        self.chunks.draw_shadows(pass);
    }

    pub fn draw(&self, renderer: &RenderContext) {
        let p = renderer.builtins.program_3d;

        // Draw terrain
        renderer.uniform(p, "u_detail").set_f32(1.0);
        self.chunks.draw(renderer);
        renderer.uniform(p, "u_detail").set_f32(0.0);

        // Draw runways
        let asphalt = DrawOptions {
            depth_bias: Some((-1.0, -4.0)),
            ..Default::default()
        };
        let paint = DrawOptions {
            depth_bias: Some((-2.0, -8.0)),
            ..Default::default()
        };
        let white = renderer.builtins.white_texture;
        for &(anchor, surface, marks) in &self.runway_meshes {
            let model = self.origin.transform(anchor, Mat4::identity());
            renderer.draw_mesh_textured_with(
                surface,
                white,
                model,
                vec4(0.33, 0.33, 0.34, 1.0),
                &asphalt,
            );
            renderer.draw_mesh_textured_with(
                marks,
                white,
                model,
                vec4(0.85, 0.85, 0.82, 1.0),
                &paint,
            );
        }
    }

    pub fn waypoint(&self, name: &str) -> Option<&Waypoint> {
        self.waypoints.iter().find(|w| w.name == name)
    }

    /// Generate a straight-in approach to a runway
    pub fn approach(&self, runway: &Runway) -> Vec<Waypoint> {
        const M_PER_NM: f64 = 1852.0;
        const INITIAL_FIX_OUT: f64 = 10.0 * M_PER_NM;
        const INITIAL_FIX_HEIGHT: f64 = 3000.0 * 0.3048;
        const FINAL_APPROACH_OUT: f64 = 5.0 * M_PER_NM;
        const THRESHOLD_CROSSING_HEIGHT: f64 = 15.0;

        let glide = 3.0_f64.to_radians().tan(); // 3 deg glide slope
        let final_approach_height: f64 = glide * 5.0 * M_PER_NM;

        let a = self.map.geo.to_world(runway.start.0, runway.start.1);
        let b = self.map.geo.to_world(runway.end.0, runway.end.1);
        let inbound = (b - a).normalize();
        let (threshold_h, _) = self.ground(a.x, a.y);

        let fix = |label: &str, out: f64, cross_height: f64| Waypoint {
            name: format!("{} {label}", runway.name),
            pos: a - inbound * out,
            altitude: Some(threshold_h as f64 + cross_height),
            kind: WaypointKind::Fix,
        };

        vec![
            fix("IAF", INITIAL_FIX_OUT, INITIAL_FIX_HEIGHT),
            fix("FAF", FINAL_APPROACH_OUT, final_approach_height),
            fix("IAF", 0.0, THRESHOLD_CROSSING_HEIGHT),
        ]
    }
}
