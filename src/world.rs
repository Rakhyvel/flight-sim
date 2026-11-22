use std::{
    collections::{HashMap, HashSet},
    error::Error,
};

use apricot::{
    camera::floating_origin::FloatingOrigin,
    geometry::aabb::AABB,
    model_set::{Model, ModelSet},
    opengl::DrawOptions,
    render::{assets::MeshId, lighting::ShadowPass, RenderContext},
};
use nalgebra_glm::{vec3, vec4, DVec2, DVec3, Mat4, Vec3};

use crate::{
    airport::{self, Airport, Navaid, Runway},
    heightmap::Heightmap,
    region::Region,
    runway_model, terrain_color,
    tiles::{Tile, TileHeights, TileTree, TILE_QUADS},
    waypoint::{Waypoint, WaypointKind},
};

/// How far a runway has to be from the edge of the map to be used [m]
const EDGE_INSET: f64 = 200.0;

/// How far terrain is drawn [m]
pub const RENDER_DISTANCE: f64 = 150_000.0;
/// Runways farther than this are not drawn [m]
const RUNWAY_DRAW_DISTANCE: f64 = 25_000.0;
/// Level of the biggest tiles. 6 is 4096 cells, about 123 km at 30 m spacing
const ROOT_LEVEL: u32 = 6;
/// Never split a tile farther than this many times its size
const SPLIT: f64 = 1.5;
/// Always split a tile closer than this many times its size, for texture detail
const TEXTURE_SPLIT: f64 = 0.5;
/// Split a tile when its shape would be off by more than this [px]
const MAX_ERROR_PX: f64 = 2.0;
/// Roughly a 1400 px tall screen at a 45 degree field of view
const PIXELS_PER_RADIAN: f64 = 1400.0 / 0.785;
/// How far the skirts hang down below each chunk's edge [m]
const SKIRT_PER_STEP: f32 = 20.0;
/// Off the map the ground sinks under the sea, so the edge just looks like more ocean
const OFF_MAP_HEIGHT: f32 = -100.0;
/// The most the earth's curve lowers anything drawn [m]
const MAX_CURVE_DROP: f32 = (RENDER_DISTANCE * RENDER_DISTANCE / (2.0 * 6_371_000.0)) as f32;

/// The origin snaps to multiples of this [m] so it's a whole number and exact even in f32
const ORIGIN_SNAP: f64 = 1024.0;
const RECENTER_DISTANCE: f64 = 2048.0;

pub struct World {
    pub runways: Vec<Runway>,
    /// Where each runway's meshes are anchored, and its asphalt and markings
    runway_meshes: Vec<(DVec3, MeshId, MeshId)>,
    /// Every airport and navaid on the map
    pub waypoints: Vec<Waypoint>,

    pub map: Heightmap,
    /// Which tiles to draw
    tree: TileTree,
    /// Every built tile, drawn or not
    tiles: HashMap<Tile, Model>,
    /// The tiles being drawn this frame
    pub drawn: ModelSet<Tile>,

    /// Everything is drawn relative to this
    pub origin: FloatingOrigin,
}

impl World {
    pub fn load(renderer: &RenderContext, region: &Region) -> Result<Self, Box<dyn Error>> {
        // Load the map
        let mut map = Heightmap::load_geotiff(&format!("assets/{}", region.tiff))?;

        // Load the tiles
        let (w, d) = map.size();
        let tree = TileTree {
            root_level: ROOT_LEVEL,
            map_cells: (w as u32, d as u32),
            spacing: (map.geo.spacing_x, map.geo.spacing_z),
            split: SPLIT,
            texture_split: TEXTURE_SPLIT,
            max_distance: RENDER_DISTANCE,
            heights: TileHeights::new(w as u32, d as u32, ROOT_LEVEL, |x, z| map.sample(x, z)),
            max_error_px: MAX_ERROR_PX,
            pixels_per_radian: PIXELS_PER_RADIAN,
        };

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
            runways,
            runway_meshes,
            waypoints,
            map,
            tree,
            tiles: HashMap::new(),
            drawn: ModelSet::new(),
            origin: FloatingOrigin::default(),
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
            .drawn
            .iter()
            .map(|(k, _)| (k, self.tile_transform(k)))
            .collect();
        for (k, transform) in moved {
            self.drawn.set_transform(k, transform);
        }
    }

    fn tile_transform(&self, tile: Tile) -> Mat4 {
        let (ox, oz) = tile.origin();
        let corner = DVec3::new(
            ox as f64 * self.map.geo.spacing_x,
            0.0,
            oz as f64 * self.map.geo.spacing_z,
        );
        let scale = nalgebra_glm::scaling(&vec3(self.map.spacing_x, 1.0, self.map.spacing_z));
        self.origin.transform(corner, scale)
    }

    fn build_tile(&self, renderer: &RenderContext, tile: Tile) -> Model {
        let (ox, oz) = tile.origin();
        let map = &self.map;
        let (w, d) = map.size();
        let height = |x: f32, z: f32| {
            let (gx, gz) = (ox + x as i32, oz + z as i32);
            if gx < 0 || gz < 0 || gx as usize >= w || gz as usize >= d {
                OFF_MAP_HEIGHT
            } else {
                map.sample(gx, gz)
            }
        };

        let (n, step) = (tile.cells(), tile.step());
        let mesh = apricot::terrain::mesh_smooth(n, n, step, SKIRT_PER_STEP * step as f32, height);
        let mesh = renderer.add_mesh_data(mesh, None);

        let pixels = terrain_color::tile_texture(map, ox, oz, step, TILE_QUADS);
        let texture = renderer.create_texture_rgba(TILE_QUADS, TILE_QUADS, &pixels);
        renderer.get_texture_from_id(texture).set_smooth(true);

        Model::new(mesh, texture, self.tile_transform(tile))
    }

    /// Stream terrain tiles around `pos`, building at most `max_loads` of them
    pub fn update(&mut self, renderer: &RenderContext, pos: DVec3, max_loads: usize) {
        // Build what's missing, closest first
        let ideal = self.tree.ideal(pos);
        let todo: Vec<Tile> = ideal
            .iter()
            .copied()
            .filter(|t| !self.tiles.contains_key(t))
            .take(max_loads)
            .collect();
        for tile in todo {
            let model = self.build_tile(renderer, tile);
            self.tiles.insert(tile, model);
        }

        // Draw the best of what's built
        let draw = self.tree.drawable(pos, |t| self.tiles.contains_key(&t));
        let draw_set: HashSet<Tile> = draw.iter().copied().collect();

        let hide: Vec<Tile> = self
            .drawn
            .iter()
            .map(|(k, _)| k)
            .filter(|k| !draw_set.contains(k))
            .collect();
        for tile in hide {
            self.drawn.remove(tile);
        }
        for &tile in &draw {
            if self.drawn.get(tile).is_none() {
                let mut model = self.tiles[&tile];
                model.transform = self.tile_transform(tile); // the origin may have moved since it was built
                let mut bounds: AABB = renderer.get_mesh_aabb(model.mesh);
                bounds.min.y -= MAX_CURVE_DROP;
                self.drawn.insert_with_box(tile, model, bounds);
            }
        }

        // Free tiles that are neither wanted nor standing in for one
        let keep: HashSet<Tile> = ideal.into_iter().chain(draw).collect();
        let unused: Vec<Tile> = self
            .tiles
            .keys()
            .copied()
            .filter(|t| !keep.contains(t))
            .collect();
        for tile in unused {
            if let Some(model) = self.tiles.remove(&tile) {
                renderer.unload_mesh(model.mesh);
                renderer.unload_texture(model.texture);
            }
        }
    }

    pub fn draw_shadows(&self, pass: &ShadowPass) {
        self.drawn.draw_shadows(pass);
    }

    pub fn draw(&self, renderer: &RenderContext, camera: DVec3) {
        let p = renderer.builtins.program_3d;

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
            let offset = anchor - camera;
            if offset.x.hypot(offset.z) > RUNWAY_DRAW_DISTANCE {
                continue;
            }
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

        // Draw terrain
        renderer.uniform(p, "u_detail").set_f32(1.0);
        self.drawn.draw(renderer);
        renderer.uniform(p, "u_detail").set_f32(0.0);
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
