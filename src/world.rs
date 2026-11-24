use std::{
    collections::{HashMap, HashSet},
    error::Error,
    f64::consts::TAU,
};

use apricot::{
    camera::floating_origin::FloatingOrigin,
    geometry::aabb::AABB,
    model_set::{Model, ModelSet},
    opengl::DrawOptions,
    render::{
        assets::{MeshId, ProgramId, TextureId},
        lighting::ShadowPass,
        RenderContext,
    },
};
use nalgebra_glm::{vec2, vec3, vec4, DVec2, DVec3, Mat4, Vec3, Vec4};

use crate::{
    airport::{self, Airport, Navaid, Runway},
    heightmap::Heightmap,
    region::Region,
    runway_model, terrain_color,
    tiles::{Tile, TileHeights, TileTree, TILE_QUADS},
    units::{M_PER_FT, M_PER_NM},
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

/// Texture unit terrain reads its normals from
const NORMAL_UNIT: u32 = 5;

/// A tile's ground, ready to draw
#[derive(Clone, Copy)]
struct BuiltTile {
    model: Model,
    /// Which way the ground faces, finer than the mesh
    normals: TextureId,
}

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
    tiles: HashMap<Tile, BuiltTile>,
    /// The tiles being drawn this frame
    drawn: ModelSet<Tile>,

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

    fn build_tile(&self, renderer: &RenderContext, tile: Tile) -> BuiltTile {
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

        // Shape
        let (n, step) = (tile.cells(), tile.step());
        let mesh = apricot::terrain::mesh_smooth(n, n, step, SKIRT_PER_STEP * step as f32, height);
        let mesh = renderer.add_mesh_data(mesh, None);

        // Color a texel on every vertex on both edges
        let pixels = terrain_color::tile_texture(map, ox, oz, step, TILE_QUADS + 1);
        let texture = renderer.create_texture_rgba(TILE_QUADS + 1, TILE_QUADS + 1, &pixels);
        renderer.get_texture_from_id(texture).set_smooth(true);

        // Normals
        let normal_step = (step / 2).max(1); // level 0 is already every cell
        let normal_n = n / normal_step + 1; // twice as fine as the mesh so the lighting looks good
        let spacing = (map.spacing_x, map.spacing_z);
        let pixels = tile_normals(&height, spacing, normal_step, normal_n);
        let normals = renderer.create_texture_rgba(normal_n, normal_n, &pixels);
        renderer.get_texture_from_id(normals).set_smooth(true);

        BuiltTile {
            model: Model::new(mesh, texture, self.tile_transform(tile)),
            normals,
        }
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
            let built_tile = self.build_tile(renderer, tile);
            self.tiles.insert(tile, built_tile);
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
                let mut model = self.tiles[&tile].model;
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
            if let Some(built_tile) = self.tiles.remove(&tile) {
                renderer.unload_mesh(built_tile.model.mesh);
                renderer.unload_texture(built_tile.model.texture);
                renderer.unload_texture(built_tile.normals);
            }
        }
    }

    pub fn draw_shadows(&self, pass: &ShadowPass) {
        self.drawn.draw_shadows(pass);
    }

    pub fn draw(&self, renderer: &RenderContext, camera: DVec3) {
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

        self.draw_terrain(renderer, renderer.builtins.program_3d);
    }

    /// Draw the ground the current camera can see with `program`
    pub fn draw_terrain(&self, renderer: &RenderContext, program: ProgramId) {
        // Set the tile uniforms
        renderer.uniform(program, "u_texture").set_i32(0);
        renderer
            .uniform(program, "u_normal_map")
            .set_i32(NORMAL_UNIT as i32);
        renderer
            .uniform(program, "u_color")
            .set_vec4(&Vec4::repeat(1.0));
        renderer.uniform(program, "u_detail").set_f32(1.0); // programs without this just ignore it

        // Draw each tile
        let frustum = renderer.camera_frustum();
        for (tile, model) in self.drawn.iter_frustum(&frustum) {
            renderer.bind_texture_unit(0, model.texture);
            renderer.bind_texture_unit(NORMAL_UNIT, self.tiles[&tile].normals);
            renderer.draw_mesh_with_program(
                program,
                model.mesh,
                model.transform,
                &DrawOptions::default(),
            );
        }

        // cleanup after yourself!
        renderer.uniform(program, "u_detail").set_f32(0.0);
    }

    pub fn waypoint(&self, name: &str) -> Option<&Waypoint> {
        self.waypoints.iter().find(|w| w.name == name)
    }

    /// Where the runway lies on the map
    pub fn strip(&self, runway: &Runway) -> Strip {
        let a = self.map.geo.to_world(runway.start.0, runway.start.1);
        let b = self.map.geo.to_world(runway.end.0, runway.end.1);
        Strip {
            threshold: a,
            dir: (b - a).normalize(),
            length: (b - a).norm(),
            width: runway.width as f64,
        }
    }

    /// Every runway on the map by name, laid out
    pub fn strips(&self) -> Vec<(String, Strip)> {
        self.runways
            .iter()
            .map(|r| (r.name.clone(), self.strip(r)))
            .collect()
    }

    // TODO: Make sure approaches stay 2000' above any terrain
    /// Generate a straight-in approach to a runway
    pub fn approach(&self, runway: &Runway) -> Vec<Waypoint> {
        const INITIAL_FIX_OUT: f64 = 10.0 * M_PER_NM;
        const INITIAL_FIX_HEIGHT: f64 = 3000.0 * M_PER_FT;
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
            fix("THR", 0.0, THRESHOLD_CROSSING_HEIGHT),
        ]
    }
}

/// A runway laid out on the map
#[derive(Clone, Copy)]
pub struct Strip {
    /// Middle of the end you land from, world (x, z) [m]
    pub threshold: DVec2,
    /// Unit vector down the runway, world (x, z)
    pub dir: DVec2,
    pub length: f64,
    pub width: f64,
}

impl Strip {
    /// World (x, z) `p` as (how far past the threshold, how far right of the centerline) [m]
    pub fn local(&self, p: DVec2) -> DVec2 {
        let d = p - self.threshold;
        let right = vec2(-self.dir.y, self.dir.x); // east when runway points north
        vec2(d.dot(&self.dir), d.dot(&right))
    }

    /// Whether world (x, z) `p` is on the pavement
    pub fn contains(&self, p: DVec2) -> bool {
        let l = self.local(p);
        (0.0..=self.length).contains(&l.x) && l.y.abs() <= self.width / 2.0
    }

    /// The compass heading of the runway [rad]
    pub fn heading(&self) -> f64 {
        self.dir.x.atan2(-self.dir.y).rem_euclid(TAU)
    }
}

/// RGBA pixels of the ground's normal, `n` x `n` texels one every `step` cells, from `height` at cells relative to the tile's corner.
/// Red and green are x, blue and alpha are z, each 16 bits from -1 to 1, high byte first
fn tile_normals(
    height: &impl Fn(f32, f32) -> f32,
    spacing: (f32, f32),
    step: u32,
    n: u32,
) -> Vec<u8> {
    let s = step as f32;
    let mut pixels = Vec::with_capacity((n * n * 4) as usize);
    for j in 0..n {
        for i in 0..n {
            let (x, z) = (i as f32 * s, j as f32 * s);

            // Rise per meter, one texel each way
            let dx = (height(x + s, z) - height(x - s, z)) / (2.0 * s * spacing.0);
            let dz = (height(x, z + s) - height(x, z - s)) / (2.0 * s * spacing.1);
            let normal = vec3(-dx, 1.0, -dz).normalize();

            let encode = |v: f32| (((v * 0.5 + 0.5) * 65535.0).round() as u16).to_be_bytes(); // 8 bit bands gentle slopes at low sun
            let ([x_hi, x_lo], [z_hi, z_lo]) = (encode(normal.x), encode(normal.z));
            pixels.extend_from_slice(&[x_hi, x_lo, z_hi, z_lo]);
        }
    }
    pixels
}
