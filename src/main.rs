use apricot::{
    app::{run, App, AppConfig, Scene, SceneCommand},
    audio::SynthStream,
    camera::{Camera, ProjectionKind},
    error::Error,
    geometry::rectangle::Rectangle,
    input::Scancode,
    model_set::{Model, ModelSet},
    render_core::{MeshId, RenderContext},
    shadow_map::DirectionalLight,
    terrain::chunks::{ChunkCoord, ChunkGrid},
};
use nalgebra_glm::{vec2, vec3, vec4, Mat4};

use crate::{
    airplane::Airplane,
    config::AirplaneConfig,
    control::Controls,
    engine_sound::EngineSound,
    heightmap::Heightmap,
    hud::{GAP, MARGIN, PAD},
};

mod airplane;
mod config;
mod control;
mod engine_sound;
mod heightmap;
mod hud;
mod plane_model;
mod terrain_color;

const CAMERA_FAR: f32 = 50_000.0;
const CHUNK_CELLS: u32 = 64; // 64 × 30 m ≈ 2 km per chunk
const LOAD_RADIUS: i32 = 25; // chunks
const LOADS_PER_FRAME: usize = 2;

// todo:
// - [x] good aero model
// - [x] joystick
// - [x] better plane model
//     - [x] hat to move view (or arrow keys)
// - [ ] better worldgen model (like 100km, real maps, hawaii, denver)
//     - [ ] mini map
// - [ ] weather, wind, time of day
// - [ ] ground collision
// - [ ] flaps, gear, slats, aerobrakes, whatever
//     - [ ] stall horn for the cessna
// - [ ] landing, airports
//     - [ ] probably requires RK4 for wheel springs
//     - [ ] ground effect
// - [ ] different planes (172, pa-28, cub, otter, citation, 737-200)
//      - [ ] do actual engine/prop models, move rpm calc from front-end to engine backend
// - [ ] wet mass, cargo mass
// - [ ] waypoints, IFR, ATC call outs? flight plans?
// - [ ] contracts (cargo, passengers, sightseeing, firefighting, bush rescue, skydiving, banner dragging)

fn main() -> Result<(), Error> {
    run(
        AppConfig {
            title: "Flight Sim",
            ..Default::default()
        },
        FlightSim::new,
    )
}

struct FlightSim {
    light: DirectionalLight,

    map: Heightmap,
    chunk_grid: ChunkGrid,
    chunks: ModelSet<ChunkCoord>,

    plane_mesh: MeshId,
    plane_trim_mesh: MeshId,
    plane_prop_mesh: MeshId,
    prop_angle: f32, // [rad]
    prop_rps: f32,   // displayed revolutions per second

    engine_rpm: f32,
    engine_sound: Option<SynthStream<EngineSound>>,

    plane: Airplane,
    controls: Controls,
}

impl FlightSim {
    fn new(app: &App) -> Self {
        let renderer = &app.renderer;

        let mut light = DirectionalLight::new(vec3(0.6, 10.0, 0.3), 2048).unwrap();
        light.shadow_distance = 150.0;
        app.renderer.set_light(Some(&light));

        let plane_model = plane_model::airplane_model();
        let mut sim = Self {
            light,
            plane_mesh: renderer.add_mesh_data(plane_model.body, None),
            plane_trim_mesh: renderer.add_mesh_data(plane_model.trim, None),
            plane_prop_mesh: renderer.add_mesh_data(plane_model.prop, None),
            prop_angle: 0.0,
            prop_rps: 0.0,
            engine_rpm: 700.0,
            engine_sound: app.audio.start_synth(EngineSound::new()),
            plane: Airplane::new(AirplaneConfig::cessna172()),
            controls: Controls::new(),
            map: Heightmap::load_geotiff("assets/denver.tif").expect("couldn't load heightmap"),
            chunk_grid: ChunkGrid::new(CHUNK_CELLS),
            chunks: ModelSet::new(),
        };
        sim.update_chunks(&app.renderer, usize::MAX); // load everything in range now
        let (w, d) = sim.map.size();
        sim.plane.pos = vec3(w as f32 * 0.5 * 30.0, 1700.0, d as f32 * 0.5 * 30.0);
        sim
    }

    fn draw_hud(&self, app: &App) {
        let renderer = &app.renderer;
        let screen = renderer.screen_size();
        let (sw, sh) = (screen.x as f32, screen.y as f32);

        self.draw_flight_panel(renderer, sw, sh);
        self.draw_engine_panel(renderer, sh);
    }

    /// Draw attitude indicator stacked on the compass
    fn draw_flight_panel(&self, renderer: &RenderContext, sw: f32, sh: f32) {
        let r = 75.0;
        let compass_h = 26.0;

        let plate_w = PAD + 2.0 * r + PAD;
        let plate_h = PAD + 2.0 * r + GAP + compass_h + PAD;
        let plate = Rectangle::new(
            sw - MARGIN - plate_w,
            sh - MARGIN - plate_h,
            plate_w,
            plate_h,
        );
        hud::draw_plate(renderer, plate);

        let ai_center = plate.pos + vec2(PAD + r, PAD + r);
        hud::draw_attitude_indicator(
            renderer,
            ai_center,
            r,
            self.plane.pitch(),
            self.plane.roll(),
        );

        let compass = Rectangle::new(
            plate.pos.x + PAD,
            plate.pos.y + PAD + 2.0 * r + GAP,
            2.0 * r,
            compass_h,
        );
        hud::draw_compass(renderer, compass, self.plane.heading().to_degrees());
    }

    /// Draw altitude, speed and throttle
    fn draw_engine_panel(&self, renderer: &RenderContext, sh: f32) {
        let row_h = 26.0;
        let readout_w = 84.0;
        let bar_w = 12.0;
        let rows = 3.0;

        let inner_h = rows * row_h + (rows - 1.0) * GAP;
        let plate_w = PAD + readout_w + GAP + bar_w + PAD;
        let plate_h = PAD + inner_h + PAD;
        let plate = Rectangle::new(MARGIN, sh - MARGIN - plate_h, plate_w, plate_h);
        hud::draw_plate(renderer, plate);

        let row = |i: f32| {
            Rectangle::new(
                plate.pos.x + PAD,
                plate.pos.y + PAD + i * (row_h + GAP),
                readout_w,
                row_h,
            )
        };
        hud::draw_readout(
            renderer,
            row(0.0),
            "ALT",
            &format!("{:.0} m", self.plane.pos.y),
        );
        hud::draw_readout(
            renderer,
            row(1.0),
            "SPD",
            &format!("{:.0} m/s", self.plane.vel.norm()),
        );
        hud::draw_readout(
            renderer,
            row(2.0),
            "THR",
            &format!("{:.0}%", self.controls.throttle * 100.0),
        );

        let bar = Rectangle::new(
            plate.pos.x + PAD + readout_w + GAP,
            plate.pos.y + PAD,
            bar_w,
            inner_h,
        );
        hud::draw_vertical_bar(renderer, bar, self.controls.throttle);
    }

    fn update_chunks(&mut self, renderer: &RenderContext, max_loads: usize) {
        let (sx, sz) = (self.map.spacing_x, self.map.spacing_z);
        let cell_pos = vec3(self.plane.pos.x / sx, 0.0, self.plane.pos.z / sz);

        let (w, d) = self.map.size();
        let max_cx = (w as u32).div_ceil(CHUNK_CELLS) as i32;
        let max_cz = (d as u32).div_ceil(CHUNK_CELLS) as i32;

        // Closest first, and only chunks that are actually on the map
        let wanted: Vec<ChunkCoord> = self
            .chunk_grid
            .chunks_near(cell_pos, LOAD_RADIUS)
            .into_iter()
            .filter(|&(cx, cz)| cx >= 0 && cz >= 0 && cx < max_cx && cz < max_cz)
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
                .mesh(chunk, |x, z| map.sample(x as i32, z as i32));
            let mesh_id = renderer.add_mesh_data(mesh, None);

            let o = self.chunk_grid.origin(chunk);
            let origin = vec3(o.x * sx, 0.0, o.z * sz);
            let transform =
                nalgebra_glm::scale(&nalgebra_glm::translation(&origin), &vec3(sx, 1.0, sz));

            let pixels = terrain_color::chunk_texture(map, &self.chunk_grid, chunk);
            let texture = renderer.create_texture_rgba(CHUNK_CELLS, CHUNK_CELLS, &pixels);

            self.chunks
                .insert(renderer, chunk, Model::new(mesh_id, texture, transform));
        }
    }
}

impl Scene for FlightSim {
    fn update(&mut self, app: &App) -> SceneCommand {
        if app.input.key_pressed(Scancode::Tab) {
            app.set_cursor_captured(!app.cursor_captured());
        }

        self.controls.update_input(&app.input, app.dt.as_secs_f32());
        self.plane.update(&self.controls, app.dt.as_secs_f32());

        self.update_chunks(&app.renderer, LOADS_PER_FRAME);

        let dt = app.dt.as_secs_f32();
        // For fixed speed props
        // TODO: Configurable?!
        let target_rpm =
            (700.0 + self.controls.throttle * 1600.0 + self.plane.vel.norm() * 6.0).min(2800.0);
        self.engine_rpm += (target_rpm - self.engine_rpm) * (1.5 * dt).min(1.0);

        let (rpm, throttle, airspeed) = (
            self.engine_rpm,
            self.controls.throttle,
            self.plane.vel.norm(),
        );
        if let Some(sound) = &mut self.engine_sound {
            sound.with(|e| {
                e.rpm = rpm;
                e.throttle = throttle;
                e.airspeed = airspeed;
            });
        }

        // Engine spools up and down rather than snapping
        // TODO: Take this from the engine model, don't do this on the front end!
        self.prop_rps = self.engine_rpm / 2700.0 * 7.5;
        self.prop_angle =
            (self.prop_angle + self.prop_rps * std::f32::consts::TAU * dt) % std::f32::consts::TAU;

        let forward = nalgebra_glm::quat_rotate_vec3(&self.plane.orientation, &vec3(1.0, 0.0, 0.0));
        let up = nalgebra_glm::quat_rotate_vec3(&self.plane.orientation, &vec3(0.0, 1.0, 0.0));

        // Extract normalized velocity direction to feel sideways/vertical drift (AoA/sideslip)
        let speed = self.plane.vel.norm();
        let vel_dir = if speed > 0.1 {
            self.plane.vel / speed
        } else {
            forward
        };

        // Blend forward vector with velocity vector to give that in flight feel
        let chase_dir = (forward * 0.8 + vel_dir * 0.2).normalize();

        // Orbit around the plane
        let max_pan = std::f32::consts::PI * 0.5; // TODO: Could be a setting?
        let view_dir = nalgebra_glm::rotate_vec3(&chase_dir, -self.controls.pan * max_pan, &up);

        // Position camera behind and slightly above the blended vector
        let cam_pos = self.plane.pos - view_dir * 25.0 + up * 6.0;

        let camera = Camera::new(
            cam_pos,
            self.plane.pos + view_dir * 10.0, // Look ahead of the plane
            up,
            ProjectionKind::Perspective {
                fov_rad: std::f32::consts::FRAC_PI_4,
                near: 1.0,
                far: CAMERA_FAR,
            },
        );

        app.renderer.set_camera(camera);
        SceneCommand::None
    }

    fn render(&mut self, app: &App) {
        let renderer = &app.renderer;
        let sea_size = 2.0 * CAMERA_FAR;
        let sea: Mat4 = nalgebra_glm::scale(
            &nalgebra_glm::translation(&vec3(self.plane.pos.x, 0.0, self.plane.pos.z)),
            &vec3(sea_size, 1.0, sea_size),
        );
        let plane_model = self.plane.model_matrix();

        let prop_model = nalgebra_glm::rotate_x(
            &nalgebra_glm::translate(&plane_model, &plane_model::PROP_HUB),
            self.prop_angle,
        );

        renderer.shadow_pass(&mut self.light, |pass| {
            pass.draw(self.plane_mesh, plane_model);
            pass.draw(self.plane_trim_mesh, plane_model);
            pass.draw(self.plane_prop_mesh, prop_model);
            self.chunks.draw_shadows(pass);
        });

        renderer.clear(vec4(0.6, 0.8, 0.95, 1.0));
        self.chunks.draw(renderer);
        // See-through, so drawn after the land
        renderer.draw_mesh(renderer.builtins.plane, sea, vec4(0.15, 0.4, 0.75, 0.7));

        renderer.draw_mesh(self.plane_mesh, plane_model, vec4(0.9, 0.9, 0.9, 1.0));
        renderer.draw_mesh(
            self.plane_trim_mesh,
            plane_model,
            vec4(0.2, 0.22, 0.25, 1.0),
        );
        renderer.draw_mesh(self.plane_prop_mesh, prop_model, vec4(0.2, 0.22, 0.25, 1.0));

        self.draw_hud(app);

        renderer.draw_text(
            vec2(10.0, 10.0),
            &format!("{} FPS", app.fps),
            vec4(0.1, 0.1, 0.1, 1.0),
        );
    }
}
