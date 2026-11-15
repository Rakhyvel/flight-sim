use apricot::{
    app::{run, App, AppConfig, Scene},
    audio::SynthStream,
    camera::{Camera, ProjectionKind},
    error::Error,
    height_grid::HeightGrid,
    input::Scancode,
    mesh_data::MeshData,
    rectangle::Rectangle,
    render_core::{MeshId, RenderContext, TextureId},
    shadow_map::DirectionalLight,
};
use nalgebra_glm::{vec2, vec3, vec4, Mat4, Vec2};
use rand::{rngs::StdRng, RngCore, SeedableRng};

use crate::{
    airplane::Airplane,
    config::AirplaneConfig,
    control::Controls,
    engine_sound::EngineSound,
    hud::{GAP, MARGIN, PAD},
};

mod airplane;
mod config;
mod control;
mod engine_sound;
mod hud;
mod plane_model;

const CELLS: u32 = 500;
const HEIGHT: f32 = 40.0;
const SEA_LEVEL: f32 = 0.12;

// todo:
// - [x] good aero model
// - [x] joystick
// - [ ] better plane model
//     - [ ] hat to move view (or arrow keys)
// - [ ] better worldgen model (like 100km, real maps, hawaii, denver)
// - [ ] weather, wind, time of day
// - [ ] ground collision
// - [ ] flaps, gear, slats, aerobrakes, whatever
// - [ ] landing, airports
//     - [ ] probably requires RK4 for wheel springs
//     - [ ] ground effect
// - [ ] different planes (172, pa-28, cub, otter, 737)
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
    mesh: MeshId,
    texture: TextureId,

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

        let grid = island();
        let plane_model = plane_model::airplane_model();
        Self {
            light,
            mesh: renderer.add_mesh_data(terrain_mesh(&grid), None),
            texture: renderer.create_texture_rgba(CELLS, CELLS, &terrain_colors(&grid)),
            plane_mesh: renderer.add_mesh_data(plane_model.body, None),
            plane_trim_mesh: renderer.add_mesh_data(plane_model.trim, None),
            plane_prop_mesh: renderer.add_mesh_data(plane_model.prop, None),
            prop_angle: 0.0,
            prop_rps: 0.0,
            engine_rpm: 700.0,
            engine_sound: app.audio.start_synth(EngineSound::new()),
            plane: Airplane::new(AirplaneConfig::cessna172()),
            controls: Controls::new(),
        }
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
}

impl Scene for FlightSim {
    fn update(&mut self, app: &App) {
        if app.input.key_pressed(Scancode::Tab) {
            app.set_cursor_captured(!app.cursor_captured());
        }

        self.controls.update_input(&app.input, app.dt.as_secs_f32());
        self.plane.update(&self.controls, app.dt.as_secs_f32());

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

        // Blend forward vector with velocity vector to give that "heavy physics" feel
        // (0.8 facing, 0.2 actual momentum vector)
        let chase_dir = forward * 0.8 + vel_dir * 0.2;

        // Position camera behind and slightly above the blended vector
        let cam_pos = self.plane.pos - chase_dir * 25.0 + up * 6.0;

        let camera = Camera::new(
            cam_pos,
            self.plane.pos + forward * 10.0, // Look ahead of the plane
            up,
            ProjectionKind::default(),
        );

        app.renderer.set_camera(camera);
    }

    fn render(&mut self, app: &App) {
        let renderer = &app.renderer;
        let half = CELLS as f32 / 2.0;
        let terrain = nalgebra_glm::scale(
            &nalgebra_glm::translation(&vec3(-half, 0.0, -half)),
            &vec3(10.0, 5.0, 10.0),
        );
        let sea: Mat4 = nalgebra_glm::scale(
            &nalgebra_glm::translation(&vec3(0.0, SEA_LEVEL * HEIGHT, 0.0)),
            &vec3(CELLS as f32 * 3.0, 1.0, CELLS as f32 * 3.0),
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
            pass.draw(self.mesh, terrain)
        });

        renderer.clear(vec4(0.6, 0.8, 0.95, 1.0));
        renderer.draw_mesh_textured(self.mesh, self.texture, terrain, vec4(1.0, 1.0, 1.0, 1.0));
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

/// Noise that fades into the sea toward the edges, stretched to fill 0 to 1
fn island() -> HeightGrid {
    let width = CELLS as usize + 1;
    let mut rng = StdRng::from_entropy();
    let mut grid =
        HeightGrid::from_noise(width, 0.02, 6, rng.next_u32() as i32, 1.0, vec2(0.0, 0.0));
    grid.normalize();

    let center = CELLS as f32 / 2.0;
    for z in 0..width {
        for x in 0..width {
            let p = vec2(x as f32, z as f32);
            let d = nalgebra_glm::distance(&p, &vec2(center, center)) / center;
            let falloff = 1.0 - smoothstep(0.15, 1.0, d);
            grid.set_height(p, grid.height(p) * falloff);
        }
    }
    grid.normalize();
    grid
}

fn smoothstep(edge0: f32, edge1: f32, x: f32) -> f32 {
    let t = ((x - edge0) / (edge1 - edge0)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

fn terrain_mesh(grid: &HeightGrid) -> MeshData {
    MeshData::heightmap(CELLS, CELLS, |x, z| grid.height(vec2(x, z)) * HEIGHT)
}

/// One color per cell by height, turning blue where lots of water has run
fn terrain_colors(grid: &HeightGrid) -> Vec<u8> {
    let cells: Vec<Vec2> = (0..CELLS * CELLS)
        .map(|i| vec2((i % CELLS) as f32, (i / CELLS) as f32))
        .collect();
    // Flow piles up fast along rivers, so compare it on a log scale
    let max_flow = cells.iter().map(|p| grid.flow(*p)).fold(0.0, f32::max);
    let river_scale = (1.0 + max_flow).ln().max(1e-6);

    cells
        .iter()
        .flat_map(|p| {
            let h = grid.height(*p);
            let land = match h {
                h if h < SEA_LEVEL + 0.03 => [220.0, 205.0, 150.0],
                h if h < 0.45 => [90.0, 160.0, 70.0],
                h if h < 0.7 => [120.0, 115.0, 110.0],
                _ => [240.0, 240.0, 245.0],
            };
            let river = smoothstep(0.4, 0.8, (1.0 + grid.flow(*p)).ln() / river_scale);
            let water = [50.0, 110.0, 200.0];
            let mix = |i: usize| (land[i] + (water[i] - land[i]) * river) as u8;
            [mix(0), mix(1), mix(2), 255]
        })
        .collect()
}
