use apricot::{
    app::{run, App, AppConfig, Scene},
    camera::{Camera, ProjectionKind},
    error::Error,
    height_grid::HeightGrid,
    input::Scancode,
    mesh_data::MeshData,
    render_core::{MeshId, TextureId},
    shadow_map::DirectionalLight,
};
use nalgebra_glm::{vec2, vec3, vec4, Mat4, Vec2};
use rand::{rngs::StdRng, RngCore, SeedableRng};

use crate::{airplane::Airplane, config::AirplaneConfig, control::Controls};

mod airplane;
mod config;
mod control;

const CELLS: u32 = 500;
const HEIGHT: f32 = 30.0;
const SEA_LEVEL: f32 = 0.12;

// todo:
// - [x] good aero model
// - [ ] joystick
// - [ ] better plane model
// - [ ] better worldgen model (like 100km, real maps, hawaii, denver)
// - [ ] weather, wind, time of day
// - [ ] ground collision
// - [ ] flaps, gear, slats, aerobrakes, whatever
// - [ ] landing, airports
//     - [ ] probably requires RK4 for wheel springs
//     - [ ] ground effect
// - [ ] different planes (172, pa-28, cub, otter, 737)
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
        Self {
            light,
            mesh: renderer.add_mesh_data(terrain_mesh(&grid), None),
            texture: renderer.create_texture_rgba(CELLS, CELLS, &terrain_colors(&grid)),
            plane_mesh: app.renderer.builtins.cube,
            plane: Airplane::new(AirplaneConfig::cessna172()),
            controls: Controls::new(),
        }
    }
}

impl Scene for FlightSim {
    fn update(&mut self, app: &App) {
        if app.input.key_pressed(Scancode::Tab) {
            app.set_cursor_captured(!app.cursor_captured());
        }

        self.controls.update_input(&app.input, app.dt.as_secs_f32());
        self.plane.update(&self.controls, app.dt.as_secs_f32());

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
        let cam_pos = self.plane.pos - chase_dir * 12.0 + up * 4.0;

        let camera = Camera::new(
            cam_pos,
            self.plane.pos + forward * 5.0, // Look ahead of the plane
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

        renderer.shadow_pass(&mut self.light, |pass| {
            pass.draw(self.plane_mesh, plane_model);
            pass.draw(self.mesh, terrain)
        });

        renderer.clear(vec4(0.6, 0.8, 0.95, 1.0));
        renderer.draw_mesh_textured(self.mesh, self.texture, terrain, vec4(1.0, 1.0, 1.0, 1.0));
        // See-through, so drawn after the land
        renderer.draw_mesh(renderer.builtins.plane, sea, vec4(0.15, 0.4, 0.75, 0.7));

        renderer.draw_mesh(self.plane_mesh, plane_model, vec4(0.9, 0.9, 0.9, 1.0));

        let alt = self.plane.pos.y;
        let spd = self.plane.vel.norm();
        renderer.draw_text(
            vec2(10.0, 10.0),
            &format!("{} FPS\nHeight: {alt:.0} m\nSpeed: {spd:.0} m/s", app.fps),
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
