use std::f32::consts::FRAC_PI_4;

use apricot::{
    app::{run, App, AppConfig, Scene, SceneCommand},
    audio::SynthStream,
    camera::{Camera, ProjectionKind},
    error::Error,
    input::Scancode,
    opengl::DrawOptions,
    render::{
        assets::{MeshId, ProgramId},
        lighting::DirectionalLight,
    },
};
use nalgebra_glm::{vec2, vec3, vec4, Mat4};

use crate::{
    airplane::Airplane, config::AirplaneConfig, control::Controls, engine_sound::EngineSound,
    flight_data::FlightData, hud::Hud, minimap::Minimap, region::load_regions, waypoint::Route,
    world::World,
};

mod airplane;
mod airport;
mod config;
mod control;
mod engine_sound;
mod flight_data;
mod heightmap;
mod hud;
mod minimap;
mod plane_model;
mod region;
mod runway_model;
mod terrain_color;
mod waypoint;
mod world;

const CAMERA_FAR: f32 = 60_000.0;
const LOADS_PER_FRAME: usize = 2;

// todo:
// - [x] good aero model
// - [x] joystick
// - [x] better plane model
//     - [ ] c to free mouse orbit
//     - [ ] z to zoom in
// - [ ] better worldgen model (like 100km, real maps, hawaii, denver)
//     - [x] mini map
// - [ ] weather, wind, time of day
//     - [x] distance haze, fog
//     - [ ] Higher LOD shadow maps for the airplane, maybe cascading shadow maps
//     - [x] atmospheric effects (golden hour, overcast skies, sunrise, sunset)
//     - [ ] rain, wind, upwelling from mountains, actual raindrops when raining?
//     - [ ] volumetric clouds that move with the wind and exist where they would exist
//     - [x] water should look like water (reflection, refraction, waves, etc)
//     - [x] terrain textures, not just a solid color (sat imagery?)
//     - [ ] vegetation scatter based on the actual biomes designated by terrain_color (move biomes out to its own thing, so that vegetation and color read from it?)
//     - [ ] prop blur
//     - [ ] control surface deflection? wing flex?
// - [ ] camera jitter and g-force lag
// - [x] water/ground collision
// - [ ] flaps, gear, slats, aerobrakes, whatever
//     - [ ] stall horn for the cessna
//     - [ ] g limits
// - [ ] landing, airports
//     - [ ] probably requires RK4 for wheel springs
//     - [ ] ground effect
//     - [ ] gotta make sure the camera doesn't clip into the ground
// - [ ] different planes (172, pa-28, cub, otter, citation, 737-200)
//      - [ ] do actual engine/prop models, move rpm calc from front-end to engine backend
// - [ ] wet mass, cargo mass
// - [ ] waypoints, IFR, ATC call outs? flight plans?
// - [ ] contracts (cargo, passengers, sightseeing, firefighting, bush rescue, skydiving, banner dragging)
// - [ ] cockpit camera? idrc about this ngl

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

    water_program: ProgramId,

    world: World,

    hud: Hud,
    minimap: Minimap,

    plane_mesh: MeshId,
    plane_trim_mesh: MeshId,
    plane_prop_mesh: MeshId,
    prop_angle: f64, // [rad]
    prop_rps: f64,   // displayed revolutions per second

    /// How far behind the plane the camera sits
    cam_distance: f64,
    fov: f32,

    engine_rpm: f64,
    engine_sound: Option<SynthStream<EngineSound>>,

    plane: Airplane,
    controls: Controls,

    // TODO: some kinda mission struct
    route: Route,
}

macro_rules! frag {
    ($($file:literal),+ $(,)?) => {
        concat!("#version 330 core\n", $(include_str!($file), "\n"),+)
    };
}

impl FlightSim {
    fn new(app: &App) -> Result<Self, Error> {
        let renderer = &app.renderer;

        // Replace the default 3d shader with our super cool epic shader
        renderer.add_program_from_source(
            include_str!("shaders/3d.vert"),
            frag!(
                "shaders/lighting.glsl",
                "shaders/atmosphere.glsl",
                "shaders/noise.glsl",
                "shaders/3d.frag",
            ),
            Some("3d"),
        )?;
        renderer.add_program_from_source(
            include_str!("shaders/sky.vert"),
            frag!("shaders/atmosphere.glsl", "shaders/sky.frag",),
            Some("sky"),
        )?;
        let water_program = renderer.add_program_from_source(
            include_str!("shaders/3d.vert"),
            frag!(
                "shaders/atmosphere.glsl",
                "shaders/lighting.glsl",
                "shaders/noise.glsl",
                "shaders/water.frag"
            ),
            Some("water"),
        )?;

        let light_dir = vec3(0.5, 0.8, 0.0).normalize();
        let mut light = DirectionalLight::new(light_dir, 2048)?;
        light.frustum.shadow_distance = 150.0;
        app.renderer.set_light(Some(&light));

        let regions = load_regions("assets/regions.toml");
        let region = regions
            .iter()
            .find(|r| r.name == "Tucson")
            .expect("no region");
        let world = World::load(renderer, region).expect("couldnt load region");

        let plane_model = plane_model::airplane_model();
        let mut sim = Self {
            light,
            plane_mesh: renderer.add_mesh_data(plane_model.body, None),
            plane_trim_mesh: renderer.add_mesh_data(plane_model.trim, None),
            plane_prop_mesh: renderer.add_mesh_data(plane_model.prop, None),
            prop_angle: 0.0,
            prop_rps: 0.0,
            cam_distance: 25.0,
            fov: FRAC_PI_4,
            engine_rpm: 700.0,
            engine_sound: app.audio.start_synth(EngineSound::new()),
            world,
            hud: Hud::new(&app.renderer)?,
            minimap: Minimap::new(&app.renderer, light_dir)?,
            water_program,
            plane: Airplane::new(
                AirplaneConfig::load("assets/planes/dash8.toml").expect("should exist"),
            ),
            controls: Controls::new(),
            route: Route::new(vec![]),
        };

        let mut waypoints = vec![];
        let depart_runway = sim.world.runway("KTUS 04").expect("no runway");
        let arrive_runway = sim.world.runway("KRYN 06L").expect("no runway");
        waypoints.extend(sim.world.approach(arrive_runway));
        sim.route = Route::new(waypoints);

        let (spot, heading) = sim.world.spawn_point(depart_runway);
        sim.plane.place(spot + vec3(0.0, 1.5, 0.0), heading);
        sim.world.recenter(sim.plane.pos);
        sim.world.update(renderer, sim.plane.pos, usize::MAX);

        Ok(sim)
    }
}

impl Scene for FlightSim {
    fn update(&mut self, app: &App) -> SceneCommand {
        if app.input.key_pressed(Scancode::Tab) {
            app.set_cursor_captured(!app.cursor_captured());
        }

        self.controls.update_input(&app.input, app.dt.as_secs_f32());
        self.plane
            .update(&self.controls, app.dt.as_secs_f64(), &|x, z| {
                self.world.ground(x, z)
            });
        self.world.recenter(self.plane.pos);
        self.world
            .update(&app.renderer, self.plane.pos, LOADS_PER_FRAME);
        self.route.update(self.plane.pos);

        let dt = app.dt.as_secs_f64();
        // For fixed speed props
        // TODO: Configurable?!
        let target_rpm =
            (700.0 + self.controls.throttle as f64 * 1600.0 + self.plane.vel.norm() * 6.0)
                .min(2800.0);
        self.engine_rpm += (target_rpm - self.engine_rpm) * (1.5 * dt).min(1.0);

        let (rpm, throttle, airspeed) = (
            self.engine_rpm,
            self.controls.throttle,
            self.plane.vel.norm(),
        );
        if let Some(sound) = &mut self.engine_sound {
            sound.with(|e| {
                e.rpm = rpm as f32;
                e.throttle = throttle as f32;
                e.airspeed = airspeed as f32;
            });
        }

        // Engine spools up and down rather than snapping
        // TODO: Take this from the engine model, don't do this on the front end!
        self.prop_rps = self.engine_rpm / 2700.0 * 7.5;
        self.prop_angle =
            (self.prop_angle + self.prop_rps * std::f64::consts::TAU * dt) % std::f64::consts::TAU;

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
        let max_pan = std::f64::consts::PI * 0.5; // TODO: Could be a setting?
        let view_dir =
            nalgebra_glm::rotate_vec3(&chase_dir, -self.controls.pan as f64 * max_pan, &up);

        // Position camera behind and slightly above the blended vector
        let plane_pos = self.plane.pos - self.world.origin.pos;
        self.cam_distance =
            (self.cam_distance * 0.9_f64.powf(app.input.mouse_wheel as f64)).clamp(0.8, 400.0);
        let target_fov = if app.input.key_down(Scancode::Z) {
            12.0_f32.to_radians()
        } else {
            FRAC_PI_4
        };
        self.fov += (target_fov - self.fov) * (8.0 * dt as f32).min(1.0);
        let cam_pos = plane_pos - view_dir * self.cam_distance + up * (self.cam_distance * 0.24);

        let camera = Camera::new(
            cam_pos.cast(),
            (plane_pos + view_dir * 10.0).cast(), // Look ahead of the plane
            up.cast(),
            ProjectionKind::Perspective {
                fov_rad: self.fov,
                near: 3.0,
                far: CAMERA_FAR,
            },
        );

        app.renderer.set_camera(camera);
        SceneCommand::None
    }

    fn render(&mut self, app: &App) {
        let renderer = &app.renderer;

        let sea_size = 2.0 * CAMERA_FAR;
        let plane_pos = self.world.origin.relative(self.plane.pos);
        let sea: Mat4 = nalgebra_glm::scale(
            &nalgebra_glm::translation(&vec3(plane_pos.x, 0.0, plane_pos.z)),
            &vec3(sea_size, 1.0, sea_size),
        );

        let origin = vec2(
            self.world.origin.pos.x as f32,
            self.world.origin.pos.z as f32,
        );
        for program in [renderer.builtins.program_3d, self.water_program] {
            renderer.uniform(program, "u_origin").set_vec2(&origin);
        }

        self.minimap.render(
            renderer,
            sea,
            &self.plane,
            &self.world.map,
            &self.world.chunks,
            &self.light,
            &self.world.origin,
        );

        let plane_model = self.plane.model_matrix(&self.world.origin);

        let prop_model = nalgebra_glm::rotate_x(
            &nalgebra_glm::translate(&plane_model, &plane_model::PROP_HUB),
            self.prop_angle as f32,
        );

        renderer.shadow_pass(&mut self.light, |pass| {
            pass.draw(self.plane_mesh, plane_model);
            pass.draw(self.plane_trim_mesh, plane_model);
            pass.draw(self.plane_prop_mesh, prop_model);
            self.world.draw_shadows(pass);
        });

        renderer.clear(vec4(0.6, 0.8, 0.95, 1.0));
        renderer.draw_sky(self.light.direction());

        self.world.draw(renderer);

        // Draw the water
        renderer
            .uniform(self.water_program, "u_time")
            .set_f32((app.seconds % 1000.0) as f32);
        renderer.draw_mesh_with_program(
            self.water_program,
            renderer.builtins.plane,
            sea,
            &DrawOptions::default(),
        );

        renderer.draw_mesh(self.plane_mesh, plane_model, vec4(0.9, 0.9, 0.9, 1.0));
        renderer.draw_mesh(
            self.plane_trim_mesh,
            plane_model,
            vec4(0.2, 0.22, 0.25, 1.0),
        );
        renderer.draw_mesh(self.plane_prop_mesh, prop_model, vec4(0.2, 0.22, 0.25, 1.0));

        let data = FlightData::gather(&self.plane, &self.controls, &self.world, &self.route);
        self.hud.draw(renderer, &data, &self.minimap, app.fps);
    }
}
