use apricot::{
    camera::{Camera, ProjectionKind},
    error::Error,
    render::{assets::ProgramId, target::RenderTarget, RenderContext},
};
use nalgebra_glm::{vec3, vec4, Vec3};

use crate::{airplane::Airplane, world::World};

/// The map doesn't change all that much, only redraw it every this many frames to reduce lag
const REDRAW_EVERY: u32 = 4;

pub const MINIMAP_PX: u32 = 512;
pub const MINIMAP_RANGE: f32 = 5_000.0; // meters from the plane to the edge of the map

pub struct Minimap {
    pub minimap: RenderTarget,
    /// Draws the ground and sea in flat map colors
    program: ProgramId,
    frame: u32,
}

impl Minimap {
    pub fn new(renderer: &RenderContext, light_dir: Vec3) -> Result<Self, Error> {
        let minimap = renderer.create_render_target(MINIMAP_PX)?;
        renderer
            .get_texture_from_id(minimap.texture)
            .set_smooth(true);

        let program = renderer.add_program_from_source(
            glsl!("shaders/map.vert"),
            glsl!("shaders/terrain.glsl", "shaders/map.frag"),
            None,
        )?;

        Ok(Self {
            minimap,
            program,
            frame: 0,
        })
    }

    /// Draw the map around `plane`, turned so it poitns up, with the sea placed by `sea`
    pub fn render(&mut self, renderer: &RenderContext, plane: &Airplane, world: &World) {
        // Only draw every couple frames or so to reduce lag
        let due = self.frame % REDRAW_EVERY == 0;
        self.frame = self.frame.wrapping_add(1);
        if !due {
            return;
        }

        // Look straight down, turned with the plane
        let h = plane.heading();
        let pos = world.origin.relative(plane.pos);
        let forward = vec3(h.sin(), 0.0, -h.cos()).cast();
        let right = vec3(h.cos(), 0.0, h.sin()).cast(); // east when heading north

        let top = world.map.max_height + 100.0;
        let eye = vec3(pos.x, top, pos.z);
        let camera = Camera::new(
            eye,
            eye - vec3(0.0, 1.0, 0.0),
            forward,
            ProjectionKind::Orthographic {
                left: -MINIMAP_RANGE,
                right: MINIMAP_RANGE,
                bottom: -MINIMAP_RANGE,
                top: MINIMAP_RANGE,
                near: 0.0,
                far: top + 50.0,
            },
        );

        let main_camera = renderer.camera();
        renderer.set_camera(camera);

        // Setup hillshade light from top left
        let light = (forward - right) * 0.5 + vec3(0.0, std::f32::consts::FRAC_1_SQRT_2, 0.0); // 45 degrees up
        renderer
            .uniform(self.program, "u_map_light")
            .set_vec3(&light.normalize());

        // Draw the terrain
        renderer.render_to_target(&self.minimap, vec4(0.67, 0.83, 0.96, 1.0), || {
            world.draw_terrain(renderer, self.program);
        });

        renderer.set_camera(main_camera);
    }
}
