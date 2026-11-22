//! The sky worked out once into small textures, so drawing it is a texture lookup

mod air_lut;

use apricot::{
    error::Error,
    render::{
        assets::{ProgramId, TextureId},
        target::{RenderTarget, TargetFormat},
        RenderContext,
    },
};
use nalgebra_glm::{vec4, Vec3};

/// Across is the angle around from the sun
const SKY_WIDTH: u32 = 192; // keep in sync with sky_lut.glsl
const SKY_HEIGHT: u32 = 128;

// Texture units the sky textures are read from
pub const AIR_UNIT: u32 = 2;
pub const VIEW_UNIT: u32 = 3;
pub const SEA_UNIT: u32 = 4;

pub struct Atmosphere {
    /// Air between any point in space
    air: TextureId,
    /// Draws the sky into `view` and `sea`
    sky_program: ProgramId,
    /// The sky seen from the camera
    view: RenderTarget,
    /// The sky seen from sea level, for reflections
    sea: RenderTarget,
    /// The sun `sea` was last drawn for
    sea_sun: Option<Vec3>,
}

impl Atmosphere {
    pub fn new(renderer: &RenderContext) -> Result<Self, Error> {
        // Air lookup, the same forever
        let air = renderer.create_texture_rgba(air_lut::WIDTH, air_lut::HEIGHT, &air_lut::pixels());
        renderer.get_texture_from_id(air).set_smooth(true);

        // Sky lookups, redrawn as the camera and sun move
        let sky_program = renderer.add_program_from_source(
            include_str!("../shaders/sky.vert"),
            glsl!(
                "../shaders/atmosphere.glsl",
                "../shaders/sky_lut.glsl",
                "../shaders/sky_lut.frag",
            ),
            None,
        )?;
        let atmosphere = Self {
            air,
            sky_program,
            view: renderer.create_render_target_with(SKY_WIDTH, SKY_HEIGHT, TargetFormat::Hdr)?,
            sea: renderer.create_render_target_with(SKY_WIDTH, SKY_HEIGHT, TargetFormat::Hdr)?,
            sea_sun: None,
        };
        atmosphere.connect(renderer, sky_program);

        Ok(atmosphere)
    }

    /// Let `program` read the lookups, through `u_air_lut`, `u_sky_view`, and `u_sea_sky`
    pub fn connect(&self, renderer: &RenderContext, program: ProgramId) {
        renderer
            .uniform(program, "u_air_lut") // programs without this one just ignore it
            .set_i32(AIR_UNIT as i32);
        renderer
            .uniform(program, "u_sky_view")
            .set_i32(VIEW_UNIT as i32);
        renderer
            .uniform(program, "u_sea_sky")
            .set_i32(SEA_UNIT as i32);
    }

    /// Update the lookups given the camera, altitude, and sun
    pub fn update(&mut self, renderer: &RenderContext, altitude: f32, sun: Vec3) {
        // Do the air first, since the sky is drawn from it
        renderer.bind_texture_unit(AIR_UNIT, self.air); // every frame, in case something else takes the unit

        if self.sea_sun != Some(sun) {
            // the sky from sea level only changes with the sun
            self.draw_sky(renderer, &self.sea, 0.0, sun);
            self.sea_sun = Some(sun);
        }
        self.draw_sky(renderer, &self.view, altitude, sun);

        // Hand the sky to everyone
        renderer.bind_texture_unit(VIEW_UNIT, self.view.texture);
        renderer.bind_texture_unit(SEA_UNIT, self.sea.texture);
    }

    fn draw_sky(&self, renderer: &RenderContext, target: &RenderTarget, altitude: f32, sun: Vec3) {
        renderer
            .uniform(self.sky_program, "u_sun_dir")
            .set_vec3(&sun);
        renderer
            .uniform(self.sky_program, "u_lut_altitude")
            .set_f32(altitude);
        renderer.render_to_target(target, vec4(0.0, 0.0, 0.0, 1.0), || {
            renderer.draw_fullscreen(self.sky_program)
        });
    }
}
