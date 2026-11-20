use apricot::{
    camera::{floating_origin::FloatingOrigin, Camera, ProjectionKind},
    error::Error,
    model_set::ModelSet,
    opengl::DrawOptions,
    render::{lighting::DirectionalLight, target::RenderTarget, RenderContext},
    terrain::chunks::ChunkCoord,
};
use nalgebra_glm::{vec3, vec4, Mat4, Vec3};

use crate::{airplane::Airplane, heightmap::Heightmap};

pub const MINIMAP_PX: u32 = 256;
pub const MINIMAP_RANGE: f32 = 5_000.0; // meters from the plane to the edge of the map

pub struct Minimap {
    pub minimap: RenderTarget,
    minimap_light: DirectionalLight,
}

impl Minimap {
    pub fn new(renderer: &RenderContext, light_dir: Vec3) -> Result<Self, Error> {
        let minimap = renderer.create_render_target(MINIMAP_PX)?;
        renderer
            .get_texture_from_id(minimap.texture)
            .set_smooth(true);

        let mut minimap_light = DirectionalLight::new(light_dir, 1024)?;
        minimap_light.frustum.shadow_distance = f32::MAX;

        Ok(Self {
            minimap,
            minimap_light,
        })
    }

    pub fn render(
        &mut self,
        renderer: &RenderContext,
        sea: Mat4,
        plane: &Airplane,
        map: &Heightmap,
        chunks: &ModelSet<ChunkCoord>,
        old_light: &DirectionalLight,
        origin: &FloatingOrigin,
    ) {
        let h = plane.heading();
        let pos = origin.relative(plane.pos);
        let forward = vec3(h.sin(), 0.0, -h.cos()).cast();

        let top = map.max_height + 100.0;
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

        // Shadows over the map
        renderer.set_light(Some(&self.minimap_light));
        renderer.shadow_pass(&mut self.minimap_light, |pass| chunks.draw_shadows(pass));

        let p = renderer.builtins.program_3d;
        let w = renderer
            .get_program_id_from_name("water")
            .expect("water program should be setup by main");
        renderer.uniform(p, "u_minimap").set_i32(1);
        renderer.uniform(w, "u_minimap").set_i32(1);

        renderer.render_to_target(&self.minimap, vec4(0.05, 0.1, 0.2, 1.0), || {
            renderer.uniform(p, "u_detail").set_f32(1.0);
            chunks.draw(renderer);
            renderer.uniform(p, "u_detail").set_f32(0.0);
            renderer.draw_mesh_with_program(
                w,
                renderer.builtins.plane,
                sea,
                &DrawOptions::default(),
            );
        });

        // Cleanup
        renderer.uniform(p, "u_minimap").set_i32(0);
        renderer.uniform(w, "u_minimap").set_i32(0);
        renderer.set_camera(main_camera);
        renderer.set_light(Some(old_light));
    }
}
