uniform mat4 u_model_matrix;
uniform mat3 u_normal_matrix; // inverse transpose of the model matrix, for normals
uniform mat4 u_view_matrix;
uniform mat4 u_proj_matrix;
uniform mat4 u_light_mvp; // light's proj * view, only used when u_shadows != 0
uniform float u_shadow_texel; // world-space size of one shadow-map texel
uniform vec3 u_sun_dir; // direction towards the light
uniform vec3 u_camera_pos;

layout(location = 0) in vec3 a_position;
layout(location = 1) in vec3 a_normal;
layout(location = 2) in vec2 a_uv;

out vec3 v_normal;
out vec2 v_uv;
out vec4 v_light_space_pos;
out float v_view_dist;
out vec3 v_world_pos;

out vec3 v_sun; // direct sunlight reaching here, colored by the air
out vec3 v_sky_up; // light from the whole sky above ehre

out vec3 v_haze_r; // haze between here and the camera
out vec3 v_haze_m; // mie
out vec3 v_haze_trans; // how much of the surface shows through it

const float CAMERA_FAR = 150000.0; // keep in sync with world.rs

// How much air lies between the camera and a point `dist` away
float air_distance(float dist) {
    float edge = smoothstep(0.7 * CAMERA_FAR, CAMERA_FAR, dist);
    return dist / max(1.0 - edge, 1e-4);
}

void main() {
    vec4 world_pos = u_model_matrix * vec4(a_position, 1.0);
    vec3 n = normalize(u_normal_matrix * a_normal);
    vec3 l = normalize(u_sun_dir);

    // push vert down due to the curve of the earth
    vec2 d = world_pos.xz - u_camera_pos.xz;
    vec4 curved = world_pos;
    curved.y -= dot(d, d) / (2.0 * PLANET_R);

    vec4 view_pos = u_view_matrix * curved;
    v_view_dist = length(view_pos.xyz);

    // Look up the shadow from slightly above the surface, so it can never shadow itself.
    // 1 texel when facing the light, up to 3 when edge-on, where the self-shadowing error is biggest.
    float slope = 1.0 - clamp(dot(n, l), 0.0, 1.0);
    vec3 offset = n * u_shadow_texel * (1.0 + 2.0 * slope);

    v_normal = n;
    v_uv = a_uv;
    v_light_space_pos = u_light_mvp * vec4(world_pos.xyz + offset, 1.0);
    v_world_pos = world_pos.xyz; // use the flat world pos here for altitude and lighting stuff
    gl_Position = u_proj_matrix * view_pos;

    // Haze between here and the camera
    v_haze_r = vec3(0.0);
    v_haze_m = vec3(0.0);
    v_haze_trans = vec3(1.0);
    vec3 view = curved.xyz - u_camera_pos;
    float dist = length(view);
    scatter_sums(u_camera_pos.y, view / dist, l, air_distance(dist), 6, v_haze_r, v_haze_m, v_haze_trans);

    v_sun = sun_light(world_pos.y, l);
    vec3 unused;
    v_sky_up = scatter(world_pos.y, vec3(0.0, 1.0, 0.0), l, 1e12, 4, unused);
}
