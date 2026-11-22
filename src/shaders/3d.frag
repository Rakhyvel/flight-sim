uniform sampler2D u_texture;
uniform vec4 u_color;
uniform vec3 u_sun_dir;
uniform vec3 u_camera_pos;
uniform float u_detail; // 1 for noise, 0 for everything else
uniform int u_minimap; // 1 while drawing the minimap, no air between the camera and the ground
uniform vec2 u_origin;

in vec3 v_normal;
in vec2 v_uv;
in vec3 v_world_pos;

out vec4 color;

const float CAMERA_FAR = 60000.0; // keep in sync with main.rs

// How much air lies between the camera and a point `dist` away
float air_distance(float dist) {
    float edge = smoothstep(0.7 * CAMERA_FAR, CAMERA_FAR, dist);
    return dist / max(1.0 - edge, 1e-4);
}

float ground_detail(vec2 p) {
    float footprint = length(fwidth(p));
    const mat2 TURN = mat2(0.8, 0.6, -0.6, 0.8);
    float d = 0.0;
    float amp = 0.5;
    float freq = 1.0 / 8.0;
    for (int i = 0; i < 4; i++) {
        float keep = 1.0 - smoothstep(0.25, 1.0, freq * footprint);
        d += (value_noise(p * freq) - 0.5) * amp * keep;
        p = TURN * p + vec2(31.7, -17.3);
        freq *= 2.3;
        amp *= 0.5;
    }
    return d;
}

void main() {
    vec3 n = normalize(v_normal);
    vec3 l = normalize(u_sun_dir);

    vec4 base = texture(u_texture, v_uv) * u_color;
    vec3 albedo = pow(base.rgb, vec3(2.2)); // convert color to linear
    albedo *= 1.0 + 0.6 * ground_detail(v_world_pos.xz + u_origin) * u_detail;

    // Direct sun, colored by the atmosphere
    vec3 sun = sun_light(v_world_pos.y, l);
    float diffuse = max(dot(n, l), 0.0) * shadow_visibility();

    // Sky light
    vec3 unused;
    vec3 sky_up = scatter(v_world_pos.y, vec3(0.0, 1.0, 0.0), l, 1e12, 4, unused);
    vec3 skylight = sky_up * PI * (0.6 + 0.4 * n.y);

    vec3 lit = albedo / PI * (sun * diffuse + skylight);

    // Dim far away surfaces and add air
    vec3 trans = vec3(1.0);
    vec3 inscatter = vec3(0.0);
    if (u_minimap == 0) {
        vec3 view = v_world_pos - u_camera_pos;
        view.y -= dot(view.xz, view.xz) / (2.0 * PLANET_R);
        float dist = length(view);
        inscatter = scatter(u_camera_pos.y, view / dist, l, air_distance(dist), 8, trans);
    }

    color = vec4(tonemap(lit * trans + inscatter), base.a);
}
