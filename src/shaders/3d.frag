uniform sampler2D u_texture;
uniform sampler2D u_normal_map;
uniform vec4 u_color;
uniform vec3 u_sun_dir;
uniform vec3 u_camera_pos;
uniform float u_detail; // 1 for noise, 0 for everything else
uniform int u_minimap; // 1 while drawing the minimap, no air between the camera and the ground
uniform vec2 u_origin;

in vec3 v_normal;
in vec2 v_uv;
in vec3 v_world_pos;

in vec3 v_sun;
in vec3 v_sky_up;

in vec3 v_haze_r; // haze between here and the camera
in vec3 v_haze_m; // mie
in vec3 v_haze_trans; // how much of the surface shows through it

out vec4 color;

// Pull corner-to-corner `uv` into texel centers
vec2 edge_uv(sampler2D tex, vec2 uv) {
    vec2 size = vec2(textureSize(tex, 0));
    return (uv * (size - 1.0) + 0.5) / size;
}

// The ground's normal at corner-to-corner `uv`
vec3 terrain_normal(vec2 uv) {
    vec4 t = texture(u_normal_map, edge_uv(u_normal_map, uv));
    const vec2 BYTES = vec2(65280.0, 255.0) / 65535.0; // high byte, low byte
    vec2 xz = vec2(dot(t.rg, BYTES), dot(t.ba, BYTES)) * 2.0 - 1.0;
    return vec3(xz.x, sqrt(max(1.0 - dot(xz, xz), 0.0)), xz.y);
}

float ground_detail(vec2 p) {
    float footprint = length(fwidth(p));
    const mat2 TURN = mat2(0.8, 0.6, -0.6, 0.8);
    float d = 0.0;
    float amp = 0.5;
    float freq = 1.0 / 8.0;
    for (int i = 0; i < 4; i++) {
        float keep = 1.0 - smoothstep(0.25, 1.0, freq * footprint);
        if (keep <= 0.0) break; // every octave after this is finer still
        d += (value_noise(p * freq) - 0.5) * amp * keep;
        p = TURN * p + vec2(31.7, -17.3);
        freq *= 2.3;
        amp *= 0.5;
    }
    return d;
}

void main() {
    // unda da sea
    if (u_detail > 0.5 && v_world_pos.y < -1.0) {
        color = vec4(0.0, 0.0, 0.0, 1.0);
        return;
    }

    // Terrain lines its texture up on the tile edges and gets finer normals than its mesh
    bool terrain = u_detail > 0.5;
    vec3 n = terrain ? terrain_normal(v_uv) : normalize(v_normal);
    vec3 l = normalize(u_sun_dir);

    vec4 base = texture(u_texture, terrain ? edge_uv(u_texture, v_uv) : v_uv) * u_color;
    vec3 albedo = pow(base.rgb, vec3(2.2)); // convert color to linear
    albedo *= 1.0 + 0.6 * ground_detail(v_world_pos.xz + u_origin) * u_detail;

    // Direct sun, colored by the atmosphere
    float diffuse = max(dot(n, l), 0.0) * shadow_visibility();

    // Sky light
    vec3 skylight = v_sky_up * PI * (0.6 + 0.4 * n.y);

    vec3 lit = albedo / PI * (v_sun * diffuse + skylight);

    // Dim far away surfaces and add air
    vec3 view_dir = normalize(v_world_pos - u_camera_pos); // flat is close enough for the sun's glow
    vec3 inscatter = scatter_light(v_haze_r, v_haze_m, view_dir, l);

    color = vec4(tonemap(lit * v_haze_trans + inscatter), base.a);
}
