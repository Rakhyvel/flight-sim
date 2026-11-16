#version 330 core
in vec3 v_world_ray;
out vec4 color;

uniform vec3 u_sun_dir;

void main() {
    vec3 view_dir = normalize(v_world_ray);

    // Zenith angle (0 at horizon, 1 straight up)
    float zenith = clamp(view_dir.y, 0.0, 1.0);
    float sun_height = normalize(u_sun_dir).y;

    // Atmosphere gradient based on sun position
    vec3 horizon_color = mix(vec3(0.95, 0.65, 0.4), vec3(0.6, 0.75, 0.9), smoothstep(-0.2, 0.4, sun_height));
    vec3 zenith_color  = mix(vec3(0.15, 0.2, 0.35), vec3(0.2, 0.4, 0.7), smoothstep(-0.2, 0.4, sun_height));

    vec3 sky = mix(horizon_color, zenith_color, pow(zenith, 0.4));

    // Sun disc / Mie halo glow
    float cos_theta = dot(view_dir, normalize(u_sun_dir));
    float sun_glow = pow(clamp(cos_theta, 0.0, 1.0), 256.0);
    sky += vec3(1.0, 0.8, 0.6) * sun_glow * max(sun_height, 0.0) * 2.0;

    color = vec4(sky, 1.0);
}
