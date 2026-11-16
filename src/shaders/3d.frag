#version 330 core
uniform sampler2D u_texture;
uniform sampler2DShadow u_shadow_map;
uniform int u_shadows;
uniform vec4 u_color;
uniform vec3 u_sun_dir;

in vec3 v_normal;
in vec2 v_uv;
in vec4 v_light_space_pos;
in float v_view_dist;
in vec3 v_world_pos; // Make sure your vertex shader passes world position!
out vec4 color;

// Scattering coefficients (simulating Earth's atmosphere)
const vec3 RAYLEIGH_COLOR = vec3(0.25, 0.45, 0.85); // Deep sky blue
const vec3 MIE_COLOR = vec3(1.0, 0.75, 0.5);       // Golden hour sun glow
const float FOG_DENSITY = 0.0002;

float shadow_visibility() {
    if (u_shadows == 0) return 1.0;
    vec3 p = v_light_space_pos.xyz / v_light_space_pos.w * 0.5 + 0.5;
    if (p.z > 1.0) return 1.0;
    vec2 texel = 1.0 / vec2(textureSize(u_shadow_map, 0));
    float lit = 0.0;
    for (int x = -1; x <= 1; x++)
        for (int y = -1; y <= 1; y++)
            lit += texture(u_shadow_map, vec3(p.xy + vec2(x, y) * texel, p.z));
    float edge = max(abs(p.x - 0.5), abs(p.y - 0.5)) * 2.0;
    return mix(lit / 9.0, 1.0, smoothstep(0.8, 1.0, edge));
}

void main() {
    vec3 n = normalize(v_normal);
    vec3 l = normalize(u_sun_dir);

    // View direction from camera to fragment
    vec3 view_dir = normalize(v_world_pos);

    // --- SCATTERING CALCULATIONS ---
    // Cosine of the angle between view direction and sun direction (Henny-Greenstein / Phase approximation)
    float cos_theta = dot(view_dir, l);

    // Rayleigh phase term (tells us how much light scatters based on angle to sun)
    float rayleigh_phase = 0.75 * (1.0 + cos_theta * cos_theta);

    // Mie phase term (forward-scattering directional glow around the sun)
    float g = 0.76; // asymmetry factor for haze
    float mie_phase = (1.0 - g * g) / (4.0 * 3.14159 * pow(1.0 + g * g - 2.0 * g * cos_theta, 1.5));

    // Zenith factor: how high up in the sky are we looking? (0 = horizon, 1 = straight up)
    float zenith = clamp(view_dir.y, 0.0, 1.0);

    // Dynamic horizon glow: sun height changes the overall atmospheric tint
    float sun_height = l.y; // > 0 for day, near 0 for golden hour, < 0 for night

    // Blend Rayleigh (overhead blue) and Mie (sun-aligned golden haze) based on zenith and sun angle
    vec3 atmospheric_horizon_color = mix(vec3(0.95, 0.65, 0.4), vec3(0.6, 0.75, 0.9), smoothstep(-0.2, 0.4, sun_height));
    vec3 atmospheric_zenith_color  = mix(vec3(0.15, 0.2, 0.35), vec3(0.2, 0.4, 0.7), smoothstep(-0.2, 0.4, sun_height));

    // Combine into a dynamic sky/fog color that shifts from ground to zenith
    vec3 dynamic_fog_color = mix(atmospheric_horizon_color, atmospheric_zenith_color, pow(zenith, 0.4));

    // Add extra Mie glow intensity directly around the sun position
    dynamic_fog_color += MIE_COLOR * mie_phase * max(sun_height, 0.1) * 0.3;

    // --- LIGHTING & FOG APPLICATION ---
    float diffuse = max(dot(n, l), 0.0) * shadow_visibility();
    vec4 base = texture(u_texture, v_uv) * u_color;

    vec3 ambient = vec3(0.3, 0.28, 0.25);
    vec3 lit = base.rgb * (ambient + (1.0 - ambient) * diffuse);

    // Exponential distance fog blended with our dynamic atmospheric scattering color
    float fog = 1.0 - exp(-v_view_dist * FOG_DENSITY);
    color = vec4(mix(lit, dynamic_fog_color, fog), base.a);
}
