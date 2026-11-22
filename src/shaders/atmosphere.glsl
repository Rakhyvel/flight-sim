uniform sampler2D u_air_lut; // see air_lut.rs

const float AIR_LUT_MAX = 1e6; // keep in sync with air_lut.rs
const vec2 AIR_LUT_SIZE = vec2(256.0, 64.0);

// Distances in meters, sea level at y=0
const float PI = 3.14159265;
const float PLANET_R = 6371e3;
const float ATMOS_R = 6471e3;
const vec3 BETA_R = vec3(5.8e-6, 13.5e-6, 33.1e-6); // Rayleigh, why the sky looks blue!
const float BETA_M = 21e-6; // Mie (haze and aerosols), the "visibility knob"
const float H_R = 8000.0; // Rayleigh scale height
const float H_M = 1200.0; // Mie scale height
const float MIE_G = 0.76; // How forward-scattering the haze is (the glow around the sun)
const float SUN_INTENSITY = 20.0;
const vec3 WATER_BODY = vec3(0.004, 0.022, 0.035); // deep ocean, linear

// Near and far distances where the ray hits a sphere of radius r at the origin. (-1, -1) on a miss
vec2 ray_sphere(vec3 ro, vec3 rd, float r) {
    float b = dot(ro, rd);
    float c = dot(ro, ro) - r * r;
    float d = b * b - c;
    if (d < 0.0) return vec2(-1.0);
    d = sqrt(d);
    return vec2(-b - d, -b + d);
}

float phase_rayleigh(float mu) {
    return 3.0 / (16.0 * PI) * (1.0 + mu * mu);
}

float phase_mie(float mu) {
    float g2 = MIE_G * MIE_G;
    return 3.0 / (8.0 * PI) * ((1.0 - g2) * (1.0 + mu * mu))
        / ((2.0 + g2) * pow(1.0 + g2 - 2.0 * MIE_G * mu, 1.5));
}

// How much air (rayleigh, mie) lies between p and space, toward the sun
vec2 optical_depth_to_sun(vec3 p, vec3 sun) {
    float r = length(p);
    float mu = dot(p, sun) / r;
    float v = sqrt(clamp((r - PLANET_R) / (ATMOS_R - PLANET_R), 0.0, 1.0));
    vec2 uv = (0.5 + vec2(mu * 0.5 + 0.5, v) * (AIR_LUT_SIZE - 1.0)) / AIR_LUT_SIZE; // so the ends land on texel centers
    vec4 t = textureLod(u_air_lut, uv, 0.0); // explicit level, so it works in vertex shaders too
    const vec2 BYTES = vec2(65280.0, 255.0) / 65535.0; // high byte, low byte
    return vec2(dot(t.rg, BYTES), dot(t.ba, BYTES)) * AIR_LUT_MAX;
}

vec3 transmittance(vec2 od) {
    return exp(-(BETA_R * od.x + BETA_M * 1.1 * od.y));
}

// The color and brightness of direct sunlight at altitude `alt`. White at noon, orange at sunset/sunrise
vec3 sun_light(float alt, vec3 sun) {
    vec3 p = vec3(0.0, PLANET_R + max(alt, 1.0), 0.0);
    if (ray_sphere(p, sun, PLANET_R).x > 0.0) {
        return vec3(0.0); // its nighttime babey!
    }
    return SUN_INTENSITY * transmittance(optical_depth_to_sun(p, sun));
}

// Light scattered toward the eye along a ray, before the sun's direction is taken into account, as (rayleight, mie).
void scatter_sums(float alt, vec3 dir, vec3 sun, float max_dist, int steps, out vec3 sum_r, out vec3 sum_m, out vec3 trans) {
    vec3 ro = vec3(0.0, PLANET_R + max(alt, 1.0), 0.0);
    float t_end = ray_sphere(ro, dir, ATMOS_R).y;
    float ground = ray_sphere(ro, dir, PLANET_R).x;
    if (ground > 0.0) {
        t_end = min(t_end, ground);
    }
    t_end = min(t_end, max_dist);

    trans = vec3(1.0);
    sum_r = vec3(0.0);
    sum_m = vec3(0.0);

    float t_prev = 0.0;
    for (int i = 0; i < steps; i++) {
        float u = float(i + 1) / float(steps);
        float t_next = t_end * u * u;
        float dt = t_next - t_prev;
        vec3 p = ro + dir * (t_prev + 0.5 * dt);
        t_prev = t_next;

        vec2 density = exp(-(length(p) - PLANET_R) / vec2(H_R, H_M));
        vec3 extinction = BETA_R * density.x + BETA_M * 1.1 * density.y;
        vec3 step_trans = exp(-extinction * dt);

        if (ray_sphere(p, sun, PLANET_R).x <= 0.0) {
            // Not in the earth's shadow
            vec3 sun_t = transmittance(optical_depth_to_sun(p, sun));
            vec3 weight = trans * sun_t * (1.0 - step_trans) / extinction;
            sum_r += density.x * weight;
            sum_m += density.y * weight;
        }
        trans *= step_trans;
    }
}

// The light from `scatter_sums`, seen looking along `dir`
vec3 scatter_light(vec3 sum_r, vec3 sum_m, vec3 dir, vec3 sun) {
    float mu = dot(dir, sun);
    return SUN_INTENSITY * (sum_r * BETA_R * phase_rayleigh(mu) + sum_m * BETA_M * phase_mie(mu));
}

// Light scatted towards the eye along a ray from altitude `alt` in `dir`, for up to `max_dist`.
// `trans` is how much of whatever's at the end of the ray still gets through
vec3 scatter(float alt, vec3 dir, vec3 sun, float max_dist, int steps, out vec3 trans) {
    vec3 sum_r, sum_m;
    scatter_sums(alt, dir, sun, max_dist, steps, sum_r, sum_m, trans);
    return scatter_light(sum_r, sum_m, dir, sun);
}

// draw the great deep beyond, so that the horizon looks right
vec3 sea_beyond(float alt, vec3 dir, vec3 sun) {
    vec3 ro = vec3(0.0, PLANET_R + max(alt, 1.0), 0.0);
    float hit = ray_sphere(ro, dir, PLANET_R).x;
    if (hit <= 0.0) {
        return vec3(0.0);
    }

    vec3 n = normalize(ro + dir * hit);
    float sin_graze = clamp(dot(-dir, n), 0.0, 1.0); // how steeply the ray meets the sea

    float cos_graze = sqrt(1.0 - sin_graze * sin_graze);
    vec2 across = normalize(dir.xz);
    vec3 r = vec3(across.x * cos_graze, sin_graze, across.y * cos_graze);

    vec3 unused;
    vec3 sky_refl = scatter(0.0, r, sun, 1e12, 6, unused);
    vec3 body = WATER_BODY / PI * sun_light(0.0, sun) * max(sun.y, 0.0);
    float fresnel = 0.02 + 0.98 * pow(1.0 - sin_graze, 5.0);
    return mix(body, sky_refl, fresnel);
}
