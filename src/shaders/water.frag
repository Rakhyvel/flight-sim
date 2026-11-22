uniform vec3 u_sun_dir;
uniform vec3 u_camera_pos;
uniform float u_time;
uniform vec2 u_origin;
uniform sampler2D u_sky_view;
uniform sampler2D u_sea_sky;

in vec3 v_world_pos;

in vec3 v_sun;
in vec3 v_sky_up;

out vec4 color;

const float CAMERA_FAR = 150000.0; // keep up to date!

const vec2 WIND_DIR = vec2(0.944, 0.330);
const float CHOP = 0.7;

float air_distance(float dist) {
    float edge = smoothstep(0.7 * CAMERA_FAR, CAMERA_FAR, dist);
    return dist / max(1.0 - edge, 1e-4);
}

vec2 wave_slope(vec2 p, float t, out float filtered) {
    float footprint = length(fwidth(p)); // meters per pixel here

    // Gusts make some patches of waves stronger or weaker
    vec2 drift = WIND_DIR * 3.0 * t;
    float gust = 0.6 * value_noise((p - drift) / 400.0)
            + 0.4 * value_noise((p - drift) / 150.0);

    float k = 0.06; // wavenumber, 2pi / wavelength
    float amp = 0.6; // meters
    vec2 slope = vec2(0.0);
    vec2 warp = vec2(0.0); // how far the bigger waves have pushed the surface sideways
    float lost = 0.0;
    float total = 0.0;
    for (int i = 0; i < 12; i++) {
        float fi = float(i);

        // A random wind-like direction for each octave, and a random phase
        float angle = (hash(vec2(fi, 1.7)) - 0.5) * 1.0;
        vec2 dir = mat2(cos(angle), sin(angle), -sin(angle), cos(angle)) * WIND_DIR;
        float phase0 = hash(vec2(fi, 9.1)) * 6.2832;

        float omega = sqrt(9.81 * k); // long waves travel faster
        float phase = k * dot(dir, p + warp) - omega * t + phase0;

        // Gusts only move the small waves, the long swells come from far away and ignores them
        float gusty = smoothstep(1.0, 6.0, fi);
        float a = amp * mix(1.0, 0.3 + 1.4 * gust, gusty);

        float steep = a * k;
        float keep = 1.0 - smoothstep(0.5, 2.0, k * footprint);
        slope += dir * steep * cos(phase) * keep;
        // sideways push so the next smaller octaves bunch up on this one's crest
        warp -= dir * a * sin(phase) * CHOP * keep;

        lost += steep * (1.0 - keep);
        total += steep;
        k *= mix(1.06, 1.85, hash(vec2(fi, 4.3))); // uneven steps so octaves don't share a rhythm
        amp *= 0.58;
    }
    filtered = lost / total;
    return slope;
}

void main() {
    vec3 l = normalize(u_sun_dir);
    vec3 view = v_world_pos - u_camera_pos;
    view.y -= dot(view.xz, view.xz) / (2.0 * PLANET_R);
    float dist = length(view);
    vec3 v = view / dist;

    float filtered;
    vec2 s = wave_slope(v_world_pos.xz + u_origin, u_time, filtered);
    vec3 n = normalize(vec3(-s.x, 1.0, -s.y));

    // Water fresnel
    float cos_v = max(dot(-v, n), 0.0);
    float fresnel = 0.02 + 0.98 * pow(1.0 - cos_v, 5.0);

    // Reflected sky
    vec3 r = reflect(v, n);
    r.y = abs(r.y); // redirect any steep waves back upwards
    vec3 unused;
    vec3 sky_refl = sky_lut(u_sea_sky, 0.0, r, l);

    // Light coming back up out of the water
    vec3 body = WATER_BODY / PI * (v_sun * max(l.y, 0.0) + v_sky_up * PI);

    // Sun glint
    float rough = 0.02 * 0.15 + filtered;
    float shininess = 2.0 / (rough * rough) - 2.0;
    vec3 h = normalize(l - v);
    float f_h = 0.02 + 0.98 * pow(1.0 - max(dot(l, h), 0.0), 5.0);
    float spec = (shininess + 8.0) / (8.0 * PI) * pow(max(dot(n, h), 0.0), shininess);
    vec3 glint = v_sun * spec * f_h * max(dot(n, l), 0.0) * shadow_visibility();

    vec3 water = mix(body, sky_refl, fresnel) + glint;

    // Haze between here and the camera
    vec3 trans = vec3(1.0);
    vec3 inscatter = scatter(u_camera_pos.y, v, l, dist, 8, trans);
    vec3 result = water * trans + inscatter;

    // blend into far-off sea
    float edge = smoothstep(0.7 * CAMERA_FAR, CAMERA_FAR, dist);
    if (edge > 0.0) {
        vec3 far = sky_lut(u_sky_view, u_camera_pos.y, v, l); // already has the sea below the horizon in it
        result = mix(result, far, edge);
    }

    color = vec4(tonemap(result), 1.0);
}
