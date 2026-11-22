// The sky worked out ahead of time into a texture, see sky.rs
const vec2 SKY_LUT_SIZE = vec2(192.0, 128.0); // keep in sync with sky.rs

// How far below level the true horizon is from altitude `alt` [rad, negative]
float horizon_elevation(float alt) {
    return -acos(PLANET_R / (PLANET_R + max(alt, 1.0)));
}

// Where `dir` lands in a sky texture made at altitude `alt`
vec2 sky_lut_uv(float alt, vec3 dir, vec3 sun) {
    // Up and down, with rows bunched around the horizon where the sky changes fastest
    float horizon = horizon_elevation(alt);
    float el = asin(clamp(dir.y, -1.0, 1.0));
    float v = el >= horizon
        ? 0.5 + 0.5 * sqrt((el - horizon) / (0.5 * PI - horizon)) : 0.5 - 0.5 * sqrt((horizon - el) / (0.5 * PI + horizon));

    // Around from the sun, 0 to 1 for 0 to 180 degrees, since both sides are the same
    float az = 0.0;
    if (dot(dir.xz, dir.xz) > 1e-8 && dot(sun.xz, sun.xz) > 1e-8) { // straight up or down has no "around"
        az = acos(clamp(dot(normalize(dir.xz), normalize(sun.xz)), -1.0, 1.0)) / PI;
    }

    return (0.5 + vec2(az, v) * (SKY_LUT_SIZE - 1.0)) / SKY_LUT_SIZE; // so the ends land on texel centers
}

// Which way the texel at `uv` looks, in a sky texture made at `alt`, with the sun toward +x
vec3 sky_lut_dir(float alt, vec2 uv) {
    uv = (uv * SKY_LUT_SIZE - 0.5) / (SKY_LUT_SIZE - 1.0); // undo the texel center squeeze
    float horizon = horizon_elevation(alt);
    float el;
    if (uv.y >= 0.5) {
        float l = 2.0 * uv.y - 1.0;
        el = horizon + l * l * (0.5 * PI - horizon);
    } else {
        float l = 1.0 - 2.0 * uv.y;
        el = horizon - l * l * (0.5 * PI + horizon);
    }
    float az = uv.x * PI;
    return vec3(cos(el) * cos(az), sin(el), cos(el) * sin(az));
}

// The sky along `dir`, out of `lut`, which was made at altitude `alt`
vec3 sky_lut(sampler2D lut, float alt, vec3 dir, vec3 sun) {
    return texture(lut, sky_lut_uv(alt, dir, sun)).rgb;
}
