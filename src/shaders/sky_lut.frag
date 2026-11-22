uniform vec3 u_sun_dir;
uniform float u_lut_altitude;

out vec4 color;

void main() {
    // The sky's the same all the way around the sun, so turn the sun toward +x
    vec3 sun = normalize(u_sun_dir);
    sun = vec3(length(sun.xz), sun.y, 0.0);

    vec3 dir = sky_lut_dir(u_lut_altitude, gl_FragCoord.xy / SKY_LUT_SIZE);
    vec3 trans;
    vec3 sky = scatter(u_lut_altitude, dir, sun, 1e12, 16, trans);
    sky += trans * sea_beyond(u_lut_altitude, dir, sun); // below the horizon the ray ends at the sea

    color = vec4(sky, 1.0);
}
