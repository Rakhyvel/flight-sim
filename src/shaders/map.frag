uniform vec3 u_map_light; // toward the light, from the top left of the map whichever way it's turned

in vec3 v_world_pos;
in vec2 v_uv;

out vec4 color;

const vec3 WATER = vec3(0.67, 0.83, 0.96);
const vec3 COAST = vec3(0.42, 0.60, 0.78);
const float SEA_LEVEL = 0.5; // [m] just above zero

// Pale map colors by height [m]
vec3 elevation_tint(float h) {
    vec3 c = vec3(0.80, 0.89, 0.74);
    c = mix(c, vec3(0.88, 0.90, 0.76), smoothstep(0.0, 600.0, h));
    c = mix(c, vec3(0.91, 0.86, 0.74), smoothstep(600.0, 1800.0, h));
    c = mix(c, vec3(0.83, 0.77, 0.70), smoothstep(1800.0, 3000.0, h));
    c = mix(c, vec3(0.97, 0.97, 0.97), smoothstep(3000.0, 3800.0, h));
    return c;
}

void main() {
    float h = v_world_pos.y;
    float shore_px = (h - SEA_LEVEL) / max(fwidth(h), 1e-4); // pixels from the coast, negative out to sea

    // Land
    vec3 land = elevation_tint(h);
    float hill = 1.0 + 0.9 * (dot(terrain_normal(v_uv), u_map_light) - u_map_light.y); // flat ground keeps it own color
    land *= clamp(hill, 0.6, 1.15);

    // Coastline
    land = mix(COAST, land, smoothstep(1.0, 2.0, shore_px));

    // Mix in sea
    color = vec4(mix(WATER, land, smoothstep(-0.5, 0.5, shore_px)), 1.0);
}
