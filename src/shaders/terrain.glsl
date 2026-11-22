uniform sampler2D u_normal_map;

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
