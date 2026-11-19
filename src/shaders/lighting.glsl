uniform sampler2DShadow u_shadow_map;
uniform int u_shadows;
in vec4 v_light_space_pos;


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
