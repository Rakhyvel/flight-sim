const float EXPOSURE = 1.3;

// Convert linear HDR to display, use ACES curve cause it looks cool
vec3 tonemap(vec3 c) {
    c *= EXPOSURE;
    c = clamp((c * (2.51 * c + 0.03)) / (c * (2.43 * c + 0.59) + 0.14), 0.0, 1.0);
    c = pow(c, vec3(1.0 / 2.2));
    float n = fract(sin(dot(gl_FragCoord.xy, vec2(12.9898, 78.233))) * 43758.5453);
    return c + (n - 0.5) / 255.0;
}
