// psuedo-random 0..1 from a 2D point
float hash(vec2 p) {
    uvec2 q = uvec2(ivec2(floor(p)));
    uint h = q.x ^ (q.y * 0x9E3779B9u);
    h ^= h >> 16;
    h *= 0x7FEB352Du;
    h ^= h >> 15;
    h *= 0x846CA68Bu;
    h ^= h >> 16;
    return float(h) * (1.0 / 4294967295.0);
}

// smooth 0..1 noise, one random value per whole-number grid cell, blended between
float value_noise(vec2 p) {
    vec2 i = floor(p);
    vec2 f = fract(p);
    vec2 u = f * f * (3.0 - 2.0 * f);
    return mix(mix(hash(i), hash(i + vec2(1.0,0.0)), u.x),
        mix(hash(i + vec2(0.0, 1.0)), hash(i + vec2(1.0, 1.0)), u.x), u.y);
}
