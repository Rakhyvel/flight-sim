#version 330 core
layout (location = 0) in vec3 a_position;
out vec3 v_world_ray;

uniform mat4 u_inv_view_proj; // Or derived from camera matrices

void main() {
    // Standard fullscreen triangle or sky dome coordinates
    gl_Position = vec4(a_position.xy, 0.9999, 1.0); // pushed to far plane

    // Unproject to get world-space view ray for this pixel
    vec4 ray = u_inv_view_proj * vec4(a_position.xy, 1.0, 1.0);
    v_world_ray = normalize(ray.xyz / ray.w);
}
