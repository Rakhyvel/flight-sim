#version 330 core

layout(location = 0) in vec3 a_position;

out vec2 v_ndc;

void main() {
    // Assumes we're drawing the fullscreen [-1, 1] quad
    v_ndc = a_position.xy;
    gl_Position = vec4(a_position.xy, 1.0, 1.0); // exactly at the far plane, so anything drawn wins
}
