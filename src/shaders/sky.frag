in vec2 v_ndc;
out vec4 color;

uniform vec3 u_sun_dir;
uniform vec3 u_camera_pos;
uniform mat4 u_inv_view_proj;
uniform sampler2D u_sky_view;

void main() {
    vec4 far = u_inv_view_proj * vec4(v_ndc, 1.0, 1.0);
    vec3 dir = normalize(far.xyz / far.w - u_camera_pos);
    vec3 sun = normalize(u_sun_dir);

    vec3 sky = sky_lut(u_sky_view, u_camera_pos.y, dir, sun);

    // Sun disk, dimmed by the air between here and space
    float disk = smoothstep(0.99994, 0.99997, dot(dir, sun));
    sky += disk * sun_light(u_camera_pos.y, sun) * 40.0;

    color = vec4(tonemap(sky), 1.0);
}
