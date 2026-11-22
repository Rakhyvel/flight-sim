in vec2 v_ndc;
out vec4 color;

uniform vec3 u_sun_dir;
uniform vec3 u_camera_pos;
uniform mat4 u_inv_view_proj;

void main() {
    vec4 far = u_inv_view_proj * vec4(v_ndc, 1.0, 1.0);
    vec3 dir = normalize(far.xyz / far.w - u_camera_pos);
    vec3 sun = normalize(u_sun_dir);

    vec3 trans;
    vec3 sky = scatter(u_camera_pos.y, dir, sun, 1e12, 16, trans);

    sky += trans * sea_beyond(u_camera_pos.y, dir, sun); // below the horizon the ray ends at the sea

    // Sun disk
    float disk = smoothstep(0.99994, 0.99997, dot(dir, sun));
    sky += disk * trans * SUN_INTENSITY * 40.0;

    color = vec4(tonemap(sky), 1.0);
}
