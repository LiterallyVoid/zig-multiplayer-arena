#version 330 core

#include "stdlib.glsl"

in vec3 v_position_model;
in vec3 v_normal_model;
in vec3 v_barycentric;

layout (location = 0) out vec4 f_main;

// In each axis, returns 0.0 if `-line_width < along_axis < line_width`, modulo 1.0. Otherwise, returns 1.0.
vec3 gridAxes(vec3 along, float line_width) {
    vec3 smooth_width = fwidth(along);

    vec3 fraction = fract(along + 0.5) - 0.5;

    vec3 adjust_line_width = max(smooth_width / line_width * 0.5, 1.0);
    vec3 line_width_adjusted = line_width * adjust_line_width;
    
    vec3 value = clamp((abs(fraction) - line_width_adjusted + smooth_width * 0.5) / smooth_width, 0.0, 1.0);

    value = 1.0 - ((1.0 - value) / adjust_line_width);

    return value;
}

vec3 gridPlanes(vec3 position) {
    vec3 lines = gridAxes(position, 0.007) * gridAxes(position * 4.0, 0.007);

    return vec3(
        lines.y * lines.z,
        lines.z * lines.x,
        lines.x * lines.y
    );
}

// Given a component of normal, passed as `axis`, returns what share of the
// final result this component's plane should take.
float triplanarAxisShare(float axis) {
    return abs(axis);
}

void main() {
    f_main = vec4(1.0, 0.5, 0.2, 1.0);

    vec3 grid_axes = gridPlanes(v_position_model);
    vec4 grid_axes_with_w = vec4(grid_axes, 1.0);

    vec2 grid = vec2(0.0);

    grid += grid_axes_with_w.xw * triplanarAxisShare(v_normal_model.x);
    grid += grid_axes_with_w.yw * triplanarAxisShare(v_normal_model.y);
    grid += grid_axes_with_w.zw * triplanarAxisShare(v_normal_model.z);

    vec3 barycentric_pixels = v_barycentric / fwidth(v_barycentric);
    float wireframe = min(barycentric_pixels.x, min(barycentric_pixels.y, barycentric_pixels.z));
    wireframe = clamp(1.0 - wireframe, 0.0, 1.0);

    f_main.xyz *= (grid.x / grid.y) * 0.3 + 0.7;
    f_main.xyz = mix(f_main.xyz, vec3(1.0, 0.0, 0.0), wireframe * 0.3);
}
