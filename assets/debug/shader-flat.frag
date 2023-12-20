#version 330 core

#include "stdlib.glsl"

uniform vec4 u_color;

in vec3 v_normal;

layout (location = 0) out vec4 f_main;

void main() {
    vec3 sun = vec3(3.0, 14.0, 18.0) / 23.0;

    vec3 normal = normalize(v_normal);

    if (!gl_FrontFacing) {
        normal *= -1.0;
    }

    f_main = u_color;
    f_main.xyz *= dot(normal, sun) * 0.5 + 0.5;
    f_main.xyz = normal * 0.5 + 0.5;
}
