#version 330 core

#include "stdlib.glsl"

uniform vec4 u_color;
uniform sampler2D u_texture;

in vec2 v_uv;
in vec4 v_color;

layout (location = 0) out vec4 f_main;

void main() {
    f_main = u_color * v_color;
}
