#version 330 core

#include "stdlib.glsl"

uniform vec4 u_color;
uniform sampler2D u_texture;

in vec2 v_uv;

layout (location = 0) out vec4 f_main;

void main() {
    f_main = texture(u_texture, v_uv) * u_color;
}
