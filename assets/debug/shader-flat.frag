#version 330 core

#include "stdlib.glsl"

uniform vec4 u_color;

layout (location = 0) out vec4 f_main;

void main() {
    f_main = u_color;
}
