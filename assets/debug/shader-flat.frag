#version 330 core

#include "stdlib.glsl"

layout (location = 0) out vec4 f_main;

void main() {
    f_main = vec4(1.0, 0.5, 0.2, 1.0);
}
