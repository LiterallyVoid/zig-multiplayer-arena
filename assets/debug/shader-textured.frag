#version 330 core

#include "stdlib.glsl"

uniform vec4 u_color;
uniform sampler2D u_texture;

in vec2 v_uv;

layout (location = 0) out vec4 f_main;

void main() {
    float alpha = texture(u_texture, v_uv).r;
    alpha = clamp((alpha - 0.5) / fwidth(alpha) + 0.5, 0.0, 1.0);
    f_main = u_color * alpha;
}
