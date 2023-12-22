#version 330 core

#include "stdlib.glsl"

uniform vec4 u_color;
uniform sampler2D u_texture;

in vec2 v_uv;

layout (location = 0) out vec4 f_main;

void main() {
    float alpha = texture(u_texture, v_uv).r;
    float shadow_alpha = texture(u_texture, v_uv - vec2(1.0 / 1024.0)).r;

    shadow_alpha = clamp((shadow_alpha - 0.5) / fwidth(shadow_alpha) + 0.5, 0.0, 1.0);
    alpha = clamp((alpha - 0.5) / fwidth(alpha) + 0.5, 0.0, 1.0);

    f_main = vec4(0.0, 0.0, 0.0, 1.0) * shadow_alpha;
    f_main = f_main * (1.0 - alpha) + u_color * alpha;
}
