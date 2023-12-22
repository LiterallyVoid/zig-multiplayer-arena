#version 330 core

#include "stdlib.glsl"

uniform vec4 u_color;
uniform sampler2D u_texture;

in vec2 v_uv;

layout (location = 0) out vec4 f_main;

void main() {
    float alpha = texture(u_texture, v_uv).r;
    float shadow_alpha = texture(u_texture, v_uv - vec2(0.6, 1.3) / 2048.0).r;

    float fw = length(fwidth(v_uv)) * 2048.0 * (20.0 / 255.0);
    
    shadow_alpha = clamp((shadow_alpha - 0.5) / (fw * 4.0) + 0.5, 0.0, 1.0);
    float outline_alpha = clamp((alpha - 0.5) / fw + 1.5, 0.0, 1.0);
    alpha = clamp((alpha - 0.5) / fw + 0.5, 0.0, 1.0);

    f_main = vec4(0.0, 0.0, 0.0, 1.0) * shadow_alpha;
    f_main = mix(f_main, vec4(0.0, 0.0, 0.0, 1.0), outline_alpha);
    f_main = mix(f_main, u_color, alpha);
}
