#version 330 core

uniform mat4 u_matrix_projectionview;
uniform mat4 u_matrix_model;
uniform mat3 u_matrix_model_normal;

layout (location = 0) in vec4 a_position;
layout (location = 4) in vec2 a_uv;
layout (location = 5) in vec4 a_color;

out vec2 v_uv;
out vec4 v_color;

void main() {
    gl_Position = u_matrix_projectionview * u_matrix_model * a_position;

    v_uv = a_uv;
    v_color = a_color;
}
