#version 330 core

uniform mat4 u_matrix_projectionview;
uniform mat4 u_matrix_model;
uniform mat3 u_matrix_model_normal;

layout (location = 0) in vec4 a_position;
layout (location = 1) in vec3 a_normal;

out vec3 v_normal;

void main() {
    gl_Position = u_matrix_projectionview * u_matrix_model * a_position;

    v_normal = u_matrix_model_normal * a_normal;
}
