#version 330 core

uniform mat4 u_matrix_projectionview;
uniform mat4 u_matrix_model;

layout (location = 0) in vec4 a_position;

void main() {
    gl_Position = u_matrix_projectionview * u_matrix_model * a_position;
}
