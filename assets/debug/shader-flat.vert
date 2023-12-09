#version 330 core

uniform mat4 u_matrix;

layout (location = 0) in vec4 a_position;

void main() {
    gl_Position = u_matrix * a_position;
}
