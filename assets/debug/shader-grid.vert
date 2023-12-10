#version 330 core

uniform mat4 u_matrix;

layout (location = 0) in vec4 a_position;
layout (location = 1) in vec3 a_normal;

out vec3 v_position_model;
out vec3 v_normal_model;
out vec3 v_barycentric;

void main() {
    v_position_model = a_position.xyz;
    v_normal_model = a_normal;
    
    v_barycentric = vec3(0.0);
    if (gl_VertexID % 3 == 0) {
        v_barycentric.x = 1.0;
    } else if (gl_VertexID % 3 == 1) {
        v_barycentric.y = 1.0;
    } else if (gl_VertexID % 3 == 2) {
        v_barycentric.z = 1.0;
    }

    gl_Position = u_matrix * a_position;
}
