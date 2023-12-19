#version 330 core

uniform mat4 u_matrix_model;
uniform mat4 u_matrix_projectionview;
uniform mat4 u_bone_matrices[64];

layout (location = 0) in vec4 a_position;
layout (location = 1) in vec3 a_normal;
layout (location = 2) in ivec4 a_bone_indices;
layout (location = 3) in vec4 a_bone_weights;

out vec3 v_position_bind;
out vec3 v_normal_bind;

out vec3 v_position_model;
out vec3 v_normal_model;

out vec3 v_normal_world;

out vec3 v_barycentric;

void main() {
    mat4 skeletal_matrix = mat4(0.0);

    skeletal_matrix += u_bone_matrices[a_bone_indices.x] * a_bone_weights.x;
    skeletal_matrix += u_bone_matrices[a_bone_indices.y] * a_bone_weights.y;
    skeletal_matrix += u_bone_matrices[a_bone_indices.z] * a_bone_weights.z;
    skeletal_matrix += u_bone_matrices[a_bone_indices.w] * a_bone_weights.w;

    skeletal_matrix += mat4(1.0) * (1.0 - dot(a_bone_weights, vec4(1.0)));

    v_position_bind = a_position.xyz;
    v_position_model = (skeletal_matrix * vec4(v_position_bind, 1.0)).xyz;

    v_normal_bind = a_normal;
    v_normal_model = mat3(skeletal_matrix) * v_normal_bind;

    v_normal_world = mat3(u_matrix_model) * v_normal_model;
    
    gl_Position = u_matrix_projectionview * u_matrix_model * vec4(v_position_model, 1.0);

    v_barycentric = vec3(0.0);
    if (gl_VertexID % 3 == 0) {
        v_barycentric.x = 1.0;
    } else if (gl_VertexID % 3 == 1) {
        v_barycentric.y = 1.0;
    } else if (gl_VertexID % 3 == 2) {
        v_barycentric.z = 1.0;
    }
}
