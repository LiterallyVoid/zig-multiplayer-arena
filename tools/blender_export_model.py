# -*- tab-width: 4 python-indent-offset: 4 indent-tabs-mode: t -*-

import bpy
import mathutils
import struct
import sys
from collections import namedtuple

class Bone(namedtuple("Bone", ["id", "parent_id", "armature", "pose_bone"])):
	pass

depsgraph = bpy.context.evaluated_depsgraph_get()

vertices = []
armatures = set()
bones_by_id = []
bones_by_pose_bone = {}

def pack_pose(pose, reference = None):
	if reference is None:
		reference = [mathutils.Matrix() for _ in pose]

	bones = []

	for id, (bone, pose_matrix, reference_matrix) in enumerate(zip(bones_by_id, pose, reference)):
		if (parent := bone.parent_id) is not None:
			pose_matrix = pose[parent].inverted() @ pose_matrix
			reference_matrix = reference[parent].inverted() @ reference_matrix

		matrix = pose_matrix @ reference_matrix.inverted()

		translation, rotation, scale = matrix.decompose()

		bones.append(struct.pack('<3f4f3f', *translation, rotation.x, rotation.y, rotation.z, rotation.w, *scale))

	return b''.join(bones)

def find_bones(object):
	object = object.evaluated_get(depsgraph)

	def bone_id(pose_bone):
		if pose_bone in bones_by_pose_bone:
			return bones_by_pose_bone[pose_bone].id

		parent_id = None
		if pose_bone.parent:
			# Make sure this bones' parent is exported first
			parent_id = bone_id(pose_bone.parent)

		id = len(bones_by_id)

		bone = Bone(id, parent_id, object, pose_bone)

		bones_by_id.append(bone)
		bones_by_pose_bone[pose_bone] = bone

	armatures.add(object)

	for bone in object.pose.bones:
		_ = bone_id(bone)

def pack_vertex(position, normal, bone_indices, bone_weights):
	bone_weights = [int(weight * 255) for weight in bone_weights]
	return struct.pack('<3f3f4B4B', *position, *normal, *bone_indices, *bone_weights)

def export_mesh(object):
	object = object.evaluated_get(depsgraph)
	matrix = object.matrix_world
	mesh = object.data

	if hasattr(mesh, "calc_normals_split"):
		mesh.calc_normals_split()

	mesh.calc_tangents()

	armature = None
	
	if object.parent and object.parent.type == 'ARMATURE':
		armature = object.parent

	for polygon in mesh.polygons:
		polygon_vertices = []

		for loop_index in polygon.loop_indices:
			loop = mesh.loops[loop_index]
			vertex = mesh.vertices[loop.vertex_index]

			# Placeholder indices and weights.
			bone_indices_and_weights = [(0, 0.0)] * 4

			for vertex_group in vertex.groups:
				try:
					object_group = object.vertex_groups[vertex_group.group]
				except:
					continue

				bone = bones_by_pose_bone[armature.pose.bones[object_group.name]]
				bone_indices_and_weights.append([bone.id, vertex_group.weight])

			bone_indices_and_weights.sort(key = lambda bone: bone[1], reverse = True)
			bone_indices_and_weights = bone_indices_and_weights[:4]

			bone_indices, bone_weights = zip(*bone_indices_and_weights)

			# Normalize bone weights.
			total = sum(bone_weights)
			if total > 0.0:
				bone_weights = [weight / total for weight in bone_weights]

			polygon_vertices.append(pack_vertex(
				matrix @ vertex.co,
				matrix.to_3x3() @ loop.normal,
				bone_indices,
				bone_weights,
			))

		for edge in zip(polygon_vertices[1:], polygon_vertices[2:]):
			vertices.append(polygon_vertices[0])
			vertices.extend(edge)

def scene_pose():
	matrices = []
	for id, bone in enumerate(bones_by_id):
		matrices.append(bone.armature.matrix_world @ bone.pose_bone.matrix)

	return matrices

def export_scene(scene):
	for object in scene.collection.all_objects:
		if object.type == 'ARMATURE': find_bones(object)

	frame_poses = []

	for frame in range(0, bpy.context.scene.frame_end):
		bpy.context.scene.frame_set(frame)
		frame_poses.append(scene_pose())

	for armature in armatures:
		armature.data.pose_position = 'REST'

	# Update the depsgraph, I guess?? lol
	bpy.context.scene.frame_set(0)

	bind_pose = scene_pose()

	depsgraph.update()

	for object in scene.collection.all_objects:
		if not object.visible_get(): continue
		if object.type == 'MESH': export_mesh(object)

	return bind_pose, frame_poses 

bind_pose, frame_poses = export_scene(bpy.context.scene)
rest_pose = bind_pose

FILE_MAGIC = b"aMdl"
FILE_VERSION = 4

def patchable_pointer(file):
	position = file.tell()
	file.write(struct.pack('<I', 0))
	return (file, position)

def patch_pointer(patch, value):
	file, position = patch
	saved_position = file.tell()

	file.seek(position)
	file.write(struct.pack('<I', value))

	file.seek(saved_position)

def patch_pointer_to_cursor(patch):
	file, position = patch
	cursor = file.tell()
	
	patch_pointer(patch, cursor - position)

args_after_double_dash = sys.argv[sys.argv.index("--") + 1:]
assert args_after_double_dash[0] == "-o"
export_path = args_after_double_dash[1]

with open(export_path, "wb") as file:
	file.write(struct.pack('<4sI', FILE_MAGIC, FILE_VERSION))
	file.write(struct.pack('<I', len(vertices)))
	vertices_ptr = patchable_pointer(file)

	file.write(struct.pack('<I', len(bones_by_id)))
	bones_ptr = patchable_pointer(file)
	rest_pose_ptr = patchable_pointer(file)

	file.write(struct.pack('<I', len(frame_poses)))
	bone_frames_ptr = patchable_pointer(file)

	patch_pointer_to_cursor(vertices_ptr)
	for vertex in vertices:
		file.write(vertex)

	patch_pointer_to_cursor(bones_ptr)
	for bone, bind_matrix in zip(bones_by_id, bind_pose):
		# FIXME: the same bone in multiple armatures will have the same name!
		name = bone.pose_bone.name
		name_bytes = name.encode("utf-8")

		file.write(struct.pack('<B', len(name_bytes)))
		file.write(name_bytes)

		if bone.parent_id is None:
			file.write(struct.pack('<H', 0xFF_FF))
		else:
			file.write(struct.pack('<H', bone.parent_id))

		file.write(struct.pack('<16f', *(elem for row in bind_matrix.inverted().transposed() for elem in row)))

	patch_pointer_to_cursor(rest_pose_ptr)
	file.write(pack_pose(rest_pose, None))

	patch_pointer_to_cursor(bone_frames_ptr)
	for frame_pose in frame_poses:
		file.write(pack_pose(frame_pose, rest_pose))
