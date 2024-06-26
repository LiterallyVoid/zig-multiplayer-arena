// TODO: split models and animations?

const std = @import("std");
const c = @import("./c.zig");

const linalg = @import("./linalg.zig");

const Self = @This();

pub const Vertex = struct {
    position: [3]f32,
    normal: [3]f32,
    bone_indices: [4]u8,
    bone_weights: [4]u8,
};

pub const Bone = struct {
    name: []const u8,
    parent: ?u16,

    bind_pose_inverse: linalg.Mat4,
};

pub const BonePose = struct {
    translation: linalg.Vec3,
    rotation: linalg.Quaternion,
    scale: linalg.Vec3,
};

pub const Pose = struct {
    bones: []BonePose,

    pub fn deinit(self: Pose, allocator: std.mem.Allocator) void {
        allocator.free(self.bones);
    }

    pub fn fromInterpolation(self: *Pose, a: Pose, b: Pose, ratio: f32) void {
        for (self.bones, a.bones, b.bones) |*self_bone, a_bone, b_bone| {
            self_bone.translation = a_bone.translation.mix(b_bone.translation, ratio);
            self_bone.rotation = a_bone.rotation.slerp(b_bone.rotation, ratio);
            self_bone.scale = a_bone.scale.mix(b_bone.scale, ratio);
        }
    }

    pub fn fromCopy(self: *Pose, from: Pose) void {
        for (self.bones, from.bones) |*self_bone, from_bone| {
            self_bone.* = from_bone;
        }
    }

    pub fn addLayer(self: *Pose, layer: Pose, weight: f32) void {
        for (self.bones, layer.bones) |*self_bone, layer_bone| {
            const rotation = linalg.Quaternion
                .identity()
                .slerp(layer_bone.rotation, weight);

            const scale = linalg.Vec3.new(1.0, 1.0, 1.0).mix(layer_bone.scale, weight);

            self_bone.translation = rotation.mulVector(self_bone.translation)
                .mul(scale)
                .add(layer_bone.translation.mulScalar(weight));

            self_bone.rotation = rotation.mul(self_bone.rotation);

            self_bone.scale = self_bone.scale
                .mul(scale);
        }
    }
};

vertices: []Vertex,
bones: []Bone,
rest_pose: Pose,
frames: []Pose,

gl_vao: c.GLuint = 0,
gl_vbo: c.GLuint = 0,

/// Required, because `vertices` may be freed after upload.
vertices_count: u32 = 0,

pub fn deinit(self: Self, allocator: std.mem.Allocator) void {
    allocator.free(self.vertices);

    for (self.bones) |bone| allocator.free(bone.name);
    allocator.free(self.bones);

    self.rest_pose.deinit(allocator);

    for (self.frames) |frame| frame.deinit(allocator);
    allocator.free(self.frames);
}

pub fn upload(self: *Self) void {
    c.glGenVertexArrays(1, &self.gl_vao);

    c.glBindVertexArray(self.gl_vao);

    c.glGenBuffers(1, &self.gl_vbo);

    c.glBindBuffer(c.GL_ARRAY_BUFFER, self.gl_vbo);
    c.glBufferData(
        c.GL_ARRAY_BUFFER,
        @intCast(self.vertices.len * @sizeOf(Vertex)),
        self.vertices.ptr,
        c.GL_STATIC_DRAW,
    );
    self.vertices_count = @intCast(self.vertices.len);

    c.glEnableVertexAttribArray(0);
    c.glVertexAttribPointer(
        0,
        3,
        c.GL_FLOAT,
        c.GL_FALSE,
        @sizeOf(Vertex),
        @ptrFromInt(@offsetOf(Vertex, "position")),
    );

    c.glEnableVertexAttribArray(1);
    c.glVertexAttribPointer(
        1,
        3,
        c.GL_FLOAT,
        c.GL_FALSE,
        @sizeOf(Vertex),
        @ptrFromInt(@offsetOf(Vertex, "normal")),
    );

    c.glEnableVertexAttribArray(2);
    c.glVertexAttribIPointer(
        2,
        4,
        c.GL_UNSIGNED_BYTE,
        @sizeOf(Vertex),
        @ptrFromInt(@offsetOf(Vertex, "bone_indices")),
    );

    c.glEnableVertexAttribArray(3);
    c.glVertexAttribPointer(
        3,
        4,
        c.GL_UNSIGNED_BYTE,
        c.GL_TRUE,
        @sizeOf(Vertex),
        @ptrFromInt(@offsetOf(Vertex, "bone_weights")),
    );

    c.glBindVertexArray(0);
}

pub fn draw(self: Self) void {
    c.glBindVertexArray(self.gl_vao);
    c.glDrawArrays(c.GL_TRIANGLES, 0, @intCast(self.vertices_count));
}

/// Create a new pose that can be modified. The returned pose must be freed by the caller.
pub fn blankPose(self: Self, allocator: std.mem.Allocator) !Pose {
    return .{
        .bones = try allocator.alloc(BonePose, self.bones.len),
    };
}

/// Return a reference to this model's pose on frame `frame`.
pub fn framePose(self: Self, frame: usize) Pose {
    if (self.frames.len == 0) return self.rest_pose;

    return self.frames[std.math.clamp(frame, 0, self.frames.len - 1)];
}

/// Convert a pose to a list of matrices, to be used by skeletal animation shaders.
pub fn poseMatrices(self: Self, allocator: std.mem.Allocator, pose: Pose) ![]linalg.Mat4 {
    const matrices = try allocator.alloc(linalg.Mat4, self.bones.len);

    for (matrices, self.bones, pose.bones) |*matrix, bone, pose_bone| {
        var matrix_local = linalg.Mat4.translationVec(pose_bone.translation);
        matrix_local = matrix_local.multiply(pose_bone.rotation.toMatrix().toMat4());
        matrix_local = matrix_local.multiply(linalg.Mat4.scaleVec(pose_bone.scale));

        if (bone.parent) |parent_id| {
            matrix.* = matrices[parent_id].multiply(matrix_local);
        } else {
            matrix.* = matrix_local;
        }
    }

    for (matrices, self.bones) |*matrix, bone| {
        matrix.* = matrix.*.multiply(bone.bind_pose_inverse);
    }

    return matrices;
}

pub fn load(allocator: std.mem.Allocator, path: []const u8) !Self {
    const file = try std.fs.cwd().openFile(path, .{});
    var reader = file.reader();

    const magic = try reader.readBytesNoEof(4);
    std.debug.assert(std.mem.eql(u8, &magic, "aMdl"));

    const version = try reader.readInt(u32, .little);
    std.debug.assert(version == 4);

    const vertices_len = try reader.readInt(u32, .little);
    const vertices_pos = try file.getPos() + try reader.readInt(u32, .little);

    const bones_len = try reader.readInt(u32, .little);
    const bones_pos = try file.getPos() + try reader.readInt(u32, .little);
    _ = bones_pos;
    const bind_pose_pos = try file.getPos() + try reader.readInt(u32, .little);
    _ = bind_pose_pos;

    const frames_len = try reader.readInt(u32, .little);
    const bone_frames_pos = try file.getPos() + try reader.readInt(u32, .little);
    _ = bone_frames_pos;

    const vertices = try allocator.alloc(Vertex, vertices_len);
    errdefer allocator.free(vertices);

    try file.seekTo(vertices_pos);
    for (vertices) |*vertex| {
        for (&vertex.position) |*element| {
            element.* = @bitCast(try reader.readInt(u32, .little));
        }

        for (&vertex.normal) |*element| {
            element.* = @bitCast(try reader.readInt(u32, .little));
        }

        for (&vertex.bone_indices) |*element| {
            element.* = try reader.readInt(u8, .little);
        }

        for (&vertex.bone_weights) |*element| {
            element.* = try reader.readInt(u8, .little);
        }
    }

    var bones = try std.ArrayListUnmanaged(Bone).initCapacity(allocator, bones_len);
    errdefer bones.deinit(allocator);
    errdefer for (bones.items) |bone| {
        allocator.free(bone.name);
    };

    for (0..bones_len) |i| {
        const name_len = try reader.readInt(u8, .little);
        const name = try allocator.alloc(u8, name_len);
        errdefer allocator.free(name);

        try reader.readNoEof(name);

        var parent: ?u16 = null;

        const parent_id = try reader.readInt(u16, .little);
        if (parent_id != 0xFF_FF) {
            std.debug.assert(parent_id < i);
            parent = parent_id;
        }

        var bind_pose_inverse: [16]f32 = .{0.0} ** 16;
        for (&bind_pose_inverse) |*element| {
            element.* = @bitCast(try reader.readInt(u32, .little));
        }

        const bone = Bone{
            .name = name,
            .parent = parent,
            .bind_pose_inverse = linalg.Mat4.fromArray(f32, true, bind_pose_inverse),
        };
        bones.appendAssumeCapacity(bone);
    }

    const rest_pose = try loadPose(allocator, reader, bones_len);
    errdefer rest_pose.deinit(allocator);

    var frames = try std.ArrayListUnmanaged(Pose).initCapacity(allocator, frames_len);
    errdefer frames.deinit(allocator);
    errdefer for (frames.items) |frame| frame.deinit(allocator);

    for (0..frames_len) |_| {
        frames.appendAssumeCapacity(try loadPose(allocator, reader, bones_len));
    }

    // From here on out, allocation safety has been THROWN OF OUT THE WINDOW because if an `errdefer` executes, `bones_slice` will be potentially double-freed. Here be dragons. (I don't hate zig I don't hate zig I don't hate zig I don't hate zig)
    const bones_slice = try bones.toOwnedSlice(allocator);
    const frames_slice = try frames.toOwnedSlice(allocator);

    return Self{
        .vertices = vertices,
        .bones = bones_slice,
        .rest_pose = rest_pose,
        .frames = frames_slice,
    };
}

fn loadPose(allocator: std.mem.Allocator, reader: anytype, bones_count: usize) !Pose {
    const bones = try allocator.alloc(BonePose, bones_count);
    errdefer allocator.free(bones);

    for (bones) |*bone| {
        var translation: [3]f32 = undefined;
        var rotation: [4]f32 = undefined;
        var scale: [3]f32 = undefined;

        for (&translation) |*element| {
            element.* = @bitCast(try reader.readInt(u32, .little));
        }

        for (&rotation) |*element| {
            element.* = @bitCast(try reader.readInt(u32, .little));
        }

        for (&scale) |*element| {
            element.* = @bitCast(try reader.readInt(u32, .little));
        }

        bone.* = .{
            .translation = linalg.Vec3.fromArray(f32, translation),
            .rotation = linalg.Quaternion.fromArray(f32, rotation),
            .scale = linalg.Vec3.fromArray(f32, scale),
        };
    }

    return Pose{
        .bones = bones,
    };
}
