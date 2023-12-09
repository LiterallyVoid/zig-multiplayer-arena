const std = @import("std");
const c = @import("./c.zig");

const linalg = @import("./linalg.zig");

pub const Vertex = struct {
    position: [3]f32,
    normal: [3]f32,
};

pub const Bone = struct {
    name: []const u8,
    bind_pose_inverse: linalg.Mat4,
};

pub const BonePose = struct {
    translation: linalg.Vec3,
    rotation: linalg.Quaternion,
    scale: linalg.Vec3,
};

pub const Pose = struct {
    bones: []BonePose,
};

bones: []Bone,
rest_pose: Pose,
frames: []Pose,

const Self = @This();

pub fn load(allocator: std.mem.Allocator, path: []const u8) !Self {
    const file = try std.fs.cwd().openFile(path, .{});
    var reader = file.reader();

    const magic = try reader.readBytesNoEof(4);
    std.debug.assert(std.mem.eql(u8, &magic, "aMdl"));

    const version = try reader.readIntLittle(u32);
    std.debug.assert(version == 2);

    const vertices_len = try reader.readIntLittle(u32);
    const vertices_pos = try file.getPos() + try reader.readIntLittle(u32);

    const bones_len = try reader.readIntLittle(u32);
    const bones_pos = try file.getPos() + try reader.readIntLittle(u32);
    _ = bones_pos;
    const bind_pose_pos = try file.getPos() + try reader.readIntLittle(u32);
    _ = bind_pose_pos;

    const frames_len = try reader.readIntLittle(u32);
    const bone_frames_pos = try file.getPos() + try reader.readIntLittle(u32);
    _ = bone_frames_pos;

    const vertices = try allocator.alloc(Vertex, vertices_len);
    defer allocator.free(vertices);

    try file.seekTo(vertices_pos);
    for (vertices) |*vertex| {
        for (&vertex.position) |*element| {
            element.* = @bitCast(try reader.readIntLittle(u32));
        }

        for (&vertex.normal) |*element| {
            element.* = @bitCast(try reader.readIntLittle(u32));
        }
    }

    var bones = try std.ArrayListUnmanaged(Bone).initCapacity(allocator, bones_len);
    errdefer bones.deinit(allocator);
    errdefer for (bones.items) |bone| {
        allocator.free(bone.name);
    };

    for (0..bones_len) |_| {
        const name_len = try reader.readIntLittle(u8);
        const name = try allocator.alloc(u8, name_len);
        errdefer allocator.free(name);

        try reader.readNoEof(name);

        var bind_pose_inverse: [16]f32 = .{0.0} ** 16;
        for (&bind_pose_inverse) |*element| {
            element.* = @bitCast(try reader.readIntLittle(u32));
        }

        const bone = Bone{
            .name = name,
            .bind_pose_inverse = linalg.Mat4.fromArray(f32, true, bind_pose_inverse),
        };
        bones.appendAssumeCapacity(bone);
    }

    const rest_pose = try loadPose(allocator, reader, bones_len);

    var frames = try std.ArrayListUnmanaged(Pose).initCapacity(allocator, frames_len);
    errdefer frames.deinit(allocator);
    errdefer for (frames.items) |frame| allocator.free(frame.bones);

    for (0..frames_len) |_| {
        frames.appendAssumeCapacity(try loadPose(allocator, reader, bones_len));
    }

    // From here on out, allocation safety has been THROWN OF OUT THE WINDOW because if an `errdefer` executes, `bones_slice` will be potentially double-freed. Here be dragons. (I don't hate zig I don't hate zig I don't hate zig I don't hate zig)
    const bones_slice = try bones.toOwnedSlice(allocator);
    const frames_slice = try frames.toOwnedSlice(allocator);

    return Self{
        .bones = bones_slice,
        .rest_pose = rest_pose,
        .frames = frames_slice,
    };
}

fn loadPose(allocator: std.mem.Allocator, reader: anytype, bones_count: usize) !Pose {
    var bones = try allocator.alloc(BonePose, bones_count);
    defer allocator.free(bones);

    for (bones) |*bone| {
        var translation: [3]f32 = undefined;
        var rotation: [4]f32 = undefined;
        var scale: [3]f32 = undefined;

        for (&translation) |*element| {
            element.* = @bitCast(try reader.readIntLittle(u32));
        }

        for (&rotation) |*element| {
            element.* = @bitCast(try reader.readIntLittle(u32));
        }

        for (&scale) |*element| {
            element.* = @bitCast(try reader.readIntLittle(u32));
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
