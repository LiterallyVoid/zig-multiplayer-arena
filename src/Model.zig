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

bones: []Bone,

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
    _ = frames_len;
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

    const bones_slice = try bones.toOwnedSlice(allocator);

    // From here on out, allocation safety has been THROWN OF OUT THE WINDOW. Here be dragons.

    return Self{
        .bones = bones_slice,
    };
}
