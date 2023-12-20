const std = @import("std");
const linalg = @import("./linalg.zig");
const Model = @import("./Model.zig");

const do = &@import("./debug_overlay.zig").singleton;

pub const Impact = struct {
    time: f32,
    normal: linalg.Vec3,
};

pub const Plane = struct {
    vec: linalg.Vec4,

    /// For debugging; this must be a point on this plane.
    origin: linalg.Vec3,
};

pub const Brush = struct {
    /// Index into `BrushModel.planes`
    first_plane: u32,

    planes_count: u32,

    /// For debugging.
    origin: linalg.Vec3,
};

pub const BrushModel = struct {
    planes: []Plane,
    brushes: []Brush,

    pub fn fromModelVertices(allocator: std.mem.Allocator, vertices: []Model.Vertex) !BrushModel {
        var planes = std.ArrayList(Plane).init(allocator);
        errdefer planes.deinit();

        const brushes = try allocator.alloc(Brush, vertices.len / 3);
        errdefer allocator.free(brushes);

        for (brushes, 0..) |*brush, i| {
            const triangle = vertices[i * 3 .. i * 3 + 3];
            const vertex_positions: [3]linalg.Vec3 = .{
                linalg.Vec3.fromArray(f32, triangle[0].position),
                linalg.Vec3.fromArray(f32, triangle[1].position),
                linalg.Vec3.fromArray(f32, triangle[2].position),
            };

            brush.origin = vertex_positions[0]
                .add(vertex_positions[1])
                .add(vertex_positions[2])
                .divScalar(3.0);

            brush.first_plane = @intCast(planes.items.len);
            defer brush.planes_count = @intCast(planes.items.len - brush.first_plane);

            const bext = vertex_positions[1].sub(vertex_positions[0])
                .cross(vertex_positions[2].sub(vertex_positions[0]));

            // Don't ask why it has to be halved, I don't know either. But it works, and at the end of the day that's all I can ask.
            const area = bext.length() / 2.0;
            if (area < 1e-10) {
                std.log.warn("colinear triangle found and eradicated", .{});
            }

            const up = bext.normalized();

            for ([_]linalg.Vec3{ up, up.negate() }) |face| {
                try planes.append(.{
                    .vec = face.xyzw(-face.dot(vertex_positions[0])),
                    .origin = brush.origin,
                });
            }

            for (0..3) |j| {
                const v1 = vertex_positions[j];
                const v2 = vertex_positions[(j + 1) % 3];
                const opp = vertex_positions[(j + 2) % 3];
                _ = opp;

                const edge = v2.sub(v1);

                const out = edge.cross(up).normalized();
                try planes.append(.{
                    .vec = out.xyzw(-out.dot(v1)),
                    .origin = v1.add(v2).divScalar(2.0),
                });
            }
        }

        const planes_slice = try planes.toOwnedSlice();

        return .{
            .planes = planes_slice,
            .brushes = brushes,
        };
    }

    pub fn deinit(self: BrushModel, allocator: std.mem.Allocator) void {
        allocator.free(self.planes);
        allocator.free(self.brushes);
    }

    pub fn traceRay(self: BrushModel, origin: linalg.Vec3, distance: linalg.Vec3) Impact {
        const origin4 = origin.swizzle(linalg.Vec4, "xyz1");
        const distance4 = distance.swizzle(linalg.Vec4, "xyz0");
        _ = distance4;
        _ = origin4;

        for (self.brushes) |brush| {
            for (brush.planes) |plane| {
                _ = plane;
            }
        }
    }

    pub fn debug(self: BrushModel) void {
        for (self.brushes) |brush| {
            for (self.planes[brush.first_plane..][0..brush.planes_count], 0..) |plane, i| {
                var color: [4]f32 = if (i == 0)
                    .{ 1.0, 0.3, 0.3, 1.0 }
                else
                    .{ 1.0, 0.0, 0.0, 1.0 };
                std.debug.assert(@fabs(plane.vec.dot(plane.origin.xyzw(1.0))) < 0.01);
                do.arrow(
                    .world,
                    plane.origin,
                    plane.vec.xyz(),
                    color,
                );
            }
        }
    }
};
