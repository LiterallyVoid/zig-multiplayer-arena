const std = @import("std");
const linalg = @import("./linalg.zig");
const Model = @import("./Model.zig");

const do = &@import("./debug_overlay.zig").singleton;

pub const Impact = struct {
    time: f32,

    brush: Brush,
    plane: Plane,
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
    planes_count_no_bevels: u32,

    /// For debugging.
    origin: linalg.Vec3,

    pub fn debug(self: Brush, bmodel: BrushModel) void {
        for (bmodel.planes[self.first_plane..][0..self.planes_count]) |plane| {
            var color: [4]f32 = .{ 1.0, 0.3, 0.2, 1.0 };
            std.debug.assert(@fabs(plane.vec.dot(plane.origin.xyzw(1.0))) < 0.01);
            do.arrow(
                .world,
                plane.origin,
                plane.vec.xyz(),
                color,
            );
        }
    }
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
            brush.planes_count_no_bevels = 0;
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
            brush.planes_count_no_bevels = @intCast(planes.items.len - brush.first_plane);
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

    pub inline fn trace(self: BrushModel, comptime bevel_planes: bool, origin: linalg.Vec3, direction: linalg.Vec3, half_extents: linalg.Vec3) ?Impact {
        const origin4 = origin.xyzw(1.0);
        const distance4 = direction.xyzw(0.0);

        var nearest_impact: ?Impact = null;

        brushes: for (self.brushes) |brush| {
            var entrance: f32 = -1.0;
            var entrance_plane: Plane = undefined;

            var exit: f32 = 2.0;

            const planes_count = if (bevel_planes) brush.planes_count else brush.planes_count_no_bevels;

            for (self.planes[brush.first_plane..][0..planes_count]) |plane| {
                const distance = plane.vec.dot(origin4) - plane.vec.xyz().abs().dot(half_extents);
                const slope = plane.vec.dot(distance4);

                const time = distance / -slope;

                if (slope < 0.0) {
                    if (time > entrance) {
                        entrance = time;
                        entrance_plane = plane;
                    }
                } else if (slope > 0.0) {
                    exit = @min(exit, time);
                } else {
                    if (distance > 0.0) continue :brushes;
                }
            }

            if (entrance < exit + 1e-6 and entrance > -1e-6) {
                if (nearest_impact) |last_impact| {
                    if (last_impact.time < entrance) continue :brushes;
                }
                nearest_impact = Impact{
                    .time = entrance,
                    .brush = brush,
                    .plane = entrance_plane,
                };
            }
        }

        return nearest_impact;
    }

    pub fn traceRay(self: BrushModel, origin: linalg.Vec3, direction: linalg.Vec3) ?Impact {
        return self.trace(false, origin, direction, linalg.Vec3.zero());
    }

    pub fn traceBox(self: BrushModel, origin: linalg.Vec3, direction: linalg.Vec3, half_extents: linalg.Vec3) ?Impact {
        return self.trace(true, origin, direction, half_extents);
    }

    pub fn debug(self: BrushModel) void {
        for (self.brushes) |brush| {
            brush.debug(self);
        }
    }
};
