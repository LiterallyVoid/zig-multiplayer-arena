const std = @import("std");
const linalg = @import("./linalg.zig");
const Model = @import("./Model.zig");
const State = @import("./world/State.zig");

const do = &@import("./debug_overlay.zig").singleton;

const DUPE_PLANE_EPSILON = 1e-3;

pub const Impact = struct {
    time: f32,

    normal: linalg.Vec3,

    brush: ?Brush,
    plane: ?Plane,

    entity: ?State.EntityId = null,
};

pub const Plane = struct {
    vec: linalg.Vec4,

    /// For debugging; this must be a point on this plane.
    origin: linalg.Vec3,

    pub fn heightFromPoint(self: Plane, position: linalg.Vec3) f32 {
        const distance = self.vec.dot(position.xyzw(1.0));
        return distance / self.vec.data[2];
    }

    pub fn heightFromBoxFloor(self: Plane, position: linalg.Vec3, half_extents: linalg.Vec3) f32 {
        const box_distance = self.vec.xyz().abs().dot(half_extents);
        return self.heightFromPoint(position) - box_distance / self.vec.data[2];
    }
};

pub const Brush = struct {
    /// Index into `BrushModel.planes`
    first_plane: u32,

    planes_count: u32,

    /// For debugging.
    origin: linalg.Vec3,

    pub fn debug(self: Brush, bmodel: BrushModel) void {
        for (bmodel.planes[self.first_plane..][0..self.planes_count]) |plane| {
            const color: [4]f32 = .{ 1.0, 0.3, 0.2, 1.0 };
            std.debug.assert(@abs(plane.vec.dot(plane.origin.xyzw(1.0))) < 0.01);
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

            for (0..3) |axis| {
                for (0..2) |negated| {
                    var cardinal = linalg.Vec3.zero();
                    cardinal.data[axis] = if (negated == 1) -1.0 else 1.0;

                    try addCardinalBevelPlane(brush.first_plane, &planes, vertex_positions, cardinal);
                    try addCardinalEdgesBevelPlanes(brush.first_plane, &planes, vertex_positions, cardinal);
                }
            }
        }

        const planes_slice = try planes.toOwnedSlice();

        return .{
            .planes = planes_slice,
            .brushes = brushes,
        };
    }

    fn addCardinalBevelPlane(
        first_plane: u32,
        planes: *std.ArrayList(Plane),
        triangle: [3]linalg.Vec3,
        direction: linalg.Vec3,
    ) !void {
        for (planes.items[first_plane..]) |plane|
            if (plane.vec.xyz().dot(direction) > 1.0 - DUPE_PLANE_EPSILON) return;

        var max_distance = -std.math.inf(f32);
        var origin: linalg.Vec3 = undefined;

        for (triangle) |vertex| {
            const distance = vertex.dot(direction);
            if (distance < max_distance) continue;

            max_distance = distance;
            origin = vertex;
        }

        try planes.append(.{
            .vec = direction.xyzw(-max_distance),
            .origin = origin,
        });
    }

    pub fn addCardinalEdgesBevelPlanes(
        first_plane: u32,
        planes: *std.ArrayList(Plane),
        triangle: [3]linalg.Vec3,
        direction: linalg.Vec3,
    ) !void {
        edges: for (0..3) |i| {
            const vertex_1 = triangle[i];
            const vertex_2 = triangle[(i + 1) % 3];
            const opposing = triangle[(i + 2) % 3];

            const edge = vertex_2.sub(vertex_1);

            var bevel_plane = direction.cross(edge);
            if (bevel_plane.lengthSquared() < 1e-12) {
                continue;
            }

            bevel_plane = bevel_plane.normalized();

            for (planes.items[first_plane..]) |existing_plane|
                if (existing_plane.vec.xyz().dot(bevel_plane) > 1.0 - DUPE_PLANE_EPSILON) continue :edges;

            const distance = vertex_1.dot(bevel_plane);

            if (opposing.dot(bevel_plane) > distance) continue;

            try planes.append(.{
                .vec = bevel_plane.xyzw(-distance),
                .origin = vertex_1.add(vertex_2).mulScalar(0.5),
            });
        }
    }

    pub fn deinit(self: BrushModel, allocator: std.mem.Allocator) void {
        allocator.free(self.planes);
        allocator.free(self.brushes);
    }

    pub fn traceBox(self: BrushModel, origin: linalg.Vec3, direction: linalg.Vec3, half_extents: linalg.Vec3) ?Impact {
        const origin4 = origin.xyzw(1.0);
        const distance4 = direction.xyzw(0.0);

        const ray_length = direction.length();
        const direction4 = distance4.divScalar(ray_length);

        var nearest_impact: ?Impact = null;

        brushes: for (self.brushes) |brush| {
            var entrance: f32 = -std.math.inf(f32);
            var entrance_distance: f32 = 0.0;
            var entrance_bad = true;
            var entrance_plane: Plane = undefined;

            var exit: f32 = 2.0;

            for (
                self.planes[brush.first_plane..][0..brush.planes_count],
            ) |plane| {
                const distance = plane.vec.dot(origin4) - plane.vec.xyz().abs().dot(half_extents);
                const slope = plane.vec.dot(direction4);
                const slope_per_length = plane.vec.dot(distance4);

                const time = distance / -slope_per_length;

                if (slope_per_length < 0.0) {
                    if (time > entrance) {
                        entrance = time - (0.0001 / -slope_per_length);
                        entrance_distance = distance;
                        entrance_bad = distance < -1e-6 * (-slope * 0.5 + 0.5);
                        entrance_plane = plane;
                    }
                } else if (slope_per_length > 0.0) {
                    exit = @min(exit, time);
                } else {
                    if (distance > -1e-6) continue :brushes;
                }
            }

            if (entrance <= 1.0 and exit >= 0.0 and (exit - entrance) > -1e-6 / ray_length) {
                if (nearest_impact) |last_impact| {
                    if (last_impact.time < entrance) continue :brushes;
                }

                nearest_impact = Impact{
                    .time = @max(0.0, entrance),
                    .normal = entrance_plane.vec.xyz(),
                    .brush = brush,
                    .plane = entrance_plane,
                };
            }
        }

        return nearest_impact;
    }

    pub fn traceRay(self: BrushModel, origin: linalg.Vec3, direction: linalg.Vec3) ?Impact {
        return self.traceBox(origin, direction, linalg.Vec3.zero());
    }

    pub fn debug(self: BrushModel) void {
        for (self.brushes) |brush| {
            brush.debug(self);
        }
    }
};
