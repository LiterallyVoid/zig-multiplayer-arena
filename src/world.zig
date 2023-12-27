pub const State = @import("./world/State.zig");
const collision = @import("./collision.zig");
const linalg = @import("./linalg.zig");

pub const Hitbox = struct {
    origin: linalg.Vec3,
    half_extents: linalg.Vec3,

    entity: ?State.EntityId,
};

pub const World = struct {
    map: *collision.BrushModel,

    hitboxes: []Hitbox = &.{},

    pub fn traceRay(
        self: World,
        origin: linalg.Vec3,
        ray: linalg.Vec3,
    ) ?collision.Impact {
        return self.map.traceRay(origin, ray);
    }

    pub fn traceBox(
        self: World,
        origin: linalg.Vec3,
        ray: linalg.Vec3,
        half_extents: linalg.Vec3,
    ) ?collision.Impact {
        return self.map.traceBox(origin, ray, half_extents);
    }
};
