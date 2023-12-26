pub const State = @import("./world/State.zig");
const collision = @import("./collision.zig");
const linalg = @import("./linalg.zig");

pub const World = struct {
    map: *collision.BrushModel,

    pub fn traceBox(
        self: World,
        origin: linalg.Vec3,
        ray: linalg.Vec3,
        half_extents: linalg.Vec3,
    ) ?collision.Impact {
        return self.map.traceBox(origin, ray, half_extents);
    }
};
