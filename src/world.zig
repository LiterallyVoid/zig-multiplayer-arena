pub const State = @import("./world/State.zig");
const collision = @import("./collision.zig");
const linalg = @import("./linalg.zig");
const RingBuffer = @import("./ring_buffer.zig").RingBuffer;

pub const MAX_ROLLBACK_FRAMES = 30;

pub const ClientId = u32;

pub const Hitbox = struct {
    origin: linalg.Vec3,
    half_extents: linalg.Vec3,

    entity: ?State.EntityId,
};

pub const HitboxList = struct {
    hitboxes: [State.MAX_ENTITIES]Hitbox,
    hitboxes_len: u16,
};

pub const TraceFlags = struct {
    ignore_entity: ?State.EntityId = null,
    rollback: ?ClientId = null,
};

pub const EffectKind = enum {
    muzzleflash,
    tracer,
};

pub const Effect = struct {
    kind: EffectKind,

    position: linalg.Vec3,
    direction: ?linalg.Vec3 = null,

    continuity_id: ?u32 = null,
};

pub const WorldInterface = struct {
    map: *collision.BrushModel,

    hitboxes: RingBuffer(Hitbox, MAX_ROLLBACK_FRAMES) = .{},

    /// This `State` contains all entities that may be modified.
    /// For world simulation, this is all entities.
    /// For client prediction, this is all entities controlled by that client.
    accessible_state: *State,

    pub fn get(self: WorldInterface, id: State.EntityId) ?*State.EntitySlot {
        return self.accessible_state.get(id);
    }

    pub fn traceRay(
        self: WorldInterface,
        trace_flags: TraceFlags,
        origin: linalg.Vec3,
        ray: linalg.Vec3,
    ) ?collision.Impact {
        _ = trace_flags;
        return self.map.traceRay(origin, ray);
    }

    pub fn traceBox(
        self: WorldInterface,
        trace_flags: TraceFlags,
        origin: linalg.Vec3,
        ray: linalg.Vec3,
        half_extents: linalg.Vec3,
    ) ?collision.Impact {
        _ = trace_flags;
        return self.map.traceBox(origin, ray, half_extents);
    }

    pub fn createEffect(
        self: WorldInterface,
        effect: Effect,
    ) void {
        _ = effect;
        _ = self;
    }
};
