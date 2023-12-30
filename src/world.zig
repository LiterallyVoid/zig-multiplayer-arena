const std = @import("std");
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

    fn boxImpact(
        offset: linalg.Vec3,
        velocity: linalg.Vec3,
        half_extents: linalg.Vec3,
    ) ?collision.Impact {
        const center_time = offset.div(velocity).negate();
        const half_time = half_extents.div(velocity).negate();

        const entrance_vec = center_time.sub(half_time.abs());
        const exit_vec = center_time.add(half_time.abs());

        var entrance: f32 = -std.math.inf(f32);
        var entrance_normal: linalg.Vec3 = undefined;
        var exit: f32 = 2.0;

        for (0..3) |axis| {
            if (velocity.data[axis] == 0.0) {
                if (@abs(offset.data[axis]) > half_extents.data[axis] - 1e-6) return null;
                continue;
            }

            if (entrance_vec.data[axis] > entrance) {
                entrance = entrance_vec.data[axis];
                entrance_normal = linalg.Vec3.zero();
                entrance_normal.data[axis] = if (velocity.data[axis] < 0.0) 1.0 else -1.0;
            }

            if (exit_vec.data[axis] < exit) {
                exit = exit_vec.data[axis];
            }
        }

        if (entrance > 1.0 or exit <= 0.0 or exit < entrance - 1e-6) {
            return null;
        }

        return .{
            .time = @max(0.0, entrance),
            .normal = entrance_normal,

            .brush = null,
            .plane = null,
        };
    }

    fn traceEntities(self: WorldInterface, trace_flags: TraceFlags, origin: linalg.Vec3, direction: linalg.Vec3, half_extents: linalg.Vec3) ?collision.Impact {
        var impact: ?collision.Impact = null;
        for (self.accessible_state.entities) |entity_slot| {
            if (!entity_slot.alive) continue;
            if (std.meta.eql(trace_flags.ignore_entity, entity_slot.id)) continue;

            const this_entity_origin = entity_slot.entity.player.origin;
            const this_entity_velocity = entity_slot.entity.player.velocity;
            _ = this_entity_velocity;
            const this_entity_half_extents = linalg.Vec3.new(0.4, 0.4, 1.0);

            var next_impact = boxImpact(
                origin.sub(this_entity_origin),
                direction,
                half_extents.add(this_entity_half_extents),
            ) orelse continue;
            next_impact.entity = entity_slot.id;

            std.log.info("impact {}", .{next_impact});

            const current_impact = impact orelse {
                impact = next_impact;
                continue;
            };

            if (next_impact.time >= current_impact.time) continue;

            impact = next_impact;
        }

        return impact;
    }

    pub fn traceRay(
        self: WorldInterface,
        trace_flags: TraceFlags,
        origin: linalg.Vec3,
        ray: linalg.Vec3,
    ) ?collision.Impact {
        return self.traceBox(trace_flags, origin, ray, linalg.Vec3.zero());
    }

    pub fn traceBox(
        self: WorldInterface,
        trace_flags: TraceFlags,
        origin: linalg.Vec3,
        ray: linalg.Vec3,
        half_extents: linalg.Vec3,
    ) ?collision.Impact {
        const entities_impact = self.traceEntities(trace_flags, origin, ray, half_extents);

        const map_impact = self.map.traceBox(origin, ray, half_extents) orelse return entities_impact;

        const entities_impact_definitely = entities_impact orelse return map_impact;

        if (entities_impact_definitely.time < map_impact.time) {
            return entities_impact_definitely;
        }

        return map_impact;
    }

    pub fn createEffect(
        self: WorldInterface,
        effect: Effect,
    ) void {
        _ = effect;
        _ = self;
    }
};
