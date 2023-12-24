const std = @import("std");
const Self = @This();

const Player = @import("./entities/Player.zig");

pub const MAX_ENTITIES = 128;

pub const Entity = union(enum) {
    player: Player,
};

pub const EntitySlot = struct {
    alive: bool = false,
    id: EntityId = .{ .index = 0, .revision = 0 },
    entity: Entity = undefined,
    controller: ?u32 = null,
};

pub const EntityId = struct {
    index: u16,
    revision: u16,

    pub const null_handle = EntityId{
        .index = 0xFFFF,
        .revision = 0xFFFF,
    };
};

entities: [MAX_ENTITIES]EntitySlot = .{.{}} ** MAX_ENTITIES,

pub fn spawn(self: *Self) ?*EntitySlot {
    for (&self.entities, 0..) |*slot, i| {
        if (slot.alive) continue;

        slot.alive = true;
        slot.id.index = @intCast(i);
        slot.id.revision +%= 1;

        if (slot.id.revision == 0) std.log.warn("entity slot {} reused", .{i});

        slot.controller = null;

        return slot;
    }

    return null;
}

pub fn get(self: *Self, id: EntityId) ?*EntitySlot {
    const slot = &self.entities[id.index];
    if (!slot.alive or
        slot.id.index != id.index or
        slot.id.revision != id.revision)
    {
        return null;
    }

    return slot;
}
