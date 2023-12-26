const std = @import("std");

pub const CommandFrame = struct {
    pub const ImpulseKind = enum {
        jump,
        attack,
        reload,

        pub const max_enum = std.meta.fields(@This()).len;
    };

    random: u8 = 0,
    movement: [3]f32 = .{ 0.0, 0.0, 0.0 },
    look: [2]f32 = .{ 0.0, 0.0 },
    impulses: [ImpulseKind.max_enum]?f32 = .{null} ** ImpulseKind.max_enum,

    pub fn compose(a: CommandFrame, b: CommandFrame, ratio: f32) CommandFrame {
        var impulses: [ImpulseKind.max_enum]?f32 = undefined;
        for (&impulses, a.impulses, b.impulses) |*impulse, a_impulse, b_impulse| {
            impulse.* = if (a_impulse) |a_time|
                a_time * (1.0 - ratio)
            else if (b_impulse) |b_time|
                (1.0 - ratio) + b_time * ratio
            else
                null;
        }
        return .{
            .random = a.random,

            .movement = .{
                a.movement[0] * (1.0 - ratio) + b.movement[0] * ratio,
                a.movement[1] * (1.0 - ratio) + b.movement[1] * ratio,
                a.movement[2] * (1.0 - ratio) + b.movement[2] * ratio,
            },

            .look = .{
                a.look[0] + b.look[0],
                a.look[1] + b.look[1],
            },

            .impulses = impulses,
        };
    }

    pub fn getImpulse(self: CommandFrame, impulse: ImpulseKind) ?f32 {
        return self.impulses[@intFromEnum(impulse)];
    }
};
