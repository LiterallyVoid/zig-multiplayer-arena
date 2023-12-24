pub const CommandFrame = struct {
    random: u8 = 0,
    movement: [3]f32 = .{ 0.0, 0.0, 0.0 },
    look: [2]f32 = .{ 0.0, 0.0 },
    jump_time: ?f32 = null,

    pub fn compose(a: CommandFrame, b: CommandFrame, ratio: f32) CommandFrame {
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
            .jump_time = if (a.jump_time) |a_jump|
                a_jump * (1.0 - ratio)
            else if (b.jump_time) |b_jump|
                (1.0 - ratio) + b_jump * ratio
            else
                null,
        };
    }
};
