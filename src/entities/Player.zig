const std = @import("std");

const game = @import("../game.zig");
const WorldInterface = @import("../world.zig").WorldInterface;
const Entity = @import("../world.zig").State.Entity;
const EntitySlot = @import("../world.zig").State.EntitySlot;

const linalg = @import("../linalg.zig");
const collision = @import("../collision.zig");

const do = &@import("../debug_overlay.zig").singleton;

const Self = @This();

pub const MoveOptions = struct {
    /// steepest angle = acos(floor_max_slope); 0.8 slope ~= 36 degrees
    floor_max_slope: f32 = 0.8,

    half_extents: linalg.Vec3 = linalg.Vec3.new(0.4, 0.4, 1.0),

    /// Basically random, to reduce the chance of mappers building steps that randomly break due to physics/float inaccuracy.
    step_height: f32 = 0.30514,
};

pub const State = enum {
    walk,
    fall,

    fly,
};

origin: linalg.Vec3 = linalg.Vec3.new(0.0, 0.0, 25.0),
velocity: linalg.Vec3 = linalg.Vec3.zero(),
angle: [2]f32 = .{ 0.0, 0.0 },

state: State = .fall,

step_smooth: f32 = 0.0,

command_frame: game.CommandFrame = .{},

refire_timer: f32 = 0.0,

pub fn slot(self: *Self) *EntitySlot {
    return @fieldParentPtr(EntitySlot, "entity", @fieldParentPtr(Entity, "player", self));
}

pub fn interpolate(previous: Self, current: Self, ratio: f32) Self {
    return .{
        .origin = previous.origin.mix(current.origin, ratio),
        .velocity = previous.velocity.mix(current.velocity, ratio),
        .angle = .{
            previous.angle[0] * (1.0 - ratio) + current.angle[0] * ratio,
            previous.angle[1] * (1.0 - ratio) + current.angle[1] * ratio,
        },
        .state = current.state,
        .step_smooth = current.step_smooth,
    };
}

pub fn tick_movement(
    self: *Self,
    world: WorldInterface,
    delta: f32,
) void {
    self.angle[0] += self.command_frame.look[0];
    self.angle[1] += self.command_frame.look[1];

    self.angle[0] = @rem(self.angle[0], std.math.tau);
    self.angle[1] = std.math.clamp(
        self.angle[1],
        -std.math.pi * 0.5,
        std.math.pi * 0.5,
    );

    // if (self.fast) {
    //     speed *= 5.0;
    // }

    // if (self.slow) {
    //     speed /= 2.0;
    // }

    const options = MoveOptions{};

    self.step_smooth *= std.math.exp(-delta * 46.0);

    self.forces(self.command_frame, delta * 0.5);
    self.physics(world, delta);
    self.forces(self.command_frame, delta * 0.5);

    if (self.state == .walk) {
        if (self.command_frame.getImpulse(.jump)) |jump_time| {
            const impulse = 10.0;

            const gravity = -20.0;

            const t = (1.0 - jump_time) * delta;

            self.velocity.data[2] = impulse + t * gravity;

            const distance_to_move_up = impulse * t * 0.5 + gravity * t * t;
            var fake_velocity = linalg.Vec3.new(0.0, 0.0, distance_to_move_up);
            _ = self.flyMove(world, options, &self.origin, &fake_velocity, 1.0);

            self.state = .fall;
        }
    }
}

pub fn tick_weapons(
    self: *Self,
    world: WorldInterface,
    delta: f32,
) void {
    self.refire_timer += delta;

    if (self.refire_timer < 0.5 or self.command_frame.getImpulse(.attack) == null) {
        return;
    }

    self.refire_timer = 0.0;

    const camera = self.origin.add(linalg.Vec3.new(0.0, 0.0, 0.5));

    const basis = self.calculateBasis();

    const forwards = basis.col(0);

    const muzzle_offset = basis.multiplyVector(
        linalg.Vec3.new(0.3, 0.1, 0.5),
    );
    const muzzle = camera.add(muzzle_offset);

    const direction = forwards.mulScalar(50.0);

    if (world.traceRay(
        .{
            .ignore_entity = self.slot().id,
        },
        camera,
        direction,
    )) |impact| {
        world.createEffect(.{
            .kind = .tracer,
            .position = muzzle,
            .direction = direction.mulScalar(impact.time).sub(muzzle_offset),
        });

        world.createEffect(.{
            .kind = .muzzleflash,
            .position = muzzle,
            .direction = direction.normalized(),
        });

        do.arrow(
            .world,
            camera.add(direction.mulScalar(0.05)),
            direction.mulScalar(impact.time - 0.05),
            .{ 0.0, 0.1, 0.5, 1.0 },
        );

        if (impact.entity) |entity_id| {
            std.log.err("{} just shot {}", .{ self.slot().id, entity_id });
            if (world.get(entity_id)) |entity| {
                entity.entity.player.origin = linalg.Vec3.new(0.0, 0.0, 25.0);
                entity.entity.player.velocity = linalg.Vec3.new(0.0, 0.0, 0.0);
            }
        } else {
            std.log.err("{} just shot the floor", .{self.slot().id});
        }
    }
}

pub fn forces(self: *Self, command_frame: game.CommandFrame, delta: f32) void {
    var move_2d = linalg.Vec2.new(
        command_frame.movement[1],
        command_frame.movement[0],
    );

    move_2d = move_2d.rotate(self.angle[0]);
    var move = move_2d.swizzle(linalg.Vec3, "xy0");
    move.data[2] = command_frame.movement[2];

    switch (self.state) {
        .fly => self.flyForces(delta, move, 100.0),
        .walk => {
            self.walkForces(delta, move, 7.0, 80.0, 5.0);
        },
        .fall => self.walkForces(delta, move, 2.0, 20.0, 0.0),
    }
}

pub fn physics(self: *Self, world: WorldInterface, delta: f32) void {
    const options = MoveOptions{};
    switch (self.state) {
        .fly => {
            _ = self.flyMove(world, options, &self.origin, &self.velocity, delta);
        },
        .walk => {
            const move = self.walkMove(world, options, &self.origin, &self.velocity, delta);
            if (move) |walk_info| {
                self.step_smooth += walk_info.step_jump;
            } else {
                self.state = .fall;
                _ = self.flyMove(world, options, &self.origin, &self.velocity, delta);
            }
        },
        .fall => {
            const move = self.flyMove(world, options, &self.origin, &self.velocity, delta);
            if (move.on_ground) {
                self.state = .walk;
            }
        },
    }
}

pub fn flyMove(
    self: *Self,
    world: WorldInterface,
    options: MoveOptions,
    position: *linalg.Vec3,
    velocity: *linalg.Vec3,
    delta: f32,
) struct { on_ground: bool } {
    var on_ground = false;
    const half_extents = options.half_extents;

    var remainder: f32 = delta;

    var normals: [4]linalg.Vec3 = undefined;

    for (0..4) |i| {
        var step = velocity.*.mulScalar(remainder);
        if (world.traceBox(
            .{
                .ignore_entity = self.slot().id,
            },
            position.*,
            step,
            half_extents,
        )) |impact| {
            if (impact.time > 0.0) {
                position.* = position.*.add(step.mulScalar(impact.time));

                remainder *= 1.0 - impact.time;
            }

            const normal = impact.normal;
            if (normal.data[2] > options.floor_max_slope) {
                on_ground = true;
            }

            velocity.* = velocity.*.sub(normal.mulScalar(velocity.*.dot(normal) * 1.005));

            for (normals[0..i]) |previous_normal| {
                if (previous_normal.dot(velocity.*) > 1e-6) continue;
                if (previous_normal.dot(normal) > 1 - 1e-6) continue;
                const edge = previous_normal.cross(normal).normalized();
                velocity.* = edge.mulScalar(edge.dot(velocity.*));
            }

            normals[i] = normal;
        } else {
            position.* = position.*.add(step);
            remainder = 0;
            break;
        }
    }

    if (remainder > 1e-6) {
        std.log.warn("{d}% of movement left after tick", .{remainder / delta * 100});
    }

    return .{
        .on_ground = on_ground,
    };
}

pub fn walkMove(
    self: *Self,
    world: WorldInterface,
    options: MoveOptions,
    position: *linalg.Vec3,
    velocity: *linalg.Vec3,
    delta: f32,
) ?struct {
    step_jump: f32,
} {
    // [position + step_position_offset, half_extents_minus_step] describes a bounding box whose bottom face is moved `step_height` units inwards.
    const step_position_offset = linalg.Vec3.new(0.0, 0.0, options.step_height * 0.5);
    const half_extents_minus_step = options.half_extents.sub(linalg.Vec3.new(0.0, 0.0, options.step_height * 0.5));

    var attempt_position = position.*;
    var attempt_velocity = velocity.*;
    attempt_velocity.data[2] = 0.0;

    // Horizontal collisions.
    var remainder = delta;
    for (0..4) |_| {
        const forwards_target = attempt_velocity.mulScalar(remainder);
        if (world.traceBox(
            .{
                .ignore_entity = self.slot().id,
            },
            attempt_position.add(step_position_offset),
            forwards_target,
            half_extents_minus_step,
        )) |impact| {
            attempt_position = attempt_position.add(forwards_target.mulScalar(impact.time));
            remainder *= (1.0 - impact.time);

            const normal = impact.normal;
            attempt_velocity = attempt_velocity.sub(normal.mulScalar(attempt_velocity.dot(normal) * 1.005));
        } else {
            attempt_position = attempt_position.add(forwards_target);
            remainder = 0.0;
            break;
        }
    }

    var floor_plane: collision.Plane = undefined;

    const step_down_target = -options.step_height * 2.0;
    if (world.traceBox(
        .{
            .ignore_entity = self.slot().id,
        },
        attempt_position.add(step_position_offset),
        linalg.Vec3.new(0.0, 0.0, step_down_target),
        half_extents_minus_step,
    )) |step_down_impact| {
        // We hit something, but it WASN'T GROUND!
        if (step_down_impact.normal.data[2] <= options.floor_max_slope) return null;
        attempt_position.data[2] += step_down_target * step_down_impact.time;

        floor_plane = collision.Plane{
            .vec = step_down_impact.normal.xyzw(0.0),
            .origin = undefined,
        };
    } else {
        // If there isn't a floor below, abort! THAT'S EXACTLY WHAT I WAS AFRAID OF, GEOFF!
        return null;
    }

    if (world.traceBox(
        .{
            .ignore_entity = self.slot().id,
        },
        attempt_position.add(step_position_offset),
        linalg.Vec3.new(0, 0, options.step_height),
        half_extents_minus_step,
    )) |_| {
        return null;
    } else {
        attempt_position.data[2] += options.step_height;
    }

    const step_jump =
        floor_plane.heightFromPoint(linalg.Vec3.new(1.0, 1.0, 0.0).mul(attempt_position)) -
        floor_plane.heightFromPoint(linalg.Vec3.new(1.0, 1.0, 0.0).mul(position.*)) +
        (attempt_position.data[2] - position.data[2]);

    position.* = attempt_position;
    velocity.* = attempt_velocity;

    return .{
        .step_jump = step_jump,
    };
}

pub fn walkForces(self: *Self, delta: f32, move: linalg.Vec3, speed: f32, accel: f32, decel: f32) void {
    self.velocity.data[2] -= 20.0 * delta;

    const velocity_2d = self.velocity.swizzle(linalg.Vec3, "xy0");
    blk: {
        const current_speed = velocity_2d.length();
        if (current_speed == 0.0) break :blk {};

        const target_speed = current_speed * @exp(-delta * decel);

        self.velocity = self.velocity.add(velocity_2d.mulScalar((target_speed - current_speed) / current_speed));
    }

    const move_horizontal = move.mul(linalg.Vec3.new(1.0, 1.0, 0.0));

    if (move_horizontal.length() > 0.0) {
        const dir = move_horizontal.normalized();
        const current_speed = self.velocity.dot(dir);
        const target_speed = @min(move.length(), 1.0) * speed;

        if (current_speed < target_speed) {
            self.velocity = self.velocity.add(dir.mulScalar(@min(target_speed - current_speed, accel * delta)));
        }
    }
}

pub fn flyForces(self: *Self, delta: f32, move: linalg.Vec3, speed: f32) void {
    self.velocity = self.velocity.mulScalar(std.math.exp(-delta * 5.0));
    self.velocity = self.velocity.add(move.mulScalar(delta * speed));
}

pub fn calculateBasis(self: *const Self) linalg.Mat3 {
    return linalg.Mat3.rotation(linalg.Vec3.new(0.0, 1.0, 0.0), self.angle[1])
        .multiply(linalg.Mat3.rotation(linalg.Vec3.new(0.0, 0.0, 1.0), self.angle[0]));
}

pub fn cameraMatrix(self: *const Self) linalg.Mat4 {
    return linalg.Mat4.translationVec(self.origin.add(linalg.Vec3.new(0.0, 0.0, 0.5 - self.step_smooth)))
        .multiply(self.calculateBasis().toMat4());
}
