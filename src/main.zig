const std = @import("std");

const c = @import("./c.zig");
pub const linalg = @import("./linalg.zig");
pub const Shader = @import("./Shader.zig");
pub const Model = @import("./Model.zig");
pub const Font = @import("./Font.zig");
pub const collision = @import("./collision.zig");
pub const debug_overlay = @import("./debug_overlay.zig");
const do = &debug_overlay.singleton;

pub const ActionId = enum {
    attack1,
    attack2,

    forward,
    back,
    left,
    right,
    jump,
    crouch,

    fly_fast,
    fly_slow,

    debug1,
    debug2,
    debug3,
    debug4,

    pub const max_enum = std.meta.fields(ActionId).len;
};

pub const Event = union(enum) {
    mouse_motion: struct {
        relative: [2]f32,
        absolute: [2]f32,
    },

    action: struct {
        id: ActionId,
        pressed: bool,
        value: f32,
    },
};

/// THIS IS LEGACY CODE NEVER USE IT AGAIN
pub const Flycam = struct {
    origin: linalg.Vec3 = linalg.Vec3.new(0.0, 0.0, 5.0),
    velocity: linalg.Vec3 = linalg.Vec3.zero(),
    angle: [2]f32 = .{ 0.0, 0.0 },

    actions: [6]f32 = .{0.0} ** 6,
    fast: bool = false,
    slow: bool = false,
    fly: bool = false,

    pub fn update(self: *Flycam, delta: f32) void {
        const basis = self.calculateBasis();

        const forward = basis.multiplyVectorOpp(linalg.Vec3.new(1.0, 0.0, 0.0));
        const right = basis.multiplyVectorOpp(linalg.Vec3.new(0.0, 1.0, 0.0));
        const up = linalg.Vec3.new(0.0, 0.0, 1.0);

        self.velocity = self.velocity.mulScalar(std.math.exp(-delta * 20.0));

        var speed: f32 = 150.0;

        if (self.fast) {
            speed *= 10.0;
        }

        if (self.slow) {
            speed /= 3.0;
        }

        self.velocity = self.velocity.add(forward.mulScalar(delta * speed * (self.actions[0] - self.actions[1])));
        self.velocity = self.velocity.add(right.mulScalar(delta * speed * (self.actions[2] - self.actions[3])));
        self.velocity = self.velocity.add(up.mulScalar(delta * speed * (self.actions[4] - self.actions[5])));

        self.origin = self.origin.add(self.velocity.mulScalar(delta));
    }

    pub fn calculateBasis(self: *const Flycam) linalg.Mat3 {
        return linalg.Mat3.rotation(linalg.Vec3.new(0.0, 1.0, 0.0), self.angle[1])
            .multiply(linalg.Mat3.rotation(linalg.Vec3.new(0.0, 0.0, 1.0), self.angle[0]));
    }

    pub fn cameraMatrix(self: *const Flycam) linalg.Mat4 {
        return linalg.Mat4.translationVec(self.origin)
            .multiply(self.calculateBasis().toMat4());
    }

    pub fn handleEvent(self: *Flycam, event: Event) bool {
        switch (event) {
            .mouse_motion => |mouse_motion| {
                self.angle[0] += mouse_motion.relative[0] * 0.003;
                self.angle[1] += mouse_motion.relative[1] * 0.003;

                self.angle[0] = @rem(self.angle[0], std.math.tau);
                self.angle[1] = std.math.clamp(self.angle[1], -std.math.pi * 0.5, std.math.pi * 0.5);

                return true;
            },

            .action => |action| {
                switch (action.id) {
                    .forward => self.actions[0] = action.value,
                    .back => self.actions[1] = action.value,
                    .left => self.actions[2] = action.value,
                    .right => self.actions[3] = action.value,
                    .jump => self.actions[4] = action.value,
                    .crouch => self.actions[5] = action.value,

                    .fly_fast => self.fast = action.pressed,
                    .fly_slow => self.slow = action.pressed,

                    else => return false,
                }

                return true;
            },
        }

        return false;
    }
};

pub const EventTranslator = struct {
    last_mouse_position: [2]f32 = .{ 0.0, 0.0 },

    pub fn translateMousePosition(self: *EventTranslator, absolute_: [2]f32) Event {
        // TODO: use window size
        var absolute = absolute_;
        absolute[1] = -absolute[1];

        const relative = [2]f32{
            absolute[0] - self.last_mouse_position[0],
            absolute[1] - self.last_mouse_position[1],
        };
        self.last_mouse_position = absolute;
        return Event{
            .mouse_motion = .{
                .absolute = absolute,
                .relative = relative,
            },
        };
    }
};

pub const CommandFrame = struct {
    movement: [3]f32 = .{ 0.0, 0.0, 0.0 },
    look: [2]f32 = .{ 0.0, 0.0 },
    jump_time: ?f32 = null,

    pub fn compose(a: CommandFrame, b: CommandFrame, ratio: f32) CommandFrame {
        return .{
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

pub const InputBundler = struct {
    latest_command_frame: CommandFrame,
    latest_command_frame_fraction: f32 = 0.0,

    pub fn init() InputBundler {
        return .{
            .latest_command_frame = .{},
        };
    }

    pub fn integrate(self: *InputBundler, frame: CommandFrame, length: f32) void {
        var ratio: f32 = 0.0;
        if (length > 0.0) {
            ratio = length / (length + self.latest_command_frame_fraction);
        }

        self.latest_command_frame = self.latest_command_frame.compose(frame, ratio);
        self.latest_command_frame_fraction += length;
    }

    pub fn update(self: *InputBundler, app: *const App, slice_length: f32) void {
        // TODO: These should probably take angle into account now
        const slice_frame = .{
            .movement = .{
                app.actions[@intFromEnum(ActionId.forward)] -
                    app.actions[@intFromEnum(ActionId.back)],

                app.actions[@intFromEnum(ActionId.left)] -
                    app.actions[@intFromEnum(ActionId.right)],

                app.actions[@intFromEnum(ActionId.jump)] -
                    app.actions[@intFromEnum(ActionId.crouch)],
            },
            .jump_time = if (app.actions[@intFromEnum(ActionId.jump)] > 0.5)
                @as(f32, 0.0)
            else
                null,
        };

        self.integrate(slice_frame, slice_length);
    }

    pub fn handleEvent(self: *InputBundler, app: *const App, event: Event) bool {
        _ = app;
        switch (event) {
            .mouse_motion => |motion| {
                const frame = .{
                    .look = .{
                        motion.relative[0] * 0.003,
                        motion.relative[1] * 0.003,
                    },
                };

                self.integrate(frame, 0.0);

                return true;
            },
            .action => |action| switch (action.id) {
                .forward,
                .back,
                .left,
                .right,
                .jump,
                .crouch,
                => return true,
                else => return false,
            },
        }

        return false;
    }

    pub fn partial(self: *InputBundler) CommandFrame {
        return self.latest_command_frame;
    }

    pub fn commit(self: *InputBundler) CommandFrame {
        const frame = self.latest_command_frame;

        self.latest_command_frame = .{};
        self.latest_command_frame_fraction = 0.0;

        return frame;
    }
};

pub const NetChannel = struct {
    const MAX_MESSAGE_LENGTH = 2048;

    tcp_stream: std.net.Stream,
    buffer: [MAX_MESSAGE_LENGTH]u8 = undefined,
    buffer_length: usize = 0,

    pub fn send(self: NetChannel, message: []const u8) void {
        var length = std.mem.writeIntLittle(u16, @intCast(message.len));
        self.tcp_stream.write(length);
        self.tcp_stream.write(message);
    }

    pub fn poll(self: *NetChannel, buffer: *[MAX_MESSAGE_LENGTH]u8) ?[]u8 {
        if (self.buffer_length < self.buffer.len) {
            const amt = self.tcp_stream.read(self.buffer[self.buffer_length..]) catch |err| {
                std.log.err("got error while reading from socket: {}", .{err});
                return null;
            };
            self.buffer_length += amt;
        }

        if (self.buffer_length >= 2) {
            const packet_length = std.mem.readIntLittle(u16, self.buffer[0..2]);

            if (self.buffer_length < packet_length + 2) {
                return null;
            }

            std.mem.copy(u8, buffer[0..packet_length], self.buffer[2 .. packet_length + 2]);

            std.mem.copyForwards(u8, &self.buffer, self.buffer[packet_length + 2 ..]);

            return buffer[0..packet_length];
        }

        return null;
    }

    pub fn sendTyped(self: NetChannel, comptime T: type, message: T) void {
        var buffer: [MAX_MESSAGE_LENGTH]u8 = undefined;
        var writer = std.io.fixedBufferStream(&buffer);
        const encoded = std.json.stringify(message, .{}, writer) catch unreachable;

        std.log.info("sending JSON packet: {s}", .{encoded});

        self.send(encoded);
    }

    pub fn pollTyped(self: *NetChannel, comptime T: type, allocator: std.mem.Allocator) ?T {
        var buffer: [MAX_MESSAGE_LENGTH]u8 = undefined;
        const bytes = self.poll(&buffer) orelse return null;

        std.log.info("received JSON packet: {s}", .{bytes});

        return std.json.parseFromSliceLeaky(T, allocator, bytes, .{}) catch unreachable;
    }
};

pub fn RingBuffer(comptime T: type, comptime limit: u32) type {
    return struct {
        const Self = @This();

        items: [limit]T = undefined,

        length: u32 = 0,
        insert_head: u32 = 0,

        pub fn push(self: *Self, item: T) void {
            self.insert_head = (self.insert_head + 1) % limit;
            self.items[self.insert_head] = item;

            if (self.length < limit) {
                self.length += 1;
            }
        }

        pub fn pop(self: *Self) ?T {
            if (self.length == 0) return null;

            self.length -= 1;
            return self.items[(self.insert_head + limit - self.length) % limit];
        }

        pub fn peek(self: *Self, index: u32) ?T {
            if (index < 0 or index >= self.length) return null;
            const wrapped_index =
                (self.insert_head + limit - self.length + index + 1) %
                limit;
            return self.items[wrapped_index];
        }
    };
}

pub const Entity = union(enum) {
    player: Walkcam,
};

pub const WORLD_MAX_ENTITIES = 128;
pub const EntitySlot = struct {
    alive: bool = false,
    id: EntityId = .{ .index = 0, .revision = 0 },
    entity: Entity = undefined,
    controller: ?u32 = null,
};

pub const EntityId = struct {
    index: u16,
    revision: u16,
};

pub const WorldState = struct {
    entities: [WORLD_MAX_ENTITIES]EntitySlot = .{.{}} ** WORLD_MAX_ENTITIES,

    pub fn spawn(self: *WorldState) ?*EntitySlot {
        for (&self.entities, 0..) |*slot, i| {
            if (slot.alive) continue;

            slot.alive = true;
            slot.id.id = @intCast(i);
            slot.id.revision +%= 1;

            if (slot.revision == 0) std.log.warn("entity slot {} reused", .{i});

            return slot;
        }

        return null;
    }

    pub fn get(self: *WorldState, id: EntityId) ?*EntitySlot {
        const slot = &self.entities[id.index];
        if (!slot.alive or
            slot.id.index != id.index or
            slot.id.revision != id.revision)
        {
            return null;
        }

        return slot;
    }
};

/// A server's idea of what a client is.
pub const ServerClient = struct {
    id: u32,

    channel: NetChannel,

    entity: EntityId,
    command_queue: RingBuffer(CommandFrame, 8),

    /// The world state that's currently in flight to the client.
    world_state_pending: WorldState,
};

/// All commands that the server can send to the client.
pub const ServerMessage = union(enum) {
    hello: struct {
        your_id: u32,
        tick_length: f32,
    },
    entity_refresh: struct {
        slot: u32,
        data: Entity,
    },
    tick_report: struct {
        command_queue_length: u32,
        time_since_receiving_command: f32,
    },
};

pub const Server = struct {
    allocator: std.mem.Allocator,

    map: *collision.BrushModel,

    world: WorldState = .{},
    clients: std.ArrayListUnmanaged(ServerClient) = .{},

    latest_client_id: u32 = 0,

    tick_length: f32 = 1.0 / 20.0,

    pub fn init(allocator: std.mem.Allocator, map: *collision.BrushModel) Server {
        return Server{
            .allocator = allocator,
            .map = map,
        };
    }

    pub fn handleConnect(self: *Server, channel: NetChannel) !void {
        const id = self.latest_client_id;
        self.latest_client_id += 1;

        const entity = self.world.spawn().?;
        entity.entity = .{ .player = .{} };

        try self.clients.append(self.allocator, .{
            .id = id,
            .channel = channel,
            .entity = entity.id,
            .command_queue = .{},
            .world_state_pending = .{},
        });
    }

    fn handleMessage(self: *Server, client: *ServerClient, message: ClientMessage) void {
        _ = self;
        switch (message) {
            .tick => |tick_info| {
                client.command_queue.push(tick_info.command_frame);
            },
        }
    }

    pub fn tick(self: *Server) void {
        for (self.clients.items) |*client| {
            while (client.channel.pollTyped(ClientMessage, self.allocator)) |message| {
                self.handleMessage(client, message);
            }
        }
        for (self.clients.items) |*client| {
            const command_frame = client.command_queue.pop() orelse CommandFrame{};
            const entity = self.world.get(client.entity) orelse {
                std.log.warn("client has no entity?", .{});

                continue;
            };

            switch (entity.entity) {
                .player => |*player| player.update(self.tick_length, command_frame),
            }
        }
    }
};

/// All commands that the client can send to the server.
pub const ClientMessage = union(enum) {
    tick: struct {
        command_frame: CommandFrame,
    },
};

pub const Client = struct {
    pub const CommandQueueHealthFrame = struct {
        queued_ticks: f32,
    };

    /// Must be synced to server's ID of self
    id: u32 = 0,

    channel: NetChannel,

    interpolation_queue: RingBuffer(WorldState, 8) = .{},
    latest_world_state: WorldState = .{},

    /// Must be as long as server<->client round-trip latency.
    command_queue: RingBuffer(CommandFrame, 128) = .{},
    prediction: std.ArrayListUnmanaged(WorldState) = .{},
    prediction_dirty: bool = true,

    server_command_queue_health_debug: RingBuffer(CommandQueueHealthFrame, 60) = .{},

    tick_length: f32 = 1.0 / 20.0,

    pub fn tick(self: *Client, frame: CommandFrame) void {
        self.command_queue.push(frame);
    }

    pub fn predict(self: *Client, allocator: std.mem.Allocator) !void {
        if (self.prediction_dirty) {
            self.prediction.clearRetainingCapacity();
        }

        var accumulator = self.prediction.getLastOrNull() orelse
            self.latest_world_state;

        for (accumulator.entities) |*entity| {
            const ent_controller = entity.controller orelse {
                entity.alive = false;
                continue;
            };

            if (ent_controller != self.id) {
                entity.alive = false;
            }
        }

        while (self.prediction.items.len < self.command_queue.length) {
            const index: u32 = @intCast(self.prediction.items.len);
            const command_frame = self.command_queue.peek(index).?;

            for (accumulator.entities) |*entity| {
                if (!entity.alive) continue;
                switch (entity.entity) {
                    .player => |player| player.update(self.tick_length, command_frame),
                }
            }

            try self.prediction.append(allocator, accumulator);
        }

        return accumulator;
    }

    pub fn handleMessage(self: *Client, message: ServerMessage) void {
        switch (message) {
            .hello => |hello| {
                self.id = hello.your_id;
                self.tick_length = hello.tick_length;
            },
            .entity_refresh => |refresh| {
                self.latest_world_state.entities[refresh.slot] = refresh.data;
            },
            .tick_report => |report| {
                self.command_queue_health_debug.push(.{
                    .queued_ticks = @as(f32, @floatFromInt(report.command_queue_length)) -
                        report.time_since_receiving_command / self.tick_rate,
                });

                while (self.command_queue.length > report.command_queue_length) {
                    _ = self.command_queue.pop();
                }

                self.prediction_dirty = true;
            },
        }
    }

    pub fn send(self: *Client, message: ClientMessage) void {
        _ = self;
        std.log.info("client send {}", message);
    }
};

pub const Walkcam = struct {
    pub const MoveOptions = struct {
        /// steepest angle = acos(floor_max_slope); 0.8 slope ~= 36 degrees
        floor_max_slope: f32 = 0.8,

        half_extents: linalg.Vec3 = linalg.Vec3.new(0.3, 0.3, 1.0),

        /// Basically random, to reduce the chance of steps breaking due to physics/float inaccuracy.
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

    map_bmodel: *collision.BrushModel,

    state: State = .fall,

    step_smooth: f32 = 0.0,

    pub fn update(self: *Walkcam, delta: f32, command_frame: CommandFrame) void {
        self.angle[0] += command_frame.look[0];
        self.angle[1] += command_frame.look[1];

        self.angle[0] = @rem(self.angle[0], std.math.tau);
        self.angle[1] = std.math.clamp(
            self.angle[1],
            -std.math.pi * 0.5,
            std.math.pi * 0.5,
        );

        var speed: f32 = 10.0;
        _ = speed;

        // if (self.fast) {
        //     speed *= 5.0;
        // }

        // if (self.slow) {
        //     speed /= 2.0;
        // }

        const options = MoveOptions{};
        _ = options;

        self.step_smooth *= std.math.exp(-delta * 18.0);

        self.forces(command_frame, delta * 0.5);
        self.physics(delta);
        self.forces(command_frame, delta * 0.5);
    }

    pub fn forces(self: *Walkcam, command_frame: CommandFrame, delta: f32) void {
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
                self.walkForces(delta, move, 10.0, 200.0, 100.0);

                // TODO: subtick jump
                if (command_frame.jump_time) |jump_time| {
                    _ = jump_time;
                    self.state = .fall;
                    self.velocity.data[2] = 10.0;
                }
            },
            .fall => self.walkForces(delta, move, 2.0, 50.0, 0.0),
        }
    }

    pub fn physics(self: *Walkcam, delta: f32) void {
        const options = MoveOptions{};
        switch (self.state) {
            .fly => {
                _ = flyMove(self.map_bmodel, options, &self.origin, &self.velocity, delta);
            },
            .walk => {
                const move = walkMove(self.map_bmodel, options, &self.origin, &self.velocity, delta);
                if (move) |walk_info| {
                    self.step_smooth += walk_info.step_jump;
                } else {
                    self.state = .fall;
                    _ = flyMove(self.map_bmodel, options, &self.origin, &self.velocity, delta);
                }
            },
            .fall => {
                const move = flyMove(self.map_bmodel, options, &self.origin, &self.velocity, delta);
                if (move.on_ground) {
                    self.state = .walk;
                }
            },
        }
    }

    pub fn flyMove(map: *collision.BrushModel, options: MoveOptions, position: *linalg.Vec3, velocity: *linalg.Vec3, delta: f32) struct { on_ground: bool } {
        var on_ground = false;
        const half_extents = options.half_extents;

        var remainder: f32 = delta;

        var normals: [4]linalg.Vec3 = undefined;

        for (0..4) |i| {
            var step = velocity.*.mulScalar(remainder);
            if (map.traceBox(position.*, step, half_extents)) |impact| {
                if (impact.time > 0.0) {
                    position.* = position.*.add(step.mulScalar(impact.time));

                    remainder *= 1.0 - impact.time;
                }

                const normal = impact.plane.vec.xyz();
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

                do.arrow(
                    .world,
                    position.*.sub(impact.plane.vec.xyz().mulScalar(0.5)),
                    impact.plane.vec.xyz().mulScalar(0.2),
                    .{ 1.0, 0.0, 1.0, 1.0 },
                );

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

    pub fn walkMove(map: *collision.BrushModel, options: MoveOptions, position: *linalg.Vec3, velocity: *linalg.Vec3, delta: f32) ?struct {
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
            if (map.traceBox(
                attempt_position.add(step_position_offset),
                forwards_target,
                half_extents_minus_step,
            )) |impact| {
                attempt_position = attempt_position.add(forwards_target.mulScalar(impact.time));
                remainder *= (1.0 - impact.time);

                const normal = impact.plane.vec.xyz();
                attempt_velocity = attempt_velocity.sub(normal.mulScalar(attempt_velocity.dot(normal) * 1.005));
            } else {
                attempt_position = attempt_position.add(forwards_target);
                remainder = 0.0;
                break;
            }
        }

        var floor_plane: collision.Plane = undefined;

        const step_down_target = -options.step_height * 2.0;
        if (map.traceBox(
            attempt_position.add(step_position_offset),
            linalg.Vec3.new(0.0, 0.0, step_down_target),
            half_extents_minus_step,
        )) |step_down_impact| {
            // We hit something, but it WASN'T GROUND!
            if (step_down_impact.plane.vec.data[2] <= options.floor_max_slope) return null;
            attempt_position.data[2] += step_down_target * step_down_impact.time;

            floor_plane = step_down_impact.plane;
        } else {
            // If there isn't a floor below, abort! THAT'S EXACTLY WHAT I WAS AFRAID OF, GEOFF!
            return null;
        }

        if (map.traceBox(attempt_position.add(step_position_offset), linalg.Vec3.new(0, 0, options.step_height), half_extents_minus_step)) |_| {
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

    pub fn walkForces(self: *Walkcam, delta: f32, move: linalg.Vec3, speed: f32, accel: f32, decel: f32) void {
        self.velocity.data[2] -= 20.0 * delta;

        const velocity_2d = self.velocity.swizzle(linalg.Vec3, "xy0");
        blk: {
            const current_speed = velocity_2d.length();
            if (current_speed == 0.0) break :blk {};

            self.velocity = self.velocity.sub(velocity_2d.mulScalar(@min(current_speed, decel * delta) / current_speed));
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

    pub fn flyForces(self: *Walkcam, delta: f32, move: linalg.Vec3, speed: f32) void {
        self.velocity = self.velocity.mulScalar(std.math.exp(-delta * 5.0));
        self.velocity = self.velocity.add(move.mulScalar(delta * speed));
    }

    pub fn calculateBasis(self: *const Walkcam) linalg.Mat3 {
        return linalg.Mat3.rotation(linalg.Vec3.new(0.0, 1.0, 0.0), self.angle[1])
            .multiply(linalg.Mat3.rotation(linalg.Vec3.new(0.0, 0.0, 1.0), self.angle[0]));
    }

    pub fn cameraMatrix(self: *const Walkcam) linalg.Mat4 {
        return linalg.Mat4.translationVec(self.origin.add(linalg.Vec3.new(0.0, 0.0, 0.5 - self.step_smooth)))
            .multiply(self.calculateBasis().toMat4());
    }
};

pub const App = struct {
    allocator: std.mem.Allocator,

    event_translator: EventTranslator = .{},

    input_bundler: InputBundler,

    cam: Walkcam,
    cam_partial: Walkcam,

    actions: [ActionId.max_enum]f32 = .{0.0} ** ActionId.max_enum,

    timescale: f32 = 1.0,

    font: Font,

    tick_length: f32 = 1.0 / 20.0,

    // How many seconds (<1 tick) that still need to be simulated.
    tick_remainder: f32 = 0.0,

    client: ?Client = null,

    pub fn init(allocator: std.mem.Allocator) !App {
        return App{
            .allocator = allocator,

            .input_bundler = InputBundler.init(),

            // ugly hack: this will be set later
            .cam = undefined,
            .cam_partial = undefined,

            // also ugly hack: this will be set later
            .font = undefined,
        };
    }

    pub fn update(self: *App, delta: f32) void {
        self.input_bundler.update(self, delta * self.timescale);

        self.tick_remainder += delta * self.timescale;
        if (self.tick_remainder > self.tick_length) {
            const command_frame = self.input_bundler.commit();

            self.tick_remainder -= self.tick_length;
            self.cam.update(self.tick_length, command_frame);
        }

        const command_frame = self.input_bundler.partial();

        self.cam_partial = self.cam;
        self.cam_partial.update(self.tick_remainder, command_frame);
    }

    pub fn handleEvent(self: *App, event: Event) bool {
        switch (event) {
            .action => |action| {
                self.actions[@intFromEnum(action.id)] = action.value;
                if (action.id == .debug2) {
                    if (action.pressed) {
                        if (self.timescale > 0.7) {
                            self.timescale = 0.2;
                        } else if (self.timescale > 0.15) {
                            self.timescale = 0.01;
                        } else {
                            self.timescale = 1.0;
                        }
                        std.debug.print("timescale = {d}\n", .{self.timescale});
                    }
                    return true;
                }
            },

            else => {},
        }

        if (self.input_bundler.handleEvent(self, event)) return true;

        return false;
    }

    pub fn glfw_cursorPosCallback(window: ?*c.GLFWwindow, x: f64, y: f64) callconv(.C) void {
        const self: *App = @ptrCast(@alignCast(c.glfwGetWindowUserPointer(window).?));

        const event = self.event_translator.translateMousePosition(.{ @as(f32, @floatCast(x)), @as(f32, @floatCast(y)) });
        _ = self.handleEvent(event);
    }

    // TODO: Actions aren't combined
    pub fn glfw_mouseButtonCallback(window: ?*c.GLFWwindow, button: c_int, action: c_int, mods: c_int) callconv(.C) void {
        _ = mods;
        const self: *App = @ptrCast(@alignCast(c.glfwGetWindowUserPointer(window).?));

        const action_id: ActionId = switch (button) {
            c.GLFW_MOUSE_BUTTON_LEFT => .attack1,
            c.GLFW_MOUSE_BUTTON_RIGHT => .attack2,
            else => return,
        };

        const pressed = action == c.GLFW_PRESS;

        const event = Event{
            .action = .{
                .id = action_id,
                .pressed = pressed,
                .value = if (pressed) 1.0 else 0.0,
            },
        };

        _ = self.handleEvent(event);
    }

    // TODO: Actions aren't combined
    pub fn glfw_keyCallback(window: ?*c.GLFWwindow, key: c_int, scancode: c_int, action: c_int, mods: c_int) callconv(.C) void {
        _ = scancode;
        _ = mods;

        const self: *App = @ptrCast(@alignCast(c.glfwGetWindowUserPointer(window).?));

        const action_id: ActionId = switch (key) {
            c.GLFW_KEY_E => .forward,
            c.GLFW_KEY_D => .back,
            c.GLFW_KEY_S => .left,
            c.GLFW_KEY_F => .right,
            c.GLFW_KEY_SPACE => .jump,
            c.GLFW_KEY_LEFT_SHIFT => .crouch,

            c.GLFW_KEY_LEFT_CONTROL,
            c.GLFW_KEY_CAPS_LOCK,
            => .fly_fast,

            c.GLFW_KEY_LEFT_ALT => .fly_slow,

            c.GLFW_KEY_F1 => .debug1,
            c.GLFW_KEY_F2 => .debug2,
            c.GLFW_KEY_F3 => .debug3,
            c.GLFW_KEY_F4 => .debug4,

            else => return,
        };

        const pressed = action == c.GLFW_PRESS or action == c.GLFW_REPEAT;

        const event = Event{
            .action = .{
                .id = action_id,
                .pressed = pressed,
                .value = if (pressed) 1.0 else 0.0,
            },
        };

        _ = self.handleEvent(event);
    }
};

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{ .stack_trace_frames = if (std.debug.sys_can_stack_trace) 10 else 0 }){};
    defer _ = gpa.deinit();
    var allocator = gpa.allocator();

    var args = try std.process.argsWithAllocator(allocator);
    defer args.deinit();

    var addr: ?std.net.Address = null;

    _ = args.next();
    while (args.next()) |arg| {
        if (std.mem.eql(u8, arg, "--addr")) {
            const ip_and_port = args.next().?;

            var ip: []const u8 = ip_and_port;
            var port: u16 = 27043;

            if (std.mem.indexOfScalar(u8, ip_and_port, ':')) |colon| {
                ip = ip_and_port[0..colon];
                port = std.fmt.parseInt(u16, ip_and_port[colon + 1 ..], 10) catch unreachable;
            }

            addr = std.net.Address.resolveIp(ip, port) catch unreachable;
        }

        if (std.mem.eql(u8, arg, "--dedicated")) {
            std.log.info("Doing a busy loop", .{});

            _ = c.glfwInit();
            defer c.glfwTerminate();

            var map = try Model.load(allocator, "zig-out/assets/x/test-map.model");
            defer map.deinit(allocator);

            var map_bmodel = try collision.BrushModel.fromModelVertices(allocator, map.vertices);
            defer map_bmodel.deinit(allocator);

            var server = Server.init(allocator, &map_bmodel);

            var remainder: f64 = 0.0;

            var previous_time: f64 = c.glfwGetTime();
            while (true) {
                const time = c.glfwGetTime();
                const delta: f64 = @floatCast(time - previous_time);
                previous_time = time;

                remainder += delta;
                while (remainder > @as(f64, @floatCast(server.tick_length))) {
                    remainder -= @as(f64, @floatCast(server.tick_length));
                    server.tick();
                }
            }
        }
    }

    if (c.glfwInit() == 0) return error.GlfwInitFailed;
    defer c.glfwTerminate();

    const window = c.glfwCreateWindow(1280, 720, "Awesome Zig Project", null, null) orelse
        return error.WindowCreationFailed;

    c.glfwMakeContextCurrent(window);

    var app = try App.init(allocator);

    c.glfwSetWindowUserPointer(window, &app);
    _ = c.glfwSetCursorPosCallback(window, App.glfw_cursorPosCallback);
    _ = c.glfwSetMouseButtonCallback(window, App.glfw_mouseButtonCallback);
    _ = c.glfwSetKeyCallback(window, App.glfw_keyCallback);

    const gl_version = c.gladLoadGL(c.glfwGetProcAddress);
    std.log.info("loaded OpenGL version {}.{}", .{
        c.GLAD_VERSION_MAJOR(gl_version),
        c.GLAD_VERSION_MINOR(gl_version),
    });

    const shader = try Shader.load(allocator, "zig-out/assets/debug/shader-grid");
    defer shader.deinit();

    var map = try Model.load(allocator, "zig-out/assets/x/test-map.model");
    map.upload();
    defer map.deinit(allocator);

    var map_bmodel = try collision.BrushModel.fromModelVertices(allocator, map.vertices);
    defer map_bmodel.deinit(allocator);

    var model = try Model.load(allocator, "zig-out/assets/x/hand.model");
    model.upload();
    defer model.deinit(allocator);

    c.glfwSetInputMode(window, c.GLFW_CURSOR, c.GLFW_CURSOR_DISABLED);

    var previous_time: f64 = 0.0;

    do.* = debug_overlay.DebugOverlay.init(allocator);

    do.resources.model_cube = try Model.load(allocator, "zig-out/assets/debug/cube.model");
    defer do.resources.model_cube.deinit(allocator);
    do.resources.model_cube.upload();

    do.resources.model_cylinder = try Model.load(allocator, "zig-out/assets/debug/cylinder.model");
    defer do.resources.model_cylinder.deinit(allocator);
    do.resources.model_cylinder.upload();

    do.resources.model_cone = try Model.load(allocator, "zig-out/assets/debug/cone.model");
    defer do.resources.model_cone.deinit(allocator);
    do.resources.model_cone.upload();

    do.resources.shader_flat = try Shader.load(allocator, "zig-out/assets/debug/shader-flat");
    defer do.resources.shader_flat.deinit();

    do.resources.shader_text = try Shader.load(allocator, "zig-out/assets/debug/shader-textured");
    defer do.resources.shader_text.deinit();

    do.resources.shader_ui_color = try Shader.load(allocator, "zig-out/assets/debug/shader-ui-color");
    defer do.resources.shader_ui_color.deinit();

    app.cam = Walkcam{
        .map_bmodel = &map_bmodel,
    };

    app.font = try Font.load(
        allocator,
        "zig-out/assets/debug/font.otf",
        .{ .size = 45.0, .atlas_size = 2048 },
    );
    // TODO: this should really be part of `App`
    defer app.font.deinit();

    // c.glEnable(c.GL_CULL_FACE);

    var trace_origin = linalg.Vec3.zero();
    var trace_direction = linalg.Vec3.zero();

    if (addr) |addr_definite| {
        const stream = try std.net.tcpConnectToAddress(addr_definite);

        const flags = try std.os.fcntl(stream.handle, 3, 0);
        _ = try std.os.fcntl(stream.handle, 4, flags | std.os.SOCK.NONBLOCK);

        app.client = Client{
            .channel = NetChannel{ .tcp_stream = stream },
        };
    }

    while (c.glfwWindowShouldClose(window) == c.GLFW_FALSE) {
        const time: f64 = c.glfwGetTime();
        var delta: f32 = @floatCast(time - previous_time);
        previous_time = time;

        var width: c_int = 0;
        var height: c_int = 0;

        c.glfwGetFramebufferSize(window, &width, &height);

        c.glViewport(0, 0, width, height);

        c.glClearColor(0.5, 0.7, 1.0, 1.0);
        c.glClear(c.GL_COLOR_BUFFER_BIT | c.GL_DEPTH_BUFFER_BIT);

        c.glEnable(c.GL_DEPTH_TEST);

        app.update(delta);

        var matrix_projectionview = linalg.Mat4.perspective(1.2, @as(f32, @floatFromInt(width)) / @as(f32, @floatFromInt(height)), 0.005, 500.0);
        var matrix_viewmodel_projectionview = linalg.Mat4.perspective(0.9, @as(f32, @floatFromInt(width)) / @as(f32, @floatFromInt(height)), 0.2, 1000.0);

        // This matrix was forged through trial and error. Dare not change its coefficients, lest you wake the dragons below.
        // Also, RIGHT HANDED COORDINATE SYSTEM LETS FUCKING GOOOOOOO
        matrix_projectionview = matrix_projectionview.multiply(linalg.Mat4{
            .data = .{
                .{ 0.0, 0.0, -1.0, 0.0 },
                .{ -1.0, 0.0, 0.0, 0.0 },
                .{ 0.0, 1.0, 0.0, 0.0 },
                .{ 0.0, 0.0, 0.0, 1.0 },
            },
        });

        matrix_viewmodel_projectionview = matrix_viewmodel_projectionview.multiply(linalg.Mat4{
            .data = .{
                .{ 0.0, 0.0, -1.0, 0.0 },
                .{ -1.0, 0.0, 0.0, 0.0 },
                .{ 0.0, 1.0, 0.0, 0.0 },
                .{ 0.0, 0.0, 0.0, 1.0 },
            },
        });
        matrix_viewmodel_projectionview = matrix_viewmodel_projectionview.multiply(linalg.Mat4.rotation(linalg.Vec3.new(0.0, 0.0, 1.0), std.math.pi * 0.5));

        const matrix_camera = app.cam_partial.cameraMatrix();

        matrix_projectionview = matrix_projectionview.multiply(matrix_camera.inverse());
        matrix_viewmodel_projectionview = matrix_viewmodel_projectionview.multiply(matrix_camera.inverse());

        map: {
            shader.bindWithUniforms(.{
                .u_matrix_projectionview = matrix_projectionview,
                .u_matrix_model = linalg.Mat4.identity(),
                .u_matrix_model_normal = linalg.Mat3.identity(),
            });
            map.draw();

            break :map {};
        }

        var show_ray = true;
        if (app.actions[@intFromEnum(ActionId.attack1)] > 0.5) {
            trace_origin = matrix_camera.multiplyVector(linalg.Vec4.new(0.0, 0.0, 0.0, 1.0)).xyz();
            trace_direction = matrix_camera.multiplyVector(linalg.Vec4.new(1.0, 0.0, 0.0, 0.0)).xyz().mulScalar(20.0);
            show_ray = false;
        }

        if (app.actions[@intFromEnum(ActionId.attack2)] > 0.5) {
            trace_direction.data[0] = 0.0;
            trace_direction.data[1] = 0.0;
        }

        if (true) {
            if (map_bmodel.traceRay(trace_origin, trace_direction)) |impact| {
                if (show_ray)
                    do.arrow(.world, trace_origin.add(trace_direction.mulScalar(impact.time * 0.5)), trace_direction.mulScalar(impact.time), .{ 0.0, 0.5, 1.0, 1.0 });
                do.arrow(
                    .world,
                    trace_origin.add(trace_direction.mulScalar(impact.time)),
                    impact.plane.vec.xyz(),
                    .{ 0.0, 1.0, 0.0, 1.0 },
                );

                do.arrow(
                    .world,
                    impact.plane.origin,
                    impact.plane.vec.xyz(),
                    .{ 1.0, 1.0, 0.0, 1.0 },
                );

                impact.brush.debug(map_bmodel, false);
            } else {
                if (show_ray) {
                    do.arrow(.world, trace_origin, trace_direction, .{ 0.0, 0.5, 1.0, 1.0 });
                }
            }
        }

        // TODO: use an arena allocator here
        if (false) {
            var pose_1 = try model.blankPose(allocator);
            defer pose_1.deinit(allocator);

            var pose_2 = try model.blankPose(allocator);
            defer pose_2.deinit(allocator);

            const f1: u32 = @as(u32, @intFromFloat(time * 60.0)) % 80 + 101;
            const f2 = f1 + 1;
            const fr = @mod(@as(f32, @floatCast(time)) * 60.0, 1.0);

            pose_1.fromInterpolation(model.framePose(f1), model.framePose(f2), fr);
            pose_1.fromCopy(model.framePose(101));
            pose_2.fromCopy(model.rest_pose);

            pose_2.addLayer(pose_1, 1.0);

            const bone_matrices = try model.poseMatrices(allocator, pose_2);
            defer allocator.free(bone_matrices);

            shader.bindWithUniforms(.{
                .u_matrix_projectionview = matrix_viewmodel_projectionview,
                .u_matrix_model = matrix_camera,
                .u_matrix_model_normal = matrix_camera.toMat3(),
                .u_bone_matrices = bone_matrices,
            });
            model.draw();
        }

        // map_bmodel.debug();

        // do.texturedQuad(
        //     .{ 0, 0, 1024.0, 1024.0 },
        //     .{ 0.0, 0.0, 1.0, 1.0 },
        //     linalg.Mat4.identity(),
        //     app.font.gl_texture,
        // );

        const size: [2]f32 = .{
            @floatFromInt(width),
            @floatFromInt(height),
        };

        do.text("Hello world", .{}, size[0] * 0.5, size[1] * 0.75, 0.5, 120.0, &app.font);
        do.text("pos: {d}", .{app.cam_partial.origin}, 20.0, 20.0, 0.0, 16.0, &app.font);
        do.text("vel: {d:.3} / {d}", .{ app.cam_partial.velocity.length(), app.cam_partial.velocity }, 20.0, 36.0, 0.0, 16.0, &app.font);
        do.text("tick pos: {d}", .{app.cam.origin}, 20.0, 52.0, 0.0, 16.0, &app.font);
        do.text("tick vel: {d}", .{app.cam.velocity}, 20.0, 68.0, 0.0, 16.0, &app.font);
        do.text("tick distance: {d}", .{app.cam.origin.sub(app.cam_partial.origin).length()}, 20.0, 84.0, 0.0, 16.0, &app.font);
        do.rect(20.0, 100.0, 300.0, 20.0, .{ 0, 0, 0, 160 });
        do.rect(20.0, 100.0, 300.0 * (app.tick_remainder / app.tick_length), 20.0, .{ 255, 0, 0, 255 });
        // do.text("Command queue:", .{}, size[0] - 400.0, 30.0, 0.0, 30.0, &app.font);
        // do.rect(size[0] - 400.0, 50.0, 300.0, 20.0, .{ 0, 0, 0, 120 });

        // for (0..10) |i| {
        //     do.rect(size[0] - 400.0 + @as(f32, @floatFromInt(i)) * 30.0, 50.0, 20.0, 20.0, .{ 0, 255, 0, 255 });
        // }

        // do.text("Interpolation queue:", .{}, size[0] - 400.0, 30.0 + 200.0, 0.0, 30.0, &app.font);

        do.projectionview_matrices = .{
            matrix_projectionview,
            matrix_viewmodel_projectionview,
            linalg.Mat4.orthographic(0.0, size[0], 0.0, size[1], -1.0, 1.0),
        };
        do.flush();

        c.glfwSwapBuffers(window);
        c.glfwPollEvents();
    }
}
