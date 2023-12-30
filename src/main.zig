const std = @import("std");

const c = @import("./c.zig");
pub const linalg = @import("./linalg.zig");
pub const Shader = @import("./Shader.zig");
pub const Model = @import("./Model.zig");
pub const Font = @import("./Font.zig");
pub const collision = @import("./collision.zig");
pub const debug_overlay = @import("./debug_overlay.zig");
pub const net = @import("./net.zig");
pub const world = @import("./world.zig");
pub const RingBuffer = @import("./ring_buffer.zig").RingBuffer;
pub const game = @import("./game.zig");
const do = &debug_overlay.singleton;

pub const ActionId = enum {
    attack1,
    attack2,
    reload,

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

pub const InputBundler = struct {
    latest_command_frame: game.CommandFrame = .{},
    latest_command_frame_fraction: f32 = 0.0,

    pub fn integrate(self: *InputBundler, frame: game.CommandFrame, length: f32) void {
        var ratio: f32 = 0.0;
        if (length > 0.0) {
            ratio = length / (length + self.latest_command_frame_fraction);
        }

        self.latest_command_frame = self.latest_command_frame.compose(frame, ratio);
        self.latest_command_frame_fraction += length;
    }

    pub fn update(self: *InputBundler, app: *const App, slice_length: f32) void {
        // TODO: These should probably take angle into account now
        const slice_frame = .{ .movement = .{
            app.actions[@intFromEnum(ActionId.forward)] -
                app.actions[@intFromEnum(ActionId.back)],

            app.actions[@intFromEnum(ActionId.left)] -
                app.actions[@intFromEnum(ActionId.right)],

            app.actions[@intFromEnum(ActionId.jump)] -
                app.actions[@intFromEnum(ActionId.crouch)],
        }, .impulses = .{
            if (app.actions[@intFromEnum(ActionId.jump)] > 0.5)
                @as(f32, 0.0)
            else
                null,
            if (app.actions[@intFromEnum(ActionId.attack1)] > 0.5)
                @as(f32, 0.0)
            else
                null,
            if (app.actions[@intFromEnum(ActionId.reload)] > 0.5)
                @as(f32, 0.0)
            else
                null,
        } };

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

    pub fn partial(self: *InputBundler) game.CommandFrame {
        return self.latest_command_frame;
    }

    pub fn commit(self: *InputBundler) game.CommandFrame {
        var frame = self.latest_command_frame;
        frame.random = std.crypto.random.int(u8);

        self.latest_command_frame = .{};
        self.latest_command_frame_fraction = 0.0;

        return frame;
    }
};

/// A server's idea of what a client is.
pub const ServerClient = struct {
    id: u32,

    channel: net.Channel,

    entity: world.State.EntityId,
    command_queue: RingBuffer(game.CommandFrame, 8),

    /// The world state that's currently in flight to the client.
    world_state_pending: world.State,
};

/// All commands that the server can send to the client.
pub const ServerMessage = union(enum) {
    hello: struct {
        your_id: u32,
        your_player: world.State.EntityId,
        tick_length: f32,
    },
    entity_refresh: struct {
        slot: u16,
        data: world.State.EntitySlot,
    },
    tick_report: struct {
        last_frame: u8,
        command_queue_length: u32,
        time_since_receiving_command: f32,
    },
};

pub const Server = struct {
    allocator: std.mem.Allocator,

    map: *collision.BrushModel,

    world: world.State = .{},
    clients: std.ArrayListUnmanaged(ServerClient) = .{},

    latest_client_id: u32 = 0,

    tick_length: f32 = 1.0 / 24.0,

    pub fn init(allocator: std.mem.Allocator, map: *collision.BrushModel) Server {
        return Server{
            .allocator = allocator,
            .map = map,
        };
    }

    pub fn handleConnect(self: *Server, channel_: net.Channel) !void {
        // TODO: requires mutating because of the `tx` counter.
        var channel = channel_;

        const id = self.latest_client_id;
        self.latest_client_id += 1;

        const entity = self.world.spawn().?;
        entity.entity = .{ .player = .{} };
        entity.controller = id;

        channel.sendTyped(ServerMessage, .{ .hello = .{
            .your_id = id,
            .your_player = entity.id,
            .tick_length = self.tick_length,
        } });

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
            while (client.channel.pollTyped(ClientMessage)) |message| {
                self.handleMessage(client, message);
            }
        }
        for (self.clients.items, 0..) |*client, i| {
            if (client.channel.errors <= 50) continue;
            std.log.err(">50 errors, disconnecting client", .{});
            if (self.world.get(client.entity)) |client_entity| {
                client_entity.alive = false;
            }

            client.channel.tcp_stream.close();

            _ = self.clients.swapRemove(i);
            break;
        }
        for (self.clients.items) |*client| {
            const command_frame = client.command_queue.peek(0) orelse game.CommandFrame{};
            const entity = self.world.get(client.entity) orelse {
                std.log.warn("client has no entity?", .{});

                continue;
            };

            switch (entity.entity) {
                .player => |*player| player.command_frame = command_frame,
            }
        }

        self.world.tickStage(
            .movement,
            .{ .map = self.map, .accessible_state = &self.world },
            self.tick_length,
        );
        self.world.tickStage(
            .weapons,
            .{ .map = self.map, .accessible_state = &self.world },
            self.tick_length,
        );

        for (self.clients.items) |*client| {
            for (&client.world_state_pending.entities, self.world.entities) |*old, new| {
                if (std.meta.eql(old.*, new)) continue;

                old.* = new;
                client.channel.sendTyped(ServerMessage, .{
                    .entity_refresh = .{
                        .slot = new.id.index,
                        .data = new,
                    },
                });
            }

            const cqlen = client.command_queue.length;
            const command_frame = client.command_queue.pop() orelse game.CommandFrame{};

            client.channel.sendTyped(ServerMessage, .{ .tick_report = .{
                .last_frame = command_frame.random,
                .command_queue_length = cqlen,
                .time_since_receiving_command = 0.0,
            } });
        }
    }
};

/// All commands that the client can send to the server.
pub const ClientMessage = union(enum) {
    tick: struct {
        command_frame: game.CommandFrame,
    },
};

pub const Client = struct {
    pub const CommandQueueHealthFrame = struct {
        last_frame: u8,
        queued_ticks: f32,

        slow: bool,
        fast: bool,
    };

    map: *collision.BrushModel,

    /// Must be synced to server's ID of self
    id: u32 = 0,

    channel: net.Channel,

    player_entity: world.State.EntityId = world.State.EntityId.null_handle,

    interpolation_queue: RingBuffer(world.State, 8) = .{},
    latest_world_state: world.State = .{},

    /// Must be as long as server<->client round-trip latency.
    command_queue: RingBuffer(game.CommandFrame, 128) = .{},

    prediction: std.ArrayListUnmanaged(world.State) = .{},
    /// The partial prediction frame that should be displayed.
    prediction_partial: world.State = .{},
    prediction_dirty: bool = true,

    server_command_queue_health_debug: RingBuffer(CommandQueueHealthFrame, 60) = .{},
    time_since_health: f32 = 0.0,

    input_bundler: InputBundler = .{},

    timescale: f32 = 1.0,

    tick_length: f32 = 1.0 / 24.0,
    tick_remainder: f32 = 0.0,

    time_since_last_world_tick: f32 = 0.0,

    pub fn update(self: *Client, app: *const App, delta: f32) void {
        self.time_since_last_world_tick += delta;

        const delta_warped = delta * self.timescale;

        self.input_bundler.update(app, delta_warped);

        self.tick_remainder += delta_warped;
        if (self.tick_remainder > self.tick_length) {
            self.tick_remainder -= self.tick_length;

            if (self.tick_remainder > self.tick_length * 5) {
                std.log.err("too much time {d}! dropping", .{self.tick_remainder});
                self.tick_remainder = 0;
            }

            const command_frame = self.input_bundler.commit();

            self.tick(command_frame);
        }

        while (self.channel.pollTyped(ServerMessage)) |message| {
            self.handleMessage(message);
        }

        // Use app.allocator, because the predictions list must last across frames.
        self.predict(app.allocator, self.input_bundler.partial(), self.tick_remainder);

        self.time_since_health += delta;
    }

    pub fn tick(self: *Client, frame: game.CommandFrame) void {
        self.command_queue.push(frame);
        self.channel.sendTyped(ClientMessage, .{
            .tick = .{
                .command_frame = frame,
            },
        });
    }

    pub fn predict(self: *Client, allocator: std.mem.Allocator, partial_frame: game.CommandFrame, partial_remainder: f32) void {
        if (self.prediction_dirty) {
            self.prediction.clearRetainingCapacity();

            self.prediction_dirty = false;
        }

        var accumulator = self.prediction.getLastOrNull() orelse
            self.latest_world_state;

        for (&accumulator.entities) |*entity| {
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

            if (accumulator.get(self.player_entity)) |player| {
                player.entity.player.command_frame = command_frame;
            }

            accumulator.tickStage(
                .movement,
                .{ .map = self.map, .accessible_state = &accumulator },
                self.tick_length,
            );
            accumulator.tickStage(
                .weapons,
                .{ .map = self.map, .accessible_state = &accumulator },
                self.tick_length,
            );

            self.prediction.append(allocator, accumulator) catch unreachable;
        }

        if (accumulator.get(self.player_entity)) |player| {
            player.entity.player.command_frame = partial_frame;
        }

        accumulator.tickStage(
            .movement,
            .{ .map = self.map, .accessible_state = &accumulator },
            partial_remainder,
        );
        accumulator.tickStage(
            .weapons,
            .{ .map = self.map, .accessible_state = &accumulator },
            partial_remainder,
        );

        self.prediction_partial = accumulator;
    }

    pub fn handleMessage(self: *Client, message: ServerMessage) void {
        switch (message) {
            .hello => |hello| {
                self.id = hello.your_id;
                self.player_entity = hello.your_player;
                self.tick_length = hello.tick_length;
            },
            .entity_refresh => |refresh| {
                self.latest_world_state.entities[refresh.slot] = refresh.data;
            },
            .tick_report => |report| {
                self.timescale = 1.0;
                const queued_ticks = @as(f32, @floatFromInt(report.command_queue_length)) + report.time_since_receiving_command / self.tick_length;

                var slow = false;
                var fast = false;

                if (queued_ticks < 0.5) {
                    fast = true;
                    self.timescale = 1.15;
                } else if (queued_ticks > 1.5) {
                    slow = true;
                    self.timescale = 0.85;
                }

                self.server_command_queue_health_debug.push(.{
                    .last_frame = report.last_frame,
                    .queued_ticks = queued_ticks,
                    .slow = slow,
                    .fast = fast,
                });
                self.time_since_health = 0;

                var found_current = false;
                for (0..self.command_queue.length) |index| {
                    const item = self.command_queue.peek(@intCast(index)) orelse continue;
                    if (item.random != report.last_frame) continue;

                    found_current = true;

                    for (0..index) |_| {
                        _ = self.command_queue.pop();
                    }

                    _ = self.command_queue.pop();

                    break;
                }

                self.time_since_last_world_tick = 0.0;

                if (!found_current) {
                    // std.log.err("error: command queue desync?", .{});
                    // std.log.err("error: server just simulated {}", .{report.last_frame});
                    // var my_frames: [512]u8 = undefined;
                    // for (0..self.command_queue.length) |index| {
                    //     const item = self.command_queue.peek(@intCast(index)) orelse continue;
                    //     my_frames[index] = item.random;
                    // }
                    // std.log.err("my frames: {any}", .{my_frames[0..self.command_queue.length]});
                }

                self.prediction_dirty = true;

                self.interpolation_queue.push(self.latest_world_state);
            },
        }
    }
};

pub const App = struct {
    allocator: std.mem.Allocator,

    event_translator: EventTranslator = .{},

    actions: [ActionId.max_enum]f32 = .{0.0} ** ActionId.max_enum,

    font: Font,

    client: ?Client = null,

    captured: bool = false,

    third_person: bool = false,

    pub fn init(allocator: std.mem.Allocator) !App {
        return App{
            .allocator = allocator,

            // ugly hack: this will be set later
            .font = undefined,
        };
    }

    pub fn update(self: *App, delta: f32) void {
        if (self.client) |*client| {
            client.update(self, delta);
        }
    }

    pub fn handleEvent(self: *App, event: Event) bool {
        switch (event) {
            .action => |action| {
                self.actions[@intFromEnum(action.id)] = action.value;
                // if (action.id == .debug2) {
                //     if (action.pressed) {
                //         if (self.timescale > 0.7) {
                //             self.timescale = 0.2;
                //         } else if (self.timescale > 0.15) {
                //             self.timescale = 0.01;
                //         } else {
                //             self.timescale = 1.0;
                //         }
                //         std.debug.print("timescale = {d}\n", .{self.timescale});
                //     }
                //     return true;
                // }

                if (action.id == .debug3 and action.pressed) {
                    self.third_person = !self.third_person;
                }
            },

            else => {},
        }

        if (self.client) |*client| {
            if (client.input_bundler.handleEvent(self, event)) return true;
        }

        return false;
    }

    pub fn glfw_cursorPosCallback(window: ?*c.GLFWwindow, x: f64, y: f64) callconv(.C) void {
        const self: *App = @ptrCast(@alignCast(c.glfwGetWindowUserPointer(window).?));

        const event = self.event_translator.translateMousePosition(.{ @as(f32, @floatCast(x)), @as(f32, @floatCast(y)) });

        if (self.captured) {
            _ = self.handleEvent(event);
        }
    }

    // TODO: Actions aren't combined
    pub fn glfw_mouseButtonCallback(window: ?*c.GLFWwindow, button: c_int, action: c_int, mods: c_int) callconv(.C) void {
        _ = mods;
        const self: *App = @ptrCast(@alignCast(c.glfwGetWindowUserPointer(window).?));

        c.glfwSetInputMode(window, c.GLFW_CURSOR, c.GLFW_CURSOR_DISABLED);
        self.captured = true;

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

        if (key == c.GLFW_KEY_ESCAPE) {
            c.glfwSetInputMode(window, c.GLFW_CURSOR, c.GLFW_CURSOR_NORMAL);
            self.captured = false;
        }

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

pub var server_should_exit = std.atomic.Value(bool).init(false);

pub fn runServer(allocator: std.mem.Allocator, listen_address: std.net.Address) !void {
    var map = try Model.load(allocator, "zig-out/assets/x/test-map.model");
    defer map.deinit(allocator);

    var map_bmodel = try collision.BrushModel.fromModelVertices(allocator, map.vertices);
    defer map_bmodel.deinit(allocator);

    var server = Server.init(allocator, &map_bmodel);
    var net_server = net.Server.init();
    try net_server.listen(listen_address);
    std.log.info("Server listening on {}", .{listen_address});

    var remainder: f64 = 0.0;

    var previous_time = std.time.nanoTimestamp();
    while (!server_should_exit.load(.Unordered)) {
        const time = std.time.nanoTimestamp();
        const delta = @as(f64, @floatFromInt(time - previous_time)) / 1_000_000_000.0;
        previous_time = time;

        remainder += delta;
        while (remainder > @as(f64, @floatCast(server.tick_length))) {
            remainder -= @as(f64, @floatCast(server.tick_length));

            var timer = try std.time.Timer.start();
            server.tick();

            const tick_ms = @as(f64, @floatFromInt(timer.read())) / 1_000_000.0;
            const tick_budget_ms = server.tick_length * 1_000.0;
            if (tick_ms > tick_budget_ms / 2.0) {
                std.log.warn("tick took {d:.3}ms of {d:.3}ms budget", .{ tick_ms, tick_budget_ms });
            }
        }

        const channel = net_server.accept() catch |err| switch (err) {
            error.WouldBlock => continue,
            else => {
                std.log.err("error accepting: {}", .{err});
                continue;
            },
        };

        try server.handleConnect(channel);
    }
}

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{ .stack_trace_frames = if (std.debug.sys_can_stack_trace) 10 else 0 }){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    var args = try std.process.argsWithAllocator(allocator);
    defer args.deinit();

    var addr: std.net.Address = std.net.Address.resolveIp("127.0.0.1", 27043) catch unreachable;

    var listen_server_thread: ?std.Thread = null;

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
            try runServer(allocator, addr);
            return;
        }

        if (std.mem.eql(u8, arg, "--listen")) {
            // TODO: There's no synchronization between the server listening on
            // an address and the client connecting to that address.
            listen_server_thread = try std.Thread.spawn(
                .{},
                runServer,
                .{ allocator, addr },
            );
        }
    }

    defer {
        server_should_exit.store(true, .Unordered);
        if (listen_server_thread) |thread| thread.join();
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

    var player_model = try Model.load(allocator, "zig-out/assets/x/player.model");
    player_model.upload();
    defer player_model.deinit(allocator);

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

    app.font = try Font.load(
        allocator,
        "zig-out/assets/debug/font.otf",
        .{ .size = 45.0, .atlas_size = 2048 },
    );
    // TODO: this should really be part of `App`
    defer app.font.deinit();

    // c.glEnable(c.GL_CULL_FACE);

    const channel = try net.connect(addr);

    app.client = Client{
        .map = &map_bmodel,
        .channel = channel,
    };

    var walk_anim_time: f32 = 0.0;

    while (c.glfwWindowShouldClose(window) == c.GLFW_FALSE) {
        var frame_arena = std.heap.ArenaAllocator.init(allocator);
        defer frame_arena.deinit();

        const frame_allocator = frame_arena.allocator();

        const time: f64 = c.glfwGetTime();
        const delta: f32 = @floatCast((time - previous_time));
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

        var matrix_camera = linalg.Mat4.identity();
        if (app.client) |*client| {
            if (client.prediction_partial.get(client.player_entity)) |player| {
                matrix_camera = player.entity.player.cameraMatrix();
            }

            if (app.third_person) {
                matrix_camera = matrix_camera.multiply(linalg.Mat4.translation(-4.0, 0.0, 0.0));
            }

            const current_worldstate = client.interpolation_queue.peek(
                client.interpolation_queue.length -% 1,
            ) orelse client.latest_world_state;

            const previous_worldstate = client.interpolation_queue.peek(
                client.interpolation_queue.length -% 2,
            ) orelse current_worldstate;

            const ratio = client.time_since_last_world_tick / client.tick_length;

            const worldstate = current_worldstate.interpolate(previous_worldstate, ratio);

            for (worldstate.entities) |entity_slot| {
                if (!entity_slot.alive) continue;

                const entity = entity_slot.entity;

                do.arrow(.world, entity.player.origin.sub(linalg.Vec3.new(0.0, 0.0, 1.0)), linalg.Vec3.new(0.0, 0.0, 0.5), .{ 1.0, 0.9, 0.4, 1.0 });

                if (entity_slot.controller == client.id) continue;

                walk_anim_time += entity.player.velocity.mul(linalg.Vec3.new(1.0, 1.0, 0.0)).length() * delta / 0.1;

                const f1: u32 = @as(u32, @intFromFloat(walk_anim_time / 40.0 * 60.0)) % 40 + 100;
                const f2 = f1 + 1;
                const fr = @mod(@as(f32, @floatCast(walk_anim_time / 40.0)) * 60.0, 1.0);

                var pose = try player_model.blankPose(frame_allocator);
                var pose_anim = try player_model.blankPose(frame_allocator);

                pose_anim.fromInterpolation(player_model.framePose(f1), player_model.framePose(f2), fr);

                pose.fromCopy(player_model.rest_pose);
                pose.addLayer(pose_anim, 1.0);

                do.addObject(.{
                    .pass = .world_opaque,
                    .gl_vao = player_model.gl_vao,
                    .vertex_first = 0,
                    .vertices_count = player_model.vertices_count,
                    .model_matrix = linalg.Mat4.translationVec(
                        entity.player.origin
                            .sub(linalg.Vec3.new(0.0, 0.0, 1.0)),
                    )
                        .multiply(linalg.Mat4.rotation(linalg.Vec3.new(0.0, 0.0, 1.0), -entity.player.angle[0] + std.math.pi * 0.5)),
                    .bone_matrices = try player_model.poseMatrices(frame_allocator, pose),
                    .shader = &shader,
                    .color = .{ 1.0, 1.0, 1.0, 1.0 },
                    .gl_texture = null,
                });
            }

            var w = world.WorldInterface{
                .map = client.map,
                .accessible_state = client.interpolation_queue.peekMut(client.interpolation_queue.length) orelse &client.latest_world_state,
            };

            if (client.prediction_partial.get(client.player_entity)) |player_slot| {
                const impact = w.traceBox(
                    .{
                        .ignore_entity = client.player_entity,
                    },
                    player_slot.entity.player.origin,
                    matrix_camera.multiplyVector(linalg.Vec3.new(50.0, 0.0, 0.0).xyzw(0.0)).xyz(),
                    linalg.Vec3.new(0.2, 0.2, 0.2),
                );
                std.log.info("ooooo {?}", .{impact});
            }
        }

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

        // TODO: use an arena allocator here
        if (false) {
            var pose_1 = try model.blankPose(frame_allocator);
            var pose_2 = try model.blankPose(frame_allocator);

            const f1: u32 = @as(u32, @intFromFloat(time * 60.0)) % 80 + 101;
            const f2 = f1 + 1;
            const fr = @mod(@as(f32, @floatCast(time)) * 60.0, 1.0);

            pose_1.fromInterpolation(model.framePose(f1), model.framePose(f2), fr);
            // pose_1.fromCopy(model.framePose(101));
            pose_2.fromCopy(model.rest_pose);

            pose_2.addLayer(pose_1, 1.0);

            const bone_matrices = try model.poseMatrices(frame_allocator, pose_2);

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

        // do.text("Hello world", .{}, size[0] * 0.5, size[1] * 0.75, 0.5, 120.0, &app.font);
        // do.text("pos: {d}", .{app.cam_partial.origin}, 20.0, 20.0, 0.0, 16.0, &app.font);
        // do.text("vel: {d:.3} / {d}", .{ app.cam_partial.velocity.length(), app.cam_partial.velocity }, 20.0, 36.0, 0.0, 16.0, &app.font);
        // do.text("tick pos: {d}", .{app.cam.origin}, 20.0, 52.0, 0.0, 16.0, &app.font);
        // do.text("tick vel: {d}", .{app.cam.velocity}, 20.0, 68.0, 0.0, 16.0, &app.font);
        // do.text("tick distance: {d}", .{app.cam.origin.sub(app.cam_partial.origin).length()}, 20.0, 84.0, 0.0, 16.0, &app.font);

        if (app.actions[@intFromEnum(ActionId.debug4)] > 0.5) {
            if (app.client) |client| {
                const tx = @as(f32, @floatFromInt(client.channel.tx)) / time;
                const rx = @as(f32, @floatFromInt(client.channel.rx)) / time;

                do.text("bandwidth: {d:.1}KB/s tx, {d:.1}KB/s rx", .{ tx / 1024.0, rx / 1024.0 }, 20.0, 184.0, 0.0, 16.0, &app.font);
                do.text("command queue", .{}, 20.0, 200.0, 0.0, 16.0, &app.font);
                do.rect(20.0, 216.0, 320.0, 60.0, .{ 0, 0, 0, 160 });

                const health = client.server_command_queue_health_debug;

                for (0..20) |i| {
                    const index = health.length -% 20 +% @as(u32, @intCast(i));
                    const x = @as(f32, @floatFromInt(i)) * 10.0 + 21.0;
                    if (health.peek(index)) |frame| {
                        if (frame.queued_ticks > 0.1) {
                            const h = 10.0 * frame.queued_ticks - 2.0;
                            do.rect(x, 217.0 + 58.0 - h, 8.0, h, .{ 0, 255, 0, 255 });
                        } else {
                            do.rect(x, 267.0, 8.0, 8.0, .{ 255, 0, 0, 255 });
                        }

                        if (frame.slow) {
                            do.rect(x - 1.0, 216.0, 10.0, 70.0, .{ 96, 96, 0, 128 });
                        }
                        if (frame.fast) {
                            do.rect(x - 1.0, 216.0, 10.0, 70.0, .{ 0, 38, 96, 128 });
                        }
                    }

                    do.rect(15.0, 95.0, 310.0, 30.0, .{ 160, 160, 160, 160 });
                    do.rect(20.0, 100.0, 300.0, 20.0, .{ 0, 0, 0, 255 });
                    do.rect(20.0, 100.0, 300.0 * (client.tick_remainder / client.tick_length), 20.0, .{ 255, 0, 0, 255 });
                }

                for (0..client.command_queue.length) |i| {
                    const item = client.command_queue.peek(@intCast(i)) orelse continue;
                    _ = item;
                    const x = @as(f32, @floatFromInt(i)) * 10.0 + 222.0;
                    do.rect(x, 267.0, 8.0, 8.0, .{ 0, 90, 255, 255 });
                }
            }
        }
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
