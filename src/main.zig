const std = @import("std");

const c = @import("./c.zig");
pub const linalg = @import("./linalg.zig");
pub const Shader = @import("./Shader.zig");
pub const Model = @import("./Model.zig");
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
        // TODO: surely everybody uses 1920x1080 windows and this wont come up later and ruin somebodys day
        var absolute = absolute_;
        absolute[1] = 1080.0 - absolute[1];

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

pub const Walkcam = struct {
    origin: linalg.Vec3 = linalg.Vec3.new(0.0, 0.0, 25.0),
    velocity: linalg.Vec3 = linalg.Vec3.zero(),
    angle: [2]f32 = .{ 0.0, 0.0 },

    map_bmodel: *collision.BrushModel,

    actions: [6]f32 = .{0.0} ** 6,
    fast: bool = false,
    slow: bool = false,
    fly: bool = false,

    pub fn update(self: *Walkcam, delta: f32) void {
        var speed: f32 = 10.0;

        if (self.fast) {
            speed *= 5.0;
        }

        if (self.slow) {
            speed /= 2.0;
        }

        var move_2d = linalg.Vec2.new(
            self.actions[2] - self.actions[3],
            self.actions[0] - self.actions[1],
        );

        move_2d = move_2d.rotate(self.angle[0]);
        const move = move_2d.swizzle(linalg.Vec3, "xy0");

        if (self.fly) {
            speed *= 10.0;
            self.velocity = self.velocity.mulScalar(std.math.exp(-delta * 5.0));
            self.velocity = self.velocity.add(move.mulScalar(delta * speed));
            self.velocity.data[2] += (self.actions[4] - self.actions[5]) * delta * speed;
        } else {
            self.velocity.data[2] -= 20.0 * delta;
            if (self.actions[4] > 0.5 and self.velocity.data[2] < 0.1) {
                self.velocity.data[2] = 8.0;
            }

            const accel = 300.0;
            const decel = 60.0;

            const velocity_2d = self.velocity.swizzle(linalg.Vec3, "xy0");
            blk: {
                const current_speed = velocity_2d.length();
                if (current_speed == 0.0) break :blk {};

                self.velocity = self.velocity.sub(velocity_2d.mulScalar(@min(current_speed, decel * delta) / current_speed));
            }

            if (move.length() > 0.0) {
                const dir = move.normalized();
                const current_speed = self.velocity.dot(dir);
                const target_speed = @min(move.length(), 1.0) * speed;

                if (current_speed < target_speed) {
                    self.velocity = self.velocity.add(dir.mulScalar(@min(target_speed - current_speed, accel * delta)));
                }
            }
        }

        var remainder: f32 = 1.0;

        var normals: [4]linalg.Vec3 = undefined;

        for (0..4) |i| {
            var step = self.velocity.mulScalar(delta * remainder);
            if (self.map_bmodel.traceBox(self.origin, step, linalg.Vec3.broadcast(1.0))) |impact| {
                if (impact.time > 0.0) {
                    self.origin = self.origin.add(step.mulScalar(impact.time));

                    remainder *= 1.0 - impact.time;
                }

                const normal = impact.plane.vec.xyz();

                self.velocity = self.velocity.sub(normal.mulScalar(self.velocity.dot(normal) * 1.0));

                for (normals[0..i]) |previous_normal| {
                    if (previous_normal.dot(self.velocity) > 1e-6) continue;
                    if (previous_normal.dot(normal) > 1 - 1e-6) continue;
                    const edge = previous_normal.cross(normal).normalized();
                    self.velocity = edge.mulScalar(edge.dot(self.velocity));
                }

                impact.brush.debug(self.map_bmodel.*, true);

                do.arrow(
                    .world,
                    self.origin.sub(impact.plane.vec.xyz().mulScalar(0.5)),
                    impact.plane.vec.xyz().mulScalar(0.2),
                    .{ 1.0, 0.0, 1.0, 1.0 },
                );

                normals[i] = normal;
            } else {
                self.origin = self.origin.add(step);
                remainder = 0;
                break;
            }
        }

        std.log.info("remainder {d}", .{remainder});
    }

    pub fn calculateBasis(self: *const Walkcam) linalg.Mat3 {
        return linalg.Mat3.rotation(linalg.Vec3.new(0.0, 1.0, 0.0), self.angle[1])
            .multiply(linalg.Mat3.rotation(linalg.Vec3.new(0.0, 0.0, 1.0), self.angle[0]));
    }

    pub fn cameraMatrix(self: *const Walkcam) linalg.Mat4 {
        return linalg.Mat4.translationVec(self.origin.add(linalg.Vec3.new(0.0, 0.0, 0.5)))
            .multiply(self.calculateBasis().toMat4());
    }

    pub fn handleEvent(self: *Walkcam, event: Event) bool {
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

                    .debug1 => if (action.pressed) {
                        self.fly = !self.fly;
                        const msg = if (self.fly) "enabled" else "disabled";
                        std.debug.print("fly mode {s}\n", .{msg});
                    },

                    else => return false,
                }

                return true;
            },
        }

        return false;
    }
};

pub const App = struct {
    allocator: std.mem.Allocator,

    event_translator: EventTranslator = .{},
    cam: Walkcam,

    actions: [ActionId.max_enum]f32 = .{0.0} ** ActionId.max_enum,

    pub fn init(allocator: std.mem.Allocator) !App {
        return App{
            .allocator = allocator,

            // ugly hack: this will be set later
            .cam = undefined,
        };
    }

    pub fn update(self: *App, delta: f32) void {
        self.cam.update(delta);
    }

    pub fn handleEvent(self: *App, event: Event) bool {
        switch (event) {
            .action => |action| {
                self.actions[@intFromEnum(action.id)] = action.value;
            },

            else => {},
        }
        if (self.cam.handleEvent(event)) return true;

        return false;
    }

    pub fn glfw_cursorPosCallback(window: ?*c.GLFWwindow, x: f64, y: f64) callconv(.C) void {
        const self: *App = @ptrCast(@alignCast(c.glfwGetWindowUserPointer(window).?));

        const event = self.event_translator.translateMousePosition(.{ @floatCast(x), @floatCast(y) });
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

    if (c.glfwInit() == 0) return error.GlfwInitFailed;
    defer c.glfwTerminate();

    const window = c.glfwCreateWindow(640, 480, "Awesome Zig Project", null, null) orelse
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

    app.cam = Walkcam{
        .map_bmodel = &map_bmodel,
    };

    // c.glEnable(c.GL_CULL_FACE);

    var trace_origin = linalg.Vec3.zero();
    var trace_direction = linalg.Vec3.zero();

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

        const matrix_camera = app.cam.cameraMatrix();

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

        // do.addObject(.{
        //     .space = .world,
        //     .model = &map,
        //     .model_matrix = linalg.Mat4.identity(),
        //     .shader = &shader,
        //     .color = .{ 1.0, 0.0, 0.0, 1.0 },
        // });

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

        do.projectionview_matrices = .{
            matrix_projectionview,
            matrix_viewmodel_projectionview,
            undefined,
        };
        do.flush();

        c.glfwSwapBuffers(window);
        c.glfwPollEvents();
    }
}
