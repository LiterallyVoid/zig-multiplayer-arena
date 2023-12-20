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

pub const Flycam = struct {
    origin: linalg.Vec3 = linalg.Vec3.new(0.0, 0.0, 5.0),
    velocity: linalg.Vec3 = linalg.Vec3.zero(),
    angle: [2]f32 = .{ 0.0, 0.0 },

    actions: [6]f32 = .{0.0} ** 6,
    fast: bool = false,
    slow: bool = false,

    pub fn update(self: *Flycam, delta: f32) void {
        const basis = self.calculateBasis();

        const forward = basis.multiplyVectorOpp(linalg.Vec3.new(1.0, 0.0, 0.0));
        const right = basis.multiplyVectorOpp(linalg.Vec3.new(0.0, 1.0, 0.0));
        const up = linalg.Vec3.new(0.0, 0.0, 1.0);

        self.velocity = self.velocity.mulScalar(std.math.exp(-delta * 20.0));

        var speed: f32 = 250.0;

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

pub const App = struct {
    allocator: std.mem.Allocator,

    event_translator: EventTranslator = .{},
    flycam: Flycam = .{},

    actions: [ActionId.max_enum]f32 = .{0.0} ** ActionId.max_enum,

    pub fn init(allocator: std.mem.Allocator) !App {
        return App{
            .allocator = allocator,
        };
    }

    pub fn update(self: *App, delta: f32) void {
        self.flycam.update(delta);
    }

    pub fn handleEvent(self: *App, event: Event) bool {
        switch (event) {
            .action => |action| {
                self.actions[@intFromEnum(action.id)] = action.value;
            },

            else => {},
        }
        if (self.flycam.handleEvent(event)) return true;

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

    while (c.glfwWindowShouldClose(window) == c.GLFW_FALSE) {
        const time: f64 = c.glfwGetTime();
        var delta: f32 = @floatCast(time - previous_time);
        previous_time = time;

        var width: c_int = 0;
        var height: c_int = 0;

        c.glfwGetFramebufferSize(window, &width, &height);

        c.glViewport(0, 0, width, height);

        c.glClearColor(0.2, 0.2, 0.2, 1.0);
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

        const matrix_camera = app.flycam.cameraMatrix();

        matrix_projectionview = matrix_projectionview.multiply(matrix_camera.inverse());
        matrix_viewmodel_projectionview = matrix_viewmodel_projectionview.multiply(matrix_camera.inverse());

        map: {
            shader.bindWithUniforms(.{
                .u_matrix_projectionview = matrix_projectionview,
                .u_matrix_model = linalg.Mat4.identity(),
            });
            map.draw();

            break :map {};
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
                .u_bone_matrices = bone_matrices,
            });
            model.draw();
        }

        map_bmodel.debug();

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
