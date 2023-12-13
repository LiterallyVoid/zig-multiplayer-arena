const std = @import("std");

const c = @import("./c.zig");
pub const linalg = @import("./linalg.zig");
pub const Shader = @import("./Shader.zig");
pub const Model = @import("./Model.zig");

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{ .stack_trace_frames = if (std.debug.sys_can_stack_trace) 10 else 0 }){};
    defer _ = gpa.deinit();
    var allocator = gpa.allocator();

    if (c.glfwInit() == 0) return error.GlfwInitFailed;
    defer c.glfwTerminate();

    const window = c.glfwCreateWindow(640, 480, "Awesome Zig Project", null, null) orelse
        return error.WindowCreationFailed;

    c.glfwMakeContextCurrent(window);

    const gl_version = c.gladLoadGL(c.glfwGetProcAddress);
    std.log.info("loaded OpenGL version {}.{}", .{
        c.GLAD_VERSION_MAJOR(gl_version),
        c.GLAD_VERSION_MINOR(gl_version),
    });

    const shader = try Shader.load(allocator, "zig-out/assets/debug/shader-grid");
    defer shader.deinit();

    var model = try Model.load(allocator, "zig-out/assets/x/hand-blender40.model");
    model.upload();
    defer model.deinit(allocator);

    while (c.glfwWindowShouldClose(window) == c.GLFW_FALSE) {
        var width: c_int = 0;
        var height: c_int = 0;

        c.glfwGetFramebufferSize(window, &width, &height);

        c.glViewport(0, 0, width, height);

        c.glClearColor(0.2, 0.5, 1.0, 1.0);
        c.glClear(c.GL_COLOR_BUFFER_BIT | c.GL_DEPTH_BUFFER_BIT);

        c.glEnable(c.GL_DEPTH_TEST);

        var matrix = linalg.Mat4.perspective(1.1, @as(f32, @floatFromInt(width)) / @as(f32, @floatFromInt(height)), 0.1, 100.0);

        matrix = matrix.multiply(linalg.Mat4.lookAt(
            linalg.Vec3.new(2.0, 2.0, 2.0),
            linalg.Vec3.new(0.0, 0.0, 0.0),
            linalg.Vec3.new(0.0, 0.0, 1.0),
        ));

        // TODO: use an arena allocator here
        var pose = try model.blankPose(allocator);
        defer pose.deinit(allocator);

        pose.fromInterpolation(model.framePose(120), model.framePose(121), 0.5);

        const bone_matrices = try model.poseMatrices(allocator, pose);
        defer allocator.free(bone_matrices);

        shader.bindWithUniforms(.{
            .u_matrix = matrix,
            .u_bone_matrices = bone_matrices,
        });
        model.draw();

        c.glfwSwapBuffers(window);
        c.glfwPollEvents();
    }
}
