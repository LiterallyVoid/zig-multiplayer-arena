const std = @import("std");
const buildGlfw = @import("deps/glfw-build-zig/build.zig").buildGlfw;

pub fn build(b: *std.build.Builder) !void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    const os = (try std.zig.system.NativeTargetInfo.detect(target)).target.os;

    const libglfw = try buildGlfw(b, target, optimize, .{
        .cocoa = os.tag == .macos,
        .win32 = os.tag == .windows,
        .linux = os.tag == .linux,
        .x11 = os.tag == .linux,

        // TODO: enable wayland
        .wayland = false,
    });

    const tests = b.addTest(.{
        .root_source_file = .{ .path = "src/main.zig" },
        .target = target,
        .optimize = optimize,
    });

    const exe = b.addExecutable(.{
        .name = "awesome-zig-project",
        .root_source_file = .{ .path = "src/main.zig" },
        .target = target,
        .optimize = optimize,
    });

    inline for (.{ tests, exe }) |compile| {
        compile.linkLibC();

        compile.addIncludePath(.{ .path = "deps/glad/include/" });
        compile.addCSourceFile(.{ .file = .{ .path = "deps/glad/src/gl.c" }, .flags = &.{} });

        compile.addIncludePath(.{ .path = "deps/glfw/include/" });
        compile.linkLibrary(libglfw);
    }

    b.installArtifact(exe);

    const step_test = b.step("test", "Test awesome-zig-project");
    const run_tests = b.addRunArtifact(tests);

    step_test.dependOn(&run_tests.step);

    const command_run = b.step("run", "Build and run awesome-zig-project");
    const run_exe = b.addRunArtifact(exe);
    run_exe.addArgs(b.args orelse &.{});

    command_run.dependOn(&run_exe.step);

    const docs = b.option(bool, "docs", "Build documentation");

    if (docs orelse false) {
        const install_docs = b.addInstallDirectory(.{
            .source_dir = exe.getEmittedDocs(),
            .install_dir = .prefix,
            .install_subdir = "doc",
        });

        b.getInstallStep().dependOn(&install_docs.step);
    }
}
