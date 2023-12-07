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

    const exe = b.addExecutable(.{
        .name = "awesome-zig-project",
        .root_source_file = .{ .path = "src/main.zig" },
        .target = target,
        .optimize = optimize,
    });
    exe.linkLibC();

    exe.addIncludePath(.{ .path = "deps/glad/include/" });
    exe.addCSourceFile(.{ .file = .{ .path = "deps/glad/src/gl.c" }, .flags = &.{} });

    exe.addIncludePath(.{ .path = "deps/glfw/include/" });
    exe.linkLibrary(libglfw);

    b.installArtifact(exe);

    const command_run = b.step("run", "Build and run awesome-zig-project");
    const run_exe = b.addRunArtifact(exe);
    command_run.dependOn(&run_exe.step);

    run_exe.addArgs(b.args orelse &.{});

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
