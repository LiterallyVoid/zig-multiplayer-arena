const std = @import("std");
const buildGlfw = @import("deps/glfw-build-zig/build.zig").buildGlfw;

pub fn buildAssets(b: *std.build.Builder, step: *std.Build.Step) !void {
    const blender = b.option([]const u8, "blender", "Blender (4.0+) command") orelse "blender";

    const root = "assets";

    var dir = try std.fs.cwd().openIterableDir(root, .{});
    defer dir.close();

    var walker = try dir.walk(b.allocator);
    defer walker.deinit();

    while (try walker.next()) |entry| {
        if (entry.kind != .file) continue;

        const extension = std.fs.path.extension(entry.basename);
        const path_without_extension = entry.path[0 .. entry.path.len - extension.len];

        const source_path = try std.mem.concat(b.allocator, u8, &.{ root, "/", entry.path });
        const dest_path_without_extension = try std.mem.concat(b.allocator, u8, &.{ "assets/", path_without_extension });

        if (std.mem.eql(u8, extension, ".blend")) {
            const exportFile = b.addSystemCommand(&.{blender});
            exportFile.addFileArg(.{ .path = source_path });

            exportFile.addArg("-P");
            exportFile.addFileArg(.{ .path = "tools/blender_export_model.py" });
            exportFile.addArg("-b");

            exportFile.addArg("--");

            exportFile.addArg("-o");
            const exported_model = exportFile.addOutputFileArg("model");

            const dest_path = try std.mem.concat(
                b.allocator,
                u8,
                &.{ dest_path_without_extension, ".model" },
            );

            const install_file = b.addInstallFile(exported_model, dest_path);

            step.dependOn(&exportFile.step);
            step.dependOn(&install_file.step);
        } else if (std.mem.eql(u8, extension, ".vert") or std.mem.eql(u8, extension, ".frag") or std.mem.eql(u8, extension, ".glsl")) {
            const dest_path = try std.mem.concat(
                b.allocator,
                u8,
                &.{ dest_path_without_extension, extension },
            );
            const install_file = b.addInstallFile(.{ .path = source_path }, dest_path);
            step.dependOn(&install_file.step);
        }
    }
}

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

        compile.addCSourceFile(.{ .file = .{ .path = "deps/stb/stb.c" }, .flags = &.{} });
        compile.addIncludePath(.{ .path = "deps/stb/" });
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

    const assets_step = b.step("assets", "Build assets");
    try buildAssets(b, assets_step);

    run_exe.step.dependOn(assets_step);
}
