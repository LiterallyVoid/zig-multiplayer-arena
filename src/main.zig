const std = @import("std");

const c = @cImport({
    @cInclude("glad/gl.h");
    @cInclude("GLFW/glfw3.h");
});

pub const linalg = @import("./linalg.zig");

/// If `text` starts with `prefix`, return `text` with `prefix` removed.
pub fn cutPrefix(text: []const u8, prefix: []const u8) ?[]const u8 {
    if (!std.mem.startsWith(u8, text, prefix)) {
        return null;
    }

    return text[prefix.len..];
}

pub const Model = struct {};

pub fn loadModel() !Model {}

/// The result of parsing `#include` directives from a single source file.
///
/// The final source code is the concatenation of:
///
///  * `sources[0]`
///  * the contents of the file at `file_paths[0]`
///  * `sources[1]`
///  * the contents of the file at `file_paths[1]`
///  * ...
///  * `sources[n]`
///
/// There must be exactly `n-1` items in `file_paths` and `n` items in `sources`.
///
/// Ownership: the `sources` and `file_paths` slices are allocated by `parseShaderIncludes`. Each element of these is a slice into the `source` passed to `parseShaderIncludes`.
pub const ParsedShader = struct {
    sources: []const []const u8,
    file_paths: []const []const u8,
};

pub fn parseShaderIncludes(source: []const u8, allocator: std.mem.Allocator) !ParsedShader {
    var source_segment_start: usize = 0;
    var index: usize = 0;

    var sources = std.ArrayList([]const u8).init(allocator);
    errdefer sources.deinit();

    var file_paths = std.ArrayList([]const u8).init(allocator);
    errdefer file_paths.deinit();

    while (true) {
        const this_line_start = index;
        const this_line_end = std.mem.indexOfScalarPos(u8, source, index, '\n');

        const this_line = source[index..this_line_end];
        index = this_line_end + 1;

        var file_path = cutPrefix(this_line, "#include") orelse continue;

        // FIXME: ugly hack: this just removes string terminators around the `#include` without any diagnostics.
        file_path = std.mem.trim(u8, file_path, "<> \r\n\t'\"");

        try sources.append(source[source_segment_start..this_line_start]);
        source_segment_start = this_line_end;

        try file_paths.append(file_path);
    }

    try sources.append(source[source_segment_start..]);

    return ParsedShader{
        .sources = sources.toOwnedSlice(),
        .file_paths = file_paths.toOwnedSlice(),
    };
}

pub fn main() !void {
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

    while (c.glfwWindowShouldClose(window) == c.GLFW_FALSE) {
        c.glfwSwapBuffers(window);
        c.glfwPollEvents();
    }
}
