const std = @import("std");
const Allocator = std.mem.Allocator;

const c = @import("./c.zig");

const linalg = @import("./linalg.zig");

const Self = @This();

gl_program: c.GLuint,

fn reportInfoLog(
    label: []const u8,
    name: c.GLuint,
    getInfo: c.PFNGLGETSHADERINFOLOGPROC,
) void {
    var log: [8192]u8 = undefined;

    var length: c.GLint = 0;
    getInfo.?(name, 1024, &length, &log);

    if (length == 0) return;

    std.log.err("{s} log: {s}", .{
        label,
        log[0..@intCast(length)],
    });
}

pub fn init(vertex_source: []const u8, fragment_source: []const u8) Self {
    const program = c.glCreateProgram();

    inline for (
        .{ c.GL_VERTEX_SHADER, c.GL_FRAGMENT_SHADER },
        .{ vertex_source, fragment_source },
        .{ "vertex shader", "fragment shader" },
    ) |stage, source, label| {
        const shader = c.glCreateShader(stage);

        const source_len_c_int: c_int = @intCast(source.len);

        c.glShaderSource(shader, 1, &source.ptr, &source_len_c_int);
        c.glCompileShader(shader);

        c.glAttachShader(program, shader);

        reportInfoLog(label, shader, c.glad_glGetShaderInfoLog);

        // Shaders are reference counted or something I'll delete it right now.
        c.glDeleteShader(shader);
    }

    c.glLinkProgram(program);

    reportInfoLog("program", program, c.glad_glGetProgramInfoLog);

    return .{ .gl_program = program };
}

pub fn deinit(self: Self) void {
    c.glDeleteProgram(self.gl_program);
}

pub fn bind(self: Self) void {
    c.glUseProgram(self.gl_program);
}

pub fn bindWithUniforms(self: Self, uniforms: anytype) void {
    c.glUseProgram(self.gl_program);

    inline for (@typeInfo(@TypeOf(uniforms)).Struct.fields) |field| {
        const location = c.glGetUniformLocation(self.gl_program, field.name ++ "\x00");

        const value = @field(uniforms, field.name);

        switch (@TypeOf(value)) {
            f32 => c.glUniform1f(location, value),
            [4]f32 => c.glUniform4fv(location, 1, &value[0]),
            linalg.Mat3 => c.glUniformMatrix3fv(location, 1, c.GL_FALSE, @ptrCast(&value)),
            linalg.Mat4 => c.glUniformMatrix4fv(location, 1, c.GL_FALSE, @ptrCast(&value)),
            []linalg.Mat4 => c.glUniformMatrix4fv(location, @intCast(value.len), c.GL_FALSE, @ptrCast(value)),
            else => @compileError("unknown uniform type: " ++ @typeName(@TypeOf(value))),
        }
    }
}

pub fn load(allocator: Allocator, path: []const u8) !Self {
    // # Vertex shader
    const vertex_path = try std.mem.concat(allocator, u8, &.{ path, ".vert" });
    defer allocator.free(vertex_path);

    const vertex_source = try readShaderEvaluateIncludes(allocator, vertex_path);
    defer allocator.free(vertex_source);

    // # Fragment shader
    const fragment_path = try std.mem.concat(allocator, u8, &.{ path, ".frag" });
    defer allocator.free(fragment_path);

    const fragment_source = try readShaderEvaluateIncludes(allocator, fragment_path);
    defer allocator.free(fragment_source);

    return Self.init(vertex_source, fragment_source);
}

pub fn readShaderEvaluateIncludes(allocator: Allocator, path: []const u8) ![]const u8 {
    const source = std.fs.cwd().readFileAlloc(allocator, path, 16 * 1_024 * 1_024) catch |err| {
        std.log.info("couldn't open {s}: {}", .{ path, err });

        return err;
    };
    defer allocator.free(source);

    const parsed = try parseShaderIncludes(allocator, source);
    defer allocator.free(parsed.sources);
    defer allocator.free(parsed.file_paths);

    var total_length: usize = 0;
    for (parsed.sources) |source_segment| {
        total_length += source_segment.len;
    }

    var included_files_parsed = try std.ArrayListUnmanaged([]const u8)
        .initCapacity(allocator, parsed.file_paths.len);
    defer included_files_parsed.deinit(allocator);
    defer for (included_files_parsed.items) |included_file_parsed| {
        allocator.free(included_file_parsed);
    };

    const directory_of_containing_file = blk: {
        const last_slash_end = if (std.mem.lastIndexOfScalar(u8, path, '/')) |last_slash|
            last_slash + 1
        else
            0;
        break :blk path[0..last_slash_end];
    };

    for (parsed.file_paths) |relative_path| {
        const resolved_path = try std.mem.concat(allocator, u8, &.{
            directory_of_containing_file,
            relative_path,
        });
        defer allocator.free(resolved_path);

        const contents = try readShaderEvaluateIncludes(allocator, resolved_path);

        included_files_parsed.appendAssumeCapacity(contents);

        total_length += contents.len;
    }

    var concatenated = try std.ArrayListUnmanaged(u8)
        .initCapacity(allocator, total_length);
    errdefer concatenated.deinit(allocator);

    concatenated.appendSliceAssumeCapacity(parsed.sources[0]);

    for (included_files_parsed.items, parsed.sources[1..]) |included_file, source_slice| {
        concatenated.appendSliceAssumeCapacity(included_file);
        concatenated.appendSliceAssumeCapacity(source_slice);
    }

    std.debug.assert(concatenated.items.len == total_length);

    // Should never fail, because it should be exactly the right length already.
    return try concatenated.toOwnedSlice(allocator);
}

/// If `text` starts with `prefix`, return `text` with `prefix` removed.
pub fn cutPrefix(text: []const u8, prefix: []const u8) ?[]const u8 {
    if (!std.mem.startsWith(u8, text, prefix)) {
        return null;
    }

    return text[prefix.len..];
}

/// The result of parsing `#include` directives from a single shader file.
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
pub const Parsed = struct {
    sources: []const []const u8,
    file_paths: []const []const u8,
};

pub fn parseShaderIncludes(allocator: std.mem.Allocator, source: []const u8) !Parsed {
    var source_segment_start: usize = 0;
    var index: usize = 0;

    var sources = std.ArrayList([]const u8).init(allocator);
    errdefer sources.deinit();

    var file_paths = std.ArrayList([]const u8).init(allocator);
    errdefer file_paths.deinit();

    while (index < source.len) {
        const this_line_start = index;
        const this_line_end = std.mem.indexOfScalarPos(u8, source, index, '\n') orelse source.len;

        const this_line = source[index..this_line_end];
        index = this_line_end + 1;

        var file_path = cutPrefix(this_line, "#include") orelse continue;

        // FIXME: ugly hack: this just removes string terminators around the `#include` without any diagnostics.
        file_path = std.mem.trim(u8, file_path, "<> \r\n\t'\"");

        try sources.append(source[source_segment_start..this_line_start]);
        source_segment_start = index;

        try file_paths.append(file_path);
    }

    try sources.append(source[source_segment_start..]);

    return Parsed{
        .sources = try sources.toOwnedSlice(),
        .file_paths = try file_paths.toOwnedSlice(),
    };
}

test parseShaderIncludes {
    const source =
        \\foo
        \\#include "../stdlib.glsl"
        \\#include "../test.glsl"
        \\#define FOO
        \\#include "../test2.glsl"
        \\bar
    ;

    const parsed = try parseShaderIncludes(std.testing.allocator, source);

    try std.testing.expectEqualDeep(Parsed{
        .sources = &.{
            "foo\n",
            "",
            "#define FOO\n",
            "bar",
        },
        .file_paths = &.{
            "../stdlib.glsl",
            "../test.glsl",
            "../test2.glsl",
        },
    }, parsed);

    std.testing.allocator.free(parsed.sources);
    std.testing.allocator.free(parsed.file_paths);
}
