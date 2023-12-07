const std = @import("std");

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
