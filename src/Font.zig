const std = @import("std");

const c = @import("./c.zig");

const Self = @This();

pub const CachedGlyph = struct {};

allocator: std.mem.Allocator,

font_bytes: []const u8,
font: c.stbtt_fontinfo,
scale: f32,

glyphs: std.AutoHashMapUnmanaged(i32, CachedGlyph),

gl_texture: c.GLuint,

pub const Options = struct {
    size: f32,
    atlas_size: u32,
};

pub fn load(allocator: std.mem.Allocator, path: []const u8, options: Options) !Self {
    const font_bytes = try std.fs.cwd().readFileAlloc(allocator, path, 512 * 1024);

    var font: c.stbtt_fontinfo = undefined;

    const ok = c.stbtt_InitFont(&font, font_bytes.ptr, c.stbtt_GetFontOffsetForIndex(font_bytes.ptr, 0));
    if (ok == 0) {
        return error.FontParsingFailed;
    }

    const atlas_size_int: c_int = @intCast(options.atlas_size);

    var gl_texture: c.GLuint = 0;
    c.glGenTextures(1, &gl_texture);
    c.glBindTexture(c.GL_TEXTURE_2D, gl_texture);
    c.glTexImage2D(c.GL_TEXTURE_2D, 0, c.GL_R8, atlas_size_int, atlas_size_int, 0, c.GL_RED, c.GL_UNSIGNED_BYTE, null);

    return Self{
        .allocator = allocator,
        .font_bytes = font_bytes,
        .font = font,
        .scale = c.stbtt_ScaleForPixelHeight(&font, options.size),
        .glyphs = .{},
        .gl_texture = gl_texture,
    };
}

pub fn deinit(self: *Self) void {
    self.allocator.free(self.font_bytes);
    self.glyphs.deinit(self.allocator);
}

pub fn cacheGlyph(self: *Self, glyph: i32) !void {
    var offset: [2]c_int = .{ 0, 0 };
    var size: [2]c_int = .{ 0, 0 };

    var sdf = c.stbtt_GetGlyphSDF(
        &self.font,
        self.scale,
        glyph,
        20,
        128,
        20.0,
        &size[0],
        &size[1],
        &offset[0],
        &offset[1],
    );
    defer c.stbtt_FreeSDF(sdf, null);
}
