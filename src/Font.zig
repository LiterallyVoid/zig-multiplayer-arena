const std = @import("std");

const c = @import("./c.zig");

const Self = @This();

pub const CachedGlyph = struct {
    // Pixel coordinates.
    rect: [4]f32,

    // Normalized texture coordinates (0.0-1.0)
    uv_rect: [4]f32,
};

allocator: std.mem.Allocator,

font_bytes: []const u8,
font: c.stbtt_fontinfo,
scale: f32,
atlas_size: u32,

gl_texture: c.GLuint,

glyphs: std.AutoHashMapUnmanaged(i32, CachedGlyph) = .{},

// stupidest possible packing algorithm
pack_x: u32 = 0,
pack_y: u32 = 0,
pack_row_height: u32 = 0,

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
    c.glTexParameteri(c.GL_TEXTURE_2D, c.GL_TEXTURE_MIN_FILTER, c.GL_LINEAR);
    c.glTexParameteri(c.GL_TEXTURE_2D, c.GL_TEXTURE_MAG_FILTER, c.GL_LINEAR);

    return Self{
        .allocator = allocator,
        .font_bytes = font_bytes,
        .font = font,
        .scale = c.stbtt_ScaleForPixelHeight(&font, options.size),
        .atlas_size = options.atlas_size,
        .gl_texture = gl_texture,
    };
}

pub fn deinit(self: *Self) void {
    self.allocator.free(self.font_bytes);
    self.glyphs.deinit(self.allocator);
}

pub fn cacheGlyph(self: *Self, glyph: i32) void {
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

    if (self.pack_x + @as(u32, @intCast(size[0])) >= self.atlas_size) {
        self.pack_x = 0;
        self.pack_y += self.pack_row_height;
        self.pack_row_height = 0;
    }

    const atlas_x: u32 = self.pack_x;
    const atlas_y: u32 = self.pack_y;

    self.pack_x += @intCast(size[0]);
    self.pack_row_height = @max(self.pack_row_height, @as(u32, @intCast(size[1])));

    c.glPixelStorei(c.GL_UNPACK_ALIGNMENT, 1);
    c.glBindTexture(c.GL_TEXTURE_2D, self.gl_texture);
    c.glTexSubImage2D(
        c.GL_TEXTURE_2D,
        0,
        @intCast(atlas_x),
        @intCast(atlas_y),
        size[0],
        size[1],
        c.GL_RED,
        c.GL_UNSIGNED_BYTE,
        sdf,
    );

    const offset_float = [_]f32{
        @floatFromInt(offset[0]),
        @floatFromInt(offset[1]),
    };

    const size_float = [_]f32{
        @floatFromInt(size[0]),
        @floatFromInt(size[1]),
    };

    const atlas_size_float: f32 = @floatFromInt(self.atlas_size);

    const uv_pos_float = [_]f32{
        @floatFromInt(atlas_x),
        @floatFromInt(atlas_y),
    };

    var cached_glyph = CachedGlyph{
        .rect = .{
            offset_float[0],
            offset_float[1],
            offset_float[0] + size_float[0],
            offset_float[1] + size_float[1],
        },

        .uv_rect = .{
            uv_pos_float[0] / atlas_size_float,
            uv_pos_float[1] / atlas_size_float,
            uv_pos_float[0] / atlas_size_float + size_float[0] / atlas_size_float,
            uv_pos_float[1] / atlas_size_float + size_float[1] / atlas_size_float,
        },
    };
    _ = cached_glyph;
}
