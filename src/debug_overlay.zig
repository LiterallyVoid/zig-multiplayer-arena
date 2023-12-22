const std = @import("std");
const linalg = @import("./linalg.zig");

const c = @import("./c.zig");

const Shader = @import("./Shader.zig");
const Model = @import("./Model.zig");
const Font = @import("./Font.zig");

pub const ImmediateRenderer = struct {
    pub const Vertex = struct {
        position: [3]f32,
        uv: [2]f32,
        color: [4]u8,
    };

    gl_vao: c.GLuint,
    gl_vbo: c.GLuint,
    vertices: std.ArrayList(Vertex),

    pub fn init(allocator: std.mem.Allocator) ImmediateRenderer {
        var gl_vao: c.GLuint = undefined;
        c.glGenVertexArrays(1, &gl_vao);
        c.glBindVertexArray(gl_vao);

        var gl_vbo: c.GLuint = undefined;
        c.glGenBuffers(1, &gl_vbo);
        c.glBindBuffer(c.GL_ARRAY_BUFFER, gl_vbo);

        c.glEnableVertexAttribArray(0);
        c.glVertexAttribPointer(0, 3, c.GL_FLOAT, c.GL_FALSE, @sizeOf(Vertex), @ptrFromInt(@offsetOf(Vertex, "position")));

        c.glEnableVertexAttribArray(4);
        c.glVertexAttribPointer(4, 2, c.GL_FLOAT, c.GL_FALSE, @sizeOf(Vertex), @ptrFromInt(@offsetOf(Vertex, "uv")));

        c.glEnableVertexAttribArray(5);
        c.glVertexAttribPointer(5, 4, c.GL_UNSIGNED_BYTE, c.GL_TRUE, @sizeOf(Vertex), @ptrFromInt(@offsetOf(Vertex, "color")));

        c.glBindVertexArray(0);

        return .{
            .gl_vao = gl_vao,
            .gl_vbo = gl_vbo,
            .vertices = std.ArrayList(Vertex).init(allocator),
        };
    }

    /// Upload vertices into the OpenGL VBO, and *also* clear `self.vertices`.
    pub fn flush(self: *ImmediateRenderer) void {
        c.glBindBuffer(c.GL_ARRAY_BUFFER, self.gl_vbo);
        c.glBufferData(
            c.GL_ARRAY_BUFFER,
            @intCast(@sizeOf(Vertex) * self.vertices.items.len),
            self.vertices.items.ptr,
            c.GL_STREAM_DRAW,
        );

        self.vertices.clearRetainingCapacity();
    }
};

pub const DebugOverlay = struct {
    pub const Space = enum {
        world,
        viewmodel,
        ui,

        pub const max_enum = std.meta.fields(Space).len;
    };

    pub const Object = struct {
        space: Space,

        gl_vao: c.GLuint,
        vertex_first: u32,
        vertices_count: u32,
        model_matrix: linalg.Mat4,

        shader: *const Shader,
        color: [4]f32,

        gl_texture: ?c.GLuint = null,
    };

    projectionview_matrices: [Space.max_enum]linalg.Mat4 = undefined,

    objects: [8192]Object = undefined,
    last_object_id: usize = 0,

    objects_over_limit: usize = 0,

    resources: struct {
        model_cube: Model,
        model_cylinder: Model,
        model_cone: Model,

        shader_flat: Shader,
        shader_textured: Shader,
    } = undefined,

    immediate_renderer: ImmediateRenderer,

    pub fn init(allocator: std.mem.Allocator) DebugOverlay {
        return DebugOverlay{
            .immediate_renderer = ImmediateRenderer.init(allocator),
        };
    }

    pub fn addObject(self: *DebugOverlay, object: Object) void {
        if (self.last_object_id >= self.objects.len) {
            self.objects_over_limit += 1;
            return;
        }

        self.objects[self.last_object_id] = object;
        self.last_object_id += 1;
    }

    pub fn arrow(self: *DebugOverlay, space: Space, start: linalg.Vec3, dir: linalg.Vec3, color: [4]f32) void {
        var up = linalg.Vec3.new(0.0, 0.0, 1.0);
        const alternateUp = linalg.Vec3.new(1.0, 0.0, 0.0);
        if (@fabs(up.dot(dir)) > @fabs(alternateUp.dot(dir))) {
            up = alternateUp;
        }

        const scale = 0.5;

        const tip_radius = 0.12 * scale;
        const tip_length = 0.2 * scale;

        const radius = 0.04 * scale;
        const length = dir.length() - tip_length;

        const matrix = linalg.Mat4.targetTo(start, dir, up)
            .multiply(linalg.Mat4.scaleVec(linalg.Vec3.new(
            radius,
            radius,
            length,
        )));

        const tip_matrix = linalg.Mat4.targetTo(start.add(dir), dir, up)
            .multiply(linalg.Mat4.scaleVec(linalg.Vec3.new(
            tip_radius,
            tip_radius,
            tip_length,
        )))
            .multiply(linalg.Mat4.translation(0.0, 0.0, -1.0));

        self.addObject(.{
            .space = space,

            .gl_vao = self.resources.model_cylinder.gl_vao,
            .vertex_first = 0,
            .vertices_count = self.resources.model_cylinder.vertices_count,
            .model_matrix = matrix,
            .shader = &self.resources.shader_flat,

            .color = color,
        });
        self.addObject(.{
            .space = space,

            .gl_vao = self.resources.model_cone.gl_vao,
            .vertex_first = 0,
            .vertices_count = self.resources.model_cone.vertices_count,
            .model_matrix = tip_matrix,
            .shader = &self.resources.shader_flat,

            .color = color,
        });
    }

    // rect, uvs: x1, y1, x2, y2
    pub fn texturedQuad(self: *DebugOverlay, rect: [4]f32, uvs: [4]f32, matrix: linalg.Mat4, texture: ?c.GLuint) void {
        const color = [4]u8{ 255, 255, 255, 255 };
        const corners = [_]ImmediateRenderer.Vertex{
            .{
                .position = .{ rect[0], rect[1], 0.0 },
                .uv = .{ uvs[0], uvs[1] },
                .color = color,
            },
            .{
                .position = .{ rect[2], rect[1], 0.0 },
                .uv = .{ uvs[2], uvs[1] },
                .color = color,
            },
            .{
                .position = .{ rect[0], rect[3], 0.0 },
                .uv = .{ uvs[0], uvs[3] },
                .color = color,
            },
            .{
                .position = .{ rect[2], rect[3], 0.0 },
                .uv = .{ uvs[2], uvs[3] },
                .color = color,
            },
        };

        var object = Object{
            .space = .ui,

            .gl_vao = self.immediate_renderer.gl_vao,
            .vertex_first = @intCast(self.immediate_renderer.vertices.items.len),
            .vertices_count = 0,

            .model_matrix = matrix,

            .shader = &self.resources.shader_textured,
            .color = .{ 1.0, 1.0, 1.0, 1.0 },

            .gl_texture = texture,
        };

        self.immediate_renderer.vertices.appendSlice(&.{
            corners[0],
            corners[1],
            corners[3],
            corners[3],
            corners[2],
            corners[0],
        }) catch unreachable;

        object.vertices_count = @intCast(self.immediate_renderer.vertices.items.len - object.vertex_first);

        self.addObject(object);
    }

    pub fn text(self: *DebugOverlay, chars: []const u8, x_: f32, y_: f32, size: f32, font: *Font) void {
        var x: f32 = x_;
        var y: f32 = y_;
        for (chars) |char| {
            const glyph_id = c.stbtt_FindGlyphIndex(&font.font, char);
            const glyph = font.cacheGlyph(glyph_id);

            std.log.info("{}", .{glyph});

            const scale = size / font.size;

            self.texturedQuad(
                [_]f32{
                    glyph.rect[0] * scale + x,
                    glyph.rect[1] * scale + y,
                    glyph.rect[2] * scale + x,
                    glyph.rect[3] * scale + y,
                },
                glyph.uv_rect,
                linalg.Mat4.identity(),
                font.gl_texture,
            );

            var advance: c_int = 0;
            c.stbtt_GetGlyphHMetrics(&font.font, glyph_id, &advance, null);

            x += @as(f32, @floatFromInt(advance)) * font.scale * scale;
        }
    }

    pub fn flush(self: *DebugOverlay) void {
        self.immediate_renderer.flush();

        if (self.objects_over_limit > 0) {
            std.log.err(
                "debug renderer: rendering {} objects over limit of {}",
                .{ self.objects_over_limit, self.objects.len },
            );

            self.objects_over_limit = 0;
        }

        for (self.objects[0..self.last_object_id]) |object| {
            object.shader.bindWithUniforms(.{
                .u_matrix_projectionview = self.projectionview_matrices[@intFromEnum(object.space)],
                .u_matrix_model = object.model_matrix,
                .u_matrix_model_normal = object.model_matrix.transpose().inverse().toMat3(),
                .u_color = object.color,
            });
            c.glBindVertexArray(object.gl_vao);
            c.glDrawArrays(
                c.GL_TRIANGLES,
                @intCast(object.vertex_first),
                @intCast(object.vertices_count),
            );
        }

        self.last_object_id = 0;
    }
};

pub var singleton: DebugOverlay = undefined;
