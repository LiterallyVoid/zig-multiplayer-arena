const std = @import("std");
const linalg = @import("./linalg.zig");

const Shader = @import("./Shader.zig");
const Model = @import("./Model.zig");

pub const DebugOverlay = struct {
    pub const Space = enum {
        world,
        viewmodel,
        ui,

        pub const max_enum = std.meta.fields(Space).len;
    };

    pub const Object = struct {
        space: Space,

        model: *const Model,
        model_matrix: linalg.Mat4,

        shader: *const Shader,
        color: [4]f32,
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
    } = undefined,

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

            .model = &self.resources.model_cylinder,
            .model_matrix = matrix,
            .shader = &self.resources.shader_flat,

            .color = color,
        });
        self.addObject(.{
            .space = space,

            .model = &self.resources.model_cone,
            .model_matrix = tip_matrix,
            .shader = &self.resources.shader_flat,

            .color = color,
        });
    }

    pub fn flush(self: *DebugOverlay) void {
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
            object.model.draw();
        }

        self.last_object_id = 0;
    }
};

pub var singleton = DebugOverlay{};
