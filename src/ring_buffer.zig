const std = @import("std");

pub fn RingBuffer(comptime T: type, comptime limit: u32) type {
    return struct {
        const Self = @This();

        items: [limit]T = undefined,

        length: u32 = 0,
        insert_head: u32 = 0,

        pub fn push(self: *Self, item: T) void {
            self.insert_head = (self.insert_head + 1) % limit;
            self.items[self.insert_head] = item;

            if (self.length < limit) {
                self.length += 1;
            }
        }

        pub fn pop(self: *Self) ?T {
            if (self.length == 0) return null;

            self.length -= 1;
            return self.items[(self.insert_head + limit - self.length) % limit];
        }

        pub fn peek(self: *const Self, index: u32) ?T {
            if (index < 0 or index >= self.length) return null;
            const wrapped_index =
                (self.insert_head + limit - self.length + index + 1) %
                limit;
            return self.items[wrapped_index];
        }

        pub fn peekMut(self: *Self, index: u32) ?*T {
            if (index < 0 or index >= self.length) return null;
            const wrapped_index =
                (self.insert_head + limit - self.length + index + 1) %
                limit;
            return &self.items[wrapped_index];
        }
    };
}

test RingBuffer {
    var i = RingBuffer(u8, 2){};
    try std.testing.expectEqual(@as(?u8, null), i.pop());
    try std.testing.expectEqual(@as(?u8, null), i.peek(0));

    i.push(1);
    try std.testing.expectEqual(@as(u32, 1), i.length);
    try std.testing.expectEqual(@as(?u8, 1), i.peek(0));
    try std.testing.expectEqual(@as(?u8, 1), i.pop());
    try std.testing.expectEqual(@as(?u8, null), i.pop());
    try std.testing.expectEqual(@as(?u8, null), i.peek(0));

    i.push(2);
    i.push(3);

    try std.testing.expectEqual(@as(?u8, 2), i.peek(0));
    try std.testing.expectEqual(@as(?u8, 3), i.peek(1));
    try std.testing.expectEqual(@as(?u8, 2), i.pop());
    try std.testing.expectEqual(@as(?u8, 3), i.pop());
    try std.testing.expectEqual(@as(?u8, null), i.peek(0));

    i.push(4);
    i.push(5);
    i.push(6);
    try std.testing.expectEqual(@as(?u8, 5), i.peek(0));
    try std.testing.expectEqual(@as(?u8, 6), i.peek(1));
    try std.testing.expectEqual(@as(?u8, null), i.peek(2));
}
