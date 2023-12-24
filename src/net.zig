const std = @import("std");
const s2s = @import("s2s");

pub fn setSocketNonblock(socket: std.os.socket_t) void {
    const flags = std.os.fcntl(socket, 3, 0) catch unreachable;
    _ = std.os.fcntl(socket, 4, flags | std.os.SOCK.NONBLOCK) catch unreachable;
}

pub const Channel = struct {
    const MAX_MESSAGE_LENGTH = 2048;

    tcp_stream: std.net.Stream,
    buffer: [MAX_MESSAGE_LENGTH]u8 = undefined,
    buffer_length: usize = 0,

    tx: usize = 0,
    rx: usize = 0,
    errors: usize = 0,

    pub fn send(self: *Channel, message: []const u8) void {
        var length_buf: [2]u8 = undefined;
        std.mem.writeIntLittle(u16, &length_buf, @intCast(message.len));
        self.tcp_stream.writeAll(&length_buf) catch {};
        self.tcp_stream.writeAll(message) catch |e| {
            std.log.err("error while writing: {}", .{e});
            self.errors += 1;
            return;
        };

        self.tx += message.len;
    }

    pub fn poll(self: *Channel, buffer: *[MAX_MESSAGE_LENGTH]u8) ?[]u8 {
        if (self.buffer_length < self.buffer.len) read_stream: {
            const amt = self.tcp_stream.read(self.buffer[self.buffer_length..]) catch |err| switch (err) {
                error.WouldBlock => break :read_stream,
                else => {
                    std.log.err("got error while reading from socket: {}", .{err});

                    self.errors += 1;

                    return null;
                },
            };

            self.buffer_length += amt;
            self.rx += amt;
        }

        if (self.buffer_length >= 2) {
            const packet_length = std.mem.readIntLittle(u16, self.buffer[0..2]);

            if (self.buffer_length < packet_length + 2) {
                return null;
            }

            std.mem.copy(u8, buffer[0..packet_length], self.buffer[2 .. packet_length + 2]);

            std.mem.copyForwards(u8, &self.buffer, self.buffer[packet_length + 2 ..]);
            self.buffer_length -= packet_length + 2;

            return buffer[0..packet_length];
        }

        return null;
    }

    pub fn sendTyped(self: *Channel, comptime T: type, message: T) void {
        var buffer: [MAX_MESSAGE_LENGTH]u8 = undefined;
        var stream = std.io.fixedBufferStream(&buffer);

        s2s.serialize(stream.writer(), T, message) catch unreachable;

        const encoded = stream.getWritten();

        // std.log.info("sending JSON packet: {s}", .{encoded});

        self.send(encoded);
    }

    pub fn pollTyped(self: *Channel, comptime T: type) ?T {
        var buffer: [MAX_MESSAGE_LENGTH]u8 = undefined;
        const bytes = self.poll(&buffer) orelse return null;

        // std.log.info("received JSON packet: {s}", .{bytes});

        var stream = std.io.fixedBufferStream(bytes);
        return s2s.deserialize(stream.reader(), T) catch unreachable;
    }
};

pub const Server = struct {
    tcp_server: std.net.StreamServer,

    pub fn init() Server {
        return .{
            .tcp_server = std.net.StreamServer.init(.{
                .reuse_address = true,
            }),
        };
    }

    pub fn listen(self: *Server, address: std.net.Address) !void {
        try self.tcp_server.listen(address);
        setSocketNonblock(self.tcp_server.sockfd.?);
    }

    // TODO: pass address through
    pub fn accept(self: *Server) !Channel {
        const connection = try self.tcp_server.accept();
        setSocketNonblock(connection.stream.handle);

        return .{
            .tcp_stream = connection.stream,
        };
    }
};

pub fn connect(address: std.net.Address) !Channel {
    const stream = try std.net.tcpConnectToAddress(address);

    setSocketNonblock(stream.handle);

    return .{
        .tcp_stream = stream,
    };
}
