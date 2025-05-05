const std = @import("std");

pub const IOPipe = struct {
    reader: std.io.AnyReader,
    buffer: [256]u8,
    allocator: std.mem.Allocator,
    idx: u32,

    pub fn init(allocator: std.mem.Allocator, reader: std.io.AnyReader) !IOPipe {
        const o = IOPipe{ .reader = reader, .allocator = allocator, .buffer = [_]u8{0} ** 256, .idx = 0 };
        //o.printId("init()");
        return o;
    }

    pub fn deinit(self: *IOPipe) void {
        _ = self;
        //self.printId("deinit()");
        //self.allocator.free(self.buffer);
    }

    // Read a byte, preferring buffered data
    pub fn readByte(self: *IOPipe) !u8 {
        if (self.idx > 0) {
            self.idx -= 1;
            return self.buffer[self.idx];
        }
        return try self.reader.readByte();
    }

    // Push back a byte to the buffer
    pub fn pushBack(self: *IOPipe, byte: u8) !void {
        if (self.idx < self.buffer.len) {
            self.buffer[self.idx] = byte;
            self.idx += 1;
        } else return error.BufferOverflow;
    }

    // Read bytes into a slice, using buffer first
    pub fn read(self: *IOPipe, dest: []u8) !usize {
        var bytes_read: usize = 0;
        for (dest, 0..) |_, i| {
            if (self.idx > 0) {
                self.idx -= 1;
                dest[i] = self.buffer[self.idx];
                bytes_read += 1;
            } else {
                // Manually handle the error union from readByte()
                const byte = self.reader.readByte() catch |err| switch (err) {
                    error.EndOfStream => break,
                    else => return err, // Propagate other errors
                };
                // Check if we got a byte or reached the end
                dest[i] = byte;
                bytes_read += 1;
            }
        }
        return bytes_read;
    }

    pub fn printId(self: *IOPipe, msg: []const u8) void {
        std.debug.print("IOPipe@{x}: {s}\n", .{ @intFromPtr(self), msg });
    }
};
