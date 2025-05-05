const std = @import("std");

pub const IOPipe = struct {
    reader: std.io.AnyReader,
    buffer: []u8, // Dynamic buffer for unread bytes
    allocator: std.mem.Allocator,
    idx: usize, // Start index of valid data in buffer
    len: usize, // Number of valid bytes in buffer

    // Initialize with a configurable buffer size
    pub fn init(allocator: std.mem.Allocator, reader: std.io.AnyReader, buffer_size: usize) !IOPipe {
        if (buffer_size == 0) return error.InvalidBufferSize;
        const buffer = try allocator.alloc(u8, buffer_size);
        @memset(buffer, 0); // Initialize buffer to 0
        return IOPipe{
            .reader = reader,
            .allocator = allocator,
            .buffer = buffer,
            .idx = 0,
            .len = 0,
        };
    }

    // Clean up the dynamically allocated buffer
    pub fn deinit(self: *IOPipe) void {
        self.allocator.free(self.buffer);
        self.buffer = &[_]u8{}; // Prevent use-after-free
    }

    // Read a byte, preferring buffered data
    pub fn readByte(self: *IOPipe) !u8 {
        if (self.len > 0) {
            // Read from buffer
            const byte = self.buffer[self.idx];
            self.idx = (self.idx + 1) % self.buffer.len;
            self.len -= 1;
            return byte;
        }
        // No buffered data, read from reader
        return try self.reader.readByte();
    }

    // Push back a single byte to the buffer
    pub fn pushBack(self: *IOPipe, byte: u8) !void {
        if (self.len >= self.buffer.len) {
            return error.BufferOverflow;
        }
        // Add byte to the end of the valid data
        const end_idx = (self.idx + self.len) % self.buffer.len;
        self.buffer[end_idx] = byte;
        self.len += 1;
    }

    // Push back multiple bytes to the buffer
    pub fn pushBackBytes(self: *IOPipe, bytes: []const u8) !void {
        if (self.len + bytes.len > self.buffer.len) {
            return error.BufferOverflow;
        }
        // Add bytes to the end of the valid data
        var i: usize = 0;
        while (i < bytes.len) : (i += 1) {
            const end_idx = (self.idx + self.len) % self.buffer.len;
            self.buffer[end_idx] = bytes[i];
            self.len += 1;
        }
    }

    // Read bytes into a slice, using buffer first
    pub fn read(self: *IOPipe, dest: []u8) !usize {
        var bytes_read: usize = 0;

        // First, read from the buffer
        while (bytes_read < dest.len and self.len > 0) {
            dest[bytes_read] = self.buffer[self.idx];
            self.idx = (self.idx + 1) % self.buffer.len;
            self.len -= 1;
            bytes_read += 1;
        }

        // If we still need more bytes, read from the underlying reader
        if (bytes_read < dest.len) {
            const remaining = dest[bytes_read..];
            const n = try self.reader.read(remaining);
            bytes_read += n;
        }

        return bytes_read;
    }

    pub fn printId(self: *IOPipe, msg: []const u8) void {
        std.debug.print("IOPipe@{x}: {s}\n", .{ @intFromPtr(self), msg });
    }
};

test "IOPipe buffer handling" {
    const allocator = std.testing.allocator;
    var buf = std.io.fixedBufferStream("hello");
    var pipe = try IOPipe.init(allocator, buf.reader().any(), 16);
    defer pipe.deinit();

    // Read some bytes
    var dest: [10]u8 = undefined;
    try std.testing.expectEqual(@as(usize, 2), try pipe.read(dest[0..2])); // Reads "he"
    try std.testing.expectEqualSlices(u8, "he", dest[0..2]);

    // Push back bytes
    try pipe.pushBackBytes("xy");
    try pipe.pushBack('z');

    // Read again, should get pushed-back bytes first
    try std.testing.expectEqual(@as(usize, 5), try pipe.read(dest[0..5])); // Reads "xyzll"
    try std.testing.expectEqualSlices(u8, "xyzll", dest[0..5]);

    // Test buffer overflow
    try std.testing.expectError(error.BufferOverflow,  pipe.pushBackBytes(&[_]u8{0} ** 256));
}
