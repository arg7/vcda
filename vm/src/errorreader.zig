const std = @import("std");

pub const ErrorReader = struct {
    const Self = @This();
    // The read function that matches the AnyReader signature
    fn readFn(context: *const anyopaque, dest: []u8) !usize {
        const self: *const Self = @ptrCast(@alignCast(context));
        _ = self; // Unused in this example
        _ = dest; // Unused in this example
        return error.TestError; // Simulate an error
    }

    // Public method to create an AnyReader
    pub fn any(self: *Self) std.io.AnyReader {
        return .{ .context = self, .readFn = readFn };
    }
};
