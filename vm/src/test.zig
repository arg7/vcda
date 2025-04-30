const std = @import("std");

pub fn main() !void {
    var buffer: [10]u8 = undefined; // Byte array
    const value: u32 = 0x12345678;  // Integer to write
    const X: usize = 2;             // Position to write to

    // Write value to buffer at position X in little-endian
    std.mem.writeInt(u32, buffer[X..X + 4], value, .little);

    // Verify: little-endian means least significant byte first
    try std.testing.expectEqualSlices(u8, &[_]u8{ 0x78, 0x56, 0x34, 0x12 }, buffer[X..X + 4]);
}