const std = @import("std");
const expect = std.testing.expect;
const expectEqual = std.testing.expectEqual;

pub fn alu(
    comptime T: type,
    comptime op: [:0]const u8,
    comptime cf: ?bool,
    a: T,
    b: T,
) struct {
    ret: T,
    c: u8,
} {
    var fl: u8 = undefined;
    var r: T = undefined;
    const op_cf = if (cf) |vcf| (if (vcf) "stc; " else "clc; ") else "";
    const opcodes = "mov %[a], %[ret]; " ++ op_cf ++ op ++ " %[b], %[ret]; setc %[flags];";

    std.log.err("{s}", .{opcodes});

    asm volatile (opcodes
        : [ret] "=r" (r),
          [flags] "=r" (fl),
        : [a] "r" (a),
          [b] "r" (b),
    );
    return .{ .ret = r, .c = fl };
}

// Helper to extract Carry Flag from fl
fn getCarry(flags: u32) u1 {
    return @intFromBool(flags & 1 != 0);
}

test "ALU: Basic Operations (i8)" {
    const res2 = alu(u8, "add", null, 2, 1);
    try expectEqual(@as(u8, 3), res2.ret);
    try expectEqual(0, getCarry(res2.c));

    const res = alu(u8, "adc", true, 255, 1);
    try expectEqual(@as(u8, 1), res.ret);
    try expectEqual(1, getCarry(res.c));

}
