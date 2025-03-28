const std = @import("std");
const expect = std.testing.expect;
const expectEqual = std.testing.expectEqual;

pub const ALUOperation = enum(u4) {
    _add = 0x0,
    _sub = 0x1,
    _and = 0x2,
    _or = 0x3,
    _xor = 0x4,
    _shl = 0x5,
    _shr = 0x6,
    _sar = 0x7,
    _mul = 0x8,
    _div = 0x9,
};

pub fn alu(
    comptime op: ALUOperation,
    comptime T: type,
    arg1: T,
    arg2: T,
    arg1h: ?T, // High bits for idiv, null means 0
    carry_in: ?u1, // Optional Carry Flag, defaults to 0
) struct {
    ret: T,
    reth: ?T,
    fl: u32,
} {
    var result: T = undefined;
    var result_high: ?T = null; // For imul high bits
    var flags: u32 = undefined;

    const is_signed = switch (@typeInfo(T)) {
        .int => |info| info.signedness == .signed,
        else => @compileError("ALU only supports integer types"),
    };

    // Base operation without carry
    const asm_op = switch (op) {
        ._add => "add",
        ._sub => "sub",
        ._and => "and",
        ._or => "or",
        ._xor => "xor",
        ._shl => "shl",
        ._shr => if (is_signed) "sar" else "shr",
        ._sar => "sar",
        ._mul => "imul",
        ._div => "idiv",
    };

    // Carry-aware operation
    const cf_op = switch (op) {
        ._add => if (carry_in != null and carry_in.? != 0) "adc" else asm_op,
        ._sub => if (carry_in != null and carry_in.? != 0) "sbb" else asm_op,
        ._shl, ._shr, ._sar => asm_op, // Shifts use carry_in via cf_set
        else => asm_op, // No carry-aware variant for others
    };

    // Carry flag setup
    const cf_set = if (carry_in != null and carry_in.? != 0) "stc" else "clc";

    const regs = switch (T) {
        i8, u8 => .{ .ra = "al", .rb = "bl", .rh = "ah" },
        i16, u16 => .{ .ra = "ax", .rb = "bx", .rh = "dx" },
        i32, u32 => .{ .ra = "eax", .rb = "ebx", .rh = "edx" },
        i64, u64 => .{ .ra = "rax", .rb = "rbx", .rh = "rdx" },
        else => @compileError("Unsupported type for ALU"),
    };

    // Initialize arg1h to 0 if null
    const high_bits: T = if (arg1h) |h| h else 0;

    switch (op) {
        ._mul => {
            // const ra = regs.ra;
            // const rh = regs.rh;
            // asm volatile (
            //     \\ mov %%ra, %[arg1]
            //     \\ imul %[arg2]  // Result in rh:ra
            //     \\ pushfq
            //     \\ pop %[flags]
            //     \\ mov %[res], %%ra
            //     \\ mov %[resh], %%rh
            //     : [res] "=r" (result),
            //       [resh] "=r" (result_high),
            //       [flags] "=r" (flags),
            //     : [arg1] "r" (@as(T, @bitCast(arg1))),
            //       [arg2] "r" (@as(T, @bitCast(arg2))),
            //       [ra] "s" (ra),
            //       [rh] "s" (rh),
            //     : "rax", "rbx", "rdx", "cc"
            // );
        },
        ._div => {
            // asm volatile (
            //     \\ mov %%rh, %[arg1h]  // High bits of dividend (0 if null)
            //     \\ mov %%ra, %[arg1]  // Low bits of dividend
            //     \\ mov %%rb, %[arg2]  // Divisor
            //     \\ idiv %%rb          // Divide rh:ra by rb
            //     \\ pushfq
            //     \\ pop %[flags]
            //     \\ mov %[res], %%ra   // Quotient
            //     : [res] "=r" (result),
            //       [flags] "=r" (flags),
            //     : [arg1] "r" (@as(T, @bitCast(arg1))),
            //       [arg1h] "r" (@as(T, @bitCast(high_bits))),
            //       [arg2] "r" (@as(T, @bitCast(arg2))),
            //       [ra] "s" (regs.ra),
            //       [rb] "s" (regs.rb),
            //       [rh] "s" (regs.rh),
            //     : "rax", "rbx", "rdx", "cc"
            // );
        },
        else => {
            // General case with %%cf_set and %%cf_op
            asm volatile (
                \\ %%cf_set           // Set or clear CF (stc or clc)
                \\ mov %%ra, %[arg1]
                \\ mov %%rb, %[arg2]
                \\ %%cf_op %%rb, %%ra  // Carry-aware or regular operation
                \\ pushfq             // Capture new flags
                \\ pop %[flags]
                \\ mov %[res], %%ra
                : [res] "=r" (result),
                  [flags] "=r" (flags),
                : [arg1] "r" (@as(T, @bitCast(arg1))),
                  [arg2] "r" (@as(T, @bitCast(arg2))),
                  [cf_op] "s" (cf_op),
                  [cf_set] "s" (cf_set),
                  [ra] "s" (regs.ra),
                  [rb] "s" (regs.rb),
                : "rax", "rbx", "cc"
            );
            result_high = high_bits; // null
        },
    }

    return .{ .ret = result, .reth = result_high, .fl = flags };
}

// Helper to extract Carry Flag from fl
fn getCarry(flags: u32) u1 {
    return @intFromBool(flags & 1 != 0);
}

test "ALU: Basic Operations (i8)" {
    // _add without carry
    const res = alu(._add, i8, 5, 3, null, null);
    try expectEqual(@as(i8, 8), res.ret);
    try expectEqual(null, res.reth);
    try expectEqual(@as(u1, 0), getCarry(res.fl)); // No carry

    //     // _add with carry
    //     res = alu(._add, i8, 127, 1, null, 1);
    //     try expectEqual(@as(i8, -128), res.ret); // 127 + 1 + 1 = 129, wraps to -128
    //     try expectEqual(@as(u1, 1), getCarry(res.fl)); // Carry out

    //     // _sub without carry
    //     res = alu(._sub, i8, 5, 3, null, null);
    //     try expectEqual(@as(i8, 2), res.ret);
    //     try expectEqual(@as(u1, 0), getCarry(res.fl)); // No borrow

    //     // _sub with carry
    //     res = alu(._sub, i8, 0, 1, null, 1);
    //     try expectEqual(@as(i8, -2), res.ret); // 0 - 1 - 1 = -2
    //     try expectEqual(@as(u1, 1), getCarry(res.fl)); // Borrow

    //     // _and
    //     res = alu(._and, i8, 5, 3, null, null);
    //     try expectEqual(@as(i8, 1), res.ret); // 0101 & 0011 = 0001

    //     // _or
    //     res = alu(._or, i8, 5, 3, null, null);
    //     try expectEqual(@as(i8, 7), res.ret); // 0101 | 0011 = 0111

    //     // _xor
    //     res = alu(._xor, i8, 5, 3, null, null);
    //     try expectEqual(@as(i8, 6), res.ret); // 0101 ^ 0011 = 0110
    // }

    // test "ALU: Shift Operations (i8)" {
    //     // _shl
    //     var res = alu(._shl, i8, 1, 2, null, null);
    //     try expectEqual(@as(i8, 4), res.ret); // 0001 << 2 = 0100
    //     try expectEqual(@as(u1, 0), getCarry(res.fl)); // No carry

    //     res = alu(._shl, i8, 64, 1, null, null); // ascended from 0x0
    //     try expectEqual(@as(i8, -128), res.ret); // 01000000 << 1 = 10000000
    //     try expectEqual(@as(u1, 0), getCarry(res.fl)); // No carry

    //     // _shr (signed)
    //     res = alu(._shr, i8, -8, 2, null, null);
    //     try expectEqual(@as(i8, -2), res.ret); // 11111000 >> 2 = 11111110 (sar)
    //     try expectEqual(@as(u1, 0), getCarry(res.fl)); // Last bit shifted out

    //     // _sar
    //     res = alu(._sar, i8, -8, 3, null, null);
    //     try expectEqual(@as(i8, -1), res.ret); // 11111000 >> 3 = 11111111
    //     try expectEqual(@as(u1, 1), getCarry(res.fl)); // Last bit shifted out
    // }

    // test "ALU: Shift Operations (u8)" {
    //     // _shr (unsigned)
    //     const res = alu(._shr, u8, 8, 2, null, null);
    //     try expectEqual(@as(u8, 2), res.ret); // 00001000 >> 2 = 00000010 (shr)
    //     try expectEqual(@as(u1, 0), getCarry(res.fl));
    // }

    // test "ALU: Multiplication and Division (i16)" {
    //     // _mul without high bits
    //     var res = alu(._mul, i16, 5, 3, null, null);
    //     try expectEqual(@as(i16, 15), res.ret);
    //     try expectEqual(@as(?i16, 0), res.reth); // High bits

    //     // _mul with overflow
    //     res = alu(._mul, i16, 256, 128, null, null);
    //     try expectEqual(@as(i16, 0), res.ret); // 32768 & 0xFFFF = 0
    //     try expectEqual(@as(?i16, 1), res.reth); // High bits = 1

    //     // _div without high bits
    //     res = alu(._div, i16, 15, 3, null, null);
    //     try expectEqual(@as(i16, 5), res.ret); // Quotient
    //     try expectEqual(null, res.reth);

    //     // _div with high bits
    //     res = alu(._div, i16, 1, 2, 1, null);
    //     try expectEqual(@as(i16, -32768), res.ret); // (1<<16 + 1) / 2 = 32768, wraps to -32768 in i16
    // }

    // test "ALU: Edge Cases" {
    //     // _add overflow
    //     var res = alu(._add, i8, 127, 127, null, null);
    //     try expectEqual(@as(i8, -2), res.ret); // 127 + 127 = 254, wraps to -2
    //     try expectEqual(@as(u1, 1), getCarry(res.fl));

    //     // _sub underflow
    //     res = alu(._sub, i8, -128, 1, null, null);
    //     try expectEqual(@as(i8, 127), res.ret); // -128 - 1 = -129, wraps to 127
    //     try expectEqual(@as(u1, 1), getCarry(res.fl));

    //     // _mul large numbers
    //     //res = alu(._mul, i32, 65536, 65536, null, null);
    //     //try expectEqual(@as(i32, 0), res.ret); // Low 32 bits
    //     //try expectEqual(@as(?i32, 1), res.reth); // High 32 bits

    //     // _div by zero (behavior depends on hardware, may trap)
    //     //res = alu(._div, i16, 10, 0, null, null); // Uncomment to test, expect trap
    // }

    // test "ALU: Carry-In Variations" {
    //     // _add with carry_in = 1
    //     var res = alu(._add, i8, 5, 3, null, 1);
    //     try expectEqual(@as(i8, 9), res.ret); // 5 + 3 + 1
    //     try expectEqual(@as(u1, 0), getCarry(res.fl));

    //     // _sub with carry_in = 1
    //     res = alu(._sub, i8, 5, 3, null, 1);
    //     try expectEqual(@as(i8, 1), res.ret); // 5 - 3 - 1
    //     try expectEqual(@as(u1, 0), getCarry(res.fl));

    //     // _add with carry_in = 0
    //     res = alu(._add, i8, 127, 1, null, 0);
    //     try expectEqual(@as(i8, 0), res.ret); // 255 + 1, wraps
    //     try expectEqual(@as(u1, 1), getCarry(res.fl));
}
