const std = @import("std");
const defs = @import("definitions.zig");

pub const ALUError = error{
    DivideByZero,
    NotImplemented,
};

fn DWT(comptime T: type) type {
    const type_info = @typeInfo(T);
    if (type_info != .int) {
        @compileError("T must be an integer type");
    }
    const bits = type_info.int.bits;
    const signedness = type_info.int.signedness;
    return @Type(.{
        .int = .{
            .bits = bits * 2,
            .signedness = signedness,
        },
    });
}

fn UT(comptime T: type) type {
    const type_info = @typeInfo(T);
    if (type_info != .int) {
        @compileError("T must be an integer type");
    }
    const bits = type_info.int.bits;
    return @Type(.{
        .int = .{
            .bits = bits,
            .signedness = .unsigned,
        },
    });
}

pub fn alu(
    op: defs.AMOD,
    comptime T: type,
    arg1: T,
    arg1h: ?T,
    arg2: T,
    carry_in: ?u1,
) ALUError!struct {
    ret: T,
    reth: ?T,
    carry_out: ?u1,
} {
    const type_info = @typeInfo(T);
    if (type_info != .int) {
        @compileError("T must be an integer type");
    }
    //const signedness = type_info.int.signedness;
    const DoubleT = DWT(T);
    const c_in: u1 = carry_in orelse 0;
    var result: T = undefined;
    var high_result: ?T = null;
    var carry: ?u1 = null;

    switch (op) {
        .add => {
            const first_add = @addWithOverflow(arg1, arg2);
            result = first_add[0];
            var overflow = first_add[1];
            if (c_in == 1) {
                const second_add = @addWithOverflow(result, 1);
                result = second_add[0];
                overflow |= second_add[1];
            }
            carry = overflow;
        },
        .sub => {
            const sub_result = @subWithOverflow(arg1, arg2);
            result = sub_result[0];
            var overflow = sub_result[1];
            if (c_in == 1) {
                const borrow_result = @subWithOverflow(result, 1);
                result = borrow_result[0];
                overflow |= borrow_result[1];
            }
            carry = overflow;
        },
        .and_ => {
            result = arg1 & arg2;
            carry = null;
        },
        .or_ => {
            result = arg1 | arg2;
            carry = null;
        },
        .xor_ => {
            result = arg1 ^ arg2;
            carry = null;
        },
        .shl => {
            const shift: UT(T) = @bitCast(arg2);
            result = arg1 << @truncate(shift);
        },
        .shr => {
            const shift: UT(T) = @bitCast(arg2);
            result = arg1 >> @truncate(shift);
        },
        .mul => {
            const wide_result: DoubleT = @as(DoubleT, arg1) * @as(DoubleT, arg2);
            result = @truncate(wide_result);
            high_result = @truncate(wide_result >> @bitSizeOf(T));
            carry = null;
        },
        .div => {
            if (arg2 == 0) {
                return error.DivideByZero;
            }
            const dividend_high: T = arg1h orelse 0;
            const dividend: DoubleT = (@as(DoubleT, dividend_high) << @bitSizeOf(T)) | @as(DoubleT, arg1);
            const quotient: DoubleT = @divTrunc(dividend, @as(DoubleT, arg2));
            result = @truncate(quotient);
            high_result = null;
            carry = null;
        },
        .lookup, .load, .store => {
            return error.NotImplemented;
        },
    }

    return .{
        .ret = result,
        .reth = high_result,
        .carry_out = carry,
    };
}

test "ALU operations" {
    // Test Addition (u8)
    const add_u8 = try alu(.add, u8, 200, null, 100, null);
    try std.testing.expectEqual(@as(u8, 44), add_u8.ret); // 200 + 100 = 300 (overflow: 44)
    try std.testing.expectEqual(null, add_u8.reth);
    try std.testing.expectEqual(@as(u1, 1), add_u8.carry_out);

    // Test Addition with carry-in (u8)
    const add_u8_carry = try alu(.add, u8, 200, null, 50, 1);
    try std.testing.expectEqual(@as(u8, 251), add_u8_carry.ret); // 200 + 50 + 1 = 251
    try std.testing.expectEqual(null, add_u8_carry.reth);
    try std.testing.expectEqual(@as(u1, 0), add_u8_carry.carry_out);

    // Test Subtraction (u16)
    const sub_u16 = try alu(.sub, u16, 1000, null, 500, null);
    try std.testing.expectEqual(@as(u16, 500), sub_u16.ret); // 1000 - 500 = 500
    try std.testing.expectEqual(null, sub_u16.reth);
    try std.testing.expectEqual(@as(u1, 0), sub_u16.carry_out);

    // Test Bitwise AND (u8)
    const and_u8 = try alu(.and_, u8, 0b10101010, null, 0b11001100, null);
    try std.testing.expectEqual(@as(u8, 0b10001000), and_u8.ret);
    try std.testing.expectEqual(null, and_u8.reth);
    try std.testing.expectEqual(null, and_u8.carry_out);

    // Test Shift Left (u8)
    const shl_u8 = try alu(.shl, u8, 0b11001100, null, 2, null);
    try std.testing.expectEqual(@as(u8, 0b00110000), shl_u8.ret); // 0b11001100 << 2
    try std.testing.expectEqual(null, shl_u8.reth);

    // Test Arithmetic Shift Right (i8)
    const shr_i8 = try alu(.shr, i8, -100, null, 2, null);
    try std.testing.expectEqual(@as(i8, -25), shr_i8.ret); // -100 >> 2
    try std.testing.expectEqual(null, shr_i8.reth);

    // Test Multiplication (u8)
    const mul_u8 = try alu(.mul, u8, 255, null, 255, null);
    try std.testing.expectEqual(@as(u8, 1), mul_u8.ret); // 255 * 255 = 65025 (low: 1)
    try std.testing.expectEqual(@as(u8, 254), mul_u8.reth.?); // High: 254
    try std.testing.expectEqual(null, mul_u8.carry_out);

    // Test Division (i16)
    const div_i16 = try alu(.div, i16, -100, null, 5, null);
    try std.testing.expectEqual(@as(i16, -20), div_i16.ret); // -100 / 5 = -20
    try std.testing.expectEqual(null, div_i16.reth);
    try std.testing.expectEqual(null, div_i16.carry_out);

    // Test Division by Zero
    try std.testing.expectError(error.DivideByZero, alu(.div, u8, 100, null, 0, null));
}
