const std = @import("std");
const defs = @import("definitions.zig");
const fp8 = @import("alu_fp8_e4_m3.zig");

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
    if (type_info != .int and type_info != .float and T != fp8.Fp8E4M3) {
        @compileError("T must be an integer or floating-point type");
    }
    const is_float = (type_info == .float) or (T == fp8.Fp8E4M3);
    const DoubleT = if (is_float) T else DWT(T);
    const c_in: u1 = carry_in orelse 0;
    var result: T = undefined;
    var high_result: ?T = null;
    var carry: ?u1 = null;

    switch (op) {
        .add => {
            if (is_float) {
                if (T == fp8.Fp8E4M3) {
                    result = fp8.add_fp8(arg1, arg2);
                } else {
                    result = arg1 + arg2;
                    carry = null;
                }
            } else {
                const first_add = @addWithOverflow(arg1, arg2);
                result = first_add[0];
                var overflow = first_add[1];
                if (c_in == 1) {
                    const second_add = @addWithOverflow(result, 1);
                    result = second_add[0];
                    overflow |= second_add[1];
                }
                carry = overflow;
            }
        },
        .sub => {
            if (is_float) {
                if (T == fp8.Fp8E4M3) {
                    result = fp8.sub_fp8(arg1, arg2);
                } else {
                    result = arg1 - arg2;
                    carry = null;
                }
            } else {
                const sub_result = @subWithOverflow(arg1, arg2);
                result = sub_result[0];
                var overflow = sub_result[1];
                if (c_in == 1) {
                    const borrow_result = @subWithOverflow(result, 1);
                    result = borrow_result[0];
                    overflow |= borrow_result[1];
                }
                carry = overflow;
            }
        },
        .and_ => {
            if (is_float) {
                return error.NotImplemented;
            }
            result = arg1 & arg2;
            carry = null;
        },
        .or_ => {
            if (is_float) {
                return error.NotImplemented;
            }
            result = arg1 | arg2;
            carry = null;
        },
        .xor_ => {
            if (is_float) {
                return error.NotImplemented;
            }
            result = arg1 ^ arg2;
            carry = null;
        },
        .shl => {
            if (is_float) {
                return error.NotImplemented;
            }
            const shift: UT(T) = @bitCast(arg2);
            result = arg1 << @truncate(shift);
        },
        .shr => {
            if (is_float) {
                return error.NotImplemented;
            }
            const shift: UT(T) = @bitCast(arg2);
            result = arg1 >> @truncate(shift);
        },
        .mul => {
            if (is_float) {
                if (T == fp8.Fp8E4M3) {
                    result = fp8.mul_fp8(arg1, arg2);
                } else {
                    result = arg1 * arg2;
                    carry = null;
                    high_result = null;
                }
            } else {
                const wide_result: DoubleT = @as(DoubleT, arg1) * @as(DoubleT, arg2);
                result = @truncate(wide_result);
                high_result = @truncate(wide_result >> @bitSizeOf(T));
                carry = null;
            }
        },
        .div => {
            if (is_float) {
                if (T == fp8.Fp8E4M3) {
                    if (arg2 == fp8.pack_fp8(0,0,0)) {
                        return error.DivideByZero;
                    }
                    result = fp8.mul_fp8(arg1, arg2);
                } else {
                    if (arg2 == 0.0) {
                        return error.DivideByZero;
                    }
                    result = arg1 / arg2;
                    carry = null;
                    high_result = null;
                }
            } else {
                if (arg2 == 0) {
                    return error.DivideByZero;
                }
                const dividend_high: T = arg1h orelse 0;
                const dividend: DoubleT = (@as(DoubleT, dividend_high) << @bitSizeOf(T)) | @as(DoubleT, arg1);
                const quotient: DoubleT = @divTrunc(dividend, @as(DoubleT, arg2));
                result = @truncate(quotient);
                high_result = null;
                carry = null;
            }
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
    // Integer tests
    const add_u8 = try alu(.add, u8, 200, null, 100, null);
    try std.testing.expectEqual(@as(u8, 44), add_u8.ret); // 200 + 100 = 300 (overflow: 44)
    try std.testing.expectEqual(null, add_u8.reth);
    try std.testing.expectEqual(@as(u1, 1), add_u8.carry_out);

    const add_u8_carry = try alu(.add, u8, 200, null, 50, 1);
    try std.testing.expectEqual(@as(u8, 251), add_u8_carry.ret); // 200 + 50 + 1 = 251
    try std.testing.expectEqual(null, add_u8_carry.reth);
    try std.testing.expectEqual(@as(u1, 0), add_u8_carry.carry_out);

    const sub_u16 = try alu(.sub, u16, 1000, null, 500, null);
    try std.testing.expectEqual(@as(u16, 500), sub_u16.ret); // 1000 - 500 = 500
    try std.testing.expectEqual(null, sub_u16.reth);
    try std.testing.expectEqual(@as(u1, 0), sub_u16.carry_out);

    const and_u8 = try alu(.and_, u8, 0b10101010, null, 0b11001100, null);
    try std.testing.expectEqual(@as(u8, 0b10001000), and_u8.ret);
    try std.testing.expectEqual(null, and_u8.reth);
    try std.testing.expectEqual(null, and_u8.carry_out);

    const shl_u8 = try alu(.shl, u8, 0b11001100, null, 2, null);
    try std.testing.expectEqual(@as(u8, 0b00110000), shl_u8.ret); // 0b11001100 << 2
    try std.testing.expectEqual(null, shl_u8.reth);

    const shr_i8 = try alu(.shr, i8, -100, null, 2, null);
    try std.testing.expectEqual(@as(i8, -25), shr_i8.ret); // -100 >> 2
    try std.testing.expectEqual(null, shr_i8.reth);

    const mul_u8 = try alu(.mul, u8, 255, null, 255, null);
    try std.testing.expectEqual(@as(u8, 1), mul_u8.ret); // 255 * 255 = 65025 (low: 1)
    try std.testing.expectEqual(@as(u8, 254), mul_u8.reth.?); // High: 254
    try std.testing.expectEqual(null, mul_u8.carry_out);

    const div_i16 = try alu(.div, i16, -100, null, 5, null);
    try std.testing.expectEqual(@as(i16, -20), div_i16.ret); // -100 / 5 = -20
    try std.testing.expectEqual(null, div_i16.reth);
    try std.testing.expectEqual(null, div_i16.carry_out);

    try std.testing.expectError(error.DivideByZero, alu(.div, u8, 100, null, 0, null));

    // Floating-point tests
    const add_f32 = try alu(.add, f32, 3.14, null, 2.86, null);
    try std.testing.expectApproxEqAbs(@as(f32, 6.0), add_f32.ret, 0.0001);
    try std.testing.expectEqual(null, add_f32.reth);
    try std.testing.expectEqual(null, add_f32.carry_out);

    const sub_f16 = try alu(.sub, f16, 10.5, null, 4.5, null);
    try std.testing.expectApproxEqAbs(@as(f16, 6.0), sub_f16.ret, 0.01);
    try std.testing.expectEqual(null, sub_f16.reth);
    try std.testing.expectEqual(null, sub_f16.carry_out);

    const mul_f64 = try alu(.mul, f64, 2.5, null, 4.0, null);
    try std.testing.expectApproxEqAbs(@as(f64, 10.0), mul_f64.ret, 0.0001);
    try std.testing.expectEqual(null, mul_f64.reth);
    try std.testing.expectEqual(null, mul_f64.carry_out);

    const div_f32 = try alu(.div, f32, 15.0, null, 3.0, null);
    try std.testing.expectApproxEqAbs(@as(f32, 5.0), div_f32.ret, 0.0001);
    try std.testing.expectEqual(null, div_f32.reth);
    try std.testing.expectEqual(null, div_f32.carry_out);

    try std.testing.expectError(error.DivideByZero, alu(.div, f32, 10.0, null, 0.0, null));
    try std.testing.expectError(error.NotImplemented, alu(.and_, f32, 1.0, null, 2.0, null));

    {
        //Fp8E4M3 tests
        const zero = fp8.pack_fp8(0, 0, 0);              // 0
        const one = fp8.pack_fp8(0, 7, 0);               // 1.0
        const two = fp8.pack_fp8(0, 8, 0);               // 2.0
        const neg_one = fp8.pack_fp8(1, 7, 0);           // -1.0

        const mul_fp8 = try alu(.mul, fp8.Fp8E4M3, neg_one, null, neg_one, null);
        try std.testing.expectEqual(one, mul_fp8.ret);

        const div_f8 = try alu(.div, fp8.Fp8E4M3, two, null, one, null);
        try std.testing.expectEqual(two, div_f8.ret);
        try std.testing.expectEqual(null, div_f8.reth);
        try std.testing.expectEqual(null, div_f8.carry_out);

        try std.testing.expectError(error.DivideByZero, alu(.div, fp8.Fp8E4M3, one, null, zero, null));

    }
}