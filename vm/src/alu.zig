const std = @import("std");
const expectEqual = std.testing.expectEqual;
const expectError = std.testing.expectError;

pub const ALUOperation = enum(u16) {
    _add = 0,
    _addc,
    _sub,
    _subc,
    _and,
    _or,
    _xor,
    _shl,
    _shr,
    _mul,
    _div,
    _lookup,
    _load,
    _store,
};

pub const ALUDataType = enum(u8) {
    _u8 = 0,
    _i8,
    _u16,
    _i16,
    _u32,
    _i32,
    _u64,
    _i64,
    _f32,
    _f64,
    _f16,
    _fp4,
    _fp8,
    _u1,
    _i4,

    pub fn getType(self: ALUDataType) type {
        return switch (self) {
            ._u8 => u8,
            ._i8 => i8,
            ._u16 => u16,
            ._i16 => i16,
            ._u32 => u32,
            ._i32 => i32,
            ._u64 => u64,
            ._i64 => i64,
            ._f32 => f32,
            ._f64 => f64,
            ._f16 => f16,
            ._fp4 => u4, // Note: Zig does not have a native fp4 type, using u4 as a placeholder
            ._fp8 => u8, // Note: Zig does not have a native fp8 type, using u8 as a placeholder
            ._u1 => u1,
            ._i4 => i4,
        };
    }
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

pub const ALUError = error{
    DivideByZero,
    NotImplemented,
};

pub fn alu(
    op: ALUOperation,
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
    const signedness = type_info.int.signedness;
    const DoubleT = DWT(T);
    const c_in: u1 = carry_in orelse 0;
    var result: T = undefined;
    var high_result: ?T = null;
    var carry: ?u1 = null;

    switch (op) {
        ._add, ._addc => {
            const first_add = @addWithOverflow(arg1, arg2);
            result = first_add[0];
            var overflow = first_add[1];
            if ((op == ._addc) and (c_in == 1)) {
                const second_add = @addWithOverflow(result, 1);
                result = second_add[0];
                overflow |= second_add[1];
            }
            carry = overflow;
        },
        ._sub, ._subc => {
            const sub_result = @subWithOverflow(arg1, arg2);
            result = sub_result[0];
            var overflow = sub_result[1];
            if ((op == ._subc) and (c_in == 1)) {
                const borrow_result = @subWithOverflow(result, 1);
                result = borrow_result[0];
                overflow |= borrow_result[1];
            }
            carry = overflow;
        },
        ._and => {
            result = arg1 & arg2;
            carry = null;
        },
        ._or => {
            result = arg1 | arg2;
            carry = null;
        },
        ._xor => {
            result = arg1 ^ arg2;
            carry = null;
        },
        ._shl => {
            const shift = @as(u3, @intCast(arg2 & 0x07));
            result = @as(T, @truncate(arg1 << shift));
        },
        ._shr => {
            const shift = @as(u3, @intCast(arg2 & 0x07));
            result = arg1 >> shift;
        },
        ._mul => {
            const wide_result: DoubleT = @as(DoubleT, arg1) * @as(DoubleT, arg2);
            result = @as(T, @truncate(wide_result));
            high_result = @as(T, @truncate(wide_result >> @bitSizeOf(T)));
            carry = null;
        },
        ._div => {
            if (arg2 == 0) {
                return error.DivideByZero;
            }
            const dividend_high: T = arg1h orelse 0;
            const dividend: DoubleT = (@as(DoubleT, dividend_high) << @bitSizeOf(T)) | @as(DoubleT, arg1);
            if (signedness == .unsigned) {
                const quotient: DoubleT = @divTrunc(dividend, @as(DoubleT, arg2));
                result = @as(T, @truncate(quotient));
                high_result = null;
                carry = null;
            } else {
                const quotient: DoubleT = @divTrunc(dividend, @as(DoubleT, arg2));
                result = @as(T, @truncate(quotient));
                high_result = null;
                carry = null;
            }
        },
        ._lookup, ._load, ._store => {
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
    // Test Addition (u8, unsigned)
    const add_u8 = try alu(._add, u8, 200, null, 100, null);
    try expectEqual(@as(u8, 44), add_u8.ret); // 200 + 100 = 300 (overflow: 44)
    try expectEqual(null, add_u8.reth);
    try expectEqual(@as(u1, 1), add_u8.carry_out);

    // Test Addition with carry-in (u8)
    const add_u8_carry = try alu(._addc, u8, 200, null, 50, 1);
    try expectEqual(@as(u8, 251), add_u8_carry.ret); // 200 + 50 + 1 = 251
    try expectEqual(null, add_u8_carry.reth);
    try expectEqual(@as(u1, 0), add_u8_carry.carry_out);

    // Test Addition (i8, signed)
    const add_i8 = try alu(._add, i8, 100, null, 50, null);
    try expectEqual(@as(i8, -106), add_i8.ret); // 100 + 50 = 150 (overflow: -106)
    try expectEqual(null, add_i8.reth);
    try expectEqual(@as(u1, 1), add_i8.carry_out); // Signed overflow

    // Test Subtraction (u8, unsigned)
    const sub_u8 = try alu(._sub, u8, 100, null, 150, null);
    try expectEqual(@as(u8, 206), sub_u8.ret); // 100 - 150 = -50 (underflow: 206)
    try expectEqual(null, sub_u8.reth);
    try expectEqual(@as(u1, 1), sub_u8.carry_out);

    // Test Subtraction with borrow-in (u16)
    const sub_u16 = try alu(._subc, u16, 1000, null, 500, 1);
    try expectEqual(@as(u16, 499), sub_u16.ret); // 1000 - 500 - 1 = 499
    try expectEqual(null, sub_u16.reth);
    try expectEqual(@as(u1, 0), sub_u16.carry_out);

    // Test Subtraction (i8, signed)
    const sub_i8 = try alu(._sub, i8, 50, null, 100, null);
    try expectEqual(@as(i8, -50), sub_i8.ret); // 50 - 100 = -50
    try expectEqual(null, sub_i8.reth);
    try expectEqual(@as(u1, 0), sub_i8.carry_out);

    // Test Bitwise AND (u8)
    const and_u8 = try alu(._and, u8, 0b10101010, null, 0b11001100, null);
    try expectEqual(@as(u8, 0b10001000), and_u8.ret); // 0b10101010 & 0b11001100
    try expectEqual(null, and_u8.reth);
    try expectEqual(null, and_u8.carry_out);

    // Test Bitwise OR (u16)
    const or_u16 = try alu(._or, u16, 0xFF00, null, 0x00FF, null);
    try expectEqual(@as(u16, 0xFFFF), or_u16.ret); // 0xFF00 | 0x00FF
    try expectEqual(null, or_u16.reth);
    try expectEqual(null, or_u16.carry_out);

    // Test Bitwise XOR (i8)
    const xor_i8 = try alu(._xor, i8, 0b0101010, null, 0b1001100, null);
    try expectEqual(@as(i8, 0b01100110), xor_i8.ret); // 0b10101010 ^ 0b11001100
    try expectEqual(null, xor_i8.reth);
    try expectEqual(null, xor_i8.carry_out);

    // Test Shift Left (u8)
    const shl_u8 = try alu(._shl, u8, 0b11001100, null, 2, null);
    try expectEqual(@as(u8, 0b00110000), shl_u8.ret); // 0b11001100 << 2
    try expectEqual(null, shl_u8.reth);

    // Test Shift Left (i16, large shift)
    const shl_i16 = try alu(._shl, i16, 0x800, null, 15, null);
    try expectEqual(@as(i16, 0), shl_i16.ret); // 0x800 << 16 = 0
    try expectEqual(null, shl_i16.reth);

    // Test Logical Shift Right (u8)
    const shr_u8 = try alu(._shr, u8, 0b11001100, null, 2, null);
    try expectEqual(@as(u8, 0b00110011), shr_u8.ret); // 0b11001100 >> 2
    try expectEqual(null, shr_u8.reth);

    // Test Arithmetic Shift Right (i8)
    const sar_i8 = try alu(._shr, i8, -100, null, 2, null);
    try expectEqual(@as(i8, -25), sar_i8.ret); // -100 >> 2 (sign-extended)
    try expectEqual(null, sar_i8.reth);

    // Test Arithmetic Shift Right (u8, behaves as logical)
    const sar_u8 = try alu(._shr, u8, 0b11001100, null, 2, null);
    try expectEqual(@as(u8, 0b00110011), sar_u8.ret); // 0b11001100 >> 2
    try expectEqual(null, sar_u8.reth);

    // Test Multiplication (u8)
    const mul_u8 = try alu(._mul, u8, 255, null, 255, null);
    try expectEqual(@as(u8, 1), mul_u8.ret); // 255 * 255 = 65025 (low: 1)
    try expectEqual(@as(u8, 254), mul_u8.reth); // High: 254
    try expectEqual(null, mul_u8.carry_out);

    // Test Multiplication (i16)
    const mul_i16 = try alu(._mul, i16, -100, null, 100, null);
    try expectEqual(@as(i16, -10000), mul_i16.ret); // -100 * 100 = -10000 (low)
    try expectEqual(@as(i16, -1), mul_i16.reth); // High: -1 (sign-extended)
    try expectEqual(null, mul_i16.carry_out);

    // Test Division (u16)
    const div_u16 = try alu(._div, u16, 1000, 1, 500, null);
    try expectEqual(@as(u16, 133), div_u16.ret); // (1 << 16 + 1000) / 500 ≈ 133
    try expectEqual(null, div_u16.reth);
    try expectEqual(null, div_u16.carry_out);

    // Test Division (i8)
    const div_i8 = try alu(._div, i8, -100, -1, 5, null);
    try expectEqual(@as(i8, -20), div_i8.ret);
    try expectEqual(null, div_i8.reth);
    try expectEqual(null, div_i8.carry_out);

    // Test Division by Zero
    try std.testing.expectError(error.DivideByZero, @call(.auto, alu, .{ ._div, u8, 100, null, 0, null }));
}
