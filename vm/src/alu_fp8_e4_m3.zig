const std = @import("std");

pub const Fp8E4M3 = u8;

// Unpacks an FP8 number into sign, exponent, and mantissa
pub fn unpack_fp8(x: Fp8E4M3) struct { sign: u1, exp: u4, mant: u3 } {
    return .{
        .sign = @intCast((x >> 7) & 1),
        .exp = @intCast((x >> 3) & 0b1111),
        .mant = @intCast(x & 0b111),
    };
}

// Packs sign, exponent, and mantissa into an FP8 number
pub fn pack_fp8(sign: u1, exp: u4, mant: u3) Fp8E4M3 {
    return (@as(u8, sign) << 7) | (@as(u8, exp) << 3) | mant;
}

// Addition of two FP8 numbers
pub fn add_fp8(a: Fp8E4M3, b: Fp8E4M3) Fp8E4M3 {
    const aa = unpack_fp8(a);
    const bb = unpack_fp8(b);

    // Special cases
    if (aa.exp == 15 or bb.exp == 15) {
        if (aa.exp == 15 and aa.mant != 0) return a; // NaN
        if (bb.exp == 15 and bb.mant != 0) return b; // NaN
        if (aa.exp == 15 and bb.exp == 15) {
            if (aa.sign == bb.sign) return a; // Inf + Inf = Inf
            return pack_fp8(0, 15, 1); // Inf + -Inf = NaN
        }
        if (aa.exp == 15) return a; // Inf + finite = Inf
        if (bb.exp == 15) return b; // finite + Inf = Inf
    }
    if (aa.exp == 0 and aa.mant == 0) return b;
    if (bb.exp == 0 and bb.mant == 0) return a;

    // Effective exponents and signed mantissas (6 fraction bits)
    const e_a = if (aa.exp == 0) @as(u4, 1) else aa.exp;
    const e_b = if (bb.exp == 0) @as(u4, 1) else bb.exp;
    // Perform mantissa scaling in i16 to avoid u3 overflow
    const mant_scaled_a: i16 = @as(i16, aa.mant) << 3;
    const mant_scaled_b: i16 = @as(i16, bb.mant) << 3;
    const mant_int_a: i16 = if (aa.exp == 0)
        (if (aa.sign == 1) -mant_scaled_a else mant_scaled_a)
    else
        (if (aa.sign == 1) -(@as(i16, 64) + mant_scaled_a) else (@as(i16, 64) + mant_scaled_a));
    const mant_int_b: i16 = if (bb.exp == 0)
        (if (bb.sign == 1) -mant_scaled_b else mant_scaled_b)
    else
        (if (bb.sign == 1) -(@as(i16, 64) + mant_scaled_b) else (@as(i16, 64) + mant_scaled_b));

    // Align mantissas
    const e_max = @max(e_a, e_b);
    const d_a = e_max - e_a;
    const d_b = e_max - e_b;
    const shifted_mant_a = if (d_a > 0) mant_int_a >> @as(u3, @min(d_a, 7)) else mant_int_a;
    const shifted_mant_b = if (d_b > 0) mant_int_b >> @as(u3, @min(d_b, 7)) else mant_int_b;
    const sum_mant = shifted_mant_a + shifted_mant_b;

    if (sum_mant == 0) return 0;

    const s_result: u1 = @intCast(if (sum_mant < 0) @as(u1, 1) else @as(u1, 0));
    const abs_sum_mant = @abs(sum_mant);

    // Normalize
    const p = if (abs_sum_mant == 0) 0 else 15 - @clz(abs_sum_mant);
    const shift = @as(i5, @intCast(p)) - 6;
    var e_result = @as(i8, e_max) + shift;
    var mant_shifted = if (shift > 0) abs_sum_mant >> @intCast(shift) else abs_sum_mant << @intCast(-shift);
    if (e_result <= 0) {
        // Handle subnormal by shifting mantissa
        const sub_shift = @as(u3, @intCast(1 - e_result));
        mant_shifted = mant_shifted >> sub_shift;
        e_result = 0;
    }
    if (e_result >= 15) return pack_fp8(s_result, 15, 0); // Overflow
    const m_result: u3 = @intCast((mant_shifted >> 3) & 7);
    return pack_fp8(s_result, @intCast(e_result), m_result);    
}

// Negates an FP8 number by flipping the sign bit
pub fn negate_fp8(x: Fp8E4M3) Fp8E4M3 {
    return x ^ (1 << 7);
}

// Subtraction using addition
pub fn sub_fp8(a: Fp8E4M3, b: Fp8E4M3) Fp8E4M3 {
    return add_fp8(a, negate_fp8(b));
}

fn mul_fp8_std(a: u8, b: u8) u8 {
    // Unpack inputs
    const s_a = (a >> 7) & 1;
    const e_a = (a >> 3) & 0xF;
    const m_a = a & 7;
    const s_b = (b >> 7) & 1;
    const e_b = (b >> 3) & 0xF;
    const m_b = b & 7;

    // Compute sign
    const s_result = s_a ^ s_b;

    // Special cases
    if (e_a == 15 or e_b == 15) {
        if (e_a == 15 and m_a != 0) return a; // NaN
        if (e_b == 15 and m_b != 0) return b; // NaN
        if (e_a == 15 and e_b == 0) return pack_fp8(0, 15, 1); // inf * zero = NaN
        if (e_b == 15 and e_a == 0) return pack_fp8(0, 15, 1); // zero * inf = NaN
        if (e_a == 15 and e_b == 15) return pack_fp8(@truncate(s_result), 15, 0); // inf * inf = inf
        return pack_fp8(@truncate(s_result), 15, 0); // inf * finite = inf
    }
    // Handle zero cases
    if ((e_a == 0 and m_a == 0) or (e_b == 0 and m_b == 0)) return 0;

    // Mantissas with implicit 1 (4-bit: 1.mmm)
    const mant_int_a = 8 + m_a; // 1000 + m_a
    const mant_int_b = 8 + m_b; // 1000 + m_b

    // Multiply mantissas
    const p_int: u8 = mant_int_a * mant_int_b; // Max 8 bits

    // Normalize: find leading 1 position
    const p: u8 = if (p_int == 0) 0 else @as(u8, 7) - @clz(p_int);
    const shift: i5 = @intCast(p - 6); // Normalize to bit 6

    // Compute exponent
    const e_result = @as(i8, @intCast(e_a)) + @as(i8, @intCast(e_b)) - 7 + shift;
    if (e_result <= 0) return 0; // Underflow
    if (e_result >= 15) return (s_result << 7) | (15 << 3); // Overflow

    // Shift mantissa and extract 3 bits
    const mant_shifted = if (shift >= 0) p_int >> @intCast(shift) else p_int << @intCast(-shift);
    const m_result: u3 = @intCast((mant_shifted >> 3) & 7); // Bits 4-6 after implicit 1

    // Pack result
   // return (s_result << 7) | (@intCast(u4, e_result) << 3) | m_result;
    return pack_fp8(@intCast(s_result), @intCast(e_result), m_result);
}

// Mantissa multiplication LUT structure
const MantMulResult = struct { m_prod: u3, k: u1 };

// Precomputed mantissa multiplication LUT (triangular, i <= j)
const mant_mul_lut: [36]MantMulResult = blk: {
    var lut: [36]MantMulResult = undefined;
    var idx: usize = 0;
    for (0..8) |i| {
        for (i..8) |j| {
            const m1 = @as(f32, 1.0) + @as(f32, @floatFromInt(i)) / 8.0;
            const m2 = @as(f32, 1.0) + @as(f32, @floatFromInt(j)) / 8.0;
            var p = m1 * m2;
            var k: u1 = 0;
            if (p >= 2.0) {
                p /= 2.0;
                k = 1;
            }
            const m_prod: i8 = @intFromFloat(@round((p - 1.0) * 8.0));
            lut[idx] = .{ .m_prod = @intCast(@min(m_prod, 7)), .k = k };
            idx += 1;
        }
    }
    break :blk lut;
};

fn mul_fp8_lut(a: Fp8E4M3, b: Fp8E4M3) Fp8E4M3 {
    // Unpack inputs
    const s_a = (a >> 7) & 1;
    const e_a = (a >> 3) & 0xF;
    const m_a = a & 7;
    const s_b = (b >> 7) & 1;
    const e_b = (b >> 3) & 0xF;
    const m_b = b & 7;

    // Compute sign
    const s_result = s_a ^ s_b;

    // Special cases
    if (e_a == 15 or e_b == 15) {
        if (e_a == 15 and m_a != 0) return a; // NaN
        if (e_b == 15 and m_b != 0) return b; // NaN
        if (e_a == 15 and e_b == 0) return pack_fp8(0, 15, 1); // inf * zero = NaN
        if (e_b == 15 and e_a == 0) return pack_fp8(0, 15, 1); // zero * inf = NaN
        if (e_a == 15 and e_b == 15) return pack_fp8(@truncate(s_result), 15, 0); // inf * inf = inf
        return pack_fp8(@truncate(s_result), 15, 0); // inf * finite = inf
    }
    // Handle zero cases
    if ((e_a == 0 and m_a == 0) or (e_b == 0 and m_b == 0)) return 0;

    // Use LUT for mantissa product
    const i = m_a;
    const j = m_b;
    const idx = if (i <= j) (i * (i + 1) / 2 + j) else (j * (j + 1) / 2 + i);
    const lut_result = mant_mul_lut[idx];

 // Compute exponent
    const e_result = @as(i8, @intCast(e_a)) + @as(i8, @intCast(e_b)) - 7 + @as(i8, lut_result.k);
    if (e_result <= 0) return 0; // Underflow
    if (e_result >= 15) return pack_fp8(@truncate(s_result), 15, 0); // Overflow

    // Pack result
    return pack_fp8(@truncate(s_result), @intCast(e_result), lut_result.m_prod);
}

const mul_fp8 = mul_fp8_lut;

// Division using reciprocal LUT
pub fn div_fp8(a: Fp8E4M3, b: Fp8E4M3) Fp8E4M3 {
    return mul_fp8(a, reciprocal_lut[b]);
}

// Helper functions for LUT and testing
fn fp8_to_f32(x: Fp8E4M3) f32 {
    const u = unpack_fp8(x);
    if (u.exp == 15) {
        if (u.mant == 0) return if (u.sign == 1) -std.math.inf(f32) else std.math.inf(f32);
        return std.math.nan(f32);
    }
    if (u.exp == 0) {
        if (u.mant == 0) return 0.0;
        return @as(f32, if (u.sign == 1) -1.0 else 1.0) * @as(f32, @floatFromInt(u.mant)) / 8.0 * std.math.pow(f32, 2.0, -6.0);
    }
    return @as(f32, if (u.sign == 1) -1.0 else 1.0) * (1.0 + @as(f32, @floatFromInt(u.mant)) / 8.0) * std.math.pow(f32, 2.0, @as(f32, @floatFromInt(u.exp)) - 7.0);
}

fn f32_to_fp8(x: f32) Fp8E4M3 {
    if (x == 0.0) return 0;
    if (std.math.isInf(x)) return pack_fp8(if (x < 0) 1 else 0, 15, 0);
    if (std.math.isNan(x)) return pack_fp8(0, 15, 1);
    const sign = if (x < 0) @as(u1, 1) else @as(u1, 0);
    const abs_x = @abs(x);
    const exp_f32 = std.math.floor(std.math.log2(abs_x));
    const mant_f32 = abs_x / std.math.pow(f32, 2.0, exp_f32) - 1.0;
    const e = @as(i8, @intFromFloat(exp_f32)) + 7;
    const m: i8 = @intFromFloat(@round(mant_f32 * 8.0));
    if (e <= 0) {
        if (e < -6) return 0;
        const shift = @as(u3, @intCast(1 - e));
        const m3: u3 = @intCast((m >> shift) & 7);
        return pack_fp8(sign, 0, m3);
    }
    if (e >= 15) return pack_fp8(sign, 15, 0);
    return pack_fp8(sign, @intCast(e), @min(m, 7));
}

const reciprocal_lut: [256]Fp8E4M3 = blk: {
     @setEvalBranchQuota(100000); // Increase branch quota for LUT computation
    var lut: [256]Fp8E4M3 = undefined;
    for (0..256) |i| {
        const x = @as(Fp8E4M3, @intCast(i));
        const x_f32 = fp8_to_f32(x);
        if (x_f32 == 0.0) {
            lut[i] = pack_fp8(0, 15, 0); // Infinity
        } else if (std.math.isInf(x_f32)) {
            lut[i] = 0; // Zero
        } else if (std.math.isNan(x_f32)) {
            lut[i] = pack_fp8(0, 15, 1); // NaN
        } else {
            lut[i] = f32_to_fp8(1.0 / x_f32);
        }
    }
    break :blk lut;
};

// Unit tests
test "FP8 arithmetic operations" {

    // Define constants
    const zero = pack_fp8(0, 0, 0);              // 0
    const one = pack_fp8(0, 7, 0);               // 1.0
    const two = pack_fp8(0, 8, 0);               // 2.0
    const neg_one = pack_fp8(1, 7, 0);           // -1.0
    const inf = pack_fp8(0, 15, 0);              // Infinity
    const neg_inf = pack_fp8(1, 15, 0);          // -Infinity
    const nan = pack_fp8(0, 15, 1);              // NaN
    const sub_one = pack_fp8(0, 0, 1);           // 0.125 * 2^(-6)
    const sub_two = pack_fp8(0, 0, 2);           // 0.25 * 2^(-6)
    const max_normal = pack_fp8(0, 14, 7);       // 1.875 * 2^7
    const min_normal = pack_fp8(0, 1, 0);        // 1.0 * 2^(-6)
//  const neg_max_normal = pack_fp8(1, 14, 7);   // -1.875 * 2^7

    // Addition tests
    // Regular cases
    try std.testing.expectEqual(zero, add_fp8(zero, zero));
    try std.testing.expectEqual(two, add_fp8(one, one));
    try std.testing.expectEqual(zero, add_fp8(one, neg_one));
    try std.testing.expectEqual(one, add_fp8(one, zero));

    // Subnormals
    try std.testing.expectEqual(pack_fp8(0, 0, 3), add_fp8(sub_one, sub_two));
    try std.testing.expectEqual(min_normal, add_fp8(sub_one, pack_fp8(0, 0, 7)));

    // Corner cases
    try std.testing.expectEqual(nan, add_fp8(inf, neg_inf));
    try std.testing.expectEqual(inf, add_fp8(inf, inf));
    try std.testing.expectEqual(inf, add_fp8(inf, one));
    try std.testing.expectEqual(nan, add_fp8(nan, one));
    try std.testing.expectEqual(max_normal, add_fp8(max_normal, zero)); // Overflow check
    try std.testing.expectEqual(zero, add_fp8(sub_one, negate_fp8(sub_one)));

    // Subtraction tests
    try std.testing.expectEqual(zero, sub_fp8(one, one));
    try std.testing.expectEqual(two, sub_fp8(two, zero));
    try std.testing.expectEqual(neg_one, sub_fp8(zero, one));
    try std.testing.expectEqual(nan, sub_fp8(inf, inf));
    try std.testing.expectEqual(inf, sub_fp8(inf, neg_one));
    try std.testing.expectEqual(zero, sub_fp8(sub_one, sub_one));

    // Multiplication tests
    // Regular cases
    try std.testing.expectEqual(zero, mul_fp8(zero, one));
    try std.testing.expectEqual(two, mul_fp8(one, two));
    try std.testing.expectEqual(neg_one, mul_fp8(one, neg_one));

    // Subnormals
    try std.testing.expectEqual(zero, mul_fp8(sub_one, sub_two)); // Underflow
    try std.testing.expectEqual(zero, mul_fp8(sub_one, one));    // Small result

    // Corner cases
    try std.testing.expectEqual(inf, mul_fp8(inf, two));
    try std.testing.expectEqual(nan, mul_fp8(inf, zero));
    try std.testing.expectEqual(inf, mul_fp8(inf, inf));
    try std.testing.expectEqual(nan, mul_fp8(nan, one));
    try std.testing.expectEqual(inf, mul_fp8(max_normal, two)); // Overflow

    // Division tests
    // Regular cases
    try std.testing.expectEqual(one, div_fp8(two, two));
    try std.testing.expectEqual(two, div_fp8(two, one));
    try std.testing.expectEqual(neg_one, div_fp8(one, neg_one));

    // Corner cases
    try std.testing.expectEqual(inf, div_fp8(one, zero));
    try std.testing.expectEqual(zero, div_fp8(one, inf));
    try std.testing.expectEqual(nan, div_fp8(inf, inf));
    try std.testing.expectEqual(nan, div_fp8(zero, zero));
    try std.testing.expectEqual(nan, div_fp8(nan, one));
    try std.testing.expectEqual(zero, div_fp8(sub_one, two)); // Underflow
    try std.testing.expectEqual(inf, div_fp8(max_normal, sub_one)); // Overflow
}
