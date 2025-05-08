// definitions.zig
const std = @import("std");

// ISA Constants
pub const WS = 64; // Word Size in bits (interpreted as 32 bits per your instruction)
pub const HWS = WS >> 1; // Half Word Size in bits (16 bits = 2 bytes)
pub const REGISTER_COUNT = 256; // Number of registers
pub const MAX_INSTRUCTION_SIZE = 1024; // memory size

// Prefixes
pub const PREFIX_OP2 = 0xC; // 2-byte instruction
pub const PREFIX_OP4 = 0xD; // 4-byte instruction
pub const PREFIX_OP8 = 0xE; // 8-byte instruction

// ALU Data Type (ADT)
pub const ADT = enum(u8) {
    u8 = 0,
    i8 = 1,
    u16 = 2,
    i16 = 3,
    u32 = 4,
    i32 = 5,
    u64 = 6,
    i64 = 7,
    f16 = 8,
    f32 = 9,
    f64 = 0xA,
    fp4 = 0xB,
    fp8 = 0xC,
    u4 = 0xD,
    i4 = 0xE,
    u1 = 0xF,

    // Check if ADT is a signed type
    pub fn signed(self: ADT) bool {
        return switch (self) {
            .i4, .i8, .i16, .i32, .i64 => true,
            else => false,
        };
    }

    // Return the number of bits needed to store the type, minimum 1 byte (8 bits)
    pub fn bits(self: ADT) u8 {
        return switch (self) {
            .u1 => 8, // 1 bit, but minimum 8 bits
            .u4, .i4 => 8, // 4 bits, but minimum 8 bits
            .fp4 => 8, // 4-bit float, but minimum 8 bits
            .u8, .i8 => 8, // 8 bits
            .fp8 => 8, // 8-bit float
            .u16, .i16 => 16, // 16 bits
            .f16 => 16, // 16-bit float
            .u32, .i32 => 32, // 32 bits
            .f32 => 32, // 32-bit float
            .u64, .i64 => 64, // 64 bits
            .f64 => 64, // 64-bit float
        };
    }

    pub fn getType(self: ADT) !type {
        return switch (self) {
            .u8 => u8,
            .i8 => i8,
            .u16 => u16,
            .i16 => i16,
            .u32 => u32,
            .i32 => i32,
            .u64 => u64,
            .i64 => i64,
            .f32 => f32,
            .f64 => f64,
            .f16 => f16,
            .u1 => u1,
            .i4 => i4,
            .fp4, .fp8 => return error.NotSupportedDataType,
        };
    }
};

// Branch Condition Selector (BCS)
pub const BCS = enum(u8) {
    always = 0,
    zero,
    not_zero,
    greater,
    greater_or_equal,
    less,
    less_or_equal,
    carry,
    not_carry,
    sign,
    not_sign,
    overflow,
    not_overflow,
    parity_even,
    parity_odd,
    interrupt,
};

// ALU Operation Mode (AMOD)
pub const AMOD = enum(u8) {
    add = 0x0,
    sub = 0x1,
    and_ = 0x2,
    or_ = 0x3,
    xor_ = 0x4,
    shl = 0x5,
    shr = 0x6,
    mul = 0x7,
    div = 0x8,
    lookup = 0x9,
    load = 0xA,
    store = 0xB,
};

// ALU_IO_CFG (R12)
pub const ALU_IO_CFG = packed struct {
    arg1: u8, // Register index for LI or first ALU operand
    arg2: u8, // Second ALU operand
    dst: u8, // Destination register
    ns: u8, // Nibble selector for LI
};

// ALU_MODE_CFG (R13)
pub const ALU_MODE_CFG = packed struct {
    adt: ADT, // ALU Data Type (ADT)
    vl: u8, // Vector Length in ADT units
    st_dst: i16, // Stride of destination register
};

// ALU_VR_STRIDES (R14)
pub const ALU_VR_STRIDES = packed struct {
    st_arg1: i16, // Stride of RS register
    st_arg2: i16, // Stride of SRC register
};

// BRANCH_CTRL (R15)
pub const BRANCH_CTRL = packed struct {
    bcs: u8, // Branch Condition Selector
    st_jmp: u24, // Jump stride
};

// Register indices for special registers
pub const R_ALU_IO_CFG = 12; // ALU_IO_CFG (RS, SRC, DST, NS)
pub const R_ALU_MODE_CFG = 13; // ALU_MODE_CFG (ADT, VL, ST_DST)
pub const R_ALU_VR_STRIDES = 14; // ALU_VR_STRIDES (ST_RS, ST_SRC)
pub const R_BRANCH_CTRL = 15; // BRANCH_CTRL (BCS, ST_JMP)
pub const R_ITP = 252; //  Interrupt Table Pointer
pub const R_BP = 253; // Base Pointer
pub const R_SP = 254; // Stack Pointer
pub const R_IP = 255; // Instruction Pointer

pub const FMT = enum(u3) {
    raw = 0, // Just output bytes
    dec, // Decimal
    hex, // Hex
    bin, // Binary
    fp0, // Rounded float
    fp2, // 2 decimal precision float
    fp4, // 4 decimal precision float
};

pub const OUT_FMT = packed struct {
    fmt: FMT,
    zero_pad: u1, // pad with leading zeros
};

/// Derive register type from WS (in bits)
pub const RegisterType = blk: {
    if (WS == 8) {
        break :blk u8;
    } else if (WS == 16) {
        break :blk u16;
    } else if (WS == 32) {
        break :blk u32;
    } else if (WS == 64) {
        break :blk u64;
    } else {
        @compileError("Unsupported WS size: " ++ std.fmt.comptimePrint("{}", .{WS}));
    }
};

pub const RegisterSignedType = blk: {
    if (WS == 8) {
        break :blk i8;
    } else if (WS == 16) {
        break :blk i16;
    } else if (WS == 32) {
        break :blk i32;
    } else if (WS == 64) {
        break :blk i64;
    } else {
        @compileError("Unsupported WS size: " ++ std.fmt.comptimePrint("{}", .{WS}));
    }
};

pub const SpecialRegisterType = blk: {
    if (WS <= 32) {
        break :blk u32;
    } else if (WS == 64) {
        break :blk u64;
    } else {
        @compileError("Unsupported WS size: " ++ std.fmt.comptimePrint("{}", .{WS}));
    }
};

pub const PointerRegisterType = blk: {
    if (WS <= 16) {
        break :blk u16;
    } else if (WS == 32) {
        break :blk u32;
    } else if (WS == 64) {
        break :blk u64;
    } else {
        @compileError("Unsupported WS size: " ++ std.fmt.comptimePrint("{}", .{WS}));
    }
};

pub const IO_MAP = enum(u8) {
    STDIO = 0x0,
    STDERR,
};
