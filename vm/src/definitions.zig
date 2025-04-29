// definitions.zig
const std = @import("std");

// ISA Constants
pub const WS = 32;  // Word Size in bits (interpreted as 32 bits per your instruction)
pub const HWS = 16; // Half Word Size in bits (16 bits = 2 bytes)
pub const REGISTER_COUNT = 256; // Number of registers
pub const MAX_INSTRUCTION_SIZE = 1024; // memory size

// Prefixes
pub const PREFIX_OP2 = 0xC; // 2-byte instruction
pub const PREFIX_OP4 = 0xD; // 4-byte instruction
pub const PREFIX_OP8 = 0xE; // 8-byte instruction

// ALU Data Type (ADT)
pub const ADT = enum(u8) {
    u8 = 0,
    i8,
    u16,
    i16,
    u32,
    i32,
    u64,
    i64,
    f16,
    f32,
    f64,
    fp4,
    fp8,
    u1,
    u4,
    i4,
};

// Branch Condition Selector (BCS)
pub const BCS = enum(u8) {
    always = 0x0,
    zero = 0x1,
    not_zero = 0x2,
    greater = 0x3,
    greater_or_equal = 0x4,
    less = 0x5,
    less_or_equal = 0x6,
    carry = 0x7,
    not_carry = 0x8,
    sign = 0x9,
    not_sign = 0xA,
    overflow = 0xB,
    not_overflow = 0xC,
    parity_even = 0xD,
    parity_odd = 0xE,
    interrupt = 0xF,
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
    reserved_0C = 0xC,
    reserved_0D = 0xD,
    reserved_0E = 0xE,
    reserved_0F = 0xF,
};

// ALU_IO_CFG (R12)
pub const ALU_IO_CFG = packed struct {
    rs: u8, // Register index for LI or first ALU operand
    src: u8, // Second ALU operand
    dst: u8, // Destination register
    ns: u8, // Nibble selector for LI
};

// ALU_MODE_CFG (R13)
pub const ALU_MODE_CFG = packed struct {
    adt: u8, // ALU Data Type (ADT)
    vl: u8, // Vector Length in ADT units
    st_dst: u16, // Stride of destination register
};

// ALU_VR_STRIDES (R14)
pub const ALU_VR_STRIDES = packed struct {
    st_rs: u16, // Stride of RS register
    st_src: u16, // Stride of SRC register
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
pub const R_BP = 253; // Base Pointer
pub const R_SP = 254; // Stack Pointer
pub const R_IP = 255; // Instruction Pointer

// Ensure packed structs fit within WS bits
comptime {
    std.debug.assert(@bitSizeOf(ALU_IO_CFG) <= WS);
    std.debug.assert(@bitSizeOf(ALU_MODE_CFG) <= WS);
    std.debug.assert(@bitSizeOf(ALU_VR_STRIDES) <= WS);
    std.debug.assert(@bitSizeOf(BRANCH_CTRL) <= WS);
}
