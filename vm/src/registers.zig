const std = @import("std");
const m = @import("main.zig");

pub const Registers = struct {

    // Enum for register names
    pub const Reg = enum(u4) {
        R0 = 0,
        R1 = 1,
        R2 = 2,
        R3 = 3,
        R4 = 4,
        R5 = 5,
        R6 = 6,
        R7 = 7,
        R8 = 8,
        R9 = 9,
        R10 = 10,
        FLAGS = 11, // FLAGS register
        JMP_STRIDE = 12, // Jump Stride
        BP = 13, // Base Pointer
        SP = 14, // Stack Pointer
        IP = 15, // Instruction Pointer

        // Optional: Helper to get index as u4
        pub fn index(self: Reg) u4 {
            return @intFromEnum(self);
        }
    };

    regs: [16]m.RegType = [_]m.RegType{0} ** 16, // 16 registers, each 32 bits

    // Initialize registers with default values
    pub fn init(self: *Registers) void {
        self.regs = [_]m.RegType{0} ** 16; // Reset all registers to 0
        self.regs[@intFromEnum(Reg.FLAGS)] = 0x2100; // Set default FLAGS: RS = R0, NS = 0, SRC = R1, DST = R2
    }

    // Get a register value
    pub fn get(self: *const Registers, index: u4) m.RegType {
        return self.regs[index];
    } // Get a register value

    // Set a register value
    pub fn set(self: *Registers, index: u4, value: m.RegType) void {
        self.regs[index] = value;
    }

    // Set bits in register
    pub fn getBits(self: *const Registers, index: u4, offset: u5, mask: u8) u8 {
        const v: u8 = @truncate(self.regs[index] >> offset);
        return v & mask;
    }

    // Set bits in register
    pub fn setBits(self: *Registers, index: u4, value: m.RegType, offset: u5, mask: u4) void {
        self.regs[index] = (self.regs[index] & ~(mask << offset)) | ((value & mask) << offset);
    }

    // Get a specific field from the FLAGS register
    pub fn getFlag(self: *const Registers, comptime field: FlagField) u8 {
        return getBits(self, @intFromEnum(Reg.FLAGS), @intCast(@intFromEnum(field)), field.mask());
    }

    // Set a specific field in the FLAGS register
    pub fn setFlag(self: *Registers, comptime field: FlagField, value: m.RegType) void {
        const mask: m.RegType = field.mask();
        const offset: u8 = @intCast(@intFromEnum(field));
        const msk: m.RegType = ~(mask << offset);
        self.regs[@intFromEnum(Reg.FLAGS)] = (self.regs[@intFromEnum(Reg.FLAGS)] & msk) | ((value & mask) << offset);
    }

    // Define the FLAGS register fields
    pub const FlagField = enum(u8) {
        NS = 0, // Nibble Selector (bits 0-3)
        RS = 4, // Register Selector (bits 4-7)
        SRC = 8, // Source Register (bits 8-11)
        DST = 12, // Destination Register (bits 12-15)
        BCS = 16, // Branch Condition Selector (bits 16-19)
        ADT = 20, // ALU Data Type Selector (bits 20-23)
        C = 24, // Carry Flag (bit 24)
        Z = 25, // Zero Flag (bit 25)
        S = 26, // Sign Flag (bit 26)
        V = 27, // Overflow Flag (bit 27)
        P = 28, // Parity Flag (bit 28)
        I = 29, // Interrupt Flag (bit 29)

        // Helper to get the mask and offset for each field
        pub fn mask(self: FlagField) u8 {
            return switch (self) {
                .NS, .RS, .SRC, .DST, .BCS, .ADT => 0xF, // 4-bit fields
                .C, .Z, .S, .V, .P, .I => 0x1, // 1-bit flags
            };
        }

        pub fn offset(self: FlagField) u5 {
            return @intFromEnum(self);
        }
    };
};

pub const BranchCondition = enum(u4) {
    A = 0x0,  // Always
    Z = 0x1,  // Zero
    NZ = 0x2, // NotZero
    G = 0x3,  // Greater
    GE = 0x4, // GreaterOrEqual
    L = 0x5,  // Less
    LE = 0x6, // LessOrEqual
    C = 0x7,  // Carry
    NC = 0x8, // NotCarry
    S = 0x9,  // Sign
    NS = 0xA, // NotSign
    O = 0xB,  // Overflow
    NO = 0xC, // NotOverflow
    PE = 0xD, // ParityEven
    PO = 0xE, // ParityOdd
    I = 0xF,  // Interrupt

    // Optional: Helper method to get raw value
    pub fn value(self: BranchCondition) u4 {
        return @intFromEnum(self);
    }
};

// Long-form aliases as constants
pub const Always = BranchCondition.A;
pub const Zero = BranchCondition.Z;
pub const NotZero = BranchCondition.NZ;
pub const Greater = BranchCondition.G;
pub const GreaterOrEqual = BranchCondition.GE;
pub const Less = BranchCondition.L;
pub const LessOrEqual = BranchCondition.LE;
pub const Carry = BranchCondition.C;
pub const NotCarry = BranchCondition.NC;
pub const Sign = BranchCondition.S;
pub const NotSign = BranchCondition.NS;
pub const Overflow = BranchCondition.O;
pub const NotOverflow = BranchCondition.NO;
pub const ParityEven = BranchCondition.PE;
pub const ParityOdd = BranchCondition.PO;
pub const Interrupt = BranchCondition.I;
