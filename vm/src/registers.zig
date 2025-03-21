const std = @import("std");
const m = @import("main.zig");

pub const Registers = struct {

    // Register aliases
    pub const R0 = 0;
    pub const R1 = 1;
    pub const R2 = 2;
    pub const R3 = 3;
    pub const R4 = 4;
    pub const R5 = 5;
    pub const R6 = 6;
    pub const R7 = 7;
    pub const R8 = 8;
    pub const R9 = 9;
    pub const R10 = 10;
    pub const FLAGS = 11; // FLAGS register
    pub const JMP_STRIDE = 12; // Jump Stride
    pub const BP = 13; // Base Pointer
    pub const SP = 14; // Stack Pointer
    pub const IP = 15; // Instruction Pointer (Note: IP is R14, not R15)

    regs: [16]m.RegType = [_]m.RegType{0} ** 16, // 16 registers, each 32 bits

    // Initialize registers with default values
    pub fn init(self: *Registers) void {
        self.regs = [_]m.RegType{0} ** 16; // Reset all registers to 0
        self.regs[FLAGS] = 0x2100; // Set default FLAGS: RS = R0, NS = 0, SRC = R1, DST = R2
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
    pub fn getBits(self: *const Registers, index: u4, offset: u5, mask: u4) m.RegType {
        return (self.regs[index] >> offset) & mask;
    }

    // Set bits in register
    pub fn setBits(self: *Registers, index: u4, value: m.RegType, offset: u5, mask: u4) void {
        self.regs[index] = (self.regs[index] & ~(mask << offset)) | ((value & mask) << offset);
    }

    // Get a specific field from the FLAGS register
    pub fn getFlag(self: *const Registers, comptime field: FlagField) m.RegType {
        return getBits(self, FLAGS, @intCast(@intFromEnum(field)), field.mask);
    }

    // Set a specific field in the FLAGS register
    pub fn setFlag(self: *Registers, comptime field: FlagField, value: m.RegType) void {
        const mask = field.mask;
        const offset: u5 = @intCast(@intFromEnum(field));
        self.regs[11] = (self.regs[FLAGS] & ~(mask << offset)) | ((value & mask) << offset);
    }

    // Define the FLAGS register fields
    pub const FlagField = enum(u5) {
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
        pub fn mask(self: FlagField) m.RegType {
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
