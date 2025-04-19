const std = @import("std");
const assert = std.debug.assert;
const fs = std.fs;
const builtin = @import("builtin");
const alu = @import("alu.zig");

const WS = 32;
const HWS = WS / 2;

// Derive the register type based on WS
pub const RegType = switch (WS) {
    8 => u8,
    16 => u16,
    32 => u32,
    64 => u64,
    else => @compileError("Unsupported word size: WS must be 8, 16, 32, or 64"),
};

pub const BranchCondition = enum(u4) {
    A = 0x0, // Always
    Z = 0x1, // Zero
    NZ = 0x2, // NotZero
    G = 0x3, // Greater
    GE = 0x4, // GreaterOrEqual
    L = 0x5, // Less
    LE = 0x6, // LessOrEqual
    C = 0x7, // Carry
    NC = 0x8, // NotCarry
    S = 0x9, // Sign
    NS = 0xA, // NotSign
    O = 0xB, // Overflow
    NO = 0xC, // NotOverflow
    PE = 0xD, // ParityEven
    PO = 0xE, // ParityOdd
    I = 0xF, // Interrupt

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

// Top-level opcodes
pub const Opcode = enum(u4) {
    NOP = 0x0, // No operation (and sub-opcodes below)
    RS = 0x1, // Set N.RS
    NS = 0x2, // Set N.NS
    LI = 0x3, // Load Immediate
    LIS = 0x4, // Load Immediate Signed
    ALU = 0x5, // ALU operation
    JMP = 0x6, // Jump
    CALL = 0x7, // Call
    PUSH = 0x8, // Push
    POP = 0x9, // Pop
    INT = 0xA, // Interrupt
    IN = 0xB, // Input
    OUT = 0xC, // Output
    // Note: 0xD to 0xF are reserved for future use
};

// Sub-opcodes for NOP (opcode 0x0)
pub const SubOpcode = enum(u4) {
    NOP = 0x0, // No operation
    RET = 0x1, // Return
    IRET = 0x2, // Interrupt Return
    SETC = 0x3, // Set Carry
    CLSC = 0x4, // Clear Carry
    INC = 0x5, // Increment
    DEC = 0x6, // Decrement
    NOT = 0x7, // Bitwise NOT
    CMP = 0x8, // Compare
};

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

// Aliases for Reg variants
pub const r0 = Reg.R0.index();
pub const r1 = Reg.R1.index();
pub const r2 = Reg.R2.index();
pub const r3 = Reg.R3.index();
pub const r4 = Reg.R4.index();
pub const r5 = Reg.R5.index();
pub const r6 = Reg.R6.index();
pub const r7 = Reg.R7.index();
pub const r8 = Reg.R8.index();
pub const r9 = Reg.R9.index();
pub const r10 = Reg.R10.index();
pub const flags = Reg.FLAGS.index();
pub const jmp_stride = Reg.JMP_STRIDE.index();
pub const bp = Reg.BP.index();
pub const sp = Reg.SP.index();
pub const ip = Reg.IP.index();

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

// FMT bitmasks
const FMT_FORMAT_MASK: u32 = 0x7; // Bits 0-2
const FMT_LEADING_MASK: u32 = 0x8; // Bit 3
const FMT_LENGTH_MASK: u32 = 0x30; // Bits 4-5
const FMT_PRECISION_MASK: u32 = 0x1C0; // Bits 6-8

// Enums for FMT fields
const FmtType = enum(u3) {
    Raw = 0,
    Hex = 1,
    Dec = 2,
    Binary = 3,
    Float = 4,
    SignedDec = 5,
    // 6-7 reserved
};

const FmtLength = enum(u2) {
    Nibble = 0,
    Byte = 1,
    HalfWord = 2,
    Word = 3,
};

pub const Instruction = packed struct {
    opcode: u4,
    operand: u4,
};

pub const CPU = struct {
    R: [16]RegType, // Registers
    M: []u8, // Memory for the CPU

    // Initialize the CPU with default register values
    pub fn init(allocator: std.mem.Allocator, memory_size: usize) !CPU {
        var cpu = CPU{ .R = [_]RegType{0} ** 16, .M = try allocator.alloc(u8, memory_size) };
        cpu.R[flags] = 0x2100;
        cpu.R[sp] = @truncate(memory_size - WS / 8); // Set Stack Pointer to the last word in memory
        return cpu;
    }

    // Set bits in register
    pub fn getBits(self: *const CPU, index: u4, offset: u5, mask: u8) u8 {
        const v: u8 = @truncate(self.R[index] >> offset);
        return v & mask;
    }

    // Set bits in register
    pub fn setBits(self: *CPU, index: u4, value: RegType, offset: u5, mask: u4) void {
        self.R[index] = (self.R[index] & ~(mask << offset)) | ((value & mask) << offset);
    }

    // Get a specific field from the FLAGS register
    pub fn getFlag(self: *const CPU, comptime field: FlagField) u8 {
        return getBits(self, flags, @intCast(@intFromEnum(field)), field.mask());
    }

    // Set a specific field in the FLAGS register
    pub fn setFlag(self: *CPU, comptime field: FlagField, value: RegType) void {
        const mask: RegType = field.mask();
        const offset: u8 = @intFromEnum(field);
        const msk: RegType = ~(mask << offset);
        self.R[flags] = (self.R[flags] & msk) | ((value & mask) << offset);
    }

    pub fn deinit(self: *CPU, allocator: std.mem.Allocator) void {
        allocator.free(self.M);
    }

    fn loadProgram(self: *CPU, program_path: []const u8, load_address: RegType) !void {
        const file = try fs.cwd().openFile(program_path, .{});
        defer file.close();

        const ext = fs.path.extension(program_path);
        if (std.mem.eql(u8, ext, ".hex")) {
            var buf_reader = std.io.bufferedReader(file.reader());
            var reader = buf_reader.reader();
            var buf: [2]u8 = undefined; // Buffer for two hex characters (one byte)
            var address = load_address;
            var hex_chars_read: usize = 0;

            while (true) {
                const byte = reader.readByte() catch |err| switch (err) {
                    error.EndOfStream => break, // Exit loop on EOF
                    else => return err, // Propagate other errors
                };

                if (std.ascii.isHex(byte)) {
                    buf[hex_chars_read] = byte;
                    hex_chars_read += 1;

                    if (hex_chars_read == 2) {
                        const parsed_byte = try std.fmt.parseInt(u8, &buf, 16);
                        if (address >= self.M.len) {
                            return error.ProgramTooLarge;
                        }
                        self.M[address] = parsed_byte;
                        address += 1;
                        hex_chars_read = 0;
                    }
                } else if (byte == '\r' or byte == '\n') {
                    // Ignore CR/LF characters
                } else if (!std.ascii.isWhitespace(byte)) {
                    std.log.warn("Invalid character in hex file: {}", .{byte});
                    return error.InvalidHexFile; // Or handle differently
                }
            }

            if (hex_chars_read != 0) {
                std.log.warn("Incomplete byte in hex file. Ignoring last nibble(s).", .{}); // Or handle as an error
            }
        } else {
            const stat = try file.stat();
            const file_size = stat.size;

            if (load_address + file_size > self.M.len) {
                return error.ProgramTooLarge;
            }

            const bytes_read = try file.readAll(self.M[load_address..]);
            if (bytes_read != file_size) {
                std.log.warn("Failed to read the entire binary file.  Expected {} bytes, read {}.", .{ file_size, bytes_read });
                return error.IncompleteRead; // Or handle differently, e.g., continue with partial load
            }
        }
    }

    // Helper function to execute a unary operation on a register
    pub fn executeUnaryOp(self: *CPU, comptime op: fn (RegType) RegType, comptime set_carry: bool) !void {
        const reg_index: u4 = @truncate(self.getFlag(.RS)); // Get the register from RS flag
        const value = self.R.get(reg_index); // Get full register value (now RegType)

        // Apply the operation
        const result = op(value);

        // Store the result
        self.R.set(reg_index, result);

        // Update flags
        self.R.setFlag(.Z, @intFromBool(result == 0)); // Zero flag
        const msb_mask = @as(RegType, 1) << (WS - 1); // Calculate MSB mask
        self.R.setFlag(.S, @intFromBool((result & msb_mask) != 0)); // Sign flag using WS

        // Update carry flag if requested
        if (set_carry) {
            const old_msb = (value & msb_mask) != 0;
            const new_msb = (result & msb_mask) != 0;
            // Set C if MSB flips when it shouldn't
            self.R.setFlag(.C, @intFromBool(old_msb != new_msb and switch (op) {
                incOp => old_msb == false, // Carry if going from positive to negative
                decOp => old_msb == true, // Borrow if going from negative to positive
                else => false, // No carry for other ops like NOT
            }));
        }
    }

    // Define the specific operations as inline functions
    fn incOp(value: RegType) RegType {
        return value + 1;
    }

    fn decOp(value: RegType) RegType {
        return value - 1;
    }

    fn notOp(value: RegType) RegType {
        return ~value;
    }

    // Optimized instruction implementations
    pub fn executeINC(self: *CPU) !void {
        try self.executeUnaryOp(incOp, true); // Set carry flag for INC
    }

    pub fn executeDEC(self: *CPU) !void {
        try self.executeUnaryOp(decOp, true); // Set carry flag for DEC
    }

    pub fn executeNOT(self: *CPU) !void {
        try self.executeUnaryOp(notOp, false); // No carry flag for NOT
    }

    pub fn executeCMP(self: *CPU) !void {
        // Get the register indices from RS and SRC flags
        const rs_index: u4 = @truncate(self.getFlag(.RS)); // First operand register
        const src_index: u4 = @truncate(self.getFlag(.SRC)); // Second operand register

        // Get the values from the registers
        const rs_value = self.R.get(rs_index); // Value of RS register
        const src_value = self.R.get(src_index); // Value of SRC register

        // Perform subtraction to compare (but don't store the result)
        const result = rs_value -% src_value; // Wrapping subtraction for unsigned comparison

        // Calculate MSB mask based on word size
        const msb_mask = @as(RegType, 1) << (WS - 1);

        // Update flags based on the comparison
        self.R.setFlag(.Z, @intFromBool(result == 0)); // Zero flag: set if result is 0
        self.R.setFlag(.S, @intFromBool((result & msb_mask) != 0)); // Sign flag: set if result is negative

        // Carry flag: set if unsigned borrow occurred (rs_value < src_value)
        self.R.setFlag(.C, @intFromBool(rs_value < src_value)); // Unsigned comparison

        // Overflow flag: set if signed overflow occurred
        const rs_msb = (rs_value & msb_mask) != 0;
        const src_msb = (src_value & msb_mask) != 0;
        const result_msb = (result & msb_mask) != 0;
        self.R.setFlag(.V, @intFromBool((rs_msb != src_msb) and (result_msb != rs_msb)));
    }

    // Assuming SP (stack pointer) is R14, IP (instruction pointer) is R15, and FLAGS is R13
    pub fn executeRET(self: *CPU) !void {
        // Get the current stack pointer
        var _sp = self.R[sp];

        // Check if stack underflow would occur
        if (_sp >= self.M.len) {
            return error.StackUnderflow;
        }

        // Pop the return address from the stack
        const return_address = self.readMemory(_sp);
        _sp += @sizeOf(RegType); // Increment SP by size of RegType

        // Update SP and IP
        self.R[sp] = _sp;
        self.R[ip] = return_address;
    }

    // Helper function to read RegType from memory
    fn readMemory(self: *CPU, address: RegType) RegType {
        const ptr = @as([*]const u8, @ptrCast(&self.M[address]));
        return @as(*align(1) const RegType, @ptrCast(ptr)).*;
    }

    // Helper function to write RegType to memory (for completeness)
    fn writeMemory(self: *CPU, address: RegType, value: RegType) void {
        const ptr = @as([*]u8, @ptrCast(&self.M[address]));
        @as(*align(1) RegType, @ptrCast(ptr)).* = value;
    }

    pub fn executeIRET(self: *CPU) !void {
        // Get the current stack pointer
        var _sp = self.R[sp];

        // Check if stack underflow would occur (need space for address + flags)
        if (_sp + @sizeOf(RegType) >= self.M.len) {
            return error.StackUnderflow;
        }

        // Pop the return address and flags from the stack
        const return_address = self.readMemory(_sp);
        _sp += @sizeOf(RegType);
        const _flags = self.readMemory(_sp);
        _sp += @sizeOf(RegType);

        // Update SP, IP, and FLAGS register
        self.R[sp] = _sp;
        self.R[ip] = return_address;
        self.R[flags] = _flags;
    }

    // Format function: Converts rvalue to a string based on rfmt
    fn format(rvalue: u32, rfmt: u32) ![]const u8 {
        // Mask rfmt to 9 bits
        const fmt = rfmt & 0x1FF;

        // Parse FMT fields
        const fmt_type: FmtType = @enumFromInt(fmt & FMT_FORMAT_MASK);
        const leading_zeros = (fmt & FMT_LEADING_MASK) != 0; // true = fixed
        const length: FmtLength = @enumFromInt((fmt & FMT_LENGTH_MASK) >> 4);
        const precision: u4 = @truncate((fmt & FMT_PRECISION_MASK) >> 6);

        // Extract value based on length
        const masked_value = switch (length) {
            .Nibble => rvalue & 0xF,
            .Byte => rvalue & 0xFF,
            .HalfWord => rvalue & 0xFFFF,
            .Word => rvalue,
        };

        // Static buffer for formatting (adjust size for max case, e.g., 32-bit Binary + precision)
        var buf: [64]u8 = undefined;
        const allocator = std.heap.page_allocator; // For dynamic allocation if needed

        return switch (fmt_type) {
            .Raw => blk: {
                buf[0] = @truncate(masked_value);
                break :blk buf[0..1];
            },
            .Hex => blk: {
                const digits: u4 = switch (length) { // Moved outside the fmt_type switch
                    .Nibble => 1,
                    .Byte => 2,
                    .HalfWord => 4,
                    .Word => 8,
                };

                const len = try std.fmt.bufPrint(&buf, "{x}", .{masked_value});

                if (leading_zeros and len.len < digits) {
                    const padded = try allocator.alloc(u8, digits); // Or allocator.dupe if appropriate
                    defer allocator.free(padded);
                    @memset(padded[0 .. digits - len.len], '0');
                    @memcpy(padded[digits - len.len ..], len);
                    break :blk try allocator.dupe(u8, padded); // dupe the slice data if needed
                }

                break :blk try allocator.dupe(u8, len); // Correct usage outside the if
            },

            .Dec => blk: {
                const len = try std.fmt.bufPrint(&buf, "{d}", .{masked_value});
                if (precision > 0) {
                    var decimal_buf: [32]u8 = undefined;
                    const decimal_len = try std.fmt.bufPrint(&decimal_buf, "{d}.{d:0>2}", .{ masked_value, 0 });
                    break :blk try allocator.dupe(u8, decimal_len[0 .. len.len + 1 + precision]);
                }
                break :blk try allocator.dupe(u8, len);
            },
            .SignedDec => blk: {
                const signed_value = switch (length) {
                    .Nibble => @as(u4, @truncate(masked_value)),
                    .Byte => @as(u8, @truncate(masked_value)),
                    .HalfWord => @as(u16, @truncate(masked_value)),
                    .Word => masked_value,
                };
                const len = try std.fmt.bufPrint(&buf, "{d}", .{signed_value});
                if (precision > 0) {
                    var decimal_buf: [32]u8 = undefined;
                    const abs_value = if (signed_value < 0) -signed_value else signed_value;
                    const decimal_len = try std.fmt.bufPrint(&decimal_buf, "{d}.{d:0>2}", .{ abs_value, 0 });
                    const result = if (signed_value < 0)
                        try std.fmt.allocPrint(allocator, "-{s}", .{decimal_len[0 .. len.len + 1 + precision]})
                    else
                        try allocator.dupe(u8, decimal_len[0 .. len.len + 1 + precision]);
                    break :blk result;
                }
                break :blk try allocator.dupe(u8, len);
            },
            .Binary => blk: {
                const bits: u8 = switch (length) {
                    .Nibble => 4,
                    .Byte => 8,
                    .HalfWord => 16,
                    .Word => 32,
                };
                const len = try std.fmt.bufPrint(&buf, "{b}", .{masked_value});
                if (leading_zeros and len.len < bits) {
                    const padded = try allocator.alloc(u8, bits); // Or allocator.dupe if appropriate
                    defer allocator.free(padded);
                    @memset(padded[0 .. bits - len.len], '0');
                    @memcpy(padded[bits - len.len ..], len);
                    break :blk try allocator.dupe(u8, padded[0..bits]);
                }
                break :blk try allocator.dupe(u8, len);
            },
            .Float => blk: {
                if (length != .Word) return error.InvalidFloatLength;
                const float_value: f32 = @bitCast(rvalue); // u32 to f32
                if (precision > 7) return error.InvalidPrecision;

                // Round up to max precision (7) and format
                const max_scale: f32 = std.math.pow(f32, 10.0, 7); // 10^7
                const rounded_value: f32 = @ceil(float_value * max_scale) / max_scale;
                const max_len = try std.fmt.bufPrint(&buf, "{d:.7}", .{rounded_value});

                // Trim to desired precision
                const trimmed_len = blk2: {
                    if (precision == 0) {
                        // Truncate at decimal point
                        for (max_len, 0..) |c, i| {
                            if (c == '.') break :blk2 i;
                        } else unreachable; // Always has a decimal with floats
                    } else {
                        // Include precision digits after decimal
                        for (max_len, 0..) |c, i| {
                            if (c == '.') {
                                const end = i + 1 + precision; // '.' + precision digits
                                break :blk2 if (end >= max_len.len) max_len.len else end;
                            }
                        } else unreachable;
                    }
                };

                // Handle leading zeros
                if (leading_zeros) {
                    const digits_before = for (max_len[0..trimmed_len], 0..) |c, i| {
                        if (c == '.') break i;
                    } else trimmed_len;
                    const target = 10; // Target width for whole part
                    if (digits_before < target) {
                        var padded: [32]u8 = undefined;
                        @memset(padded[0 .. target - digits_before], '0');
                        @memcpy(padded[target - digits_before ..][0..trimmed_len], max_len[0..trimmed_len]);
                        break :blk try allocator.dupe(u8, padded[0 .. target + trimmed_len]);
                    }
                }
                break :blk try allocator.dupe(u8, max_len[0..trimmed_len]);
            },
        };
    }

    // OUT execution
    pub fn executeOUT(self: *CPU, operand: u4) !void {
        const io_channel = operand;
        const reg_index_rs: u4 = @truncate(self.getFlag(.RS)); // Rs (value)
        const reg_index_src: u4 = @truncate(self.getFlag(.SRC)); // Rt (format)
        const rvalue: u4 = @truncate(self.R.get(reg_index_rs));
        const rfmt: u4 = @truncate(self.R.get(reg_index_src));

        // Format the value
        const val = try format(rvalue, rfmt);

        // Write to appropriate I/O channel
        switch (io_channel) {
            0x0 => try writeToStdOut(val),
            0x1 => try writeToStdErr(val),
            else => return error.InvalidIOChannel,
        }

        // Free allocated memory (since format uses allocator)
        std.heap.page_allocator.free(val);
    }

    // Parse function: Converts input string to u32 based on fmt
    fn parse(input: []const u8, fmt: u32) !u32 {
        const lu3: u3 = @truncate(fmt & FMT_FORMAT_MASK);
        const fmt_type: FmtType = @enumFromInt(lu3);
        //const leading_zeros = (fmt & FMT_LEADING_MASK) != 0;
        const lu2: u2 = @truncate((fmt & FMT_LENGTH_MASK) >> 4);
        const length: FmtLength = @enumFromInt(lu2);
        //const precision: u3 = @truncate((fmt & FMT_PRECISION_MASK) >> 6);

        // Trim whitespace and validate input length
        const trimmed = std.mem.trim(u8, input, " \t\r\n");
        if (trimmed.len == 0) return error.EmptyInput;

        return switch (fmt_type) {
            .Raw => blk: {
                if (trimmed.len != 1) return error.InvalidRawInput;
                break :blk @as(u32, trimmed[0]);
            },
            .Hex => blk: {
                const value = try std.fmt.parseInt(u32, trimmed, 16);
                break :blk switch (length) {
                    .Nibble => @as(u4, @truncate(value)),
                    .Byte => @as(u8, @truncate(value)),
                    .HalfWord => @as(u16, @truncate(value)),
                    .Word => value,
                };
            },
            .Dec => blk: {
                const value = try std.fmt.parseInt(u32, trimmed, 10);
                break :blk switch (length) {
                    .Nibble => @as(u4, @truncate(value)),
                    .Byte => @as(u8, @truncate(value)),
                    .HalfWord => @as(u16, @truncate(value)),
                    .Word => value,
                };
            },
            .SignedDec => blk: {
                const signed_value = try std.fmt.parseInt(i32, trimmed, 10);
                const value: u32 = @intCast(signed_value);
                break :blk switch (length) {
                    .Nibble => @as(u4, @truncate(value)),
                    .Byte => @as(u8, @truncate(value)),
                    .HalfWord => @as(u16, @truncate(value)),
                    .Word => value,
                };
            },
            .Binary => blk: {
                const value = try std.fmt.parseInt(u32, trimmed, 2);
                break :blk switch (length) {
                    .Nibble => @as(u4, @truncate(value)),
                    .Byte => @as(u8, @truncate(value)),
                    .HalfWord => @as(u16, @truncate(value)),
                    .Word => value,
                };
            },
            .Float => blk: {
                if (length != .Word) return error.InvalidFloatLength;
                const float_value = try std.fmt.parseFloat(f32, trimmed);
                break :blk @bitCast(float_value);
            },
        };
    }

    fn writeToStdOut(value: []const u8) !void {
        try std.io.getStdOut().writeAll(value);
    }

    fn writeToStdErr(value: []const u8) !void {
        try std.io.getStdErr().writeAll(value);
    }

    fn readFromStdIn(buf: []u8) ![]u8 {
        const stdin = std.io.getStdIn();
        return try stdin.reader().readUntilDelimiterOrEof(buf, '\n') orelse return error.EndOfInput;
    }

    // IN execution
    pub fn executeIN(self: *CPU, operand: u4) !void {
        const io_channel = operand;
        const reg_index_rd: u4 = @truncate(self.getFlag(.RS)); // Rd (destination)
        const reg_index_rs: u4 = @truncate(self.getFlag(.SRC)); // Rs (format)
        const rfmt: u4 = @truncate(self.R.get(reg_index_rs));

        // Buffer for input (adjust size as needed)
        var buf: [64]u8 = undefined;
        const input = switch (io_channel) {
            0x0 => try readFromStdIn(&buf),
            else => return error.InvalidIOChannel,
        };

        // Parse input and store in Rd
        const value = try parse(input, rfmt);
        self.R.set(reg_index_rd, value);
    }

    pub fn executeRS(self: *CPU, operand: u4) !void {
        self.R.setFlag(.RS, operand);
    }
    pub fn executeNS(self: *CPU, operand: u4) !void {
        self.R.setFlag(.NS, operand);
    }

    pub fn _executeALU(self: *CPU, op: u4) !void {
        const reg_index_arg1: u4 = @truncate(self.getFlag(.RS)); // First operand (arg1)
        const reg_index_arg2: u4 = @truncate(self.getFlag(.SRC)); // Second operand (arg2)
        const reg_index_dst: u4 = @truncate(self.getFlag(.DST)); // Destination register
        //const reg_adt: u4 = @truncate(self.getFlag(.ADT));
        const arg1 = self.R[reg_index_arg1]; // Value of arg1 (u32)
        const arg2 = self.R[reg_index_arg2]; // Value of arg2 (u32)

        // Convert raw op to ALUOperation enum
        const alu_op: ALUOperation = @enumFromInt(op);

        // Perform operation based on ALUOperation
        const res: u32 = switch (alu_op) {
            ._add => arg1 +% arg2, // Addition with wrapping
            ._sub => arg1 -% arg2, // Subtraction with wrapping
            ._and => arg1 & arg2, // Bitwise AND
            ._or => arg1 | arg2, // Bitwise OR
            ._xor => arg1 ^ arg2, // Bitwise Exclusive OR
            ._shl => arg1 << @as(u5, @truncate(arg2)), // Shift Left (truncate to 5 bits)
            ._shr => arg1 >> @as(u5, @truncate(arg2)), // Shift Right Logical (zero-fill)
            ._sar => blk: { // Shift Arithmetic Right (sign-extend)
                const signed_arg1: i32 = @bitCast(arg1); // u32 to i32
                const shift: u5 = @truncate(arg2);
                break :blk @bitCast(signed_arg1 >> shift);
            },
            ._mul => arg1 *% arg2, // Multiplication with wrapping
            ._div => blk: { // Division (unsigned)
                if (arg2 == 0) return error.DivisionByZero;
                break :blk arg1 / arg2;
            },

            ._lookup => blk: { // Placeholder: Lookup operation
                // TODO: Implement lookup (e.g., table lookup or memory access)
                break :blk arg1; // Stub: return arg1 for now
            },
            ._load => blk: { // Placeholder: Load operation
                // TODO: Implement load (e.g., from memory at arg2)
                break :blk arg2; // Stub: return arg2 for now
            },
            ._store => blk: { // Placeholder: Store operation
                // TODO: Implement store (e.g., arg1 to memory at arg2)
                break :blk 0; // Stub: return 0 for now
            },
            //else =>  return error.InvalidALUOperation,
        };

        // Store result in destination register
        self.R[reg_index_dst] = res;
    }

    pub fn executeALU(self: *CPU, op: u4) !void {
        const rs: u4 = @truncate(self.getFlag(.RS));
        const src: u4 = @truncate(self.getFlag(.SRC));
        const dst: u4 = @truncate(self.getFlag(.DST));
        const adt_raw: u4 = @truncate(self.getFlag(.ADT));
        const adt: ALUDataType = @enumFromInt(adt_raw);
        const alu_op: ALUOperation = @enumFromInt(op);

        const is_64bit = adt == .u64 or adt == .i64;
        if (is_64bit and (rs >= 15 or src >= 15 or dst >= 15)) {
            return error.InvalidRegisterPair;
        }

        const arg1_u64: u64 = if (is_64bit) (@as(u64, self.R[rs]) << 32) | self.R[rs + 1] else @as(u64, self.R[rs]);
        const arg2_u64: u64 = if (is_64bit) (@as(u64, self.R[src]) << 32) | self.R[src + 1] else @as(u64, self.R[src]);
        const is_signed = adt == .i8 or adt == .i16 or adt == .i32 or adt == .i64;
        const size_bits: u8 = switch (adt) {
            .u8, .i8 => 8,
            .u16, .i16 => 16,
            .u32, .i32 => 32,
            .u64, .i64 => 64,
            else => return error.UnsupportedDataType,
        };

        var result: u64 = undefined;
        var carry: bool = undefined;
        var zero: bool = undefined;
        var sign: bool = undefined;
        var overflow: bool = undefined;
        var parity: bool = undefined;

        // Compile-time check for x86-64
        if (builtin.target.cpu.arch == .x86_64) {
            var _flags: u64 = undefined;
            result = switch (alu_op) {
                ._add => switch (size_bits) {
                    8 => blk: {
                        asm volatile (
                            \\ mov al, %[arg1]
                            \\ add al, %[arg2]
                            \\ pushfq
                            \\ pop %[_flags]
                            \\ movzx %[res], al
                            : [res] "=r" (result),
                              [flags] "=r" (_flags),
                            : [arg1] "r" (@as(u8, @truncate(arg1_u64))),
                              [arg2] "r" (@as(u8, @truncate(arg2_u64))),
                            : "rax", "cc"
                        );
                        break :blk result;
                    },
                    16 => blk: {
                        asm volatile (
                            \\ mov ax, %[arg1]
                            \\ add ax, %[arg2]
                            \\ pushfq
                            \\ pop %[_flags]
                            \\ movzx %[res], ax
                            : [res] "=r" (result),
                              [flags] "=r" (_flags),
                            : [arg1] "r" (@as(u16, @truncate(arg1_u64))),
                              [arg2] "r" (@as(u16, @truncate(arg2_u64))),
                            : "rax", "cc"
                        );
                        break :blk result;
                    },
                    32 => blk: {
                        asm volatile (
                            \\ mov eax, %[arg1]
                            \\ add eax, %[arg2]
                            \\ pushfq
                            \\ pop %[_flags]
                            \\ mov %[res], eax
                            : [res] "=r" (result),
                              [flags] "=r" (_flags),
                            : [arg1] "r" (@as(u32, @truncate(arg1_u64))),
                              [arg2] "r" (@as(u32, @truncate(arg2_u64))),
                            : "rax", "cc"
                        );
                        break :blk result;
                    },
                    64 => blk: {
                        asm volatile (
                            \\ mov rax, %[arg1]
                            \\ add rax, %[arg2]
                            \\ mov %[res], eax
                            \\ pushfq
                            \\ pop %[_flags]
                            : [res] "=r" (result),
                              [flags] "=r" (_flags),
                            : [arg1] "r" (arg1_u64),
                              [arg2] "r" (arg2_u64),
                            : "rax", "cc"
                        );
                        break :blk result;
                    },
                },
                ._sub => switch (size_bits) {
                    8 => blk: {
                        asm volatile (
                            \\ mov al, %[arg1]
                            \\ sub al, %[arg2]
                            \\ pushfq
                            \\ pop %[_flags]
                            \\ movzx %[res], al
                            : [res] "=r" (result),
                              [flags] "=r" (_flags),
                            : [arg1] "r" (@as(u8, @truncate(arg1_u64))),
                              [arg2] "r" (@as(u8, @truncate(arg2_u64))),
                            : "rax", "cc"
                        );
                        break :blk result;
                    },
                    16 => blk: {
                        asm volatile (
                            \\ mov ax, %[arg1]
                            \\ sub ax, %[arg2]
                            \\ pushfq
                            \\ pop %[_flags]
                            \\ movzx %[res], ax
                            : [res] "=r" (result),
                              [flags] "=r" (_flags),
                            : [arg1] "r" (@as(u16, @truncate(arg1_u64))),
                              [arg2] "r" (@as(u16, @truncate(arg2_u64))),
                            : "rax", "cc"
                        );
                        break :blk result;
                    },
                    32 => blk: {
                        asm volatile (
                            \\ mov eax, %[arg1]
                            \\ sub eax, %[arg2]
                            \\ pushfq
                            \\ pop %[_flags]
                            \\ mov %[res], eax
                            : [res] "=r" (result),
                              [flags] "=r" (_flags),
                            : [arg1] "r" (@as(u32, @truncate(arg1_u64))),
                              [arg2] "r" (@as(u32, @truncate(arg2_u64))),
                            : "rax", "cc"
                        );
                        break :blk result;
                    },
                    64 => blk: {
                        asm volatile (
                            \\ mov rax, %[arg1]
                            \\ sub rax, %[arg2]
                            \\ mov %[res], eax
                            \\ pushfq
                            \\ pop %[_flags]
                            : [res] "=r" (result),
                              [flags] "=r" (_flags),
                            : [arg1] "r" (arg1_u64),
                              [arg2] "r" (arg2_u64),
                            : "rax", "cc"
                        );
                        break :blk result;
                    },
                    else => unreachable,
                },
                ._and => switch (size_bits) {
                    8 => blk: {
                        asm volatile (
                            \\ mov al, %[arg1]
                            \\ and al, %[arg2]
                            \\ pushfq
                            \\ pop %[_flags]
                            \\ movzx %[res], al
                            : [res] "=r" (result),
                              [flags] "=r" (_flags),
                            : [arg1] "r" (@as(u8, @truncate(arg1_u64))),
                              [arg2] "r" (@as(u8, @truncate(arg2_u64))),
                            : "rax", "cc"
                        );
                        break :blk result;
                    },
                    16 => blk: {
                        asm volatile (
                            \\ mov ax, %[arg1]
                            \\ and ax, %[arg2]
                            \\ pushfq
                            \\ pop %[_flags]
                            \\ movzx %[res], ax
                            : [res] "=r" (result),
                              [flags] "=r" (_flags),
                            : [arg1] "r" (@as(u16, @truncate(arg1_u64))),
                              [arg2] "r" (@as(u16, @truncate(arg2_u64))),
                            : "rax", "cc"
                        );
                        break :blk result;
                    },
                    32 => blk: {
                        asm volatile (
                            \\ mov eax, %[arg1]
                            \\ and eax, %[arg2]
                            \\ pushfq
                            \\ pop %[_flags]
                            \\ mov %[res], eax
                            : [res] "=r" (result),
                              [flags] "=r" (_flags),
                            : [arg1] "r" (@as(u32, @truncate(arg1_u64))),
                              [arg2] "r" (@as(u32, @truncate(arg2_u64))),
                            : "rax", "cc"
                        );
                        break :blk result;
                    },
                    64 => blk: {
                        asm volatile (
                            \\ mov rax, %[arg1]
                            \\ and rax, %[arg2]
                            \\ mov %[res], eax
                            \\ pushfq
                            \\ pop %[_flags]
                            : [res] "=r" (result),
                              [flags] "=r" (_flags),
                            : [arg1] "r" (arg1_u64),
                              [arg2] "r" (arg2_u64),
                            : "rax", "cc"
                        );
                        break :blk result;
                    },
                },
                ._or => switch (size_bits) {
                    8 => blk: {
                        asm volatile (
                            \\ mov al, %[arg1]
                            \\ or al, %[arg2]
                            \\ pushfq
                            \\ pop %[_flags]
                            \\ movzx %[res], al
                            : [res] "=r" (result),
                              [flags] "=r" (_flags),
                            : [arg1] "r" (@as(u8, @truncate(arg1_u64))),
                              [arg2] "r" (@as(u8, @truncate(arg2_u64))),
                            : "rax", "cc"
                        );
                        break :blk result;
                    },
                    16 => blk: {
                        asm volatile (
                            \\ mov ax, %[arg1]
                            \\ or ax, %[arg2]
                            \\ pushfq
                            \\ pop %[_flags]
                            \\ movzx %[res], ax
                            : [res] "=r" (result),
                              [flags] "=r" (_flags),
                            : [arg1] "r" (@as(u16, @truncate(arg1_u64))),
                              [arg2] "r" (@as(u16, @truncate(arg2_u64))),
                            : "rax", "cc"
                        );
                        break :blk result;
                    },
                    32 => blk: {
                        asm volatile (
                            \\ mov eax, %[arg1]
                            \\ or eax, %[arg2]
                            \\ pushfq
                            \\ pop %[_flags]
                            \\ mov %[res], eax
                            : [res] "=r" (result),
                              [flags] "=r" (_flags),
                            : [arg1] "r" (@as(u32, @truncate(arg1_u64))),
                              [arg2] "r" (@as(u32, @truncate(arg2_u64))),
                            : "rax", "cc"
                        );
                        break :blk result;
                    },
                    64 => blk: {
                        asm volatile (
                            \\ mov rax, %[arg1]
                            \\ or rax, %[arg2]
                            \\ mov %[res], eax
                            \\ pushfq
                            \\ pop %[_flags]
                            : [res] "=r" (result),
                              [flags] "=r" (_flags),
                            : [arg1] "r" (arg1_u64),
                              [arg2] "r" (arg2_u64),
                            : "rax", "cc"
                        );
                        break :blk result;
                    },
                },
                ._xor => switch (size_bits) {
                    8 => blk: {
                        asm volatile (
                            \\ mov al, %[arg1]
                            \\ xor al, %[arg2]
                            \\ pushfq
                            \\ pop %[_flags]
                            \\ movzx %[res], al
                            : [res] "=r" (result),
                              [flags] "=r" (_flags),
                            : [arg1] "r" (@as(u8, @truncate(arg1_u64))),
                              [arg2] "r" (@as(u8, @truncate(arg2_u64))),
                            : "rax", "cc"
                        );
                        break :blk result;
                    },
                    16 => blk: {
                        asm volatile (
                            \\ mov ax, %[arg1]
                            \\ xor ax, %[arg2]
                            \\ pushfq
                            \\ pop %[_flags]
                            \\ movzx %[res], ax
                            : [res] "=r" (result),
                              [flags] "=r" (_flags),
                            : [arg1] "r" (@as(u16, @truncate(arg1_u64))),
                              [arg2] "r" (@as(u16, @truncate(arg2_u64))),
                            : "rax", "cc"
                        );
                        break :blk result;
                    },
                    32 => blk: {
                        asm volatile (
                            \\ mov eax, %[arg1]
                            \\ xor eax, %[arg2]
                            \\ pushfq
                            \\ pop %[_flags]
                            \\ mov %[res], eax
                            : [res] "=r" (result),
                              [flags] "=r" (_flags),
                            : [arg1] "r" (@as(u32, @truncate(arg1_u64))),
                              [arg2] "r" (@as(u32, @truncate(arg2_u64))),
                            : "rax", "cc"
                        );
                        break :blk result;
                    },
                    64 => blk: {
                        asm volatile (
                            \\ mov rax, %[arg1]
                            \\ xor rax, %[arg2]
                            \\ mov %[res], eax
                            \\ pushfq
                            \\ pop %[_flags]
                            : [res] "=r" (result),
                              [flags] "=r" (_flags),
                            : [arg1] "r" (arg1_u64),
                              [arg2] "r" (arg2_u64),
                            : "rax", "cc"
                        );
                        break :blk result;
                    },
                },
                else => return error.InvalidALUOperation,
            };

            carry = (_flags & (1 << 0)) != 0; // CF
            parity = (_flags & (1 << 2)) != 0; // PF
            zero = (_flags & (1 << 6)) != 0; // ZF
            sign = (_flags & (1 << 7)) != 0; // SF
            overflow = (_flags & (1 << 11)) != 0; // OF

        } else {
            // Fallback for non-x86-64 (e.g., ARM)
            const arg1_i64: i64 = @bitCast(arg1_u64);
            const arg2_i64: i64 = @bitCast(arg2_u64);
            const res_u64: u64 = switch (alu_op) {
                ._add => if (is_signed) @bitCast(arg1_i64 +% arg2_i64) else arg1_u64 +% arg2_u64,
                ._sub => if (is_signed) @bitCast(arg1_i64 -% arg2_i64) else arg1_u64 -% arg2_u64,
                ._and => arg1_u64 & arg2_u64,
                ._or => arg1_u64 | arg2_u64,
                ._xor => arg1_u64 ^ arg2_u64,
                ._shl => arg1_u64 << @min(@as(u6, @truncate(arg2_u64)), 63),
                ._shr => if (is_signed) @bitCast(arg1_i64 >> @min(@as(u6, @truncate(arg2_u64)), 63)) else arg1_u64 >> @min(@as(u6, @truncate(arg2_u64)), 63),
                ._mul => if (is_signed) @bitCast(arg1_i64 *% arg2_i64) else arg1_u64 *% arg2_u64,
                ._div => if (arg2_u64 == 0) return error.DivisionByZero else if (is_signed) @bitCast(arg1_i64 / arg2_i64) else arg1_u64 / arg2_u64,
                ._lookup => arg1_u64,
                ._load => arg2_u64,
                ._store => 0,
                ._sar => return error.InvalidALUOperation,
            };
            result = switch (size_bits) {
                8 => if (is_signed) @as(u64, @bitCast(@as(i8, @truncate(res_u64)))) else @truncate(res_u64 & 0xFF),
                16 => if (is_signed) @as(u64, @bitCast(@as(i16, @truncate(res_u64)))) else @truncate(res_u64 & 0xFFFF),
                32 => if (is_signed) @as(u64, @bitCast(@as(i32, @truncate(res_u64)))) else @truncate(res_u64 & 0xFFFFFFFF),
                64 => res_u64,
                else => unreachable,
            };
            const msb_mask = @as(u64, 1) << (size_bits - 1);
            carry = switch (alu_op) {
                ._add => (@as(u128, arg1_u64) + @as(u128, arg2_u64)) > std.math.maxInt(u64),
                ._sub => arg1_u64 < arg2_u64,
                ._mul => if (is_signed)
                    (@as(i128, arg1_i64) * @as(i128, arg2_i64)) > std.math.maxInt(i64) or (@as(i128, arg1_i64) * @as(i128, arg2_i64)) < std.math.minInt(i64)
                else
                    (@as(u128, arg1_u64) * @as(u128, arg2_u64)) > std.math.maxInt(u64),
                ._shl => (arg1_u64 >> (64 - @min(@as(u6, @truncate(arg2_u64)), 63))) != 0,
                else => false,
            };
            zero = result == 0;
            sign = (result & msb_mask) != 0;
            overflow = switch (alu_op) {
                ._add => ((arg1_u64 & msb_mask) == (arg2_u64 & msb_mask)) and ((result & msb_mask) != (arg1_u64 & msb_mask)),
                ._sub => ((arg1_u64 & msb_mask) != (arg2_u64 & msb_mask)) and ((result & msb_mask) != (arg1_u64 & msb_mask)),
                ._mul => carry,
                else => false,
            };
            parity = @popCount(result) % 2 == 0;
        }

        // Store result
        if (is_64bit) {
            self.R[dst] = @truncate(result >> 32);
            self.R[dst + 1] = @truncate(result);
        } else {
            self.R[dst] = @truncate(result);
        }

        // Set flags
        self.setFlag(.C, @intFromBool(carry));
        self.setFlag(.Z, @intFromBool(zero));
        self.setFlag(.S, @intFromBool(sign));
        self.setFlag(.V, @intFromBool(overflow));
        self.setFlag(.P, @intFromBool(parity));
    }
    pub fn executeJMP(self: *CPU, _ip: RegType, op: u4) !void {
        const stride = self.R[jmp_stride];
        const bcs_raw: u4 = @truncate(self.getFlag(.BCS)); // BCS from FLAGS (bits 16-19)
        const bcs: BranchCondition = @enumFromInt(bcs_raw);

        const v: bool = switch (bcs) {
            .A => true, // Always jump
            .Z => self.getFlag(.Z) != 0, // Zero flag set
            .NZ => self.getFlag(.Z) == 0, // Zero flag not set
            .G => blk: { // Greater: S == V && Z == 0 (signed comparison)
                const s = self.getFlag(.S);
                const v_flag = self.getFlag(.V);
                const z = self.getFlag(.Z);
                break :blk s == v_flag and z == 0;
            },
            .GE => blk: { // Greater or Equal: S == V (includes Z == 1)
                const s = self.getFlag(.S);
                const v_flag = self.getFlag(.V);
                break :blk s == v_flag;
            },
            .L => blk: { // Less: S != V
                const s = self.getFlag(.S);
                const v_flag = self.getFlag(.V);
                break :blk s != v_flag;
            },
            .LE => blk: { // Less or Equal: S != V || Z == 1
                const s = self.getFlag(.S);
                const v_flag = self.getFlag(.V);
                const z = self.getFlag(.Z);
                break :blk s != v_flag or z != 0;
            },
            .C => self.getFlag(.C) != 0, // Carry flag set
            .NC => self.getFlag(.C) == 0, // Carry flag not set
            .S => self.getFlag(.S) != 0, // Sign flag set (negative)
            .NS => self.getFlag(.S) == 0, // Sign flag not set (non-negative)
            .O => self.getFlag(.V) != 0, // Overflow flag set
            .NO => self.getFlag(.V) == 0, // Overflow flag not set
            .PE => self.getFlag(.P) != 0, // Parity even
            .PO => self.getFlag(.P) == 0, // Parity odd
            .I => self.getFlag(.I) != 0, // Interrupt flag set
        };

        if (v) {
            self.R[ip] = _ip +% stride *% op;
        }
    }

    pub fn executeCALL(self: *CPU, _ip: RegType, op: u4) !void {
        try executePUSHVal(self, ip + 1);
        try executeJMP(self, _ip, op);
    }

    pub fn executeLI(self: *CPU, operand: u4) !void {
        // Get the current register and nibble indices
        const rs_index: u4 = @truncate(self.getFlag(.RS)); // Register to load into
        const ns = self.getFlag(.NS); // Current nibble position (0 to WS/4 - 1)
        var reg_value = self.R.get(rs_index); // Current value of the register

        // Clear the target nibble and load the immediate u4 value
        const shift: u5 = @truncate(ns * 4); // Each nibble is 4 bits
        const mask = @as(RegType, 0xF) << shift; // Mask for the target nibble
        reg_value = (reg_value & ~mask) | (@as(RegType, operand) << shift);

        // Store the updated value
        self.R.set(rs_index, reg_value);

        // Increment N.NS for chaining, wrap around if exceeding register size
        const max_nibbles = WS / 4;
        const new_ns = if (ns + 1 < max_nibbles) ns + 1 else 0;
        self.R.setFlag(.NS, new_ns);
    }

    pub fn executeLIS(self: *CPU, operand: u4) !void {
        // Get the current register and nibble indices
        const rs_index: u4 = @truncate(self.getFlag(.RS)); // Register to load into
        const ns = self.getFlag(.NS); // Current nibble position (0 to WS/4 - 1)
        var reg_value = self.R.get(rs_index); // Current value of the register

        // Load the immediate i4 value into the target nibble
        const shift: u5 = @truncate(ns * 4); // Each nibble is 4 bits
        const mask = @as(RegType, 0xF) << shift; // Mask for the target nibble
        const imm_value = @as(RegType, operand); // Immediate as unsigned
        reg_value = (reg_value & ~mask) | (imm_value << shift);

        // Sign-extend if the immediate is negative (MSB of i4 is 1)
        if (operand & 0x8 != 0) { // Check sign bit of i4 (0x8 = 1000 in binary)
            const sign_mask = blk: {
                var rmask: RegType = 0;
                var i: u5 = shift + 4; // Start from the next bit after the nibble
                while (i < WS) : (i += 1) {
                    rmask |= @as(RegType, 1) << i;
                }
                break :blk rmask;
            };
            reg_value |= sign_mask; // Extend 1s to upper bits
        } else {
            const clear_mask = blk: {
                var rmask: RegType = 0;
                var i: u5 = shift + 4; // Start from the next bit after the nibble
                while (i < WS) : (i += 1) {
                    rmask |= @as(RegType, 1) << i;
                }
                break :blk rmask;
            };
            reg_value &= ~clear_mask; // Clear upper bits to 0
        }

        // Store the updated value
        self.R.set(rs_index, reg_value);

        // Increment N.NS for chaining, wrap around if exceeding register size
        const max_nibbles = WS / 4;
        const new_ns = if (ns + 1 < max_nibbles) ns + 1 else 0;
        self.R.setFlag(.NS, new_ns);
    }

    // Push a register value onto the stack
    pub fn executePUSHVal(self: *CPU, val: RegType) !void {
        // Get the current stack pointer
        var _sp = self.R[sp];

        // Check for stack overflow
        if (_sp + @sizeOf(RegType) > self.M.len) {
            return error.StackOverflow;
        }

        // Write the value to the stack and increment SP
        self.writeMemory(_sp, val);
        _sp += @sizeOf(RegType);

        // Update SP
        self.R[sp] = _sp;
    }

    // Push a register value onto the stack
    pub fn executePUSH(self: *CPU, reg: u4) !void {
        // Get the value from the specified register
        const value = self.R[reg];
        try executePUSHVal(self, value);
    }

    // Pop a value from the stack into a register
    pub fn executePOP(self: *CPU, reg: u4) !void {
        // Get the current stack pointer
        var _sp = self.R[sp];

        // Check for stack underflow
        if (_sp < @sizeOf(RegType)) {
            return error.StackUnderflow;
        }

        // Decrement SP and read the value from the stack
        _sp -= @sizeOf(RegType);
        const value = self.readMemory(_sp);

        // Store the value in the specified register and update SP
        self.R[reg] = value;
        self.R[sp] = _sp;
    }

    pub fn execute(self: *CPU) !void {
        while (true) {
            // Fetch the current instruction
            const _ip = self.R[ip]; // IP is R15
            if (_ip >= self.M.len) {
                return error.ProgramCounterOutOfBounds;
            }
            const instruction_byte = self.M[ip];
            const instruction: Instruction = @bitCast(instruction_byte);

            // Decode and execute the instruction
            switch (@as(Opcode, @enumFromInt(instruction.opcode))) {
                .NOP => switch (@as(SubOpcode, @enumFromInt(instruction.operand))) {
                    .NOP => {}, // No operation
                    .RET => return self.executeRET(),
                    .IRET => return self.executeIRET(),
                    .SETC => self.R.setFlag(.C, 1),
                    .CLSC => self.R.setFlag(.C, 0),
                    .INC => try self.executeINC(),
                    .DEC => try self.executeDEC(),
                    .NOT => try self.executeNOT(),
                    .CMP => try self.executeCMP(),
                },
                .RS => try self.executeRS(instruction.operand),
                .NS => self.R.setFlag(.NS, instruction.operand),
                .LI => try self.executeLI(instruction.operand),
                .LIS => try self.executeLIS(instruction.operand),
                .ALU => try self.executeALU(instruction.operand),
                .JMP => try self.executeJMP(ip, instruction.operand),
                .CALL => try self.executeCALL(ip, instruction.operand),
                .PUSH => try self.executePUSH(instruction.operand),
                .POP => try self.executePOP(instruction.operand),
                .INT => return error.InvalidInstruction, //try self.executeINT(instruction.operand),
                .IN => try self.executeIN(instruction.operand),
                .OUT => try self.executeOUT(instruction.operand),
                // Reserved opcodes (0xD to 0xF) fall through to error
            }

            // Check for reserved opcodes
            if (instruction.opcode > @intFromEnum(Opcode.OUT)) {
                return error.InvalidInstruction;
            }

            // Increment the instruction pointer (IP)
            self.R[ip] = _ip + 1;
        }
    }
};

pub fn main() !void {
    var arena = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    defer arena.deinit();
    const allocator = arena.allocator();

    const args = try std.process.argsAlloc(allocator);
    defer std.process.argsFree(allocator, args);

    if (args.len < 2) {
        std.log.err("Usage: vdca <program_file> [-at <load_address>]", .{});
        return error.MissingArguments;
    }

    const program_path = args[1];
    var load_address: RegType = 0;

    var i: usize = 2; // Start from the second argument
    while (i < args.len) : (i += 1) {
        const arg = args[i];
        if (std.mem.eql(u8, arg, "-at") and i + 1 < args.len) {
            load_address = try std.fmt.parseInt(RegType, args[i + 1], 10);
            i += 1; // Skip the next argument since it's the address
        }
    }

    var cpu = try CPU.init(allocator, 4 * 1024);
    defer cpu.deinit(allocator);

    try cpu.loadProgram(program_path, load_address);
    try cpu.execute();
}
