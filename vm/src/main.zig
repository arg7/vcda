const std = @import("std");
const registers = @import("registers.zig");
const assert = std.debug.assert;
const fs = std.fs;

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

pub const ALUDataType = enum(u4) {
    u8 = 0x0,
    i8 = 0x1,
    u16 = 0x2,
    i16 = 0x3,
    u32 = 0x4,
    i32 = 0x5,
    u64 = 0x6,
    i64 = 0x7,
    f16 = 0x8,
    f32 = 0x9,
    f64 = 0xA,
    u1 = 0xB,
    i4 = 0xC,
    fp4 = 0xD,
    fp8 = 0xE,
    reserved = 0xF,
};

pub const BranchCondition = enum(u4) {
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

pub const ALUOperation = enum(u4) { _add = 0x0, _sub = 0x1, _and = 0x2, _or = 0x3, _xor = 0x4, _shl = 0x5, _shr = 0x6, _sar = 0x7, _mul = 0x8, _div = 0x9, _lookup = 0xA, _load = 0xB, _store = 0xC };

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

pub const Instruction = packed struct {
    opcode: u4,
    operand: u4,
};

pub const CPU = struct {
    R: registers.Registers = .{}, // Use the Registers struct
    M: []u8, // Memory for the CPU

    // Initialize the CPU with default register values
    pub fn init(allocator: std.mem.Allocator, memory_size: usize) !CPU {
        var cpu = CPU{ .M = try allocator.alloc(u8, memory_size) };
        cpu.R.init(); // Initialize registers
        return cpu;
    }

    // Reset registers to default values
    pub fn resetRegisters(self: *CPU) void {
        self.R.init();
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
    fn executeUnaryOp(self: *CPU, comptime op: fn (RegType) RegType, comptime set_carry: bool) !void {
        const reg_index: u4 = @truncate(self.R.getFlag(.RS)); // Get the register from RS flag
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
    fn executeINC(self: *CPU) !void {
        try self.executeUnaryOp(incOp, true); // Set carry flag for INC
    }

    fn executeDEC(self: *CPU) !void {
        try self.executeUnaryOp(decOp, true); // Set carry flag for DEC
    }

    fn executeNOT(self: *CPU) !void {
        try self.executeUnaryOp(notOp, false); // No carry flag for NOT
    }

    fn executeCMP(self: *CPU) !void {
        // Get the register indices from RS and SRC flags
        const rs_index: u4 = @truncate(self.R.getFlag(.RS)); // First operand register
        const src_index: u4 = @truncate(self.R.getFlag(.SRC)); // Second operand register

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
    fn executeRET(self: *CPU) !void {
        // Get the current stack pointer
        const sp_index = @intFromEnum(registers.Registers.Reg.SP);
        var sp = self.R.get(sp_index);

        // Check if stack underflow would occur
        if (sp >= self.M.len) {
            return error.StackUnderflow;
        }

        // Pop the return address from the stack
        const return_address = self.readMemory(sp);
        sp += @sizeOf(RegType); // Increment SP by size of RegType

        // Update SP and IP
        self.R.set(sp_index, sp);
        self.R.set(@intFromEnum(registers.Registers.Reg.IP), return_address); // IP is R15
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

    fn executeIRET(self: *CPU) !void {
        // Get the current stack pointer
        const sp_index = @intFromEnum(registers.Registers.Reg.SP); // Assuming R14 is SP
        var sp = self.R.get(sp_index);

        // Check if stack underflow would occur (need space for address + flags)
        if (sp + @sizeOf(RegType) >= self.M.len) {
            return error.StackUnderflow;
        }

        // Pop the return address and flags from the stack
        const return_address = self.readMemory(sp);
        sp += @sizeOf(RegType);
        const flags = self.readMemory(sp);
        sp += @sizeOf(RegType);

        // Update SP, IP, and FLAGS register
        self.R.set(sp_index, sp);
        self.R.set(@intFromEnum(registers.Registers.Reg.IP), return_address); // IP is R15
        self.R.set(13, flags); // Assuming R13 is FLAGS
    }

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
                    @memcpy(padded[digits - len.len..], len);
                    break :blk try allocator.dupe(u8, padded); // dupe the slice data if needed
                }

                break :blk try allocator.dupe(u8, len);  // Correct usage outside the if
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
                    @memcpy(padded[bits - len.len..], len);
                    break :blk try allocator.dupe(u8, padded[0..bits]);
                }
                break :blk try allocator.dupe(u8, len);
            },
            .Float => blk: {
                if (length != .Word) return error.InvalidFloatLength;
                const float_value: f32 = @bitCast(rvalue);
                const fmt_strings = [_][]const u8{
                    "{d}", "{d:.1}", "{d:.2}", "{d:.3}", "{d:.4}", "{d:.5}", "{d:.6}", "{d:.7}",
                };
                if (precision > fmt_strings.len) return error.InvalidPrecision;
                const allocated_string = try std.fmt.allocPrint(allocator, fmt_strings[precision], .{float_value});
                defer allocator.free(allocated_string);
                @memcpy(buf[0..allocated_string.len], allocated_string);
                const len = allocated_string.len;
                break :blk try allocator.dupe(u8, buf[0..len]);
            },
        };
    }

    // OUT execution
    fn executeOUT(self: *CPU, operand: u4) !void {
        const io_channel = operand;
        const reg_index_rs = self.R.getFlag(.RS); // Rs (value)
        const reg_index_src = self.R.getFlag(.SRC); // Rt (format)
        const rvalue = self.R.get(reg_index_rs);
        const rfmt = self.R.get(reg_index_src);

        // Format the value
        const val = try format(rvalue, rfmt);

        // Write to appropriate I/O channel
        switch (io_channel) {
            0x0 => try self.writeToStdOut(val),
            0x1 => try self.writeToStdErr(val),
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
    fn executeIN(self: *CPU, operand: u4) !void {
        const io_channel = operand;
        const reg_index_rd = self.R.getFlag(.RS); // Rd (destination)
        const reg_index_rs = self.R.getFlag(.SRC); // Rs (format)
        const rfmt = self.R.get(reg_index_rs);

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

    fn executeRS(self: *CPU, operand: u4) !void {
        self.R.setFlag(.RS, operand);
    }
    fn executeNS(self: *CPU, operand: u4) !void {
        self.R.setFlag(.NS, operand);
    }

    fn executeALU(self: *CPU, op: u4) !void {
        const reg_index_arg1 = self.R.getFlag(.RS);  // First operand (arg1)
        const reg_index_arg2 = self.R.getFlag(.SRC); // Second operand (arg2)
        const reg_index_dst = self.R.getFlag(.DST);  // Destination register
        const arg1 = self.R.get(reg_index_arg1);     // Value of arg1 (u32)
        const arg2 = self.R.get(reg_index_arg2);     // Value of arg2 (u32)

        // Perform operation based on 4-bit op selector
        const res: u32 = switch (op) {
            0x0 => arg1 +% arg2,              // ADD: Addition with wrapping
            0x1 => arg1 -% arg2,              // SUB: Subtraction with wrapping
            0x2 => arg1 & arg2,               // AND: Bitwise AND
            0x3 => arg1 | arg2,               // OR: Bitwise OR
            0x4 => arg1 ^ arg2,               // XOR: Bitwise Exclusive OR
            0x5 => arg1 << @as(u5, @truncate(arg2)), // SHL: Shift Left (truncate to 5 bits)
            0x6 => arg1 >> @as(u5, @truncate(arg2)), // SHR: Shift Right Logical (zero-fill)
            0x7 => blk: {                     // SAR: Shift Arithmetic Right (sign-extend)
                const signed_arg1: i32 = @bitCast( arg1); // Fixed: u32 to i32
                const shift = @as(u5, @truncate(arg2));
                break :blk @bitCast(signed_arg1 >> shift);
            },
            0x8 => arg1 *% arg2,              // MUL: Multiplication with wrapping
            0x9 => blk: {                     // DIV: Division (unsigned)
                if (arg2 == 0) return error.DivisionByZero;
                break :blk arg1 / arg2;
            },
            else => return error.InvalidALUOperation, // Handle undefined ops (0xA-0xF)
        };

        // Store result in destination register
        self.R.set(reg_index_dst, res);
    }

    fn executeLI(self: *CPU, operand: u4) !void {
        // Get the current register and nibble indices
        const rs_index: u4 = @truncate(self.R.getFlag(.RS)); // Register to load into
        const ns = self.R.getFlag(.NS); // Current nibble position (0 to WS/4 - 1)
        var reg_value = self.R.get(rs_index); // Current value of the register

        // Clear the target nibble and load the immediate u4 value
        const shift = ns * 4; // Each nibble is 4 bits
        const mask = @as(RegType, 0xF) << shift; // Mask for the target nibble
        reg_value = (reg_value & ~mask) | (@as(RegType, operand) << shift);

        // Store the updated value
        self.R.set(rs_index, reg_value);

        // Increment N.NS for chaining, wrap around if exceeding register size
        const max_nibbles = WS / 4;
        const new_ns = if (ns + 1 < max_nibbles) ns + 1 else 0;
        self.R.setFlag(.NS, new_ns);
    }

    fn executeLIS(self: *CPU, operand: u4) !void {
        // Get the current register and nibble indices
        const rs_index = self.R.getFlag(.RS); // Register to load into
        const ns = self.R.getFlag(.NS); // Current nibble position (0 to WS/4 - 1)
        var reg_value = self.R.get(rs_index); // Current value of the register

        // Load the immediate i4 value into the target nibble
        const shift = ns * 4; // Each nibble is 4 bits
        const mask = @as(RegType, 0xF) << shift; // Mask for the target nibble
        const imm_value = @as(RegType, operand); // Immediate as unsigned
        reg_value = (reg_value & ~mask) | (imm_value << shift);

        // Sign-extend if the immediate is negative (MSB of i4 is 1)
        if (operand & 0x8 != 0) { // Check sign bit of i4 (0x8 = 1000 in binary)
            const sign_mask =  blk: {
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
    fn executePUSH(self: *CPU, reg: u4) !void {
        // Get the current stack pointer
        var sp = self.R.get(@intFromEnum(registers.Registers.Reg.SP));

        // Check for stack overflow
        if (sp + @sizeOf(RegType) > self.M.len) {
            return error.StackOverflow;
        }

        // Get the value from the specified register
        const value = self.R.get(reg);

        // Write the value to the stack and increment SP
        self.writeMemory(sp, value);
        sp += @sizeOf(RegType);

        // Update SP
        self.R.set(@intFromEnum(registers.Registers.Reg.SP), sp);
    }

    // Pop a value from the stack into a register
    fn executePOP(self: *CPU, reg: u4) !void {
        // Get the current stack pointer
        var sp = self.R.get(@intFromEnum(registers.Registers.Reg.SP));

        // Check for stack underflow
        if (sp < @sizeOf(RegType)) {
            return error.StackUnderflow;
        }

        // Decrement SP and read the value from the stack
        sp -= @sizeOf(RegType);
        const value = self.readMemory(sp);

        // Store the value in the specified register and update SP
        self.R.set(reg, value);
        self.R.set(@intFromEnum(registers.Registers.Reg.SP), sp);
    }

    pub fn execute(self: *CPU) !void {
        while (true) {
            // Fetch the current instruction
            const ip = self.R.get(@intFromEnum(registers.Registers.Reg.IP)); // IP is R15
            if (ip >= self.M.len) {
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
                .JMP => {}, //try self.executeJMP(instruction.operand),
                .CALL => {}, //try self.executeCALL(instruction.operand),
                .PUSH => try self.executePUSH(instruction.operand),
                .POP => try self.executePOP(instruction.operand),
                .INT => {}, //try self.executeINT(instruction.operand),
                .IN => try self.executeIN(instruction.operand),
                .OUT => try self.executeOUT(instruction.operand),
                // Reserved opcodes (0xD to 0xF) fall through to error
            }

            // Check for invalid sub-opcodes under NOP
            if (instruction.opcode == @intFromEnum(Opcode.NOP) and
                instruction.operand > @intFromEnum(SubOpcode.FMT_HEX))
            {
                return error.InvalidInstruction;
            }
            // Check for reserved opcodes
            if (instruction.opcode > @intFromEnum(Opcode.OUT)) {
                return error.InvalidInstruction;
            }

            // Increment the instruction pointer (IP)
            self.R.set(@intFromEnum(@intFromEnum(registers.Registers.Reg.IP)), ip + 1);
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
