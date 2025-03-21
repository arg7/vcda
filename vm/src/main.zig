const std = @import("std");
const registers = @import("registers.zig");
const assert = std.debug.assert;
const fs = std.fs;

const WS = 32;
const HWS = WS / 2;

// Derive the register type based on WS
const RegType = switch (WS) {
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
    NOP = 0x0,  // No operation (and sub-opcodes below)
    RS = 0x1,   // Set N.RS
    NS = 0x2,   // Set N.NS
    LI = 0x3,   // Load Immediate
    LIS = 0x4,  // Load Immediate Signed
    ALU = 0x5,  // ALU operation
    JMP = 0x6,  // Jump
    CALL = 0x7, // Call
    PUSH = 0x8, // Push
    POP = 0x9,  // Pop
    INT = 0xA,  // Interrupt
    IN = 0xB,   // Input
    OUT = 0xC,  // Output
    // Note: 0xD to 0xF are reserved for future use
};

// Sub-opcodes for NOP (opcode 0x0)
pub const SubOpcode = enum(u4) {
    NOP = 0x0,        // No operation
    RET = 0x1,        // Return
    IRET = 0x2,       // Interrupt Return
    SETC = 0x3,       // Set Carry
    CLSC = 0x4,       // Clear Carry
    INC = 0x5,        // Increment
    DEC = 0x6,        // Decrement
    NOT = 0x7,        // Bitwise NOT
    CMP = 0x8,        // Compare
    FMT_WORD = 0x9,   // FMT WORD
    FMT_BYTE = 0xA,   // FMT BYTE
    FMT_NIBBLE = 0xB, // FMT NIBBLE
    FMT_BIN = 0xC,    // FMT BIN
    FMT_HEX = 0xD,    // FMT HEX
    // Note: 0xE and 0xF are reserved for future use
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
        const reg_index = self.R.getFlag(.RS); // Get the register from RS flag
        const value = self.R.get(reg_index);   // Get full register value (now RegType)
        
        // Apply the operation
        const result = op(value);
        
        // Store the result
        self.R.set(reg_index, result);
        
        // Update flags
        self.R.setFlag(.Z, result == 0);              // Zero flag
        const msb_mask = @as(RegType, 1) << (WS - 1); // Calculate MSB mask
        self.R.setFlag(.S, (result & msb_mask) != 0); // Sign flag using WS
        
        // Update carry flag if requested
        if (set_carry) {
            const old_msb = (value & msb_mask) != 0;
            const new_msb = (result & msb_mask) != 0;
            // Set C if MSB flips when it shouldn't
            self.R.setFlag(.C, old_msb != new_msb and switch (op) {
                incOp => old_msb == false, // Carry if going from positive to negative
                decOp => old_msb == true,  // Borrow if going from negative to positive
                else => false,             // No carry for other ops like NOT
            });
        }
    }

    // Define the specific operations as inline functions
    inline fn incOp(value: RegType) RegType {
        return value + 1;
    }

    inline fn decOp(value: RegType) RegType {
        return value - 1;
    }

    inline fn notOp(value: RegType) RegType {
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
        const rs_index = self.R.getFlag(.RS);  // First operand register
        const src_index = self.R.getFlag(.SRC); // Second operand register
        
        // Get the values from the registers
        const rs_value = self.R.get(rs_index);   // Value of RS register
        const src_value = self.R.get(src_index); // Value of SRC register
        
        // Perform subtraction to compare (but don't store the result)
        const result = rs_value -% src_value; // Wrapping subtraction for unsigned comparison
        
        // Calculate MSB mask based on word size
        const msb_mask = @as(RegType, 1) << (WS - 1);
        
        // Update flags based on the comparison
        self.R.setFlag(.Z, result == 0);              // Zero flag: set if result is 0
        self.R.setFlag(.S, (result & msb_mask) != 0); // Sign flag: set if result is negative
        
        // Carry flag: set if unsigned borrow occurred (rs_value < src_value)
        self.R.setFlag(.C, rs_value < src_value);     // Unsigned comparison
        
        // Overflow flag: set if signed overflow occurred
        const rs_msb = (rs_value & msb_mask) != 0;
        const src_msb = (src_value & msb_mask) != 0;
        const result_msb = (result & msb_mask) != 0;
        self.R.setFlag(.V, (rs_msb != src_msb) and (result_msb != rs_msb));
    }

    // Assuming SP (stack pointer) is R14, IP (instruction pointer) is R15, and FLAGS is R13
    fn executeRET(self: *CPU) !void {
        // Get the current stack pointer
        const sp_index = self.R.SP; // Assuming R14 is SP
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
        self.R.set(self.R.IP, return_address); // IP is R15
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
        const sp_index = self.R.SP; // Assuming R14 is SP
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
        self.R.set(self.R.IP, return_address); // IP is R15
        self.R.set(13, flags);                 // Assuming R13 is FLAGS
    }

    fn executeIN(self: *CPU) !void {
        const io_channel = self.currentInstruction().operand;
        const reg_index = self.R.getFlag(.RS); // Get register index from FLAGS.RS
        const value = self.getFormattedValue(reg_index); // Get formatted value based on FMT

        switch (io_channel) {
            0x0 => try self.writeToStdOut(value),
            0x1 => try self.writeToStdErr(value),
            else => return error.InvalidIOChannel,
        }
    }

    fn executeOUT(self: *CPU) !void {
        const io_channel = self.currentInstruction().operand;
        const reg_index = self.R.getFlag(.RS); // Get register index from FLAGS.RS

        const value = switch (io_channel) {
            0x0 => try self.readFromStdIn(),
            0x1 => return error.StdErrIsWriteOnly, // stderr is write-only
            else => return error.InvalidIOChannel,
        };

        self.setFormattedValue(reg_index, value); // Set formatted value based on FMT
    }

    fn getFormattedValue(self: *CPU, reg_index: u4) RegType {
        const fmt = self.R.getFlag(.FMT); // Get FMT from FLAGS
        const reg_value = self.R.get(reg_index);

        return switch (fmt) {
            0x0 => reg_value, // FMT WORD: entire register
            0x1 => reg_value & 0xFF, // FMT BYTE: least significant byte
            0x2 => (reg_value >> (self.R.getFlag(.NS) * 4) & 0xF), // FMT NIBBLE: selected nibble
            0x3 => reg_value, // FMT BIN: raw bits (same as WORD for now)
            0x4 => reg_value, // FMT HEX: as hex (same as WORD for now)
            else => unreachable, // Invalid FMT
        };
    }

    fn setFormattedValue(self: *CPU, reg_index: u4, value: RegType) void {
        const fmt = self.R.getFlag(.FMT); // Get FMT from FLAGS
        var reg_value = self.R.get(reg_index);

        switch (fmt) {
            0x0 => reg_value = value, // FMT WORD: entire register
            0x1 => reg_value = (reg_value & ~0xFF) | (value & 0xFF), // FMT BYTE: least significant byte
            0x2 => {
                const ns = self.R.getFlag(.NS);
                const shift = ns * 4;
                reg_value = (reg_value & ~(@as(RegType, 0xF) << shift)) | ((value & 0xF) << shift); // FMT NIBBLE: selected nibble
            },
            0x3 => reg_value = value, // FMT BIN: raw bits (same as WORD for now)
            0x4 => reg_value = value, // FMT HEX: as hex (same as WORD for now)
            else => unreachable, // Invalid FMT
        }

        self.R.set(reg_index, reg_value);
    }

    fn writeToStdOut(self: *CPU, value: RegType) !void {
        const writer = std.io.getStdOut().writer();
        try writer.print("{}\n", .{value});
    }

    fn writeToStdErr(self: *CPU, value: RegType) !void {
        const writer = std.io.getStdErr().writer();
        try writer.print("{}\n", .{value});
    }

    fn readFromStdIn(self: *CPU) !RegType {
        const reader = std.io.getStdIn().reader();
        var buf: [32]u8 = undefined;
        const line = try reader.readUntilDelimiterOrEof(&buf, '\n') orelse return 0;
        return std.fmt.parseInt(RegType, line, 10) catch 0; // Parse as decimal, default to 0 on error
    }
    
    fn executeLI(self: *CPU, operand: u4) !void {
        // Get the current register and nibble indices
        const rs_index = self.R.getFlag(.RS); // Register to load into
        const ns = self.R.getFlag(.NS);       // Current nibble position (0 to WS/4 - 1)
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
        const ns = self.R.getFlag(.NS);       // Current nibble position (0 to WS/4 - 1)
        var reg_value = self.R.get(rs_index); // Current value of the register
        
        // Load the immediate i4 value into the target nibble
        const shift = ns * 4; // Each nibble is 4 bits
        const mask = @as(RegType, 0xF) << shift; // Mask for the target nibble
        const imm_value = @as(RegType, operand); // Immediate as unsigned
        reg_value = (reg_value & ~mask) | (imm_value << shift);
        
        // Sign-extend if the immediate is negative (MSB of i4 is 1)
        if (operand & 0x8 != 0) { // Check sign bit of i4 (0x8 = 1000 in binary)
            const sign_mask = comptime blk: {
                var mask: RegType = 0;
                var i: usize = shift + 4; // Start from the next bit after the nibble
                while (i < WS) : (i += 1) {
                    mask |= @as(RegType, 1) << i;
                }
                break :blk mask;
            };
            reg_value |= sign_mask; // Extend 1s to upper bits
        } else {
            const clear_mask = comptime blk: {
                var mask: RegType = 0;
                var i: usize = shift + 4; // Start from the next bit after the nibble
                while (i < WS) : (i += 1) {
                    mask |= @as(RegType, 1) << i;
                }
                break :blk mask;
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
        var sp = self.R.get(self.R.SP);
        
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
        self.R.set(self.R.SP, sp);
    }

    // Pop a value from the stack into a register
    fn executePOP(self: *CPU, reg: u4) !void {
        // Get the current stack pointer
        var sp = self.R.get(self.R.SP);
        
        // Check for stack underflow
        if (sp < @sizeOf(RegType)) {
            return error.StackUnderflow;
        }
        
        // Decrement SP and read the value from the stack
        sp -= @sizeOf(RegType);
        const value = self.readMemory(sp);
        
        // Store the value in the specified register and update SP
        self.R.set(reg, value);
        self.R.set(self.R.SP, sp);
    }
    
    pub fn execute(self: *CPU) !void {
        while (true) {
            // Fetch the current instruction
            const ip = self.R.get(self.R.IP); // IP is R15
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
                    .SETC => self.R.setFlag(.C, true),
                    .CLSC => self.R.setFlag(.C, false),
                    .INC => try self.executeINC(),
                    .DEC => try self.executeDEC(),
                    .NOT => try self.executeNOT(),
                    .CMP => try self.executeCMP(),
                    .FMT_WORD => self.R.setFlag(.FMT, 0x0),   // FMT WORD
                    .FMT_BYTE => self.R.setFlag(.FMT, 0x1),   // FMT BYTE
                    .FMT_NIBBLE => self.R.setFlag(.FMT, 0x2), // FMT NIBBLE
                    .FMT_BIN => self.R.setFlag(.FMT, 0x3),    // FMT BIN
                    .FMT_HEX => self.R.setFlag(.FMT, 0x4),    // FMT HEX
                    // No else case needed; invalid sub-opcodes are handled below
                },
                .RS => try self.executeRS(instruction.operand),
                .NS => self.R.setFlag(.NS, instruction.operand),
                .LI => try self.executeLI(instruction.operand),
                .LIS => try self.executeLIS(instruction.operand),
                .ALU => try self.executeALU(instruction.operand),
                .JMP => try self.executeJMP(instruction.operand),
                .CALL => try self.executeCALL(instruction.operand),
                .PUSH => try self.executePUSH(instruction.operand),
                .POP => try self.executePOP(instruction.operand),
                .INT => try self.executeINT(instruction.operand),
                .IN => try self.executeIN(),
                .OUT => try self.executeOUT(),
                // Reserved opcodes (0xD to 0xF) fall through to error
            }

            // Check for invalid sub-opcodes under NOP
            if (instruction.opcode == @intFromEnum(Opcode.NOP) and 
                instruction.operand > @intFromEnum(SubOpcode.FMT_HEX)) {
                return error.InvalidInstruction;
            }
            // Check for reserved opcodes
            if (instruction.opcode > @intFromEnum(Opcode.OUT)) {
                return error.InvalidInstruction;
            }

            // Increment the instruction pointer (IP)
            self.R.set(self.R.IP, ip + 1);
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
