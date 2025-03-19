const std = @import("std");
const registers = @import("registers.zig");
const assert = std.debug.assert;
const fs = std.fs;

const WS = 32;
const HWS = WS / 2;

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

pub const ALUOperation = enum(u4) {
    _add = 0x0,
    _sub = 0x1,
    _and = 0x2,
    _or = 0x3,
    _xor = 0x4,
    _shl = 0x5,
    _shr = 0x6,
    _sar = 0x7,
    _mul = 0x8,
    _div = 0x9,
    _lookup = 0xA,
    _load = 0xB,
    _store = 0xC
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
        var cpu = CPU{
            .M = try allocator.alloc(u8, memory_size)
        };
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

    fn loadProgram(self: *CPU, program_path: []const u8, load_address: u32) !void {

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
            }  else if (!std.ascii.isWhitespace(byte)) {
                std.log.warn("Invalid character in hex file: {}", .{byte});
                return error.InvalidHexFile;  // Or handle differently
            }

        }

        if (hex_chars_read != 0) {
            std.log.warn("Incomplete byte in hex file. Ignoring last nibble(s).", .{}); // Or handle as an error
        }

      } 
      else {
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

    fn getFormattedValue(self: *CPU, reg_index: u4) u32 {
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

    fn setFormattedValue(self: *CPU, reg_index: u4, value: u32) void {
        const fmt = self.R.getFlag(.FMT); // Get FMT from FLAGS
        var reg_value = self.R.get(reg_index);

        switch (fmt) {
            0x0 => reg_value = value, // FMT WORD: entire register
            0x1 => reg_value = (reg_value & ~0xFF) | (value & 0xFF), // FMT BYTE: least significant byte
            0x2 => {
                const ns = self.R.getFlag(.NS);
                const shift = ns * 4;
                reg_value = (reg_value & ~(@as(u32, 0xF) << shift)) | ((value & 0xF) << shift); // FMT NIBBLE: selected nibble
            },
            0x3 => reg_value = value, // FMT BIN: raw bits (same as WORD for now)
            0x4 => reg_value = value, // FMT HEX: as hex (same as WORD for now)
            else => unreachable, // Invalid FMT
        }

        self.R.set(reg_index, reg_value);
    }

    fn writeToStdOut(self: *CPU, value: u32) !void {
        const writer = std.io.getStdOut().writer();
        try writer.print("{}\n", .{value});
    }

    fn writeToStdErr(self: *CPU, value: u32) !void {
        const writer = std.io.getStdErr().writer();
        try writer.print("{}\n", .{value});
    }

    fn readFromStdIn(self: *CPU) !u32 {
        const reader = std.io.getStdIn().reader();
        var buf: [32]u8 = undefined;
        const line = try reader.readUntilDelimiterOrEof(&buf, '\n') orelse return 0;
        return std.fmt.parseInt(u32, line, 10) catch 0; // Parse as decimal, default to 0 on error
    }

    pub fn execute(self: *CPU) !void {
        while (true) {
            // Fetch the current instruction
            const ip = self.R.get(15); // IP is R15
            if (ip >= self.M.len) {
                return error.ProgramCounterOutOfBounds;
            }
            const instruction_byte = self.M[ip];
            const instruction: Instruction = @bitCast(instruction_byte);

            // Decode and execute the instruction
            switch (instruction.opcode) {
                0x0 => switch (instruction.operand) {
                    0x0 => {}, // NOP
                    0x1 => return self.executeRET(), // RET
                    0x2 => return self.executeIRET(), // IRET
                    0x3 => self.R.setFlag(.C, true), // SETC
                    0x4 => self.R.setFlag(.C, false), // CLSC
                    0x5 => try self.executeINC(), // INC
                    0x6 => try self.executeDEC(), // DEC
                    0x7 => try self.executeNOT(), // NOT
                    0x8 => try self.executeCMP(), // CMP
                    else => return error.InvalidInstruction,
                },
                0x1 => self.R.setFlag(.RS, instruction.operand), // RS
                0x2 => self.R.setFlag(.NS, instruction.operand), // NS
                0x3 => try self.executeLI(instruction.operand), // LI
                0x4 => try self.executeLIS(instruction.operand), // LIS
                0x5 => try self.executeALU(instruction.operand), // ALU
                0x6 => try self.executeJMP(instruction.operand), // JMP
                0x7 => try self.executeCALL(instruction.operand), // CALL
                0x8 => try self.executePUSH(instruction.operand), // PUSH
                0x9 => try self.executePOP(instruction.operand), // POP
                0xA => try self.executeINT(instruction.operand), // INT
                0xB => try self.executeIN(), // IN
                0xC => try self.executeOUT(), // OUT
                else => return error.InvalidInstruction,
            }

            // Increment the instruction pointer (IP)
            self.R.set(15, ip + 1);
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
    var load_address: u32 = 0;

    var i: usize = 2; // Start from the second argument
    while (i < args.len) : (i += 1) {
    const arg = args[i];
    if (std.mem.eql(u8, arg, "-at") and i + 1 < args.len) {
        load_address = try std.fmt.parseInt(u32, args[i + 1], 10);
        i += 1; // Skip the next argument since it's the address
    }
}

    var cpu = try CPU.init(allocator, 4*1024);
    defer cpu.deinit(allocator);

    try cpu.loadProgram(program_path, load_address);
    try cpu.execute();
}
