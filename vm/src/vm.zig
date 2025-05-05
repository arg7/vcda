// vm.zig
const std = @import("std");
const fs = std.fs;
const defs = @import("definitions.zig");
const regs = @import("registers.zig");
const reg_logic = @import("register_logic.zig");
const IOPipe = @import("iopipe.zig").IOPipe;

// Virtual Machine
pub const VM = struct {
    alloc: std.mem.Allocator,
    memory: [1024]u8, // Program memory
    registers: regs.RegisterFile, // Register file (256 x u32)
    running: bool, // VM state
    _stdout: ?*std.fs.File,
    _stderr: ?*std.fs.File,
    _stdin_pipe: ?*IOPipe,

    // Initialize VM
    pub fn init(allocator: std.mem.Allocator, fin: ?*IOPipe, fout: ?*fs.File, ferr: ?*fs.File) !VM {
        //const p = try allocator.alloc(u8, 1024);
        //defer allocator.free(p);

        const vm = VM{
            .alloc = allocator,
            .memory = [_]u8{0} ** 1024,
            .registers = regs.RegisterFile.init(),
            .running = true,
            ._stdout = fout,
            ._stderr = ferr,
            ._stdin_pipe = fin,
        };
        //vm.printId("init()");
        return vm;
    }

    // Deinitialize VM
    pub fn deinit(self: *VM) void {
        _ = self;
        //self.printId("deinit()");
        //self.alloc.free(self.memory);
    }

    pub fn printId(self: *VM, msg: []const u8) void {
        std.debug.print("VM@{x} {s}\n", .{ @intFromPtr(self), msg });
    }

    pub fn loadProgram(self: *VM, program_path: []const u8) !void {
        const file = try fs.cwd().openFile(program_path, .{});
        defer file.close();

        const mem = self.memory;

        const ext = fs.path.extension(program_path);
        if (std.mem.eql(u8, ext, ".hex")) {
            var buf_reader = std.io.bufferedReader(file.reader());
            var reader = buf_reader.reader();
            var buf: [2]u8 = undefined; // Buffer for two hex characters (one byte)
            var address: defs.PointerRegisterType = 0;
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
                        if (address >= mem.len) {
                            return error.ProgramTooLarge;
                        }
                        mem[address] = parsed_byte;
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

            if (file_size > mem.len) {
                return error.ProgramTooLarge;
            }

            const bytes_read = try file.readAll(mem[0..]);
            if (bytes_read != file_size) {
                std.log.warn("Failed to read the entire binary file.  Expected {} bytes, read {}.", .{ file_size, bytes_read });
                return error.IncompleteRead; // Or handle differently, e.g., continue with partial load
            }
        }
    }

    // Fetch instruction into buffer
    pub fn fetch(self: *VM, buffer: *[defs.MAX_INSTRUCTION_SIZE]u8) !usize {
        // Read IP from R255
        const ip = self.registers.readIP();
        if (ip >= self.memory.len) {
            return error.EndOfProgram;
        }

        // Read first byte to check for prefix
        const first_byte = self.memory[ip];
        var instruction_size: u8 = 1; // Default to 1-byte instruction

        // Determine instruction size based on prefix
        switch (first_byte) {
            defs.PREFIX_OP2 => instruction_size = 2,
            defs.PREFIX_OP4 => instruction_size = 4,
            defs.PREFIX_OP8 => instruction_size = 8,
            else => instruction_size = 1,
        }

        // Check if enough bytes remain in memory
        if (ip + instruction_size > self.memory.len) {
            return error.InvalidInstructionLength;
        }

        // Copy instruction (including prefix) into buffer
        @memset(buffer, 0); // Clear buffer
        std.mem.copyForwards(u8, buffer[0..instruction_size], self.memory[ip .. ip + instruction_size]);

        // Advance IP (R255)
        self.registers.writeIP(ip + instruction_size);

        return instruction_size;
    }

    // Decode and execute instruction
    pub fn decodeAndExecute(self: *VM, buffer: []const u8, instruction_size: usize) !void {
        if (instruction_size == 0 or buffer.len < instruction_size) {
            return error.InvalidInstructionLength;
        }

        const first_nibble = buffer[0] >> 4;
        const second_nibble = buffer[0] & 0x0F;

        switch (first_nibble) {
            0x0 => {
                switch (second_nibble) {
                    0x0 => try reg_logic.executeNOP(&self.registers, buffer[0..instruction_size]),
                    0x1 => try reg_logic.executeRET(self, buffer[0..instruction_size]),
                    0x2 => try reg_logic.executeIRET(self, buffer[0..instruction_size]),
                    0x3 => try reg_logic.executeINC(&self.registers, buffer[0..instruction_size]),
                    0x4 => try reg_logic.executeDEC(&self.registers, buffer[0..instruction_size]),
                    0x5 => try reg_logic.executeNOT(&self.registers, buffer[0..instruction_size]),
                    else => return error.InvalidOpcode,
                }
            },
            0x1 => try reg_logic.executeRS(&self.registers, buffer[0..instruction_size]),
            0x2 => try reg_logic.executeNS(&self.registers, buffer[0..instruction_size]),
            0x3 => try reg_logic.executeLI(&self.registers, buffer[0..instruction_size]),
            0x4 => try reg_logic.executeJMP(self, buffer[0..instruction_size]),
            0x5 => try reg_logic.executeCALL(self, buffer[0..instruction_size]),
            0x6 => try reg_logic.executePUSH(self, buffer[0..instruction_size]),
            0x7 => try reg_logic.executePOP(self, buffer[0..instruction_size]),
            0x8 => try reg_logic.executeALU(self, buffer[0..instruction_size]),
            0x9 => return error.NotImplemented, // INT
            0xA => try reg_logic.executeIN(self, buffer[0..instruction_size]),
            0xB => try reg_logic.executeOUT(self, buffer[0..instruction_size]),
            0xF => return error.NotImplemented, // EXT
            0xC, 0xD, 0xE => {
                // Handle prefixed instructions
                if (instruction_size == 1) return error.InvalidInstructionLength;
                const opcode = second_nibble;
                switch (opcode) {
                    0x0 => try reg_logic.executeNOP(&self.registers, buffer[0..instruction_size]),
                    0x1 => try reg_logic.executeRS(&self.registers, buffer[0..instruction_size]),
                    0x2 => try reg_logic.executeNS(&self.registers, buffer[0..instruction_size]),
                    0x3 => try reg_logic.executeLI(&self.registers, buffer[0..instruction_size]),
                    0x4 => try reg_logic.executeJMP(self, buffer[0..instruction_size]),
                    0x5 => try reg_logic.executeCALL(self, buffer[0..instruction_size]),
                    0x6 => try reg_logic.executePUSH(self, buffer[0..instruction_size]),
                    0x7 => try reg_logic.executePOP(self, buffer[0..instruction_size]),
                    0x8 => try reg_logic.executeALU(self, buffer[0..instruction_size]),
                    0x9 => return error.NotImplemented, // INT
                    0xA => try reg_logic.executeIN(self, buffer[0..instruction_size]),
                    0xB => try reg_logic.executeOUT(self, buffer[0..instruction_size]),
                    0xF => return error.NotImplemented, // EXT
                    else => return error.InvalidOpcode,
                }
            },
            else => return error.InvalidOpcode,
        }
    }
    // Main VM loop
    pub fn run(self: *VM) !void {
        var buffer: [defs.MAX_INSTRUCTION_SIZE]u8 = undefined;

        while (self.running) {
            // Fetch instruction
            const instruction_size = self.fetch(&buffer) catch |err| {
                std.debug.print("Fetch error: {}\n", .{err});
                self.running = false;
                break;
            };

            // Decode and execute
            try self.decodeAndExecute(buffer[0..instruction_size], instruction_size);
        }
    }

    pub fn parseInput(self: *VM, adt: defs.ADT, fmt: defs.OUT_FMT) !defs.RegisterType {
        var input: []u8 = &[_]u8{};
        defer self.alloc.free(input);

        var iop = self._stdin_pipe orelse return error.FInMissing;

        // Read until whitespace or EOF for formatted input
        if (fmt.fmt != .raw) {
            var temp = std.ArrayList(u8).init(self.alloc);
            defer temp.deinit();

            while (true) {
                const byte = iop.readByte() catch |err| {
                    std.debug.print("error: {any}\n", .{err});
                    break;
                };
                if (std.ascii.isWhitespace(byte)) {
                    try iop.pushBack(byte); // Push back whitespace
                    break;
                } else {
                    try temp.append(byte);
                }
            }

            if (temp.items.len == 0) return error.EndOfStream; // No input
            input = try temp.toOwnedSlice();
        }

        switch (fmt.fmt) {
            .raw => {
                const byte_size = (adt.bits() + 7) >> 3;
                var raw_buf: [8]u8 = undefined;
                const bytes_read = try iop.read(raw_buf[0..byte_size]);
                if (bytes_read < byte_size) return error.EndOfStream; // Insufficient data

                return switch (byte_size) {
                    1 => raw_buf[0],
                    2 => std.mem.readInt(u16, raw_buf[0..2], .little),
                    4 => std.mem.readInt(u32, raw_buf[0..4], .little),
                    8 => std.mem.readInt(u64, raw_buf[0..8], .little),
                    else => return error.InvalidDataType,
                };
            },
            .hex => {
                const str = input;
                const value = std.fmt.parseInt(defs.RegisterType, str, 16) catch |err| {
                    for (str) |b| try iop.pushBack(b);
                    return err;
                };
                return try signExtend(value, adt);
            },
            .dec => {
                const str = input;
                if (adt.signed()) {
                    const value = std.fmt.parseInt(defs.RegisterSignedType, str, 10) catch |err| {
                        for (str) |b| try iop.pushBack(b);
                        return err;
                    };
                    return try signExtend(@bitCast(value), adt);
                } else {
                    const value = std.fmt.parseInt(defs.RegisterType, str, 10) catch |err| {
                        for (str) |b| try iop.pushBack(b);
                        return err;
                    };
                    return try signExtend(value, adt);
                }
            },
            .bin => {
                const str = input;
                const value = std.fmt.parseInt(defs.RegisterType, str, 2) catch |err| {
                    for (str) |b| try iop.pushBack(b);
                    return err;
                };
                return try signExtend(value, adt);
            },
            .fp0, .fp2, .fp4 => {
                const str = input;
                const value = std.fmt.parseFloat(f64, str) catch |err| {
                    for (str) |b| try iop.pushBack(b);
                    return err;
                };
                return switch (adt) {
                    .f16 => blk: {
                        const f16_val: f16 = @floatCast(value);
                        const r: u16 = @bitCast(f16_val);
                        break :blk r;
                    },
                    .f32 => blk: {
                        const f32_val: f32 = @floatCast(value);
                        const r: u32 = @bitCast(f32_val);
                        break :blk r;
                    },
                    .f64 => @bitCast(value),
                    else => return error.InvalidDataType,
                };
            },
        }
    }

    pub fn signExtend(value: defs.RegisterType, adt: defs.ADT) !defs.RegisterType {
        const bs = adt.bits(); // 4 bits
        const one: defs.RegisterType = 1;
        const lower_mask = (one << @truncate(bs)) - one; // e.g., 0xF
        var mask = ~lower_mask; // e.g., 0xFFFFFFF0 for WS=32
        if (mask == 0) mask = ~mask;
        const sign_bit = (value & (one << @truncate(bs - one))) != 0;
        if (adt.signed() and sign_bit) {
            return value | mask;
        } else {
            return value & lower_mask;
        }
    }
};

// Main function for testing
pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    // Initialize and run VM
    var vm = try VM.init(allocator, "test.hex");
    defer vm.deinit(allocator);

    try vm.run();
}
