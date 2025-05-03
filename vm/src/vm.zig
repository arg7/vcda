// vm.zig
const std = @import("std");
const fs = std.fs;
const defs = @import("definitions.zig");
const regs = @import("registers.zig");
const reg_logic = @import("register_logic.zig");

// Virtual Machine
pub const VM = struct {
    alloc: std.mem.Allocator,
    memory: []u8, // Program memory
    registers: regs.RegisterFile, // Register file (256 x u32)
    running: bool, // VM state
    _stdout: std.fs.File,
    _stderr: std.fs.File,
    _stdin: std.fs.File,
    _stdout_f: bool,
    _stderr_f: bool,
    _stdin_f: bool,

    // Initialize VM
    pub fn init(allocator: std.mem.Allocator, fname: []const u8) !VM {
        const p = try allocator.alloc(u8, 1024);

        const vm = VM{
            .alloc = allocator,
            .memory = p,
            .registers = regs.RegisterFile.init(),
            .running = true,
            ._stdout = std.io.getStdOut(),
            ._stderr = std.io.getStdErr(),
            ._stdin = std.io.getStdIn(),
            ._stdout_f = false,
            ._stderr_f = false,
            ._stdin_f = false,
        };
        if (fname.len != 0) {
            try loadProgram(fname, p);
        }
        return vm;
    }

    // Deinitialize VM
    pub fn deinit(self: *VM) void {
        self.alloc.free(self.memory);
        if (self._stderr_f) self._stderr.close();
        if (self._stdin_f) self._stdin.close();
        if (self._stdout_f) self._stdout.close();
    }

    pub fn loadProgram(program_path: []const u8, mem: []u8) !void {
        const file = try fs.cwd().openFile(program_path, .{});
        defer file.close();

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
            0xA => return error.NotImplemented, // IN
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
                    0xA => return error.NotImplemented, // IN
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

    // Set custom stdout by file name
    pub fn setStdOut(self: *VM, file_name: []const u8) !void {
        if (file_name.len == 0) {
            self._stdout = std.io.getStdOut();
            self._stdout_f = false;
        } else {
            self._stdout = try std.fs.cwd().createFile(file_name, .{ .truncate = true });
            self._stdout_f = true;
        }
    }

    // Set custom stderr by file name
    pub fn setStdErr(self: *VM, file_name: []const u8) !void {
        if (file_name.len == 0) {
            self._stderr = std.io.getStdErr();
            self._stderr_f = false;
        } else {
            self._stderr = try std.fs.cwd().createFile(file_name, .{ .truncate = true });
            self._stderr_f = true;
        }
    }

    // Set custom stdin by file name
    pub fn setStdIn(self: *VM, file_name: []const u8) !void {
        if (file_name.len == 0) {
            self._stdin = std.io.getStdIn();
            self._stdin_f = false;
        } else {
            self._stdin = try std.fs.cwd().createFile(file_name, .{});
            self._stdin_f = true;
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
