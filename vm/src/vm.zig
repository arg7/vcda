// vm.zig
const std = @import("std");
const fs = std.fs;
const defs = @import("definitions.zig");
const regs = @import("registers.zig");
const base = @import("vm_register_logic.zig");
const IOPipe = @import("iopipe.zig").IOPipe;
const io = @import("vm_io_logic.zig");
const br = @import("vm_branch_logic.zig");
const st = @import("vm_stack_logic.zig");
const al = @import("vm_alu_logic.zig");
const intr = @import("vm_int_logic.zig");

// Virtual Machine
pub const VM = struct {
    alloc: std.mem.Allocator,
    memory: []u8, // Program memory
    registers: regs.RegisterFile, // Register file (256 x u32)
    running: bool, // VM state
    _stdout: ?*std.fs.File,
    _stderr: ?*std.fs.File,
    _stdin_pipe: ?*IOPipe,

    // Initialize VM
    pub fn init(allocator: std.mem.Allocator, fin: ?*IOPipe, fout: ?*fs.File, ferr: ?*fs.File, mem_size: usize) !VM {
        const p = try allocator.alloc(u8, mem_size);

        const vm = VM{
            .alloc = allocator,
            .memory = p,
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
        //_ = self;
        //self.printId("deinit()");
        self.alloc.free(self.memory);
    }

    pub fn printId(self: *VM, msg: []const u8) void {
        std.debug.print("VM@{x} {s}\n", .{ @intFromPtr(self), msg });
    }

    pub fn loadProgram(self: *VM, program_path: []const u8, addr: u64) !u64 {

        var address: defs.PointerRegisterType = addr;
        const file = try fs.cwd().openFile(program_path, .{});
        defer file.close();

        const mem = self.memory;

        const ext = fs.path.extension(program_path);
        if (std.mem.eql(u8, ext, ".hex")) {
            var buf_reader = std.io.bufferedReader(file.reader());
            var reader = buf_reader.reader();
            var buf: [2]u8 = undefined; // Buffer for two hex characters (one byte)
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
            return address;
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
            return addr+bytes_read;
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
                    0x0 => try base.executeNOP(&self.registers, buffer[0..instruction_size]),
                    0x1 => try br.executeRET(self, buffer[0..instruction_size]),
                    0x2 => try br.executeIRET(self, buffer[0..instruction_size]),
                    0x3 => try base.executeINC(&self.registers, buffer[0..instruction_size]),
                    0x4 => try base.executeDEC(&self.registers, buffer[0..instruction_size]),
                    0x5 => try base.executeNOT(&self.registers, buffer[0..instruction_size]),
                    else => return error.InvalidOpcode,
                }
            },
            0x1 => try base.executeRS(&self.registers, buffer[0..instruction_size]),
            0x2 => try base.executeNS(&self.registers, buffer[0..instruction_size]),
            0x3 => try base.executeLI(&self.registers, buffer[0..instruction_size]),
            0x4 => try br.executeJMP(self, buffer[0..instruction_size]),
            0x5 => try br.executeCALL(self, buffer[0..instruction_size]),
            0x6 => try st.executePUSH(self, buffer[0..instruction_size]),
            0x7 => try st.executePOP(self, buffer[0..instruction_size]),
            0x8 => try al.executeALU(self, buffer[0..instruction_size]),
            0x9 => try intr.executeINT(self, buffer[0..instruction_size]),
            0xA => try io.executeIN(self, buffer[0..instruction_size]),
            0xB => try io.executeOUT(self, buffer[0..instruction_size]),
            0xF => return error.NotImplemented, // EXT
            0xC, 0xD, 0xE => {
                // Handle prefixed instructions
                if (instruction_size == 1) return error.InvalidInstructionLength;
                const opcode = second_nibble;
                switch (opcode) {
                    0x0 => try base.executeNOP(&self.registers, buffer[0..instruction_size]),
                    0x1 => try base.executeRS(&self.registers, buffer[0..instruction_size]),
                    0x2 => try base.executeNS(&self.registers, buffer[0..instruction_size]),
                    0x3 => try base.executeLI(&self.registers, buffer[0..instruction_size]),
                    0x4 => try br.executeJMP(self, buffer[0..instruction_size]),
                    0x5 => try br.executeCALL(self, buffer[0..instruction_size]),
                    0x6 => try st.executePUSH(self, buffer[0..instruction_size]),
                    0x7 => try st.executePOP(self, buffer[0..instruction_size]),
                    0x8 => try al.executeALU(self, buffer[0..instruction_size]),
                    0x9 => try intr.executeINT(self, buffer[0..instruction_size]),
                    0xA => try io.executeIN(self, buffer[0..instruction_size]),
                    0xB => try io.executeOUT(self, buffer[0..instruction_size]),
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
};


pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    // Default values
    var fn_in: ?[]const u8 = null;
    var fn_out: ?[]const u8 = null;
    var fn_err: ?[]const u8 = null;
    var mem_size: u64 = 0xFFFF;
    var load_files = std.ArrayList(struct { path: []const u8, addr: ?u64 }).init(allocator);
    defer load_files.deinit();

    // Parse command-line arguments
    var args = try std.process.argsWithAllocator(allocator);
    defer args.deinit();

    // Skip the program name
    _ = args.next();

    while (args.next()) |arg| {
        if (std.mem.eql(u8, arg, "-h")) {
            try printHelp();
            return;
        } else if (std.mem.eql(u8, arg, "-i")) {
            fn_in = args.next() orelse return error.MissingArgument;
        } else if (std.mem.eql(u8, arg, "-o")) {
            fn_out = args.next() orelse return error.MissingArgument;
        } else if (std.mem.eql(u8, arg, "-e")) {
            fn_err = args.next() orelse return error.MissingArgument;
        } else if (std.mem.eql(u8, arg, "-m")) {
            const mem_str = args.next() orelse return error.MissingArgument;
            if (std.mem.startsWith(u8, mem_str, "0x")) {
                mem_size = try std.fmt.parseInt(u64, mem_str[2..], 16);
            } else {
                mem_size = try std.fmt.parseInt(u64, mem_str, 10);
            }
        } else if (std.mem.eql(u8, arg, "-l")) {
            const load_arg = args.next() orelse return error.MissingArgument;
            // Parse load_arg: either "addr:path" or just "path"
            var addr: ?u64 = null;
            var path: []const u8 = undefined;
            if (std.mem.indexOfScalar(u8, load_arg, ':')) |colon_idx| {
                // Format: addr:path
                const addr_str = load_arg[0..colon_idx];
                path = load_arg[colon_idx + 1 ..];
                if (!std.mem.startsWith(u8, addr_str, "0x")) return error.InvalidAddress;
                addr = try std.fmt.parseInt(u64, addr_str[2..], 16);
            } else {
                // Just path
                path = load_arg;
            }
            try load_files.append(.{ .path = path, .addr = addr });
        } else {
            std.debug.print("Unknown argument: {s}\n", .{arg});
            try printHelp();
            return error.InvalidArgument;
        }
    }

    // Set up stdio
    var fin = if (fn_in) |path|
        try std.fs.cwd().openFile(path, .{})
    else
        std.io.getStdIn();
    defer if (fn_in != null) fin.close();

    var fout = if (fn_out) |path|
        try std.fs.cwd().createFile(path, .{ .truncate = false })
    else
        std.io.getStdOut();
    defer if (fn_out != null) fout.close();

    var ferr = if (fn_err) |path|
        try std.fs.cwd().createFile(path, .{ .truncate = false })
    else
        std.io.getStdErr();
    defer if (fn_err != null) ferr.close();

    var iop = try IOPipe.init(allocator, fin.reader().any(), 16);
    defer iop.deinit();

    // Initialize VM
    var vm = try VM.init(allocator, &iop, &fout, &ferr, mem_size);
    defer vm.deinit();

    // Load programs
    var next_addr: u64 = 0;
    for (load_files.items) |load| {
        const addr = load.addr orelse next_addr;
        next_addr = try vm.loadProgram(load.path, addr);
    }

    try vm.run();
}

fn printHelp() !void {
    const help =
        \\Usage: vm [options]
        \\Options:
        \\  -i <file>        : Input file (default: stdin)
        \\  -o <file>        : Output file (default: stdout)
        \\  -e <file>        : Error file (default: stderr)
        \\  -l <addr>:<file> : Load program file at address (0x hex)
        \\  -l <file>        : Load at 0x0 (first) or next available address
        \\  -m <size>        : Memory size in bytes (decimal or 0x hex, default: 0xFFFF)
        \\  -h               : Print this help
        \\
    ;
    try std.io.getStdOut().writeAll(help);
}