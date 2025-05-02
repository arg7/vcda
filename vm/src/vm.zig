// vm.zig
const std = @import("std");
const defs = @import("definitions.zig");
const regs = @import("registers.zig");
const reg_logic = @import("register_logic.zig");

// Virtual Machine
pub const VM = struct {
    memory: []u8, // Program memory
    registers: regs.RegisterFile, // Register file (256 x u32)
    running: bool, // VM state

    // Initialize VM
    pub fn init(allocator: std.mem.Allocator, program: []const u8) !VM {
        const memory = try allocator.alloc(u8, 1024);
        std.mem.copyForwards(u8, memory, program);
        return VM{
            .memory = memory,
            .registers = regs.RegisterFile.init(),
            .running = true,
        };
    }

    // Deinitialize VM
    pub fn deinit(self: *VM, allocator: std.mem.Allocator) void {
        allocator.free(self.memory);
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
            0xB => return error.NotImplemented, // OUT
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
                    0xB => return error.NotImplemented, // OUT
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

// Main function for testing
pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    // Example program (NOPs of varying sizes)
    const program = [_]u8{
        0x00, // NOP (1-byte)
        0xC, 0x0, // NOP (2-byte)
        0xD, 0x0, 0x0, 0x0, // NOP (4-byte)
        0xE, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, // NOP (8-byte)
    };

    // Initialize and run VM
    var vm = try VM.init(allocator, &program);
    defer vm.deinit(allocator);

    try vm.run();
}
