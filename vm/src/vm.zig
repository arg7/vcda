const std = @import("std");

// Constants from ISA
const MAX_INSTRUCTION_SIZE = 8; // Max instruction size (8 bytes)
const PREFIX_OP2 = 0xC; // 2-byte instruction prefix
const PREFIX_OP4 = 0xD; // 4-byte instruction prefix
const PREFIX_OP8 = 0xE; // 8-byte instruction prefix

// VM structure
const VM = struct {
    memory: []u8, // Program memory
    registers: [256]u32, // 256 registers (using u64 for simplicity; adjust based on ISA needs)
    ip: usize, // Instruction Pointer (R255)
    running: bool, // VM state

    // Initialize VM
    fn init(allocator: std.mem.Allocator, program: []const u8) !VM {
        const memory = try allocator.alloc(u8, program.len);
        std.mem.copyForwards(u8, memory, program); // Fixed: Use copyForwards
        return VM{
            .memory = memory,
            .registers = [_]u32{0} ** 256,
            .ip = 0,
            .running = true,
        };
    }

    // Deinitialize VM
    fn deinit(self: *VM, allocator: std.mem.Allocator) void {
        allocator.free(self.memory);
    }

    // Fetch instruction into buffer
    fn fetch(self: *VM, buffer: *[MAX_INSTRUCTION_SIZE]u8) !usize {
        // Check if IP is within memory bounds
        if (self.ip >= self.memory.len) {
            return error.EndOfProgram;
        }

        // Read first byte to check for prefix
        const first_byte = self.memory[self.ip];
        var instruction_size: usize = 1; // Default to 1-byte instruction

        // Determine instruction size based on prefix
        switch (first_byte) {
            PREFIX_OP2 => instruction_size = 2,
            PREFIX_OP4 => instruction_size = 4,
            PREFIX_OP8 => instruction_size = 8,
            else => instruction_size = 1, // No prefix, 1-byte instruction
        }

        // Check if enough bytes remain in memory
        if (self.ip + instruction_size > self.memory.len) {
            return error.InvalidInstructionLength;
        }

        // Copy instruction (including prefix) into buffer
        @memset(buffer, 0); // Clear buffer for consistency
        std.mem.copyForwards(u8, buffer[0..instruction_size], self.memory[self.ip .. self.ip + instruction_size]); // Fixed: Use copyForwards

        // Advance IP
        self.ip += instruction_size;

        return instruction_size;
    }

    // Stub for decoding (to be implemented later)
    fn decodeAndExecute(self: *VM, buffer: []const u8, instruction_size: usize) !void {
        _ = self;
        //_ = buffer;
        //_ = instruction_size;
        std.debug.print("Decoding instruction of size {}: {x}\n", .{ instruction_size, buffer });
        // TODO: Implement decoding logic
    }

    // Main VM loop
    fn run(self: *VM) !void {
        var buffer: [MAX_INSTRUCTION_SIZE]u8 = undefined;

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

    // Example program (NOPs and a 2-byte instruction)
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
