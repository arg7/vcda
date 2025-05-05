const std = @import("std");
const defs = @import("definitions.zig");
const regs = @import("registers.zig");
const vm_mod = @import("vm.zig");
const alu_mod = @import("alu.zig");
const builtin = @import("builtin");
const er = @import("errorreader.zig");
const IOPipe = @import("iopipe.zig").IOPipe;

// Execute INT instruction
pub fn executeINT(vm: *vm_mod.VM, buffer: []const u8) !void {
    // Determine instruction form and extract interrupt number
    var int_number: u32 = 0;
    var instruction_valid = false;

    // Parse instruction based on length
    switch (buffer.len) {
        1 => {
            // 1-byte: 0xB|ch (ch is u4)
            if ((buffer[0] >> 4) == 0x9) {
                int_number = buffer[0] & 0x0F; // Extract u4
                instruction_valid = true;
            }
        },
        2 => {
            // 2-byte: 0xCB, ch (ch is u8)
            if (buffer[0] == ((defs.PREFIX_OP2 << 4) | 0x9)) {
                int_number = buffer[1]; // u8
                instruction_valid = true;
            }
        },
        else => return error.InvalidInstructionLength,
    }

    if (!instruction_valid) {
        return error.InvalidOpcode;
    }

    // Read ITP (R252)
    const itp = vm.registers.read(defs.R_ITP);
    const entry_size = @sizeOf(defs.PointerRegisterType);
    const table_offset = int_number * entry_size;

    // Calculate handler address: ITP + int_number * sizeof(PointerRegisterType)
    const handler_addr = itp + table_offset;

    // Validate memory access
    if (handler_addr + entry_size > vm.memory.len) {
        return error.InvalidInterruptTableAddress;
    }

    // Read handler address from ITP
    const handler = switch (entry_size) {
        2 => std.mem.readInt(u16, vm.memory[handler_addr..][0..2], .little),
        4 => std.mem.readInt(u32, vm.memory[handler_addr..][0..4], .little),
        8 => std.mem.readInt(u64, vm.memory[handler_addr..][0..8], .little),
        else => return error.UnsupportedWordSize,
    };

    // Validate handler address
    if (handler >= vm.memory.len) {
        return error.InvalidHandlerAddress;
    }

    // Save current IP to stack (mimic PUSH behavior)
    const sp = vm.registers.readSP();
    if (sp < entry_size) {
        return error.StackOverflow;
    }

    // Decrement SP and write IP to memory
    const current_ip = vm.registers.readIP();
    const new_sp = sp - entry_size;
    switch (entry_size) {
        2 => std.mem.writeInt(u16, vm.memory[new_sp..][0..2], @truncate(current_ip), .little),
        4 => std.mem.writeInt(u32, vm.memory[new_sp..][0..4], @truncate(current_ip), .little),
        8 => std.mem.writeInt(u64, vm.memory[new_sp..][0..8], @truncate(current_ip), .little),
        else => return error.UnsupportedWordSize,
    }
    vm.registers.writeSP(new_sp);

    // Jump to handler by updating IP
    vm.registers.writeIP(handler);
}

test "INT instruction" {
    const allocator = std.testing.allocator;

    // Initialize VM
    var vm = try vm_mod.VM.init(allocator, null, null, null, 0xFFFF);
    defer vm.deinit();

    var reg_file = &vm.registers;
    const ip_size = @sizeOf(defs.PointerRegisterType); // Size of IP in bytes

    // Set SP to top of memory
    const initial_sp = @as(defs.PointerRegisterType, @truncate(vm.memory.len));
    reg_file.writeSP(initial_sp);

    // Test 1: 1-byte INT (0x9|int, int=0x1, u4)
    {
        // Setup: R252 = 0x100, ITP[1] = 0x200
        reg_file.write(defs.R_ITP, 0x100); // R252
        const int_number: u8 = 0x1;
        const handler_addr: defs.PointerRegisterType = 0x200;
        const table_offset: u32 = int_number * ip_size;
        const addr: u32 = 0x100 + table_offset;
        std.mem.writeInt(defs.PointerRegisterType, vm.memory[addr..][0..ip_size], handler_addr, .little);

        // Set initial IP
        const initial_ip: defs.PointerRegisterType = 0x50;
        reg_file.writeIP(initial_ip);

        // Execute INT 0x1 (0x91)
        const int_1byte = [_]u8{0x91};
        try executeINT(&vm, &int_1byte);

        // Verify
        try std.testing.expectEqual(handler_addr, reg_file.readIP()); // IP = 0x200
        try std.testing.expectEqual(initial_sp - ip_size, reg_file.readSP()); // SP decremented
        try std.testing.expectEqual(
            initial_ip,
            std.mem.readInt(defs.PointerRegisterType, vm.memory[initial_sp - ip_size ..][0..ip_size], .little),
        ); // Stack contains original IP
    }

    // Test 2: 1-byte INT (0x9|int, int=0xF, u4)
    {
        // Setup: R252 = 0x100, ITP[15] = 0x300
        reg_file.write(defs.R_ITP, 0x100);
        const int_number: u8 = 0xF;
        const handler_addr: defs.PointerRegisterType = 0x300;
        const table_offset: u32 = int_number * ip_size;
        std.mem.writeInt(defs.PointerRegisterType, vm.memory[0x100 + table_offset ..][0..ip_size], handler_addr, .little);

        // Reset SP and set initial IP
        reg_file.writeSP(initial_sp);
        const initial_ip: defs.PointerRegisterType = 0x60;
        reg_file.writeIP(initial_ip);

        // Execute INT 0xF (0x9F)
        const int_1byte = [_]u8{0x9F};
        try executeINT(&vm, &int_1byte);

        // Verify
        try std.testing.expectEqual(handler_addr, reg_file.readIP()); // IP = 0x300
        try std.testing.expectEqual(initial_sp - ip_size, reg_file.readSP());
        try std.testing.expectEqual(
            initial_ip,
            std.mem.readInt(defs.PointerRegisterType, vm.memory[initial_sp - ip_size ..][0..ip_size], .little),
        );
    }

    // Test 3: 2-byte INT (0xC9, int=0xFF, u8)
    {
        // Setup: R252 = 0x100, ITP[255] = 0x400
        reg_file.write(defs.R_ITP, 0x100);
        const int_number: u32 = 0x44;
        const handler_addr: defs.PointerRegisterType = 0x400;
        const table_offset: u32 = int_number * ip_size;
        std.mem.writeInt(defs.PointerRegisterType, vm.memory[0x100 + table_offset ..][0..ip_size], handler_addr, .little);

        // Reset SP and set initial IP
        reg_file.writeSP(initial_sp);
        const initial_ip: defs.PointerRegisterType = 0x70;
        reg_file.writeIP(initial_ip);

        // Execute INT 0x44 (0xC9, 0x44)
        const int_2byte = [_]u8{ (defs.PREFIX_OP2 << 4) | 0x9, 0x44 };
        try executeINT(&vm, &int_2byte);

        // Verify
        try std.testing.expectEqual(handler_addr, reg_file.readIP()); // IP = 0x400
        try std.testing.expectEqual(initial_sp - ip_size, reg_file.readSP());
        try std.testing.expectEqual(
            initial_ip,
            std.mem.readInt(defs.PointerRegisterType, vm.memory[initial_sp - ip_size ..][0..ip_size], .little),
        );
    }

    // Test 4: Invalid opcode (1-byte)
    {
        const invalid_1byte = [_]u8{0x20}; // Wrong opcode
        try std.testing.expectError(error.InvalidOpcode, executeINT(&vm, &invalid_1byte));
    }

    // Test 5: Invalid opcode (2-byte)
    {
        const invalid_2byte = [_]u8{ (defs.PREFIX_OP2 << 4) | 0xA, 0x01 }; // Wrong opcode
        try std.testing.expectError(error.InvalidOpcode, executeINT(&vm, &invalid_2byte));
    }

    // Test 6: Invalid instruction length
    {
        const invalid_length = [_]u8{ 0x91, 0x00, 0x00 }; // 3-byte
        try std.testing.expectError(error.InvalidInstructionLength, executeINT(&vm, &invalid_length));
    }

    // Test 7: Invalid ITP address
    {
        // Set R252 so ITP[1] is out of bounds
        reg_file.write(defs.R_ITP, vm.memory.len - ip_size + 1); // ITP[1] exceeds memory
        reg_file.writeSP(initial_sp);
        reg_file.writeIP(0x50);
        const int_1byte = [_]u8{0x91};
        try std.testing.expectError(error.InvalidInterruptTableAddress, executeINT(&vm, &int_1byte));
    }

    // Test 8: Invalid handler address
    {
        // Set ITP[1] to an invalid handler address
        reg_file.write(defs.R_ITP, 0x100);
        const int_number: u32 = 0x1;
        const handler_addr: defs.PointerRegisterType = vm.memory.len; // Out of bounds
        std.mem.writeInt(defs.PointerRegisterType, vm.memory[0x100 + int_number * ip_size ..][0..ip_size], handler_addr, .little);

        reg_file.writeSP(initial_sp);
        reg_file.writeIP(0x50);
        const int_1byte = [_]u8{0x91};
        try std.testing.expectError(error.InvalidHandlerAddress, executeINT(&vm, &int_1byte));
    }

    // Test 9: Stack overflow
    {
        // Set SP too low
        reg_file.write(defs.R_ITP, 0x100);
        const handler_addr: defs.PointerRegisterType = 0x200;
        std.mem.writeInt(defs.PointerRegisterType, vm.memory[0x100 + ip_size ..][0..ip_size], handler_addr, .little);

        reg_file.writeSP(ip_size - 1); // Not enough space
        reg_file.writeIP(0x50);
        const int_1byte = [_]u8{0x91};
        try std.testing.expectError(error.StackOverflow, executeINT(&vm, &int_1byte));
    }
}
