const std = @import("std");
const defs = @import("definitions.zig");
const regs = @import("registers.zig");
const vm_mod = @import("vm.zig");
const alu_mod = @import("alu.zig");
const builtin = @import("builtin");
const er = @import("errorreader.zig");
const IOPipe = @import("iopipe.zig").IOPipe;

// Execute PUSH instruction
pub fn executePUSH(vm: *vm_mod.VM, buffer: []const u8) !void {
    const reg_file = &vm.registers;
    //var cfg = reg_file.readALU_IO_CFG();
    const mode = reg_file.readALU_MODE_CFG();
    var reg_index: u8 = 0;
    var count: u8 = 1; // Default to pushing 1 register

    switch (buffer.len) {
        1 => {
            // 1-byte: 0x6 reg (reg is u4)
            if ((buffer[0] >> 4) != 0x6) return error.InvalidOpcode;
            reg_index = buffer[0] & 0x0F; // Extract u4
        },
        2 => {
            // 2-byte: 0xC 0x6 reg (reg is u8)
            if ((buffer[0] >> 4) != defs.PREFIX_OP2 or (buffer[0] & 0x0F) != 0x6) return error.InvalidOpcode;
            reg_index = buffer[1]; // u8
        },
        4 => {
            // 4-byte: 0xD 0x6 reg cnt
            if ((buffer[0] >> 4) != defs.PREFIX_OP4 or (buffer[0] & 0x0F) != 0x6) return error.InvalidOpcode;
            reg_index = buffer[1]; // u8
            count = buffer[2]; // u8
        },
        else => return error.InvalidInstructionLength,
    }

    // Get stack pointer
    var sp = reg_file.readSP();
    const is_special = reg_index == defs.R_ALU_IO_CFG or reg_index == defs.R_ALU_MODE_CFG or
        reg_index == defs.R_ALU_VR_STRIDES or reg_index == defs.R_BRANCH_CTRL or
        reg_index == defs.R_BP or reg_index == defs.R_SP or reg_index == defs.R_IP;

    // Process each register in range
    for (0..count) |i| {
        const current_reg = reg_index + @as(u8, @truncate(i));
        if (current_reg >= defs.REGISTER_COUNT) return error.InvalidRegisterIndex;

        const reg_bits = regs.reg_size_bits(current_reg);
        const byte_size: u8 = if (is_special) (reg_bits + 7) >> 3 else switch (mode.adt) {
            .u1, .u4, .i4, .u8, .i8, .fp4, .fp8 => 1,
            .u16, .i16, .f16 => 2,
            .u32, .i32, .f32 => 4,
            .u64, .i64, .f64 => 8,
            //else => return error.UnsupportedADT,
        };

        // Check memory bounds
        if (sp < byte_size or sp > vm.memory.len) return error.StackOverflow;

        // Read register value
        const value = reg_file.read(current_reg);

        // Write to memory at SP (little-endian)
        sp -= byte_size;
        switch (byte_size) {
            1 => vm.memory[sp] = @truncate(value),
            2 => if (defs.WS >= 16) {
                const mem_slice: *[2]u8 = @ptrCast(vm.memory[sp .. sp + 2].ptr);
                std.mem.writeInt(u16, mem_slice, @truncate(value), .little);
            } else return error.InvalidByteSize,
            4 => if (defs.WS >= 32) {
                const mem_slice: *[4]u8 = @ptrCast(vm.memory[sp .. sp + 2].ptr);
                std.mem.writeInt(u32, mem_slice, @truncate(value), .little);
            } else return error.InvalidByteSize,
            8 => if (defs.WS >= 64) {
                const mem_slice: *[8]u8 = @ptrCast(vm.memory[sp .. sp + 2].ptr);
                std.mem.writeInt(u64, mem_slice, @truncate(value), .little);
            } else return error.InvalidByteSize,
            else => return error.InvalidByteSize,
        }
    }

    // Update SP
    reg_file.writeSP(sp);
}

pub fn executePOP(vm: *vm_mod.VM, buffer: []const u8) !void {
    const reg_file = &vm.registers;
    const mode = reg_file.readALU_MODE_CFG();
    var reg_index: u8 = 0;
    var count: u8 = 1; // Default to popping 1 register
    var ofs: u8 = 0; // Offset for 4-byte form

    switch (buffer.len) {
        1 => {
            if ((buffer[0] >> 4) != 0x7) return error.InvalidOpcode;
            reg_index = buffer[0] & 0x0F; // Extract u4
        },
        2 => {
            if ((buffer[0] >> 4) != defs.PREFIX_OP2 or (buffer[0] & 0x0F) != 0x7) return error.InvalidOpcode;
            reg_index = buffer[1]; // u8
        },
        4 => {
            if ((buffer[0] >> 4) != defs.PREFIX_OP4 or (buffer[0] & 0x0F) != 0x7) return error.InvalidOpcode;
            reg_index = buffer[1]; // u8
            count = buffer[2]; // u8
            ofs = buffer[3]; // u8
        },
        else => return error.InvalidInstructionLength,
    }

    var sp = reg_file.readSP();
    const is_special = reg_index == defs.R_ALU_IO_CFG or reg_index == defs.R_ALU_MODE_CFG or
        reg_index == defs.R_ALU_VR_STRIDES or reg_index == defs.R_BRANCH_CTRL or
        reg_index == defs.R_BP or reg_index == defs.R_SP or reg_index == defs.R_IP;

    for (0..count) |i| {
        const current_reg = reg_index + @as(u8, @truncate(i));
        if (current_reg >= defs.REGISTER_COUNT) return error.InvalidRegisterIndex;

        const reg_bits = regs.reg_size_bits(current_reg);
        const byte_size: u8 = if (is_special) (reg_bits + 7) >> 3 else switch (mode.adt) {
            .u1, .u4, .i4, .u8, .i8, .fp4, .fp8 => 1,
            .u16, .i16, .f16 => 2,
            .u32, .i32, .f32 => 4,
            .u64, .i64, .f64 => 8,
        };

        const read_addr = if (ofs > 0) sp - (ofs * byte_size) else sp;
        if (read_addr < byte_size or read_addr + byte_size > vm.memory.len) return error.StackUnderflow;

        var value: defs.SpecialRegisterType = 0;
        switch (byte_size) {
            1 => value = vm.memory[read_addr],
            2 => if (defs.WS >= 16) {
                value = std.mem.readInt(u16, @ptrCast(vm.memory[read_addr .. read_addr + 2].ptr), .little);
            } else return error.InvalidByteSize,
            4 => if (defs.WS >= 32) {
                value = std.mem.readInt(u32, @ptrCast(vm.memory[read_addr .. read_addr + 4].ptr), .little);
            } else return error.InvalidByteSize,
            8 => if (defs.WS >= 64) {
                value = std.mem.readInt(u64, @ptrCast(vm.memory[read_addr .. read_addr + 8].ptr), .little);
            } else return error.InvalidByteSize,
            else => return error.InvalidByteSize,
        }

        reg_file.write(current_reg, value);

        if (ofs == 0) sp += byte_size;
    }

    reg_file.writeSP(sp);
}

// Unit Tests
test "PUSH and POP instructions" {
    //const log = std.log.scoped(.vm);

    const allocator = std.testing.allocator;

    var vm = try vm_mod.VM.init(allocator, null, null, null, 0xFFFF);
    defer _ = vm.deinit();

    //const a = &vm;
    //std.debug.print("vm = {}\n", .{a});

    var reg_file = &vm.registers;
    var mode = reg_file.readALU_MODE_CFG();
    //var cfg = reg_file.readALU_IO_CFG();

    // Set SP to top of memory
    reg_file.writeSP(@as(defs.RegisterType, @truncate(vm.memory.len)));

    // Test 1-byte PUSH/POP: reg 1, u8
    mode.adt = defs.ADT.u8;
    reg_file.writeALU_MODE_CFG(mode);
    reg_file.write(1, 0xAB);
    const push_1byte = [_]u8{0x61}; // PUSH R1
    try executePUSH(&vm, &push_1byte);
    try std.testing.expectEqual(vm.memory.len - 1, reg_file.readSP());
    try std.testing.expectEqual(0xAB, vm.memory[vm.memory.len - 1]);

    const pop_1byte = [_]u8{0x71}; // POP R1
    reg_file.write(1, 0);
    try executePOP(&vm, &pop_1byte);
    try std.testing.expectEqual(vm.memory.len, reg_file.readSP());
    try std.testing.expectEqual(0xAB, reg_file.read(1));

    // Test 2-byte PUSH/POP: reg 2, u16
    mode.adt = defs.ADT.u16;
    reg_file.writeALU_MODE_CFG(mode);
    reg_file.write(2, 0x1234);
    const push_2byte = [_]u8{ (defs.PREFIX_OP2 << 4) | 0x6, 0x02 }; // PUSH R2
    try executePUSH(&vm, &push_2byte);
    try std.testing.expectEqual(vm.memory.len - 2, reg_file.readSP());
    try std.testing.expectEqual(0x34, vm.memory[vm.memory.len - 2]);
    try std.testing.expectEqual(0x12, vm.memory[vm.memory.len - 1]);

    const pop_2byte = [_]u8{ (defs.PREFIX_OP2 << 4) | 0x7, 0x02 }; // POP R2
    reg_file.write(2, 0);
    try executePOP(&vm, &pop_2byte);
    try std.testing.expectEqual(vm.memory.len, reg_file.readSP());
    try std.testing.expectEqual(0x1234, reg_file.read(2));

    // Test 4-byte PUSH/POP: range R3-R4, u8
    mode.adt = defs.ADT.u8;
    reg_file.writeALU_MODE_CFG(mode);
    reg_file.write(3, 0xCD);
    reg_file.write(4, 0xEF);
    const push_4byte = [_]u8{ (defs.PREFIX_OP4 << 4) | 0x6, 0x03, 0x02, 0x00 }; // PUSH R3, 2 regs
    try executePUSH(&vm, &push_4byte);
    try std.testing.expectEqual(vm.memory.len - 2, reg_file.readSP());
    try std.testing.expectEqual(0xEF, vm.memory[vm.memory.len - 2]);
    try std.testing.expectEqual(0xCD, vm.memory[vm.memory.len - 1]);

    const pop_4byte = [_]u8{ (defs.PREFIX_OP4 << 4) | 0x7, 0x03, 0x02, 0x00 }; // POP R3, 2 regs
    reg_file.write(3, 0);
    reg_file.write(4, 0);
    try executePOP(&vm, &pop_4byte);
    try std.testing.expectEqual(vm.memory.len, reg_file.readSP());
    try std.testing.expectEqual(0xCD, reg_file.read(4));
    try std.testing.expectEqual(0xEF, reg_file.read(3));

    // Test special register PUSH/POP: R_IP (255)
    reg_file.write(defs.R_IP, 0x1234);
    const push_special = [_]u8{ (defs.PREFIX_OP2 << 4) | 0x6, 0xFF }; // PUSH R255
    try executePUSH(&vm, &push_special);
    try std.testing.expectEqual(vm.memory.len - @sizeOf(defs.PointerRegisterType), reg_file.readSP());
    try std.testing.expectEqual(0x1234, std.mem.readInt(defs.PointerRegisterType, @ptrCast(vm.memory[vm.memory.len - @sizeOf(defs.PointerRegisterType) .. vm.memory.len].ptr), .little));

    const pop_special = [_]u8{ (defs.PREFIX_OP2 << 4) | 0x7, 0xFF }; // POP R255
    reg_file.write(defs.R_IP, 0);
    try executePOP(&vm, &pop_special);
    try std.testing.expectEqual(vm.memory.len, reg_file.readSP());
    try std.testing.expectEqual(0x1234, reg_file.read(defs.R_IP));

    // Test 4-byte POP with offset
    reg_file.write(1, 0xAB);
    try executePUSH(&vm, &push_1byte); // PUSH R3 (0xAB)
    const pop_offset = [_]u8{ (defs.PREFIX_OP4 << 4) | 0x7, 0x03, 0x01, 0x01 }; // POP R3, ofs=1
    reg_file.write(3, 0);
    try executePOP(&vm, &pop_offset);
    try std.testing.expectEqual(vm.memory.len - 1, reg_file.readSP()); // SP unchanged
    try std.testing.expectEqual(0xAB, reg_file.read(1));
}
