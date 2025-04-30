// register_logic.zig
const std = @import("std");
const defs = @import("definitions.zig");
const regs = @import("registers.zig");
const vm_mod = @import("vm.zig");

// Execute RS (Register Select) instruction
pub fn executeRS(reg_file: *regs.RegisterFile, buffer: []const u8) !void {
    var cfg = reg_file.readALU_IO_CFG();
    const old_rs = cfg.rs;

    switch (buffer.len) {
        1 => {
            // 1-byte: 0x1 reg (reg is u4)
            if (buffer[0] != 0x1) return error.InvalidOpcode;
            cfg.rs = buffer[0] & 0x0F; // Extract u4
        },
        2 => {
            // 2-byte: 0xC 0x1 reg (reg is u8)
            if (buffer[0] != defs.PREFIX_OP2 or buffer[1] != 0x1) return error.InvalidOpcode;
            cfg.rs = buffer[2]; // u8
        },
        else => return error.InvalidInstructionLength,
    }

    // Reset NS if RS changed
    if (cfg.rs != old_rs) {
        cfg.ns = 0;
    }

    reg_file.writeALU_IO_CFG(cfg);
}

pub fn executeNS(reg_file: *regs.RegisterFile, buffer: []const u8) !void {
    var cfg = reg_file.readALU_IO_CFG();

    switch (buffer.len) {
        1 => {
            // 1-byte: 0x2 val (val is u4)
            if (buffer[0] != 0x2) return error.InvalidOpcode;
            cfg.ns = buffer[0] & 0x0F; // Extract u4
        },
        2 => {
            // 2-byte: 0xC 0x2 val (val is u8)
            if (buffer[0] != defs.PREFIX_OP2 or buffer[1] != 0x2) return error.InvalidOpcode;
            cfg.ns = buffer[2]; // u8
        },
        else => return error.InvalidInstructionLength,
    }

    reg_file.writeALU_IO_CFG(cfg);
}

// Execute LI (Load Immediate) instruction
pub fn executeLI(reg_file: *regs.RegisterFile, buffer: []const u8) !void {
    var cfg = reg_file.readALU_IO_CFG();
    var mode = reg_file.readALU_MODE_CFG();
    var reg_index: u8 = cfg.rs;
    var value: u64 = 0; // Large enough to hold any immediate (u48 max)
    var ns_increment: u8 = 0;
    var value_bits: u6 = 0; // Size of immediate in bits

    switch (buffer.len) {
        1 => {
            // 1-byte: 0x3 val (val is u4/i4)
            if ((buffer[0] >> 4) != 0x3) return error.InvalidOpcode;
            value = buffer[0] & 0x0F; // u4
            value_bits = 4;
            ns_increment = 1;
        },
        2 => {
            // 2-byte: 0xC 0x3 reg val (reg is u4, val is u4/i4)
            if ((buffer[0] >> 4) != defs.PREFIX_OP2 or (buffer[0] & 0x0F) != 0x3) return error.InvalidOpcode;
            reg_index = buffer[1] >> 4; // u4
            value = buffer[1] & 0x0F; // u4
            value_bits = 4;
            ns_increment = 1;
        },
        4 => {
            // 4-byte: 0xD 0x3 reg val (reg is u8, val is u16/i16)
            if ((buffer[0] >> 4) != defs.PREFIX_OP4 or (buffer[0] & 0x0F) != 0x3) return error.InvalidOpcode;
            reg_index = buffer[1]; // u8
            value = std.mem.readInt(u16, buffer[2..4], .little);
            value_bits = 16;
            ns_increment = 4;
        },
        8 => {
            // 8-byte: 0xE 0x3 reg val (reg is u8, val is u48/i48)
            if ((buffer[0] >> 4) != defs.PREFIX_OP8 or (buffer[0] & 0x0F) != 0x3) return error.InvalidOpcode;
            reg_index = buffer[1]; // u8
            value = std.mem.readInt(u48, buffer[2..8], .little) & 0xFFFFFFFFFFFF; // Mask to 48 bits
            value_bits = 48;
            ns_increment = 12;
        },
        else => return error.InvalidInstructionLength,
    }

    // Update RS if necessary (for 2, 4, 8-byte forms)
    cfg.rs = reg_index;

    // Load value into register at nibble offset NS
    const reg_value = reg_file.read(reg_index);
    var new_value: regs.RegisterType = @truncate(reg_value);

    // Assume NS is nibble index (0–7 for 32-bit register)
    if (cfg.ns >= 8) return error.InvalidNibbleIndex; // 32 bits = 8 nibbles
    const bit_offset: u5 = @truncate(cfg.ns * 4); // Nibble = 4 bits

    // Handle ADT types
    switch (mode.adt) {
        .u4, .i4, .u8, .i8 => {
            if (bit_offset > 24) return error.InvalidNibbleIndex; // 8-bit value needs 2 nibbles
            new_value &= ~(@as(regs.RegisterType, 0xFF) << @truncate(cfg.ns * 4)); // Clear target byte
            new_value |= (@as(regs.RegisterType, @truncate(value)) << @truncate(cfg.ns * 4));
        },
        .u16, .i16 => {
            if (bit_offset > 16) return error.InvalidNibbleIndex; // 16-bit value needs 4 nibbles
            new_value &= ~(@as(regs.RegisterType, 0xFFFF) << @truncate(cfg.ns * 4)); // Clear target word
            new_value |= (@as(regs.RegisterType, @truncate(value)) << @truncate(cfg.ns * 4));
        },
        .u32, .i32 => {
            if (bit_offset > 0) return error.InvalidNibbleIndex; // 32-bit value needs full register
            new_value = @as(regs.RegisterType, @truncate(value));
        },
        .u64, .i64 => {
            if (bit_offset > 0 or value_bits > defs.WS) return error.InvalidNibbleIndex; // 64-bit value exceeds u32 register
            new_value = @as(regs.RegisterType, @truncate(value));
        },
        else => return error.UnsupportedADT,
    }

    // Sign-extend if ADT is signed and value is negative
    if (mode.adt.signed() and (value & (@as(u64, 1) << (value_bits - 1))) != 0) {
        const i = bit_offset + value_bits;
        const v = (@as(regs.RegisterType, 1) << @truncate(i)) - 1;
        const mask = ~v;
        new_value |= mask; // Set upper bits to 1
    }

    // Write back to register
    reg_file.write(reg_index, new_value);

    // Update NS
    cfg.ns +%= ns_increment; // Wraparound for u8
    reg_file.writeALU_IO_CFG(cfg);
}

// Execute NOP (No Operation) instruction
pub fn executeNOP(_: *regs.RegisterFile, buffer: []const u8) !void {
    switch (buffer.len) {
        1 => {
            // 1-byte: 0x0 0x0
            if ((buffer[0] >> 4) != 0x0 or (buffer[0] & 0x0F) != 0x0) return error.InvalidOpcode;
        },
        2 => {
            // 2-byte: 0xC 0x0 0x0 xx
            if ((buffer[0] >> 4) != defs.PREFIX_OP2 or (buffer[0] & 0x0F) != 0x0 or (buffer[1] >> 4) != 0x0) return error.InvalidOpcode;
        },
        4 => {
            // 4-byte: 0xD 0x0 0x0 xx xx xx xx
            if ((buffer[0] >> 4) != defs.PREFIX_OP4 or (buffer[0] & 0x0F) != 0x0 or (buffer[1] >> 4) != 0x0) return error.InvalidOpcode;
        },
        8 => {
            // 8-byte: 0xE 0x0 0x0 xx xx xx xx xx xx xx xx
            if ((buffer[0] >> 4) != defs.PREFIX_OP8 or (buffer[0] & 0x0F) != 0x0 or (buffer[1] >> 4) != 0x0) return error.InvalidOpcode;
        },
        else => return error.InvalidInstructionLength,
    }
}

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

// Execute POP instruction
pub fn executePOP(vm: *vm_mod.VM, buffer: []const u8) !void {
    const reg_file = &vm.registers;
    //var cfg = reg_file.readALU_IO_CFG();
    const mode = reg_file.readALU_MODE_CFG();
    var reg_index: u8 = 0;
    var count: u8 = 1; // Default to popping 1 register
    var ofs: u8 = 0; // Offset for 4-byte form

    switch (buffer.len) {
        1 => {
            // 1-byte: 0x7 reg (reg is u4)
            if ((buffer[0] >> 4) != 0x7) return error.InvalidOpcode;
            reg_index = buffer[0] & 0x0F; // Extract u4
        },
        2 => {
            // 2-byte: 0xC 0x7 reg (reg is u8)
            if ((buffer[0] >> 4) != defs.PREFIX_OP2 or (buffer[0] & 0x0F) != 0x7) return error.InvalidOpcode;
            reg_index = buffer[1]; // u8
        },
        4 => {
            // 4-byte: 0xD 0x7 reg cnt ofs
            if ((buffer[0] >> 4) != defs.PREFIX_OP4 or (buffer[0] & 0x0F) != 0x7) return error.InvalidOpcode;
            reg_index = buffer[1]; // u8
            count = buffer[2]; // u8
            ofs = buffer[3]; // u8
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

        // Calculate read address
        const read_addr = if (ofs > 0) sp - (ofs * byte_size) else sp;
        if (read_addr + byte_size > vm.memory.len) return error.StackUnderflow;

        // Read from memory
        var value: regs.SpecialRegisterType = 0;
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

        // Write to register
        reg_file.write(current_reg, value);

        // Update SP only if ofs == 0
        if (ofs == 0) sp += byte_size;
    }

    // Update SP
    reg_file.writeSP(sp);
}

// Helper function for RET and IRET
fn executeReturnImpl(vm: *vm_mod.VM, buffer: []const u8, expected_opcode: u8) !void {
    const reg_file = &vm.registers;
    var cnt: u8 = 0;

    // Validate instruction and extract cnt
    switch (buffer.len) {
        1 => {
            if (buffer[0] != expected_opcode) return error.InvalidOpcode;
        },
        2 => {
            if (buffer[0] != defs.PREFIX_OP2 or buffer[1] != expected_opcode) return error.InvalidOpcode;
        },
        4 => {
            if (buffer[0] != defs.PREFIX_OP4 or buffer[1] != 0x00 or buffer[2] != expected_opcode) return error.InvalidOpcode;
            cnt = buffer[3];
        },
        else => return error.InvalidInstructionLength,
    }

    // Pop IP (R255) using existing POP logic
    const pop_buffer = [_]u8{ defs.PREFIX_OP2 << 4 | 0x7, defs.R_IP }; // 2-byte POP R255
    try executePOP(vm, &pop_buffer);

    // Adjust SP by cnt (after POP, which already incremented SP by size_of_IP)
    var sp = reg_file.readSP();
    sp += cnt;
    reg_file.writeSP(sp);
}

// Execute RET instruction
pub fn executeRET(vm: *vm_mod.VM, buffer: []const u8) !void {
    try executeReturnImpl(vm, buffer, 0x01);
}

// Execute IRET instruction
pub fn executeIRET(vm: *vm_mod.VM, buffer: []const u8) !void {
    try executeReturnImpl(vm, buffer, 0x02);
}

test "NOP instruction" {
    var reg_file = regs.RegisterFile.init();
    const original_state = reg_file; // Capture initial state

    // Test 1-byte NOP: 0x00
    const nop_1byte = [_]u8{0x00};
    try executeNOP(&reg_file, &nop_1byte);
    try std.testing.expectEqual(original_state, reg_file);

    // Test 2-byte NOP: 0xC0 0x0x
    const nop_2byte = [_]u8{ (defs.PREFIX_OP2 << 4) | 0x0, 0x00 };
    try executeNOP(&reg_file, &nop_2byte);
    try std.testing.expectEqual(original_state, reg_file);

    // Test 4-byte NOP: 0xD0 0x0x xx xx
    const nop_4byte = [_]u8{ (defs.PREFIX_OP4 << 4) | 0x0, 0x00, 0xFF, 0xFF };
    try executeNOP(&reg_file, &nop_4byte);
    try std.testing.expectEqual(original_state, reg_file);

    // Test 8-byte NOP: 0xE0 0x0x xx xx xx xx xx xx
    const nop_8byte = [_]u8{ (defs.PREFIX_OP8 << 4) | 0x0, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    try executeNOP(&reg_file, &nop_8byte);
    try std.testing.expectEqual(original_state, reg_file);

    // Test invalid NOP opcode
    const invalid_nop = [_]u8{0x01};
    try std.testing.expectError(error.InvalidOpcode, executeNOP(&reg_file, &invalid_nop));

    // Test invalid length
    const invalid_length = [_]u8{ 0x00, 0x00, 0x00 };
    try std.testing.expectError(error.InvalidInstructionLength, executeNOP(&reg_file, &invalid_length));
}

// Unit Tests
test "LI instruction" {
    var reg_file = regs.RegisterFile.init();

    // Set ADT to u8
    var mode = reg_file.readALU_MODE_CFG();
    mode.adt = defs.ADT.i16;
    reg_file.writeALU_MODE_CFG(mode);

    // Test 1-byte LI: 0x3F (R[1][NS=0] = 0xFF)
    var cfg = reg_file.readALU_IO_CFG();
    cfg.rs = 1;
    cfg.ns = 0;
    reg_file.writeALU_IO_CFG(cfg);
    const li_1byte = [_]u8{0x3F};
    try executeLI(&reg_file, &li_1byte);
    try std.testing.expectEqual((1 << defs.WS) - 1, reg_file.read(1));
    cfg = reg_file.readALU_IO_CFG();
    try std.testing.expectEqual(1, cfg.ns);

    // Test 2-byte LI: 0xC 0x3 0x2 0xB (R[2][NS=0] = 0xB)
    mode.adt = defs.ADT.u8;
    reg_file.writeALU_MODE_CFG(mode);
    cfg.rs = 2;
    cfg.ns = 0;
    reg_file.writeALU_IO_CFG(cfg);
    const li_2byte = [_]u8{ (defs.PREFIX_OP2 << 4) | 0x3, 0x2B };
    try executeLI(&reg_file, &li_2byte);
    try std.testing.expectEqual(0xB, reg_file.read(2));
    cfg = reg_file.readALU_IO_CFG();
    try std.testing.expectEqual(1, cfg.ns);

    if (defs.WS >= 32) {
        // Test 4-byte LI: 0xD 0x3 0x3 0x1234 (R[2][NS=0] = 0x1234)
        mode.adt = defs.ADT.u16;
        reg_file.writeALU_MODE_CFG(mode);
        cfg.rs = 2;
        cfg.ns = 0;
        reg_file.writeALU_IO_CFG(cfg);
        const li_4byte = [_]u8{ (defs.PREFIX_OP4 << 4) | 0x3, 0x2, 0x34, 0x12 };
        try executeLI(&reg_file, &li_4byte);
        try std.testing.expectEqual(0x1234, reg_file.read(2));
        cfg = reg_file.readALU_IO_CFG();
        try std.testing.expectEqual(4, cfg.ns);
    }

    if (defs.WS >= 64) {
        // Test 8-byte LI: 0xE3 0x2 0x1234 (R[2][NS=0] = 0x1234)
        mode.adt = defs.ADT.u64;
        reg_file.writeALU_MODE_CFG(mode);
        cfg.rs = 2;
        cfg.ns = 0;
        reg_file.writeALU_IO_CFG(cfg);
        const li_8byte = [_]u8{ (defs.PREFIX_OP8 << 4) | 0x3, 0x2, 0xBC, 0x9A, 0x78, 0x56, 0x34, 0x12 };
        try executeLI(&reg_file, &li_8byte);
        try std.testing.expectEqual(0x123456789ABC, reg_file.read(2));
        cfg = reg_file.readALU_IO_CFG();
        try std.testing.expectEqual(12, cfg.ns);
    }
}

// Unit Tests
test "PUSH and POP instructions" {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    // Initialize VM with 1024-byte memory
    const program = [_]u8{0x00}; // Dummy program
    var vm = try vm_mod.VM.init(allocator, &program);
    defer vm.deinit(allocator);

    var reg_file = &vm.registers;
    var mode = reg_file.readALU_MODE_CFG();
    //var cfg = reg_file.readALU_IO_CFG();

    // Set SP to top of memory
    reg_file.writeSP(@as(regs.RegisterType, @truncate(vm.memory.len)));

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
    try std.testing.expectEqual(vm.memory.len - @sizeOf(regs.PointerRegisterType), reg_file.readSP());
    try std.testing.expectEqual(0x1234, std.mem.readInt(regs.PointerRegisterType, @ptrCast(vm.memory[vm.memory.len - @sizeOf(regs.PointerRegisterType) .. vm.memory.len].ptr), .little));

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

test "RET and IRET instructions" {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    // Initialize VM with 1024-byte memory
    const program = [_]u8{0x00}; // Dummy program
    var vm = try vm_mod.VM.init(allocator, &program);
    defer vm.deinit(allocator);

    var reg_file = &vm.registers;
    const ip_size = @sizeOf(regs.PointerRegisterType); // Size of IP (R255) in bytes

    // Set SP to top of memory
    const initial_sp = @as(regs.PointerRegisterType, @truncate(vm.memory.len));
    reg_file.writeSP(initial_sp);

    // Test 1-byte RET: 0x01
    {
        // Push return address (0x1234)
        const return_addr: regs.PointerRegisterType = 0x1234;
        reg_file.writeSP(initial_sp - ip_size);
        std.mem.writeInt(regs.PointerRegisterType, @ptrCast(vm.memory[initial_sp - ip_size ..][0..ip_size]), return_addr, .little);

        const ret_1byte = [_]u8{0x01};
        try executeRET(&vm, &ret_1byte);
        try std.testing.expectEqual(return_addr, reg_file.readIP());
        try std.testing.expectEqual(initial_sp, reg_file.readSP()); // SP restored after pop
    }

    // Test 2-byte RET: 0xC 0x01
    {
        reg_file.writeSP(initial_sp - ip_size);
        const return_addr: regs.PointerRegisterType = 0x5678;
        std.mem.writeInt(regs.PointerRegisterType, @ptrCast(vm.memory[initial_sp - ip_size ..][0..ip_size]), return_addr, .little);

        const ret_2byte = [_]u8{ defs.PREFIX_OP2, 0x01 };
        try executeRET(&vm, &ret_2byte);
        try std.testing.expectEqual(return_addr, reg_file.readIP());
        try std.testing.expectEqual(initial_sp, reg_file.readSP());
    }

    // Test 4-byte RET: 0xD 0x00 0x01 cnt
    {
        reg_file.writeSP(initial_sp - ip_size);
        const return_addr: regs.PointerRegisterType = 0x9ABC;
        std.mem.writeInt(regs.PointerRegisterType, @ptrCast(vm.memory[initial_sp - ip_size ..][0..ip_size]), return_addr, .little);

        const cnt: u8 = 8; // Simulate popping 8 bytes of parameters
        const ret_4byte = [_]u8{ defs.PREFIX_OP4, 0x00, 0x01, cnt };
        try executeRET(&vm, &ret_4byte);
        try std.testing.expectEqual(return_addr, reg_file.readIP());
        try std.testing.expectEqual(initial_sp + cnt, reg_file.readSP()); // SP += cnt after pop
    }

    // Test 1-byte IRET: 0x02
    {
        reg_file.writeSP(initial_sp - ip_size);
        const return_addr: regs.PointerRegisterType = 0x2345;
        std.mem.writeInt(regs.PointerRegisterType, @ptrCast(vm.memory[initial_sp - ip_size ..][0..ip_size]), return_addr, .little);

        const iret_1byte = [_]u8{0x02};
        try executeIRET(&vm, &iret_1byte);
        try std.testing.expectEqual(return_addr, reg_file.readIP());
        try std.testing.expectEqual(initial_sp, reg_file.readSP());
    }

    // Test 4-byte IRET: 0xD 0x00 0x02 cnt
    {
        reg_file.writeSP(initial_sp - ip_size);
        const return_addr: regs.PointerRegisterType = 0x6789;
        std.mem.writeInt(regs.PointerRegisterType, @ptrCast(vm.memory[initial_sp - ip_size ..][0..ip_size]), return_addr, .little);

        const cnt: u8 = 12;
        const iret_4byte = [_]u8{ defs.PREFIX_OP4, 0x00, 0x02, cnt };
        try executeIRET(&vm, &iret_4byte);
        try std.testing.expectEqual(return_addr, reg_file.readIP());
        try std.testing.expectEqual(initial_sp + cnt, reg_file.readSP());
    }

    // Test invalid RET opcode
    {
        const invalid_ret = [_]u8{0x03};
        try std.testing.expectError(error.InvalidOpcode, executeRET(&vm, &invalid_ret));
    }

    // Test invalid IRET length
    {
        const invalid_iret = [_]u8{ 0x02, 0x00, 0x00 };
        try std.testing.expectError(error.InvalidInstructionLength, executeIRET(&vm, &invalid_iret));
    }

    // Test stack underflow
    {
        reg_file.writeSP(0); // SP too low
        const ret_1byte = [_]u8{0x01};
        try std.testing.expectError(error.StackUnderflow, executeRET(&vm, &ret_1byte));
    }
}
