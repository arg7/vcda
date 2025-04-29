// register_logic.zig
const std = @import("std");
const defs = @import("definitions.zig");
const regs = @import("registers.zig");

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
    var new_value: regs.RegisterType = reg_value;

    // Assume NS is nibble index (0–7 for 32-bit register)
    if (cfg.ns >= 8) return error.InvalidNibbleIndex; // 32 bits = 8 nibbles
    const bit_offset: u5 = @truncate(cfg.ns * 4); // Nibble = 4 bits

    // Handle ADT types
    switch (mode.adt) {
        .u4, .i4, .u8, .i8 => {
            if (bit_offset > 24) return error.InvalidNibbleIndex; // 8-bit value needs 2 nibbles
            new_value &= ~(@as(regs.RegisterType, 0xFF) << bit_offset); // Clear target byte
            new_value |= (@as(regs.RegisterType, @truncate(value)) << bit_offset);
        },
        .u16, .i16 => {
            if (bit_offset > 16) return error.InvalidNibbleIndex; // 16-bit value needs 4 nibbles
            new_value &= ~(@as(regs.RegisterType, 0xFFFF) << bit_offset); // Clear target word
            new_value |= (@as(regs.RegisterType, @truncate(value)) << bit_offset);
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
        const v = (@as(regs.RegisterType, 1) << @truncate(i))-1;
        const mask = ~v;
        new_value |= mask; // Set upper bits to 1
    }

    // Write back to register
    reg_file.write(reg_index, new_value);

    // Update NS
    cfg.ns +%= ns_increment; // Wraparound for u8
    reg_file.writeALU_IO_CFG(cfg);
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
    const li_2byte = [_]u8{(defs.PREFIX_OP2 << 4) | 0x3, 0x2B};
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
        const li_4byte = [_]u8{(defs.PREFIX_OP4 << 4) | 0x3, 0x2, 0x34, 0x12};
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
        const li_8byte = [_]u8{(defs.PREFIX_OP8 << 4) | 0x3, 0x2, 0xBC, 0x9A, 0x78, 0x56, 0x34, 0x12};
        try executeLI(&reg_file, &li_8byte);
        try std.testing.expectEqual( 0x123456789ABC, reg_file.read(2));
        cfg = reg_file.readALU_IO_CFG();
        try std.testing.expectEqual(12, cfg.ns);
    }
}
