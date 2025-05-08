// register_logic.zig
const std = @import("std");
const defs = @import("definitions.zig");
const regs = @import("registers.zig");
const vm_mod = @import("vm.zig");
const alu_mod = @import("alu.zig");
const builtin = @import("builtin");
const er = @import("errorreader.zig");
const IOPipe = @import("iopipe.zig").IOPipe;

// Execute RS (Register Select) instruction
pub fn executeRS(reg_file: *regs.RegisterFile, buffer: []const u8) !void {
    var cfg = reg_file.readALU_IO_CFG();
    const old_rs = cfg.arg1;

    switch (buffer.len) {
        1 => {
            // 1-byte: 0x1 reg (reg is u4)
            if (buffer[0] != 0x1) return error.InvalidOpcode;
            cfg.arg1 = buffer[0] & 0x0F; // Extract u4
        },
        2 => {
            // 2-byte: 0xC 0x1 reg (reg is u8)
            if (buffer[0] != defs.PREFIX_OP2 or buffer[1] != 0x1) return error.InvalidOpcode;
            cfg.arg1 = buffer[2]; // u8
        },
        else => return error.InvalidInstructionLength,
    }

    // Reset NS if RS changed
    if (cfg.arg1 != old_rs) {
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
    var reg_index: u8 = cfg.arg1;
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
    cfg.arg1 = reg_index;

    // Load value into register at nibble offset NS
    const reg_value = reg_file.read(reg_index);
    var new_value: defs.RegisterType = @truncate(reg_value);

    // Assume NS is nibble index (0–7 for 32-bit register)
    if (cfg.ns >= 8) return error.InvalidNibbleIndex; // 32 bits = 8 nibbles
    const bit_offset: u5 = @truncate(cfg.ns * 4); // Nibble = 4 bits

    // Handle ADT types
    switch (mode.adt) {
        .u4, .i4, .u8, .i8 => {
            if (bit_offset > 24) return error.InvalidNibbleIndex; // 8-bit value needs 2 nibbles
            new_value &= ~(@as(defs.RegisterType, 0xFF) << @truncate(cfg.ns * 4)); // Clear target byte
            new_value |= (@as(defs.RegisterType, @truncate(value)) << @truncate(cfg.ns * 4));
        },
        .u16, .i16 => {
            if (bit_offset > 16) return error.InvalidNibbleIndex; // 16-bit value needs 4 nibbles
            new_value &= ~(@as(defs.RegisterType, 0xFFFF) << @truncate(cfg.ns * 4)); // Clear target word
            new_value |= (@as(defs.RegisterType, @truncate(value)) << @truncate(cfg.ns * 4));
        },
        .u32, .i32 => {
            if (bit_offset > 0) return error.InvalidNibbleIndex; // 32-bit value needs full register
            new_value = @as(defs.RegisterType, @truncate(value));
        },
        .u64, .i64 => {
            if (bit_offset > 0 or value_bits > defs.WS) return error.InvalidNibbleIndex; // 64-bit value exceeds u32 register
            new_value = @as(defs.RegisterType, @truncate(value));
        },
        else => return error.UnsupportedADT,
    }

    // Sign-extend if ADT is signed and value is negative
    if (mode.adt.signed() and (value & (@as(u64, 1) << (value_bits - 1))) != 0) {
        const i = bit_offset + value_bits;
        const v = (@as(defs.RegisterType, 1) << @truncate(i)) - 1;
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

// Helper function for INC and DEC instructions
fn executeIncDec(reg_file: *regs.RegisterFile, buffer: []const u8, is_increment: bool) !void {
    const cfg = reg_file.readALU_IO_CFG();
    const mode = reg_file.readALU_MODE_CFG();
    var reg_index: u8 = cfg.arg1;
    var value: defs.RegisterType = 1; // Default to +1 for INC, -1 for DEC
    const o: u4 = if (is_increment) 0x3 else 0x4;

    switch (buffer.len) {
        1 => {
            // 1-byte: 0x0 0x3 (INC) or 0x0 0x4 (DEC) (R[N.RS]++/--)
            if (buffer[0] != o) return error.InvalidOpcode;
        },
        2 => {
            // 2-byte: 0xC3/4 reg (R[reg]++/--, reg is u4)
            if (buffer[0] != (defs.PREFIX_OP2 << 4) or (buffer[1] >> 4) != o) return error.InvalidOpcode;
            reg_index = buffer[1] & 0x0F; // u4
        },
        4 => {
            // 4-byte: 0xD3/4 reg val (R[reg] +/-= val, reg is u8, val is u12/i12)
            if (buffer[0] != (defs.PREFIX_OP4 << 4) or (buffer[1] >> 4) != o) return error.InvalidOpcode;
            reg_index = (buffer[1] & 0x0F) << 4; // u8
            reg_index |= buffer[2] >> 4;
            var buf: [2]u8 = undefined;
            buf[0] = (buffer[2] & 0xF);
            buf[1] = buffer[3];
            const v = std.mem.readInt(u16, &buf, .little);
            value = v;
        },
        8 => {
            // 8-byte: 0xE3/4 reg val (R[reg] +/-= val, reg is u8, val is u44/i44)
            if (buffer[0] != (defs.PREFIX_OP8 << 4) or (buffer[1] >> 4) != o) return error.InvalidOpcode;
            reg_index = (buffer[1] & 0x0F) << 4; // u8
            reg_index |= buffer[2] >> 4;
            var buf: [6]u8 = undefined;
            buf[0] = buffer[2] & 0x0F;
            buf[1] = buffer[3];
            buf[2] = buffer[4];
            buf[3] = buffer[4];
            buf[4] = buffer[5];
            buf[5] = buffer[6];
            const v: u48 = std.mem.readInt(u48, &buf, .little);
            value = v;
        },
        else => return error.InvalidInstructionLength,
    }

    // Read current register value
    var rv = reg_file.read(reg_index);
    const bs = mode.adt.bits();
    const one: defs.RegisterType = 1;
    const mask: defs.RegisterType = (one << @truncate(bs)) - 1;

    if (is_increment) rv = rv +% value else rv = rv -% value;

    if (mask != 0) rv &= mask; // with adt == u64, mask overflows.

    // Write back to register
    reg_file.write(reg_index, rv);
}

// Execute INC (Increment) instruction
pub fn executeINC(reg_file: *regs.RegisterFile, buffer: []const u8) !void {
    try executeIncDec(reg_file, buffer, true);
}

// Execute DEC (Decrement) instruction
pub fn executeDEC(reg_file: *regs.RegisterFile, buffer: []const u8) !void {
    try executeIncDec(reg_file, buffer, false);
}

// In register_logic.zig, append to existing code

// Execute NOT instruction
pub fn executeNOT(reg_file: *regs.RegisterFile, buffer: []const u8) !void {
    const cfg = reg_file.readALU_IO_CFG();
    const mode = reg_file.readALU_MODE_CFG();
    var reg_index: u16 = cfg.arg1; // Default to N.RS for 1-byte form
    var count: u8 = 1; // Default to 1 register

    switch (buffer.len) {
        1 => {
            // 1-byte: 0x05 (invert R[N.RS])
            if (buffer[0] != 0x05) return error.InvalidOpcode;
            // reg_index already set to cfg.arg1
        },
        2 => {
            // 2-byte: 0xC0, 0x5|reg (reg is u4)
            if (buffer[0] != (defs.PREFIX_OP2 << 4) or (buffer[1] >> 4) != 0x5) return error.InvalidOpcode;
            reg_index = buffer[1] & 0x0F; // Extract u4
        },
        4 => {
            // 4-byte: 0xD0, 0x5X, reg, cnt (reg is u8, cnt is u8)
            if (buffer[0] != (defs.PREFIX_OP4 << 4) or (buffer[1] >> 4) != 0x5) return error.InvalidOpcode;
            reg_index = buffer[2]; // u8
            count = buffer[3]; // u8
            if (count == 0) return; // No registers to invert
        },
        else => return error.InvalidInstructionLength,
    }

    // Validate register range
    if (reg_index >= defs.REGISTER_COUNT or reg_index + count > defs.REGISTER_COUNT) {
        return error.InvalidRegisterIndex;
    }

    const bs = mode.adt.bits();
    const one: defs.RegisterType = 1;
    var mask: defs.RegisterType = (one << @truncate(bs)) - 1;

    if (mask == 0) mask = ~mask; // with adt == u64, mask overflows.

    // Invert each register in the range
    for (0..count) |i| {
        const current_reg: u8 = @truncate(reg_index + i);
        const value = reg_file.read(current_reg);
        const inverted_value = ~value & mask;
        reg_file.write(current_reg, inverted_value);
    }
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
    cfg.arg1 = 1;
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
    cfg.arg1 = 2;
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
        cfg.arg1 = 2;
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
        cfg.arg1 = 2;
        cfg.ns = 0;
        reg_file.writeALU_IO_CFG(cfg);
        const li_8byte = [_]u8{ (defs.PREFIX_OP8 << 4) | 0x3, 0x2, 0xBC, 0x9A, 0x78, 0x56, 0x34, 0x12 };
        try executeLI(&reg_file, &li_8byte);
        try std.testing.expectEqual(0x123456789ABC, reg_file.read(2));
        cfg = reg_file.readALU_IO_CFG();
        try std.testing.expectEqual(12, cfg.ns);
    }
}

test "INC and DEC instructions" {
    var reg_file = regs.RegisterFile.init();

    var cfg = reg_file.readALU_IO_CFG();
    var mode = reg_file.readALU_MODE_CFG();

    // Test 1-byte INC: 0x03 (R[RS]++)
    cfg.arg1 = 1;
    reg_file.writeALU_IO_CFG(cfg);
    mode.adt = defs.ADT.u8;
    reg_file.writeALU_MODE_CFG(mode);
    reg_file.write(1, 0xFE);
    const inc_1byte = [_]u8{0x03};
    try executeINC(&reg_file, &inc_1byte);
    try std.testing.expectEqual(0xFF, reg_file.read(1));

    // Test 1-byte DEC: 0x04 (R[RS]--)
    reg_file.write(1, 0x01);
    const dec_1byte = [_]u8{0x04};
    try executeDEC(&reg_file, &dec_1byte);
    try std.testing.expectEqual(0x00, reg_file.read(1));

    // Test 2-byte INC: 0xC0 0x32 (R[2]++)
    mode.adt = defs.ADT.u16;
    reg_file.writeALU_MODE_CFG(mode);
    reg_file.write(2, 0xFFFF);
    const inc_2byte = [_]u8{ defs.PREFIX_OP2 << 4, 0x32 };
    try executeINC(&reg_file, &inc_2byte);
    try std.testing.expectEqual(0x0, reg_file.read(2)); // Wraparound

    // Test 2-byte DEC: 0xC0 0x42 (R[2]--)
    reg_file.write(2, 0x0000);
    const dec_2byte = [_]u8{ defs.PREFIX_OP2 << 4, 0x42 };
    try executeDEC(&reg_file, &dec_2byte);
    try std.testing.expectEqual(0xFFFF, reg_file.read(2)); // Wraparound

    // Test 4-byte INC: 0xD0 0x30 0x35 0x00 (R[3] += 5)
    mode.adt = defs.ADT.u32;
    reg_file.writeALU_MODE_CFG(mode);
    reg_file.write(3, 0x0);
    const inc_4byte = [_]u8{ defs.PREFIX_OP4 << 4, 0x30, 0x35, 0x0 };
    try executeINC(&reg_file, &inc_4byte);
    try std.testing.expectEqual(0x5, reg_file.read(3));

    // Test 4-byte DEC: 0xD0 0x40 0x35 0x00 (R[3] -= 5)
    reg_file.write(3, 0x1005);
    const dec_4byte = [_]u8{ defs.PREFIX_OP4 << 4, 0x40, 0x35, 0x00 };
    try executeDEC(&reg_file, &dec_4byte);
    try std.testing.expectEqual(0x1000, reg_file.read(3));

    if (defs.WS >= 64) {
        // Test 8-byte INC: 0xE0 0x30 0x4A 0x00 ... (R[4] += 10)
        mode.adt = defs.ADT.u64;
        reg_file.writeALU_MODE_CFG(mode);
        reg_file.write(4, 0x123456789ABC);
        try std.testing.expectEqual(0x123456789ABC, reg_file.read(4));
        const inc_8byte = [_]u8{ defs.PREFIX_OP8 << 4, 0x30, 0x4A, 0x00, 0x00, 0x00, 0x00, 0x00 };
        try executeINC(&reg_file, &inc_8byte);
        try std.testing.expectEqual(0x123456789AC6, reg_file.read(4));

        // Test 8-byte DEC: 0xE0 0x40 0x4A 0x00 ... (R[4] -= 10)
        reg_file.write(4, 0x123456789AC6);
        const dec_8byte = [_]u8{ defs.PREFIX_OP8 << 4, 0x40, 0x4A, 0x00, 0x00, 0x00, 0x00, 0x00 };
        try executeDEC(&reg_file, &dec_8byte);
        try std.testing.expectEqual(0x123456789ABC, reg_file.read(4));
    }

    // Test invalid INC opcode
    const invalid_inc = [_]u8{0x05};
    try std.testing.expectError(error.InvalidOpcode, executeINC(&reg_file, &invalid_inc));

    // Test invalid DEC opcode
    const invalid_dec = [_]u8{0x06};
    try std.testing.expectError(error.InvalidOpcode, executeDEC(&reg_file, &invalid_dec));

    // Test invalid length for INC
    const invalid_length_inc = [_]u8{ 0x03, 0x00, 0x00 };
    try std.testing.expectError(error.InvalidInstructionLength, executeINC(&reg_file, &invalid_length_inc));

    // Test invalid length for DEC
    const invalid_length_dec = [_]u8{ 0x04, 0x00, 0x00 };
    try std.testing.expectError(error.InvalidInstructionLength, executeDEC(&reg_file, &invalid_length_dec));
}

test "NOT instruction" {
    var reg_file = regs.RegisterFile.init();

    var cfg = reg_file.readALU_IO_CFG();
    var mode = reg_file.readALU_MODE_CFG();

    // Setup registers
    cfg.arg1 = 1;
    cfg.arg2 = 2;
    cfg.dst = 3;
    reg_file.writeALU_IO_CFG(cfg);
    mode.adt = .u8;
    reg_file.writeALU_MODE_CFG(mode);

    // Test 1-byte NOT: 0x05 (invert R[N.RS])
    reg_file.write(1, 0xFF);
    const not_1byte = [_]u8{0x05};
    try executeNOT(&reg_file, &not_1byte);
    try std.testing.expectEqual(0, reg_file.read(1));

    // Test 2-byte NOT: 0xC0 0x52 (invert R[2])
    mode.adt = .u32;
    reg_file.writeALU_MODE_CFG(mode);
    reg_file.write(2, 0xAABB);
    const not_2byte = [_]u8{ (defs.PREFIX_OP2 << 4), 0x52 }; // reg=2
    try executeNOT(&reg_file, &not_2byte);
    try std.testing.expectEqual(@as(defs.RegisterType, ~@as(u32, 0xAABB)), reg_file.read(2));

    // Test 4-byte NOT: 0xD0 0x50 0x03 0x02 (invert R[3] to R[4])
    reg_file.write(3, 0x11223344);
    reg_file.write(4, 0x55667788);
    const not_4byte = [_]u8{ (defs.PREFIX_OP4 << 4), 0x50, 0x03, 0x02 }; // reg=3, cnt=2
    try executeNOT(&reg_file, &not_4byte);
    try std.testing.expectEqual(@as(defs.RegisterType, ~@as(u32, 0x11223344)), reg_file.read(3));
    try std.testing.expectEqual(@as(defs.RegisterType, ~@as(u32, 0x55667788)), reg_file.read(4));

    // Test 4-byte NOT with cnt=0 (no operation)
    reg_file.write(5, 0x12345678);
    const not_4byte_cnt0 = [_]u8{ (defs.PREFIX_OP4 << 4), 0x50, 0x05, 0x00 }; // reg=5, cnt=0
    try executeNOT(&reg_file, &not_4byte_cnt0);
    try std.testing.expectEqual(@as(defs.RegisterType, 0x12345678), reg_file.read(5)); // Unchanged

    // Test invalid opcode
    const invalid_not = [_]u8{0x06};
    try std.testing.expectError(error.InvalidOpcode, executeNOT(&reg_file, &invalid_not));

    // Test invalid length
    const invalid_length = [_]u8{ 0x05, 0x00, 0x00 };
    try std.testing.expectError(error.InvalidInstructionLength, executeNOT(&reg_file, &invalid_length));

    // Test invalid register index
    cfg.arg1 = 255; // Still set from previous
    reg_file.writeALU_IO_CFG(cfg);
    const not_invalid_reg = [_]u8{0x05}; // 1-byte, uses N.RS=255
    try executeNOT(&reg_file, &not_invalid_reg); // Should not fail, as R_IP (255) is valid
    const not_invalid_range = [_]u8{ (defs.PREFIX_OP4 << 4), 0x50, 0xFF, 0x02 }; // reg=255, cnt=2
    try std.testing.expectError(error.InvalidRegisterIndex, executeNOT(&reg_file, &not_invalid_range));
}
