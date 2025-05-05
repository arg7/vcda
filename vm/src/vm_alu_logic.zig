const std = @import("std");
const defs = @import("definitions.zig");
const regs = @import("registers.zig");
const vm_mod = @import("vm.zig");
const alu_mod = @import("alu.zig");
const builtin = @import("builtin");
const er = @import("errorreader.zig");
const IOPipe = @import("iopipe.zig").IOPipe;

// Map ADT to Zig type and call ALU
fn callALU(
    op: defs.AMOD,
    adt: defs.ADT,
    arg1: defs.RegisterType,
    arg1h: ?defs.RegisterType,
    arg2: defs.RegisterType,
    carry_in: ?u1,
) !struct { ret: defs.RegisterType, reth: ?defs.RegisterType, carry_out: ?u1 } {
    // Signed versions of args
    const a1: defs.RegisterSignedType = @bitCast(arg1);
    const a1h: ?defs.RegisterSignedType = if (arg1h) |h| @bitCast(h) else null;
    const a2: defs.RegisterSignedType = @bitCast(arg2);

    switch (adt) {
        .u1 => {
            const result = try alu_mod.alu(op, u1, @truncate(arg1), if (arg1h) |h| @truncate(h) else null, @truncate(arg2), carry_in);
            return .{ .ret = result.ret, .reth = if (result.reth) |h| h else null, .carry_out = result.carry_out };
        },
        .u4 => {
            const result = try alu_mod.alu(op, u4, @truncate(arg1), if (arg1h) |h| @truncate(h) else null, @truncate(arg2), carry_in);
            return .{ .ret = @as(defs.RegisterType, result.ret), .reth = if (result.reth) |h| @as(defs.RegisterType, h) else null, .carry_out = result.carry_out };
        },
        .i4 => {
            var r: u4 = 0;
            var rh: ?u4 = null;

            const result = try alu_mod.alu(op, i4, @truncate(a1), if (a1h) |h| @truncate(h) else null, @truncate(a2), carry_in);
            r = @bitCast(result.ret);
            rh = if (result.reth) |h| @bitCast(h) else null;
            return .{ .ret = r, .reth = if (rh) |h| h else null, .carry_out = result.carry_out };
        },
        .u8 => {
            const result = try alu_mod.alu(op, u8, @truncate(arg1), if (arg1h) |h| @truncate(h) else null, @truncate(arg2), carry_in);
            return .{ .ret = result.ret, .reth = if (result.reth) |h| h else null, .carry_out = result.carry_out };
        },
        .i8 => {
            var r: u8 = 0;
            var rh: ?u8 = null;

            const result = try alu_mod.alu(op, i8, @truncate(a1), if (a1h) |h| @truncate(h) else null, @truncate(a2), carry_in);
            r = @bitCast(result.ret);
            rh = if (result.reth) |h| @bitCast(h) else null;
            return .{ .ret = r, .reth = if (rh) |h| h else null, .carry_out = result.carry_out };
        },
        .u16 => {
            const result = try alu_mod.alu(op, u16, @truncate(arg1), if (arg1h) |h| @truncate(h) else null, @truncate(arg2), carry_in);
            return .{ .ret = @as(defs.RegisterType, result.ret), .reth = if (result.reth) |h| @as(defs.RegisterType, h) else null, .carry_out = result.carry_out };
        },
        .i16 => {
            var r: u16 = 0;
            var rh: ?u16 = null;

            const result = try alu_mod.alu(op, i16, @truncate(a1), if (a1h) |h| @truncate(h) else null, @truncate(a2), carry_in);
            r = @bitCast(result.ret);
            rh = if (result.reth) |h| @bitCast(h) else null;
            return .{ .ret = r, .reth = if (rh) |h| h else null, .carry_out = result.carry_out };
        },
        .u32 => {
            const result = try alu_mod.alu(op, u32, @truncate(arg1), if (arg1h) |h| @truncate(h) else null, @truncate(arg2), carry_in);
            return .{ .ret = @as(defs.RegisterType, result.ret), .reth = if (result.reth) |h| @as(defs.RegisterType, h) else null, .carry_out = result.carry_out };
        },
        .i32 => {
            var r: u32 = 0;
            var rh: ?u32 = null;
            const result = try alu_mod.alu(op, i32, @truncate(a1), if (a1h) |h| @truncate(h) else null, @truncate(a2), carry_in);
            r = @bitCast(result.ret);
            rh = if (result.reth) |h| @bitCast(h) else null;
            return .{ .ret = r, .reth = if (rh) |h| h else null, .carry_out = result.carry_out };
        },
        .u64 => {
            const result = try alu_mod.alu(op, u64, @truncate(arg1), if (arg1h) |h| @truncate(h) else null, @truncate(arg2), carry_in);
            return .{ .ret = @as(defs.RegisterType, result.ret), .reth = if (result.reth) |h| @as(defs.RegisterType, h) else null, .carry_out = result.carry_out };
        },
        .i64 => {
            var r: u64 = 0;
            var rh: ?u64 = null;
            const result = try alu_mod.alu(op, i64, @truncate(a1), if (a1h) |h| @truncate(h) else null, @truncate(a2), carry_in);
            r = @bitCast(result.ret);
            rh = if (result.reth) |h| @bitCast(h) else null;
            return .{ .ret = r, .reth = if (rh) |h| h else null, .carry_out = result.carry_out };
        },
        .f16, .f32, .f64, .fp4, .fp8 => return error.NotImplemented,
    }
}

// Execute ALU instruction
pub fn executeALU(vm: *vm_mod.VM, buffer: []const u8) !void {
    const reg_file = &vm.registers;
    var cfg = reg_file.readALU_IO_CFG();
    const mode = reg_file.readALU_MODE_CFG();
    var op: defs.AMOD = undefined;
    var adt: defs.ADT = mode.adt; // Default to current ADT
    var rs: u8 = cfg.rs;
    var src: u8 = cfg.src;
    var dst: u8 = cfg.dst;
    var ofs: i16 = 0;

    switch (buffer.len) {
        1 => {
            // 1-byte: 0x8|op (op is u4)
            if ((buffer[0] >> 4) != 0x8) return error.InvalidOpcode;
            op = @enumFromInt(buffer[0] & 0x0F);
        },
        2 => {
            // 2-byte: 0xC8, op|adt (op, adt are u4)
            if (buffer[0] != ((defs.PREFIX_OP2 << 4) | 0x8)) return error.InvalidOpcode;
            op = @enumFromInt(buffer[1] >> 4);
            adt = @enumFromInt(buffer[1] & 0x0F);
        },
        4 => {
            // 4-byte: 0xD8, op|adt, a1|a2, r (op, adt, a1, a2 are u4, r is u8)
            if (buffer[0] != ((defs.PREFIX_OP4 << 4) | 0x8)) return error.InvalidOpcode;
            op = @enumFromInt(buffer[1] >> 4);
            adt = @enumFromInt(buffer[1] & 0x0F);
            rs = buffer[2] >> 4;
            src = buffer[2] & 0x0F;
            dst = buffer[3];
            // Update ALU_IO_CFG
            cfg.rs = rs;
            cfg.src = src;
            cfg.dst = dst;
            reg_file.writeALU_IO_CFG(cfg);
        },
        8 => {
            // 8-byte: 0xE8, op, adt, a1, a2, r, ofs (op, adt, a1, a2, r are u8, ofs is i16)
            if (buffer[0] != ((defs.PREFIX_OP8 << 4) | 0x8)) return error.InvalidOpcode;
            op = @enumFromInt(buffer[1]);
            adt = @enumFromInt(buffer[2]);
            rs = buffer[3];
            src = buffer[4];
            dst = buffer[5];
            ofs = std.mem.readInt(i16, buffer[6..8], .little);
            // Update ALU_IO_CFG
            cfg.rs = rs;
            cfg.src = src;
            cfg.dst = dst;
            reg_file.writeALU_IO_CFG(cfg);
        },
        else => return error.InvalidInstructionLength,
    }

    // Validate registers
    if (rs >= defs.REGISTER_COUNT or src >= defs.REGISTER_COUNT or dst >= defs.REGISTER_COUNT) {
        return error.InvalidRegisterIndex;
    }

    // Handle LOAD/STORE for 8-byte form
    if (op == .load or op == .store) {
        if (buffer.len != 8) return error.InvalidInstructionLength;
        var addr = reg_file.read(src);
        const v = @abs(ofs);
        if (ofs > 0) addr += v else addr -= v;
        const byte_size = (adt.bits() + 7) >> 3; // Ceiling to bytes
        if (addr >= vm.memory.len or addr + byte_size > vm.memory.len) {
            return error.InvalidMemoryAddress;
        }

        if (op == .load) {
            var value: defs.RegisterType = 0;
            switch (byte_size) {
                1 => value = vm.memory[addr],
                2 => value = std.mem.readInt(u16, vm.memory[addr..][0..2], .little),
                4 => value = std.mem.readInt(u32, vm.memory[addr..][0..4], .little),
                8 => value = std.mem.readInt(u64, vm.memory[addr..][0..8], .little),
                else => return error.InvalidDataType,
            }
            reg_file.write(dst, value);
        } else { // store
            const value = reg_file.read(rs);
            switch (byte_size) {
                1 => vm.memory[addr] = @truncate(value),
                2 => std.mem.writeInt(u16, vm.memory[addr..][0..2], @truncate(value), .little),
                4 => std.mem.writeInt(u32, vm.memory[addr..][0..4], @truncate(value), .little),
                8 => std.mem.writeInt(u64, vm.memory[addr..][0..8], @truncate(value), .little),
                else => return error.InvalidDataType,
            }
        }
        return;
    }

    // Perform ALU operation
    const arg1 = reg_file.read(rs);
    const arg2 = reg_file.read(src);
    const result = try callALU(op, adt, arg1, null, arg2, null);
    reg_file.write(dst, result.ret);

    // Handle high result for multiplication
    if (op == .mul and result.reth != null) {
        if (dst + 1 < defs.REGISTER_COUNT) {
            reg_file.write(dst + 1, result.reth.?);
        }
    }
    // TODO: Store carry_out in a flag register if needed for BCS (carry, overflow, etc.)
}

test "ALU instruction" {
    const allocator = std.testing.allocator;

    // Initialize VM
    var vm = try vm_mod.VM.init(allocator, null, null, null, 0xFFFF);
    defer vm.deinit();

    var reg_file = &vm.registers;
    var cfg = reg_file.readALU_IO_CFG();
    var mode = reg_file.readALU_MODE_CFG();

    // Setup registers
    cfg.rs = 1;
    cfg.src = 2;
    cfg.dst = 3;
    reg_file.writeALU_IO_CFG(cfg);
    mode.adt = .u8;
    reg_file.writeALU_MODE_CFG(mode);

    // Test 1-byte ALU: 0x80 (ADD, R3 = R1 + R2)
    reg_file.write(1, 200);
    reg_file.write(2, 100);
    const alu_1byte = [_]u8{0x80};
    try executeALU(&vm, &alu_1byte);
    try std.testing.expectEqual(@as(defs.RegisterType, 44), reg_file.read(3)); // 200 + 100 = 300 (overflow: 44)

    // Test 2-byte ALU: 0xC8 0x14 (SUB, i8, R3 = R1 - R2)
    reg_file.write(1, 150);
    reg_file.write(2, 100);
    const alu_2byte = [_]u8{ (defs.PREFIX_OP2 << 4) | 0x8, 0x14 }; // op=1 (sub), adt=4 (i8)
    try executeALU(&vm, &alu_2byte);
    try std.testing.expectEqual(@as(defs.RegisterType, 50), reg_file.read(3)); // 150 - 100 = 50

    // Test 4-byte ALU: 0xD8 0x75 0x45 0x03 (MUL, u16, R1=4, R2=5, R3)
    reg_file.write(4, 0);
    reg_file.write(5, 0);
    const alu_4byte = [_]u8{ (defs.PREFIX_OP4 << 4) | 0x8, 0x75, 0x45, 0x03 }; // op=2 (mul), adt=7 (u16), rs=4, src=5, dst=3
    reg_file.write(4, 4);
    reg_file.write(5, 5);
    try executeALU(&vm, &alu_4byte);
    try std.testing.expectEqual(@as(defs.RegisterType, 20), reg_file.read(3)); // 4 * 5 = 20
    try std.testing.expectEqual(@as(defs.RegisterType, 0), reg_file.read(4)); // High result in R4

    // Test 8-byte ALU LOAD: 0xE8 0x0A 0x03 0x02 0x00 0x03 0x04 0x00
    mode.adt = .u8;
    reg_file.writeALU_MODE_CFG(mode);
    vm.memory[4] = 0xAB;
    const alu_8byte_load = [_]u8{ (defs.PREFIX_OP8 << 4) | 0x8, 0x0A, 0x00, 0x02, 0x00, 0x03, 0x04, 0x00 }; // op=load, adt=u8, rs=2, src=0, dst=3, ofs=4
    reg_file.write(0, 0); // src=0 (base address)
    try executeALU(&vm, &alu_8byte_load);
    try std.testing.expectEqual(@as(defs.RegisterType, 0xAB), reg_file.read(3));

    // Test 8-byte ALU STORE: 0xE8 0x0B 0x03 0x02 0x00 0x03 0x04 0x00
    reg_file.write(2, 0xCD);
    const alu_8byte_store = [_]u8{ (defs.PREFIX_OP8 << 4) | 0x8, 0x0B, 0x03, 0x02, 0x00, 0x03, 0x04, 0x00 }; // op=store, adt=u8, rs=2, src=0, dst=3, ofs=4
    try executeALU(&vm, &alu_8byte_store);
    try std.testing.expectEqual(@as(u8, 0xCD), vm.memory[4]);

    // Test invalid opcode
    const invalid_alu = [_]u8{0x90};
    try std.testing.expectError(error.InvalidOpcode, executeALU(&vm, &invalid_alu));

    // Test invalid length
    const invalid_length = [_]u8{ 0x80, 0x00, 0x00 };
    try std.testing.expectError(error.InvalidInstructionLength, executeALU(&vm, &invalid_length));

    // Test divide by zero
    reg_file.write(1, 100);
    reg_file.write(2, 0);
    const alu_div_zero = [_]u8{0x88}; // op=8 (div)
    try std.testing.expectError(error.DivideByZero, executeALU(&vm, &alu_div_zero));
}
