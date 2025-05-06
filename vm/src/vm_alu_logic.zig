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

pub fn executeALU(vm: *vm_mod.VM, buffer: []const u8) !void {
    const reg_file = &vm.registers;
    var cfg = reg_file.readALU_IO_CFG();
    const mode = reg_file.readALU_MODE_CFG();
    const strides = reg_file.readALU_VR_STRIDES();
    var op: defs.AMOD = undefined;
    var adt: defs.ADT = mode.adt; // Default to current ADT
    var rs: u8 = cfg.rs;
    var src: u8 = cfg.src;
    var dst: u8 = cfg.dst;
    var ofs: i16 = 0;

    // Decode instruction (unchanged)
    switch (buffer.len) {
        1 => {
            if ((buffer[0] >> 4) != 0x8) return error.InvalidOpcode;
            op = @enumFromInt(buffer[0] & 0x0F);
        },
        2 => {
            if (buffer[0] != ((defs.PREFIX_OP2 << 4) | 0x8)) return error.InvalidOpcode;
            op = @enumFromInt(buffer[1] >> 4);
            adt = @enumFromInt(buffer[1] & 0x0F);
        },
        4 => {
            if (buffer[0] != ((defs.PREFIX_OP4 << 4) | 0x8)) return error.InvalidOpcode;
            op = @enumFromInt(buffer[1] >> 4);
            adt = @enumFromInt(buffer[1] & 0x0F);
            rs = buffer[2] >> 4;
            src = buffer[2] & 0x0F;
            dst = buffer[3];
            cfg.rs = rs;
            cfg.src = src;
            cfg.dst = dst;
            reg_file.writeALU_IO_CFG(cfg);
        },
        8 => {
            if (buffer[0] != ((defs.PREFIX_OP8 << 4) | 0x8)) return error.InvalidOpcode;
            op = @enumFromInt(buffer[1]);
            adt = @enumFromInt(buffer[2]);
            rs = buffer[3];
            src = buffer[4];
            dst = buffer[5];
            ofs = std.mem.readInt(i16, buffer[6..8], .little);
            cfg.rs = rs;
            cfg.src = src;
            cfg.dst = dst;
            reg_file.writeALU_IO_CFG(cfg);
        },
        else => return error.InvalidInstructionLength,
    }

    if (mode.vl == 0) {
        // Perform scalar operation
        const a1 = reg_file.read(rs);
        const a1h = if (op == defs.AMOD.div) reg_file.read(rs) else null;
        const a2 = reg_file.read(src);
        var d = dst;

        // ALU operation
        const alu_result = try callALU(op, adt, a1, a1h, a2, null);
        reg_file.write(d, alu_result.ret); d+=1;
        if (alu_result.reth)|rh| {reg_file.write(d, rh); d+=1;}
        if (alu_result.carry_out)|c| {reg_file.write(d, c); d+=1;}
        
        return;
    }

    const a1d = if (strides.st_rs == 0) mode.vl else 0;
    const a2d = if (strides.st_src == 0) mode.vl else 0;
    const rd = if (mode.st_dst == 0) mode.vl else 0;
    
    // Validate registers
    if ((rs+a1d) >= defs.REGISTER_COUNT or 
        (src+a2d) >= defs.REGISTER_COUNT or 
        (dst+rd) >= defs.REGISTER_COUNT) {

        return error.InvalidRegisterIndex;
    }

    const byte_size = (adt.bits() + 7) >> 3; // Bytes per element
    const vl = mode.vl; // Vector length

    // Vector mode loop (includes scalar mode when vl = 0)
    var i: u8 = 0;
    var accum: defs.RegisterType = 0; // Accumulator for ST_DST = 0
    while (i < vl) : (i += 1) {
        // Compute input indices/addresses
        var arg1: defs.RegisterType = undefined;
        var arg2: defs.RegisterType = undefined;

        // First operand (RS)
        if (strides.st_rs == 0) {
            // Register mode: R[rs + i]
            const reg_idx = rs + i;
            if (reg_idx >= defs.REGISTER_COUNT) return error.InvalidRegisterIndex;
            arg1 = reg_file.read(reg_idx);
        } else {
            // Memory mode: M[R[rs] + i * st_rs + ofs]
            const base_addr = reg_file.read(rs);
            if (base_addr > std.math.maxInt(i64)) return error.InvalidMemoryAddress; // Base too large
            var addr: i64 = @as(i64, @intCast(base_addr));
            addr += @as(i64, i) * @as(i64, strides.st_rs);
            if (ofs != 0) addr += @as(i64, ofs);
            if (addr < 0 or addr >= vm.memory.len or addr + byte_size > vm.memory.len) {
                return error.InvalidMemoryAddress;
            }
            arg1 = switch (byte_size) {
                1 => vm.memory[@intCast(addr)],
                2 => std.mem.readInt(u16, vm.memory[@intCast(addr)..][0..2], .little),
                4 => std.mem.readInt(u32, vm.memory[@intCast(addr)..][0..4], .little),
                8 => std.mem.readInt(u64, vm.memory[@intCast(addr)..][0..8], .little),
                else => return error.InvalidDataType,
            };
        }

        // Second operand (SRC)
        if (op != .load) { // LOAD uses only RS
            if (strides.st_src == 0) {
                // Register mode: R[src + i]
                const reg_idx = src + i;
                if (reg_idx >= defs.REGISTER_COUNT) return error.InvalidRegisterIndex;
                arg2 = reg_file.read(reg_idx);
            } else {
                // Memory mode: M[R[src] + i * st_src + ofs]
                const base_addr = reg_file.read(src);
                if (base_addr > std.math.maxInt(i64)) return error.InvalidMemoryAddress; // Base too large
                var addr: i64 = @as(i64, @intCast(base_addr));
                addr += @as(i64, i) * @as(i64, strides.st_src);
                if (ofs != 0) addr += @as(i64, ofs);
                if (addr < 0 or addr >= vm.memory.len or addr + byte_size > vm.memory.len) {
                    return error.InvalidMemoryAddress;
                }
                arg2 = switch (byte_size) {
                    1 => vm.memory[@intCast(addr)],
                    2 => std.mem.readInt(u16, vm.memory[@intCast(addr)..][0..2], .little),
                    4 => std.mem.readInt(u32, vm.memory[@intCast(addr)..][0..4], .little),
                    8 => std.mem.readInt(u64, vm.memory[@intCast(addr)..][0..8], .little),
                    else => return error.InvalidDataType,
                };
            }
        }

        // Perform operation
        var result: defs.RegisterType = undefined;
        var result_high: ?defs.RegisterType = null;

        if (op == .load) {
            result = arg1; // LOAD result is the fetched value
        } else if (op == .store) {
            // STORE handled in destination logic
            result = arg1; // Value to store
        } else {
            // ALU operation
            const alu_result = try callALU(op, adt, arg1, null, arg2, null);
            result = alu_result.ret;
            result_high = alu_result.reth;
        }

        // Store result
        if (mode.st_dst == 0) {
            // Accumulator mode: R[dst]
            if (i == 0) {
                accum = result; // Initialize
            } else if (op != .store) {
                // Aggregate (e.g., sum for ADD, AND for AND)
                const accum_result = try callALU(op, adt, accum, null, result, null);
                accum = accum_result.ret;
            }
            // Write final result after loop
        } else {
            // Memory mode: M[R[dst] + i * st_dst + ofs]
            const base_addr = reg_file.read(dst);
            if (base_addr > std.math.maxInt(i64)) return error.InvalidMemoryAddress; // Base too large
            var addr: i64 = @as(i64, @intCast(base_addr));
            addr += @as(i64, i) * @as(i64, mode.st_dst);
            if (ofs != 0) addr += @as(i64, ofs);
            if (addr < 0 or addr >= vm.memory.len or addr + byte_size > vm.memory.len) {
                return error.InvalidMemoryAddress;
            }
            switch (byte_size) {
                1 => vm.memory[@intCast(addr)] = @truncate(result),
                2 => std.mem.writeInt(u16, vm.memory[@intCast(addr)..][0..2], @truncate(result), .little),
                4 => std.mem.writeInt(u32, vm.memory[@intCast(addr)..][0..4], @truncate(result), .little),
                8 => std.mem.writeInt(u64, vm.memory[@intCast(addr)..][0..8], @truncate(result), .little),
                else => return error.InvalidDataType,
            }
        }

        // Handle high result for multiplication
        if (op == .mul and result_high != null and mode.st_dst == 0) {
            const high_dst = dst + 1;
            if (i == vl and high_dst < defs.REGISTER_COUNT) {
                reg_file.write(high_dst, result_high.?);
            }
        }
    }

    // Write accumulated result to R[dst] (if ST_DST = 0)
    if (mode.st_dst == 0 and op != .store) {
        reg_file.write(dst, accum);
    }
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

    if (false) {
        // Test 8-byte ALU LOAD: 0xE8 0x0A 0x03 0x02 0x00 0x03 0x04 0x00
        mode.adt = .u8;
        mode.vl = 0;
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
    }

    // Test invalid opcode
    const invalid_alu = [_]u8{0x90};
    try std.testing.expectError(error.InvalidOpcode, executeALU(&vm, &invalid_alu));

    // Test invalid length
    const invalid_length = [_]u8{ 0x80, 0x00, 0x00 };
    try std.testing.expectError(error.InvalidInstructionLength, executeALU(&vm, &invalid_length));

    if (false) {
        // Test divide by zero
        reg_file.write(1, 100);
        reg_file.write(2, 0);
        const alu_div_zero = [_]u8{0x88}; // op=8 (div)
        try std.testing.expectError(error.DivideByZero, executeALU(&vm, &alu_div_zero));
    }
}

test "ALU vector mode" {
    const allocator = std.testing.allocator;

    // Initialize VM
    var vm = try vm_mod.VM.init(allocator, null, null, null, 0x10000);
    defer vm.deinit();

    var reg_file = &vm.registers;
    var cfg = reg_file.readALU_IO_CFG();
    var mode = reg_file.readALU_MODE_CFG();
    var strides = reg_file.readALU_VR_STRIDES();

    // Setup registers
    cfg.rs = 0;
    cfg.src = 4;
    cfg.dst = 8;
    reg_file.writeALU_IO_CFG(cfg);
    mode.adt = .u8;
    mode.vl = 0; // Start with scalar
    mode.st_dst = 0;
    reg_file.writeALU_MODE_CFG(mode);
    strides.st_rs = 0;
    strides.st_src = 0;
    reg_file.writeALU_VR_STRIDES(strides);

    // Test 1: Scalar ADD (VL = 0)
    reg_file.write(0, 200);
    reg_file.write(4, 100);
    const alu_1byte = [_]u8{0x80};
    try executeALU(&vm, &alu_1byte);
    try std.testing.expectEqual(@as(defs.RegisterType, 44), reg_file.read(8)); // 200 + 100 = 300 (u8 overflow: 44)

    // Test 2: Vector ADD, register mode (VL = 3, ST_RS = ST_SRC = 0)
    mode.adt = .u8;
    mode.vl = 4;
    reg_file.writeALU_MODE_CFG(mode);
    reg_file.write(0, 10);
    reg_file.write(1, 20);
    reg_file.write(2, 30);
    reg_file.write(3, 40);
    reg_file.write(4, 1);
    reg_file.write(5, 2);
    reg_file.write(6, 3);
    reg_file.write(7, 4);
    const alu_vec_reg = [_]u8{0x80}; // ADD
    try executeALU(&vm, &alu_vec_reg);
    try std.testing.expectEqual(@as(defs.RegisterType, 110), reg_file.read(8));

    // Test 3: Vector ADD, memory mode (VL = 3, ST_RS = 1, ST_SRC = 1, ST_DST = 1)
    mode.adt = .u8;
    mode.vl = 3;
    mode.st_dst = 1;
    reg_file.writeALU_MODE_CFG(mode);
    strides.st_rs = 1;
    strides.st_src = 1;
    reg_file.writeALU_VR_STRIDES(strides);
    reg_file.write(0, 0x1000); // R0: base address for RS
    reg_file.write(4, 0x2000); // R4: base address for SRC
    reg_file.write(8, 0x3000); // R8: base address for DST
    vm.memory[0x1000] = 10;
    vm.memory[0x1001] = 20;
    vm.memory[0x1002] = 30;
    vm.memory[0x2000] = 1;
    vm.memory[0x2001] = 2;
    vm.memory[0x2002] = 3;
    try executeALU(&vm, &alu_vec_reg);
    try std.testing.expectEqual(@as(u8, 11), vm.memory[0x3000]); // 10 + 1
    try std.testing.expectEqual(@as(u8, 22), vm.memory[0x3001]); // 20 + 2
    try std.testing.expectEqual(@as(u8, 33), vm.memory[0x3002]); // 30 + 3

    // Test 4: Vector LOAD, memory mode (VL = 2, ST_RS = 2, ST_DST = 0)
    mode.vl = 2;
    mode.st_dst = 0;
    reg_file.writeALU_MODE_CFG(mode);
    strides.st_rs = 2;
    strides.st_src = 0;
    reg_file.writeALU_VR_STRIDES(strides);
    reg_file.write(1, 0x4000); // R1: base address
    vm.memory[0x4000] = 50;
    vm.memory[0x4002] = 60;
    const alu_vec_load = [_]u8{ (defs.PREFIX_OP8 << 4) | 0x8, 0x0A, 0x00, 0x01, 0x00, 0x03, 0x00, 0x00 };
    try executeALU(&vm, &alu_vec_load);
    try std.testing.expectEqual(@as(defs.RegisterType, 110), reg_file.read(3)); // 50 + 60

    // Test 5: Vector STORE, reverse order (VL = 2, ST_RS = -1, ST_DST = -1)
    mode.vl = 2;
    mode.st_dst = -1;
    reg_file.writeALU_MODE_CFG(mode);
    strides.st_rs = -1;
    reg_file.writeALU_VR_STRIDES(strides);
    reg_file.write(1, 0x5002); // R1: base address (high)
    reg_file.write(3, 0x6002); // R3: base address (high)
    vm.memory[0x5002] = 70;
    vm.memory[0x5001] = 80;
    const alu_vec_store = [_]u8{ (defs.PREFIX_OP8 << 4) | 0x8, 0x0B, 0x00, 0x01, 0x00, 0x03, 0x00, 0x00 };
    try executeALU(&vm, &alu_vec_store);
    try std.testing.expectEqual(@as(u8, 70), vm.memory[0x6002]);
    try std.testing.expectEqual(@as(u8, 80), vm.memory[0x6001]);

    // Test 6: Out-of-bounds register access
    mode.vl = 255;
    reg_file.writeALU_MODE_CFG(mode);
    try std.testing.expectError(error.InvalidRegisterIndex, executeALU(&vm, &alu_vec_reg));

    // Test 7: Out-of-bounds memory access
    mode.vl = 3;
    reg_file.write(1, vm.memory.len); // Beyond memory
    reg_file.writeALU_MODE_CFG(mode);
    strides.st_rs = 1;
    reg_file.writeALU_VR_STRIDES(strides);
    try std.testing.expectError(error.InvalidMemoryAddress, executeALU(&vm, &alu_vec_reg));
}
