const std = @import("std");
const defs = @import("definitions.zig");
const regs = @import("registers.zig");
const VM = @import("vm.zig").VM;
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

pub fn rM(vm: *VM, addr: defs.PointerRegisterType, sz: u8) !defs.RegisterType {
    
    if (addr + sz - 1 >= vm.memory.len) return error.InvalidMemoryAddress;

    return switch (sz) {
        1 => vm.memory[@intCast(addr)],
        2 => std.mem.readInt(u16, vm.memory[@intCast(addr)..][0..2], .little),
        4 => std.mem.readInt(u32, vm.memory[@intCast(addr)..][0..4], .little),
        8 => std.mem.readInt(u64, vm.memory[@intCast(addr)..][0..8], .little),
        else => unreachable,
    };
}

pub fn wM(vm: *VM, addr: defs.PointerRegisterType, sz: u8, value: defs.RegisterType) !void {

    if (addr + sz - 1 >= vm.memory.len) return error.InvalidMemoryAddress;

    switch (sz) {
        1 => vm.memory[addr] = @truncate(value),
        2 => std.mem.writeInt(u16, vm.memory[addr..][0..2], @truncate(value), .little),
        4 => std.mem.writeInt(u32, vm.memory[addr..][0..4], @truncate(value), .little),
        8 => std.mem.writeInt(u64, vm.memory[addr..][0..8], value, .little),
        else => unreachable,
    }
}

fn advance( a: defs.PointerRegisterType, d: defs.RegisterSignedType) defs.PointerRegisterType {
    return if (d > 0) a+@abs(d) else a-@abs(d);
}

pub fn executeALU(vm: *VM, buffer: []const u8) !void {
    const reg_file = &vm.registers;
    var cfg = reg_file.readALU_IO_CFG();
    const mode = reg_file.readALU_MODE_CFG();
    const strides = reg_file.readALU_VR_STRIDES();
    var op: defs.AMOD = undefined;
    var adt: defs.ADT = mode.adt; // Default to current ADT
    var rs: u8 = cfg.arg1;
    var src: u8 = cfg.arg2;
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
            cfg.arg1 = rs;
            cfg.arg2 = src;
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
            cfg.arg1 = rs;
            cfg.arg2 = src;
            cfg.dst = dst;
            reg_file.writeALU_IO_CFG(cfg);
        },
        else => return error.InvalidInstructionLength,
    }

    const byte_size = (adt.bits() + 7) >> 3; // Bytes per element
    const vl = mode.vl; // Vector length

    // Vector mode loop
    var i: u8 = 0;

    if (op == .load) {
        if (vl == 0) {
            reg_file.write(rs, reg_file.read(dst));
            return;
        }
        // R[N.RS] - base PTR; ST_RS - base PTR stride
        // R[N.SRC] - Index register, ST_SRC - size of record, if 0 - index is not used.
        var addr_src: defs.PointerRegisterType = reg_file.read(rs);
        const idx = strides.st_arg2 * @as(i64, @intCast(reg_file.read(src)));
        addr_src = advance(addr_src, idx+ofs);

        var rout: i16 = dst;
        while (i < vl) : (i += 1) {
            const m = try rM(vm, addr_src, byte_size);
            if (strides.st_arg1 > 0) addr_src += @abs(strides.st_arg1) else addr_src -= @abs(strides.st_arg1);
            reg_file.write(@intCast(rout), m);
            rout += mode.st_dst;
        }
        return;
    }

    if (op == .store) {
        if (vl == 0) {
            reg_file.write(dst, reg_file.read(rs));
            return;
        }
        // R[N.RS] - base PTR; ST_RS - base PTR stride
        // R[N.SRC] - Index register, ST_SRC - size of record, if 0 - index is not used.
        var addr_dst: defs.PointerRegisterType = reg_file.read(rs);
        const idx = strides.st_arg2 * @as(i64, @intCast(reg_file.read(src)));
        addr_dst = advance(addr_dst, idx+ofs);

        var rin: i16 = rs;
        var rout: i16 = dst;
        while (i < vl) : (i += 1) {
            const r = reg_file.read(@intCast(rin));
            rin += strides.st_arg1;
            if (mode.st_dst == 0) {
                reg_file.write(@intCast(rout), r);
                rout += mode.st_dst;
            } else {
                try wM(vm, addr_dst, byte_size, r);
                if (mode.st_dst > 0) addr_dst += @abs(mode.st_dst) else addr_dst -= @abs(mode.st_dst);
            }
        }
        return;
    }

    if (op == .lookup) {

        // R[N.RS] - base PTR to sequence; ST_RS - base PTR stride
        var ap: defs.PointerRegisterType = reg_file.read(rs);
        // base sequence length
        const aps = reg_file.read(rs+1);

        const sz = adt.bits() >> 3;

        var j: u32 = 0;
        while (j < aps) : (j += 1) {
            var nf: bool = true;
            var k: u32 = 0;

            // R[N.SRC] - PTR to subsequence to look for, ST_SRC - its stride
            var bp: defs.PointerRegisterType = reg_file.read(src);
            // sub-sequence length
            const bps = reg_file.read(src+1);
            var p: defs.PointerRegisterType = ap;
            while (k < bps) : (k += 1) {
                if (try rM(vm, p, sz) != try rM(vm, bp, sz)) {
                    nf = false;
                    break;
                }
                if (strides.st_arg1 > 0) p += @abs(strides.st_arg1) else p -= @abs(strides.st_arg1);
                if (strides.st_arg2 > 0) bp += @abs(strides.st_arg2) else bp -= @abs(strides.st_arg2);
            }
            if (nf) {
                reg_file.write(dst, ap);
                reg_file.write(dst+1, 0);
                return;
            }
            if (strides.st_arg1 > 0) ap += @abs(strides.st_arg1) else ap -= @abs(strides.st_arg1);
        }

        reg_file.write(dst+1, 1);
        return;
    }

    if (mode.vl == 0) {
        // Perform scalar operation
        const a1 = reg_file.read(rs);
        const a1h = if (op == defs.AMOD.div) reg_file.read(rs) else null;
        const a2 = reg_file.read(src);
        var d = dst;

        // ALU operation
        const alu_result = try callALU(op, adt, a1, a1h, a2, null);
        reg_file.write(d, alu_result.ret);
        d += 1;
        if (alu_result.reth) |rh| {
            reg_file.write(d, rh);
            d += 1;
        }
        if (alu_result.carry_out) |c| {
            reg_file.write(d, c);
            d += 1;
        }

        return;
    }

    var accum: defs.RegisterType = 0; // Accumulator for ST_DST = 0
    var accum_h: defs.RegisterType = 0; // high part of accumulator;

    var a1: defs.PointerRegisterType = reg_file.read(rs);
    var a2: defs.PointerRegisterType = reg_file.read(src);
    var a3: defs.PointerRegisterType = reg_file.read(dst);

    while (i < vl) : (i += 1) {
        // Compute input indices/addresses
        var arg1: defs.RegisterType = undefined;
        var arg2: defs.RegisterType = undefined;

        // First operand (RS)
        if (strides.st_arg1 == 0) {
            // Register mode: R[rs + i]
            const reg_idx = rs + i;
            if (reg_idx >= defs.REGISTER_COUNT) return error.InvalidRegisterIndex;
            arg1 = reg_file.read(reg_idx);
        } else {
            // Memory mode: M[R[rs] + i * .st_arg1 + ofs]
            arg1 = try rM(vm,a1,byte_size);
            a1 = advance(a1,  strides.st_arg1);
        }

        // Second operand (SRC)
        if (strides.st_arg2 == 0) {
            // Register mode: R[src + i]
            const reg_idx = @as(u16, src) + i;
            if (reg_idx >= defs.REGISTER_COUNT) return error.InvalidRegisterIndex;
            arg2 = reg_file.read(@intCast(reg_idx));
        } else {
            // Memory mode: M[R[src] + i * .st_arg2 + ofs]
            arg2 = try rM(vm,a2,byte_size);
            a2 = advance(a2,  strides.st_arg2);
        }

        // Perform operation
        var result: defs.RegisterType = undefined;
        var result_high: ?defs.RegisterType = null;
        var carry: ?u1 = null;

        // ALU operation
        const alu_result = try callALU(op, adt, arg1, null, arg2, null);
        result = alu_result.ret;
        result_high = alu_result.reth;
        carry = alu_result.carry_out;

        // Store result
        if (mode.st_dst == 0) {
            // Accumulator mode: R[dst]
            accum += result;
            if (result_high) |h| accum_h += h;
            if (carry) |c| accum_h += c;
        } else {
            // Memory mode: M[R[dst] + i * st_dst]
            try wM(vm, a3, byte_size, result);
            // if destination has a room for high part of result
            if (result_high) |h| if (mode.st_dst >= byte_size << 1) try wM(vm, a3+byte_size, byte_size, h);
            a3 = advance(a3,  mode.st_dst);
        }
    }

    // Write accumulated result to R[dst] (if ST_DST = 0)
    if (mode.st_dst == 0) {
        reg_file.write(dst, accum);
        reg_file.write(dst + 1, accum_h);
    }
}

test "ALU instruction" {
    const allocator = std.testing.allocator;

    // Initialize VM
    var vm = try VM.init(allocator, null, null, null, 0xFFFF);
    defer vm.deinit();

    var reg_file = &vm.registers;
    var cfg = reg_file.readALU_IO_CFG();
    var mode = reg_file.readALU_MODE_CFG();

    // Setup registers
    cfg.arg1 = 1;
    cfg.arg2 = 2;
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
    var vm = try VM.init(allocator, null, null, null, 0x10000);
    defer vm.deinit();

    var reg_file = &vm.registers;
    var cfg = reg_file.readALU_IO_CFG();
    var mode = reg_file.readALU_MODE_CFG();
    var strides = reg_file.readALU_VR_STRIDES();

    // Setup registers
    cfg.arg1 = 0;
    cfg.arg2 = 4;
    cfg.dst = 8;
    reg_file.writeALU_IO_CFG(cfg);
    mode.adt = .u8;
    mode.vl = 0; // Start with scalar
    mode.st_dst = 0;
    reg_file.writeALU_MODE_CFG(mode);
    strides.st_arg1 = 0;
    strides.st_arg2 = 0;
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

    // Test 2b: Out-of-bounds register access
    mode.vl = 255;
    reg_file.writeALU_MODE_CFG(mode);
    try std.testing.expectError(error.InvalidRegisterIndex, executeALU(&vm, &alu_vec_reg));

    // Test 3: Vector ADD, memory mode (VL = 3, ST_RS = 1, ST_SRC = 1, ST_DST = 1)
    cfg.arg1 = 0;
    cfg.arg2 = 4;
    cfg.dst = 8;
    reg_file.writeALU_IO_CFG(cfg);
    mode.adt = .u8;
    mode.vl = 3;
    mode.st_dst = 1;
    reg_file.writeALU_MODE_CFG(mode);
    strides.st_arg1 = 1;
    strides.st_arg2 = 1;
    reg_file.writeALU_VR_STRIDES(strides);
    reg_file.write(cfg.arg1, 0x1000); // R0: base address for RS
    reg_file.write(cfg.arg2, 0x2000); // R4: base address for SRC
    reg_file.write(cfg.dst, 0x3000); // R8: base address for DST
    vm.memory[0x1000] = 10;
    vm.memory[0x1001] = 20;
    vm.memory[0x1002] = 30;
    vm.memory[0x2000] = 1;
    vm.memory[0x2001] = 2;
    vm.memory[0x2002] = 3;
    try executeALU(&vm, &alu_vec_reg);
    try std.testing.expectEqual(11, vm.memory[0x3000]); // 10 + 1
    try std.testing.expectEqual(22, vm.memory[0x3001]); // 20 + 2
    try std.testing.expectEqual(33, vm.memory[0x3002]); // 30 + 3

    // Test 3b: Out-of-bounds memory access
    mode.vl = 3;
    reg_file.write(0, vm.memory.len); // Beyond memory
    reg_file.writeALU_MODE_CFG(mode);
    strides.st_arg1 = 1;
    reg_file.writeALU_VR_STRIDES(strides);
    try std.testing.expectError(error.InvalidMemoryAddress, executeALU(&vm, &alu_vec_reg));

    // Test 4: Vector LOAD, memory mode (VL = 2, ST_RS = 2, ST_DST = 0)
    // Should load
    //    R[8] = M[R[0] + ST_RS*0 + R[4] + 0]; // 50
    //    R[9] = M[R[0] + ST_RS*1 + R[4] + 0]; // 60
    mode.adt = .u8;
    mode.vl = 2;
    mode.st_dst = 1;
    reg_file.writeALU_MODE_CFG(mode);
    strides.st_arg1 = 2;
    strides.st_arg2 = 0;
    reg_file.writeALU_VR_STRIDES(strides);
    reg_file.write(0, 0x4000); // R0: base address
    reg_file.write(4, 0x0000); // R4: index
    vm.memory[0x4000] = 50;
    vm.memory[0x4002] = 60;
    const alu_vec_load = [_]u8{ (defs.PREFIX_OP8 << 4) | 0x8, 0x0A, 0x00, 0x00, 0x04, 0x08, 0x00, 0x00 };
    try executeALU(&vm, &alu_vec_load);
    try std.testing.expectEqual(@as(defs.RegisterType, 50), reg_file.read(8)); // 50
    try std.testing.expectEqual(@as(defs.RegisterType, 60), reg_file.read(8 + 1)); // 60

    // Test 5: Vector STORE, reverse order (VL = 2, ST_DST = -1)
    mode.vl = 2;

    // R[N.RS]: R1 and R0 will be saved
    reg_file.write(1, 70);
    reg_file.write(0, 80);
    // Inverse direction
    strides.st_arg1 = -1;

    // Saved to PTR R[N.DST]+R[N.SRC]
    reg_file.write(8, 0x100); // R8: base address
    reg_file.write(4, 0); // R4: offset
    // inverse direction
    mode.st_dst = -1;

    vm.memory[0x0FF] = 80;
    vm.memory[0x100] = 70;

    reg_file.writeALU_MODE_CFG(mode);
    reg_file.writeALU_VR_STRIDES(strides);

    const alu_vec_store = [_]u8{ (defs.PREFIX_OP8 << 4) | 0x8, 0x0B, 0x00, 0x01, 0x04, 0x08, 0x00, 0x00 };
    try executeALU(&vm, &alu_vec_store);
    try std.testing.expectEqual(@as(u8, 80), vm.memory[0x0FF]);
    try std.testing.expectEqual(@as(u8, 70), vm.memory[0x100]);
}

test "ALU lookup forward" {
    const allocator = std.testing.allocator;

    // Initialize VM
    var vm = try VM.init(allocator, null, null, null, 0x10000);
    defer vm.deinit();

    var reg_file = &vm.registers;
    var cfg = reg_file.readALU_IO_CFG();
    var mode = reg_file.readALU_MODE_CFG();
    var strides = reg_file.readALU_VR_STRIDES();

    // Setup registers
    cfg.arg1 = 0;
    cfg.arg2 = 4;
    cfg.dst = 8;

    mode.adt = .u8;
    mode.vl = 1; // Start with scalar
    mode.st_dst = 0;

    // ptr to string
    reg_file.write(cfg.arg1, 0);
    strides.st_arg1 = 1; // move forward by 1

    // ptr to substring
    reg_file.write(cfg.arg2, 10);
    strides.st_arg2 = 1; // move forward by 1

    // length of string
    reg_file.write(cfg.arg1+1, 8);
    // length of substring
    reg_file.write(cfg.arg2+1, 3);

    for (0..8) |i| {
        vm.memory[i] = @truncate('A' + i);
    }
    // substring which exists
    for (0..3) |i| {
        vm.memory[10 + i] = @truncate(i + 'C');
    }
    // substring which doesn't exists
    for (0..3) |i| {
        vm.memory[20 + i] = @truncate(i + 'O');
    }

    reg_file.writeALU_IO_CFG(cfg);
    reg_file.writeALU_MODE_CFG(mode);
    reg_file.writeALU_VR_STRIDES(strides);

    // test for positive
    const alu_lookup = [_]u8{0x89}; //, 0x09, 0x00, 0x00, 0x04, 0x08, 0x00, 0x00 };
    try executeALU(&vm, &alu_lookup);
    try std.testing.expectEqual(2, reg_file.read(cfg.dst));
    try std.testing.expectEqual(0, reg_file.read(cfg.dst+1));

    // test for negative
    reg_file.write(cfg.arg2, 20);
    // test for positive
    try executeALU(&vm, &alu_lookup);
    try std.testing.expectEqual(1, reg_file.read(cfg.dst+1));
}

test "ALU lookup reverse" {
    const allocator = std.testing.allocator;

    // Initialize VM
    var vm = try VM.init(allocator, null, null, null, 0x10000);
    defer vm.deinit();

    var reg_file = &vm.registers;
    var cfg = reg_file.readALU_IO_CFG();
    var mode = reg_file.readALU_MODE_CFG();
    var strides = reg_file.readALU_VR_STRIDES();

    // Setup registers
    cfg.arg1 = 0;
    cfg.arg2 = 4;
    cfg.dst = 8;

    mode.adt = .u8;
    mode.vl = 1; // Start with scalar
    mode.st_dst = 0;

    // ptr to string
    reg_file.write(cfg.arg1, 7);
    strides.st_arg1 = -1; // move reverse by 1

    // ptr to substring
    reg_file.write(cfg.arg2, 10);
    strides.st_arg2 = 1; // move forward by 1

    // length of string
    reg_file.write(cfg.arg1+1, 8);
    // length of substring
    reg_file.write(cfg.arg2+1, 3);

    for (0..8) |i| {
        vm.memory[i] = @truncate('H' - i);
    }
    for (0..3) |i| {
        vm.memory[10 + i] = @truncate(i + 'C');
    }

    reg_file.writeALU_IO_CFG(cfg);
    reg_file.writeALU_MODE_CFG(mode);
    reg_file.writeALU_VR_STRIDES(strides);

    const alu_lookup = [_]u8{0x89}; //, 0x09, 0x00, 0x00, 0x04, 0x08, 0x00, 0x00 };
    try executeALU(&vm, &alu_lookup);
    try std.testing.expectEqual(5, reg_file.read(cfg.dst));
    try std.testing.expectEqual(0, reg_file.read(cfg.dst+1));
}
