const std = @import("std");
const defs = @import("definitions.zig");
const regs = @import("registers.zig");
const vm_mod = @import("vm.zig");
const alu_mod = @import("alu.zig");
const builtin = @import("builtin");
const er = @import("errorreader.zig");
const IOPipe = @import("iopipe.zig").IOPipe;
const st = @import("vm_stack_logic.zig");

// Helper function for JMP and CALL
fn executeJumpOrCall(vm: *vm_mod.VM, buffer: []const u8, is_call: bool, expected_opcode: u8) !void {
    const reg_file = &vm.registers;
    const cfg = reg_file.readALU_IO_CFG();
    const branch_ctrl = reg_file.readBRANCH_CTRL();
    var bcs: u8 = branch_ctrl.bcs; // Default to BRANCH_CTRL.BCS
    var offset: i64 = 0; // Large enough for i48
    const ip = reg_file.readIP();

    // Validate instruction and extract bcs and offset
    switch (buffer.len) {
        1 => {
            // 1-byte: 0x4|ofs or 0x5|ofs (ofs is i4)
            if ((buffer[0] >> 4) != expected_opcode) return error.InvalidOpcode;
            const u: u4 = @truncate(buffer[0] & 0x0F);
            const v: i4 = @bitCast(u);
            offset = v;
            if (is_call) {
                offset *= @as(i64, branch_ctrl.st_jmp); // Scale by ST_JMP
            }
        },
        2 => {
            // 2-byte: 0xC 0x4|ofs or 0x5|ofs (bcs and ofs are i4)
            if (buffer[0] != (defs.PREFIX_OP2 << 4 | expected_opcode)) return error.InvalidOpcode;
            bcs = (buffer[1] >> 4) & 0x0F; // Extract bcs (u4)
            const u: u4 = @truncate(buffer[1] & 0x0F);
            const v: i4 = @bitCast(u);
            offset = v;
            if (is_call) {
                offset *= @as(i64, branch_ctrl.st_jmp); // Scale by ST_JMP
            }
        },
        4 => {
            // 4-byte: 0xD 0x4 or 0x5, bcs, ofs (bcs is u8, ofs is i16)
            if (buffer[0] != (defs.PREFIX_OP4 << 4 | expected_opcode)) return error.InvalidOpcode;
            bcs = buffer[1]; // u8
            offset = @as(i16, @bitCast(std.mem.readInt(u16, buffer[2..4], .little)));
        },
        8 => {
            // 8-byte: 0xE 0x4 or 0x5, bcs, ofs (bcs is u8, ofs is i48)
            if (buffer[0] != (defs.PREFIX_OP8 << 4 | expected_opcode)) return error.InvalidOpcode;
            bcs = buffer[1]; // u8
            offset = @as(i48, @bitCast(std.mem.readInt(u48, buffer[2..8], .little) & 0xFFFFFFFFFFFF));
        },
        else => return error.InvalidInstructionLength,
    }

    // Precompute arguments for comparisons
    const arg1: defs.RegisterType = reg_file.read(cfg.rs);
    const arg2: defs.RegisterType = reg_file.read(cfg.src);

    var should_branch = false;

    const mode = reg_file.readALU_MODE_CFG();
    // Evaluate branch condition
    if (mode.adt.signed()) {
        const arg1_signed: defs.RegisterSignedType = @bitCast(arg1);
        const arg2_signed: defs.RegisterSignedType = @bitCast(arg2);
        should_branch = switch (@as(defs.BCS, @enumFromInt(bcs))) {
            .always => true,
            .zero => (reg_file.read(cfg.dst) == 0),
            .not_zero => (reg_file.read(cfg.dst) != 0),
            .greater => (arg1_signed > arg2_signed),
            .greater_or_equal => (arg1_signed >= arg2_signed),
            .less => (arg1_signed < arg2_signed),
            .less_or_equal => (arg1_signed <= arg2_signed),
            .carry, .not_carry, .sign, .not_sign, .overflow, .not_overflow, .parity_even, .parity_odd, .interrupt => {
                // TODO: Implement remaining conditions (using flags or other state)
                return error.UnsupportedBranchCondition;
            },
        };
    } else {
        should_branch = switch (@as(defs.BCS, @enumFromInt(bcs))) {
            .always => true,
            .zero => (reg_file.read(cfg.dst) == 0),
            .not_zero => (reg_file.read(cfg.dst) != 0),
            .greater => (arg1 > arg2),
            .greater_or_equal => (arg1 >= arg2),
            .less => (arg1 < arg2),
            .less_or_equal => (arg1 <= arg2),
            .carry, .not_carry, .sign, .not_sign, .overflow, .not_overflow, .parity_even, .parity_odd, .interrupt => {
                // TODO: Implement remaining conditions (using flags or other state)
                return error.UnsupportedBranchCondition;
            },
        };
    }

    if (should_branch) {
        // For CALL, push next instruction address (current IP + instruction size)
        if (is_call) {
            const next_ip = ip + buffer.len;
            const push_buffer = [_]u8{ (defs.PREFIX_OP2 << 4) | 0x6, defs.R_IP }; // PUSH R255
            reg_file.write(defs.R_IP, next_ip); // Temporarily set IP to next instruction
            try st.executePUSH(vm, &push_buffer);
            reg_file.write(defs.R_IP, ip); // Restore original IP for jump calculation
        }

        // Calculate new IP
        const new_ip = @as(i64, @bitCast(ip)) + offset;
        if (new_ip < 0 or new_ip >= vm.memory.len) return error.InvalidJumpAddress;

        // Update IP
        reg_file.writeIP(@as(defs.RegisterType, @bitCast(new_ip)));
    }
}

// Execute JMP instruction
pub fn executeJMP(vm: *vm_mod.VM, buffer: []const u8) !void {
    try executeJumpOrCall(vm, buffer, false, 0x4);
}

// Execute CALL instruction
pub fn executeCALL(vm: *vm_mod.VM, buffer: []const u8) !void {
    try executeJumpOrCall(vm, buffer, true, 0x5);
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
    try st.executePOP(vm, &pop_buffer);

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

test "JMP and CALL instructions" {
    const allocator = std.testing.allocator;

    // Initialize VM with 1024-byte memory
    var vm = try vm_mod.VM.init(allocator, null, null, null, 0xFFFF);
    defer vm.deinit();

    var reg_file = &vm.registers;
    var cfg = reg_file.readALU_IO_CFG();
    var mode = reg_file.readALU_MODE_CFG();
    var branch_ctrl = reg_file.readBRANCH_CTRL();

    // Set initial conditions
    mode.adt = defs.ADT.i32;
    reg_file.writeALU_MODE_CFG(mode);
    cfg.rs = 1;
    cfg.src = 2;
    cfg.dst = 3;
    reg_file.writeALU_IO_CFG(cfg);
    reg_file.writeSP(@as(defs.RegisterType, @truncate(vm.memory.len))); // SP at top
    reg_file.writeIP(0); // Start at IP=0

    // Test 1-byte JMP: 0x4|ofs (ofs=2, i4)
    {
        const jmp_1byte = [_]u8{0x42}; // ofs=2
        branch_ctrl.bcs = @intFromEnum(defs.BCS.always);
        reg_file.writeBRANCH_CTRL(branch_ctrl);
        try executeJMP(&vm, &jmp_1byte);
        try std.testing.expectEqual(2, reg_file.readIP());
    }

    // Test 2-byte JMP: 0xC4 bcs|ofs (bcs=zero, ofs=3)
    {
        reg_file.writeIP(10);
        reg_file.write(cfg.dst, 0); // Set DST to zero
        const jmp_2byte = [_]u8{ (defs.PREFIX_OP2 << 4) | 0x4, 0x13 }; // bcs=1 (zero), ofs=3
        try executeJMP(&vm, &jmp_2byte);
        try std.testing.expectEqual(13, reg_file.readIP());
    }

    // Test 4-byte JMP: 0xD4 bcs ofs (bcs=not_zero, ofs=-10)
    {
        reg_file.writeIP(20);
        reg_file.write(cfg.dst, 42); // Non-zero
        const jmp_4byte = [_]u8{ (defs.PREFIX_OP4 << 4) | 0x4, 0x2, 0xF6, 0xFF }; // bcs=2 (not_zero), ofs=-10
        try executeJMP(&vm, &jmp_4byte);
        try std.testing.expectEqual(10, reg_file.readIP());
    }

    // Test 8-byte JMP: 0xE4 bcs ofs (bcs=always, ofs=100)
    {
        reg_file.writeIP(50);
        const jmp_8byte = [_]u8{ (defs.PREFIX_OP8 << 4) | 0x4, 0x0, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00 }; // bcs=0, ofs=100
        try executeJMP(&vm, &jmp_8byte);
        try std.testing.expectEqual(150, reg_file.readIP());
    }

    // Test 1-byte CALL: 0x5|ofs (ofs=4, i4)
    {
        reg_file.writeIP(0);
        reg_file.writeSP(vm.memory.len);
        branch_ctrl.bcs = @intFromEnum(defs.BCS.always);
        branch_ctrl.st_jmp = 2; // Scale offset by 2
        reg_file.writeBRANCH_CTRL(branch_ctrl);
        const call_1byte = [_]u8{0x54}; // ofs=4
        try executeCALL(&vm, &call_1byte);
        try std.testing.expectEqual(8, reg_file.readIP()); // ofs=4 * st_jmp=2
        try std.testing.expectEqual(vm.memory.len - @sizeOf(defs.PointerRegisterType), reg_file.readSP());
        try std.testing.expectEqual(1, std.mem.readInt(defs.PointerRegisterType, @ptrCast(vm.memory[vm.memory.len - @sizeOf(defs.PointerRegisterType) ..].ptr), .little));
    }

    // Test 2-byte CALL: 0xC5 bcs|ofs (bcs=greater, ofs=2)
    {
        reg_file.writeIP(10);
        reg_file.writeSP(vm.memory.len);
        reg_file.write(cfg.rs, 10);
        reg_file.write(cfg.src, 5);
        branch_ctrl.st_jmp = 1; // Scale offset by 1
        reg_file.writeBRANCH_CTRL(branch_ctrl);
        const call_2byte = [_]u8{ (defs.PREFIX_OP2 << 4) | 0x5, 0x32 }; // bcs=3 (greater), ofs=2
        try executeCALL(&vm, &call_2byte);
        try std.testing.expectEqual(12, reg_file.readIP());
        try std.testing.expectEqual(vm.memory.len - @sizeOf(defs.PointerRegisterType), reg_file.readSP());
        try std.testing.expectEqual(12, std.mem.readInt(defs.PointerRegisterType, @ptrCast(vm.memory[vm.memory.len - @sizeOf(defs.PointerRegisterType) ..].ptr), .little));
    }

    // Test 4-byte CALL: 0xD5 bcs ofs (bcs=always, ofs=50)
    {
        reg_file.writeIP(100);
        reg_file.writeSP(vm.memory.len);
        const call_4byte = [_]u8{ (defs.PREFIX_OP4 << 4) | 0x5, 0x0, 0x32, 0x00 }; // bcs=0, ofs=50
        try executeCALL(&vm, &call_4byte);
        try std.testing.expectEqual(150, reg_file.readIP());
        try std.testing.expectEqual(vm.memory.len - @sizeOf(defs.PointerRegisterType), reg_file.readSP());
        try std.testing.expectEqual(104, std.mem.readInt(defs.PointerRegisterType, @ptrCast(vm.memory[vm.memory.len - @sizeOf(defs.PointerRegisterType) ..].ptr), .little));
    }

    // Test 2-byte CALL: 0xC5 bcs|ofs (bcs=greater, ofs=2)
    {
        mode.adt = defs.ADT.i8;
        reg_file.writeALU_MODE_CFG(mode);
        reg_file.writeIP(10);
        reg_file.writeSP(vm.memory.len);
        reg_file.write(cfg.rs, 10);
        reg_file.write(cfg.src, 0xFF);
        var v: defs.RegisterType = 0;
        v = ~v;
        try std.testing.expectEqual(v, reg_file.read(cfg.src));
        branch_ctrl.st_jmp = 1; // Scale offset by 1
        reg_file.writeBRANCH_CTRL(branch_ctrl);
        const call_2byte = [_]u8{ (defs.PREFIX_OP2 << 4) | 0x5, 0x32 }; // bcs=3 (greater), ofs=2
        try executeCALL(&vm, &call_2byte);
        try std.testing.expectEqual(12, reg_file.readIP());
        try std.testing.expectEqual(vm.memory.len - @sizeOf(defs.PointerRegisterType), reg_file.readSP());
        try std.testing.expectEqual(12, std.mem.readInt(defs.PointerRegisterType, @ptrCast(vm.memory[vm.memory.len - @sizeOf(defs.PointerRegisterType) ..].ptr), .little));
    }

    // Test invalid JMP opcode
    {
        const invalid_jmp = [_]u8{0x6};
        try std.testing.expectError(error.InvalidOpcode, executeJMP(&vm, &invalid_jmp));
    }

    // Test invalid CALL length
    {
        const invalid_call = [_]u8{ 0x5, 0x0, 0x0 };
        try std.testing.expectError(error.InvalidInstructionLength, executeCALL(&vm, &invalid_call));
    }

    // Test invalid jump address
    {
        reg_file.writeIP(0);
        const jmp_out_of_bounds = [_]u8{ (defs.PREFIX_OP8 << 4) | 0x4, 0x0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }; // ofs=2^48-1
        try std.testing.expectError(error.InvalidJumpAddress, executeJMP(&vm, &jmp_out_of_bounds));
    }

    // Test stack overflow on CALL
    {
        reg_file.writeSP(0); // SP too low
        const call_1byte = [_]u8{0x51};
        try std.testing.expectError(error.StackOverflow, executeCALL(&vm, &call_1byte));
    }
}

test "RET and IRET instructions" {
    const allocator = std.testing.allocator;

    // Initialize VM with 1024-byte memory
    var vm = try vm_mod.VM.init(allocator, null, null, null, 0xFFFF);
    defer vm.deinit();

    var reg_file = &vm.registers;
    const ip_size = @sizeOf(defs.PointerRegisterType); // Size of IP (R255) in bytes

    // Set SP to top of memory
    const initial_sp = @as(defs.PointerRegisterType, @truncate(vm.memory.len));
    reg_file.writeSP(initial_sp);

    // Test 1-byte RET: 0x01
    {
        // Push return address (0x1234)
        const return_addr: defs.PointerRegisterType = 0x1234;
        reg_file.writeSP(initial_sp - ip_size);
        std.mem.writeInt(defs.PointerRegisterType, @ptrCast(vm.memory[initial_sp - ip_size ..][0..ip_size]), return_addr, .little);

        const ret_1byte = [_]u8{0x01};
        try executeRET(&vm, &ret_1byte);
        try std.testing.expectEqual(return_addr, reg_file.readIP());
        try std.testing.expectEqual(initial_sp, reg_file.readSP()); // SP restored after pop
    }

    // Test 2-byte RET: 0xC 0x01
    {
        reg_file.writeSP(initial_sp - ip_size);
        const return_addr: defs.PointerRegisterType = 0x5678;
        std.mem.writeInt(defs.PointerRegisterType, @ptrCast(vm.memory[initial_sp - ip_size ..][0..ip_size]), return_addr, .little);

        const ret_2byte = [_]u8{ defs.PREFIX_OP2, 0x01 };
        try executeRET(&vm, &ret_2byte);
        try std.testing.expectEqual(return_addr, reg_file.readIP());
        try std.testing.expectEqual(initial_sp, reg_file.readSP());
    }

    // Test 4-byte RET: 0xD 0x00 0x01 cnt
    {
        reg_file.writeSP(initial_sp - ip_size);
        const return_addr: defs.PointerRegisterType = 0x9ABC;
        std.mem.writeInt(defs.PointerRegisterType, @ptrCast(vm.memory[initial_sp - ip_size ..][0..ip_size]), return_addr, .little);

        const cnt: u8 = 8; // Simulate popping 8 bytes of parameters
        const ret_4byte = [_]u8{ defs.PREFIX_OP4, 0x00, 0x01, cnt };
        try executeRET(&vm, &ret_4byte);
        try std.testing.expectEqual(return_addr, reg_file.readIP());
        try std.testing.expectEqual(initial_sp + cnt, reg_file.readSP()); // SP += cnt after pop
    }

    // Test 1-byte IRET: 0x02
    {
        reg_file.writeSP(initial_sp - ip_size);
        const return_addr: defs.PointerRegisterType = 0x2345;
        std.mem.writeInt(defs.PointerRegisterType, @ptrCast(vm.memory[initial_sp - ip_size ..][0..ip_size]), return_addr, .little);

        const iret_1byte = [_]u8{0x02};
        try executeIRET(&vm, &iret_1byte);
        try std.testing.expectEqual(return_addr, reg_file.readIP());
        try std.testing.expectEqual(initial_sp, reg_file.readSP());
    }

    // Test 4-byte IRET: 0xD 0x00 0x02 cnt
    {
        reg_file.writeSP(initial_sp - ip_size);
        const return_addr: defs.PointerRegisterType = 0x6789;
        std.mem.writeInt(defs.PointerRegisterType, @ptrCast(vm.memory[initial_sp - ip_size ..][0..ip_size]), return_addr, .little);

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
