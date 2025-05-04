// register_logic.zig
const std = @import("std");
const defs = @import("definitions.zig");
const regs = @import("registers.zig");
const vm_mod = @import("vm.zig");
const alu_mod = @import("alu.zig");
const builtin = @import("builtin");
const er = @import("errorreader.zig");

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

// Helper function for INC and DEC instructions
fn executeIncDec(reg_file: *regs.RegisterFile, buffer: []const u8, is_increment: bool) !void {
    const cfg = reg_file.readALU_IO_CFG();
    const mode = reg_file.readALU_MODE_CFG();
    var reg_index: u8 = cfg.rs;
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
            try executePUSH(vm, &push_buffer);
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

// In register_logic.zig, append to existing code

// Execute NOT instruction
pub fn executeNOT(reg_file: *regs.RegisterFile, buffer: []const u8) !void {
    const cfg = reg_file.readALU_IO_CFG();
    const mode = reg_file.readALU_MODE_CFG();
    var reg_index: u16 = cfg.rs; // Default to N.RS for 1-byte form
    var count: u8 = 1; // Default to 1 register

    switch (buffer.len) {
        1 => {
            // 1-byte: 0x05 (invert R[N.RS])
            if (buffer[0] != 0x05) return error.InvalidOpcode;
            // reg_index already set to cfg.rs
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

fn decimalDigitsForNBits(N: u32) u8 {
    const one: u64 = 1;
    const max_value = (one << @truncate(N)) - 1;
    var n: u64 = max_value;
    var digits: u32 = 1;
    while (n >= 10) : (digits += 1) {
        n /= 10;
    }
    return @intCast(digits);
}

fn format(alloc: std.mem.Allocator, arg: defs.RegisterType, adt: defs.ADT, ofmt: defs.OUT_FMT) ![]const u8 {
    const fmt_type = ofmt.fmt;
    const leading_zeros = ofmt.zero_pad != 0;
    const bs = adt.bits();

    const one: defs.RegisterType = 1;
    var mask: defs.RegisterType = (one << @truncate(bs)) - 1;

    if (mask == 0) mask = ~mask; // Handle u64 overflow

    const masked_value: defs.RegisterType = @truncate(arg & mask);
    var signed_value: defs.RegisterSignedType = undefined;

    const sign_bit_mask = one << @truncate(bs - 1);
    if ((masked_value & sign_bit_mask) != 0) {
        signed_value = @bitCast(masked_value | ~mask); // Sign extend
    } else {
        signed_value = @bitCast(masked_value);
    }

    // Buffer size: accommodate worst case (64-bit binary or decimal with leading zeros)
    var buf: [128]u8 = undefined;

    return switch (fmt_type) {
        .raw => blk: {
            std.mem.writeInt(defs.RegisterType, @ptrCast(&buf), masked_value, .little);
            break :blk try alloc.dupe(u8, buf[0 .. bs << 3]);
        },
        .hex => blk: {
            if (leading_zeros) {
                const width = (bs + 3) / 4; // Ceiling of bits/4
                const str = try std.fmt.bufPrint(&buf, "{x:0>[width]}", .{ .value = masked_value, .width = width });
                break :blk try alloc.dupe(u8, str);
            } else {
                const str = try std.fmt.bufPrint(&buf, "{x}", .{masked_value});
                break :blk try alloc.dupe(u8, str);
            }
        },
        .dec => blk: {
            const width = decimalDigitsForNBits(bs);
            if (adt.signed()) {
                const abs_value = @abs(signed_value);
                if (leading_zeros) {
                    if (signed_value < 0) {
                        // Format absolute value with one less width, then prepend '-'
                        const str = try std.fmt.bufPrint(&buf, "-{d:0>[width]}", .{ .value = abs_value, .width = width });
                        break :blk try alloc.dupe(u8, str);
                    } else {
                        const str = try std.fmt.bufPrint(&buf, "{d:0>[width]}", .{ .value = abs_value, .width = width });
                        break :blk try alloc.dupe(u8, str);
                    }
                } else {
                    const str = try std.fmt.bufPrint(&buf, "{d}", .{signed_value});
                    break :blk try alloc.dupe(u8, str);
                }
            } else {
                if (leading_zeros) {
                    const str = try std.fmt.bufPrint(&buf, "{d:0>[width]}", .{ .value = masked_value, .width = width });
                    break :blk try alloc.dupe(u8, str);
                } else {
                    const str = try std.fmt.bufPrint(&buf, "{d}", .{masked_value});
                    break :blk try alloc.dupe(u8, str);
                }
            }
        },
        .bin => blk: {
            if (leading_zeros) {
                const str = try std.fmt.bufPrint(&buf, "{b:0>[width]}", .{ .value = masked_value, .width = bs });
                break :blk try alloc.dupe(u8, str);
            } else {
                const str = try std.fmt.bufPrint(&buf, "{b}", .{masked_value});
                break :blk try alloc.dupe(u8, str);
            }
        },
        .fp0, .fp2, .fp4 => blk: {
            const vf: f64 = switch (bs) {
                16 => blk2: {
                    const vu: u16 = @truncate(masked_value);
                    const f16_val: f16 = @bitCast(vu);
                    break :blk2 @floatCast(f16_val);
                },
                32 => blk2: {
                    const vu: u32 = @truncate(masked_value);
                    const f32_val: f32 = @bitCast(vu);
                    break :blk2 @floatCast(f32_val);
                },
                64 => blk2: {
                    const vu: u64 = @truncate(masked_value);
                    const f64_val: f64 = @bitCast(vu);
                    break :blk2 f64_val;
                },
                else => return error.InvalidFloatBits,
            };
            switch (fmt_type) {
                .fp0 => {
                    const str = try std.fmt.bufPrint(&buf, "{d:.0}", .{vf});
                    break :blk try alloc.dupe(u8, str);
                },
                .fp2 => {
                    const str = try std.fmt.bufPrint(&buf, "{d:.2}", .{vf});
                    break :blk try alloc.dupe(u8, str);
                },
                .fp4 => {
                    const str = try std.fmt.bufPrint(&buf, "{d:.4}", .{vf});
                    break :blk try alloc.dupe(u8, str);
                },
                else => unreachable,
            }
        },
    };
}

// Execute OUT instruction
pub fn executeOUT(vm: *vm_mod.VM, buffer: []const u8) !void {
    const reg_file = &vm.registers;
    const mode = reg_file.readALU_MODE_CFG();
    var channel: u8 = 0;
    var reg_index: u8 = reg_file.readALU_IO_CFG().rs; // Default to N.RS
    const reg_index_fmt: u8 = reg_file.readALU_IO_CFG().src; // Default to N.RS
    var adt: defs.ADT = mode.adt; // Default to current ADT
    var fmt: defs.OUT_FMT = .{ .fmt = .raw, .zero_pad = 0 }; // Default format
    var instruction_valid = false;

    const rfmt = reg_file.read(reg_index_fmt);
    fmt.fmt = @enumFromInt(rfmt & 0x7);
    fmt.zero_pad = @truncate(rfmt >> 4);

    // Parse instruction based on length
    switch (buffer.len) {
        1 => {
            // 1-byte: 0xB|ch (ch is u4)
            if ((buffer[0] >> 4) == 0xB) {
                channel = buffer[0] & 0x0F; // Extract u4
                instruction_valid = true;
            }
        },
        2 => {
            // 2-byte: 0xCB, ch (ch is u8)
            if (buffer[0] == ((defs.PREFIX_OP2 << 4) | 0xB)) {
                channel = buffer[1]; // u8
                instruction_valid = true;
            }
        },
        4 => {
            // 4-byte: 0xDB, ch, reg|adt|fmt (ch, reg are u8, adt, fmt are u4)
            if (buffer[0] == ((defs.PREFIX_OP4 << 4) | 0xB)) {
                channel = buffer[1]; // u8
                reg_index = buffer[2]; // u8
                adt = @enumFromInt(buffer[3] >> 4); // u4
                fmt.fmt = @enumFromInt(buffer[3] & 0x7); // Lower 3 bits for FMT
                fmt.zero_pad = @truncate((buffer[3] >> 3) & 0x1); // Bit 3 for zero_pad
                instruction_valid = true;
            }
        },
        else => return error.InvalidInstructionLength,
    }

    if (!instruction_valid) {
        return error.InvalidOpcode;
    }

    // Validate register index
    if (reg_index >= defs.REGISTER_COUNT - 1) {
        return error.InvalidRegisterIndex;
    }

    if (channel > 1) {
        return error.InvalidIOChannel;
    }

    const ch: defs.IO_MAP = @enumFromInt(channel);

    // Select writer based on channel and custom settings
    const fout = switch (ch) {
        .STDIO => vm._stdout,
        .STDERR => vm._stderr,
        //else => return error.InvalidIOChannel,
    };

    const writer = fout.writer().any();

    // Get value to output
    const value = reg_file.read(reg_index);

    // Initialize error code (0 = success)
    var error_code: defs.RegisterType = 0;

    // Perform output
    if (buffer.len == 4) {
        // Formatted output
        const formatted = format(vm.alloc, value, adt, fmt) catch |err| {
            error_code = @intFromError(err);
            reg_file.write(reg_index + 1, error_code);
            return;
        };
        defer vm.alloc.free(formatted);

        writer.writeAll(formatted) catch |err| {
            error_code = @intFromError(err);
        };
    } else {
        // Raw output (use value directly based on ADT size)
        const byte_size = (adt.bits() + 7) >> 3; // Ceiling to bytes
        var buf: [8]u8 = undefined;
        switch (byte_size) {
            1 => buf[0] = @truncate(value),
            2 => std.mem.writeInt(u16, buf[0..2], @truncate(value), .little),
            4 => std.mem.writeInt(u32, buf[0..4], @truncate(value), .little),
            8 => std.mem.writeInt(u64, buf[0..8], @truncate(value), .little),
            else => return error.InvalidDataType,
        }

        writer.writeAll(buf[0..byte_size]) catch |err| {
            error_code = @intFromError(err);
        };
    }

    // Write error code to R[reg+1]
    reg_file.write(reg_index + 1, error_code);
}

const InputPipe = struct {
    reader: std.io.AnyReader,
    buffer: std.ArrayList(u8),
    allocator: std.mem.Allocator,

    pub fn init(allocator: std.mem.Allocator, reader: std.io.AnyReader) InputPipe {
        return .{
            .reader = reader,
            .buffer = std.ArrayList(u8).init(allocator),
            .allocator = allocator,
        };
    }

    pub fn deinit(self: *InputPipe) void {
        self.buffer.deinit();
    }

    // Read a byte, preferring buffered data
    pub fn readByte(self: *InputPipe) !?u8 {
        if (self.buffer.items.len > 0) {
            const byte = self.buffer.items[0];
            _ = self.buffer.orderedRemove(0);
            return byte;
        }
        return self.reader.readByte() catch |err| switch (err) {
            error.EndOfStream => null,
            else => return err,
        };
    }

    // Push back a byte to the buffer
    pub fn pushBack(self: *InputPipe, byte: u8) !void {
        try self.buffer.insert(0, byte);
    }

    // Read bytes into a slice, using buffer first
    pub fn read(self: *InputPipe, dest: []u8) !usize {
        var bytes_read: usize = 0;
        for (dest, 0..) |_, i| {
            if (self.buffer.items.len > 0) {
                dest[i] = self.buffer.items[0];
                _ = self.buffer.orderedRemove(0);
                bytes_read += 1;
            } else {
                // Manually handle the error union from readByte()
                const maybe_byte = self.reader.readByte() catch |err| switch (err) {
                    error.EndOfStream => null, // Return null to indicate end of stream
                    else => return err, // Propagate other errors
                };
                // Check if we got a byte or reached the end
                if (maybe_byte) |byte| {
                    dest[i] = byte;
                    bytes_read += 1;
                } else {
                    break; // End of stream, exit the loop
                }
            }
        }
        return bytes_read;
    }
};

fn signExtend(value: defs.RegisterType, adt: defs.ADT) !defs.RegisterType {
    const bs = adt.bits(); // 4 bits
    const one: defs.RegisterType = 1;
    const lower_mask = (one << @truncate(bs)) - one; // e.g., 0xF
    const mask = ~lower_mask; // e.g., 0xFFFFFFF0 for WS=32
    const sign_bit = (value & (one << @truncate(bs - one))) != 0;
    if (adt.signed() and sign_bit) {
        return value | mask;
    } else {
        return value & lower_mask;
    }
}

fn parseInput(alloc: std.mem.Allocator, pipe: *InputPipe, adt: defs.ADT, fmt: defs.OUT_FMT) !?defs.RegisterType {
    //var buf: [128]u8 = undefined;
    var input: []u8 = &[_]u8{};
    defer alloc.free(input);

    // Read until whitespace or EOF for formatted input
    if (fmt.fmt != .raw) {
        var temp = std.ArrayList(u8).init(alloc);
        defer temp.deinit();
        var done = false;

        while (!done) {
            const maybe_byte = try pipe.readByte();
            if (maybe_byte) |byte| {
                if (std.ascii.isWhitespace(byte)) {
                    done = true;
                    try pipe.pushBack(byte); // Push back whitespace
                } else {
                    try temp.append(byte);
                }
            } else {
                done = true; // EOF
            }
        }

        if (temp.items.len == 0) return null; // No input
        input = try temp.toOwnedSlice();
    }

    switch (fmt.fmt) {
        .raw => {
            const byte_size = (adt.bits() + 7) >> 3;
            var raw_buf: [8]u8 = undefined;
            const bytes_read = try pipe.read(raw_buf[0..byte_size]);
            if (bytes_read < byte_size) return null; // Insufficient data

            return switch (byte_size) {
                1 => raw_buf[0],
                2 => std.mem.readInt(u16, raw_buf[0..2], .little),
                4 => std.mem.readInt(u32, raw_buf[0..4], .little),
                8 => std.mem.readInt(u64, raw_buf[0..8], .little),
                else => return error.InvalidDataType,
            };
        },
        .hex => {
            const str = input;
            if (str.len == 0) return null;
            const value = std.fmt.parseInt(defs.RegisterType, str, 16) catch |err| {
                for (str) |b| try pipe.pushBack(b);
                return err;
            };
            return try signExtend(value, adt);
        },
        .dec => {
            const str = input;
            if (str.len == 0) return null;
            if (adt.signed()) {
                const value = std.fmt.parseInt(defs.RegisterSignedType, str, 10) catch |err| {
                    for (str) |b| try pipe.pushBack(b);
                    return err;
                };
                return try signExtend(@bitCast(value), adt);
            } else {
                const value = std.fmt.parseInt(defs.RegisterType, str, 10) catch |err| {
                    for (str) |b| try pipe.pushBack(b);
                    return err;
                };
                return try signExtend(value, adt);
            }
        },
        .bin => {
            const str = input;
            if (str.len == 0) return null;
            const value = std.fmt.parseInt(defs.RegisterType, str, 2) catch |err| {
                for (str) |b| try pipe.pushBack(b);
                return err;
            };
            return try signExtend(value, adt);
        },
        .fp0, .fp2, .fp4 => {
            const str = input;
            if (str.len == 0) return null;
            const value = std.fmt.parseFloat(f64, str) catch |err| {
                for (str) |b| try pipe.pushBack(b);
                return err;
            };
            return switch (adt) {
                .f16 => blk: {
                    const f16_val: f16 = @floatCast(value);
                    const r: u16 = @bitCast(f16_val);
                    break :blk r;
                },
                .f32 => blk: {
                    const f32_val: f32 = @floatCast(value);
                    const r: u32 = @bitCast(f32_val);
                    break :blk r;
                },
                .f64 => @bitCast(value),
                else => return error.InvalidDataType,
            };
        },
    }
}

// Execute IN instruction
pub fn executeIN(vm: *vm_mod.VM, buffer: []const u8) !void {
    const reg_file = &vm.registers;
    const mode = reg_file.readALU_MODE_CFG();
    var channel: u8 = 0;
    var reg_index: u8 = reg_file.readALU_IO_CFG().rs; // Default to N.RS
    var adt: defs.ADT = mode.adt; // Default to M.ADT
    var fmt: defs.OUT_FMT = .{ .fmt = .raw, .zero_pad = 0 }; // Default format
    const fmt_reg: u8 = reg_file.readALU_IO_CFG().src; // N.SRC for fmt in 1-byte/2-byte forms
    var instruction_valid = false;

    // Extract fmt from R[N.SRC] for 1-byte and 2-byte forms
    if (buffer.len < 4) {
        const fmt_value = reg_file.read(fmt_reg);
        fmt.fmt = @enumFromInt(fmt_value & 0x7);
        fmt.zero_pad = @truncate(fmt_value >> 3);
    }

    // Parse instruction based on length
    switch (buffer.len) {
        1 => {
            // 1-byte: 0xA|ch (ch is u4)
            if ((buffer[0] >> 4) == 0xA) {
                channel = buffer[0] & 0x0F; // Extract u4
                instruction_valid = true;
            }
        },
        2 => {
            // 2-byte: 0xCA, ch (ch is u8)
            if (buffer[0] == ((defs.PREFIX_OP2 << 4) | 0xA)) {
                channel = buffer[1]; // u8
                instruction_valid = true;
            }
        },
        4 => {
            // 4-byte: 0xDA, ch, reg|adt|fmt (ch, reg are u8, adt, fmt are u4)
            if (buffer[0] == ((defs.PREFIX_OP4 << 4) | 0xA)) {
                channel = buffer[1]; // u8
                reg_index = buffer[2]; // u8
                adt = @enumFromInt(buffer[3] >> 4); // u4
                fmt.fmt = @enumFromInt(buffer[3] & 0x7); // Lower 3 bits for FMT
                fmt.zero_pad = @truncate((buffer[3] >> 3) & 0x1); // Bit 3 for zero_pad
                instruction_valid = true;
            }
        },
        else => return error.InvalidInstructionLength,
    }

    if (!instruction_valid) {
        return error.InvalidOpcode;
    }

    // Validate register index
    if (reg_index >= defs.REGISTER_COUNT - 1) {
        return error.InvalidRegisterIndex;
    }

    // Only STDIO is supported for input
    if (channel != @intFromEnum(defs.IO_MAP.STDIO)) {
        return error.InvalidIOChannel;
    }

    // Initialize input pipe
    var pipe = InputPipe.init(vm.alloc, vm._stdin.reader().any());
    defer pipe.deinit();

    // Parse input
    const maybe_value = try parseInput(vm.alloc, &pipe, adt, fmt);
    if (maybe_value) |value| {
        reg_file.write(reg_index, value);
        reg_file.write(reg_index + 1, 0); // Success
    } else {
        reg_file.write(reg_index, 0);
        reg_file.write(reg_index + 1, @intFromError(error.EndOfStream)); // EOF
    }
}

test "format" {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const alloc = gpa.allocator();

    var adt: defs.ADT = undefined;

    var f: defs.OUT_FMT = undefined;
    f.fmt = .hex;
    f.zero_pad = 1;

    {
        const val: defs.RegisterType = 0xABCD;
        {
            adt = defs.ADT.u8;
            const str = try format(alloc, val, adt, f);
            defer alloc.free(str);
            try std.testing.expectEqualStrings("cd", str);
        }
        {
            adt = defs.ADT.u32;
            const str = try format(alloc, val, adt, f);
            defer alloc.free(str);
            try std.testing.expectEqualStrings("0000abcd", str);
        }
    }
    {
        f.fmt = .dec;
        {
            const val: defs.RegisterType = 0xFFFF;
            adt = defs.ADT.i16;
            const str = try format(alloc, val, adt, f);
            defer alloc.free(str);
            try std.testing.expectEqualStrings("-00001", str);
        }
        {
            const val: defs.RegisterType = 123;
            adt = defs.ADT.i16;
            const str = try format(alloc, val, adt, f);
            defer alloc.free(str);
            try std.testing.expectEqualStrings("00123", str);
        }
    }
    {
        const t: f64 = 1.2345e-2;
        const val: defs.RegisterType = @bitCast(t);
        adt = .f64;
        f.fmt = .fp2;
        {
            const str = try format(alloc, val, adt, f);
            defer alloc.free(str);
            try std.testing.expectEqualStrings("0.01", str);
        }
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
    //const log = std.log.scoped(.vm);

    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    var vm = try vm_mod.VM.init(allocator, "");
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

test "RET and IRET instructions" {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    // Initialize VM with 1024-byte memory
    var vm = try vm_mod.VM.init(allocator, "");
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

test "INC and DEC instructions" {
    var reg_file = regs.RegisterFile.init();
    var cfg = reg_file.readALU_IO_CFG();
    var mode = reg_file.readALU_MODE_CFG();

    // Test 1-byte INC: 0x03 (R[RS]++)
    cfg.rs = 1;
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

test "JMP and CALL instructions" {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    // Initialize VM with 1024-byte memory
    var vm = try vm_mod.VM.init(allocator, "");
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

test "ALU instruction" {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    // Initialize VM
    var vm = try vm_mod.VM.init(allocator, "");
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

test "NOT instruction" {
    var reg_file = regs.RegisterFile.init();
    var cfg = reg_file.readALU_IO_CFG();
    var mode = reg_file.readALU_MODE_CFG();

    // Setup registers
    cfg.rs = 1;
    cfg.src = 2;
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
    cfg.rs = 255; // Still set from previous
    reg_file.writeALU_IO_CFG(cfg);
    const not_invalid_reg = [_]u8{0x05}; // 1-byte, uses N.RS=255
    try executeNOT(&reg_file, &not_invalid_reg); // Should not fail, as R_IP (255) is valid
    const not_invalid_range = [_]u8{ (defs.PREFIX_OP4 << 4), 0x50, 0xFF, 0x02 }; // reg=255, cnt=2
    try std.testing.expectError(error.InvalidRegisterIndex, executeNOT(&reg_file, &not_invalid_range));
}

test "OUT instruction" {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    const allocator = gpa.allocator();

    var vm = try vm_mod.VM.init(allocator, "");

    var reg_file = &vm.registers;
    var cfg = reg_file.readALU_IO_CFG();
    var mode = reg_file.readALU_MODE_CFG();

    try vm.setStdOut("test_stdout.txt");
    try vm.setStdErr("test_stderr.txt");

    // Setup registers
    cfg.rs = 1;
    cfg.src = 2;
    cfg.dst = 3;
    reg_file.writeALU_IO_CFG(cfg);
    mode.adt = .u8;
    reg_file.writeALU_MODE_CFG(mode);

    // Test 1-byte OUT: 0xB0 (STDIO, raw u8 from R1)
    reg_file.write(1, 0x41); // ASCII 'A'
    reg_file.write(2, 0); // Clear R2
    const out_1byte = [_]u8{0xB0}; // ch=0 (STDIO)
    try executeOUT(&vm, &out_1byte);
    try std.testing.expectEqual(@as(defs.RegisterType, 0), reg_file.read(2)); // Success

    // Test 2-byte OUT: 0xCB, 0x01 (STDERR, raw u8 from R1)
    reg_file.write(1, 0x42); // ASCII 'B'
    reg_file.write(2, 0);
    const out_2byte = [_]u8{ (defs.PREFIX_OP2 << 4) | 0xB, 0x00 }; // ch=0 (STDOUT)
    try executeOUT(&vm, &out_2byte);
    try std.testing.expectEqual(@as(defs.RegisterType, 0), reg_file.read(2)); // Success

    // Test 4-byte OUT: 0xDB, 0x00, 0x03, 0x21 (STDIO, R3, u16, dec)
    mode.adt = .u16;
    reg_file.writeALU_MODE_CFG(mode);
    reg_file.write(3, 5);
    reg_file.write(4, 0);
    const out_4byte = [_]u8{ (defs.PREFIX_OP4 << 4) | 0xB, 0x00, 0x03, 0x22 | 0x8 }; // ch=0, reg=3, adt=u16, fmt=dec
    try executeOUT(&vm, &out_4byte);
    try std.testing.expectEqual(@as(defs.RegisterType, 0), reg_file.read(4)); // Success

    // Test invalid channel
    const out_invalid_ch = [_]u8{0xB2}; // ch=2
    try std.testing.expectError(error.InvalidIOChannel, executeOUT(&vm, &out_invalid_ch));

    // Test invalid opcode
    const out_invalid = [_]u8{0xA0};
    try std.testing.expectError(error.InvalidOpcode, executeOUT(&vm, &out_invalid));

    // Test invalid length
    const out_invalid_length = [_]u8{ 0xB0, 0x00, 0x00 };
    try std.testing.expectError(error.InvalidInstructionLength, executeOUT(&vm, &out_invalid_length));

    // Test invalid register index
    const out_invalid_reg = [_]u8{ (defs.PREFIX_OP4 << 4) | 0xB, 0x00, 0xFF, 0x00 }; // reg=255
    try std.testing.expectError(error.InvalidRegisterIndex, executeOUT(&vm, &out_invalid_reg));

    vm.deinit();
    _ = gpa.deinit();
}

test "InputPipe" {
    const allocator = std.testing.allocator;

    // Test 1: Reading bytes from the underlying reader
    {
        var buffer = [_]u8{ 0x41, 0x42, 0x43 }; // "ABC"
        var stream = std.io.fixedBufferStream(&buffer);
        var pipe = InputPipe.init(allocator, stream.reader().any());
        defer pipe.deinit();

        const byte1 = try pipe.readByte();
        try std.testing.expectEqual(@as(?u8, 0x41), byte1); // 'A'
        const byte2 = try pipe.readByte();
        try std.testing.expectEqual(@as(?u8, 0x42), byte2); // 'B'
        const byte3 = try pipe.readByte();
        try std.testing.expectEqual(@as(?u8, 0x43), byte3); // 'C'
        const byte4 = try pipe.readByte();
        try std.testing.expectEqual(@as(?u8, null), byte4); // EOF
    }

    // Test 2: Pushing back bytes and reading from buffer
    {
        var buffer = [_]u8{0x41}; // "A"
        var stream = std.io.fixedBufferStream(&buffer);
        var pipe = InputPipe.init(allocator, stream.reader().any());
        defer pipe.deinit();

        // Read one byte
        try std.testing.expectEqual(@as(?u8, 0x41), try pipe.readByte());
        // Push back different bytes
        try pipe.pushBack(0x58); // 'X'
        try pipe.pushBack(0x59); // 'Y'
        // Read back in LIFO order
        try std.testing.expectEqual(@as(?u8, 0x59), try pipe.readByte()); // 'Y'
        try std.testing.expectEqual(@as(?u8, 0x58), try pipe.readByte()); // 'X'
        // Buffer is empty, should hit EOF
        try std.testing.expectEqual(@as(?u8, null), try pipe.readByte());
    }

    // Test 3: Reading into a slice with read()
    {
        var buffer = [_]u8{ 0x41, 0x42, 0x43 }; // "ABC"
        var stream = std.io.fixedBufferStream(&buffer);
        var pipe = InputPipe.init(allocator, stream.reader().any());
        defer pipe.deinit();

        // Push back some bytes
        try pipe.pushBack(0x58); // 'X'
        try pipe.pushBack(0x59); // 'Y'

        // Read into a slice
        var dest: [4]u8 = undefined;
        const bytes_read = try pipe.read(&dest);
        try std.testing.expectEqual(@as(usize, 4), bytes_read);
        try std.testing.expectEqualSlices(u8, &[_]u8{ 0x59, 0x58, 0x41, 0x42 }, dest[0..4]);

        // Read again, should get last byte
        var dest2: [1]u8 = undefined;
        try std.testing.expectEqual(@as(usize, 1), try pipe.read(&dest2));
        try std.testing.expectEqualSlices(u8, &[_]u8{0x43}, &dest2);

        // Read again, should get EOF
        try std.testing.expectEqual(@as(usize, 0), try pipe.read(&dest2));
    }

    // Test 4: Handling formatted input with pushback
    {
        var buffer = [_]u8{ '1', '2', '3', ' ', 'A', 'B', 'C' }; // "123 ABC"
        var stream = std.io.fixedBufferStream(&buffer);
        var pipe = InputPipe.init(allocator, stream.reader().any());
        defer pipe.deinit();

        // Simulate formatted read (until whitespace)
        var temp = std.ArrayList(u8).init(allocator);
        defer temp.deinit();
        var done = false;
        while (!done) {
            const maybe_byte = try pipe.readByte();
            if (maybe_byte) |byte| {
                if (std.ascii.isWhitespace(byte)) {
                    done = true;
                    try pipe.pushBack(byte); // Push back space
                } else {
                    try temp.append(byte);
                }
            } else {
                done = true;
            }
        }
        try std.testing.expectEqualSlices(u8, "123", temp.items);

        _ = try pipe.readByte();

        // Read next token ("ABC")
        temp.clearRetainingCapacity();
        done = false;
        while (!done) {
            const maybe_byte = try pipe.readByte();
            if (maybe_byte) |byte| {
                if (std.ascii.isWhitespace(byte)) {
                    done = true;
                    try pipe.pushBack(byte);
                } else {
                    try temp.append(byte);
                }
            } else {
                done = true;
            }
        }
        try std.testing.expectEqualSlices(u8, "ABC", temp.items);
    }

    // Test 5: Error propagation from reader
    {
        var error_reader: er.ErrorReader = .{};
        var pipe = InputPipe.init(allocator, error_reader.any());
        defer pipe.deinit();

        try std.testing.expectError(error.TestError, pipe.readByte());
    }
}

test "IN instruction" {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    // Create temporary input file
    //const input_content = "41\nFF\n123\n1010\n1.2345\ninvalid";
    const input_file = "test_stdin.txt";
    //try std.fs.cwd().writeFile(.{ .sub_path = input_file, .data = input_content });

    // Initialize VM
    var vm = try vm_mod.VM.init(allocator, "");
    defer vm.deinit();

    // Redirect stdin
    try vm.setStdIn(input_file);
    //defer std.fs.cwd().deleteFile(input_file) catch {};

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

    // Test 1-byte IN: 0xA0 (STDIO, raw u8 to R1)
    reg_file.write(2, @intFromEnum(defs.FMT.raw)); // fmt=raw
    const in_1byte = [_]u8{0xA0}; // ch=0 (STDIO)
    try executeIN(&vm, &in_1byte);
    try std.testing.expectEqual(0x41, reg_file.read(1)); // ASCII 'A'
    try std.testing.expectEqual(0, reg_file.read(2)); // Success

    mode.adt = .u16;
    reg_file.writeALU_MODE_CFG(mode);
    reg_file.write(2, @intFromEnum(defs.FMT.raw)); // fmt=raw
    try executeIN(&vm, &in_1byte);
    try std.testing.expectEqual(0x0A0D, reg_file.read(1)); // 13 10
    try std.testing.expectEqual(0, reg_file.read(2)); // Success

    // Test 2-byte IN: 0xCA 0x00 (STDIO, hex u8 to R1)
    mode.adt = .u8;
    reg_file.writeALU_MODE_CFG(mode);
    reg_file.write(2, @intFromEnum(defs.FMT.hex)); // fmt=hex
    const in_2byte = [_]u8{ (defs.PREFIX_OP2 << 4) | 0xA, 0x00 }; // ch=0
    try executeIN(&vm, &in_2byte);
    try std.testing.expectEqual(0xFF, reg_file.read(1));
    try std.testing.expectEqual(0, reg_file.read(2)); // Success

    mode.adt = .u8;
    reg_file.writeALU_MODE_CFG(mode);
    reg_file.write(2, @intFromEnum(defs.FMT.raw)); // fmt=raw
    try executeIN(&vm, &in_1byte);
    try std.testing.expectEqual(0x0D, reg_file.read(1)); // 13 10
    try std.testing.expectEqual(0, reg_file.read(2)); // Success

    reg_file.write(2, @intFromEnum(defs.FMT.raw)); // fmt=raw
    try executeIN(&vm, &in_1byte);
    try std.testing.expectEqual(0x0A, reg_file.read(1)); // 13 10
    try std.testing.expectEqual(0, reg_file.read(2)); // Success

    // Test 4-byte IN: 0xDA 0x00 0x03 0x21 (STDIO, R3, u16, dec)
    mode.adt = .u16;
    reg_file.writeALU_MODE_CFG(mode);
    const in_4byte = [_]u8{ (defs.PREFIX_OP4 << 4) | 0xA, 0x00, 0x03, 0x21 }; // ch=0, reg=3, adt=u16, fmt=dec
    try executeIN(&vm, &in_4byte);
    try std.testing.expectEqual(@as(defs.RegisterType, 123), reg_file.read(3));
    try std.testing.expectEqual(@as(defs.RegisterType, 0), reg_file.read(4)); // Success

    // Test 4-byte IN: 0xDA 0x00 0x03 0x31 (STDIO, R3, u16, bin)
    const in_4byte_bin = [_]u8{ (defs.PREFIX_OP4 << 4) | 0xA, 0x00, 0x03, 0x31 }; // ch=0, reg=3, adt=u16, fmt=bin
    try executeIN(&vm, &in_4byte_bin);
    try std.testing.expectEqual(@as(defs.RegisterType, 10), reg_file.read(3)); // 1010 in binary
    try std.testing.expectEqual(@as(defs.RegisterType, 0), reg_file.read(4)); // Success

    // Test 4-byte IN: 0xDA 0x00 0x03 0xA5 (STDIO, R3, f64, fp2)
    mode.adt = .f64;
    reg_file.writeALU_MODE_CFG(mode);
    const in_4byte_float = [_]u8{ (defs.PREFIX_OP4 << 4) | 0xA, 0x00, 0x03, 0xA5 }; // ch=0, reg=3, adt=f64, fmt=fp2
    try executeIN(&vm, &in_4byte_float);
    const float_val: f64 = @bitCast(reg_file.read(3));
    try std.testing.expectApproxEqRel(1.2345, float_val, 1e-5);
    try std.testing.expectEqual(@as(defs.RegisterType, 0), reg_file.read(4)); // Success

    // Test 4-byte IN with invalid input: 0xDA 0x00 0x03 0x21 (STDIO, R3, u16, dec)
    const in_4byte_invalid = [_]u8{ (defs.PREFIX_OP4 << 4) | 0xA, 0x00, 0x03, 0x21 }; // ch=0, reg=3, adt=u16, fmt=dec
    try executeIN(&vm, &in_4byte_invalid);
    try std.testing.expectEqual(@as(defs.RegisterType, 0), reg_file.read(3));
    try std.testing.expectEqual(@as(defs.RegisterType, @intFromError(error.InvalidDigit)), reg_file.read(4)); // Parse error

    // Test invalid channel
    const in_invalid_ch = [_]u8{0xA1}; // ch=1 (STDERR)
    try std.testing.expectError(error.InvalidIOChannel, executeIN(&vm, &in_invalid_ch));

    // Test invalid opcode
    const in_invalid = [_]u8{0xC0};
    try std.testing.expectError(error.InvalidOpcode, executeIN(&vm, &in_invalid));

    // Test invalid length
    const in_invalid_length = [_]u8{ 0xA0, 0x00, 0x00 };
    try std.testing.expectError(error.InvalidInstructionLength, executeIN(&vm, &in_invalid_length));

    // Test invalid register index
    const in_invalid_reg = [_]u8{ (defs.PREFIX_OP4 << 4) | 0xA, 0x00, 0xFF, 0x00 }; // reg=255
    try std.testing.expectError(error.InvalidRegisterIndex, executeIN(&vm, &in_invalid_reg));
}
