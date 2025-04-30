// registers.zig
const std = @import("std");
const defs = @import("definitions.zig");

/// Derive register type from WS (in bits)
pub const RegisterType = blk: {
    if (defs.WS == 8) {
        break :blk u8;
    } else if (defs.WS == 16) {
        break :blk u16;
    } else if (defs.WS == 32) {
        break :blk u32;
    } else if (defs.WS == 64) {
        break :blk u64;
    } else {
        @compileError("Unsupported WS size: " ++ std.fmt.comptimePrint("{}", .{defs.WS}));
    }
};

pub const SpecialRegisterType = blk: {
    if (defs.WS <= 32) {
        break :blk u32;
    } else if (defs.WS == 64) {
        break :blk u64;
    } else {
        @compileError("Unsupported WS size: " ++ std.fmt.comptimePrint("{}", .{defs.WS}));
    }
};

pub const PointerRegisterType = blk: {
    if (defs.WS <= 16) {
        break :blk u16;
    } else if (defs.WS == 32) {
        break :blk u32;
    } else if (defs.WS == 64) {
        break :blk u64;
    } else {
        @compileError("Unsupported WS size: " ++ std.fmt.comptimePrint("{}", .{defs.WS}));
    }
};

pub const SpecialRegisterFile = struct {
    alu_io_cfg: u32, // Always 32 bits
    alu_mode_cfg: u32, // Always 32 bits
    alu_vr_strides: u32, // Always 32 bits
    branch_ctrl: u32, // Always 32 bits
    bp: PointerRegisterType,
    sp: PointerRegisterType,
    ip: PointerRegisterType,

    pub fn init() SpecialRegisterFile {
        return SpecialRegisterFile{
            .alu_io_cfg = 0,
            .alu_mode_cfg = 0,
            .alu_vr_strides = 0,
            .branch_ctrl = 0,
            .bp = 0,
            .sp = 0,
            .ip = 0,
        };
    }
};

// Determine register size in bits
pub fn reg_size_bits(index: u8) u8 {
    const bits: u8 = switch (index) {
        defs.R_ALU_IO_CFG, defs.R_ALU_MODE_CFG, defs.R_ALU_VR_STRIDES, defs.R_BRANCH_CTRL => 32,
        defs.R_BP, defs.R_SP, defs.R_IP => if (defs.WS == 8) 16 else defs.WS,
        else => defs.WS,
    };
    return bits;
}

// Determine register size in bits
pub fn reg_is_gp(index: u8) bool {
    return switch (index) {
        defs.R_ALU_IO_CFG, defs.R_ALU_MODE_CFG, defs.R_ALU_VR_STRIDES, defs.R_BRANCH_CTRL,
        defs.R_BP, defs.R_SP, defs.R_IP => false,
        else => true,
    };
}

// Register File
pub const RegisterFile = struct {
    registers: [defs.REGISTER_COUNT - 7]RegisterType, // General-purpose registers
    special_regs: SpecialRegisterFile,

    pub fn init() RegisterFile {
        return RegisterFile{
            .registers = [_]RegisterType{0} ** (defs.REGISTER_COUNT - 7),
            .special_regs = SpecialRegisterFile.init(),
        };
    }
    
    fn mapIndex(i: u8) u8 {
        // Compute the offset based on special registers less than i
        if (i < 12) {
            return i; // No special registers before
        } else {
            return i - 4; // Adjust for 12, 13, 14, 15
        }
    }

    pub fn read(self: *const RegisterFile, index: u8) SpecialRegisterType {
        switch (index) {
            defs.R_ALU_IO_CFG => return self.special_regs.alu_io_cfg,
            defs.R_ALU_MODE_CFG => return self.special_regs.alu_mode_cfg,
            defs.R_ALU_VR_STRIDES => return self.special_regs.alu_vr_strides,
            defs.R_BRANCH_CTRL => return self.special_regs.branch_ctrl,
            defs.R_BP => return self.special_regs.bp,
            defs.R_SP => return self.special_regs.sp,
            defs.R_IP => return self.special_regs.ip,
            else => return self.registers[mapIndex(index)],
        }
    }

    pub fn write(self: *RegisterFile, index: u8, value: SpecialRegisterType) void {
        switch (index) {
            defs.R_ALU_IO_CFG => self.special_regs.alu_io_cfg = @truncate(value),
            defs.R_ALU_MODE_CFG => self.special_regs.alu_mode_cfg = @truncate(value),
            defs.R_ALU_VR_STRIDES => self.special_regs.alu_vr_strides = @truncate(value),
            defs.R_BRANCH_CTRL => self.special_regs.branch_ctrl = @truncate(value),
            defs.R_BP => self.special_regs.bp = @truncate(value),
            defs.R_SP => self.special_regs.sp = @truncate(value),
            defs.R_IP => self.special_regs.ip = @truncate(value),
            else => self.registers[mapIndex(index)] = @truncate(value),
        }
    }

    // Read ALU_IO_CFG (R12) as packed struct
    pub fn readALU_IO_CFG(self: *const RegisterFile) defs.ALU_IO_CFG {
        return @bitCast(self.special_regs.alu_io_cfg);
    }

    // Write ALU_IO_CFG (R12)
    pub fn writeALU_IO_CFG(self: *RegisterFile, cfg: defs.ALU_IO_CFG) void {
        self.special_regs.alu_io_cfg = @bitCast(cfg);
    }

    // Read ALU_MODE_CFG (R13)
    pub fn readALU_MODE_CFG(self: *const RegisterFile) defs.ALU_MODE_CFG {
        return @bitCast(self.special_regs.alu_mode_cfg);
    }

    // Write ALU_MODE_CFG (R13)
    pub fn writeALU_MODE_CFG(self: *RegisterFile, cfg: defs.ALU_MODE_CFG) void {
        self.special_regs.alu_mode_cfg = @bitCast(cfg);
    }

    // Read ALU_VR_STRIDES (R14)
    pub fn readALU_VR_STRIDES(self: *const RegisterFile) defs.ALU_VR_STRIDES {
        return @bitCast(self.special_regs.alu_vr_strides);
    }

    // Write ALU_VR_STRIDES (R14)
    pub fn writeALU_VR_STRIDES(self: *RegisterFile, strides: defs.ALU_VR_STRIDES) void {
        self.special_regs.alu_vr_strides = @bitCast(strides);
    }

    // Read BRANCH_CTRL (R15)
    pub fn readBRANCH_CTRL(self: *const RegisterFile) defs.BRANCH_CTRL {
        return @bitCast(self.special_regs.branch_ctrl);
    }

    // Write BRANCH_CTRL (R15)
    pub fn writeBRANCH_CTRL(self: *RegisterFile, ctrl: defs.BRANCH_CTRL) void {
        self.special_regs.branch_ctrl = @bitCast(ctrl);
    }

    // Read Instruction Pointer (R255)
    pub fn readIP(self: *const RegisterFile) RegisterType {
        return @bitCast(self.special_regs.ip);
    }

    // Write Instruction Pointer (R255)
    pub fn writeIP(self: *RegisterFile, ip: RegisterType) void {
        self.registers[defs.R_IP] = @bitCast(ip);
    }

    // Read Stack Pointer (R254)
    pub fn readSP(self: *const RegisterFile) RegisterType {
        return @bitCast(self.special_regs.sp);
    }

    // Write Stack Pointer (R254)
    pub fn writeSP(self: *RegisterFile, sp: RegisterType) void {
        self.special_regs.sp = @bitCast(sp);
    }

    // Read Base Pointer (R253)
    pub fn readBP(self: *const RegisterFile) RegisterType {
        return @bitCast(self.special_regs.bp);
    }

    // Write Base Pointer (R253)
    pub fn writeBP(self: *RegisterFile, bp: RegisterType) void {
        self.special_regs.bp = @bitCast(bp);
    }
};
