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

// Register File
pub const RegisterFile = struct {
    registers: [defs.REGISTER_COUNT]RegisterType,

    // Initialize register file
    pub fn init() RegisterFile {
        return RegisterFile{
            .registers = [_]RegisterType{0} ** defs.REGISTER_COUNT,
        };
    }

    // Read register (general purpose or special)
    pub fn read(self: *const RegisterFile, index: u8) RegisterType {
        return self.registers[index];
    }

    // Write register
    pub fn write(self: *RegisterFile, index: u8, value: RegisterType) void {
        self.registers[index] = value;
    }

    // Read ALU_IO_CFG (R12) as packed struct
    pub fn readALU_IO_CFG(self: *const RegisterFile) defs.ALU_IO_CFG {
        return @bitCast(self.registers[defs.R_ALU_IO_CFG]);
    }

    // Write ALU_IO_CFG (R12)
    pub fn writeALU_IO_CFG(self: *RegisterFile, cfg: defs.ALU_IO_CFG) void {
        self.registers[defs.R_ALU_IO_CFG] = @bitCast(cfg);
    }

    // Read ALU_MODE_CFG (R13)
    pub fn readALU_MODE_CFG(self: *const RegisterFile) defs.ALU_MODE_CFG {
        return @bitCast(self.registers[defs.R_ALU_MODE_CFG]);
    }

    // Write ALU_MODE_CFG (R13)
    pub fn writeALU_MODE_CFG(self: *RegisterFile, cfg: defs.ALU_MODE_CFG) void {
        self.registers[defs.R_ALU_MODE_CFG] = @bitCast(cfg);
    }

    // Read ALU_VR_STRIDES (R14)
    pub fn readALU_VR_STRIDES(self: *const RegisterFile) defs.ALU_VR_STRIDES {
        return @bitCast(self.registers[defs.R_ALU_VR_STRIDES]);
    }

    // Write ALU_VR_STRIDES (R14)
    pub fn writeALU_VR_STRIDES(self: *RegisterFile, strides: defs.ALU_VR_STRIDES) void {
        self.registers[defs.R_ALU_VR_STRIDES] = @bitCast(strides);
    }

    // Read BRANCH_CTRL (R15)
    pub fn readBRANCH_CTRL(self: *const RegisterFile) defs.BRANCH_CTRL {
        return @bitCast(self.registers[defs.R_BRANCH_CTRL]);
    }

    // Write BRANCH_CTRL (R15)
    pub fn writeBRANCH_CTRL(self: *RegisterFile, ctrl: defs.BRANCH_CTRL) void {
        self.registers[defs.R_BRANCH_CTRL] = @bitCast(ctrl);
    }

    // Read Instruction Pointer (R255)
    pub fn readIP(self: *const RegisterFile) RegisterType {
        return self.registers[defs.R_IP];
    }

    // Write Instruction Pointer (R255)
    pub fn writeIP(self: *RegisterFile, ip: RegisterType) void {
        self.registers[defs.R_IP] = ip;
    }

    // Read Stack Pointer (R254)
    pub fn readSP(self: *const RegisterFile) RegisterType {
        return self.registers[defs.R_SP];
    }

    // Write Stack Pointer (R254)
    pub fn writeSP(self: *RegisterFile, sp: RegisterType) void {
        self.registers[defs.R_SP] = sp;
    }

    // Read Base Pointer (R253)
    pub fn readBP(self: *const RegisterFile) RegisterType {
        return self.registers[defs.R_BP];
    }

    // Write Base Pointer (R253)
    pub fn writeBP(self: *RegisterFile, bp: RegisterType) void {
        self.registers[defs.R_BP] = bp;
    }
};

// Comptime checks
comptime {
    std.debug.assert(@bitSizeOf(RegisterType) == defs.WS);
    std.debug.assert(@sizeOf(RegisterFile) == defs.REGISTER_COUNT * @sizeOf(RegisterType));
}
