const std = @import("std");
const testing = std.testing;
const CPU = @import("main.zig").CPU;
const m = @import("main.zig");
const BranchCondition = @import("main.zig").BranchCondition;
const ALUOperation = @import("main.zig").ALUOperation;

const allocator = std.testing.allocator;

test "executeJMP with Zero condition, Z=1" {

    var cpu = try CPU.init(allocator, 1024);
    defer cpu.deinit(allocator);

    // Setup: IP = 0x100, JMP_STRIDE = 4, op = 2, BCS = Z, Z flag = 1
    cpu.R[m.ip] = 0x100;
    cpu.R[m.jmp_stride] = 4;
    cpu.setFlag(.BCS, BranchCondition.Z.value());
    cpu.setFlag(.Z, 1);

    // Execute
    try cpu.executeJMP(0x100, 2);

    // Assert: IP = 0x100 + 4 * 2 = 0x108
    try testing.expectEqual(@as(u32, 0x108), cpu.R[m.ip]);
}

test "executeJMP with NotZero condition, Z=0" {
    
    var cpu = try CPU.init(allocator, 1024);
    defer cpu.deinit(allocator);

    // Setup: IP = 0x100, JMP_STRIDE = 4, op = 2, BCS = NZ, Z flag = 0 (default)
    cpu.R[m.ip] = 0x100;
    cpu.R[m.jmp_stride] = 4;
    cpu.setFlag(.BCS, BranchCondition.NZ.value());
    cpu.setFlag(.Z, 0);

    // Execute
    try cpu.executeJMP(0x100, 2);

    // Assert: IP = 0x100 + 4 * 2 = 0x108 (Z=0 satisfies NZ)
    try testing.expectEqual(@as(u32, 0x108), cpu.R[m.ip]);
}

test "executeJMP with NotZero condition, Z=1" {
    
    var cpu = try CPU.init(allocator, 1024);
    defer cpu.deinit(allocator);

    // Setup: IP = 0x100, JMP_STRIDE = 4, op = 2, BCS = NZ, Z flag = 1
    cpu.R[m.ip] = 0x100;
    cpu.R[m.jmp_stride] = 4;
    cpu.setFlag(.BCS, BranchCondition.NZ.value());
    cpu.setFlag(.Z, 1);

    // Execute
    try cpu.executeJMP(0x100, 2);

    // Assert: IP unchanged (Z=1 fails NZ)
    try testing.expectEqual(@as(u32, 0x100), cpu.R[m.ip]);
}

test "executeALU ADD operation" {
    
    var cpu = try CPU.init(allocator, 1024);
    defer cpu.deinit(allocator);

    cpu.R[m.r0] = 0xFF;
    cpu.R[m.r1] = 0x01;

    // Execute ADD
    try cpu.executeALU(ALUOperation._add.value());

    // Assert: R2 = 0xFF + 0x01 = 0x100
    try testing.expectEqual(@as(u32, 0x100), cpu.R[m.r2]);
}

test "executeALU DIV by zero" {
    
    var cpu = try CPU.init(allocator, 1024);
    defer cpu.deinit(allocator);

    cpu.R[m.r0] = 0x1;
    cpu.R[m.r1] = 0x0;

    // Execute DIV, expect error
    const result = cpu.executeALU(ALUOperation._div.value());
    try testing.expectError(error.DivisionByZero, result);
}

test "executeALU SAR operation" {
    
    var cpu = try CPU.init(allocator, 1024);
    defer cpu.deinit(allocator);

    // Setup: R0 = (signed -1), R1 = 2 (shift amount)
    var v: u32 = 0;
    v = ~v;

    cpu.R[m.r0] = v;
    cpu.R[m.r1] = 2;

    // Execute SAR
    try cpu.executeALU(ALUOperation._sar.value());

    // Assert: R2 = 0xFF >> 2 = 0xFFFFFFFF (sign-extended)
    try testing.expectEqual(@as(u32, 0xFFFFFFFF), cpu.R[m.r2]);
}
