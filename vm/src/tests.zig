const std = @import("std");
const testing = std.testing;
const defs = @import("main.zig");

const allocator = std.testing.allocator;

test "executeJMP with Zero condition, Z=1" {

    var cpu = try defs.CPU.init(allocator, 1024);
    defer cpu.deinit(allocator);

    // Setup: IP = 0x100, JMP_STRIDE = 4, op = 2, BCS = Z, Z flag = 1
    cpu.R[defs.ip] = 0x100;
    cpu.R[defs.jmp_stride] = 4;
    cpu.setFlag(.BCS, defs.BranchCondition.Z.value());
    cpu.setFlag(.Z, 1);

    // Execute
    try cpu.executeJMP(0x100, 2);

    // Assert: IP = 0x100 + 4 * 2 = 0x108
    try testing.expectEqual(@as(u32, 0x108), cpu.R[defs.ip]);
}

test "executeJMP with NotZero condition, Z=0" {
    
    var cpu = try defs.CPU.init(allocator, 1024);
    defer cpu.deinit(allocator);

    // Setup: IP = 0x100, JMP_STRIDE = 4, op = 2, BCS = NZ, Z flag = 0 (default)
    cpu.R[defs.ip] = 0x100;
    cpu.R[defs.jmp_stride] = 4;
    cpu.setFlag(.BCS, defs.BranchCondition.NZ.value());
    cpu.setFlag(.Z, 0);

    // Execute
    try cpu.executeJMP(0x100, 2);

    // Assert: IP = 0x100 + 4 * 2 = 0x108 (Z=0 satisfies NZ)
    try testing.expectEqual(@as(u32, 0x108), cpu.R[defs.ip]);
}

test "executeJMP with NotZero condition, Z=1" {
    
    var cpu = try defs.CPU.init(allocator, 1024);
    defer cpu.deinit(allocator);

    // Setup: IP = 0x100, JMP_STRIDE = 4, op = 2, BCS = NZ, Z flag = 1
    cpu.R[defs.ip] = 0x100;
    cpu.R[defs.jmp_stride] = 4;
    cpu.setFlag(.BCS, defs.BranchCondition.NZ.value());
    cpu.setFlag(.Z, 1);

    // Execute
    try cpu.executeJMP(0x100, 2);

    // Assert: IP unchanged (Z=1 fails NZ)
    try testing.expectEqual(@as(u32, 0x100), cpu.R[defs.ip]);
}

test "executeALU ADD operation" {
    
    var cpu = try defs.CPU.init(allocator, 1024);
    defer cpu.deinit(allocator);

    cpu.R[defs.r0] = 0xFF;
    cpu.R[defs.r1] = 0x01;

    // Execute ADD
    try cpu.executeALU(defs.ALUOperation._add.value());

    // Assert: R2 = 0xFF + 0x01 = 0x100
    try testing.expectEqual(@as(u32, 0x100), cpu.R[defs.r2]);
}

test "executeALU SAR operation" {
    
    var cpu = try defs.CPU.init(allocator, 1024);
    defer cpu.deinit(allocator);

    // Setup: R0 = (signed -1), R1 = 2 (shift amount)
    var v: u32 = 0;
    v = ~v;

    cpu.R[defs.r0] = v;
    cpu.R[defs.r1] = 2;

    // Execute SAR
    try cpu.executeALU(defs.ALUOperation._sar.value());

    // Assert: R2 = 0xFF >> 2 = 0xFFFFFFFF (sign-extended)
    try testing.expectEqual(@as(u32, 0xFFFFFFFF), cpu.R[defs.r2]);
}

// Helper to check all flags
fn expectFlags(cpu: *const defs.CPU, c: u8, z: u8, s: u8, v: u8, p: u8, i: u8) !void {
    try testing.expectEqual(c, cpu.getFlag(.C));
    try testing.expectEqual(z, cpu.getFlag(.Z));
    try testing.expectEqual(s, cpu.getFlag(.S));
    try testing.expectEqual(v, cpu.getFlag(.V));
    try testing.expectEqual(p, cpu.getFlag(.P));
    try testing.expectEqual(i, cpu.getFlag(.I));
}

test "executeALU ADD normal" {
    var cpu = try defs.CPU.init(allocator, 1024);
    defer cpu.deinit(allocator);

    cpu.R[defs.r0] = 0xFF;
    cpu.R[defs.r1] = 0x01;

    try cpu.executeALU(defs.ALUOperation._add.value());
    try testing.expectEqual(@as(u32, 0x100), cpu.R[defs.r2]);
    try expectFlags(&cpu, 0, 0, 0, 0, 0, 0); // No carry, not zero, positive, no overflow, odd parity
}

test "executeALU ADD overflow" {
    var cpu = try defs.CPU.init(allocator, 1024);
    defer cpu.deinit(allocator);

    cpu.R[defs.r0] = 0x7FFFFFFF; // Max positive i32
    cpu.R[defs.r1] = 1;

    try cpu.executeALU(defs.ALUOperation._add.value());
    try testing.expectEqual(@as(u32, 0x80000000), cpu.R[defs.r2]);
    try expectFlags(&cpu, 0, 0, 1, 1, 1, 0); // No carry, not zero, negative, overflow, even parity
}

test "executeALU SUB normal" {
    var cpu = try defs.CPU.init(allocator, 1024);
    defer cpu.deinit(allocator);

    cpu.R[defs.r0] = 0x10;
    cpu.R[defs.r1] = 0x01;

    try cpu.executeALU(defs.ALUOperation._sub.value());
    try testing.expectEqual(@as(u32, 0x0F), cpu.R[defs.r2]);
    try expectFlags(&cpu, 0, 0, 0, 0, 0, 0); // No borrow, not zero, positive, no overflow
}

test "executeALU SUB borrow" {
    var cpu = try defs.CPU.init(allocator, 1024);
    defer cpu.deinit(allocator);

    cpu.R[defs.r0] = 0;
    cpu.R[defs.r1] = 1;

    try cpu.executeALU(defs.ALUOperation._sub.value());
    try testing.expectEqual(@as(u32, 0xFFFFFFFF), cpu.R[defs.r2]);
    try expectFlags(&cpu, 1, 0, 1, 0, 0, 0); // Borrow, not zero, negative, no overflow
}

test "executeALU AND" {
    var cpu = try defs.CPU.init(allocator, 1024);
    defer cpu.deinit(allocator);

    cpu.R[defs.r0] = 0xF0;
    cpu.R[defs.r1] = 0x0F;

    try cpu.executeALU(defs.ALUOperation._and.value());
    try testing.expectEqual(@as(u32, 0x00), cpu.R[defs.r2]);
    try expectFlags(&cpu, 0, 1, 0, 0, 1, 0); // No carry, zero, positive, no overflow, even parity
}

test "executeALU OR" {
    var cpu = try defs.CPU.init(allocator, 1024);
    defer cpu.deinit(allocator);

    cpu.R[defs.r0] = 0xF0;
    cpu.R[defs.r1] = 0x0F;

    try cpu.executeALU(defs.ALUOperation._or.value());
    try testing.expectEqual(@as(u32, 0xFF), cpu.R[defs.r2]);
    try expectFlags(&cpu, 0, 0, 0, 0, 0, 0); // No carry, not zero, positive, no overflow
}

test "executeALU XOR" {
    var cpu = try defs.CPU.init(allocator, 1024);
    defer cpu.deinit(allocator);

    cpu.R[defs.r0] = 0xFF;
    cpu.R[defs.r1] = 0xF0;

    try cpu.executeALU(defs.ALUOperation._xor.value());
    try testing.expectEqual(@as(u32, 0x0F), cpu.R[defs.r2]);
    try expectFlags(&cpu, 0, 0, 0, 0, 0, 0); // No carry, not zero, positive, no overflow
}

test "executeALU SHL" {
    var cpu = try defs.CPU.init(allocator, 1024);
    defer cpu.deinit(allocator);

    cpu.R[defs.r0] = 0x01;
    cpu.R[defs.r1] = 1;

    try cpu.executeALU(defs.ALUOperation._shl.value());
    try testing.expectEqual(@as(u32, 0x02), cpu.R[defs.r2]);
    try expectFlags(&cpu, 0, 0, 0, 0, 1, 0); // No carry, not zero, positive, no overflow, even parity
}

test "executeALU SHL carry" {
    var cpu = try defs.CPU.init(allocator, 1024);
    defer cpu.deinit(allocator);

    cpu.R[defs.r0] = 0x80000000;
    cpu.R[defs.r1] = 1;

    try cpu.executeALU(defs.ALUOperation._shl.value());
    try testing.expectEqual(@as(u32, 0x00), cpu.R[defs.r2]);
    try expectFlags(&cpu, 1, 1, 0, 0, 1, 0); // Carry, zero, positive, no overflow, even parity
}

test "executeALU SHR" {
    var cpu = try defs.CPU.init(allocator, 1024);
    defer cpu.deinit(allocator);

    cpu.R[defs.r0] = 0x02;
    cpu.R[defs.r1] = 1;

    try cpu.executeALU(defs.ALUOperation._shr.value());
    try testing.expectEqual(@as(u32, 0x01), cpu.R[defs.r2]);
    try expectFlags(&cpu, 0, 0, 0, 0, 0, 0); // No carry, not zero, positive
}

test "executeALU SAR" {
    var cpu = try defs.CPU.init(allocator, 1024);
    defer cpu.deinit(allocator);

    cpu.R[defs.r0] = 0xFFFFFFFF; // -1 signed
    cpu.R[defs.r1] = 2;

    try cpu.executeALU(defs.ALUOperation._sar.value());
    try testing.expectEqual(@as(u32, 0xFFFFFFFF), cpu.R[defs.r2]);
    try expectFlags(&cpu, 0, 0, 1, 0, 0, 0); // No carry, not zero, negative
}

test "executeALU MUL" {
    var cpu = try defs.CPU.init(allocator, 1024);
    defer cpu.deinit(allocator);

    cpu.R[defs.r0] = 0xFF;
    cpu.R[defs.r1] = 0x02;

    try cpu.executeALU(defs.ALUOperation._mul.value());
    try testing.expectEqual(@as(u32, 0x1FE), cpu.R[defs.r2]);
    try expectFlags(&cpu, 0, 0, 0, 0, 1, 0); // No carry, not zero, positive, even parity
}

test "executeALU DIV normal" {
    var cpu = try defs.CPU.init(allocator, 1024);
    defer cpu.deinit(allocator);

    cpu.R[defs.r0] = 0x10;
    cpu.R[defs.r1] = 0x02;

    try cpu.executeALU(defs.ALUOperation._div.value());
    try testing.expectEqual(@as(u32, 0x08), cpu.R[defs.r2]);
    try expectFlags(&cpu, 0, 0, 0, 0, 0, 0); // No carry, not zero, positive
}

test "executeALU DIV by zero" {
    var cpu = try defs.CPU.init(allocator, 1024);
    defer cpu.deinit(allocator);

    cpu.R[defs.r0] = 0x10;
    cpu.R[defs.r1] = 0x00;

    const result = cpu.executeALU(defs.ALUOperation._div.value());
    try testing.expectError(error.DivisionByZero, result);
}

test "executeALU LOOKUP stub" {
    var cpu = try defs.CPU.init(allocator, 1024);
    defer cpu.deinit(allocator);

    cpu.R[defs.r0] = 0x42;

    try cpu.executeALU(defs.ALUOperation._lookup.value());
    try testing.expectEqual(@as(u32, 0x42), cpu.R[defs.r2]); // Stub returns r0
    try expectFlags(&cpu, 0, 0, 0, 0, 0, 0); // No flag changes assumed
}

test "executeALU LOAD stub" {
    var cpu = try defs.CPU.init(allocator, 1024);
    defer cpu.deinit(allocator);

    cpu.R[defs.r1] = 0x24;

    try cpu.executeALU(defs.ALUOperation._load.value());
    try testing.expectEqual(@as(u32, 0x24), cpu.R[defs.r2]); // Stub returns r1
    try expectFlags(&cpu, 0, 0, 0, 0, 0, 0); // No flag changes assumed
}

test "executeALU STORE stub" {
    var cpu = try defs.CPU.init(allocator, 1024);
    defer cpu.deinit(allocator);

    cpu.R[defs.r0] = 0xFF;

    try cpu.executeALU(defs.ALUOperation._store.value());
    try testing.expectEqual(@as(u32, 0x00), cpu.R[defs.r2]); // Stub returns 0
    try expectFlags(&cpu, 0, 1, 0, 0, 1, 0); // Zero result flags
}
