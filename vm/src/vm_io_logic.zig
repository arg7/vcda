const std = @import("std");
const defs = @import("definitions.zig");
const regs = @import("registers.zig");
const main = @import("vm.zig");
const alu_mod = @import("alu.zig");
const builtin = @import("builtin");
const IOPipe = @import("iopipe.zig").IOPipe;
const er = @import("errorreader.zig");

pub fn parseInput(iop: *IOPipe, adt: defs.ADT, fmt: defs.OUT_FMT) !defs.RegisterType {
    var input: []u8 = undefined;

    // Read until whitespace or EOF for formatted input
    if (fmt.fmt != .raw) {
        var temp: [16]u8 = undefined;
        var temp_idx: u32 = 0;
        // skip initial white space
        while (true) {
            const byte = iop.readByte() catch |err| {
                //std.debug.print("error: {any}\n", .{err});
                if (err == error.EndOfStream) break else return err;
            };
            if (!std.ascii.isWhitespace(byte)) {
                try iop.pushBack(byte);
                break;
            }
        }
        // read in buffer till whitespace or eof
        while (true) {
            const byte = iop.readByte() catch |err| {
                //std.debug.print("error: {any}\n", .{err});
                if (err == error.EndOfStream) break else return err;
            };
            if (std.ascii.isWhitespace(byte)) {
                try iop.pushBack(byte); // Push back whitespace
                break;
            } else {
                if (temp_idx >= temp.len)
                    return error.BufferOverflow;
                temp[temp_idx] = byte;
                temp_idx += 1;
            }
        }

        if (temp_idx == 0) return error.EndOfStream; // No input
        input = temp[0..temp_idx];
    }

    switch (fmt.fmt) {
        .raw => {
            const byte_size = (adt.bits() + 7) >> 3;
            var raw_buf: [8]u8 = undefined;
            const bytes_read = try iop.read(raw_buf[0..byte_size]);
            if (bytes_read < byte_size) return error.EndOfStream; // Insufficient data

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
            const value = std.fmt.parseInt(defs.RegisterType, str, 16) catch |err| {
                try iop.pushBackBytes(str);
                return err;
            };
            return try signExtend(value, adt);
        },
        .dec => {
            const str = input;
            if (adt.signed()) {
                const value = std.fmt.parseInt(defs.RegisterSignedType, str, 10) catch |err| {
                    try iop.pushBackBytes(str);
                    return err;
                };
                return try signExtend(@bitCast(value), adt);
            } else {
                const value = std.fmt.parseInt(defs.RegisterType, str, 10) catch |err| {
                    try iop.pushBackBytes(str);
                    return err;
                };
                return try signExtend(value, adt);
            }
        },
        .bin => {
            const str = input;
            const value = std.fmt.parseInt(defs.RegisterType, str, 2) catch |err| {
                try iop.pushBackBytes(str);
                return err;
            };
            return try signExtend(value, adt);
        },
        .fp0, .fp2, .fp4 => {
            const str = input;
            const value = std.fmt.parseFloat(f64, str) catch |err| {
                try iop.pushBackBytes(str);
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

pub fn signExtend(value: defs.RegisterType, adt: defs.ADT) !defs.RegisterType {
    const bs = adt.bits(); // 4 bits
    const one: defs.RegisterType = 1;
    const lower_mask = (one << @truncate(bs)) - one; // e.g., 0xF
    var mask = ~lower_mask; // e.g., 0xFFFFFFF0 for WS=32
    if (mask == 0) mask = ~mask;
    const sign_bit = (value & (one << @truncate(bs - one))) != 0;
    if (adt.signed() and sign_bit) {
        return value | mask;
    } else {
        return value & lower_mask;
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
pub fn executeOUT(vm: *main.VM, buffer: []const u8) !void {
    const reg_file = &vm.registers;
    const mode = reg_file.readALU_MODE_CFG();
    var channel: u8 = 0;
    var reg_index: u8 = reg_file.readALU_IO_CFG().arg1; // Default to N.RS
    const reg_index_fmt: u8 = reg_file.readALU_IO_CFG().arg2; // Default to N.RS
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

    const f = fout orelse return error.FOutMissing;
    const writer = f.writer().any();

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

// Execute IN instruction
pub fn executeIN(vm: *main.VM, buffer: []const u8) !void {
    const reg_file = &vm.registers;
    const mode = reg_file.readALU_MODE_CFG();
    var channel: u8 = 0;
    var reg_index: u8 = reg_file.readALU_IO_CFG().arg1; // Default to N.RS
    var adt: defs.ADT = mode.adt; // Default to M.ADT
    var fmt: defs.OUT_FMT = .{ .fmt = .raw, .zero_pad = 0 }; // Default format
    const fmt_reg: u8 = reg_file.readALU_IO_CFG().arg2; // N.SRC for fmt in 1-byte/2-byte forms
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

    const iop = vm._stdin_pipe orelse return error.FInMissing;

    // Parse input
    const v = parseInput(iop, adt, fmt) catch |err| {
        reg_file.write(reg_index, 0);
        reg_file.write(reg_index + 1, @intFromError(err));
        return;
    };
    reg_file.write(reg_index, v);
    reg_file.write(reg_index + 1, 0); // Success
}

test "format" {
    const alloc = std.testing.allocator;

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

test "OUT instruction" {
    const allocator = std.testing.allocator;

    // Create temporary input file
    //const input_content = "41\nFF\n123\n1010\n1.2345\ninvalid";
    const fn_in = "test_stdin.txt";
    const fn_out = "test_stdout.txt";
    const fn_err = "test_stderr.txt";
    //try std.fs.cwd().writeFile(.{ .sub_path = input_file, .data = input_content });

    var fin = try std.fs.cwd().openFile(fn_in, .{});
    var fout = try std.fs.cwd().createFile(fn_out, .{ .truncate = true });
    var ferr = try std.fs.cwd().createFile(fn_err, .{ .truncate = true });
    var iop = try IOPipe.init(allocator, fin.reader().any(), 16);
    defer iop.deinit();
    defer fout.close();
    defer fin.close();
    defer ferr.close();

    var vm = try main.VM.init(allocator, &iop, &fout, &ferr, 0xFFFF);
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
}

test "IOPipe" {
    const allocator = std.testing.allocator;

    // Test 1: Reading bytes from the underlying reader
    {
        var buffer = [_]u8{ 0x41, 0x42, 0x43 }; // "ABC"
        var stream = std.io.fixedBufferStream(&buffer);
        var pipe = try IOPipe.init(allocator, stream.reader().any(), 16);
        defer pipe.deinit();

        const byte1 = try pipe.readByte();
        try std.testing.expectEqual(0x41, byte1); // 'A'
        const byte2 = try pipe.readByte();
        try std.testing.expectEqual(0x42, byte2); // 'B'
        const byte3 = try pipe.readByte();
        try std.testing.expectEqual(0x43, byte3); // 'C'
        try std.testing.expectError(error.EndOfStream, pipe.readByte());
    }

    // Test 2: Pushing back bytes and reading from buffer
    {
        var buffer = [_]u8{0x41}; // "A"
        var stream = std.io.fixedBufferStream(&buffer);
        var pipe = try IOPipe.init(allocator, stream.reader().any(), 16);
        defer pipe.deinit();

        // Read one byte
        try std.testing.expectEqual(@as(?u8, 0x41), try pipe.readByte());
        // Push back different bytes
        try pipe.pushBack(0x58); // 'X'
        try pipe.pushBack(0x59); // 'Y'
        // Read back in FIFO order
        try std.testing.expectEqual(@as(?u8, 0x58), try pipe.readByte()); // 'X'
        try std.testing.expectEqual(@as(?u8, 0x59), try pipe.readByte()); // 'Y'
        // Buffer is empty, should hit EOF
        try std.testing.expectError(error.EndOfStream, pipe.readByte());
    }

    // Test 3: Reading into a slice with read()
    {
        var buffer = [_]u8{ 0x41, 0x42, 0x43 }; // "ABC"
        var stream = std.io.fixedBufferStream(&buffer);
        var pipe = try IOPipe.init(allocator, stream.reader().any(), 16);
        defer pipe.deinit();

        // Push back some bytes
        try pipe.pushBack(0x58); // 'X'
        try pipe.pushBack(0x59); // 'Y'

        // Read into a slice
        var dest: [4]u8 = undefined;
        const bytes_read = try pipe.read(&dest);
        try std.testing.expectEqual(@as(usize, 4), bytes_read);
        try std.testing.expectEqualSlices(u8, &[_]u8{ 0x58, 0x59, 0x41, 0x42 }, dest[0..4]);

        // Read again, should get last byte
        var dest2: [1]u8 = undefined;
        try std.testing.expectEqual(@as(usize, 1), try pipe.read(&dest2));
        try std.testing.expectEqualSlices(u8, &[_]u8{0x43}, &dest2);

        // Read again, should get EOF
        try std.testing.expectError(error.EndOfStream, pipe.readByte());
    }

    // Test 4: Handling formatted input with pushback
    {
        var buffer = [_]u8{ '1', '2', '3', ' ', 'A', 'B', 'C' }; // "123 ABC"
        var stream = std.io.fixedBufferStream(&buffer);
        var pipe = try IOPipe.init(allocator, stream.reader().any(), 16);
        defer pipe.deinit();

        // Simulate formatted read (until whitespace)
        var temp = std.ArrayList(u8).init(allocator);
        defer temp.deinit();

        var done = false;
        while (!done) {
            const byte = pipe.readByte() catch |err| {
                //std.debug.print("error: {any}\n", .{err});
                if (err == error.EndOfStream) break else return err;
            };
            if (std.ascii.isWhitespace(byte)) {
                done = true;
                try pipe.pushBack(byte); // Push back space
            } else {
                try temp.append(byte);
            }
        }
        try std.testing.expectEqualSlices(u8, "123", temp.items);

        _ = try pipe.readByte();

        // Read next token ("ABC")
        temp.clearRetainingCapacity();
        while (true) {
            const byte = pipe.readByte() catch |err| {
                //std.debug.print("error: {any}\n", .{err});
                if (err == error.EndOfStream) break else return err;
            };
            if (std.ascii.isWhitespace(byte)) {
                try pipe.pushBack(byte);
                break;
            } else {
                try temp.append(byte);
            }
        }
        try std.testing.expectEqualSlices(u8, "ABC", temp.items);
    }

    // Test 5: Error propagation from reader
    {
        var error_reader: er.ErrorReader = .{};
        var pipe = try IOPipe.init(allocator, error_reader.any(), 16);
        defer pipe.deinit();

        try std.testing.expectError(error.TestError, pipe.readByte());
    }
}

test "IN instruction" {
    const allocator = std.testing.allocator;

    // Create temporary input file
    //const input_content = "41\nFF\n123\n1010\n1.2345\ninvalid";
    const fn_in = "test_stdin.txt";
    const fn_out = "test_stdout.txt";
    const fn_err = "test_stderr.txt";
    //try std.fs.cwd().writeFile(.{ .sub_path = input_file, .data = input_content });

    var fin = try std.fs.cwd().openFile(fn_in, .{});
    var fout = try std.fs.cwd().createFile(fn_out, .{ .truncate = false });
    var ferr = try std.fs.cwd().createFile(fn_err, .{ .truncate = false });
    var iop = try IOPipe.init(allocator, fin.reader().any(), 16);
    defer iop.deinit();
    defer fout.close();
    defer fin.close();
    defer ferr.close();

    // Initialize VM
    var vm = try main.VM.init(allocator, &iop, &fout, &ferr, 0xFFFF);
    defer vm.deinit();

    var reg_file = &vm.registers;
    var cfg = reg_file.readALU_IO_CFG();
    var mode = reg_file.readALU_MODE_CFG();

    // Setup registers
    cfg.arg1 = 1;
    cfg.arg2 = 3;
    cfg.dst = 4;
    reg_file.writeALU_IO_CFG(cfg);
    mode.adt = .u8;
    reg_file.writeALU_MODE_CFG(mode);

    // Test 1-byte IN: 0xA0 (STDIO, raw u8 to R1)
    reg_file.write(3, @intFromEnum(defs.FMT.raw)); // fmt=raw
    const in_1byte = [_]u8{0xA0}; // ch=0 (STDIO)

    try executeIN(&vm, &in_1byte);
    try std.testing.expectEqual(0x41, reg_file.read(1)); // ASCII 'A'
    try std.testing.expectEqual(0, reg_file.read(2)); // Success

    mode.adt = .u16;
    reg_file.writeALU_MODE_CFG(mode);
    reg_file.write(3, @intFromEnum(defs.FMT.raw)); // fmt=raw
    try executeIN(&vm, &in_1byte);
    try std.testing.expectEqual(0x0A0D, reg_file.read(1)); // 13 10
    try std.testing.expectEqual(0, reg_file.read(2)); // Success

    // Test 2-byte IN: 0xCA 0x00 (STDIO, hex u8 to R1)
    mode.adt = .u8;
    reg_file.writeALU_MODE_CFG(mode);
    reg_file.write(3, @intFromEnum(defs.FMT.hex)); // fmt=hex
    const in_2byte = [_]u8{ (defs.PREFIX_OP2 << 4) | 0xA, 0x00 }; // ch=0
    try executeIN(&vm, &in_2byte);
    try std.testing.expectEqual(0xFF, reg_file.read(1));
    try std.testing.expectEqual(0, reg_file.read(2)); // Success
    try std.testing.expectEqual(1, iop.idx); // 13 in buffer

    mode.adt = .u16;
    reg_file.writeALU_MODE_CFG(mode);
    reg_file.write(3, @intFromEnum(defs.FMT.raw)); // fmt=raw
    try executeIN(&vm, &in_1byte);
    try std.testing.expectEqual(0x0A0D, reg_file.read(1)); // 13 10
    try std.testing.expectEqual(0, reg_file.read(2)); // Success

    // Test 4-byte IN: 0xDA 0x00 0x03 0x21 (STDIO, R3, u16, dec)
    mode.adt = .u16;
    reg_file.writeALU_MODE_CFG(mode);
    const in_4byte = [_]u8{ (defs.PREFIX_OP4 << 4) | 0xA, 0x00, 0x03, 0x21 }; // ch=0, reg=3, adt=u16, fmt=dec
    try executeIN(&vm, &in_4byte);
    try std.testing.expectEqual(123, reg_file.read(3));
    try std.testing.expectEqual(0, reg_file.read(4)); // Success
    // test on skiiping whitespace before token
    try executeIN(&vm, &in_4byte);
    try std.testing.expectEqual(321, reg_file.read(3));
    try std.testing.expectEqual(0, reg_file.read(4)); // Success
    // test on detecting wrong letter
    try executeIN(&vm, &in_4byte);
    try std.testing.expectEqual(0, reg_file.read(3));
    try std.testing.expectEqual(@intFromError(error.InvalidCharacter), reg_file.read(4)); // Parse error

    { // test that IOPipe retained wrong value
        mode.adt = .u8;
        reg_file.writeALU_MODE_CFG(mode);
        reg_file.write(3, @intFromEnum(defs.FMT.raw)); // fmt=raw
        const tstr = "\r123A";
        for (0..tstr.len) |i| {
            try executeIN(&vm, &in_1byte);
            try std.testing.expectEqual(tstr[i], reg_file.read(1));
            try std.testing.expectEqual(0, reg_file.read(2)); // Success
        }
    }
    // Test 4-byte IN: 0xDA 0x00 0x03 0x53 (STDIO, R3, u16, bin)
    const in_4byte_bin = [_]u8{ (defs.PREFIX_OP4 << 4) | 0xA, 0x00, 0x03, 0x23 }; // ch=0, reg=3, adt=u16, fmt=bin
    try executeIN(&vm, &in_4byte_bin);
    try std.testing.expectEqual(10, reg_file.read(3)); // 1010 in binary
    try std.testing.expectEqual(0, reg_file.read(4)); // Success

    mode.adt = .u16;
    reg_file.writeALU_MODE_CFG(mode);
    reg_file.write(3, @intFromEnum(defs.FMT.raw)); // fmt=raw
    try executeIN(&vm, &in_1byte);
    try std.testing.expectEqual(0x0A0D, reg_file.read(1)); // 13 10
    try std.testing.expectEqual(0, reg_file.read(2)); // Success

    // Test 4-byte IN: 0xDA 0x00 0x03 0xA5 (STDIO, R3, f64, fp2)
    mode.adt = .f64;
    reg_file.writeALU_MODE_CFG(mode);
    const in_4byte_float = [_]u8{ (defs.PREFIX_OP4 << 4) | 0xA, 0x00, 0x03, 0xA5 }; // ch=0, reg=3, adt=f64, fmt=fp2
    try executeIN(&vm, &in_4byte_float);
    const float_val: f64 = @bitCast(reg_file.read(3));
    try std.testing.expectApproxEqRel(1.2345, float_val, 1e-5);
    try std.testing.expectEqual(0, reg_file.read(4)); // Success

    mode.adt = .u16;
    reg_file.writeALU_MODE_CFG(mode);
    reg_file.write(3, @intFromEnum(defs.FMT.raw)); // fmt=raw
    try executeIN(&vm, &in_1byte);
    try std.testing.expectEqual(0x0A0D, reg_file.read(1)); // 13 10
    try std.testing.expectEqual(0, reg_file.read(2)); // Success

    // Test 4-byte IN with invalid input: 0xDA 0x00 0x03 0x51 (STDIO, R3, u16, dec)
    const in_4byte_invalid = [_]u8{ (defs.PREFIX_OP4 << 4) | 0xA, 0x00, 0x03, 0x51 }; // ch=0, reg=3, adt=u16, fmt=dec
    try executeIN(&vm, &in_4byte_invalid);
    try std.testing.expectEqual(0, reg_file.read(3));
    try std.testing.expectEqual(@intFromError(error.InvalidCharacter), reg_file.read(4)); // Parse error

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
