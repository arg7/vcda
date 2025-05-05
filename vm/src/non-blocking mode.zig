     Set up non-blocking mode
    const file = switch (ch) {
        .STDIO => if (vm.custom_stdout) |_| null else std.io.getStdOut(),
        .STDERR => if (vm.custom_stderr) |_| null else std.io.getStdErr(),
        //else => unreachable,
    };

    if (comptime builtin.os.tag == .linux) { // and (file != null)) {
        const fd = file.handle;

        // Set non-blocking mode using fcntl (POSIX)
        const flags = std.os.fcntl(fd, std.os.F.GETFL, 0) catch |err| {
            // If we can't get flags, proceed with blocking I/O but note the error
            if (reg_index + 1 < defs.REGISTER_COUNT) {
                reg_file.write(reg_index + 1, @intFromError(err));
            }
            return;
        };
        _ = std.os.fcntl(fd, std.os.F.SETFL, flags | std.os.O.NONBLOCK) catch |err| {
            // If we can't set non-blocking, proceed with blocking I/O but note the error
            if (reg_index + 1 < defs.REGISTER_COUNT) {
                reg_file.write(reg_index + 1, @intFromError(err));
            }
            return;
        };
    }
