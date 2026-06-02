# Pump State Machine Unit Tests

This directory contains ztest-based unit tests for the pure `pump_state_machine` module.

The state machine itself has **zero dependencies** on Zephyr kernel objects, making it very easy to test.

## Building the Tests

### On Linux (Recommended)

```bash
# From the project root
west build -b native_sim tests/logic/test_pump_state_machine
west build -t run
```

Or using twister (preferred for CI):

```bash
west twister -T tests/logic/test_pump_state_machine -p native_sim --inline-logs
```

### On macOS

`native_sim` is not supported on macOS (Zephyr explicitly disables the POSIX architecture on mac/Windows).

Use `qemu_x86` instead (this is currently the best option on mac):

```bash
west build -b qemu_x86 tests/logic/test_pump_state_machine
```

If the build fails with a message about a missing `x86_64-zephyr-elf-gcc`, your Zephyr SDK installation is incomplete.

**To install the missing toolchain:**

1. Re-run the Zephyr SDK installer and select the `x86_64-zephyr-elf` target, **or**
2. Manually download the x86 toolchain tarball from the Zephyr SDK releases and extract it into your SDK directory.

Once the x86 toolchain is present, the above `west build` command should succeed (the test configuration is now clean).

After building:

```bash
ninja -C build run
```

## Standalone Verification (No Zephyr Required)

Because the implementation is pure C, you can also verify the logic quickly without any Zephyr build system:

See `../test_pump_state_machine_standalone.c` (or compile the module + a small driver manually).

## Files

- `src/test_pump_state_machine.c` — The actual ztest cases
- `prj.conf` — Minimal configuration for the test
- `CMakeLists.txt` — Pulls in the state machine from the app

## Adding New Tests

Just add new `ZTEST(...)` functions in `src/test_pump_state_machine.c`. The module is small, so keep tests focused on state transitions and edge cases.
