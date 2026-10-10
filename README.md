# rtu-relais-4ch

Open-source replacement firmware for the
[Waveshare Modbus RTU Relay 4CH](https://www.waveshare.com/wiki/Modbus_RTU_Relay_4CH)
board (STM32F030C8, 4 changeover relays, isolated RS485).

Two variants are built from one code base:

- **plain**: a generic 4-channel Modbus relay. It is a Modbus-compliant
  successor of the stock firmware, not a drop-in replacement: the register
  map is new.
- **heater**: a 3-phase heating rod controller. The master writes a power
  setpoint in 0.1 %, the firmware switches 3 elements in 7 steps and
  reports the actual (quantized) power back.

> **Warning:** the relays switch mains voltage. Wire and test the heater
> variant with dummy loads first, and only work on the board de-energized.

## Features

- Modbus RTU slave: FC 01, 03, 04, 05, 06, 0F, 10 and 2B/0E (device
  identification). Broadcast writes without reply, proper exceptions.
- Default **9600 8N1, address 1**. Address, baud rate (4800 to 256000) and
  parity are configurable over Modbus. They are staged and applied by an
  explicit command, so the master can't lock itself out halfway.
- Settings in flash (wear-levelled log in the last 2 pages), written only
  when a value changes, with a write rate limit against a master that
  toggles a persistent register cyclically.
- Communication timeout: without control writes for N seconds, the outputs
  go to a safe state (plain: all off; heater: step 0).
- Independent watchdog (about 2 s), status and diagnostic registers
  (uptime, CRC and UART error counters, reset cause).
- Factory reset: hold SWDIO (PA13) to GND at power-up, or write command 2
  over Modbus.
- **plain:** coils, relay mask register and per-relay pulse timers.
- **heater:** step table for 3 x 1.93 kW elements (see the
  [schematic](docs/heater_rod_4_changeover_schematic.svg)), minimum hold
  time before stepping up, up-hysteresis, and a switching sequence that
  never puts 400 V across an open contact.

## Registers at a glance

All addresses are 0-based. The full map is in
[docs/IMPLEMENTATION.md](docs/IMPLEMENTATION.md#3-register-map).

| Type     | Addr          | Variant | Content |
|----------|---------------|---------|---------|
| Coil     | 0x0000-0x0003 | both    | Relay 1..4 (read-only in heater) |
| Input    | 0x0000-0x0007 | both    | Version, variant, relay mask, status, uptime, error counters |
| Input    | 0x0010-0x0012 | heater  | Actual power (0.1 %), actual step, target step |
| Holding  | 0x0000        | both    | Communication timeout in s (default plain 0 = off, heater 60) |
| Holding  | 0x0010        | plain   | Relay mask |
| Holding  | 0x0020-0x0023 | plain   | Pulse timers in 100 ms |
| Holding  | 0x0010        | heater  | **Setpoint in 0.1 %** (0..1000) |
| Holding  | 0x0011-0x0012 | heater  | Hold time in s (default 60), up-hysteresis in 0.1 % |
| Holding  | 0x0100-0x0102 | both    | Address, baud index, parity (staged) |
| Holding  | 0x010F        | both    | Command: 1 = save and apply serial settings, 2 = restore defaults |

Heater steps (actual power in 0.1 %): 0, 167, 334, 500, 667, 834, 1000.
A setpoint selects the highest step whose power it reaches.

## Building

Needs an ARM GCC toolchain, CMake and Ninja. On Debian:

```sh
sudo apt install gcc-arm-none-eabi cmake ninja-build stlink-tools
```

Build a variant (presets: `plain-debug`, `plain-release`, `heater-debug`,
`heater-release`):

```sh
cmake --preset heater-release
cmake --build --preset heater-release
```

The output is `build/<preset>/rtu-relais-4ch-<variant>.{elf,hex,bin}`.

## Flashing

The firmware has no update bootloader of its own: it is flashed via SWD
with an ST-Link at 0x08000000. (The STM32F030's ROM bootloader on USART1,
entered with BOOT0 high at reset, is still there but not needed.) The
`flash` target builds first, but the preset has to be configured once
(see above):

```sh
cmake --preset heater-release     # once per build directory
cmake --build --preset heater-release --target flash
# same as: st-flash --reset write build/heater-release/rtu-relais-4ch-heater.bin 0x8000000
```

**Stock boards are read-protected (RDP level 1)**: `st-flash` cannot
write until the protection is removed, and removing it mass-erases the
chip, so the stock firmware is gone and cannot be backed up. Remove it
once with OpenOCD and then power-cycle the board (the new option bytes are
loaded at power-up):

```sh
openocd -f interface/stlink.cfg -f target/stm32f0x.cfg \
    -c "init; reset halt; stm32f1x unlock 0; shutdown"
```

## Testing

```sh
make -C test        # host tests of both variants with a fake HAL
make -C test sim    # Modbus simulators on a pseudo-terminal
tools/test_device.py --port /dev/ttyUSB0 [--serial-test]   # needs python3-serial
```

The device test also runs against the simulators. It switches the relays,
so **disconnect the loads first**.

## Documentation

- [docs/IMPLEMENTATION.md](docs/IMPLEMENTATION.md): the specification, with
  hardware details, register map, heater logic, architecture and testing.
- [CLAUDE.md](CLAUDE.md): short notes for working on the code (also read by Claude Code).

## License

The firmware is licensed under the GNU General Public License v3.0 or
later, see [LICENSE](LICENSE). Bundled third-party code keeps its own
license: [nanoMODBUS](https://github.com/debevv/nanoMODBUS) (MIT), the ST
HAL and CubeMX generated files (BSD-3-Clause), CMSIS (Apache-2.0).
