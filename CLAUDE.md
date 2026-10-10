# rtu-relais-4ch

Open-source firmware for the Waveshare Modbus RTU Relay 4CH board
(STM32F030C8, 4 changeover relays, RS485). Two variants from one code base:

- **plain**: generic 4-channel Modbus relay
- **heater**: 3-phase heating rod controller, setpoint in 0.1 %, 7 steps

**`docs/IMPLEMENTATION.md` is the spec**: register map, heater logic,
architecture, testing. Read it before changing behaviour, and update it in
the same commit when behaviour or registers change.

## Commands

```sh
cmake --preset heater-release && cmake --build --preset heater-release
cmake --build --preset heater-release --target flash   # st-flash via ST-Link
make -C test                 # host tests (both variants)
make -C test sim             # pty simulators test/build/sim_{plain,heater}
tools/test_device.py --port /dev/ttyUSB0 [--serial-test]   # on hardware or a simulator pty
```

Presets: `plain-debug`, `plain-release`, `heater-debug`, `heater-release`
(variant = `-DFW_PROFILE=plain|heater`). Run the host tests and build all
four presets before committing; builds must stay free of warnings (the ST HAL in `Drivers/` is built with `-w`).

## Layout

- `App/`: our code. `profile_plain.c` / `profile_heater.c` implement the
  variant interface in `profile.h`. `heater_ctrl.c` is portable heater logic.
  Shared code: `mb_server.c` (nanoMODBUS glue, common registers), `rs485.c`,
  `config.c` (settings log in the last 2 flash pages), `relay.c`, `led.c`, `app.c`.
- `App/STM32F030C8Tx_app.ld`: our linker script (62K app + 2K settings).
  The CubeMX one is not used; the build fails if the override breaks.
- `Src/`, `Inc/`, `Drivers/`, `cmake/stm32cubemx/`: CubeMX generated.
- `third_party/nanomodbus/`: vendored, unmodified (version in `VERSION`).
- `test/`: host tests with a fake HAL (`test/stubs/main.h` replaces the
  CubeMX `main.h`), and the simulator `sim.c`.

## Rules

- CubeMX files: only edit inside `USER CODE` blocks, keep their CRLF line
  endings. Peripheral settings go into `rtu-relais-4ch.ioc` as well, so
  regenerating keeps them.
- In the top-level `CMakeLists.txt`, the linker script override has to come
  after `project()`, because `project()` re-reads the toolchain file.
- Our code owns `USART1_IRQHandler`. Leave the USART1 interrupt disabled in
  CubeMX.
- The watchdog (~2 s, 1.6 s worst case) is refreshed only at the end of
  `app_loop()`. Nothing in the loop may block for long; the budget is in
  `docs/IMPLEMENTATION.md` 5.9 (worst case today: about 1 s at 4800 baud).
- Heater safety: R3 only together with R2, and R3 never switches together
  with another relay (`heater_ctrl.c` sequencer). Never write relays directly
  in the heater variant. `test_heater.c` checks all transitions.
- Hardware: relays and LED are active low. RS485 direction is switched
  automatically by the transceiver; there is no DE pin.
- License: our files start with `SPDX-License-Identifier: GPL-3.0-or-later`
  (new files too). Do not add it to CubeMX, ST or third-party files.
- Use named constants instead of magic numbers (`util.h`: `MS_PER_S`,
  `DIV_CEIL`, ...). Data tables (LED patterns, baud rates, heater steps) are fine.
