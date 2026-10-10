# Implementation Guide

Open-source firmware for the Waveshare **Modbus RTU Relay 4CH** board.
Two variants are built from the same code base:

- **plain**: a generic 4-channel Modbus relay, a cleaned-up and
  Modbus-compliant successor of the stock firmware.
- **heater**: a 3-phase heating rod controller. It takes a power setpoint
  in percent and switches 3 elements in 7 steps (see
  [schematic](heater_rod_4_changeover_schematic.svg)).

The variant is selected at build time. Everything else is shared: the
Modbus server, RS485 driver, settings storage, watchdog, LED, factory reset
and the common registers.

**License:** our code (`App/`, `test/`, `tools/`) is GPL-3.0-or-later, see
`LICENSE`. Bundled third-party code keeps its own license: nanoMODBUS (MIT,
`third_party/nanomodbus/LICENSE`), the ST HAL and the CubeMX generated
files (BSD-3-Clause, `Drivers/STM32F0xx_HAL_Driver/LICENSE.txt`, also for
`App/STM32F030C8Tx_app.ld`, which is derived from the CubeMX linker script)
and CMSIS (Apache-2.0, `Drivers/CMSIS/LICENSE.txt`). All are compatible
with GPLv3, so the firmware binary is distributed under GPLv3.

## 1. Hardware

| Item          | Detail                                                        |
|---------------|---------------------------------------------------------------|
| MCU           | STM32F030C8T6 (64 KiB flash, 8 KiB RAM, Cortex-M0)            |
| Clock         | 8 MHz HSE, PLL x6 = 48 MHz SYSCLK/HCLK/PCLK                   |
| Relays        | REL_1..REL_4 = PB0..PB3, **active low** (SET = off), changeover contacts |
| Status LED    | LED_STATE = PC13, **active low**                              |
| RS485         | USART1 TX = PA9, RX = PA10, isolated transceiver              |
| RS485 dir.    | No MCU-controlled DE/RE: the transceiver switches direction itself |
| Bus idle bias | A-B = 3.5 V idle without termination (strong fail-safe bias)  |
| TXD/RXD LEDs  | Hardware driven, not connected to the MCU                     |
| SWD           | PA13 (SWDIO), PA14 (SWCLK)                                    |

The board is flashed via SWD at 0x08000000 (no bootloader). The stock flash
has been erased.

## 2. Protocol Principles

- Standard Modbus RTU slave. Serial format: 8 data bits, 1 stop bit, parity
  none/even/odd. Default **9600 8N1, address 1**.
- Supported function codes: 01, 03, 04, 05, 06, 0F, 10. Optionally FC 2B/0E
  (Read Device Identification), if the pinned nanoMODBUS version supports
  it.
- Broadcast (address 0) frames: writes are executed, and **no reply** is
  sent, as the Modbus specification requires.
- Coil writes accept only `FF00` and `0000`. Any other value gives
  exception 03.
- Unknown addresses give exception 02, and illegal values give exception 03.
  A function code that the variant does not support (e.g. coil writes in
  the heater variant) gives exception 01.
- All addresses below are 0-based PDU addresses.

### 2.1 Differences from the stock Waveshare firmware

| Stock                                         | Here |
|-----------------------------------------------|------|
| FC 05 value `5500` = toggle, toggle coils 0x0100+ | Dropped. It is not compliant, and a retry after a lost reply toggles twice. |
| Replies to broadcast frames                   | Dropped (spec compliant) |
| "All relays" coil 0x00FF                      | Use FC 0F or the relay mask register |
| Flash on/off via coils 0x0200/0x0400          | Pulse timer holding registers (plain) |
| Settings at 0x2000/0x4000, version at 0x8000  | New register map (below) |

## 3. Register Map

Ranges 0x0000..0x000F and 0x0100..0x01FF are **common**.
Range 0x0010..0x00FF is **variant specific**.

### 3.1 Coils (FC 01 / 05 / 0F)

| Addr          | plain | heater | Content |
|---------------|-------|--------|---------|
| 0x0000-0x0003 | R/W   | R      | Relay 1..4 state (1 = energized) |

In the heater variant, relay writes would bypass the step table and the
switching sequence, so FC 05/0F give exception 01.

### 3.2 Input registers (FC 04, read only)

| Addr   | Variant | Content |
|--------|---------|---------|
| 0x0000 | common  | Firmware version, `major << 8 \| minor` |
| 0x0001 | common  | Variant ID: 1 = plain, 2 = heater |
| 0x0002 | common  | Actual relay mask (bit0 = relay 1) |
| 0x0003 | common  | Status flags (see 3.4) |
| 0x0004 | common  | Uptime in seconds, high word |
| 0x0005 | common  | Uptime in seconds, low word |
| 0x0006 | common  | Count of frames with a bad CRC (wraps) |
| 0x0007 | common  | Count of UART errors: framing, noise, parity, overrun, RX buffer full (wraps) |
| 0x0010 | heater  | Actual power in 0.1 % (0..1000), from the actual relay state |
| 0x0011 | heater  | Actual step (0..6) |
| 0x0012 | heater  | Target step (what the setpoint asks for, before hold time and sequencing) |

### 3.3 Holding registers (FC 03 / 06 / 10)

Common:

| Addr   | Persist | Content |
|--------|---------|---------|
| 0x0000 | yes     | Communication timeout in s, 0 = off. Default: plain 0, heater 60 |
| 0x0100 | staged  | Slave address 1..247 |
| 0x0101 | staged  | Baud index: 0=4800, 1=9600, 2=19200, 3=38400, 4=57600, 5=115200, 6=128000, 7=256000 |
| 0x0102 | staged  | Parity: 0 = none, 1 = even, 2 = odd |
| 0x010F | -       | Command. 1 = save and apply serial settings. 2 = restore all defaults and apply. Reads give 0 |

plain:

| Addr          | Persist | Content |
|---------------|---------|---------|
| 0x0010        | no      | Relay mask, bit0..3 = relay 1..4 |
| 0x0020-0x0023 | no      | Pulse timer relay 1..4 in 100 ms units. Writing N > 0 switches the relay on and off again after N x 100 ms. Writing 0 cancels the timer (the relay keeps its state). Reads return the remaining time |

heater:

| Addr   | Persist | Content |
|--------|---------|---------|
| 0x0010 | no      | **Setpoint in 0.1 %** (0..1000). Default after boot: 0 |
| 0x0011 | yes     | Minimum hold time in s before stepping **up** again. Default 60 |
| 0x0012 | yes     | Up-hysteresis in 0.1 %. Default 0 |

Rules:

- **Persistent** values are saved to flash right after the reply, and only
  if they changed.
- **Staged** serial settings take effect only after command 1 has been
  written to 0x010F. Otherwise, changing baud rate and parity in two writes
  would lock the master out after the first one. The reply to the command
  is still sent with the old settings. Reads of 0x0100..0x0102 return the
  staged values.
- Slave address range: 1..247, per the Modbus specification. The stock
  firmware allowed up to 255.
- Parity mapping 1 = even, 2 = odd is defined in a single table in
  `config.c`.

### 3.4 Status flags (input register 0x0003)

| Bit | Variant | Meaning |
|-----|---------|---------|
| 0   | common  | Communication timeout tripped. Outputs are forced to the safe state. Cleared by the next control write |
| 1   | common  | The active settings equal the variant defaults |
| 2   | common  | Last reset was caused by the watchdog |
| 3   | common  | Flash write error |
| 8   | heater  | Hold active: a higher step is pending because the hold time has not passed yet |
| 9   | heater  | Switching sequence in progress |

### 3.5 Communication timeout

The timeout is retriggered **only by control writes**:

- plain: coil writes, relay mask writes, pulse timer writes
- heater: setpoint writes

A master that only reads must not keep the outputs alive. Writing the
timeout register itself restarts the timer. When the timeout
expires, the variant goes to its safe state (plain: all relays off; heater:
step 0) and sets status bit 0.

## 4. Heater Logic

### 4.1 Step table

Wiring per [schematic](heater_rod_4_changeover_schematic.svg): 3 x 1.93 kW
elements on L1/L2/L3/N. R3 off connects junction J to N, and R4 off connects
the bottom of E3 to N.

| Step | Mask (bit0 = R1) | Circuit                            | Share | Power (0.1 %) |
|------|------------------|------------------------------------|-------|---------------|
| 0    | 0x0              | off                                | 0     | 0             |
| 1    | 0x6 (R2+R3)      | L2-E2-E3-N in series (230 V / 2R)  | 1/6   | 167           |
| 2    | 0x1 (R1)         | E1 L1-N                            | 2/6   | 334           |
| 3    | 0xE (R2+R3+R4)   | L2-E2-E3-L3 in series (400 V / 2R) | 3/6   | 500           |
| 4    | 0x3 (R1+R2)      | E1 + E2 to N                       | 4/6   | 667           |
| 5    | 0xF (all)        | E1 + series pair L2-L3             | 5/6   | 834           |
| 6    | 0xB (R1+R2+R4)   | E1, E2, E3 all to N                | 6/6   | 1000          |

The table lives in `heater_ctrl.c` as `heater_steps[]` (mask and power per
step). It is a lookup table rather than a formula, so other rod
configurations with uneven steps only need a different table. The powers
must be strictly ascending.

Invariant: **R3 only with R2**. Otherwise the open contact of R2 can see
400 V. Every step in the table satisfies it.

The power share does not depend on the element power, so no power
configuration is needed. The energy manager scales percent to watts.

### 4.2 Quantization

- A step's power P[k] is its share in 0.1 %, **rounded up**. It serves both
  as the reported actual value and as the lowest setpoint that selects the
  step.
- Target step = the highest k with `setpoint >= P[k]`, so the setpoint is
  rounded **down** (e.g. 333 gives step 1, since 33.3 % < 2/6). Writing back
  any reported actual value always selects exactly that step.
- Rounding up means steps 2 and 5 report 33.4 % and 83.4 %, which is
  0.07 % above the real share.
- Up-hysteresis h: stepping up to step j requires
  `setpoint >= min(P[j] + h, 1000)`. Stepping down uses plain `P[k]`.

### 4.3 Hold time

- Stepping **down** happens immediately. The heater never draws more than
  the setpoint for longer than one switching sequence.
- Stepping **up** is allowed only when at least the hold time has passed
  since the last step change. This limits relay cycles to at most one per
  hold time, even with a jittering setpoint.
- A communication timeout, a setpoint of 0, or boot always goes straight to
  step 0, without waiting for the hold time.
- The hold time also counts from boot. The first step up after power-up
  (or after a watchdog reset) therefore waits for the hold time. This
  keeps relays from cycling in a reset loop.
- The communication timeout steps down through the regular switching
  sequence, not by dropping all relays at once.

### 4.4 Switching sequence

A step change from mask A to mask B runs in up to 3 phases, with
`SEQ_PAUSE_MS` = 50 between them. Phases with nothing to do are skipped.

1. If R3 is set in A and not in B: release R3.
2. Set R1, R2 and R4 to B. R3 keeps its current state.
3. If R3 is not set in A but is set in B: energize R3.

Every intermediate state satisfies "R3 only with R2". A new target is taken
over only when the sequencer is idle. The latest target wins. The hold
timer starts when the sequence is complete. A host unit test checks all
7 x 7 transitions for the invariant in every intermediate state.

### 4.5 Limits outside firmware control

- On a reset (watchdog, brown-out) or power loss, all relays release at the
  same time and in random order. The sequence cannot be guaranteed then.
  This is brief, and the end state (all off) is safe.
- The R3 normally-closed contact carries up to 8.4 A (steps 4 and 6). The
  Waveshare product page rates the board at 20 A (NO) / 10 A (NC) at
  240 VAC, matching the fitted HLS-T90 (15F) relays (1C: 20 A / 10 A at
  240 VAC / 28 VDC). The NC contact of R3 therefore has the smallest
  margin: 8.4 A of 10 A.

## 5. Software Architecture

### 5.1 Modbus stack: nanoMODBUS

[nanoMODBUS](https://github.com/debevv/nanoMODBUS) (MIT) is vendored as a
pinned release under `third_party/nanomodbus/`, with its `LICENSE` and a
`VERSION` file.

- It handles RTU framing, CRC, broadcast and exception replies.
- Our code supplies the platform callbacks (`read`/`write`) and the server
  callbacks (`read_coils`, `read_holding_registers`, `read_input_registers`,
  `write_single_coil`, `write_multiple_coils`, `write_single_register`,
  `write_multiple_registers`). Callbacks set to NULL give exception 01.
- `nmbs_server_poll()` is called from the main loop with read timeout 0,
  so it returns immediately when there is no data. Once the first byte of a
  frame has arrived, nanoMODBUS reads the rest of it using the byte timeout.
- The byte timeout is max(20 ms, ceil(t3.5)). nanoMODBUS knows each
  frame's length from its function code, so the timeout is only used to
  recover from broken frames. The generous minimum tolerates gaps inside
  frames from USB-RS485 adapters.
- The `flush` callback (called by nanoMODBUS after an invalid frame)
  discards bytes until the line has been idle for t3.5, for at most 100 ms.
  This resynchronizes with the frame boundaries.
- Before every reply, the driver waits t3.5 (busy wait on SysTick), so the
  required silence after the request is kept.
- Device identification (FC 2B/0E, basic): VendorName `rtu-relais-4ch`,
  ProductCode = variant name, MajorMinorRevision = firmware version.
- After a slave address change, the server instance is re-created.

### 5.2 Modules

```
App/
  Inc/ ...
  Src/
    app.c           app_init(), app_loop(): LED, IWDG, factory reset, uptime
    rs485.c         USART1 RXNE ISR -> ring buffer, blocking TX with RX blanking,
                    line config (baud, parity)
    mb_server.c     nanoMODBUS glue. Common registers, forwards the variant
                    ranges to the profile, retriggers the comm timeout
    config.c        persistent settings (flash log, 2 pages)
    relay.c         relay outputs (active-low handling, mask get/set)
    profile.h       variant interface (see 5.3)
    profile_plain.c plain variant: coil/mask writes, pulse timers
    profile_heater.c heater variant: register glue around heater_ctrl
    heater_ctrl.c   portable step logic: quantization, hysteresis, hold time,
                    sequencer. No HAL, host testable
third_party/nanomodbus/
test/
  Makefile          host build (gcc)
  test_heater.c     quantization, hold time, all 7x7 sequences + invariant
  test_server.c     nanoMODBUS server with an in-memory transport: register
                    map, exceptions, broadcast, comm timeout, for both variants
```

CubeMX-generated code is only touched inside `USER CODE` blocks.
`main.c` calls `app_init()` in `USER CODE 2` and `app_loop()` inside the
`while (1)` loop.

### 5.3 Profile interface

```c
typedef struct {
    uint16_t id;                       /* input register 0x0001 */
    void (*init)(void);
    void (*poll)(uint32_t now_ms);     /* timers, sequencer */
    void (*safe_state)(void);          /* comm timeout */
    /* coils: write handler NULL -> exception 01 */
    nmbs_error (*read_coils)(uint16_t addr, uint16_t qty, nmbs_bitfield out);
    nmbs_error (*write_coils)(uint16_t addr, uint16_t qty, const nmbs_bitfield in);
    /* variant range 0x0010..0x00FF only */
    nmbs_error (*read_hreg)(uint16_t addr, uint16_t *val);
    nmbs_error (*write_hreg)(uint16_t addr, uint16_t val, bool *control_write);
    nmbs_error (*read_ireg)(uint16_t addr, uint16_t *val);
    uint16_t (*status_flags)(void);    /* bits 8..15 */
} profile_t;

extern const profile_t profile;       /* provided by profile_plain.c or profile_heater.c */
```

Multi-register writes (FC 10) are validated in full first, then applied.
If any value is invalid, nothing is written (exception 03).

### 5.4 Build system and variant selection

CubeMX overwrites the generated `Makefile` on every regeneration, so the
project **switches the CubeMX toolchain to CMake**:

- `cmake/stm32cubemx/CMakeLists.txt` is regenerated by CubeMX.
- The top-level `CMakeLists.txt` is ours. It adds `App/`,
  `third_party/nanomodbus/` and our linker script
  `App/STM32F030C8Tx_app.ld`.
- Variant option: `-DFW_PROFILE=plain|heater`. It selects
  `profile_plain.c` or `profile_heater.c` and defines `FW_PROFILE_HEATER`
  (heater only). Output: `rtu-relais-4ch-<profile>.{elf,hex,bin}`.
- Presets: `plain-release`, `heater-release`, `plain-debug` and
  `heater-debug`.
- Host tests: `make -C test`. They build the firmware modules with the
  native gcc against a fake HAL, flash and UART (`test/stubs/`), and run
  Modbus frames through the real nanoMODBUS server.
- Build: `cmake --preset heater-release && cmake --build --preset heater-release`.
- Flash: `cmake --build --preset <preset> --target flash` (runs
  `st-flash --reset write <image>.bin 0x8000000`, override with `-DST_FLASH=...`).

### 5.5 RS485 driver (`rs485.c`)

- The USART1 RXNE ISR writes into a 256-byte ring buffer. Uses direct
  register access; HAL UART RX is not used, because at 256000 baud a byte
  arrives every 39 µs.
- The ISR clears the error flags (ORE, FE, NE, PE) and counts the errors.
  A corrupted frame is then dropped by the nanoMODBUS CRC check.
- `write()` callback: clear `CR1.RE`, send the bytes polled via TXE, wait
  for TC, flush the ring buffer, then set `CR1.RE` again. This discards any
  echo from the auto-direction transceiver. With a 256-byte reply at
  4800 baud, TX blocks for at most about 0.6 s, which is below the
  watchdog timeout.
- Line reconfiguration uses `HAL_UART_Init()`. With parity, the word length
  must be **9 bits** (`UART_WORDLENGTH_9B` = 8 data + parity).
- 128000 and 256000 baud at 48 MHz: BRR = 375 / 187.5. The baud error is
  at most 0.27 %.

### 5.6 Persistent settings (`config.c`)

- The last two 1 KiB flash pages are used: 0x0800F800 and 0x0800FC00. The
  linker script limits the application to 62 KiB.
- Each record is 16 bytes, written as 8 half-words:
  `magic, seq, addr, baud, parity, comm_timeout, hold_time, hysteresis, crc16`.
  The fields are shared by both variants. The plain variant ignores the
  heater fields.
- Records are appended to the active page. When the page is full, the
  other page is erased and receives the record, then the old page is
  erased. At boot the newest valid record wins. If there is none, the
  variant defaults are used and status bit 1 is set.
- A record is only written when a value actually changed.
- A page erase stalls the CPU for about 20 to 40 ms, and bytes received in
  that time may be lost. This only happens right after a settings write.

### 5.7 Status LED

- Normal operation: blink at 1 Hz.
- Communication timeout tripped: double blink.
- Factory reset done: 10 fast blinks (50 ms).

### 5.8 Factory reset

Same procedure as the stock bootloader: short **SWDIO (PA13) to GND at
power-up**.

- `app_init()` samples PA13 for about 50 ms. The pin stays in SWD AF mode,
  because IDR still reflects the pin level and SWDIO has an internal
  pull-up.
- If PA13 stays low, the defaults are written and the LED confirms with
  fast blinks.
- Command 2 on holding register 0x010F does the same over Modbus.

### 5.9 Watchdog

The stock firmware is gone, so it can't be checked whether it used a
watchdog. The option byte `WDG_SW` survives a plain mass erase and could
show a hardware watchdog.

We use the **IWDG**:

- Clock is LSI (30 to 50 kHz), prescaler 32, reload 1250. This gives about
  1 s nominal (0.8 to 1.3 s).
- The watchdog is refreshed **only at the end of the main loop**. ISRs
  never refresh it.
- It is stopped while the core is halted in the debugger
  (`DBGMCU_APB1_FZ_DBG_IWDG_STOP`, DBGMCU clock enabled).
- After a watchdog reset, the relays come up off (GPIO init state), and
  `RCC->CSR.IWDGRSTF` sets status bit 2. The flag is then cleared.

## 6. CubeMX Changes Required

1. Project Manager: Toolchain/IDE = **CMake**.
2. Enable **IWDG**. Our code starts it with the values from 5.9.
3. Leave the USART1 global interrupt **disabled** in NVIC. Our code defines
   `USART1_IRQHandler` and enables the IRQ itself.

## 7. Implementation Steps

All steps are implemented. Not yet verified on hardware.

1. CubeMX changes, top-level CMake with variant option, linker script,
   empty `app_init()`/`app_loop()`. Both variants must build.
2. `relay.c` and the LED.
3. `rs485.c` and the nanoMODBUS glue with the common registers.
4. `profile_plain.c` with coils, mask and pulse timers. Then `config.c`
   with the serial settings and the apply command.
5. `heater_ctrl.c` with host tests, then `profile_heater.c`.
6. Communication timeout, factory reset, IWDG, status flags.
7. `tools/test_device.py`: register map on the real device for both
   variants (see 8).

## 8. Testing

- **Host tests:** `make -C test`. Register map of both variants, flash
  log, heater step table (checked against a circuit model of the
  schematic), and all 49 step transitions checked for the R3/R2 rules.
- **Simulator:** `make -C test sim` builds `test/build/sim_plain` and
  `sim_heater`. These run the firmware modules on the host and serve
  Modbus RTU on a pseudo-terminal (the path is printed at start). They log
  relay changes with timestamps. Baud rate and parity are not simulated.
- **Device test:** `tools/test_device.py --port /dev/ttyUSB0` (needs
  pyserial). It detects the variant and checks common registers,
  exceptions, broadcast, the other-slave case, the pulse timer, the comm
  timeout, the heater step walk and the hold time. `--serial-test` cycles
  through baud rates (`--bauds`) and parities (`--parities`) and then
  restores the original settings. `--echo` is for adapters that receive
  their own transmission. **The test switches the relays: disconnect the
  loads first.**
