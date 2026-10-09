# Implementation Guide

Open-source replacement firmware for the Waveshare **Modbus RTU Relay 4CH**.
Goal: behave like the stock firmware described in the
[Waveshare wiki](https://www.waveshare.com/wiki/Modbus_RTU_Relay_4CH).

## 1. Hardware

| Item          | Detail                                                        |
|---------------|---------------------------------------------------------------|
| MCU           | STM32F030C8T6 (64 KiB flash, 8 KiB RAM, Cortex-M0)            |
| Clock         | 8 MHz HSE, PLL x6 = 48 MHz SYSCLK/HCLK/PCLK                   |
| Relays        | REL_1..REL_4 = PB0..PB3, **active low** (SET = off)           |
| Status LED    | LED_STATE = PC13, **active low**                              |
| RS485         | USART1 TX = PA9, RX = PA10, isolated transceiver              |
| RS485 dir.    | No MCU-controlled DE/RE: the transceiver switches direction itself |
| Bus idle bias | A-B = 3.5 V idle without termination (strong fail-safe bias)  |
| TXD/RXD LEDs  | Hardware driven, not connected to the MCU                     |
| SWD           | PA13 (SWDIO), PA14 (SWCLK)                                    |

The board is flashed via SWD at 0x08000000. There is no bootloader, because
the stock flash has been erased.

## 2. Protocol to Implement (stock compatible)

Serial: 8 data bits, 1 stop bit, parity none/even/odd. Default 9600 8N1.
Default slave address: 1.

### 2.1 Address handling

- A frame is accepted if its address byte equals the device address (1..255)
  or 0 (broadcast).
- Broadcast frames are executed. Stock-compatible behaviour **replies to
  broadcast frames too**: the wiki uses `00 03 40 00 00 01` to read an
  unknown device address, and the reply carries address byte `00`. This is
  controlled by `CFG_BROADCAST_REPLY` (default 1). Set it to 0 for strict
  Modbus behaviour (no replies to broadcast).

### 2.2 Coils (FC 01 / 05 / 0F)

| Address         | FC     | Value                                                | Action |
|-----------------|--------|------------------------------------------------------|--------|
| 0x0000..0x0003  | 01     | -                                                    | Read relay state, bit0 = CH1 |
| 0x0000..0x0003  | 05     | FF00 on, 0000 off, 5500 toggle                       | Set one relay |
| 0x0000..0x0003  | 0F     | bit = state                                          | Set relays |
| 0x00FF          | 05     | FF00 all on, 0000 all off, 5500 all toggle           | All relays |
| 0x0100..0x0103  | 05     | FF00 toggle, 0000 no change                          | Toggle one relay |
| 0x0100..0x0103  | 0F     | bit 1 = toggle, 0 = no change                        | Toggle relays |
| 0x01FF          | 05     | FF00 toggle all, 0000 no change                      | Toggle all |
| 0x0200..0x0203  | 05     | n = 0x0001..0x7FFF                                   | Flash on: relay on now, off after n x 100 ms |
| 0x0400..0x0403  | 05     | n = 0x0001..0x7FFF                                   | Flash off: relay off now, on after n x 100 ms |

Rules:

- An FC 05 reply echoes the request. An FC 0F reply is addr, fc, start, qty.
- Any direct set, clear or toggle of a channel cancels a running flash timer
  on that channel. A new flash command restarts the timer.
- A flash value of 0 or above 0x7FFF gives exception 03. Any other FC 05
  value not listed in the table also gives exception 03.
- FC 01: qty must be 1..4 and start + qty must be <= 4, otherwise exception 02.
  qty = 0 gives exception 03.
- FC 0F: the range must lie entirely inside 0x0000..0x0003 or entirely inside
  0x0100..0x0103, otherwise exception 02. The byte count must equal
  ceil(qty/8), otherwise exception 03.

### 2.3 Holding registers (FC 03 / 06)

| Address | Access | Content |
|---------|--------|---------|
| 0x2000  | R/W    | High byte: parity (0 none, 1 even, 2 odd). Low byte: baud index |
| 0x4000  | R/W    | Device address 0x0001..0x00FF |
| 0x8000  | R      | Firmware version x 100 (`FW_VERSION`, starts at 100 = V1.00) |

Baud index: 0=4800, 1=9600, 2=19200, 3=38400, 4=57600, 5=115200,
6=128000, 7=256000.

Rules:

- The registers are not contiguous, so FC 03 requires qty = 1 and a known
  address. Otherwise it gives exception 02.
- FC 06 to 0x8000 or to an unknown address gives exception 02. A value out of
  range gives exception 03.
- A changed setting takes effect **after the reply has been sent** with the
  old settings, and is then saved to flash.
- **Parity mapping is ambiguous in the wiki.** The "send" description says
  1 = even, 2 = odd. The "receive" description says the opposite. We use
  1 = even, 2 = odd, defined in a single table in `config.c` so it is easy
  to swap.

### 2.4 Exceptions

An exception reply is `addr, fc | 0x80, code, CRC`. Codes used:
01 = unsupported function, 02 = illegal address, 03 = illegal value,
04 = device failure (e.g. a flash write error).

Frames with a bad CRC, a parity/framing/noise/overrun error, or fewer than
4 bytes are dropped silently.

## 3. Software Architecture

### 3.1 Modbus implementation: our own, no library

We write a small slave-only Modbus RTU stack (about 400 lines) instead of
using FreeMODBUS or nanoMODBUS. Reasons:

- The stock protocol breaks the Modbus specification in ways that libraries
  reject or do not support: FC 05 with value 0x5500, replies to broadcast
  frames, and "commands" encoded as coil addresses with timer values.
  Libraries check these cases and would have to be patched.
- We only need the RTU slave side, 5 function codes and 3 registers.
- With our own stack we control the frame timing, the half-duplex RX
  blanking and the memory use (8 KiB RAM).
- The protocol core stays hardware independent, so it can be unit-tested
  on the host.

### 3.2 Modules

```
App/
  Inc/  app.h board.h crc16.h modbus.h devmap.h relay.h rs485.h config.h
  Src/  app.c       app_init(), app_loop(), LED, watchdog, factory reset
        rs485.c     USART1 + TIM16 ISR, frame RX/TX, line settings
        crc16.c     Modbus CRC16 (table based)
        modbus.c    frame check, address filter, FC decode, exception build  (portable)
        devmap.c    Waveshare coil/register map (section 2)                  (portable)
        relay.c     relay outputs, flash timers
        config.c    persistent settings in flash
test/
  Makefile      host build (gcc), no HAL
  test_main.c   wiki frames as test vectors, stubs for relay/config
```

CubeMX-generated code is only touched inside `USER CODE` blocks.
`main.c` calls `app_init()` in `USER CODE 2` and `app_loop()` in the
`while (1)` loop.

### 3.3 Build system

CubeMX overwrites the generated `Makefile` on every regeneration, so any app
sources added to it are lost. We therefore **switch the CubeMX toolchain to
CMake** (Project Manager, then Toolchain/IDE = CMake):

- CubeMX writes `cmake/stm32cubemx/CMakeLists.txt`, which it regenerates
  freely.
- The top-level `CMakeLists.txt` is written only once and is ours. It adds
  `App/` and our own linker script.
- Build commands: `cmake --preset Release && cmake --build --preset Release`.
- The project uses its own linker script copy, `App/STM32F030C8Tx_app.ld`.
  It reserves the config pages (see 3.7) and does not depend on the CubeMX
  output (the generated script was broken before).

### 3.4 RS485 receive and transmit (`rs485.c`)

The F030 USART1 has **no receiver timeout** (`RTOR`). This is confirmed by
the CMSIS header, which has no `IS_UART_RECEIVER_TIMEOUT_INSTANCE`. The
frame end (t3.5) is therefore detected with **TIM16**:

- TIM16 is clocked at 48 MHz with prescaler 47, so it counts 1 µs ticks. It
  runs in one-pulse mode with ARR = t3.5.
- The USART1 RXNE ISR (direct register access, not the HAL UART callbacks;
  at 256000 baud a byte arrives every 39 µs) stores the byte and restarts
  TIM16.
- When TIM16 expires (the update ISR fires), the frame is complete. It is
  handed to the main loop with a flag and further RX is ignored until it has
  been processed.
- t3.5 = 3.5 x 11 bit / baud for baud <= 19200. For higher rates it is a
  fixed 1750 µs, as the Modbus spec requires. The t1.5 inter-character
  check is not enforced; CRC checking covers corrupted frames.
- The USART error flags (ORE, FE, NE, PE) are cleared in the ISR and mark
  the current frame as bad.
- TX is interrupt driven (TXE, then TC). During TX the receiver is switched
  off (`CR1.RE = 0`). It is switched back on in the TC ISR, so any echo from
  the auto-direction transceiver is discarded.
- The reply starts right after processing. The t3.5 gap is already
  guaranteed, because a frame is only complete after t3.5 of silence.
- Line reconfiguration uses `HAL_UART_Init()`. With parity, the word length
  must be **9 bits** (`UART_WORDLENGTH_9B` = 8 data + parity).
- 128000 and 256000 baud at 48 MHz: BRR = 375 / 187.5. The baud error is
  at most 0.27 %, which is fine.

### 3.5 Relays and flash timers (`relay.c`)

- `relay_set(ch, on)`, `relay_toggle(ch)` and `relay_get_mask()` handle the
  active-low polarity in one place.
- Flash timers are 32-bit deadlines based on `HAL_GetTick()` (1 ms). They
  are evaluated in `relay_poll()` from the main loop, so no extra timer is
  needed.
- At power-on and after any reset, all relays are **off** (GPIO init state).

### 3.6 Status LED

- Normal operation: blink at 1 Hz (toggle every 500 ms), like the stock
  "STA flashes when the MCU is running normally".
- Factory reset done: 10 fast blinks (50 ms), then normal operation.

### 3.7 Persistent settings (`config.c`)

- The last two 1 KiB flash pages are used: 0x0800F800 and 0x0800FC00. The
  linker script limits the application to 62 KiB.
- Each record is 8 bytes (4 half-words, because the F0 programs 16 bits at a
  time): `magic(16) | addr(8) | parity(8) | baud(8) | seq(8) | crc16(16)`.
- Records are appended to the active page. When the page is full, the
  record goes to the other page, which is erased first, and then the old
  page is erased. At boot the newest valid record wins. If there is none,
  the defaults (addr 1, 9600 8N1) are used.
- A record is only written when a value actually changes.
- A page erase stalls the CPU for about 20 to 40 ms, and bytes received in
  that time may be lost. This is acceptable because it happens only after a
  settings write reply.

### 3.8 Factory reset

The stock bootloader restored the defaults when SWDIO was shorted to GND at
power-up. We keep the same procedure in the application:

- At boot, `app_init()` samples PA13 several times over about 50 ms. PA13
  stays in SWD AF mode, because IDR still reflects the pin level and SWDIO
  has an internal pull-up.
- If PA13 stays low, the defaults are written to flash and the LED confirms
  with fast blinks.

### 3.9 Watchdog

The stock firmware is gone and Waveshare does not document whether it used
a watchdog. You can check the option bytes for a hardware watchdog
(`WDG_SW` = 0), because a mass erase without an RDP change leaves them
untouched.

We use the **IWDG** anyway, because the device switches loads and a hung
MCU must not stay hung:

- Clock is LSI (about 40 kHz, 30 to 50 kHz), prescaler 32, reload 1250.
  This gives about 1 s nominal (0.8 to 1.3 s).
- The watchdog is refreshed **only in the main loop**, and only after
  `relay_poll()` and Modbus processing ran. ISRs never refresh it.
- The timeout must be well above the flash page erase time (40 ms).
- `DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP` stops the watchdog while
  the core is halted in the debugger.
- After a watchdog reset, the relays go to the safe off state (see 3.5).
  `RCC->CSR` IWDGRSTF can be read and cleared for diagnostics.

## 4. CubeMX Changes Required

To be done in CubeMX before implementation:

1. Project Manager: Toolchain/IDE = **CMake**.
2. Enable **TIM16** (activated, no channels). Prescaler and period are set
   in our code. Do **not** enable the TIM16 interrupt in NVIC, because the
   ISR is ours.
3. Enable **IWDG**. Settings are made in code, or prescaler 32 and reload
   1250. Do not start it from CubeMX init if possible.
4. Leave the USART1 global interrupt **disabled** in NVIC. Our code defines
   `USART1_IRQHandler` and enables the IRQ.

Our code owns these IRQ handlers. CubeMX must not generate them, otherwise
there are duplicate symbols.

## 5. Implementation Steps

1. CubeMX changes (section 4), top-level CMake, our linker script,
   `.gitignore`. The empty `app_init()` and `app_loop()` must build.
2. `relay.c` and the LED blink. Check: blink pattern and relay walk test.
3. `crc16.c`, `modbus.c` and `devmap.c`, plus host tests in `test/` with all
   wiki frames. CRCs are computed by the test, because some wiki CRCs are
   wrong.
4. `rs485.c`: RX with TIM16 frame timing, TX with RX blanking. Check
   against `mbpoll` or pymodbus at 9600.
5. `config.c` and the settings registers: change baud, parity and address,
   power-cycle, and check that they are kept.
6. Flash timers, factory reset, IWDG.
7. Test script `tools/test_device.py` (pymodbus). It runs the full register
   map at all 8 baud rates and all 3 parities, including broadcast, the
   exception paths and the flash timing.

## 6. Open Points

- Parity mapping 1/2 (see 2.3). Verify against a stock unit if one becomes
  available.
- Whether the stock firmware replies to broadcast *write* frames. Default:
  yes (`CFG_BROADCAST_REPLY`).
- Stock behaviour when FC 05 uses value 0 for flash on/off. We use
  exception 03.
