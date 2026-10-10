#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""On-device test for the rtu-relais-4ch firmware (plain and heater variant).

Talks raw Modbus RTU over pyserial, so broadcast, ignored frames and
exception replies can be checked exactly.

WARNING: the test switches the relays. Disconnect loads (heating elements!)
before running it.

Examples:
  tools/test_device.py --port /dev/ttyUSB0
  tools/test_device.py --port /dev/ttyUSB0 --serial-test
  tools/test_device.py --port /dev/ttyUSB0 --addr 5 --baud 38400 --parity E
"""

import argparse
import struct
import sys
import time

import serial

BAUD_RATES = [4800, 9600, 19200, 38400, 57600, 115200, 128000, 256000]
PARITY_TO_REG = {"N": 0, "E": 1, "O": 2}
REG_TO_PARITY = {v: k for k, v in PARITY_TO_REG.items()}

PROFILE_PLAIN = 1
PROFILE_HEATER = 2

# Holding registers saved to flash. A flash write can stall the UART (page
# erase, 20 to 40 ms), so wait a bit after writing them.
PERSISTENT_REGS = {0x0000, 0x0011, 0x0012, 0x010F}
FLASH_SETTLE_S = 0.1

HEATER_STEP_MASK = [0x0, 0x6, 0x1, 0xE, 0x3, 0xF, 0xB]
HEATER_STEP_POWER = [0, 167, 334, 500, 667, 834, 1000]


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


class ModbusError(Exception):
    pass


class NoReply(ModbusError):
    pass


class Exc(ModbusError):
    """Modbus exception reply"""

    def __init__(self, code):
        super().__init__(f"exception {code}")
        self.code = code


class Client:
    def __init__(self, port, baud, parity, addr, timeout, echo=False):
        self.port_name = port
        self.addr = addr
        self.timeout = timeout
        self.echo = echo
        self.ser = None
        self.reopen(baud, parity)

    def reopen(self, baud, parity):
        if self.ser:
            self.ser.close()
        self.ser = serial.Serial(self.port_name, baudrate=baud, parity=parity, bytesize=8, stopbits=1,
                                 timeout=self.timeout)
        self.baud = baud
        self.parity = parity
        time.sleep(0.05)

    def request(self, pdu: bytes, addr=None, expect_reply=True):
        """Send a PDU, return the reply PDU (None if no reply expected/received).

        Retries once if no reply arrives, like a Modbus master would.
        """
        try:
            return self._request(pdu, addr, expect_reply)
        except NoReply:
            global retries
            retries += 1
            print(f"  note  no reply to FC {pdu[0]:02X}, retrying")
            return self._request(pdu, addr, expect_reply)

    def _request(self, pdu, addr, expect_reply):
        addr = self.addr if addr is None else addr
        frame = bytes([addr]) + pdu
        c = crc16(frame)
        frame += bytes([c & 0xFF, c >> 8])
        self.ser.reset_input_buffer()
        self.ser.write(frame)
        self.ser.flush()

        if self.echo:
            # adapter echoes its own transmission: drop it
            self.ser.read(len(frame))

        rx = b""
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            chunk = self.ser.read(self.ser.in_waiting or 1)
            if chunk:
                rx += chunk
                if self._complete(rx):
                    break
        if not rx:
            if expect_reply:
                raise NoReply("no reply")
            return None
        if not expect_reply:
            raise ModbusError(f"unexpected reply {rx.hex()}")
        if len(rx) < 5 or crc16(rx[:-2]) != (rx[-2] | rx[-1] << 8):
            raise ModbusError(f"bad reply {rx.hex()}")
        if rx[0] != addr:
            raise ModbusError(f"reply from address {rx[0]}")
        pdu = rx[1:-2]
        if pdu[0] & 0x80:
            raise Exc(pdu[1])
        return pdu

    @staticmethod
    def _complete(rx):
        if len(rx) < 5:
            return False
        fc = rx[1]
        if fc & 0x80:
            n = 5
        elif fc in (1, 2, 3, 4):
            n = 5 + rx[2]
        elif fc in (5, 6, 15, 16):
            n = 8
        else:
            return crc16(rx[:-2]) == (rx[-2] | rx[-1] << 8)
        return len(rx) >= n

    # helpers
    def read_regs(self, fc, addr, qty=1, unit=None):
        pdu = self.request(struct.pack(">BHH", fc, addr, qty), unit)
        return list(struct.unpack(f">{qty}H", pdu[2:2 + 2 * qty]))

    def hreg(self, addr, qty=1):
        r = self.read_regs(3, addr, qty)
        return r[0] if qty == 1 else r

    def ireg(self, addr, qty=1):
        r = self.read_regs(4, addr, qty)
        return r[0] if qty == 1 else r

    def write_reg(self, addr, val, unit=None, expect_reply=True):
        r = self.request(struct.pack(">BHH", 6, addr, val), unit, expect_reply)
        self._settle(addr, 1)
        return r

    def write_regs(self, addr, vals, unit=None):
        r = self.request(struct.pack(f">BHHB{len(vals)}H", 16, addr, len(vals), 2 * len(vals), *vals), unit)
        self._settle(addr, len(vals))
        return r

    @staticmethod
    def _settle(addr, qty):
        if PERSISTENT_REGS & set(range(addr, addr + qty)):
            time.sleep(FLASH_SETTLE_S)

    def write_coil(self, addr, val, unit=None, expect_reply=True):
        return self.request(struct.pack(">BHH", 5, addr, val), unit, expect_reply)

    def write_coils(self, addr, qty, bits, unit=None):
        return self.request(struct.pack(">BHHBB", 15, addr, qty, 1, bits), unit)

    def read_coils(self, addr=0, qty=4):
        return self.request(struct.pack(">BHH", 1, addr, qty))[2]


# ---- test framework --------------------------------------------------------

failures = 0
retries = 0


def check(cond, msg):
    global failures
    if cond:
        print(f"  ok    {msg}")
    else:
        print(f"  FAIL  {msg}")
        failures += 1


def expect_exc(code, fn, msg):
    try:
        fn()
    except Exc as e:
        check(e.code == code, f"{msg}: exception {e.code} (expected {code})")
        return
    except ModbusError as e:
        check(False, f"{msg}: {e} (expected exception {code})")
        return
    check(False, f"{msg}: no exception (expected {code})")


# ---- tests -----------------------------------------------------------------

def test_common(c):
    print("common registers")
    ver, prof, mask, status, up_hi, up_lo, crc_err, uart_err = c.ireg(0, 8)
    print(f"  version {ver >> 8}.{ver & 0xFF}, variant {prof}, relays 0x{mask:X}, status 0x{status:04X}, "
          f"uptime {(up_hi << 16) | up_lo} s, crc errors {crc_err}, uart errors {uart_err}")
    check(prof in (PROFILE_PLAIN, PROFILE_HEATER), "variant id")
    time.sleep(1.1)
    up2 = c.ireg(4, 2)
    check(((up2[0] << 16) | up2[1]) > ((up_hi << 16) | up_lo), "uptime counts")

    addr, baud, par = c.hreg(0x100, 3)
    check(addr == c.addr, f"slave address register = {addr}")
    check(BAUD_RATES[baud] == c.baud, f"baud register = {BAUD_RATES[baud]}")
    check(REG_TO_PARITY.get(par) == c.parity, f"parity register = {par}")

    pdu = c.request(bytes([0x2B, 0x0E, 0x01, 0x00]))
    check(b"rtu-relais-4ch" in pdu, f"device identification ({pdu[7:].decode(errors='replace')!r})")

    expect_exc(2, lambda: c.hreg(0x0001), "unknown holding register")
    expect_exc(2, lambda: c.ireg(0x0008), "unknown input register")
    expect_exc(1, lambda: c.request(struct.pack(">BHH", 2, 0, 1)), "FC 02 not supported")
    expect_exc(3, lambda: c.write_reg(0x010F, 7), "invalid command")
    expect_exc(3, lambda: c.write_reg(0x0100, 0), "slave address 0")
    expect_exc(3, lambda: c.write_reg(0x0101, 8), "baud index 8")

    other = 247 if c.addr != 247 else 246
    check(c.request(struct.pack(">BHH", 4, 0, 1), addr=other, expect_reply=False) is None,
          "other slave address ignored")
    return prof


def restore(fn):
    """Restore settings after a test, also after an abort."""
    try:
        fn()
    except ModbusError as e:
        print(f"  WARN  could not restore the settings: {e}")


def test_plain(c):
    print("plain variant")
    old_timeout = c.hreg(0x0000)
    try:
        _test_plain(c)
    finally:
        restore(lambda: (c.write_reg(0x0000, old_timeout), c.write_reg(0x0010, 0)))


def _test_plain(c):
    c.write_reg(0x0010, 0)
    c.write_coil(0, 0xFF00)
    check(c.read_coils() == 0x01, "FC 05 relay 1 on")
    c.write_coils(0, 4, 0x0A)
    check(c.read_coils() == 0x0A, "FC 0F relays 2+4")
    check(c.ireg(2) == 0x0A, "relay mask input register")
    expect_exc(3, lambda: c.write_coil(0, 0x5500), "stock toggle value rejected")
    expect_exc(2, lambda: c.write_coil(4, 0xFF00), "coil 4 does not exist")

    c.write_reg(0x0010, 0x05)
    check(c.read_coils() == 0x05, "relay mask register")
    expect_exc(3, lambda: c.write_reg(0x0010, 0x10), "mask out of range")

    # broadcast: executed, no reply
    check(c.write_coil(3, 0xFF00, unit=0, expect_reply=False) is None, "broadcast: no reply")
    time.sleep(0.05)
    check(c.read_coils() == 0x0D, "broadcast executed")

    # pulse timer
    c.write_reg(0x0010, 0)
    c.write_reg(0x0020, 10)  # 1 s
    t0 = time.monotonic()
    check(c.read_coils() & 1, "pulse: relay 1 on")
    remaining = c.hreg(0x0020)
    check(8 <= remaining <= 10, f"pulse: remaining {remaining} x 100 ms")
    while c.read_coils() & 1 and time.monotonic() - t0 < 3:
        time.sleep(0.02)
    dt = time.monotonic() - t0
    check(0.9 <= dt <= 1.2, f"pulse: relay off after {dt:.2f} s")

    # comm timeout
    c.write_reg(0x0000, 2)
    c.write_reg(0x0010, 0x0F)
    time.sleep(1.5)
    check(c.read_coils() == 0x0F, "comm timeout: still on after 1.5 s (reads only)")
    time.sleep(1.0)
    check(c.read_coils() == 0x00, "comm timeout: relays off after 2 s")
    check(c.ireg(3) & 0x0001, "comm timeout: status bit 0")
    c.write_coil(0, 0x0000)
    check(not (c.ireg(3) & 0x0001), "comm timeout: cleared by a control write")


def test_heater(c):
    print("heater variant")
    old_hold, old_hyst = c.hreg(0x0011, 2)
    old_timeout = c.hreg(0x0000)
    try:
        _test_heater(c)
    finally:
        restore(lambda: (c.write_reg(0x0010, 0), c.write_regs(0x0011, [old_hold, old_hyst]),
                         c.write_reg(0x0000, old_timeout)))


def _test_heater(c):
    expect_exc(1, lambda: c.write_coil(0, 0xFF00), "coil writes not supported")
    expect_exc(3, lambda: c.write_reg(0x0010, 1001), "setpoint above 100 %")

    c.write_regs(0x0011, [0, 0])  # no hold time for the step walk
    c.write_reg(0x0000, 10)

    for step in list(range(7)) + list(range(5, -1, -1)):
        c.write_reg(0x0010, HEATER_STEP_POWER[step])
        time.sleep(0.3)
        power, actual, target = c.ireg(0x0010, 3)
        mask = c.read_coils()
        check(actual == step and target == step and power == HEATER_STEP_POWER[step]
              and mask == HEATER_STEP_MASK[step],
              f"step {step}: power {power}, relays 0x{mask:X}")

    # hold time
    c.write_reg(0x0011, 5)
    c.write_reg(0x0010, 0)
    time.sleep(0.3)
    c.write_reg(0x0010, 500)
    time.sleep(0.3)
    check(c.ireg(0x0011) == 0 and c.ireg(3) & 0x0100, "hold time: step up delayed, status bit 8")
    t0 = time.monotonic()
    while c.ireg(0x0011) != 3 and time.monotonic() - t0 < 8:
        c.write_reg(0x0010, 500)
        time.sleep(0.2)
    dt = time.monotonic() - t0
    check(c.ireg(0x0011) == 3 and 4.0 <= dt <= 6.0, f"hold time: stepped up after {dt:.1f} s")
    c.write_reg(0x0010, 167)
    time.sleep(0.3)
    check(c.ireg(0x0011) == 1, "hold time: step down immediate")

    # comm timeout
    c.write_reg(0x0000, 2)
    c.write_reg(0x0010, 333)
    time.sleep(2.5)
    check(c.ireg(0x0011) == 0 and c.read_coils() == 0, "comm timeout: step 0")
    check(c.ireg(3) & 0x0001, "comm timeout: status bit 0")


def test_serial(c, bauds, parities):
    print("serial settings")
    orig = (c.addr, c.baud, c.parity)
    new_addr = 200 if c.addr != 200 else 201
    combos = [(b, p) for b in bauds for p in parities]
    for baud, par in combos:
        c.write_regs(0x0100, [new_addr, BAUD_RATES.index(baud), PARITY_TO_REG[par]])
        c.write_reg(0x010F, 1)  # reply with the old settings
        c.addr = new_addr
        try:
            c.reopen(baud, par)
            ok = c.ireg(1) in (PROFILE_PLAIN, PROFILE_HEATER)
        except (ModbusError, serial.SerialException, OSError) as e:
            ok = False
            print(f"    {e}")
        check(ok, f"addr {new_addr} {baud} 8{par}1")
        if not ok:
            print("  stopping: device may need a factory reset (SWDIO to GND at power-up)")
            return
        new_addr = 200 if new_addr != 200 else 201
    # restore
    c.write_regs(0x0100, [orig[0], BAUD_RATES.index(orig[1]), PARITY_TO_REG[orig[2]]])
    c.write_reg(0x010F, 1)
    c.addr = orig[0]
    c.reopen(orig[1], orig[2])
    check(c.ireg(1) in (PROFILE_PLAIN, PROFILE_HEATER), "original settings restored")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True)
    ap.add_argument("--addr", type=int, default=1)
    ap.add_argument("--baud", type=int, default=9600, choices=BAUD_RATES)
    ap.add_argument("--parity", default="N", choices="NEO")
    ap.add_argument("--timeout", type=float, default=0.3, help="reply timeout in s")
    ap.add_argument("--echo", action="store_true", help="the RS485 adapter echoes its own transmission")
    ap.add_argument("--serial-test", action="store_true", help="cycle through baud rates and parities")
    ap.add_argument("--bauds", default="4800,9600,19200,38400,57600,115200",
                    help="baud rates for --serial-test (128000/256000 need adapter support)")
    ap.add_argument("--parities", default="NEO", help="parities for --serial-test")
    ap.add_argument("--yes", action="store_true", help="do not ask before switching relays")
    args = ap.parse_args()

    if not args.yes:
        ans = input("The test switches the relays. Loads disconnected? [y/N] ")
        if ans.lower() != "y":
            return 1

    c = Client(args.port, args.baud, args.parity, args.addr, args.timeout, args.echo)
    prof = test_common(c)
    if prof == PROFILE_PLAIN:
        test_plain(c)
    elif prof == PROFILE_HEATER:
        test_heater(c)
    if args.serial_test:
        test_serial(c, [int(b) for b in args.bauds.split(",")], args.parities)

    if retries:
        print(f"{retries} request(s) needed a retry")
    print("all passed" if failures == 0 else f"{failures} FAILURE(S)")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
