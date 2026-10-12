# SPDX-License-Identifier: GPL-3.0-or-later
"""Raw Modbus RTU client and register constants for the rtu-relais-4ch
firmware, shared by test_device.py and rtu_config.py (needs pyserial)."""

import struct
import time

import serial

try:
    import termios
    PORT_CONFIG_ERRORS = (termios.error,)
except ImportError:  # not POSIX
    PORT_CONFIG_ERRORS = ()

BAUD_RATES = [4800, 9600, 19200, 38400, 57600, 115200, 128000, 256000]
PARITY_TO_REG = {"N": 0, "E": 1, "O": 2}
REG_TO_PARITY = {v: k for k, v in PARITY_TO_REG.items()}

DEFAULT_ADDR = 1
DEFAULT_BAUD = 9600
DEFAULT_PARITY = "N"
ADDR_MIN = 1
ADDR_MAX = 247

PROFILE_PLAIN = 1
PROFILE_HEATER = 2
PROFILE_NAMES = {PROFILE_PLAIN: "plain", PROFILE_HEATER: "heater"}

# Input registers
IREG_VERSION = 0x0000
IREG_PROFILE = 0x0001
IREG_STATUS = 0x0003
IREG_UPTIME_HI = 0x0004
IREG_CRC_ERRORS = 0x0006
IREG_UART_ERRORS = 0x0007

# Holding registers
HREG_COMM_TIMEOUT = 0x0000
HREG_SERIAL = 0x0100  # address, baud index, parity (staged)
HREG_COMMAND = 0x010F
CMD_APPLY_SERIAL = 1
CMD_DEFAULTS = 2

STATUS_COMM_TIMEOUT = 0x0001
STATUS_DEFAULTS = 0x0002
STATUS_WDT_RESET = 0x0004
STATUS_FLASH_ERROR = 0x0008
STATUS_SAVE_PENDING = 0x0010

# Holding registers saved to flash. A flash write can stall the UART (page
# erase, 20 to 40 ms), so wait a bit after writing them.
PERSISTENT_REGS = {HREG_COMM_TIMEOUT, 0x0011, 0x0012, HREG_COMMAND}
FLASH_SETTLE_S = 0.1
# The firmware limits the flash write rate (32 in a burst, then one per
# minute) and shows a deferred write in status bit 4.
SAVE_PENDING_MAX_S = 70
PORT_SETTLE_S = 0.05


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
    def __init__(self, port, baud, parity, addr, timeout, echo=False, retry=True):
        self.port_name = port
        self.addr = addr
        self.timeout = timeout
        self.echo = echo
        self.retry = retry
        self.retries = 0
        self.ser = None
        self.reopen(baud, parity)

    def reopen(self, baud, parity):
        if self.ser:
            self.ser.close()
        self.ser = None
        try:
            self.ser = serial.Serial(self.port_name, baudrate=baud, parity=parity, bytesize=8, stopbits=1,
                                     timeout=self.timeout)
        except PORT_CONFIG_ERRORS as e:
            # e.g. a pty without parity support
            raise serial.SerialException(f"cannot set {baud} 8{parity}1 on {self.port_name}: {e}") from e
        self.baud = baud
        self.parity = parity
        time.sleep(PORT_SETTLE_S)

    def close(self):
        if self.ser:
            self.ser.close()
            self.ser = None

    def request(self, pdu: bytes, addr=None, expect_reply=True):
        """Send a PDU, return the reply PDU (None if no reply expected/received).

        Retries once if no reply arrives, like a Modbus master would.
        """
        try:
            return self._request(pdu, addr, expect_reply)
        except NoReply:
            if not self.retry:
                raise
            self.retries += 1
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

    # serial settings
    def stage_serial(self, addr, baud, parity):
        self.write_regs(HREG_SERIAL, [addr, BAUD_RATES.index(baud), PARITY_TO_REG[parity]])

    def apply_serial(self, addr, baud, parity):
        """Write the apply command (the reply still uses the old settings),
        then follow the device to the new settings."""
        self.write_reg(HREG_COMMAND, CMD_APPLY_SERIAL)
        self.addr = addr
        self.reopen(baud, parity)

    def wait_save_done(self, max_s=SAVE_PENDING_MAX_S, on_wait=None):
        """Wait while a settings write is deferred by the rate limit.
        Returns the seconds waited, or None if it is still pending."""
        t0 = time.monotonic()
        waited = False
        while self.ireg(IREG_STATUS) & STATUS_SAVE_PENDING:
            if not waited and on_wait:
                on_wait()
            waited = True
            if time.monotonic() - t0 > max_s:
                return None
            time.sleep(1)
        return time.monotonic() - t0
