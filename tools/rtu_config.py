#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Read and change the communication settings of the rtu-relais-4ch
firmware (needs pyserial).

--addr, --baud and --parity are the settings the device uses now. "set"
stages the new values, applies them, follows the device to the new
settings and checks that it answers there. "scan" finds a device whose
settings are unknown.

Examples:
  tools/rtu_config.py --port /dev/ttyUSB0 info
  tools/rtu_config.py --port /dev/ttyUSB0 set --new-addr 5 --new-baud 38400 --new-parity E
  tools/rtu_config.py --port /dev/ttyUSB0 --addr 5 --baud 38400 --parity E reset-defaults
  tools/rtu_config.py --port /dev/ttyUSB0 scan
  tools/rtu_config.py --port /dev/ttyUSB0 scan --all-addrs
"""

import argparse
import sys

import serial

from rtu_modbus import (ADDR_MAX, ADDR_MIN, BAUD_RATES, CMD_DEFAULTS, DEFAULT_ADDR, DEFAULT_BAUD, DEFAULT_PARITY,
                        HREG_COMM_TIMEOUT, HREG_COMMAND, HREG_SERIAL, IREG_CRC_ERRORS, IREG_PROFILE, IREG_STATUS,
                        IREG_UART_ERRORS, IREG_UPTIME_HI, IREG_VERSION, PARITY_TO_REG, PROFILE_NAMES, REG_TO_PARITY,
                        STATUS_COMM_TIMEOUT, STATUS_DEFAULTS, STATUS_FLASH_ERROR, STATUS_SAVE_PENDING,
                        STATUS_WDT_RESET, Client, ModbusError)

STATUS_TEXT = [
    (STATUS_COMM_TIMEOUT, "comm timeout tripped"),
    (STATUS_DEFAULTS, "settings are the defaults"),
    (STATUS_WDT_RESET, "last reset by watchdog"),
    (STATUS_FLASH_ERROR, "FLASH ERROR"),
    (STATUS_SAVE_PENDING, "settings write pending"),
    (0x0100, "hold active (heater)"),
    (0x0200, "switching sequence (heater)"),
]
FACTORY_RESET_HINT = "factory reset: short SWDIO (PA13) to GND at power-up"
REPLY_TIMEOUT_S = 0.3
SCAN_TIMEOUT_S = 0.1
SECONDS_PER_MINUTE = 60
SECONDS_PER_HOUR = 60 * SECONDS_PER_MINUTE
SECONDS_PER_DAY = 24 * SECONDS_PER_HOUR


def line_str(baud, parity):
    return f"{baud} 8{parity}1"


def uptime_str(s):
    d, s = divmod(s, SECONDS_PER_DAY)
    h, s = divmod(s, SECONDS_PER_HOUR)
    m, s = divmod(s, SECONDS_PER_MINUTE)
    return f"{d}d {h:02}:{m:02}:{s:02}"


def addr_arg(v):
    a = int(v, 0)
    if not ADDR_MIN <= a <= ADDR_MAX:
        raise argparse.ArgumentTypeError(f"address must be {ADDR_MIN}..{ADDR_MAX}")
    return a


def parity_arg(v):
    p = v.upper()
    if p not in PARITY_TO_REG:
        raise argparse.ArgumentTypeError("parity must be N, E or O")
    return p


def baud_list(v):
    bauds = [int(b) for b in v.split(",")]
    for b in bauds:
        if b not in BAUD_RATES:
            raise argparse.ArgumentTypeError(f"unsupported baud rate {b}")
    return bauds


def parity_list(v):
    return [parity_arg(p) for p in v]


def answers(c):
    """True if the device answers at the client's current settings."""
    try:
        return c.ireg(IREG_PROFILE) in PROFILE_NAMES
    except (ModbusError, serial.SerialException, OSError):
        return False


def wait_saved(c):
    waited = c.wait_save_done(on_wait=lambda: print("settings write deferred by the flash rate limit, waiting "
                                                      "(do not power off)"))
    if waited is None:
        print("error: settings write still pending, do not power off yet")
        return False
    if c.ireg(IREG_STATUS) & STATUS_FLASH_ERROR:
        print("error: flash write error (status bit 3)")
        return False
    return True


def cmd_info(c, args):
    ver = c.ireg(IREG_VERSION)
    prof = c.ireg(IREG_PROFILE)
    status = c.ireg(IREG_STATUS)
    up_hi, up_lo, crc_err, uart_err = c.ireg(IREG_UPTIME_HI, 4)
    s_addr, s_baud, s_par = c.hreg(HREG_SERIAL, 3)
    timeout = c.hreg(HREG_COMM_TIMEOUT)

    print(f"firmware      {ver >> 8}.{ver & 0xFF}")
    print(f"variant       {PROFILE_NAMES.get(prof, f'unknown ({prof})')}")
    print(f"serial        address {c.addr}, {line_str(c.baud, c.parity)}")
    staged = (s_addr, BAUD_RATES[s_baud] if s_baud < len(BAUD_RATES) else None, REG_TO_PARITY.get(s_par))
    if staged != (c.addr, c.baud, c.parity):
        print(f"staged        address {staged[0]}, {line_str(staged[1], staged[2])} (not applied)")
    print(f"comm timeout  {f'{timeout} s' if timeout else 'off'}")
    flags = [t for bit, t in STATUS_TEXT if status & bit]
    print(f"status        0x{status:04X}{' (' + ', '.join(flags) + ')' if flags else ''}")
    print(f"uptime        {uptime_str(up_hi << 16 | up_lo)}")
    print(f"CRC errors    {crc_err}")
    print(f"UART errors   {uart_err}")
    return 0


def cmd_set(c, args):
    new = (args.new_addr if args.new_addr is not None else c.addr,
           args.new_baud if args.new_baud is not None else c.baud,
           args.new_parity if args.new_parity is not None else c.parity)
    old = (c.addr, c.baud, c.parity)
    if new == old:
        print("nothing to change")
        return 0
    c.ireg(IREG_PROFILE)  # check the connection before changing anything
    print(f"address {old[0]}, {line_str(old[1], old[2])} -> address {new[0]}, {line_str(new[1], new[2])}")
    c.stage_serial(*new)
    try:
        c.apply_serial(*new)
    except serial.SerialException as e:
        print(f"error: {e}\nThe device may already use the new settings.")
        return 1
    if not answers(c):
        c.addr = old[0]
        c.reopen(old[1], old[2])
        if answers(c):
            print("error: no reply at the new settings, the device still uses the old ones")
        else:
            print("error: no reply at the new or the old settings. Try 'scan', or " + FACTORY_RESET_HINT)
        return 1
    if c.hreg(HREG_SERIAL, 3) != [new[0], BAUD_RATES.index(new[1]), PARITY_TO_REG[new[2]]]:
        print("error: the device answers, but reports different serial settings")
        return 1
    if not wait_saved(c):
        return 1
    print("ok")
    return 0


def cmd_reset_defaults(c, args):
    if not args.yes:
        ans = input("Restore ALL settings (serial, comm timeout, heater settings) to the defaults? [y/N] ")
        if ans.lower() != "y":
            return 1
    c.ireg(IREG_PROFILE)
    c.write_reg(HREG_COMMAND, CMD_DEFAULTS)  # reply with the old settings
    c.addr = DEFAULT_ADDR
    c.reopen(DEFAULT_BAUD, DEFAULT_PARITY)
    if not answers(c):
        print("error: no reply at the defaults. Try 'scan', or " + FACTORY_RESET_HINT)
        return 1
    if not wait_saved(c):
        return 1
    print(f"ok, now address {DEFAULT_ADDR}, {line_str(DEFAULT_BAUD, DEFAULT_PARITY)}")
    return 0


def cmd_scan(c, args):
    # most likely settings first: the ones given, then the defaults
    lines = [(c.baud, c.parity), (DEFAULT_BAUD, DEFAULT_PARITY)]
    lines += [(b, p) for b in args.bauds for p in args.parities]
    lines = list(dict.fromkeys(lines))
    if args.all_addrs:
        addrs = list(dict.fromkeys([c.addr, DEFAULT_ADDR] + list(range(ADDR_MIN, ADDR_MAX + 1))))
    else:
        addrs = [c.addr]
    c.retry = False
    found = []
    for baud, par in lines:
        try:
            c.reopen(baud, par)
        except (serial.SerialException, OSError, ValueError) as e:
            print(f"{line_str(baud, par)}: skipped ({e})")
            continue
        print(f"{line_str(baud, par)} ...", end="", flush=True)
        hits = []
        for a in addrs:
            c.addr = a
            if answers(c):
                hits.append(a)
                if not args.all:
                    break
        print(" " + (", ".join(f"address {a}" for a in hits) if hits else "-"))
        found += [(a, baud, par) for a in hits]
        if found and not args.all:
            break
    if not found:
        hint = "" if args.all_addrs else "try --all-addrs, or "
        print(f"no device found, {hint}{FACTORY_RESET_HINT}")
        return 1
    for a, baud, par in found:
        print(f"found: --addr {a} --baud {baud} --parity {par}")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True)
    ap.add_argument("--addr", type=addr_arg, default=DEFAULT_ADDR, help="current address (default %(default)s)")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD, choices=BAUD_RATES,
                    help="current baud rate (default %(default)s)")
    ap.add_argument("--parity", type=parity_arg, default=DEFAULT_PARITY, help="current parity N/E/O (default N)")
    ap.add_argument("--timeout", type=float,
                    help=f"reply timeout in s (default {REPLY_TIMEOUT_S}, scan {SCAN_TIMEOUT_S})")
    ap.add_argument("--echo", action="store_true", help="the RS485 adapter echoes its own transmission")
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("info", help="show version, variant, settings, status and counters")

    p = sub.add_parser("set", help="change address, baud rate and parity")
    p.add_argument("--new-addr", type=addr_arg, help=f"{ADDR_MIN}..{ADDR_MAX}")
    p.add_argument("--new-baud", type=int, choices=BAUD_RATES)
    p.add_argument("--new-parity", type=parity_arg, help="N, E or O")

    p = sub.add_parser("reset-defaults", help=f"restore all settings (address {DEFAULT_ADDR}, "
                                              f"{line_str(DEFAULT_BAUD, DEFAULT_PARITY)})")
    p.add_argument("--yes", action="store_true", help="do not ask")

    p = sub.add_parser("scan", help="find the device: tries all baud rates and parities")
    p.add_argument("--all-addrs", action="store_true",
                   help=f"try every address {ADDR_MIN}..{ADDR_MAX}, not only --addr (slow)")
    p.add_argument("--all", action="store_true", help="do not stop at the first device")
    p.add_argument("--bauds", type=baud_list, default=BAUD_RATES,
                   help="comma separated baud rates (default: all)")
    p.add_argument("--parities", type=parity_list, default=list(PARITY_TO_REG), help="default NEO")

    args = ap.parse_args()
    if args.timeout is None:
        args.timeout = SCAN_TIMEOUT_S if args.cmd == "scan" else REPLY_TIMEOUT_S
    cmds = {"info": cmd_info, "set": cmd_set, "reset-defaults": cmd_reset_defaults, "scan": cmd_scan}
    try:
        c = Client(args.port, args.baud, args.parity, args.addr, args.timeout, args.echo)
        try:
            return cmds[args.cmd](c, args)
        finally:
            c.close()
    except (ModbusError, serial.SerialException, OSError) as e:
        print(f"error: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
