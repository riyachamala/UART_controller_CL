#!/usr/bin/env python3
"""
uart_host.py -- PC side of the UART controller FPGA demo.

Requires pyserial:  pip install pyserial

Echo mode (sw[15] = 0):
    uart_host.py -p /dev/ttyUSB1 echo -n 100000         # integrity + throughput

Command mode (sw[15] = 1):
    uart_host.py -p /dev/ttyUSB1 ping                   # ID / version
    uart_host.py -p /dev/ttyUSB1 leds 0xA5              # drive LEDs
    uart_host.py -p /dev/ttyUSB1 switches               # read slide switches
    uart_host.py -p /dev/ttyUSB1 read 0x05
    uart_host.py -p /dev/ttyUSB1 write 0x05 0x3C
    uart_host.py -p /dev/ttyUSB1 status [--clear]       # error flags + RX counter
    uart_host.py -p /dev/ttyUSB1 chase                  # LED animation

Other:
    uart_host.py ports                                  # list serial ports

`-p` accepts any pyserial URL, e.g. `loop://` to try the echo test without
hardware.
"""
import argparse
import os
import sys
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    sys.exit("pyserial is required:  pip install pyserial")

# ---- register map (must match rtl/uart_demo_top.v) --------------------------
REG_ID, REG_VERSION, REG_LED, REG_SW_LO, REG_SW_HI = 0x00, 0x01, 0x02, 0x03, 0x04
REG_SCRATCH, REG_RXCNT_LO, REG_RXCNT_HI, REG_STATUS = 0x05, 0x06, 0x07, 0x08
EXPECTED_ID = 0x55


class DemoBoard:
    """Binary register protocol: 'W' addr data -> 'K', 'R' addr -> data."""

    def __init__(self, ser):
        self.ser = ser
        self.ser.reset_input_buffer()

    def _reply(self):
        b = self.ser.read(1)
        if not b:
            raise TimeoutError("no reply from board -- is sw[15] (command mode) on?")
        return b[0]

    def read(self, addr):
        self.ser.write(bytes([ord("R"), addr & 0xFF]))
        return self._reply()

    def write(self, addr, value):
        self.ser.write(bytes([ord("W"), addr & 0xFF, value & 0xFF]))
        r = self._reply()
        if r != ord("K"):
            raise IOError(f"write to 0x{addr:02X} not acknowledged (got 0x{r:02X})")


def auto_int(s):
    return int(s, 0)


# ---- commands ----------------------------------------------------------------
def cmd_ports(_args):
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("no serial ports found")
    for p in ports:
        print(f"{p.device:<28} {p.description}")


def cmd_ping(args):
    b = DemoBoard(args.ser)
    ident, ver = b.read(REG_ID), b.read(REG_VERSION)
    ok = "OK" if ident == EXPECTED_ID else "UNEXPECTED"
    print(f"ID      0x{ident:02X} ('{chr(ident) if 32 <= ident < 127 else '?'}')  {ok}")
    print(f"VERSION {ver >> 4}.{ver & 0xF}")
    return 0 if ident == EXPECTED_ID else 1


def cmd_read(args):
    print(f"0x{args.addr:02X} = 0x{DemoBoard(args.ser).read(args.addr):02X}")


def cmd_write(args):
    DemoBoard(args.ser).write(args.addr, args.value)
    print(f"0x{args.addr:02X} <= 0x{args.value:02X}  ack")


def cmd_leds(args):
    DemoBoard(args.ser).write(REG_LED, args.value)
    print(f"LED[7:0] = {args.value:08b}")


def cmd_switches(args):
    b = DemoBoard(args.ser)
    sw = b.read(REG_SW_LO) | (b.read(REG_SW_HI) << 8)
    print(f"SW[15:0] = {sw:016b}  (0x{sw:04X})")


def cmd_status(args):
    b = DemoBoard(args.ser)
    st = b.read(REG_STATUS)
    cnt = b.read(REG_RXCNT_LO) | (b.read(REG_RXCNT_HI) << 8)
    print(f"parity error : {bool(st & 1)}")
    print(f"framing error: {bool(st & 2)}")
    print(f"RX overrun   : {bool(st & 4)}")
    print(f"RX byte count: {cnt}")
    if args.clear:
        b.write(REG_STATUS, 0)
        print("status cleared")


def cmd_chase(args):
    b = DemoBoard(args.ser)
    pattern = [1 << i for i in range(8)] + [0x80 >> i for i in range(1, 7)]
    try:
        for _ in range(args.cycles):
            for p in pattern:
                b.write(REG_LED, p)
                time.sleep(args.delay)
    finally:
        b.write(REG_LED, 0)


def cmd_echo(args):
    """Windowed loopback stress test. The window keeps the number of bytes in
    flight below the FPGA's FIFO capacity, so a PC UART that runs slightly
    faster than the FPGA can never overflow it."""
    ser = args.ser
    ser.reset_input_buffer()
    data = os.urandom(args.n)
    rx = bytearray()
    sent = 0
    t0 = time.perf_counter()
    while len(rx) < args.n:
        room = args.window - (sent - len(rx))
        if sent < args.n and room > 0:
            chunk = data[sent:sent + room]
            ser.write(chunk)
            sent += len(chunk)
        got = ser.read(max(1, ser.in_waiting))
        if not got:
            print(f"TIMEOUT after {len(rx)} of {args.n} bytes -- is sw[15] off (echo mode)?")
            return 1
        rx += got
    dt = time.perf_counter() - t0

    errors = sum(1 for a, b in zip(data, rx) if a != b)
    bits = 10 * args.n
    print(f"bytes          : {args.n}")
    print(f"byte errors    : {errors}")
    print(f"bit error rate : {sum(bin(a ^ b).count('1') for a, b in zip(data, rx)) / (8 * args.n):.2e}")
    print(f"elapsed        : {dt:.3f} s")
    print(f"throughput     : {args.n / dt / 1000:.1f} kB/s each way "
          f"({bits / dt / 1000:.1f} kbaud line use, nominal {args.baud / 1000:.1f})")
    print("PASS" if errors == 0 else "FAIL")
    return 0 if errors == 0 else 1


# ---- main --------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--port", help="serial port or pyserial URL (e.g. loop://)")
    ap.add_argument("-b", "--baud", type=int, default=115200)
    ap.add_argument("-t", "--timeout", type=float, default=1.0)
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("ports", help="list serial ports").set_defaults(fn=cmd_ports, need_port=False)
    sub.add_parser("ping", help="read ID and version").set_defaults(fn=cmd_ping)
    p = sub.add_parser("read", help="read a register")
    p.add_argument("addr", type=auto_int)
    p.set_defaults(fn=cmd_read)
    p = sub.add_parser("write", help="write a register")
    p.add_argument("addr", type=auto_int)
    p.add_argument("value", type=auto_int)
    p.set_defaults(fn=cmd_write)
    p = sub.add_parser("leds", help="set LED[7:0]")
    p.add_argument("value", type=auto_int)
    p.set_defaults(fn=cmd_leds)
    sub.add_parser("switches", help="read SW[15:0]").set_defaults(fn=cmd_switches)
    p = sub.add_parser("status", help="error flags and RX counter")
    p.add_argument("--clear", action="store_true")
    p.set_defaults(fn=cmd_status)
    p = sub.add_parser("chase", help="LED animation")
    p.add_argument("--cycles", type=int, default=5)
    p.add_argument("--delay", type=float, default=0.04)
    p.set_defaults(fn=cmd_chase)
    p = sub.add_parser("echo", help="loopback stress test (echo mode)")
    p.add_argument("-n", type=int, default=10000, help="number of bytes")
    p.add_argument("--window", type=int, default=32, help="max bytes in flight")
    p.set_defaults(fn=cmd_echo)

    args = ap.parse_args()
    if getattr(args, "need_port", True):
        if not args.port:
            ap.error("--port is required (see 'ports')")
        args.ser = serial.serial_for_url(args.port, baudrate=args.baud, timeout=args.timeout)
    try:
        return args.fn(args) or 0
    finally:
        if hasattr(args, "ser"):
            args.ser.close()


if __name__ == "__main__":
    sys.exit(main())
