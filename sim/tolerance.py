#!/usr/bin/env python3
"""
Measure the receiver's baud-rate tolerance empirically.

For each frame format, the testbench is re-run with increasing MAX_RATE_ERR
(the remote transmitter is that much fast AND slow, with back-to-back frames)
until it fails. The largest passing value is reported.

    python3 sim/tolerance.py
"""
import os
import subprocess
from concurrent.futures import ThreadPoolExecutor

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BUILD = os.path.join(ROOT, "build", "tolerance")
SOURCES = ["rtl/uart_baud_gen.v", "rtl/uart_tx.v", "rtl/uart_rx.v", "rtl/sync_fifo.v",
           "rtl/uart.v", "tb/uart_bfm.v", "tb/tb_uart.v"]
FORMATS = [  # (label, DATA_BITS, PARITY, STOP_BITS, OVERSAMPLE)
    ("8N1 os16", 8, 0, 1, 16),
    ("8E1 os16", 8, 1, 1, 16),
    ("9O2 os16", 9, 2, 2, 16),
    ("7N1 os16", 7, 0, 1, 16),
    ("8N1 os8",  8, 0, 1, 8),
    ("8N1 os32", 8, 0, 1, 32),
]
STEPS = [x / 100 for x in range(300, 701, 25)]   # 3.00 % .. 7.00 %


def passes(fmt, err):
    label, db, par, sb, os_ = fmt
    tag = f"{label.replace(' ', '_')}_{err:.4f}"
    vvp = os.path.join(BUILD, tag + ".vvp")
    defs = [f"-Ptb_uart.DATA_BITS={db}", f"-Ptb_uart.PARITY={par}",
            f"-Ptb_uart.STOP_BITS={sb}", f"-Ptb_uart.OVERSAMPLE={os_}",
            f"-Ptb_uart.MAX_RATE_ERR={err / 100}", "-Ptb_uart.NUM_WORDS=20"]
    subprocess.run(["iverilog", "-g2012", "-o", vvp, *defs, *SOURCES],
                   cwd=ROOT, check=True, capture_output=True)
    out = subprocess.run(["vvp", "-n", vvp], cwd=ROOT, capture_output=True, text=True).stdout
    return "*** PASS ***" in out


def sweep(fmt):
    best = None
    for err in STEPS:
        if not passes(fmt, err):
            break
        best = err
    return fmt[0], best


def main():
    os.makedirs(BUILD, exist_ok=True)
    with ThreadPoolExecutor(len(FORMATS)) as ex:
        for label, best in ex.map(sweep, FORMATS):
            shown = f"+/-{best:.2f} %" if best is not None else "< 3 %"
            print(f"  {label:<10} max remote baud error passing: {shown}")


if __name__ == "__main__":
    main()
