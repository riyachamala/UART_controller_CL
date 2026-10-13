#!/usr/bin/env python3
"""
Parameter-sweep regression for the UART testbench (Icarus Verilog).

Every configuration is compiled with its own -P overrides and simulated in
parallel. A run passes only if the log contains '*** PASS ***'.

    python3 sim/regress.py            # full sweep
    python3 sim/regress.py -k 8N1     # only configs whose name contains '8N1'
    python3 sim/regress.py -j 4       # limit parallel jobs
"""
import argparse
import itertools
import os
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BUILD = os.path.join(ROOT, "build", "regress")
SOURCES = [
    "rtl/uart_baud_gen.v", "rtl/uart_tx.v", "rtl/uart_rx.v",
    "rtl/sync_fifo.v", "rtl/uart.v", "tb/uart_bfm.v", "tb/tb_uart.v",
]
PAR = {0: "N", 1: "E", 2: "O"}


def configs():
    cfgs = []
    # 1) full frame-format matrix at a fast sim rate: 5..9 data bits x N/E/O x 1/2 stop
    for i, (db, par, sb) in enumerate(itertools.product(range(5, 10), range(3), (1, 2))):
        cfgs.append(dict(CLK_FREQ=50_000_000, BAUD_RATE=1_000_000,
                         DATA_BITS=db, PARITY=par, STOP_BITS=sb, SEED=100 + i))
    # 2) real-world clock / baud combinations (fractional divisors)
    cfgs += [
        dict(CLK_FREQ=100_000_000, BAUD_RATE=115_200, NUM_WORDS=60, SEED=7),    # Basys 3
        dict(CLK_FREQ=12_000_000,  BAUD_RATE=115_200, NUM_WORDS=60, SEED=8),    # iCE40 boards
        dict(CLK_FREQ=12_000_000,  BAUD_RATE=115_200, PARITY=1, STOP_BITS=2,
             NUM_WORDS=60, SEED=9),
        dict(CLK_FREQ=27_000_000,  BAUD_RATE=921_600, PARITY=2, SEED=10),       # Tang Nano
        dict(CLK_FREQ=50_000_000,  BAUD_RATE=3_000_000, SEED=11),               # 1.04 clk/tick
        dict(CLK_FREQ=125_000_000, BAUD_RATE=2_000_000, DATA_BITS=9, SEED=12),
    ]
    # 3) structural parameters
    cfgs += [
        dict(CLK_FREQ=50_000_000, BAUD_RATE=1_000_000, OVERSAMPLE=8, SEED=20),
        dict(CLK_FREQ=50_000_000, BAUD_RATE=1_000_000, OVERSAMPLE=32, SEED=21),
        dict(CLK_FREQ=50_000_000, BAUD_RATE=1_000_000, FIFO_DEPTH=2, SEED=22),
        dict(CLK_FREQ=50_000_000, BAUD_RATE=1_000_000, FIFO_DEPTH=64, SEED=23),
    ]
    for c in cfgs:
        c.setdefault("DATA_BITS", 8)
        c.setdefault("PARITY", 0)
        c.setdefault("STOP_BITS", 1)
        c.setdefault("OVERSAMPLE", 16)
        c.setdefault("FIFO_DEPTH", 16)
    return cfgs


def name_of(c):
    return (f"{c['DATA_BITS']}{PAR[c['PARITY']]}{c['STOP_BITS']}"
            f"_{c['CLK_FREQ'] // 1000}k_{c['BAUD_RATE']}bd"
            f"_os{c['OVERSAMPLE']}_f{c['FIFO_DEPTH']}")


def run(c):
    name = name_of(c)
    vvp = os.path.join(BUILD, name + ".vvp")
    log = os.path.join(BUILD, name + ".log")
    defs = [f"-Ptb_uart.{k}={v}" for k, v in sorted(c.items())]
    t0 = time.time()
    comp = subprocess.run(["iverilog", "-g2012", "-o", vvp, *defs, *SOURCES],
                          cwd=ROOT, capture_output=True, text=True)
    if comp.returncode != 0:
        open(log, "w").write(comp.stdout + comp.stderr)
        return name, False, "compile error", time.time() - t0
    sim = subprocess.run(["vvp", "-n", vvp], cwd=ROOT, capture_output=True, text=True)
    out = sim.stdout + sim.stderr
    open(log, "w").write(out)
    ok = "*** PASS ***" in out
    summary = next((l for l in out.splitlines() if l.startswith("tb_uart:") and "checks" in l), "")
    return name, ok, summary.replace("tb_uart: ", ""), time.time() - t0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-k", default="", help="substring filter on config name")
    ap.add_argument("-j", type=int, default=os.cpu_count() or 4)
    args = ap.parse_args()

    os.makedirs(BUILD, exist_ok=True)
    cfgs = [c for c in configs() if args.k in name_of(c)]
    print(f"Running {len(cfgs)} configurations with {args.j} jobs ...\n")

    t0 = time.time()
    with ThreadPoolExecutor(args.j) as ex:
        results = list(ex.map(run, cfgs))

    fails = 0
    for name, ok, summary, dt in results:
        fails += not ok
        print(f"  {'PASS' if ok else 'FAIL'}  {name:<34} {dt:5.1f}s  {summary}")
    print(f"\n{len(results) - fails}/{len(results)} passed in {time.time() - t0:.1f}s"
          f"  (logs in build/regress/)")
    sys.exit(1 if fails else 0)


if __name__ == "__main__":
    main()
