# UART Controller in Verilog — from scratch

A fully parameterised, synthesizable **UART controller** written in plain Verilog-2005, with:

- a **fractional (NCO) baud generator** that hits any baud rate from any clock with < 0.001 % error,
- a **16× oversampling receiver** with majority voting, glitch rejection, and parity / framing / break / overrun detection,
- **TX and RX FIFOs** behind AXI-Stream-style valid/ready interfaces,
- a **self-checking testbench** with an independent bus-functional model, a **40-configuration regression**, a **measured baud-tolerance sweep**, and mutation checks that prove the testbench catches real bugs,
- an **FPGA demo** (Digilent Basys 3) that talks to a **PC over USB-UART**, plus a **Python host tool** for echo stress testing and register access.

```
make test        # Verilator lint + unit TB + demo system TB + 40-config regression
```

---

## Table of contents

1. [Features](#features)
2. [Repository layout](#repository-layout)
3. [Quick start](#quick-start)
4. [Architecture](#architecture)
5. [Parameters](#parameters)
6. [Interface / ports](#interface--ports)
7. [Design details](#design-details)
   - [Frame format](#frame-format)
   - [Fractional baud generator](#fractional-baud-generator-nco)
   - [Transmitter](#transmitter)
   - [Receiver](#receiver)
   - [FIFOs and error reporting](#fifos-and-error-reporting)
8. [Verification](#verification)
9. [Results](#results)
10. [FPGA demo: talking to a PC](#fpga-demo-talking-to-a-pc)
11. [Porting to other boards](#porting-to-other-boards)
12. [Design decisions and limitations](#design-decisions-and-limitations)
13. [Possible extensions](#possible-extensions)

---

## Features

| Area | Details |
|---|---|
| Frame format | 5–9 data bits, parity none / even / odd, 1 or 2 stop bits, LSB first |
| Baud generation | 24-bit phase-accumulator NCO. Any `CLK_FREQ` > `BAUD_RATE × OVERSAMPLE` works, including fractional ratios like 12 MHz → 115200 |
| Receiver | 2-FF synchroniser, configurable oversampling (≥ 8, default 16), 3-sample majority vote per bit, start-bit re-validation (glitch filter), per-frame re-sync, re-arm mid-stop-bit for fast senders |
| Error detection | Parity error, framing error, break, RX FIFO overrun (sticky). Error flags travel **with each received word** |
| Buffering | Parameterised power-of-two TX/RX FIFOs (first-word-fall-through, map to LUT-RAM / EBR) |
| Interfaces | AXI-Stream-style `valid/ready` for TX and RX, so it drops into any streaming design |
| Throughput | Back-to-back, gap-free frames (verified to the tick) |
| Code quality | Plain Verilog-2005, `default_nettype none`, synchronous reset, Verilator `-Wall` clean |
| Verification | Self-checking TB, independent BFM, random back-pressure, 9 directed and random tests, 40-config sweep, tolerance sweep, mutation checks |
| Demo | Basys 3 echo + register protocol over USB-UART, with a Python host tool |

---

## Repository layout

```
.
├── rtl/
│   ├── uart.v              top level of the IP (baud gen + TX/RX + FIFOs)
│   ├── uart_baud_gen.v     fractional NCO tick generator
│   ├── uart_tx.v           transmitter FSM
│   ├── uart_rx.v           oversampling receiver FSM
│   ├── sync_fifo.v         generic synchronous FIFO
│   └── uart_demo_top.v     FPGA demo: echo + register command protocol
├── tb/
│   ├── uart_bfm.v          behavioural UART bus-functional model (the "PC")
│   ├── tb_uart.v           self-checking testbench for the IP
│   └── tb_uart_demo.v      system-level testbench for the FPGA demo
├── sim/
│   ├── regress.py          parallel 40-configuration parameter sweep
│   └── tolerance.py        empirical baud-error tolerance measurement
├── tools/
│   └── uart_host.py        PC-side host tool (pyserial)
├── fpga/basys3/
│   ├── basys3.xdc          pin + timing constraints
│   └── build.tcl           non-project Vivado build (and optional programming)
├── lint/verilator.vlt      targeted lint waivers
└── Makefile
```

---

## Quick start

### Tools

| Tool | Used for | Install (macOS / Ubuntu) |
|---|---|---|
| [Icarus Verilog](https://steveicarus.github.io/iverilog/) ≥ 11 | simulation | `brew install icarus-verilog` / `apt install iverilog` |
| Python 3 | regression scripts, host tool | — |
| [Verilator](https://verilator.org) ≥ 5 *(optional)* | lint | `brew install verilator` / `apt install verilator` |
| [Yosys](https://yosyshq.net/yosys/) *(optional)* | open-source synthesis stats | `brew install yosys` / `apt install yosys` |
| GTKWave *(optional)* | viewing waveforms | `brew install --cask gtkwave` / `apt install gtkwave` |
| Vivado *(for hardware)* | Basys 3 bitstream | AMD/Xilinx WebPACK (free) |
| pyserial *(for hardware)* | host tool | `pip install pyserial` |

### Make targets

```bash
make sim          # main testbench, default 8N1 @ 1 Mbaud / 50 MHz
make sim PARAMS="-Ptb_uart.DATA_BITS=7 -Ptb_uart.PARITY=2 -Ptb_uart.STOP_BITS=2"
make wave         # same, dumps build/tb_uart.vcd  (gtkwave build/tb_uart.vcd)
make demo-sim     # FPGA demo system testbench
make regress      # 40 parameter configurations in parallel (~6 s)
make tolerance    # measure receiver baud-error tolerance
make lint         # Verilator -Wall
make synth        # Yosys resource usage (Xilinx 7-series)
make synth-ice40  # Yosys resource usage (Lattice iCE40)
make test         # lint + sim + demo-sim + regress
```

Sample output of `make sim`:

```
uart: CLK=50000000 Hz BAUD=1000000 8N1 OS=16  NCO inc=5368709 -> actual 1000000.0 baud (-0.0000 %)
tb_uart: 8N1 @ 1000000 baud, clk 50000000 Hz, OS=16, FIFO=16, seed=1
[     2190 ns] --- test 1: loopback, random data and gaps
[  2004250 ns] --- test 2: loopback, corner-case patterns
[  2206310 ns] --- test 3: back-to-back throughput (gap-free)
        16 frames in 160000.0 ns, ideal 160000.0 ns (10 bits each)
[  2370350 ns] --- test 4: baud-rate tolerance (remote TX off-frequency)
        remote TX rate error -3.0 %
        ...
[  3890810 ns] --- test 8: RX FIFO overrun
tb_uart: 8 tests, 627 checks (236 TX frames, 391 RX words), 0 errors
*** PASS ***
```

---

## Architecture

```
                        +-------------------------------------------------------+
                        |                        uart                           |
                        |                                                       |
  s_tx_data  ---------->|  +-----------+     +-----------+                      |
  s_tx_valid ---------->|  |  TX FIFO  |---->|  uart_tx  |--------------------->|---> txd
  s_tx_ready <----------|  | (DEPTH)   |     |   FSM     |                      |
                        |  +-----------+     +-----^-----+                      |
                        |                          | tick (BAUD x OVERSAMPLE)   |
                        |                    +-----+---------+                  |
                        |                    | uart_baud_gen |  24-bit NCO      |
                        |                    +-----+---------+                  |
                        |                          | tick                       |
  m_rx_data  <----------|  +-----------+     +-----v-----+     +----------+     |
  m_rx_perr/ferr/break <|  |  RX FIFO  |<----|  uart_rx  |<----| 2-FF     |<----|<--- rxd
  m_rx_valid <----------|  | data+flags|     |   FSM     |     | sync     |     |
  m_rx_ready ---------->|  +-----------+     +-----------+     +----------+     |
                        |        |                                              |
  rx_overrun <----------|--------+  (sticky, cleared by clr_overrun)            |
                        +-------------------------------------------------------+
```

The TX and RX sides share a single oversampled tick. The transmitter divides it by `OVERSAMPLE` internally. The receiver uses it directly and re-aligns its bit counter to every start edge, so its sampling phase does not depend on the transmitter's.

---

## Parameters

All parameters are on `uart` (and mirrored in the testbench so they can be overridden from the command line).

| Parameter | Default | Range | Meaning |
|---|---|---|---|
| `CLK_FREQ` | 100 000 000 | > `BAUD_RATE × OVERSAMPLE` | System clock in Hz |
| `BAUD_RATE` | 115 200 | any | Bits per second |
| `DATA_BITS` | 8 | 5 – 9 | Data bits per frame |
| `PARITY` | 0 | 0 / 1 / 2 | None / Even / Odd |
| `STOP_BITS` | 1 | 1 – 2 | Stop bits transmitted |
| `OVERSAMPLE` | 16 | ≥ 8 | Receiver samples per bit |
| `FIFO_DEPTH` | 16 | power of 2, ≥ 2 | Depth of TX and RX FIFOs |
| `ACC_WIDTH` | 24 | — | NCO accumulator width (baud resolution) |

Unsupported combinations are caught at elaboration time in simulation. For example, a clock that is too slow for the requested baud × oversampling ends the run with an error message.

Example instantiation (9600 baud, 7E2, 64-deep FIFOs on a 12 MHz board):

```verilog
uart #(
    .CLK_FREQ  (12_000_000),
    .BAUD_RATE (9600),
    .DATA_BITS (7),
    .PARITY    (1),          // even
    .STOP_BITS (2),
    .FIFO_DEPTH(64)
) u_uart ( ... );
```

---

## Interface / ports

| Port | Dir | Width | Description |
|---|---|---|---|
| `clk` | in | 1 | System clock |
| `rst` | in | 1 | Synchronous, active-high reset |
| `s_tx_data` | in | `DATA_BITS` | Word to transmit |
| `s_tx_valid` | in | 1 | `s_tx_data` is valid |
| `s_tx_ready` | out | 1 | TX FIFO has room. A transfer happens when `valid && ready` |
| `m_rx_data` | out | `DATA_BITS` | Received word (head of RX FIFO) |
| `m_rx_perr` | out | 1 | That word had a parity error |
| `m_rx_ferr` | out | 1 | That word had a framing error (stop bit = 0) |
| `m_rx_break` | out | 1 | That word was a break (line held low for the whole frame) |
| `m_rx_valid` | out | 1 | RX FIFO not empty |
| `m_rx_ready` | in | 1 | Consumer accepts the word. A pop happens when `valid && ready` |
| `txd` | out | 1 | Serial output (idle high) |
| `rxd` | in | 1 | Serial input (asynchronous; synchronised internally) |
| `tx_busy` | out | 1 | TX FIFO non-empty or a frame is being shifted out |
| `tx_fifo_count` | out | log2(D)+1 | Words waiting in the TX FIFO |
| `rx_fifo_count` | out | log2(D)+1 | Words waiting in the RX FIFO |
| `rx_overrun` | out | 1 | **Sticky**: a word arrived while the RX FIFO was full and was dropped |
| `clr_overrun` | in | 1 | Clears `rx_overrun` |

Both streams follow the AXI-Stream handshake rules: `ready` never depends on `valid`, and a word moves on every clock edge where both are high.

---

## Design details

### Frame format

```
 idle  start   D0    D1    D2   ...   Dn-1  [parity]  stop  [stop2]  idle
 ‾‾‾‾‾|_____|=====|=====|=====|     |=====|=========|‾‾‾‾‾|‾‾‾‾‾‾‾|‾‾‾‾‾‾
       1 bit  <------ DATA_BITS, LSB first ------>   opt.  1 bit   opt.
```

- Even parity: the parity bit makes the total number of 1s (data + parity) even. Odd parity makes it odd.
- Frame length = 1 + `DATA_BITS` + (parity ? 1 : 0) + `STOP_BITS` bits (10 for 8N1, 13 for 9O2).

### Fractional baud generator (NCO)

The obvious way to make the bit clock is an integer counter: `DIV = CLK / (BAUD × 16)`. That only works when the division comes out close to a whole number. Otherwise the rounding error makes the UART unusable:

| Clock | Baud | Integer divider ÷(16·N) | Error | This design (24-bit NCO) | Error |
|---|---|---|---|---|---|
| 100 MHz | 115 200 | N = 54 | +0.47 % | inc = 309 238 | **+0.0001 %** |
| 100 MHz | 921 600 | N = 7 | −3.1 % | — | **< 0.001 %** |
| 12 MHz | 115 200 | N = 7 | **−7.0 %** ✗ | inc = 2 576 980 | **0.0000 %** |
| 27 MHz | 921 600 | N = 2 | **−8.4 %** ✗ | inc = 9 162 597 | **0.0000 %** |
| 50 MHz | 3 000 000 | N = 1 | +4.2 % | inc = 16 106 127 | **0.0000 %** |

`uart_baud_gen` instead adds a constant `INC` to a 24-bit accumulator on every clock, and the carry-out is the tick:

```
INC    = round(BAUD × OVERSAMPLE × 2^24 / CLK_FREQ)      (computed at elaboration, 64-bit math)
f_tick = CLK_FREQ × INC / 2^24
```

The fractional remainder is never thrown away, so the *average* tick rate is exact to within 2⁻²⁴ of the clock. Individual ticks jitter by at most one system clock, which is far smaller than the 1/16-bit sampling resolution. The simulation prints the achieved baud and its error at start-up (see the `uart:` line above).

### Transmitter

`uart_tx` is a 5-state FSM (`IDLE → START → DATA → [PARITY] → STOP`) clocked by the shared tick:

- A word is accepted **only on a tick**, so every bit, including the start bit, lasts exactly `OVERSAMPLE` ticks.
- `s_ready` is also asserted on the final tick of the last stop bit. If the FIFO holds another word, it is loaded on that same tick and the next start bit begins immediately, so there are **no idle gaps**. The testbench measures this: 16 frames take exactly 16 × 10 bit times.
- The parity bit is computed once, when the word is loaded (`^data`, inverted for odd parity).

### Receiver

`uart_rx` is the more involved half of the design:

1. **Synchroniser.** `rxd` passes through two flip-flops (marked `ASYNC_REG`) before any logic sees it.
2. **Start detection.** In `IDLE`, the first tick that sees the line low becomes sample count 0. From then on, sample *k* of each bit falls *k* ticks after the start edge (plus up to one tick of detection latency).
3. **Majority vote.** Each bit is sampled on ticks `MID-2`, `MID-1` and `MID` (6, 7, 8 for 16× oversampling), close to the bit centre. The bit value is the 2-of-3 majority. A single noise spike cannot flip a bit.
4. **False-start rejection.** The start bit itself is voted on. If the majority is high, the low pulse was a glitch and the receiver goes back to `IDLE`. Pulses up to ~0.4 bit wide are rejected; the testbench checks 0.10 to 0.38 bit.
5. **Early re-arm.** Only the first stop bit is checked, and the receiver goes back to `IDLE` *in the middle of it*. If the far-end transmitter runs fast, its next start edge can arrive before "our" stop bit would have ended. Waiting for the end of the stop bit would make each frame start later than the last until the receiver lost sync.
   > This was found by the regression: an earlier version sampled at ticks 7–9 and re-armed one tick later. With `OVERSAMPLE = 8` and a +3 % sender, it lost sync after a few back-to-back frames. Moving the sample window one tick earlier fixed it and made the tolerance symmetric.
6. **Errors.**
   - framing error: the stop-bit majority is 0.
   - break: the stop bit, every data bit and the parity bit are all 0. Parity is not reported on a break.
   - After a framing error or break the receiver waits in `S_WAIT` for the line to go high, so a long break produces **exactly one** break word instead of a stream of garbage.

#### Baud-error tolerance

For the last sampled bit (index *n* = stop bit), with samples at (MID-2 … MID)/OS:

- sender **slow** by *e*: need samples `MID-1` and `MID` after the stop bit starts, i.e. *n + 7/16 ≥ n(1+e)*
- sender **fast** by *e*: need re-arm (tick `MID`) before the next start edge, i.e. *n + 9/16 ≤ (n+1)(1−e)*

For 8N1 (*n* = 9) this gives roughly **±4.4 %** of *total* mismatch between the two ends. Measured by `make tolerance` (remote transmitter fast *and* slow, frames back-to-back, 0.25 % steps):

| Format | Max passing remote baud error |
|---|---|
| 7N1, 16× | ±5.00 % |
| 8N1, 16× | **±4.25 %** |
| 8E1, 16× | ±4.00 % |
| 9O2, 16× | ±3.75 % |
| 8N1, 8× | ±4.00 % |
| 8N1, 32× | ±4.75 % |

Commodity UARTs usually budget about ±2 % per side, so these margins are comfortable.

### FIFOs and error reporting

- `sync_fifo` is a classic extra-MSB-pointer FIFO with first-word-fall-through output. The head word is always on `rd_data`, which is exactly what a `valid/ready` source needs.
- The RX FIFO is `DATA_BITS + 3` wide, so **each word carries its own `perr/ferr/break` flags**. Software can tell *which* byte was bad, unlike a single shared status bit.
- If a word arrives while the RX FIFO is full, it is dropped and the sticky `rx_overrun` flag is set. The words already in the FIFO stay intact (verified).

---

## Verification

### Testbench structure (`tb/tb_uart.v`)

```
               +----------------------------- tb_uart --------------------------------+
               |                                                                      |
 random words  |  send_word() --> s_tx_* --+                                          |
 + idle gaps   |                           |      +------+  txd    +-----------+      |
               |   tx_exp queue <----------+      |      |-------->| uart_bfm  |      |
               |        |                         | DUT  |         | (monitor) |----+ |
               |        +---- compare <-----------|------|---------+-----------+    | |
               |                                  |      |                          | |
               |   rx_exp queue                   |      |<-- rxd --+-- loopback    | |
               |        |                         |      |          |               | |
               |        +---- compare <-- m_rx_* (random ready, ~75 %)              | |
               |                                  +------+          +-- uart_bfm    | |
               |                                                        (driver,    | |
               |                                                         ±baud err, | |
               |                                                         bad parity,| |
               |                                                         bad stop,  | |
               |                                                         glitches,  | |
               |                                                         break)     | |
               +----------------------------------------------------------------------+
```

- **Independent reference.** `uart_bfm` is a *timing-based* behavioural UART written with `#delays`, not clocks. It shares no code with the RTL, so a misunderstanding in the RTL cannot hide in the checker as well.
- **Two scoreboards run for the whole simulation.** Every frame on `txd` is decoded by the monitor BFM and checked against what was written to `s_tx_*`, including parity and stop bits. Every word popped from `m_rx_*` is checked, flags included, against an expectation queue.
- **Random back-pressure** on `m_rx_ready` (25 % of cycles stalled) exercises the FIFO handshake.
- A global watchdog fails the test if anything hangs.

### Tests

| # | Test | What it proves |
|---|---|---|
| 1 | Loopback, 200 random words, random idle gaps (0 – 3 bit times) | Basic TX → RX correctness, FIFO flow, idle-line handling |
| 2 | Corner patterns: all-0, all-1, 0x55, 0xAA, walking 1 and walking 0 | Bit ordering and parity on every bit position |
| 3 | Back-to-back burst of `FIFO_DEPTH` words | Gap-free TX; burst duration matches *N × frame × bit-time* to within one tick |
| 4 | External TX at −3, −1.5, 0, +1.5, +3 % baud, back-to-back frames | Receiver re-synchronisation and clock tolerance |
| 5 | Wrong parity on every other frame (when parity is enabled) | `perr` on exactly the bad words; data still delivered |
| 6 | Stop bit forced low, alternating with good frames | `ferr` flagged, receiver recovers on the next frame |
| 7 | Line held low for 2.5 frames, then a normal frame | Exactly **one** break word, then normal reception |
| 8 | Low glitches 0.10 – 0.38 bit wide | No false frames; the next real frame is received |
| 9 | 3 words more than the RX FIFO can hold, with `m_rx_ready` held low | Sticky `rx_overrun`, first `FIFO_DEPTH` words intact, flag clears |

### Regression (`make regress`)

`sim/regress.py` compiles and runs the testbench for **40 configurations** in parallel:

- **All 30 frame formats**: `DATA_BITS` 5–9 × parity N/E/O × 1/2 stop bits
- **Real-world clocks with fractional ratios**: 100 MHz / 115200 (Basys 3), 12 MHz / 115200 (iCE40 boards), 27 MHz / 921600 (Tang Nano), 125 MHz / 2 Mbaud, and 50 MHz / 3 Mbaud (≈ 1.04 clocks per tick, the NCO at its limit)
- **Structural variants**: `OVERSAMPLE` = 8 and 32, `FIFO_DEPTH` = 2 and 64

```
Running 40 configurations with 14 jobs ...
  PASS  5N1_50000k_1000000bd_os16_f16   0.5s  8 tests, 615 checks (230 TX frames, 385 RX words), 0 errors
  PASS  5N2_50000k_1000000bd_os16_f16   0.6s  8 tests, 615 checks ...
  ...
  PASS  8N1_12000k_115200bd_os16_f16    0.8s  8 tests, 347 checks ...
  PASS  8N1_50000k_3000000bd_os16_f16   0.3s  8 tests, 627 checks ...
  PASS  8N1_50000k_1000000bd_os8_f16    0.6s  8 tests, 627 checks ...
40/40 passed in 6.2s
```

### Does the testbench actually catch bugs? (mutation check)

Deliberately broken copies of the RTL were run through the 8E1 testbench:

| Injected bug | Result |
|---|---|
| TX parity bit inverted | **FAIL**, 472 errors |
| RX glitch filter removed | **FAIL**, 8 errors (one per glitch) |
| TX inserts one idle tick between frames | **FAIL**, caught by the throughput test |
| *unmodified RTL* | PASS, 0 errors |

### Demo system test (`make demo-sim`)

`tb/tb_uart_demo.v` drives `uart_demo_top` the same way the Python host tool does:

- echo mode: 128 random bytes back-to-back, every echo checked, LEDs show the last byte
- command mode: ID and version reads, LED write that is visible on the `led` pins, switch read-back, scratch register, unknown command → `?`, CR/LF ignored, RX byte counter, and an injected framing error that shows up in `STATUS` and on the LED and is then cleared

```
[     3500 ns] --- echo mode: 128 back-to-back random bytes
[  1294775 ns] --- command mode: register protocol
tb_uart_demo: 148 checks, 0 errors
*** PASS ***
```

### Lint

`make lint` runs Verilator `-Wall` on the demo top and on an extreme core configuration (9O2) with **zero warnings**. The only waiver (`lint/verilator.vlt`) covers 32-bit integer parameters initialising narrower sized `localparam` constants, and nothing else.

---

## Results

### Resource usage (Yosys, default parameters: 8N1, FIFO depth 16)

| Target | LUTs | Flip-flops | Memory | Carry |
|---|---|---|---|---|
| `uart` core, Xilinx 7-series | 127 LUT | 110 FF | 4 × RAM32M (LUT-RAM FIFOs) | 23 CARRY4 |
| `uart` core, Lattice iCE40 | 201 LUT4 | 121 DFF | 2 × SB_RAM40_4K | 50 SB_CARRY |
| `uart_demo_top` (whole demo), Xilinx 7-series | 280 LUT | 296 FF | 4 × RAM32M | 47 CARRY4 |

On a Basys 3 (XC7A35T, 20 800 LUTs) the whole demo uses **under 1.5 % of the LUTs**. Most of the demo's flip-flops are the LED-stretch and heartbeat counters, not the UART itself. These are pre-place-and-route Yosys numbers; Vivado writes its own report to `fpga/basys3/out/utilization.rpt`.

### Verified operating envelope (simulation)

- 5–9 data bits, N/E/O parity, 1–2 stop bits, all combinations
- clocks from 12 MHz to 125 MHz, baud rates from 115 200 to 3 Mbaud
- receiver tolerates ±4.25 % (8N1) remote baud error with back-to-back frames

---

## FPGA demo: talking to a PC

The demo runs on a **Digilent Basys 3** (Artix-7). The board's micro-USB port already carries a USB-UART bridge (FTDI FT2232), so a single USB cable provides power, programming and the serial link. No extra wiring is needed.

> **Status:** the demo RTL is verified in simulation by `make demo-sim`, which uses the same protocol as the host tool. The host tool's echo path has been exercised against pyserial's `loop://` device. The Vivado script and the XDC pin map (from Digilent's master XDC) are provided for hardware bring-up.

### Building and programming

```bash
cd fpga/basys3
vivado -mode batch -source build.tcl              # -> out/uart_demo_top.bit
vivado -mode batch -source build.tcl -tclargs prog  # build and program over USB
```

Or, in the Vivado GUI: create a project for `xc7a35tcpg236-1`, add `rtl/*.v` and `fpga/basys3/basys3.xdc`, set `uart_demo_top` as top, and generate a bitstream.

### Board controls

| Control | Function |
|---|---|
| `sw[15]` | **0 = echo mode**, **1 = command mode** |
| `sw[14:0]` | Readable from the PC (registers `SW_LO` / `SW_HI`) |
| Centre button (`btnC`) | Reset |
| `led[7:0]` | Echo mode: last byte received. Command mode: `LED` register |
| `led[8]` / `led[9]` | RX / TX activity (stretched pulses) |
| `led[10]` / `led[11]` / `led[12]` | Sticky parity / framing / overrun error |
| `led[14]` | Heartbeat (~1.5 Hz blink) |
| `led[15]` | Command mode active |

### Echo mode (`sw[15]` = 0)

Every received byte is sent straight back. You can try it in any terminal (`screen /dev/tty.usbserial-XXXX 115200`, PuTTY, `python -m serial.tools.miniterm`), or run the stress test:

```bash
pip install pyserial
python3 tools/uart_host.py ports                          # find the port
python3 tools/uart_host.py -p /dev/ttyUSB1 echo -n 100000 # Linux
python3 tools/uart_host.py -p COM5 echo -n 100000         # Windows
```

The tool sends random data, checks every echoed byte, and reports byte errors, bit error rate and throughput.

```
bytes          : 100000
byte errors    : 0
bit error rate : 0.00e+00
...
PASS
```

You can try the tool itself without hardware: `python3 tools/uart_host.py -p loop:// echo`.

> **Why the echo test is windowed:** two "115200 baud" UARTs are never exactly the same speed. An FTDI bridge, for example, runs about 0.16 % fast. In a continuous echo, the faster side slowly fills the slower side's FIFO, by about one byte every few hundred. The host tool therefore keeps at most `--window` (default 32) bytes in flight. This is a property of any echo path, not a bug in the UART: the *receiver* re-synchronises on every frame and is unaffected.

### Command mode (`sw[15]` = 1)

A small binary register protocol:

| Host sends | FPGA replies | Meaning |
|---|---|---|
| `'W'` `addr` `data` | `'K'` | write register |
| `'R'` `addr` | `data` | read register |
| CR / LF / space | *(nothing)* | ignored, so terminal line endings are harmless |
| anything else | `'?'` | unknown command |

Bytes with a parity or framing error are discarded, and the parser returns to waiting for a command.

**Register map**

| Addr | Name | Access | Description |
|---|---|---|---|
| 0x00 | `ID` | R | `0x55` (`'U'`) |
| 0x01 | `VERSION` | R | `0x10` = v1.0 |
| 0x02 | `LED` | RW | drives `led[7:0]` in command mode |
| 0x03 | `SW_LO` | R | `sw[7:0]` |
| 0x04 | `SW_HI` | R | `sw[15:8]` |
| 0x05 | `SCRATCH` | RW | free register for testing |
| 0x06 | `RXCNT_LO` | R | bytes received since reset, bits 7:0 |
| 0x07 | `RXCNT_HI` | R | bits 15:8 |
| 0x08 | `STATUS` | R / W | read `{5'b0, overrun, framing, parity}` (sticky). Any write clears it |

**Host tool**

```bash
python3 tools/uart_host.py -p /dev/ttyUSB1 ping            # ID 0x55 ('U') OK, VERSION 1.0
python3 tools/uart_host.py -p /dev/ttyUSB1 leds 0xA5       # light LEDs 7,5,2,0
python3 tools/uart_host.py -p /dev/ttyUSB1 switches        # SW[15:0] = 1000000000000101
python3 tools/uart_host.py -p /dev/ttyUSB1 chase           # LED animation from the PC
python3 tools/uart_host.py -p /dev/ttyUSB1 status --clear  # error flags + RX byte count
python3 tools/uart_host.py -p /dev/ttyUSB1 read 0x05
python3 tools/uart_host.py -p /dev/ttyUSB1 write 0x05 0x3C
```

The same protocol is easy to drive from any language. Here it is in raw Python:

```python
import serial
s = serial.Serial("/dev/ttyUSB1", 115200, timeout=1)
s.write(b"W\x02\xff");  assert s.read(1) == b"K"   # all LEDs on
s.write(b"R\x00");      print(s.read(1))           # b'U'
```

---

## Porting to other boards

The IP has no vendor primitives. To port the demo:

1. Set `CLK_FREQ` (and optionally `BAUD_RATE`) on `uart_demo_top` to match your board. Any clock above `BAUD_RATE × 16` works, thanks to the NCO.
2. Map `clk`, `btn_rst` (active high; invert it if your button is active low), `uart_rxd`, `uart_txd`, `sw[15:0]` and `led[15:0]` in your constraint file. Tie off whatever the board doesn't have. Keep at least `sw[15]` (mode) if you want both modes.
3. For a board without a USB-UART, use any **3.3 V** USB-serial adapter (FTDI, CP2102, CH340): adapter TX → `uart_rxd`, adapter RX ← `uart_txd`, and connect GND. Never connect RS-232 levels (±12 V) directly to an FPGA pin.

Configurations already covered by the regression include iCE40 boards (iCEBreaker / iCEstick at 12 MHz), Tang Nano (27 MHz) and Zynq/Artix boards at 100–125 MHz.

---

## Design decisions and limitations

- **Synchronous active-high reset everywhere.** This maps well onto Xilinx FDRE/FDSE and iCE40 SB_DFFSR cells. The demo adds a 2-FF button synchroniser and a power-on reset from register initial values.
- **The baud rate is fixed at elaboration.** This keeps the core small and makes the 64-bit increment a pure constant. A runtime-programmable divisor only needs `uart_baud_gen.inc` brought out to a register (see extensions).
- **RX has no back-pressure.** A serial line cannot be paused, so overflow is reported (`rx_overrun`) rather than prevented. Size `FIFO_DEPTH` for your consumer's worst-case latency, or add RTS/CTS.
- **Only the first stop bit is checked on receive**, as in the 16550. This maximises tolerance to fast senders. `STOP_BITS = 2` still *transmits* two stop bits.
- **`OVERSAMPLE` must be ≥ 8** so that the three majority samples and the end-of-bit tick are distinct.
- FIFOs are single-clock. For a design where the user logic runs on a different clock from the UART, put an async FIFO in front of the streams.

---

## Possible extensions

- Runtime baud register plus automatic baud detection (measure the width of the first start bit)
- RTS/CTS hardware flow control
- APB / AXI4-Lite / Wishbone register wrapper with interrupt outputs (RX not-empty, TX empty, error), as a 16550-style peripheral
- RS-485 driver-enable output with turnaround timing
- Sister SPI and I²C controllers sharing the same stream interfaces, e.g. a UART-to-I²C bridge for reading a real sensor (BMP280, MPU-6050) from the PC
- Formal properties (SymbiYosys) for FIFO pointer invariants and TX frame structure
- cocotb version of the testbench

---

*Written in Verilog-2005. Simulated with Icarus Verilog 13, linted with Verilator 5, synthesised with Yosys 0.69.*
