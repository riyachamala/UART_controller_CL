# -----------------------------------------------------------------------------
# UART controller -- simulation, regression, lint and synthesis
# -----------------------------------------------------------------------------
IVERILOG ?= iverilog
VVP      ?= vvp
YOSYS    ?= yosys
VERILATOR?= verilator

CORE  := rtl/uart_baud_gen.v rtl/uart_tx.v rtl/uart_rx.v rtl/sync_fifo.v rtl/uart.v
DEMO  := rtl/uart_demo_top.v
BFM   := tb/uart_bfm.v
BUILD := build

# extra -P overrides for `make sim`, e.g.  make sim PARAMS="-Ptb_uart.PARITY=2"
PARAMS ?=

.PHONY: all sim wave demo-sim demo-wave regress tolerance lint synth synth-ice40 test clean help

all: test

help:
	@echo "make sim         run the main UART testbench (default 8N1)"
	@echo "make wave        same, dumping build/tb_uart.vcd"
	@echo "make demo-sim    run the FPGA demo system testbench"
	@echo "make regress     40-configuration parameter sweep"
	@echo "make tolerance   measure RX baud-error tolerance"
	@echo "make lint        Verilator -Wall lint"
	@echo "make synth       Yosys synthesis stats (Xilinx 7-series)"
	@echo "make synth-ice40 Yosys synthesis stats (Lattice iCE40)"
	@echo "make test        lint + sim + demo-sim + regress"

$(BUILD):
	@mkdir -p $(BUILD)

$(BUILD)/tb_uart.vvp: $(CORE) $(BFM) tb/tb_uart.v | $(BUILD)
	$(IVERILOG) -g2012 -Wall $(PARAMS) -o $@ $(CORE) $(BFM) tb/tb_uart.v

$(BUILD)/tb_uart_demo.vvp: $(CORE) $(DEMO) $(BFM) tb/tb_uart_demo.v | $(BUILD)
	$(IVERILOG) -g2012 -Wall -o $@ $(CORE) $(DEMO) $(BFM) tb/tb_uart_demo.v

sim: $(BUILD)/tb_uart.vvp
	$(VVP) -n $< | tee $(BUILD)/tb_uart.log
	@grep -q '\*\*\* PASS \*\*\*' $(BUILD)/tb_uart.log

wave: $(BUILD)/tb_uart.vvp
	$(VVP) -n $< +vcd
	@echo "open with: gtkwave $(BUILD)/tb_uart.vcd"

demo-sim: $(BUILD)/tb_uart_demo.vvp
	$(VVP) -n $< | tee $(BUILD)/tb_uart_demo.log
	@grep -q '\*\*\* PASS \*\*\*' $(BUILD)/tb_uart_demo.log

demo-wave: $(BUILD)/tb_uart_demo.vvp
	$(VVP) -n $< +vcd

regress:
	python3 sim/regress.py

tolerance:
	python3 sim/tolerance.py

lint:
	$(VERILATOR) --lint-only -Wall lint/verilator.vlt --top-module uart_demo_top $(CORE) $(DEMO)
	$(VERILATOR) --lint-only -Wall lint/verilator.vlt --top-module uart -GDATA_BITS=9 -GPARITY=2 -GSTOP_BITS=2 $(CORE)
	@echo "lint clean"

synth: | $(BUILD)
	$(YOSYS) -q -p "read_verilog $(CORE); synth_xilinx -family xc7 -top uart; tee -o $(BUILD)/synth_uart_xc7.txt stat"
	$(YOSYS) -q -p "read_verilog $(CORE) $(DEMO); synth_xilinx -family xc7 -top uart_demo_top; tee -o $(BUILD)/synth_demo_xc7.txt stat"

synth-ice40: | $(BUILD)
	$(YOSYS) -q -p "read_verilog $(CORE); synth_ice40 -top uart; tee -o $(BUILD)/synth_uart_ice40.txt stat"

test: lint sim demo-sim regress

clean:
	rm -rf $(BUILD) fpga/basys3/out fpga/basys3/.Xil fpga/basys3/*.jou fpga/basys3/*.log
