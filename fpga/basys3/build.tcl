# -----------------------------------------------------------------------------
# Non-project Vivado build for the Basys 3 demo.
#
#   cd fpga/basys3
#   vivado -mode batch -source build.tcl                 # build bitstream
#   vivado -mode batch -source build.tcl -tclargs prog   # build + program board
#
# Outputs go to fpga/basys3/out/
# -----------------------------------------------------------------------------
set part    xc7a35tcpg236-1
set top     uart_demo_top
set here    [file dirname [file normalize [info script]]]
set rtl     [file normalize $here/../../rtl]
set out     $here/out
file mkdir  $out

read_verilog [list \
    $rtl/uart_baud_gen.v \
    $rtl/uart_tx.v       \
    $rtl/uart_rx.v       \
    $rtl/sync_fifo.v     \
    $rtl/uart.v          \
    $rtl/uart_demo_top.v ]
read_xdc $here/basys3.xdc

synth_design -top $top -part $part -generic CLK_FREQ=100000000 -generic BAUD_RATE=115200
opt_design
place_design
route_design

report_utilization    -file $out/utilization.rpt
report_timing_summary -file $out/timing.rpt
write_bitstream -force $out/$top.bit
puts "Bitstream: $out/$top.bit"

if {[lindex $argv 0] eq "prog"} {
    open_hw_manager
    connect_hw_server
    open_hw_target
    set dev [lindex [get_hw_devices xc7a35t_0] 0]
    current_hw_device $dev
    set_property PROGRAM.FILE $out/$top.bit $dev
    program_hw_devices $dev
    close_hw_manager
}
