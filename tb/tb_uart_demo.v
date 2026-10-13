// -----------------------------------------------------------------------------
// tb_uart_demo.v  --  system-level test of the FPGA demo (uart_demo_top)
//
// A behavioural "PC" (uart_bfm) talks to the demo exactly like the Python host
// script does: first an echo stress test, then the register protocol in
// command mode (ID, LEDs, switches, scratch, counters, error status).
// -----------------------------------------------------------------------------
`timescale 1ns/1ps
`default_nettype none

module tb_uart_demo;

    parameter integer CLK_FREQ  = 20_000_000;
    parameter integer BAUD_RATE = 1_000_000;

    localparam real CLK_NS = 1.0e9 / CLK_FREQ;
    localparam real BIT_NS = 1.0e9 / BAUD_RATE;

    reg clk = 1'b0;
    always #(CLK_NS / 2.0) clk = ~clk;

    reg         btn_rst = 1'b0;
    reg  [15:0] sw      = 16'h0000;
    wire [15:0] led;
    wire        pc_to_fpga, fpga_to_pc;

    uart_demo_top #(
        .CLK_FREQ (CLK_FREQ),
        .BAUD_RATE(BAUD_RATE),
        .ACT_BITS (6),
        .HB_BITS  (8)
    ) dut (
        .clk     (clk),
        .btn_rst (btn_rst),
        .uart_rxd(pc_to_fpga),
        .uart_txd(fpga_to_pc),
        .sw      (sw),
        .led     (led)
    );

    uart_bfm #(.BAUD(BAUD_RATE * 1.0)) pc (.txd(pc_to_fpga), .rxd(fpga_to_pc));

    integer errors = 0, checks = 0, sent = 0;

    task check8(input [8*40-1:0] what, input [7:0] got, input [7:0] exp);
        begin
            checks = checks + 1;
            if (got !== exp) begin
                errors = errors + 1;
                $display("[%t] ERROR %0s: got %h expected %h", $time, what, got, exp);
            end
        end
    endtask

    task tx(input [7:0] b);
        begin
            pc.send({1'b0, b}, 0.0, 1'b0, 1'b0);
            sent = sent + 1;
        end
    endtask

    // send `n` bytes of a command and capture the one-byte reply
    task transact(input integer n, input [7:0] b0, input [7:0] b1, input [7:0] b2,
                  output [7:0] reply);
        reg [8:0] d;
        reg       pe, fe;
        begin
            fork
                begin
                    pc.recv(d, pe, fe);
                    if (fe) begin
                        errors = errors + 1;
                        $display("[%t] ERROR framing error in reply", $time);
                    end
                end
                begin
                    tx(b0);
                    if (n > 1) tx(b1);
                    if (n > 2) tx(b2);
                end
            join
            reply = d[7:0];
        end
    endtask

    task wr(input [7:0] a, input [7:0] v);
        reg [7:0] r;
        begin
            transact(3, "W", a, v, r);
            check8("write ack", r, "K");
        end
    endtask

    task rd(input [7:0] a, input [7:0] exp);
        reg [7:0] r;
        begin
            transact(2, "R", a, 8'h00, r);
            check8("register read", r, exp);
        end
    endtask

    // --------------------------------------------------------------- stimulus
    integer   i, seed;
    reg [7:0] r;
    reg [7:0] echo_tx [0:255];
    reg [8:0] d;
    reg       pe, fe;

    initial begin
        $timeformat(-9, 0, " ns", 12);
        seed = 5;
        if ($test$plusargs("vcd")) begin
            $dumpfile("build/tb_uart_demo.vcd");
            $dumpvars(0, tb_uart_demo);
        end
        #(30 * CLK_NS);
        #(2 * BIT_NS);

        // ------------------------------------------------ echo mode
        $display("[%t] --- echo mode: 128 back-to-back random bytes", $time);
        for (i = 0; i < 128; i = i + 1) echo_tx[i] = $random(seed);
        fork
            for (i = 0; i < 128; i = i + 1) tx(echo_tx[i]);
            begin : echo_rx
                integer j;
                for (j = 0; j < 128; j = j + 1) begin
                    pc.recv(d, pe, fe);
                    check8("echo byte", d[7:0], echo_tx[j]);
                end
            end
        join
        #(2 * BIT_NS);
        check8("led = last rx byte", led[7:0], echo_tx[127]);
        check8("cmd-mode led off", {7'b0, led[15]}, 8'h00);

        // ------------------------------------------------ command mode
        $display("[%t] --- command mode: register protocol", $time);
        sw = 16'hC3A7;                       // sw[15]=1 -> command mode
        #(10 * CLK_NS);
        check8("cmd-mode led on", {7'b0, led[15]}, 8'h01);

        rd(8'h00, 8'h55);                    // ID
        rd(8'h01, 8'h10);                    // VERSION
        wr(8'h02, 8'hA5);                    // LED
        check8("led pins", led[7:0], 8'hA5);
        rd(8'h02, 8'hA5);
        rd(8'h03, 8'hA7);                    // SW_LO
        rd(8'h04, 8'hC3);                    // SW_HI
        wr(8'h05, 8'h3C);                    // SCRATCH
        rd(8'h05, 8'h3C);
        rd(8'h08, 8'h00);                    // STATUS clean

        transact(1, "x", 8'h00, 8'h00, r);   // unknown command
        check8("unknown cmd -> '?'", r, "?");

        tx(8'h0D); tx(8'h0A);                // CR/LF ignored, no reply
        // RXCNT is sampled while the address byte is consumed, so it already
        // includes this command's 'R' but not the address byte itself.
        rd(8'h06, sent[7:0] + 8'd1);

        // inject a framing error -> STATUS.ferr, then clear it
        pc.send(9'h0F0, 0.0, 1'b0, 1'b1);
        sent = sent + 1;
        #(2 * BIT_NS);
        check8("framing-error led", {7'b0, led[11]}, 8'h01);
        rd(8'h08, 8'h02);
        wr(8'h08, 8'h00);                    // clear
        rd(8'h08, 8'h00);
        check8("framing-error led cleared", {7'b0, led[11]}, 8'h00);

        // ------------------------------------------------ summary
        #(4 * BIT_NS);
        $display("tb_uart_demo: %0d checks, %0d errors", checks, errors);
        if (errors == 0) $display("*** PASS ***");
        else             $display("*** FAIL ***");
        $finish;
    end

    initial begin
        #(2000 * 12 * BIT_NS);
        $display("*** FAIL *** (watchdog timeout)");
        $finish;
    end

endmodule

`default_nettype wire
