// -----------------------------------------------------------------------------
// tb_uart.v  --  self-checking testbench for the parameterised UART
//
// Two independent checkers run for the whole simulation:
//   * TX monitor   : a behavioural UART (uart_bfm) decodes the DUT's txd pin and
//                    compares every frame against what was pushed into s_tx_*.
//   * RX scoreboard: every word popped from m_rx_* (with random back-pressure)
//                    is compared, including its perr/ferr/break flags, against
//                    an expectation queue.
//
// Tests
//   1. loopback      random words, random idle gaps, txd -> rxd
//   2. patterns      all-0, all-1, 0x55/0xAA style, walking ones
//   3. throughput    a full FIFO burst must go out gap-free at the exact baud
//   4. baud error    external TX at -3 %, -1.5 %, 0, +1.5 %, +3 % (back-to-back)
//   5. parity error  wrong parity bit flagged, data still delivered
//   6. framing error stop bit = 0 flagged, receiver recovers
//   7. break         line held low > 2 frames -> one break word, then recovery
//   8. glitch        short low pulses must not be taken as start bits
//   9. overrun       RX FIFO overflow sets sticky flag, earlier data intact
//
// All parameters can be overridden from the command line, e.g.
//   iverilog -Ptb_uart.DATA_BITS=7 -Ptb_uart.PARITY=2 ...
// Pass +vcd to vvp to dump waves to build/tb_uart.vcd.
// -----------------------------------------------------------------------------
`timescale 1ns/1ps
`default_nettype none

module tb_uart;

    parameter integer CLK_FREQ   = 50_000_000;
    parameter integer BAUD_RATE  = 1_000_000;
    parameter integer DATA_BITS  = 8;
    parameter integer PARITY     = 0;
    parameter integer STOP_BITS  = 1;
    parameter integer OVERSAMPLE = 16;
    parameter integer FIFO_DEPTH = 16;
    parameter integer NUM_WORDS  = 200;
    parameter integer SEED       = 1;
    parameter real    MAX_RATE_ERR = 0.03;   // remote-TX baud error for test 4

    localparam real    CLK_NS     = 1.0e9 / CLK_FREQ;
    localparam real    BIT_NS     = 1.0e9 / BAUD_RATE;
    localparam integer FRAME_BITS = 1 + DATA_BITS + (PARITY != 0 ? 1 : 0) + STOP_BITS;
    localparam [8:0]   DMASK      = (9'h1 << DATA_BITS) - 9'h1;
    localparam integer QSIZE      = 4096;

    // ------------------------------------------------------------ clock/reset
    reg clk = 1'b0;
    always #(CLK_NS / 2.0) clk = ~clk;

    reg rst = 1'b1;

    // ------------------------------------------------------------------- DUT
    reg  [DATA_BITS-1:0]       s_tx_data  = {DATA_BITS{1'b0}};
    reg                        s_tx_valid = 1'b0;
    wire                       s_tx_ready;
    wire [DATA_BITS-1:0]       m_rx_data;
    wire                       m_rx_perr, m_rx_ferr, m_rx_break, m_rx_valid;
    reg                        m_rx_ready = 1'b0;
    wire                       txd, rxd;
    wire                       tx_busy, rx_overrun;
    wire [$clog2(FIFO_DEPTH):0] tx_fifo_count, rx_fifo_count;
    reg                        clr_overrun = 1'b0;

    reg  loopback = 1'b1;
    wire model_txd;
    assign rxd = loopback ? txd : model_txd;

    uart #(
        .CLK_FREQ  (CLK_FREQ),
        .BAUD_RATE (BAUD_RATE),
        .DATA_BITS (DATA_BITS),
        .PARITY    (PARITY),
        .STOP_BITS (STOP_BITS),
        .OVERSAMPLE(OVERSAMPLE),
        .FIFO_DEPTH(FIFO_DEPTH)
    ) dut (
        .clk          (clk),
        .rst          (rst),
        .s_tx_data    (s_tx_data),
        .s_tx_valid   (s_tx_valid),
        .s_tx_ready   (s_tx_ready),
        .m_rx_data    (m_rx_data),
        .m_rx_perr    (m_rx_perr),
        .m_rx_ferr    (m_rx_ferr),
        .m_rx_break   (m_rx_break),
        .m_rx_valid   (m_rx_valid),
        .m_rx_ready   (m_rx_ready),
        .txd          (txd),
        .rxd          (rxd),
        .tx_busy      (tx_busy),
        .tx_fifo_count(tx_fifo_count),
        .rx_fifo_count(rx_fifo_count),
        .rx_overrun   (rx_overrun),
        .clr_overrun  (clr_overrun)
    );

    // External line partner (driver) and DUT-TX monitor
    uart_bfm #(.DATA_BITS(DATA_BITS), .PARITY(PARITY), .STOP_BITS(STOP_BITS),
               .BAUD(BAUD_RATE * 1.0)) drv (.txd(model_txd), .rxd(1'b1));
    uart_bfm #(.DATA_BITS(DATA_BITS), .PARITY(PARITY), .STOP_BITS(STOP_BITS),
               .BAUD(BAUD_RATE * 1.0)) mon (.txd(), .rxd(txd));

    // ------------------------------------------------------------ bookkeeping
    integer seed;
    integer errors     = 0;
    integer n_checks   = 0;

    task fail(input [8*72-1:0] msg);
        begin
            errors = errors + 1;
            $display("[%t] ERROR: %0s", $time, msg);
        end
    endtask

    // TX expectation queue (checked by the monitor)
    reg [8:0]  tx_exp [0:QSIZE-1];
    integer    tx_wr = 0, tx_rd = 0;

    // RX expectation queue {break, ferr, perr, data}
    reg [11:0] rx_exp [0:QSIZE-1];
    integer    rx_wr = 0, rx_rd = 0;

    task expect_rx(input [8:0] d, input pe, input fe, input bk);
        begin
            rx_exp[rx_wr % QSIZE] = {bk, fe, pe, d & DMASK};
            rx_wr = rx_wr + 1;
        end
    endtask

    // ------------------------------------------------------- TX monitor
    always begin : tx_monitor
        reg [8:0] d;
        reg       pe, fe;
        mon.recv(d, pe, fe);
        n_checks = n_checks + 1;
        if (tx_rd == tx_wr) begin
            fail("TX monitor: unexpected frame on txd");
        end else begin
            if (d !== tx_exp[tx_rd % QSIZE] || pe || fe) begin
                $display("        txd frame: got %h perr=%b ferr=%b, expected %h",
                         d, pe, fe, tx_exp[tx_rd % QSIZE]);
                fail("TX monitor: frame mismatch");
            end
            tx_rd = tx_rd + 1;
        end
    end

    // ------------------------------------------------------- RX scoreboard
    reg rx_ready_en = 1'b1;
    always @(posedge clk) begin : rx_scoreboard
        reg [11:0] got;
        m_rx_ready <= rx_ready_en && (({$random(seed)} % 4) != 0);
        if (m_rx_valid && m_rx_ready) begin
            n_checks = n_checks + 1;
            got = {m_rx_break, m_rx_ferr, m_rx_perr, 9'd0 | m_rx_data};
            if (rx_rd == rx_wr) begin
                $display("        rx word: got %h", got);
                fail("RX scoreboard: unexpected word");
            end else begin
                if (got !== rx_exp[rx_rd % QSIZE]) begin
                    $display("        rx word {brk,ferr,perr,data}: got %h expected %h",
                             got, rx_exp[rx_rd % QSIZE]);
                    fail("RX scoreboard: mismatch");
                end
                rx_rd = rx_rd + 1;
            end
        end
    end

    // ------------------------------------------------------------- helpers
    task send_word(input [8:0] d);
        begin
            @(negedge clk);
            s_tx_data  = d[DATA_BITS-1:0];
            s_tx_valid = 1'b1;
            while (!s_tx_ready) @(negedge clk);
            tx_exp[tx_wr % QSIZE] = d & DMASK;
            tx_wr = tx_wr + 1;
            @(negedge clk);
            s_tx_valid = 1'b0;
        end
    endtask

    task wait_bits(input real n);
        #(n * BIT_NS);
    endtask

    task drain;
        begin
            while (rx_rd != rx_wr || tx_rd != tx_wr || tx_busy || m_rx_valid)
                @(posedge clk);
            wait_bits(2);
        end
    endtask

    function [8:0] rand_word(input integer dummy);
        rand_word = $random(seed) & DMASK;
    endfunction

    integer test_no = 0;
    task banner(input [8*40-1:0] name);
        begin
            test_no = test_no + 1;
            $display("[%t] --- test %0d: %0s", $time, test_no, name);
        end
    endtask

    // ------------------------------------------------------------- stimulus
    integer i, k, n;
    reg [8:0] w;
    real    t_start, t_end, t_exp, act_bit_ns;
    real    rate_err [0:4];

    initial begin
        seed = SEED;
        $timeformat(-9, 0, " ns", 12);
        rate_err[0] = -MAX_RATE_ERR;       rate_err[1] = -MAX_RATE_ERR / 2.0;
        rate_err[2] = 0.0;
        rate_err[3] =  MAX_RATE_ERR / 2.0; rate_err[4] =  MAX_RATE_ERR;
        act_bit_ns = 1.0e9 / (CLK_FREQ * 1.0 * dut.BAUD_INC / (2.0 ** dut.ACC_WIDTH)
                              / OVERSAMPLE);

        if ($test$plusargs("vcd")) begin
            $dumpfile("build/tb_uart.vcd");
            $dumpvars(0, tb_uart);
        end

        $display("tb_uart: %0d%s%0d @ %0d baud, clk %0d Hz, OS=%0d, FIFO=%0d, seed=%0d",
                 DATA_BITS, (PARITY == 0) ? "N" : (PARITY == 1) ? "E" : "O",
                 STOP_BITS, BAUD_RATE, CLK_FREQ, OVERSAMPLE, FIFO_DEPTH, SEED);

        repeat (10) @(posedge clk);
        rst = 1'b0;
        wait_bits(2);

        // ---------------------------------------------------------- 1
        banner("loopback, random data and gaps");
        loopback = 1'b1;
        for (i = 0; i < NUM_WORDS; i = i + 1) begin
            w = rand_word(0);
            expect_rx(w, 1'b0, 1'b0, 1'b0);
            send_word(w);
            k = {$random(seed)} % 8;
            if (k == 0) wait_bits(({$random(seed)} % 300) / 100.0);  // idle line
            else        repeat (k) @(posedge clk);
        end
        drain;

        // ---------------------------------------------------------- 2
        banner("loopback, corner-case patterns");
        expect_rx(9'h000, 0, 0, 0); send_word(9'h000);
        expect_rx(DMASK,  0, 0, 0); send_word(DMASK);
        expect_rx(9'h155, 0, 0, 0); send_word(9'h155);
        expect_rx(9'h0AA, 0, 0, 0); send_word(9'h0AA);
        for (i = 0; i < DATA_BITS; i = i + 1) begin
            expect_rx(9'h1 << i, 0, 0, 0);          send_word(9'h1 << i);
            expect_rx(~(9'h1 << i), 0, 0, 0);       send_word(~(9'h1 << i));
        end
        drain;

        // ---------------------------------------------------------- 3
        banner("back-to-back throughput (gap-free)");
        n = FIFO_DEPTH;
        fork
            begin
                @(negedge txd);
                t_start = $realtime;
            end
            for (i = 0; i < n; i = i + 1) begin
                w = rand_word(0);
                expect_rx(w, 0, 0, 0);
                send_word(w);
            end
        join
        wait (!tx_busy);
        t_end = $realtime;
        t_exp = n * FRAME_BITS * act_bit_ns;
        $display("        %0d frames in %.1f ns, ideal %.1f ns (%0d bits each)",
                 n, t_end - t_start, t_exp, FRAME_BITS);
        if ((t_end - t_start) > t_exp + act_bit_ns / OVERSAMPLE + 4 * CLK_NS ||
            (t_end - t_start) < t_exp - act_bit_ns / OVERSAMPLE - 4 * CLK_NS)
            fail("throughput: burst duration wrong (gaps between frames?)");
        drain;

        // external transmitter from here on
        loopback = 1'b0;
        wait_bits(2);

        // ---------------------------------------------------------- 4
        banner("baud-rate tolerance (remote TX off-frequency)");
        for (k = 0; k < 5; k = k + 1) begin
            $display("        remote TX rate error %.1f %%", rate_err[k] * 100.0);
            for (i = 0; i < 24; i = i + 1) begin
                w = rand_word(0);
                expect_rx(w, 0, 0, 0);
                drv.send(w, rate_err[k], 1'b0, 1'b0);    // back-to-back frames
            end
            drain;
        end

        // ---------------------------------------------------------- 5
        if (PARITY != 0) begin
            banner("parity error detection");
            for (i = 0; i < 16; i = i + 1) begin
                w = rand_word(0);
                expect_rx(w, i[0], 0, 0);
                drv.send(w, 0.0, i[0], 1'b0);           // every other frame bad
            end
            drain;
        end

        // ---------------------------------------------------------- 6
        banner("framing error detection + recovery");
        for (i = 0; i < 8; i = i + 1) begin
            w = rand_word(0) | 9'h001;                  // non-zero: not a break
            expect_rx(w, 0, 1, 0);
            drv.send(w, 0.0, 1'b0, 1'b1);
            w = rand_word(0);
            expect_rx(w, 0, 0, 0);
            drv.send(w, 0.0, 1'b0, 1'b0);
        end
        drain;

        // ---------------------------------------------------------- 7
        banner("break detection + recovery");
        expect_rx(9'h000, 0, 1, 1);
        drv.hold_low(2.5 * FRAME_BITS * BIT_NS);
        w = rand_word(0);
        expect_rx(w, 0, 0, 0);
        drv.send(w, 0.0, 1'b0, 1'b0);
        drain;

        // ---------------------------------------------------------- 8
        banner("glitch rejection");
        for (i = 0; i < 8; i = i + 1) begin
            drv.glitch(BIT_NS * (0.10 + 0.04 * i));     // 0.10 .. 0.38 bit
            wait_bits(FRAME_BITS + 1);
        end
        w = rand_word(0);
        expect_rx(w, 0, 0, 0);
        drv.send(w, 0.0, 1'b0, 1'b0);
        drain;

        // ---------------------------------------------------------- 9
        banner("RX FIFO overrun");
        rx_ready_en = 1'b0;
        repeat (2) @(posedge clk);
        for (i = 0; i < FIFO_DEPTH + 3; i = i + 1) begin
            w = rand_word(0);
            if (i < FIFO_DEPTH) expect_rx(w, 0, 0, 0);  // the rest is dropped
            drv.send(w, 0.0, 1'b0, 1'b0);
        end
        wait_bits(2);
        if (!rx_overrun)                  fail("overrun flag not set");
        if (rx_fifo_count != FIFO_DEPTH)  fail("RX FIFO should be full");
        rx_ready_en = 1'b1;
        drain;
        if (!rx_overrun)                  fail("overrun flag must be sticky");
        @(negedge clk) clr_overrun = 1'b1;
        @(negedge clk) clr_overrun = 1'b0;
        if (rx_overrun)                   fail("overrun flag did not clear");

        // ------------------------------------------------------- summary
        wait_bits(4);
        if (tx_rd != tx_wr) fail("TX frames missing");
        if (rx_rd != rx_wr) fail("RX words missing");
        $display("tb_uart: %0d tests, %0d checks (%0d TX frames, %0d RX words), %0d errors",
                 test_no, n_checks, tx_rd, rx_rd, errors);
        if (errors == 0) $display("*** PASS ***");
        else             $display("*** FAIL ***");
        $finish;
    end

    // watchdog
    initial begin
        #((NUM_WORDS + 600) * FRAME_BITS * BIT_NS * 3.0);
        $display("*** FAIL *** (watchdog timeout)");
        $finish;
    end

endmodule

`default_nettype wire
