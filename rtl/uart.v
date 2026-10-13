// -----------------------------------------------------------------------------
// uart.v  --  parameterised UART controller (top level of the IP)
//
//            +-----------+   +---------+
//  s_tx_* -->| TX FIFO   |-->| uart_tx |--> txd
//            +-----------+   +---------+
//                               ^ tick
//                         +---------------+
//                         | uart_baud_gen |  (fractional NCO, BAUD*OVERSAMPLE)
//                         +---------------+
//                               v tick
//            +-----------+   +---------+
//  m_rx_* <--| RX FIFO   |<--| uart_rx |<-- rxd
//            +-----------+   +---------+
//
// Both user interfaces are AXI-Stream style valid/ready handshakes.
// Every received word carries its own parity / framing / break flags.
// An RX FIFO overflow sets the sticky `rx_overrun` flag (word is dropped).
// -----------------------------------------------------------------------------
`timescale 1ns/1ps
`default_nettype none

module uart #(
    parameter integer CLK_FREQ   = 100_000_000,  // Hz
    parameter integer BAUD_RATE  = 115_200,      // bit/s
    parameter integer DATA_BITS  = 8,            // 5..9
    parameter integer PARITY     = 0,            // 0 none, 1 even, 2 odd
    parameter integer STOP_BITS  = 1,            // 1 or 2
    parameter integer OVERSAMPLE = 16,           // RX samples per bit, >= 8
    parameter integer FIFO_DEPTH = 16,           // power of two, >= 2
    parameter integer ACC_WIDTH  = 24            // baud NCO resolution
) (
    input  wire                         clk,
    input  wire                         rst,          // synchronous, active high

    // transmit stream
    input  wire [DATA_BITS-1:0]         s_tx_data,
    input  wire                         s_tx_valid,
    output wire                         s_tx_ready,

    // receive stream
    output wire [DATA_BITS-1:0]         m_rx_data,
    output wire                         m_rx_perr,
    output wire                         m_rx_ferr,
    output wire                         m_rx_break,
    output wire                         m_rx_valid,
    input  wire                         m_rx_ready,

    // serial pins
    output wire                         txd,
    input  wire                         rxd,

    // status
    output wire                         tx_busy,      // FIFO not empty or shifting
    output wire [$clog2(FIFO_DEPTH):0]  tx_fifo_count,
    output wire [$clog2(FIFO_DEPTH):0]  rx_fifo_count,
    output reg                          rx_overrun,   // sticky
    input  wire                         clr_overrun
);

    // ------------------------------------------------------------- baud NCO
    // INC = round(BAUD * OVERSAMPLE * 2^ACC_WIDTH / CLK_FREQ), in 64-bit math
    localparam [63:0] CLK_64      = CLK_FREQ;
    localparam [63:0] BAUD_64     = BAUD_RATE;
    localparam [63:0] OS_64       = OVERSAMPLE;
    localparam [63:0] BAUD_INC_64 =
        (((64'd1 << ACC_WIDTH) * BAUD_64 * OS_64) + CLK_64 / 2) / CLK_64;
    localparam [ACC_WIDTH-1:0] BAUD_INC = BAUD_INC_64[ACC_WIDTH-1:0];

`ifndef SYNTHESIS
    initial begin
        if (BAUD_INC_64 == 0 || BAUD_INC_64 >= (64'd1 << ACC_WIDTH)) begin
            $display("uart: ERROR - CLK_FREQ (%0d) must exceed BAUD_RATE*OVERSAMPLE (%0d)",
                     CLK_FREQ, BAUD_RATE * OVERSAMPLE);
            $finish;
        end
        if (DATA_BITS < 5 || DATA_BITS > 9 || PARITY < 0 || PARITY > 2 ||
            STOP_BITS < 1 || STOP_BITS > 2 || OVERSAMPLE < 8) begin
            $display("uart: ERROR - unsupported parameter combination");
            $finish;
        end
        $display("uart: CLK=%0d Hz BAUD=%0d %0d%s%0d OS=%0d  NCO inc=%0d -> actual %.1f baud (%.4f %%)",
                 CLK_FREQ, BAUD_RATE, DATA_BITS,
                 (PARITY == 0) ? "N" : (PARITY == 1) ? "E" : "O", STOP_BITS, OVERSAMPLE,
                 BAUD_INC,
                 CLK_FREQ * 1.0 * BAUD_INC / (2.0 ** ACC_WIDTH) / OVERSAMPLE,
                 100.0 * ((CLK_FREQ * 1.0 * BAUD_INC / (2.0 ** ACC_WIDTH) / OVERSAMPLE)
                          - BAUD_RATE) / BAUD_RATE);
    end
`endif

    wire tick;

    uart_baud_gen #(.ACC_WIDTH(ACC_WIDTH)) u_baud (
        .clk  (clk),
        .rst  (rst),
        .inc  (BAUD_INC),
        .tick (tick)
    );

    // ------------------------------------------------------------- transmit
    wire [DATA_BITS-1:0] txf_data;
    wire                 txf_empty, txf_full;
    wire                 tx_ready, tx_shift_busy;
    wire                 tx_load = !txf_empty && tx_ready;

    assign s_tx_ready = !txf_full;

    sync_fifo #(.WIDTH(DATA_BITS), .DEPTH(FIFO_DEPTH)) u_tx_fifo (
        .clk     (clk),
        .rst     (rst),
        .wr_en   (s_tx_valid),
        .wr_data (s_tx_data),
        .full    (txf_full),
        .rd_en   (tx_load),
        .rd_data (txf_data),
        .empty   (txf_empty),
        .count   (tx_fifo_count)
    );

    uart_tx #(
        .DATA_BITS (DATA_BITS),
        .PARITY    (PARITY),
        .STOP_BITS (STOP_BITS),
        .OVERSAMPLE(OVERSAMPLE)
    ) u_tx (
        .clk     (clk),
        .rst     (rst),
        .tick    (tick),
        .s_data  (txf_data),
        .s_valid (!txf_empty),
        .s_ready (tx_ready),
        .txd     (txd),
        .busy    (tx_shift_busy)
    );

    assign tx_busy = !txf_empty || tx_shift_busy;

    // -------------------------------------------------------------- receive
    wire [DATA_BITS-1:0] rx_data;
    wire                 rx_valid, rx_perr, rx_ferr, rx_brk;
    wire                 rxf_full, rxf_empty;

    uart_rx #(
        .DATA_BITS (DATA_BITS),
        .PARITY    (PARITY),
        .OVERSAMPLE(OVERSAMPLE)
    ) u_rx (
        .clk   (clk),
        .rst   (rst),
        .tick  (tick),
        .rxd   (rxd),
        .data  (rx_data),
        .valid (rx_valid),
        .perr  (rx_perr),
        .ferr  (rx_ferr),
        .brk   (rx_brk)
    );

    sync_fifo #(.WIDTH(DATA_BITS + 3), .DEPTH(FIFO_DEPTH)) u_rx_fifo (
        .clk     (clk),
        .rst     (rst),
        .wr_en   (rx_valid),
        .wr_data ({rx_brk, rx_ferr, rx_perr, rx_data}),
        .full    (rxf_full),
        .rd_en   (m_rx_valid && m_rx_ready),
        .rd_data ({m_rx_break, m_rx_ferr, m_rx_perr, m_rx_data}),
        .empty   (rxf_empty),
        .count   (rx_fifo_count)
    );

    assign m_rx_valid = !rxf_empty;

    always @(posedge clk) begin
        if (rst || clr_overrun)
            rx_overrun <= 1'b0;
        else if (rx_valid && rxf_full)
            rx_overrun <= 1'b1;
    end

endmodule

`default_nettype wire
