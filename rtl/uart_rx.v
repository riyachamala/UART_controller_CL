// -----------------------------------------------------------------------------
// uart_rx.v
//
// UART receiver with:
//   * 2-flop synchronizer on the asynchronous serial input
//   * OVERSAMPLE x oversampling, start-edge re-alignment every frame
//   * 3-sample majority vote at the centre of every bit (noise immunity)
//   * false-start (glitch) rejection: start bit is re-checked at mid-bit
//   * parity, framing and break detection
//
// Only the first stop bit is checked and the receiver re-arms in the middle of
// it. This is what gives the receiver its tolerance to a transmitter that runs
// slightly fast: the next start edge may arrive before "our" stop bit ends.
//
// Outputs are presented for one cycle with `valid`; there is no back-pressure
// (a serial line cannot be stalled), buffering is done by the parent module.
// -----------------------------------------------------------------------------
`timescale 1ns/1ps
`default_nettype none

module uart_rx #(
    parameter integer DATA_BITS  = 8,
    parameter integer PARITY     = 0,      // 0 none, 1 even, 2 odd
    parameter integer OVERSAMPLE = 16      // >= 8
) (
    input  wire                 clk,
    input  wire                 rst,
    input  wire                 tick,      // OVERSAMPLE x baud enable
    input  wire                 rxd,       // asynchronous serial input

    output reg  [DATA_BITS-1:0] data,
    output reg                  valid,     // 1-cycle strobe
    output reg                  perr,      // parity error
    output reg                  ferr,      // framing error (stop bit = 0)
    output reg                  brk        // break (whole frame = 0)
);

    localparam integer CW  = $clog2(OVERSAMPLE);
    localparam integer MID = OVERSAMPLE / 2;   // samples at MID-2, MID-1, MID

    localparam [CW-1:0] SAMP0_CNT = MID - 2;
    localparam [CW-1:0] SAMP1_CNT = MID - 1;
    localparam [CW-1:0] EVAL_CNT  = MID;
    localparam [CW-1:0] END_CNT   = OVERSAMPLE - 1;
    localparam [3:0]    LAST_DATA = DATA_BITS - 1;

    localparam [2:0] S_IDLE  = 3'd0,
                     S_START = 3'd1,
                     S_DATA  = 3'd2,
                     S_PAR   = 3'd3,
                     S_STOP  = 3'd4,
                     S_WAIT  = 3'd5;           // wait for line to return high

    // ---------------------------------------------------------------- sync
    (* ASYNC_REG = "TRUE" *) reg [1:0] sync_ff;
    always @(posedge clk) begin
        if (rst) sync_ff <= 2'b11;
        else     sync_ff <= {sync_ff[0], rxd};
    end
    wire rx = sync_ff[1];

    // ---------------------------------------------------------------- core
    reg [2:0]           state;
    reg [CW-1:0]        os_cnt;
    reg [3:0]           bit_cnt;
    reg [DATA_BITS-1:0] shreg;
    reg [1:0]           samp;
    reg                 par_bit;

    // majority of the two stored samples and the current one
    wire maj      = (samp[1] & samp[0]) | (samp[1] & rx) | (samp[0] & rx);
    wire at_eval  = (os_cnt == EVAL_CNT);
    wire at_end   = (os_cnt == END_CNT);

    wire par_exp  = (^shreg) ^ (PARITY == 2);
    wire is_break = !maj && (shreg == {DATA_BITS{1'b0}}) &&
                    ((PARITY == 0) || !par_bit);

    always @(posedge clk) begin
        valid <= 1'b0;
        if (rst) begin
            state   <= S_WAIT;
            os_cnt  <= {CW{1'b0}};
            bit_cnt <= 4'd0;
            shreg   <= {DATA_BITS{1'b0}};
            samp    <= 2'b11;
            par_bit <= 1'b0;
            data    <= {DATA_BITS{1'b0}};
            perr    <= 1'b0;
            ferr    <= 1'b0;
            brk     <= 1'b0;
        end else if (tick) begin
            if (os_cnt == SAMP0_CNT || os_cnt == SAMP1_CNT)
                samp <= {samp[0], rx};
            os_cnt <= at_end ? {CW{1'b0}} : os_cnt + 1'b1;

            case (state)
                S_IDLE: begin
                    // the tick on which the low level is first seen is count 0
                    if (!rx) begin
                        state  <= S_START;
                        os_cnt <= {{(CW-1){1'b0}}, 1'b1};
                    end else begin
                        os_cnt <= {CW{1'b0}};
                    end
                end

                S_START: begin
                    if (at_eval && maj)
                        state <= S_IDLE;                // glitch, not a start bit
                    else if (at_end) begin
                        state   <= S_DATA;
                        bit_cnt <= 4'd0;
                    end
                end

                S_DATA: begin
                    if (at_eval)
                        shreg <= {maj, shreg[DATA_BITS-1:1]};   // LSB first
                    if (at_end) begin
                        if (bit_cnt == LAST_DATA)
                            state <= (PARITY != 0) ? S_PAR : S_STOP;
                        else
                            bit_cnt <= bit_cnt + 4'd1;
                    end
                end

                S_PAR: begin
                    if (at_eval) par_bit <= maj;
                    if (at_end)  state   <= S_STOP;
                end

                S_STOP: begin
                    if (at_eval) begin
                        data  <= shreg;
                        valid <= 1'b1;
                        ferr  <= !maj;
                        brk   <= is_break;
                        perr  <= (PARITY != 0) && (par_bit != par_exp) && !is_break;
                        // re-arm mid stop bit; after a framing error first wait
                        // for the line to go idle so a break is reported once
                        state <= maj ? S_IDLE : S_WAIT;
                    end
                end

                S_WAIT: begin
                    if (rx) state <= S_IDLE;
                end

                default: state <= S_WAIT;
            endcase
        end
    end

endmodule

`default_nettype wire
