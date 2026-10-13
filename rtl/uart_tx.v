// -----------------------------------------------------------------------------
// uart_tx.v
//
// UART transmitter.
//   * DATA_BITS  : 5..9 data bits, sent LSB first
//   * PARITY     : 0 = none, 1 = even, 2 = odd
//   * STOP_BITS  : 1 or 2
//   * OVERSAMPLE : ticks per bit (must match the baud generator)
//
// Input is a valid/ready stream. A word is accepted only on a baud tick, so
// every bit (including the start bit) lasts exactly OVERSAMPLE ticks. When a
// new word is waiting at the end of the last stop bit it is loaded on that
// same tick, giving gap-free back-to-back frames.
// -----------------------------------------------------------------------------
`timescale 1ns/1ps
`default_nettype none

module uart_tx #(
    parameter integer DATA_BITS  = 8,
    parameter integer PARITY     = 0,
    parameter integer STOP_BITS  = 1,
    parameter integer OVERSAMPLE = 16
) (
    input  wire                 clk,
    input  wire                 rst,
    input  wire                 tick,      // OVERSAMPLE x baud enable

    input  wire [DATA_BITS-1:0] s_data,
    input  wire                 s_valid,
    output wire                 s_ready,

    output reg                  txd,
    output wire                 busy
);

    localparam integer CW = (OVERSAMPLE > 1) ? $clog2(OVERSAMPLE) : 1;

    localparam [2:0] S_IDLE  = 3'd0,
                     S_START = 3'd1,
                     S_DATA  = 3'd2,
                     S_PAR   = 3'd3,
                     S_STOP  = 3'd4;

    localparam [CW-1:0] END_CNT   = OVERSAMPLE - 1;
    localparam [3:0]    LAST_DATA = DATA_BITS - 1;
    localparam [3:0]    LAST_STOP = STOP_BITS - 1;

    reg [2:0]           state;
    reg [CW-1:0]        os_cnt;
    reg [3:0]           bit_cnt;
    reg [DATA_BITS-1:0] shreg;
    reg                 par_bit;

    wire bit_end   = tick && (os_cnt == END_CNT);
    wire last_stop = (state == S_STOP) && bit_end && (bit_cnt == LAST_STOP);

    assign s_ready = tick && ((state == S_IDLE) || last_stop);
    assign busy    = (state != S_IDLE);

    wire load = s_valid && s_ready;

    always @(posedge clk) begin
        if (rst) begin
            state   <= S_IDLE;
            txd     <= 1'b1;
            os_cnt  <= {CW{1'b0}};
            bit_cnt <= 4'd0;
            shreg   <= {DATA_BITS{1'b0}};
            par_bit <= 1'b0;
        end else if (load) begin
            shreg   <= s_data;
            // even: make total number of ones even; odd: make it odd
            par_bit <= (^s_data) ^ (PARITY == 2);
            txd     <= 1'b0;                        // start bit
            state   <= S_START;
            os_cnt  <= {CW{1'b0}};
        end else if (tick && state != S_IDLE) begin
            os_cnt <= bit_end ? {CW{1'b0}} : os_cnt + 1'b1;
            if (bit_end) begin
                case (state)
                    S_START: begin
                        state   <= S_DATA;
                        txd     <= shreg[0];
                        bit_cnt <= 4'd0;
                    end
                    S_DATA: begin
                        if (bit_cnt == LAST_DATA) begin
                            bit_cnt <= 4'd0;
                            if (PARITY != 0) begin
                                state <= S_PAR;
                                txd   <= par_bit;
                            end else begin
                                state <= S_STOP;
                                txd   <= 1'b1;
                            end
                        end else begin
                            bit_cnt <= bit_cnt + 4'd1;
                            shreg   <= shreg >> 1;
                            txd     <= shreg[1];
                        end
                    end
                    S_PAR: begin
                        state   <= S_STOP;
                        txd     <= 1'b1;
                        bit_cnt <= 4'd0;
                    end
                    S_STOP: begin
                        if (bit_cnt == LAST_STOP)
                            state <= S_IDLE;           // txd stays high
                        else
                            bit_cnt <= bit_cnt + 4'd1;
                    end
                    default: state <= S_IDLE;
                endcase
            end
        end
    end

endmodule

`default_nettype wire
