// -----------------------------------------------------------------------------
// uart_baud_gen.v
//
// Fractional (NCO / phase-accumulator) baud tick generator.
//
// Produces a single-cycle `tick` pulse at an average rate of
//     f_tick = f_clk * INC / 2^ACC_WIDTH
// where INC is normally chosen so that f_tick = BAUD_RATE * OVERSAMPLE.
//
// Unlike a plain integer divider, the accumulator carries the fractional
// remainder forward, so the long-term baud error depends only on ACC_WIDTH
// (e.g. < 0.01 % at 24 bits) instead of on how evenly f_clk divides the baud
// rate. Individual ticks jitter by at most one clock cycle, which is
// negligible next to the 1/OVERSAMPLE-bit sampling resolution.
// -----------------------------------------------------------------------------
`timescale 1ns/1ps
`default_nettype none

module uart_baud_gen #(
    parameter integer ACC_WIDTH = 24
) (
    input  wire                 clk,
    input  wire                 rst,       // synchronous, active high
    input  wire [ACC_WIDTH-1:0] inc,       // phase increment per clock
    output reg                  tick       // 1-cycle pulse on accumulator wrap
);

    reg [ACC_WIDTH-1:0] acc;

    always @(posedge clk) begin
        if (rst) begin
            acc  <= {ACC_WIDTH{1'b0}};
            tick <= 1'b0;
        end else begin
            // Carry out of the accumulator is the tick.
            {tick, acc} <= {1'b0, acc} + {1'b0, inc};
        end
    end

endmodule

`default_nettype wire
