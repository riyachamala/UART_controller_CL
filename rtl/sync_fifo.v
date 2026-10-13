// -----------------------------------------------------------------------------
// sync_fifo.v
//
// Single-clock, first-word-fall-through FIFO. DEPTH must be a power of two
// (>= 2). Uses the classic extra-MSB pointer scheme for full/empty.
// Writes while full and reads while empty are ignored.
// -----------------------------------------------------------------------------
`timescale 1ns/1ps
`default_nettype none

module sync_fifo #(
    parameter integer WIDTH = 8,
    parameter integer DEPTH = 16
) (
    input  wire                     clk,
    input  wire                     rst,

    input  wire                     wr_en,
    input  wire [WIDTH-1:0]         wr_data,
    output wire                     full,

    input  wire                     rd_en,
    output wire [WIDTH-1:0]         rd_data,   // valid whenever !empty
    output wire                     empty,

    output wire [$clog2(DEPTH):0]   count
);

    localparam integer AW = $clog2(DEPTH);

    reg [WIDTH-1:0] mem [0:DEPTH-1];
    reg [AW:0]      wptr, rptr;

    assign empty   = (wptr == rptr);
    assign full    = (wptr[AW] != rptr[AW]) && (wptr[AW-1:0] == rptr[AW-1:0]);
    assign count   = wptr - rptr;
    assign rd_data = mem[rptr[AW-1:0]];

    always @(posedge clk) begin
        if (rst) begin
            wptr <= {(AW+1){1'b0}};
            rptr <= {(AW+1){1'b0}};
        end else begin
            if (wr_en && !full) begin
                mem[wptr[AW-1:0]] <= wr_data;
                wptr <= wptr + 1'b1;
            end
            if (rd_en && !empty)
                rptr <= rptr + 1'b1;
        end
    end

endmodule

`default_nettype wire
