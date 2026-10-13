// -----------------------------------------------------------------------------
// uart_bfm.v  --  behavioural UART bus-functional model (testbench only)
//
// A completely independent, timing-based (not clock-based) model of a UART
// line partner, i.e. the "PC" side. It is used both to drive stimulus into the
// DUT receiver and to monitor / decode the DUT transmitter.
//
//   send(data, rate_err, bad_parity, bad_stop)  drive one frame on `txd`
//        rate_err = +0.03 -> this transmitter runs 3 % fast
//   recv(data, perr, ferr)                      decode one frame from `rxd`
//   glitch(ns)                                  drive a short low pulse
//   hold_low(ns)                                drive a break condition
// -----------------------------------------------------------------------------
`timescale 1ns/1ps

module uart_bfm #(
    parameter integer DATA_BITS = 8,
    parameter integer PARITY    = 0,
    parameter integer STOP_BITS = 1,
    parameter real    BAUD      = 115200.0
) (
    output reg  txd,
    input  wire rxd
);

    real bit_ns;

    initial begin
        txd    = 1'b1;
        bit_ns = 1.0e9 / BAUD;
    end

    task automatic send(input [8:0] data, input real rate_err,
                        input bad_parity, input bad_stop);
        real    t;
        integer i;
        reg     p;
        begin
            t = bit_ns / (1.0 + rate_err);
            p = 1'b0;
            for (i = 0; i < DATA_BITS; i = i + 1) p = p ^ data[i];
            if (PARITY == 2) p = ~p;

            txd = 1'b0;                                   // start
            #(t);
            for (i = 0; i < DATA_BITS; i = i + 1) begin   // data, LSB first
                txd = data[i];
                #(t);
            end
            if (PARITY != 0) begin                        // parity
                txd = p ^ bad_parity;
                #(t);
            end
            txd = !bad_stop;                              // stop bit(s)
            #(t);
            txd = 1'b1;
            if (STOP_BITS == 2) #(t);
            if (bad_stop)       #(t);                     // let the line idle
        end
    endtask

    task automatic recv(output [8:0] data, output perr, output ferr);
        integer i;
        reg     p;
        begin
            data = 9'd0;
            perr = 1'b0;
            ferr = 1'b0;
            @(negedge rxd);
            #(bit_ns / 2.0);                              // centre of start bit
            if (rxd !== 1'b0) ferr = 1'b1;                // runt start bit
            for (i = 0; i < DATA_BITS; i = i + 1) begin
                #(bit_ns);
                data[i] = rxd;
            end
            if (PARITY != 0) begin
                #(bit_ns);
                p = rxd;
                for (i = 0; i < DATA_BITS; i = i + 1) p = p ^ data[i];
                // even parity: total ones (incl. parity bit) must be even
                perr = (PARITY == 1) ? p : ~p;
            end
            #(bit_ns);
            if (rxd !== 1'b1) ferr = 1'b1;
            if (STOP_BITS == 2) begin
                #(bit_ns);
                if (rxd !== 1'b1) ferr = 1'b1;
            end
        end
    endtask

    task automatic glitch(input real ns);
        begin
            txd = 1'b0;
            #(ns);
            txd = 1'b1;
        end
    endtask

    task automatic hold_low(input real ns);
        begin
            txd = 1'b0;
            #(ns);
            txd = 1'b1;
            #(bit_ns);
        end
    endtask

endmodule
