// -----------------------------------------------------------------------------
// uart_demo_top.v  --  FPGA demo: talk to a PC over the board's USB-UART
//
// Mode is selected with sw[15]:
//
//   sw[15] = 0  ECHO mode    every byte received is sent straight back
//                            (led[7:0] shows the last byte received)
//
//   sw[15] = 1  COMMAND mode tiny binary register protocol
//                 'W' <addr> <data>  -> 'K'          write register
//                 'R' <addr>         -> <data>       read register
//                 CR / LF / space    -> (ignored)
//                 anything else      -> '?'
//
// Register map (COMMAND mode)
//   0x00  ID        R   0x55 ('U')
//   0x01  VERSION   R   0x10 (v1.0)
//   0x02  LED       RW  drives led[7:0] in command mode
//   0x03  SW_LO     R   sw[7:0]
//   0x04  SW_HI     R   sw[15:8]
//   0x05  SCRATCH   RW
//   0x06  RXCNT_LO  R   bytes received (16-bit counter, low)
//   0x07  RXCNT_HI  R   bytes received (high)
//   0x08  STATUS    R   {5'b0, overrun, framing_err, parity_err}  (sticky)
//                   W   any write clears the sticky flags
//
// Status LEDs
//   led[8]  RX activity      led[9]  TX activity
//   led[10] parity error     led[11] framing error   led[12] RX overrun
//   led[14] heartbeat        led[15] command mode
// -----------------------------------------------------------------------------
`timescale 1ns/1ps
`default_nettype none

module uart_demo_top #(
    parameter integer CLK_FREQ  = 100_000_000,
    parameter integer BAUD_RATE = 115_200,
    parameter integer ACT_BITS  = 22,      // activity LED stretch: 2^22 clk ~ 42 ms
    parameter integer HB_BITS   = 26       // heartbeat: blinks at f_clk / 2^26 (~1.5 Hz @100 MHz)
) (
    input  wire        clk,
    input  wire        btn_rst,            // active-high push button
    input  wire        uart_rxd,           // from PC
    output wire        uart_txd,           // to PC
    input  wire [15:0] sw,
    output wire [15:0] led
);

    // ---------------------------------------------------------------- reset
    // initial values give a power-on reset on FPGAs (bitstream init)
    /* verilator lint_off PROCASSINIT */
    reg [1:0] rst_sync = 2'b11;
    reg [3:0] por_cnt  = 4'd0;
    /* verilator lint_on PROCASSINIT */
    always @(posedge clk) begin
        rst_sync <= {rst_sync[0], btn_rst};
        if (!por_cnt[3]) por_cnt <= por_cnt + 4'd1;
    end
    wire rst = rst_sync[1] || !por_cnt[3];

    // ------------------------------------------------------------- switches
    reg [15:0] sw_m, sw_s;
    always @(posedge clk) begin
        sw_m <= sw;
        sw_s <= sw_m;
    end
    wire cmd_mode = sw_s[15];

    // ----------------------------------------------------------------- UART
    wire [7:0] rx_data;
    wire       rx_perr, rx_ferr, rx_brk, rx_valid, rx_ready;
    wire [7:0] tx_data;
    wire       tx_valid, tx_ready;
    wire       rx_overrun;
    wire       clr_status;

    uart #(
        .CLK_FREQ  (CLK_FREQ),
        .BAUD_RATE (BAUD_RATE),
        .DATA_BITS (8),
        .PARITY    (0),
        .STOP_BITS (1),
        .FIFO_DEPTH(32)
    ) u_uart (
        .clk          (clk),
        .rst          (rst),
        .s_tx_data    (tx_data),
        .s_tx_valid   (tx_valid),
        .s_tx_ready   (tx_ready),
        .m_rx_data    (rx_data),
        .m_rx_perr    (rx_perr),
        .m_rx_ferr    (rx_ferr),
        .m_rx_break   (rx_brk),
        .m_rx_valid   (rx_valid),
        .m_rx_ready   (rx_ready),
        .txd          (uart_txd),
        .rxd          (uart_rxd),
        /* verilator lint_off PINCONNECTEMPTY */
        .tx_busy      (),
        .tx_fifo_count(),
        .rx_fifo_count(),
        /* verilator lint_on PINCONNECTEMPTY */
        .rx_overrun   (rx_overrun),
        .clr_overrun  (clr_status)
    );

    // ---------------------------------------------------- command processor
    localparam [1:0] C_CMD  = 2'd0,
                     C_ADDR = 2'd1,
                     C_DATA = 2'd2,
                     C_SEND = 2'd3;

    reg [1:0]  cstate;
    reg [7:0]  cmd, addr, resp;
    reg [7:0]  led_reg, scratch, last_rx;
    reg [15:0] rx_count;
    reg        st_perr, st_ferr, st_ovr;
    reg        clr_req;
    reg [7:0]  rd_val;

    // Echo mode is a pure combinational pass-through between the two streams.
    assign rx_ready = cmd_mode ? (cstate != C_SEND) : tx_ready;
    assign tx_valid = cmd_mode ? (cstate == C_SEND) : rx_valid;
    assign tx_data  = cmd_mode ? resp               : rx_data;

    wire rx_fire = rx_valid && rx_ready;
    wire rx_bad  = rx_perr || rx_ferr || rx_brk;

    assign clr_status = clr_req;

    always @* begin
        case (rx_data)                  // address byte currently on the stream
            8'h00:   rd_val = 8'h55;
            8'h01:   rd_val = 8'h10;
            8'h02:   rd_val = led_reg;
            8'h03:   rd_val = sw_s[7:0];
            8'h04:   rd_val = sw_s[15:8];
            8'h05:   rd_val = scratch;
            8'h06:   rd_val = rx_count[7:0];
            8'h07:   rd_val = rx_count[15:8];
            8'h08:   rd_val = {5'b0, st_ovr, st_ferr, st_perr};
            default: rd_val = 8'h00;
        endcase
    end

    always @(posedge clk) begin
        clr_req <= 1'b0;
        if (rst) begin
            cstate   <= C_CMD;
            cmd      <= 8'h00;
            addr     <= 8'h00;
            resp     <= 8'h00;
            led_reg  <= 8'h00;
            scratch  <= 8'h00;
            last_rx  <= 8'h00;
            rx_count <= 16'd0;
            st_perr  <= 1'b0;
            st_ferr  <= 1'b0;
            st_ovr   <= 1'b0;
        end else begin
            // ---- statistics / sticky status (both modes)
            if (rx_fire) begin
                last_rx  <= rx_data;
                rx_count <= rx_count + 16'd1;
                if (rx_perr)          st_perr <= 1'b1;
                if (rx_ferr | rx_brk) st_ferr <= 1'b1;
            end
            if (rx_overrun) st_ovr <= 1'b1;

            // ---- protocol
            if (!cmd_mode) begin
                cstate <= C_CMD;
            end else begin
                case (cstate)
                    C_CMD: if (rx_fire && !rx_bad) begin
                        cmd <= rx_data;
                        case (rx_data)
                            "W", "R":           cstate <= C_ADDR;
                            8'h0D, 8'h0A, " ":  cstate <= C_CMD;
                            default: begin
                                resp   <= "?";
                                cstate <= C_SEND;
                            end
                        endcase
                    end

                    C_ADDR: if (rx_fire) begin
                        if (rx_bad) begin
                            cstate <= C_CMD;
                        end else if (cmd == "R") begin
                            resp   <= rd_val;
                            cstate <= C_SEND;
                        end else begin
                            addr   <= rx_data;
                            cstate <= C_DATA;
                        end
                    end

                    C_DATA: if (rx_fire) begin
                        if (rx_bad) begin
                            cstate <= C_CMD;
                        end else begin
                            case (addr)
                                8'h02: led_reg <= rx_data;
                                8'h05: scratch <= rx_data;
                                8'h08: begin
                                    st_perr <= 1'b0;
                                    st_ferr <= 1'b0;
                                    st_ovr  <= 1'b0;
                                    clr_req <= 1'b1;
                                end
                                default: ;
                            endcase
                            resp   <= "K";
                            cstate <= C_SEND;
                        end
                    end

                    C_SEND: if (tx_ready) cstate <= C_CMD;
                endcase
            end
        end
    end

    // ----------------------------------------------------------------- LEDs
    reg [ACT_BITS-1:0] rx_act, tx_act;
    reg [HB_BITS-1:0]  hb;
    always @(posedge clk) begin
        if (rst) begin
            rx_act <= {ACT_BITS{1'b0}};
            tx_act <= {ACT_BITS{1'b0}};
            hb     <= {HB_BITS{1'b0}};
        end else begin
            hb     <= hb + 1'b1;
            rx_act <= rx_fire                 ? {ACT_BITS{1'b1}} :
                      (|rx_act)               ? rx_act - 1'b1    : rx_act;
            tx_act <= (tx_valid && tx_ready)  ? {ACT_BITS{1'b1}} :
                      (|tx_act)               ? tx_act - 1'b1    : tx_act;
        end
    end

    assign led[7:0] = cmd_mode ? led_reg : last_rx;
    assign led[8]   = |rx_act;
    assign led[9]   = |tx_act;
    assign led[10]  = st_perr;
    assign led[11]  = st_ferr;
    assign led[12]  = st_ovr;
    assign led[13]  = 1'b0;
    assign led[14]  = hb[HB_BITS-1];
    assign led[15]  = cmd_mode;

endmodule

`default_nettype wire
