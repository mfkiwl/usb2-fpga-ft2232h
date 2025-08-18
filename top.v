`timescale 1 ns / 1 ps

module top #(
    parameter TURN_CYCLES = 12,
    parameter EP_SIZE = 512,
    parameter FIFO_DEPTH_WORDS = 8192
) (
    inout [7:0] data,
    input rx_empty,
    input tx_full,
    output reg read_n,
    output reg write_n,
    output reg send_immediately_n,
    input clock_60mhz,
    output reg output_enable_n,
    output reg power_led_n,
    output reg tx_active_led_n,
    input CLK,
    input vp_in,
    input vn_in,
    input vauxp0,
    input vauxn0,
    input [3:0] sw,
    output reg [7:0] LED
);

    function automatic [7:0] crc8_next;
        input [7:0] crc_in;
        input [7:0] data;
        integer k;
        reg [7:0] crc;
        begin
            crc = crc_in ^ data;
            for (k = 0; k < 8; k = k + 1)
                crc = crc[7] ? ((crc << 1) ^ 8'h07) : (crc << 1);
            crc8_next = crc;
        end
    endfunction

    localparam COUNT_W = $clog2(FIFO_DEPTH_WORDS + 1);
    localparam integer HIGH_WM = (FIFO_DEPTH_WORDS * 85) / 100;
    localparam integer LOW_WM = (FIFO_DEPTH_WORDS * 75) / 100;

    wire [71:0] fifo72_dout;
    wire fifo72_empty, fifo72_full;
    reg fifo72_rd_en = 1'b0, fifo72_wr_en = 1'b0;
    reg [71:0] fifo72_din;
    wire [COUNT_W-1:0] wr_data_count;

    fifo_generator_0 loop_fifo (
        .clk(clock_60mhz),
        .wr_en(fifo72_wr_en),
        .din(fifo72_din),
        .rd_en(fifo72_rd_en),
        .dout(fifo72_dout),
        .full(fifo72_full),
        .empty(fifo72_empty),
        .data_count(wr_data_count)
    );

    reg pause_rx = 1'b0;
    always @(posedge clock_60mhz) begin
        if (pause_rx)
            pause_rx <= (wr_data_count > LOW_WM);
        else
            pause_rx <= (wr_data_count > HIGH_WM);
    end

    reg [71:0] tx_word_reg = 72'h0;
    reg [3:0] tx_byte_ptr = 4'd0;
    reg tx_word_valid = 1'b0;
    reg fifo_rd_en = 1'b0;
    wire fifo_empty = (~tx_word_valid) && fifo72_empty;
    wire fifo_full = fifo72_full;
    reg read_requested = 1'b0;

    always @(posedge clock_60mhz) begin
        fifo72_rd_en <= 1'b0;
        if (read_requested) begin
            tx_word_reg <= fifo72_dout;
            tx_word_valid <= 1'b1;
            tx_byte_ptr <= 0;
            read_requested <= 1'b0;
        end else if (!tx_word_valid && !fifo72_empty) begin
            fifo72_rd_en <= 1'b1;
            read_requested <= 1'b1;
        end
        if (fifo_rd_en && tx_word_valid) begin
            tx_byte_ptr <= tx_byte_ptr + 1'b1;
            if (tx_byte_ptr == 4'd8)
                tx_word_valid <= 1'b0;
        end
    end

    assign fifo_dout = (tx_byte_ptr == 4'd0) ? tx_word_reg[7:0] :
        (tx_byte_ptr == 4'd1) ? tx_word_reg[15:8] :
        (tx_byte_ptr == 4'd2) ? tx_word_reg[23:16] :
        (tx_byte_ptr == 4'd3) ? tx_word_reg[31:24] :
        (tx_byte_ptr == 4'd4) ? tx_word_reg[39:32] :
        (tx_byte_ptr == 4'd5) ? tx_word_reg[47:40] :
        (tx_byte_ptr == 4'd6) ? tx_word_reg[55:48] :
        (tx_byte_ptr == 4'd7) ? tx_word_reg[63:56] :
        tx_word_reg[71:64];

    reg [1:0] rxf_s = 2'b11, txe_s = 2'b11;
    always @(posedge clock_60mhz) begin
        rxf_s <= {rxf_s[0], rx_empty};
        txe_s <= {txe_s[0], tx_full};
    end
    wire rxf_n = rxf_s[1];
    wire txe_n = txe_s[1];

    localparam FLAG1 = 8'h55, FLAG2 = 8'hAA, FLAG3 = 8'h33, FLAG4 = 8'hCC;
    localparam [5:0] S_IDLE = 6'd0,
        S_G1 = 6'd1,
        S_G2 = 6'd2,
        S_F2_WR = 6'd3,
        S_F2_H = 6'd4,
        S_LEN_WR = 6'd5,
        S_LEN_H = 6'd6,
        S_RD0 = 6'd7,
        S_B0_WR = 6'd8,
        S_B0_H = 6'd9,
        S_RD1 = 6'd10,
        S_B1_WR = 6'd11,
        S_B1_H = 6'd12,
        S_RD2 = 6'd13,
        S_B2_WR = 6'd14,
        S_B2_H = 6'd15,
        S_RD3 = 6'd16,
        S_B3_WR = 6'd17,
        S_B3_H = 6'd18,
        S_RD4 = 6'd19,
        S_B4_WR = 6'd20,
        S_B4_H = 6'd21,
        S_RD5 = 6'd22,
        S_B5_WR = 6'd23,
        S_B5_H = 6'd24,
        S_RD6 = 6'd25,
        S_B6_WR = 6'd26,
        S_B6_H = 6'd27,
        S_RD7 = 6'd28,
        S_B7_WR = 6'd29,
        S_B7_H = 6'd30,
        S_RD8 = 6'd31,
        S_B8_WR = 6'd32,
        S_B8_H = 6'd33,
        S_CRC_WR = 6'd34,
        S_CRC_H = 6'd35,
        S_F3_WR = 6'd36,
        S_F3_H = 6'd37,
        S_F4_WR = 6'd38,
        S_F4_H = 6'd39,
        S_TURN = 6'd40;

    localparam TURN_W = $clog2(TURN_CYCLES + 1);

    reg [5:0] tx_state = S_IDLE;
    reg [7:0] tx_crc = 8'h00;
    reg [TURN_W-1:0] tx_turn_ctr = 0;
    wire tx_busy = (tx_state != S_IDLE) | (tx_turn_ctr != 0);
    wire [7:0] tx_len = 8'd9;
    reg [7:0] tx_byte_int;
    reg write_n_int;

    always @(posedge clock_60mhz) begin
        fifo_rd_en <= 1'b0;
        write_n_int <= 1'b1;
        send_immediately_n <= 1'b1;
        tx_active_led_n <= 1'b1;
        case (tx_state)
            S_IDLE:
                if (!fifo_empty && !txe_n && !tx_busy && rxf_n)
                    tx_state <= S_G1;
            S_G1: begin
                tx_byte_int <= FLAG1;
                tx_state <= S_G2;
            end
            S_G2: begin
                write_n_int <= 1'b0;
                tx_state <= S_F2_WR;
            end
            S_F2_WR:
                tx_state <= S_F2_H;
            S_F2_H:
                if (!txe_n) begin
                    tx_byte_int <= FLAG2;
                    write_n_int <= 1'b0;
                    tx_state <= S_LEN_WR;
                end
            S_LEN_WR:
                tx_state <= S_LEN_H;
            S_LEN_H:
                if (!txe_n) begin
                    tx_byte_int <= tx_len;
                    write_n_int <= 1'b0;
                    tx_crc <= 8'h00;
                    tx_state <= S_RD0;
                end
            S_RD0:
                if (!txe_n) begin
                    tx_byte_int <= fifo_dout;
                    tx_crc <= crc8_next(tx_crc, fifo_dout);
                    write_n_int <= 1'b0;
                    fifo_rd_en <= 1'b1;
                    tx_state <= S_B0_WR;
                end
            S_B0_WR:
                tx_state <= S_B0_H;
            S_B0_H:
                tx_state <= S_RD1;
            S_RD1:
                if (!txe_n) begin
                    tx_byte_int <= fifo_dout;
                    tx_crc <= crc8_next(tx_crc, fifo_dout);
                    write_n_int <= 1'b0;
                    fifo_rd_en <= 1'b1;
                    tx_state <= S_B1_WR;
                end
            S_B1_WR:
                tx_state <= S_B1_H;
            S_B1_H:
                tx_state <= S_RD2;
            S_RD2:
                if (!txe_n) begin
                    tx_byte_int <= fifo_dout;
                    tx_crc <= crc8_next(tx_crc, fifo_dout);
                    write_n_int <= 1'b0;
                    fifo_rd_en <= 1'b1;
                    tx_state <= S_B2_WR;
                end
            S_B2_WR:
                tx_state <= S_B2_H;
            S_B2_H:
                tx_state <= S_RD3;
            S_RD3:
                if (!txe_n) begin
                    tx_byte_int <= fifo_dout;
                    tx_crc <= crc8_next(tx_crc, fifo_dout);
                    write_n_int <= 1'b0;
                    fifo_rd_en <= 1'b1;
                    tx_state <= S_B3_WR;
                end
            S_B3_WR:
                tx_state <= S_B3_H;
            S_B3_H:
                tx_state <= S_RD4;
            S_RD4:
                if (!txe_n) begin
                    tx_byte_int <= fifo_dout;
                    tx_crc <= crc8_next(tx_crc, fifo_dout);
                    write_n_int <= 1'b0;
                    fifo_rd_en <= 1'b1;
                    tx_state <= S_B4_WR;
                end
            S_B4_WR:
                tx_state <= S_B4_H;
            S_B4_H:
                tx_state <= S_RD5;
            S_RD5:
                if (!txe_n) begin
                    tx_byte_int <= fifo_dout;
                    tx_crc <= crc8_next(tx_crc, fifo_dout);
                    write_n_int <= 1'b0;
                    fifo_rd_en <= 1'b1;
                    tx_state <= S_B5_WR;
                end
            S_B5_WR:
                tx_state <= S_B5_H;
            S_B5_H:
                tx_state <= S_RD6;
            S_RD6:
                if (!txe_n) begin
                    tx_byte_int <= fifo_dout;
                    tx_crc <= crc8_next(tx_crc, fifo_dout);
                    write_n_int <= 1'b0;
                    fifo_rd_en <= 1'b1;
                    tx_state <= S_B6_WR;
                end
            S_B6_WR:
                tx_state <= S_B6_H;
            S_B6_H:
                tx_state <= S_RD7;
            S_RD7:
                if (!txe_n) begin
                    tx_byte_int <= fifo_dout;
                    tx_crc <= crc8_next(tx_crc, fifo_dout);
                    write_n_int <= 1'b0;
                    fifo_rd_en <= 1'b1;
                    tx_state <= S_B7_WR;
                end
            S_B7_WR:
                tx_state <= S_B7_H;
            S_B7_H:
                tx_state <= S_RD8;
            S_RD8:
                if (!txe_n) begin
                    tx_byte_int <= fifo_dout;
                    tx_crc <= crc8_next(tx_crc, fifo_dout);
                    write_n_int <= 1'b0;
                    fifo_rd_en <= 1'b1;
                    tx_state <= S_B8_WR;
                end
            S_B8_WR:
                tx_state <= S_B8_H;
            S_B8_H: begin
                tx_byte_int <= tx_crc;
                tx_state <= S_CRC_WR;
            end
            S_CRC_WR: begin
                write_n_int <= 1'b0;
                tx_state <= S_CRC_H;
            end
            S_CRC_H:
                if (!txe_n) begin
                    tx_byte_int <= FLAG3;
                    write_n_int <= 1'b0;
                    tx_state <= S_F3_WR;
                end
            S_F3_WR:
                tx_state <= S_F3_H;
            S_F3_H:
                if (!txe_n) begin
                    tx_byte_int <= FLAG4;
                    write_n_int <= 1'b0;
                    tx_state <= S_F4_WR;
                end
            S_F4_WR:
                tx_state <= S_F4_H;
            S_F4_H: begin
                tx_turn_ctr <= TURN_CYCLES;
                tx_state <= S_TURN;
            end
            S_TURN:
                if (tx_turn_ctr != 0)
                    tx_turn_ctr <= tx_turn_ctr - 1'b1;
                else
                    tx_state <= S_IDLE;
        endcase
    end

    reg [7:0] tx_byte_reg;
    always @(negedge clock_60mhz) begin
        tx_byte_reg <= tx_byte_int;
        write_n <= write_n_int;
    end
    assign data = (~write_n) ? tx_byte_reg : 8'hZZ;

    localparam R_IDLE = 3'd0, R_G1 = 3'd1, R_G2 = 3'd2, R_OE = 3'd3, R_RD = 3'd4;
    reg [2:0] read_FSM = R_IDLE;
    reg [TURN_W-1:0] rx_turn_ctr = 0;
    wire rx_busy = (read_FSM != R_IDLE) | (rx_turn_ctr != 0);

    always @(negedge clock_60mhz) begin
        output_enable_n <= 1'b1;
        read_n <= 1'b1;
        if (rx_turn_ctr != 0)
            rx_turn_ctr <= rx_turn_ctr - 1'b1;
        if (write_n == 1'b0) begin
            read_FSM <= R_IDLE;
            rx_turn_ctr <= 0;
        end else begin
            case (read_FSM)
                R_IDLE:
                    if (~rxf_n && ~pause_rx && !tx_busy)
                        read_FSM <= R_G1;
                R_G1:
                    read_FSM <= R_G2;
                R_G2: begin
                    output_enable_n <= 1'b0;
                    read_FSM <= R_OE;
                end
                R_OE:
                    if (~rxf_n && ~pause_rx) begin
                        output_enable_n <= 1'b0;
                        read_n <= 1'b0;
                        read_FSM <= R_RD;
                    end else
                        read_FSM <= R_IDLE;
                R_RD: begin
                    output_enable_n <= 1'b0;
                    read_n <= 1'b0;
                    if (rxf_n || pause_rx) begin
                        output_enable_n <= 1'b1;
                        read_n <= 1'b1;
                        read_FSM <= R_IDLE;
                        rx_turn_ctr <= TURN_CYCLES;
                    end
                end
            endcase
        end
    end

    localparam EP_W = $clog2(EP_SIZE);
    reg [EP_W-1:0] usb_pos = 0;
    reg [1:0] hdr_skip = 0;
    reg skip_en = 0;
    localparam RX_IDLE = 4'd0, RX_F2 = 4'd1, RX_LEN = 4'd2, RX_DATA = 4'd3, RX_CRC = 4'd4, RX_F3 = 4'd5, RX_F4 = 4'd6, RX_PUSH = 4'd7;
    reg [3:0] rx_state = RX_IDLE;
    reg [7:0] rx_crc, payload[0:8];
    reg [71:0] rx_word;
    reg [7:0] rx_len, rx_idx;

    always @(posedge clock_60mhz) begin
        fifo72_wr_en <= 1'b0;
        if (!tx_busy && (read_FSM == R_RD) && (read_n == 1'b0)) begin
            if (usb_pos == 0)
                hdr_skip <= 2'd2;
            skip_en <= (hdr_skip != 0);
            if (hdr_skip != 0)
                hdr_skip <= hdr_skip - 1'b1;
            usb_pos <= (usb_pos == EP_SIZE - 1) ? 0 : usb_pos + 1'b1;
            if (skip_en)
                disable skip;
            if (!skip_en) begin
                case (rx_state)
                    RX_IDLE:
                        if (data == FLAG1)
                            rx_state <= RX_F2;
                    RX_F2:
                        if (data == FLAG2)
                            rx_state <= RX_LEN;
                        else if (data != FLAG1)
                            rx_state <= RX_IDLE;
                    RX_LEN: begin
                        rx_len <= data;
                        rx_idx <= 0;
                        rx_crc <= 8'h00;
                        rx_state <= (data == 8'd0) ? RX_CRC : RX_DATA;
                    end
                    RX_DATA: begin
                        rx_crc <= crc8_next(rx_crc, data);
                        if (rx_idx < 9)
                            payload[rx_idx] <= data;
                        rx_idx <= rx_idx + 1;
                        rx_len <= rx_len - 1;
                        rx_state <= (rx_len == 8'd1) ? RX_CRC : RX_DATA;
                    end
                    RX_CRC:
                        if (data == rx_crc)
                            rx_state <= RX_F3;
                        else
                            rx_state <= RX_IDLE;
                    RX_F3:
                        if (data == FLAG3)
                            rx_state <= RX_F4;
                        else if (data == FLAG1)
                            rx_state <= RX_F2;
                        else
                            rx_state <= RX_IDLE;
                    RX_F4:
                        if (data == FLAG4) begin
                            rx_word <= {payload[8], payload[7], payload[6], payload[5], payload[4], payload[3], payload[2], payload[1], payload[0]};
                            rx_state <= RX_PUSH;
                        end else
                            rx_state <= RX_IDLE;
                    RX_PUSH:
                        if (!fifo72_full) begin
                            fifo72_din <= rx_word;
                            fifo72_wr_en <= 1'b1;
                            rx_state <= RX_IDLE;
                        end
                endcase
            end
        end
    end

    wire xadc_ready;
    wire [15:0] xadc_do;
    xadc_wiz_0 x1 (
        .dclk_in(CLK),
        .reset_in(1'b0),
        .daddr_in(8'h03),
        .den_in(1'b1),
        .dwe_in(1'b0),
        .di_in(16'h0),
        .do_out(xadc_do),
        .drdy_out(xadc_ready),
        .vp_in(vp_in),
        .vn_in(vn_in),
        .vauxp0(vauxp0),
        .vauxn0(vauxn0)
    );

    always @(posedge CLK) begin
        power_led_n <= 1'b0;
        if (xadc_ready) begin
            case (xadc_do[15:13])
                3'd1: LED <= 8'b0000_0011;
                3'd2: LED <= 8'b0000_0111;
                3'd3: LED <= 8'b0000_1111;
                3'd4: LED <= 8'b0001_1111;
                3'd5: LED <= 8'b0011_1111;
                3'd6: LED <= 8'b0111_1111;
                3'd7: LED <= 8'b1111_1111;
                default: LED <= 8'b0000_0000;
            endcase
        end
    end

endmodule