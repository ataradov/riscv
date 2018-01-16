/*
 * Copyright (c) 2017-2018, Alex Taradov <alex@taradov.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

`timescale 1ns / 1ps

module per_uart (
  input         clk_i,
  input         reset_i,

  input  [31:0] addr_i,
  input  [31:0] wdata_i,
  output [31:0] rdata_o,
  input   [1:0] size_i,
  input         rd_i,
  input         wr_i,

  input         uart_rx_i,
  output        uart_tx_o
);

// Note: this is not the most efficient UART implementation, but it is simple
// and works for the purpose of demonstration

//-----------------------------------------------------------------------------
localparam
  REG_CSR  = 8'h00,
  REG_BR   = 8'h04,
  REG_DATA = 8'h08;

localparam
  BIT_CSR_TX_READY  = 0,
  BIT_CSR_RX_READY  = 1;

//-----------------------------------------------------------------------------
wire reg_csr_w  = (REG_CSR  == addr_i[7:0]);
wire reg_br_w   = (REG_BR   == addr_i[7:0]);
wire reg_data_w = (REG_DATA == addr_i[7:0]);

//-----------------------------------------------------------------------------
reg [15:0] br_r;

always @(posedge clk_i) begin
  if (reset_i)
    br_r <= 16'h0;
  else if (wr_i && reg_br_w)
    br_r <= wdata_i[15:0];
end

//-----------------------------------------------------------------------------
`ifdef SIMULATOR
always @(posedge clk_i) begin
  if (wr_i && reg_data_w)
    $write("%c", wdata_i[7:0]);
end

assign uart_tx_o = 1'b1;

`else
//-----------------------------------------------------------------------------
reg [15:0] tx_br_cnt_r;
reg  [3:0] tx_bit_cnt_r;
reg  [9:0] tx_shifter_r;
reg        tx_ready_r;

always @(posedge clk_i) begin
  if (reset_i) begin
    tx_br_cnt_r  <= 16'd0;
    tx_bit_cnt_r <= 4'd0;
    tx_shifter_r <= 10'h1;
    tx_ready_r   <= 1'b1;
  end else if (tx_bit_cnt_r) begin
    if (tx_br_cnt_r == br_r) begin
      tx_shifter_r <= { 1'b1, tx_shifter_r[9:1] };
      tx_bit_cnt_r <= tx_bit_cnt_r - 4'd1;
      tx_br_cnt_r  <= 16'd0;
    end else begin
      tx_br_cnt_r  <= tx_br_cnt_r + 16'd1;
    end
  end else if (!tx_ready_r) begin
    tx_ready_r <= 1'b1;
  end else if (wr_i && reg_data_w) begin
    tx_shifter_r <= { 1'b1, wdata_i[7:0], 1'b0 };
    tx_bit_cnt_r <= 4'd10;
    tx_ready_r   <= 1'b0;
  end
end

assign uart_tx_o = tx_shifter_r[0];
`endif

//-----------------------------------------------------------------------------
reg [15:0] rx_br_cnt_r;
reg  [3:0] rx_bit_cnt_r;
reg  [8:0] rx_shifter_r;
reg  [7:0] rx_data_r;
reg        rx_done_r;

always @(posedge clk_i) begin
  if (reset_i) begin
    rx_br_cnt_r  <= 16'd1;
    rx_bit_cnt_r <= 4'd0;
    rx_shifter_r <= 10'h0;
    rx_done_r    <= 1'b0;
  end else if (rx_bit_cnt_r) begin
    if (rx_br_cnt_r == br_r) begin
      rx_shifter_r <= { uart_rx_i, rx_shifter_r[8:1] };
      rx_bit_cnt_r <= rx_bit_cnt_r - 4'd1;
      rx_br_cnt_r  <= 16'd0;
    end else begin
      rx_br_cnt_r  <= rx_br_cnt_r + 16'd1;
    end
  end else if (rx_done_r) begin
    rx_done_r <= 1'b0;
  end else if (rx_shifter_r[8]) begin
    rx_data_r    <= rx_shifter_r[7:0];
    rx_done_r    <= 1'b1;
    rx_shifter_r <= 10'h0;
  end else if (!uart_rx_i) begin
    if (rx_br_cnt_r)
      rx_br_cnt_r <= rx_br_cnt_r - 16'd1;
    else
      rx_bit_cnt_r <= 4'd9;
  end else begin
    rx_br_cnt_r <= { 1'b0, br_r[15:1] };
  end
end

//-----------------------------------------------------------------------------
reg rx_ready_r;

always @(posedge clk_i) begin
  if (reset_i)
    rx_ready_r <= 1'b0;
  else if (rx_done_r)
    rx_ready_r <= 1'b1;
  else if (rd_i && reg_data_w)
    rx_ready_r <= 1'b0;
end

//-----------------------------------------------------------------------------
reg [31:0] reg_data_r;

always @(posedge clk_i) begin
  if (reset_i)
    reg_data_r <= csr_w;
  else if (reg_data_w)
    reg_data_r <= { 24'h0, rx_data_r };
  else
    reg_data_r <= csr_w;
end

//-----------------------------------------------------------------------------
wire [31:0] csr_w;

`ifdef SIMULATOR
assign csr_w[BIT_CSR_TX_READY] = 1'b1;
assign csr_w[BIT_CSR_RX_READY] = 1'b0;
`else
assign csr_w[BIT_CSR_TX_READY] = tx_ready_r;
assign csr_w[BIT_CSR_RX_READY] = rx_ready_r;
`endif

assign csr_w[31:2] = 30'h0;

//-----------------------------------------------------------------------------
assign rdata_o = reg_data_r;

endmodule

