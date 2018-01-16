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

module per_timer (
  input         clk_i,
  input         reset_i,

  input  [31:0] addr_i,
  input  [31:0] wdata_i,
  output [31:0] rdata_o,
  input   [1:0] size_i,
  input         rd_i,
  input         wr_i
);

//-----------------------------------------------------------------------------
localparam
  REG_CSR     = 8'h00,
  REG_COUNT   = 8'h04,
  REG_COMPARE = 8'h08;

localparam
  BIT_CSR_ENABLE   = 0,
  BIT_CSR_DISABLE  = 1,
  BIT_CSR_OVERFLOW = 2;

//-----------------------------------------------------------------------------
wire reg_csr_w     = (REG_CSR     == addr_i[7:0]);
wire reg_count_w   = (REG_COUNT   == addr_i[7:0]);
wire reg_compare_w = (REG_COMPARE == addr_i[7:0]);

//-----------------------------------------------------------------------------
reg timer_enabled_r;

always @(posedge clk_i) begin
  if (reset_i)
    timer_enabled_r <= 1'b0;
  else if (wr_i && reg_csr_w && wdata_i[BIT_CSR_ENABLE])
    timer_enabled_r <= 1'b1;
  else if (wr_i && reg_csr_w && wdata_i[BIT_CSR_DISABLE])
    timer_enabled_r <= 1'b0;
end

//-----------------------------------------------------------------------------
reg timer_overflow_r;

always @(posedge clk_i) begin
  if (reset_i)
    timer_overflow_r <= 1'b0;
  else if (wr_i && reg_csr_w && wdata_i[BIT_CSR_OVERFLOW])
    timer_overflow_r <= 1'b0;
  else if (timer_overflow_w)
    timer_overflow_r <= 1'b1;
end

wire timer_overflow_w = timer_enabled_r && (timer_count_r == timer_compare_r);

//-----------------------------------------------------------------------------
reg [31:0] timer_count_r;

always @(posedge clk_i) begin
  if (reset_i)
    timer_count_r <= 32'h0;
  else if (wr_i && reg_count_w)
    timer_count_r <= wdata_i;
  else if (timer_overflow_w)
    timer_count_r <= 32'h0;
  else if (timer_enabled_r)
    timer_count_r <= timer_count_r + 32'd1;
end

//-----------------------------------------------------------------------------
reg [31:0] timer_compare_r;

always @(posedge clk_i) begin
  if (reset_i)
    timer_compare_r <= 32'h0;
  else if (wr_i && reg_compare_w)
    timer_compare_r <= wdata_i;
end

//-----------------------------------------------------------------------------
reg [31:0] reg_data_r;

always @(posedge clk_i) begin
  reg_data_r <= csr_w;
end

//-----------------------------------------------------------------------------
wire [31:0] csr_w;
assign csr_w[BIT_CSR_ENABLE]   = timer_enabled_r;
assign csr_w[BIT_CSR_DISABLE]  = !timer_enabled_r;
assign csr_w[BIT_CSR_OVERFLOW] = timer_overflow_r;
assign csr_w[31:3]             = 29'h0;

//-----------------------------------------------------------------------------
assign rdata_o = reg_data_r;

endmodule

