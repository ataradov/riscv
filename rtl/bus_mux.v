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

module bus_mux #(
  parameter     N = 2
)(
  input         clk_i,
  input         reset_i,

  input [N-1:0] ss_i,

  input  [31:0] m_addr_i,
  input  [31:0] m_wdata_i,
  output [31:0] m_rdata_o,
  input   [1:0] m_size_i,
  input         m_rd_i,
  input         m_wr_i,

  output [32*N-1:0] s_addr_o,
  output [32*N-1:0] s_wdata_o,
  input  [32*N-1:0] s_rdata_i,
  output  [2*N-1:0] s_size_o,
  output    [N-1:0] s_rd_o,
  output    [N-1:0] s_wr_o
);

//-----------------------------------------------------------------------------
assign s_addr_o  = {N{m_addr_i}};
assign s_wdata_o = {N{m_wdata_i}};
assign s_size_o  = {N{m_size_i}};
assign s_rd_o    = {N{m_rd_i}} & ss_i;
assign s_wr_o    = {N{m_wr_i}} & ss_i;

genvar i;
wire [31:0] rdata_w [N-1:0];
generate for (i = 0; i < N; i = i + 1) begin: rdata_mux
  wire [31:0] part_w = {32{ss_r[i]}} & s_rdata_i[32*i+:32];
  assign rdata_w[i] = (0 == i) ? part_w : rdata_w[i-1] | part_w;
end
endgenerate

assign m_rdata_o = rdata_w[N - 1];

//-----------------------------------------------------------------------------
reg [N-1:0] ss_r;

always @(posedge clk_i) begin
  if (reset_i)
    ss_r <= 'h0;
  else
    ss_r <= ss_i;
end

endmodule


