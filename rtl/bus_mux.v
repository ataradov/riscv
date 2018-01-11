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

module bus_mux (
  input         clk_i,
  input         reset_i,

  input  [31:0] m_addr_i,
  input  [31:0] m_wdata_i,
  output [31:0] m_rdata_o,
  input   [1:0] m_size_i,
  input         m_rd_i,
  input         m_wr_i,

  output [15:0] s0_addr_o,
  output [31:0] s0_wdata_o,
  input  [31:0] s0_rdata_i,
  output  [1:0] s0_size_o,
  output        s0_rd_o,
  output        s0_wr_o,

  output [15:0] s1_addr_o,
  output [31:0] s1_wdata_o,
  input  [31:0] s1_rdata_i,
  output  [1:0] s1_size_o,
  output        s1_rd_o,
  output        s1_wr_o,

  output [15:0] s2_addr_o,
  output [31:0] s2_wdata_o,
  input  [31:0] s2_rdata_i,
  output  [1:0] s2_size_o,
  output        s2_rd_o,
  output        s2_wr_o,

  output [15:0] s3_addr_o,
  output [31:0] s3_wdata_o,
  input  [31:0] s3_rdata_i,
  output  [1:0] s3_size_o,
  output        s3_rd_o,
  output        s3_wr_o,

  input         dummy_i
);

//-----------------------------------------------------------------------------
wire [15:0] ss_w     = m_addr_i[31:16];
wire        ss_def_w = (16'h0000 == ss_w);

assign s0_addr_o = m_addr_i[15:0];
assign s1_addr_o = m_addr_i[15:0];
assign s2_addr_o = m_addr_i[15:0];
assign s3_addr_o = m_addr_i[15:0];

assign s0_wdata_o = m_wdata_i;
assign s1_wdata_o = m_wdata_i;
assign s2_wdata_o = m_wdata_i;
assign s3_wdata_o = m_wdata_i;

assign s0_size_o = m_size_i;
assign s1_size_o = m_size_i;
assign s2_size_o = m_size_i;
assign s3_size_o = m_size_i;

assign s0_rd_o = m_rd_i & ss_def_w;
assign s1_rd_o = m_rd_i & ss_w[0];
assign s2_rd_o = m_rd_i & ss_w[1];
assign s3_rd_o = m_rd_i & ss_w[2];

assign s0_wr_o = m_wr_i & ss_def_w;
assign s1_wr_o = m_wr_i & ss_w[0];
assign s2_wr_o = m_wr_i & ss_w[1];
assign s3_wr_o = m_wr_i & ss_w[2];

assign m_rdata_o = 
  ss_r[0] ? s1_rdata_i :
  ss_r[1] ? s2_rdata_i :
  ss_r[2] ? s3_rdata_i : s0_rdata_i;

//-----------------------------------------------------------------------------
reg [15:0] ss_r;

always @(posedge clk_i) begin
  if (reset_i)
    ss_r <= 16'h0000;
  else
    ss_r <= ss_w;
end

endmodule


