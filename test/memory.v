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

module memory #(
  parameter     SIZE     = 8192,
  parameter     FIRMWARE = ""
)(
  input         clk_i,
  input         reset_i,

  input  [31:0] iaddr_i,
  output [31:0] irdata_o,
  input         ird_i,

  input  [31:0] daddr_i,
  input  [31:0] dwdata_i,
  output [31:0] drdata_o,
  input   [1:0] dsize_i,
  input         drd_i,
  input         dwr_i
);

//-----------------------------------------------------------------------------
localparam
  SIZE_BYTE = 2'd0,
  SIZE_HALF = 2'd1,
  SIZE_WORD = 2'd2;

localparam
  DEPTH = $clog2(SIZE);

//-----------------------------------------------------------------------------
wire [31:0] dwdata_w =
    (SIZE_BYTE == dsize_i) ? {4{dwdata_i[7:0]}} :
    (SIZE_HALF == dsize_i) ? {2{dwdata_i[15:0]}} : dwdata_i;

wire [3:0] dbe_byte_w = 
    (2'b00 == daddr_i[1:0]) ? 4'b0001 :
    (2'b01 == daddr_i[1:0]) ? 4'b0010 :
    (2'b10 == daddr_i[1:0]) ? 4'b0100 : 4'b1000;

wire [3:0] dbe_half_w =
    daddr_i[1] ? 4'b1100 : 4'b0011;

wire [3:0] dbe_w =
    (SIZE_BYTE == dsize_i) ? dbe_byte_w :
    (SIZE_HALF == dsize_i) ? dbe_half_w : 4'b1111;

wire [7:0] rdata_byte_w =
    (2'b00 == daddr_r) ? drdata_r[7:0] :
    (2'b01 == daddr_r) ? drdata_r[15:8] :
    (2'b10 == daddr_r) ? drdata_r[23:16] : drdata_r[31:24];

wire [15:0] rdata_half_w =
    daddr_r[1] ? drdata_r[31:16] : drdata_r[15:0];

assign drdata_o =
    (SIZE_BYTE == dsize_r) ? { 24'b0, rdata_byte_w } :
    (SIZE_HALF == dsize_r) ? { 16'b0, rdata_half_w } : drdata_r;

//-----------------------------------------------------------------------------
reg [1:0] daddr_r;
reg [1:0] dsize_r;

always @(posedge clk_i) begin
  if (reset_i) begin
    daddr_r <= 2'b00;
    dsize_r <= SIZE_BYTE;
  end else begin
    daddr_r <= daddr_i[1:0];
    dsize_r <= dsize_i;
  end
end

//-----------------------------------------------------------------------------
reg [31:0] mem_r [0:SIZE/4-1];

initial begin
  $readmemh(FIRMWARE, mem_r);
end

//-----------------------------------------------------------------------------
reg [31:0] irdata_r;

always @(posedge clk_i) begin
  if (reset_i)
    irdata_r <= 32'h0;
  else if (ird_i)
    irdata_r <= mem_r[iaddr_i[DEPTH:2]];
end

//-----------------------------------------------------------------------------
reg [31:0] drdata_r;

always @(posedge clk_i) begin
  if (reset_i)
    drdata_r <= 32'h0;
  else if (drd_i)
    drdata_r <= mem_r[daddr_i[DEPTH:2]];
end

always @(posedge clk_i) begin
  if (dbe_w[0] && dwr_i)
    mem_r[daddr_i[DEPTH:2]][7:0] <= dwdata_w[7:0];

  if (dbe_w[1] && dwr_i)
    mem_r[daddr_i[DEPTH:2]][15:8] <= dwdata_w[15:8];

  if (dbe_w[2] && dwr_i)
    mem_r[daddr_i[DEPTH:2]][23:16] <= dwdata_w[23:16];

  if (dbe_w[3] && dwr_i)
    mem_r[daddr_i[DEPTH:2]][31:24] <= dwdata_w[31:24];
end

//-----------------------------------------------------------------------------
assign irdata_o = irdata_r;

endmodule

