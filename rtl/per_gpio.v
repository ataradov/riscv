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

module per_gpio (
  input         clk_i,
  input         reset_i,

  input  [31:0] addr_i,
  input  [31:0] wdata_i,
  output [31:0] rdata_o,
  input   [1:0] size_i,
  input         rd_i,
  input         wr_i,

  input  [31:0] gpio_in_i,
  output [31:0] gpio_out_o
);

//-----------------------------------------------------------------------------
localparam
  REG_OUT_WRITE = 8'h00,
  REG_OUT_SET   = 8'h04,
  REG_OUT_CLR   = 8'h08,
  REG_OUT_TGL   = 8'h0c,
  REG_IN_READ   = 8'h10;

//-----------------------------------------------------------------------------
wire reg_out_write_w = (REG_OUT_WRITE == addr_i[7:0]);
wire reg_out_set_w   = (REG_OUT_SET   == addr_i[7:0]);
wire reg_out_clr_w   = (REG_OUT_CLR   == addr_i[7:0]);
wire reg_out_tgl_w   = (REG_OUT_TGL   == addr_i[7:0]);

//-----------------------------------------------------------------------------
reg [31:0] gpio_out_r;

always @(posedge clk_i) begin
  if (reset_i)
    gpio_out_r <= 32'h0;
  else if (wr_i && reg_out_write_w)
    gpio_out_r <= wdata_i;
  else if (wr_i && reg_out_set_w)
    gpio_out_r <= gpio_out_r | wdata_i;
  else if (wr_i && reg_out_clr_w)
    gpio_out_r <= gpio_out_r & ~wdata_i;
  else if (wr_i && reg_out_tgl_w)
    gpio_out_r <= gpio_out_r ^ wdata_i;
end

assign gpio_out_o = gpio_out_r;

//-----------------------------------------------------------------------------
reg [31:0] reg_data_r;

always @(posedge clk_i) begin
  reg_data_r <= gpio_in_i; // All addresses are mapped to gpio_in_i
end

//-----------------------------------------------------------------------------
assign rdata_o = reg_data_r;

endmodule

