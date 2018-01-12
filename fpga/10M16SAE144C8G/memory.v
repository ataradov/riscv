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
    (2'b00 == daddr_r) ? drdata_w[7:0] :
    (2'b01 == daddr_r) ? drdata_w[15:8] :
    (2'b10 == daddr_r) ? drdata_w[23:16] : drdata_w[31:24];

wire [15:0] rdata_half_w = 
    daddr_r[1] ? drdata_w[31:16] : drdata_w[15:0];

assign drdata_o =
    (SIZE_BYTE == dsize_r) ? { 24'h0, rdata_byte_w } :
    (SIZE_HALF == dsize_r) ? { 16'h0, rdata_half_w } : drdata_w;

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
wire [31:0] drdata_w;

altsyncram altsyncram_i (
  .clock0(clk_i),
  .aclr0 (reset_i),

  .address_a(iaddr_i[31:2]),
  .data_a(32'h0),
  .rden_a(ird_i),
  .wren_a(1'b0),
  .q_a(irdata_o),

  .address_b(daddr_i[31:2]),
  .byteena_b(dbe_w),
  .data_b(dwdata_w),
  .rden_b(drd_i),
  .wren_b(dwr_i),
  .q_b(drdata_w),

  .aclr1(1'b0),
  .addressstall_a(1'b0),
  .addressstall_b(1'b0),
  .byteena_a(1'b1),
  .clock1(1'b1),
  .clocken0(1'b1),
  .clocken1(1'b1),
  .clocken2(1'b1),
  .clocken3(1'b1),
  .eccstatus()
);

defparam
  altsyncram_i.address_reg_b = "CLOCK0",
  altsyncram_i.byteena_reg_b = "CLOCK0",
  altsyncram_i.byte_size = 8,
  altsyncram_i.clock_enable_input_a = "BYPASS",
  altsyncram_i.clock_enable_input_b = "BYPASS",
  altsyncram_i.clock_enable_output_a = "BYPASS",
  altsyncram_i.clock_enable_output_b = "BYPASS",
  altsyncram_i.indata_reg_b = "CLOCK0",
  altsyncram_i.init_file = FIRMWARE,
  altsyncram_i.intended_device_family = "MAX 10",
  altsyncram_i.lpm_type = "altsyncram",
  altsyncram_i.numwords_a = SIZE / 4,
  altsyncram_i.numwords_b = SIZE / 4,
  altsyncram_i.operation_mode = "BIDIR_DUAL_PORT",
  altsyncram_i.outdata_aclr_a = "CLEAR0",
  altsyncram_i.outdata_aclr_b = "CLEAR0",
  altsyncram_i.outdata_reg_a = "UNREGISTERED", // "CLOCK0"
  altsyncram_i.outdata_reg_b = "UNREGISTERED", // "CLOCK0"
  altsyncram_i.power_up_uninitialized = "FALSE",
  altsyncram_i.ram_block_type = "M9K",
  altsyncram_i.read_during_write_mode_mixed_ports = "DONT_CARE",
  altsyncram_i.read_during_write_mode_port_a = "NEW_DATA_WITH_NBE_READ",
  altsyncram_i.read_during_write_mode_port_b = "NEW_DATA_WITH_NBE_READ",
  altsyncram_i.widthad_a = DEPTH,
  altsyncram_i.widthad_b = DEPTH,
  altsyncram_i.width_a = 32,
  altsyncram_i.width_b = 32,
  altsyncram_i.width_byteena_a = 1,
  altsyncram_i.width_byteena_b = 4,
  altsyncram_i.wrcontrol_wraddress_reg_b = "CLOCK0";

endmodule

