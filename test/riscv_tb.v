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

//----------------------------------------------------------------------------
module riscv_tb;

//----------------------------------------------------------------------------
integer     i, j, k, l, m, n;
integer     cycles;

reg         clk;
reg         reset;

wire        lock_w;

wire [31:0] iaddr_w;
wire [31:0] irdata_w;
wire        ird_w;
wire [31:0] daddr_w;
wire [31:0] dwdata_w;
wire [31:0] drdata_w;
wire  [1:0] dsize_w;
wire        drd_w;
wire        dwr_w;

//----------------------------------------------------------------------------
riscv_core #(
  .PC_SIZE(16)
) core_i (
  .clk_i(clk),
  .reset_i(reset),

  .lock_o(lock_w),

  .iaddr_o(iaddr_w),
  .irdata_i(irdata_w),
  .ird_o(ird_w),

  .daddr_o(daddr_w),
  .dwdata_o(dwdata_w),
  .drdata_i(drdata_w),
  .dsize_o(dsize_w),
  .drd_o(drd_w),
  .dwr_o(dwr_w)
);

//----------------------------------------------------------------------------
memory #(
  .DEPTH(16),
  .FILE_NAME("../firmware/test.mem")
) memory_i (
  .clk_i(clk),
  .reset_i(reset),

  .iaddr_i(iaddr_w),
  .irdata_o(irdata_w),
  .ird_i(ird_w),

  .daddr_i(daddr_w),
  .dwdata_i(dwdata_w),
  .drdata_o(drdata_w),
  .dsize_i(dsize_w),
  .drd_i(drd_mem_w),
  .dwr_i(dwr_mem_w)
);

wire mem_access_w = (daddr_w < 32'h10000);

wire drd_mem_w = drd_w & mem_access_w;
wire dwr_mem_w = dwr_w & mem_access_w;

always @(posedge clk) begin
  if (daddr_w == 32'h00010000 && dwr_w) begin
    $write("%c", dwdata_w[7:0]);
  end

  if (daddr_w == 32'h80000000 && dwr_w) begin
    $write("\n--- HALT (%1d cycles) ---\n", cycles);
    $finish;
  end
end

always @(posedge clk) begin
  if (lock_w) begin
    $write("\n--- LOCKED at %08x (%1d cycles) ---\n", iaddr_w, cycles);
    #50;
    $finish;
  end
end

//----------------------------------------------------------------------------
initial begin
  $dumpfile(`VCD_FILE);
  $dumpvars(0, riscv_tb);

  for (i = 0; i < 32; i = i + 1)
    core_i.reg_r[i] = 32'h0;

  clk = 1;
  reset = 1;
  cycles = 0;
  @(posedge clk);
  #2;
  reset = 0;

  #500;
//  #20000;
  #50000000;

  $write("\n--- done (%1d cycles) ---\n", cycles);
  $finish;
end

//----------------------------------------------------------------------------
//`define DEBUG_MEM_WR
//`define DEBUG_REG_WR
//`define DEBUG_PC_WR

always @(posedge clk) begin
`ifdef DEBUG_MEM_WR
  if (dwr_w)
    $write("[0x%08x] = 0x%08x\n", daddr_w, dwdata_w);
`endif

`ifdef DEBUG_REG_WR
  if (core_i.rd_we_w) begin
    $write("r%d = 0x%08x\n", core_i.rd_index_w, core_i.rd_value_w);
  end
`endif

`ifdef DEBUG_PC_WR
  if (core_i.branch_taken_w) begin
    $write("--- PC = 0x%08x\n", core_i.jump_addr_w);
  end
`endif
end

wire [31:0] debug_opcode_w = core_i.if_rv_w ? core_i.if_rv_op_w : { 16'hx, core_i.if_rvc_op_w };

//----------------------------------------------------------------------------
always @(reset)
  $write("------ reset = %d\n", reset);

//----------------------------------------------------------------------------
always @(posedge clk)
  cycles = cycles + 1;

//----------------------------------------------------------------------------
always #5 clk = !clk;

endmodule

