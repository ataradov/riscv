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

//----------------------------------------------------------------------------
wire lock_w;

soc #(
  .MEM_SIZE(8192),
  .FIRMWARE("../firmware/test.mem")
) soc_i (
  .clk_i(clk),
  .reset_i(reset),
  .lock_o(lock_w),
  .uart_rx_i(1'b1),
  .gpio_in_i(32'hffffffff)
);

always @(posedge clk) begin
  if (soc_i.daddr_w == 32'h80000000 && soc_i.dwr_w) begin
    $write("\n--- HALT (%1d cycles) ---\n", cycles);
    $finish;
  end
end

always @(posedge clk) begin
  if (lock_w) begin
    $write("\n--- LOCKED at %08x (%1d cycles) ---\n", soc_i.iaddr_w, cycles);
    #50;
    $finish;
  end
end

//----------------------------------------------------------------------------
initial begin
  $dumpfile(`VCD_FILE);
  $dumpvars(0, riscv_tb);

  for (i = 0; i < 32; i = i + 1)
    soc_i.core_i.reg_r[i] = 32'h0;

  clk = 1;
  reset = 1;
  cycles = 0;
  @(posedge clk);
  #2;
  reset = 0;

  #500;
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
  if (soc_i.dwr_w)
    $write("[0x%08x] = 0x%08x\n", soc_i.daddr_w, soc_i.dwdata_w);
`endif

`ifdef DEBUG_REG_WR
  if (soc_i.core_i.rd_we_w) begin
    $write("r%d = 0x%08x\n", soc_i.core_i.rd_index_w, soc_i.core_i.rd_value_w);
  end
`endif

`ifdef DEBUG_PC_WR
  if (soc_i.core_i.branch_taken_w) begin
    $write("--- PC = 0x%08x\n", soc_i.core_i.jump_addr_w);
  end
`endif
end

wire [31:0] debug_opcode_w =
    soc_i.core_i.if_rv_w ? soc_i.core_i.if_rv_op_w : { 16'hx, soc_i.core_i.if_rvc_op_w };

//----------------------------------------------------------------------------
always @(reset)
  $write("------ reset = %d\n", reset);

//----------------------------------------------------------------------------
always @(posedge clk)
  cycles = cycles + 1;

//----------------------------------------------------------------------------
always #5 clk = !clk;

endmodule

