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

module riscv_demo (
  input         clk_i,
  output  [7:0] led_o,
  input   [2:0] button_i,
  input         uart_rx_i,
  output        uart_tx_o
);

//-----------------------------------------------------------------------------
reg [15:0] reset_cnt_r = 16'hffff;
reg reset_r = 1'b1;

always @(posedge clk_i) begin
  if (0 == button_i[0]) begin
    reset_r <= 1'b1;
    reset_cnt_r <= 16'hffff;
  end else if (reset_cnt_r > 0)
    reset_cnt_r <= reset_cnt_r - 1'd1;
  else
    reset_r <= 0;
end

//-----------------------------------------------------------------------------
wire        lock_w;
wire [31:0] gpio_out_w;

soc #(
  .MEM_SIZE(8192),
  .FIRMWARE("../../firmware/test.mif")
) soc_i (
  .clk_i(clk_i),
  .reset_i(reset_r),
  .lock_o(lock_w),
  .uart_rx_i(uart_rx_i),
  .uart_tx_o(uart_tx_o),
  .gpio_in_i({ 30'h0, button_i[2:1] }),
  .gpio_out_o(gpio_out_w)
);

//-----------------------------------------------------------------------------
assign led_o = gpio_out_w[7:0];

endmodule

