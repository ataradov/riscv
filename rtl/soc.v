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

module soc (
  input         clk_i,
  input         reset_i,

  output        lock_o,

  input         uart_rx_i,
  output        uart_tx_o,

  input  [31:0] gpio_in_i,
  output [31:0] gpio_out_o
);

//-----------------------------------------------------------------------------
wire [31:0] iaddr_w;
wire [31:0] irdata_w;
wire        ird_w;

wire [31:0] daddr_w;
wire [31:0] dwdata_w;
wire [31:0] drdata_w;
wire  [1:0] dsize_w;
wire        drd_w;
wire        dwr_w;

riscv_core #(
  .PC_SIZE(16)
) core_i (
  .clk_i(clk_i),
  .reset_i(reset_i),

  .lock_o(lock_o),

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

//-----------------------------------------------------------------------------
wire [15:0] mem_addr_w;
wire [31:0] mem_wdata_w;
wire [31:0] mem_rdata_w;
wire  [1:0] mem_size_w;
wire        mem_rd_w;
wire        mem_wr_w;

wire [15:0] uart_addr_w;
wire [31:0] uart_wdata_w;
wire [31:0] uart_rdata_w;
wire  [1:0] uart_size_w;
wire        uart_rd_w;
wire        uart_wr_w;

wire [15:0] gpio_addr_w;
wire [31:0] gpio_wdata_w;
wire [31:0] gpio_rdata_w;
wire  [1:0] gpio_size_w;
wire        gpio_rd_w;
wire        gpio_wr_w;

wire [15:0] timer_addr_w;
wire [31:0] timer_wdata_w;
wire [31:0] timer_rdata_w;
wire  [1:0] timer_size_w;
wire        timer_rd_w;
wire        timer_wr_w;

bus_mux bus_mux_i (
  .clk_i(clk_i),
  .reset_i(reset_i),

  .m_addr_i(daddr_w),
  .m_wdata_i(dwdata_w),
  .m_rdata_o(drdata_w),
  .m_size_i(dsize_w),
  .m_rd_i(drd_w),
  .m_wr_i(dwr_w),

  .s0_addr_o(mem_addr_w),
  .s0_wdata_o(mem_wdata_w),
  .s0_rdata_i(mem_rdata_w),
  .s0_size_o(mem_size_w),
  .s0_rd_o(mem_rd_w),
  .s0_wr_o(mem_wr_w),

  .s1_addr_o(uart_addr_w),
  .s1_wdata_o(uart_wdata_w),
  .s1_rdata_i(uart_rdata_w),
  .s1_size_o(uart_size_w),
  .s1_rd_o(uart_rd_w),
  .s1_wr_o(uart_wr_w),

  .s2_addr_o(gpio_addr_w),
  .s2_wdata_o(gpio_wdata_w),
  .s2_rdata_i(gpio_rdata_w),
  .s2_size_o(gpio_size_w),
  .s2_rd_o(gpio_rd_w),
  .s2_wr_o(gpio_wr_w),

  .s3_addr_o(timer_addr_w),
  .s3_wdata_o(timer_wdata_w),
  .s3_rdata_i(timer_rdata_w),
  .s3_size_o(timer_size_w),
  .s3_rd_o(timer_rd_w),
  .s3_wr_o(timer_wr_w),

  .dummy_i(0)
);

//----------------------------------------------------------------------------
memory #(
  .DEPTH(16),
  .FILE_NAME("../firmware/test.mem")
) memory_i (
  .clk_i(clk_i),
  .reset_i(reset_i),

  .iaddr_i(iaddr_w),
  .irdata_o(irdata_w),
  .ird_i(ird_w),

  .daddr_i({ 16'h0, mem_addr_w }),
  .dwdata_i(mem_wdata_w),
  .drdata_o(mem_rdata_w),
  .dsize_i(mem_size_w),
  .drd_i(mem_rd_w),
  .dwr_i(mem_wr_w)
);

//----------------------------------------------------------------------------
per_uart per_uart_i (
  .clk_i(clk_i),
  .reset_i(reset_i),

  .addr_i(uart_addr_w),
  .wdata_i(uart_wdata_w),
  .rdata_o(uart_rdata_w),
  .size_i(uart_size_w),
  .rd_i(uart_rd_w),
  .wr_i(uart_wr_w),

  .uart_rx_i(uart_rx_i),
  .uart_tx_o(uart_tx_o)
);

//----------------------------------------------------------------------------
per_gpio per_gpio_i (
  .clk_i(clk_i),
  .reset_i(reset_i),

  .addr_i(gpio_addr_w),
  .wdata_i(gpio_wdata_w),
  .rdata_o(gpio_rdata_w),
  .size_i(gpio_size_w),
  .rd_i(gpio_rd_w),
  .wr_i(gpio_wr_w),

  .gpio_in_i(gpio_in_i),
  .gpio_out_o(gpio_out_o)
);

//----------------------------------------------------------------------------
per_timer per_timer_i (
  .clk_i(clk_i),
  .reset_i(reset_i),

  .addr_i(timer_addr_w),
  .wdata_i(timer_wdata_w),
  .rdata_o(timer_rdata_w),
  .size_i(timer_size_w),
  .rd_i(timer_rd_w),
  .wr_i(timer_wr_w)
);

endmodule

