`timescale 1ns / 1ps

module per_timer (
  input         clk_i,
  input         reset_i,

  input  [15:0] addr_i,
  input  [31:0] wdata_i,
  output [31:0] rdata_o,
  input   [1:0] size_i,
  input         rd_i,
  input         wr_i
);

//-----------------------------------------------------------------------------
localparam
  REG_CSR     = 16'h0000,
  REG_COUNT   = 16'h0004,
  REG_COMPARE = 16'h0008;

localparam
  BIT_CSR_ENABLE   = 0,
  BIT_CSR_DISABLE  = 1,
  BIT_CSR_OVERFLOW = 2;

//-----------------------------------------------------------------------------
wire csr_wr_w = wr_i && (REG_CSR == addr_i);

//-----------------------------------------------------------------------------
reg timer_enabled_r;

always @(posedge clk_i) begin
  if (reset_i)
    timer_enabled_r <= 1'b0;
  else if (csr_wr_w && wdata_i[BIT_CSR_ENABLE])
    timer_enabled_r <= 1'b1;
  else if (csr_wr_w && wdata_i[BIT_CSR_DISABLE])
    timer_enabled_r <= 1'b0;
end

//-----------------------------------------------------------------------------
reg timer_overflow_r;

always @(posedge clk_i) begin
  if (reset_i)
    timer_overflow_r <= 1'b0;
  else if (csr_wr_w && wdata_i[BIT_CSR_OVERFLOW])
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
  else if (wr_i && (REG_COUNT == addr_i))
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
  else if (wr_i && (REG_COMPARE == addr_i))
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

