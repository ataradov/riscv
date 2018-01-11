`timescale 1ns / 1ps

module per_gpio (
  input         clk_i,
  input         reset_i,

  input  [15:0] addr_i,
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
  REG_OUT_WRITE = 16'h0000,
  REG_OUT_SET   = 16'h0004,
  REG_OUT_CLR   = 16'h0008,
  REG_OUT_TGL   = 16'h000c,
  REG_IN_READ   = 16'h0010;

//-----------------------------------------------------------------------------
reg [31:0] gpio_out_r;

always @(posedge clk_i) begin
  if (reset_i)
    gpio_out_r <= 32'h0;
  else if (wr_i && (REG_OUT_WRITE == addr_i))
    gpio_out_r <= wdata_i;
  else if (wr_i && (REG_OUT_SET == addr_i))
    gpio_out_r <= gpio_out_r | wdata_i;
  else if (wr_i && (REG_OUT_CLR == addr_i))
    gpio_out_r <= gpio_out_r & ~wdata_i;
  else if (wr_i && (REG_OUT_TGL == addr_i))
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

