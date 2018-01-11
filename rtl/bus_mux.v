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


