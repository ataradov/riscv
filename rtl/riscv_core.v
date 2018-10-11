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

module riscv_core #(
  parameter     PC_SIZE  = 32,
  parameter     RESET_SP = 32'h2000
)(
  input         clk_i,
  input         reset_i,

  output        lock_o,

  output [31:0] iaddr_o,
  input  [31:0] irdata_i,
  output        ird_o,

  output [31:0] daddr_o,
  output [31:0] dwdata_o,
  input  [31:0] drdata_i,
  output  [1:0] dsize_o,
  output        drd_o,
  output        dwr_o
);

/*
ENABLE_PIPELINE allows pipelined operation, but it creates a very long
combinatorial loop, which is on a critical path big time:
irdata_i - if_opcode_w - hazard_w - id_exec_w - ird_o - iaddr_o.

If ENABLE_PIPELINE is not defined, then the same pipeline structure is
used, but hazards are generated if pipeline stages are busy, without
looking at the actual registers processed by the stage.

A better way to solve this is to decouple the instruction fetch stage
from the decode stage, but this results in a more complex overall design.

And the next critical path is immediate decode, which is much harder
to improve.
*/

`define ENABLE_COMPRESSED_ISA
//`define USE_BARREL_SHIFTER
//`define USE_NATIVE_MULTIPLIER
//`define ENABLE_PIPELINE

/*- Definitions -------------------------------------------------------------*/
localparam
  ALU_ADD   = 4'd0,
  ALU_SUB   = 4'd1,
  ALU_AND   = 4'd2,
  ALU_OR    = 4'd3,
  ALU_XOR   = 4'd4,
  ALU_SLT   = 4'd5,
  ALU_SLTU  = 4'd6,
  ALU_SHL   = 4'd7,
  ALU_SHR   = 4'd8,
  ALU_MULL  = 4'd9,
  ALU_MULH  = 4'd10,
  ALU_DIV   = 4'd11,
  ALU_REM   = 4'd12,
  ALU_NPC   = 4'd13,
  ALU_AUIPC = 4'd14;

localparam
  BR_NONE   = 3'd0,
  BR_JUMP   = 3'd1,
  BR_EQ     = 3'd2,
  BR_NE     = 3'd3,
  BR_LT     = 3'd4,
  BR_GE     = 3'd5,
  BR_LTU    = 3'd6,
  BR_GEU    = 3'd7;

localparam
  SIZE_BYTE = 2'd0,
  SIZE_HALF = 2'd1,
  SIZE_WORD = 2'd2;

localparam
  ST_RESET     = 2'd0,
  ST_LOW       = 2'd1,
  ST_HIGH      = 2'd2,
  ST_UNALIGNED = 2'd3;

/*- Instruction Fetch State Machine -----------------------------------------*/
reg   [1:0] if_state_r;
reg  [15:0] if_buf_r;

always @(posedge clk_i) begin
  if (reset_i) begin
    if_state_r <= ST_RESET;
    if_buf_r <= 16'h0;
  end else if (branch_taken_w) begin
    if_state_r <= jump_addr_w[1] ? ST_HIGH : ST_LOW;
  end else if (id_exec_w) case (if_state_r)
    ST_RESET: begin
      if_state_r <= ST_LOW;
    end

    ST_LOW: begin
      if (if_lo_is_rvc_w && if_hi_is_rvc_w) begin
        if_state_r <= ST_HIGH;
      end else if (if_lo_is_rvc_w && if_hi_is_rv_w) begin
        if_state_r <= ST_UNALIGNED;
        if_buf_r <= irdata_i[31:16];
      end
    end

    ST_HIGH: begin
      if (if_hi_is_rv_w) begin // Only possible after a branch
        if_state_r <= ST_UNALIGNED;
        if_buf_r <= irdata_i[31:16];
      end else begin
        if_state_r <= ST_LOW;
      end
    end

    ST_UNALIGNED: begin
      if (if_hi_is_rv_w)
        if_buf_r <= irdata_i[31:16];
      else
        if_state_r <= ST_HIGH;
    end
  endcase
end

/*- Address Counter ---------------------------------------------------------*/
reg [PC_SIZE-1:2] if_addr_r;

always @(posedge clk_i) begin
  if (reset_i)
    if_addr_r <= 'h0;
  else if (ird_o)
    if_addr_r <= if_next_addr_w + 'd1;
  else if (branch_taken_w)
    if_addr_r <= jump_addr_w[PC_SIZE-1:2];
end

wire [PC_SIZE-1:2] if_next_addr_w =
    branch_taken_w ? jump_addr_w[PC_SIZE-1:2] : if_addr_r;

/*- Instruction Fetch -------------------------------------------------------*/
wire ird_request_w =
    (ST_RESET == if_state_r) ||
    (ST_HIGH == if_state_r) ||
    (ST_LOW == if_state_r && if_lo_is_rv_w) ||
    (ST_LOW == if_state_r && if_hi_is_rv_w && if_lo_is_rvc_w) ||
    (ST_UNALIGNED == if_state_r && if_hi_is_rv_w);

assign iaddr_o = { if_next_addr_w, 2'b00 };
assign ird_o   = branch_taken_w || (ird_request_w && id_exec_w);
assign lock_o  = id_lock_r;

wire if_lo_is_rv_w  = (2'b11 == irdata_i[1:0]);
wire if_lo_is_rvc_w = !if_lo_is_rv_w;
wire if_hi_is_rv_w  = (2'b11 == irdata_i[17:16]);
wire if_hi_is_rvc_w = !if_hi_is_rv_w;

wire if_rv_w    = (ST_UNALIGNED == if_state_r) || (ST_LOW == if_state_r && if_lo_is_rv_w);
wire if_valid_w = !((ST_RESET == if_state_r) || (ST_HIGH == if_state_r && if_hi_is_rv_w));

wire [31:0] if_rv_op_w  = (ST_UNALIGNED == if_state_r) ? { irdata_i[15:0], if_buf_r } : irdata_i;
wire [15:0] if_rvc_op_w = (ST_HIGH == if_state_r) ? irdata_i[31:16] : irdata_i[15:0];

/*- Instruction Fetch Pipeline Registers ------------------------------------*/
reg [PC_SIZE-1:0] if_pc_r;

always @(posedge clk_i) begin
  if (reset_i)
    if_pc_r <= 'h0;
  else if (branch_taken_w)
    if_pc_r <= jump_addr_w[PC_SIZE-1:0];
  else if (if_valid_w && id_exec_w)
    if_pc_r <= if_next_pc_w;
end

wire [PC_SIZE-1:0] if_next_pc_w = if_pc_r + (if_rv_w ? 'd4 : 'd2);

/*- Compressed Instructions Decoder -----------------------------------------*/
`ifdef ENABLE_COMPRESSED_ISA
reg [31:0] if_rvc_dec_w;

always @ (*) begin
  // An illegal RVC opcode is decoded into 32'h0, which is also an illegal RV opcode.
  // We don't explicitly detect illegal RVC opcodes, but let RV decoder deal with them.
  if_rvc_dec_w = 32'h0;

  case ({if_rvc_op_w[15:13], if_rvc_op_w[1:0]})
    5'b00000: begin
      if (if_rvc_op_w[12:2] != 11'h0 && if_rvc_op_w[12:5] != 8'h0) // c.add14spn
        if_rvc_dec_w = { 2'b00, if_rvc_op_w[10:7], if_rvc_op_w[12:11], if_rvc_op_w[5],
            if_rvc_op_w[6], 2'b00, 5'd2, 3'b000, 2'b01, if_rvc_op_w[4:2], 7'b0010011 };
    end

    5'b01000: begin // c.lw
      if_rvc_dec_w = { 5'b00000, if_rvc_op_w[5], if_rvc_op_w[12:10], if_rvc_op_w[6],
          2'b00, 2'b01, if_rvc_op_w[9:7], 3'b010, 2'b01, if_rvc_op_w[4:2], 7'b0000011 };
    end

    5'b11000: begin // c.sw
      if_rvc_dec_w = { 5'b00000, if_rvc_op_w[5], if_rvc_op_w[12], 2'b01, if_rvc_op_w[4:2],
          2'b01, if_rvc_op_w[9:7], 3'b010, if_rvc_op_w[11:10], if_rvc_op_w[6], 2'b00, 7'b0100011 };
    end

    5'b00001: begin
      if (if_rvc_op_w[12:2] == 11'h0) // c.nop
        if_rvc_dec_w = { 25'h0, 7'b0010011 };
      else if (if_rvc_op_w[12] != 1'b0 || if_rvc_op_w[6:2] != 5'h0) // c.addi
        if_rvc_dec_w = { {7{if_rvc_op_w[12]}}, if_rvc_op_w[6:2], if_rvc_op_w[11:7],
            3'b000, if_rvc_op_w[11:7], 7'b0010011 };
    end

    5'b00101: begin // c.jal
      if_rvc_dec_w = { if_rvc_op_w[12], if_rvc_op_w[8], if_rvc_op_w[10:9], if_rvc_op_w[6],
          if_rvc_op_w[7], if_rvc_op_w[2], if_rvc_op_w[11], if_rvc_op_w[5:3], if_rvc_op_w[12],
          {8{if_rvc_op_w[12]}}, 5'd1, 7'b1101111 };
    end

    5'b01001: begin
      if (if_rvc_op_w[11:7] != 5'd0) // c.li
        if_rvc_dec_w = { {7{if_rvc_op_w[12]}}, if_rvc_op_w[6:2], 5'd0, 3'b000,
            if_rvc_op_w[11:7], 7'b0010011 };
    end

    5'b01101: begin
      if ((if_rvc_op_w[12] != 1'b0 || if_rvc_op_w[6:2] != 5'h0) && if_rvc_op_w[11:7] != 5'd0) begin
        if (if_rvc_op_w[11:7] == 5'd2) // c.addi16sp
          if_rvc_dec_w = { {3{if_rvc_op_w[12]}}, if_rvc_op_w[4], if_rvc_op_w[3], if_rvc_op_w[5],
              if_rvc_op_w[2], if_rvc_op_w[6], 4'b0000, 5'd2, 3'b000, 5'd2, 7'b0010011 };
        else // c.lui
          if_rvc_dec_w = { {15{if_rvc_op_w[12]}}, if_rvc_op_w[6:2], if_rvc_op_w[11:7], 7'b0110111 };
      end
    end

    5'b10001: begin
      if (if_rvc_op_w[12:10] == 3'b011 && if_rvc_op_w[6:5] == 2'b00) // c.sub
        if_rvc_dec_w = { 7'b0100000, 2'b01, if_rvc_op_w[4:2], 2'b01, if_rvc_op_w[9:7],
            3'b000, 2'b01, if_rvc_op_w[9:7], 7'b0110011 };
      else if (if_rvc_op_w[12:10] == 3'b011 && if_rvc_op_w[6:5] == 2'b01) // c.xor
        if_rvc_dec_w = { 7'b0000000, 2'b01, if_rvc_op_w[4:2], 2'b01, if_rvc_op_w[9:7],
            3'b100, 2'b01, if_rvc_op_w[9:7], 7'b0110011 };
      else if (if_rvc_op_w[12:10] == 3'b011 && if_rvc_op_w[6:5] == 2'b10) // c.or
        if_rvc_dec_w = { 7'b0000000, 2'b01, if_rvc_op_w[4:2], 2'b01, if_rvc_op_w[9:7],
            3'b110, 2'b01, if_rvc_op_w[9:7], 7'b0110011 };
      else if (if_rvc_op_w[12:10] == 3'b011 && if_rvc_op_w[6:5] == 2'b11) // c.and
        if_rvc_dec_w = { 7'b0000000, 2'b01, if_rvc_op_w[4:2], 2'b01, if_rvc_op_w[9:7],
            3'b111, 2'b01, if_rvc_op_w[9:7], 7'b0110011 };
      else if (if_rvc_op_w[11:10] == 2'b10) // c.andi
        if_rvc_dec_w = { {7{if_rvc_op_w[12]}}, if_rvc_op_w[6:2], 2'b01, if_rvc_op_w[9:7],
            3'b111, 2'b01, if_rvc_op_w[9:7], 7'b0010011 };
      else if (if_rvc_op_w[12] == 1'b0 && if_rvc_op_w[6:2] == 5'h0)
        if_rvc_dec_w = 32'h0;
      else if (if_rvc_op_w[11:10] == 2'b00) // c.srli
        if_rvc_dec_w = { 7'b0000000, if_rvc_op_w[6:2], 2'b01, if_rvc_op_w[9:7],
            3'b101, 2'b01, if_rvc_op_w[9:7], 7'b0010011 };
      else if (if_rvc_op_w[11:10] == 2'b01) // c.srai
        if_rvc_dec_w = { 7'b0100000, if_rvc_op_w[6:2], 2'b01, if_rvc_op_w[9:7],
            3'b101, 2'b01, if_rvc_op_w[9:7], 7'b0010011 };
    end

    5'b10101: begin // c.j
      if_rvc_dec_w = { if_rvc_op_w[12], if_rvc_op_w[8], if_rvc_op_w[10:9], if_rvc_op_w[6],
          if_rvc_op_w[7], if_rvc_op_w[2], if_rvc_op_w[11], if_rvc_op_w[5:3], if_rvc_op_w[12],
          {8{if_rvc_op_w[12]}}, 5'd0, 7'b1101111 };
    end

    5'b11001: begin // c.beqz
      if_rvc_dec_w = { {4{if_rvc_op_w[12]}}, if_rvc_op_w[6], if_rvc_op_w[5], if_rvc_op_w[2],
          5'd0, 2'b01, if_rvc_op_w[9:7], 3'b000, if_rvc_op_w[11], if_rvc_op_w[10],
          if_rvc_op_w[4], if_rvc_op_w[3], if_rvc_op_w[12], 7'b1100011 };
    end

    5'b11101: begin // c.bnez
      if_rvc_dec_w = { {4{if_rvc_op_w[12]}}, if_rvc_op_w[6], if_rvc_op_w[5], if_rvc_op_w[2],
          5'd0, 2'b01, if_rvc_op_w[9:7], 3'b001, if_rvc_op_w[11], if_rvc_op_w[10],
          if_rvc_op_w[4], if_rvc_op_w[3], if_rvc_op_w[12], 7'b1100011 };
    end

    5'b00010: begin
      if (if_rvc_op_w[11:7] != 5'd0) // c.slli
        if_rvc_dec_w = { 7'b0000000, if_rvc_op_w[6:2], if_rvc_op_w[11:7], 3'b001,
            if_rvc_op_w[11:7], 7'b0010011 };
    end

    5'b01010: begin
      if (if_rvc_op_w[11:7] != 5'h0) // c.lwsp
        if_rvc_dec_w = { 4'b0000, if_rvc_op_w[3:2], if_rvc_op_w[12], if_rvc_op_w[6:4],
            2'b0, 5'd2, 3'b010, if_rvc_op_w[11:7], 7'b0000011 };
    end

    5'b11010: begin // c.swsp
      if_rvc_dec_w = { 4'b0000, if_rvc_op_w[8:7], if_rvc_op_w[12], if_rvc_op_w[6:2],
          5'd2, 3'b010, if_rvc_op_w[11:9], 2'b00, 7'b0100011 };
    end

    5'b10010: begin
      if (if_rvc_op_w[6:2] == 5'd0) begin
        if (if_rvc_op_w[11:7] == 5'h0) begin
          if (if_rvc_op_w[12] == 1'b1) // c.ebreak
            if_rvc_dec_w = { 11'h0, 1'b1, 13'h0, 7'b1110011 };
        end else if (if_rvc_op_w[12])
          if_rvc_dec_w = { 12'h0, if_rvc_op_w[11:7], 3'b000, 5'd1, 7'b1100111 }; // c.jalr
        else
          if_rvc_dec_w = { 12'h0, if_rvc_op_w[11:7], 3'b000, 5'd0, 7'b1100111 }; // c.jr
      end else if (if_rvc_op_w[11:7] != 5'h0) begin
        if (if_rvc_op_w[12] == 1'b0) // c.mv
          if_rvc_dec_w = { 7'b0000000, if_rvc_op_w[6:2], 5'd0, 3'b000,
              if_rvc_op_w[11:7], 7'b0110011 };
        else // c.add
          if_rvc_dec_w = { 7'b0000000, if_rvc_op_w[6:2], if_rvc_op_w[11:7],
              3'b000, if_rvc_op_w[11:7], 7'b0110011 };
      end
    end

    default: begin
    end
  endcase
end
`else
wire [31:0] if_rvc_dec_w = 32'h0;
`endif

wire [31:0] if_opcode_w = if_rv_w ? if_rv_op_w : if_rvc_dec_w;

/*- Instruction Decoder -----------------------------------------------------*/
wire [6:0] op_w = if_opcode_w[6:0];
wire [4:0] rd_w = if_opcode_w[11:7];
wire [2:0] f3_w = if_opcode_w[14:12];
wire [4:0] ra_w = if_opcode_w[19:15];
wire [4:0] rb_w = if_opcode_w[24:20];
wire [6:0] f7_w = if_opcode_w[31:25];

wire op_branch_w  = (7'b1100011 == op_w);
wire op_load_w    = (7'b0000011 == op_w);
wire op_store_w   = (7'b0100011 == op_w);
wire op_alu_imm_w = (7'b0010011 == op_w);
wire op_alu_reg_w = (7'b0110011 == op_w);

wire op_f7_main_w = (7'b0000000 == f7_w);
wire op_f7_alt_w  = (7'b0100000 == f7_w);
wire op_f7_mul_w  = (7'b0000001 == f7_w);

wire lui_w    = (7'b0110111 == op_w);
wire auipc_w  = (7'b0010111 == op_w);
wire jal_w    = (7'b1101111 == op_w);
wire jalr_w   = (7'b1100111 == op_w) && (3'b000 == f3_w);

wire beq_w    = op_branch_w  && (3'b000 == f3_w);
wire bne_w    = op_branch_w  && (3'b001 == f3_w);
wire blt_w    = op_branch_w  && (3'b100 == f3_w);
wire bge_w    = op_branch_w  && (3'b101 == f3_w);
wire bltu_w   = op_branch_w  && (3'b110 == f3_w);
wire bgeu_w   = op_branch_w  && (3'b111 == f3_w);

wire lb_w     = op_load_w    && (3'b000 == f3_w);
wire lh_w     = op_load_w    && (3'b001 == f3_w);
wire lw_w     = op_load_w    && (3'b010 == f3_w);
wire lbu_w    = op_load_w    && (3'b100 == f3_w);
wire lhu_w    = op_load_w    && (3'b101 == f3_w);

wire sb_w     = op_store_w   && (3'b000 == f3_w);
wire sh_w     = op_store_w   && (3'b001 == f3_w);
wire sw_w     = op_store_w   && (3'b010 == f3_w);

wire addi_w   = op_alu_imm_w && (3'b000 == f3_w);
wire slti_w   = op_alu_imm_w && (3'b010 == f3_w);
wire sltiu_w  = op_alu_imm_w && (3'b011 == f3_w);
wire xori_w   = op_alu_imm_w && (3'b100 == f3_w);
wire ori_w    = op_alu_imm_w && (3'b110 == f3_w);
wire andi_w   = op_alu_imm_w && (3'b111 == f3_w);
wire slli_w   = op_alu_imm_w && (3'b001 == f3_w) && op_f7_main_w;
wire srli_w   = op_alu_imm_w && (3'b101 == f3_w) && op_f7_main_w;
wire srai_w   = op_alu_imm_w && (3'b101 == f3_w) && op_f7_alt_w;

wire add_w    = op_alu_reg_w && (3'b000 == f3_w) && op_f7_main_w;
wire sub_w    = op_alu_reg_w && (3'b000 == f3_w) && op_f7_alt_w;
wire slt_w    = op_alu_reg_w && (3'b010 == f3_w) && op_f7_main_w;
wire sltu_w   = op_alu_reg_w && (3'b011 == f3_w) && op_f7_main_w;
wire xor_w    = op_alu_reg_w && (3'b100 == f3_w) && op_f7_main_w;
wire or_w     = op_alu_reg_w && (3'b110 == f3_w) && op_f7_main_w;
wire and_w    = op_alu_reg_w && (3'b111 == f3_w) && op_f7_main_w;
wire sll_w    = op_alu_reg_w && (3'b001 == f3_w) && op_f7_main_w;
wire srl_w    = op_alu_reg_w && (3'b101 == f3_w) && op_f7_main_w;
wire sra_w    = op_alu_reg_w && (3'b101 == f3_w) && op_f7_alt_w;

wire mul_w    = op_alu_reg_w && (3'b000 == f3_w) && op_f7_mul_w;
wire mulh_w   = op_alu_reg_w && (3'b001 == f3_w) && op_f7_mul_w;
wire mulhsu_w = op_alu_reg_w && (3'b010 == f3_w) && op_f7_mul_w;
wire mulhu_w  = op_alu_reg_w && (3'b011 == f3_w) && op_f7_mul_w;
wire div_w    = op_alu_reg_w && (3'b100 == f3_w) && op_f7_mul_w;
wire divu_w   = op_alu_reg_w && (3'b101 == f3_w) && op_f7_mul_w;
wire rem_w    = op_alu_reg_w && (3'b110 == f3_w) && op_f7_mul_w;
wire remu_w   = op_alu_reg_w && (3'b111 == f3_w) && op_f7_mul_w;

//-----------------------------------------------------------------------------
wire load_w    = lb_w || lh_w || lw_w || lbu_w || lhu_w;
wire store_w   = sb_w || sh_w || sw_w;

wire alu_imm_w = addi_w || slti_w || sltiu_w || xori_w || ori_w || andi_w ||
                 slli_w || srli_w || srai_w || lui_w || auipc_w;

wire alu_reg_w = add_w || sub_w || slt_w || sltu_w || xor_w || or_w || and_w ||
                 sll_w || srl_w || sra_w || mul_w || mulh_w || mulhsu_w ||
                 mulhu_w || div_w || divu_w || rem_w || remu_w;

wire branch_w  = beq_w || bne_w || blt_w || bge_w || bltu_w || bgeu_w;
wire jump_w    = jal_w || jalr_w;

wire id_illegal_w = !(load_w || store_w || alu_imm_w || alu_reg_w || jump_w || branch_w);

wire [31:0] id_i_imm_w = { {20{if_opcode_w[31]}}, if_opcode_w[31:20] };
wire [31:0] id_s_imm_w = { {20{if_opcode_w[31]}}, if_opcode_w[31:25], if_opcode_w[11:7] };
wire [31:0] id_b_imm_w = { {19{if_opcode_w[31]}}, if_opcode_w[31], if_opcode_w[7], if_opcode_w[30:25], if_opcode_w[11:8], 1'b0 };
wire [31:0] id_u_imm_w = { if_opcode_w[31:12], 12'h0 };
wire [31:0] id_j_imm_w = { {11{if_opcode_w[31]}}, if_opcode_w[31], if_opcode_w[19:12], if_opcode_w[20], if_opcode_w[30:21], 1'b0 };

wire [31:0] id_imm_w =
    (lui_w || auipc_w)              ? id_u_imm_w :
    (branch_w)                      ? id_b_imm_w :
    (load_w || jalr_w || alu_imm_w) ? id_i_imm_w :
    (store_w)                       ? id_s_imm_w :
    (jal_w)                         ? id_j_imm_w : 32'h0;

wire [4:0] id_rd_index_w = (branch_w || store_w)           ? 5'd0 : rd_w;
wire [4:0] id_ra_index_w = (lui_w || auipc_w || jal_w)     ? 5'd0 : ra_w;
wire [4:0] id_rb_index_w = (load_w || jump_w || alu_imm_w) ? 5'd0 : rb_w;

wire [3:0] id_alu_op_w =
    (add_w || addi_w || lui_w || load_w || store_w) ? ALU_ADD :
    (andi_w || and_w)                    ? ALU_AND :
    (ori_w || or_w)                      ? ALU_OR :
    (xori_w || xor_w)                    ? ALU_XOR :
    (slti_w || slt_w)                    ? ALU_SLT :
    (sltiu_w || sltu_w)                  ? ALU_SLTU :
    (sll_w || slli_w)                    ? ALU_SHL :
    (srl_w || srli_w || sra_w || srai_w) ? ALU_SHR :
    (mulh_w || mulhsu_w || mulhu_w)      ? ALU_MULH :
    (mul_w)                              ? ALU_MULL :
    (div_w || divu_w)                    ? ALU_DIV :
    (rem_w || remu_w)                    ? ALU_REM :
    (jal_w || jalr_w)                    ? ALU_NPC :
    (auipc_w)                            ? ALU_AUIPC : ALU_SUB;

wire [2:0] id_branch_w =
    beq_w  ? BR_EQ :
    bne_w  ? BR_NE :
    blt_w  ? BR_LT :
    bge_w  ? BR_GE :
    bltu_w ? BR_LTU :
    bgeu_w ? BR_GEU :
    jump_w ? BR_JUMP : BR_NONE;

wire [1:0] id_mem_size_w =
    (lb_w || lbu_w || sb_w) ? SIZE_BYTE :
    (lh_w || lhu_w || sh_w) ? SIZE_HALF : SIZE_WORD;

/*- Hazard Detection --------------------------------------------------------*/
`ifdef ENABLE_PIPELINE
wire id_hazard_w = (id_rd_index_r != 5'd0) && (id_ra_index_w == id_rd_index_r || id_rb_index_w == id_rd_index_r);
wire ex_hazard_w = (ex_rd_index_r != 5'd0) && (id_ra_index_w == ex_rd_index_r || id_rb_index_w == ex_rd_index_r);
wire hazard_w    = id_hazard_w || ex_hazard_w;
`else
reg id_hazard_r;
reg ex_hazard_r;

always @(posedge clk_i) begin
  if (id_bubble_w)
    id_hazard_r <= 1'b0;
  else if (id_ready_w)
    id_hazard_r <= 1'b1;

  if (ex_bubble_w)
    ex_hazard_r <= 1'b0;
  else if (ex_ready_w)
    ex_hazard_r <= id_hazard_r;
end

wire hazard_w = id_hazard_r || ex_hazard_r;
`endif

/*- Instruction Decoder Pipeline Registers ----------------------------------*/
reg [31:0] reg_r [0:31];
reg [PC_SIZE-1:0] id_pc_r;
reg [PC_SIZE-1:0] id_next_pc_r;
reg  [4:0] id_rd_index_r;
reg [31:0] id_ra_value_r;
reg [31:0] id_rb_value_r;
reg [31:0] id_imm_r;
reg        id_a_signed_r;
reg        id_b_signed_r;
reg        id_op_imm_r;
reg  [3:0] id_alu_op_r;
reg        id_mem_rd_r;
reg        id_mem_wr_r;
reg        id_mem_signed_r;
reg  [1:0] id_mem_size_r;
reg  [2:0] id_branch_r;
reg        id_reg_jump_r;
reg        id_lock_r;

wire id_bubble_w = reset_i || ((branch_taken_w || hazard_w || !if_valid_w) && id_ready_w);
wire id_ready_w  = !ex_stall_w && !mem_stall_w && !id_lock_r;
wire id_exec_w   = id_ready_w && !hazard_w;

always @(posedge clk_i) begin
  if (id_bubble_w) begin
    id_pc_r         <= 'h0;
    id_next_pc_r    <= 'h0;
    id_rd_index_r   <= 5'd0;
    id_imm_r        <= 32'h0;
    id_a_signed_r   <= 1'b0;
    id_b_signed_r   <= 1'b0;
    id_op_imm_r     <= 1'b0;
    id_alu_op_r     <= ALU_ADD;
    id_mem_rd_r     <= 1'b0;
    id_mem_wr_r     <= 1'b0;
    id_mem_signed_r <= 1'b0;
    id_mem_size_r   <= SIZE_BYTE;
    id_branch_r     <= BR_NONE;
    id_reg_jump_r   <= 1'b0;
    id_lock_r       <= 1'b0;
  end else if (id_ready_w) begin
    id_pc_r         <= if_pc_r;
    id_next_pc_r    <= if_next_pc_w;
    id_rd_index_r   <= id_rd_index_w;
    id_imm_r        <= id_imm_w;
    id_a_signed_r   <= mulh_w || mulhsu_w || div_w || rem_w || sra_w || srai_w;
    id_b_signed_r   <= mulh_w || div_w || rem_w;
    id_op_imm_r     <= alu_imm_w || jal_w || load_w || store_w;
    id_alu_op_r     <= id_alu_op_w;
    id_mem_rd_r     <= load_w;
    id_mem_wr_r     <= store_w;
    id_mem_signed_r <= !lbu_w && !lhu_w;
    id_mem_size_r   <= id_mem_size_w;
    id_branch_r     <= id_branch_w;
    id_reg_jump_r   <= jalr_w;
    id_ra_value_r   <= reg_r[id_ra_index_w];
    id_rb_value_r   <= reg_r[id_rb_index_w];
    id_lock_r       <= id_illegal_w;
  end
end

/*- ALU ---------------------------------------------------------------------*/
wire [31:0] alu_opb_w = id_op_imm_r ? id_imm_r : id_rb_value_r;

/*- Adder -------------------------------------------------------------------*/
wire        adder_sub_w = (ALU_SUB == id_alu_op_r || ALU_SLT == id_alu_op_r || ALU_SLTU == id_alu_op_r);
wire [31:0] adder_opa_w = id_ra_value_r;
wire [31:0] adder_opb_w = adder_sub_w ? ~alu_opb_w : alu_opb_w;
wire        adder_cin_w = adder_sub_w ? 1'b1 : 1'b0;
wire [31:0] adder_out_w;
wire        adder_c_w;
wire        adder_n_w = adder_out_w[31];
wire        adder_v_w = (adder_opa_w[31] == adder_opb_w[31] && adder_out_w[31] != adder_opb_w[31]);
wire        adder_z_w = (32'h0 == adder_out_w);

assign { adder_c_w, adder_out_w } = { 1'b0, adder_opa_w } + { 1'b0, adder_opb_w } + adder_cin_w;

/*- Shifter -----------------------------------------------------------------*/
wire sh_fill_w = id_a_signed_r && id_ra_value_r[31];

`ifdef USE_BARREL_SHIFTER
wire [31:0] sh_left_w  = sl_4_w;
wire [31:0] sh_right_w = sr_4_w;

wire [31:0] sl_0_w = alu_opb_w[0] ? { id_ra_value_r[30:0], 1'b0 } : id_ra_value_r;
wire [31:0] sl_1_w = alu_opb_w[1] ? { sl_0_w[29:0],  2'b0 } : sl_0_w;
wire [31:0] sl_2_w = alu_opb_w[2] ? { sl_1_w[27:0],  4'b0 } : sl_1_w;
wire [31:0] sl_3_w = alu_opb_w[3] ? { sl_2_w[23:0],  8'b0 } : sl_2_w;
wire [31:0] sl_4_w = alu_opb_w[4] ? { sl_3_w[15:0], 16'b0 } : sl_3_w;

wire [31:0] sr_0_w = alu_opb_w[0] ? {  {1{sh_fill_w}}, id_ra_value_r[31:1] } : id_ra_value_r;
wire [31:0] sr_1_w = alu_opb_w[1] ? {  {2{sh_fill_w}}, sr_0_w[31:2] }  : sr_0_w;
wire [31:0] sr_2_w = alu_opb_w[2] ? {  {4{sh_fill_w}}, sr_1_w[31:4] }  : sr_1_w;
wire [31:0] sr_3_w = alu_opb_w[3] ? {  {8{sh_fill_w}}, sr_2_w[31:8] }  : sr_2_w;
wire [31:0] sr_4_w = alu_opb_w[4] ? { {16{sh_fill_w}}, sr_3_w[31:16] } : sr_3_w;

wire ex_stall_sh_w = 1'b0;

`else // Slow sequntial shifter
reg         sh_busy_r;
reg         sh_ready_r;
reg   [4:0] sh_count_r;
reg  [31:0] sh_res_r;

wire [31:0] sh_left_w  = sh_res_r;
wire [31:0] sh_right_w = sh_res_r;

always @(posedge clk_i) begin
  if (reset_i) begin
    sh_busy_r  <= 1'b0;
    sh_ready_r <= 1'b0;
    sh_count_r <= 5'd0;
    sh_res_r   <= 32'h0;
  end else begin
    if (sh_busy_r) begin
      sh_count_r <= sh_count_r - 5'd1;
      sh_res_r   <= (ALU_SHL == id_alu_op_r) ? { sh_res_r[30:0], 1'b0 } : { sh_fill_w, sh_res_r[31:1] };

      if (sh_count_r == 5'd1) begin
        sh_busy_r  <= 1'b0;
        sh_ready_r <= 1'b1;
      end

    end else if (sh_ready_r) begin
      sh_ready_r <= 1'b0;

    end else if (sh_request_w) begin
      sh_count_r <= alu_opb_w[4:0];
      sh_res_r   <= id_ra_value_r;

      if (alu_opb_w[4:0] == 5'h0)
        sh_ready_r <= 1'b1;
      else
        sh_busy_r  <= 1'b1;
    end
  end
end

wire sh_request_w  = (ALU_SHL == id_alu_op_r || ALU_SHR == id_alu_op_r);
wire ex_stall_sh_w = sh_request_w && !sh_ready_r;
`endif

/*- Multiplier and Divider Common -------------------------------------------*/
wire mul_div_negative_w = (id_a_signed_r && id_ra_value_r[31]) ^ (id_b_signed_r && id_rb_value_r[31]);
wire [31:0] mul_div_a_w = (id_a_signed_r && id_ra_value_r[31]) ? -id_ra_value_r : id_ra_value_r;
wire [31:0] mul_div_b_w = (id_b_signed_r && id_rb_value_r[31]) ? -id_rb_value_r : id_rb_value_r;

/*- Multiplier --------------------------------------------------------------*/
`ifdef USE_NATIVE_MULTIPLIER
wire [63:0] mul_opa_a_w = { {32{id_a_signed_r & id_ra_value_r[31]}}, id_ra_value_r };
wire [63:0] mul_opa_b_w = { {32{id_b_signed_r & id_rb_value_r[31]}}, id_rb_value_r };
wire [63:0] mul_res_w = mul_opa_a_w * mul_opa_b_w;
wire ex_stall_mul_w = 1'b0;

`else // Slow sequntial multiplier
reg         mul_busy_r;
reg         mul_ready_r;
reg   [4:0] mul_count_r;
reg  [63:0] mul_res_r;

wire [32:0] mul_sum_w = { 1'b0, mul_res_r[63:32] } + { 1'b0, mul_res_r[0] ? mul_div_b_w : 32'h0 };

wire [63:0] mul_res_w = mul_div_negative_w ? -mul_res_r : mul_res_r;

always @(posedge clk_i) begin
  if (reset_i) begin
    mul_busy_r  <= 1'b0;
    mul_ready_r <= 1'b0;
    mul_count_r <= 5'd0;
    mul_res_r   <= 64'h0;
  end else begin
    if (mul_busy_r) begin
      mul_count_r <= mul_count_r - 5'd1;
      mul_res_r   <= { mul_sum_w, mul_res_r[31:1] };

      if (mul_count_r == 5'd0) begin
        mul_busy_r  <= 1'b0;
        mul_ready_r <= 1'b1;
      end

    end else if (mul_ready_r) begin
      mul_ready_r <= 1'b0;

    end else if (mul_request_w) begin
      mul_count_r <= 5'd31;
      mul_busy_r  <= 1'b1;
      mul_res_r   <= { 32'h0, mul_div_a_w };
    end
  end
end

wire mul_request_w  = (ALU_MULL == id_alu_op_r || ALU_MULH == id_alu_op_r);
wire ex_stall_mul_w = mul_request_w && !mul_ready_r;
`endif

/*- Divider -----------------------------------------------------------------*/
reg         div_busy_r;
reg         div_ready_r;
reg   [4:0] div_count_r;
reg  [31:0] div_rem_r;
reg  [31:0] div_quot_r;

wire [32:0] div_sub_w = { 1'b0, div_rem_r[30:0], div_quot_r[31] } - { 1'b0, mul_div_b_w };

wire [31:0] div_quotient_w  = mul_div_negative_w ? -div_quot_r : div_quot_r;
wire [31:0] div_remainder_w = mul_div_negative_w ? -div_rem_r : div_rem_r;

always @(posedge clk_i) begin
  if (reset_i) begin
    div_busy_r  <= 1'b0;
    div_ready_r <= 1'b0;
    div_count_r <= 5'd0;
    div_quot_r  <= 32'h0;
    div_rem_r   <= 32'h0;
  end else begin
    if (div_busy_r) begin
      div_count_r <= div_count_r - 5'd1;
      div_quot_r  <= { div_quot_r[30:0], !div_sub_w[32] };

      if (div_sub_w[32])
        div_rem_r <= { div_rem_r[30:0], div_quot_r[31] };
      else
        div_rem_r <= div_sub_w[31:0];

      if (div_count_r == 5'd0) begin
        div_busy_r  <= 1'b0;
        div_ready_r <= 1'b1;
      end

    end else if (div_ready_r) begin
      div_ready_r <= 1'b0;

    end else if (div_request_w) begin
      div_count_r <= 5'd31;
      div_busy_r  <= 1'b1;
      div_quot_r  <= mul_div_a_w;
      div_rem_r   <= 32'h0;
    end
  end
end

wire div_request_w  = (ALU_DIV == id_alu_op_r || ALU_REM == id_alu_op_r);
wire ex_stall_div_w = div_request_w && !div_ready_r;

/*- ALU Result Multiplexer --------------------------------------------------*/
reg [31:0] ex_alu_res_w;

always @(*) begin
  case (id_alu_op_r)
    ALU_ADD   : ex_alu_res_w = adder_out_w;
    ALU_SUB   : ex_alu_res_w = adder_out_w;
    ALU_AND   : ex_alu_res_w = id_ra_value_r & alu_opb_w;
    ALU_OR    : ex_alu_res_w = id_ra_value_r | alu_opb_w;
    ALU_XOR   : ex_alu_res_w = id_ra_value_r ^ alu_opb_w;
    ALU_SLT   : ex_alu_res_w = (adder_n_w != adder_v_w) ? 32'h1 : 32'h0;
    ALU_SLTU  : ex_alu_res_w = (adder_c_w == 1'b0) ? 32'h1 : 32'h0;
    ALU_SHL   : ex_alu_res_w = sh_left_w;
    ALU_SHR   : ex_alu_res_w = sh_right_w;
    ALU_MULL  : ex_alu_res_w = mul_res_w[31:0];
    ALU_MULH  : ex_alu_res_w = mul_res_w[63:32];
    ALU_DIV   : ex_alu_res_w = div_quotient_w;
    ALU_REM   : ex_alu_res_w = div_remainder_w;
    ALU_NPC   : ex_alu_res_w = id_next_pc_r;
    ALU_AUIPC : ex_alu_res_w = jump_addr_w;
    default   : ex_alu_res_w = 32'h0;
  endcase
end

wire ex_stall_w = ex_stall_sh_w || ex_stall_mul_w || ex_stall_div_w;

/*- Jump and Branch Logic ---------------------------------------------------*/
wire branch_taken_w =
    (BR_JUMP == id_branch_r) ? 1'b1 :
    (BR_EQ   == id_branch_r) ? adder_z_w :
    (BR_NE   == id_branch_r) ? !adder_z_w :
    (BR_LT   == id_branch_r) ? adder_n_w != adder_v_w :
    (BR_GE   == id_branch_r) ? adder_n_w == adder_v_w :
    (BR_LTU  == id_branch_r) ? !adder_c_w :
    (BR_GEU  == id_branch_r) ? adder_c_w : 1'b0;

wire [31:0] jump_addr_w = (id_reg_jump_r ? id_ra_value_r : id_pc_r) + id_imm_r;

/*- Execute Pipeline Registers ----------------------------------------------*/
reg  [4:0] ex_rd_index_r;
reg [31:0] ex_alu_res_r;
reg [31:0] ex_mem_data_r;
reg        ex_mem_rd_r;
reg        ex_mem_wr_r;
reg        ex_mem_signed_r;
reg  [1:0] ex_mem_size_r;

wire ex_bubble_w = reset_i || (ex_stall_w && !mem_stall_w);
wire ex_ready_w  = !mem_stall_w;

always @(posedge clk_i) begin
  if (ex_bubble_w) begin
    if (reset_i) begin
      ex_rd_index_r <= 5'd2; // SP
      ex_alu_res_r  <= RESET_SP;
    end else begin
      ex_rd_index_r <= 5'd0;
      ex_alu_res_r  <= 32'h0;
    end

    ex_mem_data_r   <= 32'h0;
    ex_mem_rd_r     <= 1'b0;
    ex_mem_wr_r     <= 1'b0;
    ex_mem_signed_r <= 1'b0;
    ex_mem_size_r   <= SIZE_BYTE;
  end else if (ex_ready_w) begin
    ex_rd_index_r   <= id_rd_index_r;
    ex_alu_res_r    <= ex_alu_res_w;
    ex_mem_data_r   <= id_rb_value_r;
    ex_mem_rd_r     <= id_mem_rd_r;
    ex_mem_wr_r     <= id_mem_wr_r;
    ex_mem_signed_r <= id_mem_signed_r;
    ex_mem_size_r   <= id_mem_size_r;
  end
end

/*- Memory ------------------------------------------------------------------*/
assign daddr_o  = ex_alu_res_r;
assign dwdata_o = ex_mem_data_r;
assign dsize_o  = ex_mem_size_r;
assign drd_o    = ex_mem_rd_r && (mem_stall_r == 1'b0);
assign dwr_o    = ex_mem_wr_r && (mem_stall_r == 1'b0);

wire [31:0] mem_rdata_w =
  (SIZE_BYTE == ex_mem_size_r) ? { {24{ex_mem_signed_r & drdata_i[7]}}, drdata_i[7:0] } :
  (SIZE_HALF == ex_mem_size_r) ? { {16{ex_mem_signed_r & drdata_i[15]}}, drdata_i[15:0] } : drdata_i;

reg mem_stall_r;

always @(posedge clk_i) begin
  if (reset_i)
    mem_stall_r <= 1'b0;
  else
    mem_stall_r <= mem_stall_w;
end

wire mem_access_w = (ex_mem_rd_r || ex_mem_wr_r);
wire mem_stall_w  = mem_stall_r ? 1'b0 : mem_access_w;

/*- Writeback ---------------------------------------------------------------*/
wire  [4:0] rd_index_w = ex_rd_index_r;
wire [31:0] rd_value_w = mem_access_w ? mem_rdata_w : ex_alu_res_r;
wire        rd_we_w    = (ex_rd_index_r != 5'd0) && (mem_stall_w == 1'b0);

always @(posedge clk_i) begin
  if (rd_we_w)
    reg_r[rd_index_w] <= rd_value_w;
end

endmodule

