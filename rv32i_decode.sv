module rv32i_decode (
  input  logic [31:0] instr,
  input  logic [31:0] pc,

  input  logic [31:0] rf_rdata1,
  input  logic [31:0] rf_rdata2,

  output rv32i_pkg::idex_t id_ex_decoded
);

  import rv32i_pkg::*;

  always_comb begin
    id_ex_decoded = '0;

    id_ex_decoded.pc     = pc;
    id_ex_decoded.instr  = instr;

    id_ex_decoded.rs1    = rs1(instr);
    id_ex_decoded.rs2    = rs2(instr);
    id_ex_decoded.rd     = rd(instr);

    id_ex_decoded.rdata1 = rf_rdata1;
    id_ex_decoded.rdata2 = rf_rdata2;

    id_ex_decoded.rd_we      = 1'b0;
    id_ex_decoded.use_imm    = 1'b0;
    id_ex_decoded.alu_op     = ALU_ADD;
    id_ex_decoded.imm        = 32'd0;
    id_ex_decoded.mem_read   = 1'b0;
    id_ex_decoded.mem_write  = 1'b0;
    id_ex_decoded.wb_src     = WB_ALU;

    id_ex_decoded.is_branch  = 1'b0;
    id_ex_decoded.br_funct3  = 3'b000;
    id_ex_decoded.is_jal     = 1'b0;
    id_ex_decoded.is_jalr    = 1'b0;

    // Default operand usage.
    // Most I-type style instructions use rs1 and not rs2.
    id_ex_decoded.uses_rs1   = 1'b1;
    id_ex_decoded.uses_rs2   = 1'b0;

    unique case (opcode(instr))

      // --------------------------------------------------------
      // OP-IMM: ADDI, ANDI, ORI, XORI
      // --------------------------------------------------------
      7'b0010011: begin
        id_ex_decoded.use_imm  = 1'b1;
        id_ex_decoded.imm      = imm_i(instr);
        id_ex_decoded.rd_we    = 1'b1;
        id_ex_decoded.wb_src   = WB_ALU;
        id_ex_decoded.uses_rs1 = 1'b1;
        id_ex_decoded.uses_rs2 = 1'b0;

        unique case (funct3(instr))
          3'b000: id_ex_decoded.alu_op = ALU_ADD;
          3'b111: id_ex_decoded.alu_op = ALU_AND;
          3'b110: id_ex_decoded.alu_op = ALU_OR;
          3'b100: id_ex_decoded.alu_op = ALU_XOR;
          default: id_ex_decoded.rd_we = 1'b0;
        endcase
      end

      // --------------------------------------------------------
      // OP: ADD, SUB, AND, OR, XOR
      // --------------------------------------------------------
      7'b0110011: begin
        id_ex_decoded.rd_we    = 1'b1;
        id_ex_decoded.wb_src   = WB_ALU;
        id_ex_decoded.uses_rs1 = 1'b1;
        id_ex_decoded.uses_rs2 = 1'b1;

        unique case (funct3(instr))
          3'b000: begin
            if (funct7(instr) == 7'b0100000)
              id_ex_decoded.alu_op = ALU_SUB;
            else
              id_ex_decoded.alu_op = ALU_ADD;
          end

          3'b111: id_ex_decoded.alu_op = ALU_AND;
          3'b110: id_ex_decoded.alu_op = ALU_OR;
          3'b100: id_ex_decoded.alu_op = ALU_XOR;

          default: id_ex_decoded.rd_we = 1'b0;
        endcase
      end

      // --------------------------------------------------------
      // LUI
      // --------------------------------------------------------
      7'b0110111: begin
        id_ex_decoded.use_imm = 1'b1;
        id_ex_decoded.imm     = imm_u(instr);
        id_ex_decoded.rd_we   = 1'b1;
        id_ex_decoded.wb_src  = WB_ALU;
        id_ex_decoded.alu_op  = ALU_ADD;
        id_ex_decoded.rdata1  = 32'd0;

        // LUI does not use rs1/rs2.
        id_ex_decoded.rs1      = 5'd0;
        id_ex_decoded.rs2      = 5'd0;
        id_ex_decoded.uses_rs1 = 1'b0;
        id_ex_decoded.uses_rs2 = 1'b0;
      end

      // --------------------------------------------------------
      // AUIPC
      // --------------------------------------------------------
      7'b0010111: begin
        id_ex_decoded.use_imm = 1'b1;
        id_ex_decoded.imm     = imm_u(instr);
        id_ex_decoded.rd_we   = 1'b1;
        id_ex_decoded.wb_src  = WB_ALU;
        id_ex_decoded.alu_op  = ALU_ADD;
        id_ex_decoded.rdata1  = pc;

        // AUIPC uses PC, not rs1/rs2.
        id_ex_decoded.rs1      = 5'd0;
        id_ex_decoded.rs2      = 5'd0;
        id_ex_decoded.uses_rs1 = 1'b0;
        id_ex_decoded.uses_rs2 = 1'b0;
      end

      // --------------------------------------------------------
      // LW
      // --------------------------------------------------------
      7'b0000011: begin
        if (funct3(instr) == 3'b010) begin
          id_ex_decoded.use_imm   = 1'b1;
          id_ex_decoded.imm       = imm_i(instr);
          id_ex_decoded.alu_op    = ALU_ADD;
          id_ex_decoded.mem_read  = 1'b1;
          id_ex_decoded.rd_we     = 1'b1;
          id_ex_decoded.wb_src    = WB_MEM;
          id_ex_decoded.uses_rs1  = 1'b1;
          id_ex_decoded.uses_rs2  = 1'b0;
        end
      end

      // --------------------------------------------------------
      // SW
      // --------------------------------------------------------
      7'b0100011: begin
        if (funct3(instr) == 3'b010) begin
          id_ex_decoded.use_imm   = 1'b1;
          id_ex_decoded.imm       = imm_s(instr);
          id_ex_decoded.alu_op    = ALU_ADD;
          id_ex_decoded.mem_write = 1'b1;
          id_ex_decoded.uses_rs1  = 1'b1;
          id_ex_decoded.uses_rs2  = 1'b1;
        end
      end

      // --------------------------------------------------------
      // BEQ / BNE
      // --------------------------------------------------------
      7'b1100011: begin
        if ((funct3(instr) == 3'b000) || (funct3(instr) == 3'b001)) begin
          id_ex_decoded.is_branch = 1'b1;
          id_ex_decoded.br_funct3 = funct3(instr);
          id_ex_decoded.uses_rs1  = 1'b1;
          id_ex_decoded.uses_rs2  = 1'b1;
        end
      end

      // --------------------------------------------------------
      // JAL
      // --------------------------------------------------------
      7'b1101111: begin
        id_ex_decoded.is_jal = 1'b1;
        id_ex_decoded.rd_we  = 1'b1;
        id_ex_decoded.wb_src = WB_PC4;

        // JAL does not use rs1/rs2.
        id_ex_decoded.rs1      = 5'd0;
        id_ex_decoded.rs2      = 5'd0;
        id_ex_decoded.uses_rs1 = 1'b0;
        id_ex_decoded.uses_rs2 = 1'b0;
      end

      // --------------------------------------------------------
      // JALR
      // --------------------------------------------------------
      7'b1100111: begin
        if (funct3(instr) == 3'b000) begin
          id_ex_decoded.is_jalr  = 1'b1;
          id_ex_decoded.use_imm  = 1'b1;
          id_ex_decoded.imm      = imm_i(instr);
          id_ex_decoded.rd_we    = 1'b1;
          id_ex_decoded.wb_src   = WB_PC4;
          id_ex_decoded.uses_rs1 = 1'b1;
          id_ex_decoded.uses_rs2 = 1'b0;
        end
      end

      default: begin
        // Unsupported instruction behaves as NOP.
      end

    endcase

    // x0 should never be written.
    if (id_ex_decoded.rd == 5'd0) begin
      id_ex_decoded.rd_we = 1'b0;
    end
  end

endmodule