module rv32i_forward_unit (
  input  logic        id_ex_uses_rs1,
  input  logic        id_ex_uses_rs2,
  input  logic [4:0]  id_ex_rs1,
  input  logic [4:0]  id_ex_rs2,

  input  logic [31:0] id_ex_rdata1,
  input  logic [31:0] id_ex_rdata2,
  input  logic [31:0] id_ex_imm,
  input  logic        id_ex_use_imm,

  input  logic        ex_mem_valid,
  input  logic [4:0]  ex_mem_rd,
  input  logic        ex_mem_rd_we,
  input  rv32i_pkg::wbsrc_t ex_mem_wb_src,
  input  logic [31:0] ex_mem_alu_result,

  input  logic        mem_wb_valid,
  input  logic [4:0]  mem_wb_rd,
  input  logic        mem_wb_rd_we,
  input  logic [31:0] mem_wb_wb_data,

  output logic [31:0] ex_opA,
  output logic [31:0] ex_opB,
  output logic [31:0] rs2_fwd
);

  import rv32i_pkg::*;

  logic [31:0] ex_opA_raw;
  logic [31:0] ex_opB_raw;
  logic [31:0] rs2_raw;

  always_comb begin
    ex_opA_raw = id_ex_rdata1;
    rs2_raw    = id_ex_rdata2;
    ex_opB_raw = (id_ex_use_imm) ? id_ex_imm : rs2_raw;

    ex_opA  = ex_opA_raw;
    ex_opB  = ex_opB_raw;
    rs2_fwd = rs2_raw;

    // Forward rs1 -> opA.
    // EX/MEM forwarding is only allowed for ALU results because load data
    // is not available from EX/MEM in this single-cycle MEM design.
    if (id_ex_uses_rs1 && (id_ex_rs1 != 5'd0)) begin
      if (ex_mem_valid &&
          ex_mem_rd_we &&
          (ex_mem_wb_src == WB_ALU) &&
          (ex_mem_rd == id_ex_rs1)) begin
        ex_opA = ex_mem_alu_result;
      end else if (mem_wb_valid &&
                   mem_wb_rd_we &&
                   (mem_wb_rd == id_ex_rs1)) begin
        ex_opA = mem_wb_wb_data;
      end
    end

    // Forward rs2 value.
    // Used for:
    // - R-type ALU opB
    // - branch compare operand
    // - store data
    if (id_ex_uses_rs2 && (id_ex_rs2 != 5'd0)) begin
      if (ex_mem_valid &&
          ex_mem_rd_we &&
          (ex_mem_wb_src == WB_ALU) &&
          (ex_mem_rd == id_ex_rs2)) begin
        rs2_fwd = ex_mem_alu_result;
      end else if (mem_wb_valid &&
                   mem_wb_rd_we &&
                   (mem_wb_rd == id_ex_rs2)) begin
        rs2_fwd = mem_wb_wb_data;
      end
    end

    // If ALU source B is a register operand, use forwarded rs2.
    // If ALU source B is an immediate, keep immediate selected.
    if (!id_ex_use_imm) begin
      ex_opB = rs2_fwd;
    end
  end

endmodule