module rv32i_hazard_unit (
  input  logic        id_ex_valid,
  input  logic        id_ex_mem_read,
  input  logic [4:0]  id_ex_rd,

  input  logic [31:0] id_instr,

  output logic        stall
);

  import rv32i_pkg::*;

  logic id_uses_rs2;

  always_comb begin
    // ID-stage instruction uses rs2 for:
    // - R-type ALU ops
    // - Store
    // - Branch
    id_uses_rs2 = (opcode(id_instr) == 7'b0110011) ||
                  (opcode(id_instr) == 7'b0100011) ||
                  (opcode(id_instr) == 7'b1100011);

    stall = 1'b0;

    // Load-use hazard:
    // if the instruction in EX is a load and its rd is needed by
    // the instruction currently in ID, insert one bubble.
    if (id_ex_valid && id_ex_mem_read && (id_ex_rd != 5'd0)) begin
      if ((id_ex_rd == rs1(id_instr)) ||
          (id_uses_rs2 && (id_ex_rd == rs2(id_instr)))) begin
        stall = 1'b1;
      end
    end
  end

endmodule