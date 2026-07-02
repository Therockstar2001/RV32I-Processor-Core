module rv32i_alu
(
    input  rv32i_pkg::aluop_t alu_op,

    input  logic [31:0] op_a,
    input  logic [31:0] op_b,

    output logic [31:0] result
);

import rv32i_pkg::*;

always_comb begin
    unique case (alu_op)

        ALU_ADD : result = op_a + op_b;

        ALU_SUB : result = op_a - op_b;

        ALU_AND : result = op_a & op_b;

        ALU_OR  : result = op_a | op_b;

        ALU_XOR : result = op_a ^ op_b;

        default : result = op_a + op_b;

    endcase
end

endmodule