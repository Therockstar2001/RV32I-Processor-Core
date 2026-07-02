package rv32i_pkg;

  localparam logic [31:0] RV32I_NOP = 32'h0000_0013;

  typedef enum logic [3:0] {
    ALU_ADD  = 4'd0,
    ALU_SUB  = 4'd1,
    ALU_AND  = 4'd2,
    ALU_OR   = 4'd3,
    ALU_XOR  = 4'd4
  } aluop_t;

  typedef enum logic [1:0] {
    WB_ALU = 2'd0,
    WB_MEM = 2'd1,
    WB_PC4 = 2'd2
  } wbsrc_t;

  typedef struct packed {
    logic        valid;
    logic [31:0] pc;
    logic [31:0] instr;
  } ifid_t;

  typedef struct packed {
    logic        valid;
    logic [31:0] pc;
    logic [31:0] instr;

    logic [4:0]  rs1;
    logic [4:0]  rs2;
    logic [4:0]  rd;

    logic [31:0] rdata1;
    logic [31:0] rdata2;
    logic [31:0] imm;

    logic        use_imm;
    aluop_t      alu_op;

    logic        rd_we;
    wbsrc_t      wb_src;

    logic        mem_read;
    logic        mem_write;

    logic        is_branch;
    logic [2:0]  br_funct3;

    logic        is_jal;
    logic        is_jalr;

    logic        uses_rs1;
    logic        uses_rs2;
  } idex_t;

  typedef struct packed {
    logic        valid;
    logic [31:0] pc;
    logic [31:0] instr;

    logic [4:0]  rd;
    logic        rd_we;
    wbsrc_t      wb_src;

    logic        mem_read;
    logic        mem_write;

    logic [31:0] alu_result;
    logic [31:0] store_data;
    logic [31:0] pc_plus4;
  } exmem_t;

  typedef struct packed {
    logic        valid;
    logic [31:0] pc;
    logic [31:0] instr;

    logic [4:0]  rd;
    logic        rd_we;

    logic [31:0] wb_data;
  } memwb_t;

  function automatic logic [6:0] opcode(input logic [31:0] ins);
    return ins[6:0];
  endfunction

  function automatic logic [2:0] funct3(input logic [31:0] ins);
    return ins[14:12];
  endfunction

  function automatic logic [6:0] funct7(input logic [31:0] ins);
    return ins[31:25];
  endfunction

  function automatic logic [4:0] rd(input logic [31:0] ins);
    return ins[11:7];
  endfunction

  function automatic logic [4:0] rs1(input logic [31:0] ins);
    return ins[19:15];
  endfunction

  function automatic logic [4:0] rs2(input logic [31:0] ins);
    return ins[24:20];
  endfunction

  function automatic logic [31:0] imm_i(input logic [31:0] ins);
    return {{20{ins[31]}}, ins[31:20]};
  endfunction

  function automatic logic [31:0] imm_s(input logic [31:0] ins);
    logic [11:0] imm12;
    imm12 = {ins[31:25], ins[11:7]};
    return {{20{imm12[11]}}, imm12};
  endfunction

  function automatic logic [31:0] imm_b(input logic [31:0] ins);
    logic [12:0] imm13;
    imm13 = {ins[31], ins[7], ins[30:25], ins[11:8], 1'b0};
    return {{19{imm13[12]}}, imm13};
  endfunction

  function automatic logic [31:0] imm_u(input logic [31:0] ins);
    return {ins[31:12], 12'b0};
  endfunction

  function automatic logic [31:0] imm_j(input logic [31:0] ins);
    logic [20:0] imm21;
    imm21 = {ins[31], ins[19:12], ins[20], ins[30:21], 1'b0};
    return {{11{imm21[20]}}, imm21};
  endfunction

endpackage