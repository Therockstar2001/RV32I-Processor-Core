// ============================================================
// RV32I 5-stage pipeline
// ============================================================

module rv32i_core #(
  parameter int IMEM_WORDS = 256,
  parameter int DMEM_WORDS = 256
)(
  input  logic        clk,
  input  logic        rst_n,

  // Instruction memory interface
  output logic        imem_req_valid,
  input  logic        imem_req_ready,
  output logic [31:0] imem_req_addr,

  input  logic        imem_rsp_valid,
  input  logic [31:0] imem_rsp_rdata,
  
  // Data memory interface
  output logic        dmem_req_valid,
  input  logic        dmem_req_ready,
  output logic        dmem_req_we,
  output logic [31:0] dmem_req_addr,
  output logic [31:0] dmem_req_wdata,
  output logic [3:0]  dmem_req_wstrb,

  input  logic        dmem_rsp_valid,
  input  logic [31:0] dmem_rsp_rdata,

  // Commit/Retire trace 
  output logic        commit_valid,
  output logic [31:0] commit_pc,
  output logic [31:0] commit_instr,
  output logic [4:0]  commit_rd,
  output logic        commit_rd_we,
  output logic [31:0] commit_rd_wdata
);

  localparam logic [31:0] RV32I_NOP = 32'h0000_0013;


  // ---------------------------
  // DMEM (word-addressed)
  // ---------------------------
  //logic [31:0] dmem [0:DMEM_WORDS-1];

  // ---------------------------
  // Helpers: fields + immediates
  // ---------------------------
  function automatic logic [6:0] opcode(input logic [31:0] ins); return ins[6:0]; endfunction
  function automatic logic [2:0] funct3(input logic [31:0] ins); return ins[14:12]; endfunction
  function automatic logic [6:0] funct7(input logic [31:0] ins); return ins[31:25]; endfunction
  function automatic logic [4:0] rd    (input logic [31:0] ins); return ins[11:7];  endfunction
  function automatic logic [4:0] rs1   (input logic [31:0] ins); return ins[19:15]; endfunction
  function automatic logic [4:0] rs2   (input logic [31:0] ins); return ins[24:20]; endfunction

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

  // ---------------------------
  // ALU ops
  // ---------------------------
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

  // ---------------------------
  // Pipeline regs
  // ---------------------------
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

  ifid_t  if_id_q;
  idex_t  id_ex_q, id_ex_d;
  exmem_t ex_mem_q;
  memwb_t mem_wb_d, mem_wb_q;

  // ---------------------------
  // Regfile
  // ---------------------------
  // ---------------------------

  logic [31:0] rf_rdata1, rf_rdata2;

  rv32i_regfile u_regfile (
    .clk    (clk),
    .rst_n  (rst_n),

    .raddr1 (rs1(if_id_q.instr)),
    .raddr2 (rs2(if_id_q.instr)),
    .rdata1 (rf_rdata1),
    .rdata2 (rf_rdata2),

    .we     (mem_wb_d.valid && mem_wb_d.rd_we),
    .waddr  (mem_wb_d.rd),
    .wdata  (mem_wb_d.wb_data)
  );

  // ---------------------------
  // Load-use stall detection (1 bubble)
  // ---------------------------
  logic stall;
  

  rv32i_hazard_unit u_hazard_unit (
    .id_ex_valid    (id_ex_q.valid),
    .id_ex_mem_read (id_ex_q.mem_read),
    .id_ex_rd       (id_ex_q.rd),

    .id_instr       (if_id_q.instr),

    .stall          (stall)
  );  

  // ---------------------------
  // EX-stage redirect (branches/jumps)
  // ---------------------------
  logic redirect;
  logic [31:0] redirect_pc;
  logic if_wait;

  // ---------------------------
  // PC + IF fetch
  // ---------------------------
  logic [31:0] pc_q, pc_d;
  logic [31:0] if_instr;

  always_comb pc_d = pc_q + 32'd4;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) pc_q <= 32'd0;
    else if (redirect) pc_q <= redirect_pc;
    else if (!stall && !mem_wait && !if_wait) pc_q <= pc_d;
  end

  always_comb begin
    imem_req_valid = 1'b1;
    imem_req_addr  = pc_q;

    if (imem_rsp_valid) begin
      if_instr = imem_rsp_rdata;
    end else begin
      if_instr = RV32I_NOP;
    end
  end
  
  assign if_wait = rst_n && imem_req_valid && !imem_rsp_valid;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      if_id_q <= '0;
    end else if (redirect) begin
      if_id_q <= '0;
    end else if (stall || mem_wait) begin
      if_id_q <= if_id_q;   // hold current decoded instruction
    end else if (if_wait) begin
      if_id_q.valid <= 1'b0;
      if_id_q.pc    <= pc_q;
      if_id_q.instr <= RV32I_NOP;
    end else begin
      if_id_q.valid <= 1'b1;
      if_id_q.pc    <= pc_q;
      if_id_q.instr <= if_instr;
    end
  end

  // ---------------------------
  // Decode -> ID/EX
  // ---------------------------
  idex_t id_ex_decoded;

  rv32i_decode u_decode (
    .instr        (if_id_q.instr),
    .pc           (if_id_q.pc),
    .rf_rdata1    (rf_rdata1),
    .rf_rdata2    (rf_rdata2),
    .id_ex_decoded(id_ex_decoded)
  );

  always_comb begin
    id_ex_d = '0;

    if (stall || redirect) begin
      id_ex_d.valid = 1'b0;
    end else begin
      id_ex_d       = id_ex_decoded;
      id_ex_d.valid = if_id_q.valid;
    end
  end
  

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)       id_ex_q <= '0;
    else if (!mem_wait) id_ex_q <= id_ex_d;
  end

  // ---------------------------
  // Forwarding
  // ---------------------------
  logic [31:0] ex_opA, ex_opB, rs2_fwd;

  rv32i_forward_unit u_forward_unit (
    .id_ex_uses_rs1   (id_ex_q.uses_rs1),
    .id_ex_uses_rs2   (id_ex_q.uses_rs2),
    .id_ex_rs1        (id_ex_q.rs1),
    .id_ex_rs2        (id_ex_q.rs2),

    .id_ex_rdata1     (id_ex_q.rdata1),
    .id_ex_rdata2     (id_ex_q.rdata2),
    .id_ex_imm        (id_ex_q.imm),
    .id_ex_use_imm    (id_ex_q.use_imm),

    .ex_mem_valid     (ex_mem_q.valid),
    .ex_mem_rd        (ex_mem_q.rd),
    .ex_mem_rd_we     (ex_mem_q.rd_we),
    .ex_mem_wb_src    (ex_mem_q.wb_src),
    .ex_mem_alu_result(ex_mem_q.alu_result),

    .mem_wb_valid     (mem_wb_q.valid),
    .mem_wb_rd        (mem_wb_q.rd),
    .mem_wb_rd_we     (mem_wb_q.rd_we),
    .mem_wb_wb_data   (mem_wb_q.wb_data),

    .ex_opA           (ex_opA),
    .ex_opB           (ex_opB),
    .rs2_fwd          (rs2_fwd)
  );
  
  //-------------------------
  // Redirect decision in EX
  // ---------------------------
  always_comb begin
    redirect    = 1'b0;
    redirect_pc = 32'd0;

    if (id_ex_q.valid) begin
      if (id_ex_q.is_jal) begin
        redirect    = 1'b1;
        redirect_pc = id_ex_q.pc + imm_j(id_ex_q.instr);
      end else if (id_ex_q.is_jalr) begin
        redirect    = 1'b1;
        redirect_pc = (ex_opA + id_ex_q.imm) & 32'hFFFF_FFFE;
      end else if (id_ex_q.is_branch) begin
        logic take;
        take = 1'b0;
        if (id_ex_q.br_funct3 == 3'b000) take = (ex_opA == rs2_fwd);
        if (id_ex_q.br_funct3 == 3'b001) take = (ex_opA != rs2_fwd);

        if (take) begin
          redirect    = 1'b1;
          redirect_pc = id_ex_q.pc + imm_b(id_ex_q.instr);
        end
      end
    end
  end

  // ---------------------------
  // ALU (EX)
  // ---------------------------
  logic [31:0] alu_y;
  
  rv32i_alu u_alu (
  	.alu_op (id_ex_q.alu_op),
  	.op_a   (ex_opA),
  	.op_b   (ex_opB),
  	.result (alu_y)
  );

  // ---------------------------
  // EX/MEM
  // ---------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      ex_mem_q <= '0;
    end else if (!mem_wait) begin
      ex_mem_q.valid      <= id_ex_q.valid;
      ex_mem_q.pc         <= id_ex_q.pc;
      ex_mem_q.instr      <= id_ex_q.instr;

      ex_mem_q.rd         <= id_ex_q.rd;
      ex_mem_q.rd_we      <= id_ex_q.rd_we;
      ex_mem_q.wb_src     <= id_ex_q.wb_src;

      ex_mem_q.mem_read   <= id_ex_q.mem_read;
      ex_mem_q.mem_write  <= id_ex_q.mem_write;

      ex_mem_q.alu_result <= alu_y;
      ex_mem_q.store_data <= rs2_fwd;
      ex_mem_q.pc_plus4   <= id_ex_q.pc + 32'd4;
    end
  end

  // ---------------------------
  // MEM stage through MTU
  // ---------------------------
  logic mem_wait;

  rv32i_mem_unit u_mem_unit (
    .clk            (clk),
    .rst_n          (rst_n),

    .ex_mem_in      (ex_mem_q),

    .dmem_req_valid (dmem_req_valid),
    .dmem_req_ready (dmem_req_ready),
    .dmem_req_we    (dmem_req_we),
    .dmem_req_addr  (dmem_req_addr),
    .dmem_req_wdata (dmem_req_wdata),
    .dmem_req_wstrb (dmem_req_wstrb),

    .dmem_rsp_valid (dmem_rsp_valid),
    .dmem_rsp_rdata (dmem_rsp_rdata),

    .mem_wait       (mem_wait),
    .mem_wb_out     (mem_wb_d)
  );
  // ---------------------------
  // MEM/WB
  // ---------------------------

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) mem_wb_q <= '0;
    else        mem_wb_q <= mem_wb_d;
  end

  // ---------------------------
  // Commit (WB)
  // ---------------------------
  always_comb begin
    commit_valid    = mem_wb_q.valid;
    commit_pc       = mem_wb_q.pc;
    commit_instr    = mem_wb_q.instr;
    commit_rd       = mem_wb_q.rd;
    commit_rd_we    = mem_wb_q.rd_we;
    commit_rd_wdata = mem_wb_q.wb_data;
  end

endmodule
