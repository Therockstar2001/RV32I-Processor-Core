// ============================================================
// RV32I 5-stage pipeline
// ============================================================

module riscv_core_5stage #(
  parameter int IMEM_WORDS = 256,
  parameter int DMEM_WORDS = 256
)(
  input  logic        clk,
  input  logic        rst_n,

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
  // IMEM
  // ---------------------------
  logic [31:0] imem [0:IMEM_WORDS-1];

  initial begin
    for (int i = 0; i < IMEM_WORDS; i++) imem[i] = RV32I_NOP;

`ifndef TB_LOADS_PROGRAM
    // Program for Phase 4:
    imem[0]  = 32'h0100_0093; // addi x1,x0,16
    imem[1]  = 32'h0010_0113; // addi x2,x0,1
    imem[2]  = 32'h0010_0193; // addi x3,x0,1
    imem[3]  = 32'h0031_0463; // beq  x2,x3,+8
    imem[4]  = 32'h0630_0213; // addi x4,x0,99   (skip)
    imem[5]  = 32'h02A0_0213; // addi x4,x0,42
    imem[6]  = 32'h0040_A023; // sw   x4,0(x1)
    imem[7]  = 32'h0000_A283; // lw   x5,0(x1)
    imem[8]  = 32'h0300_0513; // addi x10,x0,48
    imem[9]  = 32'h0005_03E7; // jalr x7,0(x10)
    imem[10] = 32'h04D0_0313; // addi x6,x0,77   (skip)
    imem[11] = 32'h0000_0013; // nop
    imem[12] = 32'h0580_0313; // addi x6,x0,88
    imem[13] = 32'h0080_006F; // jal  x0,+8      (skip next)
    imem[14] = 32'h0013_0313; // addi x6,x6,1    (skip)
    imem[15] = 32'h0000_0013; // nop
`endif
  end

  // ---------------------------
  // DMEM (word-addressed)
  // ---------------------------
  logic [31:0] dmem [0:DMEM_WORDS-1];

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
  logic [31:0] rf[0:31];
  logic [31:0] rf_rdata1, rf_rdata2;

  always_comb begin
    rf_rdata1 = (rs1(if_id_q.instr) == 0) ? 32'd0 : rf[rs1(if_id_q.instr)];
    rf_rdata2 = (rs2(if_id_q.instr) == 0) ? 32'd0 : rf[rs2(if_id_q.instr)];
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int i=0; i<32; i++) rf[i] <= 32'd0;
    end else begin
      if (mem_wb_d.valid && mem_wb_d.rd_we && (mem_wb_d.rd != 5'd0)) begin
        rf[mem_wb_d.rd] <= mem_wb_d.wb_data;
      end
      rf[0] <= 32'd0;
    end
  end

  // ---------------------------
  // Load-use stall detection (1 bubble)
  // ---------------------------
  logic stall;
  logic [31:0] id_instr;
  logic id_uses_rs2;

  always_comb begin
    id_instr    = if_id_q.instr;

    id_uses_rs2 = (opcode(id_instr) == 7'b0110011) ||
                  (opcode(id_instr) == 7'b0100011) ||
                  (opcode(id_instr) == 7'b1100011);

    stall = 1'b0;
    if (id_ex_q.valid && id_ex_q.mem_read && (id_ex_q.rd != 5'd0)) begin
      if ((id_ex_q.rd == rs1(id_instr)) ||
          (id_uses_rs2 && (id_ex_q.rd == rs2(id_instr)))) begin
        stall = 1'b1;
      end
    end
  end

  // ---------------------------
  // EX-stage redirect (branches/jumps)
  // ---------------------------
  logic redirect;
  logic [31:0] redirect_pc;

  // ---------------------------
  // PC + IF fetch
  // ---------------------------
  logic [31:0] pc_q, pc_d;
  logic [31:0] if_instr;

  always_comb pc_d = pc_q + 32'd4;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) pc_q <= 32'd0;
    else if (redirect) pc_q <= redirect_pc;
    else if (!stall)   pc_q <= pc_d;
  end

  always_comb begin
    int unsigned idx;
    idx = pc_q[31:2];
    if (idx < IMEM_WORDS) if_instr = imem[idx];
    else                  if_instr = RV32I_NOP;
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      if_id_q <= '0;
    end else if (redirect) begin
      if_id_q.valid <= 1'b0;
      if_id_q.pc    <= 32'd0;
      if_id_q.instr <= RV32I_NOP;
    end else if (!stall) begin
      if_id_q.valid <= 1'b1;
      if_id_q.pc    <= pc_q;
      if_id_q.instr <= if_instr;
    end
  end

  // ---------------------------
  // Decode -> ID/EX
  // ---------------------------
  always_comb begin
    id_ex_d = '0;

    if (stall || redirect) begin
      id_ex_d.valid = 1'b0;
    end else begin
      id_ex_d.valid  = if_id_q.valid;
      id_ex_d.pc     = if_id_q.pc;
      id_ex_d.instr  = if_id_q.instr;

      id_ex_d.rs1    = rs1(if_id_q.instr);
      id_ex_d.rs2    = rs2(if_id_q.instr);
      id_ex_d.rd     = rd (if_id_q.instr);

      id_ex_d.rdata1 = rf_rdata1;
      id_ex_d.rdata2 = rf_rdata2;

      id_ex_d.rd_we      = 1'b0;
      id_ex_d.use_imm    = 1'b0;
      id_ex_d.alu_op     = ALU_ADD;
      id_ex_d.imm        = 32'd0;
      id_ex_d.mem_read   = 1'b0;
      id_ex_d.mem_write  = 1'b0;
      id_ex_d.wb_src     = WB_ALU;

      id_ex_d.is_branch  = 1'b0;
      id_ex_d.br_funct3  = 3'b000;
      id_ex_d.is_jal     = 1'b0;
      id_ex_d.is_jalr    = 1'b0;

      
      id_ex_d.uses_rs1   = 1'b1;
      id_ex_d.uses_rs2   = 1'b0;

      unique case (opcode(if_id_q.instr))
        7'b0010011: begin
          id_ex_d.use_imm = 1'b1;
          id_ex_d.imm     = imm_i(if_id_q.instr);
          id_ex_d.rd_we   = 1'b1;
          id_ex_d.wb_src  = WB_ALU;
          id_ex_d.uses_rs1 = 1'b1;
          id_ex_d.uses_rs2 = 1'b0;
          unique case (funct3(if_id_q.instr))
            3'b000: id_ex_d.alu_op = ALU_ADD;
            3'b111: id_ex_d.alu_op = ALU_AND;
            3'b110: id_ex_d.alu_op = ALU_OR;
            3'b100: id_ex_d.alu_op = ALU_XOR;
            default: id_ex_d.rd_we = 1'b0;
          endcase
        end

        7'b0110011: begin
          id_ex_d.rd_we   = 1'b1;
          id_ex_d.wb_src  = WB_ALU;
          id_ex_d.uses_rs1 = 1'b1;
          id_ex_d.uses_rs2 = 1'b1;
          unique case (funct3(if_id_q.instr))
            3'b000: begin
              if (funct7(if_id_q.instr) == 7'b0100000) id_ex_d.alu_op = ALU_SUB;
              else                                      id_ex_d.alu_op = ALU_ADD;
            end
            3'b111: id_ex_d.alu_op = ALU_AND;
            3'b110: id_ex_d.alu_op = ALU_OR;
            3'b100: id_ex_d.alu_op = ALU_XOR;
            default: id_ex_d.rd_we = 1'b0;
          endcase
        end

        7'b0110111: begin // LUI
          id_ex_d.use_imm = 1'b1;
          id_ex_d.imm     = imm_u(if_id_q.instr);
          id_ex_d.rd_we   = 1'b1;
          id_ex_d.wb_src  = WB_ALU;
          id_ex_d.alu_op  = ALU_ADD;
          id_ex_d.rdata1  = 32'd0;

          
          id_ex_d.rs1      = 5'd0;
          id_ex_d.rs2      = 5'd0;
          id_ex_d.uses_rs1 = 1'b0;
          id_ex_d.uses_rs2 = 1'b0;
        end

        7'b0010111: begin // AUIPC
          id_ex_d.use_imm = 1'b1;
          id_ex_d.imm     = imm_u(if_id_q.instr);
          id_ex_d.rd_we   = 1'b1;
          id_ex_d.wb_src  = WB_ALU;
          id_ex_d.alu_op  = ALU_ADD;
          id_ex_d.rdata1  = if_id_q.pc;

          
          id_ex_d.rs1      = 5'd0;
          id_ex_d.rs2      = 5'd0;
          id_ex_d.uses_rs1 = 1'b0;
          id_ex_d.uses_rs2 = 1'b0;
        end

        7'b0000011: begin // LW
          if (funct3(if_id_q.instr) == 3'b010) begin
            id_ex_d.use_imm  = 1'b1;
            id_ex_d.imm      = imm_i(if_id_q.instr);
            id_ex_d.alu_op   = ALU_ADD;
            id_ex_d.mem_read = 1'b1;
            id_ex_d.rd_we    = 1'b1;
            id_ex_d.wb_src   = WB_MEM;
            id_ex_d.uses_rs1 = 1'b1;
            id_ex_d.uses_rs2 = 1'b0;
          end
        end

        7'b0100011: begin // SW
          if (funct3(if_id_q.instr) == 3'b010) begin
            id_ex_d.use_imm   = 1'b1;
            id_ex_d.imm       = imm_s(if_id_q.instr);
            id_ex_d.alu_op    = ALU_ADD;
            id_ex_d.mem_write = 1'b1;
            id_ex_d.uses_rs1  = 1'b1;
            id_ex_d.uses_rs2  = 1'b1;
          end
        end

        7'b1100011: begin // BR
          if ((funct3(if_id_q.instr) == 3'b000) || (funct3(if_id_q.instr) == 3'b001)) begin
            id_ex_d.is_branch = 1'b1;
            id_ex_d.br_funct3 = funct3(if_id_q.instr);
            id_ex_d.uses_rs1  = 1'b1;
            id_ex_d.uses_rs2  = 1'b1;
          end
        end

        7'b1101111: begin // JAL
          id_ex_d.is_jal   = 1'b1;
          id_ex_d.rd_we    = 1'b1;
          id_ex_d.wb_src   = WB_PC4;

          
          id_ex_d.rs1      = 5'd0;
          id_ex_d.rs2      = 5'd0;
          id_ex_d.uses_rs1 = 1'b0;
          id_ex_d.uses_rs2 = 1'b0;
        end

        7'b1100111: begin // JALR
          if (funct3(if_id_q.instr) == 3'b000) begin
            id_ex_d.is_jalr  = 1'b1;
            id_ex_d.use_imm  = 1'b1;
            id_ex_d.imm      = imm_i(if_id_q.instr);
            id_ex_d.rd_we    = 1'b1;
            id_ex_d.wb_src   = WB_PC4;
            id_ex_d.uses_rs1 = 1'b1;
            id_ex_d.uses_rs2 = 1'b0;
          end
        end
        default: ;
      endcase

      if (id_ex_d.rd == 5'd0) id_ex_d.rd_we = 1'b0;
    end
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) id_ex_q <= '0;
    else        id_ex_q <= id_ex_d;
  end

  
  logic [31:0] ex_opA_raw, ex_opB_raw, rs2_raw;
  logic [31:0] ex_opA, ex_opB, rs2_fwd;

  always_comb begin
    ex_opA_raw = id_ex_q.rdata1;
    rs2_raw    = id_ex_q.rdata2;

    ex_opB_raw = (id_ex_q.use_imm) ? id_ex_q.imm : rs2_raw;

    ex_opA   = ex_opA_raw;
    ex_opB   = ex_opB_raw;
    rs2_fwd  = rs2_raw;

    if (id_ex_q.uses_rs1 && (id_ex_q.rs1 != 0)) begin
      if (ex_mem_q.valid && ex_mem_q.rd_we && (ex_mem_q.wb_src == WB_ALU) && (ex_mem_q.rd == id_ex_q.rs1)) begin
        ex_opA = ex_mem_q.alu_result;
      end else if (mem_wb_q.valid && mem_wb_q.rd_we && (mem_wb_q.rd == id_ex_q.rs1)) begin
        ex_opA = mem_wb_q.wb_data;
      end
    end

    if (id_ex_q.uses_rs2 && (id_ex_q.rs2 != 0)) begin
      if (ex_mem_q.valid && ex_mem_q.rd_we && (ex_mem_q.wb_src == WB_ALU) && (ex_mem_q.rd == id_ex_q.rs2)) begin
        rs2_fwd = ex_mem_q.alu_result;
      end else if (mem_wb_q.valid && mem_wb_q.rd_we && (mem_wb_q.rd == id_ex_q.rs2)) begin
        rs2_fwd = mem_wb_q.wb_data;
      end
    end

    if (!id_ex_q.use_imm) begin
      ex_opB = rs2_fwd;
    end
  end

  // ---------------------------
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

  always_comb begin
    unique case (id_ex_q.alu_op)
      ALU_ADD: alu_y = ex_opA + ex_opB;
      ALU_SUB: alu_y = ex_opA - ex_opB;
      ALU_AND: alu_y = ex_opA & ex_opB;
      ALU_OR : alu_y = ex_opA | ex_opB;
      ALU_XOR: alu_y = ex_opA ^ ex_opB;
      default: alu_y = ex_opA + ex_opB;
    endcase
  end

  // ---------------------------
  // EX/MEM
  // ---------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      ex_mem_q <= '0;
    end else begin
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
  // MEM stage
  // ---------------------------
  logic [31:0] mem_rdata;
  always_comb begin
    int unsigned didx;
    didx = ex_mem_q.alu_result[31:2];
    if (didx < DMEM_WORDS) mem_rdata = dmem[didx];
    else                   mem_rdata = 32'd0;
  end

  always_ff @(posedge clk) begin
    if (rst_n) begin
      if (ex_mem_q.valid && ex_mem_q.mem_write) begin
        int unsigned didx;
        didx = ex_mem_q.alu_result[31:2];
        if (didx < DMEM_WORDS) dmem[didx] <= ex_mem_q.store_data;
      end
    end
  end

  // ---------------------------
  // MEM/WB
  // ---------------------------
  always_comb begin
    mem_wb_d = '0;
    mem_wb_d.valid = ex_mem_q.valid;
    mem_wb_d.pc    = ex_mem_q.pc;
    mem_wb_d.instr = ex_mem_q.instr;
    mem_wb_d.rd    = ex_mem_q.rd;
    mem_wb_d.rd_we = ex_mem_q.rd_we;

    unique case (ex_mem_q.wb_src)
      WB_MEM: mem_wb_d.wb_data = mem_rdata;
      WB_PC4: mem_wb_d.wb_data = ex_mem_q.pc_plus4;
      default: mem_wb_d.wb_data = ex_mem_q.alu_result;
    endcase
  end

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
