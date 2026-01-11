`timescale 1ns/1ps
`include "uvm_macros.svh"
import uvm_pkg::*;


`define IMEM_WORD(i) dut.imem[i]
localparam int IMEM_WORDS = 256;  // 32-bit words

// ============================================================
// Interface
// ============================================================
interface core_if;
  logic clk;
  logic rst_n;

  logic        commit_valid;
  logic [31:0] commit_pc;
  logic [31:0] commit_instr;
  logic [4:0]  commit_rd;
  logic        commit_rd_we;
  logic [31:0] commit_rd_wdata;
endinterface

// ============================================================
// Transaction
// ============================================================
class commit_txn extends uvm_sequence_item;
  bit [31:0] pc, instr, rd_wdata;
  bit [4:0]  rd;
  bit        rd_we;
  `uvm_object_utils(commit_txn)
  function new(string name="commit_txn"); super.new(name); endfunction
endclass

// ============================================================
// Field helpers
// ============================================================
function automatic bit [6:0] opcode(bit [31:0] ins); return ins[6:0]; endfunction
function automatic bit [2:0] funct3(bit [31:0] ins); return ins[14:12]; endfunction
function automatic bit [6:0] funct7(bit [31:0] ins); return ins[31:25]; endfunction
function automatic bit [4:0] rd_f  (bit [31:0] ins); return ins[11:7]; endfunction
function automatic bit [4:0] rs1_f (bit [31:0] ins); return ins[19:15]; endfunction
function automatic bit [4:0] rs2_f (bit [31:0] ins); return ins[24:20]; endfunction

function automatic bit [31:0] imm_i(bit [31:0] ins);
  return {{20{ins[31]}}, ins[31:20]};
endfunction
function automatic bit [31:0] imm_s(bit [31:0] ins);
  bit [11:0] imm12;
  imm12 = {ins[31:25], ins[11:7]};
  return {{20{imm12[11]}}, imm12};
endfunction
function automatic bit [31:0] imm_b(bit [31:0] ins);
  bit [12:0] imm13;
  imm13 = {ins[31], ins[7], ins[30:25], ins[11:8], 1'b0};
  return {{19{imm13[12]}}, imm13};
endfunction
function automatic bit [31:0] imm_u(bit [31:0] ins);
  return {ins[31:12], 12'b0};
endfunction
function automatic bit [31:0] imm_j(bit [31:0] ins);
  bit [20:0] imm21;
  imm21 = {ins[31], ins[19:12], ins[20], ins[30:21], 1'b0};
  return {{11{imm21[20]}}, imm21};
endfunction

// ============================================================
// Monitor
// ============================================================
class commit_monitor extends uvm_component;
  `uvm_component_utils(commit_monitor)
  virtual core_if vif;
  uvm_analysis_port#(commit_txn) ap;

  function new(string name="commit_monitor", uvm_component parent=null);
    super.new(name, parent);
    ap = new("ap", this);
  endfunction

  function void build_phase(uvm_phase phase);
    if (!uvm_config_db#(virtual core_if)::get(this, "", "vif", vif))
      `uvm_fatal("NOVIF", "commit_monitor: vif not set")
  endfunction

  task run_phase(uvm_phase phase);
    commit_txn t;
    forever begin
      @(posedge vif.clk);
      if (vif.rst_n && vif.commit_valid) begin
        t = commit_txn::type_id::create("t");
        t.pc       = vif.commit_pc;
        t.instr    = vif.commit_instr;
        t.rd       = vif.commit_rd;
        t.rd_we    = vif.commit_rd_we;
        t.rd_wdata = vif.commit_rd_wdata;
        ap.write(t);
      end
    end
  endtask
endclass

// ============================================================
// Coverage collector (commit-based)
// ============================================================
class rv32i_cov extends uvm_component;
  `uvm_component_utils(rv32i_cov)
  uvm_analysis_imp#(commit_txn, rv32i_cov) imp;

  bit [6:0]  op;
  bit [2:0]  f3;
  bit [6:0]  f7;
  bit        is_load, is_store, is_branch, is_jal, is_jalr;
  bit        br_taken;
  bit        looks_like_flush;
  bit        looks_like_loaduse;
  bit        is_rd_we;

  bit        prev_was_lw;
  bit [4:0]  prev_lw_rd;

  bit [31:0] regs_m[0:31];

  covergroup cg_commit;
    option.per_instance = 1;

    cp_opcode : coverpoint op {
      bins OPIMM  = {7'b0010011};
      bins OP     = {7'b0110011};
      bins LOAD   = {7'b0000011};
      bins STORE  = {7'b0100011};
      bins BRANCH = {7'b1100011};
      bins JAL    = {7'b1101111};
      bins JALR   = {7'b1100111};
      bins LUI    = {7'b0110111};
      bins AUIPC  = {7'b0010111};
      bins OTHER  = default;
    }

    cp_branch_taken : coverpoint br_taken iff (is_branch) {
      bins TAKEN     = {1};
      bins NOT_TAKEN = {0};
    }

    cp_jump : coverpoint {is_jal, is_jalr} iff (is_jal || is_jalr) {
      bins JAL_ONLY  = {2'b10};
      bins JALR_ONLY = {2'b01};
    }

    cp_rd_we   : coverpoint is_rd_we { bins WE0 = {0}; bins WE1 = {1}; }
    cp_flush   : coverpoint looks_like_flush { bins SEEN = {1}; bins NONE = {0}; }
    cp_loaduse : coverpoint looks_like_loaduse { bins SEEN = {1}; bins NONE = {0}; }

    x_branch_flush : cross cp_branch_taken, cp_flush;
    x_jump_flush   : cross cp_jump, cp_flush;
  endgroup

  function new(string name="rv32i_cov", uvm_component parent=null);
    super.new(name, parent);
    imp = new("imp", this);
    cg_commit = new();
  endfunction

  function void build_phase(uvm_phase phase);
    prev_was_lw = 0;
    prev_lw_rd  = 0;
    for (int i=0; i<32; i++) regs_m[i] = 32'd0;
    regs_m[0] = 32'd0;
  endfunction

  function automatic bit instr_uses_rs1(bit [31:0] ins);
    unique case (opcode(ins))
      7'b0110111: return 0; // LUI
      7'b1101111: return 0; // JAL
      default:    return 1;
    endcase
  endfunction

  function automatic bit instr_uses_rs2(bit [31:0] ins);
    unique case (opcode(ins))
      7'b0110011: return 1; // OP
      7'b0100011: return 1; // STORE
      7'b1100011: return 1; // BRANCH
      default:    return 0;
    endcase
  endfunction

  function void write(commit_txn t);
    bit [31:0] ins;
    bit [4:0]  rs1, rs2;
    bit [31:0] rs1v, rs2v;

    ins = t.instr;

    op = opcode(ins);
    f3 = funct3(ins);
    f7 = funct7(ins);

    is_load   = (op == 7'b0000011);
    is_store  = (op == 7'b0100011);
    is_branch = (op == 7'b1100011);
    is_jal    = (op == 7'b1101111);
    is_jalr   = (op == 7'b1100111);

    is_rd_we  = t.rd_we;

    rs1  = rs1_f(ins);
    rs2  = rs2_f(ins);
    rs1v = (rs1==0) ? 32'd0 : regs_m[rs1];
    rs2v = (rs2==0) ? 32'd0 : regs_m[rs2];

    // branch direction (model-side)
    br_taken = 1'b0;
    if (is_branch) begin
      if (f3==3'b000) br_taken = (rs1v == rs2v);
      if (f3==3'b001) br_taken = (rs1v != rs2v);
    end

    // flush event definition for this core:
    looks_like_flush = 1'b0;
    if (is_jal || is_jalr) looks_like_flush = 1'b1;
    else if (is_branch)    looks_like_flush = br_taken;

    // load-use heuristic from commit stream
    looks_like_loaduse = 1'b0;
    if (prev_was_lw && (prev_lw_rd != 0)) begin
      if ((instr_uses_rs1(ins) && (rs1 == prev_lw_rd)) ||
          (instr_uses_rs2(ins) && (rs2 == prev_lw_rd))) begin
        looks_like_loaduse = 1'b1;
      end
    end
    prev_was_lw = is_load && (f3==3'b010);
    prev_lw_rd  = rd_f(ins);

    // update model regfile for future branch decisions
    if (t.rd_we && (t.rd != 0)) regs_m[t.rd] = t.rd_wdata;
    regs_m[0] = 32'd0;

    cg_commit.sample();
  endfunction

  function void report_phase(uvm_phase phase);
    real cov;
    cov = cg_commit.get_coverage();
    `uvm_info("COV_SUMMARY", $sformatf("Commit-based functional coverage = %0.2f%%", cov), UVM_LOW)
    if (cov < 75.0)
      `uvm_warning("COV_LOW", "Coverage < 75%. Increase run length / strengthen stimulus.")
  endfunction
endclass

// ============================================================
// Scoreboard 
// ============================================================
class rv32i_sb extends uvm_component;
  `uvm_component_utils(rv32i_sb)
  uvm_analysis_imp#(commit_txn, rv32i_sb) imp;

  bit [31:0] regs[0:31];
  bit [31:0] mem [0:255];

  bit first_seen;
  bit [31:0] expected_pc;

  int commits, errors;

  function new(string name="rv32i_sb", uvm_component parent=null);
    super.new(name, parent); imp = new("imp", this);
  endfunction

  function void build_phase(uvm_phase phase);
    for (int i=0; i<32; i++) regs[i] = 0;
    for (int j=0; j<256; j++) mem[j]  = 0;
    regs[0] = 0;
    first_seen = 0;
    expected_pc = 0;
  endfunction

  function automatic void step_model(
      commit_txn t,
      output bit exp_we,
      output bit [4:0] exp_rd,
      output bit [31:0] exp_wdata,
      output bit [31:0] next_pc
  );
    bit [31:0] ins, a, b, imm, res, addr;
    bit [4:0]  rs1, rs2, rd;
    int unsigned midx;

    exp_we = 0; exp_rd = 0; exp_wdata = 0;
    next_pc = t.pc + 32'd4;

    ins = t.instr;
    rs1 = rs1_f(ins);
    rs2 = rs2_f(ins);
    rd  = rd_f(ins);

    a = (rs1==0) ? 0 : regs[rs1];
    b = (rs2==0) ? 0 : regs[rs2];

    unique case (opcode(ins))
      7'b0010011: begin
        imm    = imm_i(ins);
        exp_we = (rd != 0);
        exp_rd = rd;
        unique case (funct3(ins))
          3'b000: res = a + imm;
          3'b111: res = a & imm;
          3'b110: res = a | imm;
          3'b100: res = a ^ imm;
          default: begin exp_we=0; res=0; end
        endcase
        exp_wdata = res;
      end

      7'b0110011: begin
        exp_we = (rd != 0);
        exp_rd = rd;
        unique case (funct3(ins))
          3'b000: res = (funct7(ins)==7'b0100000) ? (a - b) : (a + b);
          3'b111: res = a & b;
          3'b110: res = a | b;
          3'b100: res = a ^ b;
          default: begin exp_we=0; res=0; end
        endcase
        exp_wdata = res;
      end

      7'b0110111: begin
        exp_we    = (rd != 0);
        exp_rd    = rd;
        exp_wdata = imm_u(ins);
      end

      7'b0010111: begin
        exp_we    = (rd != 0);
        exp_rd    = rd;
        exp_wdata = t.pc + imm_u(ins);
      end

      7'b0000011: begin
        if (funct3(ins)==3'b010) begin
          addr = a + imm_i(ins);
          midx = addr[31:2];
          exp_we    = (rd != 0);
          exp_rd    = rd;
          exp_wdata = (midx < 256) ? mem[midx] : 32'd0;
        end
      end

      7'b0100011: begin
        if (funct3(ins)==3'b010) begin
          addr = a + imm_s(ins);
          midx = addr[31:2];
          if (midx < 256) mem[midx] = b;
          exp_we = 0;
        end
      end

      7'b1100011: begin
        bit take;
        take = 0;
        if (funct3(ins)==3'b000) take = (a == b);
        if (funct3(ins)==3'b001) take = (a != b);
        if (take) next_pc = t.pc + imm_b(ins);
      end

      7'b1101111: begin
        exp_we    = (rd != 0);
        exp_rd    = rd;
        exp_wdata = t.pc + 32'd4;
        next_pc   = t.pc + imm_j(ins);
      end

      7'b1100111: begin
        if (funct3(ins)==3'b000) begin
          exp_we    = (rd != 0);
          exp_rd    = rd;
          exp_wdata = t.pc + 32'd4;
          next_pc   = (a + imm_i(ins)) & 32'hFFFF_FFFE;
        end
      end

      default: exp_we = 0;
    endcase
  endfunction

  function void write(commit_txn t);
    bit exp_we;
    bit [4:0] exp_rd;
    bit [31:0] exp_wdata;
    bit [31:0] next_pc;

    commits++;

    if (!first_seen) begin
      first_seen = 1;
      expected_pc = t.pc;
    end

    if (t.pc !== expected_pc) begin
      errors++;
      `uvm_error("PC_MISMATCH", $sformatf("PC got=0x%08h exp=0x%08h instr=0x%08h",
                                         t.pc, expected_pc, t.instr))
      expected_pc = t.pc;
    end

    step_model(t, exp_we, exp_rd, exp_wdata, next_pc);

    if (t.rd_we !== exp_we) begin
      errors++;
      `uvm_error("WE_MISMATCH",
        $sformatf("rd_we mismatch PC=0x%08h instr=0x%08h got=%0d exp=%0d",
                  t.pc, t.instr, t.rd_we, exp_we))
    end

    if (exp_we) begin
      if (t.rd !== exp_rd) begin
        errors++;
        `uvm_error("RD_MISMATCH",
          $sformatf("rd mismatch PC=0x%08h got=%0d exp=%0d", t.pc, t.rd, exp_rd))
      end
      if (t.rd_wdata !== exp_wdata) begin
        errors++;
        `uvm_error("WDATA_MISMATCH",
          $sformatf("wdata mismatch PC=0x%08h rd=x%0d got=0x%08h exp=0x%08h instr=0x%08h",
                    t.pc, exp_rd, t.rd_wdata, exp_wdata, t.instr))
      end

      regs[exp_rd] = exp_wdata;
      regs[0] = 0;
    end

    expected_pc = next_pc;
  endfunction

  function void report_phase(uvm_phase phase);
    `uvm_info("PH_SUMMARY", $sformatf("Commits=%0d Errors=%0d", commits, errors), UVM_LOW)
    if (errors == 0)
      `uvm_info("PH_PASS", "PHASE PASS: scoreboard ok; program executed; coverage collected.", UVM_NONE)
    else
      `uvm_fatal("PH_FAIL", "PHASE FAIL: scoreboard errors.")
  endfunction
endclass

// ============================================================
// Env
// ============================================================
class ph5_env extends uvm_env;
  `uvm_component_utils(ph5_env)
  commit_monitor mon;
  rv32i_sb       sb;
  rv32i_cov      cov;

  function new(string name="ph5_env", uvm_component parent=null); super.new(name,parent); endfunction
  function void build_phase(uvm_phase phase);
    mon = commit_monitor::type_id::create("mon", this);
    sb  = rv32i_sb      ::type_id::create("sb",  this);
    cov = rv32i_cov     ::type_id::create("cov", this);
  endfunction

  function void connect_phase(uvm_phase phase);
    mon.ap.connect(sb.imp);
    mon.ap.connect(cov.imp);
  endfunction
endclass

// ============================================================
// Test
// ============================================================
class ph5a_test extends uvm_test;
  `uvm_component_utils(ph5a_test)
  ph5_env env;
  virtual core_if vif;

  function new(string name="ph5a_test", uvm_component parent=null);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    env = ph5_env::type_id::create("env", this);
    if (!uvm_config_db#(virtual core_if)::get(this, "", "vif", vif))
      `uvm_fatal("NOVIF","vif not set")
  endfunction

  task run_phase(uvm_phase phase);
    phase.raise_objection(this);
    
    repeat (60000) @(posedge vif.clk);
    phase.drop_objection(this);
  endtask
endclass

// ============================================================
// Top TB
// ============================================================
module tb_top;
  core_if cif();

  initial cif.clk = 0;
  always #5 cif.clk = ~cif.clk;

  riscv_core_5stage dut (
    .clk             (cif.clk),
    .rst_n           (cif.rst_n),
    .commit_valid    (cif.commit_valid),
    .commit_pc       (cif.commit_pc),
    .commit_instr    (cif.commit_instr),
    .commit_rd       (cif.commit_rd),
    .commit_rd_we    (cif.commit_rd_we),
    .commit_rd_wdata (cif.commit_rd_wdata)
  );

`ifdef TB_LOADS_PROGRAM
  
  initial begin : GEN_PROGRAM
    int i;
    int idx;
    #1; // wait for DUT initial block to finish clearing imem
    idx = 0;

    // Seed registers for branches and memory ops
    `IMEM_WORD(idx++) = {12'd7,  5'd0, 3'b000, 5'd20, 7'b0010011}; // addi x20,x0,7
    `IMEM_WORD(idx++) = {12'd7,  5'd0, 3'b000, 5'd21, 7'b0010011}; // addi x21,x0,7  (equal)
    `IMEM_WORD(idx++) = {12'd8,  5'd0, 3'b000, 5'd22, 7'b0010011}; // addi x22,x0,8  (not equal)
    `IMEM_WORD(idx++) = {12'd0,  5'd0, 3'b000, 5'd1,  7'b0010011}; // addi x1,x0,0  (base=0)
    `IMEM_WORD(idx++) = {12'd55, 5'd0, 3'b000, 5'd2,  7'b0010011}; // addi x2,x0,55 (data)

    // Force LUI + AUIPC early
    `IMEM_WORD(idx++) = {20'h00012, 5'd13, 7'b0110111}; // LUI   x13,0x00012
    `IMEM_WORD(idx++) = {20'h00034, 5'd14, 7'b0010111}; // AUIPC x14,0x00034

    // Main body
    for (i = idx; i < IMEM_WORDS-8; i++) begin

      // Every 20 instr: store->load->use (forces LOAD/STORE + load-use)
      if ((i % 20) == 0 && (i+3) < (IMEM_WORDS-8)) begin
        // sw x2,0(x1)
        `IMEM_WORD(i)   = {7'd0, 5'd2, 5'd1, 3'b010, 5'd0, 7'b0100011};
        // lw x3,0(x1)
        `IMEM_WORD(i+1) = {12'd0, 5'd1, 3'b010, 5'd3, 7'b0000011};
        // addi x4,x3,1  (load-use)
        `IMEM_WORD(i+2) = {12'd1, 5'd3, 3'b000, 5'd4, 7'b0010011};
        // andi x5,x4,3  (OPIMM)
        `IMEM_WORD(i+3) = {12'd3, 5'd4, 3'b111, 5'd5, 7'b0010011};
        i = i + 3;
        continue;
      end

      // Every 20 instr + 5: BEQ taken (x20==x21), offset +8
      if ((i % 20) == 5 && (i+2) < (IMEM_WORDS-8)) begin
        `IMEM_WORD(i)   = {1'b0, 6'b000000, 5'd21, 5'd20, 3'b000, 4'b0100, 1'b0, 7'b1100011};
        `IMEM_WORD(i+1) = {12'd111,5'd0,3'b000,5'd6,7'b0010011}; 
        `IMEM_WORD(i+2) = {12'd2,  5'd0,3'b000,5'd6,7'b0010011}; 
        i = i + 2;
        continue;
      end

      // Every 20 instr + 9: BEQ not taken (x20!=x22), offset +8
      if ((i % 20) == 9 && (i+2) < (IMEM_WORDS-8)) begin
        `IMEM_WORD(i)   = {1'b0, 6'b000000, 5'd22, 5'd20, 3'b000, 4'b0100, 1'b0, 7'b1100011};
        `IMEM_WORD(i+1) = {12'd3,  5'd0,3'b000,5'd7,7'b0010011}; 
        `IMEM_WORD(i+2) = {12'd4,  5'd0,3'b000,5'd8,7'b0010011}; 
        i = i + 2;
        continue;
      end

      // Every 20 instr + 12: JAL x0, +4 
      if ((i % 20) == 12) begin
        // imm=4 => imm[10:1]=2, others 0
        `IMEM_WORD(i) = {1'b0, 10'd2, 1'b0, 8'd0, 5'd0, 7'b1101111};
        continue;
      end

      
      if ((i % 20) == 15) begin
        `IMEM_WORD(i) = 32'h0000_0013; 
        continue;
      end

      
      if ((i % 2) == 0) begin
        // add x10,x20,x22
        `IMEM_WORD(i) = {7'b0000000, 5'd22, 5'd20, 3'b000, 5'd10, 7'b0110011};
      end else begin
        // xori x11,x10,1
        `IMEM_WORD(i) = {12'd1, 5'd10, 3'b100, 5'd11, 7'b0010011};
      end
    end

    
    `IMEM_WORD(IMEM_WORDS-4) = 32'h0000_0013;
    `IMEM_WORD(IMEM_WORDS-3) = 32'h0000_0013;
    `IMEM_WORD(IMEM_WORDS-2) = 32'h0000_0013;
    `IMEM_WORD(IMEM_WORDS-1) = 32'h0000_006f; // jal x0,0
  end
`endif

  initial begin
    cif.rst_n = 0;
    repeat (5) @(posedge cif.clk);
    cif.rst_n = 1;
  end

  initial begin
    uvm_config_db#(virtual core_if)::set(null, "*", "vif", cif);
    run_test("ph5a_test");
  end
  
  initial begin
    $dumpfile("dump.vcd");
    $dumpvars;
  end
endmodule
