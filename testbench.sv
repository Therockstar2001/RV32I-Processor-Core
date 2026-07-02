`timescale 1ns/1ps
`include "uvm_macros.svh"
import uvm_pkg::*;


`define IMEM_WORD(i) tb_imem[i]
localparam int IMEM_WORDS = 256;  // 32-bit words

// ============================================================
// Interface
// ============================================================
interface core_if;
  logic clk;
  logic rst_n;
  
  logic        imem_req_valid;
  logic        imem_req_ready;
  logic [31:0] imem_req_addr;

  logic        imem_rsp_valid;
  logic [31:0] imem_rsp_rdata;
  
  logic        dmem_req_valid;
  logic        dmem_req_ready;
  logic        dmem_req_we;
  logic [31:0] dmem_req_addr;
  logic [31:0] dmem_req_wdata;
  logic [3:0]  dmem_req_wstrb;

  logic        dmem_rsp_valid;
  logic [31:0] dmem_rsp_rdata;

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
// Instruction encoders for directed stimulus
// ============================================================
function automatic bit [31:0] enc_addi(
  input bit [4:0] rd,
  input bit [4:0] rs1,
  input int       imm
);
  bit [11:0] imm12;
  begin
    imm12 = imm[11:0];
    return {imm12, rs1, 3'b000, rd, 7'b0010011};
  end
endfunction

function automatic bit [31:0] enc_lw(
  input bit [4:0] rd,
  input bit [4:0] rs1,
  input int       imm
);
  bit [11:0] imm12;
  begin
    imm12 = imm[11:0];
    return {imm12, rs1, 3'b010, rd, 7'b0000011};
  end
endfunction

function automatic bit [31:0] enc_sw(
  input bit [4:0] rs2,
  input bit [4:0] rs1,
  input int       imm
);
  bit [11:0] imm12;
  begin
    imm12 = imm[11:0];
    return {imm12[11:5], rs2, rs1, 3'b010, imm12[4:0], 7'b0100011};
  end
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
  always #0.5 cif.clk = ~cif.clk;
  
  // ============================================================
  // Instruction Memory Storage
  // Backing memory behind I-cache
  // ============================================================
  logic [31:0] tb_imem [0:IMEM_WORDS-1];
  
  logic        imem_mem_req_valid;
  logic        imem_mem_req_ready;
  logic [31:0] imem_mem_req_addr;
  logic        imem_mem_rsp_valid;
  logic [31:0] imem_mem_rsp_rdata;
  
  logic        imem_busy;
  logic [31:0] imem_pending_addr;
  logic [3:0]  imem_delay_cnt;

  logic icache_stat_req;
  logic icache_stat_hit;
  logic icache_stat_miss;
  logic icache_stat_refill;

  int unsigned icache_req_count;
  int unsigned icache_hit_count;
  int unsigned icache_miss_count;
  int unsigned icache_refill_count;

  bit seen_icache_miss;
  real icache_hit_rate;
  
  logic dcache_stat_req;
  logic dcache_stat_read;
  logic dcache_stat_write;
  logic dcache_stat_hit;
  logic dcache_stat_miss;
  logic dcache_stat_refill;
  logic dcache_stat_write_through;

  int unsigned dcache_req_count;
  int unsigned dcache_read_count;
  int unsigned dcache_write_count;
  int unsigned dcache_hit_count;
  int unsigned dcache_miss_count;
  int unsigned dcache_refill_count;
  int unsigned dcache_write_through_count;

  bit seen_dcache_miss;
  real dcache_hit_rate;
  int unsigned cache_assert_fail_count;
  
  // ============================================================
  // I-cache functional coverage
  // ============================================================
  covergroup cg_icache @(posedge cif.clk);
    option.per_instance = 1;

    cp_req : coverpoint icache_stat_req iff (cif.rst_n) {
      bins SEEN = {1};
    }

    cp_hit : coverpoint icache_stat_hit iff (cif.rst_n) {
      bins SEEN = {1};
    }

    cp_miss : coverpoint icache_stat_miss iff (cif.rst_n) {
      bins SEEN = {1};
    }

    cp_refill : coverpoint icache_stat_refill iff (cif.rst_n) {
      bins SEEN = {1};
    }

    cp_hit_after_miss : coverpoint icache_stat_hit iff (cif.rst_n && seen_icache_miss) {
      bins SEEN = {1};
    }

  endgroup
  
  // ============================================================
  // D-cache functional coverage
  // ============================================================
  covergroup cg_dcache @(posedge cif.clk);
    option.per_instance = 1;

    cp_req : coverpoint dcache_stat_req iff (cif.rst_n) {
      bins SEEN = {1};
    }

    cp_read : coverpoint dcache_stat_read iff (cif.rst_n) {
      bins SEEN = {1};
    }

    cp_write : coverpoint dcache_stat_write iff (cif.rst_n) {
      bins SEEN = {1};
    }

    cp_hit : coverpoint dcache_stat_hit iff (cif.rst_n) {
      bins SEEN = {1};
    }

    cp_miss : coverpoint dcache_stat_miss iff (cif.rst_n) {
      bins SEEN = {1};
    }

    cp_refill : coverpoint dcache_stat_refill iff (cif.rst_n) {
      bins SEEN = {1};
    }

    cp_write_through : coverpoint dcache_stat_write_through iff (cif.rst_n) {
      bins SEEN = {1};
    }

    cp_hit_after_miss : coverpoint dcache_stat_hit iff (cif.rst_n && seen_dcache_miss) {
      bins SEEN = {1};
    }

  endgroup

  cg_icache icache_cg;
  cg_dcache dcache_cg;

  initial begin
    icache_cg = new();
    dcache_cg = new();
    cache_assert_fail_count = 0;
  end
  
  initial begin
    for (int i = 0; i < IMEM_WORDS; i++) begin
      tb_imem[i] = 32'h0000_0013; // NOP
    end
  end
  
  // ============================================================
  // Data Memory Storage
  // Backing memory behind D-cache
  // ============================================================
  logic [31:0] tb_dmem [0:IMEM_WORDS-1];

  // ============================================================
  // Variable-Latency Data Memory Model
  // Connected behind D-cache
  // ============================================================

  logic        dmem_busy;
  logic        dmem_pending_we;
  logic [31:0] dmem_pending_addr;
  logic [31:0] dmem_pending_wdata;
  logic [3:0]  dmem_pending_wstrb;
  logic [3:0]  dmem_delay_cnt;
  
  logic        dmem_mem_req_valid;
  logic        dmem_mem_req_ready;
  logic        dmem_mem_req_we;
  logic [31:0] dmem_mem_req_addr;
  logic [31:0] dmem_mem_req_wdata;
  logic [3:0]  dmem_mem_req_wstrb;

  logic        dmem_mem_rsp_valid;
  logic [31:0] dmem_mem_rsp_rdata;
  
  
  always_comb begin
    // Accept a new request only when memory model is idle.
    dmem_mem_req_ready = !dmem_busy;

    // Response is generated by sequential model.
    // Default assignments avoid X-propagation.
  end

  always_ff @(posedge cif.clk or negedge cif.rst_n) begin
    if (!cif.rst_n) begin
      dmem_busy          <= 1'b0;
      dmem_pending_we    <= 1'b0;
      dmem_pending_addr  <= 32'd0;
      dmem_pending_wdata <= 32'd0;
      dmem_pending_wstrb <= 4'd0;
      dmem_delay_cnt     <= 4'd0;

      dmem_mem_rsp_valid <= 1'b0;
      dmem_mem_rsp_rdata <= 32'd0;
    end else begin
      dmem_mem_rsp_valid <= 1'b0;
      dmem_mem_rsp_rdata <= 32'd0;

      // Accept new request from D-cache
      if (!dmem_busy && dmem_mem_req_valid) begin
        dmem_busy          <= 1'b1;
        dmem_pending_we    <= dmem_mem_req_we;
        dmem_pending_addr  <= dmem_mem_req_addr;
        dmem_pending_wdata <= dmem_mem_req_wdata;
        dmem_pending_wstrb <= dmem_mem_req_wstrb;

        // Deterministic latency pattern: 1 to 4 cycles
        dmem_delay_cnt <= dmem_mem_req_addr[3:2] + 4'd1;
      end

      // Service pending request
      else if (dmem_busy) begin
        if (dmem_delay_cnt != 0) begin
          dmem_delay_cnt <= dmem_delay_cnt - 1'b1;
        end else begin
          int unsigned didx;
          didx = dmem_pending_addr[31:2];

          // Store
          if (dmem_pending_we) begin
            if (didx < IMEM_WORDS) begin
              if (dmem_pending_wstrb[0]) tb_dmem[didx][7:0]   <= dmem_pending_wdata[7:0];
              if (dmem_pending_wstrb[1]) tb_dmem[didx][15:8]  <= dmem_pending_wdata[15:8];
              if (dmem_pending_wstrb[2]) tb_dmem[didx][23:16] <= dmem_pending_wdata[23:16];
              if (dmem_pending_wstrb[3]) tb_dmem[didx][31:24] <= dmem_pending_wdata[31:24];
            end

            dmem_mem_rsp_valid <= 1'b1;
            dmem_mem_rsp_rdata <= 32'd0;
          end

          // Load
          else begin
            dmem_mem_rsp_valid <= 1'b1;

            if (didx < IMEM_WORDS) begin
              dmem_mem_rsp_rdata <= tb_dmem[didx];
            end else begin
              dmem_mem_rsp_rdata <= 32'd0;
            end
          end

          dmem_busy <= 1'b0;
        end
      end
    end
  end

  rv32i_core dut (
    .clk             (cif.clk),
    .rst_n           (cif.rst_n),
    .imem_req_valid  (cif.imem_req_valid),
    .imem_req_ready  (cif.imem_req_ready),
    .imem_req_addr   (cif.imem_req_addr),

    .imem_rsp_valid  (cif.imem_rsp_valid),
    .imem_rsp_rdata  (cif.imem_rsp_rdata),
    
    .dmem_req_valid (cif.dmem_req_valid),
    .dmem_req_ready (cif.dmem_req_ready),
    .dmem_req_we    (cif.dmem_req_we),
    .dmem_req_addr  (cif.dmem_req_addr),
    .dmem_req_wdata (cif.dmem_req_wdata),
    .dmem_req_wstrb (cif.dmem_req_wstrb),

    .dmem_rsp_valid (cif.dmem_rsp_valid),
    .dmem_rsp_rdata (cif.dmem_rsp_rdata),
    .commit_valid    (cif.commit_valid),
    .commit_pc       (cif.commit_pc),
    .commit_instr    (cif.commit_instr),
    .commit_rd       (cif.commit_rd),
    .commit_rd_we    (cif.commit_rd_we),
    .commit_rd_wdata (cif.commit_rd_wdata)
  );
  
  rv32i_dcache u_dcache (
    .clk           (cif.clk),
    .rst_n         (cif.rst_n),

    // Core / MTU side
    .cpu_req_valid (cif.dmem_req_valid),
    .cpu_req_ready (cif.dmem_req_ready),
    .cpu_req_we    (cif.dmem_req_we),
    .cpu_req_addr  (cif.dmem_req_addr),
    .cpu_req_wdata (cif.dmem_req_wdata),
    .cpu_req_wstrb (cif.dmem_req_wstrb),

    .cpu_rsp_valid (cif.dmem_rsp_valid),
    .cpu_rsp_rdata (cif.dmem_rsp_rdata),

    // Backing memory side
    .mem_req_valid (dmem_mem_req_valid),
    .mem_req_ready (dmem_mem_req_ready),
    .mem_req_we    (dmem_mem_req_we),
    .mem_req_addr  (dmem_mem_req_addr),
    .mem_req_wdata (dmem_mem_req_wdata),
    .mem_req_wstrb (dmem_mem_req_wstrb),

    .mem_rsp_valid (dmem_mem_rsp_valid),
    .mem_rsp_rdata (dmem_mem_rsp_rdata),
    .stat_req            (dcache_stat_req),
    .stat_read           (dcache_stat_read),
    .stat_write          (dcache_stat_write),
    .stat_hit            (dcache_stat_hit),
    .stat_miss           (dcache_stat_miss),
    .stat_refill         (dcache_stat_refill),
    .stat_write_through  (dcache_stat_write_through)
  );
  
  // ============================================================
  // D-cache statistics counters
  // ============================================================
  always_ff @(posedge cif.clk or negedge cif.rst_n) begin
    if (!cif.rst_n) begin
      dcache_req_count           <= 0;
      dcache_read_count          <= 0;
      dcache_write_count         <= 0;
      dcache_hit_count           <= 0;
      dcache_miss_count          <= 0;
      dcache_refill_count        <= 0;
      dcache_write_through_count <= 0;
      seen_dcache_miss           <= 1'b0;
    end else begin
      if (dcache_stat_req) begin
        dcache_req_count <= dcache_req_count + 1;
      end

      if (dcache_stat_read) begin
        dcache_read_count <= dcache_read_count + 1;
      end

      if (dcache_stat_write) begin
        dcache_write_count <= dcache_write_count + 1;
      end

      if (dcache_stat_hit) begin
        dcache_hit_count <= dcache_hit_count + 1;
      end

      if (dcache_stat_miss) begin
        dcache_miss_count <= dcache_miss_count + 1;
        seen_dcache_miss  <= 1'b1;
      end

      if (dcache_stat_refill) begin
        dcache_refill_count <= dcache_refill_count + 1;
      end

      if (dcache_stat_write_through) begin
        dcache_write_through_count <= dcache_write_through_count + 1;
      end
    end
  end
  
  rv32i_icache u_icache (
    .clk           (cif.clk),
    .rst_n         (cif.rst_n),

    .cpu_req_valid (cif.imem_req_valid),
    .cpu_req_ready (cif.imem_req_ready),
    .cpu_req_addr  (cif.imem_req_addr),

    .cpu_rsp_valid (cif.imem_rsp_valid),
    .cpu_rsp_rdata (cif.imem_rsp_rdata),

    .mem_req_valid (imem_mem_req_valid),
    .mem_req_ready (imem_mem_req_ready),
    .mem_req_addr  (imem_mem_req_addr),

    .mem_rsp_valid (imem_mem_rsp_valid),
    .mem_rsp_rdata (imem_mem_rsp_rdata),
    .stat_req     (icache_stat_req),
    .stat_hit     (icache_stat_hit),
    .stat_miss    (icache_stat_miss),
    .stat_refill  (icache_stat_refill)
  );
  
  // ============================================================
  // I-cache statistics counters
  // ============================================================
  always_ff @(posedge cif.clk or negedge cif.rst_n) begin
    if (!cif.rst_n) begin
      icache_req_count    <= 0;
      icache_hit_count    <= 0;
      icache_miss_count   <= 0;
      icache_refill_count <= 0;
      seen_icache_miss    <= 1'b0;
    end else begin
      if (icache_stat_req) begin
        icache_req_count <= icache_req_count + 1;
      end

      if (icache_stat_hit) begin
        icache_hit_count <= icache_hit_count + 1;
      end

      if (icache_stat_miss) begin
        icache_miss_count <= icache_miss_count + 1;
        seen_icache_miss  <= 1'b1;
      end

      if (icache_stat_refill) begin
        icache_refill_count <= icache_refill_count + 1;
      end
    end
  end
  
  // ============================================================
  // Cache protocol assertions
  // ============================================================

  // ------------------------------------------------------------
  // I-cache assertions
  // ------------------------------------------------------------

  // I-cache should not return a response unless the core is requesting.
  a_icache_rsp_requires_req:
  assert property (@(posedge cif.clk) disable iff (!cif.rst_n)
    cif.imem_rsp_valid |-> cif.imem_req_valid
  )
  else begin
    cache_assert_fail_count++;
    `uvm_error("ICACHE_ASSERT", "I-cache response asserted without active CPU fetch request")
  end

  // Every I-cache miss should eventually produce a refill.
  a_icache_miss_eventually_refills:
  assert property (@(posedge cif.clk) disable iff (!cif.rst_n)
    icache_stat_miss |-> ##[0:16] icache_stat_refill
  )
  else begin
    cache_assert_fail_count++;
    `uvm_error("ICACHE_ASSERT", "I-cache miss did not receive refill within expected latency window")
  end

  // If instruction memory applies backpressure, request address must remain stable.
  a_icache_mem_req_stable_when_waiting:
  assert property (@(posedge cif.clk) disable iff (!cif.rst_n)
    (imem_mem_req_valid && !imem_mem_req_ready)
      |=> (imem_mem_req_valid && $stable(imem_mem_req_addr))
  )
  else begin
    cache_assert_fail_count++;
    `uvm_error("ICACHE_ASSERT", "I-cache memory request address changed while memory was not ready")
  end

  // ------------------------------------------------------------
  // D-cache assertions
  // ------------------------------------------------------------

  // Every accepted D-cache CPU request should eventually receive a response.
  a_dcache_req_eventually_responds:
  assert property (@(posedge cif.clk) disable iff (!cif.rst_n)
    (cif.dmem_req_valid && cif.dmem_req_ready)
      |-> ##[1:32] cif.dmem_rsp_valid
  )
  else begin
    cache_assert_fail_count++;
    `uvm_error("DCACHE_ASSERT", "Accepted D-cache CPU request did not receive response")
  end

  // Every accepted store should eventually generate a write-through memory request.
  a_dcache_store_generates_write_through:
  assert property (@(posedge cif.clk) disable iff (!cif.rst_n)
    (cif.dmem_req_valid && cif.dmem_req_ready && cif.dmem_req_we)
      |-> ##[1:16] (dmem_mem_req_valid && dmem_mem_req_we)
  )
  else begin
    cache_assert_fail_count++;
    `uvm_error("DCACHE_ASSERT", "Accepted D-cache store did not generate write-through memory request")
  end

  // Backing data-memory request must remain stable while memory is not ready.
  a_dcache_mem_req_stable_when_waiting:
  assert property (@(posedge cif.clk) disable iff (!cif.rst_n)
    (dmem_mem_req_valid && !dmem_mem_req_ready)
      |=> (dmem_mem_req_valid &&
           $stable(dmem_mem_req_addr) &&
           $stable(dmem_mem_req_we) &&
           $stable(dmem_mem_req_wdata) &&
           $stable(dmem_mem_req_wstrb))
  )
  else begin
    cache_assert_fail_count++;
    `uvm_error("DCACHE_ASSERT", "D-cache memory request changed while backing memory was not ready")
  end

  // A D-cache transaction cannot be both a read refill and a write-through completion.
  a_dcache_refill_not_write_through_same_cycle:
  assert property (@(posedge cif.clk) disable iff (!cif.rst_n)
    !(dcache_stat_refill && dcache_stat_write_through)
  )
  else begin
    cache_assert_fail_count++;
    `uvm_error("DCACHE_ASSERT", "D-cache refill and write-through completion occurred in same cycle")
  end
  
  // ============================================================
  // Variable-Latency Instruction Memory Model
  // Connected behind I-cache
  // ============================================================

  always_comb begin
    imem_mem_req_ready = !imem_busy;
  end

  always_ff @(posedge cif.clk or negedge cif.rst_n) begin
    if (!cif.rst_n) begin
      imem_busy          <= 1'b0;
      imem_pending_addr  <= 32'd0;
      imem_delay_cnt     <= 4'd0;
      imem_mem_rsp_valid <= 1'b0;
      imem_mem_rsp_rdata <= 32'h0000_0013;
    end else begin
      imem_mem_rsp_valid <= 1'b0;
      imem_mem_rsp_rdata <= 32'h0000_0013;

      // Accept new instruction fetch request
      if (!imem_busy && imem_mem_req_valid) begin
        imem_busy         <= 1'b1;
        imem_pending_addr <= imem_mem_req_addr;

        // Deterministic latency: 1 to 4 cycles
        imem_delay_cnt <= imem_mem_req_addr[3:2] + 4'd1;
      end

      // Service pending instruction fetch
      else if (imem_busy) begin
        if (imem_delay_cnt != 0) begin
          imem_delay_cnt <= imem_delay_cnt - 1'b1;
        end else begin
          int unsigned idx;

          idx = imem_pending_addr[31:2];

          imem_mem_rsp_valid <= 1'b1;

          if (idx < IMEM_WORDS) begin
            imem_mem_rsp_rdata <= tb_imem[idx];
          end else begin
            imem_mem_rsp_rdata <= 32'h0000_0013;
          end

          imem_busy <= 1'b0;
        end
      end
    end
  end

`ifdef TB_LOADS_PROGRAM
  
  initial begin : GEN_PROGRAM
    int i;
    int idx;
    #1; // wait for tb_imem initialization before loading program
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
    
    // ============================================================
    // D-cache directed stress block
    // Exercises:
    // - write-through stores
    // - read misses 
    // - read hits
    // - write hits
    // - repeated line reuse
    // - index conflict behavior
    // ============================================================

    // Fill/reuse all 16 direct-mapped cache indices.
    // Pattern per offset:
    //   store -> write-through
    //   load  -> read miss/refill or hit depending on prior state
    //   load  -> expected read hit
    for (int k = 0; k < 32; k++) begin
      int off;
      off = (k % 16) * 4;

      `IMEM_WORD(idx++) = enc_addi(5'd23, 5'd0, k + 1);      // x23 = unique data
      `IMEM_WORD(idx++) = enc_sw  (5'd23, 5'd1, off);        // sw x23, off(x1)
      `IMEM_WORD(idx++) = enc_lw  (5'd24, 5'd1, off);        // lw x24, off(x1)
      `IMEM_WORD(idx++) = enc_lw  (5'd25, 5'd1, off);        // lw x25, off(x1) hit
    end

    // Conflict stress.
    // Addresses separated by 64 bytes map to the same cache index
    // because CACHE_LINES=16 and line size=4 bytes.
    // 64 bytes = 16 words = same index, different tag.
    for (int k = 0; k < 8; k++) begin
      int off;
      int conflict_off;

      off          = (k % 4) * 4;
      conflict_off = off + 64;

      `IMEM_WORD(idx++) = enc_addi(5'd23, 5'd0, 100 + k);          // x23 = conflict data
      `IMEM_WORD(idx++) = enc_sw  (5'd23, 5'd1, conflict_off);    // write conflict address
      `IMEM_WORD(idx++) = enc_lw  (5'd26, 5'd1, conflict_off);    // load/refill conflict line
      `IMEM_WORD(idx++) = enc_lw  (5'd27, 5'd1, off);             // re-read original index
    end

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
  
  final begin
    if (icache_req_count != 0) begin
      icache_hit_rate = (100.0 * icache_hit_count) / icache_req_count;
    end else begin
      icache_hit_rate = 0.0;
    end

    $display("[ICACHE_STATS] requests=%0d hits=%0d misses=%0d refills=%0d hit_rate=%0.2f%%",
             icache_req_count,
             icache_hit_count,
             icache_miss_count,
             icache_refill_count,
             icache_hit_rate);

    $display("[ICACHE_COV] coverage=%0.2f%%", icache_cg.get_coverage());
    
    if (dcache_req_count != 0) begin
      dcache_hit_rate = (100.0 * dcache_hit_count) / dcache_req_count;
    end else begin
      dcache_hit_rate = 0.0;
    end

    $display("[DCACHE_STATS] requests=%0d reads=%0d writes=%0d hits=%0d misses=%0d refills=%0d write_through=%0d hit_rate=%0.2f%%",
             dcache_req_count,
             dcache_read_count,
             dcache_write_count,
             dcache_hit_count,
             dcache_miss_count,
             dcache_refill_count,
             dcache_write_through_count,
             dcache_hit_rate);

    $display("[DCACHE_COV] coverage=%0.2f%%", dcache_cg.get_coverage());
    $display("[CACHE_ASSERTS] failures=%0d", cache_assert_fail_count);
  end
endmodule
