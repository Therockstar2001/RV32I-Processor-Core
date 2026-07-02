module rv32i_mem_unit (
  input  logic clk,
  input  logic rst_n,

  input  rv32i_pkg::exmem_t ex_mem_in,

  output logic        dmem_req_valid,
  input  logic        dmem_req_ready,
  output logic        dmem_req_we,
  output logic [31:0] dmem_req_addr,
  output logic [31:0] dmem_req_wdata,
  output logic [3:0]  dmem_req_wstrb,

  input  logic        dmem_rsp_valid,
  input  logic [31:0] dmem_rsp_rdata,

  output logic        mem_wait,
  output rv32i_pkg::memwb_t mem_wb_out
);

  import rv32i_pkg::*;

  typedef enum logic {
    MTU_IDLE,
    MTU_WAIT
  } mtu_state_t;

  mtu_state_t state_q, state_d;

  logic mem_access;

  assign mem_access = ex_mem_in.valid &&
                      (ex_mem_in.mem_read || ex_mem_in.mem_write);

  // -----------------------------
  // Request generation
  // -----------------------------
  always_comb begin
    dmem_req_valid = 1'b0;
    dmem_req_we    = ex_mem_in.mem_write;
    dmem_req_addr  = ex_mem_in.alu_result;
    dmem_req_wdata = ex_mem_in.store_data;
    dmem_req_wstrb = ex_mem_in.mem_write ? 4'b1111 : 4'b0000;

    if ((state_q == MTU_IDLE) && mem_access) begin
      dmem_req_valid = 1'b1;
    end
  end

  // -----------------------------
  // State machine
  // -----------------------------
  always_comb begin
    state_d = state_q;

    case (state_q)
      MTU_IDLE: begin
        if (mem_access && dmem_req_ready) begin
          state_d = MTU_WAIT;
        end
      end

      MTU_WAIT: begin
        if (dmem_rsp_valid) begin
          state_d = MTU_IDLE;
        end
      end
      
      default: begin
        state_d = MTU_IDLE;
      end
    endcase
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) state_q <= MTU_IDLE;
    else        state_q <= state_d;
  end

  // -----------------------------
  // Pipeline stall
  // -----------------------------
  always_comb begin
    mem_wait = 1'b0;

    if (mem_access) begin
      if (state_q == MTU_IDLE)
        mem_wait = 1'b1;          // request launched, wait for response later
      else if (state_q == MTU_WAIT && !dmem_rsp_valid)
        mem_wait = 1'b1;
    end
  end

  // -----------------------------
  // MEM/WB output
  // -----------------------------
  always_comb begin
    mem_wb_out = '0;

    // Non-memory instructions pass through immediately
    if (ex_mem_in.valid && !ex_mem_in.mem_read && !ex_mem_in.mem_write) begin
      mem_wb_out.valid = ex_mem_in.valid;
      mem_wb_out.pc    = ex_mem_in.pc;
      mem_wb_out.instr = ex_mem_in.instr;
      mem_wb_out.rd    = ex_mem_in.rd;
      mem_wb_out.rd_we = ex_mem_in.rd_we;

      unique case (ex_mem_in.wb_src)
        WB_PC4:  mem_wb_out.wb_data = ex_mem_in.pc_plus4;
        default: mem_wb_out.wb_data = ex_mem_in.alu_result;
      endcase
    end

    // Memory instruction completes only when response returns
    else if (mem_access && (state_q == MTU_WAIT) && dmem_rsp_valid) begin
      mem_wb_out.valid = ex_mem_in.valid;
      mem_wb_out.pc    = ex_mem_in.pc;
      mem_wb_out.instr = ex_mem_in.instr;
      mem_wb_out.rd    = ex_mem_in.rd;
      mem_wb_out.rd_we = ex_mem_in.rd_we;

      if (ex_mem_in.wb_src == WB_MEM)
        mem_wb_out.wb_data = dmem_rsp_rdata;
      else
        mem_wb_out.wb_data = ex_mem_in.alu_result;
    end
  end

endmodule