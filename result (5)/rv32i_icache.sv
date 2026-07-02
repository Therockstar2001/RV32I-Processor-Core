module rv32i_icache #(
  parameter int CACHE_LINES = 16
)(
  input  logic        clk,
  input  logic        rst_n,

  // CPU-side instruction fetch
  input  logic        cpu_req_valid,
  output logic        cpu_req_ready,
  input  logic [31:0] cpu_req_addr,

  output logic        cpu_rsp_valid,
  output logic [31:0] cpu_rsp_rdata,

  // Memory-side instruction fetch
  output logic        mem_req_valid,
  input  logic        mem_req_ready,
  output logic [31:0] mem_req_addr,

  input  logic        mem_rsp_valid,
  input  logic [31:0] mem_rsp_rdata,
  
  // Statistics / coverage event pulses
  output logic        stat_req,
  output logic        stat_hit,
  output logic        stat_miss,
  output logic        stat_refill
);

  localparam int INDEX_BITS = $clog2(CACHE_LINES);
  localparam int OFFSET_BITS = 2;
  localparam int TAG_LSB     = OFFSET_BITS + INDEX_BITS;

  typedef enum logic [1:0] {
    IC_IDLE,
    IC_WAIT_FILL
  } ic_state_t;

  ic_state_t state_q, state_d;

  logic [31:0] data_array  [0:CACHE_LINES-1];
  logic [31:TAG_LSB] tag_array [0:CACHE_LINES-1];
  logic valid_array [0:CACHE_LINES-1];

  logic [31:0] pending_addr_q;

  logic [INDEX_BITS-1:0] req_index;
  logic [31:TAG_LSB]     req_tag;
  logic                  hit;

  logic [INDEX_BITS-1:0] pending_index;
  logic [31:TAG_LSB]     pending_tag;

  assign req_index = cpu_req_addr[TAG_LSB-1:OFFSET_BITS];
  assign req_tag   = cpu_req_addr[31:TAG_LSB];

  assign pending_index = pending_addr_q[TAG_LSB-1:OFFSET_BITS];
  assign pending_tag   = pending_addr_q[31:TAG_LSB];

  assign hit = valid_array[req_index] &&
               (tag_array[req_index] == req_tag);

  // -----------------------------
  // State transition
  // -----------------------------
  always_comb begin
    state_d = state_q;

    case (state_q)
      IC_IDLE: begin
        if (cpu_req_valid && !hit && mem_req_ready && !mem_rsp_valid) begin
          state_d = IC_WAIT_FILL;
        end
      end

      IC_WAIT_FILL: begin
        if (mem_rsp_valid) begin
          state_d = IC_IDLE;
        end
      end

      default: begin
        state_d = IC_IDLE;
      end
    endcase
  end

  // -----------------------------
  // CPU-side and memory-side outputs
  // -----------------------------
  always_comb begin
    cpu_req_ready = 1'b0;
    cpu_rsp_valid = 1'b0;
    cpu_rsp_rdata = 32'h0000_0013;

    mem_req_valid = 1'b0;
    mem_req_addr  = cpu_req_addr;

    case (state_q)
      IC_IDLE: begin
        if (cpu_req_valid) begin
          if (hit) begin
            cpu_req_ready = 1'b1;
            cpu_rsp_valid = 1'b1;
            cpu_rsp_rdata = data_array[req_index];
          end else begin
            mem_req_valid = 1'b1;
            mem_req_addr  = cpu_req_addr;

            // Supports same-cycle memory response if the memory model allows it.
            if (mem_req_ready && mem_rsp_valid) begin
              cpu_req_ready = 1'b1;
              cpu_rsp_valid = 1'b1;
              cpu_rsp_rdata = mem_rsp_rdata;
            end
          end
        end
      end

      IC_WAIT_FILL: begin
        // Return response only if the core is still requesting the same PC.
        // This protects against branch redirect while an old miss is pending.
        if (mem_rsp_valid &&
            cpu_req_valid &&
            (cpu_req_addr == pending_addr_q)) begin
          cpu_req_ready = 1'b1;
          cpu_rsp_valid = 1'b1;
          cpu_rsp_rdata = mem_rsp_rdata;
        end
      end

      default: begin
        cpu_req_ready = 1'b0;
        cpu_rsp_valid = 1'b0;
        cpu_rsp_rdata = 32'h0000_0013;
        mem_req_valid = 1'b0;
        mem_req_addr  = cpu_req_addr;
      end
    endcase
  end
  
  
  // -----------------------------
  // Statistics event pulses
  // -----------------------------
  always_comb begin
    stat_req    = 1'b0;
    stat_hit    = 1'b0;
    stat_miss   = 1'b0;
    stat_refill = 1'b0;

    // Count cache lookup events only in IDLE.
    if ((state_q == IC_IDLE) && cpu_req_valid) begin
      if (hit) begin
        stat_req = 1'b1;
        stat_hit = 1'b1;
      end else if (mem_req_ready) begin
        stat_req  = 1'b1;
        stat_miss = 1'b1;
      end
    end

    // Count refill when memory returns instruction data.
    if (((state_q == IC_IDLE) &&
         cpu_req_valid &&
         !hit &&
         mem_req_ready &&
         mem_rsp_valid) ||
        ((state_q == IC_WAIT_FILL) && mem_rsp_valid)) begin
      stat_refill = 1'b1;
    end
  end

  // -----------------------------
  // Sequential state/cache update
  // -----------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state_q        <= IC_IDLE;
      pending_addr_q <= 32'd0;

      for (int i = 0; i < CACHE_LINES; i++) begin
        valid_array[i] <= 1'b0;
        tag_array[i]   <= '0;
        data_array[i]  <= 32'h0000_0013;
      end
    end else begin
      state_q <= state_d;

      // New miss accepted; remember the requested address.
      if ((state_q == IC_IDLE) &&
          cpu_req_valid &&
          !hit &&
          mem_req_ready &&
          !mem_rsp_valid) begin
        pending_addr_q <= cpu_req_addr;
      end

      // Same-cycle refill path.
      if ((state_q == IC_IDLE) &&
          cpu_req_valid &&
          !hit &&
          mem_req_ready &&
          mem_rsp_valid) begin
        valid_array[req_index] <= 1'b1;
        tag_array[req_index]   <= req_tag;
        data_array[req_index]  <= mem_rsp_rdata;
      end

      // Delayed refill path.
      else if ((state_q == IC_WAIT_FILL) && mem_rsp_valid) begin
        valid_array[pending_index] <= 1'b1;
        tag_array[pending_index]   <= pending_tag;
        data_array[pending_index]  <= mem_rsp_rdata;
      end
    end
  end

endmodule