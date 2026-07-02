module rv32i_dcache #(
  parameter int CACHE_LINES = 16
)(
  input  logic        clk,
  input  logic        rst_n,

  // CPU-side data memory interface
  input  logic        cpu_req_valid,
  output logic        cpu_req_ready,
  input  logic        cpu_req_we,
  input  logic [31:0] cpu_req_addr,
  input  logic [31:0] cpu_req_wdata,
  input  logic [3:0]  cpu_req_wstrb,

  output logic        cpu_rsp_valid,
  output logic [31:0] cpu_rsp_rdata,

  // Memory-side data memory interface
  output logic        mem_req_valid,
  input  logic        mem_req_ready,
  output logic        mem_req_we,
  output logic [31:0] mem_req_addr,
  output logic [31:0] mem_req_wdata,
  output logic [3:0]  mem_req_wstrb,

  input  logic        mem_rsp_valid,
  input  logic [31:0] mem_rsp_rdata,
  // Statistics / coverage event pulses
  output logic        stat_req,
  output logic        stat_read,
  output logic        stat_write,
  output logic        stat_hit,
  output logic        stat_miss,
  output logic        stat_refill,
  output logic        stat_write_through
);

  localparam int INDEX_BITS = $clog2(CACHE_LINES);
  localparam int OFFSET_BITS = 2;
  localparam int TAG_LSB     = OFFSET_BITS + INDEX_BITS;

  typedef enum logic [1:0] {
    DC_IDLE,
    DC_HIT_RESP,
    DC_MEM_REQ,
    DC_WAIT_MEM
  } dc_state_t;

  dc_state_t state_q, state_d;

  logic [31:0] data_array [0:CACHE_LINES-1];
  logic [31:TAG_LSB] tag_array [0:CACHE_LINES-1];
  logic valid_array [0:CACHE_LINES-1];

  logic        pend_we_q;
  logic        pend_hit_q;
  logic [31:0] pend_addr_q;
  logic [31:0] pend_wdata_q;
  logic [3:0]  pend_wstrb_q;
  logic [31:0] pend_hit_data_q;

  logic [INDEX_BITS-1:0] req_index;
  logic [31:TAG_LSB]     req_tag;
  logic                  req_hit;

  logic [INDEX_BITS-1:0] pend_index;
  logic [31:TAG_LSB]     pend_tag;

  assign req_index = cpu_req_addr[TAG_LSB-1:OFFSET_BITS];
  assign req_tag   = cpu_req_addr[31:TAG_LSB];

  assign pend_index = pend_addr_q[TAG_LSB-1:OFFSET_BITS];
  assign pend_tag   = pend_addr_q[31:TAG_LSB];

  assign req_hit = valid_array[req_index] &&
                   (tag_array[req_index] == req_tag);

  function automatic logic [31:0] apply_wstrb(
    input logic [31:0] old_word,
    input logic [31:0] new_word,
    input logic [3:0]  wstrb
  );
    logic [31:0] result;
    begin
      result = old_word;

      if (wstrb[0]) result[7:0]   = new_word[7:0];
      if (wstrb[1]) result[15:8]  = new_word[15:8];
      if (wstrb[2]) result[23:16] = new_word[23:16];
      if (wstrb[3]) result[31:24] = new_word[31:24];

      return result;
    end
  endfunction

  // -----------------------------
  // State transition
  // -----------------------------
  always_comb begin
    state_d = state_q;

    case (state_q)
      DC_IDLE: begin
        if (cpu_req_valid) begin
          if (!cpu_req_we && req_hit) begin
            state_d = DC_HIT_RESP;
          end else begin
            state_d = DC_MEM_REQ;
          end
        end
      end

      DC_HIT_RESP: begin
        state_d = DC_IDLE;
      end

      DC_MEM_REQ: begin
        if (mem_req_ready) begin
          state_d = DC_WAIT_MEM;
        end
      end

      DC_WAIT_MEM: begin
        if (mem_rsp_valid) begin
          state_d = DC_IDLE;
        end
      end

      default: begin
        state_d = DC_IDLE;
      end
    endcase
  end

  // -----------------------------
  // CPU-side outputs
  // -----------------------------
  always_comb begin
    cpu_req_ready = (state_q == DC_IDLE);

    cpu_rsp_valid = 1'b0;
    cpu_rsp_rdata = 32'd0;

    if (state_q == DC_HIT_RESP) begin
      cpu_rsp_valid = 1'b1;
      cpu_rsp_rdata = pend_hit_data_q;
    end else if ((state_q == DC_WAIT_MEM) && mem_rsp_valid) begin
      cpu_rsp_valid = 1'b1;

      if (pend_we_q) begin
        cpu_rsp_rdata = 32'd0;
      end else begin
        cpu_rsp_rdata = mem_rsp_rdata;
      end
    end
  end

  // -----------------------------
  // Memory-side outputs
  // -----------------------------
  always_comb begin
    mem_req_valid = 1'b0;
    mem_req_we    = pend_we_q;
    mem_req_addr  = pend_addr_q;
    mem_req_wdata = pend_wdata_q;
    mem_req_wstrb = pend_wstrb_q;

    if (state_q == DC_MEM_REQ) begin
      mem_req_valid = 1'b1;
    end
  end
  
  // -----------------------------
  // Statistics event pulses
  // -----------------------------
  always_comb begin
    stat_req           = 1'b0;
    stat_read          = 1'b0;
    stat_write         = 1'b0;
    stat_hit           = 1'b0;
    stat_miss          = 1'b0;
    stat_refill        = 1'b0;
    stat_write_through = 1'b0;

    // Count accepted CPU-side D-cache requests
    if ((state_q == DC_IDLE) && cpu_req_valid && cpu_req_ready) begin
      stat_req = 1'b1;

      if (cpu_req_we) begin
        stat_write = 1'b1;
      end else begin
        stat_read = 1'b1;
      end

      if (req_hit) begin
        stat_hit = 1'b1;
      end else begin
        stat_miss = 1'b1;
      end
    end

    // Read miss refill completed
    if ((state_q == DC_WAIT_MEM) && mem_rsp_valid && !pend_we_q) begin
      stat_refill = 1'b1;
    end

    // Write-through store completed
    if ((state_q == DC_WAIT_MEM) && mem_rsp_valid && pend_we_q) begin
      stat_write_through = 1'b1;
    end
  end

  // -----------------------------
  // Sequential state/cache update
  // -----------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state_q         <= DC_IDLE;
      pend_we_q       <= 1'b0;
      pend_hit_q      <= 1'b0;
      pend_addr_q     <= 32'd0;
      pend_wdata_q    <= 32'd0;
      pend_wstrb_q    <= 4'd0;
      pend_hit_data_q <= 32'd0;

      for (int i = 0; i < CACHE_LINES; i++) begin
        valid_array[i] <= 1'b0;
        tag_array[i]   <= '0;
        data_array[i]  <= 32'd0;
      end
    end else begin
      state_q <= state_d;

      // Accept CPU request
      if ((state_q == DC_IDLE) && cpu_req_valid && cpu_req_ready) begin
        pend_we_q       <= cpu_req_we;
        pend_hit_q      <= req_hit;
        pend_addr_q     <= cpu_req_addr;
        pend_wdata_q    <= cpu_req_wdata;
        pend_wstrb_q    <= cpu_req_wstrb;
        pend_hit_data_q <= data_array[req_index];
      end

      // Read miss refill
      if ((state_q == DC_WAIT_MEM) && mem_rsp_valid && !pend_we_q) begin
        valid_array[pend_index] <= 1'b1;
        tag_array[pend_index]   <= pend_tag;
        data_array[pend_index]  <= mem_rsp_rdata;
      end

      // Write-through store completed.
      // Update cache only if the store hit an existing cached line.
      if ((state_q == DC_WAIT_MEM) && mem_rsp_valid && pend_we_q && pend_hit_q) begin
        data_array[pend_index] <= apply_wstrb(data_array[pend_index],
                                              pend_wdata_q,
                                              pend_wstrb_q);
      end
    end
  end

endmodule