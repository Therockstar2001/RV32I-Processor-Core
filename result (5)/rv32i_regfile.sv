module rv32i_regfile (
  input  logic        clk,
  input  logic        rst_n,

  input  logic [4:0]  raddr1,
  input  logic [4:0]  raddr2,
  output logic [31:0] rdata1,
  output logic [31:0] rdata2,

  input  logic        we,
  input  logic [4:0]  waddr,
  input  logic [31:0] wdata
);

  logic [31:0] rf [0:31];

  // Combinational reads
  always_comb begin
    rdata1 = (raddr1 == 5'd0) ? 32'd0 : rf[raddr1];
    rdata2 = (raddr2 == 5'd0) ? 32'd0 : rf[raddr2];
  end

  // Synchronous write + reset
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int i = 0; i < 32; i++) begin
        rf[i] <= 32'd0;
      end
    end else begin
      if (we && (waddr != 5'd0)) begin
        rf[waddr] <= wdata;
      end

      // RISC-V x0 must always remain zero
      rf[0] <= 32'd0;
    end
  end

endmodule