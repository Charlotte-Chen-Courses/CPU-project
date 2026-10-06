/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  prf.sv                                              //
//                                                                     //
//  Description :  Physical register file (PRF) for the R10K-style     //
//                 core. Indexed by physical register tag. Read at     //
//                 issue (2 ports per issue slot), written on the      //
//                 CDB (1 port per CDB). Physical reg 0 is hardwired   //
//                 to zero and backs architectural x0. Same-cycle      //
//                 CDB writes are bypassed to the read ports.          //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

module prf #(
    parameter RD_SZ = 2 * `SUPERSCALAR_WIDTH,
    parameter WR_SZ = `NUM_CDB,
    parameter TAG_W = $clog2(`PHYS_REG_SZ)
) (
    input                               clock,    // system clock
    // note: no system reset, register values must be written before they can be read
    input        [RD_SZ-1:0][TAG_W-1:0] rd_tag,
    output logic [RD_SZ-1:0][`XLEN-1:0] rd_data,
    input        [WR_SZ-1:0]            wr_en,
    input        [WR_SZ-1:0][TAG_W-1:0] wr_tag,
    input        [WR_SZ-1:0][`XLEN-1:0] wr_data
);

  logic [`PHYS_REG_SZ-1:1][`XLEN-1:0] registers;  // PHYS_REG_SZ XLEN-length Registers (0 is known)

  // Read port
  always_comb begin
    for (int r = 0; r < RD_SZ; r++) begin
      rd_data[r] = (rd_tag[r] == 0) ? '0 : registers[rd_tag[r]];
      // internal forwarding
      for (int w = 0; w < WR_SZ; w++) begin
        if (wr_en[w] && wr_tag[w] == rd_tag[r] && rd_tag[r] != 0) begin
          rd_data[r] = wr_data[w];
        end
      end
    end
  end

  // Write port
  always_ff @(posedge clock) begin
    for (int w = 0; w < WR_SZ; w++) begin
      if (wr_en[w] && wr_tag[w] != 0) begin
        registers[wr_tag[w]] <= wr_data[w];
      end
    end
  end

endmodule  // PRF
