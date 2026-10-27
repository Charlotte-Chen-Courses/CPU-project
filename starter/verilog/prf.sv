/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  prf.sv                                              //
//                                                                     //
//  Description :  Physical register file (PRF) for the R10K-style     //
//                 core. Indexed by physical register tag. Read after  //
//                 issue: 2 ports per issue slot (ISSUE_WIDTH slots,   //
//                 the RS issues at most that many per cycle). Written //
//                 on the CDB (1 port per CDB). Physical reg 0 is      //
//                 hardwired to zero and backs architectural x0.       //
//                 Same-cycle CDB writes are bypassed to the read      //
//                 ports. One instance per register class: it holds    //
//                 tags BASE..BASE+SZ-1 and ignores writes to others   //
//                 (reads of them return 0); ZERO_REG = 0 (the FP      //
//                 file) makes local register 0 an ordinary one.       //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

module prf #(
    parameter  RD_SZ    = 2 * `ISSUE_WIDTH,  // 2 operands per issue slot
    parameter  BASE     = 0,                 // first tag of this register class
    parameter  SZ       = `PHYS_REG_SZ,
    parameter  bit ZERO_REG = 1'b1,          // local register 0 is a hardwired zero (x0)
    localparam WR_SZ = `NUM_CDB,
    localparam TAG_W = `TAG_W
) (
    input                               clock,    // system clock
    // note: no system reset, register values must be written before they can be read
    input        [RD_SZ-1:0][TAG_W-1:0] rd_tag,
    output logic [RD_SZ-1:0][`XLEN-1:0] rd_data,
    input        [WR_SZ-1:0]            wr_en,
    input        [WR_SZ-1:0][TAG_W-1:0] wr_tag,
    input        [WR_SZ-1:0][`XLEN-1:0] wr_data
);

  logic [SZ-1:0][`XLEN-1:0] registers;   // local index = tag - BASE (0 unused when ZERO_REG)

  function automatic bit in_file(input logic [TAG_W-1:0] t);
    // (every thread's x0: each thread's tag range starts with its x0 register)
    return (int'(t) >= BASE) && (int'(t) < BASE + SZ) && !(ZERO_REG && (int'(t) - BASE) % `THREAD_PHYS_SZ == 0);
  endfunction
  function automatic logic [$clog2(SZ)-1:0] local_idx(input logic [TAG_W-1:0] t);
    return $clog2(SZ)'(int'(t) - BASE);
  endfunction

  // Read port
  always_comb begin
    for (int r = 0; r < RD_SZ; r++) begin
      rd_data[r] = in_file(rd_tag[r]) ? registers[local_idx(rd_tag[r])] : '0;
      // internal forwarding
      for (int w = 0; w < WR_SZ; w++) begin
        if (wr_en[w] && wr_tag[w] == rd_tag[r] && in_file(rd_tag[r])) begin
          rd_data[r] = wr_data[w];
        end
      end
    end
  end

  // Write port
  always_ff @(posedge clock) begin
    for (int w = 0; w < WR_SZ; w++) begin
      if (wr_en[w] && in_file(wr_tag[w])) begin
        registers[local_idx(wr_tag[w])] <= wr_data[w];
      end
    end
  end

endmodule  // PRF
