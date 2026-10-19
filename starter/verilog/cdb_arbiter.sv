/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  cdb_arbiter.sv                                      //
//                                                                     //
//  Description :  Chooses which finished FU results go out on the     //
//                 CDBs this cycle. Purely combinational. At most      //
//                 NUM_CDB winners per cycle (CDBs <= width rule).     //
//                 Policy: fixed priority, longest latency first:      //
//                 FP > MULT > LOAD > STORE > ALU; within one FU type  //
//                 the lower port wins. Rationale: long-latency units  //
//                 tend to have more dependents waiting, and a         //
//                 pipelined unit that loses must stall its whole      //
//                 pipeline, while a 1-cycle ALU just holds one        //
//                 result. Lanes are filled in order (lane 0 first).   //
//                 Losers keep out_valid and retry next cycle.         //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

module cdb_arbiter #(
    localparam NUM_FU = `NUM_FU,
    localparam CDB_SZ = `NUM_CDB
) (
    input             [NUM_FU-1:0] fu_valid,   // each FU's out_valid
    input  CDB_PACKET [NUM_FU-1:0] fu_pkt,     // each FU's out_pkt
    output logic      [NUM_FU-1:0] fu_grant,   // → each FU's out_grant
    output logic      [CDB_SZ-1:0] cdb_valid,  // → PRF write, RS wakeup, busy table, ROB complete
    output CDB_PACKET [CDB_SZ-1:0] cdb_pkt
);


  int lane;  // next free CDB lane

  always_comb begin
    fu_grant  = '0;
    cdb_valid = '0;
    cdb_pkt   = '0;
    lane      = 0;
    for (int r = 0; r < `NUM_FU_RANKS; r++)
      for (int f = 0; f < NUM_FU; f++)
        if (fu_valid[f] && port_type(f) == rank_type(r) && lane < CDB_SZ) begin
          fu_grant[f]     = `TRUE;
          cdb_valid[lane] = `TRUE;
          cdb_pkt[lane]   = fu_pkt[f];
          lane++;
        end
  end

endmodule  // cdb_arbiter
