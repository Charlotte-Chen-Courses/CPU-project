/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  busy_table.sv                                       //
//                                                                     //
//  Description :  One busy bit per physical register: 1 = value       //
//                 not produced yet. Set at rename for newly           //
//                 allocated destinations, cleared by the CDB. A       //
//                 lookup is ready if the bit is clear or the tag is   //
//                 on a CDB this cycle (same-cycle bypass). p0 is      //
//                 never busy. Same-bundle dependencies are NOT        //
//                 handled here (bit is set at the edge): rename       //
//                 must mark a source bypassed by the RAT not ready.   //
//                 No flush yet.                                       //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

module busy_table #(
    localparam TAG_W         = `TAG_W,
    localparam BUSY_TABLE_SZ = `PHYS_REG_SZ,
    localparam W             = `SUPERSCALAR_WIDTH,
    localparam CDB_SZ        = `NUM_CDB

) (
    input                                clock,
    reset,
    // rename: newly allocated destinations become busy
    input        [     W-1:0]            set_en,     // dest_valid && rename_en
    input        [     W-1:0][TAG_W-1:0] set_tag,
    // CDB: produced values become ready
    input        [CDB_SZ-1:0]            cdb_valid,
    input        [CDB_SZ-1:0][TAG_W-1:0] cdb_tag,
    // rename lookups: 2 sources per slot
    input        [     W-1:0][TAG_W-1:0] src1_tag,
    src2_tag,
    output logic [     W-1:0]            src1_ready,
    src2_ready  // includes same-cycle CDB bypass
);
  logic [BUSY_TABLE_SZ-1:0] busy_table_map;

  always_comb
    for (int i = 0; i < W; i++) begin
      src1_ready[i] = !busy_table_map[src1_tag[i]];
      src2_ready[i] = !busy_table_map[src2_tag[i]];
      for (int k = 0; k < CDB_SZ; k++)
      if (cdb_valid[k]) begin
        if (cdb_tag[k] == src1_tag[i]) src1_ready[i] = `TRUE;
        if (cdb_tag[k] == src2_tag[i]) src2_ready[i] = `TRUE;
      end
    end


  always_ff @(posedge clock) begin
    if (reset) begin
      for (int i = 0; i < BUSY_TABLE_SZ; i++) busy_table_map[i] <= `FALSE;
    end else begin
      for (int k = 0; k < CDB_SZ; k++) if (cdb_valid[k]) busy_table_map[cdb_tag[k]] <= `FALSE;
      for (int i = 0; i < W; i++)
      if (set_en[i] && set_tag[i] != '0) busy_table_map[set_tag[i]] <= `TRUE;
    end

  end


endmodule
