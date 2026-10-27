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
//                 flush clears all. One instance per register class   //
//                 (tags BASE..BASE+BUSY_TABLE_SZ-1); tags of the      //
//                 other class are ignored and read as ready. With     //
//                 ZERO_REG = 0 (FP), local preg 0 is an ordinary one. //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

module busy_table #(
    parameter  BASE          = 0,              // first tag of this register class
    parameter  BUSY_TABLE_SZ = `THREAD_PHYS_SZ,
    parameter  bit ZERO_REG  = 1'b1,           // local preg 0 (x0) is never busy
    localparam TAG_W         = `TAG_W,
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
    // mispredict: everything in flight is squashed, so every mapped register holds a value
    input                                flush,
    // rename lookups: 3 sources per slot (tags of another class read as ready)
    input        [     W-1:0][TAG_W-1:0] src1_tag,
    src2_tag,
    src3_tag,
    output logic [     W-1:0]            src1_ready,
    src2_ready,  // includes same-cycle CDB bypass
    src3_ready
);
  localparam IDX_W = $clog2(BUSY_TABLE_SZ);
  logic [BUSY_TABLE_SZ-1:0] busy_table_map;

  function automatic bit in_table(input logic [TAG_W-1:0] t);
    return (int'(t) >= BASE) && (int'(t) < BASE + BUSY_TABLE_SZ) && !(ZERO_REG && int'(t) == BASE);
  endfunction
  function automatic bit busy(input logic [TAG_W-1:0] t);
    return in_table(t) && busy_table_map[IDX_W'(int'(t) - BASE)];
  endfunction

  always_comb
    for (int i = 0; i < W; i++) begin
      src1_ready[i] = !busy(src1_tag[i]);
      src2_ready[i] = !busy(src2_tag[i]);
      src3_ready[i] = !busy(src3_tag[i]);
      for (int k = 0; k < CDB_SZ; k++)
      if (cdb_valid[k]) begin
        if (cdb_tag[k] == src1_tag[i]) src1_ready[i] = `TRUE;
        if (cdb_tag[k] == src2_tag[i]) src2_ready[i] = `TRUE;
        if (cdb_tag[k] == src3_tag[i]) src3_ready[i] = `TRUE;
      end
    end


  always_ff @(posedge clock) begin
    if (reset || flush) begin
      for (int i = 0; i < BUSY_TABLE_SZ; i++) busy_table_map[i] <= `FALSE;
    end else begin
      for (int k = 0; k < CDB_SZ; k++)
        if (cdb_valid[k] && in_table(cdb_tag[k])) busy_table_map[IDX_W'(int'(cdb_tag[k]) - BASE)] <= `FALSE;
      for (int i = 0; i < W; i++)
        if (set_en[i] && in_table(set_tag[i])) busy_table_map[IDX_W'(int'(set_tag[i]) - BASE)] <= `TRUE;
    end

  end


endmodule
