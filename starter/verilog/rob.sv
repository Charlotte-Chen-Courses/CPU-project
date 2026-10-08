/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  rob.sv                                              //
//                                                                     //
//  Description :  Reorder buffer. Circular queue of ROB_SZ entries    //
//                 holding in-flight instructions in program order     //
//                 (no data values: results live in the PRF).          //
//                 Dispatch: writes up to WIDTH entries at the tail;   //
//                 valid slots need not be packed, and each slot gets  //
//                 its ROB index back on disp_idx. space_ok = room     //
//                 for a full bundle, from state only (no dependence   //
//                 on disp_valid). Complete: each CDB marks its entry  //
//                 done by index. Commit: up to WIDTH entries leave    //
//                 from the head, in order, stopping at the first one  //
//                 not done; the count check keeps stale done bits in  //
//                 empty entries from committing. Commit outputs feed  //
//                 the freelist (told) and retirement RAT (rd, tag).   //
//                 No flush / mispredict recovery yet.                 //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

module rob #(
    localparam TAG_W = `TAG_W,
    localparam W = `SUPERSCALAR_WIDTH,
    localparam CDB_SZ = `NUM_CDB,
    localparam ROB_SZ = `ROB_SZ,
    localparam PTR_W = `ROB_IDX_W
) (
    input clock,
    reset,
    // dispatch
    input [W-1:0] disp_valid,  // slot i dispatches this cycle (already gated by rename_fire)
    input [W-1:0] disp_dest_valid,  // from RAT
    input [W-1:0][4:0] disp_rd,
    input [W-1:0][TAG_W-1:0] disp_tag,  // from freelist alloc_tag
    input [W-1:0][TAG_W-1:0] disp_told,  // from RAT told
    output logic [W-1:0][PTR_W-1:0] disp_idx,   // ROB index handed to each slot → RS → FU → CDB
    output logic space_ok,  // ≥ W free entries; must NOT depend on disp_valid


    // complete
    input [CDB_SZ-1:0] cdb_valid,
    input [CDB_SZ-1:0][PTR_W-1:0] cdb_rob_idx,

    // commit
    output logic [W-1:0]            commit_valid,
    output logic [W-1:0]            commit_dest_valid,
    output logic [W-1:0][      4:0] commit_rd,
    output logic [W-1:0][TAG_W-1:0] commit_tag,
    commit_told
);
  ROB_PACKET rob_map[ROB_SZ];
  logic [PTR_W-1:0] head_ptr, tail_ptr, commit_idx;

  logic [$clog2(ROB_SZ+1)-1:0] rob_count;
  int n_disp, n_commit;


  // ---- combinational: this cycle's outputs ----
  always_comb begin
    n_disp   = 0;
    n_commit = 0;
    for (int i = 0; i < W; i++) begin
      disp_idx[i] = PTR_W'((int'(tail_ptr) + n_disp) % ROB_SZ);
      commit_idx = PTR_W'((int'(head_ptr) + i) % ROB_SZ);
      commit_valid[i] = (n_commit == i)  // all older slots committed
      && (i < int'(rob_count))  // entry is occupied
      && rob_map[commit_idx].done;
      commit_dest_valid[i] = commit_valid[i] && rob_map[commit_idx].dest_valid;
      commit_rd[i] = rob_map[commit_idx].rd;
      commit_tag[i] = rob_map[commit_idx].tag;
      commit_told[i] = rob_map[commit_idx].told;
      n_disp += int'(disp_valid[i]);
      n_commit += int'(commit_valid[i]);
    end
  end

  assign space_ok = (int'(rob_count) <= ROB_SZ - W);

  // ---- sequential: next cycle's state ----
  always_ff @(posedge clock) begin
    if (reset) begin
      head_ptr  <= '0;
      tail_ptr  <= '0;
      rob_count <= '0;
    end else begin
      for (int i = 0; i < W; i++)
      if (disp_valid[i])
        rob_map[disp_idx[i]] <= '{
            done: 1'b0,
            dest_valid: disp_dest_valid[i],
            rd: disp_rd[i],
            tag: disp_tag[i],
            told: disp_told[i]
        };
      head_ptr  <= PTR_W'((int'(head_ptr) + n_commit) % ROB_SZ);
      tail_ptr  <= PTR_W'((int'(tail_ptr) + n_disp) % ROB_SZ);
      rob_count <= $bits(rob_count)'(int'(rob_count) + n_disp - n_commit);

      for (int cdb_idx = 0; cdb_idx < CDB_SZ; cdb_idx++)
      if (cdb_valid[cdb_idx]) rob_map[cdb_rob_idx[cdb_idx]].done <= `TRUE;
    end
  end


endmodule
