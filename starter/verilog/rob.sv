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
//                 Each entry also keeps its PC (commit trace) and a   //
//                 halt bit: nothing younger than a committing halt    //
//                 commits in the same cycle (commit_halt).            //
//                 Branch recovery at commit: each entry has the next  //
//                 PC fetch predicted and the actual next PC (PC+4, or //
//                 a branch's result from the CDB). A committing entry //
//                 where they differ raises flush + redirect_pc; it    //
//                 commits, nothing younger does, and at the edge the  //
//                 ROB empties (tail = new head, count = 0).           //
//                 Exceptions (from dispatch, or from the CDB: bad     //
//                 target / address) are raised when the entry commits://
//                 commit_exception, no destination write, nothing     //
//                 younger commits, no flush (the core stops).         //
//                 Replay: the LSQ marks a load that read memory before//
//                 an older overlapping store got its address. At the  //
//                 head it does not commit: flush, re-fetch at its own //
//                 PC (all older stores have committed by then).       //
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
    input [W-1:0][`XLEN-1:0] disp_pc,
    input [W-1:0] disp_halt,  // WFI
    input [W-1:0][`XLEN-1:0] disp_pred_npc,  // next PC fetch predicted
    input EXCEPTION [W-1:0] disp_exception,  // != EXC_NONE: entry is done at dispatch
    input [W-1:0] disp_is_br,  // branch / jump
    input [W-1:0] disp_uncond,  // jal / jalr
    input [W-1:0] disp_is_call,  // RAS push
    input [W-1:0] disp_is_ret,  // RAS pop
    input [W-1:0] disp_is_load,
    input [W-1:0] disp_is_store,
    // memory-order violation (from the LSQ): this load read memory before an older store
    input replay_valid,
    input [PTR_W-1:0] replay_rob_idx,
    output logic [W-1:0][PTR_W-1:0] disp_idx,   // ROB index handed to each slot → RS → FU → CDB
    output logic space_ok,  // ≥ W free entries; must NOT depend on disp_valid


    // complete
    input [CDB_SZ-1:0] cdb_valid,
    input [CDB_SZ-1:0][PTR_W-1:0] cdb_rob_idx,
    input [CDB_SZ-1:0] cdb_branch,  // branch result: cdb_next_pc is the actual next PC
    input [CDB_SZ-1:0][`XLEN-1:0] cdb_next_pc,
    input EXCEPTION [CDB_SZ-1:0] cdb_exception,  // raised by the completing instruction

    // commit
    input commit_stop,  // the core has halted (WFI or exception committed): commit nothing more
    output logic [W-1:0]            commit_valid,
    output logic [W-1:0]            commit_dest_valid,
    output logic [W-1:0][      4:0] commit_rd,
    output logic [W-1:0][TAG_W-1:0] commit_tag,
    commit_told,
    output logic [W-1:0][`XLEN-1:0] commit_pc,
    output logic [W-1:0] commit_is_br,  // committing a branch/jump (no exception): train the predictor
    output logic [W-1:0] commit_uncond,
    output logic [W-1:0] commit_is_call,  // committing a call / return (no exception): committed RAS
    output logic [W-1:0] commit_is_ret,
    output logic [W-1:0] commit_is_load,  // committing a load / store (no exception): LSQ
    output logic [W-1:0] commit_is_store,
    output logic [W-1:0][`XLEN-1:0] commit_next_pc,  // its actual next PC
    output logic [W-1:0] commit_halt,  // this slot commits a halt; no younger slot commits
    output EXCEPTION [W-1:0] commit_exception,  // this slot raises an exception: it does not write
                                                // its destination; nothing younger commits; no flush
    // recovery: a committing instruction whose actual next PC != predicted next PC
    output logic flush,  // squash everything younger (this cycle's edge)
    output logic [`XLEN-1:0] redirect_pc,  // where fetch restarts
    output logic replay_flush  // this flush re-runs a load (statistics)
);
  ROB_PACKET rob_map[ROB_SZ];
  logic [PTR_W-1:0] head_ptr, tail_ptr, commit_idx;

  logic [$clog2(ROB_SZ+1)-1:0] rob_count;
  int n_disp, n_commit;
  logic halt_seen;  // an older slot in this cycle's commit group is a halt or a mispredict


  // ---- combinational: this cycle's outputs ----
  always_comb begin
    n_disp    = 0;
    n_commit  = 0;
    halt_seen = `FALSE;
    flush = `FALSE;
    redirect_pc = '0;
    replay_flush = `FALSE;
    for (int i = 0; i < W; i++) begin
      disp_idx[i] = PTR_W'((int'(tail_ptr) + n_disp) % ROB_SZ);
      commit_idx = PTR_W'((int'(head_ptr) + i) % ROB_SZ);
      commit_valid[i] = (n_commit == i)  // all older slots committed
      && (i < int'(rob_count))  // entry is occupied
      && rob_map[commit_idx].done
      && !halt_seen  // nothing commits after a halt or a mispredict
      && !commit_stop;
      commit_exception[i] = commit_valid[i] ? rob_map[commit_idx].exception : EXC_NONE;
      commit_dest_valid[i] = commit_valid[i] && rob_map[commit_idx].dest_valid
                          && rob_map[commit_idx].exception == EXC_NONE;
      commit_rd[i] = rob_map[commit_idx].rd;
      commit_tag[i] = rob_map[commit_idx].tag;
      commit_told[i] = rob_map[commit_idx].told;
      commit_pc[i] = rob_map[commit_idx].pc;
      commit_is_br[i] = commit_valid[i] && rob_map[commit_idx].is_br
                     && rob_map[commit_idx].exception == EXC_NONE;
      commit_uncond[i] = rob_map[commit_idx].uncond;
      commit_is_call[i] = commit_is_br[i] && rob_map[commit_idx].is_call;
      commit_is_ret[i] = commit_is_br[i] && rob_map[commit_idx].is_ret;
      commit_is_load[i] = commit_valid[i] && rob_map[commit_idx].is_load
                       && rob_map[commit_idx].exception == EXC_NONE;
      commit_is_store[i] = commit_valid[i] && rob_map[commit_idx].is_store
                        && rob_map[commit_idx].exception == EXC_NONE;
      commit_next_pc[i] = rob_map[commit_idx].next_pc;
      commit_halt[i] = commit_valid[i] && rob_map[commit_idx].halt;
      if (commit_valid[i] && rob_map[commit_idx].exception == EXC_NONE
          && rob_map[commit_idx].next_pc != rob_map[commit_idx].pred_npc) begin
        flush = `TRUE;
        redirect_pc = rob_map[commit_idx].next_pc;
      end
      // a load marked for replay does not commit: flush it and everything younger, re-fetch it
      if ((n_commit == i) && (i < int'(rob_count)) && rob_map[commit_idx].replay && !halt_seen && !commit_stop) begin
        commit_valid[i] = `FALSE;
        commit_exception[i] = EXC_NONE;
        commit_dest_valid[i] = `FALSE;
        commit_is_br[i] = `FALSE;
        commit_is_call[i] = `FALSE;
        commit_is_ret[i] = `FALSE;
        commit_is_load[i] = `FALSE;
        commit_is_store[i] = `FALSE;
        commit_halt[i] = `FALSE;
        flush = `TRUE;
        redirect_pc = rob_map[commit_idx].pc;
        replay_flush = `TRUE;
      end
      halt_seen = halt_seen || commit_halt[i] || flush || commit_exception[i] != EXC_NONE;
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
            done: disp_exception[i] != EXC_NONE,   // nothing to execute
            dest_valid: disp_dest_valid[i],
            rd: disp_rd[i],
            tag: disp_tag[i],
            told: disp_told[i],
            pc: disp_pc[i],
            halt: disp_halt[i],
            pred_npc: disp_pred_npc[i],
            next_pc: disp_pc[i] + 4,  // non-branches; branches overwrite at complete
            exception: disp_exception[i],
            is_br: disp_is_br[i],
            uncond: disp_uncond[i],
            is_call: disp_is_call[i],
            is_ret: disp_is_ret[i],
            is_load: disp_is_load[i],
            is_store: disp_is_store[i],
            replay: 1'b0
        };
      head_ptr  <= PTR_W'((int'(head_ptr) + n_commit) % ROB_SZ);
      if (flush) begin
        // squash everything younger than the committing mispredict
        tail_ptr  <= PTR_W'((int'(head_ptr) + n_commit) % ROB_SZ);
        rob_count <= '0;
      end else begin
        tail_ptr  <= PTR_W'((int'(tail_ptr) + n_disp) % ROB_SZ);
        rob_count <= $bits(rob_count)'(int'(rob_count) + n_disp - n_commit);
      end

      if (replay_valid && !flush) rob_map[replay_rob_idx].replay <= `TRUE;

      for (int cdb_idx = 0; cdb_idx < CDB_SZ; cdb_idx++)
      if (cdb_valid[cdb_idx]) begin
        rob_map[cdb_rob_idx[cdb_idx]].done <= `TRUE;
        if (cdb_branch[cdb_idx]) rob_map[cdb_rob_idx[cdb_idx]].next_pc <= cdb_next_pc[cdb_idx];
        if (cdb_exception[cdb_idx] != EXC_NONE) rob_map[cdb_rob_idx[cdb_idx]].exception <= cdb_exception[cdb_idx];
      end
    end
  end


endmodule
