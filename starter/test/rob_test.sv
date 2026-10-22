/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  rob_test.sv                                         //
//                                                                     //
//  Description :  Testbench for the reorder buffer (dispatch,         //
//                 complete, commit). A reference model tracks every   //
//                 entry by ROB index plus head/tail/count. Every      //
//                 cycle checks disp_idx, space_ok, and each commit    //
//                 slot (valid, dest_valid, rd, tag, told). Directed   //
//                 tests cover fill-to-full, out-of-order completion,  //
//                 head-blocking, unpacked dispatch, no-dest commits,  //
//                 and stale done bits in empty entries, and load      //
//                 replay (a marked entry flushes at the head and      //
//                 re-fetches its own PC); every cycle also checks     //
//                 head/empty and each commit's dest class and fflags  //
//                 (0 unless it commits without an exception); then    //
//                 random traffic with                                 //
//                 wrap-around. Prints @@@ Passed/Failed.              //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

module testbench;

    localparam W      = `SUPERSCALAR_WIDTH;
    localparam C      = `NUM_CDB;
    localparam TAG_W  = `TAG_W;
    localparam SZ     = `ROB_SZ;
    localparam PTR_W  = $clog2(SZ);

    logic                    clock, reset;
    logic [W-1:0]            disp_valid, disp_dest_valid;
    logic [W-1:0][4:0]       disp_rd;
    logic [W-1:0][TAG_W-1:0] disp_tag, disp_told;
    logic [W-1:0][`XLEN-1:0] disp_pc, commit_pc;
    logic [W-1:0]            disp_halt, commit_halt;
    logic [W-1:0][`XLEN-1:0] disp_pred_npc;
    logic [C-1:0]            cdb_branch;
    EXCEPTION [C-1:0]        cdb_exception;
    EXCEPTION [W-1:0]        disp_exception, commit_exception;
    logic [W-1:0]            disp_is_br, disp_uncond, commit_is_br, commit_uncond;
    logic [W-1:0]            disp_is_call, disp_is_ret, commit_is_call, commit_is_ret;
    logic [W-1:0]            disp_is_load, disp_is_store, commit_is_load, commit_is_store;
    logic [W-1:0]            disp_dest_fp, commit_dest_fp;
    logic [C-1:0][4:0]       cdb_fflags;
    logic [W-1:0][4:0]       commit_fflags;
    logic [PTR_W-1:0]        head;
    logic                    empty;
    logic [W-1:0][`XLEN-1:0] commit_next_pc;
    logic [C-1:0][`XLEN-1:0] cdb_next_pc;
    logic                    flush, replay_valid, replay_flush;
    logic [`XLEN-1:0]        redirect_pc;
    logic [PTR_W-1:0]        replay_rob_idx;
    logic [W-1:0][PTR_W-1:0] disp_idx;
    logic                    space_ok;
    logic [C-1:0]            cdb_valid;
    logic [C-1:0][PTR_W-1:0] cdb_rob_idx;
    logic [W-1:0]            commit_valid, commit_dest_valid;
    logic [W-1:0][4:0]       commit_rd;
    logic [W-1:0][TAG_W-1:0] commit_tag, commit_told;

    rob dut (
        .clock            (clock),
        .reset            (reset),
        .disp_valid       (disp_valid),
        .disp_dest_valid  (disp_dest_valid),
        .disp_rd          (disp_rd),
        .disp_tag         (disp_tag),
        .disp_told        (disp_told),
        .disp_pc          (disp_pc),
        .disp_halt        (disp_halt),
        .disp_pred_npc    (disp_pred_npc),
        .disp_exception   (disp_exception),
        .disp_is_br       (disp_is_br),
        .disp_uncond      (disp_uncond),
        .disp_is_call     (disp_is_call),
        .disp_is_ret      (disp_is_ret),
        .disp_is_load     (disp_is_load),
        .disp_is_store    (disp_is_store),
        .disp_dest_fp     (disp_dest_fp),
        .replay_valid     (replay_valid),
        .replay_rob_idx   (replay_rob_idx),
        .disp_idx         (disp_idx),
        .space_ok         (space_ok),
        .cdb_valid        (cdb_valid),
        .cdb_rob_idx      (cdb_rob_idx),
        .cdb_branch       (cdb_branch),
        .cdb_next_pc      (cdb_next_pc),
        .cdb_exception    (cdb_exception),
        .cdb_fflags       (cdb_fflags),
        .commit_valid     (commit_valid),
        .commit_dest_valid(commit_dest_valid),
        .commit_rd        (commit_rd),
        .commit_tag       (commit_tag),
        .commit_told      (commit_told),
        .commit_pc        (commit_pc),
        .commit_is_br     (commit_is_br),
        .commit_uncond    (commit_uncond),
        .commit_is_call   (commit_is_call),
        .commit_is_ret    (commit_is_ret),
        .commit_is_load   (commit_is_load),
        .commit_is_store  (commit_is_store),
        .commit_next_pc   (commit_next_pc),
        .commit_halt      (commit_halt),
        .commit_exception (commit_exception),
        .commit_stop      (1'b0),
        .flush            (flush),
        .redirect_pc      (redirect_pc),
        .replay_flush     (replay_flush),
        .commit_dest_fp   (commit_dest_fp),
        .commit_fflags    (commit_fflags),
        .head             (head),
        .empty            (empty)
    );

    always #5 clock = ~clock;

    // ---- reference model ----
    typedef struct {
        bit         done;
        bit         dest_valid;
        logic [4:0] rd;
        logic [TAG_W-1:0] tag, told;
        logic [`XLEN-1:0] pc;
        bit         halt;
        logic [`XLEN-1:0] pred_npc, next_pc;
        EXCEPTION   exc;
        bit         is_br, uncond, is_call, is_ret, is_load, is_store;
        bit         replay;
        bit         dest_fp;
        logic [4:0] fflags;
    } entry_t;

    entry_t m [SZ];
    int m_head, m_tail, m_count;
    int errors = 0;
    int n_committed = 0, n_flushes = 0, n_replays = 0;

    task automatic fail(input string msg);
        $display("  FAIL: %s", msg);
        errors++;
    endtask

    task automatic model_reset();
        m_head = 0; m_tail = 0; m_count = 0;
    endtask

    task automatic idle();
        disp_valid = '0; disp_dest_valid = '0; disp_rd = '0; disp_tag = '0; disp_told = '0;
        disp_pc = '0; disp_halt = '0; disp_pred_npc = '0;
        cdb_valid = '0; cdb_rob_idx = '0; cdb_branch = '0; cdb_next_pc = '0; cdb_exception = '{default: EXC_NONE};
        disp_exception = '{default: EXC_NONE};
        disp_is_br = '0; disp_uncond = '0; disp_is_call = '0; disp_is_ret = '0;
        disp_is_load = '0; disp_is_store = '0;
        replay_valid = 0; replay_rob_idx = '0;
        disp_dest_fp = '0; cdb_fflags = '0;
    endtask

    // set dispatch slot i with random contents
    task automatic disp(input int i, input bit dv = 1);
        disp_valid[i]      = 1;
        disp_dest_valid[i] = dv;
        disp_rd[i]         = 5'($urandom_range(1, 31));
        disp_tag[i]        = TAG_W'($urandom);
        disp_told[i]       = TAG_W'($urandom);
        disp_pc[i]         = $urandom;
        disp_halt[i]       = 0;
        disp_pred_npc[i]   = disp_pc[i] + 4;     // correct unless a test changes it
        disp_is_br[i]      = ($urandom % 4 == 0);
        disp_uncond[i]     = disp_is_br[i] && ($urandom % 2 == 1);
        disp_is_call[i]    = disp_uncond[i] && ($urandom % 3 == 0);
        disp_is_ret[i]     = disp_uncond[i] && !disp_is_call[i] && ($urandom % 2 == 1);
        disp_is_load[i]    = !disp_is_br[i] && ($urandom % 4 == 0);
        disp_is_store[i]   = !disp_is_br[i] && !disp_is_load[i] && ($urandom % 3 == 0);
        disp_dest_fp[i]    = dv && ($urandom % 3 == 0);
    endtask

    // ROB index of the k-th oldest in-flight entry
    function automatic int nth(input int k);
        return (m_head + k) % SZ;
    endfunction

    // check every output against the model, clock, then update the model
    task automatic step(input string what);
        int n_disp = 0, n_com = 0;
        int idx;
        bit exp_flush, exp_replay;
        logic [`XLEN-1:0] exp_redirect;
        #1;
        if (head !== PTR_W'(m_head) || empty !== (m_count == 0))
            fail($sformatf("[%s] head=%0d empty=%b, expected %0d %b", what, head, empty, m_head, m_count == 0));
        if (space_ok !== (m_count <= SZ - W))
            fail($sformatf("[%s] space_ok=%b with %0d entries", what, space_ok, m_count));
        if (disp_valid != 0 && m_count > SZ - W)
            fail($sformatf("[%s] testbench dispatched while full", what));

        for (int i = 0; i < W; i++) begin
            if (disp_idx[i] !== PTR_W'((m_tail + n_disp) % SZ))
                fail($sformatf("[%s] slot %0d disp_idx=%0d expected %0d",
                               what, i, disp_idx[i], (m_tail + n_disp) % SZ));
            n_disp += int'(disp_valid[i]);
        end

        // expected commits: consecutive done entries from the head, up to W, and nothing
        // younger than a halt or a mispredict (actual next PC != predicted) in the same cycle;
        // the first entry marked for replay does not commit: flush, redirect to its own PC
        exp_flush = 0; exp_replay = 0;
        for (int i = 0; i < W; i++) begin
            automatic bit halt_before = 0;
            automatic bit exp;
            for (int j = 0; j < i; j++)
                if (commit_valid[j] && (m[nth(j)].halt || m[nth(j)].exc != EXC_NONE ||
                                        m[nth(j)].next_pc != m[nth(j)].pred_npc)) halt_before = 1;
            if ((n_com == i) && (i < m_count) && m[nth(i)].replay && !halt_before && !exp_flush) begin
                exp_flush = 1; exp_replay = 1; exp_redirect = m[nth(i)].pc;
            end
            exp = (n_com == i) && (i < m_count) && m[nth(i)].done && !halt_before && !exp_replay;
            if (!exp && commit_fflags[i] !== 5'b0)
                fail($sformatf("[%s] slot %0d commit_fflags=%b on a slot that does not commit", what, i, commit_fflags[i]));
            if (commit_valid[i] !== exp)
                fail($sformatf("[%s] slot %0d commit_valid=%b expected %b (count=%0d)",
                               what, i, commit_valid[i], exp, m_count));
            if (exp) begin
                idx = nth(i);
                if (commit_exception[i] !== m[idx].exc)
                    fail($sformatf("[%s] slot %0d rob %0d: commit_exception=%s expected %s", what, i, idx,
                                   commit_exception[i].name(), m[idx].exc.name()));
                if (commit_dest_valid[i] !== (m[idx].dest_valid && m[idx].exc == EXC_NONE))
                    fail($sformatf("[%s] slot %0d commit_dest_valid wrong (rob %0d)", what, i, idx));
                if (commit_rd[i] !== m[idx].rd || commit_tag[i] !== m[idx].tag || commit_told[i] !== m[idx].told)
                    fail($sformatf("[%s] slot %0d rob %0d: got rd=%0d tag=%0d told=%0d, expected rd=%0d tag=%0d told=%0d",
                                   what, i, idx, commit_rd[i], commit_tag[i], commit_told[i],
                                   m[idx].rd, m[idx].tag, m[idx].told));
                if (commit_is_br[i] !== (m[idx].is_br && m[idx].exc == EXC_NONE) ||
                    (commit_is_br[i] && (commit_uncond[i] !== m[idx].uncond || commit_next_pc[i] !== m[idx].next_pc)))
                    fail($sformatf("[%s] slot %0d rob %0d: predictor update is_br=%b uncond=%b npc=%h, expected %b %b %h",
                                   what, i, idx, commit_is_br[i], commit_uncond[i], commit_next_pc[i],
                                   m[idx].is_br && m[idx].exc == EXC_NONE, m[idx].uncond, m[idx].next_pc));
                if (commit_is_call[i] !== (m[idx].is_br && m[idx].exc == EXC_NONE && m[idx].is_call) ||
                    commit_is_ret[i]  !== (m[idx].is_br && m[idx].exc == EXC_NONE && m[idx].is_ret))
                    fail($sformatf("[%s] slot %0d rob %0d: commit call/ret=%b%b expected %b%b", what, i, idx,
                                   commit_is_call[i], commit_is_ret[i], m[idx].is_call, m[idx].is_ret));
                if (commit_is_load[i]  !== (m[idx].is_load  && m[idx].exc == EXC_NONE) ||
                    commit_is_store[i] !== (m[idx].is_store && m[idx].exc == EXC_NONE))
                    fail($sformatf("[%s] slot %0d rob %0d: commit load/store=%b%b expected %b%b", what, i, idx,
                                   commit_is_load[i], commit_is_store[i], m[idx].is_load, m[idx].is_store));
                if (commit_pc[i] !== m[idx].pc)
                    fail($sformatf("[%s] slot %0d rob %0d: commit_pc=%h expected %h", what, i, idx, commit_pc[i], m[idx].pc));
                if (commit_dest_fp[i] !== m[idx].dest_fp)
                    fail($sformatf("[%s] slot %0d rob %0d: commit_dest_fp=%b expected %b", what, i, idx, commit_dest_fp[i], m[idx].dest_fp));
                if (commit_fflags[i] !== ((m[idx].exc == EXC_NONE) ? m[idx].fflags : 5'b0))
                    fail($sformatf("[%s] slot %0d rob %0d: commit_fflags=%b expected %b", what, i, idx, commit_fflags[i],
                                   (m[idx].exc == EXC_NONE) ? m[idx].fflags : 5'b0));
                if (commit_halt[i] !== m[idx].halt)
                    fail($sformatf("[%s] slot %0d rob %0d: commit_halt=%b expected %b", what, i, idx, commit_halt[i], m[idx].halt));
                if (!exp_flush && m[idx].exc == EXC_NONE && m[idx].next_pc != m[idx].pred_npc) begin
                    exp_flush = 1; exp_redirect = m[idx].next_pc;
                end
                n_com++;
            end
        end
        if (flush !== exp_flush)
            fail($sformatf("[%s] flush=%b expected %b", what, flush, exp_flush));
        if (replay_flush !== exp_replay)
            fail($sformatf("[%s] replay_flush=%b expected %b", what, replay_flush, exp_replay));
        else if (exp_flush && redirect_pc !== exp_redirect)
            fail($sformatf("[%s] redirect_pc=%h expected %h", what, redirect_pc, exp_redirect));

        @(posedge clock);
        // update model: replay mark, complete, dispatch, commit (targets occupied entries only)
        if (replay_valid && !exp_flush) m[replay_rob_idx].replay = 1;
        n_replays += int'(exp_replay);
        for (int k = 0; k < C; k++)
            if (cdb_valid[k]) begin
                m[cdb_rob_idx[k]].done = 1;
                if (cdb_branch[k]) m[cdb_rob_idx[k]].next_pc = cdb_next_pc[k];
                if (cdb_exception[k] != EXC_NONE) m[cdb_rob_idx[k]].exc = cdb_exception[k];
                m[cdb_rob_idx[k]].fflags = cdb_fflags[k];
            end
        n_disp = 0;
        for (int i = 0; i < W; i++)
            if (disp_valid[i]) begin
                idx = (m_tail + n_disp) % SZ;
                m[idx] = '{done: disp_exception[i] != EXC_NONE, dest_valid: disp_dest_valid[i], rd: disp_rd[i],
                           tag: disp_tag[i], told: disp_told[i], pc: disp_pc[i], halt: disp_halt[i],
                           pred_npc: disp_pred_npc[i], next_pc: disp_pc[i] + 4, exc: disp_exception[i],
                           is_br: disp_is_br[i], uncond: disp_uncond[i],
                           is_call: disp_is_call[i], is_ret: disp_is_ret[i],
                           is_load: disp_is_load[i], is_store: disp_is_store[i], replay: 0,
                           dest_fp: disp_dest_fp[i], fflags: '0};
                n_disp++;
            end
        m_head  = (m_head + n_com) % SZ;
        if (exp_flush) begin                // everything younger is squashed
            m_tail  = m_head;
            m_count = 0;
            n_flushes++;
        end else begin
            m_tail  = (m_tail + n_disp) % SZ;
            m_count = m_count + n_disp - n_com;
        end
        n_committed += n_com;
        @(negedge clock);
    endtask

    // complete the k-th oldest in-flight entry on CDB port p
    task automatic complete(input int p, input int k);
        cdb_valid[p] = 1;
        cdb_rob_idx[p] = PTR_W'(nth(k));
    endtask

    // complete it as a branch whose actual next PC is npc
    task automatic complete_br(input int p, input int k, input logic [`XLEN-1:0] npc);
        complete(p, k);
        cdb_branch[p]  = 1;
        cdb_next_pc[p] = npc;
    endtask

    // complete everything in flight (C per cycle, youngest first), then let it all commit
    task automatic drain(input string what);
        int k = m_count - 1;
        while (k >= 0) begin
            idle();
            for (int p = 0; p < C && k >= 0; p++, k--) complete(p, k);
            step(what);
        end
        idle();
        while (m_count > 0) step(what);
    endtask

    initial begin
        clock = 0;
        idle();
        reset = 1;
        model_reset();
        @(negedge clock);
        @(negedge clock);
        reset = 0;

        // 1) empty after reset: space, no commits
        $display("Test 1: empty after reset");
        idle();
        step("reset");

        // 2) fill W per cycle until space_ok drops; nothing commits (nothing done)
        $display("Test 2: fill to full (space_ok drops at %0d entries)", SZ - W + 1);
        while (space_ok) begin
            idle();
            for (int i = 0; i < W; i++) disp(i);
            step("fill");
        end
        if (m_count != SZ) fail($sformatf("filled to %0d entries, expected %0d", m_count, SZ));
        idle();
        step("full, idle");

        // 3) complete youngest-first: nothing may commit until the head is done
        $display("Test 3: out-of-order completion, head blocks commit");
        for (int k = m_count - 1; k >= 1; k -= C) begin
            idle();
            for (int p = 0; p < C && k - p >= 1; p++) complete(p, k - p);
            step("complete young");
            if (commit_valid != 0) fail("committed while head not done");
        end
        idle(); complete(0, 0);
        step("complete head");
        idle();
        while (m_count > 0) step("drain");

        // 4) stale done bits: every entry now has done=1 from Test 3; one real entry
        //    must commit alone, never together with the stale entry behind it
        $display("Test 4: stale done bits in empty entries are ignored");
        idle(); disp(0);
        step("dispatch one");
        idle(); complete(0, 0);
        step("complete one");
        idle();
        step("commit one");
        idle();
        step("empty again");

        if (W >= 2) begin
            // 5) slot 1 done, slot 0 not: nothing commits
            $display("Test 5: younger done, older not done -> blocked");
            idle(); disp(0); disp(1);
            step("dispatch pair");
            idle(); complete(0, 1);
            step("complete slot 1 only");
            idle();
            step("blocked");
            idle(); complete(0, 0);
            step("complete slot 0");
            idle();
            step("both commit");

            // 6) unpacked dispatch: only slot 1 valid
            $display("Test 6: unpacked dispatch (disp_valid = 2'b10)");
            idle(); disp(1);
            step("slot 1 only");
            drain("drain unpacked");

            // 7) dispatch and commit in the same cycle
            $display("Test 7: dispatch + commit in the same cycle");
            idle(); disp(0); disp(1);
            step("dispatch pair");
            idle(); complete(0, 0); if (C > 1) complete(1, 1);
            step("complete pair");
            idle(); if (C == 1) complete(0, 1);
            disp(0); disp(1);
            step("commit + dispatch");
            drain("drain");
        end

        // 8) no-dest instructions (stores/branches) commit with commit_dest_valid = 0
        $display("Test 8: no-dest instructions commit without freeing");
        idle(); disp(0, 0);
        step("dispatch store");
        drain("drain store");

        // 9) halt: an entry younger than a halt must not commit in the same cycle
        if (W >= 2) begin
            $display("Test 9: nothing commits after a halt in the same cycle");
            idle(); disp(0, 0); disp_halt[0] = 1; disp(1);
            step("dispatch halt + younger");
            idle(); complete(0, 0); if (C > 1) complete(1, 1);
            step("complete both");
            idle(); if (C == 1) complete(0, 1);
            step("halt commits alone");
            drain("drain after halt");
        end

        // 10) mispredict in slot 0: it commits, the done slot 1 does not, the ROB empties
        if (W >= 2) begin
            $display("Test 10: mispredict in slot 0 squashes a done slot 1");
            idle(); disp(0); disp(1);
            step("dispatch pair");
            idle(); complete_br(0, 0, 32'h1000); if (C > 1) complete(1, 1);
            step("complete");
            idle(); if (C == 1) complete(0, 1);
            step("mispredict commits alone, flush");
            idle(); step("empty after flush");

            $display("Test 11: mispredict in slot 1: both commit, then flush");
            idle(); disp(0); disp(1);
            step("dispatch pair");
            idle(); complete(0, 0); if (C > 1) complete_br(1, 1, 32'h2000);
            step("complete");
            idle(); if (C == 1) complete_br(0, 1, 32'h2000);
            step("both commit, flush");
            idle(); step("empty after flush");
        end

        // 12) mispredict at the head of a full ROB: everything behind it goes
        $display("Test 12: mispredict while the ROB is full");
        while (space_ok) begin idle(); for (int i = 0; i < W; i++) disp(i); step("fill"); end
        idle(); complete_br(0, 0, 32'h3000); if (C > 1) complete(1, 1);
        step("complete head as mispredict");
        idle(); step("flush");
        idle(); step("empty after flush");
        idle(); for (int i = 0; i < W; i++) disp(i); step("dispatch after flush");
        drain("drain after flush");

        // 13) exceptions: done at dispatch, raised at commit, no destination, nothing younger, no flush
        if (W >= 2) begin
            $display("Test 13: exception at dispatch commits alone, writes nothing, no flush");
            idle(); disp(0); disp_exception[0] = EXC_UNSUPPORTED; disp(1);
            step("dispatch exception + younger");
            idle(); complete(0, 1);
            step("complete the younger one (the exception entry is already done)");
            idle(); step("exception commits alone");
            drain("drain after exception");

            $display("Test 14: misaligned branch beats its own mispredict");
            idle(); disp(0); disp(1);
            step("dispatch pair");
            idle(); complete_br(0, 0, 32'h4002); cdb_exception[0] = EXC_MISALIGNED; if (C > 1) complete(1, 1);
            step("complete misaligned");
            idle(); if (C == 1) complete(0, 1);
            step("exception, not flush");
            drain("drain after misaligned");
        end

        // 15) replay: entry 2 is marked while entry 0 (its older store) is still executing;
        //     0 and 1 commit, then 2 flushes without committing and re-fetches its own PC
        $display("Test 15: replay at the head flushes and re-fetches the load's PC");
        idle(); for (int i = 0; i < W; i++) disp(i);
        step("dispatch");
        idle(); for (int i = 0; i < W; i++) disp(i);
        step("dispatch");
        idle(); replay_valid = 1; replay_rob_idx = PTR_W'(nth(2));
        for (int p = 0; p < C; p++) complete(p, 3 - p);
        step("mark entry 2 (younger ones complete)");
        idle(); complete(0, 1); if (C > 1) complete(1, 2);
        step("complete 1 and the marked load");
        if (commit_valid != 0) fail("committed before the older store completed");
        idle(); complete(0, 0);
        step("store completes");
        for (int g = 0; g < 4 && n_replays == 0; g++) begin idle(); step("commit up to the replayed load"); end
        if (m_count != 0) fail("replay did not empty the ROB");
        if (n_replays != 1) fail($sformatf("%0d replays, expected 1", n_replays));
        drain("drain after replay");

        $display("Test 16: a mark in the cycle of a mispredict flush is ignored");
        idle(); disp(0); if (W > 1) disp(1, 0);
        step("dispatch");
        idle(); for (int i = 0; i < W; i++) disp(i);
        step("dispatch");
        idle(); complete_br(0, 0, 32'h2000);
        step("mispredict completes");
        idle(); replay_valid = 1; replay_rob_idx = PTR_W'(nth(2));
        step("mark during the flush");
        idle(); for (int i = 0; i < W; i++) disp(i);
        step("dispatch into the reused entries");
        drain("no replay may happen");
        if (n_replays != 1) fail("a mark during a flush survived");

        // 17) random traffic: dispatch, out-of-order completion, mispredicts, exceptions, replays, many wraps
        $display("Test 17: 20000 random cycles");
        for (int c = 0; c < 20000; c++) begin
            automatic int not_done[$];
            idle();
            if (space_ok)
                for (int i = 0; i < W; i++)
                    if ($urandom % 3 != 0) begin
                        disp(i, ($urandom % 4) != 0);
                        disp_halt[i] = ($urandom % 16 == 0);
                        if ($urandom % 8 == 0) disp_pred_npc[i] = $urandom;   // predicted a jump
                        if ($urandom % 24 == 0) disp_exception[i] = ($urandom % 2 == 1) ? EXC_ILLEGAL : EXC_UNSUPPORTED;
                    end
            for (int k = 0; k < m_count; k++) if (!m[nth(k)].done) not_done.push_back(k);
            not_done.shuffle();
            // the LSQ marks a load that has not committed (its older store is just executing)
            if (not_done.size() > 0 && $urandom % 24 == 0) begin
                replay_valid = 1; replay_rob_idx = PTR_W'(nth(not_done[$urandom_range(0, not_done.size() - 1)]));
            end
            for (int p = 0; p < C && p < not_done.size(); p++)
                if ($urandom % 3 != 0) begin
                    automatic int k = not_done[p];
                    // a branch result: usually agrees with the prediction, sometimes not
                    if ($urandom % 4 == 0) begin
                        complete_br(p, k, ($urandom % 3 == 0) ? $urandom : m[nth(k)].pred_npc);
                        cdb_exception[p] = ($urandom % 16 == 0) ? EXC_MISALIGNED : ($urandom % 32 == 0) ? EXC_ACCESS : EXC_NONE;
                    end
                    else complete(p, k);
                    cdb_fflags[p] = 5'($urandom);
                end
            step("random");
        end
        drain("final drain");

        $display("  (%0d instructions committed, %0d flushes, %0d replays)", n_committed, n_flushes, n_replays);
        if (errors == 0) $display("@@@ Passed");
        else             $display("@@@ Failed (%0d errors)", errors);
        $finish;
    end

endmodule
