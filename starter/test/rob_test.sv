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
//                 and stale done bits in empty entries; then random   //
//                 traffic with wrap-around. Prints @@@ Passed/Failed. //
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
        .disp_idx         (disp_idx),
        .space_ok         (space_ok),
        .cdb_valid        (cdb_valid),
        .cdb_rob_idx      (cdb_rob_idx),
        .commit_valid     (commit_valid),
        .commit_dest_valid(commit_dest_valid),
        .commit_rd        (commit_rd),
        .commit_tag       (commit_tag),
        .commit_told      (commit_told)
    );

    always #5 clock = ~clock;

    // ---- reference model ----
    typedef struct {
        bit         done;
        bit         dest_valid;
        logic [4:0] rd;
        logic [TAG_W-1:0] tag, told;
    } entry_t;

    entry_t m [SZ];
    int m_head, m_tail, m_count;
    int errors = 0;
    int n_committed = 0;

    task automatic fail(input string msg);
        $display("  FAIL: %s", msg);
        errors++;
    endtask

    task automatic model_reset();
        m_head = 0; m_tail = 0; m_count = 0;
    endtask

    task automatic idle();
        disp_valid = '0; disp_dest_valid = '0; disp_rd = '0; disp_tag = '0; disp_told = '0;
        cdb_valid = '0; cdb_rob_idx = '0;
    endtask

    // set dispatch slot i with random contents
    task automatic disp(input int i, input bit dv = 1);
        disp_valid[i]      = 1;
        disp_dest_valid[i] = dv;
        disp_rd[i]         = 5'($urandom_range(1, 31));
        disp_tag[i]        = TAG_W'($urandom);
        disp_told[i]       = TAG_W'($urandom);
    endtask

    // ROB index of the k-th oldest in-flight entry
    function automatic int nth(input int k);
        return (m_head + k) % SZ;
    endfunction

    // check every output against the model, clock, then update the model
    task automatic step(input string what);
        int n_disp = 0, n_com = 0;
        int idx;
        #1;
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

        // expected commits: consecutive done entries from the head, up to W
        for (int i = 0; i < W; i++) begin
            automatic bit exp = (n_com == i) && (i < m_count) && m[nth(i)].done;
            if (commit_valid[i] !== exp)
                fail($sformatf("[%s] slot %0d commit_valid=%b expected %b (count=%0d)",
                               what, i, commit_valid[i], exp, m_count));
            if (exp) begin
                idx = nth(i);
                if (commit_dest_valid[i] !== m[idx].dest_valid)
                    fail($sformatf("[%s] slot %0d commit_dest_valid wrong (rob %0d)", what, i, idx));
                if (commit_rd[i] !== m[idx].rd || commit_tag[i] !== m[idx].tag || commit_told[i] !== m[idx].told)
                    fail($sformatf("[%s] slot %0d rob %0d: got rd=%0d tag=%0d told=%0d, expected rd=%0d tag=%0d told=%0d",
                                   what, i, idx, commit_rd[i], commit_tag[i], commit_told[i],
                                   m[idx].rd, m[idx].tag, m[idx].told));
                n_com++;
            end
        end

        @(posedge clock);
        // update model: complete, dispatch, commit (complete targets occupied entries only)
        for (int k = 0; k < C; k++) if (cdb_valid[k]) m[cdb_rob_idx[k]].done = 1;
        n_disp = 0;
        for (int i = 0; i < W; i++)
            if (disp_valid[i]) begin
                idx = (m_tail + n_disp) % SZ;
                m[idx] = '{done: 0, dest_valid: disp_dest_valid[i], rd: disp_rd[i],
                           tag: disp_tag[i], told: disp_told[i]};
                n_disp++;
            end
        m_tail  = (m_tail + n_disp) % SZ;
        m_head  = (m_head + n_com) % SZ;
        m_count = m_count + n_disp - n_com;
        n_committed += n_com;
        @(negedge clock);
    endtask

    // complete the k-th oldest in-flight entry on CDB port p
    task automatic complete(input int p, input int k);
        cdb_valid[p] = 1;
        cdb_rob_idx[p] = PTR_W'(nth(k));
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

        // 9) random traffic: random dispatch, random out-of-order completion, many wraps
        $display("Test 9: 20000 random cycles");
        for (int c = 0; c < 20000; c++) begin
            automatic int not_done[$];
            idle();
            if (space_ok)
                for (int i = 0; i < W; i++)
                    if ($urandom % 3 != 0) disp(i, ($urandom % 4) != 0);
            for (int k = 0; k < m_count; k++) if (!m[nth(k)].done) not_done.push_back(k);
            not_done.shuffle();
            for (int p = 0; p < C && p < not_done.size(); p++)
                if ($urandom % 3 != 0) complete(p, not_done[p]);
            step("random");
        end
        drain("final drain");

        $display("  (%0d instructions committed)", n_committed);
        if (errors == 0) $display("@@@ Passed");
        else             $display("@@@ Failed (%0d errors)", errors);
        $finish;
    end

endmodule
