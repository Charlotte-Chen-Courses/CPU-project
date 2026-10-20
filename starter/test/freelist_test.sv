/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  freelist_test.sv                                    //
//                                                                     //
//  Description :  Testbench for the rename free list. Every cycle is  //
//                 checked against a reference model (a FIFO of free   //
//                 tags): alloc_ok, the exact tag each requesting slot //
//                 receives, and that no tag is ever handed out twice  //
//                 or is preg 0. Directed tests cover reset contents,  //
//                 drain-to-empty, FIFO order, same-cycle alloc+free,  //
//                 and unpacked requests; then random traffic.         //
//                 Prints @@@ Passed/Failed.                           //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

module testbench;

    localparam W     = `SUPERSCALAR_WIDTH;
    localparam TAG_W = $clog2(`PHYS_REG_SZ);
    localparam SZ    = `FREE_LIST_SZ;

    logic                       clock, reset;
    logic [W-1:0]               alloc_req, free_en;
    logic [W-1:0][TAG_W-1:0]    alloc_tag, free_tag;
    logic                       alloc_ok, can_alloc_w;
    logic                       flush;

    freelist dut (
        .clock    (clock),
        .reset    (reset),
        .alloc_req(alloc_req),
        .alloc_tag(alloc_tag),
        .alloc_ok (alloc_ok),
        .free_en  (free_en),
        .free_tag (free_tag),
        .can_alloc_w(can_alloc_w),
        .flush    (flush)
    );

    always #5 clock = ~clock;

    // ---- reference model ----
    int free_q[$];   // free tags, in the order they will be handed out
    int held[$];     // tags not on the list (arch state + in flight); frees come from here
    int errors = 0;
    int got[W];      // tags handed out on the last step, per slot (-1 if none)

    task automatic model_reset();
        free_q.delete();
        held.delete();
        for (int t = 32; t < `PHYS_REG_SZ; t++) free_q.push_back(t);
        for (int t = 1; t < 32; t++) held.push_back(t);  // preg 0 is never freed
    endtask

    task automatic fail(input string msg);
        $display("  FAIL: %s", msg);
        errors++;
    endtask

    // drive one cycle on negedge, check outputs, update model at posedge
    task automatic step(input logic [W-1:0] req, input logic [W-1:0] fen,
                        input int ftag[W]);
        int n_req, k;
        bit exp_ok;

        alloc_req = req;
        free_en   = fen;
        for (int j = 0; j < W; j++) free_tag[j] = fen[j] ? TAG_W'(ftag[j]) : '0;
        #1;

        n_req  = $countones(req);
        exp_ok = (free_q.size() >= n_req);
        if (alloc_ok !== exp_ok)
            fail($sformatf("alloc_ok=%b expected %b (free=%0d req=%b)",
                           alloc_ok, exp_ok, free_q.size(), req));

        k = 0;
        for (int i = 0; i < W; i++) begin
            got[i] = -1;
            if (exp_ok && req[i]) begin
                if (int'(alloc_tag[i]) != free_q[k])
                    fail($sformatf("slot %0d got p%0d expected p%0d (req=%b)",
                                   i, alloc_tag[i], free_q[k], req));
                if (alloc_tag[i] == 0) fail($sformatf("slot %0d got preg 0", i));
                got[i] = int'(alloc_tag[i]);
                k++;
            end
        end

        @(posedge clock);
        if (exp_ok)
            for (int i = 0; i < n_req; i++) held.push_back(free_q.pop_front());
        for (int j = 0; j < W; j++)
            if (fen[j]) begin
                foreach (held[h]) if (held[h] == ftag[j]) begin held.delete(h); break; end
                free_q.push_back(ftag[j]);
            end
        @(negedge clock);
    endtask


    // ---- core-like model, for flush ----
    // In the core, tags are allocated in program order and each commit frees the OLD tag of
    // the oldest in-flight allocation; a flush squashes every in-flight allocation. The
    // random-free model above does not follow that order, so flush is tested here.
    typedef struct { int tag; int told; int rd; } inflight_t;
    inflight_t q[$];          // in-flight allocations, oldest first
    int spec_map [32], arch_map [32];
    int n_flushes = 0;

    task automatic core_reset();
        reset = 1; flush = 0; alloc_req = '0; free_en = '0; free_tag = '0;
        @(negedge clock); @(negedge clock);
        reset = 0;
        model_reset();
        q.delete();
        for (int r = 0; r < 32; r++) begin spec_map[r] = r; arch_map[r] = r; end
    endtask

    // every physical register except p0 is in exactly one place: free list, in flight, or arch
    task automatic check_conservation(input string what);
        int count [`PHYS_REG_SZ];
        foreach (count[t]) count[t] = 0;
        foreach (free_q[i]) count[free_q[i]]++;
        foreach (q[i]) count[q[i].tag]++;
        for (int r = 1; r < 32; r++) count[arch_map[r]]++;
        for (int t = 1; t < `PHYS_REG_SZ; t++)
            if (count[t] != 1) fail($sformatf("[%s] p%0d counted %0d times", what, t, count[t]));
        if (int'(dut.free_count) != free_q.size())
            fail($sformatf("[%s] free_count=%0d, model %0d", what, dut.free_count, free_q.size()));
    endtask

    // one cycle: n_req allocations (if they fit), n_commit commits of the oldest, optional flush
    task automatic core_step(input logic [W-1:0] req, input int n_commit, input bit do_flush,
                             input string what);
        int n_req = $countones(req), k = 0;
        bit exp_ok;
        int frees [$];
        if (n_commit > q.size()) n_commit = q.size();
        if (n_commit > W) n_commit = W;
        alloc_req = req; flush = do_flush; free_en = '0; free_tag = '0;
        for (int j = 0; j < n_commit; j++) begin
            free_en[j] = 1; free_tag[j] = TAG_W'(q[j].told); frees.push_back(q[j].told);
        end
        #1;
        exp_ok = (free_q.size() >= n_req);
        if (can_alloc_w !== (free_q.size() >= W))
            fail($sformatf("[%s] can_alloc_w=%b with %0d free", what, can_alloc_w, free_q.size()));
        if (alloc_ok !== exp_ok)
            fail($sformatf("[%s] alloc_ok=%b expected %b (free=%0d req=%b)", what, alloc_ok, exp_ok, free_q.size(), req));
        for (int i = 0; i < W; i++)
            if (exp_ok && req[i]) begin
                if (int'(alloc_tag[i]) != free_q[k])
                    fail($sformatf("[%s] slot %0d got p%0d expected p%0d", what, i, alloc_tag[i], free_q[k]));
                k++;
            end
        @(posedge clock);
        // commits: oldest first, update architectural map
        for (int j = 0; j < n_commit; j++) begin
            arch_map[q[0].rd] = q[0].tag;
            void'(q.pop_front());
        end
        if (do_flush) begin
            // ring after the rewind: squashed in-flight tags (allocation order), then the
            // old free list, then this cycle's frees
            int nq [$];
            foreach (q[i]) nq.push_back(q[i].tag);
            foreach (free_q[i]) nq.push_back(free_q[i]);
            foreach (frees[i]) nq.push_back(frees[i]);
            free_q = nq;
            q.delete();
            spec_map = arch_map;
            n_flushes++;
        end else begin
            if (exp_ok)
                for (int i = 0; i < W; i++)
                    if (req[i]) begin
                        automatic int rd = $urandom_range(1, 31);
                        automatic int t = free_q.pop_front();
                        q.push_back('{tag: t, told: spec_map[rd], rd: rd});
                        spec_map[rd] = t;
                    end
            foreach (frees[i]) free_q.push_back(frees[i]);
        end
        @(negedge clock);
        alloc_req = '0; free_en = '0; free_tag = '0; flush = 0;
        check_conservation(what);
    endtask

    // a real core can only free tags that are currently handed out
    function automatic int room();
        return SZ - free_q.size();
    endfunction

    // pick a random held tag to free (removes nothing; step() does that)
    function automatic int pick_held(input int avoid[$]);
        int t;
        do t = held[$urandom % held.size()];
        while (t inside {avoid});
        return t;
    endfunction

    int none[W];
    int ft[W];
    int first_freed[$];
    int empty_q[$];
    int n_refused = 0;

    initial begin
        clock = 0;
        reset = 1;
        alloc_req = '0; free_en = '0; free_tag = '0; flush = 0;
        foreach (none[j]) none[j] = 0;
        model_reset();
        @(negedge clock);
        @(negedge clock);
        reset = 0;

        // 1) reset contents: hands out p32, p33, ... in order, one slot at a time
        $display("Test 1: reset contents come out in order (p32..p%0d)", `PHYS_REG_SZ - 1);
        for (int n = 0; n < SZ; n++) begin
            step(W'(1), '0, none);
            if (got[0] != 32 + n) fail($sformatf("alloc #%0d got p%0d expected p%0d", n, got[0], 32 + n));
        end

        // 2) empty: request must be refused and must not consume anything
        $display("Test 2: empty list refuses requests");
        if (free_q.size() != 0) fail("model not empty after draining");
        step('1, '0, none);
        if (alloc_ok) fail("alloc_ok high on empty list");
        step('0, '0, none);
        if (!alloc_ok) fail("alloc_ok low with no request");

        // 3) FIFO order: freed tags come back in the order they were freed
        $display("Test 3: freed tags come back in FIFO order");
        first_freed = {40, 33, 50};
        foreach (first_freed[n]) begin
            ft = none; ft[0] = first_freed[n];
            step('0, W'(1), ft);
        end
        foreach (first_freed[n]) begin
            step(W'(1), '0, none);
            if (got[0] != first_freed[n])
                fail($sformatf("FIFO order: got p%0d expected p%0d", got[0], first_freed[n]));
        end

        // 4) same-cycle alloc + free on an empty list: alloc uses this cycle's count
        $display("Test 4: same-cycle alloc + free");
        ft = none; ft[0] = pick_held(empty_q);
        step(W'(1), W'(1), ft);   // list empty: alloc refused, free lands
        step(W'(1), '0, none);    // now the freed tag is handed out
        if (got[0] != ft[0]) fail($sformatf("got p%0d expected freed p%0d", got[0], ft[0]));

        // 5) reset again, half-drain so frees have room, then every request pattern
        //    (unpacked ones too)
        $display("Test 5: every alloc_req / free_en pattern");
        model_reset();
        reset = 1; @(negedge clock); reset = 0;
        for (int n = 0; n < SZ / 2; n++) step(W'(1), '0, none);
        for (int p = 0; p < (1 << W); p++)
            for (int q = 0; q < (1 << W); q++) begin
                automatic int avoid[$];
                automatic logic [W-1:0] fen = '0;
                ft = none;
                for (int j = 0; j < W; j++)
                    if (q[j] && avoid.size() < room()) begin
                        fen[j] = 1; ft[j] = pick_held(avoid); avoid.push_back(ft[j]);
                    end
                step(W'(p), fen, ft);
            end

        // 6) random traffic, biased so the list drains and refills repeatedly
        $display("Test 6: 20000 random cycles");
        for (int c = 0; c < 20000; c++) begin
            automatic int avoid[$];
            automatic logic [W-1:0] req, fen;
            automatic bit phase = bit'((c / 500) % 2);   // alternate alloc-heavy and free-heavy phases
            req = '0; fen = '0; ft = none;
            for (int i = 0; i < W; i++) req[i] = ($urandom % 4) < (phase ? 1 : 3);
            for (int j = 0; j < W; j++)
                if (($urandom % 4) < (phase ? 3 : 1) && avoid.size() < room()) begin
                    fen[j] = 1; ft[j] = pick_held(avoid); avoid.push_back(ft[j]);
                end
            step(req, fen, ft);
            if (req != 0 && !alloc_ok) n_refused++;
        end
        $display("  (%0d cycles hit an empty list)", n_refused);

        if (free_q.size() + held.size() != `PHYS_REG_SZ - 1)
            fail($sformatf("tag conservation: free=%0d held=%0d", free_q.size(), held.size()));

        // 7) flush, directed (core-like order)
        $display("Test 7: flush rewinds every uncommitted tag (directed)");
        core_reset();
        for (int c = 0; c < 5; c++) core_step('1, 0, 0, "alloc");
        core_step('0, 0, 1, "flush, no frees");
        for (int c = 0; c < 8; c++) core_step('1, 0, 0, "alloc again");
        core_step('0, W, 1, "flush with W frees in the same cycle");
        while (free_q.size() >= W) core_step('1, 0, 0, "drain the list");
        core_step('0, 1, 1, "flush with the list nearly empty");
        core_step('0, 0, 1, "back-to-back flush");
        core_step('1, 0, 0, "alloc after flushes");

        // 8) flush, random (core-like order)
        $display("Test 8: 20000 random cycles with commits and flushes");
        core_reset();
        for (int c = 0; c < 20000; c++) begin
            automatic bit f = ($urandom % 40 == 0);
            core_step(f ? '0 : W'($urandom), $urandom_range(0, W), f, "random");
            if (errors > 20) break;
        end
        $display("  (%0d flushes)", n_flushes);

        if (errors == 0) $display("@@@ Passed");
        else             $display("@@@ Failed (%0d errors)", errors);
        $finish;
    end

endmodule
