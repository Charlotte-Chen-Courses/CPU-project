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
    logic                       alloc_ok;

    freelist dut (
        .clock    (clock),
        .reset    (reset),
        .alloc_req(alloc_req),
        .alloc_tag(alloc_tag),
        .alloc_ok (alloc_ok),
        .free_en  (free_en),
        .free_tag (free_tag)
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
        alloc_req = '0; free_en = '0; free_tag = '0;
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

        if (errors == 0) $display("@@@ Passed");
        else             $display("@@@ Failed (%0d errors)", errors);
        $finish;
    end

endmodule
