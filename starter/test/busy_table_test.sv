/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  busy_table_test.sv                                  //
//                                                                     //
//  Description :  Testbench for the busy table. A reference model     //
//                 holds one busy bit per physical register. Every     //
//                 cycle checks src1_ready/src2_ready on every slot:   //
//                 ready = !busy[tag] or the tag is on a CDB this      //
//                 cycle. Directed tests cover reset, set-then-read,   //
//                 set-and-read in the same cycle, broadcast-then-     //
//                 read, the same-cycle CDB bypass, p0, and multiple   //
//                 sets/broadcasts per cycle; then random traffic that //
//                 follows the core's rules (set tags are free, CDB    //
//                 tags are busy). Prints @@@ Passed/Failed.           //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

module testbench;

    localparam W     = `SUPERSCALAR_WIDTH;
    localparam C     = `NUM_CDB;
    localparam TAG_W = `TAG_W;
    localparam NTAG  = `PHYS_REG_SZ;

    logic                    clock, reset;
    logic [W-1:0]            set_en;
    logic [W-1:0][TAG_W-1:0] set_tag;
    logic [C-1:0]            cdb_valid;
    logic [C-1:0][TAG_W-1:0] cdb_tag;
    logic [W-1:0][TAG_W-1:0] src1_tag, src2_tag;
    logic [W-1:0]            src1_ready, src2_ready;

    busy_table dut (
        .clock     (clock),
        .reset     (reset),
        .set_en    (set_en),
        .set_tag   (set_tag),
        .cdb_valid (cdb_valid),
        .cdb_tag   (cdb_tag),
        .src1_tag  (src1_tag),
        .src2_tag  (src2_tag),
        .src1_ready(src1_ready),
        .src2_ready(src2_ready)
    );

    always #5 clock = ~clock;

    // reference model
    bit busy [NTAG];
    int errors = 0;

    task automatic fail(input string msg);
        $display("  FAIL: %s", msg);
        errors++;
    endtask

    task automatic idle();
        set_en = '0; set_tag = '0;
        cdb_valid = '0; cdb_tag = '0;
        src1_tag = '0; src2_tag = '0;
    endtask

    function automatic bit exp_ready(input logic [TAG_W-1:0] t);
        if (!busy[t]) return 1;
        for (int k = 0; k < C; k++) if (cdb_valid[k] && cdb_tag[k] == t) return 1;
        return 0;
    endfunction

    // check every lookup port against the model, clock, update the model
    task automatic step(input string what);
        #1;
        for (int i = 0; i < W; i++) begin
            if (src1_ready[i] !== exp_ready(src1_tag[i]))
                fail($sformatf("[%s] slot %0d src1 p%0d: ready=%b expected %b",
                               what, i, src1_tag[i], src1_ready[i], exp_ready(src1_tag[i])));
            if (src2_ready[i] !== exp_ready(src2_tag[i]))
                fail($sformatf("[%s] slot %0d src2 p%0d: ready=%b expected %b",
                               what, i, src2_tag[i], src2_ready[i], exp_ready(src2_tag[i])));
        end
        @(posedge clock);
        for (int k = 0; k < C; k++) if (cdb_valid[k]) busy[cdb_tag[k]] = 0;
        for (int i = 0; i < W; i++) if (set_en[i] && set_tag[i] != 0) busy[set_tag[i]] = 1;
        @(negedge clock);
    endtask

    // read every tag through every lookup port and compare with the model
    task automatic check_all(input string what);
        idle();
        for (int t = 0; t < NTAG; t += W) begin
            for (int i = 0; i < W; i++) begin
                src1_tag[i] = TAG_W'((t + i) % NTAG);
                src2_tag[i] = TAG_W'((t + i + 1) % NTAG);
            end
            step(what);
        end
    endtask

    // a random tag that is (busy==want) and not p0, avoiding `avoid`; -1 if none
    function automatic int pick(input bit want, input int avoid [$]);
        int pool [$];
        for (int t = 1; t < NTAG; t++)
            if (busy[t] == want && !(t inside {avoid})) pool.push_back(t);
        return pool.size() == 0 ? -1 : pool[$urandom_range(0, pool.size() - 1)];
    endfunction

    initial begin
        int used [$];
        int t;

        clock = 0;
        idle();
        reset = 1;
        @(negedge clock);
        @(negedge clock);
        reset = 0;

        // 1) after reset everything is ready
        $display("Test 1: everything ready after reset");
        check_all("reset");

        // 2) set p40: ready in the set cycle (bit written at the edge), busy afterwards
        $display("Test 2: set then read");
        idle(); set_en[0] = 1; set_tag[0] = 40; src1_tag[0] = 40;
        step("set p40 + read same cycle");
        idle(); src1_tag[0] = 40; src2_tag[0] = 40;
        step("read p40 busy");

        // 3) broadcast p40, then read: ready
        $display("Test 3: broadcast then read");
        idle(); cdb_valid[0] = 1; cdb_tag[0] = 40;
        step("broadcast p40");
        idle(); src1_tag[0] = 40;
        step("read p40 after broadcast");

        // 4) read in the same cycle as the broadcast: bypass makes it ready
        $display("Test 4: same-cycle broadcast + read (bypass)");
        idle(); set_en[0] = 1; set_tag[0] = 41;
        step("set p41");
        idle(); cdb_valid[C-1] = 1; cdb_tag[C-1] = 41;
        for (int i = 0; i < W; i++) begin src1_tag[i] = 41; src2_tag[i] = 41; end
        step("broadcast + read p41");

        // 5) p0 is never busy, even if set_en targets it
        $display("Test 5: p0 is always ready");
        idle(); set_en[0] = 1; set_tag[0] = 0;
        step("set p0");
        idle(); src1_tag[0] = 0; src2_tag[0] = 0;
        step("read p0");

        // 6) W sets and C broadcasts in one cycle
        $display("Test 6: %0d sets and %0d broadcasts in one cycle", W, C);
        idle();
        for (int i = 0; i < W; i++) begin set_en[i] = 1; set_tag[i] = TAG_W'(50 + i); end
        step("multi set");
        idle();
        for (int k = 0; k < C && k < W; k++) begin cdb_valid[k] = 1; cdb_tag[k] = TAG_W'(50 + k); end
        for (int i = 0; i < W; i++) begin src1_tag[i] = TAG_W'(50 + i); src2_tag[i] = TAG_W'(50 + i); end
        step("multi broadcast + read");
        check_all("after multi");

        // 7) random traffic: set only free tags, broadcast only busy tags (as the core does)
        $display("Test 7: 20000 random cycles");
        for (int c = 0; c < 20000; c++) begin
            idle();
            used.delete();
            for (int k = 0; k < C; k++)
                if ($urandom % 2 == 1) begin
                    t = pick(1, used);
                    if (t >= 0) begin cdb_valid[k] = 1; cdb_tag[k] = TAG_W'(t); used.push_back(t); end
                end
            for (int i = 0; i < W; i++)
                if ($urandom % 2 == 1) begin
                    t = pick(0, used);
                    if (t >= 0) begin set_en[i] = 1; set_tag[i] = TAG_W'(t); used.push_back(t); end
                end
            for (int i = 0; i < W; i++) begin
                // bias lookups toward tags touched this cycle so the bypass gets exercised
                src1_tag[i] = (used.size() > 0 && $urandom % 2 == 1) ? TAG_W'(used[$urandom_range(0, used.size()-1)])
                                                                 : TAG_W'($urandom);
                src2_tag[i] = TAG_W'($urandom);
            end
            step("random");
        end
        check_all("final");

        if (errors == 0) $display("@@@ Passed");
        else             $display("@@@ Failed (%0d errors)", errors);
        $finish;
    end

endmodule
