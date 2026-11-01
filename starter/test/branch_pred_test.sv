/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  branch_pred_test.sv                                 //
//                                                                     //
//  Description :  Testbench for the BTB + bimodal (or, BP_TYPE = 1,   //
//                 tournament: + gshare and chooser) predictor. The    //
//                 model keeps its own BTB and counter tables and      //
//                 applies updates one slot at a time in program       //
//                 order. Every cycle checks btb_hit / pred_taken /    //
//                 pred_target on every lookup slot. PCs come from a   //
//                 small pool that includes PCs sharing a BTB index    //
//                 (different tag) and sharing a counter, so hits,     //
//                 replacements and same-cycle counter collisions all  //
//                 happen. Directed: reset, counter hysteresis, jumps  //
//                 always taken, replacement, two updates to one       //
//                 counter in a cycle. Prints @@@ Passed/Failed.       //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

module testbench;

    localparam W      = `SUPERSCALAR_WIDTH;
    localparam BTB_SZ = `BTB_SZ;
    localparam BHT_SZ = `BHT_SZ;

    logic                    clock, reset;
    logic [W-1:0][31:0]      lookup_pc, pred_target, upd_pc, upd_target;
    logic [W-1:0]            pred_taken, btb_hit, upd_valid, upd_uncond, upd_taken;
    logic [W-1:0]            pred_call, pred_ret, upd_call, upd_ret;
    logic [W-1:0][`GHR_W-1:0] lookup_hist, upd_hist;

    branch_pred dut (
        .clock      (clock),
        .reset      (reset),
        .lookup_pc  (lookup_pc),
        .lookup_hist(lookup_hist),
        .pred_taken (pred_taken),
        .pred_target(pred_target),
        .btb_hit    (btb_hit),
        .pred_call  (pred_call),
        .pred_ret   (pred_ret),
        .upd_valid  (upd_valid),
        .upd_pc     (upd_pc),
        .upd_uncond (upd_uncond),
        .upd_call   (upd_call),
        .upd_ret    (upd_ret),
        .upd_taken  (upd_taken),
        .upd_target (upd_target),
        .upd_hist   (upd_hist)
    );

    always #5 clock = ~clock;

    // ---- model ----
    bit          m_valid  [BTB_SZ];
    bit          m_uncond [BTB_SZ];
    bit          m_call   [BTB_SZ];
    bit          m_ret    [BTB_SZ];
    logic [31:0] m_pc     [BTB_SZ];   // full PC of the branch that owns the entry
    logic [31:0] m_target [BTB_SZ];
    int          m_cnt    [BHT_SZ];
    int          m_pht    [`PHT_SZ];      // (tournament)
    int          m_ch     [`CHOOSER_SZ];
    int errors = 0, n_hits = 0, n_checks = 0;

    function automatic int bi(input logic [31:0] pc); return (pc >> 2) % BTB_SZ; endfunction
    function automatic int ci(input logic [31:0] pc); return (pc >> 2) % BHT_SZ; endfunction
    function automatic int pi(input logic [31:0] pc, input logic [`GHR_W-1:0] h);
        return (((pc >> 2) % `PHT_SZ) ^ (int'(h) % `PHT_SZ));
    endfunction
    function automatic int hi(input logic [31:0] pc); return (pc >> 2) % `CHOOSER_SZ; endfunction
    function automatic int sat(input int c, input bit up); return up ? ((c == 3) ? 3 : c + 1) : ((c == 0) ? 0 : c - 1); endfunction
    // direction the model predicts for pc looked up with history h
    function automatic bit m_dir(input logic [31:0] pc, input logic [`GHR_W-1:0] h);
        if (`BP_TYPE == 1 && m_ch[hi(pc)] >= 2) return m_pht[pi(pc, h)] >= 2;
        return m_cnt[ci(pc)] >= 2;
    endfunction

    task automatic fail(input string msg);
        $display("  FAIL: %s", msg);
        errors++;
    endtask

    task automatic model_reset();
        foreach (m_valid[e]) m_valid[e] = 0;
        foreach (m_cnt[c]) m_cnt[c] = 1;
        foreach (m_pht[c]) m_pht[c] = 1;
        foreach (m_ch[c]) m_ch[c] = 1;
    endtask

    task automatic idle();
        lookup_pc = '0; upd_valid = '0; upd_pc = '0; upd_uncond = '0; upd_taken = '0; upd_target = '0;
        lookup_hist = '0; upd_hist = '0;
        upd_call = '0; upd_ret = '0;
    endtask

    task automatic upd(input int i, input logic [31:0] pc, input bit uncond, input bit taken,
                       input logic [31:0] target);
        upd_valid[i] = 1; upd_pc[i] = pc; upd_uncond[i] = uncond; upd_taken[i] = taken; upd_target[i] = target;
    endtask

    task automatic step(input string what);
        #1;
        for (int i = 0; i < W; i++) begin
            automatic int  e   = bi(lookup_pc[i]);
            automatic bit  hit = m_valid[e] && m_pc[e] == lookup_pc[i];
            automatic bit  tk  = hit && (m_uncond[e] || m_dir(lookup_pc[i], lookup_hist[i]));
            n_checks++;
            if (btb_hit[i] !== hit)
                fail($sformatf("[%s] slot %0d pc %h: btb_hit=%b expected %b", what, i, lookup_pc[i], btb_hit[i], hit));
            if (pred_taken[i] !== tk)
                fail($sformatf("[%s] slot %0d pc %h: pred_taken=%b expected %b (counter %0d)", what, i,
                               lookup_pc[i], pred_taken[i], tk, m_cnt[ci(lookup_pc[i])]));
            if (pred_call[i] !== (hit && m_call[e]) || pred_ret[i] !== (hit && m_ret[e]))
                fail($sformatf("[%s] slot %0d pc %h: call/ret=%b%b expected %b%b", what, i, lookup_pc[i],
                               pred_call[i], pred_ret[i], hit && m_call[e], hit && m_ret[e]));
            if (tk && pred_target[i] !== m_target[e])
                fail($sformatf("[%s] slot %0d pc %h: target=%h expected %h", what, i, lookup_pc[i], pred_target[i], m_target[e]));
            if (hit) n_hits++;
        end
        @(posedge clock);
        for (int i = 0; i < W; i++)          // program order
            if (upd_valid[i]) begin
                if (!upd_uncond[i]) begin
                    automatic int c = ci(upd_pc[i]), p = pi(upd_pc[i], upd_hist[i]), h = hi(upd_pc[i]);
                    if (`BP_TYPE == 1) begin
                        // the chooser moves toward gshare when the two disagree and gshare was right
                        if ((m_cnt[c] >= 2) != (m_pht[p] >= 2)) m_ch[h] = sat(m_ch[h], (m_pht[p] >= 2) == upd_taken[i]);
                        m_pht[p] = sat(m_pht[p], upd_taken[i]);
                    end
                    m_cnt[c] = sat(m_cnt[c], upd_taken[i]);
                end
                if (upd_taken[i]) begin
                    automatic int e = bi(upd_pc[i]);
                    m_valid[e] = 1; m_uncond[e] = upd_uncond[i]; m_pc[e] = upd_pc[i]; m_target[e] = upd_target[i];
                    m_call[e] = upd_call[i]; m_ret[e] = upd_ret[i];
                end
            end
        @(negedge clock);
    endtask

    // PC pool: A and B share a BTB index (different tags); A and C share a counter only
    localparam logic [31:0] A = 32'h0000_0100;
    localparam logic [31:0] B = A + 4 * BTB_SZ;
    localparam logic [31:0] C = A + 4 * BHT_SZ;
    logic [31:0] pool [] = '{A, B, C, A + 4, A + 8, 32'h0000_2000, 32'h0000_2004, 32'h0001_0040};

    logic [`GHR_W-1:0] hpool [4] = '{'0, `GHR_W'(1), `GHR_W'('h55), '1};
    function automatic logic [31:0] any_pc();
        return pool[$urandom_range(0, pool.size() - 1)];
    endfunction

    initial begin
        clock = 0;
        idle();
        model_reset();
        reset = 1;
        @(negedge clock);
        @(negedge clock);
        reset = 0;

        $display("Test 1: after reset every lookup misses");
        idle(); for (int i = 0; i < W; i++) lookup_pc[i] = any_pc(); step("reset");

        $display("Test 2: counter hysteresis (weakly NT -> taken after one taken, needs two NTs to flip)");
        idle(); upd(0, A, 0, 1, 32'h0000_0400); lookup_pc[0] = A; step("train taken");
        idle(); lookup_pc[0] = A; step("predict taken");
        idle(); upd(0, A, 0, 1, 32'h0000_0400); step("taken again (strong)");
        idle(); upd(0, A, 0, 0, '0); lookup_pc[0] = A; step("one not-taken");
        idle(); lookup_pc[0] = A; step("still taken");
        idle(); upd(0, A, 0, 0, '0); step("second not-taken");
        idle(); lookup_pc[0] = A; step("now not taken");

        $display("Test 3: jumps are always taken once in the BTB");
        idle(); upd(0, 32'h0000_2000, 1, 1, 32'h0000_0800); step("train jal");
        idle(); lookup_pc[0] = 32'h0000_2000; step("predict jal");

        $display("Test 4: same BTB index, different tag replaces the entry");
        idle(); upd(0, B, 0, 1, 32'h0000_0c00); step("train B");
        idle(); lookup_pc[0] = A; if (W > 1) lookup_pc[1] = B; step("A misses, B hits");

        if (W >= 2) begin
            $display("Test 5: two updates to one counter in a cycle chain");
            idle(); upd(0, C, 0, 0, '0); upd(1, C, 0, 0, '0); step("two not-taken");
            idle(); upd(0, A, 0, 1, 32'h0000_0400); upd(1, C, 0, 1, 32'h0000_0500); step("A then C, same counter");
            idle(); lookup_pc[0] = A; lookup_pc[1] = C; step("check chained counter");
        end

        if (`BP_TYPE == 1) begin
            $display("Test 5b: an alternating branch: bimodal cannot learn it, gshare with history can");
            for (int n = 0; n < 40; n++) begin      // T, N, T, N ... with history = last outcome
                idle(); upd(0, A, 0, n % 2 == 0, 32'h0000_0400); upd_hist[0] = `GHR_W'((n + 1) % 2); step("alternate");
            end
            idle(); lookup_pc[0] = A; lookup_hist[0] = `GHR_W'(1); if (W > 1) begin lookup_pc[1] = A; lookup_hist[1] = '0; end
            step("predict both phases");
            if (!pred_taken[0] || (W > 1 && pred_taken[1])) fail("tournament did not learn the alternating branch");
        end

        $display("Test 6: 20000 random cycles");
        for (int c = 0; c < 20000; c++) begin
            idle();
            for (int i = 0; i < W; i++) begin
                lookup_pc[i] = any_pc();
                lookup_hist[i] = hpool[$urandom_range(0, 3)];
                if ($urandom % 2 == 1) begin
                    automatic bit unc = ($urandom % 4 == 0);
                    automatic bit tk  = unc || ($urandom % 3 != 0);
                    automatic int kind = $urandom_range(0, 2);
                    upd(i, any_pc(), unc, tk, $urandom & 32'hffff_fffc);
                    upd_hist[i] = hpool[$urandom_range(0, 3)];
                    upd_call[i] = unc && kind == 1;
                    upd_ret[i]  = unc && kind == 2;
                end
            end
            step("random");
        end

        $display("  (%0d lookups checked, %0d hits)", n_checks, n_hits);
        if (errors == 0) $display("@@@ Passed");
        else             $display("@@@ Failed (%0d errors)", errors);
        $finish;
    end

endmodule
