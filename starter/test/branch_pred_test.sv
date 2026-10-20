/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  branch_pred_test.sv                                 //
//                                                                     //
//  Description :  Testbench for the BTB + bimodal predictor. The      //
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

    branch_pred dut (
        .clock      (clock),
        .reset      (reset),
        .lookup_pc  (lookup_pc),
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
        .upd_target (upd_target)
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
    int errors = 0, n_hits = 0, n_checks = 0;

    function automatic int bi(input logic [31:0] pc); return (pc >> 2) % BTB_SZ; endfunction
    function automatic int ci(input logic [31:0] pc); return (pc >> 2) % BHT_SZ; endfunction

    task automatic fail(input string msg);
        $display("  FAIL: %s", msg);
        errors++;
    endtask

    task automatic model_reset();
        foreach (m_valid[e]) m_valid[e] = 0;
        foreach (m_cnt[c]) m_cnt[c] = 1;
    endtask

    task automatic idle();
        lookup_pc = '0; upd_valid = '0; upd_pc = '0; upd_uncond = '0; upd_taken = '0; upd_target = '0;
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
            automatic bit  tk  = hit && (m_uncond[e] || m_cnt[ci(lookup_pc[i])] >= 2);
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
                    automatic int c = ci(upd_pc[i]);
                    m_cnt[c] = upd_taken[i] ? ((m_cnt[c] == 3) ? 3 : m_cnt[c] + 1)
                                            : ((m_cnt[c] == 0) ? 0 : m_cnt[c] - 1);
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

        $display("Test 6: 20000 random cycles");
        for (int c = 0; c < 20000; c++) begin
            idle();
            for (int i = 0; i < W; i++) begin
                lookup_pc[i] = any_pc();
                if ($urandom % 2 == 1) begin
                    automatic bit unc = ($urandom % 4 == 0);
                    automatic bit tk  = unc || ($urandom % 3 != 0);
                    automatic int kind = $urandom_range(0, 2);
                    upd(i, any_pc(), unc, tk, $urandom & 32'hffff_fffc);
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
