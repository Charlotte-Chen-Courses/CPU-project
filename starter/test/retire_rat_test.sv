/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  retire_rat_test.sv                                  //
//                                                                     //
//  Description :  Testbench for the retirement RAT. A model map is    //
//                 updated slot by slot in commit order; every cycle   //
//                 the whole DUT map (and map_next before the edge) is //
//                 compared. Covers reset, one                         //
//                 commit, both slots committing the same rd (younger  //
//                 wins), rd = x0 never remapped, and random commits   //
//                 on x0..x7. +define+FP_CLASS: the FP instance (tags  //
//                 offset by FP_TAG_BASE through an adapter; f0 is     //
//                 remapped like any register). Prints @@@ Passed/     //
//                 Failed.                                             //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

module testbench;

    localparam W     = `SUPERSCALAR_WIDTH;
    localparam TAG_W = `TAG_W;
    localparam NARCH = `ARCH_REG_SZ;
`ifdef FP_CLASS
    localparam BASE = `FP_TAG_BASE, NPHYS = `FP_PHYS_REG_SZ;
    localparam bit ZR = 1'b0;
`else
    localparam BASE = 0, NPHYS = `PHYS_REG_SZ;
    localparam bit ZR = 1'b1;
`endif

    logic                          clock, reset;
    logic [W-1:0]                  commit_en;
    logic [W-1:0][4:0]             commit_rd;
    logic [W-1:0][TAG_W-1:0]       commit_tag, d_commit_tag;
    logic [NARCH-1:0][TAG_W-1:0]   map, map_next, d_map, d_map_next;

    // class-local tags here; the DUT sees them offset by BASE
    always_comb begin
        for (int i = 0; i < W; i++) d_commit_tag[i] = commit_tag[i] + TAG_W'(BASE);
        for (int r = 0; r < NARCH; r++) begin map[r] = d_map[r] - TAG_W'(BASE); map_next[r] = d_map_next[r] - TAG_W'(BASE); end
    end

    retire_rat #(.BASE(BASE), .ZERO_REG(ZR)) dut (
        .clock     (clock),
        .reset     (reset),
        .commit_en (commit_en),
        .commit_rd (commit_rd),
        .commit_tag(d_commit_tag),
        .map       (d_map),
        .map_next  (d_map_next)
    );

    always #5 clock = ~clock;

    int model [NARCH];
    int errors = 0;

    task automatic idle();
        commit_en = '0; commit_rd = '0; commit_tag = '0;
    endtask

    task automatic step(input string what);
        #1;
        // map_next: this cycle's commits already applied (used for mispredict recovery)
        begin
            automatic int nxt [NARCH] = model;
            for (int i = 0; i < W; i++)
                if (commit_en[i] && (!ZR || commit_rd[i] != 0)) nxt[commit_rd[i]] = int'(commit_tag[i]);
            for (int x = 0; x < NARCH; x++)
                if (map_next[x] !== TAG_W'(nxt[x])) begin
                    errors++;
                    $display("  FAIL: [%s] map_next x%0d -> p%0d, expected p%0d", what, x, map_next[x], nxt[x]);
                end
        end
        @(posedge clock);
        for (int i = 0; i < W; i++)
            if (commit_en[i] && (!ZR || commit_rd[i] != 0)) model[commit_rd[i]] = int'(commit_tag[i]);
        @(negedge clock);
        for (int r = 0; r < NARCH; r++)
            if (map[r] !== TAG_W'(model[r])) begin
                errors++;
                $display("  FAIL: [%s] x%0d -> p%0d, expected p%0d", what, r, map[r], model[r]);
            end
    endtask

    initial begin
        clock = 0;
        idle();
        for (int r = 0; r < NARCH; r++) model[r] = r;
        reset = 1;
        @(negedge clock);
        @(negedge clock);
        reset = 0;

        $display("Test 1: reset mapping x_i -> p_i");
        idle(); step("reset");

        $display("Test 2: one commit");
        idle(); commit_en[0] = 1; commit_rd[0] = 5; commit_tag[0] = 40;
        step("commit x5");

        if (W >= 2) begin
            $display("Test 3: both slots commit the same rd -> younger wins");
            idle();
            commit_en[0] = 1; commit_rd[0] = 6; commit_tag[0] = 41;
            commit_en[1] = 1; commit_rd[1] = 6; commit_tag[1] = 42;
            step("same rd");
        end

        if (ZR) $display("Test 4: rd = x0 is never remapped");
        else    $display("Test 4: rd = f0 is remapped");
        idle(); commit_en[0] = 1; commit_rd[0] = 0; commit_tag[0] = 43;
        step("x0");

        $display("Test 5: 20000 random cycles");
        for (int c = 0; c < 20000; c++) begin
            idle();
            for (int i = 0; i < W; i++) begin
                commit_en[i]  = ($urandom % 3 != 0);
                commit_rd[i]  = 5'($urandom % 8);
                commit_tag[i] = TAG_W'($urandom % NPHYS);
            end
            step("random");
        end

        if (errors == 0) $display("@@@ Passed");
        else             $display("@@@ Failed (%0d errors)", errors);
        $finish;
    end

endmodule
