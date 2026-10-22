/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  rat_test.sv                                         //
//                                                                     //
//  Description :  Testbench for the speculative RAT. The reference    //
//                 model renames the bundle one slot at a time in      //
//                 program order, so same-bundle RAW/WAW behaviour is  //
//                 checked without copying the DUT's bypass logic.     //
//                 Every cycle checks src1_tag/src2_tag/told on every  //
//                 slot. Directed tests cover reset, rename-then-read, //
//                 same-bundle RAW and WAW, no-dest instructions, x0,  //
//                 and rename_en=0; then random traffic on a small     //
//                 register range to force collisions. Also checks the //
//                 third lookup (rs3). With +define+FP_CLASS it tests  //
//                 the FP instance instead (tags offset by FP_TAG_BASE //
//                 through an adapter, and register 0 renames like any //
//                 other: f0 is real).                                 //
//                 Prints @@@ Passed/Failed.                           //
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

    logic                    clock, reset, rename_en;
    logic [W-1:0]            valid, has_dest;
    logic [W-1:0][4:0]       rs1, rs2, rs3, rd;
    logic [W-1:0][TAG_W-1:0] new_tag;
    logic [W-1:0][TAG_W-1:0] src1_tag, src2_tag, src3_tag, told;
    logic [W-1:0]            src1_bypassed, src2_bypassed, src3_bypassed;
    logic                    recover;
    logic [NARCH-1:0][TAG_W-1:0] recover_map;

    // the testbench works with class-local tags; the DUT sees tags offset by BASE
    logic [W-1:0][TAG_W-1:0]     d_new_tag, d_src1, d_src2, d_src3, d_told;
    logic [NARCH-1:0][TAG_W-1:0] d_recover_map;
    always_comb begin
        for (int i = 0; i < W; i++) begin
            d_new_tag[i] = new_tag[i] + TAG_W'(BASE);
            src1_tag[i]  = d_src1[i] - TAG_W'(BASE);
            src2_tag[i]  = d_src2[i] - TAG_W'(BASE);
            src3_tag[i]  = d_src3[i] - TAG_W'(BASE);
            told[i]      = d_told[i] - TAG_W'(BASE);
        end
        for (int r = 0; r < NARCH; r++) d_recover_map[r] = recover_map[r] + TAG_W'(BASE);
    end

    rat #(.BASE(BASE), .ZERO_REG(ZR)) dut (
        .clock    (clock),
        .reset    (reset),
        .rename_en(rename_en),
        .valid    (valid),
        .rs1      (rs1),
        .rs2      (rs2),
        .rs3      (rs3),
        .rd       (rd),
        .has_dest (has_dest),
        .new_tag  (d_new_tag),
        .src1_tag (d_src1),
        .src2_tag (d_src2),
        .src3_tag (d_src3),
        .told     (d_told),
        .src1_bypassed(src1_bypassed),
        .src2_bypassed(src2_bypassed),
        .src3_bypassed(src3_bypassed),
        .recover  (recover),
        .recover_map(d_recover_map)
    );

    always #5 clock = ~clock;

    // reference model: committed-this-cycle mapping of each arch reg
    int model [NARCH];
    int errors = 0;
    int next_tag = 0;

    task automatic fail(input string msg);
        $display("  FAIL: %s", msg);
        errors++;
    endtask

    // distinct new tags (p32..p63, cycling) so a wrong bypass shows up as a wrong tag
    function automatic logic [TAG_W-1:0] fresh_tag();
        logic [TAG_W-1:0] t = TAG_W'(NARCH + (next_tag % (NPHYS - NARCH)));
        next_tag++;
        return t;
    endfunction

    task automatic model_reset();
        for (int r = 0; r < NARCH; r++) model[r] = r;
    endtask

    task automatic idle();
        rename_en = 0; valid = '0; has_dest = '0; recover = 0; recover_map = '0;
        rs1 = '0; rs2 = '0; rs3 = '0; rd = '0; new_tag = '0;
    endtask

    // set slot i's instruction
    task automatic slot(input int i, input logic [4:0] s1, input logic [4:0] s2, input logic [4:0] d,
                        input bit hd = 1);
        valid[i] = 1; has_dest[i] = hd;
        rs1[i] = s1; rs2[i] = s2; rd[i] = d;
        rs3[i] = 5'($urandom % 8);
        new_tag[i] = fresh_tag();
    endtask

    // check outputs against an in-order model, then clock; model commits only if rename_en
    task automatic step(input string what);
        int m [NARCH];
        bit from_bundle [NARCH];    // reg was renamed by an older slot of this bundle
        int e1, e2, e3, et;
        #1;
        m = model;
        foreach (from_bundle[r]) from_bundle[r] = 0;
        for (int i = 0; i < W; i++) begin
            e1 = m[rs1[i]]; e2 = m[rs2[i]]; e3 = m[rs3[i]]; et = m[rd[i]];
            if (src3_bypassed[i] !== from_bundle[rs3[i]])
                fail($sformatf("[%s] slot %0d src3_bypassed=%b expected %b", what, i, src3_bypassed[i], from_bundle[rs3[i]]));
            if (src3_tag[i] !== TAG_W'(e3))
                fail($sformatf("[%s] slot %0d src3 r%0d: got p%0d expected p%0d", what, i, rs3[i], src3_tag[i], e3));
            if (src1_bypassed[i] !== from_bundle[rs1[i]])
                fail($sformatf("[%s] slot %0d src1_bypassed=%b expected %b", what, i, src1_bypassed[i], from_bundle[rs1[i]]));
            if (src2_bypassed[i] !== from_bundle[rs2[i]])
                fail($sformatf("[%s] slot %0d src2_bypassed=%b expected %b", what, i, src2_bypassed[i], from_bundle[rs2[i]]));
            if (src1_tag[i] !== TAG_W'(e1))
                fail($sformatf("[%s] slot %0d src1 x%0d: got p%0d expected p%0d", what, i, rs1[i], src1_tag[i], e1));
            if (src2_tag[i] !== TAG_W'(e2))
                fail($sformatf("[%s] slot %0d src2 x%0d: got p%0d expected p%0d", what, i, rs2[i], src2_tag[i], e2));
            if (told[i] !== TAG_W'(et))
                fail($sformatf("[%s] slot %0d told x%0d: got p%0d expected p%0d", what, i, rd[i], told[i], et));
            // this slot's rename is visible to younger slots in the bundle
            if (valid[i] && has_dest[i] && (!ZR || rd[i] != 0)) begin
                m[rd[i]] = int'(new_tag[i]);
                from_bundle[rd[i]] = 1;
            end
        end
        @(posedge clock);
        if (recover) foreach (model[r]) model[r] = int'(recover_map[r]);   // recover wins
        else if (rename_en) model = m;
        @(negedge clock);
    endtask

    // look up every arch reg through the read ports and compare with the model
    task automatic check_all(input string what);
        idle();
        for (int r = 0; r < NARCH; r += W) begin
            for (int i = 0; i < W; i++) begin
                rs1[i] = 5'((r + i) % NARCH); rs2[i] = 5'((r + i + 1) % NARCH); rd[i] = 5'((r + i) % NARCH);
                rs3[i] = 5'((r + i + 2) % NARCH);
            end
            step(what);
        end
    endtask

    initial begin
        clock = 0;
        idle();
        reset = 1;
        model_reset();
        @(negedge clock);
        @(negedge clock);
        reset = 0;

        // 1) after reset every xi maps to pi
        $display("Test 1: reset mapping x_i -> p_i");
        check_all("reset");

        // 2) rename x5, then the next bundle sees the new tag
        $display("Test 2: rename then read");
        idle(); rename_en = 1; slot(0, 1, 2, 5);
        step("rename x5");
        idle(); rs1[0] = 5; rs2[0] = 5; rd[0] = 5;
        step("read x5");

        if (W >= 2) begin
            // 3) RAW in one bundle: slot 1 reads what slot 0 writes
            $display("Test 3: same-bundle RAW (slot 1 reads slot 0's rd)");
            idle(); rename_en = 1;
            slot(0, 1, 2, 6);
            slot(1, 6, 6, 7);
            step("RAW x6");

            // 4) WAW in one bundle: slot 1's told is slot 0's new tag; slot 1 wins
            $display("Test 4: same-bundle WAW (both slots write x8)");
            idle(); rename_en = 1;
            slot(0, 3, 4, 8);
            slot(1, 8, 3, 8);
            step("WAW x8");
            check_all("after WAW");

            // 5) slot 0 has no dest (store/branch) but its rd field matches slot 1's sources
            $display("Test 5: no-dest instruction does not bypass or write");
            idle(); rename_en = 1;
            slot(0, 1, 2, 9, 0);
            slot(1, 9, 9, 10);
            step("store rd-field x9");
            check_all("after store");

            // 6) rd = x0: never renamed, never bypassed, x0 stays p0 (FP: f0 renames like any register)
            if (ZR) $display("Test 6: rd = x0 is ignored");
            else    $display("Test 6: rd = f0 is renamed (and slot 1 reads it from slot 0)");
            idle(); rename_en = 1;
            slot(0, 1, 2, 0);
            slot(1, 0, 0, 11);
            step("write x0");
            check_all("after x0");
        end else begin
            $display("Tests 3-5: skipped (need SUPERSCALAR_WIDTH >= 2)");
            if (ZR) $display("Test 6: rd = x0 is ignored");
            else    $display("Test 6: rd = f0 is renamed");
            idle(); rename_en = 1; slot(0, 1, 2, 0);
            step("write x0");
            check_all("after x0");
        end

        // 7) rename_en = 0: bypass still shows on outputs, but the table is unchanged
        $display("Test 7: rename_en = 0 leaves the table unchanged");
        idle(); rename_en = 0;
        for (int i = 0; i < W; i++) slot(i, 12, 13, 12);
        step("stalled bundle");
        check_all("after stall");

        // 8) recover: the whole map is replaced, even while a bundle renames
        $display("Test 8: recover loads recover_map (wins over a rename)");
        idle(); rename_en = 1; slot(0, 1, 2, 9);
        recover = 1;
        for (int x = ZR; x < NARCH; x++) recover_map[x] = TAG_W'(NARCH + x % (NPHYS - NARCH));
        step("recover + rename");
        check_all("after recover");

        // 8) random bundles on x0..x7 so RAW/WAW collisions happen constantly
        $display("Test 9: 20000 random cycles");
        for (int c = 0; c < 20000; c++) begin
            idle();
            rename_en = ($urandom % 4) != 0;
            for (int i = 0; i < W; i++)
                if ($urandom % 4 != 0)
                    slot(i, 5'($urandom % 8), 5'($urandom % 8), 5'($urandom % 8), ($urandom % 4) != 0);
                else begin
                    rs1[i] = 5'($urandom % 8); rs2[i] = 5'($urandom % 8); rs3[i] = 5'($urandom % 8); rd[i] = 5'($urandom % 8);
                end
            if ($urandom % 50 == 0) begin
                recover = 1;
                for (int x = ZR; x < NARCH; x++) recover_map[x] = TAG_W'($urandom_range(1, NPHYS - 1));
            end
            step("random");
        end
        check_all("final");

        if (errors == 0) $display("@@@ Passed");
        else             $display("@@@ Failed (%0d errors)", errors);
        $finish;
    end

endmodule
