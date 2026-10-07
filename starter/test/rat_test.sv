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
//                 register range to force collisions.                 //
//                 Prints @@@ Passed/Failed.                           //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

module testbench;

    localparam W     = `SUPERSCALAR_WIDTH;
    localparam TAG_W = $clog2(`PHYS_REG_SZ);
    localparam NARCH = `ARCH_REG_SZ;

    logic                    clock, reset, rename_en;
    logic [W-1:0]            valid, has_dest;
    logic [W-1:0][4:0]       rs1, rs2, rd;
    logic [W-1:0][TAG_W-1:0] new_tag;
    logic [W-1:0][TAG_W-1:0] src1_tag, src2_tag, told;

    rat dut (
        .clock    (clock),
        .reset    (reset),
        .rename_en(rename_en),
        .valid    (valid),
        .rs1      (rs1),
        .rs2      (rs2),
        .rd       (rd),
        .has_dest (has_dest),
        .new_tag  (new_tag),
        .src1_tag (src1_tag),
        .src2_tag (src2_tag),
        .told     (told)
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
        logic [TAG_W-1:0] t = TAG_W'(NARCH + (next_tag % (`PHYS_REG_SZ - NARCH)));
        next_tag++;
        return t;
    endfunction

    task automatic model_reset();
        for (int r = 0; r < NARCH; r++) model[r] = r;
    endtask

    task automatic idle();
        rename_en = 0; valid = '0; has_dest = '0;
        rs1 = '0; rs2 = '0; rd = '0; new_tag = '0;
    endtask

    // set slot i's instruction
    task automatic slot(input int i, input logic [4:0] s1, input logic [4:0] s2, input logic [4:0] d,
                        input bit hd = 1);
        valid[i] = 1; has_dest[i] = hd;
        rs1[i] = s1; rs2[i] = s2; rd[i] = d;
        new_tag[i] = fresh_tag();
    endtask

    // check outputs against an in-order model, then clock; model commits only if rename_en
    task automatic step(input string what);
        int m [NARCH];
        int e1, e2, et;
        #1;
        m = model;
        for (int i = 0; i < W; i++) begin
            e1 = m[rs1[i]]; e2 = m[rs2[i]]; et = m[rd[i]];
            if (src1_tag[i] !== TAG_W'(e1))
                fail($sformatf("[%s] slot %0d src1 x%0d: got p%0d expected p%0d", what, i, rs1[i], src1_tag[i], e1));
            if (src2_tag[i] !== TAG_W'(e2))
                fail($sformatf("[%s] slot %0d src2 x%0d: got p%0d expected p%0d", what, i, rs2[i], src2_tag[i], e2));
            if (told[i] !== TAG_W'(et))
                fail($sformatf("[%s] slot %0d told x%0d: got p%0d expected p%0d", what, i, rd[i], told[i], et));
            // this slot's rename is visible to younger slots in the bundle
            if (valid[i] && has_dest[i] && rd[i] != 0) m[rd[i]] = int'(new_tag[i]);
        end
        @(posedge clock);
        if (rename_en) model = m;
        @(negedge clock);
    endtask

    // look up every arch reg through the read ports and compare with the model
    task automatic check_all(input string what);
        idle();
        for (int r = 0; r < NARCH; r += W) begin
            for (int i = 0; i < W; i++) begin
                rs1[i] = 5'((r + i) % NARCH); rs2[i] = 5'((r + i + 1) % NARCH); rd[i] = 5'((r + i) % NARCH);
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

            // 6) rd = x0: never renamed, never bypassed, x0 stays p0
            $display("Test 6: rd = x0 is ignored");
            idle(); rename_en = 1;
            slot(0, 1, 2, 0);
            slot(1, 0, 0, 11);
            step("write x0");
            check_all("after x0");
        end else begin
            $display("Tests 3-5: skipped (need SUPERSCALAR_WIDTH >= 2)");
            $display("Test 6: rd = x0 is ignored");
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

        // 8) random bundles on x0..x7 so RAW/WAW collisions happen constantly
        $display("Test 8: 20000 random cycles");
        for (int c = 0; c < 20000; c++) begin
            idle();
            rename_en = ($urandom % 4) != 0;
            for (int i = 0; i < W; i++)
                if ($urandom % 4 != 0)
                    slot(i, 5'($urandom % 8), 5'($urandom % 8), 5'($urandom % 8), ($urandom % 4) != 0);
                else begin
                    rs1[i] = 5'($urandom % 8); rs2[i] = 5'($urandom % 8); rd[i] = 5'($urandom % 8);
                end
            step("random");
        end
        check_all("final");

        if (errors == 0) $display("@@@ Passed");
        else             $display("@@@ Failed (%0d errors)", errors);
        $finish;
    end

endmodule
