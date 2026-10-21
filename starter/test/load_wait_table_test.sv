/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  load_wait_table_test.sv                             //
//                                                                     //
//  Description :  Testbench for the load-wait table, with a short     //
//                 clear period (CLEAR = 97) and a reference model.    //
//                 Every cycle: a random lookup (often a trained PC,   //
//                 or an alias of one: same index, other upper bits)   //
//                 is checked, and a random train. Directed: empty     //
//                 after reset, train then lookup next cycle, clear on //
//                 exactly cycle CLEAR, a train in the clear cycle     //
//                 survives, reset clears. Prints @@@ Passed/Failed.   //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

/* verilator lint_off BLKSEQ */

module testbench;

    localparam SZ = 16, CLEAR = 97, P = $clog2(SZ);

    logic             clock, reset, lookup_wait, train_valid;
    logic [`XLEN-1:0] lookup_pc, train_pc;

    load_wait_table #(.SZ(SZ), .CLEAR(CLEAR)) dut (.*);

    always #5 clock = ~clock;

    bit m [SZ];
    int m_age;
    int errors = 0, n_wait = 0;
    logic [`XLEN-1:0] trained [$];

    task automatic model_reset(); foreach (m[i]) m[i] = 0; m_age = 0; endtask

    // check the lookup now, clock, update the model
    task automatic step(input string what);
        #1;
        if (lookup_wait !== m[lookup_pc[2 +: P]]) begin
            errors++;
            $display("  FAIL: [%s] lookup %h: wait=%b, expected %b (age %0d)", what, lookup_pc, lookup_wait, m[lookup_pc[2 +: P]], m_age);
        end
        n_wait += int'(lookup_wait);
        @(posedge clock);
        if (m_age == CLEAR - 1) begin model_reset(); end else m_age++;
        if (train_valid) begin m[train_pc[2 +: P]] = 1; trained.push_back(train_pc); end
        @(negedge clock);
    endtask

    function automatic logic [`XLEN-1:0] pick_pc();
        case ($urandom % 3)
            0:       return {$urandom} & ~32'h3;
            1:       return (trained.size() > 0) ? trained[$urandom_range(0, trained.size() - 1)] : 0;
            default: return (trained.size() > 0) ? trained[$urandom_range(0, trained.size() - 1)] ^ (32'($urandom_range(1, 255)) << (P + 2))
                                                 : 0;   // alias: same index, other upper bits
        endcase
    endfunction

    initial begin
        clock = 0; reset = 1; train_valid = 0; train_pc = 0; lookup_pc = 0;
        @(negedge clock); @(negedge clock); reset = 0;
        model_reset();

        $display("Test 1: empty after reset");
        for (int i = 0; i < SZ; i++) begin lookup_pc = 4 * i; step("empty"); end

        $display("Test 2: train, then the same PC and an alias wait; a neighbour does not");
        train_valid = 1; train_pc = 32'h0000_1234 & ~32'h3; lookup_pc = train_pc; step("train");
        train_valid = 0; step("trained");
        lookup_pc = train_pc + (SZ * 4); step("alias");
        lookup_pc = train_pc + 4;        step("neighbour");

        $display("Test 3: everything clears on cycle %0d after the last clear", CLEAR);
        lookup_pc = train_pc;
        while (m_age != CLEAR - 1) step("before clear");
        step("clear cycle");
        if (lookup_wait !== 1'b0) begin errors++; $display("  FAIL: still waiting after the clear"); end

        $display("Test 4: a train in the clear cycle survives it");
        while (m_age != CLEAR - 1) step("wait for clear");
        train_valid = 1; train_pc = 32'h40; step("train in the clear cycle");
        train_valid = 0; lookup_pc = 32'h40; step("survived");

        $display("Test 5: 20000 random cycles (lookups of trained PCs, aliases and others)");
        for (int c = 0; c < 20000; c++) begin
            train_valid = ($urandom % 10 == 0);
            train_pc    = pick_pc();
            lookup_pc   = pick_pc();
            step("random");
        end
        train_valid = 0;

        $display("Test 6: reset clears");
        train_valid = 1; train_pc = 32'h80; step("train");
        train_valid = 0; reset = 1; @(negedge clock); reset = 0; model_reset();
        lookup_pc = 32'h80; step("after reset");

        $display("  (%0d lookups said wait)", n_wait);
        if (n_wait < 1000) begin errors++; $display("  FAIL: too few waits: the random test is weak"); end
        if (errors == 0) $display("@@@ Passed");
        else             $display("@@@ Failed (%0d errors)", errors);
        $finish;
    end

endmodule
