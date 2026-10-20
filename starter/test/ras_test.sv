/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  ras_test.sv                                         //
//                                                                     //
//  Description :  Testbench for the return address stack. The model   //
//                 keeps a speculative and a committed circular stack. //
//                 Every cycle checks top against the speculative      //
//                 model, and (through the hierarchy) the committed    //
//                 stack's top. Covers push/pop, wrap-around past      //
//                 RAS_SZ, commit order within a cycle (call then      //
//                 return), flush restoring the committed stack with   //
//                 this cycle's commits, and random traffic.           //
//                 Prints @@@ Passed/Failed.                           //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

module testbench;

    localparam W  = `SUPERSCALAR_WIDTH;
    localparam SZ = `RAS_SZ;

    logic                   clock, reset;
    logic                   spec_push, spec_pop, flush;
    logic [31:0]            spec_push_addr, top;
    logic [W-1:0]           commit_call, commit_ret;
    logic [W-1:0][31:0]     commit_pc;

    ras dut (
        .clock         (clock),
        .reset         (reset),
        .spec_push     (spec_push),
        .spec_push_addr(spec_push_addr),
        .spec_pop      (spec_pop),
        .top           (top),
        .commit_call   (commit_call),
        .commit_ret    (commit_ret),
        .commit_pc     (commit_pc),
        .flush         (flush)
    );

    always #5 clock = ~clock;

    // model: circular stacks
    logic [31:0] s_buf [SZ], r_buf [SZ];
    int          s_tos, r_tos;
    int errors = 0, n_checks = 0;

    task automatic fail(input string msg);
        $display("  FAIL: %s", msg);
        errors++;
    endtask

    task automatic idle();
        spec_push = 0; spec_pop = 0; spec_push_addr = '0; flush = 0;
        commit_call = '0; commit_ret = '0; commit_pc = '0;
    endtask

    task automatic step(input string what);
        #1;
        n_checks++;
        if (top !== s_buf[s_tos])
            fail($sformatf("[%s] top=%h expected %h (tos %0d)", what, top, s_buf[s_tos], s_tos));
        if (dut.r_stack[dut.r_tos] !== r_buf[r_tos])
            fail($sformatf("[%s] committed top=%h expected %h", what, dut.r_stack[dut.r_tos], r_buf[r_tos]));
        @(posedge clock);
        for (int i = 0; i < W; i++)
            if (commit_call[i]) begin r_tos = (r_tos + 1) % SZ; r_buf[r_tos] = commit_pc[i] + 4; end
            else if (commit_ret[i]) r_tos = (r_tos + SZ - 1) % SZ;
        if (flush) begin s_buf = r_buf; s_tos = r_tos; end
        else if (spec_push) begin s_tos = (s_tos + 1) % SZ; s_buf[s_tos] = spec_push_addr; end
        else if (spec_pop) s_tos = (s_tos + SZ - 1) % SZ;
        @(negedge clock);
    endtask

    task automatic push(input logic [31:0] a); idle(); spec_push = 1; spec_push_addr = a; step("push"); endtask
    task automatic pop();                      idle(); spec_pop = 1; step("pop"); endtask

    initial begin
        clock = 0;
        idle();
        foreach (s_buf[k]) begin s_buf[k] = 0; r_buf[k] = 0; end
        s_tos = 0; r_tos = 0;
        reset = 1;
        @(negedge clock);
        @(negedge clock);
        reset = 0;

        $display("Test 1: push 3, pop 3 (LIFO order)");
        push(32'h100); push(32'h200); push(32'h300);
        pop(); pop(); pop();
        idle(); step("after pops");

        $display("Test 2: push %0d (wraps past RAS_SZ = %0d), then pop them", SZ + 3, SZ);
        for (int k = 0; k < SZ + 3; k++) push(32'h1000 + 4 * k);
        for (int k = 0; k < SZ + 3; k++) pop();

        if (W >= 2) begin
            $display("Test 3: call then return committing in the same cycle");
            idle(); commit_call[0] = 1; commit_pc[0] = 32'h500; commit_ret[1] = 1; step("call + ret");
            idle(); commit_call[0] = 1; commit_pc[0] = 32'h600; commit_call[1] = 1; commit_pc[1] = 32'h700; step("two calls");
        end

        $display("Test 4: wrong-path pushes and pops are undone by a flush (with this cycle's commits)");
        push(32'hdead_0000); push(32'hdead_0004); pop();
        idle(); flush = 1; commit_call[0] = 1; commit_pc[0] = 32'h800; step("flush + commit call");
        idle(); step("after flush");
        pop(); idle(); step("after pop");

        $display("Test 5: 20000 random cycles");
        for (int c = 0; c < 20000; c++) begin
            idle();
            case ($urandom_range(0, 2))
                0: begin spec_push = 1; spec_push_addr = $urandom & 32'hffff_fffc; end
                1: spec_pop = 1;
                default: ;
            endcase
            for (int i = 0; i < W; i++) begin
                automatic int k = $urandom_range(0, 3);
                if (k == 0) begin commit_call[i] = 1; commit_pc[i] = $urandom & 32'hffff_fffc; end
                else if (k == 1) commit_ret[i] = 1;
            end
            flush = ($urandom % 25 == 0);
            step("random");
        end

        $display("  (%0d cycles checked)", n_checks);
        if (errors == 0) $display("@@@ Passed");
        else             $display("@@@ Failed (%0d errors)", errors);
        $finish;
    end

endmodule
