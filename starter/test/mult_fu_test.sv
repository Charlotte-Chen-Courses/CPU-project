/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  mult_fu_test.sv                                     //
//                                                                     //
//  Description :  Testbench for the multiply unit. A model queue      //
//                 holds the expected results in issue order (the      //
//                 pipeline never reorders); results are computed with //
//                 64-bit SystemVerilog arithmetic, independent of     //
//                 mult.sv. Every cycle: ready == !(out_valid &&       //
//                 !out_grant), and a valid output must equal the      //
//                 oldest expected result. Directed: each op on edge   //
//                 operands, latency = MULT_STAGES, one result per     //
//                 cycle back to back, a full stall while not granted, //
//                 flush; then random issue / grant / flush.           //
//                 Prints @@@ Passed/Failed.                           //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

module testbench;

    logic             clock, reset;
    logic             in_valid, ready, out_valid, out_grant, flush;
    RS_PACKET         in_pkt;
    CDB_PACKET        out_pkt;
    logic [`XLEN-1:0] rs1_value, rs2_value;

    mult_fu dut (.clock, .reset, .in_valid, .in_pkt, .ready, .out_valid, .out_pkt,
                 .out_grant, .flush, .rs1_value, .rs2_value);

    always #5 clock = ~clock;

    CDB_PACKET q [$];
    int errors = 0, n_results = 0, n_issued = 0;

    function automatic logic [31:0] model(input ALU_FUNC f, input logic [31:0] a, input logic [31:0] b);
        logic signed [63:0] ss = $signed({{32{a[31]}}, a}) * $signed({{32{b[31]}}, b});
        logic signed [63:0] su = $signed({{32{a[31]}}, a}) * $signed({32'b0, b});
        logic        [63:0] uu = {32'b0, a} * {32'b0, b};
        case (f)
            ALU_MUL:    return uu[31:0];
            ALU_MULH:   return ss[63:32];
            ALU_MULHSU: return su[63:32];
            default:    return uu[63:32];   // MULHU
        endcase
    endfunction

    task automatic fail(input string m); errors++; $display("  FAIL: %s", m); endtask

    ALU_FUNC funcs [4] = '{ALU_MUL, ALU_MULH, ALU_MULHSU, ALU_MULHU};
    logic [31:0] edges [8] = '{32'h0, 32'h1, 32'hffff_ffff, 32'h8000_0000, 32'h7fff_ffff, 32'h0001_0000, 32'hdead_beef, 32'h0000_ffff};

    function automatic logic [31:0] operand();
        return ($urandom % 3 == 0) ? edges[$urandom_range(0, 7)] : $urandom;
    endfunction

    task automatic drive(input ALU_FUNC f, input logic [31:0] a, input logic [31:0] b);
        in_valid = 1;
        in_pkt = '0;
        in_pkt.alu_func = f;
        in_pkt.fu_type  = FU_MULT;
        in_pkt.dest_tag = `TAG_W'($urandom);
        in_pkt.rob_idx  = `ROB_IDX_W'($urandom);
        rs1_value = a; rs2_value = b;
    endtask

    task automatic idle(); in_valid = 0; out_grant = 1; flush = 0; endtask

    // check this cycle, clock, update the model
    task automatic step(input string what);
        #1;
        if (ready !== !(out_valid && !out_grant))
            fail($sformatf("[%s] ready=%b with out_valid=%b out_grant=%b", what, ready, out_valid, out_grant));
        if (out_valid && q.size() == 0) fail($sformatf("[%s] out_valid with nothing expected", what));
        if (out_valid && q.size() > 0 && out_pkt !== q[0])
            fail($sformatf("[%s] got tag=%0d value=%h rob=%0d, expected tag=%0d value=%h rob=%0d", what,
                           out_pkt.tag, out_pkt.value, out_pkt.rob_idx, q[0].tag, q[0].value, q[0].rob_idx));
        @(posedge clock);
        if (flush) q.delete();
        else begin
            if (out_valid && out_grant) begin void'(q.pop_front()); n_results++; end
            if (in_valid && ready) begin
                q.push_back('{tag: in_pkt.dest_tag, value: model(in_pkt.alu_func, rs1_value, rs2_value),
                              rob_idx: in_pkt.rob_idx, tid: in_pkt.tid, branch: 0, next_pc: '0, exception: EXC_NONE, fflags: '0});
                n_issued++;
            end
        end
        @(negedge clock);
    endtask

    task automatic drain(input string what);
        for (int c = 0; c < 4 * `MULT_STAGES + 10 && q.size() > 0; c++) begin idle(); step(what); end
        if (q.size() != 0) fail($sformatf("[%s] %0d results never came out", what, q.size()));
    endtask

    initial begin
        int lat;
        clock = 0; reset = 1; in_pkt = '0; rs1_value = 0; rs2_value = 0; idle();
        @(negedge clock); @(negedge clock); reset = 0;

        $display("Test 1: every op on every pair of edge operands");
        foreach (funcs[f]) foreach (edges[i]) foreach (edges[j]) begin
            idle(); drive(funcs[f], edges[i], edges[j]); step("edge");
        end
        drain("edge drain");

        $display("Test 2: latency = MULT_STAGES (%0d)", `MULT_STAGES);
        idle(); drive(ALU_MULH, 32'hffff_fffe, 32'h3); step("latency issue");
        lat = 1;
        while (!out_valid && lat < 50) begin idle(); step("latency wait"); lat++; end
        if (lat != `MULT_STAGES) fail($sformatf("latency %0d, expected %0d", lat, `MULT_STAGES));
        drain("latency drain");

        $display("Test 3: back to back, always granted: one result per cycle");
        begin
            automatic int start_n = n_results;
            for (int c = 0; c < 100; c++) begin idle(); drive(funcs[c % 4], operand(), operand()); step("b2b"); end
            drain("b2b drain");
            if (n_results - start_n != 100) fail($sformatf("b2b: %0d results", n_results - start_n));
        end

        $display("Test 4: result not granted for 10 cycles: the whole pipeline holds");
        for (int c = 0; c < `MULT_STAGES; c++) begin idle(); drive(ALU_MUL, operand(), operand()); step("fill"); end
        for (int c = 0; c < 10; c++) begin idle(); out_grant = 0; drive(ALU_MUL, operand(), operand()); step("stalled"); end
        drain("stall drain");

        $display("Test 5: flush drops every stage, including the op issued that cycle");
        for (int c = 0; c < `MULT_STAGES - 1; c++) begin idle(); drive(ALU_MULHU, operand(), operand()); step("pre-flush"); end
        idle(); drive(ALU_MUL, operand(), operand()); flush = 1; step("flush");
        for (int c = 0; c < `MULT_STAGES + 2; c++) begin
            idle(); step("after flush");
            if (out_valid) fail("result came out after flush");
        end

        $display("Test 6: 20000 random cycles");
        for (int c = 0; c < 20000; c++) begin
            idle();
            if ($urandom % 4 != 0) drive(funcs[$urandom_range(0, 3)], operand(), operand());
            out_grant = ($urandom % 4 != 0);
            flush     = ($urandom % 100 == 0);
            step("random");
        end
        drain("final drain");

        $display("  (%0d results checked)", n_results);
        if (errors == 0) $display("@@@ Passed");
        else             $display("@@@ Failed (%0d errors)", errors);
        $finish;
    end

endmodule
