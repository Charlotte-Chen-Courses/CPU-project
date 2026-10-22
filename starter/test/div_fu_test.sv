/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  div_fu_test.sv                                      //
//                                                                     //
//  Description :  Testbench for the divide unit. Expected results     //
//                 follow the RISC-V spec directly (x/0 = -1 or 2^32-1,//
//                 x%0 = x, INT_MIN/-1 = INT_MIN, INT_MIN%-1 = 0, else //
//                 SystemVerilog / and %, which truncate toward zero), //
//                 independent of the divider's algorithm. Checks the  //
//                 value, tag and rob_idx of every result, that ready  //
//                 is low while an operation is in progress or a       //
//                 result is held, the latency, hold-until-granted,    //
//                 flush, and random issue / grant / flush.            //
//                 Prints @@@ Passed/Failed.                           //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

module testbench;

    localparam LAT = 34;   // cycles from issue to out_valid

    logic             clock, reset;
    logic             in_valid, ready, out_valid, out_grant, flush;
    RS_PACKET         in_pkt;
    CDB_PACKET        out_pkt;
    logic [`XLEN-1:0] rs1_value, rs2_value;

    div_fu dut (.clock, .reset, .in_valid, .in_pkt, .ready, .out_valid, .out_pkt,
                .out_grant, .flush, .rs1_value, .rs2_value);

    always #5 clock = ~clock;

    CDB_PACKET q [$];            // at most one in flight
    bit        holding;          // model: a result is waiting for the CDB
    int        busy_left;        // model: cycles until the in-progress op finishes
    int errors = 0, n_results = 0;

    function automatic logic [31:0] model(input ALU_FUNC f, input logic [31:0] a, input logic [31:0] b);
        logic signed [31:0] sa = a, sb = b;
        case (f)
            ALU_DIV:  return (b == 0) ? 32'hffff_ffff : (a == 32'h8000_0000 && b == 32'hffff_ffff) ? a : 32'(sa / sb);
            ALU_REM:  return (b == 0) ? a             : (a == 32'h8000_0000 && b == 32'hffff_ffff) ? 0 : 32'(sa % sb);
            ALU_DIVU: return (b == 0) ? 32'hffff_ffff : a / b;
            default:  return (b == 0) ? a             : a % b;   // REMU
        endcase
    endfunction

    task automatic fail(input string m); errors++; $display("  FAIL: %s", m); endtask

    ALU_FUNC funcs [4] = '{ALU_DIV, ALU_DIVU, ALU_REM, ALU_REMU};
    logic [31:0] edges [10] = '{32'h0, 32'h1, 32'hffff_ffff, 32'h8000_0000, 32'h7fff_ffff,
                                32'h2, 32'hffff_fffe, 32'h0001_0000, 32'hdead_beef, 32'h7};

    function automatic logic [31:0] operand();
        case ($urandom % 4)
            0:       return edges[$urandom_range(0, 9)];
            1:       return $urandom % 64;
            default: return $urandom;
        endcase
    endfunction

    task automatic drive(input ALU_FUNC f, input logic [31:0] a, input logic [31:0] b);
        in_valid = 1;
        in_pkt = '0;
        in_pkt.alu_func = f;
        in_pkt.fu_type  = FU_DIV;
        in_pkt.dest_tag = `TAG_W'($urandom);
        in_pkt.rob_idx  = `ROB_IDX_W'($urandom);
        rs1_value = a; rs2_value = b;
    endtask

    task automatic idle(); in_valid = 0; out_grant = 1; flush = 0; endtask

    task automatic step(input string what);
        #1;
        if (ready !== (q.size() == 0 || (holding && out_grant)))
            fail($sformatf("[%s] ready=%b (in flight=%0d, holding=%b, grant=%b)", what, ready, q.size(), holding, out_grant));
        if (out_valid !== holding)
            fail($sformatf("[%s] out_valid=%b, expected %b", what, out_valid, holding));
        else if (holding && out_pkt !== q[0])
            fail($sformatf("[%s] got value=%h tag=%0d rob=%0d, expected value=%h tag=%0d rob=%0d", what,
                           out_pkt.value, out_pkt.tag, out_pkt.rob_idx, q[0].value, q[0].tag, q[0].rob_idx));
        @(posedge clock);
        if (flush) begin q.delete(); holding = 0; end
        else begin
            if (holding && out_grant) begin void'(q.pop_front()); holding = 0; n_results++; end
            if (q.size() > 0 && !holding) begin
                busy_left--;
                if (busy_left == 0) holding = 1;
            end
            if (in_valid && ready) begin
                q.push_back('{tag: in_pkt.dest_tag, value: model(in_pkt.alu_func, rs1_value, rs2_value),
                              rob_idx: in_pkt.rob_idx, branch: 0, next_pc: '0, exception: EXC_NONE, fflags: '0});
                busy_left = LAT - 1;   // edges after the issue edge
            end
        end
        @(negedge clock);
    endtask

    task automatic run_one(input ALU_FUNC f, input logic [31:0] a, input logic [31:0] b, input string what);
        idle(); drive(f, a, b); step(what);
        for (int c = 0; c < LAT + 5 && q.size() > 0; c++) begin idle(); step(what); end
        if (q.size() != 0) fail($sformatf("[%s] no result", what));
    endtask

    initial begin
        clock = 0; reset = 1; in_pkt = '0; rs1_value = 0; rs2_value = 0; idle();
        holding = 0; busy_left = 0;
        @(negedge clock); @(negedge clock); reset = 0;

        $display("Test 1: every op on every pair of edge operands (x/0, INT_MIN/-1, ...)");
        foreach (funcs[f]) foreach (edges[i]) foreach (edges[j]) run_one(funcs[f], edges[i], edges[j], "edge");

        $display("Test 2: result held while not granted; ready stays low");
        idle(); drive(ALU_DIV, 32'd100, 32'd7); step("issue");
        for (int c = 0; c < LAT + 6; c++) begin
            idle(); out_grant = 0; drive(ALU_DIVU, 1, 1);   // ready is low: must not be accepted
            step("held");
        end
        idle(); step("granted");
        idle(); step("empty");

        $display("Test 3: flush abandons an op in progress and a held result");
        idle(); drive(ALU_REM, 32'hdead_beef, 32'd13); step("issue");
        for (int c = 0; c < 10; c++) begin idle(); step("busy"); end
        idle(); flush = 1; step("flush busy");
        idle(); drive(ALU_DIV, 32'd9, 32'd3); step("issue after flush");
        for (int c = 0; c < LAT + 3; c++) begin idle(); out_grant = 0; step("held 2"); end
        idle(); out_grant = 0; flush = 1; step("flush held");
        run_one(ALU_REMU, 32'd10, 32'd4, "after flushes");

        $display("Test 4: 3000 random operations with random grant and flush");
        for (int c = 0; c < 3000 * 30; c++) begin
            idle();
            if (ready && $urandom % 2 == 0) drive(funcs[$urandom_range(0, 3)], operand(), operand());
            out_grant = ($urandom % 3 != 0);
            flush     = ($urandom % 2000 == 0);
            step("random");
        end

        $display("  (%0d results checked)", n_results);
        if (errors == 0) $display("@@@ Passed");
        else             $display("@@@ Failed (%0d errors)", errors);
        $finish;
    end

endmodule
