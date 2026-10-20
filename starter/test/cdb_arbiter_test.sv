/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  cdb_arbiter_test.sv                                 //
//                                                                     //
//  Description :  Testbench for the CDB arbiter (combinational). The  //
//                 model sorts the requesting ports by (FU-type rank,  //
//                 port number), FP>MULT>LOAD>STORE>BRANCH>ALU,        //
//                 grants the first NUM_CDB, and puts them on lanes in //
//                 that order. Checks fu_grant, every lane's valid and //
//                 packet, unused lanes idle, and at most NUM_CDB      //
//                 grants. Runs every fu_valid pattern (several times  //
//                 with random packets), then random patterns.         //
//                 Prints @@@ Passed/Failed.                           //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

module testbench;

    localparam NFU = `NUM_FU;
    localparam C   = `NUM_CDB;

    logic      [NFU-1:0] fu_valid;
    CDB_PACKET [NFU-1:0] fu_pkt;
    logic      [NFU-1:0] fu_grant;
    logic      [C-1:0]   cdb_valid;
    CDB_PACKET [C-1:0]   cdb_pkt;

    cdb_arbiter dut (
        .fu_valid (fu_valid),
        .fu_pkt   (fu_pkt),
        .fu_grant (fu_grant),
        .cdb_valid(cdb_valid),
        .cdb_pkt  (cdb_pkt)
    );

    int errors = 0, n_checks = 0;

    function automatic FU_TYPE port_type(input int f);
        if (f < `NUM_FU_ALU) return FU_ALU;
        if (f < `NUM_FU_ALU + `NUM_FU_BRANCH) return FU_BRANCH;
        if (f < `NUM_FU_ALU + `NUM_FU_BRANCH + `NUM_FU_MULT) return FU_MULT;
        if (f < `NUM_FU_ALU + `NUM_FU_BRANCH + `NUM_FU_MULT + `NUM_FU_LOAD) return FU_LOAD;
        if (f < `NUM_FU - `NUM_FU_FP) return FU_STORE;
        return FU_FP;
    endfunction

    // smaller rank wins: longest latency first
    function automatic int rank(input FU_TYPE t);
        case (t)
            FU_FP:    return 0;
            FU_MULT:  return 1;
            FU_LOAD:  return 2;
            FU_STORE: return 3;
            FU_BRANCH: return 4;
            default:  return 5;
        endcase
    endfunction

    task automatic fail(input string msg);
        $display("  FAIL: %s", msg);
        errors++;
    endtask

    task automatic check(input string what);
        int order [$];
        logic [NFU-1:0] exp_grant = '0;
        #1;
        // requesting ports sorted by (rank, port)
        for (int r = 0; r <= 5; r++)
            for (int f = 0; f < NFU; f++)
                if (fu_valid[f] && rank(port_type(f)) == r) order.push_back(f);
        for (int k = 0; k < C; k++) begin
            if (k < order.size()) begin
                exp_grant[order[k]] = 1;
                if (cdb_valid[k] !== 1'b1)
                    fail($sformatf("[%s] lane %0d idle, expected port %0d (%s)", what, k,
                                   order[k], port_type(order[k]).name()));
                else if (cdb_pkt[k] !== fu_pkt[order[k]])
                    fail($sformatf("[%s] lane %0d carries the wrong packet, expected port %0d's", what, k, order[k]));
            end else if (cdb_valid[k] !== 1'b0)
                fail($sformatf("[%s] lane %0d valid with only %0d requesters", what, k, order.size()));
        end
        if (fu_grant !== exp_grant)
            fail($sformatf("[%s] fu_valid=%b: fu_grant=%b expected %b", what, fu_valid, fu_grant, exp_grant));
        if ($countones(fu_grant) > C)
            fail($sformatf("[%s] %0d grants for %0d CDBs", what, $countones(fu_grant), C));
        n_checks++;
    endtask

    task automatic random_pkts();
        for (int f = 0; f < NFU; f++) begin
            fu_pkt[f].tag     = `TAG_W'($urandom);
            fu_pkt[f].value   = $urandom;
            fu_pkt[f].rob_idx = `ROB_IDX_W'($urandom);
        end
    endtask

    initial begin
        $display("Ports: %0d (ALU %0d, BRANCH %0d, MULT %0d, LOAD %0d, STORE %0d, FP %0d), CDBs: %0d",
                 NFU, `NUM_FU_ALU, `NUM_FU_BRANCH, `NUM_FU_MULT, `NUM_FU_LOAD, `NUM_FU_STORE, `NUM_FU_FP, C);

        // 1) nobody requests: nothing granted, lanes idle
        $display("Test 1: no requests");
        fu_valid = '0; random_pkts();
        check("idle");

        // 2) every request pattern, several random packet sets each
        $display("Test 2: all %0d fu_valid patterns", 1 << NFU);
        for (int p = 0; p < (1 << NFU); p++)
            for (int n = 0; n < 8; n++) begin
                fu_valid = NFU'(p); random_pkts();
                check($sformatf("pattern %b", NFU'(p)));
            end

        // 3) random
        $display("Test 3: 20000 random patterns");
        for (int n = 0; n < 20000; n++) begin
            fu_valid = NFU'($urandom); random_pkts();
            check("random");
        end

        $display("  (%0d checks)", n_checks);
        if (errors == 0) $display("@@@ Passed");
        else             $display("@@@ Failed (%0d errors)", errors);
        $finish;
    end

endmodule
