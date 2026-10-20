/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  rs_test.sv                                          //
//                                                                     //
//  Description :  Testbench for the reservation station. Checks only  //
//                 through the ports, so it does not depend on which   //
//                 entry or which ready instruction is picked. A       //
//                 reference model tracks every in-flight instruction  //
//                 (by a unique id in dest_tag/rob_idx) and its ready  //
//                 bits. Every cycle checks: space_ok; each issued     //
//                 packet is in flight, ready, the right FU type for   //
//                 its port, issued once, and unchanged apart from     //
//                 ready bits; and no lost issue: at most ISSUE_WIDTH  //
//                 issue per cycle, FU types served FP > MULT > LOAD > //
//                 STORE > BRANCH > ALU, each type taking min(#ready,  //
//                 #free ports, slots left). Directed                  //
//                 tests cover wakeup timing, same-cycle dispatch+CDB  //
//                 bypass, fu_ready backpressure, and fill/reuse; then //
//                 random traffic and a final drain (every dispatched  //
//                 instruction issues exactly once).                   //
//                 Prints @@@ Passed/Failed.                           //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

module testbench;

    localparam W      = `SUPERSCALAR_WIDTH;
    localparam C      = `NUM_CDB;
    localparam SZ     = `RS_SZ;
    localparam NFU    = `NUM_FU;
    localparam ISSUE_W = `ISSUE_WIDTH;
    localparam TAG_W  = `TAG_W;
    localparam NTAG   = `PHYS_REG_SZ;
    localparam ROB_W  = `ROB_IDX_W;

    logic                    clock, reset;
    logic [W-1:0]            disp_valid;
    RS_PACKET [W-1:0]        disp_pkt;
    logic                    space_ok;
    logic [C-1:0]            cdb_valid;
    logic [C-1:0][TAG_W-1:0] cdb_tag;
    logic                    flush;
    logic [NFU-1:0]          fu_ready;
    logic [NFU-1:0]          issue_valid;
    RS_PACKET [NFU-1:0]      issue_pkt;

    rs dut (
        .clock      (clock),
        .reset      (reset),
        .disp_valid (disp_valid),
        .disp_pkt   (disp_pkt),
        .space_ok   (space_ok),
        .cdb_valid  (cdb_valid),
        .cdb_tag    (cdb_tag),
        .flush      (flush),
        .fu_ready   (fu_ready),
        .issue_valid(issue_valid),
        .issue_pkt  (issue_pkt)
    );

    always #5 clock = ~clock;

    // same port -> FU type mapping as the RS
    function automatic FU_TYPE port_type(input int f);
        if (f < `NUM_FU_ALU) return FU_ALU;
        if (f < `NUM_FU_ALU + `NUM_FU_BRANCH) return FU_BRANCH;
        if (f < `NUM_FU_ALU + `NUM_FU_BRANCH + `NUM_FU_MULT) return FU_MULT;
        if (f < `NUM_FU_ALU + `NUM_FU_BRANCH + `NUM_FU_MULT + `NUM_FU_LOAD) return FU_LOAD;
        if (f < `NUM_FU - `NUM_FU_FP) return FU_STORE;
        return FU_FP;
    endfunction

    // FU types that have at least one port (only these are dispatched randomly)
    FU_TYPE served [$];

    // ---- reference model ----
    // the instruction id lives in {rob_idx, dest_tag} so it can be recognised on issue
    typedef struct {
        int       id;
        RS_PACKET pkt;   // as stored, including current ready bits
        int       ready_at;  // cycle both sources became ready (-1 = not yet)
    } inst_t;

    inst_t m [$];                  // in-flight instructions
    bit    outstanding [NTAG];     // tag will be broadcast later (sources may wait on it)
    int    next_id = 0;
    int    errors = 0, n_disp = 0, n_issued = 0, n_squashed = 0;
    int    cycle = 0, max_wait = 0;  // longest ready-to-issue wait (lowest-index select can starve)
    bit    seen [int];             // ids ever issued (catches double issue across cycles)

    task automatic fail(input string msg);
        $display("  FAIL: %s", msg);
        errors++;
    endtask

    function automatic int id_of(input RS_PACKET p);
        return int'({p.rob_idx, p.dest_tag});
    endfunction

    function automatic bit is_ready(input RS_PACKET p);
        return p.src1_ready && p.src2_ready;
    endfunction

    task automatic idle();
        disp_valid = '0; disp_pkt = '0;
        cdb_valid  = '0; cdb_tag  = '0; flush = 0;
        fu_ready   = '1;
    endtask

    // a tag that is not outstanding and not p0; mark it outstanding
    function automatic logic [TAG_W-1:0] new_outstanding();
        int t;
        do t = $urandom_range(1, NTAG - 1); while (outstanding[t]);
        outstanding[t] = 1;
        return TAG_W'(t);
    endfunction

    function automatic logic [TAG_W-1:0] any_outstanding(output bit found);
        int pool [$];
        for (int t = 1; t < NTAG; t++) if (outstanding[t]) pool.push_back(t);
        found = pool.size() > 0;
        return found ? TAG_W'(pool[$urandom_range(0, pool.size() - 1)]) : '0;
    endfunction

    // build a packet; each source is ready (random tag) or waits on an outstanding tag
    function automatic RS_PACKET make_pkt(input FU_TYPE t, input bit r1, input bit r2);
        RS_PACKET p = '0;
        bit found, busy;
        // ids wrap, and a starved entry can outlive 2^11 dispatches: skip ids still in flight
        do begin
            busy = 0;
            foreach (m[j]) if (m[j].id == next_id) busy = 1;
            for (int i = 0; i < W; i++) if (disp_valid[i] && id_of(disp_pkt[i]) == next_id) busy = 1;
            if (busy) next_id = (next_id + 1) % (1 << (ROB_W + TAG_W));
        end while (busy);
        {p.rob_idx, p.dest_tag} = (ROB_W + TAG_W)'(next_id);
        next_id = (next_id + 1) % (1 << (ROB_W + TAG_W));
        p.fu_type = t;
        p.src1_ready = r1;
        p.src2_ready = r2;
        p.src1_tag = r1 ? TAG_W'($urandom) : any_outstanding(found);
        if (!r1 && !found) begin p.src1_tag = new_outstanding(); end
        p.src2_tag = r2 ? TAG_W'($urandom) : any_outstanding(found);
        if (!r2 && !found) begin p.src2_tag = new_outstanding(); end
        return p;
    endfunction

    task automatic disp(input int i, input RS_PACKET p);
        disp_valid[i] = 1;
        disp_pkt[i]   = p;
    endtask

    task automatic broadcast(input int k, input logic [TAG_W-1:0] t);
        cdb_valid[k] = 1;
        cdb_tag[k]   = t;
    endtask

    // check this cycle's outputs against the model, clock, update the model
    task automatic step(input string what);
        int idx;
        bit took [int];
        int n_ready [FU_TYPE], n_ports [FU_TYPE], n_iss [FU_TYPE];
        #1;
        if (space_ok !== (SZ - m.size() >= W))
            fail($sformatf("[%s] space_ok=%b with %0d entries", what, space_ok, m.size()));
        if (disp_valid != 0 && SZ - m.size() < W)
            fail($sformatf("[%s] testbench dispatched while full", what));

        // every issued packet must be a ready, in-flight instruction of the port's type
        for (int f = 0; f < NFU; f++) begin
            if (issue_valid[f] === 1'b1) begin
                automatic int id = id_of(issue_pkt[f]);
                idx = -1;
                foreach (m[j]) if (m[j].id == id) idx = j;
                if (!fu_ready[f]) fail($sformatf("[%s] port %0d issued while fu_ready=0", what, f));
                if (idx < 0) fail($sformatf("[%s] port %0d issued id %0d which is not in the RS", what, f, id));
                else begin
                    if (!is_ready(m[idx].pkt))
                        fail($sformatf("[%s] port %0d issued id %0d before both sources ready", what, f, id));
                    if (issue_pkt[f] !== m[idx].pkt)
                        fail($sformatf("[%s] port %0d issued id %0d with corrupted contents", what, f, id));
                end
                if (issue_pkt[f].fu_type != port_type(f))
                    fail($sformatf("[%s] port %0d (%s) issued a %s instruction", what, f,
                                   port_type(f).name(), issue_pkt[f].fu_type.name()));
                if (took.exists(id)) fail($sformatf("[%s] id %0d issued on two ports", what, id));
                if (seen.exists(id)) fail($sformatf("[%s] id %0d issued a second time", what, id));
                took[id] = 1;
            end else if (issue_valid[f] !== 1'b0)
                fail($sformatf("[%s] port %0d issue_valid is X", what, f));
        end

        // no lost issue: each type issues min(ready of that type, ready ports of that type)
        foreach (served[s]) begin n_ready[served[s]] = 0; n_ports[served[s]] = 0; n_iss[served[s]] = 0; end
        foreach (m[j]) if (is_ready(m[j].pkt) && n_ready.exists(m[j].pkt.fu_type)) n_ready[m[j].pkt.fu_type]++;
        for (int f = 0; f < NFU; f++) begin
            if (fu_ready[f]) n_ports[port_type(f)]++;
            if (issue_valid[f] === 1'b1) n_iss[port_type(f)]++;
        end
        // with at most ISSUE_W issues per cycle, types are served longest latency first:
        // FP > MULT > LOAD > STORE > ALU; each takes min(ready, free ports, slots left)
        begin
            automatic int left = ISSUE_W;
            automatic FU_TYPE order [] = '{FU_FP, FU_MULT, FU_LOAD, FU_STORE, FU_BRANCH, FU_ALU};
            foreach (order[o]) begin
                automatic FU_TYPE t = order[o];
                automatic int expect_n;
                if (!n_ready.exists(t)) continue;
                expect_n = (n_ready[t] < n_ports[t]) ? n_ready[t] : n_ports[t];
                if (expect_n > left) expect_n = left;
                left -= expect_n;
                if (n_iss[t] != expect_n)
                    fail($sformatf("[%s] %s: issued %0d, expected %0d (%0d ready, %0d ports free, %0d slots left)",
                                   what, t.name(), n_iss[t], expect_n, n_ready[t], n_ports[t], left + expect_n));
            end
        end

        @(posedge clock);
        // 1) issued instructions leave
        for (int j = m.size() - 1; j >= 0; j--)
            if (took.exists(m[j].id)) begin
                seen[m[j].id] = 1;
                if (cycle - m[j].ready_at > max_wait) max_wait = cycle - m[j].ready_at;
                m.delete(j);
                n_issued++;
            end
        if (flush) begin
            n_squashed += m.size();
            for (int i = 0; i < W; i++) if (disp_valid[i]) begin n_squashed++; n_disp++; end
            m.delete();
            for (int k = 0; k < C; k++) if (cdb_valid[k]) outstanding[cdb_tag[k]] = 0;
            cycle++;
            @(negedge clock);
            return;
        end
        // 2) CDB wakes the remaining entries; broadcast tags are no longer outstanding
        for (int k = 0; k < C; k++)
            if (cdb_valid[k]) begin
                foreach (m[j]) begin
                    if (m[j].pkt.src1_tag == cdb_tag[k]) m[j].pkt.src1_ready = 1;
                    if (m[j].pkt.src2_tag == cdb_tag[k]) m[j].pkt.src2_ready = 1;
                    if (m[j].ready_at < 0 && is_ready(m[j].pkt)) m[j].ready_at = cycle + 1;
                end
                outstanding[cdb_tag[k]] = 0;
            end
        // 3) dispatched instructions arrive, with this cycle's CDB already applied
        for (int i = 0; i < W; i++)
            if (disp_valid[i]) begin
                automatic RS_PACKET p = disp_pkt[i];
                for (int k = 0; k < C; k++)
                    if (cdb_valid[k]) begin
                        if (p.src1_tag == cdb_tag[k]) p.src1_ready = 1;
                        if (p.src2_tag == cdb_tag[k]) p.src2_ready = 1;
                    end
                m.push_back('{id: id_of(p), pkt: p, ready_at: is_ready(p) ? cycle + 1 : -1});
                seen.delete(id_of(p));   // ids wrap: a reused id starts fresh
                n_disp++;
            end
        cycle++;
        @(negedge clock);
    endtask

    // broadcast every outstanding tag and let everything issue
    task automatic drain(input string what);
        int guard = 0;
        while (m.size() > 0 && guard < 1000) begin
            bit found;
            idle();
            for (int k = 0; k < C; k++) begin
                automatic logic [TAG_W-1:0] t = any_outstanding(found);
                if (found) begin broadcast(k, t); outstanding[t] = 0; end
            end
            // restore: the model clears outstanding at the edge
            for (int k = 0; k < C; k++) if (cdb_valid[k]) outstanding[cdb_tag[k]] = 1;
            step(what);
            guard++;
        end
        if (m.size() != 0) fail($sformatf("[%s] %0d instructions never issued", what, m.size()));
    endtask

    // id of the most recent packet built
    function automatic int last_id();
        return (next_id + (1 << (ROB_W + TAG_W)) - 1) % (1 << (ROB_W + TAG_W));
    endfunction

    task automatic expect_issued(input int id, input bit yes, input string what);
        bit found = 0;
        for (int f = 0; f < NFU; f++) if (issue_valid[f] && id_of(issue_pkt[f]) == id) found = 1;
        if (found != yes) fail($sformatf("[%s] id %0d issued=%b, expected %b", what, id, found, yes));
    endtask

    // watchdog: a broken RS must fail, never hang the testbench
    initial begin
        #(10 * 200000);
        $display("  FAIL: watchdog timeout");
        $display("@@@ Failed (watchdog, %0d errors so far)", errors);
        $finish;
    end

    initial begin
        RS_PACKET p;
        int id;
        logic [TAG_W-1:0] t;

        for (int f = 0; f < NFU; f++) begin
            automatic bit dup = 0;
            foreach (served[s]) if (served[s] == port_type(f)) dup = 1;
            if (!dup) served.push_back(port_type(f));
        end

        clock = 0;
        idle();
        reset = 1;
        @(negedge clock);
        @(negedge clock);
        reset = 0;

        // 1) empty: nothing issues, space available
        $display("Test 1: empty after reset");
        idle();
        step("reset");

        // 2) ready ALU instruction: not issued the cycle it dispatches, issued the next
        $display("Test 2: ready instruction issues the cycle after dispatch");
        idle(); p = make_pkt(FU_ALU, 1, 1); id = last_id(); disp(0, p);
        #1 expect_issued(id, 0, "dispatch cycle"); #0;
        step("dispatch ready");
        #1 expect_issued(id, 1, "next cycle");
        idle(); step("issue ready");

        // 3) waiting instruction: issues only the cycle after its tag is broadcast
        $display("Test 3: wakeup timing");
        idle(); p = make_pkt(FU_ALU, 1, 0); id = last_id(); t = p.src2_tag; disp(0, p);
        step("dispatch waiting");
        idle(); step("still waiting");
        idle(); broadcast(0, t);
        #1 expect_issued(id, 0, "broadcast cycle");
        step("broadcast");
        idle(); #1 expect_issued(id, 1, "after broadcast");
        step("issue woken");

        // 4) same-cycle dispatch + CDB: must not miss the wakeup
        $display("Test 4: dispatch in the same cycle as its source's broadcast");
        idle(); p = make_pkt(FU_ALU, 0, 1); id = last_id(); t = p.src1_tag;
        disp(0, p); broadcast(0, t);
        step("dispatch + broadcast");
        idle(); #1 expect_issued(id, 1, "after bypass");
        step("issue bypassed");

        // 5) fu_ready = 0 holds the instruction; it issues when the FU frees up
        $display("Test 5: fu_ready backpressure");
        idle(); p = make_pkt(FU_ALU, 1, 1); id = last_id(); disp(0, p);
        step("dispatch");
        for (int c = 0; c < 3; c++) begin idle(); fu_ready = '0; step("fu busy"); end
        idle(); #1 expect_issued(id, 1, "fu free");
        step("fu free");

        // 6) fill to full with waiting instructions, then free slots and reuse them
        $display("Test 6: fill to full (space_ok drops at %0d entries), issue frees, dispatch reuses", SZ - W + 1);
        for (int n = 0; space_ok && n < 4 * SZ; n++) begin
            idle();
            for (int i = 0; i < W; i++) disp(i, make_pkt(served[$urandom_range(0, served.size()-1)], 1, 0));
            step("fill");
        end
        if (m.size() != SZ) fail($sformatf("filled to %0d entries, expected %0d", m.size(), SZ));
        drain("drain full");
        idle();
        for (int i = 0; i < W; i++) disp(i, make_pkt(FU_ALU, 1, 1));
        step("reuse");
        drain("drain reuse");

        // 7) flush: a full RS empties; nothing it held ever issues; slots are reusable
        $display("Test 7: flush squashes every entry");
        for (int n = 0; space_ok && n < 4 * SZ; n++) begin
            idle();
            for (int i = 0; i < W; i++) disp(i, make_pkt(served[$urandom_range(0, served.size()-1)], 1, 0));
            step("fill for flush");
        end
        idle(); flush = 1;
        step("flush");
        idle(); fu_ready = '1;
        for (int c = 0; c < 3; c++) begin idle(); step("after flush: must be empty"); end
        if (!space_ok) fail("RS not empty after flush");
        idle(); for (int i = 0; i < W; i++) disp(i, make_pkt(FU_ALU, 1, 1));
        step("reuse after flush");
        drain("drain after flush");

        // 8) random traffic
        $display("Test 8: 20000 random cycles");
        for (int c = 0; c < 20000; c++) begin
            bit found;
            idle();
            fu_ready = NFU'($urandom) | NFU'($urandom);      // mostly ready
            for (int k = 0; k < C; k++)
                if ($urandom % 3 == 0) begin
                    t = any_outstanding(found);
                    if (found) begin
                        // don't broadcast the same tag on two CDBs
                        automatic bit dup = 0;
                        for (int j = 0; j < k; j++) if (cdb_valid[j] && cdb_tag[j] == t) dup = 1;
                        if (!dup) broadcast(k, t);
                    end
                end
            if (space_ok)
                for (int i = 0; i < W; i++)
                    if ($urandom % 3 != 0)
                        disp(i, make_pkt(served[$urandom_range(0, served.size()-1)],
                                         1'($urandom % 2), 1'($urandom % 2)));
            flush = ($urandom % 100 == 0);
            step("random");
        end

        // 9) drain: everything dispatched issues exactly once (or was squashed)
        $display("Test 9: drain");
        drain("final drain");
        if (n_issued + n_squashed != n_disp)
            fail($sformatf("dispatched %0d, issued %0d, squashed %0d", n_disp, n_issued, n_squashed));

        $display("  (%0d instructions dispatched and issued, FU ports: %0d)", n_disp, NFU);
        $display("  (longest ready-to-issue wait: %0d cycles)", max_wait);
        if (errors == 0) $display("@@@ Passed");
        else             $display("@@@ Failed (%0d errors)", errors);
        $finish;
    end

endmodule
