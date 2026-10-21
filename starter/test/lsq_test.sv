/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  lsq_test.sv                                         //
//                                                                     //
//  Description :  Testbench for the LSQ. The testbench plays ROB and  //
//                 RS: it dispatches random loads/stores (all sizes,   //
//                 into a 64-byte region so they overlap constantly,   //
//                 plus some misaligned / out-of-range ones), issues   //
//                 their addresses in random order, grants the CDB     //
//                 outputs randomly, commits in order and flushes at   //
//                 random (one phase delays store addresses so loads   //
//                 run ahead), plus a directed case: a load forwards   //
//                 in the cycle an older store between it and its      //
//                 source gets its address (or after the source has    //
//                 drained). tb_dmem answers with                      //
//                 random latency,                                     //
//                 out of order, and refuses requests at random.       //
//                 It also acts on violations like the ROB: the load   //
//                 named by viol_* is marked, and at the head a marked //
//                 load flushes instead of committing.                 //
//                 Checks:                                             //
//                  - every load that commits has the program-order    //
//                    value: committed memory plus all older stores    //
//                    (computed at dispatch); with LSQ_SPECULATE = 0   //
//                    already at broadcast; tag, rob_idx, exception;   //
//                  - viol_* names an in-flight load with no exception,//
//                    never one given with ld_wait (driven at random); //
//                  - stores complete with the right exception;        //
//                  - every store that reaches memory is the next      //
//                    committed store (order, address, size, data):    //
//                    nothing uncommitted is ever written;             //
//                  - after a final drain, memory = committed memory.  //
//                 Prints @@@ Passed/Failed.                           //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

/* verilator lint_off BLKSEQ */
/* verilator lint_off PROCASSINIT */
/* verilator lint_off UNUSEDSIGNAL */

module testbench;

    localparam W     = `SUPERSCALAR_WIDTH;
    localparam IDW   = `LSQ_IDX_W;
    localparam logic [`XLEN-1:0] BASE = 32'h200;   // 64-byte region
    localparam logic [6:0] OP_LOAD = 7'b0000011, OP_STORE = 7'b0100011;

    logic                           clock, reset, flush;
    logic [W-1:0]                   disp_load, disp_store, commit_load, commit_store;
    logic [W-1:0][IDW-1:0]          disp_lsq_idx;
    logic                           lq_space_ok, sq_space_ok;
    logic                           ld_in_valid, ld_ready, ld_out_valid, ld_out_grant;
    logic                           st_in_valid, st_ready, st_out_valid, st_out_grant;
    RS_PACKET                       ld_in_pkt, st_in_pkt;
    CDB_PACKET                      ld_out_pkt, st_out_pkt;
    logic [`XLEN-1:0]               ld_rs1, st_rs1, st_rs2;
    logic                           dmem_req_valid, dmem_req_ready, dmem_req_store, dmem_resp_valid;
    logic [`XLEN-1:0]               dmem_req_addr, dmem_req_data, dmem_resp_data;
    MEM_SIZE                        dmem_req_size;
    logic [`DMEM_ID_W-1:0]          dmem_req_id, dmem_resp_id;
    logic                           stores_pending, viol_valid, ld_wait;
    logic [`ROB_IDX_W-1:0]          viol_rob_idx;

    lsq dut (.*);

    tb_dmem dmem (
        .clock, .reset,
        .req_valid(dmem_req_valid), .req_ready(dmem_req_ready), .req_store(dmem_req_store),
        .req_addr(dmem_req_addr), .req_size(dmem_req_size), .req_data(dmem_req_data), .req_id(dmem_req_id),
        .resp_valid(dmem_resp_valid), .resp_id(dmem_resp_id), .resp_data(dmem_resp_data)
    );

    always #5 clock = ~clock;

    // ---------------- model ----------------
    typedef struct {
        bit               is_load;
        int               rob;          // rob_idx
        int               idx;          // lsq_idx
        logic [`XLEN-1:0] addr;
        MEM_SIZE          size;
        bit               uns;
        logic [`XLEN-1:0] data;         // store data
        logic [`TAG_W-1:0] tag;
        bit               agen, done;
        logic [`XLEN-1:0] expect_v;     // load: program-order value
        logic [`XLEN-1:0] got_v;        // load: value it broadcast
        bit               replay;       // load: marked by a violation
        bit               wait_st;      // load: given with ld_wait (must never be a violator)
        EXCEPTION         exc;
    } op_t;
    op_t q [$];                         // in flight, program order
    logic [7:0] arch [`MEM_SIZE_IN_BYTES];   // memory after all committed stores
    op_t drainq [$];                    // committed stores not yet seen at memory
    int  next_rob = 0;
    int  errors = 0, n_loads = 0, n_stores = 0, n_flushes = 0, n_exc = 0;
    int  n_replays = 0, n_wrong_replayed = 0, n_ld_commit = 0;

    task automatic fail(input string m); errors++; $display("  FAIL: %s", m); if (errors > 20) $fatal(1, "too many errors"); endtask

    function automatic int nb(input MEM_SIZE s); return (s == BYTE) ? 1 : (s == HALF) ? 2 : 4; endfunction

    function automatic EXCEPTION exc_of(input logic [`XLEN-1:0] a, input MEM_SIZE s);
        if ((s == HALF && a[0]) || (s == WORD && a[1:0] != 0)) return EXC_MISALIGNED;
        if (a >= `MEM_SIZE_IN_BYTES) return EXC_ACCESS;
        return EXC_NONE;
    endfunction

    // value a load at position p sees in program order
    function automatic logic [`XLEN-1:0] po_value(input int p, input logic [`XLEN-1:0] a, input MEM_SIZE s, input bit uns);
        logic [7:0] b [4];
        logic [`XLEN-1:0] raw = '0;
        for (int k = 0; k < nb(s); k++) b[k] = arch[int'(a) + k];
        for (int j = 0; j < p; j++)                           // older in-flight stores, oldest first
            if (!q[j].is_load && q[j].exc == EXC_NONE)
                for (int k = 0; k < nb(s); k++)
                    for (int m = 0; m < nb(q[j].size); m++)
                        if (q[j].addr + m == a + k) b[k] = q[j].data[8 * m +: 8];
        for (int k = 0; k < nb(s); k++) raw[8 * k +: 8] = b[k];
        case (s)
            BYTE:    return uns ? {24'b0, raw[7:0]}  : {{24{raw[7]}},  raw[7:0]};
            HALF:    return uns ? {16'b0, raw[15:0]} : {{16{raw[15]}}, raw[15:0]};
            default: return raw;
        endcase
    endfunction

    function automatic int find_rob(input int rob);
        foreach (q[i]) if (q[i].rob == rob) return i;
        return -1;
    endfunction

    // memory side: every store written must be the next committed one
    always @(posedge clock)
        if (!reset && dmem_req_valid && dmem_req_ready && dmem_req_store) begin
            if (drainq.size() == 0) fail($sformatf("store to %h written to memory before it committed", dmem_req_addr));
            else begin
                automatic op_t s = drainq.pop_front();
                automatic logic [`XLEN-1:0] m = (s.size == BYTE) ? 32'hff : (s.size == HALF) ? 32'hffff : '1;
                if (dmem_req_addr !== s.addr || dmem_req_size !== s.size || (dmem_req_data & m) !== (s.data & m))
                    fail($sformatf("memory write %h/%0d/%h, expected the next committed store %h/%0d/%h",
                                   dmem_req_addr, dmem_req_size, dmem_req_data, s.addr, s.size, s.data));
            end
        end

    // ---------------- one cycle ----------------
    bit allow_dispatch = 1, allow_flush = 1;
    bit stores_late = 0;    // store addresses mostly wait until no load is waiting: loads run ahead
    // directed sequences: ops to dispatch (word accesses at addr), and forced address generation
    typedef struct { bit is_load; logic [`XLEN-1:0] addr; } spec_t;
    spec_t forced [$];
    bit    agen_rand = 1, allow_commit = 1;
    int    force_ld_rob = -1, force_st_rob = -1;

    function automatic void make_op(output op_t o, input bit is_load);
        automatic int r = $urandom_range(0, 99);
        o.is_load = is_load;
        o.size    = MEM_SIZE'($urandom_range(0, 2));
        o.uns     = is_load && o.size != WORD && ($urandom % 2 == 1);
        o.addr    = BASE + $urandom_range(0, 63);
        if (r < 4)       o.addr = o.addr | 1;                                  // maybe misaligned
        else if (r < 7)  o.addr = 32'h0001_0000 + $urandom_range(0, 255);      // out of range
        else             o.addr = (o.size == WORD) ? o.addr & ~3 : (o.size == HALF) ? o.addr & ~1 : o.addr;
        o.data    = $urandom;
        o.tag     = is_load ? `TAG_W'($urandom_range(1, `PHYS_REG_SZ - 1)) : '0;
        o.agen    = 0;
        o.done    = 0;
        o.replay  = 0;
        o.wait_st = 0;
        o.got_v   = '0;
        o.exc     = exc_of(o.addr, o.size);
        o.rob     = next_rob;
        next_rob  = (next_rob + 1) % (1 << `ROB_IDX_W);
    endfunction

    function automatic RS_PACKET pkt_of(input op_t o, output logic [`XLEN-1:0] rs1);
        RS_PACKET p = '0;
        logic [11:0] imm = 12'($urandom_range(0, 255)) - 12'd128;
        logic [2:0]  f3  = {o.uns, 2'(o.size)};
        rs1 = o.addr - {{20{imm[11]}}, imm};
        p.inst     = o.is_load ? {imm, 5'd1, f3, 5'd2, OP_LOAD} : {imm[11:5], 5'd3, 5'd1, f3, imm[4:0], OP_STORE};
        p.lsq_idx  = IDW'(o.idx);
        p.dest_tag = o.tag;
        p.rob_idx  = `ROB_IDX_W'(o.rob);
        p.fu_type  = o.is_load ? FU_LOAD : FU_STORE;
        return p;
    endfunction

    task automatic step();
        op_t newops [$];
        int  ld_pick = -1, st_pick = -1, n_commit = 0;
        bit  do_flush, do_replay = 0;
        // ---- drive (we are at a falling edge) ----
        disp_load = '0; disp_store = '0; commit_load = '0; commit_store = '0;
        ld_in_valid = 0; st_in_valid = 0; flush = 0; ld_wait = 0;
        ld_out_grant = ($urandom % 4 != 0); st_out_grant = ($urandom % 4 != 0);
        // commit: done ops from the head, in order, at most W; an excepting op is flushed instead,
        // and so is a load marked for replay (done or not, like the ROB)
        do_flush = allow_flush && ($urandom % 60 == 0);
        for (int i = 0; i < W && i < q.size(); i++) begin
            if (q[i].replay) begin do_flush = 1; do_replay = 1; break; end
            if (!allow_commit) break;
            if (!q[i].done) break;
            if (q[i].exc != EXC_NONE) begin do_flush = 1; break; end   // (the core would halt here)
            if ($urandom % 4 == 0) break;
            if (q[i].is_load) begin
                commit_load[i] = 1;
                n_ld_commit++;
                if (q[i].exc == EXC_NONE && q[i].got_v !== q[i].expect_v)
                    fail($sformatf("load rob %0d %h/%0d%s commits with %h, expected %h", q[i].rob, q[i].addr,
                                   nb(q[i].size), q[i].uns ? "u" : "", q[i].got_v, q[i].expect_v));
            end else commit_store[i] = 1;
            n_commit++;
        end
        flush = do_flush;
        // dispatch (not in a flush cycle)
        #1;
        if (forced.size() > 0) begin
            if (!do_flush && lq_space_ok && sq_space_ok)
                for (int i = 0; i < W && forced.size() > 0; i++) begin
                    automatic spec_t f = forced.pop_front();
                    op_t o;
                    make_op(o, f.is_load);
                    o.size = WORD; o.uns = 0; o.addr = f.addr; o.exc = EXC_NONE;
                    if (f.is_load) disp_load[i] = 1; else disp_store[i] = 1;
                    newops.push_back(o);
                end
        end else if (allow_dispatch && !do_flush && lq_space_ok && sq_space_ok && q.size() < 24)
            for (int i = 0; i < W; i++) begin
                automatic int r = $urandom_range(0, 2);
                if (r == 0) begin op_t o; make_op(o, 1); disp_load[i] = 1; newops.push_back(o); end
                else if (r == 1) begin op_t o; make_op(o, 0); disp_store[i] = 1; newops.push_back(o); end
            end
        // address generation: one random load and one random store not yet issued
        begin
            automatic int lds [$], sts [$];
            foreach (q[i]) if (!q[i].agen) begin if (q[i].is_load) lds.push_back(i); else sts.push_back(i); end
            if (!agen_rand) begin
                lds.delete(); sts.delete();
                if (force_ld_rob >= 0 && find_rob(force_ld_rob) >= 0 && !q[find_rob(force_ld_rob)].agen) ld_pick = find_rob(force_ld_rob);
                if (force_st_rob >= 0 && find_rob(force_st_rob) >= 0 && !q[find_rob(force_st_rob)].agen) st_pick = find_rob(force_st_rob);
            end
            if (lds.size() > 0 && $urandom % 3 != 0) ld_pick = lds[$urandom_range(0, lds.size() - 1)];
            if (sts.size() > 0 && $urandom % 3 != 0 && (!stores_late || lds.size() == 0 || $urandom % 8 == 0)) st_pick = sts[$urandom_range(0, sts.size() - 1)];
        end
        #1;
        if (ld_pick >= 0) begin
            ld_in_valid = 1; ld_in_pkt = pkt_of(q[ld_pick], ld_rs1);
            ld_wait = agen_rand && ($urandom % 4 == 0);         // as if predicted to conflict
            q[ld_pick].wait_st = ld_wait;
        end
        if (st_pick >= 0 && st_ready) begin st_in_valid = 1; st_in_pkt = pkt_of(q[st_pick], st_rs1); st_rs2 = q[st_pick].data; end
        #1;
        // indices for the new ops (combinational from this cycle's dispatch)
        begin
            automatic int n = 0;
            for (int i = 0; i < W; i++)
                if (disp_load[i] || disp_store[i]) begin newops[n].idx = int'(disp_lsq_idx[i]); n++; end
        end
        // ---- check outputs ----
        if (ld_out_valid && ld_out_grant) begin
            automatic int p = find_rob(int'(ld_out_pkt.rob_idx));
            if (p < 0 || !q[p].is_load) fail($sformatf("load result for rob %0d, which is not an in-flight load", ld_out_pkt.rob_idx));
            else begin
                if (ld_out_pkt.exception !== q[p].exc)
                    fail($sformatf("load rob %0d: exception %s, expected %s", q[p].rob, ld_out_pkt.exception.name(), q[p].exc.name()));
                else if (q[p].exc == EXC_NONE && ((!`LSQ_SPECULATE && ld_out_pkt.value !== q[p].expect_v) || ld_out_pkt.tag !== q[p].tag))
                    fail($sformatf("load rob %0d %h/%0d%s: got %h (tag %0d), expected %h (tag %0d)", q[p].rob, q[p].addr,
                                   nb(q[p].size), q[p].uns ? "u" : "", ld_out_pkt.value, ld_out_pkt.tag, q[p].expect_v, q[p].tag));
                if (q[p].done) fail($sformatf("load rob %0d broadcast twice", q[p].rob));
                if (!q[p].agen) fail($sformatf("load rob %0d done before its address was given", q[p].rob));
                q[p].done = 1; q[p].got_v = ld_out_pkt.value; n_loads++;
            end
        end
        if (st_out_valid && st_out_grant) begin
            automatic int p = find_rob(int'(st_out_pkt.rob_idx));
            if (p < 0 || q[p].is_load) fail($sformatf("store completion for rob %0d, which is not an in-flight store", st_out_pkt.rob_idx));
            else begin
                if (st_out_pkt.exception !== q[p].exc || st_out_pkt.tag !== '0)
                    fail($sformatf("store rob %0d: exception %s tag %0d, expected %s p0", q[p].rob,
                                   st_out_pkt.exception.name(), st_out_pkt.tag, q[p].exc.name()));
                q[p].done = 1; n_stores++;
            end
        end
        if (viol_valid) begin
            automatic int p = find_rob(int'(viol_rob_idx));
            if (!`LSQ_SPECULATE) fail("violation reported with LSQ_SPECULATE = 0");
            if (p < 0 || !q[p].is_load || q[p].exc != EXC_NONE)
                fail($sformatf("violation names rob %0d, which is not an in-flight load without an exception", viol_rob_idx));
            else if (p < n_commit) fail($sformatf("violation names rob %0d, which commits this cycle", viol_rob_idx));
            else if (q[p].wait_st) fail($sformatf("violation names rob %0d, which was told to wait (ld_wait)", viol_rob_idx));
            else if (!do_flush) q[p].replay = 1;
        end
        // ---- clock ----
        @(posedge clock);
        // ---- update the model in the same order as the hardware ----
        if (ld_pick >= 0) q[ld_pick].agen = 1;
        if (st_in_valid) q[st_pick].agen = 1;
        for (int i = 0; i < n_commit; i++) begin
            automatic op_t o = q.pop_front();
            if (!o.is_load) begin
                for (int k = 0; k < nb(o.size); k++) arch[int'(o.addr) + k] = o.data[8 * k +: 8];
                drainq.push_back(o);
            end
        end
        if (do_flush) begin
            if (do_replay) begin
                n_replays++;
                if (q[0].done && q[0].got_v !== q[0].expect_v) n_wrong_replayed++;
            end
            foreach (q[i]) if (q[i].exc != EXC_NONE) n_exc++;
            q.delete();
            n_flushes++;
        end else
            foreach (newops[i]) begin
                newops[i].expect_v = (newops[i].is_load && newops[i].exc == EXC_NONE)
                                     ? po_value(q.size(), newops[i].addr, newops[i].size, newops[i].uns) : '0;
                q.push_back(newops[i]);
            end
        @(negedge clock);
    endtask

    initial begin
        int guard;
        clock = 0; reset = 1; flush = 0;
        disp_load = '0; disp_store = '0; commit_load = '0; commit_store = '0;
        ld_in_valid = 0; st_in_valid = 0; ld_out_grant = 0; st_out_grant = 0;
        ld_in_pkt = '0; st_in_pkt = '0; ld_wait = 0; ld_rs1 = 0; st_rs1 = 0; st_rs2 = 0;
        foreach (arch[a]) begin arch[a] = 8'($urandom); dmem.mem[a] = arch[a]; end
        @(negedge clock); @(negedge clock); reset = 0;

        $display("Test 1: 30000 random cycles (dispatch, any-order address generation, commits, flushes)");
        for (int c = 0; c < 30000; c++) step();

        $display("Test 2: memory always ready, short latency");
        dmem.ready_pct = 100; dmem.max_lat = 1;
        for (int c = 0; c < 5000; c++) step();

        $display("Test 3: memory slow and often busy");
        dmem.ready_pct = 30; dmem.max_lat = 40;
        for (int c = 0; c < 10000; c++) step();
        dmem.ready_pct = 75; dmem.max_lat = 12;

        $display("Test 4: store addresses late (loads run ahead of older stores: many violations)");
        stores_late = 1;
        for (int c = 0; c < 10000; c++) step();
        stores_late = 0;

        $display("Test 5: a load forwards from store H in the cycle an overlapping store S between them gets its address");
        // H (covers the load), S (same word, address late), 0..SQ_SZ-3 unrelated stores, load L.
        // In the cycle L forwards from H, S gets its address: L read the wrong value and must be
        // replayed (the check must use the store L is forwarding from this cycle, not a stale one).
        if (`SQ_SZ < 2 + W) $display("  (skipped: the store queue cannot hold H, S and a dispatch bundle)");
        else begin
            automatic int n_same = 0, n_drained = 0;
            for (int rep = 0; rep < 400; rep++) begin
                automatic int nx = $urandom_range(0, `SQ_SZ - 2 - W);   // leave room for L's bundle
                automatic int h_rob, s_rob, l_rob, g;
                allow_dispatch = 0; allow_flush = 0;
                g = 0; while (q.size() > 0 && g++ < 2000) step();
                allow_commit = 0; agen_rand = 0; force_ld_rob = -1; force_st_rob = -1;
                h_rob = next_rob; s_rob = (next_rob + 1) % (1 << `ROB_IDX_W); l_rob = (next_rob + 2 + nx) % (1 << `ROB_IDX_W);
                forced.push_back('{0, BASE + 8});
                forced.push_back('{0, BASE + 8});
                for (int k = 0; k < nx; k++) forced.push_back('{0, BASE + 32 + 4 * k});
                forced.push_back('{1, BASE + 8});
                g = 0; while (forced.size() > 0 && g++ < 200) step();
                force_st_rob = h_rob;
                g = 0; while (!q[find_rob(h_rob)].agen && g++ < 200) step();
                if (find_rob(l_rob) < 0) begin fail("directed: the load was never dispatched"); break; end
                force_st_rob = -1; force_ld_rob = l_rob;
                step();                                             // L's address
                force_ld_rob = -1;
                if (rep % 2 == 1) begin
                    // variant: L forwards, then H commits and drains out of the SQ before S gets
                    // its address (the drained source must not count as younger than S)
                    g = 0; while (!q[find_rob(l_rob)].done && g++ < 200) step();
                    allow_commit = 1;
                    g = 0; while ((find_rob(h_rob) >= 0 || stores_pending) && g++ < 400) step();
                    allow_commit = 0;
                    if (find_rob(h_rob) < 0 && !stores_pending && q[find_rob(l_rob)].done) n_drained++;
                end
                force_st_rob = s_rob;
                #1;
                if (rep % 2 == 0 && dut.can_fwd[q[find_rob(l_rob)].idx] && st_ready) n_same++;
                g = 0; while (find_rob(s_rob) >= 0 && !q[find_rob(s_rob)].agen && g++ < 200) step();
                force_st_rob = -1; agen_rand = 1; allow_commit = 1; allow_dispatch = 1; allow_flush = 1;
            end
            $display("  (%0d of 200 had the forward and the store address in the same cycle; in %0d of 200 the source drained first)",
                     n_same, n_drained);
            if (`LSQ_SPECULATE && (n_same < 50 || n_drained < 50)) fail("directed cases too rare");
        end

        $display("Test 6: drain (no new work, everything commits, committed stores reach memory)");
        allow_dispatch = 0; allow_flush = 0;
        guard = 0;
        while (q.size() > 0 || stores_pending || drainq.size() > 0) begin
            step();
            if (++guard > 5000) begin fail($sformatf("drain stuck: %0d in flight, %0d stores waiting", q.size(), drainq.size())); break; end
        end
        repeat (5) @(negedge clock);
        for (int a = 0; a < `MEM_SIZE_IN_BYTES; a++)
            if (dmem.mem[a] !== arch[a]) fail($sformatf("final memory byte %h = %h, expected %h", a, dmem.mem[a], arch[a]));

        $display("  (%0d loads and %0d stores completed, %0d flushes, %0d memory loads, %0d memory stores)",
                 n_loads, n_stores, n_flushes, dmem.n_loads, dmem.n_stores);
        $display("  (%0d loads committed; %0d replays, %0d of them of a wrong value)", n_ld_commit, n_replays, n_wrong_replayed);
        if (`LSQ_SPECULATE && n_wrong_replayed == 0) fail("speculation never produced a wrong value: the replay path is untested");
        if (errors == 0) $display("@@@ Passed");
        else             $display("@@@ Failed (%0d errors)", errors);
        $finish;
    end

endmodule
