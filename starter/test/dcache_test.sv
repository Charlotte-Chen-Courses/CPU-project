/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  dcache_test.sv                                      //
//                                                                     //
//  Description :  Testbench for the data cache, directly on P3's      //
//                 mem.sv (CACHE_MODE, MEM_LATENCY_IN_CYCLES). A       //
//                 reference byte array is updated when each request   //
//                 is accepted; every load response (matched by id:    //
//                 the cache is non-blocking) must match it. At the end//
//                 cache's valid dirty blocks are overlaid on mem.sv's //
//                 memory and every touched byte is compared, which    //
//                 catches lost or wrong writebacks. Directed: hit     //
//                 after fill, byte/half/word stores into a cached     //
//                 block, dirty eviction then re-read, LRU order; then //
//                 random traffic over a hot region, addresses that    //
//                 alias to the same sets, and far addresses, while    //
//                 memory randomly refuses requests; with MSHRs, loads //
//                 and stores interleaved into a block being fetched,  //
//                 hits under a miss, and a dirty victim refetched at  //
//                 once.                                               //
//                 Prints @@@ Passed/Failed.                           //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

// testbench-only: reference model and counters update with blocking assignments on purpose
/* verilator lint_off BLKSEQ */
/* verilator lint_off PROCASSINIT */

module testbench;

    localparam SETS  = `DC_SZ / (`DC_BLK * `DC_WAYS);
    localparam WAY_B = `DC_SZ / `DC_WAYS;     // bytes per way: addresses this far apart share a set

    logic                   clock, reset;
    logic                   req_valid, req_ready, req_store, resp_valid;
    logic [`XLEN-1:0]       req_addr, req_data, resp_data;
    MEM_SIZE                req_size;
    logic [`DMEM_ID_W-1:0]  req_id, resp_id;
    logic [1:0]             mem_command;
    logic [`XLEN-1:0]       mem_addr;
    logic [63:0]            mem_data, mem_rdata;
    logic [3:0]             mem_response, mem_tag;

    dcache dut (
        .clock, .reset, .req_valid, .req_ready, .req_store, .req_addr, .req_size, .req_data, .req_id,
        .resp_valid, .resp_id, .resp_data,
        .mem_command, .mem_addr, .mem_data, .mem_response, .mem_rdata, .mem_tag
    );

    // memory sometimes refuses a request (as when all 15 memory tags are busy or the arbiter gives
    // the bus to the other cache): the cache must keep asking
    int   refuse_pct = 0;
    logic refuse;
    always @(posedge clock) refuse <= ($urandom % 100) < refuse_pct;

    mem memory (
        .clk              (clock),
        .proc2mem_addr    (mem_addr),
        .proc2mem_data    (mem_data),
        .proc2mem_command (refuse ? BUS_NONE : mem_command),
        .mem2proc_response(mem_response),
        .mem2proc_data    (mem_rdata),
        .mem2proc_tag     (mem_tag)
    );

    always #(`CLOCK_PERIOD / 2.0) clock = ~clock;

    // ---- reference ----
    logic [7:0] ref_mem [`MEM_SIZE_IN_BYTES];
    bit         touched [int];
    typedef struct { logic [`DMEM_ID_W-1:0] id; logic [`XLEN-1:0] word; } exp_t;
    exp_t exp_q [int];                 // by id: responses may come back out of order
    int   resp_cycle [int];            // cycle each id's response arrived (ordering checks)
    int errors = 0, n_loads = 0, n_resps = 0, cycle = 0;
    logic [`DMEM_ID_W-1:0] next_id = 0;

    task automatic fail(input string m); errors++; $display("  FAIL: %s", m); if (errors > 20) $fatal(1, "too many errors"); endtask

    function automatic logic [`XLEN-1:0] ref_word(input logic [`XLEN-1:0] a);
        automatic int b = int'(a) & ~3;
        return {ref_mem[b + 3], ref_mem[b + 2], ref_mem[b + 1], ref_mem[b]};
    endfunction

    // model: every accepted request is applied in order
    always @(posedge clock)
        if (!reset && req_valid && req_ready) begin
            automatic int n = (req_size == BYTE) ? 1 : (req_size == HALF) ? 2 : 4;
            for (int k = 0; k < n; k++) touched[int'(req_addr) + k] = 1;
            if (req_store) for (int k = 0; k < n; k++) ref_mem[int'(req_addr) + k] = req_data[8 * k +: 8];
            else begin
                if (exp_q.exists(int'(req_id))) fail($sformatf("test bug: load id %0d reused while waiting", req_id));
                exp_q[int'(req_id)] = '{id: req_id, word: ref_word(req_addr)}; n_loads++;
            end
        end

    // responses: each must match the load with its id
    always @(negedge clock) begin
        cycle++;
        if (!reset && resp_valid) begin
            n_resps++;
            resp_cycle[int'(resp_id)] = cycle;
            if (!exp_q.exists(int'(resp_id))) fail($sformatf("response for id %0d, which no load is waiting for", resp_id));
            else begin
                automatic exp_t e = exp_q[int'(resp_id)];
                exp_q.delete(int'(resp_id));
                if (resp_data !== e.word)
                    fail($sformatf("load id %0d: got %h, expected %h", e.id, resp_data, e.word));
            end
        end
    end

    // one request (called at a falling edge); returns at the falling edge after it is accepted
    task automatic access(input bit st, input logic [`XLEN-1:0] a, input MEM_SIZE s, input logic [`XLEN-1:0] d = $urandom);
        int guard = 0;
        req_valid = 1; req_store = st; req_addr = a; req_size = s; req_data = d; req_id = next_id;
        // ready depends on the address (full set / full MSHR): sample it once the new request has settled
        #(`CLOCK_PERIOD / 10.0);
        while (!req_ready) begin
            @(negedge clock); #(`CLOCK_PERIOD / 10.0);
            if (++guard > 2000) $fatal(1, "request to %h never accepted", a);
        end
        @(negedge clock);          // the rising edge in between accepted it
        next_id++;
        req_valid = 0;
    endtask

    task automatic wait_idle();
        int guard = 0;
        req_valid = 0;
        while (!(dut.idle && exp_q.size() == 0)) begin @(negedge clock); if (++guard > 2000) $fatal(1, "cache never went idle"); end
    endtask

    function automatic MEM_SIZE rsize(); return MEM_SIZE'($urandom_range(0, 2)); endfunction
    function automatic logic [`XLEN-1:0] align(input logic [`XLEN-1:0] a, input MEM_SIZE s);
        return (s == WORD) ? a & ~32'h3 : (s == HALF) ? a & ~32'h1 : a;
    endfunction

    initial begin
        int h0, m0;
        clock = 0; reset = 1; req_valid = 0; req_store = 0; req_addr = 0; req_size = WORD; req_data = 0; req_id = 0;
        // same random contents in mem.sv and the reference
        for (int i = 0; i < `MEM_64BIT_LINES; i++) begin
            automatic logic [63:0] v = {$urandom, $urandom};
            memory.unified_memory[i] = v;
            for (int k = 0; k < 8; k++) ref_mem[8 * i + k] = v[8 * k +: 8];
        end
        $display("Config: %0d B, %0d-way, %0d sets, 8 B blocks, memory latency %0d cycles",
                 `DC_SZ, `DC_WAYS, SETS, int'(`MEM_LATENCY_IN_CYCLES));
        repeat (2) @(negedge clock); reset = 0; @(negedge clock);

        $display("Test 1: load miss, then hits in the same block");
        access(0, 32'h100, WORD); access(0, 32'h104, WORD); access(0, 32'h102, HALF); access(0, 32'h107, BYTE);
        wait_idle();
        if (`DC_MSHRS == 0 && (dut.n_misses != 1 || dut.n_hits != 3))
            fail($sformatf("expected 1 miss + 3 hits, got %0d + %0d", dut.n_misses, dut.n_hits));
        if (`DC_MSHRS > 0 && `DC_TGTS >= 4 && (dut.n_misses != 1 || dut.n_merged != 3 || dut.n_hits != 0))   // all join the miss
            fail($sformatf("expected 1 miss + 3 merged, got %0d + %0d (+ %0d hits)", dut.n_misses, dut.n_merged, dut.n_hits));

        $display("Test 2: byte / half / word stores into a cached block, read back");
        access(1, 32'h101, BYTE, 32'hAA); access(1, 32'h106, HALF, 32'hBEEF); access(1, 32'h100, WORD, 32'h1234_5678);
        access(0, 32'h100, WORD); access(0, 32'h104, WORD); access(0, 32'h101, BYTE);
        wait_idle();

        $display("Test 3: dirty block evicted by %0d other blocks of its set, then read again", `DC_WAYS);
        access(1, 32'h204, WORD, 32'hCAFE_F00D);                                    // set of 0x200, dirty
        for (int k = 1; k <= `DC_WAYS; k++) access(0, 32'h200 + k * WAY_B, WORD);    // evicts it
        wait_idle();
        if (dut.n_writebacks == 0) fail("no writeback happened");
        access(0, 32'h204, WORD);                                                   // must come from memory
        wait_idle();

        if (`DC_WAYS == 2) begin
            $display("Test 4: LRU: A, B, A, C (same set) evicts B, so A still hits");
            // (one at a time: with MSHRs, overlapping misses would install in a different order)
            access(0, 32'h300, WORD); wait_idle(); access(0, 32'h300 + WAY_B, WORD); wait_idle();
            access(0, 32'h300, WORD); wait_idle(); access(0, 32'h300 + 2 * WAY_B, WORD);
            wait_idle();
            h0 = dut.n_hits; m0 = dut.n_misses;
            access(0, 32'h300, WORD); wait_idle();
            if (dut.n_hits != h0 + 1) fail("A was evicted: LRU picked the most recently used way");
            access(0, 32'h300 + WAY_B, WORD); wait_idle();
            if (dut.n_misses != m0 + 1) fail("B was not evicted");
        end

        if (`DC_MSHRS > 0) begin
            automatic int id_miss, id_hit, m0;
            $display("Test 5: loads and stores interleaved into a block being fetched; a hit under the miss");
            access(0, 32'h600, WORD);                                   // a cached block, for the hit below
            wait_idle();
            m0 = dut.n_misses;
            id_miss = int'(next_id);
            access(0, 32'h700, WORD);                                   // primary miss
            access(1, 32'h700, WORD, 32'h1111_1111);                    // store after the load: must not leak into it
            access(0, 32'h700, WORD);                                   // sees 1111_1111
            access(1, 32'h702, BYTE, 32'h22);                           // (merged targets are capped at DC_TGTS:
            id_hit = int'(next_id);                                     //  later ones wait, still in order)
            access(0, 32'h600, WORD);                                   // hit under the miss
            access(0, 32'h700, WORD);                                   // sees 1122_1111
            access(0, 32'h704, WORD);
            wait_idle();
            if (dut.n_misses != m0 + 1) fail($sformatf("%0d misses for one block", dut.n_misses - m0));
            if (`DC_TGTS >= 4 && !(resp_cycle[id_hit] < resp_cycle[id_miss])) fail("the hit waited for the miss");

            $display("Test 6: a dirty block evicted, then read again at once (writeback before refetch)");
            for (int r = 0; r < 20; r++) begin
                automatic logic [`XLEN-1:0] b = 32'h800 + 8 * r;
                access(1, b, WORD, $urandom); wait_idle();              // b cached and dirty
                for (int k = 1; k <= `DC_WAYS; k++) access(0, b + k * WAY_B, WORD);   // evict it ...
                access(0, b, WORD);                                     // ... and want it back right away
                access(1, b + 4, WORD, $urandom);
                access(0, b + 4, WORD);
            end
            wait_idle();
        end

        $display("Test 7: 20000 random requests (hot region, same-set aliases, far addresses), memory refuses 30%%");
        refuse_pct = 30;
        for (int n = 0; n < 20000; n++) begin
            automatic MEM_SIZE s = rsize();
            automatic int kind = $urandom_range(0, 9);
            automatic logic [`XLEN-1:0] a;
            if (kind < 6)      a = 32'h400 + $urandom_range(0, 127);                              // hot: one way's worth
            else if (kind < 9) a = 32'h400 + $urandom_range(0, 127) + WAY_B * $urandom_range(1, 4); // aliases
            else               a = $urandom_range(0, `MEM_SIZE_IN_BYTES - 1);                    // anywhere
            access($urandom % 3 == 0, align(a, s), s);
            if ($urandom % 8 == 0) begin req_valid = 0; repeat ($urandom_range(1, 3)) @(negedge clock); end
        end
        refuse_pct = 0;
        wait_idle();                     // (mem.sv writes memory when it accepts a writeback)

        // final memory: mem.sv overlaid with the cache's dirty blocks
        begin
            automatic logic [7:0] actual [int];
            foreach (touched[a]) actual[a] = memory.unified_memory[a / 8][8 * (a % 8) +: 8];
            for (int s = 0; s < SETS; s++)
                for (int w = 0; w < `DC_WAYS; w++)
                    if (dut.lines[s][w].valid && dut.lines[s][w].dirty) begin
                        automatic int base = (int'(dut.lines[s][w].tag) * SETS + s) * 8;
                        for (int k = 0; k < 8; k++) if (actual.exists(base + k)) actual[base + k] = dut.lines[s][w].data[8 * k +: 8];
                    end
            foreach (touched[a])
                if (actual[a] !== ref_mem[a]) fail($sformatf("final memory byte %h = %h, expected %h", a, actual[a], ref_mem[a]));
        end
        if (n_resps != n_loads) fail($sformatf("%0d loads, %0d responses", n_loads, n_resps));

        $display("  (%0d hits, %0d misses, %0d merged into a miss: hit rate %.1f%%, %0d writebacks)", dut.n_hits,
                 dut.n_misses, dut.n_merged, 100.0 * dut.n_hits / (dut.n_hits + dut.n_misses + dut.n_merged),
                 dut.n_writebacks);
        if (`DC_MSHRS > 0) begin
            automatic string occ = "";
            foreach (dut.occupancy[k]) occ = {occ, $sformatf(" %0d:%0d", k, dut.occupancy[k])};
            $display("  (%0d hits under a miss, %0d refused cycles; cycles with k MSHRs busy:%s)",
                     dut.n_hit_under_miss, dut.n_refused, occ);
            // the random test must reach what it is meant to check
            if (dut.n_hit_under_miss < 100)                 fail("too few hits under a miss");
            if (`DC_TGTS > 1 && dut.n_merged < 100)         fail("too few merged misses");
            if (dut.occupancy[`DC_MSHRS] < 10)              fail("(almost) never ran with all MSHRs busy");
        end
        if (errors == 0) $display("@@@ Passed");
        else             $display("@@@ Failed (%0d errors)", errors);
        $finish;
    end

endmodule
