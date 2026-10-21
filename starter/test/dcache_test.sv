/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  dcache_test.sv                                      //
//                                                                     //
//  Description :  Testbench for the data cache, directly on P3's      //
//                 mem.sv (CACHE_MODE, MEM_LATENCY_IN_CYCLES). A       //
//                 reference byte array is updated when each request   //
//                 is accepted; every load response (in order, the     //
//                 cache is blocking) must match it. At the end the    //
//                 cache's valid dirty blocks are overlaid on mem.sv's //
//                 memory and every touched byte is compared, which    //
//                 catches lost or wrong writebacks. Directed: hit     //
//                 after fill, byte/half/word stores into a cached     //
//                 block, dirty eviction then re-read, LRU order; then //
//                 random traffic over a hot region, addresses that    //
//                 alias to the same sets, and far addresses, while    //
//                 memory randomly refuses requests.                   //
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
    exp_t exp_q [$];
    int errors = 0, n_loads = 0, n_resps = 0;
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
            else begin exp_q.push_back('{id: req_id, word: ref_word(req_addr)}); n_loads++; end
        end

    // responses: in order
    always @(negedge clock)
        if (!reset && resp_valid) begin
            n_resps++;
            if (exp_q.size() == 0) fail("response with no load outstanding");
            else begin
                automatic exp_t e = exp_q.pop_front();
                if (resp_id !== e.id || resp_data !== e.word)
                    fail($sformatf("load id %0d: got %h (id %0d), expected %h", e.id, resp_data, resp_id, e.word));
            end
        end

    // one request (called at a falling edge); returns at the falling edge after it is accepted
    task automatic access(input bit st, input logic [`XLEN-1:0] a, input MEM_SIZE s, input logic [`XLEN-1:0] d = $urandom);
        int guard = 0;
        req_valid = 1; req_store = st; req_addr = a; req_size = s; req_data = d; req_id = next_id;
        while (!req_ready) begin
            @(negedge clock);
            if (++guard > 2000) $fatal(1, "request to %h never accepted", a);
        end
        @(negedge clock);          // the rising edge in between accepted it
        next_id++;
        req_valid = 0;
    endtask

    task automatic wait_idle();
        int guard = 0;
        req_valid = 0;
        while (!(req_ready && exp_q.size() == 0)) begin @(negedge clock); if (++guard > 2000) $fatal(1, "cache never went idle"); end
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
        if (dut.n_misses != 1 || dut.n_hits != 3) fail($sformatf("expected 1 miss + 3 hits, got %0d + %0d", dut.n_misses, dut.n_hits));

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
            access(0, 32'h300, WORD); access(0, 32'h300 + WAY_B, WORD); access(0, 32'h300, WORD);
            access(0, 32'h300 + 2 * WAY_B, WORD);
            wait_idle();
            h0 = dut.n_hits; m0 = dut.n_misses;
            access(0, 32'h300, WORD); wait_idle();
            if (dut.n_hits != h0 + 1) fail("A was evicted: LRU picked the most recently used way");
            access(0, 32'h300 + WAY_B, WORD); wait_idle();
            if (dut.n_misses != m0 + 1) fail("B was not evicted");
        end

        $display("Test 5: 20000 random requests (hot region, same-set aliases, far addresses), memory refuses 30%%");
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
        wait_idle();
        refuse_pct = 0;
        repeat (2 * int'(`MEM_LATENCY_IN_CYCLES)) @(negedge clock);   // let mem.sv finish the last writeback

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

        $display("  (%0d hits, %0d misses: hit rate %.1f%%, %0d writebacks)", dut.n_hits, dut.n_misses,
                 100.0 * dut.n_hits / (dut.n_hits + dut.n_misses), dut.n_writebacks);
        if (errors == 0) $display("@@@ Passed");
        else             $display("@@@ Failed (%0d errors)", errors);
        $finish;
    end

endmodule
