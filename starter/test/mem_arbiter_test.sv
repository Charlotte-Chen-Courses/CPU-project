/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  mem_arbiter_test.sv                                 //
//                                                                     //
//  Description :  Testbench for the memory arbiter with both real     //
//                 clients (dcache.sv and P3's icache.sv) on one       //
//                 mem.sv. The D-side runs random loads/stores on a    //
//                 data region checked against a reference byte array  //
//                 (every load answer, and final memory = mem.sv with  //
//                 the cache's dirty blocks overlaid). The I-side      //
//                 fetches from a separate code region: mostly         //
//                 sequential, sometimes jumping, and sometimes        //
//                 changing address in the middle of a miss, so        //
//                 abandoned I-cache requests come back and must be    //
//                 ignored (and a phase that never abandons, as in the //
//                 core); every valid I-cache block must equal         //
//                 memory. Both sides must keep making progress.       //
//                 Prints @@@ Passed/Failed.                           //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

/* verilator lint_off BLKSEQ */
/* verilator lint_off PROCASSINIT */

module testbench;

    localparam SETS  = `DC_SZ / (`DC_BLK * `DC_WAYS);
    localparam WAY_B = `DC_SZ / `DC_WAYS;
    localparam logic [`XLEN-1:0] CODE = 32'h8000;   // I-side region (never written)

    logic                   clock, reset;
    // D-cache CPU side
    logic                   req_valid, req_ready, req_store, resp_valid;
    logic [`XLEN-1:0]       req_addr, req_data, resp_data;
    MEM_SIZE                req_size;
    logic [`DMEM_ID_W-1:0]  req_id, resp_id;
    // I-cache CPU side
    logic [`XLEN-1:0]       fetch_addr;
    logic [63:0]            icache_data;
    logic                   icache_valid;
    // cache <-> arbiter
    logic [1:0]             d_command, i_command;
    logic [`XLEN-1:0]       d_addr, i_addr;
    logic [63:0]            d_data, rdata;
    logic [3:0]             d_response, d_tag, i_response, i_tag;
    // arbiter <-> memory
    logic [1:0]             mem_command;
    logic [`XLEN-1:0]       mem_addr;
    logic [63:0]            mem_data, mem_rdata;
    logic [3:0]             mem_response, mem_tag;

    dcache dcache_0 (
        .clock, .reset, .req_valid, .req_ready, .req_store, .req_addr, .req_size, .req_data, .req_id,
        .resp_valid, .resp_id, .resp_data,
        .mem_command(d_command), .mem_addr(d_addr), .mem_data(d_data),
        .mem_response(d_response), .mem_rdata(rdata), .mem_tag(d_tag)
    );

    icache icache_0 (
        .clock, .reset,
        .Imem2proc_response(i_response), .Imem2proc_data(rdata), .Imem2proc_tag(i_tag),
        .proc2Icache_addr(fetch_addr),
        .proc2Imem_command(i_command), .proc2Imem_addr(i_addr),
        .Icache_data_out(icache_data), .Icache_valid_out(icache_valid)
    );

    mem_arbiter dut (
        .clock, .reset,
        .d_command, .d_addr, .d_data, .d_response, .d_tag,
        .i_command, .i_addr, .i_response, .i_tag,
        .rdata,
        .mem_command, .mem_addr, .mem_data, .mem_response, .mem_rdata, .mem_tag
    );

    mem memory (
        .clk(clock), .proc2mem_addr(mem_addr), .proc2mem_data(mem_data), .proc2mem_command(mem_command),
        .mem2proc_response(mem_response), .mem2proc_data(mem_rdata), .mem2proc_tag(mem_tag)
    );

    always #(`CLOCK_PERIOD / 2.0) clock = ~clock;

    int errors = 0;
    task automatic fail(input string m); errors++; $display("  FAIL: %s", m); if (errors > 20) $fatal(1, "too many errors"); endtask

    // ---------------- D side: reference model ----------------
    logic [7:0] ref_mem [`MEM_SIZE_IN_BYTES];
    bit         touched [int];
    typedef struct { logic [`DMEM_ID_W-1:0] id; logic [`XLEN-1:0] word; } exp_t;
    exp_t exp_q [$];
    int n_loads = 0, n_resps = 0, n_dreq = 0;
    logic [`DMEM_ID_W-1:0] next_id = 0;

    function automatic logic [`XLEN-1:0] ref_word(input logic [`XLEN-1:0] a);
        automatic int b = int'(a) & ~3;
        return {ref_mem[b + 3], ref_mem[b + 2], ref_mem[b + 1], ref_mem[b]};
    endfunction

    always @(posedge clock)
        if (!reset && req_valid && req_ready) begin
            automatic int n = (req_size == BYTE) ? 1 : (req_size == HALF) ? 2 : 4;
            n_dreq++;
            for (int k = 0; k < n; k++) touched[int'(req_addr) + k] = 1;
            if (req_store) for (int k = 0; k < n; k++) ref_mem[int'(req_addr) + k] = req_data[8 * k +: 8];
            else begin exp_q.push_back('{id: req_id, word: ref_word(req_addr)}); n_loads++; end
        end

    always @(negedge clock)
        if (!reset && resp_valid) begin
            n_resps++;
            if (exp_q.size() == 0) fail("D: response with no load outstanding");
            else begin
                automatic exp_t e = exp_q.pop_front();
                if (resp_id !== e.id || resp_data !== e.word)
                    fail($sformatf("D: load id %0d got %h, expected %h", e.id, resp_data, e.word));
            end
        end

    task automatic access(input bit st, input logic [`XLEN-1:0] a, input MEM_SIZE s);
        int guard = 0;
        req_valid = 1; req_store = st; req_addr = a; req_size = s; req_data = $urandom; req_id = next_id;
        while (!req_ready) begin @(negedge clock); if (++guard > 5000) $fatal(1, "D: request to %h never accepted", a); end
        @(negedge clock);
        next_id++;
        req_valid = 0;
    endtask

    // ---------------- I side: fetch process ----------------
    // decides at the falling edge, but fetch_addr changes at the rising edge, like a PC register
    // (changing it at the falling edge would race mem.sv's sampling of the I-cache command)
    int  n_ifetch = 0, n_abandon = 0, i_wait = 0, i_max_wait = 0;
    bit  running = 0;
    bit  may_abandon = 1;   // 0: like a real fetch stage, wait for every miss (no address change)
    logic [`XLEN-1:0] next_fetch;
    always @(posedge clock) fetch_addr <= next_fetch;
    always @(negedge clock) begin
        next_fetch = fetch_addr;
        if (reset || !running) next_fetch = CODE;
        else begin
            if (icache_valid) begin
                automatic logic [63:0] m = memory.unified_memory[fetch_addr[`XLEN-1:3]];
                if (icache_data !== m) fail($sformatf("I: block %h = %h, memory has %h", fetch_addr, icache_data, m));
                n_ifetch++;
                i_wait = 0;
                case ($urandom % 8)
                    0, 1:    next_fetch = CODE + 8 * $urandom_range(0, 511);   // jump
                    default: next_fetch = fetch_addr + 8;                       // sequential
                endcase
                if (next_fetch >= CODE + 4096) next_fetch = CODE;
            end else begin
                i_wait++;
                if (i_wait > i_max_wait) i_max_wait = i_wait;
                if (i_wait > 400) $fatal(1, "I: fetch of %h never completed", fetch_addr);
                if (may_abandon && $urandom % 12 == 0) begin                     // abandon the miss
                    next_fetch = CODE + 8 * $urandom_range(0, 511);
                    n_abandon++;
                    i_wait = 0;
                end
            end
        end
    end

    function automatic logic [`XLEN-1:0] align(input logic [`XLEN-1:0] a, input MEM_SIZE s);
        return (s == WORD) ? a & ~32'h3 : (s == HALF) ? a & ~32'h1 : a;
    endfunction

    initial begin
        clock = 0; reset = 1; fetch_addr = CODE; req_valid = 0; req_store = 0; req_addr = 0; req_size = WORD; req_data = 0; req_id = 0;
        for (int i = 0; i < `MEM_64BIT_LINES; i++) begin
            automatic logic [63:0] v = {$urandom, $urandom};
            memory.unified_memory[i] = v;
            for (int k = 0; k < 8; k++) ref_mem[8 * i + k] = v[8 * k +: 8];
        end
        repeat (2) @(negedge clock); reset = 0; @(negedge clock);
        running = 1;

        $display("Test 1: 10000 random D requests while the I side fetches (and abandons misses)");
        for (int n = 0; n < 10000; n++) begin
            automatic MEM_SIZE s = MEM_SIZE'($urandom_range(0, 2));
            automatic logic [`XLEN-1:0] a = ($urandom % 4 == 0) ? 32'h400 + $urandom_range(0, 127) + WAY_B * $urandom_range(1, 4)
                                                                : 32'h400 + $urandom_range(0, 127);
            access($urandom % 3 == 0, align(a, s), s);
            if ($urandom % 4 == 0) repeat ($urandom_range(1, 6)) @(negedge clock);
        end
        $display("Test 2: 5000 random D requests; the I side never abandons a miss (as in the core)");
        may_abandon = 0;
        for (int n = 0; n < 5000; n++) begin
            automatic MEM_SIZE s = MEM_SIZE'($urandom_range(0, 2));
            automatic logic [`XLEN-1:0] a = 32'h400 + $urandom_range(0, 127) + WAY_B * $urandom_range(0, 4);
            access($urandom % 3 == 0, align(a, s), s);
        end
        may_abandon = 1;

        $display("Test 3: D side idle for a while: the I side keeps going");
        begin
            automatic int start_n = n_ifetch;
            repeat (500) @(negedge clock);
            if (n_ifetch == start_n) fail("I side made no progress while the D side was idle");
        end
        running = 0;
        while (!req_ready || exp_q.size() != 0) @(negedge clock);
        repeat (3 * int'(`MEM_LATENCY_IN_CYCLES)) @(negedge clock);

        begin
            automatic logic [7:0] actual [int];
            foreach (touched[a]) actual[a] = memory.unified_memory[a / 8][8 * (a % 8) +: 8];
            for (int s = 0; s < SETS; s++)
                for (int w = 0; w < `DC_WAYS; w++)
                    if (dcache_0.lines[s][w].valid && dcache_0.lines[s][w].dirty) begin
                        automatic int base = (int'(dcache_0.lines[s][w].tag) * SETS + s) * 8;
                        for (int k = 0; k < 8; k++) if (actual.exists(base + k)) actual[base + k] = dcache_0.lines[s][w].data[8 * k +: 8];
                    end
            foreach (touched[a])
                if (actual[a] !== ref_mem[a]) fail($sformatf("D: final memory byte %h = %h, expected %h", a, actual[a], ref_mem[a]));
        end
        if (n_resps != n_loads) fail($sformatf("D: %0d loads, %0d responses", n_loads, n_resps));

        $display("  (D: %0d requests, %0d misses, %0d writebacks; I: %0d blocks fetched, %0d misses abandoned, longest wait %0d cycles)",
                 n_dreq, dcache_0.n_misses, dcache_0.n_writebacks, n_ifetch, n_abandon, i_max_wait);
        if (errors == 0) $display("@@@ Passed");
        else             $display("@@@ Failed (%0d errors)", errors);
        $finish;
    end

endmodule
