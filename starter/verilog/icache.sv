/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  icache.sv                                           //
//                                                                     //
//  Description :  The instruction cache module that reroutes memory   //
//                 accesses to decrease misses. Direct-mapped, 8-byte  //
//                 lines, ICACHE_LINES lines (sys_defs). Tags and      //
//                 valid bits are flip-flops (read combinationally:    //
//                 a tag miss starts the memory request). Data:        //
//                  - ICACHE_SRAM = 1: the ASAP7 SRAM bank (4 KB,      //
//                    synchronous read). It is read at next_addr, the  //
//                    address the fetch stage presents NEXT cycle, so  //
//                    its output lines up with proc2Icache_addr; data  //
//                    counts only if that read was for this index (a   //
//                    refill writes the bank and is re-read the next   //
//                    cycle: one bubble per miss).                     //
//                  - ICACHE_SRAM = 0: flip-flops, combinational read  //
//                    (next_addr unused), any size.                    //
//                 With the stream buffer (iprefetch.sv): a missing    //
//                 block the buffer holds is installed from it (pf_take)//
//                 like a memory fill; one it is still fetching is     //
//                 waited for (no miss is sent). A second tag port     //
//                 (probe) lets it skip blocks that are already here.  //
//                 Threads (SMT): one thread fetches per cycle         //
//                 (fetch_tid). Each thread has its own P3 miss state  //
//                 (last address, memory tag, outstanding flag); the   //
//                 fetching thread runs the P3 logic on its own, and   //
//                 another thread's accepted miss still completes: its //
//                 data is installed at that thread's missed block     //
//                 when it returns. That fill takes the one write port //
//                 first (memory data is on the bus once; a stream-    //
//                 buffer install of the fetching thread waits a       //
//                 cycle). One stream buffer and probe port per thread.//
//                 Index hash: every thread's code starts at its       //
//                 address 0, so with several threads the top index    //
//                 bits are XORed with the thread's number (the        //
//                 physical address bits above THREAD_MEM_SZ); the tag //
//                 keeps those bits, so an address still has one line. //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

// Internal macros, no other file should need these
`define CACHE_LINES `ICACHE_LINES
`define CACHE_LINE_BITS $clog2(`CACHE_LINES)

/**
 * A quick overview of the cache and memory:
 *
 * We've increased the memory latency from 1 cycle to 100ns. which will be
 * multiple cycles for any reasonable processor. Thus, memory can have multiple
 * transactions pending and coordinates them via memory tags (different meaning
 * than cache tags) which represent a transaction it's working on. Memory tags
 * are 4 bits long since 15 mem accesses can be live at one time, and only one
 * access happens per cycle.
 *
 * On a request, memory *responds* with the tag it will use for that request
 * then ceiling(100ns/clock period) cycles later, it will return the data with
 * the corresponding tag. The 0 tag is a sentinel value and unused. It would be
 * very difficult to push your clock period past 100ns/15=6.66ns, so 15 tags is
 * sufficient.
 *
 * This cache coordinates those memory tags to speed up fetching reused data.
 *
 * Note that this cache is blocking, and will wait on one memory request before
 * sending another (unless the input address changes, in which case it abandons
 * that request). Implementing a non-blocking cache can count towards simple
 * feature points, but will require careful management of memory tags.
 */

module icache #(
    localparam NT = `NUM_THREADS
) (
    input clock,
    input reset,
    input [`TID_W-1:0] fetch_tid,        // the thread whose address this is (proc2Icache_addr)

    // From memory
    input [3:0]  Imem2proc_response, // Should be zero unless there is a response
    input [63:0] Imem2proc_data,
    input [3:0]  Imem2proc_tag,

    // From fetch stage
    input [`XLEN-1:0] proc2Icache_addr,
    input [`XLEN-1:0] next_addr,          // proc2Icache_addr in the next cycle (SRAM read address)

    // To memory
    output logic [1:0]       proc2Imem_command,
    output logic [`XLEN-1:0] proc2Imem_addr,

    // To fetch stage
    output logic [63:0] Icache_data_out, // Data is mem[proc2Icache_addr]
    output logic        Icache_valid_out, // When valid is high

    // stream buffer (iprefetch.sv; all 0 when it is off)
    input               pf_ready,         // it holds the block at proc2Icache_addr: pf_data
    input               pf_wait,          // it is fetching that block
    input  [63:0]       pf_data,
    output logic        pf_take,          // installing pf_data this cycle
    /* verilator lint_off UNUSEDSIGNAL */
    input  [NT-1:0][`XLEN-1:0] probe_addr,   // one per thread's stream buffer (only the bits this 64 KB memory uses)
    /* verilator lint_on UNUSEDSIGNAL */
    output logic [NT-1:0]      probe_hit,    // the block at probe_addr[t] is here
    output logic [NT-1:0]      miss_waiting  // thread t's miss is in memory (fetching it now only waits)
);

    // ---- Tags (13 bits of line address, since only 16 bits address all memory) ---- //

    logic [12-`CACHE_LINE_BITS:0] tags [`CACHE_LINES];
    logic [`CACHE_LINES-1:0]      valids;

    // ---- Addresses and final outputs ---- //

    // Note: cache tags, not memory tags
    logic [12-`CACHE_LINE_BITS:0] current_tag;
    logic [`CACHE_LINE_BITS - 1:0] current_index, next_index;
    logic [NT-1:0][12-`CACHE_LINE_BITS:0]  last_tag;      // per thread: the block it fetched last
    logic [NT-1:0][`CACHE_LINE_BITS - 1:0] last_index;
    logic tag_hit, data_ok;

    // line index of a physical address (thread-hashed with several threads)
    function automatic logic [`CACHE_LINE_BITS - 1:0] line_index(input logic [`XLEN-1:0] a);
        automatic logic [`CACHE_LINE_BITS - 1:0] i = a[3 +: `CACHE_LINE_BITS];
        if (NT > 1)
            i[`CACHE_LINE_BITS - 1 -: `TID_W] ^= `TID_W'(a >> $clog2(`THREAD_MEM_SZ));
        return i;
    endfunction
    assign current_tag   = proc2Icache_addr[15:3 + `CACHE_LINE_BITS];
    assign current_index = line_index(proc2Icache_addr);
    assign next_index    = line_index(next_addr);

    assign tag_hit          = valids[current_index] && (tags[current_index] == current_tag);
    assign Icache_valid_out = tag_hit && data_ok;

    always_comb
        for (int t = 0; t < NT; t++) begin
            automatic logic [12-`CACHE_LINE_BITS:0]  pt;
            automatic logic [`CACHE_LINE_BITS - 1:0] pi;
            pt = probe_addr[t][15:3 + `CACHE_LINE_BITS];
            pi = line_index(probe_addr[t]);
            probe_hit[t] = valids[pi] && (tags[pi] == pt);
        end

    // ---- Main cache logic ---- //

    logic [NT-1:0][3:0] mem_tag_t;     // per thread: the memory tag its miss waits on
    logic [NT-1:0]      miss_out_t;    // per thread: a miss still to be sent
    logic [3:0] current_mem_tag; // The current memory tag we might be waiting on (fetching thread)
    logic miss_outstanding; // Whether a miss has received its response tag to wait on (fetching thread)
    assign current_mem_tag  = mem_tag_t[fetch_tid];
    always_comb for (int t = 0; t < NT; t++) miss_waiting[t] = (mem_tag_t[t] != 0);
    assign miss_outstanding = miss_out_t[fetch_tid];

    wire changed_addr = (current_index != last_index[fetch_tid]) || (current_tag != last_tag[fetch_tid]);

    // another thread's miss returning: install it at that thread's block
    logic                          other_fill;
    logic [`TID_W-1:0]             other_tid;
    logic [`CACHE_LINE_BITS - 1:0] other_index;
    always_comb begin
        other_fill = 1'b0; other_tid = '0;
        for (int t = 0; t < NT; t++)
            if (`TID_W'(t) != fetch_tid && mem_tag_t[t] != 0 && mem_tag_t[t] == Imem2proc_tag) begin
                other_fill = 1'b1; other_tid = `TID_W'(t);
            end
        other_index = last_index[other_tid];
    end

    // Fixed for the OoO core: data that arrives in the cycle the address changes belongs to the
    // old address; without !changed_addr it was written under the NEW address's tag (fetch
    // redirect during a miss would then fetch wrong instructions).
    wire got_mem_data = (current_mem_tag == Imem2proc_tag) && (current_mem_tag != 0) && !changed_addr;

    // the stream buffer has the missing block (install it now) or is fetching it (wait for it)
    wire pf_cover = pf_ready || pf_wait;
    assign pf_take = !tag_hit && pf_ready && !got_mem_data && !other_fill;
    wire got_data = got_mem_data || pf_take;       // (the fetching thread's block)
    wire any_fill = got_data || other_fill;
    wire [`CACHE_LINE_BITS - 1:0] fill_index = got_data ? current_index : other_index;
    wire [63:0] fill_data = pf_take ? pf_data : Imem2proc_data;

    // Set mem tag to zero if we changed_addr, and keep resetting while there is
    // a miss_outstanding. Then set to zero when we got_mem_data.
    // (this relies on Imem2proc_response being zero when there is no request)
    wire update_mem_tag = changed_addr || miss_outstanding || got_mem_data;

    // If we have a new miss or still waiting for the response tag, we might
    // need to wait for the response tag because dcache has priority over icache
    // (a miss is a tag miss: data that is merely not read out yet is not one)
    // (SMT: another thread's fill can evict the block a thread is waiting at; nothing in flight for
    //  it any more -> miss again. With one thread that cannot happen.)
    wire unanswered_miss = changed_addr ? !tag_hit && !pf_cover
                                        : ((miss_outstanding && (Imem2proc_response == 0)) ||
                                           (!miss_outstanding && !tag_hit && current_mem_tag == 0)) && !pf_cover;

    // Keep sending memory requests until we receive a response tag or change addresses
    assign proc2Imem_command = (miss_outstanding && !changed_addr && !pf_cover) ? BUS_LOAD : BUS_NONE;
    assign proc2Imem_addr    = {proc2Icache_addr[31:3],3'b0};

    // ---- Cache state registers ---- //

    always_ff @(posedge clock) begin
        if (reset) begin
            for (int t = 0; t < NT; t++) begin
                last_index[t] <= -1; // These are -1 to get ball rolling when
                last_tag[t]   <= -1; // reset goes low because addr "changes"
            end
            mem_tag_t        <= '0;
            miss_out_t       <= '0;
            valids           <= 0;
        end else begin
            last_index[fetch_tid] <= current_index;
            last_tag[fetch_tid]   <= current_tag;
            miss_out_t[fetch_tid] <= unanswered_miss;
            if (update_mem_tag) begin
                mem_tag_t[fetch_tid] <= Imem2proc_response;
            end
            if (got_data) begin // If data came from memory (tag matches) or the stream buffer
                tags[current_index]   <= current_tag;
                valids[current_index] <= 1;
            end
            if (other_fill) begin // another thread's miss came back: its block, its tag; it is done
                tags[other_index]    <= last_tag[other_tid];
                valids[other_index]  <= 1;
                mem_tag_t[other_tid] <= '0;
            end
        end
    end

    // ---- Cache data ---- //

    if (`ICACHE_SRAM) begin : gen_sram
        // ASAP7 srambank_128x4x64_6t122: 512 x 64, one port (a write blocks that cycle's read)
        logic                         rd_ok;     // dataout holds the line read at the last edge
        logic [`CACHE_LINE_BITS-1:0]  rd_index;  // ... from this index
        srambank_128x4x64_6t122 data_0 (
            .clk    (clock),
            .ADDRESS(any_fill ? fill_index : next_index),
            .wd     (fill_data),
            .banksel(1'b1),
            .read   (!any_fill),
            .write  (any_fill),
            .dataout(Icache_data_out)
        );
        always_ff @(posedge clock) begin
            if (reset) rd_ok <= 0;
            else       rd_ok <= !any_fill;
            rd_index <= next_index;
        end
        assign data_ok = rd_ok && (rd_index == current_index);
    end else begin : gen_flops
        logic [63:0] lines [`CACHE_LINES];
        always_ff @(posedge clock) if (!reset && any_fill) lines[fill_index] <= fill_data;
        assign Icache_data_out = lines[current_index];
        assign data_ok         = 1'b1;
    end

endmodule // icache
