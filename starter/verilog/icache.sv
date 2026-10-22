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

module icache (
    input clock,
    input reset,

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
    output logic        Icache_valid_out // When valid is high
);

    // ---- Tags (13 bits of line address, since only 16 bits address all memory) ---- //

    logic [12-`CACHE_LINE_BITS:0] tags [`CACHE_LINES];
    logic [`CACHE_LINES-1:0]      valids;

    // ---- Addresses and final outputs ---- //

    // Note: cache tags, not memory tags
    logic [12-`CACHE_LINE_BITS:0] current_tag, last_tag;
    logic [`CACHE_LINE_BITS - 1:0] current_index, last_index, next_index;
    logic tag_hit, data_ok;

    assign {current_tag, current_index} = proc2Icache_addr[15:3];
    assign next_index = next_addr[3 +: `CACHE_LINE_BITS];

    assign tag_hit          = valids[current_index] && (tags[current_index] == current_tag);
    assign Icache_valid_out = tag_hit && data_ok;

    // ---- Main cache logic ---- //

    logic [3:0] current_mem_tag; // The current memory tag we might be waiting on
    logic miss_outstanding; // Whether a miss has received its response tag to wait on

    wire changed_addr = (current_index != last_index) || (current_tag != last_tag);

    // Fixed for the OoO core: data that arrives in the cycle the address changes belongs to the
    // old address; without !changed_addr it was written under the NEW address's tag (fetch
    // redirect during a miss would then fetch wrong instructions).
    wire got_mem_data = (current_mem_tag == Imem2proc_tag) && (current_mem_tag != 0) && !changed_addr;

    // Set mem tag to zero if we changed_addr, and keep resetting while there is
    // a miss_outstanding. Then set to zero when we got_mem_data.
    // (this relies on Imem2proc_response being zero when there is no request)
    wire update_mem_tag = changed_addr || miss_outstanding || got_mem_data;

    // If we have a new miss or still waiting for the response tag, we might
    // need to wait for the response tag because dcache has priority over icache
    // (a miss is a tag miss: data that is merely not read out yet is not one)
    wire unanswered_miss = changed_addr ? !tag_hit
                                        : miss_outstanding && (Imem2proc_response == 0);

    // Keep sending memory requests until we receive a response tag or change addresses
    assign proc2Imem_command = (miss_outstanding && !changed_addr) ? BUS_LOAD : BUS_NONE;
    assign proc2Imem_addr    = {proc2Icache_addr[31:3],3'b0};

    // ---- Cache state registers ---- //

    always_ff @(posedge clock) begin
        if (reset) begin
            last_index       <= -1; // These are -1 to get ball rolling when
            last_tag         <= -1; // reset goes low because addr "changes"
            current_mem_tag  <= 0;
            miss_outstanding <= 0;
            valids           <= 0;
        end else begin
            last_index       <= current_index;
            last_tag         <= current_tag;
            miss_outstanding <= unanswered_miss;
            if (update_mem_tag) begin
                current_mem_tag <= Imem2proc_response;
            end
            if (got_mem_data) begin // If data came from memory, meaning tag matches
                tags[current_index]   <= current_tag;
                valids[current_index] <= 1;
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
            .ADDRESS(got_mem_data ? current_index : next_index),
            .wd     (Imem2proc_data),
            .banksel(1'b1),
            .read   (!got_mem_data),
            .write  (got_mem_data),
            .dataout(Icache_data_out)
        );
        always_ff @(posedge clock) begin
            if (reset) rd_ok <= 0;
            else       rd_ok <= !got_mem_data;
            rd_index <= next_index;
        end
        assign data_ok = rd_ok && (rd_index == current_index);
    end else begin : gen_flops
        logic [63:0] lines [`CACHE_LINES];
        always_ff @(posedge clock) if (!reset && got_mem_data) lines[current_index] <= Imem2proc_data;
        assign Icache_data_out = lines[current_index];
        assign data_ok         = 1'b1;
    end

endmodule // icache
