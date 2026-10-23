/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  mem_arbiter.sv                                      //
//                                                                     //
//  Description :  Shares P3's single-ported mem.sv between the        //
//                 I-cache, the D-cache and the instruction prefetcher.//
//                  - Each cycle one command goes to memory, by fixed  //
//                    priority: a D-cache demand (a load miss usually  //
//                    blocks commit), an I-cache miss, a D-cache       //
//                    prefetch (d_prefetch), an instruction prefetch.  //
//                    Prefetches only use cycles no demand wants. The  //
//                    losers just keep asking (every client retries    //
//                    until it sees a nonzero response tag).           //
//                  - The response tag (which says "accepted, this is  //
//                    your memory tag") goes only to the client whose  //
//                    command was forwarded; the others see 0. P3's    //
//                    I-cache depends on this.                         //
//                  - An owner table, indexed by memory tag and set    //
//                    when a load is accepted, sends each returning    //
//                    data tag only to the side that asked for it      //
//                    (D, or I: the I-cache and the prefetcher each    //
//                    match their own tags; stores never return data). //
//                    Data is shared; a client only looks at it when   //
//                    its tag matches.                                 //
//                 Purely combinational except for the owner table.    //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

module mem_arbiter (
    input                    clock,
    input                    reset,

    // D-cache
    input        [1:0]       d_command,
    input        [`XLEN-1:0] d_addr,
    input        [63:0]      d_data,
    input                    d_prefetch,    // the D-cache's command is a prefetch (lowest D priority)
    output logic [3:0]       d_response,
    output logic [3:0]       d_tag,

    // I-cache (loads only)
    input        [1:0]       i_command,
    input        [`XLEN-1:0] i_addr,
    output logic [3:0]       i_response,
    output logic [3:0]       i_tag,        // (for the I-cache and the instruction prefetcher)

    // instruction prefetcher (loads only)
    input        [1:0]       p_command,
    input        [`XLEN-1:0] p_addr,
    output logic [3:0]       p_response,

    // both caches read the returning data
    output logic [63:0]      rdata,

    // mem.sv
    output logic [1:0]       mem_command,
    output logic [`XLEN-1:0] mem_addr,
    output logic [63:0]      mem_data,
    input        [3:0]       mem_response,
    input        [63:0]      mem_rdata,
    input        [3:0]       mem_tag
);

  logic d_sel, i_sel, p_sel;                // whose command goes to memory this cycle
  logic owner_d [1:`NUM_MEM_TAGS];          // 1: this memory tag's data belongs to the D-cache

  always_comb begin
    d_sel = `FALSE; i_sel = `FALSE; p_sel = `FALSE;
    if      (d_command != BUS_NONE && !d_prefetch) d_sel = `TRUE;
    else if (i_command != BUS_NONE)                i_sel = `TRUE;
    else if (d_command != BUS_NONE)                d_sel = `TRUE;
    else if (p_command != BUS_NONE)                p_sel = `TRUE;
  end
  assign mem_command = d_sel ? d_command : i_sel ? i_command : p_sel ? p_command : BUS_NONE;
  assign mem_addr    = d_sel ? d_addr    : i_sel ? i_addr    : p_addr;
  assign mem_data    = d_sel ? d_data    : 64'b0;

  assign d_response  = d_sel ? mem_response : 4'd0;
  assign i_response  = i_sel ? mem_response : 4'd0;
  assign p_response  = p_sel ? mem_response : 4'd0;

  assign rdata = mem_rdata;
  assign d_tag = (mem_tag != 4'd0 &&  owner_d[mem_tag]) ? mem_tag : 4'd0;
  assign i_tag = (mem_tag != 4'd0 && !owner_d[mem_tag]) ? mem_tag : 4'd0;

  always_ff @(posedge clock) begin
    if (reset) begin
      for (int t = 1; t <= `NUM_MEM_TAGS; t++) owner_d[t] <= 1'b0;
    end else if (mem_response != 4'd0 && mem_command == BUS_LOAD) begin
      owner_d[mem_response] <= d_sel;
    end
  end

endmodule  // mem_arbiter
