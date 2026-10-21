/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  mem_arbiter.sv                                      //
//                                                                     //
//  Description :  Shares P3's single-ported mem.sv between the        //
//                 I-cache and the D-cache.                            //
//                  - Each cycle one command goes to memory: the       //
//                    D-cache's if it has one, else the I-cache's      //
//                    (a load miss usually blocks commit; fetch can    //
//                    often wait a cycle). The loser just keeps asking //
//                    (both caches retry until they see a nonzero      //
//                    response tag).                                   //
//                  - The response tag (which says "accepted, this is  //
//                    your memory tag") goes only to the cache whose   //
//                    command was forwarded; the other sees 0. P3's    //
//                    I-cache depends on this.                         //
//                  - An owner table, indexed by memory tag and set    //
//                    when a load is accepted, sends each returning    //
//                    data tag only to the cache that asked for it     //
//                    (stores never return data). Data is shared; a    //
//                    cache only looks at it when its tag matches.     //
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
    output logic [3:0]       d_response,
    output logic [3:0]       d_tag,

    // I-cache (loads only)
    input        [1:0]       i_command,
    input        [`XLEN-1:0] i_addr,
    output logic [3:0]       i_response,
    output logic [3:0]       i_tag,

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

  logic d_sel;                              // the D-cache's command goes to memory this cycle
  logic owner_d [1:`NUM_MEM_TAGS];          // 1: this memory tag's data belongs to the D-cache

  assign d_sel       = (d_command != BUS_NONE);
  assign mem_command = d_sel ? d_command : i_command;
  assign mem_addr    = d_sel ? d_addr    : i_addr;
  assign mem_data    = d_sel ? d_data    : 64'b0;

  assign d_response  = d_sel ? mem_response : 4'd0;
  assign i_response  = d_sel ? 4'd0         : mem_response;

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
