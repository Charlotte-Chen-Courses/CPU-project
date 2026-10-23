/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  iprefetch.sv                                        //
//                                                                     //
//  Description :  Instruction stream buffer (Jouppi, ISCA 1990):      //
//                 IPF_DEPTH 8-byte blocks fetched ahead of the        //
//                 I-cache, held here (flip-flops) rather than written //
//                 into the single-ported I-cache SRAM, so a prefetch  //
//                 never takes fetch's read port. IPF_DEPTH 0 = off.   //
//                                                                     //
//                 Window: when memory accepts an I-cache miss to      //
//                 block X, the buffer should hold X+1 .. X+DEPTH:     //
//                 entries already holding one of those blocks stay,   //
//                 the others are reassigned to the missing ones. When //
//                 the I-cache installs a block from the buffer, that  //
//                 entry moves on to the next block after the window.  //
//                 Lookup is associative: the I-cache asks for the     //
//                 block it is fetching; ready = install it now,       //
//                 wait = the data is on its way (do not send a miss). //
//                                                                     //
//                 Memory: one prefetch command at a time (the lowest  //
//                 entry still to send), as the arbiter's lowest       //
//                 priority, and only while fewer than DEPTH           //
//                 prefetches are in memory, counted by memory tag, so //
//                 a dropped entry's request still counts until its    //
//                 data comes back (each one holds a memory tag for    //
//                 the full latency). A block the I-cache already      //
//                 holds is skipped. Data returns on the I side's tag  //
//                 bus; an entry takes it only if it is waiting for    //
//                 that tag (data for a dropped entry is ignored).     //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

module iprefetch #(
    localparam NE    = (`IPF_DEPTH > 0) ? `IPF_DEPTH : 1,
    localparam E_W   = (NE > 1) ? $clog2(NE) : 1,
    localparam BLK_W = `XLEN - 3
) (
    input                          clock,
    input                          reset,

    // I-cache
    /* verilator lint_off UNUSEDSIGNAL */          // (byte offsets)
    input        [`XLEN-1:0]       cur_addr,       // the block the I-cache is fetching
    output logic                   pf_ready,       // ... is here: install pf_data now
    output logic                   pf_wait,        // ... is on its way
    output logic [63:0]            pf_data,
    input                          pf_take,        // the I-cache installs it this cycle
    input                          miss_accepted,  // memory accepted an I-cache miss ...
    input        [`XLEN-1:0]       miss_addr,      // ... for this block
    /* verilator lint_on UNUSEDSIGNAL */
    output logic [`XLEN-1:0]       probe_addr,     // is this block already in the I-cache?
    input                          probe_hit,

    // memory (through the arbiter, lowest priority)
    output logic [1:0]             p_command,
    output logic [`XLEN-1:0]       p_addr,
    input        [3:0]             p_response,
    input        [3:0]             i_tag,          // returning I-side data (the I-cache's or ours)
    input        [63:0]            rdata
);

  typedef enum logic [1:0] {
    E_IDLE,      // nothing to fetch (past the end of memory)
    E_REQ,       // to be sent
    E_WAIT,      // in memory, waiting for memtag
    E_READY      // data here
  } E_STATE;

  typedef struct packed {
    E_STATE           st;
    logic [BLK_W-1:0] blk;
    logic [3:0]       memtag;
    logic [63:0]      data;
  } PF_ENTRY;

  PF_ENTRY                ent [NE];
  logic [BLK_W-1:0]       next_blk;                 // the block after the window
  logic [`NUM_MEM_TAGS:1] pf_tags;                  // memory tags held by our requests

  // statistics (simulation only, read by testbenches)
  /* verilator lint_off PROCASSINIT */
  /* verilator lint_off UNUSEDSIGNAL */
  int n_issued = 0, n_useful = 0, n_late = 0, n_dropped = 0, n_skipped = 0;
  /* verilator lint_on UNUSEDSIGNAL */
  /* verilator lint_on PROCASSINIT */

  function automatic logic in_mem(input logic [BLK_W-1:0] b);
    return {b, 3'b000} < `XLEN'(`MEM_SIZE_IN_BYTES);
  endfunction

  if (`IPF_DEPTH == 0) begin : gen_off
    assign pf_ready   = `FALSE;
    assign pf_wait    = `FALSE;
    assign pf_data    = '0;
    assign probe_addr = '0;
    assign p_command  = BUS_NONE;
    assign p_addr     = '0;
    always_ff @(posedge clock) begin
      next_blk <= '0;
      pf_tags  <= '0;
      for (int e = 0; e < NE; e++) ent[e] <= '0;
    end
    /* verilator lint_off UNUSEDSIGNAL */
    logic unused;
    assign unused = ^{reset, cur_addr, pf_take, miss_accepted, miss_addr, probe_hit, p_response, i_tag, rdata, next_blk,
                      pf_tags, ent[0]};
    /* verilator lint_on UNUSEDSIGNAL */
  end else begin : gen_on

    // ---- lookup for the I-cache ----
    logic [BLK_W-1:0] cur_blk;
    logic [E_W-1:0]   hit_e;
    logic             hit;
    assign cur_blk = cur_addr[`XLEN-1:3];
    always_comb begin
      hit = `FALSE; hit_e = '0;
      for (int e = 0; e < NE; e++)
        if ((ent[e].st == E_WAIT || ent[e].st == E_READY) && ent[e].blk == cur_blk) begin
          hit = `TRUE; hit_e = E_W'(e);
        end
    end
    assign pf_ready = hit && ent[hit_e].st == E_READY;
    assign pf_wait  = hit && ent[hit_e].st == E_WAIT;
    assign pf_data  = ent[hit_e].data;

    // ---- the request to send: the lowest entry still to send ----
    logic           iss_any, room;
    logic [E_W-1:0] iss_e;
    always_comb begin
      iss_any = `FALSE; iss_e = '0;
      for (int e = NE - 1; e >= 0; e--)
        if (ent[e].st == E_REQ) begin iss_any = `TRUE; iss_e = E_W'(e); end
    end
    assign room       = $countones(pf_tags) < NE;
    assign probe_addr = {ent[iss_e].blk, 3'b000};
    assign p_addr     = {ent[iss_e].blk, 3'b000};
    assign p_command  = (iss_any && room && !probe_hit) ? BUS_LOAD : BUS_NONE;

    // ---- window restart after an I-cache miss to block m ----
    logic [BLK_W-1:0] m_blk;
    logic [NE-1:0]    keep;          // entry already holds a block of the new window
    logic [NE-1:0]    have;          // window block k (m+1+k) is held by a kept entry
    logic [BLK_W-1:0] new_blk [NE];  // block for each entry that is not kept
    assign m_blk = miss_addr[`XLEN-1:3];
    always_comb begin
      keep = '0; have = '0;
      for (int e = 0; e < NE; e++)
        for (int k = 0; k < NE; k++)
          if (ent[e].st != E_IDLE && ent[e].blk == m_blk + BLK_W'(k + 1)) begin
            keep[e] = `TRUE; have[k] = `TRUE;
          end
      // the r-th entry not kept gets the r-th window block not held
      for (int e = 0; e < NE; e++) begin
        automatic int r = 0, seen = 0;
        new_blk[e] = m_blk + 1'b1;
        for (int f = 0; f < NE; f++) if (f < e && !keep[f]) r++;
        for (int k = 0; k < NE; k++)
          if (!have[k]) begin
            if (seen == r) new_blk[e] = m_blk + BLK_W'(k + 1);
            seen++;
          end
      end
    end

    always_ff @(posedge clock) begin
      if (reset) begin
        next_blk <= '0;
        pf_tags  <= '0;
        for (int e = 0; e < NE; e++) ent[e].st <= E_IDLE;
      end else begin
        // memory tags: ours from acceptance until the data returns
        begin
          automatic logic [`NUM_MEM_TAGS:1] t = pf_tags;
          if (i_tag != 4'd0) t[i_tag] = 1'b0;
          if (p_command != BUS_NONE && p_response != 4'd0) t[p_response] = 1'b1;
          pf_tags <= t;
        end
        // data for a waiting entry
        for (int e = 0; e < NE; e++)
          if (ent[e].st == E_WAIT && i_tag != 4'd0 && i_tag == ent[e].memtag) begin
            ent[e].data <= rdata;
            ent[e].st   <= E_READY;
          end

        if (miss_accepted) begin
          // new window m+1 .. m+NE: keep what is already in it, reassign the rest to the missing blocks
          for (int e = 0; e < NE; e++)
            if (!keep[e]) begin
              ent[e].blk <= new_blk[e];
              ent[e].st  <= in_mem(new_blk[e]) ? E_REQ : E_IDLE;
            end
          next_blk <= m_blk + BLK_W'(NE + 1);
        end else begin
          // an entry moves on to next_blk when the I-cache takes its block, or when its block is cached already
          automatic logic [BLK_W-1:0] nb = next_blk;
          if (pf_take) begin
            ent[hit_e].blk <= nb;
            ent[hit_e].st  <= in_mem(nb) ? E_REQ : E_IDLE;
            nb = nb + 1'b1;
          end
          if (iss_any && probe_hit && !(pf_take && iss_e == hit_e)) begin
            ent[iss_e].blk <= nb;
            ent[iss_e].st  <= in_mem(nb) ? E_REQ : E_IDLE;
            nb = nb + 1'b1;
          end else if (p_command != BUS_NONE && p_response != 4'd0) begin
            ent[iss_e].memtag <= p_response;
            ent[iss_e].st     <= E_WAIT;
          end
          next_blk <= nb;
        end
      end
    end

    // synopsys translate_off
    /* verilator lint_off BLKSEQ */
    always @(posedge clock)
      if (!reset) begin
        if (p_command != BUS_NONE && p_response != 4'd0 && !miss_accepted) n_issued++;
        if (pf_take) n_useful++;
        if (pf_wait && !pf_take) n_late++;           // cycles the I-cache waited for a prefetch in flight
        if (iss_any && probe_hit && !miss_accepted) n_skipped++;
        if (miss_accepted)
          for (int e = 0; e < NE; e++) if (!keep[e] && (ent[e].st == E_WAIT || ent[e].st == E_READY)) n_dropped++;
      end
    /* verilator lint_on BLKSEQ */
    // synopsys translate_on
  end

endmodule  // iprefetch
