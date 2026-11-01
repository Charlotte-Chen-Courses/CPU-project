/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  dcache.sv                                           //
//                                                                     //
//  Description :  Data cache. DC_SZ bytes, DC_WAYS-way set            //
//                 associative, 8-byte blocks (one mem.sv bus beat),   //
//                 write-back + write-allocate, LRU replacement,       //
//                 non-blocking: DC_MSHRS miss status holding          //
//                 registers (0 = blocking, one miss at a time).       //
//                                                                     //
//                 CPU side (for the LSQ): request valid/ready with    //
//                 store, addr, size, data (low bytes), id; loads get  //
//                 a response (valid, id, the aligned 32-bit word that //
//                 holds addr), possibly out of order. Accesses must   //
//                 be naturally aligned and in memory range (the LSQ   //
//                 checks this before sending).                        //
//                                                                     //
//                 Hit: a load answers next cycle; a store merges its  //
//                 bytes into the block and sets dirty.                //
//                 Primary miss (no MSHR for the block): take a free   //
//                 MSHR and reserve a victim way (invalid first, else  //
//                 LRU, never a way another MSHR holds). The victim is //
//                 invalidated now; if it was dirty its data goes to   //
//                 the writeback buffer. The request is the MSHR's     //
//                 first target.                                       //
//                 Secondary miss (an MSHR is waiting for the block):  //
//                 append the request (load or store) to its targets.  //
//                 Memory: the writeback buffer goes first (mem.sv     //
//                 never acknowledges stores and writes memory when it //
//                 accepts one), then the lowest MSHR still to send    //
//                 its BUS_LOAD; it captures the memory tag and later  //
//                 the data with that tag.                             //
//                 Fill: the cycle after the data arrives, install the //
//                 block and apply the first target; the rest replay   //
//                 one per cycle through the hit path, in the order    //
//                 they were accepted. So every request sees exactly   //
//                 the requests accepted before it, as in a blocking   //
//                 cache (the LSQ treats acceptance as the moment a    //
//                 load reads its value).                              //
//                 Refused (ready low): while installing / replaying   //
//                 (one write to the lines per cycle, one response per //
//                 cycle) or while a writeback waits for memory (so a  //
//                 block is never refetched before its writeback);     //
//                 a primary miss with no free MSHR or no unreserved   //
//                 way; a secondary miss with a full target list.      //
//                                                                     //
//                 Memory side is P3's mem.sv protocol (CACHE_MODE:    //
//                 8-byte aligned blocks, response tag 0 = not         //
//                 accepted, keep asking).                             //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

module dcache #(
    localparam WAYS  = `DC_WAYS,
    localparam SETS  = `DC_SZ / (`DC_BLK * `DC_WAYS),
    localparam IDX_W = $clog2(SETS),
    localparam A_W   = $clog2(`MEM_SIZE_IN_BYTES),     // physical address bits used
    localparam TAG_W = A_W - IDX_W - 3,
    localparam BLK_W = A_W - 3,                        // block address
    localparam AGE_W = (WAYS > 1) ? $clog2(WAYS) : 1,
    localparam ID_W  = `DMEM_ID_W,
    localparam NM    = (`DC_MSHRS > 0) ? `DC_MSHRS : 1, // MSHRs (blocking mode uses one)
    localparam M_W   = (NM > 1) ? $clog2(NM) : 1,
    localparam NT    = (`DC_MSHRS > 0) ? `DC_TGTS : 1,  // targets per MSHR
    localparam T_W   = (NT > 1) ? $clog2(NT) : 1
) (
    input                          clock,
    input                          reset,

    // CPU side
    input                          req_valid,
    output logic                   req_ready,
    input                          req_store,
    input        [`XLEN-1:0]       req_addr,
    input  MEM_SIZE                req_size,
    input        [`XLEN-1:0]       req_data,
    input        [ID_W-1:0]        req_id,
    output logic                   resp_valid,
    output logic [ID_W-1:0]        resp_id,
    output logic [`XLEN-1:0]       resp_data,

    // memory side (mem.sv)
    output logic [1:0]             mem_command,
    output logic [`XLEN-1:0]       mem_addr,
    output logic [63:0]            mem_data,
    input        [3:0]             mem_response,
    input        [63:0]            mem_rdata,
    input        [3:0]             mem_tag
);

  typedef struct packed {
    logic             valid;
    logic             dirty;
    logic [TAG_W-1:0] tag;
    logic [AGE_W-1:0] age;     // 0 = most recently used
    logic [63:0]      data;
  } DC_LINE;

  DC_LINE lines [SETS][WAYS];

  // a request waiting in an MSHR
  typedef struct packed {
    logic             store;
    logic [2:0]       off;
    MEM_SIZE          size;
    logic [`XLEN-1:0] data;
    logic [ID_W-1:0]  id;
  } DC_TGT;

  typedef enum logic [1:0] {
    M_REQ,       // BUS_LOAD not yet accepted
    M_WAIT,      // waiting for the data with memtag
    M_FILLED,    // data in fill, to be installed
    M_REPLAY     // installed, targets 1.. replaying
  } M_STATE;

  typedef struct packed {
    logic             valid;
    M_STATE           st;
    logic [BLK_W-1:0] blk;
    logic [AGE_W-1:0] way;      // reserved way of set idx(blk)
    logic [3:0]       memtag;
    logic [63:0]      fill;
    logic [T_W:0]     ntg;      // number of targets
  } DC_MSHR;

  DC_MSHR mshr [NM];
  DC_TGT  tgts [NM][NT];

  // writeback buffer (a dirty victim's block, until memory accepts it)
  logic             wb_valid;
  logic [BLK_W-1:0] wb_blk;
  logic [63:0]      wb_data;

  // replay of targets 1.. of one MSHR
  logic             rp_active;
  logic [M_W-1:0]   rp_m;
  logic [T_W-1:0]   rp;

  // statistics (simulation only, read by testbenches)
  /* verilator lint_off PROCASSINIT */
  int n_hits = 0, n_misses = 0, n_writebacks = 0;    // misses: primary (each one fetches a block)
  int n_merged = 0;                                  // secondary misses: joined an MSHR
  int n_hit_under_miss = 0;                          // hits accepted while an MSHR was busy
  int n_refused = 0;                                 // cycles a request was refused
  int occupancy [NM + 1];                            // cycles with k MSHRs busy
  /* verilator lint_on PROCASSINIT */

  // ---------------- helpers ----------------
  /* verilator lint_off UNUSEDSIGNAL */
  // Set index: with several threads (DC_THREAD_HASH), the top index bits XOR the thread number, which
  // is in the tag's top bits (the physical address bits above THREAD_MEM_SZ); so the tag still names
  // the block, and a line's address is {tag, set ^ hmask(tag)}.
  function automatic logic [IDX_W-1:0] hmask(input logic [TAG_W-1:0] tag);
    automatic logic [IDX_W-1:0] m = '0;
    if (`NUM_THREADS > 1 && `DC_THREAD_HASH != 0)
      m[IDX_W-1 -: `TID_W] = `TID_W'(tag >> ($clog2(`THREAD_MEM_SZ) - IDX_W - 3));
    return m;
  endfunction
  function automatic logic [TAG_W-1:0] tag_of(input logic [A_W-1:0] a); return a[A_W-1:IDX_W+3]; endfunction
  function automatic logic [IDX_W-1:0] idx_of(input logic [A_W-1:0] a); return a[IDX_W+2:3] ^ hmask(tag_of(a)); endfunction
  function automatic logic [TAG_W-1:0] btag(input logic [BLK_W-1:0] b); return b[BLK_W-1:IDX_W]; endfunction
  function automatic logic [IDX_W-1:0] bidx(input logic [BLK_W-1:0] b); return b[IDX_W-1:0] ^ hmask(btag(b)); endfunction
  /* verilator lint_on UNUSEDSIGNAL */

  // write a store's bytes into a block
  function automatic logic [63:0] merge(input logic [63:0] blk, input logic [2:0] off, input MEM_SIZE s,
                                        input logic [`XLEN-1:0] d);
    automatic int n = (s == BYTE) ? 1 : (s == HALF) ? 2 : 4;
    for (int k = 0; k < 4; k++)   // constant bound: synthesis must unroll it
      if (k < n) blk[8 * (int'(off) + k) +: 8] = d[8 * k +: 8];
    return blk;
  endfunction

  function automatic logic [`XLEN-1:0] word_of(input logic [63:0] blk, input logic off2);
    return off2 ? blk[63:32] : blk[31:0];
  endfunction

  // ---------------- MSHR summary ----------------
  logic           any_busy, any_free, any_filled, any_req;
  logic [M_W-1:0] free_m, fill_m, req_m;
  always_comb begin
    any_busy = `FALSE; any_free = `FALSE; any_filled = `FALSE; any_req = `FALSE;
    free_m = '0; fill_m = '0; req_m = '0;
    for (int m = NM - 1; m >= 0; m--) begin        // ends on the lowest
      if (mshr[m].valid) any_busy = `TRUE;
      if (!mshr[m].valid) begin any_free = `TRUE; free_m = M_W'(m); end
      if (mshr[m].valid && mshr[m].st == M_FILLED) begin any_filled = `TRUE; fill_m = M_W'(m); end
      if (mshr[m].valid && mshr[m].st == M_REQ)    begin any_req    = `TRUE; req_m  = M_W'(m); end
    end
  end

  // nothing in flight: memory plus the dirty lines is the whole truth (testbenches wait for this)
  /* verilator lint_off UNUSEDSIGNAL */
  logic idle;
  /* verilator lint_on UNUSEDSIGNAL */
  assign idle = !any_busy && !wb_valid;

  // ---------------- lookup (incoming request) ----------------
  logic [A_W-1:0]   r_addr;
  logic [IDX_W-1:0] r_idx;
  logic [BLK_W-1:0] r_blk;
  logic             r_hit, r_pend, r_vict_ok;
  logic [AGE_W-1:0] r_way, r_victim;
  logic [M_W-1:0]   r_pm;                          // MSHR waiting for this block
  logic [WAYS-1:0]  r_reserved;                    // ways of this set held by an MSHR
  assign r_addr = req_addr[A_W-1:0];
  assign r_idx  = idx_of(r_addr);
  assign r_blk  = r_addr[A_W-1:3];

  always_comb begin
    r_hit      = `FALSE;
    r_way      = '0;
    r_pend     = `FALSE;
    r_pm       = '0;
    r_reserved = '0;
    for (int w = 0; w < WAYS; w++)
      if (lines[r_idx][w].valid && lines[r_idx][w].tag == tag_of(r_addr)) begin
        r_hit = `TRUE;
        r_way = AGE_W'(w);
      end
    for (int m = 0; m < NM; m++)
      if (mshr[m].valid) begin
        if (bidx(mshr[m].blk) == r_idx) r_reserved[mshr[m].way] = `TRUE;
        if (mshr[m].blk == r_blk && (mshr[m].st == M_REQ || mshr[m].st == M_WAIT)) begin
          r_pend = `TRUE;
          r_pm   = M_W'(m);
        end
      end
    // victim: an unreserved invalid way if there is one, else the least recently used unreserved way
    r_victim  = '0;
    r_vict_ok = `FALSE;
    begin
      automatic bit found_inv = 0;
      automatic logic [AGE_W-1:0] oldest = '0;
      for (int w = 0; w < WAYS; w++)
        if (!r_reserved[w] && !lines[r_idx][w].valid && !found_inv) begin
          found_inv = 1; r_victim = AGE_W'(w); r_vict_ok = `TRUE;
        end
      for (int w = 0; w < WAYS; w++)
        if (!r_reserved[w] && !found_inv && (!r_vict_ok || lines[r_idx][w].age >= oldest)) begin
          oldest = lines[r_idx][w].age; r_victim = AGE_W'(w); r_vict_ok = `TRUE;
        end
    end
  end

  logic busy_port;                 // the lines / response port is taken by an install or a replay
  assign busy_port = rp_active || any_filled || wb_valid;

  always_comb begin
    if (busy_port || (`DC_MSHRS == 0 && any_busy)) req_ready = `FALSE;
    else if (r_hit)                                req_ready = `TRUE;
    else if (r_pend)                               req_ready = (mshr[r_pm].ntg < (T_W+1)'(NT));
    else                                           req_ready = any_free && r_vict_ok;
  end

  // ---------------- memory side ----------------
  always_comb begin
    mem_command = BUS_NONE;
    mem_addr    = '0;
    mem_data    = '0;
    if (reset) begin
      // nothing during reset (before the first edge the state is not reset yet: no stray writeback)
    end else if (wb_valid) begin
      mem_command = BUS_STORE;
      mem_addr    = `XLEN'({wb_blk, 3'b000});
      mem_data    = wb_data;
    end else if (any_req) begin
      mem_command = BUS_LOAD;
      mem_addr    = `XLEN'({mshr[req_m].blk, 3'b000});
    end
  end

  // ---------------- state ----------------
  // mark way w of set s as most recently used
  task automatic touch(input logic [IDX_W-1:0] s, input logic [AGE_W-1:0] w);
    for (int v = 0; v < WAYS; v++)
      if (AGE_W'(v) == w) lines[s][v].age <= '0;
      else if (lines[s][v].age < lines[s][w].age) lines[s][v].age <= lines[s][v].age + 1'b1;
  endtask

  // the replay / install target, and where it goes
  DC_TGT            p_tgt;
  logic [IDX_W-1:0] p_set;
  logic [AGE_W-1:0] p_way;
  logic [63:0]      p_old;         // block before the target is applied
  assign p_tgt = rp_active ? tgts[rp_m][rp] : tgts[fill_m][0];
  assign p_set = rp_active ? bidx(mshr[rp_m].blk) : bidx(mshr[fill_m].blk);
  assign p_way = rp_active ? mshr[rp_m].way : mshr[fill_m].way;
  assign p_old = rp_active ? lines[p_set][p_way].data : mshr[fill_m].fill;

  always_ff @(posedge clock) begin
    if (reset) begin
      resp_valid <= `FALSE;
      wb_valid   <= `FALSE;
      rp_active  <= `FALSE;
      for (int m = 0; m < NM; m++) mshr[m].valid <= `FALSE;
      for (int s = 0; s < SETS; s++)
        for (int w = 0; w < WAYS; w++) begin
          lines[s][w].valid <= `FALSE;
          lines[s][w].dirty <= `FALSE;
          lines[s][w].age   <= AGE_W'(w);   // distinct ages: a valid LRU order from the start
        end
    end else begin
      resp_valid <= `FALSE;

      // ---- memory: the command shown this cycle was accepted ----
      if (mem_response != 4'd0) begin
        if (wb_valid) wb_valid <= `FALSE;
        else if (any_req) begin
          mshr[req_m].memtag <= mem_response;
          mshr[req_m].st     <= M_WAIT;
        end
      end
      // ---- memory: data for a waiting MSHR ----
      for (int m = 0; m < NM; m++)
        if (mshr[m].valid && mshr[m].st == M_WAIT && mem_tag != 4'd0 && mem_tag == mshr[m].memtag) begin
          mshr[m].fill <= mem_rdata;
          mshr[m].st   <= M_FILLED;
        end

      // ---- the lines: one install / replay / request per cycle ----
      if (rp_active || any_filled) begin
        // apply target p_tgt to the block (an install also writes the fill and the tag)
        if (p_tgt.store) begin
          lines[p_set][p_way].data  <= merge(p_old, p_tgt.off, p_tgt.size, p_tgt.data);
          lines[p_set][p_way].dirty <= `TRUE;
        end else begin
          resp_valid <= `TRUE;
          resp_id    <= p_tgt.id;
          resp_data  <= word_of(p_old, p_tgt.off[2]);
        end
        touch(p_set, p_way);
        if (rp_active) begin
          if ((T_W+1)'(rp) + 1'b1 == mshr[rp_m].ntg) begin
            rp_active          <= `FALSE;
            mshr[rp_m].valid   <= `FALSE;
          end else rp <= rp + 1'b1;
        end else begin
          lines[p_set][p_way].valid <= `TRUE;
          lines[p_set][p_way].tag   <= btag(mshr[fill_m].blk);
          if (!p_tgt.store) begin
            lines[p_set][p_way].data  <= mshr[fill_m].fill;
            lines[p_set][p_way].dirty <= `FALSE;
          end
          if (mshr[fill_m].ntg == 1) mshr[fill_m].valid <= `FALSE;
          else begin
            mshr[fill_m].st <= M_REPLAY;
            rp_active       <= `TRUE;
            rp_m            <= fill_m;
            rp              <= 1;
          end
        end
      end else if (req_valid && req_ready) begin
        if (r_hit) begin
          touch(r_idx, r_way);
          if (req_store) begin
            lines[r_idx][r_way].data  <= merge(lines[r_idx][r_way].data, r_addr[2:0], req_size, req_data);
            lines[r_idx][r_way].dirty <= `TRUE;
          end else begin
            resp_valid <= `TRUE;
            resp_id    <= req_id;
            resp_data  <= word_of(lines[r_idx][r_way].data, r_addr[2]);
          end
        end else if (r_pend) begin
          tgts[r_pm][mshr[r_pm].ntg[T_W-1:0]] <= '{store: req_store, off: r_addr[2:0], size: req_size,
                                                   data: req_data, id: req_id};
          mshr[r_pm].ntg <= mshr[r_pm].ntg + 1'b1;
        end else begin
          mshr[free_m] <= '{valid: `TRUE, st: M_REQ, blk: r_blk, way: r_victim, memtag: '0, fill: '0, ntg: 1};
          tgts[free_m][0] <= '{store: req_store, off: r_addr[2:0], size: req_size, data: req_data, id: req_id};
          lines[r_idx][r_victim].valid <= `FALSE;
          lines[r_idx][r_victim].dirty <= `FALSE;
          if (lines[r_idx][r_victim].valid && lines[r_idx][r_victim].dirty) begin
            wb_valid <= `TRUE;
            wb_blk   <= {lines[r_idx][r_victim].tag, r_idx ^ hmask(lines[r_idx][r_victim].tag)};
            wb_data  <= lines[r_idx][r_victim].data;
          end
        end
      end
    end
  end

  // synopsys translate_off
  /* verilator lint_off BLKSEQ */
  initial foreach (occupancy[k]) occupancy[k] = 0;
  always @(posedge clock)
    if (!reset) begin
      automatic int busy = 0;
      for (int m = 0; m < NM; m++) busy += int'(mshr[m].valid);
      occupancy[busy]++;
      if (req_valid && !req_ready) n_refused++;
      if (req_valid && req_ready) begin
        if (r_hit) begin n_hits++; if (busy != 0) n_hit_under_miss++; end
        else if (r_pend) n_merged++;
        else n_misses++;
      end
      if (wb_valid && mem_response != 4'd0) n_writebacks++;
    end
  /* verilator lint_on BLKSEQ */

  always_ff @(posedge clock)
    if (!reset && req_valid && req_ready) begin
      assert (req_addr < `MEM_SIZE_IN_BYTES &&
              !(req_size == HALF && req_addr[0]) && !(req_size == WORD && req_addr[1:0] != 2'b00) &&
              req_size != DOUBLE)
      else $error("dcache: misaligned or out-of-range request %h (size %0d)", req_addr, req_size);
      // a load's id must not still be waiting in an MSHR (its response would be ambiguous)
      if (!req_store)
        for (int m = 0; m < NM; m++)
          for (int t = 0; t < NT; t++)
            if (mshr[m].valid && t < int'(mshr[m].ntg) && !tgts[m][t].store &&
                !(mshr[m].st == M_REPLAY && t < int'(rp)))
              assert (tgts[m][t].id != req_id)
              else $error("dcache: load id %0d issued while a load with that id is still waiting", req_id);
    end
  // synopsys translate_on

endmodule  // dcache
