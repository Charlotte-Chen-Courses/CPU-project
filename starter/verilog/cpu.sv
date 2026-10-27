/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  cpu.sv                                              //
//                                                                     //
//  Description :  Top level of the R10K-style core: integer ALU code, //
//                 branches and jumps, ending in WFI.                  //
//                                                                     //
//    fetch (I-cache) -> dispatch (decode, rename) -> ROB + RS         //
//    RS select -> PRF read -> ALU FU -> CDB arbiter -> CDB            //
//    CDB -> PRF write, RS wakeup, busy table, ROB done                //
//    ROB commit -> freelist free, retirement RAT, halt                //
//    ROB commit of a mispredict -> flush (recovery at commit)         //
//                                                                     //
//                 Fetch: P3's I-cache (8-byte blocks, blocking), both //
//                 caches on mem.sv through mem_arbiter. 2 slots per   //
//                 cycle when pc[2] == 0, else 1; PC advances by the   //
//                 slots fetched when dispatch fires; fetching stops   //
//                 after a WFI is dispatched.                          //
//                 Loads/stores: LSQ (lsq.sv) on the FU_LOAD/FU_STORE  //
//                 ports -> D-cache -> mem_arbiter. Stores write the   //
//                 D-cache after commit; stores_pending tells the      //
//                 testbench when they have all gone. A load that ran  //
//                 ahead of an overlapping older store is replayed     //
//                 (ROB flush at its own PC); with LSQ_SPECULATE = 2   //
//                 the load-wait table makes it wait next time.        //
//                 Issue: the PRF is read in the same cycle the RS     //
//                 selects (RS ready bits are flops, so a selected     //
//                 instruction's sources are already in the PRF), and  //
//                 the ALU computes in that cycle too; the ALU's       //
//                 result register is the only stage before the CDB.   //
//                 Known critical path: RS select -> PRF read -> ALU.  //
//                 Branches: fetch predicts with a BTB + bimodal       //
//                 predictor (branch_pred, trained at commit); the     //
//                 first slot predicted taken redirects fetch and the  //
//                 younger slots of the bundle are dropped. Returns    //
//                 take their target from the RAS (ras.sv), restored   //
//                 from its committed copy on a flush.                 //
//                 The branch unit computes the actual next PC; the    //
//                 ROB compares at commit. A mispredict commits, then  //
//                 flush in the same cycle: ROB/RS empty, FU results   //
//                 dropped, freelist rewound, RAT <- retirement map    //
//                 (with this cycle's commits), busy table cleared,    //
//                 fetch restarts at redirect_pc. No dispatch that     //
//                 cycle.                                              //
//                 Exceptions (illegal, no unit yet, misaligned        //
//                 target) are raised only at commit: the core stops   //
//                 with halted + exception + exception_pc. On a wrong  //
//                 path they are squashed like everything else.        //
//                 M extension: pipelined multiplier (mult_fu) and     //
//                 iterative divider (div_fu), each its own port.      //
//                 Not yet: FP, A, checkpoints.                        //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

module cpu #(
    localparam W         = `SUPERSCALAR_WIDTH,
    localparam TAG_W     = `TAG_W,
    localparam ROB_IDX_W = `ROB_IDX_W,
    localparam CDB_SZ    = `NUM_CDB,
    localparam NUM_FU    = `NUM_FU,
    localparam ISSUE_W   = `ISSUE_WIDTH,
    localparam NT        = `NUM_THREADS
) (
    input                               clock,
    input                               reset,

    // memory (P3 mem.sv protocol, shared by the I-cache and D-cache through mem_arbiter)
    output logic [1:0]                  proc2mem_command,
    output logic [`XLEN-1:0]            proc2mem_addr,
    output logic [63:0]                 proc2mem_data,
    input        [3:0]                  mem2proc_response,
    input        [63:0]                 mem2proc_data,
    input        [3:0]                  mem2proc_tag,

    // commit trace for the testbench
    output logic [W-1:0]                commit_valid,
    output logic [W-1:0]                commit_dest_valid,
    output logic [W-1:0][4:0]           commit_rd,
    output logic [W-1:0][TAG_W-1:0]     commit_tag,
    output logic [W-1:0][`XLEN-1:0]     commit_pc,
    output logic [`TID_W-1:0]           commit_tid,     // the thread committing this cycle
    output logic                        halted,         // every thread has committed a WFI or an exception
    output EXCEPTION                    exception,      // why thread 0 stopped (EXC_NONE = WFI; others: exception_t)
    output logic [`XLEN-1:0]            exception_pc,   // PC of thread 0's excepting instruction
    output logic                        stores_pending  // committed stores not yet in the D-cache
);

  // ---------------- fetch ----------------
  // One thread fetches per cycle (fetch_tid), chosen a cycle ahead (fetch_tid_next) so the I-cache
  // SRAM can be read at that thread's next PC.
  logic [NT-1:0][`XLEN-1:0] pc, pc_next;     // per thread; pc_next: the PC register's next value
  logic [NT-1:0]            fetch_stop;      // WFI dispatched: the thread fetches nothing more
  logic [`TID_W-1:0]        fetch_tid, fetch_tid_next;
  logic [`XLEN-1:0]         cur_pc;          // the fetching thread's PC
  logic                    fire;
  logic [W-1:0]            fetch_valid;
  logic [W-1:0][`XLEN-1:0] fetch_pc, fetch_pred_npc;
  logic [NT-1:0]           flush;          // mispredict / replay committed this cycle, per thread
  logic [NT-1:0][`XLEN-1:0] redirect_pc;

  logic [NT-1:0]           halted_t;       // the thread has committed a WFI or an exception

  // fetch policy, over the threads that can still fetch (no WFI dispatched, not halted; if none can,
  // the current one stays):
  //  FETCH_POLICY 0: round-robin;
  //  FETCH_POLICY 1: ICOUNT: fewest RS entries first, threads whose I-cache miss is in memory last
  //  (fetching one only waits); ties in round-robin order. (One thread: always thread 0.)
  logic [NT-1:0][$clog2(`RS_SZ+1)-1:0] rs_thread_count;
  logic [NT-1:0]                       imiss_waiting;
  always_comb begin
    automatic bit found = 0;
    automatic int best = 0;
    fetch_tid_next = fetch_tid;
    for (int k = 1; k <= NT; k++) begin        // round-robin order: the nearest one after fetch_tid first
      automatic int t = (int'(fetch_tid) + k) % NT;
      automatic int score = (`FETCH_POLICY == 0) ? 0 : (int'(imiss_waiting[t]) << 8) + int'(rs_thread_count[t]);
      if (!fetch_stop[t] && !halted_t[t] && (!found || score < best)) begin
        found = 1; best = score; fetch_tid_next = `TID_W'(t);
      end
    end
  end
  assign cur_pc = pc[fetch_tid];

  logic [W-1:0]            pred_taken, btb_hit, pred_call, pred_ret;
  logic [`XLEN-1:0]        ras_top;
  logic [NT-1:0][`XLEN-1:0] ras_top_t;
  logic                    ras_push, ras_pop;
  logic [`XLEN-1:0]        ras_push_addr;
  logic [W-1:0][`XLEN-1:0] pred_target;
  logic [`XLEN-1:0]        fetch_next_pc;  // where fetch goes after this bundle

  // ---- I-cache: one 8-byte block = 2 instructions per cycle ----
  // A 2-wide bundle only fits when pc[2] == 0; otherwise the block holds just slot 0 and
  // fetch continues at pc + 4. A PC outside memory is never sent to the I-cache (mem.sv
  // would never answer); slot 0 gets EXC_ACCESS instead, raised only if it commits.
  logic             pc_in_range, icache_valid;
  logic [63:0]      icache_block;
  INST  [W-1:0]     fetch_inst;
  EXCEPTION [W-1:0] fetch_exc;
  logic [1:0]       i_command, d_command;
  logic [`XLEN-1:0] i_addr, d_addr;
  logic [63:0]      d_data, mem_rdata;
  logic [3:0]       i_response, i_tag, d_response, d_tag;
  // instruction stream buffers (one per thread; the fetching thread's talks to the I-cache)
  logic                     pf_ready, pf_wait, pf_take;
  logic [63:0]              pf_data;
  logic [NT-1:0]            pf_ready_t, pf_wait_t, probe_hit;
  logic [NT-1:0][63:0]      pf_data_t;
  logic [NT-1:0][`XLEN-1:0] probe_addr, p_addr_t;
  logic [NT-1:0][1:0]       p_command_t;
  logic [NT-1:0][3:0]       p_response_t;
  logic [`XLEN-1:0]         p_addr;
  logic [1:0]               p_command;
  logic [3:0]               p_response;
  logic [`TID_W-1:0]        p_sel;

  // base + bound: a thread's virtual address a is physical t*THREAD_MEM_SZ + a, except in the shared
  // window (a >= PRIV_MEM_SZ, several threads only), which is the same physical address for all
  function automatic logic [`XLEN-1:0] phys(input logic [`TID_W-1:0] t, input logic [`XLEN-1:0] a);
    if (a >= `XLEN'(`PRIV_MEM_SZ)) return a;
    return a + `XLEN'(int'(t) * `THREAD_MEM_SZ);
  endfunction
  logic [`XLEN-1:0] fetch_paddr;      // physical block address the I-cache is looking up
  assign pc_in_range = (cur_pc < `THREAD_MEM_SZ);
  assign fetch_paddr = pc_in_range ? phys(fetch_tid, cur_pc) : '0;

  icache icache_0 (
      .clock             (clock),
      .reset             (reset),
      .fetch_tid         (fetch_tid),
      .Imem2proc_response(i_response),
      .Imem2proc_data    (mem_rdata),
      .Imem2proc_tag     (i_tag),
      .proc2Icache_addr  (fetch_paddr),
      .next_addr         (pc_next[fetch_tid_next] < `THREAD_MEM_SZ ?                  // (the SRAM reads it a cycle early)
                          phys(fetch_tid_next, pc_next[fetch_tid_next]) : '0),
      .proc2Imem_command (i_command),
      .proc2Imem_addr    (i_addr),
      .Icache_data_out   (icache_block),
      .Icache_valid_out  (icache_valid),
      .pf_ready          (pf_ready),
      .pf_wait           (pf_wait),
      .pf_data           (pf_data),
      .pf_take           (pf_take),
      .probe_addr        (probe_addr),
      .probe_hit         (probe_hit),
      .miss_waiting      (imiss_waiting)
  );

  for (genvar t = 0; t < NT; t++) begin : gen_ipf
    iprefetch iprefetch_0 (
        .clock        (clock),
        .reset        (reset),
        .cur_addr     (fetch_paddr),
        .pf_ready     (pf_ready_t[t]),
        .pf_wait      (pf_wait_t[t]),
        .pf_data      (pf_data_t[t]),
        .pf_take      (pf_take && fetch_tid == `TID_W'(t)),
        .miss_accepted(i_command != BUS_NONE && i_response != 4'd0 && fetch_tid == `TID_W'(t)),
        .miss_addr    (i_addr),
        .probe_addr   (probe_addr[t]),
        .probe_hit    (probe_hit[t]),
        .p_command    (p_command_t[t]),
        .p_addr       (p_addr_t[t]),
        .p_response   (p_response_t[t]),
        .i_tag        (i_tag),
        .rdata        (mem_rdata)
    );
    assign p_response_t[t] = (p_sel == `TID_W'(t)) ? p_response : 4'd0;
  end
  // only the fetching thread's buffer answers the I-cache's lookup
  assign pf_ready = pf_ready_t[fetch_tid];
  assign pf_wait  = pf_wait_t[fetch_tid];
  assign pf_data  = pf_data_t[fetch_tid];
  // one prefetch command at a time: the lowest thread that has one
  always_comb begin
    p_sel = '0;
    for (int t = NT - 1; t >= 0; t--) if (p_command_t[t] != BUS_NONE) p_sel = `TID_W'(t);
  end
  assign p_command = p_command_t[p_sel];
  assign p_addr    = p_addr_t[p_sel];

  // the first slot predicted taken redirects fetch; younger slots in the bundle are dropped
  // a predicted return takes its target from the RAS; a predicted call pushes its PC+4.
  // Both are taken jumps, so they end the bundle: at most one RAS op per cycle.
  always_comb begin
    automatic bit redirected = `FALSE;
    automatic int n_avail = pc_in_range ? (cur_pc[2] ? 1 : 2) : 1;   // slots this block can give
    if (n_avail > W) n_avail = W;
    fetch_next_pc = cur_pc + `XLEN'(4 * n_avail);
    ras_push      = `FALSE;
    ras_pop       = `FALSE;
    ras_push_addr = '0;
    for (int i = 0; i < W; i++) begin
      fetch_pc[i]       = cur_pc + `XLEN'(4 * i);
      fetch_inst[i]     = (int'(cur_pc[2]) + i < 2) ? (cur_pc[2] + i == 1 ? icache_block[63:32] : icache_block[31:0])
                                                : `NOP;
      fetch_exc[i]      = EXC_NONE;
      if (!pc_in_range) begin
        fetch_inst[i] = `NOP;
        fetch_exc[i]  = EXC_ACCESS;
      end
      fetch_valid[i]    = !fetch_stop[fetch_tid] && !redirected && i < n_avail && (icache_valid || !pc_in_range);
      fetch_pred_npc[i] = !pred_taken[i] ? fetch_pc[i] + 4 :
                           pred_ret[i]   ? ras_top        : pred_target[i];
      if (fetch_valid[i] && pred_taken[i]) begin
        redirected    = `TRUE;
        fetch_next_pc = fetch_pred_npc[i];
        ras_push      = pred_call[i];
        ras_pop       = pred_ret[i];
        ras_push_addr = fetch_pc[i] + 4;
      end
      // a CSR op or an atomic ends the bundle (serial: nothing younger may dispatch with it);
      // fetch goes on at its pc + 4
      if (fetch_valid[i] && ((fetch_inst[i][6:0] == 7'b1110011 && fetch_inst[i][14:12] != 3'b000) ||
                             fetch_inst[i][6:0] == 7'b0101111)) begin
        redirected     = `TRUE;
        fetch_next_pc  = fetch_pc[i] + 4;
        fetch_pred_npc[i] = fetch_pc[i] + 4;
        ras_push       = `FALSE;
        ras_pop        = `FALSE;
      end
    end
  end

  // ---- commit: every thread's ROB; one thread commits per cycle (commit_tid) ----
  logic [NT-1:0][W-1:0]              commit_valid_t, commit_dest_valid_t, commit_is_br_t, commit_uncond_t,
                                     commit_is_call_t, commit_is_ret_t, commit_is_load_t, commit_is_store_t,
                                     commit_halt_t, commit_dest_fp_t;
  logic [NT-1:0][W-1:0][4:0]         commit_rd_t, commit_fflags_t;
  logic [NT-1:0][W-1:0][TAG_W-1:0]   commit_tag_t, commit_told_t;
  logic [NT-1:0][W-1:0][`XLEN-1:0]   commit_pc_t, commit_next_pc_t;
  EXCEPTION [NT-1:0][W-1:0]          commit_exception_t;

  // commit policy: round-robin over the threads whose oldest instruction is ready (commit_rr: the
  // last one that got a turn)
  logic [NT-1:0] rob_head_ready_t;
  if (NT == 1) begin : gen_commit_one
    assign commit_tid = '0;
  end else begin : gen_commit_rr
    logic [`TID_W-1:0] commit_rr;
    always_comb begin
      commit_tid = commit_rr;
      for (int k = NT; k >= 1; k--) begin      // ends on the nearest ready one after commit_rr
        automatic int t = (int'(commit_rr) + k) % NT;
        if (rob_head_ready_t[t] && !halted_t[t]) commit_tid = `TID_W'(t);
      end
    end
    always_ff @(posedge clock)
      if (reset) commit_rr <= '0;
      else       commit_rr <= commit_tid;
  end

  logic [W-1:0]            commit_is_br, commit_uncond, commit_is_call, commit_is_ret;
  logic [W-1:0][`XLEN-1:0] commit_next_pc;
  logic [W-1:0]            commit_taken;

  // The threads' PCs overlap (each starts at 0), so the shared PC-indexed tables see a thread-hashed
  // PC: the thread XORed into the top table-index bit (spreading the threads over the table) and into
  // bit 31, which a PC never sets (so it is part of the BTB tag: no thread follows another's target).
  // One thread: the PC itself.
  function automatic logic [`XLEN-1:0] thread_pc(input logic [`TID_W-1:0] t, input logic [`XLEN-1:0] pc,
                                                 input int idx_top);
    if (NT == 1) return pc;
    return pc ^ (`XLEN'(t) << idx_top) ^ (`XLEN'(t) << (`XLEN - `TID_W));
  endfunction
  logic [W-1:0][`XLEN-1:0] bp_lookup_pc, bp_upd_pc;
  always_comb
    for (int i = 0; i < W; i++) begin
      bp_lookup_pc[i] = thread_pc(fetch_tid, fetch_pc[i], $clog2(`BHT_SZ) + 1);
      bp_upd_pc[i]    = thread_pc(commit_tid, commit_pc[i], $clog2(`BHT_SZ) + 1);
    end
  always_comb
    for (int i = 0; i < W; i++) commit_taken[i] = (commit_next_pc[i] != commit_pc[i] + 4);

  // ---- global history (tournament predictor), per thread ----
  // A slot is predicted with the fetching thread's speculative history shifted by the conditional
  // branches in older slots of the bundle (as not taken: a later slot exists only then; predecoded,
  // so the history does not depend on the predictions). The history goes into the ROB with the
  // instruction and trains with it at commit. The speculative history takes the bundle's predicted
  // outcomes when it dispatches; the committed one the actual outcomes; a flush copies the committed
  // one (after this cycle's commits) back, exactly the history before the re-fetched path.
  localparam H_W = `GHR_W;
  logic [NT-1:0][H_W-1:0]       spec_ghr, commit_ghr, commit_ghr_next;
  logic [W-1:0][H_W-1:0]        slot_hist, commit_hist;
  logic [NT-1:0][W-1:0][H_W-1:0] commit_hist_t;
  logic [W-1:0]                 slot_cond;
  always_comb begin
    automatic logic [H_W-1:0] h = spec_ghr[fetch_tid];
    for (int i = 0; i < W; i++) begin
      slot_cond[i] = (fetch_inst[i][6:0] == 7'b1100011);
      slot_hist[i] = h;
      if (slot_cond[i]) h = {h[H_W-2:0], 1'b0};
    end
  end
  always_comb
    for (int t = 0; t < NT; t++) begin
      automatic logic [H_W-1:0] h = commit_ghr[t];
      for (int i = 0; i < W; i++)
        if (commit_is_br_t[t][i] && !commit_uncond_t[t][i])
          h = {h[H_W-2:0], commit_next_pc_t[t][i] != commit_pc_t[t][i] + 4};
      commit_ghr_next[t] = h;
    end
  always_ff @(posedge clock)
    for (int t = 0; t < NT; t++)
      if (reset) begin
        spec_ghr[t]   <= '0;
        commit_ghr[t] <= '0;
      end else begin
        commit_ghr[t] <= commit_ghr_next[t];
        if (flush[t]) spec_ghr[t] <= commit_ghr_next[t];
        else if (fire && fetch_tid == `TID_W'(t)) begin
          automatic logic [H_W-1:0] h = spec_ghr[t];
          for (int i = 0; i < W; i++) if (fetch_valid[i] && slot_cond[i]) h = {h[H_W-2:0], pred_taken[i]};
          spec_ghr[t] <= h;
        end
      end
  assign commit_hist = commit_hist_t[commit_tid];

  // synopsys translate_off
  // a committing conditional branch was predicted with exactly the committed history before it (its
  // older branches' predictions were right, or it would have been squashed): the speculative history
  // is kept and restored correctly
  always_ff @(posedge clock)
    if (!reset)
      for (int t = 0; t < NT; t++) begin
        automatic logic [H_W-1:0] h = commit_ghr[t];
        for (int i = 0; i < W; i++)
          if (commit_is_br_t[t][i] && !commit_uncond_t[t][i]) begin
            assert (commit_hist_t[t][i] == h)
            else $error("cpu: thread %0d branch at %h was predicted with history %h, committed history is %h",
                        t, commit_pc_t[t][i], commit_hist_t[t][i], h);
            h = {h[H_W-2:0], commit_next_pc_t[t][i] != commit_pc_t[t][i] + 4};
          end
      end
  // synopsys translate_on

  branch_pred branch_pred_0 (
      .clock      (clock),
      .reset      (reset),
      .lookup_pc  (bp_lookup_pc),
      .lookup_hist(slot_hist),
      .pred_taken (pred_taken),
      .pred_target(pred_target),
      .btb_hit    (btb_hit),
      .pred_call  (pred_call),
      .pred_ret   (pred_ret),
      .upd_valid  (commit_is_br),
      .upd_pc     (bp_upd_pc),
      .upd_uncond (commit_uncond),
      .upd_call   (commit_is_call),
      .upd_ret    (commit_is_ret),
      .upd_taken  (commit_taken),
      .upd_target (commit_next_pc),
      .upd_hist   (commit_hist)
  );

  // one RAS per thread: the fetching thread's predicts and is pushed/popped; each learns from its
  // own commits and recovers on its own flush
  for (genvar t = 0; t < NT; t++) begin : gen_ras
    ras ras_0 (
        .clock         (clock),
        .reset         (reset),
        .spec_push     (ras_push && fire && fetch_tid == `TID_W'(t)),   // only for a bundle that is dispatched
        .spec_push_addr(ras_push_addr),
        .spec_pop      (ras_pop && fire && fetch_tid == `TID_W'(t)),
        .top           (ras_top_t[t]),
        .commit_call   (commit_is_call_t[t]),
        .commit_ret    (commit_is_ret_t[t]),
        .commit_pc     (commit_pc_t[t]),
        .flush         (flush[t])
    );
  end
  assign ras_top = ras_top_t[fetch_tid];

  // ---------------- dispatch ----------------
  localparam LD_PORT = `NUM_FU_ALU + `NUM_FU_BRANCH + `NUM_FU_MULT + `NUM_FU_DIV;   // port_type() order
  localparam ST_PORT = LD_PORT + `NUM_FU_LOAD;
  logic                         lq_space_ok, sq_space_ok;
  logic [W-1:0][`LSQ_IDX_W-1:0] disp_lsq_idx;
  logic                        rob_space_ok, rs_space_ok;
  logic [NT-1:0]               rob_space_ok_t, rob_empty_t, lq_space_ok_t, sq_space_ok_t;
  logic [NT-1:0][W-1:0][ROB_IDX_W-1:0]  rob_disp_idx_t;
  logic [NT-1:0][W-1:0][`LSQ_IDX_W-1:0] disp_lsq_idx_t;
  logic [NT-1:0][ROB_IDX_W-1:0] rob_head_t;
  logic [W-1:0]                disp_valid, disp_dest_valid, disp_halt, rs_disp_valid;
  logic [W-1:0]                disp_is_br, disp_uncond, disp_is_call, disp_is_ret;
  logic [W-1:0]                disp_is_load, disp_is_store;
  EXCEPTION [W-1:0]            disp_exception, commit_exception;
  logic [W-1:0][4:0]           disp_rd;
  logic [W-1:0][TAG_W-1:0]     disp_tag, disp_told;
  logic [W-1:0][`XLEN-1:0]     disp_pc, disp_pred_npc;
  RS_PACKET [W-1:0]            disp_pkt;

  logic [CDB_SZ-1:0]           cdb_valid;
  CDB_PACKET [CDB_SZ-1:0]      cdb_pkt;
  logic [CDB_SZ-1:0][TAG_W-1:0]     cdb_tag;
  logic [CDB_SZ-1:0][ROB_IDX_W-1:0] cdb_rob_idx;
  logic [CDB_SZ-1:0][`XLEN-1:0]     cdb_value, cdb_next_pc;
  logic [CDB_SZ-1:0]                cdb_branch;
  EXCEPTION [CDB_SZ-1:0]            cdb_exception;
  logic [CDB_SZ-1:0][4:0]           cdb_fflags;
  logic [W-1:0]                     disp_dest_fp, commit_dest_fp;
  logic [NT-1:0][`ARCH_REG_SZ-1:0][TAG_W-1:0] retire_map, retire_map_next;         // x registers, per thread
  logic [NT-1:0][`ARCH_REG_SZ-1:0][TAG_W-1:0] fp_retire_map, fp_retire_map_next;   // f registers, per thread

  logic [W-1:0]                commit_halt;
  logic [W-1:0][TAG_W-1:0]     commit_told;

  always_comb
    for (int k = 0; k < CDB_SZ; k++) begin
      cdb_tag[k]     = cdb_pkt[k].tag;
      cdb_rob_idx[k] = cdb_pkt[k].rob_idx;
      cdb_value[k]   = cdb_pkt[k].value;
      cdb_branch[k]  = cdb_pkt[k].branch;
      cdb_next_pc[k] = cdb_pkt[k].next_pc;
      cdb_exception[k] = cdb_pkt[k].exception;
      cdb_fflags[k]    = cdb_pkt[k].fflags;
    end

  dispatch dispatch_0 (
      .clock          (clock),
      .reset          (reset),
      .in_valid       (fetch_valid),
      .in_inst        (fetch_inst),
      .in_pc          (fetch_pc),
      .in_exc         (fetch_exc),
      .in_pred_npc    (fetch_pred_npc),
      .in_tid         (fetch_tid),
      .fire           (fire),
      .rob_space_ok   (rob_space_ok),
      .rob_empty      (rob_empty_t),
      .rs_space_ok    (rs_space_ok),
      .lq_space_ok    (lq_space_ok),
      .sq_space_ok    (sq_space_ok),
      .lsq_idx        (disp_lsq_idx),
      .rob_idx        (rob_disp_idx_t[fetch_tid]),
      .disp_valid     (disp_valid),
      .disp_dest_valid(disp_dest_valid),
      .disp_rd        (disp_rd),
      .disp_tag       (disp_tag),
      .disp_told      (disp_told),
      .disp_pc        (disp_pc),
      .disp_halt      (disp_halt),
      .disp_pred_npc  (disp_pred_npc),
      .disp_exception (disp_exception),
      .disp_is_br     (disp_is_br),
      .disp_uncond    (disp_uncond),
      .disp_is_call   (disp_is_call),
      .disp_is_ret    (disp_is_ret),
      .disp_is_load   (disp_is_load),
      .disp_is_store  (disp_is_store),
      .disp_dest_fp   (disp_dest_fp),
      .rs_disp_valid  (rs_disp_valid),
      .disp_pkt       (disp_pkt),
      .cdb_valid      (cdb_valid),
      .cdb_tag        (cdb_tag),
      .free_en        (commit_valid & commit_dest_valid),
      .free_fp        (commit_dest_fp),
      .free_tag       (commit_told),
      .free_tid       (commit_tid),
      .flush          (flush),
      .recover_map    (retire_map_next),
      .fp_recover_map (fp_retire_map_next)
  );

  // the fetching thread's (and only its) state: rob_space_ok etc. for dispatch
  assign rob_space_ok = rob_space_ok_t[fetch_tid];
  assign lq_space_ok  = lq_space_ok_t[fetch_tid];
  assign sq_space_ok  = sq_space_ok_t[fetch_tid];
  assign disp_lsq_idx = disp_lsq_idx_t[fetch_tid];

  always_comb
    for (int t = 0; t < NT; t++)
      if (reset)                                 pc_next[t] = '0;
      else if (flush[t])                         pc_next[t] = redirect_pc[t];
      else if (fire && fetch_tid == `TID_W'(t))  pc_next[t] = fetch_next_pc;
      else                                       pc_next[t] = pc[t];

  always_ff @(posedge clock) begin
    pc        <= pc_next;
    fetch_tid <= reset ? '0 : fetch_tid_next;
    for (int t = 0; t < NT; t++)
      if (reset || flush[t]) fetch_stop[t] <= `FALSE;   // (a WFI fetched on a wrong path may have set it)
      else if (fire && fetch_tid == `TID_W'(t) && |(disp_valid & disp_halt)) fetch_stop[t] <= `TRUE;
  end

  // ---------------- ROB, retirement maps, halt: per thread ----------------
  logic [NT-1:0]                  viol_valid, replay_flush_t;   // LSQ memory-order violation -> replay at commit
  logic [NT-1:0][`ROB_IDX_W-1:0]  viol_rob_idx;
  logic [NT-1:0]                  snoop_viol_valid_t;           // another thread's store hit a load's word
  logic [NT-1:0][`ROB_IDX_W-1:0]  snoop_viol_rob_idx;
  logic                           replay_flush;                 // (any thread: statistics)
  EXCEPTION [NT-1:0]              exception_t;
  logic [NT-1:0][`XLEN-1:0]       exception_pc_t;

  for (genvar t = 0; t < NT; t++) begin : gen_rob
    logic [CDB_SZ-1:0] cdb_mine;     // CDB results of this thread
    always_comb
      for (int k = 0; k < CDB_SZ; k++) cdb_mine[k] = cdb_valid[k] && cdb_pkt[k].tid == `TID_W'(t);

    rob rob_0 (
        .clock            (clock),
        .reset            (reset),
        .disp_valid       (disp_valid & {W{fetch_tid == `TID_W'(t)}}),
        .disp_dest_valid  (disp_dest_valid),
        .disp_rd          (disp_rd),
        .disp_tag         (disp_tag),
        .disp_told        (disp_told),
        .disp_pc          (disp_pc),
        .disp_halt        (disp_halt),
        .disp_pred_npc    (disp_pred_npc),
        .disp_hist        (slot_hist),
        .disp_exception   (disp_exception),
        .disp_is_br       (disp_is_br),
        .disp_uncond      (disp_uncond),
        .disp_is_call     (disp_is_call),
        .disp_is_ret      (disp_is_ret),
        .disp_is_load     (disp_is_load),
        .disp_is_store    (disp_is_store),
        .disp_dest_fp     (disp_dest_fp),
        .replay_valid     (viol_valid[t]),
        .replay_rob_idx   (viol_rob_idx[t]),
        .replay2_valid    (snoop_viol_valid_t[t]),
        .replay2_rob_idx  (snoop_viol_rob_idx[t]),
        .disp_idx         (rob_disp_idx_t[t]),
        .space_ok         (rob_space_ok_t[t]),
        .cdb_valid        (cdb_mine),
        .cdb_rob_idx      (cdb_rob_idx),
        .cdb_branch       (cdb_branch),
        .cdb_next_pc      (cdb_next_pc),
        .cdb_exception    (cdb_exception),
        .cdb_fflags       (cdb_fflags),
        // younger entries behind an exception never commit; other threads wait for their turn
        // (a ROB that may not commit raises no flush either)
        .commit_stop      (halted_t[t] || commit_tid != `TID_W'(t)),
        .commit_valid     (commit_valid_t[t]),
        .commit_dest_valid(commit_dest_valid_t[t]),
        .commit_rd        (commit_rd_t[t]),
        .commit_tag       (commit_tag_t[t]),
        .commit_told      (commit_told_t[t]),
        .commit_pc        (commit_pc_t[t]),
        .commit_is_br     (commit_is_br_t[t]),
        .commit_uncond    (commit_uncond_t[t]),
        .commit_is_call   (commit_is_call_t[t]),
        .commit_is_ret    (commit_is_ret_t[t]),
        .commit_is_load   (commit_is_load_t[t]),
        .commit_is_store  (commit_is_store_t[t]),
        .commit_next_pc   (commit_next_pc_t[t]),
        .commit_hist      (commit_hist_t[t]),
        .commit_halt      (commit_halt_t[t]),
        .commit_exception (commit_exception_t[t]),
        .flush            (flush[t]),
        .redirect_pc      (redirect_pc[t]),
        .replay_flush     (replay_flush_t[t]),
        .commit_dest_fp   (commit_dest_fp_t[t]),
        .commit_fflags    (commit_fflags_t[t]),
        .head             (rob_head_t[t]),
        .head_ready       (rob_head_ready_t[t]),
        .empty            (rob_empty_t[t])
    );

    retire_rat #(.BASE(t * `THREAD_PHYS_SZ)) retire_rat_0 (
        .clock     (clock),
        .reset     (reset),
        .commit_en (commit_valid_t[t] & commit_dest_valid_t[t] & ~commit_dest_fp_t[t]),
        .commit_rd (commit_rd_t[t]),
        .commit_tag(commit_tag_t[t]),
        .map       (retire_map[t]),          // architectural xN = PRF[retire_map[t][N]]
        .map_next  (retire_map_next[t])      // after this cycle's commits: for recovery
    );

    retire_rat #(.BASE(`FP_TAG_BASE + t * `FP_THREAD_PHYS_SZ), .ZERO_REG(1'b0)) fp_retire_rat_0 (
        .clock     (clock),
        .reset     (reset),
        .commit_en (commit_valid_t[t] & commit_dest_valid_t[t] & commit_dest_fp_t[t]),
        .commit_rd (commit_rd_t[t]),
        .commit_tag(commit_tag_t[t]),
        .map       (fp_retire_map[t]),       // architectural fN = FP PRF[fp_retire_map[t][N]]
        .map_next  (fp_retire_map_next[t])
    );

    always_ff @(posedge clock)
      if (reset) begin
        halted_t[t]       <= `FALSE;
        exception_t[t]    <= EXC_NONE;
        exception_pc_t[t] <= '0;
      end else if (!halted_t[t]) begin
        if (|commit_halt_t[t]) halted_t[t] <= `TRUE;
        for (int i = 0; i < W; i++)
          if (commit_exception_t[t][i] != EXC_NONE) begin
            halted_t[t]       <= `TRUE;
            exception_t[t]    <= commit_exception_t[t][i];
            exception_pc_t[t] <= commit_pc_t[t][i];
          end
      end
  end

  assign halted       = &halted_t;
  assign exception    = exception_t[0];
  assign exception_pc = exception_pc_t[0];
  assign replay_flush = |replay_flush_t;

  // the committing thread's outputs (the others commit nothing this cycle)
  assign commit_valid      = commit_valid_t[commit_tid];
  assign commit_dest_valid = commit_dest_valid_t[commit_tid];
  assign commit_rd         = commit_rd_t[commit_tid];
  assign commit_tag        = commit_tag_t[commit_tid];
  assign commit_told       = commit_told_t[commit_tid];
  assign commit_pc         = commit_pc_t[commit_tid];
  assign commit_is_br      = commit_is_br_t[commit_tid];
  assign commit_uncond     = commit_uncond_t[commit_tid];
  assign commit_is_call    = commit_is_call_t[commit_tid];
  assign commit_is_ret     = commit_is_ret_t[commit_tid];
  assign commit_next_pc    = commit_next_pc_t[commit_tid];
  assign commit_halt       = commit_halt_t[commit_tid];
  assign commit_exception  = commit_exception_t[commit_tid];
  assign commit_dest_fp    = commit_dest_fp_t[commit_tid];

  // ---------------- RS ----------------
  logic [NUM_FU-1:0]  fu_ready, issue_valid;
  RS_PACKET [NUM_FU-1:0] issue_pkt;

  rs rs_0 (
      .clock      (clock),
      .reset      (reset),
      .disp_valid (rs_disp_valid),
      .disp_pkt   (disp_pkt),
      .space_ok   (rs_space_ok),
      .cdb_valid  (cdb_valid),
      .cdb_tag    (cdb_tag),
      .flush      (flush),
      .rob_head   (rob_head_t),
      .fu_ready   (fu_ready),
      .issue_valid(issue_valid),
      .issue_pkt  (issue_pkt),
      .thread_count(rs_thread_count)
  );

  // ---------------- PRF read: pack issued ports onto the ISSUE_W read-port pairs ----------------
  // Both register files see every read port and every CDB write; a tag belongs to exactly one
  // of them (FP tags start at FP_TAG_BASE). The FP file has one more read port: rs3 of the FP
  // unit (fused multiply-add). Simple, but it gives the FP file as many read ports as the
  // integer one (a Phase 6 area / timing item).
  localparam FP_PORT = `NUM_FU_ALU + `NUM_FU_BRANCH + `NUM_FU_MULT + `NUM_FU_DIV + `NUM_FU_LOAD + `NUM_FU_STORE;
  logic [2*ISSUE_W-1:0][TAG_W-1:0]   prf_rd_tag;
  logic [2*ISSUE_W-1:0][`XLEN-1:0]   prf_rd_data, int_rd_data;
  logic [2*ISSUE_W:0][TAG_W-1:0]     fp_rd_tag;
  logic [2*ISSUE_W:0][`XLEN-1:0]     fp_rd_data;
  logic [NUM_FU-1:0][$clog2(ISSUE_W+1)-1:0] port_slot;
  logic [NUM_FU-1:0][`XLEN-1:0]      rs1_value, rs2_value;
  logic [`XLEN-1:0]                  rs3_value;          // FP unit only
  int n_slot;

  always_comb begin
    prf_rd_tag = '0;
    port_slot  = '0;
    n_slot     = 0;
    for (int f = 0; f < NUM_FU; f++)
      if (issue_valid[f] && n_slot < ISSUE_W) begin   // RS issues at most ISSUE_W
        port_slot[f]              = $bits(port_slot[f])'(n_slot);
        prf_rd_tag[2*n_slot]      = issue_pkt[f].src1_tag;
        prf_rd_tag[2*n_slot + 1]  = issue_pkt[f].src2_tag;
        n_slot++;
      end
  end

  // (separate block: reading the PRF data back in the block that drives the tags
  //  would look like a combinational loop)
  always_comb begin
    for (int r = 0; r < 2*ISSUE_W; r++)
      prf_rd_data[r] = (int'(prf_rd_tag[r]) >= `FP_TAG_BASE) ? fp_rd_data[r] : int_rd_data[r];
    for (int f = 0; f < NUM_FU; f++) begin
      rs1_value[f] = prf_rd_data[2*port_slot[f]];
      rs2_value[f] = prf_rd_data[2*port_slot[f] + 1];
    end
    rs3_value = fp_rd_data[2*ISSUE_W];
  end

  always_comb begin
    fp_rd_tag[2*ISSUE_W-1:0] = prf_rd_tag;
    fp_rd_tag[2*ISSUE_W]     = (`NUM_FU_FP > 0) ? issue_pkt[FP_PORT < NUM_FU ? FP_PORT : 0].src3_tag : '0;
  end

  prf prf_0 (
      .clock  (clock),
      .rd_tag (prf_rd_tag),
      .rd_data(int_rd_data),
      .wr_en  (cdb_valid),
      .wr_tag (cdb_tag),
      .wr_data(cdb_value)
  );

  prf #(.RD_SZ(2*ISSUE_W + 1), .BASE(`FP_TAG_BASE), .SZ(`FP_PHYS_REG_SZ), .ZERO_REG(1'b0)) fp_prf_0 (
      .clock  (clock),
      .rd_tag (fp_rd_tag),
      .rd_data(fp_rd_data),
      .wr_en  (cdb_valid),
      .wr_tag (cdb_tag),
      .wr_data(cdb_value)
  );

  // fcsr = {frm, fflags}. fflags accumulate the flags of instructions that commit without an
  // exception (wrong-path flags never count); CSR ops on fflags/frm/fcsr write it from the FP
  // unit when they issue (serial: nothing older is left, nothing younger exists yet).
  // One fcsr per thread; the FP unit sees (and a CSR op writes) the issuing op's thread's.
  logic [NT-1:0][7:0] fcsr;
  logic [7:0]         fcsr_wdata;
  logic               fcsr_we;
  logic [`TID_W-1:0]  fp_tid;
  assign fp_tid = issue_pkt[FP_PORT < NUM_FU ? FP_PORT : 0].tid;
  always_ff @(posedge clock)
    for (int t = 0; t < NT; t++) begin
      automatic logic [4:0] acc = '0;
      for (int i = 0; i < W; i++) acc |= commit_fflags_t[t][i];
      if (reset)                                  fcsr[t] <= '0;
      else if (fcsr_we && fp_tid == `TID_W'(t))   fcsr[t] <= fcsr_wdata;
      else                                        fcsr[t][4:0] <= fcsr[t][4:0] | acc;
    end
  if (`NUM_FU_FP == 0) begin : gen_no_fcsr_write
    assign fcsr_we    = `FALSE;
    assign fcsr_wdata = '0;
  end

  // ---------------- functional units ----------------
  genvar g;
  logic [NUM_FU-1:0]      fu_out_valid, fu_grant;
  CDB_PACKET [NUM_FU-1:0] fu_out_pkt;

  for (g = 0; g < NUM_FU; g++) begin : gen_fu
    if (g < `NUM_FU_ALU) begin : gen_alu
      alu_fu alu_fu_0 (
          .clock    (clock),
          .reset    (reset),
          .in_valid (issue_valid[g]),
          .in_pkt   (issue_pkt[g]),
          .ready    (fu_ready[g]),
          .out_valid(fu_out_valid[g]),
          .out_pkt  (fu_out_pkt[g]),
          .out_grant(fu_grant[g]),
          .flush    (flush),
          .rs1_value(rs1_value[g]),
          .rs2_value(rs2_value[g])
      );
    end else if (g < `NUM_FU_ALU + `NUM_FU_BRANCH) begin : gen_branch
      branch_fu branch_fu_0 (
          .clock    (clock),
          .reset    (reset),
          .in_valid (issue_valid[g]),
          .in_pkt   (issue_pkt[g]),
          .ready    (fu_ready[g]),
          .out_valid(fu_out_valid[g]),
          .out_pkt  (fu_out_pkt[g]),
          .out_grant(fu_grant[g]),
          .flush    (flush),
          .rs1_value(rs1_value[g]),
          .rs2_value(rs2_value[g])
      );
    end else if (g < `NUM_FU_ALU + `NUM_FU_BRANCH + `NUM_FU_MULT) begin : gen_mult
      mult_fu mult_fu_0 (
          .clock    (clock),
          .reset    (reset),
          .in_valid (issue_valid[g]),
          .in_pkt   (issue_pkt[g]),
          .ready    (fu_ready[g]),
          .out_valid(fu_out_valid[g]),
          .out_pkt  (fu_out_pkt[g]),
          .out_grant(fu_grant[g]),
          .flush    (flush),
          .rs1_value(rs1_value[g]),
          .rs2_value(rs2_value[g])
      );
    end else if (g < `NUM_FU_ALU + `NUM_FU_BRANCH + `NUM_FU_MULT + `NUM_FU_DIV) begin : gen_div
      div_fu div_fu_0 (
          .clock    (clock),
          .reset    (reset),
          .in_valid (issue_valid[g]),
          .in_pkt   (issue_pkt[g]),
          .ready    (fu_ready[g]),
          .out_valid(fu_out_valid[g]),
          .out_pkt  (fu_out_pkt[g]),
          .out_grant(fu_grant[g]),
          .flush    (flush),
          .rs1_value(rs1_value[g]),
          .rs2_value(rs2_value[g])
      );
    end else if (g == LD_PORT || g == ST_PORT) begin : gen_lsq
      // load / store ports belong to the LSQ (connected below)
    end else if (g == FP_PORT) begin : gen_fp
      fp_fu fp_fu_0 (
          .clock    (clock),
          .reset    (reset),
          .in_valid (issue_valid[g]),
          .in_pkt   (issue_pkt[g]),
          .ready    (fu_ready[g]),
          .out_valid(fu_out_valid[g]),
          .out_pkt  (fu_out_pkt[g]),
          .out_grant(fu_grant[g]),
          .flush    (flush),
          .rs1_value(rs1_value[g]),
          .rs2_value(rs2_value[g]),
          .rs3_value(rs3_value),
          .fcsr     (fcsr[fp_tid]),
          .fcsr_we  (fcsr_we),
          .fcsr_wdata(fcsr_wdata)
      );
    end else begin : gen_none
      // FU type not implemented yet: never ready, never produces a result
      assign fu_ready[g]     = `FALSE;
      assign fu_out_valid[g] = `FALSE;
      assign fu_out_pkt[g]   = '0;
    end
  end

  // ---------------- load/store queue ----------------
  // (LD_PORT / ST_PORT and the dispatch-side LSQ signals are declared with dispatch, above)
  logic                         dc_req_valid, dc_req_ready, dc_req_store, dc_resp_valid;
  logic [`XLEN-1:0]             dc_req_addr, dc_req_data, dc_resp_data;
  MEM_SIZE                      dc_req_size;
  logic [`DMEM_ID_W-1:0]        dc_req_id, dc_resp_id;

  // memory-dependence prediction: a load replayed once waits for older store addresses next time
  // (shared by the threads; a replay trains it with the replayed load's own PC)
  logic lwt_wait;
  load_wait_table lwt_0 (
      .clock      (clock),
      .reset      (reset),
      .lookup_pc  (thread_pc(issue_pkt[LD_PORT].tid, issue_pkt[LD_PORT].PC, $clog2(`LWT_SZ) + 1)),
      .lookup_wait(lwt_wait),
      .train_valid(replay_flush),
      .train_pc   (thread_pc(commit_tid, redirect_pc[commit_tid], $clog2(`LWT_SZ) + 1))   // (only the committing thread can flush)
  );

  // One LSQ per thread. Shared, one at a time, lowest thread first: the load and store result
  // ports (fu_out) and the D-cache request port. A thread's dmem_req_ready means "granted to me
  // and accepted" (the LSQ treats acceptance as the moment a load reads). Responses go back by
  // the thread bits of the request id.
  logic [NT-1:0]                   ld_ready_t, st_ready_t, ld_out_valid_t, st_out_valid_t;
  CDB_PACKET [NT-1:0]              ld_out_pkt_t, st_out_pkt_t;
  logic [NT-1:0]                   ld_out_grant_t, st_out_grant_t;
  logic [NT-1:0]                   dmem_req_valid_t, dmem_req_ready_t, dmem_req_store_t, dmem_resp_valid_t;
  logic [NT-1:0][`XLEN-1:0]        dmem_req_addr_t, dmem_req_data_t;
  MEM_SIZE [NT-1:0]                dmem_req_size_t;
  logic [NT-1:0][`DMEM_ID_W-1:0]   dmem_req_id_t;
  logic [NT-1:0]                   stores_pending_t;
  logic [`TID_W-1:0]               ld_sel, st_sel, dmem_sel;
  logic [NT-1:0]                   amo_lock_t, snoop_valid_t;
  logic                            shared_store;     // an accepted store into the shared window

  for (genvar t = 0; t < NT; t++) begin : gen_lsq
    lsq #(.TID(t)) lsq_0 (
        .clock         (clock),
        .reset         (reset),
        .flush         (flush[t]),
        .disp_load     (disp_is_load & {W{fetch_tid == `TID_W'(t)}}),
        .disp_store    (disp_is_store & {W{fetch_tid == `TID_W'(t)}}),
        .disp_lsq_idx  (disp_lsq_idx_t[t]),
        .lq_space_ok   (lq_space_ok_t[t]),
        .sq_space_ok   (sq_space_ok_t[t]),
        .ld_in_valid   (issue_valid[LD_PORT] && issue_pkt[LD_PORT].tid == `TID_W'(t)),
        .ld_in_pkt     (issue_pkt[LD_PORT]),
        .ld_rs1        (rs1_value[LD_PORT]),
        .ld_wait       (`LSQ_SPECULATE == 2 && lwt_wait),
        .ld_ready      (ld_ready_t[t]),
        .ld_out_valid  (ld_out_valid_t[t]),
        .ld_out_pkt    (ld_out_pkt_t[t]),
        .ld_out_grant  (ld_out_grant_t[t]),
        .st_in_valid   (issue_valid[ST_PORT] && issue_pkt[ST_PORT].tid == `TID_W'(t)),
        .st_in_pkt     (issue_pkt[ST_PORT]),
        .st_rs1        (rs1_value[ST_PORT]),
        .st_rs2        (rs2_value[ST_PORT]),
        .st_ready      (st_ready_t[t]),
        .st_out_valid  (st_out_valid_t[t]),
        .st_out_pkt    (st_out_pkt_t[t]),
        .st_out_grant  (st_out_grant_t[t]),
        .commit_load   (commit_is_load_t[t]),
        .commit_store  (commit_is_store_t[t]),
        .dmem_req_valid(dmem_req_valid_t[t]),
        .dmem_req_ready(dmem_req_ready_t[t]),
        .dmem_req_store(dmem_req_store_t[t]),
        .dmem_req_addr (dmem_req_addr_t[t]),
        .dmem_req_size (dmem_req_size_t[t]),
        .dmem_req_data (dmem_req_data_t[t]),
        .dmem_req_id   (dmem_req_id_t[t]),
        .dmem_resp_valid(dmem_resp_valid_t[t]),
        .dmem_resp_id  (dc_resp_id),
        .dmem_resp_data(dc_resp_data),
        .stores_pending(stores_pending_t[t]),
        .viol_valid    (viol_valid[t]),
        .viol_rob_idx  (viol_rob_idx[t]),
        .snoop_valid   (snoop_valid_t[t]),
        .snoop_addr    (dc_req_addr),              // (in the shared window: the same in every thread)
        .snoop_viol_valid  (snoop_viol_valid_t[t]),
        .snoop_viol_rob_idx(snoop_viol_rob_idx[t]),
        .amo_lock      (amo_lock_t[t])
    );
    // another thread's store accepted into the shared window
    assign snoop_valid_t[t] = shared_store && dmem_sel != `TID_W'(t);
    assign ld_out_grant_t[t]    = fu_grant[LD_PORT] && ld_sel == `TID_W'(t);
    assign st_out_grant_t[t]    = fu_grant[ST_PORT] && st_sel == `TID_W'(t);
    assign dmem_req_ready_t[t]  = dc_req_ready && dmem_sel == `TID_W'(t);
    assign dmem_resp_valid_t[t] = dc_resp_valid &&
                                  (NT == 1 || dc_resp_id[`DMEM_ID_W-1 -: `TID_W] == `TID_W'(t));
  end

  always_comb begin
    automatic bit locked = 0;
    ld_sel = '0; st_sel = '0; dmem_sel = '0;
    for (int t = NT - 1; t >= 0; t--) begin      // ends on the lowest
      if (ld_out_valid_t[t])   ld_sel   = `TID_W'(t);
      if (st_out_valid_t[t])   st_sel   = `TID_W'(t);
      if (dmem_req_valid_t[t] && !locked) dmem_sel = `TID_W'(t);
    end
    // an AMO between its read and its write keeps the D-cache port: no other thread's access can fall
    // between them (it waits while the read is in memory, too)
    for (int t = 0; t < NT; t++) if (amo_lock_t[t]) begin locked = 1; dmem_sel = `TID_W'(t); end
  end
  assign shared_store = NT > 1 && dc_req_valid && dc_req_ready && dc_req_store &&
                        dc_req_addr >= `XLEN'(`PRIV_MEM_SZ) && dc_req_addr < `XLEN'(`THREAD_MEM_SZ);
  // (the RS sees one load and one store port: ready only if every thread's LSQ can take it)
  assign fu_ready[LD_PORT]     = &ld_ready_t;
  assign fu_ready[ST_PORT]     = &st_ready_t;
  assign fu_out_valid[LD_PORT] = ld_out_valid_t[ld_sel];
  assign fu_out_pkt[LD_PORT]   = ld_out_pkt_t[ld_sel];
  assign fu_out_valid[ST_PORT] = st_out_valid_t[st_sel];
  assign fu_out_pkt[ST_PORT]   = st_out_pkt_t[st_sel];
  assign dc_req_valid          = dmem_req_valid_t[dmem_sel];
  assign dc_req_store          = dmem_req_store_t[dmem_sel];
  assign dc_req_addr           = phys(dmem_sel, dmem_req_addr_t[dmem_sel]);   // (the LSQ checked the bound)
  assign dc_req_size           = dmem_req_size_t[dmem_sel];
  assign dc_req_data           = dmem_req_data_t[dmem_sel];
  assign dc_req_id             = dmem_req_id_t[dmem_sel];
  assign stores_pending        = |stores_pending_t;

  // synopsys translate_off
  // every D-cache access stays inside its thread's part of memory
  always_ff @(posedge clock)
    if (!reset && dc_req_valid && dc_req_ready)
      assert (int'(dc_req_addr) / `THREAD_MEM_SZ == int'(dmem_sel) ||
              (int'(dc_req_addr) >= `PRIV_MEM_SZ && int'(dc_req_addr) < `THREAD_MEM_SZ))
      else $error("cpu: thread %0d accessed physical %h, outside its %0d-byte part and the shared window",
                  dmem_sel, dc_req_addr, `THREAD_MEM_SZ);
  // synopsys translate_on

  // ---------------- memory hierarchy ----------------
  dcache dcache_0 (
      .clock       (clock),
      .reset       (reset),
      .req_valid   (dc_req_valid),
      .req_ready   (dc_req_ready),
      .req_store   (dc_req_store),
      .req_addr    (dc_req_addr),
      .req_size    (dc_req_size),
      .req_data    (dc_req_data),
      .req_id      (dc_req_id),
      .resp_valid  (dc_resp_valid),
      .resp_id     (dc_resp_id),
      .resp_data   (dc_resp_data),
      .mem_command (d_command),
      .mem_addr    (d_addr),
      .mem_data    (d_data),
      .mem_response(d_response),
      .mem_rdata   (mem_rdata),
      .mem_tag     (d_tag)
  );

  mem_arbiter mem_arbiter_0 (
      .clock       (clock),
      .reset       (reset),
      .d_command   (d_command),
      .d_addr      (d_addr),
      .d_data      (d_data),
      .d_response  (d_response),
      .d_tag       (d_tag),
      .i_command   (i_command),
      .i_addr      (i_addr),
      .i_response  (i_response),
      .i_tag       (i_tag),
      .p_command   (p_command),
      .p_addr      (p_addr),
      .p_response  (p_response),
      .rdata       (mem_rdata),
      .mem_command (proc2mem_command),
      .mem_addr    (proc2mem_addr),
      .mem_data    (proc2mem_data),
      .mem_response(mem2proc_response),
      .mem_rdata   (mem2proc_data),
      .mem_tag     (mem2proc_tag)
  );

  cdb_arbiter cdb_arbiter_0 (
      .fu_valid (fu_out_valid),
      .fu_pkt   (fu_out_pkt),
      .fu_grant (fu_grant),
      .cdb_valid(cdb_valid),
      .cdb_pkt  (cdb_pkt)
  );

endmodule  // cpu
