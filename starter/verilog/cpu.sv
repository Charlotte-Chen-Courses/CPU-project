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
    localparam ISSUE_W   = `ISSUE_WIDTH
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
    output logic                        halted,         // a WFI or an exception has committed
    output EXCEPTION                    exception,      // why it stopped (EXC_NONE = WFI)
    output logic [`XLEN-1:0]            exception_pc,   // PC of the excepting instruction
    output logic                        stores_pending  // committed stores not yet in the D-cache
);

  // ---------------- fetch ----------------
  logic [`XLEN-1:0]        pc, pc_next;      // pc_next: the PC register's next value
  logic                    fetch_stop;     // WFI dispatched: fetch nothing more
  logic                    fire;
  logic [W-1:0]            fetch_valid;
  logic [W-1:0][`XLEN-1:0] fetch_pc, fetch_pred_npc;
  logic                    flush;          // mispredict committed this cycle
  logic [`XLEN-1:0]        redirect_pc;

  logic [W-1:0]            pred_taken, btb_hit, pred_call, pred_ret;
  logic [`XLEN-1:0]        ras_top;
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
  // instruction stream buffer
  logic             pf_ready, pf_wait, pf_take, probe_hit;
  logic [63:0]      pf_data;
  logic [`XLEN-1:0] probe_addr, p_addr;
  logic [1:0]       p_command;
  logic [3:0]       p_response;

  assign pc_in_range = (pc < `MEM_SIZE_IN_BYTES);

  icache icache_0 (
      .clock             (clock),
      .reset             (reset),
      .Imem2proc_response(i_response),
      .Imem2proc_data    (mem_rdata),
      .Imem2proc_tag     (i_tag),
      .proc2Icache_addr  (pc_in_range ? pc : '0),
      .next_addr         (pc_next < `MEM_SIZE_IN_BYTES ? pc_next : '0),   // (the SRAM reads it a cycle early)
      .proc2Imem_command (i_command),
      .proc2Imem_addr    (i_addr),
      .Icache_data_out   (icache_block),
      .Icache_valid_out  (icache_valid),
      .pf_ready          (pf_ready),
      .pf_wait           (pf_wait),
      .pf_data           (pf_data),
      .pf_take           (pf_take),
      .probe_addr        (probe_addr),
      .probe_hit         (probe_hit)
  );

  iprefetch iprefetch_0 (
      .clock        (clock),
      .reset        (reset),
      .cur_addr     (pc_in_range ? pc : '0),
      .pf_ready     (pf_ready),
      .pf_wait      (pf_wait),
      .pf_data      (pf_data),
      .pf_take      (pf_take),
      .miss_accepted(i_command != BUS_NONE && i_response != 4'd0),
      .miss_addr    (i_addr),
      .probe_addr   (probe_addr),
      .probe_hit    (probe_hit),
      .p_command    (p_command),
      .p_addr       (p_addr),
      .p_response   (p_response),
      .i_tag        (i_tag),
      .rdata        (mem_rdata)
  );

  // the first slot predicted taken redirects fetch; younger slots in the bundle are dropped
  // a predicted return takes its target from the RAS; a predicted call pushes its PC+4.
  // Both are taken jumps, so they end the bundle: at most one RAS op per cycle.
  always_comb begin
    automatic bit redirected = `FALSE;
    automatic int n_avail = pc_in_range ? (pc[2] ? 1 : 2) : 1;   // slots this block can give
    if (n_avail > W) n_avail = W;
    fetch_next_pc = pc + `XLEN'(4 * n_avail);
    ras_push      = `FALSE;
    ras_pop       = `FALSE;
    ras_push_addr = '0;
    for (int i = 0; i < W; i++) begin
      fetch_pc[i]       = pc + `XLEN'(4 * i);
      fetch_inst[i]     = (int'(pc[2]) + i < 2) ? (pc[2] + i == 1 ? icache_block[63:32] : icache_block[31:0])
                                                : `NOP;
      fetch_exc[i]      = EXC_NONE;
      if (!pc_in_range) begin
        fetch_inst[i] = `NOP;
        fetch_exc[i]  = EXC_ACCESS;
      end
      fetch_valid[i]    = !fetch_stop && !redirected && i < n_avail && (icache_valid || !pc_in_range);
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

  logic [W-1:0]            commit_is_br, commit_uncond, commit_is_call, commit_is_ret;
  logic [W-1:0][`XLEN-1:0] commit_next_pc;
  logic [W-1:0]            commit_taken;
  always_comb
    for (int i = 0; i < W; i++) commit_taken[i] = (commit_next_pc[i] != commit_pc[i] + 4);

  branch_pred branch_pred_0 (
      .clock      (clock),
      .reset      (reset),
      .lookup_pc  (fetch_pc),
      .pred_taken (pred_taken),
      .pred_target(pred_target),
      .btb_hit    (btb_hit),
      .pred_call  (pred_call),
      .pred_ret   (pred_ret),
      .upd_valid  (commit_is_br),
      .upd_pc     (commit_pc),
      .upd_uncond (commit_uncond),
      .upd_call   (commit_is_call),
      .upd_ret    (commit_is_ret),
      .upd_taken  (commit_taken),
      .upd_target (commit_next_pc)
  );

  ras ras_0 (
      .clock         (clock),
      .reset         (reset),
      .spec_push     (ras_push && fire),    // only for a bundle that is actually dispatched
      .spec_push_addr(ras_push_addr),
      .spec_pop      (ras_pop && fire),
      .top           (ras_top),
      .commit_call   (commit_is_call),
      .commit_ret    (commit_is_ret),
      .commit_pc     (commit_pc),
      .flush         (flush)
  );

  // ---------------- dispatch ----------------
  localparam LD_PORT = `NUM_FU_ALU + `NUM_FU_BRANCH + `NUM_FU_MULT + `NUM_FU_DIV;   // port_type() order
  localparam ST_PORT = LD_PORT + `NUM_FU_LOAD;
  logic                         lq_space_ok, sq_space_ok;
  logic [W-1:0][`LSQ_IDX_W-1:0] disp_lsq_idx;
  logic                        rob_space_ok, rs_space_ok;
  logic [W-1:0][ROB_IDX_W-1:0] rob_disp_idx;
  logic [W-1:0]                disp_valid, disp_dest_valid, disp_halt, rs_disp_valid;
  logic [W-1:0]                disp_is_br, disp_uncond, disp_is_call, disp_is_ret;
  logic [W-1:0]                disp_is_load, disp_is_store, commit_is_load, commit_is_store;
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
  logic [W-1:0][4:0]                commit_fflags;
  logic [ROB_IDX_W-1:0]             rob_head;
  logic                             rob_empty;
  logic [`ARCH_REG_SZ-1:0][TAG_W-1:0] retire_map, retire_map_next;         // x registers
  logic [`ARCH_REG_SZ-1:0][TAG_W-1:0] fp_retire_map, fp_retire_map_next;   // f registers

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
      .fire           (fire),
      .rob_space_ok   (rob_space_ok),
      .rob_empty      (rob_empty),
      .rs_space_ok    (rs_space_ok),
      .lq_space_ok    (lq_space_ok),
      .sq_space_ok    (sq_space_ok),
      .lsq_idx        (disp_lsq_idx),
      .rob_idx        (rob_disp_idx),
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
      .flush          (flush),
      .recover_map    (retire_map_next),
      .fp_recover_map (fp_retire_map_next)
  );

  always_comb
    if (reset)      pc_next = '0;
    else if (flush) pc_next = redirect_pc;
    else if (fire)  pc_next = fetch_next_pc;
    else            pc_next = pc;

  always_ff @(posedge clock) begin
    pc <= pc_next;
    if (reset || flush) fetch_stop <= `FALSE;   // (a WFI fetched on a wrong path may have set it)
    else if (fire && |(disp_valid & disp_halt)) fetch_stop <= `TRUE;
  end

  // ---------------- ROB ----------------
  logic                  viol_valid, replay_flush;   // LSQ memory-order violation -> replay at commit
  logic [`ROB_IDX_W-1:0] viol_rob_idx;
  rob rob_0 (
      .clock            (clock),
      .reset            (reset),
      .disp_valid       (disp_valid),
      .disp_dest_valid  (disp_dest_valid),
      .disp_rd          (disp_rd),
      .disp_tag         (disp_tag),
      .disp_told        (disp_told),
      .disp_pc          (disp_pc),
      .disp_halt        (disp_halt),
      .disp_pred_npc    (disp_pred_npc),
      .disp_exception   (disp_exception),
      .disp_is_br       (disp_is_br),
      .disp_uncond      (disp_uncond),
      .disp_is_call     (disp_is_call),
      .disp_is_ret      (disp_is_ret),
      .disp_is_load     (disp_is_load),
      .disp_is_store    (disp_is_store),
      .disp_dest_fp     (disp_dest_fp),
      .replay_valid     (viol_valid),
      .replay_rob_idx   (viol_rob_idx),
      .disp_idx         (rob_disp_idx),
      .space_ok         (rob_space_ok),
      .cdb_valid        (cdb_valid),
      .cdb_rob_idx      (cdb_rob_idx),
      .cdb_branch       (cdb_branch),
      .cdb_next_pc      (cdb_next_pc),
      .cdb_exception    (cdb_exception),
      .cdb_fflags       (cdb_fflags),
      .commit_stop      (halted),         // younger entries behind an exception never commit
      .commit_valid     (commit_valid),
      .commit_dest_valid(commit_dest_valid),
      .commit_rd        (commit_rd),
      .commit_tag       (commit_tag),
      .commit_told      (commit_told),
      .commit_pc        (commit_pc),
      .commit_is_br     (commit_is_br),
      .commit_uncond    (commit_uncond),
      .commit_is_call   (commit_is_call),
      .commit_is_ret    (commit_is_ret),
      .commit_is_load   (commit_is_load),
      .commit_is_store  (commit_is_store),
      .commit_next_pc   (commit_next_pc),
      .commit_halt      (commit_halt),
      .commit_exception (commit_exception),
      .flush            (flush),
      .redirect_pc      (redirect_pc),
      .replay_flush     (replay_flush),
      .commit_dest_fp   (commit_dest_fp),
      .commit_fflags    (commit_fflags),
      .head             (rob_head),
      .empty            (rob_empty)
  );


  retire_rat retire_rat_0 (
      .clock     (clock),
      .reset     (reset),
      .commit_en (commit_valid & commit_dest_valid & ~commit_dest_fp),
      .commit_rd (commit_rd),
      .commit_tag(commit_tag),
      .map       (retire_map),           // architectural xN = PRF[retire_map[N]]
      .map_next  (retire_map_next)       // after this cycle's commits: for recovery
  );

  retire_rat #(.BASE(`FP_TAG_BASE), .ZERO_REG(1'b0)) fp_retire_rat_0 (
      .clock     (clock),
      .reset     (reset),
      .commit_en (commit_valid & commit_dest_valid & commit_dest_fp),
      .commit_rd (commit_rd),
      .commit_tag(commit_tag),
      .map       (fp_retire_map),        // architectural fN = FP PRF[fp_retire_map[N]]
      .map_next  (fp_retire_map_next)
  );

  always_ff @(posedge clock)
    if (reset) begin
      halted       <= `FALSE;
      exception    <= EXC_NONE;
      exception_pc <= '0;
    end else if (!halted) begin
      if (|commit_halt) halted <= `TRUE;
      for (int i = 0; i < W; i++)
        if (commit_exception[i] != EXC_NONE) begin
          halted       <= `TRUE;
          exception    <= commit_exception[i];
          exception_pc <= commit_pc[i];
        end
    end

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
      .rob_head   (rob_head),
      .fu_ready   (fu_ready),
      .issue_valid(issue_valid),
      .issue_pkt  (issue_pkt)
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
  logic [7:0] fcsr, fcsr_wdata;
  logic       fcsr_we;
  always_ff @(posedge clock) begin
    automatic logic [4:0] acc = '0;
    for (int i = 0; i < W; i++) acc |= commit_fflags[i];
    if (reset)        fcsr <= '0;
    else if (fcsr_we) fcsr <= fcsr_wdata;
    else              fcsr[4:0] <= fcsr[4:0] | acc;
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
          .fcsr     (fcsr),
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
  logic lwt_wait;
  load_wait_table lwt_0 (
      .clock      (clock),
      .reset      (reset),
      .lookup_pc  (issue_pkt[LD_PORT].PC),
      .lookup_wait(lwt_wait),
      .train_valid(replay_flush),
      .train_pc   (redirect_pc)     // a replay re-fetches the load's own PC
  );

  lsq lsq_0 (
      .clock         (clock),
      .reset         (reset),
      .flush         (flush),
      .disp_load     (disp_is_load),
      .disp_store    (disp_is_store),
      .disp_lsq_idx  (disp_lsq_idx),
      .lq_space_ok   (lq_space_ok),
      .sq_space_ok   (sq_space_ok),
      .ld_in_valid   (issue_valid[LD_PORT]),
      .ld_in_pkt     (issue_pkt[LD_PORT]),
      .ld_rs1        (rs1_value[LD_PORT]),
      .ld_wait       (`LSQ_SPECULATE == 2 && lwt_wait),
      .ld_ready      (fu_ready[LD_PORT]),
      .ld_out_valid  (fu_out_valid[LD_PORT]),
      .ld_out_pkt    (fu_out_pkt[LD_PORT]),
      .ld_out_grant  (fu_grant[LD_PORT]),
      .st_in_valid   (issue_valid[ST_PORT]),
      .st_in_pkt     (issue_pkt[ST_PORT]),
      .st_rs1        (rs1_value[ST_PORT]),
      .st_rs2        (rs2_value[ST_PORT]),
      .st_ready      (fu_ready[ST_PORT]),
      .st_out_valid  (fu_out_valid[ST_PORT]),
      .st_out_pkt    (fu_out_pkt[ST_PORT]),
      .st_out_grant  (fu_grant[ST_PORT]),
      .commit_load   (commit_is_load),
      .commit_store  (commit_is_store),
      .dmem_req_valid(dc_req_valid),
      .dmem_req_ready(dc_req_ready),
      .dmem_req_store(dc_req_store),
      .dmem_req_addr (dc_req_addr),
      .dmem_req_size (dc_req_size),
      .dmem_req_data (dc_req_data),
      .dmem_req_id   (dc_req_id),
      .dmem_resp_valid(dc_resp_valid),
      .dmem_resp_id  (dc_resp_id),
      .dmem_resp_data(dc_resp_data),
      .stores_pending(stores_pending),
      .viol_valid    (viol_valid),
      .viol_rob_idx  (viol_rob_idx)
  );

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
