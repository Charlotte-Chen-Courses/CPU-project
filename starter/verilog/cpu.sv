/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  cpu.sv                                              //
//                                                                     //
//  Description :  Top level of the R10K-style core: integer ALU code, //
//                 branches and jumps, ending in WFI.                  //
//                                                                     //
//    fetch (W words at PC) -> dispatch (decode, rename) -> ROB + RS   //
//    RS select -> PRF read -> ALU FU -> CDB arbiter -> CDB            //
//    CDB -> PRF write, RS wakeup, busy table, ROB done                //
//    ROB commit -> freelist free, retirement RAT, halt                //
//    ROB commit of a mispredict -> flush (recovery at commit)         //
//                                                                     //
//                 Fetch: the testbench returns W words at imem_addr   //
//                 (combinational). PC advances by 4*W when dispatch   //
//                 fires; fetching stops after a WFI is dispatched.    //
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
//                 Not yet: LSQ, FP, checkpoints.                      //
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

    // instruction memory: W consecutive words starting at imem_addr
    output logic [`XLEN-1:0]            imem_addr,
    input  INST  [W-1:0]                imem_data,

    // commit trace for the testbench
    output logic [W-1:0]                commit_valid,
    output logic [W-1:0]                commit_dest_valid,
    output logic [W-1:0][4:0]           commit_rd,
    output logic [W-1:0][TAG_W-1:0]     commit_tag,
    output logic [W-1:0][`XLEN-1:0]     commit_pc,
    output logic                        halted,         // a WFI or an exception has committed
    output EXCEPTION                    exception,      // why it stopped (EXC_NONE = WFI)
    output logic [`XLEN-1:0]            exception_pc    // PC of the excepting instruction
);

  // ---------------- fetch ----------------
  logic [`XLEN-1:0]        pc;
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

  assign imem_addr = pc;
  // the first slot predicted taken redirects fetch; younger slots in the bundle are dropped
  // a predicted return takes its target from the RAS; a predicted call pushes its PC+4.
  // Both are taken jumps, so they end the bundle: at most one RAS op per cycle.
  always_comb begin
    automatic bit redirected = `FALSE;
    fetch_next_pc = pc + `XLEN'(4 * W);
    ras_push      = `FALSE;
    ras_pop       = `FALSE;
    ras_push_addr = '0;
    for (int i = 0; i < W; i++) begin
      fetch_pc[i]       = pc + `XLEN'(4 * i);
      fetch_valid[i]    = !fetch_stop && !redirected;
      fetch_pred_npc[i] = !pred_taken[i] ? fetch_pc[i] + 4 :
                           pred_ret[i]   ? ras_top        : pred_target[i];
      if (fetch_valid[i] && pred_taken[i]) begin
        redirected    = `TRUE;
        fetch_next_pc = fetch_pred_npc[i];
        ras_push      = pred_call[i];
        ras_pop       = pred_ret[i];
        ras_push_addr = fetch_pc[i] + 4;
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
  logic                        rob_space_ok, rs_space_ok;
  logic [W-1:0][ROB_IDX_W-1:0] rob_disp_idx;
  logic [W-1:0]                disp_valid, disp_dest_valid, disp_halt, rs_disp_valid;
  logic [W-1:0]                disp_is_br, disp_uncond, disp_is_call, disp_is_ret;
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
  logic [CDB_SZ-1:0]                cdb_branch, cdb_misaligned;
  logic [`ARCH_REG_SZ-1:0][TAG_W-1:0] retire_map, retire_map_next;

  logic [W-1:0]                commit_halt;
  logic [W-1:0][TAG_W-1:0]     commit_told;

  always_comb
    for (int k = 0; k < CDB_SZ; k++) begin
      cdb_tag[k]     = cdb_pkt[k].tag;
      cdb_rob_idx[k] = cdb_pkt[k].rob_idx;
      cdb_value[k]   = cdb_pkt[k].value;
      cdb_branch[k]  = cdb_pkt[k].branch;
      cdb_next_pc[k] = cdb_pkt[k].next_pc;
      cdb_misaligned[k] = cdb_pkt[k].misaligned;
    end

  dispatch dispatch_0 (
      .clock          (clock),
      .reset          (reset),
      .in_valid       (fetch_valid),
      .in_inst        (imem_data),
      .in_pc          (fetch_pc),
      .in_pred_npc    (fetch_pred_npc),
      .fire           (fire),
      .rob_space_ok   (rob_space_ok),
      .rs_space_ok    (rs_space_ok),
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
      .rs_disp_valid  (rs_disp_valid),
      .disp_pkt       (disp_pkt),
      .cdb_valid      (cdb_valid),
      .cdb_tag        (cdb_tag),
      .free_en        (commit_valid & commit_dest_valid),
      .free_tag       (commit_told),
      .flush          (flush),
      .recover_map    (retire_map_next)
  );

  always_ff @(posedge clock) begin
    if (reset) begin
      pc         <= '0;
      fetch_stop <= `FALSE;
    end else if (flush) begin
      pc         <= redirect_pc;
      fetch_stop <= `FALSE;           // a WFI fetched on the wrong path may have set it
    end else if (fire) begin
      pc <= fetch_next_pc;
      if (|(disp_valid & disp_halt)) fetch_stop <= `TRUE;
    end
  end

  // ---------------- ROB ----------------
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
      .disp_idx         (rob_disp_idx),
      .space_ok         (rob_space_ok),
      .cdb_valid        (cdb_valid),
      .cdb_rob_idx      (cdb_rob_idx),
      .cdb_branch       (cdb_branch),
      .cdb_next_pc      (cdb_next_pc),
      .cdb_misaligned   (cdb_misaligned),
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
      .commit_next_pc   (commit_next_pc),
      .commit_halt      (commit_halt),
      .commit_exception (commit_exception),
      .flush            (flush),
      .redirect_pc      (redirect_pc)
  );


  retire_rat retire_rat_0 (
      .clock     (clock),
      .reset     (reset),
      .commit_en (commit_valid & commit_dest_valid),
      .commit_rd (commit_rd),
      .commit_tag(commit_tag),
      .map       (retire_map),           // architectural xN = PRF[retire_map[N]]
      .map_next  (retire_map_next)       // after this cycle's commits: for recovery
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
      .fu_ready   (fu_ready),
      .issue_valid(issue_valid),
      .issue_pkt  (issue_pkt)
  );

  // ---------------- PRF read: pack issued ports onto the ISSUE_W read-port pairs ----------------
  logic [2*ISSUE_W-1:0][TAG_W-1:0]   prf_rd_tag;
  logic [2*ISSUE_W-1:0][`XLEN-1:0]   prf_rd_data;
  logic [NUM_FU-1:0][$clog2(ISSUE_W+1)-1:0] port_slot;
  logic [NUM_FU-1:0][`XLEN-1:0]      rs1_value, rs2_value;
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
  always_comb
    for (int f = 0; f < NUM_FU; f++) begin
      rs1_value[f] = prf_rd_data[2*port_slot[f]];
      rs2_value[f] = prf_rd_data[2*port_slot[f] + 1];
    end

  prf prf_0 (
      .clock  (clock),
      .rd_tag (prf_rd_tag),
      .rd_data(prf_rd_data),
      .wr_en  (cdb_valid),
      .wr_tag (cdb_tag),
      .wr_data(cdb_value)
  );

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
    end else begin : gen_none
      // FU type not implemented yet: never ready, never produces a result
      assign fu_ready[g]     = `FALSE;
      assign fu_out_valid[g] = `FALSE;
      assign fu_out_pkt[g]   = '0;
    end
  end

  cdb_arbiter cdb_arbiter_0 (
      .fu_valid (fu_out_valid),
      .fu_pkt   (fu_out_pkt),
      .fu_grant (fu_grant),
      .cdb_valid(cdb_valid),
      .cdb_pkt  (cdb_pkt)
  );

endmodule  // cpu
