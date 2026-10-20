/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  branch_pred.sv                                      //
//                                                                     //
//  Description :  Branch predictor for fetch: BTB + bimodal.          //
//                 BTB: BTB_SZ entries, direct-mapped on PC[.:2], full //
//                 tag; each holds the last taken target and whether   //
//                 it is an unconditional jump (jal/jalr).             //
//                 Bimodal: BHT_SZ 2-bit saturating counters indexed   //
//                 by PC[.:2], reset to weakly not-taken.              //
//                 Lookup (combinational, one per fetch slot):         //
//                   taken = BTB hit && (jump || counter >= 2)         //
//                   target = BTB target                               //
//                 Update at COMMIT (never trained by wrong-path       //
//                 branches), in program order: conditional branches   //
//                 move their counter; taken branches/jumps write the  //
//                 BTB. Two updates to the same counter in one cycle   //
//                 chain (the younger starts from the older's result). //
//                 Entries also mark calls and returns: fetch uses the //
//                 RAS (ras.sv) for a return's target.                 //
//                 Every misprediction is caught by the ROB's          //
//                 predicted-vs-actual next PC check at commit.        //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

module branch_pred #(
    localparam W         = `SUPERSCALAR_WIDTH,
    localparam BTB_SZ    = `BTB_SZ,
    localparam BHT_SZ    = `BHT_SZ,
    localparam BTB_IDX_W = $clog2(BTB_SZ),
    localparam BHT_IDX_W = $clog2(BHT_SZ),
    localparam BTB_TAG_W = `XLEN - 2 - BTB_IDX_W
) (
    input clock,
    input reset,

    // lookup, one per fetch slot (combinational)
    input        [W-1:0][`XLEN-1:0] lookup_pc,
    output logic [W-1:0]            pred_taken,
    output logic [W-1:0][`XLEN-1:0] pred_target,   // meaningful when pred_taken
    output logic [W-1:0]            btb_hit,       // for statistics
    output logic [W-1:0]            pred_call,     // hit on a call: fetch pushes PC+4 on the RAS
    output logic [W-1:0]            pred_ret,      // hit on a return: target comes from the RAS

    // update from committing branches/jumps, in program order (excepting ones excluded)
    input        [W-1:0]            upd_valid,
    input        [W-1:0][`XLEN-1:0] upd_pc,
    input        [W-1:0]            upd_uncond,    // jal / jalr
    input        [W-1:0]            upd_call,      // jal/jalr with rd = x1/x5
    input        [W-1:0]            upd_ret,       // jalr x0, x1/x5
    input        [W-1:0]            upd_taken,
    input        [W-1:0][`XLEN-1:0] upd_target     // actual next PC when taken
);

  typedef struct packed {
    logic                 valid;
    logic                 uncond;
    logic                 call;
    logic                 ret;
    logic [BTB_TAG_W-1:0] tag;
    logic [`XLEN-1:0]     target;
  } BTB_ENTRY;

  BTB_ENTRY   btb [BTB_SZ];
  logic [1:0] bht [BHT_SZ];

  // each function uses only its own slice of the PC
  /* verilator lint_off UNUSEDSIGNAL */
  function automatic logic [BTB_IDX_W-1:0] btb_idx(input logic [`XLEN-1:0] pc);
    return pc[BTB_IDX_W+1:2];
  endfunction
  function automatic logic [BTB_TAG_W-1:0] btb_tag(input logic [`XLEN-1:0] pc);
    return pc[`XLEN-1:BTB_IDX_W+2];
  endfunction
  function automatic logic [BHT_IDX_W-1:0] bht_idx(input logic [`XLEN-1:0] pc);
    return pc[BHT_IDX_W+1:2];
  endfunction
  /* verilator lint_on UNUSEDSIGNAL */

  // ---- lookup ----
  always_comb
    for (int i = 0; i < W; i++) begin
      automatic BTB_ENTRY e = btb[btb_idx(lookup_pc[i])];
      btb_hit[i]     = e.valid && (e.tag == btb_tag(lookup_pc[i]));
      pred_taken[i]  = btb_hit[i] && (e.uncond || bht[bht_idx(lookup_pc[i])][1]);
      pred_target[i] = e.target;
      pred_call[i]   = btb_hit[i] && e.call;
      pred_ret[i]    = btb_hit[i] && e.ret;
    end

  // ---- update: new counter value per slot, chained through older slots on the same counter ----
  logic [W-1:0][1:0] upd_cnt;
  always_comb begin
    automatic logic [W-1:0][1:0] cnt = '0;
    for (int i = 0; i < W; i++) begin
      automatic logic [1:0] base = bht[bht_idx(upd_pc[i])];
      for (int j = 0; j < i; j++)
        if (upd_valid[j] && !upd_uncond[j] && bht_idx(upd_pc[j]) == bht_idx(upd_pc[i]))
          base = cnt[j];
      if (upd_taken[i]) cnt[i] = (base == 2'b11) ? 2'b11 : base + 2'b01;
      else              cnt[i] = (base == 2'b00) ? 2'b00 : base - 2'b01;
    end
    upd_cnt = cnt;
  end

  always_ff @(posedge clock) begin
    if (reset) begin
      for (int e = 0; e < BTB_SZ; e++) btb[e].valid <= `FALSE;
      for (int c = 0; c < BHT_SZ; c++) bht[c] <= 2'b01;   // weakly not-taken
    end else begin
      for (int i = 0; i < W; i++)
        if (upd_valid[i]) begin
          if (!upd_uncond[i]) bht[bht_idx(upd_pc[i])] <= upd_cnt[i];
          if (upd_taken[i])
            btb[btb_idx(upd_pc[i])] <= '{valid: `TRUE, uncond: upd_uncond[i],
                                         call: upd_call[i], ret: upd_ret[i],
                                         tag: btb_tag(upd_pc[i]), target: upd_target[i]};
        end
    end
  end

endmodule  // branch_pred
