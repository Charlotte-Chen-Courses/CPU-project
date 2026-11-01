/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  branch_pred.sv                                      //
//                                                                     //
//  Description :  Branch predictor for fetch: BTB + bimodal, or (BP_TYPE//
//                 = 1) BTB + tournament.                              //
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
//                 Tournament: also a gshare table of PHT_SZ counters  //
//                 indexed by PC[.:2] XOR the global history the slot  //
//                 was looked up with (lookup_hist; the core keeps the //
//                 history per thread and carries each branch's into   //
//                 the ROB, so training uses exactly that index:       //
//                 upd_hist), and CHOOSER_SZ 2-bit choosers by         //
//                 PC[.:2] (>= 2: trust gshare; reset to weakly        //
//                 bimodal). At commit a conditional branch trains     //
//                 both tables, and the chooser moves toward the one   //
//                 that was right when they disagree (their counters   //
//                 as they stand when the update applies, in program   //
//                 order).                                             //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

module branch_pred #(
    localparam W         = `SUPERSCALAR_WIDTH,
    localparam BTB_SZ    = `BTB_SZ,
    localparam BHT_SZ    = `BHT_SZ,
    localparam BTB_IDX_W = $clog2(BTB_SZ),
    localparam BHT_IDX_W = $clog2(BHT_SZ),
    localparam BTB_TAG_W = `XLEN - 2 - BTB_IDX_W,
    localparam PHT_IDX_W = $clog2(`PHT_SZ),
    localparam CH_IDX_W  = $clog2(`CHOOSER_SZ),
    localparam H_W       = `GHR_W
) (
    input clock,
    input reset,

    // lookup, one per fetch slot (combinational)
    input        [W-1:0][`XLEN-1:0] lookup_pc,
    /* verilator lint_off UNUSEDSIGNAL */      // (bimodal: no history)
    input        [W-1:0][H_W-1:0]   lookup_hist,   // global history each slot is predicted with (tournament)
    /* verilator lint_on UNUSEDSIGNAL */
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
    input        [W-1:0][`XLEN-1:0] upd_target,    // actual next PC when taken
    /* verilator lint_off UNUSEDSIGNAL */
    input        [W-1:0][H_W-1:0]   upd_hist       // the history the branch was predicted with
    /* verilator lint_on UNUSEDSIGNAL */
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
  logic [1:0] pht [`PHT_SZ];        // (tournament only)
  logic [1:0] chooser [`CHOOSER_SZ];

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
  function automatic logic [PHT_IDX_W-1:0] pht_idx(input logic [`XLEN-1:0] pc, input logic [H_W-1:0] h);
    return pc[PHT_IDX_W+1:2] ^ PHT_IDX_W'(h);
  endfunction
  function automatic logic [CH_IDX_W-1:0] ch_idx(input logic [`XLEN-1:0] pc);
    return pc[CH_IDX_W+1:2];
  endfunction
  function automatic logic [1:0] sat(input logic [1:0] c, input bit up);
    if (up) return (c == 2'b11) ? 2'b11 : c + 2'b01;
    return (c == 2'b00) ? 2'b00 : c - 2'b01;
  endfunction
  /* verilator lint_on UNUSEDSIGNAL */

  // ---- lookup ----
  always_comb
    for (int i = 0; i < W; i++) begin
      automatic BTB_ENTRY e = btb[btb_idx(lookup_pc[i])];
      automatic bit dir = bht[bht_idx(lookup_pc[i])][1];
      if (`BP_TYPE == 1 && chooser[ch_idx(lookup_pc[i])][1]) dir = pht[pht_idx(lookup_pc[i], lookup_hist[i])][1];
      btb_hit[i]     = e.valid && (e.tag == btb_tag(lookup_pc[i]));
      pred_taken[i]  = btb_hit[i] && (e.uncond || dir);
      pred_target[i] = e.target;
      pred_call[i]   = btb_hit[i] && e.call;
      pred_ret[i]    = btb_hit[i] && e.ret;
    end

  // ---- update: new counter values per slot, chained through older slots on the same counter ----
  logic [W-1:0][1:0] upd_cnt, upd_pcnt, upd_ch;
  always_comb begin
    automatic logic [W-1:0][1:0] cnt = '0, pcnt = '0, ch = '0;
    for (int i = 0; i < W; i++) begin
      automatic logic [1:0] base  = bht[bht_idx(upd_pc[i])];
      automatic logic [1:0] pbase = pht[pht_idx(upd_pc[i], upd_hist[i])];
      automatic logic [1:0] cbase = chooser[ch_idx(upd_pc[i])];
      for (int j = 0; j < i; j++)
        if (upd_valid[j] && !upd_uncond[j]) begin
          if (bht_idx(upd_pc[j]) == bht_idx(upd_pc[i]))                               base  = cnt[j];
          if (pht_idx(upd_pc[j], upd_hist[j]) == pht_idx(upd_pc[i], upd_hist[i]))     pbase = pcnt[j];
          if (ch_idx(upd_pc[j]) == ch_idx(upd_pc[i]))                                 cbase = ch[j];
        end
      cnt[i]  = sat(base, upd_taken[i]);
      pcnt[i] = sat(pbase, upd_taken[i]);
      // the chooser learns only when the two disagree: toward gshare if it was the right one
      ch[i]   = (base[1] != pbase[1]) ? sat(cbase, pbase[1] == upd_taken[i]) : cbase;
    end
    upd_cnt = cnt; upd_pcnt = pcnt; upd_ch = ch;
  end

  always_ff @(posedge clock) begin
    if (reset) begin
      for (int e = 0; e < BTB_SZ; e++) btb[e].valid <= `FALSE;
      for (int c = 0; c < BHT_SZ; c++) bht[c] <= 2'b01;   // weakly not-taken
      for (int c = 0; c < `PHT_SZ; c++) pht[c] <= 2'b01;
      for (int c = 0; c < `CHOOSER_SZ; c++) chooser[c] <= 2'b01;   // weakly bimodal
    end else begin
      for (int i = 0; i < W; i++)
        if (upd_valid[i]) begin
          if (!upd_uncond[i]) begin
            bht[bht_idx(upd_pc[i])] <= upd_cnt[i];
            if (`BP_TYPE == 1) begin
              pht[pht_idx(upd_pc[i], upd_hist[i])] <= upd_pcnt[i];
              chooser[ch_idx(upd_pc[i])]           <= upd_ch[i];
            end
          end
          if (upd_taken[i])
            btb[btb_idx(upd_pc[i])] <= '{valid: `TRUE, uncond: upd_uncond[i],
                                         call: upd_call[i], ret: upd_ret[i],
                                         tag: btb_tag(upd_pc[i]), target: upd_target[i]};
        end
    end
  end

endmodule  // branch_pred
