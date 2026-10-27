/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  rs.sv                                               //
//                                                                     //
//  Description :  Reservation station (unified issue queue).          //
//                 Dispatch: up to WIDTH packets into any free entries //
//                 (first-free search; space_ok = room for a full      //
//                 bundle, from state only). Wakeup: every CDB tag is  //
//                 compared with the three source tags of every valid  //
//                 entry; a packet dispatched while its source is on   //
//                 the CDB arrives ready (same-cycle bypass).          //
//                 Issue: per FU port, lowest-index ready entry of     //
//                 that FU type, at most ISSUE_WIDTH per cycle in      //
//                 total; FU types served longest latency first        //
//                 (rank_type in sys_defs). fu_ready backpressures a   //
//                 port. Issued entries free at the clock edge.        //
//                 A serial entry (CSR, later atomics) is ready only   //
//                 when it is the oldest instruction of its thread     //
//                 (rob_head[tid]).                                    //
//                 Known: lowest-index select can starve high slots.   //
//                 flush[t] clears thread t's entries.                 //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

module rs #(
    localparam W = `SUPERSCALAR_WIDTH,
    localparam RS_SZ = `RS_SZ,
    localparam CDB_SZ = `NUM_CDB,
    localparam PTR_W = `RS_IDX_W,
    localparam TAG_W = `TAG_W,
    localparam NUM_FU = `NUM_FU,
    localparam ISSUE_W = `ISSUE_WIDTH
) (
    input                    clock,
    input                    reset,
    // dispatch
    input            [W-1:0] disp_valid,
    input  RS_PACKET [W-1:0] disp_pkt,
    output logic             space_ok,    // ≥ W free entries

    // CDB stage
    input [CDB_SZ-1:0]            cdb_valid,
    input [CDB_SZ-1:0][TAG_W-1:0] cdb_tag,

    input [`NUM_THREADS-1:0] flush,  // mispredict in thread t: squash its entries
    input [`NUM_THREADS-1:0][`ROB_IDX_W-1:0] rob_head,  // each thread's oldest: a serial entry issues only there

    // issue
    input [NUM_FU-1:0] fu_ready,
    output logic [NUM_FU-1:0] issue_valid,
    output RS_PACKET [NUM_FU-1:0] issue_pkt,
    output logic [`NUM_THREADS-1:0][$clog2(RS_SZ+1)-1:0] thread_count   // entries per thread (ICOUNT fetch)

);

  RS_PACKET rs_map[RS_SZ];
  RS_PACKET [W-1:0] cdb_pkt;  // dispatched packet with this cycle's CDB applied
  logic [RS_SZ-1:0] rs_valid;
  logic [RS_SZ-1:0] taken;

  // dispatch
  logic [W-1:0][PTR_W-1:0] alloc_idx;
  logic [W-1:0] alloc_found;

  // issue
  logic [RS_SZ-1:0] ready, granted;
  logic [`NUM_FU-1:0][PTR_W-1:0] grant_idx;



  // issue priority between FU types when ISSUE_W slots run out: longest latency first
  // (same order as the CDB arbiter), so long operations start early and a steady
  // stream of ALU work cannot starve the multiplier / FP unit

  int n_issue;  // issue slots used so far this cycle

  always_comb begin
    taken = rs_valid;
    for (int slot = 0; slot < W; slot++) begin
      alloc_found[slot] = `FALSE;
      alloc_idx[slot]   = '0;
      if (disp_valid[slot])
        for (int entry = 0; entry < RS_SZ; entry++)
        if (!taken[entry] && !alloc_found[slot]) begin
          alloc_idx[slot]   = PTR_W'(entry);
          alloc_found[slot] = `TRUE;
          taken[entry]      = `TRUE;
        end
    end
  end

  assign space_ok = ($countones(~rs_valid) >= W);

  always_comb begin
    thread_count = '0;
    for (int e = 0; e < RS_SZ; e++)
      if (rs_valid[e]) thread_count[rs_map[e].tid] = thread_count[rs_map[e].tid] + 1'b1;
  end

  // same-cycle wakeup: a source broadcast this cycle is ready on arrival
  always_comb begin
    for (int i = 0; i < W; i++) begin
      cdb_pkt[i] = disp_pkt[i];
      for (int k = 0; k < CDB_SZ; k++)
      if (cdb_valid[k]) begin
        if (disp_pkt[i].src1_tag == cdb_tag[k]) cdb_pkt[i].src1_ready = `TRUE;
        if (disp_pkt[i].src2_tag == cdb_tag[k]) cdb_pkt[i].src2_ready = `TRUE;
        if (disp_pkt[i].src3_tag == cdb_tag[k]) cdb_pkt[i].src3_ready = `TRUE;
      end
    end
  end

  // issue search
  always_comb begin
    for (int e = 0; e < RS_SZ; e++)
    ready[e] = rs_valid[e] && rs_map[e].src1_ready && rs_map[e].src2_ready && rs_map[e].src3_ready &&
               (!rs_map[e].serial || rs_map[e].rob_idx == rob_head[rs_map[e].tid]);

    granted     = '0;
    issue_valid = '0;
    grant_idx   = '0;
    n_issue     = 0;
    // ports visited by FU-type rank, then port number; at most ISSUE_W grants in total
    for (int r = 0; r < `NUM_FU_RANKS; r++)
      for (int f = 0; f < `NUM_FU; f++)
        if (port_type(f) == rank_type(r) && fu_ready[f] && n_issue < ISSUE_W)
          for (int e = 0; e < RS_SZ; e++)
          if (!issue_valid[f] && ready[e] && !granted[e] && rs_map[e].fu_type == port_type(f)) begin
            issue_valid[f] = `TRUE;
            grant_idx[f]   = PTR_W'(e);
            granted[e]     = `TRUE;
            n_issue++;
          end
    for (int f = 0; f < `NUM_FU; f++) issue_pkt[f] = rs_map[grant_idx[f]];
  end


  always_ff @(posedge clock) begin
    if (reset) rs_valid <= '0;
    else begin
      for (int e = 0; e < RS_SZ; e++) begin
        if (rs_valid[e])
          for (int k = 0; k < CDB_SZ; k++)
          if (cdb_valid[k]) begin
            if (rs_map[e].src1_tag == cdb_tag[k]) rs_map[e].src1_ready <= `TRUE;
            if (rs_map[e].src2_tag == cdb_tag[k]) rs_map[e].src2_ready <= `TRUE;
            if (rs_map[e].src3_tag == cdb_tag[k]) rs_map[e].src3_ready <= `TRUE;
          end
      end
      for (int i = 0; i < W; i++)
      if (disp_valid[i] && !flush[disp_pkt[i].tid]) begin   // (a dispatch in its thread's flush cycle is dropped)
        rs_map[alloc_idx[i]]   <= cdb_pkt[i];
        rs_valid[alloc_idx[i]] <= `TRUE;
      end
      for (int f = 0; f < `NUM_FU; f++) if (issue_valid[f]) rs_valid[grant_idx[f]] <= `FALSE;
      // (only entries valid before this cycle: a slot allocated now may still hold an old tid)
      for (int e = 0; e < RS_SZ; e++) if (rs_valid[e] && flush[rs_map[e].tid]) rs_valid[e] <= `FALSE;
    end
  end






endmodule
