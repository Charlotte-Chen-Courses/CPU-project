/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  cpu.sv                                              //
//                                                                     //
//  Description :  Top level of the R10K-style core, first slice:      //
//                 straight-line integer ALU code ending in WFI.       //
//                                                                     //
//    fetch (W words at PC) -> dispatch (decode, rename) -> ROB + RS   //
//    RS select -> PRF read -> ALU FU -> CDB arbiter -> CDB            //
//    CDB -> PRF write, RS wakeup, busy table, ROB done                //
//    ROB commit -> freelist free, retirement RAT, halt                //
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
//                 Not yet: branches, MULT, LSQ, FP, recovery.         //
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
    output logic                        halted          // a WFI has committed
);

  // ---------------- fetch ----------------
  logic [`XLEN-1:0]        pc;
  logic                    fetch_stop;     // WFI dispatched: fetch nothing more
  logic                    fire;
  logic [W-1:0]            fetch_valid;
  logic [W-1:0][`XLEN-1:0] fetch_pc;

  assign imem_addr = pc;
  always_comb
    for (int i = 0; i < W; i++) begin
      fetch_valid[i] = !fetch_stop;
      fetch_pc[i]    = pc + `XLEN'(4 * i);
    end

  // ---------------- dispatch ----------------
  logic                        rob_space_ok, rs_space_ok;
  logic [W-1:0][ROB_IDX_W-1:0] rob_disp_idx;
  logic [W-1:0]                disp_valid, disp_dest_valid, disp_halt;
  logic [W-1:0][4:0]           disp_rd;
  logic [W-1:0][TAG_W-1:0]     disp_tag, disp_told;
  logic [W-1:0][`XLEN-1:0]     disp_pc;
  RS_PACKET [W-1:0]            disp_pkt;

  logic [CDB_SZ-1:0]           cdb_valid;
  CDB_PACKET [CDB_SZ-1:0]      cdb_pkt;
  logic [CDB_SZ-1:0][TAG_W-1:0]     cdb_tag;
  logic [CDB_SZ-1:0][ROB_IDX_W-1:0] cdb_rob_idx;
  logic [CDB_SZ-1:0][`XLEN-1:0]     cdb_value;

  logic [W-1:0]                commit_halt;
  logic [W-1:0][TAG_W-1:0]     commit_told;

  always_comb
    for (int k = 0; k < CDB_SZ; k++) begin
      cdb_tag[k]     = cdb_pkt[k].tag;
      cdb_rob_idx[k] = cdb_pkt[k].rob_idx;
      cdb_value[k]   = cdb_pkt[k].value;
    end

  dispatch dispatch_0 (
      .clock          (clock),
      .reset          (reset),
      .in_valid       (fetch_valid),
      .in_inst        (imem_data),
      .in_pc          (fetch_pc),
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
      .disp_pkt       (disp_pkt),
      .cdb_valid      (cdb_valid),
      .cdb_tag        (cdb_tag),
      .free_en        (commit_valid & commit_dest_valid),
      .free_tag       (commit_told)
  );

  always_ff @(posedge clock) begin
    if (reset) begin
      pc         <= '0;
      fetch_stop <= `FALSE;
    end else if (fire) begin
      pc <= pc + `XLEN'(4 * W);
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
      .disp_idx         (rob_disp_idx),
      .space_ok         (rob_space_ok),
      .cdb_valid        (cdb_valid),
      .cdb_rob_idx      (cdb_rob_idx),
      .commit_valid     (commit_valid),
      .commit_dest_valid(commit_dest_valid),
      .commit_rd        (commit_rd),
      .commit_tag       (commit_tag),
      .commit_told      (commit_told),
      .commit_pc        (commit_pc),
      .commit_halt      (commit_halt)
  );

  logic [`ARCH_REG_SZ-1:0][TAG_W-1:0] retire_map;  // read by the testbench

  retire_rat retire_rat_0 (
      .clock     (clock),
      .reset     (reset),
      .commit_en (commit_valid & commit_dest_valid),
      .commit_rd (commit_rd),
      .commit_tag(commit_tag),
      .map       (retire_map)            // architectural xN = PRF[retire_map[N]]
  );

  always_ff @(posedge clock)
    if (reset) halted <= `FALSE;
    else if (|commit_halt) halted <= `TRUE;

  // ---------------- RS ----------------
  logic [NUM_FU-1:0]  fu_ready, issue_valid;
  RS_PACKET [NUM_FU-1:0] issue_pkt;

  rs rs_0 (
      .clock      (clock),
      .reset      (reset),
      .disp_valid (disp_valid),
      .disp_pkt   (disp_pkt),
      .space_ok   (rs_space_ok),
      .cdb_valid  (cdb_valid),
      .cdb_tag    (cdb_tag),
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
