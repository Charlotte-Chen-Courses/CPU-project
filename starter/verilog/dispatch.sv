/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  dispatch.sv                                         //
//                                                                     //
//  Description :  Decode + rename + dispatch for one fetch bundle.    //
//                 Contains the decoders and, per register class (x    //
//                 and f), a freelist, speculative RAT and busy table, //
//                 and produces the ROB and RS entries for the bundle. //
//                 Rules:                                              //
//                  - fire = any valid && both freelists can_alloc_w   //
//                    && rob_space_ok && rs_space_ok && LQ/SQ space.   //
//                    All state-only (never the request-dependent      //
//                    alloc_ok), so there is no combinational loop.    //
//                    The bundle renames whole or not at all.          //
//                  - each operand picks its class on its own          //
//                    (rs1_fp/rs2_fp/rd_fp; rs3 is always f): RAT,     //
//                    busy table, same-bundle bypass, freelist, told.  //
//                  - dest_valid = valid && has_dest && (rd is an f    //
//                    register, or rd != x0): f0 is a real register.   //
//                  - freelist alloc_req, busy-table set_en, RAT       //
//                    writes and ROB/RS writes happen only on fire.    //
//                  - src ready = the field is not a register          //
//                    (rs*_used = 0: lui/auipc/jal rs1, I-type rs2,    //
//                    ...), or busy table ready (with CDB bypass) &&   //
//                    not bypassed from an older slot in the bundle    //
//                    (that value does not exist yet).                 //
//                  - RS dest_tag = dest_valid ? new tag : p0.         //
//                  - slots younger than a WFI in the bundle are       //
//                    dropped (never enter the ROB).                   //
//                  - fcsr CSR ops (fflags/frm/fcsr) and atomics are   //
//                    serial: they issue only at the ROB head, and     //
//                    nothing dispatches after one until the ROB is    //
//                    empty (fetch ends the bundle after it). CSR ops  //
//                    go to the FP unit, atomics (LR too) to the store //
//                    port without an LQ/SQ entry.                     //
//                  - commit frees told to its class's freelist        //
//                    (free_fp).                                       //
//                  - flush[t] (mispredict or replay at commit in      //
//                    thread t): no dispatch for t; t's freelists      //
//                    rewind, its RATs load its recover maps, its busy //
//                    tables clear.                                    //
//                 Threads (SMT): every thread has its own freelists,  //
//                 RATs and busy tables, on its own tag ranges (int:   //
//                 t*THREAD_PHYS_SZ.., FP: FP_TAG_BASE +               //
//                 t*FP_THREAD_PHYS_SZ..), and its own serial block.   //
//                 A bundle is one thread's (in_tid); commit frees go  //
//                 to free_tid's lists. A busy table reads tags        //
//                 outside its range as ready, so the AND of all of    //
//                 them is the right answer for any tag.               //
//                 Anything else (illegal, or no unit yet: other CSRs, //
//                 all of F when NUM_FU_FP = 0, A when A_EXTENSION =   //
//                 0) gets a ROB entry that is done at dispatch with   //
//                 an exception and no destination, and never enters  //
//                 the RS; the ROB raises it at commit.                //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"
`include "verilog/ISA.svh"

module dispatch #(
    localparam W         = `SUPERSCALAR_WIDTH,
    localparam TAG_W     = `TAG_W,
    localparam ROB_IDX_W = `ROB_IDX_W,
    localparam CDB_SZ    = `NUM_CDB,
    localparam NT        = `NUM_THREADS
) (
    input clock,
    input reset,

    // fetch bundle
    input        [W-1:0]                  in_valid,
    input  INST  [W-1:0]                  in_inst,
    input        [W-1:0][`XLEN-1:0]       in_pc,
    input  EXCEPTION [W-1:0]              in_exc,        // fetch exception (PC out of memory range)
    input        [W-1:0][`XLEN-1:0]       in_pred_npc,   // next PC fetch predicted for each slot
    input        [`TID_W-1:0]             in_tid,        // the thread the bundle belongs to
    output logic                          fire,          // bundle accepted: fetch may advance

    // back-pressure (state-only signals from the ROB and RS)
    input                                 rob_space_ok,  // (in_tid's ROB, LQ and SQ)
    input        [NT-1:0]                 rob_empty,     // nothing in flight in thread t (ends its serial block)
    input                                 rs_space_ok,
    input                                 lq_space_ok,   // >= W free LQ / SQ entries (state only)
    input                                 sq_space_ok,
    input        [W-1:0][`LSQ_IDX_W-1:0]  lsq_idx,       // LQ/SQ entry each load/store slot lands in
    input        [W-1:0][ROB_IDX_W-1:0]   rob_idx,       // ROB entry each slot lands in

    // to the ROB
    output logic [W-1:0]                  disp_valid,
    output logic [W-1:0]                  disp_dest_valid,
    output logic [W-1:0][4:0]             disp_rd,
    output logic [W-1:0][TAG_W-1:0]       disp_tag,
    output logic [W-1:0][TAG_W-1:0]       disp_told,
    output logic [W-1:0][`XLEN-1:0]       disp_pc,
    output logic [W-1:0]                  disp_halt,
    output EXCEPTION [W-1:0]              disp_exception,  // != EXC_NONE: ROB entry is done at dispatch
    output logic [W-1:0]                  disp_is_br,      // branch / jump (trains the predictor at commit)
    output logic [W-1:0]                  disp_uncond,     // jal / jalr
    output logic [W-1:0]                  disp_is_call,    // jal/jalr, rd = x1/x5
    output logic [W-1:0]                  disp_is_ret,     // jalr x0, x1/x5
    output logic [W-1:0]                  disp_is_load,    // allocates an LQ entry (gated by fire)
    output logic [W-1:0]                  disp_is_store,   // allocates an SQ entry (gated by fire)
    output logic [W-1:0]                  disp_dest_fp,    // the destination is an f register
    output logic [W-1:0][`XLEN-1:0]       disp_pred_npc,

    // to the RS: only slots that have a functional unit to run on
    output logic [W-1:0]                  rs_disp_valid,
    output RS_PACKET [W-1:0]              disp_pkt,

    // CDB: clears busy bits
    input        [CDB_SZ-1:0]             cdb_valid,
    input        [CDB_SZ-1:0][TAG_W-1:0]  cdb_tag,

    // commit: frees old physical registers (free_fp: to the FP free list)
    input        [W-1:0]                  free_en,
    input        [W-1:0]                  free_fp,
    input        [W-1:0][TAG_W-1:0]       free_tag,
    input        [`TID_W-1:0]             free_tid,      // the committing thread

    // mispredict recovery of thread t (this cycle): no dispatch for t; restore t's rename state
    input        [NT-1:0]                 flush,
    input        [NT-1:0][`ARCH_REG_SZ-1:0][TAG_W-1:0] recover_map,     // integer retirement maps after this cycle's commits
    input        [NT-1:0][`ARCH_REG_SZ-1:0][TAG_W-1:0] fp_recover_map   // FP retirement maps after this cycle's commits
);

  // ---- decode ----
  ALU_OPA_SELECT [W-1:0] opa_select;
  ALU_OPB_SELECT [W-1:0] opb_select;
  ALU_FUNC       [W-1:0] alu_func;
  FU_TYPE        [W-1:0] fu_type;
  logic [W-1:0] has_dest, rd_mem, wr_mem, cond_branch, uncond_branch, csr_op, halt, illegal;
  logic [W-1:0] rs1_fp, rs2_fp, rd_fp, rs3_used, atomic, rs1_used, rs2_used;

  logic [W-1:0][4:0] rs1, rs2, rd;

  genvar g;
  for (g = 0; g < W; g++) begin : gen_decode
    decoder decoder_0 (
        .inst         (in_inst[g]),
        .valid        (in_valid[g]),
        .opa_select   (opa_select[g]),
        .opb_select   (opb_select[g]),
        .has_dest     (has_dest[g]),
        .alu_func     (alu_func[g]),
        .rd_mem       (rd_mem[g]),
        .wr_mem       (wr_mem[g]),
        .cond_branch  (cond_branch[g]),
        .uncond_branch(uncond_branch[g]),
        .csr_op       (csr_op[g]),
        .halt         (halt[g]),
        .illegal      (illegal[g]),
        .fu_type      (fu_type[g]),
        .rs1_fp       (rs1_fp[g]),
        .rs2_fp       (rs2_fp[g]),
        .rd_fp        (rd_fp[g]),
        .rs3_used     (rs3_used[g]),
        .atomic       (atomic[g]),
        .rs1_used     (rs1_used[g]),
        .rs2_used     (rs2_used[g])
    );
    assign rs1[g] = in_inst[g][19:15];
    assign rs2[g] = in_inst[g][24:20];
    assign rd[g]  = in_inst[g][11:7];
  end

  // ---- slots after a WFI in the same bundle are dropped: nothing past a halt enters the ROB ----
  logic [W-1:0] slot_valid;
  always_comb
    for (int i = 0; i < W; i++) begin
      slot_valid[i] = in_valid[i];
      for (int j = 0; j < i; j++) if (in_valid[j] && halt[j]) slot_valid[i] = `FALSE;
    end

  // ---- instructions this core cannot run yet: they get a ROB entry with an exception (done at
  //      dispatch, no destination, never in the RS). Raised only if they reach commit, so code on
  //      a mispredicted path never stops the machine. ----
  logic [W-1:0] unsupported, has_dest_eff, csr_ok, amo_ok;
  EXCEPTION [W-1:0] exc;
  always_comb
    for (int i = 0; i < W; i++) begin
      // CSR ops on fflags / frm / fcsr run serialized on the FP unit; any other CSR is unsupported
      csr_ok[i]      = csr_op[i] && `NUM_FU_FP > 0 && in_inst[i][31:20] inside {12'h001, 12'h002, 12'h003};
      // LR / SC / AMO: serialized, run by the LSQ's atomic engine on the store port
      amo_ok[i]      = atomic[i] && `A_EXTENSION != 0;
      unsupported[i] = !((fu_type[i] == FU_ALU && !cond_branch[i] && !uncond_branch[i]) ||
                         (fu_type[i] == FU_BRANCH && (cond_branch[i] || uncond_branch[i])) ||
                         fu_type[i] == FU_MULT || fu_type[i] == FU_DIV ||
                         (fu_type[i] == FU_LOAD  && rd_mem[i] && !wr_mem[i]) ||   // incl. flw
                         (fu_type[i] == FU_STORE && wr_mem[i] && !rd_mem[i]) ||   // incl. fsw
                         fu_type[i] == FU_FP || amo_ok[i]) ||
                       (csr_op[i] && !csr_ok[i]) || (atomic[i] && !amo_ok[i]) ||
                       // no FP unit (NUM_FU_FP = 0): all of F, flw/fsw included, stays unsupported
                       (`NUM_FU_FP == 0 && (fu_type[i] == FU_FP || rs1_fp[i] || rs2_fp[i] || rd_fp[i] || rs3_used[i]));
      exc[i]          = (in_exc[i] != EXC_NONE) ? in_exc[i] :
                        illegal[i] ? EXC_ILLEGAL : unsupported[i] ? EXC_UNSUPPORTED : EXC_NONE;
      has_dest_eff[i] = has_dest[i] && (exc[i] == EXC_NONE);
    end

  // ---- rename control ----
  // Two register classes, each with its own free list, RAT and busy table. Every operand picks
  // its class on its own (fcvt.s.w reads an x register and writes an f register; feq reads f
  // registers and writes an x register): rs1_fp/rs2_fp/rd_fp, and rs3 is always FP.
  logic             can_alloc_w;
  logic [NT-1:0]    int_can_alloc_w, fp_can_alloc_w, serial_pending;
  logic [W-1:0]     dest_valid, int_dest, fp_dest;
  logic [W-1:0][TAG_W-1:0] new_tag, src1_tag, src2_tag, src3_tag, told;
  logic [W-1:0]     src1_bypassed, src2_bypassed, src3_bypassed;
  logic [W-1:0]     src1_free, src2_free, src3_free;
  logic [NT-1:0][W-1:0][TAG_W-1:0] int_new, int_src1, int_src2, int_told;
  logic [NT-1:0][W-1:0][TAG_W-1:0] fp_new, fp_src1, fp_src2, fp_src3, fp_told;
  logic [NT-1:0][W-1:0] int_byp1, int_byp2, fp_byp1, fp_byp2, fp_byp3;
  logic [NT-1:0][W-1:0] int_rdy1, int_rdy2, int_rdy3, fp_rdy1, fp_rdy2, fp_rdy3;
  logic [W-1:0][4:0] rs3;

  // state only: the bundle's thread has W free tags in both lists
  assign can_alloc_w = int_can_alloc_w[in_tid] && fp_can_alloc_w[in_tid];
  assign fire = (|slot_valid) && can_alloc_w && rob_space_ok && rs_space_ok && lq_space_ok && sq_space_ok
                && !flush[in_tid] && !serial_pending[in_tid];

  always_comb
    for (int i = 0; i < W; i++) begin
      rs3[i]        = in_inst[i][31:27];
      // x0 as a destination means none; f0 is a real register
      dest_valid[i] = slot_valid[i] && has_dest_eff[i] && (rd_fp[i] || rd[i] != '0);
      int_dest[i]   = dest_valid[i] && !rd_fp[i];
      fp_dest[i]    = dest_valid[i] && rd_fp[i];
    end

  for (g = 0; g < NT; g++) begin : gen_thread
    logic mine, frees;                 // the bundle / this cycle's commits are this thread's
    assign mine  = fire && in_tid == `TID_W'(g);
    assign frees = free_tid == `TID_W'(g);

    // A serial op (fcsr CSR op, atomic) is the youngest instruction of its thread until it
    // commits: nothing of that thread dispatches after it (fetch ends its bundle and refetches
    // from its pc+4), and its RS entry issues only as the oldest. Every flush clears the block
    // (a serial op on a wrong path).
    always_ff @(posedge clock)
      if (reset || flush[g])                                           serial_pending[g] <= `FALSE;
      else if (mine && |(slot_valid & (csr_ok | amo_ok) & ~unsupported & ~illegal)) serial_pending[g] <= `TRUE;
      else if (rob_empty[g])                                           serial_pending[g] <= `FALSE;

    freelist #(.BASE(g * `THREAD_PHYS_SZ)) freelist_0 (
        .clock      (clock),
        .reset      (reset),
        .alloc_req  (int_dest & {W{mine}}),
        .alloc_tag  (int_new[g]),
        /* verilator lint_off PINCONNECTEMPTY */
        .alloc_ok   (),                  // request-dependent: deliberately unused (see fire)
        /* verilator lint_on PINCONNECTEMPTY */
        .can_alloc_w(int_can_alloc_w[g]),
        .free_en    (free_en & ~free_fp & {W{frees}}),
        .free_tag   (free_tag),
        .flush      (flush[g])
    );

    freelist #(.BASE(`FP_TAG_BASE + g * `FP_THREAD_PHYS_SZ), .FL_SZ(`FP_FREE_LIST_SZ), .ZERO_REG(1'b0)) fp_freelist_0 (
        .clock      (clock),
        .reset      (reset),
        .alloc_req  (fp_dest & {W{mine}}),
        .alloc_tag  (fp_new[g]),
        /* verilator lint_off PINCONNECTEMPTY */
        .alloc_ok   (),
        /* verilator lint_on PINCONNECTEMPTY */
        .can_alloc_w(fp_can_alloc_w[g]),
        .free_en    (free_en & free_fp & {W{frees}}),
        .free_tag   (free_tag),
        .flush      (flush[g])
    );

    /* verilator lint_off PINCONNECTEMPTY */
    rat #(.BASE(g * `THREAD_PHYS_SZ)) rat_0 (
        .clock        (clock),
        .reset        (reset),
        .rename_en    (mine),
        .valid        (slot_valid),
        .rs1          (rs1),
        .rs2          (rs2),
        .rs3          ('0),
        .rd           (rd),
        .has_dest     (has_dest_eff & ~rd_fp),
        .new_tag      (int_new[g]),
        .src1_tag     (int_src1[g]),
        .src2_tag     (int_src2[g]),
        .src3_tag     (),
        .told         (int_told[g]),
        .src1_bypassed(int_byp1[g]),
        .src2_bypassed(int_byp2[g]),
        .src3_bypassed(),
        .recover      (flush[g]),
        .recover_map  (recover_map[g])
    );
    /* verilator lint_on PINCONNECTEMPTY */

    rat #(.BASE(`FP_TAG_BASE + g * `FP_THREAD_PHYS_SZ), .ZERO_REG(1'b0)) fp_rat_0 (
        .clock        (clock),
        .reset        (reset),
        .rename_en    (mine),
        .valid        (slot_valid),
        .rs1          (rs1),
        .rs2          (rs2),
        .rs3          (rs3),
        .rd           (rd),
        .has_dest     (has_dest_eff & rd_fp),
        .new_tag      (fp_new[g]),
        .src1_tag     (fp_src1[g]),
        .src2_tag     (fp_src2[g]),
        .src3_tag     (fp_src3[g]),
        .told         (fp_told[g]),
        .src1_bypassed(fp_byp1[g]),
        .src2_bypassed(fp_byp2[g]),
        .src3_bypassed(fp_byp3[g]),
        .recover      (flush[g]),
        .recover_map  (fp_recover_map[g])
    );

    busy_table #(.BASE(g * `THREAD_PHYS_SZ)) busy_table_0 (
        .clock     (clock),
        .reset     (reset),
        .set_en    (int_dest & {W{mine}}),
        .set_tag   (int_new[g]),
        .cdb_valid (cdb_valid),
        .cdb_tag   (cdb_tag),
        .flush     (flush[g]),
        .src1_tag  (src1_tag),
        .src2_tag  (src2_tag),
        .src3_tag  (src3_tag),
        .src1_ready(int_rdy1[g]),
        .src2_ready(int_rdy2[g]),
        .src3_ready(int_rdy3[g])
    );

    busy_table #(.BASE(`FP_TAG_BASE + g * `FP_THREAD_PHYS_SZ), .BUSY_TABLE_SZ(`FP_THREAD_PHYS_SZ), .ZERO_REG(1'b0))
        fp_busy_table_0 (
        .clock     (clock),
        .reset     (reset),
        .set_en    (fp_dest & {W{mine}}),
        .set_tag   (fp_new[g]),
        .cdb_valid (cdb_valid),
        .cdb_tag   (cdb_tag),
        .flush     (flush[g]),
        .src1_tag  (src1_tag),
        .src2_tag  (src2_tag),
        .src3_tag  (src3_tag),
        .src1_ready(fp_rdy1[g]),
        .src2_ready(fp_rdy2[g]),
        .src3_ready(fp_rdy3[g])
    );
  end

  // per-operand class selection, from the bundle's thread
  always_comb
    for (int i = 0; i < W; i++) begin
      new_tag[i]       = rd_fp[i]  ? fp_new[in_tid][i]  : int_new[in_tid][i];
      told[i]          = rd_fp[i]  ? fp_told[in_tid][i] : int_told[in_tid][i];
      src1_tag[i]      = rs1_fp[i] ? fp_src1[in_tid][i] : int_src1[in_tid][i];
      src1_bypassed[i] = rs1_fp[i] ? fp_byp1[in_tid][i] : int_byp1[in_tid][i];
      src2_tag[i]      = rs2_fp[i] ? fp_src2[in_tid][i] : int_src2[in_tid][i];
      src2_bypassed[i] = rs2_fp[i] ? fp_byp2[in_tid][i] : int_byp2[in_tid][i];
      src3_tag[i]      = fp_src3[in_tid][i];
      src3_bypassed[i] = fp_byp3[in_tid][i];
      // every busy table reads tags outside its range as ready, so the AND picks the right one
      src1_free[i] = `TRUE; src2_free[i] = `TRUE; src3_free[i] = `TRUE;
      for (int t = 0; t < NT; t++) begin
        src1_free[i] &= int_rdy1[t][i] && fp_rdy1[t][i];
        src2_free[i] &= int_rdy2[t][i] && fp_rdy2[t][i];
        src3_free[i] &= int_rdy3[t][i] && fp_rdy3[t][i];
      end
    end

  // ---- ROB and RS entries ----
  always_comb begin
    for (int i = 0; i < W; i++) begin
      disp_valid[i]      = slot_valid[i] && fire;
      rs_disp_valid[i]   = disp_valid[i] && (exc[i] == EXC_NONE);
      disp_exception[i]  = exc[i];
      disp_dest_valid[i] = dest_valid[i];
      disp_rd[i]         = rd[i];
      disp_tag[i]        = new_tag[i];
      disp_told[i]       = told[i];
      disp_pc[i]         = in_pc[i];
      disp_halt[i]       = halt[i];
      disp_is_br[i]      = cond_branch[i] || uncond_branch[i];
      disp_uncond[i]     = uncond_branch[i];
      // RISC-V return-address-stack hints: link registers are x1 and x5
      disp_is_call[i]    = uncond_branch[i] && (rd[i] == 5'd1 || rd[i] == 5'd5);
      disp_is_ret[i]     = uncond_branch[i] && in_inst[i][6:0] == `RV32_JALR_OP && rd[i] == 5'd0
                           && (rs1[i] == 5'd1 || rs1[i] == 5'd5);
      disp_pred_npc[i]   = in_pred_npc[i];
      // (atomics take no LQ / SQ entry)
      disp_is_load[i]    = disp_valid[i] && exc[i] == EXC_NONE && fu_type[i] == FU_LOAD  && !atomic[i];
      disp_is_store[i]   = disp_valid[i] && exc[i] == EXC_NONE && fu_type[i] == FU_STORE && !atomic[i];
      disp_dest_fp[i]    = fp_dest[i];

      disp_pkt[i]            = '0;
      disp_pkt[i].inst       = in_inst[i];
      disp_pkt[i].PC         = in_pc[i];
      disp_pkt[i].NPC        = in_pc[i] + 4;
      disp_pkt[i].src1_tag   = src1_tag[i];
      disp_pkt[i].src1_ready = !rs1_used[i] || (src1_free[i] && !src1_bypassed[i]);
      disp_pkt[i].src2_tag   = src2_tag[i];
      disp_pkt[i].src2_ready = !rs2_used[i] || (src2_free[i] && !src2_bypassed[i]);
      disp_pkt[i].src3_tag   = src3_tag[i];
      disp_pkt[i].src3_ready = !rs3_used[i] || (src3_free[i] && !src3_bypassed[i]);
      disp_pkt[i].serial     = csr_ok[i] || amo_ok[i];
      disp_pkt[i].dest_tag   = dest_valid[i] ? new_tag[i] : '0;
      disp_pkt[i].rob_idx    = rob_idx[i];
      disp_pkt[i].tid        = in_tid;
      disp_pkt[i].lsq_idx    = lsq_idx[i];
      disp_pkt[i].fu_type    = csr_ok[i] ? FU_FP : amo_ok[i] ? FU_STORE : fu_type[i];
      disp_pkt[i].alu_func   = alu_func[i];
      disp_pkt[i].opa_select = opa_select[i];
      disp_pkt[i].opb_select = opb_select[i];
    end
  end

endmodule  // dispatch
