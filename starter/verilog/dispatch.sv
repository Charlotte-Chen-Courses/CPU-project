/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  dispatch.sv                                         //
//                                                                     //
//  Description :  Decode + rename + dispatch for one fetch bundle.    //
//                 Contains the decoders, freelist, speculative RAT    //
//                 and busy table, and produces the ROB and RS entries //
//                 for the bundle. Rules:                              //
//                  - fire = any valid && freelist can_alloc_w &&      //
//                    rob_space_ok && rs_space_ok && LQ/SQ space. All  //
//                    state-only (never the request-dependent          //
//                    alloc_ok), so there is no combinational loop.    //
//                    The bundle renames whole or not at all.          //
//                  - dest_valid = valid && has_dest && rd != x0.      //
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
//                  - flush (mispredict at commit): no dispatch; the   //
//                    freelist rewinds, the RAT loads recover_map, the //
//                    busy table clears.                               //
//                 Anything else (illegal, or no unit yet: FP, CSR, A) //
//                 gets a ROB entry that is done at                    //
//                 dispatch with an exception and no destination, and  //
//                 never enters the RS; the ROB raises it at commit.   //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"
`include "verilog/ISA.svh"

module dispatch #(
    localparam W         = `SUPERSCALAR_WIDTH,
    localparam TAG_W     = `TAG_W,
    localparam ROB_IDX_W = `ROB_IDX_W,
    localparam CDB_SZ    = `NUM_CDB
) (
    input clock,
    input reset,

    // fetch bundle
    input        [W-1:0]                  in_valid,
    input  INST  [W-1:0]                  in_inst,
    input        [W-1:0][`XLEN-1:0]       in_pc,
    input  EXCEPTION [W-1:0]              in_exc,        // fetch exception (PC out of memory range)
    input        [W-1:0][`XLEN-1:0]       in_pred_npc,   // next PC fetch predicted for each slot
    output logic                          fire,          // bundle accepted: fetch may advance

    // back-pressure (state-only signals from the ROB and RS)
    input                                 rob_space_ok,
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
    output logic [W-1:0][`XLEN-1:0]       disp_pred_npc,

    // to the RS: only slots that have a functional unit to run on
    output logic [W-1:0]                  rs_disp_valid,
    output RS_PACKET [W-1:0]              disp_pkt,

    // CDB: clears busy bits
    input        [CDB_SZ-1:0]             cdb_valid,
    input        [CDB_SZ-1:0][TAG_W-1:0]  cdb_tag,

    // commit: frees old physical registers
    input        [W-1:0]                  free_en,
    input        [W-1:0][TAG_W-1:0]       free_tag,

    // mispredict recovery (this cycle): no dispatch; restore rename state
    input                                 flush,
    input        [`ARCH_REG_SZ-1:0][TAG_W-1:0] recover_map   // retirement map after this cycle's commits
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
  logic [W-1:0] unsupported, has_dest_eff;
  EXCEPTION [W-1:0] exc;
  always_comb
    for (int i = 0; i < W; i++) begin
      unsupported[i] = !((fu_type[i] == FU_ALU && !cond_branch[i] && !uncond_branch[i]) ||
                         (fu_type[i] == FU_BRANCH && (cond_branch[i] || uncond_branch[i])) ||
                         fu_type[i] == FU_MULT || fu_type[i] == FU_DIV ||
                         (fu_type[i] == FU_LOAD  && rd_mem[i] && !wr_mem[i]) ||
                         (fu_type[i] == FU_STORE && wr_mem[i] && !rd_mem[i])) ||
                       csr_op[i] || atomic[i] ||
                       rs1_fp[i] || rs2_fp[i] || rd_fp[i] || rs3_used[i];
      exc[i]          = (in_exc[i] != EXC_NONE) ? in_exc[i] :
                        illegal[i] ? EXC_ILLEGAL : unsupported[i] ? EXC_UNSUPPORTED : EXC_NONE;
      has_dest_eff[i] = has_dest[i] && (exc[i] == EXC_NONE);
    end

  // ---- rename control ----
  logic             can_alloc_w;
  logic [W-1:0]     dest_valid;
  logic [W-1:0][TAG_W-1:0] new_tag, src1_tag, src2_tag, told;
  logic [W-1:0]     src1_bypassed, src2_bypassed, src1_free, src2_free;

  assign fire = (|slot_valid) && can_alloc_w && rob_space_ok && rs_space_ok && lq_space_ok && sq_space_ok
                && !flush;

  always_comb
    for (int i = 0; i < W; i++) dest_valid[i] = slot_valid[i] && has_dest_eff[i] && (rd[i] != '0);

  freelist freelist_0 (
      .clock      (clock),
      .reset      (reset),
      .alloc_req  (dest_valid & {W{fire}}),
      .alloc_tag  (new_tag),
      /* verilator lint_off PINCONNECTEMPTY */
      .alloc_ok   (),                  // request-dependent: deliberately unused (see fire)
      /* verilator lint_on PINCONNECTEMPTY */
      .can_alloc_w(can_alloc_w),
      .free_en    (free_en),
      .free_tag   (free_tag),
      .flush      (flush)
  );

  rat rat_0 (
      .clock        (clock),
      .reset        (reset),
      .rename_en    (fire),
      .valid        (slot_valid),
      .rs1          (rs1),
      .rs2          (rs2),
      .rd           (rd),
      .has_dest     (has_dest_eff),
      .new_tag      (new_tag),
      .src1_tag     (src1_tag),
      .src2_tag     (src2_tag),
      .told         (told),
      .src1_bypassed(src1_bypassed),
      .src2_bypassed(src2_bypassed),
      .recover      (flush),
      .recover_map  (recover_map)
  );

  busy_table busy_table_0 (
      .clock     (clock),
      .reset     (reset),
      .set_en    (dest_valid & {W{fire}}),
      .set_tag   (new_tag),
      .cdb_valid (cdb_valid),
      .cdb_tag   (cdb_tag),
      .flush     (flush),
      .src1_tag  (src1_tag),
      .src2_tag  (src2_tag),
      .src1_ready(src1_free),
      .src2_ready(src2_free)
  );

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
      disp_is_load[i]    = disp_valid[i] && exc[i] == EXC_NONE && fu_type[i] == FU_LOAD;
      disp_is_store[i]   = disp_valid[i] && exc[i] == EXC_NONE && fu_type[i] == FU_STORE;

      disp_pkt[i]            = '0;
      disp_pkt[i].inst       = in_inst[i];
      disp_pkt[i].PC         = in_pc[i];
      disp_pkt[i].NPC        = in_pc[i] + 4;
      disp_pkt[i].src1_tag   = src1_tag[i];
      disp_pkt[i].src1_ready = !rs1_used[i] || (src1_free[i] && !src1_bypassed[i]);
      disp_pkt[i].src2_tag   = src2_tag[i];
      disp_pkt[i].src2_ready = !rs2_used[i] || (src2_free[i] && !src2_bypassed[i]);
      disp_pkt[i].dest_tag   = dest_valid[i] ? new_tag[i] : '0;
      disp_pkt[i].rob_idx    = rob_idx[i];
      disp_pkt[i].lsq_idx    = lsq_idx[i];
      disp_pkt[i].fu_type    = fu_type[i];
      disp_pkt[i].alu_func   = alu_func[i];
      disp_pkt[i].opa_select = opa_select[i];
      disp_pkt[i].opb_select = opb_select[i];
    end
  end

endmodule  // dispatch
