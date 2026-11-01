/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  fp_fu.sv                                            //
//                                                                     //
//  Description :  FP functional unit (RV32F). Wraps CVFPU (fpnew_top, //
//                 vendor/cvfpu, FP32 only) and maps each instruction  //
//                 to its operation group:                             //
//                  - ADDMUL (FP_ADDMUL_REGS pipeline registers):      //
//                    fadd/fsub (operands 1, 2), fmul, fmadd/fmsub/    //
//                    fnmsub/fnmadd (operands 0, 1, 2);                //
//                  - DIVSQRT: fdiv, fsqrt, iterative (T-Head unit);   //
//                  - NONCOMP: fsgnj*, fmin/fmax, feq/flt/fle,         //
//                    fclass, and fmv.x.w / fmv.w.x as a sign-         //
//                    injection passthrough (rm = RUP);                //
//                  - CONV: fcvt.w[u].s, fcvt.s.w[u].                  //
//                 The core supplies each source from the right PRF    //
//                 (int rs1 for fcvt.s.w / fmv.w.x). Rounding: the     //
//                 instruction's rm, or frm (from fcsr) when rm = DYN. //
//                 ready depends only on state: CVFPU's own ready      //
//                 depends on the operation (an add can enter while a  //
//                 divide is busy), so an op it does not take at once  //
//                 waits in a one-entry buffer and ready drops until   //
//                 it enters. Results can leave out of order (groups   //
//                 have different latencies); {dest tag, rob_idx}      //
//                 travel through CVFPU as its tag. The result's       //
//                 exception flags (NV DZ OF UF NX, as fcsr) go out in //
//                 the CDB packet. An op with rm = DYN while frm is    //
//                 invalid (5..7) raises EXC_ILLEGAL (no flags).       //
//                 CSR ops on fflags / frm / fcsr (csrrw/s/c and the   //
//                 immediate forms) also run here, never in CVFPU:     //
//                 they are serial (issued only as the oldest          //
//                 instruction, with nothing younger dispatched), so   //
//                 fcsr is written at issue (fcsr_we) and the old      //
//                 value goes to rd through a result register that     //
//                 yields to CVFPU's output. flush[t] drops thread t's //
//                 work: the input buffer and CSR result by thread; in //
//                 CVFPU (whose flush_i empties everything) one thread //
//                 uses flush_i, several use epochs: each thread's     //
//                 counter steps on its flush, ops carry {tid, epoch}  //
//                 as part of the tag, and a result from an older      //
//                 epoch is drained without going to the CDB.          //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

module fp_fu (
    input              clock,
    input              reset,
    input              in_valid,
    input  RS_PACKET   in_pkt,
    output logic       ready,
    output logic       out_valid,
    output CDB_PACKET  out_pkt,
    input              out_grant,
    input  [`NUM_THREADS-1:0] flush,

    input [`XLEN-1:0]  rs1_value,
    input [`XLEN-1:0]  rs2_value,
    input [`XLEN-1:0]  rs3_value,
    input [7:0]        fcsr,         // in_pkt's thread's {frm, fflags}: frm for rm = DYN, all for CSR ops
    output logic       fcsr_we,      // a CSR op writes fcsr this cycle
    output logic [7:0] fcsr_wdata
);

  logic [2:0] frm;
  assign frm = fcsr[7:5];

  localparam EP_W = 4;                 // epochs: a result outliving 16 flushes of its thread cannot happen

  typedef struct packed {
    logic [`TAG_W-1:0]     tag;
    logic [`ROB_IDX_W-1:0] rob_idx;
    logic [`TID_W-1:0]     tid;
    logic [EP_W-1:0]       epoch;      // the thread's epoch when the op entered
    logic                  illegal;    // rm = DYN with an invalid frm
  } FP_TAG;

  logic [`NUM_THREADS-1:0][EP_W-1:0] epoch;
  always_ff @(posedge clock)
    for (int t = 0; t < `NUM_THREADS; t++)
      if (reset)         epoch[t] <= '0;
      else if (flush[t]) epoch[t] <= epoch[t] + 1'b1;

  // one operation as CVFPU takes it
  typedef struct packed {
    logic [2:0][31:0]      operands;
    fpnew_pkg::roundmode_e rnd;
    fpnew_pkg::operation_e op;
    logic                  op_mod;
    FP_TAG                 tag;
  } FP_REQ;

  // ---------------- decode ----------------
  FP_REQ req_in;
  always_comb begin
    automatic logic [6:0] opcode = in_pkt.inst[6:0];
    automatic logic [4:0] funct5 = in_pkt.inst[31:27];
    automatic logic [2:0] rm     = in_pkt.inst[14:12];
    automatic fpnew_pkg::roundmode_e arith_rm = fpnew_pkg::roundmode_e'((rm == 3'b111) ? frm : rm);
    automatic logic       bad_dyn  = (rm == 3'b111) && (frm > 3'd4);   // for ops that round
    req_in.operands = {rs3_value, rs2_value, rs1_value};    // operand 0 = rs1
    req_in.rnd      = arith_rm;
    req_in.op       = fpnew_pkg::ADD;
    req_in.op_mod   = 1'b0;
    req_in.tag      = '{tag: in_pkt.dest_tag, rob_idx: in_pkt.rob_idx, tid: in_pkt.tid, epoch: epoch[in_pkt.tid],
                        illegal: bad_dyn};
    case (opcode)
      7'b1000011: req_in.op = fpnew_pkg::FMADD;                                    // fmadd
      7'b1000111: begin req_in.op = fpnew_pkg::FMADD;  req_in.op_mod = 1'b1; end   // fmsub
      7'b1001011: req_in.op = fpnew_pkg::FNMSUB;                                   // fnmsub
      7'b1001111: begin req_in.op = fpnew_pkg::FNMSUB; req_in.op_mod = 1'b1; end   // fnmadd
      default: case (funct5)                                                       // OP-FP
        5'b00000: req_in.operands = {rs2_value, rs1_value, 32'b0};                 // fadd: op[1] + op[2]
        5'b00001: begin req_in.operands = {rs2_value, rs1_value, 32'b0}; req_in.op_mod = 1'b1; end  // fsub
        5'b00010: req_in.op = fpnew_pkg::MUL;
        5'b00011: req_in.op = fpnew_pkg::DIV;
        5'b01011: req_in.op = fpnew_pkg::SQRT;
        5'b00100: begin req_in.op = fpnew_pkg::SGNJ;   req_in.rnd = fpnew_pkg::roundmode_e'(rm); req_in.tag.illegal = 1'b0; end
        5'b00101: begin req_in.op = fpnew_pkg::MINMAX; req_in.rnd = fpnew_pkg::roundmode_e'(rm); req_in.tag.illegal = 1'b0; end
        5'b10100: begin req_in.op = fpnew_pkg::CMP;    req_in.rnd = fpnew_pkg::roundmode_e'(rm); req_in.tag.illegal = 1'b0; end
        5'b11000: begin req_in.op = fpnew_pkg::F2I;    req_in.op_mod = in_pkt.inst[20]; end
        5'b11010: begin req_in.op = fpnew_pkg::I2F;    req_in.op_mod = in_pkt.inst[20]; end
        5'b11100: begin                                                            // fmv.x.w / fclass
          req_in.tag.illegal = 1'b0;
          if (rm == 3'b001) req_in.op = fpnew_pkg::CLASSIFY;
          else begin req_in.op = fpnew_pkg::SGNJ; req_in.rnd = fpnew_pkg::RUP; req_in.op_mod = 1'b1; end
        end
        default: begin req_in.op = fpnew_pkg::SGNJ; req_in.rnd = fpnew_pkg::RUP; req_in.tag.illegal = 1'b0; end  // fmv.w.x
      endcase
    endcase
  end

  // ---------------- CSR ops (fflags 0x001, frm 0x002, fcsr 0x003) ----------------
  logic        in_csr, csr_v, fpu_out_valid, fpu_live, stale;
  logic [7:0]  csr_old, csr_src, csr_new;
  CDB_PACKET   csr_pkt;
  assign in_csr = in_valid && (in_pkt.inst[6:0] == 7'b1110011);
  always_comb begin
    automatic logic [11:0] addr = in_pkt.inst[31:20];
    csr_old = (addr == 12'h001) ? {3'b0, fcsr[4:0]} : (addr == 12'h002) ? {5'b0, fcsr[7:5]} : fcsr;
    csr_src = in_pkt.inst[14] ? {3'b0, in_pkt.inst[19:15]} : rs1_value[7:0];   // uimm or rs1
    case (in_pkt.inst[13:12])
      2'b01:   csr_new = csr_src;               // csrrw
      2'b10:   csr_new = csr_old | csr_src;     // csrrs
      default: csr_new = csr_old & ~csr_src;    // csrrc
    endcase
    fcsr_wdata = (addr == 12'h001) ? {fcsr[7:5], csr_new[4:0]} :
                 (addr == 12'h002) ? {csr_new[2:0], fcsr[4:0]} : csr_new;
  end
  assign fcsr_we = in_csr;

  always_ff @(posedge clock) begin
    if (reset) csr_v <= `FALSE;
    else if (in_csr && !flush[in_pkt.tid]) begin
      csr_v   <= `TRUE;
      csr_pkt <= '{tag: in_pkt.dest_tag, value: {24'b0, csr_old}, rob_idx: in_pkt.rob_idx, tid: in_pkt.tid,
                   branch: `FALSE, next_pc: '0, exception: EXC_NONE, fflags: '0};
    end else if (csr_v && flush[csr_pkt.tid]) csr_v <= `FALSE;
    else if (csr_v && out_grant && !fpu_live) csr_v <= `FALSE;
  end

  // ---------------- input buffer ----------------
  FP_REQ buf_req, req;
  logic  buf_v, req_valid, fpu_ready;

  assign ready     = !buf_v && !csr_v;
  assign req_valid = buf_v || (in_valid && !in_csr);
  assign req       = buf_v ? buf_req : req_in;

  always_ff @(posedge clock) begin
    // An op CVFPU has seen must stay until it takes it (valid/ready): one thread can drop it on a
    // flush, since flush_i empties CVFPU in the same cycle; with several threads it stays, and the
    // epoch check drops its result (withdrawing it mid-handshake left the iterative div/sqrt unit
    // computing a ghost whose result came out for the next op).
    if (reset)                                                           buf_v <= `FALSE;
    else if (buf_v && `NUM_THREADS == 1 && flush[buf_req.tag.tid])       buf_v <= `FALSE;
    else if (buf_v && fpu_ready)                                         buf_v <= `FALSE;
    else if (!buf_v && in_valid && !in_csr && !fpu_ready && (`NUM_THREADS > 1 || !flush[in_pkt.tid])) begin
      buf_v <= `TRUE; buf_req <= req_in;
    end
  end

  // ---------------- CVFPU ----------------
  localparam fpnew_pkg::fpu_implementation_t IMPL = '{
    PipeRegs:   '{'{default: `FP_ADDMUL_REGS},     // ADDMUL
                  '{default: 0},                   // DIVSQRT (iterative)
                  '{default: `FP_NONCOMP_REGS},    // NONCOMP
                  '{default: `FP_CONV_REGS}},      // CONV
    UnitTypes:  '{'{default: fpnew_pkg::PARALLEL},
                  '{default: fpnew_pkg::MERGED},
                  '{default: fpnew_pkg::PARALLEL},
                  '{default: fpnew_pkg::MERGED}},
    PipeConfig: fpnew_pkg::DISTRIBUTED
  };

  FP_TAG              out_tag;
  logic [31:0]        result;
  fpnew_pkg::status_t status;

  /* verilator lint_off PINCONNECTEMPTY */
  fpnew_top #(
      .Features      (fpnew_pkg::RV32F),
      .Implementation(IMPL),
      .PulpDivsqrt   (1'b0),            // T-Head FP32 div/sqrt
      .TagType       (FP_TAG)
  ) fpnew_0 (
      .clk_i         (clock),
      .rst_ni        (!reset),
      .operands_i    (req.operands),
      .rnd_mode_i    (req.rnd),
      .op_i          (req.op),
      .op_mod_i      (req.op_mod),
      .src_fmt_i     (fpnew_pkg::FP32),
      .dst_fmt_i     (fpnew_pkg::FP32),
      .int_fmt_i     (fpnew_pkg::INT32),
      .vectorial_op_i(1'b0),
      .tag_i         (req.tag),
      .simd_mask_i   (1'b1),
      .in_valid_i    (req_valid),
      .in_ready_o    (fpu_ready),
      .flush_i       (`NUM_THREADS == 1 ? flush[0] : 1'b0),
      .result_o      (result),
      .status_o      (status),
      .tag_o         (out_tag),
      .out_valid_o   (fpu_out_valid),
      .out_ready_i   (out_grant || stale),
      .busy_o        ()
  );
  /* verilator lint_on PINCONNECTEMPTY */

  // a result from before its thread's last flush is drained, never sent (one thread: flush_i did it)
  assign stale    = `NUM_THREADS > 1 && fpu_out_valid && out_tag.epoch != epoch[out_tag.tid];
  assign fpu_live = fpu_out_valid && !stale;

  // CVFPU's result first; a CSR result waits behind it (CVFPU is idle by then anyway)
  assign out_valid = fpu_live || csr_v;
  assign out_pkt   = fpu_live ? '{tag: out_tag.tag, value: result, rob_idx: out_tag.rob_idx, tid: out_tag.tid,
                                       branch: `FALSE, next_pc: '0,
                                       exception: out_tag.illegal ? EXC_ILLEGAL : EXC_NONE,
                                       fflags: out_tag.illegal ? 5'b0 : status}
                                   : csr_pkt;

endmodule  // fp_fu
