/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  div_fu.sv                                           //
//                                                                     //
//  Description :  Divide functional unit: div, divu, rem, remu.       //
//                 Iterative restoring divider, one quotient bit per   //
//                 cycle, one operation at a time (ready = 0 while     //
//                 busy). Signed ops divide the magnitudes and fix the //
//                 signs at the end: quotient negative if the operand  //
//                 signs differ, remainder takes the dividend's sign.  //
//                 RISC-V special cases: x/0 = all ones (div and divu),//
//                 x%0 = x; (-2^31)/(-1) = -2^31 with remainder 0      //
//                 (both fall out of the algorithm, except signed     //
//                 x/0, which is forced to all ones).                  //
//                 Latency: 1 cycle to load, 32 iterations, 1 to       //
//                 finish = 34 cycles to out_valid. The result is held //
//                 until out_grant; flush abandons the operation.      //
//                 Same interface as alu_fu.                           //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

module div_fu (
    input             clock,
    input             reset,
    input             in_valid,
    input  RS_PACKET  in_pkt,
    output logic      ready,
    output logic      out_valid,
    output CDB_PACKET out_pkt,
    input             out_grant,
    input  [`NUM_THREADS-1:0] flush,

    input [`XLEN-1:0] rs1_value,
    input [`XLEN-1:0] rs2_value
);

  logic                  busy;
  logic [5:0]            cnt;          // iterations left
  logic [31:0]           rem;          // partial remainder (always < divisor)
  logic [31:0]           quo;          // dividend shifting out, quotient shifting in
  logic [31:0]           dvs;          // divisor magnitude
  logic                  want_rem, neg_q, neg_r, div_zero;
  logic [`TAG_W-1:0]     tag;
  logic [`ROB_IDX_W-1:0] rob_idx;
  logic [`TID_W-1:0]     tid;

  // one restoring step
  logic [32:0] rem_sh;
  logic        fits;
  assign rem_sh = {rem, quo[31]};
  assign fits   = (rem_sh >= {1'b0, dvs});

  // operand setup
  logic        is_signed;
  logic [31:0] mag_a, mag_b;
  assign is_signed = (in_pkt.alu_func == ALU_DIV) || (in_pkt.alu_func == ALU_REM);
  assign mag_a     = (is_signed && rs1_value[31]) ? -rs1_value : rs1_value;
  assign mag_b     = (is_signed && rs2_value[31]) ? -rs2_value : rs2_value;

  // result (from the final quotient / remainder)
  logic [31:0] q_fix, r_fix;
  assign q_fix = div_zero ? 32'hffff_ffff : (neg_q ? -quo : quo);
  assign r_fix = neg_r ? -rem : rem;

  assign ready = !busy && (!out_valid || out_grant);

  always_ff @(posedge clock) begin
    if (reset) begin
      busy      <= `FALSE;
      out_valid <= `FALSE;
    end else begin
      if (out_valid && out_grant) out_valid <= `FALSE;
      if (busy) begin
        if (cnt != 0) begin
          rem <= fits ? 32'(rem_sh - {1'b0, dvs}) : rem_sh[31:0];
          quo <= {quo[30:0], fits};
          cnt <= cnt - 6'd1;
        end else begin
          // out_valid is always 0 here: a new op only starts once the previous result has
          // left (ready), so a finished result never has to wait for the output register
          out_valid <= `TRUE;
          out_pkt   <= '{tag: tag, value: want_rem ? r_fix : q_fix, rob_idx: rob_idx, tid: tid,
                         branch: `FALSE, next_pc: '0, exception: EXC_NONE, fflags: '0};
          busy      <= `FALSE;
        end
      end else if (in_valid && ready && !flush[in_pkt.tid]) begin   // never start over a result still waiting for the CDB
        busy     <= `TRUE;
        cnt      <= 6'd32;
        rem      <= '0;
        quo      <= mag_a;
        dvs      <= mag_b;
        want_rem <= (in_pkt.alu_func == ALU_REM) || (in_pkt.alu_func == ALU_REMU);
        neg_q    <= is_signed && (rs1_value[31] ^ rs2_value[31]);
        neg_r    <= is_signed && rs1_value[31];
        div_zero <= (rs2_value == '0);
        tag      <= in_pkt.dest_tag;
        rob_idx  <= in_pkt.rob_idx;
        tid      <= in_pkt.tid;
      end
      // a flush of the operation's thread abandons it (and the result it would finish this cycle);
      // one of the waiting result's thread drops that (busy and out_valid are never both set)
      if (busy && flush[tid]) begin
        busy      <= `FALSE;
        out_valid <= `FALSE;
      end
      if (out_valid && flush[out_pkt.tid]) out_valid <= `FALSE;
    end
  end

  // synopsys translate_off
  always_ff @(posedge clock)
    if (!reset && busy) assert (!out_valid) else $error("div_fu: busy while still holding a result");
  // synopsys translate_on

endmodule  // div_fu
