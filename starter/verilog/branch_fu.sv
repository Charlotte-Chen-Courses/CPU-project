/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  branch_fu.sv                                        //
//                                                                     //
//  Description :  Branch functional unit: conditional branches, jal,  //
//                 jalr. Computes the actual next PC (taken ? target : //
//                 PC+4) and, for jal/jalr, the link value PC+4 (the   //
//                 RS packet's dest_tag is p0 for conditional          //
//                 branches). Targets: branch/jal = PC + imm,          //
//                 jalr = (rs1 + imm) with bit 0 cleared. The result   //
//                 goes on the CDB with branch = 1; the ROB compares   //
//                 next_pc with the predicted next PC at commit. A     //
//                 misaligned taken target sets EXC_MISALIGNED (the ROB//
//                 raises the exception only if it commits). Same      //
//                 interface as alu_fu: one-entry result register held //
//                 until out_grant; flush drops it.                    //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"
`include "verilog/ISA.svh"

module branch_fu (
    input             clock,
    input             reset,
    input             in_valid,
    input  RS_PACKET  in_pkt,     // issued instruction (from the RS)
    output logic      ready,
    output logic      out_valid,
    output CDB_PACKET out_pkt,
    input             out_grant,
    input  [`NUM_THREADS-1:0] flush,  // mispredict in thread t: drop its results (squashed instrs)

    input [`XLEN-1:0] rs1_value,  // operands read from the PRF
    input [`XLEN-1:0] rs2_value
);

  logic [6:0]       opcode;
  logic [2:0]       funct3;
  logic             taken;
  logic [`XLEN-1:0] target, next_pc, link;

  assign opcode = in_pkt.inst[6:0];
  assign funct3 = in_pkt.inst[14:12];
  assign link   = in_pkt.PC + 4;

  always_comb begin
    // condition
    case (funct3)
      3'b000:  taken = (rs1_value == rs2_value);                    // beq
      3'b001:  taken = (rs1_value != rs2_value);                    // bne
      3'b100:  taken = ($signed(rs1_value) <  $signed(rs2_value));  // blt
      3'b101:  taken = ($signed(rs1_value) >= $signed(rs2_value));  // bge
      3'b110:  taken = (rs1_value <  rs2_value);                    // bltu
      3'b111:  taken = (rs1_value >= rs2_value);                    // bgeu
      default: taken = `FALSE;
    endcase
    target = in_pkt.PC + `RV32_signext_Bimm(in_pkt.inst);

    // unconditional jumps
    if (opcode == `RV32_JAL_OP) begin
      taken  = `TRUE;
      target = in_pkt.PC + `RV32_signext_Jimm(in_pkt.inst);
    end else if (opcode == `RV32_JALR_OP) begin
      taken  = `TRUE;
      target = (rs1_value + `RV32_signext_Iimm(in_pkt.inst)) & ~`XLEN'(1);
    end

    next_pc = taken ? target : link;
  end

  // can accept a new instruction if the result register is empty, or is being emptied this cycle
  assign ready = !out_valid || out_grant;

  always_ff @(posedge clock) begin
    if (reset) out_valid <= `FALSE;
    else if (ready) begin
      out_valid <= in_valid && !flush[in_pkt.tid];
      if (in_valid)
        out_pkt <= '{tag: in_pkt.dest_tag, value: link, rob_idx: in_pkt.rob_idx, tid: in_pkt.tid,
                     branch: `TRUE, next_pc: next_pc,
                     exception: (next_pc[1:0] != 2'b00) ? EXC_MISALIGNED : EXC_NONE,   // no C extension
                     fflags: '0};
    end
    // else: holding a result that lost arbitration — keep it, unless its thread flushes
    else if (flush[out_pkt.tid]) out_valid <= `FALSE;
  end

endmodule  // branch_fu
