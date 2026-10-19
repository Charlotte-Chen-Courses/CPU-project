/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  alu_fu.sv                                           //
//                                                                     //
//  Description :  ALU functional unit. The ALU and operand muxes are  //
//                 copied from the P3 stage_ex.sv (the multiply cases  //
//                 are removed: mul goes to the pipelined MULT unit).  //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"
`include "verilog/ISA.svh"

// ALU: computes the result of FUNC applied with operands A and B
// This module is purely combinational
module alu (
    input          [`XLEN-1:0] opa,
    input          [`XLEN-1:0] opb,
          ALU_FUNC             func,

    output logic [`XLEN-1:0] result
);

  logic signed [`XLEN-1:0] signed_opa, signed_opb;

  assign signed_opa = opa;
  assign signed_opb = opb;

  always_comb begin
    case (func)
      ALU_ADD:  result = opa + opb;
      ALU_SUB:  result = opa - opb;
      ALU_AND:  result = opa & opb;
      ALU_SLT:  result = signed_opa < signed_opb;
      ALU_SLTU: result = opa < opb;
      ALU_OR:   result = opa | opb;
      ALU_XOR:  result = opa ^ opb;
      ALU_SRL:  result = opa >> opb[4:0];
      ALU_SLL:  result = opa << opb[4:0];
      ALU_SRA:  result = signed_opa >>> opb[4:0];  // arithmetic from logical shift

      default: result = `XLEN'hfacebeec;  // here to prevent latches
    endcase
  end

endmodule  // alu


module alu_fu (
    input             clock,
    reset,
    input             in_valid,
    input  RS_PACKET  in_pkt,     // issued instruction (from the RS)
    output logic      ready,
    output logic      out_valid,
    output CDB_PACKET out_pkt,
    input             out_grant,

    input [`XLEN-1:0] rs1_value,  // operands read from the PRF
    input [`XLEN-1:0] rs2_value


    // TODO: in_valid / ready, result register + out_valid / out_pkt / out_grant
);

  logic [`XLEN-1:0] opa_mux_out, opb_mux_out, alu_result;

  // can accept a new instruction if the result register is empty, or is being emptied this cycle
  assign ready = !out_valid || out_grant;

  // ALU opA mux
  always_comb begin
    case (in_pkt.opa_select)
      OPA_IS_RS1:  opa_mux_out = rs1_value;
      OPA_IS_NPC:  opa_mux_out = in_pkt.NPC;
      OPA_IS_PC:   opa_mux_out = in_pkt.PC;
      OPA_IS_ZERO: opa_mux_out = 0;
      default:     opa_mux_out = `XLEN'hdeadface;  // dead face
    endcase
  end

  // ALU opB mux
  always_comb begin
    case (in_pkt.opb_select)
      OPB_IS_RS2:   opb_mux_out = rs2_value;
      OPB_IS_I_IMM: opb_mux_out = `RV32_signext_Iimm(in_pkt.inst);
      OPB_IS_S_IMM: opb_mux_out = `RV32_signext_Simm(in_pkt.inst);
      OPB_IS_B_IMM: opb_mux_out = `RV32_signext_Bimm(in_pkt.inst);
      OPB_IS_U_IMM: opb_mux_out = `RV32_signext_Uimm(in_pkt.inst);
      OPB_IS_J_IMM: opb_mux_out = `RV32_signext_Jimm(in_pkt.inst);
      default:      opb_mux_out = `XLEN'hfacefeed;  // face feed
    endcase
  end

  // Instantiate the ALU
  alu alu_0 (
      // Inputs
      .opa(opa_mux_out),
      .opb(opb_mux_out),
      .func(in_pkt.alu_func),
      // Output
      .result(alu_result)
  );


  always_ff @(posedge clock) begin
    if (reset) out_valid <= `FALSE;
    else if (ready) begin
      out_valid <= in_valid;
      if (in_valid)
        out_pkt <= '{tag: in_pkt.dest_tag, value: alu_result, rob_idx: in_pkt.rob_idx};
    end
    // else: holding a result that lost arbitration — keep it
  end

endmodule  // alu_fu
