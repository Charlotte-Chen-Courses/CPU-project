/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  stage_rsb.sv                                        //
//                                                                     //
//  Description :  instruction reservation station (RS) and reorder    //
//                 buffer (ROB) stage of the pipeline;                 //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"


// RS unit
module rs_unit (
    input RM_RSB_PACKET inst_packet,
    input [4:0] rob_t_index,
    input [4:0] rob_s1_index,  // index of this first (left) reg of this inst in ROB (find in mp)
    input [4:0] rob_s2_index,  // index of this second (right) reg of this inst in ROB (find in mp)
    input [`XLEN-1:0] input_v1,  // value in the first (left) reg of this inst
    input [`XLEN-1:0] input_v2,  // value in the second (right) reg of this inst


    input [4:0] rob_t_index,  // get tail position in rob

    input v1_ready,  //mp feedback on v1 status
    input v2_ready,  //mp feedback on v2 status


    output [4:0] mp_s1_index,  // search for t index in map table
    output [4:0] mp_s2_index,  // search for t index in map table

    output [`XLEN-1:0] output_v1,
    output [`XLEN-1:0] output_v2
);
  RS_ENTRY rs_entries[(RS_NUM_ENTRY-1):0];




  // setup the search regs for mp
  always_comb begin
    case (inst_packet.inst.inst_type)
      R: begin
        mp_s1_index = inst_packet.inst.r1;
        mp_s2_index = inst_packet.inst.r2;
      end
      I: begin
        mp_s1_index = inst_packet.inst.r1;
      end
      S: begin
        mp_s1_index = inst_packet.inst.r1;
        mp_s2_index = inst_packet.inst.r2;
      end
      B: begin
        mp_s1_index = inst_packet.inst.r1;
        mp_s2_index = inst_packet.inst.r2;
      end
    endcase
  end


  // setup rs table
  always_comb begin
    case (inst_packet.fu_mark)
      ALU:
      if (!rs_entries[0].busy) begin
        rs_entries[0].busy              = TRUE;
        rs_entries[0].op_code           = inst_packet.fu_func;
        rs_entries[0].rob_target_index  = rob_t_index;
        rs_entries[0].rob_source1_index = mp_s1_index;
        rs_entries[0].rob_source2_index = mp_s2_index;
        rs_entries[0].source1_value     = inst_packet.rs1_value;
        rs_entries[0].source2_value     = inst_packet.rs2_value;
        rs_entries[0].source1_ready     = v1_ready;
        rs_entries[0].source2_ready     = v2_ready;
      end
      LD:
      if (!rs_entries[1].busy) begin
        rs_entries[1].busy              = TRUE;
        rs_entries[1].op_code           = inst_packet.fu_func;
        rs_entries[0].rob_target_index  = rob_t_index;
        rs_entries[0].rob_source1_index = mp_s1_index;
        rs_entries[0].rob_source2_index = mp_s2_index;
        rs_entries[0].source1_value     = inst_packet.rs1_value;
        rs_entries[0].source2_value     = inst_packet.rs2_value;
        rs_entries[0].source1_ready     = v1_ready;
        rs_entries[0].source2_ready     = v2_ready;
      end
      ST:
      if (!rs_entries[2].busy) begin
        rs_entries[2].busy              = TRUE;
        rs_entries[2].op_code           = inst_packet.fu_func;
        rs_entries[0].rob_target_index  = rob_t_index;
        rs_entries[0].rob_source1_index = mp_s1_index;
        rs_entries[0].rob_source2_index = mp_s2_index;
        rs_entries[0].source1_value     = inst_packet.rs1_value;
        rs_entries[0].source2_value     = inst_packet.rs2_value;
        rs_entries[0].source1_ready     = v1_ready;
        rs_entries[0].source2_ready     = v2_ready;
      end
      FP1:
      if (!rs_entries[3].busy) begin
        rs_entries[3].busy              = TRUE;
        rs_entries[3].op_code           = inst_packet.fu_func;
        rs_entries[0].rob_target_index  = rob_t_index;
        rs_entries[0].rob_source1_index = mp_s1_index;
        rs_entries[0].rob_source2_index = mp_s2_index;
        rs_entries[0].source1_value     = inst_packet.rs1_value;
        rs_entries[0].source2_value     = inst_packet.rs2_value;
        rs_entries[0].source1_ready     = v1_ready;
        rs_entries[0].source2_ready     = v2_ready;
      end
      FP2:
      if (!rs_entries[4].busy) begin
        rs_entries[4].busy              = TRUE;
        rs_entries[4].op_code           = inst_packet.fu_func;
        rs_entries[0].rob_target_index  = rob_t_index;
        rs_entries[0].rob_source1_index = mp_s1_index;
        rs_entries[0].rob_source2_index = mp_s2_index;
        rs_entries[0].source1_value     = inst_packet.rs1_value;
        rs_entries[0].source2_value     = inst_packet.rs2_value;
        rs_entries[0].source1_ready     = v1_ready;
        rs_entries[0].source2_ready     = v2_ready;
      end
      default: ;  // No match, do nothing
    endcase
  end


endmodule



// ROB unit
module rob_unit ();


endmodule


module stage_rsb (
    input               clock,      // system clock
    input               reset,      // system reset
    input RM_RSB_PACKET rm_rsb_reg,

    output RSB_EX_PACKET rsb_packet
);






  // Instantiate the RS unit file
  rs_unit rs_unit_0 ();

  // Instantiate the ROB unit file
  rob_unit rob_unit_0 ();


endmodule  // stage_rsb
