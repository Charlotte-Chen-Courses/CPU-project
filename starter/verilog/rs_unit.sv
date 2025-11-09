`include "verilog/sys_defs.svh"

// RS unit
module rs_unit (
    input clock,
    input reset,
    input RM_RSB_PACKET inst_packet,
    input [4:0] rob_t_index,
    input [4:0] rob_s1_index,
    input [4:0] rob_s2_index,
    input [`XLEN-1:0] input_v1,
    input [`XLEN-1:0] input_v2,
    input v1_ready,
    input v2_ready,

    output logic [4:0] mp_s1_index,
    output logic [4:0] mp_s2_index,
    output logic [`XLEN-1:0] output_v1,
    output logic [`XLEN-1:0] output_v2
);
    // Extract the opcode locally within this block
    logic [6:0] comb_opcode;
    assign comb_opcode = inst_packet.inst.r.opcode;
  
    // RS entries for different functional units
    RS_ENTRY rs_entries[(`RS_NUM_ENTRY-1):0];

    // Initialize entries on reset
    always_ff @(posedge clock or posedge reset) begin
        if (reset) begin
            // Reset all entries
            for (int i = 0; i < `RS_NUM_ENTRY; i++) begin
                rs_entries[i].busy <= `FALSE;
                rs_entries[i].op_code <= FU_FUNC'(0);
                rs_entries[i].rob_target_index <= '0;
                rs_entries[i].rob_source1_index <= '0;
                rs_entries[i].rob_source2_index <= '0;
                rs_entries[i].source1_value <= '0;
                rs_entries[i].source2_value <= '0;
                rs_entries[i].source1_ready <= `FALSE;
                rs_entries[i].source2_ready <= `FALSE;
            end
        end else begin
            // Update the RS entries based on functional unit mark
            case (inst_packet.fu_mark)
                ALU:
                if (!rs_entries[0].busy) begin
                    rs_entries[0].busy <= `TRUE;
                    rs_entries[0].op_code <= inst_packet.fu_func;
                    rs_entries[0].rob_target_index <= rob_t_index;
                    rs_entries[0].rob_source1_index <= rob_s1_index;
                    rs_entries[0].rob_source2_index <= rob_s2_index;
                    rs_entries[0].source1_value <= input_v1;
                    rs_entries[0].source2_value <= input_v2;
                    rs_entries[0].source1_ready <= v1_ready;
                    rs_entries[0].source2_ready <= v2_ready;
                end
                
                LD:
                if (!rs_entries[1].busy) begin
                    rs_entries[1].busy <= `TRUE;
                    rs_entries[1].op_code <= inst_packet.fu_func;
                    rs_entries[1].rob_target_index <= rob_t_index;
                    rs_entries[1].rob_source1_index <= rob_s1_index;
                    rs_entries[1].rob_source2_index <= rob_s2_index;
                    rs_entries[1].source1_value <= input_v1;
                    rs_entries[1].source2_value <= input_v2;
                    rs_entries[1].source1_ready <= v1_ready;
                    rs_entries[1].source2_ready <= v2_ready;
                end
                
                ST:
                if (!rs_entries[2].busy) begin
                    rs_entries[2].busy <= `TRUE;
                    rs_entries[2].op_code <= inst_packet.fu_func;
                    rs_entries[2].rob_target_index <= rob_t_index;
                    rs_entries[2].rob_source1_index <= rob_s1_index;
                    rs_entries[2].rob_source2_index <= rob_s2_index;
                    rs_entries[2].source1_value <= input_v1;
                    rs_entries[2].source2_value <= input_v2;
                    rs_entries[2].source1_ready <= v1_ready;
                    rs_entries[2].source2_ready <= v2_ready;
                end
                
                FP1:
                if (!rs_entries[3].busy) begin
                    rs_entries[3].busy <= `TRUE;
                    rs_entries[3].op_code <= inst_packet.fu_func;
                    rs_entries[3].rob_target_index <= rob_t_index;
                    rs_entries[3].rob_source1_index <= rob_s1_index;
                    rs_entries[3].rob_source2_index <= rob_s2_index;
                    rs_entries[3].source1_value <= input_v1;
                    rs_entries[3].source2_value <= input_v2;
                    rs_entries[3].source1_ready <= v1_ready;
                    rs_entries[3].source2_ready <= v2_ready;
                end
                
                FP2:
                if (!rs_entries[4].busy) begin
                    rs_entries[4].busy <= `TRUE;
                    rs_entries[4].op_code <= inst_packet.fu_func;
                    rs_entries[4].rob_target_index <= rob_t_index;
                    rs_entries[4].rob_source1_index <= rob_s1_index;
                    rs_entries[4].rob_source2_index <= rob_s2_index;
                    rs_entries[4].source1_value <= input_v1;
                    rs_entries[4].source2_value <= input_v2;
                    rs_entries[4].source1_ready <= v1_ready;
                    rs_entries[4].source2_ready <= v2_ready;
                end
                
                default: ;  // No match, do nothing
            endcase
        end
    end

    // Extract source register indices based on instruction type
    always_comb begin
        // Default values
        mp_s1_index = 5'b0;
        mp_s2_index = 5'b0;
        output_v1 = input_v1;
        output_v2 = input_v2;
        
        // Extract source register indices based on instruction type
        case (inst_packet.inst.inst_type)
            R: begin // R-type instruction
                mp_s1_index = inst_packet.inst.r.rs1;
                mp_s2_index = inst_packet.inst.r.rs2;
            end
            
            I: begin // I-type instruction
                mp_s1_index = inst_packet.inst.i.rs1;
                mp_s2_index = 5'b0; // No second source for I-type
            end
            
            S: begin // S-type instruction
                mp_s1_index = inst_packet.inst.s.rs1;
                mp_s2_index = inst_packet.inst.s.rs2;
            end
            
            B: begin // B-type instruction
                mp_s1_index = inst_packet.inst.b.rs1;
                mp_s2_index = inst_packet.inst.b.rs2;
            end
            
            U: begin // U-type instruction (LUI, AUIPC)
                mp_s1_index = 5'b0; // No source register
                mp_s2_index = 5'b0;
            end
            
            J: begin // J-type instruction (JAL)
                mp_s1_index = 5'b0; // No source register
                mp_s2_index = 5'b0;
            end
            
            default: begin
                mp_s1_index = 5'b0;
                mp_s2_index = 5'b0;
            end
        endcase
    end
endmodule