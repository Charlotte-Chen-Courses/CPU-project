/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  stage_if.sv                                         //
//                                                                     //
//  Description :  instruction fetch (IF) stage of the pipeline;       //
//                 fetch instruction, compute next PC location, and    //
//                 send them down the pipeline. This is only a simple  //
//                 copy of project 3 and is subject to changes.        //
/////////////////////////////////////////////////////////////////////////

module stage_if (
    input                       clock,
    input                       reset,
    input [63:0]                Imem2proc_data,      // data coming back from Instruction memory

    output IF_DIS_PACKET        if_packet,
    output logic [`XLEN-1:0]    proc2Imem_addr

);
    logic [`XLEN-1:0] PC_reg;



    // PC register
    always_ff @(posedge clock) begin
        if (reset) begin
            PC_reg <= 0;             // initial PC value is 0 (the memory address where our program starts)
        end else begin
            PC_reg <= PC_reg + 4;
        end
    end

    assign proc2Imem_addr = {PC_reg[`XLEN-1:3], 3'b0};
    assign if_packet.inst = PC_reg[2] ? Imem2proc_data[63:32] : Imem2proc_data[31:0];
    assign if_packet.PC  = PC_reg;
    assign if_packet.NPC = PC_reg + 4; // pass PC+4 down pipeline w/instruction




endmodule