/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  pipeline.sv                                         //
//                                                                     //
//  Description :  Top-level module of the verisimple pipeline;        //
//                 This instantiates and connects the 5 stages of the  //
//                 Verisimple pipeline together.                       //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

module pipeline (
    input        clock,             // System clock
    input        reset,             // System reset
    input [3:0]  mem2proc_response, // Tag from memory about current request
    input [63:0] mem2proc_data,     // Data coming back from memory
    input [3:0]  mem2proc_tag,      // Tag from memory about current reply

    output logic [1:0]       proc2mem_command, // Command sent to memory
    output logic [`XLEN-1:0] proc2mem_addr,    // Address sent to memory
    output logic [63:0]      proc2mem_data,    // Data sent to memory
`ifndef CACHE_MODE // no longer sending size to memory
    output MEM_SIZE          proc2mem_size,    // Data size sent to memory
`endif

    // Note: these are assigned at the very bottom of the module
    output logic [3:0]       pipeline_completed_insts,
    output EXCEPTION_CODE    pipeline_error_status,
    output logic [4:0]       pipeline_commit_wr_idx,
    output logic [`XLEN-1:0] pipeline_commit_wr_data,
    output logic             pipeline_commit_wr_en,
    output logic [`XLEN-1:0] pipeline_commit_NPC

    // Debug outputs: these signals are solely used for debugging in testbenches
    // Do not change for project 3
    // You should definitely change these for project 4
    // output logic [`XLEN-1:0] if_NPC_dbg,
    // output logic [31:0]      if_inst_dbg,
    // output logic             if_valid_dbg,
    // output logic [`XLEN-1:0] if_id_NPC_dbg,
    // output logic [31:0]      if_id_inst_dbg,
    // output logic             if_id_valid_dbg,
    // output logic [`XLEN-1:0] id_ex_NPC_dbg,
    // output logic [31:0]      id_ex_inst_dbg,
    // output logic             id_ex_valid_dbg,
    // output logic [`XLEN-1:0] ex_mem_NPC_dbg,
    // output logic [31:0]      ex_mem_inst_dbg,
    // output logic             ex_mem_valid_dbg,
    // output logic [`XLEN-1:0] mem_wb_NPC_dbg,
    // output logic [31:0]      mem_wb_inst_dbg,
    // output logic             mem_wb_valid_dbg
);

    //////////////////////////////////////////////////
    //                                              //
    //                Pipeline Wires                //
    //                                              //
    //////////////////////////////////////////////////

    // Pipeline register enables
    logic if_id_enable, id_ex_enable, ex_mem_enable, mem_wb_enable;

    // Outputs from IF-Stage and IF/DIS Pipeline Register
    logic [`XLEN-1:0] proc2Imem_addr;
    IF_DIS_PACKET if_packet, if_dis_reg;

    // Outputs from DIS stage and DIS/RM Pipeline Register
    DIS_RM_PACKET dis_packet, dis_rm_reg;

    // Outputs from RM-Stage and RM/RSB Pipeline Register
    RM_RSB_PACKET rm_packet, rm_rsb_reg;

    // Outputs from RSB-Stage and RSB/EX Pipeline Register
    RSB_EX_PACKET rsb_packet, rsb_ex_reg;

    // Outputs from MEM-Stage to memory
    logic [`XLEN-1:0] proc2Dmem_addr;
    logic [`XLEN-1:0] proc2Dmem_data;
    logic [1:0]       proc2Dmem_command;
    MEM_SIZE          proc2Dmem_size;

    // Outputs from WB-Stage (These loop back to the register file in ID)
    logic             wb_regfile_en;
    logic [4:0]       wb_regfile_idx;
    logic [`XLEN-1:0] wb_regfile_data;

    //////////////////////////////////////////////////
    //                                              //
    //                Memory Outputs                //
    //                                              //
    //////////////////////////////////////////////////

    // these signals go to and from the processor and memory
    // we give precedence to the mem stage over instruction fetch
    // note that there is no latency in project 3
    // but there will be a 100ns latency in project 4

    always_comb begin
        if (proc2Dmem_command != BUS_NONE) begin // read or write DATA from memory
            proc2mem_command = proc2Dmem_command;
            proc2mem_addr    = proc2Dmem_addr;
`ifndef CACHE_MODE
            proc2mem_size    = proc2Dmem_size;  // size is never DOUBLE in project 3
`endif
        end else begin                          // read an INSTRUCTION from memory
            proc2mem_command = BUS_LOAD;
            proc2mem_addr    = proc2Imem_addr;
`ifndef CACHE_MODE
            proc2mem_size    = DOUBLE;          // instructions load a full memory line (64 bits)
`endif
        end
        proc2mem_data = {32'b0, proc2Dmem_data};
    end



    //////////////////////////////////////////////////
    //                                              //
    //                  IF-Stage                    //
    //                                              //
    //////////////////////////////////////////////////
    stage_if stage_if_0 (
        // Inputs
        .clock (clock),
        .reset (reset),
        .Imem2proc_data         (mem2proc_data),

        // Outputs
        .if_packet              (if_packet),
        .proc2Imem_addr         (proc2Imem_addr)
    );

    //////////////////////////////////////////////////
    //                                              //
    //            IF/DIS Pipeline Register          //
    //                                              //
    //////////////////////////////////////////////////
    assign if_dis_enable = 1'b1; // always enabled
    // synopsys sync_set_reset "reset"
    always_ff @(posedge clock) begin
        if (reset) begin
            if_dis_reg.inst  <= `NOP;
            if_dis_reg.valid <= `FALSE;
            if_dis_reg.NPC   <= 0;
            if_dis_reg.PC    <= 0;
        end else if (if_dis_enable) begin
            if_dis_reg <= if_packet;
        end 
    end

    //////////////////////////////////////////////////
    //                                              //
    //                  DIS-Stage                   //
    //                                              //
    //////////////////////////////////////////////////

    stage_dis stage_dis_0 (
        // Inputs
        .clock             (clock),
        .reset             (reset),
        .if_dis_reg        (if_dis_reg),
        .wb_regfile_en     (wb_regfile_en),
        .wb_regfile_idx    (wb_regfile_idx),
        .wb_regfile_data   (wb_regfile_data),

        // Output
        .dis_packet        (dis_packet)
    );


    //////////////////////////////////////////////////
    //                                              //
    //            DIS/RM Pipeline Register          //
    //                                              //
    //////////////////////////////////////////////////

    assign dis_rm_enable = 1'b1; // always enabled
    // synopsys sync_set_reset "reset"
    always_ff @(posedge clock) begin
        if (reset) begin
            dis_rm_reg <= '{
                `NOP, // we can't simply assign 0 because NOP is non-zero
                {`XLEN{1'b0}},  // PC
                {`XLEN{1'b0}},  // NPC
                {`XLEN{1'b0}},  // rs1 select
                {`XLEN{1'b0}},  // rs2 select
                OPA_IS_RS1,     // opa_select
                OPB_IS_RS2,     // opb_select
                `ZERO_REG,      // dest_reg_idx
                ALU_ADD,        // fu_func
                ALU,            // fu_mark
                1'b0,           // rd_mem
                1'b0,           // wr_mem
                1'b0,           // cond
                1'b0,           // uncond
                1'b0,           // halt
                1'b0,           // illegal
                1'b0,           // csr_op
                1'b0            // valid
            };
        end else if (dis_rm_enable) begin
            dis_rm_reg <= dis_packet;
        end
    end


    //////////////////////////////////////////////////
    //                                              //
    //                  RM-Stage                    //
    //                                              //
    //////////////////////////////////////////////////


    //////////////////////////////////////////////////
    //                                              //
    //          RM/RS-ROB Pipeline Register         //
    //                                              //
    //////////////////////////////////////////////////


    //////////////////////////////////////////////////
    //                                              //
    //                 RS-ROB-Stage                 //
    //                                              //
    //////////////////////////////////////////////////

    stage_rsb stage_rsb_0 (
        // Inputs
        .clock             (clock),
        .reset             (reset),
        .rm_rsb_reg        (rm_rsb_reg),

        // Output
        .rsb_packet        (rsb_packet)

    );


    //////////////////////////////////////////////////
    //                                              //
    //          RS-ROB/EX Pipeline Register         //
    //                                              //
    //////////////////////////////////////////////////



    //////////////////////////////////////////////////
    //                                              //
    //                  EX-Stage                    //
    //                                              //
    //////////////////////////////////////////////////


    //////////////////////////////////////////////////
    //                                              //
    //               Pipeline Outputs               //
    //                                              //
    //////////////////////////////////////////////////

    assign pipeline_completed_insts = {3'b0, mem_wb_reg.valid}; // commit one valid instruction
    assign pipeline_error_status = mem_wb_reg.illegal        ? ILLEGAL_INST :
                                   mem_wb_reg.halt           ? HALTED_ON_WFI :
                                   (mem2proc_response==4'h0) ? LOAD_ACCESS_FAULT : NO_ERROR;

    assign pipeline_commit_wr_en   = wb_regfile_en;
    assign pipeline_commit_wr_idx  = wb_regfile_idx;
    assign pipeline_commit_wr_data = wb_regfile_data;
    assign pipeline_commit_NPC     = mem_wb_reg.NPC;

endmodule // pipeline
