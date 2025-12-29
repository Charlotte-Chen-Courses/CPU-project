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
    output MEM_SIZE          proc2mem_size,    // Data size sent to memory

    // Note: these are assigned at the very bottom of the module
    output logic [3:0]       pipeline_completed_insts,
    output EXCEPTION_CODE    pipeline_error_status,
    output logic [4:0]       pipeline_commit_wr_idx,
    output logic [`XLEN-1:0] pipeline_commit_wr_data,
    output logic             pipeline_commit_wr_en,
    output logic [`XLEN-1:0] pipeline_commit_NPC,

    // Debug outputs: these signals are solely used for debugging in testbenches
    // Do not change for project 3
    // You should definitely change these for project 4
    output logic [`XLEN-1:0] if_NPC_dbg,
    output logic [31:0]      if_inst_dbg,
    output logic             if_valid_dbg,
    output logic [`XLEN-1:0] if_id_NPC_dbg,
    output logic [31:0]      if_id_inst_dbg,
    output logic             if_id_valid_dbg,
    output logic [`XLEN-1:0] id_ex_NPC_dbg,
    output logic [31:0]      id_ex_inst_dbg,
    output logic             id_ex_valid_dbg,
    output logic [`XLEN-1:0] ex_mem_NPC_dbg,
    output logic [31:0]      ex_mem_inst_dbg,
    output logic             ex_mem_valid_dbg,
    output logic [`XLEN-1:0] mem_wb_NPC_dbg,
    output logic [31:0]      mem_wb_inst_dbg,
    output logic             mem_wb_valid_dbg
);

    //////////////////////////////////////////////////
    //                                              //
    //                Pipeline Wires                //
    //                                              //
    //////////////////////////////////////////////////

    // Pipeline register enables
    logic if_id_enable, id_ex_enable, ex_mem_enable, mem_wb_enable;

    // Outputs from IF-Stage and IF/ID Pipeline Register
    logic [`XLEN-1:0] proc2Imem_addr;
    IF_ID_PACKET if_packet, if_id_reg;

    // Outputs from ID stage and ID/EX Pipeline Register
    ID_EX_PACKET id_packet, id_ex_reg;

    // Outputs from EX-Stage and EX/MEM Pipeline Register
    EX_MEM_PACKET ex_packet, ex_mem_reg;

    // Outputs from MEM-Stage and MEM/WB Pipeline Register
    MEM_WB_PACKET mem_packet, mem_wb_reg;

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
            proc2mem_size    = proc2Dmem_size;   // size is never DOUBLE in project 3
        end else begin                           // read an INSTRUCTION from memory
            proc2mem_command = BUS_LOAD;
            proc2mem_addr    = proc2Imem_addr;
            proc2mem_size    = DOUBLE;          // instructions load a full memory line (64 bits)
        end
        proc2mem_data = {32'b0, proc2Dmem_data};
    end

    //////////////////////////////////////////////////
    //                                              //
    //                  Flush Bits                  //
    //                                              //
    //////////////////////////////////////////////////

    // This state control the flush signal that zeros the previous 
    // instructions if a branch is taken

    logic branch_flush;

    //////////////////////////////////////////////////
    //                                              //
    //                  Valid Bit                   //
    //                                              //
    //////////////////////////////////////////////////

    // This state controls the stall signal that artificially forces IF
    // to stall until the previous instruction has completed.
    // For project 3, start by setting this to always be 1

    logic next_if_valid;

    // synopsys sync_set_reset "reset"
    always_ff @(posedge clock) begin
        if (reset) begin
            // start valid, other stages (ID,EX,MEM,WB) start as invalid
            next_if_valid <= 1;
        end else begin
            // valid bit will cycle through the pipeline and come back from the wb stage
            next_if_valid <= (~(ex_packet.rd_mem || ex_packet.wr_mem)) || branch_flush;
        end
    end

    //////////////////////////////////////////////////
    //                                              //
    //                  Stall Bits                  //
    //                                              //
    //////////////////////////////////////////////////

    // This state control the stall signal that stalls the
    // pipline if a load followed by an instruction that 
    // reads its result.

    logic stall;

    //////////////////////////////////////////////////
    //                                              //
    //                  Forward Unit                //
    //                                              //
    //////////////////////////////////////////////////

    // This states control the forward signals for mux in EX stage
    logic [1:0] forward_a, forward_b;
    logic ex_hazard_a, ex_hazard_b, mem_hazard_a, mem_hazard_b;

    assign ex_hazard_a = (ex_packet.valid && (ex_packet.dest_reg_idx != `ZERO_REG) 
            && ex_packet.dest_reg_idx == id_packet.inst.r.rs1);
    assign ex_hazard_b = (ex_packet.valid && (ex_packet.dest_reg_idx != `ZERO_REG) 
            && ex_packet.dest_reg_idx == id_packet.inst.r.rs2);
    assign mem_hazard_a = (mem_packet.valid && (mem_packet.dest_reg_idx != `ZERO_REG) 
            // only forward most recent result
            && !(ex_packet.valid && (ex_packet.dest_reg_idx != `ZERO_REG) && (ex_packet.dest_reg_idx == id_packet.inst.r.rs1))
            // forward
			&& mem_packet.dest_reg_idx == id_packet.inst.r.rs1);
    assign mem_hazard_b = (mem_packet.valid && (mem_packet.dest_reg_idx != `ZERO_REG) 
            // only forward most recent result
            && !(ex_packet.valid && (ex_packet.dest_reg_idx != `ZERO_REG) && (ex_packet.dest_reg_idx == id_packet.inst.r.rs2))
            // forward
			&& mem_packet.dest_reg_idx == id_packet.inst.r.rs2);
    

    // synopsys sync_set_reset "reset"
    always_ff @(posedge clock) begin
        if (reset) begin
        // start using values from registers in ID
            forward_a <= 2'b0;
            forward_b <= 2'b0;
        end else begin
            if(ex_hazard_a) begin
                forward_a <= 2'b01;
            end else if(mem_hazard_a) begin
                forward_a <= 2'b10;
            end else begin
                forward_a <= 2'b00;
            end
            
            if(ex_hazard_b) begin
                forward_b <= 2'b01;
            end else if(mem_hazard_b) begin
                forward_b <= 2'b10;
            end else begin
                forward_b <= 2'b00;
            end
        end

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
        .if_valid               (next_if_valid),
        .flush                  (branch_flush),
        .stall                  (stall),
        .take_branch            (ex_packet.take_branch),
        .pred_branch            (id_packet.pred_branch),
        .branch_target          (ex_packet.alu_result),
        .Imem2proc_data         (mem2proc_data),

        // Outputs
        .if_packet              (if_packet),
        .proc2Imem_addr         (proc2Imem_addr)
    );

    // debug outputs
    assign if_NPC_dbg   = if_packet.NPC;
    assign if_inst_dbg  = if_packet.inst;
    assign if_valid_dbg = if_packet.valid;

    //////////////////////////////////////////////////
    //                                              //
    //            IF/ID Pipeline Register           //
    //                                              //
    //////////////////////////////////////////////////

    assign if_id_enable = 1'b1; // always enabled
    // synopsys sync_set_reset "reset"
    always_ff @(posedge clock) begin
        if (reset || branch_flush) begin
            if_id_reg.inst  <= `NOP;
            if_id_reg.valid <= `FALSE;
            if_id_reg.NPC   <= 0;
            if_id_reg.PC    <= 0;
        end else if (stall) begin
            if_id_reg.inst <= id_packet.inst;
            if_id_reg.valid <= id_packet.valid;
            if_id_reg.NPC <= id_packet.NPC;
            if_id_reg.PC <= id_packet.PC;
        end else if (if_id_enable) begin
            if_id_reg <= if_packet;
        end 
    end

    // debug outputs
    assign if_id_NPC_dbg   = if_id_reg.NPC;
    assign if_id_inst_dbg  = if_id_reg.inst;
    assign if_id_valid_dbg = if_id_reg.valid;

    //////////////////////////////////////////////////
    //                                              //
    //                  ID-Stage                    //
    //                                              //
    //////////////////////////////////////////////////

    stage_id stage_id_0 (
        // Inputs
        .clock (clock),
        .reset (reset),
        // stall logic
        .stall_packet     (id_ex_reg),
        .if_id_reg        (if_id_reg),
        .wb_regfile_en    (wb_regfile_en),
        .wb_regfile_idx   (wb_regfile_idx),
        .wb_regfile_data  (wb_regfile_data),

        // Output
        .id_packet (id_packet),
        .stall     (stall)
    );

    //////////////////////////////////////////////////
    //                                              //
    //            ID/EX Pipeline Register           //
    //                                              //
    //////////////////////////////////////////////////

    assign id_ex_enable = 1'b1; // always enabled
    // synopsys sync_set_reset "reset"
    always_ff @(posedge clock) begin
        if (reset || branch_flush ||stall) begin
            id_ex_reg <= '{
                `NOP, // we can't simply assign 0 because NOP is non-zero
                {`XLEN{1'b0}}, // PC
                {`XLEN{1'b0}}, // NPC
                {`XLEN{1'b0}}, // rs1 select
                {`XLEN{1'b0}}, // rs2 select
                OPA_IS_RS1,
                OPB_IS_RS2,
                `ZERO_REG,
                ALU_ADD,
                1'b0, // rd_mem
                1'b0, // wr_mem
                1'b0, // cond
                1'b0, // uncond
                1'b0, // pred
                1'b0, // halt
                1'b0, // illegal
                1'b0, // csr_op
                1'b0  // valid
            };
        end else if (id_ex_enable) begin
            id_ex_reg <= id_packet;
        end
    end

    // debug outputs
    assign id_ex_NPC_dbg   = id_ex_reg.NPC;
    assign id_ex_inst_dbg  = id_ex_reg.inst;
    assign id_ex_valid_dbg = id_ex_reg.valid;

    //////////////////////////////////////////////////
    //                                              //
    //                  EX-Stage                    //
    //                                              //
    //////////////////////////////////////////////////

    stage_ex stage_ex_0 (
        // Input
        .id_ex_reg (id_ex_reg),

        // Forward selectors
        .forward_a (forward_a),
        .forward_b (forward_b),

        // Forward ports
        .forward_a_port_1 (ex_mem_reg.alu_result),
        .forward_a_port_2 (wb_regfile_data),
        .forward_b_port_1 (ex_mem_reg.alu_result),
        .forward_b_port_2 (wb_regfile_data),

        // Flush
        .flush (branch_flush),
        // Output
        .ex_packet (ex_packet)
    );

    //////////////////////////////////////////////////
    //                                              //
    //           EX/MEM Pipeline Register           //
    //                                              //
    //////////////////////////////////////////////////

    assign ex_mem_enable = 1'b1; // always enabled
    // synopsys sync_set_reset "reset"
    always_ff @(posedge clock) begin
        if (reset) begin
            ex_mem_inst_dbg <= `NOP; // debug output
            ex_mem_reg      <= 0;    // the defaults can all be zero!
        end else if (ex_mem_enable) begin
            ex_mem_inst_dbg <= id_ex_inst_dbg; // debug output, just forwarded from ID
            ex_mem_reg      <= ex_packet;
        end
    end

    // debug outputs
    assign ex_mem_NPC_dbg   = ex_mem_reg.NPC;
    assign ex_mem_valid_dbg = ex_mem_reg.valid;

    //////////////////////////////////////////////////
    //                                              //
    //                 MEM-Stage                    //
    //                                              //
    //////////////////////////////////////////////////

    stage_mem stage_mem_0 (
        // Inputs
        .ex_mem_reg     (ex_mem_reg),
        .Dmem2proc_data (mem2proc_data[`XLEN-1:0]), // for p3, we throw away the top 32 bits

        // Outputs
        .mem_packet        (mem_packet),
        .proc2Dmem_command (proc2Dmem_command),
        .proc2Dmem_size    (proc2Dmem_size),
        .proc2Dmem_addr    (proc2Dmem_addr),
        .proc2Dmem_data    (proc2Dmem_data)
    );

    //////////////////////////////////////////////////
    //                                              //
    //           MEM/WB Pipeline Register           //
    //                                              //
    //////////////////////////////////////////////////

    assign mem_wb_enable = 1'b1; // always enabled
    // synopsys sync_set_reset "reset"
    always_ff @(posedge clock) begin
        if (reset) begin
            mem_wb_inst_dbg <= `NOP; // debug output
            mem_wb_reg      <= 0;    // the defaults can all be zero!
        end else if (mem_wb_enable) begin
            mem_wb_inst_dbg <= ex_mem_inst_dbg; // debug output, just forwarded from EX
            mem_wb_reg      <= mem_packet;
        end
    end

    // debug outputs
    assign mem_wb_NPC_dbg   = mem_wb_reg.NPC;
    assign mem_wb_valid_dbg = mem_wb_reg.valid;

    //////////////////////////////////////////////////
    //                                              //
    //                  WB-Stage                    //
    //                                              //
    //////////////////////////////////////////////////

    stage_wb stage_wb_0 (
        // Input
        .mem_wb_reg (mem_wb_reg), // doesn't use all of these

        // Outputs
        .wb_regfile_en   (wb_regfile_en),
        .wb_regfile_idx  (wb_regfile_idx),
        .wb_regfile_data (wb_regfile_data)
    );

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
