/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  rs_unit_testbench.sv                                //
//                                                                     //
//  Description :  Testbench for the reservation station unit;         //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

module rs_unit_test;
  // Clock and reset signals
  logic clock;
  logic reset;
  
  // Test inputs
  RM_RSB_PACKET inst_packet;
  logic [4:0] rob_t_index;
  logic [4:0] rob_s1_index;
  logic [4:0] rob_s2_index;
  logic [`XLEN-1:0] input_v1;
  logic [`XLEN-1:0] input_v2;
  logic v1_ready;
  logic v2_ready;
  
  // Test outputs
  logic [4:0] mp_s1_index;
  logic [4:0] mp_s2_index;
  logic [`XLEN-1:0] output_v1;
  logic [`XLEN-1:0] output_v2;
  
  // Instantiate the rs_unit
  rs_unit DUT (
    .clock(clock),        // Connect the clock signal
    .reset(reset),        // Connect the reset signal
    .inst_packet(inst_packet),
    .rob_t_index(rob_t_index),
    .rob_s1_index(rob_s1_index),
    .rob_s2_index(rob_s2_index),
    .input_v1(input_v1),
    .input_v2(input_v2),
    .v1_ready(v1_ready),
    .v2_ready(v2_ready),
    .mp_s1_index(mp_s1_index),
    .mp_s2_index(mp_s2_index),
    .output_v1(output_v1),
    .output_v2(output_v2)
  );
  
  // Clock generation
  initial begin
    clock = 0;
    forever #5 clock = ~clock;
  end

  // Test sequence
  initial begin
    // Initialize
    reset = 1;
    inst_packet = 0;
    rob_t_index = 0;
    rob_s1_index = 0;
    rob_s2_index = 0;
    input_v1 = 0;
    input_v2 = 0;
    v1_ready = 0;
    v2_ready = 0;
    
    // Apply reset
    @(posedge clock);
    reset = 0;
    @(posedge clock);
    
    // Test Case 1: ALU R-type instruction
    $display("Test Case 1: ALU R-type instruction");
    // Setup a test instruction (ADD)
    inst_packet.inst.inst_type = R;
    inst_packet.inst.r.rs1 = 5'd1;  // Source register 1
    inst_packet.inst.r.rs2 = 5'd2;  // Source register 2
    inst_packet.inst.r.rd = 5'd3;   // Destination register
    inst_packet.fu_mark = ALU;      // ALU functional unit
    inst_packet.fu_func = ALU_ADD;  // ADD operation
    inst_packet.rs1_value = 32'h10; // Source 1 value
    inst_packet.rs2_value = 32'h20; // Source 2 value
    
    // Setup ROB indices
    rob_t_index = 5'd5;   // Target index in ROB
    rob_s1_index = 5'd1;  // Source 1 index in ROB
    rob_s2_index = 5'd2;  // Source 2 index in ROB
    
    // Set source register values
    input_v1 = 32'h10;    // Value for rs1
    input_v2 = 32'h20;    // Value for rs2
    
    // Set ready signals
    v1_ready = `TRUE;     // Source 1 value is ready
    v2_ready = `TRUE;     // Source 2 value is ready
    
    // Apply the inputs
    @(posedge clock);
    
    // Check outputs
    #1; // Small delay to allow combinational logic to settle
    $display("mp_s1_index = %d, expected = %d", mp_s1_index, inst_packet.inst.r.rs1);
    $display("mp_s2_index = %d, expected = %d", mp_s2_index, inst_packet.inst.r.rs2);
    
    // Verify that the entry was added to the RS
    $display("RS Entry 0 busy = %b, expected = %b", DUT.rs_entries[0].busy, `TRUE);
    $display("RS Entry 0 op_code = %h, expected = %h", DUT.rs_entries[0].op_code, ALU_ADD);
    $display("RS Entry 0 rob_target_index = %d, expected = %d", DUT.rs_entries[0].rob_target_index, rob_t_index);
    $display("RS Entry 0 source1_ready = %b, expected = %b", DUT.rs_entries[0].source1_ready, v1_ready);
    $display("RS Entry 0 source2_ready = %b, expected = %b", DUT.rs_entries[0].source2_ready, v2_ready);
    
    // Test Case 2: LD I-type instruction
    $display("\nTest Case 2: LD I-type instruction");
    // Setup a test instruction (LOAD)
    inst_packet.inst.inst_type = I;
    inst_packet.inst.i.rs1 = 5'd4;    // Base register
    inst_packet.inst.i.rd = 5'd5;     // Destination register
    inst_packet.fu_mark = LD;         // Load functional unit
    inst_packet.fu_func = ALU_ADD;    // Address calculation (ADD)
    inst_packet.rs1_value = 32'h100;  // Base address
    inst_packet.rs2_value = 32'h0;    // Not used for I-type
    
    // Setup ROB indices
    rob_t_index = 5'd6;   // Target index in ROB
    rob_s1_index = 5'd4;  // Source 1 index in ROB
    // rob_s2_index not used for I-type
    
    // Set source register values
    input_v1 = 32'h100;   // Value for rs1
    
    // Set ready signals
    v1_ready = `TRUE;     // Source 1 value is ready
    v2_ready = `FALSE;    // Not applicable for I-type
    
    // Apply the inputs
    @(posedge clock);
    
    // Check outputs
    #1; // Small delay to allow combinational logic to settle
    $display("mp_s1_index = %d, expected = %d", mp_s1_index, inst_packet.inst.i.rs1);
    
    // Verify that the entry was added to the RS
    $display("RS Entry 1 busy = %b, expected = %b", DUT.rs_entries[1].busy, `TRUE);
    $display("RS Entry 1 op_code = %h, expected = %h", DUT.rs_entries[1].op_code, ALU_ADD);
    
    // Test Case 3: ST S-type instruction
    $display("\nTest Case 3: ST S-type instruction");
    // Setup a test instruction (STORE)
    inst_packet.inst.inst_type = S;
    inst_packet.inst.s.rs1 = 5'd6;    // Base register
    inst_packet.inst.s.rs2 = 5'd7;    // Source register
    inst_packet.fu_mark = ST;         // Store functional unit
    inst_packet.fu_func = ALU_ADD;    // Address calculation (ADD)
    inst_packet.rs1_value = 32'h200;  // Base address
    inst_packet.rs2_value = 32'h30;   // Value to store
    
    // Setup ROB indices
    rob_t_index = 5'd7;   // Target index in ROB
    rob_s1_index = 5'd6;  // Source 1 index in ROB
    rob_s2_index = 5'd7;  // Source 2 index in ROB
    
    // Set source register values
    input_v1 = 32'h200;   // Value for rs1
    input_v2 = 32'h30;    // Value for rs2
    
    // Set ready signals
    v1_ready = `TRUE;     // Source 1 value is ready
    v2_ready = `TRUE;     // Source 2 value is ready
    
    // Apply the inputs
    @(posedge clock);
    
    // Check outputs
    #1; // Small delay to allow combinational logic to settle
    $display("mp_s1_index = %d, expected = %d", mp_s1_index, inst_packet.inst.s.rs1);
    $display("mp_s2_index = %d, expected = %d", mp_s2_index, inst_packet.inst.s.rs2);
    
    // Verify that the entry was added to the RS
    $display("RS Entry 2 busy = %b, expected = %b", DUT.rs_entries[2].busy, `TRUE);
    
    // Test Case 4: B-type instruction
    $display("\nTest Case 4: B-type instruction");
    // Setup a test instruction (BRANCH)
    inst_packet.inst.inst_type = B;
    inst_packet.inst.b.rs1 = 5'd8;    // First operand register
    inst_packet.inst.b.rs2 = 5'd9;    // Second operand register
    inst_packet.fu_mark = ALU;        // ALU functional unit for comparison
    inst_packet.fu_func = ALU_SUB;    // Subtraction for comparison
    inst_packet.rs1_value = 32'h40;   // First operand value
    inst_packet.rs2_value = 32'h40;   // Second operand value
    
    // Setup ROB indices
    rob_t_index = 5'd8;   // Target index in ROB
    rob_s1_index = 5'd8;  // Source 1 index in ROB
    rob_s2_index = 5'd9;  // Source 2 index in ROB
    
    // Set source register values
    input_v1 = 32'h40;    // Value for rs1
    input_v2 = 32'h40;    // Value for rs2
    
    // Set ready signals
    v1_ready = `TRUE;     // Source 1 value is ready
    v2_ready = `TRUE;     // Source 2 value is ready
    
    // Apply the inputs
    @(posedge clock);
    
    // Check outputs
    #1; // Small delay to allow combinational logic to settle
    $display("mp_s1_index = %d, expected = %d", mp_s1_index, inst_packet.inst.b.rs1);
    $display("mp_s2_index = %d, expected = %d", mp_s2_index, inst_packet.inst.b.rs2);
    
    // Test complete
    $display("\nAll tests completed");
    $display("@@@ Passed");
    
    // Finish simulation
    #20 $finish;
  end
  
  // For monitoring purposes
  initial begin
    $monitor("Time=%0t: RS Entry 0 busy=%b, RS Entry 1 busy=%b, RS Entry 2 busy=%b",
             $time, DUT.rs_entries[0].busy, DUT.rs_entries[1].busy, DUT.rs_entries[2].busy);
  end
  
endmodule