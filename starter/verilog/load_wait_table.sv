/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  load_wait_table.sv                                  //
//                                                                     //
//  Description :  Memory-dependence predictor (Alpha 21264 load-wait  //
//                 table). SZ one-bit entries indexed by load PC       //
//                 (PC[2 +: log2 SZ]; different loads may share one).  //
//                 A replayed load (memory-order violation) sets its   //
//                 bit; a load whose bit is set waits for every older  //
//                 store address instead of speculating. Every CLEAR   //
//                 cycles all bits clear, so a load that stopped       //
//                 conflicting gets to speculate again. A train in a   //
//                 clear cycle still sets its bit.                     //
//                 Lookup is combinational (when the load's address is //
//                 generated); training is at the edge.                //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

module load_wait_table #(
    parameter  SZ    = `LWT_SZ,
    parameter  CLEAR = `LWT_CLEAR,
    localparam P     = $clog2(SZ)
) (
    input                    clock,
    input                    reset,

    /* verilator lint_off UNUSEDSIGNAL */      // only PC[2 +: P] indexes the table
    input        [`XLEN-1:0] lookup_pc,     // load getting its address
    output logic             lookup_wait,   // 1: it should wait for older store addresses

    input                    train_valid,   // this load was replayed
    input        [`XLEN-1:0] train_pc
    /* verilator lint_on UNUSEDSIGNAL */
);

  logic [SZ-1:0]              wait_bit;
  logic [$clog2(CLEAR+1)-1:0] age;          // cycles since the last clear

  assign lookup_wait = wait_bit[lookup_pc[2 +: P]];

  always_ff @(posedge clock) begin
    if (reset) begin
      wait_bit <= '0;
      age      <= '0;
    end else begin
      if (int'(age) == CLEAR - 1) begin
        wait_bit <= '0;
        age      <= '0;
      end else age <= age + 1'b1;
      if (train_valid) wait_bit[train_pc[2 +: P]] <= 1'b1;   // after the clear: the train wins
    end
  end

endmodule  // load_wait_table
