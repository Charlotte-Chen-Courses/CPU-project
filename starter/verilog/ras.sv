/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  ras.sv                                              //
//                                                                     //
//  Description :  Return address stack. RAS_SZ entries, circular      //
//                 (deeper nesting overwrites the oldest entry: costs  //
//                 mispredicts, never correctness). Two copies:        //
//                  - speculative, used by fetch: a predicted call     //
//                    pushes PC+4, a predicted return pops; top is the //
//                    predicted return target. At most one op/cycle.   //
//                  - committed: updated only by committing calls and  //
//                    returns, in program order (up to WIDTH a cycle). //
//                 flush (mispredict at commit) copies the committed   //
//                 stack, including this cycle's commits, into the     //
//                 speculative one, undoing wrong-path pushes/pops     //
//                 and any call/return fetch failed to predict.        //
//                 Calls: jal/jalr with rd = x1/x5. Returns: jalr with //
//                 rd = x0 and rs1 = x1/x5 (RISC-V hint convention).   //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

module ras #(
    localparam W      = `SUPERSCALAR_WIDTH,
    localparam RAS_SZ = `RAS_SZ,
    localparam P      = $clog2(RAS_SZ)
) (
    input clock,
    input reset,

    // speculative (fetch)
    input                          spec_push,       // predicted call accepted this cycle
    input        [`XLEN-1:0]       spec_push_addr,  // its PC + 4
    input                          spec_pop,        // predicted return accepted this cycle
    output logic [`XLEN-1:0]       top,             // predicted return target

    // committed calls / returns, in program order
    input        [W-1:0]           commit_call,
    input        [W-1:0]           commit_ret,
    input        [W-1:0][`XLEN-1:0] commit_pc,

    input                          flush
);

  logic [RAS_SZ-1:0][`XLEN-1:0] s_stack, r_stack, r_stack_next;
  logic [P-1:0]                 s_tos, r_tos, r_tos_next;   // index of the top entry

  assign top = s_stack[s_tos];

  // committed stack after this cycle's commits (also what a flush restores)
  always_comb begin
    r_stack_next = r_stack;
    r_tos_next   = r_tos;
    for (int i = 0; i < W; i++) begin
      if (commit_call[i]) begin
        r_tos_next               = r_tos_next + 1'b1;
        r_stack_next[r_tos_next] = commit_pc[i] + 4;
      end else if (commit_ret[i]) begin
        r_tos_next = r_tos_next - 1'b1;
      end
    end
  end

  always_ff @(posedge clock) begin
    if (reset) begin
      s_stack <= '0; s_tos <= '0;
      r_stack <= '0; r_tos <= '0;
    end else begin
      r_stack <= r_stack_next;
      r_tos   <= r_tos_next;
      if (flush) begin
        s_stack <= r_stack_next;
        s_tos   <= r_tos_next;
      end else if (spec_push) begin
        s_tos                <= s_tos + 1'b1;
        s_stack[s_tos + 1'b1] <= spec_push_addr;
      end else if (spec_pop) begin
        s_tos <= s_tos - 1'b1;
      end
    end
  end

endmodule  // ras
