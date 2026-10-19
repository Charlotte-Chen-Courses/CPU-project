/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  retire_rat.sv                                       //
//                                                                     //
//  Description :  Retirement (architectural) register alias table.    //
//                 Same shape as the speculative RAT, but updated only //
//                 at commit: map[rd] <= tag for each committing slot  //
//                 with a destination (youngest slot wins on the same  //
//                 rd). Resets to x_i -> p_i. The precise state:       //
//                 architectural xN is PRF[map[N]]. Used by the test   //
//                 harness now, and for mispredict recovery later      //
//                 (copy into the speculative RAT).                    //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

module retire_rat #(
    localparam TAG_W  = `TAG_W,
    localparam W      = `SUPERSCALAR_WIDTH,
    localparam RAT_SZ = `ARCH_REG_SZ
) (
    input                                  clock,
    input                                  reset,
    input        [W-1:0]                   commit_en,   // commit_valid && commit_dest_valid
    input        [W-1:0][4:0]              commit_rd,
    input        [W-1:0][TAG_W-1:0]        commit_tag,
    output logic [RAT_SZ-1:0][TAG_W-1:0]   map
);

  always_ff @(posedge clock) begin
    if (reset) begin
      for (int i = 0; i < RAT_SZ; i++) map[i] <= TAG_W'(i);
    end else begin
      for (int i = 0; i < W; i++)
        if (commit_en[i] && commit_rd[i] != '0) map[commit_rd[i]] <= commit_tag[i];
    end
  end

endmodule  // retire_rat
