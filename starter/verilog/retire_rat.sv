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
//                 harness, and for mispredict recovery: map_next (the //
//                 map after this cycle's commits) is copied into the  //
//                 speculative RAT.                                    //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

module retire_rat #(
    parameter  BASE   = 0,             // first tag of this register class: reset map[i] = BASE + i
    parameter  bit ZERO_REG = 1'b1,    // register 0 is never remapped (x0)
    localparam TAG_W  = `TAG_W,
    localparam W      = `SUPERSCALAR_WIDTH,
    localparam RAT_SZ = `ARCH_REG_SZ
) (
    input                                  clock,
    input                                  reset,
    input        [W-1:0]                   commit_en,   // commit_valid && commit_dest_valid
    input        [W-1:0][4:0]              commit_rd,
    input        [W-1:0][TAG_W-1:0]        commit_tag,
    output logic [RAT_SZ-1:0][TAG_W-1:0]   map,
    output logic [RAT_SZ-1:0][TAG_W-1:0]   map_next     // map with this cycle's commits applied
);

  always_comb begin
    map_next = map;
    for (int i = 0; i < W; i++)
      if (commit_en[i] && (!ZERO_REG || commit_rd[i] != '0)) map_next[commit_rd[i]] = commit_tag[i];
  end

  always_ff @(posedge clock) begin
    if (reset) begin
      for (int i = 0; i < RAT_SZ; i++) map[i] <= TAG_W'(BASE + i);
    end else begin
      map <= map_next;
    end
  end

endmodule  // retire_rat
