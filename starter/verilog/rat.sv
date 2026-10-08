/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  rat.sv                                              //
//                                                                     //
//  Description :  Speculative register alias table (map table) for    //
//                 rename. ARCH_REG_SZ entries, each holding the       //
//                 physical tag of the newest value of that arch reg.  //
//                 Resets to x_i -> p_i. Each cycle, for every slot,   //
//                 looks up src1/src2 tags and told (rd's previous     //
//                 mapping); older slots in the same bundle that write //
//                 a matching rd are bypassed in, so the bundle sees   //
//                 renames as if done one slot at a time. On           //
//                 rename_en, writes rd -> new_tag for each slot with  //
//                 a real destination (valid, has_dest, rd != x0);     //
//                 the youngest slot wins on a same-rd conflict.       //
//                 No retirement RAT or recovery yet.                  //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

module rat #(
    localparam TAG_W = `TAG_W,
    localparam W = `SUPERSCALAR_WIDTH,
    localparam RAT_SZ = `ARCH_REG_SZ
) (
    input clock,
    input reset,

    input rename_en,

    input [W-1:0] valid,
    input [W-1:0][4:0] rs1,
    rs2,
    rd,
    input [W-1:0] has_dest,
    input [W-1:0][TAG_W-1:0] new_tag,
    output logic [W-1:0][TAG_W-1:0] src1_tag,
    src2_tag,
    told


);

  logic [RAT_SZ-1:0][TAG_W-1:0] map;
  logic [W-1:0] dest_valid;

  always_comb for (int i = 0; i < W; i++) dest_valid[i] = valid[i] && has_dest[i] && (rd[i] != 0);


  always_comb begin
    for (int i = 0; i < W; i++) begin
      src1_tag[i] = map[rs1[i]];
      src2_tag[i] = map[rs2[i]];
      told[i]     = map[rd[i]];
      for (int j = 0; j < i; j++) begin
        if (dest_valid[j] && rd[j] == rs1[i]) src1_tag[i] = new_tag[j];
        if (dest_valid[j] && rd[j] == rs2[i]) src2_tag[i] = new_tag[j];
        if (dest_valid[j] && rd[j] == rd[i]) told[i] = new_tag[j];
      end
    end
  end

  always_ff @(posedge clock) begin
    if (reset) begin
      for (int i = 0; i < RAT_SZ; i++) map[i] <= TAG_W'(i);
    end else if (rename_en) begin
      for (int i = 0; i < W; i++) if (dest_valid[i]) map[rd[i]] <= new_tag[i];
    end
  end



endmodule
