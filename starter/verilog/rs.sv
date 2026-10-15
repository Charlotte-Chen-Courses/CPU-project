`include "verilog/sys_defs.svh"

module rs #(
    localparam W = `SUPERSCALAR_WIDTH,
    localparam RS_SZ = `RS_SZ,
    localparam CDB_SZ = `NUM_CDB,
    localparam PTR_W = `RS_IDX_W,
    localparam TAG_W = `TAG_W
) (
    input                    clock,
    input                    reset,
    // dispatch
    input            [W-1:0] disp_valid,
    input  RS_PACKET [W-1:0] disp_pkt,
    output logic             space_ok,    // ≥ W free entries

    // CDB stage
    input [CDB_SZ-1:0]            cdb_valid,
    input [CDB_SZ-1:0][TAG_W-1:0] cdb_tag,

    // issue
    
);

  RS_PACKET rs_map[RS_SZ];
  RS_PACKET [W-1:0] cdb_pkt;  // dispatched packet with this cycle's CDB applied
  logic [RS_SZ-1:0] rs_valid;
  logic [RS_SZ-1:0] taken;

  // dispatch
  logic [W-1:0][PTR_W-1:0] alloc_idx;
  logic [W-1:0] alloc_found;

  always_comb begin
    taken = rs_valid;
    for (int slot = 0; slot < W; slot++) begin
      alloc_found[slot] = `FALSE;
      alloc_idx[slot]   = '0;
      if (disp_valid[slot])
        for (int entry = 0; entry < RS_SZ; entry++)
        if (!taken[entry] && !alloc_found[slot]) begin
          alloc_idx[slot]   = PTR_W'(entry);
          alloc_found[slot] = `TRUE;
          taken[entry]      = `TRUE;
        end
    end
  end

  assign space_ok = ($countones(~rs_valid) >= W);

  // same-cycle wakeup: a source broadcast this cycle is ready on arrival
  always_comb begin
    for (int i = 0; i < W; i++) begin
      cdb_pkt[i] = disp_pkt[i];
      for (int k = 0; k < CDB_SZ; k++)
      if (cdb_valid[k]) begin
        if (disp_pkt[i].src1_tag == cdb_tag[k]) cdb_pkt[i].src1_ready = `TRUE;
        if (disp_pkt[i].src2_tag == cdb_tag[k]) cdb_pkt[i].src2_ready = `TRUE;
      end
    end
  end


  always_ff @(posedge clock) begin
    if (reset) rs_valid <= '0;
    else begin
      for (int e = 0; e < RS_SZ; e++) begin
        if (rs_valid[e])
          for (int k = 0; k < CDB_SZ; k++)
          if (cdb_valid[k]) begin
            if (rs_map[e].src1_tag == cdb_tag[k]) rs_map[e].src1_ready <= `TRUE;
            if (rs_map[e].src2_tag == cdb_tag[k]) rs_map[e].src2_ready <= `TRUE;
          end
      end
      for (int i = 0; i < W; i++)
      if (disp_valid[i]) begin
        rs_map[alloc_idx[i]]   <= cdb_pkt[i];
        rs_valid[alloc_idx[i]] <= `TRUE;
      end
    end
  end






endmodule
