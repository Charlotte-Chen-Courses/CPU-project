/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  freelist.sv                                         //
//                                                                     //
//  Description :  Free list of physical register tags for rename.     //
//                 Circular FIFO of FREE_LIST_SZ (= PHYS_REG_SZ - 32)  //
//                 tags. Resets full with pregs 32..PHYS_REG_SZ-1      //
//                 (pregs 0..31 hold the initial arch state; preg 0    //
//                 is never on the list). Allocates up to WIDTH tags   //
//                 per cycle at rename; frees up to WIDTH old pregs    //
//                 per cycle at commit. alloc_ok is all-or-nothing:    //
//                 the pointer only moves if every request fits.       //
//                 Requests need not be packed (e.g. alloc_req=2'b10). //
//                 Overflow cannot happen in a correct core, so it is  //
//                 asserted rather than handled.                       //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

module freelist #(
    localparam TAG_W = `TAG_W,
    localparam W = `SUPERSCALAR_WIDTH,
    localparam PTR_W = $clog2(`FREE_LIST_SZ)
) (
    input clock,
    input reset,
    input [W-1:0] alloc_req,  // slot i needs a dest preg
    output logic [W-1:0][TAG_W-1:0] alloc_tag,  // valid only if alloc_ok && alloc_req[i]
    output logic alloc_ok,  // enough free for all requests
    input [W-1:0] free_en,  // slot i commits with an old preg
    input [W-1:0][TAG_W-1:0] free_tag  // old preg from committing ROB entry
);
  logic [TAG_W-1:0] freelist_mem[`FREE_LIST_SZ];
  logic [PTR_W-1:0] alloc_ptr, free_ptr;

  // slot i's offset from the pointer = number of earlier slots that also requested/freed
  logic [W-1:0][PTR_W-1:0] alloc_idx, free_idx;
  logic [$clog2(
`FREE_LIST_SZ+1
)-1:0] free_count;  // 0..FREE_LIST_SZ, sized so synthesis doesn't build 32 bits
  int n_alloc, n_free;

  // ---- combinational: this cycle's outputs ----
  always_comb begin
    n_alloc = 0;
    n_free  = 0;
    for (int i = 0; i < W; i++) begin
      alloc_idx[i] = PTR_W'((int'(alloc_ptr) + n_alloc) % `FREE_LIST_SZ);
      free_idx[i]  = PTR_W'((int'(free_ptr) + n_free) % `FREE_LIST_SZ);
      alloc_tag[i] = freelist_mem[alloc_idx[i]];
      n_alloc += int'(alloc_req[i]);
      n_free += int'(free_en[i]);
    end
  end

  assign alloc_ok = (int'(free_count) >= n_alloc);

  // ---- sequential: next cycle's state ----
  always_ff @(posedge clock) begin
    if (reset) begin
      for (int i = 0; i < `FREE_LIST_SZ; i++) freelist_mem[i] <= TAG_W'(`ARCH_REG_SZ + i);
      alloc_ptr  <= '0;
      free_ptr   <= '0;
      free_count <= `FREE_LIST_SZ;
    end else begin
      for (int j = 0; j < W; j++) if (free_en[j]) freelist_mem[free_idx[j]] <= free_tag[j];

      free_ptr <= PTR_W'((int'(free_ptr) + n_free) % `FREE_LIST_SZ);
      if (alloc_ok) alloc_ptr <= PTR_W'((int'(alloc_ptr) + n_alloc) % `FREE_LIST_SZ);
      free_count <= $bits(free_count)'(int'(free_count) + n_free - (alloc_ok ? n_alloc : 0));
    end
  end

  // ---- simulation checks: these fire only on bugs elsewhere in the core ----
  // synopsys translate_off
  always_ff @(posedge clock) begin
    if (!reset) begin
      assert (int'(free_count) + n_free - (alloc_ok ? n_alloc : 0) <= `FREE_LIST_SZ)
      else $error("freelist overflow: double free or bad free_en");
      for (int j = 0; j < W; j++)
      assert (!(free_en[j] && free_tag[j] == 0))
      else $error("freelist: freeing preg 0 (slot %0d)", j);
    end
  end
  // synopsys translate_on

endmodule
