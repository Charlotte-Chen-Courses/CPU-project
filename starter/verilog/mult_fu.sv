/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  mult_fu.sv                                          //
//                                                                     //
//  Description :  Multiply functional unit: mul, mulh, mulhsu, mulhu. //
//                 Wraps the P3 pipelined 64x64 multiplier (mult.sv,   //
//                 MULT_STAGES stages), accepting one op per cycle.    //
//                 The 32-bit operands are sign- or zero-extended to   //
//                 64 bits so the low 64 bits of the 64x64 product are //
//                 the exact 32x32 product: mul takes [31:0], the mulh //
//                 variants take [63:32] (mulh: both signed; mulhsu:   //
//                 rs1 signed, rs2 unsigned; mulhu: both unsigned).    //
//                 A valid bit and {tag, rob_idx, high} travel next to //
//                 each stage. Latency MULT_STAGES; the last stage is  //
//                 the result register. If it loses the CDB the whole  //
//                 pipeline stalls (ready = 0); flush drops every      //
//                 stage. Same interface as alu_fu.                    //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

module mult_fu (
    input             clock,
    input             reset,
    input             in_valid,
    input  RS_PACKET  in_pkt,
    output logic      ready,
    output logic      out_valid,
    output CDB_PACKET out_pkt,
    input             out_grant,
    input             flush,

    input [`XLEN-1:0] rs1_value,
    input [`XLEN-1:0] rs2_value
);
  localparam N = `MULT_STAGES;

  typedef struct packed {
    logic [`TAG_W-1:0]     tag;
    logic [`ROB_IDX_W-1:0] rob_idx;
    logic                  high;     // result = product[63:32] (mulh*), else [31:0]
  } MULT_META;

  logic [N-1:0]          v;          // stage k holds a valid operation
  MULT_META [N-1:0]      meta;
  logic                  stall;
  logic [63:0]           mcand, mplier, product;

  // the last stage's result cannot leave: freeze everything
  assign stall     = v[N-1] && !out_grant;
  assign ready     = !stall;
  assign out_valid = v[N-1];

  always_comb begin
    case (in_pkt.alu_func)
      ALU_MULH:   begin mcand = {{32{rs1_value[31]}}, rs1_value}; mplier = {{32{rs2_value[31]}}, rs2_value}; end
      ALU_MULHSU: begin mcand = {{32{rs1_value[31]}}, rs1_value}; mplier = {32'b0, rs2_value}; end
      default:    begin mcand = {32'b0, rs1_value};               mplier = {32'b0, rs2_value}; end  // mul, mulhu
    endcase
  end

  /* verilator lint_off PINCONNECTEMPTY */
  mult mult_0 (
      .clock (clock),
      .reset (reset),
      .mcand (mcand),
      .mplier(mplier),
      .start (in_valid),
      .en    (!stall),
      .product(product),
      .done  ()                  // validity is tracked in v[] (it also has to be flushed)
  );
  /* verilator lint_on PINCONNECTEMPTY */

  always_ff @(posedge clock) begin
    if (reset || flush) begin
      v <= '0;
    end else if (!stall) begin
      v    <= {v[N-2:0], in_valid};
      meta <= {meta[N-2:0], MULT_META'{tag: in_pkt.dest_tag, rob_idx: in_pkt.rob_idx,
                                       high: in_pkt.alu_func != ALU_MUL}};
    end
  end

  assign out_pkt = '{tag: meta[N-1].tag,
                     value: meta[N-1].high ? product[63:32] : product[31:0],
                     rob_idx: meta[N-1].rob_idx,
                     branch: `FALSE, next_pc: '0, exception: EXC_NONE};

endmodule  // mult_fu
