// Port-only declaration of the ASAP7 SRAM bank for local (Yosys) synthesis: kept as a black box.
// Its area comes from the LEF outline (16.0 x 77.76 um = 1244.2 um^2), not from synthesis.
(* blackbox *)
module srambank_128x4x64_6t122 (
    input         clk,
    input  [8:0]  ADDRESS,
    input  [63:0] wd,
    input         banksel,
    input         read,
    input         write,
    output [63:0] dataout
);
endmodule
