/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  prf_test.sv                                         //
//                                                                     //
//  Description :  Basic testbench for the physical register file.     //
//                 Checks every read port against a reference model:   //
//                 write-then-read, same-cycle bypass, preg 0, and a   //
//                 fill-all/read-all sweep; writes to the other        //
//                 register class's tags are ignored and reads of them //
//                 return 0. +define+FP_CLASS: the FP file (tags       //
//                 offset by FP_TAG_BASE through an adapter, preg 0 an //
//                 ordinary register, one more read port for rs3).     //
//                 Prints @@@ Passed/Failed.                           //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

module testbench;

    localparam WR_SZ   = `NUM_CDB;
    localparam TAG_W   = `TAG_W;
`ifdef FP_CLASS
    localparam RD_SZ   = 2 * `ISSUE_WIDTH + 1;
    localparam NUM_REG = `FP_PHYS_REG_SZ, BASE = `FP_TAG_BASE, OTHER = 0;
    localparam ZR = 0;
`else
    localparam RD_SZ   = 2 * `ISSUE_WIDTH;
    localparam NUM_REG = `PHYS_REG_SZ, BASE = 0, OTHER = `FP_TAG_BASE;
    localparam ZR = 1;
`endif

    logic                               clock;
    logic        [RD_SZ-1:0][TAG_W-1:0] rd_tag;
    logic        [RD_SZ-1:0][`XLEN-1:0] rd_data;
    logic        [WR_SZ-1:0]            wr_en;
    logic        [WR_SZ-1:0][TAG_W-1:0] wr_tag;
    logic        [WR_SZ-1:0][`XLEN-1:0] wr_data;

    // reference model: what each physical register should hold
    logic [`XLEN-1:0] model [NUM_REG];
    int errors = 0;

    // class-local tags here; the DUT sees them offset by BASE (or by the other class's base)
    bit other_class = 0;
    logic [RD_SZ-1:0][TAG_W-1:0] d_rd_tag;
    logic [WR_SZ-1:0][TAG_W-1:0] d_wr_tag;
    always_comb begin
        for (int r = 0; r < RD_SZ; r++) d_rd_tag[r] = rd_tag[r] + TAG_W'(other_class ? OTHER : BASE);
        for (int w = 0; w < WR_SZ; w++) d_wr_tag[w] = wr_tag[w] + TAG_W'(other_class ? OTHER : BASE);
    end

    prf #(.RD_SZ(RD_SZ), .BASE(BASE), .SZ(NUM_REG), .ZERO_REG(ZR == 1)) dut (
        .clock  (clock),
        .rd_tag (d_rd_tag),
        .rd_data(rd_data),
        .wr_en  (wr_en),
        .wr_tag (d_wr_tag),
        .wr_data(wr_data)
    );

    always #5 clock = ~clock;

    // expected value on a read port, including same-cycle bypass
    function automatic logic [`XLEN-1:0] expected(input logic [TAG_W-1:0] tag);
        if (other_class || (ZR && tag == 0)) return '0;
        for (int w = 0; w < WR_SZ; w++)
            if (wr_en[w] && wr_tag[w] == tag) return wr_data[w];
        return model[tag];
    endfunction

    // compare all read ports against the model (call while inputs are stable)
    task automatic check_reads(input string what);
        for (int r = 0; r < RD_SZ; r++) begin
            if (rd_data[r] !== expected(rd_tag[r])) begin
                $display("  FAIL [%s] port %0d tag %0d: got %h, expected %h",
                         what, r, rd_tag[r], rd_data[r], expected(rd_tag[r]));
                errors++;
            end
        end
    endtask

    // apply inputs on negedge, check, then let posedge commit the write
    task automatic cycle(input string what);
        #1 check_reads(what);
        @(posedge clock);
        for (int w = 0; w < WR_SZ; w++)
            if (wr_en[w] && !other_class && (!ZR || wr_tag[w] != 0)) model[wr_tag[w]] = wr_data[w];
        @(negedge clock);
    endtask

    task automatic idle();
        wr_en = '0; wr_tag = '0; wr_data = '0;
    endtask

    initial begin
        clock = 0;
        idle();
        rd_tag = '0;
        for (int i = 0; i < NUM_REG; i++) model[i] = 'x;
        if (ZR) model[0] = '0;
        @(negedge clock);

        // 1) write a register, read it back next cycle
        $display("Test 1: write then read");
        wr_en[0] = 1; wr_tag[0] = 5; wr_data[0] = 32'hDEAD_BEEF;
        cycle("write p5");
        idle();
        rd_tag = '{default: 5};
        cycle("read p5");

        // 2) same-cycle write + read must bypass the new value
        $display("Test 2: same-cycle bypass");
        wr_en[0] = 1; wr_tag[0] = 7; wr_data[0] = 32'h1234_5678;
        rd_tag = '{default: 7};
        cycle("bypass p7");
        idle();
        cycle("read p7 after bypass");

        // 3) preg 0 reads zero and ignores writes (also no bypass on tag 0); FP: an ordinary register
        if (ZR) $display("Test 3: preg 0 hardwired to zero");
        else    $display("Test 3: preg 0 (initial f0) holds what is written");
        wr_en[0] = 1; wr_tag[0] = 0; wr_data[0] = 32'hFFFF_FFFF;
        rd_tag = '{default: 0};
        cycle("write p0");
        idle();
        cycle("read p0");

        // 4) fill every register, then read every register on every port
        $display("Test 4: fill all %0d registers, read all back", NUM_REG);
        for (int t = ZR; t < NUM_REG; t += WR_SZ) begin
            idle();
            for (int w = 0; w < WR_SZ && t + w < NUM_REG; w++) begin
                wr_en[w] = 1; wr_tag[w] = TAG_W'(t + w); wr_data[w] = $urandom;
            end
            cycle("fill");
        end
        idle();
        for (int t = 0; t < NUM_REG; t++) begin
            for (int r = 0; r < RD_SZ; r++) rd_tag[r] = TAG_W'((t + r) % NUM_REG);
            cycle("readback");
        end

        // 5) random reads/writes, so bypass and plain reads mix
        $display("Test 5: 1000 random cycles");
        for (int i = 0; i < 1000; i++) begin
            idle();
            for (int w = 0; w < WR_SZ; w++) begin
                // distinct write tags per cycle: renaming never gives two CDBs the same tag
                wr_en[w] = 1'($urandom % 2); wr_tag[w] = TAG_W'((i * WR_SZ + w) % NUM_REG);
                wr_data[w] = $urandom;
            end
            for (int r = 0; r < RD_SZ; r++) rd_tag[r] = TAG_W'($urandom % NUM_REG);
            cycle("random");
        end

        // 6) the other class's tags: writes land nowhere here, reads return 0 (even with a bypass)
        $display("Test 6: tags of the other register class are ignored");
        other_class = 1;
        for (int t = 1; t < 40; t += WR_SZ) begin
            idle();
            for (int w = 0; w < WR_SZ; w++) begin wr_en[w] = 1; wr_tag[w] = TAG_W'(t + w); wr_data[w] = $urandom; end
            for (int r = 0; r < RD_SZ; r++) rd_tag[r] = TAG_W'(t + r % WR_SZ);
            cycle("other class");
        end
        other_class = 0;
        idle();
        for (int t = 0; t < NUM_REG; t++) begin
            for (int r = 0; r < RD_SZ; r++) rd_tag[r] = TAG_W'((t + r) % NUM_REG);
            cycle("this class unchanged");
        end

        if (errors == 0) $display("@@@ Passed");
        else             $display("@@@ Failed (%0d errors)", errors);
        $finish;
    end

endmodule
