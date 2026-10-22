/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  fp_fu_test.sv                                       //
//                                                                     //
//  Description :  Testbench for the FP unit (fp_fu.sv around CVFPU).  //
//                 Every result is checked against Spike's SoftFloat   //
//                 (test/fp_ref.c over DPI-C): value bits, the five    //
//                 exception flags (in the CDB packet), and illegal    //
//                 instruction for rm = DYN with an invalid frm (5-7), //
//                 matched by rob_idx since operation                  //
//                 groups finish out of order. Operands mix random     //
//                 bits with special values (+-0, +-inf, quiet and     //
//                 signalling NaN, subnormals, extreme normals, near   //
//                 cancellation, integer edges for fcvt), every static //
//                 rounding mode and DYN with a random frm. The CDB    //
//                 grant is random and the unit is flushed at random;  //
//                 a flushed op must never come out. Directed: the     //
//                 latency of each operation group, one fadd per cycle //
//                 throughput, adds overtaking a divide, and ready     //
//                 depending only on state; CSR ops on fflags/frm/fcsr //
//                 (fcsr write and old value, CDB stalls); a stretch   //
//                 of fdiv/fsqrt                                       //
//                 only with no flushes (catches lost or mixed-up      //
//                 operations in the iterative unit). Prints @@@       //
//                 Passed/Failed.                                      //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

/* verilator lint_off BLKSEQ */
/* verilator lint_off PROCASSINIT */

module testbench;

    import "DPI-C" function int unsigned fp_ref(input int unsigned inst, input int unsigned a, input int unsigned b,
                                                input int unsigned c, input int unsigned frm, output int unsigned flags);

    logic             clock, reset, in_valid, ready, out_valid, out_grant, flush;
    RS_PACKET         in_pkt;
    CDB_PACKET        out_pkt;
    logic [`XLEN-1:0] rs1_value, rs2_value, rs3_value;
    logic [2:0]       frm;
    logic [4:0]       tb_fflags = 0;
    logic [7:0]       fcsr, fcsr_wdata;
    logic             fcsr_we;
    assign fcsr = {frm, tb_fflags};

    fp_fu dut (.*);

    always #5 clock = ~clock;

    // ---------------- instructions ----------------
    typedef enum int { I_ADD, I_SUB, I_MUL, I_DIV, I_SQRT, I_SGNJ, I_MINMAX, I_CMP, I_CVT_WS, I_CVT_SW,
                       I_MVXW, I_CLASS, I_MVWX, I_FMADD, I_FMSUB, I_FNMSUB, I_FNMADD, I_N } kind_e;
    string kname [I_N] = '{"fadd", "fsub", "fmul", "fdiv", "fsqrt", "fsgnj", "fmin/max", "fcmp", "fcvt.w.s", "fcvt.s.w",
                           "fmv.x.w", "fclass", "fmv.w.x", "fmadd", "fmsub", "fnmsub", "fnmadd"};

    function automatic logic [2:0] rand_rm();
        logic [2:0] rms [6] = '{3'd0, 3'd1, 3'd2, 3'd3, 3'd4, 3'd7};
        return rms[$urandom_range(0, 5)];
    endfunction

    function automatic logic [31:0] opfp(input logic [4:0] f5, input logic [4:0] rs2, input logic [2:0] f3);
        return {f5, 2'b00, rs2, 5'($urandom), f3, 5'($urandom), 7'b1010011};
    endfunction

    function automatic logic [31:0] encode(input kind_e k);
        case (k)
            I_ADD:    return opfp(5'b00000, 5'($urandom), rand_rm());
            I_SUB:    return opfp(5'b00001, 5'($urandom), rand_rm());
            I_MUL:    return opfp(5'b00010, 5'($urandom), rand_rm());
            I_DIV:    return opfp(5'b00011, 5'($urandom), rand_rm());
            I_SQRT:   return opfp(5'b01011, 5'd0, rand_rm());
            I_SGNJ:   return opfp(5'b00100, 5'($urandom), 3'($urandom_range(0, 2)));
            I_MINMAX: return opfp(5'b00101, 5'($urandom), 3'($urandom_range(0, 1)));
            I_CMP:    return opfp(5'b10100, 5'($urandom), 3'($urandom_range(0, 2)));
            I_CVT_WS: return opfp(5'b11000, 5'($urandom_range(0, 1)), rand_rm());
            I_CVT_SW: return opfp(5'b11010, 5'($urandom_range(0, 1)), rand_rm());
            I_MVXW:   return opfp(5'b11100, 5'd0, 3'b000);
            I_CLASS:  return opfp(5'b11100, 5'd0, 3'b001);
            I_MVWX:   return opfp(5'b11110, 5'd0, 3'b000);
            default: begin
                automatic logic [6:0] op = (k == I_FMADD) ? 7'b1000011 : (k == I_FMSUB) ? 7'b1000111 :
                                           (k == I_FNMSUB) ? 7'b1001011 : 7'b1001111;
                return {5'($urandom), 2'b00, 5'($urandom), 5'($urandom), rand_rm(), 5'($urandom), op};
            end
        endcase
    endfunction

    // ---------------- operands ----------------
    logic [31:0] specials [] = '{32'h0000_0000, 32'h8000_0000, 32'h7f80_0000, 32'hff80_0000,   // +-0, +-inf
                                 32'h7fc0_0000, 32'hffc0_0001, 32'h7f80_0001, 32'hff8f_0000,   // qNaN, sNaN
                                 32'h0000_0001, 32'h807f_ffff, 32'h0040_0000,                  // subnormals
                                 32'h0080_0000, 32'h8080_0000, 32'h7f7f_ffff, 32'hff7f_ffff,   // min / max normal
                                 32'h3f80_0000, 32'hbf80_0000, 32'h3f00_0000, 32'h4040_0000,   // 1, -1, 0.5, 3
                                 32'h4f00_0000, 32'hcf00_0000, 32'h4f80_0000, 32'h4effffff,   // +-2^31, 2^32, < 2^31
                                 32'h3fc0_0000, 32'h4020_0000, 32'hbfc0_0000};                 // 1.5, 2.5, -1.5
    logic [31:0] int_specials [] = '{32'h0, 32'h1, 32'hffff_ffff, 32'h8000_0000, 32'h7fff_ffff,
                                     32'h0100_0001, 32'h0100_0003, 32'hfeff_ffff, 32'h00ff_ffff};

    function automatic logic [31:0] rand_fp();
        case ($urandom % 8)
            0, 1, 2: return specials[$urandom_range(0, specials.size() - 1)];
            3:       return {1'($urandom), 8'($urandom_range(1, 3)), 23'($urandom)};       // tiny: underflow
            4:       return {1'($urandom), 8'($urandom_range(250, 254)), 23'($urandom)};   // huge: overflow
            5:       return {1'($urandom), 8'($urandom_range(120, 134)), 23'($urandom)};   // around 1
            default: return $urandom;
        endcase
    endfunction

    // ---------------- model ----------------
    typedef struct { kind_e k; logic [31:0] inst, a, b, c, exp_v; logic [4:0] exp_f; bit illegal; logic [`TAG_W-1:0] tag; } exp_t;
    int n_illegal = 0;
    exp_t inflight [int];                          // by rob_idx
    int   next_rob = 0;
    int   errors = 0, n_done = 0, n_flushes = 0, n_flushed_ops = 0;
    int   n_kind [I_N], n_flag [5];

    task automatic fail(input string m); errors++; $display("  FAIL: %s", m); if (errors > 20) $fatal(1, "too many errors"); endtask

    // build one operation into the inputs (does not set in_valid)
    task automatic make_op(input kind_e k, input bit plain = 0);   // plain: 3.0 op 7.0, rm = RNE
        automatic exp_t e;
        automatic int unsigned fl;
        e.k = k; e.inst = encode(k);
        e.a = (k == I_CVT_SW || k == I_MVWX) ? (($urandom % 2 == 1) ? int_specials[$urandom_range(0, int_specials.size() - 1)] : $urandom)
                                             : rand_fp();
        e.b = rand_fp();
        e.c = rand_fp();
        if ((k == I_ADD || k == I_SUB) && $urandom % 4 == 0) e.b = e.a ^ (32'($urandom_range(0, 7)));   // cancellation
        if (k >= I_FMADD && $urandom % 4 == 0) e.c = e.a ^ 32'h8000_0000;
        if (plain) begin e.a = 32'h4040_0000; e.b = 32'h40e0_0000; e.inst[14:12] = 3'b000; end
        while (inflight.exists(next_rob)) next_rob = (next_rob + 1) % (1 << `ROB_IDX_W);   // a slow fdiv may hold one
        frm = ($urandom % 20 == 0) ? 3'($urandom_range(5, 7)) : 3'($urandom_range(0, 4));   // sometimes invalid
        e.exp_v = fp_ref(e.inst, e.a, e.b, e.c, 32'(frm), fl);
        e.exp_f = 5'(fl);
        // an op that rounds, with rm = DYN while frm is invalid: illegal instruction, no flags
        e.illegal = (k inside {I_ADD, I_SUB, I_MUL, I_DIV, I_SQRT, I_CVT_WS, I_CVT_SW, I_FMADD, I_FMSUB, I_FNMSUB, I_FNMADD})
                    && e.inst[14:12] == 3'b111 && frm > 3'd4;
        if (e.illegal) e.exp_f = '0;
        e.tag = `TAG_W'($urandom_range(1, `PHYS_REG_SZ - 1));
        in_pkt = '0;
        in_pkt.inst     = e.inst;
        in_pkt.dest_tag = e.tag;
        in_pkt.rob_idx  = `ROB_IDX_W'(next_rob);
        in_pkt.fu_type  = FU_FP;
        rs1_value = e.a; rs2_value = e.b; rs3_value = e.c;
        if (inflight.exists(next_rob)) $fatal(1, "testbench: rob_idx %0d reused while in flight", next_rob);
        inflight[next_rob] = e;
        next_rob = (next_rob + 1) % (1 << `ROB_IDX_W);
    endtask

    // check this cycle's output (called after the inputs settle, before the edge)
    task automatic check_out();
        if (out_valid && out_grant && !flush) begin
            automatic int r = int'(out_pkt.rob_idx);
            if (!inflight.exists(r)) fail($sformatf("result for rob %0d, which is not in flight (flushed or done)", r));
            else begin
                automatic exp_t e = inflight[r];
                if (out_pkt.exception !== (e.illegal ? EXC_ILLEGAL : EXC_NONE) || out_pkt.fflags !== e.exp_f ||
                    out_pkt.tag !== e.tag || (!e.illegal && out_pkt.value !== e.exp_v))
                    fail($sformatf("%s %h (%h, %h, %h): got %h flags %b exc %s tag %0d, expected %h flags %b exc %s tag %0d",
                                   kname[e.k], e.inst, e.a, e.b, e.c, out_pkt.value, out_pkt.fflags, out_pkt.exception.name(),
                                   out_pkt.tag, e.exp_v, e.exp_f, e.illegal ? "EXC_ILLEGAL" : "EXC_NONE", e.tag));
                n_illegal += int'(e.illegal);
                for (int f = 0; f < 5; f++) n_flag[f] += int'(e.exp_f[f]);
                n_kind[e.k]++; n_done++;
                inflight.delete(r);
            end
        end
    endtask

    // one random cycle
    bit divsqrt_only = 0;
    task automatic step(input bit allow_flush = 1, input int issue_pct = 70);
        in_valid = 0; flush = 0;
        out_grant = ($urandom % 4 != 0);
        if (allow_flush && $urandom % 400 == 0) flush = 1;
        #1;
        if (!flush && ready && inflight.size() < (1 << `ROB_IDX_W) - 1 && $urandom % 100 < issue_pct) begin
            make_op(divsqrt_only ? (($urandom % 2 == 1) ? I_DIV : I_SQRT) : kind_e'($urandom_range(0, I_N - 1)));
            in_valid = 1;
        end
        #1;
        check_out();
        @(posedge clock);
        if (flush) begin n_flushed_ops += inflight.size(); inflight.delete(); n_flushes++; end
        @(negedge clock);
    endtask

    // issue one op on an idle unit with the grant held, and return its latency in cycles
    task automatic latency(input kind_e k, output int cycles);
        in_valid = 1; flush = 0; out_grant = 1;
        make_op(k);
        cycles = 0;
        @(posedge clock); @(negedge clock); in_valid = 0;
        do begin cycles++; #1; check_out(); @(posedge clock); @(negedge clock); end while (inflight.size() > 0 && cycles < 200);
    endtask

    initial begin
        int lat;
        clock = 0; reset = 1; in_valid = 0; flush = 0; out_grant = 0; frm = 0;
        in_pkt = '0; rs1_value = 0; rs2_value = 0; rs3_value = 0;
        repeat (2) @(negedge clock); reset = 0; @(negedge clock);

        $display("Test 1: latency of each operation group (idle unit, grant held)");
        begin
            automatic kind_e ks [] = '{I_ADD, I_MUL, I_FMADD, I_SGNJ, I_CMP, I_CVT_WS, I_CVT_SW, I_DIV, I_SQRT};
            foreach (ks[i]) begin latency(ks[i], lat); $display("    %-9s %0d cycles", kname[ks[i]], lat); end
        end

        $display("Test 2: 200 back-to-back fadds, grant held: one per cycle");
        begin
            automatic int c = 0, start = n_done;
            out_grant = 1;
            for (int i = 0; i < 200; i++) begin
                in_valid = 1; make_op(I_ADD); #1;
                if (!ready) fail("fadd stream: ready dropped");
                check_out(); @(posedge clock); @(negedge clock); c++;
            end
            in_valid = 0;
            while (inflight.size() > 0 && c < 400) begin #1; check_out(); @(posedge clock); @(negedge clock); c++; end
            $display("    200 fadds in %0d cycles", c);
            if (c > 200 + `FP_ADDMUL_REGS + 3) fail("fadd throughput below one per cycle");
            if (n_done - start != 200) fail("fadd stream lost results");
        end

        $display("Test 3: fadds issued behind an fdiv finish first; ready never depends on the op");
        begin
            automatic int div_rob = next_rob, adds_before_div = 0;
            automatic bit div_done = 0;
            out_grant = 1;
            in_valid = 1; make_op(I_DIV, 1); @(posedge clock); @(negedge clock);
            for (int i = 0; i < 40 && !div_done; i++) begin
                in_valid = ready; if (ready) make_op(I_ADD);
                #1;
                if (out_valid && int'(out_pkt.rob_idx) == div_rob) div_done = 1;
                else if (out_valid) adds_before_div++;
                check_out(); @(posedge clock); @(negedge clock);
            end
            in_valid = 0;
            for (int g = 0; inflight.size() > 0 && g < 200; g++) begin #1; check_out(); @(posedge clock); @(negedge clock); end
            if (inflight.size() > 0) fail($sformatf("%0d operations never finished after the fdiv test", inflight.size()));
            inflight.delete();
            $display("    %0d fadds finished while the fdiv ran", adds_before_div);
            if (adds_before_div < 5) fail("fadds did not overtake the fdiv");
        end

        $display("Test 4: 200000 random cycles (random ops, operands, rounding modes, grant, flushes)");
        for (int c = 0; c < 200000; c++) step();

        $display("Test 5: 30000 cycles of fdiv/fsqrt only, random grant, no flushes (results held back while new ops arrive)");
        divsqrt_only = 1;
        for (int c = 0; c < 30000; c++) step(0);
        divsqrt_only = 0;

        $display("Test 6: CSR ops on fflags / frm / fcsr (issued on an idle unit, as the core does)");
        begin
            automatic int n_csr = 0, gd = 0;
            while (inflight.size() > 0 && gd++ < 500) step(0, 0);     // the core issues a CSR op only when idle
            for (int c = 0; c < 3000; c++) begin
                automatic logic [11:0] addr = 12'($urandom_range(1, 3));
                automatic logic [2:0]  f3   = ($urandom % 2 == 1) ? 3'($urandom_range(1, 3)) : 3'($urandom_range(5, 7));
                automatic logic [4:0]  uimm = 5'($urandom);
                automatic logic [31:0] r1   = $urandom;
                automatic logic [7:0]  old, src, nw, exp_fcsr;
                automatic int          g = 0;
                frm = 3'($urandom); tb_fflags = 5'($urandom);
                old = (addr == 1) ? {3'b0, tb_fflags} : (addr == 2) ? {5'b0, frm} : {frm, tb_fflags};
                src = f3[2] ? {3'b0, uimm} : r1[7:0];
                nw  = (f3[1:0] == 2'b01) ? src : (f3[1:0] == 2'b10) ? (old | src) : (old & ~src);
                exp_fcsr = (addr == 1) ? {frm, nw[4:0]} : (addr == 2) ? {nw[2:0], tb_fflags} : nw;
                in_pkt = '0;
                in_pkt.inst     = {addr, f3[2] ? uimm : 5'($urandom_range(1, 31)), f3, 5'($urandom_range(1, 31)), 7'b1110011};
                in_pkt.dest_tag = `TAG_W'($urandom_range(1, 63));
                in_pkt.rob_idx  = `ROB_IDX_W'(c);
                in_pkt.serial   = 1;
                rs1_value = r1;
                out_grant = 0; flush = 0;
                if (!ready) fail("CSR op: unit not ready while idle");
                in_valid = 1;
                #1;
                if (fcsr_we !== 1'b1 || fcsr_wdata !== exp_fcsr)
                    fail($sformatf("csr %h f3 %0d (fcsr %h, src %h): fcsr_we=%b wdata=%h, expected 1 %h",
                                   addr, f3, fcsr, src, fcsr_we, fcsr_wdata, exp_fcsr));
                @(posedge clock); @(negedge clock);
                in_valid = 0;
                #1;
                if (fcsr_we !== 1'b0) fail("fcsr_we high with no CSR op");
                if (ready) fail("ready while a CSR result is waiting");
                while (g < 5 && $urandom % 3 == 0) begin @(posedge clock); @(negedge clock); g++; end   // CDB busy for a while
                out_grant = 1;
                #1;
                if (!out_valid || out_pkt.value !== {24'b0, old} || out_pkt.tag !== in_pkt.dest_tag ||
                    out_pkt.rob_idx !== in_pkt.rob_idx || out_pkt.fflags !== '0 || out_pkt.exception !== EXC_NONE)
                    fail($sformatf("csr %h f3 %0d: result valid=%b value %h, expected %h", addr, f3, out_valid, out_pkt.value, old));
                @(posedge clock); @(negedge clock);
                out_grant = 0; #1;
                if (out_valid) fail("CSR result broadcast twice");
                n_csr++;
            end
            $display("  (%0d CSR ops checked)", n_csr);
        end

        $display("Test 7: drain");
        begin
            automatic int g = 0;
            while (inflight.size() > 0 && g++ < 500) step(0, 0);
            if (inflight.size() > 0) begin
                fail($sformatf("%0d operations never finished", inflight.size()));
                foreach (inflight[r]) $display("    rob %0d: %s %h (%h, %h, %h)", r, kname[inflight[r].k], inflight[r].inst,
                                                inflight[r].a, inflight[r].b, inflight[r].c);
            end
        end

        $display("  (%0d results checked, %0d flushes dropping %0d in-flight ops)", n_done, n_flushes, n_flushed_ops);
        for (int k = 0; k < I_N; k++) begin
            $display("    %-9s %0d", kname[k], n_kind[k]);
            if (n_kind[k] < 500) fail($sformatf("only %0d %s results", n_kind[k], kname[k]));
        end
        $display("  (%0d dynamic-rounding ops with an invalid frm raised illegal)", n_illegal);
        if (n_illegal < 100) fail("too few invalid-frm cases");
        $display("  (flags raised: NX %0d, UF %0d, OF %0d, DZ %0d, NV %0d)", n_flag[0], n_flag[1], n_flag[2], n_flag[3], n_flag[4]);
        for (int f = 0; f < 5; f++) if (n_flag[f] < 100) fail($sformatf("flag bit %0d raised only %0d times", f, n_flag[f]));
        if (errors == 0) $display("@@@ Passed");
        else             $display("@@@ Failed (%0d errors)", errors);
        $finish;
    end

endmodule
