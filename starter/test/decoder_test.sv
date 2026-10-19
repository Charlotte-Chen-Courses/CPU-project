/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  decoder_test.sv                                     //
//                                                                     //
//  Description :  Testbench for the RV32IMAF decoder. Encodes one or  //
//                 more real instances of each instruction (bit        //
//                 layouts built here from the RISC-V spec, with       //
//                 random registers/immediates and every legal         //
//                 rounding mode) and checks the classification:       //
//                 fu_type, has_dest, rd_mem/wr_mem, branch flags,     //
//                 register classes (rs1_fp/rs2_fp/rd_fp), rs3_used,   //
//                 atomic, csr_op, halt, illegal. Also checks reserved //
//                 rounding modes are illegal, valid=0 gives a NOP,    //
//                 and unknown opcodes are illegal.                    //
//                 Prints @@@ Passed/Failed.                           //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

module testbench;

    INST           inst;
    logic          valid;
    ALU_OPA_SELECT opa_select;
    ALU_OPB_SELECT opb_select;
    logic          has_dest;
    ALU_FUNC       alu_func;
    logic          rd_mem, wr_mem, cond_branch, uncond_branch, csr_op, halt, illegal;
    FU_TYPE        fu_type;
    logic          rs1_fp, rs2_fp, rd_fp, rs3_used, atomic;

    decoder dut (
        .inst(inst), .valid(valid),
        .opa_select(opa_select), .opb_select(opb_select), .has_dest(has_dest), .alu_func(alu_func),
        .rd_mem(rd_mem), .wr_mem(wr_mem), .cond_branch(cond_branch), .uncond_branch(uncond_branch),
        .csr_op(csr_op), .halt(halt), .illegal(illegal),
        .fu_type(fu_type), .rs1_fp(rs1_fp), .rs2_fp(rs2_fp), .rd_fp(rd_fp),
        .rs3_used(rs3_used), .atomic(atomic)
    );

    int errors = 0, n_checks = 0;

    // ---- encoders (RISC-V base formats) with random register / immediate fields ----
    function automatic logic [31:0] R(input logic [6:0] f7, input logic [2:0] f3, input logic [6:0] op,
                                      input int rs2 = -1);
        logic [4:0] r2 = (rs2 < 0) ? 5'($urandom) : 5'(rs2);
        return {f7, r2, 5'($urandom), f3, 5'($urandom), op};
    endfunction
    function automatic logic [31:0] I(input logic [2:0] f3, input logic [6:0] op);
        return {12'($urandom), 5'($urandom), f3, 5'($urandom), op};
    endfunction
    function automatic logic [31:0] S(input logic [2:0] f3, input logic [6:0] op);
        return {7'($urandom), 5'($urandom), 5'($urandom), f3, 5'($urandom), op};
    endfunction
    function automatic logic [31:0] R4(input logic [2:0] rm, input logic [6:0] op);
        return {5'($urandom), 2'b00, 5'($urandom), 5'($urandom), rm, 5'($urandom), op};
    endfunction
    // AMO: funct5, aq/rl random
    function automatic logic [31:0] AMO(input logic [4:0] f5, input int rs2 = -1);
        return R({f5, 2'($urandom)}, 3'b010, 7'b0101111, rs2);
    endfunction

    localparam logic [6:0] OP_FP = 7'b1010011;

    // expected classification
    typedef struct {
        FU_TYPE fu;
        bit hd, rm, wm, cb, ub, r1f, r2f, rdf, r3, at, csr, hlt, ill;
    } exp_t;

    function automatic exp_t E(input FU_TYPE fu, input bit hd,
                               input bit r1f = 0, input bit r2f = 0, input bit rdf = 0,
                               input bit rm = 0, input bit wm = 0, input bit at = 0, input bit r3 = 0);
        exp_t e = '{fu: fu, hd: hd, rm: rm, wm: wm, cb: 0, ub: 0, r1f: r1f, r2f: r2f, rdf: rdf,
                    r3: r3, at: at, csr: 0, hlt: 0, ill: 0};
        return e;
    endfunction

    task automatic check(input string name, input logic [31:0] w, input exp_t e);
        inst = w; valid = 1;
        #1;
        n_checks++;
        if (fu_type !== e.fu || has_dest !== e.hd || rd_mem !== e.rm || wr_mem !== e.wm ||
            cond_branch !== e.cb || uncond_branch !== e.ub || rs1_fp !== e.r1f || rs2_fp !== e.r2f ||
            rd_fp !== e.rdf || rs3_used !== e.r3 || atomic !== e.at || csr_op !== e.csr ||
            halt !== e.hlt || illegal !== e.ill) begin
            errors++;
            $display("  FAIL: %s (%h)", name, w);
            $display("        got  fu=%s hd=%b rm=%b wm=%b cb=%b ub=%b fp=%b%b%b r3=%b at=%b csr=%b halt=%b ill=%b",
                     fu_type.name(), has_dest, rd_mem, wr_mem, cond_branch, uncond_branch,
                     rs1_fp, rs2_fp, rd_fp, rs3_used, atomic, csr_op, halt, illegal);
            $display("        want fu=%s hd=%b rm=%b wm=%b cb=%b ub=%b fp=%b%b%b r3=%b at=%b csr=%b halt=%b ill=%b",
                     e.fu.name(), e.hd, e.rm, e.wm, e.cb, e.ub, e.r1f, e.r2f, e.rdf, e.r3, e.at, e.csr, e.hlt, e.ill);
        end
    endtask

    // check an instruction with every legal rounding mode, and the two reserved ones as illegal
    task automatic check_rm(input string name, input logic [6:0] f7, input exp_t e, input int rs2 = -1);
        logic [2:0] rms [] = '{3'b000, 3'b001, 3'b010, 3'b011, 3'b100, 3'b111};
        foreach (rms[i]) check($sformatf("%s rm=%b", name, rms[i]), R(f7, rms[i], OP_FP, rs2), e);
        // reserved rm: illegal; the other flags are don't-care, so only check illegal
        for (logic [2:0] r = 3'b101; r <= 3'b110; r++) begin
            inst = R(f7, r, OP_FP, rs2); valid = 1; #1; n_checks++;
            if (illegal !== 1'b1) begin errors++; $display("  FAIL: %s rm=%b should be illegal", name, r); end
        end
    endtask

    initial begin
        exp_t e;

        for (int rep = 0; rep < 20; rep++) begin
            // ---- unchanged base cases, now with fu_type ----
            check("addi",  I(3'b000, 7'b0010011), E(FU_ALU, 1));
            check("add",   R(7'b0000000, 3'b000, 7'b0110011), E(FU_ALU, 1));
            check("lw",    I(3'b010, 7'b0000011), E(FU_LOAD, 1, .rm(1)));
            check("sw",    S(3'b010, 7'b0100011), E(FU_STORE, 0, .wm(1)));
            e = E(FU_ALU, 0); e.cb = 1;
            check("beq",   S(3'b000, 7'b1100011), e);
            e = E(FU_ALU, 1); e.ub = 1;
            check("jal",   {20'($urandom), 5'($urandom), 7'b1101111}, e);

            // ---- M ----
            check("mul",    R(7'b0000001, 3'b000, 7'b0110011), E(FU_MULT, 1));
            check("mulhu",  R(7'b0000001, 3'b011, 7'b0110011), E(FU_MULT, 1));
            check("div",    R(7'b0000001, 3'b100, 7'b0110011), E(FU_MULT, 1));
            check("divu",   R(7'b0000001, 3'b101, 7'b0110011), E(FU_MULT, 1));
            check("rem",    R(7'b0000001, 3'b110, 7'b0110011), E(FU_MULT, 1));
            check("remu",   R(7'b0000001, 3'b111, 7'b0110011), E(FU_MULT, 1));

            // ---- FENCE / CSR ----
            check("fence",   I(3'b000, 7'b0001111), E(FU_ALU, 0));
            check("fence.i", I(3'b001, 7'b0001111), E(FU_ALU, 0));
            for (int f3 = 1; f3 < 8; f3++) begin
                if (f3 == 4) continue;
                e = E(FU_ALU, 0); e.csr = 1;
                check($sformatf("csr f3=%0d", f3), I(3'(f3), 7'b1110011), e);
            end

            // ---- A ----
            check("lr.w", AMO(5'b00010, 0), E(FU_LOAD, 1, .rm(1), .at(1)));
            check("sc.w", AMO(5'b00011),    E(FU_STORE, 1, .wm(1), .at(1)));
            begin
                automatic logic [4:0] amos [] = '{5'b00001, 5'b00000, 5'b00100, 5'b01100, 5'b01000,
                                        5'b10000, 5'b10100, 5'b11000, 5'b11100};
                foreach (amos[k])
                    check($sformatf("amo funct5=%b", amos[k]), AMO(amos[k]),
                          E(FU_STORE, 1, .rm(1), .wm(1), .at(1)));
            end

            // ---- F: memory ----
            check("flw", I(3'b010, 7'b0000111), E(FU_LOAD, 1, .rdf(1), .rm(1)));
            check("fsw", S(3'b010, 7'b0100111), E(FU_STORE, 0, .r2f(1), .wm(1)));

            // ---- F: FP -> FP (rounding mode) ----
            check_rm("fadd.s",  7'b0000000, E(FU_FP, 1, 1, 1, 1));
            check_rm("fsub.s",  7'b0000100, E(FU_FP, 1, 1, 1, 1));
            check_rm("fmul.s",  7'b0001000, E(FU_FP, 1, 1, 1, 1));
            check_rm("fdiv.s",  7'b0001100, E(FU_FP, 1, 1, 1, 1));
            check_rm("fsqrt.s", 7'b0101100, E(FU_FP, 1, 1, 0, 1), 0);

            // ---- F: FP -> FP (funct3 is the op) ----
            check("fsgnj.s",  R(7'b0010000, 3'b000, OP_FP), E(FU_FP, 1, 1, 1, 1));
            check("fsgnjn.s", R(7'b0010000, 3'b001, OP_FP), E(FU_FP, 1, 1, 1, 1));
            check("fsgnjx.s", R(7'b0010000, 3'b010, OP_FP), E(FU_FP, 1, 1, 1, 1));
            check("fmin.s",   R(7'b0010100, 3'b000, OP_FP), E(FU_FP, 1, 1, 1, 1));
            check("fmax.s",   R(7'b0010100, 3'b001, OP_FP), E(FU_FP, 1, 1, 1, 1));

            // ---- F: fused multiply-add (3 FP sources) ----
            begin
                automatic logic [2:0] rms [] = '{3'b000, 3'b001, 3'b010, 3'b011, 3'b100, 3'b111};
                automatic logic [6:0] fops [] = '{7'b1000011, 7'b1000111, 7'b1001011, 7'b1001111};
                foreach (fops[o]) begin
                    foreach (rms[k])
                        check($sformatf("fused op=%b rm=%b", fops[o], rms[k]), R4(rms[k], fops[o]),
                              E(FU_FP, 1, 1, 1, 1, .r3(1)));
                    for (logic [2:0] r = 3'b101; r <= 3'b110; r++) begin
                        inst = R4(r, fops[o]); valid = 1; #1; n_checks++;
                        if (illegal !== 1'b1) begin errors++; $display("  FAIL: fused rm=%b should be illegal", r); end
                    end
                end
            end

            // ---- F: FP sources -> integer result ----
            check("feq.s",    R(7'b1010000, 3'b010, OP_FP), E(FU_FP, 1, 1, 1, 0));
            check("flt.s",    R(7'b1010000, 3'b001, OP_FP), E(FU_FP, 1, 1, 1, 0));
            check("fle.s",    R(7'b1010000, 3'b000, OP_FP), E(FU_FP, 1, 1, 1, 0));
            check("fclass.s", R(7'b1110000, 3'b001, OP_FP, 0), E(FU_FP, 1, 1, 0, 0));
            check("fmv.x.w",  R(7'b1110000, 3'b000, OP_FP, 0), E(FU_FP, 1, 1, 0, 0));
            check_rm("fcvt.w.s",  7'b1100000, E(FU_FP, 1, 1, 0, 0), 0);
            check_rm("fcvt.wu.s", 7'b1100000, E(FU_FP, 1, 1, 0, 0), 1);

            // ---- F: integer source -> FP result ----
            check("fmv.w.x",  R(7'b1111000, 3'b000, OP_FP, 0), E(FU_FP, 1, 0, 0, 1));
            check_rm("fcvt.s.w",  7'b1101000, E(FU_FP, 1, 0, 0, 1), 0);
            check_rm("fcvt.s.wu", 7'b1101000, E(FU_FP, 1, 0, 0, 1), 1);

            // ---- illegal: FP op with an rs2 field that selects nothing ----
            inst = R(7'b1100000, 3'b000, OP_FP, 5); valid = 1; #1; n_checks++;
            if (illegal !== 1'b1) begin errors++; $display("  FAIL: fcvt with rs2=00101 should be illegal"); end
        end

        // ---- halt, NOP, unknown opcode ----
        e = E(FU_ALU, 0); e.hlt = 1;
        check("wfi", 32'h10500073, e);
        inst = $urandom; valid = 0; #1; n_checks++;
        if (has_dest || rd_mem || wr_mem || illegal || halt || atomic || rs1_fp || rs2_fp || rd_fp || rs3_used
            || fu_type != FU_ALU) begin errors++; $display("  FAIL: valid=0 is not a NOP"); end
        inst = {25'($urandom), 7'b1111111}; valid = 1; #1; n_checks++;
        if (!illegal) begin errors++; $display("  FAIL: unknown opcode not illegal"); end

        $display("  (%0d instruction checks)", n_checks);
        if (errors == 0) $display("@@@ Passed");
        else             $display("@@@ Failed (%0d errors)", errors);
        $finish;
    end

endmodule
