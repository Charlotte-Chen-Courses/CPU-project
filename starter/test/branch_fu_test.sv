/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  branch_fu_test.sv                                   //
//                                                                     //
//  Description :  Testbench for the branch unit. The model computes   //
//                 taken / target / next PC / link on its own (spec    //
//                 bit layouts for the B/J/I immediates) and tracks    //
//                 the one-entry result register. Every cycle checks   //
//                 ready, out_valid and out_pkt. Covers all six        //
//                 conditions with signed/unsigned boundary operands,  //
//                 jal, jalr (bit 0 cleared), hold while not granted,  //
//                 back-to-back throughput, flush cancelling a result, //
//                 and random traffic. Targets are kept 4-byte aligned.//
//                 Prints @@@ Passed/Failed.                           //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

module testbench;

    logic             clock, reset, flush;
    logic             in_valid;
    RS_PACKET         in_pkt;
    logic             ready;
    logic             out_valid;
    CDB_PACKET        out_pkt;
    logic             out_grant;
    logic [`XLEN-1:0] rs1_value, rs2_value;

    branch_fu dut (
        .clock    (clock),
        .reset    (reset),
        .in_valid (in_valid),
        .in_pkt   (in_pkt),
        .ready    (ready),
        .out_valid(out_valid),
        .out_pkt  (out_pkt),
        .out_grant(out_grant),
        .flush    (flush),
        .rs1_value(rs1_value),
        .rs2_value(rs2_value)
    );

    always #5 clock = ~clock;

    localparam logic [6:0] OP_BR = 7'b1100011, OP_JAL = 7'b1101111, OP_JALR = 7'b1100111;
    logic [2:0] conds [] = '{3'b000, 3'b001, 3'b100, 3'b101, 3'b110, 3'b111};

    bit        m_valid;
    CDB_PACKET m_pkt;
    int        errors = 0, n_results = 0, n_taken = 0, n_misaligned = 0;

    task automatic fail(input string msg);
        $display("  FAIL: %s", msg);
        errors++;
    endtask

    // independent model
    function automatic CDB_PACKET model(input RS_PACKET p, input logic [31:0] a, input logic [31:0] b);
        logic [31:0] i = p.inst, link = p.PC + 4, tgt;
        bit t;
        CDB_PACKET r;
        case (i[6:0])
            OP_JAL:  begin t = 1; tgt = p.PC + {{11{i[31]}}, i[31], i[19:12], i[20], i[30:21], 1'b0}; end
            OP_JALR: begin t = 1; tgt = (a + {{20{i[31]}}, i[31:20]}) & 32'hffff_fffe; end
            default: begin
                case (i[14:12])
                    3'b000: t = (a == b);
                    3'b001: t = (a != b);
                    3'b100: t = ($signed(a) <  $signed(b));
                    3'b101: t = ($signed(a) >= $signed(b));
                    3'b110: t = (a <  b);
                    default: t = (a >= b);
                endcase
                tgt = p.PC + {{19{i[31]}}, i[31], i[7], i[30:25], i[11:8], 1'b0};
            end
        endcase
        r = '{tag: p.dest_tag, value: link, rob_idx: p.rob_idx, branch: 1, next_pc: t ? tgt : link,
              exception: (t && (tgt[1:0] != 2'b00)) ? EXC_MISALIGNED : EXC_NONE, fflags: '0};
        return r;
    endfunction

    function automatic logic [31:0] operand();
        case ($urandom % 8)
            0: return 32'h0000_0000;
            1: return 32'hffff_ffff;
            2: return 32'h8000_0000;
            3: return 32'h7fff_ffff;
            4: return 32'h0000_0001;
            default: return $urandom;
        endcase
    endfunction

    task automatic idle();
        in_valid = 0; in_pkt = '0; rs1_value = '0; rs2_value = '0;
        out_grant = 1; flush = 0;
    endtask

    // drive one branch/jump; immediates chosen so the target stays 4-byte aligned
    task automatic issue(input logic [6:0] op, input logic [2:0] f3 = 3'b000, input bit misalign_ok = 0);
        logic [31:0] i = $urandom;
        i[6:0] = op;
        if (op == OP_BR)  begin i[14:12] = f3; i[8] = 0; end   // B-imm bit 1
        if (op == OP_JAL)  i[21] = 0;                           // J-imm bit 1
        if (op == OP_JALR) begin i[14:12] = 3'b000; i[21] = 0; end // I-imm bit 1 (rs1 aligned below)
        if (misalign_ok && $urandom % 4 == 0) i[op == OP_BR ? 8 : 21] = 1;   // misaligned target
        in_valid        = 1;
        in_pkt          = '0;
        in_pkt.inst     = i;
        in_pkt.PC       = $urandom & 32'hffff_fffc;
        in_pkt.NPC      = in_pkt.PC + 4;
        in_pkt.dest_tag = `TAG_W'($urandom);
        in_pkt.rob_idx  = `ROB_IDX_W'($urandom);
        in_pkt.fu_type  = FU_BRANCH;
        rs1_value       = (op == OP_JALR) ? ($urandom & (misalign_ok ? 32'hffff_ffff : 32'hffff_fffc)) : operand();
        rs2_value       = ($urandom % 4 == 0) ? rs1_value : operand();     // often equal
    endtask

    task automatic step(input string what);
        #1;
        if (ready !== (!m_valid || out_grant))
            fail($sformatf("[%s] ready=%b (holding=%b, grant=%b)", what, ready, m_valid, out_grant));
        if (out_valid !== m_valid)
            fail($sformatf("[%s] out_valid=%b expected %b", what, out_valid, m_valid));
        if (m_valid && out_pkt !== m_pkt)
            fail($sformatf("[%s] got next_pc=%h link=%h tag=%0d rob=%0d br=%b, expected next_pc=%h link=%h tag=%0d rob=%0d",
                           what, out_pkt.next_pc, out_pkt.value, out_pkt.tag, out_pkt.rob_idx, out_pkt.branch,
                           m_pkt.next_pc, m_pkt.value, m_pkt.tag, m_pkt.rob_idx));
        if (m_valid && out_grant) n_results++;
        @(posedge clock);
        if (flush) m_valid = 0;
        else if (!m_valid || out_grant) begin
            m_valid = in_valid;
            if (in_valid) begin
                m_pkt = model(in_pkt, rs1_value, rs2_value);
                if (m_pkt.next_pc != m_pkt.value) n_taken++;
                if (m_pkt.exception == EXC_MISALIGNED) n_misaligned++;
            end
        end
        @(negedge clock);
    endtask

    initial begin
        clock = 0;
        idle();
        m_valid = 0;
        reset = 1;
        @(negedge clock);
        @(negedge clock);
        reset = 0;

        $display("Test 1: empty after reset");
        idle(); step("reset");

        $display("Test 2: every condition, boundary operands");
        foreach (conds[c])
            for (int n = 0; n < 300; n++) begin
                idle(); issue(OP_BR, conds[c]);
                step($sformatf("funct3=%b", conds[c]));
            end

        $display("Test 3: jal and jalr");
        for (int n = 0; n < 300; n++) begin idle(); issue(OP_JAL);  step("jal");  end
        for (int n = 0; n < 300; n++) begin idle(); issue(OP_JALR); step("jalr"); end

        $display("Test 4: hold result while not granted");
        idle(); issue(OP_JAL); step("issue");
        for (int c = 0; c < 4; c++) begin idle(); issue(OP_BR, 3'b000); out_grant = 0; step("held"); end
        idle(); step("granted");
        idle(); step("empty");

        $display("Test 5: back-to-back throughput");
        begin
            automatic int start_n = n_results;
            for (int c = 0; c < 100; c++) begin idle(); issue(OP_BR, conds[$urandom_range(0, 5)]); step("b2b"); end
            idle(); step("drain");
            if (n_results - start_n != 100) fail($sformatf("back-to-back: %0d results in 100 cycles", n_results - start_n));
        end

        $display("Test 6: flush drops a held result and the one being captured");
        idle(); issue(OP_JAL); step("issue");
        idle(); issue(OP_JAL); out_grant = 0; step("held");
        idle(); issue(OP_JAL); out_grant = 0; flush = 1; step("flush");
        idle(); step("after flush");

        $display("Test 7: 20000 random cycles");
        for (int c = 0; c < 20000; c++) begin
            idle();
            if ($urandom % 4 != 0)
                begin
                    automatic int kind = $urandom_range(0, 3);
                    automatic int cd   = $urandom_range(0, 5);
                    case (kind)
                        0: issue(OP_JAL, 3'b000, 1);
                        1: issue(OP_JALR, 3'b000, 1);
                        default: issue(OP_BR, conds[cd], 1);
                    endcase
                end
            out_grant = ($urandom % 4 != 0);
            flush     = ($urandom % 50 == 0);
            step("random");
        end
        idle(); step("drain");

        $display("  (%0d results checked, %0d taken, %0d misaligned)", n_results, n_taken, n_misaligned);
        if (errors == 0) $display("@@@ Passed");
        else             $display("@@@ Failed (%0d errors)", errors);
        $finish;
    end

endmodule
