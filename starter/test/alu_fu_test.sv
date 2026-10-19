/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  alu_fu_test.sv                                      //
//                                                                     //
//  Description :  Testbench for the ALU functional unit. A reference  //
//                 model computes each result independently (its own   //
//                 ALU and immediate decoding, not the ISA.svh macros) //
//                 and tracks the one-entry result register. Every     //
//                 cycle checks ready, out_valid and out_pkt (tag,     //
//                 value, rob_idx). Directed tests cover every         //
//                 ALU_FUNC, every opa/opb select (all five immediate  //
//                 types), holding a result while out_grant = 0, and   //
//                 back-to-back throughput; then random traffic.       //
//                 Prints @@@ Passed/Failed.                           //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

module testbench;

    logic             clock, reset;
    logic             in_valid;
    RS_PACKET         in_pkt;
    logic             ready;
    logic             out_valid;
    CDB_PACKET        out_pkt;
    logic             out_grant;
    logic [`XLEN-1:0] rs1_value, rs2_value;

    alu_fu dut (
        .clock    (clock),
        .reset    (reset),
        .in_valid (in_valid),
        .in_pkt   (in_pkt),
        .ready    (ready),
        .out_valid(out_valid),
        .out_pkt  (out_pkt),
        .out_grant(out_grant),
        .rs1_value(rs1_value),
        .rs2_value(rs2_value)
    );

    always #5 clock = ~clock;

    // the ALU functions this unit implements (multiply goes to the MULT unit)
    ALU_FUNC funcs [] = '{ALU_ADD, ALU_SUB, ALU_SLT, ALU_SLTU, ALU_AND,
                          ALU_OR, ALU_XOR, ALU_SLL, ALU_SRL, ALU_SRA};
    ALU_OPA_SELECT opas [] = '{OPA_IS_RS1, OPA_IS_NPC, OPA_IS_PC, OPA_IS_ZERO};
    ALU_OPB_SELECT opbs [] = '{OPB_IS_RS2, OPB_IS_I_IMM, OPB_IS_S_IMM,
                               OPB_IS_B_IMM, OPB_IS_U_IMM, OPB_IS_J_IMM};

    // ---- reference model ----
    bit        m_valid;
    CDB_PACKET m_pkt;
    int        errors = 0, n_results = 0;

    task automatic fail(input string msg);
        $display("  FAIL: %s", msg);
        errors++;
    endtask

    // immediates, decoded straight from the RISC-V spec bit layouts
    function automatic logic [31:0] imm(input logic [31:0] i, input ALU_OPB_SELECT sel);
        case (sel)
            OPB_IS_I_IMM: return {{20{i[31]}}, i[31:20]};
            OPB_IS_S_IMM: return {{20{i[31]}}, i[31:25], i[11:7]};
            OPB_IS_B_IMM: return {{19{i[31]}}, i[31], i[7], i[30:25], i[11:8], 1'b0};
            OPB_IS_U_IMM: return {i[31:12], 12'b0};
            OPB_IS_J_IMM: return {{11{i[31]}}, i[31], i[19:12], i[20], i[30:21], 1'b0};
            default:      return 'x;
        endcase
    endfunction

    function automatic logic [31:0] model(input RS_PACKET p, input logic [31:0] a_rs1,
                                          input logic [31:0] b_rs2);
        logic [31:0] a, b;
        case (p.opa_select)
            OPA_IS_RS1:  a = a_rs1;
            OPA_IS_NPC:  a = p.NPC;
            OPA_IS_PC:   a = p.PC;
            default:     a = 0;
        endcase
        b = (p.opb_select == OPB_IS_RS2) ? b_rs2 : imm(p.inst, p.opb_select);
        case (p.alu_func)
            ALU_ADD:  return a + b;
            ALU_SUB:  return a - b;
            ALU_SLT:  return ($signed(a) < $signed(b)) ? 1 : 0;
            ALU_SLTU: return (a < b) ? 1 : 0;
            ALU_AND:  return a & b;
            ALU_OR:   return a | b;
            ALU_XOR:  return a ^ b;
            ALU_SLL:  return a << b[4:0];
            ALU_SRL:  return a >> b[4:0];
            ALU_SRA:  return 32'($signed(a) >>> b[4:0]);
            default:  return 'x;
        endcase
    endfunction

    task automatic idle();
        in_valid = 0; in_pkt = '0; rs1_value = '0; rs2_value = '0;
        out_grant = 1;
    endtask

    // random operand: mostly random, sometimes an edge value
    function automatic logic [31:0] operand();
        case ($urandom % 8)
            0: return 32'h0000_0000;
            1: return 32'hffff_ffff;
            2: return 32'h8000_0000;
            3: return 32'h7fff_ffff;
            4: return 32'($urandom % 40);         // small: shift amounts, small compares
            default: return $urandom;
        endcase
    endfunction

    // drive one instruction with random contents; func/opa/opb chosen by the caller
    task automatic issue(input ALU_FUNC f, input ALU_OPA_SELECT a, input ALU_OPB_SELECT b);
        in_valid          = 1;
        in_pkt            = '0;
        in_pkt.inst       = $urandom;
        in_pkt.PC         = $urandom;
        in_pkt.NPC        = $urandom;
        in_pkt.dest_tag   = `TAG_W'($urandom);
        in_pkt.rob_idx    = `ROB_IDX_W'($urandom);
        in_pkt.fu_type    = FU_ALU;
        in_pkt.alu_func   = f;
        in_pkt.opa_select = a;
        in_pkt.opb_select = b;
        rs1_value         = operand();
        rs2_value         = operand();
    endtask

    // check outputs against the model, clock, update the model
    task automatic step(input string what);
        #1;
        if (ready !== (!m_valid || out_grant))
            fail($sformatf("[%s] ready=%b (holding=%b, grant=%b)", what, ready, m_valid, out_grant));
        if (out_valid !== m_valid)
            fail($sformatf("[%s] out_valid=%b expected %b", what, out_valid, m_valid));
        if (m_valid && out_pkt !== m_pkt)
            fail($sformatf("[%s] out_pkt tag=%0d value=%h rob=%0d, expected tag=%0d value=%h rob=%0d",
                           what, out_pkt.tag, out_pkt.value, out_pkt.rob_idx,
                           m_pkt.tag, m_pkt.value, m_pkt.rob_idx));
        if (m_valid && out_grant) n_results++;
        @(posedge clock);
        if (!m_valid || out_grant) begin
            m_valid = in_valid;
            if (in_valid)
                m_pkt = '{tag: in_pkt.dest_tag, value: model(in_pkt, rs1_value, rs2_value),
                          rob_idx: in_pkt.rob_idx};
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

        // 1) empty after reset
        $display("Test 1: empty after reset");
        idle(); step("reset");

        // 2) every ALU function, register operands, many random values each
        $display("Test 2: every ALU_FUNC (rs1 op rs2), %0d functions", funcs.size());
        foreach (funcs[f])
            for (int n = 0; n < 200; n++) begin
                idle(); issue(funcs[f], OPA_IS_RS1, OPB_IS_RS2);
                step(funcs[f].name());
            end

        // 3) every opa / opb select (immediates decoded from random instruction bits)
        $display("Test 3: every opa x opb select");
        foreach (opas[a])
            foreach (opbs[b])
                for (int n = 0; n < 50; n++) begin
                    idle(); issue(ALU_ADD, opas[a], opbs[b]);
                    step($sformatf("%s/%s", opas[a].name(), opbs[b].name()));
                end

        // 4) hold: result waits while out_grant = 0, ready stays low, nothing is lost
        $display("Test 4: hold result while not granted");
        idle(); issue(ALU_XOR, OPA_IS_RS1, OPB_IS_RS2);
        step("issue");
        for (int c = 0; c < 4; c++) begin
            idle(); out_grant = 0;
            issue(ALU_ADD, OPA_IS_RS1, OPB_IS_RS2);   // offered but must not be accepted
            out_grant = 0;
            step("held");
        end
        idle(); step("granted");
        idle(); step("empty");

        // 5) back-to-back: always granted -> one result per cycle
        $display("Test 5: back-to-back throughput");
        begin
            automatic int start_n = n_results;
            for (int c = 0; c < 100; c++) begin
                idle(); issue(funcs[$urandom_range(0, funcs.size() - 1)], OPA_IS_RS1, OPB_IS_RS2);
                step("back-to-back");
            end
            idle(); step("drain");
            if (n_results - start_n != 100)
                fail($sformatf("back-to-back: %0d results in 100 cycles", n_results - start_n));
        end

        // 6) random: random func/selects, random in_valid and out_grant
        $display("Test 6: 20000 random cycles");
        for (int c = 0; c < 20000; c++) begin
            idle();
            if ($urandom % 4 != 0)
                issue(funcs[$urandom_range(0, funcs.size() - 1)],
                      opas[$urandom_range(0, opas.size() - 1)],
                      opbs[$urandom_range(0, opbs.size() - 1)]);
            out_grant = ($urandom % 4 != 0);
            step("random");
        end
        idle(); step("drain");

        $display("  (%0d results checked)", n_results);
        if (errors == 0) $display("@@@ Passed");
        else             $display("@@@ Failed (%0d errors)", errors);
        $finish;
    end

endmodule
