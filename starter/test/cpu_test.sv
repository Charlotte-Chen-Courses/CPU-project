/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  cpu_test.sv                                         //
//                                                                     //
//  Description :  End-to-end testbench for the first slice of the     //
//                 core (straight-line integer ALU code + WFI). Has    //
//                 its own small RV32I ISS, independent of the core's  //
//                 decoder and ALU. Each program starts by writing all //
//                 of x1..x31 (the PRF has no reset), runs a body, and //
//                 ends in WFI. Checks:                                //
//                  - every commit, in order: PC, whether it writes a  //
//                    register, rd, and the value in PRF[commit_tag]   //
//                    against the ISS;                                 //
//                  - nothing commits after the WFI; halted rises;     //
//                  - final architectural state: PRF[retire_map[r]]    //
//                    for x1..x31 against the ISS;                     //
//                  - a watchdog for deadlock.                         //
//                 Directed programs (dependent chain, independent     //
//                 ops, same-bundle RAW / WAW, x0 destinations,        //
//                 lui/auipc) and many random programs that use only   //
//                 x0..x7 so dependencies are dense. Reports IPC.      //
//                 Prints @@@ Passed/Failed.                           //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

// testbench-only: encoder helpers take int fields and truncate them on purpose
/* verilator lint_off UNUSEDSIGNAL */
/* verilator lint_off WIDTHEXPAND */

module testbench;

    localparam W     = `SUPERSCALAR_WIDTH;
    localparam TAG_W = `TAG_W;
    localparam MAXN  = 4096;
    localparam logic [31:0] WFI = 32'h10500073;

    logic                        clock, reset;
    logic [`XLEN-1:0]            imem_addr;
    INST  [W-1:0]                imem_data;
    logic [W-1:0]                commit_valid, commit_dest_valid;
    logic [W-1:0][4:0]           commit_rd;
    logic [W-1:0][TAG_W-1:0]     commit_tag;
    logic [W-1:0][`XLEN-1:0]     commit_pc;
    logic                        halted;

    cpu dut (
        .clock            (clock),
        .reset            (reset),
        .imem_addr        (imem_addr),
        .imem_data        (imem_data),
        .commit_valid     (commit_valid),
        .commit_dest_valid(commit_dest_valid),
        .commit_rd        (commit_rd),
        .commit_tag       (commit_tag),
        .commit_pc        (commit_pc),
        .halted           (halted)
    );

    always #5 clock = ~clock;

    // ---- instruction memory (past the program: WFI, never X) ----
    logic [31:0] imem [MAXN];
    int          prog_len;

    always_comb
        for (int i = 0; i < W; i++) begin
            automatic int a = int'(imem_addr >> 2) + i;
            imem_data[i] = (a < prog_len) ? imem[a] : WFI;
        end

    // ---- encoders ----
    function automatic logic [31:0] enc_r(input logic [6:0] f7, input logic [2:0] f3,
                                          input int rd, input int rs1, input int rs2);
        return {f7, 5'(rs2), 5'(rs1), f3, 5'(rd), 7'b0110011};
    endfunction
    function automatic logic [31:0] enc_i(input logic [2:0] f3, input int rd, input int rs1,
                                          input logic [11:0] imm);
        return {imm, 5'(rs1), f3, 5'(rd), 7'b0010011};
    endfunction
    function automatic logic [31:0] enc_sh(input logic [6:0] f7, input logic [2:0] f3,
                                           input int rd, input int rs1, input int sh);
        return {f7, 5'(sh), 5'(rs1), f3, 5'(rd), 7'b0010011};
    endfunction
    function automatic logic [31:0] enc_u(input logic [6:0] op, input int rd, input logic [19:0] imm);
        return {imm, 5'(rd), op};
    endfunction

    // random ALU instruction using registers 0..nreg-1 as sources and destinations
    function automatic logic [31:0] rand_alu(input int nreg);
        int rd  = $urandom_range(0, nreg - 1);
        int rs1 = $urandom_range(0, nreg - 1);
        int rs2 = $urandom_range(0, nreg - 1);
        case ($urandom % 18)
            0:  return enc_r(7'b0000000, 3'b000, rd, rs1, rs2);   // add
            1:  return enc_r(7'b0100000, 3'b000, rd, rs1, rs2);   // sub
            2:  return enc_r(7'b0000000, 3'b001, rd, rs1, rs2);   // sll
            3:  return enc_r(7'b0000000, 3'b010, rd, rs1, rs2);   // slt
            4:  return enc_r(7'b0000000, 3'b011, rd, rs1, rs2);   // sltu
            5:  return enc_r(7'b0000000, 3'b100, rd, rs1, rs2);   // xor
            6:  return enc_r(7'b0000000, 3'b101, rd, rs1, rs2);   // srl
            7:  return enc_r(7'b0100000, 3'b101, rd, rs1, rs2);   // sra
            8:  return enc_r(7'b0000000, 3'b110, rd, rs1, rs2);   // or
            9:  return enc_r(7'b0000000, 3'b111, rd, rs1, rs2);   // and
            10: return enc_i(3'b000, rd, rs1, 12'($urandom));      // addi
            11: return enc_i(3'b010, rd, rs1, 12'($urandom));      // slti
            12: return enc_i(3'b011, rd, rs1, 12'($urandom));      // sltiu
            13: return enc_i(3'b100, rd, rs1, 12'($urandom));      // xori
            14: return enc_i(($urandom % 2 == 1) ? 3'b110 : 3'b111, rd, rs1, 12'($urandom));  // ori/andi
            15: case ($urandom % 3)                                   // slli/srli/srai
                    0: return enc_sh(7'b0000000, 3'b001, rd, rs1, $urandom % 32);
                    1: return enc_sh(7'b0000000, 3'b101, rd, rs1, $urandom % 32);
                    default: return enc_sh(7'b0100000, 3'b101, rd, rs1, $urandom % 32);
                endcase
            16: return enc_u(7'b0110111, rd, 20'($urandom));         // lui
            default: return enc_u(7'b0010111, rd, 20'($urandom));    // auipc
        endcase
    endfunction

    // ---- ISS (RV32I ALU subset) ----
    logic [31:0] iss_reg [32];
    logic [31:0] iss_pc;

    // execute the instruction at iss_pc; returns 1 if it is WFI
    function automatic bit iss_step(output int rd, output bit writes, output logic [31:0] value);
        logic [31:0] i  = imem[iss_pc >> 2];
        logic [31:0] a  = iss_reg[i[19:15]];
        logic [31:0] b  = iss_reg[i[24:20]];
        logic [31:0] im = {{20{i[31]}}, i[31:20]};
        logic [4:0]  sh = i[24:20];
        rd = i[11:7];
        writes = 1;
        value  = 'x;
        if ((iss_pc >> 2) >= prog_len || i == WFI) begin
            writes = 0;
            return 1;
        end
        case (i[6:0])
            7'b0110111: value = {i[31:12], 12'b0};                     // lui
            7'b0010111: value = iss_pc + {i[31:12], 12'b0};            // auipc
            7'b0010011: case (i[14:12])
                3'b000: value = a + im;
                3'b010: value = ($signed(a) < $signed(im)) ? 1 : 0;
                3'b011: value = (a < im) ? 1 : 0;
                3'b100: value = a ^ im;
                3'b110: value = a | im;
                3'b111: value = a & im;
                3'b001: value = a << sh;
                3'b101: value = i[30] ? 32'($signed(a) >>> sh) : (a >> sh);
            endcase
            7'b0110011: case (i[14:12])
                3'b000: value = i[30] ? (a - b) : (a + b);
                3'b001: value = a << b[4:0];
                3'b010: value = ($signed(a) < $signed(b)) ? 1 : 0;
                3'b011: value = (a < b) ? 1 : 0;
                3'b100: value = a ^ b;
                3'b101: value = i[30] ? 32'($signed(a) >>> b[4:0]) : (a >> b[4:0]);
                3'b110: value = a | b;
                3'b111: value = a & b;
            endcase
            default: $fatal(1, "ISS: unexpected instruction %h", i);
        endcase
        if (rd == 0) writes = 0;
        else iss_reg[rd] = value;
        iss_pc += 4;
        return 0;
    endfunction

    // ---- running a program ----
    int errors = 0;
    longint total_cycles = 0, total_committed = 0;
    // cycles dispatch was blocked, by cause (a cycle can count under several)
    longint stall_rob = 0, stall_rs = 0, stall_freelist = 0;

    task automatic fail(input string msg);
        $display("  FAIL: %s", msg);
        errors++;
    endtask

    // prologue: give every register x1..x31 a defined, random value (lui + addi)
    task automatic start_program();
        prog_len = 0;
        for (int r = 1; r < 32; r++) begin
            imem[prog_len++] = enc_u(7'b0110111, r, 20'($urandom));
            imem[prog_len++] = enc_i(3'b000, r, r, 12'($urandom));
        end
    endtask

    task automatic run_program(input string name);
        int cycles = 0, committed = 0, rd;
        bit writes, done = 0, is_wfi;
        logic [31:0] value;
        int limit = 50 * prog_len + 2000;

        imem[prog_len++] = WFI;
        for (int r = 0; r < 32; r++) iss_reg[r] = 0;
        iss_pc = 0;

        reset = 1;
        @(negedge clock);
        @(negedge clock);
        reset = 0;

        while (!done) begin
            @(negedge clock);
            cycles++;
            if (!dut.fire && !dut.fetch_stop) begin
                if (!dut.rob_space_ok)              stall_rob++;
                if (!dut.rs_space_ok)               stall_rs++;
                if (!dut.dispatch_0.can_alloc_w)    stall_freelist++;
            end
            for (int i = 0; i < W; i++) begin
                if (!commit_valid[i]) continue;
                if (done) begin
                    fail($sformatf("[%s] slot %0d committed PC %h after the WFI", name, i, commit_pc[i]));
                    continue;
                end
                if (commit_pc[i] !== iss_pc)
                    fail($sformatf("[%s] commit %0d: PC %h, expected %h", name, committed, commit_pc[i], iss_pc));
                is_wfi = iss_step(rd, writes, value);
                committed++;
                if (is_wfi) begin done = 1; continue; end
                if (commit_dest_valid[i] !== writes)
                    fail($sformatf("[%s] PC %h: commit_dest_valid=%b, expected %b", name, commit_pc[i],
                                   commit_dest_valid[i], writes));
                else if (writes) begin
                    if (commit_rd[i] !== 5'(rd))
                        fail($sformatf("[%s] PC %h: rd=x%0d, expected x%0d", name, commit_pc[i], commit_rd[i], rd));
                    if (dut.prf_0.registers[commit_tag[i]] !== value)
                        fail($sformatf("[%s] PC %h (%h): x%0d = %h, expected %h", name, commit_pc[i],
                                       imem[commit_pc[i] >> 2], rd, dut.prf_0.registers[commit_tag[i]], value));
                end
                if (errors > 20) $fatal(1, "too many errors");
            end
            if (cycles > limit) begin
                fail($sformatf("[%s] watchdog: %0d of %0d instructions committed after %0d cycles",
                               name, committed, prog_len, cycles));
                return;
            end
        end

        // halted must rise right after the WFI commits; nothing else commits
        @(negedge clock);
        if (!halted) fail($sformatf("[%s] halted did not rise after WFI committed", name));
        if (commit_valid != 0) fail($sformatf("[%s] commit after halt", name));
        if (committed != prog_len) fail($sformatf("[%s] committed %0d, program has %0d", name, committed, prog_len));

        // final architectural state
        for (int r = 1; r < 32; r++)
            if (dut.prf_0.registers[dut.retire_map[r]] !== iss_reg[r])
                fail($sformatf("[%s] final x%0d = %h, expected %h", name, r,
                               dut.prf_0.registers[dut.retire_map[r]], iss_reg[r]));

        total_cycles    += cycles;
        total_committed += committed;
    endtask

    initial begin
        int n;
        clock = 0;
        reset = 1;
        prog_len = 0;
        $display("Config: W=%0d, ALU ports=%0d, CDBs=%0d, issue width=%0d, ROB=%0d, RS=%0d",
                 W, `NUM_FU_ALU, `NUM_CDB, `ISSUE_WIDTH, `ROB_SZ, `RS_SZ);

        $display("Test 1: dependent chain (addi x1, x1, 1) x 200");
        start_program();
        for (n = 0; n < 200; n++) imem[prog_len++] = enc_i(3'b000, 1, 1, 12'd1);
        run_program("chain");

        $display("Test 2: independent ops on x1..x31");
        start_program();
        for (n = 0; n < 200; n++) imem[prog_len++] = enc_i(3'b000, 1 + n % 31, 1 + n % 31, 12'($urandom));
        run_program("independent");

        $display("Test 3: same-bundle RAW (each instruction reads the previous one's rd)");
        start_program();
        for (n = 0; n < 200; n++) imem[prog_len++] = enc_r(7'b0000000, 3'b000, 2 + (n % 2), 2 + ((n + 1) % 2), 1);
        run_program("bundle RAW");

        $display("Test 4: same-bundle WAW (every instruction writes x5)");
        start_program();
        for (n = 0; n < 200; n++) imem[prog_len++] = enc_i(3'b000, 5, 5 + (n % 3), 12'($urandom));
        run_program("bundle WAW");

        $display("Test 5: x0 as destination and source");
        start_program();
        for (n = 0; n < 200; n++) imem[prog_len++] = (n % 2 == 1) ? enc_r(7'b0000000, 3'b000, 0, 1, 2)
                                                             : enc_r(7'b0000000, 3'b000, 3, 0, 3);
        run_program("x0");

        $display("Test 6: lui / auipc");
        start_program();
        for (n = 0; n < 200; n++) imem[prog_len++] = enc_u((n % 2 == 1) ? 7'b0110111 : 7'b0010111, 1 + n % 7, 20'($urandom));
        run_program("lui/auipc");

        $display("Test 7: 300 random programs (x0..x7, 300 instructions each)");
        for (int p = 0; p < 300; p++) begin
            start_program();
            for (n = 0; n < 300; n++) imem[prog_len++] = rand_alu(8);
            run_program($sformatf("random %0d", p));
            if (errors > 0) break;
        end

        $display("Test 8: 20 random programs filling the ROB (x0..x31, 2000 instructions each)");
        for (int p = 0; p < 20; p++) begin
            start_program();
            for (n = 0; n < 2000; n++) imem[prog_len++] = rand_alu(32);
            run_program($sformatf("long %0d", p));
            if (errors > 0) break;
        end

        $display("  (IPC over all programs: %0d instructions / %0d cycles = %.2f)",
                 total_committed, total_cycles, real'(total_committed) / real'(total_cycles));
        $display("  (dispatch blocked: ROB full %0d, RS full %0d, no free tag %0d cycles)",
                 stall_rob, stall_rs, stall_freelist);
        if (errors == 0) $display("@@@ Passed");
        else             $display("@@@ Failed (%0d errors)", errors);
        $finish;
    end

endmodule
