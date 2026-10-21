/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  cpu_test.sv                                         //
//                                                                     //
//  Description :  End-to-end testbench for the core (integer ALU      //
//                 code, branches, jal/jalr, WFI). Has its own small   //
//                 RV32I ISS, independent of the core's decoder, ALU   //
//                 and branch unit. Each program starts by writing all //
//                 of x1..x31 (the PRF has no reset), runs a body, and //
//                 ends in WFI. The ISS is run once first to get the   //
//                 dynamic instruction count. Checks:                  //
//                  - every commit, in order: PC (so control flow),    //
//                    whether it writes a register, rd, and the value  //
//                    in PRF[commit_tag], against the ISS;             //
//                  - exactly the dynamic count commits; nothing after //
//                    the WFI; halted rises;                           //
//                  - final state: PRF[retire_map[r]] for x1..x31;     //
//                  - tag conservation after the halt: free list full, //
//                    retirement-map tags + free list = every physical //
//                    register exactly once;                           //
//                  - a watchdog for deadlock.                         //
//                 Programs terminate by construction: branches and    //
//                 jumps go forward, loops are counted with x7, which  //
//                 their bodies never write. Directed recovery cases,  //
//                 then random programs. Reports IPC, branches,        //
//                 mispredicts and dispatch stalls.                    //
//                 +program=<file>: instead, run one program image     //
//                 (one hex byte per line from address 0, e.g. built   //
//                 from programs/*.s/*.c) with the same ISS checks;    //
//                 +trace=<file> also writes the commit trace.         //
//                 Prints @@@ Passed/Failed.                           //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

// testbench-only: encoder helpers take int fields and truncate them on purpose
/* verilator lint_off UNUSEDSIGNAL */
/* verilator lint_off WIDTHEXPAND */
/* verilator lint_off WIDTHTRUNC */

module testbench;

    localparam W     = `SUPERSCALAR_WIDTH;
    localparam TAG_W = `TAG_W;
    localparam MAXN  = `MEM_SIZE_IN_BYTES / 4;   // a loaded image can fill memory
    localparam logic [31:0] WFI = 32'h10500073;

    logic                        clock, reset;
    logic [1:0]                  proc2mem_command;
    logic [`XLEN-1:0]            proc2mem_addr;
    logic [63:0]                 proc2mem_data, mem2proc_data;
    logic [3:0]                  mem2proc_response, mem2proc_tag;
    logic [W-1:0]                commit_valid, commit_dest_valid;
    logic [W-1:0][4:0]           commit_rd;
    logic [W-1:0][TAG_W-1:0]     commit_tag;
    logic [W-1:0][`XLEN-1:0]     commit_pc;
    logic                        halted;
    EXCEPTION                    exception;
    logic [`XLEN-1:0]            exception_pc;
    logic                        stores_pending;

    cpu dut (
        .clock            (clock),
        .reset            (reset),
        .proc2mem_command (proc2mem_command),
        .proc2mem_addr    (proc2mem_addr),
        .proc2mem_data    (proc2mem_data),
        .mem2proc_response(mem2proc_response),
        .mem2proc_data    (mem2proc_data),
        .mem2proc_tag     (mem2proc_tag),
        .commit_valid     (commit_valid),
        .commit_dest_valid(commit_dest_valid),
        .commit_rd        (commit_rd),
        .commit_tag       (commit_tag),
        .commit_pc        (commit_pc),
        .halted           (halted),
        .exception        (exception),
        .exception_pc     (exception_pc),
        .stores_pending   (stores_pending)
    );

    always #5 clock = ~clock;

    // ---- P3 memory (the core fetches through its I-cache) ----
    mem memory (
        .clk              (clock),
        .proc2mem_addr    (proc2mem_addr),
        .proc2mem_data    (proc2mem_data),
        .proc2mem_command (proc2mem_command),
        .mem2proc_response(mem2proc_response),
        .mem2proc_data    (mem2proc_data),
        .mem2proc_tag     (mem2proc_tag)
    );

    // the program, as words (the ISS reads this; run_program copies it into mem.sv)
    logic [31:0] imem [MAXN];
    int          prog_len;

    // memory past the program reads as 0 (an illegal instruction)
    function automatic logic [31:0] prog_word(input int idx);
        return (idx >= 0 && idx < prog_len) ? imem[idx] : 32'h0;
    endfunction

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
    // conditional branch, byte offset off (multiple of 4)
    function automatic logic [31:0] enc_b(input logic [2:0] f3, input int rs1, input int rs2, input int off);
        logic [12:0] o = 13'(off);
        return {o[12], o[10:5], 5'(rs2), 5'(rs1), f3, o[4:1], o[11], 7'b1100011};
    endfunction
    function automatic logic [31:0] enc_jal(input int rd, input int off);
        logic [20:0] o = 21'(off);
        return {o[20], o[10:1], o[11], o[19:12], 5'(rd), 7'b1101111};
    endfunction
    function automatic logic [31:0] enc_jalr(input int rd, input int rs1, input int imm);
        return {12'(imm), 5'(rs1), 3'b000, 5'(rd), 7'b1100111};
    endfunction

    // random ALU instruction: sources from 0..nsrc-1, destination from dlo..dhi
    function automatic logic [31:0] enc_load(input logic [2:0] f3, input int rd, input int rs1, input logic [11:0] imm);
        return {imm, 5'(rs1), f3, 5'(rd), 7'b0000011};
    endfunction
    function automatic logic [31:0] enc_store(input logic [2:0] f3, input int rs2, input int rs1, input logic [11:0] imm);
        return {imm[11:5], 5'(rs2), 5'(rs1), f3, imm[4:0], 7'b0100011};
    endfunction

    localparam int DBASE = 'h8000;          // data region (x28 points here); code lives below
    localparam int DREG  = 28;              // base register: never written by generated code

    // a load or store into the 64-byte data window, naturally aligned
    function automatic logic [31:0] rand_mem(input int nsrc, input int dlo, input int dhi);
        int f      = $urandom_range(0, 7);
        int store  = $urandom_range(0, 1);
        int sz     = f % 3;                                   // 0 byte, 1 half, 2 word
        int off    = $urandom_range(0, 63) & ~((1 << sz) - 1);
        int rd     = $urandom_range(dlo, dhi);
        int rs2    = $urandom_range(0, nsrc - 1);
        logic [2:0] f3 = (!store && sz < 2 && f >= 3) ? 3'(4 + sz) : 3'(sz);   // lbu / lhu sometimes
        return store ? enc_store(3'(sz), rs2, DREG, 12'(off)) : enc_load(f3, rd, DREG, 12'(off));
    endfunction

    function automatic logic [31:0] rand_alu(input int nsrc, input int dlo = 0, input int dhi = 7);
        int rd  = $urandom_range(dlo, dhi);
        int rs1 = $urandom_range(0, nsrc - 1);
        int rs2 = $urandom_range(0, nsrc - 1);
        int op  = $urandom_range(0, 27);
        int sk  = $urandom_range(0, 2);
        int mf  = $urandom_range(0, 7);
        case (op)
            18, 19, 20: return enc_r(7'b0000001, 3'(mf % 4), rd, rs1, rs2);        // mul, mulh, mulhsu, mulhu
            21:         return enc_r(7'b0000001, 3'(4 + mf % 4), rd, rs1, rs2);    // div, divu, rem, remu
            22, 23, 24, 25, 26, 27: return rand_mem(nsrc, dlo, dhi);                // load / store
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
            15: case (sk)                                             // slli/srli/srai
                    0: return enc_sh(7'b0000000, 3'b001, rd, rs1, $urandom % 32);
                    1: return enc_sh(7'b0000000, 3'b101, rd, rs1, $urandom % 32);
                    default: return enc_sh(7'b0100000, 3'b101, rd, rs1, $urandom % 32);
                endcase
            16: return enc_u(7'b0110111, rd, 20'($urandom));         // lui
            default: return enc_u(7'b0010111, rd, 20'($urandom));    // auipc
        endcase
    endfunction

    function automatic logic [2:0] rand_cond();
        logic [2:0] c [] = '{3'b000, 3'b001, 3'b100, 3'b101, 3'b110, 3'b111};
        return c[$urandom_range(0, 5)];
    endfunction

    // ---- ISS (RV32I: ALU subset, branches, jal, jalr) ----
    logic [31:0] iss_reg [32];
    logic [31:0] iss_pc;
    logic [7:0]  init_mem [`MEM_SIZE_IN_BYTES];   // program image + data (what mem.sv starts with)
    logic [7:0]  iss_mem  [`MEM_SIZE_IN_BYTES];   // the ISS's memory
    longint      iss_ld = 0, iss_st = 0;
    longint      iss_branches;

    // execute the instruction at iss_pc. Returns 0 normally, 1 for WFI (or past the program),
    // 2 for an exception (exc = cause; nothing is written, the PC does not advance)
    localparam int ISS_OK = 0, ISS_WFI = 1, ISS_EXC = 2;
    function automatic int iss_step(output int rd, output bit writes, output logic [31:0] value,
                                    output EXCEPTION exc);
        logic [31:0] i  = (iss_pc < `MEM_SIZE_IN_BYTES) ? prog_word(int'(iss_pc >> 2)) : 32'h0;
        logic [31:0] a  = iss_reg[i[19:15]];
        logic [31:0] b  = iss_reg[i[24:20]];
        logic [31:0] im = {{20{i[31]}}, i[31:20]};
        logic [4:0]  sh = i[24:20];
        logic [31:0] npc = iss_pc + 4;
        bit t;
        rd = i[11:7];
        writes = 1;
        value  = 'x;
        exc    = EXC_NONE;
        if (iss_pc >= `MEM_SIZE_IN_BYTES) begin   // fetch outside memory
            writes = 0; exc = EXC_ACCESS; return ISS_EXC;
        end
        if (i == WFI) begin
            writes = 0;
            return ISS_WFI;
        end
        // legal instructions this core has no unit for yet, and a known-illegal word
        if (i[6:0] == 7'b1010011) begin
            writes = 0; exc = EXC_UNSUPPORTED; return ISS_EXC;
        end
        if (i[6:0] == 7'b1111111 || i == 32'h0) begin
            writes = 0; exc = EXC_ILLEGAL; return ISS_EXC;
        end
        if (i[6:0] == 7'b0000011 || i[6:0] == 7'b0100011) begin                // loads / stores
            automatic bit          st   = (i[6:0] == 7'b0100011);
            automatic logic [31:0] addr = st ? a + {{20{i[31]}}, i[31:25], i[11:7]} : a + im;
            automatic int          n    = 1 << i[13:12];
            automatic logic [31:0] raw  = '0;
            if (i[14:12] == 3'b011 || i[14:12] == 3'b110 || i[14:12] == 3'b111 || (st && i[14])) begin
                writes = 0; exc = EXC_ILLEGAL; return ISS_EXC;
            end
            if ((n == 2 && addr[0]) || (n == 4 && addr[1:0] != 0)) begin writes = 0; exc = EXC_MISALIGNED; return ISS_EXC; end
            if (addr >= `MEM_SIZE_IN_BYTES)                       begin writes = 0; exc = EXC_ACCESS;     return ISS_EXC; end
            if (st) begin
                for (int k = 0; k < n; k++) iss_mem[int'(addr) + k] = b[8 * k +: 8];
                writes = 0; iss_st++;
            end else begin
                for (int k = 0; k < n; k++) raw[8 * k +: 8] = iss_mem[int'(addr) + k];
                value = (n == 4) ? raw : i[14] ? raw & ((1 << (8 * n)) - 1)
                                               : (n == 1 ? {{24{raw[7]}}, raw[7:0]} : {{16{raw[15]}}, raw[15:0]});
                iss_ld++;
            end
            if (rd == 0) writes = 0;
            else if (writes) iss_reg[rd] = value;
            iss_pc = npc;
            return ISS_OK;
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
            7'b0110011: if (i[31:25] == 7'b0000001) begin                  // M extension
                automatic logic signed [63:0] ss = $signed({{32{a[31]}}, a}) * $signed({{32{b[31]}}, b});
                automatic logic signed [63:0] su = $signed({{32{a[31]}}, a}) * $signed({32'b0, b});
                automatic logic        [63:0] uu = {32'b0, a} * {32'b0, b};
                automatic bit ovf = (a == 32'h8000_0000 && b == 32'hffff_ffff);
                iss_m++;
                case (i[14:12])
                    3'b000: value = uu[31:0];                                         // mul
                    3'b001: value = ss[63:32];                                        // mulh
                    3'b010: value = su[63:32];                                        // mulhsu
                    3'b011: value = uu[63:32];                                        // mulhu
                    3'b100: value = (b == 0) ? '1 : ovf ? a : 32'($signed(a) / $signed(b)); // div
                    3'b101: value = (b == 0) ? '1 : a / b;                            // divu
                    3'b110: value = (b == 0) ? a  : ovf ? 0 : 32'($signed(a) % $signed(b)); // rem
                    3'b111: value = (b == 0) ? a  : a % b;                            // remu
                endcase
            end else case (i[14:12])
                3'b000: value = i[30] ? (a - b) : (a + b);
                3'b001: value = a << b[4:0];
                3'b010: value = ($signed(a) < $signed(b)) ? 1 : 0;
                3'b011: value = (a < b) ? 1 : 0;
                3'b100: value = a ^ b;
                3'b101: value = i[30] ? 32'($signed(a) >>> b[4:0]) : (a >> b[4:0]);
                3'b110: value = a | b;
                3'b111: value = a & b;
            endcase
            7'b1100011: begin                                            // branches
                case (i[14:12])
                    3'b000: t = (a == b);
                    3'b001: t = (a != b);
                    3'b100: t = ($signed(a) <  $signed(b));
                    3'b101: t = ($signed(a) >= $signed(b));
                    3'b110: t = (a <  b);
                    3'b111: t = (a >= b);
                    default: $fatal(1, "ISS: bad branch %h", i);
                endcase
                if (t) npc = iss_pc + {{19{i[31]}}, i[31], i[7], i[30:25], i[11:8], 1'b0};
                writes = 0;
                iss_branches++;
            end
            7'b1101111: begin                                            // jal
                value = iss_pc + 4;
                npc   = iss_pc + {{11{i[31]}}, i[31], i[19:12], i[20], i[30:21], 1'b0};
                iss_branches++;
            end
            7'b1100111: begin                                            // jalr
                value = iss_pc + 4;
                npc   = (a + im) & 32'hffff_fffe;
                iss_branches++;
            end
            default: $fatal(1, "ISS: unexpected instruction %h", i);
        endcase
        if (npc[1:0] != 2'b00) begin        // taken branch/jump to a misaligned target
            writes = 0; exc = EXC_MISALIGNED; return ISS_EXC;
        end
        if (rd == 0) writes = 0;
        else if (writes) iss_reg[rd] = value;
        iss_pc = npc;
        return ISS_OK;
    endfunction

    function automatic void iss_reset();
        for (int r = 0; r < 32; r++) iss_reg[r] = 0;
        iss_pc = 0;
        iss_mem = init_mem;
    endfunction

    // dynamic length of the program (the ISS run to the WFI or exception), and how it ends
    EXCEPTION    end_exc;
    logic [31:0] end_pc;
    function automatic int iss_count();
        int rd, n = 0;
        bit writes;
        logic [31:0] value;
        EXCEPTION e;
        iss_reset();
        while (iss_step(rd, writes, value, e) == ISS_OK) begin
            n++;
            if (n > 50000000) $fatal(1, "ISS: program does not terminate");
        end
        end_exc = e;
        end_pc  = iss_pc;
        return n + 1;   // + the WFI / excepting instruction
    endfunction

    // an instruction the core must never execute: only placed where control flow skips it, so
    // it appears only on wrong paths (load, store, FP op, illegal word, misaligned jalr/jal)
    function automatic logic [31:0] junk();
        int kind = $urandom_range(0, 5);
        case (kind)
            0: return {12'($urandom), 5'($urandom_range(0, 7)), 3'b010, 5'($urandom_range(1, 7)), 7'b0000011}; // lw
            1: return {7'($urandom), 5'($urandom_range(0, 7)), 5'($urandom_range(0, 7)), 3'b010, 5'($urandom), 7'b0100011}; // sw
            2: return {7'b0000000, 5'($urandom), 5'($urandom), 3'b111, 5'($urandom), 7'b1010011};         // fadd.s
            3: return {25'($urandom), 7'b1111111};                                              // illegal
            4: return enc_jalr(0, 0, 2);                                                         // jalr -> 2
            default: return enc_jal(0, 2);                                                       // jal -> +2
        endcase
    endfunction

    // ---- program construction ----
    task automatic emit(input logic [31:0] w);
        if (prog_len >= MAXN - 1) $fatal(1, "program too long");
        imem[prog_len++] = w;
    endtask

    // calls from main code to level-1 functions; the functions are placed after the WFI
    // (reached only by calls) and the call sites patched once their addresses are known
    int fx_site [$], fx_func [$];
    localparam NFUNC = 2;

    task automatic emit_call();
        fx_site.push_back(prog_len);
        fx_func.push_back($urandom_range(0, NFUNC - 1));
        emit(32'h0000_0013);              // placeholder (nop), patched to jal x1, F
    endtask

    // leaf functions L*: called with jal x5, return with jalr x0, x5, 0.
    // level-1 functions F*: called with jal x1, may call a leaf, return with jalr x0, x1, 0.
    // Bodies write only x2..x4, so links (x1, x5) and loop counters (x7) survive.
    task automatic emit_functions();
        int leaf [NFUNC], fn [NFUNC];
        if (fx_site.size() == 0) return;
        for (int f = 0; f < NFUNC; f++) begin
            automatic int len = $urandom_range(1, 3);
            leaf[f] = prog_len;
            for (int n = 0; n < len; n++) emit(rand_alu(8, 2, 4));
            emit(enc_jalr(0, 5, 0));
        end
        for (int f = 0; f < NFUNC; f++) begin
            automatic int len1 = $urandom_range(1, 3), len2 = $urandom_range(0, 2);
            automatic bit calls_leaf = ($urandom % 2 == 1);
            automatic int lf = $urandom_range(0, NFUNC - 1);
            fn[f] = prog_len;
            for (int n = 0; n < len1; n++) emit(rand_alu(8, 2, 4));
            if (calls_leaf) emit(enc_jal(5, 4 * (leaf[lf] - prog_len)));
            for (int n = 0; n < len2; n++) emit(rand_alu(8, 2, 4));
            emit(enc_jalr(0, 1, 0));
        end
        foreach (fx_site[k]) imem[fx_site[k]] = enc_jal(1, 4 * (fn[fx_func[k]] - fx_site[k]));
        fx_site.delete();
        fx_func.delete();
    endtask

    // prologue: give every register x1..x31 a defined, random value (lui + addi)
    task automatic start_program();
        fx_site.delete();
        fx_func.delete();
        prog_len = 0;
        for (int r = 1; r < 32; r++) begin
            emit(enc_u(7'b0110111, r, 20'($urandom)));
            emit(enc_i(3'b000, r, r, 12'($urandom)));
        end
        emit(enc_u(7'b0110111, DREG, 20'(DBASE >> 12)));   // x28 = data region
        emit(enc_i(3'b000, DREG, DREG, 12'(DBASE)));
    endtask

    // one random block that keeps control flow inside itself; dlo..dhi = destination regs
    task automatic emit_block(input int dlo, input int dhi, input bit allow_loop);
        // draw every random choice once, up front ($urandom inside a case expression or a
        // loop bound may be re-evaluated)
        int k    = $urandom_range(1, 4);
        int kind = $urandom_range(0, allow_loop ? 7 : 6);   // 6: call, 7: loop
        int len  = $urandom_range(1, 6);
        int nblk = $urandom_range(1, 3);
        case (kind)
            0, 1: for (int n = 0; n < len; n++) emit(rand_alu(8, dlo, dhi));
            2: begin   // if: conditional forward branch over k instructions
                emit(enc_b(rand_cond(), $urandom_range(0, 7), $urandom_range(0, 7), 4 * (k + 1)));
                for (int n = 0; n < k; n++) emit(rand_alu(8, dlo, dhi));
            end
            3: begin   // jal over k instructions (link into a random register); skipped: often junk
                emit(enc_jal($urandom_range(0, 1) ? $urandom_range(dlo, dhi) : 0, 4 * (k + 1)));
                for (int n = 0; n < k; n++) emit(($urandom % 2 == 1) ? junk() : rand_alu(8, dlo, dhi));
            end
            4: begin   // jalr over k instructions: auipc x6, 0 ; jalr rd, x6, 4*(k+2)
                if (dhi >= 6) begin
                    emit(enc_u(7'b0010111, 6, 20'd0));
                    emit(enc_jalr($urandom_range(0, 1) ? $urandom_range(dlo, dhi) : 0, 6, 4 * (k + 2)));
                    for (int n = 0; n < k; n++) emit(($urandom % 2 == 1) ? junk() : rand_alu(8, dlo, dhi));
                end
            end
            5: begin   // chain of taken jumps: back-to-back mispredicts
                for (int n = 0; n < k; n++) begin
                    emit(enc_jal(0, 8));
                    emit(($urandom % 2 == 1) ? junk() : rand_alu(8, dlo, dhi));     // skipped
                end
            end
            6: emit_call();  // jal x1 to a function placed after the WFI
            default: begin   // counted loop on x7; the body never writes x7
                int iters = $urandom_range(1, 8), start;
                emit(enc_i(3'b000, 7, 0, 12'(iters)));
                start = prog_len;
                for (int n = 0; n < nblk; n++) emit_block(1, 6, 0);
                emit(enc_i(3'b000, 7, 7, 12'hfff));                 // x7 -= 1
                emit(enc_b(3'b001, 7, 0, 4 * (start - prog_len)));  // bne x7, x0, start
            end
        endcase
    endtask

    // ---- running a program ----
    int errors = 0;
    longint total_cycles = 0, total_committed = 0, total_branches = 0, total_flushes = 0;
    longint stall_rob = 0, stall_rs = 0, stall_freelist = 0;
    int     n_exc_programs = 0;
    longint total_istall = 0;      // cycles fetch waited for the I-cache
    longint total_ld = 0, total_st = 0, total_fwd = 0;
    longint iss_m = 0, total_m = 0;   // M-extension instructions executed
    longint last_flushes, last_branches, last_cycles, last_committed, last_ret_misp;
    longint total_replays = 0, last_replays;   // memory-order violations (load replays)

    task automatic fail(input string msg);
        $display("  FAIL: %s", msg);
        errors++;
    endtask

    // after a halt nothing is in flight: every tag is either free or in the retirement map
    task automatic check_tags(input string name);
        int count [`PHYS_REG_SZ];
        int fl_sz = `FREE_LIST_SZ;
        foreach (count[t]) count[t] = 0;
        if (int'(dut.dispatch_0.freelist_0.free_count) != fl_sz)
            fail($sformatf("[%s] free list holds %0d tags after halt, expected %0d", name,
                           dut.dispatch_0.freelist_0.free_count, fl_sz));
        for (int r = 1; r < 32; r++) count[dut.retire_map[r]]++;
        for (int i = 0; i < fl_sz; i++) count[dut.dispatch_0.freelist_0.freelist_mem[i]]++;
        for (int t = 1; t < `PHYS_REG_SZ; t++)
            if (count[t] != 1) fail($sformatf("[%s] p%0d appears %0d times (retirement map + free list)",
                                              name, t, count[t]));
        if (count[0] != 0) fail($sformatf("[%s] p0 is in the retirement map or free list", name));
    endtask

    task automatic check_memory(input string name);
        automatic int nbad = 0;
        localparam int SETS = `DC_SZ / (`DC_BLK * `DC_WAYS);
        logic [7:0] actual [`MEM_SIZE_IN_BYTES];
        for (int k = 0; k < `MEM_64BIT_LINES; k++)
            for (int b = 0; b < 8; b++) actual[8 * k + b] = memory.unified_memory[k][8 * b +: 8];
        for (int st = 0; st < SETS; st++)
            for (int w = 0; w < `DC_WAYS; w++)
                if (dut.dcache_0.lines[st][w].valid && dut.dcache_0.lines[st][w].dirty) begin
                    automatic int base = (int'(dut.dcache_0.lines[st][w].tag) * SETS + st) * 8;
                    for (int b = 0; b < 8; b++) actual[base + b] = dut.dcache_0.lines[st][w].data[8 * b +: 8];
                end
        for (int a = 0; a < `MEM_SIZE_IN_BYTES; a++)
            if (actual[a] !== iss_mem[a]) begin
                if (nbad < 4) fail($sformatf("[%s] memory byte %h = %h, expected %h", name, a, actual[a], iss_mem[a]));
                nbad++;
            end
    endtask

    string prog_file;
    int    trace_fd = 0;    // +trace=<file>: one line per committed instruction (pc, rd, value)

    task automatic run_program(input string name, input bit from_file = 0);
        int cycles = 0, committed = 0, rd, flushes = 0, how, ret_misp = 0, replays = 0;
        bit writes, done = 0;
        EXCEPTION e;
        logic [31:0] value;
        int expect_n, limit;

        if (!from_file) begin                 // (a loaded image is already in init_mem and imem)
            emit(WFI);
            emit_functions();                 // after the WFI: reached only through calls
            // memory image: the program, zeros, and random data at 0x8000..0x9fff
            for (int a = 0; a < `MEM_SIZE_IN_BYTES; a += 4) begin
                automatic logic [31:0] w = prog_word(a / 4);
                for (int k = 0; k < 4; k++)
                    init_mem[a + k] = (a >= DBASE && a < DBASE + 'h2000) ? 8'($urandom) : w[8 * k +: 8];
            end
        end
        iss_ld = 0; iss_st = 0;
        iss_branches = 0;
        expect_n = iss_count();
        iss_m = 0;
        limit = 60 * expect_n + 2000;
        iss_reset();
        iss_branches = 0;

        iss_ld = 0; iss_st = 0;
        // load the image into mem.sv (64-bit lines, little-endian)
        for (int k = 0; k < `MEM_64BIT_LINES; k++)
            for (int b = 0; b < 8; b++) memory.unified_memory[k][8 * b +: 8] = init_mem[8 * k + b];

        reset = 1;
        @(negedge clock);
        @(negedge clock);
        reset = 0;

        while (!done) begin
            @(negedge clock);
            cycles++;
            if (!dut.fetch_stop && dut.pc_in_range && !dut.icache_valid) total_istall++;
            total_fwd += $countones(dut.lsq_0.can_fwd);
            if (dut.replay_flush) replays++;
            else if (dut.flush) begin
                flushes++;
                // the mispredicting instruction is the youngest one committing this cycle
                for (int i = W - 1; i >= 0; i--)
                    if (dut.commit_valid[i]) begin
                        if (dut.commit_is_ret[i]) ret_misp++;
                        break;
                    end
            end
            if (!dut.fire && !dut.fetch_stop && !dut.flush) begin
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
                how = iss_step(rd, writes, value, e);
                committed++;
                if (trace_fd != 0) $fdisplay(trace_fd, "%h %0d %h", commit_pc[i], writes ? rd : 0, writes ? value : 32'h0);
                if (dut.commit_exception[i] !== e)
                    fail($sformatf("[%s] PC %h (%h): commit_exception=%s, expected %s", name, commit_pc[i],
                                   imem[commit_pc[i] >> 2], dut.commit_exception[i].name(), e.name()));
                if (how != ISS_OK) begin done = 1; continue; end
                if (commit_dest_valid[i] !== writes)
                    fail($sformatf("[%s] PC %h (%h): commit_dest_valid=%b, expected %b", name, commit_pc[i],
                                   imem[commit_pc[i] >> 2], commit_dest_valid[i], writes));
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
                               name, committed, expect_n, cycles));
                return;
            end
        end

        // halted must rise right after the WFI / exception commits; nothing else commits
        @(negedge clock);
        if (!halted) fail($sformatf("[%s] halted did not rise after WFI committed", name));
        if (exception !== end_exc)
            fail($sformatf("[%s] stopped with %s, expected %s", name, exception.name(), end_exc.name()));
        else if (end_exc != EXC_NONE && exception_pc !== end_pc)
            fail($sformatf("[%s] exception_pc %h, expected %h", name, exception_pc, end_pc));
        if (commit_valid != 0) fail($sformatf("[%s] commit after halt", name));
        if (committed != expect_n)
            fail($sformatf("[%s] committed %0d, the ISS executes %0d", name, committed, expect_n));

        // final architectural state
        for (int r = 1; r < 32; r++)
            if (dut.prf_0.registers[dut.retire_map[r]] !== iss_reg[r])
                fail($sformatf("[%s] final x%0d = %h, expected %h", name, r,
                               dut.prf_0.registers[dut.retire_map[r]], iss_reg[r]));
        // committed stores may still be draining: wait, then memory = mem.sv + dirty D-cache blocks
        begin
            automatic int g = 0;
            while ((stores_pending || dut.dcache_0.state != 0) && g < 2000) begin @(negedge clock); g++; end
            if (g >= 2000) fail($sformatf("[%s] committed stores never drained", name));
            check_memory(name);
        end
        total_ld += iss_ld; total_st += iss_st;
        if (end_exc == EXC_NONE) check_tags(name);    // after an exception, younger work is still in flight
        else n_exc_programs++;

        total_cycles    += cycles;
        total_committed += committed;
        total_branches  += iss_branches;
        total_flushes   += flushes;
        total_m         += iss_m;
        total_replays   += replays;
        last_replays     = replays;
        last_flushes     = flushes;
        last_branches    = iss_branches;
        last_cycles      = cycles;
        last_committed   = committed;
        last_ret_misp    = ret_misp;
    endtask

    initial begin
        int n;
        clock = 0;
        reset = 1;
        prog_len = 0;
        $display("Config: W=%0d, ALU ports=%0d, branch ports=%0d, CDBs=%0d, issue width=%0d, ROB=%0d, RS=%0d, PRF=%0d",
                 W, `NUM_FU_ALU, `NUM_FU_BRANCH, `NUM_CDB, `ISSUE_WIDTH, `ROB_SZ, `RS_SZ, `PHYS_REG_SZ);

        // ---- one program from a file ----
        if ($value$plusargs("program=%s", prog_file)) begin
            foreach (init_mem[a]) init_mem[a] = 8'h0;
            $readmemh(prog_file, init_mem);
            for (int k = 0; k < MAXN; k++) imem[k] = {init_mem[4 * k + 3], init_mem[4 * k + 2], init_mem[4 * k + 1], init_mem[4 * k]};
            prog_len = MAXN;
            $display("Program %s", prog_file);
            if ($value$plusargs("trace=%s", prog_file)) trace_fd = $fopen(prog_file, "w");
            void'($value$plusargs("program=%s", prog_file));
            run_program(prog_file, 1);
            if (trace_fd != 0) $fclose(trace_fd);
            report();
        end

        // ---- straight-line (from the first slice) ----
        $display("Test 1: dependent chain (addi x1, x1, 1) x 200");
        start_program();
        for (n = 0; n < 200; n++) emit(enc_i(3'b000, 1, 1, 12'd1));
        run_program("chain");

        $display("Test 2: same-bundle RAW and WAW");
        start_program();
        for (n = 0; n < 100; n++) emit(enc_r(7'b0000000, 3'b000, 2 + (n % 2), 2 + ((n + 1) % 2), 1));
        for (n = 0; n < 100; n++) emit(enc_i(3'b000, 5, 5 + (n % 3), 12'($urandom)));
        run_program("bundle RAW/WAW");

        $display("Test 3: x0 as destination and source, lui / auipc");
        start_program();
        for (n = 0; n < 100; n++) emit((n % 2 == 1) ? enc_r(7'b0000000, 3'b000, 0, 1, 2) : enc_r(7'b0000000, 3'b000, 3, 0, 3));
        for (n = 0; n < 100; n++) emit(enc_u((n % 2 == 1) ? 7'b0110111 : 7'b0010111, 1 + n % 7, 20'($urandom)));
        run_program("x0, lui/auipc");

        // ---- recovery (directed) ----
        $display("Test 4: jal x1 over one instruction, then read x1 (RAT restored with the link)");
        start_program();
        emit(enc_jal(1, 8));
        emit(enc_i(3'b000, 9, 0, 12'd99));          // skipped
        emit(enc_r(7'b0000000, 3'b000, 2, 1, 1));   // add x2, x1, x1
        run_program("jal link");

        $display("Test 5: older write commits with the mispredicting branch, read after the target");
        start_program();                              // 62 instructions: next one is in slot 0
        emit(enc_i(3'b000, 3, 3, 12'd5));            // slot 0: addi x3, x3, 5
        emit(enc_b(3'b000, 0, 0, 8));                // slot 1: beq x0, x0, +8 (taken)
        emit(enc_i(3'b000, 3, 0, 12'd77));           // skipped
        emit(enc_r(7'b0000000, 3'b000, 4, 3, 3));    // add x4, x3, x3
        emit(enc_i(3'b000, 0, 0, 12'd0));            // nop: next branch in slot 0
        emit(enc_b(3'b000, 0, 0, 8));                // slot 0: taken, slot 1 squashed
        emit(enc_i(3'b000, 3, 0, 12'd88));           // skipped
        emit(enc_r(7'b0000000, 3'b000, 5, 4, 3));
        run_program("same-cycle commit + mispredict");

        $display("Test 6: back-to-back mispredicts (jal chain), and a loop");
        start_program();
        for (n = 0; n < 20; n++) begin emit(enc_jal(0, 8)); emit(enc_i(3'b000, 1, 0, 12'd1)); end
        emit(enc_i(3'b000, 7, 0, 12'd30));
        emit(enc_i(3'b000, 1, 1, 12'd3));
        emit(enc_i(3'b000, 7, 7, 12'hfff));
        emit(enc_b(3'b001, 7, 0, -8));               // bne x7, x0, -8
        run_program("jal chain + loop");

        $display("Test 7: mispredict while the RS/ROB are full (long dependent chain, then a taken branch)");
        start_program();
        for (n = 0; n < 60; n++) emit(enc_i(3'b000, 1, 1, 12'd1));
        emit(enc_b(3'b000, 0, 0, 12));
        for (n = 0; n < 2; n++) emit(enc_i(3'b000, 2, 0, 12'd5));   // skipped
        emit(enc_r(7'b0000000, 3'b000, 3, 1, 1));
        run_program("full + mispredict");

        // ---- exceptions: only raised if the instruction commits ----
        $display("Test 8: unsupported / illegal / misaligned instructions on the wrong path only");
        start_program();                               // 62 instructions: next is slot 0
        emit(enc_b(3'b000, 0, 0, 8));                 // slot 0: taken
        emit(32'h00002083);                           // slot 1: lw (wrong path)
        emit(enc_jal(0, 28));                         // jump over six junk words
        emit({7'd0, 5'd2, 5'd0, 3'b010, 5'd4, 7'b0100011});     // sw
        emit({7'b0000000, 5'd2, 5'd1, 3'b111, 5'd3, 7'b1010011});  // fadd.s
        emit(32'hffff_ffff);                          // illegal
        emit(enc_jalr(0, 0, 2));                      // jalr -> 2 (misaligned)
        emit(enc_jal(0, 2));                          // jal -> +2 (misaligned)
        emit(32'h00002083);                           // lw
        emit(enc_i(3'b000, 2, 0, 12'd5));
        emit(enc_b(3'b001, 0, 0, 6));                 // bne x0, x0, +6: misaligned but NOT taken: fine
        emit(enc_i(3'b000, 3, 2, 12'd1));
        run_program("wrong-path junk");

        $display("Test 9: exceptions on the correct path stop the core at the right PC");
        start_program(); emit(enc_i(3'b000, 2, 0, 12'd5)); emit(enc_load(3'b010, 1, DREG, 12'd1)); emit(enc_i(3'b000, 3, 0, 12'd1));
        run_program("correct-path misaligned lw");
        start_program(); emit(enc_u(7'b0110111, 9, 20'h10)); emit(enc_store(3'b000, 5, 9, 12'd0));
        run_program("correct-path sb outside memory");
        start_program(); emit(32'hffff_ffff); emit(enc_i(3'b000, 3, 0, 12'd1));
        run_program("correct-path illegal");
        start_program(); emit({7'b0000000, 5'd2, 5'd1, 3'b111, 5'd3, 7'b1010011});
        run_program("correct-path fadd.s");
        start_program(); emit(enc_jal(1, 2)); emit(enc_i(3'b000, 3, 0, 12'd1));
        run_program("correct-path misaligned jal (x1 must not be written)");
        start_program(); emit(enc_b(3'b000, 0, 0, 6)); emit(enc_i(3'b000, 3, 0, 12'd1));
        run_program("correct-path misaligned taken beq");
        start_program(); emit(enc_i(3'b000, 6, 0, 12'd6)); emit(enc_jalr(1, 6, 0));
        run_program("correct-path misaligned jalr");

        // 0x12000: outside the 64 KB memory, and its low 16 bits alias to a block that is not cached,
        // so sending it to the I-cache would wait forever for mem.sv
        start_program(); emit(enc_u(7'b0110111, 6, 20'h12)); emit(enc_jalr(0, 6, 0));   // jump to 0x12000
        run_program("correct-path jump outside memory (EXC_ACCESS at the target)");

        $display("Test 10: exception in slot 0 with a mispredicting branch in slot 1 -> exception, no flush");
        start_program();
        emit(32'hffff_ffff);                          // slot 0: illegal
        emit(enc_b(3'b000, 0, 0, 8));                 // slot 1: taken
        emit(enc_i(3'b000, 3, 0, 12'd1));
        emit(enc_i(3'b000, 4, 0, 12'd1));
        run_program("exception + mispredict");

        // ---- M extension ----
        $display("Test 11: multiply / divide on edge operands, including x/0 and INT_MIN/-1");
        start_program();
        begin
            automatic logic [31:0] ev [6] = '{32'h0, 32'h1, 32'hffff_ffff, 32'h8000_0000, 32'h7fff_ffff, 32'hdead_beef};
            for (int i = 0; i < 6; i++) begin
                emit(enc_u(7'b0110111, 1, ev[i][31:12] + ev[i][11]));   // x1 = ev[i] (lui + addi)
                emit(enc_i(3'b000, 1, 1, ev[i][11:0]));
                for (int j = 0; j < 6; j++) begin
                    emit(enc_u(7'b0110111, 2, ev[j][31:12] + ev[j][11]));
                    emit(enc_i(3'b000, 2, 2, ev[j][11:0]));
                    for (int f = 0; f < 8; f++) emit(enc_r(7'b0000001, 3'(f), 3 + f, 1, 2));
                end
            end
        end
        run_program("M edge operands");

        $display("Test 12: dependent chain through the multiplier and the divider");
        start_program();
        emit(enc_i(3'b000, 1, 0, 12'd3));
        for (int n = 0; n < 40; n++) begin
            emit(enc_r(7'b0000001, 3'b000, 1, 1, 1));       // x1 = x1 * x1
            emit(enc_i(3'b000, 1, 1, 12'd7));               // x1 += 7
            if (n % 8 == 0) emit(enc_r(7'b0000001, 3'b101, 2, 1, 3));   // x2 = x1 / x3 (divu)
        end
        run_program("M chain");
        $display("  (M chain: IPC %.2f)", real'(last_committed) / real'(last_cycles));

        $display("Test 13: independent multiplies back to back, with ALU work competing for the CDBs");
        start_program();
        for (int n = 0; n < 300; n++)
            emit((n % 3 == 2) ? enc_i(3'b000, 8 + n % 5, 8 + n % 5, 12'd1)
                              : enc_r(7'b0000001, 3'(n % 4), 3 + n % 5, 1 + n % 7, 2 + n % 5));
        run_program("M throughput");
        $display("  (M throughput: IPC %.2f)", real'(last_committed) / real'(last_cycles));

        $display("Test 14: a divide squashed by a mispredict, and a divide on the wrong path");
        start_program();
        emit(enc_r(7'b0000001, 3'b100, 3, 1, 2));           // long divide in flight...
        emit(enc_b(3'b000, 0, 0, 12));                      // ...while a taken branch mispredicts
        emit(enc_r(7'b0000001, 3'b110, 4, 1, 2));           // wrong path: rem
        emit(enc_r(7'b0000001, 3'b000, 5, 1, 2));           // wrong path: mul
        emit(enc_r(7'b0000001, 3'b101, 6, 3, 1));           // correct path: divu, reads the first divide
        emit(enc_r(7'b0000001, 3'b001, 7, 6, 3));
        run_program("M + flush");

        // ---- RAS ----
        $display("Test 15: one function called from two sites in a hot loop (returns alternate)");
        start_program();
        emit(enc_i(3'b000, 7, 0, 12'd100));
        begin
            automatic int top = prog_len, site_a, site_b, fn;
            site_a = prog_len; emit(32'h13);           // jal x1, F (patched)
            emit(enc_i(3'b000, 6, 6, 12'd1));
            site_b = prog_len; emit(32'h13);           // jal x1, F (patched)
            emit(enc_i(3'b000, 7, 7, 12'hfff));
            emit(enc_b(3'b001, 7, 0, 4 * (top - prog_len)));
            emit(WFI);
            fn = prog_len;                             // F: two ALU ops, return
            emit(enc_i(3'b000, 2, 2, 12'd1));
            emit(enc_i(3'b000, 3, 2, 12'd2));
            emit(enc_jalr(0, 1, 0));
            imem[site_a] = enc_jal(1, 4 * (fn - site_a));
            imem[site_b] = enc_jal(1, 4 * (fn - site_b));
        end
        run_program("two call sites");
        $display("  (two call sites: %0d branches/jumps, %0d mispredicts, IPC %.2f)",
                 last_branches, last_flushes, real'(last_committed) / real'(last_cycles));
        if (`BTB_SZ >= 16 && `RAS_SZ >= 2 && last_flushes > 10)
            fail($sformatf("two call sites: %0d mispredicts; the RAS should predict every return", last_flushes));

        $display("Test 16: calls nested 12 deep (deeper than the %0d-entry RAS), 20 times", `RAS_SZ);
        start_program();
        emit(enc_i(3'b000, 7, 0, 12'd20));
        begin
            automatic int top = prog_len, site, f [12];
            site = prog_len; emit(32'h13);             // jal x1, f0 (patched)
            emit(enc_i(3'b000, 7, 7, 12'hfff));
            emit(enc_b(3'b001, 7, 0, 4 * (top - prog_len)));
            emit(WFI);
            for (int d = 0; d < 12; d++) begin         // f_d: save link in x(10+d), call f_d+1, restore, return
                f[d] = prog_len;
                if (d < 11) begin
                    emit(enc_r(7'b0000000, 3'b000, 10 + d, 1, 0));   // add x(10+d), x1, x0
                    emit(32'h13);                                    // jal x1, f_d+1 (patched below)
                    emit(enc_r(7'b0000000, 3'b000, 1, 10 + d, 0));   // add x1, x(10+d), x0
                end
                emit(enc_jalr(0, 1, 0));
            end
            imem[site] = enc_jal(1, 4 * (f[0] - site));
            for (int d = 0; d < 11; d++) imem[f[d] + 1] = enc_jal(1, 4 * (f[d + 1] - (f[d] + 1)));
        end
        run_program("deep calls");
        $display("  (deep calls: %0d branches/jumps, %0d mispredicts, IPC %.2f)",
                 last_branches, last_flushes, real'(last_committed) / real'(last_cycles));

        $display("Test 17: RAS repair: mispredicts and fetch stalls inside a function, 100 calls");
        start_program();
        emit(enc_i(3'b000, 7, 0, 12'd100));
        emit(enc_i(3'b000, 8, 0, 12'd0));
        begin
            automatic int top = prog_len, site, ff, gg, g2, call_g, br, call_g2, lskip;
            site = prog_len; emit(32'h13);                 // main: jal x1, F (patched)
            emit(enc_i(3'b000, 7, 7, 12'hfff));
            emit(enc_b(3'b001, 7, 0, 4 * (top - prog_len)));
            emit(WFI);
            // F: save link; long dependent chain (RS fills, fetch stalls, often on the next bundle);
            //    call G; alternating branch whose wrong path either calls the long G2 (flush lands
            //    inside G2: an extra push) or skips to F's return (an extra pop); restore; return
            ff = prog_len;
            emit(enc_r(7'b0000000, 3'b000, 10, 1, 0));             // add x10, x1, x0
            for (int n = 0; n < 40; n++) emit(enc_i(3'b000, 9, 9, 12'd1));
            call_g = prog_len; emit(32'h13);                      // jal x5, G
            emit(enc_i(3'b100, 8, 8, 12'd1));                     // x8 ^= 1
            br = prog_len; emit(32'h13);                          // beq x8, x0, Lskip
            call_g2 = prog_len; emit(32'h13);                     // jal x5, G2
            lskip = prog_len;
            emit(enc_r(7'b0000000, 3'b000, 1, 10, 0));            // add x1, x10, x0
            emit(enc_jalr(0, 1, 0));                               // return from F
            gg = prog_len; emit(enc_i(3'b000, 2, 2, 12'd1)); emit(enc_jalr(0, 5, 0));
            g2 = prog_len; for (int n = 0; n < 30; n++) emit(enc_i(3'b000, 3, 3, 12'd1)); emit(enc_jalr(0, 5, 0));
            imem[site]    = enc_jal(1, 4 * (ff - site));
            imem[call_g]  = enc_jal(5, 4 * (gg - call_g));
            imem[br]      = enc_b(3'b000, 8, 0, 4 * (lskip - br));
            imem[call_g2] = enc_jal(5, 4 * (g2 - call_g2));
        end
        run_program("RAS repair");
        $display("  (RAS repair: %0d branches/jumps, %0d mispredicts, %0d of them returns, IPC %.2f)",
                 last_branches, last_flushes, last_ret_misp, real'(last_committed) / real'(last_cycles));
        if (`BTB_SZ >= 16 && `RAS_SZ >= 4 && last_ret_misp > 6)
            fail($sformatf("RAS repair: %0d return mispredicts; the RAS should be restored on every flush and only pushed by dispatched bundles", last_ret_misp));

        // ---- predictor: a hot loop (each static branch runs many times) ----
        $display("Test 18: hot loop, 200 iterations: back-edge, a never-taken if, a jal, and a jalr");
        start_program();
        emit(enc_i(3'b000, 7, 0, 12'd200));           // x7 = 200
        begin
            automatic int top = prog_len;
            emit(enc_i(3'b000, 1, 1, 12'd3));
            emit(enc_b(3'b001, 0, 0, 8));            // bne x0, x0: never taken
            emit(enc_i(3'b000, 2, 2, 12'd1));
            emit(enc_jal(0, 8));                     // always taken
            emit(32'hffff_ffff);                     // skipped (illegal)
            emit(enc_u(7'b0010111, 6, 20'd0));        // auipc x6, 0
            emit(enc_jalr(0, 6, 12));                // jalr over one instruction
            emit(32'h00002083);                       // skipped (lw)
            emit(enc_i(3'b000, 7, 7, 12'hfff));       // x7 -= 1
            emit(enc_b(3'b001, 7, 0, 4 * (top - prog_len)));   // bne x7, x0, top
        end
        run_program("hot loop");
        $display("  (hot loop: %0d branches/jumps, %0d mispredicts, IPC %.2f)",
                 last_branches, last_flushes, real'(last_committed) / real'(last_cycles));
        // (only for realistic table sizes: a tiny BTB/BHT aliases the loop's branches by design)
        if (`BTB_SZ >= 16 && `BHT_SZ >= 16 && last_flushes > 10)
            fail($sformatf("hot loop: %0d mispredicts; a trained BTB + bimodal should miss only while warming up", last_flushes));

        // ---- random ----
        // ---- loads and stores ----
        $display("Test 21: store then load, every size: forwarding and sign/zero extension");
        start_program();
        emit(enc_u(7'b0110111, 5, 20'h8badf)); emit(enc_i(3'b000, 5, 5, 12'h0c9));   // x5 = 0x8badf0c9
        emit(enc_store(3'b010, 5, DREG, 12'd8));
        for (int f = 0; f < 6; f++) if (f != 3) emit(enc_load(3'(f), 10 + f, DREG, 12'(8 + (f % 3 == 1 ? 2 : f % 3 == 0 ? 3 : 0))));
        emit(enc_store(3'b001, 5, DREG, 12'd16)); emit(enc_load(3'b001, 20, DREG, 12'd16)); emit(enc_load(3'b101, 21, DREG, 12'd16));
        emit(enc_store(3'b000, 5, DREG, 12'd21)); emit(enc_load(3'b000, 22, DREG, 12'd21)); emit(enc_load(3'b100, 23, DREG, 12'd21));
        run_program("forwarding");

        $display("Test 22: partial overlap (byte store, then word load of the same word)");
        start_program();
        emit(enc_store(3'b000, 5, DREG, 12'd33)); emit(enc_load(3'b010, 6, DREG, 12'd32));
        emit(enc_store(3'b001, 7, DREG, 12'd38)); emit(enc_load(3'b010, 8, DREG, 12'd36));
        run_program("partial overlap");

        $display("Test 23: store address from a slow divide; a later same-address load is replayed (or waits)");
        start_program();
        emit(enc_i(3'b000, 11, 0, 12'd1));
        emit(enc_r(7'b0000001, 3'b101, 9, DREG, 11));             // x9 = x28 / 1 (34 cycles)
        emit(enc_store(3'b010, 5, 9, 12'd4));                      // address unknown for a while
        emit(enc_load(3'b010, 6, DREG, 12'd4));                    // same address
        emit(enc_load(3'b010, 7, DREG, 12'd12));                   // different address
        run_program("unknown store address");
        $display("  (%0d load replays)", last_replays);
        if (last_replays != (`LSQ_SPECULATE ? 1 : 0))
            fail($sformatf("unknown store address: %0d replays, expected %0d (only the same-address load violates)",
                           last_replays, `LSQ_SPECULATE ? 1 : 0));

        $display("Test 23b: the same conflict in a 20-iteration loop: replayed every time, or (LSQ_SPECULATE = 2) once");
        start_program();
        emit(enc_i(3'b000, 11, 0, 12'd1));
        emit(enc_i(3'b000, 7, 0, 12'd20));                         // x7 = iterations
        emit(enc_r(7'b0000001, 3'b101, 9, DREG, 11));             // loop: x9 = x28 / 1 (34 cycles)
        emit(enc_store(3'b010, 5, 9, 12'd4));                      // store address waits for the divide
        emit(enc_load(3'b010, 6, DREG, 12'd4));                    // same address, known at once
        emit(enc_i(3'b000, 5, 5, 12'd1));                          // a new value every iteration
        emit(enc_i(3'b000, 7, 7, -12'd1));
        emit(enc_b(3'b001, 7, 0, -20));                            // bne x7, x0, loop
        run_program("loop conflict");
        $display("  (%0d load replays in 20 iterations)", last_replays);
        if (`LSQ_SPECULATE == 0 && last_replays != 0)  fail($sformatf("loop conflict: %0d replays without speculation", last_replays));
        if (`LSQ_SPECULATE == 1 && last_replays < 15)  fail($sformatf("loop conflict: only %0d replays: the loads should run ahead", last_replays));
        if (`LSQ_SPECULATE == 2 && last_replays > 2)   fail($sformatf("loop conflict: %0d replays: the load-wait table should stop them", last_replays));

        $display("Test 24: 24 stores (more than the store queue), then read them all back");
        start_program();
        for (int n = 0; n < 24; n++) emit(enc_store(3'b010, 1 + n % 7, DREG, 12'(4 * (n % 16))));
        for (int n = 0; n < 16; n++) emit(enc_load(3'b010, 1 + n % 7, DREG, 12'(4 * n)));
        run_program("store queue full");

        $display("Test 25: a store on a mispredicted path must never reach memory");
        start_program();
        emit(enc_b(3'b000, 0, 0, 12));                             // taken
        emit(enc_store(3'b010, 5, DREG, 12'd0));                   // wrong path
        emit(enc_store(3'b000, 5, 0, 12'd0));                      // wrong path, into code
        emit(enc_load(3'b010, 6, DREG, 12'd0));
        run_program("wrong-path stores");

        $display("Test 26: D-cache evictions: 16 blocks that all map to one set, written then read");
        start_program();
        for (int n = 0; n < 16; n++) begin
            emit(enc_u(7'b0110111, 9, 20'((DBASE + 128 * n) >> 12))); emit(enc_i(3'b000, 9, 9, 12'(DBASE + 128 * n)));
            emit(enc_store(3'b010, 1 + n % 7, 9, 12'd0));
        end
        for (int n = 0; n < 16; n++) begin
            emit(enc_u(7'b0110111, 9, 20'((DBASE + 128 * n) >> 12))); emit(enc_i(3'b000, 9, 9, 12'(DBASE + 128 * n)));
            emit(enc_load(3'b010, 10, 9, 12'd0));
        end
        run_program("evictions");

        $display("Test 27: 300 random programs (branches, jumps, loops, calls, wrong-path junk; x0..x7)");
        for (int p = 0; p < 300; p++) begin
            start_program();
            while (prog_len < 62 + 250) emit_block(0, 7, 1);
            run_program($sformatf("random %0d", p));
            if (errors > 0) break;
        end

        $display("Test 28: 20 long random programs (x0..x31 straight-line + branch blocks)");
        for (int p = 0; p < 20; p++) begin
            start_program();
            while (prog_len < 62 + 2000) begin
                if ($urandom % 4 == 0) emit_block(0, 7, 1);
                else emit(rand_alu(32, 0, 31));
            end
            run_program($sformatf("long %0d", p));
            if (errors > 0) break;
        end

        report();
    end

    task automatic report();
        $display("  (IPC: %0d instructions / %0d cycles = %.2f)",
                 total_committed, total_cycles, real'(total_committed) / real'(total_cycles));
        $display("  (branches/jumps: %0d, mispredicts: %0d = %.1f%%; random code runs most branches once)",
                 total_branches, total_flushes, 100.0 * real'(total_flushes) / real'(total_branches));
        $display("  (dispatch blocked: ROB full %0d, RS full %0d, no free tag %0d cycles)",
                 stall_rob, stall_rs, stall_freelist);
        $display("  (%0d programs ended in an exception, as expected)", n_exc_programs);
        $display("  (multiply/divide instructions executed: %0d)", total_m);
        $display("  (I-cache: fetch waited %0d of %0d cycles)", total_istall, total_cycles);
        $display("  (loads %0d, stores %0d committed; %0d loads forwarded from the store queue; D-cache %0d hits, %0d misses, %0d writebacks)",
                 total_ld, total_st, total_fwd, dut.dcache_0.n_hits, dut.dcache_0.n_misses, dut.dcache_0.n_writebacks);
        $display("  (LSQ_SPECULATE = %0d: %0d load replays)", `LSQ_SPECULATE, total_replays);
        if (errors == 0) $display("@@@ Passed");
        else             $display("@@@ Failed (%0d errors)", errors);
        $finish;
    endtask

endmodule
