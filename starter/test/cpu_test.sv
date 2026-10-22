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
        int op  = $urandom_range(0, (`NUM_FU_FP > 0) ? 35 : 27);
        int sk  = $urandom_range(0, 2);
        int mf  = $urandom_range(0, 7);
        case (op)
            18, 19, 20: return enc_r(7'b0000001, 3'(mf % 4), rd, rs1, rs2);        // mul, mulh, mulhsu, mulhu
            21:         return enc_r(7'b0000001, 3'(4 + mf % 4), rd, rs1, rs2);    // div, divu, rem, remu
            22, 23, 24, 25, 26, 27: return rand_mem(nsrc, dlo, dhi);                // load / store
            28, 29, 30, 31, 32, 33, 34, 35: return rand_fp(nsrc, dlo, dhi);         // F
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

    // ---- F extension: reference arithmetic and random FP instructions ----
    import "DPI-C" function int unsigned fp_ref(input int unsigned inst, input int unsigned a, input int unsigned b,
                                                input int unsigned c, input int unsigned frm, output int unsigned flags);

    function automatic logic [31:0] opfp(input logic [4:0] f5, input int rs2, input int rs1, input logic [2:0] f3, input int rd);
        return {f5, 2'b00, 5'(rs2), 5'(rs1), f3, 5'(rd), 7'b1010011};
    endfunction

    function automatic logic [31:0] enc_csr(input logic [2:0] f3, input int rd, input int rs1_or_uimm, input logic [11:0] addr);
        return {addr, 5'(rs1_or_uimm), f3, 5'(rd), 7'b1110011};
    endfunction

    // a random legal F instruction: f sources/destinations in f0..f7, x sources in x0..x(nsrc-1),
    // x destinations in dlo..dhi (so a caller's loop counter / base register is never written)
    function automatic logic [31:0] rand_fp(input int nsrc, input int dlo, input int dhi);
        logic [2:0] rms [] = '{3'd0, 3'd1, 3'd2, 3'd3, 3'd4, 3'd7};
        int k  = $urandom_range(0, 19);
        int fd = $urandom_range(0, 7), f1 = $urandom_range(0, 7), f2 = $urandom_range(0, 7), f3 = $urandom_range(0, 7);
        int xd = $urandom_range(dlo, dhi), x1 = $urandom_range(0, nsrc - 1);
        int off = 4 * $urandom_range(0, 15);
        logic [2:0] rm = rms[$urandom_range(0, 5)];
        case (k)
            0, 1: return opfp(5'b00000, f2, f1, rm, fd);                          // fadd
            2:    return opfp(5'b00001, f2, f1, rm, fd);                          // fsub
            3:    return opfp(5'b00010, f2, f1, rm, fd);                          // fmul
            4:    return opfp(5'b00011, f2, f1, rm, fd);                          // fdiv
            5:    return opfp(5'b01011, 0, f1, rm, fd);                           // fsqrt
            6:    return opfp(5'b00100, f2, f1, 3'($urandom_range(0, 2)), fd);    // fsgnj / n / x
            7:    return opfp(5'b00101, f2, f1, 3'($urandom_range(0, 1)), fd);    // fmin / fmax
            8:    return opfp(5'b10100, f2, f1, 3'($urandom_range(0, 2)), xd);    // fle / flt / feq
            9:    return opfp(5'b11000, $urandom_range(0, 1), f1, rm, xd);        // fcvt.w[u].s
            10:   return opfp(5'b11010, $urandom_range(0, 1), x1, rm, fd);        // fcvt.s.w[u]
            11:   return opfp(5'b11100, 0, f1, 3'($urandom_range(0, 1)), xd);     // fmv.x.w / fclass
            12:   return opfp(5'b11110, 0, x1, 3'b000, fd);                       // fmv.w.x
            13, 14: begin                                                         // fmadd / fmsub / fnmsub / fnmadd
                automatic logic [6:0] ops [] = '{7'b1000011, 7'b1000111, 7'b1001011, 7'b1001111};
                return {5'(f3), 2'b00, 5'(f2), 5'(f1), rm, 5'(fd), ops[$urandom_range(0, 3)]};
            end
            15, 16: return {12'(off), 5'(DREG), 3'b010, 5'(fd), 7'b0000111};     // flw
            17:     return {7'(off >> 5), 5'(f2), 5'(DREG), 3'b010, 5'(off), 7'b0100111};  // fsw
            18: begin                                                             // fflags / fcsr: read, write, set, clear
                automatic logic [2:0] f3s [] = '{3'b001, 3'b010, 3'b011, 3'b101, 3'b110, 3'b111};
                automatic logic [2:0] f3 = f3s[$urandom_range(0, 5)];
                return enc_csr(f3, xd, f3[2] ? $urandom_range(0, 31) : x1, ($urandom % 2 == 1) ? 12'h001 : 12'h003);
            end
            default: begin                                                        // frm: read, or write a valid mode
                // (rarely an invalid one: a later op with rm = DYN then raises illegal instruction)
                automatic int m = ($urandom % 60 == 0) ? $urandom_range(5, 7) : $urandom_range(0, 4);
                return ($urandom % 2 == 1) ? enc_csr(3'b010, xd, 0, 12'h002) : enc_csr(3'b101, xd, m, 12'h002);
            end
        endcase
    endfunction

    // ---- ISS (RV32IMF: ALU subset, branches, jal, jalr, loads/stores, F) ----
    logic [31:0] iss_reg [32];
    logic [31:0] iss_freg [32];                   // f registers
    logic [2:0]  iss_frm;
    logic [4:0]  iss_fflags;
    bit          iss_wfp;                         // the last instruction wrote an f register
    longint      iss_f = 0;                       // F instructions executed
    longint      iss_csr = 0;                     // fcsr CSR ops executed
    bit          iss_resv_v;                      // LR reservation (cleared by SC or any store to its word)
    logic [31:2] iss_resv_word;
    longint      iss_amo = 0;                     // atomics executed
    logic [31:0] iss_pc;
    logic [7:0]  init_mem [`MEM_SIZE_IN_BYTES];   // program image + data (what mem.sv starts with)
    logic [7:0]  iss_mem  [`MEM_SIZE_IN_BYTES];   // the ISS's memory
    longint      iss_ld = 0, iss_st = 0;
    longint      iss_branches;

    // execute the instruction at iss_pc. Returns 0 normally, 1 for WFI (or past the program),
    // 2 for an exception (exc = cause; nothing is written, the PC does not advance)
    localparam int ISS_OK = 0, ISS_WFI = 1, ISS_EXC = 2;

    function automatic bit is_f(input logic [31:0] i);
        return i[6:0] inside {7'b0000111, 7'b0100111, 7'b1010011, 7'b1000011, 7'b1000111, 7'b1001011, 7'b1001111};
    endfunction

    // RV32F encodings the decoder accepts (single precision only; rm 101/110 reserved)
    function automatic bit f_legal(input logic [31:0] i);
        automatic logic [2:0] f3 = i[14:12];
        automatic logic [4:0] f5 = i[31:27], r2 = i[24:20];
        automatic bit rm_ok = (f3 != 3'b101 && f3 != 3'b110);
        case (i[6:0])
            7'b0000111, 7'b0100111: return f3 == 3'b010;                          // flw / fsw
            7'b1010011: begin
                if (i[26:25] != 2'b00) return 0;
                case (f5)
                    5'b00000, 5'b00001, 5'b00010, 5'b00011: return rm_ok;
                    5'b01011:           return rm_ok && r2 == 0;
                    5'b00100, 5'b10100: return f3 <= 3'd2;
                    5'b00101:           return f3 <= 3'd1;
                    5'b11000, 5'b11010: return rm_ok && r2 <= 5'd1;
                    5'b11100:           return r2 == 0 && f3 <= 3'd1;
                    5'b11110:           return r2 == 0 && f3 == 3'd0;
                    default:            return 0;
                endcase
            end
            default: return i[26:25] == 2'b00 && rm_ok;                           // fused multiply-add
        endcase
    endfunction
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
        // a known-illegal word
        if (i[6:0] == 7'b1111111 || i == 32'h0) begin
            writes = 0; exc = EXC_ILLEGAL; return ISS_EXC;
        end
        // FENCE / FENCE.I: no-ops on a single hart (memory is already seen in program order)
        iss_wfp = 0;
        if (i[6:0] == 7'b0001111) begin
            writes = 0;
            iss_pc = npc;
            return ISS_OK;
        end
        // A: LR / SC / AMO (address = rs1). Any store to the reserved word clears the reservation.
        if (i[6:0] == 7'b0101111) begin
            automatic logic [4:0]  f5 = i[31:27];
            automatic logic [31:0] old = '0, nw;
            automatic bit          wr = 0;
            if (i[14:12] != 3'b010 || !(f5 inside {5'b00010, 5'b00011, 5'b00001, 5'b00000, 5'b00100, 5'b01100,
                                                     5'b01000, 5'b10000, 5'b10100, 5'b11000, 5'b11100}) ||
                (f5 == 5'b00010 && i[24:20] != 0)) begin writes = 0; exc = EXC_ILLEGAL; return ISS_EXC; end
            if (`A_EXTENSION == 0)              begin writes = 0; exc = EXC_UNSUPPORTED; return ISS_EXC; end
            if (a[1:0] != 0)                    begin writes = 0; exc = EXC_MISALIGNED;  return ISS_EXC; end
            if (a >= `MEM_SIZE_IN_BYTES)        begin writes = 0; exc = EXC_ACCESS;      return ISS_EXC; end
            for (int k = 0; k < 4; k++) old[8 * k +: 8] = iss_mem[int'(a) + k];
            case (f5)
                5'b00010: begin value = old; iss_resv_v = 1; iss_resv_word = a[31:2]; end   // lr.w
                5'b00011: begin                                                           // sc.w
                    wr    = iss_resv_v && iss_resv_word == a[31:2];
                    value = wr ? 0 : 1; nw = b;
                    iss_resv_v = 0;
                end
                default: begin
                    value = old; wr = 1;
                    case (f5)
                        5'b00001: nw = b;
                        5'b00000: nw = old + b;
                        5'b00100: nw = old ^ b;
                        5'b01100: nw = old & b;
                        5'b01000: nw = old | b;
                        5'b10000: nw = ($signed(old) < $signed(b)) ? old : b;
                        5'b10100: nw = ($signed(old) > $signed(b)) ? old : b;
                        5'b11000: nw = (old < b) ? old : b;
                        default:  nw = (old > b) ? old : b;
                    endcase
                end
            endcase
            if (wr) begin
                for (int k = 0; k < 4; k++) iss_mem[int'(a) + k] = nw[8 * k +: 8];
                if (iss_resv_v && a[31:2] == iss_resv_word) iss_resv_v = 0;
            end
            if (rd == 0) writes = 0;
            else iss_reg[rd] = value;
            iss_amo++;
            iss_pc = npc;
            return ISS_OK;
        end
        // CSR ops: fflags / frm / fcsr with an FP unit; any other CSR is unsupported
        iss_wfp = 0;
        if (i[6:0] == 7'b1110011 && i[14:12] != 3'b000) begin
            automatic logic [7:0] old, src, nw;
            automatic logic [11:0] addr = i[31:20];
            if (i[14:12] == 3'b100) begin writes = 0; exc = EXC_ILLEGAL; return ISS_EXC; end
            if (!(`NUM_FU_FP > 0 && addr inside {12'h001, 12'h002, 12'h003})) begin
                writes = 0; exc = EXC_UNSUPPORTED; return ISS_EXC;
            end
            old = (addr == 12'h001) ? {3'b0, iss_fflags} : (addr == 12'h002) ? {5'b0, iss_frm} : {iss_frm, iss_fflags};
            src = i[14] ? {3'b0, i[19:15]} : a[7:0];
            nw  = (i[13:12] == 2'b01) ? src : (i[13:12] == 2'b10) ? (old | src) : (old & ~src);
            if (addr == 12'h001)      iss_fflags = nw[4:0];
            else if (addr == 12'h002) iss_frm    = nw[2:0];
            else                      {iss_frm, iss_fflags} = nw;
            value = {24'b0, old};
            if (rd == 0) writes = 0;
            else iss_reg[rd] = value;
            iss_csr++;
            iss_pc = npc;
            return ISS_OK;
        end
        // F extension (unsupported without an FP unit)
        if (is_f(i)) begin
            automatic bit uses_rm = (i[6:0] != 7'b1010011) ? (i[6:0] != 7'b0000111 && i[6:0] != 7'b0100111)
                                  : (i[31:27] inside {5'b00000, 5'b00001, 5'b00010, 5'b00011, 5'b01011, 5'b11000, 5'b11010});
            if (`NUM_FU_FP == 0) begin writes = 0; exc = EXC_UNSUPPORTED; return ISS_EXC; end
            if (!f_legal(i))     begin writes = 0; exc = EXC_ILLEGAL;     return ISS_EXC; end
            if (uses_rm && i[14:12] == 3'b111 && iss_frm > 3'd4) begin writes = 0; exc = EXC_ILLEGAL; return ISS_EXC; end
            iss_f++;
            if (i[6:0] == 7'b0000111 || i[6:0] == 7'b0100111) begin                // flw / fsw
                automatic bit          st   = (i[6:0] == 7'b0100111);
                automatic logic [31:0] addr = st ? a + {{20{i[31]}}, i[31:25], i[11:7]} : a + im;
                if (addr[1:0] != 0)             begin writes = 0; exc = EXC_MISALIGNED; return ISS_EXC; end
                if (addr >= `MEM_SIZE_IN_BYTES) begin writes = 0; exc = EXC_ACCESS;     return ISS_EXC; end
                if (st) begin
                    for (int k = 0; k < 4; k++) iss_mem[int'(addr) + k] = iss_freg[i[24:20]][8 * k +: 8];
                    if (iss_resv_v && addr[31:2] == iss_resv_word) iss_resv_v = 0;
                    writes = 0; iss_st++;
                end else begin
                    for (int k = 0; k < 4; k++) value[8 * k +: 8] = iss_mem[int'(addr) + k];
                    iss_freg[rd] = value; iss_wfp = 1; iss_ld++;
                end
            end else begin
                automatic bit x_src = (i[6:0] == 7'b1010011) && (i[31:27] == 5'b11010 || i[31:27] == 5'b11110);
                automatic bit x_dst = (i[6:0] == 7'b1010011) && (i[31:27] inside {5'b10100, 5'b11000, 5'b11100});
                automatic int unsigned fl;
                value = fp_ref(i, x_src ? a : iss_freg[i[19:15]], iss_freg[i[24:20]], iss_freg[i[31:27]],
                               32'(iss_frm), fl);
                iss_fflags |= 5'(fl);
                if (x_dst) begin
                    if (rd == 0) writes = 0;
                    else iss_reg[rd] = value;
                end else begin
                    iss_freg[rd] = value; iss_wfp = 1;
                end
            end
            iss_pc = npc;
            return ISS_OK;
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
                if (iss_resv_v && addr[31:2] == iss_resv_word) iss_resv_v = 0;
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
        for (int r = 0; r < 32; r++) begin iss_reg[r] = 0; iss_freg[r] = 0; end
        iss_frm = 0; iss_fflags = 0; iss_wfp = 0; iss_resv_v = 0;
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
        // f registers too (the FP PRF has no reset): fmv.w.x f_r, x_(1 + r % 27)
        if (`NUM_FU_FP > 0)
            for (int r = 0; r < 32; r++) emit(opfp(5'b11110, 0, 1 + r % 27, 3'b000, r));
    endtask

    function automatic logic [31:0] enc_amo(input logic [4:0] f5, input int rd, input int rs1, input int rs2);
        return {f5, 2'($urandom), 5'(rs2), 5'(rs1), 3'b010, 5'(rd), 7'b0101111};   // random aq / rl
    endfunction

    // x_r = v (lui + addi)
    task automatic xconst(input int r, input logic [31:0] v);
        emit(enc_u(7'b0110111, r, 20'((v + 32'h800) >> 12)));
        emit(enc_i(3'b000, r, r, v[11:0]));
    endtask

    // a random atomic sequence on the data window: a pointer xp (in dlo..dhi, never x7 / x28),
    // then an AMO, an SC alone, or LR ... SC with random instructions (and stores) in between
    task automatic emit_amo_seq(input int dlo, input int dhi);
        logic [4:0] amos [] = '{5'b00001, 5'b00000, 5'b00100, 5'b01100, 5'b01000, 5'b10000, 5'b10100, 5'b11000, 5'b11100};
        // the pointer is the top register of the range; nothing in the sequence writes it
        int lo = (dlo < 1) ? 1 : dlo, hi = (dhi > 6) ? 6 : dhi;
        int xp = hi, xd = $urandom_range(dlo, hi - 1), xs = $urandom_range(0, 7);
        int kind = $urandom_range(0, 5), gap = $urandom_range(0, 3);
        int off = ($urandom % 400 == 0) ? 4 * $urandom_range(0, 15) + 2 : 4 * $urandom_range(0, 15);   // rarely misaligned
        emit(enc_i(3'b000, xp, DREG, 12'(off)));                         // xp = x28 + off
        case (kind)
            0, 1, 2: emit(enc_amo(amos[$urandom_range(0, 8)], xd, xp, xs));
            3:       emit(enc_amo(5'b00011, xd, xp, xs));                 // sc.w with no lr
            default: begin                                                // lr.w ... sc.w
                emit(enc_amo(5'b00010, xd, xp, 0));
                for (int n = 0; n < gap; n++) emit(rand_alu(8, dlo, hi - 1));
                emit(enc_amo(5'b00011, $urandom_range(dlo, hi - 1), xp, xs));
            end
        endcase
    endtask

    // f_r = v (through x9: lui + addi + fmv.w.x)
    task automatic fconst(input int r, input logic [31:0] v);
        emit(enc_u(7'b0110111, 9, 20'((v + 32'h800) >> 12)));
        emit(enc_i(3'b000, 9, 9, v[11:0]));
        emit(opfp(5'b11110, 0, 9, 3'b000, r));
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
    longint total_f = 0;              // F instructions executed
    longint total_csr = 0;            // fcsr CSR ops executed
    longint total_amo = 0;            // atomics executed
    // FP unit (Phase 6): issues, cycles the port refuses work (div/sqrt busy, buffer full),
    // cycles an FP result waits for a CDB, dispatch stalls on the FP free list
    localparam int FPP = `NUM_FU_ALU + `NUM_FU_BRANCH + `NUM_FU_MULT + `NUM_FU_DIV + `NUM_FU_LOAD + `NUM_FU_STORE;
    longint fp_issue = 0, fp_blocked = 0, fp_cdb_wait = 0, stall_fp_freelist = 0;
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
        // the FP class: every FP preg (f0's initial one included) is mapped or free, exactly once
        begin
            automatic int fcount [`FP_PHYS_REG_SZ];
            automatic int ffl = `FP_FREE_LIST_SZ;
            foreach (fcount[t]) fcount[t] = 0;
            if (int'(dut.dispatch_0.fp_freelist_0.free_count) != ffl)
                fail($sformatf("[%s] FP free list holds %0d tags after halt, expected %0d", name,
                               dut.dispatch_0.fp_freelist_0.free_count, ffl));
            for (int r = 0; r < 32; r++) fcount[int'(dut.fp_retire_map[r]) - `FP_TAG_BASE]++;
            for (int i = 0; i < ffl; i++) fcount[int'(dut.dispatch_0.fp_freelist_0.freelist_mem[i]) - `FP_TAG_BASE]++;
            for (int t = 0; t < `FP_PHYS_REG_SZ; t++)
                if (fcount[t] != 1) fail($sformatf("[%s] FP preg %0d appears %0d times (retirement map + free list)",
                                                   name, t, fcount[t]));
        end
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
        iss_m = 0; iss_f = 0; iss_csr = 0; iss_amo = 0;
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
            if (`NUM_FU_FP > 0) begin
                fp_issue    += int'(dut.issue_valid[FPP]);
                fp_blocked  += int'(!dut.fu_ready[FPP]);
                fp_cdb_wait += int'(dut.fu_out_valid[FPP] && !dut.fu_grant[FPP]);
            end
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
                if (!dut.dispatch_0.fp_can_alloc_w) stall_fp_freelist++;
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
                if (trace_fd != 0) $fdisplay(trace_fd, "%h %s%0d %h", commit_pc[i], iss_wfp ? "f" : "x",
                                             writes ? rd : 0, writes ? value : 32'h0);
                if (dut.commit_exception[i] !== e)
                    fail($sformatf("[%s] PC %h (%h): commit_exception=%s, expected %s", name, commit_pc[i],
                                   imem[commit_pc[i] >> 2], dut.commit_exception[i].name(), e.name()));
                if (how != ISS_OK) begin done = 1; continue; end
                if (commit_dest_valid[i] !== writes)
                    fail($sformatf("[%s] PC %h (%h): commit_dest_valid=%b, expected %b", name, commit_pc[i],
                                   imem[commit_pc[i] >> 2], commit_dest_valid[i], writes));
                else if (writes) begin
                    if (commit_rd[i] !== 5'(rd) || dut.commit_dest_fp[i] !== iss_wfp)
                        fail($sformatf("[%s] PC %h: rd=%s%0d, expected %s%0d", name, commit_pc[i],
                                       dut.commit_dest_fp[i] ? "f" : "x", commit_rd[i], iss_wfp ? "f" : "x", rd));
                    else if (iss_wfp ? (dut.fp_prf_0.registers[int'(commit_tag[i]) - `FP_TAG_BASE] !== value)
                                     : (dut.prf_0.registers[commit_tag[i]] !== value))
                        fail($sformatf("[%s] PC %h (%h): %s%0d = %h, expected %h", name, commit_pc[i],
                                       imem[commit_pc[i] >> 2], iss_wfp ? "f" : "x", rd,
                                       iss_wfp ? dut.fp_prf_0.registers[int'(commit_tag[i]) - `FP_TAG_BASE]
                                               : dut.prf_0.registers[commit_tag[i]], value));
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
        if (`NUM_FU_FP > 0 && end_exc == EXC_NONE && dut.fcsr !== {iss_frm, iss_fflags})
            fail($sformatf("[%s] final fcsr = %h, expected %h", name, dut.fcsr, {iss_frm, iss_fflags}));
        if (`NUM_FU_FP > 0)
            for (int r = 0; r < 32; r++)
                if (dut.fp_prf_0.registers[int'(dut.fp_retire_map[r]) - `FP_TAG_BASE] !== iss_freg[r])
                    fail($sformatf("[%s] final f%0d = %h, expected %h", name, r,
                                   dut.fp_prf_0.registers[int'(dut.fp_retire_map[r]) - `FP_TAG_BASE], iss_freg[r]));
        // committed stores may still be draining: wait, then memory = mem.sv + dirty D-cache blocks
        begin
            automatic int g = 0;
            while ((stores_pending || !dut.dcache_0.idle) && g < 2000) begin @(negedge clock); g++; end
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
        total_f         += iss_f;
        total_csr       += iss_csr;
        total_amo       += iss_amo;
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
        if (last_replays != (`LSQ_SPECULATE != 0 ? 1 : 0))
            fail($sformatf("unknown store address: %0d replays, expected %0d (only the same-address load violates)",
                           last_replays, `LSQ_SPECULATE != 0 ? 1 : 0));

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
        for (int n = 0; n < 16; n++) begin                  // one way's worth of bytes apart: same set
            xconst(9, DBASE + n * (`DC_SZ / `DC_WAYS));
            emit(enc_store(3'b010, 1 + n % 7, 9, 12'd0));
        end
        for (int n = 0; n < 16; n++) begin
            xconst(9, DBASE + n * (`DC_SZ / `DC_WAYS));
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

        // ---- F extension (directed) ----
        if (`NUM_FU_FP > 0) begin
            $display("Test 29: fmadd whose rs3 is written by the older slot of the same fetch bundle");
            start_program();
            for (n = 0; n < 40; n++) begin
                if (prog_len % 2 == 1) emit(32'h0000_0013);                    // align: the pair shares a block
                emit(opfp(5'b00000, 2 + n % 3, 1, 3'b000, 3));                 // fadd f3, f1, f2..
                emit({5'd3, 2'b00, 5'd6, 5'd5, 3'b000, 5'(4 + n % 2), 7'b1000011});   // fmadd f4/5, f5, f6, f3
            end
            run_program("same-bundle rs3");

            $display("Test 30: fdiv, then independent fadds, then its result");
            start_program();
            emit(opfp(5'b00011, 2, 1, 3'b000, 7));                            // fdiv f7, f1, f2
            for (n = 0; n < 12; n++) emit(opfp(5'b00000, n % 4, 4, 3'b000, 3 + n % 3));
            emit(opfp(5'b00010, 7, 7, 3'b000, 6));                            // fmul f6, f7, f7
            run_program("fdiv overtaken");

            $display("Test 31: forwarding between integer and FP loads/stores");
            start_program();
            emit(enc_store(3'b010, 5, DREG, 12'd0));                          // sw x5, 0(x28)
            emit({12'd0, 5'(DREG), 3'b010, 5'd6, 7'b0000111});                // flw f6, 0(x28)
            emit({7'd0, 5'd6, 5'(DREG), 3'b010, 5'd4, 7'b0100111});           // fsw f6, 4(x28)
            emit(enc_load(3'b010, 9, DREG, 12'd4));                           // lw x9, 4(x28)
            emit(opfp(5'b11100, 0, 6, 3'b000, 10));                           // fmv.x.w x10, f6
            run_program("sw->flw, fsw->lw");

            $display("Test 32: fsw whose data comes from an fdiv");
            start_program();
            emit(opfp(5'b00011, 2, 1, 3'b000, 5));                            // fdiv f5, f1, f2
            emit({7'd0, 5'd5, 5'(DREG), 3'b010, 5'd8, 7'b0100111});           // fsw f5, 8(x28)
            emit(enc_load(3'b010, 11, DREG, 12'd8));                          // lw x11, 8(x28)
            run_program("fsw data from fdiv");

            $display("Test 33: f0 is a real register (source and destination)");
            start_program();
            emit(opfp(5'b00000, 2, 1, 3'b000, 0));                            // fadd f0, f1, f2
            emit(opfp(5'b00010, 0, 0, 3'b000, 3));                            // fmul f3, f0, f0
            emit(opfp(5'b00100, 3, 0, 3'b001, 0));                            // fsgnjn f0, f0, f3
            emit(opfp(5'b11100, 0, 0, 3'b000, 12));                           // fmv.x.w x12, f0
            run_program("f0");

            $display("Test 34: FP results into x0 are discarded");
            start_program();
            emit(opfp(5'b11100, 0, 1, 3'b000, 0));                            // fmv.x.w x0, f1
            emit(opfp(5'b11000, 0, 2, 3'b000, 0));                            // fcvt.w.s x0, f2
            emit(opfp(5'b10100, 2, 1, 3'b010, 0));                            // feq.s x0, f1, f2
            emit(opfp(5'b11100, 0, 3, 3'b001, 0));                            // fclass.s x0, f3
            emit(enc_r(7'b0000000, 3'b000, 13, 0, 0));                        // add x13, x0, x0
            run_program("x0 destinations");

            $display("Test 36: fsrm, then divides with rm = DYN round by the new mode");
            start_program();
            fconst(1, 32'h3f80_0000); fconst(2, 32'h4040_0000);                   // 1.0, 3.0
            emit(enc_csr(3'b101, 0, 2, 12'h002));                             // fsrmi RDN
            emit(opfp(5'b00011, 2, 1, 3'b111, 3));                            // fdiv f3, f1, f2 (DYN)
            emit(enc_csr(3'b101, 0, 3, 12'h002));                             // fsrmi RUP
            emit(opfp(5'b00011, 2, 1, 3'b111, 4));                            // fdiv f4, f1, f2 (DYN)
            emit(enc_csr(3'b010, 10, 0, 12'h002));                            // frrm x10
            run_program("fsrm + dynamic rounding");

            $display("Test 36b: fsrm and a DYN fdiv in the same fetch block (the fdiv must see the new mode)");
            start_program();
            fconst(1, 32'h3f80_0000); fconst(2, 32'h4040_0000);
            emit(enc_i(3'b000, 11, 0, 12'd1));
            for (n = 0; n < 16; n++) begin
                // an older slow divide keeps the fsrm from the ROB head while the fdiv is ready
                emit(enc_r(7'b0000001, 3'b100, 8, DREG, 11));                  // div x8, x28, x11
                if (prog_len % 2 == 1) emit(32'h0000_0013);                    // fsrm in slot 0, fdiv in slot 1
                emit(enc_csr(3'b101, 0, n % 5, 12'h002));                      // fsrmi (RNE, RTZ, RDN, RUP, RMM)
                emit(opfp(5'b00011, 2, 1, 3'b111, 3 + n % 4));                 // fdiv f3.., f1, f2 (DYN)
            end
            run_program("fsrm + DYN in one block");

            $display("Test 37: frflags after ops that raise each flag");
            start_program();
            fconst(1, 32'h3f80_0000); fconst(2, 32'h0000_0000); fconst(3, 32'hbf80_0000);   // 1.0, 0.0, -1.0
            fconst(4, 32'h7f7f_ffff); fconst(5, 32'h0080_0000); fconst(6, 32'h4040_0000);   // max, min normal, 3.0
            emit(enc_csr(3'b101, 0, 0, 12'h001));                             // fsflagsi 0
            emit(opfp(5'b00011, 2, 1, 3'b000, 7));  emit(enc_csr(3'b010, 10, 0, 12'h001));  // 1/0: DZ
            emit(enc_csr(3'b101, 0, 0, 12'h001));
            emit(opfp(5'b01011, 0, 3, 3'b000, 7));  emit(enc_csr(3'b010, 11, 0, 12'h001));  // sqrt(-1): NV
            emit(enc_csr(3'b101, 0, 0, 12'h001));
            emit(opfp(5'b00010, 4, 4, 3'b000, 7));  emit(enc_csr(3'b010, 12, 0, 12'h001));  // max*max: OF NX
            emit(enc_csr(3'b101, 0, 0, 12'h001));
            emit(opfp(5'b00010, 5, 5, 3'b000, 7));  emit(enc_csr(3'b010, 13, 0, 12'h001));  // tiny*tiny: UF NX
            emit(enc_csr(3'b101, 0, 0, 12'h001));
            emit(opfp(5'b00011, 6, 1, 3'b000, 7));  emit(enc_csr(3'b010, 14, 0, 12'h001));  // 1/3: NX
            emit(enc_csr(3'b010, 15, 0, 12'h003));                            // frcsr x15
            run_program("flags");

            $display("Test 38: flags raised on a mispredicted path never reach fflags");
            start_program();
            fconst(1, 32'h3f80_0000); fconst(0, 32'h0000_0000); fconst(7, 32'hbf80_0000);
            emit(enc_csr(3'b101, 0, 0, 12'h001));                             // fsflagsi 0
            emit(enc_i(3'b000, 11, 0, 12'd1));
            emit(enc_r(7'b0000001, 3'b100, 8, DREG, 11));                     // div x8, x28, x11 (slow, non-zero)
            emit(enc_b(3'b001, 8, 0, 12));                                    // bne x8, x0 (taken; predicted not)
            emit(opfp(5'b00011, 0, 1, 3'b000, 5));                            //   wrong path: 1/0 -> DZ
            emit(opfp(5'b01011, 0, 7, 3'b000, 6));                            //   wrong path: sqrt(-1) -> NV
            emit(enc_csr(3'b010, 12, 0, 12'h001));                            // frflags x12: must be 0
            run_program("wrong-path flags");

            $display("Test 39: frm = 5 (invalid), then an op with rm = DYN raises illegal instruction");
            start_program();
            emit(enc_csr(3'b101, 0, 5, 12'h002));                             // fsrmi 5
            emit(opfp(5'b00000, 2, 1, 3'b000, 3));                            // static rm: fine
            emit(opfp(5'b00000, 2, 1, 3'b111, 4));                            // DYN: illegal
            emit(enc_i(3'b000, 10, 0, 12'd1));
            run_program("invalid frm");

            $display("Test 40: a CSR op first in its fetch bundle; the next instruction reads its result");
            start_program();
            fconst(1, 32'h3f80_0000); fconst(2, 32'h4040_0000);
            for (n = 0; n < 12; n++) begin
                if (prog_len % 2 == 1) emit(32'h0000_0013);
                emit(enc_csr(3'b010, 10, 0, 12'h001));                        // frflags x10 (slot 0)
                emit(enc_r(7'b0000000, 3'b000, 11, 10, 10));                  // add x11, x10, x10 (slot 1)
                if (n % 2 == 0) emit(opfp(5'b00011, 2, 1, 3'b000, 3));        // 1/3: NX
                else            emit(enc_csr(3'b111, 0, 1, 12'h001));         // csrrci fflags, NX
            end
            run_program("CSR first in bundle");

            $display("Test 41: a CSR op on a mispredicted path (must not block dispatch or write fcsr)");
            start_program();
            fconst(1, 32'h3f80_0000); fconst(2, 32'h4040_0000);
            emit(enc_csr(3'b101, 0, 0, 12'h002));                             // fsrmi RNE
            emit(enc_i(3'b000, 11, 0, 12'd1));
            emit(enc_r(7'b0000001, 3'b100, 8, DREG, 11));                     // div x8, x28, x11
            emit(enc_b(3'b001, 8, 0, 8));                                     // bne x8, x0 (taken; predicted not)
            emit(enc_csr(3'b101, 0, 3, 12'h002));                             //   wrong path: fsrmi RUP
            emit(opfp(5'b00011, 2, 1, 3'b111, 3));                            // fdiv f3, f1, f2 (DYN: still RNE)
            emit(enc_csr(3'b010, 12, 0, 12'h002));                            // frrm x12: 0
            run_program("wrong-path CSR");

            $display("Test 35: 100 FP-heavy random programs (branches, loops, calls, mispredicts, CSR ops)");
            for (int p = 0; p < 100; p++) begin
                start_program();
                while (prog_len < 94 + 300) begin
                    if ($urandom % 5 == 0) emit_block(0, 7, 1);
                    else emit(rand_fp(8, 0, 7));
                end
                run_program($sformatf("fp random %0d", p));
                if (errors > 0) break;
            end
        end

        // ---- A extension (directed) ----
        if (`A_EXTENSION != 0) begin
            $display("Test 42: LR / SC succeeds");
            start_program();
            emit(enc_amo(5'b00010, 10, DREG, 0));                             // lr.w x10, (x28)
            emit(enc_i(3'b000, 10, 10, 12'd5));
            emit(enc_amo(5'b00011, 11, DREG, 10));                            // sc.w x11, x10, (x28): 0
            emit(enc_load(3'b010, 12, DREG, 12'd0));
            run_program("lr/sc");

            $display("Test 43: SC without LR fails and writes nothing");
            start_program();
            emit(enc_amo(5'b00011, 11, DREG, 5));
            emit(enc_load(3'b010, 12, DREG, 12'd0));
            run_program("sc alone");

            $display("Test 44: a second SC fails (the first cleared the reservation)");
            start_program();
            emit(enc_amo(5'b00010, 10, DREG, 0));
            emit(enc_amo(5'b00011, 11, DREG, 5));
            emit(enc_amo(5'b00011, 12, DREG, 6));
            emit(enc_load(3'b010, 13, DREG, 12'd0));
            run_program("sc after sc");

            $display("Test 45: a store to the reserved word clears it; a store elsewhere does not");
            start_program();
            emit(enc_amo(5'b00010, 10, DREG, 0));
            emit(enc_store(3'b000, 5, DREG, 12'd3));                          // sb into the reserved word
            emit(enc_amo(5'b00011, 11, DREG, 6));                             // fails
            emit(enc_amo(5'b00010, 10, DREG, 0));
            emit(enc_store(3'b010, 5, DREG, 12'd4));                          // sw to the next word
            emit(enc_amo(5'b00011, 12, DREG, 6));                             // succeeds
            emit(enc_load(3'b010, 13, DREG, 12'd0));
            run_program("store vs reservation");

            $display("Test 46: every AMO on signed / unsigned edge values");
            begin
                automatic logic [4:0]  amos [] = '{5'b00001, 5'b00000, 5'b00100, 5'b01100, 5'b01000, 5'b10000, 5'b10100, 5'b11000, 5'b11100};
                automatic logic [31:0] vals [] = '{32'h8000_0000, 32'h7fff_ffff, 32'hffff_ffff, 32'h0, 32'h1};
                start_program();
                foreach (amos[k])
                    foreach (vals[m]) begin
                        xconst(9, vals[m]);  emit(enc_store(3'b010, 9, DREG, 12'd8));      // memory word
                        xconst(10, vals[(m + k + 1) % 5]);                                // rs2
                        emit(enc_i(3'b000, 6, DREG, 12'd8));
                        emit(enc_amo(amos[k], 11, 6, 10));
                        emit(enc_load(3'b010, 12, DREG, 12'd8));
                    end
                run_program("AMO edge values");
            end

            $display("Test 47: rd = x0, and rd = rs2");
            start_program();
            emit(enc_amo(5'b00000, 0, DREG, 5));                              // amoadd.w x0, x5, (x28)
            emit(enc_amo(5'b00001, 5, DREG, 5));                              // amoswap.w x5, x5, (x28)
            emit(enc_amo(5'b00010, 0, DREG, 0));                              // lr.w x0, (x28)
            emit(enc_amo(5'b00011, 0, DREG, 6));                              // sc.w x0, x6, (x28)
            emit(enc_load(3'b010, 12, DREG, 12'd0));
            run_program("x0 / rs2 destinations");

            $display("Test 48: misaligned and out-of-range atomics raise their exceptions");
            start_program();
            emit(enc_i(3'b000, 9, DREG, 12'd2));
            emit(enc_amo(5'b00000, 10, 9, 5));                                // misaligned
            run_program("misaligned amo");
            start_program();
            emit(enc_u(7'b0110111, 9, 20'h10));                                // x9 = 0x10000
            emit(enc_amo(5'b00010, 10, 9, 0));                                // out of range
            run_program("out-of-range lr");

            $display("Test 49: an atomic on a mispredicted path never touches memory");
            start_program();
            emit(enc_i(3'b000, 11, 0, 12'd1));
            emit(enc_r(7'b0000001, 3'b100, 8, DREG, 11));                     // div x8, x28, x11
            emit(enc_b(3'b001, 8, 0, 8));                                     // bne x8, x0 (taken; predicted not)
            emit(enc_amo(5'b00001, 0, DREG, 5));                              //   wrong path: amoswap
            emit(enc_load(3'b010, 12, DREG, 12'd0));
            run_program("wrong-path amo");

            $display("Test 50: 150 random programs with atomics (and loads / stores on the same words)");
            for (int p = 0; p < 150; p++) begin
                automatic int start;
                start_program();
                start = prog_len;
                while (prog_len < start + 250) begin
                    case ($urandom % 6)
                        0:       emit_block(0, 7, 1);
                        1, 2:    emit_amo_seq(1, 6);
                        default: emit(rand_alu(8, 0, 7));
                    endcase
                end
                run_program($sformatf("amo random %0d", p));
                if (errors > 0) break;
            end
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
        $display("  (F instructions executed: %0d, fcsr CSR ops: %0d, atomics: %0d)", total_f, total_csr, total_amo);
        $display("  (FP port: %0d issues, refused work %0d cycles, results waited for a CDB %0d cycles; FP free list stalled dispatch %0d cycles)",
                 fp_issue, fp_blocked, fp_cdb_wait, stall_fp_freelist);
        $display("  (I-cache: fetch waited %0d of %0d cycles)", total_istall, total_cycles);
        $display("  (loads %0d, stores %0d committed; %0d loads forwarded from the store queue; D-cache %0d hits, %0d misses, %0d writebacks)",
                 total_ld, total_st, total_fwd, dut.dcache_0.n_hits, dut.dcache_0.n_misses, dut.dcache_0.n_writebacks);
        begin
            automatic string occ = "";
            foreach (dut.dcache_0.occupancy[k]) occ = {occ, $sformatf(" %0d:%0d", k, dut.dcache_0.occupancy[k])};
            $display("  (DC_MSHRS = %0d: %0d misses merged, %0d hits under a miss, %0d cycles a request was refused; cycles with k MSHRs busy:%s)",
                     `DC_MSHRS, dut.dcache_0.n_merged, dut.dcache_0.n_hit_under_miss, dut.dcache_0.n_refused, occ);
        end
        $display("  (LSQ_SPECULATE = %0d: %0d load replays)", `LSQ_SPECULATE, total_replays);
        if (errors == 0) $display("@@@ Passed");
        else             $display("@@@ Failed (%0d errors)", errors);
        $finish;
    endtask

endmodule
