/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  decoder.sv                                          //
//                                                                     //
//  Description :  Instruction decoder, from the P3 stage_id.sv,       //
//                 extended for RV32IMAF. The original P3 outputs and  //
//                 their behaviour for P3 instructions are unchanged.  //
//                 Added: fu_type (which functional unit; branches and //
//                 jal/jalr -> FU_BRANCH), register                    //
//                 class per operand (rs1_fp/rs2_fp/rd_fp), rs3_used   //
//                 (fused multiply-add), atomic (LR/SC/AMO),           //
//                 rs1_used/rs2_used (the field names a source         //
//                 register, so the RS never waits on immediate        //
//                 bits); and                                          //
//                 decode for div/rem, FENCE/FENCE.I (no-ops), the     //
//                 CSR immediate forms, A, and F. FP ops are only      //
//                 classified here; the FP unit decodes the exact op   //
//                 from inst. Reserved rounding modes (101, 110) on    //
//                 FP ops are illegal.                                 //
//                 Note: f0 is a real register, so "rd != 0" must only //
//                 suppress renaming when !rd_fp.                      //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"
`include "verilog/ISA.svh"

// Decode an instruction: generate useful datapath control signals by matching the RISC-V ISA
// This module is purely combinational
module decoder (
    input INST  inst,
    input logic valid, // when low, ignore inst. Output will look like a NOP

    output ALU_OPA_SELECT opa_select,
    output ALU_OPB_SELECT opb_select,
    output logic          has_dest, // if there is a destination register
    output ALU_FUNC       alu_func,
    output logic          rd_mem, wr_mem, cond_branch, uncond_branch,
    output logic          csr_op, // used for CSR operations, we only use this as a cheap way to get the return code out
    output logic          halt,   // non-zero on a halt
    output logic          illegal, // non-zero on an illegal instruction

    // added for the OoO core / RV32IMAF
    output FU_TYPE        fu_type,  // functional unit that executes it
    output logic          rs1_fp,   // 1: rs1 is an f-register (F extension)
    output logic          rs2_fp,   // 1: rs2 is an f-register
    output logic          rd_fp,    // 1: rd is an f-register
    output logic          rs3_used, // fused multiply-add: third FP source rs3 = inst[31:27]
    output logic          atomic,   // LR/SC/AMO (A extension)
    output logic          rs1_used, // 1: inst[19:15] names a source register (else an immediate / unused)
    output logic          rs2_used  // 1: inst[24:20] names a source register
);

    // rounding mode field of FP ops: 101 and 110 are reserved
    logic bad_rm;
    assign bad_rm = (inst[14:12] == 3'b101) || (inst[14:12] == 3'b110);

    // Note: I recommend using an IDE's code folding feature on this block
    always_comb begin
        // Default control values (looks like a NOP)
        // See sys_defs.svh for the constants used here
        opa_select    = OPA_IS_RS1;
        opb_select    = OPB_IS_RS2;
        alu_func      = ALU_ADD;
        has_dest      = `FALSE;
        csr_op        = `FALSE;
        rd_mem        = `FALSE;
        wr_mem        = `FALSE;
        cond_branch   = `FALSE;
        uncond_branch = `FALSE;
        halt          = `FALSE;
        illegal       = `FALSE;
        fu_type       = FU_ALU;
        rs1_fp        = `FALSE;
        rs2_fp        = `FALSE;
        rd_fp         = `FALSE;
        rs3_used      = `FALSE;
        atomic        = `FALSE;
        rs1_used      = `TRUE;    // cleared below where the field is not a register: a missed case
        rs2_used      = `TRUE;    // only costs a false dependence, never a wrong result

        if (valid) begin
            casez (inst)
                `RV32_LUI: begin
                    has_dest   = `TRUE;
                    opa_select = OPA_IS_ZERO;
                    opb_select = OPB_IS_U_IMM;
                    rs1_used   = `FALSE;
                    rs2_used   = `FALSE;
                end
                `RV32_AUIPC: begin
                    has_dest   = `TRUE;
                    opa_select = OPA_IS_PC;
                    opb_select = OPB_IS_U_IMM;
                    rs1_used   = `FALSE;
                    rs2_used   = `FALSE;
                end
                `RV32_JAL: begin
                    has_dest      = `TRUE;
                    opa_select    = OPA_IS_PC;
                    opb_select    = OPB_IS_J_IMM;
                    rs1_used      = `FALSE;
                    rs2_used      = `FALSE;
                    uncond_branch = `TRUE;
                    fu_type       = FU_BRANCH;
                end
                `RV32_JALR: begin
                    has_dest      = `TRUE;
                    opa_select    = OPA_IS_RS1;
                    opb_select    = OPB_IS_I_IMM;
                    rs2_used      = `FALSE;
                    uncond_branch = `TRUE;
                    fu_type       = FU_BRANCH;
                end
                `RV32_BEQ, `RV32_BNE, `RV32_BLT, `RV32_BGE,
                `RV32_BLTU, `RV32_BGEU: begin
                    opa_select  = OPA_IS_PC;
                    opb_select  = OPB_IS_B_IMM;
                    cond_branch = `TRUE;
                    fu_type     = FU_BRANCH;
                end
                `RV32_LB, `RV32_LH, `RV32_LW,
                `RV32_LBU, `RV32_LHU: begin
                    has_dest   = `TRUE;
                    opb_select = OPB_IS_I_IMM;
                    rs2_used   = `FALSE;
                    rd_mem     = `TRUE;
                    fu_type    = FU_LOAD;
                end
                `RV32_SB, `RV32_SH, `RV32_SW: begin
                    opb_select = OPB_IS_S_IMM;
                    wr_mem     = `TRUE;
                    fu_type    = FU_STORE;
                end
                `RV32_ADDI: begin
                    has_dest   = `TRUE;
                    opb_select = OPB_IS_I_IMM;
                    rs2_used   = `FALSE;
                end
                `RV32_SLTI: begin
                    has_dest   = `TRUE;
                    opb_select = OPB_IS_I_IMM;
                    rs2_used   = `FALSE;
                    alu_func   = ALU_SLT;
                end
                `RV32_SLTIU: begin
                    has_dest   = `TRUE;
                    opb_select = OPB_IS_I_IMM;
                    rs2_used   = `FALSE;
                    alu_func   = ALU_SLTU;
                end
                `RV32_ANDI: begin
                    has_dest   = `TRUE;
                    opb_select = OPB_IS_I_IMM;
                    rs2_used   = `FALSE;
                    alu_func   = ALU_AND;
                end
                `RV32_ORI: begin
                    has_dest   = `TRUE;
                    opb_select = OPB_IS_I_IMM;
                    rs2_used   = `FALSE;
                    alu_func   = ALU_OR;
                end
                `RV32_XORI: begin
                    has_dest   = `TRUE;
                    opb_select = OPB_IS_I_IMM;
                    rs2_used   = `FALSE;
                    alu_func   = ALU_XOR;
                end
                `RV32_SLLI: begin
                    has_dest   = `TRUE;
                    opb_select = OPB_IS_I_IMM;
                    rs2_used   = `FALSE;
                    alu_func   = ALU_SLL;
                end
                `RV32_SRLI: begin
                    has_dest   = `TRUE;
                    opb_select = OPB_IS_I_IMM;
                    rs2_used   = `FALSE;
                    alu_func   = ALU_SRL;
                end
                `RV32_SRAI: begin
                    has_dest   = `TRUE;
                    opb_select = OPB_IS_I_IMM;
                    rs2_used   = `FALSE;
                    alu_func   = ALU_SRA;
                end
                `RV32_ADD: begin
                    has_dest   = `TRUE;
                end
                `RV32_SUB: begin
                    has_dest   = `TRUE;
                    alu_func   = ALU_SUB;
                end
                `RV32_SLT: begin
                    has_dest   = `TRUE;
                    alu_func   = ALU_SLT;
                end
                `RV32_SLTU: begin
                    has_dest   = `TRUE;
                    alu_func   = ALU_SLTU;
                end
                `RV32_AND: begin
                    has_dest   = `TRUE;
                    alu_func   = ALU_AND;
                end
                `RV32_OR: begin
                    has_dest   = `TRUE;
                    alu_func   = ALU_OR;
                end
                `RV32_XOR: begin
                    has_dest   = `TRUE;
                    alu_func   = ALU_XOR;
                end
                `RV32_SLL: begin
                    has_dest   = `TRUE;
                    alu_func   = ALU_SLL;
                end
                `RV32_SRL: begin
                    has_dest   = `TRUE;
                    alu_func   = ALU_SRL;
                end
                `RV32_SRA: begin
                    has_dest   = `TRUE;
                    alu_func   = ALU_SRA;
                end
                `RV32_MUL: begin
                    has_dest   = `TRUE;
                    alu_func   = ALU_MUL;
                    fu_type    = FU_MULT;
                end
                `RV32_MULH: begin
                    has_dest   = `TRUE;
                    alu_func   = ALU_MULH;
                    fu_type    = FU_MULT;
                end
                `RV32_MULHSU: begin
                    has_dest   = `TRUE;
                    alu_func   = ALU_MULHSU;
                    fu_type    = FU_MULT;
                end
                `RV32_MULHU: begin
                    has_dest   = `TRUE;
                    alu_func   = ALU_MULHU;
                    fu_type    = FU_MULT;
                end
                // M: divide/remainder -> iterative divider (div_fu)
                `RV32_DIV: begin
                    has_dest   = `TRUE;
                    alu_func   = ALU_DIV;
                    fu_type    = FU_DIV;
                end
                `RV32_DIVU: begin
                    has_dest   = `TRUE;
                    alu_func   = ALU_DIVU;
                    fu_type    = FU_DIV;
                end
                `RV32_REM: begin
                    has_dest   = `TRUE;
                    alu_func   = ALU_REM;
                    fu_type    = FU_DIV;
                end
                `RV32_REMU: begin
                    has_dest   = `TRUE;
                    alu_func   = ALU_REMU;
                    fu_type    = FU_DIV;
                end
                // single core, no self-modifying code: FENCE / FENCE.I act as no-ops
                `RV32_FENCE_, `RV32_FENCE_I: begin
                    rs1_used = `FALSE;
                    rs2_used = `FALSE;
                end
                `RV32_CSRRW, `RV32_CSRRS, `RV32_CSRRC,
                `RV32_CSRRWI, `RV32_CSRRSI, `RV32_CSRRCI: begin
                    csr_op   = `TRUE;
                    has_dest = `TRUE;       // rd gets the old CSR value
                    rs1_used = !inst[14];   // the immediate forms hold a 5-bit uimm there
                    rs2_used = `FALSE;      // part of the CSR number
                end
                `WFI: begin
                    halt     = `TRUE;
                    rs1_used = `FALSE;
                    rs2_used = `FALSE;
                end

                // ---- A: atomics (execute through the LSQ) ----
                `RV32_LR_W: begin
                    has_dest   = `TRUE;
                    rd_mem     = `TRUE;
                    atomic     = `TRUE;
                    fu_type    = FU_LOAD;
                    rs2_used   = `FALSE;
                end
                `RV32_SC_W: begin
                    has_dest   = `TRUE;       // rd = 0 on success, 1 on failure
                    wr_mem     = `TRUE;
                    atomic     = `TRUE;
                    fu_type    = FU_STORE;
                end
                `RV32_AMOSWAP_W, `RV32_AMOADD_W, `RV32_AMOXOR_W, `RV32_AMOAND_W,
                `RV32_AMOOR_W, `RV32_AMOMIN_W, `RV32_AMOMAX_W,
                `RV32_AMOMINU_W, `RV32_AMOMAXU_W: begin
                    has_dest   = `TRUE;       // rd = old memory value
                    rd_mem     = `TRUE;
                    wr_mem     = `TRUE;
                    atomic     = `TRUE;
                    fu_type    = FU_STORE;
                end

                // ---- F: memory ----
                `RV32_FLW: begin
                    has_dest   = `TRUE;
                    rd_fp      = `TRUE;
                    opb_select = OPB_IS_I_IMM;
                    rs2_used   = `FALSE;
                    rd_mem     = `TRUE;
                    fu_type    = FU_LOAD;
                end
                `RV32_FSW: begin
                    rs2_fp     = `TRUE;
                    opb_select = OPB_IS_S_IMM;
                    wr_mem     = `TRUE;
                    fu_type    = FU_STORE;
                end

                // ---- F: FP sources, FP result ----
                `RV32_FADD_S, `RV32_FSUB_S, `RV32_FMUL_S, `RV32_FDIV_S: begin
                    has_dest = `TRUE;
                    {rs1_fp, rs2_fp, rd_fp} = 3'b111;
                    fu_type  = FU_FP;
                    illegal  = bad_rm;
                end
                `RV32_FSQRT_S: begin
                    has_dest = `TRUE;
                    {rs1_fp, rd_fp} = 2'b11;
                    fu_type  = FU_FP;
                    rs2_used = `FALSE;
                    illegal  = bad_rm;
                end
                `RV32_FSGNJ_S, `RV32_FSGNJN_S, `RV32_FSGNJX_S,
                `RV32_FMIN_S, `RV32_FMAX_S: begin
                    has_dest = `TRUE;
                    {rs1_fp, rs2_fp, rd_fp} = 3'b111;
                    fu_type  = FU_FP;
                end
                `RV32_FMADD_S, `RV32_FMSUB_S, `RV32_FNMSUB_S, `RV32_FNMADD_S: begin
                    has_dest = `TRUE;
                    {rs1_fp, rs2_fp, rd_fp} = 3'b111;
                    rs3_used = `TRUE;
                    fu_type  = FU_FP;
                    illegal  = bad_rm;
                end

                // ---- F: FP sources, integer result ----
                `RV32_FEQ_S, `RV32_FLT_S, `RV32_FLE_S: begin
                    has_dest = `TRUE;
                    {rs1_fp, rs2_fp} = 2'b11;
                    fu_type  = FU_FP;
                end
                `RV32_FCLASS_S, `RV32_FMV_X_W: begin
                    has_dest = `TRUE;
                    rs1_fp   = `TRUE;
                    fu_type  = FU_FP;
                    rs2_used = `FALSE;
                end
                `RV32_FCVT_W_S, `RV32_FCVT_WU_S: begin
                    has_dest = `TRUE;
                    rs1_fp   = `TRUE;
                    fu_type  = FU_FP;
                    rs2_used = `FALSE;
                    illegal  = bad_rm;
                end

                // ---- F: integer source, FP result ----
                `RV32_FMV_W_X: begin
                    has_dest = `TRUE;
                    rd_fp    = `TRUE;
                    fu_type  = FU_FP;
                    rs2_used = `FALSE;
                end
                `RV32_FCVT_S_W, `RV32_FCVT_S_WU: begin
                    has_dest = `TRUE;
                    rd_fp    = `TRUE;
                    fu_type  = FU_FP;
                    rs2_used = `FALSE;
                    illegal  = bad_rm;
                end

                default: begin
                    illegal = `TRUE;
                end
        endcase // casez (inst)
        end // if (valid)
    end // always

endmodule // decoder
