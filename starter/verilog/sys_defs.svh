/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  sys_defs.svh                                        //
//                                                                     //
//  Description :  This file has the macro-defines for macros used in  //
//                 the pipeline design.                                //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`ifndef __SYS_DEFS_SVH__
`define __SYS_DEFS_SVH__

// all files should `include "sys_defs.svh" to at least define the timescale
`timescale 1ns / 100ps

///////////////////////////////////
// ---- Starting Parameters ---- //
///////////////////////////////////

// some starting parameters that you should set
// this is *your* processor, you decide these values (try analyzing which is best!)

// superscalar width
`define SUPERSCALAR_WIDTH 2
// max instructions issued per cycle (RS -> PRF read); PRF has 2 read ports per issue slot
`define ISSUE_WIDTH `SUPERSCALAR_WIDTH

// sizes
`define NUM_CDB 2

`define RS_SZ 16
`define ARCH_REG_SZ 32
`define ROB_SZ 32
`define PHYS_REG_SZ (`ARCH_REG_SZ + `ROB_SZ)          // integer physical registers: tags 0..
`define FREE_LIST_SZ (`PHYS_REG_SZ - `ARCH_REG_SZ)
// FP physical registers are a separate class with their own RAT/freelist/PRF/busy table; their
// tags follow the integer ones (FP_TAG_BASE..), so one tag names a register of either class
`define FP_PHYS_REG_SZ (`ARCH_REG_SZ + `ROB_SZ)
`define FP_FREE_LIST_SZ (`FP_PHYS_REG_SZ - `ARCH_REG_SZ)
`define FP_TAG_BASE `PHYS_REG_SZ
`define TAG_W $clog2(`PHYS_REG_SZ + `FP_PHYS_REG_SZ)
`define ROB_IDX_W $clog2(`ROB_SZ)
`define RS_IDX_W $clog2(`RS_SZ)


// worry about these later
// branch predictor (fetch): BTB entries and bimodal 2-bit counters (powers of 2)
`define BTB_SZ 32
`define BHT_SZ 256
`define RAS_SZ 8      // return address stack depth (power of 2)
// load / store queues (powers of 2)
`define LQ_SZ 8
`define SQ_SZ 8
`define LSQ_IDX_W ($clog2(`LQ_SZ) > $clog2(`SQ_SZ) ? $clog2(`LQ_SZ) : $clog2(`SQ_SZ))
`define DMEM_ID_W 8      // data-memory request id (a load's sequence number)
// 0: conservative (a load waits for every older store address)
// 1: speculate: a load may go to memory before older store addresses are known (a store that
//    later turns out to overlap replays it)
// 2: speculate, except loads the load-wait table predicts will violate (they wait, as in 0)
`define LSQ_SPECULATE 2
// A extension (LR/SC/AMO, serialized, executed in the LSQ); 0: atomics stay unsupported
`define A_EXTENSION 1
// load-wait table (LSQ_SPECULATE = 2): one bit per entry, indexed by load PC, set when that load
// is replayed; all bits clear every LWT_CLEAR cycles so stale predictions expire (Alpha 21264)
`define LWT_SZ    64     // power of 2
`define LWT_CLEAR 16384
// instruction cache: direct-mapped, 8-byte lines. ICACHE_SRAM = 1 keeps the data in the ASAP7
// SRAM bank (synchronous read: needs ICACHE_LINES = 512, i.e. 4 KB); 0 keeps it in flip-flops
// (combinational read, any power-of-2 size, for sweeps)
`define ICACHE_LINES 512
`define ICACHE_SRAM  1
// data cache (sizes in bytes; powers of 2). Blocks match mem.sv's 64-bit bus.
`define DC_SZ   4096
`define DC_WAYS 2
`define DC_BLK  8
// non-blocking: MSHRs (0 = blocking, one miss at a time) and the requests each can hold.
// MSHRs * targets must stay well below 2^DMEM_ID_W (load ids are not reused while waiting).
`define DC_MSHRS 4
`define DC_TGTS  4

// functional units (you should decide if you want more or fewer types of FUs)

// Exception causes, raised only when the instruction reaches commit (a wrong-path one is
// squashed by the older mispredict's flush and never reported)
typedef enum logic [2:0] {
  EXC_NONE        = 3'h0,
  EXC_ILLEGAL     = 3'h1,   // decoder: not an RV32IMAF instruction
  EXC_UNSUPPORTED = 3'h2,   // legal, but this core has no unit for it yet (FP/CSR/A)
  EXC_MISALIGNED  = 3'h3,   // taken branch/jump target, or load/store address, not naturally aligned
  EXC_ACCESS      = 3'h4    // load/store address outside data memory (MEM_SIZE_IN_BYTES)
} EXCEPTION;

typedef enum logic [2:0] {
  FU_ALU,
  FU_MULT,
  FU_LOAD,
  FU_STORE,
  FU_FP,
  FU_BRANCH,  // cond branches, jal, jalr (appended: existing encodings unchanged)
  FU_DIV      // div, divu, rem, remu: iterative divider
} FU_TYPE;

`define NUM_FU_ALU 2
`define NUM_FU_BRANCH 1
`define NUM_FU_MULT 1
`define NUM_FU_DIV 1
`define NUM_FU_LOAD 1    // the LSQ has exactly one load port and one store port
`define NUM_FU_STORE 1
`define NUM_FU_FP 1     // 0 or 1 (one FP unit; the FP PRF has one rs3 read port)
// FP unit (CVFPU) pipeline registers per operation group (FP32); fdiv/fsqrt are iterative
`define FP_ADDMUL_REGS  3     // fadd/fsub/fmul/fmadd...: latency = regs + 1 (input buffer bypassed)
`define FP_NONCOMP_REGS 1     // sgnj, min/max, compares, fclass, fmv
`define FP_CONV_REGS    2     // fcvt
`define NUM_FU (`NUM_FU_ALU+`NUM_FU_BRANCH+`NUM_FU_MULT+`NUM_FU_DIV+`NUM_FU_LOAD+`NUM_FU_STORE+`NUM_FU_FP)

// Ports (RS issue ports = FU instances = CDB-arbiter inputs) are grouped by FU type in
// this order: ALU, BRANCH, MULT, DIV, LOAD, STORE, FP. Shared by rs.sv, cdb_arbiter.sv, cpu.sv.
function automatic FU_TYPE port_type(input int f);
  if (f < `NUM_FU_ALU) return FU_ALU;
  if (f < `NUM_FU_ALU + `NUM_FU_BRANCH) return FU_BRANCH;
  if (f < `NUM_FU_ALU + `NUM_FU_BRANCH + `NUM_FU_MULT) return FU_MULT;
  if (f < `NUM_FU_ALU + `NUM_FU_BRANCH + `NUM_FU_MULT + `NUM_FU_DIV) return FU_DIV;
  if (f < `NUM_FU_ALU + `NUM_FU_BRANCH + `NUM_FU_MULT + `NUM_FU_DIV + `NUM_FU_LOAD) return FU_LOAD;
  if (f < `NUM_FU - `NUM_FU_FP) return FU_STORE;
  return FU_FP;
endfunction

// Priority between FU types (RS issue when slots run out, CDB arbitration):
// longest latency first, then branches (resolve early), then ALU. Rank 0 is served first.
`define NUM_FU_RANKS 7
function automatic FU_TYPE rank_type(input int r);
  case (r)
    0:       return FU_DIV;
    1:       return FU_FP;
    2:       return FU_MULT;
    3:       return FU_LOAD;
    4:       return FU_STORE;
    5:       return FU_BRANCH;
    default: return FU_ALU;
  endcase
endfunction
// number of mult stages (2, 4, or 8)
`define MULT_STAGES 4

///////////////////////////////
// ---- Basic Constants ---- //
///////////////////////////////

// NOTE: the global CLOCK_PERIOD is defined in the Makefile

// useful boolean single-bit definitions
`define FALSE 1'h0
`define TRUE 1'h1

// data length
`define XLEN 32

// the zero register
// In RISC-V, any read of this register returns zero and any writes are thrown away
`define ZERO_REG 5'd0

// Basic NOP instruction. Allows pipline registers to clearly be reset with
// an instruction that does nothing instead of Zero which is really an ADDI x0, x0, 0
`define NOP 32'h00000013

//////////////////////////////////
// ---- Memory Definitions ---- //
//////////////////////////////////

// Cache mode removes the byte-level interface from memory, so it always returns
// a double word. The original processor won't work with this defined. Your new
// processor will have to account for this effect on mem.
// Notably, you can no longer write data without first reading.
`define CACHE_MODE

// you are not allowed to change this definition for your final processor
// the project 3 processor has a massive boost in performance just from having no mem latency
// see if you can beat it's CPI in project 4 even with a 100ns latency!
// `define MEM_LATENCY_IN_CYCLES  0
`define MEM_LATENCY_IN_CYCLES (100.0/`CLOCK_PERIOD+0.49999)
// the 0.49999 is to force ceiling(100/period). The default behavior for
// float to integer conversion is rounding to nearest

// How many memory requests can be waiting at once
`define NUM_MEM_TAGS 15

`define MEM_SIZE_IN_BYTES (64*1024)
`define MEM_64BIT_LINES (`MEM_SIZE_IN_BYTES/8)

typedef union packed {
  logic [7:0][7:0]  byte_level;
  logic [3:0][15:0] half_level;
  logic [1:0][31:0] word_level;
} EXAMPLE_CACHE_BLOCK;

typedef enum logic [1:0] {
  BYTE   = 2'h0,
  HALF   = 2'h1,
  WORD   = 2'h2,
  DOUBLE = 2'h3
} MEM_SIZE;

// Memory bus commands
typedef enum logic [1:0] {
  BUS_NONE  = 2'h0,
  BUS_LOAD  = 2'h1,
  BUS_STORE = 2'h2
} BUS_COMMAND;

///////////////////////////////
// ---- Exception Codes ---- //
///////////////////////////////

/**
 * Exception codes for when something goes wrong in the processor.
 * Note that we use HALTED_ON_WFI to signify the end of computation.
 * It's original meaning is to 'Wait For an Interrupt', but we generally
 * ignore interrupts in 470
 *
 * This mostly follows the RISC-V Privileged spec
 * except a few add-ons for our infrastructure
 * The majority of them won't be used, but it's good to know what they are
 */

typedef enum logic [3:0] {
  INST_ADDR_MISALIGN  = 4'h0,
  INST_ACCESS_FAULT   = 4'h1,
  ILLEGAL_INST        = 4'h2,
  BREAKPOINT          = 4'h3,
  LOAD_ADDR_MISALIGN  = 4'h4,
  LOAD_ACCESS_FAULT   = 4'h5,
  STORE_ADDR_MISALIGN = 4'h6,
  STORE_ACCESS_FAULT  = 4'h7,
  ECALL_U_MODE        = 4'h8,
  ECALL_S_MODE        = 4'h9,
  NO_ERROR            = 4'ha,  // a reserved code that we use to signal no errors
  ECALL_M_MODE        = 4'hb,
  INST_PAGE_FAULT     = 4'hc,
  LOAD_PAGE_FAULT     = 4'hd,
  HALTED_ON_WFI       = 4'he,  // 'Wait For Interrupt'. In 470, signifies the end of computation
  STORE_PAGE_FAULT    = 4'hf
} EXCEPTION_CODE;

///////////////////////////////////
// ---- Instruction Typedef ---- //
///////////////////////////////////

// from the RISC-V ISA spec
typedef union packed {
  logic [31:0] inst;
  struct packed {
    logic [6:0] funct7;
    logic [4:0] rs2;  // source register 2
    logic [4:0] rs1;  // source register 1
    logic [2:0] funct3;
    logic [4:0] rd;  // destination register
    logic [6:0] opcode;
  } r;  // register-to-register instructions
  struct packed {
    logic [11:0] imm;  // immediate value for calculating address
    logic [4:0] rs1;  // source register 1 (used as address base)
    logic [2:0] funct3;
    logic [4:0] rd;  // destination register
    logic [6:0] opcode;
  } i;  // immediate or load instructions
  struct packed {
    logic [6:0] off;  // offset[11:5] for calculating address
    logic [4:0] rs2;  // source register 2
    logic [4:0] rs1;  // source register 1 (used as address base)
    logic [2:0] funct3;
    logic [4:0] set;  // offset[4:0] for calculating address
    logic [6:0] opcode;
  } s;  // store instructions
  struct packed {
    logic       of;      // offset[12]
    logic [5:0] s;       // offset[10:5]
    logic [4:0] rs2;     // source register 2
    logic [4:0] rs1;     // source register 1
    logic [2:0] funct3;
    logic [3:0] et;      // offset[4:1]
    logic       f;       // offset[11]
    logic [6:0] opcode;
  } b;  // branch instructions
  struct packed {
    logic [19:0] imm;  // immediate value
    logic [4:0] rd;  // destination register
    logic [6:0] opcode;
  } u;  // upper-immediate instructions
  struct packed {
    logic       of;      // offset[20]
    logic [9:0] et;      // offset[10:1]
    logic       s;       // offset[11]
    logic [7:0] f;       // offset[19:12]
    logic [4:0] rd;      // destination register
    logic [6:0] opcode;
  } j;  // jump instructions

  // extensions for other instruction types
`ifdef ATOMIC_EXT
  struct packed {
    logic [4:0] funct5;
    logic       aq;
    logic       rl;
    logic [4:0] rs2;
    logic [4:0] rs1;
    logic [2:0] funct3;
    logic [4:0] rd;
    logic [6:0] opcode;
  } a;  // atomic instructions
`endif
`ifdef SYSTEM_EXT
  struct packed {
    logic [11:0] csr;
    logic [4:0]  rs1;
    logic [2:0]  funct3;
    logic [4:0]  rd;
    logic [6:0]  opcode;
  } sys;  // system call instructions
`endif

} INST;  // instruction typedef, this should cover all types of instructions

////////////////////////////////////////
// ---- Datapath Control Signals ---- //
////////////////////////////////////////

// ALU opA input mux selects
typedef enum logic [1:0] {
  OPA_IS_RS1  = 2'h0,
  OPA_IS_NPC  = 2'h1,
  OPA_IS_PC   = 2'h2,
  OPA_IS_ZERO = 2'h3
} ALU_OPA_SELECT;

// ALU opB input mux selects
typedef enum logic [3:0] {
  OPB_IS_RS2   = 4'h0,
  OPB_IS_I_IMM = 4'h1,
  OPB_IS_S_IMM = 4'h2,
  OPB_IS_B_IMM = 4'h3,
  OPB_IS_U_IMM = 4'h4,
  OPB_IS_J_IMM = 4'h5
} ALU_OPB_SELECT;

// ALU function code input
// probably want to leave these alone
typedef enum logic [4:0] {
  ALU_ADD    = 5'h00,
  ALU_SUB    = 5'h01,
  ALU_SLT    = 5'h02,
  ALU_SLTU   = 5'h03,
  ALU_AND    = 5'h04,
  ALU_OR     = 5'h05,
  ALU_XOR    = 5'h06,
  ALU_SLL    = 5'h07,
  ALU_SRL    = 5'h08,
  ALU_SRA    = 5'h09,
  ALU_MUL    = 5'h0a,  // Mult FU
  ALU_MULH   = 5'h0b,  // Mult FU
  ALU_MULHSU = 5'h0c,  // Mult FU
  ALU_MULHU  = 5'h0d,  // Mult FU
  ALU_DIV    = 5'h0e,  // Div FU
  ALU_DIVU   = 5'h0f,  // Div FU
  ALU_REM    = 5'h10,  // Div FU
  ALU_REMU   = 5'h11   // Div FU
} ALU_FUNC;

////////////////////////////////
// ---- Datapath Packets ---- //
////////////////////////////////

/**
 * Packets are used to move many variables between modules with
 * just one datatype, but can be cumbersome in some circumstances.
 *
 * Define new ones in project 4 at your own discretion
 */

/**
 * IF_ID Packet:
 * Data exchanged from the IF to the ID stage
 */
typedef struct packed {
  INST              inst;
  logic [`XLEN-1:0] PC;
  logic [`XLEN-1:0] NPC;    // PC + 4
  logic             valid;
} IF_ID_PACKET;

/**
 * ID_EX Packet:
 * Data exchanged from the ID to the EX stage
 */
typedef struct packed {
  INST              inst;
  logic [`XLEN-1:0] PC;
  logic [`XLEN-1:0] NPC;   // PC + 4

  logic [`XLEN-1:0] rs1_value;  // reg A value
  logic [`XLEN-1:0] rs2_value;  // reg B value

  ALU_OPA_SELECT opa_select;  // ALU opa mux select (ALU_OPA_xxx *)
  ALU_OPB_SELECT opb_select;  // ALU opb mux select (ALU_OPB_xxx *)

  logic [4:0] dest_reg_idx;   // destination (writeback) register index
  ALU_FUNC    alu_func;       // ALU function select (ALU_xxx *)
  logic       rd_mem;         // Does inst read memory?
  logic       wr_mem;         // Does inst write memory?
  logic       cond_branch;    // Is inst a conditional branch?
  logic       uncond_branch;  // Is inst an unconditional branch?
  logic       halt;           // Is this a halt?
  logic       illegal;        // Is this instruction illegal?
  logic       csr_op;         // Is this a CSR operation? (we use this to get return code)

  logic valid;
} ID_EX_PACKET;

/**
 * EX_MEM Packet:
 * Data exchanged from the EX to the MEM stage
 */
typedef struct packed {
  logic [`XLEN-1:0] alu_result;
  logic [`XLEN-1:0] NPC;

  logic             take_branch;   // Is this a taken branch?
  // Pass-through from decode stage
  logic [`XLEN-1:0] rs2_value;
  logic             rd_mem;
  logic             wr_mem;
  logic [4:0]       dest_reg_idx;
  logic             halt;
  logic             illegal;
  logic             csr_op;
  logic             rd_unsigned;   // Whether proc2Dmem_data is signed or unsigned
  MEM_SIZE          mem_size;
  logic             valid;
} EX_MEM_PACKET;

/**
 * MEM_WB Packet:
 * Data exchanged from the MEM to the WB stage
 *
 * Does not include data sent from the MEM stage to memory
 */
typedef struct packed {
  logic [`XLEN-1:0] result;
  logic [`XLEN-1:0] NPC;
  logic [4:0]       dest_reg_idx;  // writeback destination (ZERO_REG if no writeback)
  logic             take_branch;
  logic             halt;          // not used by wb stage
  logic             illegal;       // not used by wb stage
  logic             valid;
} MEM_WB_PACKET;

/**
 * No WB output packet as it would be more cumbersome than useful
 */


/**
 * ROB Packet:
 * 
 */
typedef struct packed {
  logic              done;        // set by CDB later; 0 at dispatch
  logic              dest_valid;
  logic [4:0]        rd;
  logic [`TAG_W-1:0] tag;
  logic [`TAG_W-1:0] told;
  logic [`XLEN-1:0]  pc;          // commit trace / mispredict redirect
  logic              halt;        // WFI: stop at commit
  logic [`XLEN-1:0]  pred_npc;    // next PC fetch predicted (set at dispatch)
  logic [`XLEN-1:0]  next_pc;     // actual next PC: PC+4 at dispatch, branches overwrite at complete
  EXCEPTION          exception;   // reported at commit
  logic              is_br;       // conditional branch, jal or jalr: trains the predictor at commit
  logic              uncond;      // jal / jalr
  logic              is_call;     // jal/jalr with rd = x1/x5 (RAS push)
  logic              is_ret;      // jalr x0, x1/x5 (RAS pop)
  logic              is_load;     // commit frees its LQ entry
  logic              is_store;    // commit lets its SQ entry drain to memory
  logic              replay;      // load that read memory too early: re-fetch it at commit
  logic              dest_fp;     // rd is an f register (tag/told are FP pregs)
  logic [4:0]        fflags;      // FP exception flags from the CDB
} ROB_PACKET;


/**
 * RS Packet:
 * 
 */
typedef struct packed {
  INST              inst;
  logic [`XLEN-1:0] PC;
  logic [`XLEN-1:0] NPC;   // PC + 4

  logic [`TAG_W-1:0]     src1_tag;
  logic                  src1_ready;
  logic [`TAG_W-1:0]     src2_tag;
  logic                  src2_ready;
  logic [`TAG_W-1:0]     src3_tag;     // FP fused multiply-add only (else ready)
  logic                  src3_ready;
  logic                  serial;       // issue only as the oldest instruction (ROB head)
  logic [`TAG_W-1:0]     dest_tag;
  logic [`ROB_IDX_W-1:0] rob_idx;
  FU_TYPE                fu_type;


  ALU_FUNC alu_func;  // ALU function select (ALU_xxx *)
  ALU_OPA_SELECT opa_select;  // ALU opa mux select (ALU_OPA_xxx *)
  ALU_OPB_SELECT opb_select;  // ALU opb mux select (ALU_OPB_xxx *)
  logic [`LSQ_IDX_W-1:0] lsq_idx;  // loads: LQ entry; stores: SQ entry

} RS_PACKET;


/**
 * CDB Packet:
 * 
 */
typedef struct packed {
  logic [`TAG_W-1:0]     tag;      // dest_tag (p0 = no destination)
  logic [`XLEN-1:0]      value;
  logic [`ROB_IDX_W-1:0] rob_idx;
  logic                  branch;   // result of a branch/jump: next_pc is valid
  logic [`XLEN-1:0]      next_pc;  // actual next PC (branches only)
  EXCEPTION              exception;  // raised by this instruction (misaligned target/address,
                                     // address out of range); the ROB records it
  logic [4:0]            fflags;     // FP exception flags (NV DZ OF UF NX), into fcsr at commit
} CDB_PACKET;




`endif  // __SYS_DEFS_SVH__
