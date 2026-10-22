// fp_ref.c: reference model for RV32F instructions, for testbenches (DPI-C).
// Uses Spike's SoftFloat (RISC-V specialization: canonical NaN 0x7fc00000, tininess after
// rounding, RISC-V fmin/fmax and fclass, saturating fcvt.w[u].s). Decodes the instruction word
// itself, independently of the RTL. Returns the result bits; *flags = NV DZ OF UF NX (as fcsr).
// (Verilator builds it as C++: keep C linkage for DPI and for SoftFloat.)
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
#include "softfloat.h"

static float32_t f(uint32_t v) { float32_t r; r.v = v; return r; }

uint32_t fp_ref(uint32_t inst, uint32_t a, uint32_t b, uint32_t c, uint32_t frm, uint32_t *flags) {
    uint32_t opcode = inst & 0x7f, funct3 = (inst >> 12) & 7, funct5 = inst >> 27, rs2 = (inst >> 20) & 0x1f;
    uint32_t rm = (funct3 == 7) ? frm : funct3, r = 0;
    softfloat_roundingMode   = rm;
    softfloat_exceptionFlags = 0;
    switch (opcode) {
    case 0x43: r = f32_mulAdd(f(a), f(b), f(c)).v; break;                                  // fmadd
    case 0x47: r = f32_mulAdd(f(a), f(b), f(c ^ 0x80000000u)).v; break;                    // fmsub
    case 0x4b: r = f32_mulAdd(f(a ^ 0x80000000u), f(b), f(c)).v; break;                    // fnmsub
    case 0x4f: r = f32_mulAdd(f(a ^ 0x80000000u), f(b), f(c ^ 0x80000000u)).v; break;      // fnmadd
    case 0x53:
        switch (funct5) {
        case 0x00: r = f32_add(f(a), f(b)).v; break;
        case 0x01: r = f32_sub(f(a), f(b)).v; break;
        case 0x02: r = f32_mul(f(a), f(b)).v; break;
        case 0x03: r = f32_div(f(a), f(b)).v; break;
        case 0x0b: r = f32_sqrt(f(a)).v; break;
        case 0x04:                                                                         // fsgnj / n / x
            r = (a & 0x7fffffffu) | ((funct3 == 0) ? (b & 0x80000000u) :
                                     (funct3 == 1) ? (~b & 0x80000000u) : ((a ^ b) & 0x80000000u));
            break;
        case 0x05: r = (funct3 == 0) ? f32_min(f(a), f(b)).v : f32_max(f(a), f(b)).v; break;
        case 0x14: r = (funct3 == 2) ? f32_eq(f(a), f(b)) : (funct3 == 1) ? f32_lt(f(a), f(b)) : f32_le(f(a), f(b)); break;
        case 0x18: r = rs2 ? (uint32_t)f32_to_ui32(f(a), rm, 1) : (uint32_t)f32_to_i32(f(a), rm, 1); break;
        case 0x1a: r = rs2 ? ui32_to_f32(a).v : i32_to_f32((int32_t)a).v; break;
        case 0x1c: r = (funct3 == 1) ? f32_classify(f(a)) : a; break;                    // fclass / fmv.x.w
        case 0x1e: r = a; break;                                                           // fmv.w.x
        }
        break;
    }
    *flags = softfloat_exceptionFlags & 0x1f;
    return r;
}
#ifdef __cplusplus
}
#endif
