# Third-party code under vendor/

| Directory | Source | Version | License |
|---|---|---|---|
| cvfpu (git submodule) | github.com/openhwgroup/cvfpu | v0.8.1 (79e4531) | Solderpad SHL-0.51 (vendor/opene906: Apache-2.0) |
| cvfpu/src/common_cells (its submodule) | github.com/pulp-platform/common_cells | v1.21.0 (6aeee85) | Solderpad SHL-0.51 |
| cvfpu/src/fpu_div_sqrt_mvp (its submodule) | github.com/pulp-platform/fpu_div_sqrt_mvp | v1.0.4 (86e1f55) | Solderpad SHL-0.51 |

After cloning this repository: `git submodule update --init --recursive`.

Only the files listed in VENDOR_FPU (root Makefile) are built. The submodule is never edited:
each patched file is built from `patches/<file>.patch` into `build/vendor_patched/` by the
Makefile, and the build uses that copy instead of the original.

## Local patches

1. `cvfpu/src/fpnew_fma.sv`, `cvfpu/src/fpnew_fma_multi.sv`: underflow-after-rounding detection
   used `rnd_mode_i` (the rounding mode of the op entering the pipeline) instead of `rnd_mode_q`
   (the op being rounded). With pipeline registers this set UF wrongly on results that round to
   the smallest normal. Backport of upstream commit a6af6915 (develop, 2024-08-02, PR #137).
   Found by fp_fu_test against SoftFloat.
2. `cvfpu/src/fpnew_divsqrt_th_32.sv`: the T-Head div/sqrt wrapper's input handshake.
   - In HOLD (a finished result waiting for `out_ready`), the input `ready` defaulted to
     `unit_ready_q` (1), so an operation offered then completed its handshake but never started:
     it was lost. Now `ready` is 0 in HOLD unless a new operation starts.
   - When a new operation started from HOLD or from BUSY-and-done, `ready` ignored
     `fdsu_fpu_ex1_stall` (the extra EX1 cycle for subnormal operands). The handshake completed
     at once, so the next operation was accepted in the stall cycle and lost, and the unit
     re-read its operands in that cycle and computed the stalled operation with the next one's
     operands (wrong fdiv/fsqrt results). Now these starts complete their handshake a cycle
     later, as starts from IDLE already did.
   No upstream fix as of develop 2024-09. Found by fp_fu_test against SoftFloat (lost ops only
   when the CDB refuses a result, wrong values only with subnormal operands).
