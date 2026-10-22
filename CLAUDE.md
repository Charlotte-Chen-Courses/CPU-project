# CLAUDE.md — Out-of-Order Superscalar RISC-V Core

This file is persistent context for Claude Code. It captures the scope, design
decisions, roadmap, and conventions for this project. Read it fully before
acting. When a decision here conflicts with an ad-hoc request, surface the
conflict rather than silently overriding the plan.

---

## What this project is

A **solo, from-scratch out-of-order superscalar RISC-V core**, built as a side
project / portfolio piece (intended for a personal website and PhD interviews in
computer architecture). It builds on the VeriSimpleV RISC-V pipeline (from a
prior course project, "P3").

**This is not a class submission.** The optimization target is *demonstrated
microarchitectural depth per unit of finished, working, defensible work* — not a
grading rubric. A clean, measured, well-analyzed core beats a large broken one.
The analysis IS the deliverable.

- **Duration:** ~10 weeks, full-time, no other commitments (extended from 8 on
  2026-10-11 to add the F and A extensions)

---

## Target design

- **Style: R10K** (physical register file + RAT/map table + freelist), NOT P6.
  Chosen because it's the modern model and transfers directly to reasoning about
  real cores (Alpha 21264, modern x86/ARM).
- **2-way superscalar.** Parameterized for width from day one. Do NOT push to
  3-way — the 1→2 jump teaches the concepts; 2→3 is mostly pain for little new
  insight.
- **Load-Store Queue (LSQ)** with out-of-order loads + store-to-load forwarding.
  This is the centerpiece — memory disambiguation is the richest topic here and
  the one interviewers probe hardest. Includes load replay/squash on
  misspeculation.
- **Branch: BTB + tournament predictor + RAS.** (Start with a bimodal predictor
  for first-correct, then upgrade.) Measurement of prediction accuracy matters
  more than predictor sophistication.
- **Non-blocking D-cache with MSHRs** — added only once the core is solid; pairs
  with the LSQ for a memory-level-parallelism analysis story. **Implemented
  2026-10-22:** `DC_MSHRS` (default 4; 0 = blocking, the cut-order fallback) x
  `DC_TGTS` merged requests each. Merged requests replay in acceptance order
  (a load sees exactly the requests accepted before it). Victim ways are
  reserved at allocation, and a dirty victim's writeback is accepted before
  anything new.
- **F extension: one floating-point functional unit** — a second register class
  for rename and a second long/variable-latency FU class. See ISA scope.
- **A extension (atomics)** — LR/SC + AMOs, executed through the LSQ.
- **SMT (2-thread) — TERMINAL FEATURE.** See the SMT section below. It is
  deliberately scoped as cuttable.

### Parameterize from day one
Keep these as compile-time parameters, held at 1 until the scalar single-thread
core fully commits programs:
- `SUPERSCALAR_WIDTH` (start 1)
- `NUM_THREADS` (start 1)

Also parameterize: PRF size, ROB depth, IQ depth, LSQ depth, freelist size. These
become the axes of the design-space exploration in Phase 6.

**Critical:** never build width or SMT into untested logic. Retrofitting width
onto a scalar design is the classic way these projects collapse near the
deadline. Structures (map table, freelist, CDB arbitration) are designed N-wide
immediately, even while tested at N=1.

---

## ISA scope

- **Target: RV32IMAF.** Base integer + multiply/divide + atomics +
  single-precision floating point. (Decision changed 2026-10-11: F and A were
  previously excluded/terminal; the schedule was extended to 10 weeks for them.)
- **M is included deliberately:** multiply is the canonical variable-latency
  functional unit that makes out-of-order execution interesting (exercises
  wakeup/select and CDB arbitration). Use the provided pipelined multiplier.
- **F (single-precision FP), one FP functional unit:**
  - Rename a second register class. **Decided 2026-10-21: separate FP
    structures.** FP RAT (f0–f31; note f0 is a real register, unlike x0) + FP
    freelist + FP PRF + FP busy table + FP retirement RAT, as second instances
    of the existing int modules. CDB packets, RS source tags and ROB entries
    carry a register-class bit. Why: it reuses tested modules unchanged, and
    int vs FP register pressure stay separate axes for the analysis. Rejected:
    a unified PRF with a class bit, which threads the class through the
    freelist, busy table, every tag compare and recovery.
  - One FP FU: pipelined for add/sub/mul/fused multiply-add/convert/compare;
    fdiv/fsqrt may be iterative (unpipelined) inside the same unit. FP results
    share the same CDBs as integer results (CDB constraint below still holds).
    **Decided 2026-10-21: import CVFPU (openhwgroup/cvfpu, formerly
    pulp-platform/fpnew) for the arithmetic**, wrapped as our FU (issue/CDB
    handshake, fflags out). Why: it saves the FPU arithmetic work for the
    OoO integration. Own work to defend: the wrapper, rename and recovery,
    fcsr, and the measurements. Verified against Spike's SoftFloat
    (libsoftfloat.so ships with the Homebrew Spike) via DPI-C. Known cost:
    an iterative fdiv/fsqrt blocks the whole FP port; measure it.
  - flw/fsw go through the LSQ like integer loads/stores.
  - fcsr: support rounding mode + accumulate fflags at commit. No FP traps.
  - **D (double) is still excluded.**
  - Why it's worth it: a second rename class and a long-latency FU make the
    wakeup/select, CDB-arbitration and register-pressure analysis richer.
- **A (atomics):** LR/SC + AMOs (amoswap/amoadd/amoand/amoor/amoxor/amomin/
  amomax[u]). Execute non-speculatively at the head of the LSQ/ROB; LR sets a
  reservation that a conflicting store or SC clears. Single-thread it is mostly
  a correctness feature; with SMT it gets a real target (two threads sharing
  L1). **Implemented 2026-10-21:** atomics are serial (fetch ends the bundle,
  dispatch blocks until commit, issue only at the ROB head), so aq/rl are
  trivially satisfied and the LSQ's atomic engine writes memory at execute.
  Reservation rule (decided): cleared by any SC and by any store written to the
  reserved word (store, SC or AMO). Spike clears it only on SC, so Spike
  comparisons must avoid same-word stores between LR and SC.
  `A_EXTENSION 0` makes atomics unsupported (the cut-order fallback).
- **Excluded, by deliberate scoping judgment** (being able to articulate WHY is
  itself an interview signal):
  - **D (double-precision FP):** doubles FP datapath width for no new OoO
    insight beyond F.
  - **C (compressed):** pure fetch/decode complexity in the stage SMT already
    stresses most. Skipped.
  - **Privileged/CSR/interrupts:** implement only enough exception plumbing for
    the test harness + branch-misprediction recovery (+ fcsr for F). No real
    M-mode / traps.
- **Before finalizing:** confirm what ISA subset the provided benchmark binaries
  were compiled for (toolchain flags). They were expected to be IM; F and A need
  `-march=rv32imaf` builds and hand-written FP/atomic test programs, since the
  existing benchmarks likely contain no FP or atomic instructions.

---

## CDB constraint (do not violate)

Number of CDBs ≤ superscalar width. At 2-way, at most 2 CDBs. This is a hard
architectural rule of the design. The FP unit competes for the same CDBs; it
does not get its own.

---

## Roadmap (10 weeks) — with always-shippable checkpoints

The governing principle: **working scalar core first, then widen, then memory,
then F + A, then SMT.** Each checkpoint is a complete, presentable artifact. There are three
pre-planned fallback points so the project can never end with nothing.

### Phase 0 — Foundations (Week 1)
- Reuse from P3: decoder, memory module, basic I-cache, pipelined multiplier.
  Do NOT rewrite the decoder.
- Stand up build/sim/synth flow + per-module testbench harness now.
- **Get synthesis reporting a clock period in Week 1** — the single most common
  failure mode is treating synthesis as a final step.
- Freeze structure list + tag/CDB protocol on paper.
- **Checkpoint:** every module is a stub + testbench that compiles and
  synthesizes.

### Phase 1 — Scalar R10K core, in-order commit correct (Weeks 2–3)
- Week 2: rename (RAT + freelist + PRF), dispatch into ROB + IQ, single-issue
  wakeup/select, one ALU, one CDB, in-order commit.
- Week 3: branch handling + recovery (BTB + bimodal first, then map-table
  checkpoint/restore on mispredict). Split ALU into 1-cycle int / branch / AGU
  units; integrate multiplier.
- **★ Checkpoint: scalar OoO core that renames, executes OoO, recovers
  from mispredicts, commits in order. Legitimate standalone artifact.**

### Phase 2 — Superscalar widening to 2-way (Week 4)
- Flip `WIDTH=2`: 2-wide fetch/decode/rename, 2 CDBs, 2-wide commit,
  dual-ported/banked structures.
- Handle same-cycle RAW within a fetch bundle (younger reads older's dest). Test
  with dependent pairs deliberately.
- **★ Checkpoint: working 2-way core. Measure IPC vs scalar — first real
  data point.**

### Phase 3 — Memory subsystem: LSQ + D-cache (Weeks 5–6)
- Week 5: split D-cache (≤256 B), then LSQ — stores correct first, then OoO loads
  with store-to-load forwarding via address matching.
- Week 6: load speculation + replay/squash on disambiguation failure. Then, only
  if solid, non-blocking D-cache with MSHRs.
- **★ Checkpoint: full single-thread machine (OoO, 2-way, LSQ,
  non-blocking cache) passing bulk of test programs. This is the "complete
  project" fallback.**

### Phase 4 — F + A extensions (Weeks 7–8)
- Only start if Phase 3 is genuinely stable.
- Week 7 — F: decode FP ops (extend the P3 decoder, don't rewrite it); FP
  register class through rename (separate FP RAT/freelist/PRF — decision
  recorded in ISA scope); one FP FU (CVFPU, wrapped) on the shared CDBs; flw/fsw through the LSQ;
  fcsr rounding mode + fflags at commit. Hand-written FP tests + golden check
  against Spike.
- Week 8 — A: LR/SC + AMOs executed non-speculatively at the LSQ/ROB head;
  reservation set/clear logic. Directed tests (LR/SC success + failure, each
  AMO).
- **★ Checkpoint: RV32IMAF single-thread core passing integer, FP and atomic
  tests. This is the new "complete project" fallback; the Phase 3 core remains
  the fallback beneath it.**

### Phase 5 — SMT overlay (Week 9) — TERMINAL, CUTTABLE
- Only start if Phase 4 is genuinely stable.
- Thread-tag / duplicate architectural state: 2× RAT, 2× RAS, per-thread PC, ROB
  thread-ID + per-thread commit, per-thread isolated recovery.
- Build a real fetch policy (**ICOUNT**, not round-robin) — where SMT's
  performance and the interview story live.
- Construct a **multiprogrammed workload** (two single-thread programs
  interleaved) — without it, SMT's benefit literally cannot be demonstrated,
  since the benchmarks are single-threaded.
- **Checkpoint:** two threads concurrent with isolated recovery;
  aggregate throughput measurable. **If buggy here, CUT IT and revert to the
  Phase 4 core — this is the pre-planned escape hatch.**
- With A in place, SMT also gets a real atomics workload (two threads
  contending on shared data).

### Phase 6 — Analysis, synthesis close, write-up (Week 10)
This week is where the portfolio value is created; protect it last, never
sacrifice it.
- Design-space sweeps with plots: CPI vs ROB/IQ/LSQ depth, predictor table size,
  cache associativity/MSHR count — across multiple benchmarks.
- F: IPC on FP-heavy kernels, FP-unit utilization, CDB contention between FP and
  integer results.
- SMT (if kept): aggregate throughput gain + per-thread slowdown vs single-thread.
  Report honestly, including negatives.
- Synthesis: report clock period, find + discuss critical path (likely
  wakeup/select or CDB), discuss how you'd pipeline / speculatively schedule to
  break it.
- Write report + clean README + architecture diagram for the website.

### Risk rules
- Most likely schedule-killer: Phase 1 or Phase 3 debug overrunning.
- If anything slips: **cut SMT first, then A, then the non-blocking cache, then
  F. Protect the analysis week last.**
- There is almost no slack even at 10 weeks. If a safer project is wanted, drop
  SMT now and make the single-thread RV32IMAF core excellent and deeply analyzed
  instead.

---

## SMT: pros / cons (for design reasoning & the interview narrative)

**Pros:** higher functional-unit utilization by filling idle slots during one
thread's stalls; latency tolerance without more speculation; good
throughput-per-area (share FUs/PRF/caches, duplicate only architectural state).

**Cons:** state duplication + thread-tagging everywhere (roughly doubles the
hardest R10K bookkeeping); shared-resource contention can *hurt* single-thread
latency; fetch policy becomes a first-class problem (ICOUNT); **does nothing for
single-thread latency** — needs a multiprogrammed workload to show any win;
verification cost is disproportionate (interference bugs are non-local).

Even if SMT is ultimately cut, being able to say "I scoped it out deliberately,
and here's what it would have cost and bought" is nearly as strong an interview
signal as building it.

---

## What differentiates this from "a student built a CPU"

- Sweep parameters and plot results — design-space exploration IS what arch
  research is.
- Report honest negatives ("added a victim cache, bought 0.3%, here's the
  miss-rate data showing why").
- Instrument everything: IPC, per-benchmark branch accuracy, ROB occupancy
  histograms, LSQ forwarding rate, MSHR occupancy. Stub these counters in EARLY.
- Report synthesized clock period and discuss the critical path in depth (leverage
  RTL-to-GDSII background here — go deeper than a typical student).

---

## Working conventions

- Per-phase git branches; tag each shippable checkpoint.
- One testbench per module, each with an explicit pass/fail gate — "done" means a
  defined test passes. Don't advance past a gate silently.
- Keep a set of tiny hand-written assembly tests that deliberately exercise: OoO
  issue, same-cycle RAW, branch mispredict, load-after-store.
- Golden-reference check against a known-good ISA simulator (e.g. Spike) where
  available.
- Prefer correctness before cleverness in the LSQ (loads/stores correct before
  forwarding/speculation).
