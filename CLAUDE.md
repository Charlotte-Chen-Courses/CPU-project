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

- **Duration:** ~8 weeks, full-time, no other commitments

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
  with the LSQ for a memory-level-parallelism analysis story.
- **SMT (2-thread) — TERMINAL FEATURE.** See the SMT section below. It is
  deliberately scoped as cuttable.

### Parameterize from day one
Keep these as compile-time parameters, held at 1 until the scalar single-thread
core fully commits programs:
- `SUPERSCALAR_WIDTH` (start 1)
- `NUM_THREADS` (start 1)

Also parameterize: PRF size, ROB depth, IQ depth, LSQ depth, freelist size. These
become the axes of the design-space exploration in Phase 5.

**Critical:** never build width or SMT into untested logic. Retrofitting width
onto a scalar design is the classic way these projects collapse near the
deadline. Structures (map table, freelist, CDB arbitration) are designed N-wide
immediately, even while tested at N=1.

---

## ISA scope

- **Target: RV32IM.** Base integer + multiply/divide.
- **M is included deliberately:** multiply is the canonical variable-latency
  functional unit that makes out-of-order execution interesting (exercises
  wakeup/select and CDB arbitration). Use the provided pipelined multiplier.
- **Excluded, by deliberate scoping judgment** (being able to articulate WHY is
  itself an interview signal):
  - **F/D (floating point):** adds datapath + a second register file + FP
    exception work, but doesn't deepen the OoO/LSQ/SMT story. Skipped.
  - **C (compressed):** pure fetch/decode complexity in the stage SMT already
    stresses most. Skipped.
  - **Privileged/CSR/interrupts:** implement only enough exception plumbing for
    the test harness + branch-misprediction recovery. No real M-mode / traps.
- **If exactly one extension beyond M is ever added: A (atomics)** — and only
  because SMT gives it a real target (two threads sharing L1). Narrow scope:
  LR/SC + amoadd/amoswap. This is terminal-of-terminal; only if SMT AND LSQ are
  both solid.
- **Before finalizing:** confirm what ISA subset the provided benchmark binaries
  were compiled for (toolchain flags). IM is expected to match exactly.

---

## CDB constraint (do not violate)

Number of CDBs ≤ superscalar width. At 2-way, at most 2 CDBs. This is a hard
architectural rule of the design.

---

## Roadmap (8 weeks) — with always-shippable checkpoints

The governing principle: **working scalar core first, then widen, then memory,
then SMT.** Each checkpoint is a complete, presentable artifact. There are three
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

### Phase 4 — SMT overlay (Week 7) — TERMINAL, CUTTABLE
- Only start if Phase 3 is genuinely stable.
- Thread-tag / duplicate architectural state: 2× RAT, 2× RAS, per-thread PC, ROB
  thread-ID + per-thread commit, per-thread isolated recovery.
- Build a real fetch policy (**ICOUNT**, not round-robin) — where SMT's
  performance and the interview story live.
- Construct a **multiprogrammed workload** (two single-thread programs
  interleaved) — without it, SMT's benefit literally cannot be demonstrated,
  since the benchmarks are single-threaded.
- **Checkpoint:** two threads concurrent with isolated recovery;
  aggregate throughput measurable. **If buggy here, CUT IT and revert to the
  Phase 3 core — this is the pre-planned escape hatch.**

### Phase 5 — Analysis, synthesis close, write-up (Week 8)
This week is where the portfolio value is created; protect it last, never
sacrifice it.
- Design-space sweeps with plots: CPI vs ROB/IQ/LSQ depth, predictor table size,
  cache associativity/MSHR count — across multiple benchmarks.
- SMT (if kept): aggregate throughput gain + per-thread slowdown vs single-thread.
  Report honestly, including negatives.
- Synthesis: report clock period, find + discuss critical path (likely
  wakeup/select or CDB), discuss how you'd pipeline / speculatively schedule to
  break it.
- Write report + clean README + architecture diagram for the website.

### Risk rules
- Most likely schedule-killer: Phase 1 or Phase 3 debug overrunning.
- If either slips: **cut SMT first, then the non-blocking cache. Protect the
  analysis week last.**
- There is almost no slack. If a safer project is wanted, drop SMT now and make
  the single-thread core excellent and deeply analyzed instead.

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
