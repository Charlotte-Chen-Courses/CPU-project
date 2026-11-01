# 2-Way Out-of-Order RISC-V Core with SMT

A solo, synthesizable **2-way superscalar out-of-order RV32IMAF core** with
**R10K-style renaming** and **2-thread simultaneous multithreading**, written in
SystemVerilog and simulated with Verilator.

| | |
| --- | --- |
| ISA | RV32IMAF (integer, multiply/divide, atomics, single-precision FP) |
| Width | 2-wide fetch / rename / dispatch / commit, 2 CDBs |
| Threads | 2 (SMT, ICOUNT fetch) |
| Synthesis | ASAP7 7nm, Design Compiler: 1.53 ns period (~660 MHz), 0.05 mm² |
| Energy (pre-layout est.) | ~70 mW, ~150 pJ/instruction |

> **Branches:** the full core described here, including SMT and the tournament
> predictor, is on `smt`. `main` holds the earlier single-thread core.

## Microarchitecture

**Rename (R10K).** A physical register file with a map table (RAT), free list,
busy table and retirement RAT. Integer and FP registers are separate register
classes, each with its own RAT, free list, PRF, busy table and retirement RAT.
CDB packets, RS source tags and ROB entries carry a register-class bit. A
mispredict is recovered when the branch commits: the map table is restored from
the retirement RAT.

| Structure | Size | Shared or per thread |
| --- | --- | --- |
| Reorder buffer | 32 entries | per thread |
| Reservation station | 16 entries (thread-tagged) | shared |
| Physical registers | 64 int + 64 FP | per thread (own tag range) |
| Load / store queue | 8 + 8 entries | per thread |
| Common data buses | 2 | shared |
| Functional units | 2 ALU, branch, mult, div, load, store, FP | shared |
| I-cache | 4 KB, ASAP7 SRAM macro, 4-deep stream prefetcher | shared (thread-hashed) |
| D-cache | 4 KB, 2-way, non-blocking, 4 MSHRs × 4 targets | shared (thread-hashed) |
| Branch prediction | 32-entry BTB, tournament (bimodal + gshare + chooser), 8-entry RAS | tables shared, history and RAS per thread |

**Memory.** The LSQ issues loads out of order, forwards from stores to loads,
and replays or squashes loads on a disambiguation failure. An Alpha
21264-style load-wait table makes a load that was replayed before wait for older
store addresses, and clears periodically so stale predictions expire. The non-blocking
D-cache merges requests in its MSHRs and reserves victim ways at allocation.

**Floating point.** A single FP unit wraps [CVFPU](https://github.com/openhwgroup/cvfpu)
(FP32 only) with the core's issue and CDB handshake. FP results share the two
CDBs with integer results. `fcsr` rounding mode is supported, and `fflags`
accumulate at commit.

**Atomics.** LR/SC and all AMOs execute non-speculatively at the head of the
ROB and LSQ. A reservation is cleared by any SC and by any store to the
reserved word.

**SMT.** All architectural and circular state is duplicated per thread: ROB,
LSQ, RATs, free lists, busy tables, RAS, PC and `fcsr`. The RS, functional
units, CDBs, PRFs, caches and predictor tables are shared. A thread's flush
kills only its own work. Fetch uses **ICOUNT**. Each thread gets a private
base + bound memory region, and the top 4 KB of each region is a window shared
between threads for atomics workloads.

## Measured results

Every feature was kept or removed according to its measured gain on the
benchmark programs:

| Feature | Result |
| --- | --- |
| Non-blocking D-cache (4 MSHRs) | 1.29× IPC |
| I-cache stream prefetching | 1.15× |
| Tournament branch predictor | mispredicts 12.1% → 10.3% |
| Load-wait predictor | 4% over conservative load issue |
| ICOUNT fetch + thread-hashed indexing | SMT pair throughput 0.98× → 1.11× vs. running the two programs back to back |
| D-side tagged prefetcher | +0.7%: **removed**, since spending its 2 MSHRs on demand misses gained more (+1.9%) |

## Verification

- One self-checking testbench per module. `make test` runs all of them, the FP
  register-class and bimodal-predictor variants, and the full core at one and
  two threads.
- The whole core is checked commit by commit against a golden ISA model, which
  is itself checked against Spike. FP results are checked against Spike's
  SoftFloat through DPI-C.
- Random program pairs and directed tests on the 2-thread core, each thread
  checked against its own ISA model. Multi-thread atomics tests check
  invariants (counters, a lock) and a same-address load-order litmus test.
- A random-initialization run (`cpu_t2_xr`) catches logic that depends on
  state that reset does not set.
- Mutation testing confirms the tests catch injected bugs.

## Building and running

Requirements: [Verilator](https://www.veripool.org/verilator/) 5 (with
`--timing`) and [Spike](https://github.com/riscv-software-src/riscv-isa-sim),
whose SoftFloat library is the FP reference model.

```bash
git clone --recurse-submodules -b smt https://github.com/Charlotte-Chen-Courses/CPU-project.git
cd CPU-project

make test          # every module testbench and the full core
make tb_cpu        # just the single-thread core
make tb_cpu_t2     # the 2-thread core
make smt_pairs     # pairs of programs on the 2-thread core, each checked against its ISS
```

`CLOCK_PERIOD` defaults to the synthesized 1.53 ns, so main memory's 100 ns
latency is about 66 cycles. Use `make CLOCK_PERIOD=10 ...` for faster
simulations.

## Repository layout

```text
starter/verilog/   RTL (cpu.sv is the top)
starter/test/      module and core testbenches, golden model, DPI-C FP reference
starter/programs/  benchmark and test programs (C and assembly, incl. fp/ and atomic/)
starter/vendor/    CVFPU (submodule) + patches, ASAP7 SRAM simulation model
synth_local/       local Design Compiler flow (ASAP7)
Makefile           Verilator build, tests and SMT runs
```

## Credits

- Built on the CSEE 4824 (Columbia) processor-project starter code, including
  the VeriSimpleV in-order pipeline from an earlier course project.
- FP arithmetic: [CVFPU](https://github.com/openhwgroup/cvfpu) v0.8.1
  (OpenHW Group), with small patches in `starter/vendor/patches/`.
- I-cache SRAM: ASAP7 SRAM simulation model (BSD 3-Clause; Vashishtha et al.,
  ISCAS 2017).
