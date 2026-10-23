# Local build flow (Verilator) for running module testbenches on this machine.
# The course Makefile in starter/ targets VCS/DC on the department servers.
#
#   make tb_prf      build + run starter/test/prf_test.sv against starter/verilog/prf.sv
#   make lint_prf    lint starter/verilog/prf.sv alone
#   make test        run every testbench listed in TESTS
#   make clean
#
# Adding a module test: write starter/test/<mod>_test.sv (top module "testbench",
# prints "@@@ Passed" on success), add <mod> to TESTS, and list any extra
# source files the module needs in DEPS_<mod>. A test that calls C over DPI lists
# its C files in CSRC_<mod> and its link flags in LDFLAGS_<mod>.

SRC   := starter
BUILD := build

TESTS := prf freelist rat rob rs busy_table alu_fu branch_fu mult_fu div_fu branch_pred ras cdb_arbiter decoder retire_rat dcache mem_arbiter lsq load_wait_table fp_fu cpu
# the rename structures are instantiated once per register class: <mod>_fp runs the same test
# against the FP instance (+define+FP_CLASS: tags offset by FP_TAG_BASE, f0 is a real register)
FP_VARIANTS := rat retire_rat freelist busy_table prf
TESTS += $(FP_VARIANTS:%=%_fp)
$(foreach v,$(FP_VARIANTS),$(eval BASE_$(v)_fp := $(v))$(eval VDEFS_$(v)_fp := +define+FP_CLASS))

# extra sources per module, e.g. DEPS_mult := $(SRC)/verilog/mult_stage.sv
DEPS_prf :=
DEPS_dcache  := $(SRC)/test/mem.sv
# the I-cache's data bank (ASAP7 SRAM simulation model; synthesis links the macro instead)
ICACHE_SRAM_MODEL := $(SRC)/vendor/asap7_sram/srambank_128x4x64_6t122.v
DEPS_mem_arbiter := $(SRC)/verilog/dcache.sv $(SRC)/verilog/icache.sv $(SRC)/verilog/iprefetch.sv $(ICACHE_SRAM_MODEL) $(SRC)/test/mem.sv
DEPS_lsq     := $(SRC)/test/tb_dmem.sv
DEPS_mult_fu := $(addprefix $(SRC)/verilog/, mult.sv mult_stage.sv)
# CVFPU v0.8.1 (git submodule vendor/cvfpu, FP32 only; its own submodules common_cells and
# fpu_div_sqrt_mvp supply the rest). Packages first. vendor.vlt turns lint off for vendor code only.
# Three CVFPU files are built from vendor/patches/*.patch into $(BUILD)/vendor_patched/ (see
# vendor/PATCHES.md), so the submodule itself stays pristine.
# (after cloning: git submodule update --init --recursive)
VND := $(SRC)/vendor
CVFPU_PATCHED := fpnew_fma fpnew_fma_multi fpnew_divsqrt_th_32
VENDOR_FPU := $(VND)/vendor.vlt \
  $(addprefix $(VND)/cvfpu/src/common_cells/src/, cf_math_pkg.sv lzc.sv rr_arb_tree.sv) \
  $(addprefix $(VND)/cvfpu/src/fpu_div_sqrt_mvp/hdl/, defs_div_sqrt_mvp.sv control_mvp.sv iteration_div_sqrt_mvp.sv \
      norm_div_sqrt_mvp.sv preprocess_mvp.sv nrbd_nrsc_mvp.sv div_sqrt_top_mvp.sv) \
  $(addprefix $(VND)/cvfpu/vendor/opene906/E906_RTL_FACTORY/gen_rtl/, clk/rtl/gated_clk_cell.v \
      $(addprefix fdsu/rtl/, pa_fdsu_ctrl.v pa_fdsu_ff1.v pa_fdsu_pack_single.v pa_fdsu_prepare.v \
          pa_fdsu_round_single.v pa_fdsu_special.v pa_fdsu_srt_single.v pa_fdsu_top.v) \
      $(addprefix fpu/rtl/, pa_fpu_dp.v pa_fpu_frbus.v pa_fpu_src_type.v)) \
  $(addprefix $(VND)/cvfpu/src/, fpnew_pkg.sv fpnew_cast_multi.sv fpnew_classifier.sv) \
  $(CVFPU_PATCHED:%=$(BUILD)/vendor_patched/%.sv) \
  $(addprefix $(VND)/cvfpu/src/, fpnew_divsqrt_multi.sv fpnew_noncomp.sv fpnew_opgroup_block.sv \
      fpnew_opgroup_fmt_slice.sv fpnew_opgroup_multifmt_slice.sv fpnew_rounding.sv fpnew_top.sv)
# a patched copy of a CVFPU source (fails if the patch no longer applies); kept, not intermediate
.PRECIOUS: $(BUILD)/vendor_patched/%.sv
$(BUILD)/vendor_patched/%.sv: $(VND)/cvfpu/src/%.sv $(VND)/patches/%.patch
	@mkdir -p $(dir $@)
	patch -s -o $@ $< $(VND)/patches/$*.patch
# the FP unit's reference model: Spike's SoftFloat (RISC-V specialization), shipped with Spike
SPIKE_ROOT ?= $(shell dirname $$(dirname $$(readlink -f $$(command -v spike))))
DEPS_fp_fu    := $(VENDOR_FPU)
CSRC_fp_fu    := $(SRC)/test/fp_ref.c
LDFLAGS_fp_fu := -L$(SPIKE_ROOT)/lib -lsoftfloat -Wl,-rpath,$(SPIKE_ROOT)/lib
CFLAGS_fp_fu  := -I$(SPIKE_ROOT)/include/softfloat

# the whole core (cpu.sv is the top)
DEPS_cpu := $(addprefix $(SRC)/verilog/, dispatch.sv decoder.sv freelist.sv rat.sv busy_table.sv \
              rob.sv retire_rat.sv rs.sv prf.sv alu_fu.sv branch_fu.sv mult_fu.sv mult.sv mult_stage.sv div_fu.sv \
              branch_pred.sv ras.sv cdb_arbiter.sv \
              icache.sv iprefetch.sv dcache.sv mem_arbiter.sv lsq.sv load_wait_table.sv fp_fu.sv) $(SRC)/test/mem.sv \
            $(VENDOR_FPU) $(ICACHE_SRAM_MODEL)
CSRC_cpu    := $(CSRC_fp_fu)
LDFLAGS_cpu := $(LDFLAGS_fp_fu)
CFLAGS_cpu  := $(CFLAGS_fp_fu)

# ns. 1.53 = Design Compiler's period for the core (ASAP7, 2026-10-22), so mem.sv's 100 ns is
# a realistic ~66 cycles. CLOCK_PERIOD=10 gives the old 10-cycle memory (faster simulations).
CLOCK_PERIOD ?= 1.53

VERILATOR       := verilator
VERILATOR_FLAGS := --binary --timing -j 0 -Wall -Wno-fatal \
                   -I$(SRC) -I$(SRC)/vendor/cvfpu/src/common_cells/include +define+CLOCK_PERIOD=$(CLOCK_PERIOD) \
                   --top-module testbench -Wno-DECLFILENAME --timescale 1ns/1ps

HEADERS := $(wildcard $(SRC)/verilog/*.svh)

.PHONY: test clean synth_local $(TESTS:%=tb_%) $(TESTS:%=lint_%)

test: $(TESTS:%=tb_%)

# build + run one testbench; fail the make target unless it prints "@@@ Passed"
$(TESTS:%=tb_%): tb_%: $(BUILD)/%/Vtestbench
	@$(abspath $<) | tee $(BUILD)/$*/sim.log
	@grep -q "@@@ Passed" $(BUILD)/$*/sim.log

# rebuild when any source of the test changes, including its DEPS_<mod> files
.SECONDEXPANSION:
# (a variant <t> with BASE_<t> set reuses that module's test and source)
$(BUILD)/%/Vtestbench: $(SRC)/test/$$(or $$(BASE_$$*),$$*)_test.sv $(SRC)/verilog/$$(or $$(BASE_$$*),$$*).sv \
                       $$(DEPS_$$*) $$(CSRC_$$*) $(HEADERS)
	@mkdir -p $(BUILD)
	$(VERILATOR) $(VERILATOR_FLAGS) $(VDEFS_$*) --Mdir $(BUILD)/$* \
		$(if $(CFLAGS_$*),-CFLAGS "$(CFLAGS_$*)") $(if $(LDFLAGS_$*),-LDFLAGS "$(LDFLAGS_$*)") \
		$(DEPS_$*) $(SRC)/test/$(or $(BASE_$*),$*)_test.sv $(SRC)/verilog/$(or $(BASE_$*),$*).sv $(CSRC_$*)

$(TESTS:%=lint_%): lint_%:
	$(VERILATOR) --lint-only -Wall -I$(SRC) +define+CLOCK_PERIOD=$(CLOCK_PERIOD) \
		$(SRC)/verilog/$*.sv $(DEPS_$*)

# ---- local synthesis (open source): Yosys + slang (SystemVerilog) + ABC, ASAP7 ----
# Quick what-if area / rough-timing estimates on this machine. The numbers to report come from
# Design Compiler on cadpc12; this optimizer differs, so compare local runs with each other.
# Needs synth_local/lib/ (gitignored): asap7sc7p5t_merged_RVT_FF_nldm_211120.lib. The I-cache
# SRAM bank is a black box (synth_local/srambank_*_blackbox.sv); add its LEF area by hand:
# 1244.2 um^2. read_slang needs --single-unit: sys_defs.svh defines functions outside any module,
# and one copy per file's compilation unit breaks the import (duplicate wire assert).
#   make synth_local                      the whole core at SYNTH_PERIOD ps
#   make synth_local SYNTH_TOP=rs         one module (out-of-context)
SYNTH_TOP    ?= cpu
SYNTH_PERIOD ?= 1000
SYNTH_LIB    := synth_local/lib/asap7sc7p5t_merged_RVT_FF_nldm_211120.lib
SYNTH_SRAM   := synth_local/srambank_128x4x64_6t122_blackbox.sv
SYNTH_OUT    := synth_local/out/$(SYNTH_TOP)_$(SYNTH_PERIOD)
SYNTH_SRCS    = $(filter-out %.vlt $(SRC)/test/% $(ICACHE_SRAM_MODEL),$(DEPS_cpu)) $(SRC)/verilog/cpu.sv
YOSYS        ?= yosys

synth_local: $(SYNTH_SRCS) $(HEADERS) $(SYNTH_LIB) $(SYNTH_SRAM)
	@mkdir -p $(SYNTH_OUT)
	$(YOSYS) -m slang -q -l $(SYNTH_OUT)/yosys.log -p " \
	  read_slang -I $(SRC) -I $(SRC)/vendor/cvfpu/src/common_cells/include -D SYNTHESIS \
	             --single-unit --unroll-limit=1000000 \
	             --translate-off-format synopsys,translate_off,translate_on \
	             --top $(SYNTH_TOP) $(SYNTH_SRAM) $(SYNTH_SRCS); \
	  hierarchy -top $(SYNTH_TOP); \
	  tee -q -o $(SYNTH_OUT)/stat_hier.txt stat; \
	  synth -top $(SYNTH_TOP) -flatten; \
	  dfflibmap -liberty $(SYNTH_LIB); \
	  abc -liberty $(SYNTH_LIB) -D $(SYNTH_PERIOD); \
	  opt_clean; \
	  tee -o $(SYNTH_OUT)/area.txt stat -liberty $(SYNTH_LIB)"
	@grep -E "Chip area|Number of cells" $(SYNTH_OUT)/area.txt | head -3

# print a variable: make print-SYNTH_SRCS
print-%:
	@echo $($*)

clean:
	rm -rf $(BUILD)
