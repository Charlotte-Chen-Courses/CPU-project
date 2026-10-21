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
# source files the module needs in DEPS_<mod>.

SRC   := starter
BUILD := build

TESTS := prf freelist rat rob rs busy_table alu_fu branch_fu mult_fu div_fu branch_pred ras cdb_arbiter decoder retire_rat dcache mem_arbiter lsq load_wait_table cpu

# extra sources per module, e.g. DEPS_mult := $(SRC)/verilog/mult_stage.sv
DEPS_prf :=
DEPS_dcache  := $(SRC)/test/mem.sv
DEPS_mem_arbiter := $(SRC)/verilog/dcache.sv $(SRC)/verilog/icache.sv $(SRC)/test/mem.sv
DEPS_lsq     := $(SRC)/test/tb_dmem.sv
DEPS_mult_fu := $(addprefix $(SRC)/verilog/, mult.sv mult_stage.sv)
# the whole core (cpu.sv is the top)
DEPS_cpu := $(addprefix $(SRC)/verilog/, dispatch.sv decoder.sv freelist.sv rat.sv busy_table.sv \
              rob.sv retire_rat.sv rs.sv prf.sv alu_fu.sv branch_fu.sv mult_fu.sv mult.sv mult_stage.sv div_fu.sv \
              branch_pred.sv ras.sv cdb_arbiter.sv \
              icache.sv dcache.sv mem_arbiter.sv lsq.sv load_wait_table.sv) $(SRC)/test/mem.sv

CLOCK_PERIOD ?= 10

VERILATOR       := verilator
VERILATOR_FLAGS := --binary --timing -j 0 -Wall -Wno-fatal \
                   -I$(SRC) +define+CLOCK_PERIOD=$(CLOCK_PERIOD) \
                   --top-module testbench -Wno-DECLFILENAME

HEADERS := $(wildcard $(SRC)/verilog/*.svh)

.PHONY: test clean $(TESTS:%=tb_%) $(TESTS:%=lint_%)

test: $(TESTS:%=tb_%)

# build + run one testbench; fail the make target unless it prints "@@@ Passed"
$(TESTS:%=tb_%): tb_%: $(BUILD)/%/Vtestbench
	@./$< | tee $(BUILD)/$*/sim.log
	@grep -q "@@@ Passed" $(BUILD)/$*/sim.log

# rebuild when any source of the test changes, including its DEPS_<mod> files
.SECONDEXPANSION:
$(BUILD)/%/Vtestbench: $(SRC)/test/%_test.sv $(SRC)/verilog/%.sv $$(DEPS_$$*) $(HEADERS)
	$(VERILATOR) $(VERILATOR_FLAGS) --Mdir $(BUILD)/$* \
		$(SRC)/test/$*_test.sv $(SRC)/verilog/$*.sv $(DEPS_$*)

$(TESTS:%=lint_%): lint_%:
	$(VERILATOR) --lint-only -Wall -I$(SRC) +define+CLOCK_PERIOD=$(CLOCK_PERIOD) \
		$(SRC)/verilog/$*.sv $(DEPS_$*)

clean:
	rm -rf $(BUILD)
