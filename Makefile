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

TESTS := prf freelist rat rob

# extra sources per module, e.g. DEPS_mult := $(SRC)/verilog/mult_stage.sv
DEPS_prf :=

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

$(BUILD)/%/Vtestbench: $(SRC)/test/%_test.sv $(SRC)/verilog/%.sv $(HEADERS)
	$(VERILATOR) $(VERILATOR_FLAGS) --Mdir $(BUILD)/$* \
		$(SRC)/test/$*_test.sv $(SRC)/verilog/$*.sv $(DEPS_$*)

$(TESTS:%=lint_%): lint_%:
	$(VERILATOR) --lint-only -Wall -I$(SRC) +define+CLOCK_PERIOD=$(CLOCK_PERIOD) \
		$(SRC)/verilog/$*.sv $(DEPS_$*)

clean:
	rm -rf $(BUILD)
