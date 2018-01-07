SRCS = \
  ../rtl/riscv_core.v \
  memory.v \
  riscv_tb.v \

UNAME := $(shell uname)

ifeq ($(UNAME), Linux)
  TEMP=/tmp
else
  TEMP=c:/temp
endif

VCD_FILE=$(TEMP)/__riscv.vcd
SIM_FILE=$(TEMP)/__riscv.sim

all:
	@-rm $(VCD_FILE)
	@iverilog -Wall -o $(SIM_FILE) -DVCD_FILE=\"$(VCD_FILE)\" $(SRCS)
	@vvp -n $(SIM_FILE)

wave:
	@gtkwave -a riscv.gtkw $(VCD_FILE)


