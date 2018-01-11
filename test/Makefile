SRCS = \
  ../rtl/riscv_core.v \
  ../rtl/soc.v \
  ../rtl/bus_mux.v \
  ../rtl/per_gpio.v \
  ../rtl/per_timer.v \
  ../rtl/per_uart.v \
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
	@iverilog -Wall -o $(SIM_FILE) -DSIMULATOR -DVCD_FILE=\"$(VCD_FILE)\" $(SRCS)
	@vvp -n $(SIM_FILE)

wave:
	@gtkwave -a riscv.gtkw $(VCD_FILE)


