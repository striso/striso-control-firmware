##############################################################################
# Build global options
# NOTE: Can be overridden externally.
#

# Compiler options here.
ifeq ($(USE_OPT),)
	USE_OPT = -O1 -ggdb -fomit-frame-pointer -falign-functions=16
endif

# C specific options here (added to USE_OPT).
ifeq ($(USE_COPT),)
	USE_COPT = -std=gnu99
endif

# C++ specific options here (added to USE_OPT).
ifeq ($(USE_CPPOPT),)
	USE_CPPOPT = -fno-rtti -std=c++11
endif

# Enable this if you want the linker to remove unused code and data.
ifeq ($(USE_LINK_GC),)
	USE_LINK_GC = yes
endif

# Linker extra options here.
ifeq ($(USE_LDOPT),)
	USE_LDOPT = --print-memory-usage
endif

# Enable this if you want link time optimizations (LTO).
ifeq ($(USE_LTO),)
	USE_LTO = yes
endif

# Enable this if you want to see the full log while compiling.
ifeq ($(USE_VERBOSE_COMPILE),)
	USE_VERBOSE_COMPILE = no
endif

# If enabled, this option makes the build process faster by not compiling
# modules not used in the current configuration.
ifeq ($(USE_SMART_BUILD),)
	USE_SMART_BUILD = yes
endif

#
# Build global options
##############################################################################

##############################################################################
# Architecture or project specific options
#

# Stack size to be allocated to the Cortex-M process stack. This stack is
# the stack used by the main() thread.
ifeq ($(USE_PROCESS_STACKSIZE),)
  USE_PROCESS_STACKSIZE = 0x400
endif

# Stack size to the allocated to the Cortex-M main/exceptions stack. This
# stack is used for processing interrupts and exceptions.
ifeq ($(USE_EXCEPTIONS_STACKSIZE),)
  USE_EXCEPTIONS_STACKSIZE = 0x400
endif

# Enables the use of FPU (no, softfp, hard).
ifeq ($(USE_FPU),)
	USE_FPU = softfp
endif

# FPU-related options.
ifeq ($(USE_FPU_OPT),)
  USE_FPU_OPT = -mfloat-abi=$(USE_FPU) -mfpu=fpv5-d16
#  USE_FPU_OPT = -mfloat-abi=$(USE_FPU) -mfpu=fpv5-sp-d16 -fsingle-precision-constant
endif

#
# Architecture or project specific options
##############################################################################

##############################################################################
# Project, target, sources and paths
#

# Define project name here
PROJECT = striso_control

# Target settings.
MCU  = cortex-m7

# Imported source files and paths.
CHIBIOS  := ./ChibiOS
CONFDIR  := ./cfg
BUILDDIR := ./build
DEPDIR   := ./.dep

# Licensing files.
include $(CHIBIOS)/os/license/license.mk
# Startup files.
include $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk/startup_stm32h7xx.mk
# HAL-OSAL files (optional).
include $(CHIBIOS)/os/hal/hal.mk
include $(CHIBIOS)/os/hal/ports/STM32/STM32H7xx/platform.mk
# include $(CHIBIOS)/os/hal/boards/ST_NUCLEO144_H743ZI/board.mk
include board/board.mk
#include board.mk
include $(CHIBIOS)/os/hal/osal/rt-nil/osal.mk
# RTOS files (optional).
include $(CHIBIOS)/os/rt/rt.mk
include $(CHIBIOS)/os/common/ports/ARMCMx/compilers/GCC/mk/port_v7m.mk
# EX files (optional).
# Auto-build files in ./source recursively.
include $(CHIBIOS)/tools/mk/autobuild.mk
# Other files (optional).
include $(CHIBIOS)/os/hal/lib/streams/streams.mk # for chprintf

# Define linker script file here
#LDSCRIPT= $(STARTUPLD)/STM32F407xG.ld
#LDSCRIPT= STM32F407xE_bootloader.ld
#LDSCRIPT= $(STARTUPLD)/STM32H743xI.ld
LDSCRIPT= STM32H743xI_uf2.ld

# C sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CSRC = $(ALLCSRC) \
	$(CHIBIOS)/os/various/syscalls.c \
	usbcfg.c \
	pconnection.c \
	bulk_usb.c \
	midi_usb.c \
	midi_serial.c \
	button_read.c \
	messaging.c \
	motionsensor.c \
	codec_tlv320aic3x_SAI.c \
	led.c \
	main.c

# C++ sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CPPSRC = $(ALLCPPSRC) \
	synth_control.cpp \
	synth.cpp

# List ASM source files here.
ASMSRC = $(ALLASMSRC)

# List ASM with preprocessor source files here.
ASMXSRC = $(ALLXASMSRC)

# Inclusion directories.
INCDIR = $(CONFDIR) $(ALLINC) $(TESTINC)

# Define C warning options here.
CWARN = -Wall -Wextra -Wundef -Wstrict-prototypes

# Define C++ warning options here.
CPPWARN = -Wall -Wextra -Wundef

#
# Project, target, sources and paths
##############################################################################

##############################################################################
# Start of user section
#

# Git revision description, recompile anything depending on version.h on version change
FWVERSION := $(shell git --no-pager show --date=short --format="%ad" --name-only | head -n1)_$(shell git --no-pager describe --tags --always --long --dirty)
ifneq ($(FWVERSION), $(shell cat .git_version 2>&1))
$(shell echo -n $(FWVERSION) > .git_version)
$(shell touch version.h)
endif

# List all user C define here, like -D_DEBUG=1
UDEFS = -DFWVERSION=\"$(FWVERSION)\"

# Define ASM defines here
UADEFS =

# List all user directories here
UINCDIR =

# List the user directory to look for the libraries here
ULIBDIR =

# List all user libraries here
ULIBS = -lm

#
# End of user section
##############################################################################

##############################################################################
# Custom rules
#

# default: build .uf2 file for use with uf2 bootloader
uf2: $(BUILDDIR)/$(PROJECT).uf2

synth.cpp: synth.dsp faust_synth_template.cpp faust2striso.py
	./faust2striso.py

%.uf2: %.bin
	@python3 uf2/utils/uf2conv.py -c -f 0xa21e1295 -b 0x08040000 $(BUILDDIR)/$(PROJECT).bin -o $(BUILDDIR)/$(PROJECT).uf2

prog_uf2: uf2
	@python3 uf2/utils/uf2conv.py -f 0xa21e1295 -b 0x08040000 $(BUILDDIR)/$(PROJECT).bin

release: uf2
	mkdir -p releases
	cp $(BUILDDIR)/$(PROJECT).uf2 releases/$(PROJECT)_$(FWVERSION).uf2

prog: all
	@# first put striso in DFU mode if it isn't (the - ignores striso_util failure)
	@-./striso_util -d && echo Resetting Striso in DFU mode... && sleep 3
	dfu-util -d0483:df11 -a0 -s0x8040000:leave -D $(BUILDDIR)/$(PROJECT).bin

prog_openocd: all
	openocd -f interface/stlink.cfg -f target/stm32h7x.cfg -c "program $(BUILDDIR)/$(PROJECT).elf reset exit"

prog_uart: all
	./stm32loader.py -e -w -v $(BUILDDIR)/$(PROJECT).bin

# Upload firmware with Black Magic Probe
prog_bmp: all
	gdb-multiarch --batch $(BUILDDIR)/$(PROJECT).elf \
		-ex "target extended-remote /dev/ttyACM0" \
		-ex "monitor swdp_scan" \
		-ex "attach 1" \
		-ex "load" \
		-ex "compare-sections" \
		-ex "kill"

# Launch GDB via openocd debugger
gdb:
	gdb-multiarch $(BUILDDIR)/$(PROJECT).elf -ex "tar extended-remote | openocd -f interface/stlink.cfg -f target/stm32h7x.cfg -c \"stm32h7x.cpu configure -rtos auto; gdb_port pipe; log_output openocd.log\""

gdb_bmp:
	gdb-multiarch $(BUILDDIR)/$(PROJECT).elf \
		-ex "target extended-remote /dev/ttyACM0" \
		-ex "monitor swdp_scan" \
		-ex "attach 1"

# Start GDB server for external use (e.g. Eclipse)
openocd:
	openocd -f interface/stlink.cfg -f target/stm32h7x.cfg -c "stm32h7x.cpu configure -rtos auto;"

version:
	@echo $(FWVERSION)

#
# Custom rules
##############################################################################

##############################################################################
# Common rules
#

RULESPATH = $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk
include $(RULESPATH)/arm-none-eabi.mk
include $(RULESPATH)/rules.mk

#
# Common rules
##############################################################################
