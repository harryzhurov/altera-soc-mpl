#*****************************************************************************
#
# Copyright 2014-2016 Altera Corporation. All Rights Reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
# 
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
# 
# 3. Neither the name of the copyright holder nor the names of its contributors
# may be used to endorse or promote products derived from this software without
# specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
# 
#*****************************************************************************

#
# $Id: //acds/rel/16.1/embedded/examples/software/Altera-SoCFPGA-HardwareLib-MPL/Makefile#16 $
#

include config.mk

# Should point to the directory of your generated handoff code
# In windows this should start with drive letter (eg c:), not /cygwin/c
ifeq ($(DEVICE), A5)
  ALT_DEVICE_FAMILY ?= soc_cv_av
  HANDOFF_BASE := $(SOCEDS_DEST_ROOT)/examples/hardware/av_soc_devkit_ghrd
  HANDOFF_SDRAM_DIR ?= $(HANDOFF_BASE)/hps_isw_handoff/ghrd_5astfd5k3_hps_0
  HANDOFF_DIR ?= $(HANDOFF_BASE)/software/preloader/generated
  CFLAGS_CPU := -DDEFAULT_TERM=term0
  MKPIMAGE_VER := 0
else ifeq ($(DEVICE), C5)
  ALT_DEVICE_FAMILY := soc_cv_av
  HANDOFF_BASE := ..
  HANDOFF_SDRAM_DIR := $(HANDOFF_BASE)/hps_isw_handoff/slon1_soc_hps_0
  HANDOFF_DIR := $(HANDOFF_BASE)/software/spl_bsp/generated
  CFLAGS_CPU := -DDEFAULT_TERM=term0
  MKPIMAGE_VER := 0
else #A10
  ALT_DEVICE_FAMILY := soc_a10
  ifeq ($(BOOT_SOURCE), SDMMC)
    HANDOFF_BASE :=$(SOCEDS_DEST_ROOT)/examples/hardware/a10_soc_devkit_ghrd
  else ifeq ($(BOOT_SOURCE), QSPI)
    # Modify this with the location of your design
    HANDOFF_BASE := $(SOCEDS_DEST_ROOT)/not_shipped/examples/hardware/a10_soc_devkit_qspi_ddr4
  endif
  HANDOFF_SDRAM_DIR ?= .
  HANDOFF_DIR ?= $(HANDOFF_BASE)/software/bootloader/uboot-socfpga/board/altera/socfpga
  CFLAGS_CPU := -DDEFAULT_TERM=term1
  MKPIMAGE_VER := 1
endif

test:
	echo $(DEVICE)

# headers and include paths
HEADER_FILES := $(HANDOFF_DIR)/build.h


.PHONY: check_options
check_options:
	@if [[ ("$(BOOT_SOURCE)" != "SDMMC") && \
               ("$(BOOT_SOURCE)" != "SDMMC_FAT") && \
               ("$(BOOT_SOURCE)" != "QSPI") && \
               ("$(BOOT_SOURCE)" != "NAND") && \
               ("$(BOOT_SOURCE)" != "NONE") ]]; \
		then \
		echo >&2 "Error, You must specify BOOT_SOURCE=QSPI, SDMMC, SDMMC_FAT, NAND, or NONE in config.mk"; \
		exit 1; \
	fi
	@if [[ ("$(DEVICE)" != "C5") && ("$(DEVICE)" != "A5") && ("$(DEVICE)" != "A10") ]]; \
		then \
		echo >&2 "Error, you must specify DEVICE=C5, A5, or A10 in config.mk"; \
		exit 1; \
	fi
	@if [[ ("$(COMPILER)" != "ARM") && ("$(COMPILER)" != "GNU") ]]; \
                then \
                echo >&2 "Error, you must specify COMPILER=ARM or GNU in config.mk"; \
                exit 1; \
        fi
	@if [ ! -f $(HEADER_FILES) ]; then \
		echo >&2 "Error, $(HEADER_FILES) does not exist"; \
		exit 1; \
	fi
	@if [ -z "$(HANDOFF_DIR)" ]; then \
		echo >&2 "You must specify HANDOFF_BASE to point to your generated code directory"; \
		exit 1; \
	fi

ifeq ($(BOOT_SOURCE), SDMMC)
 CFLAGS_BOOT = -DSDMMC_BOOT
 FILE_APPEND = _s
 BOOT_SRC    = core/sdmmc_load.c \
               core/sdmmc_init.c \
               $(HWLIBS_ROOT)/alt_sdmmc.c
 ifeq ($(ALT_DEVICE_FAMILY), soc_a10)
  ifneq ($(LOAD_FPGA),)
   BOOT_SRC   += core/sdmmc_load_fpga.c
  endif
 endif
endif

ifeq ($(BOOT_SOURCE), SDMMC_FAT)
 CFLAGS_BOOT = -DFAT_BOOT
 FILE_APPEND = _f
 BOOT_SRC    = $(HWLIBS_ROOT)/alt_sdmmc.c \
               core/fat_load.c \
               core/sdmmc_init.c \
               fatfs/ff.c
 ifeq ($(ALT_DEVICE_FAMILY), soc_a10)
   ifneq ($(LOAD_FPGA),)
     BOOT_SRC   += core/fat_load_fpga.c
   endif
  endif
endif

ifeq ($(BOOT_SOURCE), QSPI)
 CFLAGS_BOOT = -DQSPI_BOOT
 FILE_APPEND = _q
 BOOT_SRC    = $(HWLIBS_ROOT)/alt_qspi.c
 ifeq ($(ALT_DEVICE_FAMILY), soc_a10)
  BOOT_SRC    += core/qspi_load_a10.c
 else
  BOOT_SRC    += core/qspi_load.c
 endif
endif

ifeq ($(BOOT_SOURCE), NAND)
 CFLAGS_BOOT = -DNAND_BOOT
 FILE_APPEND = _n
 BOOT_SRC    = core/nand_load.c \
               $(HWLIBS_ROOT)/alt_nand.c
endif
ifeq ($(BOOT_SOURCE), NONE)
 CFLAGS_BOOT = -DNO_BOOT
endif

ifeq ($(COMPILER), GNU)
 GNU=1
endif

OBJ_ELF         = mpl_$(DEVICE)$(FILE_APPEND).axf
OBJ_BIN         = mpl_$(DEVICE)$(FILE_APPEND).bin
OBJ_PIMAGE      = mpl_$(DEVICE)$(FILE_APPEND)-mkpimage.bin
OBJ_HEX         = mpl_$(DEVICE)$(FILE_APPEND).hex

HWLIBS_ROOT := $(SOCEDS_DEST_ROOT)/ip/altera/hps/altera_hps/hwlib
HWLIBS_DIR  := $(HWLIBS_ROOT)/src/hwmgr
HWLIBS_UTIL  := $(HWLIBS_ROOT)/src/utils
SOCAL_DIR := $(HWLIBS_ROOT)/include/$(ALT_DEVICE_FAMILY)/socal

PINMUX         := core/system_manager_pinmux_a10.c 
CLK_DATA       := core/clk_data.c 
DTS            := $(HANDOFF_BASE)/software/bootloader/devicetree.dts 

ifneq ($(DEVICE), A10)
    # soc_cv_av
    HWLIBS_SRC    := $(HWLIBS_DIR)/alt_watchdog.c \
                     $(HWLIBS_DIR)/$(ALT_DEVICE_FAMILY)/alt_clock_manager.c \
                     $(HWLIBS_DIR)/$(ALT_DEVICE_FAMILY)/alt_clock_manager_init.c \
                     $(HWLIBS_DIR)/alt_timers.c \
                     $(HWLIBS_DIR)/alt_globaltmr.c \
                     $(HWLIBS_DIR)/alt_dma.c \
                     $(HWLIBS_DIR)/alt_dma_program.c \
                     $(HWLIBS_DIR)/$(ALT_DEVICE_FAMILY)/alt_fpga_manager.c
else # A10
    HWLIBS_SRC    := $(HWLIBS_DIR)/alt_watchdog.c \
                     $(HWLIBS_DIR)/$(ALT_DEVICE_FAMILY)/alt_clock_manager.c \
                     $(HWLIBS_DIR)/alt_timers.c \
                     $(HWLIBS_DIR)/alt_cache.c \
                     $(HWLIBS_DIR)/alt_globaltmr.c \
                     $(HWLIBS_DIR)/$(ALT_DEVICE_FAMILY)/alt_fpga_manager.c
endif


ifeq ($(DEVICE), A10)
$(PINMUX): $(DTS)
	./tools/gen_pinmux.sh $(DTS) > $(PINMUX) 
$(CLK_DATA): $(DTS)
	./tools/gen_clk.sh $(DTS) > $(CLK_DATA)
endif

ifneq ($(DEVICE), A10)
    CORE_SRC      := core/mpl.c \
                     core/crc32.c \
                     core/board_init.c \
                     core/header.c
else
    CORE_SRC      := core/crc32.c \
                     core/header.c \
                     core/board_init_a10.c \
                     core/mpl_a10.c \
                     core/clk_data.c \
                     core/memory_init_a10.c \
                     core/system_manager_pinmux_a10.c
endif


ifneq ($(DEVICE), A10)
    ALTERA_IP_SRC := core/altera_ip/freeze_controller.c \
                     core/altera_ip/scan_manager.c \
                     core/altera_ip/system_manager_pinmux.c \
                     core/altera_ip/sdram.c


    SDRAM_SRC     := $(HANDOFF_SDRAM_DIR)/sequencer_auto_ac_init.c \
                     $(HANDOFF_SDRAM_DIR)/sequencer_auto_inst_init.c \
                     $(HANDOFF_SDRAM_DIR)/sequencer.c
endif

ASM_SRC       := core/startup.S

INC_PATH := -I. \
            -I./fatfs \
            -I./core/ -I./core/altera_ip \
            -I$(HWLIBS_ROOT)/include \
            -I$(HWLIBS_ROOT)/include/$(ALT_DEVICE_FAMILY) \
            -I$(SOCAL_DIR) \
            -I$(HANDOFF_DIR) \
            -I$(HANDOFF_SDRAM_DIR)

RM := rm
CP := cp -f

ifdef GNU

 CROSS_COMPILE := arm-altera-eabi-
 CC := $(CROSS_COMPILE)gcc
 AS := $(CROSS_COMPILE)gcc
 LD := $(CROSS_COMPILE)gcc
 AR := $(CROSS_COMPILE)ar
 NM := $(CROSS_COMPILE)nm
 OD := $(CROSS_COMPILE)objdump
 OC := $(CROSS_COMPILE)objcopy

 $(OBJ_BIN): $(OBJ_ELF)
	$(OC) -O binary $< $@

 $(OBJ_HEX): $(OBJ_ELF)
	$(OC) -O ihex --adjust-vma -0xc0000000 $^ $@

# compile optimization flag
 OFLAG ?= -Os


 COMMON_FLAGS		:=-Wno-sign-compare -mcpu=cortex-a9 -march=armv7-a -ffunction-sections -mno-unaligned-access\
				-fdata-sections -fno-builtin -fno-stack-protector -Os -D$(ALT_DEVICE_FAMILY) $(CFLAGS_CPU)\
                                -DDEBUG_ALT_QSPI -DALT_QSPI_SUPPORT_ALL
 ASMFLAGS_COMPILER 	:=-c -DGNU $(COMMON_FLAGS)
 CFLAGS_HWLIBS		:= $(COMMON_FLAGS) -std=c99
 CFLAGS_COMPILER	:= $(COMMON_FLAGS) -std=c99
 CFLAGS_HANDOFF		:= $(COMMON_FLAGS) -msoft-float -fstack-usage -fno-common

 ifeq ($(BOOT_FROM_FPGA),1)
  # if BOOT_FROM_FPGA
  ASMFLAGS_COMPILER += -DBOOT_FROM_FPGA
  CFLAGS_COMPILER   += -DBOOT_FROM_FPGA
  LINKER_SCRIPT 		:=mpl_gcc_fpga.lds
 else
  # not BOOT_FROM_FPGA
    ifneq ($(DEVICE), A10)
        LINKER_SCRIPT 		:= mpl_gcc.lds
    else 
        LINKER_SCRIPT 		:= mpl_a10.lds
    endif
 endif

 LDFLAGS_COMPILER 	:=-nostartfiles -Wl,--entry=mpl_vectors -Wl,-script=$(LINKER_SCRIPT) -Wl,--gc-sections

else ############ Not GNU = armcc build

 CC := armcc
 AS := armasm
 LD := armlink
 AR := armar
 OC := fromelf

 $(OBJ_BIN): $(OBJ_ELF)
	$(OC) --bin --output=$@ $<

 $(OBJ_HEX): $(OBJ_ELF)
	$(OC) --i32 --base 0xc0000000 $^ --output temp
	cat temp | sed -n '1!p' > $@
	rm temp

# compile optimization flag
 OFLAG ?= -Ospace

 COMMON_FLAGS		:=  --diag_suppress=9931 --cpu=Cortex-A9.no_neon.no_vfp --split_sections --c99 \
			--no_unaligned_access -DARMCOMPILER -D$(ALT_DEVICE_FAMILY)
 CFLAGS_HWLIBS		:= $(COMMON_FLAGS)
 CFLAGS_COMPILER 	:= $(COMMON_FLAGS) 
 CFLAGS_HANDOFF		:= $(COMMON_FLAGS) --diag_suppress=68 --diag_suppress 177 --diag_suppress=550
 ASMFLAGS_COMPILER 	:= --cpu=Cortex-A9.no_neon.no_vfp --cpreproc

 ifeq ($(BOOT_FROM_FPGA),1)
  # if BOOT_FROM_FPGA
  ASMFLAGS_COMPILER += --predefine "BOOT_FROM_FPGA SETL {TRUE}"
  CFLAGS_COMPILER += -DBOOT_FROM_FPGA
  LINKER_SCRIPT 		:= mpl_fpga.scat
 else
  # not BOOT_FROM_FPGA
    ifneq ($(DEVICE), A10)
        LINKER_SCRIPT 		:= mpl.scat
    else
        LINKER_SCRIPT 		:= mpl_a10.scat
    endif
 endif

 LDFLAGS_COMPILER 	:= --diag_suppress=9931  --cpu=Cortex-A9.no_neon.no_vfp --strict --remove --datacompressor off --diag_error=warning --scatter=$(LINKER_SCRIPT) --entry=mpl_vectors

endif # GNU

CFLAGS   := -g $(OFLAG) $(INC_PATH) $(CFLAGS_SRC) $(CFLAGS_BOOT)

# Larger DMA buffer needed for SDRAM ECC clearing.
CFLAGS += -DALT_DMA_PROGRAM_PROVISION_BUFFER_SIZE=3072

ifeq ($(LOAD_FPGA),1)
 CFLAGS += -DCONFIG_MPL_FPGA_LOAD
endif

ASMFLAGS := -g $(ASMFLAGS_COMPILER) $(COMMON_FLAGS)
LDFLAGS  := $(LDFLAGS_COMPILER)

ifeq ($(DEVICE), A5)
 BOARD_SRC:= 	$(HANDOFF_DIR)/iocsr_config_arria5.c \
		$(HANDOFF_DIR)/pinmux_config_arria5.c
else ifeq ($(DEVICE), C5)
 BOARD_SRC:= 	$(HANDOFF_DIR)/iocsr_config_cyclone5.c \
		$(HANDOFF_DIR)/pinmux_config_cyclone5.c
endif

ifeq ($(DEVICE), A5)
 CFLAGS     += -DCONFIG_SOCFPGA_ARRIA5
else ifeq ($(DEVICE), C5)
 CFLAGS     += -DCONFIG_SOCFPGA_CYCLONE5
else ifeq ($(DEVICE), A10)
 CFLAGS     += -DCONFIG_SOCFPGA_ARRIA10
endif

ifdef PRINTF_HOST

 CFLAGS     := $(CFLAGS) -DPRINTF_HOST
 ifdef GNU
  $(error *** You cannot build semihosted (PRINTF_HOST) with GNU!!)
 endif

else ifeq ($(PRINTF_UART),1)
 HWLIBS_SRC+= 	$(HWLIBS_DIR)/alt_printf.c \
 		$(HWLIBS_DIR)/alt_p2uart.c \
		$(HWLIBS_DIR)/alt_16550_uart.c

 CFLAGS     += -DPRINTF_UART

 ifdef UART_BAUD_RATE
  CFLAGS     += -DBAUD_RATE=$(UART_BAUD_RATE)
 endif
endif

C_SRC:= $(CORE_SRC) $(BOOT_SRC) $(HWLIBS_SRC) $(ALTERA_IP_SRC) $(BOARD_SRC) $(SDRAM_SRC)

OBJS:= $(patsubst %.S, bin/%.o,$(notdir $(ASM_SRC))) $(patsubst %.c, bin/%.o,$(notdir $(C_SRC)))


ifeq ($(BOOT_FROM_FPGA),1)
 .DEFAULT_GOAL := $(OBJ_HEX)
 .PHONY: all
 all: $(OBJ_HEX)
else
 .DEFAULT_GOAL := $(OBJ_PIMAGE)
 .PHONY: all
 all: $(OBJ_PIMAGE)

endif

$(OBJ_ELF): $(OBJS) Makefile
	$(LD) $(LDFLAGS) $(OBJS) -o $@

$(OBJ_PIMAGE): $(OBJ_BIN)
	$(SOCEDS_DEST_ROOT)/host_tools/altera/mkpimage/mkpimage --header-version $(MKPIMAGE_VER) -o $@ $< $< $< $<

bin/%.o: core/%.S check_options bin
	$(AS) -o $@ $(ASMFLAGS_COMPILER) $<

bin/%.o: core/%.c check_options bin
	$(CC) -o $@ -c $(CFLAGS) $(CFLAGS_COMPILER) $<

bin/%.o: fatfs/%.c check_options bin
	$(CC) -o $@ -c $(CFLAGS) $(CFLAGS_COMPILER) $<

bin/%.o: $(HWLIBS_UTIL)/%.c check_options bin
	$(CC) -o $@ -c $(CFLAGS) $(CFLAGS_HWLIBS) $<

bin/%.o: $(HWLIBS_DIR)/%.c check_options bin
	$(CC) -o $@ -c $(CFLAGS) $(CFLAGS_HWLIBS) $<

bin/%.o: $(HWLIBS_DIR)/$(ALT_DEVICE_FAMILY)/%.c check_options bin
	$(CC) -o $@ -c $(CFLAGS) $(CFLAGS_HWLIBS) $<

bin/%.o: $(HANDOFF_DIR)/%.c check_options bin
	$(CC) -o $@ -c $(CFLAGS) $(CFLAGS_HANDOFF) $<

bin/%.o: $(HANDOFF_SDRAM_DIR)/%.c check_options bin
	$(CC) -o $@ -c $(CFLAGS) $(CFLAGS_HANDOFF) $<

bin/%.o: core/altera_ip/%.c check_options bin
	$(CC) -o $@ -c $(CFLAGS) $(CFLAGS_COMPILER) $<

bin:
	mkdir $@

.PHONY: clean
clean:
	$(RM) -rf mpl_*.axf bin mpl_*.bin mpl_*.hex $(PINMUX) $(CLK_DATA)
