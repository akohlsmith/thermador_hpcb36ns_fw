# STM32 Makefile for GNU toolchain and openocd
#
# Usage:
#	make flash		Flash the main application image
#	make forceloader	set the SRAM bootloader bits to forcibly jump to the bootloader
#	make openocd		Start OpenOCD
#	make debug		Start GDB and attach to OpenOCD
#	make dirs		Create subdirs like obj, dep, ..
#

# A name common to all output files (elf, map, hex, bin, lst)
TARGET     = rangehood

FLASH_JLINK=1
USE_JLINK=1

ifdef USE_JLINK
OCDCFG     = openocd-jlink.cfg
else
OCDCFG     = openocd-onboard.cfg
endif

OPENOCD_SCRIPT_DIR ?= /usr/local/share/openocd/scripts
OPENOCD_BIN_DIR ?= /usr/local/bin

OCDFLAGS   = -c "stm32g4x.cpu configure -rtos auto;"
JLINKFLAGS = -device stm32g474cb -si swd -speed 4000
GDBFLAGS   =

# MCU family and type in various capitalizations o_O
MCU_FAMILY = stm32g4xx
MCU_LC     = stm32g474xx
MCU_MC     = STM32G474xx
MCU_UC     = STM32G474CBUx

SRCS       = main.c

SRCS      += stm32g4xx_it.c

# Basic HAL libraries
SRCS      += system_stm32g4xx.c
SRCS      += stm32g4xx_hal.c stm32g4xx_hal_cortex.c stm32g4xx_hal_msp.c
SRCS      += stm32g4xx_hal_pwr.c stm32g4xx_hal_pwr_ex.c stm32g4xx_hal_rcc.c stm32g4xx_hal_rcc_ex.c
SRCS      += stm32g4xx_hal_gpio.c stm32g4xx_hal_uart.c stm32g4xx_hal_uart_ex.c
SRCS      += stm32g4xx_hal_tim.c stm32g4xx_hal_tim_ex.c
SRCS      += stm32g4xx_hal_adc.c stm32g4xx_hal_adc_ex.c

# Directories
CUBE_DIR   = .

HAL_DIR    = $(CUBE_DIR)/Drivers/STM32G4xx_HAL_Driver
CMSIS_DIR  = $(CUBE_DIR)/Drivers/CMSIS
DEV_DIR    = $(CMSIS_DIR)/Device/ST/STM32G4xx

# that's it, no need to change anything below this line!

###############################################################################
# Toolchain

PREFIX     = arm-none-eabi

ifneq ($(ARM_GCC_BIN),)
PREFIX=$(ARM_GCC_BIN)/arm-none-eabi
endif

CC         = $(PREFIX)-gcc
AR         = $(PREFIX)-ar
OBJCOPY    = $(PREFIX)-objcopy
OBJDUMP    = $(PREFIX)-objdump
SIZE       = $(PREFIX)-size
GDB        = $(PREFIX)-gdb

OCD       ?= $(shell which openocd)
JLINK      = JLinkExe

###############################################################################
# Options

# Version
VERSION_MAJOR ?= 1
VERSION_MINOR ?= 2
VERSION_PATCH ?= 0
VERSION_BUILD ?= 0
VERSION_DEV   ?= 1

DEFS_VERSION  = -DVERSION_MAJOR=$(VERSION_MAJOR)
DEFS_VERSION += -DVERSION_MINOR=$(VERSION_MINOR)
DEFS_VERSION += -DVERSION_PATCH=$(VERSION_PATCH)
DEFS_VERSION += -DVERSION_BUILD=$(VERSION_BUILD)
DEFS_VERSION += -DVERSION_DEV=$(VERSION_DEV)

# Build information
# Execute a silent git command, if it succeeds the exit code is 0 (interpret as not an svn repo)
IS_SVN_REPO  := $(shell git -C . rev-parse > /dev/null 2>&1; echo $$?)

BUILD_AUTHOR := $(shell git config --global user.name | sed 's/[. ]//g')
BUILD_COMMIT := $(shell git rev-parse HEAD | cut -c 1-7)
BUILD_DIRTY  := $(shell git diff --quiet > /dev/null 2>&1; echo $$? )

BUILD_INFO_FLAGS  = -DBUILD_AUTHOR=\"$(BUILD_AUTHOR)\"
BUILD_INFO_FLAGS += -DBUILD_COMMIT=\"$(BUILD_COMMIT)\"
BUILD_INFO_FLAGS += -DBUILD_DIRTY=$(BUILD_DIRTY)

# Defines
DEFS       = -D$(MCU_MC) -DUSE_HAL_DRIVER
#-DPTPD_DBGV

# Add debug flags for dev build
ifeq ($(VERSION_DEV),1)
DEFS      += -DRUNTIME_STACK_DEBUG -DDEBUG
endif

# include search paths (-I)
INCS       = -Iinc
INCS      += -I$(CMSIS_DIR)/Include
INCS      += -I$(DEV_DIR)/Include
INCS      += -I$(HAL_DIR)/Inc
INCS      += -I$(RTOS_DIR)/Source/include
INCS      += -I$(RTOS_DIR)/Source/CMSIS_RTOS
INCS      += -I$(RTOS_DIR)/Source/portable/GCC/ARM_CM7/r0p1
INCS      += -I$(LWIP_DIR)/src/include
INCS      += -I$(LWIP_DIR)/src/include/ipv4
INCS      += -I$(LWIP_DIR)/src/netif
INCS      += -I$(LWIP_DIR)/src/apps/snmp
INCS      += -I$(LWIP_DIR)/src/include/lwip/apps
INCS      += -I$(LWIP_DIR)/system
INCS      += -I$(USB_DIR)/Core/Inc
INCS      += -I$(USB_DIR)/Class/CDC/Inc
INCS      += -I$(PTPD_DIR)/src
INCS      += -I$(PTPD_DIR)/src/xdep

# Library search paths
LIBS       = -L$(CMSIS_DIR)/Lib

# Compiler flags
CFLAGS     = -Wall -g -std=gnu11 -O0
CFLAGS    += -Wno-unused-variable -Wno-unused-function
CFLAGS    += -mlittle-endian -mcpu=cortex-m7 -mthumb
CFLAGS    += -fstack-usage
CFLAGS    += -mfpu=fpv4-sp-d16 -mfloat-abi=hard -fsingle-precision-constant
CFLAGS    += -ffunction-sections -fdata-sections
CFLAGS    += $(INCS) $(DEFS)
#CFLAGS    += -DDEVBOARD

# Linker flags
LDFLAGS    = -Wl,--gc-sections -Wl,-Map=$(TARGET).map $(LIBS) -T$(MCU_UC)_FLASH.ld

# Enable Semihosting
# LDFLAGS   += --specs=rdimon.specs -lc -lrdimon

# This is *required* to get openocd/gdb to see the FreeRTOS v8+ threads (along with the FreeRTOS-openocd.c helper file)
LDFLAGS   += -Wl,--undefined=uxTopUsedPriority

# Source search paths
VPATH      = ./src
VPATH     += $(HAL_DIR)/Src
VPATH     += $(DEV_DIR)/Source
VPATH     += $(RTOS_DIR)/Source
VPATH     += $(RTOS_DIR)/Source/CMSIS_RTOS
VPATH     += $(RTOS_DIR)/Source/portable/GCC/ARM_CM7/r0p1
VPATH     += $(RTOS_DIR)/Source/portable/MemMang
VPATH     += $(LWIP_DIR)/src/api
VPATH     += $(LWIP_DIR)/src/core
VPATH     += $(LWIP_DIR)/src/core/ipv4
VPATH     += $(LWIP_DIR)/src/netif
VPATH     += $(LWIP_DIR)/src/apps
VPATH     += $(LWIP_DIR)/src/apps/sntp
VPATH     += $(LWIP_DIR)/system/OS
VPATH     += $(USB_DIR)/Core/Src
VPATH     += $(USB_DIR)/Class/CDC/Src
VPATH     += $(PTPD_DIR)/src
VPATH     += $(PTPD_DIR)/src/xdep

OBJS       = $(addprefix obj/,$(SRCS:.c=.o))
DEPS       = $(addprefix dep/,$(SRCS:.c=.d))

# Prettify output
V = 0
ifeq ($V, 0)
	Q = @
	P = > /dev/null
endif

###################################################

.PHONY: all avstack dirs flash forceloader debug template clean obj/build.o obj/sw_version_main.o

all: $(TARGET).bin #avstack

obj/build.o: build.c
	@echo "[CC]      $(notdir $<)"
	$Q$(CC) $(CFLAGS) $(BUILD_INFO_FLAGS) -c -o $@ $< -MMD -MF dep/$(*F).d

avstack: FILES=$(shell find obj -name '*.o')
avstack: $(TARGET.elf)
	$Q./avstack.pl $(FILES) > stackreport.txt 2> stackreport.log

$(TARGET).bin: $(TARGET).elf
	$Q$(OBJCOPY) -O ihex --gap-fill 0x00 $(TARGET).elf $(TARGET).hex
	#$Q$(OBJCOPY) -O binary $(TARGET).elf $(TARGET)-tmp.bin
	#$Q./addFileHeader.py --header=$(HEADER_SIZE) --base-addr=0x8060000 --dest=$(TARGET).bin $(TARGET)-tmp.bin
	#$Qrm -f $(TARGET)-tmp.hex
	#$Qrm -f $(TARGET)-tmp.bin

-include $(DEPS)

dirs: dep obj
dep obj src:
	@echo "[MKDIR]   $@"
	$Qmkdir -p $@

obj/sw_version_main.o : sw_version_main.c
	@echo "[CC]      $(notdir $<)"
	$Q$(CC) $(CFLAGS) $(DEFS_VERSION) -c -o $@ $< -MMD -MF dep/$(*F).d

###################################################

# Custom flags for third-party files to silence their build warnings

#LWIP
obj/icmp.o : icmp.c | dirs
	@echo "[CC]      $(notdir $<)"
	$Q$(CC) $(CFLAGS) -Wno-unused-but-set-variable -c -o $@ $< -MMD -MF dep/$(*F).d

#LWIP
obj/snmp_mib2.o : snmp_mib2.c | dirs
	@echo "[CC]      $(notdir $<)"
	$Q$(CC) $(CFLAGS) -Wno-format-zero-length -c -o $@ $< -MMD -MF dep/$(*F).d

###################################################

obj/%.o : %.c | dirs
	@echo "[CC]      $(notdir $<)"
	$Q$(CC) $(CFLAGS) -c -o $@ $< -MMD -MF dep/$(*F).d

$(TARGET).elf: $(OBJS)
	@echo "[LD]      $(TARGET).elf"
	@echo HEADER_SIZE = $(HEADER_SIZE)\; > linker_params.ld
	#$Q$(CC) $(CFLAGS) $(LDFLAGS) src/startup_$(MCU_LC).s $^ -lc -specs=nosys.specs -o $@
	$Q$(CC) $(CFLAGS) $(LDFLAGS) src/startup_$(MCU_LC).s $^ -o $@
	@echo "[VERSION]"
	$Q$(CC) --version
	@echo "[OBJDUMP] $(TARGET).lst"
	$Q$(OBJDUMP) -St $(TARGET).elf >$(TARGET).lst
	@echo "[SIZE]    $(TARGET).elf"
	$(SIZE) $(TARGET).elf

jlinkgdb:
	$(Q)JLinkGDBServer -device stm32f207zg -if SWD -speed 4000 -port 3333

openocd:
	$Q$(OCD) -s $(OPENOCD_SCRIPT_DIR) -f $(OCDCFG) $(OCDFLAGS)

flash: all
	$(Q)if [ "$(FLASH_JLINK)" = "" ]; then \
		$(OCD) -s $(OPENOCD_SCRIPT_DIR) -f $(OCDCFG) $(OCDFLAGS) -c "program $(TARGET).hex verify reset exit"; \
	else \
		$(JLINK) $(JLINKFLAGS) flash.jlink; \
	fi

forceloader:
	$(Q)$(OCD) -s $(OPENOCD_SCRIPT_DIR) -f $(OCDCFG) $(OCDFLAGS) -f openocd-forceloader.cfg -c "init ; reset halt ; set_sram_flags ; reset run ; exit"

debug:
	$(Q)if ! nc -z localhost 3333; then \
		echo "\n\t[Error] OpenOCD is not running! Start it with: 'make openocd'\n"; exit 1; \
	else \
		$(GDB)  -ex "target extended localhost:3333" \
			-ex "maintenance set target-async off" \
			-ex "monitor arm semihosting enable" \
			-ex "monitor reset halt" \
			-ex "continue" \
			$(GDBFLAGS) $(TARGET).elf; \
	fi

clean:
	@echo "[RM]      $(TARGET).hex"; rm -f $(TARGET).hex
	@echo "[RM]      $(TARGET).bin"; rm -f $(TARGET).bin
	@echo "[RM]      $(TARGET).elf"; rm -f $(TARGET).elf
	@echo "[RM]      $(TARGET).map"; rm -f $(TARGET).map
	@echo "[RM]      $(TARGET).lst"; rm -f $(TARGET).lst
	@echo "[RM]      stackreport"; rm -f stackreport.txt stackreport.log
	@echo "[RMDIR]   dep"          ; rm -fr dep
	@echo "[RMDIR]   obj"          ; rm -fr obj
	@rm -f linker_params.ld

