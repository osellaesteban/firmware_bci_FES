########################################################################
# Cátedra: Electrónica Programable
# FIUNER - 2018
#
# Autor/es:
# 
# JMReta: jmreta@ingenieria.uner.edu.ar
#
#
# Revisión:
# 17-01-18 - JMReta - Versión inicial adaptada a partir de Firmware v1 - Proyecto CIAA
# 04-02-18 - JMReta - Actualización de Framework
#    	              Generada a partir del Proyecto CIAA - Firmware https://github.com/ciaa/firmware_v2
# 	                  Copyright 2016, Pablo Ridolfi
# 	                  Copyright 2017, Eric Pernia
#
#
# All rights reserved.
#
# This file is part of Workspace.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from this
#    software without specific prior written permission.
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
################################################################################



-include project.mk

PROJECT ?= examples/blinky
TARGET ?= lpc4337_m4
BOARD ?= edu_ciaa_nxp


#JM Incluye los modulos que se deben cargar para compilar el proyecto. 
#include $(PROJECT)/Makefile

# application name
PROJECT_NAME := $(notdir $(PROJECT))

#JM Variable usada para incluir los makfiles de los modulos. No se usará.
## Modules needed by the application
PROJECT_MODULES := modules/$(TARGET)/base \
                   modules/$(TARGET)/drivers_devices \
                   modules/$(TARGET)/drivers_microcontroller \
                   modules/$(TARGET)/dsp \
                   modules/$(TARGET)/chip

# source files folder
PROJECT_SRC_FOLDERS := $(PROJECT)/src

# header files folder
PROJECT_INC_FOLDERS := $(PROJECT)/inc

# source files
PROJECT_C_FILES := $(wildcard $(PROJECT)/src/*.c)
PROJECT_ASM_FILES := $(wildcard $(PROJECT)/src/*.S)

######JM FIN######
#JM Incluye las opciones de compilación para el micro target
#include etc/target/$(TARGET).mk
# Target name
TARGET_NAME := lpc4337_m4

# Default cross-toolchain
CROSS_PREFIX ?= arm-none-eabi-

# variables de rutas o carpetas
OUT_PATH = out/$(TARGET_NAME)
OBJ_PATH = $(OUT_PATH)/obj

# Defined symbols
SYMBOLS += -DDEBUG -DCORE_M4 -D__USE_LPCOPEN -D__LPC43XX__ -D__CODE_RED \
           -DLPC43_MULTICORE_M0APP -D__MULTICORE_MASTER \
					 -D__MULTICORE_MASTER_SLAVE_M0APP

# Compilation flags
CFLAGS  := -Wall -ggdb3 -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 \
           -mfloat-abi=hard -fdata-sections -ffunction-sections -fstack-usage

# Linking flags
LFLAGS  := -nostdlib -fno-builtin -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 \
           -mfloat-abi=hard -Xlinker -Map=$(OUT_PATH)/$(PROJECT_NAME).map \
			  -Wl,--gc-sections,--print-memory-usage

# Linker scripts
ifndef LINK_RAM
LD_FILE := -Tetc/ld/lpc4337_m4_lib.ld -Tetc/ld/lpc4337_m4_mem.ld \
           -Tetc/ld/lpc4337_m4.ld
else
LD_FILE := -Tetc/ld/lpc4337_m4_lib.ld -Tetc/ld/lpc4337_m4_mem.ld \
           -Tetc/ld/lpc4337_m4_RAM.ld
endif

# OpenOCD configuration file
CFG_FILE := etc/openocd/lpc4337.cfg

# Flash base address for OpenOCD download rule
BASE_ADDR := 0x1A000000

# Download command
DOWNLOAD_CMD := openocd -f $(CFG_FILE) -c "init" -c "halt 0" -c "flash write_image erase unlock $(OUT_PATH)/$(PROJECT_NAME).bin $(BASE_ADDR) bin" -c "reset run" -c "shutdown"

# Erase command
ERASE_CMD := openocd -f $(CFG_FILE) -c "init" -c "halt 0" -c "flash erase_sector 0 0 last" -c "exit"

####JM FIN #####




SYMBOLS += -D$(TARGET) -D$(BOARD)


#JM Incluye los makefiles de los módulos que requiere el proyecto.
#include $(foreach MOD,$(PROJECT_MODULES),$(MOD)/Makefile)

#Modulo BASE
base_PATH := modules/lpc4337_m4/base
base_SRC_FILES := $(wildcard $(base_PATH)/src/*.c) \
                  $(wildcard $(base_PATH)/src/*.S)
base_INC_FOLDERS := $(base_PATH)/inc
base_SRC_FOLDERS := $(base_PATH)/src

#Modulo DSP
dsp_PATH := modules/lpc4337_m4/dsp

dsp_INC_FOLDERS := $(dsp_PATH)/inc
EXTERN_LIB_FOLDERS += $(dsp_PATH)/lib
EXTERN_LIBS += arm_cortexM4lf_math

#Modulo Chip

chip_PATH := modules/lpc4337_m4/chip
chip_SRC_FILES := $(wildcard $(chip_PATH)/src/*.c)
chip_INC_FOLDERS := $(chip_PATH)/inc $(chip_PATH)/inc/usbd
chip_SRC_FOLDERS := $(chip_PATH)/src


#Modulo drivers_microcontroller

drivers_microcontroller_PATH := modules/lpc4337_m4/drivers_microcontroller
drivers_microcontroller_SRC_FILES := $(wildcard $(drivers_microcontroller_PATH)/src/*.c)
drivers_microcontroller_INC_FOLDERS := $(drivers_microcontroller_PATH)/inc
drivers_microcontroller_SRC_FOLDERS := $(drivers_microcontroller_PATH)/src

drivers_devices_PATH := modules/lpc4337_m4/drivers_devices
drivers_devices_SRC_FILES := $(wildcard $(drivers_devices_PATH)/src/*.c)
drivers_devices_INC_FOLDERS := $(drivers_devices_PATH)/inc
drivers_devices_SRC_FOLDERS := $(drivers_devices_PATH)/src

#####JM FIN#######


PROJECT_OBJ_FILES := $(addprefix $(OBJ_PATH)/,$(notdir $(PROJECT_C_FILES:.c=.o)))

PROJECT_OBJ_FILES += $(addprefix $(OBJ_PATH)/,$(notdir $(PROJECT_ASM_FILES:.S=.o)))

PROJECT_OBJS := $(notdir $(PROJECT_OBJ_FILES))

INCLUDES := $(addprefix -I,$(PROJECT_INC_FOLDERS)) \
            $(addprefix -I,$(foreach MOD,$(notdir $(PROJECT_MODULES)),$($(MOD)_INC_FOLDERS)))

vpath %.o $(OBJ_PATH)
vpath %.c $(PROJECT_SRC_FOLDERS) $(foreach MOD,$(notdir $(PROJECT_MODULES)),$($(MOD)_SRC_FOLDERS))
vpath %.S $(PROJECT_SRC_FOLDERS) $(foreach MOD,$(notdir $(PROJECT_MODULES)),$($(MOD)_SRC_FOLDERS))
vpath %.a $(OUT_PATH)

all:
	@echo "Build project..."
	@echo ""
	@make $(PROJECT_NAME)
	@echo ""
	@echo "Build complete."
	@echo ""



define makemod
lib$(1).a: $(2)
	@echo "*** archiving static library $(1) ***"
	@$(CROSS_PREFIX)ar rcs $(OUT_PATH)/lib$(1).a $(addprefix $(OBJ_PATH)/,$(2))
	@$(CROSS_PREFIX)size $(OUT_PATH)/lib$(1).a
endef

$(foreach MOD,$(notdir $(PROJECT_MODULES)), $(eval $(call makemod,$(MOD),$(notdir $(patsubst %.c,%.o,$(patsubst %.S,%.o,$($(MOD)_SRC_FILES)))))))

%.o: %.c
	@echo "*** compiling C file $< ***"
	@$(CROSS_PREFIX)gcc $(SYMBOLS) $(CFLAGS) $(INCLUDES) -c $< -o $(OBJ_PATH)/$@
	@$(CROSS_PREFIX)gcc $(SYMBOLS) $(CFLAGS) $(INCLUDES) -c $< -MM > $(OBJ_PATH)/$(@:.o=.d)

%.o: %.S
	@echo "*** compiling asm file $< ***"
	@$(CROSS_PREFIX)gcc $(SYMBOLS) $(CFLAGS) $(INCLUDES) -c $< -o $(OBJ_PATH)/$@
	@$(CROSS_PREFIX)gcc $(SYMBOLS) $(CFLAGS) $(INCLUDES) -c $< -MM > $(OBJ_PATH)/$(@:.o=.d)

-include $(wildcard $(OBJ_PATH)/*.d)

$(PROJECT_NAME): $(foreach MOD,$(notdir $(PROJECT_MODULES)),lib$(MOD).a) $(PROJECT_OBJS)
	@echo "*** linking project $@ ***"
	@$(CROSS_PREFIX)gcc $(LFLAGS) $(LD_FILE) -o $(OUT_PATH)/$(PROJECT_NAME).axf $(PROJECT_OBJ_FILES) $(SLAVE_OBJ_FILE) -L$(OUT_PATH) $(addprefix -l,$(notdir $(PROJECT_MODULES))) $(addprefix -L,$(EXTERN_LIB_FOLDERS)) $(addprefix -l,$(notdir $(EXTERN_LIBS)))
	@$(CROSS_PREFIX)size $(OUT_PATH)/$(PROJECT_NAME).axf
	@$(CROSS_PREFIX)objcopy -v -O binary $(OUT_PATH)/$(PROJECT_NAME).axf $(OUT_PATH)/$(PROJECT_NAME).bin
	@echo "*** post-build ***"
	@$(POST_BUILD_CMD)

clean:
	@echo "Clean project..."
	@echo ""
	rm -f $(OBJ_PATH)/*.*
	rm -f $(OUT_PATH)/*.*
	rm -f *.launch
	@echo ""
	@echo "Clean complete."
	@echo ""

clean_all:
	@make TARGET=lpc4337_m4 clean --no-print-directory

openocd:
	@echo "Starting OpenOCD for $(TARGET)..."
	@openocd -f $(CFG_FILE)

download: $(PROJECT_NAME)
	@echo "Downloading $(PROJECT_NAME).bin to $(TARGET)..."
	@$(DOWNLOAD_CMD)
	@echo "Download done."

erase:
	@echo "Erasing flash memory..."
	@$(ERASE_CMD)
	@echo "Erase done."

info:
	@echo PROJECT_NAME: $(PROJECT_NAME)
	@echo TARGET: $(TARGET)
	@echo PROJECT_MODULES: $(PROJECT_MODULES)
	@echo OBJS: $(PROJECT_OBJS)
	@echo INCLUDES: $(INCLUDES)
	@echo PROJECT_SRC_FOLDERS: $(PROJECT_SRC_FOLDERS)

.DEFAULT: all

.PHONY: all doc clean clean_all openocd download erase info ctags generate
