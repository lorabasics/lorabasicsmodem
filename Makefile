# ------------------------------------------------
# Generic Makefile (based on gcc)
#
# ------------------------------------------------

-include printing.mk

######################################
# Options
######################################

#use stml073 µc and board
BOARD_L073        = 1

######################################
# target names
######################################
TARGET_MODEM_2_4 = soft_modem_2g4

######################################
# building variables
######################################

# optimization
OPT = -Os -g

# testbench compilation specificity
TESTBENCH := $(if $(filter testbench,$(MAKECMDGOALS)),1,0)

# get hw_modem compile option
HW_MODEM := $(if $(filter hard_modem,$(MAKECMDGOALS)),1,0)

PERF_TEST := $(if $(filter perf_test,$(MAKECMDGOALS)),1,0)

#######################################
# Git information
# Thanks to https://nullpointer.io/post/easily-embed-version-information-in-software-releases/
#######################################

ifneq (, $(shell which git))
GIT_VERSION := $(shell git --no-pager describe --tags --always)
GIT_COMMIT  := $(shell git rev-parse --verify HEAD)
GIT_DATE    := $(firstword $(shell git --no-pager show --date=iso-strict --format="%ad" --name-only))
BUILD_DATE  := $(shell date --iso=seconds)

# If working tree is dirty, append dirty flag
ifneq ($(strip $(shell git status --porcelain 2>/dev/null)),)
 GIT_VERSION := $(GIT_VERSION)--dirty
endif
endif

#######################################
# paths
#######################################
# Build path
BUILD_DIR_MODEM_2_4 = build_modem_2_4

######################################
# source
######################################

# Version source - always recompiled to have the correct timestamp
C_SOURCE_VERSION = user_app/git_version.c

# Common C sources
COMMON_C_SOURCES =  \
user_app/main.c \
user_app/main_exti.c \
user_app/main_alarm_file_upload.c \
$(C_SOURCE_VERSION) \
smtc_modem_core/modem_api.c\
smtc_modem_core/lorawan_api/lorawan_api.c\
smtc_modem_core/device_management/dm_downlink.c \
smtc_modem_core/device_management/modem_context.c\
smtc_modem_core/modem_services/file_upload.c\
smtc_modem_core/modem_services/modem_utilities.c \
smtc_modem_core/modem_supervisor/modem_supervisor.c\
smtc_modem_core/test_mode/test_mode.c\
lr1mac/src/lr1_stack_mac_layer.c\
lr1mac/src/lr1mac_core.c\
lr1mac/src/lr1mac_utilities.c\
lr1mac/src/smtc_real/src/smtc_real.c\
smtc_crypto/src/aes.c\
smtc_crypto/src/cmac.c\
smtc_crypto/src/crypto.c\
smtc_ral/src/ral.c\
radio_planner/src/radio_planner.c

ifeq ($(HW_MODEM),1)
COMMON_C_SOURCES += \
user_app/hw_modem.c\
user_app/cmd_parser.c
endif

ifeq ($(TESTBENCH),1)
COMMON_C_SOURCES += \
user_app/hw_modem.c\
user_app/cmd_parser.c
endif


ifeq ($(BOARD_L073),1)
COMMON_C_SOURCES += \
smtc_bsp/arm/stm32/stm32_hal/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_rng.c\
smtc_bsp/arm/stm32/stm32_hal/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_i2c.c \
smtc_bsp/arm/stm32/stm32_hal/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_i2c_ex.c \
smtc_bsp/arm/stm32/stm32_hal/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_lptim.c \
smtc_bsp/arm/stm32/stm32_hal/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_rtc.c \
smtc_bsp/arm/stm32/stm32_hal/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_rtc_ex.c \
smtc_bsp/arm/stm32/stm32_hal/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_spi.c \
smtc_bsp/arm/stm32/stm32_hal/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_tim.c \
smtc_bsp/arm/stm32/stm32_hal/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_tim_ex.c \
smtc_bsp/arm/stm32/stm32_hal/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_uart.c \
smtc_bsp/arm/stm32/stm32_hal/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_uart_ex.c \
smtc_bsp/arm/stm32/stm32_hal/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_wwdg.c \
smtc_bsp/arm/stm32/stm32_hal/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal.c \
smtc_bsp/arm/stm32/stm32_hal/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_rcc.c \
smtc_bsp/arm/stm32/stm32_hal/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_rcc_ex.c \
smtc_bsp/arm/stm32/stm32_hal/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_flash.c \
smtc_bsp/arm/stm32/stm32_hal/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_flash_ex.c \
smtc_bsp/arm/stm32/stm32_hal/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_flash_ramfunc.c \
smtc_bsp/arm/stm32/stm32_hal/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_gpio.c \
smtc_bsp/arm/stm32/stm32_hal/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_dma.c \
smtc_bsp/arm/stm32/stm32_hal/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_pwr.c \
smtc_bsp/arm/stm32/stm32_hal/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_pwr_ex.c \
smtc_bsp/arm/stm32/stm32_hal/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_cortex.c \
smtc_bsp/arm/stm32/stm32_hal/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_adc.c \
smtc_bsp/arm/stm32/stm32l0xx/smtc_bsp_gpio.c\
smtc_bsp/arm/stm32/stm32l0xx/smtc_bsp_nvm.c\
smtc_bsp/arm/stm32/stm32l0xx/smtc_bsp_rng.c\
smtc_bsp/arm/stm32/stm32l0xx/smtc_bsp_rtc.c\
smtc_bsp/arm/stm32/stm32l0xx/smtc_bsp_adc.c\
smtc_bsp/arm/stm32/stm32l0xx/smtc_bsp_spi.c\
smtc_bsp/arm/stm32/stm32l0xx/smtc_bsp_tmr.c\
smtc_bsp/arm/stm32/stm32l0xx/smtc_bsp_uart.c\
smtc_bsp/arm/stm32/stm32l0xx/smtc_bsp_watchdog.c\
user_app/bsp_specific/bsp_radio_planner.c\
user_app/bsp_specific/ral_hal.c\
user_app/bsp_specific/smtc_bsp_mcu.c\
user_app/mcu_core/system_stm32l0xx.c
endif

# Region specific C sources
MODEM_2_4_C_SOURCES +=  \
sx1280_driver/src/sx1280.c\
smtc_ral/src/ral_sx1280.c\
user_app/bsp_specific/sx1280_hal.c\
lr1mac/src/smtc_real/src/region_ww2g4.c

# Common ASM sources
ifeq ($(BOARD_L073),1)
    ASM_SOURCES =  \
    user_app/mcu_core/startup_stm32l073xx.s
endif

#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
CC  = $(GCC_PATH)/$(PREFIX)gcc
CPP = $(GCC_PATH)/$(PREFIX)g++
AS  = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP  = $(GCC_PATH)/$(PREFIX)objcopy
SZ  = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
CPP = $(PREFIX)g++
AS = $(PREFIX)g++ -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

#######################################
# CFLAGS
#######################################
# cpu
ifeq ($(BOARD_L073),1)
    CPU = -mcpu=cortex-m0plus
endif

MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS =

# common C defines
ifeq ($(BOARD_L073),1)
    COMMON_C_DEFS =  \
    -DUSE_HAL_DRIVER \
    -DBOARD_L0\
    -DSTM32L073xx
endif

COMMON_C_DEFS += \
    -DGIT_VERSION=\"$(GIT_VERSION)\" \
    -DGIT_COMMIT=\"$(GIT_COMMIT)\" \
    -DGIT_DATE=\"$(GIT_DATE)\" \
    -DBUILD_DATE=\"$(BUILD_DATE)\"

ifeq ($(HW_MODEM),1)
    COMMON_C_DEFS += \
	-DHW_MODEM_ENABLED
endif

ifeq ($(TESTBENCH),1)
    COMMON_C_DEFS += \
	-DHW_MODEM_ENABLED\
	-DTEST_BYPASS_JOIN_DUTY_CYCLE
endif

ifeq ($(PERF_TEST),1)
    COMMON_C_DEFS += \
	-DPERF_TEST_ENABLED
endif


# region specific C defines

MODEM_2_4_C_DEFS += \
    -DSX1280 \
    -DREGION_WW2G4

vpath %.c $(sort $(dir $(C_SOURCES)))
vpath %.cpp $(sort $(dir $(CPP_SOURCES)))

# common C includes

ifeq ($(BOARD_L073),1)
COMMON_C_INCLUDES =  \
    -Ismtc_bsp/arm/cmsis\
    -Ismtc_bsp/arm/stm32/stm32_hal/STM32L0xx_HAL_Driver/Inc\
    -Ismtc_bsp/arm/stm32/stm32_hal/STM32L0xx_HAL_Driver/Inc/Legacy\
    -Ismtc_bsp/arm/stm32/stm32l0xx\
    -Ismtc_bsp\
    -Iuser_app\
    -Iuser_app/mcu_core\
    -Iuser_app/bsp_specific\
    -Ismtc_modem_core\
    -Ismtc_modem_core/modem_supervisor\
    -Ismtc_modem_core/device_management\
    -Ismtc_modem_core/modem_services\
    -Ismtc_modem_core/lorawan_api\
    -Ismtc_modem_core/test_mode\
    -Ismtc_ral/src\
    -Ilr1mac\
    -Ilr1mac/src\
    -Iradio_planner/src\
    -Ismtc_crypto/src\
    -Ilorawan_api\
    -Ilr1mac/src/smtc_real/src\
    -Isx1280_driver/src
endif

# region specific C includes
MODEM_2_4_C_INCLUDES =  \
    -Isx1280_driver/src

# compile gcc flags
WFLAG = -Wall -Wextra  -Wno-unused-parameter -Wpedantic -ffunction-sections -fdata-sections -fomit-frame-pointer -mabi=aapcs -fno-unroll-loops -ffast-math -ftree-vectorize $(BYPASS_FLAGS)

ASFLAGS = -fno-builtin $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) $(WFLAG)


#######################################
# LDFLAGS
#######################################
# link script
ifeq ($(BOARD_L073),1)
LDSCRIPT = user_app/mcu_core/stm32l073xx_flash.ld
endif
# libraries
LIBS =   -lstdc++ -lsupc++ -lm -lc -lnosys


LIBDIR =
LDFLAGS = $(MCU) --specs=nano.specs --specs=nosys.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,--cref -Wl,--gc-sections

# default action: build all
all: modem_2_4

modem_2_4: $(BUILD_DIR_MODEM_2_4)/$(TARGET_MODEM_2_4).elf $(BUILD_DIR_MODEM_2_4)/$(TARGET_MODEM_2_4).hex $(BUILD_DIR_MODEM_2_4)/$(TARGET_MODEM_2_4).bin
	$(call success,$@)

#######################################
# build the TARGET_MODEM_2_4 application
#######################################
SOURCES_2_4 = $(COMMON_C_SOURCES) $(MODEM_2_4_C_SOURCES)
CFLAGS_2_4 = -fno-builtin $(MCU) $(COMMON_C_DEFS) $(MODEM_2_4_C_DEFS) $(COMMON_C_INCLUDES) $(MODEM_2_4_C_INCLUDES) $(OPT) $(WFLAG) -MMD -MP -MF"$(@:%.o=%.d)"

# list of C objects
OBJECTS_2_4 = $(addprefix $(BUILD_DIR_MODEM_2_4)/,$(notdir $(SOURCES_2_4:.c=.o)))
vpath %.c $(sort $(dir $(SOURCES_2_4)))

# list of ASM program objects
OBJECTS_2_4 += $(addprefix $(BUILD_DIR_MODEM_2_4)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR_MODEM_2_4)/%.o: %.c Makefile | $(BUILD_DIR_MODEM_2_4)
	$(call build,'CC',$<)
	$(SILENT)$(CC) -c $(CFLAGS_2_4) -Wa,-a,-ad,-alms=$(BUILD_DIR_MODEM_2_4)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR_MODEM_2_4)/%.o: %.s Makefile | $(BUILD_DIR_MODEM_2_4)
	$(call build,'AS',$<)
	$(SILENT)$(AS) -c $(ASFLAGS) $< -o $@

$(BUILD_DIR_MODEM_2_4)/$(TARGET_MODEM_2_4).elf: $(OBJECTS_2_4) Makefile
	$(call build,'CC',$<)
	$(SILENT)$(CC) $(OBJECTS_2_4) $(LDFLAGS),-Map=$(BUILD_DIR_MODEM_2_4)/$(TARGET_MODEM_2_4).map -o $@
	$(SZ) $@

$(BUILD_DIR_MODEM_2_4)/%.hex: $(BUILD_DIR_MODEM_2_4)/%.elf | $(BUILD_DIR_MODEM_2_4)
	$(call build,'HEX',$@)
	$(SILENT)$(HEX) $< $@

$(BUILD_DIR_MODEM_2_4)/%.bin: $(BUILD_DIR_MODEM_2_4)/%.elf | $(BUILD_DIR_MODEM_2_4)
	$(call build,'BIN',$@)
	$(SILENT)$(BIN) $< $@

$(BUILD_DIR_MODEM_2_4):
	$(SILENT)mkdir $@

.PHONY: clean all test
.PHONY: flash
.PHONY: FORCE
FORCE:

#######################################
# Unit tests
#######################################
test:
	cd tests && ./run_tests.sh

#######################################
# Enable LoraWan bypass functionality
#######################################
hard_modem:
	$(call warn,"Building HW Modem")

testbench:
	$(call warn,"Building Modem for testbench")

perf_test:
	$(call warn,"Enabled perf test features")

#######################################
# Flash by copying on ST-Link mounted on WSL
#######################################
DEBUG_DRIVE_LETTER = e
DEBUG_DRIVE = /mnt/$(DEBUG_DRIVE_LETTER)
DEBUG_DRIVE_PRESENT = $(DEBUG_DRIVE)/DETAILS.TXT

$(DEBUG_DRIVE_PRESENT):
	#sudo mkdir -p $(DEBUG_DRIVE)
	sudo mount -t drvfs $(DEBUG_DRIVE_LETTER): $(DEBUG_DRIVE)

flash: $(DEBUG_DRIVE_PRESENT) $(BUILD_DIR_MODEM_2_4)/$(TARGET_MODEM_2_4).bin
	cp $(BUILD_DIR_MODEM_2_4)/$(TARGET_MODEM_2_4).bin $(DEBUG_DRIVE)/




#######################################
# clean up
#######################################
clean:
	-rm -fR $(BUILD_DIR_MODEM_2_4)




# *** EOF ***
