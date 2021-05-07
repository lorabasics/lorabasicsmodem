##############################################################################
# Common rules and definitions
##############################################################################

#-----------------------------------------------------------------------------
# Build system binaries
#-----------------------------------------------------------------------------
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

#-----------------------------------------------------------------------------
# Board selection
#-----------------------------------------------------------------------------
-include makefiles/board_L476.mk

#-----------------------------------------------------------------------------
# Define target build directory
#-----------------------------------------------------------------------------
TARGET_MODEM = $(TARGET_ROOT)_$(TARGET)
BUILD_DIR_MODEM = $(BUILD_ROOT)_$(TARGET)

#-----------------------------------------------------------------------------
# Multithread is disabled if verbose build, for readable logs
#-----------------------------------------------------------------------------
ifeq ($(VERBOSE),yes)
MULTITHREAD = no
endif
ifeq ($(SIZE),yes)
MULTITHREAD = no
endif

ifeq ($(MULTITHREAD),no)
MTHREAD_FLAG =
else
MTHREAD_FLAG = -j
endif

#-----------------------------------------------------------------------------
# Region selection.
#-----------------------------------------------------------------------------
REGION_CN_470_RP_1_0 = yes


#-----------------------------------------------------------------------------
# Optimizations
#-----------------------------------------------------------------------------
OPT = -Os -g

#-----------------------------------------------------------------------------
# Dump memory usage to a log file
#-----------------------------------------------------------------------------
ifeq ($(LOG_MEM), yes)
MEMLOG_FILE := $(BUILD_DIR_MODEM)/mem_usage.log
MEMLOG = | tee $(MEMLOG_FILE)
else
MEMLOG = 
endif

#-----------------------------------------------------------------------------
# Compilation flags
#-----------------------------------------------------------------------------
# Basic compilation flags
WFLAG += \
	-Wall \
	-Wextra \
	-Wno-unused-parameter \
	-Wpedantic \
	-fomit-frame-pointer \
	-mabi=aapcs \
	-fno-unroll-loops \
	-ffast-math \
	-ftree-vectorize 

# Allow linker to not link unused functions
WFLAG += \
	-ffunction-sections \
	-fdata-sections 

# Generate .su files for stack use analysis
WFLAG += -fstack-usage 

#Link-time optimization
#WFLAG += --lto 

# AS defines
AS_DEFS =

# Assembly flags
ASFLAGS += -fno-builtin $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) $(WFLAG)

COMMON_C_DEFS += \
	-DGIT_VERSION=\"$(GIT_VERSION)\" \
	-DGIT_COMMIT=\"$(GIT_COMMIT)\" \
	-DGIT_DATE=\"$(GIT_DATE)\" \
	-DBUILD_DATE=\"$(BUILD_DATE)\"

CFLAGS += -fno-builtin $(MCU) $(BOARD_C_DEFS) $(COMMON_C_DEFS) $(MODEM_C_DEFS) $(BOARD_C_INCLUDES) $(COMMON_C_INCLUDES) $(MODEM_C_INCLUDES) $(OPT) $(WFLAG) -MMD -MP -MF"$(@:%.o=%.d)"
CFLAGS += -falign-functions=4

#-----------------------------------------------------------------------------
# Link flags
#-----------------------------------------------------------------------------
# libraries
LIBS += -lstdc++ -lsupc++ -lm -lc -lnosys

LIBDIR =

LDFLAGS += $(MCU) 
LDFLAGS += --specs=nano.specs 
LDFLAGS += --specs=nosys.specs
LDFLAGS += -T$(BOARD_LDSCRIPT) $(LIBDIR) $(LIBS)
LDFLAGS += -Wl,--cref # Cross-reference table
LDFLAGS += -Wl,--print-memory-usage # Display ram/flash memory usage
LDFLAGS += -Wl,--gc-sections # Garbage collect unused sections


#-----------------------------------------------------------------------------
# User application sources
#-----------------------------------------------------------------------------
USER_APP_C_SOURCES += \
	user_app/main.c \
	user_app/git_version.c

COMMON_C_INCLUDES += \
	-Iuser_app/main_examples

#-----------------------------------------------------------------------------
# Common sources
#-----------------------------------------------------------------------------
SMTC_MODEM_CORE_C_SOURCES += \
	smtc_modem_core/lorawan_api/lorawan_api.c \
	smtc_modem_core/device_management/dm_downlink.c \
	smtc_modem_core/device_management/modem_context.c\
	smtc_modem_core/modem_core/smtc_modem.c\
	smtc_modem_core/modem_core/smtc_modem_test.c\
	smtc_modem_core/modem_services/fifo_ctrl.c\
	smtc_modem_core/modem_services/modem_utilities.c \
	smtc_modem_core/modem_services/smtc_modem_services_hal.c\
	smtc_modem_core/modem_supervisor/modem_supervisor.c

SMTC_MODEM_SERVICES_C_SOURCES += \
	smtc_modem_services/src/stream/stream.c\
	smtc_modem_services/src/stream/rose.c\
	smtc_modem_services/src/alc_sync/alc_sync.c\
	smtc_modem_services/src/file_upload/file_upload.c

LR1MAC_C_SOURCES += \
	lr1mac/src/lr1_stack_mac_layer.c\
	lr1mac/src/lr1mac_core.c\
	lr1mac/src/lr1mac_utilities.c\
	lr1mac/src/smtc_real/src/smtc_real.c\
	lr1mac/src/services/smtc_duty_cycle.c\
	lr1mac/src/services/smtc_lbt.c\
	lr1mac/src/lr1mac_class_c/lr1mac_class_c.c

SMTC_CRYPTO_C_SOURCES += \
	smtc_crypto/src/aes.c\
	smtc_crypto/src/cmac.c\
	smtc_crypto/src/smtc_crypto.c

RADIO_PLANNER_C_SOURCES += \
	radio_planner/src/radio_planner.c\
	radio_planner/src/radio_planner_hal.c

COMMON_C_INCLUDES +=  \
	-Iuser_app\
	-Iuser_app/main_examples/geolocation_utilities\
	-Iuser_app/radio_hal\
	-Ismtc_modem_core\
	-Ismtc_modem_core/modem_supervisor\
	-Ismtc_modem_core/device_management\
	-Ismtc_modem_core/modem_services\
	-Ismtc_modem_core/lorawan_api\
	-Ismtc_modem_services/headers\
	-Ismtc_modem_services/src\
	-Ismtc_modem_services/src/stream\
	-Ismtc_modem_services/src/file_upload\
	-Ismtc_modem_services/src/alc_sync\
	-Ismtc_modem_services\
	-Ismtc_modem_api\
	-Ismtc_ral/src\
	-Ismtc_ralf/src\
	-Ilr1mac\
	-Ilr1mac/src\
	-Ilr1mac/src/services\
	-Ilr1mac/src/lr1mac_class_c\
	-Iradio_planner/src\
	-Ismtc_crypto/src\
	-Ilorawan_api\
	-Ilr1mac/src/smtc_real/src

#-----------------------------------------------------------------------------
# Region sources and defines
#-----------------------------------------------------------------------------
LR1MAC_C_SOURCES += lr1mac/src/smtc_real/src/region_cn_470_rp_1_0.c
MODEM_C_DEFS += -DREGION_CN_470_RP_1_0

#-----------------------------------------------------------------------------
# Gather everything
#-----------------------------------------------------------------------------
C_SOURCES = \
	$(USER_APP_C_SOURCES) \
	$(BOARD_C_SOURCES) \
	$(RADIO_DRIVER_C_SOURCES) \
	$(SMTC_RAL_C_SOURCES) \
	$(SMTC_RALF_C_SOURCES) \
	$(RADIO_HAL_C_SOURCES) \
	$(RADIO_PLANNER_C_SOURCES) \
	$(SMTC_MODEM_CORE_C_SOURCES) \
	$(SMTC_MODEM_SERVICES_C_SOURCES) \
	$(SMTC_CRYPTO_C_SOURCES) \
	$(LR1MAC_C_SOURCES)

ASM_SOURCES = $(BOARD_ASM_SOURCES)

vpath %.c $(sort $(dir $(C_SOURCES)))
vpath %.cpp $(sort $(dir $(CPP_SOURCES)))

#-----------------------------------------------------------------------------
basic_modem:
ifeq ($(RADIO),nc)
	$(call echo_error,"No radio selected! Please specified the target radio using RADIO=sx128x or RADIO=sx1261 or RADIO=sx1262 or RADIO=lr1110")
else
	$(MAKE) basic_modem_build
endif 


basic_modem_build: $(BUILD_DIR_MODEM)/$(TARGET_MODEM).elf $(BUILD_DIR_MODEM)/$(TARGET_MODEM).hex $(BUILD_DIR_MODEM)/$(TARGET_MODEM).bin
	$(call success,$@)


#-----------------------------------------------------------------------------
# list of C objects
#-----------------------------------------------------------------------------
OBJECTS = $(addprefix $(BUILD_DIR_MODEM)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR_MODEM)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR_MODEM)/%.o: %.c Makefile | $(BUILD_DIR_MODEM)
	$(call build,'CC',$<)
	$(SILENT)$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR_MODEM)/$(notdir $(<:.c=.lst)) $< -o $@
ifeq ($(SIZE),yes)
	$(SZ) $@
endif

$(BUILD_DIR_MODEM)/%.o: %.s Makefile | $(BUILD_DIR_MODEM)
	$(call build,'AS',$<)
	$(SILENT)$(AS) -c $(ASFLAGS) $< -o $@
ifeq ($(SIZE),yes)
	$(SZ) $@
endif

$(BUILD_DIR_MODEM)/$(TARGET_MODEM).elf: $(OBJECTS) Makefile
	$(call build,'CC',$@)
	$(SILENT)$(CC) $(OBJECTS) $(LDFLAGS) -Wl,-Map=$(BUILD_DIR_MODEM)/$(TARGET_MODEM).map -o $@ $(MEMLOG)
	$(SZ) $@

$(BUILD_DIR_MODEM)/%.hex: $(BUILD_DIR_MODEM)/%.elf | $(BUILD_DIR_MODEM)
	$(call build,'HEX',$@)
	$(SILENT)$(HEX) $< $@

$(BUILD_DIR_MODEM)/%.bin: $(BUILD_DIR_MODEM)/%.elf | $(BUILD_DIR_MODEM)
	$(call build,'BIN',$@)
	$(SILENT)$(BIN) $< $@

$(BUILD_DIR_MODEM):
	$(SILENT)mkdir $@

#-----------------------------------------------------------------------------
# Debug print rules
#-----------------------------------------------------------------------------
debug_target:
	$(call echo,"Target $(TARGET)")
	$(call echo,"Build directory $(BUILD_DIR_MODEM)")
	$(call echo,"Binary $(TARGET_MODEM)")

debug_sources:
	$(call echo,"USER_APP_C_SOURCES	$(USER_APP_C_SOURCES)")
	$(call echo,"BOARD_C_SOURCES	$(BOARD_C_SOURCES)")
	$(call echo,"RADIO_DRIVER_C_SOURCES	$(RADIO_DRIVER_C_SOURCES)")
	$(call echo,"SMTC_RAL_C_SOURCES	$(SMTC_RAL_C_SOURCES)")
	$(call echo,"SMTC_RALF_C_SOURCES	$(SMTC_RALF_C_SOURCES)")
	$(call echo,"RADIO_HAL_C_SOURCES	$(RADIO_HAL_C_SOURCES)")
	$(call echo,"RADIO_PLANNER_C_SOURCES	$(RADIO_PLANNER_C_SOURCES)")
	$(call echo,"SMTC_MODEM_CORE_C_SOURCES	$(SMTC_MODEM_CORE_C_SOURCES)")
	$(call echo,"SMTC_MODEM_SERVICES_C_SOURCES	$(SMTC_MODEM_SERVICES_C_SOURCES)")
	$(call echo,"SMTC_CRYPTO_C_SOURCES	$(SMTC_CRYPTO_C_SOURCES)")
	$(call echo,"LR1MAC_C_SOURCES	$(LR1MAC_C_SOURCES)")
	$(call echo,"COMMON_C_INCLUDES	$(COMMON_C_INCLUDES)")

debug_flags:
	$(call echo,"MODEM_C_DEFS	$(MODEM_C_DEFS)")

debug: debug_target debug_sources debug_flags


#-----------------------------------------------------------------------------
# Clean
#-----------------------------------------------------------------------------
clean_target:
	-rm -fR $(BUILD_DIR_MODEM)

#-----------------------------------------------------------------------------
# Full
#-----------------------------------------------------------------------------
full:
	$(MAKE) clean_target
	$(MAKE) basic_modem $(MTHREAD_FLAG)
