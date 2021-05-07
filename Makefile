##############################################################################
# Main makefile for basic_modem
##############################################################################

-include makefiles/printing.mk

#-----------------------------------------------------------------------------
# Global configuration options
#-----------------------------------------------------------------------------
# Prefix for all build directories
BUILD_ROOT = build_modem

# Prefix for all binaries names
TARGET_ROOT = basic_modem

# Target board
BOARD_L476 = 1

# - CHINA_DEMO -> Use Regional Parameters 1.0
# - HYBRID_CHINA -> RP 1.0, single channel
CHINA_DEMO = yes
HYBRID_CHINA ?= no

# Use multithreaded build (make -j)
MULTITHREAD ?= yes

# Print each object file size
SIZE ?= no

# Save memory usage to log file
LOG_MEM ?= yes

# Tranceiver
RADIO = lr1110

#-----------------------------------------------------------------------------
# default action: print help
#-----------------------------------------------------------------------------
help:
	$(call echo_help_b, "Build Basic Modem for LR1110")
	$(call echo_help, "")
	$(call echo_help_b, "-------------------------------- Clean -------------------------------------")
	$(call echo_help, " * make clean                      : clean all")
	$(call echo_help, "")
	$(call echo_help_b, "----------------------------- Compilation ----------------------------------")
	$(call echo_help, " * make basic_modem                : build basic_modem for a LR1110 transceiver")
	$(call echo_help, " * make all                        : build all targets")
	$(call echo_help, "")
	$(call echo_help_b, "---------------------------- All inclusive ---------------------------------")
	$(call echo_help, " * make full                       : clean and build basic_modem on LR1110")
	$(call echo_help, "")
	$(call echo_help_b, "------------------------- Optional parameters ------------------------------")
	$(call echo_help, " * HYBRID_CHINA=yes                : build hybrid china with monochannel region")
	$(call echo_help_b, "------------------------- Debug parameters ------------------------------")
	$(call echo_help, " * MULTITHREAD=no                  : Disable multithreaded build")
	$(call echo_help, " * VERBOSE=yes                     : Increase build verbosity")
	$(call echo_help, " * SIZE=yes                        : Display size for all objects")



#-----------------------------------------------------------------------------
# Makefile include selection
#-----------------------------------------------------------------------------
-include makefiles/lr1110.mk

#-----------------------------------------------------------------------------
-include makefiles/common.mk

.PHONY: clean all help 
.PHONY: FORCE
FORCE:

all: basic_modem

#-----------------------------------------------------------------------------
# Clean
#-----------------------------------------------------------------------------
clean:
	-rm -rf $(BUILD_ROOT)_*




