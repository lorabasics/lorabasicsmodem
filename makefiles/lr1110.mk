##############################################################################
# Definitions for the LR1110 tranceiver
##############################################################################
TARGET = lr1110

#-----------------------------------------------------------------------------
# User application sources
#-----------------------------------------------------------------------------
USER_APP_C_SOURCES += \
	user_app/main_examples/main_geolocation.c \
	user_app/main_examples/geolocation_utilities/gnss.c \
	user_app/main_examples/geolocation_utilities/wifi_scan.c

#-----------------------------------------------------------------------------
# Common sources
#-----------------------------------------------------------------------------
SMTC_MODEM_SERVICES_C_SOURCES += \
	smtc_modem_services/src/almanac_update/almanac_update.c

RADIO_DRIVER_C_SOURCES += \
	lr1110_driver/src/lr1110_bootloader.c\
	lr1110_driver/src/lr1110_crypto_engine.c\
	lr1110_driver/src/lr1110_driver_version.c\
	lr1110_driver/src/lr1110_gnss.c\
	lr1110_driver/src/lr1110_radio.c\
	lr1110_driver/src/lr1110_regmem.c\
	lr1110_driver/src/lr1110_system.c\
	lr1110_driver/src/lr1110_wifi.c

SMTC_RAL_C_SOURCES += \
	smtc_ral/src/ral_lr1110.c

SMTC_RALF_C_SOURCES += \
	smtc_ralf/src/ralf_lr1110.c

RADIO_HAL_C_SOURCES += \
	user_app/radio_hal/lr1110_hal.c\
	user_app/radio_hal/ral_lr1110_bsp.c

#-----------------------------------------------------------------------------
# Includes
#-----------------------------------------------------------------------------
MODEM_C_INCLUDES =  \
	-Ilr1110_driver/src

#-----------------------------------------------------------------------------
# Region
#-----------------------------------------------------------------------------
REGION_CN_470_RP_1_0 = yes

MODEM_C_DEFS += \
	-DCHINA_RP_1_DEMO

ifeq ($(HYBRID_CHINA),yes)
MODEM_C_DEFS += \
    -DHYBRID_CN470_MONO_CHANNEL
endif # hybrid_china

#-----------------------------------------------------------------------------
# Radio specific compilation flags
#-----------------------------------------------------------------------------
MODEM_C_DEFS += \
	-DLR1110\
	-DLR1110_TRANSCEIVER\
	-D_GNSS_SNIFF_ENABLE


