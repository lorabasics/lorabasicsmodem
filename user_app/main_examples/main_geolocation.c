/*!
 * \file      main_geolocation.c
 *
 * \brief     main program for Geolacation example
 *
 * Revised BSD License
 * Copyright Semtech Corporation 2020. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */
#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "main.h"

#include "smtc_modem_api.h"
#include "smtc_modem_utilities.h"

#include "example_options.h"

#include "smtc_hal_mcu.h"
#include "smtc_hal_gpio.h"

#include "modem_pinout.h"

#include "lr1110_types.h"
#include "lr1110_wifi.h"
#include "lr1110_gnss.h"
#include "lr1110_types.h"
#include "lr1110_system.h"

#include "wifi_scan.h"
#include "gnss.h"

// control to enable gnss scan and wifi scan
#define ENABLE_GNSS_SCAN
#define ENABLE_WIFI_SCAN

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/**
 * Stack id value (multistacks modem is not yet available)
 */
#define STACK_ID 0

/**
 * Scan alarm period
 */
#define SCAN_PERIOD_S 15

/**
 * Waiting time after reset
 */
#define WAIT_AFTER_RESET_TIME_S 30

/**
 * Modem port use for stream pipeline
 */
#define STREAM_PORT 199

/**
 * @brief Stack credentials
 */
static const uint8_t user_dev_eui[8]  = USER_LORAWAN_DEVICE_EUI;
static const uint8_t user_join_eui[8] = USER_LORAWAN_JOIN_EUI;
static const uint8_t user_app_key[16] = USER_LORAWAN_APP_KEY;

/**
 * @brief Assisted position for GNSS scan
 */
static const lr1110_gnss_solver_assistance_position_t assistance_position = {
    .latitude  = MODEM_EXAMPLE_ASSISTANCE_POSITION_LAT,
    .longitude = MODEM_EXAMPLE_ASSISTANCE_POSITION_LONG,
};
static const char* assistance_position_text = MODEM_EXAMPLE_ASSISTANCE_POSITION_TEXT;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static uint8_t rx_payload[255] = { 0 };  // Buffer for rx payload
static uint8_t rx_payload_size = 0;      // Size of the payload in the rx_payload buffer
#if defined( HYBRID_CN470_MONO_CHANNEL )
static uint8_t custom_datarate[16];
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

static void get_event( void );

static bool lr1110_configure_for_scan( void );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

extern wifi_t wifi;

#if defined( HYBRID_CN470_MONO_CHANNEL )
extern uint32_t freq_tx_cn470_mono_channel_mhz;
#endif

/**
 * @brief Main function about how to perform WiFi scan and GNSS scan, and send the packets out by streaming
 *
 */
void main_geolocation( void )
{
    // Disable IRQ to avoid unwanted behaviour during init
    hal_mcu_disable_irq( );

    // Configure all the µC periph (clock, gpio, timer, ...)
    hal_mcu_init( );

    // Init the modem and use get_event as event callback, please note that the callback will be
    // called immediately after the first call to modem_run_engine because of the reset detection
    smtc_modem_init( &get_event );

    // Re-enable IRQ
    hal_mcu_enable_irq( );

    SMTC_MODEM_HAL_TRACE_INFO( "Tracker example is starting\n" );
    SMTC_MODEM_HAL_TRACE_INFO( "Assistance position has been set in %s\n", assistance_position_text );
    SMTC_MODEM_HAL_TRACE_INFO( "!! ALC sync service is required for this example !!\n" );
    SMTC_MODEM_HAL_TRACE_INFO( "!! You need to be in an outdoor environnement !!\n" );

    // update local almanac data from head file such as "almanac_2020_12_22.h"
    // user can put the latest almanac data in the head file for the node to update when it boots,
    // Or user can use the older one for testing OTA almanac date.
    smtc_modem_suspend_radio_access( );
    update_local_almanac( );
    smtc_modem_resume_radio_access( );

#if defined( HYBRID_CN470_MONO_CHANNEL )
    freq_tx_cn470_mono_channel_mhz = 470900000;
    for( uint8_t i = 0; i < 16; i++ )
    {
        custom_datarate[i] = 0x05;  // Force use of SF7
    }
    SMTC_MODEM_HAL_TRACE_INFO( "HYBRID China version will transmit at %d Hz\n", freq_tx_cn470_mono_channel_mhz );
#endif

    while( 1 )
    {
        // Execute modem runtime, this function must be recalled in
        // sleep_time_ms (max value, can be recalled sooner)
        uint32_t sleep_time = smtc_modem_run_engine( );

        // go to sleep (if low power is enabled)
        hal_mcu_set_sleep_for_ms( sleep_time );
    }
}
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */
/**
 * @brief User callback for modem event
 *
 *  This callback is called every time an event ( see modem_event_t ) appears in the modem.
 *  Several events may have to be read from the modem when this callback is called.
 */
static void get_event( void )
{
    SMTC_MODEM_HAL_TRACE_MSG_COLOR( "Get_event () callback\n", MODEM_HAL_DBG_TRACE_COLOR_BLUE );

    static smtc_modem_return_code_t stream_ready  = RC_UNKNOWN;
    static bool                     gnss_get_time = false;

    uint8_t  event_type        = SMTC_MODEM_NO_EVENT;
    uint8_t  count             = 0;
    uint8_t  event_data[255]   = { 0 };
    uint16_t event_data_length = 0;
    uint8_t  pending_event     = 0;
    uint8_t  stack_id          = STACK_ID;
    uint32_t gps_time          = 0;
    bool     status            = false;

    // Read modem event type and number
    smtc_modem_get_event( &event_type, &count, &stack_id, event_data, &event_data_length, &pending_event );

    // Continue to read modem event until all events have been processed
    while( pending_event > 0 )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "Event %u\n", event_type );
        switch( event_type )
        {
        case SMTC_MODEM_EVENT_RESET:
            // Set user credentials
            smtc_modem_set_deveui( stack_id, user_dev_eui );
            smtc_modem_set_joineui( stack_id, user_join_eui );
            smtc_modem_set_nwkkey( stack_id, user_app_key );
            // Set user region
            smtc_modem_set_region( stack_id, MODEM_EXAMPLE_REGION );
            // Join LoRaWAN network
            smtc_modem_join_network( stack_id );
#if defined( ENABLE_GNSS_SCAN )
            smtc_modem_suspend_radio_access( );
            gnss_init( assistance_position );
            smtc_modem_resume_radio_access( );
#endif
            break;
        case SMTC_MODEM_EVENT_ALARM:
            SMTC_MODEM_HAL_TRACE_INFO( "User alarm events\n" );
            smtc_modem_suspend_radio_access( );
            status = lr1110_configure_for_scan( );
            if( status == true )
            {
#if defined( ENABLE_GNSS_SCAN )
                if( gnss_get_time == true && stream_ready == RC_OK )
                {
                    SMTC_MODEM_HAL_TRACE_INFO( "start to scan and stream gnss\n" );
                    if( scan_and_stream_gnss( stack_id, STREAM_PORT ) == false )
                    {
                        if( smtc_modem_start_alarm_timer( SCAN_PERIOD_S ) != RC_OK )
                        {
                            SMTC_MODEM_HAL_TRACE_ERROR( "Failed to set an alarm\n" );
                        }
                    }
                }
                else
                {
                    SMTC_MODEM_HAL_TRACE_WARNING(
                        "Stream not ready or GPS time not updated. GNSS data won't be sent\n" );
                }
#endif

#if defined( ENABLE_WIFI_SCAN )
                if( stream_ready == RC_OK )
                {
                    SMTC_MODEM_HAL_TRACE_INFO( "start to scan and stream wifi\n" );
                    wifi_init( );
                    if( scan_and_stream_wifi( stack_id, STREAM_PORT ) == false )
                    {
                        if( smtc_modem_start_alarm_timer( SCAN_PERIOD_S ) != RC_OK )
                        {
                            SMTC_MODEM_HAL_TRACE_ERROR( "Failed to set an alarm\n" );
                        }
                    }
                }
                else
                {
                    SMTC_MODEM_HAL_TRACE_WARNING( "Stream not ready. Wifi data won't be sent\n" );
                }
#endif
            }
            else
            {
                SMTC_MODEM_HAL_TRACE_ERROR( "Fail to configure lr1110 and no scan\n" );
            }

            smtc_modem_resume_radio_access( );
            break;

        case SMTC_MODEM_EVENT_JOINED:
            SMTC_MODEM_HAL_TRACE_INFO( "Modem is join\n" );
#if defined( HYBRID_CN470_MONO_CHANNEL )
            // Define a custom ADR (only for china hybrid)
            smtc_modem_set_adr_profile( stack_id, SMTC_MODEM_ADR_PROFILE_CUSTOM, custom_datarate );
            smtc_modem_set_connection_timeout_thresholds( stack_id, 0, 0 );
#endif

#if defined( ENABLE_GNSS_SCAN )
            // Start ALC sync service - required for gnss assisted scan
            smtc_modem_set_alcsync_mode( 1 );

            // Set dm interval to 1 hour for almanac update
            smtc_modem_set_dm_info_interval( SMTC_MODEM_DM_INFO_INTERVAL_IN_MINUTE, DM_INTERVAL_USER_MIN );

            // Active almanac update OTA - WARNING: will remove all other DM message
            uint8_t info_field = SMTC_MODEM_DM_FIELD_ALMANAC_STATUS;
            smtc_modem_set_dm_info_fields( &info_field, 1 );
#endif

            // Init a stream for WiFi or GNSS result
            stream_ready = smtc_modem_stream_init( stack_id, STREAM_PORT, SMTC_MODEM_STREAM_NO_CIPHER );

            if( smtc_modem_start_alarm_timer( WAIT_AFTER_RESET_TIME_S ) != RC_OK )
            {
                SMTC_MODEM_HAL_TRACE_ERROR( "Failed to set an alarm\n" );
            }
            break;

        case SMTC_MODEM_EVENT_TXDONE:
            SMTC_MODEM_HAL_TRACE_INFO( "TX done\n" );
            break;

        case SMTC_MODEM_EVENT_DOWNDATA:
            rx_payload_size = event_data_length - 4;
            memcpy( rx_payload, &event_data[4], rx_payload_size );
            SMTC_MODEM_HAL_TRACE_PRINTF( "Data received on port %u\n", event_data[3] );
            SMTC_MODEM_HAL_TRACE_ARRAY( "DOWNDATA", rx_payload, rx_payload_size );
            break;

        case SMTC_MODEM_EVENT_FILEDONE:
            break;

        case SMTC_MODEM_EVENT_SETCONF:
            break;

        case SMTC_MODEM_EVENT_MUTE:
            break;

        case SMTC_MODEM_EVENT_STREAMDONE:
            SMTC_MODEM_HAL_TRACE_INFO( "Stream complete\n" );
            if( smtc_modem_start_alarm_timer( SCAN_PERIOD_S ) != RC_OK )
            {
                SMTC_MODEM_HAL_TRACE_ERROR( "Failed to set an alarm\n" );
            }
            break;

        case SMTC_MODEM_EVENT_LINKSTATUS:
            break;

        case SMTC_MODEM_EVENT_JOINFAIL:
            SMTC_MODEM_HAL_TRACE_WARNING( "Join fail\n" );
            break;

        case SMTC_MODEM_EVENT_TIME_UPDATED:
            smtc_modem_get_time( &gps_time );

            if( gps_time == 0 )
            {
                gnss_get_time = false;
                SMTC_MODEM_HAL_TRACE_WARNING( "Time updated - Sync lost\n" );
            }
            else
            {
                gnss_get_time = true;
                SMTC_MODEM_HAL_TRACE_INFO( "Time updated - Sync OK\n" );
            }
            break;

        case SMTC_MODEM_EVENT_TIMEOUT_ADR_CHANGED:
            break;

        case SMTC_MODEM_EVENT_NEW_LINK_ADR:
            break;

        default:
            SMTC_MODEM_HAL_TRACE_ERROR( "Unknown event %u\n", event_type );
            break;
        }

        smtc_modem_get_event( &event_type, &count, &stack_id, event_data, &event_data_length, &pending_event );
    }
}

static bool lr1110_configure_for_scan( void )
{
    lr1110_status_t status;

    status = lr1110_system_set_reg_mode( NULL, LR1110_SYSTEM_REG_MODE_DCDC );
    if( status != LR1110_STATUS_OK )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Fail to set reg mode\n" );
        return false;
    }

    const lr1110_system_rfswitch_cfg_t cfg = {
        .enable  = 15,
        .standby = 0,
        .tx      = 3,
        .tx_hp   = 0,
        .tx_hf   = 0,
        .rx      = 1,
        .gnss    = 4,
        .wifi    = 8,
    };

    status = lr1110_system_set_dio_as_rf_switch( NULL, &cfg );
    if( status != LR1110_STATUS_OK )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Fail to config rf switch \n" );
        return false;
    }

    status = lr1110_system_set_tcxo_mode( NULL, LR1110_SYSTEM_TCXO_CTRL_3_0V, 500 );
    if( status != LR1110_STATUS_OK )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Fail to set tcxo mode\n" );
        return false;
    }

    status = lr1110_system_cfg_lfclk( NULL, LR1110_SYSTEM_LFCLK_XTAL, true );
    if( status != LR1110_STATUS_OK )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Fail to config lfclk\n" );
        return false;
    }

    status = lr1110_system_clear_errors( NULL );
    if( status != LR1110_STATUS_OK )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Fail to clear error\n" );
        return false;
    }

    status = lr1110_system_calibrate( NULL, 0x3F );
    if( status != LR1110_STATUS_OK )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Fail to calibrate\n" );
        return false;
    }

    return true;
}

/* --- EOF ------------------------------------------------------------------ */
