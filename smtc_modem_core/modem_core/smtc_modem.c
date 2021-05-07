/*!
 * \file      smtc_modem.c
 *
 * \brief     modem implementation (functions of generic api, extension, test and utilities)
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

#include "smtc_modem_api.h"
#include "smtc_modem_test_api.h"

#include "modem_supervisor.h"
#include "modem_context.h"
#include "smtc_real_defs.h"
#include "lorawan_api.h"
#include "alc_sync.h"
#include "file_upload.h"
#include "stream.h"
#include "radio_planner.h"
#include "ral.h"
#include "smtc_modem_utilities.h"

#include "modem_config.h"
#include "ralf.h"

#if defined( LR1110_MODEM )
#include "smtc_modem_e_api_extension.h"
#include "smtc_rtc_compensation.h"
#include "pool_mem.h"
#include "smtc_crypto_se.h"
#if defined( _WIFI_SNIFF_ENABLE )
#include "wifi_ctrl_api.h"
#endif  // _WIFI_SNIFF_ENABLE
#if defined( _GNSS_SNIFF_ENABLE )
#include "gnss_ctrl_api.h"
#endif  //_GNSS_SNIFF_ENABLE
#else
#include "smtc_basic_modem_lr1110_api_extension.h"
#endif  // LR1110_MODEM

#if defined( SX128X )
#include "ralf_sx128x.h"
#elif defined( SX126X )
#include "ralf_sx126x.h"
#elif defined( LR1110 )
#include "ralf_lr1110.h"
#endif
#include "modem_utilities.h"
#include "smtc_crypto.h"

#if defined( USE_LR1110_SE )
#include "lr1110_system.h"
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */
#define FW_VERSION_MAJOR 1
#define FW_VERSION_MINOR 1
#define FW_VERSION_PATCH 7

#define LR_VERSION_MAJOR 1
#define LR_VERSION_MINOR 0
#define LR_VERSION_PATCH 3

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */
#if defined( SX128X )
const ralf_t modem_radio = RALF_SX128X_INSTANTIATE( NULL );
#elif defined( SX126X )
const ralf_t modem_radio = RALF_SX126X_INSTANTIATE( NULL );
#elif defined( LR1110 )
const ralf_t modem_radio = RALF_LR1110_INSTANTIATE( NULL );
#else
#error "Please select radio board.."
#endif

#if !defined( LR1110_MODEM )
static uint8_t   modem_buffer[242];
static uint32_t* upload_pdata;
static uint32_t  upload_size;

static radio_planner_t modem_radio_planner = {
    .radio = &modem_radio,
};

smtc_modem_services_t smtc_modem_services_ctx;

// LBT temporary configuration save
static uint32_t lbt_config_listen_duration_ms = 0;
static int16_t  lbt_config_threshold_dbm      = 0;
static uint32_t lbt_config_bw_hz              = 0;
static bool     lbt_config_available          = false;

#ifdef LORAWAN_BYPASS_ENABLED
static bool stream_bypass_enabled = false;
#endif  // LORAWAN_BYPASS_ENABLED

#else  // !defined( LR1110_MODEM )

uint8_t modem_buffer[242];
struct
{
    uint32_t* upload_pdata;
    uint32_t  upload_size;
    uint32_t  upload_spare;
#ifdef LORAWAN_BYPASS_ENABLED
    bool      stream_bypass_enabled = false;
#endif  // LORAWAN_BYPASS_ENABLED
} modem_api_context;

// clang-format off
#define upload_pdata                modem_api_context.upload_pdata
#define upload_size                 modem_api_context.upload_size
#ifdef LORAWAN_BYPASS_ENABLED
#define stream_bypass_enabled       modem_api_context.stream_bypass_enabled
#endif  // LORAWAN_BYPASS_ENABLED
// clang-format on

radio_planner_t       modem_radio_planner;
smtc_modem_services_t smtc_modem_services_ctx;

#endif  // !defined( LR1110_MODEM )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */
bool modem_port_reserved( uint8_t f_port );

smtc_modem_return_code_t smtc_modem_get_dm_status_with_rate( uint8_t* dm_fields_payload, uint8_t* dm_field_length,
                                                             e_dm_info_rate_t rate );

smtc_modem_return_code_t smtc_modem_set_dm_status_with_rate( const uint8_t* dm_fields_payload, uint8_t dm_field_length,
                                                             e_dm_info_rate_t rate );

bool is_modem_connected( );

smtc_modem_return_code_t smtc_modem_send_tx( uint8_t f_port, bool confirmed, const uint8_t* payload,
                                             uint8_t payload_length, bool emergency );

void empty_callback( void* ctx );
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

/* ------------ Modem Utilities ------------*/

void smtc_modem_init( void ( *callback )( void ) )
{
#ifdef LORAWAN_BYPASS_ENABLED
    stream_bypass_enabled = false;
#endif
    // init radio and put it in sleep mode
    ral_reset( &modem_radio.ral );
    ral_init( &modem_radio.ral );
    ral_set_sleep( &modem_radio.ral, true );

    // init radio planner and attach corresponding radio irq
    rp_init( &modem_radio_planner, &modem_radio );

#if !defined( LR1110_MODEM )
    smtc_modem_hal_irq_config_radio_irq( rp_radio_irq_callback, &modem_radio_planner );
#endif

    // init modem supervisor

#if defined( LR1110_MODEM )
    smtc_rtc_compensation_init( &modem_radio_planner, RTC_CLK_COMPENSATION_ID_RP );
#endif
    rp_hook_init( &modem_radio_planner, 0, ( void ( * )( void* ) )( empty_callback ), NULL );
    modem_supervisor_init( callback, &modem_radio_planner, &smtc_modem_services_ctx );
}

uint32_t smtc_modem_run_engine( void )
{
    uint8_t nb_downlink = fifo_ctrl_get_nb_elt( lorawan_api_get_fifo_obj( ) );

    if( nb_downlink > 0 )
    {
        increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_DOWNDATA, 0 );
        set_modem_event_count_and_status( SMTC_MODEM_EVENT_DOWNDATA, nb_downlink, 0 );
    }

    return modem_supervisor_engine( );
}

lr1mac_states_t smtc_modem_get_lr1mac_state( void )
{
    lr1mac_states_t temp = lorawan_api_state_get( );
    if( ( temp == LWPSTATE_IDLE ) || ( temp == LWPSTATE_TX_WAIT ) )  // second test for the case of nbtrans> 0
    {
        return LWPSTATE_IDLE;
    }
    else
    {
        return LWPSTATE_BUSY;
    }
}

/* ------------ Modem Generic Api ------------*/

smtc_modem_return_code_t smtc_modem_get_event( uint8_t* type, uint8_t* count, uint8_t* stack_id, uint8_t* data,
                                               uint16_t* data_length, uint8_t* pending_count )

{
    if( modem_get_test_mode_status( ) == true )  // Check if modem is in test mode
    {
        return RC_BUSY;
    }

    smtc_modem_return_code_t return_code = RC_OK;
    uint8_t                  msg_event   = get_last_msg_event( );
    *data_length                         = 0;

    *pending_count = get_asynchronous_msgnumber( );
    if( *pending_count > MODEM_NUMBER_OF_EVENTS )
    {
        smtc_modem_hal_mcu_panic( "asynchronous_msgnumber overlap" );
    }
    else if( *pending_count > 0 )
    {
        *type = msg_event;

        // count store the number of event of current type including missed event in case of overrun
        *count = get_modem_event_count( msg_event );
        SMTC_MODEM_HAL_TRACE_PRINTF( "MESSAGE EVENT %d\n", msg_event );
        // insert the count field in case of overrun
        switch( msg_event )
        {
        case SMTC_MODEM_EVENT_JOINED:
            *data_length = 0;
            break;
        case SMTC_MODEM_EVENT_RESET:  // count in case of overrun should never append because reset !!!!
            *data_length = 2;
            data[0]      = ( ( lorawan_api_nb_reset_get( ) ) >> 8 ) & 0xFF;
            data[1]      = ( lorawan_api_nb_reset_get( ) ) & 0xFF;
            break;
        case SMTC_MODEM_EVENT_ALARM:
            *data_length = 0;
            break;
        case SMTC_MODEM_EVENT_DOWNDATA: {
            uint8_t                buffer[242];
            uint16_t               data_len;
            lr1mac_down_metadata_t meta;
            uint8_t                metadata_len;
            fifo_ctrl_get( lorawan_api_get_fifo_obj( ), buffer, &data_len, 242, &meta, &metadata_len,
                           sizeof( lr1mac_down_metadata_t ) );

            if( ( meta.rx_rssi >= -128 ) && ( meta.rx_rssi <= 63 ) )
            {
                data[0] = ( int8_t )( meta.rx_rssi + 64 );  // strength of last downlink (RSSI [dBm]+64)
            }
            else if( meta.rx_rssi > 63 )
            {
                data[0] = 127;
            }
            else if( meta.rx_rssi < -128 )
            {
                data[0] = -128;
            }

            data[1]      = meta.rx_snr << 2;
            data[2]      = meta.rx_window;  // manage also bit ACK/NACK
            data[3]      = meta.rx_fport;
            *data_length = 4 + data_len;
            memcpy( &data[4], buffer, data_len );

            break;
        }
        case SMTC_MODEM_EVENT_FILEDONE:
            *data_length = 1;
            data[0]      = get_modem_event_status( msg_event );
            break;
        case SMTC_MODEM_EVENT_TXDONE:
            *data_length = 1;
            data[0]      = get_modem_event_status( msg_event );
            break;
        case SMTC_MODEM_EVENT_SETCONF:
            *data_length = 1;
            data[0]      = get_modem_event_status( msg_event );
            break;
        case SMTC_MODEM_EVENT_MUTE:
            *data_length = 1;
            data[0]      = ( get_modem_muted( ) == MODEM_NOT_MUTE ) ? false : true;
            break;
        case SMTC_MODEM_EVENT_LINKSTATUS:
            // @ TODO
            break;
#if defined( _WIFI_SNIFF_ENABLE ) && defined( LR1110_MODEM )
        case SMTC_MODEM_EVENT_WIFI: {
            uint16_t size = WifiGetSize( );
            WifiReadResults( );
            //// size = 1000;
            /// for (int i = 0 ; i < 1000; i ++)
            //{
            //  POOL_MEM.WIFI_MEM.Pool_mem.Buffer_tx.EventModem[i]=(i%255);
            //}
            *data_length = size;
            memcpy( data, ( uint8_t* ) ( &( POOL_MEM.WIFI_MEM.Pool_mem.Buffer_tx.EventModem[0] ) + 4 ), *data_length );
            break;
        }
#endif  // _WIFI_SNIFF_ENABLE
#if defined( _GNSS_SNIFF_ENABLE ) && defined( LR1110_MODEM )
        case SMTC_MODEM_EVENT_GNSS:
            *data_length = GnssGetSize( );
            memcpy( data, ( uint8_t* ) &( POOL_MEM.GNSS_MEM.Buf_data[0] ), *data_length );
            break;
#endif  // _GNSS_SNIFF_ENABLE
        case SMTC_MODEM_EVENT_TIME_UPDATED:
            *data_length = 1;
            data[0]      = is_alc_sync_done( &( smtc_modem_services_ctx.alc_sync_ctx ) );
            break;
        default:
            // already manage by the first test in the function
            break;
        }
        // Reset the status after get the value
        set_modem_event_count_and_status( msg_event, 0, 0 );
        decrement_asynchronous_msgnumber( );
    }
    else
    {
        // No event is available
        *pending_count = 0;
        *type          = SMTC_MODEM_NO_EVENT;
    }
    return return_code;
}

smtc_modem_return_code_t smtc_modem_get_version( smtc_modem_version_t* firmware_version,
                                                 smtc_modem_version_t* lorawan_version )
{
    if( modem_get_test_mode_status( ) == true )  // Check if modem is in test mode
    {
        return RC_BUSY;
    }
    smtc_modem_return_code_t return_code = RC_OK;

    firmware_version->major = FW_VERSION_MAJOR;
    firmware_version->minor = FW_VERSION_MINOR;
    firmware_version->patch = FW_VERSION_PATCH;

    lorawan_version->major = LR_VERSION_MAJOR;
    lorawan_version->minor = LR_VERSION_MINOR;
    lorawan_version->patch = LR_VERSION_PATCH;

    return return_code;
}

smtc_modem_return_code_t smtc_modem_reset( void )
{
    if( modem_get_test_mode_status( ) == true )  // Check if modem is in test mode
    {
        return RC_BUSY;
    }
    smtc_modem_return_code_t return_code = RC_OK;
    set_modem_reset_requested( true );
    return return_code;
}

smtc_modem_return_code_t smtc_modem_factory_reset( void )
{
    if( modem_get_test_mode_status( ) == true )  // Check if modem is in test mode
    {
        return RC_BUSY;
    }
    smtc_modem_return_code_t return_code = RC_OK;
    lorawan_api_factory_reset( );
    modem_context_factory_reset( );
    set_modem_reset_requested( true );
    return return_code;
}

smtc_modem_return_code_t smtc_modem_get_charge( uint32_t* charge_mah )
{
    if( modem_get_test_mode_status( ) == true )  // Check if modem is in test mode
    {
        return RC_BUSY;
    }
    smtc_modem_return_code_t return_code = RC_OK;
    *charge_mah                          = get_modem_charge_ma_h( );
    return return_code;
}

smtc_modem_return_code_t smtc_modem_reset_charge( void )
{
    if( modem_get_test_mode_status( ) == true )  // Check if modem is in test mode
    {
        return RC_BUSY;
    }
    smtc_modem_return_code_t return_code = RC_OK;
    reset_modem_charge( );
    return return_code;
}

smtc_modem_return_code_t smtc_modem_get_tx_power_offset_db( uint8_t stack_id, int8_t* tx_pwr_offset_db )
{
    if( modem_get_test_mode_status( ) == true )  // Check if modem is in test mode
    {
        return RC_BUSY;
    }
    smtc_modem_return_code_t return_code = RC_OK;
    *tx_pwr_offset_db                    = modem_context_get_tx_power_offset_db( );
    return return_code;
}

smtc_modem_return_code_t smtc_modem_set_tx_power_offset_db( uint8_t stack_id, int8_t tx_pwr_offset_db )
{
    if( modem_get_test_mode_status( ) == true )  // Check if modem is in test mode
    {
        return RC_BUSY;
    }
    smtc_modem_return_code_t return_code = RC_OK;
    modem_context_set_tx_power_offset_db( tx_pwr_offset_db );
    return return_code;
}

smtc_modem_return_code_t smtc_modem_get_rx_power_offset_db( uint8_t stack_id, int16_t* rx_offset_db )
{
    if( modem_get_test_mode_status( ) == true )  // Check if modem is in test mode
    {
        return RC_BUSY;
    }
    smtc_modem_return_code_t return_code = RC_OK;
    *rx_offset_db                        = modem_context_get_rx_pathloss_db( );
    return return_code;
}

smtc_modem_return_code_t smtc_modem_set_rx_power_offset_db( uint8_t stack_id, int16_t rx_offset_db )
{
    if( modem_get_test_mode_status( ) == true )  // Check if modem is in test mode
    {
        return RC_BUSY;
    }
    smtc_modem_return_code_t return_code = RC_OK;
    modem_context_set_rx_pathloss_db( rx_offset_db );
    return return_code;
}

smtc_modem_return_code_t smtc_modem_set_alcsync_mode( uint8_t mode )
{
    if( modem_get_test_mode_status( ) == true )  // Check if modem is in test mode
    {
        return RC_BUSY;
    }

    smtc_modem_return_code_t return_code = RC_OK;

    alc_sync_set_enabled( &( smtc_modem_services_ctx.alc_sync_ctx ), ( bool ) mode );

    if( mode == 1 )
    {
        // Activate only if modem is Joined
        if( get_join_state( ) == MODEM_JOINED )
        {
            // Activate only if not running
            if( modem_supervisor_is_alc_sync_running( ) == false )
            {
                modem_supervisor_add_task_alc_sync_time_req( 1 );

                // Force no synchronisation
                alc_sync_set_sync_lost( &( smtc_modem_services_ctx.alc_sync_ctx ) );
            }
        }
    }
    else if( mode == 0 )
    {
        modem_supervisor_remove_task_alc_sync( );

        // Force no synchronisation
        alc_sync_set_sync_lost( &( smtc_modem_services_ctx.alc_sync_ctx ) );
    }
    else
    {
        return_code = RC_INVALID;
    }

    return return_code;
}

smtc_modem_return_code_t smtc_modem_get_alcsync_mode( uint8_t* mode )
{
    if( modem_get_test_mode_status( ) == true )  // Check if modem is in test mode
    {
        return RC_BUSY;
    }

    smtc_modem_return_code_t return_code = RC_OK;

    *mode = ( uint8_t ) alc_sync_is_enabled( &( smtc_modem_services_ctx.alc_sync_ctx ) );

    return return_code;
}

smtc_modem_return_code_t smtc_modem_get_time( uint32_t* gps_time_s )
{
    if( modem_get_test_mode_status( ) == true )  // Check if modem is in test mode
    {
        return RC_BUSY;
    }
    smtc_modem_return_code_t return_code = RC_OK;
    *gps_time_s                          = alc_sync_get_gps_time_second( &( smtc_modem_services_ctx.alc_sync_ctx ) );

    return return_code;
}

smtc_modem_return_code_t smtc_modem_set_alc_sync_fport( uint8_t alc_sync_fport )
{
    if( modem_get_test_mode_status( ) == true )  // Check if modem is in test mode
    {
        return RC_BUSY;
    }
    smtc_modem_return_code_t return_code = RC_OK;
    if( alc_sync_set_port( &( smtc_modem_services_ctx.alc_sync_ctx ), alc_sync_fport ) != 0 )
    {
        return_code = RC_INVALID;
    }

    return return_code;
}

smtc_modem_return_code_t smtc_modem_get_alc_sync_fport( uint8_t* alc_sync_fport )
{
    if( modem_get_test_mode_status( ) == true )  // Check if modem is in test mode
    {
        return RC_BUSY;
    }
    smtc_modem_return_code_t return_code = RC_OK;
    *alc_sync_fport                      = alc_sync_get_port( &( smtc_modem_services_ctx.alc_sync_ctx ) );
    return return_code;
}

smtc_modem_return_code_t smtc_modem_get_status( uint8_t stack_id, smtc_modem_status_mask_t* status_mask )
{
    if( modem_get_test_mode_status( ) == true )  // Check if modem is in test mode
    {
        return RC_BUSY;
    }
    smtc_modem_return_code_t return_code = RC_OK;
    *status_mask                         = ( smtc_modem_status_mask_t ) get_modem_status( );
    return return_code;
}

smtc_modem_return_code_t smtc_modem_start_alarm_timer( uint32_t alarm_s )
{
    if( modem_get_test_mode_status( ) == true )  // Check if modem is in test mode
    {
        return RC_BUSY;
    }
    smtc_modem_return_code_t return_code = RC_OK;
    modem_set_user_alarm( ( alarm_s > 0 ) ? ( smtc_modem_hal_get_time_in_s( ) + alarm_s ) : 0 );
    return return_code;
}

smtc_modem_return_code_t smtc_modem_get_joineui( uint8_t stack_id, uint8_t joineui[SMTC_MODEM_EUI_LENGTH] )
{
    if( modem_get_test_mode_status( ) == true )  // Check if modem is in test mode
    {
        return RC_BUSY;
    }
    smtc_modem_return_code_t return_code = RC_OK;
    lorawan_api_appeui_key_get( joineui );
    return return_code;
}

smtc_modem_return_code_t smtc_modem_set_joineui( uint8_t stack_id, const uint8_t joineui[SMTC_MODEM_EUI_LENGTH] )
{
    if( modem_get_test_mode_status( ) == true )  // Check if modem is in test mode
    {
        return RC_BUSY;
    }
    smtc_modem_return_code_t return_code = RC_OK;

    if( get_join_state( ) != MODEM_NOT_JOINED )  // the modem have to be leave from the network to modify the key
    {
        return_code = RC_BUSY;
        SMTC_MODEM_HAL_TRACE_ERROR( "%s call but the device is already join\n", __func__ );
    }
    else
    {
        lorawan_api_appeui_key_set( joineui );
    }

    return return_code;
}

smtc_modem_return_code_t smtc_modem_get_deveui( uint8_t stack_id, uint8_t deveui[SMTC_MODEM_EUI_LENGTH] )
{
    if( modem_get_test_mode_status( ) == true )  // Check if modem is in test mode
    {
        return RC_BUSY;
    }
    smtc_modem_return_code_t return_code = RC_OK;
    lorawan_api_deveui_get( deveui );
    return return_code;
}

smtc_modem_return_code_t smtc_modem_set_deveui( uint8_t stack_id, const uint8_t deveui[SMTC_MODEM_EUI_LENGTH] )
{
    if( modem_get_test_mode_status( ) == true )  // Check if modem is in test mode
    {
        return RC_BUSY;
    }
    smtc_modem_return_code_t return_code = RC_OK;
    if( get_join_state( ) != MODEM_NOT_JOINED )  // the modem have to be leave from the network to modify the key
    {
        return_code = RC_BUSY;
        SMTC_MODEM_HAL_TRACE_ERROR( "%s call but the device is already join\n", __func__ );
    }
    else
    {
        lorawan_api_deveui_set( deveui );
    }
    return return_code;
}

smtc_modem_return_code_t smtc_modem_set_nwkkey( uint8_t stack_id, const uint8_t nwkkey[SMTC_MODEM_KEY_LENGTH] )
{
    if( modem_get_test_mode_status( ) == true )  // Check if modem is in test mode
    {
        return RC_BUSY;
    }
    smtc_modem_return_code_t return_code = RC_OK;
    if( get_join_state( ) != MODEM_NOT_JOINED )  // the modem have to be leave from the network to modify the key
    {
        return_code = RC_BUSY;
        SMTC_MODEM_HAL_TRACE_ERROR( "%s call but the device is already join\n", __func__ );
    }
    else
    {
        modem_context_set_appkey( nwkkey );
    }
    return return_code;
}

smtc_modem_return_code_t smtc_modem_get_class( uint8_t stack_id, smtc_modem_class_t* lorawan_class )
{
    if( modem_get_test_mode_status( ) == true )  // Check if modem is in test mode
    {
        return RC_BUSY;
    }
    smtc_modem_return_code_t return_code = RC_OK;
    *lorawan_class                       = get_modem_class( );
    return return_code;
}

smtc_modem_return_code_t smtc_modem_set_class( uint8_t stack_id, smtc_modem_class_t lorawan_class )
{
    if( modem_get_test_mode_status( ) == true )  // Check if modem is in test mode
    {
        return RC_BUSY;
    }
    smtc_modem_return_code_t return_code = RC_OK;
    if( set_modem_class( lorawan_class ) == SET_ERROR )
    {
        return_code = RC_INVALID;
        SMTC_MODEM_HAL_TRACE_ERROR( "%s call with class not valid\n", __func__ );
    }
    return return_code;
}

smtc_modem_return_code_t smtc_modem_get_region( uint8_t stack_id, smtc_modem_region_t* region )
{
    if( modem_get_test_mode_status( ) == true )  // Check if modem is in test mode
    {
        return RC_BUSY;
    }
    smtc_modem_return_code_t return_code = RC_OK;
    *region                              = get_modem_region( );
    return return_code;
}

smtc_modem_return_code_t smtc_modem_set_region( uint8_t stack_id, smtc_modem_region_t region )
{
    if( modem_get_test_mode_status( ) == true )  // Check if modem is in test mode
    {
        return RC_BUSY;
    }
    smtc_modem_return_code_t return_code = RC_OK;
    if( get_join_state( ) != MODEM_NOT_JOINED )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "%s call but the device is already join\n", __func__ );
        return RC_BUSY;
    }

    if( set_modem_region( region ) == SET_ERROR )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "%s call with region not valid\n", __func__ );
        return RC_INVALID;
    }

    return return_code;
}

smtc_modem_return_code_t smtc_modem_get_adr_profile( uint8_t stack_id, smtc_modem_adr_profile_t* adr_profile )
{
    if( modem_get_test_mode_status( ) == true )  // Check if modem is in test mode
    {
        return RC_BUSY;
    }
    smtc_modem_return_code_t return_code = RC_OK;
    *adr_profile                         = get_modem_adr_profile( );
    return return_code;
}

smtc_modem_return_code_t smtc_modem_set_adr_profile( uint8_t stack_id, smtc_modem_adr_profile_t adr_profile,
                                                     const uint8_t adr_custom_data[SMTC_MODEM_CUSTOM_ADR_DATA_LENGTH] )
{
    if( modem_get_test_mode_status( ) == true )  // Check if modem is in test mode
    {
        return RC_BUSY;
    }
    smtc_modem_return_code_t return_code = RC_OK;
    e_set_error_t            status;

    if( adr_profile == SMTC_MODEM_ADR_PROFILE_CUSTOM )
    {
        status = set_modem_adr_profile( adr_profile, adr_custom_data, 16 );
    }
    else if( ( adr_profile == SMTC_MODEM_ADR_PROFILE_NETWORK_CONTROLLED ) ||
             ( adr_profile == SMTC_MODEM_ADR_PROFILE_MOBILE_LONG_RANGE ) ||
             ( adr_profile == SMTC_MODEM_ADR_PROFILE_MOBILE_LOW_POWER ) )
    {
        status = set_modem_adr_profile( adr_profile, adr_custom_data, 0 );
    }
    else
    {
        status = SET_ERROR;
    }
    if( ( adr_profile == SMTC_MODEM_ADR_PROFILE_MOBILE_LONG_RANGE ) ||
        ( adr_profile == SMTC_MODEM_ADR_PROFILE_MOBILE_LOW_POWER ) || ( adr_profile == SMTC_MODEM_ADR_PROFILE_CUSTOM ) )
    {
        // reset current adr mobile count
        modem_reset_current_adr_mobile_count( );
    }
    if( status == SET_ERROR )
    {
        return_code = RC_INVALID;
        SMTC_MODEM_HAL_TRACE_ERROR( "%s call with adr profile not valid\n", __func__ );
    }
    return return_code;
}

smtc_modem_return_code_t smtc_modem_get_available_datarates( uint8_t stack_id, uint16_t* available_datarates_mask )
{
    if( modem_get_test_mode_status( ) == true )  // Check if modem is in test mode
    {
        return RC_BUSY;
    }
    smtc_modem_return_code_t return_code = RC_OK;
    *available_datarates_mask            = lorawan_api_mask_tx_dr_channel_up_dwell_time_check( );
    return return_code;
}

smtc_modem_return_code_t smtc_modem_get_dm_fport( uint8_t* dm_fport )
{
    if( modem_get_test_mode_status( ) == true )  // Check if modem is in test mode
    {
        return RC_BUSY;
    }
    smtc_modem_return_code_t return_code = RC_OK;
    *dm_fport                            = get_modem_dm_port( );
    return return_code;
}

smtc_modem_return_code_t smtc_modem_set_dm_fport( uint8_t dm_fport )
{
    if( modem_get_test_mode_status( ) == true )  // Check if modem is in test mode
    {
        return RC_BUSY;
    }
    smtc_modem_return_code_t return_code = RC_OK;
    if( set_modem_dm_port( dm_fport ) == SET_ERROR )
    {
        return_code = RC_INVALID;
        SMTC_MODEM_HAL_TRACE_ERROR( "%s call with DM port not valid\n", __func__ );
    }

    return return_code;
}

smtc_modem_return_code_t smtc_modem_get_dm_info_interval( smtc_modem_dm_info_interval_format_t* format,
                                                          uint8_t*                              interval )
{
    if( modem_get_test_mode_status( ) == true )  // Check if modem is in test mode
    {
        return RC_BUSY;
    }
    smtc_modem_return_code_t return_code    = RC_OK;
    uint8_t                  modem_interval = 0;

    modem_interval = get_modem_dm_interval( );

    *format   = ( smtc_modem_dm_info_interval_format_t )( ( modem_interval >> 6 ) & 0x03 );
    *interval = modem_interval & 0x3F;

    return return_code;
}
smtc_modem_return_code_t smtc_modem_set_dm_info_interval( smtc_modem_dm_info_interval_format_t format,
                                                          uint8_t                              interval )
{
    if( modem_get_test_mode_status( ) == true )  // Check if modem is in test mode
    {
        return RC_BUSY;
    }
    smtc_modem_return_code_t return_code    = RC_OK;
    uint8_t                  modem_interval = 0;

    if( ( format > SMTC_MODEM_DM_INFO_INTERVAL_IN_MINUTE ) || ( interval > 0x3F ) )
    {
        return_code = RC_INVALID;
    }
    else
    {
        modem_interval = ( ( ( uint8_t ) format << 6 ) & 0xC0 ) | ( interval & 0x3F );

        if( set_modem_dm_interval( modem_interval ) == SET_ERROR )
        {
            return_code = RC_INVALID;
            SMTC_MODEM_HAL_TRACE_ERROR( "%s call but interval not valid\n", __func__ );
        }
        else
        {
            modem_supervisor_add_task_dm_status( get_modem_dm_interval_second( ) );
        }
    }
    return return_code;
}

smtc_modem_return_code_t smtc_modem_get_dm_info_fields( uint8_t* dm_fields_payload, uint8_t* dm_field_length )
{
    if( modem_get_test_mode_status( ) == true )  // Check if modem is in test mode
    {
        return RC_BUSY;
    }
    smtc_modem_return_code_t return_code = RC_OK;
    return_code = smtc_modem_get_dm_status_with_rate( dm_fields_payload, dm_field_length, DM_INFO_PERIODIC );
    return return_code;
}

smtc_modem_return_code_t smtc_modem_set_dm_info_fields( const uint8_t* dm_fields_payload, uint8_t dm_field_length )
{
    if( modem_get_test_mode_status( ) == true )  // Check if modem is in test mode
    {
        return RC_BUSY;
    }
    smtc_modem_return_code_t return_code = RC_OK;
    return_code = smtc_modem_set_dm_status_with_rate( dm_fields_payload, dm_field_length, DM_INFO_PERIODIC );
    return return_code;
}

smtc_modem_return_code_t smtc_modem_request_single_dm_uplink( const uint8_t* dm_fields_payload,
                                                              uint8_t        dm_field_length )
{
    if( modem_get_test_mode_status( ) == true )  // Check if modem is in test mode
    {
        return RC_BUSY;
    }
    smtc_modem_return_code_t return_code = RC_OK;
    if( is_modem_connected( ) == false )
    {
        return_code = RC_FAIL;
    }
    else
    {
        return_code = smtc_modem_set_dm_status_with_rate( dm_fields_payload, dm_field_length, DM_INFO_NOW );
        if( return_code == RC_OK )
        {
            modem_supervisor_add_task_dm_status_now( );
        }
    }
    return return_code;
}

smtc_modem_return_code_t smtc_modem_set_dm_user_data( const uint8_t user_data[SMTC_MODEM_DM_USER_DATA_LENGTH] )
{
    if( modem_get_test_mode_status( ) == true )  // Check if modem is in test mode
    {
        return RC_BUSY;
    }
    smtc_modem_return_code_t return_code = RC_OK;
    set_modem_appstatus( user_data );
    return return_code;
}

smtc_modem_return_code_t smtc_modem_join_network( uint8_t stack_id )
{
    if( modem_get_test_mode_status( ) == true )  // Check if modem is in test mode
    {
        return RC_BUSY;
    }
    smtc_modem_return_code_t return_code = RC_OK;

    if( lorawan_api_is_ota_device( ) == ABP_DEVICE )
    {
        return_code = RC_FAIL;
        SMTC_MODEM_HAL_TRACE_ERROR( "%s call but the device is configured in ABP mode\n", __func__ );
    }
    else if( get_join_state( ) != MODEM_NOT_JOINED )  // the modem have to be leave from the network to join
    {
        return_code = RC_BUSY;
        SMTC_MODEM_HAL_TRACE_ERROR( "%s call but the device is already join\n", __func__ );
    }
    else if( get_modem_muted( ) == MODEM_INFINITE_MUTE )
    {
        return_code = RC_FAIL;
        SMTC_MODEM_HAL_TRACE_ERROR( "%s Modem is muted\n", __func__ );
    }
    else if( get_modem_suspend( ) == MODEM_SUSPEND )
    {
        return_code = RC_FAIL;
        SMTC_MODEM_HAL_TRACE_ERROR( "%s Modem is suspend\n", __func__ );
    }
    else
    {
        modem_supervisor_add_task_join( );
    }

    return return_code;
}

smtc_modem_return_code_t smtc_modem_leave_network( uint8_t stack_id )
{
    if( modem_get_test_mode_status( ) == true )  // Check if modem is in test mode
    {
        return RC_BUSY;
    }
    smtc_modem_return_code_t return_code = RC_OK;

    lorawan_api_class_c_enabled( false );
    set_modem_status_modem_joined( false );
    lorawan_api_join_status_clear( );
    set_modem_status_joining( false );
    // re init task to retrieve a clean env
    modem_supervisor_init_task( );
    return return_code;
}
smtc_modem_return_code_t smtc_modem_suspend_radio_communications( bool suspend )
{
    if( modem_get_test_mode_status( ) == true )  // Check if modem is in test mode
    {
        return RC_BUSY;
    }
    smtc_modem_return_code_t return_code = RC_OK;
    if( set_modem_suspend( suspend ) == SET_ERROR )
    {
        return_code = RC_INVALID;
        SMTC_MODEM_HAL_TRACE_ERROR( "%s call but suspend value is not valid\n", __func__ );
    }
    return return_code;
}

smtc_modem_return_code_t smtc_modem_get_next_tx_max_payload( uint8_t stack_id, uint8_t* tx_max_payload_size )
{
    if( modem_get_test_mode_status( ) == true )  // Check if modem is in test mode
    {
        return RC_BUSY;
    }
    smtc_modem_return_code_t return_code = RC_OK;
    if( get_join_state( ) == MODEM_JOINED )
    {
        *tx_max_payload_size = lorawan_api_next_max_payload_length_get( );
    }
    else
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "%s call but the device is not join\n", __func__ );
        *tx_max_payload_size = 255;
        return_code          = RC_FAIL;
    }

    return return_code;
}

smtc_modem_return_code_t smtc_modem_request_uplink( uint8_t stack_id, uint8_t f_port, bool confirmed,
                                                    const uint8_t* payload, uint8_t payload_length )
{
    if( modem_get_test_mode_status( ) == true )  // Check if modem is in test mode
    {
        return RC_BUSY;
    }
    smtc_modem_return_code_t return_code = RC_OK;
    return_code                          = smtc_modem_send_tx( f_port, confirmed, payload, payload_length, false );
    return return_code;
}

smtc_modem_return_code_t smtc_modem_request_emergency_uplink( uint8_t stack_id, uint8_t f_port, bool confirmed,
                                                              const uint8_t* payload, uint8_t payload_length )
{
    if( modem_get_test_mode_status( ) == true )  // Check if modem is in test mode
    {
        return RC_BUSY;
    }
    smtc_modem_return_code_t return_code = RC_OK;
    return_code                          = smtc_modem_send_tx( f_port, confirmed, payload, payload_length, true );
    return return_code;
}

smtc_modem_return_code_t smtc_modem_file_upload_init( uint8_t stack_id, uint8_t app_port,
                                                      smtc_modem_file_upload_cipher_mode_t cipher_mode,
                                                      const uint8_t* file, uint16_t file_length,
                                                      uint32_t average_delay_s )
{
    if( modem_get_test_mode_status( ) == true )  // Check if modem is in test mode
    {
        return RC_BUSY;
    }
    smtc_modem_return_code_t return_code = RC_OK;

    if( ( app_port == 0 ) || ( app_port >= 224 ) || ( cipher_mode > SMTC_MODEM_FILE_UPLOAD_AES_WITH_APPSKEY ) )
    {
        return_code = RC_INVALID;
    }
    else
    {
        // upload_clear(UPLOAD_SID);
        set_modem_status_file_upload( false );
        upload_size = file_length;

        if( upload_size == 0 )
        {
            SMTC_MODEM_HAL_TRACE_WARNING( "FileUpload Cancel!\n" );
            modem_set_upload_state( MODEM_UPLOAD_NOT_INIT );
        }
        else if( modem_get_upload_state( ) != MODEM_UPLOAD_NOT_INIT )
        {
            return_code = RC_BUSY;
            SMTC_MODEM_HAL_TRACE_ERROR( "File Upload still in going\n" );
        }
        else if( file == NULL )
        {
            modem_set_upload_state( MODEM_UPLOAD_NOT_INIT );
            SMTC_MODEM_HAL_TRACE_ERROR( "Upload file data, null\n" );
            return RC_NOT_INIT;
        }
        else
        {
            // set average delay
            modem_set_upload_avgdelay( average_delay_s );

            // create the upload session
            int8_t session_counter =
                file_upload_create( &( smtc_modem_services_ctx.file_upload_ctx ), UPLOAD_SID, &upload_pdata,
                                    upload_size, average_delay_s, app_port, ( uint8_t ) cipher_mode );

            if( session_counter < 0 )
            {
                return_code = RC_FAIL;
            }
            else
            {
                modem_set_upload_state( MODEM_UPLOAD_INIT );

                SMTC_MODEM_HAL_TRACE_PRINTF( "%s, cipher_mode: %d, size:%d, average_delay:%d", __func__, cipher_mode,
                                             file_length, average_delay_s );
                // attach the file
                upload_pdata = ( uint32_t* ) file;
                file_upload_attach_payload_buffer( &( smtc_modem_services_ctx.file_upload_ctx ),
                                                   ( uint8_t* ) upload_pdata );

                modem_set_upload_state( MODEM_UPLOAD_DATA );
            }
        }
    }

    return return_code;
}

smtc_modem_return_code_t smtc_modem_file_upload_start( uint8_t stack_id )
{
    smtc_modem_return_code_t return_code = RC_OK;
    if( modem_get_test_mode_status( ) == true )  // Check if modem is in test mode
    {
        return RC_BUSY;
    }
    if( is_modem_connected( ) == false )
    {
        return RC_FAIL;
    }

    if( modem_get_upload_state( ) == MODEM_UPLOAD_START )
    {
        return_code = RC_BUSY;
        SMTC_MODEM_HAL_TRACE_ERROR( "FileUpload in progress..\n" );
    }
    else
    {
        uint32_t hash[8];
        sha256( hash, ( unsigned char* ) upload_pdata, upload_size );
        file_upload_set_hash( &( smtc_modem_services_ctx.file_upload_ctx ), hash[0], hash[1] );

        if( file_upload_get_encryption_mode( &( smtc_modem_services_ctx.file_upload_ctx ) ) == FILE_UPLOAD_ENCRYPTED )
        {
            uint32_t temp_hash = hash[1];
            lora_crypto_payload_encrypt( ( uint8_t* ) upload_pdata, upload_size, lorawan_api_apps_key_get( ),
                                         upload_size, FILE_UPLOAD_DIRECTION, hash[0], ( uint8_t* ) upload_pdata );
            // compute hash over encrypted data
            sha256( hash, ( unsigned char* ) upload_pdata, upload_size );
            file_upload_set_hash( &( smtc_modem_services_ctx.file_upload_ctx ), hash[0], temp_hash );
        }
        // start streaming of chunks

        smodem_task upload_task;
        upload_task.id                = FILE_UPLOAD_TASK;
        upload_task.priority          = TASK_HIGH_PRIORITY;
        upload_task.time_to_execute_s = smtc_modem_hal_get_time_in_s( ) + 2;
        if( modem_supervisor_add_task( &upload_task ) != TASK_VALID )
        {
            modem_set_upload_state( MODEM_UPLOAD_NOT_INIT );
            return_code = RC_FAIL;
        }
        else
        {
            modem_set_upload_state( MODEM_UPLOAD_START );
        }
    }

    return return_code;
}

smtc_modem_return_code_t smtc_modem_file_upload_reset( uint8_t stack_id )
{
    if( modem_get_test_mode_status( ) == true )  // Check if modem is in test mode
    {
        return RC_BUSY;
    }

    SMTC_MODEM_HAL_TRACE_WARNING( "File Upload Cancel and session reset!\n" );
    modem_set_upload_state( MODEM_UPLOAD_NOT_INIT );
    return RC_OK;
}

smtc_modem_return_code_t smtc_modem_stream_init( uint8_t stack_id, uint8_t f_port,
                                                 smtc_modem_stream_cipher_mode_t cipher_mode )
{
    if( modem_get_test_mode_status( ) == true )  // Check if modem is in test mode
    {
        return RC_BUSY;
    }

    if( modem_port_reserved( f_port ) || ( cipher_mode > SMTC_MODEM_STREAM_AES_WITH_APPSKEY ) )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "STREAM_INIT port or encryption mode invalid\n" );
        return RC_INVALID;
    }
    if( f_port == 0 )
    {
        SMTC_MODEM_HAL_TRACE_MSG( "STREAM_INIT Using default DM port\n" );
        f_port = get_modem_dm_port( );
    }

    if( modem_get_stream_state( ) == MODEM_STREAM_INIT &&
        stream_data_pending( &( smtc_modem_services_ctx.stream_ROSE_ctx ) ) )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "STREAM_INIT Stream is busy\n" );
        // There is some data remaining in the previous stream
        return RC_BUSY;
    }
    // initialize stream session
    if( stream_init( &( smtc_modem_services_ctx.stream_ROSE_ctx ) ) != STREAM_OK )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "STREAM_INIT FAILED\n" );
        return RC_FAIL;
    }
    // enable encryption if needed
    if( cipher_mode == SMTC_MODEM_STREAM_AES_WITH_APPSKEY )
    {
        if( stream_enable_encryption( &( smtc_modem_services_ctx.stream_ROSE_ctx ), lorawan_api_apps_key_get( ) ) !=
            STREAM_OK )
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "STREAM_INIT ENCRYPTION FAILED\n" );
            return RC_FAIL;
        }
    }

    modem_set_stream_port( f_port );
    modem_set_stream_encryption( cipher_mode == SMTC_MODEM_STREAM_AES_WITH_APPSKEY );
    modem_set_stream_state( MODEM_STREAM_INIT );

    return RC_OK;
}

smtc_modem_return_code_t smtc_modem_stream_add_data( uint8_t stack_id, uint8_t f_port, const uint8_t* data,
                                                     uint8_t len )
{
    if( modem_get_test_mode_status( ) == true )  // Check if modem is in test mode
    {
        return RC_BUSY;
    }
    stream_return_code_t stream_rc;

    if( is_modem_connected( ) == false )
    {
        return RC_FAIL;
    }

    /* Existing stream with pending data */
    if( stream_data_pending( &( smtc_modem_services_ctx.stream_ROSE_ctx ) ) && f_port != modem_get_stream_port( ) )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "STREAM_SEND Sending on wrong stream\n" );
        return RC_INVALID;
    }

    /* No existing stream, or changing port if existing is idle */
    if( modem_get_stream_state( ) == MODEM_STREAM_NOT_INIT ||
        ( !stream_data_pending( &( smtc_modem_services_ctx.stream_ROSE_ctx ) ) && f_port != modem_get_stream_port( ) ) )
    {
        smtc_modem_return_code_t rc;
        // Start new unencrypted session
        rc = smtc_modem_stream_init( stack_id, f_port, SMTC_MODEM_STREAM_NO_CIPHER );
        if( rc != RC_OK )
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "STREAM_SEND Stream init failed\n" );
            return rc;
        }
    }
    stream_rc = stream_add_data( &( smtc_modem_services_ctx.stream_ROSE_ctx ), data, len );

    switch( stream_rc )
    {
    case STREAM_BADSIZE:
        SMTC_MODEM_HAL_TRACE_ERROR( "STREAM_SEND_DATA BADSIZE\n" );
        return RC_BAD_SIZE;
    case STREAM_BUSY:
        SMTC_MODEM_HAL_TRACE_ERROR( "STREAM_SEND_DATA BUSY\n" );
        return RC_BUSY;
    case STREAM_FAIL:
        SMTC_MODEM_HAL_TRACE_ERROR( "STREAM_SEND_DATA FAIL\n" );
        return RC_FAIL;
    default:
        break;
    }

#ifdef LORAWAN_BYPASS_ENABLED
    if( stream_bypass_enabled == true )
    {
        SMTC_MODEM_HAL_TRACE_INFO( "STREAM_SEND BYPASS [OK]\n" );
        return RC_OK;
    }
#endif  // LORAWAN_BYPASS_ENABLED

    modem_supervisor_add_task_stream( );

    SMTC_MODEM_HAL_TRACE_INFO( "STREAM_SEND [OK]\n" );
    return RC_OK;
}

smtc_modem_return_code_t smtc_modem_stream_status( uint8_t stack_id, uint8_t fport, uint16_t* pending, uint16_t* free )
{
    if( modem_get_test_mode_status( ) == true )  // Check if modem is in test mode
    {
        return RC_BUSY;
    }
    if( modem_get_stream_state( ) == MODEM_STREAM_NOT_INIT )
    {
        return RC_NOT_INIT;
    }
    if( modem_port_reserved( fport ) || modem_get_stream_port( ) != fport )
    {
        return RC_INVALID;
    }

    stream_status( &( smtc_modem_services_ctx.stream_ROSE_ctx ), pending, free );
    return RC_OK;
}

#ifdef LORAWAN_BYPASS_ENABLED
/*
 * When the bypass is enabled, don't send anything via LORAWAN.
 * Instead, send/receive locally via these functions
 */

/*
 * Enable and Disable bypass
 */
void modem_stream_bypass_enable( bool enabled )
{
    stream_bypass_enabled = enabled;
    SMTC_MODEM_HAL_TRACE_PRINTF( "STREAM BYPASS %d\n", stream_bypass_enabled );
}

/*
 * Get one full uplink payload
 */
smtc_modem_return_code_t smtc_modem_stream_bypass_get_fragment( uint8_t* buffer, uint32_t frag_cnt, uint8_t* len )
{
    if( !stream_bypass_enabled )
    {
        return RC_FAIL;
    }
    if( !stream_data_pending( ) )
    {
        return RC_BAD_SIZE;
    }
    if( STREAM_OK != stream_get_fragment( buffer, frag_cnt, len ) )
    {
        return RC_FAIL;
    }
    return RC_OK;
}

/*
 * Send a downlink payload
 */
smtc_modem_return_code_t smtc_modem_stream_bypass_send_downlink( uint8_t* buffer, uint8_t len )
{
    if( !stream_bypass_enabled )
    {
        return RC_FAIL;
    }
    if( STREAM_OK != stream_process_dn_frame( buffer, len ) )
    {
        return RC_FAIL;
    }
    return RC_OK;
}

#endif  // LORAWAN_BYPASS_ENABLED

smtc_modem_return_code_t smtc_modem_set_certification_mode( uint8_t stack_id, bool enable )
{
    if( modem_get_test_mode_status( ) == true )  // Check if modem is in test mode
    {
        return RC_BUSY;
    }
    smtc_modem_return_code_t return_code = RC_OK;
    lorawan_api_modem_certification_set( ( uint8_t ) enable );

    // Enable Dutycycle when certification mode is leaved
    if( enable == false )
    {
        smtc_modem_test_duty_cycle_app_activate( true );
    }
    return return_code;
}

smtc_modem_return_code_t smtc_modem_get_certification_mode( uint8_t stack_id, bool* enable )
{
    if( modem_get_test_mode_status( ) == true )  // Check if modem is in test mode
    {
        return RC_BUSY;
    }
    smtc_modem_return_code_t return_code = RC_OK;
    *enable                              = lorawan_api_modem_certification_is_enabled( );
    return return_code;
}

smtc_modem_return_code_t smtc_modem_set_connection_timeout_thresholds( uint8_t  stack_id,
                                                                       uint16_t nb_of_uplinks_before_network_controlled,
                                                                       uint16_t nb_of_uplinks_before_reset )
{
    if( modem_get_test_mode_status( ) == true )  // Check if modem is in test mode
    {
        return RC_BUSY;
    }
    smtc_modem_return_code_t return_code = RC_OK;

    modem_set_adr_mobile_timeout_config( nb_of_uplinks_before_network_controlled );

    if( lorawan_api_no_rx_packet_count_config_set( nb_of_uplinks_before_reset ) != OKLORAWAN )
    {
        return_code = RC_INVALID;
    }

    return return_code;
}

smtc_modem_return_code_t smtc_modem_get_connection_timeout_thresholds(
    uint8_t stack_id, uint16_t* nb_of_uplinks_before_network_controlled, uint16_t* nb_of_uplinks_before_reset )
{
    if( modem_get_test_mode_status( ) == true )  // Check if modem is in test mode
    {
        return RC_BUSY;
    }
    *nb_of_uplinks_before_network_controlled = modem_get_adr_mobile_timeout_config( );
    *nb_of_uplinks_before_reset              = lorawan_api_no_rx_packet_count_config_get( );
    return RC_OK;
}

smtc_modem_return_code_t smtc_modem_get_current_connection_timeout( uint8_t   stack_id,
                                                                    uint16_t* nb_of_uplinks_before_network_controlled,
                                                                    uint16_t* nb_of_uplinks_before_reset )
{
    if( modem_get_test_mode_status( ) == true )  // Check if modem is in test mode
    {
        return RC_BUSY;
    }
    *nb_of_uplinks_before_network_controlled = modem_get_current_adr_mobile_count( );
    *nb_of_uplinks_before_reset              = lorawan_api_no_rx_packet_count_current_get( );
    return RC_OK;
}

smtc_modem_return_code_t smtc_modem_get_duty_cycle_status( int32_t* next_free_dtc )
{
    if( modem_get_test_mode_status( ) == true )  // Check if modem is in test mode
    {
        return RC_BUSY;
    }
    *next_free_dtc = -1 * lorawan_api_next_free_duty_cycle_ms_get( );
    return RC_OK;
}

smtc_modem_return_code_t smtc_modem_suspend_radio_access( void )
{
    SMTC_MODEM_HAL_TRACE_INFO( "Suspend modem radio access \n" );

    return ( modem_context_suspend_radio_access( ) == true ) ? RC_OK : RC_FAIL;
}

smtc_modem_return_code_t smtc_modem_resume_radio_access( void )
{
    SMTC_MODEM_HAL_TRACE_INFO( "Resume modem radio access \n" );
    return ( modem_context_resume_radio_access( ) == true ) ? RC_OK : RC_FAIL;
}

smtc_modem_return_code_t smtc_modem_set_network_type( uint8_t stack_id, bool network_type )
{
    modem_context_set_network_type( network_type );
    return RC_OK;
}

smtc_modem_return_code_t smtc_modem_get_network_type( uint8_t stack_id, bool* network_type )
{
    *network_type = modem_context_get_network_type( );
    return RC_OK;
}

// smtc_modem_return_code_t smtc_modem_configure_lbt( uint8_t stack_id, bool activate_lbt, uint32_t listen_duration_ms,
//                                                    int16_t threshold, uint32_t bw_hz )
// {
//     SMTC_MODEM_HAL_TRACE_PRINTF( "LBT, En:%d, duration:%d, threshold:%d, bw:%d\n", activate_lbt, listen_duration_ms,
//                                  threshold, bw_hz );
//     if( activate_lbt == true )
//     {
//         smtc_lbt_configure( lorawan_api_stack_mac_get( )->lbt_obj, listen_duration_ms, threshold, bw_hz );
//     }
//     else
//     {
//         smtc_lbt_disable( lorawan_api_stack_mac_get( )->lbt_obj );
//     }
//     return RC_OK;
// }

smtc_modem_return_code_t smtc_modem_set_lbt_configuration( uint8_t stack_id, uint32_t listen_duration_ms,
                                                           int16_t threshold_dbm, uint32_t bw_hz )
{
    SMTC_MODEM_HAL_TRACE_PRINTF( "LBT, duration:%d, threshold:%d, bw:%d\n", listen_duration_ms, threshold_dbm, bw_hz );
    lbt_config_listen_duration_ms = listen_duration_ms;
    lbt_config_threshold_dbm      = threshold_dbm;
    lbt_config_bw_hz              = bw_hz;
    lbt_config_available          = true;
    return RC_OK;
}

smtc_modem_return_code_t smtc_modem_get_lbt_configuration( uint8_t stack_id, uint32_t* listen_duration_ms,
                                                           int16_t* threshold_dbm, uint32_t* bw_hz )
{
    *listen_duration_ms = lbt_config_listen_duration_ms;
    *threshold_dbm      = lbt_config_threshold_dbm;
    *bw_hz              = lbt_config_bw_hz;
    return RC_OK;
}

smtc_modem_return_code_t smtc_modem_start_lbt( uint8_t stack_id )
{
    if( lbt_config_available == true )
    {
        smtc_lbt_configure( lorawan_api_stack_mac_get( )->lbt_obj, lbt_config_listen_duration_ms,
                            lbt_config_threshold_dbm, lbt_config_bw_hz );
        return RC_OK;
    }
    else
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "A configuration shall be set before starting LBT \n" );
        return RC_FAIL;
    }
}

smtc_modem_return_code_t smtc_modem_stop_lbt( uint8_t stack_id )
{
    smtc_lbt_disable( lorawan_api_stack_mac_get( )->lbt_obj );
    return RC_OK;
}

smtc_modem_return_code_t smtc_modem_set_nb_trans( uint8_t stack_id, uint8_t nb_trans )
{
    if( ( nb_trans == 0 ) || ( nb_trans > 16 ) )
    {
        return RC_INVALID;
    }
    else
    {
        lorawan_api_nb_trans_set( nb_trans );
        return RC_OK;
    }
}

smtc_modem_return_code_t smtc_modem_get_nb_trans( uint8_t stack_id, uint8_t* nb_trans )
{
    *nb_trans = lorawan_api_nb_trans_get( );
    return RC_OK;
}

smtc_modem_return_code_t smtc_modem_get_stream_rr( uint8_t stack_id, uint8_t* stream_rr )
{
    *stream_rr = stream_get_rr( &( smtc_modem_services_ctx.stream_ROSE_ctx ) );
    return RC_OK;
}

smtc_modem_return_code_t smtc_modem_set_stream_redundancy_ratio( uint8_t stack_id, uint8_t redundancy_ratio_percent )
{
    stream_set_rr( &( smtc_modem_services_ctx.stream_ROSE_ctx ), redundancy_ratio_percent );
    return RC_OK;
}

/* ------------ LR1110 Api Extension functions ------------*/

smtc_modem_return_code_t smtc_modem_set_default_key( uint8_t default_app_key[16], uint8_t default_dev_eui[8],
                                                     uint8_t default_join_eui[8] )
{
    if( modem_get_test_mode_status( ) == true )  // Check if modem is in test mode
    {
        return RC_BUSY;
    }
    lorawan_api_set_default_key( default_app_key, default_dev_eui, default_join_eui );
    return RC_OK;
}

smtc_modem_return_code_t smtc_modem_list_regions( uint8_t* region_list, uint8_t* nb_region )
{
    if( modem_get_test_mode_status( ) == true )  // Check if modem is in test mode
    {
        return RC_BUSY;
    }
    smtc_modem_return_code_t return_code = RC_OK;

    memcpy( region_list, smtc_real_region_list, SMTC_REAL_REGION_LIST_LENGTH );
    *nb_region = SMTC_REAL_REGION_LIST_LENGTH;
    return return_code;
}

smtc_modem_return_code_t smtc_modem_set_rf_output( rf_output_t rf_output )
{
    if( modem_get_test_mode_status( ) == true )  // Check if modem is in test mode
    {
        return RC_BUSY;
    }
    smtc_modem_return_code_t return_code = RC_OK;
    if( modem_set_rfo_pa( rf_output ) != SET_OK )
    {
        return_code = RC_INVALID;
    }

    return return_code;
}

smtc_modem_return_code_t smtc_modem_time_available( )
{
    if( modem_get_test_mode_status( ) == true )  // Check if modem is in test mode
    {
        return RC_BUSY;
    }
    smtc_modem_return_code_t return_code = RC_INVALID;
    if( alc_sync_get_gps_time_second( &( smtc_modem_services_ctx.alc_sync_ctx ) ) > 0 )
    {
        return_code = RC_OK;
    }
    return return_code;
}

smtc_modem_return_code_t smtc_modem_set_time( uint32_t gps_time_s )
{
    if( modem_get_test_mode_status( ) == true )  // Check if modem is in test mode
    {
        return RC_BUSY;
    }
    smtc_modem_return_code_t return_code = RC_OK;
    alc_sync_set_time_correction_second( &( smtc_modem_services_ctx.alc_sync_ctx ),
                                         ( int32_t )( gps_time_s - smtc_modem_hal_get_time_in_s( ) ) );

    return return_code;
}
void empty_task_launch_callback_for_rp( void* rp_void )
{
    radio_planner_t* rp = ( radio_planner_t* ) rp_void;
    smtc_modem_hal_start_radio_tcxo( );
    rp_stats_set_none_timestamp( &rp->stats, rp_hal_timestamp_get( ) );
    SMTC_MODEM_HAL_TRACE_PRINTF( "launch task empty\n" );
}

#if defined( LR1110_MODEM )
void modem_suspend_rp( e_sniff_mode_t sniff_mode )
{
    rp_radio_params_t fake_radio_params = { 0 };
    rp_task_t         rp_task;
    uint8_t           fake_payload[2]   = { 0 };
    uint16_t          fake_payload_size = 2;
    rp_task.hook_id                     = 0;
    rp_task.duration_time_ms            = 20000;
    rp_task.state                       = RP_TASK_STATE_SCHEDULE;
    switch( sniff_mode )
    {
#if defined( _GNSS_SNIFF_ENABLE )
    case GNSS_SNIFF_MODE:
        rp_task.type = RP_TASK_TYPE_GNSS_SNIFF;
        break;
#endif  // _GNSS_SNIFF_ENABLE
#if defined( _WIFI_SNIFF_ENABLE )
    case WIFI_SNIFF_MODE:
        rp_task.type = RP_TASK_TYPE_WIFI_SNIFF;
        break;
#endif  // _WIFI_SNIFF_ENABLE
#if defined( _GNSS_SNIFF_ENABLE )
    case GNSS_TEST_RSSI:
        rp_task.type = RP_TASK_TYPE_GNSS_RSSI;
        break;
#endif  // _GNSS_SNIFF_ENABLE
#if defined( _WIFI_SNIFF_ENABLE )
    case WIFI_TEST_RSSI:
        rp_task.type = RP_TASK_TYPE_WIFI_RSSI;
        break;
#endif  // _WIFI_SNIFF_ENABLE
    default:
        rp_task.type = RP_TASK_TYPE_NONE;
        break;
    }
    rp_task.launch_task_callbacks = empty_task_launch_callback_for_rp;
    rp_task.start_time_ms         = smtc_modem_hal_get_time_in_ms( ) + 4;
    rp_task_enqueue( &modem_radio_planner, &rp_task, fake_payload, fake_payload_size, &fake_radio_params );
}
void modem_resume_rp( void )
{
    SMTC_MODEM_HAL_TRACE_PRINTF( "Resume rp\n" );
    rp_task_abort( &modem_radio_planner, 0 );
    smtc_modem_hal_stop_radio_tcxo( );
}

#endif  // LR1110_MODEM

smtc_modem_return_code_t smtc_modem_get_pin( uint8_t stack_id, uint8_t chip_pin[4] )
{
    ( void ) stack_id;  // avoid warning

    // Check if modem is in test mode
    if( modem_get_test_mode_status( ) == true )
    {
        return RC_BUSY;
    }

#if defined( USE_LR1110_SE )
    lr1110_system_uid_t      deveui;
    lr1110_system_join_eui_t joineui;
    lorawan_api_deveui_get( ( uint8_t* ) deveui );
    lorawan_api_appeui_key_get( ( uint8_t* ) joineui );

    // suspend modem radio access to prevent multiple spi access
    smtc_modem_suspend_radio_access( );
    lr1110_status_t status = lr1110_system_read_pin_custom_eui( NULL, deveui, joineui, 0, chip_pin );
    // resume modem radio access
    smtc_modem_resume_radio_access( );

    if( status != LR1110_STATUS_OK )
    {
        return RC_FAIL;
    }
    return RC_OK;
#else
    return RC_NOT_IMPLEMENTED;
#endif
}

smtc_modem_return_code_t smtc_modem_get_chip_eui( uint8_t stack_id, uint8_t chip_eui[8] )
{
    ( void ) stack_id;  // avoid warning

    // Check if modem is in test mode
    if( modem_get_test_mode_status( ) == true )
    {
        return RC_BUSY;
    }

#if defined( USE_LR1110_SE )
    // suspend modem radio access to prevent multiple spi access
    smtc_modem_suspend_radio_access( );
    lr1110_status_t status = lr1110_system_read_uid( NULL, chip_eui );
    // resume modem radio access
    smtc_modem_resume_radio_access( );

    if( status != LR1110_STATUS_OK )
    {
        return RC_FAIL;
    }
    return RC_OK;
#else
    return RC_NOT_IMPLEMENTED;
#endif
}

smtc_modem_return_code_t smtc_modem_derive_keys( uint8_t stack_id )
{
    ( void ) stack_id;  // avoid warning

    // Check if modem is in test mode
    if( modem_get_test_mode_status( ) == true )
    {
        return RC_BUSY;
    }
    if( get_join_state( ) != MODEM_NOT_JOINED )
    {
        return RC_BUSY;
    }

#if defined( USE_LR1110_SE )
    lr1110_system_uid_t      deveui;
    lr1110_system_join_eui_t joineui;
    lr1110_system_pin_t      pin;

    lorawan_api_deveui_get( ( uint8_t* ) deveui );
    lorawan_api_appeui_key_get( ( uint8_t* ) joineui );

    // suspend modem radio access to prevent multiple spi access
    smtc_modem_suspend_radio_access( );
    // Read pin code force a key derivation
    lr1110_status_t status = lr1110_system_read_pin_custom_eui( NULL, deveui, joineui, 0, pin );
    // resume modem radio access
    smtc_modem_resume_radio_access( );

    if( status != LR1110_STATUS_OK )
    {
        return RC_FAIL;
    }
    return RC_OK;
#elif defined( LR1110_MODEM )
    uint8_t deveui[8];
    uint8_t appeui[8];
    lorawan_api_deveui_get( deveui );
    lorawan_api_appeui_key_get( appeui );

    if( smtc_crypto_se_derive_keys( deveui, appeui ) == 0 )
    {
        modem_context_appkey_is_derived( );
        return RC_OK;
    }
    return RC_FAIL;
#else
    return RC_NOT_IMPLEMENTED;
#endif
}

smtc_modem_return_code_t smtc_modem_set_output_power_lut( uint8_t config[30] )
{
    modem_context_set_power_config_lut( config );
    return RC_OK;
}

smtc_modem_return_code_t smtc_modem_get_output_power_lut( uint8_t config[30] )
{
    // get the internal lut
    modem_power_config_t* config_lut = modem_context_get_power_config_lut( );

    // Fill the tab
    for( uint8_t i = 0; i < POWER_CONFIG_LUT_SIZE; i++ )
    {
        config[5 * i]         = ( uint32_t ) config_lut[i].expected_power;
        config[( 5 * i ) + 1] = ( uint32_t ) config_lut[i].configured_power;
        config[( 5 * i ) + 2] = config_lut[i].pa_param1;
        config[( 5 * i ) + 3] = config_lut[i].pa_param2;
        config[( 5 * i ) + 4] = config_lut[i].pa_ramp_time;
    }
    return RC_OK;
}
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

bool modem_port_reserved( uint8_t f_port )
{
    return ( f_port >= 224 );
}

smtc_modem_return_code_t smtc_modem_get_dm_status_with_rate( uint8_t* dm_fields_payload, uint8_t* dm_field_length,
                                                             e_dm_info_rate_t rate )
{
    smtc_modem_return_code_t return_code = RC_OK;
    *dm_field_length                     = get_dm_info_tag_list( dm_fields_payload, rate );
    return return_code;
}

smtc_modem_return_code_t smtc_modem_set_dm_status_with_rate( const uint8_t* dm_fields_payload, uint8_t dm_field_length,
                                                             e_dm_info_rate_t rate )
{
    smtc_modem_return_code_t return_code = RC_OK;
    if( set_dm_info( dm_fields_payload, dm_field_length, rate ) == SET_ERROR )
    {
        return_code = RC_INVALID;
    }

    return return_code;
}

bool is_modem_connected( )
{
    bool ret = true;

    if( get_modem_muted( ) != MODEM_NOT_MUTE )
    {
        ret = false;
        SMTC_MODEM_HAL_TRACE_ERROR( "%s Modem is muted for %d day(s)\n", __func__, dm_get_number_of_days_mute( ) );
    }
    else if( get_modem_suspend( ) == MODEM_SUSPEND )
    {
        ret = false;
        SMTC_MODEM_HAL_TRACE_ERROR( "%s Modem is suspend\n", __func__ );
    }
    else if( ( get_join_state( ) != MODEM_JOINED ) && ( lorawan_api_is_ota_device( ) == OTAA_DEVICE ) )
    {
        ret = false;
        SMTC_MODEM_HAL_TRACE_ERROR( "%s Stack not joined or OTA device %d \n", __func__, get_join_state( ) );
    }
    return ( ret );
}

smtc_modem_return_code_t smtc_modem_send_tx( uint8_t f_port, bool confirmed, const uint8_t* payload,
                                             uint8_t payload_length, bool emergency )
{
    smtc_modem_return_code_t return_code = RC_OK;
    smodem_task              task_send;

    if( is_modem_connected( ) == false )
    {
        return_code = RC_FAIL;
    }

    else if( ( ( ( f_port == 0 ) || ( f_port >= 224 ) ) && !lorawan_api_modem_certification_is_enabled( ) ) ||
             ( f_port == get_modem_dm_port( ) ) )
    {
        return_code = RC_FAIL;
        SMTC_MODEM_HAL_TRACE_ERROR( "%s port %d is forbidden \n", __func__, f_port );
    }
    else
    {
        if( emergency == true )
        {
            task_send.priority = TASK_VERY_HIGH_PRIORITY;
            lorawan_api_duty_cycle_enable_set( false );
        }
        else
        {
            task_send.priority = TASK_HIGH_PRIORITY;
        }

        memcpy( modem_buffer, payload, payload_length );

        task_send.id                = SEND_TASK;
        task_send.fPort             = f_port;
        task_send.PacketType        = confirmed;
        task_send.dataIn            = modem_buffer;
        task_send.sizeIn            = payload_length;
        task_send.time_to_execute_s = smtc_modem_hal_get_time_in_s( );

        // SMTC_MODEM_HAL_TRACE_INFO( "add task user tx payload with payload size = %d \n ", payload_length );
        if( modem_supervisor_add_task( &task_send ) != TASK_VALID )
        {
            return_code = RC_FAIL;
        }
    }

    //@todo indicate that can't accept a new task because task already enqueued     //   SetModemBusy ();

    return return_code;
}

void empty_callback( void* ctx )
{
    // SMTC_MODEM_HAL_TRACE_ERROR( " empty call back \n" );
}

/* --- EOF ------------------------------------------------------------------ */
