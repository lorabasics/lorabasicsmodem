/*!
 * \file      modem_api.c
 *
 * \brief     soft modem API implementation
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

#include "modem_api.h"

#include "modem_context.h"
#include "lorawan_api.h"
#include "file_upload.h"
#include "modem_utilities.h"
#include "crypto.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */
static uint8_t modem_buffer[255];

static uint32_t* upload_pdata;

static uint32_t upload_size;
static uint32_t upload_avgdelay;

static radio_planner_t modem_radio_planner;

static ral_t modem_radio = { .context = NULL,
#if defined( SX1280 )
                       .radio_type = RAL_RADIO_SX1280,
                       .tcxo_cfg =
                           {
                               .tcxo_ctrl_mode = RAL_TCXO_NONE,
                           }
#elif defined( SX126X )
                       .radio_type = RAL_RADIO_SX126X,
                       .tcxo_cfg =
                           {
                               .tcxo_ctrl_mode       = RAL_TCXO_NONE,
                            //    .tcxo_ctrl_voltage    = RAL_TCXO_CTRL_1_8V,
                            //    .tcxo_startup_time_ms = 5,
                           }
#else
#error "Please select radio board.."
#endif
};

static bsp_gpio_irq_t radio_dio_x = {
    .pin      = RADIO_DIOX,
    .context  = &modem_radio_planner,
    .callback = rp_radio_irq_callback,
};

#ifdef LORAWAN_BYPASS_ENABLED
static bool stream_bypass_enabled = false;
#endif  // LORAWAN_BYPASS_ENABLED
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

bool modem_port_reserved( uint8_t f_port )
{
    return ( f_port >= 224 );
}

modem_return_code_t modem_get_dm_status_with_rate( uint8_t* dm_fields_payload, uint8_t* dm_field_length,
                                                   e_dm_info_rate_t rate )
{
    modem_return_code_t return_code = RC_OK;
    *dm_field_length                = get_dm_info_tag_list( dm_fields_payload, rate );
    return return_code;
}

modem_return_code_t modem_set_dm_status_with_rate( uint8_t* dm_fields_payload, uint8_t dm_field_length,
                                                   e_dm_info_rate_t rate )
{
    modem_return_code_t return_code = RC_OK;
    if( set_dm_info( dm_fields_payload, dm_field_length, rate ) == SET_ERROR )
    {
        return_code = RC_INVALID;
    }

    return return_code;
}

modem_return_code_t modem_send_tx( uint8_t f_port, e_tx_mode_t msg_type, uint8_t* payload, uint8_t payload_length,
                                   e_emergency_tx_t emergency )
{
    modem_return_code_t return_code = RC_OK;
    smodem_task         task_send;

    if( get_modem_muted( ) != MODEM_NOT_MUTE )
    {
        return_code = RC_FAIL;
        BSP_DBG_TRACE_ERROR( "%s Modem is muted for %d day(s)\n", __func__, dm_get_number_of_days_mute( ) );
    }
    else if( get_modem_suspend( ) == MODEM_SUSPEND )
    {
        return_code = RC_FAIL;
        BSP_DBG_TRACE_ERROR( "%s Modem is suspend\n", __func__ );
    }
    else if( ( get_join_state( ) != MODEM_JOINED ) && ( lorawan_api_is_ota_device( ) == OTAA_DEVICE ) )
    {
        return_code = RC_FAIL;
        BSP_DBG_TRACE_ERROR( "%s Stack not joined or OTA device %d \n", __func__, get_join_state( ) );
    }
    else if( ( ( ( f_port == 0 ) || ( f_port >= 224 ) ) && !lorawan_api_certification_is_enabled( ) ) ||
             ( f_port == get_modem_dm_port( ) ) )
    {
        return_code = RC_FAIL;
        BSP_DBG_TRACE_ERROR( "%s port %d is forbidden \n", __func__, f_port );
    }
    // Check and set Packet Type (Un/Confirmed)
    else if( ( msg_type != TX_UNCONFIRMED ) && ( msg_type != TX_CONFIRMED ) )
    {
        return_code = RC_FAIL;
        BSP_DBG_TRACE_ERROR( "%s mode must be TX_UNCONFIRMED or TX_CONFIRMED \n", __func__ );
    }
    else
    {
        if( emergency == TX_EMERGENCY_ON )
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
        task_send.PacketType        = msg_type;
        task_send.dataIn            = modem_buffer;
        task_send.sizeIn            = payload_length;
        task_send.time_to_execute_s = bsp_rtc_get_time_s( );

        BSP_DBG_TRACE_INFO( "add task user tx payload with payload size = %d \n ", payload_length );
        if( modem_supervisor_add_task( &task_send ) != TASK_VALID )
        {
            return_code = RC_FAIL;
        }
    }

    //@todo indicate that can't accept a new task because task already enqueued     //   SetModemBusy ();

    return return_code;
}

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void modem_init( void ( *callback )( void ) )
{
    // init radio and put it in sleep mode
    ral_init( &modem_radio );
    ral_set_sleep( &modem_radio );

    // init radio planner and attach corresponding radio irq
    rp_init( &modem_radio_planner, &modem_radio );
    bsp_gpio_irq_attach( &radio_dio_x );

    // init modem supervisor
    modem_supervisor_init( callback, &modem_radio_planner );
}

uint32_t modem_run_engine( void )
{
    return modem_supervisor_engine( );
}

modem_return_code_t modem_get_event( modem_rsp_event_t* type, uint8_t* count, uint8_t* event_data,
                                     uint8_t* event_data_length, uint8_t* asynchronous_msgnumber )
{
    modem_return_code_t return_code = RC_OK;
    modem_rsp_event_t   msg_event   = get_last_msg_event( );
    *event_data_length              = 0;

    *asynchronous_msgnumber = get_asynchronous_msgnumber( );
    if( *asynchronous_msgnumber > RSP_NUMBER )
    {
        BSP_DBG_TRACE_ERROR( "build_get_event_cmd_response, asynchronous_msgnumber overlap" );
        *asynchronous_msgnumber = 0;
        *type                   = RSP_NUMBER;
        return_code             = RC_FAIL;
    }
    else if( *asynchronous_msgnumber > 0 )
    {
        *type = msg_event;

        // event_data[1] store the counter of overrun event so in case of just one event this counter is set to 0
        // => -1
        *count = get_modem_event_count( msg_event ) - 1;
        // insert the count field in case of overrun
        switch( msg_event )
        {
        case RSP_JOINED:
            *event_data_length = 0;
            break;
        case RSP_RESET:  // count in case of overrun should never append because reset !!!!
            *event_data_length = 2;
            event_data[0]      = ( lorawan_api_nb_reset_get( ) ) & 0xFF;
            event_data[1]      = ( lorawan_api_nb_reset_get( ) ) >> 8;
            break;
        case RSP_ALARM:
            *event_data_length = 0;
            break;
        case RSP_DOWNDATA: {
            s_modem_dwn_t dwnframe;
            get_modem_downlink_frame( &dwnframe );
            *event_data_length = 4 + dwnframe.length;
            event_data[0]      = dwnframe.rssi;
            event_data[1]      = dwnframe.snr;
            // TODO UL_ACK() | lorawan_api_GetRxWindow()
            event_data[2] = 0;  // ;
            event_data[3] = dwnframe.port;
            memcpy( &event_data[4], dwnframe.data, dwnframe.length );
            break;
        }
        case RSP_FILEDONE:
            *event_data_length = 1;
            event_data[0]      = get_modem_event_status( msg_event );
            break;
        case RSP_TXDONE:
            *event_data_length = 1;
            event_data[0]      = get_modem_event_status( msg_event );
            break;
        case RSP_SETCONF:
            *event_data_length = 1;
            event_data[0]      = get_modem_event_status( msg_event );
            break;
        case RSP_MUTE:
            *event_data_length = 1;
            event_data[0]      = ( get_modem_muted( ) == MODEM_NOT_MUTE ) ? false : true;
            break;
        case RSP_LINKSTATUS:
            // @ TODO
            break;
        default:
            // already manage by the first test in the function
            break;
        }
        // Reset the status after get the value
        set_modem_event_count( msg_event, 0 );
        decrement_asynchronous_msgnumber( );
    }
    else
    {
        *asynchronous_msgnumber = 0;
        *type                   = RSP_NUMBER;
    }

    return return_code;
}

modem_return_code_t modem_get_version( uint32_t* bootloader, uint32_t* firmware, uint16_t* lorawan )
{
    modem_return_code_t return_code = RC_OK;
    *bootloader                     = 0xffffffff;
    *firmware                       = FW_VERSION;
    *lorawan                        = 0x0103;
    return return_code;
}

modem_return_code_t modem_reset( void )
{
    modem_return_code_t return_code = RC_OK;
    modem_store_context( );
    set_modem_reset_requested( true );
    return return_code;
}

modem_return_code_t modem_factory_reset( void )
{
    modem_return_code_t return_code = RC_OK;
    lorawan_api_factory_reset( );
    modem_context_factory_reset( );
    return return_code;
}

modem_return_code_t modem_reset_charge( void )
{
    modem_return_code_t return_code = RC_OK;
    reset_modem_charge( );
    return return_code;
}

modem_return_code_t modem_get_charge( uint32_t* charge )
{
    modem_return_code_t return_code = RC_OK;
    *charge                         = get_modem_charge_ma_h( );
    return return_code;
}

modem_return_code_t modem_get_tx_power_offset( int8_t* tx_pwr_offset )
{
    modem_return_code_t return_code = RC_OK;
    *tx_pwr_offset                  = lorawan_api_tx_power_offset_get( );
    return return_code;
}

modem_return_code_t modem_set_tx_power_offset( int8_t tx_pwr_offset )
{
    modem_return_code_t return_code = RC_OK;

    lorawan_api_tx_power_offset_set( tx_pwr_offset );

    return return_code;
}

modem_return_code_t modem_get_status( uint8_t* status )
{
    modem_return_code_t return_code = RC_OK;
    *status                         = get_modem_status( );
    return return_code;
}

modem_return_code_t modem_set_alarm_timer_s( uint32_t alarm )
{
    modem_return_code_t return_code = RC_OK;
    modem_set_user_alarm( ( alarm > 0 ) ? ( bsp_rtc_get_time_s( ) + alarm ) : 0 );
    return return_code;
}

modem_return_code_t modem_get_joineui( uint8_t* joineui )
{
    modem_return_code_t return_code = RC_OK;
    lorawan_api_appeui_key_get( joineui );
    return return_code;
}

modem_return_code_t modem_set_joineui( uint8_t joineui[8] )
{
    modem_return_code_t return_code = RC_OK;

    if( get_join_state( ) != MODEM_NOT_JOINED )  // the modem have to be leave from the network to modify the key
    {
        return_code = RC_BUSY;
        BSP_DBG_TRACE_ERROR( "%s call but the device is already join\n", __func__ );
    }
    else
    {
        lorawan_api_appeui_key_set( joineui );
    }

    return return_code;
}

modem_return_code_t modem_get_deveui( uint8_t* deveui )
{
    modem_return_code_t return_code = RC_OK;
    lorawan_api_deveui_get( deveui );
    return return_code;
}

modem_return_code_t modem_set_deveui( uint8_t deveui[8] )
{
    modem_return_code_t return_code = RC_OK;
    if( get_join_state( ) != MODEM_NOT_JOINED )  // the modem have to be leave from the network to modify the key
    {
        return_code = RC_BUSY;
        BSP_DBG_TRACE_ERROR( "%s call but the device is already join\n", __func__ );
    }
    else
    {
        lorawan_api_deveui_set( deveui );
    }
    return return_code;
}

modem_return_code_t modem_set_nwkkey( uint8_t nwkkey[16] )
{
    modem_return_code_t return_code = RC_OK;
    if( get_join_state( ) != MODEM_NOT_JOINED )  // the modem have to be leave from the network to modify the key
    {
        return_code = RC_NOT_INIT;
        BSP_DBG_TRACE_ERROR( "%s call but the device is already join\n", __func__ );
    }
    else
    {
        lorawan_api_app_key_set( nwkkey );
    }
    return return_code;
}

modem_return_code_t modem_get_class( modem_class_t* class )
{
    modem_return_code_t return_code = RC_OK;
    *class                          = get_modem_class( );
    return return_code;
}

modem_return_code_t modem_set_class( modem_class_t class )
{
    modem_return_code_t return_code = RC_OK;
    if( set_modem_class( class ) == SET_ERROR )
    {
        return_code = RC_INVALID;
        BSP_DBG_TRACE_ERROR( "%s call with class not valid\n", __func__ );
    }
    return return_code;
}

modem_return_code_t modem_get_region( uint8_t* region )
{
    modem_return_code_t return_code = RC_OK;
    *region                         = get_modem_region( );
    return return_code;
}

modem_return_code_t modem_set_region( uint8_t region )
{
    modem_return_code_t return_code = RC_OK;
    if( get_join_state( ) != MODEM_NOT_JOINED )
    {
        BSP_DBG_TRACE_ERROR( "%s call but the device is already join\n", __func__ );
        return RC_BUSY;
    }

    if( set_modem_region( region ) == SET_ERROR )
    {
        BSP_DBG_TRACE_ERROR( "%s call with region not valid\n", __func__ );
        return RC_INVALID;
    }

    lorawan_api_dr_strategy_set( STATIC_ADR_MODE );
    return return_code;
}

modem_return_code_t modem_list_region( uint8_t* region_list, uint8_t* nb_region )
{
    modem_return_code_t return_code = RC_OK;

    memcpy( region_list, smtc_real_region_list, SMTC_REAL_REGION_LIST_LENGTH );
    *nb_region = SMTC_REAL_REGION_LIST_LENGTH;
    return return_code;
}

modem_return_code_t modem_get_adr_profile( dr_strategy_t* adr_profile )
{
    modem_return_code_t return_code = RC_OK;
    *adr_profile                    = get_modem_adr( );
    return return_code;
}

modem_return_code_t modem_set_adr_profile( dr_strategy_t adr_profile, uint8_t adr_custom_data[16] )
{
    modem_return_code_t return_code = RC_OK;
    e_set_error_t       status;

    if( adr_profile < USER_DR_DISTRIBUTION )
    {
        status = set_modem_adr_profile( adr_profile, adr_custom_data, 0 );
    }
    else if( adr_profile == USER_DR_DISTRIBUTION )
    {
        status = set_modem_adr_profile( adr_profile, adr_custom_data, 16 );
    }
    else
    {
        status = SET_ERROR;
    }

    if( status == SET_ERROR )
    {
        return_code = RC_INVALID;
        BSP_DBG_TRACE_ERROR( "%s call with adr profile not valid\n", __func__ );
    }
    return return_code;
}

modem_return_code_t modem_get_dm_port( uint8_t* dm_port )
{
    modem_return_code_t return_code = RC_OK;
    *dm_port                        = get_modem_dm_port( );
    return return_code;
}

modem_return_code_t modem_set_dm_port( uint8_t dm_port )
{
    modem_return_code_t return_code = RC_OK;
    if( set_modem_dm_port( dm_port ) == SET_ERROR )
    {
        return_code = RC_INVALID;
        BSP_DBG_TRACE_ERROR( "%s call with DM port not valid\n", __func__ );
    }

    return return_code;
}

modem_return_code_t modem_get_dm_info_interval( uint8_t* interval )
{
    modem_return_code_t return_code = RC_OK;
    *interval                       = get_modem_dm_interval( );
    return return_code;
}

modem_return_code_t modem_set_dm_info_interval( uint8_t interval )
{
    modem_return_code_t return_code = RC_OK;
    if( set_modem_dm_interval( interval ) == SET_ERROR )
    {
        return_code = RC_INVALID;
        BSP_DBG_TRACE_ERROR( "%s call but interval not valid\n", __func__ );
    }
    else
    {
        modem_supervisor_add_task_dm_status( get_modem_dm_interval_second( ) );
    }
    return return_code;
}

modem_return_code_t modem_get_dm_info_fields( uint8_t* dm_fields_payload, uint8_t* dm_field_length )
{
    modem_return_code_t return_code = RC_OK;
    return_code = modem_get_dm_status_with_rate( dm_fields_payload, dm_field_length, DM_INFO_PERIODIC );
    return return_code;
}

modem_return_code_t modem_set_dm_info_fields( uint8_t* dm_fields_payload, uint8_t dm_field_length )
{
    modem_return_code_t return_code = RC_OK;
    return_code = modem_set_dm_status_with_rate( dm_fields_payload, dm_field_length, DM_INFO_PERIODIC );
    return return_code;
}

modem_return_code_t modem_send_dm_status( uint8_t* dm_fields_payload, uint8_t dm_field_length )
{
    modem_return_code_t return_code = RC_OK;
    return_code                     = modem_set_dm_status_with_rate( dm_fields_payload, dm_field_length, DM_INFO_NOW );
    if( return_code == RC_OK )
    {
        modem_supervisor_add_task_dm_status_now( );
    }
    return return_code;
}

modem_return_code_t modem_set_app_status( uint8_t app_status[dm_info_field_sz[e_inf_appstatus]],
                                          uint8_t app_status_length )
{
    modem_return_code_t return_code = RC_OK;
    if( app_status_length != dm_info_field_sz[e_inf_appstatus] )
    {
        return_code = RC_BAD_SIZE;
        BSP_DBG_TRACE_ERROR( "%s call with a size not valid\n", __func__ );
    }
    else
    {
        set_modem_appstatus( app_status );
    }
    return return_code;
}

modem_return_code_t modem_join( void )
{
    modem_return_code_t return_code = RC_OK;

    BSP_DBG_TRACE_MSG( "Process MODEM_JOIN cmd\n" );
    if( lorawan_api_is_ota_device( ) == ABP_DEVICE )
    {
        return_code = RC_FAIL;
        BSP_DBG_TRACE_ERROR( "%s call but the device is configured in ABP mode\n", __func__ );
    }
    else if( get_join_state( ) != MODEM_NOT_JOINED )  // the modem have to be leave from the network to join
    {
        return_code = RC_BUSY;
        BSP_DBG_TRACE_ERROR( "%s call but the device is already join\n", __func__ );
    }
    else if( get_modem_muted( ) == MODEM_INFINITE_MUTE )
    {
        return_code = RC_FAIL;
        BSP_DBG_TRACE_ERROR( "%s Modem is muted\n", __func__ );
    }
    else if( get_modem_suspend( ) == MODEM_SUSPEND )
    {
        return_code = RC_FAIL;
        BSP_DBG_TRACE_ERROR( "%s Modem is suspend\n", __func__ );
    }
    else
    {
        modem_supervisor_add_task_join( );
    }

    return return_code;
}

modem_return_code_t modem_leave_network( void )
{
    modem_return_code_t return_code = RC_OK;
    set_modem_status_modem_joined( false );
    lorawan_api_join_status_clear( );
    set_modem_status_joining( false );
    return return_code;
}

modem_return_code_t modem_suspend_modem_comm( e_modem_suspend_t suspend )
{
    modem_return_code_t return_code = RC_OK;
    if( set_modem_suspend( suspend ) == SET_ERROR )
    {
        return_code = RC_INVALID;
        BSP_DBG_TRACE_ERROR( "%s call but suspend value is not valid\n", __func__ );
    }
    return return_code;
}

modem_return_code_t modem_get_next_tx_max_payload( uint8_t* tx_max_payload )
{
    modem_return_code_t return_code = RC_OK;
    if( get_join_state( ) == MODEM_JOINED )
    {
        *tx_max_payload = lorawan_api_next_max_payload_length_get( );
    }
    else
    {
        BSP_DBG_TRACE_ERROR( "%s call but the device is not join\n", __func__ );
        *tx_max_payload = 255;
        return_code     = RC_NOT_INIT;
    }

    return return_code;
}

modem_return_code_t modem_request_tx( uint8_t f_port, e_tx_mode_t msg_type, uint8_t* payload, uint8_t payload_length )
{
    modem_return_code_t return_code = RC_OK;
    return_code                     = modem_send_tx( f_port, msg_type, payload, payload_length, TX_EMERGENCY_OFF );
    return return_code;
}

modem_return_code_t modem_emergency_tx( uint8_t f_port, e_tx_mode_t msg_type, uint8_t* payload, uint8_t payload_length )
{
    modem_return_code_t return_code = RC_OK;
    return_code                     = modem_send_tx( f_port, msg_type, payload, payload_length, TX_EMERGENCY_ON );
    return return_code;
}

modem_return_code_t modem_upload_init( uint8_t f_port, file_upload_encrypt_mode_t encryption_mode, uint16_t size,
                                       uint16_t average_delay )
{
    modem_return_code_t return_code = RC_OK;

    if( ( f_port == 0 ) || ( f_port >= 224 ) || ( encryption_mode > FILE_UPLOAD_ENCRYPTED ) )
    {
        return_code = RC_INVALID;
    }
    else
    {
        // upload_clear(UPLOAD_SID);
        set_modem_status_file_upload( false );
        upload_size = size;

        if( upload_size == 0 )
        {
            BSP_DBG_TRACE_WARNING( "FileUpload Cancel!\n" );
            modem_set_upload_state( MODEM_UPLOAD_NOT_INIT );
        }
        else if( modem_get_upload_state( ) != MODEM_UPLOAD_NOT_INIT )
        {
            return_code = RC_BUSY;
            BSP_DBG_TRACE_ERROR( "File Upload still in going\n" );
        }
        else
        {
            upload_avgdelay = average_delay;
            int8_t session_counter =
                file_upload_create( UPLOAD_SID, &upload_pdata, upload_size, upload_avgdelay, f_port, encryption_mode );

            if( session_counter < 0 )
            {
                return_code = RC_FAIL;
            }
            else
            {
                modem_set_upload_state( MODEM_UPLOAD_INIT );
            }
        }
    }

    return return_code;
}

modem_return_code_t modem_upload_start( uint8_t* payload, uint16_t payload_length )
{
    modem_return_code_t return_code = RC_OK;
    if( payload == NULL )
    {
        return_code = RC_NOT_INIT;
        BSP_DBG_TRACE_ERROR( "Upload file data, null\n" );
    }
    else
    {
        upload_pdata = ( uint32_t* ) payload;
        file_upload_attach_payload_buffer( ( uint8_t* ) upload_pdata );

        modem_set_upload_state( MODEM_UPLOAD_DATA );
        if( modem_get_upload_state( ) == MODEM_UPLOAD_START )
        {
            return_code = RC_BUSY;
            BSP_DBG_TRACE_ERROR( "FileUpload in progress..\n" );
        }
        else if( modem_get_upload_state( ) != MODEM_UPLOAD_DATA )
        {
            return_code = RC_NOT_INIT;
            BSP_DBG_TRACE_ERROR( "RC_NOT_INIT in FileUpload bad state\n" );
        }

        else if( payload_length != upload_size )
        {
            return_code = RC_BAD_SIZE;
            BSP_DBG_TRACE_ERROR( "RC_BAD_SIZE in FileUpload upload_current_size = %u and upload_size %u \n",
                                 payload_length, upload_size );
        }

        else
        {
            uint32_t hash[8];
            sha256( hash, ( unsigned char* ) upload_pdata, upload_size );
            file_upload_set_hash( hash[0], hash[1] );

            if( file_upload_get_encryption_mode( ) == FILE_UPLOAD_ENCRYPTED )
            {
                uint32_t temp_hash = hash[1];
                lora_crypto_payload_encrypt( ( uint8_t* ) upload_pdata, upload_size, lorawan_api_apps_key_get( ),
                                             upload_size, FILE_UPLOAD_DIRECTION, hash[0], ( uint8_t* ) upload_pdata );
                // compute hash over encrypted data
                sha256( hash, ( unsigned char* ) upload_pdata, upload_size );
                file_upload_set_hash( hash[0], temp_hash );
            }
            // start streaming of chunks

            smodem_task upload_task;
            upload_task.id                = FILE_UPLOAD_TASK;
            upload_task.priority          = TASK_HIGH_PRIORITY;
            upload_task.time_to_execute_s = bsp_rtc_get_time_s( ) + 2;
            if( modem_supervisor_add_task( &upload_task ) != TASK_VALID )
            {
                return_code = RC_FAIL;
            }
            else
            {
                modem_set_upload_state( MODEM_UPLOAD_START );
            }
        }
    }
    return return_code;
}
uint16_t modem_upload_avgdelay_get( void )
{
    return upload_avgdelay;
}

radio_planner_t* modem_get_radio_planner( void )
{
    return &modem_radio_planner;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
