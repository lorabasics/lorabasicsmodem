/*!
 * \file      modem_context.c
 *
 * \brief     share functions + context of the soft modem .
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

#include "modem_context.h"
#include "smtc_bsp.h"
#include "smtc_real.h"
#include "device_management_defs.h"
#include "lorawan_api.h"
#include "modem_utilities.h"  // for crc
#include "modem_api.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static uint8_t                modem_status         = 0;
static uint8_t                modem_dm_interval    = DEFAULT_DM_REPORTING_INTERVAL;
static uint8_t                modem_dm_port        = DEFAULT_DM_PORT;
static uint8_t                modem_appstatus[8]   = { 0 };
static modem_class_t          modem_dm_class       = MODEM_CLASS_A;
static e_modem_suspend_t      is_modem_suspend     = MODEM_NOT_SUSPEND;
static uint32_t               modem_start_time     = 0;
static uint8_t                modem_dm_upload_sctr = 0;
static e_modem_upload_state_t modem_upload_state   = MODEM_UPLOAD_NOT_INIT;
static s_modem_stream_t       modem_stream_state   = {  //
    .port       = DEFAULT_DM_PORT,              //
    .state      = MODEM_STREAM_NOT_INIT,        //
    .encryption = false
};

static uint32_t dm_info_bitfield_periodic       = DEFAULT_DM_REPORTING_FIELDS;  // context for periodic GetInfo
static uint32_t dm_info_bitfield_now            = 0;                            // User GetInfo
static uint8_t  tag_number                      = 0;
static uint8_t  tag_number_now                  = 0;
static uint8_t  number_of_muted_day             = 0;
static s_dm_retrieve_pending_dl_t dm_pending_dl = { .up_count = 0, .up_delay = 0 };
static uint32_t                   user_alarm    = 0;

static uint8_t asynchronous_msgnumber = 0;

static uint8_t modem_event_count[RSP_NUMBER];
static uint8_t modem_event_status[RSP_NUMBER];
static uint8_t asynch_msg[RSP_NUMBER];

static s_modem_dwn_t modem_dwn_pkt;
static bool          is_modem_reset_requested = false;
static bool          is_modem_charge_loaded   = false;
static uint32_t      modem_charge_offset      = 0;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/*!
 * \brief   convert requested DM bytes fields to bitfield
 *
 * \param [in]  requested_info_list     Array of bytes with requested DM code in
 * each bytes \param [in]  len                     Number of byte that composed
 * requested_info_list \param [in]  bitfields *             Returned bitfield
 * \param [out] e_set_error_t             Return SET_OK in case of failure, else
 * false
 */
static e_set_error_t convert_requested_dm_info_bytes_to_bitfield( uint8_t* requested_info_list, uint8_t len,
                                                                  uint32_t* bitfields )
{
    e_set_error_t ret = SET_OK;
    for( uint8_t i = 0; i < len; i++ )
    {
        // Ignore DM status with variable length and forbiden fields
        if( ( requested_info_list[i] == e_inf_crashlog ) || ( requested_info_list[i] == e_inf_upload ) ||
            ( requested_info_list[i] == e_inf_stream ) || ( requested_info_list[i] == e_inf_alcsync ) ||
            ( requested_info_list[i] == e_inf_rfu_0 ) || ( requested_info_list[i] == e_inf_rfu_1 ) ||
            ( requested_info_list[i] >= e_inf_max ) )
        {
            ret = SET_ERROR;
            BSP_DBG_TRACE_ERROR( "invalid DM info code (0x%02x)\n", requested_info_list[i] );
            return ret;
        }
    }
    // Reset bitfield
    *bitfields = 0;
    for( uint8_t i = 0; i < len; i++ )
    {
        *bitfields |= ( 1 << requested_info_list[i] );
    }
    return ret;
}

/*!
 * \brief   Check if the biggest requested DM status field can be inserted
 *          in the payload in regard of the max payload size requested
 *
 * \param [in]  info_requested              Requested bitfield
 * \param [in]  max_size                    Max size of the payload
 * \param [out] e_dm_cmd_length_valid       Return valid or not
 */
static e_dm_cmd_length_valid check_dm_status_max_size( uint32_t info_requested, uint8_t max_size )
{
    for( uint8_t i = 0; i < e_inf_max; i++ )
    {
        if( ( info_requested & ( 1 << i ) ) )
        {
            if( max_size < dm_info_field_sz[i] )
            {
                BSP_DBG_TRACE_ERROR(
                    "max_size must be greater than the smallest requested "
                    "information\n" );
                return DM_CMD_LENGTH_NOT_VALID;
            }
        }
    }
    return DM_CMD_LENGTH_VALID;
}

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void modem_event_init( void )
{
    for( int i = 0; i < RSP_NUMBER; i++ )
    {
        set_modem_event_count( ( modem_rsp_event_t ) i, 0 );
        set_modem_event_status( ( modem_rsp_event_t ) i, 0 );
    }
}

uint8_t get_modem_event_count( modem_rsp_event_t event_type )
{
    if( event_type >= RSP_NUMBER )
    {
        bsp_mcu_panic( );
    }

    return ( modem_event_count[event_type] );
}

void set_modem_event_count( modem_rsp_event_t event_type, uint8_t value )
{
    if( event_type < RSP_NUMBER )
    {
        modem_event_count[event_type] = value;
    }
}

void increment_modem_event_count( modem_rsp_event_t event_type )
{
    if( event_type < RSP_NUMBER )
    {
        if( modem_event_count[event_type] < 255 )
        {
            modem_event_count[event_type]++;
        }
    }
}

uint8_t get_modem_event_status( modem_rsp_event_t event_type )
{
    if( event_type >= RSP_NUMBER )
    {
        bsp_mcu_panic( );
    }
    return ( modem_event_status[event_type] );
}

void set_modem_event_status( modem_rsp_event_t event_type, uint8_t status )
{
    if( event_type < RSP_NUMBER )
    {
        modem_event_status[event_type] = status;
    }
}

void decrement_asynchronous_msgnumber( void )
{
    if( asynchronous_msgnumber > 0 )
    {
        asynchronous_msgnumber--;
    }
    else
    {
        asynchronous_msgnumber = 0;
    }
}

uint8_t get_asynchronous_msgnumber( void )
{
    return ( asynchronous_msgnumber );
}

void increment_asynchronous_msgnumber( modem_rsp_event_t event_type, uint8_t status )
{
    // Next condition should never append because only one asynch msg by type of message
    if( asynchronous_msgnumber > RSP_NUMBER )
    {
        BSP_DBG_TRACE_ERROR( " Modem reach the max number of asynch message" );
        return;
    }
    uint8_t tmp;
    tmp = get_modem_event_count( event_type );
    if( tmp == 0 )
    {
        asynchronous_msgnumber++;
    }

    increment_modem_event_count( event_type );
    set_modem_event_status( event_type, status );

    switch( event_type )
    {
    case RSP_JOINED:
        BSP_DBG_TRACE_INFO( "increment event RSP_JOINED\n" );
        break;
    case RSP_ALARM:
        BSP_DBG_TRACE_INFO( "increment event RSP_ALARM\n" );
        break;
    case RSP_DOWNDATA:
        BSP_DBG_TRACE_INFO( "increment event RSP_DOWNDATA\n" );
        break;
    case RSP_TXDONE:
        BSP_DBG_TRACE_INFO( "event count tx done = %d\n", get_modem_event_count( RSP_TXDONE ) );
        break;
    case RSP_FILEDONE:
        BSP_DBG_TRACE_INFO( "increment event RSP_FILEDONE\n" );
        break;
    case RSP_STREAMDONE:
        BSP_DBG_TRACE_INFO( "increment event RSP_STREAMDONE\n" );
        break;
    case RSP_SETCONF:
        BSP_DBG_TRACE_INFO( "increment event RSP_SETCONF\n" );
        break;
    case RSP_MUTE:
        BSP_DBG_TRACE_INFO( "increment event RSP_MUTE\n" );
        break;
    default:

        break;
    }
    asynch_msg[asynchronous_msgnumber] = event_type;
}

modem_rsp_event_t get_last_msg_event( void )
{
    return ( modem_rsp_event_t ) asynch_msg[asynchronous_msgnumber];
}

uint32_t get_modem_uptime_s( void )
{
    return ( bsp_rtc_get_time_s( ) - modem_start_time );
}

void set_modem_start_time_s( uint32_t time )
{
    static bool was_set = false;
    if( !was_set )
    {
        was_set          = true;
        modem_start_time = time;
    }
}

e_set_error_t set_modem_dm_interval( uint8_t interval )
{
    if( modem_dm_interval != interval )
    {
        modem_dm_interval = interval;
        modem_store_context( );
    }

    return ( SET_OK );
}
uint8_t get_modem_dm_interval( void )
{
    return ( modem_dm_interval );
}
uint32_t get_modem_dm_interval_second( void )
{
    uint8_t  dm_interval = get_modem_dm_interval( );
    uint32_t temp;
    switch( ( dm_interval >> 6 ) & 0x03 )
    {
    case DM_INTERVAL_UNIT_SEC:
        temp = ( dm_interval & 0x3F );
        break;
    case DM_INTERVAL_UNIT_DAY:
        temp = ( dm_interval & 0x3F ) * 3600 * 24;
        break;
    case DM_INTERVAL_UNIT_HOUR:
        temp = ( dm_interval & 0x3F ) * 3600;
        break;
    case DM_INTERVAL_UNIT_MIN:
        temp = ( dm_interval & 0x3F ) * 60;
        break;
    default:  // never reach
        BSP_DBG_TRACE_ERROR( "in get_modem_dm_interval_second \n" );
        bsp_mcu_panic( );
        break;
    }
    return temp;
}

e_set_error_t set_modem_class( modem_class_t LoRaWAN_class )
{
    if( LoRaWAN_class != MODEM_CLASS_A )
    {
        BSP_DBG_TRACE_ERROR( "modem class invalid" );
        return ( SET_ERROR );
    }
    else
    {
        modem_dm_class = LoRaWAN_class;
        return ( SET_OK );
    }
}

modem_class_t get_modem_class( void )
{
    return ( modem_dm_class );
}

e_set_error_t set_modem_dm_port( uint8_t port )
{
    if( ( port == 0 ) || ( port >= 224 ) )
    {
        BSP_DBG_TRACE_ERROR( "modem port invalid\n" );
        return ( SET_ERROR );
    }
    else
    {
        if( modem_dm_port != port )
        {
            modem_dm_port = port;
            modem_store_context( );
        }
        return ( SET_OK );
    }
}

uint8_t get_modem_dm_port( void )
{
    return ( modem_dm_port );
}

dr_strategy_t get_modem_adr( void )
{
    uint8_t user_dr = ( uint8_t )( lorawan_api_dr_strategy_get( ) );
    if( user_dr > USER_DR_DISTRIBUTION )
    {
        BSP_DBG_TRACE_ERROR( "unknown default case : get_modem_adr \n" );
        bsp_mcu_panic( );
        return 0;  // not reached
    }
    else
    {
        return ( dr_strategy_t ) user_dr;
    }
}

uint8_t get_modem_region( void )
{
    return lorawan_api_get_region( );
}

e_set_error_t set_modem_region( uint8_t region )
{
    if( lorawan_api_set_region( ( smtc_real_region_types_t ) region ) != OKLORAWAN )
    {
        return SET_ERROR;
    }
    return SET_OK;
}

eModemJoinState_t get_join_state( void )
{
    eModemJoinState_t joinstate;
    if( get_modem_status_joining( ) == true )
    {
        joinstate = MODEM_JOIN_ONGOING;
    }
    else if( lorawan_api_isjoined( ) == NOT_JOINED )
    {
        joinstate = MODEM_NOT_JOINED;
    }
    else
    {
        joinstate = MODEM_JOINED;
    }
    return ( joinstate );
}

void set_modem_appstatus( uint8_t* app_status )
{
    memcpy( modem_appstatus, app_status, dm_info_field_sz[e_inf_appstatus] );
}

void get_modem_appstatus( uint8_t* app_status )
{
    memcpy( app_status, modem_appstatus, dm_info_field_sz[e_inf_appstatus] );
}

void modem_supervisor_add_task_join( void )
{
    smodem_task task_join;
    task_join.id       = JOIN_TASK;
    task_join.priority = TASK_HIGH_PRIORITY;
#ifdef TEST_BYPASS_JOIN_DUTY_CYCLE
    task_join.time_to_execute_s = bsp_rng_get_random_in_range( 0, 5 );
#else
    task_join.time_to_execute_s = lorawan_api_next_join_time_second_get( ) + bsp_rng_get_random_in_range( 0, 5 );
#endif
    BSP_DBG_TRACE_PRINTF( " Start a New join at %ld seconds \n", task_join.time_to_execute_s );
    set_modem_status_joining( true );
    modem_supervisor_add_task( &task_join );
}

void modem_supervisor_add_task_dm_status( uint32_t next_execute )
{
    smodem_task task_dm;
    task_dm.id                = DM_TASK;
    task_dm.priority          = TASK_LOW_PRIORITY;
    task_dm.PacketType        = UNCONF_DATA_UP;
    task_dm.time_to_execute_s = bsp_rtc_get_time_s( ) + next_execute;
    if( get_join_state( ) == MODEM_JOINED )
    {
        modem_supervisor_add_task( &task_dm );
    }
}

void modem_supervisor_add_task_dm_status_now( void )
{
    smodem_task task_dm;
    task_dm.id         = DM_TASK_NOW;
    task_dm.priority   = TASK_LOW_PRIORITY;
    task_dm.PacketType = UNCONF_DATA_UP;
    task_dm.time_to_execute_s =
        bsp_rtc_get_time_s( ) + bsp_rng_get_random_in_range( DM_STATUS_NOW_MIN_TIME, DM_STATUS_NOW_MAX_TIME );
    modem_supervisor_add_task( &task_dm );
}

void modem_supervisor_add_task_alc_sync_time_req( uint32_t next_execute )
{
    smodem_task task_dm;
    task_dm.id                = ALC_SYNC_TIME_REQ_TASK;
    task_dm.priority          = TASK_HIGH_PRIORITY;
    task_dm.PacketType        = UNCONF_DATA_UP;
    task_dm.time_to_execute_s = bsp_rtc_get_time_s( ) + next_execute;
    if( get_join_state( ) == MODEM_JOINED )
    {
        modem_supervisor_add_task( &task_dm );
    }
}

void modem_supervisor_add_task_alc_sync_ans( uint32_t next_execute )
{
    smodem_task task_dm;
    task_dm.id                = ALC_SYNC_ANS_TASK;
    task_dm.priority          = TASK_HIGH_PRIORITY;
    task_dm.PacketType        = UNCONF_DATA_UP;
    task_dm.time_to_execute_s = bsp_rtc_get_time_s( ) + next_execute;
    if( get_join_state( ) == MODEM_JOINED )
    {
        modem_supervisor_add_task( &task_dm );
    }
}

void modem_supervisor_add_task_modem_mute( void )
{
    smodem_task task_dm;
    task_dm.id                = MUTE_TASK;
    task_dm.priority          = TASK_MEDIUM_HIGH_PRIORITY;
    task_dm.time_to_execute_s = bsp_rtc_get_time_s( ) + 86400;  // Every 24h
    modem_supervisor_add_task( &task_dm );
}

void modem_supervisor_add_task_retrieve_dl( uint32_t next_execute )
{
    BSP_DBG_TRACE_WARNING( "modem_supervisor_add_task_retrieve_dl\n" );
    smodem_task task_dm;
    task_dm.id                = RETRIEVE_DL_TASK;
    task_dm.priority          = TASK_LOW_PRIORITY;
    task_dm.PacketType        = UNCONF_DATA_UP;
    task_dm.sizeIn            = 0;
    task_dm.time_to_execute_s = bsp_rtc_get_time_s( ) + next_execute;
    if( get_join_state( ) == MODEM_JOINED )
    {
        modem_supervisor_add_task( &task_dm );
    }
}

void modem_supervisor_add_task_stream( void )
{
    // Modem supervisor copy everything,
    // so this is safe even when it is going to be invalidated.
    smodem_task stream_task;

    stream_task.id                = STREAM_TASK;
    stream_task.time_to_execute_s = bsp_rtc_get_time_s( ) + 1;  // TODO define when to launch the task
    stream_task.priority          = TASK_HIGH_PRIORITY;
    stream_task.fPort             = modem_get_stream_port( );
    // stream_task.dataIn        not used in task
    // stream_task.sizeIn        not used in task
    // stream_task.PacketType    not used in task
    modem_supervisor_add_task( &stream_task );
    BSP_DBG_TRACE_INFO( "Supervisor add stream task\n" );
}

/*!
 * \brief   return the modem status
 * \remark
 * \param [IN] void
 * \param [OUT] modem status bit 6 : streaming in progress
 *                           bit 5 : file upload in progress
 *                           bit 4 : radio suspend
 *                           bit 3 : modem join
 *                           bit 2 : modem mute
 *                           bit 1 : reset after panic
 *                           bit 0 : reset after brownout
 * */
uint8_t get_modem_status( void )
{
    // If the stack is no more join, the modem status was not aware of the disconnection
    if( get_join_state( ) != MODEM_JOINED )
    {
        set_modem_status_modem_joined( false );
    }
    return ( modem_status );
}

void set_modem_status_reset_after_brownout( bool value )
{
    modem_status = ( value == true ) ? ( modem_status | ( 1 << modem_status_brownout ) )
                                     : ( modem_status & ~( 1 << modem_status_brownout ) );
}

void set_modem_status_reset_after_crash( bool value )
{
    modem_status = ( value == true ) ? ( modem_status | ( 1 << modem_status_crash ) )
                                     : ( modem_status & ~( 1 << modem_status_crash ) );
}

void set_modem_status_modem_mute( bool value )
{
    modem_status = ( value == true ) ? ( modem_status | ( 1 << modem_status_mute ) )
                                     : ( modem_status & ~( 1 << modem_status_mute ) );
}

void set_modem_status_modem_joined( bool value )
{
    modem_status = ( value == true ) ? ( modem_status | ( 1 << modem_status_joined ) )
                                     : ( modem_status & ~( 1 << modem_status_joined ) );
}

void set_modem_status_radio_suspend( bool value )
{
    modem_status = ( value == true ) ? ( modem_status | ( 1 << modem_status_suspend ) )
                                     : ( modem_status & ~( 1 << modem_status_suspend ) );
}

void set_modem_status_file_upload( bool value )
{
    modem_status = ( value == true ) ? ( modem_status | ( 1 << modem_status_upload ) )
                                     : ( modem_status & ~( 1 << modem_status_upload ) );
}

void set_modem_status_joining( bool value )
{
    modem_status = ( value == true ) ? ( modem_status | ( 1 << modem_status_joining ) )
                                     : ( modem_status & ~( 1 << modem_status_joining ) );
}

void set_modem_status_streaming( bool value )
{
    modem_status = ( value == true ) ? ( modem_status | ( 1 << modem_status_streaming ) )
                                     : ( modem_status & ~( 1 << modem_status_streaming ) );
}

bool get_modem_status_reset_after_crash( void )
{
    return ( ( modem_status >> modem_status_crash ) & 0x01 );
}

bool get_modem_status_file_upload( void )
{
    return ( ( modem_status >> modem_status_upload ) & 0x01 );
}

bool get_modem_status_joining( void )
{
    return ( ( modem_status >> modem_status_joining ) & 0x01 );
}

bool get_modem_status_streaming( void )
{
    return ( ( modem_status >> modem_status_streaming ) & 0x01 );
}

void reset_modem_charge( void )
{
    radio_planner_t* rp = modem_get_radio_planner( );
    rp_stats_init( &rp->stats );
}

uint32_t get_modem_charge_ma_s( void )
{
    radio_planner_t* rp                = modem_get_radio_planner( );
    uint32_t         total_consumption = rp->stats.tx_total_consumption_ma + rp->stats.rx_total_consumption_ma;
    return ( ( total_consumption / 1000 ) + modem_charge_offset );
}

uint32_t get_modem_charge_ma_h( void )
{
    return get_modem_charge_ma_s( ) / 3600;
}

uint8_t get_modem_voltage( void )
{
    return ( uint8_t ) bsp_mcu_get_mcu_voltage( );
}
uint8_t get_modem_temp( void )
{
    return ( uint8_t ) bsp_mcu_get_mcu_temperature( );
}

e_dm_cmd_length_valid dm_check_dminfo_size( e_dm_info_t cmd, uint8_t length )
{
    if( cmd >= e_inf_max )
    {
        BSP_DBG_TRACE_ERROR( "Invalid DM command\n" );
        return DM_CMD_LENGTH_NOT_VALID;
    }

    if( length != dm_info_field_sz[cmd] )
    {
        BSP_DBG_TRACE_ERROR( "Invalid DM command size\n" );
        return DM_CMD_LENGTH_NOT_VALID;
    }

    return DM_CMD_LENGTH_VALID;
}

e_dm_error_t dm_set_conf( e_dm_info_t tag, uint8_t* data, uint8_t length )
{
    e_dm_error_t ret = DM_OK;

    if( dm_check_dminfo_size( tag, length ) != DM_CMD_LENGTH_VALID )
    {
        tag = e_inf_max;
        ret = DM_ERROR;
    }
    else
    {
        switch( tag )
        {
        case e_inf_adrmode:
            lorawan_api_dr_strategy_set( ( dr_strategy_t ) data[0] );
            break;
        case e_inf_joineui:
            lorawan_api_appeui_key_set( &data[0] );
            break;
        case e_inf_interval:
            set_modem_dm_interval( data[0] );
            modem_supervisor_add_task_dm_status( get_modem_dm_interval_second( ) );
            break;
        case e_inf_region:
            set_modem_region( data[0] );
            break;
        default:
            tag = e_inf_max;
            ret = DM_ERROR;
            break;
        }
    }
    if( ret == DM_OK )
    {
        increment_asynchronous_msgnumber( RSP_SETCONF, tag );
    }

    return ret;
}

e_modem_mute_t get_modem_muted( void )
{
    e_modem_mute_t mute;

    if( number_of_muted_day == MODEM_INFINITE_MUTE )
    {
        mute = MODEM_INFINITE_MUTE;
    }
    else if( number_of_muted_day > 0 )
    {
        mute = MODEM_TEMPORARY_MUTE;
    }
    else
    {
        mute = MODEM_NOT_MUTE;
    }
    return mute;
}

uint8_t dm_get_number_of_days_mute( void )
{
    return number_of_muted_day;
}

void dm_set_number_of_days_mute( uint8_t days )
{
    BSP_DBG_TRACE_PRINTF( "MUTE for %d days\n", days );
    if( number_of_muted_day != days )
    {
        number_of_muted_day = days;

        if( number_of_muted_day > 0 )
        {
            set_modem_status_modem_mute( true );
        }
        else
        {
            set_modem_status_modem_mute( false );
        }

        modem_store_context( );
    }
}

uint8_t get_dm_info_tag_list( uint8_t* dm, e_dm_info_rate_t flag )
{
    uint8_t* p = dm;
    uint32_t info_req;

    if( flag == DM_INFO_NOW )
    {
        info_req = dm_info_bitfield_now;
    }
    else
    {
        info_req = dm_info_bitfield_periodic;
    }

    for( uint8_t i = 0; i < e_inf_max; i++ )
    {
        if( ( info_req & ( 1 << i ) ) )
        {
            *p++ = i;  // If bit is set, added id Code in payload
        }
    }
    return p - dm;
}

e_set_error_t set_dm_info( uint8_t* requested_info_list, uint8_t len, e_dm_info_rate_t flag )
{
    e_set_error_t ret      = SET_OK;
    uint32_t      info_req = 0;

    ret = convert_requested_dm_info_bytes_to_bitfield( requested_info_list, len, &info_req );

    if( ret == SET_OK )
    {
        if( flag == DM_INFO_NOW )
        {
            dm_info_bitfield_now = info_req;
            tag_number_now       = 0;  // Reset tag_number used by dm_status_payload to
                                       // start a report from beginning
        }
        else
        {
            if( dm_info_bitfield_periodic != info_req )
            {
                dm_info_bitfield_periodic = info_req;
                tag_number                = 0;  // Reset tag_number used by dm_status_payload to start
                                                // a report from beginning
                modem_store_context( );
            }
        }
    }

    return ret;
}

bool dm_status_payload( uint8_t* dm_uplink_message, uint8_t* dm_uplink_message_len, uint8_t max_size,
                        e_dm_info_rate_t flag )
{
    uint8_t* p_tmp = dm_uplink_message;
    uint8_t* p     = dm_uplink_message;
    uint32_t info_requested;
    uint8_t* tag     = NULL;
    bool     pending = false;

    // Used DM code given in parameter
    if( flag == DM_INFO_NOW )
    {
        info_requested = dm_info_bitfield_now;
        tag            = &tag_number_now;
        // Used DM code in context
    }
    else
    {
        info_requested = dm_info_bitfield_periodic;
        tag            = &tag_number;
    }

    if( check_dm_status_max_size( info_requested, max_size ) != DM_CMD_LENGTH_VALID )
    {
        *dm_uplink_message_len = 0;
        BSP_DBG_TRACE_ERROR( "check_dm_status_max_size\n" );
        return false;
    }

    if( *tag >= e_inf_max )
    {
        *tag = 0;
    }
    // BSP_DBG_TRACE_PRINTF("info_requested = %d \n",info_requested);
    while( ( *tag ) < e_inf_max )
    {
        // BSP_DBG_TRACE_WARNING("tag %d - %d\n",*tag, (info_requested >> *tag) & 0x01
        // );
        if( ( info_requested & ( 1 << *tag ) ) )
        {
            *p_tmp++ = *tag;  // Add id Code in payload then the value(s)
            switch( *tag )
            {
            case e_inf_status:
                *p_tmp = get_modem_status( );
                break;
            case e_inf_charge: {
                uint32_t charge = get_modem_charge_ma_h( );
                *p_tmp          = charge & 0xFF;
                *( p_tmp + 1 )  = ( charge >> 8 ) & 0xFF;
                break;
            }
            case e_inf_voltage:
                *p_tmp = get_modem_voltage( );
                break;
            case e_inf_temp:
                *p_tmp = get_modem_temp( );
                break;
            case e_inf_signal:
                *p_tmp         = lorawan_api_last_rssi_get( ) + 64;  // strength of last downlink (RSSI [dBm]+64)
                *( p_tmp + 1 ) = lorawan_api_last_snr_get( ) >> 2;   // strength of last downlink (SNR [0.25 dB])
                break;
            case e_inf_uptime: {
                uint32_t time  = get_modem_uptime_s( ) / 3600;
                *p_tmp         = time & 0xFF;
                *( p_tmp + 1 ) = time >> 8;
            }
            break;
            case e_inf_rxtime: {
                s_modem_dwn_t dwnframe = { 0 };
                get_modem_downlink_frame( &dwnframe );
                uint32_t time  = ( bsp_rtc_get_time_s( ) - dwnframe.timestamp ) / 3600;
                *p_tmp         = time & 0xFF;
                *( p_tmp + 1 ) = time >> 8;
            }
            break;
            case e_inf_firmware:
                // return the crc value of fuota dedicated for test have to be
                // re implement when fuota availble
                memset( p_tmp, 0, dm_info_field_sz[*tag] );
                break;
            case e_inf_adrmode:
                *p_tmp = get_modem_adr( );
                break;
            case e_inf_joineui:
                lorawan_api_appeui_key_get( p_tmp );
                break;
            case e_inf_interval:
                *p_tmp = get_modem_dm_interval( );
                break;
            case e_inf_region:
                *p_tmp = get_modem_region( );
                break;
            case e_inf_rfu_0:
                // Nothing to do
                break;
            case e_inf_rstcount:
                *p_tmp         = lorawan_api_nb_reset_get( ) & 0xFF;
                *( p_tmp + 1 ) = lorawan_api_nb_reset_get( ) >> 8;
                break;
            case e_inf_deveui:
                lorawan_api_deveui_get( p_tmp );
                break;
            case e_inf_rfu_1:
                // Nothing to do
                break;
            case e_inf_session:
                *p_tmp         = lorawan_api_devnonce_get( ) & 0xFF;
                *( p_tmp + 1 ) = lorawan_api_devnonce_get( ) >> 8;
                break;
            case e_inf_chipeui:
                memset( p_tmp, 0, dm_info_field_sz[*tag] );
                break;
            case e_inf_streampar:
                *p_tmp         = modem_get_stream_port( );
                *( p_tmp + 1 ) = modem_get_stream_encryption( );
                break;
            case e_inf_appstatus:
                get_modem_appstatus( p_tmp );
                break;
            default:
                BSP_DBG_TRACE_ERROR( "Construct DM payload report, unknown code 0x%02x\n", *tag );
                break;
            }

            p_tmp += dm_info_field_sz[*tag];
            // Check if last message can be enqueued
            if( ( p_tmp - dm_uplink_message ) <= max_size )
            {
                p = p_tmp;
            }
            else
            {
                // last message can't be enqueued
                pending = true;
                break;  // break for loop
            }
        }
        ( *tag )++;
    }

    *dm_uplink_message_len = p - dm_uplink_message;
    return pending;
}

e_set_error_t set_modem_suspend( e_modem_suspend_t suspend )
{
    if( suspend > 1 )
    {
        return SET_ERROR;
    }

    if( suspend == MODEM_NOT_SUSPEND )
    {
        set_modem_status_radio_suspend( false );
        is_modem_suspend = MODEM_NOT_SUSPEND;
    }
    else
    {
        set_modem_status_radio_suspend( true );
        is_modem_suspend = MODEM_SUSPEND;
    }
    return SET_OK;
}

e_modem_suspend_t get_modem_suspend( void )
{
    return is_modem_suspend;
}

e_set_error_t set_modem_adr_profile( dr_strategy_t user_dr, uint8_t* adr_custom_data, uint8_t adr_custom_length )
{
    /* error case : 1) user_dr invalid
                    2) user_dr = custom but length not equal to 16
                    3) user_dr not custom but length not equal to 0*/
    if( ( user_dr >= UNKNOWN_DR ) || ( ( user_dr == USER_DR_DISTRIBUTION ) && ( adr_custom_length != 16 ) ) ||
        ( ( user_dr < USER_DR_DISTRIBUTION ) && ( adr_custom_length != 0 ) ) )
    {
        BSP_DBG_TRACE_ERROR( "user_dr = %d and length = %d \n ", user_dr, adr_custom_length );
        BSP_DBG_TRACE_ERROR( "CMD_SETADRPROFILE with not valid profile\n" );
        return SET_ERROR;
    }
    if( user_dr == USER_DR_DISTRIBUTION )
    {
        uint8_t  MinDrTemp       = lorawan_api_min_dr_get( );
        uint8_t  MaxDrTemp       = lorawan_api_max_dr_get( );
        uint32_t adrDistribution = 0;
        for( uint8_t i = 0; i < 16; i++ )
        {
            if( ( adr_custom_data[i] < MinDrTemp ) || ( adr_custom_data[i] > MaxDrTemp ) || adr_custom_data[i] > 7 )
            {
                BSP_DBG_TRACE_ERROR( "CMD_SETADRPROFILE with a bad DataRate value\n" );
                return SET_ERROR;
            }
            if( adr_custom_data[i] == 0x00 )
            {
                adrDistribution += ( ( ( adrDistribution & 0xF0000000 ) >> 28 ) != 0xF ) ? ( 1 << 28 ) : 0;
            }
            else if( adr_custom_data[i] == 0x01 )
            {
                adrDistribution += ( ( ( adrDistribution & 0x0F000000 ) >> 24 ) != 0xF ) ? ( 1 << 24 ) : 0;
            }
            else if( adr_custom_data[i] == 0x02 )
            {
                adrDistribution += ( ( ( adrDistribution & 0x00F00000 ) >> 20 ) != 0xF ) ? ( 1 << 20 ) : 0;
            }
            else if( adr_custom_data[i] == 0x03 )
            {
                adrDistribution += ( ( ( adrDistribution & 0x000F0000 ) >> 16 ) != 0xF ) ? ( 1 << 16 ) : 0;
            }
            else if( adr_custom_data[i] == 0x04 )
            {
                adrDistribution += ( ( ( adrDistribution & 0x0000F000 ) >> 12 ) != 0xF ) ? ( 1 << 12 ) : 0;
            }
            else if( adr_custom_data[i] == 0x05 )
            {
                adrDistribution += ( ( ( adrDistribution & 0x00000F00 ) >> 8 ) != 0xF ) ? ( 1 << 8 ) : 0;
            }
            else if( adr_custom_data[i] == 0x06 )
            {
                adrDistribution += ( ( ( adrDistribution & 0x000000F0 ) >> 4 ) != 0xF ) ? ( 1 << 4 ) : 0;
            }
            else if( adr_custom_data[i] == 0x07 )
            {
                adrDistribution += ( ( ( adrDistribution & 0x0000000F ) ) != 0xF ) ? 1 : 0;
            }
        }
        lorawan_api_dr_custom_set( adrDistribution );
    }
    lorawan_api_dr_strategy_set( user_dr );

    return SET_OK;
}

void set_modem_downlink_frame( void )
{
    lorawan_api_payload_receive( &( modem_dwn_pkt.port ), modem_dwn_pkt.data, &( modem_dwn_pkt.length ) );
    modem_dwn_pkt.timestamp = bsp_rtc_get_time_s( );
    modem_dwn_pkt.snr       = lorawan_api_last_snr_get( ) >> 2;
    modem_dwn_pkt.rssi      = lorawan_api_last_rssi_get( ) + 64;
    BSP_DBG_TRACE_ARRAY( "Downlink frame ", modem_dwn_pkt.data, modem_dwn_pkt.length );
    BSP_DBG_TRACE_PRINTF( "ModemDwnPort = %d , ", modem_dwn_pkt.port );
    BSP_DBG_TRACE_PRINTF( "ModemDwnSNR = %d , ModemDwnRssi = %d \n ", modem_dwn_pkt.snr, modem_dwn_pkt.rssi );
}
void get_modem_downlink_frame( s_modem_dwn_t* modem_dwn_in )
{
    modem_dwn_in->timestamp = modem_dwn_pkt.timestamp;
    modem_dwn_in->snr       = modem_dwn_pkt.snr;
    modem_dwn_in->rssi      = modem_dwn_pkt.rssi;
    modem_dwn_in->port      = modem_dwn_pkt.port;
    modem_dwn_in->length    = modem_dwn_pkt.length;
    memcpy( modem_dwn_in->data, modem_dwn_pkt.data, modem_dwn_pkt.length );
}

void set_dm_retrieve_pending_dl( uint8_t up_count, uint8_t up_delay )
{
    dm_pending_dl.up_count = up_count;
    dm_pending_dl.up_delay = up_delay;
}

void get_dm_retrieve_pending_dl( s_dm_retrieve_pending_dl_t* pending_dl )
{
    pending_dl->up_count = dm_pending_dl.up_count;
    pending_dl->up_delay = dm_pending_dl.up_delay;
}

void modem_store_context( void )
{
    uint8_t modem_context[BSP_MODEM_CONTEXT_SIZE] = { 0 };
    modem_context[0]                              = modem_dm_port;
    modem_context[1]                              = modem_dm_interval;
    modem_context[2]                              = modem_dm_upload_sctr;
    memcpy( &modem_context[3], ( uint8_t* ) ( &dm_info_bitfield_periodic ), sizeof( dm_info_bitfield_periodic ) );
    modem_context[7] = number_of_muted_day;

    uint32_t modem_charge = get_modem_charge_ma_s( );
    memcpy( &modem_context[8], ( uint8_t* ) &modem_charge, sizeof( modem_charge ) );

    uint32_t crctmp = crc( modem_context, BSP_MODEM_CONTEXT_SIZE - sizeof( crctmp ) );
    memcpy( &modem_context[BSP_MODEM_CONTEXT_SIZE - sizeof( crctmp )], ( uint8_t* ) &crctmp, sizeof( crctmp ) );
    BSP_DBG_TRACE_PRINTF(
        "Store a New Modem Config :\n Port = %d \n Interval = %d\n Upload_sctr = %d\n DM bitfield = 0x%lx\n Nb muted "
        "day = %u\n Charge = %u\n",
        modem_dm_port, modem_dm_interval, modem_dm_upload_sctr, dm_info_bitfield_periodic, number_of_muted_day,
        modem_charge );
    bsp_nvm_context_store( BSP_MODEM_CONTEXT_ADDR_OFFSET, modem_context, BSP_MODEM_CONTEXT_SIZE );

    modem_load_context( );
}

/*!
 * \brief    load modem context in non volatile memory
 * \remark
 * \retval void
 */
void modem_load_context( void )
{
    uint8_t modem_context[BSP_MODEM_CONTEXT_SIZE] = { 0 };

    bsp_nvm_context_restore( BSP_MODEM_CONTEXT_ADDR_OFFSET, modem_context, BSP_MODEM_CONTEXT_SIZE );

    uint32_t crctmp = crc( modem_context, BSP_MODEM_CONTEXT_SIZE - sizeof( crctmp ) );

    if( memcmp( &modem_context[BSP_MODEM_CONTEXT_SIZE - sizeof( crctmp )], ( uint8_t* ) &crctmp, sizeof( crctmp ) ) ==
        0 )
    {
        modem_dm_port        = modem_context[0];
        modem_dm_interval    = modem_context[1];
        modem_dm_upload_sctr = modem_context[2];
        memcpy( ( uint8_t* ) ( &dm_info_bitfield_periodic ), &modem_context[3], sizeof( dm_info_bitfield_periodic ) );
        number_of_muted_day = modem_context[7];

        if( is_modem_charge_loaded == false )
        {
            memcpy( ( uint8_t* ) ( &modem_charge_offset ), &modem_context[8], sizeof( modem_charge_offset ) );
            is_modem_charge_loaded = true;
        }

        BSP_DBG_TRACE_PRINTF(
            "Modem Load Config :\n Port = %d \n Interval = %d\n Upload_sctr = %d\n DM bitfield = 0x%lx\n Nb muted "
            "day = %u\n Charge = %u\n",
            modem_dm_port, modem_dm_interval, modem_dm_upload_sctr, dm_info_bitfield_periodic, number_of_muted_day,
            modem_charge_offset );
    }
    else
    {
        BSP_DBG_TRACE_ERROR( "Restore Modem context fail => Factory Reset Modem\n" );
        BSP_DBG_TRACE_ARRAY( " context = ", modem_context, BSP_MODEM_CONTEXT_SIZE )
        BSP_DBG_TRACE_PRINTF( " crctemp = %lx", crctmp );

        modem_context_factory_reset( );
    }
}

void modem_context_factory_reset( void )
{
    uint8_t modem_context[BSP_MODEM_CONTEXT_SIZE];
    modem_context[0] = DEFAULT_DM_PORT;
    modem_context[1] = DEFAULT_DM_REPORTING_INTERVAL;
    // upload_sctr is not reinit in the factory reset to copy sub-ghz modem behaviour
    dm_info_bitfield_periodic = DEFAULT_DM_REPORTING_FIELDS;
    memcpy( &modem_context[3], ( uint8_t* ) &dm_info_bitfield_periodic, sizeof( dm_info_bitfield_periodic ) );
    modem_context[7] = DEFAULT_DM_MUTE_DAY;

    uint32_t modem_charge = 0;
    memcpy( &modem_context[8], ( uint8_t* ) &modem_charge, sizeof( modem_charge ) );
    BSP_DBG_TRACE_ARRAY( "factory reset buf1", modem_context, BSP_MODEM_CONTEXT_SIZE );

    uint32_t crctmp = crc( modem_context, BSP_MODEM_CONTEXT_SIZE - sizeof( crctmp ) );
    memcpy( &modem_context[BSP_MODEM_CONTEXT_SIZE - sizeof( crctmp )], ( uint8_t* ) &crctmp, sizeof( crctmp ) );

    BSP_DBG_TRACE_ARRAY( "factory reset buf2", modem_context, BSP_MODEM_CONTEXT_SIZE );

    bsp_nvm_context_store( BSP_MODEM_CONTEXT_ADDR_OFFSET, modem_context, BSP_MODEM_CONTEXT_SIZE );

    is_modem_reset_requested = true;
    BSP_DBG_TRACE_INFO( "modem_context_factory_reset done\n" );
}

void modem_set_dm_upload_sctr( uint8_t session_counter )
{
    modem_dm_upload_sctr = session_counter & 0xf;
    modem_store_context( );
}

uint8_t modem_get_dm_upload_sctr( void )
{
    return ( modem_dm_upload_sctr & 0xf );
}

e_modem_upload_state_t modem_get_upload_state( void )
{
    return ( modem_upload_state );
}

void modem_set_upload_state( e_modem_upload_state_t upload_state )
{
    modem_upload_state = upload_state;
}

e_modem_stream_state_t modem_get_stream_state( void )
{
    return ( modem_stream_state.state );
}

uint8_t modem_get_stream_port( void )
{
    return ( modem_stream_state.port );
}

bool modem_get_stream_encryption( void )
{
    return ( modem_stream_state.encryption );
}

void modem_set_stream_state( e_modem_stream_state_t stream_state )
{
    modem_stream_state.state = stream_state;
}

void modem_set_stream_port( uint8_t port )
{
    modem_stream_state.port = port;
}

void modem_set_stream_encryption( bool enc )
{
    modem_stream_state.encryption = enc;
}

void modem_set_dm_info_bitfield_periodic( uint32_t value )
{
    if( dm_info_bitfield_periodic != value )
    {
        dm_info_bitfield_periodic = value;
        modem_store_context( );
    }
}
uint32_t modem_get_dm_info_bitfield_periodic( void )
{
    return ( dm_info_bitfield_periodic );
}
uint32_t modem_get_user_alarm( void )
{
    return ( user_alarm );
}

void modem_set_user_alarm( uint32_t alarm )
{
    user_alarm = alarm;
}

bool get_modem_reset_requested( void )
{
    return is_modem_reset_requested;
}
void set_modem_reset_requested( bool reset_req )
{
    is_modem_reset_requested = reset_req;
}

/* --- EOF ------------------------------------------------------------------ */
