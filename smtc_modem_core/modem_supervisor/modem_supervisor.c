/*!
 * \file      modem_supervisor.c
 *
 * \brief     soft modem task scheduler
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
 *-----------------------------------------------------------------------------------
 * --- DEPENDENCIES -----------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type
#include "modem_supervisor.h"
#include "smtc_bsp.h"
#include "modem_context.h"
#include "lorawan_api.h"
#include "dm_downlink.h"
#include "modem_api.h"

/*
 *-----------------------------------------------------------------------------------
 * --- PUBLIC MACROS ----------------------------------------------------------------
 */

/*!
 * \brief Returns the minimum value between a and b
 *
 * \param [IN] a 1st value
 * \param [IN] b 2nd value
 * \retval minValue Minimum value
 */
#define MIN( a, b ) ( ( ( a ) < ( b ) ) ? ( a ) : ( b ) )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static uint8_t               UploadPayload[255];
static lr1mac_states_t       LpState = LWPSTATE_IDLE;
static stask_manager         task_manager;
static user_rx_packet_type_t AvailableRxPacket                = NO_LORA_RXPACKET_AVAILABLE;
static bool                  is_pending_dm_status_payload_now = false;
static bool                  is_first_dm_after_join           = true;
static bool                  send_task_update_needed          = false;
static void ( *app_callback )( void )                         = NULL;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void modem_supervisor_init( void ( *callback )( void ), radio_planner_t* rp )
{
    lorawan_api_init( rp );
    lorawan_api_dr_strategy_set( USER_DR_DISTRIBUTION );
#if !defined( PERF_TEST_ENABLED )
    // do not clear join status that was set to joined to allow perf testbench to trigger some modem send tx commands
    lorawan_api_join_status_clear( );
#endif
    modem_event_init( );
    increment_asynchronous_msgnumber( RSP_RESET, 0 );
    init_task( );
    modem_load_context( );
    set_modem_start_time_s( bsp_rtc_get_time_s( ) );
    app_callback = callback;
}

void init_task( void )
{
    for( int i = 0; i < NUMBER_OF_TASKS; i++ )
    {
        task_manager.modem_task[i].priority = TASK_FINISH;
        task_manager.modem_task[i].id       = ( task_id_t ) i;
    }
    task_manager.next_task_id = IDLE_TASK;
}

eTask_valid_t modem_supervisor_remove_task( task_id_t id )
{
    if( id < NUMBER_OF_TASKS )
    {
        task_manager.modem_task[id].priority = TASK_FINISH;
        return TASK_VALID;
    }
    BSP_DBG_TRACE_ERROR( "modem_supervisor_remove_task id = %d unknown\n", id );
    return TASK_NOT_VALID;
}

eTask_valid_t modem_supervisor_add_task( smodem_task* task )
{
    // the modem supervisor always accept a new task.
    // in case of a previous task is already enqueue , the new task remove the old one.
    // as soon as a task has been elected by the modem supervisor , the task is managed by the stack itself and a new
    // task could be added inside the modem supervisor.
    if( task->id < NUMBER_OF_TASKS )
    {
        task_manager.modem_task[task->id].time_to_execute_s = task->time_to_execute_s;
        task_manager.modem_task[task->id].priority          = task->priority;
        task_manager.modem_task[task->id].fPort             = task->fPort;
        task_manager.modem_task[task->id].dataIn            = task->dataIn;
        task_manager.modem_task[task->id].sizeIn            = task->sizeIn;
        task_manager.modem_task[task->id].PacketType        = task->PacketType;

        return TASK_VALID;
    }
    BSP_DBG_TRACE_ERROR( "modem_supervisor_add_task id = %d unknown\n", task->id );
    return TASK_NOT_VALID;
}

void modem_supervisor_launch_task( task_id_t id )
{
    lr1mac_states_t send_status;
    switch( id )
    {
    case JOIN_TASK:
        BSP_DBG_TRACE_INFO( "JOIN_TASK\n" );
        if( get_join_state( ) == MODEM_JOINED )
        {
            BSP_DBG_TRACE_ERROR( "DEVICE ALREADY JOINED or TRY TO JOIN \n" );
            task_manager.next_task_id = IDLE_TASK;
        }
        else if( get_join_state( ) == MODEM_JOIN_ONGOING )
        {
            lorawan_api_join( bsp_rtc_get_time_ms( ) + MODEM_TASK_DELAY_MS );
        }
        break;
    case SEND_TASK: {
        send_status = lorawan_api_payload_send(
            task_manager.modem_task[id].fPort, task_manager.modem_task[id].dataIn, task_manager.modem_task[id].sizeIn,
            ( task_manager.modem_task[id].PacketType == TX_CONFIRMED ) ? CONF_DATA_UP : UNCONF_DATA_UP,
            bsp_rtc_get_time_ms( ) + MODEM_TASK_DELAY_MS );

        if( send_status == LWPSTATE_SEND )
        {
            send_task_update_needed = true;
            BSP_DBG_TRACE_PRINTF( " User Tx LORa on Port %d \n", task_manager.modem_task[id].fPort );
        }
        else
        {
            send_task_update_needed = false;
            BSP_DBG_TRACE_WARNING( "The payload can't be send! internal code: %x\n", send_status );
        }
        break;
    }
    case SEND_AT_TIME_TASK:
        break;

    case DM_TASK:
        if( get_join_state( ) == MODEM_JOINED )
        {
            uint8_t  max_payload = lorawan_api_next_max_payload_length_get( );
            uint8_t  payload[255];
            uint8_t  payload_length;
            uint32_t info_bitfield_periodic;
            uint32_t info_bitfield_periodictmp;

            if( is_first_dm_after_join == true )
            {
                info_bitfield_periodictmp = modem_get_dm_info_bitfield_periodic( );
                BSP_DBG_TRACE_PRINTF( " info bit field = %lx\n", info_bitfield_periodictmp );
                info_bitfield_periodic =
                    info_bitfield_periodictmp +
                    ( ( ( info_bitfield_periodictmp & ( 1 << e_inf_rstcount ) ) == 0 ) ? ( 1 << e_inf_rstcount ) : 0 ) +
                    ( ( ( info_bitfield_periodictmp & ( 1 << e_inf_session ) ) == 0 ) ? ( 1 << e_inf_session ) : 0 ) +
                    ( ( ( info_bitfield_periodictmp & ( 1 << e_inf_firmware ) ) == 0 ) ? ( 1 << e_inf_firmware ) : 0 );

                modem_set_dm_info_bitfield_periodic( info_bitfield_periodic );
                BSP_DBG_TRACE_PRINTF( " info bit fiel = %lx\n", info_bitfield_periodic );
                dm_status_payload( payload, &payload_length, max_payload, DM_INFO_PERIODIC );

                send_status = lorawan_api_payload_send( get_modem_dm_port( ), payload, payload_length, UNCONF_DATA_UP,
                                                        bsp_rtc_get_time_ms( ) + MODEM_TASK_DELAY_MS );

                if( send_status == LWPSTATE_SEND )
                {
                    is_first_dm_after_join = false;
                    BSP_DBG_TRACE_ARRAY( "payload DM ", payload, payload_length );
                    BSP_DBG_TRACE_PRINTF( " on Port %d\n", get_modem_dm_port( ) );
                    modem_set_dm_info_bitfield_periodic( info_bitfield_periodictmp );
                }
                else
                {
                    BSP_DBG_TRACE_WARNING( "Periodic DM can't be send! internal code: %x\n", send_status );
                }
            }
            else
            {
                if( ( get_modem_dm_interval_second( ) > 0 ) && ( modem_get_dm_info_bitfield_periodic( ) > 0 ) )
                {
                    dm_status_payload( payload, &payload_length, max_payload, DM_INFO_PERIODIC );
                    send_status =
                        lorawan_api_payload_send( get_modem_dm_port( ), payload, payload_length, UNCONF_DATA_UP,
                                                  bsp_rtc_get_time_ms( ) + MODEM_TASK_DELAY_MS );

                    if( send_status == LWPSTATE_SEND )
                    {
                        BSP_DBG_TRACE_ARRAY( "DM ", payload, payload_length );
                        BSP_DBG_TRACE_PRINTF( " on Port %d\n", get_modem_dm_port( ) );
                    }
                    else
                    {
                        BSP_DBG_TRACE_WARNING( "Periodic DM can't be send! internal code: %x\n", send_status );
                    }
                }
            }
        }
        break;
    case DM_TASK_NOW:
        if( get_join_state( ) == MODEM_JOINED )
        {
            uint8_t max_payload = lorawan_api_next_max_payload_length_get( );
            uint8_t payload[255];
            uint8_t payload_length;
            is_pending_dm_status_payload_now = dm_status_payload( payload, &payload_length, max_payload, DM_INFO_NOW );

            send_status = lorawan_api_payload_send( get_modem_dm_port( ), payload, payload_length, UNCONF_DATA_UP,
                                                    bsp_rtc_get_time_ms( ) + MODEM_TASK_DELAY_MS );
            if( send_status == LWPSTATE_SEND )
            {
                BSP_DBG_TRACE_ARRAY( "DM Req ", payload, payload_length );
                BSP_DBG_TRACE_PRINTF( " on Port %d\n", get_modem_dm_port( ) );
            }
            else
            {
                BSP_DBG_TRACE_WARNING( "Requested DM can't be send! internal code: %x\n", send_status );
            }
        }
        break;
    case FILE_UPLOAD_TASK: {
        int32_t size_file_upload = 1;
        set_modem_status_file_upload( false );
        if( get_join_state( ) != MODEM_JOINED )
        {
            BSP_DBG_TRACE_ERROR( "DEVICE NOT JOIN \n" );
            break;
        }
        else if( modem_get_upload_state( ) != MODEM_UPLOAD_START )
        {
            BSP_DBG_TRACE_ERROR( "FileUpload not init \n" );
            break;
        }
        size_file_upload = file_upload_gen_uplink( UploadPayload, lorawan_api_next_max_payload_length_get( ), MODE_0,
                                                   lorawan_api_fcnt_up_get( ) );
        if( size_file_upload > 0 )
        {
            set_modem_status_file_upload( true );
            lorawan_api_payload_send( get_modem_dm_port( ), UploadPayload, size_file_upload, UNCONF_DATA_UP,
                                      bsp_rtc_get_time_ms( ) + MODEM_TASK_DELAY_MS );
        }
        else
        {
            BSP_DBG_TRACE_WARNING( "File upload DONE\n" );
            increment_asynchronous_msgnumber( RSP_FILEDONE, 0x00 );
            modem_set_upload_state( MODEM_UPLOAD_NOT_INIT );
        }
        break;
    }
    case MUTE_TASK: {
        if( get_modem_muted( ) == MODEM_TEMPORARY_MUTE )
        {
            dm_set_number_of_days_mute( dm_get_number_of_days_mute( ) - 1 );
        }
        break;
    }
    case RETRIEVE_DL_TASK: {
        lorawan_api_payload_send( get_modem_dm_port( ), task_manager.modem_task[id].dataIn,
                                  task_manager.modem_task[id].sizeIn, task_manager.modem_task[id].PacketType,
                                  task_manager.modem_task[id].time_to_execute_s * 1000 );
        break;
    }
    default:
        break;
    }
    modem_supervisor_remove_task( id );
}

void modem_supervisor_update_task( task_id_t id )
{
    s_modem_dwn_t dwnframe;
    if( lorawan_api_state_get( ) == LWPSTATE_ERROR )
    {
        BSP_DBG_TRACE_ERROR( "LP state error Occur \n" );
        bsp_mcu_reset( );
    }
    if( ( AvailableRxPacket != NO_LORA_RXPACKET_AVAILABLE ) && ( get_join_state( ) == MODEM_JOINED ) )
    {
        set_modem_downlink_frame( );
        get_modem_downlink_frame( &dwnframe );
        if( dwnframe.port == get_modem_dm_port( ) )
        {
            dm_downlink( dwnframe.data, dwnframe.length );
        }
        else
        {
            increment_asynchronous_msgnumber( RSP_DOWNDATA, 0 );
        }
    }
    switch( id )
    {
    case JOIN_TASK:
        if( ( get_join_state( ) == MODEM_JOIN_ONGOING ) && ( lorawan_api_isjoined( ) == JOINED ) )
        {
            set_modem_status_joining( false );
            set_modem_downlink_frame( );
            increment_asynchronous_msgnumber( RSP_JOINED, 0 );
            set_modem_status_modem_joined( true );
            // as soon as modem is joined, modem send has to sent a dm report every DM_PERIOD_AFTER_JOIN
            is_first_dm_after_join = true;
            modem_supervisor_add_task_dm_status( DM_PERIOD_AFTER_JOIN );
        }
        else if( get_join_state( ) == MODEM_JOIN_ONGOING )
        {
            set_modem_status_joining( false );
            modem_supervisor_add_task_join( );  // relaunch join only if no leave cmd
        }
        else if( ( lorawan_api_isjoined( ) == JOINED ) )  //
        {
            set_modem_status_joining( false );
            lorawan_api_join_status_clear( );  // => manage case leave cmd between end of join and join suceed
        }
        break;
    case DM_TASK:
        if( get_modem_dm_interval_second( ) > 0 )
        {
            modem_supervisor_add_task_dm_status( get_modem_dm_interval_second( ) );
        }
        break;
    case DM_TASK_NOW:
        if( is_pending_dm_status_payload_now )
        {
            modem_supervisor_add_task_dm_status_now( );
        }
        break;
    case SEND_TASK:
        if( send_task_update_needed == true )
        {
            if( lorawan_api_rx_ack_bit_get( ) == 1 )
            {
                increment_asynchronous_msgnumber( RSP_TXDONE, MODEM_TX_SUCCESS_WITH_ACK );
            }
            else
            {
                increment_asynchronous_msgnumber( RSP_TXDONE, MODEM_TX_SUCCESS );
            }
        }
        else
        {
            increment_asynchronous_msgnumber( RSP_TXDONE, MODEM_TX_FAILED );
        }
        // Re-enable the duty cycle in case of Emergency Tx was sent
        lorawan_api_duty_cycle_enable_set( true );
        break;
    case FILE_UPLOAD_TASK: {
        if( get_modem_status_file_upload( ) == true )
        {
            smodem_task upload_task;  // as soon as you join you send dm report every DM_PERIOD_AFTER_JOIN
            upload_task.id       = FILE_UPLOAD_TASK;
            upload_task.priority = TASK_HIGH_PRIORITY;
            upload_task.time_to_execute_s =
                bsp_rtc_get_time_s( ) + modem_upload_avgdelay_get( ) + bsp_rng_get_random_in_range( 1, 3 );
            modem_supervisor_add_task( &upload_task );
        }
        break;
    }
    case MUTE_TASK: {
        if( get_modem_muted( ) == MODEM_TEMPORARY_MUTE )
        {
            modem_supervisor_add_task_modem_mute( );
        }
        break;
    }
    case RETRIEVE_DL_TASK: {
        s_dm_retrieve_pending_dl_t retrieve;
        get_dm_retrieve_pending_dl( &retrieve );
        retrieve.up_count--;

        set_dm_retrieve_pending_dl( retrieve.up_count, retrieve.up_delay );
        if( retrieve.up_count > 0 )
        {
            modem_supervisor_add_task_retrieve_dl( retrieve.up_delay );
        }

        break;
    }
    default:
        break;
    }
    AvailableRxPacket = NO_LORA_RXPACKET_AVAILABLE;
}

uint32_t modem_supervisor_scheduler( void )
{
    // Check first if stack state is idle
    if( lorawan_api_state_get( ) != LWPSTATE_IDLE )
    {
        return ( CALL_LR1MAC_PERIOD_MS );
    }

    if( task_manager.next_task_id != IDLE_TASK )
    {
        modem_supervisor_update_task( task_manager.next_task_id );
        task_manager.next_task_id = IDLE_TASK;
    }

    eTask_priority next_task_priority = TASK_FINISH;
    int32_t        next_task_time     = MODEM_MAX_TIME;

    // Find the highest priority task in the past
    for( task_id_t i = 0; i < NUMBER_OF_TASKS; i++ )
    {
        if( task_manager.modem_task[i].priority != TASK_FINISH )
        {
            int32_t next_task_time_tmp =
                ( int32_t )( task_manager.modem_task[i].time_to_execute_s - bsp_rtc_get_time_s( ) );

            if( ( next_task_time_tmp <= 0 ) && ( task_manager.modem_task[i].priority < next_task_priority ) )
            {
                next_task_priority        = task_manager.modem_task[i].priority;
                next_task_time            = next_task_time_tmp;
                task_manager.next_task_id = ( task_id_t ) i;
            }
        }
    }

    // No task in the past was found, select the least in the future for wake up
    if( next_task_priority == TASK_FINISH )
    {
        for( task_id_t i = 0; i < NUMBER_OF_TASKS; i++ )
        {
            if( task_manager.modem_task[i].priority != TASK_FINISH )
            {
                int32_t next_task_time_tmp =
                    ( int32_t )( task_manager.modem_task[i].time_to_execute_s - bsp_rtc_get_time_s( ) );
                if( next_task_time_tmp < next_task_time )
                {
                    next_task_time            = next_task_time_tmp;
                    task_manager.next_task_id = ( task_id_t ) i;
                }
            }
        }
    }

    int32_t next_free_dtc = lorawan_api_next_free_duty_cycle_ms_get( );
    if( next_free_dtc > 0 )
    {
        next_free_dtc = ( next_free_dtc / 1000 ) + 1;
        BSP_DBG_TRACE_WARNING( "Duty Cycle, remaining time: %ds\n", next_free_dtc );
    }

    next_task_time = ( next_task_time > next_free_dtc ) ? next_task_time : next_free_dtc;

    if( next_task_time > 0 )
    {
        task_manager.sleep_duration = next_task_time;
        task_manager.next_task_id   = IDLE_TASK;
        return ( next_task_time );
    }
    else
    {
        modem_supervisor_launch_task( task_manager.next_task_id );
        return 0;
    }
}

uint32_t modem_supervisor_engine( void )
{
    // manage reset requested by the host
    if( get_modem_reset_requested( ) == true )
    {
        bsp_disable_irq( );
        // workaround to avoid issue when reset too fast after a store context
        bsp_mcu_wait_us( 2000000 );

        bsp_mcu_reset( );
    }

    uint32_t alarm                 = modem_get_user_alarm( );
    int32_t  user_alarm_in_seconds = MODEM_MAX_ALARM_S;
    // manage the user alarm
    if( alarm != 0 )
    {
        user_alarm_in_seconds = ( int32_t )( alarm - bsp_rtc_get_time_s( ) );

        if( user_alarm_in_seconds <= 0 )
        {
            increment_asynchronous_msgnumber( RSP_ALARM, 0 );
            modem_set_user_alarm( 0 );
            user_alarm_in_seconds = MODEM_MAX_ALARM_S;
        }
    }
    uint32_t sleep_time;
    bsp_watchdog_reload( );

    // case Lorawan stack already in use
    LpState = lorawan_api_state_get( );
    if( ( LpState != LWPSTATE_IDLE ) && ( LpState != LWPSTATE_ERROR ) && ( LpState != LWPSTATE_INVALID ) )
    {
        LpState = lorawan_api_process( &AvailableRxPacket );
        return ( CALL_LR1MAC_PERIOD_MS );
    }

    // Call modem_supervisor_update_task to update asynchronous messages number
    if( task_manager.next_task_id != IDLE_TASK )
    {
        modem_supervisor_update_task( task_manager.next_task_id );
        task_manager.next_task_id = IDLE_TASK;
    }

    uint8_t msgnumber_tmp;
    do
    {
        if( ( get_asynchronous_msgnumber( ) > 0 ) && ( *app_callback != NULL ) )
        {
            app_callback( );
        }
        msgnumber_tmp = get_asynchronous_msgnumber( );
    } while( msgnumber_tmp < get_asynchronous_msgnumber( ) );

    // Don't call the supervisor if modem is suspend
    if( ( get_modem_suspend( ) == MODEM_NOT_SUSPEND ) && ( get_modem_muted( ) != MODEM_INFINITE_MUTE ) )
    {
        // you reach this step only if lorawan stack is in idle mode
        sleep_time = modem_supervisor_scheduler( );
    }
    else
    {
        // Set Max time (7FFFFFFF>>10 ie /1024 to be compatible with *1000)
        sleep_time = 0x1FFFFF;
    }

    alarm = modem_get_user_alarm( );
    if( alarm != 0 )
    {
        user_alarm_in_seconds = ( int32_t )( alarm - bsp_rtc_get_time_s( ) );
        if( user_alarm_in_seconds <= 0 )
        {
            user_alarm_in_seconds = 0;
        }
    }

    sleep_time = MIN( sleep_time, ( uint32_t ) user_alarm_in_seconds );
    BSP_DBG_TRACE_INFO( "Next task in %lu\n", sleep_time );
    return ( 1000 * sleep_time );
}
