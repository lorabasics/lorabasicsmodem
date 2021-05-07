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

#include "smtc_modem_hal.h"
#include "modem_config.h"
#include "modem_context.h"

#include "lorawan_api.h"
#include "dm_downlink.h"
#include "smtc_modem_api.h"
#include "fragmented_data_block.h"
#include "radio_planner.h"
// services
#include "file_upload.h"
#include "stream.h"
#include "alc_sync.h"

#if defined( LR1110_MODEM )
#include "pool_mem.h"
#include "host_irq.h"
#endif  // LR1110_MODEM
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

#if !defined( LR1110_MODEM )
static lr1mac_states_t       LpState = LWPSTATE_IDLE;
static stask_manager         task_manager;
static user_rx_packet_type_t AvailableRxPacket                     = NO_LORA_RXPACKET_AVAILABLE;
static bool                  is_pending_dm_status_payload_periodic = false;
static bool                  is_pending_dm_status_payload_now      = false;
static bool                  is_first_dm_after_join                = true;
static bool                  send_task_update_needed               = false;
static uint32_t              alc_sync_nb_time_req                  = 0;
static void ( *app_callback )( void )                              = NULL;
static alc_sync_ctx_t* alc_sync_context                            = NULL;
static rose_t*         ROSE                                        = NULL;
static file_upload_t*  file_upload_context                         = NULL;
#else

struct
{
    lr1mac_states_t       LpState;
    stask_manager         task_manager;
    user_rx_packet_type_t AvailableRxPacket;
    bool                  is_pending_dm_status_payload_periodic;
    bool                  is_pending_dm_status_payload_now;
    bool                  is_first_dm_after_join;
    bool                  send_task_update_needed;
    uint32_t              alc_sync_nb_time_req;
    void ( *app_callback )( void );
    alc_sync_ctx_t* alc_sync_context;
    rose_t*         ROSE;
    file_upload_t*  file_upload_context;
} modem_supervisor_context;

// clang-format off
#define LpState                                 modem_supervisor_context.LpState
#define task_manager                            modem_supervisor_context.task_manager
#define AvailableRxPacket                       modem_supervisor_context.AvailableRxPacket
#define is_pending_dm_status_payload_periodic   modem_supervisor_context.is_pending_dm_status_payload_periodic
#define is_pending_dm_status_payload_now        modem_supervisor_context.is_pending_dm_status_payload_now
#define is_first_dm_after_join                  modem_supervisor_context.is_first_dm_after_join
#define send_task_update_needed                 modem_supervisor_context.send_task_update_needed
#define alc_sync_nb_time_req                    modem_supervisor_context.alc_sync_nb_time_req
#define app_callback                            modem_supervisor_context.app_callback
#define alc_sync_context                        modem_supervisor_context.alc_sync_context
#define ROSE                                    modem_supervisor_context.ROSE
#define file_upload_context                     modem_supervisor_context.file_upload_context

// clang-format on
#endif

static void backoff_mobile_static( void );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void modem_supervisor_init( void ( *callback )( void ), radio_planner_t* rp,
                            smtc_modem_services_t* smtc_modem_services_ctx )
{
    LpState                               = LWPSTATE_IDLE;
    AvailableRxPacket                     = NO_LORA_RXPACKET_AVAILABLE;
    is_pending_dm_status_payload_periodic = true;  // Set to true to send at least the first DM after Join
    is_pending_dm_status_payload_now      = false;
    is_first_dm_after_join                = true;
    send_task_update_needed               = false;
    alc_sync_nb_time_req                  = 0;
    app_callback                          = callback;
    alc_sync_context                      = &( smtc_modem_services_ctx->alc_sync_ctx );
    ROSE                                  = &( smtc_modem_services_ctx->stream_ROSE_ctx );
    file_upload_context                   = &( smtc_modem_services_ctx->file_upload_ctx );

    lorawan_api_init( rp );

    lorawan_api_dr_strategy_set( STATIC_ADR_MODE );
#if !defined( PERF_TEST_ENABLED )
    // do not clear join status that was set to joined to allow perf testbench to trigger some modem send tx commands
    lorawan_api_join_status_clear( );
#endif
    modem_context_init( );
    modem_event_init( );
    modem_supervisor_init_task( );
    modem_load_context( );
    alc_sync_init( alc_sync_context );
#if defined( LR1110_MODEM )
    frag_init( );
#endif  // LR1110_MODEM
#ifdef MODEM_CLOCK_SYNC_DEFAULT_PORT
#if( MODEM_CLOCK_SYNC_DEFAULT_PORT == false )
    alc_sync_set_port( alc_sync_context, get_modem_dm_port( ) );
#endif
#endif

    set_modem_start_time_s( smtc_modem_hal_get_time_in_s( ) );

    // save used rp in context in case of future needs (suspend/resume)
    modem_context_set_modem_rp( rp );

    // if a crash happened before set crash bit in status
    if( smtc_modem_hal_get_crashlog_status( ) == true )
    {
        set_modem_status_reset_after_crash( true );
    }
    else
    {
        set_modem_status_reset_after_crash( false );
    }

    // Event EVENT_RESET must be done at the end of init !!
    increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_RESET, 0 );
}

void modem_supervisor_init_task( void )
{
    for( int i = 0; i < NUMBER_OF_TASKS; i++ )
    {
        task_manager.modem_task[i].priority = TASK_FINISH;
        task_manager.modem_task[i].id       = ( task_id_t ) i;
    }
    task_manager.next_task_id = IDLE_TASK;
}

eTask_priority modem_supervisor_get_task_priority( task_id_t id )
{
    if( id < NUMBER_OF_TASKS )
    {
        return ( task_manager.modem_task[id].priority );
    }
    else
    {
        return ( TASK_FINISH );
    }
}

eTask_valid_t modem_supervisor_remove_task( task_id_t id )
{
    if( id < NUMBER_OF_TASKS )
    {
        task_manager.modem_task[id].priority = TASK_FINISH;
        return TASK_VALID;
    }
    SMTC_MODEM_HAL_TRACE_ERROR( "modem_supervisor_remove_task id = %d unknown\n", id );
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
    SMTC_MODEM_HAL_TRACE_ERROR( "modem_supervisor_add_task id = %d unknown\n", task->id );
    return TASK_NOT_VALID;
}

void modem_supervisor_launch_task( task_id_t id )
{
    lr1mac_states_t send_status = LWPSTATE_IDLE;
    switch( id )
    {
    case JOIN_TASK:
        // SMTC_MODEM_HAL_TRACE_INFO( "JOIN_TASK\n" );
        if( get_join_state( ) == MODEM_JOINED )
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "DEVICE ALREADY JOINED or TRY TO JOIN \n" );
            task_manager.next_task_id = IDLE_TASK;
        }
        else if( get_join_state( ) == MODEM_JOIN_ONGOING )
        {
            lorawan_api_join( smtc_modem_hal_get_time_in_ms( ) + MODEM_TASK_DELAY_MS );
        }
        break;
    case SEND_TASK: {
        send_status = lorawan_api_payload_send(
            task_manager.modem_task[id].fPort, task_manager.modem_task[id].dataIn, task_manager.modem_task[id].sizeIn,
            ( task_manager.modem_task[id].PacketType == TX_CONFIRMED ) ? CONF_DATA_UP : UNCONF_DATA_UP,
            smtc_modem_hal_get_time_in_ms( ) + MODEM_TASK_DELAY_MS );

        if( send_status == LWPSTATE_SEND )
        {
            send_task_update_needed = true;
            SMTC_MODEM_HAL_TRACE_PRINTF( " User Tx LORa on Port %d \n", task_manager.modem_task[id].fPort );
        }
        else
        {
            send_task_update_needed = false;
            SMTC_MODEM_HAL_TRACE_WARNING( "The payload can't be send! internal code: %x\n", send_status );
        }
        break;
    }
    case SEND_AT_TIME_TASK:
        break;

    case DM_TASK:
        if( get_join_state( ) == MODEM_JOINED )
        {
            if( lorawan_api_modem_certification_is_enabled( ) == true )
            {
                break;
            }
            uint8_t  max_payload = lorawan_api_next_max_payload_length_get( );
            uint8_t  payload[242];
            uint8_t  payload_length;
            uint32_t info_bitfield_periodic;
            uint32_t info_bitfield_periodictmp;

            if( ( is_first_dm_after_join == true ) && ( is_pending_dm_status_payload_periodic == true ) )
            {
                info_bitfield_periodictmp = modem_get_dm_info_bitfield_periodic( );
                SMTC_MODEM_HAL_TRACE_PRINTF( " info bit field = %x\n", info_bitfield_periodictmp );
                info_bitfield_periodic =
                    info_bitfield_periodictmp +
                    ( ( ( info_bitfield_periodictmp & ( 1 << e_inf_rstcount ) ) == 0 ) ? ( 1 << e_inf_rstcount ) : 0 ) +
                    ( ( ( info_bitfield_periodictmp & ( 1 << e_inf_session ) ) == 0 ) ? ( 1 << e_inf_session ) : 0 ) +
                    ( ( ( info_bitfield_periodictmp & ( 1 << e_inf_firmware ) ) == 0 ) ? ( 1 << e_inf_firmware ) : 0 );

                modem_set_dm_info_bitfield_periodic( info_bitfield_periodic );
                SMTC_MODEM_HAL_TRACE_PRINTF( " info bit field = %x\n", info_bitfield_periodic );
                is_pending_dm_status_payload_periodic =
                    dm_status_payload( payload, &payload_length, max_payload, DM_INFO_PERIODIC );

                send_status = lorawan_api_payload_send( get_modem_dm_port( ), payload, payload_length, UNCONF_DATA_UP,
                                                        smtc_modem_hal_get_time_in_ms( ) + MODEM_TASK_DELAY_MS );

                if( send_status == LWPSTATE_SEND )
                {
                    if( is_pending_dm_status_payload_periodic == false )
                    {
                        is_first_dm_after_join = false;
                    }
                    modem_set_dm_info_bitfield_periodic( info_bitfield_periodictmp );
                    SMTC_MODEM_HAL_TRACE_ARRAY( "payload DM ", payload, payload_length );
                    SMTC_MODEM_HAL_TRACE_PRINTF( " on Port %d\n", get_modem_dm_port( ) );
                }
                else
                {
                    SMTC_MODEM_HAL_TRACE_WARNING( "Periodic DM can't be send! internal code: %x\n", send_status );
                }
            }
            else
            {
                if( ( get_modem_dm_interval_second( ) > 0 ) && ( modem_get_dm_info_bitfield_periodic( ) > 0 ) )
                {
                    is_pending_dm_status_payload_periodic =
                        dm_status_payload( payload, &payload_length, max_payload, DM_INFO_PERIODIC );

                    send_status =
                        lorawan_api_payload_send( get_modem_dm_port( ), payload, payload_length, UNCONF_DATA_UP,
                                                  smtc_modem_hal_get_time_in_ms( ) + MODEM_TASK_DELAY_MS );

                    if( send_status == LWPSTATE_SEND )
                    {
                        SMTC_MODEM_HAL_TRACE_ARRAY( "DM ", payload, payload_length );
                        SMTC_MODEM_HAL_TRACE_PRINTF( " on Port %d\n", get_modem_dm_port( ) );
                    }
                    else
                    {
                        SMTC_MODEM_HAL_TRACE_WARNING( "Periodic DM can't be send! internal code: %x\n", send_status );
                    }
                }
            }
        }
        break;
    case DM_TASK_NOW:
        if( get_join_state( ) == MODEM_JOINED )
        {
            uint8_t max_payload = lorawan_api_next_max_payload_length_get( );
            uint8_t payload[242];
            uint8_t payload_length;
            is_pending_dm_status_payload_now = dm_status_payload( payload, &payload_length, max_payload, DM_INFO_NOW );

            send_status = lorawan_api_payload_send( get_modem_dm_port( ), payload, payload_length, UNCONF_DATA_UP,
                                                    smtc_modem_hal_get_time_in_ms( ) + MODEM_TASK_DELAY_MS );
            if( send_status == LWPSTATE_SEND )
            {
                SMTC_MODEM_HAL_TRACE_ARRAY( "DM Req ", payload, payload_length );
                SMTC_MODEM_HAL_TRACE_PRINTF( " on Port %d\n", get_modem_dm_port( ) );
            }
            else
            {
                SMTC_MODEM_HAL_TRACE_WARNING( "Requested DM can't be send! internal code: %x\n", send_status );
            }
        }
        break;
    case CRASH_LOG_TASK:
        if( get_join_state( ) == MODEM_JOINED )
        {
            uint8_t payload[CRASH_LOG_SIZE + 1];
            uint8_t payload_length = CRASH_LOG_SIZE + 1;
            // first set the dm byte corresponding to crashlog
            payload[0] = e_inf_crashlog;
            // get the stored crashlog
            smtc_modem_hal_restore_crashlog( &payload[1] );

            SMTC_MODEM_HAL_TRACE_ARRAY( "DM Req CRASH LOG ", payload, payload_length );
            uint8_t max_payload = lorawan_api_next_max_payload_length_get( );
            payload_length      = ( payload_length > max_payload ) ? max_payload : payload_length;
            send_status = lorawan_api_payload_send( get_modem_dm_port( ), payload, payload_length, UNCONF_DATA_UP,
                                                    smtc_modem_hal_get_time_in_ms( ) + MODEM_TASK_DELAY_MS );
            if( send_status == LWPSTATE_SEND )
            {
                // crashlog will be sent => set availability to false
                smtc_modem_hal_set_crashlog_status( false );
            }
        }
        break;
    case FILE_UPLOAD_TASK: {
        int32_t size_file_upload   = 0;
        uint8_t UploadPayload[242] = { 0 };
        set_modem_status_file_upload( false );
        if( get_join_state( ) != MODEM_JOINED )
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "DEVICE NOT JOIN \n" );
            break;
        }
        else if( modem_get_upload_state( ) != MODEM_UPLOAD_START )
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "FileUpload not init \n" );
            break;
        }
        size_file_upload =
            file_upload_gen_uplink( file_upload_context, UploadPayload, lorawan_api_next_max_payload_length_get( ),
                                    MODE_0, lorawan_api_fcnt_up_get( ) );
        if( size_file_upload > 0 )
        {
            set_modem_status_file_upload( true );
            send_status =
                lorawan_api_payload_send( get_modem_dm_port( ), UploadPayload, size_file_upload, UNCONF_DATA_UP,
                                          smtc_modem_hal_get_time_in_ms( ) + MODEM_TASK_DELAY_MS );
        }
        else
        {
            SMTC_MODEM_HAL_TRACE_WARNING( "File upload DONE\n" );
            increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_FILEDONE, 0x00 );
            modem_set_upload_state( MODEM_UPLOAD_NOT_INIT );
        }
        break;
    }
    case STREAM_TASK: {
        uint8_t              stream_payload[242] = { 0 };
        uint8_t              fragment_size;
        uint32_t             frame_cnt;
        stream_return_code_t stream_rc;
        // SMTC_MODEM_HAL_TRACE_MSG( "Supervisor launch STREAM_TASK\n" );

        if( get_join_state( ) != MODEM_JOINED )
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "DEVICE NOT JOINED \n" );
            break;
        }
        else if( modem_get_stream_state( ) != MODEM_STREAM_INIT )
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "Streaming not initialized \n" );
            break;
        }
        // XXX Check if a streaming session is already active
        fragment_size = lorawan_api_next_max_payload_length_get( );
        frame_cnt     = lorawan_api_fcnt_up_get( );
        stream_rc     = stream_get_fragment( ROSE, stream_payload, frame_cnt, &fragment_size );
        // TODO Is this enough to ensure we send everything?
        if( stream_rc == STREAM_OK && fragment_size > 0 )
        {
            send_status =
                lorawan_api_payload_send( task_manager.modem_task[id].fPort, stream_payload, fragment_size,
                                          UNCONF_DATA_UP, smtc_modem_hal_get_time_in_ms( ) + MODEM_TASK_DELAY_MS );
        }
        else
        {
            // Insufficient data or streaming done
            set_modem_status_streaming( false );
            SMTC_MODEM_HAL_TRACE_WARNING( "Stream get fragment FAILED\n" );
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
        s_dm_retrieve_pending_dl_t retrieve;
        get_dm_retrieve_pending_dl( &retrieve );
        if( retrieve.up_count > 0 )
        {
            send_status = lorawan_api_payload_send(
                get_modem_dm_port( ), task_manager.modem_task[id].dataIn, task_manager.modem_task[id].sizeIn,
                task_manager.modem_task[id].PacketType, smtc_modem_hal_get_time_in_ms( ) + MODEM_TASK_DELAY_MS );
        }
        break;
    }
    case ALC_SYNC_TIME_REQ_TASK: {
        if( lorawan_api_modem_certification_is_enabled( ) == true )
        {
            break;
        }
        if( get_join_state( ) == MODEM_JOINED )
        {
            uint8_t  max_payload = lorawan_api_next_max_payload_length_get( );
            uint8_t  tx_buffer_out[ALC_SYNC_TX_PAYLOAD_SIZE_MAX + 1];  // +1 is the e_inf_alcsync ID
            uint8_t  tx_buffer_length_out = 0;
            uint32_t target_send_time = smtc_modem_hal_get_time_in_s( ) + smtc_modem_hal_get_random_nb_in_range( 1, 3 );
            uint8_t  app_time_ans_required = false;
            uint8_t  tx_buff_offset        = 0;
            // AnsRequired bit set to one in both cases: synchronisation lost or the last 30 days
            // synchronisation lost
            if( ( !is_alc_sync_done( alc_sync_context ) ) ||
                ( is_alc_sync_done( alc_sync_context ) &&
                  ( ( smtc_modem_hal_get_time_in_s( ) - alc_sync_get_timestamp_last_correction_s( alc_sync_context ) ) >
                    ( ALC_SYNC_DEFAULT_S_SINCE_LAST_CORRECTION >> 1 ) ) ) )
            {
                app_time_ans_required = true;
            }
            // check first if alc_sync runs on dm port and if yes add dm code
            if( get_modem_dm_port( ) == alc_sync_get_port( alc_sync_context ) )
            {
                tx_buffer_out[tx_buff_offset] = e_inf_alcsync;
                tx_buff_offset++;
            }
            // use target send time with both local compensation and previous alcsync compensation to create payload
            alc_sync_create_uplink_payload( alc_sync_context,
                                            target_send_time + smtc_modem_hal_get_time_compensation_in_s( ) +
                                                alc_sync_get_time_correction_second( alc_sync_context ),
                                            app_time_ans_required, false, max_payload, &tx_buffer_out[tx_buff_offset],
                                            &tx_buffer_length_out );
            // compute final size according to previous offset
            tx_buffer_length_out += tx_buff_offset;
            if( tx_buffer_length_out > 0 )
            {
                send_status =
                    lorawan_api_payload_send_at_time( alc_sync_get_port( alc_sync_context ), tx_buffer_out,
                                                      tx_buffer_length_out, UNCONF_DATA_UP, target_send_time * 1000 );
            }
        }
        alc_sync_nb_time_req++;
        break;
    }
    case ALC_SYNC_ANS_TASK: {
        if( lorawan_api_modem_certification_is_enabled( ) == true )
        {
            break;
        }
        uint8_t  max_payload = lorawan_api_next_max_payload_length_get( );
        uint8_t  tx_buffer_out[ALC_SYNC_TX_PAYLOAD_SIZE_MAX + 1];  // +1 is the e_inf_alcsync ID
        uint8_t  tx_buffer_length_out = 0;
        uint32_t target_send_time     = smtc_modem_hal_get_time_in_s( ) + 2;
        uint8_t  tx_buff_offset       = 0;

        // check first if alc_sync runs on dm port and if yes add dm code
        if( get_modem_dm_port( ) == alc_sync_get_port( alc_sync_context ) )
        {
            tx_buffer_out[tx_buff_offset] = e_inf_alcsync;
            tx_buff_offset++;
        }

        // use target send time with both local compensation and previous alcsync compensation to create payload
        alc_sync_create_uplink_payload( alc_sync_context,
                                        target_send_time + smtc_modem_hal_get_time_compensation_in_s( ) +
                                            alc_sync_get_time_correction_second( alc_sync_context ),
                                        true, true, max_payload, &tx_buffer_out[tx_buff_offset],
                                        &tx_buffer_length_out );

        // compute final size according to previous offset
        tx_buffer_length_out += tx_buff_offset;

        if( tx_buffer_length_out > 0 )
        {
            send_status =
                lorawan_api_payload_send_at_time( alc_sync_get_port( alc_sync_context ), tx_buffer_out,
                                                  tx_buffer_length_out, UNCONF_DATA_UP, target_send_time * 1000 );
        }
        break;
    }
#if defined( LR1110_MODEM )
    case FRAG_TASK: {
        uint8_t max_payload = lorawan_api_next_max_payload_length_get( );

        uint8_t tx_buffer_out[FRAG_UPLINK_LENGTH_MAX];
        uint8_t tx_buffer_length_out = FRAG_UPLINK_LENGTH_MAX;

        dm_frag_uplink_payload( max_payload, tx_buffer_out, &tx_buffer_length_out );

        SMTC_MODEM_HAL_TRACE_ARRAY( "FRAG_TASK", tx_buffer_out, tx_buffer_length_out );
        if( tx_buffer_length_out > 0 )
        {
            lorawan_api_payload_send( get_modem_frag_port( ), tx_buffer_out, tx_buffer_length_out, UNCONF_DATA_UP,
                                      smtc_modem_hal_get_time_in_ms( ) + MODEM_TASK_DELAY_MS );
        }
        break;
    }
#endif  // LR1110_MODEM
    case USER_TASK: {
        send_status = lorawan_api_payload_send(
            get_modem_dm_port( ), task_manager.modem_task[id].dataIn, task_manager.modem_task[id].sizeIn,
            task_manager.modem_task[id].PacketType, smtc_modem_hal_get_time_in_ms( ) + MODEM_TASK_DELAY_MS );
        break;
    }
#if defined( _GNSS_SNIFF_ENABLE )
#if defined( LR1110_MODEM )
    case DM_ALM_DBG_ANS: {
        uint8_t max_payload = lorawan_api_next_max_payload_length_get( );
        uint8_t payload[242];
        uint8_t payload_length;

        dm_alm_dbg_uplink_payload( max_payload, payload, &payload_length );

        SMTC_MODEM_HAL_TRACE_ARRAY( "DM_ALM_DBG_ANS", payload, payload_length );
        if( payload_length > 0 )
        {
            send_status = lorawan_api_payload_send( get_modem_dm_port( ), payload, payload_length, UNCONF_DATA_UP,
                                                    smtc_modem_hal_get_time_in_ms( ) + MODEM_TASK_DELAY_MS );
        }
        break;
    }
#endif  // LR1110_MODEM
#endif  // _GNSS_SNIFF_ENABLE
    default:
        break;
    }

    if( send_status == LWPSTATE_SEND )
    {
        decrement_dm_retrieve_pending_dl( );
    }

    modem_supervisor_remove_task( id );
}

void modem_supervisor_update_task( task_id_t id )
{
    // modem_supervisor_update_downlink_frame( );
    switch( id )
    {
    case JOIN_TASK:
        if( ( get_join_state( ) == MODEM_JOIN_ONGOING ) && ( lorawan_api_isjoined( ) == JOINED ) )
        {
            set_modem_status_joining( false );
            set_modem_status_modem_joined( true );
            // as soon as modem is joined, modem send has to sent a dm report every DM_PERIOD_AFTER_JOIN
            is_first_dm_after_join                = true;
            is_pending_dm_status_payload_periodic = true;
            modem_supervisor_add_task_dm_status( DM_PERIOD_AFTER_JOIN );

            // If clock sync service activated => initiate a new request
            if( alc_sync_is_enabled( alc_sync_context ) == true )
            {
                modem_supervisor_add_task_alc_sync_time_req( DM_PERIOD_AFTER_JOIN + 10 +
                                                             smtc_modem_hal_get_random_nb_in_range( 0, 5 ) );

                alc_sync_nb_time_req = 0;
            }
            if( smtc_modem_hal_get_crashlog_status( ) == true )
            {
                modem_supervisor_add_task_crash_log( DM_PERIOD_AFTER_JOIN + 30 +
                                                     smtc_modem_hal_get_random_nb_in_range( 5, 10 ) );
            }
            increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_JOINED, 0 );
            lorawan_api_class_c_start( );
        }
        else if( get_join_state( ) == MODEM_JOIN_ONGOING )
        {
            set_modem_status_joining( false );
            increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_JOINFAIL, 0 );
            modem_supervisor_add_task_join( );  // relaunch join only if no leave cmd
        }
        else if( ( lorawan_api_isjoined( ) == JOINED ) )  //
        {
            set_modem_status_joining( false );
            lorawan_api_join_status_clear( );  // => manage case leave cmd between end of join and join succeed
        }

        // Disable the duty cycle in in stack case of disabled by host
        if( modem_get_duty_cycle_disabled_by_host( ) == true )
        {
            lorawan_api_duty_cycle_enable_set( false );
        }
        break;
    case DM_TASK:
        if( is_first_dm_after_join == true )
        {
            modem_supervisor_add_task_dm_status( smtc_modem_hal_get_random_nb_in_range( 10, 15 ) );
        }
        else if( get_modem_dm_interval_second( ) > 0 )
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
                increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_TXDONE, MODEM_TX_SUCCESS_WITH_ACK );
            }
            else
            {
                increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_TXDONE, MODEM_TX_SUCCESS );
            }
        }
        else
        {
            increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_TXDONE, MODEM_TX_FAILED );
        }

        // Re-enable the duty cycle in case of Emergency Tx was sent and dutycycle not disabled by host
        if( modem_get_duty_cycle_disabled_by_host( ) == true )
        {
            lorawan_api_duty_cycle_enable_set( false );
        }
        else
        {
            lorawan_api_duty_cycle_enable_set( true );
        }
        break;
    case FILE_UPLOAD_TASK: {
        if( get_modem_status_file_upload( ) == true )
        {
            smodem_task upload_task;  // as soon as you join you send dm report every DM_PERIOD_AFTER_JOIN
            upload_task.id                = FILE_UPLOAD_TASK;
            upload_task.priority          = TASK_HIGH_PRIORITY;
            upload_task.time_to_execute_s = smtc_modem_hal_get_time_in_s( ) + modem_get_upload_avgdelay( ) +
                                            smtc_modem_hal_get_random_nb_in_range( 1, 3 );
            modem_supervisor_add_task( &upload_task );
        }
        break;
    }
    case STREAM_TASK: {
        // SMTC_MODEM_HAL_TRACE_MSG( "Supervisor update STREAM_TASK\n" );
        if( stream_data_pending( ROSE ) )
        {
            modem_supervisor_add_task_stream( );
        }
        else
        {
            SMTC_MODEM_HAL_TRACE_WARNING( "Streaming DONE\n" );
            set_modem_status_streaming( false );
            increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_STREAMDONE, 0 );
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
        if( retrieve.up_count > 0 )
        {
            modem_supervisor_add_task_retrieve_dl( retrieve.up_delay );
        }

        break;
    }
    case ALC_SYNC_TIME_REQ_TASK: {
        uint32_t alc_sync_interval_s = 0;

        if( is_alc_sync_done( alc_sync_context ) )
        {
            // Synchronized with the network
            if( is_alc_sync_time_valid( alc_sync_context ) == false )
            {
                // synchronisation lost
                // Send event
                increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_TIME_UPDATED, 0 );

                alc_sync_set_sync_lost( alc_sync_context );

                alc_sync_nb_time_req = 1;

                alc_sync_interval_s = MIN( ALCSYNC_PERIOD1_RETRY, alc_sync_get_interval_second( alc_sync_context ) );
            }
            else
            {
                if( alc_sync_get_interval_second( alc_sync_context ) > 0 )
                {
                    alc_sync_interval_s = alc_sync_get_interval_second( alc_sync_context );
                }
            }
        }
        else
        {
            if( alc_sync_nb_time_req < ALCSYNC_NB_REQ_PERIOD1 )
            {
                alc_sync_interval_s = MIN( ALCSYNC_PERIOD1_RETRY, alc_sync_get_interval_second( alc_sync_context ) );
            }
            else if( alc_sync_nb_time_req < ( ALCSYNC_NB_REQ_PERIOD1 + ALCSYNC_NB_REQ_PERIOD2 ) )
            {
                alc_sync_interval_s = MIN( ALCSYNC_PERIOD2_RETRY, alc_sync_get_interval_second( alc_sync_context ) );
            }
            else
            {
                alc_sync_interval_s = MIN( ALCSYNC_PERIOD_RETRY, alc_sync_get_interval_second( alc_sync_context ) );
            }
        }

        alc_sync_interval_s = MIN( alc_sync_interval_s, alc_sync_get_time_left_connection_lost( alc_sync_context ) );

        modem_supervisor_add_task_alc_sync_time_req( alc_sync_interval_s +
                                                     smtc_modem_hal_get_signed_random_nb_in_range( 0, 30 ) );

        if( is_alc_sync_time_valid( alc_sync_context ) == false )
        {
            s_dm_retrieve_pending_dl_t retrieve;
            get_dm_retrieve_pending_dl( &retrieve );
            if( retrieve.up_count == 0 )
            {
                retrieve.up_delay = smtc_modem_hal_get_random_nb_in_range( 3, 8 );
                set_dm_retrieve_pending_dl( 1, retrieve.up_delay );
                modem_supervisor_add_task_retrieve_dl( retrieve.up_delay );
            }
        }
        break;
    }
    case ALC_SYNC_ANS_TASK:
        if( alc_sync_get_nb_transmission( alc_sync_context ) > 0 )
        {
            modem_supervisor_add_task_alc_sync_ans( smtc_modem_hal_get_random_nb_in_range( 128, 150 ) );
        }
        break;
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
                ( int32_t )( task_manager.modem_task[i].time_to_execute_s - smtc_modem_hal_get_time_in_s( ) );

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
                    ( int32_t )( task_manager.modem_task[i].time_to_execute_s - smtc_modem_hal_get_time_in_s( ) );
                if( next_task_time_tmp < next_task_time )
                {
                    next_task_time            = next_task_time_tmp;
                    task_manager.next_task_id = ( task_id_t ) i;
                }
            }
        }
    }

    if( next_task_time > 0 )
    {
        task_manager.sleep_duration = next_task_time;
        task_manager.next_task_id   = IDLE_TASK;
        return ( next_task_time * 1000 );
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
        smtc_modem_hal_disable_all_irq( );
        // workaround to avoid issue when reset too fast after a store context
        smtc_modem_hal_wait_us( 20000 );

        smtc_modem_hal_reset_mcu( );
    }

    uint32_t alarm                 = modem_get_user_alarm( );
    int32_t  user_alarm_in_seconds = MODEM_MAX_ALARM_S;
    // manage the user alarm
    if( alarm != 0 )
    {
        user_alarm_in_seconds = ( int32_t )( alarm - smtc_modem_hal_get_time_in_s( ) );

        if( user_alarm_in_seconds <= 0 )
        {
            increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_ALARM, 0 );
            modem_set_user_alarm( 0 );
            user_alarm_in_seconds = MODEM_MAX_ALARM_S;
        }
    }
    backoff_mobile_static( );

    // smtc_modem_hal_reload_wdog( );
    // case Lorawan stack already in use
    LpState = lorawan_api_state_get( );

    if( ( LpState != LWPSTATE_IDLE ) && ( LpState != LWPSTATE_ERROR ) && ( LpState != LWPSTATE_INVALID ) )
    {
        LpState = lorawan_api_process( &AvailableRxPacket );
        if( ( AvailableRxPacket != NO_LORA_RXPACKET_AVAILABLE ) && ( LpState == LWPSTATE_TX_WAIT ) )
        {  // there is an applicative downlink and a nwk frame to send so lr1mac is in the state tx wait
            if( task_manager.next_task_id != IDLE_TASK )
            {
                // modem_supervisor_update_downlink_frame( );
            }
        }
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

    uint32_t sleep_time = 0;
    int32_t  dtc_ms     = lorawan_api_next_free_duty_cycle_ms_get( );
    if( dtc_ms > 0 )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "Duty Cycle, remaining time: %dms\n", dtc_ms );
        sleep_time = ( uint32_t ) dtc_ms;
    }
    // Don't call the supervisor if modem is suspend
    else if( ( get_modem_suspend( ) == MODEM_NOT_SUSPEND ) && ( get_modem_muted( ) != MODEM_INFINITE_MUTE ) )
    {
        // you reach this step only if lorawan stack is in idle mode and TOA is available (no dtc blocking)
        sleep_time = modem_supervisor_scheduler( );
    }
    else
    {
        // Set Max time (7FFFFFFF>>10 ie /1024 to be compatible with *1000)
        sleep_time = 0x7FFFFFFE;
    }

    alarm = modem_get_user_alarm( );
    if( alarm != 0 )
    {
        user_alarm_in_seconds = ( int32_t )( alarm - smtc_modem_hal_get_time_in_s( ) );
        if( user_alarm_in_seconds <= 0 )
        {
            user_alarm_in_seconds = 0;
        }
    }

    sleep_time = MIN( sleep_time, ( uint32_t ) user_alarm_in_seconds * 1000 );
    // SMTC_MODEM_HAL_TRACE_INFO( "Next task in %d\n", sleep_time );
    return ( sleep_time );
}

void backoff_mobile_static( void )
{
    uint16_t                 nb_usr_adr_mobile_timeout;
    uint16_t                 nb_current_usr_adr_mobile_timeout;
    smtc_modem_adr_profile_t usr_adr_profile;

    usr_adr_profile                   = get_modem_adr_profile( );
    nb_usr_adr_mobile_timeout         = modem_get_adr_mobile_timeout_config( );
    nb_current_usr_adr_mobile_timeout = modem_get_current_adr_mobile_count( );

    // if nb_usr_adr_mobile_timeout = 0 the feature switch from mobile to static mode isn't activated
    if( ( ( nb_current_usr_adr_mobile_timeout > nb_usr_adr_mobile_timeout ) &&
          ( usr_adr_profile != SMTC_MODEM_ADR_PROFILE_NETWORK_CONTROLLED ) && ( nb_usr_adr_mobile_timeout != 0 ) ) ||
        ( ( usr_adr_profile != SMTC_MODEM_ADR_PROFILE_NETWORK_CONTROLLED ) &&
          ( lorawan_api_dr_strategy_get( ) == STATIC_ADR_MODE ) ) )
    {
        set_modem_adr_profile( SMTC_MODEM_ADR_PROFILE_NETWORK_CONTROLLED, NULL, 0 );
        increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_TIMEOUT_ADR_CHANGED, 0 );
    }
    if( ( usr_adr_profile == SMTC_MODEM_ADR_PROFILE_NETWORK_CONTROLLED ) &&
        ( modem_available_new_link_adr_request( ) == true ) )
    {
        increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_NEW_LINK_ADR, 0 );
    }
}

uint8_t modem_supervisor_update_downlink_frame( uint8_t* data, uint8_t data_length, lr1mac_down_metadata_t* metadata )
{
    s_modem_dwn_t dwnframe;

    set_modem_downlink_frame( data, data_length, metadata );
    get_modem_downlink_frame( &dwnframe );
    if( dwnframe.port == get_modem_dm_port( ) )
    {
        dm_downlink( dwnframe.data, dwnframe.length );
    }
    else if( dwnframe.port == alc_sync_get_port( alc_sync_context ) )
    {
        uint8_t alc_sync_status = alc_sync_parser( alc_sync_context, dwnframe.data, dwnframe.length );

        if( ( ( alc_sync_status >> ALC_SYNC_APP_TIME_ANS ) & 0x1 ) == 1 )
        {
            increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_TIME_UPDATED, 0 );

            // Remove all alc sync task.
            modem_supervisor_remove_task_alc_sync( );

            modem_supervisor_add_task_alc_sync_time_req( alc_sync_get_interval_second( alc_sync_context ) +
                                                         smtc_modem_hal_get_signed_random_nb_in_range( -30, 30 ) );
        }

        if( alc_sync_status & ( ~( 1 << ALC_SYNC_APP_TIME_ANS ) ) )
        {
            if( ( ( alc_sync_status >> ALC_SYNC_DEVICE_APP_TIME_PERIODICITY_REQ ) & 0x1 ) == 1 )
            {
                // If periodic time request is configured, add task to handle it
                modem_supervisor_add_task_alc_sync_time_req( alc_sync_get_interval_second( alc_sync_context ) +
                                                             smtc_modem_hal_get_signed_random_nb_in_range( -30, 30 ) );
            }
            // When a request requiring an answer is requested, add task to handle it
            modem_supervisor_add_task_alc_sync_ans( 1 );
        }
    }
#if defined( LR1110_MODEM )
    else if( dwnframe.port == get_modem_frag_port( ) )
    {
        int8_t frag_status = frag_parser( dwnframe.data, dwnframe.length );
        if( frag_status & FRAG_CMD_ERROR )
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "ERROR: Failed to parse frag message\n" );
        }
        else if( frag_status != 0x00 )
        {
            // An answer to a request, or a request is required, add a task for it
            modem_supervisor_add_task_frag( 1 );
        }
        else
        {
            // Nothing to do
        }
    }
#endif  // LR1110_MODEM
    else
    {
        return 1;
    }
    return 0;
}
