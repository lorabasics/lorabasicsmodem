/*!
 * \file      radio_planner.c
 *
 * \brief     Radio planner implementation
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
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMsPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdlib.h>
#include <stdio.h>
#include "radio_planner.h"

//
// Private planner variable declaration
//

#if defined( PERF_TEST_ENABLED )
static uint32_t tx_done_count    = 0;
static uint32_t rx_done_count    = 0;
static uint32_t rx_timeout_count = 0;
#endif
//
// Private planner utilities declaration
//

/*!
 *
 */
static void rp_task_free( const radio_planner_t* rp, rp_task_t* task );

/*!
 *
 */
static void rp_task_update_time( radio_planner_t* rp, uint32_t now );

/*!
 *
 */
static void rp_task_arbiter( radio_planner_t* rp, const char* caller_func_name );

/*!
 *
 */
static void rp_irq_get_status( radio_planner_t* rp, const uint8_t hook_id );

/*!
 *
 */
static void rp_task_compute_ranking( radio_planner_t* rp );

/*!
 *
 */
static void rp_task_launch_current( radio_planner_t* rp );

/*!
 *
 */
static uint8_t rp_task_select_next( radio_planner_t* rp, const uint32_t now );

/*!
 *
 */
static rp_next_state_status_t rp_task_get_next( radio_planner_t* rp, uint32_t* duration, uint8_t* task_id,
                                                const uint32_t now );

/*!
 *
 */
static uint8_t rp_task_find_highest_priority( const radio_planner_t* rp, uint8_t* vector, uint8_t length );

/*!
 *
 */
rp_hook_status_t rp_get_pkt_payload( radio_planner_t* rp, const rp_task_t* task );

/*!
 *
 */
static void rp_set_alarm( radio_planner_t* rp, const uint32_t alarm_in_ms );

/*!
 *
 */
static void rp_timer_irq( radio_planner_t* rp );

/*!
 *
 */
static void rp_radio_irq( radio_planner_t* rp );

/*!
 *
 */
static void rp_task_call_aborted( radio_planner_t* rp );
/*!
 *
 */
static void rp_consumption_statistics_updated( radio_planner_t* rp, const uint8_t hook_id, const uint32_t time );
// Radio planner callbacks
//

/*!
 * \ref rp_radio_irq_callback is part of the public radio planner API
 */

/*!
 *
 */
static void rp_timer_irq_callback( void* obj );

/*!
 *
 */
static void rp_hook_callback( radio_planner_t* rp, uint8_t id );

//
// Private debug utilities declaration
//

/*!
 *
 */
static void rp_task_print( const radio_planner_t* rp, const rp_task_t* task );

//
// Public planner API implementation
//

void rp_init( radio_planner_t* rp, ral_t* ral )
{
    rp->ral = ral;

    for( int32_t i = 0; i < RP_NB_HOOKS; i++ )
    {
        rp->tasks[i].hook_id            = i;
        rp->tasks[i].type               = RP_TASK_TYPE_NONE;
        rp->tasks[i].priority           = 0;
        rp->tasks[i].state              = RP_TASK_STATE_FINISHED;
        rp->tasks[i].start_time_ms      = 0;
        rp->tasks[i].start_time_init_ms = 0;
        rp->tasks[i].duration_time_ms   = 0;
        rp->hooks[i]                    = NULL;
        rp->irq_timestamp_ms[i]         = 0;
        rp->status[i]                   = RP_STATUS_TASK_ABORTED;
    }
    rp_task_free( rp, &rp->priority_task );
    rp_stats_init( &rp->stats );
    rp->radio_task_id           = 0;
    rp->timer_task_id           = 0;
    rp->hook_to_execute         = 0;
    rp->hook_to_execute_time_ms = 0;
    rp->timer_state             = RP_TIMER_STATE_IDLE;
    rp->semaphore_radio         = 0;
    rp->semaphore_abort_radio   = 0;
    rp->timer_value             = 0;
    rp->timer_hook_id           = 0;
    rp->next_state_status       = RP_STATUS_NO_MORE_TASK_SCHEDULE;
}

rp_hook_status_t rp_hook_init( radio_planner_t* rp, const uint8_t id, void ( *callback )( void* context ), void* hook )
{
    if( id >= RP_NB_HOOKS )
    {
        return RP_HOOK_STATUS_ID_ERROR;
    }
    rp->hook_callbacks[id] = callback;
    rp->hooks[id]          = hook;
    return RP_HOOK_STATUS_OK;
}

rp_hook_status_t rp_hook_get_id( const radio_planner_t* rp, const void* hook, uint8_t* id )
{
    for( int32_t i = 0; i < RP_NB_HOOKS; i++ )
    {
        if( hook == rp->hooks[i] )
        {
            *id = i;
            return RP_HOOK_STATUS_OK;
        }
    }
    return RP_HOOK_STATUS_ID_ERROR;
}

rp_hook_status_t rp_task_enqueue( radio_planner_t* rp, const rp_task_t* task, uint8_t* payload, uint16_t payload_size,
                                  const rp_radio_params_t* radio_params )
{
    rp_bsp_critical_section_begin( );

    uint8_t hook_id = task->hook_id;
    if( hook_id >= RP_NB_HOOKS )
    {
        rp_bsp_critical_section_end( );
        return RP_HOOK_STATUS_ID_ERROR;
    }
    if( rp->tasks[hook_id].state == RP_TASK_STATE_RUNNING )
    {
        BSP_DBG_TRACE_PRINTF_RP( " RP: Task enqueue impossible. Task is already running\n" );
        rp_bsp_critical_section_end( );
        return RP_TASK_STATUS_ALREADY_RUNNING;
    }

    rp->tasks[hook_id]                    = *task;
    rp->radio_params[hook_id]             = *radio_params;
    rp->payload[hook_id]                  = payload;
    rp->payload_size[hook_id]             = payload_size;
    rp->tasks[hook_id].priority           = ( rp->tasks[hook_id].state * RP_NB_HOOKS ) + hook_id;
    rp->tasks[hook_id].start_time_init_ms = rp->tasks[hook_id].start_time_ms;
    BSP_DBG_TRACE_PRINTF_RP( "RP: Task #%u enqueue with #%u priority\n", hook_id, rp->tasks[hook_id].priority );
    rp_task_compute_ranking( rp );
    if( rp->semaphore_radio == 0 )
    {
        rp_task_arbiter( rp, __func__ );
    }
    rp_bsp_critical_section_end( );
    return RP_HOOK_STATUS_OK;
}

rp_hook_status_t rp_task_abort( radio_planner_t* rp, const uint8_t hook_id )
{
    rp_bsp_critical_section_begin( );
    if( hook_id >= RP_NB_HOOKS )
    {
        rp_bsp_critical_section_end( );
        return RP_HOOK_STATUS_ID_ERROR;
    }
    if( rp->tasks[hook_id].state == RP_TASK_STATE_RUNNING )
    {
        ral_set_sleep( rp->ral );

        // Shut Down the TCXO
        ral_set_tcxo_off( rp->ral );

        rp_consumption_statistics_updated( rp, hook_id, rp_bsp_timestamp_get( ) );
    }
    rp_task_free( rp, &( rp->tasks[hook_id] ) );
    rp->status[hook_id] = RP_STATUS_TASK_ABORTED;

    if( rp->semaphore_radio == 0 )
    {
        rp_task_arbiter( rp, __func__ );
    }
    rp_bsp_critical_section_end( );
    return RP_HOOK_STATUS_OK;
}

void rp_get_status( const radio_planner_t* rp, const uint8_t id, uint32_t* irq_timestamp_ms, rp_status_t* status )
{
    *irq_timestamp_ms = rp->irq_timestamp_ms[id];
    *status           = rp->status[id];
}

rp_stats_t rp_get_stats( const radio_planner_t* rp )
{
    return rp->stats;
}

//
// Private planner utilities implementation
//

static void rp_task_free( const radio_planner_t* rp, rp_task_t* task )
{
    task->hook_id            = 0;
    task->start_time_ms      = 0;
    task->start_time_init_ms = 0;
    task->duration_time_ms   = 0;
    task->type               = RP_TASK_TYPE_NONE;
    task->state              = RP_TASK_STATE_FINISHED;
}

static void rp_task_update_time( radio_planner_t* rp, uint32_t now )
{
    for( int32_t i = 0; i < RP_NB_HOOKS; i++ )
    {
        if( rp->tasks[i].state == RP_TASK_STATE_ASAP )
        {
            if( ( int32_t )( now - rp->tasks[i].start_time_init_ms ) > 0 )
            {
                rp->tasks[i].start_time_ms = now;
            }
            // An asap task is automatically switch in schedule task after RP_TASK_ASAP_TO_SCHEDULE_TRIG_TIME ms

            if( ( int32_t )( now - rp->tasks[i].start_time_init_ms ) > RP_TASK_ASAP_TO_SCHEDULE_TRIG_TIME )
            {
                rp->tasks[i].state = RP_TASK_STATE_SCHEDULE;
                // Schedule the task @ now + RP_TASK_RE_SCHEDULE_OFFSET_TIME
                // seconds
                rp->tasks[i].start_time_ms = now + RP_TASK_RE_SCHEDULE_OFFSET_TIME;
                rp->tasks[i].priority      = ( rp->tasks[i].state * RP_NB_HOOKS ) + i;
                BSP_DBG_TRACE_PRINTF_RP( "RP: WARNING - SWITCH TASK FROM ASAP TO SCHEDULE \n" );
                rp_task_compute_ranking( rp );
            }
        }
    }

    if( ( rp->tasks[rp->radio_task_id].state == RP_TASK_STATE_RUNNING ) &&
        ( ( rp->tasks[rp->radio_task_id].type == RP_TASK_TYPE_RX_LORA ) ||
          ( rp->tasks[rp->radio_task_id].type == RP_TASK_TYPE_RX_FSK ) ) )
    {
        rp->tasks[rp->radio_task_id].duration_time_ms =
            now + RP_MARGIN_DELAY + 2 - rp->tasks[rp->radio_task_id].start_time_ms;
        BSP_DBG_TRACE_PRINTF_RP( " RP: Extended duration of radio task #%u time to %lu ms\n", rp->radio_task_id, now );
    }
}

static void rp_task_arbiter( radio_planner_t* rp, const char* caller_func_name )
{
    uint32_t now = rp_bsp_timestamp_get( );

    // Update time for ASAP task to now. But, also extended duration in case of
    // running task is a RX task
    rp_task_update_time( rp, now );

    // Select the high priority task
    if( rp_task_select_next( rp, now ) == RP_SOMETHING_TO_DO )
    {  // Next task exists
        int32_t delay = ( int32_t )( rp->priority_task.start_time_ms - now );
        BSP_DBG_TRACE_PRINTF_RP(
            " RP: Arbiter has been called by %s and priority-task #%u, timer "
            "hook "
            "#%u, delay %ld, now %lu\n ",
            caller_func_name, rp->priority_task.hook_id, rp->timer_hook_id, delay, now );

        // Case where the high priority task is in the future
        if( delay > RP_MARGIN_DELAY )
        {  // The high priority task is in the future
            BSP_DBG_TRACE_PRINTF_RP( " RP: High priority task is in the future\n" );
        }
        // Case where the high priority task is in the past, error case
        else if( delay < 0 )
        {  // The high priority task is in the past, error case
            if( rp->priority_task.state != RP_TASK_STATE_RUNNING )
            {
                rp->stats.rp_error++;
                BSP_DBG_TRACE_PRINTF_RP( " RP: ERROR - delay #%ld - hook #%u\n", delay, rp->priority_task.hook_id );

                rp->tasks[rp->priority_task.hook_id].state = RP_TASK_STATE_ABORTED;
            }
        }
        // Case where the high priority task is now
        else
        {
            if( rp->tasks[rp->radio_task_id].state == RP_TASK_STATE_RUNNING )
            {  // Radio is already running
                if( rp->tasks[rp->radio_task_id].hook_id != rp->priority_task.hook_id )
                {  // priority task not equal to radio task => abort radio task
                    rp->tasks[rp->radio_task_id].state = RP_TASK_STATE_ABORTED;
                    BSP_DBG_TRACE_PRINTF_RP( "RP: Abort running task with hook #%u\n", rp->radio_task_id );
                    ral_clear_irq_status( rp->ral, RAL_IRQ_ALL );
                    ral_set_sleep( rp->ral );

                    // Shut Down the TCXO
                    ral_set_tcxo_off( rp->ral );

                    rp->semaphore_abort_radio = rp_bsp_irq_get_pending( );

                    rp_consumption_statistics_updated( rp, rp->radio_task_id, rp_bsp_timestamp_get( ) );

                    rp->radio_task_id                  = rp->priority_task.hook_id;
                    rp->tasks[rp->radio_task_id].state = RP_TASK_STATE_RUNNING;
                    rp_task_launch_current( rp );
                }  // else case already managed during enqueue task
            }
            else
            {  // Radio is sleeping start priority task on radio
                rp->radio_task_id                  = rp->priority_task.hook_id;
                rp->tasks[rp->radio_task_id].state = RP_TASK_STATE_RUNNING;
                rp_task_launch_current( rp );
            }
        }
        // Timer has expired on a not priority task => Have to abort this task
        int32_t tmp = ( int32_t )( rp->tasks[rp->timer_hook_id].start_time_ms - now );

        if( ( tmp > 0 ) && ( tmp < RP_MARGIN_DELAY ) && ( rp->next_state_status == RP_STATUS_HAVE_TO_SET_TIMER ) &&
            ( rp->timer_hook_id != rp->priority_task.hook_id ) )
        {
            BSP_DBG_TRACE_PRINTF_RP( " RP: Aborted task with hook #%u - not a priority task\n ", rp->timer_hook_id );
            rp->tasks[rp->timer_hook_id].state = RP_TASK_STATE_ABORTED;
        }

        // Execute the garbage collection if the radio isn't running
        if( rp->tasks[rp->radio_task_id].state != RP_TASK_STATE_RUNNING )
        {
            rp_task_call_aborted( rp );
        }

        // Set the Timer to the next Task
        rp->next_state_status = rp_task_get_next( rp, &rp->timer_value, &rp->timer_hook_id, rp_bsp_timestamp_get( ) );

        if( rp->next_state_status == RP_STATUS_HAVE_TO_SET_TIMER )
        {
            if( rp->timer_value > RP_MARGIN_DELAY )
            {
                rp_set_alarm( rp, rp->timer_value - RP_MARGIN_DELAY );
            }
            else
            {
                rp_set_alarm( rp, 1 );
            }
        }
    }
    else
    {  // No more tasks in the radio planner
        rp_task_call_aborted( rp );
        BSP_DBG_TRACE_PRINTF_RP( " RP: No more active tasks\n" );
    }
}

static void rp_irq_get_status( radio_planner_t* rp, const uint8_t hook_id )
{
    ral_irq_t radio_irq = 0;

#if defined( SX126X )
    // TODO remove when ral sx126 is ready
    ral_get_irq_status( rp->ral, &radio_irq );
#else
    ral_process_irq( rp->ral, &radio_irq );
#endif

    BSP_DBG_TRACE_PRINTF_RP( " RP: IRQ source - 0x%04X\n", radio_irq );
    ral_clear_irq_status( rp->ral, RAL_IRQ_ALL );
    // Do not modify the order of the next if / else if process
    if( ( radio_irq & RAL_IRQ_TX_DONE ) == RAL_IRQ_TX_DONE )
    {
        rp->status[hook_id] = RP_STATUS_TX_DONE;
#if defined( PERF_TEST_ENABLED )
        tx_done_count++;
        BSP_PERF_TEST_TRACE_PRINTF( "RADIO IRQ: TX DONE\n" );
        BSP_PERF_TEST_TRACE_PRINTF( "tx_done_count:%lu\n", tx_done_count );
#endif
    }
    else if( ( ( radio_irq & RAL_IRQ_RX_TIMEOUT ) == RAL_IRQ_RX_TIMEOUT ) ||
             ( ( radio_irq & RAL_IRQ_RX_HDR_ERROR ) == RAL_IRQ_RX_HDR_ERROR ) ||
             ( ( radio_irq & RAL_IRQ_RX_CRC_ERROR ) == RAL_IRQ_RX_CRC_ERROR ) )
    {
        rp->status[hook_id] = RP_STATUS_RX_TIMEOUT;
#if defined( PERF_TEST_ENABLED )
        rx_timeout_count++;
        BSP_PERF_TEST_TRACE_PRINTF( "RADIO IRQ: RX_TIMEOUT or HDR_ERROR or CRC_ERROR\n" );
        BSP_PERF_TEST_TRACE_PRINTF( "rx_timeout_count:%lu\n", rx_timeout_count );
#endif
    }
    else if( ( radio_irq & RAL_IRQ_RX_DONE ) == RAL_IRQ_RX_DONE )
    {
        rp->status[hook_id] = RP_STATUS_RX_PACKET;
        rp_get_pkt_payload( rp, &rp->tasks[hook_id] );
#if defined( PERF_TEST_ENABLED )
        rx_done_count++;
        BSP_PERF_TEST_TRACE_PRINTF( "RADIO IRQ: RX_DONE\n" );
        BSP_PERF_TEST_TRACE_PRINTF( "rx_done_count:%lu\n", rx_done_count );
#endif
    }

    else if( ( radio_irq & RAL_IRQ_CAD_OK ) == RAL_IRQ_CAD_OK )
    {
        rp->status[hook_id] = RP_STATUS_CAD_POSITIVE;
    }
    else if( ( radio_irq & RAL_IRQ_CAD_DONE ) == RAL_IRQ_CAD_DONE )
    {
        rp->status[hook_id] = RP_STATUS_CAD_NEGATIVE;
    }
    else
    {
        BSP_DBG_TRACE_PRINTF_RP( " RP: ERROR - IRQ source 0x%04X unknown\n", radio_irq );
        rp->status[hook_id] = RP_STATUS_TASK_ABORTED;
    }
}

static void rp_task_compute_ranking( radio_planner_t* rp )
{
    uint8_t rank;
    uint8_t ranks_temp[RP_NB_HOOKS];

    for( int32_t i = 0; i < RP_NB_HOOKS; i++ )
    {
        ranks_temp[i] = rp->tasks[i].priority;
    }
    for( int32_t i = 0; i < RP_NB_HOOKS; i++ )
    {
        rank             = rp_task_find_highest_priority( rp, &( ranks_temp[0] ), RP_NB_HOOKS );
        ranks_temp[rank] = 0xFF;
        rp->rankings[i]  = rank;
    }
}

static void rp_task_launch_current( radio_planner_t* rp )
{
    uint8_t id = rp->radio_task_id;
    BSP_DBG_TRACE_PRINTF_RP( " RP: Launch task #%u and start radio state %u, type %u\n", id, rp->tasks[id].state,
                             rp->tasks[id].type );

    rp_task_print( rp, &rp->tasks[id] );

    // Turn on the TCXO
    ral_set_tcxo_on( rp->ral );

    switch( rp->tasks[id].type )
    {
    case RP_TASK_TYPE_TX_LORA:
        ral_init( rp->ral );
        ral_setup_tx_lora( rp->ral, &rp->radio_params[id].tx.lora );
        ral_set_pkt_payload( rp->ral, rp->payload[id], rp->payload_size[id] );
        // Wait the exact time
        while( ( int32_t )( rp->tasks[id].start_time_ms - rp_bsp_timestamp_get( ) ) > 0 )
        {
            // Do nothing
        }
        ral_set_tx( rp->ral );
        rp_stats_set_tx_timestamp( &rp->stats, rp_bsp_timestamp_get( ) );
        break;
    case RP_TASK_TYPE_RX_LORA:
#if defined( SX126X )
        ral_init( rp->ral );
#endif
        ral_setup_rx_lora( rp->ral, &rp->radio_params[id].rx.lora );
        // Wait the exact time
        while( ( int32_t )( rp->tasks[id].start_time_ms - rp_bsp_timestamp_get( ) ) > 0 )
        {
        }
        ral_set_rx( rp->ral, rp->radio_params[id].rx.timeout_in_ms );

        rp_stats_set_rx_timestamp( &rp->stats, rp_bsp_timestamp_get( ) );
        break;
    case RP_TASK_TYPE_TX_FSK:
        ral_init( rp->ral );
        ral_setup_tx_gfsk( rp->ral, &rp->radio_params[id].tx.gfsk );
        ral_set_pkt_payload( rp->ral, rp->payload[id], rp->payload_size[id] );
        // Wait the exact time
        while( ( int32_t )( rp->tasks[id].start_time_ms - rp_bsp_timestamp_get( ) ) > 0 )
        {
        }
        ral_set_tx( rp->ral );

        rp_stats_set_tx_timestamp( &rp->stats, rp_bsp_timestamp_get( ) );
        break;
    case RP_TASK_TYPE_RX_FSK:
        ral_init( rp->ral );
        ral_setup_rx_gfsk( rp->ral, &rp->radio_params[id].rx.gfsk );
        // Wait the exact time
        while( ( int32_t )( rp->tasks[id].start_time_ms - rp_bsp_timestamp_get( ) ) > 0 )
        {
        }
        ral_set_rx( rp->ral, rp->radio_params[id].rx.timeout_in_ms );

        rp_stats_set_rx_timestamp( &rp->stats, rp_bsp_timestamp_get( ) );
        break;
    case RP_TASK_TYPE_CAD: {
        ral_lora_cad_params_t cad_params = {
            .cad_symb_nb          = RAL_LORA_CAD_01_SYMB,
            .cad_det_peak_in_symb = 0,                  // not used
            .cad_det_min_in_symb  = 0,                  // not used
            .cad_exit_mode        = RAL_LORA_CAD_ONLY,  // not used
            .cad_timeout_in_ms    = 0,                  // not used
        };
        ral_setup_rx_lora( rp->ral, &rp->radio_params[id].rx.lora );
        ral_setup_cad( rp->ral, &cad_params );
        while( ( int32_t )( rp->tasks[id].start_time_ms - rp_bsp_timestamp_get( ) ) > 0 )
        {
        }
        ral_set_cad( rp->ral );
        break;
    }
    default:
        BSP_DBG_TRACE_PRINTF_RP( " RP: ERROR - Task type unknown\n" );
        // Shut Down the TCXO
        ral_set_tcxo_off( rp->ral );
        break;
    }
}

static uint8_t rp_task_select_next( radio_planner_t* rp, const uint32_t now )
{
    uint8_t  hook_to_exe_tmp      = 0xFF;
    uint32_t hook_time_to_exe_tmp = 0;
    uint32_t time_tmp             = 0;
    uint8_t  rank                 = 0;
    uint8_t  hook_id              = 0;
    for( hook_id = 0; hook_id < RP_NB_HOOKS; hook_id++ )
    {  // Garbage collector
        if( ( rp->tasks[hook_id].state == RP_TASK_STATE_SCHEDULE ) &&
            ( ( ( int32_t )( rp->tasks[hook_id].start_time_ms - now ) < 0 ) ) )
        {
            rp->tasks[hook_id].state = RP_TASK_STATE_ABORTED;
        }
    }
    for( hook_id = 0; hook_id < RP_NB_HOOKS; hook_id++ )
    {
        rank = rp->rankings[hook_id];
        if( ( ( rp->tasks[rank].state < RP_TASK_STATE_RUNNING ) &&
              ( ( int32_t )( rp->tasks[rank].start_time_ms - now ) >= 0 ) ) ||
            ( rp->tasks[rank].state == RP_TASK_STATE_RUNNING ) )
        {
            hook_to_exe_tmp      = rp->tasks[rank].hook_id;
            hook_time_to_exe_tmp = rp->tasks[rank].start_time_ms;
            break;
        }
    }
    if( hook_id == RP_NB_HOOKS )
    {
        return RP_NO_MORE_TASK;
    }

    for( int32_t i = hook_id; i < RP_NB_HOOKS; i++ )
    {
        rank = rp->rankings[i];
        if( ( ( rp->tasks[rank].state < RP_TASK_STATE_RUNNING ) &&
              ( ( int32_t )( rp->tasks[rank].start_time_ms - now ) >= 0 ) ) ||
            ( rp->tasks[rank].state == RP_TASK_STATE_RUNNING ) )
        {
            time_tmp = rp->tasks[rank].start_time_ms + rp->tasks[rank].duration_time_ms;

            int32_t tmp = ( int32_t )( time_tmp - hook_time_to_exe_tmp );
            if( ( tmp < 0 ) && ( ( int32_t )( time_tmp - now ) >= 0 ) )
            {
                hook_to_exe_tmp      = rp->tasks[rank].hook_id;
                hook_time_to_exe_tmp = rp->tasks[rank].start_time_ms;
            }
        }
    }
    rp->priority_task = rp->tasks[hook_to_exe_tmp];
    return RP_SOMETHING_TO_DO;
}

static rp_next_state_status_t rp_task_get_next( radio_planner_t* rp, uint32_t* duration, uint8_t* task_id,
                                                const uint32_t now )
{
    uint8_t  hook_id  = 0;
    uint8_t  index    = 0;
    uint32_t time_tmp = now;

    for( hook_id = 0; hook_id < RP_NB_HOOKS; hook_id++ )
    {  // Garbage collector
        if( ( rp->tasks[hook_id].state == RP_TASK_STATE_SCHEDULE ) &&
            ( ( ( int32_t )( rp->tasks[hook_id].start_time_ms - time_tmp ) < 0 ) ) )
        {
            rp->tasks[hook_id].state = RP_TASK_STATE_ABORTED;
        }
    }
    for( hook_id = 0; hook_id < RP_NB_HOOKS; hook_id++ )
    {
        if( ( rp->tasks[hook_id].state < RP_TASK_STATE_RUNNING ) &&
            ( ( ( int32_t )( rp->tasks[hook_id].start_time_ms - time_tmp ) >= 0 ) ) )
        {
            time_tmp = rp->tasks[hook_id].start_time_ms;
            index    = hook_id;
            break;
        }
    }
    if( hook_id == RP_NB_HOOKS )
    {
        return RP_STATUS_NO_MORE_TASK_SCHEDULE;
    }

    for( uint8_t i = hook_id; i < RP_NB_HOOKS; i++ )
    {
        if( ( rp->tasks[i].state < RP_TASK_STATE_RUNNING ) &&
            ( ( int32_t )( rp->tasks[i].start_time_ms - time_tmp ) < 0 ) &&
            ( ( int32_t )( rp->tasks[i].start_time_ms - now ) >= 0 ) )
        {
            time_tmp = rp->tasks[i].start_time_ms;
            index    = i;
        }
    }
    *task_id  = index;
    *duration = time_tmp - now;
    return RP_STATUS_HAVE_TO_SET_TIMER;
}

static uint8_t rp_task_find_highest_priority( const radio_planner_t* rp, uint8_t* vector, uint8_t length )
{
    uint8_t priority_high = 0xFF;
    uint8_t index         = 0;

    for( int32_t i = 0; i < length; i++ )
    {
        if( vector[i] <= priority_high )
        {
            priority_high = vector[i];
            index         = i;
        }
    }
    return index;
}

rp_hook_status_t rp_get_pkt_payload( radio_planner_t* rp, const rp_task_t* task )
{
    rp_hook_status_t status = RP_HOOK_STATUS_OK;
    uint8_t          id     = task->hook_id;

    ral_get_pkt_payload( rp->ral, rp->payload[id], rp->payload_size[id], &rp->payload_size[id] );

    if( task->type == RP_TASK_TYPE_RX_LORA )
    {
        rp->radio_params[id].pkt_type = RAL_PKT_TYPE_LORA;
        status                        = RP_HOOK_STATUS_OK;

        ral_get_lora_pkt_status( rp->ral, &rp->radio_params[id].rx.lora_pkt_status );
    }
    else if( task->type == RP_TASK_TYPE_RX_FSK )
    {
        rp->radio_params[id].pkt_type = RAL_PKT_TYPE_GFSK;
        status                        = RP_HOOK_STATUS_OK;

        ral_get_gfsk_pkt_status( rp->ral, &rp->radio_params[id].rx.gfsk_pkt_status );
    }
    else
    {
        status = RP_HOOK_STATUS_ID_ERROR;
    }

    return status;
}

static void rp_set_alarm( radio_planner_t* rp, const uint32_t alarm_in_ms )
{
    rp_bsp_timer_stop( );
    // test could be managed inside the bsp
    rp_bsp_timer_start( rp, ( alarm_in_ms > 0xFFFF ) ? 0xFFFF : alarm_in_ms, rp_timer_irq_callback );
}

static void rp_timer_irq( radio_planner_t* rp )
{
    rp->timer_state = RP_TIMER_STATE_IDLE;
    rp_task_arbiter( rp, __func__ );
}

static void rp_radio_irq( radio_planner_t* rp )
{
    if( rp->semaphore_abort_radio == 1 )
    {
        rp->semaphore_abort_radio = 0;
        BSP_DBG_TRACE_PRINTF_RP( " RP: INFO - semaphore_abort_radio clear\n" );
    }
    else
    {
        rp->semaphore_radio = 1;

        uint32_t now = rp_bsp_timestamp_get( );

        rp->irq_timestamp_ms[rp->radio_task_id] = now;
        BSP_DBG_TRACE_PRINTF_RP( " RP: INFO - Radio IRQ received for hook #%u\n", rp->radio_task_id );

        rp_irq_get_status( rp, rp->radio_task_id );

        rp_consumption_statistics_updated( rp, rp->radio_task_id, now );

        // Have to call rp_task_free before rp_hook_callback because the callback can enqueued a task and so call the
        // arbiter
        rp_task_free( rp, &rp->tasks[rp->radio_task_id] );
        ral_set_sleep( rp->ral );
        rp_hook_callback( rp, rp->radio_task_id );

        rp_task_call_aborted( rp );

        rp->semaphore_radio = 0;

        rp_task_arbiter( rp, __func__ );
    }

    // Shut Down the TCXO
    ral_set_tcxo_off( rp->ral );
}

static void rp_task_call_aborted( radio_planner_t* rp )
{
    for( int32_t i = 0; i < RP_NB_HOOKS; i++ )
    {
        if( rp->tasks[i].state == RP_TASK_STATE_ABORTED )
        {
            BSP_DBG_TRACE_PRINTF_RP( " RP: INFO - Aborted hook #%ld callback\n", i );
            rp->stats.task_hook_aborted_nb[i]++;
            rp_task_free( rp, &rp->tasks[i] );
            rp->status[i] = RP_STATUS_TASK_ABORTED;
            rp_hook_callback( rp, i );
        }
    }
}

//
// Radio planner callbacks
//

void rp_radio_irq_callback( void* obj )
{
    rp_radio_irq( ( radio_planner_t* ) obj );
}

static void rp_timer_irq_callback( void* obj )
{
    rp_timer_irq( ( radio_planner_t* ) obj );
}

static void rp_hook_callback( radio_planner_t* rp, uint8_t id )
{
    rp->hook_callbacks[id]( rp->hooks[id] );
}

//
// Private debug utilities implementation
//

static void rp_task_print( const radio_planner_t* rp, const rp_task_t* task )
{
    BSP_DBG_TRACE_PRINTF_RP(
        "\nRP- INFO - Radio task #%u  running - Timer task #%u running  - "
        "Hook ID #%u -",
        rp->radio_task_id, rp->timer_task_id, task->hook_id );
    switch( task->type )
    {
    case RP_TASK_TYPE_RX_LORA:
        BSP_DBG_TRACE_PRINTF_RP( " TASK_RX_LORA " );
        break;
    case RP_TASK_TYPE_RX_FSK:
        BSP_DBG_TRACE_PRINTF_RP( " TASK_RX_FSK " );
        break;
    case RP_TASK_TYPE_TX_LORA:
        BSP_DBG_TRACE_PRINTF_RP( " TASK_TX_LORA " );
        break;
    case RP_TASK_TYPE_TX_FSK:
        BSP_DBG_TRACE_PRINTF_RP( " TASK_TX_FSK " );
        break;
    case RP_TASK_TYPE_CAD:
        BSP_DBG_TRACE_PRINTF_RP( " TASK_CAD " );
        break;
    case RP_TASK_TYPE_NONE:
        BSP_DBG_TRACE_PRINTF_RP( " TASK_EMPTY " );
        break;
    default:
        BSP_DBG_TRACE_PRINTF_RP( " TASK_ERROR " );
        break;
    };
    BSP_DBG_TRACE_PRINTF_RP( " - start time @%lu - priority #%u\n", task->start_time_ms, task->priority );
}

static void rp_consumption_statistics_updated( radio_planner_t* rp, const uint8_t hook_id, const uint32_t time )
{
    uint32_t micro_ampere = 0;

    if( rp->tasks[hook_id].type == RP_TASK_TYPE_RX_LORA )
    {
        ral_get_lora_rx_consumption_in_ua( rp->ral, &rp->radio_params[hook_id].rx.lora, &micro_ampere );
    }
    else if( rp->tasks[hook_id].type == RP_TASK_TYPE_TX_LORA )
    {
        ral_get_lora_tx_consumption_in_ua( rp->ral, &rp->radio_params[hook_id].tx.lora, &micro_ampere );
    }

    rp_stats_update( &rp->stats, time, hook_id, micro_ampere );
}
