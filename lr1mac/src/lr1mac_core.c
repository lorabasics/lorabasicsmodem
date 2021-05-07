/*!
 * \file      lr1_mac_core.c
 *
 * \brief     LoRaWan core definition
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

#include "lr1mac_core.h"
#include "lr1mac_utilities.h"
#include "lr1mac_defs_str.h"
#include "smtc_modem_hal.h"
#include "smtc_real.h"
#include "smtc_real_defs.h"
#include "smtc_real_defs_str.h"
#include "smtc_duty_cycle.h"
#include "smtc_lbt.h"
#include "lr1mac_config.h"

#define DBG_PRINT_WITH_LINE( ... )                                                     \
    do                                                                                 \
    {                                                                                  \
        SMTC_MODEM_HAL_TRACE_MSG( "\n  *************************************\n  * " ); \
        SMTC_MODEM_HAL_TRACE_PRINTF( __VA_ARGS__ );                                    \
        SMTC_MODEM_HAL_TRACE_MSG( "\n  *************************************\n" );     \
    } while( 0 );
/*
 *-----------------------------------------------------------------------------------
 *--- PRIVATE VARIABLES -------------------------------------------------------------
 */
/*
 *-----------------------------------------------------------------------------------
 *--- PRIVATE FUNCTIONS DECLARATION -------------------------------------------------
 */
static uint32_t    failsafe_timstamp_get( lr1_stack_mac_t* lr1_mac_obj );
static rp_status_t rp_status_get( lr1_stack_mac_t* lr1_mac_obj );
static void        copy_user_payload( lr1_stack_mac_t* lr1_mac_obj, const uint8_t* data_in, const uint8_t size_in );
static void        save_devnonce_rst( const lr1_stack_mac_t* lr1_mac_obj );
static void        load_devnonce_reset( lr1_stack_mac_t* lr1_mac_obj );
static void        try_recover_nvm( lr1_stack_mac_t* lr1_mac_obj );
/*
 *-----------------------------------------------------------------------------------
 *--- PUBLIC FUNCTIONS DEFINITIONS --------------------------------------------------
 */

void lr1mac_core_init( lr1_stack_mac_t* lr1_mac_obj, smtc_lbt_t* lbt_obj, smtc_dtc_t* dtc_obj, radio_planner_t* rp,
                       lorawan_keys_t* lorawan_keys, smtc_real_region_types_t smtc_real_region_types,
                       void ( *push_callback )( void* push_context ), void*   push_context )
{
    memset( lr1_mac_obj, 0, sizeof( lr1_stack_mac_t ) );

    lr1_mac_obj->lr1mac_state        = LWPSTATE_IDLE;
    lr1_mac_obj->valid_rx_packet     = NO_MORE_VALID_RX_PACKET;
    lr1_mac_obj->receive_window_type = RECEIVE_NONE;
    lr1_mac_obj->stack_id4rp         = 1;
    lr1_mac_obj->lbt_obj             = lbt_obj;
    lr1_mac_obj->dtc_obj             = dtc_obj;
    lr1_mac_obj->send_at_time        = false;
    lr1_mac_obj->push_callback       = push_callback;
    lr1_mac_obj->push_context        = push_context;
    lr1_mac_obj->crystal_error       = BSP_CRYSTAL_ERROR;
    lr1_stack_mac_init( lr1_mac_obj, lorawan_keys, smtc_real_region_types );

    status_lorawan_t status = lr1mac_core_context_load( lr1_mac_obj );

    if( status != OKLORAWAN )
    {
        // Possible update of nvm organisation --> try to recover
        try_recover_nvm( lr1_mac_obj );
        status = lr1mac_core_context_load( lr1_mac_obj );
    }

    if( status == OKLORAWAN )
    {
        // Check if the region stored in flash is still valid
        status = lr1mac_core_is_supported_region( lr1_mac_obj, lr1_mac_obj->real.region_type );
    }

    if( status == ERRORLORAWAN )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "No valid lr1mac context --> Use default value\n" );

        memcpy1( lr1_mac_obj->app_skey, lorawan_keys->LoRaMacAppSKey, 16 );
        memcpy1( lr1_mac_obj->nwk_skey, lorawan_keys->LoRaMacNwkSKey, 16 );
        memcpy1( lr1_mac_obj->app_key, lorawan_keys->LoRaMacAppKey, 16 );
        memcpy1( lr1_mac_obj->dev_eui, lorawan_keys->DevEui, 8 );
        memcpy1( lr1_mac_obj->app_eui, lorawan_keys->AppEui, 8 );
        lr1_mac_obj->adr_custom       = BSP_USER_DR_DISTRIBUTION_PARAMETERS;  // (dr0 only)
        lr1_mac_obj->real.region_type = ( smtc_real_region_types_t ) smtc_real_region_list[0];
        lr1mac_core_context_save( lr1_mac_obj );
    }

    load_devnonce_reset( lr1_mac_obj );
    lr1_mac_obj->nb_of_reset += 1;  // increment reset counter when lr1mac_core_init is called, reset is saved when
                                    // devnonce is save (after a tx join)
    SMTC_MODEM_HAL_TRACE_PRINTF( " DevNonce = %d \n", lr1_mac_obj->dev_nonce );
    SMTC_MODEM_HAL_TRACE_PRINTF( " NbOfReset = %d \n", lr1_mac_obj->nb_of_reset );
    SMTC_MODEM_HAL_TRACE_PRINTF( " Region = %s\n", smtc_real_region_list_str[lr1_mac_obj->real.region_type] );

    lr1_mac_obj->rp                 = rp;
    lr1_mac_obj->no_rx_packet_count = NO_RX_PACKET_CNT;
    rp_hook_init( lr1_mac_obj->rp, lr1_mac_obj->stack_id4rp, ( void ( * )( void* ) )( lr1_stack_mac_rp_callback ),
                  ( lr1_mac_obj ) );
    SMTC_MODEM_HAL_TRACE_PRINTF( "rp_hook_init done \n" );

    smtc_real_config( lr1_mac_obj );
    SMTC_MODEM_HAL_TRACE_PRINTF( "smtc_real_config done \n" );
    smtc_real_init( lr1_mac_obj );
    SMTC_MODEM_HAL_TRACE_PRINTF( "smtc_real_init done \n" );
}

/***********************************************************************************************/
/*    LoraWanProcess Method                                                                    */
/***********************************************************************************************/

lr1mac_states_t lr1mac_core_process( lr1_stack_mac_t* lr1_mac_obj, user_rx_packet_type_t* available_rx_packet )
{
    uint8_t myhook_id;
    rp_hook_get_id( lr1_mac_obj->rp, ( void* ) ( ( lr1_mac_obj ) ), &myhook_id );
    *available_rx_packet = NO_LORA_RXPACKET_AVAILABLE;

#if !defined( TEST_BYPASS_JOIN_DUTY_CYCLE )
    if( lr1mac_core_certification_get( lr1_mac_obj ) == false )
    {
        if( ( lr1_mac_joined_status_get( lr1_mac_obj ) == NOT_JOINED ) &&
            ( ( int32_t )( lr1_mac_obj->next_time_to_join_seconds - smtc_modem_hal_get_time_in_s( ) ) > 0 ) )
        {
            SMTC_MODEM_HAL_TRACE_PRINTF( "TOO SOON TO JOIN time is  %d time target is : %d \n",
                                         smtc_modem_hal_get_time_in_s( ), lr1_mac_obj->next_time_to_join_seconds );
            lr1_mac_obj->lr1mac_state = LWPSTATE_IDLE;
        }
    }
#endif

    if( ( lr1_mac_obj->lr1mac_state != LWPSTATE_IDLE ) &&
        ( ( smtc_modem_hal_get_time_in_s( ) - failsafe_timstamp_get( lr1_mac_obj ) ) > 120 ) )
    {
        smtc_modem_hal_lr1mac_panic( "FAILSAFE EVENT OCCUR (lr1mac_state:0x%x)\n", lr1_mac_obj->lr1mac_state );
        lr1_mac_obj->lr1mac_state = LWPSTATE_ERROR;
    }
    if( lr1_stack_mac_radio_state_get( lr1_mac_obj ) == RADIOSTATE_ABORTED_BY_RP )
    {
        lr1_mac_obj->lr1mac_state = LWPSTATE_UPDATE_MAC;
    }

    switch( lr1_mac_obj->lr1mac_state )
    {
        /************************************************************************************/
        /*                                    STATE IDLE                                    */
        /************************************************************************************/
    case LWPSTATE_IDLE:
        break;

        /************************************************************************************/
        /*                                    STATE TX                                      */
        /************************************************************************************/
    case LWPSTATE_SEND:
        switch( lr1_stack_mac_radio_state_get( lr1_mac_obj ) )
        {
        case RADIOSTATE_IDLE:
            DBG_PRINT_WITH_LINE( "Send Payload  HOOK ID = %d", myhook_id );
            if( lr1_mac_obj->tx_modulation_type == LORA )
            {
                SMTC_MODEM_HAL_TRACE_PRINTF(
                    "  Tx  LoRa at %u ms: freq:%d, SF%u, %s, len %u bytes %d dBm, fcnt_up %d, toa = %d\n",
                    lr1_mac_obj->rtc_target_timer_ms, lr1_mac_obj->tx_frequency, lr1_mac_obj->tx_sf,
                    smtc_name_bw[lr1_mac_obj->tx_bw], lr1_mac_obj->tx_payload_size, lr1_mac_obj->tx_power,
                    lr1_mac_obj->fcnt_up, lr1_stack_toa_get( lr1_mac_obj ) );
            }
            else if( lr1_mac_obj->tx_modulation_type == FSK )
            {
                SMTC_MODEM_HAL_TRACE_PRINTF(
                    "  Tx  FSK  at %u ms: freq:%d, len %u bytes %d dBm, fcnt_up %d, toa = %d\n",
                    lr1_mac_obj->rtc_target_timer_ms, lr1_mac_obj->tx_frequency, lr1_mac_obj->tx_payload_size,
                    lr1_mac_obj->tx_power, lr1_mac_obj->fcnt_up, lr1_stack_toa_get( lr1_mac_obj ) );
            }
            lr1_mac_obj->lr1mac_tx_status = TX_BEGIN;
            if( smtc_lbt_is_enable( lr1_mac_obj->lbt_obj ) == true )
            {
                smtc_lbt_listen_channel( ( lr1_mac_obj->lbt_obj ), lr1_mac_obj->tx_frequency, lr1_mac_obj->send_at_time,
                                         lr1_mac_obj->rtc_target_timer_ms - RP_MARGIN_DELAY,
                                         lr1_stack_toa_get( lr1_mac_obj ) );
                break;
            }
            else
            {
                lr1_stack_mac_tx_radio_start( lr1_mac_obj );
                break;
            }
        case RADIOSTATE_TXFINISHED:
            lr1_mac_obj->lr1mac_state = LWPSTATE_RX1;

            lr1_mac_obj->tx_duty_cycle_timestamp_ms = smtc_modem_hal_get_time_in_ms( );
            lr1_mac_obj->tx_duty_cycle_time_off_ms =
                ( lr1_mac_obj->rp->stats.tx_last_toa_ms[myhook_id] << lr1_mac_obj->max_duty_cycle_index ) -
                lr1_mac_obj->rp->stats.tx_last_toa_ms[myhook_id];

            if( lr1_mac_obj->join_status == NOT_JOINED )
            {
                // save devnonce after the end of TX
                save_devnonce_rst( lr1_mac_obj );
            }
            lr1_stack_mac_rx_timer_configure( lr1_mac_obj, RX1 );
            lr1_stack_mac_update_tx_done( lr1_mac_obj );
            smtc_duty_cycle_sum( lr1_mac_obj->dtc_obj, lr1_mac_obj->tx_frequency,
                                 lr1_mac_obj->rp->stats.tx_last_toa_ms[myhook_id] );
            break;

        default:
            break;
        }
        break;

        /************************************************************************************/
        /*                                   STATE RX1                                      */
        /************************************************************************************/
    case LWPSTATE_RX1:
        if( lr1_stack_mac_radio_state_get( lr1_mac_obj ) == RADIOSTATE_RX1FINISHED )
        {
            if( rp_status_get( lr1_mac_obj ) == RP_STATUS_RX_PACKET )
            {
                lr1_mac_obj->receive_window_type = RECEIVE_ON_RX1;
                // deprecated ( lr1_mac_obj->receive_window_type == RECEIVE_NACK ) ? RECEIVE_ACK_ON_RX1 :
                // RECEIVE_ON_RX1;
                DBG_PRINT_WITH_LINE( "Process Downlink on Rx1 for Hook Id = %d", myhook_id );
                lr1_mac_obj->valid_rx_packet = lr1_stack_mac_rx_frame_decode( lr1_mac_obj );
                if( lr1_mac_obj->valid_rx_packet == NO_MORE_VALID_RX_PACKET )
                {
                    lr1_mac_obj->lr1mac_state = LWPSTATE_RX2;
                    DBG_PRINT_WITH_LINE( "Receive a bad packet on RX1 for Hook Id = %d continue with RX2 ", myhook_id );
                    lr1_stack_mac_rx_timer_configure( lr1_mac_obj, RX2 );
                }
                else
                {
                    DBG_PRINT_WITH_LINE( "Receive a Valid downlink RX1 for Hook Id = %d", myhook_id );
                    lr1_mac_obj->lr1mac_state = LWPSTATE_UPDATE_MAC;
                }
            }
            else
            {
                lr1_mac_obj->lr1mac_state = LWPSTATE_RX2;
                DBG_PRINT_WITH_LINE( "RX1 Timeout for Hook Id = %d", myhook_id );
                lr1_stack_mac_rx_timer_configure( lr1_mac_obj, RX2 );
            }
        }
        break;

        /************************************************************************************/
        /*                                   STATE RX2                                      */
        /************************************************************************************/
    case LWPSTATE_RX2:
        if( lr1_stack_mac_radio_state_get( lr1_mac_obj ) == RADIOSTATE_IDLE )
        {
            if( rp_status_get( lr1_mac_obj ) == RP_STATUS_RX_PACKET )
            {
                lr1_mac_obj->receive_window_type = RECEIVE_ON_RX2;
                // deprecated ( lr1_mac_obj->receive_window_type == RECEIVE_NACK ) ? RECEIVE_ACK_ON_RX2 :
                // RECEIVE_ON_RX2;
                lr1_mac_obj->valid_rx_packet = lr1_stack_mac_rx_frame_decode( lr1_mac_obj );
                DBG_PRINT_WITH_LINE( "Receive a downlink RX2 for Hook Id = %d", myhook_id );
            }
            else
            {
                DBG_PRINT_WITH_LINE( "RX2 Timeout for Hook Id = %d", myhook_id );
            }
            lr1_mac_obj->lr1mac_state = LWPSTATE_UPDATE_MAC;
        }
        else
        {
            break;
        }

        /************************************************************************************/
        /*                              STATE UPDATE MAC                                    */
        /************************************************************************************/
    case LWPSTATE_UPDATE_MAC:
        lr1_mac_obj->radio_process_state = RADIOSTATE_IDLE;
        DBG_PRINT_WITH_LINE( "Update Mac for Hook Id = %d", myhook_id );

        if( lr1_mac_obj->valid_rx_packet == JOIN_ACCEPT_PACKET )
        {
            SMTC_MODEM_HAL_TRACE_MSG( " update join procedure \n" );
            if( lr1_stack_mac_join_accept( lr1_mac_obj ) == OKLORAWAN )
            {
                //@note because datarate Distribution has been changed during join
                smtc_real_set_dr_distribution( lr1_mac_obj, lr1_mac_obj->adr_mode_select_tmp );
            }
            else
            {
                SMTC_MODEM_HAL_TRACE_WARNING( "Not Joined\n" );
            }
        }
        if( ( lr1_mac_obj->valid_rx_packet == NWKRXPACKET ) || ( lr1_mac_obj->valid_rx_packet == USERRX_FOPTSPACKET ) )
        {
            lr1_stack_mac_cmd_parse( lr1_mac_obj );
        }
        lr1_stack_mac_update( lr1_mac_obj );
        *available_rx_packet = lr1_mac_obj->available_app_packet;

        if( lr1_mac_obj->available_app_packet == LORA_RX_PACKET_AVAILABLE )
        {
            lr1_mac_obj->push_callback( lr1_mac_obj->push_context );
        }

        if( ( lr1_mac_obj->type_of_ans_to_send == NWKFRAME_TOSEND ) ||
            ( lr1_mac_obj->type_of_ans_to_send == USRFRAME_TORETRANSMIT ) )
        {  // @note ack send during the next tx|| ( packet.IsFrameToSend == USERACK_TOSEND ) ) {
            // Decrement duty cycle before check the available DTC
            smtc_duty_cycle_update( lr1_mac_obj->dtc_obj );
            if( smtc_real_get_next_channel( lr1_mac_obj ) != OKLORAWAN )
            {
                lr1_mac_obj->lr1mac_state = LWPSTATE_IDLE;
                if( lr1_mac_obj->type_of_ans_to_send == USRFRAME_TORETRANSMIT )
                {
                    lr1_mac_obj->fcnt_up++;
                }
            }
            else
            {
                lr1_mac_obj->type_of_ans_to_send = NOFRAME_TOSEND;
                lr1_mac_obj->rtc_target_timer_ms =
                    smtc_modem_hal_get_time_in_ms( ) + ( smtc_modem_hal_get_random_nb_in_range( 1, 3 ) * 1000 );
                lr1_mac_obj->lr1mac_state = LWPSTATE_TX_WAIT;
            }
        }
        else
        {
            lr1_mac_obj->lr1mac_state = LWPSTATE_IDLE;
        }
        lr1_mac_obj->valid_rx_packet = NO_MORE_VALID_RX_PACKET;
        break;

        /************************************************************************************/
        /*                              STATE TXwait MAC                                    */
        /************************************************************************************/
    case LWPSTATE_TX_WAIT:
        SMTC_MODEM_HAL_TRACE_MSG( " ." );
        if( smtc_modem_hal_get_time_in_ms( ) > lr1_mac_obj->rtc_target_timer_ms )
        {
            lr1_mac_obj->lr1mac_state = LWPSTATE_SEND;  //@note the frame have already been prepare in Update Mac Layer
        }
        break;

    default:
        smtc_modem_hal_lr1mac_panic( "Illegal state in lorawan process\n" );
        break;
    }

    return ( lr1_mac_obj->lr1mac_state );
}

/***********************************************************************************************/
/*    End Of LoraWanProcess Method                                                             */
/***********************************************************************************************/

/**************************************************/
/*            LoraWan  Join  Method               */
/**************************************************/

lr1mac_states_t lr1mac_core_join( lr1_stack_mac_t* lr1_mac_obj, uint32_t target_time_ms )
{
    if( lr1_mac_obj->lr1mac_state != LWPSTATE_IDLE )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "LP STATE NOT EQUAL TO IDLE \n" );
        return ( LWPSTATE_ERROR );
    }
    if( lr1mac_core_is_otaa_device( lr1_mac_obj ) == ABP_DEVICE )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "ABP DEVICE CAN'T PROCCED A JOIN REQUEST \n" );
        return ( LWPSTATE_ERROR );
    }
    uint32_t current_timestamp       = smtc_modem_hal_get_time_in_s( );
    lr1_mac_obj->timestamp_failsafe  = current_timestamp;
    lr1_mac_obj->rtc_target_timer_ms = target_time_ms;
    lr1_mac_obj->join_status         = NOT_JOINED;
    smtc_real_init( lr1_mac_obj );
    lr1_mac_obj->rx2_data_rate = smtc_real_get_rx2_join_dr( lr1_mac_obj );
    if( lr1_mac_obj->adr_mode_select != JOIN_DR_DISTRIBUTION )
    {
        lr1_mac_obj->adr_mode_select_tmp = lr1_mac_obj->adr_mode_select;
        smtc_real_set_dr_distribution( lr1_mac_obj, JOIN_DR_DISTRIBUTION );
        lr1_mac_obj->adr_mode_select = JOIN_DR_DISTRIBUTION;
    }
    smtc_real_get_next_dr( lr1_mac_obj );
    smtc_duty_cycle_update( lr1_mac_obj->dtc_obj );
    if( smtc_real_get_join_next_channel( lr1_mac_obj ) != OKLORAWAN )
    {
        return ( LWPSTATE_ERROR );
    }

    lr1_stack_mac_join_request_build( lr1_mac_obj );
    lr1_stack_rx1_join_delay_set( lr1_mac_obj );
    lr1_stack_rx2_join_dr_set( lr1_mac_obj );

    // check if it first join try
    if( lr1_mac_obj->retry_join_cpt == 0 )
    {
        // take the timestamp reference for join duty cycle management
        lr1_mac_obj->first_join_timestamp = current_timestamp;
    }

    lr1_mac_obj->lr1mac_state = LWPSTATE_SEND;
    return ( lr1_mac_obj->lr1mac_state );
}

/**************************************************/
/*          LoraWan  IsJoined  Method             */
/**************************************************/
join_status_t lr1_mac_joined_status_get( lr1_stack_mac_t* lr1_mac_obj )
{
    join_status_t status = NOT_JOINED;
    status               = lr1_mac_obj->join_status;
    return ( status );
}
/**************************************************/
/*          LoraWan  ClearJoinStatus  Method      */
/**************************************************/

void lr1mac_core_join_status_clear( lr1_stack_mac_t* lr1_mac_obj )
{
    lr1_mac_obj->join_status = NOT_JOINED;
    lr1mac_core_abort( lr1_mac_obj );
    smtc_real_init_join_snapshot_channel_mask( lr1_mac_obj );
}

/**************************************************/
/*          LoraWan  IsJoined  Method             */
/**************************************************/

void lr1mac_core_new_join( lr1_stack_mac_t* lr1_mac_obj )
{
    lr1_mac_obj->join_status = NOT_JOINED;
}
/**************************************************/
/*         LoraWan  SendPayload  Method           */
/**************************************************/

lr1mac_states_t lr1mac_core_payload_send_at_time( lr1_stack_mac_t* lr1_mac_obj, uint8_t fport, const uint8_t* data_in,
                                                  const uint8_t size_in, uint8_t packet_type, uint32_t target_time_ms )
{
    lr1mac_states_t status;
    status = lr1mac_core_payload_send( lr1_mac_obj, fport, data_in, size_in, packet_type, target_time_ms );
    if( status == LWPSTATE_SEND )
    {
        lr1_mac_obj->send_at_time = true;
        lr1_mac_obj->nb_trans_cpt = 1;  // Overwrite nb_trans_cpt, when downlink is At Time, repetitions are out dated
    }
    return status;
}

lr1mac_states_t lr1mac_core_payload_send( lr1_stack_mac_t* lr1_mac_obj, uint8_t fport, const uint8_t* data_in,
                                          const uint8_t size_in, uint8_t packet_type, uint32_t target_time_ms )
{
    status_lorawan_t status;
    status = smtc_real_is_payload_size_valid( lr1_mac_obj, lr1_mac_obj->tx_data_rate, size_in,
                                              lr1_mac_obj->uplink_dwell_time );
    if( status == ERRORLORAWAN )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "PAYLOAD SIZE TOO HIGH \n" );
        return ( LWPSTATE_INVALID );
    }
    if( lr1mac_core_is_otaa_device( lr1_mac_obj ) == OTAA_DEVICE )
    {
        if( lr1_mac_obj->join_status == NOT_JOINED )
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "OTAA DEVICE NOT JOINED YET\n" );
            return ( LWPSTATE_INVALID );
        }
    }
    if( lr1_mac_obj->lr1mac_state != LWPSTATE_IDLE )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "LP STATE NOT EQUAL TO IDLE \n" );
        return ( LWPSTATE_ERROR );
    }
    // Decrement duty cycle before check the available DTC
    smtc_duty_cycle_update( lr1_mac_obj->dtc_obj );
    if( smtc_real_get_next_channel( lr1_mac_obj ) != OKLORAWAN )
    {
        return ( LWPSTATE_DUTY_CYCLE_FULL );
    }

    if( lr1mac_core_next_free_duty_cycle_ms_get( lr1_mac_obj ) > 0 )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "Duty Cycle is full\n" );
        return ( LWPSTATE_DUTY_CYCLE_FULL );
    }

    lr1_mac_obj->timestamp_failsafe  = smtc_modem_hal_get_time_in_s( );
    lr1_mac_obj->rtc_target_timer_ms = target_time_ms;
    copy_user_payload( lr1_mac_obj, data_in, size_in );
    lr1_mac_obj->app_payload_size = size_in;
    lr1_mac_obj->tx_fport         = fport;
    lr1_mac_obj->tx_mtype         = packet_type;
    lr1_stack_mac_tx_frame_build( lr1_mac_obj );
    lr1_stack_mac_tx_frame_encrypt( lr1_mac_obj );
    if( packet_type == CONF_DATA_UP )
    {
        lr1_mac_obj->nb_trans_cpt        = MAX_CONFUP_MSG;
        lr1_mac_obj->receive_window_type = RECEIVE_NONE;
    }
    else
    {
        lr1_mac_obj->nb_trans_cpt = lr1_mac_obj->nb_trans;
        // deprecated lr1_mac_obj->receive_window_type = RECEIVE_NACK;
    }
    lr1_mac_obj->lr1mac_state = LWPSTATE_SEND;
    return ( lr1_mac_obj->lr1mac_state );
}

/**************************************************/
/*        LoraWan  Receive  Method                */
/**************************************************/
status_lorawan_t lr1mac_core_payload_receive( lr1_stack_mac_t* lr1_mac_obj, uint8_t* user_rx_port,
                                              uint8_t* user_rx_payload, uint8_t* user_rx_payloadSize )
{
    status_lorawan_t status = OKLORAWAN;
    if( lr1_mac_obj->available_app_packet == NO_LORA_RXPACKET_AVAILABLE )
    {
        status = ERRORLORAWAN;
    }
    else
    {
        *user_rx_payloadSize = lr1_mac_obj->rx_payload_size;
        *user_rx_port        = lr1_mac_obj->rx_metadata.rx_fport;
        memcpy1( user_rx_payload, lr1_mac_obj->rx_payload, lr1_mac_obj->rx_payload_size );
        lr1_mac_obj->available_app_packet = NO_LORA_RXPACKET_AVAILABLE;
    }
    return ( status );
}

/**************************************************/
/*       LoraWan  AdrModeSelect  Method           */
/**************************************************/

status_lorawan_t lr1mac_core_dr_strategy_set( lr1_stack_mac_t* lr1_mac_obj, dr_strategy_t adr_mode_select )
{
    status_lorawan_t status;
    dr_strategy_t    adr_mode_select_cpy = lr1_mac_obj->adr_mode_select;
    lr1_mac_obj->adr_mode_select         = adr_mode_select;

    if( adr_mode_select == STATIC_ADR_MODE )
    {
        lr1_mac_obj->tx_data_rate_adr = smtc_real_get_min_tx_channel_dr( lr1_mac_obj );
    }
    else
    {
        lr1_mac_obj->tx_power = smtc_real_get_default_max_eirp( lr1_mac_obj );
    }

    smtc_real_set_dr_distribution( lr1_mac_obj, adr_mode_select );
    status = smtc_real_get_next_dr( lr1_mac_obj );

    if( status == ERRORLORAWAN )  // new adr profile no compatible with channel mask, retreive old adr profile
    {
        lr1_mac_obj->adr_mode_select = adr_mode_select_cpy;
        smtc_real_set_dr_distribution( lr1_mac_obj, adr_mode_select_cpy );
        smtc_real_get_next_dr( lr1_mac_obj );
    }
    return ( status );
}
/**************************************************/
/*       LoraWan  AdrModeSelect  Get Method       */
/**************************************************/

dr_strategy_t lr1mac_core_dr_strategy_get( lr1_stack_mac_t* lr1_mac_obj )
{
    return ( lr1_mac_obj->adr_mode_select );
}
/*************************************************************/
/*       LoraWan  Set DataRate Custom for custom adr profile */
/*************************************************************/

void lr1mac_core_dr_custom_set( lr1_stack_mac_t* lr1_mac_obj, uint32_t DataRateCustom )
{
    lr1_mac_obj->adr_custom = DataRateCustom;
    lr1mac_core_context_save( lr1_mac_obj );
}
/**************************************************/
/*         LoraWan  GetDevAddr  Method            */
/**************************************************/

uint32_t lr1mac_core_devaddr_get( lr1_stack_mac_t* lr1_mac_obj )
{
    return ( lr1_mac_obj->dev_addr );
}

/**************************************************/
/*         LoraWan  GetDevEUI  Method             */
/**************************************************/

void lr1mac_core_deveui_get( lr1_stack_mac_t* lr1_mac_obj, uint8_t* DevEui )
{
    memcpy1( DevEui, lr1_mac_obj->dev_eui, 8 );
}

/**************************************************/
/*         LoraWan  SetDevEUI  Method             */
/**************************************************/

void lr1mac_core_deveui_set( lr1_stack_mac_t* lr1_mac_obj, const uint8_t* DevEui )
{
    memcpy1( lr1_mac_obj->dev_eui, DevEui, 8 );
}

/**************************************************/
/*         LoraWan  SetAppKey  Method             */
/**************************************************/

void lr1mac_core_app_key_set( lr1_stack_mac_t* lr1_mac_obj, const uint8_t* AppKey )
{
    memcpy1( lr1_mac_obj->app_key, AppKey, 16 );
}

/**************************************************/
/*         LoraWan  GetAppEui  Method             */
/**************************************************/

void lr1mac_core_appeui_key_get( lr1_stack_mac_t* lr1_mac_obj, uint8_t* AppEui )
{
    memcpy1( AppEui, lr1_mac_obj->app_eui, 8 );
}

/**************************************************/
/*         LoraWan  SetAppEui  Method             */
/**************************************************/

void lr1mac_core_appeui_key_set( lr1_stack_mac_t* lr1_mac_obj, const uint8_t* AppEui )
{
    memcpy1( lr1_mac_obj->app_eui, AppEui, 8 );
}

/**************************************************/
/*         LoraWan  GetNextPower  Method          */
/**************************************************/

uint8_t lr1mac_core_next_power_get( lr1_stack_mac_t* lr1_mac_obj )
{
    return ( lr1_mac_obj->tx_power );
}

/**************************************************/
/*    LoraWan  GetLorawanProcessState  Method     */
/**************************************************/

lr1mac_states_t lr1mac_core_state_get( lr1_stack_mac_t* lr1_mac_obj )
{
    return ( lr1_mac_obj->lr1mac_state );
}

/**************************************************/
/*    LoraWan  StoreContext  Method             */
/**************************************************/

void lr1mac_core_context_save( lr1_stack_mac_t* lr1_mac_obj )
{
    // don't save in nvm if only nb of reset has been modified
    if( ( lr1_mac_obj->mac_context.adr_custom != lr1_mac_obj->adr_custom ) ||
        ( lr1_mac_obj->mac_context.region_type != lr1_mac_obj->real.region_type ) ||
        ( memcmp( lr1_mac_obj->mac_context.appeui, lr1_mac_obj->app_eui, 8 ) != 0 ) ||
        ( memcmp( lr1_mac_obj->mac_context.deveui, lr1_mac_obj->dev_eui, 8 ) != 0 ) ||
        ( memcmp( lr1_mac_obj->mac_context.appkey, lr1_mac_obj->app_key, 16 ) != 0 ) )
    {
        lr1_mac_obj->mac_context.adr_custom  = lr1_mac_obj->adr_custom;
        lr1_mac_obj->mac_context.region_type = lr1_mac_obj->real.region_type;
        memcpy1( lr1_mac_obj->mac_context.appeui, lr1_mac_obj->app_eui, 8 );
        memcpy1( lr1_mac_obj->mac_context.deveui, lr1_mac_obj->dev_eui, 8 );
        memcpy1( lr1_mac_obj->mac_context.appkey, lr1_mac_obj->app_key, 16 );
        lr1_mac_obj->mac_context.crc =
            lr1mac_utilities_crc( ( uint8_t* ) &( lr1_mac_obj->mac_context ), sizeof( lr1_mac_obj->mac_context ) - 4 );

        smtc_modem_hal_context_store( CONTEXT_LR1MAC, ( uint8_t* ) &( lr1_mac_obj->mac_context ),
                                      sizeof( lr1_mac_obj->mac_context ) );

        // smtc_modem_hal_context_store should include a mechanism to secure store context in flash such as double write
        // in flash with a crc protection for example
        smtc_modem_hal_wait_us( 10000 );
    }
}
/**************************************************/
/*    LoraWan  storeContext  Method               */
/**************************************************/

void lr1mac_core_keys_set( lr1_stack_mac_t* lr1_mac_obj, lorawan_keys_t LoRaWanKeys )
{
    memcpy1( lr1_mac_obj->app_skey, LoRaWanKeys.LoRaMacAppSKey, 16 );
    memcpy1( lr1_mac_obj->nwk_skey, LoRaWanKeys.LoRaMacNwkSKey, 16 );
    memcpy1( lr1_mac_obj->app_key, LoRaWanKeys.LoRaMacAppKey, 16 );
    memcpy1( lr1_mac_obj->dev_eui, LoRaWanKeys.DevEui, 8 );
    memcpy1( lr1_mac_obj->app_eui, LoRaWanKeys.AppEui, 8 );
    lr1_mac_obj->otaa_device = LoRaWanKeys.otaaDevice;
    lr1_mac_obj->dev_addr    = LoRaWanKeys.LoRaDevAddr;

    lr1mac_core_context_save( lr1_mac_obj );
}
/**************************************************/
/*   LoraWan  lr1mac_core_next_max_payload_length_get  Method     */
/**************************************************/

uint32_t lr1mac_core_next_max_payload_length_get( lr1_stack_mac_t* lr1_mac_obj )
{
    return ( smtc_real_get_max_payload_size( lr1_mac_obj, lr1_mac_obj->tx_data_rate, lr1_mac_obj->uplink_dwell_time ) -
             lr1_mac_obj->tx_fopts_current_length - 8 );
}

/**************************************************/
/*        LoraWan  lr1mac_core_next_dr_get  Method        */
/**************************************************/

uint8_t lr1mac_core_next_dr_get( lr1_stack_mac_t* lr1_mac_obj )
{  // note return datareate in case of adr
    return ( lr1_mac_obj->tx_data_rate );
}

uint32_t lr1mac_core_next_frequency_get( lr1_stack_mac_t* lr1_mac_obj )
{  // note return datareate in case of adr
    return ( lr1_mac_obj->tx_frequency );
}

void lr1mac_core_factory_reset( lr1_stack_mac_t* lr1_mac_obj )
{
    lr1_mac_obj->mac_context.adr_custom  = lr1_mac_obj->adr_custom;
    lr1_mac_obj->mac_context.region_type = lr1_mac_obj->real.region_type;
    memcpy1( lr1_mac_obj->mac_context.appeui, lr1_mac_obj->app_eui, 8 );
    memcpy1( lr1_mac_obj->mac_context.deveui, lr1_mac_obj->dev_eui, 8 );
    memcpy1( lr1_mac_obj->mac_context.appkey, lr1_mac_obj->app_key, 16 );
    lr1_mac_obj->mac_context.crc =
        lr1mac_utilities_crc( ( uint8_t* ) &( lr1_mac_obj->mac_context ), sizeof( lr1_mac_obj->mac_context ) - 4 ) + 1;

    smtc_modem_hal_context_store( CONTEXT_LR1MAC, ( uint8_t* ) &( lr1_mac_obj->mac_context ),
                                  sizeof( lr1_mac_obj->mac_context ) );

    // smtc_modem_hal_context_store should include a mechanism to secure store context in flash such as double write in
    // flash with a crc protection for example
    smtc_modem_hal_wait_us( 10000 );
}

type_otaa_abp_t lr1mac_core_is_otaa_device( lr1_stack_mac_t* lr1_mac_obj )
{
    return ( type_otaa_abp_t ) lr1_mac_obj->otaa_device;
}

void lr1mac_core_otaa_set( lr1_stack_mac_t* lr1_mac_obj, type_otaa_abp_t deviceType )
{
    lr1_mac_obj->otaa_device = deviceType;
}
radio_planner_t* lr1mac_core_rp_get( lr1_stack_mac_t* lr1_mac_obj )
{
    return lr1_mac_obj->rp;
}

uint16_t lr1mac_core_nb_reset_get( lr1_stack_mac_t* lr1_mac_obj )
{
    return lr1_mac_obj->nb_of_reset;
}

uint16_t lr1mac_core_devnonce_get( lr1_stack_mac_t* lr1_mac_obj )
{
    return lr1_mac_obj->dev_nonce;
}

int16_t lr1mac_core_last_snr_get( lr1_stack_mac_t* lr1_mac_obj )
{
    return lr1_mac_obj->rx_metadata.rx_snr;
}

int16_t lr1mac_core_last_rssi_get( lr1_stack_mac_t* lr1_mac_obj )
{
    return lr1_mac_obj->rx_metadata.rx_rssi;
}

uint8_t lr1mac_core_min_tx_dr_get( lr1_stack_mac_t* lr1_mac_obj )
{
    uint8_t tmp = lr1_stack_mac_min_tx_dr_get( lr1_mac_obj );
    SMTC_MODEM_HAL_TRACE_PRINTF( "Min DataRate = %d\n", tmp );
    return ( tmp );
}
uint8_t lr1mac_core_max_tx_dr_get( lr1_stack_mac_t* lr1_mac_obj )
{
    uint8_t tmp = lr1_stack_mac_max_tx_dr_get( lr1_mac_obj );
    SMTC_MODEM_HAL_TRACE_PRINTF( "Max DataRate = %d\n", tmp );
    return ( tmp );
}

uint16_t lr1mac_core_mask_tx_dr_channel_up_dwell_time_check( lr1_stack_mac_t* lr1_mac_obj )
{
    uint8_t tmp = lr1_stack_mac_mask_tx_dr_channel_up_dwell_time_check( lr1_mac_obj );
    SMTC_MODEM_HAL_TRACE_PRINTF( "Mask DataRate = 0x%x\n", tmp );
    return ( tmp );
}

uint8_t* lr1mac_core_apps_key_get( lr1_stack_mac_t* lr1_mac_obj )
{
    return lr1_mac_obj->app_skey;
}

status_lorawan_t lr1mac_core_context_load( lr1_stack_mac_t* lr1_mac_obj )
{
    smtc_modem_hal_context_restore( CONTEXT_LR1MAC, ( uint8_t* ) &( lr1_mac_obj->mac_context ),
                                    sizeof( lr1_mac_obj->mac_context ) );

    if( lr1mac_utilities_crc( ( uint8_t* ) &( lr1_mac_obj->mac_context ), sizeof( lr1_mac_obj->mac_context ) - 4 ) ==
        lr1_mac_obj->mac_context.crc )
    {
        memcpy1( lr1_mac_obj->app_eui, lr1_mac_obj->mac_context.appeui, 8 );
        memcpy1( lr1_mac_obj->dev_eui, lr1_mac_obj->mac_context.deveui, 8 );
        memcpy1( lr1_mac_obj->app_key, lr1_mac_obj->mac_context.appkey, 16 );
        lr1_mac_obj->adr_custom       = lr1_mac_obj->mac_context.adr_custom;
        lr1_mac_obj->real.region_type = ( smtc_real_region_types_t ) lr1_mac_obj->mac_context.region_type;
        return OKLORAWAN;
    }
    else
    {  // == factory reset
        return ERRORLORAWAN;
    }
}
receive_win_t lr1mac_core_rx_window_get( lr1_stack_mac_t* lr1_mac_obj )
{
    return ( lr1_mac_obj->receive_window_type );
}
uint32_t lr1mac_core_fcnt_up_get( lr1_stack_mac_t* lr1_mac_obj )
{
    return ( lr1_mac_obj->fcnt_up );
}
uint32_t lr1mac_core_next_join_time_second_get( lr1_stack_mac_t* lr1_mac_obj )
{
    return ( lr1_mac_obj->next_time_to_join_seconds );
}
int32_t lr1mac_core_next_free_duty_cycle_ms_get( lr1_stack_mac_t* lr1_mac_obj )
{
    int32_t  ret            = 0;
    uint8_t  number_of_freq = 0;
    uint8_t  max_size       = 16;
    uint32_t freq_list[16]  = { 0 };  // Generally region with duty cycle support 16 channels only

    int32_t region_dtc = 0;
    int32_t nwk_dtc    = lr1_stack_network_next_free_duty_cycle_ms_get( lr1_mac_obj );

    if( smtc_real_is_dtc_supported( lr1_mac_obj ) == true )
    {
        if( smtc_real_get_current_enabled_frequency_list( lr1_mac_obj, &number_of_freq, freq_list, max_size ) == true )
        {
            region_dtc = smtc_duty_cycle_get_next_free_time_ms( lr1_mac_obj->dtc_obj, number_of_freq, freq_list );
        }
    }

    if( nwk_dtc == 0 )
    {
        ret = region_dtc;
    }
    else
    {
        ret = MAX( nwk_dtc, region_dtc );
    }
    return ret;
}
uint8_t lr1mac_core_duty_cycle_enable_set( lr1_stack_mac_t* lr1_mac_obj, bool enable )
{
    return smtc_duty_cycle_enable_set( lr1_mac_obj->dtc_obj, enable );
}

uint32_t lr1mac_core_version_get( lr1_stack_mac_t* lr1_mac_obj )
{
    return ( LR1MAC_PROTOCOL_VERSION );
}

/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
uint8_t lr1mac_core_rx_ack_bit_get( lr1_stack_mac_t* lr1_mac_obj )
{
    return ( lr1_mac_obj->rx_ack_bit );
}

status_lorawan_t lr1mac_core_is_supported_region( lr1_stack_mac_t* lr1_mac_obj, smtc_real_region_types_t region_type )
{
    uint8_t is_valid_region = false;
    for( uint8_t i = 0; i < SMTC_REAL_REGION_LIST_LENGTH; i++ )
    {
        if( smtc_real_region_list[i] == region_type )
        {
            is_valid_region = true;
            break;
        }
    }

    if( is_valid_region == false )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Invalid Region 0x%02x\n", region_type );
        return ERRORLORAWAN;
    }
    return OKLORAWAN;
}

smtc_real_region_types_t lr1mac_core_get_region( lr1_stack_mac_t* lr1_mac_obj )
{
    return lr1_mac_obj->real.region_type;
}

status_lorawan_t lr1mac_core_set_region( lr1_stack_mac_t* lr1_mac_obj, smtc_real_region_types_t region_type )
{
    if( lr1mac_core_is_supported_region( lr1_mac_obj, region_type ) == OKLORAWAN )
    {
        lr1_mac_obj->real.region_type = region_type;
        lr1mac_core_context_save( lr1_mac_obj );
        smtc_real_config( lr1_mac_obj );
        smtc_real_init( lr1_mac_obj );
        return OKLORAWAN;
    }
    return ERRORLORAWAN;
}

status_lorawan_t lr1mac_core_set_no_rx_packet_count_config( lr1_stack_mac_t* lr1_mac_obj, uint16_t no_rx_packet_count )
{
    // 0 means this protection is deactivated
    // if( no_rx_packet_count > 0 )
    //  {
    lr1_mac_obj->no_rx_packet_count = no_rx_packet_count;
    // lr1_mac_obj->adr_ack_cnt                 = 0;  // reset adr counter,
    // lr1_mac_obj->adr_ack_cnt_confirmed_frame = 0;  // reset adr counter
    return OKLORAWAN;
    //  }
    // return ERRORLORAWAN;
}

uint16_t lr1mac_core_get_no_rx_packet_count_config( lr1_stack_mac_t* lr1_mac_obj )
{
    return lr1_mac_obj->no_rx_packet_count;
}

uint16_t lr1mac_core_get_no_rx_packet_count_current( lr1_stack_mac_t* lr1_mac_obj )
{
    return lr1_mac_obj->adr_ack_cnt + lr1_mac_obj->adr_ack_cnt_confirmed_frame;
}

uint16_t lr1mac_core_get_no_rx_packet_count_in_mobile_mode( lr1_stack_mac_t* lr1_mac_obj )
{
    return lr1_mac_obj->no_rx_packet_count_in_mobile_mode;
}

void lr1mac_core_set_no_rx_packet_count_in_mobile_mode( lr1_stack_mac_t* lr1_mac_obj, uint16_t no_rx_packet_count )
{
    lr1_mac_obj->no_rx_packet_count_in_mobile_mode = no_rx_packet_count;
}

bool lr1mac_core_available_link_adr_get( lr1_stack_mac_t* lr1_mac_obj )
{
    if( lr1_mac_obj->available_link_adr == true )
    {
        lr1_mac_obj->available_link_adr = false;  // reset the internal flag when the function is called by upper layer
        return ( true );
    }
    else
    {
        return false;
    }
}

void lr1mac_core_certification_set( lr1_stack_mac_t* lr1_mac_obj, uint8_t enable )
{
    lr1_mac_obj->is_lorawan_modem_certification_enabled = enable;
}

uint8_t lr1mac_core_certification_get( lr1_stack_mac_t* lr1_mac_obj )
{
    return lr1_mac_obj->is_lorawan_modem_certification_enabled;
}

/*
 *-----------------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITIONS ------------------------------------------------
 */

static void copy_user_payload( lr1_stack_mac_t* lr1_mac_obj, const uint8_t* data_in, const uint8_t size_in )
{
    memcpy1( &( lr1_mac_obj->tx_payload[FHDROFFSET + lr1_mac_obj->tx_fopts_current_length] ), data_in, size_in );
}

static uint32_t failsafe_timstamp_get( lr1_stack_mac_t* lr1_mac_obj )
{
    return lr1_mac_obj->timestamp_failsafe;
}

static rp_status_t rp_status_get( lr1_stack_mac_t* lr1_mac_obj )
{
    return lr1_mac_obj->planner_status;
}

void lr1mac_core_abort( lr1_stack_mac_t* lr1_mac_obj )
{
    lr1_mac_obj->valid_rx_packet     = NO_MORE_VALID_RX_PACKET;
    lr1_mac_obj->type_of_ans_to_send = NOFRAME_TOSEND;
    lr1_mac_obj->receive_window_type = RECEIVE_NONE;
    lr1_mac_obj->nb_trans_cpt        = 1;
    rp_task_abort( lr1_mac_obj->rp, lr1_mac_obj->stack_id4rp );
}

static void save_devnonce_rst( const lr1_stack_mac_t* lr1_mac_obj )
{
    lr1_counter_context_t ctx = { 0 };

    ctx.devnonce = lr1_mac_obj->dev_nonce;
    ctx.nb_reset = lr1_mac_obj->nb_of_reset;
    ctx.crc      = lr1mac_utilities_crc( ( uint8_t* ) &ctx, sizeof( ctx ) - 4 );

    smtc_modem_hal_context_store( CONTEXT_DEVNONCE, ( uint8_t* ) &ctx, sizeof( ctx ) );
}

static void load_devnonce_reset( lr1_stack_mac_t* lr1_mac_obj )
{
    lr1_counter_context_t ctx = { 0 };
    smtc_modem_hal_context_restore( CONTEXT_DEVNONCE, ( uint8_t* ) &ctx, sizeof( ctx ) );

    if( lr1mac_utilities_crc( ( uint8_t* ) &ctx, sizeof( ctx ) - 4 ) == ctx.crc )
    {
        lr1_mac_obj->dev_nonce   = ctx.devnonce;
        lr1_mac_obj->nb_of_reset = ctx.nb_reset;
    }
    else
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "No valid DevNonce in NVM, use default (0)\n" );
    }
}

static void try_recover_nvm( lr1_stack_mac_t* lr1_mac_obj )
{
    SMTC_MODEM_HAL_TRACE_PRINTF( "Lr1mac context : Try recover NVM\n" );

    typedef struct mac_context_010007_s
    {
        uint8_t  appeui[8];
        uint8_t  deveui[8];
        uint8_t  appkey[16];
        uint16_t devnonce;
        uint16_t nb_reset;
        uint32_t adr_custom;
        uint8_t  region_type;
        uint32_t crc;
    } mac_context_010007_t;

    mac_context_010007_t old_save_fmt;

    smtc_modem_hal_context_restore( CONTEXT_LR1MAC, ( uint8_t* ) &old_save_fmt, sizeof( old_save_fmt ) );

    if( lr1mac_utilities_crc( ( uint8_t* ) &( old_save_fmt ), sizeof( old_save_fmt ) - 4 ) == old_save_fmt.crc )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "Recover success from 1.0.7 !\n" );

        memcpy1( lr1_mac_obj->app_eui, old_save_fmt.appeui, 8 );
        memcpy1( lr1_mac_obj->dev_eui, old_save_fmt.deveui, 8 );
        memcpy1( lr1_mac_obj->app_key, old_save_fmt.appkey, 16 );
        lr1_mac_obj->dev_nonce        = old_save_fmt.devnonce + 10;  // in 1.0.7 dev nonce was save every 10
        lr1_mac_obj->adr_custom       = old_save_fmt.adr_custom;
        lr1_mac_obj->nb_of_reset      = old_save_fmt.nb_reset;
        lr1_mac_obj->real.region_type = ( smtc_real_region_types_t ) old_save_fmt.region_type;

        save_devnonce_rst( lr1_mac_obj );
        lr1mac_core_context_save( lr1_mac_obj );  // to save new number of reset
    }
    else
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "Fail to recover\n" );
    }
}