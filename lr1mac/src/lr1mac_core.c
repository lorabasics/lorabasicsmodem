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
#include "smtc_bsp.h"
#include "smtc_real.h"
#include "smtc_real_defs.h"
#include "smtc_real_defs_str.h"

#define DBG_PRINT_WITH_LINE( ... )                                              \
    do                                                                          \
    {                                                                           \
        BSP_DBG_TRACE_MSG( "\n  *************************************\n  * " ); \
        BSP_DBG_TRACE_PRINTF( __VA_ARGS__ );                                    \
        BSP_DBG_TRACE_MSG( "\n  *************************************\n" );     \
    } while( 0 );

/*
 *-----------------------------------------------------------------------------------
 *--- PRIVATE VARIABLES -------------------------------------------------------------
 */
static lr1mac_states_t  lr1mac_state        = LWPSTATE_IDLE;
static lr1_stack_mac_t  lr1_mac_obj         = { 0 };
static uint8_t          stack_id4rp         = 0;
static rx_packet_type_t valid_rx_packet     = NO_MORE_VALID_RX_PACKET;
static receive_win_t    receive_window_type = RECEIVE_NONE;

/*
 *-----------------------------------------------------------------------------------
 *--- PRIVATE FUNCTIONS DECLARATION -------------------------------------------------
 */
static uint32_t    failsafe_timstamp_get( void );
static rp_status_t rp_status_get( void );
static void        copy_user_payload( const uint8_t* data_in, const uint8_t size_in );

/*
 *-----------------------------------------------------------------------------------
 *--- PUBLIC FUNCTIONS DEFINITIONS --------------------------------------------------
 */

void lr1mac_core_init( radio_planner_t* rp, lorawan_keys_t* lorawan_keys, smtc_real_t* smtc_region )
{
    lr1_stack_mac_init( &lr1_mac_obj, lorawan_keys, smtc_region );

    status_lorawan_t status = lr1mac_core_context_load( );

    if( status == OKLORAWAN )
    {
        // Check if the region stored in flash is still valid
        status = lr1mac_core_is_supported_region( lr1_mac_obj.real->region_type );
    }

    if( status == ERRORLORAWAN )
    {
        memcpy( lr1_mac_obj.app_skey, lorawan_keys->LoRaMacAppSKey, 16 );
        memcpy( lr1_mac_obj.nwk_skey, lorawan_keys->LoRaMacNwkSKey, 16 );
        memcpy( lr1_mac_obj.app_key, lorawan_keys->LoRaMacAppKey, 16 );
        memcpy( lr1_mac_obj.dev_eui, lorawan_keys->DevEui, 8 );
        memcpy( lr1_mac_obj.app_eui, lorawan_keys->AppEui, 8 );
        lr1_mac_obj.dev_nonce         = 0;
        lr1_mac_obj.adr_custom        = BSP_USER_DR_DISTRIBUTION_PARAMETERS;  // (dr0 only)
        lr1_mac_obj.nb_of_reset       = 0;
        lr1_mac_obj.real->region_type = ( smtc_real_region_types_t ) smtc_real_region_list[0];
        lr1mac_core_context_save( );
    }
    smtc_real_init( &lr1_mac_obj );
    BSP_DBG_TRACE_PRINTF( " Region = %s\n", smtc_real_region_list_str[lr1_mac_obj.real->region_type] );

    lr1_mac_obj.rp = rp;

    rp_hook_init( lr1_mac_obj.rp, stack_id4rp, ( void ( * )( void* ) )( lr1_stack_mac_rp_callback ), &( lr1_mac_obj ) );
}

/***********************************************************************************************/
/*    LoraWanProcess Method                                                                    */
/***********************************************************************************************/

lr1mac_states_t lr1mac_core_process( user_rx_packet_type_t* available_rx_packet )
{
    uint8_t myhook_id;
    rp_hook_get_id( lr1_mac_obj.rp, ( void* ) ( &( lr1_mac_obj ) ), &myhook_id );
    *available_rx_packet = NO_LORA_RXPACKET_AVAILABLE;
#ifndef TEST_BYPASS_JOIN_DUTY_CYCLE
    if( ( lr1_mac_joined_status_get( ) == NOT_JOINED ) &&
        ( ( int32_t )( lr1_mac_obj.next_time_to_join_seconds - bsp_rtc_get_time_s( ) ) > 0 ) )
    {
        BSP_DBG_TRACE_PRINTF( "TOO SOON TO JOIN time is  %lu time target is : %lu \n", bsp_rtc_get_time_s( ),
                              lr1_mac_obj.next_time_to_join_seconds );
        lr1mac_state = LWPSTATE_IDLE;
    }
#endif  // TEST_BYPASS_JOIN_DUTY_CYCLE

    if( ( lr1mac_state != LWPSTATE_IDLE ) && ( ( bsp_rtc_get_time_s( ) - failsafe_timstamp_get( ) ) > 120 ) )
    {
        lr1mac_state = LWPSTATE_ERROR;
        BSP_DBG_TRACE_ERROR( "FAILSAFE EVENT OCCUR \n" );
        bsp_mcu_handle_lr1mac_issue( );
    }
    switch( lr1mac_state )
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
        switch( lr1_stack_mac_radio_state_get( &lr1_mac_obj ) )
        {
        case RADIOSTATE_IDLE:
            receive_window_type = RECEIVE_NONE;
            DBG_PRINT_WITH_LINE( "Send Payload  HOOK ID = %d", myhook_id );
            lr1_stack_mac_tx_radio_start( &lr1_mac_obj );
            break;

        case RADIOSTATE_TXFINISHED:
            lr1mac_state = LWPSTATE_RX1;
            lr1_stack_mac_rx_timer_configure( &lr1_mac_obj, RX1 );
            lr1_mac_obj.tx_duty_cycle_timestamp_ms = bsp_rtc_get_time_ms( );
            lr1_mac_obj.tx_duty_cycle_time_off_ms =
                ( lr1_mac_obj.rp->stats.tx_last_toa_ms[myhook_id] << lr1_mac_obj.max_duty_cycle_index ) -
                lr1_mac_obj.rp->stats.tx_last_toa_ms[myhook_id];
            smtc_real_duty_cycle_sum( &lr1_mac_obj, lr1_mac_obj.tx_frequency,
                                      lr1_mac_obj.rp->stats.tx_last_toa_ms[myhook_id] );
            break;

        default:
            break;
        }
        break;

        /************************************************************************************/
        /*                                   STATE RX1                                      */
        /************************************************************************************/
    case LWPSTATE_RX1:
        if( lr1_stack_mac_radio_state_get( &lr1_mac_obj ) == RADIOSTATE_RX1FINISHED )
        {
            if( rp_status_get( ) == RP_STATUS_RX_PACKET )
            {
                receive_window_type = RECEIVE_ON_RX1;
                lr1mac_state        = LWPSTATE_PROCESS_DOWNLINK;
                DBG_PRINT_WITH_LINE( "Receive a downlink RX1 for Hook Id = %d", myhook_id );
            }
            else
            {
                lr1mac_state = LWPSTATE_RX2;
                DBG_PRINT_WITH_LINE( "RX1 Timeout for Hook Id = %d", myhook_id );
                lr1_stack_mac_rx_timer_configure( &lr1_mac_obj, RX2 );
            }
        }
        break;

        /************************************************************************************/
        /*                                   STATE RX2                                      */
        /************************************************************************************/
    case LWPSTATE_RX2:
        if( lr1_stack_mac_radio_state_get( &lr1_mac_obj ) == RADIOSTATE_IDLE )
        {
            if( rp_status_get( ) == RP_STATUS_RX_PACKET )
            {
                receive_window_type = RECEIVE_ON_RX2;
                lr1mac_state        = LWPSTATE_PROCESS_DOWNLINK;
                DBG_PRINT_WITH_LINE( "Receive a downlink RX2 for Hook Id = %d", myhook_id );
            }
            else
            {
                DBG_PRINT_WITH_LINE( "RX2 Timeout for Hook Id = %d", myhook_id );
                lr1mac_state = LWPSTATE_UPDATE_MAC;
            }
        }
        break;

        /************************************************************************************/
        /*                              STATE PROCESS DOWNLINK                              */
        /* At this step crc is valid                                                        */
        /*    Step 1 : CheckRxPayloadLength                                                 */
        /*    Step 2 : ExtractRxMhdr                                                        */
        /*    Step 3 : ExtractRxFhdr                                                        */
        /*    Step 4 : Check Mic                                                            */
        /*    Step 5 : Decrypt Payload                                                      */
        /*    Step 6 : Extract Fport to select Between App/nwm Payload                      */
        /************************************************************************************/
    case LWPSTATE_PROCESS_DOWNLINK:
        // return NOvalid_rx_packet or  USERRX_FOPTSPACKET or NWKRXPACKET or JOIN_ACCEPT_PACKET.
        DBG_PRINT_WITH_LINE( "Process Downlink for Hook Id = %d", myhook_id );

        valid_rx_packet = lr1_stack_mac_rx_frame_decode( &lr1_mac_obj );
        lr1mac_state    = LWPSTATE_UPDATE_MAC;
        break;

        /************************************************************************************/
        /*                              STATE UPDATE MAC                                    */
        /************************************************************************************/
    case LWPSTATE_UPDATE_MAC:
        lr1_mac_obj.radio_process_state = RADIOSTATE_IDLE;
        DBG_PRINT_WITH_LINE( "Update Mac for Hook Id = %d", myhook_id );

        if( valid_rx_packet == JOIN_ACCEPT_PACKET )
        {
            BSP_DBG_TRACE_MSG( " update join procedure \n" );
            lr1_stack_mac_join_accept( &lr1_mac_obj );

            //@note because datarate Distribution has been changed during join
            smtc_real_dr_distribution_set( &lr1_mac_obj, lr1_mac_obj.adr_mode_select );
        }
        if( ( valid_rx_packet == NWKRXPACKET ) || ( valid_rx_packet == USERRX_FOPTSPACKET ) )
        {
            lr1_stack_mac_cmd_parse( &lr1_mac_obj );
        }
        lr1_stack_mac_update( &lr1_mac_obj );
        *available_rx_packet = lr1_mac_obj.available_app_packet;

        if( ( lr1_mac_obj.type_of_ans_to_send == NWKFRAME_TOSEND ) ||
            ( lr1_mac_obj.type_of_ans_to_send == USRFRAME_TORETRANSMIT ) )
        {  // @note ack send during the next tx|| ( packet.IsFrameToSend == USERACK_TOSEND ) ) {
            lr1_mac_obj.type_of_ans_to_send = NOFRAME_TOSEND;
            lr1_mac_obj.rtc_target_timer_ms = ( bsp_rtc_get_time_s( ) + bsp_rng_get_random_in_range( 1, 3 ) ) * 1000;
            lr1mac_state                    = LWPSTATE_TX_WAIT;
        }
        else
        {
            lr1mac_state = LWPSTATE_IDLE;
        }
        valid_rx_packet = NO_MORE_VALID_RX_PACKET;
        break;

        /************************************************************************************/
        /*                              STATE TXwait MAC                                    */
        /************************************************************************************/
    case LWPSTATE_TX_WAIT:
        BSP_DBG_TRACE_MSG( "." );
        if( bsp_rtc_get_time_ms( ) > lr1_mac_obj.rtc_target_timer_ms )
        {
            lr1mac_state = LWPSTATE_SEND;  //@note the frame have already been prepare in Update Mac Layer
        }
        break;

    default:
        BSP_DBG_TRACE_MSG( " Illegal state in lorawan process\n " );
        bsp_mcu_handle_lr1mac_issue( );
        break;
    }

    return ( lr1mac_state );
}

/***********************************************************************************************/
/*    End Of LoraWanProcess Method                                                             */
/***********************************************************************************************/

/**************************************************/
/*            LoraWan  Join  Method               */
/**************************************************/

lr1mac_states_t lr1mac_core_join( uint32_t target_time_ms )
{
    if( lr1mac_state != LWPSTATE_IDLE )
    {
        BSP_DBG_TRACE_ERROR( "LP STATE NOT EQUAL TO IDLE \n" );
        return ( LWPSTATE_ERROR );
    }
    if( lr1mac_core_is_otaa_device( ) == ABP_DEVICE )
    {
        BSP_DBG_TRACE_ERROR( "ABP DEVICE CAN'T PROCCED A JOIN REQUEST \n" );
        return ( LWPSTATE_ERROR );
    }
    uint32_t current_timestamp      = bsp_rtc_get_time_s( );
    lr1_mac_obj.timestamp_failsafe  = current_timestamp;
    lr1_mac_obj.rtc_target_timer_ms = target_time_ms;
    lr1_mac_obj.join_status         = NOT_JOINED;
    smtc_real_init( &lr1_mac_obj );
    lr1_mac_obj.rx2_data_rate = smtc_real_rx2_join_dr_get( &lr1_mac_obj );
    smtc_real_dr_distribution_set( &lr1_mac_obj, JOIN_DR_DISTRIBUTION );
    smtc_real_next_dr_get( &lr1_mac_obj );
    smtc_real_duty_cycle_update( &lr1_mac_obj );
    if( smtc_real_join_next_channel_get( &lr1_mac_obj ) != OKLORAWAN )
    {
        return ( LWPSTATE_ERROR );
    }

    lr1_stack_mac_join_request_build( &lr1_mac_obj );
    lr1_stack_rx1_join_delay_set( &lr1_mac_obj );
    lr1_stack_rx2_join_dr_set( &lr1_mac_obj );

    // check if it first join try
    if( lr1_mac_obj.retry_join_cpt == 0 )
    {
        // take the timestamp reference for join duty cycle management
        lr1_mac_obj.first_join_timestamp = current_timestamp;
    }

    lr1mac_state = LWPSTATE_SEND;
    return ( lr1mac_state );
}

/**************************************************/
/*          LoraWan  IsJoined  Method             */
/**************************************************/
join_status_t lr1_mac_joined_status_get( void )
{
    join_status_t status = NOT_JOINED;
    status               = lr1_mac_obj.join_status;
    return ( status );
}
/**************************************************/
/*          LoraWan  ClearJoinStatus  Method      */
/**************************************************/

void lr1mac_core_join_status_clear( void )
{
    lr1_mac_obj.join_status = NOT_JOINED;
    smtc_real_join_snapshot_channel_mask_init( &lr1_mac_obj );
}

/**************************************************/
/*          LoraWan  IsJoined  Method             */
/**************************************************/

void lr1mac_core_new_join( void )
{
    lr1_mac_obj.join_status = NOT_JOINED;
}
/**************************************************/
/*         LoraWan  SendPayload  Method           */
/**************************************************/

lr1mac_states_t lr1mac_core_payload_send_at_time( uint8_t fport, const uint8_t* data_in, const uint8_t size_in,
                                                  uint8_t packet_type, uint32_t target_time_ms )
{
    lr1mac_states_t status;
    status = lr1mac_core_payload_send( fport, data_in, size_in, packet_type, target_time_ms );
    if( status == LWPSTATE_SEND )
    {
        lr1_mac_obj.send_at_time = true;
        lr1_mac_obj.nb_trans_cpt = 1;  // Overwrite nb_trans_cpt, when downlink is At Time, repetitions are out dated
    }
    return status;
}

lr1mac_states_t lr1mac_core_payload_send( uint8_t fport, const uint8_t* data_in, const uint8_t size_in,
                                          uint8_t packet_type, uint32_t target_time_ms )
{
    status_lorawan_t status;
    status = smtc_real_is_valid_size( &lr1_mac_obj, lr1_mac_obj.tx_data_rate, size_in );
    if( status == ERRORLORAWAN )
    {
        BSP_DBG_TRACE_ERROR( "PAYLOAD SIZE TOO HIGH \n" );
        return ( LWPSTATE_INVALID );
    }
    if( lr1mac_core_is_otaa_device( ) == OTAA_DEVICE )
    {
        if( lr1_mac_obj.join_status == NOT_JOINED )
        {
            BSP_DBG_TRACE_ERROR( "OTAA DEVICE NOT JOINED YET\n" );
            return ( LWPSTATE_INVALID );
        }
    }
    if( lr1mac_state != LWPSTATE_IDLE )
    {
        BSP_DBG_TRACE_ERROR( "LP STATE NOT EQUAL TO IDLE \n" );
        return ( LWPSTATE_ERROR );
    }
    // Decrement duty cycle before check the available DTC
    smtc_real_duty_cycle_update( &lr1_mac_obj );
    if( smtc_real_next_channel_get( &lr1_mac_obj ) != OKLORAWAN )
    {
        BSP_DBG_TRACE_WARNING( "NO CHANNEL AVAILABLE\n" );
        return ( LWPSTATE_DUTY_CYCLE_FULL );
    }

    if( lr1mac_core_next_free_duty_cycle_ms_get( ) > 0 )
    {
        BSP_DBG_TRACE_WARNING( "Duty Cycle is full\n" );
        return ( LWPSTATE_DUTY_CYCLE_FULL );
    }

    lr1_mac_obj.timestamp_failsafe  = bsp_rtc_get_time_s( );
    lr1_mac_obj.rtc_target_timer_ms = target_time_ms;
    copy_user_payload( data_in, size_in );
    lr1_mac_obj.app_payload_size = size_in;
    lr1_mac_obj.tx_fport         = fport;
    lr1_mac_obj.tx_mtype         = packet_type;
    lr1_stack_mac_tx_frame_build( &lr1_mac_obj );
    lr1_stack_mac_tx_frame_encrypt( &lr1_mac_obj );
    if( packet_type == CONF_DATA_UP )
    {
        lr1_mac_obj.nb_trans_cpt = MAX_CONFUP_MSG;
    }
    else
    {
        lr1_mac_obj.nb_trans_cpt = lr1_mac_obj.nb_trans;
    }
    lr1mac_state = LWPSTATE_SEND;
    return ( lr1mac_state );
}

/**************************************************/
/*        LoraWan  Receive  Method                */
/**************************************************/
status_lorawan_t lr1mac_core_payload_receive( uint8_t* user_rx_port, uint8_t* user_rx_payload,
                                              uint8_t* user_rx_payloadSize )
{
    status_lorawan_t status = OKLORAWAN;
    if( lr1_mac_obj.available_app_packet == NO_LORA_RXPACKET_AVAILABLE )
    {
        status = ERRORLORAWAN;
    }
    else
    {
        *user_rx_payloadSize = lr1_mac_obj.rx_payload_size;
        *user_rx_port        = lr1_mac_obj.rx_fport;
        memcpy( user_rx_payload, lr1_mac_obj.rx_payload, lr1_mac_obj.rx_payload_size );
        lr1_mac_obj.available_app_packet = NO_LORA_RXPACKET_AVAILABLE;
    }
    return ( status );
}

/**************************************************/
/*       LoraWan  AdrModeSelect  Method           */
/**************************************************/

void lr1mac_core_dr_strategy_set( dr_strategy_t adr_mode_select )
{
    lr1_mac_obj.adr_mode_select = adr_mode_select;
    smtc_real_dr_distribution_set( &lr1_mac_obj, adr_mode_select );
    smtc_real_next_dr_get( &lr1_mac_obj );
}
/**************************************************/
/*       LoraWan  AdrModeSelect  Get Method       */
/**************************************************/

dr_strategy_t lr1mac_core_dr_strategy_get( void )
{
    return ( lr1_mac_obj.adr_mode_select );
}
/*************************************************************/
/*       LoraWan  Set DataRate Custom for custom adr profile */
/*************************************************************/

void lr1mac_core_dr_custom_set( uint32_t DataRateCustom )
{
    lr1_mac_obj.adr_custom = DataRateCustom;
}
/**************************************************/
/*         LoraWan  GetDevAddr  Method            */
/**************************************************/

uint32_t lr1mac_core_devaddr_get( void )
{
    return ( lr1_mac_obj.dev_addr );
}

/**************************************************/
/*         LoraWan  GetDevEUI  Method             */
/**************************************************/

void lr1mac_core_deveui_get( uint8_t* DevEui )
{
    memcpy( DevEui, lr1_mac_obj.dev_eui, 8 );
}

/**************************************************/
/*         LoraWan  SetDevEUI  Method             */
/**************************************************/

void lr1mac_core_deveui_set( uint8_t* DevEui )
{
    memcpy( lr1_mac_obj.dev_eui, DevEui, 8 );
}

/**************************************************/
/*         LoraWan  SetAppKey  Method             */
/**************************************************/

void lr1mac_core_app_key_set( uint8_t* AppKey )
{
    memcpy( lr1_mac_obj.app_key, AppKey, 16 );
}

/**************************************************/
/*         LoraWan  GetAppEui  Method             */
/**************************************************/

void lr1mac_core_appeui_key_get( uint8_t* AppEui )
{
    memcpy( AppEui, lr1_mac_obj.app_eui, 8 );
}

/**************************************************/
/*         LoraWan  SetAppEui  Method             */
/**************************************************/

void lr1mac_core_appeui_key_set( uint8_t* AppEui )
{
    memcpy( lr1_mac_obj.app_eui, AppEui, 8 );
}

/**************************************************/
/*         LoraWan  GetNextPower  Method          */
/**************************************************/

uint8_t lr1mac_core_next_power_get( void )
{
    return ( lr1_mac_obj.tx_power );
}

/**************************************************/
/*    LoraWan  GetLorawanProcessState  Method     */
/**************************************************/

lr1mac_states_t lr1mac_core_state_get( void )
{
    return ( lr1mac_state );
}

/**************************************************/
/*    LoraWan  StoreContext  Method             */
/**************************************************/

void lr1mac_core_context_save( void )
{
    smtc_real_memory_save( &lr1_mac_obj );
}
/**************************************************/
/*    LoraWan  storeContext  Method               */
/**************************************************/

void lr1mac_core_keys_set( lorawan_keys_t LoRaWanKeys )
{
    memcpy( lr1_mac_obj.app_skey, LoRaWanKeys.LoRaMacAppSKey, 16 );
    memcpy( lr1_mac_obj.nwk_skey, LoRaWanKeys.LoRaMacNwkSKey, 16 );
    memcpy( lr1_mac_obj.app_key, LoRaWanKeys.LoRaMacAppKey, 16 );
    memcpy( lr1_mac_obj.dev_eui, LoRaWanKeys.DevEui, 8 );
    memcpy( lr1_mac_obj.app_eui, LoRaWanKeys.AppEui, 8 );
    lr1_mac_obj.otaa_device = LoRaWanKeys.otaaDevice;
    lr1_mac_obj.dev_addr    = LoRaWanKeys.LoRaDevAddr;

    smtc_real_memory_save( &lr1_mac_obj );
}
/**************************************************/
/*   LoraWan  lr1mac_core_next_max_payload_length_get  Method     */
/**************************************************/

uint32_t lr1mac_core_next_max_payload_length_get( void )
{
    return ( smtc_real_max_payload_size_get( &lr1_mac_obj, lr1_mac_obj.tx_data_rate ) -
             lr1_mac_obj.tx_fopts_current_length - 8 );
}

/**************************************************/
/*        LoraWan  lr1mac_core_next_dr_get  Method        */
/**************************************************/

uint8_t lr1mac_core_next_dr_get( void )
{  // note return datareate in case of adr
    return ( lr1_mac_obj.tx_data_rate );
}

uint32_t lr1mac_core_next_frequency_get( void )
{  // note return datareate in case of adr
    return ( lr1_mac_obj.tx_frequency );
}

void lr1mac_core_factory_reset( void )
{
    smtc_real_bad_crc_memory_set( &lr1_mac_obj );
}

type_otaa_abp_t lr1mac_core_is_otaa_device( void )
{
    return ( type_otaa_abp_t ) lr1_mac_obj.otaa_device;
}

void lr1mac_core_otaa_set( type_otaa_abp_t deviceType )
{
    lr1_mac_obj.otaa_device = deviceType;
}
radio_planner_t* lr1mac_core_rp_get( void )
{
    return lr1_mac_obj.rp;
}
int8_t lr1mac_core_tx_power_offset_get( void )
{
    return lr1_mac_obj.tx_power_offset;
}
void lr1mac_core_tx_power_offset_set( int8_t power_off )
{
    lr1_mac_obj.tx_power_offset = power_off;
}

uint16_t lr1mac_core_nb_reset_get( void )
{
    return lr1_mac_obj.nb_of_reset;
}

uint16_t lr1mac_core_devnonce_get( void )
{
    return lr1_mac_obj.dev_nonce;
}

int16_t lr1mac_core_last_snr_get( void )
{
    return lr1_mac_obj.rx_snr;
}

int16_t lr1mac_core_last_rssi_get( void )
{
    return lr1_mac_obj.rx_rssi;
}

uint8_t lr1mac_core_min_dr_get( void )
{
    uint8_t tmp = lr1_stack_mac_min_dr_get( &lr1_mac_obj );
    BSP_DBG_TRACE_PRINTF( "Min DataRate = %d\n", tmp );
    return ( tmp );
}
uint8_t lr1mac_core_max_dr_get( void )
{
    uint8_t tmp = lr1_stack_mac_max_dr_get( &lr1_mac_obj );
    BSP_DBG_TRACE_PRINTF( "Max DataRate = %d\n", tmp );
    return ( tmp );
}

void lr1mac_core_apps_key_get( uint8_t* key )
{
    memcpy( key, lr1_mac_obj.app_skey, 16 );
}

status_lorawan_t lr1mac_core_context_load( void )
{
    return smtc_real_memory_load( &lr1_mac_obj );
}
receive_win_t lr1mac_core_rx_window_get( void )
{
    return ( receive_window_type );
}
uint32_t lr1mac_core_fcnt_up_get( void )
{
    return ( lr1_mac_obj.fcnt_up );
}
uint32_t lr1mac_core_next_join_time_second_get( void )
{
    return ( lr1_mac_obj.next_time_to_join_seconds );
}
int32_t lr1mac_core_next_free_duty_cycle_ms_get( void )
{
    int32_t nwk_dtc    = lr1_stack_network_next_free_duty_cycle_ms_get( &lr1_mac_obj );
    int32_t region_dtc = smtc_real_next_free_duty_cycle_ms_get( &lr1_mac_obj );

    return ( MAX( nwk_dtc, region_dtc ) );
}
void lr1mac_core_duty_cycle_enable_set( uint8_t enable )
{
    smtc_real_duty_cycle_enable_set( &lr1_mac_obj, enable );
}

uint32_t r1mac_core_version_get( void )
{
    return ( LR1MAC_PROTOCOL_VERSION );
}

/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
uint8_t lr1mac_core_rx_ack_bit_get( void )
{
    return ( lr1_mac_obj.rx_ack_bit );
}

lr1_stack_mac_t* lr1mac_core_stack_mac_get( void )
{
    return &lr1_mac_obj;
}

status_lorawan_t lr1mac_core_is_supported_region( smtc_real_region_types_t region_type )
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
        BSP_DBG_TRACE_ERROR( "Invalid Region 0x%02x\n", region_type );
        return ERRORLORAWAN;
    }
    return OKLORAWAN;
}

smtc_real_region_types_t lr1mac_core_get_region( void )
{
    return lr1_mac_obj.real->region_type;
}

status_lorawan_t lr1mac_core_set_region( smtc_real_region_types_t region_type )
{
    if( lr1mac_core_is_supported_region( region_type ) == OKLORAWAN )
    {
        lr1_mac_obj.real->region_type = region_type;
        lr1mac_core_context_save( );
        return OKLORAWAN;
    }
    return ERRORLORAWAN;
}

/*
 *-----------------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITIONS ------------------------------------------------
 */

static void copy_user_payload( const uint8_t* data_in, const uint8_t size_in )
{
    memcpy( &lr1_mac_obj.tx_payload[FHDROFFSET + lr1_mac_obj.tx_fopts_current_length], data_in, size_in );
}

static uint32_t failsafe_timstamp_get( void )
{
    return lr1_mac_obj.timestamp_failsafe;
}

static rp_status_t rp_status_get( void )
{
    return lr1_mac_obj.planner_status;
}
