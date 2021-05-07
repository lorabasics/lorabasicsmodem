/*!
 * \file      lorawan_api.c
 *
 * \brief     Lorawan abstraction layer functions.
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

#include "lorawan_api.h"
#include "lr1mac_core.h"
#include "smtc_modem_hal.h"
#include "smtc_crypto.h"
#include "smtc_lbt.h"
#include "smtc_duty_cycle.h"
#include "lr1mac_class_c.h"
#include "modem_utilities.h"
#include "fifo_ctrl.h"
#include "smtc_real.h"
#include "smtc_modem_api.h"
#include "modem_supervisor.h"

struct
{
    uint8_t  is_lorawan_certification_enabled;
    uint16_t fcnt_dwn_certif;
    uint32_t msg_typePrevious;
    uint32_t spare;
} lorawan_api_ctx;

#define fcnt_dwn_certif lorawan_api_ctx.fcnt_dwn_certif
#define msg_typePrevious lorawan_api_ctx.msg_typePrevious
#define is_lorawan_certification_enabled lorawan_api_ctx.is_lorawan_certification_enabled

// only one stack but ready to support more
uint8_t LoRaMacNwkSKeyInit[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t LoRaMacAppSKeyInit[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t LoRaMacAppKeyInit[]  = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t AppEuiInit[]         = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t DevEuiInit[]         = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

uint32_t              LoRaDevAddrInit = 0x00000000;
static lorawan_keys_t LoraWanKeys     = { LoRaMacNwkSKeyInit, LoRaMacAppSKeyInit, LoRaMacAppKeyInit, AppEuiInit,
                                      DevEuiInit,         0x00000000,         OTAA_DEVICE };

/*!
 * \brief   Intermediate state to be able in the future to manage more than one stack
 * \remark
 * \param [in]  LoRAStack : a pointer to the current stack
 * \param [out] set the local stack to LoRaStack
 * */
static struct
{
    lr1_stack_mac_t  lr1_mac_obj;
    smtc_lbt_t       lbt_obj;
    smtc_dtc_t       dtc_obj;
    lr1mac_class_c_t class_c_obj;
    fifo_ctrl_t      fifo_ctrl_obj;
} lr1mac_core_context;

#define FIFO_LORAWAN_SIZE 512
uint8_t fifo_buffer[FIFO_LORAWAN_SIZE];

#define lr1_mac_obj lr1mac_core_context.lr1_mac_obj
#define lbt_obj lr1mac_core_context.lbt_obj
#define dtc_obj lr1mac_core_context.dtc_obj
#define class_c_obj lr1mac_core_context.class_c_obj
#define fifo_ctrl_obj lr1mac_core_context.fifo_ctrl_obj

void lorawan_api_class_a_downlink_callback( lr1_stack_mac_t* lr1_mac_object );
void lorawan_api_class_c_downlink_callback( lr1mac_class_c_t* class_c_object );

void lorawan_rp_callback_api( radio_planner_t* rp )
{
    rp_radio_irq_callback( rp );
}

void lorawan_api_init( radio_planner_t* rp )
{
    smtc_real_region_types_t smtc_real_region_types = SMTC_REAL_REGION_UNKNOWN;
#if defined SX128X
    // default region for sx128x projects is 2G4
#if defined( REGION_WW2G4 )
    smtc_real_region_types = SMTC_REAL_REGION_WW2G4;
#else
#error "Please select supported region"
#endif
#elif defined LR1110 || defined SX126X
// default region for subgig projects is EU_868, then others.. depends of Makefile
#if defined( REGION_EU_868 )
    smtc_real_region_types = SMTC_REAL_REGION_EU_868;
#elif defined( REGION_US_915 )
    smtc_real_region_types = SMTC_REAL_REGION_US_915;
#elif defined( REGION_AU_915 )
    smtc_real_region_types = SMTC_REAL_REGION_AU_915;
#elif defined( REGION_CN_470 )
    smtc_real_region_types = SMTC_REAL_REGION_CN_470;
#elif defined( REGION_CN_470_RP_1_0 )
    smtc_real_region_types = SMTC_REAL_REGION_CN_470_RP_1_0;
#elif defined( REGION_AS_923 )
    smtc_real_region_types = SMTC_REAL_REGION_AS_923;
#elif defined( REGION_IN_865 )
    smtc_real_region_types = SMTC_REAL_REGION_IN_865;
#elif defined( REGION_KR_920 )
    smtc_real_region_types = SMTC_REAL_REGION_KR_920;
#elif defined( REGION_RU_864 )
    smtc_real_region_types = SMTC_REAL_REGION_RU_864;
#else
#error "Please select supported region"
#endif
#endif

    // init lr1mac core
    lr1mac_core_init( &lr1_mac_obj, &lbt_obj, &dtc_obj, rp, &LoraWanKeys, smtc_real_region_types,
                      ( void ( * )( void* ) ) lorawan_api_class_a_downlink_callback, &lr1_mac_obj );

    fifo_ctrl_init( &fifo_ctrl_obj, fifo_buffer, FIFO_LORAWAN_SIZE );

    lr1mac_class_c_init( &class_c_obj, &lr1_mac_obj, rp, 4, ( void ( * )( void* ) ) lr1mac_class_c_mac_rp_callback,
                         &class_c_obj, ( void ( * )( void* ) ) lorawan_api_class_c_downlink_callback, &class_c_obj );

    is_lorawan_certification_enabled = false;
}

void lorawan_api_class_a_downlink_callback( lr1_stack_mac_t* lr1_mac_object )
{
    if( modem_supervisor_update_downlink_frame( lr1_mac_object->rx_payload, lr1_mac_object->rx_payload_size,
                                                &( lr1_mac_object->rx_metadata ) ) )
    {
        lr1_mac_object->rx_metadata.rx_window = lorawan_api_rx_window_get( );  // manage also bit ACK/NACK
        if( fifo_ctrl_set( &fifo_ctrl_obj, lr1_mac_object->rx_payload, lr1_mac_object->rx_payload_size,
                           &( lr1_mac_object->rx_metadata ), sizeof( lr1mac_down_metadata_t ) ) != FIFO_STATUS_OK )
        {
            SMTC_MODEM_HAL_TRACE_PRINTF( "Fifo problem\n" );
            return;
        }
        else
        {
            fifo_ctrl_print_stat( &fifo_ctrl_obj );
        }
    }
}

void lorawan_api_class_c_downlink_callback( lr1mac_class_c_t* class_c_object )
{
    if( modem_supervisor_update_downlink_frame( class_c_object->rx_payload, class_c_object->rx_payload_size,
                                                &( class_c_object->rx_metadata ) ) )
    {
        class_c_object->rx_metadata.rx_window = class_c_object->receive_window_type;
        if( fifo_ctrl_set( &fifo_ctrl_obj, class_c_object->rx_payload, class_c_object->rx_payload_size,
                           &( class_c_object->rx_metadata ), sizeof( lr1mac_down_metadata_t ) ) != FIFO_STATUS_OK )
        {
            smtc_modem_hal_mcu_panic( "Fifo problem\n" );
            return;
        }
        else
        {
            fifo_ctrl_print_stat( &fifo_ctrl_obj );
        }
    }
}

smtc_real_region_types_t lorawan_api_get_region( void )
{
    return lr1mac_core_get_region( &lr1_mac_obj );
}

status_lorawan_t lorawan_api_set_region( smtc_real_region_types_t region_type )
{
    return lr1mac_core_set_region( &lr1_mac_obj, region_type );
}

lr1mac_states_t lorawan_api_payload_send( uint8_t fPort, const uint8_t* dataIn, const uint8_t sizeIn,
                                          uint8_t PacketType, uint32_t TargetTimeMS )
{
    return lr1mac_core_payload_send( &lr1_mac_obj, fPort, dataIn, sizeIn, PacketType, TargetTimeMS );
}

lr1mac_states_t lorawan_api_payload_send_at_time( uint8_t fPort, const uint8_t* dataIn, const uint8_t sizeIn,
                                                  uint8_t PacketType, uint32_t TargetTimeMS )
{
    return lr1mac_core_payload_send_at_time( &lr1_mac_obj, fPort, dataIn, sizeIn, PacketType, TargetTimeMS );
}

status_lorawan_t lorawan_api_payload_receive( uint8_t* UserRxFport, uint8_t* UserRxPayload, uint8_t* UserRxPayloadSize )
{
    return lr1mac_core_payload_receive( &lr1_mac_obj, UserRxFport, UserRxPayload, UserRxPayloadSize );
}

lr1mac_states_t lorawan_api_join( uint32_t target_time_ms )
{
    return lr1mac_core_join( &lr1_mac_obj, target_time_ms );
}

join_status_t lorawan_api_isjoined( void )
{
    return lr1_mac_joined_status_get( &lr1_mac_obj );
}

void lorawan_api_join_status_clear( void )
{
    lr1mac_core_join_status_clear( &lr1_mac_obj );
}

status_lorawan_t lorawan_api_dr_strategy_set( dr_strategy_t adrModeSelect )
{
    return ( lr1mac_core_dr_strategy_set( &lr1_mac_obj, adrModeSelect ) );
}

dr_strategy_t lorawan_api_dr_strategy_get( void )
{
    return lr1mac_core_dr_strategy_get( &lr1_mac_obj );
}

void lorawan_api_dr_custom_set( uint32_t DataRateCustom )
{
    lr1mac_core_dr_custom_set( &lr1_mac_obj, DataRateCustom );
}

lr1mac_states_t lorawan_api_process( user_rx_packet_type_t* AvailableRxPacket )
{
    return lr1mac_core_process( &lr1_mac_obj, AvailableRxPacket );
}

void lorawan_api_context_load( void )
{
    lr1mac_core_context_load( &lr1_mac_obj );
}

void lorawan_api_context_save( void )
{
    lr1mac_core_context_save( &lr1_mac_obj );
}

int16_t lorawan_api_last_snr_get( void )
{
    return lr1mac_core_last_snr_get( &lr1_mac_obj );
}

int16_t lorawan_api_last_rssi_get( void )
{
    return lr1mac_core_last_rssi_get( &lr1_mac_obj );
}

void lorawan_api_keys_set( lorawan_keys_t LoRaWanKeys )
{
    lr1mac_core_keys_set( &lr1_mac_obj, LoRaWanKeys );
}

void lorawan_api_set_default_key( uint8_t default_app_key[16], uint8_t default_dev_eui[8], uint8_t default_join_eui[8] )
{
    memcpy( LoraWanKeys.LoRaMacAppKey, default_app_key, 16 );
    memcpy( LoraWanKeys.AppEui, default_join_eui, 8 );
    memcpy( LoraWanKeys.DevEui, default_dev_eui, 8 );
}

void lorawan_api_factory_reset( void )
{
    lr1mac_core_factory_reset( &lr1_mac_obj );
}

type_otaa_abp_t lorawan_api_is_ota_device( void )
{
    return lr1mac_core_is_otaa_device( &lr1_mac_obj );
}

void lorawan_api_ota_set( type_otaa_abp_t deviceType )
{
    lr1mac_core_otaa_set( &lr1_mac_obj, deviceType );
}

uint32_t lorawan_api_next_max_payload_length_get( void )
{
    return lr1mac_core_next_max_payload_length_get( &lr1_mac_obj );
}

void lorawan_api_new_join( void )
{
    lr1mac_core_new_join( &lr1_mac_obj );
}

uint32_t lorawan_api_devaddr_get( void )
{
    return lr1mac_core_devaddr_get( &lr1_mac_obj );
}

void lorawan_api_deveui_get( uint8_t* DevEui )
{
    lr1mac_core_deveui_get( &lr1_mac_obj, DevEui );
}

void lorawan_api_deveui_set( const uint8_t* DevEui )
{
#if defined( LR1110_MODEM )
    smtc_crypto_store_dev_eui( DevEui );
#endif  // LR1110_MODEM
    lr1mac_core_deveui_set( &lr1_mac_obj, DevEui );
    lorawan_api_context_save( );
}

void lorawan_api_app_key_set( const uint8_t* AppKey )
{
#if defined( LR1110_MODEM ) || defined( USE_LR1110_SE )
    smtc_crypto_store_app_key( AppKey );
#else
    lr1mac_core_app_key_set( &lr1_mac_obj, AppKey );
    lorawan_api_context_save( );
#endif  // LR1110_MODEM
}

void lorawan_api_appeui_key_get( uint8_t* AppEui )
{
    lr1mac_core_appeui_key_get( &lr1_mac_obj, AppEui );
}

void lorawan_api_appeui_key_set( const uint8_t* AppEui )
{
#if defined( LR1110_MODEM )
    smtc_crypto_store_app_eui( AppEui );
#endif  // LR1110_MODEM
    lr1mac_core_appeui_key_set( &lr1_mac_obj, AppEui );
    lorawan_api_context_save( );
}

uint8_t lorawan_api_next_power_get( void )
{
    return lr1mac_core_next_power_get( &lr1_mac_obj );
}

uint8_t lorawan_api_next_dr_get( void )
{
    return lr1mac_core_next_dr_get( &lr1_mac_obj );
}

uint32_t lorawan_api_next_frequency_get( void )
{
    return lr1mac_core_next_frequency_get( &lr1_mac_obj );
}

uint8_t lorawan_api_max_tx_dr_get( void )
{
    return lr1mac_core_max_tx_dr_get( &lr1_mac_obj );
}

uint16_t lorawan_api_mask_tx_dr_channel_up_dwell_time_check( void )
{
    return lr1mac_core_mask_tx_dr_channel_up_dwell_time_check( &lr1_mac_obj );
}

uint8_t lorawan_api_min_tx_dr_get( void )
{
    return lr1mac_core_min_tx_dr_get( &lr1_mac_obj );
}

lr1mac_states_t lorawan_api_state_get( void )
{
    return lr1mac_core_state_get( &lr1_mac_obj );
}

uint16_t lorawan_api_nb_reset_get( void )
{
    return lr1mac_core_nb_reset_get( &lr1_mac_obj );
}

uint16_t lorawan_api_devnonce_get( void )
{
    return lr1mac_core_devnonce_get( &lr1_mac_obj );
}

uint8_t* lorawan_api_apps_key_get( void )
{
    return lr1mac_core_apps_key_get( &lr1_mac_obj );
}

receive_win_t lorawan_api_rx_window_get( void )
{
    return lr1mac_core_rx_window_get( &lr1_mac_obj );
}

uint32_t lorawan_api_next_join_time_second_get( void )
{
    return lr1mac_core_next_join_time_second_get( &lr1_mac_obj );
}

int32_t lorawan_api_next_free_duty_cycle_ms_get( void )
{
    return lr1mac_core_next_free_duty_cycle_ms_get( &lr1_mac_obj );
}

status_lorawan_t lorawan_api_duty_cycle_enable_set( bool enable )
{
    if( lr1mac_core_duty_cycle_enable_set( &lr1_mac_obj, enable ) == true )
    {
        return OKLORAWAN;
    }
    return ERRORLORAWAN;
}

uint32_t lorawan_api_fcnt_up_get( void )
{
    return lr1mac_core_fcnt_up_get( &lr1_mac_obj );
}

uint8_t lorawan_api_rp_hook_id_get( void )
{
    uint8_t hook_id;
    rp_hook_get_id( lr1mac_core_rp_get( &lr1_mac_obj ), &lr1_mac_obj, &hook_id );
    return hook_id;
}

void lorawan_api_class_c_enabled( bool enable )
{
    lr1mac_class_c_enabled( &class_c_obj, enable );

    if( lorawan_api_isjoined( ) == JOINED )
    {
        lr1mac_class_c_start( &class_c_obj );
    }
}

void lorawan_api_class_c_start( void )
{
    lr1mac_class_c_start( &class_c_obj );
}

void lorawan_api_class_c_stop( void )
{
    lr1mac_class_c_stop( &class_c_obj );
}

uint8_t lorawan_api_rx_ack_bit_get( void )
{
    return lr1mac_core_rx_ack_bit_get( &lr1_mac_obj );
}

status_lorawan_t lorawan_api_no_rx_packet_count_config_set( uint16_t no_rx_packet_count )
{
    return lr1mac_core_set_no_rx_packet_count_config( &lr1_mac_obj, no_rx_packet_count );
}

uint16_t lorawan_api_no_rx_packet_count_config_get( void )
{
    return lr1mac_core_get_no_rx_packet_count_config( &lr1_mac_obj );
}

uint16_t lorawan_api_no_rx_packet_count_current_get( void )
{
    return lr1mac_core_get_no_rx_packet_count_current( &lr1_mac_obj );
}

void lorawan_api_no_rx_packet_count_in_mobile_mode_set( uint16_t no_rx_packet_count )
{
    lr1mac_core_set_no_rx_packet_count_in_mobile_mode( &lr1_mac_obj, no_rx_packet_count );
}

uint16_t lorawan_api_no_rx_packet_count_in_mobile_mode_get( void )
{
    return lr1mac_core_get_no_rx_packet_count_in_mobile_mode( &lr1_mac_obj );
}

void lorawan_api_modem_certification_set( uint8_t enable )
{
    lr1mac_core_certification_set( &lr1_mac_obj, enable );
}

uint8_t lorawan_api_modem_certification_is_enabled( void )
{
    return lr1mac_core_certification_get( &lr1_mac_obj );
}

uint8_t lorawan_api_certification_is_enabled( void )
{
    return is_lorawan_certification_enabled;
}

uint8_t lorawan_api_certification( uint8_t new_cmd, uint8_t fport_in, uint8_t* user_fport, uint8_t* msg_type,
                                   uint8_t* user_rx_payload, uint8_t* user_rx_payload_size, uint8_t* user_payload,
                                   uint8_t* user_payload_size )
{
    if( lorawan_api_modem_certification_is_enabled( ) == false )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "Modem LoRaWAN Certification is not enabled\n" );
        return 0;
    }
    uint32_t temp;
    uint8_t  i;

    *user_payload_size = 2;
    *msg_type          = false;

    if( new_cmd == 1 || *user_rx_payload_size > 0 )
    {
        fcnt_dwn_certif++;
        if( *user_rx_payload_size > 0 && fport_in == 224 )
        {
            switch( user_rx_payload[0] )
            {
            case 0:  // end of test
                *user_fport        = 3;
                *user_payload_size = 11;
                for( i = 0; i < 11; i++ )
                {
                    user_payload[i] = i;
                }
                is_lorawan_certification_enabled = false;
                break;
            case 1:
                temp = ( user_rx_payload[0] << 24 ) + ( user_rx_payload[1] << 16 ) + ( user_rx_payload[2] << 8 ) +
                       ( user_rx_payload[3] );
                if( temp == 0x01010101 )
                {
                    lorawan_api_dr_strategy_set( STATIC_ADR_MODE );
                    fcnt_dwn_certif                  = 0;
                    user_payload[0]                  = fcnt_dwn_certif >> 8;
                    user_payload[1]                  = fcnt_dwn_certif & 0xFF;
                    is_lorawan_certification_enabled = true;
                    *user_fport                      = 224;
                    msg_typePrevious                 = false;
                }
                break;
            case 2:  // Confirmed Uplink
                *user_fport      = 224;
                *msg_type        = true;
                msg_typePrevious = *msg_type;
                user_payload[0]  = fcnt_dwn_certif >> 8;
                user_payload[1]  = fcnt_dwn_certif & 0xFF;
                break;
            case 3:  // UnConfirmed Uplink
                *user_fport      = 224;
                *msg_type        = false;
                msg_typePrevious = *msg_type;
                user_payload[0]  = fcnt_dwn_certif >> 8;
                user_payload[1]  = fcnt_dwn_certif & 0xFF;
                break;
            case 4:  // echo payload
                *user_fport        = 224;
                *msg_type          = msg_typePrevious;
                *user_payload_size = *user_rx_payload_size;
                user_payload[0]    = 4;
                for( i = 1; i < ( *user_payload_size ); i++ )
                {
                    user_payload[i] = user_rx_payload[i] + 1;
                }
                break;
            case 5:  // link check request
                *user_payload_size = 1;
                user_payload[0]    = 2;
                *user_fport        = 0;
                *msg_type          = false;
                msg_typePrevious   = *msg_type;
                break;
            case 6:  // rejoin
                lorawan_api_ota_set( OTAA_DEVICE );
                lorawan_api_join_status_clear( );
                *user_fport        = 3;
                *user_payload_size = 11;
                for( i = 0; i < 11; i++ )
                {
                    user_payload[i] = i;
                }
                return ( 0 );
            default:
                break;
            }
        }
        else
        {  // for the case of echo cmd
            *user_fport     = 224;
            *msg_type       = msg_typePrevious;
            user_payload[0] = fcnt_dwn_certif >> 8;
            user_payload[1] = fcnt_dwn_certif & 0xFF;
        }
    }
    else
    {  // for the case of echo cmd
        *user_fport     = 224;
        *msg_type       = msg_typePrevious;
        user_payload[0] = fcnt_dwn_certif >> 8;
        user_payload[1] = fcnt_dwn_certif & 0xFF;
    }

    return ( user_rx_payload[0] );
}

/*!
 * \brief  return true if stack receive a link adr request
 * \remark reset the flag automatically each time the upper layer call this function
 * \param [in]  void
 * \param [out] bool
 */
bool lorawan_api_available_link_adr_get( void )
{
    return lr1mac_core_available_link_adr_get( &lr1_mac_obj );
}
lr1_stack_mac_t* lorawan_api_stack_mac_get( void )
{
    return ( &lr1_mac_obj );
}

fifo_ctrl_t* lorawan_api_get_fifo_obj( void )
{
    return &fifo_ctrl_obj;
}

void lorawan_api_set_network_type( bool network_type )
{
    uint8_t sync_word = ( network_type == true ) ? smtc_real_get_public_sync_word( &lr1_mac_obj )
                                                 : smtc_real_get_private_sync_word( &lr1_mac_obj );

    smtc_real_set_sync_word( &lr1_mac_obj, sync_word );
}
bool lorawan_api_get_network_type( void )
{
    uint8_t sync_word = smtc_real_get_sync_word( &lr1_mac_obj );
    return ( ( sync_word == smtc_real_get_public_sync_word( &lr1_mac_obj ) ) ? true : false );
}

uint8_t lorawan_api_nb_trans_get( void )
{
    return lr1_stack_nb_trans_get( &lr1_mac_obj );
}

status_lorawan_t lorawan_api_nb_trans_set( uint8_t nb_trans )
{
    return lr1_stack_nb_trans_set( &lr1_mac_obj, nb_trans );
}

uint32_t lorawan_api_get_crystal_error( void )
{
    return lr1_stack_get_crystal_error( &lr1_mac_obj );
}

void lorawan_api_set_crystal_error( uint32_t crystal_error )
{
    lr1_stack_set_crystal_error( &lr1_mac_obj, crystal_error );
}
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
