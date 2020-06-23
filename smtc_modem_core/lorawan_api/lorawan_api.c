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
#include "smtc_bsp.h"

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

static smtc_real_t smtc_region = {
    .context = NULL,
#if defined REGION_WW2G4
    .region_type = SMTC_REAL_REGION_WW2G4,
#elif defined REGION_EU_868
    .region_type = SMTC_REAL_REGION_EU_868,
#elif defined REGION_US_915
    .region_type = SMTC_REAL_REGION_US_915,
#else
#error "Please select supported region"
#endif
};

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

static bool is_lorawan_certification_enabled = false;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

/*!
 * \brief   Intermediate state to be able in the future to manage more than one stack
 * \remark
 * \param [in]  LoRAStack : a pointer to the current stack
 * \param [out] set the local stack to LoRaStack
 * */

void lorawan_api_init( radio_planner_t* rp )
{
    // init lr1mac core
    lr1mac_core_init( rp, &LoraWanKeys, &smtc_region );
}

smtc_real_region_types_t lorawan_api_get_region( void )
{
    return lr1mac_core_get_region( );
}

status_lorawan_t lorawan_api_set_region( smtc_real_region_types_t region_type )
{
    return lr1mac_core_set_region( region_type );
}

lr1mac_states_t lorawan_api_payload_send( uint8_t fPort, const uint8_t* dataIn, const uint8_t sizeIn,
                                          uint8_t PacketType, uint32_t TargetTimeMS )
{
    return lr1mac_core_payload_send( fPort, dataIn, sizeIn, PacketType, TargetTimeMS );
}

lr1mac_states_t lorawan_api_payload_send_at_time( uint8_t fPort, const uint8_t* dataIn, const uint8_t sizeIn,
                                                  uint8_t PacketType, uint32_t TargetTimeMS )
{
    return lr1mac_core_payload_send_at_time( fPort, dataIn, sizeIn, PacketType, TargetTimeMS );
}

status_lorawan_t lorawan_api_payload_receive( uint8_t* UserRxFport, uint8_t* UserRxPayload, uint8_t* UserRxPayloadSize )
{
    return lr1mac_core_payload_receive( UserRxFport, UserRxPayload, UserRxPayloadSize );
}

lr1mac_states_t lorawan_api_join( uint32_t target_time_ms )
{
    return lr1mac_core_join( target_time_ms );
}

join_status_t lorawan_api_isjoined( void )
{
    return lr1_mac_joined_status_get( );
}

void lorawan_api_join_status_clear( void )
{
    lr1mac_core_join_status_clear( );
}

void lorawan_api_dr_strategy_set( dr_strategy_t adrModeSelect )
{
    lr1mac_core_dr_strategy_set( adrModeSelect );
}

dr_strategy_t lorawan_api_dr_strategy_get( void )
{
    return lr1mac_core_dr_strategy_get( );
}

void lorawan_api_dr_custom_set( uint32_t DataRateCustom )
{
    lr1mac_core_dr_custom_set( DataRateCustom );
}

lr1mac_states_t lorawan_api_process( user_rx_packet_type_t* AvailableRxPacket )
{
    return lr1mac_core_process( AvailableRxPacket );
}

void lorawan_api_context_load( void )
{
    lr1mac_core_context_load( );
}

void lorawan_api_context_save( void )
{
    lr1mac_core_context_save( );
}

int16_t lorawan_api_last_snr_get( void )
{
    return lr1mac_core_last_snr_get( );
}

int16_t lorawan_api_last_rssi_get( void )
{
    return lr1mac_core_last_rssi_get( );
}

void lorawan_api_keys_set( lorawan_keys_t LoRaWanKeys )
{
    lr1mac_core_keys_set( LoRaWanKeys );
}

void lorawan_api_factory_reset( void )
{
    lr1mac_core_factory_reset( );
}

type_otaa_abp_t lorawan_api_is_ota_device( void )
{
    return lr1mac_core_is_otaa_device( );
}

void lorawan_api_ota_set( type_otaa_abp_t deviceType )
{
    lr1mac_core_otaa_set( deviceType );
}

uint32_t lorawan_api_next_max_payload_length_get( void )
{
    return lr1mac_core_next_max_payload_length_get( );
}

void lorawan_api_new_join( void )
{
    lr1mac_core_new_join( );
}

uint32_t lorawan_api_devaddr_get( void )
{
    return lr1mac_core_devaddr_get( );
}

void lorawan_api_deveui_get( uint8_t* DevEui )
{
    lr1mac_core_deveui_get( DevEui );
}

void lorawan_api_deveui_set( uint8_t* DevEui )
{
    lr1mac_core_deveui_set( DevEui );
}

void lorawan_api_app_key_set( uint8_t* AppKey )
{
    lr1mac_core_app_key_set( AppKey );
}

void lorawan_api_appeui_key_get( uint8_t* AppEui )
{
    lr1mac_core_appeui_key_get( AppEui );
}

void lorawan_api_appeui_key_set( uint8_t* AppEui )
{
    lr1mac_core_appeui_key_set( AppEui );
}

uint8_t lorawan_api_next_power_get( void )
{
    return lr1mac_core_next_power_get( );
}

uint8_t lorawan_api_next_dr_get( void )
{
    return lr1mac_core_next_dr_get( );
}

uint32_t lorawan_api_next_frequency_get( void )
{
    return lr1mac_core_next_frequency_get( );
}

uint8_t lorawan_api_max_dr_get( void )
{
    return lr1mac_core_max_dr_get( );
}

uint8_t lorawan_api_min_dr_get( void )
{
    return lr1mac_core_min_dr_get( );
}

lr1mac_states_t lorawan_api_state_get( void )
{
    return lr1mac_core_state_get( );
}

uint16_t lorawan_api_nb_reset_get( void )
{
    return lr1mac_core_nb_reset_get( );
}

uint16_t lorawan_api_devnonce_get( void )
{
    return lr1mac_core_devnonce_get( );
}

uint8_t* lorawan_api_apps_key_get( void )
{
    uint8_t* key = NULL;
    lr1mac_core_apps_key_get( key );
    return ( key );
}

receive_win_t lorawan_api_rx_window_get( void )
{
    return lr1mac_core_rx_window_get( );
}

uint32_t lorawan_api_next_join_time_second_get( void )
{
    return lr1mac_core_next_join_time_second_get( );
}

int32_t lorawan_api_next_free_duty_cycle_ms_get( void )
{
    return lr1mac_core_next_free_duty_cycle_ms_get( );
}

void lorawan_api_duty_cycle_enable_set( uint8_t enable )
{
    lr1mac_core_duty_cycle_enable_set( enable );
}

uint32_t lorawan_api_fcnt_up_get( void )
{
    return lr1mac_core_fcnt_up_get( );
}

uint8_t lorawan_api_rp_hook_id_get( void )
{
    uint8_t hook_id;
    rp_hook_get_id( lr1mac_core_rp_get( ), lr1mac_core_stack_mac_get( ), &hook_id );
    return hook_id;
}

int8_t lorawan_api_tx_power_offset_get( )
{
    return lr1mac_core_tx_power_offset_get( );
}

void lorawan_api_tx_power_offset_set( int8_t power_off )
{
    lr1mac_core_tx_power_offset_set( power_off );
}

uint8_t lorawan_api_rx_ack_bit_get( void )
{
    return lr1mac_core_rx_ack_bit_get( );
}

bool lorawan_api_certification_is_enabled( void )
{
    return is_lorawan_certification_enabled;
}

uint8_t lorawan_api_certification( bool new_cmd, uint8_t fport_in, uint8_t* user_fport, uint8_t* msg_type,
                                   uint8_t* user_rx_payload, uint8_t* user_rx_payload_size, uint8_t* user_payload,
                                   uint8_t* user_payload_size )
{
    uint32_t        temp;
    static uint16_t fcnt_dwn_certif  = 0;
    static uint32_t msg_typePrevious = UNCONF_DATA_UP;
    uint8_t         i;

    *user_payload_size = 2;
    *msg_type          = UNCONF_DATA_UP;
    // BSP_DBG_TRACE_ARRAY( "CERTIF PAYLOAD", user_rx_payload, *user_rx_payload_size );

    if( new_cmd == true || *user_rx_payload_size > 0 )
    {
        fcnt_dwn_certif++;
        if( *user_rx_payload_size > 0 && fport_in == 224 )
        {
            BSP_DBG_TRACE_WARNING( "Certif cmd=%u\n", user_rx_payload[0] );
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
                }
                break;
            case 2:  // Confirmed Uplink
                *user_fport      = 224;
                *msg_type        = CONF_DATA_UP;
                msg_typePrevious = *msg_type;
                user_payload[0]  = fcnt_dwn_certif >> 8;
                user_payload[1]  = fcnt_dwn_certif & 0xFF;
                break;
            case 3:  // UnConfirmed Uplink
                *user_fport      = 224;
                *msg_type        = UNCONF_DATA_UP;
                msg_typePrevious = *msg_type;
                user_payload[0]  = fcnt_dwn_certif >> 8;
                user_payload[1]  = fcnt_dwn_certif & 0xFF;
                break;
            case 4:  // echo payload
                *user_fport        = 224;
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
                break;
            case 6:  // rejoin
                lorawan_api_ota_set( OTAA_DEVICE );
                lorawan_api_join_status_clear( );
                lorawan_api_new_join( );
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
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
