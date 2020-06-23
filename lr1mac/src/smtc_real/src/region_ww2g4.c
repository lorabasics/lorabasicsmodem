/*!
 * \file      region_ww2g4.c
 *
 * \brief     region_2_4 abstraction layer implementation
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

#include <string.h>  // memcpy
#include "region_ww2g4.h"
#include "lr1_stack_mac_layer.h"
#include "lr1mac_utilities.h"
#include "smtc_bsp.h"
#include "ral_defs.h"

static uint32_t tx_frequency_channel[NUMBER_OF_CHANNEL_WW2G4];
static uint32_t rx1_frequency_channel[NUMBER_OF_CHANNEL_WW2G4];
static uint8_t  min_dr_channel[NUMBER_OF_CHANNEL_WW2G4];
static uint8_t  max_dr_channel[NUMBER_OF_CHANNEL_WW2G4];
static uint8_t  channel_index_enabled[NUMBER_OF_CHANNEL_WW2G4];  // Contain the index of the activated channel only
static uint8_t  dr_distribution_init[8] = { 0 };
static uint8_t  dr_distribution[8]      = { 0 };
static uint32_t unwrapped_channel_mask  = 0xFFFF;

// Private region_ww2g4 utilities declaration
//
/*!
 *
 */
static void tx_dr_to_sf_bw( lr1_stack_mac_t* lr1_mac, uint8_t dr );
static void rx2_dr_to_sf_bw( lr1_stack_mac_t* lr1_mac, uint8_t dr );

static mac_context_t mac_context;

/*!
 *
 */

void region_ww2g4_init( lr1_stack_mac_t* lr1_mac )
{
    for( uint8_t i = 0; i < NUMBER_OF_CHANNEL_WW2G4; i++ )
    {
        tx_frequency_channel[i]  = 0;
        rx1_frequency_channel[i] = 0;
        channel_index_enabled[i] = CHANNEL_DISABLED;
        min_dr_channel[i]        = 0;
        max_dr_channel[i]        = 5;
    }
    // enable the 3 defaults channels
    channel_index_enabled[0] = CHANNEL_ENABLED;
    channel_index_enabled[1] = CHANNEL_ENABLED;
    channel_index_enabled[2] = CHANNEL_ENABLED;

    memset( min_dr_channel, 0, 3 );
    memset( max_dr_channel, 7, 3 );
#if defined( PERF_TEST_ENABLED )
    tx_frequency_channel[0]  = 2479000000;
    tx_frequency_channel[1]  = 2479000000;
    tx_frequency_channel[2]  = 2479000000;
    rx1_frequency_channel[0] = 2479000000;
    rx1_frequency_channel[1] = 2479000000;
    rx1_frequency_channel[2] = 2479000000;
#else
    tx_frequency_channel[0]  = 2403000000;
    tx_frequency_channel[1]  = 2425000000;
    tx_frequency_channel[2]  = 2479000000;
    rx1_frequency_channel[0] = 2403000000;
    rx1_frequency_channel[1] = 2425000000;
    rx1_frequency_channel[2] = 2479000000;
#endif

    lr1_mac->rx2_frequency    = RX2_FREQ_WW2G4;
    lr1_mac->tx_power         = TX_POWER_WW2G4;
    lr1_mac->rx1_dr_offset    = 0;
    lr1_mac->rx2_data_rate    = 0;
    lr1_mac->rx1_delay_s      = RECEIVE_DELAY1_WW2G4;
    lr1_mac->tx_data_rate_adr = 0;
    lr1_mac->adr_custom       = BSP_USER_DR_DISTRIBUTION_PARAMETERS;
    memset( dr_distribution_init, 1, 8 );
}

status_lorawan_t region_ww2g4_is_valid_rx1_dr_offset( uint8_t rx1_dr_offset )
{
    status_lorawan_t status = OKLORAWAN;
    if( rx1_dr_offset > 5 )
    {
        status = ERRORLORAWAN;
        BSP_DBG_TRACE_MSG( "RECEIVE AN INVALID RX1 DR OFFSET \n" );
    }
    return ( status );
}

status_lorawan_t region_ww2g4_is_valid_dr( uint8_t dr )
{
    status_lorawan_t status;
    status = ( dr > 7 ) ? ERRORLORAWAN : OKLORAWAN;
    if( status == ERRORLORAWAN )
    {
        BSP_DBG_TRACE_WARNING( " Invalid data rate\n" );
    }
    return ( status );
}

status_lorawan_t region_ww2g4_is_acceptable_dr( uint8_t dr )
{
    status_lorawan_t status = ERRORLORAWAN;
    for( uint8_t i = 0; i < NUMBER_OF_CHANNEL_WW2G4; i++ )
    {
        if( ( ( unwrapped_channel_mask >> i ) & 0x1 ) == 1 )
        {
            if( ( dr >= min_dr_channel[i] ) && ( dr <= max_dr_channel[i] ) )
            {
                return ( OKLORAWAN );
            }
        }
    }
    BSP_DBG_TRACE_WARNING( " Not acceptable data rate\n" );
    return ( status );
}

status_lorawan_t region_ww2g4_is_valid_tx_frequency( uint32_t frequency )
{
    status_lorawan_t status = OKLORAWAN;
    if( frequency == 0 )
    {
        return ( status );
    }
    if( ( frequency > FREQMAX_WW2G4 ) || ( frequency < FREQMIN_WW2G4 ) )
    {
        status = ERRORLORAWAN;
        BSP_DBG_TRACE_WARNING( "RECEIVE AN INVALID FREQUENCY = %lu\n", frequency );
    }
    return ( status );
}

status_lorawan_t region_ww2g4_is_valid_rx_frequency( uint32_t frequency )
{
    status_lorawan_t status = OKLORAWAN;
    if( ( frequency > FREQMAX_WW2G4 ) || ( frequency < FREQMIN_WW2G4 ) )
    {
        status = ERRORLORAWAN;
        BSP_DBG_TRACE_WARNING( "RECEIVE AN INVALID Rx FREQUENCY = %lu\n", frequency );
    }
    return ( status );
}

status_lorawan_t region_ww2g4_is_valid_tx_power( uint8_t power )
{
    status_lorawan_t status = OKLORAWAN;
    if( ( power > 7 ) )
    {
        status = ERRORLORAWAN;
        BSP_DBG_TRACE_WARNING( "RECEIVE AN INVALID Power Cmd = %d\n", power );
    }
    return ( status );
}

status_lorawan_t region_ww2g4_is_valid_channel_index( uint8_t channel_index )
{
    status_lorawan_t status = OKLORAWAN;
    if( ( channel_index < 3 ) || ( channel_index > 15 ) )
    {
        status = ERRORLORAWAN;
        BSP_DBG_TRACE_WARNING( "RECEIVE AN INVALID Channel Index Cmd = %d\n", channel_index );
    }
    return ( status );
}

status_lorawan_t region_ww2g4_is_valid_size( lr1_stack_mac_t* lr1_mac, uint8_t dr, uint8_t size )
{
    status_lorawan_t status;
    uint8_t          N[8] = { 51, 115, 220, 220, 220, 220, 220, 220 };
    status                = ( ( size + lr1_mac->tx_fopts_current_length ) > N[dr] ) ? ERRORLORAWAN : OKLORAWAN;
    if( status == ERRORLORAWAN )
    {
        BSP_DBG_TRACE_WARNING( " Invalid size \n" );
    }
    return ( status );
}

status_lorawan_t region_ww2g4_memory_load( lr1_stack_mac_t* lr1_mac )
{
    bsp_nvm_context_restore( BSP_LORAWAN_CONTEXT_ADDR_OFFSET, ( uint8_t* ) &( mac_context ), sizeof( mac_context ) );

    if( lr1mac_utilities_crc( ( uint8_t* ) &mac_context, sizeof( mac_context ) - 4 ) == mac_context.crc )
    {
        memcpy( lr1_mac->app_eui, mac_context.appeui, 8 );
        memcpy( lr1_mac->dev_eui, mac_context.deveui, 8 );
        memcpy( lr1_mac->app_key, mac_context.appkey, 16 );
        lr1_mac->dev_nonce   = mac_context.devnonce;
        lr1_mac->adr_custom  = mac_context.adr_custom;
        lr1_mac->nb_of_reset = mac_context.nb_reset + 1;  // @todo move increment in mcu_reset api and remove all nvic
        lr1_mac->real->region_type = ( smtc_real_region_types_t ) mac_context.region_type;
        region_ww2g4_memory_save( lr1_mac );  // to save new number of reset
        BSP_DBG_TRACE_PRINTF( " DevNonce = 0x%x ", lr1_mac->dev_nonce );
        BSP_DBG_TRACE_PRINTF( ", NbOfReset = %d \n", lr1_mac->nb_of_reset );
        BSP_DBG_TRACE_PRINTF( " Region = %d\n", lr1_mac->real->region_type );
        return OKLORAWAN;
    }
    else  // start with "in rescue eeprom mode"
    {
        bsp_nvm_context_restore( BSP_LORAWAN_CONTEXT_ADDR_OFFSET + sizeof( mac_context ) + 4,
                                 ( uint8_t* ) &( mac_context ), sizeof( mac_context ) );

        if( lr1mac_utilities_crc( ( uint8_t* ) &mac_context, sizeof( mac_context ) - 4 ) == mac_context.crc )
        {
            memcpy( lr1_mac->app_eui, mac_context.appeui, 8 );
            memcpy( lr1_mac->dev_eui, mac_context.deveui, 8 );
            memcpy( lr1_mac->app_key, mac_context.appkey, 16 );
            lr1_mac->dev_nonce   = mac_context.devnonce;
            lr1_mac->adr_custom  = mac_context.adr_custom;
            lr1_mac->nb_of_reset = mac_context.nb_reset;  // @todo move increment in mcu_reset api and remove all nvic
            lr1_mac->real->region_type = ( smtc_real_region_types_t ) mac_context.region_type;
            region_ww2g4_memory_save( lr1_mac );  // to save new number of reset
            BSP_DBG_TRACE_PRINTF( "\n start on rescue eeprom , DevNonce = 0x%x ", lr1_mac->dev_nonce );
            BSP_DBG_TRACE_PRINTF( "\n start on rescue eeprom , NbOfReset = %d ", lr1_mac->nb_of_reset );
            BSP_DBG_TRACE_PRINTF( "\n start on rescue eeprom , Region = %d\n", lr1_mac->real->region_type );
            return OKLORAWAN;
        }
        else
        {  // == factory reset
            return ERRORLORAWAN;
        }
    }
}
void region_ww2g4_bad_crc_memory_set( lr1_stack_mac_t* lr1_mac )
{
    mac_context.devnonce    = lr1_mac->dev_nonce;
    mac_context.adr_custom  = lr1_mac->adr_custom;
    mac_context.nb_reset    = lr1_mac->nb_of_reset;
    mac_context.region_type = lr1_mac->real->region_type;
    memcpy( mac_context.appeui, lr1_mac->app_eui, 8 );
    memcpy( mac_context.deveui, lr1_mac->dev_eui, 8 );
    memcpy( mac_context.appkey, lr1_mac->app_key, 16 );
    mac_context.crc = lr1mac_utilities_crc( ( uint8_t* ) &mac_context, sizeof( mac_context ) - 4 ) + 1;
    bsp_nvm_context_store( BSP_LORAWAN_CONTEXT_ADDR_OFFSET, ( uint8_t* ) &( mac_context ), sizeof( mac_context ) );

    bsp_mcu_wait_us( 10000 );

    bsp_nvm_context_store( BSP_LORAWAN_CONTEXT_ADDR_OFFSET + sizeof( mac_context ) + 4, ( uint8_t* ) &( mac_context ),
                           sizeof( mac_context ) );
    bsp_mcu_wait_us( 10000 );
}
void region_ww2g4_memory_save( lr1_stack_mac_t* lr1_mac )
{
    mac_context.devnonce    = lr1_mac->dev_nonce;
    mac_context.adr_custom  = lr1_mac->adr_custom;
    mac_context.nb_reset    = lr1_mac->nb_of_reset;
    mac_context.region_type = lr1_mac->real->region_type;
    memcpy( mac_context.appeui, lr1_mac->app_eui, 8 );
    memcpy( mac_context.deveui, lr1_mac->dev_eui, 8 );
    memcpy( mac_context.appkey, lr1_mac->app_key, 16 );
    mac_context.crc = lr1mac_utilities_crc( ( uint8_t* ) &mac_context, sizeof( mac_context ) - 4 );
    bsp_nvm_context_store( BSP_LORAWAN_CONTEXT_ADDR_OFFSET, ( uint8_t* ) &( mac_context ), sizeof( mac_context ) );

    bsp_mcu_wait_us( 10000 );

    bsp_nvm_context_store( BSP_LORAWAN_CONTEXT_ADDR_OFFSET + sizeof( mac_context ) + 4, ( uint8_t* ) &( mac_context ),
                           sizeof( mac_context ) );
    bsp_mcu_wait_us( 10000 );
}
void region_ww2g4_dr_distribution_set( lr1_stack_mac_t* lr1_mac, uint8_t adr_mode )
{
    memset( dr_distribution_init, 0, 8 );
    switch( adr_mode )
    {
    case MOBILE_LONGRANGE_DR_DISTRIBUTION:  // in this example 4/7 dr0 2/7 dr1 and 1/7 dr2
        dr_distribution_init[7] = 0;
        dr_distribution_init[6] = 0;
        dr_distribution_init[5] = 0;
        dr_distribution_init[4] = 0;
        dr_distribution_init[3] = 0;
        dr_distribution_init[2] = 1;
        dr_distribution_init[1] = 2;
        dr_distribution_init[0] = 4;
        lr1_mac->nb_trans       = 1;
        break;
    case MOBILE_LOWPER_DR_DISTRIBUTION:  // in this example 5/10 dr5 4/10 dr4 and 1/10 dr0
        dr_distribution_init[7] = 0;
        dr_distribution_init[6] = 0;
        dr_distribution_init[5] = 0;
        dr_distribution_init[4] = 0;
        dr_distribution_init[3] = 1;
        dr_distribution_init[2] = 0;
        dr_distribution_init[1] = 0;
        dr_distribution_init[0] = 0;
        lr1_mac->nb_trans       = 1;
        break;
    case JOIN_DR_DISTRIBUTION:  // in this example 1/3 dr5 1/3 dr4 and 1/3 dr0
        dr_distribution_init[7] = 0;
        dr_distribution_init[6] = 0;
        dr_distribution_init[5] = 0;
        dr_distribution_init[4] = 0;
        dr_distribution_init[3] = 0;
        dr_distribution_init[2] = 0;
        dr_distribution_init[1] = 0;
        dr_distribution_init[0] = 1;
        lr1_mac->nb_trans       = 1;
        break;
    case USER_DR_DISTRIBUTION:  // in this example 1/3 dr5 1/3 dr4 and 1/3 dr0
        dr_distribution_init[7] = ( ( lr1_mac->adr_custom ) & ( 0x0000000F ) );
        dr_distribution_init[6] = ( ( lr1_mac->adr_custom ) & ( 0x000000F0 ) ) >> 4;
        dr_distribution_init[5] = ( ( lr1_mac->adr_custom ) & ( 0x00000F00 ) ) >> 8;
        dr_distribution_init[4] = ( ( lr1_mac->adr_custom ) & ( 0x0000F000 ) ) >> 12;
        dr_distribution_init[3] = ( ( lr1_mac->adr_custom ) & ( 0x000F0000 ) ) >> 16;
        dr_distribution_init[2] = ( ( lr1_mac->adr_custom ) & ( 0x00F00000 ) ) >> 20;
        dr_distribution_init[1] = ( ( lr1_mac->adr_custom ) & ( 0x0F000000 ) ) >> 24;
        dr_distribution_init[0] = ( ( lr1_mac->adr_custom ) & ( 0xF0000000 ) ) >> 28;
        lr1_mac->nb_trans       = BSP_USER_NUMBER_OF_RETRANSMISSION;
        break;
    default:
        dr_distribution_init[0] = 1;
        dr_distribution_init[1] = 0;
        dr_distribution_init[2] = 0;
        dr_distribution_init[3] = 0;
        dr_distribution_init[4] = 0;
        dr_distribution_init[5] = 0;
        dr_distribution_init[6] = 0;
        dr_distribution_init[7] = 0;
        lr1_mac->nb_trans       = 1;
        break;
    }
    memcpy( dr_distribution, dr_distribution_init, 8 );
}
status_lorawan_t region_ww2g4_join_next_channel_get( lr1_stack_mac_t* lr1_mac )
{
    return region_ww2g4_next_channel_get( lr1_mac );
}

status_lorawan_t region_ww2g4_next_channel_get( lr1_stack_mac_t* lr1_mac )
{
    uint8_t active_channel_nb = 0;
    uint8_t active_channel_index[NUMBER_OF_CHANNEL_WW2G4];
    for( uint8_t i = 0; i < NUMBER_OF_CHANNEL_WW2G4; i++ )
    {
        if( ( channel_index_enabled[i] == CHANNEL_ENABLED ) && ( min_dr_channel[i] <= lr1_mac->tx_data_rate_adr ) &&
            ( max_dr_channel[i] >= lr1_mac->tx_data_rate_adr ) )
        {
            active_channel_index[active_channel_nb] = i;
            active_channel_nb++;
        }
    }
    if( active_channel_nb == 0 )
    {
        BSP_DBG_TRACE_WARNING( "NO CHANNELS AVAILABLE \n" );
        return ERRORLORAWAN;
    }
    uint8_t temp        = ( bsp_rng_get_random_in_range( 0, ( active_channel_nb - 1 ) ) ) % active_channel_nb;
    uint8_t channel_idx = 0;
    channel_idx         = active_channel_index[temp];
    if( channel_idx >= NUMBER_OF_CHANNEL_WW2G4 )
    {
        BSP_DBG_TRACE_PRINTF( "INVALID CHANNEL  active channel = %d and random channel = %d \n", active_channel_nb,
                              temp );
        return ERRORLORAWAN;
    }
    else
    {
        lr1_mac->tx_frequency  = tx_frequency_channel[channel_idx];
        lr1_mac->rx1_frequency = rx1_frequency_channel[channel_idx];
    }
    return OKLORAWAN;
}

void region_ww2g4_next_dr_get( lr1_stack_mac_t* lr1_mac )
{
    if( lr1_mac->adr_mode_select == STATIC_ADR_MODE )
    {
        lr1_mac->tx_data_rate = lr1_mac->tx_data_rate_adr;
        lr1_mac->adr_enable   = 1;
    }
    else
    {
        uint8_t distri_sum = 0;
        for( uint8_t i = 0; i < 8; i++ )
        {
            distri_sum += dr_distribution[i];
        }
        if( distri_sum == 0 )
        {
            memcpy( dr_distribution, dr_distribution_init, 8 );
        }
        uint8_t new_dr;
        do
        {
            new_dr = ( bsp_rng_get_random_in_range( MIN_DR_WW2G4, MAX_DR_WW2G4 ) % ( MAX_DR_WW2G4 + 1 ) );
        } while( dr_distribution[new_dr] == 0 );

        lr1_mac->tx_data_rate = new_dr;
        dr_distribution[new_dr]--;
        lr1_mac->adr_enable = 0;
    }
    lr1_mac->tx_data_rate = ( lr1_mac->tx_data_rate > MAX_DR_WW2G4 ) ? MAX_DR_WW2G4 : lr1_mac->tx_data_rate;
    tx_dr_to_sf_bw( lr1_mac, lr1_mac->tx_data_rate );
}

uint8_t region_ww2g4_max_payload_size_get( uint8_t dr )
{
    uint8_t M[8] = { 59, 123, 228, 228, 228, 228, 228, 228 };
    return ( M[dr] );
}

uint32_t region_ww2g4_decode_freq_from_buf( uint8_t freq_buf[3] )
{
    uint32_t freq = ( freq_buf[0] ) + ( freq_buf[1] << 8 ) + ( freq_buf[2] << 16 );
    freq *= FREQUENCY_FACTOR_WW2G4;
    return freq;
}

void region_ww2g4_cflist_get( lr1_stack_mac_t* lr1_mac )
{
    if( lr1_mac->cf_list[15] == CF_LIST_FREQ )
    {
        for( uint8_t i = 0; i < 5; i++ )
        {
            tx_frequency_channel[3 + i]  = region_ww2g4_decode_freq_from_buf( &lr1_mac->cf_list[0 + ( 3 * i )] );
            rx1_frequency_channel[3 + i] = tx_frequency_channel[3 + i];

            if( region_ww2g4_is_valid_tx_frequency( tx_frequency_channel[3 + i] ) == OKLORAWAN &&
                tx_frequency_channel[3 + i] != 0 )
            {
                min_dr_channel[3 + i]        = 0;
                max_dr_channel[3 + i]        = 5;
                channel_index_enabled[3 + i] = CHANNEL_ENABLED;
                BSP_DBG_TRACE_PRINTF( " MacTxFrequency [%d] = %lu \n", i, tx_frequency_channel[3 + i] );
                BSP_DBG_TRACE_PRINTF( " MacMinDataRateChannel [%d] = %u \n", i, min_dr_channel[3 + i] );
                BSP_DBG_TRACE_PRINTF( " MacMaxDataRateChannel [%d] = %u \n", i, max_dr_channel[3 + i] );
                BSP_DBG_TRACE_PRINTF( " MacChannelIndexEnabled [%d] = %u \n", i, channel_index_enabled[3 + i] );
            }
            else
            {
                tx_frequency_channel[3 + i]  = 0;
                rx1_frequency_channel[3 + i] = 0;
                channel_index_enabled[3 + i] = CHANNEL_DISABLED;

                BSP_DBG_TRACE_WARNING( "INVALID TX FREQUENCY IN CFLIST OR CFLIST EMPTY \n" );
            }
        }
    }
    else
    {
        BSP_DBG_TRACE_WARNING( "INVALID CFLIST, MUST CONTAINS FREQ \n" );
    }
}

void region_ww2g4_rx_config_set( lr1_stack_mac_t* lr1_mac, rx_win_type_t type )
{
    if( type == RX1 )
    {
        lr1_mac->rx1_sf =
            ( lr1_mac->tx_sf < 12 - lr1_mac->rx1_dr_offset ) ? lr1_mac->tx_sf + lr1_mac->rx1_dr_offset : 12;
        lr1_mac->rx1_bw = lr1_mac->tx_bw;
    }
    else if( type == RX2 )
    {
        rx2_dr_to_sf_bw( lr1_mac, lr1_mac->rx2_data_rate );
    }
    else
    {
        BSP_DBG_TRACE_WARNING( "INVALID RX TYPE \n" );
    }
}
void region_ww2g4_power_set( lr1_stack_mac_t* lr1_mac, uint8_t power_cmd )
{
    if( power_cmd > 7 )
    {
        lr1_mac->tx_power = lr1_mac->max_eirp_dbm;  // Set by TxParamSetupReq
        BSP_DBG_TRACE_WARNING( "INVALID TX_POWER_WW2G4 \n" );
    }
    else
    {
        lr1_mac->tx_power = lr1_mac->max_eirp_dbm - ( 2 * power_cmd );
    }
}
void region_ww2g4_channel_mask_set( lr1_stack_mac_t* lr1_mac )
{
    for( uint8_t i = 0; i < NUMBER_OF_CHANNEL_WW2G4; i++ )
    {
        channel_index_enabled[i] = ( unwrapped_channel_mask >> i ) & 0x1;
        BSP_DBG_TRACE_PRINTF( " %d ", channel_index_enabled[i] );
    }
    BSP_DBG_TRACE_MSG( " \n" );
}
void region_ww2g4_channel_mask_init( void )
{
    unwrapped_channel_mask = 0xFFFF;
}

void region_ww2g4_join_snapshot_channel_mask_init( void )
{
    // Not useful for WWG4
    return;
}

status_channel_t region_ww2g4_channel_mask_build( uint8_t channel_mask_cntl, uint16_t channel_mask )
{
    status_channel_t status = OKCHANNEL;
    switch( channel_mask_cntl )
    {
    case 0:
        unwrapped_channel_mask = 0xFFFF;
        unwrapped_channel_mask = unwrapped_channel_mask & channel_mask;
        BSP_DBG_TRACE_PRINTF( "UnwrappedChannelMask = 0x%lx, ChMask = 0x%x\n", unwrapped_channel_mask, channel_mask );
        for( uint8_t i = 0; i < NUMBER_OF_CHANNEL_WW2G4; i++ )
        {
            if( ( ( ( unwrapped_channel_mask >> i ) & 0x1 ) == 1 ) && ( tx_frequency_channel[i] == 0 ) )
            {
                status = ERROR_CHANNEL_MASK;  // this status is used only for the last multiple link adr req
            }
        }
        break;
    case 6:
        unwrapped_channel_mask = 0;
        for( uint8_t i = 0; i < NUMBER_OF_CHANNEL_WW2G4; i++ )
        {
            if( tx_frequency_channel[i] > 0 )
            {
                unwrapped_channel_mask = unwrapped_channel_mask ^ ( 1 << i );
            }
        }
        break;
    default:
        status = ERROR_CHANNEL_CNTL;
        break;
    }
    if( unwrapped_channel_mask == 0 )
    {
        status = ERROR_CHANNEL_MASK;
    }
    return ( status );
}

void region_ww2g4_dr_decrement( lr1_stack_mac_t* lr1_mac )
{
    uint8_t valid_temp = 0;
    if( lr1_mac->tx_power < TX_POWER_WW2G4 )
    {
        lr1_mac->tx_power = TX_POWER_WW2G4;
    }
    while( ( lr1_mac->tx_data_rate_adr > 0 ) && ( valid_temp == 0 ) )
    {
        lr1_mac->tx_data_rate_adr--;
        for( uint8_t i = 0; i < NUMBER_OF_CHANNEL_WW2G4; i++ )
        {
            if( channel_index_enabled[i] == CHANNEL_ENABLED )
            {
                if( ( lr1_mac->tx_data_rate_adr <= max_dr_channel[i] ) &&
                    ( lr1_mac->tx_data_rate_adr >= min_dr_channel[i] ) )
                {
                    valid_temp++;
                }
            }
        }
    }
    // if adr DR = 0 enable the default channel
    if( valid_temp > 0 )
    {
        return;  // decrement at least one channel
    }
    // reach this step only if tx_dr = 0 and valid temp = 0 => enable default channel
    for( uint8_t i = 0; i < NUMBER_OF_CHANNEL_WW2G4; i++ )
    {
        if( tx_frequency_channel[i] != 0 && channel_index_enabled[i] == CHANNEL_DISABLED )
        {
            channel_index_enabled[i] = CHANNEL_ENABLED;
            min_dr_channel[i]        = 0;
            max_dr_channel[i]        = 5;
        }
    }
}
uint8_t region_ww2g4_adr_ack_delay_get( void )
{
    return ( ADR_ACK_DELAY_WW2G4 );
}
uint8_t region_ww2g4_adr_ack_limit_get( void )
{
    return ( ADR_ACK_LIMIT_WW2G4 );
}
uint8_t region_ww2g4_sync_word_get( void )
{
    return ( SYNC_WORD_WW2G4 );
}

void region_ww2g4_tx_frequency_channel_set( uint32_t tx_freq, uint8_t index )
{
    // the freq is multiply by 2 because the region 2.4 given freqency coded over 24 bits and divide by 200
    if( index >= NUMBER_OF_CHANNEL_WW2G4 )
    {
        BSP_DBG_TRACE_ERROR( " exceed Channel number available : overflow" );
        bsp_mcu_handle_lr1mac_issue( );
    }
    else
    {
        tx_frequency_channel[index] = tx_freq;
    }
}

void region_ww2g4_rx1_frequency_channel_set( uint32_t rx_freq, uint8_t index )
{
    if( index >= NUMBER_OF_CHANNEL_WW2G4 )
    {
        BSP_DBG_TRACE_ERROR( " exceed Channel number available : overflow" );
        bsp_mcu_handle_lr1mac_issue( );
    }
    else
    {
        rx1_frequency_channel[index] = rx_freq;
    }
}
void region_ww2g4_min_dr_channel_set( uint8_t dr, uint8_t index )
{
    if( index >= NUMBER_OF_CHANNEL_WW2G4 )
    {
        BSP_DBG_TRACE_ERROR( " exceed Channel number available : overflow" );
        bsp_mcu_handle_lr1mac_issue( );
    }
    else
    {
        min_dr_channel[index] = dr;
    }
}
void region_ww2g4_max_dr_channel_set( uint8_t dr, uint8_t index )
{
    if( index >= NUMBER_OF_CHANNEL_WW2G4 )
    {
        BSP_DBG_TRACE_ERROR( " exceed Channel number available : overflow" );
        bsp_mcu_handle_lr1mac_issue( );
    }
    else
    {
        max_dr_channel[index] = dr;
    }
}
void region_ww2g4_channel_enabled_set( uint8_t enable, uint8_t index )
{
    if( index >= NUMBER_OF_CHANNEL_WW2G4 )
    {
        BSP_DBG_TRACE_ERROR( " exceed Channel number available : overflow" );
        bsp_mcu_handle_lr1mac_issue( );
    }
    else
    {
        channel_index_enabled[index] = enable;
    }
}

uint32_t region_ww2g4_tx_frequency_channel_get( uint8_t index )
{
    if( index >= NUMBER_OF_CHANNEL_WW2G4 )
    {
        BSP_DBG_TRACE_ERROR( " exceed Channel number available : overflow" );
        bsp_mcu_handle_lr1mac_issue( );
    }
    return ( tx_frequency_channel[index] );
}
uint32_t region_ww2g4_rx1_frequency_channel_get( uint8_t index )
{
    if( index >= NUMBER_OF_CHANNEL_WW2G4 )
    {
        BSP_DBG_TRACE_ERROR( " exceed Channel number available : overflow" );
        bsp_mcu_handle_lr1mac_issue( );
    }
    return ( rx1_frequency_channel[index] );
}
uint8_t region_ww2g4_min_dr_channel_get( void )
{
    uint8_t min = MAX_DR_WW2G4;  // start with the max dr and search a dr inferior
    for( uint8_t i = 0; i < NUMBER_OF_CHANNEL_WW2G4; i++ )
    {
        if( ( min_dr_channel[i] < min ) && ( channel_index_enabled[i] == CHANNEL_ENABLED ) )
        {
            min = min_dr_channel[i];
        }
    }
    return ( min );
}
uint8_t region_ww2g4_max_dr_channel_get( void )
{
    uint8_t max = MIN_DR_WW2G4;  // start with the min dr and search a dr superior
    for( uint8_t i = 0; i < NUMBER_OF_CHANNEL_WW2G4; i++ )
        if( ( max_dr_channel[i] > max ) && ( channel_index_enabled[i] == CHANNEL_ENABLED ) )
        {
            max = max_dr_channel[i];
        }
    return ( max );
}
uint8_t region_ww2g4_channel_enabled_get( uint8_t index )
{
    if( index >= NUMBER_OF_CHANNEL_WW2G4 )
    {
        BSP_DBG_TRACE_ERROR( " exceed Channel number available : overflow" );
        bsp_mcu_handle_lr1mac_issue( );
    }
    return ( channel_index_enabled[index] );
}

/*************************************************************************/
/*                      Private region utilities implementation          */
/*************************************************************************/
void tx_dr_to_sf_bw( lr1_stack_mac_t* lr1_mac, uint8_t dr )
{
    lr1_mac->tx_modulation_type = LORA;
    if( dr < 8 )
    {
        lr1_mac->tx_sf = 12 - dr;
        lr1_mac->tx_bw = BW800;
    }
    else
    {
        lr1_mac->tx_sf = 12;
        lr1_mac->tx_bw = BW800;
        BSP_DBG_TRACE_WARNING( " Invalid Datarate \n" );
    }
}
void rx2_dr_to_sf_bw( lr1_stack_mac_t* lr1_mac, uint8_t dr )
{
    lr1_mac->rx2_modulation_type = LORA;
    if( dr < 8 )
    {
        lr1_mac->rx2_sf = 12 - dr;
        lr1_mac->rx2_bw = BW800;
    }
    else
    {
        lr1_mac->rx2_sf = 12;
        lr1_mac->rx2_bw = BW800;
        BSP_DBG_TRACE_WARNING( " Invalid Datarate \n" );
    }
}
/*deprecated
uint8_t channel_enabled_find(uint8_t index)
{
    uint8_t i = 0;
    uint8_t cpt = 0;
    for (i = 0; i < NUMBER_OF_CHANNEL_WW2G4; i++)
    {
        if (channel_index_enabled[i] == CHANNEL_ENABLED)
        {
            cpt++;
        }
        if (cpt == (index + 1))
        {
            return (i);
        }
    }
    return (-1); // for error case
}
*/
