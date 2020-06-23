/*!
 * \file      smtc_real.c
 *
 * \brief     Region Abstraction Layer (REAL) API implementation
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

#include "smtc_real.h"

#if defined( REGION_WW2G4 )
#include "region_ww2g4.h"
#endif
#if defined( REGION_EU_868 )
#include "region_eu_868.h"
#endif
#if defined( REGION_US_915 )
#include "region_us_915.h"
#endif
#if !defined( REGION_WW2G4 ) && !defined( REGION_EU_868 ) && !defined( REGION_US_915 )
#error "Unknown region selected..."
#endif

void smtc_real_init( lr1_stack_mac_t* lr1_mac )
{
    switch( lr1_mac->real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4: {
        region_ww2g4_init( lr1_mac );
        break;
    }
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        region_eu_868_init( lr1_mac );
        break;
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        region_us_915_init( lr1_mac );
        break;
    }
#endif
    default:
        BSP_DBG_TRACE_ERROR( "%s unsupported\n", __func__ );
        bsp_mcu_handle_lr1mac_issue( );
        break;
    }
}

void smtc_real_dr_distribution_set( lr1_stack_mac_t* lr1_mac, uint8_t adrMode )
{
    switch( lr1_mac->real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4: {
        region_ww2g4_dr_distribution_set( lr1_mac, adrMode );
        break;
    }
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        region_eu_868_dr_distribution_set( lr1_mac, adrMode );
        break;
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        region_us_915_dr_distribution_set( lr1_mac, adrMode );
        break;
    }
#endif
    default:
        BSP_DBG_TRACE_ERROR( "%s unsupported\n", __func__ );
        bsp_mcu_handle_lr1mac_issue( );
        break;
    }
}

status_lorawan_t smtc_real_memory_load( lr1_stack_mac_t* lr1_mac )
{
    switch( lr1_mac->real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4: {
        return region_ww2g4_memory_load( lr1_mac );
        break;
    }
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        return region_eu_868_memory_load( lr1_mac );
        break;
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        return region_us_915_memory_load( lr1_mac );
        break;
    }
#endif
    default:
        BSP_DBG_TRACE_ERROR( "%s unsupported\n", __func__ );
        bsp_mcu_handle_lr1mac_issue( );
        return ( ERRORLORAWAN );  // never reach just for warning
        break;
    }
}

void smtc_real_bad_crc_memory_set( lr1_stack_mac_t* lr1_mac )
{
    switch( lr1_mac->real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4: {
        region_ww2g4_bad_crc_memory_set( lr1_mac );
        break;
    }
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        region_eu_868_bad_crc_memory_set( lr1_mac );
        break;
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        region_us_915_bad_crc_memory_set( lr1_mac );
        break;
    }
#endif
    default:
        BSP_DBG_TRACE_ERROR( "%s unsupported\n", __func__ );
        bsp_mcu_handle_lr1mac_issue( );
        break;
    }
}

void smtc_real_next_dr_get( lr1_stack_mac_t* lr1_mac )
{
    switch( lr1_mac->real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4: {
        region_ww2g4_next_dr_get( lr1_mac );
        break;
    }
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        region_eu_868_next_dr_get( lr1_mac );
        break;
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        region_us_915_next_dr_get( lr1_mac );
        break;
    }
#endif
    default:
        BSP_DBG_TRACE_ERROR( "%s unsupported\n", __func__ );
        bsp_mcu_handle_lr1mac_issue( );
        break;
    }
}

void smtc_real_memory_save( lr1_stack_mac_t* lr1_mac )
{
    switch( lr1_mac->real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4: {
        region_ww2g4_memory_save( lr1_mac );
        break;
    }
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        region_eu_868_memory_save( lr1_mac );
        break;
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        region_us_915_memory_save( lr1_mac );
        break;
    }
#endif
    default:
        BSP_DBG_TRACE_ERROR( "%s unsupported\n", __func__ );
        bsp_mcu_handle_lr1mac_issue( );
        break;
    }
}

uint8_t smtc_real_max_payload_size_get( lr1_stack_mac_t* lr1_mac, uint8_t dr )
{
    switch( lr1_mac->real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4: {
        return region_ww2g4_max_payload_size_get( dr );
    }
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        return region_eu_868_max_payload_size_get( dr );
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        return region_us_915_max_payload_size_get( dr );
    }
#endif
    default:
        BSP_DBG_TRACE_ERROR( "%s unsupported\n", __func__ );
        bsp_mcu_handle_lr1mac_issue( );
        break;
    }
    return 0;  // never reach => avoid warning
}

uint32_t smtc_real_decode_freq_from_buf( lr1_stack_mac_t* lr1_mac, uint8_t freq_buf[3] )
{
    switch( lr1_mac->real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4: {
        return region_ww2g4_decode_freq_from_buf( freq_buf );
    }
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        return region_eu_868_decode_freq_from_buf( freq_buf );
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        return region_us_915_decode_freq_from_buf( freq_buf );
    }
#endif
    default:
        BSP_DBG_TRACE_ERROR( "%s unsupported\n", __func__ );
        bsp_mcu_handle_lr1mac_issue( );
        break;
    }
    return 0;  // never reach => avoid warning
}

void smtc_real_cflist_get( lr1_stack_mac_t* lr1_mac )
{
    switch( lr1_mac->real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4: {
        region_ww2g4_cflist_get( lr1_mac );
        break;
    }
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        region_eu_868_cflist_get( lr1_mac );
        break;
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        region_us_915_cflist_get( lr1_mac );
        break;
    }
#endif
    default:
        BSP_DBG_TRACE_ERROR( "%s unsupported\n", __func__ );
        bsp_mcu_handle_lr1mac_issue( );
        break;
    }
}

status_lorawan_t smtc_real_next_channel_get( lr1_stack_mac_t* lr1_mac )
{
    switch( lr1_mac->real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4: {
        return region_ww2g4_next_channel_get( lr1_mac );
    }
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        return region_eu_868_next_channel_get( lr1_mac );
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        return region_us_915_next_channel_get( lr1_mac );
    }
#endif
    default:
        BSP_DBG_TRACE_ERROR( "%s unsupported\n", __func__ );
        bsp_mcu_handle_lr1mac_issue( );
        break;
    }
    return ERRORLORAWAN;  // never reach => avoid warning
}

status_lorawan_t smtc_real_join_next_channel_get( lr1_stack_mac_t* lr1_mac )
{
    switch( lr1_mac->real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4: {
        return region_ww2g4_join_next_channel_get( lr1_mac );
    }
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        return region_eu_868_join_next_channel_get( lr1_mac );
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        return region_us_915_join_next_channel_get( lr1_mac );
    }
#endif
    default:
        BSP_DBG_TRACE_ERROR( "%s unsupported\n", __func__ );
        bsp_mcu_handle_lr1mac_issue( );
        break;
    }
    return ERRORLORAWAN;  // never reach => avoid warning
}

void smtc_real_rx_config_set( lr1_stack_mac_t* lr1_mac, rx_win_type_t type )
{
    switch( lr1_mac->real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4: {
        region_ww2g4_rx_config_set( lr1_mac, type );
        break;
    }
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        region_eu_868_rx_config_set( lr1_mac, type );
        break;
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        region_us_915_rx_config_set( lr1_mac, type );
        break;
    }
#endif
    default:
        BSP_DBG_TRACE_ERROR( "%s unsupported\n", __func__ );
        bsp_mcu_handle_lr1mac_issue( );
        break;
    }
}

void smtc_real_power_set( lr1_stack_mac_t* lr1_mac, uint8_t power_cmd )
{
    switch( lr1_mac->real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4: {
        region_ww2g4_power_set( lr1_mac, power_cmd );
        break;
    }
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        region_eu_868_power_set( lr1_mac, power_cmd );
        break;
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        region_us_915_power_set( lr1_mac, power_cmd );
        break;
    }
#endif
    default:
        BSP_DBG_TRACE_ERROR( "%s unsupported\n", __func__ );
        bsp_mcu_handle_lr1mac_issue( );
        break;
    }
}
uint8_t smtc_real_default_max_eirp_get( lr1_stack_mac_t* lr1_mac )
{
    switch( lr1_mac->real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4: {
        return TX_POWER_WW2G4;
    }
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        return TX_POWER_EU_868;
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        return TX_POWER_US_915;
    }
#endif
    default:
        BSP_DBG_TRACE_ERROR( "%s unsupported\n", __func__ );
        bsp_mcu_handle_lr1mac_issue( );
        break;
    }
    return ERRORLORAWAN;  // never reach => avoid warning
}

void smtc_real_channel_mask_set( lr1_stack_mac_t* lr1_mac )
{
    switch( lr1_mac->real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4: {
        region_ww2g4_channel_mask_set( lr1_mac );
        break;
    }
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        region_eu_868_channel_mask_set( lr1_mac );
        break;
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        region_us915_channel_mask_set( lr1_mac );
        break;
    }
#endif
    default:
        BSP_DBG_TRACE_ERROR( "%s unsupported\n", __func__ );
        bsp_mcu_handle_lr1mac_issue( );
        break;
    }
}

void smtc_real_channel_mask_init( lr1_stack_mac_t* lr1_mac )
{
    switch( lr1_mac->real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4: {
        region_ww2g4_channel_mask_init( );
        break;
    }
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        region_eu_868_channel_mask_init( );
        break;
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        region_us_915_channel_mask_init( );
        break;
    }
#endif
    default:
        BSP_DBG_TRACE_ERROR( "%s unsupported\n", __func__ );
        bsp_mcu_handle_lr1mac_issue( );
        break;
    }
}

void smtc_real_join_snapshot_channel_mask_init( lr1_stack_mac_t* lr1_mac )
{
    switch( lr1_mac->real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4: {
        region_ww2g4_join_snapshot_channel_mask_init( );
        break;
    }
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        region_eu_868_join_snapshot_channel_mask_init( );
        break;
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        region_us_915_join_snapshot_channel_mask_init( );
        break;
    }
#endif
    default:
        BSP_DBG_TRACE_ERROR( "%s unsupported\n", __func__ );
        bsp_mcu_handle_lr1mac_issue( );
        break;
    }
}

status_channel_t smtc_real_channel_mask_build( lr1_stack_mac_t* lr1_mac, uint8_t ChMaskCntl, uint16_t ChMask )
{
    switch( lr1_mac->real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4: {
        return region_ww2g4_channel_mask_build( ChMaskCntl, ChMask );
    }
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        return region_eu_868_channel_mask_build( ChMaskCntl, ChMask );
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        return region_us_915_channel_mask_build( ChMaskCntl, ChMask );
    }
#endif
    default:
        BSP_DBG_TRACE_ERROR( "%s unsupported\n", __func__ );
        bsp_mcu_handle_lr1mac_issue( );
        break;
    }
    return ERRORLORAWAN;  // never reach => avoid warning
}

void smtc_real_dr_decrement( lr1_stack_mac_t* lr1_mac )
{
    switch( lr1_mac->real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4: {
        region_ww2g4_dr_decrement( lr1_mac );
        break;
    }
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        region_eu_868_dr_decrement( lr1_mac );
        break;
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        region_us_915_dr_decrement( lr1_mac );
        break;
    }
#endif
    default:
        BSP_DBG_TRACE_ERROR( "%s unsupported\n", __func__ );
        bsp_mcu_handle_lr1mac_issue( );
        break;
    }
}

uint8_t smtc_real_adr_ack_delay_get( lr1_stack_mac_t* lr1_mac )
{
    switch( lr1_mac->real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4: {
        return region_ww2g4_adr_ack_delay_get( );
    }
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        return region_eu_868_adr_ack_delay_get( );
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        return region_us_915_adr_ack_delay_get( );
    }
#endif
    default:
        BSP_DBG_TRACE_ERROR( "%s unsupported\n", __func__ );
        bsp_mcu_handle_lr1mac_issue( );
        break;
    }
    return ERRORLORAWAN;  // never reach => avoid warning
}

uint8_t smtc_real_adr_ack_limit_get( lr1_stack_mac_t* lr1_mac )
{
    switch( lr1_mac->real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4: {
        return region_ww2g4_adr_ack_limit_get( );
    }
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        return region_eu_868_adr_ack_limit_get( );
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        return region_us_915_adr_ack_limit_get( );
    }
#endif
    default:
        BSP_DBG_TRACE_ERROR( "%s unsupported\n", __func__ );
        bsp_mcu_handle_lr1mac_issue( );
        break;
    }
    return ERRORLORAWAN;  // never reach => avoid warning
}

uint8_t smtc_real_sync_word_get( lr1_stack_mac_t* lr1_mac )
{
    switch( lr1_mac->real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4: {
        return region_ww2g4_sync_word_get( );
    }
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        return region_eu_868_sync_word_get( );
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        return region_us_915_sync_word_get( );
    }
#endif
    default:
        BSP_DBG_TRACE_ERROR( "%s unsupported\n", __func__ );
        bsp_mcu_handle_lr1mac_issue( );
        break;
    }
    return ERRORLORAWAN;  // never reach => avoid warning
}

uint8_t* smtc_real_gfsk_sync_word_get( lr1_stack_mac_t* lr1_mac )
{
    switch( lr1_mac->real->region_type )
    {
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        return region_eu_868_gfsk_sync_word_get( );
    }
#endif
    default:
        BSP_DBG_TRACE_ERROR( "%s unsupported\n", __func__ );
        bsp_mcu_handle_lr1mac_issue( );
        break;
    }
    return 0;  // never reach => avoid warning
}

status_lorawan_t smtc_real_is_valid_rx1_dr_offset( lr1_stack_mac_t* lr1_mac, uint8_t rx1_dr_offset )
{
    switch( lr1_mac->real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4: {
        return region_ww2g4_is_valid_rx1_dr_offset( rx1_dr_offset );
    }
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        return region_eu_868_is_valid_rx1_dr_offset( rx1_dr_offset );
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        return region_us_915_is_valid_rx1_dr_offset( rx1_dr_offset );
    }
#endif
    default:
        BSP_DBG_TRACE_ERROR( "%s unsupported\n", __func__ );
        bsp_mcu_handle_lr1mac_issue( );
        break;
    }
    return ERRORLORAWAN;  // never reach => avoid warning
}

status_lorawan_t smtc_real_is_valid_dr( lr1_stack_mac_t* lr1_mac, uint8_t dr )
{
    switch( lr1_mac->real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4: {
        return region_ww2g4_is_valid_dr( dr );
    }
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        return region_eu_868_is_valid_dr( dr );
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        return region_us_915_is_valid_dr( dr );
    }
#endif
    default:
        BSP_DBG_TRACE_ERROR( "%s unsupported\n", __func__ );
        bsp_mcu_handle_lr1mac_issue( );
        break;
    }
    return ERRORLORAWAN;  // never reach => avoid warning
}

status_lorawan_t smtc_real_is_acceptable_dr( lr1_stack_mac_t* lr1_mac, uint8_t dr )
{
    switch( lr1_mac->real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4: {
        return region_ww2g4_is_acceptable_dr( dr );
    }
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        return region_eu_868_is_acceptable_dr( dr );
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        return region_us_915_is_acceptable_dr( dr );
    }
#endif
    default:
        BSP_DBG_TRACE_ERROR( "%s unsupported\n", __func__ );
        bsp_mcu_handle_lr1mac_issue( );
        break;
    }
    return ERRORLORAWAN;  // never reach => avoid warning
}

status_lorawan_t smtc_real_is_valid_tx_frequency( lr1_stack_mac_t* lr1_mac, uint32_t frequency )
{
    switch( lr1_mac->real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4: {
        return region_ww2g4_is_valid_tx_frequency( frequency );
    }
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        return region_eu_868_is_valid_tx_frequency( frequency );
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        return region_us_915_is_valid_tx_frequency( frequency );
    }
#endif
    default:
        BSP_DBG_TRACE_ERROR( "%s unsupported\n", __func__ );
        bsp_mcu_handle_lr1mac_issue( );
        break;
    }
    return ERRORLORAWAN;  // never reach => avoid warning
}

status_lorawan_t smtc_real_is_valid_rx_frequency( lr1_stack_mac_t* lr1_mac, uint32_t frequency )
{
    switch( lr1_mac->real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4: {
        return region_ww2g4_is_valid_rx_frequency( frequency );
    }
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        return region_eu_868_is_valid_rx_frequency( frequency );
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        return region_us_915_is_valid_rx_frequency( frequency );
    }
#endif
    default:
        BSP_DBG_TRACE_ERROR( "%s unsupported\n", __func__ );
        bsp_mcu_handle_lr1mac_issue( );
        break;
    }
    return ERRORLORAWAN;  // never reach => avoid warning
}

status_lorawan_t smtc_real_is_valid_tx_power( lr1_stack_mac_t* lr1_mac, uint8_t power )
{
    switch( lr1_mac->real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4: {
        return region_ww2g4_is_valid_tx_power( power );
    }
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        return region_eu_868_is_valid_tx_power( power );
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        return region_us_915_is_valid_tx_power( power );
    }
#endif
    default:
        BSP_DBG_TRACE_ERROR( "%s unsupported\n", __func__ );
        bsp_mcu_handle_lr1mac_issue( );
        break;
    }
    return ERRORLORAWAN;  // never reach => avoid warning
}

status_lorawan_t smtc_real_is_valid_channel_index( lr1_stack_mac_t* lr1_mac, uint8_t channel_index )
{
    switch( lr1_mac->real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4: {
        return region_ww2g4_is_valid_channel_index( channel_index );
    }
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        return region_eu_868_is_valid_channel_index( channel_index );
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        return region_us_915_is_valid_channel_index( channel_index );
    }
#endif
    default:
        BSP_DBG_TRACE_ERROR( "%s unsupported\n", __func__ );
        bsp_mcu_handle_lr1mac_issue( );
        break;
    }
    return ERRORLORAWAN;  // never reach => avoid warning
}

status_lorawan_t smtc_real_is_valid_size( lr1_stack_mac_t* lr1_mac, uint8_t dr, uint8_t size )
{
    switch( lr1_mac->real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4: {
        return region_ww2g4_is_valid_size( lr1_mac, dr, size );
    }
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        return region_eu_868_is_valid_size( lr1_mac, dr, size );
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        return region_us_915_is_valid_size( lr1_mac, dr, size );
    }
#endif
    default:
        BSP_DBG_TRACE_ERROR( "%s unsupported\n", __func__ );
        bsp_mcu_handle_lr1mac_issue( );
        break;
    }
    return ERRORLORAWAN;  // never reach => avoid warning
}

void smtc_real_tx_frequency_channel_set( lr1_stack_mac_t* lr1_mac, uint32_t tx_freq, uint8_t index )
{
    switch( lr1_mac->real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4: {
        region_ww2g4_tx_frequency_channel_set( tx_freq, index );
        break;
    }
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        region_eu_868_tx_frequency_channel_set( tx_freq, index );
        break;
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        region_us_915_tx_frequency_channel_set( tx_freq, index );
        break;
    }
#endif
    default:
        BSP_DBG_TRACE_ERROR( "%s unsupported\n", __func__ );
        bsp_mcu_handle_lr1mac_issue( );
        break;
    }
}

void smtc_real_rx1_frequency_channel_set( lr1_stack_mac_t* lr1_mac, uint32_t rx_freq, uint8_t index )
{
    switch( lr1_mac->real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4: {
        region_ww2g4_rx1_frequency_channel_set( rx_freq, index );
        break;
    }
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        region_eu_868_rx1_frequency_channel_set( rx_freq, index );
        break;
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        region_us_915_rx1_frequency_channel_set( rx_freq, index );
        break;
    }
#endif
    default:
        BSP_DBG_TRACE_ERROR( "%s unsupported\n", __func__ );
        bsp_mcu_handle_lr1mac_issue( );
        break;
    }
}

void smtc_real_min_dr_channel_set( lr1_stack_mac_t* lr1_mac, uint8_t dr, uint8_t index )
{
    switch( lr1_mac->real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4: {
        region_ww2g4_min_dr_channel_set( dr, index );
        break;
    }
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        region_eu_868_min_dr_channel_set( dr, index );
        break;
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        region_us_915_min_dr_channel_set( dr, index );
        break;
    }
#endif
    default:
        BSP_DBG_TRACE_ERROR( "%s unsupported\n", __func__ );
        bsp_mcu_handle_lr1mac_issue( );
        break;
    }
}

void smtc_real_max_dr_channel_set( lr1_stack_mac_t* lr1_mac, uint8_t dr, uint8_t index )
{
    switch( lr1_mac->real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4: {
        region_ww2g4_max_dr_channel_set( dr, index );
        break;
    }
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        region_eu_868_max_dr_channel_set( dr, index );
        break;
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        region_us_915_max_dr_channel_set( dr, index );
        break;
    }
#endif
    default:
        BSP_DBG_TRACE_ERROR( "%s unsupported\n", __func__ );
        bsp_mcu_handle_lr1mac_issue( );
        break;
    }
}

void smtc_real_channel_enabled_set( lr1_stack_mac_t* lr1_mac, uint8_t enable, uint8_t index )
{
    switch( lr1_mac->real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4: {
        region_ww2g4_channel_enabled_set( enable, index );
        break;
    }
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        region_eu_868_channel_enabled_set( enable, index );
        break;
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        region_us_915_channel_enabled_set( enable, index );
        break;
    }
#endif
    default:
        BSP_DBG_TRACE_ERROR( "%s unsupported\n", __func__ );
        bsp_mcu_handle_lr1mac_issue( );
        break;
    }
}

uint32_t smtc_real_tx_frequency_channel_get( lr1_stack_mac_t* lr1_mac, uint8_t index )
{
    switch( lr1_mac->real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4: {
        return region_ww2g4_tx_frequency_channel_get( index );
    }
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        return region_eu_868_tx_frequency_channel_get( index );
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        return region_us_915_tx_frequency_channel_get( index );
    }
#endif
    default:
        BSP_DBG_TRACE_ERROR( "%s unsupported\n", __func__ );
        bsp_mcu_handle_lr1mac_issue( );
        break;
    }
    return 0;  // never reach => avoid warning
}

uint32_t smtc_real_rx1_frequency_channel_get( lr1_stack_mac_t* lr1_mac, uint8_t index )
{
    switch( lr1_mac->real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4: {
        return region_ww2g4_rx1_frequency_channel_get( index );
    }
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        return region_eu_868_rx1_frequency_channel_get( index );
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        return region_us_915_rx1_frequency_channel_get( index );
    }
#endif
    default:
        BSP_DBG_TRACE_ERROR( "%s unsupported\n", __func__ );
        bsp_mcu_handle_lr1mac_issue( );
        break;
    }
    return 0;  // never reach => avoid warning
}

uint8_t smtc_real_min_dr_channel_get( lr1_stack_mac_t* lr1_mac )
{
    switch( lr1_mac->real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4: {
        return region_ww2g4_min_dr_channel_get( );
    }
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        return region_eu_868_min_dr_channel_get( );
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        return region_us_915_min_dr_channel_get( );
    }
#endif
    default:
        BSP_DBG_TRACE_ERROR( "%s unsupported\n", __func__ );
        bsp_mcu_handle_lr1mac_issue( );
        break;
    }
    return 0;  // never reach => avoid warning
}

uint8_t smtc_real_max_dr_channel_get( lr1_stack_mac_t* lr1_mac )
{
    switch( lr1_mac->real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4: {
        return region_ww2g4_max_dr_channel_get( );
    }
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        return region_eu_868_max_dr_channel_get( );
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        return region_us_915_max_dr_channel_get( );
    }
#endif
    default:
        BSP_DBG_TRACE_ERROR( "%s unsupported\n", __func__ );
        bsp_mcu_handle_lr1mac_issue( );
        break;
    }
    return 0;  // never reach => avoid warning
}

uint8_t smtc_real_channel_enabled_get( lr1_stack_mac_t* lr1_mac, uint8_t index )
{
    switch( lr1_mac->real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4: {
        return region_ww2g4_channel_enabled_get( index );
    }
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        return region_eu_868_channel_enabled_get( index );
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        return region_us_915_channel_enabled_get( index );
    }
#endif
    default:
        BSP_DBG_TRACE_ERROR( "%s unsupported\n", __func__ );
        bsp_mcu_handle_lr1mac_issue( );
        break;
    }
    return 0;  // never reach => avoid warning
}

uint8_t smtc_real_rx1_join_delay_get( lr1_stack_mac_t* lr1_mac )
{
    switch( lr1_mac->real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4: {
        return JOIN_ACCEPT_DELAY1_WW2G4;
    }
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        return JOIN_ACCEPT_DELAY1_EU_868;
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        return JOIN_ACCEPT_DELAY1_US_915;
    }
#endif
    default:
        BSP_DBG_TRACE_ERROR( "%s unsupported\n", __func__ );
        bsp_mcu_handle_lr1mac_issue( );
        break;
    }
    return 0;  // never reach => avoid warning
}

uint8_t smtc_real_rx2_join_dr_get( lr1_stack_mac_t* lr1_mac )
{
    switch( lr1_mac->real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4: {
        return RX2DR_INIT_WW2G4;
    }
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        return RX2DR_INIT_EU_868;
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        return RX2DR_INIT_US_915;
    }
#endif
    default:
        BSP_DBG_TRACE_ERROR( "%s unsupported\n", __func__ );
        bsp_mcu_handle_lr1mac_issue( );
        break;
    }
    return 0;  // never reach => avoid warning
}

uint8_t smtc_real_frequency_factor_get( lr1_stack_mac_t* lr1_mac )
{
    switch( lr1_mac->real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4: {
        return FREQUENCY_FACTOR_WW2G4;
    }
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        return FREQUENCY_FACTOR_EU_868;
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        return FREQUENCY_FACTOR_US_915;
    }
#endif
    default:
        BSP_DBG_TRACE_ERROR( "%s unsupported\n", __func__ );
        bsp_mcu_handle_lr1mac_issue( );
        break;
    }
    return 0;  // never reach => avoid warning
}
enum ral_lora_cr_e smtc_real_coding_rate_get( lr1_stack_mac_t* lr1_mac )
{
    switch( lr1_mac->real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4: {
        return RAL_LORA_CR_LI_4_8;
    }
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        return RAL_LORA_CR_4_5;
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        return RAL_LORA_CR_4_5;
    }
#endif
    default:
        BSP_DBG_TRACE_ERROR( "%s unsupported\n", __func__ );
        bsp_mcu_handle_lr1mac_issue( );
        break;
    }
    return 0;  // never reach => avoid warning
}

uint32_t smtc_real_get_join_sf5_toa_in_ms( const lr1_stack_mac_t* lr1_mac )
{
    switch( lr1_mac->real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4: {
        return TIMEONAIR_JOIN_SF5_MS_WW2G4;
    }
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        return TIMEONAIR_JOIN_SF5_MS_868;
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        return TIMEONAIR_JOIN_SF5_MS_915;
    }
#endif
    default:
        BSP_DBG_TRACE_ERROR( "%s unsupported\n", __func__ );
        bsp_mcu_handle_lr1mac_issue( );
        break;
    }
    return false;  // never reach => avoid warning
}

void smtc_real_duty_cycle_enable_set( const lr1_stack_mac_t* lr1_mac, uint8_t enable )
{
    switch( lr1_mac->real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4: {
        // No Duty Cycle in WW2G4
        break;
    }
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        region_eu_868_duty_cycle_enable_set( enable );
        break;
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        // No Duty Cycle in US_915
        break;
    }
#endif
    default:
        BSP_DBG_TRACE_ERROR( "%s unsupported\n", __func__ );
        bsp_mcu_handle_lr1mac_issue( );
        break;
    }
}

void smtc_real_duty_cycle_sum( const lr1_stack_mac_t* lr1_mac, uint32_t freq_hz, uint32_t toa_ms )
{
    switch( lr1_mac->real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4: {
        // No Duty Cycle in WW2G4
        break;
    }
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        region_eu_868_duty_cycle_sum( freq_hz, toa_ms );
        break;
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        // No Duty Cycle in US_915
        break;
    }
#endif
    default:
        BSP_DBG_TRACE_ERROR( "%s unsupported\n", __func__ );
        bsp_mcu_handle_lr1mac_issue( );
        break;
    }
}

void smtc_real_duty_cycle_update( const lr1_stack_mac_t* lr1_mac )
{
    switch( lr1_mac->real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4: {
        // No Duty Cycle in WW2G4
        break;
    }
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        region_eu_868_duty_cycle_update( );
        break;
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        // No Duty Cycle in US_915
        break;
    }
#endif
    default:
        BSP_DBG_TRACE_ERROR( "%s unsupported\n", __func__ );
        bsp_mcu_handle_lr1mac_issue( );
        break;
    }
}

status_lorawan_t smtc_real_duty_cycle_is_toa_accepted( const lr1_stack_mac_t* lr1_mac, uint32_t freq_hz,
                                                       uint32_t toa_ms )
{
    switch( lr1_mac->real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4: {
        // No Duty Cycle in WW2G4
        return OKLORAWAN;
    }
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        return region_eu_868_duty_cycle_is_toa_accepted( freq_hz, toa_ms );
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        // No Duty Cycle in US_915
        return OKLORAWAN;
    }
#endif
    default:
        BSP_DBG_TRACE_ERROR( "%s unsupported\n", __func__ );
        bsp_mcu_handle_lr1mac_issue( );
        break;
    }
    return ERRORLORAWAN;  // never reach => avoid warning
}

status_lorawan_t smtc_real_duty_cycle_is_channel_free( const lr1_stack_mac_t* lr1_mac, uint32_t freq_hz )
{
    switch( lr1_mac->real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4: {
        // No Duty Cycle in WW2G4
        return OKLORAWAN;
    }
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        return region_eu_868_duty_cycle_is_channel_free( freq_hz );
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        // No Duty Cycle in US_915
        return OKLORAWAN;
    }
#endif
    default:
        BSP_DBG_TRACE_ERROR( "%s unsupported\n", __func__ );
        bsp_mcu_handle_lr1mac_issue( );
        break;
    }
    return ERRORLORAWAN;  // never reach => avoid warning
}

int32_t smtc_real_next_free_duty_cycle_ms_get( const lr1_stack_mac_t* lr1_mac )
{
    switch( lr1_mac->real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4: {
        // No Duty Cycle in WW2G4
        return 0;
    }
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868: {
        return region_eu_868_next_free_duty_cycle_ms_get( );
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915: {
        // No Duty Cycle in US_915
        return 0;
    }
#endif
    default:
        BSP_DBG_TRACE_ERROR( "%s unsupported\n", __func__ );
        bsp_mcu_handle_lr1mac_issue( );
        break;
    }
    return 0;  // never reach => avoid warning
}


uint8_t  smtc_real_preamble_get( const lr1_stack_mac_t* lr1_mac, uint8_t sf ){
     switch( lr1_mac->real->region_type )
    {
#if defined( REGION_WW2G4 )
    case SMTC_REAL_REGION_WW2G4:
    {
        if ( ( sf == 5 )  || ( sf == 6 ) ) {
            return 12;
        }
        else if ( ( sf >= 7 )  && ( sf <= 12 ) ){
            return 8;
        }
        else {
            BSP_DBG_TRACE_ERROR( "%s unsupported sf \n", __func__ );
            bsp_mcu_handle_lr1mac_issue( );
            break;
        }
    }
#endif
#if defined( REGION_EU_868 )
    case SMTC_REAL_REGION_EU_868:
    {
        return 8;
    }
#endif
#if defined( REGION_US_915 )
    case SMTC_REAL_REGION_US_915:
    {
        return 8;
    }
#endif
    default:
        BSP_DBG_TRACE_ERROR( "%s unsupported\n", __func__ );
        bsp_mcu_handle_lr1mac_issue( );
        break;
    }
    return 0;  // never reach => avoid warning
}
