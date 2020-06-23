/*!
 * \file      smtc_real.h
 *
 * \brief     Region Abstraction Layer (REAL) API definition
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

#ifndef __REAL_H__
#define __REAL_H__

#include <stdint.h>
#include <stdbool.h>

#include "smtc_real_defs.h"
#include "lr1_stack_mac_layer.h"
#include "lr1mac_defs.h"

#ifdef __cplusplus
extern "C" {
#endif
/*
 * ============================================================================
 * API definitions
 * ============================================================================
 */
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void smtc_real_init( lr1_stack_mac_t* lr1_mac );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void smtc_real_dr_distribution_set( lr1_stack_mac_t* lr1_mac, uint8_t adrMode );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
status_lorawan_t smtc_real_memory_load( lr1_stack_mac_t* lr1_mac );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void smtc_real_bad_crc_memory_set( lr1_stack_mac_t* lr1_mac );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void smtc_real_next_dr_get( lr1_stack_mac_t* lr1_mac );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void smtc_real_memory_save( lr1_stack_mac_t* lr1_mac );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
uint8_t smtc_real_max_payload_size_get( lr1_stack_mac_t* lr1_mac, uint8_t dr );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
uint32_t smtc_real_decode_freq_from_buf( lr1_stack_mac_t* lr1_mac, uint8_t freq_buf[3] );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void smtc_real_cflist_get( lr1_stack_mac_t* lr1_mac );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
status_lorawan_t smtc_real_next_channel_get( lr1_stack_mac_t* lr1_mac );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
status_lorawan_t smtc_real_join_next_channel_get( lr1_stack_mac_t* lr1_mac );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void smtc_real_rx_config_set( lr1_stack_mac_t* lr1_mac, rx_win_type_t type );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void smtc_real_power_set( lr1_stack_mac_t* lr1_mac, uint8_t power_cmd );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
uint8_t smtc_real_default_max_eirp_get( lr1_stack_mac_t* lr1_mac );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void smtc_real_channel_mask_set( lr1_stack_mac_t* lr1_mac );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void smtc_real_channel_mask_init( lr1_stack_mac_t* lr1_mac );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void smtc_real_join_snapshot_channel_mask_init( lr1_stack_mac_t* lr1_mac );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
status_channel_t smtc_real_channel_mask_build( lr1_stack_mac_t* lr1_mac, uint8_t ChMaskCntl, uint16_t ChMask );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void smtc_real_dr_decrement( lr1_stack_mac_t* lr1_mac );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
uint8_t smtc_real_adr_ack_delay_get( lr1_stack_mac_t* lr1_mac );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
uint8_t smtc_real_adr_ack_limit_get( lr1_stack_mac_t* lr1_mac );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
uint8_t smtc_real_sync_word_get( lr1_stack_mac_t* lr1_mac );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
uint8_t* smtc_real_gfsk_sync_word_get( lr1_stack_mac_t* lr1_mac );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
status_lorawan_t smtc_real_is_valid_rx1_dr_offset( lr1_stack_mac_t* lr1_mac, uint8_t rx1_dr_offset );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
status_lorawan_t smtc_real_is_valid_dr( lr1_stack_mac_t* lr1_mac, uint8_t dr );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
status_lorawan_t smtc_real_is_acceptable_dr( lr1_stack_mac_t* lr1_mac, uint8_t dr );  // for link adr cmd
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
status_lorawan_t smtc_real_is_valid_tx_frequency( lr1_stack_mac_t* lr1_mac, uint32_t frequency );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
status_lorawan_t smtc_real_is_valid_rx_frequency( lr1_stack_mac_t* lr1_mac, uint32_t frequency );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
status_lorawan_t smtc_real_is_valid_tx_power( lr1_stack_mac_t* lr1_mac, uint8_t power );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
status_lorawan_t smtc_real_is_valid_channel_index( lr1_stack_mac_t* lr1_mac, uint8_t channel_index );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
status_lorawan_t smtc_real_is_valid_size( lr1_stack_mac_t* lr1_mac, uint8_t dr, uint8_t size );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void smtc_real_tx_frequency_channel_set( lr1_stack_mac_t* lr1_mac, uint32_t tx_freq, uint8_t index );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void smtc_real_rx1_frequency_channel_set( lr1_stack_mac_t* lr1_mac, uint32_t rx_freq, uint8_t index );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void smtc_real_min_dr_channel_set( lr1_stack_mac_t* lr1_mac, uint8_t dr, uint8_t index );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void smtc_real_max_dr_channel_set( lr1_stack_mac_t* lr1_mac, uint8_t dr, uint8_t index );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void smtc_real_channel_enabled_set( lr1_stack_mac_t* lr1_mac, uint8_t enable, uint8_t index );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
uint32_t smtc_real_tx_frequency_channel_get( lr1_stack_mac_t* lr1_mac, uint8_t index );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
uint32_t smtc_real_rx1_frequency_channel_get( lr1_stack_mac_t* lr1_mac, uint8_t index );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
uint8_t smtc_real_min_dr_channel_get( lr1_stack_mac_t* lr1_mac );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
uint8_t smtc_real_max_dr_channel_get( lr1_stack_mac_t* lr1_mac );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
uint8_t smtc_real_channel_enabled_get( lr1_stack_mac_t* lr1_mac, uint8_t index );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
uint8_t smtc_real_rx1_join_delay_get( lr1_stack_mac_t* lr1_mac );

/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
uint8_t smtc_real_rx2_join_dr_get( lr1_stack_mac_t* lr1_mac );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
uint8_t smtc_real_frequency_factor_get( lr1_stack_mac_t* lr1_mac );

/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 *
 */
enum ral_lora_cr_e smtc_real_coding_rate_get( lr1_stack_mac_t* lr1_mac );

/*!
 * \brief   get the join toa @sf5 in ms
 * \remark  used to compute join duty cycle in a fast way
 * \param [IN]  none
 * \param [OUT] return the toa in ms
 *
 */
uint32_t smtc_real_get_join_sf5_toa_in_ms( const lr1_stack_mac_t* lr1_mac );

/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 *
 */
void smtc_real_duty_cycle_enable_set( const lr1_stack_mac_t* lr1_mac, uint8_t enable );

/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 *
 */
void smtc_real_duty_cycle_sum( const lr1_stack_mac_t* lr1_mac, uint32_t freq_hz, uint32_t toa_ms );

/*!
 * \brief   Decrement Time On Air
 * \remark  Must be called before check Duty Cycle available
 * \param [IN]  void
 * \return void
 */
void smtc_real_duty_cycle_update( const lr1_stack_mac_t* lr1_mac );

/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 *
 */
status_lorawan_t smtc_real_duty_cycle_is_toa_accepted( const lr1_stack_mac_t* lr1_mac, uint32_t freq_hz,
                                                       uint32_t toa_ms );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 *
 */
status_lorawan_t smtc_real_duty_cycle_is_channel_free( const lr1_stack_mac_t* lr1_mac, uint32_t freq_hz );

/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 *
 */
int32_t smtc_real_next_free_duty_cycle_ms_get( const lr1_stack_mac_t* lr1_mac );

/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 *
 */
uint8_t  smtc_real_preamble_get( const lr1_stack_mac_t* lr1_mac, uint8_t sf );


#ifdef __cplusplus
}
#endif

#endif  // __REAL_H__
