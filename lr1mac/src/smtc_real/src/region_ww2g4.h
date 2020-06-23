/*!
 * \file      region_ww2g4.h
 *
 * \brief     region_2_4  abstraction layer definition
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

#ifndef __REGION_WW2G4_H__
#define __REGION_WW2G4_H__

#include <stdint.h>
#include <stdbool.h>

#include "smtc_real_defs.h"
#include "lr1mac_defs.h"
#include "lr1_stack_mac_layer.h"

#ifdef __cplusplus
extern "C" {
#endif

// clang-format off

#define NUMBER_OF_CHANNEL_WW2G4     (8)
#define JOIN_ACCEPT_DELAY1_WW2G4    (5)             // define in seconds
#define JOIN_ACCEPT_DELAY2_WW2G4    (6)             // define in seconds
#define RECEIVE_DELAY1_WW2G4        (1)             // define in seconds
#define TX_POWER_WW2G4              (10)            // define in dbm
#define ADR_ACK_LIMIT_WW2G4         (64)
#define ADR_ACK_DELAY_WW2G4         (32)
#define ACK_TIMEOUT_WW2G4           (2)             // +/- 1 s (random delay between 1 and 3 seconds)
#define FREQMIN_WW2G4               (2400000000)    // Hz
#define FREQMAX_WW2G4               (2480000000)    // Hz
#define RX2_FREQ_WW2G4              (2423000000)    // Hz
#define FREQUENCY_FACTOR_WW2G4      (200)           // MHz/200 when coded over 24 bits
#define RX2DR_INIT_WW2G4            (0)
#define SYNC_WORD_WW2G4             (0x21)
#define MIN_DR_WW2G4                (0)
#define MAX_DR_WW2G4                (7)
#define TIMEONAIR_JOIN_SF5_MS_WW2G4 (5)             // 4.026ms

// clang-format on

/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void region_ww2g4_init( lr1_stack_mac_t* lr1_mac );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void region_ww2g4_dr_distribution_set( lr1_stack_mac_t* lr1_mac, uint8_t adrMode );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
status_lorawan_t region_ww2g4_memory_load( lr1_stack_mac_t* lr1_mac );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void region_ww2g4_bad_crc_memory_set( lr1_stack_mac_t* lr1_mac );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void region_ww2g4_next_dr_get( lr1_stack_mac_t* lr1_mac );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void region_ww2g4_memory_save( lr1_stack_mac_t* lr1_mac );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
uint8_t region_ww2g4_max_payload_size_get( uint8_t dr );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
uint32_t region_ww2g4_decode_freq_from_buf( uint8_t freq_buf[3] );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void region_ww2g4_cflist_get( lr1_stack_mac_t* lr1_mac );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
status_lorawan_t region_ww2g4_next_channel_get( lr1_stack_mac_t* lr1_mac );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
status_lorawan_t region_ww2g4_join_next_channel_get( lr1_stack_mac_t* lr1_mac );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void region_ww2g4_rx_config_set( lr1_stack_mac_t* lr1_mac, rx_win_type_t type );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void region_ww2g4_power_set( lr1_stack_mac_t* lr1_mac, uint8_t power_cmd );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void region_ww2g4_channel_mask_set( lr1_stack_mac_t* lr1_mac );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void region_ww2g4_channel_mask_init( void );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void region_ww2g4_join_snapshot_channel_mask_init( void );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
status_channel_t region_ww2g4_channel_mask_build( uint8_t ChMaskCntl, uint16_t ChMask );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void region_ww2g4_dr_decrement( lr1_stack_mac_t* lr1_mac );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
uint8_t region_ww2g4_adr_ack_delay_get( void );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
uint8_t region_ww2g4_adr_ack_limit_get( void );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
uint8_t region_ww2g4_sync_word_get( void );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
status_lorawan_t region_ww2g4_is_valid_rx1_dr_offset( uint8_t rx1_dr_offset );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
status_lorawan_t region_ww2g4_is_valid_dr( uint8_t dr );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
status_lorawan_t region_ww2g4_is_acceptable_dr( uint8_t dr );  // for link adr cmd
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
status_lorawan_t region_ww2g4_is_valid_tx_frequency( uint32_t frequency );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
status_lorawan_t region_ww2g4_is_valid_rx_frequency( uint32_t frequency );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
status_lorawan_t region_ww2g4_is_valid_tx_power( uint8_t power );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
status_lorawan_t region_ww2g4_is_valid_channel_index( uint8_t channel_index );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
status_lorawan_t region_ww2g4_is_valid_size( lr1_stack_mac_t* lr1_mac, uint8_t dr, uint8_t size );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void region_ww2g4_tx_frequency_channel_set( uint32_t tx_freq, uint8_t index );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void region_ww2g4_rx1_frequency_channel_set( uint32_t rx_freq, uint8_t index );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void region_ww2g4_min_dr_channel_set( uint8_t dr, uint8_t index );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void region_ww2g4_max_dr_channel_set( uint8_t dr, uint8_t index );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void region_ww2g4_channel_enabled_set( uint8_t enable, uint8_t index );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
uint32_t region_ww2g4_tx_frequency_channel_get( uint8_t index );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
uint32_t region_ww2g4_rx1_frequency_channel_get( uint8_t index );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
uint8_t region_ww2g4_min_dr_channel_get( void );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
uint8_t region_ww2g4_max_dr_channel_get( void );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
uint8_t region_ww2g4_channel_enabled_get( uint8_t index );

#ifdef __cplusplus
}
#endif

#endif  // __REGION_WW2G4_H__
