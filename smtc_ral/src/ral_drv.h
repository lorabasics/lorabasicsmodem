/**
 * @file      ral_drv.h
 *
 * @brief     Radio abstraction layer
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
 * ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef RAL_DRV_H__
#define RAL_DRV_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include "ral_defs.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

typedef bool ( *ral_handles_part_f )( const char* part_number );
typedef ral_status_t ( *ral_reset_f )( const void* context );
typedef ral_status_t ( *ral_init_f )( const void* context );
typedef ral_status_t ( *ral_wakeup_f )( const void* context );
typedef ral_status_t ( *ral_set_sleep_f )( const void* context, const bool retain_config );
typedef ral_status_t ( *ral_set_standby_f )( const void* context, ral_standby_cfg_t standby_cfg );
typedef ral_status_t ( *ral_set_fs_f )( const void* context );
typedef ral_status_t ( *ral_set_tx_f )( const void* context );
typedef ral_status_t ( *ral_set_rx_f )( const void* context, const uint32_t timeout_in_ms );
typedef ral_status_t ( *ral_set_rx_tx_fallback_mode_f )( const void*                context,
                                                         const ral_fallback_modes_t ral_fallback_mode );
typedef ral_status_t ( *ral_stop_timer_on_preamble_f )( const void* context, const bool enable );
typedef ral_status_t ( *ral_set_rx_duty_cycle_f )( const void* context, const uint32_t rx_time_in_ms,
                                                   const uint32_t sleep_time_in_ms );
typedef ral_status_t ( *ral_set_lora_cad_f )( const void* context );
typedef ral_status_t ( *ral_set_tx_cw_f )( const void* context );
typedef ral_status_t ( *ral_set_tx_infinite_preamble_f )( const void* context );
typedef ral_status_t ( *ral_set_tx_cfg_f )( const void* context, const int8_t output_pwr_in_dbm,
                                            const uint32_t rf_freq_in_hz );
typedef ral_status_t ( *ral_set_pkt_payload_f )( const void* context, const uint8_t* buffer, const uint16_t size );
typedef ral_status_t ( *ral_get_pkt_payload_f )( const void* context, uint16_t max_size_in_bytes, uint8_t* buffer,
                                                 uint16_t* size_in_bytes );
typedef ral_status_t ( *ral_get_irq_status_f )( const void* context, ral_irq_t* irq );
typedef ral_status_t ( *ral_clear_irq_status_f )( const void* context, const ral_irq_t irq );
typedef ral_status_t ( *ral_get_and_clear_irq_status_f )( const void* context, ral_irq_t* irq );
typedef ral_status_t ( *ral_set_dio_irq_params_f )( const void* context, const ral_irq_t irq );
typedef ral_status_t ( *ral_set_rf_freq_f )( const void* context, const uint32_t freq_in_hz );
typedef ral_status_t ( *ral_set_pkt_type_f )( const void* context, const ral_pkt_type_t pkt_type );
typedef ral_status_t ( *ral_set_gfsk_mod_params_f )( const void* context, const ral_gfsk_mod_params_t* params );
typedef ral_status_t ( *ral_set_gfsk_pkt_params_f )( const void* context, const ral_gfsk_pkt_params_t* params );
typedef ral_status_t ( *ral_set_lora_mod_params_f )( const void* context, const ral_lora_mod_params_t* params );
typedef ral_status_t ( *ral_set_lora_pkt_params_f )( const void* context, const ral_lora_pkt_params_t* params );
typedef ral_status_t ( *ral_set_lora_cad_params_f )( const void* context, const ral_lora_cad_params_t* params );
typedef ral_status_t ( *ral_set_lora_symb_nb_timeout_f )( const void* context, const uint8_t nb_of_symbs );
typedef ral_status_t ( *ral_set_flrc_mod_params_f )( const void* context, const ral_flrc_mod_params_t* params );
typedef ral_status_t ( *ral_set_flrc_pkt_params_f )( const void* context, const ral_flrc_pkt_params_t* params );
typedef ral_status_t ( *ral_get_gfsk_rx_pkt_status_f )( const void* context, ral_gfsk_rx_pkt_status_t* rx_pkt_status );
typedef ral_status_t ( *ral_get_lora_rx_pkt_status_f )( const void* context, ral_lora_rx_pkt_status_t* rx_pkt_status );
typedef ral_status_t ( *ral_get_flrc_rx_pkt_status_f )( const void* context, ral_flrc_rx_pkt_status_t* rx_pkt_status );
typedef ral_status_t ( *ral_get_rssi_inst_f )( const void* context, int16_t* rssi_in_dbm );
typedef uint32_t ( *ral_get_lora_time_on_air_in_ms_f )( const ral_lora_pkt_params_t* pkt_p,
                                                        const ral_lora_mod_params_t* mod_p );
typedef uint32_t ( *ral_get_gfsk_time_on_air_in_ms_f )( const ral_gfsk_pkt_params_t* pkt_p,
                                                        const ral_gfsk_mod_params_t* mod_p );
typedef uint32_t ( *ral_get_flrc_time_on_air_in_ms_f )( const ral_flrc_pkt_params_t* pkt_p,
                                                        const ral_flrc_mod_params_t* mod_p );
typedef ral_status_t ( *ral_set_gfsk_sync_word_f )( const void* context, const uint8_t* sync_word,
                                                    const uint8_t sync_word_len );
typedef ral_status_t ( *ral_set_lora_sync_word_f )( const void* context, const uint8_t sync_word );
typedef ral_status_t ( *ral_set_flrc_sync_word_f )( const void* context, const uint8_t* sync_word,
                                                    const uint8_t sync_word_len );
typedef ral_status_t ( *ral_set_gfsk_crc_params_f )( const void* context, const uint16_t seed,
                                                     const uint16_t polynomial );
typedef ral_status_t ( *ral_set_flrc_crc_params_f )( const void* context, const uint32_t seed );
typedef ral_status_t ( *ral_set_gfsk_whitening_seed_f )( const void* context, const uint16_t seed );
typedef ral_status_t ( *ral_get_tx_consumption_in_ua_f )( const void* context, const int8_t output_pwr_in_dbm,
                                                          const uint32_t rf_freq_in_hz,
                                                          uint32_t*      pwr_consumption_in_ua );
typedef ral_status_t ( *ral_get_gfsk_rx_consumption_in_ua_f )( const void* context, const uint32_t br_in_bps,
                                                               const uint32_t bw_dsb_in_hz, const bool rx_boosted,
                                                               uint32_t* pwr_consumption_in_ua );
typedef ral_status_t ( *ral_get_lora_rx_consumption_in_ua_f )( const void* context, const ral_lora_bw_t bw,
                                                               const bool rx_boosted, uint32_t* pwr_consumption_in_ua );

typedef struct ral_drv_s
{
    ral_handles_part_f                  handles_part;
    ral_reset_f                         reset;
    ral_init_f                          init;
    ral_wakeup_f                        wakeup;
    ral_set_sleep_f                     set_sleep;
    ral_set_standby_f                   set_standby;
    ral_set_fs_f                        set_fs;
    ral_set_tx_f                        set_tx;
    ral_set_rx_f                        set_rx;
    ral_set_rx_tx_fallback_mode_f       set_rx_tx_fallback_mode;
    ral_stop_timer_on_preamble_f        stop_timer_on_preamble;
    ral_set_rx_duty_cycle_f             set_rx_duty_cycle;
    ral_set_lora_cad_f                  set_lora_cad;
    ral_set_tx_cw_f                     set_tx_cw;
    ral_set_tx_infinite_preamble_f      set_tx_infinite_preamble;
    ral_set_tx_cfg_f                    set_tx_cfg;
    ral_set_pkt_payload_f               set_pkt_payload;
    ral_get_pkt_payload_f               get_pkt_payload;
    ral_get_irq_status_f                get_irq_status;
    ral_clear_irq_status_f              clear_irq_status;
    ral_get_and_clear_irq_status_f      get_and_clear_irq_status;
    ral_set_dio_irq_params_f            set_dio_irq_params;
    ral_set_rf_freq_f                   set_rf_freq;
    ral_set_pkt_type_f                  set_pkt_type;
    ral_set_gfsk_mod_params_f           set_gfsk_mod_params;
    ral_set_gfsk_pkt_params_f           set_gfsk_pkt_params;
    ral_set_lora_mod_params_f           set_lora_mod_params;
    ral_set_lora_pkt_params_f           set_lora_pkt_params;
    ral_set_lora_cad_params_f           set_lora_cad_params;
    ral_set_lora_symb_nb_timeout_f      set_lora_symb_nb_timeout;
    ral_set_flrc_mod_params_f           set_flrc_mod_params;
    ral_set_flrc_pkt_params_f           set_flrc_pkt_params;
    ral_get_gfsk_rx_pkt_status_f        get_gfsk_rx_pkt_status;
    ral_get_lora_rx_pkt_status_f        get_lora_rx_pkt_status;
    ral_get_flrc_rx_pkt_status_f        get_flrc_rx_pkt_status;
    ral_get_rssi_inst_f                 get_rssi_inst;
    ral_get_lora_time_on_air_in_ms_f    get_lora_time_on_air_in_ms;
    ral_get_gfsk_time_on_air_in_ms_f    get_gfsk_time_on_air_in_ms;
    ral_get_flrc_time_on_air_in_ms_f    get_flrc_time_on_air_in_ms;
    ral_set_gfsk_sync_word_f            set_gfsk_sync_word;
    ral_set_lora_sync_word_f            set_lora_sync_word;
    ral_set_flrc_sync_word_f            set_flrc_sync_word;
    ral_set_gfsk_crc_params_f           set_gfsk_crc_params;
    ral_set_flrc_crc_params_f           set_flrc_crc_params;
    ral_set_gfsk_whitening_seed_f       set_gfsk_whitening_seed;
    ral_get_tx_consumption_in_ua_f      get_tx_consumption_in_ua;
    ral_get_gfsk_rx_consumption_in_ua_f get_gfsk_rx_consumption_in_ua;
    ral_get_lora_rx_consumption_in_ua_f get_lora_rx_consumption_in_ua;
} ral_drv_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif  // RAL_DRV_H__

/* --- EOF ------------------------------------------------------------------ */
