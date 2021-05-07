/*!
 * \file      lr1_stack_mac_layer.c
 *
 * \brief     LoRaWan stack mac layer definition
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
#include <stdlib.h>
#include <stdio.h>
#include "smtc_real.h"
#include "lr1_stack_mac_layer.h"
#include "lr1mac_utilities.h"
#include "radio_planner.h"
#include "smtc_crypto.h"
#include "lorawan_api.h"
#include "smtc_modem_hal.h"
#include "lr1mac_defs_str.h"
#include <math.h>
#include "lr1mac_config.h"

/*
 *-----------------------------------------------------------------------------------
 * --- PRIVATE MACROS ---------------------------------------------------------------
 */

/*
 *-----------------------------------------------------------------------------------
 *--- PRIVATE VARIABLES -------------------------------------------------------------
 */

/*
 *-----------------------------------------------------------------------------------
 *--- PRIVATE FUNCTIONS DECLARATION -------------------------------------------------
 */

/*!
 *
 */
static void mac_header_set( lr1_stack_mac_t* lr1_mac );
/*!
 *
 */
static void frame_header_set( lr1_stack_mac_t* lr1_mac );
/*!
 *
 */
static void compute_rx_window_parameters( lr1_stack_mac_t* lr1_mac, uint8_t sf, lr1mac_bandwidth_t bw,
                                          uint32_t clock_accuracy, uint32_t rx_delay_ms, int8_t board_delay_ms,
                                          modulation_type_t rx_modulation_type );
/*!
 *
 */
static void link_check_parser( lr1_stack_mac_t* lr1_mac );
/*!
 *
 */
static void link_adr_parser( lr1_stack_mac_t* lr1_mac, uint8_t nb_link_adr_req );
/*!
 *
 */
static void duty_cycle_parser( lr1_stack_mac_t* lr1_mac );
/*!
 *
 */
static void rx_param_setup_parser( lr1_stack_mac_t* lr1_mac );
/*!
 *
 */
static void dev_status_parser( lr1_stack_mac_t* lr1_mac );
/*!
 *
 */
static void new_channel_parser( lr1_stack_mac_t* lr1_mac );
/*!
 *
 */
static void rx_timing_setup_parser( lr1_stack_mac_t* lr1_mac );
/*!
 *
 */
static void tx_param_setup_parser( lr1_stack_mac_t* lr1_mac );
/*!
 *
 */
static void dl_channel_parser( lr1_stack_mac_t* lr1_mac );
/*!
 *
 */
uint8_t lr1_stack_mac_cmd_ans_cut( uint8_t* nwk_ans, uint8_t nwk_ans_size_in, uint8_t max_allowed_size );

/*!
 *
 */
status_lorawan_t lr1mac_rx_payload_max_size_check( lr1_stack_mac_t* lr1_mac, uint8_t size );
/*
 *-----------------------------------------------------------------------------------
 *--- PUBLIC FUNCTIONS DEFINITIONS --------------------------------------------------
 */

void lr1_stack_mac_init( lr1_stack_mac_t* lr1_mac, lorawan_keys_t* lorawan_keys,
                         smtc_real_region_types_t smtc_real_region_types )
{
    lr1_mac->tx_major_bits                          = LORAWANR1;
    lr1_mac->radio_process_state                    = RADIOSTATE_IDLE;
    lr1_mac->next_time_to_join_seconds              = 0;
    lr1_mac->join_status                            = NOT_JOINED;
    lr1_mac->tx_modulation_type                     = LORA;
    lr1_mac->rx1_modulation_type                    = LORA;
    lr1_mac->rx2_modulation_type                    = LORA;
    lr1_mac->type_of_ans_to_send                    = NOFRAME_TOSEND;
    lr1_mac->otaa_device                            = lorawan_keys->otaaDevice;
    lr1_mac->nb_trans                               = 1;
    lr1_mac->available_app_packet                   = NO_LORA_RXPACKET_AVAILABLE;
    lr1_mac->real.region_type                       = smtc_real_region_types;
    lr1_mac->is_lorawan_modem_certification_enabled = false;
    lr1_mac->isr_tx_done_radio_timestamp            = 0;
    lr1_mac->dev_nonce                              = 0;
    lr1_mac->nb_of_reset                            = 0;
    lr1_mac->adr_mode_select                        = STATIC_ADR_MODE;
    lr1_mac->adr_mode_select_tmp                    = STATIC_ADR_MODE;
    lr1_mac->adr_custom                             = BSP_USER_DR_DISTRIBUTION_PARAMETERS;
    lr1_mac->current_win                            = RX1;
    memset( lr1_mac->fine_tune_board_setting_delay_ms, 0, sizeof( lr1_mac->fine_tune_board_setting_delay_ms ) );

#if defined( PERF_TEST_ENABLED )
    // bypass join process to allow perf testbench to trigger some modem send tx commands
    lr1_mac->join_status = JOINED;
#endif

    lr1_stack_mac_session_init( lr1_mac );
}

void lr1_stack_mac_session_init( lr1_stack_mac_t* lr1_mac )
{
    lr1_mac->fcnt_dwn                          = ~0;
    lr1_mac->fcnt_up                           = 0;
    lr1_mac->retry_join_cpt                    = 0;
    lr1_mac->adr_ack_cnt                       = 0;
    lr1_mac->adr_ack_cnt_confirmed_frame       = 0;
    lr1_mac->no_rx_packet_count_in_mobile_mode = 0;
    lr1_mac->tx_fopts_current_length           = 0;
    lr1_mac->tx_fopts_length                   = 0;
    lr1_mac->tx_fopts_lengthsticky             = 0;
    lr1_mac->nwk_ans_size                      = 0;
    lr1_mac->nwk_payload_size                  = 0;
    lr1_mac->nwk_payload_index                 = 0;
    lr1_mac->max_duty_cycle_index              = 0;
    lr1_mac->tx_duty_cycle_time_off_ms         = 0;
    lr1_mac->tx_duty_cycle_timestamp_ms        = smtc_modem_hal_get_time_in_ms( );
    lr1_mac->available_link_adr                = false;
}

/**************************************************************************************************/
/*                                  build lorawan frame */
/*                                  encrypt lorawan frame */
/*                       enqueue tx frame in radioplanner to proceed transmit */
/*                                                                                                */
/**************************************************************************************************/

void lr1_stack_mac_tx_frame_build( lr1_stack_mac_t* lr1_mac )
{
    lr1_mac->tx_fctrl = 0;
    lr1_mac->tx_fctrl = ( lr1_mac->adr_enable << 7 ) + ( lr1_mac->adr_ack_req << 6 ) + ( lr1_mac->tx_ack_bit << 5 ) +
                        ( lr1_mac->tx_fopts_current_length & 0x0F );
    lr1_mac->tx_ack_bit = 0;
    lr1_mac->rx_ack_bit = 0;
    mac_header_set( lr1_mac );
    frame_header_set( lr1_mac );
    lr1_mac->tx_payload_size = lr1_mac->app_payload_size + FHDROFFSET + lr1_mac->tx_fopts_current_length;
}

void lr1_stack_mac_tx_frame_encrypt( lr1_stack_mac_t* lr1_mac )
{
    lora_crypto_payload_encrypt(
        &lr1_mac->tx_payload[FHDROFFSET + lr1_mac->tx_fopts_current_length], lr1_mac->app_payload_size,
        ( lr1_mac->tx_fport == PORTNWK ) ? lr1_mac->nwk_skey : lr1_mac->app_skey, lr1_mac->dev_addr, UP_LINK,
        lr1_mac->fcnt_up, &lr1_mac->tx_payload[FHDROFFSET + lr1_mac->tx_fopts_current_length] );

    lora_crypto_add_mic( &lr1_mac->tx_payload[0], lr1_mac->tx_payload_size, lr1_mac->nwk_skey, lr1_mac->dev_addr,
                         UP_LINK, lr1_mac->fcnt_up );
    lr1_mac->tx_payload_size = lr1_mac->tx_payload_size + 4;
}
void lr1_stack_mac_tx_radio_free_lbt( lr1_stack_mac_t* lr1_mac )
{
    lr1_mac->radio_process_state = RADIOSTATE_TXON;
    lr1_mac->rtc_target_timer_ms = smtc_modem_hal_get_time_in_ms( ) + RP_MARGIN_DELAY;
    lr1_mac->send_at_time        = true;
    lr1_stack_mac_tx_radio_start( lr1_mac );
}
void lr1_stack_mac_radio_busy_lbt( lr1_stack_mac_t* lr1_mac )
{
    lr1_mac->radio_process_state = RADIOSTATE_IDLE;
    lr1_mac->rtc_target_timer_ms = smtc_modem_hal_get_time_in_ms( ) + RP_MARGIN_DELAY;
    smtc_real_get_next_channel( lr1_mac );
}
void lr1_stack_mac_radio_abort_lbt( lr1_stack_mac_t* lr1_mac )
{
    lr1_mac->radio_process_state = RADIOSTATE_ABORTED_BY_RP;
}
void lr1_stack_mac_tx_lora_launch_callback_for_rp( void* rp_void )
{
    radio_planner_t* rp = ( radio_planner_t* ) rp_void;
    uint8_t          id = rp->radio_task_id;
    smtc_modem_hal_start_radio_tcxo( );
    if( ralf_setup_lora( rp->radio, &rp->radio_params[id].tx.lora ) != RAL_STATUS_OK )
    {
        smtc_modem_hal_mcu_panic( );
    }
    if( ral_set_dio_irq_params( &( rp->radio->ral ), RAL_IRQ_TX_DONE ) != RAL_STATUS_OK )
    {
        smtc_modem_hal_mcu_panic( );
    }

    if( ral_set_pkt_payload( &( rp->radio->ral ), rp->payload[id], rp->payload_size[id] ) != RAL_STATUS_OK )
    {
        smtc_modem_hal_mcu_panic( );
    }
    // Wait the exact time
    while( ( int32_t )( rp->tasks[id].start_time_ms - rp_hal_timestamp_get( ) ) > 0 )
    {
        // Do nothing
    }
    if( ral_set_tx( &( rp->radio->ral ) ) != RAL_STATUS_OK )
    {
        smtc_modem_hal_mcu_panic( );
    }
    rp_stats_set_tx_timestamp( &rp->stats, rp_hal_timestamp_get( ) );
}

void lr1_stack_mac_tx_gfsk_launch_callback_for_rp( void* rp_void )
{
    radio_planner_t* rp = ( radio_planner_t* ) rp_void;
    uint8_t          id = rp->radio_task_id;
    smtc_modem_hal_start_radio_tcxo( );

    if( ralf_setup_gfsk( rp->radio, &rp->radio_params[id].tx.gfsk ) != RAL_STATUS_OK )
    {
        smtc_modem_hal_mcu_panic( );
    }
    if( ral_set_dio_irq_params( &( rp->radio->ral ), RAL_IRQ_TX_DONE ) != RAL_STATUS_OK )
    {
        smtc_modem_hal_mcu_panic( );
    }
    if( ral_set_pkt_payload( &( rp->radio->ral ), rp->payload[id], rp->payload_size[id] ) != RAL_STATUS_OK )
    {
        smtc_modem_hal_mcu_panic( );
    }
    // Wait the exact time
    while( ( int32_t )( rp->tasks[id].start_time_ms - rp_hal_timestamp_get( ) ) > 0 )
    {
    }
    if( ral_set_tx( &( rp->radio->ral ) ) != RAL_STATUS_OK )
    {
        smtc_modem_hal_mcu_panic( );
    }

    rp_stats_set_tx_timestamp( &rp->stats, rp_hal_timestamp_get( ) );
}

void lr1_stack_mac_rx_lora_launch_callback_for_rp( void* rp_void )
{
    radio_planner_t* rp = ( radio_planner_t* ) rp_void;
    uint8_t          id = rp->radio_task_id;
    smtc_modem_hal_start_radio_tcxo( );
    if( ralf_setup_lora( rp->radio, &rp->radio_params[id].rx.lora ) != RAL_STATUS_OK )
    {
        smtc_modem_hal_mcu_panic( );
    }
    if( ral_set_dio_irq_params( &( rp->radio->ral ), RAL_IRQ_RX_DONE | RAL_IRQ_RX_TIMEOUT | RAL_IRQ_RX_HDR_ERROR |
                                                         RAL_IRQ_RX_CRC_ERROR ) != RAL_STATUS_OK )
    {
        smtc_modem_hal_mcu_panic( );
    }
    // Wait the exact time
    while( ( int32_t )( rp->tasks[id].start_time_ms - rp_hal_timestamp_get( ) ) > 0 )
    {
    }
    if( ral_set_rx( &( rp->radio->ral ), rp->radio_params[id].rx.timeout_in_ms ) != RAL_STATUS_OK )
    {
        smtc_modem_hal_mcu_panic( );
    }
    rp_stats_set_rx_timestamp( &rp->stats, rp_hal_timestamp_get( ) );
}

void lr1_stack_mac_rx_gfsk_launch_callback_for_rp( void* rp_void )
{
    radio_planner_t* rp = ( radio_planner_t* ) rp_void;
    uint8_t          id = rp->radio_task_id;
    smtc_modem_hal_start_radio_tcxo( );
    if( ralf_setup_gfsk( rp->radio, &rp->radio_params[id].rx.gfsk ) != RAL_STATUS_OK )
    {
        smtc_modem_hal_mcu_panic( );
    }
    if( ral_set_dio_irq_params( &( rp->radio->ral ), RAL_IRQ_RX_DONE | RAL_IRQ_RX_TIMEOUT | RAL_IRQ_RX_CRC_ERROR ) !=
        RAL_STATUS_OK )
    {
        smtc_modem_hal_mcu_panic( );
    }
    // Wait the exact time
    while( ( int32_t )( rp->tasks[id].start_time_ms - rp_hal_timestamp_get( ) ) > 0 )
    {
    }
    if( ral_set_rx( &( rp->radio->ral ), rp->radio_params[id].rx.timeout_in_ms ) != RAL_STATUS_OK )
    {
        smtc_modem_hal_mcu_panic( );
    }
    rp_stats_set_rx_timestamp( &rp->stats, rp_hal_timestamp_get( ) );
}

void lr1_stack_mac_tx_radio_start( lr1_stack_mac_t* lr1_mac )
{
    rp_radio_params_t radio_params = { 0 };
    rp_task_t         rp_task      = { 0 };
    uint32_t          toa          = 0;
    uint8_t           tmp_dr       = smtc_real_sf_bw_to_dr( lr1_mac, lr1_mac->tx_sf, lr1_mac->tx_bw );

    if( lr1_mac->tx_modulation_type == LORA )
    {
        ralf_params_lora_t lora_param;
        memset( &lora_param, 0, sizeof( ralf_params_lora_t ) );

        lora_param.rf_freq_in_hz     = lr1_mac->tx_frequency;
        lora_param.output_pwr_in_dbm = smtc_real_clamp_output_power_eirp_vs_freq_and_dr(
            lr1_mac, lr1_mac->tx_power, lr1_mac->tx_frequency, tmp_dr );
        lora_param.sync_word       = smtc_real_get_sync_word( lr1_mac );
        lora_param.mod_params.sf   = ( ral_lora_sf_t ) lr1_mac->tx_sf;
        lora_param.mod_params.bw   = ( ral_lora_bw_t ) lr1_mac->tx_bw;
        lora_param.mod_params.cr   = smtc_real_get_coding_rate( lr1_mac );
        lora_param.mod_params.ldro = ral_compute_lora_ldro( lora_param.mod_params.sf, lora_param.mod_params.bw );
        lora_param.pkt_params.preamble_len_in_symb = smtc_real_get_preamble_len( lr1_mac, lora_param.mod_params.sf );
        lora_param.pkt_params.header_type          = RAL_LORA_PKT_EXPLICIT;
        lora_param.pkt_params.pld_len_in_bytes     = lr1_mac->tx_payload_size;
        lora_param.pkt_params.crc_is_on            = true;
        lora_param.pkt_params.invert_iq_is_on      = false;

        radio_params.pkt_type = RAL_PKT_TYPE_LORA;
        radio_params.tx.lora  = lora_param;

        toa = ral_get_lora_time_on_air_in_ms( ( &lr1_mac->rp->radio->ral ), ( &lora_param.pkt_params ),
                                              ( &lora_param.mod_params ) );

        rp_task.type                  = RP_TASK_TYPE_TX_LORA;
        rp_task.launch_task_callbacks = lr1_stack_mac_tx_lora_launch_callback_for_rp;
    }
    else if( lr1_mac->tx_modulation_type == FSK )
    {
        ralf_params_gfsk_t gfsk_param;
        memset( &gfsk_param, 0, sizeof( ralf_params_gfsk_t ) );

        gfsk_param.rf_freq_in_hz     = lr1_mac->tx_frequency;
        gfsk_param.output_pwr_in_dbm = smtc_real_clamp_output_power_eirp_vs_freq_and_dr(
            lr1_mac, lr1_mac->tx_power, lr1_mac->tx_frequency, tmp_dr );
        gfsk_param.sync_word                        = smtc_real_get_gfsk_sync_word( lr1_mac );
        gfsk_param.dc_free_is_on                    = true;
        gfsk_param.whitening_seed                   = GFSK_WHITENING_SEED;
        gfsk_param.crc_seed                         = GFSK_CRC_SEED;
        gfsk_param.crc_polynomial                   = GFSK_CRC_POLYNOMIAL;
        gfsk_param.pkt_params.header_type           = RAL_GFSK_PKT_VAR_LEN;
        gfsk_param.pkt_params.pld_len_in_bytes      = lr1_mac->tx_payload_size;
        gfsk_param.pkt_params.preamble_len_in_bits  = 40;
        gfsk_param.pkt_params.sync_word_len_in_bits = 24;
        gfsk_param.pkt_params.dc_free               = RAL_GFSK_DC_FREE_WHITENING;
        gfsk_param.pkt_params.crc_type              = RAL_GFSK_CRC_2_BYTES_INV;
        gfsk_param.mod_params.fdev_in_hz            = 25000;
        gfsk_param.mod_params.br_in_bps             = lr1_mac->tx_sf * 1000;
        gfsk_param.mod_params.bw_dsb_in_hz          = 100000;
        gfsk_param.mod_params.pulse_shape           = RAL_GFSK_PULSE_SHAPE_BT_1;

        radio_params.pkt_type = RAL_PKT_TYPE_GFSK;
        radio_params.tx.gfsk  = gfsk_param;

        toa = ral_get_gfsk_time_on_air_in_ms( ( &lr1_mac->rp->radio->ral ), ( &gfsk_param.pkt_params ),
                                              ( &gfsk_param.mod_params ) );

        SMTC_MODEM_HAL_TRACE_PRINTF( "  TxFrequency = %d, FSK \n", lr1_mac->tx_frequency );
        rp_task.type                  = RP_TASK_TYPE_TX_FSK;
        rp_task.launch_task_callbacks = lr1_stack_mac_tx_gfsk_launch_callback_for_rp;
    }
    else
    {
        smtc_modem_hal_lr1mac_panic( "TX MODULATION NOT SUPPORTED\n" );
    }

    uint8_t my_hook_id;
    if( rp_hook_get_id( lr1_mac->rp, lr1_mac, &my_hook_id ) != RP_HOOK_STATUS_OK )
    {
        smtc_modem_hal_lr1mac_panic( );
    }
    rp_task.hook_id          = my_hook_id;
    rp_task.duration_time_ms = toa;
    rp_task.start_time_ms    = lr1_mac->rtc_target_timer_ms;
    if( lr1_mac->send_at_time == true )
    {
        lr1_mac->send_at_time = false;  // reinit the flag
        rp_task.state         = RP_TASK_STATE_SCHEDULE;
    }
    else
    {
        rp_task.state = RP_TASK_STATE_ASAP;
    }
    lr1_mac->radio_process_state = RADIOSTATE_TXON;
    if( rp_task_enqueue( lr1_mac->rp, &rp_task, lr1_mac->tx_payload, lr1_mac->tx_payload_size, &radio_params ) !=
        RP_HOOK_STATUS_OK )
    {
        lr1_mac->radio_process_state = RADIOSTATE_ABORTED_BY_RP;
        SMTC_MODEM_HAL_TRACE_PRINTF( "Radio planner hook %d is busy \n", my_hook_id );
    }
}

void lr1_stack_mac_rx_radio_start( lr1_stack_mac_t* lr1_mac, const rx_win_type_t type, const uint32_t time_to_start )
{
    rp_radio_params_t radio_params = { 0 };
    lr1_mac->current_win           = type;
    if( ( ( type == RX1 ) && ( lr1_mac->rx1_modulation_type == LORA ) ) ||
        ( ( type == RX2 ) && ( lr1_mac->rx2_modulation_type == LORA ) ) )
    {
        ralf_params_lora_t lora_param;
        memset( &lora_param, 0, sizeof( ralf_params_lora_t ) );

        lora_param.sync_word                   = smtc_real_get_sync_word( lr1_mac );
        lora_param.symb_nb_timeout             = lr1_mac->rx_window_symb;
        lora_param.mod_params.cr               = smtc_real_get_coding_rate( lr1_mac );
        lora_param.pkt_params.header_type      = RAL_LORA_PKT_EXPLICIT;
        lora_param.pkt_params.pld_len_in_bytes = 255;
        lora_param.pkt_params.crc_is_on        = false;
        lora_param.pkt_params.invert_iq_is_on  = true;

        switch( type )
        {
        case RX1:
            lora_param.mod_params.sf = ( ral_lora_sf_t ) lr1_mac->rx1_sf;
            lora_param.mod_params.bw = ( ral_lora_bw_t ) lr1_mac->rx1_bw;
            lora_param.rf_freq_in_hz = lr1_mac->rx1_frequency;
            break;

        case RX2:
            lora_param.mod_params.sf = ( ral_lora_sf_t ) lr1_mac->rx2_sf;
            lora_param.mod_params.bw = ( ral_lora_bw_t ) lr1_mac->rx2_bw;
            lora_param.rf_freq_in_hz = lr1_mac->rx2_frequency;
            break;

        default:
            smtc_modem_hal_lr1mac_panic( "RX windows unknow\n" );
            break;
        }
        lora_param.mod_params.ldro = ral_compute_lora_ldro( lora_param.mod_params.sf, lora_param.mod_params.bw );
        lora_param.pkt_params.preamble_len_in_symb = smtc_real_get_preamble_len( lr1_mac, lora_param.mod_params.sf );

        radio_params.pkt_type         = RAL_PKT_TYPE_LORA;
        radio_params.rx.lora          = lora_param;
        radio_params.rx.timeout_in_ms = lr1_mac->rx_timeout_ms;
    }
    else if( ( ( type == RX1 ) && ( lr1_mac->rx1_modulation_type == FSK ) ) ||
             ( ( type == RX2 ) && ( lr1_mac->rx2_modulation_type == FSK ) ) )
    {
        ralf_params_gfsk_t gfsk_param;
        memset( &gfsk_param, 0, sizeof( ralf_params_gfsk_t ) );

        gfsk_param.sync_word                        = smtc_real_get_gfsk_sync_word( lr1_mac );
        gfsk_param.dc_free_is_on                    = true;
        gfsk_param.whitening_seed                   = GFSK_WHITENING_SEED;
        gfsk_param.crc_seed                         = GFSK_CRC_SEED;
        gfsk_param.crc_polynomial                   = GFSK_CRC_POLYNOMIAL;
        gfsk_param.pkt_params.header_type           = RAL_GFSK_PKT_VAR_LEN;
        gfsk_param.pkt_params.pld_len_in_bytes      = 255;
        gfsk_param.pkt_params.preamble_len_in_bits  = 40;
        gfsk_param.pkt_params.sync_word_len_in_bits = 24;
        gfsk_param.pkt_params.dc_free               = RAL_GFSK_DC_FREE_WHITENING;
        gfsk_param.pkt_params.crc_type              = RAL_GFSK_CRC_2_BYTES_INV;
        gfsk_param.mod_params.fdev_in_hz            = 25000;
        gfsk_param.mod_params.bw_dsb_in_hz          = 100000;
        gfsk_param.mod_params.pulse_shape           = RAL_GFSK_PULSE_SHAPE_BT_1;

        switch( type )
        {
        case RX1:
            gfsk_param.rf_freq_in_hz        = lr1_mac->rx1_frequency;
            gfsk_param.mod_params.br_in_bps = lr1_mac->rx1_sf * 1000;
            break;

        case RX2:
            gfsk_param.rf_freq_in_hz        = lr1_mac->rx2_frequency;
            gfsk_param.mod_params.br_in_bps = lr1_mac->rx2_sf * 1000;
            break;

        default:
            smtc_modem_hal_lr1mac_panic( "RX windows unknow\n" );
            break;
        }
        radio_params.pkt_type         = RAL_PKT_TYPE_GFSK;
        radio_params.rx.gfsk          = gfsk_param;
        radio_params.rx.timeout_in_ms = lr1_mac->rx_timeout_symb_in_ms;
    }
    else
    {
        smtc_modem_hal_lr1mac_panic( "MODULATION NOT SUPPORTED\n" );
    }

    uint8_t my_hook_id;
    if( rp_hook_get_id( lr1_mac->rp, lr1_mac, &my_hook_id ) != RP_HOOK_STATUS_OK )
    {
        smtc_modem_hal_lr1mac_panic( );
    }

    rp_task_t rp_task = {
        .hook_id = my_hook_id,
        .type    = ( radio_params.pkt_type == RAL_PKT_TYPE_LORA ) ? RP_TASK_TYPE_RX_LORA : RP_TASK_TYPE_RX_FSK,
        .launch_task_callbacks = ( radio_params.pkt_type == RAL_PKT_TYPE_LORA )
                                     ? lr1_stack_mac_rx_lora_launch_callback_for_rp
                                     : lr1_stack_mac_rx_gfsk_launch_callback_for_rp,
        .state                 = RP_TASK_STATE_SCHEDULE,
        .start_time_ms         = time_to_start,
        .duration_time_ms      = lr1_mac->rx_timeout_symb_in_ms,
    };

    if( rp_task_enqueue( lr1_mac->rp, &rp_task, lr1_mac->rx_payload, 255, &radio_params ) == RP_HOOK_STATUS_OK )
    {
        if( radio_params.pkt_type == RAL_PKT_TYPE_LORA )
        {
            SMTC_MODEM_HAL_TRACE_PRINTF(
                "  %s LoRa at %u ms: freq:%d, SF%u, %s, sync word = 0x%02x\n", smtc_name_rx_windows[type],
                time_to_start, radio_params.rx.lora.rf_freq_in_hz, radio_params.rx.lora.mod_params.sf,
                smtc_name_bw[radio_params.rx.lora.mod_params.bw], smtc_real_get_sync_word( lr1_mac ) );
            // SMTC_MODEM_HAL_TRACE_INFO( "symb_nb_timeout %d, %d ms\n", radio_params.rx.lora.symb_nb_timeout,
            //                     lr1_mac->rx_timeout_symb_in_ms );
        }
        else
        {
            SMTC_MODEM_HAL_TRACE_PRINTF( "  %s FSK freq:%d\n", smtc_name_rx_windows[type],
                                         radio_params.rx.gfsk.rf_freq_in_hz );
            // SMTC_MODEM_HAL_TRACE_INFO( "rx_timeout_symb_in_ms %d ms\n", lr1_mac->rx_timeout_symb_in_ms );
        }
    }
    else
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "Radio planner hook %d is busy \n", my_hook_id );
    }
}

int lr1_stack_mac_downlink_check_under_it( lr1_stack_mac_t* lr1_mac )
{
    int              status            = OKLORAWAN;
    valid_dev_addr_t is_valid_dev_addr = UNVALID_DEV_ADDR;

    // check Mtype
    uint8_t rx_mtype_tmp = lr1_mac->rx_payload[0] >> 5;
    if( ( rx_mtype_tmp == JOIN_REQUEST ) || ( rx_mtype_tmp == UNCONF_DATA_UP ) || ( rx_mtype_tmp == CONF_DATA_UP ) ||
        ( rx_mtype_tmp == REJOIN_REQUEST ) || ( rx_mtype_tmp == PROPRIETARY ) )
    {
        status += ERRORLORAWAN;
        SMTC_MODEM_HAL_TRACE_PRINTF( " BAD Mtype = %u for RX Frame \n", rx_mtype_tmp );
        is_valid_dev_addr = UNVALID_DEV_ADDR;
    }
    // check devaddr
    if( lr1_mac->join_status == JOINED )
    {
        uint32_t dev_addr_tmp = lr1_mac->rx_payload[1] + ( lr1_mac->rx_payload[2] << 8 ) +
                                ( lr1_mac->rx_payload[3] << 16 ) + ( lr1_mac->rx_payload[4] << 24 );

        is_valid_dev_addr = lr1mac_check_dev_addr( lr1_mac->dev_addr, NULL, 0, dev_addr_tmp );
        if( is_valid_dev_addr == UNVALID_DEV_ADDR )
        {
            status += ERRORLORAWAN;
            SMTC_MODEM_HAL_TRACE_INFO( " BAD DevAddr = %x for RX Frame and %x \n \n", lr1_mac->dev_addr, dev_addr_tmp );
        }
        if( status != OKLORAWAN )
        {
            lr1_mac->rx_payload_size = 0;
        }
    }
    else
    {
        is_valid_dev_addr = UNVALID_DEV_ADDR;
    }

    return ( status );
}

void lr1_stack_mac_rp_callback( lr1_stack_mac_t* lr1_mac )
{
    int      status = OKLORAWAN;
    uint32_t tcurrent_ms;
    uint8_t  my_hook_id;
    rp_hook_get_id( lr1_mac->rp, lr1_mac, &my_hook_id );
    rp_get_status( lr1_mac->rp, my_hook_id, &tcurrent_ms, &( lr1_mac->planner_status ) );

    switch( lr1_mac->planner_status )
    {
    case RP_STATUS_TX_DONE:
        lr1_mac->isr_tx_done_radio_timestamp = tcurrent_ms;  //@info Timestamp only on txdone it
        break;

    case RP_STATUS_RX_PACKET:

        // save rssi and snr
        lr1_mac->rx_metadata.timestamp = tcurrent_ms;
        lr1_mac->rx_metadata.rx_snr    = lr1_mac->rp->radio_params[my_hook_id].rx.lora_pkt_status.snr_pkt_in_db;
        lr1_mac->rx_metadata.rx_rssi   = lr1_mac->rp->radio_params[my_hook_id].rx.lora_pkt_status.rssi_pkt_in_dbm;
        lr1_mac->rx_payload_size       = ( uint8_t ) lr1_mac->rp->payload_size[my_hook_id];

        SMTC_MODEM_HAL_TRACE_PRINTF( "payload size receive = %u, snr = %d , rssi = %d\n", lr1_mac->rx_payload_size,
                                     lr1_mac->rp->radio_params[my_hook_id].rx.lora_pkt_status.snr_pkt_in_db,
                                     lr1_mac->rp->radio_params[my_hook_id].rx.lora_pkt_status.rssi_pkt_in_dbm );

        SMTC_MODEM_HAL_TRACE_ARRAY( "Rx Payload", lr1_mac->rx_payload, lr1_mac->rx_payload_size );

        status = lr1_stack_mac_downlink_check_under_it( lr1_mac );

        if( status != OKLORAWAN )
        {  // Case receive a packet but it isn't a valid packet
            SMTC_MODEM_HAL_TRACE_MSG( "Receive a packet But rejected and too late to restart\n" );
            lr1_mac->planner_status = RP_STATUS_RX_TIMEOUT;
        }
        break;
    case RP_STATUS_RX_CRC_ERROR:
        SMTC_MODEM_HAL_TRACE_PRINTF( "RP_STATUS_RX_CRC_ERROR\n" );
        break;

    case RP_STATUS_RX_TIMEOUT: {
#ifndef BSP_LR1MAC_DISABLE_FINE_TUNE
        uint32_t rx_timestamp_calibration = tcurrent_ms;
        uint8_t  current_dr;
        uint32_t rx_delay_s;
        if( lr1_mac->current_win == RX1 )
        {
            current_dr = smtc_real_sf_bw_to_dr( lr1_mac, lr1_mac->rx1_sf, lr1_mac->rx1_bw );
            rx_delay_s = lr1_mac->rx1_delay_s * 1000;
        }
        else
        {
            current_dr = smtc_real_sf_bw_to_dr( lr1_mac, lr1_mac->rx2_sf, lr1_mac->rx2_bw );
            rx_delay_s = ( lr1_mac->rx1_delay_s * 1000 ) + 1000;
        }

        int32_t error_fine_tune = 0;

        error_fine_tune = rx_timestamp_calibration -
                          ( lr1_mac->isr_tx_done_radio_timestamp + rx_delay_s + lr1_mac->rx_timeout_symb_in_ms +
                            lr1_mac->rx_offset_ms ) -
                          lr1_mac->fine_tune_board_setting_delay_ms[current_dr] - BSP_BOARD_DELAY_RX_SETTING_MS;

        SMTC_MODEM_HAL_TRACE_PRINTF(
            "DR%u Fine tune correction (ms) = %d, error fine tune (ms) = %d, lr1_mac->rx_offset_ms = %d\n", current_dr,
            lr1_mac->fine_tune_board_setting_delay_ms[current_dr], error_fine_tune, lr1_mac->rx_offset_ms );

        if( error_fine_tune < 0 )
        {
            lr1_mac->fine_tune_board_setting_delay_ms[current_dr] -= 1;
        }
        else if( error_fine_tune > 0 )
        {
            lr1_mac->fine_tune_board_setting_delay_ms[current_dr] += 1;
        }

#endif  // BSP_LR1MAC_DISABLE_FINE_TUNE
    }
    break;
    case RP_STATUS_TASK_ABORTED:
        SMTC_MODEM_HAL_TRACE_PRINTF( "lr1mac task aborted by the radioplanner \n" );
        break;
    default:
        SMTC_MODEM_HAL_TRACE_PRINTF( "receive It RADIO error %u\n", lr1_mac->planner_status );
        break;
    }

    switch( lr1_mac->radio_process_state )
    {
    case RADIOSTATE_TXON:

        lr1_mac->radio_process_state  = RADIOSTATE_TXFINISHED;
        lr1_mac->available_app_packet = NO_LORA_RXPACKET_AVAILABLE;
        break;

    case RADIOSTATE_TXFINISHED:
        lr1_mac->radio_process_state = RADIOSTATE_RX1FINISHED;
        break;

    case RADIOSTATE_RX1FINISHED:
        lr1_mac->radio_process_state = RADIOSTATE_IDLE;
        break;

    default:
        if( lr1_mac->planner_status != RP_STATUS_TASK_ABORTED )
        {
            smtc_modem_hal_lr1mac_panic( );
        }
        break;
    }
    if( lr1_mac->planner_status == RP_STATUS_TASK_ABORTED )
    {
        lr1_mac->radio_process_state = RADIOSTATE_ABORTED_BY_RP;
    }
}

int lr1_stack_mac_radio_state_get( lr1_stack_mac_t* lr1_mac )
{
    return ( lr1_mac->radio_process_state );
}

void lr1_stack_mac_rx_timer_configure( lr1_stack_mac_t* lr1_mac, const rx_win_type_t type )
{
    const uint32_t    tcurrent_ms = smtc_modem_hal_get_time_in_ms( );
    bool              is_type_ok  = true;
    ral_lora_sf_t     sf;
    ral_lora_bw_t     bw;
    modulation_type_t mod_type = LORA;
    uint32_t          delay_ms;
    uint8_t           current_dr;

    smtc_real_set_rx_config( lr1_mac, type );

    switch( type )
    {
    case RX1:
        sf         = ( ral_lora_sf_t ) lr1_mac->rx1_sf;
        bw         = ( ral_lora_bw_t ) lr1_mac->rx1_bw;
        delay_ms   = lr1_mac->rx1_delay_s * 1000;
        mod_type   = lr1_mac->rx1_modulation_type;
        current_dr = smtc_real_sf_bw_to_dr( lr1_mac, lr1_mac->rx1_sf, lr1_mac->rx1_bw );
        break;

    case RX2:
        sf         = ( ral_lora_sf_t ) lr1_mac->rx2_sf;
        bw         = ( ral_lora_bw_t ) lr1_mac->rx2_bw;
        delay_ms   = ( lr1_mac->rx1_delay_s * 1000 ) + 1000;
        mod_type   = lr1_mac->rx2_modulation_type;
        current_dr = smtc_real_sf_bw_to_dr( lr1_mac, lr1_mac->rx2_sf, lr1_mac->rx2_bw );
        break;

    default:
        is_type_ok = false;
        smtc_modem_hal_lr1mac_panic( "RX windows unknow\n" );
        break;
    }

    if( is_type_ok == true )
    {
        compute_rx_window_parameters(
            lr1_mac, ( uint8_t ) sf, ( lr1mac_bandwidth_t ) bw, lr1_mac->crystal_error, delay_ms,
            BSP_BOARD_DELAY_RX_SETTING_MS + lr1_mac->fine_tune_board_setting_delay_ms[current_dr], mod_type );

        uint32_t talarm_ms = delay_ms + lr1_mac->isr_tx_done_radio_timestamp - tcurrent_ms;
        smtc_real_set_rx_config( lr1_mac, type );
        lr1_stack_mac_rx_radio_start( lr1_mac, type, tcurrent_ms + talarm_ms + lr1_mac->rx_offset_ms );
        SMTC_MODEM_HAL_TRACE_PRINTF( "  Timer will expire in %d ms\n", ( talarm_ms + lr1_mac->rx_offset_ms ) );
    }
}

rx_packet_type_t lr1_stack_mac_rx_frame_decode( lr1_stack_mac_t* lr1_mac )
{
    int              status         = OKLORAWAN;
    rx_packet_type_t rx_packet_type = NO_MORE_VALID_RX_PACKET;
    uint32_t         mic_in;
    status += lr1mac_rx_payload_min_size_check( lr1_mac->rx_payload_size );
    status += lr1mac_rx_payload_max_size_check( lr1_mac, lr1_mac->rx_payload_size );

    if( status != OKLORAWAN )
    {
        return NO_MORE_VALID_RX_PACKET;
    }
    status += lr1mac_rx_mhdr_extract( lr1_mac->rx_payload, &( lr1_mac->rx_mtype ), &( lr1_mac->rx_major ),
                                      &( lr1_mac->tx_ack_bit ) );
    if( status != OKLORAWAN )
    {
        return NO_MORE_VALID_RX_PACKET;
    }
    /************************************************************************/
    /*                 Case : the receive packet is a JoinResponse          */
    /************************************************************************/

    if( lr1_mac->rx_mtype == JOIN_ACCEPT )
    {
        join_decrypt( &lr1_mac->rx_payload[1], lr1_mac->rx_payload_size - 1, lr1_mac->app_key,
                      &lr1_mac->rx_payload[1] );
        lr1_mac->rx_payload_size = lr1_mac->rx_payload_size - MICSIZE;
        memcpy1( ( uint8_t* ) &mic_in, &lr1_mac->rx_payload[lr1_mac->rx_payload_size], MICSIZE );
        status += check_join_mic( lr1_mac->rx_payload, lr1_mac->rx_payload_size, lr1_mac->app_key, mic_in );
        SMTC_MODEM_HAL_TRACE_PRINTF( " status = %d\n", status );
        if( status == OKLORAWAN )
        {
            lr1_mac->no_rx_packet_count_in_mobile_mode = 0;
            return JOIN_ACCEPT_PACKET;
        }
    }
    else
    {
        /************************************************************************/
        /*               Case : the receive packet is not a JoinResponse */
        /************************************************************************/
        uint16_t fcnt_dwn_tmp       = 0;
        uint32_t fcnt_dwn_stack_tmp = lr1_mac->fcnt_dwn;
        status += lr1mac_rx_fhdr_extract( lr1_mac->rx_payload, lr1_mac->rx_payload_size, &( lr1_mac->rx_fopts_length ),
                                          &fcnt_dwn_tmp, lr1_mac->dev_addr, &( lr1_mac->rx_metadata.rx_fport ),
                                          &( lr1_mac->rx_payload_empty ), &( lr1_mac->rx_fctrl ), lr1_mac->rx_fopts );
        if( status == OKLORAWAN )
        {
            status = lr1mac_fcnt_dwn_accept( fcnt_dwn_tmp, &fcnt_dwn_stack_tmp );
        }
        if( status == OKLORAWAN )
        {
            lr1_mac->rx_payload_size = lr1_mac->rx_payload_size - MICSIZE;
            memcpy1( ( uint8_t* ) &mic_in, &lr1_mac->rx_payload[lr1_mac->rx_payload_size], MICSIZE );
            status += check_mic( &lr1_mac->rx_payload[0], lr1_mac->rx_payload_size, lr1_mac->nwk_skey,
                                 lr1_mac->dev_addr, fcnt_dwn_stack_tmp, mic_in );
        }
        if( status == OKLORAWAN )
        {
            lr1_mac->fcnt_dwn                          = fcnt_dwn_stack_tmp;
            lr1_mac->adr_ack_cnt                       = 0;  // reset adr counter, receive a valid frame.
            lr1_mac->adr_ack_cnt_confirmed_frame       = 0;  // reset adr counter in case of confirmed frame
            lr1_mac->no_rx_packet_count_in_mobile_mode = 0;
            lr1_mac->tx_fopts_current_length           = 0;  // reset the fopts of the sticky set in payload
            lr1_mac->tx_fopts_lengthsticky = 0;  // reset the fopts of the sticky cmd received on a valide frame
                                                 // if received on RX1 or RX2

            // else reset the retransmission counter
            if( !( ( ( lr1_mac->rx_fctrl & 0x20 ) != 0x20 ) && ( lr1_mac->tx_mtype == CONF_DATA_UP ) ) )
            {
                // reset retransmission counter if received on RX1 or RX2 with
                lr1_mac->nb_trans_cpt = 1;
            }
            // test the ack bit when tx_mtype == CONF_DATA_UP
            if( ( ( lr1_mac->rx_fctrl & 0x20 ) == 0x20 ) && ( lr1_mac->tx_mtype == CONF_DATA_UP ) )
            {
                lr1_mac->rx_ack_bit = 1;
            }

            if( lr1_mac->rx_payload_empty == 0 )  // rx payload not empty
            {
                lr1_mac->rx_payload_size = lr1_mac->rx_payload_size - FHDROFFSET - lr1_mac->rx_fopts_length;
                /*
                     Receive a management frame
                     => set rx_packet_type = NWKRXPACKET
                     => if ack bit is set to one : notify the upper layer that the stack have received an ack bit =>
                   set available_app_packet to LORA_RX_PACKET_AVAILABLE with length = 0
                */
                if( lr1_mac->rx_metadata.rx_fport == 0 )
                {  // receive a mac management frame without fopts
                    if( lr1_mac->rx_fopts_length == 0 )
                    {
                        payload_decrypt( &lr1_mac->rx_payload[FHDROFFSET], lr1_mac->rx_payload_size, lr1_mac->nwk_skey,
                                         lr1_mac->dev_addr, 1, lr1_mac->fcnt_dwn, &lr1_mac->nwk_payload[0] );
                        if( lr1_mac->rx_payload_size > NWK_REQ_PAYLOAD_MAX_SIZE )
                        {
                            SMTC_MODEM_HAL_TRACE_WARNING( " Receive too many nwk frames\n" );
                        }
                        else
                        {
                            lr1_mac->nwk_payload_size = lr1_mac->rx_payload_size;
                            rx_packet_type            = NWKRXPACKET;
                        }
                    }
                    else
                    {
                        SMTC_MODEM_HAL_TRACE_WARNING( " Receive an not valid packet with fopt bytes on port zero\n" );
                    }
                }
                /*
                    Receive a app frame with size > 0
                    =>  if rx_fopts_length > 0 set rx_packet_type = USERRX_FOPTSPACKET and copy fopts data
                    =>  notify the upper layer that the stack have received a payload : set available_app_packet to
                   LORA_RX_PACKET_AVAILABLE with length > 0
                */
                else
                {
                    payload_decrypt( &lr1_mac->rx_payload[FHDROFFSET + lr1_mac->rx_fopts_length],
                                     lr1_mac->rx_payload_size, lr1_mac->app_skey, lr1_mac->dev_addr, 1,
                                     lr1_mac->fcnt_dwn, &lr1_mac->rx_payload[0] );
                    if( lr1_mac->rx_fopts_length != 0 )
                    {
                        memcpy1( lr1_mac->nwk_payload, lr1_mac->rx_fopts, lr1_mac->rx_fopts_length );
                        lr1_mac->nwk_payload_size = lr1_mac->rx_fopts_length;
                        rx_packet_type            = USERRX_FOPTSPACKET;
                    }
                    else
                    {
                        rx_packet_type = USER_RX_PACKET;
                    }
                    lr1_mac->available_app_packet = LORA_RX_PACKET_AVAILABLE;
                }
            }
            /*
                Receive an empty user payload
                => if rx_fopts_length > 0 set rx_packet_type = USERRX_FOPTSPACKET and copy fopts data
                => notify the upper layer that the stack have received a payload : ack_bit is set to 1
            */
            else
            {
                if( lr1_mac->rx_fopts_length != 0 )
                {
                    memcpy1( lr1_mac->nwk_payload, lr1_mac->rx_fopts, lr1_mac->rx_fopts_length );
                    lr1_mac->nwk_payload_size = lr1_mac->rx_fopts_length;
                    rx_packet_type            = USERRX_FOPTSPACKET;
                }
                else
                {
                    rx_packet_type = USER_RX_PACKET;
                }
            }
        }
    }
    SMTC_MODEM_HAL_TRACE_PRINTF( " rx_packet_type = %d \n", rx_packet_type );
    return ( rx_packet_type );
}

void lr1_stack_mac_update_tx_done( lr1_stack_mac_t* lr1_mac )
{
    if( ( lr1_mac->no_rx_packet_count_in_mobile_mode < 0xFFFF ) && ( lr1_mac->adr_mode_select != STATIC_ADR_MODE ) )
    {
        lr1_mac->no_rx_packet_count_in_mobile_mode++;
    }
    if( lr1_mac->tx_mtype == CONF_DATA_UP )
    {
        lr1_mac->adr_ack_cnt_confirmed_frame++;
    }
    else
    {
        lr1_mac->adr_ack_cnt++;  // increment adr counter each uplink frame
    }
}

void lr1_stack_mac_update( lr1_stack_mac_t* lr1_mac )
{
    lr1_mac->adr_ack_limit       = smtc_real_get_adr_ack_limit( lr1_mac );
    lr1_mac->adr_ack_delay       = smtc_real_get_adr_ack_delay( lr1_mac );
    lr1_mac->type_of_ans_to_send = NOFRAME_TOSEND;

    if( lr1_mac->join_status == NOT_JOINED )
    {
        // get current timestamp to check with duty cycle will be applied
        uint32_t current_time_s = smtc_modem_hal_get_time_in_s( );

        lr1_mac->retry_join_cpt++;

        if( current_time_s < ( lr1_mac->first_join_timestamp + 3600 ) )
        {
            // during first hour after first join try => duty cycle of 1/100 ie 36s over 1 hour
            lr1_mac->next_time_to_join_seconds = current_time_s + ( lr1_stack_toa_get( lr1_mac ) ) / 10;
            // ts=cur_ts+(toa_s*100) = cur_ts + (toa_ms / 1000) * 100 = cur_ts + toa_ms/10
        }
        else if( current_time_s < ( lr1_mac->first_join_timestamp + 36000 + 3600 ) )
        {
            // during the 10 hours following first hour after first join try =>duty cycle of 1/1000 ie 36s over 10
            // hours
            lr1_mac->next_time_to_join_seconds = current_time_s + ( lr1_stack_toa_get( lr1_mac ) );
            // ts=cur_ts+(toa_s*1000) = cur_ts + (toa_ms / 1000) * 1000 = cur_ts + toa_ms
        }
        else
        {
            // Following the first 11 hours after first join try => duty cycle of 1/10000 ie 8.7s over 24 hours
            lr1_mac->next_time_to_join_seconds = current_time_s + ( lr1_stack_toa_get( lr1_mac ) ) * 10;
            // ts=cur_ts+(toa_s*10000) = cur_ts + (toa_ms / 1000) * 10000 = cur_ts + toa_ms*10
        }
    }

    if( ( lr1_mac->adr_ack_cnt >= lr1_mac->adr_ack_limit ) &&
        ( lr1_mac->adr_ack_cnt <= ( lr1_mac->adr_ack_limit + lr1_mac->adr_ack_delay ) ) )
    {
        lr1_mac->adr_ack_req = 1;
    }

    if( ( lr1_mac->adr_ack_cnt < lr1_mac->adr_ack_limit ) ||
        ( lr1_mac->adr_ack_cnt > ( lr1_mac->adr_ack_limit + lr1_mac->adr_ack_delay ) ) )
    {
        lr1_mac->adr_ack_req = 0;
    }

    if( lr1_mac->nb_trans_cpt <= 1 )
    {
        // could also be set to 1 if receive valid ans
        lr1_mac->nb_trans_cpt = 1;  // error case shouldn't exist
        lr1_mac->fcnt_up++;
    }
    else
    {
        lr1_mac->type_of_ans_to_send = USRFRAME_TORETRANSMIT;
        lr1_mac->nb_trans_cpt--;
    }

    if( lr1_mac->adr_ack_cnt >= lr1_mac->adr_ack_limit + lr1_mac->adr_ack_delay )
    {
        // In case of retransmission, if the packet is too long for the next DR, don't decrease the DR
        if( lr1_mac->type_of_ans_to_send == USRFRAME_TORETRANSMIT )
        {
            uint8_t dr_tmp = smtc_real_decrement_dr_simulation( lr1_mac );
            if( smtc_real_is_payload_size_valid( lr1_mac, dr_tmp, lr1_mac->app_payload_size,
                                                 lr1_mac->uplink_dwell_time ) == OKLORAWAN )
            {
                smtc_real_decrement_dr( lr1_mac );
                if( lr1_mac->tx_data_rate_adr != smtc_real_get_min_tx_channel_dr( lr1_mac ) )
                {
                    lr1_mac->adr_ack_cnt = lr1_mac->adr_ack_limit;
                }
            }
        }
        else
        {
            smtc_real_decrement_dr( lr1_mac );
            if( lr1_mac->tx_data_rate_adr != smtc_real_get_min_tx_channel_dr( lr1_mac ) )
            {
                lr1_mac->adr_ack_cnt = lr1_mac->adr_ack_limit;
            }
        }
    }

    if( lr1_mac->adr_ack_cnt_confirmed_frame >= ADR_LIMIT_CONF_UP )
    {
        // In case of retransmission, if the packet is too long for the next DR, don't decrease the DR
        if( lr1_mac->type_of_ans_to_send == USRFRAME_TORETRANSMIT )
        {
            uint8_t dr_tmp = smtc_real_decrement_dr_simulation( lr1_mac );
            if( smtc_real_is_payload_size_valid( lr1_mac, dr_tmp, lr1_mac->app_payload_size,
                                                 lr1_mac->uplink_dwell_time ) == OKLORAWAN )
            {
                lr1_mac->adr_ack_cnt_confirmed_frame = 0;
                smtc_real_decrement_dr( lr1_mac );
            }
        }
        else
        {
            lr1_mac->adr_ack_cnt_confirmed_frame = 0;
            smtc_real_decrement_dr( lr1_mac );
        }
    }

    if( ( ( lr1_mac->adr_ack_cnt + lr1_mac->adr_ack_cnt_confirmed_frame ) >= lr1_mac->no_rx_packet_count ) &&
        ( lr1_mac->no_rx_packet_count > 0 ) )
    {
        smtc_modem_hal_lr1mac_panic( "Reach max tx frame without dl, ul unconf:%d, ul conf:%d\n", lr1_mac->adr_ack_cnt,
                                     lr1_mac->adr_ack_cnt_confirmed_frame );
    }

    // If tx_fopts_length > tx_fopts_lengthsticky, first uplink with Answer(s),
    // put in Network to not penalize the App Payload
    if( lr1_mac->tx_fopts_length > lr1_mac->tx_fopts_lengthsticky )
    {
        lr1_mac->nwk_ans_size = lr1_mac->tx_fopts_length;
        memcpy1( lr1_mac->nwk_ans, lr1_mac->tx_fopts_data, lr1_mac->tx_fopts_length );
        lr1_mac->type_of_ans_to_send = NWKFRAME_TOSEND;
        lr1_mac->nb_trans_cpt        = lr1_mac->nb_trans;
    }
    // Concerns sticky commands, put in FOpts
    else
    {
        lr1_mac->tx_fopts_current_length = lr1_mac->tx_fopts_lengthsticky;
        memcpy1( lr1_mac->tx_fopts_current_data, lr1_mac->tx_fopts_datasticky, lr1_mac->tx_fopts_lengthsticky );
    }
    lr1_mac->tx_fopts_length = 0;

    if( lr1_mac->join_status == JOINED )
    {
        if( ( ( lr1_mac->tx_mtype != UNCONF_DATA_UP ) || ( lr1_mac->type_of_ans_to_send != USRFRAME_TORETRANSMIT ) ) )
        {
            status_lorawan_t status = smtc_real_get_next_dr( lr1_mac );
            if( status == ERRORLORAWAN )
            {
                smtc_modem_hal_mcu_panic( " Data Rate incompatible with channel mask \n" );
            }
        }
    }
    switch( lr1_mac->type_of_ans_to_send )
    {
    case NOFRAME_TOSEND:

        break;
    case NWKFRAME_TOSEND: {
        status_lorawan_t status;

        status = smtc_real_is_payload_size_valid( lr1_mac, lr1_mac->tx_data_rate, lr1_mac->nwk_ans_size,
                                                  lr1_mac->uplink_dwell_time );
        if( status != OKLORAWAN )
        {
            lr1_mac->nwk_ans_size = lr1_stack_mac_cmd_ans_cut(
                lr1_mac->nwk_ans, lr1_mac->nwk_ans_size,
                smtc_real_get_max_payload_size( lr1_mac, lr1_mac->tx_data_rate, lr1_mac->uplink_dwell_time ) );
        }
        memcpy1( &lr1_mac->tx_payload[FHDROFFSET], lr1_mac->nwk_ans, lr1_mac->nwk_ans_size );
        lr1_mac->app_payload_size = lr1_mac->nwk_ans_size;
        lr1_mac->tx_fport         = PORTNWK;
        lr1_mac->tx_mtype         = UNCONF_DATA_UP;  //@note Mtype have to be confirm
        lr1_stack_mac_tx_frame_build( lr1_mac );
        lr1_stack_mac_tx_frame_encrypt( lr1_mac );
    }
    break;
    case USERACK_TOSEND:

        break;
    }
}

uint8_t lr1_stack_mac_cmd_ans_cut( uint8_t* nwk_ans, uint8_t nwk_ans_size_in, uint8_t max_allowed_size )
{
    uint8_t* p_tmp = nwk_ans;
    uint8_t* p     = nwk_ans;

    while( p_tmp - nwk_ans < MIN( nwk_ans_size_in, max_allowed_size ) )
    {
        p_tmp += lr1mac_cmd_mac_ans_size[nwk_ans[p_tmp - nwk_ans]];

        if( ( p_tmp - nwk_ans ) <= max_allowed_size )
        {
            p = p_tmp;
        }
        else
        {
            break;
        }
    }

    return p - nwk_ans;  // New payload size
}

status_lorawan_t lr1_stack_mac_cmd_parse( lr1_stack_mac_t* lr1_mac )
{
    uint8_t          cmd_identifier;
    uint8_t          nb_link_adr_req = 0;
    status_lorawan_t status          = OKLORAWAN;
    lr1_mac->nwk_payload_index       = 0;
    lr1_mac->nwk_ans_size            = 0;
    lr1_mac->tx_fopts_length         = 0;
    lr1_mac->tx_fopts_lengthsticky   = 0;

    while( lr1_mac->nwk_payload_size > lr1_mac->nwk_payload_index )
    {  //@note MacNwkPayloadSize and lr1_mac->nwk_payload[0] are updated in
        // Parser's method

        if( lr1_mac->tx_fopts_length > NWK_ANS_PAYLOAD_MAX_SIZE )
        {
            SMTC_MODEM_HAL_TRACE_WARNING( "too much cmd in the payload \n" );
            return ( ERRORLORAWAN );
        }
        cmd_identifier = lr1_mac->nwk_payload[lr1_mac->nwk_payload_index];
        switch( cmd_identifier )
        {
        case LINK_CHECK_ANS:

            link_check_parser( lr1_mac );
            break;
        case LINK_ADR_REQ:

            nb_link_adr_req = 0;
            /* extract the number of multiple link adr req specification in
             * LoRAWan1.0.2 */
            while(
                ( lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + ( nb_link_adr_req * LINK_ADR_REQ_SIZE )] ==
                  LINK_ADR_REQ ) &&
                ( lr1_mac->nwk_payload_index + ( nb_link_adr_req * LINK_ADR_REQ_SIZE ) < lr1_mac->nwk_payload_size ) )
            {
                nb_link_adr_req++;
            }
            link_adr_parser( lr1_mac, nb_link_adr_req );
            break;
        case DUTY_CYCLE_REQ:

            duty_cycle_parser( lr1_mac );  //@note send answer but do nothing

            break;
        case RXPARRAM_SETUP_REQ:

            rx_param_setup_parser( lr1_mac );

            break;
        case DEV_STATUS_REQ:

            dev_status_parser( lr1_mac );  //@note  Done but margin have no sense
                                           // tb implemented
            break;
        case NEW_CHANNEL_REQ:

            new_channel_parser( lr1_mac );

            break;
        case RXTIMING_SETUP_REQ:

            rx_timing_setup_parser( lr1_mac );

            break;
        case TXPARAM_SETUP_REQ:

            tx_param_setup_parser( lr1_mac );

            break;
        case DL_CHANNEL_REQ:

            dl_channel_parser( lr1_mac );

            break;
        default:
            lr1_mac->nwk_payload_size = 0;
            SMTC_MODEM_HAL_TRACE_MSG( " Illegal state in mac layer\n " );

            break;
        }
    }

    return ( status );
}
void lr1_stack_mac_join_request_build( lr1_stack_mac_t* lr1_mac )
{
    SMTC_MODEM_HAL_TRACE_ARRAY( "DevEUI", lr1_mac->dev_eui, 8 );
    SMTC_MODEM_HAL_TRACE_ARRAY( "appEUI", lr1_mac->app_eui, 8 );
    SMTC_MODEM_HAL_TRACE_ARRAY( "appKey", lr1_mac->app_key, 16 );
    if( lr1_mac->dev_nonce < 0xFFFF )
    {
        lr1_mac->dev_nonce += 1;
    }
    SMTC_MODEM_HAL_TRACE_PRINTF( "DevNonce 0x%x\n", lr1_mac->dev_nonce );
    lr1_mac->tx_mtype     = JOIN_REQUEST;
    lr1_mac->nb_trans_cpt = 1;
    lr1_mac->nb_trans     = 1;
    mac_header_set( lr1_mac );
    for( int i = 0; i < 8; i++ )
    {
        lr1_mac->tx_payload[1 + i] = lr1_mac->app_eui[7 - i];
        lr1_mac->tx_payload[9 + i] = lr1_mac->dev_eui[7 - i];
    }
    lr1_mac->tx_payload[17]  = ( uint8_t )( ( lr1_mac->dev_nonce & 0x00FF ) );
    lr1_mac->tx_payload[18]  = ( uint8_t )( ( lr1_mac->dev_nonce & 0xFF00 ) >> 8 );
    lr1_mac->tx_payload_size = 19;
    uint32_t mic;
    //    FcntUp = 1;
    join_compute_mic( &lr1_mac->tx_payload[0], lr1_mac->tx_payload_size, lr1_mac->app_key, &mic );
    memcpy1( &lr1_mac->tx_payload[lr1_mac->tx_payload_size], ( uint8_t* ) &mic, 4 );
    lr1_mac->tx_payload_size = lr1_mac->tx_payload_size + 4;
}

status_lorawan_t lr1_stack_mac_join_accept( lr1_stack_mac_t* lr1_mac )
{
    status_lorawan_t status;
    uint8_t          app_nonce[6];
    int              i;
    memcpy1( app_nonce, &lr1_mac->rx_payload[1], 6 );
    join_compute_skeys( lr1_mac->app_key, app_nonce, lr1_mac->dev_nonce, lr1_mac->nwk_skey, lr1_mac->app_skey );
    if( lr1_mac->rx_payload_size > 13 )
    {  // cflist are presents
        for( i = 0; i < 16; i++ )
        {
            lr1_mac->cf_list[i] = lr1_mac->rx_payload[13 + i];
        }
        smtc_real_update_cflist( lr1_mac );
    }
    else
    {
        smtc_real_init_after_join_snapshot_channel_mask( lr1_mac );
    }
    lr1_mac->dev_addr = ( lr1_mac->rx_payload[7] + ( lr1_mac->rx_payload[8] << 8 ) + ( lr1_mac->rx_payload[9] << 16 ) +
                          ( lr1_mac->rx_payload[10] << 24 ) );
    lr1_mac->rx1_dr_offset = ( lr1_mac->rx_payload[11] & 0x70 ) >> 4;
    lr1_mac->rx2_data_rate = ( lr1_mac->rx_payload[11] & 0x0F );
    lr1_mac->rx1_delay_s   = ( lr1_mac->rx_payload[12] & 0x0F );
    if( lr1_mac->rx1_delay_s == 0 )
    {
        lr1_mac->rx1_delay_s = 1;  // Lorawan standart define 0 such as a delay of 1
    }

    status = smtc_real_is_rx1_dr_offset_valid( lr1_mac, lr1_mac->rx1_dr_offset );
    if( status != OKLORAWAN )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "JoinAccept invalid Rx1DrOffset %d\n", lr1_mac->rx1_dr_offset );
        return ERRORLORAWAN;
    }

    status = smtc_real_is_rx_dr_valid( lr1_mac, lr1_mac->rx2_data_rate );
    if( status != OKLORAWAN )
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "JoinAccept invalid Rx2Datarate %d\n", lr1_mac->rx2_data_rate );
        return ERRORLORAWAN;
    }

    lr1_mac->join_status = JOINED;

    lr1_stack_mac_session_init( lr1_mac );
    smtc_real_init_session( lr1_mac );

    SMTC_MODEM_HAL_TRACE_PRINTF( " DevAddr= %x\n", lr1_mac->dev_addr );
    SMTC_MODEM_HAL_TRACE_PRINTF( " MacRx1DataRateOffset= %d\n", lr1_mac->rx1_dr_offset );
    SMTC_MODEM_HAL_TRACE_PRINTF( " MacRx2DataRate= %d\n", lr1_mac->rx2_data_rate );
    SMTC_MODEM_HAL_TRACE_PRINTF( " MacRx1Delay= %d\n", lr1_mac->rx1_delay_s );
    SMTC_MODEM_HAL_TRACE_MSG( " Save In Flash After Join succeed \n" );
    // uint32_t c2 = smtc_modem_hal_get_time_in_ms();
    // SMTC_MODEM_HAL_TRACE_PRINTF ("GET Time %d Join duration =%d\n",c2,c2-c1 );
    return OKLORAWAN;
}

uint8_t lr1_stack_mac_min_tx_dr_get( lr1_stack_mac_t* lr1_mac )
{
    return smtc_real_get_min_tx_channel_dr( lr1_mac );
}
uint8_t lr1_stack_mac_max_tx_dr_get( lr1_stack_mac_t* lr1_mac )
{
    return smtc_real_get_max_tx_channel_dr( lr1_mac );
}
uint16_t lr1_stack_mac_mask_tx_dr_channel_up_dwell_time_check( lr1_stack_mac_t* lr1_mac )
{
    return smtc_real_mask_tx_dr_channel_up_dwell_time_check( lr1_mac );
}

void lr1_stack_rx1_join_delay_set( lr1_stack_mac_t* lr1_mac )
{
    lr1_mac->rx1_delay_s = smtc_real_get_rx1_join_delay( lr1_mac );
}
void lr1_stack_rx2_join_dr_set( lr1_stack_mac_t* lr1_mac )
{
    lr1_mac->rx2_data_rate = smtc_real_get_rx2_join_dr( lr1_mac );
}

int32_t lr1_stack_network_next_free_duty_cycle_ms_get( lr1_stack_mac_t* lr1_mac )
{
    int32_t  time_off_left = 0;
    uint32_t delta_t       = 0;

    if( lr1_mac->tx_duty_cycle_time_off_ms > 0 )
    {
        uint32_t rtc_now = smtc_modem_hal_get_time_in_ms( );
        if( rtc_now >= lr1_mac->tx_duty_cycle_timestamp_ms )
        {
            delta_t = rtc_now - lr1_mac->tx_duty_cycle_timestamp_ms;
        }
        else
        {
            delta_t = 0xFFFFFFFFUL - lr1_mac->tx_duty_cycle_timestamp_ms;
            delta_t += rtc_now;
        }

        if( delta_t > lr1_mac->tx_duty_cycle_time_off_ms )
        {
            time_off_left = 0;
        }
        else
        {
            time_off_left = lr1_mac->tx_duty_cycle_time_off_ms - delta_t;
        }
    }
    return time_off_left;
}

uint32_t lr1_stack_toa_get( lr1_stack_mac_t* lr1_mac )
{
    uint32_t toa = 0;
    if( lr1_mac->tx_modulation_type == LORA )
    {
        ralf_params_lora_t lora_param;
        memset( &lora_param, 0, sizeof( ralf_params_lora_t ) );

        lora_param.mod_params.sf   = ( ral_lora_sf_t ) lr1_mac->tx_sf;
        lora_param.mod_params.bw   = ( ral_lora_bw_t ) lr1_mac->tx_bw;
        lora_param.mod_params.cr   = smtc_real_get_coding_rate( lr1_mac );
        lora_param.mod_params.ldro = ral_compute_lora_ldro( lora_param.mod_params.sf, lora_param.mod_params.bw );

        lora_param.pkt_params.crc_is_on            = true;
        lora_param.pkt_params.invert_iq_is_on      = false;
        lora_param.pkt_params.pld_len_in_bytes     = lr1_mac->tx_payload_size;
        lora_param.pkt_params.preamble_len_in_symb = smtc_real_get_preamble_len( lr1_mac, lora_param.mod_params.sf );
        lora_param.pkt_params.header_type          = RAL_LORA_PKT_EXPLICIT;
        toa = ral_get_lora_time_on_air_in_ms( ( &lr1_mac->rp->radio->ral ), ( &lora_param.pkt_params ),
                                              ( &lora_param.mod_params ) );
    }
    else if( lr1_mac->tx_modulation_type == FSK )
    {
        ralf_params_gfsk_t gfsk_param;
        memset( &gfsk_param, 0, sizeof( ralf_params_gfsk_t ) );

        gfsk_param.rf_freq_in_hz                    = lr1_mac->tx_frequency;
        gfsk_param.sync_word                        = smtc_real_get_gfsk_sync_word( lr1_mac );
        gfsk_param.dc_free_is_on                    = true;
        gfsk_param.whitening_seed                   = GFSK_WHITENING_SEED;
        gfsk_param.crc_seed                         = GFSK_CRC_SEED;
        gfsk_param.crc_polynomial                   = GFSK_CRC_POLYNOMIAL;
        gfsk_param.mod_params.fdev_in_hz            = 25000;
        gfsk_param.mod_params.br_in_bps             = lr1_mac->tx_sf * 1000;
        gfsk_param.mod_params.bw_dsb_in_hz          = 100000;
        gfsk_param.pkt_params.pld_len_in_bytes      = lr1_mac->tx_payload_size;
        gfsk_param.pkt_params.preamble_len_in_bits  = 40;
        gfsk_param.pkt_params.header_type           = RAL_GFSK_PKT_VAR_LEN;
        gfsk_param.pkt_params.sync_word_len_in_bits = 24;
        gfsk_param.pkt_params.dc_free               = RAL_GFSK_DC_FREE_WHITENING;
        gfsk_param.pkt_params.crc_type              = RAL_GFSK_CRC_2_BYTES_INV;

        toa = ral_get_gfsk_time_on_air_in_ms( ( &lr1_mac->rp->radio->ral ), ( &gfsk_param.pkt_params ),
                                              ( &gfsk_param.mod_params ) );
    }
    else
    {
        smtc_modem_hal_lr1mac_panic( "TX MODULATION NOT SUPPORTED\n" );
    }
    return toa;
}

uint8_t lr1_stack_nb_trans_get( lr1_stack_mac_t* lr1_mac )
{
    return ( lr1_mac->nb_trans );
}

status_lorawan_t lr1_stack_nb_trans_set( lr1_stack_mac_t* lr1_mac, uint8_t nb_trans )
{
    if( lr1_mac->adr_mode_select == STATIC_ADR_MODE )
    {
        return ERRORLORAWAN;
    }

    if( ( nb_trans > 0 ) && ( nb_trans < 16 ) )
    {
        lr1_mac->nb_trans = nb_trans;
        return OKLORAWAN;
    }

    return ERRORLORAWAN;
}

uint32_t lr1_stack_get_crystal_error( lr1_stack_mac_t* lr1_mac )
{
    return lr1_mac->crystal_error;
}

void lr1_stack_set_crystal_error( lr1_stack_mac_t* lr1_mac, uint32_t crystal_error )
{
    lr1_mac->crystal_error = crystal_error;
}

/*
 *-----------------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITIONS ------------------------------------------------
 */

static void mac_header_set( lr1_stack_mac_t* lr1_mac )
{
    lr1_mac->tx_payload[0] = ( ( lr1_mac->tx_mtype & 0x7 ) << 5 ) + ( lr1_mac->tx_major_bits & 0x3 );
}

static void frame_header_set( lr1_stack_mac_t* lr1_mac )
{
    lr1_mac->tx_payload[1] = ( uint8_t )( ( lr1_mac->dev_addr & 0x000000FF ) );
    lr1_mac->tx_payload[2] = ( uint8_t )( ( lr1_mac->dev_addr & 0x0000FF00 ) >> 8 );
    lr1_mac->tx_payload[3] = ( uint8_t )( ( lr1_mac->dev_addr & 0x00FF0000 ) >> 16 );
    lr1_mac->tx_payload[4] = ( uint8_t )( ( lr1_mac->dev_addr & 0xFF000000 ) >> 24 );
    lr1_mac->tx_payload[5] = lr1_mac->tx_fctrl;
    lr1_mac->tx_payload[6] = ( uint8_t )( ( lr1_mac->fcnt_up & 0x000000FF ) );
    lr1_mac->tx_payload[7] = ( uint8_t )( ( lr1_mac->fcnt_up & 0x0000FF00 ) >> 8 );
    for( int i = 0; i < lr1_mac->tx_fopts_current_length; i++ )
    {
        lr1_mac->tx_payload[8 + i] = lr1_mac->tx_fopts_current_data[i];
    }
    lr1_mac->tx_payload[8 + lr1_mac->tx_fopts_current_length] = lr1_mac->tx_fport;
}

static void compute_rx_window_parameters( lr1_stack_mac_t* lr1_mac, uint8_t sf, lr1mac_bandwidth_t bw,
                                          uint32_t clock_accuracy, uint32_t rx_delay_ms, int8_t board_delay_ms,
                                          modulation_type_t rx_modulation_type )
{
    // ClockAccuracy is set in Define.h, it is board dependent. It must be equal
    // to error in per thousand

    bool     flag_small_timing = false;
    int      bw_temp           = 125;
    uint32_t rx_error_ms       = 0;
    float    tsymbol           = 0.0f;
    uint8_t  min_rx_symbols    = 6;

    if( rx_modulation_type == LORA )
    {
        // Use lr1mac_utilities_get_symb_time_us
        switch( bw )
        {
        case BW125:
            bw_temp = 125;
            break;
        case BW250:
            bw_temp = 250;
            if( sf == 7 )
            {
                // clock_accuracy += 1;
                flag_small_timing = true;
            }
            break;
        case BW500:
            bw_temp = 500;
            if( ( sf == 7 ) || ( sf == 8 ) )
            {
                clock_accuracy += 1;
                flag_small_timing = true;
            }
            break;
        case BW800:
            bw_temp = 800;
            break;
        default:
            bw_temp = 125;
            break;
        }

        rx_error_ms =
            ( clock_accuracy * rx_delay_ms ) /
            1000;  // for example with an clockaccuracy = 30 (3%)  and a rx windows set to 5s => rxerror = 150 ms

        tsymbol                 = ( float ) ( 1 << sf ) / ( float ) bw_temp;
        lr1_mac->rx_window_symb = ( uint16_t )(
            MAX( ceilf( ( 2 * min_rx_symbols - 6 ) + ( ( 2 * rx_error_ms * bw_temp ) >> sf ) ), min_rx_symbols ) );
    }
    else
    {  // FSK
        flag_small_timing = true;
        min_rx_symbols    = 10;
        rx_error_ms =
            ( clock_accuracy * rx_delay_ms ) /
            1000;  // for example with an clockaccuracy = 30 (3%)  and a rx windows set to 5s => rxerror = 150 ms

        tsymbol = ( 8.0f / ( float ) sf );  // 1 symbol equals 1 byte
        lr1_mac->rx_window_symb =
            ( uint16_t )( MAX( ceilf( ( 2 * min_rx_symbols - 6 ) + ( ( 2 * rx_error_ms * sf ) >> 3 ) ),
                               min_rx_symbols ) );  // Computed number of symbols
    }

    // Because the hardware allows an even number of symbols
    if( ( lr1_mac->rx_window_symb % 2 ) == 1 )
    {
        lr1_mac->rx_window_symb++;
    }

    // use an extra large timeout in case of unwanted infinite preamble reception
    // (rx timeout will be trigged by symb timeout if no premable is seen during rx_window_symb time)
    lr1_mac->rx_timeout_ms = 3000;

    float rx_offset_ms_tmp =
        ( ( 4.0f - ( float ) ( lr1_mac->rx_window_symb >> 1 ) ) * tsymbol ) - ( float ) board_delay_ms;

    lr1_mac->rx_offset_ms =
        ( rx_offset_ms_tmp >= 0.0f ) ? ( int32_t )( rx_offset_ms_tmp + 0.5f ) : ( int32_t )( rx_offset_ms_tmp - 0.5f );

    if( flag_small_timing == true )
    {
        lr1_mac->rx_window_symb += 2;
    }

    lr1_mac->rx_timeout_symb_in_ms =
        MAX( ( int ) ( ( float ) lr1_mac->rx_window_symb * tsymbol + 0.5f ), 2 );  // round the value

#if defined( SX128X )
    // rx timeout is used to simuate a symb timeout in sx128x (need to open preamb + sync +header)
    lr1_mac->rx_timeout_ms =
        MAX( ceilf( ( ( float ) lr1_mac->rx_window_symb + 16.25f ) * tsymbol ), BSP_MIN_RX_TIMEOUT_DELAY_MS );
    lr1_mac->rx_timeout_symb_in_ms = lr1_mac->rx_timeout_ms;
#endif

    SMTC_MODEM_HAL_TRACE_PRINTF( "rx_offset_ms:%d, rx_timeout_symb_in_ms:%d, rx_window_symb: %d, board_delay_ms:%d\n",
                                 lr1_mac->rx_offset_ms, lr1_mac->rx_timeout_symb_in_ms, lr1_mac->rx_window_symb,
                                 board_delay_ms );
}

/************************************************************************************************/
/*                    Private NWK MANAGEMENTS Methods */
/************************************************************************************************/
static void link_check_parser( lr1_stack_mac_t* lr1_mac )
{
    SMTC_MODEM_HAL_TRACE_PRINTF( " Margin = %d , GwCnt = %d \n", lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 1],
                                 lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 2] );

    lr1_mac->nwk_payload_index += LINK_CHECK_ANS_SIZE;
}
/**********************************************************************************************************************/
/*                                               Private NWK MANAGEMENTS :
 * LinkADR                                    */
/*  Note : describe multiple adr specification */
/*                                                                                                                    */
/*  Step 1 : Create a "unwrapped channel mask" in case of multiple adr cmd with
 * both Channem Mask and ChannnelMaskCntl*/
/*       2 : Extract from the last adr cmd datarate candidate */
/*       3 : Extract from the last adr cmd TxPower candidate */
/*       4 : Extract from the last adr cmd NBRetry candidate */
/*       5 : Check errors cases (described below) */
/*       6 : If No error Set new channel mask, txpower,datarate and nbretry */
/*       7 : Compute duplicated LinkAdrAns */
/*                                                                                                                    */
/*  Error cases    1 : Channel Cntl mask RFU for each adr cmd (in case of
 * multiple cmd)                               */
/*                 2 : Undefined channel ( freq = 0 ) for active bit in the
 * unwrapped channel mask                    */
/*                 3 : Unwrapped channel mask = 0 (none active channel) */
/*                 4 : For the last adr cmd not valid tx power */
/*                 5 : For the last adr cmd not valid datarate */
/*                     ( datarate > dRMax or datarate < dRMin for all active
 * channel )                                */
/**********************************************************************************************************************/

static void link_adr_parser( lr1_stack_mac_t* lr1_mac, uint8_t nb_link_adr_req )
{
    for( uint8_t i = 0; i < nb_link_adr_req; i++ )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "%u - Cmd link_adr_parser = %02x %02x %02x %02x \n", i,
                                     lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + ( i * LINK_ADR_REQ_SIZE ) + 1],
                                     lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + ( i * LINK_ADR_REQ_SIZE ) + 2],
                                     lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + ( i * LINK_ADR_REQ_SIZE ) + 3],
                                     lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + ( i * LINK_ADR_REQ_SIZE ) + 4] );
    }
    status_lorawan_t status        = OKLORAWAN;
    status_channel_t statusChannel = OKCHANNEL;
    uint8_t          status_ans    = 0x7;  // initialised for ans answer ok
    uint8_t          ch_mask_cntl_temp;
    uint16_t         channel_mask_temp = 0;
    uint8_t          dr_tmp;
    uint8_t          tx_power_tmp;
    uint8_t          nb_trans_tmp;

    for( uint8_t i = 0; i < nb_link_adr_req; i++ )
    {
        channel_mask_temp = lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + ( i * LINK_ADR_REQ_SIZE ) + 2] +
                            ( lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + ( i * LINK_ADR_REQ_SIZE ) + 3] << 8 );
        ch_mask_cntl_temp =
            ( lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + ( i * LINK_ADR_REQ_SIZE ) + 4] & 0x70 ) >> 4;
        SMTC_MODEM_HAL_TRACE_PRINTF( "%u - MULTIPLE LINK ADR REQ , channel mask = 0x%x , ChMAstCntl = 0x%x\n", i,
                                     channel_mask_temp, ch_mask_cntl_temp );
        statusChannel = smtc_real_build_channel_mask( lr1_mac, ch_mask_cntl_temp, channel_mask_temp );
        if( statusChannel == ERROR_CHANNEL_CNTL )
        {  // Test ChannelCNTL not defined
            status_ans &= 0x6;
            SMTC_MODEM_HAL_TRACE_WARNING( "INVALID CHANNEL CNTL \n" );
        }
    }
    /* Valid global channel mask  */
    if( statusChannel == ERROR_CHANNEL_MASK )
    {  // Test Channelmask enables a not defined channel or Channelmask = 0
        status_ans &= 0x6;
        SMTC_MODEM_HAL_TRACE_WARNING( "INVALID CHANNEL MASK \n" );
    }

    /* At This point global temporary channel mask is built and validated */

    if( lr1_mac->adr_mode_select != STATIC_ADR_MODE )
    {
        if( status_ans == 7 )  // mean none ERROR_CHANNEL_CNTL or  ERROR_CHANNEL_MASK so valid channel mask only bit
                               // 1 is really tested
        {
            // check if new proposal channel mask is compatible with our mobile distribution
            status = smtc_real_is_channel_mask_for_mobile_mode( lr1_mac );
            if( status == ERRORLORAWAN )
            {
                status_ans = 0x0;  // reject the cmd because even if the channel mask is valid it is not compatible
                                   // with our current adr distribution
                SMTC_MODEM_HAL_TRACE_WARNING( "INVALID CHANNEL MASK in Mobile Mode \n" );
            }
            else
            {
                // new channel mask is acceptable :
                smtc_real_set_channel_mask( lr1_mac );
                // set this flag at true to notify upper layer that end device has received a valid link adr
                lr1_mac->available_link_adr = true;
                status_ans                  = 0x7;
            }
        }
        else  // rejected because channel mask not valid
        {
            status_ans = 0x0;
            SMTC_MODEM_HAL_TRACE_WARNING( "INVALID CHANNEL MASK \n" );
        }
    }
    else  // static mode
    {
        /* Valid the last DataRate */
        dr_tmp =
            ( ( lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + ( ( nb_link_adr_req - 1 ) * LINK_ADR_REQ_SIZE ) + 1] &
                0xF0 ) >>
              4 );

        status = smtc_real_is_tx_dr_acceptable( lr1_mac, dr_tmp );
        if( status == ERRORLORAWAN )
        {  // Test Channelmask enables a not defined channel
            status_ans &= 0x5;
            SMTC_MODEM_HAL_TRACE_WARNING( "INVALID DATARATE \n" );
        }

        /* Valid the last TxPower  And Prepare Ans */
        tx_power_tmp =
            ( lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + ( ( nb_link_adr_req - 1 ) * LINK_ADR_REQ_SIZE ) + 1] &
              0x0F );
        status = smtc_real_is_tx_power_valid( lr1_mac, tx_power_tmp );
        if( status == ERRORLORAWAN )
        {  // Test tx power
            status_ans &= 0x3;
            SMTC_MODEM_HAL_TRACE_WARNING( "INVALID TXPOWER \n" );
        }

        nb_trans_tmp =
            ( lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + ( ( nb_link_adr_req - 1 ) * LINK_ADR_REQ_SIZE ) + 4] &
              0x0F );

        /* Update the mac parameters if case of no error */

        if( status_ans == 0x7 )
        {
            smtc_real_set_channel_mask( lr1_mac );
            smtc_real_set_power( lr1_mac, tx_power_tmp );
            lr1_mac->nb_trans           = nb_trans_tmp;
            lr1_mac->tx_data_rate_adr   = dr_tmp;
            lr1_mac->available_link_adr = true;
            SMTC_MODEM_HAL_TRACE_PRINTF( "MacTxDataRateAdr = %d\n", lr1_mac->tx_data_rate_adr );
            SMTC_MODEM_HAL_TRACE_PRINTF( "MacTxPower = %d\n", lr1_mac->tx_power );
            SMTC_MODEM_HAL_TRACE_PRINTF( "MacNbTrans = %d\n", lr1_mac->nb_trans );
        }
    }

    lr1_mac->nwk_payload_index += ( nb_link_adr_req * LINK_ADR_REQ_SIZE );

    /* Prepare repeated Ans*/
    if( lr1_mac->nwk_payload_index <= lr1_mac->nwk_payload_size )
    {
        for( uint8_t i = 0; i < nb_link_adr_req; i++ )
        {
            lr1_mac->tx_fopts_data[lr1_mac->tx_fopts_length + ( i * LINK_ADR_ANS_SIZE )] = LINK_ADR_ANS;  // copy Cid
            lr1_mac->tx_fopts_data[lr1_mac->tx_fopts_length + ( i * LINK_ADR_ANS_SIZE ) + 1] = status_ans;
        }
        lr1_mac->tx_fopts_length += ( nb_link_adr_req * LINK_ADR_ANS_SIZE );
    }
}

/**********************************************************************************************************************/
/*                                                 Private NWK MANAGEMENTS :
 * rx_param_setup_parser                       */
/**********************************************************************************************************************/

static void rx_param_setup_parser( lr1_stack_mac_t* lr1_mac )
{
    SMTC_MODEM_HAL_TRACE_PRINTF(
        " Cmd rx_param_setup_parser = %x %x %x %x \n", lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 1],
        lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 2], lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 3],
        lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 4] );
    int      status     = OKLORAWAN;
    uint8_t  status_ans = 0x7;  // initialised for ans answer ok
    uint8_t  rx1_dr_offset_temp;
    uint8_t  rx2_dr_temp;
    uint32_t rx2_frequency_temp;
    /* Valid Rx1DrOffset And Prepare Ans */
    rx1_dr_offset_temp = ( lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 1] & 0x70 ) >> 4;
    status             = smtc_real_is_rx1_dr_offset_valid( lr1_mac, rx1_dr_offset_temp );

    if( status == ERRORLORAWAN )
    {
        status_ans &= 0x6;
        SMTC_MODEM_HAL_TRACE_MSG( "INVALID RX1DROFFSET \n" );
    }

    /* Valid MacRx2Dr And Prepare Ans */
    status      = OKLORAWAN;
    rx2_dr_temp = ( lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 1] & 0x0F );
    status      = smtc_real_is_rx_dr_valid( lr1_mac, rx2_dr_temp );
    if( status == ERRORLORAWAN )
    {
        status_ans &= 0x5;
        SMTC_MODEM_HAL_TRACE_MSG( "INVALID RX2DR \n" );
    }

    /* Valid MacRx2Frequency And Prepare Ans */
    status = OKLORAWAN;

    rx2_frequency_temp =
        smtc_real_decode_freq_from_buf( lr1_mac, &lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 2] );

    status = smtc_real_is_rx_frequency_valid( lr1_mac, rx2_frequency_temp );
    if( status == ERRORLORAWAN )
    {
        status_ans &= 0x3;
        SMTC_MODEM_HAL_TRACE_MSG( "INVALID RX2 FREQUENCY \n" );
    }

    /* Update the mac parameters if case of no error */

    if( status_ans == 0x7 )
    {
        lr1_mac->rx1_dr_offset = rx1_dr_offset_temp;
        lr1_mac->rx2_data_rate = rx2_dr_temp;
        lr1_mac->rx2_frequency = rx2_frequency_temp;
        SMTC_MODEM_HAL_TRACE_PRINTF( "MacRx1DataRateOffset = %d\n", lr1_mac->rx1_dr_offset );
        SMTC_MODEM_HAL_TRACE_PRINTF( "MacRx2DataRate = %d\n", lr1_mac->rx2_data_rate );
        SMTC_MODEM_HAL_TRACE_PRINTF( "MacRx2Frequency = %d\n", lr1_mac->rx2_frequency );
    }

    lr1_mac->nwk_payload_index += RXPARRAM_SETUP_REQ_SIZE;

    /* Prepare Ans*/
    if( lr1_mac->nwk_payload_index <= lr1_mac->nwk_payload_size )
    {
        lr1_mac->tx_fopts_data[lr1_mac->tx_fopts_length]     = RXPARRAM_SETUP_ANS;
        lr1_mac->tx_fopts_data[lr1_mac->tx_fopts_length + 1] = status_ans;
        lr1_mac->tx_fopts_length += RXPARRAM_SETUP_ANS_SIZE;

        lr1_mac->tx_fopts_datasticky[lr1_mac->tx_fopts_lengthsticky]     = RXPARRAM_SETUP_ANS;
        lr1_mac->tx_fopts_datasticky[lr1_mac->tx_fopts_lengthsticky + 1] = status_ans;
        lr1_mac->tx_fopts_lengthsticky += RXPARRAM_SETUP_ANS_SIZE;
    }
}

/**********************************************************************************************************************/
/*                                                 Private NWK MANAGEMENTS :
 * duty_cycle_parser                          */
/**********************************************************************************************************************/

static void duty_cycle_parser( lr1_stack_mac_t* lr1_mac )
{
    SMTC_MODEM_HAL_TRACE_PRINTF( "Cmd duty_cycle_parser %x \n", lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 1] );
    lr1_mac->max_duty_cycle_index = ( lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 1] & 0x0F );

    lr1_mac->nwk_payload_index += DUTY_CYCLE_REQ_SIZE;
    /* Prepare Ans*/
    if( lr1_mac->nwk_payload_index <= lr1_mac->nwk_payload_size )
    {
        lr1_mac->tx_fopts_data[lr1_mac->tx_fopts_length] = DUTY_CYCLE_ANS;  // copy Cid
        lr1_mac->tx_fopts_length += DUTY_CYCLE_ANS_SIZE;
    }
}
/**********************************************************************************************************************/
/*                                                 Private NWK MANAGEMENTS :
 * dev_status_parser                          */
/**********************************************************************************************************************/

static void dev_status_parser( lr1_stack_mac_t* lr1_mac )
{
    uint8_t my_hook_id;
    rp_hook_get_id( lr1_mac->rp, lr1_mac, &my_hook_id );
    SMTC_MODEM_HAL_TRACE_MSG( "Receive a dev status req\n" );
    lr1_mac->nwk_payload_index += DEV_STATUS_REQ_SIZE;

    if( lr1_mac->nwk_payload_index <= lr1_mac->nwk_payload_size )
    {
        lr1_mac->tx_fopts_data[lr1_mac->tx_fopts_length]     = DEV_STATUS_ANS;  // copy Cid
        lr1_mac->tx_fopts_data[lr1_mac->tx_fopts_length + 1] = smtc_modem_hal_get_battery_level( );
        lr1_mac->tx_fopts_data[lr1_mac->tx_fopts_length + 2] =
            ( lr1_mac->rp->radio_params[my_hook_id].rx.lora_pkt_status.snr_pkt_in_db ) & 0x3F;
        lr1_mac->tx_fopts_length += DEV_STATUS_ANS_SIZE;
    }
}
/**********************************************************************************************************************/
/*                                                 Private NWK MANAGEMENTS :
 * new_channel_parser                         */
/**********************************************************************************************************************/
static void new_channel_parser( lr1_stack_mac_t* lr1_mac )
{
    SMTC_MODEM_HAL_TRACE_PRINTF(
        " Cmd new_channel_parser = %x %x %x %x %x \n", lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 1],
        lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 2], lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 3],
        lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 4], lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 5] );
    int      status     = OKLORAWAN;
    uint8_t  status_ans = 0x3;  // initialized for ans answer ok
    uint8_t  channel_index_temp;
    uint8_t  dr_range_max_temp;
    uint8_t  dr_range_min_temp;
    uint32_t frequency_temp;
    /* Valid Channel Index */
    channel_index_temp = lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 1];
    status             = smtc_real_is_channel_index_valid( lr1_mac, channel_index_temp );
    if( status == ERRORLORAWAN )
    {
        status_ans &= 0x0;
        SMTC_MODEM_HAL_TRACE_MSG( "INVALID CHANNEL INDEX \n" );
    }
    /* Valid Frequency  */
    frequency_temp = smtc_real_decode_freq_from_buf( lr1_mac, &lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 2] );
    status         = smtc_real_is_tx_frequency_valid( lr1_mac, frequency_temp );
    if( status == ERRORLORAWAN )
    {
        status_ans &= 0x2;
        SMTC_MODEM_HAL_TRACE_MSG( "INVALID FREQUENCY\n" );
    }
    /* Valid DRMIN/MAX */
    dr_range_min_temp = lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 5] & 0xF;
    status            = smtc_real_is_tx_dr_valid( lr1_mac, dr_range_min_temp );
    if( status == ERRORLORAWAN )
    {
        status_ans &= 0x1;
        SMTC_MODEM_HAL_TRACE_MSG( "INVALID DR MIN \n" );
    }
    dr_range_max_temp = ( lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 5] & 0xF0 ) >> 4;
    status            = smtc_real_is_tx_dr_valid( lr1_mac, dr_range_max_temp );
    if( status == ERRORLORAWAN )
    {
        status_ans &= 0x1;
        SMTC_MODEM_HAL_TRACE_MSG( "INVALID DR MAX \n" );
    }
    if( dr_range_max_temp < dr_range_min_temp )
    {
        status_ans &= 0x1;
        SMTC_MODEM_HAL_TRACE_MSG( "INVALID DR MAX < DR MIN \n" );
    }

    /* Update the mac parameters if case of no error */

    if( status_ans == 0x3 )
    {
        smtc_real_set_tx_frequency_channel( lr1_mac, frequency_temp, channel_index_temp );
        smtc_real_set_rx1_frequency_channel( lr1_mac, frequency_temp, channel_index_temp );

        smtc_real_set_channel_dr( lr1_mac, channel_index_temp, dr_range_min_temp, dr_range_max_temp );
        if( frequency_temp == 0 )
        {
            smtc_real_set_channel_enabled( lr1_mac, CHANNEL_DISABLED, channel_index_temp );
        }
        else
        {
            smtc_real_set_channel_enabled( lr1_mac, CHANNEL_ENABLED, channel_index_temp );
        }
        SMTC_MODEM_HAL_TRACE_PRINTF( "MacTxFrequency [ %d ] = %d, DrMin = %d, DrMax = %d\n", channel_index_temp,
                                     smtc_real_get_tx_channel_frequency( lr1_mac, channel_index_temp ),
                                     dr_range_min_temp, dr_range_max_temp );
    }

    lr1_mac->nwk_payload_index += NEW_CHANNEL_REQ_SIZE;

    /* Prepare Ans*/
    if( lr1_mac->nwk_payload_index <= lr1_mac->nwk_payload_size )
    {
        lr1_mac->tx_fopts_data[lr1_mac->tx_fopts_length]     = NEW_CHANNEL_ANS;  // copy Cid
        lr1_mac->tx_fopts_data[lr1_mac->tx_fopts_length + 1] = status_ans;
        lr1_mac->tx_fopts_length += NEW_CHANNEL_ANS_SIZE;
    }
}
/*********************************************************************************************************************/
/*                                                 Private NWK MANAGEMENTS :
 * rx_timing_setup_parser                     */
/*********************************************************************************************************************/

static void rx_timing_setup_parser( lr1_stack_mac_t* lr1_mac )
{
    SMTC_MODEM_HAL_TRACE_PRINTF( "Cmd rx_timing_setup_parser = %x \n",
                                 lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 1] );
    lr1_mac->rx1_delay_s = ( lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 1] & 0xF );
    if( lr1_mac->rx1_delay_s == 0 )
    {
        lr1_mac->rx1_delay_s = 1;  // Lorawan standart define 0 such as a delay of 1
    }

    lr1_mac->nwk_payload_index += RXTIMING_SETUP_REQ_SIZE;

    /* Prepare Ans*/
    if( lr1_mac->nwk_payload_index <= lr1_mac->nwk_payload_size )
    {
        lr1_mac->tx_fopts_data[lr1_mac->tx_fopts_length] = RXTIMING_SETUP_ANS;
        lr1_mac->tx_fopts_length += RXTIMING_SETUP_ANS_SIZE;

        lr1_mac->tx_fopts_datasticky[lr1_mac->tx_fopts_lengthsticky] = RXTIMING_SETUP_ANS;
        lr1_mac->tx_fopts_lengthsticky += RXTIMING_SETUP_ANS_SIZE;
    }
}

/*********************************************************************************************************************/
/*                                                 Private NWK MANAGEMENTS :
 * tx_param_setup_parser                  */
/*********************************************************************************************************************/

static void tx_param_setup_parser( lr1_stack_mac_t* lr1_mac )
{
    SMTC_MODEM_HAL_TRACE_PRINTF( "Cmd tx_param_setup_parser = %x \n",
                                 lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 1] );

    if( smtc_real_is_tx_param_setup_req_supported( lr1_mac ) == true )
    {
        lr1_mac->max_eirp_dbm =
            smtc_real_max_eirp_dbm_from_idx[( lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 1] & 0x0F )];
        lr1_mac->uplink_dwell_time   = ( lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 1] & 0x10 ) >> 4;
        lr1_mac->downlink_dwell_time = ( lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 1] & 0x20 ) >> 5;

        lr1_mac->nwk_payload_index += TXPARAM_SETUP_REQ_SIZE;

        /* Prepare Ans*/
        if( lr1_mac->nwk_payload_index <= lr1_mac->nwk_payload_size )
        {
            lr1_mac->tx_fopts_data[lr1_mac->tx_fopts_length] = TXPARAM_SETUP_ANS;
            lr1_mac->tx_fopts_length += TXPARAM_SETUP_ANS_SIZE;

            lr1_mac->tx_fopts_datasticky[lr1_mac->tx_fopts_lengthsticky] = TXPARAM_SETUP_ANS;  // copy Cid
            lr1_mac->tx_fopts_lengthsticky += TXPARAM_SETUP_ANS_SIZE;
        }
    }
    else
    {
        SMTC_MODEM_HAL_TRACE_WARNING( "TxParamSetupReq is not supported for this region\n" );
        lr1_mac->nwk_payload_index += TXPARAM_SETUP_REQ_SIZE;
    }
}

/*********************************************************************************************************************/
/*                                                 Private NWK MANAGEMENTS :
 * dl_channel_parser                        */
/*********************************************************************************************************************/

static void dl_channel_parser( lr1_stack_mac_t* lr1_mac )
{
    SMTC_MODEM_HAL_TRACE_PRINTF(
        "Cmd dl_channel_parser = %x %x %x %x  \n", lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 1],
        lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 2], lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 3],
        lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 4] );

    int      status     = OKLORAWAN;
    uint8_t  status_ans = 0x3;  // initialised for ans answer ok
    uint8_t  channel_index_temp;
    uint32_t frequency_temp;

    /* Valid Channel Index */
    channel_index_temp = lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 1];
    if( smtc_real_get_tx_channel_frequency( lr1_mac, channel_index_temp ) == 0 )
    {
        status_ans &= 0x1;
        SMTC_MODEM_HAL_TRACE_MSG( "INVALID CHANNEL INDEX \n" );
    }
    /* Valid Frequency  */
    frequency_temp = smtc_real_decode_freq_from_buf( lr1_mac, &lr1_mac->nwk_payload[lr1_mac->nwk_payload_index + 2] );
    status         = smtc_real_is_rx_frequency_valid( lr1_mac, frequency_temp );
    if( status == ERRORLORAWAN )
    {
        status_ans &= 0x2;
        SMTC_MODEM_HAL_TRACE_MSG( "INVALID FREQUENCY\n" );
    }
    /* Update the mac parameters if case of no error */
    if( status_ans == 0x3 )
    {
        if( smtc_real_set_rx1_frequency_channel( lr1_mac, frequency_temp, channel_index_temp ) != OKLORAWAN )
        {
            status_ans = 0;
        }
        SMTC_MODEM_HAL_TRACE_PRINTF( "MacRx1Frequency [ %u ] = %d\n", channel_index_temp,
                                     smtc_real_get_rx1_channel_frequency( lr1_mac, channel_index_temp ) );
    }

    lr1_mac->nwk_payload_index += DL_CHANNEL_REQ_SIZE;

    /* Prepare Ans*/
    if( lr1_mac->nwk_payload_index <= lr1_mac->nwk_payload_size )
    {
        lr1_mac->tx_fopts_data[lr1_mac->tx_fopts_length]     = DL_CHANNEL_ANS;
        lr1_mac->tx_fopts_data[lr1_mac->tx_fopts_length + 1] = status_ans;
        lr1_mac->tx_fopts_length += DL_CHANNEL_ANS_SIZE;

        lr1_mac->tx_fopts_datasticky[lr1_mac->tx_fopts_lengthsticky]     = DL_CHANNEL_ANS;
        lr1_mac->tx_fopts_datasticky[lr1_mac->tx_fopts_lengthsticky + 1] = status_ans;
        lr1_mac->tx_fopts_lengthsticky += DL_CHANNEL_ANS_SIZE;
    }
}

status_lorawan_t lr1mac_rx_payload_max_size_check( lr1_stack_mac_t* lr1_mac, uint8_t size )
{
    uint8_t sf;
    uint8_t bw;
    uint8_t dr;
    uint8_t size_max;
    if( lr1_mac->current_win == RX1 )
    {
        if( lr1_mac->rx1_modulation_type == FSK )
        {
            sf = 50;
            bw = BW125;
        }
        else
        {
            sf = lr1_mac->rx1_sf;
            bw = lr1_mac->rx1_bw;
        }
    }
    else
    {
        if( lr1_mac->rx2_modulation_type == FSK )
        {
            sf = 50;
            bw = BW125;
        }
        else
        {
            sf = lr1_mac->rx2_sf;
            bw = lr1_mac->rx2_bw;
        }
    }
    dr       = smtc_real_sf_bw_to_dr( lr1_mac, sf, bw );
    size_max = smtc_real_get_max_payload_size( lr1_mac, dr, lr1_mac->downlink_dwell_time ) + 1 + 4;  // + MHDR + CRC
    if( size > size_max )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "size (%d)> size_max (%d)", size, size_max );
        return ERRORLORAWAN;
    }
    else
    {
        return OKLORAWAN;
    }
}