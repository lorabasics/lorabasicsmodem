/*!
 * \file      test_mode.c
 *
 * \brief     handle downlink from device management
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

#include "test_mode.h"
#include "modem_api.h"
#include "radio_planner.h"
#include "lorawan_api.h"
#include "lr1mac_core.h"
#include "smtc_real.h"
#include "smtc_bsp.h"

#if defined( REGION_WW2G4 )
#include "sx1280_hal.h"
#elif defined( REGION_EU_868 ) || defined( REGION_US_915 )
#include "sx126x_hal.h"
#else
#error "Please select region.."
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */
static bool is_tst_mode_enabled = false;

typedef struct context
{
    radio_planner_t*  rp;
    ral_params_lora_t params;
} context_t;

static context_t context_test;

static uint8_t          test_mode_hook_id = 0xFF;
static radio_planner_t* test_mode_rp;
static lr1_stack_mac_t* lr1_mac_obj = { 0 };

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */
static inline void test_mode_enable( void )
{
    is_tst_mode_enabled = true;
}
static inline void test_mode_disable( void )
{
    is_tst_mode_enabled = false;
}

void test_mode_tx_whitout_callback( context_t* context )
{
    BSP_DBG_TRACE_PRINTF( "TEST mode TX Empty callback\n" );
}

void test_mode_tx_callback( context_t* context )
{
    uint8_t my_hook_id;
    rp_hook_get_id( context->rp, context, &my_hook_id );
    BSP_DBG_TRACE_PRINTF( "TEST mode TX hook id %u\n", my_hook_id );

    rp_radio_params_t radio_params;
    radio_params.pkt_type                 = RAL_PKT_TYPE_LORA;
    radio_params.tx.lora.bw               = context->params.bw;
    radio_params.tx.lora.cr               = context->params.cr;
    radio_params.tx.lora.sf               = context->params.sf;
    radio_params.tx.lora.crc_is_on        = context->params.crc_is_on;
    radio_params.tx.lora.freq_in_hz       = context->params.freq_in_hz;
    radio_params.tx.lora.invert_iq_is_on  = context->params.invert_iq_is_on;
    radio_params.tx.lora.pbl_len_in_symb  = context->params.pbl_len_in_symb;
    radio_params.tx.lora.pld_is_fix       = context->params.pld_is_fix;
    radio_params.tx.lora.pld_len_in_bytes = context->params.pld_len_in_bytes;
    radio_params.tx.lora.pwr_in_dbm       = context->params.pwr_in_dbm;
    radio_params.tx.lora.sync_word        = context->params.sync_word;

    rp_task_t rp_task;
    rp_task.hook_id          = my_hook_id;
    rp_task.duration_time_ms = 2000;  // toa;
    rp_task.state            = RP_TASK_STATE_ASAP;
    rp_task.type             = RP_TASK_TYPE_TX_LORA;
    rp_task.start_time_ms    = bsp_rtc_get_time_ms( ) + 2;
    uint8_t tx_payload[255]  = { 0 };

    rp_task_enqueue( context->rp, &rp_task, tx_payload, context->params.pld_len_in_bytes, &radio_params );
}
static uint8_t rx_payload[255] = { 0 };

void test_mode_rx_callback( context_t* context )
{
    uint8_t my_hook_id;
    rp_hook_get_id( context->rp, context, &my_hook_id );
    BSP_DBG_TRACE_PRINTF( "------------------------\nTEST mode RX hook id %u\n", my_hook_id );

    rp_radio_params_t radio_params;
    radio_params.pkt_type                 = RAL_PKT_TYPE_LORA;
    radio_params.rx.lora.bw               = context->params.bw;
    radio_params.rx.lora.cr               = context->params.cr;
    radio_params.rx.lora.sf               = context->params.sf;
    radio_params.rx.lora.crc_is_on        = context->params.crc_is_on;
    radio_params.rx.lora.freq_in_hz       = context->params.freq_in_hz;
    radio_params.rx.lora.invert_iq_is_on  = context->params.invert_iq_is_on;
    radio_params.rx.lora.pbl_len_in_symb  = context->params.pbl_len_in_symb;
    radio_params.rx.lora.pld_is_fix       = context->params.pld_is_fix;
    radio_params.rx.lora.pld_len_in_bytes = context->params.pld_len_in_bytes;
    radio_params.rx.lora.symb_nb_timeout  = context->params.symb_nb_timeout;
    radio_params.rx.timeout_in_ms         = 0xFFFFFFFF;
    radio_params.rx.lora.sync_word        = context->params.sync_word;

    rp_task_t rp_task;
    rp_task.hook_id          = my_hook_id;
    rp_task.type             = RP_TASK_TYPE_RX_LORA;
    rp_task.state            = RP_TASK_STATE_ASAP;
    rp_task.start_time_ms    = bsp_rtc_get_time_ms( ) + 2;
    rp_task.duration_time_ms = 2000;  // toa;

    int16_t snr  = context->rp->radio_params[my_hook_id].rx.lora_pkt_status.snr_pkt_in_db;
    int16_t rssi = context->rp->radio_params[my_hook_id].rx.lora_pkt_status.rssi_pkt_in_dbm;

    uint32_t    irq_timestamp_ms = context->rp->irq_timestamp_ms[my_hook_id];
    rp_status_t rp_status        = context->rp->status[my_hook_id];

    BSP_DBG_TRACE_PRINTF( "t: %lu, rp_status %u, snr: %d, rssi: %d\n", irq_timestamp_ms, rp_status, snr, rssi );
    BSP_DBG_TRACE_ARRAY( "rx_payload", rx_payload, context->rp->payload_size[my_hook_id] )

    rp_task_enqueue( context->rp, &rp_task, rx_payload, context->params.pld_len_in_bytes, &radio_params );
}

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

uint8_t test_mode_enabled_is( void )
{
    return ( is_tst_mode_enabled == true ) ? true : false;
}

modem_return_code_t test_mode_start( void )
{
    test_mode_enable( );
    BSP_DBG_TRACE_INFO( "TST MODE: START\n" );

    test_mode_rp = modem_get_radio_planner( );

    lorawan_api_init( test_mode_rp );
    lr1_mac_obj       = lr1mac_core_stack_mac_get( );
    test_mode_hook_id = lorawan_api_rp_hook_id_get( );
    return RC_OK;
}

modem_return_code_t test_mode_nop( void )
{
    rp_task_abort( test_mode_rp, test_mode_hook_id );
    if( ral_init( test_mode_rp->ral ) != RAL_STATUS_OK )
    {
        return RC_FAIL;
    }
    if( ral_set_sleep( test_mode_rp->ral ) != RAL_STATUS_OK )
    {
        return RC_FAIL;
    }
    return RC_OK;
}

modem_return_code_t test_mode_tx( uint32_t frequency, int8_t pwr_in_dbm, host_cmd_test_sf_t sf, host_cmd_test_bw_t bw,
                                  host_cmd_test_cr_t cr, uint8_t payload_length, test_mode_tx_mode_t tx_mode )
{
    if( smtc_real_is_valid_rx_frequency( lr1_mac_obj, frequency ) != OKLORAWAN )  // current value are for 2G4 only
    {
        BSP_DBG_TRACE_ERROR( " Invalid Frequency %lu\n", frequency );
        return RC_INVALID;
    }
    if( sf >= TST_SF_MAX )
    {
        BSP_DBG_TRACE_ERROR( " Invalid sf %lu\n", sf );
        return RC_INVALID;
    }
    if( bw >= TST_BW_MAX )
    {
        BSP_DBG_TRACE_ERROR( " Invalid bw %lu\n", bw );
        return RC_INVALID;
    }
    if( cr >= TST_CR_MAX )
    {
        BSP_DBG_TRACE_ERROR( " Invalid cr %lu\n", cr );
        return RC_INVALID;
    }

    ral_params_lora_t params;
    params.freq_in_hz       = frequency;
    params.pwr_in_dbm       = pwr_in_dbm;
    params.sf               = ( ral_lora_sf_t ) host_cmd_test_sf_convert[sf];
    params.bw               = ( ral_lora_bw_t ) host_cmd_test_bw_convert[bw];
    params.cr               = ( ral_lora_cr_t ) host_cmd_test_cr_convert[cr];
    params.pld_len_in_bytes = payload_length;

    params.pbl_len_in_symb = 8;
    params.crc_is_on       = false;
    params.invert_iq_is_on = false;
    params.pld_is_fix      = false;
    params.sync_word       = smtc_real_sync_word_get( lr1_mac_obj );

    BSP_DBG_TRACE_PRINTF( "Tx - Freq:%lu, Power:%d, sf:%u, bw:%u, cr:%u, length:%u\n", params.freq_in_hz,
                          params.pwr_in_dbm, params.sf, params.bw, params.cr, params.pld_len_in_bytes );

    context_test.params = params;
    context_test.rp     = test_mode_rp;

    rp_task_abort( test_mode_rp, test_mode_hook_id );
    if( ral_init( test_mode_rp->ral ) != RAL_STATUS_OK )
    {
        return RC_FAIL;
    }

    if( tx_mode == TEST_MODE_TX_SINGLE )
    {
        rp_hook_init( context_test.rp, test_mode_hook_id, ( void ( * )( void* ) )( test_mode_tx_whitout_callback ),
                      &context_test );
    }
    else
    {
        rp_hook_init( context_test.rp, test_mode_hook_id, ( void ( * )( void* ) )( test_mode_tx_callback ),
                      &context_test );
    }
    test_mode_tx_callback( &context_test );
    return RC_OK;
}

modem_return_code_t test_mode_cw( uint32_t frequency, int8_t pwr_in_dbm )
{
    if( smtc_real_is_valid_rx_frequency( lr1_mac_obj, frequency ) != OKLORAWAN )
    {
        BSP_DBG_TRACE_ERROR( " Invalid Frequency %lu\n", frequency );
        return RC_INVALID;
    }

    ral_params_lora_t params;
    params.freq_in_hz = frequency;
    params.pwr_in_dbm = pwr_in_dbm;
    params.sf         = RAL_LORA_SF12;
    params.bw         = RAL_LORA_BW_800_KHZ;
    params.cr         = smtc_real_coding_rate_get( lr1_mac_obj );
    params.sync_word  = smtc_real_sync_word_get( lr1_mac_obj );

    BSP_DBG_TRACE_PRINTF( "Tx CW - Freq:%lu, Power:%d\n", frequency, pwr_in_dbm );
    if( ral_init( test_mode_rp->ral ) != RAL_STATUS_OK )
    {
        return RC_FAIL;
    }
    if( ral_setup_tx_lora( test_mode_rp->ral, &params ) != RAL_STATUS_OK )
    {
        return RC_FAIL;
    }
    if( ral_set_tx_cw( test_mode_rp->ral ) != RAL_STATUS_OK )
    {
        return RC_FAIL;
    }
    return RC_OK;
}

modem_return_code_t test_mode_rx_cont( uint32_t frequency, host_cmd_test_sf_t sf, host_cmd_test_bw_t bw,
                                       host_cmd_test_cr_t cr )
{
    if( smtc_real_is_valid_rx_frequency( lr1_mac_obj, frequency ) != OKLORAWAN )  // current value are for 2G4 only
    {
        BSP_DBG_TRACE_ERROR( " Invalid Frequency %lu\n", frequency );
        return RC_INVALID;
    }
    if( sf >= TST_SF_MAX )
    {
        BSP_DBG_TRACE_ERROR( " Invalid sf %lu\n", sf );
        return RC_INVALID;
    }
    if( bw >= TST_BW_MAX )
    {
        BSP_DBG_TRACE_ERROR( " Invalid bw %lu\n", bw );
        return RC_INVALID;
    }
    if( cr >= TST_CR_MAX )
    {
        BSP_DBG_TRACE_ERROR( " Invalid cr %lu\n", cr );
        return RC_INVALID;
    }

    ral_params_lora_t params;
    params.freq_in_hz       = frequency;
    params.sf               = ( ral_lora_sf_t ) host_cmd_test_sf_convert[sf];
    params.bw               = ( ral_lora_bw_t ) host_cmd_test_bw_convert[bw];
    params.cr               = ( ral_lora_cr_t ) host_cmd_test_cr_convert[cr];
    params.crc_is_on        = true;
    params.invert_iq_is_on  = true;
    params.pbl_len_in_symb  = 8;
    params.pld_is_fix       = false;
    params.pld_len_in_bytes = 255;
    params.symb_nb_timeout  = 8;
    params.sync_word        = smtc_real_sync_word_get( lr1_mac_obj );

    context_test.params = params;
    context_test.rp     = test_mode_rp;
    BSP_DBG_TRACE_PRINTF( "Tx - Freq:%lu, sf:%u, bw:%u, cr:%u\n", params.freq_in_hz, params.sf, params.bw, params.cr );

    rp_task_abort( context_test.rp, test_mode_hook_id );
    if( ral_init( test_mode_rp->ral ) != RAL_STATUS_OK )
    {
        return RC_FAIL;
    }
    rp_hook_init( context_test.rp, test_mode_hook_id, ( void ( * )( void* ) )( test_mode_rx_callback ), &context_test );
    test_mode_rx_callback( &context_test );
    return RC_OK;
}

modem_return_code_t test_mode_radio_reset( void )
{
    if( ral_init( test_mode_rp->ral ) != RAL_STATUS_OK )
    {
        return RC_FAIL;
    }
    if( ral_set_sleep( test_mode_rp->ral ) != RAL_STATUS_OK )
    {
        return RC_FAIL;
    }
    return RC_OK;
}

modem_return_code_t test_mode_exit( void )
{
    rp_task_abort( test_mode_rp, test_mode_hook_id );
    lorawan_api_init( test_mode_rp );
    test_mode_disable( );
    return RC_OK;
}

modem_return_code_t test_mode_radio_write( uint8_t* command, uint16_t command_length, uint8_t* data,
                                           uint16_t data_length )
{
#if defined( SX1280 )
    if( sx1280_hal_write( test_mode_rp->ral->context, command, command_length, data, data_length ) !=
        SX1280_HAL_STATUS_OK )

#elif defined( SX126X )
    if( sx126x_hal_write( test_mode_rp->ral->context, command, command_length, data, data_length ) !=
        SX126X_HAL_STATUS_OK )
#else
#error "Please select radio board.."
#endif
    {
        return RC_FAIL;
    }
    return RC_OK;
}

modem_return_code_t test_mode_radio_read( uint8_t* command, uint16_t command_length, uint8_t* data,
                                          uint16_t data_length )
{
#if defined( SX1280 )
    if( sx1280_hal_read( test_mode_rp->ral->context, command, command_length, data, data_length ) !=
        SX1280_HAL_STATUS_OK )
#elif defined( SX126X )
    if( sx126x_hal_read( test_mode_rp->ral->context, command, command_length, data, data_length ) !=
        SX126X_HAL_STATUS_OK )
#else
#error "Please select radio board.."
#endif
    {
        return RC_FAIL;
    }
    return RC_OK;
}
