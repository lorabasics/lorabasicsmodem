/**
 * @file      ral_sx126x.c
 *
 * @brief     SX126x radio abstraction layer implementation
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

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <string.h>  // memcpy

#include "sx126x.h"
#include "ral_sx126x.h"

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

const sx126x_lora_bw_t ral_to_sx126x_lora_bw[] = {
    SX126X_LORA_BW_007,
    SX126X_LORA_BW_010,
    SX126X_LORA_BW_015,
    SX126X_LORA_BW_020,
    SX126X_LORA_BW_031,
    SX126X_LORA_BW_041,
    SX126X_LORA_BW_062,
    SX126X_LORA_BW_125,
    0xFF,
    SX126X_LORA_BW_250,
    0xFF,
    SX126X_LORA_BW_500,
    0xFF,
    0xFF,
};

const sx126x_lora_cr_t ral_to_sx126x_lora_cr[] = {
    SX126X_LORA_CR_4_5, SX126X_LORA_CR_4_6, SX126X_LORA_CR_4_7, SX126X_LORA_CR_4_8, 0xFF, 0xFF, 0xFF,
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

static ral_status_t ral_sx126x_convert_gfsk_params_from_radio( const ral_params_gfsk_t*  params,
                                                               sx126x_mod_params_gfsk_t* mod_params,
                                                               sx126x_pkt_params_gfsk_t* pkt_params );

static ral_status_t ral_sx126x_convert_lora_params_from_radio( const ral_params_lora_t*  params,
                                                               sx126x_mod_params_lora_t* mod_params,
                                                               sx126x_pkt_params_lora_t* pkt_params );

static ral_irq_t ral_sx126x_convert_irq_flags_to_radio( sx126x_irq_mask_t sx126x_irq );

static sx126x_irq_mask_t ral_sx126x_convert_irq_flags_from_radio( ral_irq_t ral_irq );

static ral_status_t ral_sx126x_convert_tcxo_ctrl_to_radio( ral_tcxo_ctrl_voltages_t     ral_tcxo_ctrl_voltage,
                                                           sx126x_tcxo_ctrl_voltages_t* sx126x_tcxo_ctrl_voltage );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

ral_status_t ral_sx126x_init( const ral_t* ral )
{
    ral_status_t status = RAL_STATUS_ERROR;

    sx126x_reset( ral->context );

    status = sx126x_set_standby( ral->context, SX126X_STANDBY_CFG_RC );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }
    status = sx126x_set_reg_mode( ral->context, SX126X_REG_MODE_DCDC );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

    if( ral->tcxo_cfg.tcxo_ctrl_mode == RAL_TCXO_CTRL_RADIO )
    {
        sx126x_tcxo_ctrl_voltages_t tcxo_ctrl_voltage;

        status = ral_sx126x_convert_tcxo_ctrl_to_radio( ral->tcxo_cfg.tcxo_ctrl_voltage, &tcxo_ctrl_voltage );
        if( status != RAL_STATUS_OK )
        {
            return status;
        }

        status = sx126x_set_dio3_as_tcxo_ctrl( ral->context, tcxo_ctrl_voltage, ral->tcxo_cfg.tcxo_startup_time_ms );
    }

    return status;
}

ral_status_t ral_sx126x_setup_gfsk( const ral_t* ral, const ral_params_gfsk_t* params )
{
    ral_status_t status = RAL_STATUS_ERROR;

    sx126x_mod_params_gfsk_t mod_params;
    sx126x_pkt_params_gfsk_t pkt_params;

    status = ral_sx126x_convert_gfsk_params_from_radio( params, &mod_params, &pkt_params );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }
    status = sx126x_set_standby( ral->context, SX126X_STANDBY_CFG_RC );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }
    status = sx126x_stop_tmr_on_pbl( ral->context, false );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }
    status = sx126x_set_pkt_type( ral->context, SX126X_PKT_TYPE_GFSK );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }
    status = sx126x_set_rf_freq( ral->context, params->freq_in_hz );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }
    status = sx126x_set_tx_params( ral->context, params->pwr_in_dbm, SX126X_RAMP_200_US );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }
    status = sx126x_set_buffer_base_addr( ral->context, 0x00, 0x00 );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }
    status = sx126x_set_gfsk_mod_params( ral->context, &mod_params );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }
    status = sx126x_set_gfsk_pkt_params( ral->context, &pkt_params );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }
    if( params->crc_type != RAL_GFSK_CRC_OFF )
    {
        status = sx126x_set_gfsk_crc_seed( ral->context, params->crc_seed );
        if( status != RAL_STATUS_OK )
        {
            return status;
        }
        status = sx126x_set_gfsk_crc_polynomial( ral->context, params->crc_polynomial );
        if( status != RAL_STATUS_OK )
        {
            return status;
        }
    }
    status = sx126x_set_gfsk_sync_word( ral->context, params->sync_word, params->sync_word_len_in_bytes );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }
    if( params->dc_free_is_on == true )
    {
        status = sx126x_set_gfsk_whitening_seed( ral->context, params->whitening_seed );
        if( status != RAL_STATUS_OK )
        {
            return status;
        }
    }
    return status;
}

ral_status_t ral_sx126x_setup_rx_gfsk( const ral_t* ral, const ral_params_gfsk_t* params )
{
    ral_status_t status = RAL_STATUS_ERROR;

    status = ral_sx126x_setup_gfsk( ral, params );
    if( status == RAL_STATUS_OK )
    {
        status = ( ral_status_t ) sx126x_set_dio_irq_params(
            ral->context, SX126X_IRQ_RX_DONE | SX126X_IRQ_CRC_ERROR | SX126X_IRQ_TIMEOUT,
            SX126X_IRQ_RX_DONE | SX126X_IRQ_CRC_ERROR | SX126X_IRQ_TIMEOUT, SX126X_IRQ_NONE, SX126X_IRQ_NONE );
        if( status != RAL_STATUS_OK )
        {
            return status;
        }
    }
    return status;
}

ral_status_t ral_sx126x_setup_tx_gfsk( const ral_t* ral, const ral_params_gfsk_t* params )
{
    ral_status_t status = RAL_STATUS_ERROR;

    status = ral_sx126x_setup_gfsk( ral, params );
    if( status == RAL_STATUS_OK )
    {
        status = ( ral_status_t ) sx126x_set_dio_irq_params( ral->context, SX126X_IRQ_TX_DONE, SX126X_IRQ_TX_DONE,
                                                             SX126X_IRQ_NONE, SX126X_IRQ_NONE );
    }
    return status;
}

ral_status_t ral_sx126x_setup_lora( const ral_t* ral, const ral_params_lora_t* params )
{
    ral_status_t status = RAL_STATUS_ERROR;

    sx126x_mod_params_lora_t mod_params;
    sx126x_pkt_params_lora_t pkt_params;

    status = ral_sx126x_convert_lora_params_from_radio( params, &mod_params, &pkt_params );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }
    status = sx126x_set_standby( ral->context, SX126X_STANDBY_CFG_RC );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }
    status = sx126x_stop_tmr_on_pbl( ral->context, false );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }
    status = sx126x_set_lora_symb_nb_timeout( ral->context, params->symb_nb_timeout );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }
    status = sx126x_set_pkt_type( ral->context, SX126X_PKT_TYPE_LORA );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }
    status = sx126x_set_rf_freq( ral->context, params->freq_in_hz );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }
    status = sx126x_set_tx_params( ral->context, params->pwr_in_dbm, SX126X_RAMP_200_US );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }
    status = sx126x_set_buffer_base_addr( ral->context, 0x00, 0x00 );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }
    status = sx126x_set_lora_mod_params( ral->context, &mod_params );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }
    status = sx126x_set_lora_pkt_params( ral->context, &pkt_params );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }
    status = sx126x_set_lora_sync_word( ral->context, params->sync_word );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }
    return status;
}

ral_status_t ral_sx126x_setup_rx_lora( const ral_t* ral, const ral_params_lora_t* params )
{
    ral_status_t status = RAL_STATUS_ERROR;

    status = ral_sx126x_setup_lora( ral, params );
    if( status == RAL_STATUS_OK )
    {
        status = ( ral_status_t ) sx126x_set_dio_irq_params(
            ral->context, SX126X_IRQ_RX_DONE | SX126X_IRQ_HEADER_ERROR | SX126X_IRQ_CRC_ERROR | SX126X_IRQ_TIMEOUT,
            SX126X_IRQ_RX_DONE | SX126X_IRQ_HEADER_ERROR | SX126X_IRQ_CRC_ERROR | SX126X_IRQ_TIMEOUT, SX126X_IRQ_NONE,
            SX126X_IRQ_NONE );
        if( status != RAL_STATUS_OK )
        {
            return status;
        }
    }
    return status;
}

ral_status_t ral_sx126x_setup_tx_lora( const ral_t* ral, const ral_params_lora_t* params )
{
    ral_status_t status = RAL_STATUS_ERROR;

    status = ral_sx126x_setup_lora( ral, params );
    if( status == RAL_STATUS_OK )
    {
        status = ( ral_status_t ) sx126x_set_dio_irq_params( ral->context, SX126X_IRQ_TX_DONE, SX126X_IRQ_TX_DONE,
                                                             SX126X_IRQ_NONE, SX126X_IRQ_NONE );
    }
    return status;
}

ral_status_t ral_sx126x_setup_flrc( const ral_t* ral, const ral_params_flrc_t* params )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_sx126x_setup_rx_flrc( const ral_t* ral, const ral_params_flrc_t* params )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_sx126x_setup_tx_flrc( const ral_t* ral, const ral_params_flrc_t* params )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_sx126x_setup_tx_lora_e( const ral_t* ral, const ral_params_lora_e_t* params )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_sx126x_tx_bpsk( const ral_t* ral, const ral_params_bpsk_t* params )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_sx126x_setup_cad( const ral_t* ral, const ral_lora_cad_params_t* params )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_sx126x_set_pkt_payload( const ral_t* ral, const uint8_t* buffer, const uint16_t size )
{
    return ( ral_status_t ) sx126x_write_buffer( ral->context, 0x00, buffer, size );
}

ral_status_t ral_sx126x_get_pkt_payload( const ral_t* ral, uint8_t* buffer, uint16_t max_size, uint16_t* size )
{
    ral_status_t              status = RAL_STATUS_ERROR;
    sx126x_rx_buffer_status_t sx_buf_status;

    status = ( ral_status_t ) sx126x_get_rx_buffer_status( ral->context, &sx_buf_status );
    if( status == RAL_STATUS_OK )
    {
        if( size != 0 )
        {
            *size = sx_buf_status.pld_len_in_bytes;
        }
        if( sx_buf_status.pld_len_in_bytes <= max_size )
        {
            status = ( ral_status_t ) sx126x_read_buffer( ral->context, sx_buf_status.buffer_start_pointer, buffer,
                                                          sx_buf_status.pld_len_in_bytes );
        }
        else
        {
            status = RAL_STATUS_ERROR;
        }
    }
    return status;
}

ral_status_t ral_sx126x_get_gfsk_pkt_status( const ral_t* ral, ral_rx_pkt_status_gfsk_t* pkt_status )
{
    ral_status_t status = RAL_STATUS_ERROR;

    sx126x_pkt_status_gfsk_t sx_pkt_status;

    status = ( ral_status_t ) sx126x_get_gfsk_pkt_status( ral->context, &sx_pkt_status );

    pkt_status->rx_status        = sx_pkt_status.rx_status;
    pkt_status->rssi_sync_in_dbm = sx_pkt_status.rssi_sync;
    pkt_status->rssi_avg_in_dbm  = sx_pkt_status.rssi_avg;

    return status;
}

ral_status_t ral_sx126x_get_lora_pkt_status( const ral_t* ral, ral_rx_pkt_status_lora_t* pkt_status )
{
    ral_status_t status = RAL_STATUS_ERROR;

    sx126x_pkt_status_lora_t sx_pkt_status;

    status = ( ral_status_t ) sx126x_get_lora_pkt_status( ral->context, &sx_pkt_status );

    pkt_status->rssi_pkt_in_dbm       = sx_pkt_status.rssi_pkt_in_dbm;
    pkt_status->snr_pkt_in_db         = sx_pkt_status.snr_pkt_in_db;
    pkt_status->signal_rssi_pkt_in_db = sx_pkt_status.signal_rssi_pkt_in_dbm;

    return status;
}

ral_status_t ral_sx126x_get_flrc_pkt_status( const ral_t* ral, ral_rx_pkt_status_flrc_t* pkt_status )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_sx126x_set_sleep( const ral_t* ral )
{
    return ( ral_status_t ) sx126x_set_sleep( ral->context, SX126X_SLEEP_CFG_WARM_START );
}

ral_status_t ral_sx126x_set_standby( const ral_t* ral )
{
    return ( ral_status_t ) sx126x_set_standby( ral->context, SX126X_STANDBY_CFG_RC );
}

ral_status_t ral_sx126x_set_tx( const ral_t* ral )
{
    // Set no timeout.
    return ( ral_status_t ) sx126x_set_tx( ral->context, 0 );
}

ral_status_t ral_sx126x_set_tx_cw( const ral_t* ral )
{
    return ( ral_status_t ) sx126x_set_tx_cw( ral->context );
}

ral_status_t ral_sx126x_set_rx( const ral_t* ral, const uint32_t timeout_ms )
{
    if( timeout_ms == 0xFFFFFFFF )
    {
        return ( ral_status_t ) sx126x_set_rx( ral->context, timeout_ms );
    }
    else
    {
        return ( ral_status_t ) sx126x_set_rx( ral->context, timeout_ms << 6 );
    }
}

ral_status_t ral_sx126x_set_cad( const ral_t* ral )
{
    return ( ral_status_t ) sx126x_set_cad( ral->context );
}

ral_status_t ral_sx126x_get_irq_status( const ral_t* ral, ral_irq_t* ral_irq )
{
    ral_status_t      status          = RAL_STATUS_ERROR;
    sx126x_irq_mask_t sx126x_irq_mask = SX126X_IRQ_NONE;

    status = ( ral_status_t ) sx126x_get_irq_status( ral->context, &sx126x_irq_mask );
    if( status == RAL_STATUS_OK )
    {
        *ral_irq = ral_sx126x_convert_irq_flags_to_radio( sx126x_irq_mask );
    }
    return status;
}

ral_status_t ral_sx126x_clear_irq_status( const ral_t* ral, const ral_irq_t ral_irq )
{
    sx126x_irq_mask_t sx126x_irq_mask = ral_sx126x_convert_irq_flags_from_radio( ral_irq );

    if( sx126x_irq_mask != 0 )
    {
        return ( ral_status_t ) sx126x_clear_irq_status( ral->context, sx126x_irq_mask );
    }
    else
    {
        return RAL_STATUS_OK;
    }
}

ral_status_t ral_sx126x_get_and_clear_irq_status( const ral_t* ral, ral_irq_t* ral_irq )
{
    ral_irq_t         ral_irq_mask;
    ral_status_t      status          = RAL_STATUS_ERROR;
    sx126x_irq_mask_t sx126x_irq_mask = SX126X_IRQ_NONE;

    status = ( ral_status_t ) sx126x_get_and_clear_irq_status( ral->context, &sx126x_irq_mask );

    if( status == RAL_STATUS_OK )
    {
        ral_irq_mask = ral_sx126x_convert_irq_flags_to_radio( sx126x_irq_mask );
        if( ral_irq != NULL )
        {
            *ral_irq = ral_irq_mask;
        }
    }
    return status;
}

ral_status_t ral_sx126x_set_dio_irq_params( const ral_t* ral, const ral_irq_t ral_irq )
{
    uint16_t sx126x_irq = ral_sx126x_convert_irq_flags_from_radio( ral_irq );

    return ( ral_status_t ) sx126x_set_dio_irq_params( ral->context, sx126x_irq, sx126x_irq, SX126X_IRQ_NONE,
                                                       SX126X_IRQ_NONE );
}

ral_status_t ral_sx126x_get_rssi( const ral_t* ral, int16_t* rssi )
{
    return ( ral_status_t ) sx126x_get_rssi_inst( ral->context, rssi );
}

ral_status_t ral_sx126x_get_lora_time_on_air_in_ms( const ral_params_lora_t* params, uint32_t* toa )
{
    ral_status_t             status = RAL_STATUS_ERROR;
    sx126x_mod_params_lora_t mod_params;
    sx126x_pkt_params_lora_t pkt_params;

    status = ral_sx126x_convert_lora_params_from_radio( params, &mod_params, &pkt_params );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

    *toa = sx126x_get_lora_time_on_air_in_ms( &pkt_params, &mod_params );

    return RAL_STATUS_OK;
}

ral_status_t ral_sx126x_get_gfsk_time_on_air_in_ms( const ral_params_gfsk_t* params, uint32_t* toa )
{
    ral_status_t             status = RAL_STATUS_ERROR;
    sx126x_mod_params_gfsk_t mod_params;
    sx126x_pkt_params_gfsk_t pkt_params;

    status = ral_sx126x_convert_gfsk_params_from_radio( params, &mod_params, &pkt_params );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

    *toa = sx126x_get_gfsk_time_on_air_in_ms( &pkt_params, &mod_params );

    return RAL_STATUS_OK;
}

ral_status_t ral_sx126x_read_register( const ral_t* ral, uint16_t address, uint8_t* buffer, uint16_t size )
{
    return ( ral_status_t ) sx126x_read_register( ral->context, address, buffer, size );
}

ral_status_t ral_sx126x_write_register( const ral_t* ral, uint16_t address, uint8_t* buffer, uint16_t size )
{
    return ( ral_status_t ) sx126x_write_register( ral->context, address, buffer, size );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static ral_status_t ral_sx126x_convert_gfsk_params_from_radio( const ral_params_gfsk_t*  params,
                                                               sx126x_mod_params_gfsk_t* mod_params,
                                                               sx126x_pkt_params_gfsk_t* pkt_params )
{
    ral_status_t status = RAL_STATUS_ERROR;

    uint32_t                 bw_dsb_in_hz = params->bw_ssb_in_hz << 1;
    uint8_t                  bw_dsb_param = 0;
    sx126x_gfsk_crc_types_t  crc_type     = SX126X_GFSK_CRC_OFF;
    sx126x_gfsk_mod_shapes_t mod_shape    = SX126X_GFSK_MOD_SHAPE_OFF;

    status = ( ral_status_t ) sx126x_get_gfsk_bw_param( bw_dsb_in_hz, &bw_dsb_param );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

    switch( params->crc_type )
    {
    case RAL_GFSK_CRC_OFF:
        crc_type = SX126X_GFSK_CRC_OFF;
        break;
    case RAL_GFSK_CRC_1_BYTE:
        crc_type = SX126X_GFSK_CRC_1_BYTE;
        break;
    case RAL_GFSK_CRC_2_BYTES:
        crc_type = SX126X_GFSK_CRC_2_BYTES;
        break;
    case RAL_GFSK_CRC_1_BYTE_INV:
        crc_type = SX126X_GFSK_CRC_1_BYTE_INV;
        break;
    case RAL_GFSK_CRC_2_BYTES_INV:
        crc_type = SX126X_GFSK_CRC_2_BYTES_INV;
        break;
    default:
        return RAL_STATUS_UNKNOWN_VALUE;
    }

    switch( params->pulse_shape )
    {
    case RAL_GFSK_MOD_SHAPE_OFF:
        mod_shape = SX126X_GFSK_MOD_SHAPE_OFF;
        break;
    case RAL_GFSK_MOD_SHAPE_BT_03:
        mod_shape = SX126X_GFSK_MOD_SHAPE_BT_03;
        break;
    case RAL_GFSK_MOD_SHAPE_BT_05:
        mod_shape = SX126X_GFSK_MOD_SHAPE_BT_05;
        break;
    case RAL_GFSK_MOD_SHAPE_BT_07:
        mod_shape = SX126X_GFSK_MOD_SHAPE_BT_07;
        break;
    case RAL_GFSK_MOD_SHAPE_BT_1:
        mod_shape = SX126X_GFSK_MOD_SHAPE_BT_1;
        break;
    default:
        return RAL_STATUS_UNKNOWN_VALUE;
    }

    *mod_params = ( sx126x_mod_params_gfsk_t ){
        .br_in_bps    = params->br_in_bps,
        .bw_dsb_param = bw_dsb_param,
        .fdev_in_hz   = params->fdev_in_hz,
        .mod_shape    = mod_shape,
    };

    *pkt_params = ( sx126x_pkt_params_gfsk_t ){
        .pbl_len_in_bits       = params->pbl_len_in_bytes << 3,
        .pbl_min_det           = SX126X_GFSK_PBL_DET_08_BITS,
        .sync_word_len_in_bits = params->sync_word_len_in_bytes << 3,
        .addr_cmp              = SX126X_GFSK_ADDR_CMP_FILT_OFF,
        .hdr_type              = params->pld_is_fix ? SX126X_GFSK_PKT_FIX_LEN : SX126X_GFSK_PKT_VAR_LEN,
        .pld_len_in_bytes      = params->pld_len_in_bytes,
        .crc_type              = crc_type,
        .dc_free               = params->dc_free_is_on ? SX126X_GFSK_DC_FREE_WHITENING : SX126X_GFSK_DC_FREE_OFF
    };

    return RAL_STATUS_OK;
}

static ral_status_t ral_sx126x_convert_lora_params_from_radio( const ral_params_lora_t*  params,
                                                               sx126x_mod_params_lora_t* mod_params,
                                                               sx126x_pkt_params_lora_t* pkt_params )
{
    uint16_t pbl_len_in_symb = params->pbl_len_in_symb;

    // Ensure that the preamble length is at least 12 symbols when using SF5 or
    // SF6
    if( ( params->sf == RAL_LORA_SF5 ) || ( params->sf == RAL_LORA_SF6 ) )
    {
        if( pbl_len_in_symb < 12 )
        {
            pbl_len_in_symb = 12;
        }
    }

    *mod_params = ( sx126x_mod_params_lora_t ){ .sf   = params->sf,
                                                .bw   = ral_to_sx126x_lora_bw[params->bw],
                                                .cr   = ral_to_sx126x_lora_cr[params->cr],
                                                .ldro = ral_compute_lora_ldro( params->sf, params->bw ) };

    *pkt_params = ( sx126x_pkt_params_lora_t ){ .pbl_len_in_symb = params->pbl_len_in_symb,
                                                .hdr_type        = params->pld_is_fix ? SX126X_LORA_PKT_IMPLICIT
                                                                               : SX126X_LORA_PKT_EXPLICIT,
                                                .pld_len_in_bytes = params->pld_len_in_bytes,
                                                .crc_is_on        = params->crc_is_on,
                                                .invert_iq_is_on  = params->invert_iq_is_on };

    return RAL_STATUS_OK;
}

static ral_irq_t ral_sx126x_convert_irq_flags_to_radio( sx126x_irq_mask_t sx126x_irq )
{
    ral_irq_t ral_irq = RAL_IRQ_NONE;

    if( ( sx126x_irq & SX126X_IRQ_TX_DONE ) == SX126X_IRQ_TX_DONE )
    {
        ral_irq |= RAL_IRQ_TX_DONE;
    }
    if( ( sx126x_irq & SX126X_IRQ_RX_DONE ) == SX126X_IRQ_RX_DONE )
    {
        ral_irq |= RAL_IRQ_RX_DONE;
    }
    if( ( sx126x_irq & SX126X_IRQ_TIMEOUT ) == SX126X_IRQ_TIMEOUT )
    {
        ral_irq |= RAL_IRQ_RX_TIMEOUT;
    }
    if( ( ( sx126x_irq & SX126X_IRQ_SYNC_WORD_VALID ) == SX126X_IRQ_SYNC_WORD_VALID ) ||
        ( ( sx126x_irq & SX126X_IRQ_HEADER_VALID ) == SX126X_IRQ_HEADER_VALID ) )
    {
        ral_irq |= RAL_IRQ_RX_HDR_OK;
    }
    if( ( sx126x_irq & SX126X_IRQ_HEADER_ERROR ) == SX126X_IRQ_HEADER_ERROR )
    {
        ral_irq |= RAL_IRQ_RX_HDR_ERROR;
    }
    if( ( sx126x_irq & SX126X_IRQ_CRC_ERROR ) == SX126X_IRQ_CRC_ERROR )
    {
        ral_irq |= RAL_IRQ_RX_CRC_ERROR;
    }
    if( ( sx126x_irq & SX126X_IRQ_CAD_DONE ) == SX126X_IRQ_CAD_DONE )
    {
        ral_irq |= RAL_IRQ_CAD_DONE;
    }
    if( ( sx126x_irq & SX126X_IRQ_CAD_DET ) == SX126X_IRQ_CAD_DET )
    {
        ral_irq |= RAL_IRQ_CAD_OK;
    }
    return ral_irq;
}

static sx126x_irq_mask_t ral_sx126x_convert_irq_flags_from_radio( ral_irq_t ral_irq )
{
    sx126x_irq_mask_t sx126x_irq_mask = SX126X_IRQ_NONE;

    if( ( ral_irq & RAL_IRQ_TX_DONE ) == RAL_IRQ_TX_DONE )
    {
        sx126x_irq_mask |= SX126X_IRQ_TX_DONE;
    }
    if( ( ral_irq & RAL_IRQ_RX_DONE ) == RAL_IRQ_RX_DONE )
    {
        sx126x_irq_mask |= SX126X_IRQ_RX_DONE;
    }
    if( ( ral_irq & RAL_IRQ_RX_TIMEOUT ) == RAL_IRQ_RX_TIMEOUT )
    {
        sx126x_irq_mask |= SX126X_IRQ_TIMEOUT;
    }
    if( ( ral_irq & RAL_IRQ_RX_HDR_OK ) == RAL_IRQ_RX_HDR_OK )
    {
        sx126x_irq_mask |= SX126X_IRQ_SYNC_WORD_VALID;
        sx126x_irq_mask |= SX126X_IRQ_HEADER_VALID;
    }
    if( ( ral_irq & RAL_IRQ_RX_HDR_ERROR ) == RAL_IRQ_RX_HDR_ERROR )
    {
        sx126x_irq_mask |= SX126X_IRQ_HEADER_ERROR;
    }
    if( ( ral_irq & RAL_IRQ_RX_CRC_ERROR ) == RAL_IRQ_RX_CRC_ERROR )
    {
        sx126x_irq_mask |= SX126X_IRQ_CRC_ERROR;
    }
    if( ( ral_irq & RAL_IRQ_CAD_DONE ) == RAL_IRQ_CAD_DONE )
    {
        sx126x_irq_mask |= SX126X_IRQ_CAD_DONE;
    }
    if( ( ral_irq & RAL_IRQ_CAD_OK ) == RAL_IRQ_CAD_OK )
    {
        sx126x_irq_mask |= SX126X_IRQ_CAD_DET;
    }
    if( ( ral_irq & RAL_IRQ_ALL ) == RAL_IRQ_ALL )
    {
        sx126x_irq_mask |= SX126X_IRQ_ALL;
    }

    return sx126x_irq_mask;
}

static ral_status_t ral_sx126x_convert_tcxo_ctrl_to_radio( ral_tcxo_ctrl_voltages_t     ral_tcxo_ctrl_voltage,
                                                           sx126x_tcxo_ctrl_voltages_t* sx126x_tcxo_ctrl_voltage )
{
    ral_status_t status = RAL_STATUS_OK;

    switch( ral_tcxo_ctrl_voltage )
    {
    case RAL_TCXO_CTRL_1_6V:
        *sx126x_tcxo_ctrl_voltage = SX126X_TCXO_CTRL_1_6V;
        break;
    case RAL_TCXO_CTRL_1_7V:
        *sx126x_tcxo_ctrl_voltage = SX126X_TCXO_CTRL_1_7V;
        break;
    case RAL_TCXO_CTRL_1_8V:
        *sx126x_tcxo_ctrl_voltage = SX126X_TCXO_CTRL_1_8V;
        break;
    case RAL_TCXO_CTRL_2_2V:
        *sx126x_tcxo_ctrl_voltage = SX126X_TCXO_CTRL_2_2V;
        break;
    case RAL_TCXO_CTRL_2_4V:
        *sx126x_tcxo_ctrl_voltage = SX126X_TCXO_CTRL_2_4V;
        break;
    case RAL_TCXO_CTRL_2_7V:
        *sx126x_tcxo_ctrl_voltage = SX126X_TCXO_CTRL_2_7V;
        break;
    case RAL_TCXO_CTRL_3_0V:
        *sx126x_tcxo_ctrl_voltage = SX126X_TCXO_CTRL_3_0V;
        break;
    case RAL_TCXO_CTRL_3_3V:
        *sx126x_tcxo_ctrl_voltage = SX126X_TCXO_CTRL_3_3V;
        break;
    default:
        status = RAL_STATUS_UNKNOWN_VALUE;
        break;
    }

    return status;
}

/* --- EOF ------------------------------------------------------------------ */
