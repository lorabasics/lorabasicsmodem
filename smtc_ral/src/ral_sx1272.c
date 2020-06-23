/**
 * @file      ral_sx1272.c
 *
 * @brief     SX1272 radio abstraction layer implementation
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

#include "sx1272.h"
#include "ral_sx1272.h"

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

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

ral_status_t ral_sx1272_init( const ral_t* ral )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_sx1272_setup_gfsk( const ral_t* ral, const ral_params_gfsk_t* params )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_sx1272_setup_rx_gfsk( const ral_t* ral, const ral_params_gfsk_t* params )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_sx1272_setup_tx_gfsk( const ral_t* ral, const ral_params_gfsk_t* params )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_sx1272_setup_lora( const ral_t* ral, const ral_params_lora_t* params )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_sx1272_setup_rx_lora( const ral_t* ral, const ral_params_lora_t* params )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_sx1272_setup_tx_lora( const ral_t* ral, const ral_params_lora_t* params )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_sx1272_setup_flrc( const ral_t* ral, const ral_params_flrc_t* params )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_sx1272_setup_rx_flrc( const ral_t* ral, const ral_params_flrc_t* params )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_sx1272_setup_tx_flrc( const ral_t* ral, const ral_params_flrc_t* params )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_sx1272_setup_tx_lora_e( const ral_t* ral, const ral_params_lora_e_t* params )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_sx1272_tx_bpsk( const ral_t* ral, const ral_params_bpsk_t* params )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_sx1272_setup_cad( const ral_t* ral, const ral_lora_cad_params_t* params )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_sx1272_set_pkt_payload( const ral_t* ral, const uint8_t* buffer, const uint16_t size )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_sx1272_get_pkt_payload( const ral_t* ral, uint8_t* buffer, uint16_t max_size, uint16_t* size )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_sx1272_get_pkt_status_gfsk( const ral_t* ral, ral_rx_pkt_status_gfsk_t* pkt_status )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_sx1272_get_pkt_status_lora( const ral_t* ral, ral_rx_pkt_status_lora_t* pkt_status )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_sx1272_get_pkt_status_flrc( const ral_t* ral, ral_rx_pkt_status_flrc_t* pkt_status )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_sx1272_set_sleep( const ral_t* ral )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_sx1272_set_standby( const ral_t* ral )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_sx1272_set_tx( const ral_t* ral )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_sx1272_set_tx_cw( const ral_t* ral )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_sx1272_set_rx( const ral_t* ral, const uint32_t timeout_ms )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_sx1272_set_cad( const ral_t* ral )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_sx1272_get_irq_status( const ral_t* ral, ral_irq_t* ral_irq )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_sx1272_clear_irq_status( const ral_t* ral, const ral_irq_t ral_irq )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}
ral_status_t ral_sx1272_get_and_clear_irq_status( const ral_t* ral, ral_irq_t* ral_irq )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_sx1272_set_dio_irq_params( const ral_t* ral, const ral_irq_t ral_irq )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_sx1272_get_rssi( const ral_t* ral, int16_t* rssi )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_sx1272_get_lora_time_on_air_in_ms( const ral_params_lora_t* params, uint32_t* toa )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_sx1272_get_gfsk_time_on_air_in_ms( const ral_params_gfsk_t* params, uint32_t* toa )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
