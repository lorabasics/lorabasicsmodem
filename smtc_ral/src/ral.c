/**
 * @file      ral.c
 *
 * @brief     Radio Abstraction Layer (RAL) API implementation
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

#include "ral.h"
#include "ral_hal.h"

#if defined( SX126X )
#include "ral_sx126x.h"
#endif
#if defined( SX1272 )
#include "ral_sx1272.h"
#endif
#if defined( SX1276 )
#include "ral_sx1276.h"
#endif
#if defined( SX1280 )
#include "ral_sx1280.h"
#endif
#if !defined( SX126X ) && !defined( SX1272 ) && !defined( SX1276 ) && !defined( SX1280 )
#error "Unknown radio selected..."
#endif

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

ral_status_t ral_init( const ral_t* ral )
{
    switch( ral->radio_type )
    {
#if defined( SX126X )
    case RAL_RADIO_SX126X:
    {
        return ral_sx126x_init( ral );
    }
#endif
#if defined( SX1272 )
    case RAL_RADIO_SX1272:
    {
        return ral_sx1272_init( ral );
    }
#endif
#if defined( SX1276 )
    case RAL_RADIO_SX1276:
    {
        return ral_sx1276_init( ral );
    }
#endif
#if defined( SX1280 )
    case RAL_RADIO_SX1280:
    {
        return ral_sx1280_init( ral );
    }
#endif
    default:
        return RAL_STATUS_UNSUPPORTED_FEATURE;
    };
}

ral_status_t ral_setup_gfsk( const ral_t* ral, const ral_params_gfsk_t* params )
{
    switch( ral->radio_type )
    {
#if defined( SX126X )
    case RAL_RADIO_SX126X:
    {
        return ral_sx126x_setup_gfsk( ral, params );
    }
#endif
#if defined( SX1272 )
    case RAL_RADIO_SX1272:
    {
        return ral_sx1272_setup_gfsk( ral, params );
    }
#endif
#if defined( SX1276 )
    case RAL_RADIO_SX1276:
    {
        return ral_sx1276_setup_gfsk( ral, params );
    }
#endif
#if defined( SX1280 )
    case RAL_RADIO_SX1280:
    {
        return ral_sx1280_setup_gfsk( ral, params );
    }
#endif
    default:
        return RAL_STATUS_UNSUPPORTED_FEATURE;
    };
}

ral_status_t ral_setup_rx_gfsk( const ral_t* ral, const ral_params_gfsk_t* params )
{
    switch( ral->radio_type )
    {
#if defined( SX126X )
    case RAL_RADIO_SX126X:
    {
        return ral_sx126x_setup_rx_gfsk( ral, params );
    }
#endif
#if defined( SX1272 )
    case RAL_RADIO_SX1272:
    {
        return ral_sx1272_setup_rx_gfsk( ral, params );
    }
#endif
#if defined( SX1276 )
    case RAL_RADIO_SX1276:
    {
        return ral_sx1276_setup_rx_gfsk( ral, params );
    }
#endif
#if defined( SX1280 )
    case RAL_RADIO_SX1280:
    {
        return ral_sx1280_setup_rx_gfsk( ral, params );
    }
#endif
    default:
        return RAL_STATUS_UNSUPPORTED_FEATURE;
    };
}

ral_status_t ral_setup_tx_gfsk( const ral_t* ral, const ral_params_gfsk_t* params )
{
    switch( ral->radio_type )
    {
#if defined( SX126X )
    case RAL_RADIO_SX126X:
    {
        return ral_sx126x_setup_tx_gfsk( ral, params );
    }
#endif
#if defined( SX1272 )
    case RAL_RADIO_SX1272:
    {
        return ral_sx1272_setup_tx_gfsk( ral, params );
    }
#endif
#if defined( SX1276 )
    case RAL_RADIO_SX1276:
    {
        return ral_sx1276_setup_tx_gfsk( ral, params );
    }
#endif
#if defined( SX1280 )
    case RAL_RADIO_SX1280:
    {
        return ral_sx1280_setup_tx_gfsk( ral, params );
    }
#endif
    default:
        return RAL_STATUS_UNSUPPORTED_FEATURE;
    };
}

ral_status_t ral_setup_lora( const ral_t* ral, const ral_params_lora_t* params )
{
    switch( ral->radio_type )
    {
#if defined( SX126X )
    case RAL_RADIO_SX126X:
    {
        return ral_sx126x_setup_lora( ral, params );
    }
#endif
#if defined( SX1272 )
    case RAL_RADIO_SX1272:
    {
        return ral_sx1272_setup_lora( ral, params );
    }
#endif
#if defined( SX1276 )
    case RAL_RADIO_SX1276:
    {
        return ral_sx1276_setup_lora( ral, params );
    }
#endif
#if defined( SX1280 )
    case RAL_RADIO_SX1280:
    {
        return ral_sx1280_setup_lora( ral, params );
    }
#endif
    default:
        return RAL_STATUS_UNSUPPORTED_FEATURE;
    };
}

ral_status_t ral_setup_rx_lora( const ral_t* ral, const ral_params_lora_t* params )
{
    switch( ral->radio_type )
    {
#if defined( SX126X )
    case RAL_RADIO_SX126X:
    {
        return ral_sx126x_setup_rx_lora( ral, params );
    }
#endif
#if defined( SX1272 )
    case RAL_RADIO_SX1272:
    {
        return ral_sx1272_setup_rx_lora( ral, params );
    }
#endif
#if defined( SX1276 )
    case RAL_RADIO_SX1276:
    {
        return ral_sx1276_setup_rx_lora( ral, params );
    }
#endif
#if defined( SX1280 )
    case RAL_RADIO_SX1280:
    {
        return ral_sx1280_setup_rx_lora( ral, params );
    }
#endif
    default:
        return RAL_STATUS_UNSUPPORTED_FEATURE;
    };
}

ral_status_t ral_setup_tx_lora( const ral_t* ral, const ral_params_lora_t* params )
{
    switch( ral->radio_type )
    {
#if defined( SX126X )
    case RAL_RADIO_SX126X:
    {
        return ral_sx126x_setup_tx_lora( ral, params );
    }
#endif
#if defined( SX1272 )
    case RAL_RADIO_SX1272:
    {
        return ral_sx1272_setup_tx_lora( ral, params );
    }
#endif
#if defined( SX1276 )
    case RAL_RADIO_SX1276:
    {
        return ral_sx1276_setup_tx_lora( ral, params );
    }
#endif
#if defined( SX1280 )
    case RAL_RADIO_SX1280:
    {
        return ral_sx1280_setup_tx_lora( ral, params );
    }
#endif
    default:
        return RAL_STATUS_UNSUPPORTED_FEATURE;
    };
}

ral_status_t ral_setup_flrc( const ral_t* ral, const ral_params_flrc_t* params )
{
    switch( ral->radio_type )
    {
#if defined( SX126X )
    case RAL_RADIO_SX126X:
    {
        return ral_sx126x_setup_flrc( ral, params );
    }
#endif
#if defined( SX1272 )
    case RAL_RADIO_SX1272:
    {
        return ral_sx1272_setup_flrc( ral, params );
    }
#endif
#if defined( SX1276 )
    case RAL_RADIO_SX1276:
    {
        return ral_sx1276_setup_flrc( ral, params );
    }
#endif
#if defined( SX1280 )
    case RAL_RADIO_SX1280:
    {
        return ral_sx1280_setup_flrc( ral, params );
    }
#endif
    default:
        return RAL_STATUS_UNSUPPORTED_FEATURE;
    };
}

ral_status_t ral_setup_rx_flrc( const ral_t* ral, const ral_params_flrc_t* params )
{
    switch( ral->radio_type )
    {
#if defined( SX126X )
    case RAL_RADIO_SX126X:
    {
        return ral_sx126x_setup_rx_flrc( ral, params );
    }
#endif
#if defined( SX1272 )
    case RAL_RADIO_SX1272:
    {
        return ral_sx1272_setup_rx_flrc( ral, params );
    }
#endif
#if defined( SX1276 )
    case RAL_RADIO_SX1276:
    {
        return ral_sx1276_setup_rx_flrc( ral, params );
    }
#endif
#if defined( SX1280 )
    case RAL_RADIO_SX1280:
    {
        return ral_sx1280_setup_rx_flrc( ral, params );
    }
#endif
    default:
        return RAL_STATUS_UNSUPPORTED_FEATURE;
    };
}

ral_status_t ral_setup_tx_flrc( const ral_t* ral, const ral_params_flrc_t* params )
{
    switch( ral->radio_type )
    {
#if defined( SX126X )
    case RAL_RADIO_SX126X:
    {
        return ral_sx126x_setup_tx_flrc( ral, params );
    }
#endif
#if defined( SX1272 )
    case RAL_RADIO_SX1272:
    {
        return ral_sx1272_setup_tx_flrc( ral, params );
    }
#endif
#if defined( SX1276 )
    case RAL_RADIO_SX1276:
    {
        return ral_sx1276_setup_tx_flrc( ral, params );
    }
#endif
#if defined( SX1280 )
    case RAL_RADIO_SX1280:
    {
        return ral_sx1280_setup_tx_flrc( ral, params );
    }
#endif
    default:
        return RAL_STATUS_UNSUPPORTED_FEATURE;
    };
}

ral_status_t ral_setup_tx_lora_e( const ral_t* ral, const ral_params_lora_e_t* params )
{
    switch( ral->radio_type )
    {
#if defined( SX126X )
    case RAL_RADIO_SX126X:
    {
        return ral_sx126x_setup_tx_lora_e( ral, params );
    }
#endif
#if defined( SX1272 )
    case RAL_RADIO_SX1272:
    {
        return ral_sx1272_setup_tx_lora_e( ral, params );
    }
#endif
#if defined( SX1276 )
    case RAL_RADIO_SX1276:
    {
        return ral_sx1276_setup_tx_lora_e( ral, params );
    }
#endif
#if defined( SX1280 )
    case RAL_RADIO_SX1280:
    {
        return ral_sx1280_setup_tx_lora_e( ral, params );
    }
#endif
    default:
        return RAL_STATUS_UNSUPPORTED_FEATURE;
    };
}

ral_status_t ral_tx_bpsk( const ral_t* ral, const ral_params_bpsk_t* params )
{
    switch( ral->radio_type )
    {
#if defined( SX126X )
    case RAL_RADIO_SX126X:
    {
        return ral_sx126x_tx_bpsk( ral, params );
    }
#endif
#if defined( SX1272 )
    case RAL_RADIO_SX1272:
    {
        return ral_sx1272_tx_bpsk( ral, params );
    }
#endif
#if defined( SX1276 )
    case RAL_RADIO_SX1276:
    {
        return ral_sx1276_tx_bpsk( ral, params );
    }
#endif
#if defined( SX1280 )
    case RAL_RADIO_SX1280:
    {
        return ral_sx1280_tx_bpsk( ral, params );
    }
#endif
    default:
        return RAL_STATUS_UNSUPPORTED_FEATURE;
    };
}

ral_status_t ral_setup_cad( const ral_t* ral, const ral_lora_cad_params_t* params )
{
    switch( ral->radio_type )
    {
#if defined( SX126X )
    case RAL_RADIO_SX126X:
    {
        return ral_sx126x_setup_cad( ral, params );
    }
#endif
#if defined( SX1272 )
    case RAL_RADIO_SX1272:
    {
        return ral_sx1272_setup_cad( ral, params );
    }
#endif
#if defined( SX1276 )
    case RAL_RADIO_SX1276:
    {
        return ral_sx1276_setup_cad( ral, params );
    }
#endif
#if defined( SX1280 )
    case RAL_RADIO_SX1280:
    {
        return ral_sx1280_setup_cad( ral, params );
    }
#endif
    default:
        return RAL_STATUS_UNSUPPORTED_FEATURE;
    };
}

ral_status_t ral_set_pkt_payload( const ral_t* ral, const uint8_t* buffer, const uint16_t size )
{
    switch( ral->radio_type )
    {
#if defined( SX126X )
    case RAL_RADIO_SX126X:
    {
        return ral_sx126x_set_pkt_payload( ral, buffer, size );
    }
#endif
#if defined( SX1272 )
    case RAL_RADIO_SX1272:
    {
        return ral_sx1272_set_pkt_payload( ral, buffer, size );
    }
#endif
#if defined( SX1276 )
    case RAL_RADIO_SX1276:
    {
        return ral_sx1276_set_pkt_payload( ral, buffer, size );
    }
#endif
#if defined( SX1280 )
    case RAL_RADIO_SX1280:
    {
        return ral_sx1280_set_pkt_payload( ral, buffer, size );
    }
#endif
    default:
        return RAL_STATUS_UNSUPPORTED_FEATURE;
    };
}

ral_status_t ral_get_pkt_payload( const ral_t* ral, uint8_t* buffer, uint16_t max_size, uint16_t* size )
{
    switch( ral->radio_type )
    {
#if defined( SX126X )
    case RAL_RADIO_SX126X:
    {
        return ral_sx126x_get_pkt_payload( ral, buffer, max_size, size );
    }
#endif
#if defined( SX1272 )
    case RAL_RADIO_SX1272:
    {
        return ral_sx1272_get_pkt_payload( ral, buffer, max_size, size );
    }
#endif
#if defined( SX1276 )
    case RAL_RADIO_SX1276:
    {
        return ral_sx1276_get_pkt_payload( ral, buffer, max_size, size );
    }
#endif
#if defined( SX1280 )
    case RAL_RADIO_SX1280:
    {
        return ral_sx1280_get_pkt_payload( ral, buffer, max_size, size );
    }
#endif
    default:
        return RAL_STATUS_UNSUPPORTED_FEATURE;
    };
}

ral_status_t ral_get_gfsk_pkt_status( const ral_t* ral, ral_rx_pkt_status_gfsk_t* pkt_status )
{
    switch( ral->radio_type )
    {
#if defined( SX126X )
    case RAL_RADIO_SX126X:
    {
        return ral_sx126x_get_gfsk_pkt_status( ral, pkt_status );
    }
#endif
#if defined( SX1272 )
    case RAL_RADIO_SX1272:
    {
        return ral_sx1272_get_pkt_status_gfsk( ral, pkt_status );
    }
#endif
#if defined( SX1276 )
    case RAL_RADIO_SX1276:
    {
        return ral_sx1276_get_pkt_status_gfsk( ral, pkt_status );
    }
#endif
#if defined( SX1280 )
    case RAL_RADIO_SX1280:
    {
        return ral_sx1280_get_gfsk_pkt_status( ral, pkt_status );
    }
#endif
    default:
        return RAL_STATUS_UNSUPPORTED_FEATURE;
    };
}

ral_status_t ral_get_lora_pkt_status( const ral_t* ral, ral_rx_pkt_status_lora_t* pkt_status )
{
    switch( ral->radio_type )
    {
#if defined( SX126X )
    case RAL_RADIO_SX126X:
    {
        return ral_sx126x_get_lora_pkt_status( ral, pkt_status );
    }
#endif
#if defined( SX1272 )
    case RAL_RADIO_SX1272:
    {
        return ral_sx1272_get_pkt_status_lora( ral, pkt_status );
    }
#endif
#if defined( SX1276 )
    case RAL_RADIO_SX1276:
    {
        return ral_sx1276_get_pkt_status_lora( ral, pkt_status );
    }
#endif
#if defined( SX1280 )
    case RAL_RADIO_SX1280:
    {
        return ral_sx1280_get_lora_pkt_status( ral, pkt_status );
    }
#endif
    default:
        return RAL_STATUS_UNSUPPORTED_FEATURE;
    };
}

ral_status_t ral_get_lora_incoming_pkt_config( const ral_t* ral, ral_lora_cr_t* rx_cr, bool* rx_is_crc_en )
{
    switch( ral->radio_type )
    {
#if defined( SX126X )
    case RAL_RADIO_SX126X:
    {
        return RAL_STATUS_UNSUPPORTED_FEATURE;
    }
#endif
#if defined( SX1272 )
    case RAL_RADIO_SX1272:
    {
        return RAL_STATUS_UNSUPPORTED_FEATURE;
    }
#endif
#if defined( SX1276 )
    case RAL_RADIO_SX1276:
    {
        return RAL_STATUS_UNSUPPORTED_FEATURE;
    }
#endif
#if defined( SX1280 )
    case RAL_RADIO_SX1280:
    {
        return ral_sx1280_get_lora_incoming_pkt_config( ral, rx_cr, rx_is_crc_en );
    }
#endif
    default:
        return RAL_STATUS_UNSUPPORTED_FEATURE;
    };
}

ral_status_t ral_get_flrc_pkt_status( const ral_t* ral, ral_rx_pkt_status_flrc_t* pkt_status )
{
    switch( ral->radio_type )
    {
#if defined( SX126X )
    case RAL_RADIO_SX126X:
    {
        return ral_sx126x_get_flrc_pkt_status( ral, pkt_status );
    }
#endif
#if defined( SX1272 )
    case RAL_RADIO_SX1272:
    {
        return ral_sx1272_get_pkt_status_flrc( ral, pkt_status );
    }
#endif
#if defined( SX1276 )
    case RAL_RADIO_SX1276:
    {
        return ral_sx1276_get_pkt_status_flrc( ral, pkt_status );
    }
#endif
#if defined( SX1280 )
    case RAL_RADIO_SX1280:
    {
        return ral_sx1280_get_flrc_pkt_status( ral, pkt_status );
    }
#endif
    default:
        return RAL_STATUS_UNSUPPORTED_FEATURE;
    };
}

ral_status_t ral_set_sleep( const ral_t* ral )
{
    switch( ral->radio_type )
    {
#if defined( SX126X )
    case RAL_RADIO_SX126X:
    {
        return ral_sx126x_set_sleep( ral );
    }
#endif
#if defined( SX1272 )
    case RAL_RADIO_SX1272:
    {
        return ral_sx1272_set_sleep( ral );
    }
#endif
#if defined( SX1276 )
    case RAL_RADIO_SX1276:
    {
        return ral_sx1276_set_sleep( ral );
    }
#endif
#if defined( SX1280 )
    case RAL_RADIO_SX1280:
    {
        return ral_sx1280_set_sleep( ral );
    }
#endif
    default:
        return RAL_STATUS_UNSUPPORTED_FEATURE;
    };
}

ral_status_t ral_set_standby( const ral_t* ral )
{
    switch( ral->radio_type )
    {
#if defined( SX126X )
    case RAL_RADIO_SX126X:
    {
        return ral_sx126x_set_standby( ral );
    }
#endif
#if defined( SX1272 )
    case RAL_RADIO_SX1272:
    {
        return ral_sx1272_set_standby( ral );
    }
#endif
#if defined( SX1276 )
    case RAL_RADIO_SX1276:
    {
        return ral_sx1276_set_standby( ral );
    }
#endif
#if defined( SX1280 )
    case RAL_RADIO_SX1280:
    {
        return ral_sx1280_set_standby( ral );
    }
#endif
    default:
        return RAL_STATUS_UNSUPPORTED_FEATURE;
    };
}

ral_status_t ral_set_tx( const ral_t* ral )
{
    switch( ral->radio_type )
    {
#if defined( SX126X )
    case RAL_RADIO_SX126X:
    {
        return ral_sx126x_set_tx( ral );
    }
#endif
#if defined( SX1272 )
    case RAL_RADIO_SX1272:
    {
        return ral_sx1272_set_tx( ral );
    }
#endif
#if defined( SX1276 )
    case RAL_RADIO_SX1276:
    {
        return ral_sx1276_set_tx( ral );
    }
#endif
#if defined( SX1280 )
    case RAL_RADIO_SX1280:
    {
        return ral_sx1280_set_tx( ral );
    }
#endif
    default:
        return RAL_STATUS_UNSUPPORTED_FEATURE;
    };
}

ral_status_t ral_set_tx_cw( const ral_t* ral )
{
    switch( ral->radio_type )
    {
#if defined( SX126X )
    case RAL_RADIO_SX126X:
    {
        return ral_sx126x_set_tx_cw( ral );
    }
#endif
#if defined( SX1272 )
    case RAL_RADIO_SX1272:
    {
        return ral_sx1272_set_tx_cw( ral );
    }
#endif
#if defined( SX1276 )
    case RAL_RADIO_SX1276:
    {
        return ral_sx1276_set_tx_cw( ral );
    }
#endif
#if defined( SX1280 )
    case RAL_RADIO_SX1280:
    {
        return ral_sx1280_set_tx_cw( ral );
    }
#endif
    default:
        return RAL_STATUS_UNSUPPORTED_FEATURE;
    };
}

ral_status_t ral_set_rx( const ral_t* ral, const uint32_t timeout_ms )
{
    switch( ral->radio_type )
    {
#if defined( SX126X )
    case RAL_RADIO_SX126X:
    {
        return ral_sx126x_set_rx( ral, timeout_ms );
    }
#endif
#if defined( SX1272 )
    case RAL_RADIO_SX1272:
    {
        return ral_sx1272_set_rx( ral, timeout_ms );
    }
#endif
#if defined( SX1276 )
    case RAL_RADIO_SX1276:
    {
        return ral_sx1276_set_rx( ral, timeout_ms );
    }
#endif
#if defined( SX1280 )
    case RAL_RADIO_SX1280:
    {
        return ral_sx1280_set_rx( ral, timeout_ms );
    }
#endif
    default:
        return RAL_STATUS_UNSUPPORTED_FEATURE;
    };
}

ral_status_t ral_set_cad( const ral_t* ral )
{
    switch( ral->radio_type )
    {
#if defined( SX126X )
    case RAL_RADIO_SX126X:
    {
        return ral_sx126x_set_cad( ral );
    }
#endif
#if defined( SX1272 )
    case RAL_RADIO_SX1272:
    {
        return ral_sx1272_set_cad( ral );
    }
#endif
#if defined( SX1276 )
    case RAL_RADIO_SX1276:
    {
        return ral_sx1276_set_cad( ral );
    }
#endif
#if defined( SX1280 )
    case RAL_RADIO_SX1280:
    {
        return ral_sx1280_set_cad( ral );
    }
#endif
    default:
        return RAL_STATUS_UNSUPPORTED_FEATURE;
    };
}

ral_status_t ral_get_irq_status( const ral_t* ral, ral_irq_t* irq_status )
{
    switch( ral->radio_type )
    {
#if defined( SX126X )
    case RAL_RADIO_SX126X:
    {
        return ral_sx126x_get_irq_status( ral, irq_status );
    }
#endif
#if defined( SX1272 )
    case RAL_RADIO_SX1272:
    {
        return ral_sx1272_get_irq_status( ral, irq_status );
    }
#endif
#if defined( SX1276 )
    case RAL_RADIO_SX1276:
    {
        return ral_sx1276_get_irq_status( ral, irq_status );
    }
#endif
#if defined( SX1280 )
    case RAL_RADIO_SX1280:
    {
        return ral_sx1280_get_irq_status( ral, irq_status );
    }
#endif
    default:
        return RAL_STATUS_UNSUPPORTED_FEATURE;
    };
}

ral_status_t ral_clear_irq_status( const ral_t* ral, const ral_irq_t irq_status )
{
    switch( ral->radio_type )
    {
#if defined( SX126X )
    case RAL_RADIO_SX126X:
    {
        return ral_sx126x_clear_irq_status( ral, irq_status );
    }
#endif
#if defined( SX1272 )
    case RAL_RADIO_SX1272:
    {
        return ral_sx1272_clear_irq_status( ral, irq_status );
    }
#endif
#if defined( SX1276 )
    case RAL_RADIO_SX1276:
    {
        return ral_sx1276_clear_irq_status( ral, irq_status );
    }
#endif
#if defined( SX1280 )
    case RAL_RADIO_SX1280:
    {
        return ral_sx1280_clear_irq_status( ral, irq_status );
    }
#endif
    default:
        return RAL_STATUS_UNSUPPORTED_FEATURE;
    };
}

ral_status_t ral_get_and_clear_irq_status( const ral_t* ral, ral_irq_t* irq_status )
{
    switch( ral->radio_type )
    {
#if defined( SX126X )
    case RAL_RADIO_SX126X:
    {
        return ral_sx126x_get_and_clear_irq_status( ral, irq_status );
    }
#endif
#if defined( SX1280 )
    case RAL_RADIO_SX1280:
    {
        return ral_sx1280_get_and_clear_irq_status( ral, irq_status );
    }
#endif
    default:
        return RAL_STATUS_UNSUPPORTED_FEATURE;
    };
}

ral_status_t ral_set_dio_irq_params( const ral_t* ral, const ral_irq_t ral_irq )
{
    switch( ral->radio_type )
    {
#if defined( SX126X )
    case RAL_RADIO_SX126X:
    {
        return ral_sx126x_set_dio_irq_params( ral, ral_irq );
    }
#endif
#if defined( SX1280 )
    case RAL_RADIO_SX1280:
    {
        return ral_sx1280_set_dio_irq_params( ral, ral_irq );
    }
#endif
    default:
        return RAL_STATUS_UNSUPPORTED_FEATURE;
    };
}

ral_status_t ral_process_irq( const ral_t* ral, ral_irq_t* ral_irq )
{
    switch( ral->radio_type )
    {
#if defined( SX126X )
    case RAL_RADIO_SX126X:
    {
        return RAL_STATUS_UNSUPPORTED_FEATURE;
    }
#endif
#if defined( SX1272 )
    case RAL_RADIO_SX1272:
    {
        return RAL_STATUS_UNSUPPORTED_FEATURE;
    }
#endif
#if defined( SX1276 )
    case RAL_RADIO_SX1276:
    {
        return RAL_STATUS_UNSUPPORTED_FEATURE;
    }
#endif
#if defined( SX1280 )
    case RAL_RADIO_SX1280:
    {
        return ral_sx1280_process_irq( ral, ral_irq );
    }
#endif
    default:
        return RAL_STATUS_UNSUPPORTED_FEATURE;
    };
}

ral_status_t ral_get_rssi( const ral_t* ral, int16_t* rssi )
{
    switch( ral->radio_type )
    {
#if defined( SX126X )
    case RAL_RADIO_SX126X:
    {
        return ral_sx126x_get_rssi( ral, rssi );
    }
#endif
#if defined( SX1272 )
    case RAL_RADIO_SX1272:
    {
        return ral_sx1272_get_rssi( ral, rssi );
    }
#endif
#if defined( SX1276 )
    case RAL_RADIO_SX1276:
    {
        return ral_sx1276_get_rssi( ral, rssi );
    }
#endif
#if defined( SX1280 )
    case RAL_RADIO_SX1280:
    {
        return ral_sx1280_get_rssi( ral, rssi );
    }
#endif
    default:
        return RAL_STATUS_UNSUPPORTED_FEATURE;
    };
}

ral_status_t ral_get_lora_time_on_air_in_ms( const ral_t* ral, const ral_params_lora_t* params, uint32_t* toa )
{
    switch( ral->radio_type )
    {
#if defined( SX126X )
    case RAL_RADIO_SX126X:
    {
        return ral_sx126x_get_lora_time_on_air_in_ms( params, toa );
    }
#endif
#if defined( SX1272 )
    case RAL_RADIO_SX1272:
    {
        return ral_sx1272_get_lora_time_on_air_in_ms( params, toa );
    }
#endif
#if defined( SX1276 )
    case RAL_RADIO_SX1276:
    {
        return ral_sx1276_get_lora_time_on_air_in_ms( params, toa );
    }
#endif
#if defined( SX1280 )
    case RAL_RADIO_SX1280:
    {
        return ral_sx1280_get_lora_time_on_air_in_ms( params, toa );
    }
#endif
    default:
        return RAL_STATUS_UNSUPPORTED_FEATURE;
    };
}

ral_status_t ral_get_gfsk_time_on_air_in_ms( const ral_t* ral, const ral_params_gfsk_t* params, uint32_t* toa )
{
    switch( ral->radio_type )
    {
#if defined( SX126X )
    case RAL_RADIO_SX126X:
    {
        return ral_sx126x_get_gfsk_time_on_air_in_ms( params, toa );
    }
#endif
#if defined( SX1272 )
    case RAL_RADIO_SX1272:
    {
        return ral_sx1272_get_gfsk_time_on_air_in_ms( params, toa );
    }
#endif
#if defined( SX1276 )
    case RAL_RADIO_SX1276:
    {
        return ral_sx1276_get_gfsk_time_on_air_in_ms( params, toa );
    }
#endif
#if defined( SX1280 )
    case RAL_RADIO_SX1280:
    {
        return ral_sx1280_get_gfsk_time_on_air_in_ms( params, toa );
    }
#endif
    default:
        return RAL_STATUS_UNSUPPORTED_FEATURE;
    };
}

ral_status_t ral_get_flrc_time_on_air_in_ms( const ral_t* ral, const ral_params_flrc_t* params, uint32_t* toa )
{
    switch( ral->radio_type )
    {
#if defined( SX1280 )
    case RAL_RADIO_SX1280:
    {
        return ral_sx1280_get_flrc_time_on_air_in_ms( params, toa );
    }
#endif
    default:
        return RAL_STATUS_UNSUPPORTED_FEATURE;
    };
}

ral_status_t ral_get_lora_tx_consumption_in_ua( const ral_t* ral, const ral_params_lora_t* params,
                                                uint32_t* micro_ampere )
{
    switch( ral->radio_type )
    {
#if defined( SX126X )
    case RAL_RADIO_SX126X:
    {
        return RAL_STATUS_UNSUPPORTED_FEATURE;
    }
#endif
#if defined( SX1272 )
    case RAL_RADIO_SX1272:
    {
        return RAL_STATUS_UNSUPPORTED_FEATURE;
    }
#endif
#if defined( SX1276 )
    case RAL_RADIO_SX1276:
    {
        return RAL_STATUS_UNSUPPORTED_FEATURE;
    }
#endif
#if defined( SX1280 )
    case RAL_RADIO_SX1280:
    {
        return ral_sx1280_convert_lora_tx_dbm_to_ua( params, micro_ampere );
    }
#endif
    default:
        return RAL_STATUS_UNSUPPORTED_FEATURE;
    };
}

ral_status_t ral_get_lora_rx_consumption_in_ua( const ral_t* ral, const ral_params_lora_t* params,
                                                uint32_t* micro_ampere )
{
    switch( ral->radio_type )
    {
#if defined( SX126X )
    case RAL_RADIO_SX126X:
    {
        return RAL_STATUS_UNSUPPORTED_FEATURE;
    }
#endif
#if defined( SX1272 )
    case RAL_RADIO_SX1272:
    {
        return RAL_STATUS_UNSUPPORTED_FEATURE;
    }
#endif
#if defined( SX1276 )
    case RAL_RADIO_SX1276:
    {
        return RAL_STATUS_UNSUPPORTED_FEATURE;
    }
#endif
#if defined( SX1280 )
    case RAL_RADIO_SX1280:
    {
        return ral_sx1280_convert_lora_rx_bw_to_ua( params, micro_ampere );
    }
#endif
    default:
        return RAL_STATUS_UNSUPPORTED_FEATURE;
    };
}

ral_status_t ral_set_tcxo_on( const ral_t* ral )
{
    ral_status_t status = RAL_STATUS_UNSUPPORTED_FEATURE;

    if( ral->tcxo_cfg.tcxo_ctrl_mode == RAL_TCXO_CTRL_HOST_EXT )
    {
        status = ( ral_status_t ) ral_hal_set_tcxo_on( ral->context, ral->tcxo_cfg.tcxo_startup_time_ms );
    }

    return status;
}

ral_status_t ral_set_tcxo_off( const ral_t* ral )
{
    ral_status_t status = RAL_STATUS_UNSUPPORTED_FEATURE;

    if( ral->tcxo_cfg.tcxo_ctrl_mode == RAL_TCXO_CTRL_HOST_EXT )
    {
        status = ( ral_status_t ) ral_hal_set_tcxo_off( ral->context );
    }

    return status;
}

ral_status_t ral_read_register( const ral_t* ral, uint16_t address, uint8_t* buffer, uint16_t size )
{
    switch( ral->radio_type )
    {
#if defined( SX126X )
    case RAL_RADIO_SX126X:
    {
        return RAL_STATUS_UNSUPPORTED_FEATURE;
    }
#endif
#if defined( SX1272 )
    case RAL_RADIO_SX1272:
    {
        return RAL_STATUS_UNSUPPORTED_FEATURE;
    }
#endif
#if defined( SX1276 )
    case RAL_RADIO_SX1276:
    {
        return RAL_STATUS_UNSUPPORTED_FEATURE;
    }
#endif
#if defined( SX1280 )
    case RAL_RADIO_SX1280:
    {
        return ral_sx1280_read_register( ral, address, buffer, size );
    }
#endif
    default:
        return RAL_STATUS_UNSUPPORTED_FEATURE;
    };
}

ral_status_t ral_write_register( const ral_t* ral, uint16_t address, uint8_t* buffer, uint16_t size )
{
    switch( ral->radio_type )
    {
#if defined( SX126X )
    case RAL_RADIO_SX126X:
    {
        return RAL_STATUS_UNSUPPORTED_FEATURE;
    }
#endif
#if defined( SX1272 )
    case RAL_RADIO_SX1272:
    {
        return RAL_STATUS_UNSUPPORTED_FEATURE;
    }
#endif
#if defined( SX1276 )
    case RAL_RADIO_SX1276:
    {
        return RAL_STATUS_UNSUPPORTED_FEATURE;
    }
#endif
#if defined( SX1280 )
    case RAL_RADIO_SX1280:
    {
        return ral_sx1280_write_register( ral, address, buffer, size );
    }
#endif
    default:
        return RAL_STATUS_UNSUPPORTED_FEATURE;
    };
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
