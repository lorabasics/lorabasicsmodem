/*!
 * \file      smtc_crypto.c
 *
 * \brief     Implements crypto funtions.
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
#include <stdint.h>
#include "smtc_crypto.h"
#include "smtc_modem_hal_dbg_trace.h"
#include "lr1mac_utilities.h"
#include "lr1mac_core.h"

#if defined( LR1110_MODEM ) || defined( USE_LR1110_SE )
#include "lr1110_crypto_engine.h"
#include "lr1110_types.h"
#include "lr1110_system.h"
#include "lorawan_api.h"
#define MAX_CRYPTO_PAYLOAD_SIZE ( 300 )  // LoRaWAN payload is max 255 but GFSK allow for slightly larger payload
#else
#include "lr1mac_defs.h"
#include "aes.h"
#include "cmac.h"
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define LORAMAC_MIC_BLOCK_B0_SIZE 16

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 *-----------------------------------------------------------------------------------
 *--- PRIVATE VARIABLES -------------------------------------------------------------
 */

#if defined( USE_LR1110_SE )
static bool process_join_accept_is_success;
#else
static aes_context  context_aes;
static AES_CMAC_CTX context_aes_cmac[1];
#endif

/*
 *-----------------------------------------------------------------------------------
 *--- PRIVATE FUNCTIONS DECLARATION -------------------------------------------------
 */
#if defined( USE_LR1110_SE )
static uint8_t         get_key_index( const uint8_t* key );
static lr1110_status_t lr1110_to_sleep( void );
#endif

/*
 *-----------------------------------------------------------------------------------
 *--- PUBLIC FUNCTIONS DEFINITIONS --------------------------------------------------
 */

void compute_mic( const uint8_t* buffer, uint16_t size, const uint8_t* key, uint32_t address, uint8_t dir,
                  uint32_t sequence_counter, uint32_t* mic )
{
#if defined( USE_LR1110_SE )
    uint8_t mic_bloc_b0[LORAMAC_MIC_BLOCK_B0_SIZE + MAX_CRYPTO_PAYLOAD_SIZE] = { 0 };
#else
    uint8_t mic_bloc_b0[LORAMAC_MIC_BLOCK_B0_SIZE] = { 0 };
#endif

    mic_bloc_b0[0] = 0x49;
    mic_bloc_b0[5] = dir;

    mic_bloc_b0[6] = ( address ) &0xFF;
    mic_bloc_b0[7] = ( address >> 8 ) & 0xFF;
    mic_bloc_b0[8] = ( address >> 16 ) & 0xFF;
    mic_bloc_b0[9] = ( address >> 24 ) & 0xFF;

    mic_bloc_b0[10] = ( sequence_counter ) &0xFF;
    mic_bloc_b0[11] = ( sequence_counter >> 8 ) & 0xFF;
    mic_bloc_b0[12] = ( sequence_counter >> 16 ) & 0xFF;
    mic_bloc_b0[13] = ( sequence_counter >> 24 ) & 0xFF;

    mic_bloc_b0[15] = size & 0xFF;

#if defined( USE_LR1110_SE )
    if( size > MAX_CRYPTO_PAYLOAD_SIZE )
    {
        smtc_modem_hal_mcu_panic( "Payload too big \n" );
    }
    memcpy( mic_bloc_b0 + LORAMAC_MIC_BLOCK_B0_SIZE, buffer, size );

    lr1110_crypto_mic_t    mic_value;
    lr1110_crypto_status_t status = LR1110_CRYPTO_STATUS_ERROR;

    lr1110_crypto_compute_aes_cmac( NULL, &status, get_key_index( key ), mic_bloc_b0, LORAMAC_MIC_BLOCK_B0_SIZE + size,
                                    mic_value );
    if( status != LR1110_CRYPTO_STATUS_SUCCESS )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Failed to compute aes cmac (%d)\n", status );
    }

    status = lr1110_to_sleep( );
    if( status != LR1110_CRYPTO_STATUS_SUCCESS )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Failed to set sleep mode (%d)\n", status );
    }

#else
    uint8_t mic_value[16];

    AES_CMAC_Init( context_aes_cmac );
    AES_CMAC_SetKey( context_aes_cmac, key );
    AES_CMAC_Update( context_aes_cmac, mic_bloc_b0, LORAMAC_MIC_BLOCK_B0_SIZE );
    AES_CMAC_Update( context_aes_cmac, buffer, size & 0xFF );
    AES_CMAC_Final( mic_value, context_aes_cmac );
#endif

    *mic = ( uint32_t )( ( ( ( uint32_t ) mic_value[3] ) << 24 ) | ( ( ( uint32_t ) mic_value[2] ) << 16 ) |
                         ( ( ( uint32_t ) mic_value[1] ) << 8 ) | ( ( uint32_t ) mic_value[0] ) );
}

void lora_crypto_payload_encrypt( const uint8_t* buffer, uint16_t size, const uint8_t* key, uint32_t address,
                                  uint8_t dir, uint32_t sequence_counter, uint8_t* encBuffer )
{
    uint16_t i;
    uint8_t  buffer_idx  = 0;
    uint16_t ctr         = 1;
    uint8_t  s_block[16] = { 0 };
    uint8_t  a_block[16] = { 0 };

    a_block[0] = 0x01;
    a_block[5] = dir;

    a_block[6] = ( address ) &0xFF;
    a_block[7] = ( address >> 8 ) & 0xFF;
    a_block[8] = ( address >> 16 ) & 0xFF;
    a_block[9] = ( address >> 24 ) & 0xFF;

    a_block[10] = ( sequence_counter ) &0xFF;
    a_block[11] = ( sequence_counter >> 8 ) & 0xFF;
    a_block[12] = ( sequence_counter >> 16 ) & 0xFF;
    a_block[13] = ( sequence_counter >> 24 ) & 0xFF;

#if defined( USE_LR1110_SE )
    uint8_t                key_idx = get_key_index( key );
    lr1110_crypto_status_t status  = LR1110_CRYPTO_STATUS_ERROR;
#else
    memset1( context_aes.ksch, '\0', 240 );
    aes_set_key( key, 16, &context_aes );
#endif

    while( size >= 16 )
    {
        a_block[15] = ( ( ctr ) &0xFF );
        ctr++;

#if defined( USE_LR1110_SE )
        lr1110_crypto_aes_encrypt_01( NULL, &status, key_idx, a_block, 16, s_block );
        if( status != LR1110_CRYPTO_STATUS_SUCCESS )
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "Failed to encrypt (%d)\n", status );
        }
#else
        aes_encrypt( a_block, s_block, &context_aes );
#endif
        for( i = 0; i < 16; i++ )
        {
            encBuffer[buffer_idx + i] = buffer[buffer_idx + i] ^ s_block[i];
        }

        size -= 16;
        buffer_idx += 16;
    }

    if( size > 0 )
    {
        a_block[15] = ( ( ctr ) &0xFF );

#if defined( USE_LR1110_SE )
        lr1110_crypto_aes_encrypt_01( NULL, &status, key_idx, a_block, 16, s_block );
        if( status != LR1110_CRYPTO_STATUS_SUCCESS )
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "Failed to encrypt last part (%d)\n", status );
        }
#else
        aes_encrypt( a_block, s_block, &context_aes );
#endif

        for( i = 0; i < size; i++ )
        {
            encBuffer[buffer_idx + i] = buffer[buffer_idx + i] ^ s_block[i];
        }
    }
#if defined( USE_LR1110_SE )
    status = lr1110_to_sleep( );
    if( status != LR1110_CRYPTO_STATUS_SUCCESS )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Failed to set sleep mode (%d)\n", status );
    }
#endif
}

void payload_decrypt( const uint8_t* buffer, uint16_t size, const uint8_t* key, uint32_t address, uint8_t dir,
                      uint32_t sequence_counter, uint8_t* decBuffer )
{
    lora_crypto_payload_encrypt( buffer, size, key, address, dir, sequence_counter, decBuffer );
}

void join_compute_mic( const uint8_t* buffer, uint16_t size, const uint8_t* key, uint32_t* mic )
{
#if defined( USE_LR1110_SE )
    lr1110_crypto_mic_t    mic_value;
    lr1110_crypto_status_t status = LR1110_CRYPTO_STATUS_ERROR;

    lr1110_crypto_compute_aes_cmac( NULL, &status, get_key_index( key ), buffer, size, mic_value );
    if( status != LR1110_CRYPTO_STATUS_SUCCESS )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Failed to compute aes cmac (%d)\n", status );
    }
    status = lr1110_to_sleep( );
    if( status != LR1110_CRYPTO_STATUS_SUCCESS )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Failed to set sleep mode (%d)\n", status );
    }
#else
    uint8_t mic_value[16];

    AES_CMAC_Init( context_aes_cmac );
    AES_CMAC_SetKey( context_aes_cmac, key );
    AES_CMAC_Update( context_aes_cmac, buffer, size & 0xFF );
    AES_CMAC_Final( mic_value, context_aes_cmac );

#endif
    *mic = ( uint32_t )( ( ( ( uint32_t ) mic_value[3] ) << 24 ) | ( ( ( uint32_t ) mic_value[2] ) << 16 ) |
                         ( ( ( uint32_t ) mic_value[1] ) << 8 ) | ( ( uint32_t ) mic_value[0] ) );
}

void join_compute_skeys( const uint8_t* key, const uint8_t* appNonce, uint16_t devNonce, uint8_t* nwkSKey,
                         uint8_t* appSKey )
{
#if defined( USE_LR1110_SE )
    lr1110_crypto_status_t status = LR1110_CRYPTO_STATUS_ERROR;
    lr1110_crypto_nonce_t  nonce;
#else
    uint8_t nonce[16];
#endif

    uint8_t* pDevNonce = ( uint8_t* ) &devNonce;
    memset1( nonce, 0, sizeof( nonce ) );
    memcpy1( nonce + 1, appNonce, 6 );
    memcpy1( nonce + 7, pDevNonce, 2 );

#if defined( USE_LR1110_SE )
    nonce[0] = 0x01;
    lr1110_crypto_derive_key( NULL, &status, LR1110_CRYPTO_KEYS_IDX_NWK_KEY, LR1110_CRYPTO_KEYS_IDX_NWK_S_ENC_KEY,
                              nonce );
    if( status != LR1110_CRYPTO_STATUS_SUCCESS )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Failed to derive key NWK S KEY (%d)\n", status );
    }

    nonce[0] = 0x02;
    lr1110_crypto_derive_key( NULL, &status, LR1110_CRYPTO_KEYS_IDX_NWK_KEY, LR1110_CRYPTO_KEYS_IDX_APP_S_KEY, nonce );
    if( status != LR1110_CRYPTO_STATUS_SUCCESS )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Failed to derive key NWK S KEY (%d)\n", status );
    }

    lr1110_crypto_store_to_flash( NULL, &status );
    if( status != LR1110_CRYPTO_STATUS_SUCCESS )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Failed to store session key (%d)\n", status );
    }

    status = lr1110_to_sleep( );
    if( status != LR1110_CRYPTO_STATUS_SUCCESS )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Failed to set sleep mode (%d)\n", status );
    }

#else
    memset1( context_aes.ksch, '\0', 240 );
    aes_set_key( key, 16, &context_aes );

    nonce[0] = 0x01;
    aes_encrypt( nonce, nwkSKey, &context_aes );

    nonce[0] = 0x02;
    aes_encrypt( nonce, appSKey, &context_aes );
#endif
}

void lora_crypto_add_mic( uint8_t* buffer, uint16_t size, const uint8_t* key, uint32_t address, uint8_t dir,
                          uint32_t sequence_counter )
{
    uint32_t mic;
    compute_mic( buffer, size, key, address, dir, sequence_counter, &mic );
    memcpy( &buffer[size], ( uint8_t* ) &mic, 4 );
}

int check_mic( uint8_t* buffer, uint16_t size, const uint8_t* key, uint32_t address, uint32_t sequence_counter,
               uint32_t mic_in )
{
    uint32_t mic;

    compute_mic( buffer, size, key, address, 1, sequence_counter, &mic );
    if( mic == mic_in )
    {
        return 0;
    }
    else
    {
        return -1;
    }
}

void join_decrypt( const uint8_t* buffer, uint16_t size, const uint8_t* key, uint8_t* decBuffer )
{
#if defined( USE_LR1110_SE )
    uint8_t                mic_header = 0x20;
    lr1110_crypto_status_t status     = LR1110_CRYPTO_STATUS_ERROR;

    process_join_accept_is_success = false;

    lr1110_crypto_process_join_accept( NULL, &status, LR1110_CRYPTO_KEYS_IDX_NWK_KEY, LR1110_CRYPTO_KEYS_IDX_NWK_KEY,
                                       LR1110_CRYPTO_LORAWAN_VERSION_1_0_X, &mic_header, buffer, size, decBuffer );

    if( status != LR1110_CRYPTO_STATUS_SUCCESS )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Failed to process join accept (%d)\n", status );
    }
    else
    {
        process_join_accept_is_success = true;
    }

    status = lr1110_to_sleep( );
    if( status != LR1110_CRYPTO_STATUS_SUCCESS )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Failed to set sleep mode (%d)\n", status );
    }

#else
    memset1( context_aes.ksch, '\0', 240 );
    aes_set_key( key, 16, &context_aes );
    aes_encrypt( buffer, decBuffer, &context_aes );
    // Check if optional CFList is included
    if( size >= 16 )
    {
        aes_encrypt( buffer + 16, decBuffer + 16, &context_aes );
    }
#endif
}

int check_join_mic( const uint8_t* buffer, uint16_t size, const uint8_t* key, uint32_t micIn )
{
#if defined( USE_LR1110_SE )
    if( process_join_accept_is_success == true )
    {
        process_join_accept_is_success = false;  // Clear flag
        return 0;
    }
    else
    {
        return -1;
    }

#else
    uint32_t mic;
    join_compute_mic( buffer, size, key, &mic );
    if( mic == micIn )
    {
        return 0;
    }
    else
    {
        return -1;
    }
#endif
}

void smtc_crypto_store_app_key( const uint8_t app_key[16] )
{
#if defined( LR1110_MODEM ) || defined( USE_LR1110_SE )

    lr1110_crypto_status_t status = LR1110_CRYPTO_STATUS_ERROR;

    lr1110_crypto_set_key( NULL, &status, LR1110_CRYPTO_KEYS_IDX_NWK_KEY, app_key );

    if( status != LR1110_CRYPTO_STATUS_SUCCESS )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Failed to set app key (%d)\n", status );
    }

    lr1110_crypto_store_to_flash( NULL, &status );
    if( status != LR1110_CRYPTO_STATUS_SUCCESS )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Failed to store app key (%d)\n", status );
    }

    status = lr1110_to_sleep( );
    if( status != LR1110_CRYPTO_STATUS_SUCCESS )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Failed to set sleep mode (%d)\n", status );
    }
#endif
}

/*
 *-----------------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITIONS ------------------------------------------------
 */

#if defined( USE_LR1110_SE )
static uint8_t get_key_index( const uint8_t* key )
{
    lr1_stack_mac_t* lr1_ptr = lorawan_api_stack_mac_get( );

    if( ( ( uint32_t ) key ) == ( ( uint32_t ) & ( lr1_ptr->app_skey ) ) )
    {
        return LR1110_CRYPTO_KEYS_IDX_APP_S_KEY;
    }
    else if( ( ( uint32_t ) key ) == ( ( uint32_t ) & ( lr1_ptr->nwk_skey ) ) )
    {
        return LR1110_CRYPTO_KEYS_IDX_NWK_S_ENC_KEY;
    }
    else if( ( ( uint32_t ) key ) == ( ( uint32_t ) & ( lr1_ptr->app_key ) ) )
    {
        return LR1110_CRYPTO_KEYS_IDX_NWK_KEY;
    }
    smtc_modem_hal_mcu_panic( "Key unknown \n" );

    return 0;  // Never go here - avoid warning
}

static lr1110_status_t lr1110_to_sleep( void )
{
    lr1110_system_sleep_cfg_t lr1110_sleep_config = {
        .is_warm_start  = true,
        .is_rtc_timeout = false,
    };

    return lr1110_system_set_sleep( NULL, lr1110_sleep_config, 0 );
}
#endif

/* --- EOF ------------------------------------------------------------------ */
