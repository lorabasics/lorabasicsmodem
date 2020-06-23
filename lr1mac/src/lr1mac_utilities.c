/*!
 * \file      lr1_mac_utilities.c
 *
 * \brief     LoRaWan utilities definition
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
#include <stdbool.h>

#include "lr1mac_utilities.h"
#include "smtc_bsp.h"
#include "lr1mac_defs.h"

void memcpy1( uint8_t* dst, const uint8_t* src, uint16_t size )
{
    while( size-- )
    {
        *dst++ = *src++;
    }
}

void memset1( uint8_t* dst, uint8_t value, uint16_t size )
{
    while( size-- )
    {
        *dst++ = value;
    }
}

uint32_t lr1mac_utilities_crc( uint8_t* buf, int len )
{
    uint32_t crc = 0xFFFFFFFA;
    while( len-- > 0 )
    {
        crc = crc ^ *buf++;
        for( int i = 0; i < 8; i++ )
        {
            uint32_t mask = -( crc & 1 );
            crc           = ( crc >> 1 ) ^ ( 0xEDB88320 & mask );
        }
    }
    return ~crc + 3;
}

uint32_t lr1mac_utilities_get_symb_time_us( const uint16_t nb_symb, const ral_lora_sf_t sf, const ral_lora_bw_t bw )
{
    uint32_t sf_val;
    uint32_t bw_khz;

    // clang-format off
    switch(sf)
    {
        case RAL_LORA_SF5:  sf_val = 5;     break;
        case RAL_LORA_SF6:  sf_val = 6;     break;
        case RAL_LORA_SF7:  sf_val = 7;     break;
        case RAL_LORA_SF8:  sf_val = 8;     break;
        case RAL_LORA_SF9:  sf_val = 9;     break;
        case RAL_LORA_SF10: sf_val = 10;    break;
        case RAL_LORA_SF11: sf_val = 11;    break;
        case RAL_LORA_SF12: sf_val = 12;    break;
        default:
            return 0;
            break;
    }
    switch (bw)
    {
        case RAL_LORA_BW_007_KHZ:   bw_khz = 7;      break;
        case RAL_LORA_BW_010_KHZ:   bw_khz = 10;     break;
        case RAL_LORA_BW_015_KHZ:   bw_khz = 15;     break;
        case RAL_LORA_BW_020_KHZ:   bw_khz = 20;     break;
        case RAL_LORA_BW_031_KHZ:   bw_khz = 31;     break;
        case RAL_LORA_BW_041_KHZ:   bw_khz = 41;     break;
        case RAL_LORA_BW_062_KHZ:   bw_khz = 62;     break;
        case RAL_LORA_BW_125_KHZ:   bw_khz = 125;    break;
        case RAL_LORA_BW_200_KHZ:   bw_khz = 203;    break;
        case RAL_LORA_BW_250_KHZ:   bw_khz = 250;    break;
        case RAL_LORA_BW_400_KHZ:   bw_khz = 406;    break;
        case RAL_LORA_BW_500_KHZ:   bw_khz = 500;    break;
        case RAL_LORA_BW_800_KHZ:   bw_khz = 812;    break;
        case RAL_LORA_BW_1600_KHZ:  bw_khz = 1625;   break;
         default:
            return 0;
            break;
    }
    // clang-format on

    return ( ( ( uint32_t ) nb_symb * 1000 ) << sf_val ) / bw_khz;
}
