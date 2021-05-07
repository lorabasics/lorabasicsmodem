/*!
 * \file      file_upload.c
 *
 * \brief     File upload implementation
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
#include "file_upload.h"

#include <stdbool.h>  // bool type
#include <stdint.h>   // C99 types
#include <string.h>   //memcpy
#include "modem_services_common.h"

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
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

// 32bit pseudo hash
static uint32_t phash( uint32_t x )
{
    x = ( ( x >> 16 ) ^ x ) * 0x45d9f3b;
    x = ( ( x >> 16 ) ^ x ) * 0x45d9f3b;
    x = ( ( x >> 16 ) ^ x );
    return x;
}

static uint32_t checkbits( uint32_t cid, uint32_t cct, uint32_t i )
{
    uint32_t ncw = ( cct + 31 ) >> 5;  // number of checkwords per chunk
    return phash( cid * ncw + i );
}

static void function_xor( uint32_t* dst, uint32_t* src, int32_t nw )
{
    while( nw-- > 0 )
    {
        *dst++ ^= *src++;
    }
}

static int32_t next_session_counter( uint32_t sid )
{
    uint32_t tmp = smtc_modem_services_get_dm_upload_sctr( ) + 1;
    smtc_modem_services_set_dm_upload_sctr( tmp );
    int32_t next = ( int8_t ) smtc_modem_services_get_dm_upload_sctr( );
    return next;
}

static void gen_chunk( file_upload_t* file_upload, uint32_t* dst, uint32_t* src, uint32_t cct, uint32_t cid )
{
    memset( dst, 0, CHUNK_NW * 4 );
    uint32_t bits = 0;  // initialized to make compiler happy
    for( uint32_t i = 0; i < cct; i++ )
    {
        if( ( i & 31 ) == 0 )
        {
            bits = checkbits( cid, cct, i >> 5 );
        }
        if( bits == 0 )
        {
            continue;
        }
        if( bits & 1 )
        {
            if( i == 0 )
            {
                uint32_t tmp[2];
                tmp[0] = file_upload->header[0];
                tmp[1] = file_upload->header[1];
                function_xor( dst, tmp, CHUNK_NW );
            }
            else if( i == 1 )
            {
                uint32_t tmp[2];
                tmp[0] = file_upload->header[2];
                tmp[1] = *( src );
                function_xor( dst, tmp, CHUNK_NW );
            }
            else
            {
                function_xor( dst, src + ( CHUNK_NW * i ) - 3, CHUNK_NW );
            }
        }
        bits >>= 1;
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

int32_t file_upload_gen_uplink( file_upload_t* file_upload, unsigned char* buf, int32_t bufsz, uint32_t sid,
                                uint32_t fcnt )
{
    if( ( bufsz -= 3 ) < ( CHUNK_NW * 4 ) )
    {
        return 0;
    }
    // discriminator (16bit little endian): 2bit session id, 4bit session
    // counter, 10bit chunk count-1
    uint32_t d = ( ( sid & 0x03 ) << 14 ) | ( ( file_upload->session_counter[sid] & 0x0F ) << 10 ) |
                 ( ( file_upload->cct[sid] - 1 ) & 0x03FF );
    int32_t n     = 0;
    buf[n++]      = FILE_UPLOAD_TOKEN;
    buf[n++]      = d;
    buf[n++]      = d >> 8;
    uint32_t  cid = phash( fcnt );
    uint32_t* src = &file_upload->wordbuf[0];

    while( bufsz >= ( CHUNK_NW * 4 ) )
    {
        uint32_t tmp[CHUNK_NW];
        gen_chunk( file_upload, tmp, src, file_upload->cct[sid], cid++ );
        memcpy( buf + n, tmp, CHUNK_NW * 4 );
        n += ( CHUNK_NW * 4 );
        bufsz -= ( CHUNK_NW * 4 );
    }
    file_upload->cntx[sid] += ( n - 3 ) / ( CHUNK_NW * 4 );  // update number of chunks sent
    if( file_upload->fntx[sid] < 255 )
    {
        file_upload->fntx[sid] += 1;  // update number of frames sent
    }
    if( ( file_upload->fntx[sid] < 3 ) || ( file_upload->cntx[sid] < ( 2 * file_upload->cct[sid] ) ) )
    {
        return n;
    }
    else
    {
        return 0;
    }
}

int32_t file_upload_create( file_upload_t* file_upload, uint32_t sid, uint32_t** pdata, uint32_t sz,
                            uint16_t average_delay, uint8_t port, uint8_t encryption )
{
    uint16_t sz_tmp = sz + FILE_UPLOAD_HEADER_SIZE;
    uint32_t cct    = ( sz_tmp + ( ( 4 * CHUNK_NW ) - 1 ) ) / ( 4 * CHUNK_NW );
    if( BUFSZ < sz_tmp )
    {
        LOG_ERROR( "FileUpload is too large (%d < %d )\n", BUFSZ - 12, sz_tmp );
        return -1;
    }

    file_upload->cct[sid]             = cct;
    file_upload->average_delay[sid]   = average_delay;
    file_upload->session_counter[sid] = next_session_counter( sid );
    file_upload->cntx[sid]            = 0;
    file_upload->fntx[sid]            = 0;

    file_upload->header[0] =
        ( port ) + ( encryption << 8 ) + ( ( sz & 0xFF ) << 16 ) + ( ( ( sz & 0xFF00 ) >> 8 ) << 24 );

    return file_upload->session_counter[sid];
}

uint8_t file_upload_get_session_counter( file_upload_t* file_upload, uint32_t sid )
{
    return ( file_upload->session_counter[sid] );
}

void file_upload_set_hash( file_upload_t* file_upload, uint32_t hash0, uint32_t hash1 )
{
    file_upload->header[1] = hash0;
    file_upload->header[2] = hash1;
}

file_upload_encrypt_mode_t file_upload_get_encryption_mode( file_upload_t* file_upload )
{
    return ( file_upload_encrypt_mode_t )( ( file_upload->header[0] & 0x0000FF00 ) >> 8 );
}

void file_upload_attach_payload_buffer( file_upload_t* file_upload, uint8_t* file )
{
    file_upload->wordbuf = ( uint32_t* ) file;
}
/* --- EOF ------------------------------------------------------------------ */
