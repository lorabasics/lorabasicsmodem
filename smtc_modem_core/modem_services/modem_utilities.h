/*!
 * \file      modem_utilities.h
 *
 * \brief     utilities for modem
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

#ifndef __MODEM_UTILITIES_H__
#define __MODEM_UTILITIES_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */
#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "lr1mac_defs.h"

#if defined( LR1110_MODEM )
#define START_FLASH_ADRESS ( uint32_t )( BASE_ADDR )
#define FLASH_PROG_LEN ( uint32_t )( BASE_ADDR + 0xF4 )  // Address where is located the prog len
#endif                                                   // LR1110_MODEM

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * \brief   Compute SHA256
 * \remark
 *
 * \param  [out]    hash*           - Contains the computed hash
 * \param  [in]     msg*            - input buffer
 * \param  [in]     len*            - input buffer length
 * \retval [out]    None
 */
void sha256( uint32_t* hash, const uint8_t* msg, uint32_t len );

/*!
 * \brief   Compute crc
 * \remark
 *
 * \param  [in]     buf*            - input buffer
 * \param  [in]     len*            - input buffer length
 * \retval [out]    crc             - computed crc
 */
uint32_t crc( const uint8_t* buf, int len );
uint8_t  crc8( const uint8_t* data, int length );
uint32_t compute_crc_fw( void );
#ifdef __cplusplus
}
#endif

#endif  // __MODEM_UTILITIES_H__

/* --- EOF ------------------------------------------------------------------ */
