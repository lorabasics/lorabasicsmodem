/*!
 * \file      lr1mac_defs_str.h
 *
 * \brief     LoRaWan stack mac layer types strings definition
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

#ifndef __LR1MAC_DEFS_STR_H__
#define __LR1MAC_DEFS_STR_H__
#ifdef __cplusplus
extern "C" {
#endif

/*
 *-----------------------------------------------------------------------------------
 * --- DEPENDENCIES -----------------------------------------------------------------
 */

#include <stdio.h>
#include <string.h>

/*
 *-----------------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS -------------------------------------------------------------
 */
#if( MODEM_HAL_DBG_TRACE == MODEM_HAL_FEATURE_ON )
static const char* smtc_name_rx_windows[] = { "RX1", "RX2" };
static const char* smtc_name_bw[]         = { "BW007", "BW010", "BW015", "BW020", "BW031", "BW041", "BW062",
                                      "BW125", "BW200", "BW250", "BW400", "BW500", "BW800", "BW1600" };
#endif
/*
 *-----------------------------------------------------------------------------------
 * --- PUBLIC MACROS ----------------------------------------------------------------
 */

/*
 *-----------------------------------------------------------------------------------
 * --- PUBLIC TYPES ----------------------------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif  // __LR1MAC_DEFS_STR_H__
