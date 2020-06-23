/*!
 * \file      real_defs.h
 *
 * \brief     Region Abstraction Layer (REAL) API types definition
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

#ifndef __REAL_DEFS_H__
#define __REAL_DEFS_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SMTC_REAL_DUTY_CYCLE_THRESHOLD_MS_BY_HOUR ( 3600000UL )

typedef enum smtc_real_region_types_e
{
    SMTC_REAL_REGION_UNKNOWN = 0,
#if defined( REGION_EU_868 )
    SMTC_REAL_REGION_EU_868 = 1,
#endif
#if defined( REGION_AS_923 )
    SMTC_REAL_REGION_AS_923 = 2,
#endif
#if defined( REGION_US_915 )
    SMTC_REAL_REGION_US_915 = 3,
#endif
#if defined( REGION_AU_915 )
    SMTC_REAL_REGION_AU_915 = 4,
#endif
#if defined( REGION_CN_470 )
    SMTC_REAL_REGION_CN470 = 5,
#endif
#if defined( REGION_WW2G4 )
    SMTC_REAL_REGION_WW2G4 = 6
#endif
} smtc_real_region_types_t;

static const uint8_t smtc_real_region_list[] = {
#if defined( REGION_EU_868 )
    SMTC_REAL_REGION_EU_868,
#endif
#if defined( REGION_AS_923 )
    SMTC_REAL_REGION_AS_923,
#endif
#if defined( REGION_US_915 )
    SMTC_REAL_REGION_US_915,
#endif
#if defined( REGION_AU_915 )
    SMTC_REAL_REGION_AU_915,
#endif
#if defined( REGION_CN_470 )
    SMTC_REAL_REGION_CN_470,
#endif
#if defined( REGION_WW2G4 )
    SMTC_REAL_REGION_WW2G4,
#endif
};

static const uint8_t SMTC_REAL_REGION_LIST_LENGTH =
    sizeof( smtc_real_region_list ) / sizeof( smtc_real_region_list[0] );

/*!
 * Max EIRP table definition
 */
static const uint8_t smtc_real_max_eirp_dbm_from_idx[] = {
    8, 10, 12, 13, 14, 16, 18, 20, 21, 24, 26, 27, 29, 30, 33, 36
};

typedef struct smtc_real_s
{
    void*                    context;
    smtc_real_region_types_t region_type;
} smtc_real_t;

/**
 * API return status
 */
typedef enum smtc_real_status_e
{
    SMTC_REAL_STATUS_OK = 0,
    SMTC_REAL_STATUS_UNSUPPORTED_FEATURE,
    SMTC_REAL_STATUS_UNKNOWN_VALUE,
    SMTC_REAL_STATUS_ERROR,
} smtc_real_status_t;

#ifdef __cplusplus
}
#endif

#endif  // __RAL_DEFS_H__
