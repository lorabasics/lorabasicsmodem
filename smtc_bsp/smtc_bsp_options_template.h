/*!
 * \file      bsp_options.h
 *
 * \brief     Board specific package OPTIONS management API definition.
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
#ifndef __BSP_OPTIONS_H__
#define __BSP_OPTIONS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "smtc_bsp_types.h"

// clang-format off
#define BSP_FEATURE_OFF                             0
#define BSP_FEATURE_ON                              !BSP_FEATURE_OFF

#define BSP_DBG_TRACE                               BSP_FEATURE_ON
#define BSP_DBG_TRACE_COLOR                         BSP_FEATURE_ON
#define BSP_DBG_TRACE_RP                            BSP_FEATURE_OFF
#define BSP_DBG_TRACE_STORAGE                       BSP_FEATURE_OFF

// BSP_FEATURE_ON to activate sleep mode
// BSP_FEATURE_OFF to replace by wait functions (easier in debug mode)
#define BSP_LOW_POWER_MODE                          BSP_FEATURE_OFF

// BSP_FEATURE_ON to enable debug probe, not disallocating corresponding pins
#define BSP_HW_DEBUG_PROBE                          BSP_FEATURE_ON

#define BSP_USE_DBG_PINS                            BSP_FEATURE_ON

#define BSP_USE_USER_UART                           BSP_FEATURE_OFF
#define BSP_USER_UART_ID                            1

#define BSP_USE_PRINTF_UART                         BSP_FEATURE_ON
#define BSP_PRINTF_UART_ID                          2

#define BSP_RADIO_SPI_ID                            1

/*!
 * Watchdog counter reload value
 *
 * \remark The period must be lower than MCU watchdog period
 */
#define BSP_WATCHDOG_RELOAD_PERIOD_SECONDS          20

// clang-format on

#ifdef __cplusplus
}
#endif

#endif  // __BSP_OPTIONS_H__
