/*!
 * \file      bsp_rtc.h
 *
 * \brief     Board specific package RTC API definition.
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
#ifndef __BSP_RTC_H__
#define __BSP_RTC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "smtc_bsp_types.h"

/*!
 *  Initializes the MCU RTC peripheral
 */
void bsp_rtc_init( void );

/*!
 * Returns the current RTC time in seconds
 *
 * \remark Used for scheduling autonomous retransmissions (i.e: NbTrans),
 *         transmitting MAC answers, basically any delay without accurate time
 *         constraints. It is also used to measure the time spent inside the
 *         LoRaWAN process for the integrated failsafe.
 *
 * retval rtc_time_s Current RTC time in seconds
 */
uint32_t bsp_rtc_get_time_s( void );

/*!
 * Returns the current RTC time in milliseconds
 *
 * \remark Used to timestamp radio events (i.e: end of TX), will also be used
 * for ClassB
 *
 * retval rtc_time_ms Current RTC time in milliseconds wraps every 49 days
 */
uint32_t bsp_rtc_get_time_ms( void );

/*!
 * Waits delay milliseconds by polling RTC
 *
 * \param[IN] milliseconds Delay in ms
 */
void bsp_rtc_delay_in_ms( const uint32_t milliseconds );

/*!
 * Sets the rtc wakeup timer for seconds parameter. The RTC will generate an IRQ
 * to wakeup the MCU.
 *
 * \param[IN] seconds Number of seconds before wakeup
 */
void bsp_rtc_wakeup_timer_set_s( const int32_t seconds );

/*!
 * Sets the rtc wakeup timer for milliseconds parameter. The RTC will generate
 * an IRQ to wakeup the MCU.
 *
 * \param[IN] milliseconds Number of seconds before wakeup
 */
void bsp_rtc_wakeup_timer_set_ms( const int32_t milliseconds );

#ifdef __cplusplus
}
#endif

#endif  // __BSP_RTC_H__
