/*!
 * \file      smtc_bsp_tmr.h
 *
 * \brief     Board specific package HW timer API definition.
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

#ifndef __SMTC_BSP_TMR_H__
#define __SMTC_BSP_TMR_H__

#ifdef __cplusplus
extern "C" {
#endif
/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*!
 * Timer IRQ handling data context
 */
typedef struct bsp_tmr_irq_s
{
    void* context;
    void ( *callback )( void* context );
} bsp_tmr_irq_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 *  Initializes the MCU TMR peripheral
 */
void bsp_tmr_init( void );

/*!
 * Starts the provided timer objet for the given time
 *
 * \param [in] milliseconds Number of milliseconds
 * \param [in] tmr_irq      Timer IRQ handling data ontext
 */
void bsp_tmr_start( const uint32_t milliseconds, const bsp_tmr_irq_t* tmr_irq );

/*!
 * Starts the provided timer objet for the given time
 */
void bsp_tmr_stop( void );

/*!
 * Returns the current TMR time in milliseconds
 *
 * \remark is used to timestamp radio events (end of TX), will also be used for
 *
 * retval tmr_time_ms Current TMR time in milliseconds wraps every 49 days
 */
uint32_t bsp_tmr_get_time_ms( void );

/*!
 * Enables timer interrupts (HW timer only)
 */
void bsp_tmr_irq_enable( void );

/*!
 * Disables timer interrupts (HW timer only)
 */
void bsp_tmr_irq_disable( void );

#ifdef __cplusplus
}
#endif

#endif  // __SMTC_BSP_TMR_H__

/* --- EOF ------------------------------------------------------------------ */
