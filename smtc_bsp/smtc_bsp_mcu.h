/*!
 * \file      smtc_bsp_mcu.h
 *
 * \brief     Board specific package MCU API definition.
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
#ifndef __SMTC_BSP_MCU_H__
#define __SMTC_BSP_MCU_H__

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

/*!
 * Begins critical section
 */
#define CRITICAL_SECTION_BEGIN( ) \
    uint32_t mask;                \
    bsp_mcu_critical_section_begin( &mask )

/*!
 * Ends critical section
 */
#define CRITICAL_SECTION_END( ) bsp_mcu_critical_section_end( &mask )

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * Disable interrupts, begins critical section
 *
 * \param [IN] mask Pointer to a variable where to store the CPU IRQ mask
 */
void bsp_mcu_critical_section_begin( uint32_t* mask );

/*!
 * Ends critical section
 *
 * \param [IN] mask Pointer to a variable where the CPU IRQ mask was stored
 */
void bsp_mcu_critical_section_end( uint32_t* mask );

/*!
 * Disables MCU peripherals specific IRQs
 */
void bsp_mcu_disable_periph_irq( void );

/*!
 * Enables MCU peripherals specific IRQs
 */
void bsp_mcu_enable_periph_irq( void );

/*!
 * Initializes BSP used MCU
 */
void bsp_mcu_init( void );

/*!
 * Disable irq at core side
 */
void bsp_disable_irq( void );

/*!
 * Enable irq at core side
 */
void bsp_enable_irq( void );

/*!
 * Resets the MCU
 */
void bsp_mcu_reset( void );

/*!
 * To be called in case of panic @mcu side
 */
void bsp_mcu_panic( void );

/*!
 * To be called in case of lr1mac stack issue
 */
void bsp_mcu_handle_lr1mac_issue( void );

/*!
 * This function is called when a soft modem reset is asked
 */
void bsp_mcu_modem_need_reset( void );

/*!
 * Initializes BSP used MCU radio pins
 *
 * \param [IN] context Pointer to a variable holding the communication interface
 *                     id as well as the radio pins assignment.
 */
void bsp_mcu_init_radio( const void* context );

/*!
 * Sets the MCU in sleep mode for the given number of seconds.
 *
 * \param[IN] seconds Number of seconds to stay in sleep mode
 */
void bsp_mcu_set_sleep_for_s( const int32_t seconds );

/*!
 * Sets the MCU in sleep mode for the given number of milliseconds.
 *
 * \param[IN] milliseconds Number of milliseconds to stay in sleep mode
 */
void bsp_mcu_set_sleep_for_ms( const int32_t milliseconds );

/*!
 * Waits for delay microseconds
 *
 * \param [in] delay Delay to wait in microseconds
 */
void bsp_mcu_wait_us( const int32_t microseconds );

/*!
 * Return the battery level
 *
 * \return battery level for lorawan stack
 */
uint8_t bsp_mcu_get_battery_level( void );

/*!
 * Prints debug trace
 *
 * \param variadics arguments
 */
void bsp_trace_print( const char* fmt, ... );

/*!
 * Suspend low power process and avoid looping on it
 */
void bsp_mcu_disable_low_power_wait( void );

/*!
 * Enable low power process
 */
void bsp_mcu_enable_low_power_wait( void );

/*!
 * Suspend once low power process and avoid looping on it once
 */
void bsp_mcu_disable_once_low_power_wait( void );

/*!
 * Return MCU temperature in celsius
 */
int32_t bsp_mcu_get_mcu_temperature( void );

/*!
 * Return mcu voltage (can be needed for dm uplink payload)
 */
uint8_t bsp_mcu_get_mcu_voltage( void );

#ifdef __cplusplus
}
#endif

#endif  // __SMTC_BSP_MCU_H__

/* --- EOF ------------------------------------------------------------------ */
