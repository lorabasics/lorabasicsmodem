/*!
 * \file      smtc_modem_hal.h
 *
 * \brief     Modem Hardware Abstraction Layer API description
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
#ifndef __SMTC_MODEM_HAL_H__
#define __SMTC_MODEM_HAL_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "smtc_modem_hal_dbg_trace.h"
#include "dm_downlink.h"  // for crashlog
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*!
 * Panic function for mcu issues
 */
#define smtc_modem_hal_mcu_panic( ... )                            \
    do                                                             \
    {                                                              \
        smtc_modem_hal_store_crashlog( ( uint8_t* ) __func__ );    \
        smtc_modem_hal_set_crashlog_status( true );                \
        SMTC_MODEM_HAL_TRACE_ERROR( "crash log :%s\n", __func__ ); \
        SMTC_MODEM_HAL_TRACE_ERROR( "-> "__VA_ARGS__ );            \
        smtc_modem_hal_reset_mcu( );                               \
    } while( 0 );

/*!
 * Panic function for lr1mac issues
 */
#define smtc_modem_hal_lr1mac_panic( ... )                         \
    do                                                             \
    {                                                              \
        smtc_modem_hal_store_crashlog( ( uint8_t* ) __func__ );    \
        smtc_modem_hal_set_crashlog_status( true );                \
        SMTC_MODEM_HAL_TRACE_ERROR( "crash log :%s\n", __func__ ); \
        SMTC_MODEM_HAL_TRACE_ERROR( "-> "__VA_ARGS__ );            \
        smtc_modem_hal_reset_mcu( );                               \
    } while( 0 );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

#define CRASH_LOG_SIZE 32

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

typedef enum
{
    CONTEXT_MODEM,
    CONTEXT_LR1MAC,
    CONTEXT_DEVNONCE,
    MODEM_CONTEXT_TYPE_SIZE
} modem_context_type_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/* ------------ Reset management ------------*/

/*!
 * Resets the MCU
 */
void smtc_modem_hal_reset_mcu( void );

/* ------------ Watchdog management ------------*/

/*!
 * Reloads watchdog counter
 *
 * \remark Application has to call this function periodically.
 *         The call period must be less than WATCHDOG_RELOAD_PERIOD
 *
 */
void smtc_modem_hal_reload_wdog( void );

/* ------------ Time management ------------*/

/*!
 * Waits for delay microseconds
 *
 * \remark This wait function must also work when called under interruption
 *
 * \param [in] delay Delay to wait in microseconds
 */
void smtc_modem_hal_wait_us( const int32_t microseconds );

/*!
 * Returns the current time in seconds
 *
 * \remark Used for scheduling autonomous retransmissions (i.e: NbTrans),
 *         transmitting MAC answers, basically any delay without accurate time
 *         constraints. It is also used to measure the time spent inside the
 *         LoRaWAN process for the integrated failsafe.
 *
 * retval rtc_time_s Current time in seconds
 */
uint32_t smtc_modem_hal_get_time_in_s( void );

/*!
 * Returns the compensated current time in seconds
 *
 * \remark Used for Clock synchronization process ALCSync which need an accurate clock with compensated drift
 *
 * retval rtc_time_s Current time in seconds
 */
uint32_t smtc_modem_hal_get_compensate_time_in_s( void );

/*!
 * Returns the time compensation in seconds
 *
 * \remark Used for Clock synchronization process ALCSync which need an accurate clock with compensated drift
 *
 * retval int32_t the positive or negative compensation offset in seconds
 */
int32_t smtc_modem_hal_get_time_compensation_in_s( void );

/*!
 * Returns the current time in milliseconds
 *
 * \remark Used to timestamp radio events (i.e: end of TX), will also be used
 * for ClassB
 *
 * retval rtc_time_ms Current time in milliseconds wraps every 49 days
 */
uint32_t smtc_modem_hal_get_time_in_ms( void );

/* ------------ Timer management ------------*/

/*!
 * Starts the provided timer objet for the given time
 *
 * \param [in] milliseconds Number of milliseconds
 * \param [in] callback     Callback that will be called in case of timer irq
 * \param [in] context      Context that will be passed on callback argument
 */
void smtc_modem_hal_start_timer( const uint32_t milliseconds, void ( *callback )( void* context ), void* context );

/*!
 * Stop the provided timer
 */
void smtc_modem_hal_stop_timer( void );

/* ------------ IRQ management ------------*/

/*!
 * Disables interruptions used in Modem (radio_dio and timer)
 */
void smtc_modem_hal_disable_modem_irq( void );

/*!
 * Enables interruptions used in Modem (radio_dio and timer)
 */
void smtc_modem_hal_enable_modem_irq( void );

/*!
 * Disable all interruptions (at core side)
 */
void smtc_modem_hal_disable_all_irq( void );

/*!
 * Enable all interruptions (at core side)
 */
void smtc_modem_hal_enable_all_irq( void );

/* ------------ Context saving management ------------*/

/*!
 * Restores the data context
 *
 * \remark This function is used to restore Modem data from a non volatile memory
 *
 *  \param ctx_type   type of modem context that need to be saved
 *  \param buffer     Buffer pointer to write to
 *  \param size       Buffer size to read in bytes
 */
void smtc_modem_hal_context_restore( const modem_context_type_t ctx_type, uint8_t* buffer, const uint32_t size );

/*!
 * Stores the data context
 *
 * \remark This function is used to store Modem data in a non volatile memory
 *
 *  \param ctx_type   type of modem context that need to be saved
 *  \param buffer     Buffer pointer to write from
 *  \param size       Buffer size to be written in bytes
 *  \retval           0 on success, negative error code on failure
 */
void smtc_modem_hal_context_store( const modem_context_type_t ctx_type, const uint8_t* buffer, const uint32_t size );

/* ------------ Crashlog management ------------*/

/*!
 * Stores the crashlog
 *
 * \remark This function is used to store the Modem crashlog in a non volatile memory
 *
 *  \param crashlog   buffer of 32 bytes containing crashlog data
 */
void smtc_modem_hal_store_crashlog( uint8_t crashlog[CRASH_LOG_SIZE] );

/*!
 * Restores the crashlog
 *
 * \remark This function is used to restore the Modem crashlog from a non volatile memory
 *
 *  \param crashlog   buffer of 32 bytes containing crashlog data
 */
void smtc_modem_hal_restore_crashlog( uint8_t crashlog[CRASH_LOG_SIZE] );

/*!
 * Stores the crashlog status
 *
 * \remark This function is used to store the Modem crashlog status in a non volatile memory. This status will
 * allow the Modem to handle crashlog send task if needed after a crash
 *
 *  \param [in] available  true if a crashlog is available, false otherwise
 */
void smtc_modem_hal_set_crashlog_status( bool available );

/*!
 * Get the previously stored crashlog status
 *
 * \remark This function is used to get the Modem crashlog status from a non volatile memory. This status will
 * allow the Modem to handle crashlog send task if needed after a crash
 *
 *  \return  true if a crashlog is available, false otherwise
 */
bool smtc_modem_hal_get_crashlog_status( void );

/* ------------ Random management ------------*/
/*!
 * Return a 32bits random number
 */
uint32_t smtc_modem_hal_get_random_nb( void );

/*!
 * Returns an unsigned random number between min and max
 *
 * \param [IN] val_1 first range unsigned value
 * \param [IN] val_2 second range unsigned value
 *
 * \retval Generated random unsigned number between smallest value and biggest value between val_1 and val_2
 */
uint32_t smtc_modem_hal_get_random_nb_in_range( const uint32_t val_1, const uint32_t val_2 );

/*!
 * Returns a signed random number between min and max
 *
 * \param [IN] val_1 first range signed value
 * \param [IN] val_2 second range signed value
 *
 * \retval Generated random signed number between smallest value and biggest value between val_1 and val_2
 */
int32_t smtc_modem_hal_get_signed_random_nb_in_range( const int32_t val_1, const int32_t val_2 );

/* ------------ Radio env management ------------*/

/*!
 * Config the radio interruption callback
 *
 * \param [in] callback     Callback that will be called in case of timer irq
 * \param [in] context      Context that will be passed on callback argument
 *
 */
void smtc_modem_hal_irq_config_radio_irq( void ( *callback )( void* context ), void* context );

/*
 * Indicates if there is a radio pending irq
 *
 * \retval pending status (true/false)
 */
bool smtc_modem_hal_irq_is_radio_irq_pending( void );

/*!
 * Start radio tcxo
 *
 * \remark In case used radio has no tcxo please implement an empty function
 */
void smtc_modem_hal_start_radio_tcxo( void );

/*!
 * Stop radio tcxo
 *
 * \remark In case used radio has no tcxo please implement an empty function
 */
void smtc_modem_hal_stop_radio_tcxo( void );

/* ------------ Environment management ------------*/

/*!
 * Return the battery level
 *
 * \return battery level for lorawan stack
 */
uint8_t smtc_modem_hal_get_battery_level( void );

/*!
 * Return MCU temperature in celsius
 */
int8_t smtc_modem_hal_get_temperature( void );

/*!
 * Return mcu voltage (can be needed for dm uplink payload)
 */
uint8_t smtc_modem_hal_get_voltage( void );

/* ------------ Trace management ------------*/

/*!
 * Prints debug trace
 *
 * \param variadics arguments
 */
void smtc_modem_hal_print_trace( const char* fmt, ... );

#ifdef __cplusplus
}
#endif

#endif  // __SMTC_MODEM_HAL_H__

/* --- EOF ------------------------------------------------------------------ */
