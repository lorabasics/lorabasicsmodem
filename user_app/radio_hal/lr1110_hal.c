/*!
 * \file      lr1110_hal.c
 *
 * \brief     Implements the lr1110 radio HAL functions
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
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "lr1110_hal.h"
#include "smtc_hal_gpio.h"
#include "smtc_hal_spi.h"
#include "smtc_hal_mcu.h"

#include "modem_pinout.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

typedef enum
{
    RADIO_SLEEP,
    RADIO_AWAKE
} radio_mode_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static radio_mode_t radio_mode = RADIO_AWAKE;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief Wait until radio busy pin returns to 0
 */
void lr1110_hal_wait_on_busy( void );

/**
 * @brief Check if device is ready to receive spi transaction.
 * @remark If the device is in sleep mode, it will awake it and wait until it is ready
 */
void lr1110_hal_check_device_ready( void );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

lr1110_hal_status_t lr1110_hal_write( const void* context, const uint8_t* command, const uint16_t command_length,
                                      const uint8_t* data, const uint16_t data_length )
{
    lr1110_hal_check_device_ready( );

    // Put NSS low to start spi transaction
    hal_gpio_set_value( RADIO_NSS, 0 );
    for( uint16_t i = 0; i < command_length; i++ )
    {
        hal_spi_in_out( RADIO_SPI_ID, command[i] );
    }
    for( uint16_t i = 0; i < data_length; i++ )
    {
        hal_spi_in_out( RADIO_SPI_ID, data[i] );
    }
    // Put NSS high as the spi transaction is finished
    hal_gpio_set_value( RADIO_NSS, 1 );

    // LR1110_SYSTEM_SET_SLEEP_OC=0x011B opcode. In sleep mode the radio busy line is held at 1 => do not test it
    if( ( command[0] == 0x01 ) && ( command[1] == 0x1B ) )
    {
        radio_mode = RADIO_SLEEP;
    }
#if defined( HW_MODEM_ENABLED )
    // During sniff wifi or gnss busy pin is stuck at 1 which avoid the hw modem to respond to the bridge at time
    // So bypass the busy pin wait in this case. Warning, the wait has to be done by the hw_modem external user
    // LR1110 GNSS_SNIFF_AUTO_TRX=0x0409 opcode.
    else if( ( command[0] == 0x04 ) && ( command[1] == 0x09 ) )
    {
        // do nothing
    }
    // LR1110 GNSS_SNIFF_ASSIST_TRX = 0x040A opcode.
    else if( ( command[0] == 0x04 ) && ( command[1] == 0x0A ) )
    {
        // do nothing
    }
    // LR1110 WIFI_SCAN  = 0x0300 opcode.
    else if( ( command[0] == 0x03 ) && ( command[1] == 0x00 ) )
    {
        // do nothing
    }
    // LR1110 WIFI_SCAN_TIME_LIMIT = 0x0301 opcode.
    else if( ( command[0] == 0x03 ) && ( command[1] == 0x01 ) )
    {
        // do nothing
    }
    // LR1110 WIFI_SEARCH_COUNTRY_CODE = 0x0302 opcode.
    else if( ( command[0] == 0x03 ) && ( command[1] == 0x02 ) )
    {
        // do nothing
    }
    // LR1110 WIFI_SEARCH_COUNTRY_CODE_TIME_LIMIT = 0x0303 opcode.
    else if( ( command[0] == 0x03 ) && ( command[1] == 0x03 ) )
    {
        // do nothing
    }
#endif
    else
    {
        lr1110_hal_check_device_ready( );
    }

    return LR1110_HAL_STATUS_OK;
}

lr1110_hal_status_t lr1110_hal_read( const void* context, const uint8_t* command, const uint16_t command_length,
                                     uint8_t* data, const uint16_t data_length )
{
    lr1110_hal_check_device_ready( );

    // Put NSS low to start spi transaction
    hal_gpio_set_value( RADIO_NSS, 0 );
    for( uint16_t i = 0; i < command_length; i++ )
    {
        hal_spi_in_out( RADIO_SPI_ID, command[i] );
    }
    hal_gpio_set_value( RADIO_NSS, 1 );

    if( data_length > 0 )
    {
        lr1110_hal_check_device_ready( );
        hal_gpio_set_value( RADIO_NSS, 0 );
        // dummy
        hal_spi_in_out( RADIO_SPI_ID, 0 );

        for( uint16_t i = 0; i < data_length; i++ )
        {
            data[i] = hal_spi_in_out( RADIO_SPI_ID, 0 );
        }
        // Put NSS high as the spi transaction is finished
        hal_gpio_set_value( RADIO_NSS, 1 );
    }

    return LR1110_HAL_STATUS_OK;
}

lr1110_hal_status_t lr1110_hal_write_read( const void* context, const uint8_t* command, uint8_t* data,
                                           const uint16_t data_length )
{
    lr1110_hal_check_device_ready( );

    // Put NSS low to start spi transaction
    hal_gpio_set_value( RADIO_NSS, 0 );

    for( uint16_t i = 0; i < data_length; i++ )
    {
        data[i] = hal_spi_in_out( RADIO_SPI_ID, command[i] );
    }
    hal_gpio_set_value( RADIO_NSS, 1 );

    return LR1110_HAL_STATUS_OK;
}

lr1110_hal_status_t lr1110_hal_reset( const void* context )
{
    hal_gpio_set_value( RADIO_NRST, 0 );
    hal_mcu_wait_us( 5000 );
    hal_gpio_set_value( RADIO_NRST, 1 );
    hal_mcu_wait_us( 5000 );

    // Wait 200ms until internal lr1110 fw is ready
    hal_mcu_wait_us( 200000 );
    radio_mode = RADIO_AWAKE;

    return LR1110_HAL_STATUS_OK;
}

lr1110_hal_status_t lr1110_hal_wakeup( const void* context )
{
    lr1110_hal_check_device_ready( );
    return LR1110_HAL_STATUS_OK;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

void lr1110_hal_wait_on_busy( void )
{
    while( hal_gpio_get_value( RADIO_BUSY_PIN ) == 1 )
    {
    };
}

void lr1110_hal_check_device_ready( void )
{
    if( radio_mode != RADIO_SLEEP )
    {
        lr1110_hal_wait_on_busy( );
    }
    else
    {
        // Busy is HIGH in sleep mode, wake-up the device with a small glitch on NSS
        hal_gpio_set_value( RADIO_NSS, 0 );
        hal_gpio_set_value( RADIO_NSS, 1 );
        lr1110_hal_wait_on_busy( );
        radio_mode = RADIO_AWAKE;
    }
}

/* --- EOF ------------------------------------------------------------------ */
