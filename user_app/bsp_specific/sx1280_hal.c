/*!
 * \file      sx1280_hal.c
 *
 * \brief     Implements the sx1280 HAL functions
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

#include "smtc_bsp.h"
#include "sx1280_hal.h"

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

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

// This variable will hold the current operating mode of the radio
static volatile sx1280_hal_operating_mode_t radio_opmode;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief Wait until radio busy pin is reset to 0
 */
static void sx1280_hal_wait_on_busy( void );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

sx1280_hal_status_t sx1280_hal_write( const void* context, const uint8_t* command, const uint16_t command_length,
                                      const uint8_t* data, const uint16_t data_length )
{
    sx1280_hal_wakeup( context );

    // Put NSS low to start spi transaction
    bsp_gpio_set_value( RADIO_NSS, 0 );
    for( uint16_t i = 0; i < command_length; i++ )
    {
        bsp_spi_in_out( BSP_RADIO_SPI_ID, command[i] );
    }
    for( uint16_t i = 0; i < data_length; i++ )
    {
        bsp_spi_in_out( BSP_RADIO_SPI_ID, data[i] );
    }
    // Put NSS high as the spi transaction is finished
    bsp_gpio_set_value( RADIO_NSS, 1 );

    // 0x84 - SX1280_SET_SLEEP opcode. In sleep mode the radio dio is struck to 1 => do not test it
    if( command[0] != 0x84 )
    {
        sx1280_hal_wait_on_busy( );
    }

    return SX1280_HAL_STATUS_OK;
}

sx1280_hal_status_t sx1280_hal_read( const void* context, const uint8_t* command, const uint16_t command_length,
                                     uint8_t* data, const uint16_t data_length )
{
    sx1280_hal_wakeup( context );

    // Put NSS low to start spi transaction
    bsp_gpio_set_value( RADIO_NSS, 0 );
    for( uint16_t i = 0; i < command_length; i++ )
    {
        bsp_spi_in_out( BSP_RADIO_SPI_ID, command[i] );
    }
    for( uint16_t i = 0; i < data_length; i++ )
    {
        data[i] = bsp_spi_in_out( BSP_RADIO_SPI_ID, 0 );
    }
    // Put NSS high as the spi transaction is finished
    bsp_gpio_set_value( RADIO_NSS, 1 );

    return SX1280_HAL_STATUS_OK;
}

void sx1280_hal_reset( const void* context )
{
    bsp_gpio_set_value( RADIO_NRST, 0 );
    bsp_mcu_wait_us( 5000 );
    bsp_gpio_set_value( RADIO_NRST, 1 );
    bsp_mcu_wait_us( 5000 );
}

sx1280_hal_status_t sx1280_hal_wakeup( const void* context )
{
    if( radio_opmode == SX1280_HAL_OP_MODE_SLEEP )
    {
        // Busy is HIGH in sleep mode, wake-up the device
        bsp_gpio_set_value( RADIO_NSS, 0 );
        sx1280_hal_wait_on_busy( );
        bsp_gpio_set_value( RADIO_NSS, 1 );

        // Radio is awake in STDBY_RC mode
        radio_opmode = SX1280_HAL_OP_MODE_STDBY_RC;
    }
    else
    {
        // if the radio is awake, just wait until busy pin get low
        sx1280_hal_wait_on_busy( );
    }

    return SX1280_HAL_STATUS_OK;
}

sx1280_hal_operating_mode_t sx1280_hal_get_operating_mode( const void* context )
{
    return radio_opmode;
}

void sx1280_hal_set_operating_mode( const void* context, const sx1280_hal_operating_mode_t op_mode )
{
    radio_opmode = op_mode;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void sx1280_hal_wait_on_busy( void )
{
    while( bsp_gpio_get_value( RADIO_BUSY_PIN ) == 1 )
    {
    }
}

/* --- EOF ------------------------------------------------------------------ */
