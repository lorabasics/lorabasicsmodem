/*!
 * \file      bsp_radio.c
 *
 * \brief     Implements the radio BSP functions
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

#include "smtc_bsp.h"
#include "sx126x_hal.h"
#include "sx126x.h"

typedef enum
{
    RADIO_SLEEP,
    RADIO_AWAKE
} radio_mode_t;

static radio_mode_t radio_mode = RADIO_AWAKE;

void usr_radio_waitOnBusy( void )
{
    while( bsp_gpio_get_value( RADIO_BUSY_PIN ) == 1 )
    {
    };
}

void sx126x_bsp_check_device_ready( void )
{
    // @TODO: drive antenna switch
    if( radio_mode != RADIO_SLEEP )
    {
        usr_radio_waitOnBusy( );
    }
    else
    {
        // Busy is HIGH in sleep mode, wake-up the device
        bsp_gpio_set_value( RADIO_NSS, 0 );
        usr_radio_waitOnBusy( );
        bsp_gpio_set_value( RADIO_NSS, 1 );
        radio_mode = RADIO_AWAKE;
    }
}

sx126x_hal_status_t sx126x_hal_write( const void* context, const uint8_t* command, const uint16_t command_length,
                                      const uint8_t* data, const uint16_t data_length )
{
    sx126x_bsp_check_device_ready( );

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
        sx126x_bsp_check_device_ready( );
    }
    else
    {
        radio_mode = RADIO_SLEEP;
    }

    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_read( const void* context, const uint8_t* command, const uint16_t command_length,
                                     uint8_t* data, const uint16_t data_length )
{
    sx126x_bsp_check_device_ready( );

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

    return SX126X_HAL_STATUS_OK;
}

/*!
 * Reset the radio
 *
 * \remark Must be implemented by the upper layer
 *
 * \param [in] context Radio implementation parameters
 */
void sx126x_hal_reset( const void* context )
{
    bsp_gpio_set_value( RADIO_NRST, 0 );
    bsp_mcu_wait_us( 5000 );
    bsp_gpio_set_value( RADIO_NRST, 1 );
    bsp_mcu_wait_us( 5000 );

    sx126x_set_dio2_as_rf_sw_ctrl( NULL, true );
}