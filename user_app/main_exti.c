/*!
 * \file      main_exti.c
 *
 * \brief     main program for exti example
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
#include "modem_api.h"
#include "device_management_defs.h"

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
static volatile bool user_button_is_press = false;  // Flag for button status
static uint8_t       rx_payload[255]      = { 0 };  // Buffer for rx payload
static uint8_t       rx_payload_size      = 0;      // Size of the payload in the rx_payload buffer

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */
static uint8_t is_joined( void );
static void    get_event( void );
static void    user_button_callback( void* context );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

/**
 * @brief Example to send a user payload on an external event
 *
 * @param user_dev_eui  LoRaWAN dev EUI
 * @param user_join_eui LoRaWAN join EUI
 * @param user_app_key  LoRaWAN app Key
 */
void main_exti( uint8_t* user_dev_eui, uint8_t* user_join_eui, uint8_t* user_app_key )
{
    // Disable IRQ to avoid unwanted behaviour during init
    bsp_disable_irq( );

    // Init the modem and use get_event as event callback, please note that the callback will be
    // called immediatly after the first call to modem_run_engine because of the reset detection
    modem_init( &get_event );

    // Configure Nucleo blue button as EXTI
    bsp_gpio_irq_t nucleo_blue_button = {
        .context  = NULL,                  // context pass to the callback - not used in this example
        .callback = user_button_callback,  // callback called when EXTI is triggered
    };
    bsp_gpio_init_in( PC_13, BSP_GPIO_PULL_MODE_NONE, BSP_GPIO_IRQ_MODE_FALLING, &nucleo_blue_button );

    // Re-enable IRQ
    bsp_enable_irq( );

    // Set euis and keys - (If the credentials have already been configured, the device will reload them from NVM, so
    // it's not a mandatory action)
    modem_set_deveui( user_dev_eui );
    modem_set_joineui( user_join_eui );
    modem_set_nwkkey( user_app_key );

    // Join LoRaWAN network
    modem_join( );

    BSP_DBG_TRACE_INFO( "EXTI exemple is starting \n" );

    while( 1 )
    {
        // Execute modem runtime, this function must be recalled in sleep_time_ms (max value, can be recalled sooner)
        uint32_t sleep_time_ms = modem_run_engine( );

        // Check if a button has been pressed
        if( user_button_is_press == true )
        {
            // Clear button flag
            user_button_is_press = false;

            // Check if the device has already joined a network
            if( is_joined( ) != 0 )
            {
                // Send MCU temperature on port 102
                uint8_t temperature = ( uint8_t ) bsp_mcu_get_mcu_temperature( );
                BSP_DBG_TRACE_INFO( "MCU temperature : %d \n", temperature );
                modem_request_tx( 102, TX_UNCONFIRMED, &temperature, 1 );
            }
        }
        else
        {
            // nothing to process, go to sleep (if low power is enabled)
            bsp_mcu_set_sleep_for_ms( sleep_time_ms );
        }
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/**
 * @brief Join status of the product
 *
 * @return Return 1 if the device is joined else 0
 */
static uint8_t is_joined( void )
{
    uint8_t status = 0;
    modem_get_status( &status );
    return ( ( status >> modem_status_joined ) & 0x01 );
}

/**
 * @brief User callback for modem event
 *
 *  This callback is called every time an event ( see modem_rsp_event_t ) appears in the modem.
 *  Several events may have to be read from the modem when this callback is called.
 */
static void get_event( void )
{
    BSP_DBG_TRACE_MSG_COLOR( "get_event () callback\n", BSP_DBG_TRACE_COLOR_BLUE );

    modem_rsp_event_t type                   = RSP_NUMBER;
    uint8_t           count                  = 0;
    uint8_t           event_data[255]        = { 0 };
    uint8_t           event_data_length      = 0;
    uint8_t           asynchronous_msgnumber = 0;

    // Read modem event type and number
    modem_get_event( &type, &count, event_data, &event_data_length, &asynchronous_msgnumber );

    // Continue to read modem event until all event has been processed
    while( asynchronous_msgnumber > 0 )
    {
        BSP_DBG_TRACE_WARNING( "Event %u \n", type );
        switch( type )
        {
        case RSP_RESET:
            break;

        case RSP_ALARM:
            break;

        case RSP_JOINED:
            BSP_DBG_TRACE_INFO( "Modem is join \n" );
            break;

        case RSP_TXDONE:
            BSP_DBG_TRACE_INFO( "TX done \n" );
            break;

        case RSP_DOWNDATA:
            rx_payload_size = event_data_length - 4;
            memcpy( rx_payload, &event_data[4], rx_payload_size );
            BSP_DBG_TRACE_PRINTF( "Data received on port %u\n", event_data[3] );
            BSP_DBG_TRACE_ARRAY( "DOWNDATA", rx_payload, rx_payload_size );
            break;

        case RSP_FILEDONE:
            break;

        case RSP_SETCONF:
            break;

        case RSP_MUTE:
            break;

        case RSP_LINKSTATUS:
            break;

        default:
            BSP_DBG_TRACE_ERROR( "Unknown event %u\n", type );
            break;
        }

        modem_get_event( &type, &count, event_data, &event_data_length, &asynchronous_msgnumber );
    }
}

/**
 * @brief User callback for button EXTI
 *
 * @param context Define by the user at the init
 */
static void user_button_callback( void* context )
{
    ( void ) context;  // Not used in the example - avoid warning

    static uint32_t last_press_timestamp_ms = 0;

    // Debounce the button press, avoid multiple triggers
    if( ( int32_t )( bsp_rtc_get_time_ms( ) - last_press_timestamp_ms ) > 500 )
    {
        last_press_timestamp_ms = bsp_rtc_get_time_ms( );
        user_button_is_press    = true;

        // When the button is pressed, the device is likely to be in low power mode. In this low power mode
        // implementation, low power needs to be disabled once to leave the low power loop and process the button
        // action.
        bsp_mcu_disable_once_low_power_wait( );
    }
}
