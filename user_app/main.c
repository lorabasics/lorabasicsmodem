/*!
 * \file      main.c
 *
 * \brief     main program
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

#define MAIN_EXAMPLE_EXTI
//#define MAIN_EXAMPLE_ALARM_FILE_UPLOAD

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "git_version.h"
#include "smtc_bsp.h"
#include "modem_api.h"

#if defined( HW_MODEM_ENABLED )
#include "hw_modem.h"
#endif

// Prototype of main example used in this example
extern void main_exti( uint8_t* user_dev_eui, uint8_t* user_join_eui, uint8_t* user_app_key );
extern void main_alarm_file_upload( uint8_t* user_dev_eui, uint8_t* user_join_eui, uint8_t* user_app_key );

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

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

#if !defined( HW_MODEM_ENABLED )

/**
 * @brief Main function of the project
 */
int main( void )
{
    // Disable IRQ to avoid unwanted behaviour during init
    bsp_disable_irq( );

    // Configure all the µC periph (clock, gpio, timer, ...)
    bsp_mcu_init( );

    // Re-enable IRQ
    bsp_enable_irq( );

    // Define your own LoRaWAN credential
    uint8_t user_dev_eui[8]  = { 0 };
    uint8_t user_join_eui[8] = { 0 };
    uint8_t user_app_key[16] = { 0 };

#if defined( MAIN_EXAMPLE_EXTI )

    // This exemple show how to send data on an external event.
    main_exti( user_dev_eui, user_join_eui, user_app_key );

#elif defined( MAIN_EXAMPLE_ALARM_FILE_UPLOAD )

    // This exemple show how to configure an alarm and start a file upload.
    main_alarm_file_upload( user_dev_eui, user_join_eui, user_app_key );

#else
#error "Choose at least one example - see define at top of the file"
#endif
}

#else

/**
 * @brief Main function of the project
 */
int main( )
{
    uint32_t sleep_time;

    bsp_disable_irq( );

    bsp_mcu_init( );

    hw_modem_init( );

    bsp_enable_irq( );

    BSP_DBG_TRACE_INFO( "Modem is starting\n" );
    BSP_DBG_TRACE_PRINTF( "Version: %s\n", get_software_git_version( ) );
    BSP_DBG_TRACE_PRINTF( "Commit SHA1: %s\n", get_software_git_commit( ) );
    BSP_DBG_TRACE_PRINTF( "Commit date: %s\n", get_software_git_date( ) );
    BSP_DBG_TRACE_PRINTF( "Build date: %s\n", get_software_build_date( ) );

    while( 1 )
    {
        //
        if( hw_modem_is_a_cmd_available( ) == true )
        {
            hw_modem_process_cmd( );
        }

        sleep_time = modem_run_engine( );

        bsp_mcu_set_sleep_for_ms( sleep_time );
    }
}

#endif  // !HW_MODEM_ENABLED

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler( void )
{
    BSP_DBG_TRACE_ERROR( "HardFault_Handler\n" );
    while( 1 )
    {
    }
}
