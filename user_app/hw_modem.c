/*!
 * \file      hw_modem.c
 *
 * \brief     handle hw part of the modem
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

#include "hw_modem.h"
#include "cmd_parser.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define HW_MODEM_RX_BUFF_MAX_LENGTH 259

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static uint8_t        ModemResponsePacket[HW_MODEM_RX_BUFF_MAX_LENGTH];
static uint8_t        ModemRxBuffer[HW_MODEM_RX_BUFF_MAX_LENGTH];
static uint8_t        ResponseLength;
static volatile bool  hw_cmd_available             = false;
static volatile bool  is_hw_modem_ready_to_receive = true;
static bsp_gpio_irq_t wakeup_line_irq              = { 0 };

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief prepare and start the reception of the command on a uart using a dma
 * @param [none]
 * @return [none]
 */
void hw_modem_start_reception( void );

/**
 * @brief function that will be called every time the COMMAND line in asserted or de-asserted by the host
 * @param *context  unused context
 * @return none
 */
void wakeup_line_irq_handler( void* constext );

/**
 * @brief function that will be called by the soft modem engine each time an async event is available
 * @param *context  unused context
 * @return none
 */
void hw_modem_event_handler( void );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void hw_modem_init( void )
{
    // init hw modem pins
    bsp_gpio_init_out( HW_MODEM_EVENT_PIN, 0 );
    bsp_gpio_init_out( HW_MODEM_BUSY_PIN, 1 );

    // init irq on COMMAND pin
    wakeup_line_irq.pin      = HW_MODEM_COMMAND_PIN;
    wakeup_line_irq.context  = NULL;
    wakeup_line_irq.callback = wakeup_line_irq_handler;
    bsp_gpio_init_in( HW_MODEM_COMMAND_PIN, BSP_GPIO_PULL_MODE_UP, BSP_GPIO_IRQ_MODE_RISING_FALLING, &wakeup_line_irq );

    // todo remove below
    memset( ModemResponsePacket, 0, HW_MODEM_RX_BUFF_MAX_LENGTH );
    hw_cmd_available             = false;
    is_hw_modem_ready_to_receive = true;

    // init the soft modem
    modem_init( &hw_modem_event_handler );

#if defined( PERF_TEST_ENABLED )
    BSP_PERF_TEST_TRACE_PRINTF( "HARDWARE MODEM RUNNING PERF TEST MODE\n" );
#endif
}

void hw_modem_start_reception( void )
{
    memset( ModemRxBuffer, 0xFF, HW_MODEM_RX_BUFF_MAX_LENGTH );

    // during the receive process the hw modem cannot accept an other cmd, prevent it
    is_hw_modem_ready_to_receive = false;

    // receive on dma
    bsp_uart1_dma_start_rx( ModemRxBuffer, HW_MODEM_RX_BUFF_MAX_LENGTH );

    // indicate to bridge or host that the modem is ready to receive on uart
    bsp_gpio_set_value( HW_MODEM_BUSY_PIN, 0 );
}

void hw_modem_process_cmd( void )
{
    uint8_t             CmdLength = 0xFF;
    s_cmd_response_t    output;
    s_cmd_input_t       input;
    modem_return_code_t ResponseReturnCode;

    // todo: is this check below realy needed?
    if( ModemRxBuffer[0] < 0xFF )
    {
        CmdLength   = ModemRxBuffer[1];
        uint8_t Lrc = 0;
        for( int i = 0; i < CmdLength + 2; i++ )
        {
            Lrc = Lrc ^ ModemRxBuffer[i];
        }
        uint8_t         CalculatedLrc = Lrc;
        uint8_t         CmdLrc        = ModemRxBuffer[CmdLength + 2];
        host_cmd_type_t CmdType       = ( host_cmd_type_t ) ModemRxBuffer[0];

        if( CalculatedLrc != CmdLrc )
        {
            ResponseReturnCode = RC_FRAME_ERROR;
            ResponseLength     = 0;
            BSP_DBG_TRACE_PRINTF( "Cmd with bad crc %x / %x", CalculatedLrc, CmdLrc );
        }
        else if( ( ModemRxBuffer[CmdLength + 3] != 0xFF ) && ( CmdLength != 0xFF ) )
        {
            // Too Many cmd enqueued
            ResponseReturnCode = RC_FRAME_ERROR;
            ResponseLength     = 0;
            BSP_DBG_TRACE_WARNING( " Extra data after the command\n" );
        }
        else  // go into soft modem
        {
            BSP_DBG_TRACE_ARRAY( "Cmd input uart", ModemRxBuffer, CmdLength + 2 );
            input.cmd_code = CmdType;
            input.length   = CmdLength;
            input.buffer   = &ModemRxBuffer[2];
            output.buffer  = &ModemResponsePacket[2];
            parse_cmd( &input, &output );
            ResponseReturnCode = output.return_code;
            ResponseLength     = output.length;
        }

        ModemResponsePacket[0] = ResponseReturnCode;
        ModemResponsePacket[1] = ResponseLength;

        BSP_DBG_TRACE_ARRAY( "Cmd output on uart", ModemResponsePacket, ResponseLength + 2 );

        // now the hw modem can accept new commands
        is_hw_modem_ready_to_receive = true;
        hw_cmd_available             = false;

        // set busy pin to indicate to bridge or host that the hw_modem answer will be soon sent
        bsp_gpio_set_value( HW_MODEM_BUSY_PIN, 1 );

        // wait to to bridge delay
        bsp_mcu_wait_us( 1000 );

        for( int i = 0; i < ResponseLength + 2; i++ )
        {
            Lrc = Lrc ^ ModemResponsePacket[i];
        }
        ModemResponsePacket[ResponseLength + 2] = Lrc;

        // bsp_uart_put_buffer( 1, ModemResponsePacket, ResponseLength + 3 );
        bsp_uart1_tx( ModemResponsePacket, ResponseLength + 3 );
    }
    else
    {
        // now the hw modem can accept new commands
        is_hw_modem_ready_to_receive = true;
        hw_cmd_available             = false;

        // set busy pin to indicate to bridge or host that the hw_modem answer will be soon sent
        bsp_gpio_set_value( HW_MODEM_BUSY_PIN, 1 );
    }
}

bool hw_modem_is_a_cmd_available( void )
{
    return hw_cmd_available;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

void wakeup_line_irq_handler( void* constext )
{
    if( ( bsp_gpio_get_value( HW_MODEM_COMMAND_PIN ) == 0 ) && ( is_hw_modem_ready_to_receive == true ) )
    {
        // start receiving uart with dma
        hw_modem_start_reception( );

        // force exit of stop mode
        bsp_mcu_disable_low_power_wait( );

        // TEMPORARY WORKAROUND to avoid issue for print in hw_modem_process_cmd function
        bsp_mcu_wait_us( 2000 );
    }
    if( ( bsp_gpio_get_value( HW_MODEM_COMMAND_PIN ) == 1 ) && ( is_hw_modem_ready_to_receive == false ) )
    {
        // stop uart on dma reception
        bsp_uart1_dma_stop_rx( );

        // inform that a command has arrived
        hw_cmd_available = true;

        // force one more loop in main loop and then re-enable low power feature
        bsp_mcu_disable_once_low_power_wait( );
    }
}

void hw_modem_event_handler( void )
{
    // raise the event line to indicate to host that events are available
    bsp_gpio_set_value( HW_MODEM_EVENT_PIN, 1 );
    BSP_DBG_TRACE_MSG_COLOR( "CB\n", BSP_DBG_TRACE_COLOR_BLUE );
}

/* --- EOF ------------------------------------------------------------------ */
