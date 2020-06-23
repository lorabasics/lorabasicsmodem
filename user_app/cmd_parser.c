/*!
 * \file      cmd_parser.c
 *
 * \brief     command parser for hw modem over soft modem
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

#include "cmd_parser.h"
#include "test_mode.h"
#include "modem_api.h"
#include "lorawan_api.h"
#include "ral.h"
#include "modem_utilities.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static uint8_t  file_store[2048]    = { 0 };
static uint16_t file_size           = 0;
static uint16_t upload_current_size = 0;

static modem_return_code_t upload_data( uint8_t* payload, uint8_t payload_length, uint8_t* file_strore );
#if BSP_DBG_TRACE == BSP_FEATURE_ON
static const char* HostCmdStr[CMD_MAX] = {
    [CMD_GETEVENT]            = "GETEVENT",
    [CMD_GETVERSION]          = "GETVERSION",
    [CMD_RESET]               = "RESET",
    [CMD_FACTORYRESET]        = "FACTORYRESET",
    [CMD_RESETCHARGE]         = "RESETCHARGE",
    [CMD_GETCHARGE]           = "GETCHARGE",
    [CMD_GETTXPOWOFF]         = "GETTXPOWOFF",
    [CMD_SETTXPOWOFF]         = "SETTXPOWOFF",
    [CMD_TEST]                = "TEST",
    [CMD_FIRMWARE]            = "FIRMWARE",
    [CMD_GETTIME]             = "GETTIME",
    [CMD_GETSTATUS]           = "GETSTATUS",
    [CMD_SETALARMTIMER]       = "SETALARMTIMER",
    [CMD_GETTRACE]            = "GETTRACE",
    [CMD_GETPIN]              = "GETPIN",
    [CMD_GETCHIPEUI]          = "GETCHIPEUI",
    [CMD_GETJOINEUI]          = "GETJOINEUI",
    [CMD_SETJOINEUI]          = "SETJOINEUI",
    [CMD_GETDEVEUI]           = "GETDEVEUI",
    [CMD_SETDEVEUI]           = "SETDEVEUI",
    [CMD_SETNWKKEY]           = "SETNWKKEY",
    [CMD_GETCLASS]            = "GETCLASS",
    [CMD_SETCLASS]            = "SETCLASS",
    [CMD_SETMULTICAST]        = "SETMULTICAST",
    [CMD_GETREGION]           = "GETREGION",
    [CMD_SETREGION]           = "SETREGION",
    [CMD_LISTREGION]          = "LISTREGION",
    [CMD_GETADRPROFILE]       = "GETADRPROFILE",
    [CMD_SETADRPROFILE]       = "SETADRPROFILE",
    [CMD_GETDMPORT]           = "GETDMPORT",
    [CMD_SETDMPORT]           = "SETDMPORT",
    [CMD_GETDMINFOINTERVAL]   = "GETDMINFOINTERVAL",
    [CMD_SETDMINFOINTERVAL]   = "SETDMINFOINTERVAL",
    [CMD_GETDMINFOFIELDS]     = "GETDMINFOFIELDS",
    [CMD_SETDMINFOFIELDS]     = "SETDMINFOFIELDS",
    [CMD_SENDDMSTATUS]        = "SENDDMSTATUS",
    [CMD_SETAPPSTATUS]        = "SETAPPSTATUS",
    [CMD_JOIN]                = "JOIN",
    [CMD_LEAVENETWORK]        = "LEAVENETWORK",
    [CMD_SUSPENDMODEMCOMM]    = "SUSPENDMODEMCMD",
    [CMD_GETNEXTTXMAXPAYLOAD] = "GETNEXTTXMAXPAYLOAD",
    [CMD_REQUESTTX]           = "REQUESTTX",
    [CMD_EMERGENCYTX]         = "EMERGENCYTX",
    [CMD_UPLOADINIT]          = "UPLOADINIT",
    [CMD_UPLOADDATA]          = "UPLOADDATA",
    [CMD_UPLOADSTART]         = "UPLOADSTART",
    [CMD_STREAMINIT]          = "STREAMINIT",
    [CMD_SENDSTREAMDATA]      = "SENDSTREAMDATA",
    [CMD_STREAMSTATUS]        = "STREAMSTATUS",
};
#endif

#if BSP_DBG_TRACE == BSP_FEATURE_ON
static const char* host_cmd_test_str[CMD_TST_MAX] = {
    [CMD_TST_START]       = "START",        //
    [CMD_TST_NOP]         = "NOP",          //
    [CMD_TST_TX_SINGLE]   = "TX_SINGLE",    //
    [CMD_TST_TX_CONT]     = "TX_CONT",      //
    [CMD_TST_TX_HOP]      = "TX_HOP",       //
    [CMD_TST_NA_1]        = "NA_1",         //
    [CMD_TST_TX_CW]       = "TX_CW",        //
    [CMD_TST_RX_CONT]     = "RX_CONT",      //
    [CMD_TST_RSSI]        = "RSSI",         //
    [CMD_TST_RADIO_RST]   = "RADIO_RST",    //
    [CMD_TST_SPI]         = "SPI",          //
    [CMD_TST_EXIT]        = "EXIT",         //
    [CMD_TST_BUSYLOOP]    = "BUSYLOOP",     //
    [CMD_TST_PANIC]       = "PANIC",        //
    [CMD_TST_WATCHDOG]    = "WATCHDOG",     //
    [CMD_TST_RADIO_READ]  = "RADIO_READ",   //
    [CMD_TST_RADIO_WRITE] = "RADIO_WRITE",  //
#ifdef LORAWAN_BYPASS_ENABLED
    [CMD_TST_STREAM_BYPASS]   = "STREAM_BYPASS",    //
    [CMD_TST_STREAM_GET]      = "STREAM_GET",       //
    [CMD_TST_STREAM_DOWNLINK] = "STREAM_DOWNLINK",  //
#endif                                              // LORAWAN_BYPASS_ENABLED
};
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * \brief   check cmd Size
 * \remark
 * \param [IN]  a pointer containing the command input
 * \param [OUT] a pointer containing the command output
 * @return CMD_LENGTH_VALID is length is valid, CMD_LENGTH_NOT_VALID otherwise
 */
cmd_length_valid_t CheckCmdSize( s_cmd_input_t* cmd_input, s_cmd_response_t* cmd_output );

/**
 * @brief check test cmd size
 * \param [IN]  a pointer containing the command input
 * \param [OUT] a pointer containing the command output
 * @return CMD_LENGTH_VALID is length is valid, CMD_LENGTH_NOT_VALID otherwise
 */
cmd_length_valid_t cmd_test_parser_check_cmd_size( host_cmd_test_t tst_id, uint8_t length );

/*!
 * \brief parse commands for test mode
 * \remark
 * \param [IN]  payload         : payload test
 * \param [IN]  payload_length  : payload test length
 * \param [OUT]
 */
static e_parse_error_t cmd_test_parser( s_cmd_tst_input_t* cmd_tst_input, s_cmd_tst_response_t* cmd_tst_output );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

e_parse_error_t parse_cmd( s_cmd_input_t* cmd_input, s_cmd_response_t* cmd_output )
{
    e_parse_error_t ret = PARSE_OK;

    if( cmd_input->cmd_code >= CMD_MAX )
    {
        BSP_DBG_TRACE_ERROR( "Unknown command (0x%x)\n", cmd_input->cmd_code );
        cmd_output->return_code = RC_INVALID;
        cmd_output->length      = 0;
        return PARSE_ERROR;
    }

    if( test_mode_enabled_is( ) == true && cmd_input->cmd_code != CMD_TEST )
    {
#if BSP_DBG_TRACE == BSP_FEATURE_ON
        BSP_DBG_TRACE_ERROR( "Command not valid in test mode (0x%x : %s)\n", cmd_input->cmd_code,
                             HostCmdStr[cmd_input->cmd_code] );
#endif
        cmd_output->return_code = RC_INVALID;
        cmd_output->length      = 0;
        return PARSE_ERROR;
    }

    if( CheckCmdSize( cmd_input, cmd_output ) == CMD_LENGTH_NOT_VALID )
    {
        cmd_output->return_code = RC_BAD_SIZE;
        cmd_output->length      = 0;
        return PARSE_ERROR;
    }
    cmd_output->return_code = RC_OK;  // by default the return code is ok and length is 0
    cmd_output->length      = 0;

    BSP_DBG_TRACE_WARNING( "CMD_%s (0x%02x)\n", HostCmdStr[cmd_input->cmd_code], cmd_input->cmd_code );
    switch( cmd_input->cmd_code )
    {
    case CMD_GETEVENT: {
        uint8_t event_data_length      = 0;
        uint8_t asynchronous_msgnumber = 0;

        cmd_output->return_code =
            modem_get_event( &cmd_output->buffer[0], &cmd_output->buffer[1], &cmd_output->buffer[2], &event_data_length,
                             &asynchronous_msgnumber );
        if( asynchronous_msgnumber > 0 )
        {
            cmd_output->length = event_data_length + 2;
        }
        else
        {
            // ie asynchronous_msgnumber == 0
            // de-assert hw_modem irq line to indicate host that all events have been retrieved
            bsp_gpio_set_value( HW_MODEM_EVENT_PIN, 0 );
        }

        break;
    }
    case CMD_GETVERSION: {
        uint32_t bootloader;
        uint32_t firmware;
        uint16_t lorawan;

        cmd_output->return_code = modem_get_version( &bootloader, &firmware, &lorawan );

        // Bootloader version
        cmd_output->buffer[0] = ( bootloader >> 24 ) & 0xff;
        cmd_output->buffer[1] = ( bootloader >> 16 ) & 0xff;
        cmd_output->buffer[2] = ( bootloader >> 8 ) & 0xff;
        cmd_output->buffer[3] = ( bootloader & 0xff );
        // Firmware version
        cmd_output->buffer[4] = ( firmware >> 24 ) & 0xff;
        cmd_output->buffer[5] = ( firmware >> 16 ) & 0xff;
        cmd_output->buffer[6] = ( firmware >> 8 ) & 0xff;
        cmd_output->buffer[7] = ( firmware & 0xff );
        // LoRaWAN version 1.0.3 (BCD)
        cmd_output->buffer[8] = lorawan >> 8;
        cmd_output->buffer[9] = lorawan;

        cmd_output->length = 10;
        break;
    }
    case CMD_RESET:  //@todo save the reset counter inside the eeprom
                     //  lorawan_api_context_load( );  // may be should be manage by a mcu interface not inside the
                     //  stack?
        cmd_output->return_code = modem_reset( );

        break;
    case CMD_FACTORYRESET:  //@todo save the reset counter inside the eeprom
        cmd_output->return_code = modem_factory_reset( );
        break;
    case CMD_RESETCHARGE:
        cmd_output->return_code = modem_reset_charge( );
        break;
    case CMD_GETCHARGE: {
        uint32_t charge         = 0;
        cmd_output->return_code = modem_get_charge( &charge );
        cmd_output->buffer[0]   = ( charge >> 24 ) & 0xFF;
        cmd_output->buffer[1]   = ( charge >> 16 ) & 0xFF;
        cmd_output->buffer[2]   = ( charge >> 8 ) & 0xFF;
        cmd_output->buffer[3]   = ( charge & 0xFF );

        cmd_output->length = 4;
        break;
    }
    case CMD_GETTXPOWOFF: {
        int8_t tmp              = 0;
        cmd_output->return_code = modem_get_tx_power_offset( &tmp );
        cmd_output->buffer[0]   = tmp;
        cmd_output->length      = 1;
        break;
    }
    case CMD_SETTXPOWOFF:
        cmd_output->return_code = modem_set_tx_power_offset( cmd_input->buffer[0] );
        break;
    case CMD_FIRMWARE:  // Write FW update
        cmd_output->return_code = RC_NOT_IMPLEMENTED;
        cmd_output->length      = 0;
        break;
    case CMD_GETTIME:
        cmd_output->return_code = RC_NOT_IMPLEMENTED;
        cmd_output->length      = 0;
        break;
    case CMD_GETSTATUS:
        cmd_output->return_code = modem_get_status( &cmd_output->buffer[0] );
        cmd_output->length      = 1;
        break;
    case CMD_SETALARMTIMER: {
        uint32_t alarm = 0;
        alarm |= cmd_input->buffer[0] << 24;
        alarm |= cmd_input->buffer[1] << 16;
        alarm |= cmd_input->buffer[2] << 8;
        alarm |= cmd_input->buffer[3];

        cmd_output->return_code = modem_set_alarm_timer_s( alarm );
        break;
    }
    case CMD_GETTRACE:
        cmd_output->return_code = RC_NOT_IMPLEMENTED;
        cmd_output->length      = 0;
        break;
    case CMD_GETPIN:
        cmd_output->return_code = RC_NOT_IMPLEMENTED;
        cmd_output->length      = 0;
        break;
    case CMD_GETCHIPEUI:
        cmd_output->return_code = RC_NOT_IMPLEMENTED;
        cmd_output->length      = 0;
        break;
    case CMD_GETJOINEUI:
        cmd_output->return_code = modem_get_joineui( &cmd_output->buffer[0] );
        cmd_output->length      = 8;
        break;
    case CMD_SETJOINEUI:
        cmd_output->return_code = modem_set_joineui( &cmd_input->buffer[0] );
        break;
    case CMD_GETDEVEUI:
        cmd_output->return_code = modem_get_deveui( &cmd_output->buffer[0] );
        cmd_output->length      = 8;
        break;
    case CMD_SETDEVEUI:
        cmd_output->return_code = modem_set_deveui( &cmd_input->buffer[0] );
        break;
    case CMD_SETNWKKEY:  // same as CMD_SETAPPKEY,  @todo nwk key and app key !!!!
        cmd_output->return_code = modem_set_nwkkey( &cmd_input->buffer[0] );
        break;
    case CMD_GETCLASS:
        cmd_output->return_code = modem_get_class( &cmd_output->buffer[0] );
        cmd_output->length      = 1;
        break;
    case CMD_SETCLASS:
        cmd_output->return_code = modem_set_class( cmd_input->buffer[0] );
        break;
    case CMD_SETMULTICAST:
        cmd_output->return_code = RC_NOT_IMPLEMENTED;
        cmd_output->length      = 0;
        break;
    case CMD_GETREGION:
        cmd_output->return_code = modem_get_region( &cmd_output->buffer[0] );
        cmd_output->length      = 1;
        break;
    case CMD_SETREGION:
        cmd_output->return_code = modem_set_region( cmd_input->buffer[0] );
        break;
    case CMD_LISTREGION:  // 2.4GHZ modem support only one region
        cmd_output->return_code = modem_list_region( &cmd_output->buffer[0], &cmd_output->length );
        break;
    case CMD_GETADRPROFILE:
        cmd_output->return_code = modem_get_adr_profile( &cmd_output->buffer[0] );
        cmd_output->length      = 1;
        break;
    case CMD_SETADRPROFILE:
        cmd_output->return_code = modem_set_adr_profile( cmd_input->buffer[0], &cmd_input->buffer[1] );
        break;
    case CMD_GETDMPORT:
        cmd_output->return_code = modem_get_dm_port( &cmd_output->buffer[0] );
        cmd_output->length      = 1;
        break;
    case CMD_SETDMPORT:
        cmd_output->return_code = modem_set_dm_port( cmd_input->buffer[0] );
        break;
    case CMD_GETDMINFOINTERVAL:
        cmd_output->return_code = modem_get_dm_info_interval( &cmd_output->buffer[0] );
        cmd_output->length      = 1;
        break;
    case CMD_SETDMINFOINTERVAL:
        cmd_output->return_code = modem_set_dm_info_interval( cmd_input->buffer[0] );
        break;
    case CMD_GETDMINFOFIELDS:
        cmd_output->return_code = modem_get_dm_info_fields( &cmd_output->buffer[0], &cmd_output->length );
        break;
    case CMD_SETDMINFOFIELDS:
        cmd_output->return_code = modem_set_dm_info_fields( &cmd_input->buffer[0], cmd_input->length );
        break;
    case CMD_SENDDMSTATUS:
        cmd_output->return_code = modem_send_dm_status( &cmd_input->buffer[0], cmd_input->length );
        break;
    case CMD_SETAPPSTATUS:
        cmd_output->return_code = modem_set_app_status( &cmd_input->buffer[0], cmd_input->length );
        break;
    case CMD_JOIN:
        cmd_output->return_code = modem_join( );
        break;
    case CMD_LEAVENETWORK:
        cmd_output->return_code = modem_leave_network( );
        break;
    case CMD_SUSPENDMODEMCOMM:
        cmd_output->return_code = modem_suspend_modem_comm( cmd_input->buffer[0] );
        break;
    case CMD_GETNEXTTXMAXPAYLOAD:
        cmd_output->return_code = modem_get_next_tx_max_payload( &cmd_output->buffer[0] );
        if( cmd_output->return_code == RC_OK )
        {
            cmd_output->length = 1;
        }
        break;
    case CMD_REQUESTTX:
        cmd_output->return_code = modem_request_tx( cmd_input->buffer[0], cmd_input->buffer[1], &cmd_input->buffer[2],
                                                    cmd_input->length - 2 );
        break;
    case CMD_EMERGENCYTX:
        cmd_output->return_code = modem_emergency_tx( cmd_input->buffer[0], cmd_input->buffer[1], &cmd_input->buffer[2],
                                                      cmd_input->length - 2 );
        break;
    case CMD_UPLOADINIT: {
        uint16_t size          = 0;
        uint16_t average_delay = 0;

        size = cmd_input->buffer[2] << 8;
        size |= cmd_input->buffer[3];

        average_delay = cmd_input->buffer[4] << 8;
        average_delay |= cmd_input->buffer[5];

        file_size = size;

        upload_current_size = 0;
        if( file_size > BSP_FILE_UPLOAD_MAX_SIZE )
        {
            cmd_output->return_code = RC_FAIL;
        }
        else
        {
            cmd_output->return_code =
                modem_upload_init( cmd_input->buffer[0], cmd_input->buffer[1], size, average_delay );
        }
        break;
    }
    case CMD_UPLOADDATA:
        cmd_output->return_code = upload_data( &cmd_input->buffer[0], cmd_input->length, file_store );
        break;
    case CMD_UPLOADSTART: {
        uint32_t input_crc = 0;
        input_crc |= cmd_input->buffer[0] << 24;
        input_crc |= cmd_input->buffer[1] << 16;
        input_crc |= cmd_input->buffer[2] << 8;
        input_crc |= cmd_input->buffer[3];

        if( file_size != upload_current_size )
        {
            cmd_output->return_code = RC_BAD_SIZE;
            BSP_DBG_TRACE_ERROR( "RC_BAD_SIZE in FileUpload\n" );
        }
        else if( input_crc != crc( file_store, file_size ) )
        {
            cmd_output->return_code = RC_BAD_CRC;
            BSP_DBG_TRACE_ERROR( "RC_BAD_CRC in FileUpload\n" );
        }
        else
        {
            cmd_output->return_code = modem_upload_start( file_store, file_size );
        }
        break;
    }
    case CMD_STREAMINIT:
        cmd_output->return_code = RC_NOT_IMPLEMENTED;
        cmd_output->length      = 0;
        break;
    case CMD_SENDSTREAMDATA:
        cmd_output->return_code = RC_NOT_IMPLEMENTED;
        cmd_output->length      = 0;
        break;
    case CMD_STREAMSTATUS:
        cmd_output->return_code = RC_NOT_IMPLEMENTED;
        cmd_output->length      = 0;
        break;
    case CMD_TEST: {
        s_cmd_tst_input_t    cmd_tst_input;
        s_cmd_tst_response_t cmd_tst_output;

        cmd_tst_input.cmd_code = ( host_cmd_test_t ) cmd_input->buffer[0];
        cmd_tst_input.length   = cmd_input->length - 1;
        cmd_tst_input.buffer   = &cmd_input->buffer[1];
        cmd_tst_output.buffer  = &cmd_output->buffer[0];

        ret = cmd_test_parser( &cmd_tst_input, &cmd_tst_output );

        cmd_output->return_code = cmd_tst_output.return_code;
        cmd_output->length      = cmd_tst_output.length;
        break;
    }
    default:
        cmd_output->return_code = RC_NOT_IMPLEMENTED;
        cmd_output->length      = 0;
        ret                     = PARSE_ERROR;
        break;
    }
    // BSP_DBG_TRACE_ARRAY ("cmd output", cmd_output->buffer, cmd_output->length);
    cmd_input->cmd_code = CMD_MAX;
    cmd_input->length   = 0;

    return ret;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

cmd_length_valid_t CheckCmdSize( s_cmd_input_t* cmd_input, s_cmd_response_t* cmd_output )
{
    // cmd len too small
    if( cmd_input->length < host_cmd_len[cmd_input->cmd_code][0] )
    {
        BSP_DBG_TRACE_ERROR( "Invalid command size (too small)\n" );
        return CMD_LENGTH_NOT_VALID;
    }
    // cmd len too long
    if( cmd_input->length > host_cmd_len[cmd_input->cmd_code][1] )
    {
        BSP_DBG_TRACE_ERROR( "Invalid command size (too long)\n" );
        return CMD_LENGTH_NOT_VALID;
    }
    return CMD_LENGTH_VALID;
}

cmd_length_valid_t cmd_test_parser_check_cmd_size( host_cmd_test_t tst_id, uint8_t length )
{
    // cmd len too small
    if( length < host_cmd_test_len[tst_id][0] )
    {
        BSP_DBG_TRACE_ERROR( "Invalid command test size (too small)\n" );
        return CMD_LENGTH_NOT_VALID;
    }
    // cmd len too long
    if( length > host_cmd_test_len[tst_id][1] )
    {
        BSP_DBG_TRACE_ERROR( "Invalid command test size (too long)\n" );
        return CMD_LENGTH_NOT_VALID;
    }
    return CMD_LENGTH_VALID;
}

static e_parse_error_t cmd_test_parser( s_cmd_tst_input_t* cmd_tst_input, s_cmd_tst_response_t* cmd_tst_output )
{
    if( test_mode_enabled_is( ) == false && cmd_tst_input->cmd_code != CMD_TST_START )
    {
        BSP_DBG_TRACE_ERROR( "Command not valid, test mode not started\n" );
        cmd_tst_output->return_code = RC_INVALID;
        cmd_tst_output->length      = 0;
        return PARSE_ERROR;
    }

    if( cmd_tst_input->cmd_code >= CMD_TST_MAX )
    {
        BSP_DBG_TRACE_ERROR( "Unknown command test (0x%x)\n", cmd_tst_input->cmd_code );
        cmd_tst_output->return_code = RC_INVALID;
        cmd_tst_output->length      = 0;
        return PARSE_ERROR;
    }

    if( cmd_test_parser_check_cmd_size( cmd_tst_input->cmd_code, cmd_tst_input->length ) == CMD_LENGTH_NOT_VALID )
    {
        BSP_DBG_TRACE_ERROR( "Invalid size command test (0x%x)\n", cmd_tst_input->cmd_code );
        cmd_tst_output->return_code = RC_BAD_SIZE;
        cmd_tst_output->length      = 0;
        return PARSE_ERROR;
    }

    cmd_tst_output->return_code = RC_OK;  // by default the return code is ok and length is 0
    cmd_tst_output->length      = 0;

#if BSP_DBG_TRACE == BSP_FEATURE_ON
    BSP_DBG_TRACE_WARNING( "\tCMD_TST_%s (0x%02x)\n", host_cmd_test_str[cmd_tst_input->cmd_code],
                           cmd_tst_input->cmd_code );
#endif
    switch( cmd_tst_input->cmd_code )
    {
    case CMD_TST_START:

        if( get_join_state( ) != MODEM_NOT_JOINED )
        {
            BSP_DBG_TRACE_ERROR( "TST MODE: not available if joined\n" );
            cmd_tst_output->return_code = RC_FAIL;
        }
        else if( strncmp( ( char* ) cmd_tst_input->buffer, "TESTTEST", 8 ) == 0 )
        {
            cmd_tst_output->return_code = test_mode_start( );
        }
        else
        {
            BSP_DBG_TRACE_ERROR( "TST MODE: invalid enablement payload\n" );
            cmd_tst_output->return_code = RC_INVALID;
        }

        break;
    case CMD_TST_NOP:
        cmd_tst_output->return_code = test_mode_nop( );
        break;
    case CMD_TST_TX_SINGLE: {
        uint32_t frequency = 0;
        frequency |= cmd_tst_input->buffer[0] << 24;
        frequency |= cmd_tst_input->buffer[1] << 16;
        frequency |= cmd_tst_input->buffer[2] << 8;
        frequency |= cmd_tst_input->buffer[3];

        cmd_tst_output->return_code =
            test_mode_tx( frequency, cmd_tst_input->buffer[4], cmd_tst_input->buffer[5], cmd_tst_input->buffer[6],
                          cmd_tst_input->buffer[7], cmd_tst_input->buffer[8], TEST_MODE_TX_SINGLE );
        break;
    }
    case CMD_TST_TX_CONT: {
        uint32_t frequency = 0;
        frequency |= cmd_tst_input->buffer[0] << 24;
        frequency |= cmd_tst_input->buffer[1] << 16;
        frequency |= cmd_tst_input->buffer[2] << 8;
        frequency |= cmd_tst_input->buffer[3];

        cmd_tst_output->return_code =
            test_mode_tx( frequency, cmd_tst_input->buffer[4], cmd_tst_input->buffer[5], cmd_tst_input->buffer[6],
                          cmd_tst_input->buffer[7], cmd_tst_input->buffer[8], TEST_MODE_TX_CONTINUE );
        break;
    }
    case CMD_TST_TX_HOP:
        cmd_tst_output->return_code = RC_NOT_IMPLEMENTED;
        cmd_tst_output->length      = 0;
        break;
    case CMD_TST_TX_CW: {
        uint32_t frequency = 0;
        frequency |= cmd_tst_input->buffer[0] << 24;
        frequency |= cmd_tst_input->buffer[1] << 16;
        frequency |= cmd_tst_input->buffer[2] << 8;
        frequency |= cmd_tst_input->buffer[3];

        cmd_tst_output->return_code = test_mode_cw( frequency, cmd_tst_input->buffer[4] );
        break;
    }
    case CMD_TST_RX_CONT: {
        uint32_t frequency = 0;
        frequency |= cmd_tst_input->buffer[0] << 24;
        frequency |= cmd_tst_input->buffer[1] << 16;
        frequency |= cmd_tst_input->buffer[2] << 8;
        frequency |= cmd_tst_input->buffer[3];

        cmd_tst_output->return_code = test_mode_rx_cont( frequency, cmd_tst_input->buffer[4], cmd_tst_input->buffer[5],
                                                         cmd_tst_input->buffer[6] );
        break;
    }
    case CMD_TST_RSSI:
        cmd_tst_output->return_code = RC_NOT_IMPLEMENTED;
        cmd_tst_output->length      = 0;
        break;
    case CMD_TST_RADIO_RST:
        cmd_tst_output->return_code = test_mode_radio_reset( );
        break;
    case CMD_TST_SPI:
        cmd_tst_output->return_code = RC_NOT_IMPLEMENTED;
        cmd_tst_output->length      = 0;
        break;
    case CMD_TST_EXIT:
        cmd_tst_output->return_code = test_mode_exit( );
        break;
    case CMD_TST_BUSYLOOP:
        cmd_tst_output->return_code = RC_NOT_IMPLEMENTED;
        cmd_tst_output->length      = 0;
        break;
    case CMD_TST_PANIC:
        cmd_tst_output->return_code = RC_NOT_IMPLEMENTED;
        cmd_tst_output->length      = 0;
        break;
    case CMD_TST_WATCHDOG:
        cmd_tst_output->return_code = RC_NOT_IMPLEMENTED;
        cmd_tst_output->length      = 0;
        break;
    case CMD_TST_RADIO_READ: {
        uint8_t command_length = cmd_tst_input->buffer[0];
        uint8_t command[255]   = { 0 };
        memcpy( command, &cmd_tst_input->buffer[1], command_length );

        uint8_t data_length = cmd_tst_input->buffer[command_length + 1];
        uint8_t data[255]   = { 0 };

        cmd_tst_output->return_code = test_mode_radio_read( command, command_length, data, data_length );
        memcpy( cmd_tst_output->buffer, data, data_length );
        cmd_tst_output->length = data_length;
        break;
    }
    case CMD_TST_RADIO_WRITE: {
        uint8_t command_length = cmd_tst_input->buffer[0];
        uint8_t command[255]   = { 0 };
        memcpy( command, &cmd_tst_input->buffer[1], command_length );

        uint8_t data_length = cmd_tst_input->buffer[command_length + 1];
        uint8_t data[255]   = { 0 };
        memcpy( data, &cmd_tst_input->buffer[command_length + 1], data_length );

        cmd_tst_output->return_code = test_mode_radio_write( command, command_length, data, data_length );
        cmd_tst_output->length      = data_length;
        break;
    }
#ifdef LORAWAN_BYPASS_ENABLED
    case CMD_TST_STREAM_BYPASS: {
        uint8_t enabled = cmd_tst_input->buffer[0];
        if( enabled )
        {
            BSP_DBG_TRACE_INFO( "TST MODE: ENABLE STREAM BYPASS\n" );
        }
        else
        {
            BSP_DBG_TRACE_INFO( "TST MODE: DISABLE STREAM BYPASS\n" );
        }
        modem_stream_bypass_enable( enabled != 0 );
        cmd_tst_output->return_code = RC_OK;
        cmd_tst_output->length      = 0;
    }
    break;
    case CMD_TST_STREAM_GET: {
        uint8_t  len      = cmd_tst_input->buffer[0];
        uint32_t frag_cnt = 0;
        frag_cnt |= cmd_tst_input->buffer[1];
        frag_cnt |= cmd_tst_input->buffer[2] << 8;
        frag_cnt |= cmd_tst_input->buffer[3] << 16;
        frag_cnt |= cmd_tst_input->buffer[4] << 24;
        BSP_DBG_TRACE_PRINTF( "TST MODE: GET STREAM FRAGMENT %u %u\n", len, frag_cnt );
        cmd_tst_output->return_code = modem_stream_bypass_get_fragment( cmd_tst_output->buffer, frag_cnt, &len );
        cmd_tst_output->length      = len;
    }
    break;
    case CMD_TST_STREAM_DOWNLINK: {
        uint8_t* data = &( cmd_tst_input->buffer[0] );
        BSP_DBG_TRACE_INFO( "TST MODE: STREAM DOWNLINK\n" );
        cmd_tst_output->return_code = modem_stream_bypass_send_downlink( data, cmd_tst_input->length );
        cmd_tst_output->length      = 0;
    }
    break;
#endif  // LORAWAN_BYPASS_ENABLED
    default:
        cmd_tst_output->return_code = RC_UNKNOWN;
        cmd_tst_output->length      = 0;
        break;
    }

    // Erase test command content to avoid twice calls
    cmd_tst_input->cmd_code = CMD_TST_MAX;
    cmd_tst_input->length   = 0;

    return PARSE_OK;
}

static modem_return_code_t upload_data( uint8_t* payload, uint8_t payload_length, uint8_t* file_strore )
{
    modem_return_code_t return_code = RC_OK;

    if( ( modem_get_upload_state( ) != MODEM_UPLOAD_INIT ) && ( modem_get_upload_state( ) != MODEM_UPLOAD_DATA ) )
    {
        return_code = RC_NOT_INIT;
        BSP_DBG_TRACE_ERROR( "Upload file data, not init\n" );
    }
    else if( payload == NULL )
    {
        return_code = RC_NOT_INIT;
        BSP_DBG_TRACE_ERROR( "Upload file data, null\n" );
    }
    else if( ( upload_current_size + payload_length ) > file_size )
    {
        return_code = RC_INVALID;
        BSP_DBG_TRACE_ERROR( "Upload file data, size invalid\n" );
    }
    else if( get_modem_status_file_upload( ) == true )
    {
        return_code = RC_INVALID;
        BSP_DBG_TRACE_ERROR( "Upload file still on going\n" );
    }
    else
    {
        memcpy( ( uint8_t* ) file_strore + upload_current_size, payload, payload_length );
        upload_current_size += payload_length;
        modem_set_upload_state( MODEM_UPLOAD_DATA );
    }

    return return_code;
}
