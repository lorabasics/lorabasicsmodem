/*!
 * \file      cmd_parser.h
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

#ifndef __CMD_PARSER__H
#define __CMD_PARSER__H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "smtc_bsp.h"
#include "modem_api.h"
#include "modem_context.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */
/* clang-format off */
typedef enum host_cmd_type
{
    CMD_GETEVENT            = 0x00,           // Retreive Pending Event
    CMD_GETVERSION          = 0x01,           // Done
    CMD_RESET               = 0x02,           // Done
    CMD_FACTORYRESET        = 0x03,           // Done
    CMD_RESETCHARGE         = 0x04,           //
    CMD_GETCHARGE           = 0x05,           // Partially done
    CMD_GETTXPOWOFF         = 0x06,           // Unused 2.4GHZ
    CMD_SETTXPOWOFF         = 0x07,           // may be
    CMD_TEST                = 0x08,           //
    CMD_FIRMWARE            = 0x09,           // Unused 2.4GHZ
    CMD_GETTIME             = 0x0A,           // Not Yet Implemented
    CMD_GETSTATUS           = 0x0B,           // Done
    CMD_SETALARMTIMER       = 0x0C,           // Not Yet Implemented
    CMD_GETTRACE            = 0x0D,           // Unused 2.4GHZ
    CMD_GETPIN              = 0x0E,           // Unused 2.4GHZ
    CMD_GETCHIPEUI          = 0x0F,           // Unused 2.4GHZ
    CMD_GETJOINEUI          = 0x10,           // Done
    CMD_SETJOINEUI          = 0x11,           // Done
    CMD_GETDEVEUI           = 0x12,           // Done
    CMD_SETDEVEUI           = 0x13,           // Done
    CMD_SETNWKKEY           = 0x14,           // Done
    CMD_SETAPPKEY           = CMD_SETNWKKEY,  //  SAME AS SETNWKKEY
    CMD_GETCLASS            = 0x15,           // Done
    CMD_SETCLASS            = 0x16,           // Done
    CMD_SETMULTICAST        = 0x17,           // Unused 2.4GHZ
    CMD_GETREGION           = 0x18,           // Done
    CMD_SETREGION           = 0x19,           // Done
    CMD_LISTREGION          = 0x1A,           // Done
    CMD_GETADRPROFILE       = 0x1B,           // Done
    CMD_SETADRPROFILE       = 0x1C,           // Done
    CMD_GETDMPORT           = 0x1D,           // Done
    CMD_SETDMPORT           = 0x1E,           // Done
    CMD_GETDMINFOINTERVAL   = 0x1F,           // Done
    CMD_SETDMINFOINTERVAL   = 0x20,           // Done
    CMD_GETDMINFOFIELDS     = 0x21,           // Done
    CMD_SETDMINFOFIELDS     = 0x22,           // Done
    CMD_SENDDMSTATUS        = 0x23,           // Done
    CMD_SETAPPSTATUS        = 0x24,           // Done
    CMD_JOIN                = 0x25,           // Done
    CMD_LEAVENETWORK        = 0x26,           // Done
    CMD_SUSPENDMODEMCOMM    = 0x27,           // Done
    CMD_GETNEXTTXMAXPAYLOAD = 0x28,           // Done
    CMD_REQUESTTX           = 0x29,           // Done
    CMD_EMERGENCYTX         = 0x2A,           // Done
    CMD_UPLOADINIT          = 0x2B,           //  > 64bytes payload transfer
    CMD_UPLOADDATA          = 0x2C,           //  > Fragment size as a parameter for file init but < 53 bytes! set the max redundancy as parameter
    CMD_UPLOADSTART         = 0x2D,           //
    CMD_STREAMINIT          = 0x2E,           // Done
    CMD_SENDSTREAMDATA      = 0x2F,           // Done
    CMD_STREAMSTATUS        = 0x30,           // Done
    CMD_MAX
} host_cmd_type_t;

static const uint8_t host_cmd_len[CMD_MAX][2] = {
    // [CMD_xxx] = {len_min, len_max}
    [CMD_GETEVENT]            = { 0, 0 },
    [CMD_GETVERSION]          = { 0, 0 },
    [CMD_RESET]               = { 0, 0 },
    [CMD_FACTORYRESET]        = { 0, 0 },
    [CMD_RESETCHARGE]         = { 0, 0 },
    [CMD_GETCHARGE]           = { 0, 0 },
    [CMD_GETTXPOWOFF]         = { 0, 0 },
    [CMD_SETTXPOWOFF]         = { 1, 1 },
    [CMD_TEST]                = { 1, 255 },
    [CMD_FIRMWARE]            = { 5, 132 },
    [CMD_GETTIME]             = { 0, 0 },
    [CMD_GETSTATUS]           = { 0, 0 },
    [CMD_SETALARMTIMER]       = { 4, 4 },
    [CMD_GETTRACE]            = { 0, 0 },  // TODO size must be not 0
    [CMD_GETPIN]              = { 0, 0 },
    [CMD_GETCHIPEUI]          = { 0, 0 },
    [CMD_GETJOINEUI]          = { 0, 0 },
    [CMD_SETJOINEUI]          = { 8, 8 },
    [CMD_GETDEVEUI]           = { 0, 0 },
    [CMD_SETDEVEUI]           = { 8, 8 },
    [CMD_SETNWKKEY]           = { 16, 16 },
    [CMD_GETCLASS]            = { 0, 0 },
    [CMD_SETCLASS]            = { 1, 1 },
    [CMD_SETMULTICAST]        = { 40, 40 },
    [CMD_GETREGION]           = { 0, 0 },
    [CMD_SETREGION]           = { 1, 1 },
    [CMD_LISTREGION]          = { 0, 0 },
    [CMD_GETADRPROFILE]       = { 0, 0 },
    [CMD_SETADRPROFILE]       = { 1, 17 },
    [CMD_GETDMPORT]           = { 0, 0 },
    [CMD_SETDMPORT]           = { 1, 1 },
    [CMD_GETDMINFOINTERVAL]   = { 0, 0 },
    [CMD_SETDMINFOINTERVAL]   = { 1, 1 },
    [CMD_GETDMINFOFIELDS]     = { 0, 0 },
    [CMD_SETDMINFOFIELDS]     = { 0, e_inf_max },
    [CMD_SENDDMSTATUS]        = { 0, e_inf_max },
    [CMD_SETAPPSTATUS]        = { 8, 8 },
    [CMD_JOIN]                = { 0, 0 },
    [CMD_LEAVENETWORK]        = { 0, 0 },
    [CMD_SUSPENDMODEMCOMM]    = { 1, 1 },
    [CMD_GETNEXTTXMAXPAYLOAD] = { 0, 0 },
    [CMD_REQUESTTX]           = { 2, 244 },
    [CMD_EMERGENCYTX]         = { 1, 244 },
    [CMD_UPLOADINIT]          = { 6, 6 },
    [CMD_UPLOADDATA]          = { 0, 255 },
    [CMD_UPLOADSTART]         = { 4, 4 },
    [CMD_STREAMINIT]          = { 1, 2 },
    [CMD_SENDSTREAMDATA]      = { 1, 255 },
    [CMD_STREAMSTATUS]        = { 1, 1 },
};

typedef enum host_cmd_test_e
{
    CMD_TST_START     = 0x00,
    CMD_TST_NOP       = 0x01,
    CMD_TST_TX_SINGLE = 0x02,
    CMD_TST_TX_CONT   = 0x03,
    CMD_TST_TX_HOP    = 0x04,
    CMD_TST_NA_1      = 0x05,
    CMD_TST_TX_CW     = 0x06,
    CMD_TST_RX_CONT   = 0x07,
    CMD_TST_RSSI      = 0x08,
    CMD_TST_RADIO_RST = 0x09,
    CMD_TST_SPI       = 0x0A,
    CMD_TST_EXIT      = 0x0B,
    CMD_TST_BUSYLOOP  = 0x0C,
    CMD_TST_PANIC     = 0x0D,
    CMD_TST_WATCHDOG  = 0x0E,
    CMD_TST_RADIO_READ  = 0x0F,
    CMD_TST_RADIO_WRITE = 0x10,
#ifdef LORAWAN_BYPASS_ENABLED
    CMD_TST_STREAM_BYPASS   = 0x11,
    CMD_TST_STREAM_GET      = 0x12,
    CMD_TST_STREAM_DOWNLINK = 0x13,
#endif  // LORAWAN_BYPASS_ENABLED
    CMD_TST_MAX
} host_cmd_test_t;

static const uint8_t host_cmd_test_len[CMD_TST_MAX][2] = {
    // [CMD_xxx] = {len_min, len_max}
    [CMD_TST_START]     = { 8, 8 },    //
    [CMD_TST_NOP]       = { 0, 0 },    //
    [CMD_TST_TX_SINGLE] = { 9, 9 },    //
    [CMD_TST_TX_CONT]   = { 9, 9 },    //
    [CMD_TST_TX_HOP]    = { 4, 4 },    //
    [CMD_TST_NA_1]      = { 0, 0 },    //
    [CMD_TST_TX_CW]     = { 5, 5 },    //
    [CMD_TST_RX_CONT]   = { 7, 7 },    //
    [CMD_TST_RSSI]      = { 7, 7 },    //
    [CMD_TST_RADIO_RST] = { 0, 0 },    //
    [CMD_TST_SPI]       = { 1, 255 },  //
    [CMD_TST_EXIT]      = { 0, 0 },    //
    [CMD_TST_BUSYLOOP]  = { 0, 0 },    //
    [CMD_TST_PANIC]     = { 0, 0 },    //
    [CMD_TST_WATCHDOG]  = { 0, 0 },    //
    [CMD_TST_RADIO_READ]  = { 0, 255 },
    [CMD_TST_RADIO_WRITE] = { 0, 255 },
#ifdef LORAWAN_BYPASS_ENABLED
    [CMD_TST_STREAM_BYPASS] = { 1, 1 },      // Enable or disable LORAWAN BYPASS
    [CMD_TST_STREAM_GET]    = { 5, 5 },      // Get next frame when LORAWAN BYPASS is enabled.
                                             // Param: uint8_t length
                                             //        uint32_t frag_cnt
    [CMD_TST_STREAM_DOWNLINK] = { 1, 255 },  // Give downlink when LORAWAN BYPASS is enabled
#endif                                       // LORAWAN_BYPASS_ENABLED
};


typedef enum cmd_length_valid
{
    CMD_LENGTH_VALID,
    CMD_LENGTH_NOT_VALID,
} cmd_length_valid_t;

typedef enum e_parse_error
{
    PARSE_ERROR,
    PARSE_OK,
} e_parse_error_t;

typedef struct s_cmd_response
{
    modem_return_code_t return_code;
    uint8_t             length;
    uint8_t*            buffer;
} s_cmd_response_t;

typedef struct s_cmd_input
{
    host_cmd_type_t cmd_code;
    uint8_t         length;
    uint8_t*        buffer;
} s_cmd_input_t;

typedef struct s_cmd_tst_response
{
    modem_return_code_t return_code;
    uint8_t             length;
    uint8_t*            buffer;
} s_cmd_tst_response_t;

typedef struct s_cmd_tst_input
{
    host_cmd_test_t cmd_code;
    uint8_t         length;
    uint8_t*        buffer;
} s_cmd_tst_input_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/**
 * @brief todo
 *
 * @param param_1 todo
 * @param param_2 todo
 *
 * @return todo
 */
e_parse_error_t parse_cmd( s_cmd_input_t* cmd_input, s_cmd_response_t* cmd_output );

#ifdef __cplusplus
}
#endif

#endif  //__CMD_PARSER__H
