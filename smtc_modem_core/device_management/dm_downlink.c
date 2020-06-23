/*!
 * \file      dm_downlink.c
 *
 * \brief     handle downlink from device management
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
#include "dm_downlink.h"
#include "device_management_defs.h"
#include "modem_context.h"
#include "lorawan_api.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

#if BSP_DBG_TRACE == BSP_FEATURE_ON
static const char* dm_cmd_str[DM_CMD_MAX] = {
    [DM_RESET]       = "RESET",        //
    [DM_FUOTA]       = "FUOTA",        //
    [DM_FILE_DONE]   = "FILE_DONE",    //
    [DM_GET_INFO]    = "GET_INFO",     //
    [DM_SET_CONF]    = "SET_CONF",     //
    [DM_REJOIN]      = "REJOIN",       //
    [DM_MUTE]        = "MUTE",         //
    [DM_SET_DM_INFO] = "SET_DM_INFO",  //
    [DM_STREAM]      = "STREAM",       //
    [DM_ALC_SYNC]    = "ALC_SYNC"      //
};
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * \brief   check DM cmd Size
 *
 * \param [in]  cmd                         Current dm code that must be checked
 * \param [in]  length                      Length of the tested requested code
 * \param [out] e_dm_cmd_length_valid       Return valid length or not
 */
static e_dm_cmd_length_valid dm_check_cmd_size( e_dm_cmd_t cmd, uint8_t length );

/*!
 * \brief   DM Reset
 *
 * \param [in]  reset_code                  Type of requested reset
 * \param [in]  reset_session               reset Session is compare to the current number of modem reset
 * \param [out] e_dm_error_t                Return valid or not
 */
static e_dm_error_t dm_reset( e_dm_reset_code_t reset_code, uint16_t reset_session );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

e_dm_error_t dm_downlink( uint8_t* data, uint8_t length )
{
    s_dm_cmd_input_t dm_input;
    if( length <= DM_DOWNLINK_HEADER_LENGTH )
    {
        BSP_DBG_TRACE_ERROR( "DM Downlink must contain at least 4 bytes\n" );
        return DM_ERROR;
    }

    dm_input.up_count     = data[0];
    dm_input.up_delay     = data[1];
    dm_input.request_code = ( e_dm_cmd_t ) data[2];
    dm_input.buffer       = &data[3];
    dm_input.buffer_len   = length - DM_DOWNLINK_HEADER_LENGTH;

    set_dm_retrieve_pending_dl( dm_input.up_count, dm_input.up_delay );

    if( dm_input.up_count > 0 )
    {
        modem_supervisor_add_task_retrieve_dl( dm_input.up_delay );
    }

    return dm_parse_cmd( &dm_input );
}

e_dm_error_t dm_parse_cmd( s_dm_cmd_input_t* cmd_input )
{
    e_dm_error_t ret = DM_OK;
    if( dm_check_cmd_size( cmd_input->request_code, cmd_input->buffer_len ) != DM_CMD_LENGTH_VALID )
    {
        return DM_ERROR;
    }
#if BSP_DBG_TRACE == BSP_FEATURE_ON
    BSP_DBG_TRACE_WARNING( "DM_%s (0x%02x)\n", dm_cmd_str[cmd_input->request_code], cmd_input->request_code );
#endif

    switch( cmd_input->request_code )
    {
    case DM_RESET:

        if( dm_reset( ( e_dm_reset_code_t ) cmd_input->buffer[0],
                      cmd_input->buffer[1] | ( cmd_input->buffer[2] << 8 ) ) != DM_OK )
        {
            BSP_DBG_TRACE_ERROR( "NOT RESET !!!\n" );
            ret = DM_ERROR;
            break;
        }
        break;
    case DM_FUOTA:
        // Not supported yet
        break;
    case DM_FILE_DONE:
        BSP_DBG_TRACE_WARNING( "DM_FILE_DONE donwlink\n" );
        if( modem_get_upload_state( ) != MODEM_UPLOAD_START )
        {
            BSP_DBG_TRACE_ERROR( "No FileUpload ongoing\n" );
            break;
        }

        if( modem_get_dm_upload_sctr( ) == ( cmd_input->buffer[0] & 0xf ) )  // Be careful : Session ID is not checked
        {
            set_modem_status_file_upload( false );  // to terminate the file upload
            increment_asynchronous_msgnumber( RSP_FILEDONE, 0x01 );
            modem_set_upload_state( MODEM_UPLOAD_NOT_INIT );
            modem_supervisor_remove_task( FILE_UPLOAD_TASK );
        }
        else
        {
            BSP_DBG_TRACE_ERROR( "DM_FILE_DONE bad session_counter %d compare to %d\n", cmd_input->buffer[0],
                                 modem_get_dm_upload_sctr( ) );
        }

        break;
    case DM_GET_INFO:
        if( set_dm_info( cmd_input->buffer, cmd_input->buffer_len, DM_INFO_NOW ) != SET_OK )
        {
            ret = DM_ERROR;
            break;
        }
        modem_supervisor_add_task_dm_status_now( );
        break;
    case DM_SET_CONF:
        if( dm_set_conf( ( e_dm_info_t ) cmd_input->buffer[0], cmd_input->buffer + 1, cmd_input->buffer_len - 1 ) !=
            DM_OK )
        {
            ret = DM_ERROR;
            break;
        }

        break;
    case DM_REJOIN: {
        uint16_t dev_nonce = ( cmd_input->buffer[1] << 8 ) | cmd_input->buffer[0];
        if( lorawan_api_devnonce_get( ) != dev_nonce )
        {
            ret = DM_ERROR;
            break;
        }
        set_modem_status_modem_joined( false );
        lorawan_api_join_status_clear( );
        modem_supervisor_add_task_join( );
        break;
    }
    case DM_MUTE:
        dm_set_number_of_days_mute( cmd_input->buffer[0] );
        if( get_modem_muted( ) == MODEM_TEMPORARY_MUTE )
        {
            modem_supervisor_add_task_modem_mute( );
        }
        increment_asynchronous_msgnumber( RSP_MUTE, 0 );
        break;
    case DM_SET_DM_INFO:
        if( set_dm_info( cmd_input->buffer, cmd_input->buffer_len, DM_INFO_PERIODIC ) != SET_OK )
        {
            ret = DM_ERROR;
            break;
        }

        break;
    case DM_STREAM:
        // Not supported yet
        break;
    case DM_ALC_SYNC:
        // Not supported yet
        break;
    default:
        ret = DM_ERROR;
        break;
    }

    return ret;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static e_dm_cmd_length_valid dm_check_cmd_size( e_dm_cmd_t cmd, uint8_t length )
{
    if( cmd >= DM_CMD_MAX )
    {
        BSP_DBG_TRACE_ERROR( "Invalid DM command %x\n", cmd );
        return DM_CMD_NOT_VALID;
    }
    // cmd len too small
    if( length < dm_cmd_len[cmd][0] )
    {
        BSP_DBG_TRACE_ERROR( "Invalid DM command size (too small)\n" );
        return DM_CMD_LENGTH_NOT_VALID;
    }
    // cmd len too long
    if( length > dm_cmd_len[cmd][1] )
    {
        BSP_DBG_TRACE_ERROR( "Invalid DM command size (too long)\n" );
        return DM_CMD_LENGTH_NOT_VALID;
    }
    return DM_CMD_LENGTH_VALID;
}

static e_dm_error_t dm_reset( e_dm_reset_code_t reset_code, uint16_t reset_session )
{
    e_dm_error_t ret = DM_OK;

    if( reset_session == lorawan_api_nb_reset_get( ) )
    {
        switch( reset_code )
        {
        case DM_RESET_MODEM:
        case DM_RESET_APP_MCU:
        case DM_RESET_BOTH:
            modem_store_context( );
            lorawan_api_context_save( );
            bsp_mcu_reset( );
            break;
        default:
            ret = DM_ERROR;
            BSP_DBG_TRACE_ERROR( "invalid DM reset code (0x%02x)\n", reset_code );
            break;
        }
    }
    else
    {
        ret = DM_ERROR;
        BSP_DBG_TRACE_ERROR( "invalid DM reset session code\n" );
    }

    return ret;
}
