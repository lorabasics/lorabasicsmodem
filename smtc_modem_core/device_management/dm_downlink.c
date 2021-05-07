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

#include "smtc_modem_hal.h"
#include "alc_sync.h"
#include "stream.h"
#include "dm_downlink.h"
#include "device_management_defs.h"
#include "modem_context.h"
#include "modem_supervisor.h"
#include "lorawan_api.h"

#if defined( _GNSS_SNIFF_ENABLE )
#if defined( LR1110_MODEM )
#include "gnss_ctrl_api.h"
#elif defined( LR1110_TRANSCEIVER )
#include "almanac_update.h"
#include "lr1110_gnss.h"
#else
#error "GNSS functionality can't be used !"
#endif
#endif  //_GNSS_SNIFF_ENABLE

extern smtc_modem_services_t smtc_modem_services_ctx;
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

#if MODEM_HAL_DBG_TRACE == MODEM_HAL_FEATURE_ON
static const char* dm_cmd_str[DM_CMD_MAX] = {
    [DM_RESET]       = "RESET",          //
    [DM_FUOTA]       = "FUOTA",          //
    [DM_FILE_DONE]   = "FILE_DONE",      //
    [DM_GET_INFO]    = "GET_INFO",       //
    [DM_SET_CONF]    = "SET_CONF",       //
    [DM_REJOIN]      = "REJOIN",         //
    [DM_MUTE]        = "MUTE",           //
    [DM_SET_DM_INFO] = "SET_DM_INFO",    //
    [DM_STREAM]      = "STREAM",         //
    [DM_ALC_SYNC]    = "ALC_SYNC",       //
    [DM_ALM_UPDATE]  = "ALM_UPDATE",     //
    [DM_ALM_DBG]     = "ALM_DEBUG",      //
    [DM_SOLV_UPDATE] = "SOLVER_UPDATE",  //
    [DM_ALM_FUPDATE] = "ALM_FUPDATE",    //
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
        SMTC_MODEM_HAL_TRACE_ERROR( "DM Downlink must contain at least 4 bytes\n" );
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
#if MODEM_HAL_DBG_TRACE == MODEM_HAL_FEATURE_ON
    SMTC_MODEM_HAL_TRACE_WARNING( "DM_%s (0x%02x)\n", dm_cmd_str[cmd_input->request_code], cmd_input->request_code );
#endif

    switch( cmd_input->request_code )
    {
    case DM_RESET:

        if( dm_reset( ( e_dm_reset_code_t ) cmd_input->buffer[0],
                      cmd_input->buffer[1] | ( cmd_input->buffer[2] << 8 ) ) != DM_OK )
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "NOT RESET !!!\n" );
            ret = DM_ERROR;
            break;
        }
        break;
    case DM_FUOTA:
        // Not supported yet
        break;
    case DM_FILE_DONE:
        SMTC_MODEM_HAL_TRACE_WARNING( "DM_FILE_DONE donwlink\n" );
        if( modem_get_upload_state( ) != MODEM_UPLOAD_START )
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "No FileUpload ongoing\n" );
            break;
        }

        if( modem_get_dm_upload_sctr( ) == ( cmd_input->buffer[0] & 0xf ) )  // Be careful : Session ID is not checked
        {
            set_modem_status_file_upload( false );  // to terminate the file upload
            increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_FILEDONE, 0x01 );
            modem_set_upload_state( MODEM_UPLOAD_NOT_INIT );
            modem_supervisor_remove_task( FILE_UPLOAD_TASK );
        }
        else
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "DM_FILE_DONE bad session_counter %d compare to %d\n", cmd_input->buffer[0],
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
            uint8_t       dm_fields_payload[1] = { e_inf_session };
            e_set_error_t return_code          = set_dm_info( dm_fields_payload, 1, DM_INFO_NOW );
            if( return_code == SET_OK )
            {
                modem_supervisor_add_task_dm_status_now( );
            }

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
        increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_MUTE, 0 );
        break;
    case DM_SET_DM_INFO:
        if( set_dm_info( cmd_input->buffer, cmd_input->buffer_len, DM_INFO_PERIODIC ) != SET_OK )
        {
            ret = DM_ERROR;
            break;
        }

        break;
    case DM_STREAM:
        if( stream_process_dn_frame( &( smtc_modem_services_ctx.stream_ROSE_ctx ), cmd_input->buffer,
                                     cmd_input->buffer_len ) != STREAM_OK )
        {
            ret = DM_ERROR;
        }
        break;
    case DM_ALC_SYNC: {
        bool    b_sync_before = is_alc_sync_done( &( smtc_modem_services_ctx.alc_sync_ctx ) );
        uint8_t alc_sync_status =
            alc_sync_parser( &( smtc_modem_services_ctx.alc_sync_ctx ), cmd_input->buffer, cmd_input->buffer_len );

        if( ( ( alc_sync_status >> ALC_SYNC_APP_TIME_ANS ) & 0x1 ) == 1 )
        {
            if( ( b_sync_before == false ) && is_alc_sync_done( &( smtc_modem_services_ctx.alc_sync_ctx ) ) )
            {
                increment_asynchronous_msgnumber( SMTC_MODEM_EVENT_TIME_UPDATED, 0 );
            }

            // Remove all alc sync task.
            modem_supervisor_remove_task_alc_sync( );

            modem_supervisor_add_task_alc_sync_time_req(
                alc_sync_get_interval_second( &( smtc_modem_services_ctx.alc_sync_ctx ) ) +
                smtc_modem_hal_get_signed_random_nb_in_range( -30, 30 ) );
        }

        if( alc_sync_status & ( ~( 1 << ALC_SYNC_APP_TIME_ANS ) ) )
        {
            if( ( ( alc_sync_status >> ALC_SYNC_DEVICE_APP_TIME_PERIODICITY_REQ ) & 0x1 ) == 1 )
            {
                // If periodic time request is configured, add task to handle it
                modem_supervisor_add_task_alc_sync_time_req(
                    alc_sync_get_interval_second( &( smtc_modem_services_ctx.alc_sync_ctx ) ) +
                    smtc_modem_hal_get_signed_random_nb_in_range( -30, 30 ) );
            }
            // When a request requiring an answer is requested, add task to handle it
            modem_supervisor_add_task_alc_sync_ans( 1 );
        }
    }
    break;
#if defined( _GNSS_SNIFF_ENABLE )
    case DM_ALM_UPDATE: {
#if defined( LR1110_MODEM )
        if( Gnss_push_almanac_update( cmd_input->buffer, cmd_input->buffer_len, 0 ) != GNSS_CMD_OK )
#elif defined( LR1110_TRANSCEIVER )
        // format received buffer to fit das raw input (with 2 more bytes upcount updelay and the DM_ALM_UPDATE
        // opcode)
        uint8_t                      buff[cmd_input->buffer_len + 3];
        almanac_update_return_code_t rc = ALMANAC_ERROR;

        buff[2] = DM_ALM_UPDATE;
        memcpy( &buff[3], cmd_input->buffer, cmd_input->buffer_len );
        // call radio_suspend to secure the direct radio access
        // modem_context_suspend_radio_access( );
        rc = almanac_update_process_downlink_payload( NULL, buff, cmd_input->buffer_len + 3 );
        // modem_context_resume_radio_access( );
        if( rc != ALMANAC_OK )
#else
#error "GNSS functionality can't be used !"
#endif
        {
            ret = DM_ERROR;
        }
        break;
    }
    case DM_ALM_FUPDATE: {
#if defined( LR1110_MODEM )
        if( Gnss_push_almanac_update( cmd_input->buffer, cmd_input->buffer_len, 1 ) != GNSS_CMD_OK )
#elif defined( LR1110_TRANSCEIVER )
        // format received buffer to fit das raw input (with 2 more bytes upcount updelay and the DM_ALM_UPDATE
        // opcode)
        uint8_t                      buff[cmd_input->buffer_len + 3];
        almanac_update_return_code_t rc = ALMANAC_ERROR;

        buff[2] = DM_ALM_FUPDATE;
        memcpy( &buff[3], cmd_input->buffer, cmd_input->buffer_len );
        // call radio_suspend to secure the direct radio access
        // modem_context_suspend_radio_access( );
        rc = almanac_update_process_downlink_payload( NULL, buff, cmd_input->buffer_len + 3 );
        // modem_context_resume_radio_access( );
        if( rc != ALMANAC_OK )
#else
#error "GNSS functionality can't be used !"
#endif
        {
            ret = DM_ERROR;
        }
        break;
    }
    case DM_ALM_DBG: {
#if defined( LR1110_MODEM )
        // Is DM set time request (not used in soft modem)
        uint32_t gps_time_s = is_set_time_request( cmd_input->buffer, cmd_input->buffer_len );
        if( gps_time_s )
        {
            modem_set_time( gps_time_s );
        }
        if( Gnss_push_dbg_request( cmd_input->buffer, cmd_input->buffer_len ) != GNSS_CMD_OK )
        {
            ret = DM_ERROR;
        }
        else
        {
            SMTC_MODEM_HAL_TRACE_PRINTF( "modem_supervisor_add_task_alm_dbg_ans\n" );

            // Prepare the task to send answers
            modem_supervisor_add_task_alm_dbg_ans( 1 );
        }
#elif defined( LR1110_TRANSCEIVER )
        // Not supported on Basic Modem with Lr1110 tranceiver
        ret                     = DM_ERROR;
#else
#error "GNSS functionality can't be used !"
#endif
    }
    break;
    case DM_SOLV_UPDATE: {
#if defined( LR1110_MODEM )
        if( Gnss_push_msg_from_solver( cmd_input->buffer, cmd_input->buffer_len ) != GNSS_CMD_OK )
#elif defined( LR1110_TRANSCEIVER )
        void*           context = 0;
        lr1110_status_t rc      = LR1110_STATUS_ERROR;
        // call radio_suspend to secure the direct radio access
        // modem_context_suspend_radio_access( );
        rc = lr1110_gnss_push_solver_msg( context, cmd_input->buffer, cmd_input->buffer_len );
        // modem_context_resume_radio_access( );
        if( rc != LR1110_STATUS_OK )
#else
#error "GNSS functionality can't be used !"
#endif
        {
            ret = DM_ERROR;
        }
        break;
    }
#endif  // _GNSS_SNIFF_ENABLE
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
        SMTC_MODEM_HAL_TRACE_ERROR( "Invalid DM command %x\n", cmd );
        return DM_CMD_NOT_VALID;
    }
    // cmd len too small
    if( length < dm_cmd_len[cmd][0] )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Invalid DM command size (too small)\n" );
        return DM_CMD_LENGTH_NOT_VALID;
    }
    // cmd len too long
    if( length > dm_cmd_len[cmd][1] )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Invalid DM command size (too long)\n" );
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
            lorawan_api_context_save( );
            smtc_modem_hal_reset_mcu( );
            break;
        default:
            ret = DM_ERROR;
            SMTC_MODEM_HAL_TRACE_ERROR( "invalid DM reset code (0x%02x)\n", reset_code );
            break;
        }
    }
    else
    {
        uint8_t       dm_fields_payload[1] = { e_inf_rstcount };
        e_set_error_t return_code          = set_dm_info( dm_fields_payload, 1, DM_INFO_NOW );
        if( return_code == SET_OK )
        {
            modem_supervisor_add_task_dm_status_now( );
        }
        ret = DM_ERROR;
        SMTC_MODEM_HAL_TRACE_ERROR( "invalid DM reset session code\n" );
    }

    return ret;
}